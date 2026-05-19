#include "telemetry.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <stdexcept>

// Debugging exercise notes:
// this file intentionally contains four runtime defects.
// The defects are related to malformed input shape, invalid numeric values,
// unsafe time deltas, and empty logs. Exact locations are not marked on purpose.

constexpr int EXPECTED_FIELD_COUNT = 7;
constexpr int MAX_LINE_LENGTH = 256;

int split_line(char line[], char* fields[], int max_fields) {
    int count = 0;
    char* cursor = line;

    while (*cursor != '\0' && count < max_fields) {
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
            *cursor = '\0';
            ++cursor;
        }

        if (*cursor == '\0') {
            break;
        }

        fields[count] = cursor;
        ++count;

        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' &&
               *cursor != '\r') {
            ++cursor;
        }
    }

    return count;
}

long parse_long(const char* text, int line_number) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);

    if (end == text) {
        throw std::invalid_argument("failed to parse long value at line " + std::to_string(line_number+1) + ": " + std::string(text));
    }

    return value;
}

int parse_int(const char* text, int line_number) {
    return static_cast<int>(parse_long(text, line_number));
}

double parse_double(const char* text, int line_number) {
    char* end = nullptr;
    const double value = std::strtod(text, &end);

    if (end == text) {
        throw std::invalid_argument("failed to parse double value at line " + std::to_string(line_number+1) + ": " + std::string(text));
    }

    return value;
}

Frame parse_frame(char line[], int line_number) {
    char* fields[EXPECTED_FIELD_COUNT] = {};
    const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);

    if (field_count != EXPECTED_FIELD_COUNT) {
        throw std::invalid_argument("invalid frame at line " + std::to_string(line_number+1) + ": expected " + std::to_string(EXPECTED_FIELD_COUNT) + " fields, got " + std::to_string(field_count));
    }

    Frame frame{};
    frame.timestamp_ms = parse_long(fields[0], line_number);
    frame.seq = parse_int(fields[1], line_number);
    frame.voltage_v = parse_double(fields[2], line_number);
    frame.current_a = parse_double(fields[3], line_number);
    frame.temperature_c = parse_double(fields[4], line_number);
    frame.gps_fix = parse_int(fields[5], line_number);
    frame.satellites = parse_int(fields[6], line_number);
    return frame;
}

double compute_frame_rate_hz(const Frame frames[], int frame_count) {
    const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;

    return static_cast<double>((frame_count - 1) * 1000 / elapsed_ms);
}

int read_frames(const char* path, Frame frames[], int max_frames) {
    std::ifstream input{path};
    if (!input) {
        std::cerr << "error: failed to open input file: " << path << '\n';
        return 0;
    }

    int frame_count = 0;
    char line[MAX_LINE_LENGTH];

    while (input.getline(line, MAX_LINE_LENGTH)) {
        if (line[0] == '\0') {
            continue;
        }

        if (frame_count < max_frames) {
            frames[frame_count] = parse_frame(line, frame_count);
            ++frame_count;
        }
    }

    return frame_count;
}

void validate_frames(const Frame frames[], int frame_count) {
    if (frame_count == 0) {
        throw std::invalid_argument("empty telemetry log");
    }

    long prev_timestamp_ms = -1;
    int prev_seq = 0;
    for (int i = 0; i < frame_count; ++i) {
        if (frames[i].timestamp_ms <= prev_timestamp_ms) {
            throw std::invalid_argument("non-increasing timestamp at line " + std::to_string(i+1) + ": " + std::to_string(frames[i].timestamp_ms) + " <= " + std::to_string(prev_timestamp_ms));
        }
        prev_timestamp_ms = frames[i].timestamp_ms;

        if (frames[i].seq != prev_seq + 1) {
            throw std::invalid_argument("non-sequential frame number at line " + std::to_string(i+1) + ": " + std::to_string(frames[i].seq) + " != " + std::to_string(prev_seq + 1));
        }
        prev_seq = frames[i].seq;

        if (frames[i].voltage_v <= 0.0) {
            throw std::invalid_argument("non-positive voltage at line " + std::to_string(i+1) + ": " + std::to_string(frames[i].voltage_v));
        }

        if (frames[i].current_a < 0.0) {
            throw std::invalid_argument("negative current at line " + std::to_string(i+1) + ": " + std::to_string(frames[i].current_a));
        }

        if (frames[i].temperature_c < -40.0 || frames[i].temperature_c > 120.0) {
            throw std::invalid_argument("invalid temperature at line " + std::to_string(i+1) + ": " + std::to_string(frames[i].temperature_c));
        }

        if (frames[i].gps_fix != 0 && frames[i].gps_fix != 1) {
            throw std::invalid_argument("invalid GPS fix at line " + std::to_string(i+1) + ": " + std::to_string(frames[i].gps_fix));
        }

        if (frames[i].satellites < 0) {
            throw std::invalid_argument("negative satellite count at line " + std::to_string(i+1) + ": " + std::to_string(frames[i].satellites));
        }        
    }
}

Summary summarize(const Frame frames[], int frame_count) {
    Summary summary{};
    summary.frames_total = frame_count;
    summary.frames_valid = frame_count;
    summary.voltage_min = frames[0].voltage_v;
    summary.voltage_max = frames[0].voltage_v;
    summary.low_voltage_frames = 0;

    double temperature_sum = 0.0;

    for (int i = 0; i < frame_count; ++i) {
        if (frames[i].voltage_v < summary.voltage_min) {
            summary.voltage_min = frames[i].voltage_v;
        }

        if (frames[i].voltage_v > summary.voltage_max) {
            summary.voltage_max = frames[i].voltage_v;
        }

        temperature_sum += frames[i].temperature_c;

        if (frames[i].voltage_v < 22.0) {
            ++summary.low_voltage_frames;
        }
    }

    const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
    summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
    summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
    return summary;
}

void print_summary(const Summary& summary) {
    std::cout << "frames_total " << summary.frames_total << '\n';
    std::cout << "frames_valid " << summary.frames_valid << '\n';
    std::cout << "voltage_min " << summary.voltage_min << '\n';
    std::cout << "voltage_max " << summary.voltage_max << '\n';
    std::cout << "temperature_avg " << summary.temperature_avg << '\n';
    std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
    std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
