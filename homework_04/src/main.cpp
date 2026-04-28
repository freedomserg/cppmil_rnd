#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

// given parameters
constexpr int ticksPerRevolution = 1024;
constexpr float wheelRadiusM = 0.3f;
constexpr float wheelbaseM = 1.0f;

struct OdotemtrySample {
    long timestampMs;
    int flTicks;
    int frTicks;
    int blTicks;
    int brTicks;
};

OdotemtrySample* loadOdometrySamplesFromFile(const char* filename, int& outSamplesCount) {
    std::fstream odoFile(filename, std::ios::in);
    if (!odoFile) {
        std::cerr << "Error opening file with odometry samples: " << filename << std::endl;
        outSamplesCount = 0;
        return nullptr;
    }

    int count = 0;
    std::string line;
    while(std::getline(odoFile, line)) {
        count++;
    }

    if (count == 0) {
        std::cerr << "No samples found in the file: " << filename << std::endl;
        outSamplesCount = 0;
        return nullptr;
    }

    OdotemtrySample* samples = new OdotemtrySample[count];
    odoFile.clear();
    odoFile.seekg(0, std::ios::beg);

    for (int i = 0; i < count; ++i) {
       if (!(odoFile >> samples[i].timestampMs >> samples[i].flTicks >> samples[i].frTicks >> samples[i].blTicks >> samples[i].brTicks)) {
           std::cerr << "Error reading sample at line " << (i + 1) << " in file: " << filename << std::endl;
           delete[] samples;
           outSamplesCount = 0;
           return nullptr;
       }
    }

    outSamplesCount = count;
    return samples;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }

    const char* filename = argv[1];
    // wheel_circumference = 2π × wheel_radius
    const float wheelCircumference = 2.0f * M_PI * wheelRadiusM;

    int samplesCount = 0;
    OdotemtrySample* samples = loadOdometrySamplesFromFile(filename, samplesCount);
    if (!samples) {
        return 1;
    }

    float prevX = 0.0f, prevY = 0.0f, prevTheta = 0.0f;
    for (int i = 1; i < samplesCount; ++i) {
        // 1 - Delta ticks (encoders change between two samples)
        // Δfl = fl_ticks[t] - fl_ticks[t-1]
        // Δfr = fr_ticks[t] - fr_ticks[t-1]
        // Δbl = bl_ticks[t] - bl_ticks[t-1]
        // Δbr = br_ticks[t] - br_ticks[t-1]

        float deltaFl = samples[i].flTicks - samples[i - 1].flTicks;
        float deltaFr = samples[i].frTicks - samples[i - 1].frTicks;
        float deltaBl = samples[i].blTicks - samples[i - 1].blTicks;
        float deltaBr = samples[i].brTicks - samples[i - 1].brTicks;

        // Δleft  = (Δfl + Δbl) / 2
        // Δright = (Δfr + Δbr) / 2
        float deltaLeft = (deltaFl + deltaBl) / 2.0f;
        float deltaRight = (deltaFr + deltaBr) / 2.0f;

        // 2 - Ticks → meters
        // dist_left  = (Δleft  / ticks_per_rev) × wheel_circumference
        // dist_right = (Δright / ticks_per_rev) × wheel_circumference
        float distLeft = (deltaLeft / ticksPerRevolution) * wheelCircumference;
        float distRight = (deltaRight / ticksPerRevolution) * wheelCircumference;

        // 3 - Linear and angular displacement
        // dist_center = (dist_left + dist_right) / 2      <- how much the robot's center has moved
        // delta_theta = (dist_right - dist_left) / wheelbase <- how much the robot has rotated

        float distCenter = (distLeft + distRight) / 2.0f;
        float distTheta = (distRight - distLeft) / wheelbaseM;

        // 4 - Update pose
        // x     = x     + dist_center × cos(theta + delta_theta / 2)
        // y     = y     + dist_center × sin(theta + delta_theta / 2)
        // theta = theta + delta_theta

        float x = prevX + distCenter * cos(prevTheta + distTheta / 2.0f);
        float y = prevY + distCenter * sin(prevTheta + distTheta / 2.0f);
        float theta = prevTheta + distTheta;

        // 5 - Log the result to the console in the format: timestamp x y theta
        std::cout << samples[i].timestampMs << " " << x << " " << y << " " << theta << std::endl;

        // 6 - Prepare for the next iteration
        prevX = x;
        prevY = y;
        prevTheta = theta;
    }

    delete[] samples; // clean up allocated memory
    samples = nullptr;

    return 0;
}
