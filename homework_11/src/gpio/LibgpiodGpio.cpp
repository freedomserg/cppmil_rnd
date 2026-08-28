#include "gpio/LibgpiodGpio.h"

#include <unistd.h>

namespace {
constexpr useconds_t kDropPulseUs = 80000;   // 80 мс ∈ [50,100]
}

LibgpiodGpio::LibgpiodGpio(const std::string& chipName, unsigned startLine, unsigned dropLine) {
    chip_ = gpiod_chip_open_lookup(chipName.c_str());
    if (chip_ == nullptr) return;
    start_ = gpiod_chip_get_line(chip_, startLine);
    drop_  = gpiod_chip_get_line(chip_, dropLine);
    if (start_ != nullptr) gpiod_line_request_output(start_, "drone", 0);
    if (drop_  != nullptr) gpiod_line_request_output(drop_,  "drone", 0);
}

LibgpiodGpio::~LibgpiodGpio() {
    if (start_ != nullptr) gpiod_line_release(start_);
    if (drop_  != nullptr) gpiod_line_release(drop_);
    if (chip_  != nullptr) gpiod_chip_close(chip_);
}

void LibgpiodGpio::setStart(bool on) {
    if (start_ != nullptr) gpiod_line_set_value(start_, on ? 1 : 0);
}

void LibgpiodGpio::pulseDrop() {
    if (drop_ == nullptr) return;
    gpiod_line_set_value(drop_, 1);
    ::usleep(kDropPulseUs);
    gpiod_line_set_value(drop_, 0);
}
