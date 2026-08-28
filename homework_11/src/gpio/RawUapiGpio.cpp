#include "gpio/RawUapiGpio.h"

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>

namespace {
constexpr unsigned    kStartIdx    = 0;
constexpr unsigned    kDropIdx     = 1;
constexpr useconds_t  kDropPulseUs = 80000;   // 80 мс ∈ [50,100]
}

RawUapiGpio::RawUapiGpio(const std::string& chipName, unsigned startLine, unsigned dropLine) {
    const std::string path = "/dev/" + chipName;
    chipFd_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (chipFd_ < 0) return;

    gpio_v2_line_request req{};
    req.offsets[kStartIdx] = startLine;
    req.offsets[kDropIdx]  = dropLine;
    req.num_lines          = 2;
    req.config.flags       = GPIO_V2_LINE_FLAG_OUTPUT;
    std::strncpy(req.consumer, "drone", sizeof(req.consumer) - 1);

    if (::ioctl(chipFd_, GPIO_V2_GET_LINE, &req) < 0 || req.fd < 0) return;
    lineFd_ = req.fd;
}

RawUapiGpio::~RawUapiGpio() {
    if (lineFd_ >= 0) ::close(lineFd_);
    if (chipFd_ >= 0) ::close(chipFd_);
}

void RawUapiGpio::setValue(unsigned idx, bool value) {
    if (lineFd_ < 0) return;
    gpio_v2_line_values v{};
    v.mask = static_cast<uint64_t>(1) << idx;
    v.bits = static_cast<uint64_t>(value ? 1 : 0) << idx;
    ::ioctl(lineFd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &v);
}

void RawUapiGpio::setStart(bool on) { setValue(kStartIdx, on); }

void RawUapiGpio::pulseDrop() {
    setValue(kDropIdx, true);
    ::usleep(kDropPulseUs);
    setValue(kDropIdx, false);
}
