#include "uart/UartLink.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>

auto UartLink::open(const char* dev) -> bool {
    fd_ = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);                      // 8N1, без обробки символів
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &tio);
    return true;
}

void UartLink::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

UartLink::~UartLink() { close(); }

auto UartLink::poll() -> int {
    uint8_t buf[256];
    const ssize_t n = ::read(fd_, buf, sizeof buf);
    if (n <= 0) return static_cast<int>(n);

    uint8_t type = 0;
    uint8_t len  = 0;
    uint8_t payload[260];
    for (ssize_t i = 0; i < n; ++i) {
        if (!parser_.feed(buf[i], type, payload, len)) continue;
        switch (type) {
        case dlink::PKT_TELEMETRY: {
            dlink::Telemetry t{};
            std::memcpy(&t, payload, sizeof t);
            if (handlers_.onTelemetry) handlers_.onTelemetry(t);
            break;
        }
        case dlink::PKT_TARGET: {
            dlink::TargetPos tp{};
            std::memcpy(&tp, payload, sizeof tp);
            if (handlers_.onTarget) handlers_.onTarget(tp);
            break;
        }
        case dlink::PKT_AMMO: {
            dlink::AmmoCfg a{};
            std::memcpy(&a, payload, sizeof a);
            if (handlers_.onAmmo) handlers_.onAmmo(a);
            break;
        }
        case dlink::PKT_CONFIG: {
            dlink::DroneCfg c{};
            std::memcpy(&c, payload, sizeof c);
            if (handlers_.onConfig) handlers_.onConfig(c);
            break;
        }
        case dlink::PKT_RESULT: {
            dlink::Result r{};
            std::memcpy(&r, payload, sizeof r);
            if (handlers_.onResult) handlers_.onResult(r);
            break;
        }
        default: break;
        }
    }
    return static_cast<int>(n);
}

auto UartLink::sendControl(float accel, float turnRate) -> bool {
    dlink::Control c{accel, turnRate};
    uint8_t out[64];
    const size_t m = dlink::encode(dlink::PKT_CONTROL, &c, sizeof c, out);
    return ::write(fd_, out, m) == static_cast<ssize_t>(m);
}
