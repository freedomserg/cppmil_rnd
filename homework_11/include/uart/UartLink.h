#pragma once

#include <drone_link.h>

#include <cstdint>
#include <functional>
#include <utility>

// Колбеки на розпаковані пакети (all-public агрегат, тримається в UartLink приватно).
struct UartHandlers {
    std::function<void(const dlink::Telemetry&)> onTelemetry;
    std::function<void(const dlink::TargetPos&)> onTarget;
    std::function<void(const dlink::AmmoCfg&)>   onAmmo;
    std::function<void(const dlink::DroneCfg&)>  onConfig;
    std::function<void(const dlink::Result&)>    onResult;
};

// UART-лінк: termios-порт + інкрементальний парсер кадрів. poll() читає доступні
// байти, збирає кадри й роздає їх колбекам; sendControl() шле PKT_CONTROL.
class UartLink {
public:
    UartLink() = default;
    UartLink(const UartLink&) = delete;
    auto operator=(const UartLink&) -> UartLink& = delete;
    ~UartLink();

    [[nodiscard]] auto open(const char* dev) -> bool;
    void close();
    void setHandlers(UartHandlers handlers) { handlers_ = std::move(handlers); }

    auto poll() -> int;                                   // байтів прочитано (0 = порожньо, <0 = помилка)
    auto sendControl(float accel, float turnRate) -> bool;
    [[nodiscard]] auto fd() const -> int { return fd_; }

private:
    int fd_ = -1;
    dlink::Parser parser_;
    UartHandlers  handlers_;
};
