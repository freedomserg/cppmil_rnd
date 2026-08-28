#pragma once

#include "gpio/IGpio.h"

#include <gpiod.h>

#include <string>

// Бекенд на libgpiod v1 (Pi OS Bookworm). Вмикається -DUSE_GPIOD=ON.
class LibgpiodGpio : public IGpio {
public:
    LibgpiodGpio(const std::string& chipName, unsigned startLine, unsigned dropLine);
    LibgpiodGpio(const LibgpiodGpio&) = delete;
    auto operator=(const LibgpiodGpio&) -> LibgpiodGpio& = delete;
    ~LibgpiodGpio() override;

    void setStart(bool on) override;
    void pulseDrop() override;

    [[nodiscard]] auto ok() const -> bool {
        return chip_ != nullptr && start_ != nullptr && drop_ != nullptr;
    }

private:
    gpiod_chip* chip_  = nullptr;
    gpiod_line* start_ = nullptr;
    gpiod_line* drop_  = nullptr;
};
