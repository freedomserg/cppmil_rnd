#pragma once

#include "gpio/IGpio.h"

#include <string>

// Бекенд на сирому kernel GPIO character-device uAPI v2 (<linux/gpio.h>).
// Без зовнішніх залежностей; працює і в gpio-sim, і на реальному чипі.
class RawUapiGpio : public IGpio {
public:
    RawUapiGpio(const std::string& chipName, unsigned startLine, unsigned dropLine);
    RawUapiGpio(const RawUapiGpio&) = delete;
    auto operator=(const RawUapiGpio&) -> RawUapiGpio& = delete;
    ~RawUapiGpio() override;

    void setStart(bool on) override;
    void pulseDrop() override;

    [[nodiscard]] auto ok() const -> bool { return lineFd_ >= 0; }

private:
    void setValue(unsigned idx, bool value);

    int chipFd_ = -1;
    int lineFd_ = -1;
};
