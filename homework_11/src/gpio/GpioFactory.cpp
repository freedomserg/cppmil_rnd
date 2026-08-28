#include "gpio/GpioFactory.h"

#ifdef USE_GPIOD
#include "gpio/LibgpiodGpio.h"
#else
#include "gpio/RawUapiGpio.h"
#endif

auto makeGpio(const std::string& chipName, unsigned startLine, unsigned dropLine)
    -> std::unique_ptr<IGpio> {
#ifdef USE_GPIOD
    return std::make_unique<LibgpiodGpio>(chipName, startLine, dropLine);
#else
    return std::make_unique<RawUapiGpio>(chipName, startLine, dropLine);
#endif
}
