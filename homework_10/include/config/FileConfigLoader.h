#pragma once

#include "interfaces/IConfigLoader.h"
#include "Types.h"
#include <string>
#include <vector>

class FileConfigLoader : public IConfigLoader {
private:
    std::string configFile_;
    std::string ammoFile_;
    DroneConfig cfg_{};
    AmmoParams  ammo_{};

    auto loadDroneConfig(const std::string& filename, bool& outLoaded) -> DroneConfig;
    auto loadAmmoParamsArray(const std::string& filename) -> std::vector<AmmoParams>;

public:
    FileConfigLoader(std::string configFile, std::string ammoFile);

    auto load() -> bool override;

    [[nodiscard]] auto getConfig() const -> DroneConfig override;
    [[nodiscard]] auto getAmmoParams() const -> AmmoParams override;
};
