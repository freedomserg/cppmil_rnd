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

        DroneConfig loadDroneConfig(const std::string& filename, bool& outLoaded);
        std::vector<AmmoParams> loadAmmoParamsArray(const std::string& filename);

    public:
        FileConfigLoader(const std::string& configFile, const std::string& ammoFile);

        bool load() override;

        DroneConfig getConfig()     const override;
        AmmoParams  getAmmoParams() const override;
};