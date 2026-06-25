#pragma once

struct DroneConfig;
struct AmmoParams;

class IConfigLoader {
public:
    virtual bool        load()          = 0;
    virtual DroneConfig getConfig()     const = 0;
    virtual AmmoParams  getAmmoParams() const = 0;
    virtual ~IConfigLoader() {}
};