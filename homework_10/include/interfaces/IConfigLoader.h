#pragma once

struct DroneConfig;
struct AmmoParams;

class IConfigLoader {
public:
    [[nodiscard]] virtual auto load() -> bool = 0;
    [[nodiscard]] virtual auto getConfig() const -> DroneConfig = 0;
    [[nodiscard]] virtual auto getAmmoParams() const -> AmmoParams = 0;
    virtual ~IConfigLoader() = default;
};
