#pragma once

#include <memory>
#include <string>

struct DroneContext;

class IDroneState {
public:
    virtual ~IDroneState() = default;

    // Заповнює ctx.command і повертає наступний стан (nullptr — лишитись).
    virtual auto execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> = 0;

    [[nodiscard]] virtual auto name() const -> const std::string& = 0;
    [[nodiscard]] virtual auto stateId() const -> int = 0;
};
