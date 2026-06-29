#pragma once
#include "IDroneState.h"

class StateDecelerating : public IDroneState {
public:
    auto execute(DroneContext& ctx) -> std::unique_ptr<IDroneState> override;
    [[nodiscard]] auto name() const -> const std::string& override;
    [[nodiscard]] auto stateId() const -> int override;

private:
    static inline const std::string name_{"Decelerating"};
    static constexpr int id_ = 2;
};
