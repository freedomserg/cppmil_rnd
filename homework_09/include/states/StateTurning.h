#pragma once
#include "IDroneState.h"

class StateTurning : public IDroneState {
    public:
        std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
        const std::string& name() const override;
        int stateId() const override;

    private:
        static inline const std::string name_{"Turning"};
        static constexpr int id_ = 3;
};