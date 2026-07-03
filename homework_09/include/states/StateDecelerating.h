#pragma once
#include "IDroneState.h"

class StateDecelerating : public IDroneState {
    
    public: 
        std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
        const std::string& name() const override;
        int stateId() const override;

    private:
        static inline const std::string name_{"Decelerating"};
        static constexpr int id_ = 2;
};