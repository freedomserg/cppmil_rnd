#pragma once

#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
#include "states/IDroneState.h"
#include "states/DroneContext.h"
#include "Types.h"
#include <string>
#include <vector>
#include <memory>

class MissionProcessor {

    private:
        std::unique_ptr<ITargetProvider> targets_;
        std::unique_ptr<IBallisticSolver> solver_;
        std::unique_ptr<IConfigLoader>    loader_;

        DroneConfig cfg_{};
        AmmoParams  ammo_{};

        DroneContext ctx_{cfg_};                  // кінематика + планування, прив'язана до cfg_
        std::unique_ptr<IDroneState> state_;      // поточний стан (патерн State)

        float currentTime_      = 0.0f;
        int   currentTargetIdx_ = -1;

        std::vector<SimStep> steps_;

        bool initialized_ = false;
        bool done_        = false;

    public:
        MissionProcessor(std::unique_ptr<ITargetProvider> t, std::unique_ptr<IBallisticSolver> s, std::unique_ptr<IConfigLoader> l);

        bool init();

        bool hasNext() const;

        SimStep step();

        void reset();

        void changeSolver(std::unique_ptr<IBallisticSolver> s);

        bool writeOutput(const std::string& filename) const;

        int  getRecordedSteps() const;

        bool isDone()           const;
};