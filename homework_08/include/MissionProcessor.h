#pragma once

#include "interfaces/IBallisticSolver.h"
#include "interfaces/ITargetProvider.h"
#include "interfaces/IConfigLoader.h"
#include "Types.h"
#include <string>
#include <vector>

class MissionProcessor {

    private:
        ITargetProvider*  targets_;
        IBallisticSolver* solver_;
        IConfigLoader*    loader_;

        DroneConfig cfg_{};
        AmmoParams  ammo_{};

        DroneState droneState_       = DroneState::STOPPED;
        Coord      currentPos_       = {};
        float      currentDir_       = 0.0f;
        float      currentSpeed_     = 0.0f;
        float      currentTime_      = 0.0f;
        int        currentTargetIdx_ = -1;
        float      acceleration_     = 0.0f;

        std::vector<SimStep> steps_;

        bool initialized_ = false;
        bool done_        = false;

    public:
        MissionProcessor(ITargetProvider* t, IBallisticSolver* s, IConfigLoader* l);

        bool init();

        bool hasNext() const;

        SimStep step();

        void reset();

        void changeSolver(IBallisticSolver* s);

        bool writeOutput(const std::string& filename) const;

        int  getRecordedSteps() const;

        bool isDone()           const;
};