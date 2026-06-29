#pragma once

struct Coord;
struct Target;
struct AmmoParams;
struct DroneConfig;

class IBallisticSolver {
public:
    [[nodiscard]] virtual auto solve(
        const Coord&       dronePos,
        float              droneDir,
        float              droneSpeed,
        const Target&      target,
        float              currentTime,
        bool               alreadyApproaching,
        const DroneConfig& cfg,
        const AmmoParams&  ammo,
        Coord& outFirePos,
        Coord& outIntermediatePos,
        bool&  outHasIntermediate,
        float& outTotalTime,
        Coord& outPredictedTarget,
        Coord& outAimPoint) -> bool = 0;

    virtual ~IBallisticSolver() = default;
};
