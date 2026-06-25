#pragma once

struct Coord;
struct Target;
struct AmmoParams;
struct DroneConfig;

class IBallisticSolver {
public:
    // Computes fire point and total flight time for the given drone state and target track
    // alreadyApproaching: drone is already in route to this target — skip intermediate maneuver
    // outPredictedTarget: where the target is expected to be at drop time
    // outAimPoint: current drone aim point if to drop right now
    virtual bool solve(
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
        Coord& outAimPoint) = 0;

    virtual ~IBallisticSolver() {}
};