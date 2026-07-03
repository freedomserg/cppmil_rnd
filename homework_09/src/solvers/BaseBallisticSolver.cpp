#include "solvers/BaseBallisticSolver.h"
#include "Types.h"
#include "Logger.h"
#include <cmath>

constexpr int   NUM_APPROX_STEPS      = 3;
constexpr float TIME_CHANGE_THRESHOLD = 0.05f;

// ── Геометрія місії ─────────────────────────────────────────────────────────

void BaseBallisticSolver::getIntermediateAndDropPoint(
    const Coord& dronePos, const Coord& targetPos,
    float hDist, float accelPath,
    Coord& outIntermediatePos, Coord& outFirePos, bool& outHasIntermediate)
{
    Coord local = dronePos;
    float dist  = calcDistance(local, targetPos);

    outHasIntermediate = (hDist + accelPath > dist);
    if (outHasIntermediate) {
        if (dist < 1e-6f)
            local = { targetPos.x - (hDist + accelPath), targetPos.y };
        else
            local = targetPos - (targetPos - local) * ((hDist + accelPath) / dist);
        dist = calcDistance(local, targetPos);
    }
    outIntermediatePos = local;
    outFirePos         = targetPos - (targetPos - local).normalize() * hDist;
}

Coord BaseBallisticSolver::interpolate(const Target& target, float t, float arrayTimeStep) {
    int          raw  = static_cast<int>(std::floor(t / arrayTimeStep));
    int          n    = target.timeStepsCount;
    int          idx  = raw % n;
    int          next = (idx + 1) % n;
    float        frac = (t - raw * arrayTimeStep) / arrayTimeStep;
    const Coord* pos  = target.positions;
    return { pos[idx].x + (pos[next].x - pos[idx].x) * frac,
             pos[idx].y + (pos[next].y - pos[idx].y) * frac };
}

Coord BaseBallisticSolver::extrapolate(
    const Target& target, float currentTime, float dt, float arrayTimeStep)
{
    int          n    = target.timeStepsCount;
    int          idx  = static_cast<int>(std::floor(currentTime / arrayTimeStep)) % n;
    int          next = (idx + 1) % n;
    const Coord* pos  = target.positions;
    float        vx   = (pos[next].x - pos[idx].x) / arrayTimeStep;
    float        vy   = (pos[next].y - pos[idx].y) / arrayTimeStep;
    return interpolate(target, currentTime, arrayTimeStep) + Coord{ vx, vy } * dt;
}

bool BaseBallisticSolver::needStop(float desiredDir, float currentDir, float angleThreshold) {
    return std::fabs(normalizeAngle(desiredDir - currentDir)) > angleThreshold;
}

// Estimates drone flight time to targetPos, accounting for deceleration,
// optional stop-and-turn, re-acceleration, and cruise at maxSpeed.
float BaseBallisticSolver::calcTimeOfFlight(
    Coord currentPos, const Coord& targetPos,
    float currentDir, float angleThreshold, float angularSpeed,
    float currentSpeed, float maxSpeed, float accelPath)
{
    float totalTime  = 0.0f;
    float desiredDir = std::atan2(targetPos.y - currentPos.y, targetPos.x - currentPos.x);
    float a          = calcAccel(maxSpeed, accelPath);

    if (needStop(desiredDir, currentDir, angleThreshold)) {
        float pathToStop  = accelPathFromSpeed(currentSpeed, a);
        currentPos.x     += std::cos(currentDir) * pathToStop;
        currentPos.y     += std::sin(currentDir) * pathToStop;
        totalTime        += accelTimeFromDist(pathToStop, a);
        totalTime        += std::fabs(normalizeAngle(desiredDir - currentDir)) / angularSpeed;
        currentSpeed      = 0.0f;
        currentDir        = desiredDir;
    }

    float dist         = calcDistance(currentPos, targetPos);
    float decelPath    = std::min(accelPathFromSpeed(maxSpeed, a), dist);
    totalTime         += accelTimeFromDist(decelPath, a);
    dist              -= decelPath;

    float accelDist    = std::max(0.0f,
        std::min(accelPathFromSpeed(maxSpeed, a) - accelPathFromSpeed(currentSpeed, a), dist));
    totalTime         += accelTimeFromDist(accelDist, a);
    dist              -= accelDist;

    totalTime += dist / maxSpeed;
    return totalTime;
}

// ── IBallisticSolver ────────────────────────────────────────────────────────

bool BaseBallisticSolver::solve(
    const Coord&       dronePos,
    float              droneDir,
    float              droneSpeed,
    const Target&      target,
    float              currentTime,
    bool               alreadyApproaching,
    const DroneConfig& cfg,
    const AmmoParams&  /*ammo*/,
    Coord& outFirePos,
    Coord& outIntermediatePos,
    bool&  outHasIntermediate,
    float& outTotalTime,
    Coord& outPredictedTarget,
    Coord& outAimPoint)
{
    if (!ready_) return false;

    // Iterative refinement: repeat NUM_APPROX_STEPS times or until convergence.
    float totalTime = 0.0f;
    for (int iter = 0; iter < NUM_APPROX_STEPS; ++iter) {
        float prevTotal    = totalTime;
        outPredictedTarget = extrapolate(target, currentTime,
                                         prevTotal + ammoFlightTime_,
                                         cfg.arrayTimeStep);

        getIntermediateAndDropPoint(dronePos, outPredictedTarget, hDist_, cfg.accelPath,
            outIntermediatePos, outFirePos, outHasIntermediate);

        // Drone already in route to this target — skip the intermediate waypoint.
        if (outHasIntermediate && alreadyApproaching)
            outHasIntermediate = false;

        if (outHasIntermediate) {
            totalTime = calcTimeOfFlight(dronePos, outIntermediatePos,
                droneDir, cfg.turnThreshold, cfg.angularSpeed,
                droneSpeed, cfg.attackSpeed, cfg.accelPath);
            float dirToFire = std::atan2(outFirePos.y - outIntermediatePos.y,
                                          outFirePos.x - outIntermediatePos.x);
            totalTime += std::fabs(normalizeAngle(dirToFire - droneDir)) / cfg.angularSpeed;
            totalTime += calcTimeOfFlight(outIntermediatePos, outFirePos,
                dirToFire, cfg.turnThreshold, cfg.angularSpeed,
                cfg.attackSpeed, cfg.attackSpeed, cfg.accelPath);
        } else {
            totalTime = calcTimeOfFlight(dronePos, outFirePos,
                droneDir, cfg.turnThreshold, cfg.angularSpeed,
                droneSpeed, cfg.attackSpeed, cfg.accelPath);
        }

        if (std::fabs(totalTime - prevTotal) < TIME_CHANGE_THRESHOLD) break;
    }

    outTotalTime = totalTime;
    outAimPoint  = dronePos + Coord{ std::cos(droneDir), std::sin(droneDir) } * hDist_;
    return true;
}
