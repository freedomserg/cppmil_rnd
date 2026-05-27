#include "solvers/AnalyticalSolver.h"
#include "Types.h"
#include "Logger.h"
#include <cmath>

constexpr int   NUM_APPROX_STEPS      = 3;
constexpr float TIME_CHANGE_THRESHOLD = 0.05f;

// ── Private helpers ─────────────────────────────────────────────────────────

// Ammo drop time from cfg.altitude.
bool AnalyticalSolver::calcAmmoDropTime(
    const DroneConfig& cfg, const AmmoParams& ammo, float& outT)
{
    float a = ammo.drag * GRAVITY * ammo.mass
            - 2.0f * ammo.drag * ammo.drag * ammo.lift * cfg.attackSpeed;
    if (std::fabs(a) < 1e-6f) { LOG("Error: coeff a ~ 0, a = " << a); return false; }

    float b = -3.0f * GRAVITY * ammo.mass * ammo.mass
            + 3.0f * ammo.drag * ammo.lift * ammo.mass * cfg.attackSpeed;
    float c =  6.0f * ammo.mass * ammo.mass * cfg.altitude;
    float p = -b * b / (3.0f * a * a);
    float q =  2.0f * b * b * b / (27.0f * a * a * a) + c / a;

    if (p >= 0) { LOG("Error: p must be negative, p = " << p); return false; }

    float arg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);
    if (arg < -1.0f - 1e-6f || arg > 1.0f + 1e-6f) {
        LOG("Error: arccos arg out of [-1,1]: " << arg); return false;
    }
    arg       = std::max(-1.0f, std::min(1.0f, arg));
    float phi = std::acos(arg);

    // t = 2√(−p/3) · cos( (φ + 4π) / 3 ) − b / (3a)
    constexpr float PI = std::numbers::pi_v<float>;
    float t = 2.0f * std::sqrt(-p / 3.0f)
            * std::cos((phi + 4.0f * PI) / 3.0f) - b / (3.0f * a);
    if (t < 0) { LOG("Error: negative flight time, t = " << t); return false; }

    outT = t;
    return true;
}

// Horizontal distance travelled by ammo during flight time t.
float AnalyticalSolver::calcAmmoHDistance(
    const DroneConfig& cfg, const AmmoParams& ammo, float t)
{
    float V  = cfg.attackSpeed;
    float m  = ammo.mass,  d  = ammo.drag,  l  = ammo.lift;
    float l2 = l*l,        d2 = d*d,        d3 = d2*d,       d4 = d3*d;
    float m2 = m*m,        m3 = m2*m,       m4 = m3*m,       lp = 1.0f + l2;

    return V*t
        - std::pow(t,2)*d*V / (2.0f*m)
        + std::pow(t,3)*(6.0f*d*GRAVITY*l*m - 6.0f*d2*(l2-1.0f)*V) / (36.0f*m2)
        + std::pow(t,4)*(-6.0f*d2*GRAVITY*l*(1.0f+l2+l2*l2)*m
            + 3.0f*d3*l2*lp*V + 6.0f*d3*l2*l2*lp*V) / (36.0f*lp*lp*m3)
        + std::pow(t,5)*(3.0f*d3*GRAVITY*l2*l*m - 3.0f*d4*l2*lp*V) / (36.0f*lp*m4);
}

void AnalyticalSolver::getIntermediateAndDropPoint(
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

Coord AnalyticalSolver::interpolate(const Target& target, float t, float arrayTimeStep) {
    int          raw  = static_cast<int>(std::floor(t / arrayTimeStep));
    int          n    = target.timeStepsCount;
    int          idx  = raw % n;
    int          next = (idx + 1) % n;
    float        frac = (t - raw * arrayTimeStep) / arrayTimeStep;
    const Coord* pos  = target.positions;
    return { pos[idx].x + (pos[next].x - pos[idx].x) * frac,
             pos[idx].y + (pos[next].y - pos[idx].y) * frac };
}

Coord AnalyticalSolver::extrapolate(
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

bool AnalyticalSolver::needStop(float desiredDir, float currentDir, float angleThreshold) {
    return std::fabs(normalizeAngle(desiredDir - currentDir)) > angleThreshold;
}

// Estimates drone flight time to targetPos, accounting for deceleration,
// optional stop-and-turn, re-acceleration, and cruise at maxSpeed.
float AnalyticalSolver::calcTimeOfFlight(
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

// ── Constructor ─────────────────────────────────────────────────────────────

AnalyticalSolver::AnalyticalSolver(const DroneConfig& cfg, const AmmoParams& ammo) {
    ready_ = calcAmmoDropTime(cfg, ammo, ammoFlightTime_);
    if (ready_) {
        hDist_ = calcAmmoHDistance(cfg, ammo, ammoFlightTime_);
        LOG("Ammo flight time: " << ammoFlightTime_
            << " s, horizontal distance: " << hDist_ << " m");
    }
}

// ── IBallisticSolver ────────────────────────────────────────────────────────

bool AnalyticalSolver::solve(
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
