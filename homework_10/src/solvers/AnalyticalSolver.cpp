#include "solvers/AnalyticalSolver.h"
#include "MathUtils.h"
#include "Logger.h"
#include <algorithm>
#include <cmath>
#include <numbers>

// Час падіння боєприпасу з cfg.altitude (кубічне рівняння, метод Кардано).
auto AnalyticalSolver::calcAmmoDropTime(
    const DroneConfig& cfg, const AmmoParams& ammo, float& outT) -> bool
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

    constexpr float PI = std::numbers::pi_v<float>;
    float t = 2.0f * std::sqrt(-p / 3.0f)
            * std::cos((phi + 4.0f * PI) / 3.0f) - b / (3.0f * a);
    if (t < 0) { LOG("Error: negative flight time, t = " << t); return false; }

    outT = t;
    return true;
}

// Горизонтальна дистанція боєприпасу за час t.
auto AnalyticalSolver::calcAmmoHDistance(
    const DroneConfig& cfg, const AmmoParams& ammo, float t) -> float
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

AnalyticalSolver::AnalyticalSolver(const DroneConfig& cfg, const AmmoParams& ammo) {
    ready_ = calcAmmoDropTime(cfg, ammo, ammoFlightTime_);
    if (ready_) {
        hDist_ = calcAmmoHDistance(cfg, ammo, ammoFlightTime_);
        LOG("Ammo flight time: " << ammoFlightTime_
            << " s, horizontal distance: " << hDist_ << " m");
    }
}
