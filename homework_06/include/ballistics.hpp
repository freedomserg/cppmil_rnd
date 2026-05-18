#include <cstring>
#include <cmath>
#define _USE_MATH_DEFINES

constexpr int MAX_AMMO_NAME_LENGTH = 12; // 11 chars + 1 for null terminator
constexpr float GRAVITY = 9.81f; // m/s^2
constexpr float ACCEPTABLE_DEVIATION = 1e-7f; // small value for floating-point comparisons

void readInputParameters(const char* inputPath, int ammoNameSize, float& outXd, float& outYd, float& outZd, float& outTargetX, float& outTargetY, float& outAttackSpeed, float& outAccelerationPath, char* outAmmoName);

void validateInputParameters(const float& attackSpeed, const float& accelerationPath, const char* ammoName, float& outAmmoMassKg, float& outAmmoDrag, float& outAmmoLift, bool& outValidAttackSpeed, bool& outValidAccelerationPath, bool& outValidAmmoName);

float calculateAmmoFlightTime(float ammoMass, float ammoDrag, float ammoLift, float droneAttackSpeed, float droneHeight);

float calculateAmmoHorizontalDistance(float ammoMass, float ammoDrag, float ammoLift, float droneAttackSpeed, float ammoFlightTime);

inline float distance2D(float x1, float y1, float x2, float y2) {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

bool prepareDropApproach(float& xd, float& yd, const float& targetX, const float& targetY,
                         const float& ammoHorizontalDistance, const float& accelerationPath,
                         float& outDistanceToTarget, bool& outWithManeuver);

void calculateDropPointCoordinates(const float& distanceToTarget, const float& ammoHorizontalDistance, const float& xd, const float& yd, const float& targetX, const float& targetY, float& outFireX, float& outFireY);