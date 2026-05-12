#include <cstring>
#include <cmath>
#define _USE_MATH_DEFINES

constexpr float GRAVITY = 9.81f; // m/s^2
constexpr float ACCEPTABLE_DEVIATION = 1e-7f; // small value for floating-point comparisons

float calculateAmmoFlightTime(float ammoMass, float ammoDrag, float ammoLift, float droneAttackSpeed, float droneHeight);

float calculateAmmoHorizontalDistance(float ammoMass, float ammoDrag, float ammoLift, float droneAttackSpeed, float ammoFlightTime);