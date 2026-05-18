#include "ballistics.hpp"
#include <iostream>
#include <fstream>
#include <string>

void readInputParameters(const char* inputPath, int ammoNameSize, float& outXd, float& outYd, float& outZd, float& outTargetX, float& outTargetY, float& outAttackSpeed, float& outAccelerationPath, char* outAmmoName) {
    std::ifstream inputFile(inputPath);
    if (!inputFile) {
        std::cerr << "Error opening file: " << inputPath << std::endl;
        return;
    }

    std::string name;
    if (inputFile >> outXd >> outYd >> outZd >> outTargetX >> outTargetY >> outAttackSpeed >> outAccelerationPath >> name) {
        std::strncpy(outAmmoName, name.c_str(), ammoNameSize - 1);
        outAmmoName[ammoNameSize - 1] = '\0';
        std::cout << "Input drone coordinates: " << outXd << ", " << outYd << ", " << outZd << std::endl;
        std::cout << "Input target coordinates: " << outTargetX << ", " << outTargetY << std::endl;
        std::cout << "Input attack speed: " << outAttackSpeed << std::endl;
        std::cout << "Input acceleration path: " << outAccelerationPath << std::endl;
        std::cout << "Input ammo name: " << outAmmoName << std::endl;
    } else {
        std::cerr << "Error reading data from file: " << inputPath << std::endl;
    }

    if (inputFile.is_open()) {
        inputFile.close();     
    }
}

void validateInputParameters(const float& attackSpeed, const float& accelerationPath, const char* ammoName, float& outAmmoMassKg, float& outAmmoDrag, float& outAmmoLift, bool& outValidAttackSpeed, bool& outValidAccelerationPath, bool& outValidAmmoName) {
    if (attackSpeed <= 0) {
        std::cerr << "Error: attack speed must be greater than zero, attackSpeed=" << attackSpeed << std::endl;
        outValidAttackSpeed = false;
        return;
    } else {
        outValidAttackSpeed = true;
    }
    if (accelerationPath <= 0) {
        std::cerr << "Error: acceleration path must be greater than zero, accelerationPath=" << accelerationPath << std::endl;
        outValidAccelerationPath = false;
        return;
    } else {
        outValidAccelerationPath = true;
    }

    if (std::strcmp(ammoName, "VOG-17") == 0) {
        outAmmoMassKg = 0.35f;
        outAmmoDrag = 0.07f;
        outAmmoLift = 0.0f;
    } else if (std::strcmp(ammoName, "M67") == 0) {
        outAmmoMassKg = 0.6f;
        outAmmoDrag = 0.10f;
        outAmmoLift = 0.0f;
    } else if (std::strcmp(ammoName, "RKG-3") == 0) {
        outAmmoMassKg = 1.2f;
        outAmmoDrag = 0.10f;
        outAmmoLift = 0.0f;
    } else if (std::strcmp(ammoName, "GLIDING-VOG") == 0) {
        outAmmoMassKg = 0.45f;
        outAmmoDrag = 0.10f;
        outAmmoLift = 1.0f;
    } else if (std::strcmp(ammoName, "GLIDING-RKG") == 0) {
        outAmmoMassKg = 1.4f;
        outAmmoDrag = 0.10f;
        outAmmoLift = 1.0f;
    } else {
        std::cerr << "Error: unknown ammo type: " << ammoName << std::endl;
        outValidAmmoName = false;
        return;
    }
    outValidAmmoName = true;
}

float calculateAmmoFlightTime(float ammoMass, float ammoDrag, float ammoLift, float droneAttackSpeed, float droneHeight) {

    // a = d·g·m − 2d²·l·V₀
    float a = ammoDrag * GRAVITY * ammoMass - 2.0f * ammoDrag * ammoDrag * ammoLift * droneAttackSpeed;
    std::cout << "Calculated coefficient a: " << a << std::endl;

    if (std::abs(a) < ACCEPTABLE_DEVIATION) {
        std::cerr << "Error: coefficient a cannot be zero or too close to zero. a = " << a << std::endl;
        return -1.0f; // return an error value
    }

    // b = −3g·m² + 3d·l·m·V₀
    float b = -3.0f * GRAVITY * ammoMass * ammoMass + 3.0f * ammoDrag * ammoLift * ammoMass * droneAttackSpeed;
    std::cout << "Calculated coefficient b: " << b << std::endl;

    // c = 6m²·Z₀
    float c = 6.0f * ammoMass * ammoMass * droneHeight;
    std::cout << "Calculated coefficient c: " << c << std::endl;

    // p = − b² / (3a²)
    float p = -b * b / (3.0f * a * a);
    std::cout << "Calculated coefficient p: " << p << std::endl;

    // q = 2b³ / (27a³) + c / a
    float q = 2.0f * b * b * b / (27.0f * a * a * a) + c / a;
    std::cout << "Calculated coefficient q: " << q << std::endl;

    if (p >= 0) {
        std::cerr << "Error: p must be negative to calculate the root correctly. p = " << p << std::endl;
        return -1.0f; // return an error value
    }

    // φ = arccos( 3q / (2p) · √(−3/p) )
    float acosArg = 3.0f * q / (2.0f * p) * std::sqrt(-3.0f / p);
    // verify that acosArg is within the valid range for arccos which is [-1, 1]
    if (acosArg < (-1.0f - ACCEPTABLE_DEVIATION) || acosArg > (1.0f + ACCEPTABLE_DEVIATION)) {
        std::cerr << "Error: arccos argument out of range [-1, 1]: " << acosArg << std::endl;
        return -1.0f; // return an error value
    }
    if (acosArg < -1.0f) acosArg = -1.0f; // clamp to -1 if slightly below due to floating-point errors
    if (acosArg > 1.0f) acosArg = 1.0f; // clamp to 1 if slightly above due to floating-point errors
    float phi = std::acos(acosArg);
    std::cout << "Calculated angle phi: " << phi << " radians" << std::endl;

    // t = 2√(−p/3) · cos( (φ + 4π) / 3 ) − b / (3a)
    float t = 2.0f * std::sqrt(-p / 3.0f) * std::cos((phi + 4.0f * M_PI) / 3.0f) - b / (3.0f * a);

    if (t < 0) {
        std::cerr << "Error: calculated flight time cannot be negative. t = " << t << std::endl;
        return -1.0f; // return an error value
    }

    return t;
}

float calculateAmmoHorizontalDistance(float ammoMass, float ammoDrag, float ammoLift, float droneAttackSpeed, float ammoFlightTime) {
    // h = V₀t − t²d·V₀/(2m) + t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
    // + t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³)
    // + t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)

    const float ammoFlightTimeSquared = ammoFlightTime * ammoFlightTime;
    const float ammoLiftSquared = ammoLift * ammoLift;
    const float ammoMassSquared = ammoMass * ammoMass;
    const float ammoDragTimesSix = 6.0f * ammoDrag;
    const float ammoDragQubed = ammoDrag * ammoDrag * ammoDrag;

    // quadratic part: −t²d·V₀/(2m)
    // quadratic part numerator: −t²d·V₀
    float quadraticPartNumerator = -ammoFlightTimeSquared * ammoDrag * droneAttackSpeed;
    // quadratic part denominator: 2m
    float quadraticPartDenominator = 2.0f * ammoMass;
    float quadraticPart = quadraticPartNumerator / quadraticPartDenominator;

    // cubic part: t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²)
    // 6d·g·l·m
    float cubicPartNumeratorTerm1 = ammoDragTimesSix * GRAVITY * ammoLift * ammoMass;
    // 6d²(l²-1)·V₀
    float cubicPartNumeratorTerm2 = ammoDragTimesSix * ammoDrag * (ammoLiftSquared - 1.0f) * droneAttackSpeed;
    // cubic part numerator: t³(6d·g·l·m − 6d²(l²-1)·V₀)
    float cubicPartNumerator = ammoFlightTimeSquared * ammoFlightTime * (cubicPartNumeratorTerm1 - cubicPartNumeratorTerm2);
    // cubic part denominator: 36m²
    float cubicPartDenominator = 36.0f * ammoMassSquared;
    float cubicPart = cubicPartNumerator / cubicPartDenominator;

    // quartic part: t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³)
    // −6d²g·l·(1+l²+l⁴)m
    float quatricPartNumeratorTerm1 = -ammoDragTimesSix * ammoDrag * GRAVITY * ammoLift * (1.0f + ammoLiftSquared + ammoLiftSquared * ammoLiftSquared) * ammoMass;
    // 3d³l²(1+l²)V₀
    float quatricPartNumeratorTerm2 = 3.0f * ammoDragQubed * ammoLiftSquared * (1.0f + ammoLiftSquared) * droneAttackSpeed;
    // 6d³l⁴(1+l²)V₀
    float quatricPartNumeratorTerm3 = 6.0f * ammoDragQubed * ammoLiftSquared * ammoLiftSquared * (1.0f + ammoLiftSquared) * droneAttackSpeed;
    float quatricPartNumerator = ammoFlightTimeSquared * ammoFlightTimeSquared * (quatricPartNumeratorTerm1 + quatricPartNumeratorTerm2 + quatricPartNumeratorTerm3);
    
    // quartic part denominator: 36(1+l²)²m³
    float quatricPartDenominator = 36.0f * (1.0f + ammoLiftSquared) * (1.0f + ammoLiftSquared) * ammoMassSquared * ammoMass;
    float quatricPart = quatricPartNumerator / quatricPartDenominator;

    // quintic part: t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
    // 3d³g·l³m
    float quinticPartNumeratorTerm1 = 3.0f * ammoDragQubed * GRAVITY * ammoLiftSquared * ammoLift * ammoMass;
    // -3d⁴l²(1+l²)V₀
    float quinticPartNumeratorTerm2 = -3.0f * ammoDragQubed * ammoDrag * ammoLiftSquared * (1.0f + ammoLiftSquared) * droneAttackSpeed;;
    // quintic part numerator
    float quinticPartNumerator = ammoFlightTimeSquared * ammoFlightTimeSquared * ammoFlightTime * (quinticPartNumeratorTerm1 + quinticPartNumeratorTerm2);

    // quintic part denominator: 36(1+l²)m⁴
    float quinticPartDenominator = 36.0f * (1.0f + ammoLiftSquared) * ammoMassSquared * ammoMassSquared;
    float quinticPart = quinticPartNumerator / quinticPartDenominator;

    return droneAttackSpeed * ammoFlightTime + quadraticPart + cubicPart + quatricPart + quinticPart;
}

bool prepareDropApproach(float& xd, float& yd, const float& targetX, const float& targetY,
                         const float& ammoHorizontalDistance, const float& accelerationPath,
                         float& outDistanceToTarget, bool& outWithManeuver) {
    outDistanceToTarget = distance2D(xd, yd, targetX, targetY);
    std::cout << "Calculated distance to target: " << outDistanceToTarget << " meters" << std::endl;

    if (outDistanceToTarget < ACCEPTABLE_DEVIATION) {
        std::cerr << "Error: calculated distance to target cannot be zero or too close to zero. calculatedDistance=" << outDistanceToTarget << std::endl;
        return false;
    }

    outWithManeuver = (ammoHorizontalDistance + accelerationPath > outDistanceToTarget);
    if (outWithManeuver) {
        xd = targetX - (targetX - xd) * (ammoHorizontalDistance + accelerationPath) / outDistanceToTarget;
        yd = targetY - (targetY - yd) * (ammoHorizontalDistance + accelerationPath) / outDistanceToTarget;
        outDistanceToTarget = distance2D(xd, yd, targetX, targetY);
        std::cout << "Drone needs to maneuver to new coordinates before dropping ammo: xd = " << xd << ", yd = " << yd << std::endl;
        std::cout << "Recalculated distance to target after maneuver: " << outDistanceToTarget << " meters" << std::endl;
    }

    return true;
}

void calculateDropPointCoordinates(const float& distanceToTarget, const float& ammoHorizontalDistance, const float& xd, const float& yd, const float& targetX, const float& targetY, float& outFireX, float& outFireY) {
    float ratio = (distanceToTarget - ammoHorizontalDistance) / distanceToTarget;
    outFireX = xd + (targetX - xd) * ratio;
    outFireY = yd + (targetY - yd) * ratio;
}