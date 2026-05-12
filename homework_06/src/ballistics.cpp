#include "ballistics.hpp"
#include <iostream>

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