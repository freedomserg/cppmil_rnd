#include "ballistics.hpp"
#include <gtest/gtest.h>

TEST(BallisticsTest, ComputesAmmoFlightTime) {
    float flightTimeVOG17 = calculateAmmoFlightTime(0.35f, 0.07f, 0.0f, 14.0f, 60.0f);
    EXPECT_NEAR(flightTimeVOG17, 4.104f, 0.001f);
}

TEST(BallisticsTest, ComputesAmmoHorizontalDistance) {
    float horizontalDistanceVOG17 = calculateAmmoHorizontalDistance(0.35f, 0.07f, 0.0f, 14.0f, 4.104f);
    EXPECT_NEAR(horizontalDistanceVOG17, 40.328f, 0.001f);
}

// End-to-end test using the values from data/sample_vog17.txt.
// Drone at (-100, -100, alt=60), target at (50, 50), VOG-17, speed=14, accel path=5.
// No maneuver needed.
// Expected drop point: (~21.5, ~21.5).
TEST(BallisticsTest, ReferenceSampleVOG17DropPoint) {
    float xd = -100.0f, yd = -100.0f;
    const float targetX = 50.0f, targetY = 50.0f;
    const float attackSpeed = 14.0f, accelerationPath = 5.0f;

    float ammoMass{}, ammoDrag{}, ammoLift{};
    bool validSpeed{}, validAccel{}, validAmmo{};
    validateInputParameters(attackSpeed, accelerationPath, "VOG-17",
                            ammoMass, ammoDrag, ammoLift,
                            validSpeed, validAccel, validAmmo);
    ASSERT_TRUE(validSpeed && validAccel && validAmmo);

    const float flightTime = calculateAmmoFlightTime(ammoMass, ammoDrag, ammoLift, attackSpeed, 60.0f);
    ASSERT_GT(flightTime, 0.0f);

    const float horizDist = calculateAmmoHorizontalDistance(ammoMass, ammoDrag, ammoLift, attackSpeed, flightTime);

    float distToTarget{};
    bool withManeuver{};
    ASSERT_TRUE(prepareDropApproach(xd, yd, targetX, targetY, horizDist, accelerationPath, distToTarget, withManeuver));
    EXPECT_FALSE(withManeuver);

    float fireX{}, fireY{};
    calculateDropPointCoordinates(distToTarget, horizDist, xd, yd, targetX, targetY, fireX, fireY);
    EXPECT_NEAR(fireX, 21.49f, 0.01f);
    EXPECT_NEAR(fireY, 21.49f, 0.01f);
}

// If the target is very close, the drone needs to perform a maneuver to get into position for the drop.
TEST(BallisticsTest, PrepareDropApproachTriggersManeuver) {
    float xd = 0.0f, yd = 0.0f;
    const float targetX = 30.0f, targetY = 0.0f;
    const float h = 40.324f, accel = 5.0f;

    float distToTarget{};
    bool withManeuver{};
    ASSERT_TRUE(prepareDropApproach(xd, yd, targetX, targetY, h, accel, distToTarget, withManeuver));
    EXPECT_TRUE(withManeuver);
    EXPECT_NEAR(xd, -15.324f, 0.01f);
    EXPECT_NEAR(yd, 0.0f, 0.01f);
    EXPECT_NEAR(distToTarget, h + accel, 0.01f);
}

TEST(BallisticsTest, GlidingAmmoHasPositiveFlightTime) {
    float t = calculateAmmoFlightTime(0.45f, 0.10f, 1.0f, 14.0f, 60.0f);
    EXPECT_GT(t, 0.0f);
    EXPECT_NEAR(t, 4.691f, 0.01f);
}

TEST(BallisticsTest, UnknownAmmoIsRejected) {
    float mass{}, drag{}, lift{};
    bool validSpeed{}, validAccel{}, validAmmo{};
    validateInputParameters(14.0f, 5.0f, "VOG-22", mass, drag, lift,
                            validSpeed, validAccel, validAmmo);
    EXPECT_FALSE(validAmmo);
}

TEST(BallisticsTest, ZeroAttackSpeedIsRejected) {
    float mass{}, drag{}, lift{};
    bool validSpeed{}, validAccel{}, validAmmo{};
    validateInputParameters(0.0f, 5.0f, "VOG-17", mass, drag, lift,
                            validSpeed, validAccel, validAmmo);
    EXPECT_FALSE(validSpeed);
}

TEST(BallisticsTest, ZeroAccelerationPathIsRejected) {
    float mass{}, drag{}, lift{};
    bool validSpeed{}, validAccel{}, validAmmo{};
    validateInputParameters(14.0f, 0.0f, "VOG-17", mass, drag, lift,
                            validSpeed, validAccel, validAmmo);
    EXPECT_FALSE(validAccel);
}