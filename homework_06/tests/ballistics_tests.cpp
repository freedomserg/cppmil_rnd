#include "ballistics.hpp"
#include <gtest/gtest.h>

TEST(BallisticsTest, ComputesAmmoFlightTime)
{
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers) -- VOG-17 physical parameters and scenario altitude
  float flight_time_vog17 = calculate_ammo_flight_time(0.35F, 0.07F, 0.0F, 14.0F, 60.0F);
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
  EXPECT_NEAR(flight_time_vog17, 4.104F, 0.001F);
}

TEST(BallisticsTest, ComputesAmmoHorizontalDistance)
{
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers) -- VOG-17 physical parameters and known flight time
  float horizontal_distance_vog17 = calculate_ammo_horizontal_distance(0.35F, 0.07F, 0.0F, 14.0F, 4.104F);
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
  EXPECT_NEAR(horizontal_distance_vog17, 40.328F, 0.001F);
}

// End-to-end test using the values from data/sample_vog17.txt.
// Drone at (-100, -100, alt=60), target at (50, 50), VOG-17, speed=14, accel path=5.
// No maneuver needed.
// Expected drop point: (~21.5, ~21.5).
TEST(BallisticsTest, ReferenceSampleVOG17DropPoint)
{
  float drone_x = -100.0F;
  float drone_y = -100.0F;

  float ammo_mass{};
  float ammo_drag{};
  float ammo_lift{};
  bool valid_speed{};
  bool valid_accel{};
  bool valid_ammo{};
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers) -- sample scenario values from data/sample_vog17.txt
  validate_input_parameters(14.0F, 5.0F, "VOG-17", ammo_mass, ammo_drag, ammo_lift, valid_speed, valid_accel, valid_ammo);
  ASSERT_TRUE(valid_speed && valid_accel && valid_ammo);

  float flight_time = calculate_ammo_flight_time(ammo_mass, ammo_drag, ammo_lift, 14.0F, 60.0F);
  ASSERT_GT(flight_time, 0.0F);

  float horiz_dist = calculate_ammo_horizontal_distance(ammo_mass, ammo_drag, ammo_lift, 14.0F, flight_time);

  float dist_to_target{};
  bool with_maneuver{};
  ASSERT_TRUE(prepare_drop_approach(drone_x, drone_y, 50.0F, 50.0F, horiz_dist, 5.0F, dist_to_target, with_maneuver));
  EXPECT_FALSE(with_maneuver);

  float fire_x{};
  float fire_y{};
  calculate_drop_point_coordinates(dist_to_target, horiz_dist, drone_x, drone_y, 50.0F, 50.0F, fire_x, fire_y);
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
  EXPECT_NEAR(fire_x, 21.49F, 0.01F);
  EXPECT_NEAR(fire_y, 21.49F, 0.01F);
}

// If the target is very close, the drone needs to perform a maneuver to get into position for the drop.
TEST(BallisticsTest, PrepareDropApproachTriggersManeuver)
{
  float drone_x = 0.0F;
  float drone_y = 0.0F;

  float dist_to_target{};
  bool with_maneuver{};
  ASSERT_TRUE(prepare_drop_approach(drone_x, drone_y, 30.0F, 0.0F, 40.324F, 5.0F, dist_to_target, with_maneuver));
  EXPECT_TRUE(with_maneuver);
  EXPECT_NEAR(drone_x, -15.324F, 0.01F);
  EXPECT_NEAR(drone_y, 0.0F, 0.01F);
  EXPECT_NEAR(dist_to_target, 40.324F + 5.0F, 0.01F);
}

TEST(BallisticsTest, GlidingAmmoHasPositiveFlightTime)
{
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers) -- GLIDING-VOG physical parameters and scenario altitude
  float flight_time = calculate_ammo_flight_time(0.45F, 0.10F, 1.0F, 14.0F, 60.0F);
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
  EXPECT_GT(flight_time, 0.0F);
  EXPECT_NEAR(flight_time, 4.691F, 0.01F);
}

TEST(BallisticsTest, UnknownAmmoIsRejected)
{
  float ammo_mass{};
  float ammo_drag{};
  float ammo_lift{};
  bool valid_speed{};
  bool valid_accel{};
  bool valid_ammo{};
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers) -- arbitrary valid speed/accel to isolate ammo name check
  validate_input_parameters(14.0F, 5.0F, "VOG-22", ammo_mass, ammo_drag, ammo_lift, valid_speed, valid_accel, valid_ammo);
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
  EXPECT_FALSE(valid_ammo);
}

TEST(BallisticsTest, ZeroAttackSpeedIsRejected)
{
  float ammo_mass{};
  float ammo_drag{};
  float ammo_lift{};
  bool valid_speed{};
  bool valid_accel{};
  bool valid_ammo{};
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers) -- zero speed is the boundary value under test; 5.0F is a placeholder
  validate_input_parameters(0.0F, 5.0F, "VOG-17", ammo_mass, ammo_drag, ammo_lift, valid_speed, valid_accel, valid_ammo);
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
  EXPECT_FALSE(valid_speed);
}

TEST(BallisticsTest, ZeroAccelerationPathIsRejected)
{
  float ammo_mass{};
  float ammo_drag{};
  float ammo_lift{};
  bool valid_speed{};
  bool valid_accel{};
  bool valid_ammo{};
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers) -- zero accel path is the boundary value under test; 14.0F is a placeholder
  validate_input_parameters(14.0F, 0.0F, "VOG-17", ammo_mass, ammo_drag, ammo_lift, valid_speed, valid_accel, valid_ammo);
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
  EXPECT_FALSE(valid_accel);
}
