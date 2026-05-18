#include "ballistics.hpp"
#include <cstring>
#include <fstream>
#include <iostream>
#include <numbers>
#include <string>

auto read_input_parameters(const char* input_path,
                           int ammo_name_size,
                           float& out_xd,
                           float& out_yd,
                           float& out_zd,
                           float& out_target_x,
                           float& out_target_y,
                           float& out_attack_speed,
                           float& out_acceleration_path,
                           char* out_ammo_name) -> void
{
  std::ifstream input_file(input_path);
  if (!input_file) {
    std::cerr << "Error opening file: " << input_path << '\n';
    return;
  }

  std::string name;
  if (input_file >> out_xd >> out_yd >> out_zd >> out_target_x >> out_target_y >> out_attack_speed >> out_acceleration_path >> name) {
    std::strncpy(out_ammo_name, name.c_str(), static_cast<std::size_t>(ammo_name_size - 1));
    out_ammo_name[ammo_name_size - 1] =
      '\0';  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic) -- null-terminate the C-string buffer
    std::cout << "Input drone coordinates: " << out_xd << ", " << out_yd << ", " << out_zd << '\n';
    std::cout << "Input target coordinates: " << out_target_x << ", " << out_target_y << '\n';
    std::cout << "Input attack speed: " << out_attack_speed << '\n';
    std::cout << "Input acceleration path: " << out_acceleration_path << '\n';
    std::cout << "Input ammo name: " << out_ammo_name << '\n';
  }
  else {
    std::cerr << "Error reading data from file: " << input_path << '\n';
  }

  if (input_file.is_open()) {
    input_file.close();
  }
}

auto validate_input_parameters(const float& attack_speed,
                               const float& acceleration_path,
                               const char* ammo_name,
                               float& out_ammo_mass_kg,
                               float& out_ammo_drag,
                               float& out_ammo_lift,
                               bool& out_valid_attack_speed,
                               bool& out_valid_acceleration_path,
                               bool& out_valid_ammo_name) -> void
{
  if (attack_speed <= 0) {
    std::cerr << "Error: attack speed must be greater than zero, attack_speed=" << attack_speed << '\n';
    out_valid_attack_speed = false;
    return;
  }
  out_valid_attack_speed = true;

  if (acceleration_path <= 0) {
    std::cerr << "Error: acceleration path must be greater than zero, acceleration_path=" << acceleration_path << '\n';
    out_valid_acceleration_path = false;
    return;
  }
  out_valid_acceleration_path = true;

  if (std::strcmp(ammo_name, "VOG-17") == 0) {
    out_ammo_mass_kg = kVog17MassKg;
    out_ammo_drag = kVog17Drag;
    out_ammo_lift = kVog17Lift;
  }
  else if (std::strcmp(ammo_name, "M67") == 0) {
    out_ammo_mass_kg = kM67MassKg;
    out_ammo_drag = kM67Drag;
    out_ammo_lift = kM67Lift;
  }
  else if (std::strcmp(ammo_name, "RKG-3") == 0) {
    out_ammo_mass_kg = kRkg3MassKg;
    out_ammo_drag = kRkg3Drag;
    out_ammo_lift = kRkg3Lift;
  }
  else if (std::strcmp(ammo_name, "GLIDING-VOG") == 0) {
    out_ammo_mass_kg = kGlidingVogMassKg;
    out_ammo_drag = kGlidingVogDrag;
    out_ammo_lift = kGlidingVogLift;
  }
  else if (std::strcmp(ammo_name, "GLIDING-RKG") == 0) {
    out_ammo_mass_kg = kGlidingRkgMassKg;
    out_ammo_drag = kGlidingRkgDrag;
    out_ammo_lift = kGlidingRkgLift;
  }
  else {
    std::cerr << "Error: unknown ammo type: " << ammo_name << '\n';
    out_valid_ammo_name = false;
    return;
  }
  out_valid_ammo_name = true;
}

auto calculate_ammo_flight_time(float ammo_mass, float ammo_drag, float ammo_lift, float drone_attack_speed, float drone_height) -> float
{
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers) -- numeric coefficients from depressed cubic formula
  // a = d·g·m − 2d²·l·V₀
  float coeff_a = ammo_drag * kGravity * ammo_mass - 2.0F * ammo_drag * ammo_drag * ammo_lift * drone_attack_speed;
  std::cout << "Calculated coefficient a: " << coeff_a << '\n';

  if (std::abs(coeff_a) < kAcceptableDeviation) {
    std::cerr << "Error: coefficient a cannot be zero or too close to zero. a = " << coeff_a << '\n';
    return -1.0F;
  }

  // b = −3g·m² + 3d·l·m·V₀
  float coeff_b = -3.0F * kGravity * ammo_mass * ammo_mass + 3.0F * ammo_drag * ammo_lift * ammo_mass * drone_attack_speed;
  std::cout << "Calculated coefficient b: " << coeff_b << '\n';

  // c = 6m²·Z₀
  float coeff_c = 6.0F * ammo_mass * ammo_mass * drone_height;
  std::cout << "Calculated coefficient c: " << coeff_c << '\n';

  // p = − b² / (3a²)
  float coeff_p = -coeff_b * coeff_b / (3.0F * coeff_a * coeff_a);
  std::cout << "Calculated coefficient p: " << coeff_p << '\n';

  // q = 2b³ / (27a³) + c / a
  float coeff_q = 2.0F * coeff_b * coeff_b * coeff_b / (27.0F * coeff_a * coeff_a * coeff_a) + coeff_c / coeff_a;
  std::cout << "Calculated coefficient q: " << coeff_q << '\n';

  if (coeff_p >= 0) {
    std::cerr << "Error: p must be negative to calculate the root correctly. p = " << coeff_p << '\n';
    return -1.0F;
  }

  // φ = arccos( 3q / (2p) · √(−3/p) )
  float acos_arg = 3.0F * coeff_q / (2.0F * coeff_p) * std::sqrt(-3.0F / coeff_p);
  // verify that acos_arg is within the valid range for arccos which is [-1, 1]
  if (acos_arg < (-1.0F - kAcceptableDeviation) || acos_arg > (1.0F + kAcceptableDeviation)) {
    std::cerr << "Error: arccos argument out of range [-1, 1]: " << acos_arg << '\n';
    return -1.0F;
  }
  if (acos_arg < -1.0F) {
    acos_arg = -1.0F;  // clamp to -1 if slightly below due to floating-point errors
  }
  if (acos_arg > 1.0F) {
    acos_arg = 1.0F;  // clamp to 1 if slightly above due to floating-point errors
  }
  float phi = std::acos(acos_arg);
  std::cout << "Calculated angle phi: " << phi << " radians\n";

  // t = 2√(−p/3) · cos( (φ + 4π) / 3 ) − b / (3a)
  float flight_t =
    2.0F * std::sqrt(-coeff_p / 3.0F) * std::cos((phi + 4.0F * std::numbers::pi_v<float>) / 3.0F) - coeff_b / (3.0F * coeff_a);

  if (flight_t < 0) {
    std::cerr << "Error: calculated flight time cannot be negative. t = " << flight_t << '\n';
    return -1.0F;
  }
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

  return flight_t;
}

auto calculate_ammo_horizontal_distance(float ammo_mass, float ammo_drag, float ammo_lift, float drone_attack_speed, float ammo_flight_time)
  -> float
{
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers) -- numeric coefficients from Taylor series expansion
  // h = V₀t − t²d·V₀/(2m) + t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
  // + t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³)
  // + t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)

  const float kAmmoFlightTimeSquared = ammo_flight_time * ammo_flight_time;
  const float kAmmoLiftSquared = ammo_lift * ammo_lift;
  const float kAmmoMassSquared = ammo_mass * ammo_mass;
  const float kAmmoDragTimesSix = 6.0F * ammo_drag;
  const float kAmmoDragCubed = ammo_drag * ammo_drag * ammo_drag;

  // quadratic part: −t²d·V₀/(2m)
  float quadratic_part_numerator = -kAmmoFlightTimeSquared * ammo_drag * drone_attack_speed;
  float quadratic_part_denominator = 2.0F * ammo_mass;
  float quadratic_part = quadratic_part_numerator / quadratic_part_denominator;

  // cubic part: t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²)
  float cubic_part_numerator_term1 = kAmmoDragTimesSix * kGravity * ammo_lift * ammo_mass;
  float cubic_part_numerator_term2 = kAmmoDragTimesSix * ammo_drag * (kAmmoLiftSquared - 1.0F) * drone_attack_speed;
  float cubic_part_numerator = kAmmoFlightTimeSquared * ammo_flight_time * (cubic_part_numerator_term1 - cubic_part_numerator_term2);
  float cubic_part_denominator = 36.0F * kAmmoMassSquared;
  float cubic_part = cubic_part_numerator / cubic_part_denominator;

  // quartic part: t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³)
  float quartic_part_numerator_term1 =
    -kAmmoDragTimesSix * ammo_drag * kGravity * ammo_lift * (1.0F + kAmmoLiftSquared + kAmmoLiftSquared * kAmmoLiftSquared) * ammo_mass;
  float quartic_part_numerator_term2 = 3.0F * kAmmoDragCubed * kAmmoLiftSquared * (1.0F + kAmmoLiftSquared) * drone_attack_speed;
  float quartic_part_numerator_term3 =
    6.0F * kAmmoDragCubed * kAmmoLiftSquared * kAmmoLiftSquared * (1.0F + kAmmoLiftSquared) * drone_attack_speed;
  float quartic_part_numerator = kAmmoFlightTimeSquared * kAmmoFlightTimeSquared *
                                 (quartic_part_numerator_term1 + quartic_part_numerator_term2 + quartic_part_numerator_term3);
  float quartic_part_denominator = 36.0F * (1.0F + kAmmoLiftSquared) * (1.0F + kAmmoLiftSquared) * kAmmoMassSquared * ammo_mass;
  float quartic_part = quartic_part_numerator / quartic_part_denominator;

  // quintic part: t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
  float quintic_part_numerator_term1 = 3.0F * kAmmoDragCubed * kGravity * kAmmoLiftSquared * ammo_lift * ammo_mass;
  float quintic_part_numerator_term2 =
    -3.0F * kAmmoDragCubed * ammo_drag * kAmmoLiftSquared * (1.0F + kAmmoLiftSquared) * drone_attack_speed;
  float quintic_part_numerator =
    kAmmoFlightTimeSquared * kAmmoFlightTimeSquared * ammo_flight_time * (quintic_part_numerator_term1 + quintic_part_numerator_term2);
  float quintic_part_denominator = 36.0F * (1.0F + kAmmoLiftSquared) * kAmmoMassSquared * kAmmoMassSquared;
  float quintic_part = quintic_part_numerator / quintic_part_denominator;

  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
  return drone_attack_speed * ammo_flight_time + quadratic_part + cubic_part + quartic_part + quintic_part;
}

auto prepare_drop_approach(float& drone_x,
                           float& drone_y,
                           const float& target_x,
                           const float& target_y,
                           const float& ammo_horizontal_distance,
                           const float& acceleration_path,
                           float& out_distance_to_target,
                           bool& out_with_maneuver) -> bool
{
  out_distance_to_target = distance_2d(drone_x, drone_y, target_x, target_y);
  std::cout << "Calculated distance to target: " << out_distance_to_target << " meters\n";

  if (out_distance_to_target < kAcceptableDeviation) {
    std::cerr << "Error: calculated distance to target cannot be zero or too close to zero. calculatedDistance=" << out_distance_to_target
              << '\n';
    return false;
  }

  out_with_maneuver = (ammo_horizontal_distance + acceleration_path > out_distance_to_target);
  if (out_with_maneuver) {
    drone_x = target_x - (target_x - drone_x) * (ammo_horizontal_distance + acceleration_path) / out_distance_to_target;
    drone_y = target_y - (target_y - drone_y) * (ammo_horizontal_distance + acceleration_path) / out_distance_to_target;
    out_distance_to_target = distance_2d(drone_x, drone_y, target_x, target_y);
    std::cout << "Drone needs to maneuver to new coordinates before dropping ammo: drone_x = " << drone_x << ", drone_y = " << drone_y
              << '\n';
    std::cout << "Recalculated distance to target after maneuver: " << out_distance_to_target << " meters\n";
  }

  return true;
}

auto calculate_drop_point_coordinates(const float& distance_to_target,
                                      const float& ammo_horizontal_distance,
                                      const float& drone_x,
                                      const float& drone_y,
                                      const float& target_x,
                                      const float& target_y,
                                      float& out_fire_x,
                                      float& out_fire_y) -> void
{
  float ratio = (distance_to_target - ammo_horizontal_distance) / distance_to_target;
  out_fire_x = drone_x + (target_x - drone_x) * ratio;
  out_fire_y = drone_y + (target_y - drone_y) * ratio;
}
