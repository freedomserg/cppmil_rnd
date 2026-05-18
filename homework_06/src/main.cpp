#include "ballistics.hpp"

#include <array>
#include <fstream>
#include <iostream>

auto main(int argc, char** argv) -> int
{
  if (argc != 2) {
    std::cerr << "usage: ballistics_cli <input_path>\n";
    return 1;
  }

  float drone_x{};
  float drone_y{};
  float drone_z{};
  float target_x{};
  float target_y{};
  float attack_speed{};
  float acceleration_path{};
  float ammo_mass_kg{};
  float ammo_drag{};
  float ammo_lift{};
  std::array<char, kMaxAmmoNameLength> ammo_name{};
  bool with_maneuver{false};

  read_input_parameters(argv[1],  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic) -- standard CLI argv access
                        kMaxAmmoNameLength,
                        drone_x,
                        drone_y,
                        drone_z,
                        target_x,
                        target_y,
                        attack_speed,
                        acceleration_path,
                        ammo_name.data());

  bool valid_attack_speed{false};
  bool valid_acceleration_path{false};
  bool valid_ammo_name{false};
  validate_input_parameters(attack_speed,
                            acceleration_path,
                            ammo_name.data(),
                            ammo_mass_kg,
                            ammo_drag,
                            ammo_lift,
                            valid_attack_speed,
                            valid_acceleration_path,
                            valid_ammo_name);

  if (!valid_attack_speed) {
    std::cerr << "Error: attack speed must be greater than zero, attack_speed=" << attack_speed << '\n';
    return 1;
  }
  if (!valid_acceleration_path) {
    std::cerr << "Error: acceleration path must be greater than zero, acceleration_path=" << acceleration_path << '\n';
    return 1;
  }
  if (!valid_ammo_name) {
    std::cerr << "Error: unknown ammo type: " << ammo_name.data() << '\n';
    return 1;
  }

  // calculate the ammo flight time
  float ammo_flight_time = calculate_ammo_flight_time(ammo_mass_kg, ammo_drag, ammo_lift, attack_speed, drone_z);
  std::cout << "Calculated ammo flight time: " << ammo_flight_time << " seconds\n";
  if (ammo_flight_time < 0) {
    std::cerr << "Failed to calculate ammo flight time due to invalid parameters.\n";
    return 1;
  }

  // calculate the horizontal flight distance of the ammo
  float ammo_horizontal_distance = calculate_ammo_horizontal_distance(ammo_mass_kg, ammo_drag, ammo_lift, attack_speed, ammo_flight_time);
  std::cout << "Calculated ammo horizontal distance: " << ammo_horizontal_distance << " meters\n";

  float distance_to_target{};
  float fire_x{};
  float fire_y{};
  if (!prepare_drop_approach(
        drone_x, drone_y, target_x, target_y, ammo_horizontal_distance, acceleration_path, distance_to_target, with_maneuver)) {
    return 1;
  }
  calculate_drop_point_coordinates(distance_to_target, ammo_horizontal_distance, drone_x, drone_y, target_x, target_y, fire_x, fire_y);
  std::cout << "Calculated drop point coordinates: fire_x = " << fire_x << ", fire_y = " << fire_y << '\n';

  // log drop point coordinates to the output file
  std::ofstream output_file("output.txt");
  if (!output_file) {
    std::cerr << "Error opening output file\n";
    return 1;
  }

  // if the drone needs to maneuver, log the new drone coordinates along with the drop point coordinates, otherwise log only the drop point
  // coordinates
  if (with_maneuver) {
    output_file << drone_x << " " << drone_y << " " << fire_x << " " << fire_y << '\n';
  }
  else {
    output_file << fire_x << " " << fire_y << '\n';
  }

  output_file.close();

  return 0;
}
