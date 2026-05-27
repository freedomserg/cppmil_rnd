#include <cmath>

constexpr int kMaxAmmoNameLength = 12;         // 11 chars + 1 for null terminator
constexpr float kGravity = 9.81F;              // m/s^2
constexpr float kAcceptableDeviation = 1e-7F;  // small value for floating-point comparisons

constexpr float kVog17MassKg = 0.35F;
constexpr float kVog17Drag = 0.07F;
constexpr float kVog17Lift = 0.0F;
constexpr float kM67MassKg = 0.6F;
constexpr float kM67Drag = 0.10F;
constexpr float kM67Lift = 0.0F;
constexpr float kRkg3MassKg = 1.2F;
constexpr float kRkg3Drag = 0.10F;
constexpr float kRkg3Lift = 0.0F;
constexpr float kGlidingVogMassKg = 0.45F;
constexpr float kGlidingVogDrag = 0.10F;
constexpr float kGlidingVogLift = 1.0F;
constexpr float kGlidingRkgMassKg = 1.4F;
constexpr float kGlidingRkgDrag = 0.10F;
constexpr float kGlidingRkgLift = 1.0F;

auto read_input_parameters(const char* input_path,
                           int ammo_name_size,
                           float& out_xd,
                           float& out_yd,
                           float& out_zd,
                           float& out_target_x,
                           float& out_target_y,
                           float& out_attack_speed,
                           float& out_acceleration_path,
                           char* out_ammo_name) -> void;

auto validate_input_parameters(const float& attack_speed,
                               const float& acceleration_path,
                               const char* ammo_name,
                               float& out_ammo_mass_kg,
                               float& out_ammo_drag,
                               float& out_ammo_lift,
                               bool& out_valid_attack_speed,
                               bool& out_valid_acceleration_path,
                               bool& out_valid_ammo_name) -> void;

auto calculate_ammo_flight_time(float ammo_mass, float ammo_drag, float ammo_lift, float drone_attack_speed, float drone_height) -> float;

auto calculate_ammo_horizontal_distance(float ammo_mass, float ammo_drag, float ammo_lift, float drone_attack_speed, float ammo_flight_time)
  -> float;

inline auto distance_2d(float from_x, float from_y, float to_x, float to_y) -> float
{
  return std::sqrt((to_x - from_x) * (to_x - from_x) + (to_y - from_y) * (to_y - from_y));
}

auto prepare_drop_approach(float& drone_x,
                           float& drone_y,
                           const float& target_x,
                           const float& target_y,
                           const float& ammo_horizontal_distance,
                           const float& acceleration_path,
                           float& out_distance_to_target,
                           bool& out_with_maneuver) -> bool;

auto calculate_drop_point_coordinates(const float& distance_to_target,
                                      const float& ammo_horizontal_distance,
                                      const float& drone_x,
                                      const float& drone_y,
                                      const float& target_x,
                                      const float& target_y,
                                      float& out_fire_x,
                                      float& out_fire_y) -> void;
