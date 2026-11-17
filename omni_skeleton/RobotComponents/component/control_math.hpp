#ifndef CONTROL_MATH_HPP_
#define CONTROL_MATH_HPP_
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include "module_state.hpp"
#include "robot_config.hpp"
namespace robot {
struct Freshness {
  uint32_t tick = 0;
  bool seen = false;
  void update(uint32_t now) { tick = now; seen = true; }
  bool online(uint32_t now) const {
    return seen && static_cast<uint32_t>(now - tick) <= config::kTimeoutMs;
  }
};
inline float Deadband(float value) {
  if (!std::isfinite(value)) return 0.0f;
  value = std::clamp(value, -1.0f, 1.0f);
  return std::fabs(value) < 0.04f ? 0.0f :
      std::copysign((std::fabs(value) - 0.04f) / 0.96f, value);
}
inline float Approach(float current, float target, float step) {
  return current + std::clamp(target - current, -step, step);
}
inline bool ValidDbus(const uint8_t *d) {
  if (!d) return false;
  const unsigned ch[5] = {
    static_cast<unsigned>((d[0] | (d[1] << 8)) & 0x7ff),
    static_cast<unsigned>(((d[1] >> 3) | (d[2] << 5)) & 0x7ff),
    static_cast<unsigned>(((d[2] >> 6) | (d[3] << 2) | (d[4] << 10)) & 0x7ff),
    static_cast<unsigned>(((d[4] >> 1) | (d[5] << 7)) & 0x7ff),
    static_cast<unsigned>(d[16] | (d[17] << 8))};
  for (unsigned v : ch) if (v < 364 || v > 1684) return false;
  return ((d[5] >> 6) & 3) != 0 && ((d[5] >> 4) & 3) != 0 &&
      d[12] <= 1 && d[13] <= 1;
}
// DESIGN TRADE-OFF: commands stay in the operator/gimbal frame, then rotate
// into the body frame every 1 ms. This costs sin/cos per cycle but keeps push-
// forward aligned with the barrel even while the chassis spins.
// x forward, y left, z CCW; delta = body heading - gimbal heading.
// Motor order LF, LB, RB, RF; positive wheel speeds use the configured motor directions.
inline std::array<float, 4> WheelTargets(ChassisMotion motion, float delta) {
  const float c = std::cos(delta), s = std::sin(delta);
  const float vx = motion.vx * c + motion.vy * s;
  const float vy = -motion.vx * s + motion.vy * c;
  const float r = config::kWheelRadius;
  const float rot = (config::kHalfWheelbase + config::kHalfTrack) * motion.w;
  std::array<float, 4> wheel = {(vx-vy-rot)/r, (vx+vy-rot)/r,
                              (vx-vy+rot)/r, (vx+vy+rot)/r};
  float scale = 1.0f;
  for (float value : wheel) scale = std::max(scale, std::fabs(value)/config::kMaxWheelSpeed);
  // DESIGN TRADE-OFF: scale every wheel by one factor instead of clipping each
  // wheel independently. Absolute speed is reduced, but motion direction and
  // translation/rotation ratio are preserved under saturation.
  for (float &value : wheel) value /= scale;
  return wheel;
}
} // namespace robot
#endif
