#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

#include <cmath>
#include <cstdint>

namespace robot {

enum class LifeState : uint8_t { Dead = 0, Resurrection = 1, Working = 2 };
enum class ControlSource : uint8_t { Remote = 0, Keyboard = 1 };
enum class ChassisMode : uint8_t { Separate = 0, Follow = 1, Spin = 2 };
enum class GimbalMode : uint8_t { Joint = 0 };

struct ChassisMotion {
  float vx = 0.0f;
  float vy = 0.0f;
  float w = 0.0f;
  ChassisMotion &operator+=(const ChassisMotion &rhs) { vx += rhs.vx; vy += rhs.vy; w += rhs.w; return *this; }
  ChassisMotion &operator-=(const ChassisMotion &rhs) { vx -= rhs.vx; vy -= rhs.vy; w -= rhs.w; return *this; }
};

inline ChassisMotion operator+(ChassisMotion lhs, const ChassisMotion &rhs) { return lhs += rhs; }
inline ChassisMotion operator-(ChassisMotion lhs, const ChassisMotion &rhs) { return lhs -= rhs; }
inline ChassisMotion operator*(ChassisMotion value, float scale) { value.vx *= scale; value.vy *= scale; value.w *= scale; return value; }

struct GimbalMotion {
  float yaw = 0.0f;
  float pitch = 0.0f;
  GimbalMotion &operator+=(const GimbalMotion &rhs) { yaw+=rhs.yaw; pitch+=rhs.pitch; return *this; }
  GimbalMotion &operator-=(const GimbalMotion &rhs) { yaw-=rhs.yaw; pitch-=rhs.pitch; return *this; }
};
inline GimbalMotion operator+(GimbalMotion lhs, const GimbalMotion &rhs) { return lhs+=rhs; }
inline GimbalMotion operator-(GimbalMotion lhs, const GimbalMotion &rhs) { return lhs-=rhs; }
inline GimbalMotion operator*(GimbalMotion v, float scale) { v.yaw*=scale; v.pitch*=scale; return v; }

inline float NormalizeAngle(float angle) {
  constexpr float kPi = 3.14159265358979323846f;
  if (!std::isfinite(angle)) return 0.0f;
  return std::remainder(angle, 2.0f * kPi);
}

}  // namespace robot

#endif /* MODULE_STATE_HPP_ */
