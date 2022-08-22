#ifndef DIFF_DRIVE__SPEED_LIMITER_HPP_
#define DIFF_DRIVE__SPEED_LIMITER_HPP_

#include <cmath>

namespace diff_drive
{
class SpeedLimiter
{
public:
  SpeedLimiter(bool has_velocity_limits = false, bool has_acceleration_limits = false, bool has_jerk_limits = false,
               double min_velocity = NAN, double max_velocity = NAN, double min_acceleration = NAN,
               double max_acceleration = NAN, double min_jerk = NAN, double max_jerk = NAN);

  double limit(double& v, double v0, double v1, double dt);

  double limit_velocity(double& v);

  double limit_acceleration(double& v, double v0, double dt);

  double limit_jerk(double& v, double v0, double v1, double dt);

private:
  // Enable/Disable velocity/acceleration/jerk limits:
  bool has_velocity_limits_;
  bool has_acceleration_limits_;
  bool has_jerk_limits_;

  // Velocity limits:
  double min_velocity_;
  double max_velocity_;

  // Acceleration limits:
  double min_acceleration_;
  double max_acceleration_;

  // Jerk limits:
  double min_jerk_;
  double max_jerk_;
};

}  // namespace diff_drive

#endif  // DIFF_DRIVE__SPEED_LIMITER_HPP_
