#include "ugv_path_tracking/stanley_controller.hpp"

#include <algorithm>
#include <cmath>

#include <ros/ros.h>

#include <ugv_path_tracking/trajectory_utils.hpp>

namespace ugv {
namespace path_tracking {

StanleyController::StanleyController()
    : ControllerBase("stanley"),
      wheel_base_(2.5),
      gain_(1.2),
      softening_speed_(0.2),
      throttle_gain_(0.5),
      brake_gain_(0.4),
      max_steer_angle_(0.6),
      min_target_speed_(1.0) {}

bool StanleyController::Configure(const ros::NodeHandle& nh) {
  nh.param("wheel_base", wheel_base_, wheel_base_);
  nh.param("gain", gain_, gain_);
  nh.param("softening_speed", softening_speed_, softening_speed_);
  nh.param("throttle_gain", throttle_gain_, throttle_gain_);
  nh.param("brake_gain", brake_gain_, brake_gain_);
  nh.param("max_steer_angle", max_steer_angle_, max_steer_angle_);
  nh.param("min_target_speed", min_target_speed_, min_target_speed_);
  return wheel_base_ > 0.0 && gain_ > 0.0;
}

ugv_msgs::ControlCommand StanleyController::ComputeCommand(
    const ugv_msgs::Trajectory& reference,
    const ugv_msgs::VehicleState& state) {
  auto cmd = CreateCommand(state);
  if (reference.points.empty()) {
    ROS_WARN_THROTTLE(1.0, "[Stanley] Empty reference trajectory");
    return cmd;
  }
  const std::size_t closest_index =
      trajectory::FindClosestIndex(reference, state.pose.position);
  const auto& reference_point = reference.points[closest_index];

  const double heading_error =
      trajectory::ComputeHeadingError(reference_point, state.pose);
  const double lateral_error =
      trajectory::ComputeLateralError(reference_point, state.pose);
  const double velocity = state.twist.linear.x;
  const double control_term = std::atan2(gain_ * lateral_error,
                                         velocity + softening_speed_);
  const double steering = heading_error + control_term;
  cmd.steering_angle =
      std::min(std::max(steering, -max_steer_angle_), max_steer_angle_);
  cmd.steering_rate = (velocity / std::max(wheel_base_, 1e-3)) * std::tan(cmd.steering_angle);

  const double target_velocity =
      std::max(reference_point.velocity, min_target_speed_);
  const double speed_error = target_velocity - velocity;
  double accel_command = throttle_gain_ * speed_error;
  if (accel_command >= 0.0) {
    cmd.throttle = std::min(accel_command, 1.0);
    cmd.brake = 0.0;
  } else {
    cmd.throttle = 0.0;
    cmd.brake = std::min(-accel_command * brake_gain_, 1.0);
  }
  cmd.target_velocity = target_velocity;
  return cmd;
}

}  // namespace path_tracking
}  // namespace ugv
