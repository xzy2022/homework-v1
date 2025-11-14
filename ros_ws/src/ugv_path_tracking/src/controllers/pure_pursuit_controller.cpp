#include "ugv_path_tracking/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>

#include <ros/ros.h>

#include <ugv_path_tracking/trajectory_utils.hpp>

namespace ugv {
namespace path_tracking {

PurePursuitController::PurePursuitController()
    : ControllerBase("pure_pursuit"),
      wheel_base_(2.5),
      min_lookahead_(2.0),
      max_lookahead_(15.0),
      lookahead_gain_(0.5),
      throttle_gain_(0.6),
      brake_gain_(0.4),
      max_steer_angle_(0.6),
      feedforward_velocity_(0.0) {}

bool PurePursuitController::Configure(const ros::NodeHandle& nh) {
  nh.param("wheel_base", wheel_base_, wheel_base_);
  nh.param("min_lookahead", min_lookahead_, min_lookahead_);
  nh.param("max_lookahead", max_lookahead_, max_lookahead_);
  nh.param("lookahead_gain", lookahead_gain_, lookahead_gain_);
  nh.param("throttle_gain", throttle_gain_, throttle_gain_);
  nh.param("brake_gain", brake_gain_, brake_gain_);
  nh.param("max_steer_angle", max_steer_angle_, max_steer_angle_);
  nh.param("feedforward_velocity", feedforward_velocity_, feedforward_velocity_);
  return wheel_base_ > 0.1 && min_lookahead_ > 0.0 && max_lookahead_ > min_lookahead_;
}

ugv_msgs::ControlCommand PurePursuitController::ComputeCommand(
    const ugv_msgs::Trajectory& reference,
    const ugv_msgs::VehicleState& state) {
  auto cmd = CreateCommand(state);
  if (reference.points.empty()) {
    ROS_WARN_THROTTLE(1.0, "[PurePursuit] Empty reference trajectory");
    return cmd;
  }

  const std::size_t closest_index =
      trajectory::FindClosestIndex(reference, state.pose.position);
  const double speed = state.twist.linear.x;
  const double desired_lookahead = lookahead_gain_ * std::fabs(speed) + min_lookahead_;
  const double lookahead_distance =
      std::min(std::max(desired_lookahead, min_lookahead_), max_lookahead_);

  ugv_msgs::TrajectoryPoint lookahead_point;
  trajectory::InterpolateByDistance(reference, closest_index, lookahead_distance,
                                    &lookahead_point);

  const double yaw = trajectory::GetYaw(state.pose);
  const double alpha = trajectory::NormalizeAngle(lookahead_point.heading - yaw);
  const double steering =
      std::atan2(2.0 * wheel_base_ * std::sin(alpha),
                 std::max(lookahead_distance, 1e-3));
  cmd.steering_angle =
      std::min(std::max(steering, -max_steer_angle_), max_steer_angle_);

  const double target_velocity =
      std::max(lookahead_point.velocity, feedforward_velocity_);
  const double speed_error = target_velocity - speed;
  double accel_command = throttle_gain_ * speed_error;
  if (accel_command >= 0.0) {
    cmd.throttle = std::min(accel_command, 1.0);
    cmd.brake = 0.0;
  } else {
    cmd.throttle = 0.0;
    cmd.brake = std::min(-accel_command * brake_gain_, 1.0);
  }
  cmd.target_velocity = target_velocity;
  cmd.steering_rate =
      2.0 * speed * std::fabs(std::sin(alpha)) /
      std::max(lookahead_distance, 1e-3);
  return cmd;
}

}  // namespace path_tracking
}  // namespace ugv
