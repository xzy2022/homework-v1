#pragma once

#include <ros/ros.h>

#include <string>

#include <ugv_msgs/ControlCommand.h>
#include <ugv_msgs/Trajectory.h>
#include <ugv_msgs/VehicleState.h>

namespace ugv {
namespace path_tracking {

class ControllerBase {
 public:
  explicit ControllerBase(std::string name) : controller_name_(std::move(name)) {}
  virtual ~ControllerBase() = default;

  virtual bool Configure(const ros::NodeHandle& nh) = 0;

  virtual ugv_msgs::ControlCommand ComputeCommand(
      const ugv_msgs::Trajectory& reference,
      const ugv_msgs::VehicleState& state) = 0;

  const std::string& name() const { return controller_name_; }

 protected:
  ugv_msgs::ControlCommand CreateCommand(const ugv_msgs::VehicleState& state) const {
    ugv_msgs::ControlCommand cmd;
    cmd.header.stamp = state.header.stamp;
    cmd.controller_name = controller_name_;
    cmd.target_velocity = 0.0;
    cmd.brake = 0.0;
    cmd.steering_angle = 0.0;
    cmd.steering_rate = 0.0;
    cmd.throttle = 0.0;
    return cmd;
  }

 private:
  std::string controller_name_;
};

}  // namespace path_tracking
}  // namespace ugv
