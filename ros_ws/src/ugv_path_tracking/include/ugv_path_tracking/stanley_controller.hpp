#pragma once

#include <ugv_path_tracking/controller_base.hpp>

namespace ugv {
namespace path_tracking {

class StanleyController : public ControllerBase {
 public:
  StanleyController();

  bool Configure(const ros::NodeHandle& nh) override;

  ugv_msgs::ControlCommand ComputeCommand(const ugv_msgs::Trajectory& reference,
                                          const ugv_msgs::VehicleState& state) override;

 private:
  double wheel_base_;
  double gain_;
  double softening_speed_;
  double throttle_gain_;
  double brake_gain_;
  double max_steer_angle_;
  double min_target_speed_;
};

}  // namespace path_tracking
}  // namespace ugv
