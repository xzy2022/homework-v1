#pragma once

#include <ugv_path_tracking/controller_base.hpp>

namespace ugv {
namespace path_tracking {

class PurePursuitController : public ControllerBase {
 public:
  PurePursuitController();

  bool Configure(const ros::NodeHandle& nh) override;

  ugv_msgs::ControlCommand ComputeCommand(const ugv_msgs::Trajectory& reference,
                                          const ugv_msgs::VehicleState& state) override;

 private:
  double wheel_base_;
  double min_lookahead_;
  double max_lookahead_;
  double lookahead_gain_;
  double throttle_gain_;
  double brake_gain_;
  double max_steer_angle_;
  double feedforward_velocity_;
};

}  // namespace path_tracking
}  // namespace ugv
