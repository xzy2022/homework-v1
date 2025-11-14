#include <ros/ros.h>

#include <memory>
#include <string>

#include <ugv_msgs/ControlCommand.h>
#include <ugv_msgs/Trajectory.h>
#include <ugv_msgs/VehicleState.h>

#include "ugv_path_tracking/pure_pursuit_controller.hpp"

class PurePursuitNode {
 public:
  PurePursuitNode()
      : nh_(), private_nh_("~"), control_period_(0.02) {
    controller_ = std::make_unique<ugv::path_tracking::PurePursuitController>();

    private_nh_.param("trajectory_topic", trajectory_topic_, std::string("/planning/global_path"));
    private_nh_.param("vehicle_state_topic", vehicle_state_topic_, std::string("/vehicle/state"));
    private_nh_.param("command_topic", command_topic_, std::string("/vehicle/pure_pursuit_cmd"));
    private_nh_.param("control_period", control_period_, control_period_);

    if (!controller_->Configure(private_nh_)) {
      ROS_WARN("[PurePursuitNode] Failed to configure controller with provided parameters");
    }

    command_pub_ = nh_.advertise<ugv_msgs::ControlCommand>(command_topic_, 10);
    trajectory_sub_ =
        nh_.subscribe(trajectory_topic_, 1, &PurePursuitNode::OnTrajectory, this);
    state_sub_ = nh_.subscribe(vehicle_state_topic_, 1, &PurePursuitNode::OnState, this);
    timer_ = nh_.createTimer(ros::Duration(control_period_), &PurePursuitNode::OnTimer, this);
  }

 private:
  void OnTrajectory(const ugv_msgs::TrajectoryConstPtr& msg) {
    latest_trajectory_ = *msg;
    has_trajectory_ = true;
  }

  void OnState(const ugv_msgs::VehicleStateConstPtr& msg) {
    latest_state_ = *msg;
    has_state_ = true;
  }

  void OnTimer(const ros::TimerEvent&) {
    if (!has_trajectory_ || !has_state_) {
      return;
    }
    auto command = controller_->ComputeCommand(latest_trajectory_, latest_state_);
    command.header.stamp = ros::Time::now();
    command_pub_.publish(command);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher command_pub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber state_sub_;
  ros::Timer timer_;

  std::unique_ptr<ugv::path_tracking::PurePursuitController> controller_;
  ugv_msgs::Trajectory latest_trajectory_;
  ugv_msgs::VehicleState latest_state_;
  bool has_trajectory_ = false;
  bool has_state_ = false;

  std::string trajectory_topic_;
  std::string vehicle_state_topic_;
  std::string command_topic_;
  double control_period_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_pursuit_node");
  PurePursuitNode node;
  ros::spin();
  return 0;
}
