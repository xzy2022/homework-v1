#include <ros/ros.h>

#include <cmath>
#include <string>

#include <ugv_msgs/ControlCommand.h>
#include <ugv_msgs/TrackingPerformance.h>
#include <ugv_msgs/Trajectory.h>
#include <ugv_msgs/VehicleState.h>

#include "ugv_path_tracking/trajectory_utils.hpp"

class TrackingEvaluatorNode {
 public:
  TrackingEvaluatorNode()
      : nh_(), private_nh_("~"), evaluation_period_(5.0) {
    private_nh_.param("trajectory_topic", trajectory_topic_, std::string("/planning/global_path"));
    private_nh_.param("vehicle_state_topic", vehicle_state_topic_, std::string("/vehicle/state"));
    private_nh_.param("command_topic", command_topic_, std::string("/vehicle/pure_pursuit_cmd"));
    private_nh_.param("evaluation_period", evaluation_period_, evaluation_period_);
    private_nh_.param("controller_name", controller_name_, controller_name_);

    performance_pub_ =
        nh_.advertise<ugv_msgs::TrackingPerformance>("/tracking/performance", 10);
    trajectory_sub_ =
        nh_.subscribe(trajectory_topic_, 1, &TrackingEvaluatorNode::OnTrajectory, this);
    state_sub_ = nh_.subscribe(vehicle_state_topic_, 1, &TrackingEvaluatorNode::OnState, this);
    command_sub_ = nh_.subscribe(command_topic_, 1, &TrackingEvaluatorNode::OnCommand, this);
    timer_ = nh_.createTimer(ros::Duration(evaluation_period_),
                             &TrackingEvaluatorNode::OnTimer, this);
  }

 private:
  void OnTrajectory(const ugv_msgs::TrajectoryConstPtr& msg) {
    trajectory_ = *msg;
    has_trajectory_ = true;
  }

  void OnState(const ugv_msgs::VehicleStateConstPtr& msg) {
    latest_state_ = *msg;
    has_state_ = true;
    if (!has_trajectory_ || trajectory_.points.empty()) {
      return;
    }

    const std::size_t closest_index = ugv::path_tracking::trajectory::FindClosestIndex(
        trajectory_, latest_state_.pose.position);
    const auto& reference_point = trajectory_.points[closest_index];
    const double lateral_error =
        ugv::path_tracking::trajectory::ComputeLateralError(reference_point,
                                                            latest_state_.pose);
    const double heading_error =
        ugv::path_tracking::trajectory::ComputeHeadingError(reference_point,
                                                            latest_state_.pose);

    sum_lateral_error_sq_ += lateral_error * lateral_error;
    sum_heading_error_sq_ += heading_error * heading_error;
    sum_velocity_ += latest_state_.twist.linear.x;
    ++state_samples_;

    if (!window_active_) {
      window_active_ = true;
      window_start_stamp_ = latest_state_.header.stamp;
    }
    window_end_stamp_ = latest_state_.header.stamp;
  }

  void OnCommand(const ugv_msgs::ControlCommandConstPtr& msg) {
    if (msg->controller_name.size() > 0U) {
      controller_name_ = msg->controller_name;
    }

    const double effort = std::fabs(msg->steering_angle) + std::fabs(msg->throttle) +
                          std::fabs(msg->brake);
    sum_control_effort_ += effort;
    ++control_samples_;

    if (has_previous_command_) {
      const double delta = std::fabs(msg->steering_angle - previous_command_.steering_angle);
      sum_smoothness_ += delta;
    }
    previous_command_ = *msg;
    has_previous_command_ = true;
  }

  void OnTimer(const ros::TimerEvent&) {
    if (!window_active_ || state_samples_ == 0U) {
      return;
    }

    ugv_msgs::TrackingPerformance performance;
    performance.header.stamp = ros::Time::now();
    performance.controller_name = controller_name_;
    performance.rms_lateral_error =
        std::sqrt(sum_lateral_error_sq_ / static_cast<double>(state_samples_));
    performance.rms_heading_error =
        std::sqrt(sum_heading_error_sq_ / static_cast<double>(state_samples_));
    performance.mean_velocity = sum_velocity_ / static_cast<double>(state_samples_);
    performance.control_effort = (control_samples_ > 0)
                                     ? sum_control_effort_ / control_samples_
                                     : 0.0;
    performance.smoothness = (control_samples_ > 1)
                                 ? sum_smoothness_ / static_cast<double>(control_samples_ - 1)
                                 : 0.0;
    performance.lap_time = (window_end_stamp_ - window_start_stamp_).toSec();

    performance_pub_.publish(performance);
    ResetWindow();
  }

  void ResetWindow() {
    sum_lateral_error_sq_ = 0.0;
    sum_heading_error_sq_ = 0.0;
    sum_velocity_ = 0.0;
    sum_control_effort_ = 0.0;
    sum_smoothness_ = 0.0;
    state_samples_ = 0U;
    control_samples_ = 0U;
    window_active_ = false;
    has_previous_command_ = false;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher performance_pub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber command_sub_;
  ros::Timer timer_;

  std::string trajectory_topic_;
  std::string vehicle_state_topic_;
  std::string command_topic_;
  std::string controller_name_ = "controller";
  double evaluation_period_;

  ugv_msgs::Trajectory trajectory_;
  ugv_msgs::VehicleState latest_state_;
  ugv_msgs::ControlCommand previous_command_;
  bool has_trajectory_ = false;
  bool has_state_ = false;
  bool window_active_ = false;
  bool has_previous_command_ = false;
  ros::Time window_start_stamp_;
  ros::Time window_end_stamp_;

  double sum_lateral_error_sq_ = 0.0;
  double sum_heading_error_sq_ = 0.0;
  double sum_velocity_ = 0.0;
  double sum_control_effort_ = 0.0;
  double sum_smoothness_ = 0.0;
  std::size_t state_samples_ = 0U;
  std::size_t control_samples_ = 0U;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "tracking_evaluator_node");
  TrackingEvaluatorNode node;
  ros::spin();
  return 0;
}
