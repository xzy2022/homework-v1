#include <ros/ros.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <ugv_msgs/Trajectory.h>
#include <ugv_msgs/TrajectoryPoint.h>

namespace {
struct Point2D {
  double x;
  double y;
};
}

class GlobalPathNode {
 public:
  GlobalPathNode()
      : nh_(), private_nh_("~"), publish_rate_(10.0),
        sample_resolution_(0.5), path_type_("circle"),
        trajectory_frame_("map"), nominal_velocity_(5.0),
        circle_radius_(15.0), figure_eight_radius_(15.0),
        sine_amplitude_(5.0), sine_wavelength_(20.0),
        sine_length_(80.0) {
    private_nh_.param("path_type", path_type_, path_type_);
    private_nh_.param("publish_rate", publish_rate_, publish_rate_);
    private_nh_.param("sample_resolution", sample_resolution_, sample_resolution_);
    private_nh_.param("trajectory_frame", trajectory_frame_, trajectory_frame_);
    private_nh_.param("nominal_velocity", nominal_velocity_, nominal_velocity_);
    private_nh_.param("circle_radius", circle_radius_, circle_radius_);
    private_nh_.param("figure_eight_radius", figure_eight_radius_, figure_eight_radius_);
    private_nh_.param("sine_amplitude", sine_amplitude_, sine_amplitude_);
    private_nh_.param("sine_wavelength", sine_wavelength_, sine_wavelength_);
    private_nh_.param("sine_length", sine_length_, sine_length_);
    private_nh_.param("path_csv", path_csv_, path_csv_);

    trajectory_pub_ = nh_.advertise<ugv_msgs::Trajectory>("/planning/global_path", 1, true);
    BuildTrajectory();
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                             &GlobalPathNode::OnTimer, this);
  }

 private:
  void OnTimer(const ros::TimerEvent&) {
    if (trajectory_.points.empty()) {
      return;
    }
    trajectory_.header.frame_id = trajectory_frame_;
    trajectory_.header.stamp = ros::Time::now();
    trajectory_pub_.publish(trajectory_);
  }

  void BuildTrajectory() {
    std::vector<Point2D> raw_points;
    if (path_type_ == "circle") {
      raw_points = GenerateCircle();
    } else if (path_type_ == "figure8") {
      raw_points = GenerateFigureEight();
    } else if (path_type_ == "sine") {
      raw_points = GenerateSine();
    } else if (path_type_ == "csv" && !path_csv_.empty()) {
      raw_points = LoadFromCsv(path_csv_);
    } else {
      ROS_WARN("[GlobalPath] Unknown path_type %s, fallback to circle", path_type_.c_str());
      raw_points = GenerateCircle();
    }

    if (raw_points.empty()) {
      ROS_ERROR("[GlobalPath] Failed to generate reference path");
      return;
    }

    trajectory_.points.clear();
    trajectory_.points.reserve(raw_points.size());
    double accumulated_s = 0.0;
    for (std::size_t i = 0; i < raw_points.size(); ++i) {
      ugv_msgs::TrajectoryPoint point;
      point.x = raw_points[i].x;
      point.y = raw_points[i].y;
      point.velocity = nominal_velocity_;
      point.relative_time = 0.0;
      point.curvature = 0.0;
      point.heading = 0.0;
      point.s = accumulated_s;
      trajectory_.points.emplace_back(point);
      if (i + 1U < raw_points.size()) {
        const double dx = raw_points[i + 1U].x - raw_points[i].x;
        const double dy = raw_points[i + 1U].y - raw_points[i].y;
        accumulated_s += std::hypot(dx, dy);
      }
    }

    for (std::size_t i = 0; i + 1U < trajectory_.points.size(); ++i) {
      const auto& current = trajectory_.points[i];
      const auto& next = trajectory_.points[i + 1U];
      trajectory_.points[i].heading = std::atan2(next.y - current.y, next.x - current.x);
      trajectory_.points[i].curvature = ComputeCurvature(i);
      trajectory_.points[i].relative_time = current.s / std::max(nominal_velocity_, 0.1);
    }
    if (!trajectory_.points.empty()) {
      if (trajectory_.points.size() > 1U) {
        trajectory_.points.back().heading =
            trajectory_.points[trajectory_.points.size() - 2U].heading;
      } else {
        trajectory_.points.back().heading = 0.0;
      }
      trajectory_.points.back().curvature = 0.0;
      trajectory_.points.back().relative_time =
          trajectory_.points.back().s / std::max(nominal_velocity_, 0.1);
    }
    ROS_INFO("[GlobalPath] Generated %zu points (%s)", trajectory_.points.size(),
             path_type_.c_str());
  }

  double ComputeCurvature(std::size_t index) const {
    if (index + 2U >= trajectory_.points.size()) {
      return 0.0;
    }
    const auto& p0 = trajectory_.points[index];
    const auto& p1 = trajectory_.points[index + 1U];
    const auto& p2 = trajectory_.points[index + 2U];
    const double a = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const double b = std::hypot(p2.x - p1.x, p2.y - p1.y);
    const double c = std::hypot(p2.x - p0.x, p2.y - p0.y);
    const double s = (a + b + c) / 2.0;
    const double area = std::max(s * (s - a) * (s - b) * (s - c), 0.0);
    if (area < 1e-9 || a < 1e-6 || b < 1e-6 || c < 1e-6) {
      return 0.0;
    }
    const double radius = (a * b * c) / (4.0 * std::sqrt(area));
    return (radius > 1e-6) ? 1.0 / radius : 0.0;
  }

  std::vector<Point2D> GenerateCircle() const {
    const double circumference = 2.0 * M_PI * circle_radius_;
    const int samples = std::max(10, static_cast<int>(circumference / sample_resolution_));
    std::vector<Point2D> points;
    points.reserve(samples);
    for (int i = 0; i < samples; ++i) {
      const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples);
      points.push_back(Point2D{circle_radius_ * std::cos(theta),
                               circle_radius_ * std::sin(theta)});
    }
    points.push_back(points.front());
    return points;
  }

  std::vector<Point2D> GenerateFigureEight() const {
    const double total_angle = 2.0 * M_PI;
    const int samples = std::max(50, static_cast<int>(total_angle * figure_eight_radius_ /
                                                      sample_resolution_));
    std::vector<Point2D> points;
    points.reserve(samples);
    for (int i = 0; i < samples; ++i) {
      const double t = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples);
      const double x = figure_eight_radius_ * std::sin(t);
      const double y = figure_eight_radius_ * std::sin(t) * std::cos(t);
      points.push_back(Point2D{x, y});
    }
    points.push_back(points.front());
    return points;
  }

  std::vector<Point2D> GenerateSine() const {
    const int samples =
        std::max(10, static_cast<int>(sine_length_ / sample_resolution_));
    std::vector<Point2D> points;
    points.reserve(samples);
    for (int i = 0; i < samples; ++i) {
      const double x = i * sample_resolution_;
      const double angle = 2.0 * M_PI * x / sine_wavelength_;
      const double y = sine_amplitude_ * std::sin(angle);
      points.push_back(Point2D{x, y});
    }
    return points;
  }

  std::vector<Point2D> LoadFromCsv(const std::string& path) const {
    std::vector<Point2D> points;
    std::ifstream file(path);
    if (!file.is_open()) {
      ROS_ERROR_STREAM("[GlobalPath] Failed to open csv: " << path);
      return points;
    }
    std::string line;
    while (std::getline(file, line)) {
      if (line.empty()) {
        continue;
      }
      double x = 0.0;
      double y = 0.0;
      if (sscanf(line.c_str(), "%lf,%lf", &x, &y) == 2) {
        points.push_back(Point2D{x, y});
      }
    }
    return points;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher trajectory_pub_;
  ros::Timer timer_;

  double publish_rate_;
  double sample_resolution_;
  std::string path_type_;
  std::string trajectory_frame_;
  std::string path_csv_;
  double nominal_velocity_;
  double circle_radius_;
  double figure_eight_radius_;
  double sine_amplitude_;
  double sine_wavelength_;
  double sine_length_;

  ugv_msgs::Trajectory trajectory_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_path_node");
  GlobalPathNode node;
  ros::spin();
  return 0;
}
