#include "ugv_path_tracking/trajectory_utils.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ugv {
namespace path_tracking {
namespace trajectory {

double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double GetYaw(const geometry_msgs::Pose& pose) {
  const tf2::Quaternion quat(pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, pose.orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  (void)roll;
  (void)pitch;
  return yaw;
}

std::size_t FindClosestIndex(const ugv_msgs::Trajectory& trajectory,
                             const geometry_msgs::Point& position) {
  if (trajectory.points.empty()) {
    return 0U;
  }

  double best_dist = std::numeric_limits<double>::infinity();
  std::size_t best_index = 0U;
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto& pt = trajectory.points[i];
    const double dx = pt.x - position.x;
    const double dy = pt.y - position.y;
    const double dist = dx * dx + dy * dy;
    if (dist < best_dist) {
      best_dist = dist;
      best_index = i;
    }
  }
  return best_index;
}

bool InterpolateByDistance(const ugv_msgs::Trajectory& trajectory,
                           std::size_t start_index,
                           double target_distance,
                           ugv_msgs::TrajectoryPoint* out_point) {
  if (!out_point || trajectory.points.empty()) {
    return false;
  }
  const std::size_t safe_start = std::min(start_index, trajectory.points.size() - 1U);
  auto current = trajectory.points[safe_start];
  if (target_distance <= 0.0) {
    *out_point = current;
    return true;
  }

  double accumulated = 0.0;
  for (std::size_t i = safe_start; i + 1U < trajectory.points.size(); ++i) {
    const auto& p0 = trajectory.points[i];
    const auto& p1 = trajectory.points[i + 1U];
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double segment = std::hypot(dx, dy);
    if (segment < 1e-6) {
      continue;
    }
    if (accumulated + segment >= target_distance) {
      const double ratio = (target_distance - accumulated) / segment;
      ugv_msgs::TrajectoryPoint result = p0;
      result.x = p0.x + ratio * dx;
      result.y = p0.y + ratio * dy;
      result.heading = std::atan2(dy, dx);
      result.s = p0.s + ratio * (p1.s - p0.s);
      result.velocity = p0.velocity + ratio * (p1.velocity - p0.velocity);
      result.curvature = ComputeCurvature(p0, p1);
      *out_point = result;
      return true;
    }
    accumulated += segment;
  }

  *out_point = trajectory.points.back();
  return true;
}

double ComputeLateralError(const ugv_msgs::TrajectoryPoint& reference,
                            const geometry_msgs::Pose& pose) {
  const double dx = pose.position.x - reference.x;
  const double dy = pose.position.y - reference.y;
  return -std::sin(reference.heading) * dx + std::cos(reference.heading) * dy;
}

double ComputeHeadingError(const ugv_msgs::TrajectoryPoint& reference,
                            const geometry_msgs::Pose& pose) {
  const double yaw = GetYaw(pose);
  return NormalizeAngle(reference.heading - yaw);
}

double ComputeCurvature(const ugv_msgs::TrajectoryPoint& p0,
                         const ugv_msgs::TrajectoryPoint& p1) {
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  const double ds = std::hypot(dx, dy);
  if (ds < 1e-6) {
    return 0.0;
  }
  const double dheading = NormalizeAngle(p1.heading - p0.heading);
  return dheading / ds;
}

}  // namespace trajectory
}  // namespace path_tracking
}  // namespace ugv
