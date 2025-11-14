#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ugv_msgs/Trajectory.h>

#include <cstddef>

namespace ugv {
namespace path_tracking {
namespace trajectory {

double NormalizeAngle(double angle);

double GetYaw(const geometry_msgs::Pose& pose);

std::size_t FindClosestIndex(const ugv_msgs::Trajectory& trajectory,
                             const geometry_msgs::Point& position);

bool InterpolateByDistance(const ugv_msgs::Trajectory& trajectory,
                           std::size_t start_index,
                           double target_distance,
                           ugv_msgs::TrajectoryPoint* out_point);

double ComputeLateralError(const ugv_msgs::TrajectoryPoint& reference,
                            const geometry_msgs::Pose& pose);

double ComputeHeadingError(const ugv_msgs::TrajectoryPoint& reference,
                            const geometry_msgs::Pose& pose);

double ComputeCurvature(const ugv_msgs::TrajectoryPoint& p0,
                         const ugv_msgs::TrajectoryPoint& p1);

}  // namespace trajectory
}  // namespace path_tracking
}  // namespace ugv
