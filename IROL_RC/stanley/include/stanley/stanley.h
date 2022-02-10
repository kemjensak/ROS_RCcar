/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef STANLEY_STANLEY_H
#define STANLEY_STANLEY_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// C++ includes
#include <vector>

// User defined includes
#include <autoware_msgs/Lane.h>
#include <libwaypoint_follower/libwaypoint_follower.h>

namespace waypoint_follower
{
  class Stanley
  {
  public:
    Stanley() = default;
    ~Stanley() = default;

    // for setting data
    void setCurrentVelocity(const double &cur_vel)
    {
      current_linear_velocity_ = cur_vel;
    }
    void setCurrentWaypoints(const std::vector<autoware_msgs::Waypoint> &wps)
    {
      current_waypoints_ = wps;
    }
    void setCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
    {
      current_pose_ = msg->pose;
    }
    void setNextDistanceThreshold(const double &dist)
    {
      next_distance_threshold_ = dist;
    }

    // for debug on ROS
    geometry_msgs::Pose getPoseOfNextWaypoint() const
    {
      return current_waypoints_.at(next_waypoint_number_).pose.pose;
    }
    geometry_msgs::Pose getPoseOfNextTarget() const
    {
      return next_target_pose_;
    }
    geometry_msgs::Point getPointOfNextTarget() const
    {
      return next_target_pose_.position;
    }
    geometry_msgs::Pose getCurrentPose() const
    {
      return current_pose_;
    }
    std::vector<autoware_msgs::Waypoint> getCurrentWaypoints() const
    {
      return current_waypoints_;
    }
    double getNextDistanceThreshold() const
    {
      return next_distance_threshold_;
    }

    // processing
    bool canGetNextWaypoint(double *output_kappa);

  private:
    // constant
    static constexpr double RADIUS_MAX_ = 9e10;
    static constexpr double KAPPA_MIN_ = 1.0 / 9e10;

    // variables
    int next_waypoint_number_{-1};
    double next_distance_threshold_{};
    double current_linear_velocity_{0.0};
    geometry_msgs::Pose current_pose_{};
    geometry_msgs::Pose next_target_pose_{};
    std::vector<autoware_msgs::Waypoint> current_waypoints_{};

    // functions
    double calcCurvature(const geometry_msgs::Point &target) const;
    void getNextWaypoint();
    // double AngleBetwe enTwoAngles(const double &a1, const double &a2);
    // bool isForwardWaypoint(geometry_msgs::Pose p, geometry_msgs::Pose base_pose);
    // int getClosestWaypointNumber(const autoware_msgs::Lane &curr_path, geometry_msgs::Pose curr_pose);
  };
} // namespace waypoint_follower

#endif // STANLEY_STANLEY_H
