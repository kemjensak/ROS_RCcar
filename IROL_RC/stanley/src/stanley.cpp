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

#include <stanley/stanley.h>
#include <cmath>

namespace waypoint_follower
{
    double Stanley::calcCurvature(const geometry_msgs::Point &target) const
    {
        double kappa;
        const geometry_msgs::Point pt = calcRelativeCoordinate(target, current_pose_);
        const double denominator = pt.x * pt.x + pt.y * pt.y;
        const double numerator = 2.0 * pt.y;

        if (denominator != 0.0)
        {
            kappa = numerator / denominator;
        }
        else
        {
            kappa = numerator > 0.0 ? KAPPA_MIN_ : -KAPPA_MIN_;
        }

        return kappa;
    }

    void Stanley::getNextWaypoint()
    {
        const int path_size = static_cast<int>(current_waypoints_.size());
        // ROS_INFO("path_size: %f", path_size);

        // if waypoints are not given, do nothing.
        if (path_size == 0)
        {
            next_waypoint_number_ = -1;
            return;
        }

        // look for the next waypoint.
        for (int i = 0; i < path_size; i++)
        {
            // ROS_INFO("path_size_ 2: %d", path_size);
            // if search waypoint is the last
            if (i == (path_size - 1))
            {
                ROS_INFO("search waypoint is the last");
                next_waypoint_number_ = i;
                return;
            }

            // if there exists an effective waypoint
            if (getPlaneDistance(current_waypoints_.at(i).pose.pose.position, current_pose_.position) > next_distance_threshold_)
            {
                next_waypoint_number_ = i;
                ROS_INFO("next_waypoint_number_(i>th): %d", next_waypoint_number_);
                return;
            }
        }

        // if this program reaches here , it means we lost the waypoint!c
        next_waypoint_number_ = -1;
        return;
    }

    bool Stanley::canGetNextWaypoint(double *output_kappa)
    {
        // search closest waypoint
        getNextWaypoint();
        if (next_waypoint_number_ == -1)
        {
            ROS_INFO("lost next waypoint");
            return false;
        }
        next_target_pose_ = current_waypoints_.at(next_waypoint_number_).pose.pose;
        *output_kappa = calcCurvature(next_target_pose_.position);
        return true;
    }
} // namespace waypoint_follower
