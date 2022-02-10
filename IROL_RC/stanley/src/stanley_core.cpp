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

#include <vector>
#include <string>

#include <stanley/stanley_core.h>
#include <stanley/stanley_viz.h>

namespace waypoint_follower
{
    // Constructor
    StanleyNode::StanleyNode()
        : private_nh_("~"), st_(), update_rate_(30.0), is_waypoint_set_(false), is_pose_set_(false), is_velocity_set_(false), current_linear_velocity_(0), command_linear_velocity_(0), direction_(LaneDirection::Forward), velocity_source_(-1), const_velocity_(5.0)
    {
        initForROS();
        health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh_, private_nh_);
        health_checker_ptr_->ENABLE();
    }

    void StanleyNode::initForROS()
    {
        // ros parameter settings
        std::string out_twist, out_ctrl_cmd;
        private_nh_.param("velocity_source", velocity_source_, 0);
        private_nh_.param("add_virtual_end_waypoints", add_virtual_end_waypoints_, false);
        private_nh_.param("const_velocity", const_velocity_, 5.0);
        private_nh_.param("update_rate", update_rate_, 30.0);
        private_nh_.param("out_twist_name", out_twist, std::string("twist_raw"));
        private_nh_.param("out_ctrl_cmd_name", out_ctrl_cmd, std::string("ctrl_raw"));
        private_nh_.param("output_interface", output_interface_, std::string("ctrl_cmd"));
        private_nh_.param("next_distance_threshold", next_distance_threshold_, 0.5);
        private_nh_.param("control_gain", control_gain_, 0.1);
        private_nh_.param("max_steer", max_steer_, 1.0472);
        private_nh_.param("max_cross_track_error", max_cross_track_error_, 1.0472);

        // Output type, use old parameter name only if it is set
        if (private_nh_.hasParam("publishes_for_steering_robot"))
        {
            bool publishes_for_steering_robot;
            private_nh_.param(
                "publishes_for_steering_robot", publishes_for_steering_robot, false);
            if (publishes_for_steering_robot)
            {
                output_interface_ = "ctrl_cmd";
            }
            else
            {
                output_interface_ = "twist";
            }
        }
        else
        {
            private_nh_.param(
                "output_interface", output_interface_, std::string("all"));
        }

        if (output_interface_ != "twist" && output_interface_ != "ctrl_cmd" &&
            output_interface_ != "all")
        {
            ROS_ERROR("Control command interface type is not valid");
            ros::shutdown();
        }

        // setup subscriber
        sub1_ = nh_.subscribe("final_waypoints", 10, &StanleyNode::callbackFromWayPoints, this);
        sub2_ = nh_.subscribe("current_pose", 10, &StanleyNode::callbackFromCurrentPose, this);
        sub3_ = nh_.subscribe("config/waypoint_follower", 10, &StanleyNode::callbackFromConfig, this);
        sub4_ = nh_.subscribe("current_velocity", 10, &StanleyNode::callbackFromCurrentVelocity, this);
        gpath_points_sub_ = nh_.subscribe("lane_waypoints_array", 1, &StanleyNode::callbackGetGPoints, this);

        // setup publishers
        pub1_ = nh_.advertise<geometry_msgs::TwistStamped>(out_twist, 10);
        pub2_ = nh_.advertise<autoware_msgs::ControlCommandStamped>(out_ctrl_cmd, 10);
        pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
        pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
        pub13_ = nh_.advertise<visualization_msgs::Marker>("closest_waypoint_mark", 0);
        // debug tool
        pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0);
        pub15_ = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
        pub16_ = nh_.advertise<std_msgs::Float32>("angular_gravity", 0);
        pub17_ = nh_.advertise<std_msgs::Float32>("deviation_of_current_position", 0);
        pub18_ = nh_.advertise<visualization_msgs::Marker>("expanded_waypoints_mark", 0);
    }

    void StanleyNode::callbackGetGPoints(const autoware_msgs::LaneArrayConstPtr &ptr)
    {
        gpath_points_ = ptr->lanes[0];
    }

    void StanleyNode::run()
    {
        ros::Rate loop_rate(update_rate_);
        while (ros::ok())
        {
            ros::spinOnce();
            if (!is_pose_set_ || !is_waypoint_set_ || !is_velocity_set_)
            {
                if (!is_pose_set_)
                {
                    ROS_WARN_THROTTLE(5, "Waiting for current_pose topic ...");
                }
                if (!is_waypoint_set_)
                {
                    ROS_WARN_THROTTLE(5, "Waiting for final_waypoints topic ...");
                }
                if (!is_velocity_set_)
                {
                    ROS_WARN_THROTTLE(5, "Waiting for current_velocity topic ...");
                }

                loop_rate.sleep();
                continue;
            }

            st_.setNextDistanceThreshold(next_distance_threshold_);

            double kappa = 0;
            bool can_get_next_waypoint = st_.canGetNextWaypoint(&kappa);

            publishControlCommands(can_get_next_waypoint, kappa);
            health_checker_ptr_->NODE_ACTIVATE();
            health_checker_ptr_->CHECK_RATE("topic_rate_vehicle_cmd_slow", 8, 5, 1, "topic vehicle_cmd publish rate slow.");

            // for visualization with Rviz
            pub11_.publish(displayNextWaypoint(st_.getPoseOfNextWaypoint()));
            pub12_.publish(displayNextTarget(st_.getPoseOfNextTarget()));
            pub15_.publish(displayTrajectoryCircle(
                waypoint_follower::generateTrajectoryCircle(st_.getPointOfNextTarget(), st_.getCurrentPose())));
            if (add_virtual_end_waypoints_)
            {
                pub18_.publish(displayExpandWaypoints(st_.getCurrentWaypoints(), expand_size_));
            }
            std_msgs::Float32 angular_gravity_msg;
            angular_gravity_msg.data = computeAngularGravity(computeCommandVelocity(), kappa);
            pub16_.publish(angular_gravity_msg);

            publishDeviationCurrentPosition(st_.getCurrentPose().position, st_.getCurrentWaypoints());

            is_pose_set_ = false;
            is_velocity_set_ = false;

            loop_rate.sleep();
        }
    }

    void StanleyNode::publishControlCommands(const bool &can_get_next_waypoint, const double &kappa) const
    {
        if (output_interface_ == "twist")
        {
            publishTwistStamped(can_get_next_waypoint, kappa);
        }
        else if (output_interface_ == "ctrl_cmd")
        {
            publishCtrlCmdStamped(can_get_next_waypoint);
        }
        else if (output_interface_ == "all")
        {
            publishTwistStamped(can_get_next_waypoint, kappa);
            publishCtrlCmdStamped(can_get_next_waypoint);
        }
        else
        {
            ROS_WARN("[stanley] control command interface is not appropriate");
        }
    }

    void StanleyNode::publishTwistStamped(const bool &can_get_next_waypoint, const double &kappa) const
    {
        geometry_msgs::TwistStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.twist.linear.x = can_get_next_waypoint ? computeCommandVelocity() : 0;
        ts.twist.angular.z = can_get_next_waypoint ? kappa * ts.twist.linear.x : 0;
        pub1_.publish(ts);
    }

    void StanleyNode::publishCtrlCmdStamped(const bool &can_get_next_waypoint) const
    {
        autoware_msgs::ControlCommandStamped ccs;
        ccs.header.stamp = ros::Time::now();
        ccs.cmd.linear_velocity = can_get_next_waypoint ? computeCommandVelocity() : 0;
        ccs.cmd.linear_acceleration = can_get_next_waypoint ? computeCommandAccel() : 0;
        ccs.cmd.steering_angle = can_get_next_waypoint ? computeCommandSteeringAngle() : 0;
        pub2_.publish(ccs);
    }

    int StanleyNode::getSgn() const
    {
        int sgn = 0;
        if (direction_ == LaneDirection::Forward)
        {
            sgn = 1;
        }
        else if (direction_ == LaneDirection::Backward)
        {
            sgn = -1;
        }
        return sgn;
    }

    double StanleyNode::getYaw(const geometry_msgs::Pose p) const
    {
        tf::Quaternion qt(
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w);
        tf::Matrix3x3 m(qt);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw; // deg
    }

    double StanleyNode::computeCommandVelocity() const
    {
        if (velocity_source_ == enumToInteger(Mode::dialog))
        {
            return getSgn() * kmph2mps(const_velocity_);
        }

        return command_linear_velocity_;
    }

    // Assume constant acceleration motion, v_f^2 - v_i^2 = 2 * a * delta_d
    double StanleyNode::computeCommandAccel() const
    {
        const geometry_msgs::Pose current_pose = st_.getCurrentPose();
        const geometry_msgs::Pose target_pose = st_.getCurrentWaypoints().at(1).pose.pose;

        const double delta_d =
            std::hypot(target_pose.position.x - current_pose.position.x, target_pose.position.y - current_pose.position.y);
        const double v_i = current_linear_velocity_;
        const double v_f = computeCommandVelocity();
        return (v_f * v_f - v_i * v_i) / (2 * delta_d);
    }

    double pi2pi(double angle)
    {
        if (angle > M_PI)
        {
            return angle - 2.0 * M_PI;
        }
        if (angle < -M_PI)
        {
            return angle + 2.0 * M_PI;
        }
        return angle;
    }

    double StanleyNode::maxSteerNormalize(double angle) const
    {
        if (angle > max_steer_)
        {
            return max_steer_;
        }
        if (angle < -max_steer_)
        {
            return -max_steer_;
        }
    }

    double StanleyNode::maxCTENormalize(double cte) const
    {
        if (cte > max_cross_track_error_)
        {
            return max_cross_track_error_;
        }
        if (cte < -1 * max_cross_track_error_)
        {
            return -1 * max_cross_track_error_;
        }
        ROS_INFO("NOT NORMALIZED !");
        return cte;
    }

    double StanleyNode::distanceBetweenTwoPoints(const geometry_msgs::Point p1, const geometry_msgs::Point p2) const
    {
        double dist = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
        return dist;
    }

    int StanleyNode::getClosestWaypointIdx(const geometry_msgs::Point current_pt) const
    {
        double dist, min_dist = 9999999;
        int min_idx, idx = 0;
        for (const autoware_msgs::Waypoint &p : gpath_points_.waypoints)
        {
            dist = distanceBetweenTwoPoints(p.pose.pose.position, current_pt);
            if (dist < min_dist)
            {
                min_dist = dist;
                min_idx = idx;
            }
            idx++;
        }
        return min_idx;
    }

    double StanleyNode::OrientationBetweenTargetandCurrent(const geometry_msgs::Point target, const geometry_msgs::Point current, double referenceOrientation) const
    {
        double xError = target.x - current.x;
        double yError = target.y - current.y;
        double orientationError = atan2(yError, xError);
        double orientation = referenceOrientation - orientationError; // deg
        ROS_INFO("referenceOrientation: %f, orientationError: %f", referenceOrientation, orientationError);
        return orientation;
    }

    double StanleyNode::computeCommandSteeringAngle() const
    {
        const geometry_msgs::Pose current_pose = st_.getCurrentPose();
        const geometry_msgs::Pose target_pose = st_.getPoseOfNextTarget();
        // 1) heading_error
        // Euler
        const double target_heading = getYaw(target_pose); // deg
        ROS_INFO("target_heading: %f", target_heading);
        const double current_heading = getYaw(current_pose);     // deg
        double heading_error = target_heading - current_heading; // rad
        // Quaternion
        // const double target_heading = 0, current_heading = 0;
        // double heading_error = getRelativeAngle(target_pose, current_pose);

        // ROS_INFO("heading_error[deg]: %f", heading_error);
        // heading_error = heading_error * M_PI / 180;
        // ROS_INFO("heading_error[rad]: %f, target_heading: %f, current_heading: %f", heading_error, target_heading, current_heading);
        // heading_error = maxSteerNormalize(heading_error);

        // 2) cross_track_error
        int closest_wp = getClosestWaypointIdx(current_pose.position);
        // ROS_INFO("closest_wp: %d", closest_wp);
        geometry_msgs::Pose closest_wp_pose = gpath_points_.waypoints[closest_wp].pose.pose;
        pub13_.publish(displayClosestWaypoint(closest_wp_pose));

        // double cross_track_error = 0;
        double cross_track_error = getPlaneDistance(current_pose.position, closest_wp_pose.position);
        ROS_INFO("cross_track_error: %f", cross_track_error);
        cross_track_error = maxCTENormalize(cross_track_error);
        ROS_INFO("cross_track_error[Normalize]: %f", cross_track_error);

        double orientation = OrientationBetweenTargetandCurrent(target_pose.position, current_pose.position, target_heading);

        if (orientation > 0) // car is on the left
        {
            cross_track_error *= -1;
            ROS_INFO("cross_track_error[Orientation]: %f", cross_track_error);
        }

        /*  y = tan(theta)x + b
            b = y - tab(theta)x     */
        // double target_b = target_pose.position.y - tan(target_heading * M_PI / 180) * target_pose.position.x;
        // double current_b = current_pose.position.y - tan(target_heading * M_PI / 180) * current_pose.position.x;

        // if (target_b > current_b)
        // {
        //     cross_track_error *= -1;
        //     ROS_INFO("cross_track_error[sign]: %f", cross_track_error);
        // }
        // if (target_heading / current_heading > 0)
        // { // different sign
        //     heading_error *= -1;
        // }
        ROS_INFO("heading_error: %f + %f", heading_error, atan(control_gain_ * cross_track_error / current_linear_velocity_));
        double steering = 0;
        if (current_linear_velocity_ < 0.5)
        {
            steering = heading_error + atan(control_gain_ * cross_track_error / 0.5);
        }
        else
        {
            steering = heading_error + atan(control_gain_ * cross_track_error / current_linear_velocity_);
        }
        return steering;
    }

    double StanleyNode::computeAngularGravity(double velocity, double kappa) const
    {
        const double gravity = 9.80665;
        return (velocity * velocity) / (1.0 / kappa * gravity);
    }

    void StanleyNode::callbackFromConfig(const autoware_config_msgs::ConfigWaypointFollowerConstPtr &config)
    {
        velocity_source_ = config->param_flag;
        const_velocity_ = config->velocity;
    }

    void StanleyNode::publishDeviationCurrentPosition(const geometry_msgs::Point &point,
                                                      const std::vector<autoware_msgs::Waypoint> &waypoints) const
    {
        // Calculate the deviation of current position from the waypoint approximate line
        if (waypoints.size() < 3)
        {
            return;
        }

        const geometry_msgs::Point end = waypoints.at(2).pose.pose.position;
        const geometry_msgs::Point start = waypoints.at(1).pose.pose.position;

        const tf::Vector3 p_A(start.x, start.y, 0.0);
        const tf::Vector3 p_B(end.x, end.y, 0.0);
        const tf::Vector3 p_C(point.x, point.y, 0.0);

        // The distance form a point C to a line passing through A and B is given by
        // length(AB.crossProduct(AC))/length(AC)
        const tf::Vector3 AB = p_B - p_A;
        const tf::Vector3 AC = p_C - p_A;
        const float distance = (AB.cross(AC)).length() / AC.length();

        std_msgs::Float32 msg;
        msg.data = distance;
        pub17_.publish(msg);
    }

    void StanleyNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        st_.setCurrentPose(msg);
        is_pose_set_ = true;
    }

    void StanleyNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
    {
        current_linear_velocity_ = msg->twist.linear.x;
        st_.setCurrentVelocity(current_linear_velocity_);
        is_velocity_set_ = true;
    }

    void StanleyNode::callbackFromWayPoints(const autoware_msgs::LaneConstPtr &msg)
    {
        command_linear_velocity_ = (!msg->waypoints.empty()) ? msg->waypoints.at(0).twist.twist.linear.x : 0;
        if (add_virtual_end_waypoints_)
        {
            const LaneDirection solved_dir = getLaneDirection(*msg);
            direction_ = (solved_dir != LaneDirection::Error) ? solved_dir : direction_;
            autoware_msgs::Lane expanded_lane(*msg);
            expand_size_ = -expanded_lane.waypoints.size();
            connectVirtualLastWaypoints(&expanded_lane, direction_);
            expand_size_ += expanded_lane.waypoints.size();

            st_.setCurrentWaypoints(expanded_lane.waypoints);
        }
        else
        {
            st_.setCurrentWaypoints(msg->waypoints);
        }
        is_waypoint_set_ = true;
    }

    void StanleyNode::connectVirtualLastWaypoints(autoware_msgs::Lane *lane, LaneDirection direction)
    {
        if (lane->waypoints.empty())
        {
            return;
        }
        static double interval = 1.0;
        const geometry_msgs::Pose &pn = lane->waypoints.back().pose.pose;
        autoware_msgs::Waypoint virtual_last_waypoint;
        virtual_last_waypoint.pose.pose.orientation = pn.orientation;
        virtual_last_waypoint.twist.twist.linear.x = 0.0;
        geometry_msgs::Point virtual_last_point_rlt;
        const int sgn = getSgn();
        for (double dist = st_.getNextDistanceThreshold(); dist > 0.0; dist -= interval)
        {
            virtual_last_point_rlt.x += interval * sgn;
            virtual_last_waypoint.pose.pose.position = calcAbsoluteCoordinate(virtual_last_point_rlt, pn);
            lane->waypoints.emplace_back(virtual_last_waypoint);
        }
    }

} // namespace waypoint_follower
