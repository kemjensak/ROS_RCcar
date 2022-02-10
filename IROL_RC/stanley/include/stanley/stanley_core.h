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

#ifndef PURE_PURSUIT_PURE_PURSUIT_CORE_H
#define PURE_PURSUIT_PURE_PURSUIT_CORE_H

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// User defined includes
#include <autoware_config_msgs/ConfigWaypointFollower.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_health_checker/health_checker/health_checker.h>
#include <stanley/stanley.h>
// #include <vector_map_msgs/PointArray.h>

// C++ includes
#include <vector>
#include <memory>
#include <string>

namespace waypoint_follower
{
  enum class Mode : int32_t
  {
    waypoint,
    dialog,
    unknown = -1,
  };

  template <class T>
  typename std::underlying_type<T>::type enumToInteger(T t)
  {
    return static_cast<typename std::underlying_type<T>::type>(t);
  }

  class StanleyNode
  {
  public:
    StanleyNode();
    ~StanleyNode() = default;

    void run();
    friend class StanleyNodeTestSuite;

  private:
    // handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    std::shared_ptr<autoware_health_checker::HealthChecker> health_checker_ptr_;

    // class
    Stanley st_;

    // publisher
    ros::Publisher pub1_, pub2_, pub11_, pub12_, pub13_, pub14_, pub15_, pub16_, pub17_, pub18_;

    // subscriber
    ros::Subscriber sub1_, sub2_, sub3_, sub4_, gpath_points_sub_;

    // control loop update rate
    double update_rate_;

    // variables
    bool add_virtual_end_waypoints_;
    bool is_waypoint_set_, is_pose_set_, is_velocity_set_;
    double current_linear_velocity_, command_linear_velocity_;
    int expand_size_;
    LaneDirection direction_;
    int32_t velocity_source_; // 0 = waypoint, 1 = Dialog
    double const_velocity_;   // km/h
    std::string output_interface_;
    double next_distance_threshold_;
    double control_gain_;
    double max_steer_;
    double max_cross_track_error_;
    autoware_msgs::Lane gpath_points_;

    // callbacks
    void callbackFromConfig(const autoware_config_msgs::ConfigWaypointFollowerConstPtr &config);
    void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
    void callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
    void callbackFromWayPoints(const autoware_msgs::LaneConstPtr &msg);
    void callbackGetGPoints(const autoware_msgs::LaneArrayConstPtr &ptr);

    // initializer
    void initForROS();

    // functions
    void publishControlCommands(const bool &can_get_curvature, const double &kappa) const;
    void publishTwistStamped(const bool &can_get_curvature, const double &kappa) const;
    void publishCtrlCmdStamped(const bool &can_get_curvature) const;
    void publishDeviationCurrentPosition(const geometry_msgs::Point &point,
                                         const std::vector<autoware_msgs::Waypoint> &waypoints) const;
    void connectVirtualLastWaypoints(autoware_msgs::Lane *expand_lane, LaneDirection direction);

    int getSgn() const;
    double getYaw(const geometry_msgs::Pose p) const;
    double computeCommandVelocity() const;
    double computeCommandAccel() const;
    double computeAngularGravity(double velocity, double kappa) const;
    double computeCommandSteeringAngle() const;
    double maxSteerNormalize(double angle) const;
    double maxCTENormalize(double cte) const;
    double distanceBetweenTwoPoints(const geometry_msgs::Point p1, const geometry_msgs::Point p2) const;
    int getClosestWaypointIdx(const geometry_msgs::Point current_pt) const;
    double OrientationBetweenTargetandCurrent(const geometry_msgs::Point target, const geometry_msgs::Point current, double referenceOrientation) const;
  };

  inline double kmph2mps(double velocity_kmph)
  {
    return (velocity_kmph * 1000) / (60 * 60);
  }

} // namespace waypoint_follower

#endif // PURE_PURSUIT_PURE_PURSUIT_CORE_H
