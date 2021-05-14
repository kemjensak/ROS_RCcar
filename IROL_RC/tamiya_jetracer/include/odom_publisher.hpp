#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<string>
#include<cmath>
#include<gazebo_msgs/LinkStates.h>
#include<geometry_msgs/Twist.h>
#include<vector>
#include<iostream>
#include <string>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <tamiya_jetracer/vesc_feedback.h>

namespace mobile_robot_odometry
{
    class MobileRobotOdomety
    {
        public:
            MobileRobotOdomety();
            ~MobileRobotOdomety();

            void storeFeedback(const tamiya_jetracer::vesc_feedback::ConstPtr &ptr);
            //void calcWheelVelocityGazeboCB(const gazebo_msgs::LinkStates::ConstPtr& ptr);

            void boardcastTransform();
            void pubTF();

        private:
            ros::NodeHandle nh;
            ros::NodeHandle private_nh;

            ros::Subscriber sub;
            ros::Publisher odomPub;

            std::string base_link_id, odom_link_id, wheel_1_id, wheel_3_id;
            double separation_length;
            double delta_x, delta_y, delta_th;
            double v, vx, vy, vth;
            double x, y, th;
            int seq;
            tf::TransformBroadcaster br;

            double velocity;
            double wheel_1_velocity;
            double wheel_3_velocity;
            ros::Time last_time;

            double R = 1.0;
            double meter_per_pulse = 0.004166666;
            double meter_per_rotate = 0.205;
            double wheel_rpm_per_speed = 116.144/9;
            double ms_per_speed = (meter_per_rotate * wheel_rpm_per_speed) / 60;
            double speed_per_ms = 1 / ms_per_speed;
            double wheelbase = 0.257;

            double delta_tacho, tacho, last_tacho, steer;


    };
}