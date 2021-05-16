#include <odom_publisher.hpp>

namespace mobile_robot_odometry
{
    MobileRobotOdomety::MobileRobotOdomety()
	: private_nh("~")
    , seq(0)
	, x(0.0)
	, y(0.0)
	, th(0.0)
    {
        sub = nh.subscribe("/vesc_feedback", 100, &MobileRobotOdomety::storeFeedback, this);
        odomPub = nh.advertise<nav_msgs::Odometry>("/scanmatch_odom",100);
        base_link_id = "/base_link";
        odom_link_id = "/scanmatch_odom";
        // while((!private_nh.getParam("base_link_id", base_link_id))|(!private_nh.getParam("odom_link_id", odom_link_id))) ros::Duration(0.5).sleep();
        // if(!private_nh.getParam("base_link_id", base_link_id)) throw std::runtime_error("set base_link_id");
        // if(!private_nh.getParam("odom_link_id", odom_link_id)) throw std::runtime_error("set odom_link_id");
        // if(!private_nh.getParam("leftwheel_linkname", wheel_1_id)) throw std::runtime_error("set wheel_1_id");
        // if(!private_nh.getParam("rightwheel_linkname", wheel_3_id)) throw std::runtime_error("set wheel_3_id");
        // if(!private_nh.getParam("separation_length", separation_length)) throw std::runtime_error("set separation_length");
    
        last_time = ros::Time::now();
        last_tacho = 0;
    }
    
    MobileRobotOdomety::~MobileRobotOdomety()
    {
        
    }

    void MobileRobotOdomety::storeFeedback(const tamiya_jetracer::vesc_feedback::ConstPtr &ptr){
        tacho = ptr->tacho.data;
        steer = ptr->steer.data;
    }

    void MobileRobotOdomety::boardcastTransform(){

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x,y,0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, th);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_link_id, base_link_id)); // linking base_link 
    }

    void MobileRobotOdomety::pubTF(){
        nav_msgs::Odometry odom;
        double dt = (ros::Time::now() - last_time).toSec();
        delta_tacho = tacho - last_tacho;
        
        v = (delta_tacho * meter_per_pulse) / dt;
        vth = (v / wheelbase) * tan(steer);
        vx = v * cos(th);
        vy = v * sin(th);

        delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        odom.header.seq             = seq++;
        odom.header.stamp           = last_time;
        odom.header.frame_id        = odom_link_id;
        odom.child_frame_id         = base_link_id; 
        odom.pose.pose.position.x   = x;
        odom.pose.pose.position.y   = y;
        odom.pose.pose.position.z   = 0;
        odom.pose.pose.orientation  = tf::createQuaternionMsgFromYaw(th) ;
        odom.twist.twist.linear.x   = vx;
        odom.twist.twist.linear.y   = vy;
        odom.twist.twist.angular.z  = vth;

        odomPub.publish(odom);
        boardcastTransform();
        last_time = ros::Time::now();
        last_tacho = tacho;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mobile_robot_odometry");
    mobile_robot_odometry::MobileRobotOdomety MRO;
    ros::Rate loop_rate(10); // dt is always 100ms

    while(ros::ok()){
        ros::spinOnce();
        MRO.pubTF();
        loop_rate.sleep();
    }
    return 0;
}
