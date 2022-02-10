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
        odomPub = nh.advertise<nav_msgs::Odometry>("/vesc/odom",100);
        velPub = nh.advertise<geometry_msgs::TwistStamped>("/current_velocity",100);
        private_nh.getParam("meter_per_pulse", meter_per_pulse);
        private_nh.getParam("wheelbase", wheelbase);
        base_link_id = "base_link";
        odom_link_id = "odom";    
        last_time = ros::Time::now();
        last_tacho = 0;
    }
    
    MobileRobotOdomety::~MobileRobotOdomety()
    {
        
    }

    void MobileRobotOdomety::storeFeedback(const rccar_jetson::vesc_feedback::ConstPtr &ptr){
        tacho = ptr->tacho.data;
        steer = ptr->steer.data;
        pubTF();
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
        geometry_msgs::TwistStamped vel;

        dt = (ros::Time::now() - last_time).toSec();
        delta_tacho = tacho - last_tacho;
        
        v = (delta_tacho * meter_per_pulse) / dt;

        vth = (v / wheelbase) * tan(steer);
        vx = v * cos(th);
        vy = v * sin(th);
        delta_x = v * dt * cos(th);
        delta_y = v * dt * sin(th);
        // (changed) wrt map to base_link 
        // delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        // delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        
        //th += delta_th;

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
        odom.twist.covariance = {0.03,0,0,0,0,0, 0,0.03,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.5445};

        vel.header.seq             = seq;
        vel.header.stamp           = last_time;
        vel.header.frame_id        = base_link_id;
        vel.twist.linear.x         = vx;
        vel.twist.linear.y         = vy;
        vel.twist.linear.z         = 0;
        vel.twist.angular.x        = 0;
        vel.twist.angular.y        = 0;
        vel.twist.angular.z        = vth;


        odomPub.publish(odom);
        velPub.publish(vel);
        // boardcastTransform();
        last_time = ros::Time::now();
        last_tacho = tacho;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mobile_robot_odometry");
    mobile_robot_odometry::MobileRobotOdomety MRO;
    ros::Rate loop_rate(20); // dt is always 100ms

    while(ros::ok()){
        ros::spin();
        
    }
    return 0;
}
