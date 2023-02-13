#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

double wheel_separation, wheel_radius;
double x, y, theta;
double vx, vy, vtheta;
ros::Time current_time, last_time;

void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    double vr = cmd_msg->linear.x + cmd_msg->angular.z * wheel_separation / 2.0;
    double vl = cmd_msg->linear.x - cmd_msg->angular.z * wheel_separation / 2.0;
    vx = (vr + vl) / 2.0;
    vtheta = (vr - vl) / wheel_separation;
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    x += vx * cos(theta) * dt;
    y += vx * sin(theta) * dt;
    theta += vtheta * dt;
    last_time = current_time;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "two_wheel_odom");
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 10, cmd_callback);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    tf::TransformBroadcaster odom_broadcaster;

    nh.param("wheel_separation", wheel_separation, 0.3);
    nh.param("wheel_radius", wheel_radius, 0.1);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
        odom_broadcaster.sendTransform(odom_trans);
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = x
        odom.pose.pose.orientation.y = orientation.y();
        odom.pose.pose.orientation.z = orientation.z();
        odom.pose.pose.orientation.w = orientation.w();
        odom_pub.publish(odom);
    }
}