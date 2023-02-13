#include "../include/imu_to_odom/imu_to_odom.h"

ImuToOdom::ImuToOdom(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      seq_(0),
      have_odom_(false),
      have_bias_(false) {

   constexpr size_t kROSQueueLength = 100;         

   imu_sub_ = nh_.subscribe("/imu/data", kROSQueueLength, &ImuToOdom::imuCallback, this);
   imu_bias_sub_ = nh_.subscribe("imu_bias", kROSQueueLength, &ImuToOdom::imuBiasCallback, this);

   odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>("imu_odometry", kROSQueueLength);
   transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
         "imu_transform", kROSQueueLength);

  geometry_msgs::Point pos;
  geometry_msgs::Pose pose;

  pos.x = 0; 
  pos.y = 0; 
  pos.z = 1.0;
  
  geometry_msgs::Quaternion quat;
  quat.x = 0;
  quat.y = 0;
  quat.z = 0;
  quat.w = 1.0;
  pose.orientation = quat;
  
  pose.position = pos;         

  tf::poseMsgToKindr(pose, &transform_);

  linear_velocity_ = {0, 0, 0};
  angular_velocity_ = {0, 0, 0};

  frame_id_ = "world";
  child_frame_id_ = "odom";   
}

void ImuToOdom::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
   if (msg->orientation_covariance[0] == -1.0) {
      have_orientation_ = false;
   }
   else {
      tf::quaternionMsgToKindr(msg->orientation, &orientation_);
      transform_.getRotation() = orientation_;
   }

   if (msg->header.stamp < imu_queue_.back().header.stamp) {
      ROS_ERROR_STREAM("Latest IMU message occured at time: "
                        << msg->header.stamp
                        << ". This is before the previously received IMU "
                           "message that ouccured at: "
                        << imu_queue_.back().header.stamp
                        << ". The current imu queue will be reset.");
      imu_queue_.clear();
   }

   imu_queue_.push_back(*msg);

   try {
      integrateIMUData(*msg);
   } catch (std::exception& e) {
      ROS_ERROR_STREAM(
         "IMU INTEGRATION FAILED, RESETING EVERYTHING: " << e.what());
      have_bias_ = false;
      have_odom_ = false;
      imu_queue_.clear();
      return;
   }

   publishOdometry();
   publishTF();
   ++seq_;
}      

void ImuToOdom::imuBiasCallback(const sensor_msgs::ImuConstPtr& msg) {
  tf::vectorMsgToKindr(msg->linear_acceleration,
                       &imu_linear_acceleration_bias_);
  tf::vectorMsgToKindr(msg->angular_velocity, &imu_angular_velocity_bias_);

  have_bias_ = true;
}

void ImuToOdom::integrateIMUData(const sensor_msgs::Imu& msg) {
  if (!has_imu_meas) {
    estimate_timestamp_ = msg.header.stamp;
    has_imu_meas = true;
    return;
  }

  double dt = (msg.header.stamp - estimate_timestamp_).toSec();

  const Vector3 kGravity(0.0, 0.0, -9.8);

  Vector3 imu_linear_acceleration, imu_angular_velocity;

  geometry_msgs::Vector3 linear_acceleration = msg.linear_acceleration;

  tf::vectorMsgToKindr(linear_acceleration, &imu_linear_acceleration);
  tf::vectorMsgToKindr(msg.angular_velocity, &imu_angular_velocity);

  const Vector3 final_angular_velocity =
      (imu_angular_velocity - imu_angular_velocity_bias_);

   /*
   orientation_ = orientation_ * tf::Quaternion(imu_angular_velocity.x() * dt,
                                                imu_angular_velocity.y() * dt,
                                                imu_angular_velocity.z() * dt);
   orientation_.normalize();
   */

   orientationQuat_.orientation = msg.orientation;

   // Integrate the linear acceleration data to get the position
   linear_velocity_.x() += linear_acceleration.x * dt;
   linear_velocity_.y() += linear_acceleration.y * dt;
   linear_velocity_.z() += linear_acceleration.z * dt;

   geometry_msgs::Point point_;
   point_.x += linear_velocity_.x() * dt;
   point_.y += linear_velocity_.y() * dt;
   point_.z += linear_velocity_.z() * dt;

   // Stores odom information
   odom_.header.stamp = msg.header.stamp;
   odom_.header.frame_id = "odom";
   odom_.pose.pose.position.x = point_.x;
   odom_.pose.pose.position.y = point_.y;
   odom_.pose.pose.position.z = point_.z;
   odom_.pose.pose.orientation.x = orientationQuat_.orientation.x;
   odom_.pose.pose.orientation.y = orientationQuat_.orientation.y;
   odom_.pose.pose.orientation.z = orientationQuat_.orientation.z;
   odom_.pose.pose.orientation.w = orientationQuat_.orientation.w;

   estimate_timestamp_ = msg.header.stamp;
}


void ImuToOdom::publishOdometry() {

   odom_.header.frame_id = frame_id_;
   odom_.header.seq = seq_;
   odom_.header.stamp = estimate_timestamp_;
   odom_.child_frame_id = child_frame_id_;

   tf::poseKindrToMsg(transform_, &odom_.pose.pose);
   //msg.pose.covariance = pose_covariance_;

   tf::vectorKindrToMsg(linear_velocity_, &odom_.twist.twist.linear);
   tf::vectorKindrToMsg(angular_velocity_, &odom_.twist.twist.angular);
   //msg.twist.covariance = twist_covariance_;

   odom_pub_.publish(odom_);
}

void ImuToOdom::publishTF() {
   geometry_msgs::TransformStamped msg;

   msg.header.frame_id = frame_id_;
   msg.header.seq = seq_;
   msg.header.stamp = estimate_timestamp_;
   msg.child_frame_id = child_frame_id_;

   tf::transformKindrToMsg(transform_, &msg.transform);

   transform_pub_.publish(msg);
   br_.sendTransform(msg);
}

