#include "../include/imu_to_odom/imu_to_odom.h"
#include "../include/imu_to_odom/imu_to_odom_predictor.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_to_odom_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  //OdomPredictor imu_to_odom(nh, nh_private);
  ImuToOdom imu_to_odom(nh, nh_private);

  ros::spin();

  return 0;
}
