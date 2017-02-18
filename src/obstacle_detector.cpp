#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO("Received Point Cloud");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_detector");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/points2", 1, pointCloudCallback);

  ros::spin();

  return 0;
}
