#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <deque>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>

typedef std::deque<std::pair<pcl::PointXYZRGB, ros::Time> > OBST_HISTORY;



void add_obstacle(const pcl::PointXYZRGB obs);

bool checkCollision(pcl::PointXYZRGB& collision_point);

#endif
