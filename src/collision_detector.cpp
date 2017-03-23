#include "../include/collision_detector.h"

OBST_HISTORY obst_history;

extern uint32_t blue;
extern ros::Publisher obst_vector_pub;

void publishArrow(float x1, float y1, float z1, float x2, float y2, float z2)
{
    // Create Line List for visualization
    visualization_msgs::Marker  arrow;
    arrow.header.frame_id = "/stereo";
    arrow.header.stamp = ros::Time::now();
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.id = 0;
    arrow.pose.orientation.w = 1.0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.color.b = 1.0;
    arrow.color.a = 1.0;
    arrow.ns = "obstacle_vector";

    geometry_msgs::Point p;
    p.x = x1; p.y = y1; p.z = z1;
    arrow.points.push_back(p);
    p.x = x2; p.y = y2; p.z = z2;
    arrow.points.push_back(p);
    obst_vector_pub.publish(arrow);
}

void add_obstacle(const pcl::PointXYZRGB obs)
{
    obst_history.push_back(std::make_pair(obs, ros::Time::now()));
    if (obst_history.size() > 5)
        obst_history.pop_front();
}

bool checkCollision(pcl::PointXYZRGB& collision_point)
{
    if(obst_history.size() < 3)
        return false;

    // Create point cloud from history
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr line_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(OBST_HISTORY::iterator it = obst_history.begin(); it != obst_history.end(); ++it)
        line_cloud->points.push_back(it->first);

    // Calculate how many points fit a straight line
    pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr
            line_model (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB>(line_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (line_model);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();
    std::vector<int> inlier_indices;
    ransac.getInliers(inlier_indices);

    // If the number of points is large enough
    if(inlier_indices.size() >= 3)
    {
        std::cout << "Found enough inliers" << std::endl;
        // Get latest and earliest inlier indexes
        int earliest_idx = inlier_indices.front(), latest_idx = inlier_indices.front();
        for (int i = 0; i < inlier_indices.size(); ++i)
        {
            if(obst_history[earliest_idx].second.toSec() > obst_history[i].second.toSec())
                earliest_idx = i;
            if(obst_history[latest_idx].second.toSec() < obst_history[i].second.toSec())
                latest_idx = i;
        }

        // Calculate the motion vector's intersection point with the xy plane
        float x1 = obst_history[earliest_idx].first.x, x2 = obst_history[latest_idx].first.x;
        float y1 = obst_history[earliest_idx].first.y, y2 = obst_history[latest_idx].first.y;
        float z1 = obst_history[earliest_idx].first.z, z2 = obst_history[latest_idx].first.z;


        Eigen::Matrix4f numerator, denominator;
        numerator <<    1, 1, 1,  1,
                        0, 0, 0, x1,
                        0, 1, 0, y1,
                        1, 0, 0, z1;

        denominator <<  1, 1, 1, 0,
                        0, 0, 0, x2-x1,
                        0, 1, 0, y2-y1,
                        1, 0, 0, z2-z1;

        float t = -numerator.determinant()/denominator.determinant();

        collision_point.x = x1 + (x2 - x1)*t;
        collision_point.y = y1 + (y2 - y1)*t;
        collision_point.z = z1 + (z2 - z1)*t;

        publishArrow(x1, y1, z1, collision_point.x, collision_point.y, collision_point.z);

        // Check if intercept is within region of interest
        if (collision_point.x > -0.5 && collision_point.x < 0.5
        &&  collision_point.y > 0 && collision_point.y < 2)
        {
            std::cout << "Collision Detected" << std::endl;
            return true;
        }
        return true;
    }
    return false;
}
