#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <deque>

//std::deque<std::pair<pcl::PointXYZ> > history;

//void register_obstacle(const std::pair<pcl::PointXYZ, ros::Time>& obs)
//{
//    history.push_back(obs);
//    if(history.size() > 5)
//        history.pop_back();
//}

//pcl::PointXYZ checkCollision()
//{
//    // Create point cloud from history
//    pcl::PointCloud<pcl::PointXYZ> line_cloud;
//    for(std::deque<pcl::PointXYZ>::iterator it = history.begin(); it != history.end(); ++it)
//        line_cloud.points.push_back(*it);

//    // Calculate how many points fit a straight line
//    pcl::SampleConsensusModelLine<pcl::PointXY>::Ptr line_model;
//    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (line_model);
//    ransac.setDistanceThreshold(0.1);
//    ransac.computeModel();
//    std::vector<int> inlier_indices;
//    ransac.getInliers(inlier_indices);

//    // If the number of points is large enough
//    if(inlier_indices.size() >= 3)
//    {
//        // Get min and max indices (chronological order)
//        int min = inlier_indices.front(), max = inlier_indices.front();
//        for (int i = 0; i < inlier_indices.size(); ++i)
//        {
//            min = std::min(min, inlier_indices[i]);
//            max = std::max(max, inlier_indices[i]);
//        }

//        // Calculate vector

//    }
//}
