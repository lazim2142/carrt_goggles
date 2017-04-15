#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <pcl/segmentation/extract_clusters.h>
#include "collision_detector.h"
#include "std_msgs/String.h"

ros::Publisher obst_cluster_pub;
ros::Publisher obst_vector_pub;
ros::Publisher obst_warning_pub;

// Define Colors
uint32_t red = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
uint32_t green = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
uint32_t blue = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Filter out points below or more than 2m above the ground plane
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    filter.setInputCloud(msg);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(0.0, 2.5);
    filter.filter(*cloud);

    // Filter out points more than 3 meters away
    filter.setFilterFieldName("z");
    filter.setFilterLimits(0, 3.0);
    filter.filter(*cloud);

    // Filter out points more than 1 meter to each side
    filter.setFilterFieldName("x");
    filter.setFilterLimits(-1, 1);
    filter.filter(*cloud);

    // Apply voxel-grid filter
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud (cloud);
    vox.setLeafSize (0.05f, 0.05f, 0.05f);
    vox.filter (*cloud);

    // Find K nearest points to the origin
    int K = 100;
    pcl::PointXYZRGB origin;    // defaults to (0,0,0)
    if (cloud->width > K)
    {        
        // KD tree to find nearest points
        pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>(false));
        kdtree->setInputCloud(cloud);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        // If at least K nearest points have been found
        if (kdtree->nearestKSearch(origin, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            // Filter out other points
            pcl::ExtractIndices<pcl::PointXYZRGB> eifilter;
            eifilter.setInputCloud(cloud);
            pcl::PointIndices::Ptr nearest_points (new pcl::PointIndices());
            nearest_points->indices = pointIdxNKNSearch;
            eifilter.setIndices(nearest_points);
            eifilter.setNegative(false);
            eifilter.filter(*cloud);

            // Create Clustering object and set parameters
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_extractor;
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            tree->setInputCloud (cloud);
            cluster_extractor.setInputCloud(cloud);
            cluster_extractor.setClusterTolerance (0.15f);
            cluster_extractor.setMinClusterSize (0.5*K + 1);    // Ensures at most only one cluster is found
            cluster_extractor.setMaxClusterSize (K);
            cluster_extractor.setSearchMethod (tree);

            // Extract Clusters
            std::vector<pcl::PointIndices> cluster_indices;
            cluster_extractor.extract(cluster_indices);

            // If a cluster is found
            std::vector<pcl::PointIndices>::const_iterator cit = cluster_indices.begin();
            if (cit != cluster_indices.end())
            {
                // Calculate cluster center
                pcl::PointXYZRGB center;
                for (std::vector<int>::const_iterator pit = cit->indices.begin(); pit != cit->indices.end(); ++pit)
                {
                    cloud->points[*pit].rgb = *reinterpret_cast<float*>(&red);
                    center.x += cloud->points[*pit].x;
                    center.y += cloud->points[*pit].y;
                    center.z += cloud->points[*pit].z;
                }
                center.x /= cit->indices.size();
                center.y /= cit->indices.size();
                center.z /= cit->indices.size();

                // Pass cluster center into collision detector
                add_obstacle(center);
                pcl::PointXYZRGB collision_point;
                if(checkCollision(collision_point))
                {
                    std_msgs::String warning_msg;
                    warning_msg.data = boost::str(boost::format("%.1f") % center.z);
                    obst_warning_pub.publish(warning_msg);
                    //sound_play_client_ptr->say(warning_msg.data);
                }
            }
        }
    }

    // Visualize the origin with a blue point
    cloud->points.push_back(origin);
    ++cloud->width;
    obst_cluster_pub.publish(cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("point_cloud", 1, pointCloudCallback);
    obst_cluster_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("obstacles/cluster", 1);
    obst_vector_pub = n.advertise<visualization_msgs::Marker> ("obstacles/vector", 1);
    obst_warning_pub = n.advertise<std_msgs::String> ("obstacles/warning", 1);

    ros::spin();

    return 0;
}
