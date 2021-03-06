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

int close_obstacle_count = 0;

pcl::PassThrough<pcl::PointXYZRGB> x_filter, y_filter, z_filter;
pcl::VoxelGrid<pcl::PointXYZRGB> vox;
void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    // Filter out points more than 1 meter to each side
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>());
    x_filter.setInputCloud(msg);
    x_filter.setFilterFieldName("x");
    x_filter.setFilterLimits(-0.5, 0.5);
    x_filter.filter(*cloud3);

    // Filter out points more than 3 meters away
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>());
    y_filter.setInputCloud(cloud3);
    y_filter.setFilterFieldName("y");
    y_filter.setFilterLimits(-2, 0);
    y_filter.filter(*cloud2);

    // Filter out points below or more than 2m above the ground plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    z_filter.setInputCloud(cloud2);
    z_filter.setFilterFieldName("z");
    z_filter.setFilterLimits(0, 3.0);
    z_filter.filter(*cloud1);

    // Apply voxel-grid filter
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    vox.setInputCloud (cloud1);
    vox.setLeafSize (0.05f, 0.05f, 0.05f);
    vox.filter (*cloud);

    // Find K nearest points to the origin
    int K = 50;
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
                bool center_is_close = center.x > -0.25 && center.x < 0.25 && center.y > -2 && center.y < 0 && center.z < 0.7;
                if(center_is_close)
                    close_obstacle_count = std::min(close_obstacle_count + 4, 6);

                if(checkCollision(collision_point) || close_obstacle_count > 3)
                {
                    std_msgs::String warning_msg;
//                    warning_msg.data = boost::str(boost::format("%.1f") % center.z);
                    warning_msg.data = boost::lexical_cast<std::string>((int)(center.z * 10));
                    obst_warning_pub.publish(warning_msg);
                }
            }
        }
    }

    if (--close_obstacle_count < 0)
        close_obstacle_count = 0;

    ROS_INFO("Close obstacle count: %d", close_obstacle_count);

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
