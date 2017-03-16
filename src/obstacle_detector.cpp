#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_listener.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher vis_pub;
tf::TransformListener* tf_sub = NULL;

// Define Colors
uint32_t red = ((uint32_t)255 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
uint32_t blue = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)255);

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    // Transform point cloud to world frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    tf_sub->waitForTransform("/stereo_camera", "/world", ros::Time::now(), ros::Duration(1.0));
    pcl_ros::transformPointCloud("/world", *msg, *cloud, *tf_sub);

    // Filter out points less than 4 inches or more than 2m above the ground plane
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(0.1016, 2.0);    // 4 inches = 0.1016 meters
    filter.filter(*cloud);

    // Filter out points more than 3 meters away
    filter.setFilterFieldName("y");
    filter.setFilterLimits(0, 3.0);
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

            // If a cluster is found, profile its velocity
            std::vector<pcl::PointIndices>::const_iterator cit = cluster_indices.begin();
            if (cit != cluster_indices.end())
            {
                for (std::vector<int>::const_iterator pit = cit->indices.begin(); pit != cit->indices.end(); ++pit)
                    cloud->points[*pit].rgb = *reinterpret_cast<float*>(&red);

            }
        }
    }

    // Visualize the origin with a blue point
    origin.rgb = *reinterpret_cast<float*>(&blue);
    cloud->points.push_back(origin);
    ++cloud->width;

    vis_pub.publish(cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detector");

    ros::NodeHandle n;
    tf_sub = new (tf::TransformListener);

    ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/points2", 1, pointCloudCallback);
    vis_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("obstacles", 1);

    ros::spin();

    return 0;
}
