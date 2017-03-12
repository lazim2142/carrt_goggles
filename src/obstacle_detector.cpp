#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
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
    vox.setLeafSize (0.2f, 0.2f, 0.2f);
    vox.filter (*cloud);

    // Find K nearest points to the origin
    int K = 25;
    pcl::PointXYZRGB origin;    // defaults to (0,0,0)
    if (cloud->width > K)
    {        
        // KD tree to find nearest points
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(cloud);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_extractor;
        cluster_extractor.setClusterTolerance (0.05);
        cluster_extractor.setMinClusterSize (15);
        cluster_extractor.setMaxClusterSize (25);
        cluster_extractor.setSearchMethod (kdtree);
        cluster_extractor.setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        cluster_extractor.extract (cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
            j++;
        }

        if (kdtree.nearestKSearch(origin, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
               cloud->points[pointIdxNKNSearch[i]].rgb = *reinterpret_cast<float*>(&red);
    }

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
