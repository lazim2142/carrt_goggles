#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_listener.h>

ros::Publisher vis_pub;
tf::TransformListener* tf_sub = NULL;

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    // Transform point cloud to world frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    tf_sub->waitForTransform("/stereo_camera", "/world", ros::Time::now(), ros::Duration(1.0));
    pcl_ros::transformPointCloud("/world", *msg, *cloud, *tf_sub);

    // Filter out points low to the ground and above 2m from the ground plane
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(0.1016, 2.0);
    filter.filter(*cloud);

    // Filter out points more than 3 meters away
    filter.setFilterFieldName("y");
    filter.setFilterLimits(0, 3.0);
    filter.filter(*cloud);

    if (cloud->width > 25)
    {
        // Filter Noise
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (100);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud);

        pcl::PointXYZ origin;
        origin.x = 0.0f;
        origin.y = 0.0f;
        origin.z = 0.0f;
        int K = 1;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        pcl::PointCloud<pcl::PointXYZ> nneighbors;
        nneighbors.header.frame_id = "/world";
        nneighbors.height = 1;
        nneighbors.width = K;

        if (kdtree.nearestKSearch(origin, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
                nneighbors.points.push_back(pcl::PointXYZ(cloud->points[ pointIdxNKNSearch[i] ].x,
                                                            cloud->points[ pointIdxNKNSearch[i] ].y,
                                                            cloud->points[ pointIdxNKNSearch[i] ].z));
        }

        vis_pub.publish(nneighbors.makeShared());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detector");

    ros::NodeHandle n;
    tf_sub = new (tf::TransformListener);

    ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/points2", 1, pointCloudCallback);
    vis_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("obstacles", 1);

    ros::spin();

    return 0;
}
