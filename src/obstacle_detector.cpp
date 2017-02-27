#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>

ros::Publisher vis_pub;
tf::TransformListener* tf_sub = NULL;

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    // Transform point cloud to world frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud("/world", *msg, *transformed_pcl, *tf_sub);

    // Filter out points low to the ground
    pcl::PassThrough<pcl::PointXYZ> low_filter;
    low_filter.setInputCloud(transformed_pcl);
    low_filter.setFilterFieldName("z");
    low_filter.setFilterLimits(-1.0, 1.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr high_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    low_filter.filter(*high_cloud);
    vis_pub.publish(high_cloud);

    if (high_cloud->points.size() > 0)
    {
        pcl::PointXYZ origin;
        origin.x = 0.0f;
        origin.y = 0.0f;
        origin.z = 0.0f;
        int K = 10;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(high_cloud);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        pcl::PointCloud<pcl::PointXYZ> nneighbors;
        nneighbors.header.frame_id = "/world";
        nneighbors.height = 1;
        nneighbors.width = K;

        if (kdtree.nearestKSearch(origin, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
                nneighbors.points.push_back(pcl::PointXYZ(high_cloud->points[ pointIdxNKNSearch[i] ].x,
                                                            high_cloud->points[ pointIdxNKNSearch[i] ].y,
                                                            high_cloud->points[ pointIdxNKNSearch[i] ].z));
        }

        //vis_pub.publish(nneighbors.makeShared());
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
