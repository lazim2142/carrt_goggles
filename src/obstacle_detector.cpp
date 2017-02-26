#include <ros/ros.h>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    // Filter out points low to the ground
    pcl::PassThrough<pcl::PointXYZ> low_filter;
    low_filter.setInputCloud(msg);
    low_filter.setFilterFieldName("z");
    low_filter.setFilterLimits(-10.0, 0.20);

    pcl::PointCloud<pcl::PointXYZ>::Ptr high_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    low_filter.filter(*high_cloud);

    if (high_cloud->points.size() > 0)
    {
        ROS_INFO("FOUND FILTERED POINTS");
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
        nneighbors.header.frame_id = msg->header.frame_id;
        nneighbors.height = 1;
        nneighbors.width = K;

        if (kdtree.nearestKSearch(origin, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
                nneighbors.points.push_back(pcl::PointXYZ(high_cloud->points[ pointIdxNKNSearch[i] ].x,
                                                            high_cloud->points[ pointIdxNKNSearch[i] ].y,
                                                            high_cloud->points[ pointIdxNKNSearch[i] ].z));
        }

        pub.publish(nneighbors.makeShared());
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_detector");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/points2", 1, pointCloudCallback);
  pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("obstacles", 1);

  ros::spin();

  return 0;
}
