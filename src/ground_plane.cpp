#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <visualization_msgs/Marker.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>

ros::Publisher obst_cluster_pub;
ros::Publisher model_pub;
bool visualize = true;

tf::Transform ros_tf;   // Transform to publish


void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (msg);

    // Coefficients of plane equation ax + by + cz + d = 0
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Indices of points in the plane
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    seg.segment (*inliers, *coefficients);

    // Output error if the plane found is less than 1/3 of the point cloud
    if (inliers->indices.size() < msg->points.size()/3.0)
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    else
    {
        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];
        if (visualize)
        {
            // Publish plane visualization
            visualization_msgs::Marker plane_points;
            plane_points.type = visualization_msgs::Marker::SPHERE_LIST;
            plane_points.header.frame_id = "/stereo_camera";
            plane_points.header.stamp = ros::Time::now();
            plane_points.id = 0;
            plane_points.color.r = 0.0f;
            plane_points.color.g = 1.0f;
            plane_points.color.b = 0.0f;
            plane_points.color.a = 1.0;
            plane_points.scale.x = 0.02;
            plane_points.scale.y = 0.02;
            plane_points.scale.z = 0.02;
            plane_points.action = visualization_msgs::Marker::ADD;

            for(float x = -5; x < 5; x += 0.05)
            {
                for(float y = 0; y < 5; y += 0.05)
                {
                    float z = (-a*x - b*y - d)/c;
                    geometry_msgs::Point p;
                    p.x = x;
                    p.y = y;
                    p.z = z;
                    plane_points.points.push_back(p);
                }
            }

            obst_cluster_pub.publish(plane_points);
        }

        // Publish coefficients
        pcl_msgs::ModelCoefficients ros_coeff;
        pcl_conversions::fromPCL(*coefficients, ros_coeff);
        model_pub.publish(ros_coeff);

        // Calculate angle between the point-cloud ground plane and the world xy plane
        Eigen::Matrix<double, 1, 3> pcl_plane_normal, world_xy_normal, rotation_vector;
        pcl_plane_normal[0] = a;
        pcl_plane_normal[1] = b;
        pcl_plane_normal[2] = c;
        world_xy_normal[0] = 0.0;
        world_xy_normal[1] = 0.0;
        world_xy_normal[2] = 1.0;

        rotation_vector = (pcl_plane_normal.cross(world_xy_normal)).normalized();
        double theta = acos(pcl_plane_normal.dot(world_xy_normal)/sqrt(a*a + b*b + c*c));

        // Use the angle to create an affine transform
        Eigen::Affine3d tf_ground_plane = Eigen::Affine3d::Identity();
        tf_ground_plane.rotate(Eigen::AngleAxisd(theta, rotation_vector));

        // Apply the affine transform to the ground plane to determine translation
        Eigen::Vector4d original, transformed;
        original[0] = a*d;
        original[1] = b*d;
        original[2] = c*d;
        original[3] = 1.0;
        transformed = tf_ground_plane * original;
        tf_ground_plane.translation() << transformed[0], transformed[1], transformed[2];

        // Convert Eigen Affine transform to a ROS transform
        tf::transformEigenToTF(tf_ground_plane, ros_tf);
    }

    // Publish ROS transform
    static tf::TransformBroadcaster transform_br;
    transform_br.sendTransform(tf::StampedTransform(ros_tf, ros::Time::now(), "world", "stereo_camera"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_plane");

    ros::NodeHandle n;

    ros::param::get("~visualize", visualize);

    ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/points2", 1, pointCloudCallback);
    model_pub = n.advertise<pcl_msgs::ModelCoefficients> ("ground_plane/model_coefficients", 1);
    if(visualize)
        obst_cluster_pub = n.advertise<visualization_msgs::Marker> ("ground_plane/visual_markers", 1);

    ros::spin();

    return 0;
}
