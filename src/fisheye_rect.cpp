#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <carrt_goggles/DisparityConfig.h>
#include <pcl_ros/point_cloud.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> StereoSyncPolicy;

// Camera Parameters
cv::Mat K1, K2, R;
cv::Vec3d T;
cv::Vec4d D1, D2;
cv::Mat R1, R2, P1, P2, Q;

// Stereo Block Matchers
cv::Ptr<cv::StereoBM> block_matcher = cv::StereoBM::create(96, 21);
cv::Ptr<cv::StereoSGBM> sg_block_matcher = cv::StereoSGBM::create(0, 128, 21);

// Publishers
ros::Publisher point_cloud_pub;
image_transport::Publisher left_rect, right_rect;

int y_offset = 0;   // Offset of the right image. Set by dynamic reconfigure.
unsigned int sequence = 0;   // Point cloud header sequence
bool use_sgbm = true;

// Dynamic reconfigure callback
void reconfigCallback(carrt_goggles::DisparityConfig& config, uint32_t level)
{
    // Tweak all settings to be valid
    config.prefilter_size |= 0x1; // must be odd
    config.block_size |= 0x1; // must be odd
    config.num_disparities = (config.num_disparities / 16) * 16; // must be multiple of 16

    // Select Block Matcher
    use_sgbm = config.sgbm;
    if(use_sgbm)
    {
        // Update SG block matcher
        sg_block_matcher->setPreFilterCap(config.prefilter_cap);
        sg_block_matcher->setUniquenessRatio(config.uniqueness_ratio);

        sg_block_matcher->setBlockSize (config.block_size);
        sg_block_matcher->setDisp12MaxDiff (config.disp12MaxDiff);
        sg_block_matcher->setMinDisparity (config.min_disparity);
        sg_block_matcher->setNumDisparities (config.num_disparities);
        sg_block_matcher->setSpeckleRange (config.speckle_range);
        sg_block_matcher->setSpeckleWindowSize (config.speckle_window_size);

        sg_block_matcher->setP1(config.p1);
        sg_block_matcher->setP1(config.p2);
        sg_block_matcher->setMode(config.mode);
    }
    else
    {
        // Update block matcher
        block_matcher->setPreFilterCap(config.prefilter_cap);
        block_matcher->setPreFilterSize(config.prefilter_size);

        block_matcher->setSmallerBlockSize(config.smaller_block_size);
        block_matcher->setTextureThreshold(config.texture_threshold);
        block_matcher->setUniquenessRatio(config.uniqueness_ratio);

        block_matcher->setBlockSize (config.block_size);
        block_matcher->setDisp12MaxDiff (config.disp12MaxDiff);
        block_matcher->setMinDisparity (config.min_disparity);
        block_matcher->setNumDisparities (config.num_disparities);
        block_matcher->setSpeckleRange (config.speckle_range);
        block_matcher->setSpeckleWindowSize (config.speckle_window_size);
    }

    y_offset = config.y_offset; // Y-offset of right image

}

void disparity2PCL(pcl::PointCloud<pcl::PointXYZRGB>& point_cloud, const cv::Mat& disparity)
{
    // Fill in new PointCloud message (2D image-like layout)
    point_cloud.header.frame_id = "stereo";
    pcl_conversions::toPCL(ros::Time::now(), point_cloud.header.stamp);
    point_cloud.header.seq = sequence++;


    point_cloud.height = 1;
    point_cloud.is_dense = false; // there may be invalid points

    // Calculate point cloud
    cv::Mat points_mat;
    cv::reprojectImageTo3D(disparity, points_mat, Q, true);

    for (unsigned int u = 0; u < points_mat.rows; ++u)
    {
        for (unsigned int v = 0; v < points_mat.cols; ++v)
        {
            float z = points_mat.at<cv::Vec3f>(u, v)[2];
            if(z != 10000 && !std::isinf(z))
            {
                pcl::PointXYZRGB point;
                point.x = points_mat.at<cv::Vec3f>(u, v)[0]*25 - 2.5;
                point.y = points_mat.at<cv::Vec3f>(u, v)[1]*25 - 1.7;   // Average male eye-height
                point.z = z*30;
                point_cloud.points.push_back(point);
            }
        }
    }
    point_cloud.width = point_cloud.points.size();
}

void stereoCallback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right)
{
    // Parse ROS Message to opencv
    cv_bridge::CvImagePtr left_ptr;
    try {left_ptr = cv_bridge::toCvCopy(left, "bgr8");}
    catch (cv_bridge::Exception& e) { ROS_ERROR("%s", e.what()); return; }

    cv_bridge::CvImagePtr right_ptr;
    try {right_ptr = cv_bridge::toCvCopy(right, "bgr8");}
    catch (cv_bridge::Exception& e) { ROS_ERROR("%s", e.what()); return; }

    // Calculate Knew
    //cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K1, D1, left_ptr->image.size(), R1, P1, 0, left_ptr->image.size(), 1.0);
    //cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K2, D2, right_ptr->image.size(), R2, P2, 0, right_ptr->image.size(), 1.0);

    // Undistort
    cv::Mat m1, m2;
    cv::fisheye::initUndistortRectifyMap(K1,D1,R1,K1,left_ptr->image.size(), CV_32FC1, m1, m2);
    cv::remap(left_ptr->image, left_ptr->image, m1, m2, cv::INTER_LINEAR);

    cv::fisheye::initUndistortRectifyMap(K2,D2,R2,K2,right_ptr->image.size(), CV_32FC1, m1, m2);
    cv::remap(right_ptr->image, right_ptr->image, m1, m2, cv::INTER_LINEAR);

    // Translate
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, 0, 0, 1, y_offset);
    cv::warpAffine(right_ptr->image,right_ptr->image,trans_mat,right_ptr->image.size());

    // Publish
    left_rect.publish(left_ptr->toImageMsg());
    right_rect.publish(right_ptr->toImageMsg());

    // convert to grayscale
    cv::cvtColor(left_ptr->image, left_ptr->image, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_ptr->image, right_ptr->image, cv::COLOR_BGR2GRAY);

    // Run Stereo Block Matching
    cv::Mat disparity;
    if(use_sgbm)
        sg_block_matcher->compute(left_ptr->image, right_ptr->image, disparity);
    else
        block_matcher->compute(left_ptr->image, right_ptr->image, disparity);

    // Create and publish Point Cloud
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    disparity2PCL(point_cloud, disparity);
    point_cloud_pub.publish(point_cloud);

    // Update visualization windows
    normalize(disparity, disparity, 0, 255, CV_MINMAX, CV_8U);
    imshow("disparity", disparity);
}


int main(int argc, char **argv)
{
    // Read calibration info from yaml.
    std::string calib_path = ros::package::getPath("carrt_goggles") + "/data/cam_stereo.yml";
    cv::FileStorage fs(calib_path, cv::FileStorage::READ);
    fs["K1"] >> K1;
    fs["K2"] >> K2;
    fs["D1"] >> D1;
    fs["D2"] >> D2;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["R"] >> R;
    fs["P"] >> T;
    fs["Q"] >> Q;

    ros::init(argc, argv, "fisheye_rect");
    ros::NodeHandle nh;

    // Dynamic Reconfigure
    dynamic_reconfigure::Server<carrt_goggles::DisparityConfig> reconfig_server;
    reconfig_server.setCallback(boost::bind(&reconfigCallback, _1, _2));

    // Sync incoming images
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/stereo/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/stereo/right/image_raw", 1);
    message_filters::Synchronizer<StereoSyncPolicy> sync(StereoSyncPolicy(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&stereoCallback, _1, _2));

    // Point cloud publisher
    point_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("point_cloud", 1);

    // Image publishers
    image_transport::ImageTransport it(nh);
    left_rect = it.advertise("left/image_rect", 1);
    right_rect = it.advertise("right/image_rect", 1);

    // Visualization Windows
    cv::namedWindow("disparity");
    cv::startWindowThread();

    ros::Rate loop_rate(30);
    while (nh.ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
