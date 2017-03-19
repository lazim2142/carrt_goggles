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
#include <carrt_goggles/disparityConfig.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> StereoSyncPolicy;

// Camera Parameters
cv::Mat K1, K2, R;
cv::Vec3d T;
cv::Vec4d D1, D2;
cv::Mat R1, R2, P1, P2, Q;

// Stereo Block Matcher
cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(96, 21);

// Dynamic reconfigure callback
void reconfigCallback(carrt_goggles::disparityConfig& config, uint32_t level)
{
    // Tweak all settings to be valid
    config.prefilter_size |= 0x1; // must be odd
    config.block_size |= 0x1; // must be odd
    config.num_disparities = (config.num_disparities / 16) * 16; // must be multiple of 16

    // Update block matcher
    sbm->setPreFilterCap(config.prefilter_cap);
    sbm->setPreFilterSize(config.prefilter_size);

    sbm->setSmallerBlockSize(config.smaller_block_size);
    sbm->setTextureThreshold(config.texture_threshold);
    sbm->setUniquenessRatio(config.uniqueness_ratio);

    sbm->setBlockSize (config.block_size);
    sbm->setDisp12MaxDiff (config.disp12MaxDiff);
    sbm->setMinDisparity (config.min_disparity);
    sbm->setNumDisparities (config.num_disparities);
    sbm->setSpeckleRange (config.speckle_range);
    sbm->setSpeckleWindowSize (config.speckle_window_size);
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

    // Undistort and convert to grayscale
    cv::Mat m1, m2;
    cv::fisheye::initUndistortRectifyMap(K1,D1,R1,K1,left_ptr->image.size(), CV_32FC1, m1, m2);
    cv::remap(left_ptr->image, left_ptr->image, m1, m2, cv::INTER_LINEAR);
    cv::cvtColor(left_ptr->image, left_ptr->image, cv::COLOR_BGR2GRAY);

    cv::fisheye::initUndistortRectifyMap(K2,D2,R2,K2,right_ptr->image.size(), CV_32FC1, m1, m2);
    cv::remap(right_ptr->image, right_ptr->image, m1, m2, cv::INTER_LINEAR);
    cv::cvtColor(right_ptr->image, right_ptr->image, cv::COLOR_BGR2GRAY);

    // Run Stereo Block Matching
    cv::Mat disparity;
    sbm->compute(left_ptr->image, right_ptr->image, disparity);
    normalize(disparity, disparity, 0, 255, CV_MINMAX, CV_8U);

    // Update visualization windows
    imshow("left", left_ptr->image);
    imshow("right", right_ptr->image);
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

    ros::init(argc, argv, "fisheye_rect");
    ros::NodeHandle nh;

    // Dynamic Reconfigure
    dynamic_reconfigure::Server<carrt_goggles::disparityConfig> reconfig_server;
    reconfig_server.setCallback(boost::bind(&reconfigCallback, _1, _2));

    // Sync incoming images
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/stereo/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/stereo/right/image_raw", 1);
    message_filters::Synchronizer<StereoSyncPolicy> sync(StereoSyncPolicy(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&stereoCallback, _1, _2));

    // Visualization Windows
    cv::namedWindow("left");
    cv::namedWindow("right");
    cv::namedWindow("disparity");
    cv::startWindowThread();

    ros::Rate loop_rate(30);
    while (nh.ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
