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
image_transport::Publisher left_rect_pub, right_rect_pub, disparity_pub;

// Dynamic Reconfigure Parameters
int right_img_y_offset = 0;   // Vertical offset of the right image.
double x_skew, y_skew, z_skew;  // Skew multiplier for point clouds.
double x_scale, y_scale;  // scale multiplier for point clouds.
double y_offset;  // scale multiplier for point clouds.
bool use_sgbm = false;

unsigned int sequence = 0;   // Point cloud header sequence

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

    right_img_y_offset = config.right_img_y_offset; // Y-offset of right image
    x_skew = config.x_skew;
    y_skew = config.y_skew;
    z_skew = config.z_skew;

    x_scale = config.x_scale;
    y_scale = config.y_scale;

    y_offset = config.y_offset;
}

void disparityCallback(const sensor_msgs::ImageConstPtr& disparity_image)
{
    // Convert to cv::Mat.
    cv_bridge::CvImagePtr disparity = cv_bridge::toCvCopy(disparity_image, sensor_msgs::image_encodings::TYPE_16SC1);
    disparity->image.convertTo(disparity->image, CV_32F, 1./16);
    cv::Mat points_mat(disparity->image.size(), CV_32FC3);
    cv::reprojectImageTo3D(disparity->image, points_mat, Q, true, CV_32F);

    // Fill in new PointCloud message (2D image-like layout)
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    point_cloud.points.reserve(disparity->image.rows * disparity->image.cols);
    point_cloud.header.frame_id = "stereo";
    pcl_conversions::toPCL(ros::Time::now(), point_cloud.header.stamp);
    point_cloud.header.seq = sequence++;
    point_cloud.height = 1;
    point_cloud.is_dense = false; // there may be invalid points

    for (unsigned int u = 0; u < points_mat.rows; ++u) {
        for (unsigned int v = 0; v < points_mat.cols; ++v) {
            float z = points_mat.at<cv::Vec3f>(u, v)[2];
            if(z != 10000 && !std::isinf(z)) {
                pcl::PointXYZRGB point;
                point.x = (points_mat.at<cv::Vec3f>(u, v)[0] + z*x_skew) / ((z+1)*x_scale);
                point.y = (points_mat.at<cv::Vec3f>(u, v)[1] + z*y_skew) / ((z+1)*y_scale) - y_offset;   // Average eye-height in meters
                point.z = z * z_skew;
                point_cloud.points.push_back(point);
            }
        }
    }

    point_cloud.width = point_cloud.points.size();
    point_cloud_pub.publish(point_cloud);
}

void stereoCallback(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right)
{
    // Parse ROS Message to opencv
    cv_bridge::CvImageConstPtr left_ptr;
    try {left_ptr = cv_bridge::toCvShare(left, "");}
    catch (cv_bridge::Exception& e) { ROS_ERROR("%s", e.what()); return; }

    cv_bridge::CvImageConstPtr right_ptr;
    try {right_ptr = cv_bridge::toCvShare(right, "");}
    catch (cv_bridge::Exception& e) { ROS_ERROR("%s", e.what()); return; }

    // Calculate Knew
    //cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K1, D1, left_ptr->image.size(), R1, P1, 0, left_ptr->image.size(), 1.0);
    //cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K2, D2, right_ptr->image.size(), R2, P2, 0, right_ptr->image.size(), 1.0);

    // Undistort
    cv::Mat m1, m2;
    cv::fisheye::initUndistortRectifyMap(K1,D1,R1,K1,left_ptr->image.size(), CV_16SC2, m1, m2);
    cv::remap(left_ptr->image, left_ptr->image, m1, m2, cv::INTER_LINEAR);

    cv::fisheye::initUndistortRectifyMap(K2,D2,R2,K2,right_ptr->image.size(), CV_16SC2, m1, m2);
    cv::remap(right_ptr->image, right_ptr->image, m1, m2, cv::INTER_LINEAR);

    // Translate
    cv::Mat trans_mat = (cv::Mat_<double>(2,3) << 1, 0, 0, 0, 1, right_img_y_offset);
    cv::warpAffine(right_ptr->image,right_ptr->image,trans_mat,right_ptr->image.size());

    // Publish Rectified Images
    left_rect_pub.publish(left_ptr->toImageMsg());
    right_rect_pub.publish(right_ptr->toImageMsg());

    // convert to grayscale
    cv::Mat left_gray, right_gray;
    cv::cvtColor(left_ptr->image, left_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_ptr->image, right_gray, cv::COLOR_BGR2GRAY);

    // Run Stereo Block Matching
    cv::Mat disparity;
    if(use_sgbm)
        sg_block_matcher->compute(left_gray, right_gray, disparity);
    else
        block_matcher->compute(left_gray, right_gray, disparity);

    // Publish Disparity Image
    cv_bridge::CvImage disparity_image;
    disparity_image.header = left->header; // Same timestamp and tf frame as left image
    disparity_image.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    disparity_image.image = disparity;
    disparity_pub.publish(disparity_image.toImageMsg());
}


int main(int argc, char **argv)
{
    // Read calibration info from yaml.
    std::string calib_path = ros::package::getPath("carrt_goggles") + "/data/shamsi_head_480p.yml";
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

    // Subscribe to disparity image to generate pointcloud
    ros::Subscriber disparity_sub = nh.subscribe("disparity", 1, disparityCallback);

    // Point cloud publisher
    point_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("point_cloud", 1);

    // Image publishers
    image_transport::ImageTransport it(nh);
    left_rect_pub = it.advertise("left/image_rect", 1);
    right_rect_pub = it.advertise("right/image_rect", 1);
    disparity_pub = it.advertise("disparity", 1);

    ros::MultiThreadedSpinner mult_spinner(3);
    mult_spinner.spin();
}
