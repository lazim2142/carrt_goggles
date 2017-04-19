#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <map>
#include "std_msgs/String.h"

#define IM_WIDTH 120
#define IM_HEIGHT 90

std::vector<int> labels;
cv::CascadeClassifier haar_cascade;
std::string package_path;

double prev_capture_time;
std::string name;
int num_captures = 0;
void trainCallback(const sensor_msgs::ImageConstPtr& image){
    if(ros::Time::now().toSec() - prev_capture_time > 0.5) {
        ++num_captures;
        ROS_INFO("Capturing training image %d", num_captures);
        cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(image, "bgr8");
        cv::Mat gray;
        cv::cvtColor(cv_img->image, gray, CV_BGR2GRAY);

        std::vector<cv::Rect_<int> > faces;
        haar_cascade.detectMultiScale(gray, faces);

        for(int i = 0; i < faces.size(); i++) {
            cv::Rect face_i = faces[i];
            cv::rectangle(cv_img->image, face_i, CV_RGB(0, 255,0), 1);
            cv::Mat face = gray(face_i);
            cv::resize(face, face, cv::Size(IM_WIDTH, IM_HEIGHT), 1.0, 1.0, cv::INTER_CUBIC);
            cv::imwrite(package_path + "/data/faces/" + name + "_"
                        + boost::lexical_cast<std::string>(num_captures) + ".pgm", face);
        }

        cv::imshow("Detected Faces", cv_img->image);
        cv::waitKey(3);
        prev_capture_time = ros::Time::now().toSec();
    }
}

cv::Ptr<cv::face::FaceRecognizer> model;
std::map<int, std::string> hash_name_dict;
ros::Publisher detection_pub;
void recCallback(const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(image, "bgr8");
    cv::Mat gray;
    cv::cvtColor(cv_img->image, gray, CV_BGR2GRAY);

    std::vector<cv::Rect_<int> > faces;
    haar_cascade.detectMultiScale(gray, faces);

    for(int i = 0; i < faces.size(); i++) {
        cv::Rect face_i = faces[i];
        cv::Mat face = gray(face_i);
        cv::resize(face, face, cv::Size(IM_WIDTH, IM_HEIGHT), 1.0, 1.0, cv::INTER_CUBIC);
        int prediction; double confidence;
        model->predict(face, prediction, confidence);

        std_msgs::String recognized_face;
        recognized_face.data = "Found " + hash_name_dict[prediction];
        detection_pub.publish(recognized_face);
        ROS_INFO("%s with confidence %f", recognized_face.data.c_str(), (float)confidence);
    }
}

int hash(unsigned char * c_str) {
    int hash = 5381;
    int c;
    while (c = *c_str++)
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
    return abs(hash);
}

void loadImagesAndLabels(std::vector<cv::Mat>& images, std::vector<int>& labels){
    std::string image_dir = package_path + "/data/faces/";
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(image_dir); itr != end_itr; ++itr) {
        std::string image_path = image_dir + itr->path().filename().string();
        cv::Mat image = cv::imread(image_path);
        images.push_back(image);

        std::size_t underscore_index = itr->path().filename().string().find("_");
        std::string person_name = itr->path().filename().string().substr(0, underscore_index);
        std::vector<unsigned char> c_vec(person_name.begin(), person_name.end());
        c_vec.push_back(0);
        labels.push_back(hash(&c_vec[0]));
    }
}

void initHash2NameDictionary(){
    std::string image_dir = package_path + "/data/faces/";
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(image_dir); itr != end_itr; ++itr) {
        std::size_t underscore_index = itr->path().filename().string().find("_");
        std::string person_name = itr->path().filename().string().substr(0, underscore_index);
        std::vector<unsigned char> c_vec(person_name.begin(), person_name.end());
        hash_name_dict[hash(&c_vec[0])] = person_name;
    }
}

int main(int argc, char **argv) {
    package_path = ros::package::getPath("carrt_goggles");
    std::string haar_cascade_path = package_path + "/data/haarcascade_frontalface_default.xml";
    std::string fisher_model_path = package_path + "/data/fisher_model.yml";
    ROS_INFO("Loading HAAR Cascade from: %s", haar_cascade_path.c_str());
    haar_cascade.load(haar_cascade_path);

    name = "";
    if (argc == 2 || argc == 4)
        name = argv[1];

    if(name.compare("recognize") == 0 || name.compare("Recognize") == 0){
        ROS_INFO("Recognizing");
        initHash2NameDictionary();
        model = cv::face::createFisherFaceRecognizer();
        model->load(fisher_model_path);

        ros::init(argc, argv, "face_rec");
        ros::NodeHandle nh;
        detection_pub = nh.advertise<std_msgs::String>("recognized_faces", 1);
        ros::Subscriber image_sub = nh.subscribe("image_raw", 1, recCallback);
        ros::spin();
    }
    else if (name.compare("train") == 0 || name.compare("Train") == 0) {
        ROS_INFO("Training");
        std::vector<cv::Mat> images;
        std::vector<int> labels;
        loadImagesAndLabels(images, labels);
        model = cv::face::createFisherFaceRecognizer();
        model->train(images, labels);
        model->save(fisher_model_path);
    }
    else if (name.length() > 0){
        ROS_INFO("Capturing Training Images");
        ros::init(argc, argv, "face_rec_train");
        ros::NodeHandle nh;
        ros::Subscriber image_sub = nh.subscribe("image_raw", 1, trainCallback);
        ros::spin();
    }
}
