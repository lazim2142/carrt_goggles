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
void captureCallback(const sensor_msgs::ImageConstPtr& image){
    if(ros::Time::now().toSec() - prev_capture_time > 0.25) {
        ROS_INFO("Capturing training image %d", num_captures);
        cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(image, "bgr8");
        cv::Mat gray;
        cv::cvtColor(cv_img->image, gray, CV_BGR2GRAY);

        std::vector<cv::Rect_<int> > faces;
        haar_cascade.detectMultiScale(gray, faces);

        for(int i = 0; i < faces.size(); i++) {
            ++num_captures;
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

cv::Ptr<cv::face::BasicFaceRecognizer> model;
std::map<int, std::string> hash_2_name;
std::map<int, double> hash_2_seen_time;
std::map<int, int> hash_2_counter;
ros::Publisher detection_pub;
bool visualize = false;
void recognizeCallback(const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(image, "bgr8");
    cv::Mat gray;
    cv::cvtColor(cv_img->image, gray, CV_BGR2GRAY);

    // Detect Faces
    std::vector<cv::Rect_<int> > faces;
    haar_cascade.detectMultiScale(gray, faces);

    // Try to recognize each face
    for(int i = 0; i < faces.size(); i++) {
        cv::Rect face_i = faces[i];
        cv::Mat face = gray(face_i);
        cv::resize(face, face, cv::Size(IM_WIDTH, IM_HEIGHT), 1.0, 1.0, cv::INTER_CUBIC);
        int prediction = -1; double distance;
        model->predict(face, prediction, distance);

        // if recognized
        if(prediction >= 0) {
            hash_2_seen_time[prediction] = ros::Time::now().toSec();
            hash_2_counter[prediction] = std::max(hash_2_counter[prediction] + 2, 10);

            if(hash_2_counter[prediction] > 7) {
                std_msgs::String recognized_face;
                recognized_face.data = "Found " + hash_2_name[prediction];
                detection_pub.publish(recognized_face);

                if(visualize){
                    cv::rectangle(gray, face_i, CV_RGB(0, 255,0), 1);
                    std::string box_text = cv::format("%s, %f", hash_2_name[prediction].c_str(), distance);
                    int pos_x = std::max(face_i.tl().x - 10, 0);
                    int pos_y = std::max(face_i.tl().y - 10, 0);
                    cv::putText(gray, box_text, cv::Point(pos_x, pos_y), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255,255,255), 2.0);
                }
            }
        }
    }



    // Complain about people leaving you, like your ex-wife.
    for(std::map<int,double>::iterator itr = hash_2_seen_time.begin(); itr != hash_2_seen_time.end(); ++itr){
        double time_since_last_seen = ros::Time::now().toSec() - itr->second;
        hash_2_counter[itr->first] = std::min(hash_2_counter[itr->first] - 1, 0);
        if(time_since_last_seen > 15 && time_since_last_seen < 15.2) {
            std_msgs::String gone_message;
            gone_message.data = hash_2_name[itr->first] + "is no longer detected";
            detection_pub.publish(gone_message);
        }
    }

    if(visualize) {
        cv::imshow("face_recognizer", gray);
        cv::waitKey(3);
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
        cv::Mat image = cv::imread(image_path, CV_8U);
        images.push_back(image);

        std::size_t underscore_index = itr->path().filename().string().find("_");
        std::string person_name = itr->path().filename().string().substr(0, underscore_index);
        std::vector<unsigned char> c_vec(person_name.begin(), person_name.end());        
        c_vec.push_back(0);
        labels.push_back(hash(&c_vec[0]));
    }
}

void initDictionaries(){
    std::string image_dir = package_path + "/data/faces/";
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(image_dir); itr != end_itr; ++itr) {
        std::size_t underscore_index = itr->path().filename().string().find("_");
        std::string person_name = itr->path().filename().string().substr(0, underscore_index);
        std::vector<unsigned char> c_vec(person_name.begin(), person_name.end());
        int key = hash(&c_vec[0]);
        hash_2_name[key] = person_name;
        hash_2_seen_time[key] = 0;
        hash_2_counter[key] = 0;
    }
}

int main(int argc, char **argv) {
    package_path = ros::package::getPath("carrt_goggles");
    std::string haar_cascade_path = package_path + "/data/haarcascade_frontalface_default.xml";
    std::string fisher_model_path = package_path + "/data/fisher_model.yml";
    ROS_INFO("Loading HAAR Cascade from: %s", haar_cascade_path.c_str());
    haar_cascade.load(haar_cascade_path);

    ros::init(argc, argv, "face_rec");
    ros::NodeHandle nh("~");
    name = "";
    if(!nh.getParam("cmd", name)) {
        ROS_INFO("Please pass in param cmd, possible values = recognize, train, name_of_person");
        return 0;
    }
    nh.getParam("visualize", visualize);

    if(name.compare("recognize") == 0 || name.compare("Recognize") == 0){
        ROS_INFO("Recognizing");
        initDictionaries();

        model = cv::face::createFisherFaceRecognizer();
        model->load(fisher_model_path);

        detection_pub = nh.advertise<std_msgs::String>("recognized_faces", 1);
        ros::Subscriber image_sub = nh.subscribe("image_raw", 1, recognizeCallback);
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

        // Initialize num_captures so as to not overwrite existing images
        std::string image_dir = package_path + "/data/faces/";
        boost::filesystem::directory_iterator end_itr;
        for (boost::filesystem::directory_iterator itr(image_dir); itr != end_itr; ++itr) {
            std::string image_name = itr->path().filename().string();
            if(image_name.compare(name) == 0) {
                std::string image_number;
                for (int i = 0; i < image_name.length(); ++i)
                    if(std::isdigit(image_name.at(i)))
                        image_number += image_name.at(i);
                num_captures = std::max(num_captures, boost::lexical_cast<int>(image_number));
            }
        }

        ros::Subscriber image_sub = nh.subscribe("image_raw", 1, captureCallback);
        ros::spin();
    }
}
