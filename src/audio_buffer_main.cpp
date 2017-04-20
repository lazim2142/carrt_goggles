#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sound_play/sound_play.h>

#define CROSSWALK_PRIORITY 1
#define OBSTACLE_PRIORITY 2
#define HAZARD_PRIORITY 3
#define PHONE_PRIORITY 4
#define FRIEND_PRIORITY 5

sound_play::SoundClient* sc;
ros::Subscriber sub_1;
ros::Subscriber sub_2;
ros::Subscriber sub_3;
ros::Subscriber sub_4;
ros::Subscriber sub_5;

class PriorityQueue
{
private:
    struct Node {
        int priority;
        std_msgs::String message;
        struct Node* link = NULL;
        Node(std_msgs::String s, int p)
            : message(s), priority(p) {}
    };

    Node *front;
    double pub_time;

public:

    PriorityQueue() {
	   front = NULL;
	}
	
    void insert (std_msgs::String item, int priority) {
        Node* insert_ptr = front;
        while (insert_ptr != NULL && insert_ptr->priority < priority)
            insert_ptr = insert_ptr->link;

        if(insert_ptr == NULL){
            Node* tmp = new Node(item, priority);
            front = tmp;
        } else if (priority == OBSTACLE_PRIORITY && insert_ptr->priority == OBSTACLE_PRIORITY) {
            insert_ptr->message = item;
        }
        else {
            Node* tmp = new Node(item, priority);
            tmp->link = insert_ptr->link;
            insert_ptr->link = tmp;
        }
	}

    void publish() {
       if (ros::Time::now().toSec() - pub_time > 1.5 && front != NULL) {
            sc->say(front->message.data);
            Node *tmp = front;
            front = front->link;
            delete tmp;
            pub_time = ros::Time::now().toSec();
	   }
	}
	
    void printqueue() {
        Node *print;
        print = front;
        ROS_INFO ("Audio Queue");
        while (print != NULL) {
            ROS_INFO("%s", print->message.data.c_str());
            print = print->link;
        }
        ROS_INFO (" ");
    }
	
};

PriorityQueue pq;

void crosswalkCallback(const std_msgs::String input)
{
    pq.insert(input, CROSSWALK_PRIORITY);
    pq.printqueue();
}

void obstacleCallback(const std_msgs::String input2)
{
    pq.insert(input2, OBSTACLE_PRIORITY);
    pq.printqueue();
}
void hazardCallback(const std_msgs::String  input3)
{
    pq.insert(input3, HAZARD_PRIORITY);
    pq.printqueue();
}
void phoneCallback(const std_msgs::String input4)
{
    pq.insert(input4, PHONE_PRIORITY);
    pq.printqueue();
}
void friendCallback(const std_msgs::String input5)
{
    pq.insert(input5, FRIEND_PRIORITY);
    pq.printqueue();
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "audio_buffer");
    ros::NodeHandle nh;
    sc = new sound_play::SoundClient();

    sub_1 = nh.subscribe("crosswalk_signal", 2, crosswalkCallback);
    sub_2 = nh.subscribe("obstacle", 1, obstacleCallback);
    sub_3 = nh.subscribe("hazard", 2, hazardCallback);
    sub_4 = nh.subscribe("phone", 2,phoneCallback);
    sub_5 = nh.subscribe("friend", 2, friendCallback);

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        pq.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
