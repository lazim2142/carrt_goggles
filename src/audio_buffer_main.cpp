#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>

struct node
{
	int priority;
	std_msgs::String message;
	struct node* link;
};
/*
class Priority_Queue
{
  private:
	
  public:
	node *front;
	Priority_Queue()
	{
	   front = NULL;
	}
	
	//Insert
	
	void insert (string item, int priority)
	{
	   node *tmp, *q;
	   tmp = new node;
	   tmp->message = item;
	   tmp->priority = priority;
	   if (front == NULL || priority < front ->priority)
	   {
		tmp->link = front;
		front = tmp;
	   }
	   else
	   {
		q = front;
		while (q->link != NULL && q->link->priority <= priority)
		q = q->link;
		tmp->link = q->link;
		q->link = tmp;
	   }
	}
	//Publish

	void publish()
	{
	   node *tmp;
	   if (front == NULL)
		cout << "There are no messages \n";
	   else
	   {
		pub_.publish(front->message);
		tmp = front;		
		front = front->link;
		free tmp;
	   }
	}
};
*/

class SubscribeAndPublish
{
private:
  ros::NodeHandle audio_buffer_node; 
  
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
  ros::Subscriber sub_3;
  ros::Subscriber sub_4;
  ros::Subscriber sub_5;
  node *front;
public:
  ros::Publisher pub_;

 SubscribeAndPublish()
 {
  //Topic Published
  pub_ = audio_buffer_node.advertise<std_msgs::String>("/robotsound", 1);

  //Topic Subscribed
  sub_1 = audio_buffer_node.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback1, this);
  sub_2 = audio_buffer_node.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback2, this);
  sub_3 = audio_buffer_node.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback3, this);
  sub_4 = audio_buffer_node.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback4, this);
  sub_5 = audio_buffer_node.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback5, this);

  //Node Front
  front = NULL;
  
}



//Insert
	
void insert (std_msgs::String item, int priority)
{
	node *tmp, *q;
	tmp = new node;
	tmp->message = item;
	tmp->priority = priority;
	if (front == NULL || priority < front ->priority)
	{
		tmp->link = front;
		front = tmp;
	}
	else
	{
		q = front;
		while (q->link != NULL && q->link->priority <= priority)
		q = q->link;
		tmp->link = q->link;
		q->link = tmp;
	}
}
	//Publish

void publish()
{
	node *tmp;
	if (front == NULL)
		std::cout << "There are no messages \n";
	else
	   {
		pub_.publish(front->message);
		tmp = front;		
		front = front->link;
		free(tmp);
	   }
}


void callback1(const std_msgs::String& input)
  {
   insert(input, 1);
  }
void callback2(const std_msgs::String& input)
  {
   insert(input, 2);
  }
void callback3(const std_msgs::String& input)
  {
   insert(input, 3);
  }
void callback4(const std_msgs::String& input)
  {
   insert(input, 4);
  }
void callback5(const std_msgs::String& input)
  {
   insert(input, 5);
  }



}; //End of Class

int main (int argc, char **argv)
{

ros::init(argc, argv, "audio_buffer");


SubscribeAndPublish AudioSAP;
ros::spin();
ros::Rate loop_rate(5);

while (ros::ok())
{

AudioSAP.publish();
ros::spinOnce();

}
return 0;
}
