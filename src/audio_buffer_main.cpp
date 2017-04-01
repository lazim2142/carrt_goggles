#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>


 ros::Publisher pub_;
 ros::Subscriber sub_1;
 ros::Subscriber sub_2;
 ros::Subscriber sub_3;
 ros::Subscriber sub_4;
 ros::Subscriber sub_5;


struct node
{
	int priority;
	std_msgs::String message;
	struct node* link;
};

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
	
	void insert (std_msgs::String item, int priority)
	{
	   node *tmp, *q;
	   tmp = new node;
	   tmp->message = item;
	   tmp->priority = priority;
	   if (front == NULL || priority < front->priority)
	   {
		tmp->link = front;
		front = tmp;
	   }
	   else
	   {
		q = front;
		while (q->link != NULL && q->link->priority <= priority)
		{	q = q->link;    }
			tmp->link = q->link;
			q->link = tmp;
		
	   }
	}
	//Publish

	void publish()
	{
	   node *tmp;
	   if (front == NULL)
		{
			;
		}
	   else
	   {
		pub_.publish(front->message);
		tmp = front;		
		front = front->link;
		free(tmp);
	   }
	}
	
//Print
	void printqueue()
	{ 
	node *print;
	print = front;
	while (print->link !=NULL)	
	{
		ROS_INFO("%s", print->message.data.c_str());
		print = print->link;
	}	
	ROS_INFO ("/n");
}
	
};

Priority_Queue pq;

void callback1(const std_msgs::String input)
  {
   pq.insert(input, 1);
   pq.printqueue();
  }
void callback2(const std_msgs::String input2)
  {
   pq.insert(input2, 2);
   pq.printqueue();
  }
void callback3(const std_msgs::String  input3)
  {
   pq.insert(input3, 3);
   pq.printqueue();
  }
void callback4(const std_msgs::String input4)
  {
   pq.insert(input4, 4);
   pq.printqueue();
  }
void callback5(const std_msgs::String input5)
  {
   pq.insert(input5, 5);
   pq.printqueue();
  }






/*
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
  ros::Publisher pub;


 SubscribeAndPublish()
 {
  //Topic Published
  pub_ = audio_buffer_node.advertise<std_msgs::String>("publish", 1);
  pub = audio_buffer_node.advertise<std_msgs::String>("publish", 1);


  //Topic Subscribed
  sub_1 = audio_buffer_node.subscribe("textout", 1, &SubscribeAndPublish::callback1, this);
  sub_2 = audio_buffer_node.subscribe("textout2", 1, &SubscribeAndPublish::callback2, this);
  sub_3 = audio_buffer_node.subscribe("textout", 1, &SubscribeAndPublish::callback3, this);
  sub_4 = audio_buffer_node.subscribe("textout2", 1, &SubscribeAndPublish::callback4, this);
  sub_5 = audio_buffer_node.subscribe("textout", 1, &SubscribeAndPublish::callback5, this);

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
	std_msgs::String msg;
	msg.data = "Hello World";
	pub.publish(msg);
	if (front == NULL)
	{
		std_msgs::String msg;
		msg.data = "There are no messages.";
		pub_.publish(msg);
	}
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
void callback2(const std_msgs::String& input2)
  {
   insert(input2, 2);
  }
void callback3(const std_msgs::String& input3)
  {
   insert(input3, 3);
  }
void callback4(const std_msgs::String& input4)
  {
   insert(input4, 4);
  }
void callback5(const std_msgs::String& input5)
  {
   insert(input5, 5);
  }



}; *///End of Class



int main (int argc, char **argv)
{

ros::init(argc, argv, "subscribe_and_publish");
ros::NodeHandle audio_buffer_node;

  pub_ = audio_buffer_node.advertise<std_msgs::String>("publish", 10);
  sub_1 = audio_buffer_node.subscribe("textout", 2, callback1);
  sub_2 = audio_buffer_node.subscribe("textout3", 2, callback2);
  sub_3 = audio_buffer_node.subscribe("textout4", 2, callback3);
  sub_4 = audio_buffer_node.subscribe("textout5", 2,callback4);
  sub_5 = audio_buffer_node.subscribe("textout2", 2, callback5);




ros::Rate loop_rate(.9);
//loop_rate.sleep();

while (ros::ok())
{

	
pq.publish();
ros::spinOnce();
loop_rate.sleep();


}
ros::spin();
return 0;
}
