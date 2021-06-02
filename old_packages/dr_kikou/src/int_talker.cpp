/*
#include "ros/ros.h"
#include "std_msgs/Int64.h"


//int accept;


void chatterCallback(const std_msgs::Int64 msg){
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  //accept = msg.data;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "int_talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Int64>("chatter", 1);
  //ros::Subscriber sub = n.subscribe("chatter", 1, chatterCallback);
  ros::Rate loop_rate(10);
  
  while(ros::ok()){
	ros::spinOnce();
    std_msgs::Int64 msg;

    msg.data = 10;

    chatter_pub.publish(msg);



    loop_rate.sleep();
  }


  return 0;
}

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"


ros::Publisher  pub;
ros::Subscriber sub;

void chatterCallback(const std_msgs::Int64::ConstPtr& msg){
	std_msgs::Int64 hoge;
	hoge.data = msg->data;
	pub.publish(hoge);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "listener");

	ros::NodeHandle nh;

	pub = nh.advertise<std_msgs::Int64>("out", 1);
	sub = nh.subscribe("in", 1, chatterCallback);

	int accept = 0;
	ROS_FATAL("%d",accept);

	ros::spin();

	return 0;
}
