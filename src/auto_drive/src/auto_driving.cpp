#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "abu2021_msgs/cmd_vw.h"

void get_gyro(const std_msgs::Float64::ConstPtr& yaw);

int main(int argc, char **argv){
	ros::init(argc, argv, "auto_drive");

	ros::NodeHandle nh;

	ros::Publisher  pub = nh.advertise<abu2021_msgs::cmd_vw>("target", 1);
	ros::Subscriber sub_yaw = nh.subscribe("gyro_yaw", 1, get_gyro);

	ros::spin();

	return 0;
}

void get_gyro(const std_msgs::Float64::ConstPtr& yaw){

}


