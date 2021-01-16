#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <auto_drive_sim/vw_cmd.h>

auto_drive_sim::vw_cmd pose;

void callback(const sensor_msgs::Joy& joy);

int main(int argc, char **argv){
	ros::init(argc, argv, "get_joy");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("joy", 1, callback);
	ros::Publisher  pub = nh.advertise<auto_drive_sim::vw_cmd>("target_pose", 1);

	ros::Rate loop_rate(10);

	while (ros::ok()){
		pub.publish(pose);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}

void callback(const sensor_msgs::Joy& joy){
	pose.vx = joy.axes[3];
	pose.vy = joy.axes[2];
	pose.w  = joy.axes[0];
}

