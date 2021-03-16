#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include "abu2021_msgs/odom_rad.h"

//初期状態[m][m][rad]
double WHEEL = 0.0292; //オドメータ車輪半径[m]

double st_x = 0;
double st_y = 0;
double st_yaw = 0;
double odom_x = 0;
double odom_y = 0;
double odom_pre_x = 0;
double odom_pre_y = 0;

double LOOP_RATE = 100;

void broadcast_tf();
void get_odom(const abu2021_msgs::odom_rad::ConstPtr& odm);
void get_gyro(const std_msgs::Float64::ConstPtr& yaw);

int main(int argc, char **argv){
	ros::init(argc, argv, "dead_reckoning");
	ros::NodeHandle nh;

	ros::Subscriber sub_odm = nh.subscribe("odometer", 1, get_odom);
	ros::Subscriber sub_yaw = nh.subscribe("gyro_yaw", 1, get_gyro);

	ros::Rate rate(LOOP_RATE);

	while(ros::ok()){
		ros::spinOnce();

		//calculation position & pose
		st_x += (odom_x - odom_pre_x)*WHEEL*cos(st_yaw) - (odom_y - odom_pre_y)*WHEEL*sin(st_yaw);
		st_y += (odom_x - odom_pre_x)*WHEEL*sin(st_yaw) + (odom_y - odom_pre_y)*WHEEL*cos(st_yaw);
		odom_pre_x = odom_x;
		odom_pre_y = odom_y;

		// tf broadcast
		broadcast_tf();
		
		rate.sleep();
	}

	return 0;
}

void broadcast_tf(){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped tfstamp;
	  
	tfstamp.header.stamp = ros::Time::now();
	tfstamp.header.frame_id = "odom";
	tfstamp.child_frame_id = "base_link";
	tfstamp.transform.translation.x = st_x;
	tfstamp.transform.translation.y = st_y;
	tfstamp.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, st_yaw);
	tfstamp.transform.rotation.x = q.x();
	tfstamp.transform.rotation.y = q.y();
	tfstamp.transform.rotation.z = q.z();
	tfstamp.transform.rotation.w = q.w();
	br.sendTransform(tfstamp);
}

void get_odom(const abu2021_msgs::odom_rad::ConstPtr& odm){
	odom_x = odm->x;
	odom_y = odm->y;
}

void get_gyro(const std_msgs::Float64::ConstPtr& yaw){
	st_yaw = yaw->data*M_PI/180;
}
