#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <abu2021_msgs/cmd_vw.h>

const double MOT_MAX = 2.5;
const double FRAME_PARAM = 0.4/sqrt(2);

ros::Publisher pub;

void get_joy(const sensor_msgs::Joy::ConstPtr& msg);
void mot_over_correct(double vx, double vy, double w);

int main(int argc, char **argv){
	ros::init(argc, argv, "task_manager_DR");

	ros::NodeHandle nh;

	pub = nh.advertise<abu2021_msgs::cmd_vw>("cmd_dr", 1);
	ros::Subscriber sub = nh.subscribe("joy", 1, get_joy);

	ros::spin();

	return 0;
}

// ジョイコン
void get_joy(const sensor_msgs::Joy::ConstPtr& msg){
	abu2021_msgs::cmd_vw cmd;

	cmd.vx = -msg.axes[1];
	cmd.vy = msg.axes[0];
	cmd.w  = msg.axes[2];

	pub.publish(cmd);
}


/* void mot_over_correct(double vx, double vy, double w){ */
/* 	double mot_max = fabs(vx/sqrt(2)) + fabs(vy/sqrt(2)) + FRAME_PARAM*w; */
/* 	if(mot_max > MOT_MAX){ */

/* 	} */
/* } */
