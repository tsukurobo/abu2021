#include "../include/point.h"
#include "../include/pure_pursuit.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "abu2021_msgs/cmd_vw.h"
#include "abu2021_msgs/odom_rad.h"
#include "abu2021_msgs/auto_drive_order.h"

/*****************************************************/
/********** body *************************************/
/*****************************************************/

//pure pursuit parameter
double MAX_SPEED_V = 0.3; //最大移動速度[m/s]
int AHEAD_NUM = 10; //何個先の経路点列を目指すか[･]
double RANGE_FIN = 0.01; //終了範囲[m]
double RANGE_DCL = 1; //減速開始範囲[m]
double YAW_GAIN_P = 1; //yaw軸PID制御Pゲイン
double YAW_GAIN_I = 0; //yaw軸PID制御Iゲイン
double YAW_GAIN_D = 0; //yaw軸PID制御Dゲイン
double MAX_SPEED_W = 0.3; //最大角速度[rad/s]
std::string PATH1_PATH;
std::string PATH2_PATH;
std::string PATH3_PATH;
std::string PATH4_PATH;
std::string PATH5_PATH;
std::string PATH6_PATH;
std::string PATH7_PATH;
std::string PATH8_PATH;
/* std::string PATH1_PATH("/home/koki/abu2021/src/auto_drive/pathes/dr_st_rt.csv"); */

//自己位置推定パラメータ
//初期状態[m][m][rad]
double INIT_X = 3;
double INIT_Y = 3;
double INIT_YAW = 0;
double WHEEL = 0.0292; //オドメータ車輪半径[m]

//パラメータ
const int LOOP_RATE = 100; //loop rate [Hz]

void get_gyro(const std_msgs::Float64::ConstPtr& yaw);
void get_odom(const abu2021_msgs::odom_rad::ConstPtr& odm);
void get_order(const abu2021_msgs::auto_drive_order::ConstPtr& order);

//Pure pursuit
Pure_pursuit pp(1/(double)LOOP_RATE);

abu2021_msgs::cmd_vw cmd;
int emg_stop = 0;
int order_go = 0;
int order_path1 = 0;
int order_path2 = 0;
int order_path3 = 0;
int order_path4 = 0;
int order_path5 = 0;
int order_path6 = 0;
int order_path7 = 0;
int order_path8 = 0;
int order_set1 = 0;
int order_set2 = 0;
int order_set3 = 0;

int mode_go = 0;


int main(int argc, char **argv){
	//ROS
	ros::init(argc, argv, "auto_drive");
	ros::NodeHandle nh;
	ros::Publisher  pub = nh.advertise<abu2021_msgs::cmd_vw>("cmd", 1);
	ros::Subscriber sub_yaw = nh.subscribe("gyro_yaw", 1, get_gyro);
	ros::Subscriber sub_odm = nh.subscribe("odometer", 1, get_odom);
	ros::Subscriber sub_order = nh.subscribe("ad_order", 1, get_order);
	//parameter
	nh.getParam("state/init_x", INIT_X);
	nh.getParam("state/init_y", INIT_Y);
	nh.getParam("state/init_yaw", INIT_YAW);
	nh.getParam("state/wheel", WHEEL);
	nh.getParam("yaw_pid/p", YAW_GAIN_P);
	nh.getParam("yaw_pid/i", YAW_GAIN_I);
	nh.getParam("yaw_pid/d", YAW_GAIN_D);
	nh.getParam("yaw_pid/max_speed_w", MAX_SPEED_W);
	nh.getParam("pure_pursuit/max_speed_v", MAX_SPEED_V);
	nh.getParam("pure_pursuit/ahead_num", AHEAD_NUM);
	nh.getParam("pure_pursuit/range_fin", RANGE_FIN);
	nh.getParam("pure_pursuit/range_dcl", RANGE_DCL);
	nh.getParam("pure_pursuit/path1", PATH1_PATH);
	nh.getParam("pure_pursuit/path2", PATH2_PATH);
	nh.getParam("pure_pursuit/path3", PATH3_PATH);
	nh.getParam("pure_pursuit/path4", PATH4_PATH);
	nh.getParam("pure_pursuit/path5", PATH5_PATH);
	nh.getParam("pure_pursuit/path6", PATH6_PATH);
	nh.getParam("pure_pursuit/path7", PATH7_PATH);
	nh.getParam("pure_pursuit/path8", PATH8_PATH);

	ros::Rate rate(LOOP_RATE);

	/* pp.reset_path("/home/koki/abu2021/src/auto_drive/pathes/square_3m.csv", AHEAD_NUM); */
	pp.reset_path(PATH3_PATH, AHEAD_NUM);
	pp.set_state(INIT_X, INIT_Y, INIT_YAW);
	pp.set_pos_pid(YAW_GAIN_P, YAW_GAIN_I, YAW_GAIN_D, MAX_SPEED_W);

	while(ros::ok()){
		ros::spinOnce();

		//経路設定
		if(order_path1 == 1) pp.reset_path(PATH1_PATH, AHEAD_NUM);
		else if(order_path2 == 1) pp.reset_path(PATH2_PATH, AHEAD_NUM);
		else if(order_path3 == 1) pp.reset_path(PATH3_PATH, AHEAD_NUM);
		else if(order_path4 == 1) pp.reset_path(PATH4_PATH, AHEAD_NUM);
		else if(order_path5 == 1) pp.reset_path(PATH5_PATH, AHEAD_NUM);
		else if(order_path6 == 1) pp.reset_path(PATH6_PATH, AHEAD_NUM);
		else if(order_path7 == 1) pp.reset_path(PATH7_PATH, AHEAD_NUM);
		else if(order_path8 == 1) pp.reset_path(PATH8_PATH, AHEAD_NUM);
		
		//位置設定
		if(order_set1 == 1) pp.set_state(0.5, 5.425, -M_PI/2);
		else if(order_set2 == 1) pp.set_state(5.425, 2.45, M_PI/2);
		else if(order_set3 == 1) pp.set_state(11.4, 0.5, M_PI);

		//動作司令
		if(emg_stop == 1){ //緊急停止
			mode_go = 0;

			cmd.vx = 0;
			cmd.vy = 0;
			cmd.w  = 0;
			pub.publish(cmd);
		}else if(order_go ==  1 || mode_go == 1){
			mode_go = 1;

			if(pp.cmd_velocity(MAX_SPEED_V, RANGE_FIN, RANGE_DCL) != 0){ //台形制御
				pp.cmd_angular_v(pp.target_dir_global());
				cmd.vx = pp.cmd_vx;
				cmd.vy = pp.cmd_vy;
				cmd.w  = pp.cmd_w;
			}else{ //終了
				cmd.vx = 0;
				cmd.vy = 0;
				cmd.w  = 0;

				mode_go = 0;
			}
			
			pub.publish(cmd);
		}
		


		ROS_FATAL("\nstate_x: %f\tstate_y: %f\tstate_yaw: %f\ncmd_vx: %f\tcmd_vy: %f\tcmd_w: %f"
				, pp.state_p.x, pp.state_p.y, pp.state_yaw/M_PI*180, cmd.vx, cmd.vy, cmd.w/M_PI*180);

		rate.sleep();
	}

	return 0;
}

void get_gyro(const std_msgs::Float64::ConstPtr& yaw){
	pp.set_posture(yaw->data*M_PI/180 + INIT_YAW);
}

void get_odom(const abu2021_msgs::odom_rad::ConstPtr& odm){
	static double pre_x = 0;
	static double pre_y = 0;

	double dx = (odm->x - pre_x)*WHEEL*cos(pp.state_yaw) - (-odm->y - pre_y)*WHEEL*sin(pp.state_yaw);
	double dy = (odm->x - pre_x)*WHEEL*sin(pp.state_yaw) + (-odm->y - pre_y)*WHEEL*cos(pp.state_yaw);

	pp.set_position(pp.state_p.x + dx, pp.state_p.y + dy);

	pre_x = odm->x;
	pre_y = -odm->y;
}

void get_order(const abu2021_msgs::auto_drive_order::ConstPtr& order){
	emg_stop = order->emg_stop;
	order_go    = order->go;
	order_path1 = order->path1;
	order_path2 = order->path2;
	order_path3 = order->path3;
	order_path4 = order->path4;
	order_path5 = order->path5;
	order_path6 = order->path6;
	order_path7 = order->path7;
	order_path8 = order->path8;
	order_set1 = order->set1;
	order_set2 = order->set2;
	order_set3 = order->set3;
}
