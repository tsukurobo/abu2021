//Point class
#include <stdio.h>
#include <math.h>
//Pure_pursuit class
#include <math.h>
#include <fstream> //file read
#include <vector>
#include <string>
#include <algorithm> //min_element
//body
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "abu2021_msgs/cmd_vw.h"
#include "abu2021_msgs/odom_rad.h"

class Point{
	public:
		double x;
		double y;

		Point(double x=0, double y=0);
		Point(const Point& point);
		double change_to(double x, double y);
		double dist(double x, double y);
		double dist(Point point);
		double print();
};

class Pure_pursuit{
	public:
		//状態
		std::vector<Point> path; //経路点列
		Point state_p; //ロボ位置([m],[m])
		double state_yaw; //ロボ姿勢[rad]

		//司令変数
		double cmd_vx;
		double cmd_vy;
		double cmd_w;

		//関数
		/* Pure_pursuit(std::string file_name, int ahead_num); //コンストラクタ */
		void reset_path(std::string file_name, int ahead_num); //経路再設定
		void set_state(Point pos, double yaw);          //ロボ状態設定
		void set_state(double x, double y, double yaw); //ロボ状態設定
		void set_position(Point pos);          //ロボ位置設定
		void set_position(double x, double y); //ロボ位置設定
		void set_posture(double yaw); //ロボ姿勢設定
		void cmd_angular_v(double p, double i, double d); //角速度司令[rad/s]
		double cmd_velocity(double speed, double fin, double dcl); //速度司令[m/s]
		int print_path(); //経路点列表示
	
	private:
		//パラメータ
		std::string file_name; //読み込むCSVファイル名
		int ahead_num; //最近経路点からahead_num個先の点を目標点とする
		
		//関数
		double dist_fin(); //経路点列終端との距離[m]
		double target_dir_local(); //目標角度[rad](ローカル角度)
		double target_dir_global(); //目標角度[rad](グローバル角度)
		Point target_point(); //目標点
		int load_csv(); //CSVフィアル読み込み
};



/*****************************************************/
/********** body *************************************/
/*****************************************************/

//pure pursuit parameter
const double MAX_SPEED = 0.3; //最大移動速度[m/s]
const int AHEAD_NUM = 10; //何個先の経路点列を目指すか[･]
const double RANGE_FIN = 0.01; //終了範囲[m]
const double RANGE_DCL = 1; //減速開始範囲[m]
double YAW_GAIN_P = 1; //yaw軸PID制御Pゲイン
double YAW_GAIN_I = 0; //yaw軸PID制御Iゲイン
double YAW_GAIN_D = 0; //yaw軸PID制御Dゲイン

//自己位置推定パラメータ
//初期状態[m][m][rad]
/* double INIT_X = 0.5; */
/* double INIT_Y = 5.425; */
/* double INIT_YAW = -M_PI/2; */
double INIT_X = 0;
double INIT_Y = 0;
double INIT_YAW = 0;
double WHEEL = 0.0292; //オドメータ車輪半径[m]

//パラメータ
const int LOOP_RATE = 100; //loop rate [Hz]

void get_gyro(const std_msgs::Float64::ConstPtr& yaw);
void get_odom(const abu2021_msgs::odom_rad::ConstPtr& odm);
void get_path(const std_msgs::Int16::ConstPtr& path);

//Pure pursuit
Pure_pursuit pp;

abu2021_msgs::cmd_vw cmd;
int order_path = 0;
int mode_path = 0;


int main(int argc, char **argv){
	ros::init(argc, argv, "auto_drive");

	//ROS
	ros::NodeHandle nh;
	ros::Publisher  pub = nh.advertise<abu2021_msgs::cmd_vw>("cmd", 1);
	ros::Subscriber sub_yaw = nh.subscribe("gyro_yaw", 1, get_gyro);
	ros::Subscriber sub_odm = nh.subscribe("odometer", 1, get_odom);
	ros::Subscriber sub_path = nh.subscribe("ad_path", 1, get_path);
	//parameter
	nh.getParam("state/init_x", INIT_X);
	nh.getParam("state/init_y", INIT_Y);
	nh.getParam("yaw_pid/p", YAW_GAIN_P);
	nh.getParam("yaw_pid/i", YAW_GAIN_I);
	nh.getParam("yaw_pid/d", YAW_GAIN_D);

	ros::Rate rate(LOOP_RATE);

	/* pp.reset_path("/home/koki/abu2021/src/auto_drive/pathes/straight_3m.csv", AHEAD_NUM); */
	pp.reset_path("/home/koki/abu2021/src/auto_drive/pathes/square_3m.csv", AHEAD_NUM);
	pp.set_state(INIT_X, INIT_Y, INIT_YAW);

	while(ros::ok()){
		ros::spinOnce();

		if(order_path ==  1 || mode_path == 1){
			mode_path = 1;

			if(pp.cmd_velocity(MAX_SPEED, RANGE_FIN, RANGE_DCL) != 0){
				pp.cmd_angular_v(YAW_GAIN_P, YAW_GAIN_I, YAW_GAIN_D);
				cmd.vx = pp.cmd_vx;
				cmd.vy = pp.cmd_vy;
				cmd.w  = pp.cmd_w;
			}else{
				cmd.vx = 0;
				cmd.vy = 0;
				cmd.w  = 0;

				mode_path = 0;
			}
			
			pub.publish(cmd);
		}
		ROS_FATAL("\nstate_x: %f\tstate_y: %f\tstate_yaw: %f\ncmd_vx: %f\tcmd_vy: %f\tcmd_w: %f\n"
				, pp.state_p.x, pp.state_p.y, pp.state_yaw/M_PI*180, pp.cmd_vx, pp.cmd_vy, pp.cmd_w/M_PI*180);

		rate.sleep();
	}

	return 0;
}

void get_gyro(const std_msgs::Float64::ConstPtr& yaw){
	pp.set_posture(yaw->data*M_PI/180 + INIT_YAW);
}

void get_odom(const abu2021_msgs::odom_rad::ConstPtr& odm){
	static double pre_x = INIT_X;
	static double pre_y = INIT_Y;

	double dx = (odm->x - pre_x)*WHEEL*cos(pp.state_yaw) - (odm->y - pre_y)*WHEEL*sin(pp.state_yaw);
	double dy = (odm->x - pre_x)*WHEEL*sin(pp.state_yaw) + (odm->y - pre_y)*WHEEL*cos(pp.state_yaw);

	pp.set_position(pp.state_p.x + dx, pp.state_p.y + dy);

	pre_x = odm->x;
	pre_y = odm->y;
}

void get_path(const std_msgs::Int16::ConstPtr& path){
	order_path = path->data;
}




/************************************************************/
/********** Point class *************************************/
/************************************************************/

//コンストラクタ
Point::Point(double x, double y){
	this->x = x;
	this->y = y;
}

//コピーコンストラクタ（vector<Point>用）
Point::Point(const Point& point){
	this->x = point.x;
	this->y = point.y;
}

//座標変化
double Point::change_to(double x, double y){
	this->x = x;
	this->y = y;
}

//2点間距離
double Point::dist(double x, double y){
	return sqrt(pow(x - this->x, 2) + pow(y - this->y, 2));
}
double Point::dist(Point point){
	return sqrt(pow(point.x - this->x, 2) + pow(point.y - this->y, 2));
}

//表示
double Point::print(){
	printf("x:%f\ty:%f\n", x, y);
}




/*******************************************************************/
/********** Pure_pursuit class *************************************/
/*******************************************************************/

/******* 設定 *******/
//コンストラクタ
/* Pure_pursuit::Pure_pursuit(std::string file_name, int ahead_num){ */
/* 	this->file_name = file_name; */
/* 	this->ahead_num = ahead_num; */

/* 	load_csv(); */
/* } */

//経路再設定
void Pure_pursuit::reset_path(std::string file_name, int ahead_num){
	this->file_name = file_name;
	this->ahead_num = ahead_num;

	load_csv();
}

//ロボ状態設定
void Pure_pursuit::set_state(Point pos, double yaw){
	this->state_p = pos;
	this->state_yaw = yaw;
}
void Pure_pursuit::set_state(double x, double y, double yaw){
	this->state_p.x = x;
	this->state_p.y = y;
	this->state_yaw = yaw;
}

//ロボ位置設定
void Pure_pursuit::set_position(Point pos){
	this->state_p = pos;
}
void Pure_pursuit::set_position(double x, double y){
	this->state_p.x = x;
	this->state_p.y = y;
}

//ロボ姿勢設定
void Pure_pursuit::set_posture(double yaw){
	this->state_yaw = yaw;
}

/******* 司令 *******/
//角速度司令[rad/s]
void Pure_pursuit::cmd_angular_v(double p, double i, double d){
	double trgt_dir = target_dir_global(); //目標角
	static double sum_yaw = 0;
	static double pre_yaw = 0;

	sum_yaw += trgt_dir - state_yaw;
	pre_yaw = state_yaw;

	//PID制御
	cmd_w = p*(trgt_dir - state_yaw) + i*sum_yaw - d*(state_yaw - pre_yaw);
}

//速度司令[m/s]
double Pure_pursuit::cmd_velocity(double max_speed, double fin, double dcl){
	double speed; //速さ（スカラー）

	//終端点との距離に応じ速さ変更
	if(dist_fin() < fin)      speed = 0; //終了判定
	else if(dist_fin() < dcl) speed = max_speed * (dist_fin() - fin) / (dcl - fin); //台形制御
	else                      speed = max_speed;

	cmd_vx = speed * cos(target_dir_local());
	cmd_vy = speed * sin(target_dir_local());

	return speed;
}

//経路点列終端との距離
double Pure_pursuit::dist_fin(){
	return state_p.dist(path.back());
}


/******* 目標角度 *******/
//目標角度[rad](ローカル角度)
double Pure_pursuit::target_dir_local(){
	return target_dir_global() - state_yaw;
}

//目標角度[rad](グローバル角度)
double Pure_pursuit::target_dir_global(){
	Point trgt = target_point(); //目標点
	return atan2(trgt.y - state_p.y, trgt.x - state_p.x); //偏角[rad](グローバル角度)
}

//目標点
Point Pure_pursuit::target_point(){
	std::vector<double> dist; //各点との距離

	//距離計算
	for(int i=0; i<path.size(); i++){
		/* dist.push_back(path->at(i).dist(state_p)); */
		dist.push_back(state_p.dist(path.at(i)));
	}
	//最近経路点
	std::vector<double>::iterator itr = std::min_element(dist.begin(), dist.end());
	int index = std::distance(dist.begin(), itr);
	//最近経路点からahead_num個先の点を目標点とする
	if(path.size() > index+ahead_num){
		return path.at(index + ahead_num);
	}else{
		return path.back();
	}
}

/******* デバッグ *******/
//経路点列表示
int Pure_pursuit::print_path(){
	for(int i=0; i<path.size(); i++) path.at(i).print();
}

/******* CSVファイル読み込み *******/
//CSVファイル読み込み
int Pure_pursuit::load_csv(){
	std::ifstream file(file_name);

	// check file open
	if(file.fail()){
		printf("Failed to open file\n");
		return -1;
	}

	// get file
	while(1){
		std::string line_x;
		std::string line_y;

		//取得
		getline(file, line_x, ',');
		getline(file, line_y);

		//ファイル終端判定
		if(file.eof()) break;

		//pathに追加
		Point tmp(stod(line_x), stod(line_y));
		path.push_back(tmp);
	}

	file.close();

	return 0;
}
