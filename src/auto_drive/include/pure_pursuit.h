#ifndef Pure_pursuit_h
#define Pure_pursuit_h

#include "pure_pursuit.h"
#include <math.h>
#include <fstream> //file read
#include <vector>
#include <string>
#include <algorithm> //min_element
#include "point.h"

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
		Pure_pursuit(double dt); //コンストラクタ
		void reset_path(std::string file_name, int ahead_num); //経路再設定

		void set_state(Point pos, double yaw);          //ロボ状態設定
		void set_state(double x, double y, double yaw); //ロボ状態設定
		void set_position(Point pos);          //ロボ位置設定
		void set_position(double x, double y); //ロボ位置設定
		void set_posture(double yaw); //ロボ姿勢設定
		
		void set_pos_pid(double p, double i, double d, double max_w); //pidパラメータ設定
		void cmd_angular_v(double target); //角速度司令
		double target_dir_global(); //目標角度[rad](グローバル角度)
		double cmd_velocity(double speed, double fin, double dcl); //速度司令[m/s]
		int print_path(); //経路点列表示

	
	private:
		double loop_dt;
		//pid変数
		double pid_p;
		double pid_i;
		double pid_d;
		double pid_max;

		//パラメータ
		std::string file_name; //読み込むCSVファイル名
		int ahead_num; //最近経路点からahead_num個先の点を目標点とする
		
		//関数
		double dist_fin(); //経路点列終端との距離[m]
		double target_dir_local(); //目標角度[rad](ローカル角度)
		Point target_point(); //目標点
		int load_csv(); //CSVフィアル読み込み
};

#endif
