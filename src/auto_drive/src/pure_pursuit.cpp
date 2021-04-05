#include "pure_pursuit.h"

//コンストラクタ
Pure_pursuit::Pure_pursuit(std::string file_name, int ahead_num){
	this->file_name = file_name;
	this->ahead_num = ahead_num;

	load_csv();
}

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
void Pure_pursuit::cmd_angular_v(double p, double i, double d){
	double angle = target_dir_global() - state_yaw; //姿勢角と目標角との偏角
	static double sum_yaw = 0;
	static double pre_yaw = 0;

	//偏角定義域修正
	if(angle > M_PI)       while(angle >  M_PI) angle -= 2*M_PI;	
	else if(angle < -M_PI) while(angle < -M_PI) angle += 2*M_PI;

	sum_yaw += angle;
	pre_yaw = state_yaw;

	//PID制御
	cmd_w = p*angle + i*sum_yaw - d*(state_yaw - pre_yaw);
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

