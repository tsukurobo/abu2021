#include "pure_pursuit.h"

/******* 設定 *******/
//コンストラクタ
Pure_pursuit::Pure_pursuit(std::string file_name, int ahead_num){
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

/******* 司令 *******/
//角速度司令[rad/s]
void Pure_pursuit::cmd_angular_v(double p, double i, double d){
	double trgt_dir = target_dir_local(); //目標角
	static double sum_yaw = 0;
	static double pre_yaw = 0;
	printf("%f\n", trgt_dir);
	sum_yaw += trgt_dir - state_yaw;
	pre_yaw = state_yaw;

	cmd_w = p*(trgt_dir - state_yaw) + i*sum_yaw - d*(state_yaw - pre_yaw);
}

//速度司令[m/s]
void Pure_pursuit::cmd_velocity(double max_speed, double fin, double dcl){
	double speed; //速さ（スカラー）

	if(dist_fin() < fin)      speed = 0; //終了判定
	else if(dist_fin() < dcl) speed = max_speed * (dist_fin() - fin) / (dcl - fin); //台形制御
	else                      speed = max_speed;

	cmd_vx = speed * cos(target_dir_local());
	cmd_vy = speed * sin(target_dir_local());
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

