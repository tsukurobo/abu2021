#include "ros/ros.h"
#include "abu2021_msgs/launch_act.h"
#include "abu2021_msgs/launch_sens.h"
#include "abu2021_msgs/rack_msg.h"
#include "abu2021_msgs/tr_order.h"


//parameter
//common
int FREQ = 100; // [Hz] 制御周期
int SOL_HIGH_TIME = 200; //[msec] エア射出のソレノイドonする時間
//air
int AIR_HL = 1; //エア電磁弁HIGH-LOWに対応する司令
int AIR_LH = 2; //エア電磁弁LOW-HIGHに対応する司令
int AIR_LAUNCH_TIME = 2000; //[msec] エア射出のシリンダ伸ばす時間
//const
double TARGET_ROT_1 = 0.5; //[rotate] コンスト射出の引っ張る回転数
double TARGET_ROT_2 = 1.65;//[rotate] コンスト射出の引っ張る回転数
double TARGET_ROT_3 = 2.0; //[rotate] コンスト射出の引っ張る回転数
int MOT_POW_UP = 100;   //ロック機構上昇時のモータパワー[0-255]
int MOT_POW_DOWN = -100; //ロック機構下降時のモータパワー[0-255]
int CONST_HIGH_TIME = 200; //[msec] コンスト射出のソレノイドonする時間
int CONST_WAIT_TIME = 200; //[msec] コンスと射出のロック解除後待つ時間
int SOL_LOW  = 0; //ソレノイドLOWに対応する司令
int SOL_HIGH = 1; //ソレノイドHIGHに対応する司令
int TOUCH_LOCK_ON  = 1; //タッチセンサ，ロックONに対応する司令
int TOUCH_LOCK_OFF = 0; //タッチセンサ，ロックOFFに対応する司令
//rack
int R_AIR_INI1 = 0;
int R_AIR_INI2 = 0;
int R_AIR_PICK1_1 = 0;
int R_AIR_PICK1_2 = 0;
int R_AIR_PICK2_1 = 0;
int R_AIR_PICK2_2 = 0;
int R_AIR_LOAD1 = 0;
int R_AIR_LOAD2 = 0;
int R_AIR_UP1_1 = 0;
int R_AIR_UP1_2 = 0;
int R_AIR_UP2_1 = 0;
int R_AIR_UP2_2 = 0;
int R_AIR_HL = 1;
int R_AIR_LH = 2;

int R_CR_INI1 = 0;
int R_CR_INI2 = 0;
int R_CR_PICK1_1 = 0;
int R_CR_PICK1_2 = 0;
int R_CR_PICK2_1 = 0;
int R_CR_PICK2_2 = 0;
int R_CR_LOAD1 = 0;
int R_CR_LOAD2 = 0;
int R_CR_UP1_1 = 0;
int R_CR_UP1_2 = 0;
int R_CR_UP2_1 = 0;
int R_CR_UP2_2 = 0;
int R_CR_HL = 1;
int R_CR_LH = 2;

int R_CL_INI1 = 0;
int R_CL_INI2 = 0;
int R_CL_PICK1_1 = 0;
int R_CL_PICK1_2 = 0;
int R_CL_PICK2_1 = 0;
int R_CL_PICK2_2 = 0;
int R_CL_LOAD1 = 0;
int R_CL_LOAD2 = 0;
int R_CL_UP1_1 = 0;
int R_CL_UP1_2 = 0;
int R_CL_UP2_1 = 0;
int R_CL_UP2_2 = 0;
int R_CL_HL = 1;
int R_CL_LH = 2;

int R_PICK_UP = -5;
int R_DELAY_TIME = 500;

//ROS message
abu2021_msgs::launch_act data_launch;
abu2021_msgs::rack_msg data_rack;
abu2021_msgs::launch_act data_launch_pre;
abu2021_msgs::rack_msg data_rack_pre;

//state
//センサ値
double sens_enc = 0;
int sens_touch = 0;
//上流からの司令
int order_air = 0;
int order_c_ready = 0;
int order_c_launch = 0;
int order_rack = 0;
int order_stop = 0;
//タスクのステップ
int step_air = 0;
int step_c_ready = 0;
int step_c_launch = 0;
int step_rack = 0;
//基準位置（タッチセンサが触れ始めた位置）[deg]
int deg_standard = 0;

//function prototype
void task_air();
void task_const_ready();
void task_const_launch();
void task_rack();
void set_rack_vlue(int launch, int srv1, int srv2);
void set_rack_hand(int sol);
void all_stop();
void get_launch_sensor(const abu2021_msgs::launch_sens::ConstPtr& msg);
void get_order(const abu2021_msgs::tr_order::ConstPtr& msg);

//main
int main(int argc, char **argv){
	ros::init(argc, argv, "tr_mechanism_node");

	ros::NodeHandle nh;

	ros::Publisher  pub_launch = nh.advertise<abu2021_msgs::launch_act>("launch_act", 1);
	ros::Publisher  pub_rack   = nh.advertise<abu2021_msgs::rack_msg>("rack_tpc", 1);
	ros::Subscriber sub_sens   = nh.subscribe("launch_sens", 1, get_launch_sensor);
	ros::Subscriber sub_order  = nh.subscribe("tr_order", 1, get_order);

	//parameter
	nh.getParam("common/freq", FREQ);
	nh.getParam("common/sol_high_time", SOL_HIGH_TIME);
	nh.getParam("air/launch_time", AIR_LAUNCH_TIME);
	nh.getParam("air/hl", AIR_HL);
	nh.getParam("air/lh", AIR_LH);

	nh.getParam("const/target_rot_1", TARGET_ROT_1);
	nh.getParam("const/target_rot_2", TARGET_ROT_2);
	nh.getParam("const/target_rot_3", TARGET_ROT_3);
	nh.getParam("const/mot_pow_up", MOT_POW_UP);
	nh.getParam("const/mot_pow_down", MOT_POW_DOWN);
	nh.getParam("const/sol_high_time", CONST_HIGH_TIME);
	nh.getParam("const/sol_wait_time", CONST_WAIT_TIME);
	nh.getParam("const/sol_low", SOL_LOW);
	nh.getParam("const/sol_high", SOL_HIGH);
	nh.getParam("const/touch_lock_on", TOUCH_LOCK_ON);
	nh.getParam("const/touch_lock_off", TOUCH_LOCK_OFF);

	nh.getParam("rack/air/deg_ini1",   R_AIR_INI1   );
	nh.getParam("rack/air/deg_ini2",   R_AIR_INI2   );
	nh.getParam("rack/air/deg_pick1",  R_AIR_PICK1_1);
	nh.getParam("rack/air/deg_pick2",  R_AIR_PICK1_2);
	nh.getParam("rack/air/deg_pick2_1",R_AIR_PICK2_1);
	nh.getParam("rack/air/deg_pick2_2",R_AIR_PICK2_2);
	nh.getParam("rack/air/deg_load1",  R_AIR_LOAD1  );
	nh.getParam("rack/air/deg_load2",  R_AIR_LOAD2  );
	nh.getParam("rack/air/deg_up1",    R_AIR_UP1_1  );
	nh.getParam("rack/air/deg_up2",    R_AIR_UP1_2  );
	nh.getParam("rack/air/deg_up2_1",  R_AIR_UP2_1  );
	nh.getParam("rack/air/deg_up2_2",  R_AIR_UP2_2  );
	nh.getParam("rack/air/hl",         R_AIR_HL     );
	nh.getParam("rack/air/lh",         R_AIR_LH     );

	nh.getParam("rack/const_r/deg_ini1",   R_CR_INI1   );
	nh.getParam("rack/const_r/deg_ini2",   R_CR_INI2   );
	nh.getParam("rack/const_r/deg_pick1",  R_CR_PICK1_1);
	nh.getParam("rack/const_r/deg_pick2",  R_CR_PICK1_2);
	nh.getParam("rack/const_r/deg_pick2_1",R_CR_PICK2_1);
	nh.getParam("rack/const_r/deg_pick2_2",R_CR_PICK2_2);
	nh.getParam("rack/const_r/deg_load1",  R_CR_LOAD1  );
	nh.getParam("rack/const_r/deg_load2",  R_CR_LOAD2  );
	nh.getParam("rack/const_r/deg_up1",    R_CR_UP1_1  );
	nh.getParam("rack/const_r/deg_up2",    R_CR_UP1_2  );
	nh.getParam("rack/const_r/deg_up2_1",  R_CR_UP2_1  );
	nh.getParam("rack/const_r/deg_up2_2",  R_CR_UP2_2  );
	nh.getParam("rack/const_r/hl",         R_CR_HL     );
	nh.getParam("rack/const_r/lh",         R_CR_LH     );

	nh.getParam("rack/const_l/deg_ini1",   R_CL_INI1   );
	nh.getParam("rack/const_l/deg_ini2",   R_CL_INI2   );
	nh.getParam("rack/const_l/deg_pick1",  R_CL_PICK1_1);
	nh.getParam("rack/const_l/deg_pick2",  R_CL_PICK1_2);
	nh.getParam("rack/const_l/deg_pick2_1",R_CL_PICK2_1);
	nh.getParam("rack/const_l/deg_pick2_2",R_CL_PICK2_2);
	nh.getParam("rack/const_l/deg_load1",  R_CL_LOAD1  );
	nh.getParam("rack/const_l/deg_load2",  R_CL_LOAD2  );
	nh.getParam("rack/const_l/deg_up1",    R_CL_UP1_1  );
	nh.getParam("rack/const_l/deg_up2",    R_CL_UP1_2  );
	nh.getParam("rack/const_l/deg_up2_1",  R_CL_UP2_1  );
	nh.getParam("rack/const_l/deg_up2_2",  R_CL_UP2_2  );
	nh.getParam("rack/const_l/hl",         R_CL_HL     );
	nh.getParam("rack/const_l/lh",         R_CL_LH     );

	nh.getParam("rack/delay_time", R_DELAY_TIME);
	nh.getParam("rack/pick_up_deg", R_PICK_UP);

	ros::Rate loop_rate(FREQ);

	//body
	while (ros::ok()){
		ros::spinOnce();

		//task
		if(order_air      == 1 || step_air      > 0) task_air();
		if(order_c_ready   > 0 || step_c_ready  > 0) task_const_ready();
		if(order_c_launch == 1 || step_c_launch > 0) task_const_launch();
		if(order_rack      > 0 || step_rack     > 0) task_rack();
		if(order_stop == 1) all_stop();

		//publish
		if(data_launch != data_launch_pre) pub_launch.publish(data_launch);
		if(data_rack   != data_rack_pre  ) pub_rack.publish(data_rack);

		//pre publish data update
		data_launch_pre = data_launch;
		data_rack_pre = data_rack;

		loop_rate.sleep();
	}
	return 0;
}

//tasks
void task_air(){
	static int cnt = 0;

	if(step_air == 0){
		data_launch.air = 0;
		step_air = 1;
	}else if(step_air == 1){ //h-l
		data_launch.air = AIR_HL;
		cnt++;
		if(cnt > SOL_HIGH_TIME*FREQ/1000){ //high時間過ぎたら次へ
			step_air = 2;
		}
	}else if(step_air == 2){ //l-l
		data_launch.air = 0;
		cnt++;
		if(cnt > AIR_LAUNCH_TIME*FREQ/1000){ //launch時間過ぎたら次へ
			step_air = 3;
			cnt = 0;
		}
	}else if(step_air == 3){ //l-h
		data_launch.air = AIR_LH;
		cnt++;
		if(cnt > SOL_HIGH_TIME*FREQ/1000){ //high時間過ぎたらl-l
			data_launch.air = 0;
			step_air = 0;
			cnt = 0;
		}
	}
}


void task_const_ready(){
	static double rot_pre = 0;
	static double target_rot = 0;

	if(step_c_ready == 0){
		data_launch.motor = 0;
		data_launch.solenoid = SOL_LOW;
		if(order_c_ready == 1) target_rot = TARGET_ROT_2;
		if(order_c_ready == 2) target_rot = TARGET_ROT_3;
		step_c_ready = 1;
	}else if(step_c_ready == 1){ // 下降
		data_launch.motor = MOT_POW_DOWN;
		if(abs(sens_enc - deg_standard)/360.0f > target_rot){ //射出位置まで来たら終了
			data_launch.motor = 0;
			step_c_ready = 0;
		}
	}
}

void task_const_launch(){
	static int cnt = 0;

	if(step_c_launch == 0){
		data_launch.motor = 0;
		data_launch.solenoid = SOL_LOW;
		step_c_launch = 1;
	}else if(step_c_launch == 1){ // solenoid on
		data_launch.solenoid = SOL_HIGH;
		cnt++;
		if(cnt > CONST_HIGH_TIME*FREQ/1000){ //high時間過ぎたら次へ
			step_c_launch = 2;
			cnt = 0;
		}
	}else if(step_c_launch == 2){ // solenoid off
		data_launch.solenoid = SOL_LOW;
		cnt++;
		if(cnt > CONST_WAIT_TIME*FREQ/1000){ //wait時間過ぎたら次へ
			step_c_launch = 3;
			cnt = 0;
		}
	}else if(step_c_launch == 3){ // 上昇
		data_launch.motor = MOT_POW_UP;
		if(sens_touch == TOUCH_LOCK_ON){ //ロックかかったら次へ
			deg_standard = sens_enc;
			step_c_launch = 4;
		}
	}else if(step_c_launch == 4){ // 下降
		data_launch.motor = MOT_POW_DOWN;
		if(abs(sens_enc - deg_standard)/360.0f > TARGET_ROT_1){ //装填位置まで下がったら終了
			data_launch.motor = 0;
			step_c_launch = 0;
		}
	}
}

void task_rack(){
	const int AIR = 1;
	const int C_R = 2;
	const int C_L = 3;

	const int INIT = 1;
	const int DOWN1 = 2;
	const int DOWN2 = 3;
	const int CLOSE = 4;
	const int UP    = 5;
	const int LOAD  = 6;
	const int OPEN  = 7;

	static int order = 0;
	static int cnt = 0;

	set_rack_hand(0);

	//orderにタスクの数字を代入（0以外の時）（orderはタスク終了するまで0にならない）
	if(order_rack != 0) order = order_rack;

	switch(order){
		case INIT:
			set_rack_vlue(AIR, R_AIR_INI1, R_AIR_INI2);
			set_rack_vlue(C_R, R_CR_INI1, R_CR_INI2);
			set_rack_vlue(C_L, R_CL_INI1, R_CL_INI2);
			order = 0;
			break;
		case DOWN1:
			set_rack_vlue(AIR, R_AIR_PICK1_1, R_AIR_PICK1_2);
			set_rack_vlue(C_R, R_CR_PICK1_1, R_CR_PICK1_2);
			set_rack_vlue(C_L, R_CL_PICK1_1, R_CL_PICK1_2);
			order = 0;
			break;
		case DOWN2:
			set_rack_vlue(AIR, R_AIR_PICK2_1, R_AIR_PICK2_2);
			set_rack_vlue(C_R, R_CR_PICK2_1, R_CR_PICK2_2);
			set_rack_vlue(C_L, R_CL_PICK2_1, R_CL_PICK2_2);
			order = 0;
			break;
		case CLOSE:
			if(step_rack == 0){
				step_rack = 1;
				set_rack_hand(1);
			}else if(step_rack == 1){
				cnt++;
				if(cnt > SOL_HIGH_TIME*FREQ/1000){
					set_rack_hand(0);
					step_rack = 2;
				}
			}else if(step_rack == 2){
				cnt++;
				if(cnt > R_DELAY_TIME*FREQ/1000){
					step_rack = 0;
					set_rack_vlue(AIR, R_AIR_PICK1_1+R_PICK_UP, R_AIR_PICK1_2);
					set_rack_vlue(C_R, R_CR_PICK1_1+R_PICK_UP, R_CR_PICK1_2);
					set_rack_vlue(C_L, R_CL_PICK1_1+R_PICK_UP, R_CL_PICK1_2);
					cnt = 0;
					order = 0;
				}
			}
			break;
		case UP:
			if(step_rack == 0){
				step_rack = 1;
				set_rack_vlue(AIR, R_AIR_UP1_1, R_AIR_UP1_2);
				set_rack_vlue(C_R, R_CR_UP1_1, R_CR_UP1_2);
				set_rack_vlue(C_L, R_CL_UP1_1, R_CL_UP1_2);
			}else if(step_rack == 1){
				cnt++;
				if(cnt > R_DELAY_TIME*FREQ/1000){
					set_rack_vlue(AIR, R_AIR_UP2_1, R_AIR_UP2_2);
					set_rack_vlue(C_R, R_CR_UP2_1, R_CR_UP2_2);
					set_rack_vlue(C_L, R_CL_UP2_1, R_CL_UP2_2);
					step_rack = 0;
					cnt = 0;
					order = 0;
				}
			}
			break;
		case LOAD:
			set_rack_vlue(AIR, R_AIR_LOAD1, R_AIR_LOAD2);
			set_rack_vlue(C_R, R_CR_LOAD1, R_CR_LOAD2);
			set_rack_vlue(C_L, R_CL_LOAD1, R_CL_LOAD2);
			order = 0;
			break;
		case OPEN:
			if(step_rack == 0){
				step_rack = 1;
			}else if(step_rack == 1){
				set_rack_hand(2);
				cnt++;
				if(cnt > SOL_HIGH_TIME*FREQ/1000){
					step_rack = 0;
					set_rack_hand(0);
					cnt = 0;
					order = 0;
				}
			}
			break;
		default:
			break;
	}
}

void set_rack_vlue(int launch, int srv1, int srv2){
	const int AIR = 1;
	const int CONST_R = 2;
	const int CONST_L = 3;

	if(launch == AIR){
		data_rack.air_1 = srv1;
		data_rack.air_2 = srv2;
	}else if(launch == CONST_R){
		data_rack.const_r_1 = srv1;
		data_rack.const_r_2 = srv2;
	}else if(launch == CONST_L){
		data_rack.const_l_1 = srv1;
		data_rack.const_l_2 = srv2;
	}
}

void set_rack_hand(int sol){
	if(sol == 1){
		data_rack.air_hand = R_AIR_HL;
		data_rack.const_r_hand = R_CR_HL;
		data_rack.const_l_hand = R_CR_HL;
	}else if(sol == 2){
		data_rack.air_hand = R_AIR_LH;
		data_rack.const_r_hand = R_CR_LH;
		data_rack.const_l_hand = R_CR_LH;
	}else{
		data_rack.air_hand = 0;
		data_rack.const_r_hand = 0;
		data_rack.const_l_hand = 0;
	}
}

void all_stop(){
	data_launch.air = 0;
	data_launch.motor = 0;
	data_launch.solenoid = 0;
	step_air = 0;
	step_c_ready = 0;
	step_c_launch = 0;
	step_rack = 0;
}

//subscribe callback
void get_launch_sensor(const abu2021_msgs::launch_sens::ConstPtr& msg){
	sens_enc = msg->enc;
	sens_touch = msg->touch;
}

void get_order(const abu2021_msgs::tr_order::ConstPtr& msg){
	order_air      = msg->air;
	order_c_ready  = msg->const_ready;
	order_c_launch = msg->const_launch;
	order_rack     = msg->rack;
	order_stop     = msg->emg_stop;
}
