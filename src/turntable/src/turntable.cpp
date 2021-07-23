#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include <abu2021_msgs/turn_and_dist.h>

ros::Publisher pub;
ros::Subscriber sub;

std_msgs::Int16MultiArray mode;
std_msgs::Int16MultiArray pre_mode; ///////////////////////////値が更新されないとpubされない仕様に

int pw_dist = 150; //妨害の速度
int time_count = 50;

int count_r_o = 0;
int count_r_c = 0;
int count_l_o = 0;
int count_l_c = 0;

void order_callback(const abu2021_msgs::turn_and_dist& t_d){
	//mode.data.resize(4); ////////////////////////////////////コメントアウトしても大丈夫っぽい？

	//buttons[1]を押しているときのみ右アームを開く
	if(t_d.solenoid_r == 1){//右アームを開く
		mode.data[0] = 2;
	}else{//右アームを閉じる
		mode.data[0] = 1;
	}

	//buttons[2]を押しているときのみ左アームを開く
	if(t_d.solenoid_l == 1){//左アームを開く
		mode.data[1] = 2;
	}else{//左アームを閉じる
		mode.data[1] = 1;
	}

	//buttons[3]を押しているときのみ妨害する
	if(t_d.dist == 1) {//妨害する
		mode.data[2] = 1;
	}else{				//妨害しない
		mode.data[2] = 0;
	}

	//pub.publish(mode);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "joy_op");
	ros::NodeHandle nh;
	
	pub = nh.advertise<std_msgs::Int16MultiArray>("turn_and_dist", 1);
	sub = nh.subscribe("turn_and_dist_order", 1, order_callback);
	
	ros::NodeHandle nnh("~");
	nnh.getParam("pw_dist", pw_dist);

	mode.data.resize(4);
	mode.data[3] = pw_dist;

	pre_mode.data.resize(4);
	pre_mode.data[0] = -1;
	pre_mode.data[1] = -1;
	pre_mode.data[2] = -1;
	pre_mode.data[3] = -1;

	//ros::spin();
	ros::Rate loop_rate(100);
	while(ros::ok()) {
		ros::spinOnce();
		
		if (mode.data[0] == 2) {count_r_o++; count_r_c = 0;}
		else if (mode.data[0] == 1) {count_r_c++; count_r_o = 0;}
		if (count_r_o > time_count || count_r_c > time_count) mode.data[0] = 0;

		if (mode.data[1] == 2) {count_l_o++; count_l_c = 0;}
		else if (mode.data[1] == 1) {count_l_c++; count_l_o = 0;}
		if (count_l_o > time_count || count_l_c > time_count) mode.data[1] = 0;
		
		


		//値が更新されたらpublishする
		if(mode.data[0]!=pre_mode.data[0] ||mode.data[1]!=pre_mode.data[1]
				|| mode.data[2]!=pre_mode.data[2] || mode.data[3]!=pre_mode.data[3]){
			pub.publish(mode);
		}

		//preの更新
		pre_mode = mode;

		loop_rate.sleep();
	}

	return 0;
}
