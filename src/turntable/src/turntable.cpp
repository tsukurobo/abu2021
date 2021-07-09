#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include <abu2021_msgs/turn_and_dist.h>

ros::Publisher pub;
ros::Subscriber sub;

std_msgs::Int16MultiArray mode;

int pw_dist = 150; //妨害の速度

void order_callback(const abu2021_msgs::turn_and_dist& t_d){
	mode.data.resize(4);
	//buttons[1]を押しているときのみ右アームを開く
	if(t_d.solenoid_r == 1){//右アームを開く
    		mode.data[0] = 1;
	}else{//右アームを閉じる
		mode.data[0] = 0;
	}

	//buttons[2]を押しているときのみ左アームを開く
	if(t_d.solenoid_l == 1){//左アームを開く
		mode.data[1] = 1;
	}else{//左アームを閉じる
		mode.data[1] = 0;
	}

	//buttons[3]を押しているときのみ妨害する
	if(t_d.dist == 1) {//妨害する
		mode.data[2] = 1;
	}else{				//妨害しない
		mode.data[2] = 0;
	}

	pub.publish(mode);
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

	ros::spin();

	return 0;
}
