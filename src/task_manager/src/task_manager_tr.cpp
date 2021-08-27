#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <abu2021_msgs/cmd_vw.h>
#include <abu2021_msgs/tr_order.h>
#include <abu2021_msgs/rack_msg.h>

#define ONE buttons[0]
#define TWO buttons[1]
#define THREE buttons[2]
#define FOUR buttons[3]
/* #define AX_UPDOWN axes[5] */
#define AX_UD axes[5]
#define AX_LR axes[4]
#define LB buttons[4]
#define LT buttons[6]
#define RB buttons[5]
#define RT buttons[7]

#define PUSHED 1
#define UP 1
#define DOWN -1
#define LEFT 1
#define RIGHT -1

#define CONST_LAUNCH 0
#define AIR_LAUNCH 1
#define RACK_COL 2

ros::Publisher pub;
ros::Publisher pub_tr;
ros::Publisher pub_rack;

int srv[6] = {90, 90, 90, 90, 90, 90};

void get_joy(const sensor_msgs::Joy::ConstPtr& msg);
void rack_adjust(int mode, int val);
void get_rack(const abu2021_msgs::rack_msg::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "task_manager_TR");

	ros::NodeHandle nh;

	pub    = nh.advertise<abu2021_msgs::cmd_vw>("cmd", 1);
	pub_tr = nh.advertise<abu2021_msgs::tr_order>("tr_order", 1);
	pub_rack = nh.advertise<abu2021_msgs::rack_msg>("rack_tpc", 1);
	ros::Subscriber sub = nh.subscribe("joy", 1, get_joy);
	ros::Subscriber sub_rack = nh.subscribe("rack_tpc", 1, get_rack);

	ros::spin();

	return 0;
}

void get_joy(const sensor_msgs::Joy::ConstPtr& msg){
	static abu2021_msgs::cmd_vw cmd;
	static abu2021_msgs::tr_order order;
	static abu2021_msgs::tr_order order_pre;

	//move
	cmd.vx = 6.0*msg->axes[1];
	cmd.vy = 6.0*msg->axes[0];
	cmd.w  = 4.0*msg->axes[3];
	pub.publish(cmd);

	//mechanism
	order.air = 0;
	order.const_ready = 0;
	order.const_launch = 0;
	order.rack = 0;
	order.emg_stop = 0;

	//air
	if ((msg->RB == PUSHED) && (msg->TWO == PUSHED)) order.air = 1;
	//const
	if(msg->LB == PUSHED){
		if      (msg->ONE   == PUSHED) order.emg_stop = 1;
		else if (msg->TWO   == PUSHED) order.const_launch = 1;
		else if (msg->FOUR  == PUSHED) order.const_ready = 1;
		else if (msg->THREE == PUSHED && msg->LT != PUSHED) order.const_ready = 2;
		else if (msg->THREE == PUSHED && msg->LT == PUSHED) order.const_ready = 3;
	}
	//rack
	if(msg->LB != PUSHED && msg->RB != PUSHED && msg->RT != PUSHED){
		if     (msg->AX_LR == LEFT ) order.rack = 1;
		else if(msg->AX_UD == UP   ) order.rack = 2;
		else if(msg->AX_LR == RIGHT) order.rack = 3;
		else if(msg->AX_UD == DOWN ) order.rack = 4;
	}else if(msg->LB == PUSHED){
		if     (msg->AX_LR == LEFT ) order.rack = 5;
		else if(msg->AX_LR == RIGHT) order.rack = 6;
		else if(msg->AX_UD == UP   ) order.rack = 7;
	}else if(msg->RB == PUSHED){
		if     (msg->AX_LR == LEFT ) order.rack = 8;
		else if(msg->AX_LR == RIGHT) order.rack = 9;
		else if(msg->AX_UD == UP   ) order.rack = 10;
	}

	if(order != order_pre) pub_tr.publish(order);

	order_pre = order;
	
	//rack調整モード
	if(msg->RT == PUSHED){
		if     (msg->AX_LR == LEFT ) rack_adjust( 1, 0);
		else if(msg->AX_LR == RIGHT) rack_adjust(-1, 0);
		if(msg->LT == PUSHED){
			if     (msg->AX_UD == UP  ) rack_adjust( 0, 5);
			else if(msg->AX_UD == DOWN) rack_adjust( 0,-5);
		}else{
			if     (msg->AX_UD == UP  ) rack_adjust( 0, 1);
			else if(msg->AX_UD == DOWN) rack_adjust( 0,-1);
		}
	}
	
}

void rack_adjust(int mode, int val){
	static abu2021_msgs::rack_msg order;
	static int which_srv = 0;

	which_srv += mode;
	if(which_srv < 0) which_srv = 5;
	else if (which_srv > 5) which_srv = 0;

	srv[which_srv] += val;
	if     (srv[which_srv] > 180) srv[which_srv] = 180;
	else if(srv[which_srv] <   0) srv[which_srv] = 0;

	order.air_r_1 = srv[0];
	order.air_r_2 = srv[1];
	order.air_l_1 = srv[2];
	order.air_l_2 = srv[3];
	order.const_1 = srv[4];
	order.const_2 = srv[5];

	pub_rack.publish(order);
}

void get_rack(const abu2021_msgs::rack_msg::ConstPtr& msg){
	srv[0] = msg->air_r_1;
	srv[1] = msg->air_r_2;
	srv[2] = msg->air_l_1;
	srv[3] = msg->air_l_2;
	srv[4] = msg->const_1;
	srv[5] = msg->const_2;
}
