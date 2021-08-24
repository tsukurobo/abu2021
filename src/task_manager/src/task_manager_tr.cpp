#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <abu2021_msgs/cmd_vw.h>
#include <abu2021_msgs/tr_order.h>

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

void get_joy(const sensor_msgs::Joy::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "task_manager_TR");

	ros::NodeHandle nh;

	pub    = nh.advertise<abu2021_msgs::cmd_vw>("cmd", 1);
	pub_tr = nh.advertise<abu2021_msgs::tr_order>("tr_order", 1);
	ros::Subscriber sub = nh.subscribe("joy", 1, get_joy);

	ros::spin();

	return 0;
}

void get_joy(const sensor_msgs::Joy::ConstPtr& msg){
	static abu2021_msgs::cmd_vw cmd;
	static abu2021_msgs::tr_order order;

	//move
	cmd.vx = 6.0*msg->axes[1];
	cmd.vy = 6.0*msg->axes[0];
	cmd.w  = 4.0*msg->axes[3];
	
	pub.publish(cmd);

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
		else if (msg->THREE == PUSHED) order.const_ready = 1;
		else if (msg->FOUR  == PUSHED) order.const_ready = 2;
	}
	//rack
	/* if     (AX_UD == UP) order.rack = 1; */
	/* else if(AX_LR == LEFT) order.rack = 2; */

	pub_tr.publish(order);

	/*
	//LB+(LT+)1~4 -> drive const_launch
	if ((msg->LB == PUSHED) && (msg->ONE == PUSHED)) {
		//order.nodeId = CONST_LAUNCH;
		//order.orderId = 0;
		pub_touteki.publish(order);

	} else if ((msg->LB == PUSHED) && (msg->TWO == PUSHED)) {
		//order.nodeId = CONST_LAUNCH;
		//order.orderId = 2;
		pub_touteki.publish(order);

	} else if ((msg->LB == PUSHED) && (msg->FOUR == PUSHED)) {
		//order.nodeId = CONST_LAUNCH;
		//order.orderId = 1;
		pub_touteki.publish(order);
	
	}  else if ((msg->LB == PUSHED) && (msg->THREE == PUSHED)) {
		//order.nodeId = CONST_LAUNCH;
		//(msg->LT != PUSHED) ? (order.orderId = 3) : (order.orderId = 4);
		pub_touteki.publish(order);
	
	//(RB +) UP, DOWN, LEFT, RIGHT -> drive rack_collection
	} else if (msg->AX_UPDOWN == UP) {
		//order.nodeId = RACK_COL;
		//(msg->LB != PUSHED) ? (order.orderId = 0) : (order.orderId = 6);
		pub_touteki.publish(order);
	
	} else if (msg->AX_UPDOWN == DOWN) {
		//order.nodeId = RACK_COL;
		//(msg->LB != PUSHED) ? (order.orderId = 2) :(order.orderId = 5);
		pub_touteki.publish(order);
	
	} else if (msg->AX_LR == RIGHT) {
		//order.nodeId = RACK_COL;
		//(msg->LB != PUSHED) ? (order.orderId = 1) : (order.orderId = 4);
		pub_touteki.publish(order);

	} else if (msg->AX_LR == LEFT) {
		//order.nodeId = RACK_COL;
		//order.orderId = 3;
		pub_touteki.publish(order);

	}*/ /*else if ((msg->RB == PUSHED) && (msg->AX_UPDOWN == UP)) {
		order.nodeId = RACK_COL;
		(msg->RT != PUSHED) ? (order.orderId = 7) : (order.orderId = 13);
		pub_touteki.publish(order);
	
	} else if ((msg->RB == PUSHED) && (msg->AX_UPDOWN == DOWN)) {
		order.nodeId = RACK_COL;
		(msg->RT != PUSHED) ? (order.orderId = 9) : (order.orderId = 12);
		pub_touteki.publish(order);
	
	} else if ((msg->RB == PUSHED) && (msg->AX_LR == RIGHT)) {
		order.nodeId = RACK_COL;
		(msg->RT != PUSHED) ? (order.orderId =8) : (order.orderId = 11);
		pub_touteki.publish(order);
	
	} else if ((msg->RB == PUSHED) && (msg->AX_LR == LEFT)) {
		order.nodeId = RACK_COL;
		order.orderId = 10;
		pub_touteki.publish(order);
	
	}*/
	
	/*
	//RB+(RT)+1~4 -> drive air launch
	else if ((msg->RB == PUSHED) && (msg->ONE == PUSHED)) {
		if(msg->RT != PUSHED){
			air_order.mode = 0;
			air_order.pow = 0;
			air_order.set = 0;
		}else{
			air_order.mode = 3;
			air_order.pow = 0;
			air_order.set = 0;
		}
		
		pub_air.publish(air_order);
	} else if ((msg->RB == PUSHED) && (msg->THREE == PUSHED)) {
		if(msg->RT != PUSHED){
			air_order.mode = 4;
			air_order.pow = 0;
			air_order.set = 0;
		}else{
			air_order.mode = 1;
			air_order.pow = 0;
			air_order.set = 1;
		}
		pub_air.publish(air_order);	
	} else if ((msg->RB == PUSHED) && (msg->TWO == PUSHED)) {
		if(msg->RT != PUSHED){
			air_order.mode = 2;
			air_order.pow = 0;
			air_order.set = 0;
		}else{
			air_order.mode = 1;
			air_order.pow = power;
			air_order.set = 0;
		}
		pub_air.publish(air_order);
	} else if ((msg->RB == PUSHED) && (msg->FOUR == PUSHED)) {
		air_order.mode = 1;
		air_order.pow = -power;
		air_order.set = 0;
		pub_air.publish(air_order);	
	}
	*/

}
