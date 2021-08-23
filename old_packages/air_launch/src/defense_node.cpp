
#include <ros/ros.h>
#include <std_msgs/Int16.h>

//defense_to_arduinoにパブリッシュ
//defense_ordderをサブスクライブ


std_msgs::Int16 pub_msg;
ros::Publisher  pub;
ros::Subscriber sub;


int onoff=0;
int speed=-200;
int pw=0;

//コールバック関数
void chatterCallback(const std_msgs::Int16::ConstPtr & sub_msg){
	ROS_FATAL("I heeard order");
	onoff=sub_msg->data;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "defense_node");
	ros::NodeHandle nh;
	pub = nh.advertise<std_msgs::Int16>("defense_to_ardiuno", 1);
	//ros::Subscriber sub = nh.subscribe("air_launch_enc", 1, chatterCallback);
	sub = nh.subscribe("defense_order", 1, chatterCallback);
	ros::Rate loop_rate(10);
	while (ros::ok()){
		ros::spinOnce();
		
		if(onoff==0){
			pw=0;
		}
		if(onoff==1){
			pw=speed;
			//ROS_INFO("I am defense_node2");
		}
		pub_msg.data =pw;
		pub.publish(pub_msg);
		
		loop_rate.sleep();
	}
}


