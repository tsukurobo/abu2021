#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"

//topicにエンコーダの値くる
//inをサブスクライブ、outにパブリッシュ
//できた


ros::Publisher  pub;
ros::Subscriber sub;

int subed_enc = 0;
void chatterCallback(const std_msgs::Int64::ConstPtr& msg){
	//メッセージのオブジェクト
	std_msgs::Int64 hoge;
	
	//hogeがパブリッシュするhogeの中に購読したデータそのまま入れる
	
	hoge.data = msg->data;
	subed_enc = msg->data;
	pub.publish(hoge);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "pub_enc");
	ros::NodeHandle nh;

	pub = nh.advertise<std_msgs::Int64>("out", 1);
	//outっていうトピックにパブリッシュするよ1はどれくらい貯めるかの数字
	sub = nh.subscribe("in", 1, chatterCallback);
	//inっていうトピックをサブスクライブするよ、サブスクライブしたらcallbackするよ
	


	ROS_FATAL("%d from pub_enc", subed_enc);

	ros::spin();

	return 0;
}
