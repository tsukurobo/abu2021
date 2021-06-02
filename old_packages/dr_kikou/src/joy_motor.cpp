//"joy"をサブスクライブ
//"topic"にパブリッシュ
//上で正回転、下で逆回転、エンコーダーの値をパブリッシュ
//できた！


#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>

ros::Publisher  pub;
ros::Subscriber sub;
int subed_enc = 2;

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg){
	//メッセージのオブジェクト
	std_msgs::Int16 hoge;
	
	//hogeがパブリッシュするhogeの中に購読したデータそのまま入れる
	if(msg->axes[5]==1){
		hoge.data =100;
		pub.publish(hoge);
	}
	if(msg->axes[5]==-1){
		hoge.data=-100;
		pub.publish(hoge);	
	}
	
}


int main(int argc, char **argv){
	ros::init(argc, argv, "joy_motor");
	ros::NodeHandle nh;

	pub = nh.advertise<std_msgs::Int16>("topic", 1);
	//topicっていうトピックにパブリッシュするよ1はどれくらい貯めるかの数字
	sub = nh.subscribe("joy", 1, chatterCallback);
	//joyっていうトピックをサブスクライブするよ、サブスクライブしたらcallbackするよ
	


	ROS_FATAL("%d from pub_enc", subed_enc);

	ros::spin();

	return 0;
}
