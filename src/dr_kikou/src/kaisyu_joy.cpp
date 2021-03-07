//"joy"をサブスクライブ
//pub_enc2もサブスクライブ
//"topic"にパブリッシュ
//奇数回押してオン、偶数回押してオフ
//


#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>

ros::Publisher  pub;
ros::Subscriber sub;
int subed_enc = 2;
int count=0;

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg){
	//メッセージのオブジェクト
	std_msgs::Int16 onoff;
	
	//hogeがパブリッシュするhogeの中に購読したデータそのまま入れる
	if(msg->buttons[12]==1){
		count=count+1;
		if((count%2)==1){
			onoff.data =0;
			pub.publish(onoff);
		}
		if((count%2)==0){
			onoff.data =0;
			pub.publish(onoff);
		}
	}

}
int main(int argc, char **argv){
	ros::init(argc, argv, "joy_kaisyu");
	ros::NodeHandle nh;

	pub = nh.advertise<std_msgs::Int16>("kaisyu_topic", 1);
	//topicっていうトピックにパブリッシュするよ1はどれくらい貯めるかの数字
	sub = nh.subscribe("joy", 1, chatterCallback);
	//joyっていうトピックをサブスクライブするよ、サブスクライブしたらcallbackするよ
	


	ROS_FATAL("%d count from pub_kaisyu", count);

	ros::spin();

	return 0;
}
