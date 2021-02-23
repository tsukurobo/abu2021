//"joy"をサブスクライブ
//"pub_enc"もサブスクライブ
//"topic"にパブリッシュ mode_hassya.msg使う　pom/aizu,なんかだめだから配列にした
//4段階調整、横流し
//

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>

ros::Publisher  pub;
ros::Subscriber sub1;
ros::Subscriber sub2;


int subed_enc = 2;

//ここでしきい値を先にきめて５箇所で止められうように設定する
long sikii1=40000;
long sikii2=60000;
long sikii3=70000;
long sikii4=80000;
int pw=-200;
long e=0;
long mode=0;
std_msgs::Int32MultiArray pub_msg;

void chatterCallback1(const sensor_msgs::Joy::ConstPtr& msg1){
	//メッセージのオブジェクト	
	//hogeがパブリッシュするhogeの中に購読したデータそのまま入れる
	if(msg1->axes[5]==1){
		mode=1;
	}
	if(msg1->axes[5]==-1){
		mode=3;			
	}
	if(msg1->axes[4]==1){
		mode=4;
	}
	if(msg1->axes[4]==-1){
		mode=2;
	}
	if(msg1->buttons[4]==1 && msg1->buttons[5]==1){
		pub_msg.data[1]=1;
		pub.publish(pub_msg);
	}
	if(msg1->buttons[4]==0 || msg1->buttons[5]==0){
		pub_msg.data[1]=0;
		pub.publish(pub_msg);
	}
	if(msg1->buttons[10]==1){
		mode=10;
	}
	if(msg1->buttons[11]==1){
		mode=11;
	}
	
	if(msg1->buttons[12]==1){
		mode=12;
	}
	if(msg1->buttons[10]==1 && msg1->buttons[11]==1 ){
		mode=13;
	}
	
}

void chatterCallback2(const std_msgs::Int32::ConstPtr& msg2){
	subed_enc = msg2->data;
	ROS_FATAL("%d from pub_enc", subed_enc);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "joy_kakudo_hassya");
	ros::NodeHandle nh;
	pub_msg.data.resize(3);
	pub_msg.data[0]=0;
	pub_msg.data[1]=0;
	pub_msg.data[2]=0;
	pub = nh.advertise<std_msgs::Int32MultiArray>("topic", 1);
	//topicっていうトピックにパブリッシュするよ1はどれくらい貯めるかの数字
	sub1 = nh.subscribe("joy", 1, chatterCallback1);
	sub2 = nh.subscribe("pub_enc", 1, chatterCallback2);
	//joyっていうトピックをサブスクライブするよ、サブスクライブしたらcallbackする


	ROS_FATAL("%d from main", subed_enc);
	ros::Rate loop_rate(10);
	while(ros::ok()){
		ros::spinOnce();

		ROS_FATAL("%d from pub_enc main loop", mode);
		if(mode==1){
			if(subed_enc<=sikii1){
				pub_msg.data[0]=pw;
				pub.publish(pub_msg);
			}
			if(subed_enc>sikii1){	
				pub_msg.data[0]=0;
				pub.publish(pub_msg);
			}	
		}
		if(mode==2){
			if(subed_enc<=sikii2){
				pub_msg.data[0]=pw;
				pub.publish(pub_msg);
			}
			if(subed_enc>sikii2){
				pub_msg.data[0]=0;
				pub.publish(pub_msg);
			}	
		}
		if(mode==3){
			if(subed_enc<=sikii3){
				pub_msg.data[0]=pw;
				pub.publish(pub_msg);
			}
			if(subed_enc>sikii3){
				pub_msg.data[0]=0;
				pub.publish(pub_msg);
			}	
		}
		if(mode==4){
			if(subed_enc<=sikii4){
				pub_msg.data[0]=pw;
				pub.publish(pub_msg);
			}
			if(subed_enc>sikii4){
				pub_msg.data[0]=0;
				pub.publish(pub_msg);
			}	
		}
		if(mode==10){
			//	上がるモード、一番上で止まる
			if(subed_enc>=50){
				pub_msg.data[0]=-pw;
				pub.publish(pub_msg);
			}
			if(subed_enc<=50){
				pub_msg.data[0]=0;
				pub.publish(pub_msg);
			}	
		}
		if(mode==11){
		//	下がるモード
			pub_msg.data[0]=pw;
			pub.publish(pub_msg);
		}
	
		if(mode==12){
		//	ストップモード
			pub_msg.data[0]=0;
			pub.publish(pub_msg);
		}
			
		if(mode==13){
		//	ずっと止まらない上がるモード
			pub_msg.data[0]=-pw;
			pub.publish(pub_msg);
		}	
		loop_rate.sleep();
	}	
	return 0;	
}
