//topic通信で最初の方に0が入るバグあり
#include<math.h> //M_PI
#include"ros/ros.h"
#include"std_msgs/Float64.h"
#include "abu2021_msgs/cmd_vw.h"

//constant (main)
#define WAIT_LOOP 400 //初期センサー値無視回数(最少でも25？)
#define SAMPLE_NUM 500 //センサ特性計算用サンプル数
#define MAIN_FREQUENCY 100 //メインループ周波数[Hz]
//constant (kalman filter)
#define TARGET_POSTURE_NOISE 0.1 //目標角速度の誤差分散
#define INIT_ANGLE_ERROR 4 //初期角度の誤差分散
#define INIT_ANGLE 0            //初期角度[rad]
#define INIT_ANGLAR_VELOCITY 0 //初期角速度[rad/sec]

//global variable
ros::Subscriber sub_raw; //subscriber from topic "gyro_raw"
ros::Subscriber sub_pst; //subscriber from topic "target_posture"
ros::Publisher  pub;     //publisher to topic "gyro_yaw"
double valRaw; //data from topic "gyro_raw"
double valPst; //data from topic "target_posture"

//function prototype
void get_gyro_raw(const std_msgs::Float64::ConstPtr& valRaw);
void get_target_posture(const abu2021_msgs::cmd_vw::ConstPtr& val);
void kalman_init(double arr[], double *mean, double *vari);
void kalman_loop(double mean, double vari);
void debugPrint(double val);

//main function
int main(int argc,char **argv){
	//variable declaration
	double snsArr[SAMPLE_NUM] = {}; //samples of sensor value
	double snsMean = 0; //mean of sensor value 
	double snsVari = 0; //variance of sensor value
	int count = 0; //for get samples

	//** node declaration **//
	ros::init(argc,argv,"gyro_test");
	ros::NodeHandle nh;

	sub_raw = nh.subscribe("gyro_raw",1,get_gyro_raw);
	sub_pst = nh.subscribe("cmd",1,get_target_posture);
	/* sub_pst = nh.subscribe("cmd_tr",10,get_target_posture); */
	pub     = nh.advertise <std_msgs::Float64>("gyro_yaw",1);
	
	ros::Rate loop_rate(MAIN_FREQUENCY);
	
	//** node body **//
	//standing by
	while(ros::ok() && count<WAIT_LOOP){
		ros::spinOnce();
		loop_rate.sleep();
		count++;
	}
	//standing by2
	while(ros::ok() && valRaw==0.0){
		ros::spinOnce();
		loop_rate.sleep();
	}

	count = 0;
	//get samples
	while(ros::ok() && count<SAMPLE_NUM){
		ros::spinOnce();
		snsArr[count] = valRaw;
		loop_rate.sleep();
		count++;
	}

	//calculation mean and variance
	kalman_init(snsArr, &snsMean, &snsVari);

	//debug (check sample array & mean & variance)
	//for(int i=0;i<60;i++) debugPrint(snsArr[i]);
	//debugPrint(snsMean);
	//debugPrint(snsVari);

	//run loop
	while(ros::ok()){
		ros::spinOnce();
		//debugPrint(5.55);
		kalman_loop(snsMean, snsVari);
		loop_rate.sleep();
	}

	return 0;
}

//kalman filter
void kalman_loop(double mean, double vari){
	std_msgs::Float64 pubData; //data for publish to topic "gyro_yaw"
	//kalman gain
	double kgain1;
	double kgain2;
	//covariance matrix
	double p11; double p12;
	double p21; double p22;
	//pre covariance marix
	static double pp11 = INIT_ANGLE_ERROR; static double pp12 = 0;
	static double pp21 = 0;                static double pp22 = vari;
	//state vectors
	double tht = 0; //angle
	double omg = 0; //angular velocity
	//pre state vectors
	static double ptht = INIT_ANGLE; //pre angle
	static double pomg = INIT_ANGLAR_VELOCITY; //pre angular velocity
	//delta t
	double dlt = 1.0/MAIN_FREQUENCY;

	//G = P'C^t(W+CP'C^t)^-1
	kgain1 = pp12 / (vari+pp22);
	kgain2 = pp22 / (vari+pp22);
	//P = (I-GC)P'
	p11 = pp11 - (pp21*kgain1); p12 = pp12 - (pp21*kgain1);
	p21 = pp21*(1.0-kgain2);    p22 = pp22*(1.0-kgain2);
	//x = x' + G(y-Cx')
	tht = ptht+(valRaw-pomg)*kgain1;
	omg = pomg+(valRaw-pomg)*kgain2;
	//P' = APA^t + BUB^t
	pp11 = p11 + dlt*p21 + dlt*(p12+dlt*p22) + dlt*dlt*vari;
	pp12 = -dlt*vari;
	pp21 = -dlt*vari;
	pp22 = vari + TARGET_POSTURE_NOISE; 
	//x' = Ax + Bu
	ptht = tht + dlt*(omg-mean);
	pomg = valPst + mean;
	
	//publish
	pubData.data = tht/M_PI*180.0;
	pub.publish(pubData);
}

//calculate mean and variance of sensor value
void kalman_init(double arr[], double *mean, double *vari){
	//calculate mean
	for(int i=0; i<SAMPLE_NUM; i++) *mean += arr[i];
	*mean /= SAMPLE_NUM;

	//calulate variance
	for(int i=0; i<SAMPLE_NUM; i++) *vari += (arr[i]-*mean)*(arr[i]-*mean); 
	*vari /= SAMPLE_NUM;
}

//get data from topic "gyro_raw"
void get_gyro_raw(const std_msgs::Float64::ConstPtr& val){
	valRaw = M_PI*(val->data)/180; //[deg/sec]->[rad/sec]
}

//get data from topic "target posture"
void get_target_posture(const abu2021_msgs::cmd_vw::ConstPtr& val){
	valPst = val->w;
}

//print for debug
void debugPrint(double val){
	std_msgs::Float64 pubData; 

	pubData.data = val;
	pub.publish(pubData);
}
