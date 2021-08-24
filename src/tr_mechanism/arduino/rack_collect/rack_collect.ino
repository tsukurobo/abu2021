#include <ros.h>
#include <abu2021_msgs/rack_msg.h>
#include <Servo.h>

#define pin_air_servo1 10
#define pin_air_servo2 11
#define pin_air_solenoid 6
#define pin_const_r_servo1 12
#define pin_const_r_servo2 13
#define pin_const_r_solenoid 2
#define pin_const_l_servo1 14
#define pin_const_l_servo2 15
#define pin_const_l_solenoid 8

int c_r_1;
int c_r_2;
int c_r_s;
int c_l_1;
int c_l_2;
int c_l_s;
int a_1;
int a_2;
int a_s;

class RackCollection{
  private:
  Servo ser1, ser2;
  int sol_pin;

  public:
  RackCollection(){};
  void init(int, int, int);
  void moveServo(int, int);
  void moveSol(int);
};

inline void RackCollection::init(int servo1_pin, int servo2_pin, int sol){
  ser1.attach(servo1_pin);
  ser2.attach(servo2_pin);

  sol_pin = sol;


  pinMode(sol_pin, OUTPUT);
  digitalWrite(sol_pin, LOW);
}

inline void RackCollection::moveServo(int deg1, int deg2){
  ser1.write(deg1);
  ser2.write(deg2);
}


inline void RackCollection::moveSol(int state){
  digitalWrite(sol_pin, state);
}

ros::NodeHandle nh;

 RackCollection constRackCol_R,constRackCol_L, airRackCol;

void get_order(const abu2021_msgs::rack_msg& sub_msg) {
 c_r_1=sub_msg.const_r_1;
 c_r_2=sub_msg.const_r_2;
 c_r_s=sub_msg.const_l_hand;
 c_l_1=sub_msg.const_l_1;
 c_l_2=sub_msg.const_l_1;
 c_l_s=sub_msg.const_l_hand;
 a_1=sub_msg.air_1;
 a_2=sub_msg.air_2;
 a_s=sub_msg.air_hand;
}

ros::Subscriber<abu2021_msgs::rack_msg> sub_order("rack_tpc", get_order);

void setup() {
  // put your setup code here, to run once:
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    nh.subscribe(sub_order);

    constRackCol_R.init(pin_const_r_servo1, pin_const_r_servo2, 
                      pin_const_r_solenoid);
    constRackCol_L.init(pin_const_l_servo1, pin_const_l_servo2, 
                      pin_const_l_solenoid);                    
    airRackCol.init(pin_air_servo1, pin_air_servo2, 
                    pin_air_solenoid);
    constRackCol_R.moveServo(c_r_1,c_r_2);
    constRackCol_R.moveSol(c_r_s);
    constRackCol_L.moveServo(c_l_1,c_l_2);
    constRackCol_L.moveSol(c_l_s);
    airRackCol.moveServo(a_1, a_2);
    airRackCol.moveSol(a_s);
    
}

void loop() {
  nh.spinOnce();
  // put your main code here, to run repeatedly:
     constRackCol_R.moveServo(c_r_1,c_r_2);
    constRackCol_R.moveSol(c_r_s);
    constRackCol_L.moveServo(c_l_1,c_l_2);
    constRackCol_L.moveSol(c_l_s);
    airRackCol.moveServo(a_1, a_2);
    airRackCol.moveSol(a_s);
    delay(10);

}
