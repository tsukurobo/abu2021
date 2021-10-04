#include <ros.h>
#include <abu2021_msgs/rack_msg.h>
#include <Servo.h>

#define pin_const_servo1 12
#define pin_const_servo2 13
#define pin_const_solenoid1 17
#define pin_const_solenoid2 19
#define pin_air_r_servo1 10
#define pin_air_r_servo2 11
#define pin_air_r_solenoid1 2
#define pin_air_r_solenoid2 14
#define pin_air_l_servo1 44
#define pin_air_l_servo2 46
#define pin_air_l_solenoid1 8
#define pin_air_l_solenoid2 6

#define R_HL 1
#define R_LH 2
#define FREE 0

int a_r_1 = 90;
int a_r_2 = 90;
int a_r_s = FREE;
int a_l_1 = 90;
int a_l_2 = 90;
int a_l_s = FREE;
int c_1 = 90;
int c_2 = 90;
int c_s = FREE;
int prevS_r_1;
int prevS_r_2;
int prevS_l_1;
int prevS_l_2;
int prevS_c_1;
int prevS_c_2;
class RackCollection{
  private:
  Servo ser1, ser2;
  int sol_pin_open, sol_pin_close;
  void hand_open();
  void hand_close();
  void hand_free();

  public:
  RackCollection(){};
  void init(int, int, int, int);
  void moveServo(int, int);
  void moveAirCylinder(int);
};

inline void RackCollection::init(int servo1_pin, int servo2_pin, int sol_pin_op, int sol_pin_cl){
  ser1.attach(servo1_pin);
  ser2.attach(servo2_pin);

  sol_pin_open = sol_pin_op;
  sol_pin_close = sol_pin_cl;

  pinMode(sol_pin_open, OUTPUT);
  digitalWrite(sol_pin_open, LOW);
  pinMode(sol_pin_close, OUTPUT);
  digitalWrite(sol_pin_close, LOW);
}

inline void RackCollection::moveServo(int deg1, int deg2){
  ser1.write(deg1);
  ser2.write(deg2);
}


void RackCollection::hand_free(void){
  digitalWrite(sol_pin_open, LOW);
  digitalWrite(sol_pin_close, LOW);
}

void RackCollection::hand_open(void){
  digitalWrite(sol_pin_open, HIGH);
  digitalWrite(sol_pin_close, LOW);
}

void RackCollection::hand_close(void){
  digitalWrite(sol_pin_open, LOW);
  digitalWrite(sol_pin_close, HIGH);
}

inline void RackCollection::moveAirCylinder(int cmd){
  switch(cmd){
    case R_HL:
    hand_open();
    break;

    case R_LH:
    hand_close();
    break;

    case FREE:
    hand_free();
    break;
  }
}

ros::NodeHandle nh;

RackCollection airRackCol_R, airRackCol_L, constRackCol;

void get_order(const abu2021_msgs::rack_msg& sub_msg) {
 //独自メッセージ型も編集するべき
 a_r_1=sub_msg.air_r_1;
 a_r_2=sub_msg.air_r_2;
 a_r_s=sub_msg.air_r_hand;
 a_l_1=sub_msg.air_l_1;
 a_l_2=sub_msg.air_l_2;
 a_l_s=sub_msg.air_l_hand;
 c_1=sub_msg.const_1;
 c_2=sub_msg.const_2;
 c_s=sub_msg.const_hand;
}

ros::Subscriber<abu2021_msgs::rack_msg> sub_order("rack_tpc", get_order);

void setup() {
  // put your setup code here, to run once:
    nh.getHardware()->setBaud(250000);
    nh.initNode();

    nh.subscribe(sub_order);

    airRackCol_R.init(pin_air_r_servo1, pin_air_r_servo2, 
                      pin_air_r_solenoid1, pin_air_r_solenoid2);
    airRackCol_L.init(pin_air_l_servo1, pin_air_l_servo2, 
                      pin_air_l_solenoid1, pin_air_l_solenoid2);                    
    constRackCol.init(pin_const_servo1, pin_const_servo2, 
                      pin_const_solenoid1, pin_const_solenoid2);
    airRackCol_R.moveServo(a_r_1,a_r_2);
    airRackCol_R.moveAirCylinder(a_r_s);
    airRackCol_L.moveServo(a_l_1,a_l_2);
    airRackCol_L.moveAirCylinder(a_l_s);
    constRackCol.moveServo(c_1, c_2);
    constRackCol.moveAirCylinder(c_s);
    prevS_r_1=a_r_1;
    prevS_r_2=a_r_2;
    prevS_l_1=a_l_1;
    prevS_l_2=a_l_2;
    prevS_c_1=c_1;
    prevS_c_2=c_2;
    
}

void loop() {
  nh.spinOnce();
  // put your main code here, to run repeatedly:
  if(a_r_1!=prevS_r_1||a_r_2!=prevS_r_2){
     airRackCol_R.moveServo(a_r_1,a_r_2);
     prevS_r_1=a_r_1;
     prevS_r_2=a_r_2;
  }
  airRackCol_R.moveAirCylinder(a_r_s);
  if(a_l_1!=prevS_l_1||a_l_2!=prevS_l_2){
     airRackCol_L.moveServo(a_l_1,a_l_2);
     prevS_l_1=a_l_1;
     prevS_l_2=a_l_2;
  }
  airRackCol_L.moveAirCylinder(a_l_s);
  if(c_1!=prevS_c_1||c_2!=prevS_c_2){
    constRackCol.moveServo(c_1, c_2);
    prevS_c_1=c_1;
    prevS_c_2=c_2;
  }
  constRackCol.moveAirCylinder(c_s);
  delay(10);

}
