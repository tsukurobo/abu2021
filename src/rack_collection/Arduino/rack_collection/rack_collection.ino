#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <Servo.h>
Servo servo1;
Servo servo2;

#define pin_servo1 2
#define pin_servo2 8
#define pin_solenoid1 4 //HIGHで開く
#define pin_solenoid2 6 //HIHGで閉まる

int deg_ini1 = 140;
int deg_ini2 = 130;

int deg_servo1 = deg_ini1;
int deg_servo2 = deg_ini2;
int solenoid = 0;

ros::NodeHandle nh;

std_msgs::Int32MultiArray ard_order;

void get_order(const std_msgs::Int32MultiArray& ard_order) {
    deg_servo1 = ard_order.data[0];
    deg_servo2 = ard_order.data[1];
    solenoid = ard_order.data[2];
}

ros::Subscriber<std_msgs::Int32MultiArray> sub_order("rack_ard_order", get_order);


void setup() {
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    nh.subscribe(sub_order);

    ard_order.data = (long int*)malloc(sizeof(long int) *3);
    ard_order.data_length = 3;
    
    servo1.attach(pin_servo1);
    servo2.attach(pin_servo2);
    pinMode(pin_solenoid1,OUTPUT);
    pinMode(pin_solenoid2,OUTPUT);
  
    servo1.write(deg_ini1);
    servo2.write(deg_ini2);
    digitalWrite(pin_solenoid1,LOW);
    digitalWrite(pin_solenoid2,LOW);
}

void loop() {
    nh.spinOnce();

    //サーボ１
    servo1.write(deg_servo1);

    //サーボ２
    servo2.write(deg_servo2);

    //ソレノイド(solenoid=0:緩める, solenoid=1:開ける, solenoid=2:閉める)
    if (solenoid == 0) {
        
        digitalWrite(pin_solenoid1, LOW); // 緩める
        digitalWrite(pin_solenoid2, LOW);
        
    } else if (solenoid == 1) {
        
        digitalWrite(pin_solenoid1, HIGH); //　開ける
        digitalWrite(pin_solenoid2, LOW);
        
    } else if (solenoid == 2) {
        
        digitalWrite(pin_solenoid1, LOW); // 閉める
        digitalWrite(pin_solenoid2, HIGH);
        
    }

    delay(1);
}
