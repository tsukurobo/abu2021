/*includes*/
#include <ros.h>
#include <abu2021_msgs/tr_order.h>
#include <Servo.h>

/*defines*/
#define pin_const_servo1 12
#define pin_const_servo2 13
#define pin_const_solenoid1 2 //HIGHで開く
#define pin_const_solenoid2 14 //HIGHで閉まる
#define pin_air_servo1 10
#define pin_air_servo2 11
#define pin_air_solenoid1 6 //HIGHで開く
#define pin_air_solenoid2 8 //HIHGで閉まる

#define RACK_COLLECT 2

#define CONST 0
#define AIR 1

#define CONST_DOWN 0
#define CONST_CLOSE 1
#define CONST_UP 2
#define CONST_LOAD 3
#define CONST_OPEN 4
#define CONST_DOWN2 5
#define CONST_INITIAL_POSITION 6
#define AIR_DOWN 7
#define AIR_CLOSE 8
#define AIR_UP 9
#define AIR_LOAD 10
#define AIR_OPEN 11
#define AIR_DOWN2 12
#define AIR_INITIAL_POSITION 13

/*class definition*/
class RackCollection{
  private:
  Servo ser1, ser2;
  int sol_pin_open, sol_pin_close;

  public:
  RackCollection(){};
  void init(int, int, int, int);
  void moveServo(int, int);
  void hand_open();
  void hand_close();
  void hand_free();
};

/*RackCollection::RackCollection(int servo1_pin, int servo2_pin, int sol_pin_op, int sol_pin_cl):
sol_pin_open(sol_pin_op), sol_pin_close(sol_pin_cl)
{
  ser1.attach(servo1_pin);
  ser2.attach(servo2_pin);

  pinMode(sol_pin_open, OUTPUT);
  digitalWrite(sol_pin_open, LOW);
  pinMode(sol_pin_close, OUTPUT);
  digitalWrite(sol_pin_close, LOW);
}*/

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

inline void RackCollection::hand_open(void){
  digitalWrite(sol_pin_open, HIGH);
  digitalWrite(sol_pin_close, LOW);
}

inline void RackCollection::hand_close(void){
  digitalWrite(sol_pin_open, LOW);
  digitalWrite(sol_pin_close, HIGH);
}

inline void RackCollection::hand_free(void){
  digitalWrite(sol_pin_open, LOW);
  digitalWrite(sol_pin_close, LOW);
}



/*main body*/
/*int const_deg_ini1 = 140;
int const_deg_ini2 = 130;
int air_deg_ini1 = 140;
int air_deg_ini2 = 130;*/
int deg_ini1[2]={}, deg_ini2[2]={}, deg_pick1[2]={},
    deg_pick2[2]={}, deg_load1[2]={}, deg_load2[2]={},
    deg_up1[2]={}, deg_up2[2]={}, deg_pick2_1[2], deg_pick2_2[2],
    deg_up2_1[2]={}, deg_up2_2[2]={},pick_up = 0;
int time_delay = 0;
int orderId = 0;
bool isMsgReceived = false;

//int const_deg_servo1 = deg_ini1;
//int const_deg_servo2 = deg_ini2;
//int solenoid_const = 0, solenoid_air = 0;

ros::NodeHandle nh;
//abu2021_msgs::tr_order ord;

/*RackCollection constRackCol(pin_const_servo1, pin_const_servo2, 
                            pin_const_solenoid1, pin_const_solenoid2), 
               airRackCol(pin_air_servo1, pin_air_servo2, 
                          pin_air_solenoid1, pin_air_solenoid2);*/
  RackCollection constRackCol, airRackCol;
//Servo ser1, ser2;

void get_order(const abu2021_msgs::tr_order& ard_order) {
  if(ard_order.nodeId ==RACK_COLLECT){
    isMsgReceived = true;
    orderId = ard_order.orderId;
  } 
}


ros::Subscriber<abu2021_msgs::tr_order> sub_order("tr_order", get_order);

void setup() {
    //ser1.attach(pin_const_servo1);
    //ser2.attach(pin_const_servo2);
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    nh.subscribe(sub_order);

    /*ard_order.data = (long int*)malloc(sizeof(long int) *3);
    ard_order.data_length = 3;*/
    
    
    /*pinMode(pin_solenoid1,OUTPUT);
    pinMode(pin_solenoid2,OUTPUT);
  
    servo1.write(deg_ini1);
    servo2.write(deg_ini2);
    digitalWrite(pin_solenoid1,LOW);
    digitalWrite(pin_solenoid2,LOW);*/
    while(!nh.getParam("/rack_col/const/deg_ini1",&deg_ini1[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_ini2",&deg_ini2[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_pick1",&deg_pick1[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_pick2",&deg_pick2[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_load1",&deg_load1[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_load2",&deg_load2[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_up1",&deg_up1[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_up2",&deg_up2[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_up2_1",&deg_up2_1[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_up2_2",&deg_up2_2[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_pick2_1",&deg_pick2_1[CONST], 1));
    while(!nh.getParam("/rack_col/const/deg_pick2_2",&deg_pick2_2[CONST], 1));

    while(!nh.getParam("/rack_col/air/deg_ini1",&deg_ini1[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_ini2",&deg_ini2[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_pick1",&deg_pick1[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_pick2",&deg_pick2[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_load1",&deg_load1[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_load2",&deg_load2[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_up1",&deg_up1[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_up2",&deg_up2[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_up2_1",&deg_up2_1[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_up2_2",&deg_up2_2[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_pick2_1",&deg_pick2_1[AIR], 1));
    while(!nh.getParam("/rack_col/air/deg_pick2_2",&deg_pick2_2[AIR], 1));

    while(!nh.getParam("/rack_col/time_delay",&time_delay, 1));
    while(!nh.getParam("/rack_col/pick_up",&pick_up, 1));

    constRackCol.init(pin_const_servo1, pin_const_servo2, 
                      pin_const_solenoid1, pin_const_solenoid2);
    airRackCol.init(pin_air_servo1, pin_air_servo2, 
                    pin_air_solenoid1, pin_air_solenoid2);
    constRackCol.moveServo(deg_ini1[CONST], deg_ini2[CONST]);
    constRackCol.hand_free();
    airRackCol.moveServo(deg_ini1[AIR], deg_ini2[AIR]);
    airRackCol.hand_free();
}

void loop() {
    static RackCollection *col;
    static int mecha_sel = 0;

    nh.spinOnce();
    //constRackCol.moveServo(0,10);
    //airRackCol.moveServo(30,180);
    /*//サーボ１
    ser1.write(100);

    //サーボ２
    ser2.write(100);

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
        
    }*/
  if(isMsgReceived == true){
    isMsgReceived = false;
  
    if(orderId <= CONST_INITIAL_POSITION){
      col = &constRackCol;
      mecha_sel = CONST;
      
    }else{
      col = &airRackCol;
      mecha_sel = AIR;
    }
    
    switch(orderId){
      case CONST_DOWN:
      case AIR_DOWN:
        col->moveServo(deg_pick1[mecha_sel], deg_pick2[mecha_sel]);
        break;

      case CONST_DOWN2:
      case AIR_DOWN2:
        col->moveServo(deg_pick2_1[mecha_sel], deg_pick2_2[mecha_sel]);
        break;

      case CONST_CLOSE:
      case AIR_CLOSE:
        col->hand_close();
        delay(time_delay);
        col->moveServo(deg_pick1[mecha_sel]+pick_up, deg_pick2[mecha_sel]);
        break;

      case CONST_UP:
      case AIR_UP:
        col->moveServo(deg_up1[mecha_sel], deg_up2[mecha_sel]);
        delay(time_delay);
        //nh.spinOnce();
        col->moveServo(deg_up2_1[mecha_sel], deg_up2_2[mecha_sel]);
        break;

      case CONST_LOAD:
      case AIR_LOAD:
        col->moveServo(deg_load1[mecha_sel], deg_load2[mecha_sel]);
        break;

      case CONST_OPEN:
      case AIR_OPEN:
        col->hand_open();
        break;

      case CONST_INITIAL_POSITION:
      case AIR_INITIAL_POSITION:
        col->moveServo(deg_ini1[mecha_sel], deg_ini2[mecha_sel]);
        break;

      default:
        break;

    }
  }
    //delay(100);
}
