
 /*version 3_for_driving_system*/
#define F_CPU 20000000UL

#include <avr/io.h>
#include "I2CSlave.h"
#include "motordriver.h"
#include "pid_calculator.h"
#include <avr/interrupt.h>

#define I2C_ADDR 0x14
#define PWM_8KHZ 0x02
/*#define MSB 0x80
#define BIT2 0x40
#define BIT1 0x20   
#define READ_BIT0_TO_4 0x1F
#define READ_BIT0_TO_2 0x07
#define READ_BIT0_TO_6 0x7F*/

#define DT_PID 10 //ms
#define ENC_RESOL 1024.0
#define _PI 3.141592

volatile int count = 0;
volatile int goal_vel = 0; //rad/s: -128~128

//const unsigned long dt_pid = 10;
const float ks = 0.1, conv_to_rot = 2000.0*_PI/(ENC_RESOL*DT_PID);
PIDCalculator *pidcal;

//int ddt = 0;

void i2c_received_cb(uint8_t data) {//送られてきた2バイトのデータを結合
  //volatile static uint8_t sign_flag = 0, seq_flag = 0;
  //volatile static unsigned int power_raw=0;
  //static int power;
  
  /*if((data&MSB) == 0){//前半部分のデータが来た場合
    if((data&BIT2) == 0){ //速度指令の場合
      sign_flag = ((data&BIT1)==0) ? 0 : 1 ; //符号ビットを確認し、sign_flagの値を設定
      power_raw = (unsigned int)(data&READ_BIT0_TO_4)<<7; //速度指令(前半)を取り出し、7ビット左シフトさせてpower_rawに代入
      seq_flag = 1;
    }else{ //MD設定用のデータの場合
      motor_init(data&READ_BIT0_TO_2, (data>>3)&READ_BIT0_TO_2); //モーターの回転設定を行う
      seq_flag = 1;
      
    }
  }else{ //後半部分のデータが来た場合
    if(seq_flag == 1){
      power_raw |= (unsigned int)(data&READ_BIT0_TO_6); //速度指令(後半)を取り出し、power_rawとのORを取る
      power = (sign_flag==0) ? (int)power_raw : -(int)power_raw; //sign_flagに応じて-1倍する
      motor_set_speed(power); //モーターを回転させる
      //Serial.println(power);
      seq_flag = 0;
    }
  }*/
  goal_vel = (int)((int8_t)data);
}

void i2c_requested_cb() {
	//long a = -1000;
  /*volatile static unsigned long x;
  volatile static uint8_t part=0;
  if(part==0) x=count;
  uint8_t y = (x<<part*8)>>24;
  I2C_transmitByte(y); //32ビットを8ビットずつ区切って上位ビットから送る
  part++;
  if(part>=4){
    part=0;
    //count -= 10;
  }*/
  I2C_transmitByte((uint8_t)count);
  //digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));
}

ISR(PCINT1_vect, ISR_NOBLOCK){//encorder
	if((PINC & (1 << PINC0)) ^ ((PINC & (1 << PINC1))>> 1))++count;
	else --count;	
}


void setup (){
	
	//ピン変化割り込み許可（PCINT8~15）
	PCICR |= (1<<PCIE1);
	////ピン変化割り込み許可（PCINT8）
	PCMSK1 |= (1<<PCINT8);
  
	motor_init(SM_BRAKE_LOW_SIDE, PWM_8KHZ); //デフォルトの設定
	motor_set_speed(0);
	
	DDRC = 0x00;
	PORTC = 0x00;
	
	// LED of addresses 0x10~ 0x1F 0~F -> 0~15
	PORTD |= (0b00001111 & I2C_ADDR);

  //enable pullups of SDA and SCL pins
  digitalWrite(SDA, HIGH);
  digitalWrite(SCL, HIGH);

	//sei();
  //intial settings
	I2C_init(I2C_ADDR); //I2Cの割り込みオン(使うべきでないぞ)
  //I2C_init_sync(I2C_ADDR);
  I2C_setCallbacks(i2c_received_cb, i2c_requested_cb);
  //Serial.begin(57600);

  PIDSettings pidset;
  pidset.kp = 3.8;
  pidset.ki = 1.0;
  pidset.kd = 0.05;
  pidset.ts = (float)DT_PID*0.001;
  pidset.tdel = 0.002;
  pidset.vmin = -TOP;
  pidset.vmax = TOP;
  pidset.mode = NORMAL;

  pidcal = new PIDCalculator(pidset);

  //pinMode(LED_BUILTIN, OUTPUT);
}


void loop() {
  //static unsigned long time_pre = millis(), time_now = 0;
  static float enc_s = 0, rot_spd = 0;
  
  //I2C_main();
  
  //time_now = millis();
  //if(time_now - time_pre >= DT_PID){
    //int time_pre_micro = micros();
    
    int count_buf = count;
    count = 0;

    enc_s = ks*count_buf + enc_s*(1.0-ks);
    rot_spd = enc_s*conv_to_rot;

    int motor_pwm = (int)pidcal->calcValue((float)goal_vel - rot_spd);
    if(goal_vel == 0) motor_pwm = 0;
    motor_set_speed(motor_pwm);
    //motor_set_speed(200);

    for(int i=0; i<DT_PID-1; i++) delayMicroseconds(1000);
    delayMicroseconds(800);

    //time_pre = time_now;

    //ddt = micros() - time_pre_micro;
  //}
  //delayMicroseconds(1000);
}
