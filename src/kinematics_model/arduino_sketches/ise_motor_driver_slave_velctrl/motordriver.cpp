/*
 * ContorolMotor.cpp
 *
 * Created: 2018/01/21 19:11:22
 *  Author: magon
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "motordriver.h"


//const int TOP=255;
uint8_t drivemode = 0;

/*enum{
	TMR0Full = TOP,//255,
	TMR0INI = 0, //122,
};*/

void motor_init(uint8_t mode, uint8_t freq) //左から順にHブリッジの駆動方法、PWM周波数
{
	//setup
  DDRD |= 0b11110000; //1で出力 0で入力
  TCCR0B = freq; //TOP=255
  drivemode = mode;
  
  if(drivemode == SM_BRAKE_LOW_SIDE){ 
	  TCCR0A = 0b00100011;  //FastPWM, OCR0AのPWMは使用しない
    PORTD |= 0b01110000; //SR active
  }else if(drivemode == SM_BRAKE_HIGH_SIDE){
    TCCR0A = 0b10000011;  //FastPWM, OCR0BのPWMは使用しない
    PORTD |= 0b01110000; //SR active
  }else if(drivemode == SM_COAST){
    TCCR0A = 0b10100011;  //FastPWM, OCR0B/A両方のPWMを使用する
    PORTD |= 0b01100000; //SR Off
  }else if(drivemode == LAP){
    //iseモードラではA3921のPHASEにPWMを送ることはできない
    //よって、ここでは逆回転か正回転のみできるモードとなる
    PORTD |= 0b11110000;
  }
}

/*int pwm_map (int duty){
	return TMR0INI + (int)(TOP/100*duty); //duty=0~100
}*/
	
	
	
void motor_set_speed (int power) {
	  if(drivemode == SM_BRAKE_LOW_SIDE){
      //OCR0A = pwm_map(abs(power));
      OCR0B = abs(power);
		  if (power < 0){
			  //PORTD = 0b01110000;	  //PHASE=high
			  PORTD &= 0b01111111;
		  }
		  else if(power > 0){
			  //PORTD = 0b11100000;   //PHASE=low
			  PORTD |= 0b10000000;
		  }
	  }
	  else if(drivemode == SM_BRAKE_HIGH_SIDE){
      OCR0A = abs(power);
      if(power < 0){
        PORTD &= 0b01111111;
      }
      else if(power > 0){
        PORTD |= 0b10000000;
      }
	  }
    else if(drivemode == SM_COAST){
      OCR0A = abs(power);
      OCR0B = abs(power);
      if(power < 0){
        PORTD &= 0b01111111;
      }
      else if(power > 0){
        PORTD |= 0b10000000;
      }
    }

    else if(drivemode == LAP){
      if(power > TOP/2){
        PORTD |= 0b10000000; //逆転
      }else{
        PORTD &= 0b01111111; //正転
      }
    }
				
}
