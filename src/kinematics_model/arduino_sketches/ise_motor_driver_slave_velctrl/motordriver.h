/*
 * motordriver.h
 *
 * Created: 2018/01/21 19:37:23
 *  Author: magon
 */ 

#define SM_BRAKE_LOW_SIDE 0x00
#define SM_BRAKE_HIGH_SIDE 0x01
#define SM_COAST 0x02
#define LAP 0x03

#define TOP 255

void motor_init(uint8_t, uint8_t);
void motor_set_speed(int power);
void pwm_map(int duty);
