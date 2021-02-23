#include <Arduino.h>
#include "pid_calculator.h"

PIDCalculator::PIDCalculator(PIDSettings s)
: kp(s.kp), ki(s.ki), kd(s.kd), ts(s.ts), tdel(s.tdel), T(s.T), K(s.K), 
vmax(s.vmax), vmin(s.vmin), u(0.0), pre_e(0), err_sum(0),D_pre(0), mode(s.mode) {}

float PIDCalculator::calcValue(float curr_e){
  switch (mode){
    case NORMAL: //位置型PID
      err_sum += curr_e*ts;
      P = kp*curr_e;
      I = ki*err_sum;
      D = (tdel/(ts+tdel))*u + (kd/(ts+tdel))*(curr_e - pre_e); //不完全微分
      u = P + I + D;
      
      //アンチワインドアップ
      if(u > vmax){
        u = vmax;
        err_sum -= curr_e*ts;
      }else if(u < vmin){
        u = vmin;
        err_sum -= curr_e*ts;
      }
      
      pre_e = curr_e;
      
      return u;

    case BUMPLESS: //速度型PID
      P = kp*(curr_e - pre_e);
      I = ki*ts*curr_e;
      D = kd*(tdel/(ts+tdel))*u + (kd/(ts+tdel))*(curr_e - pre_e);
      u += P + I + (D-D_pre);
      if(u > vmax) u = vmax;
      if(u < vmin) u = vmin;
      
      pre_e = curr_e;
      D_pre = D;
      
      return u;
  }
  
}
