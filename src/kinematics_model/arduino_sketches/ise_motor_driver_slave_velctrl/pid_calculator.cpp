#include <Arduino.h>
#include "pid_calculator.h"

PIDCalculator::PIDCalculator(PIDSettings s)
: kp(s.kp), ki(s.ki), kd(s.kd), ts(s.ts), tdel(s.tdel), a1(s.a1), a2(s.a2), 
vmax(s.vmax), vmin(s.vmin), u(0.0), pre_e(0), err_sum(0),D_pre(0), mode(s.mode),
C0(tdel/(ts+tdel)), C1(kd/(ts+tdel))
{}

float PIDCalculator::calcValue(float curr_e, float goal_v){
  switch (mode){
    case NORMAL: //位置型PID
      err_sum += curr_e*ts;
      P = kp*curr_e;
      I = ki*err_sum;
      D = C0*D + C1*(curr_e - pre_e); //不完全微分
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
      D = kd*C0*u + C1*(curr_e - pre_e);
      u += P + I + (D-D_pre);
      if(u > vmax) u = vmax;
      if(u < vmin) u = vmin;
      
      pre_e = curr_e;
      D_pre = D;
      
      return u;
      
      //2自由度PID
      case TWO_DOF:
      //FF部分
      u = (goal_v == 0) ? 0 : ((goal_v>0) ? (a1*goal_v + a2) : (a1*goal_v - a2));
      
      //FB部分
      err_sum += curr_e*ts;
      P = kp*curr_e;
      I = ki*err_sum;
      D = C0*D + C1*(curr_e - pre_e); //不完全微分
      u += P + I + D;
      
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
  }
  
}

float PIDCalculator::getP(){
  return P;
}

float PIDCalculator::getI(){
  return I;
}

float PIDCalculator::getD(){
  return D;
}
