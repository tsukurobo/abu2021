#ifndef Encoder_h
#define Encoder_h
#include "Arduino.h"

class Encoder{
  public:
    //pin number
    int pin_a;
    int pin_b;
    //interrupt number
    int interrupt_num_a;
    int interrupt_num_b;
    //encoder
    volatile byte stt_a = 0;
    volatile byte stt_b = 0;
    volatile long step_cnt = 0;
    
    Encoder(int p_a, int p_b);
    void enc_read_a();
    void enc_read_b();
};

#endif
