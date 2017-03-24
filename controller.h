#ifndef CONTROLLER
#define CONTROLLER

#include "mbed.h"


class Controller{
    private:
    float input;
    float reference;
    float error;
    float pre_error;
    float kp;
    float ki;
    float kd;
    float k;
    float time_constant;
    float proportional;
    float integral;
    float derivative;
    float sum;
    float thresh1;
    float thresh2;
    float thresh3;
    float thresh4;
    float thresh5;
    
    
    public:
    Controller(float kp_val, float ki_val, float kd_val, float k_val, float time_constant_val);
    
    int8_t feedback(float input_val, float reference_val);
    
    float total();
    
    
    
};



#endif