#include "controller.h"

//Controller constructor
Controller::Controller(float kp_val, float ki_val, float kd_val, float k_val, float time_constant_val){
    //Intialise the control constants as specified
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
    k = k_val;
    //Set time constant between samples
    time_constant = time_constant_val;
    //set previous error to 0
    pre_error = 0;
    integral = 0;
    
    //Define thresholds to map the controller output to potential lead/lags
    thresh1 = 2.0f;
    thresh2 = 1.0f;
    thresh3 = 0.0f;
    thresh4 = -1.0f;
    thresh5 = -2.0f;
    
}

//Update the controller to find the correct feedback lead/lag
int8_t Controller::feedback(float input_val, float reference_val){
    //Pass in parameters
    input = input_val;
    reference = reference_val;
    
    error = input - reference;
    
    proportional = kp * error;
    derivative = kd * ((error-pre_error)/time_constant);
    //integral = ki * ((error-pre_error)*time_constant);
    
    integral += ki * (error*time_constant/2);
    sum = k*(proportional + derivative + integral);
    
    //error saves into previous erro
    pre_error = error;
    
    //scale the controller output to the lead lag values
    if(sum > thresh1){
        return 2;
    }
    else if(sum > thresh2){
        return 1;
    }
    else if(sum > thresh3){
        return 0;
    }
    else if(sum > thresh4){
        return 0;
    }
    else if(sum > thresh5){
        return -1;
    }
    else if(sum <= thresh5){
        return -2;
    }
    return 0;
}

//public function for debugging
float Controller::total(){
    return sum;   
}
    



