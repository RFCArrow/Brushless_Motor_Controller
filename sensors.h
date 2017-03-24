#ifndef _SENSORS_
#define _SENSORS_

#include "mbed.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12
 
//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  





//DigitalIn CHA(CHApin);


    //CHA_Interrupt.rise(callback(this, &cha_rise));
    //CHB_Interrupt.rise(&chb_rise);
    //CHA_Interrupt.fall(&cha_fall);
    //CHB_Interrupt.fall(&chb_fall);

class Sensors{
    
    private:
    
    int8_t cha_state;
    int8_t chb_state;
    int32_t quadrature_position;
    uint32_t previous_time;
    uint32_t rotation_period;
    InterruptIn CHA_Interrupt;
    InterruptIn CHB_Interrupt;
    void cha_rise();
    void chb_rise();
    void cha_fall();
    void chb_fall();
    void i1_rise();
    void i1_fall();
    void i2_rise();
    void i2_fall();
    void i3_rise();
    void i3_fall();
    int oldState;
    //InterruptIn I1_Interrupt;
    //InterruptIn I2_Interrupt;
    //InterruptIn I3_Interrupt;
    
    int32_t rotations;
    int8_t state_transitions;
    int8_t direction;
    int i1_previous;
    int i2_previous;
    int i3_previous;
    float sample_period;
    int32_t quad_counter;
    
    
    public:
    
    Sensors();
    int8_t rotor_state();
    float current_position();
    float current_velocity();
    int32_t current_rotations();
    int8_t current_transitions();
    int32_t current_quad();
    uint32_t time();
    int8_t callibrate();
    void calculate_velocity();
    float current_speed();
    
};



#endif
