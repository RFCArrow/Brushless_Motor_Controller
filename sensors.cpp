#include "sensors.h"
#include "mbed.h"

//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);

//#include "RawSerial.h"
//RawSerial serial(SERIAL_TX, SERIAL_RX);


int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07}; 

Sensors::Sensors() : CHA_Interrupt(PinName(CHA)), CHB_Interrupt(PinName(CHB))
{
    
    sample_period = 0.1f;
    oldState = rotor_state();
    previous_time = 0; //deprecated
    rotation_period = 0; //deprecated
    rotations = 0;
    //Quad position for no rotations
    quadrature_position = 0;
    //Quad position for velocity
    quad_counter = 0;
    direction = 0;
    
    CHA_Interrupt.rise(callback(this, &Sensors::cha_rise));
    CHB_Interrupt.rise(callback(this, &Sensors::chb_rise));
    CHA_Interrupt.fall(callback(this, &Sensors::cha_fall));
    CHB_Interrupt.fall(callback(this, &Sensors::chb_fall));
    
    us_ticker_init();
    
}

//Get rotor state
int8_t Sensors::rotor_state(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//On rising edge of quad encoder
void Sensors::cha_rise(){
    cha_state = 1;
    //Update quad position
    if(chb_state){
        quadrature_position--;
        quad_counter--;
    }
    else{
        quadrature_position++;
        quad_counter++;
    }
    //If state has changed, run "soft interrupts"
    if(I1 == 0 && i1_previous == 1){
       i1_rise(); 
    }
    else if(I1 == 1 && i1_previous == 0){
       i1_rise(); 
    }
    if(I2 == 0 && i2_previous == 1){
       i2_rise(); 
    }
    else if(I2 == 1 && i2_previous == 0){
       i2_rise(); 
    }
    if(I3 == 0 && i3_previous == 1){
       i3_rise(); 
    }
    else if(I3 == 1 && i3_previous == 0){
       i3_rise(); 
    }
    //Save previous state for "soft interrupts"
    i1_previous =I1;
    i2_previous =I2;
    i3_previous =I3;
    
    
}

void Sensors::chb_rise(){
    chb_state = 1;
    if(cha_state){
        quadrature_position++;
        quad_counter++;
    }
    else{
        quadrature_position--;
        quad_counter--;
    }
}

void Sensors::cha_fall(){
    cha_state = 0;
    if(chb_state){
        quadrature_position++;
        quad_counter++;
    }
    else{
        quadrature_position--;
        quad_counter--;
    }
}

void Sensors::chb_fall(){
    chb_state = 0; 
    if(cha_state){
        quadrature_position--;
        quad_counter--;
    }
    else{
        quadrature_position++;
        quad_counter++;
    }
}

void Sensors::i3_rise(){
    int intState = rotor_state();
    if( intState == 0 && oldState == 5){
        state_transitions++;
    }
    if( intState == 2 && oldState == 3){
        state_transitions--;
    }
    oldState = intState;
    if(state_transitions > 5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations++;
        direction = 0;
        state_transitions = 0;
        quadrature_position = 0;
        previous_time = current_time;
    }
    if(state_transitions < -5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations--;
        direction = 1;
        state_transitions = 0;
        quadrature_position = 467;
        previous_time = current_time;
    }
    oldState = intState;
}
void Sensors::i3_fall(){
    int intState = rotor_state();
    if( intState == 3 && oldState == 2){
        state_transitions++;
    }
    if( intState == 5 && oldState == 0){
        state_transitions--;
    }
    oldState = intState;
    if(state_transitions > 5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations++;
        direction = 0;
        state_transitions = 0;
        quadrature_position = 0;
        previous_time = current_time;
    }
    if(state_transitions < -5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations--;
        direction = 1;
        state_transitions = 0;
        quadrature_position = 467;
        previous_time = current_time;
    }
    oldState = intState;
}
void Sensors::i2_rise(){
    int intState = rotor_state();
    if( intState == 4 && oldState == 5){
        state_transitions--;
    }
    if( intState == 2 && oldState == 1){
        state_transitions++;
    }
    oldState = intState;
    if(state_transitions > 5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations++;
        direction = 0;
        state_transitions = 0;
        quadrature_position = 0;
        previous_time = current_time;
    }
    if(state_transitions < -5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations--;
        direction = 1;
        state_transitions = 0;
        quadrature_position = 467;
        previous_time = current_time;
    }
    oldState = intState;
}
void Sensors::i2_fall(){
    int intState = rotor_state();
    if( intState == 5 && oldState == 4){
        state_transitions++;
    }
    if( intState == 1 && oldState == 2){
        state_transitions--;
    }
    oldState = intState;
    if(state_transitions > 5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations++;
        direction = 0;
        state_transitions = 0;
        quadrature_position = 0;
        previous_time = current_time;
    }
    if(state_transitions < -5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations--;
        direction = 1;
        state_transitions = 0;
        quadrature_position = 467;
        previous_time = current_time;
    }
    oldState = intState;
}
void Sensors::i1_rise(){
    int intState = rotor_state();
    if( intState == 4 && oldState == 3){
        state_transitions++;
    }
    if( intState == 0 && oldState == 1){
        state_transitions--;
    }
    oldState = intState;
    if(state_transitions > 5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations++;
        direction = 0;
        state_transitions = 0;
        quadrature_position = 0;
        previous_time = current_time;
    }
    if(state_transitions < -5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations--;
        direction = 1;
        state_transitions = 0;
        quadrature_position = 467;
        previous_time = current_time;
    }
    oldState = intState;
}
void Sensors::i1_fall(){
    int intState = rotor_state();
    if( intState == 1 && oldState == 0){
        state_transitions++;
    }
    if( intState == 3 && oldState == 4){
        state_transitions--;
    }
    oldState = intState;
    if(state_transitions > 5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations++;
        direction = 0;
        state_transitions = 0;
        quadrature_position = 0;
        previous_time = current_time;
    }
    if(state_transitions < -5){
        uint32_t current_time = us_ticker_read();
        rotation_period = current_time - previous_time;
        rotations--;
        direction = 1;
        state_transitions = 0;
        quadrature_position = 467;
        previous_time = current_time;
    }
    oldState = intState;
}

void Sensors::calculate_velocity(){
    uint32_t current_time = us_ticker_read();
    rotation_period = current_time - previous_time;
    previous_time = current_time;
}


float Sensors::current_position(){
    float position = 0;
    position = (float)rotations*360.0f + (float)quadrature_position*(360.0f/467.0f);
    return position;
}

float Sensors::current_velocity(){
    float velocity = 1000000.0f/(float)rotation_period;
    if(direction){
        return -velocity;
    }
    else{
        return velocity;
    }
}

int32_t Sensors::current_rotations(){
    return rotations;
}
int8_t Sensors::current_transitions(){
    return state_transitions;
}
int32_t Sensors::current_quad(){
    return quadrature_position;
}
uint32_t Sensors::time(){
    return previous_time;
}

int8_t Sensors::callibrate(){
    rotations = 0;
    quadrature_position = 0;
    state_transitions = 0;
    rotation_period = 100000000000000;
    return rotor_state();
}

float Sensors::current_speed(){
    float speed = (float)quad_counter / (sample_period*468.0f);
    quad_counter = 0;
    return speed;
}
