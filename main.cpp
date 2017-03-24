#include "mbed.h"
#include "rtos.h"
#include "sensors.h"
#include "Motor.h"
#include "serial_interface.h"
#include "controller.h"


#include <math.h>

#include "RawSerial.h"
RawSerial serial(SERIAL_TX, SERIAL_RX);

#define BAUDRATE 115200

//#define DEBUG_INTERFACE
//#define DEBUG_MOTOR

//Ticker to sample feedback at intervals
Ticker feedback_sampler;

//set PID constants
const  float sample_interval = 0.1f;
const  float k = -19.0f;
const  float kp = 0.07f;
const  float ki = 0.000001f;
const  float kd = 0.0002f;

Controller pid(kp, ki, kd, k, sample_interval);

Sensors sense;

Motor motor;
    
Serial_Interface serial_interface; 

Ticker quad_sampler;

//Function prototypes
void handle_feedback();
float calc_rotation_offset(float target, float current);
void serial_h_thread();
void update_speed();
float current_speed;

Thread serial_thread;
    
//Main
int main() {
    
    //Initialise the serial port
    
    //Serial_Interface serial_interface;
    serial.baud(BAUDRATE);
    serial.printf("Hello");
       
    int8_t intState = 0;
    int8_t intStateOld = 0;
    
    //Run the motor synchronisation
    motor.home(sense);
    motor.set_lead(2);
    
    //Set the speed sampler to 100ms intervals
    quad_sampler.attach(&update_speed, 0.1f);
    
    //Set the feedback to sample at intervals
    feedback_sampler.attach(&handle_feedback, sample_interval);
    
    //Start the serial command thread
    serial_thread.start(serial_h_thread);
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {

        intState = sense.rotor_state();
        if (intState != intStateOld) {
            intStateOld = intState;
            
            motor.output(intState); 
            
            //Uncomment for debug, if left it, serial handler crashes though so you can't send it commands
            //serial.printf("%f\n\r", current_speed);
        }
    }
}

//Calculate the speeds required to achieve target rotations
float calc_rotation_offset(float target, float current){
    float tmp;
    tmp = 4.0f*((target - current)/(serial_interface.saved_position()));
    if(tmp > 1){
       return 1; 
    }
    if(tmp < -1){
       return -1; 
    }
    return tmp;
}

//Ticker interrupt function to find new feedback values 
void handle_feedback(){
    float speed = 0;
    //R and V commands received
    if (serial_interface.position_change() && serial_interface.velocity_change()){
        //Find speed to achieve roations
        speed = serial_interface.velocity() * calc_rotation_offset(serial_interface.position(), sense.current_position());
        //Set speed
        motor.set_lead(pid.feedback(current_speed, speed));
    }
    //Only V command received
    else if (!serial_interface.position_change() && serial_interface.velocity_change()){
        speed = serial_interface.velocity();
        //Set speed
        motor.set_lead(pid.feedback(current_speed, speed));
    }
    //Only R command received
    else if (serial_interface.position_change() && !serial_interface.velocity_change()){
        //1st Case * 100.0f
        speed = 10000.0f * calc_rotation_offset(serial_interface.position(), sense.current_position());
        motor.set_lead(pid.feedback(current_speed, speed));
    }
    //Stop motor
    else {
        motor.set_lead(pid.feedback(sense.current_position(), 0.0f));
    }
    
}
//Get speed from quadrature encoder
void update_speed(){
   current_speed = sense.current_speed();
}
//Thread
void serial_h_thread(){
    while(1){
        serial_interface.handle(serial, sense.current_position());
    }
 }
 