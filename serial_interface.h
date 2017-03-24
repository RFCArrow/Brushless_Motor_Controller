#ifndef SERIAL_INTERFACE
#define SERIAL_INTERFACE

#include "mbed.h"

#include <RawSerial.h>

#define BUFFLENGTH 20

class Serial_Interface {
    
        int buf_index;
        int finished;
        void parse_input(RawSerial &serial);
        char buffer[BUFFLENGTH];
        float current_position;
        float target_position;
        float target_angular_velocity;
        int v_change;
        int r_change;
        float save_position;
        
    public:
   
        void handle(RawSerial &serial,float current_position);
        float position();
        float velocity();
        float saved_position();
        int position_change();
        int velocity_change();
        Serial_Interface();      
};
#endif