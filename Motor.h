#ifndef _MOTOR_
#define _MOTOR_

#include "mbed.h"
#include "sensors.h"

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/


//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};





class Motor{
    int8_t orState;    //Rotot offset at motor state 0
    
    //Phase lead to make motor spin
    int8_t lead;
    
    
    public:
    void home(Sensors &sense);
    void set_lead(int8_t lead_val);
    void output(int8_t driveState);
    Motor();
    
    
    
};



#endif