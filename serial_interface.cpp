#include "serial_interface.h"
#include <math.h>


Serial_Interface::Serial_Interface(){
    buf_index = 0;
    finished = 0;
    save_position = 1;
}


void Serial_Interface::handle(RawSerial &serial, float position){
    current_position = position;
    
    //Get all the contents of the serial bus
    while(serial.readable() && buf_index<BUFFLENGTH) {
        buffer[buf_index] = serial.getc();
        //At the new line, add null terminator
        if(buffer[buf_index] == '\n') {
             buffer[buf_index] = '\0';
             finished = 1;
             buf_index = 0;
             break;
        }
            buf_index++;
    }
    //If new line, end of transmission
    if(finished) {
        //Parse the commands
        parse_input( serial );
        finished = 0;
        save_position = target_position - current_position;
    }   
}

void Serial_Interface::parse_input(RawSerial &serial)
{
    v_change = 0;
    r_change = 0;
    float rotation = 0.0f;
    int negative = 0;
    float speed = 0.0f;
    int index = 0;
    #ifdef DEBUG_INTERFACE
        serial.puts("Processing string: ");
        serial.puts((char*)buffer);
        serial.puts("\n\r");
    #endif
    switch(buffer[index]) {
        case 'R': {
            index++;
            if(buffer[index] == '-') {
                negative = 1;
                index++;
            }
            if(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                rotation +=(float)(buffer[index]-48);
                index++;
                while(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                    rotation *= 10.0f;
                    rotation += (float)(buffer[index]-48);
                    index++;
                }
                if(buffer[index] == '\0') {
                    //rotations
                    if(negative) {
                        rotation *= -1;
                    }
                    #ifdef DEBUG_INTERFACE
                        serial.printf("Rotations: %f \n\r",rotation);
                    #endif
                    target_position = (rotation*360)+current_position;
                    r_change = 1;
                    return;
                }
                if(buffer[index] == '.') {
                    index++;
                    if(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                        float significance = 0.1f;
                        while(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                            rotation += (buffer[index]-48)*significance;
                            significance *= 0.1f;
                            index++;
                        }
                        if(buffer[index] == '\0') {
                            //return rotations
                            if(negative) {
                                rotation *= -1;
                            }
                            #ifdef DEBUG_INTERFACE
                                serial.printf("Rotations: %f \n\r",rotation);
                            #endif
                            target_position = (rotation*360)+current_position;
                            r_change = 1;
                            return;
                        }
                        if(buffer[index] == 'V') {
                            index++;
                            if(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                                speed += (buffer[index]-48);
                                index++;
                                while(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                                    speed *= 10;
                                    speed += (buffer[index]-48);
                                    index++;
                                }
                                if(buffer[index] == '\0') {
                                    //return rotation and speed
                                    if(negative) {
                                        rotation *= -1;
                                    }
                                    #ifdef DEBUG_INTERFACE
                                        serial.printf("Rotations: %f",rotation);
                                        serial.printf("\t Velocity: ");
                                        serial.printf("%f \n\r", speed);
                                    #endif
                                    target_angular_velocity = speed;
                                    target_position = (rotation*360)+current_position;
                                    r_change = 1;
                                    v_change = 1;
                                    return;
                                }
                                if(buffer[index] == '.') {
                                    index++;
                                    if(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                                        float significance = 0.1f;
                                        while(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                                            speed += (buffer[index]-48)*significance;
                                            significance *= 0.1f;
                                            index++;
                                        }
                                        if(buffer[index] == '\0') {
                                            #ifdef DEBUG_INTERFACE
                                                serial.printf("Rotations: %f",rotation);
                                                serial.printf("\t Velocity: ");
                                                serial.printf("%f \n\r", speed);
                                            #endif
                                            target_angular_velocity = speed;
                                            target_position = (rotation*360)+current_position;
                                            r_change = 1;
                                            v_change = 1;
                                            return;
                                        }
                                    }
                                } else {
                                    //invalid
                                    return;
                                }

                            } else {
                                //invalid
                                return;
                            }
                        }
                    } else {
                        //invalid
                        return;
                    }
                }
                if(buffer[index] == 'V') {
                            index++;
                            if(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                                speed += (buffer[index]-48);
                                index++;
                                while(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                                    speed *= 10;
                                    speed += (buffer[index]-48);
                                    index++;
                                }
                                if(buffer[index] == '\0') {
                                    //return rotation and speed
                                    if(negative) {
                                        rotation *= -1;
                                    }
                                    #ifdef DEBUG_INTERFACE
                                        serial.printf("Rotations: %f",rotation);
                                        serial.printf("\t Velocity: ");
                                        serial.printf("%f \n\r", speed);
                                    #endif
                                    target_angular_velocity = speed;
                                    target_position = (rotation*360)+current_position;
                                    r_change = 1;
                                    v_change = 1;
                                    return;
                                }
                                if(buffer[index] == '.') {
                                    index++;
                                    if(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                                        float significance = 0.1f;
                                        while(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                                            speed += (buffer[index]-48)*significance;
                                            significance *= 0.1f;
                                            index++;
                                        }
                                        if(buffer[index] == '\0') {
                                            //return rotation and speed
                                            if(negative) {
                                                rotation *= -1;
                                            }
                                            #ifdef DEBUG_INTERFACE
                                                serial.printf("Rotations: %f",rotation);
                                                serial.printf("\t Velocity: ");
                                                serial.printf("%f \n\r", speed);
                                            #endif
                                            target_angular_velocity = speed;
                                            target_position = (rotation*360)+current_position;
                                            target_angular_velocity = speed;
                                            target_position = (rotation*360)+current_position;
                                            r_change = 1;
                                            v_change = 1;

                                            return;
                                        }
                                    }
                                } else {
                                    //invalid
                                    #ifdef DEBUG_INTERFACE
                                        serial.printf("Invalid Input! \n\r");
                                    #endif
                                    return;
                                }

                            } else {
                                //invalid
                                #ifdef DEBUG_INTERFACE
                                    serial.printf("Invalid Input!\n\r");
                                #endif
                                return;
                            }
                        }

            } else {
                //invalid
                #ifdef DEBUG_INTERFACE
                    serial.printf("Invalid Input!\n\r");
                #endif
                return;
            }
        }
        break;
        case 'V': {
            index++;
            if((buffer[index]-48) >= 0 &&  (buffer[index]-48) < 10) {
                speed += (buffer[index]-48);
                index++;
                while(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                    speed *= 10;
                    speed += (buffer[index]-48);
                    index++;
                }
                if(buffer[index] == '\0') {
                    //return speed
                    #ifdef DEBUG_INTERFACE
                        serial.printf("Velocity: %f \n\r",speed);
                    #endif
                    target_angular_velocity = speed;
                    v_change = 1;
                    return;
                }
                if(buffer[index] == '.') {
                    index++;
                    if(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                        float significance = 0.1f;
                        while(buffer[index]-48 >= 0 &&  buffer[index]-48 < 10) {
                            speed += (buffer[index]-48)*significance;
                            significance *= 0.1f;
                            index++;
                        }
                        if(buffer[index] == '\0') {
                            //return speed
                            #ifdef DEBUG_INTERFACE
                                serial.printf("Velocity: %f \n\r",speed);
                            #endif
                            target_angular_velocity = speed;
                            v_change = 1;
                            return;
                        }
                    }
                } else {
                    //invalid
                    #ifdef DEBUG_INTERFACE
                        serial.printf("Invalid Input!\n\r");
                    #endif
                    return;
                }
            }
            break;
            default:{
               //invalid
                #ifdef DEBUG_INTERFACE
                    serial.printf("Invalid Input!\n\r");
                #endif
                return; 
            }
        }
    }
}

float Serial_Interface::position(){
    return target_position;
}

float Serial_Interface::velocity(){
    return target_angular_velocity;
}

int Serial_Interface::velocity_change(){
    return v_change;
}

int Serial_Interface::position_change(){
    return r_change;
}

float Serial_Interface::saved_position(){
    return fabs(save_position);
}
