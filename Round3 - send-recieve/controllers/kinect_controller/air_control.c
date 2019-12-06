#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdio.h>
#include "comunication.h"
#include "air_control.h"

void request_robot_position(WbDeviceTag emitter,char ROBOT_ID){
    Command request_position;
    request_position.id = ROBOT_ID;
    request_position.type = REQUEST_POSITION;
    request_position.data_length = 1;
    char data = 0;
    request_position.data = &data;
    send_message(&request_position,emitter);

}

void send_goto_command(WbDeviceTag emitter,char ROBOT_ID,double position[3]){
    Command goto_command;
    goto_command.id = ROBOT_ID;
    goto_command.data_length = sizeof(double)*3;
    char* cpositions = (char*)position;
    char* data_holder[goto_command.data_length];
    for(int i=0; i<goto_command.data_length; i++){
        data_holder[i] = cpositions[i];
    }
    goto_command.data = data_holder;
    send_message(&goto_command,emitter); 
}
