#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include "comunication.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
/*
void dump_recieved_packets(WbDeviceTag receiver)
{
    while (wb_receiver_get_queue_length(receiver) > 0)
    {
        wb_receiver_next_packet(receiver);
    }
}
*/

int construct_message( Message command,char *message, int message_length)
{
    if(command.data_length+1 == message_length){
        message[0] = command.type;
        for(int i=0; i< command.data_length;i++){
            message[i+1] = command.data[i+1];
        }
        return 0;
    }
    else{
        return 1;
    }  
}

int get_data_length(char command_type){
    switch(command_type){
        case 0:
        //command type single_character
            return 1;
        case 1:
        //position type [double,double,double]
            return 3*sizeof(double);
        case 2:
        //message type string of 10 chars;
            return 10*sizeof(char);
        default:
            return 0;


    }
}
int receive_message(char* message,int message_size,Message command){
    command.type = message[0];
    command.data_length = message_size -1;
    if(command.data_length != 0){
        for(int i=0; i<command.data_length;i++){
            command.data[i+1] = message[i+1];
        }
    }

}