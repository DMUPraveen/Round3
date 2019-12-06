#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include "comunication.h"
#include <windows.h>

#include <stdio.h>
#include <math.h>
#include <string.h>

/*
void dump_recieved_packets(WbDeviceTag receiver)
{
    while (wb_receiver_get_queue_length(receiver) > 0)
    {
        wb_reciever_next_packet(receiver);
    }
}
*/
int get_byte_stream_length(int data_length){
    return (data_length+2);
}
int construct_message(Command *scommand, char *byte_stream, int stream_length)
{
    if (get_byte_stream_length(scommand->data_length) == stream_length)
    {
        byte_stream[0] = scommand->id;   //setting the first byte to the id of the addressed robot
        byte_stream[1] = scommand->type; //setting the second byte to the type of command that follows
        for (int i = 0; i < scommand->data_length; i++)
        {
            byte_stream[i + 2] = (scommand->data)[i];
        }
        return 0; //success
        //printf("done");
    }
    else
    {
        return 1; //failure
    }
}
/*
int get_data_length(char command_type)
{
    switch (command_type)
    {
    case 0:
        //command type single_character
        return 1;
    case 1:
        //position type [double,double,double]
        return 3 * sizeof(double);
    case 2:
        //message type string of 10 chars;
        return 10 * sizeof(char);
    default:
        return 0;
    }
}
*/
char get_message_id(const char *message)
{
    return message[0];
}
char get_message_type(const char *message)
{
    return message[1];
}
int get_data_length(int size)
{
    return (size - 2);
}

void construct_goto_command(char robot,double* position,Command* goto_command){
    goto_command->id = robot;
    goto_command->type = 1;
    
    goto_command ->data = (char*)position;
    goto_command ->data_length = sizeof(double)*3;
    

}
void send_message(Command* scommand,WbDeviceTag emitter){
    int byte_stream_length = get_byte_stream_length(scommand->data_length);
    char byte_stream[byte_stream_length];
    construct_message(scommand,byte_stream,byte_stream_length);
    wb_emitter_send(emitter,byte_stream,byte_stream_length);


}
void get_target_position(char* data,double* position){
    double* position_ptr = (double*)data;
    for(int i=0; i<3;i++){
        position[i] = position_ptr[i];
    }

}

void get_message_data(const char *message, int message_length, char *data_holder)
{
    for (int i = 2; i < message_length; i++)
    {
        data_holder[i - 2] = message[i];
    }
}

int deconstruct_message(const char *message, int message_size, Command *scommand, char *data_holder, int data_holder_length)
{
    scommand->id = get_message_id(message);
    scommand->type = get_message_type(message);
    scommand->data_length = get_data_length(message_size);
    if (data_holder_length >= get_data_length(message_size))
    {
        get_message_data(message, message_size, data_holder);
        scommand->data = data_holder;
        return 0;//message_deconstructed successfully
    }
    else{
        return 1;//data_holder buffer is not of enough size
    }
}

void delete_command(Command* scommand){
    printf("scommand:data%d\n",scommand->data);
    free(scommand->data);
}

int message_handler(WbDeviceTag reciever,char ROBOT_ID,Command* scommand){
    int is_message_recieved =0;
    if(wb_receiver_get_queue_length(reciever)>0){
        printf("recieved\n");
        int message_size = wb_receiver_get_data_size(reciever); 
        char* message = wb_receiver_get_data(reciever);
        int data_holder_size = get_data_length(message_size);
        char* data_holder = (char*)malloc(data_holder_size);
        deconstruct_message(message,message_size,scommand,data_holder,data_holder_size);
        wb_receiver_next_packet(reciever);
        printf("%d\n",scommand->type);
        printf("data_holder:%d\n",data_holder);
        if((scommand->id) == ROBOT_ID){
            return 1;
        }
        else{
            delete_command(scommand);
            return 0;
        }
    }
    return 0;
}