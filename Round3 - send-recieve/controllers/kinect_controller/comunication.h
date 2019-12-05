


#ifndef COMUNICATION
#define CUMUNICATION

#include <webots/receiver.h>
#include <webots/emitter.h>



#include <stdio.h>
#include <math.h>

typedef struct comunication{
    WbDeviceTag emmiter;
    WbDeviceTag reciever;

}comunicator;

typedef struct command{
    char type;
    char id;
    int data_length;
    char* data
}Command;


int deconstruct_message(const char *message, int message_size, Command *scommand, char *data_holder, int data_holder_length);
int get_data_length(int size);
int construct_message(Command *scommand, char *byte_stream, int stream_length);
int get_byte_stream_length(int data_length);
void get_target_position(char* data,double* position);
void construct_goto_command(char robot,double* position,Command* goto_command);
#endif