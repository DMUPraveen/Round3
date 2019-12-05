


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

typedef struct message{
    char type;
    char* data;
    int data_length;
}Message;
int construct_message( Message command,char *message, int message_length);
int receive_message(char* message,int message_size,Message command);

#endif