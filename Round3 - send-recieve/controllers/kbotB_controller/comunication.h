


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
    
    int data_length;
    char data[];
}Message;
int construct_message( Message command,char *message, int message_length);
int receive_message(const char* message,int message_size,Message* command);

#endif