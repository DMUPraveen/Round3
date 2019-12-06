#ifndef AIR_CONTROL
#define AIR_CONTROL
enum transmitted_messages{
        GOTO = 1,
        REQUEST_POSITION = 3
};

enum recieved_messages{

    SEND_POSITION = 4,
    REACHED = 2
    
};
void request_robot_position(WbDeviceTag emitter,char ROBOT_ID);
void send_goto_command(WbDeviceTag emitter,char ROBOT_ID,double position[3]);
#endif