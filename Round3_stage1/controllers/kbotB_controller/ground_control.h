#ifndef GROUND_CONTROL
#define GROUND_CONTROL
enum transmitted_messages{
    
    REACHED = 2,
    SEND_POSITION = 4
};

enum recieved_messages{
    GOTO = 1,
    REQUEST_POSITION=3,
    DO_ANIMATION = 5
    
    
};
void send_my_position_to_kinect(WbDeviceTag Emmiter,WbDeviceTag Gps,WbDeviceTag Compass,char SELF_ID);
void send_reached_message(WbDeviceTag Emitter,char SELF_ID);
#endif