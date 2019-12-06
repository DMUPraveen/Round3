#ifndef GROUND_CONTROL
#define GROUND_CONTROL
enum transmitted_messages{
    
    REACHED = 2,
    SEND_POSITION = 4
};

enum recieved_messages{
    GOTO = 1,
    REQUEST_POSITION=3
    
    
};
void send_my_position_to_kinect(WbDeviceTag Emmiter,WbDeviceTag Gps,WbDeviceTag Compass);
void send_reached_message(WbDeviceTag Emitter);
#endif