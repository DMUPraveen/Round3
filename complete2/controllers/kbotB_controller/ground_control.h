#ifndef GROUND_CONTROL
#define GROUND_CONTROL
enum transmitted_messages{
    
    REACHED = 2,
    SEND_POSITION = 4,
    DONE_ANIMATION = 6,
    PLACED_ON_TOWER = 7
};

enum recieved_messages{
    GOTO = 1,
    REQUEST_POSITION=3,
    DO_ANIMATION = 5,
    PLACE_ON_TOWER = 8,
    SIMPLE_GO_TO = 30
    
    
};


void send_my_position_to_kinect(WbDeviceTag Emmiter,WbDeviceTag Gps,WbDeviceTag Compass,char SELF_ID);
void send_reached_message(WbDeviceTag Emitter,char SELF_ID);
void send_done_animation_to_kinect(WbDeviceTag Emitter, char SELF_ID, char DONE_ANIMATION);
#endif