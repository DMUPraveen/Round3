#include <webots/robot.h>
#ifndef AIR_CONTROL
#define AIR_CONTROL
enum transmitted_messages{
        GOTO = 1,
        REQUEST_POSITION = 3,
        DO_ANIMATION = 5,
        PLACE_ON_TOWER = 8
};

enum recieved_messages{

    SEND_POSITION = 4,
    REACHED = 2,
    DONE_ANIMATION = 6,
    PLACED_ON_TOWER = 7
    
};
enum animation{
    PICK_BOX = 0,
    PLACE_BOX_IN_GRID = 1,
    PLACE_BOX_ON_TOWER = 3
};



void request_robot_position(WbDeviceTag emitter,char ROBOT_ID);
void send_goto_command(WbDeviceTag emitter,char ROBOT_ID,double position[3]);
void send_do_animation_command(WbDeviceTag emmiter,char ROBOT_ID,enum animation ANIMATION);
void place_box_on_tower(WbDeviceTag emitter,char ROBOT_ID,double x,double y);
#endif