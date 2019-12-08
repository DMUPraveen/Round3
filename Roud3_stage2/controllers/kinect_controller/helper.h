#ifndef HELPER
#define HELPER
#include <webots/robot.h>
enum KBOT_state{
    INITIAL,
    POSITION_REQUESTED,
    POSITION_KNOWN,
    GOING_TO_BOX,
    BOX_REACHED,
    BOX_PICKING,
    BOX_PICKED,
    GOING_TO_GRID,
    GONE_TO_GRID,
    PLACE_IN_GRID,
    PLACED_IN_GRID
};

typedef struct state_machine{
    enum KBOT_state state;
    double position[2];     
};
typedef struct state_machine KBOT_state_machine;
void send_to_grid_position(int i,int j,WbDeviceTag emitter,char Robot_ID);



#endif