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
struct filled_slots{
    int slots[4];
    int tail;
};
typedef struct state_machine{
    enum KBOT_state state;
    enum KBOT_state next_state;
    double position[2];
    char ROBOT_ID;
    struct filled_slots f_slot;

};
typedef struct state_machine KBOT_state_machine;

void send_to_grid_position(double current_position[2],WbDeviceTag emitter,KBOT_state_machine* machine);
void intilize_state_machine(KBOT_state_machine* machine,char ROBOT_ID);


#endif