#ifndef HELPER
#define HELPER
void set_arm_position(double x,double y,double z);
int pick_up_box();
int place_box();

enum mystates{
    INITIAL,
    GOING,
    REACHED_DESTINATION,
    DOING,
    DONE,
    AVOIDING
    
};
enum animation{
    NO_ANIMATION = -1,
    PICK_BOX = 0,
    PLACE_BOX_IN_GRID = 1,
    PLACE_BOX_ON_TOWER = 2
};
typedef struct state_mach{
    enum mystates status;
    enum animation current_animation;
    int path;
    double position[3];
    double tower_arm[2];
};
typedef struct state_mach State_mach;

int do_animation(enum animation Anim);
int do_tower_task_and_put_box(State_mach* machine);
int advanced_go_to();
void set_target_path_and_position(State_mach* machine,double position[3],WbDeviceTag gps);
#endif