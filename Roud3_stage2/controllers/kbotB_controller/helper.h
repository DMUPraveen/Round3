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
    DONE
    
};
enum animation{
    NO_ANIMATION = -1,
    PICK_BOX = 0,
    PLACE_BOX_IN_GRID = 1,
};
typedef struct state_mach{
    enum mystates status;
    enum animation current_animation;
};
typedef struct state_mach State_mach;

int do_animation(enum animation Anim);
#endif