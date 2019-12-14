#include "helper.h"
#include "air_control.h"
#include "comunication.h"
#include <math.h>
#include <stdio.h>
#define KINECT_SELF_ID 0;
const double distance_to_tower = 0.360135;
const double distance_to_tower_arm_x = 0.417998;
const double offset = 0.204001-0.41;
const double collision_range = 1;

double y_ranges[9] = {0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4};
double corresponding_x[8] = {
                            0.457,
                            0.460,
                            0.454,
                            0.440,
                            0.417,
                            0.385,
                            0.345,
                            0.290
                        };

const int number_of_slots = 1;
double grid_position_BLUE[4][3] = {
                                    {0.0680707,-0.53027,-M_PI_2},
                                    {-0.0680707,-0.53027,-M_PI_2},
                                    {-0.0680707,0.53027,M_PI_2},
                                    {0.0680707,0.53027,M_PI_2}
                                    };
double grid_position_RED[4][3] = {
                                    {0.169071,-0.642174,-M_PI_2},
                                    {-0.169071,-0.642174,-M_PI_2},
                                    {-0.169071,0.642174,M_PI_2},
                                    {0.169071,0.642174,M_PI_2},
                                };
/*
arm_position_red = {0.284000,-0.165000,0};
*/

void intilize_state_machine(KBOT_state_machine* machine,char ROBOT_ID){
    machine->state = INITIAL;
    machine->next_state = INITIAL;
    machine ->f_slot.tail = 0;
    machine ->ROBOT_ID = ROBOT_ID;

}

void fill_slot(KBOT_state_machine* machine,int i){
    machine->f_slot.slots[machine->f_slot.tail] = i;
    machine->f_slot.tail++;
}

int is_filled(KBOT_state_machine* machine,int slot){
    for(int i=0;i<machine->f_slot.tail;i++){
        if(machine->f_slot.slots[i] == slot){
            return 1;
        }
    }
    return 0;
}

int are_all_slots_filled(KBOT_state_machine* machine){
    
    if(machine->f_slot.tail < 4){
        return 0;
    }
    else{
        return 1;
    }
    
}
static double helper_get_distance2(double* pos1,double* pos2){
    double x2 = (pos1[0]-pos2[0])*(pos1[0]-pos2[0]);
    double y2 = (pos1[1]-pos2[1])*(pos1[1]-pos2[1]);
    return (x2+y2);
}
int get_quadrant(double position[2]){
    if(position[0]>0 && position[1]>0){
        return 0;
    }
    if(position[0]<0 && position[1]>0){
        return 1;
    }
    if(position[0]<0 && position[1]<0){
        return 2;
    }
    else{
        return 3;
    }
}

static int get_closest_unfilled_slot(KBOT_state_machine* machine,double position[2],double grid_position[][3]){
    double shortest_distance2 = INFINITY;
    int chosen_slot = -1;
    double grid_pos[2];
    
    for(int i =0;i<4;i++){
        grid_pos[0] = grid_position[i][0];
        grid_pos[1] = grid_position[i][1];
        double distance2 = helper_get_distance2(position,grid_pos);
        if(!is_filled(machine,i) && shortest_distance2>distance2){
            chosen_slot = i;
            shortest_distance2 = distance2;
        }

    }
    return chosen_slot;
}
int get_unfilled_closest_slot(KBOT_state_machine* machine,double position[2]){
    int slot = get_quadrant(position);
    for(int i=slot;i<4+slot;i++){
        if(!is_filled(machine,i%4)){
            return i;
        }   
    }
    return -1;
    
}




void send_to_grid_position(double current_position[2],WbDeviceTag emitter,KBOT_state_machine* machine){
    char Robot_ID = machine->ROBOT_ID;
    if(Robot_ID == 1){
    int g =  get_closest_unfilled_slot(machine,current_position,grid_position_BLUE);
    printf("slot_no:%d\n",g);  
    send_goto_command(emitter,Robot_ID,grid_position_BLUE[g]);
    fill_slot(machine,g);
    }
    else if(Robot_ID == 2){
            int g =  get_closest_unfilled_slot(machine,current_position,grid_position_RED);
            printf("slot_no:%d\n",g);  
            send_goto_command(emitter,Robot_ID,grid_position_RED[g]);
            fill_slot(machine,g);
    }
}

double piece_wise(double y){
    for(int i=0;i<8;i++){
        if(y_ranges[i]<y && y<y_ranges[i+1]){
            return corresponding_x[i];
        }
    }
    return corresponding_x[4];
}

void get_tower_destination(double position[2],double t_position[3],double y){
    t_position[0] = 1;
    double dist = 0.456/2 + piece_wise(y);
    if(position[1] <0){
        t_position[1] = -dist;
        t_position[2] = -M_PI_2;
    }
    if(position[1]>0){
        t_position[1] = dist;
        t_position[2] = +M_PI_2;
    }
}
void advanced_get_tower_destination(char ROBOT_ID,double t_position[3],double y){
    t_position[0] = 1;
    double dist = 0.456/2 + piece_wise(y)-0.02;
    if(ROBOT_ID == 1){
        t_position[1] = -dist;
        t_position[2] = -M_PI_2;
    }
    else{
        t_position[1] = dist;
        t_position[2] = +M_PI_2;
    }
}

void go_to_tower(WbDeviceTag emitter,KBOT_state_machine* machine,double tower_height){
    double pos[3];
    double y = offset + tower_height;
    //get_tower_destination(machine->position,pos,y);
    advanced_get_tower_destination(machine->ROBOT_ID,pos,y);
    
    send_goto_command(emitter,machine->ROBOT_ID,pos);

}

void place_box_on_tower_advanced(WbDeviceTag emitter,KBOT_state_machine* machine,double tower_height){
    printf("tower_height:%g",tower_height);
    double height = tower_height+offset;
    printf("y:%g,x:%g\n",height,piece_wise(height));
    place_box_on_tower(emitter,machine->ROBOT_ID,piece_wise(height),height);
}



int has_collision(KBOT_state_machine mach1,KBOT_state_machine mach2){
    double X2 = (mach1.position[0]-mach2.position[0])*(mach1.position[0]-mach2.position[0]);
    double Y2 = (mach1.position[1]-mach2.position[1])*(mach1.position[1]-mach2.position[1]);
    //printf("distance between bots%g",X2+Y2);
    if(X2+Y2<0.5){
        return 3;
    }
    if(X2+Y2<1){
        return 1;
    }
    if(X2+Y2>1.5){
        return 2;
    }
    
    return 0;
}


int am_i_going(KBOT_state_machine mach){
    printf("state of mach:%d\n",mach.state);
    if(mach.state == GOING_TO_TOWER || mach.state ==GOING_TO_GRID|| mach.state == GOING_TO_BOX || mach.state == AVOIDANCE){
        return 1;
    }
    else{
        return 0;
    }
}

void set_avoidance_state(KBOT_state_machine* mach){
    switch(mach->state){
        case GOING_TO_BOX:
            mach->preavoidance_state = INITIAL;
            mach->next_state = GOING_TO_BOX;
            break;
        case GOING_TO_GRID:
            mach->preavoidance_state = BOX_PICKED;
            mach->f_slot.tail--;
            mach->next_state = GOING_TO_GRID;
            break;
        case GOING_TO_TOWER:
            mach->preavoidance_state = BOX_PICKED;
            mach->next_state = GOING_TO_TOWER;
            break;
        
    }
    mach->state = AVOIDANCE;
}

void go_to_avoidance_point(KBOT_state_machine avoiding_machine,KBOT_state_machine machine,WbDeviceTag emitter){
    double translation[2];
    translation[0] = avoiding_machine.position[0] - machine.position[0];
    translation[1] = avoiding_machine.position[1] - machine.position[1];
    double target[3];
    target[0] = translation[0] + avoiding_machine.position[0];
    target[1] = translation[1] + avoiding_machine.position[1];
    target[2] = 0;
    simple_go_to_command(emitter,avoiding_machine.ROBOT_ID,target); 

}

void resume_path(KBOT_state_machine* mach){
        if(mach->state == AVOIDANCE){
            mach->state = mach->preavoidance_state;
        }
}