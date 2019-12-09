#include "helper.h"
#include "air_control.h"
#include <math.h>
#include <stdio.h>




const int number_of_slots = 1;
double grid_position_BLUE[4][3] = {
                                    {0.0680707,-0.53027,-M_PI_2},
                                    {-0.0680707,-0.53027,-M_PI_2},
                                    {-0.0680707,0.53027,M_PI_2},
                                    {0.0680707,0.53027,M_PI_2}
                                    };

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
int get_unfilled_closest_slot(KBOT_state_machine* machine,double position[2]){
    int slot = get_quadrant(position);
    for(int i=slot;i<4;i++){
        if(!is_filled(machine,i)){
            return i;
        }   
    }
    return -1;
    
}




void send_to_grid_position(double current_position[2],WbDeviceTag emitter,KBOT_state_machine* machine){
    char Robot_ID = machine->ROBOT_ID;
    if(Robot_ID == 1){
    int g =  get_unfilled_closest_slot(machine,current_position);
    printf("slot_no:%d\n",g);  
    send_goto_command(emitter,Robot_ID,grid_position_BLUE[g]);
    fill_slot(machine,g);
    }
}