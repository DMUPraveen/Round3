#include "helper.h"
#include "air_control.h"
#include <math.h>




const int number_of_slots = 1;
double grid_position[3] = {-0.0680707,0.53027,M_PI_2};


void send_to_grid_position(int i,int j,WbDeviceTag emitter,char Robot_ID){
    send_goto_command(emitter,Robot_ID,grid_position);
}