#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <math.h>
#include "comunication.h"
#include "air_control.h"
#include "helper.h"

#define KINECT_SELF_ID 0


void request_robot_position(WbDeviceTag emitter,char ROBOT_ID){
    Command request_position;
    request_position.id = ROBOT_ID;
    request_position.type = REQUEST_POSITION;
    request_position.caller_id = KINECT_SELF_ID;
    request_position.data_length = 1;
    char data = 0;
    request_position.data = &data;
    send_message(&request_position,emitter);

}

void send_goto_command(WbDeviceTag emitter,char ROBOT_ID,double position[3]){
    Command goto_command;
    goto_command.id = ROBOT_ID;
    goto_command.type = GOTO;
    goto_command.caller_id = KINECT_SELF_ID;
    goto_command.data_length = sizeof(double)*3;
    char* cpositions = (char*)position;
    char data_holder[goto_command.data_length];
    for(int i=0; i<goto_command.data_length; i++){
        data_holder[i] = cpositions[i];
    }
    goto_command.data = data_holder;
    send_message(&goto_command,emitter); 
}

void send_do_animation_command(WbDeviceTag emmiter,char ROBOT_ID,enum animation ANIMATION){
    Command do_animation;
    do_animation.id = ROBOT_ID;
    do_animation.data_length = 1;
    do_animation.caller_id = KINECT_SELF_ID;
    do_animation.data = &ANIMATION;
    do_animation.type = DO_ANIMATION;
    send_message(&do_animation,emmiter);
}

void place_box_on_tower(WbDeviceTag emitter,char ROBOT_ID,double x,double y){
    Command place_on_tower;
    place_on_tower.id = ROBOT_ID;
    place_on_tower.data_length = sizeof(double)*2;
    place_on_tower.caller_id = KINECT_SELF_ID;
    double data_cos[2] = {x,y};
    char* datas = (char*)data_cos;
    place_on_tower.data = datas;
    printf("%g,%g\n",((double*)place_on_tower.data)[0],((double*)place_on_tower.data)[1]);
    place_on_tower.type = PLACE_ON_TOWER;
    send_message(&place_on_tower,emitter);

}
