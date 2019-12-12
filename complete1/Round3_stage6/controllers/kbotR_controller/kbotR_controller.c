#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/keyboard.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "comunication.h"
#include "ground_control.h"
#include "helper.h"
#include "advanced_navigation.h"

#define TIME_STEP 16
double get_bearing_in_degrees(WbDeviceTag compass)
{
  const double *north = wb_compass_get_values(compass);
  double rad = atan2(north[0], north[1]);

  return rad;
}
#define SELF_ID 2
#define KINECT_ID 0

int main(int argc, char **argv)
{
  // initialization: robot
  wb_robot_init();

  //enum mystates self_status=INITIAL;
  State_mach self;
  self.status = INITIAL;
  self.current_animation = NO_ANIMATION;
  /*
  int pickup_status =0;
  double time =0;
  */
  //intializing the reciver
  WbDeviceTag RECIEVER;
  WbDeviceTag EMMITER;
  RECIEVER = wb_robot_get_device("receiver");
  EMMITER = wb_robot_get_device("emitter");
  wb_receiver_enable(RECIEVER, TIME_STEP);
  //intiailizong compass
  WbDeviceTag compass;
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  //initializing gps
  WbDeviceTag gps;
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  // initialization: robot base, arm and the gripper
  base_init();
  arm_init();
  gripper_init();
  //base_turn_left();
  //base_goto_init(TIME_STEP);
  //base_goto_set_target(4,4,0.1234);
  wb_keyboard_enable(TIME_STEP);
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1)
  {

    if (self.status == GOING)
    {
      //base_goto_run();
      if (advanced_go_to(&self))
      {
        self.status = REACHED_DESTINATION;
      }
    }
    if (self.status == REACHED_DESTINATION)
    {
      send_reached_message(EMMITER, SELF_ID);
      self.status = INITIAL;
    }
    if (self.status == DOING && self.current_animation != PLACE_BOX_ON_TOWER)
    {
      int is_picked = do_animation(self.current_animation);

      if (is_picked == 1)
      {
        //printf("sending mesage\n");
        send_done_animation_to_kinect(EMMITER, SELF_ID, self.current_animation);
        self.status = DONE;
      }
    }
    if (self.status == DOING && self.current_animation == PLACE_BOX_ON_TOWER)
    {
      int is_placed = do_tower_task_and_put_box(&self);
      if (is_placed)
      {
        self.status = DONE;
      }
    }
    while (wb_receiver_get_queue_length(RECIEVER) > 0)
    {
      Command command;
      int message_recieved = message_handler(RECIEVER, SELF_ID, &command);
      if (message_recieved && command.caller_id == 0 && command.id == SELF_ID)
      {
        if (command.type == GOTO)
        {
          double *position = (double *)command.data;
          //set_target(position);
          initialize_naviagtion();
        //set_target(position); 
        set_target_path_and_position(&self,position,gps);
       
          printf("message reieved going to %g,%g,%g\n", position[0], position[1], position[2]);
          self.status = GOING;
        }
        if (command.type == REQUEST_POSITION)
        {
          send_my_position_to_kinect(EMMITER, gps, compass, SELF_ID);
        }
        if (command.type == DO_ANIMATION)
        {
          self.status = DOING;
          self.current_animation = command.data[0];
        }
        if (command.type == PLACE_ON_TOWER)
        {
          self.status = DOING;
          self.current_animation = PLACE_BOX_ON_TOWER;
          double *arm_pos = (double *)command.data;
          self.tower_arm[0] = arm_pos[0];
          self.tower_arm[1] = arm_pos[1];
          printf("tower_pos x:%g,y:%g\n", self.tower_arm[0], self.tower_arm[1]);
        }

        delete_command(&command);
      }
    }
  }
  wb_robot_cleanup();
  return 0;
}
