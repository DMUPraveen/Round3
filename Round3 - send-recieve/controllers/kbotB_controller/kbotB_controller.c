#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>
#include <webots/compass.h>
#include <webots/gps.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "comunication.h"

#define TIME_STEP 16
double get_bearing_in_degrees(WbDeviceTag compass) {
  const double *north = wb_compass_get_values(compass);
  double rad = atan2(north[0],north[1]);
  
  return rad;
}


int main(int argc, char **argv)
{
  // initialization: robot
  wb_robot_init();

  //intializing the reciver
  WbDeviceTag RECIEVER;
  RECIEVER = wb_robot_get_device("receiver");
  wb_receiver_enable(RECIEVER, TIME_STEP);
  //intiailizong compass
  WbDeviceTag compass;
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass,TIME_STEP);
  //initializing gps
  WbDeviceTag gps;
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps,TIME_STEP); 

  // initialization: robot base, arm and the gripper
  base_init();
  arm_init();
  gripper_init();
  //base_turn_left();
  base_goto_init(TIME_STEP);
  base_goto_set_target(4,4,1.5);

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1)
  {
    
    base_goto_run();
    const double* pos =wb_gps_get_values(gps);
    //printf("%g,%g,%g\n",pos[0],pos[1],pos[2]);
    
    //printf("bearing:%g\n",get_bearing_in_degrees(compass));
    //const double *north = wb_compass_get_values(compass);
    //printf("%g,%g\n",north[0],north[1]);
    if (wb_receiver_get_queue_length(RECIEVER) > 0)
    {
      printf("recieved\n");
      const char *message = wb_receiver_get_data(RECIEVER);
      int message_length = wb_receiver_get_data_size(RECIEVER);
      int num = (int)message[0];
      Message my_message;
      receive_message(message,message_length,&my_message);
      printf("type: %d, data: %s\n", my_message.type, my_message.data);
      wb_receiver_next_packet(RECIEVER);
    }
  }
  wb_robot_cleanup();
  return 0;
}
