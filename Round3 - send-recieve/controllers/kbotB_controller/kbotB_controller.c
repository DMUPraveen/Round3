#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TIME_STEP 16


int main(int argc, char **argv) {
  // initialization: robot
  wb_robot_init();

  //intializing the reciver
  WbDeviceTag RECIEVER;
  RECIEVER = wb_robot_get_device("receiver");
  wb_receiver_enable(RECIEVER,TIME_STEP);

  
  // initialization: robot base, arm and the gripper
  base_init();
  arm_init();
  gripper_init();
  
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    //base_backwards();
    if(wb_receiver_get_queue_length(RECIEVER) > 0){
      const char* message = wb_receiver_get_data(RECIEVER);
      double signal = wb_receiver_get_signal_strength(RECIEVER);
      printf("recieved: %s, with siganl strength: %g\n",message,signal);
      wb_receiver_next_packet(RECIEVER);

    }
       
  }
  wb_robot_cleanup();
  return 0;
}
