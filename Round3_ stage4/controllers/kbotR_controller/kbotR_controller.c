#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>

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
  
  // initialization: robot base, arm and the gripper
  base_init();
  arm_init();
  gripper_init();
  
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    //base_forwards(); 
       
  }
  wb_robot_cleanup();
  return 0;
}
