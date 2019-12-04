#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>
#include <webots/keyboard.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TIME_STEP 16
double Arm_pos[5] = {0, M_PI_2,M_PI_2,M_PI_2,M_PI_2};

void set_arm_Pos(double ARM_POS[])
{
  for (int i = 0; i < 5; i++)
  {
    arm_set_sub_arm_rotation(i, ARM_POS[i]);
  }
}
void increment_pos(int arm,double* ARM_POS){
  ARM_POS[arm] = ARM_POS[arm] + 0.01;
  set_arm_Pos(ARM_POS);
  printf("%f,%f,%f,%f,%f\n",ARM_POS[0],ARM_POS[1],ARM_POS[2],ARM_POS[3],ARM_POS[4]);

}

void decrement_pos(int arm,double* ARM_POS){
  ARM_POS[arm] = ARM_POS[arm] - 0.01;
  set_arm_Pos(ARM_POS);
  printf("%f,%f,%f,%f,%f\n",ARM_POS[0],ARM_POS[1],ARM_POS[2],ARM_POS[3],ARM_POS[4]);


}
int mode =0;
int main(int argc, char **argv)
{
  // initialization: robot
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);

  int key;

  // initialization: robot base, arm and the gripper
  base_init();
  arm_init();
  gripper_init();
  typedef enum
  {
    W = 87,
    A = 65,
    S = 83,
    D = 68,
    B = 66,
    T = 84,
    X_plus = 55,
    X_minus = 52,
    Y_plus = 56,
    Y_minus = 53,
    Z_plus = 57,
    Z_minus = 54,
    Q = 81,
    E = 69,
    G = 71,
    U = 85,
    F = 70,
    H = 72

  } keyboard_controls_blue;

  float x = 0.20;
  float y = 0.10;
  float z = 0.00;

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1)
  {
    //base_backwards();
    //arm_ik(x, y, z);
    key = wb_keyboard_get_key();

    
    //printf("x:%f, y:%f, z:%f \n", x, y, z);

    switch (key)
    {
    case A:
      base_turn_left();
      break;
    case D:
      base_turn_right();
      break;
    case S:
      base_backwards();
      break;
    case W:
      base_forwards();
      break;
    case B:
      base_reset();
      break;
    case X_plus:
      if(mode == 0){
      increment_pos(3,Arm_pos);
      }
      if(mode ==1){
        x = x + 0.01;
        arm_ik(x,y,z);
        printf("x:%f, y:%f, z:%f \n", x, y, z);
      }
      break;
    case X_minus:
      if(mode == 0){
      decrement_pos(3,Arm_pos);
      }
      if(mode ==1){
        x = x - 0.01;
        arm_ik(x,y,z);
        printf("x:%f, y:%f, z:%f \n", x, y, z);
      }
      break;
    case Y_plus:
      if(mode == 0){
      increment_pos(1,Arm_pos);
      }
      if(mode ==1){
        y = y + 0.01;
        arm_ik(x,y,z);
        printf("x:%f, y:%f, z:%f \n", x, y, z);
      }
      break;
    case Y_minus:
      if(mode == 0){
      decrement_pos(1,Arm_pos);
      }
      if(mode ==1){
        y = y - 0.01;
        arm_ik(x,y,z);
        printf("x:%f, y:%f, z:%f \n", x, y, z);
      }
      break;
    case Z_plus:
      if(mode == 0){
      increment_pos(2,Arm_pos);
      }
      if(mode ==1){
        z = z + 0.01;
        arm_ik(x,y,z);
        printf("x:%f, y:%f, z:%f \n", x, y, z);
      }
      break;
    case Z_minus:
      if(mode == 0){
      decrement_pos(2,Arm_pos);
      }
      if(mode ==1){
        z = z - 0.01;
        arm_ik(x,y,z);
        printf("x:%f, y:%f, z:%f \n", x, y, z);
      }
      break;
    case Q:
      arm_set_height(ARM_BACK_PLATE_LOW);
      break;
    case E:
      arm_set_height(ARM_FRONT_FLOOR);
      break;
    case G:
      gripper_grip();
      break;
    case U:
      gripper_release();
      break;
    case F:
      mode = 0;
      printf("%d\n",mode);
      break;
    case H:
      mode = 1;
      printf("%d\n",mode);
      break;
    case 51:
      increment_pos(4,Arm_pos);
      break;
    case 49:
      decrement_pos(4,Arm_pos);
      break;

    }

  }
  wb_robot_cleanup();
  return 0;
}
