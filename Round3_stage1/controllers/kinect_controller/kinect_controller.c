#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>
#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/emitter.h>
#include <webots/keyboard.h>
#include <webots/display.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "image_recognition.h"
#include "range_finder_lib.h"
#include "navigation.h"
#include "comunication.h"
#include "air_control.h"

#define TIME_STEP 16
#define SELF_ID 0
#define KBOTB_ID 1

int main(int argc, char **argv)
{
  // initialization: robot
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
  int key;
  COLOUR blob_color = RED;
  FILE *file;
  int kbotB_status = 0;
  double kbotB_position[2];

  // initialization: range finder
  int rf_width, rf_height;
  float distance;

  // initialization: kinect
  WbDeviceTag camera;
  WbDeviceTag range_finder;
  WbDeviceTag display;
  display = wb_robot_get_device("display");
  camera = wb_robot_get_device("kinect color");
  range_finder = wb_robot_get_device("kinect range");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);
  wb_range_finder_enable(range_finder, TIME_STEP);

  rf_width = wb_range_finder_get_width(range_finder);
  rf_height = wb_range_finder_get_height(range_finder);

  // initialization: camera parameters
  int width, height;
  int red, blue, green;
  width = wb_camera_get_width(camera);
  height = wb_camera_get_height(camera);

  //intializing the emitter
  WbDeviceTag EMITTER;
  WbDeviceTag RECIEVER;
  EMITTER = wb_robot_get_device("emitter");
  RECIEVER = wb_robot_get_device("receiver");
  // initialization: color-boxes
  int i, j;
  int no_boxes = 30;
  WbNodeRef KKboxes[no_boxes];
  char Obs_names[no_boxes][6];
  char tmp[3];
  /* random position vectors-values- for color boxes defined as KKBx */
  srand(time(0));
  double values[no_boxes][3];
  for (i = 0; i < no_boxes; i++)
  {
    char kb[6] = {"KKB"};
    sprintf(tmp, "%d", i);
    strcpy(Obs_names[i], strcat(kb, tmp));
    for (j = 0; j < 3; j++)
    {
      values[i][j] = (rand() % 95 - 47) / 10.0;
      //values[i][j]= (rand()%81-35)/10;
      values[i][1] = 0.04;
      //fprintf(stderr, "VALUE, %d\n",values[i][1] );
    }
    if (fabs(values[i][0]) < 1.5 && fabs(values[i][2]) < 1.5)
    {
      if (values[i][0] < 0)
      {
        values[i][0] = values[i][0] - 1.5;
      }
      else
      {
        values[i][0] = values[i][0] + 1.5;
      }
    }
  }
  /* setting KKBx translations to above random values */
  for (i = 0; i < no_boxes; i++)
  {
    KKboxes[i] = wb_supervisor_node_get_from_def(Obs_names[i]);
    WbFieldRef tr = wb_supervisor_node_get_field(KKboxes[i], "translation");
    wb_supervisor_field_set_sf_vec3f(tr, values[i]);
  }

  // random cylindrical tower platform height
  WbNodeRef TowerSD, TowerSH;
  float rvalue;
  double TWtl[1][3];
  rvalue = (rand() % 36 + 20) / 100.0;
  TWtl[0][0] = 1;
  TWtl[0][1] = 0.001 + rvalue / 2;
  TWtl[0][2] = 0;
  TowerSD = wb_supervisor_node_get_from_def("TOWER");
  WbFieldRef hy = wb_supervisor_node_get_field(TowerSD, "translation");
  TowerSH = wb_supervisor_node_get_from_def("TOWER_cy");
  WbFieldRef ht = wb_supervisor_node_get_field(TowerSH, "height");
  wb_supervisor_field_set_sf_vec3f(hy, TWtl[0]);
  wb_supervisor_field_set_sf_float(ht, rvalue);

  int num;
  num = 0;
  int camera_image[height * width][3];
  int filter_array[height * width];
  int r_filter_array[rf_height * rf_width];
  wb_receiver_enable(RECIEVER, TIME_STEP);
  Box_array RED_ARRAY;
  RED_ARRAY.tail = -1;
  Box_array BLUE_ARRAY;
  BLUE_ARRAY.tail = -1;
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1)
  {
    // collection of range-finder and camera images
    const float *rfimage = wb_range_finder_get_range_image(range_finder);
    const unsigned char *image = wb_camera_get_image(camera);
    filter_range_image(r_filter_array, range_finder, rf_width, rf_height, 2.88);

    /////////////////////////////////////////////Transmission//////////////////////////////////
    if (kbotB_status == 0)
    {
      kbotB_status = 1;
      //double position[3] = {4, 4, 0};
      request_robot_position(EMITTER,1);
      //send_goto_command(EMITTER, KBOTB_ID, position);
      printf("requesting KbotB positionl\n");
    };
    if(kbotB_status == 2){
      fill_and_sort_boxes(&RED_ARRAY,&BLUE_ARRAY,camera);
      double destination_pos[3];
      //printf("%d\n",RED_ARRAY.tail);
      get_destination(kbotB_position,RED_ARRAY,destination_pos);
      send_goto_command(EMITTER,1,destination_pos);
      kbotB_status = 3;
      printf("sending webot to %g,%g,%g\n",destination_pos[0],destination_pos[1],destination_pos[2]);
    }
    if(kbotB_status == 4){
      send_do_animation_command(EMITTER,1,0);
      kbotB_status = 5;
    }


    //////////////////////////////////////////RECEIVE///////////////////////////////////////////
    Command command;
    int message_recieved = message_handler(RECIEVER, SELF_ID, &command);
    if (message_recieved)
    {

      if (command.type == SEND_POSITION && kbotB_status ==1)
      {
        kbotB_status = 2;
        double* position = (double*)command.data;
        printf("KbotB has replyed with position %g,%g,%g,%g\n",position[0],position[1],position[2],position[3]);
        kbotB_position[0] = position[0];
        kbotB_position[1] = position[3];
      }
      if(command.type == REACHED && kbotB_status ==3){
        printf("KbotB has reached its destination\n");
        kbotB_status =4;
      }
      delete_command(&command);
    }
  }
  wb_robot_cleanup();
  return 0;
}
