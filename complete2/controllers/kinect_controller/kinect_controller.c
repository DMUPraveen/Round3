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
#include "helper.h"

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
  /*
  int kbotB_status = 0;
  double kbotB_position[2];
  */
  KBOT_state_machine KbotB;
  KBOT_state_machine KbotR;
  KBOT_state_machine Kbots[2] = {KbotB, KbotR};
  for (int i = 0; i < 2; i++)
  {
    intilize_state_machine(&Kbots[i], i + 1);
  }

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
  Box_array BLUE_ARRAY;

  Box_array colour_arrays[2] = {RED_ARRAY, BLUE_ARRAY};
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1)
  {
    // collection of range-finder and camera images
    const float *rfimage = wb_range_finder_get_range_image(range_finder);
    const unsigned char *image = wb_camera_get_image(camera);
    filter_range_image(r_filter_array, range_finder, rf_width, rf_height, 2.88);
    //printf("tower_height:real:%g,calculated:%g\n",rvalue,find_tower_height(rfimage,rf_width));
    /////////////////////////////////////////////Transmission//////////////////////////////////
    for (int i = 0; i < 2; i++)
    {

      if (Kbots[i].state == PLACED_IN_GRID)
      {
        Kbots[i].state = INITIAL;
        Kbots[i].pos_know = 0;
        printf("back to square one\n");
      }

      if (Kbots[i].state == INITIAL)
      {

        request_robot_position(EMITTER, Kbots[i].ROBOT_ID);
        Kbots[i].state = POSITION_REQUESTED;
        Kbots[i].next_state = GOING_TO_BOX;
        printf("requesting Kbot positionl\n");
      };
      if (Kbots[i].state == POSITION_KNOWN)
      {
        if (Kbots[i].next_state == GOING_TO_BOX)
        {
          for (int i = 0; i < 2; i++)
          {
            //Box_array RED_ARRAY;
            colour_arrays[i].tail = -1;
          }
          printf("calculating\n");
          fill_and_sort_boxes(&colour_arrays[0], &colour_arrays[1], camera);
          //printf("filled\n");
          double destination_pos[3];
          Kbots[i].box_id = get_destination(Kbots[i].position, colour_arrays[i], destination_pos);
          send_goto_command(EMITTER, Kbots[i].ROBOT_ID, destination_pos);
          Kbots[i].state = GOING_TO_BOX;
          printf("sending webot to %g,%g,%g\n", destination_pos[0], destination_pos[1], destination_pos[2]);
        }
        if (Kbots[i].next_state == GOING_TO_GRID)
        {
          send_to_grid_position(Kbots[i].position, EMITTER, &Kbots[i]);
          Kbots[i].state = GOING_TO_GRID;
        }
        if (Kbots[i].next_state == GOING_TO_TOWER)
        {
          go_to_tower(EMITTER, &Kbots[i], find_tower_height(rfimage, rf_width));
          Kbots[i].state = GOING_TO_TOWER;
        }
      }
      if (Kbots[i].state == BOX_REACHED)
      {
        send_do_animation_command(EMITTER, Kbots[i].ROBOT_ID, PICK_BOX);
        printf("KbotB is picking up the box\n");
        Kbots[i].state = BOX_PICKING;
      }
      if (Kbots[i].state == BOX_PICKED)
      {
        request_robot_position(EMITTER, Kbots[i].ROBOT_ID);
        Kbots[i].state = POSITION_REQUESTED;
        if (!are_all_slots_filled(&Kbots[i]))
        {

          Kbots[i].next_state = GOING_TO_GRID;
        }
        else
        {
          printf("all slots are filled\n");
          Kbots[i].next_state = GOING_TO_TOWER;
        }
      }
      if (Kbots[i].state == GONE_TO_GRID)
      {
        send_do_animation_command(EMITTER, Kbots[i].ROBOT_ID, PLACE_BOX_IN_GRID);
        Kbots[i].state = PLACE_IN_GRID;
      }
      if (Kbots[i].state == TOWER_REACHED)
      {
        place_box_on_tower_advanced(EMITTER, &Kbots[i], find_tower_height(rfimage, rf_width));
        Kbots[i].state = PLACING_BOX_ON_TOWER;
      }

      if (Kbots[i].state == GOING_TO_GRID || Kbots[i].state == GOING_TO_TOWER)
      {

        if (!do_i_have_the_box(i, Kbots[i].box_id, camera, Kbots[i].position))
        {
          if (Kbots[i].state == GOING_TO_GRID)
          {
            Kbots[i].f_slot.tail--;
            printf("slot tail:%d\n", Kbots[i].f_slot.tail);
          }
          
          Kbots[i].state = INITIAL;

          printf("ooooooops dropped it\n");
        }
      }
      request_robot_position(EMITTER, Kbots[i].ROBOT_ID);
      if (has_collision(Kbots[0], Kbots[1]) == 1 && Kbots[0].pos_know == 1 && Kbots[i].pos_know == 1)
      {
        //printf("avoiding\n");
        if (am_i_going(Kbots[0]) && am_i_going(Kbots[1]))
        {
          printf("both\n");
          set_avoidance_state(&Kbots[0]);
          go_to_avoidance_point(Kbots[0], Kbots[1], EMITTER);
        }
        else if (am_i_going(Kbots[0]) && !am_i_going(Kbots[1]))
        {
          printf("not both 0\n");
          set_avoidance_state(&Kbots[0]);
          go_to_avoidance_point(Kbots[0], Kbots[1], EMITTER);
        }
        else
        {
          printf("not both 1\n");
          set_avoidance_state(&Kbots[1]);
          go_to_avoidance_point(Kbots[1], Kbots[0], EMITTER);
        }
      }
      else if(has_collision(Kbots[0],Kbots[1]) == 3){
          set_avoidance_state(&Kbots[0]);
          go_to_avoidance_point(Kbots[0], Kbots[1], EMITTER);
          set_avoidance_state(&Kbots[1]);
          go_to_avoidance_point(Kbots[1], Kbots[0], EMITTER);
      }
      else if (has_collision(Kbots[0], Kbots[1]) == 2)
      {
        if (Kbots[0].state == AVOIDANCE)
        {
          printf("resuming\n");
          resume_path(&Kbots[0]);
        }
        if (Kbots[1].state == AVOIDANCE)
        {
          printf("resuming\n");
          resume_path(&Kbots[1]);
        }
      }
    }
    //////////////////////////////////////////RECEIVE///////////////////////////////////////////
    while (wb_receiver_get_queue_length(RECIEVER) > 0)
    {
      Command command;
      int message_recieved = message_handler(RECIEVER, SELF_ID, &command);
      char index = command.caller_id - 1;
      if (message_recieved && command.caller_id > 0 && command.id == 0)
      {
        if (command.type == SEND_POSITION)
        {
          if (Kbots[index].state == POSITION_REQUESTED)
          {
            Kbots[index].state = POSITION_KNOWN;
            double *position = (double *)command.data;
            printf("KbotB has replyed with position %g,%g,%g,%g\n", position[0], position[1], position[2], position[3]);
            Kbots[index].position[0] = position[0];
            Kbots[index].position[1] = position[2];
            Kbots[index].pos_know = 1;
          }
          else
          {
            //printf("ipdating position\n");
            double *position = (double *)command.data;
            //printf("KbotB has replyed with position %g,%g,%g,%g\n", position[0], position[1], position[2], position[3]);
            Kbots[index].position[0] = position[0];
            Kbots[index].position[1] = position[2];
            Kbots[index].orientation = position[3];
            Kbots[index].pos_know = 1;
          }
        }
        if (command.type == REACHED)
        {
          printf("KbotB has reached its destination\n");
          switch (Kbots[index].state)
          {
          case GOING_TO_BOX:
            Kbots[index].state = BOX_REACHED;
            break;
          case GOING_TO_GRID:
            printf("setting gone to grid\n");
            Kbots[index].state = GONE_TO_GRID;
            break;
          case GOING_TO_TOWER:
            printf("tower reahed\n");
            Kbots[index].state = TOWER_REACHED;
            break;
          default:
            printf("gone_somwehre\n");
          }
        }
        if (command.type == DONE_ANIMATION)
        {
          switch (Kbots[index].state)
          {
          case BOX_PICKING:
            Kbots[index].state = BOX_PICKED;
            printf("KbotB has picked up the box\n");
            break;
          case PLACE_IN_GRID:
            Kbots[index].state = PLACED_IN_GRID;
            printf("KbotB has placed the box in th grid\n");
            break;
          }
        }
        delete_command(&command);
      }
    }
  }
  wb_robot_cleanup();
  return 0;
}
