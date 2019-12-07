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


int main(int argc, char **argv) {
  // initialization: robot
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
  int key;
  COLOUR blob_color =RED;
  FILE *file;
  int kbotB_status =0;
  
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
  for(i = 0; i<no_boxes; i++){
    char kb[6]={"KKB"};
    sprintf(tmp, "%d", i); 
    strcpy(Obs_names[i], strcat(kb, tmp));   
    for(j = 0; j<3; j++){ 
      values[i][j]= (rand()%95-47)/10.0;      
      //values[i][j]= (rand()%81-35)/10;
      values[i][1]= 0.04;
      //fprintf(stderr, "VALUE, %d\n",values[i][1] );
    }
    if (fabs(values[i][0])<1.5 && fabs(values[i][2])<1.5){
      if (values[i][0]<0){
        values[i][0]= values[i][0]-1.5;
      }
      else{
        values[i][0]= values[i][0]+1.5;
      }
    }
  }
  /* setting KKBx translations to above random values */
  for (i = 0; i < no_boxes ; i++) {
    KKboxes[i] = wb_supervisor_node_get_from_def(Obs_names[i]);
    WbFieldRef tr = wb_supervisor_node_get_field(KKboxes[i], "translation");
    wb_supervisor_field_set_sf_vec3f(tr, values[i]);
  }
  
  // random cylindrical tower platform height
  WbNodeRef TowerSD, TowerSH;
  float rvalue;
  double TWtl[1][3];
  rvalue = (rand()%36 +20)/100.0;
  TWtl[0][0]=1;
  TWtl[0][1]=0.001+rvalue/2;
  TWtl[0][2]=0;
  TowerSD = wb_supervisor_node_get_from_def("TOWER");  
  WbFieldRef hy = wb_supervisor_node_get_field(TowerSD, "translation");
  TowerSH = wb_supervisor_node_get_from_def("TOWER_cy");  
  WbFieldRef ht = wb_supervisor_node_get_field(TowerSH, "height");
  wb_supervisor_field_set_sf_vec3f(hy, TWtl[0]);
  wb_supervisor_field_set_sf_float(ht, rvalue);  
  
  int num;
  num = 0;
  int camera_image[height*width][3];
  int filter_array[height*width];
  int r_filter_array[rf_height*rf_width];
  wb_receiver_enable(RECIEVER,TIME_STEP);
  // Main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // collection of range-finder and camera images
    const float *rfimage = wb_range_finder_get_range_image(range_finder);
    const unsigned char *image = wb_camera_get_image(camera);
    Blob blob_array[50] = BLOB_ARRAY;
    BOX BOX_ARRAY[50] = BLOB_ARRAY;
    get_image(camera,width,height,camera_image);
    //draw_display(display,camera_image,width,height);
    filter_image(camera_image,filter_array,width,height,blob_color);
    filter_range_image(r_filter_array,range_finder,rf_width,rf_height,2.88);
    //display_show_range(r_filter_array,rf_width,rf_height,display);
    //printf("%f\n",get_range_finder_image(range_finder,rf_width,rf_height));
    //float tower_height = 2.895 - wb_range_finder_image_get_depth(rfimage,rf_width,152,125);
    //printf("cal_height:%f, real_height:%f\n",find_tower_height(rfimage,rf_width),rvalue);
    //draw_display(display,&camera_image);
    //findblobs(blob_array,filter_array,width,height,5,50);
    
    //Draw_blobs(display,blob_array);
    
  /*
    key = wb_keyboard_get_key();
    //printf("%d\n",key);
    if(key == 65){
      //if letter a is pressed send in the message hello
      Command scommand;
      scommand.id = 0;
      scommand.type = 3;
      char* data_holder = "hello";
      scommand.data = data_holder;
      scommand.data_length = strlen(data_holder)+1;
      const int byte_stream_length = get_byte_stream_length(scommand.data_length); 
      char byte_stream[byte_stream_length];
      
      construct_message(&scommand,byte_stream,byte_stream_length);
      int status = wb_emitter_send(EMITTER,byte_stream,byte_stream_length);
      blob_array_to_box_array(blob_array,BOX_ARRAY,RED,50);

    }
    if(key ==66){
      printf("sending a position\n");
      double position[3] = {1.5,3.14,2.7};
      Command goto_command;
      construct_goto_command(0,position,&goto_command);
      const int byte_stream_length = get_byte_stream_length(goto_command.data_length);
      char byte_stream[byte_stream_length];
      construct_message(&goto_command,byte_stream,byte_stream_length);
      wb_emitter_send(EMITTER,byte_stream,byte_stream_length);
      //blob_array_to_box_array(blob_array,BOX_ARRAY,RED,50);

    }
  
    if (wb_receiver_get_queue_length(RECIEVER) > 0)
    {
      printf("recieved\n");
      const char *message = wb_receiver_get_data(RECIEVER);
      int message_length = wb_receiver_get_data_size(RECIEVER);
      
      Command scommand;
      char data_holder[get_data_length(message_length)];
      deconstruct_message(message,message_length,&scommand,data_holder,get_data_length(message_length));
      //double* positions = (double*)scommand.data;
      printf("%d,%d,%s,%d\n",scommand.id,scommand.type,scommand.data,scommand.data_length);
      wb_receiver_next_packet(RECIEVER);
    }
    
  */
    if(kbotB_status ==0){
      kbotB_status =1;
      double position[3] = {4,4,-M_PI_4};
      send_goto_command(EMITTER,KBOTB_ID,position);
      printf("sending goto message to KbotB\n");

    };
    Command command;
    int message_recieved = message_handler(RECIEVER,SELF_ID,&command);
    //printf("%d\n",message_recieved);
    if(message_recieved){
      //command.data[3] =65;
      //printf("%d,%d,%s,%d\n",command.id,command.type,command.data,command.data_length);
      if(command.type == REACHED){
        //printf("boom%d\n",kbotB_status);
        kbotB_status =2;
        printf("KbotB has reached its destination\n");
        printf("%d\n",command.caller_id);
      }
      delete_command(&command);
      //free(command.data);
    }

  /*  
   switch(key){
     case 82:
       blob_color = RED;
       printf("red\n");
       break;
      case 71:
        blob_color = GREEN;
        printf("green\n");
        break;
      case 66:
        printf("blue\n");
        blob_color = BLUE;
        break;
   }
  */  
    /*
    // put the depth array (rfimage) in a file
    file = fopen("depth_array.txt", "w");
    if (!file) {
      fprintf(stderr, "Cannot open \"depth_array.txt\"");
      exit(EXIT_FAILURE);
    }
    for (j = 0; j < rf_height; ++j) {
      for (i = 0; i < rf_width; ++i) {
        float depth = wb_range_finder_image_get_depth(rfimage, rf_width, i, j);
        fprintf(file, "%d, %d, %f\n", i, j, depth);
      }
    }
    fclose(file);*/
       
  }
  wb_robot_cleanup();
  return 0;
}
