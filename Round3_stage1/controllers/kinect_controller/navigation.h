#ifndef NAVIGATION
#define NAVIGATION
#include "image_recognition.h"
typedef enum box_status{
    NOT_PICKED,
    PICKED,
    PLACED

}BOX_STATUS;
void image_postion_to_cordinate(int image_pos[2],double cordinate_pos[2]);
typedef struct box {
    double x;
    double y;
    double orientation;
    int id;
    COLOUR box_color;
    BOX_STATUS status;
    //int initialized;


}BOX;

typedef struct box_array{
    BOX Barray[10];
    int tail;

}Box_array;
//double get_distance(double* position1, double* position2);
//void get_nearest_box(COLOUR color, BOX *box_array, double current_position[2],BOX *closest_box);
//void blob_to_box(COLOUR color, Blob *sblob, BOX *sbox, int id);
//void blob_array_to_box_array(Blob *blob_array, BOX *box_array, COLOUR color, int box_count);
double find_tower_height(const float* rfimage,int rf_width);
void fill_and_sort_boxes(Box_array* Red, Box_array* Blue, WbDeviceTag Camera);
void get_destination(double my_position[2],Box_array array,double target_position[3]);
#endif
