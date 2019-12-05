#ifndef NAVIGATION
#define NAVIGATION
#include "image_recognition.h"
void image_postion_to_cordinate(int image_pos[2],double cordinate_pos[2]);
typedef struct box {
    double x;
    double y;
    int id;
    COLOUR box_color;
    int initialized;


}BOX;
double get_distance(double* position1, double* position2);
void get_nearest_box(COLOUR color, BOX *box_array, double current_position[2],BOX *closest_box);
void blob_to_box(COLOUR color, Blob *sblob, BOX *sbox, int id);
void blob_array_to_box_array(Blob *blob_array, BOX *box_array, COLOUR color, int box_count);
double find_tower_height(const float* rfimage,int rf_width);

#endif
