#include <stdio.h>
#include <math.h>
#include <webots/range_finder.h>
#include <webots/camera.h>
#include "navigation.h"

#define PIXEL_WIDTH 250
#define PIXEL_HEIGHT 250
#define FLOOR_WIDTH 10
#define FLOOR_HEIGHT 10

const double optimal_distance = 0.45807873;
const int tower_position_x = 152;
const int tower_position_y = 125;
const double distance_to_floor = 2.895;

void image_postion_to_cordinate(int image_pos[2], double cordinate_pos[2])
{
    cordinate_pos[0] = (double)(image_pos[0] - 125) / 25;
    cordinate_pos[1] = (double)(image_pos[1] - 125) / 25;
    printf("%d,%d,%g,%g\n", image_pos[0], image_pos[1], cordinate_pos[0], cordinate_pos[1]);
}

int box_in_grid(double x,double y){
    if((y<0.25 && y> -0.25) && (x <2.5 && x>-0.25)){
        return 1;
    }
    else{
        return 0;
    }
    return 0;
}

/*

int get_nearest_box_position(COLOUR color, BOX *box_array, double current_position[2], BOX *closest_box)
{
    int current_box_index = 0;
    double shortest_distance = INFINITY;
    while (box_array[current_box_index].initialized)
    {
        double box_position[2] = {box_array[current_box_index].x, box_array[current_box_index].y};
        double distance = get_distance(current_position, box_position);
        if (distance < shortest_distance)
        {
            closest_box->x = box_array[current_box_index].x;
            closest_box->y = box_array[current_box_index].y;
            closest_box ->box_color = box_array[current_box_index].box_color;
            closest_box ->id = box_array[current_box_index].id;
            closest_box ->initialized = box_array[current_box_index].initialized;



            shortest_distance = distance;
        }
        current_box_index++;
    }
    return closest_box ->id;
}
*/
/*
double get_distance(double *position1, double *position2)
{
    double distance = sqrt((position1[0] - position2[0]) * (position1[0] - position2[0]) + (position1[1] - position2[1]) * (position1[1] - position2[1]));
    return distance;
}
*/

/*
void blob_to_box(COLOUR color, Blob *sblob, BOX *sbox, int id)
{
    int blob_position[2] = {sblob->corner_x, sblob->corner_y};
    double box_position[2];
    image_postion_to_cordinate(blob_position, box_position);
    sbox->initialized = 1;
    sbox->id = id;
    sbox->box_color = color;
    sbox->x = box_position[0];
    sbox->y = box_position[1];
    printf("id:%d, x:%g,y:%g \n",sbox->id,sbox->x,sbox->y);
}
*/
/*
void blob_array_to_box_array(Blob *blob_array, BOX *box_array, COLOUR color, int box_count)
{
    for (int i = 0; i < box_count; i++)
    {
        if (blob_array[i].initialized)
        {
            blob_to_box(color,&blob_array[i],&box_array[i],i);
        }
        else
        {
            break;
        }
    }
}
*/
double find_tower_height(const float *rfimage, int rf_width)
{
    double tower_height = distance_to_floor - wb_range_finder_image_get_depth(rfimage, rf_width, tower_position_x, tower_position_y);
    return tower_height;
}

int get_box_colour(WbCameraRecognitionObject object)
{
    double red = object.colors[0];
    double green = object.colors[1];
    double blue = object.colors[2];
    if (red == 1 && blue == 0 && green == 0)
    {
        return 0; //object is red
    }
    if (blue == 1 && green == 0 && red == 0)
    {
        return 1; //object is blue
    }
    else {
        return 3; //not interested
    }
    return 3;
}

void push_to_box_array(BOX box,Box_array* array){
    array->tail++;
    //printf("tail:%d\n",array->tail);
    array->Barray[array->tail] = box; 
}

double relative_to_absolute_x(double x){
    return x + 0.01;
}
double relative_to_absolute_y(double y){
    return -y - 0.05;
}

double relative_to_absolute_z(double z){
    return z +2.88;
}

void get_four_possible_positions(double x,double z,double positions[4][2]){
    positions[0][0] = x - optimal_distance;
    positions[0][1] = z;

    positions[1][0] = x + optimal_distance;
    positions[1][1] = z;

    positions[2][0] = x;
    positions[2][1] = z + optimal_distance;

    positions[3][0] = x;
    positions[3][1] = z - optimal_distance;
}

double get_distance_between_points(double position1[2],double position2[2]){
    double X2 = (position1[0]-position2[0])*(position1[0]-position2[0]);
    double Y2 = (position1[1] - position2[1])*(position1[1] - position2[1]);
    double distance = sqrt(X2 + Y2);
    return distance;
}


void fill_and_sort_boxes(Box_array* Red, Box_array* Blue, WbDeviceTag Camera)
{
    int no_of_objects = wb_camera_recognition_get_number_of_objects(Camera);
    const WbCameraRecognitionObject *recognized_boxes = wb_camera_recognition_get_objects(Camera);
    for (int i = 0; i < no_of_objects; i++)
    {   
        BOX box;
        box.box_color = get_box_colour(recognized_boxes[i]);
        box.id = recognized_boxes[i].id;
        box.x = relative_to_absolute_x(recognized_boxes[i].position[0]);
        box.y = relative_to_absolute_y(recognized_boxes[i].position[1]);
        box.height = relative_to_absolute_z(recognized_boxes[i].position[2]);
        //printf("sorter x:%g,y%g\n",box.x,box.y);
        box.orientation = 0;
        box.status = NOT_PICKED;

        if(box_in_grid(box.x,box.y) == 0){
        if(box.box_color == 0){
            //if the boxes are red
            //printf("pushing to red\n");
            push_to_box_array(box,Red);
        }
        if(box.box_color == 1){
            //if the boxes are blue
            push_to_box_array(box,Blue);

        }
        }
        
    }
}
double get_distance_to_box(BOX box,double position[2]){
    double X2 = (box.x-position[0])*(box.x-position[0]);
    double Y2 = (box.y - position[1])*(box.y - position[1]);
    double distance = sqrt(X2 + Y2);
    return distance;
}
void get_nearest_box(BOX* nearest_box,Box_array array,double position[2]){
    double shortest_distance = INFINITY;
    //printf("tail:%d\n",array.tail);
    for(int i=0; i<=array.tail; i++){
         if(shortest_distance > get_distance_to_box(array.Barray[i],position)){
             shortest_distance = get_distance_to_box(array.Barray[i],position);
             nearest_box->box_color = array.Barray[i].box_color;
             nearest_box ->id = array.Barray[i].id;
             nearest_box ->orientation = array.Barray[i].orientation;
             nearest_box ->status = array.Barray[i].status;
             nearest_box->x = array.Barray[i].x;
             nearest_box->y = array.Barray[i].y;

         }
    }
    //return nearest_box->id;
    //printf("nearest_box x:%g,y%g,id%d\n",nearest_box->x,nearest_box->y,nearest_box->id);  
}

int get_optimal_position(double possition_array[][2],int no_of_options,double my_position[2]){
    double shortest_distance = INFINITY;
    int best_option = 0;
    for(int i=0; i< no_of_options; i++){
        if(shortest_distance > get_distance_between_points(possition_array[i],my_position)){
            shortest_distance = get_distance_between_points(possition_array[i],my_position);
            best_option = i;
        }
    }
    return best_option;
}

int get_destination(double my_position[2],Box_array array,double target_position[3]){
    BOX nearest_box;
    double angle_array[4] = {0,M_PI,M_PI_2,-M_PI_2};
    double possible_targets[4][2];
    //printf("tail get destinaiton:%d",array.tail);
    get_nearest_box(&nearest_box,array,my_position);
    get_four_possible_positions(nearest_box.x,nearest_box.y,possible_targets);
    /*
    for(int i=0; i<4;i++){
        printf("possible_position:%d, x:%g,y:%g\n",i,possible_targets[i][0],possible_targets[i][1]);
    }
    */
    int best = get_optimal_position(possible_targets,4,my_position);
    for(int i=0; i<2;i++){
    target_position[i] = possible_targets[best][i];
    }
    target_position[2] = angle_array[best];
    //printf("destination x:%g,y:%g,alpha:%g\n",target_position[0],target_position[1],target_position[2]);
    return nearest_box.id;


}

int do_i_have_the_box(int col,int box_id,WbDeviceTag camera,double position[2]){
    Box_array arrays[2];
    arrays[0].tail = -1;
    arrays[1].tail = -1;
    fill_and_sort_boxes(&arrays[0],&arrays[1],camera);
    
    for(int i=0; i<=arrays[col].tail; i++){
        if(arrays[col].Barray[i].id == box_id){
            
             double x = arrays[col].Barray[i].x;
             double y = arrays[col].Barray[i].y;
             double z = arrays[col].Barray[i].height;
             /*
             if(fabs(x - position[0])>1 || fabs(y - position[1]) >1){
                 return 0;
             }
             */
            if(z < 0.3){
                return 0;
            }
        }

    }
    
    return 1;

}