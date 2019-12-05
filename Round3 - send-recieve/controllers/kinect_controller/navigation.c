#include <stdio.h>
#include <math.h>
#include "navigation.h"

#define PIXEL_WIDTH 250
#define PIXEL_HEIGHT 250
#define FLOOR_WIDTH 10
#define FLOOR_HEIGHT 10

void image_postion_to_cordinate(int image_pos[2], double cordinate_pos[2])
{
    cordinate_pos[0] = (double)(image_pos[0] - 125)/25;
    cordinate_pos[1] = (double)(image_pos[1] - 125)/25;
    printf("%d,%d,%g,%g\n",image_pos[0],image_pos[1],cordinate_pos[0],cordinate_pos[1]);
}

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

double get_distance(double *position1, double *position2)
{
    double distance = sqrt((position1[0] - position2[0]) * (position1[0] - position2[0]) + (position1[1] - position2[1]) * (position1[1] - position2[1]));
    return distance;
}

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