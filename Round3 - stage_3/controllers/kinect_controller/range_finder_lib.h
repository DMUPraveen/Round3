#ifndef RANGE
#define RANGE
#include <webots/robot.h>
#include <webots/display.h>
#include <webots/range_finder.h>

#include <stdio.h>
#include <math.h>

void display_show_range(int *filter_array, int width, int height, WbDeviceTag display);
void filter_range_image(int *filter_array, WbDeviceTag rangefinder, int width, int height, float filter_depth);
float get_range_finder_image(WbDeviceTag rangefinder, int width, int height);




#endif RANGE