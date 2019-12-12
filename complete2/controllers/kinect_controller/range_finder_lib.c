#include <webots/robot.h>
#include <webots/display.h>
#include <webots/range_finder.h>
#include "range_finder_lib.h"

#include <stdio.h>
#include <math.h>


void filter_range_image(int *filter_array, WbDeviceTag rangefinder, int width, int height, float filter_depth)
{
    const float *rimage = wb_range_finder_get_range_image(rangefinder);
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            filter_array[y * height + x] = (wb_range_finder_image_get_depth(rimage, width, x, y) < filter_depth) ? 1 : 0;
            if (y > height * 5 / 6)
            {
                filter_array[y * height + x] = 0;
            }
        }
    }
}

void display_show_range(int *filter_array, int width, int height, WbDeviceTag display)
{
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (filter_array[y * height + x] == 1)
            {
                wb_display_set_color(display, 0xffffff);
            }
            else
            {
                wb_display_set_color(display, 0x000000);
            }
            wb_display_draw_pixel(display, x, y);
        }
    }
}

float get_range_finder_image(WbDeviceTag rangefinder, int width, int height)
{
    /*this function gets the average of all the distance read by the range_finder*/
    const float *rimage = wb_range_finder_get_range_image(rangefinder);
    float distance = 0;
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            float d_pixel = wb_range_finder_image_get_depth(rimage, width, x, y);
            distance += d_pixel;
        }
    }
    float av_distance = distance / (height * width);
    return av_distance;
}