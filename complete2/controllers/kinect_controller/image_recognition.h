#ifndef IMAGE
#define IMAGE

#include <webots/robot.h>
#define BLOB_ARRAY {{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}}
typedef enum{
    RED,
    GREEN,
    BLUE
}COLOUR;
typedef struct blob
{ //this structs is used to store the parameters detected boxes
    int corner_x;
    int corner_y;
    int width;
    int height;
    int initialized;
}Blob;
void set_pixel(WbDeviceTag Display, int color[3], int x, int y);
void draw_display(WbDeviceTag Display, int image[][3], int width, int height);
void get_image(WbDeviceTag Camera, int width, int height, int image_array[][3]);
void filter_image(int image_array[][3], int filter_array[], int width, int height,COLOUR color);
int isnear(Blob *blob, int boundry, int x, int y);
void blob_update(Blob *blob, int x, int y);
void findblobs(Blob* blob_array, int filter_array[], int width, int height, int boundry,int max_blob_count);
void Draw_blobs(WbDeviceTag display,Blob* blob_array);


#endif