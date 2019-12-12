#include "image_recognition.h"
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/display.h>
#include <stdio.h>

void set_pixel(WbDeviceTag Display, int color[3], int x, int y)
{
    int colour = ((color[0]<<16)+(color[1] <<8)+(color[2]));

    //printf("%x\n",((color[0])+(color[1] <<8)+(color[2]<<16)));

    wb_display_set_color(Display, colour);
    wb_display_draw_pixel(Display, x, y);
}

void draw_display(WbDeviceTag Display, int image[][3], int width, int height)
{ //this draws an image on the display image is given as 2d array and the rgb valus of a certain position can be accesed by accesing image[height*y+x]
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            set_pixel(Display, image[height * y + x], x, y);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_image(WbDeviceTag Camera, int width, int height, int image_array[][3])
{ //takes the image pointer given by the camera and stores thte image in the image_array(image_array[width*height][3])
    const unsigned char *image = wb_camera_get_image(Camera);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {

            int R = wb_camera_image_get_red(image, width, x, y);
            int G = wb_camera_image_get_green(image, width, x, y);
            int B = wb_camera_image_get_blue(image, width, x, y);
            image_array[height * y + x][0] = R;
            image_array[height * y + x][1] = G;
            image_array[height * y + x][2] = B;
        }
    }
}

int isnear(Blob *blob, int boundry, int x, int y)
{ //this codes check whether a certain pixel is near a given blob(given as a pointer), and returns a 1 if so else returns a 0
    //this is used as a helper finction for the find_blob function
    int cx = blob->corner_x;
    int cy = blob->corner_y;
    int w = blob->width;
    int h = blob->height;
    int d = boundry;
    if ((x > cx - d && x < cx + w + d) && (y > cy - d && y < cy + h + d))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
void filter_image_G(int image_array[][3], int filter_array[], int width,int height)
{   // this takes an image array as an argument and filter it and find the(x,y)values where green pixels are detected and writes ones(1s) to height*y +x if (x,y) is green else writes a zero
    /*This function is used to convert the image to a binary array which idiacates the prescence or abscence of pixels of the color of the boxes*/
    int status = 0;
    
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int R = image_array[height * y + x][0];
            int G = image_array[height * y + x][1];
            int B = image_array[height * y + x][2];
            

            if (B+R<10 && G>10)
            {
                filter_array[height * y + x] = 1;
            }
            else
            {
                filter_array[height * y + x] = 0;
            }
        }
    }
}
void filter_image_B(int image_array[][3], int filter_array[], int width,int height)
{   // this takes an image array as an argument and filter it and find the(x,y)values where green pixels are detected and writes ones(1s) to height*y +x if (x,y) is green else writes a zero
    /*This function is used to convert the image to a binary array which idiacates the prescence or abscence of pixels of the color of the boxes*/
    int status = 0;
    
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int R = image_array[height * y + x][0];
            int G = image_array[height * y + x][1];
            int B = image_array[height * y + x][2];
            

            if (G+R<10 && B>10)
            {
                filter_array[height * y + x] = 1;
            }
            else
            {
                filter_array[height * y + x] = 0;
            }
        }
    }
}
void filter_image_R(int image_array[][3], int filter_array[], int width,int height)
{   // this takes an image array as an argument and filter it and find the(x,y)values where green pixels are detected and writes ones(1s) to height*y +x if (x,y) is green else writes a zero
    /*This function is used to convert the image to a binary array which idiacates the prescence or abscence of pixels of the color of the boxes*/
    int status = 0;
    
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int R = image_array[height * y + x][0];
            int G = image_array[height * y + x][1];
            int B = image_array[height * y + x][2];
            

            if (G+B<10 && R>10)
            {
                filter_array[height * y + x] = 1;
            }
            else
            {
                filter_array[height * y + x] = 0;
            }
        }
    }
}
void filter_image(int image_array[][3], int filter_array[], int width, int height,COLOUR color){
    switch(color){
        case RED:
            filter_image_R(image_array,filter_array, width, height);
            break;
        case BLUE:
            filter_image_B(image_array,filter_array, width, height);
            break;
        case GREEN:
            filter_image_G(image_array,filter_array, width, height);
            break;

    }

}
void blob_update(Blob *blob, int x, int y)
{ //this updates the dimensions of a blob so that it include the given pixel(given as x,y)
    //this is also used as a helper function for the find_blobs function
    int cx = blob->corner_x;
    int cy = blob->corner_y;
    int w = blob->width;
    int h = blob->height;
    if (x < cx)
    {
        blob->corner_x = x;
    }
    if (y < cy)
    {
        blob->corner_y = y;
    }
    if (x > cx + w)
    {
        blob->width = (x - cx);
    }
    if (x > cx + h)
    {
        blob->height = (y - cy);
    }
}

void findblobs(Blob blob_array[], int filter_array[], int width, int height, int boundry, int max_blob_count)
{ //this codes find blobs and store them in a blob array
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int pixel = filter_array[height * y + x];
            if (pixel)
            {

                int last = 0;
                int new = 0;
                for (int i = 0; i < max_blob_count; i++)
                {

                    if (blob_array[i].initialized == 0)
                    {
                        last = i;
                        break;
                    }
                    int status = isnear(&blob_array[i], boundry, x, y);
                    if (status)
                    {
                        blob_update(&blob_array[i], x, y);
                        new += status;
                    }
                }
                if (new == 0)
                {
                    blob_array[last].initialized = 1;
                    blob_array[last].corner_x = x;
                    blob_array[last].corner_y = y;
                    blob_array[last].width = 0;
                    blob_array[last].height = 0;
                }
            }
        }
    }
}


/*The following function is no longer used it was used for debugging purposes
It allowed us to display the detected boxes on a display in white and draw a red box around them according to the
parameters found by the find_blobs function it also printed the height of the boxes near them
*/
void Draw_blobs(WbDeviceTag display, Blob *blob_array)
{
    for (int i = 0; i < 50; i++)
    {
        if (blob_array[i].initialized == 0)
        {
            break;
        }
        wb_display_set_color(display, 0xff0000);
        if (blob_array[i].width > 0 && blob_array[i].height > 0)
        {
            wb_display_draw_rectangle(display, blob_array[i].corner_x, blob_array[i].corner_y, blob_array[i].width, blob_array[i].height);

            char num[5];
            sprintf(num, "%d", blob_array[i].height);
            wb_display_draw_text(display, num, blob_array[i].corner_x + (blob_array[i].width / 2), blob_array[i].corner_y + (blob_array[i].height / 2));
        }
    }
}