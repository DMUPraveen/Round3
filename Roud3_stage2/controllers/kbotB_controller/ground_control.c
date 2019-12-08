#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/gps.h>
#include <webots/compass.h>

#include <base.h>
#include "comunication.h"
#include "ground_control.h"

double get_bearing_in_radinas(WbDeviceTag compass)
{
    const double *north = wb_compass_get_values(compass);
    double rad = atan2(north[0], north[1]);

    return rad;
}
void send_reached_message(WbDeviceTag Emitter, char SELF_ID)
{
    Command command;
    command.type = REACHED;
    command.id = 0;
    command.data_length = 1;
    command.caller_id = SELF_ID;
    char data = 0;
    char *data_holder = &data;
    command.data = data_holder;
    int byte_stream_length = get_byte_stream_length(command.data_length);
    char byte_stream[byte_stream_length];
    construct_message(&command, byte_stream, byte_stream_length);
    wb_emitter_send(Emitter, byte_stream, byte_stream_length);
}
void position_send_constructor(double *positions, double *bearing, char robot_id, char *data_holder, Command *my_position, char SELF_ID)
{
    my_position->id = robot_id;
    my_position->data_length = sizeof(double) * 4;
    my_position->type = SEND_POSITION;
    my_position->caller_id = SELF_ID;
    char *cpositions = (char *)positions;
    char *cbearing = (char *)bearing;
    for (int i = 0; i < sizeof(double) * 3; i++)
    {
        data_holder[i] = cpositions[i];
    }
    for (int i = sizeof(double) * 3; i < sizeof(double) * 4; i++)
    {
        data_holder[i] = cbearing[i - sizeof(double) * 3];
    }
    my_position->data = data_holder;
}

void send_my_position_to_kinect(WbDeviceTag Emmiter, WbDeviceTag Gps, WbDeviceTag Compass, char SELF_ID)
{
    Command my_position;
    double *positions = wb_gps_get_values(Gps);
    //double positions[3] = {1.1,2.1,3.1};
    double bearing = get_bearing_in_radinas(Compass);
    //double bearing = 4.1;
    double *bearing_ptr = &bearing;
    char data_holder[sizeof(double) * 4];
    position_send_constructor(positions, bearing_ptr, 0, data_holder, &my_position, SELF_ID);
    int byte_stream_length = get_byte_stream_length(my_position.data_length);
    char byte_stream[byte_stream_length];
    construct_message(&my_position, byte_stream, byte_stream_length);
    wb_emitter_send(Emmiter, byte_stream, byte_stream_length);
}

void send_done_animation_to_kinect(WbDeviceTag Emitter, char SELF_ID, char animation)
{
    Command done_animation;
    done_animation.caller_id = SELF_ID;
    done_animation.id = 0;
    done_animation.type = DONE_ANIMATION;
    done_animation.data_length = 0;
    done_animation.data = &animation;
    send_message(&done_animation, Emitter);
}