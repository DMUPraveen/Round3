#include <webots/robot.h>

#include <stdio.h>
#include <math.h>
#include <base.h>
#include <gripper.h>
#include <arm.h>
#include "helper.h"
#include "advanced_navigation.h"
#include "viranga_samith.h"

int pickup_status = 0;
int place_status = 0;
double Time = 0;

void set_arm_position(double x, double y, double z)
{

    double arm3 = arm_get_sub_arm_length(ARM4) + arm_get_sub_arm_length(ARM5);
    double arm1 = arm_get_sub_arm_length(ARM2);
    double arm2 = arm_get_sub_arm_length(ARM3);
    double x2 = sqrt(x * x + z * z);
    double alpha = atan(y / x2) - M_PI_2;

    if (atan(y / x2) < (M_PI_2 - 1.13446))
    {
        alpha = -1.13446;
    }

    else
    {

        alpha = atan(y / x2) - M_PI_2;
    }

    if (x2 == 0)
    {
        alpha = 0;
    }

    double x1 = x2 - arm1 * sin(-alpha);
    double y1 = y - arm1 * cos(-alpha);
    double k = sqrt(x1 * x1 + y1 * y1);
    double phi = atan(y1 / x1);
    double value = (arm2 * arm2 + arm3 * arm3 - (k * k)) / (2 * arm2 * arm3);
    //printf("value:%g",value);
    double gamma = acos(value) - M_PI;
    double beta = phi - alpha - M_PI_2 + asin((-arm3 * sin(gamma)) / k);
    double psi = atan(z / x);

    if (x < 0 && z > 0)
    {

        psi = M_PI + atan(z / x);
    }

    else if (x < 0 && z < 0)
    {
        psi = atan(z / x) - M_PI;
    }

    arm_set_sub_arm_rotation(ARM1, psi);
    arm_set_sub_arm_rotation(ARM2, alpha);
    arm_set_sub_arm_rotation(ARM3, beta);
    arm_set_sub_arm_rotation(ARM4, gamma);
}

int pick_up_box()
{
    //printf("pick_up_status:%d\n",*pick_up_status);
    //printf("time:%g",*time);
    //printf("time_now:%g",wb_robot_get_time());
    if (pickup_status == 0)
    {
        gripper_release();
        set_arm_position(0.259001, -0.2, 0);
        pickup_status = 1;
        Time = wb_robot_get_time();
    }
    if (pickup_status == 1)
    {
        if (wb_robot_get_time() - Time > 5)
        {
            pickup_status = 2;
        }
    }
    if (pickup_status == 2)
    {
        gripper_grip();
        Time = wb_robot_get_time();
        pickup_status = 3;
    }
    if (pickup_status == 3)
    {
        if (wb_robot_get_time() - Time > 2)
        {
            pickup_status = 4;
        }
    }
    if (pickup_status == 4)
    {
        set_arm_position(0.3, 0.3, 0);
        pickup_status = 5;
        Time = wb_robot_get_time();
        //return 1;
    }
    if (pickup_status == 5)
    {
        if (wb_robot_get_time() - Time > 3)
        {
            pickup_status = 6;
        }
    }
    if (pickup_status == 6)
    {
        //set_arm_position(0.3,0.3,0);
        pickup_status = 0;
        //Time = wb_robot_get_time();
        return 1;
    }
    return 0;
}

int place_box()
{
    //printf("pick_up_status:%d\n",*pick_up_status);
    //printf("time:%g",*time);
    //printf("time_now:%g",wb_robot_get_time());
    if (place_status == 0)
    {
        set_arm_position(0.284000, -0.165000, 0);
        place_status = 1;
        Time = wb_robot_get_time();
    }
    if (place_status == 1)
    {
        if (wb_robot_get_time() - Time > 5)
        {
            place_status = 2;
        }
    }
    if (place_status == 2)
    {
        gripper_release();
        Time = wb_robot_get_time();
        place_status = 3;
    }
    if (place_status == 3)
    {
        if (wb_robot_get_time() - Time > 5)
        {
            place_status = 4;
        }
    }
    if (place_status == 4)
    {
        set_arm_position(0.3, 0.3, 0);
        place_status = 0;
        return 1;
    }
    return 0;
}

int do_animation(enum animation Anim)
{
    int done;
    switch (Anim)
    {
    case PICK_BOX:
        done = pick_up_box();
        break;
    case PLACE_BOX_IN_GRID:
        done = place_box();
        break;
    default:
        done = 0;
    }
    return done;
}


/*
void set_target_path_and_position(State_mach *machine, double position[3])
{
    initialize_naviagtion();
    machine->path = get_path(position);
    for (int i = 0; i < 3; i++)
    {
        machine->position[i] = position[i];
    }
}
*/
void set_target_path_and_position(State_mach* machine,double position[3],WbDeviceTag gps){
    initialize_naviagtion();
    double *my_pos = wb_gps_get_values(gps);
    machine ->path = get_path(my_pos[0],my_pos[2],position[0],position[1]);
    if(machine ->path == 3*8+1){
        machine ->path = 3*8*8+4*8+1;
    }
    if(machine ->path == 1*8+3){
        machine ->path = 1*8*8+4*8+3;
    }
    printf("hello\n");
    printf("the path:%o\n",machine ->path);
    for(int i=0;i<3;i++){
        machine->position[i] = position[i]; 
    }
}

int advanced_go_to(State_mach *machine)
{
    return higher_level_go_to(machine->path, machine->position);
}

int put_box_on_tower(double cyl_x, double cyl_y)
{
    printf("place_status:%d\n", place_status);
    if (place_status == 0)
    {
        printf("y:%g, x:%g\n", cyl_y, cyl_x);
        set_arm_position(cyl_x, cyl_y, 0);
        place_status = 1;
        Time = wb_robot_get_time();
    }
    if (place_status == 1)
    {
        if (wb_robot_get_time() - Time > 5)
        {
            place_status = 2;
        }
    }
    if (place_status == 2)
    {
        gripper_release();
        Time = wb_robot_get_time();
        place_status = 3;
    }
    if (place_status == 3)
    {
        if (wb_robot_get_time() - Time > 1)
        {
            place_status = 4;
        }
    }
    if (place_status == 4)
    {
        set_arm_position(0.3, 0.3, 0);
        place_status = 0;
        return 1;
    }
    return 0;
}

int do_tower_task_and_put_box(State_mach *machine)
{
    return put_box_on_tower(machine->tower_arm[0], machine->tower_arm[1]);
}