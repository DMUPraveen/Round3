#include <webots/robot.h>
#include <webots/nodes.h>
#include <webots/supervisor.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/compass.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TIME_STEP 16

static WbDeviceTag gps;
static WbDeviceTag compass;
static WbDeviceTag motors[4];

static const double max_speed =8;
static double target_position[3];

static void setup_motors(){
gps = wb_robot_get_device("gps");
compass = wb_robot_get_device("compass");
wb_gps_enable(gps,TIME_STEP);
wb_compass_enable(compass,TIME_STEP);

for(int i=0;i<4;i++){
char name[7];
sprintf(name,"wheel%d",i+1);
motors[i] = wb_robot_get_device(name); 

}


}

static void set_motor_speed(double speed[4]){
for(int i =0;i<4;i++){
wb_motor_set_position(motors[i],INFINITY);
wb_motor_set_velocity(motors[i],speed[i]);
//printf("speed:%g\n",speed[i]);
}
}

static int get_direction(double a,double b){
  int dir = sin(a - b)<=0?-1:1;
  return dir;
}
void set_target(double position[3]){
    setup_motors();
    for(int i=0;i<3;i++){
        target_position[i] = position[i];
        printf("%g\n",position[i]);
    }
}

int go_to_position(){
const double* temp =wb_gps_get_values(gps);
const double* north = wb_compass_get_values(compass);
double alpha = target_position[2];
double angle = -atan2(north[1],north[0]);
//angle = M_PI_2 - angle;
//printf("angle:%g\n",angle);

double direction = get_direction(angle,alpha);

double current_position[2];
current_position[0] = temp[0];
current_position[1] = temp[2];
double box_pos[2] = {target_position[0] ,target_position[1]};
double temp1[2];
double U_velocity[2];
double omega;
if(fabs(alpha -angle) >0.001){
omega = direction*8;
}
else{
omega = 0;
}
//printf("alpha:%g,angle:%g,omega%g,direction:%g\n",alpha,angle,omega,direction);
double wheel_speed[4] ={0,0,0,0};
const double r = 0.051;
const double a = 0.316;
const double b = 0.456;
for(int i =0;i<2;i++){
  temp1[i] = box_pos[i] - current_position[i];
}
double mod_temp1 = sqrt(temp1[0]*temp1[0] + temp1[1]*temp1[1]);

U_velocity[0] = ((temp1[0]/mod_temp1)*cos(-angle) + (temp1[1]/mod_temp1)*sin(-angle));
U_velocity[1] = ((temp1[1]/mod_temp1)*cos(-angle) - (temp1[0]/mod_temp1)*sin(-angle));

if(fabs(mod_temp1) > 0.001){
wheel_speed[0] = (1/r)*(U_velocity[0]-U_velocity[1] - (a+b)*omega);
wheel_speed[1] = (1/r)*(U_velocity[0]+U_velocity[1] + (a+b)*omega);
wheel_speed[2] = (1/r)*(U_velocity[0]+U_velocity[1] - (a+b)*omega);
wheel_speed[3] = (1/r)*(U_velocity[0]-U_velocity[1] + (a+b)*omega);
double magnitude = sqrt(wheel_speed[0]*wheel_speed[0]+wheel_speed[1]*wheel_speed[1]+wheel_speed[2]*wheel_speed[2]+wheel_speed[3]*wheel_speed[3]);
for(int i=0;i<4;i++){
  wheel_speed[i] = (max_speed)*(wheel_speed[i]/magnitude);
}
}
set_motor_speed(wheel_speed);
if(fabs(mod_temp1) < 0.001){
return 1;
}
else{
  return 0;
}




}