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
#include "advanced_navigation.h"
//#include "viranga&samith.h"

#define TIME_STEP 16
static int current_path=-1;
const static int bitmask = 0b111;
const double safe_position[4][2] = {
                                {0,2},
                                {-2,0},
                                {0,-2},
                                {2,0}

                            };

static WbDeviceTag gps;
static WbDeviceTag compass;
static WbDeviceTag motors[4];

static const double max_speed =14.8;
static double target_position[3];

static int going_status = 1;
static int current_safe_point = 0;

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
        //printf("%g\n",position[i]);
    }
}

double step_speed_angle(double delta){
  //printf("delta:%g\n",delta);
  /*
  
  if(fabs(delta)< 0.01){
    return 1;
  }
  else if(fabs(delta) < 0.1){
    //printf("reducing speed\n");
    return 2;
    
  }
  else{
    return 8;
  }
  */
 return 8;

}
double step_speed_linear(double delta){
  //printf("delta:%g\n",delta);
  
  if(fabs(delta) < 0.1){
    //printf("reducing speed\n");
    return max_speed/4;
    
  }
  /*
  
  else if(fabs(delta<0.1)){
    return max_speed/2;
  }
  
  else{
    return max_speed;
  }
  */
 return max_speed;

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
omega = direction*step_speed_angle(alpha-angle);
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
  wheel_speed[i] = step_speed_linear(mod_temp1)*(wheel_speed[i]/magnitude);
  
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

int get_next_safe_point(int path){
  if(current_path == -1){
    current_path = path;
  }

  printf("curretn path:%o\n",current_path);
  int destination = current_path & bitmask;
  current_path = current_path >>3;
  printf("destination:%d\n",destination);
  return destination;

}

int higher_level_go_to(int path,double position[3]){
  
  if(going_status == 1){
   current_safe_point = get_next_safe_point(path);
  }
  if(current_safe_point != 0){
  double sposition[3];
  sposition[0] = safe_position[current_safe_point-1][0];
  sposition[1] = safe_position[current_safe_point-1][1];
  sposition[2] = position[2];
  set_target(sposition);
  }
  else{
    set_target(position);
  }
  going_status = go_to_position();
  if(going_status ==1 && current_safe_point ==0){
    return 1;
  }
  return 0;

}




void initialize_naviagtion(){
  current_path=-1;
  going_status = 1;
  current_safe_point = 0;
}