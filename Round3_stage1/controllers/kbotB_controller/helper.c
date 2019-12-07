#include <webots/robot.h>

#include <stdio.h>
#include <math.h>
#include <base.h>
#include <gripper.h>
#include <arm.h>
#include "helper.h"

void set_arm_position(double x,double y,double z){
  
 
  double arm3=arm_get_sub_arm_length(ARM4) + arm_get_sub_arm_length(ARM5);
  double arm1=arm_get_sub_arm_length(ARM2);
  double arm2=arm_get_sub_arm_length(ARM3); 
  double x2=sqrt(x*x+z*z); 
  double alpha=atan(y/x2)-M_PI_2; 
  
 
 if(atan(y/x2)<(M_PI_2-1.13446)){
 alpha=-1.13446;
 
 }
 
 else{
 
 alpha=atan(y/x2)-M_PI_2;
 }
 
  if(x2==0){
  alpha=0;
  }
  
  
  double x1=x2-arm1*sin(-alpha);
  double y1=y-arm1*cos(-alpha);
  double k = sqrt(x1 * x1 + y1 * y1);
  double phi=atan(y1/x1); 
  double value=(arm2*arm2+arm3*arm3-(k*k))/(2*arm2*arm3);
  //printf("value:%g",value);
  double gamma=acos(value)-M_PI;
  double beta=phi-alpha-M_PI_2+asin((-arm3*sin(gamma))/k);
  double psi=atan(z/x);
  
  if (x<0 && z>0){
  
  psi=M_PI + atan(z/x);
  }
  
  else if(x<0 && z<0){
  psi=atan(z/x)-M_PI;
  
  }
  
  arm_set_sub_arm_rotation(ARM1,psi);
  arm_set_sub_arm_rotation(ARM2,alpha);
  arm_set_sub_arm_rotation(ARM3,beta);
  arm_set_sub_arm_rotation(ARM4,gamma);
}


int pick_up_box(int* pick_up_status,double* time){
    //printf("pick_up_status:%d\n",*pick_up_status);
    //printf("time:%g",*time);
    //printf("time_now:%g",wb_robot_get_time());
    if(*pick_up_status == 0){
    gripper_release();
    set_arm_position(0.259001,-0.2,0);
    *pick_up_status = 1;
    *time = wb_robot_get_time();
    }
    if(*pick_up_status == 1){
        if(wb_robot_get_time() - *time > 5 ){
            *pick_up_status = 2;
        }
    }
    if(*pick_up_status == 2){
        gripper_grip();
        *time = wb_robot_get_time();
        *pick_up_status = 3;

    }
    if(*pick_up_status == 3){
        if(wb_robot_get_time() - *time > 2 ){
            *pick_up_status = 4;
        }
    }
    if(*pick_up_status == 4){
        set_arm_position(0.3,0.3,0);
        *pick_up_status = 0;
        return 1;
    }
    return 0;

}