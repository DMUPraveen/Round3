

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TIME_STEP 16

/*void grid_avoidance(double x, double y, double x1, double y1)
{

  double xp1 = 1.5;
  double yp1 = 0;
  double xp2 = -1.5;
  double yp2 = 0;
  double xp3 = 0;
  double yp3 = 1.5;
  double xp4 = 0;
  double yp4 = -1.5;

  double distance1 = sqrt((x - xp1) * (x - xp1) + (y - yp1) * (y - yp1));
  double distance2 = sqrt((x - xp2) * (x - xp2) + (y - yp2) * (y - yp2));
  double distance3 = sqrt((x - xp3) * (x - xp3) + (y - yp3) * (y - yp3));
  double distance4 = sqrt((x - xp4) * (x - xp4) + (y - yp4) * (y - yp4));

  if (y1 > 0)
  {

    if (distance2 < distance4)
    {

      //go to A2
    }
    else
    {
      if (distance1 < distance3)
      {
        //go to A1
      }
      else
      {
        //go to A3
      }
    }
  }
  else
  {
    if (distance2 < distance4)
    {
      if (distance1 < distance3)
      {
        //go to A1
      }
      else
      {
        //go to A3
      }
    }
    else
    {
      //go to A4
    }
  }
} */

int robo_inside_safe_zone(double xr,double yr){
  double shortest_distance = 10000000;
  int closest_path = 0;
  double safe_points[4][2] = {{0,1.5},{-1.5,0},{0,-1.5},{1.5,0}};
  //int i;
  for(int i=0; i<4;i++){
    double distance = (safe_points[i][0]-xr)*(safe_points[i][0]-xr) + (safe_points[i][1]-yr)*(safe_points[i][1]-yr);
    if(distance < shortest_distance){
      shortest_distance = distance;
      closest_path = i+1;
    }
  }
  return closest_path; 
}


int path_planing(double xr, double yr, double xb, double yb)
{
  printf("xr:%g,yr:%g,xb:%g,yb:%g\n",xr,yr,xb,yb);
  
  double x1 = 0;
  double y1 = 1.5;
  double x2 = -1.5;
  double y2 = 0;
  double x3 = 0;
  double y3 = -1.5;
  double x4 = 1.5;
  double y4 = 0;

  int data[2];
  int pos = 0;
  int data1[2];
  int pos1 = 0;
  double x[4] = {0, -1.5, 0, 1.5};
  double y[4] = {1.5, 0, -1.5, 0};
  int path = 0;
  double xl = 0;
  double yl = 0;
  double xm = 0;
  double ym = 0;
  double xb1 = 1;
  double yb1 = 1;
  double k = 1;
  double sc=0;
  double xn=5;
  double yn=5;
  double yk=5;
  double xk=5;
  
  if (xr == 1.5 || xr == 0)
  {
    xr += 0.01;
  }
  if (yr == 1.5 || yr == 0)
  {
    yr += 0.01;
  }

  for (int i = 0; i < 4; i++)
  {
    double value = 0;

    if (fmod(i, 2) == 0)
    {
      value = ((y2 - y[i]) - (yr - y[i]) * (x2 - x[i]) / (xr - x[i])) * ((y4 - y[i]) - (yr - y[i]) * (x4 - x[i]) / (xr - x[i]));
   //printf("value1:%g\n",((y2 - y[i]) - (yr - y[i]) * (x2 - x[i]) / (xr - x[i])));
    //printf("value:%g\n,i:%d\n",value,i);
    }
    
    else
    {
      value = ((y3 - y[i]) - (yr - y[i]) * (x3 - x[i]) / (xr - x[i])) * ((y1 - y[i]) - (yr - y[i]) * (x1 - x[i]) / (xr - x[i]));
   //printf("value:%g\n,i:%d\n",value,i);
    }

    if (value > 0)
    {
      k = k * (((yb - y[i]) - (yr - y[i]) * (xb - x[i]) / (xr - x[i])) * (-y[i] + x[i] * (yr - y[i]) / (xr - x[i])));

      if (k > 0)
      {
        data[pos] = i;
        pos++;
       // printf("k>0:\n");
      }
      else
      {
        path = 0;
        //printf("k<=:\n");
      }
    }
    else
    {
      data1[pos1] = i;
      pos1++;
      //printf("value<=:\n");
    }
   // printf("k:%g\n", k);
   // printf("value:%g\n", value);
  }
  //if ( (yr-xr-1.5)*(yb-xb-1.5)>0 && ( yr +xr +1.5)*(yb+xb+1.5)>0 && (yr-xr+1.5)*(yr-xr+1.5)>0 && (yr+xr-1.5)*(yr+xr-1.5)>0){
 // path=0;} 
  //printf("y[0]:%g,y[1]:%g\n", y[data[0]], y[data[1]]);
  //printf("pos1:%d,pos2:%d\n", pos, pos1);

  if (k <= 0)
  {
    path = 0; //go to position
  }
  else
  {
    double point = (yb - yr * xb / xr) * (y[data[0]] - yr * x[data[0]] / xr);
    if (point > 0)
    {
      yl = y[data[0]];
      xl = x[data[0]];
      yn = y[data[1]];
      xn = x[data[1]];
    }
    else
    {
      yl = y[data[1]];
      xl = x[data[1]];
      yn = y[data[0]];
      xn = x[data[0]];
    }
    double point1 = (yb - yr * xb / xr) * (y[data1[0]] - yr * x[data1[0]] / xr);
    if (point1 > 0)
    {
      ym = y[data1[0]];
      xm = x[data1[0]];
      yk = y[data1[1]];
      xk = x[data1[1]];
      
    }
    else
    {
      ym = y[data1[1]];
      xm = x[data1[1]];
      yk = y[data1[0]];
      xk = x[data1[0]];
      
    }
   // printf("x[data1
  // printf("xm:%g\nym:%g\n",xm,ym);
//   printf("xl:%g\nyl:%g\n",xl,yl);
    
    //printf("1:%g\n",(xr-xl)*(xr-xl)+(yr-yl)*(yr-yl));
   //printf("2:%g\n",(xr-xm)*(xr-xm)+(yr-ym)*(yr-ym));
   // printf("3:%g\n",(xr-xb)*(xr-xb)+(yr-yb)*(yr-yb));
  
  //special case
   if( (xl == -1.5 || xl == 1.5) && yr<0 ){
    sc=yr+0.25; //-0.25
    } 
    if ((xl == -1.5||xl == 1.5) && yr>0){
    sc=0.25-yr;
     }
    if ((yl== 1.5 ||yl==-1.5) && xr >0){
    sc=0.25-xr;
    }
    if ((yl==1.5 || yl==-1.5) && xr<0){
    sc=0.25+xr;}  
   
    if ( ((xr-xl)*(xr-xl)+(yr-yl)*(yr-yl)>(xr-xm)*(xr-xm)+(yr-ym)*(yr-ym)) && sc<0 ){
    // goto (xm,ym) then (xb,yb)
  
   if (xm==0 && ym == 1.5)  
      {path=1;}
   if (xm== -1.5 && ym== 0)
      {path=2;}
   if (xm==0 && ym== -1.5)
      {path=3;}
   if (xm==1.5 && ym== 0)
      {path=4;} 
    
    }
    
    
    
    else{
    
    double coordinate = (y[data1[1]] - yl - (yb - yl) * (x[data1[1]] - xl) / (xb - xl)) * (y[data1[0]] - yl - (yb - yl) * (x[data1[0]] - xl) / (xb - xl));
    if (coordinate > 0)
    {
      //printf("go to (xl,yl)then(xb,yb)\n");
      if (xl == 0 && yl == 1.5)
      {
        path = 1 + path * 8;
      }
      else if (xl == -1.5 && yl == 0)
      {
        path = 2 + path * 8;
      }
      else if (xl == 0 && yl == -1.5)
      {
        path = 3 + path * 8;
      }
      else if (xl == 1.5 && yl == 0)
      {
        path = 4 + path * 8;
      }
      //printf("path:%o\n", path);
    }
    else
    {
      //printf("go to (xl,yl)then(xm,ym)then(xb,yb)\n");
      if (xm == 0 && ym == 1.5)
      {
        path = 1 + path * 8;
      }
      else if (xm == -1.5 && ym == 0)
      {
        path = 2 + path * 8;
      }
      else if (xm == 0 && ym == -1.5)
      {
        path = 3 + path * 8;
      }
      else if (xm == 1.5 && ym == 0)
      {
        path = 4 + path * 8;
      }
      if (xl == 0 && yl == 1.5)
      {
        path = 1 + path * 8;
      }
      else if (xl == -1.5 && yl == 0)
      {
        path = 2 + path * 8;
      }
      else if (xl == 0 && yl == -1.5)
      {
        path = 3 + path * 8;
      }
      else if (xl == 1.5 && yl == 0)
      {
        path = 4 + path * 8;
      }
    }

    //printf("path:%o\n", path);
  
  if ((xb < 1.5 && xb > -1.5) && (yb < 1.5 && yb > -1.5))
  {
    //printf("123456\n");
    if (-0.25 < xb && xb < 0.25)
    {
      xb1 = 0;
    }
    else if (xb > 0.25)
    {
      xb1 = 1.5;
    }
    else if (xb < -0.25)
    {
      xb1 = -1.5;
    }

    if (-0.25 < yb && yb < 0.25)
    {
      yb1 = 0;
    }
    else if (yb > 0.25)
    {
      yb1 = 1.5;
    }
    else if (yb < -0.25)
    {
      yb1 = -1.5;
    }

    //printf("xb1:%g,yb1:%g", xb1, yb1);
  }

  if ((yl == yb1) && (xl == xb1))
  {
   printf("goto\n");
    if (xl == 0 && yl == 1.5)
    {
      path = 1;
    }
    else if (xl == -1.5 && yl == 0)
    {
      path = 2;
    }
    else if (xl == 0 && yl == -1.5)
    {
      path = 3;
    }
    else if (xl == 1.5 && yl == 0)
    {
      path = 4;
    }
  }

  else if ((ym == yb1) && (xm == xb1))
  {
  printf("go to (xl,yl) then (xm,ym) then (xb,yb)\n");
    if (xm == 0 && ym == 1.5)
    {
      path = 1;
    }
    else if (xm == -1.5 && ym == 0)
    {
      path = 2;
    }
    else if (xm == 0 && ym == -1.5)
    {
      path = 3;
    }
    else if (xm == 1.5 && ym == 0)
    {
      path = 4;
    }
    if (xl == 0 && yl == 1.5)
    {
      path = 1 + path * 8;
    }
    else if (xl == -1.5 && yl == 0)
    {
      path = 2 + path * 8;
    }
    else if (xl == 0 && yl == -1.5)
    {
      path = 3 + path * 8;
    }
    else if (xl == 1.5 && yl == 0)
    {
      path = 4 + path * 8;
    }
   } 
   else if( ( xn == xb1 && yn == yb1) || ( xk == xb1 && yk == yb1)){
     if (xb1 == 0 && yb1== 1.5)
     {
     path=1;
     }
     if (xb1== -1.5 && yb1 == 0)
     {
     path=2;
     }
     if (xb1 == 0 && yb1== -1.5){
     path=3;}
     if ( xb1 == 1.5 && yb1 == 0){
     path=4;}
  
  }
   
 // if (yb-xb-1.5>0 &&
  
  //printf("xb1:%g\nyb1:%g\n",xb1,yb1);
  
}
  }
  printf("path:%o\n", path);
  return path;
}

int get_path(double xr,double yr,double xt,double yt){
  int path=0;
  double rev_position[4][2] = {{0,2},{-2,0},{0,-2},{2,0}};
  if((yr-xr+1.5)>0 && (yr-xr-1.5)<0 &&( yr+xr+1.5>0) && (yr+xr-1.5)<0){
    int close_path = robo_inside_safe_zone(xr,yr);
    path = path_planing(rev_position[close_path-1][0],rev_position[close_path-1][1],xt,yt);
    if(path !=0){
    path = path*8 + close_path;
    }
    else{
      path = 0;
    }
  }
  else{
    path = path_planing(xr,yr,xt,yt);
  }

  return path;


}

int main(int argc, char **argv)
{
  // initialization: robot
  wb_robot_init();

  // initialization: robot base, arm and the gripper
  base_init();
  arm_init();
  gripper_init();

 printf("finalpath:%o\n",get_path(0.1982,2.33858,0.0680707,0.53027));
  

  //path_planing(-1, 0, 3, 2);

  // Main loop
  while (wb_robot_step(TIME_STEP) != -1)
  {
    //base_backwards();
  }
  wb_robot_cleanup();
  return 0;
}
