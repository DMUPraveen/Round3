#ifndef ADVANCED_NAVIGATION
#define ADVANCED_NAVIGATION


int go_to_position();
void set_target(double position[3]);
int higher_level_go_to(int path,double position[3]);
void initialize_naviagtion();
#endif