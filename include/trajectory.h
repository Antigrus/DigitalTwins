#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include "sim_types.h"
int load_trajectory(NodePosition *position,const char *path);
int update_state(Node *node,NodePosition *position,double time_s);
#endif
