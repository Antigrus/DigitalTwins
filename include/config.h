#ifndef CONFIG_H
#define CONFIG_H
#include "sim_types.h"
extern NodePosition g_node_positions[MAX_NODES];
extern size_t g_node_position_count;
int config_load(SimConfig *config,const char *ini_path);
void config_free(SimConfig *config);
Node *find_node(SimConfig *config,int id);
NodePosition *find_pos(SimConfig *config,int id);
#endif
