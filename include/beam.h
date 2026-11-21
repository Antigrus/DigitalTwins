#ifndef BEAM_H
#define BEAM_H
#include "sim_types.h"
extern BeamTableCache g_beam_cache;
extern TriNodesCalResult g_tri_nodes_cal_result;
void TriNodesCalResult_Init(TriNodesCalResult *result);
int TriNodesCalResult_Update(TriNodesCalResult *result,const ChannelResult *channel_result);
const ChannelResult *TriNodesCalResult_Get(const TriNodesCalResult *result,uint16_t index);
void BeamTable_Init(BeamTableCache *cache);
bool BeamTable_ConfigEqual(const BeamTableConfig *a,const BeamTableConfig *b);
bool BeamTable_Generate(const BeamTableConfig *config,float gain[NTH][NPH]);
int16_t BeamTable_GetOrCreate(BeamTableCache *cache,const BeamTableConfig *config);
const float (*BeamTable_Get(const BeamTableCache *cache,int16_t index))[NPH];
bool BeamTable_InitAllNodes(BeamTableCache *cache,Node *nodes,uint16_t node_count);
float beam_gain(const BeamTableCache *cache,int16_t index,float theta_deg,float phi_deg);
#endif
