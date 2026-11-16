#ifndef CHANNEL_H
#define CHANNEL_H
#include "sim_types.h"
ChannelResult channel(const SimConfig *config,const Node *tx,const NodePosition *tx_position,
                      const Node *rx,const NodePosition *rx_position,
                      const ArrayMount *tx_array,const ArrayMount *rx_array,float time_s);
void print_channel(ChannelResult result);
#endif
