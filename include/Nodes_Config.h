#ifndef _NODES_CONFIG_H_
#define _NODES_CONFIG_H_

#include "math3d.h"

#define MAX_NODES              16
#define MAX_ARRAYS_PER_NODE    8
#define MAX_PATH               260
#define MAX_NAME               64
#define NTH                    181
#define NPH                    181
#define BEAM_TABLE_SIZE        (NTH * NPH)
#ifndef MAX_BEAM_TABLES
#define MAX_BEAM_TABLES        8

#define C0                     299792458.0
#define PI                     3.14159265358979323846

typedef enum { NODE_RX = 0, NODE_TX = 1 } NodeTRState;


#endif