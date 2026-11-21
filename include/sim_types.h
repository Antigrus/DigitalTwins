#ifndef SIM_TYPES_H
#define SIM_TYPES_H
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "geodesy.h"
#define MAX_NODES 3
#define MAX_ARRAYS_PER_NODE 16
#define LINK_INDEX_SPACE (1U << 12)
#define LINK_INDEX_BITMAP_SIZE (LINK_INDEX_SPACE / 8U)
#define LINK_INDEX_INVALID UINT16_MAX
#define MAX_PATH 260
#define MAX_NAME 64
#define NTH 181
#define NPH 181
#ifndef MAX_BEAM_TABLES
#define MAX_BEAM_TABLES 8
#endif
#define C0 299792458.0
#define PI 3.14159265358979323846
static inline bool LinkIndex_Encode(uint8_t txid,uint8_t rxid,uint8_t tx_slot,uint8_t rx_slot,uint16_t *index){if(!index||txid>3U||rxid>3U||txid==rxid||tx_slot>=MAX_ARRAYS_PER_NODE||rx_slot>=MAX_ARRAYS_PER_NODE)return false;*index=(uint16_t)(((uint16_t)txid<<10)|((uint16_t)rxid<<8)|((uint16_t)tx_slot<<4)|(uint16_t)rx_slot);return true;}
typedef enum {NODE_RX=0,NODE_TX=1} NodeTRState;
typedef struct {float heading,pitch,roll;} Attitude;
typedef struct {float azimuth,elevation;} BeamDirection;
typedef struct {float re,im;} Complex;
typedef struct {double t;Geodetic geo;Attitude attitude;Vec3 ecef;} TrajectoryPoint;
typedef struct {uint16_t num_h,num_v;float spacing_h_cm,spacing_v_cm,frequency_mhz;} BeamTableConfig;
typedef struct {BeamTableConfig config;float gain[NTH][NPH];} BeamTableEntry;
typedef struct {uint16_t count;BeamTableEntry entries[MAX_BEAM_TABLES];} BeamTableCache;
typedef struct {int id,enabled;char name[MAX_NAME];Vec3 offset;Attitude initial_attitude;BeamTableConfig beam_config;float element_tx_power_dbm;int16_t beam_table_index;} ArrayMount;
typedef struct {int id,enabled;char name[MAX_NAME];NodeTRState tr_state;Vec3 velocity;Attitude attitude;int tx_array_id;BeamDirection beam_direction;ArrayMount arrays[MAX_ARRAYS_PER_NODE];size_t array_count;} Node;
typedef struct {int id,enabled;char name[MAX_NAME],trajectory_file[MAX_PATH];TrajectoryPoint *trajectory;size_t trajectory_count;Vec3 position,velocity;Geodetic geodetic;} NodePosition;
typedef struct {double t,d,tau,vr,fd,fspl;float gtx_db,grx_db,element_tx_power_dbm,pr_dbm;int txid,rxid,txaid,rxaid;uint16_t index;uint8_t valid;Complex h;} ChannelResult;
typedef struct {ChannelResult links[LINK_INDEX_SPACE];uint8_t occupied[LINK_INDEX_BITMAP_SIZE];size_t count;} TriNodesCalResult;
typedef struct {double start,end,step;Node nodes[MAX_NODES];size_t nn;NodePosition positions[MAX_NODES];size_t np;} SimConfig;
#endif
