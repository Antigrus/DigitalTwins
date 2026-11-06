#ifndef SIM_H
#define SIM_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define MAX_NODES              3
#define MAX_ARRAYS_PER_NODE    16
#define LINK_INDEX_SPACE       (1U << 12) /* 2-bit TX node + 2-bit RX node + 4-bit TX/RX array slots */
#define LINK_INDEX_BITMAP_SIZE (LINK_INDEX_SPACE / 8U)
#define LINK_INDEX_INVALID     UINT16_MAX

/* index layout: [11:10] txid, [9:8] rxid, [7:4] tx array slot, [3:0] rx array slot.
   Array slots are positions in Node.arrays[], NOT ArrayMount.id values. */
static inline bool LinkIndex_Encode(uint8_t txid, uint8_t rxid, uint8_t tx_slot, uint8_t rx_slot, uint16_t *index) {
    if (!index || txid > 3U || rxid > 3U || txid == rxid || tx_slot >= MAX_ARRAYS_PER_NODE || rx_slot >= MAX_ARRAYS_PER_NODE) return false;
    *index = (uint16_t)(((uint16_t)txid << 10) | ((uint16_t)rxid << 8) | ((uint16_t)tx_slot << 4) | (uint16_t)rx_slot);
    return true;
  }
#define MAX_PATH               260
#define MAX_NAME               64
#define NTH                    181
#define NPH                    181
#ifndef MAX_BEAM_TABLES
#define MAX_BEAM_TABLES        8
#endif
#define C0                     299792458.0f
#define PI                     3.14159265358979323846f

typedef enum { NODE_RX = 0, NODE_TX = 1 } NodeTRState;
typedef struct { float x, y, z; } Vec3;
typedef struct { float heading, pitch, roll; } Attitude;
typedef struct { float azimuth, elevation; } BeamDirection;
typedef struct { float re, im; } Complex;
typedef struct { float t; Vec3 p; } TrajectoryPoint;

typedef struct {
    uint16_t num_h;             /* horizontal element count */
    uint16_t num_v;             /* vertical element count */
    float spacing_h_cm;         /* horizontal spacing, centimetres */
    float spacing_v_cm;         /* vertical spacing, centimetres */
    float frequency_mhz;        /* operating frequency, MHz */
} BeamTableConfig;

typedef struct {
    BeamTableConfig config;
    float gain[NTH][NPH]; /* gain[theta_idx][phi_idx], linear field amplitude */
} BeamTableEntry;

typedef struct {
    uint16_t count;
    BeamTableEntry entries[MAX_BEAM_TABLES];
} BeamTableCache;

typedef struct {
    int id;
    int enabled;
    char name[MAX_NAME];
    Vec3 offset;
    Attitude initial_attitude;
    BeamTableConfig beam_config;
    float element_tx_power_dbm;
    int16_t beam_table_index;
} ArrayMount;

/* Node owns the half-duplex TX/RX state. Every array inherits this state. */
typedef struct {
    int id;
    int enabled;
    char name[MAX_NAME];
    NodeTRState tr_state;
    Vec3 velocity;
    Attitude attitude;
    int tx_array_id;              /* -1: automatic; otherwise ArrayMount.id */
    BeamDirection beam_direction; /* scan offset in each array local frame, degrees */
    ArrayMount arrays[MAX_ARRAYS_PER_NODE];
    size_t array_count;
} Node;

typedef struct {
    int id;
    int enabled;
    char name[MAX_NAME];
    char trajectory_file[MAX_PATH];
    TrajectoryPoint *trajectory;
    size_t trajectory_count;
    Vec3 position;
} NodePosition;

typedef struct {
    float t, d, tau, vr, fd, fspl;
    float gtx_db, grx_db;
    float element_tx_power_dbm, pr_dbm;
    int txid, rxid, txaid, rxaid;
    uint16_t index;             /* direct-cache key; array fields are array slots */
    uint8_t valid;
    Complex h;
} ChannelResult;

typedef struct {
    ChannelResult links[LINK_INDEX_SPACE];
    uint8_t occupied[LINK_INDEX_BITMAP_SIZE];
    size_t count;               /* number of occupied index entries */
} TriNodesCalResult;

typedef struct {
    float start, end, step;
    Node nodes[MAX_NODES];
    size_t nn;
    NodePosition positions[MAX_NODES];
    size_t np;
} SimConfig;

extern NodePosition g_node_positions[MAX_NODES];
extern size_t g_node_position_count;
extern BeamTableCache g_beam_cache;
extern TriNodesCalResult g_tri_nodes_cal_result;
void TriNodesCalResult_Init(TriNodesCalResult *result);
int TriNodesCalResult_Update(TriNodesCalResult *result, const ChannelResult *channel_result);
const ChannelResult *TriNodesCalResult_Get(const TriNodesCalResult *result, uint16_t index);

/* 3-D math and attitude */
Vec3 v3(float x, float y, float z);
Vec3 add(Vec3 a, Vec3 b);
Vec3 sub(Vec3 a, Vec3 b);
Vec3 scale(Vec3 a, float s);
float dot(Vec3 a, Vec3 b);
float norm(Vec3 a);
float deg(float r);
typedef struct { float m[3][3]; } Mat3;
Mat3 node_rot(Attitude a);
Mat3 array_rot(Attitude node, Attitude mount);
Mat3 compose_rot(Mat3 a, Mat3 b);
Vec3 apply(Mat3 m, Vec3 v);

/* Configuration and trajectories */
int config_load(SimConfig *c, const char *ini);
void config_free(SimConfig *c);
Node *find_node(SimConfig *c, int id);
NodePosition *find_pos(SimConfig *c, int id);
int load_trajectory(NodePosition *p, const char *path);
int update_state(Node *n, NodePosition *p, float t);

/* Global shared beam-table cache */
void BeamTable_Init(BeamTableCache *cache);
bool BeamTable_ConfigEqual(const BeamTableConfig *a, const BeamTableConfig *b);
bool BeamTable_Generate(const BeamTableConfig *config, float gain[NTH][NPH]);
int16_t BeamTable_GetOrCreate(BeamTableCache *cache, const BeamTableConfig *config);
const float (*BeamTable_Get(const BeamTableCache *cache, int16_t index))[NPH];
bool BeamTable_InitAllNodes(BeamTableCache *cache, Node *nodes, uint16_t node_count);
float beam_gain(const BeamTableCache *cache, int16_t index, float theta_deg, float phi_deg);

/* Link calculation */
ChannelResult channel(const SimConfig *c, const Node *tx, const NodePosition *tp,
                      const Node *rx, const NodePosition *rp,
                      const ArrayMount *ta, const ArrayMount *ra, float t);
void print_channel(ChannelResult r);

#endif
