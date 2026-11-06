#include "sim.h"
#include <math.h>
#include <string.h>

/* Shared beam tables and latest result per directed node/array link. */
BeamTableCache g_beam_cache;
TriNodesCalResult g_tri_nodes_cal_result;

void TriNodesCalResult_Init(TriNodesCalResult *result) {
    if (result) memset(result,0,sizeof(*result));
}

static int same_link(const ChannelResult *a, const ChannelResult *b) {
    return a->txid==b->txid && a->rxid==b->rxid && a->txaid==b->txaid && a->rxaid==b->rxaid;
}

int TriNodesCalResult_Update(TriNodesCalResult *result, const ChannelResult *r) {
    if (!result || !r) return 0;
    for (size_t i=0;i<result->count;i++) if (same_link(&result->links[i],r)) {
        result->links[i]=*r;
        return 1;
    }
    if (result->count>=MAX_LINK_RESULTS) return 0;
    result->links[result->count++]=*r;
    return 1;
}

void BeamTable_Init(BeamTableCache *cache) {
    if (cache) cache->count = 0;
}

bool BeamTable_ConfigEqual(const BeamTableConfig *a, const BeamTableConfig *b) {
    if (!a || !b) return false;
    /* Values originate from the same parsed configuration records. Small tolerance
       avoids false misses caused by harmless floating-point representation noise. */
    const float eps_d = 1e-6f; /* cm */
    const float eps_f = 1e-6f; /* MHz, i.e. 1 Hz */
    return a->num_h == b->num_h && a->num_v == b->num_v &&
           fabsf(a->spacing_h_cm-b->spacing_h_cm) <= eps_d &&
           fabsf(a->spacing_v_cm-b->spacing_v_cm) <= eps_d &&
           fabsf(a->frequency_mhz-b->frequency_mhz) <= eps_f;
}

/* Magnitude of sum(k=0..count-1) exp(j*k*phase), using the closed form.
   This is mathematically equivalent to the complex summation in cal_beamform(). */
static float uniform_line_factor(uint16_t count, float phase) {
    if (count <= 1) return 1.0f;
    float den = sinf(0.5f * phase);
    if (fabsf(den) < 1e-6f) return (float)count;
    return fabsf(sinf(0.5f * (float)count * phase) / den);
}

bool BeamTable_Generate(const BeamTableConfig *cfg, float gain[NTH][NPH]) {
    if (!cfg || !gain || cfg->num_h == 0 || cfg->num_v == 0 ||
        cfg->spacing_h_cm <= 0.0f || cfg->spacing_v_cm <= 0.0f || cfg->frequency_mhz <= 0.0f)
        return false;

    const float lambda = (float)(C0 / (cfg->frequency_mhz * 1.0e6f));
    const float kh = (float)(2.0f * PI * (cfg->spacing_h_cm * 0.01f) / lambda);
    const float kv = (float)(2.0f * PI * (cfg->spacing_v_cm * 0.01f) / lambda);
    float sin_theta[91], cos_theta[91], sin_phi[91];

    /* Same angular convention and quadrant symmetry as the supplied cal_beamform():
       theta, phi are -90..0 degrees for the first quadrant of the table. */
    for (int i=0; i<=90; ++i) {
        float a = ((float)i - 90.0f) * (float)(PI/180.0f);
        sin_theta[i] = sinf(a);
        cos_theta[i] = cosf(a);
        sin_phi[i] = sinf(a);
    }

    const float norm_factor = sqrtf((float)cfg->num_h * (float)cfg->num_v);
    for (int ti=0; ti<=90; ++ti) {
        for (int pi=0; pi<=90; ++pi) {
            /* Generalized form of the supplied phase expression.
               For dH=dV=lambda/2 this becomes 2*pi*(m*sinf(phi)*cosf(theta)+n*sinf(theta))/2. */
            float phase_h = kh * sin_phi[pi] * cos_theta[ti];
            float phase_v = kv * sin_theta[ti];
            float ah = uniform_line_factor(cfg->num_h, phase_h);
            float av = uniform_line_factor(cfg->num_v, phase_v);
            float value = (ah * av) / norm_factor;

            gain[ti][pi] = value;
            gain[180-ti][pi] = value;
            gain[180-ti][180-pi] = value;
            gain[ti][180-pi] = value;
        }
    }
    return true;
}

int16_t BeamTable_GetOrCreate(BeamTableCache *cache, const BeamTableConfig *cfg) {
    if (!cache || !cfg) return -1;
    for (uint16_t i=0; i<cache->count; ++i)
        if (BeamTable_ConfigEqual(&cache->entries[i].config, cfg)) return (int16_t)i;
    if (cache->count >= MAX_BEAM_TABLES) return -1;

    uint16_t index = cache->count;
    BeamTableEntry *entry = &cache->entries[index];
    if (!BeamTable_Generate(cfg, entry->gain)) return -1;
    entry->config = *cfg;
    cache->count++;
    return (int16_t)index;
}

const float (*BeamTable_Get(const BeamTableCache *cache, int16_t index))[NPH] {
    if (!cache || index < 0 || index >= (int16_t)cache->count) return NULL;
    return cache->entries[index].gain;
}

bool BeamTable_InitAllNodes(BeamTableCache *cache, Node *nodes, uint16_t node_count) {
    if (!cache || (!nodes && node_count)) return false;
    BeamTable_Init(cache);
    for (uint16_t n=0; n<node_count; ++n) {
        for (size_t a=0; a<nodes[n].array_count; ++a) {
            ArrayMount *array = &nodes[n].arrays[a];
            array->beam_table_index = BeamTable_GetOrCreate(cache, &array->beam_config);
            if (array->beam_table_index < 0) return false;
        }
    }
    return true;
}

/* theta/phi are the angles used by cal_beamform(), both in [-90,90] degrees.
   Bilinear interpolation is used instead of nearest-neighbour lookup. */
float beam_gain(const BeamTableCache *cache, int16_t index, float theta_deg, float phi_deg) {
    const float (*table)[NPH] = BeamTable_Get(cache, index);
    if (!table) return 0.0f;
    if (theta_deg < -90.0f) theta_deg = -90.0f;
    if (theta_deg >  90.0f) theta_deg =  90.0f;
    if (phi_deg < -90.0f) phi_deg = -90.0f;
    if (phi_deg >  90.0f) phi_deg =  90.0f;

    float ti = theta_deg + 90.0f, pi = phi_deg + 90.0f;
    int t0=(int)floorf(ti), p0=(int)floorf(pi);
    int t1=(t0<180)?t0+1:180, p1=(p0<180)?p0+1:180;
    float at=ti-t0, ap=pi-p0;
    float g00=table[t0][p0], g01=table[t0][p1];
    float g10=table[t1][p0], g11=table[t1][p1];
    return (1.0f-at)*((1.0f-ap)*g00+ap*g01) + at*((1.0f-ap)*g10+ap*g11);
}
