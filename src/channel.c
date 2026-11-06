#include "sim.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static Mat3 transpose(Mat3 a) {
    Mat3 r={{{a.m[0][0],a.m[1][0],a.m[2][0]},
             {a.m[0][1],a.m[1][1],a.m[2][1]},
             {a.m[0][2],a.m[1][2],a.m[2][2]}}};
    return r;
}
static Vec3 array_position(const Node *n, const NodePosition *p, const ArrayMount *a) {
    return add(p->position,apply(node_rot(n->attitude),a->offset));
}
/* Beam formula convention: array local +X is broadside, +Y is horizontal aperture,
   +Z is vertical aperture. phi=atan2f(local_y,abs(local_x)); theta is elevation. */
static float array_gain_amplitude(const Node *n, const ArrayMount *a, Vec3 direction_global) {
    /* Scan azimuth/elevation are local to the array. node_rot() gives
       Rz(azimuth)*Ry(-elevation), rotating local +X boresight toward the scan vector. */
    Mat3 r=array_rot(n->attitude,a->initial_attitude);
    r=compose_rot(r,node_rot((Attitude){n->beam_direction.azimuth,n->beam_direction.elevation,0.0f}));
    Vec3 local=apply(transpose(r),direction_global);
    float theta=deg(atan2f(local.z,hypotf(local.x,local.y)));
    float phi=deg(atan2f(local.y,fabsf(local.x)));
    return beam_gain(&g_beam_cache,a->beam_table_index,theta,phi);
}

ChannelResult channel(const SimConfig *c, const Node *tx, const NodePosition *tp,
                      const Node *rx, const NodePosition *rp,
                      const ArrayMount *ta, const ArrayMount *ra, float t) {
    ChannelResult o; memset(&o,0,sizeof(o));
    o.index=LINK_INDEX_INVALID;
    o.t=t; o.txid=tx?tx->id:-1; o.rxid=rx?rx->id:-1;
    o.txaid=ta?ta->id:-1; o.rxaid=ra?ra->id:-1;
    if (!c || !tx || !rx || !tp || !rp || !ta || !ra || !tx->enabled || !rx->enabled ||
        !ta->enabled || !ra->enabled || tx->tr_state!=NODE_TX || rx->tr_state!=NODE_RX) return o;
    ptrdiff_t tx_slot=ta-tx->arrays, rx_slot=ra-rx->arrays;
    if (tx_slot<0 || tx_slot>=(ptrdiff_t)tx->array_count || rx_slot<0 || rx_slot>=(ptrdiff_t)rx->array_count ||
        !LinkIndex_Encode((uint8_t)tx->id,(uint8_t)rx->id,(uint8_t)tx_slot,(uint8_t)rx_slot,&o.index)) return o;
    if (tx->tx_array_id>=0 && ta->id!=tx->tx_array_id) return o;
    if (fabsf(ta->beam_config.frequency_mhz-ra->beam_config.frequency_mhz)>1e-6f) return o;

    const float fc=ta->beam_config.frequency_mhz*1.0e6f;
    Vec3 delta=sub(array_position(rx,rp,ra),array_position(tx,tp,ta));
    o.d=norm(delta); if (o.d<1e-6f) return o;
    Vec3 u=scale(delta,1.0f/o.d);
    o.tau=o.d/C0;
    o.vr=dot(sub(rx->velocity,tx->velocity),u);
    o.fd=-o.vr*fc/C0;
    o.fspl=20.0f*log10f(4.0f*PI*o.d*fc/C0);

    float atx=array_gain_amplitude(tx,ta,u);
    float arx=array_gain_amplitude(rx,ra,scale(u,-1.0f));
    if (atx<=0.0f || arx<=0.0f) return o;
    o.gtx_db=20.0f*log10f(atx);
    o.grx_db=20.0f*log10f(arx);
    o.element_tx_power_dbm=ta->element_tx_power_dbm;

    /* The table is |AF|/sqrtf(M*N). Per-element TX power is converted to
       total array input power by +10log10f(M*N); AF contributes its directional gain. */
    float ntx=(float)ta->beam_config.num_h*ta->beam_config.num_v;
    o.pr_dbm=o.element_tx_power_dbm+10.0f*log10f(ntx)+o.gtx_db+o.grx_db-o.fspl;

    /* h is referenced to one TX element's input signal and includes coherent TX
       array combining, RX array combining, and free-space propagation. */
    float lambda=C0/fc;
    float amp=sqrtf(ntx)*atx*arx*lambda/(4.0f*PI*o.d);
    float phase=-2.0f*PI*fc*o.tau;
    o.h.re=amp*cosf(phase); o.h.im=amp*sinf(phase);
    o.valid=1;
    return o;
}

void print_channel(ChannelResult r) {
    if (!r.valid) { printf("  H%d%d array %d->%d unavailable (state/frequency/config check)\n",r.txid,r.rxid,r.txaid,r.rxaid); return; }
    printf("  H%d%d array %d->%d [index=%u]: d=%.3f m tau=%.4e s vr=%.3f m/s fd=%.3f Hz FSPL=%.2f dB "
           "Gtx=%.2f dB Grx=%.2f dB Pelem=%.2f dBm Pr=%.2f dBm h=(%+.4e,%+.4e)\n",
           r.txid,r.rxid,r.txaid,r.rxaid,(unsigned)r.index,r.d,r.tau,r.vr,r.fd,r.fspl,r.gtx_db,r.grx_db,
           r.element_tx_power_dbm,r.pr_dbm,r.h.re,r.h.im);
}
