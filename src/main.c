#include "sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *state_name(NodeTRState s) { return s==NODE_TX ? "TX" : "RX"; }

static void print_beam_cache(const BeamTableCache *cache) {
    printf("\nBeam table cache: %u unique table(s)\n",cache->count);
    for (uint16_t i=0;i<cache->count;i++) {
        const BeamTableConfig *b=&cache->entries[i].config;
        printf("  index=%u MxN=%ux%u dH=%.6f cm dV=%.6f cm f=%.6f MHz\n",
               i,b->num_h,b->num_v,b->spacing_h_cm,b->spacing_v_cm,b->frequency_mhz);
    }
}

static void run(SimConfig *c, float t) {
    printf("\n===== t=%.3f s =====\n",t);
    for (size_t i=0;i<c->nn;i++) {
        Node *n=&c->nodes[i]; NodePosition *p=find_pos(c,n->id);
        if (!p || !update_state(n,p,t)) continue;
        printf("Node %d %-8s state=%s tx_array=%d beam_scan=(%.1f,%.1f)deg pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) "
               "attitude=(%.2f,%.2f,%.2f) deg\n",n->id,n->name,state_name(n->tr_state),n->tx_array_id,n->beam_direction.azimuth,n->beam_direction.elevation,
               p->position.x,p->position.y,p->position.z,n->velocity.x,n->velocity.y,n->velocity.z,
               n->attitude.heading,n->attitude.pitch,n->attitude.roll);
    }

    /* A node is half-duplex: its own state determines whether all its arrays TX or RX.
       Every pair with opposite states creates one directed link. */
    for (size_t i=0;i<c->nn;i++) for (size_t j=i+1;j<c->nn;j++) {
        Node *a=&c->nodes[i], *b=&c->nodes[j];
        if (!a->enabled || !b->enabled || a->tr_state==b->tr_state) continue;
        Node *tx=(a->tr_state==NODE_TX)?a:b;
        Node *rx=(a->tr_state==NODE_RX)?a:b;
        NodePosition *tp=find_pos(c,tx->id), *rp=find_pos(c,rx->id);
        if (!tp || !rp) continue;
        printf("\n[Link %d(%s) -> %d(%s)]\n",tx->id,state_name(tx->tr_state),rx->id,state_name(rx->tr_state));
        for (size_t ta=0;ta<tx->array_count;ta++) {
            if (tx->tx_array_id>=0 && tx->arrays[ta].id!=tx->tx_array_id) continue;
            for (size_t ra=0;ra<rx->array_count;ra++) {
                ChannelResult result=channel(c,tx,tp,rx,rp,&tx->arrays[ta],&rx->arrays[ra],t);
                if (!TriNodesCalResult_Update(&g_tri_nodes_cal_result,&result))
                    fprintf(stderr,"Global channel result capacity exceeded\n");
                print_channel(result);
            }
        }
    }
}

int main(int argc, char **argv) {
    const char *ini="config/simulation.ini"; float requested_time=0.0f; int once=0;
    for (int i=1;i<argc;i++) {
        if (!strcmp(argv[i],"--time") && i+1<argc) { requested_time=strtof(argv[++i], NULL); once=1; }
        else if (!strcmp(argv[i],"--help")) { puts("Usage: channel_sim [config.ini] [--time seconds]"); return 0; }
        else ini=argv[i];
    }

    SimConfig *c=calloc(1,sizeof(*c));
    if (!c) { fprintf(stderr,"Cannot allocate simulation config\n"); return 1; }
    if (!config_load(c,ini)) { fprintf(stderr,"Cannot load configuration: %s\n",ini); free(c); return 1; }
    if (!BeamTable_InitAllNodes(&g_beam_cache,c->nodes,(uint16_t)c->nn)) {
        fprintf(stderr,"Beam table initialization failed (capacity=%d or invalid array config)\n",MAX_BEAM_TABLES);
        config_free(c); free(c); return 1;
    }

    TriNodesCalResult_Init(&g_tri_nodes_cal_result);
    printf("Loaded %zu node(s). Node TR state controls all arrays.\n",c->nn);
    for (size_t n=0;n<c->nn;n++) {
        Node *node=&c->nodes[n];
        for (size_t a=0;a<node->array_count;a++) {
            ArrayMount *ar=&node->arrays[a];
            printf("Node %d array %d: %ux%u, dH=%.4f cm, dV=%.4f cm, f=%.3f MHz, "
                   "element Ptx=%.2f dBm, beam_index=%d%s\n",node->id,ar->id,
                   ar->beam_config.num_h,ar->beam_config.num_v,ar->beam_config.spacing_h_cm,
                   ar->beam_config.spacing_v_cm,ar->beam_config.frequency_mhz,
                   ar->element_tx_power_dbm,ar->beam_table_index,(node->tx_array_id==ar->id?" [selected TX]":""));
        }
    }
    print_beam_cache(&g_beam_cache);

    if (once) run(c,requested_time);
    else for (float t=c->start;t<=c->end+1e-6f;t+=c->step) run(c,t);
    printf("\nGlobal TriNodesCalResult: %zu directed link record(s)\n",g_tri_nodes_cal_result.count);
    config_free(c); free(c); return 0;
}
