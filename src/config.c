#include "sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

NodePosition g_node_positions[MAX_NODES];
size_t g_node_position_count = 0;

static void trim(char *s) {
    size_t n=strlen(s);
    while (n && (s[n-1]=='\n' || s[n-1]=='\r' || isspace((unsigned char)s[n-1]))) s[--n]=0;
    char *p=s; while (*p && isspace((unsigned char)*p)) ++p;
    if (p!=s) memmove(s,p,strlen(p)+1);
}
static int path_join(char *out, size_t cap, const char *base, const char *path) {
    if (!out || !base || !path || !*path) return 0;
    if (path[0]=='/' || (isalpha((unsigned char)path[0]) && path[1]==':'))
        return snprintf(out,cap,"%s",path) < (int)cap;
    return snprintf(out,cap,"%s/%s",base,path) < (int)cap;
}
static int parse_state(const char *s, NodeTRState *state) {
    if (!strcmp(s,"TX") || !strcmp(s,"tx") || !strcmp(s,"1")) { *state=NODE_TX; return 1; }
    if (!strcmp(s,"RX") || !strcmp(s,"rx") || !strcmp(s,"0")) { *state=NODE_RX; return 1; }
    return 0;
}
Node *find_node(SimConfig *c, int id) {
    for (size_t i=0;i<c->nn;i++) if (c->nodes[i].id==id) return &c->nodes[i];
    return NULL;
}
NodePosition *find_pos(SimConfig *c, int id) {
    for (size_t i=0;i<c->np;i++) if (c->positions[i].id==id) return &c->positions[i];
    return NULL;
}

int config_load(SimConfig *c, const char *ini) {
    if (!c || !ini) return 0;
    memset(c,0,sizeof(*c));
    char base[MAX_PATH], line[1024], nodes_csv[MAX_PATH]="", arrays_csv[MAX_PATH]="";
    if (snprintf(base,sizeof(base),"%s",ini)>=(int)sizeof(base)) return 0;
    char *slash=strrchr(base,'/'); char *bslash=strrchr(base,'\\');
    if (bslash && (!slash || bslash>slash)) slash=bslash;
    if (slash) *slash=0; else strcpy(base,".");

    FILE *f=fopen(ini,"r"); if (!f) return 0;
    while (fgets(line,sizeof(line),f)) {
        trim(line); if (!line[0] || line[0]=='#' || line[0]==';') continue;
        char *eq=strchr(line,'='); if (!eq) continue; *eq++=0; trim(line); trim(eq);
        if (!strcmp(line,"start_time_s")) c->start=strtof(eq, NULL);
        else if (!strcmp(line,"end_time_s")) c->end=strtof(eq, NULL);
        else if (!strcmp(line,"step_s")) c->step=strtof(eq, NULL);
        else if (!strcmp(line,"nodes_csv")) snprintf(nodes_csv,sizeof(nodes_csv),"%s",eq);
        else if (!strcmp(line,"arrays_csv")) snprintf(arrays_csv,sizeof(arrays_csv),"%s",eq);
    }
    fclose(f);
    if (c->step<=0 || c->end<c->start || !nodes_csv[0] || !arrays_csv[0]) return 0;

    char full[MAX_PATH];
    if (!path_join(full,sizeof(full),base,nodes_csv)) return 0;
    f=fopen(full,"r"); if (!f) { perror(full); return 0; }
    while (fgets(line,sizeof(line),f)) {
        int id,enabled,tx_array_id; float beam_azimuth,beam_elevation;
        char name[MAX_NAME],state_text[16],trajectory[MAX_PATH];
        int fields=sscanf(line,"%d,%63[^,],%d,%15[^,],%259[^,],%d,%f,%f",&id,name,&enabled,state_text,trajectory,&tx_array_id,&beam_azimuth,&beam_elevation);
        if (fields!=8) continue; /* header or malformed line */
        if (c->nn>=MAX_NODES || id<0 || id>3 || find_node(c,id)) { fclose(f); return 0; }
        Node *n=&c->nodes[c->nn]; NodePosition *p=&c->positions[c->np];
        memset(n,0,sizeof(*n)); memset(p,0,sizeof(*p));
        if (!parse_state(state_text,&n->tr_state)) { fclose(f); return 0; }
        n->id=id; n->enabled=enabled; n->tx_array_id=tx_array_id; n->beam_direction=(BeamDirection){beam_azimuth,beam_elevation}; snprintf(n->name,sizeof(n->name),"%s",name);
        p->id=id; p->enabled=enabled; snprintf(p->name,sizeof(p->name),"%s",name);
        snprintf(p->trajectory_file,sizeof(p->trajectory_file),"%s",trajectory);
        if (!path_join(full,sizeof(full),base,trajectory) || !load_trajectory(p,full)) { fclose(f); return 0; }
        c->nn++; c->np++;
    }
    fclose(f);

    if (!path_join(full,sizeof(full),base,arrays_csv)) return 0;
    f=fopen(full,"r"); if (!f) { perror(full); return 0; }
    while (fgets(line,sizeof(line),f)) {
        int aid,nid,enabled; char name[MAX_NAME];
        unsigned int nh,nv;
        float x,y,z,yaw,pitch,roll,dh,dv,fmhz,power;
        int fields=sscanf(line,"%d,%d,%63[^,],%d,%f,%f,%f,%f,%f,%f,%u,%u,%f,%f,%f,%f",
            &aid,&nid,name,&enabled,&x,&y,&z,&yaw,&pitch,&roll,&nh,&nv,&dh,&dv,&fmhz,&power);
        if (fields!=16) continue; /* header line */
        Node *n=find_node(c,nid);
        if (!n || n->array_count>=MAX_ARRAYS_PER_NODE || nh>65535 || nv>65535) { fclose(f); return 0; }
        ArrayMount *a=&n->arrays[n->array_count++];
        memset(a,0,sizeof(*a));
        a->id=aid; a->enabled=enabled; snprintf(a->name,sizeof(a->name),"%s",name);
        a->offset=v3(x,y,z); a->initial_attitude=(Attitude){yaw,pitch,roll};
        a->beam_config=(BeamTableConfig){(uint16_t)nh,(uint16_t)nv,dh,dv,fmhz};
        a->element_tx_power_dbm=power; a->beam_table_index=-1;
        if (nh==0 || nv==0 || dh<=0 || dv<=0 || fmhz<=0) { fclose(f); return 0; }
    }
    fclose(f);
    if (c->nn==0 || c->np!=c->nn) return 0;
    for (size_t i=0;i<c->nn;i++) if (c->nodes[i].tx_array_id>=0) {
        int found=0;
        for (size_t a=0;a<c->nodes[i].array_count;a++)
            if (c->nodes[i].arrays[a].id==c->nodes[i].tx_array_id && c->nodes[i].arrays[a].enabled) found=1;
        if (!found) { fprintf(stderr,"Node %d specifies missing/disabled TX array ID %d\n",c->nodes[i].id,c->nodes[i].tx_array_id); return 0; }
    }
    memcpy(g_node_positions,c->positions,sizeof(c->positions));
    g_node_position_count=c->np;
    return 1;
}

void config_free(SimConfig *c) {
    if (!c) return;
    for (size_t i=0;i<c->np;i++) free(c->positions[i].trajectory);
    memset(c,0,sizeof(*c));
    g_node_position_count=0;
}
