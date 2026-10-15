/* 运行实际底盘编排与仓库原语；设备替身不证明现场间隙、停车距离。 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include "app_main_under_test.inc"
static uint32_t tick;
static float velocity, pose_y;
static map_point_t pose;
static route_side_t side;
static unsigned phase, sensor_reads, event_count, next_command, paths, stairs_calls;
static unsigned fault_case, motion_after_stop, stopped, fail_events, homes;
static uint16_t last_id;
static jmp_buf finished;
volatile mission_color_t g_mission_side;
volatile mission_run_mode_t g_mission_run_mode;
static const mission_command_type_t commands[] = {
 MISSION_CMD_GO_PLATFORM, MISSION_CMD_GO_DEPOT_1, MISSION_CMD_GO_DEPOT_2,
 MISSION_CMD_GO_DEPOT_4, MISSION_CMD_GO_DEPOT_1, MISSION_CMD_DEPOT_OK };
uint32_t osKernelGetTickCount(void) { return tick; }
uint32_t osKernelGetTickFreq(void) { return 1000U; }
osStatus_t osDelay(uint32_t n) {
 tick+=n;
 if(phase==ROUTE_DEPOT && velocity!=0) { pose_y -= velocity*route_lat_sign()*n/1000.0f;pose.y_mm=(int16_t)pose_y; }
 if(n==APP_HOME_WAIT_MS) longjmp(finished,1);
 return osOK;
}
osThreadId_t osThreadNew(osThreadFunc_t fn,void *arg,const osThreadAttr_t *attr)
{ (void)fn;(void)arg;(void)attr;return (osThreadId_t)&tick; }
route_side_t route_side(void) { return side; }
float route_lat_sign(void) { return side==ROUTE_SIDE_MIRROR?-1.0f:1.0f; }
int16_t route_side_x(int16_t x) { return side==ROUTE_SIDE_MIRROR?2500-x:x; }
app_status_t route_set_side(route_side_t s) { side=s;return APP_OK; }
csvc_status_t csvc_free(float vx,float vy,float wz) {
 (void)vx;(void)wz;
 if(stopped && (vx!=0||vy!=0||wz!=0)) motion_after_stop++;
 velocity=vy;return CSVC_OK;
}
align_status_t align_stop(void) { velocity=0;tick+=200;return ALIGN_OK; }
align_status_t align_on_line(uint8_t id,uint8_t *on) {
 if(phase==ROUTE_DEPOT) {
  assert(id==(side==ROUTE_SIDE_MIRROR?1:6));
  if(fault_case==2) return ALIGN_ERR;
  *on=sensor_reads++<2;
 } else *on=1;
 return ALIGN_OK;
}
csvc_status_t csvc_get_pose(map_point_t *p,float *yaw) { *p=pose;*yaw=180;return CSVC_OK; }
csvc_status_t csvc_set_pose(map_point_t p,float yaw) {
 (void)yaw;pose=p;pose_y=p.y_mm;
 if(phase==ROUTE_DEPOT) { assert(p.y_mm==2144);assert(velocity==0); }
 return CSVC_OK;
}
app_status_t route_go(route_id_t id) {
 phase=id;paths++;printf("ROUTE %u\n",id);
 if(id==ROUTE_DEPOT) {pose.x_mm=403;pose.y_mm=2208;pose_y=2208;}
 if(id==ROUTE_HOME) homes++;
 return APP_OK;
}
#ifndef BASELINE
app_status_t route_red_backup_go(route_id_t id,uint8_t (*guard)(void)) {
 if(!guard()) return APP_ERR;
 if(fault_case==1 && id==ROUTE_DEPOT) return APP_ERR;
 return route_go(id);
}
#endif
app_status_t link_post(chassis_command_type_t type,uint16_t id,uint8_t ok) {
 event_count++;printf("EVENT %u %u %u\n",type,id,ok);
 if(!ok) fail_events++;
 if(type==CHASSIS_CMD_DEPOT_1_READY && ok) {assert(abs(pose.y_mm-2308)<=10);assert(velocity==0);}
 return APP_OK;
}
app_status_t link_wait(mission_command_type_t type,uint16_t *id,uint32_t tmo) {
 (void)tmo;*id=++last_id;printf("WAIT %u\n",type);return APP_OK;
}
void link_handshake(void) {}
uint8_t link_poll(link_hook_t hook,void *ctx,uint32_t tmo) {
 chassis_mission_command_t cmd={0};tick+=tmo?tmo:1;
 if(!hook) {assert(velocity==0);longjmp(finished,1);}
#ifndef BASELINE
 if(hook==backup_stop_hook) {
  if(fault_case==3 && next_command==2 && !stopped) {
   cmd.type=MISSION_CMD_STOP;cmd.request_id=77;
   hook(&cmd,ctx);stopped=1;align_stop();
  }
  return 0;
 }
 if(hook==backup_command_hook) {
  assert(next_command<sizeof commands/sizeof commands[0]);
  cmd.type=commands[next_command++];cmd.request_id=++last_id;
  hook(&cmd,ctx);return 1;
 }
#endif
 if(hook==depot_hook) {
  if(next_command<2) next_command=2;
  assert(next_command<sizeof commands/sizeof commands[0]);
  cmd.type=commands[next_command++];cmd.request_id=++last_id;
  hook(&cmd,ctx);return 1;
 }
 return 0;
}
app_status_t stairs_sweep(uint16_t id) { (void)id;stairs_calls++;return APP_OK; }
csvc_status_t csvc_arc(float v,float r,bool insitu) {(void)v;(void)r;(void)insitu;return CSVC_OK;}
csvc_status_t csvc_init(void) {return CSVC_OK;}
hwt101_status_t hwt101_adp_boot_cfg(void) {return HWT101_OK;}
zdt_status_t system_assembly_init(void) {return ZDT_OK;}
int main(int argc,char **argv) {
 assert(argc==3);unsigned mode=(unsigned)atoi(argv[1]);fault_case=(unsigned)atoi(argv[2]);
 g_app_up=1;phase=99;
 g_mission_side=mode==1?MISSION_COLOR_BLUE:MISSION_COLOR_RED;
 g_mission_run_mode=mode==2?MISSION_RUN_RED_BACKUP:MISSION_RUN_FORMAL;
 if(!setjmp(finished)) {app_task(NULL);assert(0);}
 assert(!motion_after_stop);
 if(mode==2) {
  assert(!stairs_calls);
  if(fault_case) {assert(!homes);assert(paths==(fault_case==2?2U:1U));assert(fail_events==1);}
  else {assert(paths==3 && homes==1 && next_command==6);}
 } else {assert(stairs_calls==1 && paths==6 && homes==1);}
 return 0;
}
