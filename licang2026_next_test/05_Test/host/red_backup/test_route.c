/* 实际路线与找线算法：正式模式基线对照，备用回调注入STOP/超时。 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "route_under_test.inc"
#include "align_under_test.inc"
static uint32_t tick;
static float vx_now, vy_now, wz_now, imu_yaw;
static uint8_t *nav_done;
static unsigned timeout_case;
static unsigned guard_calls, fail_at, aborted, after_abort, quiet;
static uint8_t guard(void) {
 if(++guard_calls==fail_at || (timeout_case && tick>=30000U)) aborted=1;
 return !aborted;
}
uint32_t osKernelGetTickCount(void) {return tick;}
uint32_t osKernelGetTickFreq(void) {return 1000U;}
osStatus_t osDelay(uint32_t n) {
 tick+=n;imu_yaw=util_ang_norm(imu_yaw+wz_now*n/1000.0f);
 if(nav_done && timeout_case!=1) {*nav_done=1;nav_done=NULL;}
 return osOK;
}
csvc_status_t csvc_nav(uint8_t mode,map_point_t p,float yaw,float v,float w,uint8_t *fin) {
 if(!quiet) printf("NAV %u %d %d %.2f %.2f %.2f\n",mode,p.x_mm,p.y_mm,yaw,v,w);
 if(aborted) after_abort++;
 *fin=0;nav_done=fin;return CSVC_OK;
}
csvc_status_t csvc_free(float vx,float vy,float wz) {
 if(aborted && (vx||vy||wz)) after_abort++;
 vx_now=vx;vy_now=vy;wz_now=wz;
 if(nav_done) {*nav_done=1;nav_done=NULL;}
 return CSVC_OK;
}
csvc_status_t csvc_set_pose(map_point_t p,float yaw) {
 if(aborted) after_abort++;
 if(!quiet) printf("POSE %d %d %.2f\n",p.x_mm,p.y_mm,yaw);
 return CSVC_OK;
}
csvc_status_t csvc_load_field(uint8_t mirror) {(void)mirror;return CSVC_OK;}
lsensor_level_t lsh_get_level(lsensor_id_t id) {
 (void)id;
 if(timeout_case==2) return LSENSOR_LEVEL_HIGH;
 return wz_now!=0 ? LSENSOR_LEVEL_HIGH : LSENSOR_LEVEL_LOW;
}
hwt101_status_t hwt101_adp_read(float *gyro,float *yaw) {*gyro=wz_now;*yaw=imu_yaw;return HWT101_OK;}
hwt101_status_t hwt101_adp_set_yaw(float yaw) {imu_yaw=yaw;return HWT101_OK;}
int main(int argc,char **argv) {
 assert(argc==2);unsigned mode=atoi(argv[1]);
 if(mode<2) {
  assert(route_set_side(mode?ROUTE_SIDE_MIRROR:ROUTE_SIDE_DEFAULT)==APP_OK);
  for(unsigned i=0;i<ROUTE_NUM;i++) assert(route_go((route_id_t)i)==APP_OK);
 } else {
#ifndef BASELINE
  unsigned checks=0;
  const route_id_t ids[]={ROUTE_PLATFORM,ROUTE_DEPOT,ROUTE_HOME};
  assert(route_set_side(ROUTE_SIDE_DEFAULT)==APP_OK);
  for(unsigned i=0;i<3;i++) {
   fail_at=guard_calls=aborted=0;
   assert(route_red_backup_go(ids[i],guard)==APP_OK);
   unsigned total=guard_calls;
   quiet=1;
   for(unsigned f=1;f<=total;f++) {
    fail_at=f;guard_calls=aborted=after_abort=0;imu_yaw=0;
    assert(route_red_backup_go(ids[i],guard)==APP_ERR);
    assert(aborted && !after_abort && !vx_now && !vy_now && !wz_now && !g_guard);
    checks++;
   }
   quiet=0;
  }
  for(timeout_case=1;timeout_case<=2;timeout_case++) {
   tick=guard_calls=fail_at=aborted=after_abort=0;
   assert(route_red_backup_go(ROUTE_DEPOT,guard)==APP_ERR);
   assert(tick>=30000 && !after_abort && !g_guard && !vx_now && !vy_now && !wz_now);
  }
  timeout_case=0;
  fail_at=guard_calls=aborted=0;
  assert(route_set_side(ROUTE_SIDE_MIRROR)==APP_OK);
  assert(route_red_backup_go(ROUTE_DEPOT,guard)==APP_ERR && !guard_calls);
  printf("PASS backup guard injection %u cases; no movement after abort; guard released\n",checks);
#endif
 }
 return 0;
}
