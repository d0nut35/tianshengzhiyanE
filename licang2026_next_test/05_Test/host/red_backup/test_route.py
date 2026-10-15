"""真实路线/校准算法；用实际地图复验可达性，主机不代表硬件验证。"""
from pathlib import Path
import subprocess,tempfile
ROOT=Path(__file__).resolve().parents[3];REPO=ROOT.parent;HERE=Path(__file__).resolve().parent
INCLUDES=['lhy/01_App','lhy/02_Service','lhy/04_Bsp','lhy/04_Bsp/hwt101','lhy/99_Utils','lhy/03_Middleware/map','Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2']
with tempfile.TemporaryDirectory(prefix='licang-backup-route-') as d:
 p=Path(d);bins=[]
 cfg=(ROOT/'01_App/mission/mission_red_backup_config.h').read_text(encoding='utf-8')
 cfg=cfg.replace('525','535').replace('2276','2286').replace('(-0.32f)','(-0.62f)').replace('1215','1225')
 (p/'perturbed.h').write_text(cfg,encoding='utf-8')
 for ref in ['91b53e9',None,'perturbed']:
  for rel,name in [('lhy/01_App/app_route.c','route_under_test.inc'),('lhy/02_Service/chassis_align.c','align_under_test.inc')]:
   s=subprocess.check_output(['git','show',ref+':licang2026_next_test/'+rel],cwd=REPO).decode() if ref=='91b53e9' else (ROOT/rel).read_text(encoding='utf-8')
   s=s.replace('../../01_App/mission/',(ROOT/'01_App/mission').as_posix()+'/')
   if ref=='perturbed': s=s.replace((ROOT/'01_App/mission/mission_red_backup_config.h').as_posix(),(p/'perturbed.h').as_posix())
   (p/name).write_text(s,encoding='utf-8')
  binary=p/('perturbed.exe' if ref=='perturbed' else 'baseline.exe' if ref else 'current.exe');cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-I',str(p)]
  for inc in INCLUDES:cmd+=['-I',str(ROOT/inc)]
  if ref=='91b53e9':cmd+=['-DBASELINE','-Wno-unused-function']
  subprocess.run(cmd+[str(HERE/'test_route.c'),'-o',str(binary)],check=True);bins.append(binary)
 for side in [0,1]:
  outputs=[subprocess.check_output([str(x),str(side)]) for x in bins]
  assert outputs[0]==outputs[1]==outputs[2],outputs
  print('PASS formal route/pose parameters unchanged, including perturbed backup parameters:', 'red' if side==0 else 'blue')
 subprocess.run([str(bins[1]),'2'],check=True)
 # 原地图/障碍/膨胀/A*不打替身。
 probe='''#include <assert.h>
#include <stdio.h>
#include "map_adaption.h"
int main(void){map_point_t p[512];uint16_t n;int xy[][2]={{473,4300},{525,4300},{453,4280},{493,4320}};
assert(map_adp_init()==MAP_OK);for(unsigned j=0;j<4;j++){assert(map_adp_find_path(xy[j][0],xy[j][1],440,2276,p,512,&n)==MAP_OK);printf("MAP (%d,%d):",xy[j][0],xy[j][1]);for(unsigned i=0;i<n;i++)printf(" (%d,%d)",p[i].x_mm,p[i].y_mm);puts("");}return 0;}'''
 (p/'map_probe.c').write_text(probe,encoding='utf-8')
 m=ROOT/'lhy/03_Middleware/map';binary=p/'map.exe'
 subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-I',str(m),str(p/'map_probe.c'),str(m/'map.c'),str(m/'map_adaption.c'),'-lm','-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
