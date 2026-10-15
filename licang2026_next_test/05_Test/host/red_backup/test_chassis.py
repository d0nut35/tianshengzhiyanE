"""真实底盘编排：正式模式与91b53e9逐项对照；备用阶段/故障单独验证。"""
from pathlib import Path
import subprocess, tempfile
ROOT=Path(__file__).resolve().parents[3]
REPO=ROOT.parent
HERE=Path(__file__).resolve().parent
INCLUDES=['05_Test/host/stairs/fakes','lhy/01_App','lhy/02_Service','lhy/04_Bsp','lhy/04_Bsp/zdt_motor','lhy/04_Bsp/hwt101','lhy/99_Utils','lhy/03_Middleware/map','02_Service/chassis_bridge','Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2']
def source_at(ref=None):
 rel='licang2026_next_test/lhy/01_App/app_main.c'
 s=subprocess.check_output(['git','show',ref+':'+rel],cwd=REPO).decode() if ref else (ROOT/'lhy/01_App/app_main.c').read_text(encoding='utf-8')
 # 放到临时目录后，原相对Mission include需指向真实头文件。
 return s.replace('../../01_App/mission/',(ROOT/'01_App/mission').as_posix()+'/')
with tempfile.TemporaryDirectory(prefix='licang-backup-') as d:
 p=Path(d);binaries=[]
 cfg=(ROOT/'01_App/mission/mission_red_backup_config.h').read_text(encoding='utf-8')
 cfg=cfg.replace('2308','2338').replace('2144','2154').replace('(-100.0f)','(-80.0f)')
 (p/'perturbed.h').write_text(cfg,encoding='utf-8')
 for ref in ['91b53e9',None,'perturbed']:
  s=source_at(None if ref=='perturbed' else ref)
  if ref=='perturbed': s=s.replace((ROOT/'01_App/mission/mission_red_backup_config.h').as_posix(),(p/'perturbed.h').as_posix())
  (p/'app_main_under_test.inc').write_text(s,encoding='utf-8')
  binary=p/('perturbed.exe' if ref=='perturbed' else 'baseline.exe' if ref else 'current.exe')
  cmd=['gcc','-std=c11','-Wall','-Wextra','-Werror','-ffunction-sections','-fdata-sections','-I',str(p)]
  for inc in INCLUDES:cmd+=['-I',str(ROOT/inc)]
  if ref=='91b53e9':cmd+=['-DBASELINE']
  subprocess.run(cmd+[str(HERE/'test_chassis.c'),'-Wl,--gc-sections','-o',str(binary)],check=True)
  binaries.append(binary)
 for side in [0,1]:
  outputs=[subprocess.check_output([str(b),str(side),'0']) for b in binaries]
  assert outputs[0]==outputs[1]==outputs[2],outputs
  print('PASS formal chassis trace unchanged, including perturbed backup parameters:', 'red' if side==0 else 'blue')
 for fault in range(4):
  subprocess.run([str(binaries[1]),'2',str(fault)],check=True,stdout=subprocess.DEVNULL)
 print('PASS backup: platform/depot/home, navigation failure, calibration failure, STOP; no stairs/cylinder')
