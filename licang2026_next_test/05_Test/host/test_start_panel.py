"""Compile actual Mission startup functions with fake GPIO/ticks (no hardware)."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / '01_App/mission/mission_app.c').read_text(encoding='utf-8')
panel = source[source.index('static struct {\n    bool base_ready'):source.index('\n#endif\n\nstatic void mission_model_process')]
begin = source.index('static void mission_try_ready(mission_context_t *ctx)\n{')
end = source.index('\n}\n', begin) + 3
ready = source[begin:end]
stub = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define MISSION_CHASSIS_ROUTE_TEST_ENABLED 0
#define MISSION_BUTTON_START_DELAY_MS 1000U
#define MISSION_OPERATION_TIMEOUT_MS 30000U
#define GPIO_PIN_11 (1U<<11)
#define GPIO_PIN_12 (1U<<12)
#define GPIO_PIN_13 (1U<<13)
#define GPIO_PIN_15 (1U<<15)
enum { MISSION_RUN_FORMAL, MISSION_RUN_RED_BACKUP };
static unsigned g_mission_run_mode;
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define DEPOT_TRACE(...) ((void)0)
enum { MISSION_STATE_WAIT_HOME, MISSION_STATE_WAIT_CHASSIS_READY, MISSION_STATE_READY, FAULT };
enum { MISSION_COLOR_NONE, MISSION_COLOR_RED, MISSION_COLOR_BLUE };
enum { MISSION_FAULT_NONE, MISSION_FAULT_QUEUE };
#define MISSION_CMD_MISSION_READY 1
typedef struct { unsigned state, fault_code, request_id; bool arm_home_ready, block_model_ready, chassis_ready; uint32_t deadline_tick; } mission_context_t;
static struct {uint32_t IDR;} port;
#define GPIOD (&port)
#define GPIOE 0
static uint32_t tick;
static unsigned g_mission_side, led_ready, led_blue, writes, sends;
static bool send_ok = true;
static uint32_t osKernelGetTickCount(void) { return tick; }
static uint32_t mission_ms_to_ticks(uint32_t ms) { return ms; }
static void HAL_GPIO_WritePin(unsigned p, unsigned pin, unsigned val) {
 (void)p; ++writes; if(pin==GPIO_PIN_11) led_ready=val; else led_blue=val;
}
static void mission_enter_state(mission_context_t *c,unsigned state,uint32_t ms) { c->state=state; c->deadline_tick=ms?tick+ms:0; }
static bool mission_send_chassis(unsigned type,unsigned id) { (void)type;(void)id;++sends;return send_ok; }
static void mission_fail(mission_context_t *c,unsigned f) { c->fault_code=f;c->state=FAULT; }
'''
tests = r'''
static mission_context_t c;
static void reset(void) {
 memset(&c,0,sizeof c);memset(&g_start,0,sizeof g_start);
 g_start.stable=GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;g_mission_run_mode=MISSION_RUN_FORMAL;
 c.state=MISSION_STATE_WAIT_CHASSIS_READY;c.request_id=1;
 tick=0;port.IDR=GPIO_PIN_12|GPIO_PIN_13;g_mission_side=0;sends=writes=0;send_ok=true;
}
static void sample(unsigned keys,unsigned dt) { port.IDR=keys|GPIO_PIN_15;tick+=dt;mission_start_panel(&c); }
static void exact(unsigned keys,unsigned dt) { port.IDR=keys;tick+=dt;mission_start_panel(&c); }
static void make_ready(void) { c.arm_home_ready=c.block_model_ready=true;g_start.base_ready=true; }
static void press(void) { sample(port.IDR & ~GPIO_PIN_13,1); sample(port.IDR,50); }
int main(void) {
 reset(); sample(port.IDR,0);sample(port.IDR,50);press(); assert(!g_start.confirmed && led_ready==1);
 make_ready(); sample(port.IDR,100);assert(led_ready==0 && !g_start.confirmed); /* held early */
 sample(GPIO_PIN_12|GPIO_PIN_13,1);sample(port.IDR,50);press();
 assert(g_start.confirmed && g_mission_side==MISSION_COLOR_RED && led_blue==1);
 mission_try_ready(&c);assert(!sends); /* no final handshake */
 c.chassis_ready=true;mission_try_ready(&c);
 assert(sends==1 && c.state==MISSION_STATE_READY && c.deadline_tick-g_start.confirmed_tick>=1000);
 mission_try_ready(&c);assert(sends==1);
 sample(0,100); assert(g_mission_side==MISSION_COLOR_RED); /* color locked */
 g_start.departed=true;unsigned old=writes;sample(GPIO_PIN_13,100);assert(writes==old);

 reset();make_ready();sample(GPIO_PIN_13,0);sample(GPIO_PIN_13,50);
 assert(led_blue==0 && !g_start.confirmed);press();assert(g_mission_side==MISSION_COLOR_BLUE);
 tick+=1500;c.chassis_ready=true;mission_try_ready(&c);assert(c.deadline_tick==tick+1); /* slow handshake */

 reset();make_ready();sample(port.IDR,0);sample(port.IDR,50);
 sample(GPIO_PIN_12,1);sample(GPIO_PIN_12|GPIO_PIN_13,20);sample(port.IDR,50);
 assert(!g_start.confirmed); /* bounce */
 c.block_model_ready=false;press();assert(!g_start.confirmed && led_ready==1);
 reset();make_ready();c.arm_home_ready=false;sample(port.IDR,0);sample(port.IDR,50);press();assert(!g_start.confirmed);
 reset();make_ready();g_start.base_ready=false;sample(port.IDR,0);sample(port.IDR,50);press();assert(!g_start.confirmed);
 reset();make_ready();c.state=FAULT;c.fault_code=1;sample(port.IDR,0);sample(port.IDR,50);press();assert(!g_start.confirmed);

 reset();make_ready();sample(port.IDR,0);sample(port.IDR,50);press();
 c.chassis_ready=true;send_ok=false;mission_try_ready(&c);assert(c.state==FAULT);
 mission_start_panel(&c);assert(led_ready==1);
 /* Red backup is sampled and locked with color; later release cannot change it. */
 reset();make_ready();exact(GPIO_PIN_12|GPIO_PIN_13,0);exact(port.IDR,50);
 exact(GPIO_PIN_12,1);exact(port.IDR,50);
 assert(g_start.confirmed && g_mission_side==MISSION_COLOR_RED && g_mission_run_mode==MISSION_RUN_RED_BACKUP);
 exact(GPIO_PIN_15|GPIO_PIN_13,100);assert(g_mission_run_mode==MISSION_RUN_RED_BACKUP && g_mission_side==MISSION_COLOR_RED);
 /* Blue+backup must reject; releasing PD15 while holding KEY1 must not start. */
 reset();make_ready();exact(GPIO_PIN_13,0);exact(port.IDR,50);
 exact(0,1);exact(0,50);assert(!g_start.confirmed && g_mission_side==MISSION_COLOR_NONE);
 exact(GPIO_PIN_15,1);exact(port.IDR,50);assert(!g_start.confirmed);
 exact(GPIO_PIN_13|GPIO_PIN_15,1);exact(port.IDR,50);press();
 assert(g_start.confirmed && g_mission_side==MISSION_COLOR_BLUE && g_mission_run_mode==MISSION_RUN_FORMAL);
 /* PD15 bounce cancels a simultaneous KEY1 edge until all inputs are stable. */
 reset();make_ready();sample(GPIO_PIN_12|GPIO_PIN_13,0);sample(port.IDR,50);
 exact(GPIO_PIN_12,1);exact(GPIO_PIN_12|GPIO_PIN_15,20);exact(port.IDR,20);assert(!g_start.confirmed);
 exact(port.IDR,30);assert(g_start.confirmed && g_mission_run_mode==MISSION_RUN_FORMAL);
 puts("PASS: readiness, early-held key, debounce, red/blue, lock, handshake timing, fault and departure");
}
'''
with tempfile.TemporaryDirectory(prefix='licang-start-') as directory:
    path = Path(directory)
    (path / 'test.c').write_text(stub + panel + ready + tests, encoding='utf-8')
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(path / 'test.c'), '-o', str(path / 'test.exe')], check=True)
    subprocess.run([str(path / 'test.exe')], check=True)
