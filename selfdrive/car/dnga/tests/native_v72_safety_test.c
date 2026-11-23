// Host C exercise for exact V7.2 BENCH-ONLY safety prototype.
// A green test proves V7.2 stays TX-locked, not that steering is road-safe.
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  uint32_t addr;
  int bus;
  int len;
  uint8_t data[8];
} CANPacket_t;
#define GET_ADDR(m) ((m)->addr)
#define GET_BUS(m) ((m)->bus)
#define GET_LEN(m) ((m)->len)
#define GET_BYTE(m,b) ((m)->data[b])
#define UNUSED(x) ((void)(x))

typedef struct {
  const int addr, bus, len;
  const bool check_checksum;
  const uint8_t max_counter;
  const uint32_t expected_timestep;
} CanMsgCheck;
typedef struct {
  const CanMsgCheck msg[3];
  bool msg_seen;
  int index;
  bool valid_checksum;
  int wrong_counters;
  uint8_t last_counter;
  uint32_t last_timestamp;
  bool lagging;
} AddrCheckStruct;
typedef struct { AddrCheckStruct *check; int len; } addr_checks;
typedef const addr_checks *(*safety_hook_init)(int16_t);
typedef int (*rx_hook)(CANPacket_t *);
typedef int (*tx_hook)(CANPacket_t *);
typedef int (*tx_lin_hook)(int,uint8_t *,int);
typedef int (*fwd_hook)(int,CANPacket_t *);
typedef struct {
  safety_hook_init init;
  rx_hook rx;
  tx_hook tx;
  tx_lin_hook tx_lin;
  fwd_hook fwd;
} safety_hooks;

static bool controls_allowed = false;
static uint32_t mock_now = 5000000U;
static uint32_t microsecond_timer_get(void) { return mock_now; }
static int nooutput_tx_lin_hook(int bus,uint8_t *data,int len) {
  UNUSED(bus);UNUSED(data);UNUSED(len);return 0;
}
static bool addr_safety_check(CANPacket_t *m,const addr_checks *checks,
                              uint8_t (*get)(CANPacket_t *),
                              uint8_t (*calc)(CANPacket_t *),
                              uint8_t (*counter)(CANPacket_t *)) {
  UNUSED(counter);
  return checks->len == 1 && checks->check[0].msg[0].addr == (int)m->addr &&
    checks->check[0].msg[0].bus == m->bus &&
    checks->check[0].msg[0].len == m->len && get(m) == calc(m);
}

#include "panda/board/safety/safety_dnga_v72.h"

static int failed = 0;
static int count = 0;
static void check(const char *label, bool ok) {
  count++;
  printf("%s %s\n", ok ? "PASS" : "FAIL", label);
  if (!ok) failed++;
}
static CANPacket_t msg(uint32_t addr,int bus,int len) {
  CANPacket_t m={0};
  m.addr=addr;m.bus=bus;m.len=len;
  return m;
}
static void checksum(CANPacket_t *m) {
  int sum=(int)m->addr + 7 + 1 + (m->addr==0x1D0U ? 1 : 2);
  for(int i=0;i<7;i++)sum+=m->data[i];
  m->data[7]=(uint8_t)(sum&255);
}
static CANPacket_t steer(int cmd, int cnt, bool request) {
  CANPacket_t m=msg(0x1D0U,0,8);
  int raw=cmd&0x7FF;
  m.data[0]=(uint8_t)(raw>>3);
  m.data[1]=(uint8_t)(((raw&7)<<5)|(cnt&15));
  m.data[2]=request ? 0x60U : 0x40U;
  m.data[3]=0x40U;
  checksum(&m);
  return m;
}
static CANPacket_t camera(bool engaged) {
  CANPacket_t m=msg(0x273U,2,8);
  m.data[1]=engaged?0x22U:0x00U;
  m.data[6]=engaged?0x40U:0x00U;
  checksum(&m);
  return m;
}
static void set_dummy_fresh_inputs(void) {
  controls_allowed=true; // host fixture only: never set by actual V7.2 RX
  dnga_v72_cam_known=true;
  dnga_v72_cam_engaged=true;
  dnga_v72_brake_known=true;
  dnga_v72_gas_known=true;
  dnga_v72_driver_known=true;
  dnga_v72_brake=false;
  dnga_v72_gas=false;
  dnga_v72_driver_torque=0;
  dnga_v72_cam_ts=mock_now;
  dnga_v72_brake_ts=mock_now;
  dnga_v72_gas_ts=mock_now;
  dnga_v72_driver_ts=mock_now;
  dnga_v72_last_counter=-1;
  dnga_v72_last_torque=0;
}

int main(void) {
  const addr_checks *checks=dnga_v72_hooks.init(0);
  check("start with controls disallowed",!controls_allowed);
  check("verified camera RX list nonempty",checks!=NULL && checks->len==1);
  check("0x273 has checksum and freshness metadata",
    checks->check[0].msg[0].check_checksum &&
    checks->check[0].msg[0].expected_timestep==50000U);

  CANPacket_t c=camera(true);
  check("camera valid rx",dnga_v72_hooks.rx(&c)==1 && dnga_v72_cam_known);
  check("camera cannot self-authorize steering",!controls_allowed);
  CANPacket_t s=steer(5,0,true);
  check("host TX LOCKED even on valid steer",dnga_v72_hooks.tx(&s)==0);
  check("host 0x271 locked",dnga_v72_hooks.tx(&(CANPacket_t){.addr=0x271U,.bus=0,.len=8})==0);
  check("host 0x273 locked",dnga_v72_hooks.tx(&(CANPacket_t){.addr=0x273U,.bus=0,.len=8})==0);
  check("factory camera braking forwards",dnga_v72_hooks.fwd(2,&(CANPacket_t){.addr=0x271U,.bus=2,.len=8})==0);
  check("factory camera accel forwards",dnga_v72_hooks.fwd(2,&(CANPacket_t){.addr=0x273U,.bus=2,.len=8})==0);
  check("factory camera HUD forwards",dnga_v72_hooks.fwd(2,&(CANPacket_t){.addr=0x274U,.bus=2,.len=8})==0);
  check("factory camera steer forwards in locked mode",dnga_v72_hooks.fwd(2,&(CANPacket_t){.addr=0x1D0U,.bus=2,.len=8})==0);
  check("powertrain to camera forwards",dnga_v72_hooks.fwd(0,&s)==2);
  check("unconnected bus not forwarded",dnga_v72_hooks.fwd(1,&s)==-1);
  check("LIN TX blocked",dnga_v72_hooks.tx_lin(0,NULL,0)==0);

  // Isolated candidate policy: tests cannot unlock actual TX.
  set_dummy_fresh_inputs();
  s=steer(5,0,true);
  check("candidate accepts properly encoded small torque",dnga_v72_steer_candidate_ok(&s));
  check("actual TX still LOCKED after synthetic candidate",dnga_v72_hooks.tx(&s)==0);
  CANPacket_t bad_counter=steer(10,3,true);
  check("counter replay/jump rejected",!dnga_v72_steer_candidate_ok(&bad_counter));
  s=steer(50,1,true);
  check("per-frame torque jump rejected",!dnga_v72_steer_candidate_ok(&s));
  s=steer(15,1,true);
  check("allowed increment within provisional limit",dnga_v72_steer_candidate_ok(&s));
  s=steer(256,2,true);
  check("torque absolute limit rejected",!dnga_v72_steer_candidate_ok(&s));
  s=steer(0,2,true);
  s.data[7]^=0xFFU;
  check("bad steering checksum rejected",!dnga_v72_steer_candidate_ok(&s));
  s=steer(0,2,true);s.len=4;
  check("bad steering length rejected",!dnga_v72_steer_candidate_ok(&s));
  s=steer(10,2,false);
  check("nonzero torque without request rejected",!dnga_v72_steer_candidate_ok(&s));

  set_dummy_fresh_inputs();controls_allowed=false;
  s=steer(5,0,true);
  check("disengaged candidate blocked",!dnga_v72_steer_candidate_ok(&s));
  set_dummy_fresh_inputs();dnga_v72_brake=true;
  check("brake override candidate blocked",!dnga_v72_steer_candidate_ok(&s));
  set_dummy_fresh_inputs();dnga_v72_gas=true;
  check("gas override candidate blocked",!dnga_v72_steer_candidate_ok(&s));
  set_dummy_fresh_inputs();dnga_v72_driver_torque=30;
  check("driver torque candidate blocked",!dnga_v72_steer_candidate_ok(&s));
  set_dummy_fresh_inputs();dnga_v72_cam_ts=mock_now-200000U;
  check("stale camera candidate blocked",!dnga_v72_steer_candidate_ok(&s));
  set_dummy_fresh_inputs();dnga_v72_driver_known=false;
  check("missing driver input blocks",!dnga_v72_steer_candidate_ok(&s));

  dnga_v72_hooks.init(0);
  c=camera(true); c.data[7]^=0x1U;
  check("invalid factory camera checksum rejected",dnga_v72_hooks.rx(&c)==0 && !dnga_v72_cam_known);
  c=camera(false);
  check("camera disengagement accepted",dnga_v72_hooks.rx(&c)==1 && !dnga_v72_cam_engaged && !controls_allowed);
  CANPacket_t brake=msg(0xA1U,0,8);brake.data[0]=0x20;
  set_dummy_fresh_inputs();
  check("brake RX disarms",dnga_v72_hooks.rx(&brake)==1 && !controls_allowed);
  CANPacket_t gas=msg(0x277U,1,8);gas.data[2]=150;
  set_dummy_fresh_inputs();
  check("gas RX disarms",dnga_v72_hooks.rx(&gas)==1 && !controls_allowed);
  CANPacket_t driver=msg(0xA4U,0,8);driver.data[2]=7;driver.data[3]=0x80;
  set_dummy_fresh_inputs();
  check("driver torque RX disarms",dnga_v72_hooks.rx(&driver)==1 && !controls_allowed);

  printf("V72_NATIVE_TESTS=%d FAILURES=%d TX_LOCKED=YES\n",count,failed);
  return failed==0?0:1;
}
