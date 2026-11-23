// Host-side C audit of the EXACT V7.1-R1 Panda hook source.
// Never compiled into production Panda firmware. Does not access USB/CAN.
//
// The definitions below only represent the public interface required by the
// original safety_dnga_stock.h. This test includes that exact unchanged file.
// It proves policy decisions under controlled inputs; NOT real ECU forwarding.
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
#define UNUSED(x) ((void)(x))

typedef struct { int len; } addr_checks;
static const addr_checks dnga_rx_checks = { .len = 0 };
static bool controls_allowed = false;
static bool gas_pressed = false;
static bool brake_pressed = false;

typedef const addr_checks *(*hook_init_t)(int16_t);
typedef int (*hook_rx_t)(CANPacket_t *);
typedef int (*hook_tx_t)(CANPacket_t *);
typedef int (*hook_tx_lin_t)(int, uint8_t *, int);
typedef int (*hook_fwd_t)(int, CANPacket_t *);
typedef struct {
  hook_init_t init;
  hook_rx_t rx;
  hook_tx_t tx;
  hook_tx_lin_t tx_lin;
  hook_fwd_t fwd;
} safety_hooks;

static int nooutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num); UNUSED(data); UNUSED(len);
  return 0;
}

#include "panda/board/safety/safety_dnga_stock.h"

static int tests = 0;
static int defects = 0;
static int proved_gaps = 0;

static void expect_policy(const char *name, bool ok) {
  tests++;
  if (!ok) {
    defects++;
    printf("POLICY_FAIL %s\n", name);
  } else {
    printf("POLICY_PASS %s\n", name);
  }
}

// Prove a missing safety protection by supplying an inappropriate input.
// Passing this test demonstrates a KNOWN SAFETY GAP, not road readiness.
static void expect_known_gap(const char *name, bool gap_is_present) {
  tests++;
  if (gap_is_present) {
    proved_gaps++;
    printf("UNSAFE_CONFIRMED %s\n", name);
  } else {
    printf("GAP_NOT_REPRODUCED %s (audit contract needs review)\n", name);
    defects++;
  }
}

static CANPacket_t msg(uint32_t addr, int bus, int len) {
  CANPacket_t m;
  memset(&m, 0, sizeof(m));
  m.addr = addr;
  m.bus = bus;
  m.len = len;
  return m;
}

int main(void) {
  const addr_checks *rx = dnga_stock_hooks.init(1);
  expect_policy("safety mode initialized", rx != NULL);

  CANPacket_t steer = msg(0x1D0U, 0, 8);
  CANPacket_t wrong_bus = msg(0x1D0U, 2, 8);
  CANPacket_t stock_brake = msg(0x271U, 0, 8);
  CANPacket_t stock_acc = msg(0x273U, 0, 8);
  CANPacket_t stock_hud = msg(0x274U, 0, 8);
  CANPacket_t other = msg(0x275U, 0, 8);

  expect_policy("steering 0x1D0 bus0 allowed", dnga_stock_hooks.tx(&steer) == 1);
  expect_policy("bus2 steering TX blocked", dnga_stock_hooks.tx(&wrong_bus) == 0);
  expect_policy("OP brake 0x271 TX blocked", dnga_stock_hooks.tx(&stock_brake) == 0);
  expect_policy("OP ACC 0x273 TX blocked", dnga_stock_hooks.tx(&stock_acc) == 0);
  expect_policy("OP HUD 0x274 TX blocked", dnga_stock_hooks.tx(&stock_hud) == 0);
  expect_policy("other OP ID 0x275 TX blocked", dnga_stock_hooks.tx(&other) == 0);
  CANPacket_t source271 = msg(0x271U, 2, 8);
  CANPacket_t source273 = msg(0x273U, 2, 8);
  CANPacket_t source274 = msg(0x274U, 2, 8);
  CANPacket_t source1d0 = msg(0x1D0U, 2, 8);
  expect_policy("camera brake 0x271 bus2->0", dnga_stock_hooks.fwd(2, &source271) == 0);
  expect_policy("camera ACC 0x273 bus2->0", dnga_stock_hooks.fwd(2, &source273) == 0);
  expect_policy("camera HUD 0x274 bus2->0", dnga_stock_hooks.fwd(2, &source274) == 0);
  expect_policy("camera steer 0x1D0 blocked", dnga_stock_hooks.fwd(2, &source1d0) == -1);
  expect_policy("powertrain->camera bus0->2", dnga_stock_hooks.fwd(0, &other) == 2);
  expect_policy("no bus1 forwarding", dnga_stock_hooks.fwd(1, &other) == -1);
  expect_policy("LIN transmission blocked", dnga_stock_hooks.tx_lin(0, NULL, 0) == 0);

  // All of these are unsafe behaviors in the *current* implementation.
  expect_known_gap("controls_allowed forced on during init", controls_allowed);
  expect_known_gap("RX monitored-message list is empty", rx->len == 0);

  CANPacket_t zero_len_steer = msg(0x1D0U, 0, 0);
  expect_known_gap("malformed DLC steering TX accepted", dnga_stock_hooks.tx(&zero_len_steer) == 1);

  // All-zero 0x1D0 bytes cannot have the expected additive steering checksum
  // (for 7 zero data bytes, checksum is nonzero).
  CANPacket_t bad_checksum = msg(0x1D0U, 0, 8);
  expect_known_gap("bad steering checksum accepted", dnga_stock_hooks.tx(&bad_checksum) == 1);

  CANPacket_t arbitrary_steer = msg(0x1D0U, 0, 8);
  memset(arbitrary_steer.data, 0xFF, sizeof(arbitrary_steer.data));
  expect_known_gap("arbitrary steering payload accepted", dnga_stock_hooks.tx(&arbitrary_steer) == 1);

  controls_allowed = false;
  expect_known_gap("steer TX accepted when controls_allowed false", dnga_stock_hooks.tx(&steer) == 1);
  controls_allowed = true;

  gas_pressed = true;
  brake_pressed = true;
  expect_known_gap("steer TX allowed under gas+brake override", dnga_stock_hooks.tx(&steer) == 1);
  CANPacket_t bad_rx = msg(0xFFFFFFFFU, 0, 0);
  expect_known_gap("unknown RX packet accepted", dnga_stock_hooks.rx(&bad_rx) == 1);
  expect_known_gap("RX never clears controls_allowed on override", controls_allowed == true);

  printf("AUDIT_TESTS=%d POLICY_DEFECTS=%d UNSAFE_CONFIRMED=%d\n",
         tests, defects, proved_gaps);
  if (defects > 0 || proved_gaps < 8) {
    puts("AUDIT_INCOMPLETE");
    return 1;
  }
  puts("AUDIT_COMPLETE_RESULT=NOT_ROAD_QUALIFIED");
  return 0;
}
