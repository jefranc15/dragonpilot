// Early bringup
#define ENTER_BOOTLOADER_MAGIC 0xdeadbeefU
#define ENTER_SOFTLOADER_MAGIC 0xdeadc0deU
#define BOOT_NORMAL 0xdeadb111U

extern void *g_pfnVectors;
extern uint32_t enter_bootloader_mode;
extern uint32_t f413_enter_bootloader_mode;

void jump_to_bootloader(void) {
  // do enter bootloader
  enter_bootloader_mode = 0;
  void (*bootloader)(void) = (void (*)(void)) (*((uint32_t *)BOOTLOADER_ADDRESS));

  // jump to bootloader
  enable_interrupts();
  bootloader();

  // reset on exit
  enter_bootloader_mode = BOOT_NORMAL;
  NVIC_SystemReset();
}

void early_initialization(void) {
  // Reset global critical depth
  disable_interrupts();
  global_critical_depth = 0;

  // Init register and interrupt tables
  init_registers();

  // Black Panda cross-host compatibility:
  // Dragonpilot/NEOS F4 uses 0x2001FFFC for the boot handoff word, while the
  // LG G8 F413 application uses 0x2003FFFC. The physical F413 has both
  // addresses, so a shared bootstub can accept either without changing either
  // application's RAM/stack layout.
#if defined(BOOTSTUB) && defined(STM32F4)
  if ((f413_enter_bootloader_mode == ENTER_SOFTLOADER_MAGIC) ||
      (f413_enter_bootloader_mode == ENTER_BOOTLOADER_MAGIC)) {
    enter_bootloader_mode = f413_enter_bootloader_mode;
    f413_enter_bootloader_mode = 0U;
  }
#endif

  // after it's been in the bootloader, things are initted differently, so we reset
  if ((enter_bootloader_mode != BOOT_NORMAL) &&
      (enter_bootloader_mode != ENTER_BOOTLOADER_MAGIC) &&
      (enter_bootloader_mode != ENTER_SOFTLOADER_MAGIC)) {
    enter_bootloader_mode = BOOT_NORMAL;
    NVIC_SystemReset();
  }

  // if wrong chip, reboot
  volatile unsigned int id = DBGMCU->IDCODE;
  if ((id & 0xFFFU) != MCU_IDCODE) {
    enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
  }

  // setup interrupt table
  SCB->VTOR = (uint32_t)&g_pfnVectors;

  // early GPIOs float everything
  early_gpio_float();

  detect_external_debug_serial();
  detect_board_type();

  if (enter_bootloader_mode == ENTER_BOOTLOADER_MAGIC) {
  #ifdef PANDA
    current_board->set_gps_mode(GPS_DISABLED);
  #endif
    current_board->set_led(LED_GREEN, 1);
    jump_to_bootloader();
  }
}
