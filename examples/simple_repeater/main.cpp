#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "MyMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[160];

#ifdef POWERSAVING_MODE
  // Sleep time calculated relative to this
  uint32_t last_activity = millis();
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef DISPLAY_CLASS
  #ifdef POWERSAVING_MODE
    if (board.getStartupReason() != BD_STARTUP_RX_PACKET) { // Only display if not waken up from deepsleep
      if (display.begin()) {
        display.startFrame();
        display.setCursor(0, 0);
        display.print("Please wait...");
        display.endFrame();
      }
    }
  #else
    if (display.begin()) {
      display.startFrame();
      display.setCursor(0, 0);
      display.print("Please wait...");
      display.endFrame();
    }
  #endif
#endif

#ifdef POWERSAVING_MODE
  if (board.getStartupReason() == BD_STARTUP_RX_PACKET) {
    MESH_DEBUG_PRINTLN("Waking from sleep");
    the_mesh.restoreFromSleep();
    last_activity = millis();
  } else {
    // On fresh start, don't sleep for at least 30s
    last_activity = millis() + 30000;
    MESH_DEBUG_PRINTLN("Cold booting, last_activity=%d", last_activity);
  }
#endif

  if (!radio_init()) {
    halt();
  }

  fast_rng.begin(radio_get_rng_seed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Repeater ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

#ifdef POWERSAVING_MODE
  // Do not send an Advert after wakeup from deepsleep due to RX
  // Only send in first startup or reset
  if (board.getStartupReason() != BD_STARTUP_RX_PACKET) {
    // send out initial Advertisement to the mesh
    the_mesh.sendSelfAdvertisement(16000);
  }
#else
  // send out initial Advertisement to the mesh
  the_mesh.sendSelfAdvertisement(16000);
#endif
}

void enterSleep() {
  uint32_t duration = the_mesh.prepareForSleep();
  if (duration > 0) {
    // sleep until the next RX packet or when an event was due
    MESH_DEBUG_PRINTLN("Sleeping %d sec", (duration+500)/1000);
    Serial.flush();
    board.enterDeepSleep((duration+500) / 1000);
  } else {
    MESH_DEBUG_PRINTLN("Sleeping until woken by radio");
    Serial.flush();
    board.powerOff();
  }
}

void loop() {
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
#ifdef POWERSAVING_MODE
    // Delay sleep by 2m on serial input (the ESP32 disables its
    // USB serial interface on deep sleep, annoying for humans
    // configuring the radio.)
    last_activity = millis() + 120 * 1000;
#endif
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
      Serial.print(c);
    }
    if (c == '\r') break;
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    Serial.print('\n');
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  #ifdef POWERSAVING_MODE
    if (board.getStartupReason() != BD_STARTUP_RX_PACKET) { // Only display if not waken up from deepsleep
      ui_task.loop();
    }
  #else
    ui_task.loop();
  #endif
#endif
  rtc_clock.tick();

#ifdef POWERSAVING_MODE
  if (last_activity + 5000 < millis()) {
    enterSleep();
  }
#endif
}
