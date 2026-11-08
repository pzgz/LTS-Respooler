// ---------------- LTS RESPOOLER DRIVER BOARD CODE -----------------

#include <TMCStepper.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <math.h>
#include <string>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <stdarg.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_coexist.h"
#include <driver/ledc.h>
#include "esp_private/periph_ctrl.h"
#include "driver/gpio.h"
#include <freertos/semphr.h>
#include "soc/ledc_struct.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/rtc.h"
#include "hal/wdt_hal.h"
#include "hal/brownout_ll.h"
#include "esp_rtc_time.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "esp_rom_crc.h"
#include "esp_attr.h"

// ------------------------ Board Info ----------------------------
#define FIRMWARE_VERSION "1.3.0"  // reported to the app; kept so the app treats this build like 1.3.0
#define FIRMWARE_BUILD "1.3.0-wd8"  // printed on the serial port and reported as FW_B
#define BOARD_NAME "LTS DB"

// ------------------ Robustness / power settings ------------------
// See the "Watchdog, auto-resume and diagnostics" section further down.
// CPU and radio run at the stock values. The lower settings were a stop-gap while the 5 V rail was weak;
// with D1 bridged, the extra capacitors and a separate servo supply they are no longer needed.
#define CPU_FREQ_MHZ 240              // stock: 240 (160 draws a little less)
#define BLE_TX_POWER_DBM 6            // stock: +6 dBm
#define WIFI_TX_LIMIT 0               // 1 = cap the Wi-Fi transmit power at WIFI_TX_POWER (stock: no cap)
#define WIFI_TX_POWER WIFI_POWER_11dBm
#define WIFI_OFF_WHILE_WINDING 1      // 1 = Wi-Fi off while winding, back on 5 s after stopping (stock: 0).
                                      // The ESP32 shares one radio between Wi-Fi and BLE; with both on, the BLE
                                      // link dropped during winding (5 s without a packet at -61 dBm).

// Wireless firmware update pushed from a computer (ArduinoOTA / espota, PlatformIO env "ota").
// The password comes from local_ota.ini (git-ignored) through a build flag; empty = no password.
#define OTA_HOSTNAME "lts-respooler"  // -> lts-respooler.local
#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif
#define BROWNOUT_LEVEL 7              // 0..7 = 2.43/2.48/2.58/2.62/2.67/2.70/2.77/2.80 V (stock: 0)
#define RWDT_TIMEOUT_MS 6000          // no healthy loop() pass for this long -> system reset
#define RWDT_STAGE1_MS 4000           // still stuck this much later -> full RTC reset (like pressing EN)
#define HEALTH_CHECK_INTERVAL_MS 1000
#define HEARTBEAT_INTERVAL_MS 1000
#define RUNSTATE_SAVE_INTERVAL_MS 250
#define AUTO_RESUME_DELAY_MS 4000     // wait after an abnormal reset before the motor restarts
#define AUTO_RESUME_MAX_QUICK 3       // more quick crashes than this in a row -> stay paused
#define AUTO_RESUME_STABLE_MS 60000   // winding this long after a resume counts as stable again
#define BOOTLOOP_WINDOW_MS 30000
#define BOOTLOOP_LIMIT 4              // this many abnormal boots within the window -> one full RTC reset

// ------------------------ Diagnostic log ------------------------
// Every '#' line printed on the serial port also goes into a RAM ring buffer with an uptime stamp, so it
// can be read over Wi-Fi without a USB cable: http://lts-respooler.local/log (#HB only every 10 s).
#define DLOG_SIZE 8192
static char dlogBuf[DLOG_SIZE];
static size_t dlogHead = 0;
static bool dlogWrapped = false;
static portMUX_TYPE dlogMux = portMUX_INITIALIZER_UNLOCKED;

static void dlogAppend(const char* s, size_t n) {
  portENTER_CRITICAL(&dlogMux);
  for (size_t i = 0; i < n; i++) {
    dlogBuf[dlogHead++] = s[i];
    if (dlogHead >= DLOG_SIZE) {
      dlogHead = 0;
      dlogWrapped = true;
    }
  }
  portEXIT_CRITICAL(&dlogMux);
}

static void dlogv(bool keep, const char* fmt, va_list ap) {
  char line[256];
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  if (n <= 0) return;
  if (n >= (int)sizeof(line)) n = sizeof(line) - 1;
  Serial.write((const uint8_t*)line, n);
  if (!keep) return;
  char ts[16];
  unsigned long ms = millis();
  int tn = snprintf(ts, sizeof(ts), "%5lu.%01lu ", ms / 1000UL, (ms / 100UL) % 10UL);
  dlogAppend(ts, tn);
  dlogAppend(line, n);
}

static void dlogf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  dlogv(true, fmt, ap);
  va_end(ap);
}

static void dlogfKeep(bool keep, const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  dlogv(keep, fmt, ap);
  va_end(ap);
}

static void dlogln(const char* s) {
  dlogf("%s\n", s);
}

// Link statistics, shown at the top of /log
static volatile uint32_t bleDisconnects = 0, wifiDisconnects = 0;
static volatile int bleLastReason = 0, wifiLastReason = 0;
static volatile uint16_t bleConnItvl = 0, bleConnLatency = 0, bleConnTimeout = 0;

// --------------------- Hardware Pin Defines ---------------------
#define TMC_UART_RX 21
#define TMC_UART_TX 22
#define STEP_PIN 14
#define DIR_PIN 27
#define EN_PIN 13
#define LED_PIN 19
#define LED_CONN_PIN 2
#define FILAMENT_PIN 33
#define BUTTON_PIN 25
#define FAN_PIN 32
#define SERVO_PIN 26

// ------------------------ Motor Parameters ----------------------
#define MICROSTEPPING 8
#define R_SENSE 0.11f
#define HIGH_SPEED_FACTOR 1.4

// -------------------- Timing and Limits --------------------------
#define START_INTERVAL_US 700
#define DEFAULT_INTERVAL_US 116
#define TORQUE_CHECK_INTERVAL_MS 50
#define BUTTON_DEBOUNCE_MS 50
#define ACCEL_UPDATE_INTERVAL 20
#define ACCEL_STEP 5
#define LED_CONN_PULSE_MAX 60
#define STATUS_NOTIFY_INTERVAL 400
#define LED_BLINK_INTERVAL 1000
#define TORQUE_SG_IGNORE 20
#define TORQUE_SG_LIMIT_LOW 0.72
#define TORQUE_SG_LIMIT_MED 0.85
#define TORQUE_SG_LIMIT_HIGH 0.97
#define TORQUE_BELOW_MS 700
#define MIN_RSSI_THRESHOLD -90
#define FILAMENT_LOSS_CONFIRM_MS 100
#define STEP_DUTY_ON 512

#define TARGET_WEIGHT_FACTOR_2 0.59f
#define TARGET_WEIGHT_FACTOR_3 0.34f

#define FAN_CHANNEL LEDC_CHANNEL_1
#define FAN_TIMER LEDC_TIMER_1

#define STEP_LEDC_CHANNEL LEDC_CHANNEL_3
#define STEP_LEDC_TIMER LEDC_TIMER_2

#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_0
#define SERVO_LEDC_TIMER LEDC_TIMER_3

// ------------------- BLE UUIDs -----------------------------------
#define SERVICE_UUID "9E05D06D-68A7-4E1F-A503-AE26713AC101"
#define CHARACTERISTIC_UUID "7CB2F1B4-7E3F-43D2-8C92-DF58C9A7B1A8"

// ------------------- Direct settings variables -------------------
int speedPercent = 85;                        // 50-100
int motorDirection = 0;                       // 0, 1
int ledBrightness = 50;                       // 0-100
bool useFilamentSensor = true;                // true, false
int motorStrength = 100;                      // 80-120
int targetWeight = 0;                         // 0-3
int torqueLimit = 0;                          // 0-3
int jingleStyle = 0;                          // 0-3
unsigned long calibrationAt80Speed = 895000;  // ms
String wifiSSID = "";                         // string
String wifiPassword = "";                     // string
int fanSpeed = 60;                            // 0–100
bool fanAlwaysOn = false;                     // true, false
unsigned long fanStopAfter = 0;

int servoAngleR = 5;        // deg
int servoAngleL = 175;      // deg
float servoStepMm = 1.75f;  // mm the guide advances per spool revolution (normally the filament diameter)
bool servoHomeIsR = true;   // true, false

// Guide travel per servo degree. The stock firmware assumes a 35-tooth module-1.2 pinion driven 1:1 by
// a servo that turns exactly 180 deg over 500-2500 us. Real servos differ by several percent, so this is
// a per-machine calibration (SV_MMD): measured guide travel between the two end stops / (SV_L - SV_R).
#define SERVO_MM_PER_DEG_NOMINAL (35.0f * 3.77f / 360.0f)
float servoMmPerDeg = SERVO_MM_PER_DEG_NOMINAL;
float spoolWidthMm = 0.0f;  // inner width of the current spool (SV_W), 0 = not set
// Spool revolutions the guide stays at an end stop before turning back (SV_DW). Without it the turn
// next to each flange gets only half the filament of every other position (the guide passes it once
// per round trip instead of twice), so the edges fall behind and the middle builds up.
#define GUIDE_DWELL_DEFAULT 1.0f
float guideDwellRevs = GUIDE_DWELL_DEFAULT;

// Spool presets: end stops + inner width per spool type, switchable in one step (SPOOL / SPOOL_SAVE).
#define SPOOL_PRESET_MAX 6
struct SpoolPreset {
  char name[16];
  uint8_t used;
  uint8_t angleL;
  uint8_t angleR;
  uint8_t reserved;
  float widthMm;
};
SpoolPreset spoolPresets[SPOOL_PRESET_MAX];
int8_t activeSpool = -1;  // -1 = none selected

// ----------------- Board variant identifier -----------------
uint8_t boardVariant = 0;

// --------------- High-Speed mode state -----------------
bool highSpeedMode = false;
int savedTorqueLimitHS = 0;
unsigned long baseTargetStepIntervalMicros = DEFAULT_INTERVAL_US;

// -------------------- Status variables & objects ------------------
HardwareSerial TMCSerial(1);
TMC2209Stepper driver(&TMCSerial, R_SENSE, 0);
NimBLECharacteristic* pCharacteristic = nullptr;
Preferences prefs;
String serialRxLine;

float guideServoPos = 0.0f;
bool guideServoTowardsL = true;
float motorStepAcc = 0.0f;
unsigned long lastMotorStepAccTime = 0;
bool servoWasRunning = false;
char lastStateForServo = 'I';
bool servoHomingToHome = false;
unsigned long lastServoHomeStepTime = 0;
bool servoHomeAfterDonePending = false;
char lastStateForDone = 'I';
bool servoGotoActive = false;
char servoGotoSide = 0;

StaticJsonDocument<640> lastStatusDoc;

bool deviceConnected = false;
bool isMotorRunning = false;
bool shouldStartMotorNow = false;
bool filamentDetected = false;
bool pendingDirectionChange = false;
bool lastStableButtonState = HIGH;
bool ledState = false;
int newMotorDirection = 0;
int pwmValue = 0;
int triggerJingleNow = 0;
unsigned long delayStartUntil = 0;
unsigned long filamentLostSince = 0;
unsigned long stepIntervalMicros = START_INTERVAL_US;
unsigned long targetStepIntervalMicros = DEFAULT_INTERVAL_US;
unsigned long spoolingStartTime = 0;
unsigned long lastNotify = 0;
unsigned long lastLedToggle = 0;
unsigned long lastAccelUpdate = 0;
unsigned long lastConnLedTime = 0;
unsigned long lastDebounceTime = 0;
unsigned long lastIntervalMicros = 0;
unsigned long totalEstimatedTime = 0;
unsigned long pausedElapsed = 0;
unsigned long buttonPressSince = 0;
bool buttonLongPressHandled = false;
bool buttonAwaitingDecision = false;
bool doneHoldActive = false;
unsigned long doneHoldStart = 0;
bool remHoldActive = false;
unsigned long remHoldExpiry = 0;
bool progHoldActive = false;
unsigned long progHoldExpiry = 0;
unsigned long remStartHoldUntil = 0;
float progress = 0.0;
bool lastIsMotorRunning = false;
bool lastFilamentDetected = false;
float lastProgress = 0.0;
int lastSpeedPercent = 100;
int lastChipTemperature = 0;
int lastRemainingTime = 0;
volatile bool otaInProgress = false;
SemaphoreHandle_t bleNotifyMutex = nullptr;
char currentState = 'I';

Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);
int ledPulseValue = 0;
int ledPulseDirection = 1;
unsigned long lastLedPulseTime = 0;
uint16_t greenDitherAcc = 0;

// Used before their definitions. The Arduino IDE generates these automatically; PlatformIO's .ino
// converter skips some of them (e.g. std::string arguments), so they are declared explicitly.
int calculatePWM(int brightnessPercent);
void handleCommand(const std::string& cmd);
void setMotorCurrent(int percent);

// ---------- LEDC duty helpers ------------
// Since ESP-IDF v5.5.x (arduino-esp32 3.3.x), ledc_update_duty() on the ESP32 busy-waits for the
// channel's duty_start bit with interrupts disabled. Rewriting the same duty on every loop() pass
// can leave duty_start stuck at 1, which hangs the CPU until the interrupt watchdog resets it.
// The ESP32 must not get a new duty while the previous update is still pending (the IDF source says its
// "internal logic might mess up"), but waiting for it with interrupts off is what hung 1.3.0. All
// runtime duty updates therefore go through ledcSetDutyNoWait(): if the channel is free, the duty is
// latched at once the way ESP-IDF <= 5.5.1 did it; if the previous update is still pending, the new
// duty is parked and ledcServicePending() applies it from loop() once the hardware is done (normally
// within one PWM period, at most 20 ms for the servo). The fan and connection LED channels are also
// only written when their duty changes.
#define LEDC_PENDING_STUCK_MS 200  // an update still pending after this long means the LEDC is broken
static portMUX_TYPE ledcNoWaitMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t ledcPendingMask = 0;
static uint32_t ledcPendingDuty[8];
static unsigned long ledcPendingSince[8];
static volatile bool ledcStuckSeen = false;

static inline bool ledcUpdateBusy(int ch) {
  return LEDC.channel_group[LEDC_LOW_SPEED_MODE].channel[ch].conf1.duty_start;
}

static void ledcLatchDuty(ledc_channel_t ch, uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);  // writes duty + fade parameters, no busy-wait
  portENTER_CRITICAL(&ledcNoWaitMux);
  LEDC.channel_group[LEDC_LOW_SPEED_MODE].channel[ch].conf0.sig_out_en = 1;
  LEDC.channel_group[LEDC_LOW_SPEED_MODE].channel[ch].conf1.duty_start = 1;
  LEDC.channel_group[LEDC_LOW_SPEED_MODE].channel[ch].conf0.low_speed_update = 1;
  portEXIT_CRITICAL(&ledcNoWaitMux);
}

static void ledcSetDutyNoWait(ledc_channel_t ch, uint32_t duty) {
  bool busy;
  portENTER_CRITICAL(&ledcNoWaitMux);
  busy = ledcUpdateBusy(ch);
  if (busy) {
    if (!(ledcPendingMask & (1UL << ch))) ledcPendingSince[ch] = millis();
    ledcPendingDuty[ch] = duty;
    ledcPendingMask |= (1UL << ch);
  } else {
    ledcPendingMask &= ~(1UL << ch);
  }
  portEXIT_CRITICAL(&ledcNoWaitMux);
  if (!busy) ledcLatchDuty(ch, duty);
}

// Called from loop(): applies parked duty updates once the previous update has finished.
static void ledcServicePending() {
  if (ledcPendingMask == 0) return;
  for (int ch = 0; ch < 8; ch++) {
    if (!(ledcPendingMask & (1UL << ch))) continue;
    bool stuck = false;
    portENTER_CRITICAL(&ledcNoWaitMux);
    bool busy = ledcUpdateBusy(ch);
    if (busy && millis() - ledcPendingSince[ch] >= LEDC_PENDING_STUCK_MS) stuck = true;
    uint32_t duty = ledcPendingDuty[ch];
    if (!busy || stuck) ledcPendingMask &= ~(1UL << ch);
    portEXIT_CRITICAL(&ledcNoWaitMux);
    if (busy && !stuck) continue;
    if (stuck) {
      ledcStuckSeen = true;  // the health check reports it and lets the watchdog reset the board
      portENTER_CRITICAL(&ledcNoWaitMux);
      LEDC.channel_group[LEDC_LOW_SPEED_MODE].channel[ch].conf1.duty_start = 0;
      portEXIT_CRITICAL(&ledcNoWaitMux);
    }
    ledcLatchDuty((ledc_channel_t)ch, duty);
  }
}

// ledc_channel_config() calls ledc_update_duty() internally. Clearing a possibly stuck duty_start bit
// first keeps that internal busy-wait from hanging when the STEP channel is re-attached at runtime.
static void ledcChannelConfigSafe(const ledc_channel_config_t* cfg) {
  portENTER_CRITICAL(&ledcNoWaitMux);
  LEDC.channel_group[LEDC_LOW_SPEED_MODE].channel[cfg->channel].conf1.duty_start = 0;
  ledcPendingMask &= ~(1UL << cfg->channel);
  portEXIT_CRITICAL(&ledcNoWaitMux);
  ledc_channel_config(cfg);
}

uint32_t fanDutyCurrent = UINT32_MAX;
uint32_t connLedDutyCurrent = UINT32_MAX;

void setFanDuty(uint32_t duty) {
  if (duty == fanDutyCurrent) return;
  fanDutyCurrent = duty;
  ledcSetDutyNoWait(FAN_CHANNEL, duty);
}

void setConnLedDuty(uint32_t duty) {
  if (duty == connLedDutyCurrent) return;
  connLedDutyCurrent = duty;
  ledcSetDutyNoWait(LEDC_CHANNEL_2, duty);
}

// ---------- OTA LED pulse task handle and function ------------
TaskHandle_t otaLedPulseTaskHandle = nullptr;
TaskHandle_t wifiScanTaskHandle = nullptr;
void otaLedPulseTask(void* parameter) {
  pwmValue = calculatePWM(ledBrightness);
  int otaLedPulse = 0;
  int otaLedDir = 1;
  uint16_t blueDitherAcc = 0;

  while (otaInProgress) {
    otaLedPulse += otaLedDir * 1;
    if (otaLedPulse >= LED_CONN_PULSE_MAX) {
      otaLedPulse = LED_CONN_PULSE_MAX;
      otaLedDir = -1;
    } else if (otaLedPulse <= 0) {
      otaLedPulse = 0;
      otaLedDir = 1;
    }

    setConnLedDuty(LED_CONN_PULSE_MAX - otaLedPulse);

    uint16_t pulse255 = (uint16_t)((otaLedPulse * 255 + (LED_CONN_PULSE_MAX / 2)) / LED_CONN_PULSE_MAX);
    uint16_t scaled16 = (uint16_t)pulse255 * (uint16_t)pwmValue;
    uint8_t baseVal = scaled16 / 255;
    uint8_t remainder = scaled16 % 255;
    blueDitherAcc = (uint16_t)(blueDitherAcc + remainder);
    if (blueDitherAcc >= 255) {
      baseVal++;
      blueDitherAcc -= 255;
    }

    led.setPixelColor(0, led.Color(0, 0, baseVal));
    led.show();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  setConnLedDuty(LED_CONN_PULSE_MAX);
  led.setPixelColor(0, 0);
  led.show();
  vTaskDelete(NULL);
}

void wifiScanTask(void* parameter) {
  WiFi.disconnect(true);
  int n = WiFi.scanNetworks();
  StaticJsonDocument<256> ssidDoc;
  JsonArray arr = ssidDoc.createNestedArray("SSID_LIST");
  for (int i = 0; i < n; i++) {
    int rssi = WiFi.RSSI(i);
    if (rssi > MIN_RSSI_THRESHOLD) {
      String ssid = WiFi.SSID(i);
      if (ssid.length() > 0 && ssid.length() < 33) {
        arr.add(ssid);
      }
    }
  }
  if (deviceConnected && pCharacteristic) {
    String jsonOut;
    serializeJson(ssidDoc, jsonOut);
    if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
    pCharacteristic->setValue(jsonOut.c_str());
    pCharacteristic->notify();
    if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
  }
  WiFi.scanDelete();
  prefs.begin("respooler", true);
  String ssid = prefs.getString("ssid", "");
  String pwd = prefs.getString("pwd", "");
  prefs.end();
  if (ssid.length() > 0 && pwd.length() > 0) {
    WiFi.begin(ssid.c_str(), pwd.c_str());
  }
  wifiScanTaskHandle = nullptr;
  vTaskDelete(NULL);
}

static void handleDiagCommand(const String& line);

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      serialRxLine.trim();
      if (serialRxLine.length() > 0) {
        if (serialRxLine.length() > 512) {
          serialRxLine = "";
        } else if (serialRxLine[0] == '!') {
          String line = serialRxLine;
          serialRxLine = "";
          handleDiagCommand(line);
        } else {
          std::string cmd(serialRxLine.c_str());
          handleCommand(cmd);
          serialRxLine = "";
        }
      } else {
        serialRxLine = "";
      }
    } else {
      if (serialRxLine.length() < 512) serialRxLine += c;
    }
  }
}

// -------------- Wi-Fi connect async status --------------
bool wifiConnectInProgress = false;
unsigned long wifiConnectStartTime = 0;

// ================ Watchdog, auto-resume and diagnostics ================
// Some boards stop dead while winding (fan stops, LED frozen, BLE gone, garbage on the USB serial
// port) until EN is pressed. That points at the chip's clocks or peripherals being upset by a supply
// dip rather than a software hang, and the task/interrupt watchdogs run from the same APB clock.
//
// 1. RTC watchdog. It runs from the RTC slow clock, independent of APB/PLL. loop() feeds it only
//    while the board is healthy: loop() is running, esp_timer agrees with the RTC clock, and the LEDC
//    outputs (fan, servo, connection LED, STEP) are still configured, routed and ticking. Not fed for
//    RWDT_TIMEOUT_MS -> system reset; still stuck RWDT_STAGE1_MS later -> full RTC reset.
// 2. Auto-resume. While winding, the job (elapsed time, guide servo position) is kept in RTC memory,
//    which survives that reset. After an abnormal reset (watchdog, panic, brown-out) the board shows
//    "paused" for AUTO_RESUME_DELAY_MS and then continues. After power-on or EN the job is restored
//    as paused only, so the motor never starts by itself when the power is switched on.
// 3. The brown-out threshold is raised (BROWNOUT_LEVEL) so a real supply dip gives a clean reset,
//    and then a resume, instead of an undefined state.
// 4. Serial diagnostics at 115200 baud, all lines start with '#': the reset reason and what the
//    board was doing before it, then a heartbeat every second.
//    Commands: !help !diag !hb 0|1 !wdt !fault !panic !reset

#define RUNSTATE_MAGIC (0x52535000UL ^ (uint32_t)sizeof(RunState))
#define DIAG_MAGIC (0x44474E00UL ^ (uint32_t)sizeof(DiagState))

struct RunState {
  uint32_t magic;
  uint32_t elapsedMs;
  uint32_t totalEstMs;
  float servoPos;
  float stepAcc;
  uint8_t active;  // 1 = a winding job is in progress (state 'R')
  uint8_t dir;
  uint8_t servoTowardsL;
  uint8_t quickResumes;  // auto-resumes since the job last ran AUTO_RESUME_STABLE_MS
  uint32_t crc;
};

struct DiagState {
  uint32_t magic;
  uint32_t bootCount;
  uint32_t abnormalCount;
  uint32_t resumeCount;
  uint32_t lastBootRtcMs;
  uint32_t lastUptimeS;
  uint32_t lastLoopMaxMs;
  uint32_t lastFreeHeap;
  int32_t lastClkPermille;
  char lastState;
  uint8_t lastFault;
  uint8_t rapidBoots;
  uint8_t rtcResetDone;
};

RTC_NOINIT_ATTR RunState rtcRun;
RTC_NOINIT_ATTR DiagState rtcDiag;
RTC_NOINIT_ATTR volatile uint32_t rtcCrumb;  // last loop()/setup() section entered
#define CRUMB(n) (rtcCrumb = (uint32_t)(n))

enum { FAULT_NONE = 0, FAULT_CLOCK = 1, FAULT_LEDC = 2, FAULT_TEST = 99 };

static wdt_hal_context_t rwdtCtx = RWDT_HAL_CONTEXT_DEFAULT();
static bool rwdtRunning = false;
static volatile uint8_t healthFault = FAULT_NONE;
static bool heartbeatOn = true;
static bool autoResumePending = false;
static unsigned long autoResumeAt = 0;
static bool resumeStartRequested = false;
static char bootResumeMode = 0;  // 0 = nothing to restore, 'A' = auto-resume, 'P' = restore as paused
static uint32_t loopMaxMs = 0;
static float lastClkRatio = 1.0f;
static bool wifiSuspended = false;
static esp_reset_reason_t bootReason = ESP_RST_UNKNOWN;
static int bootRawReason = 0;
static char bootLine[200] = "";
static char prevLine[200] = "";

static uint32_t runStateCrc() {
  return esp_rom_crc32_le(0, (const uint8_t*)&rtcRun, offsetof(RunState, crc));
}
static bool runStateValid() {
  return rtcRun.magic == RUNSTATE_MAGIC && rtcRun.crc == runStateCrc();
}
static void runStateCommit() {
  rtcRun.magic = RUNSTATE_MAGIC;
  rtcRun.crc = runStateCrc();
}

static uint32_t rwdtTicks(uint32_t ms) {
  return (uint32_t)((uint64_t)ms * rtc_clk_slow_freq_get_hz() / 1000ULL);
}

static void rwdtStart() {
  wdt_hal_init(&rwdtCtx, WDT_RWDT, 0, false);
  wdt_hal_write_protect_disable(&rwdtCtx);
  wdt_hal_config_stage(&rwdtCtx, WDT_STAGE0, rwdtTicks(RWDT_TIMEOUT_MS), WDT_STAGE_ACTION_RESET_SYSTEM);
  wdt_hal_config_stage(&rwdtCtx, WDT_STAGE1, rwdtTicks(RWDT_STAGE1_MS), WDT_STAGE_ACTION_RESET_RTC);
  wdt_hal_set_flashboot_en(&rwdtCtx, false);
  wdt_hal_enable(&rwdtCtx);
  wdt_hal_write_protect_enable(&rwdtCtx);
  rwdtRunning = true;
}

static void rwdtFeed() {
  if (!rwdtRunning) return;
  wdt_hal_write_protect_disable(&rwdtCtx);
  wdt_hal_feed(&rwdtCtx);
  wdt_hal_write_protect_enable(&rwdtCtx);
}

// Resets the digital system and the RTC domain: the closest software equivalent of pressing EN.
static void rtcFullReset() {
  Serial.flush();
  wdt_hal_init(&rwdtCtx, WDT_RWDT, 0, false);
  wdt_hal_write_protect_disable(&rwdtCtx);
  wdt_hal_config_stage(&rwdtCtx, WDT_STAGE0, rwdtTicks(20), WDT_STAGE_ACTION_RESET_RTC);
  wdt_hal_enable(&rwdtCtx);
  wdt_hal_write_protect_enable(&rwdtCtx);
  volatile uint32_t spin = 0;
  while (true) { spin++; }
}

static const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

static bool isAbnormalReset(esp_reset_reason_t r) {
  return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT
         || r == ESP_RST_BROWNOUT || r == ESP_RST_UNKNOWN;
}

static const char* faultName(uint8_t f) {
  switch (f) {
    case FAULT_CLOCK: return "clock";
    case FAULT_LEDC: return "LEDC";
    case FAULT_TEST: return "test";
    default: return "none";
  }
}

// Runs first thing in setup(). Reports why we booted, breaks boot loops and decides what to do with
// a winding job that was interrupted.
static void bootAnalyze() {
  bootReason = esp_reset_reason();
  bootRawReason = (int)esp_rom_get_reset_reason(0);
  bool diagValid = rtcDiag.magic == DIAG_MAGIC && rtcDiag.bootCount < 100000000UL;
  DiagState prev;
  memset(&prev, 0, sizeof(prev));
  uint32_t prevCrumb = 0;
  if (diagValid) {
    prev = rtcDiag;
    prevCrumb = rtcCrumb;
  } else {
    memset(&rtcDiag, 0, sizeof(rtcDiag));
    rtcDiag.magic = DIAG_MAGIC;
  }
  rtcCrumb = 0;

  bool abnormal = isAbnormalReset(bootReason);
  uint32_t nowRtcMs = (uint32_t)(esp_rtc_get_time_us() / 1000ULL);
  rtcDiag.bootCount++;
  if (abnormal) rtcDiag.abnormalCount++;
  if (abnormal && diagValid && nowRtcMs >= rtcDiag.lastBootRtcMs
      && nowRtcMs - rtcDiag.lastBootRtcMs < BOOTLOOP_WINDOW_MS) {
    rtcDiag.rapidBoots++;
  } else {
    rtcDiag.rapidBoots = abnormal ? 1 : 0;
  }
  rtcDiag.lastBootRtcMs = nowRtcMs;
  bool brokeLoop = rtcDiag.rtcResetDone;
  rtcDiag.rtcResetDone = 0;

  snprintf(bootLine, sizeof(bootLine), "#BOOT fw=%s reset=%s raw=%d boots=%lu abnormal=%lu resumes=%lu",
           FIRMWARE_BUILD, resetReasonName(bootReason), bootRawReason, (unsigned long)rtcDiag.bootCount,
           (unsigned long)rtcDiag.abnormalCount, (unsigned long)rtcDiag.resumeCount);
  if (diagValid && prev.bootCount > 0) {
    snprintf(prevLine, sizeof(prevLine), "#PREV up=%lus state=%c crumb=%lu loopmax=%lums clk=%.3f fault=%s heap=%lu",
             (unsigned long)prev.lastUptimeS, prev.lastState ? prev.lastState : '-', (unsigned long)prevCrumb,
             (unsigned long)prev.lastLoopMaxMs, prev.lastClkPermille / 1000.0, faultName(prev.lastFault),
             (unsigned long)prev.lastFreeHeap);
  } else {
    snprintf(prevLine, sizeof(prevLine), "#PREV none (RTC memory was empty, i.e. power-on)");
  }
  Serial.println();
  dlogln(bootLine);
  dlogln(prevLine);
  if (brokeLoop) dlogln("#BOOT the previous boot loop was broken with a full RTC reset");

  if (rtcDiag.rapidBoots >= BOOTLOOP_LIMIT) {
    rtcDiag.rapidBoots = 0;
    rtcDiag.rtcResetDone = 1;
    dlogln("#BOOTLOOP abnormal resets keep coming: full RTC reset");
    rtcFullReset();
  }

  bootResumeMode = 0;
  if (runStateValid() && rtcRun.active) {
    bool sane = rtcRun.elapsedMs < 24UL * 3600UL * 1000UL && rtcRun.servoPos >= 0.0f && rtcRun.servoPos <= 180.0f;
    if (!sane) {
      dlogln("#JOB interrupted, but the saved state looks wrong: ignored");
    } else {
      bool powerUp = bootReason == ESP_RST_POWERON || bootReason == ESP_RST_EXT || bootReason == ESP_RST_DEEPSLEEP;
      if (powerUp) bootResumeMode = 'P';
      else if (rtcRun.quickResumes >= AUTO_RESUME_MAX_QUICK) bootResumeMode = 'P';
      else bootResumeMode = 'A';
      dlogf("#JOB interrupted at %lus, servo %.1f deg -> %s\n", (unsigned long)(rtcRun.elapsedMs / 1000UL),
                    rtcRun.servoPos,
                    bootResumeMode == 'A' ? "auto-resume" : powerUp ? "paused (power-on: press the button or use the app)"
                                                                    : "paused (too many quick crashes)");
    }
  }
  if (bootResumeMode == 0) {
    rtcRun.active = 0;
    runStateCommit();
  }
}

// Called in setup() after loadSettings(): puts an interrupted job back as a paused job.
static void applyBootResume() {
  if (!bootResumeMode) return;
  pausedElapsed = rtcRun.elapsedMs > 0 ? rtcRun.elapsedMs : 1;
  totalEstimatedTime = rtcRun.totalEstMs;
  spoolingStartTime = 0;
  guideServoPos = rtcRun.servoPos;
  guideServoTowardsL = rtcRun.servoTowardsL != 0;
  float stepsPerSpoolRev = (200.0f * (float)MICROSTEPPING) * (52.0f / 18.0f);
  motorStepAcc = (rtcRun.stepAcc >= 0.0f && rtcRun.stepAcc < stepsPerSpoolRev) ? rtcRun.stepAcc : 0.0f;
  if (rtcRun.dir != (uint8_t)motorDirection) {
    dlogf("#JOB note: saved direction %u, settings say %d (using settings)\n", rtcRun.dir, motorDirection);
  }
  if (bootResumeMode == 'A') {
    rtcRun.quickResumes++;
    rtcDiag.resumeCount++;
    runStateCommit();
    autoResumePending = true;
    autoResumeAt = millis() + AUTO_RESUME_DELAY_MS;
  } else {
    rtcRun.active = 0;
    runStateCommit();
  }
}

// ---- health checks ----
#define LEDC_TIMER_CONF_MASK 0x03FFFFFFUL   // duty_resolution, clock_divider, pause, rst, tick_sel
#define LEDC_TIMER_FIXED_MASK 0x0180001FUL  // duty_resolution, pause, rst (STEP timer changes frequency)
static uint32_t snapFanTimer, snapServoTimer, snapConnTimer, snapStepTimer;
static uint32_t snapFanCh, snapServoCh, snapConnCh;
static bool ledcSnapValid = false;

static inline uint32_t ledcTimerConf(int t) {
  return LEDC.timer_group[LEDC_LOW_SPEED_MODE].timer[t].conf.val;
}
static inline uint32_t ledcChConf0(int ch) {
  return LEDC.channel_group[LEDC_LOW_SPEED_MODE].channel[ch].conf0.val & 0x7;  // timer_sel + sig_out_en
}
static inline uint32_t gpioOutSel(int pin) {
  return GPIO.func_out_sel_cfg[pin].func_sel;
}

// Called at the end of setup(), once all LEDC timers and channels are configured.
static void ledcSnapshot() {
  snapFanTimer = ledcTimerConf(FAN_TIMER) & LEDC_TIMER_CONF_MASK;
  snapServoTimer = ledcTimerConf(SERVO_LEDC_TIMER) & LEDC_TIMER_CONF_MASK;
  snapConnTimer = ledcTimerConf(LEDC_TIMER_0) & LEDC_TIMER_CONF_MASK;
  snapStepTimer = ledcTimerConf(STEP_LEDC_TIMER) & LEDC_TIMER_FIXED_MASK;
  snapFanCh = ledcChConf0(FAN_CHANNEL);
  snapServoCh = ledcChConf0(SERVO_LEDC_CHANNEL);
  snapConnCh = ledcChConf0(LEDC_CHANNEL_2);
  ledcSnapValid = true;
}

static bool ledcTimerTicking(int t) {
  uint32_t a = LEDC.timer_group[LEDC_LOW_SPEED_MODE].timer[t].value.timer_cnt;
  for (int i = 0; i < 3; i++) {
    esp_rom_delay_us(30);
    if (LEDC.timer_group[LEDC_LOW_SPEED_MODE].timer[t].value.timer_cnt != a) return true;
  }
  return false;
}

static bool ledcHealthy() {
  if (!ledcSnapValid) return true;
  if ((ledcTimerConf(FAN_TIMER) & LEDC_TIMER_CONF_MASK) != snapFanTimer) return false;
  if ((ledcTimerConf(SERVO_LEDC_TIMER) & LEDC_TIMER_CONF_MASK) != snapServoTimer) return false;
  if ((ledcTimerConf(LEDC_TIMER_0) & LEDC_TIMER_CONF_MASK) != snapConnTimer) return false;
  if (ledcChConf0(FAN_CHANNEL) != snapFanCh) return false;
  if (ledcChConf0(SERVO_LEDC_CHANNEL) != snapServoCh) return false;
  if (ledcChConf0(LEDC_CHANNEL_2) != snapConnCh) return false;
  if (gpioOutSel(FAN_PIN) != (uint32_t)(LEDC_LS_SIG_OUT0_IDX + FAN_CHANNEL)) return false;
  if (gpioOutSel(SERVO_PIN) != (uint32_t)(LEDC_LS_SIG_OUT0_IDX + SERVO_LEDC_CHANNEL)) return false;
  if (gpioOutSel(LED_CONN_PIN) != (uint32_t)(LEDC_LS_SIG_OUT0_IDX + LEDC_CHANNEL_2)) return false;
  if (!ledcTimerTicking(SERVO_LEDC_TIMER) || !ledcTimerTicking(LEDC_TIMER_0)) return false;
  if (isMotorRunning) {
    if ((ledcTimerConf(STEP_LEDC_TIMER) & LEDC_TIMER_FIXED_MASK) != snapStepTimer) return false;
    if (ledcChConf0(STEP_LEDC_CHANNEL) != (uint32_t)(STEP_LEDC_TIMER | 0x4)) return false;
    if (gpioOutSel(STEP_PIN) != (uint32_t)(LEDC_LS_SIG_OUT0_IDX + STEP_LEDC_CHANNEL)) return false;
    if (!ledcTimerTicking(STEP_LEDC_TIMER)) return false;
  }
  return true;
}

static uint8_t healthCheck() {
  static int64_t prevSysUs = 0;
  static uint64_t prevRtcUs = 0;
  static uint8_t clkBad = 0, ledcBad = 0;
  int64_t sysUs = esp_timer_get_time();
  uint64_t rtcUs = esp_rtc_get_time_us();
  if (prevSysUs != 0) {
    int64_t ds = sysUs - prevSysUs;
    int64_t dr = (int64_t)(rtcUs - prevRtcUs);
    if (dr > 200000) {
      lastClkRatio = (float)((double)ds / (double)dr);
      if (lastClkRatio < 0.85f || lastClkRatio > 1.15f) {
        if (clkBad < 255) clkBad++;
      } else {
        clkBad = 0;
      }
    }
  }
  prevSysUs = sysUs;
  prevRtcUs = rtcUs;
  if (ledcHealthy()) ledcBad = 0;
  else if (ledcBad < 255) ledcBad++;
  if (clkBad >= 2) return FAULT_CLOCK;
  if (ledcBad >= 2 || ledcStuckSeen) return FAULT_LEDC;
  return FAULT_NONE;
}

// ---- Wi-Fi radio power ----
static void wifiRadioOn() {
  if (wifiSSID.length() == 0 || wifiPassword.length() == 0) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  if (WIFI_TX_LIMIT) WiFi.setTxPower(WIFI_TX_POWER);
}

static void wifiPowerManage(unsigned long now) {
#if WIFI_OFF_WHILE_WINDING
  static unsigned long idleSince = 0;
  bool winding = currentState == 'R' || autoResumePending;
  bool busy = otaInProgress || wifiConnectInProgress || wifiScanTaskHandle != nullptr;
  if (winding) {
    idleSince = 0;
    if (!busy && WiFi.getMode() != WIFI_OFF) {
      wifiSuspended = true;  // first, so the disconnect event is not counted as a Wi-Fi drop
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      dlogln("#WIFI off while winding");
    }
  } else if (wifiSuspended) {
    if (idleSince == 0) idleSince = now;
    if (now - idleSince >= 5000) {
      wifiSuspended = false;
      idleSince = 0;
      wifiRadioOn();
      dlogln("#WIFI on");
    }
  }
#endif
}

static void printDiag(bool withBoot) {
  if (withBoot) {
    Serial.println(bootLine);
    Serial.println(prevLine);
  }
  dlogf("#CFG cpu=%luMHz ble=%ddBm wifi_off_winding=%d brownout_lvl=%d rwdt=%d+%dms resume_delay=%dms\n",
                (unsigned long)getCpuFrequencyMhz(), BLE_TX_POWER_DBM, WIFI_OFF_WHILE_WINDING, BROWNOUT_LEVEL,
                RWDT_TIMEOUT_MS, RWDT_STAGE1_MS, AUTO_RESUME_DELAY_MS);
  dlogf("#NOW state=%c job=%s quick_resumes=%u fault=%s heartbeat=%d\n", currentState,
                (runStateValid() && rtcRun.active) ? "saved" : "none",
                (runStateValid() && rtcRun.active) ? rtcRun.quickResumes : 0,
                faultName(healthFault),
                heartbeatOn ? 1 : 0);
}

static void handleDiagCommand(const String& line) {
  if (line == "!help") {
    dlogln("#CMD !diag | !hb 0 / !hb 1 | !wdt = hang loop() | !fault = stop feeding watchdog | !panic | !reset");
  } else if (line == "!diag") {
    printDiag(true);
  } else if (line.startsWith("!hb")) {
    heartbeatOn = !line.endsWith("0");
    dlogf("#CMD heartbeat %s\n", heartbeatOn ? "on" : "off");
  } else if (line == "!wdt") {
    dlogln("#TEST loop() hangs now; the RTC watchdog should reset the board in about 6 s");
    Serial.flush();
    volatile uint32_t spin = 0;
    while (true) { spin++; }
  } else if (line == "!fault") {
    healthFault = FAULT_TEST;
    dlogln("#TEST stopped feeding the watchdog");
  } else if (line == "!panic") {
    dlogln("#TEST abort()");
    Serial.flush();
    abort();
  } else if (line == "!reset") {
    dlogln("#TEST esp_restart()");
    Serial.flush();
    esp_restart();
  } else {
    dlogln("#CMD unknown, try !help");
  }
}

// First thing in every loop() pass (loop() has early returns further down).
static void healthTick(unsigned long now) {
  static unsigned long lastLoopAt = 0, lastHealthAt = 0, lastHbAt = 0, lastSaveAt = 0, rStateSince = 0;
  static uint32_t loopsThisSecond = 0;
  static char prevState = 0;
  static bool wifiTxSet = false;
  static unsigned long lastFeedAt = 0;
  CRUMB(1);
  ledcServicePending();
  if (lastLoopAt != 0 && now - lastLoopAt > loopMaxMs) loopMaxMs = now - lastLoopAt;
  lastLoopAt = now;
  loopsThisSecond++;

  if (currentState != prevState) {
    if (prevState) dlogf("#STATE %c -> %c\n", prevState, currentState);
    if (currentState == 'R') {
      rStateSince = now;
      if (!resumeStartRequested) rtcRun.quickResumes = 0;  // started by the user: a fresh count
    }
    resumeStartRequested = false;
    if (currentState != 'P') autoResumePending = false;  // stopped or started by the user meanwhile
    prevState = currentState;
  }

  if (autoResumePending && (long)(now - autoResumeAt) >= 0) {
    autoResumePending = false;
    if (currentState == 'P' && !isMotorRunning && !shouldStartMotorNow && !otaInProgress) {
      resumeStartRequested = true;
      shouldStartMotorNow = true;
      delayStartUntil = now;
      dlogln("#RESUME starting the motor");
    }
  }

  if (now - lastSaveAt >= RUNSTATE_SAVE_INTERVAL_MS) {
    lastSaveAt = now;
    if (currentState == 'R') {
      rtcRun.active = 1;
      rtcRun.elapsedMs = spoolingStartTime > 0 ? (uint32_t)(now - spoolingStartTime) : (uint32_t)pausedElapsed;
      rtcRun.totalEstMs = (uint32_t)totalEstimatedTime;
      rtcRun.servoPos = guideServoPos;
      rtcRun.stepAcc = motorStepAcc;
      rtcRun.servoTowardsL = guideServoTowardsL ? 1 : 0;
      rtcRun.dir = (uint8_t)motorDirection;
      if (isMotorRunning && now - rStateSince >= AUTO_RESUME_STABLE_MS) rtcRun.quickResumes = 0;
      runStateCommit();
    } else if (rtcRun.active && !autoResumePending) {
      rtcRun.active = 0;
      runStateCommit();
    }
  }

  if (now - lastHealthAt >= HEALTH_CHECK_INTERVAL_MS) {
    lastHealthAt = now;
    uint8_t f = healthCheck();
    if (f != FAULT_NONE && healthFault == FAULT_NONE) {
      healthFault = f;
      dlogf("#FAULT %s: no longer feeding the watchdog, reset follows\n", faultName(f));
    }
    rtcDiag.lastFault = healthFault;
  }

  if (now - lastHbAt >= HEARTBEAT_INTERVAL_MS) {
    lastHbAt = now;
    uint32_t heap = ESP.getFreeHeap();
    rtcDiag.lastUptimeS = now / 1000UL;
    rtcDiag.lastState = currentState;
    rtcDiag.lastLoopMaxMs = loopMaxMs;
    rtcDiag.lastFreeHeap = heap;
    rtcDiag.lastClkPermille = (int32_t)lroundf(lastClkRatio * 1000.0f);
    bool wifiUp = !wifiSuspended && WiFi.status() == WL_CONNECTED;
    if (wifiUp && !wifiTxSet) {
      if (WIFI_TX_LIMIT) WiFi.setTxPower(WIFI_TX_POWER);
      wifiTxSet = true;
    } else if (!wifiUp) {
      wifiTxSet = false;
    }
    if (heartbeatOn) {
      unsigned long el = spoolingStartTime > 0 ? now - spoolingStartTime : pausedElapsed;
      static uint8_t hbKeep = 0;
      hbKeep = (hbKeep + 1) % 30;  // every 30th heartbeat goes into the /log buffer
      int8_t bleRssi = 0;
      if (deviceConnected) {
        NimBLEServer* srv = NimBLEDevice::getServer();
        if (srv) {
          std::vector<uint16_t> peers = srv->getPeerDevices();
          if (!peers.empty()) ble_gap_conn_rssi(peers[0], &bleRssi);
        }
      }
      dlogfKeep(hbKeep == 1, "#HB up=%lu st=%c run=%d el=%lu iv=%lu/%lu sv=%.1f lps=%lu lmax=%lu clk=%.3f heap=%lu wifi=%s ble=%d/%d f=%s\n",
                    now / 1000UL, currentState, isMotorRunning ? 1 : 0, el / 1000UL, stepIntervalMicros,
                    targetStepIntervalMicros, guideServoPos, (unsigned long)loopsThisSecond, (unsigned long)loopMaxMs,
                    lastClkRatio, (unsigned long)heap, wifiSuspended ? "off" : (wifiUp ? "ok" : "--"),
                    deviceConnected ? 1 : 0, (int)bleRssi, faultName(healthFault));
    }
    loopsThisSecond = 0;
    loopMaxMs = 0;
  }

  wifiPowerManage(now);

  if (healthFault == FAULT_NONE && now - lastFeedAt >= 100) {
    lastFeedAt = now;
    rwdtFeed();
  }
}

// ------------------ Helper and utility functions ---------------
int calculatePWM(int brightnessPercent) {
  float gamma = 1.15f;
  float n = (float)constrain(brightnessPercent, 0, 100) / 100.0f;
  return (int)round(powf(n, gamma) * 255.0f);
}
int fanPWMFromSpeed(int percent) {
  return map(constrain(percent, 0, 100), 0, 100, 0, 255);
}

// Servo position with sub-degree resolution: 500-2500 us over 0-180 deg, 14-bit duty at 50 Hz
// (1.22 us per count, about 0.11 deg). The stock firmware rounded to whole degrees, so the guide
// advanced 4 or 5 deg per spool revolution in turn instead of a steady 4.8 deg.
static uint32_t servoDutyLast = UINT32_MAX;
inline void setServoAngleF(float angle) {
  if (angle < 0.0f) angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;
  float pulseUs = 500.0f + angle * (2000.0f / 180.0f);
  uint32_t duty = (uint32_t)lroundf(pulseUs * (float)((1UL << 14) - 1) / 20000.0f);
  if (duty == servoDutyLast) return;
  servoDutyLast = duty;
  ledcSetDutyNoWait(SERVO_LEDC_CHANNEL, duty);
}
inline void setServoAngle(int angle) {
  setServoAngleF((float)angle);
}
// Servo degrees the guide advances per spool revolution.
inline float guideStepDeg() {
  float d = servoStepMm / servoMmPerDeg;
  return d < 0.05f ? 0.05f : d;
}

// Current guide segment: start and end position, and how many spool revolutions it lasts.
// The guide always moves at one step (servoStepMm) per revolution. The last piece before an end stop is
// usually shorter than a step and lasts only that fraction of a revolution (the old code stretched it to
// a full revolution, which packed extra filament next to the flanges); then the guide stays at the end
// stop for guideDwellRevs revolutions and turns back.
float guideSegFrom = 0.0f;
float guideSegTo = 0.0f;
float guideSegRevs = 1.0f;
bool guideDwellPending = false;

void guideNextSegment() {
  float from = guideSegTo;
  guideSegFrom = from;
  if (guideDwellPending) {
    guideDwellPending = false;
    if (guideDwellRevs >= 0.05f) {
      guideSegRevs = guideDwellRevs;
      return;  // guideSegTo stays at the end stop
    }
  }
  float target = guideServoTowardsL ? (float)servoAngleL : (float)servoAngleR;
  float stepDeg = guideStepDeg();
  float dist = fabsf(target - from);
  if (dist <= stepDeg) {
    guideSegTo = target;
    guideSegRevs = dist / stepDeg;
    if (guideSegRevs < 0.02f) guideSegRevs = 0.02f;
    guideServoTowardsL = !guideServoTowardsL;
    guideDwellPending = true;
  } else {
    guideSegTo = from + ((from < target) ? stepDeg : -stepDeg);
    guideSegRevs = 1.0f;
  }
}
inline void setConnLed(uint8_t duty) {
  duty = constrain(duty, 0, 255);
  setConnLedDuty(255 - duty);
}
inline uint32_t stepFreqFromIntervalUs(unsigned long us) {
  if (us < 1) us = 1;
  return (uint32_t)(1000000UL / (2UL * us));
}
inline void stepperSetFreq(uint32_t freqHz) {
  if (freqHz < 1) freqHz = 1;
  ledc_set_freq(LEDC_LOW_SPEED_MODE, STEP_LEDC_TIMER, freqHz);
}
inline void stepperStart(uint32_t freqHz) {
  stepperSetFreq(freqHz);
  ledcSetDutyNoWait(STEP_LEDC_CHANNEL, STEP_DUTY_ON);
}
inline void stepperStop() {
  ledcSetDutyNoWait(STEP_LEDC_CHANNEL, 0);
}
int getSpeedPercentFromInterval(unsigned long us) {
  double t = (double)us;
  double p = 50.0 + ((170.0 - t) * 50.0 / (170.0 - 93.0));
  int percent = (int)round(p);
  return constrain(percent, 50, 100);
}

void applySpeedTarget() {
  if (highSpeedMode) {
    unsigned long adjusted = (unsigned long)round((double)baseTargetStepIntervalMicros / HIGH_SPEED_FACTOR);
    if (adjusted < 50) adjusted = 50;
    targetStepIntervalMicros = adjusted;
  } else {
    targetStepIntervalMicros = baseTargetStepIntervalMicros;
  }
}
void loadSettings() {
  prefs.begin("respooler", true);
  boardVariant = prefs.getUChar("variant", 0);
  baseTargetStepIntervalMicros = prefs.getUInt("speed", DEFAULT_INTERVAL_US);
  speedPercent = getSpeedPercentFromInterval(baseTargetStepIntervalMicros);
  applySpeedTarget();
  motorDirection = prefs.getUInt("dir", 0);
  ledBrightness = prefs.getUInt("led", 50);
  useFilamentSensor = prefs.getBool("filamentSensor", true);
  motorStrength = prefs.getUInt("motorStrength", 100);
  torqueLimit = prefs.getUInt("torqueLimit", 0);
  highSpeedMode = prefs.getBool("hsMode", false);
  if (highSpeedMode) {
    savedTorqueLimitHS = torqueLimit;
    torqueLimit = 0;
  }
  jingleStyle = prefs.getUInt("jingle", 0);
  calibrationAt80Speed = prefs.getULong("cal80_093", 895000);
  fanSpeed = prefs.getUInt("fanSpeed", 60);
  fanAlwaysOn = prefs.getBool("fanAlways", false);
  targetWeight = prefs.getUInt("targetWeight", 0);

  servoAngleR = (int)prefs.getUInt("servoR", 5);
  servoAngleL = (int)prefs.getUInt("servoL", 175);
  servoStepMm = prefs.getFloat("servoStep", 1.75f);
  servoHomeIsR = prefs.getBool("servoHomeR", true);

  servoAngleR = constrain(servoAngleR, 0, 180);
  servoAngleL = constrain(servoAngleL, 0, 180);
  servoStepMm = constrain(servoStepMm, 0.05f, 20.0f);

  servoMmPerDeg = prefs.getFloat("svMmDeg", SERVO_MM_PER_DEG_NOMINAL);
  if (!(servoMmPerDeg >= 0.1f && servoMmPerDeg <= 1.0f)) servoMmPerDeg = SERVO_MM_PER_DEG_NOMINAL;
  spoolWidthMm = prefs.getFloat("spoolW", 0.0f);
  if (!(spoolWidthMm >= 0.0f && spoolWidthMm <= 300.0f)) spoolWidthMm = 0.0f;
  guideDwellRevs = prefs.getFloat("svDwell", GUIDE_DWELL_DEFAULT);
  if (!(guideDwellRevs >= 0.0f && guideDwellRevs <= 3.0f)) guideDwellRevs = GUIDE_DWELL_DEFAULT;
  memset(spoolPresets, 0, sizeof(spoolPresets));
  if (prefs.getBytesLength("spools1") == sizeof(spoolPresets)) {
    prefs.getBytes("spools1", spoolPresets, sizeof(spoolPresets));
  }
  for (int i = 0; i < SPOOL_PRESET_MAX; i++) {
    spoolPresets[i].name[sizeof(spoolPresets[i].name) - 1] = 0;
    if (spoolPresets[i].used != 1 || spoolPresets[i].angleL > 180 || spoolPresets[i].angleR > 180) {
      memset(&spoolPresets[i], 0, sizeof(SpoolPreset));
    }
  }
  activeSpool = (int8_t)prefs.getChar("spoolSel", -1);
  if (activeSpool < -1 || activeSpool >= SPOOL_PRESET_MAX || (activeSpool >= 0 && !spoolPresets[activeSpool].used)) {
    activeSpool = -1;
  }

  applySpeedTarget();
  prefs.end();
  setMotorCurrent(motorStrength);
  pwmValue = calculatePWM(ledBrightness);
  digitalWrite(DIR_PIN, motorDirection);
}

void setMotorCurrent(int percent) {
  int strengthCurrent = map(percent, 80, 120, 900, 1600);
  driver.rms_current(strengthCurrent);
}

void reSanitizeDriver() {
  driver.GSTAT(0b111);
  driver.I_scale_analog(false);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  driver.microsteps(MICROSTEPPING);
  setMotorCurrent(motorStrength);
}

// ------------------- BLE status/error/settings -------------------
void sendStatus(bool forceSend = false) {
  if (!deviceConnected || !pCharacteristic) return;

  float progVal = progress;
  int remVal = 0;
  if (currentState == 'D') {
    progVal = 100.0f;
    remVal = 0;
  } else if (totalEstimatedTime > 0) {
    unsigned long elapsed;
    if (spoolingStartTime > 0) {
      elapsed = millis() - spoolingStartTime;
    } else if (pausedElapsed > 0) {
      elapsed = pausedElapsed;
    } else if (currentState == 'A') {
      elapsed = pausedElapsed;
    } else {
      elapsed = 0;
    }
    unsigned long effectiveTotal = totalEstimatedTime;
    if (targetWeight == 2) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_2);
    else if (targetWeight == 3) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_3);

    if (effectiveTotal == 0) {
      progVal = 0.0f;
      remVal = 0;
    } else if (elapsed >= effectiveTotal) {
      progVal = 100.0f;
      remVal = 0;
    } else {
      progVal = (100.0f * (float)elapsed) / (float)effectiveTotal;
      unsigned long remainingMs = effectiveTotal - elapsed;
      remVal = (int)(remainingMs / 1000UL);
    }
  }
  if (remHoldActive) {
    unsigned long nowMs = millis();
    if (nowMs < remHoldExpiry) {
      remVal = lastRemainingTime;
    } else {
      remHoldActive = false;
    }
  }
  if (progHoldActive) {
    unsigned long nowMs = millis();
    if (nowMs < progHoldExpiry) {
      progVal = lastProgress;
    } else {
      progHoldActive = false;
    }
  }
  if (currentState == 'R') {
    unsigned long nowMs = millis();
    if (nowMs < remStartHoldUntil) {
      remVal = lastRemainingTime;
    }
  }
  int chipTemp = (int)temperatureRead();

  StaticJsonDocument<640> doc;
  doc["STAT"] = String(currentState);
  doc["HAS_FIL"] = filamentDetected;
  doc["USE_FIL"] = useFilamentSensor;
  doc["PROG"] = progVal;
  doc["REM"] = remVal;
  doc["TEMP"] = chipTemp;
  doc["WIFI_SSID"] = wifiSSID.length() > 0 ? wifiSSID.c_str() : nullptr;
  doc["WIFI_OK"] = WiFi.status() == WL_CONNECTED;
  doc["FW"] = FIRMWARE_VERSION;
  doc["FW_B"] = FIRMWARE_BUILD;
  doc["OTA_OK"] = nullptr;
  doc["SPD"] = speedPercent;
  doc["JIN"] = jingleStyle;
  doc["LED"] = ledBrightness;
  doc["DIR"] = motorDirection;
  doc["POW"] = motorStrength;
  doc["TRQ"] = highSpeedMode ? 0 : torqueLimit;
  doc["WGT"] = targetWeight;
  doc["DUR"] = calibrationAt80Speed / 1000;
  doc["HS"] = highSpeedMode;
  doc["FAN_SPD"] = fanSpeed;
  doc["FAN_ON"] = (fanAlwaysOn || isMotorRunning || millis() < fanStopAfter);
  doc["FAN_ALW"] = fanAlwaysOn;
  doc["SV_R"] = servoAngleR;
  doc["SV_L"] = servoAngleL;
  doc["SV_STP"] = servoStepMm;
  doc["SV_HOME"] = servoHomeIsR ? "R" : "L";
  doc["SV_MMD"] = roundf(servoMmPerDeg * 10000.0f) / 10000.0f;
  doc["SV_W"] = roundf(spoolWidthMm * 10.0f) / 10.0f;
  doc["SV_DW"] = roundf(guideDwellRevs * 10.0f) / 10.0f;
  doc["SPOOL"] = activeSpool;
  doc["VAR"] = (boardVariant == 2) ? "PRO" : (boardVariant == 1) ? "STD"
                                                                 : "UNK";

  lastIsMotorRunning = isMotorRunning;
  lastFilamentDetected = filamentDetected;
  lastProgress = progVal;
  lastChipTemperature = chipTemp;
  lastRemainingTime = remVal;
  lastSpeedPercent = speedPercent;

  String jsonOut;
  serializeJson(doc, jsonOut);
  if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
  pCharacteristic->setValue(jsonOut.c_str());
  pCharacteristic->notify();
  if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
}

// ------------------- Stepper jingle/tones --------------------------
void playTone(unsigned int freqHz, unsigned long durationMs) {
  uint16_t prevCurrent = driver.rms_current();
  driver.rms_current(1400);
  unsigned long halfPeriod = 1000000UL / (2 * freqHz);
  unsigned long totalCycles = (durationMs * 1000UL) / (2 * halfPeriod);
  bool dirState = false;
  for (unsigned long i = 0; i < totalCycles; i++) {
    dirState = !dirState;
    digitalWrite(DIR_PIN, dirState);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(40);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(halfPeriod * 2 - 40);
  }
  driver.rms_current(prevCurrent);
}
void playStepperJingle() {
  if (jingleStyle == 0) return;
  delay(500);
  stepperStop();
  ledc_stop(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
  gpio_reset_pin((gpio_num_t)STEP_PIN);
  gpio_set_direction((gpio_num_t)STEP_PIN, GPIO_MODE_OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  {
    led.setPixelColor(0, led.Color(0, pwmValue, 0));
    led.show();
  }
  switch (jingleStyle) {
    case 1:  // Simple
      playTone(1000, 140);
      delay(20);
      playTone(800, 140);
      delay(20);
      playTone(1200, 400);
      break;
    case 2:  // Glissando
      playTone(523, 220);
      delay(20);
      playTone(587, 70);
      delay(20);
      playTone(659, 70);
      delay(20);
      playTone(698, 70);
      delay(20);
      playTone(784, 70);
      delay(20);
      playTone(880, 70);
      delay(20);
      playTone(988, 100);
      delay(20);
      playTone(1047, 400);
      break;
    case 3:  // Star Wars
      playTone(440, 400);
      delay(40);
      playTone(440, 400);
      delay(40);
      playTone(440, 400);
      delay(40);
      playTone(349, 250);
      delay(40);
      playTone(523, 150);
      delay(40);
      playTone(440, 400);
      delay(40);
      playTone(349, 250);
      delay(40);
      playTone(523, 150);
      delay(40);
      playTone(440, 500);
      break;
  }
  digitalWrite(EN_PIN, HIGH);
  {
    ledc_channel_config_t step_ledc_chan = {
      .gpio_num = STEP_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = STEP_LEDC_CHANNEL,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = STEP_LEDC_TIMER,
      .duty = 0,
      .hpoint = 0
    };
    ledcChannelConfigSafe(&step_ledc_chan);
  }
  stepperStop();
}

// ------------------ Wireless update pushed from a computer ------------------
// `pio run -e ota -t upload` sends the new build over Wi-Fi (espota, UDP/TCP 3232). The board only
// listens while it is not winding, so a push during a job fails cleanly with "No response from device".
// The transfer runs inside ArduinoOTA.handle(), i.e. it blocks loop(); onProgress keeps the RTC
// watchdog fed meanwhile.
static bool arduinoOtaStarted = false;
static volatile bool arduinoOtaActive = false;

static WebServer diagServer(80);

static void diagHandleLog() {
  String out;
  out.reserve(DLOG_SIZE + 700);
  char hdr[640];
  snprintf(hdr, sizeof(hdr),
           "fw=%s up=%lus state=%c heap=%lu min_heap=%lu\n"
           "wifi rssi=%d dBm ch=%d disconnects=%lu last_reason=%d\n"
           "ble peers=%d disconnects=%lu last_reason=0x%03x itvl=%.1fms latency=%u timeout=%ums\n"
           "---- log (uptime s), oldest first ----\n",
           FIRMWARE_BUILD, millis() / 1000UL, currentState, (unsigned long)ESP.getFreeHeap(),
           (unsigned long)ESP.getMinFreeHeap(), WiFi.RSSI(), WiFi.channel(), (unsigned long)wifiDisconnects,
           wifiLastReason, NimBLEDevice::getServer() ? NimBLEDevice::getServer()->getConnectedCount() : 0,
           (unsigned long)bleDisconnects, bleLastReason, bleConnItvl * 1.25f, (unsigned)bleConnLatency,
           (unsigned)bleConnTimeout * 10U);
  out += hdr;
  portENTER_CRITICAL(&dlogMux);
  size_t head = dlogHead;
  bool wrapped = dlogWrapped;
  portEXIT_CRITICAL(&dlogMux);
  if (wrapped) {
    size_t i = head;
    while (i < DLOG_SIZE && dlogBuf[i] != '\n') i++;  // skip the partly overwritten first line
    for (i = i + 1; i < DLOG_SIZE; i++) out += dlogBuf[i];
  }
  for (size_t i = 0; i < head; i++) out += dlogBuf[i];
  diagServer.send(200, "text/plain; charset=utf-8", out);
}

static void arduinoOtaService() {
  if (WiFi.status() != WL_CONNECTED) return;
  static unsigned long lastHttpPoll = 0;
  if (arduinoOtaStarted && millis() - lastHttpPoll >= 25) {  // the page is rarely asked for; don't poll every pass
    lastHttpPoll = millis();
    diagServer.handleClient();
  }
  if (!arduinoOtaStarted) {
    diagServer.enableDelay(false);  // WebServer otherwise sleeps 1 ms per idle poll, i.e. in every loop() pass
    diagServer.on("/", diagHandleLog);
    diagServer.on("/log", diagHandleLog);
    diagServer.begin();
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.setRebootOnSuccess(true);
    ArduinoOTA.onStart([]() {
      arduinoOtaActive = true;
      isMotorRunning = false;
      shouldStartMotorNow = false;
      stepperStop();
      digitalWrite(STEP_PIN, LOW);
      digitalWrite(EN_PIN, HIGH);
      otaInProgress = true;
      currentState = 'U';
      sendStatus(true);
      if (otaLedPulseTaskHandle == nullptr) {
        xTaskCreate(otaLedPulseTask, "OtaLedPulse", 1024, NULL, 2, &otaLedPulseTaskHandle);
      }
      dlogln("#OTA push: receiving");
      rwdtFeed();
    });
    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
      rwdtFeed();
    });
    ArduinoOTA.onEnd([]() {
      dlogln("#OTA push: done, restarting");
      rwdtFeed();
    });
    ArduinoOTA.onError([](ota_error_t err) {
      dlogf("#OTA push: error %d\n", (int)err);
      arduinoOtaActive = false;
      otaInProgress = false;
      otaLedPulseTaskHandle = nullptr;
      currentState = 'I';
      sendStatus(true);
      rwdtFeed();
    });
    ArduinoOTA.begin();
    arduinoOtaStarted = true;
    dlogf("#OTA push ready: %s.local (%s) auth=%s\n", OTA_HOSTNAME, WiFi.localIP().toString().c_str(),
                  strlen(OTA_PASSWORD) > 0 ? "yes" : "no");
  }
  bool busy = isMotorRunning || shouldStartMotorNow || autoResumePending || currentState == 'R' ||
              (otaInProgress && !arduinoOtaActive);
  if (!busy) ArduinoOTA.handle();
}

// --------------------- BLE command handling -------------------------
// OTA source. The official server is the default. {"CMD":"OTA","URL":"http://..."} installs a build from
// any other address instead (e.g. a computer on the LAN); the URL is used once and not stored.
static const char* OTA_URL_OFFICIAL = "https://respooler.lts-design.com/Firmware/ESP32-WROOM-32_OTA.bin";
static char otaUrlOverride[200] = "";

static void notifyOtaResult(bool ok) {
  if (!deviceConnected || !pCharacteristic) return;
  StaticJsonDocument<64> doc;
  doc["OTA_OK"] = ok;
  String jsonOut;
  serializeJson(doc, jsonOut);
  if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
  pCharacteristic->setValue(jsonOut.c_str());
  pCharacteristic->notify();
  if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
}

void sendOTAUpdate() {
  if (otaInProgress) {
    return;
  }
  isMotorRunning = false;
  digitalWrite(STEP_PIN, LOW);
  stepperStop();
  digitalWrite(EN_PIN, HIGH);
  currentState = 'U';
  sendStatus(true);
  xTaskCreate([](void*) {
    String ota_url = otaUrlOverride[0] ? String(otaUrlOverride) : String(OTA_URL_OFFICIAL);
    otaUrlOverride[0] = 0;
    bool plainHttp = ota_url.startsWith("http://");
    WiFiClientSecure secureClient;
    secureClient.setInsecure();
    WiFiClient plainClient;
    WiFiClient& client = plainHttp ? plainClient : (WiFiClient&)secureClient;
    HTTPClient http;
    dlogf("#OTA from %s\n", ota_url.c_str());

    if (WiFi.status() != WL_CONNECTED || WiFi.localIP() == INADDR_NONE) {
      currentState = 'I';
      sendStatus(true);
      vTaskDelete(NULL);
    }

    otaInProgress = true;
    sendStatus(true);
    if (otaLedPulseTaskHandle == nullptr) {
      xTaskCreate(otaLedPulseTask, "OtaLedPulse", 1024, NULL, 2, &otaLedPulseTaskHandle);
    }

    http.begin(client, ota_url);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET();
    bool otaSuccess = false;
    dlogf("#OTA http=%d size=%d\n", httpCode, http.getSize());

    if (httpCode == HTTP_CODE_OK) {
      int contentLength = http.getSize();
      if (Update.begin(contentLength)) {
        WiFiClient* stream = http.getStreamPtr();
        while (http.connected() && Update.writeStream(*stream) > 0) {
          delay(1);
        }
        if (Update.end() && Update.isFinished()) {
          otaSuccess = true;
          if (deviceConnected && pCharacteristic) {
            StaticJsonDocument<64> doc;
            doc["OTA_OK"] = true;
            delay(100);
            sendStatus(true);
            String jsonOut;
            serializeJson(doc, jsonOut);
            if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
            pCharacteristic->setValue(jsonOut.c_str());
            pCharacteristic->notify();
            if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
            NimBLEDevice::deinit(true);
            delay(1000);
          }
          otaInProgress = false;
          otaLedPulseTaskHandle = nullptr;
          ESP.restart();
        }
      } else {
        if (deviceConnected && pCharacteristic) {
          StaticJsonDocument<64> doc;
          doc["OTA_OK"] = false;
          String jsonOut;
          serializeJson(doc, jsonOut);
          if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
          pCharacteristic->setValue(jsonOut.c_str());
          pCharacteristic->notify();
          if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
        }
        otaInProgress = false;
        otaLedPulseTaskHandle = nullptr;
        currentState = 'I';
        sendStatus(true);
        vTaskDelete(NULL);
      }
    } else {
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["OTA_OK"] = false;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
      otaInProgress = false;
      otaLedPulseTaskHandle = nullptr;
      currentState = 'I';
      sendStatus(true);
      vTaskDelete(NULL);
    }
    http.end();
    if (!otaSuccess) {
      dlogf("#OTA failed: %s\n", Update.errorString());
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["OTA_OK"] = false;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
    }
    otaInProgress = false;
    otaLedPulseTaskHandle = nullptr;
    currentState = 'I';
    sendStatus(true);
    vTaskDelete(NULL);
  },
              "OTAUpdateTask", 8192, NULL, 1, NULL);
}

// ------------------- Spool presets ------------------------
static void spoolPresetsSave() {
  prefs.begin("respooler", false);
  prefs.putBytes("spools1", spoolPresets, sizeof(spoolPresets));
  prefs.putChar("spoolSel", activeSpool);
  prefs.end();
}

// Keeps the selected preset in step with calibration changes made while it is selected.
static void spoolPresetSyncActive() {
  if (activeSpool < 0 || activeSpool >= SPOOL_PRESET_MAX || !spoolPresets[activeSpool].used) return;
  SpoolPreset& p = spoolPresets[activeSpool];
  if (p.angleL == servoAngleL && p.angleR == servoAngleR && fabsf(p.widthMm - spoolWidthMm) < 0.05f) return;
  p.angleL = (uint8_t)servoAngleL;
  p.angleR = (uint8_t)servoAngleR;
  p.widthMm = spoolWidthMm;
  spoolPresetsSave();
}

// {"SPOOLS":[{"I":0,"N":"Bambu","L":165,"R":25,"W":58.0},...],"SPOOL":0}, sent on request only
void sendSpools() {
  if (!deviceConnected || !pCharacteristic) return;
  StaticJsonDocument<768> doc;
  JsonArray arr = doc["SPOOLS"].to<JsonArray>();
  for (int i = 0; i < SPOOL_PRESET_MAX; i++) {
    if (!spoolPresets[i].used) continue;
    JsonObject o = arr.add<JsonObject>();
    o["I"] = i;
    o["N"] = spoolPresets[i].name;
    o["L"] = spoolPresets[i].angleL;
    o["R"] = spoolPresets[i].angleR;
    o["W"] = roundf(spoolPresets[i].widthMm * 10.0f) / 10.0f;
  }
  doc["SPOOL"] = activeSpool;
  String out;
  serializeJson(doc, out);
  if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
  pCharacteristic->setValue(out.c_str());
  pCharacteristic->notify();
  if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
}

void handleCommand(const std::string& cmd) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, cmd.c_str());
  if (error) return;

  if (doc.containsKey("CMD")) {
    String command = doc["CMD"].as<String>();
    if (command == "START") {
      if (!isMotorRunning) {
        shouldStartMotorNow = true;
        delayStartUntil = millis();
      }
    } else if (command == "STOP") {
      if (otaInProgress) {
        return;
      }
      if (currentState == 'A') {
        remHoldActive = true;
        remHoldExpiry = millis() + 200;
        progHoldActive = true;
        progHoldExpiry = millis() + 200;
        isMotorRunning = false;
        shouldStartMotorNow = false;
        pendingDirectionChange = false;
        triggerJingleNow = 0;
        progress = 0.0;
        totalEstimatedTime = 0;
        spoolingStartTime = 0;
        pausedElapsed = 0;
        filamentLostSince = 0;
        stepperStop();
        ledc_stop(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
        gpio_reset_pin((gpio_num_t)STEP_PIN);
        gpio_set_direction((gpio_num_t)STEP_PIN, GPIO_MODE_OUTPUT);
        digitalWrite(STEP_PIN, LOW);
        digitalWrite(EN_PIN, HIGH);
        {
          ledc_channel_config_t step_ledc_chan = {
            .gpio_num = STEP_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = STEP_LEDC_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = STEP_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0
          };
          ledcChannelConfigSafe(&step_ledc_chan);
        }
        currentState = 'I';
        sendStatus(true);
      } else {
        remHoldActive = true;
        remHoldExpiry = millis() + 200;
        progHoldActive = true;
        progHoldExpiry = millis() + 200;
        isMotorRunning = false;
        progress = 0.0;
        totalEstimatedTime = 0;
        spoolingStartTime = 0;
        pausedElapsed = 0;
        stepperStop();
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        currentState = 'I';
        sendStatus(true);
      }
    } else if (command == "PAUSE") {
      if (isMotorRunning) {
        isMotorRunning = false;
        stepperStop();
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        unsigned long elapsed = 0;
        if (spoolingStartTime > 0) {
          elapsed = millis() - spoolingStartTime;
        }
        pausedElapsed = elapsed;
        spoolingStartTime = 0;
        currentState = 'P';
        sendStatus(true);
      }
    } else if (command == "GET_SPOOLS") {
      sendSpools();
    } else if (command == "WIFI_SCAN") {
      if (wifiScanTaskHandle == nullptr) {
        xTaskCreate(wifiScanTask, "WiFiScan", 4096, NULL, 1, &wifiScanTaskHandle);
      }
    } else if (command == "OTA") {
      if (!otaInProgress) {
        const char* u = doc["URL"] | "";
        size_t n = strlen(u);
        if (n == 0) {
          otaUrlOverride[0] = 0;
        } else if (n < sizeof(otaUrlOverride) && (strncmp(u, "http://", 7) == 0 || strncmp(u, "https://", 8) == 0)) {
          memcpy(otaUrlOverride, u, n + 1);
        } else {
          notifyOtaResult(false);
          return;
        }
        sendOTAUpdate();
      }
    } else if (command == "WIFI_CONNECT") {
      if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
        WiFi.disconnect(true);
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
        wifiConnectInProgress = true;
        wifiConnectStartTime = millis();
      }
    }
  }

  if (doc.containsKey("SET")) {
    JsonObject set = doc["SET"].as<JsonObject>();
    bool changed = false;
    bool servoEndpointsChanged = false;
    bool spoolsChanged = false;
    if (set.containsKey("VAR")) {
      JsonVariant vj = set["VAR"];
      uint8_t newVar = boardVariant;
      if (vj.is<const char*>()) {
        const char* s = vj.as<const char*>();
        if (s && s[0]) {
          String up = String(s);
          up.toUpperCase();
          if (up == "PRO") newVar = 2;
          else if (up == "STD" || up == "STANDARD") newVar = 1;
          else if (up == "UNK" || up == "UNKNOWN") newVar = 0;
        }
      } else {
        int vi = vj.as<int>();
        if (vi < 0) vi = 0;
        if (vi > 2) vi = 2;
        newVar = (uint8_t)vi;
      }
      if (newVar != boardVariant) {
        boardVariant = newVar;
        changed = true;
      }
    }
    if (set.containsKey("DIR")) {
      int dir = set["DIR"];
      if (dir != motorDirection) {
        if (isMotorRunning) {
          pendingDirectionChange = true;
          newMotorDirection = dir;
          isMotorRunning = false;
          stepperStop();
          digitalWrite(EN_PIN, HIGH);
          digitalWrite(STEP_PIN, LOW);
        } else {
          motorDirection = dir;
          digitalWrite(DIR_PIN, motorDirection);
          prefs.begin("respooler", false);
          prefs.putUInt("dir", motorDirection);
          prefs.end();
        }
        changed = true;
      }
    }
    if (set.containsKey("LED")) {
      ledBrightness = constrain((int)set["LED"], 0, 100);
      pwmValue = calculatePWM(ledBrightness);
      prefs.begin("respooler", false);
      prefs.putUInt("led", ledBrightness);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("USE_FIL")) {
      useFilamentSensor = (int)set["USE_FIL"] != 0;
      prefs.begin("respooler", false);
      prefs.putBool("filamentSensor", useFilamentSensor);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("POW")) {
      motorStrength = constrain((int)set["POW"], 80, 120);
      setMotorCurrent(motorStrength);
      prefs.begin("respooler", false);
      prefs.putUInt("motorStrength", motorStrength);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("TRQ")) {
      int newTrq = constrain((int)set["TRQ"], 0, 3);
      if (highSpeedMode) {
        savedTorqueLimitHS = newTrq;
        prefs.begin("respooler", false);
        prefs.putUInt("torqueLimit", savedTorqueLimitHS);
        prefs.end();
        torqueLimit = 0;
      } else {
        torqueLimit = newTrq;
        prefs.begin("respooler", false);
        prefs.putUInt("torqueLimit", torqueLimit);
        prefs.end();
      }
      changed = true;
    }
    if (set.containsKey("JIN")) {
      int newJingleStyle = constrain((int)set["JIN"], 0, 3);
      if (newJingleStyle != jingleStyle) {
        jingleStyle = newJingleStyle;
        prefs.begin("respooler", false);
        prefs.putUInt("jingle", jingleStyle);
        prefs.end();
        if (!isMotorRunning) triggerJingleNow = jingleStyle;
        changed = true;
      }
    }
    if (set.containsKey("DUR")) {
      unsigned long dur = (unsigned long)set["DUR"];
      if (dur >= 10 && dur <= 20000) {
        calibrationAt80Speed = dur * 1000UL;
        prefs.begin("respooler", false);
        prefs.putULong("cal80_093", calibrationAt80Speed);
        prefs.end();
        changed = true;
      }
    }
    if (set.containsKey("WGT")) {
      int newWgt = constrain((int)set["WGT"], 0, 3);
      targetWeight = newWgt;
      prefs.begin("respooler", false);
      prefs.putUInt("targetWeight", (unsigned int)targetWeight);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("SPD")) {
      int speed = constrain((int)set["SPD"], 50, 100);
      double span = 170.0 - 93.0;
      double baseInterval = 170.0 - ((double)(speed - 50) * span / 50.0);
      baseTargetStepIntervalMicros = (unsigned long)lround(baseInterval);
      speedPercent = speed;
      prefs.begin("respooler", false);
      prefs.putUInt("speed", baseTargetStepIntervalMicros);
      prefs.end();
      applySpeedTarget();
      changed = true;
    }
    if (set.containsKey("HS")) {
      bool newHS = (int)set["HS"] != 0;
      if (newHS != highSpeedMode) {
        if (newHS) {
          highSpeedMode = true;
          savedTorqueLimitHS = torqueLimit;
          torqueLimit = 0;
          applySpeedTarget();
          prefs.begin("respooler", false);
          prefs.putBool("hsMode", true);
          prefs.end();
        } else {
          highSpeedMode = false;
          torqueLimit = savedTorqueLimitHS;
          applySpeedTarget();
          prefs.begin("respooler", false);
          prefs.putBool("hsMode", false);
          prefs.putUInt("torqueLimit", torqueLimit);
          prefs.end();
        }
        changed = true;
      }
    }
    if (set.containsKey("FAN_SPD")) {
      fanSpeed = constrain((int)set["FAN_SPD"], 0, 100);
      prefs.begin("respooler", false);
      prefs.putUInt("fanSpeed", fanSpeed);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("FAN_ALW")) {
      fanAlwaysOn = (int)set["FAN_ALW"] != 0;
      prefs.begin("respooler", false);
      prefs.putBool("fanAlways", fanAlwaysOn);
      prefs.end();
      changed = true;
    }
    if (set.containsKey("SV_R")) {
      int r = constrain((int)set["SV_R"], 0, 180);
      if (r != servoAngleR) {
        servoAngleR = r;
        prefs.begin("respooler", false);
        prefs.putUInt("servoR", (unsigned int)servoAngleR);
        prefs.end();
        servoEndpointsChanged = true;
        changed = true;
      }
      if (!isMotorRunning && (currentState == 'I' || currentState == 'D') && servoGotoSide == 'R') {
        servoGotoActive = true;
        servoHomingToHome = false;
        lastServoHomeStepTime = millis();
      }
    }
    if (set.containsKey("SV_L")) {
      int l = constrain((int)set["SV_L"], 0, 180);
      if (l != servoAngleL) {
        servoAngleL = l;
        prefs.begin("respooler", false);
        prefs.putUInt("servoL", (unsigned int)servoAngleL);
        prefs.end();
        servoEndpointsChanged = true;
        changed = true;
      }
      if (!isMotorRunning && (currentState == 'I' || currentState == 'D') && servoGotoSide == 'L') {
        servoGotoActive = true;
        servoHomingToHome = false;
        lastServoHomeStepTime = millis();
      }
    }
    if (set.containsKey("SV_STP")) {
      float s = (float)set["SV_STP"].as<float>();
      s = constrain(s, 0.05f, 20.0f);
      if (fabsf(s - servoStepMm) > 0.0001f) {
        servoStepMm = s;
        prefs.begin("respooler", false);
        prefs.putFloat("servoStep", servoStepMm);
        prefs.end();
        changed = true;
      }
    }
    if (set.containsKey("SV_MMD")) {
      float m = set["SV_MMD"].as<float>();
      if (m >= 0.1f && m <= 1.0f && fabsf(m - servoMmPerDeg) > 0.00001f) {
        servoMmPerDeg = m;
        prefs.begin("respooler", false);
        prefs.putFloat("svMmDeg", servoMmPerDeg);
        prefs.end();
        changed = true;
      }
    }
    if (set.containsKey("SV_DW")) {
      float d = set["SV_DW"].as<float>();
      if (d >= 0.0f && d <= 3.0f && fabsf(d - guideDwellRevs) > 0.001f) {
        guideDwellRevs = d;
        prefs.begin("respooler", false);
        prefs.putFloat("svDwell", guideDwellRevs);
        prefs.end();
        changed = true;
      }
    }
    if (set.containsKey("SV_W")) {
      float w = set["SV_W"].as<float>();
      if (w >= 0.0f && w <= 300.0f && fabsf(w - spoolWidthMm) > 0.01f) {
        spoolWidthMm = w;
        prefs.begin("respooler", false);
        prefs.putFloat("spoolW", spoolWidthMm);
        prefs.end();
        changed = true;
      }
    }
    if (set.containsKey("SPOOL")) {
      int i = set["SPOOL"].as<int>();
      if (i == -1) {
        activeSpool = -1;
        spoolPresetsSave();
        changed = true;
      } else if (i >= 0 && i < SPOOL_PRESET_MAX && spoolPresets[i].used) {
        activeSpool = (int8_t)i;
        servoAngleL = spoolPresets[i].angleL;
        servoAngleR = spoolPresets[i].angleR;
        spoolWidthMm = spoolPresets[i].widthMm;
        prefs.begin("respooler", false);
        prefs.putUInt("servoL", (unsigned int)servoAngleL);
        prefs.putUInt("servoR", (unsigned int)servoAngleR);
        prefs.putFloat("spoolW", spoolWidthMm);
        prefs.putChar("spoolSel", activeSpool);
        prefs.end();
        servoEndpointsChanged = true;
        changed = true;
      }
    }
    if (set.containsKey("SPOOL_SAVE")) {
      JsonObject o = set["SPOOL_SAVE"].as<JsonObject>();
      int i = o["I"] | -1;
      if (i < 0) {  // first free slot
        for (int k = 0; k < SPOOL_PRESET_MAX; k++) {
          if (!spoolPresets[k].used) { i = k; break; }
        }
      }
      if (i >= 0 && i < SPOOL_PRESET_MAX) {
        SpoolPreset& p = spoolPresets[i];
        const char* n = o["N"] | "";
        memset(&p, 0, sizeof(p));
        strncpy(p.name, (n && n[0]) ? n : "Spool", sizeof(p.name) - 1);
        p.used = 1;
        p.angleL = (uint8_t)servoAngleL;
        p.angleR = (uint8_t)servoAngleR;
        p.widthMm = spoolWidthMm;
        activeSpool = (int8_t)i;
        spoolPresetsSave();
        spoolsChanged = true;
        changed = true;
      }
    }
    if (set.containsKey("SPOOL_DEL")) {
      int i = set["SPOOL_DEL"].as<int>();
      if (i >= 0 && i < SPOOL_PRESET_MAX && spoolPresets[i].used) {
        memset(&spoolPresets[i], 0, sizeof(SpoolPreset));
        if (activeSpool == i) activeSpool = -1;
        spoolPresetsSave();
        spoolsChanged = true;
        changed = true;
      }
    }
    if (set.containsKey("SV_L") || set.containsKey("SV_R") || set.containsKey("SV_W")) {
      spoolPresetSyncActive();
    }
    if (set.containsKey("SV_HOME")) {
      const char* v = set["SV_HOME"].as<const char*>();
      char c = (v && v[0]) ? v[0] : 0;
      if (c >= 'a' && c <= 'z') c = (char)(c - 32);
      if (c == 'R' || c == 'L') {
        bool homeR = (c == 'R');
        if (homeR != servoHomeIsR) {
          servoHomeIsR = homeR;
          prefs.begin("respooler", false);
          prefs.putBool("servoHomeR", servoHomeIsR);
          prefs.end();
          servoEndpointsChanged = true;
          changed = true;
        }
      }
    }
    if (set.containsKey("SV_GOTO") && !isMotorRunning && (currentState == 'I' || currentState == 'D')) {
      const char* v = set["SV_GOTO"].as<const char*>();
      char c = (v && v[0]) ? v[0] : 0;
      if (c >= 'a' && c <= 'z') c = (char)(c - 32);

      if (c == 'L' || c == 'R' || c == 'H') {
        servoGotoSide = c;
        servoGotoActive = true;
        servoHomingToHome = false;
        lastServoHomeStepTime = millis();
      }
    }
    if (servoEndpointsChanged && !isMotorRunning && (currentState == 'I' || currentState == 'D') && !(servoGotoSide == 'L' || servoGotoSide == 'R')) {
      guideServoTowardsL = servoHomeIsR;
      servoHomingToHome = true;
      lastServoHomeStepTime = millis();
    }
    if (set.containsKey("WIFI_SSID")) {
      wifiSSID = set["WIFI_SSID"].as<String>();
      changed = true;
    }
    if (set.containsKey("WIFI_PASS")) {
      wifiPassword = set["WIFI_PASS"].as<String>();
      changed = true;
    }
    if (changed) {
      prefs.begin("respooler", false);
      prefs.putString("ssid", wifiSSID);
      prefs.putString("pwd", wifiPassword);
      prefs.putUChar("variant", boardVariant);
      prefs.end();
      sendStatus(true);
      if (spoolsChanged) sendSpools();
    }
  }
}

// ------------------- BLE callback classes ------------------------
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& info) override {
    deviceConnected = true;
    // Ask the central for a 5 s supervision timeout (interval 30-50 ms, no latency; inside Apple's
    // limits). Typical defaults are 0.7-2 s, and the ESP32 shares one radio between Wi-Fi and BLE, so a
    // short gap in BLE connection events otherwise drops the link although the signal is strong.
    pServer->updateConnParams(info.getConnHandle(), 24, 40, 0, 500);
    bleConnItvl = info.getConnInterval();
    bleConnLatency = info.getConnLatency();
    bleConnTimeout = info.getConnTimeout();
    dlogf("#BLE connect peers=%d itvl=%.1fms timeout=%ums\n", pServer->getConnectedCount(),
          bleConnItvl * 1.25f, (unsigned)bleConnTimeout * 10U);
    if (pServer->getConnectedCount() < 2) {
      NimBLEDevice::startAdvertising();
    }
    xTaskCreate([](void*) {
      delay(1100);
      sendStatus(true);
      vTaskDelete(NULL);
    },
                "delayedSend", 2048, NULL, 1, NULL);
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo&, int reason) override {
    deviceConnected = pServer->getConnectedCount() > 0;
    bleDisconnects++;
    bleLastReason = reason;
    // NimBLE reports HCI reasons as 0x200 + code: 0x208 link timeout, 0x213 closed by the other side,
    // 0x216 closed locally, 0x23e connection failed to be established
    dlogf("#BLE disconnect reason=0x%03x peers=%d st=%c wifi=%s\n", reason, pServer->getConnectedCount(), currentState,
          wifiSuspended ? "off" : (WiFi.status() == WL_CONNECTED ? "on" : "--"));
    if (pServer->getConnectedCount() < 2) {
      NimBLEDevice::startAdvertising();
    }
  }
  void onConnParamsUpdate(NimBLEConnInfo& info) override {
    bleConnItvl = info.getConnInterval();
    bleConnLatency = info.getConnLatency();
    bleConnTimeout = info.getConnTimeout();
    dlogf("#BLE params itvl=%.1fms latency=%u timeout=%ums\n", bleConnItvl * 1.25f, (unsigned)bleConnLatency,
          (unsigned)bleConnTimeout * 10U);
  }
};
class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo&) override {
    std::string rx = pChar->getValue();
    if (!rx.empty()) handleCommand(rx);
  }
};

// -------------------------- Setup --------------------------------
void setup() {
  // A software reset (panic, esp_restart(), OTA reboot) does not reset the LEDC peripheral on the ESP32,
  // so a stuck channel would survive the reboot and hang the ledc_*_config() calls below.
  periph_module_reset(PERIPH_LEDC_MODULE);

  setCpuFrequencyMhz(CPU_FREQ_MHZ);
  brownout_ll_set_threshold(BROWNOUT_LEVEL);
  Serial.setTxBufferSize(2048);  // heartbeat lines must not block loop()
  Serial.begin(115200);
  bootAnalyze();  // reports the reset reason; may do a full RTC reset to break a boot loop
  rwdtStart();
  CRUMB(100);

  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  esp_coex_preference_set(ESP_COEX_PREFER_BT);
  pinMode(FILAMENT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_CONN_PIN, OUTPUT);

  pinMode(FAN_PIN, OUTPUT);

  ledc_timer_config_t servo_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_14_BIT,
    .timer_num = SERVO_LEDC_TIMER,
    .freq_hz = 50,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&servo_timer);

  ledc_channel_config_t servo_channel = {
    .gpio_num = SERVO_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = SERVO_LEDC_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = SERVO_LEDC_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&servo_channel);

  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = FAN_TIMER,
    .freq_hz = 35000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
    .gpio_num = FAN_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = FAN_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = FAN_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf);

  ledc_timer_config_t conn_led_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&conn_led_timer);

  ledc_channel_config_t conn_led_conf = {
    .gpio_num = LED_CONN_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&conn_led_conf);

  ledc_timer_config_t step_ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = STEP_LEDC_TIMER,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&step_ledc_timer);

  ledc_channel_config_t step_ledc_chan = {
    .gpio_num = STEP_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = STEP_LEDC_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = STEP_LEDC_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&step_ledc_chan);

  led.begin();
  led.clear();
  led.show();
  CRUMB(101);

  TMCSerial.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);

  driver.begin();
  driver.I_scale_analog(false);
  driver.toff(3);
  driver.microsteps(MICROSTEPPING);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  driver.SGTHRS(40);
  driver.semin(0);
  driver.TCOOLTHRS(0xFFFFF);
  uint16_t cool = driver.COOLCONF();
  driver.COOLCONF(cool | (1 << 13));
  CRUMB(102);

  prefs.begin("respooler", true);
  wifiSSID = prefs.getString("ssid", "");
  wifiPassword = prefs.getString("pwd", "");
  prefs.end();
  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      if (wifiSuspended) return;  // switched off on purpose while winding
      wifiDisconnects++;
      wifiLastReason = info.wifi_sta_disconnected.reason;
      dlogf("#WIFI disconnected reason=%d st=%c\n", wifiLastReason, currentState);
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      dlogf("#WIFI connected ip=%s rssi=%d ch=%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.channel());
    }
  });
  if (bootResumeMode == 'A' && WIFI_OFF_WHILE_WINDING) {
    // about to resume winding: leave the radio off, wifiPowerManage() turns it on afterwards
    WiFi.mode(WIFI_OFF);
    wifiSuspended = true;
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
      WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
      if (WIFI_TX_LIMIT) WiFi.setTxPower(WIFI_TX_POWER);
    }
  }
  CRUMB(103);
  bleNotifyMutex = xSemaphoreCreateMutex();
  loadSettings();
  setMotorCurrent(motorStrength);
  pwmValue = calculatePWM(ledBrightness);

  guideServoPos = (float)(servoHomeIsR ? servoAngleR : servoAngleL);
  guideServoTowardsL = servoHomeIsR;
  applyBootResume();  // an interrupted job puts the guide servo back where it was
  setServoAngleF(guideServoPos);

  NimBLEDevice::init(BOARD_NAME);
  NimBLEDevice::setPower(BLE_TX_POWER_DBM);
  NimBLEDevice::setMTU(512);
  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setName(BOARD_NAME);
  advData.addServiceUUID(SERVICE_UUID);

  std::string manufacturerData;
  if (boardVariant == 2) {
    manufacturerData = "\xFF\xFF\x02";
  } else {
    manufacturerData = "\xFF\xFF\x01";
  }
  advData.setManufacturerData(manufacturerData);

  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();
  CRUMB(104);
  setFanDuty(0);
  setConnLed(150);
  currentState = bootResumeMode ? 'P' : 'I';
  ledcSnapshot();
  printDiag(false);
  if (autoResumePending) {
    dlogf("#RESUME in %d s (hold the button 1 s or send STOP to cancel)\n", AUTO_RESUME_DELAY_MS / 1000);
  }
  CRUMB(105);
  rwdtFeed();
}

// ------------------- Main loop -----------------------------------
void loop() {
  unsigned long now = millis();
  healthTick(now);
  pollSerialCommands();
  arduinoOtaService();
  CRUMB(2);

  // ------------------ Wi-Fi connect async check -----------------
  CRUMB(3);
  if (wifiConnectInProgress) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnectInProgress = false;
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["WIFI_CONN_RESULT"] = true;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
      prefs.begin("respooler", false);
      prefs.putString("ssid", wifiSSID);
      prefs.putString("pwd", wifiPassword);
      prefs.end();
    } else if (millis() - wifiConnectStartTime > 10000) {
      wifiConnectInProgress = false;
      if (deviceConnected && pCharacteristic) {
        StaticJsonDocument<64> doc;
        doc["WIFI_CONN_RESULT"] = false;
        String jsonOut;
        serializeJson(doc, jsonOut);
        if (bleNotifyMutex) xSemaphoreTake(bleNotifyMutex, portMAX_DELAY);
        pCharacteristic->setValue(jsonOut.c_str());
        pCharacteristic->notify();
        if (bleNotifyMutex) xSemaphoreGive(bleNotifyMutex);
      }
    }
  }

  // ------------------ Filament sensor -----------------
  CRUMB(4);
  bool rawFilamentPresent = digitalRead(FILAMENT_PIN) == LOW;
  if (rawFilamentPresent) {
    filamentLostSince = 0;
    filamentDetected = true;
  } else {
    if (filamentLostSince == 0) {
      filamentLostSince = now;
    }
    if (filamentDetected && (now - filamentLostSince >= FILAMENT_LOSS_CONFIRM_MS)) {
      filamentDetected = false;
    }
  }

  if (useFilamentSensor && (currentState == 'P' || currentState == 'A') && !filamentDetected) {
    isMotorRunning = false;
    shouldStartMotorNow = false;
    pendingDirectionChange = false;
    triggerJingleNow = 0;
    remHoldActive = true;
    remHoldExpiry = millis() + 200;
    progHoldActive = true;
    progHoldExpiry = millis() + 200;
    progress = 0.0;
    totalEstimatedTime = 0;
    spoolingStartTime = 0;
    pausedElapsed = 0;
    stepperStop();
    digitalWrite(EN_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    currentState = 'I';
    sendStatus(true);
  }

  // ------------------ Motor/LED state -----------------
  CRUMB(5);
  if (!otaInProgress) {
    if (isMotorRunning) {
      if (now - lastLedPulseTime > 5) {
        lastLedPulseTime = now;
        ledPulseValue += ledPulseDirection * 1;
        if (ledPulseValue >= 255) {
          ledPulseValue = 255;
          ledPulseDirection = -1;
        } else if (ledPulseValue <= 0) {
          ledPulseValue = 0;
          ledPulseDirection = 1;
        }
        uint16_t scaled16 = (uint16_t)ledPulseValue * (uint16_t)pwmValue;
        uint8_t baseVal = scaled16 / 255;
        uint8_t remainder = scaled16 % 255;
        greenDitherAcc = (uint16_t)(greenDitherAcc + remainder);
        if (greenDitherAcc >= 255) {
          baseVal++;
          greenDitherAcc -= 255;
        }

        led.setPixelColor(0, led.Color(0, baseVal, 0));
        led.show();
      }
    } else if (currentState == 'P') {
      if (now - lastLedPulseTime > 5) {
        lastLedPulseTime = now;
        ledPulseValue += ledPulseDirection * 1;
        if (ledPulseValue >= 255) {
          ledPulseValue = 255;
          ledPulseDirection = -1;
        } else if (ledPulseValue <= 0) {
          ledPulseValue = 0;
          ledPulseDirection = 1;
        }
        uint16_t scaled16 = (uint16_t)ledPulseValue * (uint16_t)pwmValue;
        uint8_t baseVal = scaled16 / 255;
        uint8_t remainder = scaled16 % 255;
        greenDitherAcc = (uint16_t)(greenDitherAcc + remainder);
        if (greenDitherAcc >= 255) {
          baseVal++;
          greenDitherAcc -= 255;
        }
        led.setPixelColor(0, led.Color(baseVal, baseVal / 2, 0));
        led.show();
      }
    } else if (currentState == 'A') {
      if (millis() - lastLedToggle > 400) {
        lastLedToggle = now;
        ledState = !ledState;
        led.setPixelColor(0, ledState ? led.Color(pwmValue, 0, 0) : 0);
        led.show();
      }
    } else if (useFilamentSensor && filamentDetected) {
      led.setPixelColor(0, led.Color(0, pwmValue, 0));
      led.show();
    } else {
      led.setPixelColor(0, led.Color(0, 0, pwmValue));
      led.show();
    }
  }

  // ----------------- Torque Auto-Stop -----------------
  CRUMB(6);
  static unsigned long lastTorqueCheck = 0;
  static unsigned long torqueBelowStartTime = 0;
  if (torqueLimit > 0 && isMotorRunning && stepIntervalMicros == targetStepIntervalMicros) {
    if (now - lastTorqueCheck >= TORQUE_CHECK_INTERVAL_MS) {
      lastTorqueCheck = now;
      uint16_t sg = driver.SG_RESULT();
      if (sg > TORQUE_SG_IGNORE) {

        float strengthScale = 1.0;
        int baseLimit = 350 * strengthScale;
        int limit = 999;
        if (torqueLimit == 1) limit = baseLimit * TORQUE_SG_LIMIT_LOW;
        else if (torqueLimit == 2) limit = baseLimit * TORQUE_SG_LIMIT_MED;
        else if (torqueLimit == 3) limit = baseLimit * TORQUE_SG_LIMIT_HIGH;

        int currSpeedPercent = getSpeedPercentFromInterval(stepIntervalMicros);
        if (currSpeedPercent > 94) {
          limit = limit * 0.95;
        } else if (currSpeedPercent > 85) {
          limit = limit * 0.97;
        } else if (currSpeedPercent < 65) {
          limit = limit * 0.98;
        }

        if (sg < limit) {
          if (torqueBelowStartTime == 0) torqueBelowStartTime = now;
          else if (now - torqueBelowStartTime >= TORQUE_BELOW_MS) {
            isMotorRunning = false;
            currentState = 'A';
            sendStatus(true);
            digitalWrite(STEP_PIN, LOW);
            delay(100);
            digitalWrite(EN_PIN, HIGH);
            unsigned long elapsed = millis() - spoolingStartTime;
            spoolingStartTime = 0;
            pausedElapsed = elapsed;
            torqueBelowStartTime = 0;
            stepperStop();
            ledc_stop(LEDC_LOW_SPEED_MODE, STEP_LEDC_CHANNEL, 0);
            gpio_reset_pin((gpio_num_t)STEP_PIN);
            gpio_set_direction((gpio_num_t)STEP_PIN, GPIO_MODE_OUTPUT);
            digitalWrite(EN_PIN, LOW);
            digitalWrite(STEP_PIN, LOW);
            delay(120);
            {
              led.setPixelColor(0, led.Color(pwmValue, 0, 0));
              led.show();
            }
            for (int i = 0; i < 3; i++) {
              playTone(850, 250);
              delay(100);
            }
            digitalWrite(EN_PIN, HIGH);
            {
              ledc_channel_config_t step_ledc_chan = {
                .gpio_num = STEP_PIN,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = STEP_LEDC_CHANNEL,
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = STEP_LEDC_TIMER,
                .duty = 0,
                .hpoint = 0
              };
              ledcChannelConfigSafe(&step_ledc_chan);
            }
            stepperStop();
          }
        } else {
          torqueBelowStartTime = 0;
        }
      }
    }
  } else {
    torqueBelowStartTime = 0;
  }

  CRUMB(7);
  static unsigned long wgtFinishStart = 0;
  if (isMotorRunning && targetWeight >= 1 && targetWeight <= 3 && totalEstimatedTime > 0) {
    unsigned long elapsed = 0;
    if (spoolingStartTime > 0) elapsed = now - spoolingStartTime;
    else if (pausedElapsed > 0) elapsed = pausedElapsed;
    else if (currentState == 'A') elapsed = pausedElapsed;

    unsigned long effectiveTotal = totalEstimatedTime;
    if (targetWeight == 2) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_2);
    else if (targetWeight == 3) effectiveTotal = (unsigned long)(totalEstimatedTime * TARGET_WEIGHT_FACTOR_3);

    if (effectiveTotal == 0) {
      wgtFinishStart = 0;
    } else if (elapsed >= effectiveTotal) {
      if (!(useFilamentSensor && !filamentDetected)) {
        if (wgtFinishStart == 0) wgtFinishStart = now;
        if (now - wgtFinishStart >= 700) {
          isMotorRunning = false;
          digitalWrite(STEP_PIN, LOW);
          stepperStop();
          digitalWrite(EN_PIN, HIGH);
          totalEstimatedTime = 0;
          spoolingStartTime = 0;
          currentState = 'D';
          sendStatus(true);
          progress = 100.0f;
          sendStatus(true);
          doneHoldActive = true;
          doneHoldStart = millis();
          playStepperJingle();
          wgtFinishStart = 0;
        }
      } else {
        wgtFinishStart = 0;
      }
    } else {
      wgtFinishStart = 0;
    }
  }

  // ------------------ Respooler done -------------------
  CRUMB(8);
  if (isMotorRunning && useFilamentSensor && !filamentDetected && filamentLostSince > 0 && now - filamentLostSince > FILAMENT_LOSS_CONFIRM_MS) {
    isMotorRunning = false;
    digitalWrite(STEP_PIN, LOW);
    stepperStop();
    delay(40);
    digitalWrite(EN_PIN, HIGH);
    totalEstimatedTime = 0;
    spoolingStartTime = 0;
    currentState = 'D';
    sendStatus(true);
    progress = 100.0f;
    sendStatus(true);
    doneHoldActive = true;
    doneHoldStart = millis();
    delay(300);
    playStepperJingle();
  }
  if (doneHoldActive && (millis() - doneHoldStart >= 20000)) {
    progress = 0.0;
    currentState = 'I';
    sendStatus(true);
    doneHoldActive = false;
  }

  // ------- Direction change after stop -----------
  CRUMB(9);
  if (pendingDirectionChange && !isMotorRunning) {
    motorDirection = newMotorDirection;
    prefs.begin("respooler", false);
    prefs.putUInt("dir", motorDirection);
    prefs.end();
    digitalWrite(DIR_PIN, motorDirection);
    delayStartUntil = now + 1000;
    shouldStartMotorNow = true;
    pendingDirectionChange = false;
  }

  // ------------ Motor start logic ----------------
  CRUMB(10);
  if (shouldStartMotorNow && now >= delayStartUntil) {
    if (otaInProgress) {
      shouldStartMotorNow = false;
      return;
    }
    if (isMotorRunning) {
      shouldStartMotorNow = false;
    } else if (!useFilamentSensor || filamentDetected) {
      int currSpeedPercent = round((float)speedPercent * ((float)baseTargetStepIntervalMicros / (float)targetStepIntervalMicros));
      currSpeedPercent = constrain(currSpeedPercent, 50, 140);
      totalEstimatedTime = calibrationAt80Speed * (80.0 / currSpeedPercent);
      if (pausedElapsed > 0) {
        spoolingStartTime = millis() - pausedElapsed;
        pausedElapsed = 0;
      } else {
        spoolingStartTime = millis();
      }
      currentState = 'R';
      sendStatus(true);
      remStartHoldUntil = millis() + 800;
      doneHoldActive = false;
      stepperStop();
      reSanitizeDriver();
      digitalWrite(DIR_PIN, motorDirection);
      digitalWrite(EN_PIN, LOW);
      digitalWrite(STEP_PIN, LOW);
      delay(15);
      {
        uint16_t prevI = driver.rms_current();
        driver.en_spreadCycle(true);
        int boosted = prevI + prevI / 4;
        if (boosted > 1600) boosted = 1600;
        driver.rms_current(boosted);
        stepperStart(stepFreqFromIntervalUs(1800));
        delay(150);
        driver.rms_current(prevI);
        driver.en_spreadCycle(false);
      }
      stepIntervalMicros = START_INTERVAL_US;
      lastAccelUpdate = now;
      isMotorRunning = true;
      stepperStart(stepFreqFromIntervalUs(stepIntervalMicros));
    }
    shouldStartMotorNow = false;
  }

  // ------------ Button handling with debounce ------------
  CRUMB(11);
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastStableButtonState && now - lastDebounceTime > BUTTON_DEBOUNCE_MS) {
    lastDebounceTime = now;

    if (reading == LOW) {
      if (!isMotorRunning) {
        if (currentState == 'P' || currentState == 'A') {
          buttonPressSince = now;
          buttonLongPressHandled = false;
          buttonAwaitingDecision = true;
        } else if (useFilamentSensor && !filamentDetected) {
          if (!otaInProgress) {
            for (int i = 0; i < 5; i++) {
              led.setPixelColor(0, led.Color(0, 0, pwmValue));
              led.show();
              delay(100);
              led.setPixelColor(0, 0);
              led.show();
              delay(100);
            }
          }
        } else {
          shouldStartMotorNow = true;
          delayStartUntil = now;
        }
      } else {
        isMotorRunning = false;
        stepperStop();
        digitalWrite(EN_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        unsigned long elapsed = 0;
        if (spoolingStartTime > 0) {
          elapsed = millis() - spoolingStartTime;
        }
        pausedElapsed = elapsed;
        spoolingStartTime = 0;
        currentState = 'P';
        sendStatus(true);
      }
    } else {
      if (buttonAwaitingDecision) {
        if (!buttonLongPressHandled) {
          if (!(useFilamentSensor && !filamentDetected)) {
            shouldStartMotorNow = true;
            delayStartUntil = now;
          }
        }
        buttonAwaitingDecision = false;
      }
    }

    lastStableButtonState = reading;
  }
  if (buttonAwaitingDecision && !buttonLongPressHandled && (digitalRead(BUTTON_PIN) == LOW)) {
    if (now - buttonPressSince >= 1000) {
      remHoldActive = true;
      remHoldExpiry = millis() + 200;
      progHoldActive = true;
      progHoldExpiry = millis() + 200;
      isMotorRunning = false;
      progress = 0.0;
      totalEstimatedTime = 0;
      spoolingStartTime = 0;
      pausedElapsed = 0;
      stepperStop();
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(STEP_PIN, LOW);
      currentState = 'I';
      sendStatus(true);
      buttonLongPressHandled = true;
    }
  }

  // ----------- Motor acceleration (speed ramp) --------------
  CRUMB(12);
  if (isMotorRunning && now - lastAccelUpdate > ACCEL_UPDATE_INTERVAL) {
    lastAccelUpdate = now;
    if (stepIntervalMicros > targetStepIntervalMicros) {
      stepIntervalMicros -= ACCEL_STEP;
      if (stepIntervalMicros < targetStepIntervalMicros) stepIntervalMicros = targetStepIntervalMicros;
    } else if (stepIntervalMicros < targetStepIntervalMicros) {
      stepIntervalMicros += ACCEL_STEP;
      if (stepIntervalMicros > targetStepIntervalMicros) stepIntervalMicros = targetStepIntervalMicros;
    }
    if (stepIntervalMicros != lastIntervalMicros) {
      stepperSetFreq(stepFreqFromIntervalUs(stepIntervalMicros));
      lastIntervalMicros = stepIntervalMicros;
    }
    if (stepIntervalMicros == targetStepIntervalMicros) {
      int currSpeedPercent = round((float)speedPercent * ((float)baseTargetStepIntervalMicros / (float)stepIntervalMicros));
      currSpeedPercent = constrain(currSpeedPercent, 50, 140);
      unsigned long newTotal = calibrationAt80Speed * (80.0 / currSpeedPercent);
      if (totalEstimatedTime > 0 && newTotal != totalEstimatedTime) {
        unsigned long elapsed = millis() - spoolingStartTime;
        double progressRatio = (double)elapsed / totalEstimatedTime;
        spoolingStartTime = millis() - (unsigned long)(newTotal * progressRatio);
      }
      totalEstimatedTime = newTotal;
    }
  }

  // ------------- Connection LED animation ------------------
  CRUMB(13);
  if (!otaInProgress) {
    static int connLedPulse = 0;
    static int connLedDir = 2;

    if (!deviceConnected) {
      if (now - lastConnLedTime > 25) {
        lastConnLedTime = now;
        connLedPulse += connLedDir;
        if (connLedPulse >= LED_CONN_PULSE_MAX) {
          connLedPulse = LED_CONN_PULSE_MAX;
          connLedDir = -2;
        } else if (connLedPulse <= 0) {
          connLedPulse = 0;
          connLedDir = 2;
        }
        setConnLedDuty(LED_CONN_PULSE_MAX - connLedPulse);
      }
    } else {
      setConnLedDuty(LED_CONN_PULSE_MAX);
    }
  }

  // ---------------------- Jingle trigger ---------------------
  CRUMB(14);
  if (triggerJingleNow != 0) {
    playStepperJingle();
    triggerJingleNow = 0;
  }

  // ----------- Restart BLE advertising if disconnected --------------
  CRUMB(15);
  if (!deviceConnected && !NimBLEDevice::getAdvertising()->isAdvertising()) {
    NimBLEDevice::startAdvertising();
  }

  // ------------------ Guide servo movement -----------------
  CRUMB(16);
  // The guide moves continuously at one step (servoStepMm) per spool revolution, stays guideDwellRevs
  // revolutions at each end stop, then turns back. The stock firmware jumped a whole step at once at the
  // same spool angle every revolution and turned back immediately.
  if (isMotorRunning) {
    if (!servoWasRunning) {
      servoWasRunning = true;
      lastMotorStepAccTime = now;
      servoHomingToHome = false;
      servoGotoActive = false;
      // Start a fresh segment at the current guide position (no jump after a pause). Starting at the
      // end stop the guide is about to leave counts as arriving there: dwell first.
      motorStepAcc = 0.0f;
      guideSegTo = guideServoPos;
      float behind = guideServoTowardsL ? (float)servoAngleR : (float)servoAngleL;
      guideDwellPending = fabsf(guideServoPos - behind) < 0.05f;
      guideNextSegment();
    } else {
      unsigned long dtMs = now - lastMotorStepAccTime;
      if (dtMs > 0) {
        lastMotorStepAccTime = now;

        float stepsPerSec = (float)stepFreqFromIntervalUs(stepIntervalMicros);
        motorStepAcc += stepsPerSec * ((float)dtMs / 1000.0f);
        float stepsPerSpoolRev = (200.0f * (float)MICROSTEPPING) * (52.0f / 18.0f);

        float segSteps = guideSegRevs * stepsPerSpoolRev;
        while (motorStepAcc >= segSteps) {
          motorStepAcc -= segSteps;
          guideNextSegment();
          segSteps = guideSegRevs * stepsPerSpoolRev;
        }
        float frac = motorStepAcc / segSteps;
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
        guideServoPos = guideSegFrom + (guideSegTo - guideSegFrom) * frac;
        setServoAngleF(guideServoPos);
      }
    }
  } else {
    servoWasRunning = false;
  }

  if (currentState == 'I' && lastStateForServo != 'I') {
    guideServoTowardsL = servoHomeIsR;
    motorStepAcc = 0.0f;
    servoWasRunning = false;
    servoHomingToHome = true;
    lastServoHomeStepTime = now;
  }
  lastStateForServo = currentState;

  // ------------------ Respooler done ------------------
  CRUMB(17);
  if (currentState == 'D' && lastStateForDone != 'D') {
    servoHomeAfterDonePending = true;
  }
  if (currentState != 'D' && lastStateForDone == 'D') {
    servoHomeAfterDonePending = false;
  }
  lastStateForDone = currentState;

  if (servoHomeAfterDonePending && currentState == 'D' && !isMotorRunning) {
    guideServoTowardsL = servoHomeIsR;
    servoHomingToHome = true;
    lastServoHomeStepTime = now;
    servoHomeAfterDonePending = false;
  }

  // ------------------ Servo goto -----------------
  CRUMB(18);
  if (!isMotorRunning && servoGotoActive) {
    if (now - lastServoHomeStepTime >= 10) {
      lastServoHomeStepTime = now;

      float target;
      if (servoGotoSide == 'L') target = (float)servoAngleL;
      else if (servoGotoSide == 'R') target = (float)servoAngleR;
      else if (servoGotoSide == 'H') target = (float)(servoHomeIsR ? servoAngleR : servoAngleL);
      else {
        servoGotoActive = false;
        return;
      }

      float stepDeg = 1.0f;

      if (fabsf(guideServoPos - target) <= stepDeg) {
        guideServoPos = target;
        setServoAngleF(guideServoPos);
        servoGotoActive = false;
      } else {
        guideServoPos += (guideServoPos < target) ? stepDeg : -stepDeg;
        setServoAngleF(guideServoPos);
      }
    }
  }

  // ------------------ Servo homing -----------------
  CRUMB(19);
  if (!isMotorRunning && servoHomingToHome) {
    if (now - lastServoHomeStepTime >= 10) {
      lastServoHomeStepTime = now;

      float targetHome = (float)(servoHomeIsR ? servoAngleR : servoAngleL);
      float stepDeg = 1.0f;

      if (fabsf(guideServoPos - targetHome) <= stepDeg) {
        guideServoPos = targetHome;
        setServoAngleF(guideServoPos);
        servoHomingToHome = false;
      } else {
        guideServoPos += (guideServoPos < targetHome) ? stepDeg : -stepDeg;
        setServoAngleF(guideServoPos);
      }
    }
  }

  // ---------------- Fan control logic ----------------
  CRUMB(20);
  int fanPWM = (fanAlwaysOn || isMotorRunning || millis() < fanStopAfter) ? fanPWMFromSpeed(fanSpeed) : 0;
  setFanDuty(fanPWM);
  if (!isMotorRunning && fanPWM > 0 && fanStopAfter == 0) {
    fanStopAfter = millis() + 10000;
  }
  if (isMotorRunning) fanStopAfter = millis() + 10000;
  if (!fanAlwaysOn && !isMotorRunning && millis() >= fanStopAfter) {
    fanStopAfter = 0;
  }

  // ----------- Periodic Status-Update every 500 ms -----------
  CRUMB(21);
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 500) {
    sendStatus();
    lastStatusTime = millis();
  }
}