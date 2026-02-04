// Coolant_automation firmware sketch (initial draft)
// Safe defaults: valve closed on boot and on any error.

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "DFRobot_DS1307.h"

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// Pin map (Arduino UNO form-factor per AGENTS diagrams)
static const uint8_t PIN_LEVEL_MIN = 3;    // MIN level sensor (digital)
static const uint8_t PIN_LEVEL_MAX = 2;    // MAX level sensor (digital)
static const uint8_t PIN_FLOW_PULSE = 5;   // Flowmeter pulses (interrupt-capable INT0)
static const uint8_t PIN_PH_ANALOG = A0;   // pH sensor analog output
static const uint8_t PIN_SD_CS = 10;       // SD chip select (DFR0229 is SPI)
static const uint8_t PIN_VALVE_RELAY = 4;  // Relay control for valve
static const uint8_t PIN_STATUS_LED = 13;  // Built-in LED (optional status)

// Constants
const unsigned long PULSES_PER_LITER = 150;  // From Liquid Flow Sensor G1/2 wiki
const unsigned long MAX_FILL_TIME_MS = 50000;    // NEED_USER_INPUT: define timeout
const unsigned long MIN_STABLE_MS = 100;     // NEED_USER_INPUT: debounce MIN
const unsigned long MAX_STABLE_MS = 300;     // NEED_USER_INPUT: debounce MAX

// pH measurement (based on ph_Meter_Template.ino for SEN0161)
static const unsigned long PH_SESSION_EVERY_MS = 3UL * 60UL * 60UL * 1000UL;   // every 3 hours
static const unsigned long PH_SESSION_DURATION_MS = 1UL * 60UL * 60UL * 1000UL; // for 1 hour
static const unsigned long PH_SAMPLING_INTERVAL_MS = 20;   // template: 20ms
static const unsigned long PH_PRINT_INTERVAL_MS = 800;     // template: 800ms
static const unsigned long PH_SD_LOG_INTERVAL_MS = 60000;  // NEED_USER_INPUT: how often to log pH to SD during session
static const float PH_OFFSET = 0.00f;                      // NEED_USER_INPUT: calibration offset for your probe/system

// Status/state
enum SystemState { BOOT,
                   IDLE,
                   FILLING,
                   ERROR };
volatile unsigned long pulseCount = 0;
SystemState state = BOOT;
unsigned long lastReportedPulseCount = 0;
unsigned long lastFlowReportMs = 0;
bool rtcOk = false;
DFRobot_DS1307 rtc(&Wire, DS1307_I2C_ADDR);

// Placeholders for RTC/time and pH thresholds
// NEED_USER_INPUT: pH thresholds/policy; RTC library hookup.

// Interrupt for flow pulses
void IRAM_ATTR onFlowPulse() {
  pulseCount++;
}

// Helpers
void closeValve() {
  digitalWrite(PIN_VALVE_RELAY, LOW);  // assumes LOW = valve closed via relay
}

void openValve() {
  digitalWrite(PIN_VALVE_RELAY, HIGH);
}

double avergearray(int *arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) {  // less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0];
      max = arr[1];
    } else {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;  // arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  // arr>max
          max = arr[i];
        } else {
          amount += arr[i];  // min<=arr<=max
        }
      }  // if
    }  // for
    avg = (double)amount / (number - 2);
  }  // if
  return avg;
}

String readTimestamp() {
  if (!rtcOk) {
    return String("1970-01-01 00:00:00");
  }
  uint16_t t[7] = {0};
  rtc.getTime(t);
  char buf[24];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u",
           t[DFRobot_DS1307::eYR],
           t[DFRobot_DS1307::eMTH],
           t[DFRobot_DS1307::eDATE],
           t[DFRobot_DS1307::eHR],
           t[DFRobot_DS1307::eMIN],
           t[DFRobot_DS1307::eSEC]);
  return String(buf);
}

bool levelMinActive() {
  // Active LOW with INPUT_PULLUP wiring; NEED_USER_INPUT for real sensor polarity.
  return digitalRead(PIN_LEVEL_MIN) == LOW;
}

bool levelMaxActive() {
  // Active LOW with INPUT_PULLUP wiring; NEED_USER_INPUT for real sensor polarity.
  return digitalRead(PIN_LEVEL_MAX) == LOW;
}

bool debounceLevel(bool (*levelFn)(), unsigned long stableMs) {
  if (stableMs == 0) {
    return levelFn();
  }
  const unsigned long start = millis();
  while (millis() - start < stableMs) {
    if (!levelFn()) {
      return false;
    }
  }
  return true;
}

float samplePH() {
  // One-shot pH read (kept for FILLING log). IDLE autonomous monitoring uses the session sampler below.
  const int raw = analogRead(PIN_PH_ANALOG);
  const float voltage = (raw * 5.0f) / 1024.0f;
  return 3.5f * voltage + PH_OFFSET;
}

struct PhSessionState {
  static const int ARRAY_LENGTH = 40;  // template: 40
  int rawArray[ARRAY_LENGTH];
  int rawIndex = 0;
  bool sessionActive = false;

  unsigned long nextSessionAtMs = 0;
  unsigned long sessionStartMs = 0;
  unsigned long lastSampleMs = 0;
  unsigned long lastPrintMs = 0;
  unsigned long lastSdLogMs = 0;

  int lastRaw = 0;
  float lastVoltage = 0.0f;
  float lastPh = 0.0f;
};

PhSessionState phSession;

bool logPhSample(const String &ts, int raw, float voltage, float ph, const String &status) {
  File file = SD.open("/ph_log.csv", FILE_WRITE);
  if (!file) {
    return false;
  }
  file.print(ts);
  file.print(',');
  file.print(raw);
  file.print(',');
  file.print(voltage, 3);
  file.print(',');
  file.print(ph, 2);
  file.print(',');
  file.println(status);
  file.close();
  return true;
}

void phSessionStart(unsigned long now) {
  phSession.sessionActive = true;
  phSession.sessionStartMs = now;
  phSession.lastSampleMs = 0;
  phSession.lastPrintMs = 0;
  phSession.lastSdLogMs = 0;
  phSession.rawIndex = 0;
  for (int i = 0; i < PhSessionState::ARRAY_LENGTH; i++) {
    phSession.rawArray[i] = 0;
  }
  Serial.println(F("[PH] Session START (IDLE autonomous)"));
}

void phSessionStop(unsigned long now, const __FlashStringHelper *reason) {
  phSession.sessionActive = false;
  phSession.nextSessionAtMs = now + PH_SESSION_EVERY_MS;
  Serial.print(F("[PH] Session STOP: "));
  Serial.println(reason);
}

void tickPhSession(SystemState currentState, unsigned long now) {
  if (currentState != IDLE) {
    if (phSession.sessionActive) {
      phSessionStop(now, F("ABORTED_BY_STATE"));
    }
    return;
  }

  if (phSession.nextSessionAtMs == 0) {
    phSession.nextSessionAtMs = now + PH_SESSION_EVERY_MS;
  }

  if (!phSession.sessionActive) {
    if ((long)(now - phSession.nextSessionAtMs) >= 0) {
      phSessionStart(now);
    }
    return;
  }

  if (PH_SESSION_DURATION_MS > 0 && now - phSession.sessionStartMs >= PH_SESSION_DURATION_MS) {
    phSessionStop(now, F("DURATION_DONE"));
    return;
  }

  if (PH_SAMPLING_INTERVAL_MS > 0) {
    if (phSession.lastSampleMs == 0 || now - phSession.lastSampleMs >= PH_SAMPLING_INTERVAL_MS) {
      phSession.lastRaw = analogRead(PIN_PH_ANALOG);
      phSession.rawArray[phSession.rawIndex++] = phSession.lastRaw;
      if (phSession.rawIndex >= PhSessionState::ARRAY_LENGTH) {
        phSession.rawIndex = 0;
      }

      const float avgRaw = (float)avergearray(phSession.rawArray, PhSessionState::ARRAY_LENGTH);
      phSession.lastVoltage = (avgRaw * 5.0f) / 1024.0f;
      phSession.lastPh = 3.5f * phSession.lastVoltage + PH_OFFSET;

      phSession.lastSampleMs = now;
    }
  }

  if (PH_PRINT_INTERVAL_MS > 0) {
    if (phSession.lastPrintMs == 0 || now - phSession.lastPrintMs >= PH_PRINT_INTERVAL_MS) {
      Serial.print(F("[PH] Voltage:"));
      Serial.print(phSession.lastVoltage, 2);
      Serial.print(F(" pH:"));
      Serial.println(phSession.lastPh, 2);
      digitalWrite(PIN_STATUS_LED, digitalRead(PIN_STATUS_LED) ^ 1);
      phSession.lastPrintMs = now;
    }
  }

  if (PH_SD_LOG_INTERVAL_MS > 0) {
    if (phSession.lastSdLogMs == 0 || now - phSession.lastSdLogMs >= PH_SD_LOG_INTERVAL_MS) {
      const String ts = readTimestamp();
      if (!logPhSample(ts, phSession.lastRaw, phSession.lastVoltage, phSession.lastPh, "OK")) {
        Serial.println(F("[PH][ERR] SD log failed (/ph_log.csv)"));
      }
      phSession.lastSdLogMs = now;
    }
  }
}

bool logFillCycle(const String &startTs, const String &stopTs, unsigned long pulses, float ph, const String &status) {
  // NEED_USER_INPUT: SD error handling policy (retry/buffer).
  File file = SD.open("/fill_log.csv", FILE_WRITE);
  if (!file) {
    return false;
  }
  const float volumeLiters = (float)pulses / (float)PULSES_PER_LITER;
  file.print(startTs);
  file.print(',');
  file.print(stopTs);
  file.print(',');
  file.print(pulses);
  file.print(',');
  file.print(volumeLiters, 3);
  file.print(',');
  file.print(ph, 2);
  file.print(',');
  file.println(status);
  file.close();
  return true;
}

bool initSubsystems() {
  pinMode(PIN_VALVE_RELAY, OUTPUT);
  closeValve();

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  pinMode(PIN_LEVEL_MIN, INPUT_PULLUP);  // NEED_USER_INPUT: adjust for sensor type
  pinMode(PIN_LEVEL_MAX, INPUT_PULLUP);  // NEED_USER_INPUT: adjust for sensor type
  pinMode(PIN_FLOW_PULSE, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_PULSE), onFlowPulse, RISING);

  Wire.begin();  // RTC/I2C hub
  rtcOk = rtc.begin();
  if (!rtcOk) {
    Serial.println(F("[ERR] RTC begin failed"));
    return false;
  }
  Serial.println(F("[OK] RTC.begin"));

  if (!SD.begin(PIN_SD_CS)) {
    Serial.println(F("[ERR] SD.begin failed"));
    return false;
  }
  Serial.println(F("[OK] SD.begin"));
  return true;
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("[BOOT] Coolant automation start"));
  if (initSubsystems()) {
    Serial.println(F("[BOOT] Init OK -> IDLE"));
    state = IDLE;
  } else {
    Serial.println(F("[BOOT] Init FAIL -> ERROR"));
    state = ERROR;
  }
}

bool handleFilling() {
  const String startTs = readTimestamp();
  pulseCount = 0;
  Serial.println(F("[FILLING] Start, valve OPEN"));
  openValve();
  const unsigned long startMs = millis();

  while (true) {
    if (MAX_FILL_TIME_MS > 0 && millis() - startMs > MAX_FILL_TIME_MS) {
      closeValve();
      Serial.println(F("[FILLING] TIMEOUT, valve CLOSED"));
      const String stopTs = readTimestamp();
      const float ph = samplePH();
      logFillCycle(startTs, stopTs, pulseCount, ph, "TIMEOUT");
      return false;
    }
    if (debounceLevel(levelMaxActive, MAX_STABLE_MS)) {
      closeValve();
      Serial.print(F("[FILLING] MAX reached, pulses="));
      Serial.println(pulseCount);
      const String stopTs = readTimestamp();
      const float ph = samplePH();
      logFillCycle(startTs, stopTs, pulseCount, ph, "OK");
      return true;
    }
    // Idle briefly to yield
    delay(10);
  }
}

void loop() {
  const unsigned long now = millis();
  tickPhSession(state, now);

  if (pulseCount != lastReportedPulseCount || (now - lastFlowReportMs >= 1000)) {
    Serial.print(F("[FLOW] count="));
    Serial.println(pulseCount);
    lastReportedPulseCount = pulseCount;
    lastFlowReportMs = now;
  }

  switch (state) {
    case BOOT:
      // Should not stay here; init happens in setup
      Serial.println(F("[ERR] Unexpected BOOT state"));
      state = ERROR;
      break;
    case IDLE:
      closeValve();
      if (debounceLevel(levelMinActive, MIN_STABLE_MS)) {
        Serial.println(F("[IDLE] MIN active -> FILLING"));
        state = FILLING;
      }
      delay(50);  // simple polling interval
      break;
    case FILLING:
      if (handleFilling()) {
        Serial.println(F("[FILLING] Cycle OK -> IDLE"));
        state = IDLE;
      } else {
        Serial.println(F("[FILLING] ERROR -> ERROR state"));
        state = ERROR;
      }
      break;
    case ERROR:
      closeValve();
      Serial.println(F("[ERROR] Valve closed, waiting reset"));
      // Wait for manual reset (user can power-cycle or add button)
      delay(500);
      break;
  }
}
