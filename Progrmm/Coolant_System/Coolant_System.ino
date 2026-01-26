// Coolant_automation firmware sketch (initial draft)
// Safe defaults: valve closed on boot and on any error.

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
// #include <RTClib.h> // Add when RTC library is available

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// Pin map (Arduino UNO form-factor per AGENTS diagrams)
static const uint8_t PIN_LEVEL_MIN = 3;    // MIN level sensor (digital)
static const uint8_t PIN_LEVEL_MAX = 2;    // MAX level sensor (digital)
static const uint8_t PIN_FLOW_PULSE = 5;   // Flowmeter pulses (interrupt-capable)
static const uint8_t PIN_PH_ANALOG = A0;   // pH sensor analog output
static const uint8_t PIN_SD_CS = 10;       // SD chip select (DFR0229 is SPI)
static const uint8_t PIN_VALVE_RELAY = 4;  // Relay control for valve

// Constants
const unsigned long PULSES_PER_LITER = 150;  // From Liquid Flow Sensor G1/2 wiki
const unsigned long MAX_FILL_TIME_MS = 0;    // NEED_USER_INPUT: define timeout
const unsigned long MIN_STABLE_MS = 0;       // NEED_USER_INPUT: debounce MIN
const unsigned long MAX_STABLE_MS = 0;       // NEED_USER_INPUT: debounce MAX

// Status/state
enum SystemState { BOOT,
                   IDLE,
                   FILLING,
                   ERROR };
volatile unsigned long pulseCount = 0;
SystemState state = BOOT;

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

String readTimestamp() {
  // NEED_USER_INPUT: replace with RTC timestamp retrieval
  return String("1970-01-01 00:00:00");
}

bool levelMinActive() {
  return digitalRead(PIN_LEVEL_MIN) == HIGH;
}

bool levelMaxActive() {
  return digitalRead(PIN_LEVEL_MAX) == HIGH;
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
  const int raw = analogRead(PIN_PH_ANALOG);
  // NEED_USER_INPUT: calibration factors; using sample formula from DFR SEN0161
  const float voltage = (raw * 5.0f) / 1024.0f;
  return 3.5f * voltage;
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

  pinMode(PIN_LEVEL_MIN, INPUT_PULLUP);  // NEED_USER_INPUT: adjust for sensor type
  pinMode(PIN_LEVEL_MAX, INPUT_PULLUP);  // NEED_USER_INPUT: adjust for sensor type
  pinMode(PIN_FLOW_PULSE, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_PULSE), onFlowPulse, RISING);

  Wire.begin();  // RTC/I2C hub
  // TODO: init RTC when library available

  if (!SD.begin(PIN_SD_CS)) {
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(9600);
  if (initSubsystems()) {
    state = IDLE;
  } else {
    state = ERROR;
  }
}

bool handleFilling() {
  const String startTs = readTimestamp();
  pulseCount = 0;
  openValve();
  const unsigned long startMs = millis();

  while (true) {
    if (MAX_FILL_TIME_MS > 0 && millis() - startMs > MAX_FILL_TIME_MS) {
      closeValve();
      const String stopTs = readTimestamp();
      const float ph = samplePH();
      logFillCycle(startTs, stopTs, pulseCount, ph, "TIMEOUT");
      return false;
    }
    if (debounceLevel(levelMaxActive, MAX_STABLE_MS)) {
      closeValve();
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
  switch (state) {
    case BOOT:
      // Should not stay here; init happens in setup
      state = ERROR;
      break;
    case IDLE:
      closeValve();
      if (debounceLevel(levelMinActive, MIN_STABLE_MS)) {
        state = FILLING;
      }
      delay(50);  // simple polling interval
      break;
    case FILLING:
      if (handleFilling()) {
        state = IDLE;
      } else {
        state = ERROR;
      }
      break;
    case ERROR:
      closeValve();
      // Wait for manual reset (user can power-cycle or add button)
      delay(500);
      break;
  }
}
