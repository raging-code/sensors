#include <WiFi.h>
#include <WebSocketsClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HX711.h>
#include <Preferences.h>
#include <ModbusMaster.h>
#include <WebServer.h>

// ========== WiFi Configuration ==========
const char* ssid = "hellisliterallyengineering2"; 
const char* password = "ram123456789";

// PC Server for WebSocket
const char* pc_server = "192.168.18.8";
const int pc_websocket_port = 4572; 

// ESP32 Web Server for Calibration
WebServer espServer(80);
WebSocketsClient webSocket;
Preferences preferences;

// ========== Pin Definitions ==========
#define RELAY_SENSOR1 27
#define RELAY_SENSOR2 14
#define RELAY_SENSOR3 13
#define RELAY_WATER_PUMP 12
#define RELAY_SOLENOID 33

#define DO_PIN 32
#define PH_PIN 35
#define TDS_PIN 34
#define TURBIDITY_PIN 36
#define ONE_WIRE_BUS 5
#define LOADCELL_DOUT 25
#define LOADCELL_SCK 26
#define TRIG_PIN 18
#define ECHO_PIN 19
#define WATER_DETECT_PIN 39

#define RS485_CONTROL 4
#define RS485_RX 16
#define RS485_TX 17
#define AMMONIUM_ADDR 1

ModbusMaster ammoniumSensor;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
HX711 scale;

// ========== ADC Settings for Voltage Divider ==========
const float ADC_REF = 3.9;      // ESP32 reference voltage
const float ADC_MAX = 4095.0;   // 12-bit ADC max value
// Voltage divider: R1=10k, R2=20k, factor = (R1+R2)/R2 = 30k/20k = 1.5
const float VOLTAGE_DIVIDER_FACTOR = 1.5;

// Helper function to get actual sensor voltage
float getSensorVoltage(int pin) {
  int adc = analogRead(pin);
  float gpio_voltage = (adc * ADC_REF / ADC_MAX);
  return gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
}

// Helper function to get raw ADC (still useful for calibration)
int getRawADC(int pin) {
  return analogRead(pin);
}

// ========== Sensor Structure ==========
struct {
  float do_value = 0.0;
  float temperature = 0.0;
  float compensated_do = 0.0;
  float ph_value = 0.0;
  float tds_value = 0.0;
  float turbidity = 0.0;
  float ammonium = 0.0;
  float weight = 0.0;
  float water_level_cm = 0.0;
  bool water_detected = false;
  
  bool do_ok = false;
  bool ph_ok = false;
  bool tds_ok = false;
  bool turbidity_ok = false;
  bool ammonium_ok = false;
  bool temp_ok = false;
  bool loadcell_ok = false;
  bool waterlevel_ok = false;
} sensors;

// ========== Calibration Variables ==========
float do_zero_raw = 0.0, do_100_raw = 0.0;
bool do_calibrated = false;
unsigned long last_do_print = 0;
bool do_cal_printing = false;
enum DOCalState { DO_IDLE, DO_WAITING_ZERO, DO_WAITING_100 };
DOCalState do_cal_state = DO_IDLE;

float ph_zero_raw = 0.0, ph_slope_raw = 0.0;
bool ph_calibrated = false;
unsigned long last_ph_print = 0;
bool ph_cal_printing = false;
enum PHCalState { PH_IDLE, PH_WAITING_6_86, PH_WAITING_4_01 };
PHCalState ph_cal_state = PH_IDLE;

float tds_15ppt_raw = 0.0, tds_25ppt_raw = 0.0;
bool tds_calibrated = false;
unsigned long last_tds_print = 0;
bool tds_cal_printing = false;
enum TDSCalState { TDS_IDLE, TDS_WAITING_15PPT, TDS_WAITING_25PPT };
TDSCalState tds_cal_state = TDS_IDLE;

float turbidity_zero_raw = 0.0, turbidity_1000_raw = 0.0;
bool turbidity_calibrated = false;
unsigned long last_turb_print = 0;
bool turb_cal_printing = false;
enum TurbCalState { TURB_IDLE, TURB_WAITING_ZERO, TURB_WAITING_1000 };
TurbCalState turb_cal_state = TURB_IDLE;

float cal_factor = -7050.0;
float empty_dist = 0.0, full_dist = 0.0;
bool empty_cal = false, full_cal = false;
int solenoid_time = 10;

// ========== Test State Machine ==========
enum TestState {
  IDLE, SENSOR_RELAY_ON, PUMP_ON, WAIT_WATER, STABILIZE, RECORD, SENSOR_RELAY_OFF, SOLENOID_ON, DONE
};
TestState state = IDLE;
unsigned long state_start = 0;

// ========== Timing ==========
unsigned long last_data_send = 0;
unsigned long last_temp_read = 0;
unsigned long last_ammonium_read = 0;
bool temp_waiting = false;
bool status_printed = false;

// Forward declarations
void allSensorRelays(bool on);
void setRelay(int relay, bool on);
void sendSensorData();
void cancelTest();

// ========== Helper Functions ==========
void printAction(const char* action) {
  Serial.printf("[ACTION] %s at %lu ms\n", action, millis());
}

void printTestStep(int step, const char* description) {
  Serial.printf("\n========================================\n");
  Serial.printf("📍 TEST STEP %d: %s\n", step, description);
  Serial.printf("⏱️  Time: %lu ms\n", millis());
  Serial.printf("========================================\n\n");
}

float getSaturatedDO(float tempC) {
  return 14.6 - (0.394 * tempC) + (0.00714 * tempC * tempC);
}

// ========== MODIFIED: pH Reading with Voltage Divider ==========
float adcToPH(int adc, float tempC) {
  float gpio_voltage = (adc * ADC_REF / ADC_MAX);
  float sensor_voltage = gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
  
  if (!ph_calibrated || ph_slope_raw <= ph_zero_raw) {
    // Fallback calculation using sensor voltage
    return (7.0 - sensor_voltage) / 0.18;
  }
  
  float kelvin = 273.15 + tempC;
  float current_slope = 0.1984 * kelvin / 1000.0;
  float intercept = 7.0 + (current_slope * 7.0);
  float ph = (intercept - sensor_voltage) / current_slope;
  return constrain(ph, 0.0, 14.0);
}

// ========== MODIFIED: TDS Reading with Voltage Divider ==========
float readTDS() {
  if (!sensors.tds_ok) return 0.0;
  int adc = analogRead(TDS_PIN);
  float gpio_voltage = (adc * ADC_REF / ADC_MAX);
  float sensor_voltage = gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
  
  // If calibrated with both points, use linear interpolation on ADC values
  if (tds_calibrated && tds_25ppt_raw > tds_15ppt_raw) {
    // Linear interpolation between 15ppt (15000 ppm) and 25ppt (25000 ppm)
    float tds_value = 15000.0 + (adc - tds_15ppt_raw) * (10000.0) / (tds_25ppt_raw - tds_15ppt_raw);
    return constrain(tds_value, 0.0, 50000.0);
  }
  
  // Fallback: rough estimate based on sensor voltage
  // Typical TDS sensor: 0ppm = 0V, 1000ppm = 1V, etc.
  float tds_estimate = sensor_voltage * 1000.0;
  return constrain(tds_estimate, 0.0, 50000.0);
}

// ========== MODIFIED: DO Reading with Voltage Divider ==========
float readDO() {
  if (!sensors.do_ok) return 0.0;
  int adc = analogRead(DO_PIN);
  
  if (do_calibrated && sensors.temp_ok && do_100_raw > do_zero_raw) {
    float saturated = getSaturatedDO(sensors.temperature);
    return constrain((adc - do_zero_raw) / (do_100_raw - do_zero_raw) * saturated, 0.0, 20.0);
  }
  
  // Fallback: estimate based on sensor voltage
  float gpio_voltage = (adc * ADC_REF / ADC_MAX);
  float sensor_voltage = gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
  return constrain((sensor_voltage / 5.0) * 20.0, 0.0, 20.0);
}

// ========== MODIFIED: Turbidity Reading with Voltage Divider ==========
float readTurbidity() {
  if (!sensors.turbidity_ok) return 0.0;
  int adc = analogRead(TURBIDITY_PIN);
  
  if (turbidity_calibrated && turbidity_1000_raw > turbidity_zero_raw) {
    return constrain((adc - turbidity_zero_raw) / (turbidity_1000_raw - turbidity_zero_raw) * 1000.0, 0.0, 4000.0);
  }
  
  // Fallback: estimate based on sensor voltage
  float gpio_voltage = (adc * ADC_REF / ADC_MAX);
  float sensor_voltage = gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
  return constrain(5000 - (sensor_voltage * 1125), 0.0, 4000.0);
}

float readWeight() {
  if (!sensors.loadcell_ok) return 0.0;
  return scale.is_ready() ? scale.get_units(3) : sensors.weight;
}

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 0.0;
  float dist = duration * 0.0343 / 2;
  return (dist > 0 && dist < 500) ? dist : 0.0;
}

float getWaterPercent() {
  if (!empty_cal || !full_cal) return 0.0;
  float depth = empty_dist - sensors.water_level_cm;
  float total = empty_dist - full_dist;
  return total <= 0 ? 0.0 : constrain((depth / total) * 100.0, 0.0, 100.0);
}

void readAmmonium() {
  uint8_t result = ammoniumSensor.readInputRegisters(0x0000, 1);
  if (result == ammoniumSensor.ku8MBSuccess) {
    sensors.ammonium = ammoniumSensor.getResponseBuffer(0) / 10.0;
    sensors.ammonium_ok = true;
  } else {
    sensors.ammonium_ok = false;
  }
}

void checkConnections() {
  sensors.do_ok = (analogRead(DO_PIN) > 10 && analogRead(DO_PIN) < 4090);
  sensors.ph_ok = (analogRead(PH_PIN) > 10 && analogRead(PH_PIN) < 4090);
  sensors.tds_ok = (analogRead(TDS_PIN) > 10 && analogRead(TDS_PIN) < 4090);
  sensors.turbidity_ok = (analogRead(TURBIDITY_PIN) > 10 && analogRead(TURBIDITY_PIN) < 4090);
  sensors.temp_ok = (ds18b20.getDeviceCount() > 0);
  sensors.loadcell_ok = scale.is_ready();
  float test = readUltrasonic();
  sensors.waterlevel_ok = (test > 0 && test < 500);
  sensors.water_detected = digitalRead(WATER_DETECT_PIN) == HIGH;
  
  if (!status_printed) {
    Serial.println("\n[STATUS] Sensor Connection Check:");
    Serial.printf("  DO Sensor: %s\n", sensors.do_ok ? "Connected" : "Not Connected");
    Serial.printf("  pH Sensor: %s\n", sensors.ph_ok ? "Connected" : "Not Connected");
    Serial.printf("  TDS Sensor: %s\n", sensors.tds_ok ? "Connected" : "Not Connected");
    Serial.printf("  Turbidity Sensor: %s\n", sensors.turbidity_ok ? "Connected" : "Not Connected");
    Serial.printf("  Temperature: %s\n", sensors.temp_ok ? "Connected" : "Not Connected");
    Serial.printf("  Load Cell: %s\n", sensors.loadcell_ok ? "Connected" : "Not Connected");
    status_printed = true;
  }
}

void sendSensorData() {
  String json = "{";
  json += "\"do\":" + String(sensors.do_ok ? sensors.do_value : -1, 2) + ",";
  json += "\"temperature\":" + String(sensors.temp_ok ? sensors.temperature : -1, 2) + ",";
  json += "\"compensated_do\":" + String(sensors.do_ok ? sensors.compensated_do : -1, 2) + ",";
  json += "\"ph\":" + String(sensors.ph_ok ? sensors.ph_value : -1, 2) + ",";
  json += "\"tds\":" + String(sensors.tds_ok ? sensors.tds_value : -1, 0) + ",";
  json += "\"turbidity\":" + String(sensors.turbidity_ok ? sensors.turbidity : -1, 0) + ",";
  json += "\"ammonium\":" + String(sensors.ammonium_ok ? sensors.ammonium : -1, 2) + ",";
  json += "\"weight\":" + String(sensors.loadcell_ok ? sensors.weight : -1, 2) + ",";
  json += "\"water_level\":" + String(sensors.waterlevel_ok ? sensors.water_level_cm : -1, 1) + ",";
  json += "\"water_percent\":" + String(getWaterPercent(), 1) + ",";
  json += "\"water_detected\":" + String(sensors.water_detected ? "true" : "false") + ",";
  json += "\"test_state\":" + String(state);
  json += "}";
  webSocket.sendTXT(json);
}

void setRelay(int relay, bool on) {
  int pin;
  switch(relay) {
    case 1: pin = RELAY_SENSOR1; break;
    case 2: pin = RELAY_SENSOR2; break;
    case 3: pin = RELAY_SENSOR3; break;
    case 4: pin = RELAY_WATER_PUMP; break;
    case 5: pin = RELAY_SOLENOID; break;
    default: return;
  }
  digitalWrite(pin, on ? LOW : HIGH);
  char msg[50];
  sprintf(msg, "Relay %d: %s", relay, on ? "ON" : "OFF");
  printAction(msg);
}

void allSensorRelays(bool on) {
  setRelay(1, on);
  setRelay(2, on);
  setRelay(3, on);
}

void startTest() {
  if (state != IDLE) {
    Serial.println("Test already running!");
    return;
  }
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     TEST SEQUENCE STARTED             ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  state = SENSOR_RELAY_ON;
  state_start = millis();
}

void cancelTest() {
    if (state != IDLE) {
        Serial.println("\n╔════════════════════════════════════════╗");
        Serial.println("║     TEST CANCELLED BY USER            ║");
        Serial.println("╚════════════════════════════════════════╝\n");
        
        setRelay(4, false);
        setRelay(5, false);
        state = IDLE;
        webSocket.sendTXT("TEST_CANCELLED");
        Serial.println("✅ All systems stopped. Test cancelled.");
        Serial.println("ℹ️ Sensor relays remain ON for continuous monitoring.");
    } else {
        Serial.println("No active test to cancel.");
    }
}

void updateTestMachine() {
  if (state == IDLE) return;
  
  switch(state) {
    case SENSOR_RELAY_ON:
      if (millis() - state_start > 500) {
        printTestStep(1, "Verify Sensor Relays are ON");
        state = PUMP_ON;
        state_start = millis();
      }
      break;
      
    case PUMP_ON:
      if (millis() - state_start > 500) {
        printTestStep(2, "Turn ON Water Pump Relay");
        setRelay(4, true);
        state = WAIT_WATER;
        state_start = millis();
      }
      break;
      
    case WAIT_WATER:
      if (millis() - state_start == 0) {
        Serial.println("\n🔄 STEP 3: Water detection sensor monitoring...");
      }
      if (sensors.water_detected) {
        Serial.println("\n✅ Water detected successfully!");
        setRelay(4, false);
        printTestStep(4, "Turn OFF Water Pump Relay");
        state = STABILIZE;
        state_start = millis();
        Serial.println("\n⏳ STEP 5: Waiting 10 minutes for sensor stabilization...");
      } else if (millis() - state_start > 30000) {
        Serial.println("\n⚠️ WARNING: No water detected after 30 seconds!");
        Serial.println("Continuing test sequence without water...");
        setRelay(4, false);
        printTestStep(4, "Turn OFF Water Pump Relay (No water detected)");
        state = STABILIZE;
        state_start = millis();
        Serial.println("\n⏳ STEP 5: Waiting 10 minutes for sensor stabilization...");
      } else if ((millis() - state_start) % 5000 == 0 && (millis() - state_start) > 0) {
        Serial.printf("   ⏳ Waiting for water... (%lu seconds elapsed)\n", (millis() - state_start)/1000);
      }
      break;
      
    case STABILIZE:
      if (millis() - state_start >= 600000) {
        printTestStep(6, "Record ALL sensor readings");
        state = RECORD;
        state_start = millis();
      } else {
        int elapsed_min = (millis() - state_start) / 60000;
        if (elapsed_min > 0 && (millis() - state_start) % 60000 < 100) {
          Serial.printf("   ⏳ Stabilization: %d minute(s) elapsed / 10 minutes\n", elapsed_min);
        }
      }
      break;
      
    case RECORD:
      Serial.println("\n📊 Recording sensor data...");
      sendSensorData();
      webSocket.sendTXT("RECORD_READING");
      printTestStep(7, "Display last 5 recordings on dashboard");
      state = SENSOR_RELAY_OFF;
      state_start = millis();
      break;
      
    case SENSOR_RELAY_OFF:
      if (millis() - state_start > 500) {
        printTestStep(8, "Sensor relays remain ON for continuous monitoring");
        state = SOLENOID_ON;
        state_start = millis();
      }
      break;
      
    case SOLENOID_ON:
      printTestStep(9, "Turn ON Solenoid Valve Relay (drain water)");
      setRelay(5, true);
      state = DONE;
      state_start = millis();
      printTestStep(10, "Wait for configured time");
      Serial.printf("   ⏱️  Solenoid ON for %d seconds\n", solenoid_time);
      break;
      
    case DONE:
      if (millis() - state_start >= (unsigned long)solenoid_time * 1000) {
        printTestStep(11, "Turn OFF Solenoid Valve Relay");
        setRelay(5, false);
        state = IDLE;
        webSocket.sendTXT("TEST_COMPLETE");
        Serial.println("\n╔════════════════════════════════════════╗");
        Serial.println("║     TEST SEQUENCE COMPLETE!          ║");
        Serial.println("╚════════════════════════════════════════╝\n");
        Serial.println("ℹ️ Sensor relays remain ON for continuous monitoring.");
      }
      break;
  }
}

void updateSensors() {
  sensors.do_value = readDO();
  sensors.ph_value = readPH();
  sensors.tds_value = readTDS();
  sensors.turbidity = readTurbidity();
  sensors.weight = readWeight();
  
  if (sensors.waterlevel_ok) {
    sensors.water_level_cm = readUltrasonic();
  }
  sensors.water_detected = digitalRead(WATER_DETECT_PIN) == HIGH;
  
  if (sensors.temp_ok) {
    if (!temp_waiting || (millis() - last_temp_read > 750)) {
      sensors.temperature = ds18b20.getTempCByIndex(0);
      ds18b20.requestTemperatures();
      last_temp_read = millis();
      temp_waiting = true;
    }
  }
  
  if (millis() - last_ammonium_read > 3000) {
    readAmmonium();
    last_ammonium_read = millis();
  }
  
  // Print continuous readings during calibration with voltage info
  unsigned long now = millis();
  
  if (do_cal_printing && do_cal_state != DO_IDLE && (now - last_do_print > 1000)) {
    int adc = analogRead(DO_PIN);
    float gpio_voltage = (adc * ADC_REF / ADC_MAX);
    float sensor_voltage = gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
    Serial.printf("📊 DO | ADC: %4d | GPIO: %.2f V | Sensor: %.2f V | DO: %.2f mg/L\n", 
                  adc, gpio_voltage, sensor_voltage, sensors.do_value);
    last_do_print = now;
  }
  
  if (ph_cal_printing && ph_cal_state != PH_IDLE && (now - last_ph_print > 1000)) {
    int adc = analogRead(PH_PIN);
    float gpio_voltage = (adc * ADC_REF / ADC_MAX);
    float sensor_voltage = gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
    Serial.printf("📊 pH | ADC: %4d | GPIO: %.2f V | Sensor: %.2f V | pH: %.2f\n", 
                  adc, gpio_voltage, sensor_voltage, sensors.ph_value);
    last_ph_print = now;
  }
  
  if (tds_cal_printing && tds_cal_state != TDS_IDLE && (now - last_tds_print > 1000)) {
    int adc = analogRead(TDS_PIN);
    float gpio_voltage = (adc * ADC_REF / ADC_MAX);
    float sensor_voltage = gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
    float estimated_ppm = sensor_voltage * 1000;
    Serial.printf("📊 TDS | ADC: %4d | GPIO: %.2f V | Sensor: %.2f V | Est: %.0f ppm | Target: %s\n", 
                  adc, gpio_voltage, sensor_voltage, estimated_ppm,
                  tds_cal_state == TDS_WAITING_15PPT ? "15,000 ppm (15ppt)" : "25,000 ppm (25ppt)");
    last_tds_print = now;
  }
  
  if (turb_cal_printing && turb_cal_state != TURB_IDLE && (now - last_turb_print > 1000)) {
    int adc = analogRead(TURBIDITY_PIN);
    float gpio_voltage = (adc * ADC_REF / ADC_MAX);
    float sensor_voltage = gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
    Serial.printf("📊 Turbidity | ADC: %4d | GPIO: %.2f V | Sensor: %.2f V | NTU: %.0f\n", 
                  adc, gpio_voltage, sensor_voltage, sensors.turbidity);
    last_turb_print = now;
  }
}

// ========== Calibration Functions ==========
void startDOCalibration() {
  do_cal_state = DO_WAITING_ZERO;
  do_cal_printing = true;
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     DO CALIBRATION STARTED            ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println("Step 1: Place sensor in ZERO OXYGEN solution");
}

void setDOZeroPoint() {
  if (do_cal_state != DO_WAITING_ZERO) return;
  float sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(DO_PIN); delay(50); }
  do_zero_raw = sum / 10.0;
  Serial.printf("Zero point saved! ADC = %.2f\n", do_zero_raw);
  Serial.println("Step 2: Place sensor in OPEN AIR for 100% point");
  do_cal_state = DO_WAITING_100;
}

void setDO100Point() {
  if (do_cal_state != DO_WAITING_100) return;
  float sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(DO_PIN); delay(50); }
  do_100_raw = sum / 10.0;
  do_calibrated = true;
  preferences.putFloat("do_zero_raw", do_zero_raw);
  preferences.putFloat("do_100_raw", do_100_raw);
  preferences.putBool("do_calibrated", true);
  Serial.println("DO CALIBRATION COMPLETE!");
  do_cal_state = DO_IDLE;
  do_cal_printing = false;
}

void resetDOCalibration() {
  do_calibrated = false;
  do_cal_printing = false;
  preferences.putBool("do_calibrated", false);
  Serial.println("DO calibration reset");
}

void startPHCalibration() {
  ph_cal_state = PH_WAITING_6_86;
  ph_cal_printing = true;
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     pH CALIBRATION STARTED            ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println("Step 1: Place sensor in pH 6.86 buffer solution");
}

void setPH_6_86_Point() {
  if (ph_cal_state != PH_WAITING_6_86) return;
  float sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(PH_PIN); delay(50); }
  ph_zero_raw = sum / 10.0;
  Serial.printf("pH 6.86 point saved! ADC = %.2f\n", ph_zero_raw);
  Serial.println("Step 2: Place sensor in pH 4.01 buffer solution");
  ph_cal_state = PH_WAITING_4_01;
}

void setPH_4_01_Point() {
  if (ph_cal_state != PH_WAITING_4_01) return;
  float sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(PH_PIN); delay(50); }
  ph_slope_raw = sum / 10.0;
  ph_calibrated = true;
  preferences.putFloat("ph_zero_raw", ph_zero_raw);
  preferences.putFloat("ph_slope_raw", ph_slope_raw);
  preferences.putBool("ph_calibrated", true);
  Serial.println("pH CALIBRATION COMPLETE!");
  ph_cal_state = PH_IDLE;
  ph_cal_printing = false;
}

void resetPHCalibration() {
  ph_calibrated = false;
  ph_cal_printing = false;
  preferences.putBool("ph_calibrated", false);
  Serial.println("pH calibration reset");
}

void startTDSCalibration() {
  tds_cal_state = TDS_WAITING_15PPT;
  tds_cal_printing = true;
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     TDS CALIBRATION STARTED           ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println("📌 Step 1: Place sensor in 15 ppt (15000 ppm) solution");
  Serial.println("💡 Wait for reading to stabilize (30 seconds), then click 'Set 15ppt'");
  Serial.println("📊 Current ADC reading will show below:");
}

void setTDS_15PPT_Point() {
  if (tds_cal_state != TDS_WAITING_15PPT) return;
  
  float sum = 0;
  for (int i = 0; i < 20; i++) { 
    sum += analogRead(TDS_PIN); 
    delay(100);
  }
  tds_15ppt_raw = sum / 20.0;
  
  Serial.printf("\n✅ 15ppt point saved! ADC = %.2f\n", tds_15ppt_raw);
  float gpio_voltage = (tds_15ppt_raw * ADC_REF / ADC_MAX);
  float sensor_voltage = gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
  Serial.printf("📊 GPIO Voltage: %.2f V | Sensor Voltage: %.2f V\n", gpio_voltage, sensor_voltage);
  Serial.println("\n📌 Step 2: Place sensor in 25 ppt (25000 ppm) solution");
  Serial.println("💡 Rinse sensor with distilled water first, then place in 25ppt solution");
  Serial.println("💡 Wait for reading to stabilize (30 seconds), then click 'Set 25ppt'");
  tds_cal_state = TDS_WAITING_25PPT;
}

void setTDS_25PPT_Point() {
  if (tds_cal_state != TDS_WAITING_25PPT) return;
  
  float sum = 0;
  for (int i = 0; i < 20; i++) { 
    sum += analogRead(TDS_PIN); 
    delay(100);
  }
  tds_25ppt_raw = sum / 20.0;
  
  Serial.printf("\n✅ 25ppt point saved! ADC = %.2f\n", tds_25ppt_raw);
  float gpio_voltage = (tds_25ppt_raw * ADC_REF / ADC_MAX);
  float sensor_voltage = gpio_voltage * VOLTAGE_DIVIDER_FACTOR;
  Serial.printf("📊 GPIO Voltage: %.2f V | Sensor Voltage: %.2f V\n", gpio_voltage, sensor_voltage);
  
  if (tds_25ppt_raw > tds_15ppt_raw) {
    tds_calibrated = true;
    preferences.putFloat("tds_15ppt_raw", tds_15ppt_raw);
    preferences.putFloat("tds_25ppt_raw", tds_25ppt_raw);
    preferences.putBool("tds_calibrated", true);
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║     TDS CALIBRATION COMPLETE!        ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.printf("📈 Calibration range: %.0f ADC points difference\n", tds_25ppt_raw - tds_15ppt_raw);
  } else {
    Serial.println("\n⚠️ ERROR: 25ppt ADC value is not higher than 15ppt!");
    Serial.println("⚠️ Please check your sensor connections and solution concentrations");
    Serial.println("⚠️ Try calibration again");
    tds_calibrated = false;
  }
  
  tds_cal_state = TDS_IDLE;
  tds_cal_printing = false;
}

void resetTDSCalibration() {
  tds_calibrated = false;
  tds_cal_printing = false;
  tds_15ppt_raw = 0.0;
  tds_25ppt_raw = 0.0;
  preferences.putBool("tds_calibrated", false);
  preferences.putFloat("tds_15ppt_raw", 0.0);
  preferences.putFloat("tds_25ppt_raw", 0.0);
  Serial.println("TDS calibration reset");
}

void startTurbidityCalibration() {
  turb_cal_state = TURB_WAITING_ZERO;
  turb_cal_printing = true;
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     TURBIDITY CALIBRATION STARTED     ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println("Step 1: Place sensor in DISTILLED WATER (0 NTU)");
}

void setTurbidityZeroPoint() {
  if (turb_cal_state != TURB_WAITING_ZERO) return;
  float sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(TURBIDITY_PIN); delay(50); }
  turbidity_zero_raw = sum / 10.0;
  Serial.printf("0 NTU point saved! ADC = %.2f\n", turbidity_zero_raw);
  Serial.println("Step 2: Place sensor in 1000 NTU standard solution");
  turb_cal_state = TURB_WAITING_1000;
}

void setTurbidity1000Point() {
  if (turb_cal_state != TURB_WAITING_1000) return;
  float sum = 0;
  for (int i = 0; i < 10; i++) { sum += analogRead(TURBIDITY_PIN); delay(50); }
  turbidity_1000_raw = sum / 10.0;
  turbidity_calibrated = true;
  preferences.putFloat("turbidity_zero_raw", turbidity_zero_raw);
  preferences.putFloat("turbidity_1000_raw", turbidity_1000_raw);
  preferences.putBool("turbidity_calibrated", true);
  Serial.println("TURBIDITY CALIBRATION COMPLETE!");
  turb_cal_state = TURB_IDLE;
  turb_cal_printing = false;
}

void resetTurbidityCalibration() {
  turbidity_calibrated = false;
  turb_cal_printing = false;
  preferences.putBool("turbidity_calibrated", false);
  Serial.println("Turbidity calibration reset");
}

void updateLoadCellFactor(float new_factor) {
  cal_factor = new_factor;
  preferences.putFloat("weight_factor", cal_factor);
  scale.set_scale(cal_factor);
  Serial.printf("Factor updated to: %.2f\n", cal_factor);
}

void adjustLoadCellFactor(int delta) {
  cal_factor += delta;
  updateLoadCellFactor(cal_factor);
}

float readPH() {
  if (!sensors.ph_ok) return 0.0;
  int adc = analogRead(PH_PIN);
  if (adc <= 10 || adc >= 4090) { sensors.ph_ok = false; return 0.0; }
  return sensors.temp_ok ? adcToPH(adc, sensors.temperature) : adcToPH(adc, 25.0);
}

void wsEvent(WStype_t type, uint8_t* payload, size_t len) {
  switch(type) {
    case WStype_CONNECTED:
      Serial.println("[WEBSOCKET] ✅ Connected to PC server");
      break;
    case WStype_DISCONNECTED:
      Serial.println("[WEBSOCKET] ❌ Disconnected from PC server");
      break;
    case WStype_TEXT:
      String msg = String((char*)payload);
      Serial.printf("[WEBSOCKET] Received: %s\n", msg.c_str());
      if (msg == "START_TEST") {
        Serial.println("[WEBSOCKET] START_TEST command received!");
        startTest();
      } else if (msg == "CANCEL_TEST") {
        Serial.println("[WEBSOCKET] CANCEL_TEST command received!");
        cancelTest();
      } else if (msg == "RECORD_READING") {
        webSocket.sendTXT("RECORD_CONFIRM");
      }
      break;
  }
}

// ========== Web Server with Relay Controls ==========
void sendHTML() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>ESP32 Control Panel</title>";
  html += "<style>";
  html += "body{font-family:Arial;margin:20px;background:#f0f0f0;}";
  html += ".container{max-width:800px;margin:auto;}";
  html += ".card{background:white;padding:20px;margin:10px 0;border-radius:10px;box-shadow:0 2px 5px rgba(0,0,0,0.1);}";
  html += "button{background:#007bff;color:white;border:none;padding:10px 20px;margin:5px;border-radius:5px;cursor:pointer;}";
  html += "button:hover{background:#0056b3;}";
  html += ".btn-on{background:#28a745;}";
  html += ".btn-on:hover{background:#218838;}";
  html += ".btn-off{background:#dc3545;}";
  html += ".btn-off:hover{background:#c82333;}";
  html += ".btn-success{background:#28a745;}";
  html += ".btn-danger{background:#dc3545;}";
  html += ".btn-secondary{background:#6c757d;}";
  html += ".cal-step{background:#e9ecef;padding:15px;border-radius:5px;margin-top:10px;text-align:center;}";
  html += ".reading{font-size:20px;font-weight:bold;text-align:center;margin:10px 0;}";
  html += ".connected{color:green;}.disconnected{color:red;}";
  html += "table{width:100%;}td{padding:8px;}";
  html += ".relay-row{display:flex;align-items:center;justify-content:space-between;margin:10px 0;padding:10px;background:#f8f9fa;border-radius:5px;}";
  html += ".relay-name{font-weight:bold;width:150px;}";
  html += ".relay-desc{color:#666;flex:1;}";
  html += ".factor-buttons{display:flex;flex-wrap:wrap;justify-content:center;gap:8px;margin:15px 0;}";
  html += ".factor-btn{background:#17a2b8;min-width:70px;}";
  html += ".sensor-status{background:#d4edda;color:#155724;padding:5px;border-radius:5px;margin-top:10px;text-align:center;}";
  html += ".warning{background:#fff3cd;color:#856404;padding:5px;border-radius:5px;margin:5px 0;font-size:12px;}";
  html += "</style></head><body>";
  html += "<div class='container'><h2>ESP32 Control Panel</h2>";
  
  html += "<div class='sensor-status'>🔌 Sensor Relays: ALWAYS ON (Continuous monitoring)</div>";
  html += "<div class='warning'>💡 Voltage Divider: 10k/20k (1.5x compensation applied)</div>";
  
  // Relay Control Section
  html += "<div class='card'><h3>🔌 Relay Controls</h3>";
  
  html += "<div class='relay-row'><span class='relay-name'>Relay 1</span><span class='relay-desc'>Sensor 1 Power (Always ON)</span>";
  html += "<button class='btn-on' onclick='controlRelay(1,1)'>ON</button>";
  html += "<button class='btn-off' onclick='controlRelay(1,0)'>OFF</button></div>";
  
  html += "<div class='relay-row'><span class='relay-name'>Relay 2</span><span class='relay-desc'>Sensor 2 Power (Always ON)</span>";
  html += "<button class='btn-on' onclick='controlRelay(2,1)'>ON</button>";
  html += "<button class='btn-off' onclick='controlRelay(2,0)'>OFF</button></div>";
  
  html += "<div class='relay-row'><span class='relay-name'>Relay 3</span><span class='relay-desc'>Sensor 3 Power (Always ON)</span>";
  html += "<button class='btn-on' onclick='controlRelay(3,1)'>ON</button>";
  html += "<button class='btn-off' onclick='controlRelay(3,0)'>OFF</button></div>";
  
  html += "<div class='relay-row'><span class='relay-name'>Relay 4</span><span class='relay-desc'>Water Pump</span>";
  html += "<button class='btn-on' onclick='controlRelay(4,1)'>ON</button>";
  html += "<button class='btn-off' onclick='controlRelay(4,0)'>OFF</button></div>";
  
  html += "<div class='relay-row'><span class='relay-name'>Relay 5</span><span class='relay-desc'>Solenoid Valve (Drain)</span>";
  html += "<button class='btn-on' onclick='controlRelay(5,1)'>ON</button>";
  html += "<button class='btn-off' onclick='controlRelay(5,0)'>OFF</button></div>";
  
  html += "</div>";
  
  html += "<div class='card'><h3>📡 Sensor Status</h3><div id='sensorStatus'></div></div>";
  html += "<div class='card'><h3>🌊 DO Sensor</h3><div class='reading'>DO: <span id='doCurrent'>--</span> mg/L | Temp: <span id='tempCurrent'>--</span>°C</div><div id='doCalStep'></div></div>";
  html += "<div class='card'><h3>🧪 pH Sensor</h3><div class='reading'>pH: <span id='phCurrent'>--</span></div><div id='phCalStep'></div></div>";
  html += "<div class='card'><h3>💧 TDS Sensor</h3><div class='reading'>TDS: <span id='tdsCurrent'>--</span> ppm (<span id='tdsPpt'>--</span> ppt)</div><div id='tdsCalStep'></div></div>";
  html += "<div class='card'><h3>🌊 Turbidity Sensor</h3><div class='reading'>Turbidity: <span id='turbCurrent'>--</span> NTU</div><div id='turbCalStep'></div></div>";
  
  html += "<div class='card'><h3>⚖️ Load Cell</h3>";
  html += "<div class='reading'>Weight: <span id='weightCurrent'>--</span> g</div>";
  html += "<div style='text-align:center;margin:10px 0;'><button class='btn-secondary' onclick='sendCommand(\"/cal/weight/tare\")'>⚖️ Tare (Zero)</button></div>";
  html += "<div class='factor-buttons'>";
  html += "<button class='factor-btn' onclick='adjustFactor(-1000)'>-1000</button>";
  html += "<button class='factor-btn' onclick='adjustFactor(-100)'>-100</button>";
  html += "<button class='factor-btn' onclick='adjustFactor(-10)'>-10</button>";
  html += "<button class='factor-btn' onclick='adjustFactor(-1)'>-1</button>";
  html += "<button class='factor-btn' onclick='adjustFactor(1)'>+1</button>";
  html += "<button class='factor-btn' onclick='adjustFactor(10)'>+10</button>";
  html += "<button class='factor-btn' onclick='adjustFactor(100)'>+100</button>";
  html += "<button class='factor-btn' onclick='adjustFactor(1000)'>+1000</button>";
  html += "</div><div>Current Factor: <strong id='currentFactor'>--</strong></div>";
  html += "<div style='font-size:12px;color:#666;margin-top:10px;'>💡 Tip: Place a known weight, adjust until reading matches</div></div>";
  
  html += "<div class='card'><h3>📏 Water Level</h3>";
  html += "<p>Current: <span id='wlCurrent'>--</span> cm</p>";
  html += "<button onclick='sendCommand(\"/cal/water/empty\")'>Set Empty Level</button>";
  html += "<button onclick='sendCommand(\"/cal/water/full\")'>Set Full Level</button></div></div>";
  
  html += "<script>";
  html += "let lastDoState=-1,lastPhState=-1,lastTdsState=-1,lastTurbState=-1;";
  html += "async function controlRelay(relay,state){let resp=await fetch('/relay/control?relay='+relay+'&state='+state);let data=await resp.json();alert(data.message);}";
  html += "async function sendCommand(url){await fetch(url);updateAll();}";
  html += "async function adjustFactor(delta){await fetch('/cal/weight/adjust?delta='+delta);updateAll();}";
  html += "async function updateAll(){try{";
  html += "let resp=await fetch('/sensor/values');let data=await resp.json();";
  html += "document.getElementById('doCurrent').innerHTML=data.do;";
  html += "document.getElementById('tempCurrent').innerHTML=data.temp;";
  html += "document.getElementById('phCurrent').innerHTML=data.ph;";
  html += "document.getElementById('tdsCurrent').innerHTML=data.tds;";
  html += "document.getElementById('tdsPpt').innerHTML=(data.tds/1000).toFixed(1);";
  html += "document.getElementById('turbCurrent').innerHTML=data.turbidity;";
  html += "document.getElementById('weightCurrent').innerHTML=data.weight;";
  html += "document.getElementById('wlCurrent').innerHTML=data.water_level;";
  html += "document.getElementById('currentFactor').innerHTML=data.cal_factor;";
  html += "let s='<table>';";
  html += "s+='<tr><td>DO:</td><td>'+(data.do==='--'?'❌ Not Connected':'✅ Connected')+'</td><td>'+(data.do_calibrated==='true'?'Calibrated':'Uncalibrated')+'</td></tr>';";
  html += "s+='<tr><td>pH:</td><td>'+(data.ph==='--'?'❌ Not Connected':'✅ Connected')+'</td><td>'+(data.ph_calibrated==='true'?'Calibrated':'Uncalibrated')+'</td></tr>';";
  html += "s+='<tr><td>TDS:</td><td>'+(data.tds==='--'?'❌ Not Connected':'✅ Connected')+'</td><td>'+(data.tds_calibrated==='true'?'Calibrated':'Uncalibrated')+'</td></tr>';";
  html += "s+='<tr><td>Turbidity:</td><td>'+(data.turbidity==='--'?'❌ Not Connected':'✅ Connected')+'</td><td>'+(data.turbidity_calibrated==='true'?'Calibrated':'Uncalibrated')+'</td></tr>';";
  html += "s+='<tr><td>Load Cell:</td><td>'+(data.weight==='--'?'❌ Not Connected':'✅ Connected')+'</td><td>Factor: '+data.cal_factor+'</td></tr>';";
  html += "s+='</table>';document.getElementById('sensorStatus').innerHTML=s;";
  html += "let ds=parseInt(data.do_cal_state);if(ds!==lastDoState){";
  html += "if(ds===1)document.getElementById('doCalStep').innerHTML='<div class=\"cal-step\"><h4>Step 1/2: Set ZERO Point</h4><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/do/setzero\\')\">Set Zero</button><button onclick=\"sendCommand(\\'/cal/do/cancel\\')\">Cancel</button></div>';";
  html += "else if(ds===2)document.getElementById('doCalStep').innerHTML='<div class=\"cal-step\"><h4>Step 2/2: Set 100% Point</h4><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/do/set100\\')\">Set 100%</button><button onclick=\"sendCommand(\\'/cal/do/cancel\\')\">Cancel</button></div>';";
  html += "else document.getElementById('doCalStep').innerHTML='<div><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/do/start\\')\">Start DO Cal</button><button onclick=\"sendCommand(\\'/cal/do/reset\\')\">Reset</button></div>';";
  html += "lastDoState=ds;}";
  html += "let ps=parseInt(data.ph_cal_state);if(ps!==lastPhState){";
  html += "if(ps===1)document.getElementById('phCalStep').innerHTML='<div class=\"cal-step\"><h4>Step 1/2: Set pH 6.86</h4><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/ph/set686\\')\">Set pH 6.86</button><button onclick=\"sendCommand(\\'/cal/ph/cancel\\')\">Cancel</button></div>';";
  html += "else if(ps===2)document.getElementById('phCalStep').innerHTML='<div class=\"cal-step\"><h4>Step 2/2: Set pH 4.01</h4><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/ph/set401\\')\">Set pH 4.01</button><button onclick=\"sendCommand(\\'/cal/ph/cancel\\')\">Cancel</button></div>';";
  html += "else document.getElementById('phCalStep').innerHTML='<div><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/ph/start\\')\">Start pH Cal</button><button onclick=\"sendCommand(\\'/cal/ph/reset\\')\">Reset</button></div>';";
  html += "lastPhState=ps;}";
  html += "let ts=parseInt(data.tds_cal_state);if(ts!==lastTdsState){";
  html += "if(ts===1)document.getElementById('tdsCalStep').innerHTML='<div class=\"cal-step\"><h4>Step 1/2: Set 15ppt (15000 ppm)</h4><p>Place sensor in 15ppt solution, wait 30 seconds</p><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/tds/set15ppt\\')\">Set 15ppt</button><button onclick=\"sendCommand(\\'/cal/tds/cancel\\')\">Cancel</button></div>';";
  html += "else if(ts===2)document.getElementById('tdsCalStep').innerHTML='<div class=\"cal-step\"><h4>Step 2/2: Set 25ppt (25000 ppm)</h4><p>Place sensor in 25ppt solution, wait 30 seconds</p><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/tds/set25ppt\\')\">Set 25ppt</button><button onclick=\"sendCommand(\\'/cal/tds/cancel\\')\">Cancel</button></div>';";
  html += "else document.getElementById('tdsCalStep').innerHTML='<div><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/tds/start\\')\">Start TDS Cal</button><button onclick=\"sendCommand(\\'/cal/tds/reset\\')\">Reset</button></div>';";
  html += "lastTdsState=ts;}";
  html += "let us=parseInt(data.turb_cal_state);if(us!==lastTurbState){";
  html += "if(us===1)document.getElementById('turbCalStep').innerHTML='<div class=\"cal-step\"><h4>Step 1/2: Set 0 NTU</h4><p>Place in DISTILLED WATER</p><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/turbidity/setzero\\')\">Set 0 NTU</button><button onclick=\"sendCommand(\\'/cal/turbidity/cancel\\')\">Cancel</button></div>';";
  html += "else if(us===2)document.getElementById('turbCalStep').innerHTML='<div class=\"cal-step\"><h4>Step 2/2: Set 1000 NTU</h4><p>Place in 1000 NTU STANDARD</p><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/turbidity/set1000\\')\">Set 1000 NTU</button><button onclick=\"sendCommand(\\'/cal/turbidity/cancel\\')\">Cancel</button></div>';";
  html += "else document.getElementById('turbCalStep').innerHTML='<div><button class=\"btn-success\" onclick=\"sendCommand(\\'/cal/turbidity/start\\')\">Start Turbidity Cal</button><button onclick=\"sendCommand(\\'/cal/turbidity/reset\\')\">Reset</button></div>';";
  html += "lastTurbState=us;}";
  html += "}catch(e){}}";
  html += "setInterval(updateAll,500);updateAll();";
  html += "</script></body></html>";
  
  espServer.send(200, "text/html", html);
}

void setupCalibrationRoutes() {
  espServer.on("/", sendHTML);
  
  espServer.on("/relay/control", []() {
    if (espServer.hasArg("relay") && espServer.hasArg("state")) {
      int relay = espServer.arg("relay").toInt();
      int state = espServer.arg("state").toInt();
      setRelay(relay, state == 1);
      espServer.send(200, "application/json", "{\"message\":\"Relay " + String(relay) + " turned " + String(state == 1 ? "ON" : "OFF") + "\"}");
    } else {
      espServer.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
    }
  });
  
  espServer.on("/sensor/values", []() {
    String json = "{";
    json += "\"do\":" + String(sensors.do_ok ? String(sensors.do_value, 2) : "\"--\"") + ",";
    json += "\"do_cal_state\":" + String((int)do_cal_state) + ",";
    json += "\"do_calibrated\":" + String(do_calibrated ? "true" : "false") + ",";
    json += "\"ph\":" + String(sensors.ph_ok ? String(sensors.ph_value, 2) : "\"--\"") + ",";
    json += "\"ph_cal_state\":" + String((int)ph_cal_state) + ",";
    json += "\"ph_calibrated\":" + String(ph_calibrated ? "true" : "false") + ",";
    json += "\"tds\":" + String(sensors.tds_ok ? String(sensors.tds_value, 0) : "\"--\"") + ",";
    json += "\"tds_cal_state\":" + String((int)tds_cal_state) + ",";
    json += "\"tds_calibrated\":" + String(tds_calibrated ? "true" : "false") + ",";
    json += "\"turbidity\":" + String(sensors.turbidity_ok ? String(sensors.turbidity, 0) : "\"--\"") + ",";
    json += "\"turb_cal_state\":" + String((int)turb_cal_state) + ",";
    json += "\"turbidity_calibrated\":" + String(turbidity_calibrated ? "true" : "false") + ",";
    json += "\"temp\":" + String(sensors.temp_ok ? String(sensors.temperature, 1) : "\"--\"") + ",";
    json += "\"weight\":" + String(sensors.loadcell_ok ? String(sensors.weight, 1) : "\"--\"") + ",";
    json += "\"water_level\":" + String(sensors.waterlevel_ok ? String(sensors.water_level_cm, 1) : "\"--\"") + ",";
    json += "\"cal_factor\":" + String(cal_factor);
    json += "}";
    espServer.send(200, "application/json", json);
  });
  
  espServer.on("/cal/do/start", []() { startDOCalibration(); espServer.send(200, "application/json", "{\"message\":\"Started\"}"); });
  espServer.on("/cal/do/setzero", []() { setDOZeroPoint(); espServer.send(200, "application/json", "{\"message\":\"Zero saved\"}"); });
  espServer.on("/cal/do/set100", []() { setDO100Point(); espServer.send(200, "application/json", "{\"message\":\"100% saved\"}"); });
  espServer.on("/cal/do/cancel", []() { do_cal_state = DO_IDLE; do_cal_printing = false; espServer.send(200, "application/json", "{\"message\":\"Cancelled\"}"); });
  espServer.on("/cal/do/reset", []() { resetDOCalibration(); espServer.send(200, "application/json", "{\"message\":\"Reset\"}"); });
  
  espServer.on("/cal/ph/start", []() { startPHCalibration(); espServer.send(200, "application/json", "{\"message\":\"Started\"}"); });
  espServer.on("/cal/ph/set686", []() { setPH_6_86_Point(); espServer.send(200, "application/json", "{\"message\":\"pH 6.86 saved\"}"); });
  espServer.on("/cal/ph/set401", []() { setPH_4_01_Point(); espServer.send(200, "application/json", "{\"message\":\"pH 4.01 saved\"}"); });
  espServer.on("/cal/ph/cancel", []() { ph_cal_state = PH_IDLE; ph_cal_printing = false; espServer.send(200, "application/json", "{\"message\":\"Cancelled\"}"); });
  espServer.on("/cal/ph/reset", []() { resetPHCalibration(); espServer.send(200, "application/json", "{\"message\":\"Reset\"}"); });
  
  espServer.on("/cal/tds/start", []() { startTDSCalibration(); espServer.send(200, "application/json", "{\"message\":\"Started\"}"); });
  espServer.on("/cal/tds/set15ppt", []() { setTDS_15PPT_Point(); espServer.send(200, "application/json", "{\"message\":\"15ppt saved\"}"); });
  espServer.on("/cal/tds/set25ppt", []() { setTDS_25PPT_Point(); espServer.send(200, "application/json", "{\"message\":\"25ppt saved\"}"); });
  espServer.on("/cal/tds/cancel", []() { tds_cal_state = TDS_IDLE; tds_cal_printing = false; espServer.send(200, "application/json", "{\"message\":\"Cancelled\"}"); });
  espServer.on("/cal/tds/reset", []() { resetTDSCalibration(); espServer.send(200, "application/json", "{\"message\":\"Reset\"}"); });
  
  espServer.on("/cal/turbidity/start", []() { startTurbidityCalibration(); espServer.send(200, "application/json", "{\"message\":\"Started\"}"); });
  espServer.on("/cal/turbidity/setzero", []() { setTurbidityZeroPoint(); espServer.send(200, "application/json", "{\"message\":\"0 NTU saved\"}"); });
  espServer.on("/cal/turbidity/set1000", []() { setTurbidity1000Point(); espServer.send(200, "application/json", "{\"message\":\"1000 NTU saved\"}"); });
  espServer.on("/cal/turbidity/cancel", []() { turb_cal_state = TURB_IDLE; turb_cal_printing = false; espServer.send(200, "application/json", "{\"message\":\"Cancelled\"}"); });
  espServer.on("/cal/turbidity/reset", []() { resetTurbidityCalibration(); espServer.send(200, "application/json", "{\"message\":\"Reset\"}"); });
  
  espServer.on("/cal/weight/tare", []() { scale.tare(10); espServer.send(200, "application/json", "{\"message\":\"Tared\"}"); });
  espServer.on("/cal/weight/adjust", []() {
    if (espServer.hasArg("delta")) {
      adjustLoadCellFactor(espServer.arg("delta").toInt());
      espServer.send(200, "application/json", "{\"message\":\"Adjusted\",\"factor\":" + String(cal_factor) + "}");
    }
  });
  espServer.on("/cal/weight/set", []() {
    if (espServer.hasArg("factor")) {
      updateLoadCellFactor(espServer.arg("factor").toFloat());
      espServer.send(200, "application/json", "{\"message\":\"Set\",\"factor\":" + String(cal_factor) + "}");
    }
  });
  
  espServer.on("/cal/water/empty", []() { empty_dist = sensors.water_level_cm; empty_cal = true; preferences.putFloat("empty_dist", empty_dist); espServer.send(200, "application/json", "{\"message\":\"Empty level set\"}"); });
  espServer.on("/cal/water/full", []() { full_dist = sensors.water_level_cm; full_cal = true; preferences.putFloat("full_dist", full_dist); espServer.send(200, "application/json", "{\"message\":\"Full level set\"}"); });
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     ESP32 Water Quality System       ║");
  Serial.println("║   Voltage Divider: 10k/20k (1.5x)    ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  pinMode(RELAY_SENSOR1, OUTPUT);
  pinMode(RELAY_SENSOR2, OUTPUT);
  pinMode(RELAY_SENSOR3, OUTPUT);
  pinMode(RELAY_WATER_PUMP, OUTPUT);
  pinMode(RELAY_SOLENOID, OUTPUT);
  
  // Sensor relays always ON
  digitalWrite(RELAY_SENSOR1, LOW);
  digitalWrite(RELAY_SENSOR2, LOW);
  digitalWrite(RELAY_SENSOR3, LOW);
  digitalWrite(RELAY_WATER_PUMP, HIGH);
  digitalWrite(RELAY_SOLENOID, HIGH);
  
  pinMode(WATER_DETECT_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
  
  preferences.begin("calib", false);
  
  do_zero_raw = preferences.getFloat("do_zero_raw", 0.0);
  do_100_raw = preferences.getFloat("do_100_raw", 0.0);
  do_calibrated = preferences.getBool("do_calibrated", false);
  
  ph_zero_raw = preferences.getFloat("ph_zero_raw", 0.0);
  ph_slope_raw = preferences.getFloat("ph_slope_raw", 0.0);
  ph_calibrated = preferences.getBool("ph_calibrated", false);
  
  tds_15ppt_raw = preferences.getFloat("tds_15ppt_raw", 0.0);
  tds_25ppt_raw = preferences.getFloat("tds_25ppt_raw", 0.0);
  tds_calibrated = preferences.getBool("tds_calibrated", false);
  
  turbidity_zero_raw = preferences.getFloat("turbidity_zero_raw", 0.0);
  turbidity_1000_raw = preferences.getFloat("turbidity_1000_raw", 0.0);
  turbidity_calibrated = preferences.getBool("turbidity_calibrated", false);
  
  cal_factor = preferences.getFloat("weight_factor", -7050.0);
  empty_dist = preferences.getFloat("empty_dist", 0.0);
  full_dist = preferences.getFloat("full_dist", 0.0);
  solenoid_time = preferences.getInt("solenoid_time", 10);
  
  if (empty_dist > 0 && full_dist > 0) { empty_cal = true; full_cal = true; }
  
  ds18b20.begin();
  scale.begin(LOADCELL_DOUT, LOADCELL_SCK);
  scale.set_scale(cal_factor);
  scale.tare(10);
  
  pinMode(RS485_CONTROL, OUTPUT);
  digitalWrite(RS485_CONTROL, LOW);
  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  ammoniumSensor.begin(AMMONIUM_ADDR, Serial2);
  ammoniumSensor.preTransmission([]() { digitalWrite(RS485_CONTROL, HIGH); });
  ammoniumSensor.postTransmission([]() { digitalWrite(RS485_CONTROL, LOW); });
  
  Serial.print("[WIFI] Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\n[WIFI] Connected");
  Serial.print("[WIFI] IP: ");
  Serial.println(WiFi.localIP());
  
  setupCalibrationRoutes();
  espServer.begin();
  Serial.print("[WEB] http://");
  Serial.println(WiFi.localIP());
  
  webSocket.begin(pc_server, pc_websocket_port, "/esp32");
  webSocket.onEvent(wsEvent);
  webSocket.setReconnectInterval(5000);
  
  checkConnections();
  Serial.println("\n✅ System Ready!");
  Serial.println("🔌 Sensor relays are ALWAYS ON for continuous monitoring");
  Serial.printf("📐 Voltage divider factor: %.1fx (10k/20k)\n", VOLTAGE_DIVIDER_FACTOR);
  Serial.println("\n📌 TDS Calibration Instructions:");
  Serial.println("   1. Click 'Start TDS Cal'");
  Serial.println("   2. Place sensor in 15ppt solution, wait 30 seconds");
  Serial.println("   3. Click 'Set 15ppt'");
  Serial.println("   4. Place sensor in 25ppt solution, wait 30 seconds");
  Serial.println("   5. Click 'Set 25ppt'");
  Serial.println("Waiting for commands...\n");
}

// ========== Loop ==========
void loop() {
  espServer.handleClient();
  webSocket.loop();
  
  static unsigned long last_state = 0, last_sensor = 0, last_send = 0, last_check = 0;
  unsigned long now = millis();
  
  if (now - last_state >= 50) { updateTestMachine(); last_state = now; }
  if (now - last_sensor >= 100) { updateSensors(); last_sensor = now; }
  if (now - last_send >= 2000) { sendSensorData(); last_send = now; }
  if (now - last_check >= 5000) { checkConnections(); last_check = now; }
  
  delay(1);
}