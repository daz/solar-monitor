#define HOSTNAME "esp-solar-monitor" // esp-solar-monitor.local

#include <Secrets.h>
#include <WiFiSetup.h>
#define DEBUG_PORT TelnetDebug
#include <Debuggy.h>

#include <Adafruit_ADS1015.h>
#include <BlynkSimpleEsp8266.h>
#include <Losant.h>

// Set these in Secrets.h
const char* BLYNK_AUTH                = SECRET_BLYNK_AUTH;
const char* LOSANT_DEVICE_ID          = SECRET_LOSANT_DEVICE_ID;
const char* LOSANT_ACCESS_KEY         = SECRET_LOSANT_ACCESS_KEY;
const char* LOSANT_ACCESS_SECRET      = SECRET_LOSANT_ACCESS_SECRET;

const float MAX_BATTERY_VOLTAGE       = 33;
const int BATTERY_VOLTAGE_PIN         = A0;
const int LCD_BUTTON_PIN              = D0;
const int CALL_FOR_POWER_LED          = D3;
const int CALL_FOR_POWER_PIN          = D5;

const int ADS_SCL                     = D1;
const int ADS_SDA                     = D2;
const uint8_t LCD_ADDR                = 0x3F;
const uint8_t ADS1115_ADDR            = 0x48;

const int INVERTER_TEMP_ADC_PIN       = 0;
const int SOLAR_1_ADC_PIN             = 1;
const int SOLAR_2_ADC_PIN             = 2;
const int SOLAR_3_ADC_PIN             = 3;
const int SOLAR_MAX_CURRENT           = 20;

// For now just find values for 0-30A at GAIN_TWOTHIRDS
const int SOLAR_CURRENT_ADC_LOWER     = 8605;
const int SOLAR_CURRENT_ADC_UPPER     = 15612;

// When an ADC input is bridged straight to 3.3v at GAIN_TWOTHIRDS
// More about gains:
// https://github.com/adafruit/Adafruit_ADS1X15/blob/master/examples/singleended/singleended.pde
const int ADC_MAX                     = 17795;

const int INVERTER_1_PIN              = D6;
const int INVERTER_2_PIN              = D7;

// Array of inverter pins, in order of how they ramp up.
// Update as inverters are added to the system
const std::vector<int> INVERTERS {INVERTER_1_PIN, INVERTER_2_PIN};
// Delay between ramping inverters up/down
const int INVERTER_RAMP_INTERVAL      = 1000;

// Trigger fan if temp >40ºC, goes off at <32ºC
const int INVERTER_TEMP_TRIGGER_UPPER = 40;
const int INVERTER_TEMP_TRIGGER_LOWER = 32;
const int FAN_PIN                     = D8;

// Blynk virtual pins
const int CALLING_FOR_POWER_VPIN      = V0;
const int CALLING_FOR_POWER_VLED      = V6;
const int SOLAR_1_CURRENT_VPIN        = V1;
const int SOLAR_2_CURRENT_VPIN        = V2;
const int SOLAR_3_CURRENT_VPIN        = V3;
const int INVERTER_TEMP_VPIN          = V4;
const int BATTERY_VOLTAGE_VPIN        = V5;

void updateServer();
void connect();
void runInverterQueue();
void getInverterTemp();
void getCallForPower();
void getSolarCurrent();
void getBatteryVoltage();
void doFan();
float KY013toCelcius(int val);
float solarCurrent(int adcVal);

BlynkTimer timer;
WiFiClientSecure wifiClient;
Adafruit_ADS1115 ads(ADS1115_ADDR);
LosantDevice losant(LOSANT_DEVICE_ID);

// State
struct {
  bool fan;
  bool callingForPower;
  float inverterTemp;
  float batteryVoltage;
  float solar1Current, solar2Current, solar3Current;
} State;

// Timers
int checkConnectionTimer = timer.setTimer(10000, connect, timer.RUN_FOREVER);
int updateServerTimer = timer.setTimer(5000, updateServer, timer.RUN_FOREVER);
int inverterTimer = timer.setTimer(INVERTER_RAMP_INTERVAL, runInverterQueue, timer.RUN_FOREVER);

std::vector<int> inverterQueue;
bool firstRun = true;

void setup() {
  Serial.begin(115200);

  connect();
  initOTA();
  initTelnetDebug();

  ads.setGain(GAIN_TWOTHIRDS);
  ads.begin();

  Serial.println("Connecting to Blynk...");
  Blynk.config(BLYNK_AUTH);
  Blynk.connect();

  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pinMode(LCD_BUTTON_PIN, INPUT);
  pinMode(CALL_FOR_POWER_PIN, INPUT);
  pinMode(CALL_FOR_POWER_LED, OUTPUT);
  pinMode(INVERTER_1_PIN, OUTPUT);
  pinMode(INVERTER_2_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
}

void loop() {
  getCallForPower();
  getSolarCurrent();
  getBatteryVoltage();
  getInverterTemp();
  doFan();

  timer.run();

  Blynk.run();
  losant.loop();

  TelnetDebug.handle();
  ArduinoOTA.handle();

  firstRun = false;
}

// TODO: Ensure reconnection to WiFi and services after dropout
void connect() {
  if (!losant.connected()) {
    Serial.println("Connecting to Losant...");
    losant.connectSecure(wifiClient, LOSANT_ACCESS_KEY, LOSANT_ACCESS_SECRET);
  }
}

void updateServer() {
  // Blynk
  Blynk.virtualWrite(SOLAR_1_CURRENT_VPIN, State.solar1Current);
  Blynk.virtualWrite(SOLAR_2_CURRENT_VPIN, State.solar2Current);
  Blynk.virtualWrite(SOLAR_3_CURRENT_VPIN, State.solar3Current);
  Blynk.virtualWrite(INVERTER_TEMP_VPIN, State.inverterTemp);
  Blynk.virtualWrite(BATTERY_VOLTAGE_VPIN, State.batteryVoltage);

  // Losant
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["cfp"] = State.callingForPower;
  root["bv"] = State.batteryVoltage;
  root["s1"] = State.solar1Current;
  root["s2"] = State.solar2Current;
  root["s3"] = State.solar3Current;
  root["it"] = State.inverterTemp;
  root["f"] = State.fan;
  losant.sendState(root);

  // Debug
  DEBUG_PRINT("callingForPower %s\n",   State.callingForPower ? "true" : "false");
  DEBUG_PRINT("batteryVoltage  %.4f\n", State.batteryVoltage);
  DEBUG_PRINT("solar1Current   %.4f\n", State.solar1Current);
  DEBUG_PRINT("solar2Current   %.4f\n", State.solar2Current);
  DEBUG_PRINT("solar3Current   %.4f\n", State.solar3Current);
  DEBUG_PRINT("inverterTemp    %.4f\n", State.inverterTemp);
  DEBUG_PRINT("fan             %s\n",   State.fan ? "true" : "false");
  DEBUG_PRINTLN();
}

void getCallForPower() {
  bool newCallingForPower = (digitalRead(CALL_FOR_POWER_PIN) == HIGH);

  if (newCallingForPower != State.callingForPower || firstRun) {
    State.callingForPower = newCallingForPower;

    if (inverterQueue.size() > 0) {
      timer.disable(inverterTimer);
    }
    inverterQueue = INVERTERS;
    if (!State.callingForPower) {
      std::reverse(std::begin(inverterQueue), std::end(inverterQueue));
    }
    timer.restartTimer(inverterTimer);
    runInverterQueue();

    digitalWrite(CALL_FOR_POWER_LED, State.callingForPower ? HIGH : LOW);
    Blynk.virtualWrite(CALLING_FOR_POWER_VPIN, State.callingForPower);
  }
}

// Pop inverters off the front of inverterQueue and pull them high or low based
// on State.callingForPower and schedule itself if inverterQueue isn't empty
void runInverterQueue() {
  if (inverterQueue.size() > 0) {
    int inverterPin = inverterQueue.front();
    inverterQueue.erase(inverterQueue.begin());
    digitalWrite(inverterPin, State.callingForPower ? HIGH : LOW);
    if (!timer.isEnabled(inverterTimer)) {
      timer.enable(inverterTimer);
    }
  }
  if (inverterQueue.size() == 0) {
    timer.disable(inverterTimer);
  }
}

void getSolarCurrent() {
  State.solar1Current = solarCurrent(ads.readADC_SingleEnded(SOLAR_1_ADC_PIN));
  State.solar2Current = solarCurrent(ads.readADC_SingleEnded(SOLAR_2_ADC_PIN));
  State.solar3Current = solarCurrent(ads.readADC_SingleEnded(SOLAR_3_ADC_PIN));
}

void getBatteryVoltage() {
  float val = map(analogRead(BATTERY_VOLTAGE_PIN), 0, 1023, 0, MAX_BATTERY_VOLTAGE);
  State.batteryVoltage = val;
}

void getInverterTemp() {
  // TODO: This is just reading 0-ADC_MAX, not calibrated at all
  float val = map(ads.readADC_SingleEnded(INVERTER_TEMP_ADC_PIN), 0, ADC_MAX, 0, 1023);
  State.inverterTemp = KY013toCelcius(val);
}

// Trigger fan if inverterTemp >= INVERTER_TEMP_TRIGGER_UPPER, keep it on
// until temp <= INVERTER_TEMP_TRIGGER_LOWER. Also triggers fan on boot if
// temp >= INVERTER_TEMP_TRIGGER_LOWER
void doFan() {
  bool newFan;
  if (State.inverterTemp <= INVERTER_TEMP_TRIGGER_LOWER) {
    newFan = false;
  } else if (State.inverterTemp >= INVERTER_TEMP_TRIGGER_UPPER || firstRun) {
    newFan = true;
  }
  if (newFan != State.fan) {
    State.fan = newFan;
    digitalWrite(FAN_PIN, State.fan ? HIGH : LOW);
  }
}

// From https://github.com/themactep/KY013
float KY013toCelcius(int val) {
  const float RESISTOR = 10000.0;
  const float CONST_A = 0.001129148;
  const float CONST_B = 0.000234125;
  const float CONST_C = 0.0000000876741;

  float r = RESISTOR / (1024.0 / val - 1);
  float lnR = log(r);
  float tempK = 1 / (CONST_A + CONST_B * lnR + CONST_C * pow(lnR, 3));
  float tempCelcius = tempK - 273.15;
  return tempCelcius;
}

float solarCurrent(int adcVal) {
  return map(adcVal, SOLAR_CURRENT_ADC_LOWER, SOLAR_CURRENT_ADC_UPPER, 0, SOLAR_MAX_CURRENT);
}
