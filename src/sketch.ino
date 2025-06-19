#include <ArduinoJson.h>
#include <DHT.h>
#include <Adafruit_LiquidCrystal.h>
#include <Servo.h>
#include <IRremote.hpp>

#include <modules/Variables.ino>
#include <modules/SensorsEnvironment.ino>
#include <modules/SensorsBattery.ino>
#include <modules/BatteryTemperatureControlServos.ino>
#include <modules/LED.ino>
#include <modules/LCD.ino>
#include <modules/IR.ino>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Initialize hardware
  lcd.begin(16, 2);
  dht.begin();
  pinMode(LDR_PIN, INPUT);
  pinMode(NTC_PIN, INPUT);
  servoCool.attach(5);
  servoHeat.attach(3);
  IrReceiver.begin(IR_RECEIVER_PIN);

  // Initialize Battery Heating Resistance, LED and LCD
  analogWrite(LED_PIN, 0);

  servoCool.write(0);
  servoHeat.write(0);

  clearLCDRow(0);
  clearLCDRow(1);
  lcd.setCursor(0, 0);
  lcd.print(" LUX   TMP  HUM ");
  lcd.print("0      0.0  0.0 ");
}

void loop() {
  // put your main code here, to run repeatedly:
  float currentEnvironmentLux = readEnvironmentLuminosity();
  float currentEnvironmentTemperature = readEnvironmentTemperature();
  float currentEnvironmentHumidity = readEnvironmentHumidity();

  float currentBatteryTemperature = readBatteryTemperature();

  int irData = receiveIrData();
  processIrData(irData);

  // Send lecture through serial
  sendData(currentEnvironmentLux, currentEnvironmentTemperature, currentEnvironmentHumidity, currentBatteryTemperature);

  // Store current data in history
  storeCurrentDataInHistory(currentEnvironmentLux, currentEnvironmentTemperature, currentEnvironmentHumidity, currentBatteryTemperature);

  // Obtain current data
  float environmentLux = calculeHistoricalMeanValue(historyLux, historyLength);
  float environmentTemperature = calculeHistoricalMeanValue(historyTmp, historyLength);
  float environmentHumidity = calculeHistoricalMeanValue(historyHum, historyLength);
  float batteryTemperature = calculeHistoricalMeanValue(historyBatteryTmp, historyLength);

  // Enable Battery Heating Resistance if needed
  int batteryHeatingServoStatus = setBatteryTemperatureControlServosPosition(batteryTemperature);

  // Update LED Brightness
  if (evenCycle) {
    long ledBrightness = getLEDBrightness(environmentLux);
    analogWrite(LED_PIN, ledBrightness);
  } else {
    analogWrite(LED_PIN, 0);
  }

  // Update LCD data
  if (evenCycle) {
    updateLCD(environmentLux, environmentTemperature, environmentHumidity, batteryHeatingServoStatus);
  }

  // Toggle evenCycle variable
  evenCycle = !evenCycle;

  delay(500);
}

//
void sendData(float environmentLux, float environmentTemperature, float environmentHumidity, float batteryTemperature) {
  String s = "";

  // Create JSON document
  StaticJsonDocument<200> doc;
  doc["environment"]["lux"] = environmentLux;
  doc["environment"]["temperature"] = environmentTemperature;
  doc["environment"]["humidity"] = environmentHumidity;
  doc["battery"]["temperature"] = batteryTemperature;

  // Serialize JSON document to string
  serializeJson(doc, s);

  Serial.println(s);
}

void storeCurrentDataInHistory(float environmentLux, float environmentTemperature, float environmentHumidity, float batteryTemperature) {
  historyLux[historyStep] = environmentLux;
  historyTmp[historyStep] = environmentTemperature;
  historyHum[historyStep] = environmentHumidity;
  historyBatteryTmp[historyStep] = batteryTemperature;

  if (historyStep == historyLength - 1) {
    historyStep = 0;
  } else {
    historyStep++;
  }
}

float calculeHistoricalMeanValue(float values[], int arrayLength) {
  float valuesSum = 0;

  for (int m = 0; m < arrayLength; ++m) {
    valuesSum += values[m];
  }

  return valuesSum / arrayLength;
}