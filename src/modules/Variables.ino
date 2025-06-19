// Define LCD variables
Adafruit_LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// Define DHT variables
DHT dht(4, DHT22);

// Define servo variables
Servo servoCool;
Servo servoHeat;

// Define LDR variables
const int LDR_PIN = A5;
const float LDR_GAMMA = 0.7;
const float LDR_RL10 = 50;

// Define NTC variables
const int NTC_PIN = A3;
const float NTC_BETA = 3950;

// Define LED variables
const int LED_PIN = 6;
const long LED_MIN_ENVIRONMENT_LUX = 10;
const long LED_MAX_ENVIRONMENT_LUX = 10000;
const long LED_MIN_BRIGHTNESS = 10;
const long LED_MAX_BRIGHTNESS = 255;

// Define BATTERY variables (hysteresis)
int batteryTmpOptimal = 20;
const int BATTERY_TMP_MIN_DIFF = 3;
const int BATTERY_TMP_MAX_DIFF = 9;

// IR
const int IR_RECEIVER_PIN = 2;

// Aux variables
bool evenCycle = false;

float lcdLux = 0;
float lcdTmp = 0;
float lcdHum = 0;

int historyStep = 0;
const int historyLength = 25;
float historyLux[historyLength] = { };
float historyTmp[historyLength] = { };
float historyHum[historyLength] = { };
float historyBatteryTmp[historyLength] = { };