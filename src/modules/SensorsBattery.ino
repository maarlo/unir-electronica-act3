float readBatteryTemperature() {
  int analogValue = analogRead(NTC_PIN);
  float celsius = 1 / (log(1 / (1023. / analogValue - 1)) / NTC_BETA + 1.0 / 298.15) - 273.15;

  return celsius;
}