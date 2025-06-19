float readEnvironmentLuminosity() {
  int analogValue = analogRead(LDR_PIN);
  float voltage = analogValue / 1024. * 5;
  float resistance = 2000 * voltage / (1 - voltage / 5);
  float lux = pow(LDR_RL10 * 1e3 * pow(10, LDR_GAMMA) / resistance, (1 / LDR_GAMMA));

  return lux;
}

float readEnvironmentTemperature() {
  float temperature = dht.readTemperature();

  return temperature;
}

float readEnvironmentHumidity() {
  float humidity = dht.readHumidity();

  return humidity;
}