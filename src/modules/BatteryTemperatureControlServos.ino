/**
 * Function to set angle to the corresponding servo motor
 * in charge of open or close the electronic valve for
 * heating and cooling the battery
 *
 * @param float batteryTemperature: Current temperature of the battery
 * @return int: 0: No EV opened, 1: Cool EV opened, 2: Heating EV opened.
 */
int setBatteryTemperatureControlServosPosition(float batteryTemperature) {
  if (batteryTemperature > BATTERY_TMP + BATTERY_TMP_MIN_DIFF) {
    // Cool
    const float angle = map(batteryTemperature, BATTERY_TMP + BATTERY_TMP_MIN_DIFF, BATTERY_TMP + BATTERY_TMP_MAX_DIFF, 0, 180);
    servoCool.write(angle);
    servoHeat.write(0);

    return 1;
  } else if (batteryTemperature < BATTERY_TMP - BATTERY_TMP_MIN_DIFF) {
    // Heat
    const float angle = map(batteryTemperature, BATTERY_TMP - BATTERY_TMP_MIN_DIFF, BATTERY_TMP - BATTERY_TMP_MAX_DIFF, 0, 180);

    servoCool.write(0);
    servoHeat.write(angle);

    return 2;
  } else {
    servoCool.write(0);
    servoHeat.write(0);

    return 0;
  }
}
