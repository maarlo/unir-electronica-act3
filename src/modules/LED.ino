long getLEDBrightness(float environmentLux) {
  const long brightness = environmentLux > LED_MAX_ENVIRONMENT_LUX ? LED_MAX_BRIGHTNESS : map(environmentLux, LED_MIN_ENVIRONMENT_LUX, LED_MAX_ENVIRONMENT_LUX, LED_MIN_BRIGHTNESS, LED_MAX_BRIGHTNESS);
  return LED_MIN_BRIGHTNESS + LED_MAX_BRIGHTNESS - brightness;
}