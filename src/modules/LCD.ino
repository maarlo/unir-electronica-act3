void updateLCD(float environmentLux, float environmentTemperature, float environmentHumidity, int batteryHeatingServoStatus) {
  // Set Environment Luminosity on LCD
  lcd.setCursor(0, 1);
  if (environmentLux > 99999 && lcdLux != 99999) {
    lcd.println("99999");
    lcdLux = 99999;
  } else if (environmentLux != lcdLux) {
    lcd.println(environmentLux, 0);
    lcdLux = environmentLux;
  }

  // Set Environment Temperature on LCD
  if (environmentTemperature != lcdTmp) {
    if (environmentTemperature < 0) {
      lcd.setCursor(6, 1);
    } else {
      lcd.setCursor(7, 1);
    }
    lcd.println(environmentTemperature, 1);

    lcdTmp = environmentTemperature;
  }

  // Set Environment Humidity on LCD
  if (environmentHumidity != lcdHum) {
    lcd.setCursor(12, 1);
    if (environmentHumidity != 100) {
      lcd.println(environmentHumidity, 1);
    } else {
      lcd.println(environmentHumidity, 0);
    }

    lcdHum = environmentHumidity;
  }

  switch (batteryHeatingServoStatus) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(15, 0);
      lcd.print(" ");
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("*");
      lcd.setCursor(15, 0);
      lcd.print(" ");
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(15, 0);
      lcd.print("*");
      break;
  }
}

// Clear LCD row
void clearLCDRow(int row) {
  lcd.setCursor(0, row);
  lcd.print("                ");
}