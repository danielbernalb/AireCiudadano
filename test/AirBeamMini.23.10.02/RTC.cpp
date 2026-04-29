#include "Arduino.h"
#include "Color.h"
#include "EEPROM.h"
#include "ESP32Time.h"
#include "RTC.h"

extern ESP32Time rtc;

extern char conf[50];

extern byte mode_set_eeprom;

extern int checkyears;
extern int checkmonths;
extern int checkdays;
extern int checkhours;
extern int checkmins;
extern int checksecs;

extern int years;
extern int months;
extern int days;
extern int hours;
extern int mins;
extern int secs;

extern int eepromyears;

extern int trigger;

extern RTC_DATA_ATTR unsigned long epoch;

extern int i;

extern byte bright_percent;

void RTCsettime(int hours, int mins, int secs, int days, int months, int years) {
  rtc.setTime(secs, mins, hours, days, months, printDigitsYear(years).toInt());
  EEPROM.write(8, String(years).length());
  String(years).toCharArray(conf, String(years).length() + 1);
  for (i = 0; i < String(years).length(); i++) {
    EEPROM.write(800 + i, conf[i]);
  }
  EEPROM.commit();
}

byte RTCgettime() {
  for (byte v = 0; v < 10; v++) {
    String temp;
    for (i = 0; i < EEPROM.read(8); i++) {
      temp += (char)EEPROM.read(800 + i);
    }
    eepromyears = temp.toInt();
    secs = rtc.getSecond();
    mins = rtc.getMinute();
    hours = rtc.getHour(true);
    days = rtc.getDay();
    months = rtc.getMonth() + 1;
    years = rtc.getYear() % 100;
    //Serial.printf("Get Date:%02dM/%02dD/%sY %02dh:%02dm:%02ds\n", months, days, printDigitsYear(years).c_str(), hours, mins, secs);
    if (((months > 0 && days > 0 && years > -1 && hours > -1 && mins > -1 && secs > -1 && years < 100 && months < 13 && days < 32 && hours < 24 && mins < 60 && secs < 60) && (years == 0 ? (eepromyears - years) == 99 || (eepromyears == years) : (years - eepromyears) < 2 && (years - eepromyears) >= 0)) || mode_set_eeprom == 0 || mode_set_eeprom == 0xFF) {
      //Serial.println("RTC Time Get Good");
      if (!trigger) {
        green(bright_percent);
      }
      return 1;
    } else {
      if (trigger) {
        Serial.print("RTC Get Error Two Date&Time:");
        Serial.printf("%dEY,%02dM/%02dD/%sY %02dh:%02dm:%02ds\n", eepromyears, months, days, printDigitsYear(years).c_str(), hours, mins, secs);
        red(bright_percent);
      }
    }
  }
  return 0;
}
