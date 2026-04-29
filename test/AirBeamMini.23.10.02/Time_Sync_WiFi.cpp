#include "EEPROM.h"
#include "ESP32Time.h"
#include "RTC.h"
#include "Time_Sync.h"
#include "WiFi.h"

extern WiFiClient wifi_Serial;

extern ESP32Time rtc;

String datetime;

extern int checkyears;
extern int checkmonths;
extern int checkdays;
extern int checkhours;
extern int checkmins;
extern int checksecs;

boolean USDSTflag;
boolean EURDSTflag;

extern int i;

extern String ssid;
extern String pass;

extern String zone;

extern uint64_t century;
extern int years;
extern int months;
extern int days;
extern int hours;
extern int mins;
extern int secs;
extern String Aircastingserver;

String time_request = F("HEAD / HTTP/1.1\nHost: ");
String time_request_1 = F("\nConnection: close\n\n");

void TimeZone_from_UTC_with_DST(int TimeZone, int months, int days, int years, int hours, int mins, int secs) {
  hours = hours + TimeZone;
  if (hours < 0 && TimeZone < 0) {
    hours = hours + 24;
    days = days - 1;

    if (days < 1) {
      months = months - 1;
      if (months == 0) {
        months = 12;
        days = 31;
        years = years - 1;
      }
      if (months == 1)
        days = 31;
      if (months == 2) {
        if (!(years % 4) && ((years % 100) || !(years % 400))) {
          days = 29;
        } else
          days = 28;
      }
      if (months == 3)
        days = 31;
      if (months == 4)
        days = 30;
      if (months == 5)
        days = 31;
      if (months == 6)
        days = 30;
      if (months == 7)
        days = 31;
      if (months == 8)
        days = 31;
      if (months == 9)
        days = 30;
      if (months == 10)
        days = 31;
      if (months == 11)
        days = 30;
    }
  }

  if (hours > 23 && TimeZone > 0) {
    hours = hours - 24;
    if (((months == 1 && days == 31) || (months == 2 && days == 29 && (!(years % 4) && ((years % 100) || !(years % 400)))) || (months == 2 && days == 28 && ((years % 4) && (!(years % 100) || (years % 400)))) || (months == 3 && days == 31) || (months == 4 && days == 30) || (months == 5 && days == 31) || (months == 6 && days == 30) || (months == 7 && days == 31) || (months == 8 && days == 31) || (months == 9 && days == 30) || (months == 10 && days == 31) || (months == 11 && days == 30) || (months == 12 && days == 31))) {
      days = 1;
      months = months + 1;
      if (months > 12) {
        months = 1;
        years = years + 1;
      }
    } else
      days = days + 1;
  }

  if (!(years % 4) && ((years % 100) || !(years % 400))) {
    Serial.print(F("Leap Year Active, "));
  } else {
    Serial.print(F("Leap Year Inactive, "));
  }

  uint64_t n = years + years / 4 - years / 100 + years / 400;
  byte marday = (n + 2) % 7;
  byte octday = (n + 6) % 7;
  byte novday = marday;

  if (TimeZone == -5 || TimeZone == -6 || TimeZone == -7 || TimeZone == -8 || TimeZone == -9) {
    if ((months == 3 && days == (14 - marday) && hours >= 2) || (months == 11 && days == (7 - novday) && hours <= 1) || (months == 3 && days > (14 - marday)) || (months == 11 && days < (7 - novday)) || (months > 3 && months < 11)) {
      USDSTflag = true;
      Serial.println(F("American Daylight Savings Time Active"));
    } else {
      USDSTflag = false;
      Serial.println(F("American Daylight Savings Time Inactive"));
    }

    if (USDSTflag) {
      hours = hours + 1;
      if (hours == 24) {
        hours = 0;
        if (((months == 1 && days == 31) || (months == 2 && days == 29 && (!(years % 4) && ((years % 100) || !(years % 400)))) || (months == 2 && days == 28 && ((years % 4) && (!(years % 100) || (years % 400)))) || (months == 3 && days == 31) || (months == 4 && days == 30) || (months == 5 && days == 31) || (months == 6 && days == 30) || (months == 7 && days == 31) || (months == 8 && days == 31) || (months == 9 && days == 30) || (months == 10 && days == 31) || (months == 11 && days == 30) || (months == 12 && days == 31))) {
          days = 1;
          months = months + 1;
          if (months > 12) {
            months = 1;
            years = years + 1;
          }
        } else {
          days = days + 1;
        }
      }
    }
  } else if (TimeZone == 0 || TimeZone == 1 || TimeZone == 2) {
    //Below calculates last Sunday of March and October of every year.
    /*
      if ((35 - marday <= 31) && (35 - octday <= 31) && ((months == 3 && days == (35 - marday) && hours > 0) || (months == 10 && days == (35 - octday) && hours == 0) || (months == 3 && days > (35 - marday)) || (months == 10 && days < (35 - octday)) || (months > 3 && months < 10))) {
      EURDSTflag = true;
      Serial.println(F("European Daylight Savings Time Active 1"));
      }
    */
    if ((35 - marday <= 31) && (28 - octday >= 25) && ((months == 3 && days == (35 - marday) && hours >= 1) || (months == 10 && days == (28 - octday) && hours <= 1) || (months == 3 && days > (35 - marday)) || (months == 10 && days < (28 - octday)) || (months > 3 && months < 10))) {
      EURDSTflag = true;
      Serial.println(F("European Daylight Savings Time Active"));
    } else if ((28 - marday >= 25) && (35 - octday <= 31) && ((months == 3 && days == (28 - marday) && hours >= 1) || (months == 10 && days == (35 - octday) && hours <= 1) || (months == 3 && days > (28 - marday)) || (months == 10 && days < (35 - octday)) || (months > 3 && months < 10))) {
      EURDSTflag = true;
      Serial.println(F("European Daylight Savings Time Active"));
    } else if ((28 - marday >= 25) && (28 - octday >= 25) && ((months == 3 && days == (28 - marday) && hours >= 1) || (months == 10 && days == (28 - octday) && hours <= 1) || (months == 3 && days > (28 - marday)) || (months == 10 && days < (28 - octday)) || (months > 3 && months < 10))) {
      EURDSTflag = true;
      Serial.println(F("European Daylight Savings Time Active"));
    } else {
      EURDSTflag = false;
      Serial.println(F("European Daylight Savings Time Inactive"));
    }
    if (EURDSTflag) {
      hours = hours + 1;
      if (hours == 24) {
        hours = 0;
        if (((months == 1 && days == 31) || (months == 2 && days == 29 && (!(years % 4) && ((years % 100) || !(years % 400)))) || (months == 2 && days == 28 && ((years % 4) && (!(years % 100) || (years % 400)))) || (months == 3 && days == 31) || (months == 4 && days == 30) || (months == 5 && days == 31) || (months == 6 && days == 30) || (months == 7 && days == 31) || (months == 8 && days == 31) || (months == 9 && days == 30) || (months == 10 && days == 31) || (months == 11 && days == 30) || (months == 12 && days == 31))) {
          days = 1;
          months = months + 1;
          if (months > 12) {
            months = 1;
            years = years + 1;
          }
        } else {
          days = days + 1;
        }
      }
    }
  } else {
    Serial.println(F("No Daylight Savings Time Detected"));
  }
  years = years % 100;
  RTCsettime(hours, mins, secs, days, months, years);
  checkyears = years;
  checkmonths = months;
  checkdays = days;
  checkhours = hours;
  checkmins = mins;
  checksecs = secs;
}

byte WiFitimesync() {
  if (WiFi.status() == WL_CONNECTED) {
    String TimeHEADrequest;
    wifi_Serial.stop();
    Serial.println();
    Serial.print(F("TCP Connection:"));
    for (i = 0; !wifi_Serial.connect(Aircastingserver.c_str(), 80) && i < 10; i++) {  // CHECK PM
      sensors();
    }
    if (i == 10) {
      Serial.println(F("Error"));
      return 1;
    } else {
      Serial.println(F("Good"));
      TimeHEADrequest += time_request;
      TimeHEADrequest += Aircastingserver;
      TimeHEADrequest += time_request_1;
      wifi_Serial.print(TimeHEADrequest);
      wifi_Serial.print("\r\n");
      Serial.print(F("Searching for Date&Time:"));
      for (i = 0; !wifi_Serial.find("Date: ") && i < 30; i++) {
        sensors();
        if (!(i % 10)) {
          Serial.println();
        }
        Serial.print(F("."));
      }
      if (i == 30) {
        Serial.println(F("Error"));
        return 1;
      } else {
        Serial.println(F("Good"));
      }
    }

    datetime = F("");
    for (i = 0; i < 25; i++) {
      if (wifi_Serial.available()) {
        char c = wifi_Serial.read();
        Serial.write(c);
        datetime += c;
      }
    }
    Serial.print(" UTC ");
    years = datetime.substring(12, 16).toInt();
    century = years / 100;
    if (datetime.substring(8, 11) == "Jan") {
      months = 1;
    }
    if (datetime.substring(8, 11) == "Feb") {
      months = 2;
    }
    if (datetime.substring(8, 11) == "Mar") {
      months = 3;
    }
    if (datetime.substring(8, 11) == "Apr") {
      months = 4;
    }
    if (datetime.substring(8, 11) == "May") {
      months = 5;
    }
    if (datetime.substring(8, 11) == "Jun") {
      months = 6;
    }
    if (datetime.substring(8, 11) == "Jul") {
      months = 7;
    }
    if (datetime.substring(8, 11) == "Aug") {
      months = 8;
    }
    if (datetime.substring(8, 11) == "Sep") {
      months = 9;
    }
    if (datetime.substring(8, 11) == "Oct") {
      months = 10;
    }
    if (datetime.substring(8, 11) == "Nov") {
      months = 11;
    }
    if (datetime.substring(8, 11) == "Dec") {
      months = 12;
    }
    days = datetime.substring(5, 7).toInt();
    hours = datetime.substring(17, 19).toInt();
    mins = datetime.substring(20, 22).toInt();
    secs = datetime.substring(23, 25).toInt();
    if (months > 0 && days > 0 && years > -1 && hours > -1 && mins > -1 && secs > -1 && months < 13 && days < 32 && hours < 24 && mins < 60 && secs < 60) {
      TimeZone_from_UTC_with_DST(zone.toInt(), months, days, years, hours, mins, secs);
      wifi_Serial.stop();
      Serial.println(F("TCP Closed"));
      Serial.print(F("Set Date&Time"));
      RTCgettime();
      if ((abs(checksecs - secs) < 2 || abs(checksecs - secs) == 59) && checkmins == mins && checkhours == hours && checkdays == days && checkmonths == months && checkyears == years) {  //Checks if time is set wrong from above
        Serial.printf(":%02dM/%02dD/%sY %02dh:%02dm:%02ds\n", months, days, printDigitsYear(years).c_str(), hours, mins, secs);
        return 0;
      } else {
        Serial.printf(" Mismatch:%02dM/%02dD/%sY %02dh:%02dm:%02ds\n", months, days, printDigitsYear(years).c_str(), hours, mins, secs);
        return 1;
      }
    } else {
      return 1;
    }
  } else {
    Serial.println(F("WiFi network currently disconnected, skipping WiFi time sync"));
    return 1;
  }
}
