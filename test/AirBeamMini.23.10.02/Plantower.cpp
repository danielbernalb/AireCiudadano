#include "BLEDevice.h"
#include "BLEServer.h"
#include "EEPROM.h"
#include "ESP32Time.h"
#include "esp_bt_main.h"
#include "HardwareSerial.h"
#include "RTC.h"
#include "Plantower.h"
#include "SDCard.h"
#include "WiFi.h"

#define vmeas 2

byte buf[31];

String SDstring;
String tempuuid = F("A");

extern HardwareSerial pt_Serial;

extern ESP32Time rtc;

extern char conf[50];

/*Temp variables for averaging*/
//uint32_t tempconcPM1_0_CF1 = 0;
//uint32_t tempconcPM2_5_CF1 = 0;
//uint32_t tempconcPM10_0_CF1 = 0;
extern uint32_t tempconcPM1_0_amb;
//extern uint32_t tempconcPM2_5_amb;
//extern uint32_t tempconcPM10_0_amb;
//uint32_t temprawGt0_3um = 0;
//uint32_t temprawGt0_5um = 0;
//uint32_t temprawGt1_0um = 0;
//uint32_t temprawGt2_5um = 0;
//uint32_t temprawGt5_0um = 0;
//uint32_t temprawGt10_0um = 0;

extern uint32_t plantowercount;

extern int mode;
/*Global variables used for streaming (Used in main code)*/
//extern uint16_t concPM1_0_CF1;
//extern uint16_t concPM2_5_CF1;
//extern uint16_t concPM10_0_CF1;
extern double concPM1_0_amb;
//extern double concPM2_5_amb;
//extern double concPM10_0_amb;

extern double PM1_0;
extern double PM2_5;
//extern uint16_t BAMPM2_5;
//extern uint16_t BAMPM10;
//extern uint16_t GRIMMPM1;
//extern uint16_t GRIMMPM2_5;
//extern uint16_t GRIMMPM10;

extern boolean USB;
extern boolean BLEflag;
extern boolean BLE_device_connected;

boolean HDC1080flag = false;

extern uint32_t BLE_connection_time;
extern String uuid;
extern String uuidauth;
extern String BLEmac;
extern String firmwareversion;

extern int years;
extern int months;
extern int days;
extern int hours;
extern int mins;
extern int secs;

extern int avgyears;
extern int avgmonths;
extern int avgdays;
extern int avghours;
extern int avgmins;
extern int avgsecs;

extern double avgBat;

extern int i;

extern String EEPROMString;
extern String BLE_SD_Index;
extern String WiFi_SD_Index;
extern String BatteryLevel;


/*Plantower reading function*/
byte plantower() {
  if (pt_Serial.find(0x42)) {
    pt_Serial.readBytes(buf, 31);
    if (buf[0] == 0x4d && ((buf[29] << 8) + buf[30] == (0x42 + buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7] + buf[8] + buf[9] + buf[10] + buf[11] + buf[12] + buf[13] + buf[14] + buf[15] + buf[16] + buf[17] + buf[18] + buf[19] + buf[20] + buf[21] + buf[22] + buf[23] + buf[24] + buf[25] + buf[26] + buf[27] + buf[28]))) {
      plantowercount++;
      tempconcPM1_0_amb += ((buf[9] << 8) + buf[10]);
      return 1;
    }
  }
  return 0;
}

void sensor_average() {  //Average the number of readings taken
  if (plantowercount == 0) {
    //Error from Plantower with zero count do nothing
  }
  if (plantowercount != 0) {
    concPM1_0_amb = (double)tempconcPM1_0_amb / (double)plantowercount;

    if (concPM1_0_amb < 22.14) {
      PM1_0 = (1.0566 * concPM1_0_amb);
    } else if (concPM1_0_amb > 22.13 && concPM1_0_amb < 37.95) {
      PM1_0 = (-30.9677 + (2.4554 * concPM1_0_amb));
    } else if (concPM1_0_amb > 37.94) {
      PM1_0 = (11.2026 + (1.3442 * concPM1_0_amb));
    }

    if (PM1_0 < 30.16) {
      PM2_5 = (1.0888 * PM1_0);
    } else if (PM1_0 > 30.15 && PM1_0 < 126.85) {
      PM2_5 = (-5.84 + (1.2826 * PM1_0));
    } else if (PM1_0 > 126.84) {
      PM2_5 = (-25.94 + (1.441 * PM1_0));
    }

    PM1_0 = round(PM1_0);
    PM2_5 = round(PM2_5);
  }
  if (mode == 0 || mode == 1) {
    Serial.println();
  }
  avgyears = years;
  avgmonths = months;
  avgdays = days;
  avghours = hours;
  avgmins = mins;
  avgsecs = secs;
  tempuuid = tempuuid == "A" ? uuid + String("\n") : "";
  SDstring = String(tempuuid + printDigits(avgmonths) + "/" + printDigits(avgdays) + "/" + printDigitsYear(avgyears) + "," + printDigits(avghours) + ":" + printDigits(avgmins) + ":" + printDigits(avgsecs) + "," + (int)PM1_0 + "," + (int)PM2_5 + "\n");
  /*Serial Monitor Output*/
  if (mode == 0 || mode == 1) {
    Serial.printf("AirBeamMini:%s%s %s ", BLEmac.c_str(), firmwareversion.c_str(), Vmeas(1).c_str());
  }
  Serial.printf("%02dM/%02dD/%sY %02dh:%02dm:%02ds", avgmonths, avgdays, printDigitsYear(avgyears).c_str(), avghours, avgmins, avgsecs);
  if (mode == 0) {
    Serial.print(F(" BLE:"));
    if (BLE_device_connected) {
      stream_BLE_data();
      Serial.printf("%-12s", String("Connected").c_str());
      BLE_connection_time = millis();
    }
    if (!BLE_device_connected) {
      if (BLEflag && millis() - BLE_connection_time > 180000) {
        BLEflag = false;
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
      }
      if (BLEflag) {
        Serial.printf("%-12s", String("Disconnected").c_str());
      }
      if (!BLEflag) {
        Serial.printf("%-12s", String("Off").c_str());
      }
    }
    Serial.println();

    BLE_SD_Index = F("");
    for (i = 0; i < EEPROM.read(11); i++) {
      BLE_SD_Index += (char)EEPROM.read(1100 + i);
    }
    EEPROMString = compFile(BLE_SD_Index, "/BLE.csv", "/WiFi.csv");
    EEPROM.write(11, EEPROMString.length());
    EEPROMString.toCharArray(conf, EEPROMString.length() + 1);
    for (i = 0; i < EEPROMString.length(); i++) {
      EEPROM.write(1100 + i, conf[i]);
    }
    EEPROM.commit();
    appendFile("/BLE.csv", SDstring.c_str()); //Index Count Removed
  }

  Serial.printf(" P:%-7d ", plantowercount);

  if (mode == 1) {
    Serial.print(F("WiFi IP:"));
    Serial.print(WiFi.localIP());
    Serial.print(F(" RSSI:"));
    Serial.print(WiFi.RSSI());
    Serial.print(F("dBm "));
    WiFi_SD_Index = F("");
    for (i = 0; i < EEPROM.read(12); i++) {
      WiFi_SD_Index += (char)EEPROM.read(1200 + i);
    }
    EEPROMString = compFile(WiFi_SD_Index, "/WiFi.csv", "/BLE.csv");
    EEPROM.write(12, EEPROMString.length());
    EEPROMString.toCharArray(conf, EEPROMString.length() + 1);
    for (i = 0; i < EEPROMString.length(); i++) {
      EEPROM.write(1200 + i, conf[i]);
    }
    EEPROM.commit();
    appendFile("/WiFi.csv", SDstring.c_str()); //Index Count Removed
  }
  if (mode != 1) {
    Serial.printf("PM1.0 PM2.5 % 4d % 4d", (int)PM1_0, (int)PM2_5);
  }
  Serial.println();
  tempconcPM1_0_amb = 0;
  plantowercount = 0;
}
