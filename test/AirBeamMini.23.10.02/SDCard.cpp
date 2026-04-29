#include "Arduino.h"
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLE2902.h"
#include "Color.h"
#include "EEPROM.h"
#include "HardwareSerial.h"
#include "FFat.h"
#include "RTC.h"
#include "SDCard.h"

#define vmeas 2
#define V_USB 3

char FFATbuffer[129];
char FFATString[517];

extern BLEAdvertising* pAdvertising;

extern BLEService* pService;

extern BLEServer* pServer;

extern BLECharacteristic* pCharacteristicSDSync;

extern HardwareSerial pt_Serial;

extern boolean USB;
extern boolean USBflag;

extern boolean BLE_device_connected;

extern char conf[50];

extern String BLEmac;
extern String firmwareversion;
extern String uuid;
extern String uuidauth;

extern int years;
extern int months;
extern int days;
extern int hours;
extern int mins;
extern int secs;

extern int i;

extern uint16_t PM1_0;
extern uint16_t PM2_5;

extern byte bright_percent;

void readFile(const char* path) {
  Serial.printf("File:%s", path);
  File file = FFat.open(path);
  if (file) {
    Serial.printf(" Size:\t%8uB\n", file.size());
    cyan(bright_percent);
    while (file.available() && BLE_device_connected) {
      USB = digitalRead(V_USB);
      memset(FFATbuffer, 0, sizeof(FFATbuffer));
      file.readBytesUntil('\n', FFATbuffer, sizeof(FFATbuffer));
      strcat(FFATbuffer, "\r\n");
      strcpy(FFATString, FFATbuffer);
      memset(FFATbuffer, 0, sizeof(FFATbuffer));
      file.readBytesUntil('\n', FFATbuffer, sizeof(FFATbuffer));
      strcat(FFATbuffer, "\r\n");
      strcat(FFATString, FFATbuffer);
      memset(FFATbuffer, 0, sizeof(FFATbuffer));
      file.readBytesUntil('\n', FFATbuffer, sizeof(FFATbuffer));
      strcat(FFATbuffer, "\r\n");
      strcat(FFATString, FFATbuffer);
      memset(FFATbuffer, 0, sizeof(FFATbuffer));
      file.readBytesUntil('\n', FFATbuffer, sizeof(FFATbuffer));
      strcat(FFATbuffer, "\r\n");
      strcat(FFATString, FFATbuffer);
      pCharacteristicSDSync->setValue(FFATString);
      Serial.print(FFATString);
      if (USBflag && USB) {
        USBflag = false;
        setCpuFrequencyMhz(160);
        Serial.begin(115200);
        Serial.setTimeout(100);
      }
      if (!USBflag && !USB) {
        USBflag = true;
        setCpuFrequencyMhz(80);
        Serial.end();
      }
      if (Serial.read() == '=') {
        RTCgettime();
        Serial.printf("\nAirBeamMini:%s%s\n%02dM/%02dD/%sY% 02dh:%02dm:%02ds %3.2fV(Not Averaged)\n", BLEmac.c_str(), firmwareversion.c_str(), months, days, printDigitsYear(years).c_str(), hours, mins, secs, (analogReadMilliVolts(vmeas) * 1499.0) / 1000000.0);
        file.close();
        Soft_Reset_ESP();
      }
      pCharacteristicSDSync->notify();
    }
  } else {
    red(bright_percent);
    Serial.println(F("Failed To Open File For Reading"));
    file.close();
    Soft_Reset_ESP();
  }
  file.close();
}

void refreshFile(const char* path) {
  Serial.printf("File:%s", path);
  File file = FFat.open(path, FILE_APPEND);
  if (file) {
    Serial.printf(" Size:\t%8uB\n", file.size());
  } else {
    red(bright_percent);
    Serial.println(F(" Failed to Open File for Refreshing"));
    file.close();
    Soft_Reset_ESP();
  }
  file.close();
}

String compFile(String SD_index, const char* path, const char* path1) {
  SD_index = String(SD_index.toInt() + 1);
  if (FFat.usedBytes() / FFat.totalBytes()) {
    if (FFat.exists(path1) && FFat.remove(path1)) {
      if (strcmp(path, "/BLE.csv") == 0 && strcmp(path1, "/WiFi.csv") == 0) {
        EEPROM.write(12, 1);
        EEPROM.write(1200, 0);
      }
      if (strcmp(path, "/WiFi.csv") == 0 && strcmp(path1, "/BLE.csv") == 0) {
        EEPROM.write(11, 1);
        EEPROM.write(1100, 0);
      }
      EEPROM.commit();
    }
    else if (FFat.exists(path) && FFat.remove(path)) {
      SD_index = '1';
    }
    else {
      red(bright_percent);
      Soft_Reset_ESP();
    }
  }
  return SD_index;
}

void appendFile(const char* path, const char* message) {
  Serial.printf("File:%s", path);
  File file = FFat.open(path, FILE_APPEND);
  if (file.print(message)) {
    Serial.print(F(" File Appended"));
  } else {
    red(bright_percent);
    Serial.print(F(" Append Failed"));
    file.close();
    Soft_Reset_ESP();
  }
  file.close();
}

void deleteFile(const char* path) {
  Serial.printf("File:%s", path);
  if (FFat.remove(path)) {
    Serial.println(F(" Deleted"));
  } else {
    red(bright_percent);
    Serial.println(F(" Delete Failed"));
    Soft_Reset_ESP();
  }
}
