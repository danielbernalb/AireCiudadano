#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include "Build_String.h"
#include "Color.h"
#include "driver/adc.h"
#include "EEPROM.h"
#include "ESP32Time.h"
#include "FFat.h"
#include "esp_bt_main.h"
#include "esp_wifi.h"
#include "HardwareSerial.h"
#include "Read_EEPROM.h"
#include "RTC.h"
#include "Plantower.h"
#include "SDCard.h"
#include "Time_Sync.h"
#include "WiFi.h"

String firmwareversion = F("[v23.10.2]");

byte bright_percent = 100;  //Percentage of LED brightness from 0 to 100 percent only.

#define tempselect 2
//change tempselect above number to 1 for Celsius
//change tempselect above number to 2 for Fahrenheit
//change tempselect above number to 3 for Kelvin
//change tempselect above number to 4 for Celsius and Fahrenheit
//change tempselect above number to 5 for Fahrenheit and Kelvin
//change tempselect above number to 6 for Celsius and Kelvin
//change tempselect above number to 7 for Celsius, Fahrenheit and Kelvin

BLEAdvertising* pAdvertising = NULL;
BLEService* pService = NULL;
BLEServer* pServer = NULL;
BLEScan* pBLEScan = NULL;
BLECharacteristic* pCharacteristicWriteRead = NULL;
BLECharacteristic* pCharacteristicSDSync = NULL;
BLECharacteristic* pCharacteristicTempC = NULL;
BLECharacteristic* pCharacteristicTempF = NULL;
BLECharacteristic* pCharacteristicTempK = NULL;
BLECharacteristic* pCharacteristicHumi = NULL;
BLECharacteristic* pCharacteristicPM1_0 = NULL;
BLECharacteristic* pCharacteristicPM2_5 = NULL;
BLECharacteristic* pCharacteristicPM10_0 = NULL;
BLECharacteristic* pCharacteristicBatteryLevel = NULL;
BLECharacteristic* pCharacteristic = NULL;

WiFiClient wifi_Serial;

HardwareSerial pt_Serial(1);

ESP32Time rtc;

//#define ARRAY_SIZE 300    //size of buffer to hold first hex value
#define EEPROM_SIZE 2048  //set EEPROM size

char conf[50];  //temporary buffer for saved configuration

/*Variables used for sensors*/
uint32_t plantowercount;

#define vmeas 3
#define V_USB 4

#define red_led 2
#define green_led 1
#define blue_led 0

#define CHRG_USB 10

String Aircastingserver = F("aircasting.org");  //Host name

//Strings used for parameters set via BT//
String ssid;
String pass;
String uuid;
String uuidauth;
String longitude = F("200");
String latitude = F("200");
String zone;

String ESPble_resets;  //Used to keep count of BLE restarting AB3

String BLEmac;     //BLE MAC address
String post_data;  //Used to build HTTP strings
String BLEstr;     //Used to read from BLE

byte MobileLED = 8;  //Use for LED sychronization with RTC
int trigger = 0;
int mode = -1;  //Used to select streaming method
int stream_light = 1;

int i = 0;
char j = 0;

char u = 0;

uint16_t y = 0;
uint16_t z = 0;

/*SD Card Index Counter Variables*/
String BLE_SD_Index;
String WiFi_SD_Index;

String EEPROMString;  //EEPROM temporary Strings

boolean USB;
boolean USBflag = false;
boolean BLE_device_connected = false;
boolean BLE_connect_flag = true;
boolean config_window = false;
boolean BLEflag = true;
boolean timesyncflag = true;
boolean encapstart = false;
boolean encapend = false;
boolean LowBatFlag = false;

byte mode_set_eeprom;

/*Time variables*/
uint64_t century;
int years;
int months;
int days;
int hours;
int mins;
int secs;

int eepromyears;

int avgyears;
int avgmonths;
int avgdays;
int avghours;
int avgmins;
int avgsecs;

int checkyears;
int checkmonths;
int checkdays;
int checkhours;
int checkmins;
int checksecs;

int verifytime = -1;

/*Battery variables*/
const byte numAnalogReads = 5;
uint16_t Analog[numAnalogReads];
byte readAnalogIndex = 0;
uint16_t totAnalog = 0;
double avgBat = 0;
String BatteryLevel;

/*BT updates*/
//char firstByte[ARRAY_SIZE];
//char packet_buffer[ARRAY_SIZE];

uint16_t postlen = 0;  //Stores post data length

/*Timers*/
uint32_t starttime = 0;                //Pauser timer
uint32_t starttime0 = 0;               //Used for Sensors Streaming timer
uint32_t starttime1 = 0;               //Used for BLE and WiFi Streaming timer
uint32_t starttime2 = 0;               //Used for LED timer
uint32_t led_stream = 0;               //Used for White Comm LED reconnect timer
uint32_t configtime_expired = 180000;  //Change this for time frame period for configuration window
uint32_t BLE_timer = 1000;             //Reading interval for Bluetooth
uint32_t wifi_timer = 60000;           //Reading interval for WiFi
uint32_t sensor_timer = 100;           //Frequency of query for Plantower/HDC1080
uint32_t BLE_connection_time = 0;
uint32_t BLE_disconnection_time = 0;
uint32_t SDbegintime = 0;
uint32_t SDelapsedtime = 0;
uint32_t begintime = 0;    //Begin time for the start of the AB3 firmware
uint32_t elapsedtime = 0;  //Elapsed time for the AB3 configuration window

/*Plantower global variables*/
double concPM1_0_amb;

uint32_t tempconcPM1_0_amb;

double PM1_0;
double PM2_5;

/*Analyze buffers*/
void analyze_buf(String& str) {
  while (!str.indexOf(byte(0xFE)) && str.indexOf(byte(0xFF)) > 0) {
    byte a;
    byte b;
    Serial.print(F("Encapped>"));
    if (str.indexOf(byte(0x04)) > 0) {
      a = str.indexOf(byte(0xFE));
      b = str.indexOf(byte(0xFF));
      str.remove(b);
      /*Hex code with UUID*/
      config_window = true;
      uuid = str.substring(str.indexOf(byte(0x04)) + 1);
      Serial.print(F("UUID:"));
      Serial.println(uuid);
      EEPROM.write(1, uuid.length());
      uuid.toCharArray(conf, uuid.length() + 1);
      for (i = 0; i < uuid.length(); i++) {
        EEPROM.write(100 + i, conf[i]);
      }
      str.remove(a, ((b - 1) - a));
      EEPROM.commit();
    }

    if (str.indexOf(byte(0x05)) > 0) {
      a = str.indexOf(byte(0xFE));
      b = str.indexOf(byte(0xFF));
      str.remove(b);
      /*Hex code with uuidAuth*/
      Serial.print(F("UUIDAuth:"));
      //str = (char*)packet_buffer;
      uuidauth = str.substring(str.indexOf(byte(0x05)) + 1);
      Serial.println(uuidauth);
      EEPROM.write(2, uuidauth.length());
      uuidauth.toCharArray(conf, uuidauth.length() + 1);
      for (i = 0; i < uuidauth.length(); i++) {
        EEPROM.write(200 + i, conf[i]);
      }
      str.remove(a, ((b - 1) - a));
      EEPROM.commit();
    }

    if (str.indexOf(byte(0x06)) > 0) {
      a = str.indexOf(byte(0xFE));
      b = str.indexOf(byte(0xFF));
      str.remove(b);
      /*Hex code with latitude and longitude*/
      //str = (char*)packet_buffer;
      longitude = str.substring(str.indexOf(byte(0x06)) + 1, str.indexOf(','));
      latitude = str.substring(str.indexOf(',') + 1);
      latitude = latitude.substring(0, latitude.indexOf('.')) + latitude.substring(latitude.indexOf('.'), latitude.indexOf('.') + 7);        //Shortens the latitude for iPhone to 7 decimal places
      longitude = longitude.substring(0, longitude.indexOf('.')) + longitude.substring(longitude.indexOf('.'), longitude.indexOf('.') + 7);  //Shortens the longitude for iPhone to 7 decimal places
      Serial.print(F("Lat&Long:"));
      Serial.print(latitude);
      Serial.print(F(","));
      Serial.println(longitude);
      EEPROM.write(3, latitude.length());
      latitude.toCharArray(conf, latitude.length() + 1);
      for (i = 0; i < latitude.length(); i++) {
        EEPROM.write(300 + i, conf[i]);
      }
      EEPROM.write(4, longitude.length());
      longitude.toCharArray(conf, longitude.length() + 1);
      for (i = 0; i < longitude.length(); i++) {
        EEPROM.write(400 + i, conf[i]);
      }
      str.remove(str.indexOf(byte(0xFE)), (str.indexOf(byte(0xFF)) - str.indexOf(byte(0xFE))));
      EEPROM.commit();
    }

    if (str.indexOf(byte(0x08)) > 0) {
      a = str.indexOf(byte(0xFE));
      b = str.indexOf(byte(0xFF));
      str.remove(b);
      /*Hex code with date and time*/
      //str = (char*)packet_buffer;  //22/10/22-11:54:05 Day/Month/Year-Hour:Minute:Second
      days = str.substring(str.indexOf(byte(0x08)) + 1, str.indexOf('/')).toInt();
      months = str.substring(str.indexOf('/') + 1, str.lastIndexOf('/')).toInt();
      years = str.substring(str.lastIndexOf('/') + 1, str.indexOf('-')).toInt();
      hours = str.substring(str.indexOf('-') + 1, str.indexOf(':')).toInt();
      mins = str.substring(str.indexOf(':') + 1, str.lastIndexOf(':')).toInt();
      secs = str.substring(str.lastIndexOf(':') + 1, str.lastIndexOf(':') + 3).toInt();
      if (months > 0 && days > 0 && years > -1 && hours > -1 && mins > -1 && secs > -1 && months < 13 && days < 32 && hours < 24 && mins < 60 && secs < 60) {
        RTCsettime(hours, mins, secs, days, months, years);
        Serial.printf("Date&Time:%02dM/%02dD/%sY %02dh:%02dm:%02ds\n", months, days, printDigitsYear(years).c_str(), hours, mins, secs);
      }
      str.remove(a, ((b - 1) - a));
    }

    if (str.indexOf(byte(0x01)) > 0) {
      a = str.indexOf(byte(0xFE));
      b = str.indexOf(byte(0xFF));
      str.remove(b);
      str.remove(str.indexOf(byte(0xFE)), (str.indexOf(byte(0xFF)) - str.indexOf(byte(0xFE))));
      /*BT is selected as streaming method*/
      EEPROM.write(0, 1);
      EEPROM.commit();
      load_saved_config();
    }

    if (str.indexOf(byte(0x02)) > 0) {
      a = str.indexOf(byte(0xFE));
      b = str.indexOf(byte(0xFF));
      str.remove(b);
      /*WiFi is selected as streaming method*/
      /*Get parameters from hex code with SSID, PASS, and zone*/
      ssid = str.substring(str.indexOf(byte(0x02)) + 1, str.indexOf(','));
      pass = str.substring(str.indexOf(',') + 1, str.indexOf(',', str.indexOf(',') + 1));
      zone = str.substring(str.indexOf(',', str.indexOf(',') + 1) + 1);

      //Write variables in EEPROM//
      EEPROM.write(5, ssid.length());
      EEPROM.write(6, pass.length());
      EEPROM.write(7, zone.length());

      String(ssid).toCharArray(conf, ssid.length() + 1);
      for (i = 0; i < ssid.length(); i++) {
        EEPROM.write(500 + i, conf[i]);
      }

      String(pass).toCharArray(conf, pass.length() + 1);
      for (i = 0; i < pass.length(); i++) {
        EEPROM.write(600 + i, conf[i]);
      }

      zone.toCharArray(conf, zone.length() + 1);
      for (i = 0; i < zone.length(); i++) {
        EEPROM.write(700 + i, conf[i]);
      }
      EEPROM.write(0, 2);
      str.remove(a, ((b - 1) - a));
      EEPROM.commit();
      load_saved_config();
    }

    if (str.indexOf(byte(0x09)) > 0) {
      a = str.indexOf(byte(0xFE));
      b = str.indexOf(byte(0xFF));
      str.remove(b);
      /*Hex code with microSD card sync*/
      SDbegintime = millis();
      config_window = true;
      Serial.println();
      Serial.println(F("SD Syncing..."));
      BLE_SD_Index = F("");
      for (i = 0; i < EEPROM.read(11); i++) {
        BLE_SD_Index += (char)EEPROM.read(1100 + i);
      }
      Serial.print(F("SD Card BLE File Index Counter:"));
      Serial.println(BLE_SD_Index.toInt());

      WiFi_SD_Index = F("");
      for (i = 0; i < EEPROM.read(12); i++) {
        WiFi_SD_Index += (char)EEPROM.read(1200 + i);
      }
      Serial.print(F("SD Card WiFi File Index Counter:"));
      Serial.println(WiFi_SD_Index.toInt());

      pCharacteristicWriteRead->setValue((String(F("BLE Entry Count: ")) + BLE_SD_Index).c_str());
      pCharacteristicWriteRead->notify();
      delay(1000);
      readFile("/BLE.csv");
      pCharacteristicWriteRead->setValue((String(F("WiFi Entry Count: ")) + WiFi_SD_Index).c_str());
      pCharacteristicWriteRead->notify();
      delay(1000);
      readFile("/WiFi.csv");
      pCharacteristicWriteRead->setValue(String(F("SD_SYNC_FINISH")).c_str());
      pCharacteristicWriteRead->notify();
      SDelapsedtime = millis();
      Serial.print(F("Time Elapsed Days:"));
      Serial.print(printDigits(((SDelapsedtime - SDbegintime) / 1000) / 86400));
      Serial.print(F("\tHours:"));
      Serial.print(printDigits((((SDelapsedtime - SDbegintime) / 1000) % 86400) / 3600));
      Serial.print(F("\tMinutes:"));
      Serial.print(printDigits((((SDelapsedtime - SDbegintime) / 1000) / 60) % 60));
      Serial.print(F("\tSeconds:"));
      Serial.println(printDigits(((SDelapsedtime - SDbegintime) / 1000) % 60));
      if (BLE_device_connected) {
        Serial.println(F("SD Sync Done"));
      }
      if (!BLE_device_connected) {
        red(bright_percent);
        Serial.println(F("SD Sync Failed"));
      }
      str.remove(a, ((b - 1) - a));
    }

    if (str.indexOf(byte(0x0A)) > 0) {
      a = str.indexOf(byte(0xFE));
      b = str.indexOf(byte(0xFF));
      str.remove(b);
      config_window = true;
      Serial.println(F("SD Files Deleting"));
      deleteFile("/BLE.csv");
      EEPROMString = F("0");
      EEPROM.write(11, EEPROMString.length());
      EEPROMString.toCharArray(conf, EEPROMString.length() + 1);
      for (i = 0; i < EEPROMString.length(); i++) {
        EEPROM.write(1100 + i, conf[i]);
      }
      EEPROM.commit();
      deleteFile("/WiFi.csv");
      EEPROMString = F("0");
      EEPROM.write(12, EEPROMString.length());
      EEPROMString.toCharArray(conf, EEPROMString.length() + 1);
      for (i = 0; i < EEPROMString.length(); i++) {
        EEPROM.write(1200 + i, conf[i]);
      }
      EEPROM.commit();
      str.remove(a, ((b - 1) - a));
      pCharacteristicWriteRead->setValue(String(F("SD_DELETE_FINISH")).c_str());
      pCharacteristicWriteRead->notify();
      Serial.println(F("SD Files Deleted"));
      Soft_Reset_ESP();
    }
  }
  BLEstr = "";
}

String printDigits(byte digits) {
  String Digits;
  if (digits < 10) {
    Digits = String(F("0")) + String(digits);
  } else {
    Digits = String(digits);
  }
  return Digits;
}

String printDigitsYear(byte digits) {
  digits = digits % 100;
  String Digits;
  if (century == 0) {
    if (digits < 10) {
      Digits = String(F("210")) + String(digits);
    } else {
      Digits = String(F("20")) + String(digits);
    }
  } else {
    if (digits < 10) {
      Digits = String(century) + String(F("0")) + String(digits);
    } else {
      Digits = String(century) + String(digits);
    }
  }
  return Digits;
}

void Soft_Reset_ESP() {
  Serial.println(("Soft Resetting AirBeamMini"));
  delay(1000);
  EEPROM.commit();
  Serial.end();
  pt_Serial.end();
  ESP.restart();
}

byte connect_AP() {
  esp_wifi_disconnect();
  WiFi.begin(ssid.c_str(), pass.c_str());  //This needs to be here to connect to WiFi the first time and quickly
  Serial.print(F("Started connecting to WiFi network"));
  for (i = 0; (WiFi.status() != WL_CONNECTED) && i < 60; i++) {  //Check for the connection
    delay(50);
    sensors();
    if (!(i % 10)) {
      Serial.println();
      //WiFi.begin(ssid.c_str(), pass.c_str());  //This needs to be here to connect to WiFi the first time and quickly
    }
    Serial.print(F("."));
  }
  Serial.println();
  if (i == 60) {
    Serial.println(F("Failed connected to WiFi network"));
    return 1;
  } else {
    Serial.println(F("Successfully connected to WiFi network"));
    //Serial.print(F("IP Address:"));
    //Serial.println(WiFi.localIP());
    return 0;
  }
}

char post_WiFi(int data, char caseswitch) {
  String sensor;
  if (caseswitch == 1) {
    sensor = "\tC\t";
  }
  if (caseswitch == 2) {
    sensor = "\tF\t";
  }
  if (caseswitch == 3) {
    sensor = "\tK\t";
  }
  if (caseswitch == 4) {
    sensor = "\tRH\t";
  }
  if (caseswitch == 5) {
    sensor = "\tPM1\t";
  }
  if (caseswitch == 6) {
    sensor = "\tPM2.5\t";
  }
  if (caseswitch == 7) {
    sensor = "\tPM10\t";
  }
  for (i = 0; !wifi_Serial.connect(Aircastingserver.c_str(), 80) && i < 15; i++) {
    sensors();
  }
  Serial.print(data);
  Serial.print(sensor);
  if (i == 15) {
    Serial.print(F("\t\tTCP Not Opened"));
    wifi_Serial.stop();
    Serial.println(F("\t\tTCP Closed"));
    return 1;
  } else {
    Serial.print(F("\t\tTCP Opened"));
    wifi_Serial.print(getPost());
    wifi_Serial.print(postlen);
    wifi_Serial.print(F("\nConnection: close\n\n"));
    wifi_Serial.print(getData_1(caseswitch));
    wifi_Serial.print(getData_3(avgyears, avgmonths, avgdays, avghours, avgmins, avgsecs, data));
    wifi_Serial.print(getData_2(caseswitch));
    //wifi_Serial.print(F("\r\n"));
    wifi_Serial.stop();
    Serial.println(F("\t\tTCP Closed"));
    return 0;
  }
}

char senddata(int data, char caseswitch) {
  char k;
  if (stream_light == 2 && mode == 1 && (uint32_t)(millis() - led_stream) >= 120000) {  //Turn off white LED after 120 seconds
    off();
    stream_light = 0;
  }
  postlen = 0;  //Stores data length
  postlen = postlen + getData_1(caseswitch).length();
  if (mode == 1) {
    postlen = postlen + getData_3(avgyears, avgmonths, avgdays, avghours, avgmins, avgsecs, data).length();
  }
  postlen = postlen + getData_2(caseswitch).length();
  if (mode == 1) {
    for (k = 0; k < 1 && post_WiFi(data, caseswitch); k++) {  //The k represents how many post_WiFi attempts without reconnection
      sensors();
    }
    if (k == 1) {
      //if (post_WiFi(data, caseswitch)) {
      return 1;  //return 1 means failure
    } else {
      return 0;  //return 0 means success
    }
  }
  return 1;
}


// Function to Stream Bluetooth Data
void stream_BLE_data() {
  pCharacteristicPM1_0->setValue(String(String((int)PM1_0) + ";AirBeamMini:" + BLEmac + ";AirBeamMini-PM1;Particulate Matter;PM;micrograms per cubic meter;µg/m³;0;12;35;55;150").c_str());
  pCharacteristicPM2_5->setValue(String(String((int)PM2_5) + ";AirBeamMini:" + BLEmac + ";AirBeamMini-PM2.5;Particulate Matter;PM;micrograms per cubic meter;µg/m³;0;12;35;55;150").c_str());
  pCharacteristicBatteryLevel->setValue(String(Vmeas(1)).c_str());
  delay(10);
  pCharacteristicPM1_0->notify();
  pCharacteristicPM2_5->notify();
  pCharacteristicBatteryLevel->notify();
}

// Function to Stream WiFi
void stream_wifi_data() {
  char h;
  for (h = 0; h < 2 && senddata(PM1_0, 5); h++) {
    sensors();
    connect_AP();
  }
  for (h = 0; h < 2 && senddata(PM2_5, 6); h++) {
    sensors();
    connect_AP();
  }
  /*
    senddata((rawGt0_3um), 8);
    senddata((rawGt0_5um), 9);
    senddata((rawGt1_0um), 10);
    senddata((rawGt2_5um), 11);
    senddata((rawGt5_0um), 12);
    senddata((rawGt10_0um), 13);
    senddata((BAMPM2_5), 14);
    senddata((BAMPM10), 15);
    senddata((GRIMMPM1), 16);
    senddata((GRIMMPM2_5), 17);
    senddata((GRIMMPM10), 18);
  */
}

byte sensors() {
  serial_debugger();
  //Serial.println(getCpuFrequencyMhz());
  USB = digitalRead(V_USB);
  if (USBflag && USB) {
    USBflag = false;
    setCpuFrequencyMhz(160);
    Serial.begin(115200);
    Serial.setTimeout(100);
    //blue(bright_percent); //Comment out the top two lines to test the blue LED with USB detect
  }
  if (!USBflag && !USB) {
    USBflag = true;
    setCpuFrequencyMhz(80);
    Serial.end();
    //red(bright_percent); //Comment out the top one line to test the red LED with USB detect
  }
  return plantower();
}

void load_saved_config() {
  mode_set_eeprom = EEPROM.read(0);
  trigger = 1;
  blue(bright_percent);
  if (mode_set_eeprom == 1) {  //BLE
    Serial.println(F("BLE Mode Selected"));
    mode = 0;
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
      esp_bluedroid_init();
      esp_bluedroid_enable();
      esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
      esp_bt_controller_init(&bt_cfg);
      esp_bt_controller_enable(ESP_BT_MODE_BLE);
    }
    esp_wifi_disconnect();
    esp_wifi_stop();
    read_eeprom();
  }
  if (mode_set_eeprom == 2) {  //Wifi
    Serial.println(F("WiFi Mode Selected"));
    mode = 1;
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    read_eeprom();
    connect_AP();
    for (u = 0; WiFitimesync() && u < 2; u++) {
      connect_AP();
    }
  }
}

String Vmeas(byte batloop) {
  for (byte z = 0; z < batloop; z++) {
    totAnalog = totAnalog - Analog[readAnalogIndex];
    Analog[readAnalogIndex] = analogReadMilliVolts(vmeas);  //(3.3 / 4096.0) * (1499.0 / 1000.0));
    totAnalog = totAnalog + Analog[readAnalogIndex];
    readAnalogIndex = readAnalogIndex + 1;
    if (readAnalogIndex >= numAnalogReads) {
      readAnalogIndex = 0;
    }
    avgBat = (totAnalog * 1499.0) / (numAnalogReads * 1000000.0);
    //avgBat = round(totAnalog * 1.2076904296875) / (numAnalogReads * 1000.0);
  }
  //avgBat = avgBat + 0.01;
  //avgBat = avgBat - 0.01;
  //Serial.println(avgBat);
  //avgBat = 3.04;

  if (avgBat > 3.04) {
    LowBatFlag = false;
  } else {
    LowBatFlag = true;
  }

  BatteryLevel = String(constrain(map(avgBat * 1000, 3050, 4000, 0, 100), 0, 100));
  uint16_t BatteryPercent = BatteryLevel.toInt();
  if (USB && BatteryPercent < 100) {
    BatteryLevel = " " + BatteryLevel + "% Char";
    if (BatteryPercent < 10) {
      BatteryLevel = " " + BatteryLevel;
    }
  }

  if (USB && BatteryPercent == 100) {
    BatteryLevel = BatteryLevel + "% Full";
  }

  if (!USB && BatteryPercent < 100) {
    BatteryLevel = " " + BatteryLevel + "% Load";
    if (BatteryPercent < 10) {
      BatteryLevel = " " + BatteryLevel;
    }
  }
  if (batloop == 1) {
    return String(String(avgBat) + F("V ") + BatteryLevel);
  } else {
    Serial.print(String(String(avgBat) + F("V ") + BatteryLevel));
    return "";
  }
  return "Battery Read Error";
}

void serial_debugger() {
  char b = Serial.read();
  if (b == '`' || b == '-' || b == '=' || b == '~' || b == '\\' || b == '/') {
    RTCgettime();
    Serial.printf("\nAirBeamMini:%s%s\n%02dM/%02dD/%sY% 02dh:%02dm:%02ds %3.2fV(Not Averaged)\n", BLEmac.c_str(), firmwareversion.c_str(), months, days, printDigitsYear(years).c_str(), hours, mins, secs,  (analogReadMilliVolts(vmeas) * 1499.0) / 1000000.0);
  }
  if (b == '`') {
    Serial.println(F("|AirBeamMini Pauser||Now Exit when Entered '' or Auto Exit after 30 Seconds|"));
    String c = "";
    starttime = millis();
    while (1) {
      USB = digitalRead(V_USB);
      if (USB) {
        purple(map((uint32_t)(millis() - starttime), 30000, 0, 0, 100));
      }
      if (!USB) {
        white(map((uint32_t)(millis() - starttime), 30000, 0, 0, 100));
      }
      while (Serial.available()) {
        c += (char)Serial.read();
        starttime = millis();
      }
      if (c.indexOf("Time") > 0) {
        c = "";
        String d = "";
        Serial.print("Enter Date&Time MM/DD/YY-HH:MM:SS Format:");
        while (1) {
          while (Serial.available()) {
            d += (char)Serial.read();
            starttime = millis();
          }
          if (d.length()) {
            months = d.substring(0, d.indexOf('/')).toInt();
            days = d.substring(d.indexOf('/') + 1, d.lastIndexOf('/')).toInt();
            years = d.substring(d.lastIndexOf('/') + 1, d.indexOf('-')).toInt();
            hours = d.substring(d.indexOf('-') + 1, d.indexOf(':')).toInt();
            mins = d.substring(d.indexOf(':') + 1, d.lastIndexOf(':')).toInt();
            secs = d.substring(d.lastIndexOf(':') + 1, d.lastIndexOf(':') + 3).toInt();
            Serial.printf("\n%02dM/%02dD/%sY %02dh:%02dm:%02ds\n", months, days, printDigitsYear(years).c_str(), hours, mins, secs);
            if (months > 0 && days > 0 && years > -1 && hours > -1 && mins > -1 && secs > -1 && years < 100 && months < 13 && days < 32 && hours < 24 && mins < 60 && secs < 60) {
              RTCsettime(hours, mins, secs, days, months, years);
              starttime = 60000;
              Soft_Reset_ESP();
            } else {
              d = "";
              Serial.println("Incorrect Input Try Again");
            }
          }
        }
      }
      if ((c.indexOf("``") > 0) || (uint32_t)(millis() - starttime) >= 30000) {
        Serial.println(F("Pauser Exiting..."));
        if (!trigger) {
          green(bright_percent);
        }
        if (trigger) {
          off();
        }
        break;
      }
    }
    delay(1);
  }
  if (b == '-') {
    Serial.println(F("Resetting AirBeamMini Error Counters"));

    EEPROMString = F("0");
    EEPROM.write(15, EEPROMString.length());
    EEPROMString.toCharArray(conf, EEPROMString.length() + 1);
    for (i = 0; i < EEPROMString.length(); i++) {
      EEPROM.write(1500 + i, conf[i]);
    }

    EEPROMString = F("0");
    EEPROM.write(9, EEPROMString.length());
    EEPROMString.toCharArray(conf, EEPROMString.length() + 1);
    for (i = 0; i < EEPROMString.length(); i++) {
      EEPROM.write(900 + i, conf[i]);
    }

    EEPROMString = F("0");
    EEPROM.write(10, EEPROMString.length());
    EEPROMString.toCharArray(conf, EEPROMString.length() + 1);
    for (i = 0; i < EEPROMString.length(); i++) {
      EEPROM.write(1000 + i, conf[i]);
    }
    EEPROM.commit();
  }
  if (b == '=') {
    Soft_Reset_ESP();
  }
  if (!trigger) {
    if (b == '~') {
      Serial.println("Wait Time Skipped...");
      load_saved_config();
    }
    if (b == '\\') { /* 92 is \ */
      Serial.println(F("AirBeamMini Comprehensive System Check"));
      ESPble_resets = F("");
      for (i = 0; i < EEPROM.read(15); i++) {
        ESPble_resets += (char)EEPROM.read(1500 + i);
      }
      Serial.print(F("BLE Reconnection Error Count:"));
      Serial.println(ESPble_resets.toInt());

      BLE_SD_Index = F("");
      for (i = 0; i < EEPROM.read(11); i++) {
        BLE_SD_Index += (char)EEPROM.read(1100 + i);
      }
      Serial.print(F("File System BLE Index Count:"));
      Serial.println(BLE_SD_Index.toInt());

      WiFi_SD_Index = F("");
      for (i = 0; i < EEPROM.read(12); i++) {
        WiFi_SD_Index += (char)EEPROM.read(1200 + i);
      }
      Serial.print(F("File System WiFi Index Count:"));
      Serial.println(WiFi_SD_Index.toInt());

      Serial.print(F("File System Mount:"));
      if (FFat.begin(true)) {
        Serial.println(F("Good"));
      } else {
        Serial.println(F("Error"));
      }
      Serial.printf("Total Size:\t\t%8uB\n", FFat.totalBytes());
      Serial.printf("Used Size:\t\t%8uB\n", FFat.usedBytes());
      Serial.printf("Free Size:\t\t%8uB\n", FFat.totalBytes() - FFat.usedBytes());
      refreshFile("/BLE.csv");
      refreshFile("/WiFi.csv");

      Serial.println(F("Checking AirBeamMini Plantower:"));
      sensor_average();
      mode_set_eeprom = EEPROM.read(0);  // Saving EEPROM location 0 to use later in the code
      Serial.print(F("Current Mode:"));
      if (mode_set_eeprom == 0 || mode_set_eeprom == 0xFF) {
        Serial.println(F("Defaulted Zero Config"));
        config_window = true;
      }
      if (mode_set_eeprom == 1) {
        Serial.print(F("BLE Config"));
        if (!RTCgettime()) {
          Serial.print(F(" Frozen"));
          config_window = true;
        }
        Serial.println();
      }
      if (mode_set_eeprom == 2) {
        Serial.println(F("WiFi Config"));
        config_window = false;
      }
      Serial.println(F("System Check Complete"));
      Serial.println();
    }
    if (b == '/') { /* 47 is / */
      int battery_flag = 0;
      int plantower_count_flag = 0;
      Serial.println(F("AirBeamMini System Checker"));
      for (i = 0; i <= 100; i++) {
        white(i);
        Vmeas(1);
        delay(10);
      }
      //Checking Battery
      if (avgBat > 2.5 && avgBat < 4.5) {
        battery_flag = 1;
        //Good
      } else {
        battery_flag = 0;
        //Error
      }

      if (plantowercount != 0) {
        plantower_count_flag = 1;
        //Good
      } else {
        plantower_count_flag = 0;
        //Error
      }

      for (i = 100; i >= 0; i--) {
        white(i);
        delay(10);
      }
      if (battery_flag == 0) {
        Serial.print(F("Battery Voltage Out of Range, must be greater than 2.5V & less than 4.5V. Currently:"));
        Serial.print(avgBat);
        Serial.println("V");
      }
      if (plantower_count_flag == 0) {
        Serial.println(F("Plantower Sensor Failed"));
      }
      if (battery_flag == 1 && plantower_count_flag == 1) {
        Serial.println(F("All Test Passed"));
      }
      Serial.println(F("System Check Complete"));
      Serial.println();
    }
  }
}

class BLE_Connection_Status : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    BLE_device_connected = true;
    if (!trigger) {
      Serial.println(F("BLE Connect"));
      blue(bright_percent);
      config_window = true;
    }
    if (mode == 0 && (uint32_t)(millis() - BLE_disconnection_time) < 10000) {
      ESPble_resets = F("");
      for (i = 0; i < EEPROM.read(15); i++) {
        ESPble_resets += (char)EEPROM.read(1500 + i);
      }
      EEPROMString = String(ESPble_resets.toInt() + 1);
      EEPROM.write(15, EEPROMString.length());
      EEPROMString.toCharArray(conf, EEPROMString.length() + 1);
      for (i = 0; i < EEPROMString.length(); i++) {
        EEPROM.write(1500 + i, conf[i]);
      }
      EEPROM.commit();
    }
  }
  void onDisconnect(BLEServer* pServer) {
    BLE_device_connected = false;
    BLEDevice::startAdvertising();
    if (!trigger) {
      Serial.println(F("BLE Disconnect"));
      green(bright_percent);
      BLEstr = "";
      if (!(rtc.getYear() < 2023 && rtc.getYear() < printDigitsYear(eepromyears).toInt())) {
        config_window = false;
        configtime_expired = millis() + 180000;
      } else {
        config_window = true;
      }
    }
    if (mode == 0) {
      BLE_disconnection_time = millis();
    }
  }
};

class Receive : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    BLEstr = "";
    BLEstr = pCharacteristic->getValue().c_str();
  }
};

void setup() {
  begintime = millis();
  setCpuFrequencyMhz(160);
  Serial.begin(115200);
  pt_Serial.begin(9600, SERIAL_8N1, 6, 7);  //RX, TX
  Serial.setTimeout(100);
  pt_Serial.setTimeout(50);
  EEPROM.begin(EEPROM_SIZE);
  pinMode(V_USB, INPUT);
  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);
  ledcSetup(2, 1000, 8);
  ledcAttachPin(red_led, 0);
  ledcAttachPin(green_led, 1);
  ledcAttachPin(blue_led, 2);
  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
  green(bright_percent);
  mode_set_eeprom = EEPROM.read(0);  // Saving EEPROM location 0 to use later in the code
  uint64_t chipid = ESP.getEfuseMac();
  //Reorganized the BLE MAC Address
  char chipid1 = (chipid & 0x0000F00000000000) >> 44;
  char chipid2 = (chipid & 0x00000F0000000000) >> 40;
  char chipid3 = (chipid & 0x000000F000000000) >> 36;
  char chipid4 = (chipid & 0x0000000F00000000) >> 32;
  char chipid5 = (chipid & 0x00000000F0000000) >> 28;
  char chipid6 = (chipid & 0x000000000F000000) >> 24;
  char chipid7 = (chipid & 0x0000000000F00000) >> 20;
  char chipid8 = (chipid & 0x00000000000F0000) >> 16;
  char chipid9 = (chipid & 0x000000000000F000) >> 12;
  char chipid10 = (chipid & 0x0000000000000F00) >> 8;
  char chipid11 = (chipid & 0x00000000000000F0) >> 4;
  char chipid12 = (chipid & 0x000000000000000F);
  BLEmac = String(chipid11, HEX) + String(chipid12, HEX) + String(chipid9, HEX) + String(chipid10, HEX) + String(chipid7, HEX) + String(chipid8, HEX) + String(chipid5, HEX) + String(chipid6, HEX) + String(chipid3, HEX) + String(chipid4, HEX) + String(chipid1, HEX) + String(chipid2, HEX);
  BLEDevice::init((String(F("AirBeamMini:")) + BLEmac).c_str());
  BLEDevice::setMTU(517);
  uint16_t SERVICE_UUID = 0xFFDD;
  uint16_t WRITE_READ_UUID = 0xFFDE;
  uint16_t SDSYNC_UUID = 0xFFDF;
  uint16_t TEMPC_UUID = 0xFFE0;
  uint16_t TEMPF_UUID = 0xFFE1;
  uint16_t TEMPK_UUID = 0xFFE2;
  uint16_t HUMI_UUID = 0xFFE3;
  uint16_t PM1_0_UUID = 0xFFE4;
  uint16_t PM2_5_UUID = 0xFFE5;
  uint16_t PM10_0_UUID = 0xFFE6;
  uint16_t BATTERYLEVEL_UUID = 0xFFE7;
  pServer = BLEDevice::createServer();
  pService = pServer->createService(BLEUUID((uint16_t)SERVICE_UUID), 50, 0);  //Characteristics(Services),Handles
  pServer->setCallbacks(new BLE_Connection_Status());
  pCharacteristicWriteRead = pService->createCharacteristic(
    BLEUUID((uint16_t)WRITE_READ_UUID),
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicSDSync = pService->createCharacteristic(
    BLEUUID((uint16_t)SDSYNC_UUID),
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTempC = pService->createCharacteristic(
    BLEUUID((uint16_t)TEMPC_UUID),
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTempF = pService->createCharacteristic(
    BLEUUID((uint16_t)TEMPF_UUID),
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTempK = pService->createCharacteristic(
    BLEUUID((uint16_t)TEMPK_UUID),
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicHumi = pService->createCharacteristic(
    BLEUUID((uint16_t)HUMI_UUID),
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicPM1_0 = pService->createCharacteristic(
    BLEUUID((uint16_t)PM1_0_UUID),
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicPM2_5 = pService->createCharacteristic(
    BLEUUID((uint16_t)PM2_5_UUID),
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicPM10_0 = pService->createCharacteristic(
    BLEUUID((uint16_t)PM10_0_UUID),
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicBatteryLevel = pService->createCharacteristic(
    BLEUUID((uint16_t)BATTERYLEVEL_UUID),
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicWriteRead->setCallbacks(new Receive());
  pCharacteristicWriteRead->addDescriptor(new BLE2902());
  pCharacteristicSDSync->addDescriptor(new BLE2902());
  pCharacteristicTempC->addDescriptor(new BLE2902());
  pCharacteristicTempF->addDescriptor(new BLE2902());
  pCharacteristicTempK->addDescriptor(new BLE2902());
  pCharacteristicHumi->addDescriptor(new BLE2902());
  pCharacteristicPM1_0->addDescriptor(new BLE2902());
  pCharacteristicPM2_5->addDescriptor(new BLE2902());
  pCharacteristicPM10_0->addDescriptor(new BLE2902());
  pCharacteristicBatteryLevel->addDescriptor(new BLE2902());
  pService->start();
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  //pAdvertising->setScanResponse(false);
  //pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  //pAdvertising->setScanResponse(true);
  //pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  //pAdvertising->setMaxPreferred(0x12);
  //BLEDevice::init((String(F("AirBeamMini|")) + BLEmac + firmwareversion).c_str());
  //esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  //esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  //esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
  /*
    Serial.println("Power Settings (ADV,SCAN,DEFAULT):");            //all should show index 7, aka +9dbm
    Serial.println(esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV));
    Serial.println(esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_SCAN));
    Serial.println(esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT));
  */
  Serial.println();
  sensors();
  RTCgettime();
  Serial.printf("AirBeamMini:%s%s\n%02dM/%02dD/%sY% 02dh:%02dm:%02ds ", BLEmac.c_str(), firmwareversion.c_str(), months, days, printDigitsYear(years).c_str(), hours, mins, secs);
  Vmeas(20);
  Serial.println();
  Serial.print(F("File System Mount:"));
  if (FFat.begin(true)) {
    Serial.println(F("Good"));
  } else {
    Serial.println(F("Error"));
  }
  Serial.printf("Total Size:\t\t%8uB\n", FFat.totalBytes());
  Serial.printf("Used Size:\t\t%8uB\n", FFat.usedBytes());
  Serial.printf("Free Size:\t\t%8uB\n", FFat.totalBytes() - FFat.usedBytes());
  refreshFile("/BLE.csv");
  refreshFile("/WiFi.csv");
  Serial.print(F("Current Mode:"));
  if (mode_set_eeprom == 0 || mode_set_eeprom == 0xFF) {
    Serial.println(F("Defaulted Zero Config"));
    config_window = true;
  }
  if (mode_set_eeprom == 1) {
    Serial.print(F("BLE Config"));
    if (!RTCgettime()) {
      config_window = true;
      Serial.print(F(" Frozen"));
    }
    Serial.println();
  }
  if (mode_set_eeprom == 2) {
    Serial.println(F("WiFi Config"));
    config_window = false;
  }
  elapsedtime = millis();  // Used to add time the configuration window for the time lost using millis()*/
  while (!trigger) {
    if (sensors() && (uint32_t)(millis() - starttime0) >= sensor_timer) {
      starttime0 = millis();
    }
    if (!(config_window || (millis() <= configtime_expired + (elapsedtime - begintime)))) {  //Updates BLE status within the one minute window to get information
      load_saved_config();
    }
    analyze_buf(BLEstr);
    //Serial.println(temperatureRead());
  }
}

void loop() {
  if (sensors() && (uint32_t)(millis() - starttime0) >= sensor_timer) {
    starttime0 = millis();
  }

  if (LowBatFlag) {
    if (((uint32_t)(millis() - starttime2) >= 1000)) {
      starttime2 = millis();
      MobileLED++;
    }
    if (MobileLED == 9) {
      magenta(bright_percent);
    }
    if (MobileLED == 10) {
      off();
      MobileLED = 0;
    }
  } else if (stream_light == 1 && ((mode == 0 && BLE_device_connected) || (mode == 1))) {  //The statement to start the white LED after reconnection of Bluetooth or WiFi
    led_stream = millis();
    white(bright_percent);
    stream_light = 2;
  } else if (stream_light == 2 && (mode == 0 || mode == 1) && ((uint32_t)(millis() - led_stream) >= 120000)) {  //Turn off white LED after 120 seconds
    off();
    stream_light = 0;
  }

  if (mode == 0 && RTCgettime() && (secs != verifytime || ((uint32_t)(millis() - starttime1) >= BLE_timer))) {  //Stream Over Bluetooth every second
    starttime1 = millis();
    sensor_average();
    if (!LowBatFlag) {
      if (stream_light != 2) {
        MobileLED++;
      }
      if (MobileLED == 9 || stream_light == 2) {
        if (BLE_device_connected && stream_light == 0) {
          white(bright_percent);
        }
        if (!BLE_device_connected) {
          stream_light = 1;
          yellow(bright_percent);
        }
      }
      if (MobileLED == 10) {
        off();
        MobileLED = 0;
      }
    }
    verifytime = secs;  //has to be placed here to produce a timing gap difference to prevent duplicate seconds and also no skip seconds
  }


  if (mode == 1 && (mins != verifytime || ((uint32_t)(millis() - starttime1) >= wifi_timer))) {  //Stream Over WiFi every minute averaged
    starttime1 = millis();
    if (RTCgettime()) {
      sensor_average();
      stream_wifi_data();
    } else {
      blue(bright_percent);
      for (u = 0; WiFitimesync() && u < 2; u++) {
        connect_AP();
      }
      off();
    }
    verifytime = mins;
    timesyncflag = false;
  }

  if (mode == 1 && mins == 0 && !timesyncflag) {
    timesyncflag = true;
    blue(bright_percent);
    for (u = 0; WiFitimesync() && u < 2; u++) {
      connect_AP();
    }
    off();
    if ((stream_light == 2 && (uint32_t)(millis() - led_stream) >= 120000)) {  //Incase time sync happens within two minutes of startup of WiFi, it will not turn off white LED
      off();
      stream_light = 0;
    } else if (((uint32_t)(millis() - led_stream) < 120000)) {
      white(bright_percent);
      stream_light = 2;
    }
  }
}
