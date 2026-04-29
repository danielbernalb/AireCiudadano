#include "EEPROM.h"
#include "Read_EEPROM.h"

extern String ssid;
extern String pass;
extern String uuid;
extern String uuidauth;
extern String zone;
extern int mode;

extern int i;

void read_eeprom() {
  /*Clear string to avoid overlapping*/
  uuid = F("");
  uuidauth = F("");

  for (i = 0; i < EEPROM.read(1); i++) {
    uuid += (char)EEPROM.read(100 + i);
  }

  for (i = 0; i < EEPROM.read(2); i++) {
    uuidauth += (char)EEPROM.read(200 + i);
  }

  Serial.print(F("UUID:"));
  Serial.println(uuid);
  Serial.print(F("UUIDAuth:"));
  Serial.println(uuidauth);

  /*Read these only if Wifi was selected*/
  if (mode == 1) {
    ssid = F("");
    pass = F("");
    zone = F("");

    for (i = 0; i < EEPROM.read(5); i++) {
      ssid += (char)EEPROM.read(500 + i);
    }

    for (i = 0; i < EEPROM.read(6); i++) {
      pass += (char)EEPROM.read(600 + i);
    }

    for (i = 0; i < EEPROM.read(7); i++) {
      zone += (char)EEPROM.read(700 + i);
    }

    Serial.print(F("SSID:"));
    Serial.println(ssid);
    Serial.println(pass);
    Serial.print(F("Time Zone:"));
    Serial.println(zone);
  }
}
