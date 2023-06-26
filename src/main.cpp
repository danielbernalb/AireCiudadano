// VERSION ROSVER WPA2 ESP8266

#include <Arduino.h>
#include "main.hpp"

////////////////////////////////
// Modo de comunicaciones del sensor:
#define Rosver2 true    // Dejar menu de portal cautivo solo con SSID, identidad y password
#define Rosver3 true    // Eliminar Wifimanager
#define Rosver4 true    // Minimo minimo
#define RosverRTOS true // RosverRTOS version

String aireciudadano_device_id;

// Init to default values; if they have been chaged they will be readed later, on initialization
struct MyConfigStruct
{
  char aireciudadano_device_name[30] = "AireCiudadano_DBB_01"; // Nombre de la estacion
  char wifi_user[24] = "prueba1";
  char wifi_password[24] = "daniel2022";
} eepromConfig;

String wifissid = "TPwpa2";

char wifi_passwpa2[24];
bool ConfigPortalSave = false;

// Save config values to EEPROM
#include <ESP_EEPROM.h>

// WiFi
#include <ESP8266WiFi.h> // Wifi ESP8266
extern "C"
{
#include "user_interface.h"

#include "wpa2_enterprise.h"

#include "c_types.h"
  bool PortalFlag = false;
}

#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
WiFiClient wifi_client;
const int WIFI_CONNECT_TIMEOUT = 10000; // 10 seconds
int wifi_status = WL_IDLE_STATUS;
WiFiServer wifi_server(80); // to check if it is alive
                            // String wifi_ssid = WiFi.SSID();                  // your network SSID (name)
                            // String wifi_password = WiFi.psk();               // your network psk password

#include <ESP8266WebServer.h>
#include <DNSServer.h>

#define LEDPIN 2

///////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // Initialize serial port for serial monitor in Arduino IDE
  Serial.begin(115200);
  delay(100);
  while (!Serial)
  {
    delay(500); // wait 0.5 seconds for connection
  }
  Serial.setDebugOutput(true);
  Serial.println(F(""));
  Serial.println(F(""));
  enable_wifi_enterprise_patch();
  Serial.print(F("ESP CoreVersion: "));
  Serial.println(ESP.getCoreVersion());
  Serial.printf("SDK version: %s\n", system_get_sdk_version());
  Serial.printf("Free Heap: %4d\n", ESP.getFreeHeap());

  // print info
  Serial.println();
  Serial.println(F("##### Inicializando Medidor Aire Ciudadano #####"));

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);

  aireciudadano_device_id = eepromConfig.aireciudadano_device_name;

  delay(100);

  // Attempt to connect to WiFi network:
  Connect_WiFi();

  // Init control loops
  Serial.println(F(""));
  Serial.println(F("### Configuración del medidor AireCiudadano finalizada ###\n"));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println(F("Error wifi"));
      WiFi.reconnect();
    }
    else
    {
      Serial.println(F("Wifi conectada"));
    }

    delay(1000);

}
void Connect_WiFi()
{ // Connect to WiFi

  WiFi.disconnect(true); // disconnect from wifi to set new wifi connection
  WiFi.mode(WIFI_STA);   // init wifi mode

  //  If there are not wifi identity and wifi password defined, proceed to traight forward configuration

#if !Rosver3
  String wifi_ssid = WiFi.SSID(); // your network SSID (name)
#else
  String wifi_ssid = wifissid;
#endif
  // String wifi_password = WiFi.psk()); // your network psk password
  Serial.println(F("Attempting to authenticate with WPA2 Enterprise..."));
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());
  Serial.print(F("Identity: "));
  Serial.println(eepromConfig.wifi_user);
  Serial.print(F("Password: "));
  Serial.println(eepromConfig.wifi_password);

  // Setting ESP into STATION mode only (no AP mode or dual mode)
  wifi_set_opmode(STATION_MODE);
  struct station_config wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  strcpy((char *)wifi_config.ssid, wifi_ssid.c_str());
  strcpy((char *)wifi_config.password, eepromConfig.wifi_password);
  wifi_station_set_config(&wifi_config);
  // uint8_t target_esp_mac[6] = {0x24, 0x0a, 0xc4, 0x9a, 0x58, 0x28};
  // wifi_set_macaddr(STATION_IF,target_esp_mac);
  //    wifi_station_set_wpa2_enterprise_auth(1);
  // Clean up to be sure no old data is still inside
  wifi_station_clear_cert_key();
  wifi_station_clear_enterprise_ca_cert();
  wifi_station_clear_enterprise_identity();
  wifi_station_clear_enterprise_username();
  wifi_station_clear_enterprise_password();
  wifi_station_clear_enterprise_new_password();

  wifi_station_set_wpa2_enterprise_auth(1);

  // Set up authentication
  wifi_station_set_enterprise_identity((uint8 *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));
  wifi_station_set_enterprise_username((uint8 *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));
  wifi_station_set_enterprise_password((uint8 *)eepromConfig.wifi_password, strlen((char *)eepromConfig.wifi_password));
  wifi_station_set_enterprise_new_password((uint8 *)eepromConfig.wifi_password, strlen((char *)eepromConfig.wifi_password));
  wifi_station_connect();

  // Timestamp for connection timeout
  int wifi_timeout_start = millis();

  // Wait for warming time while blinking blue led AQUI ESTA EL PROBLEMA DEL DHCP
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - wifi_timeout_start) < WIFI_CONNECT_TIMEOUT))
  {
    delay(500); // wait 0.5 seconds for connection
    Serial.println(F("."));
  }

  // Status
  if (WiFi.status() != WL_CONNECTED)
  {
    err_wifi = true;
    Serial.println(F("WiFi not connected"));
  }
  else
  {
    err_wifi = false;
    Serial.println(F("WiFi connected"));

    // start the web server on port 80
    wifi_server.begin();

    digitalWrite(LEDPIN, LOW);
    delay(800);
    digitalWrite(LEDPIN, HIGH);
    delay(800);
    digitalWrite(LEDPIN, LOW);
    delay(800);
    digitalWrite(LEDPIN, HIGH);
    delay(800);
    digitalWrite(LEDPIN, LOW);
  }
}