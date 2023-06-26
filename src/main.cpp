// VERSION ROSVER WPA2 ESP8266

#include <Arduino.h>
#include "main.hpp"

// Init to default values; if they have been chaged they will be readed later, on initialization

char aireciudadano_device_name[30] = "AireCiudadano_Test1wpa2"; // Nombre de la estacion

String wifissid = "TPwpa2";
char wifi_user[24] = "prueba1";
char wifi_password[24] = "daniel2022";

String aireciudadano_device_id;
char wifi_passwpa2[24];
bool ConfigPortalSave = false;

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
  while (!Serial)
  {
    delay(500); // wait 0.5 seconds for connection
  }
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println(F("##### Inicializando Medidor Aire Ciudadano #####"));

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);

  aireciudadano_device_id = aireciudadano_device_name;

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
    Connect_WiFi();
  }
  else
  {
    Serial.println(F("Wifi conectada"));
  }

  delay(5000);
}
void Connect_WiFi()
{ // Connect to WiFi

  WiFi.disconnect(true); // disconnect from wifi to set new wifi connection
  WiFi.mode(WIFI_STA);   // init wifi mode

  //  If there are not wifi identity and wifi password defined, proceed to traight forward configuration

  String wifi_ssid = wifissid;
  // String wifi_password = WiFi.psk()); // your network psk password
  Serial.println(F("Attempting to authenticate with WPA2 Enterprise..."));
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());
  Serial.print(F("Identity: "));
  Serial.println(wifi_user);
  Serial.print(F("Password: "));
  Serial.println(wifi_password);

  // Setting ESP into STATION mode only (no AP mode or dual mode)
  wifi_set_opmode(STATION_MODE);
  struct station_config wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  strcpy((char *)wifi_config.ssid, wifi_ssid.c_str());
  strcpy((char *)wifi_config.password, wifi_password);
  wifi_station_set_config(&wifi_config);
  // Clean up to be sure no old data is still inside
  wifi_station_clear_cert_key();
  wifi_station_clear_enterprise_ca_cert();
  wifi_station_clear_enterprise_identity();
  wifi_station_clear_enterprise_username();
  wifi_station_clear_enterprise_password();
  wifi_station_clear_enterprise_new_password();

  wifi_station_set_wpa2_enterprise_auth(1);

  // Set up authentication
  wifi_station_set_enterprise_identity((uint8 *)wifi_user, strlen(wifi_user));
  wifi_station_set_enterprise_username((uint8 *)wifi_user, strlen(wifi_user));
  wifi_station_set_enterprise_password((uint8 *)wifi_password, strlen((char *)wifi_password));
  wifi_station_set_enterprise_new_password((uint8 *)wifi_password, strlen((char *)wifi_password));
  wifi_station_connect();

  // Timestamp for connection timeout
  int wifi_timeout_start = millis();

  while ((WiFi.status() != WL_CONNECTED) && ((millis() - wifi_timeout_start) < WIFI_CONNECT_TIMEOUT))
  {
    delay(500); // wait 0.5 seconds for connection
    Serial.println(F("."));
  }

  // Status
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(F("WiFi not connected"));
  }
  else
  {
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