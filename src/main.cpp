//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AireCiudadano medidor Fijo - Medidor de PM2.5 abierto, medición opcional de humedad y temperatura.
// Más información en: aireciudadano.com
// Este firmware es un fork del proyecto Anaire (https://www.anaire.org/) recomendado para la medición de CO2.
// 25/06/2023 info@aireciudadano.com
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Pendientes:
// Revisar todas las versiones y pruebas para pasar a v2.0
// MODIFICACIONES EXTERNAS:
// Modificado libreria WifiManager para compatibilidad
// Modificado PubSubClient.cpp : para quitar warning

// VERSION ROSVER WPA2 ESP8266

#include <Arduino.h>
#include "main.hpp"

////////////////////////////////
// Modo de comunicaciones del sensor:
#define Rosver true     // Set to true URosario version
#define Rosver2 true    // Dejar menu de portal cautivo solo con SSID, identidad y password
#define Rosver3 false    // Eliminar Wifimanager
#define RosverRTOS true  // RosverRTOS version

#define PreProgSensor false // Variables de sensor preprogramadas:
                            // Latitude: char sensor_lat[10] = "xx.xxxx";
                            // Longitude: char sensor_lon[10] = "xx.xxxx";
                            // Valores de configuración: char ConfigValues[9] = "000xxxxx";
                            // Nombre estación: char aireciudadano_device_name[36] = "xxxxxxxxxxxxxx";


bool PMSsen = false;        // Sensor Plantower PMS
bool AdjPMS = false;        // PMS sensor adjust
bool SHT31sen = false;      // Sensor SHT31 / SHT4x humedad y temperatura

bool AmbInOutdoors = false; // Set to true if your sensor is indoors measuring outside environment, false if is outdoors

uint8_t CustomValue = 0;
uint16_t CustomValtotal = 0;
char CustomValTotalString[9] = "00000000";
uint32_t IDn = 0;
String chipIdHEX;
uint32_t chipId = 0;

// device id, automatically filled by concatenating the last three fields of the wifi mac address, removing the ":" in betweeen, in HEX format. Example: ChipId (HEX) = 85e646, ChipId (DEC) = 8775238, macaddress = E0:98:06:85:E6:46
String sw_version = "2.0";
String aireciudadano_device_id;
uint8_t Swver;

// Init to default values; if they have been chaged they will be readed later, on initialization
struct MyConfigStruct
{
  uint16_t PublicTime = 1;     // Publication Time
                               //  uint16_t MQTT_port = 80;                           // MQTT port; Default Port on 80
                               //  char MQTT_server[30] = "sensor.aireciudadano.com"; // MQTT server url or public IP address.
#if !PreProgSensor
#if !Rosver2
  char sensor_lat[10] = "0.0"; // Sensor latitude  GPS
  char sensor_lon[10] = "0.0"; // Sensor longitude GPS
#else
  char sensor_lat[10] = "4.6947"; // Sensor latitude  GPS
  char sensor_lon[10] = "-74.0946"; // Sensor longitude GPS
#endif
  char ConfigValues[10] = "000100000";
  char aireciudadano_device_name[30]; // Device name; default to aireciudadano_device_id
#else
  char sensor_lat[10] = "4.69375";   // Aquí colocar la Latitud del sensor
  char sensor_lon[10] = "-74.09382"; // Colocar la Longitud del sensor
  char ConfigValues[10] = "000010111";
  char aireciudadano_device_name[30] = "AireCiudadano_DBB_01"; // Nombre de la estacion
#endif
  char wifi_user[24];     // WiFi user to be used on WPA Enterprise. Default to null (not used)
  char wifi_password[24]; // WiFi password to be used on WPA Enterprise. Default to null (not used)
} eepromConfig;

char wifi_passwpa2[24];
bool ConfigPortalSave = false;

#if PreProgSensor
// const char *ssid = "Techotyva";
// const char *password = "Layuyux31";
const char *ssid = "TPred";
const char *password = "apt413sago16";
// const char *ssid = "Rosa";
// const char *password = "Rudysicha";
char aireciudadano_device_nameTemp[30] = {0};
#endif

// Save config values to EEPROM
#include <ESP_EEPROM.h>

// Measurements
float PM25_value = 0;           // PM25 measured value
float PM25_value_ori = 0;       // PM25 original measured value in PMS adjust TRUE
float PM25_accumulated = 0;     // Accumulates pm25 measurements for a MQTT period
float PM25_accumulated_ori = 0; // Accumulates pm25 measurements for a MQTT period in PMS Adjust TRUE
float temperature;              // Read temperature as Celsius
float humidity;                 // Read humidity in %
int PM25_samples = 0;           // Counts de number of samples for a MQTT period
int pm25int;                    // PM25 publicado
int pm25intori;

int temp;
int humi;

float latitudef = 0.0;
float longitudef = 0.0;

bool err_wifi = false;
bool err_MQTT = false;
bool err_sensor = false;
bool FlagDATAicon = false;
bool NoSensor = false;

// Measurements loop: time between measurements
unsigned int measurements_loop_duration = 1000; // 1 second
unsigned long measurements_loop_start;          // holds a timestamp for each control loop start

unsigned int Con_loop_times = 0;

// MQTT loop: time between MQTT measurements sent to the cloud
unsigned long MQTT_loop_start;          // holds a timestamp for each cloud loop start
unsigned long lastReconnectAttempt = 0; // MQTT reconnections

// Errors loop: time between error condition recovery
unsigned int errors_loop_duration = 60000; // 60 seconds
unsigned long errors_loop_start;           // holds a timestamp for each error loop start

#include <Wire.h>

#include <SPI.h>

#include "PMS.h"

#include <SoftwareSerial.h>

#define PMS_TX 0  // PMS TX pin
#define PMS_RX 16 // PMS RX pin

SoftwareSerial pmsSerial(PMS_TX, PMS_RX); // SoftwareSerial(rxPin, txPin)

PMS pms(pmsSerial);
PMS::DATA data;
// bool PMSflag = false;

#include "Adafruit_SHT4x.h"
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
bool SHT31flag = false;
byte failh = 0;

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

WiFiManager wifiManager;

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

// MQTT
#include <PubSubClient.h>
char MQTT_message[256];
PubSubClient MQTT_client(wifi_client);
char received_payload[384];
String MQTT_send_topic;
String MQTT_receive_topic;

// JSON
#include <ArduinoJson.h>
StaticJsonDocument<384> jsonBuffer;

// For http binary updates
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

bool ResetFlag = false;
bool DeepSleepFlag = false;
bool NoiseBUTTONFlag = false;
// int reason;

// to know when there is an updating process in place
bool updating = false;

// To know when the device is in the following states
bool InCaptivePortal = false;
bool Calibrating = false;

#define LEDPIN 2

bool FlagLED = false;

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
  Serial.printf("Free Heap: %4d\n",ESP.getFreeHeap());

  uint16_t Resetvar = 0;
  Serial.print(F("CPU reset reason: "));
  rst_info *rinfo = ESP.getResetInfoPtr();
  Serial.println(rinfo->reason);
  Resetvar = rinfo->reason;
  ResetFlag = true;
  if (Resetvar == 1 || Resetvar == 2 || Resetvar == 3 || Resetvar == 4)
  {
    ResetFlag = false;
    Serial.print(F("Resetvar: false"));
  }
  else
  {
    delay(100);
#if Rosver
    delay(1900);
#endif
  }
  Serial.print(F("Resetvar: "));
  Serial.println(Resetvar);

  // print info
  Serial.println();
  Serial.println(F("##### Inicializando Medidor Aire Ciudadano #####"));

#if PreProgSensor
  Serial.print(F("T1: "));
  Serial.println(eepromConfig.aireciudadano_device_name);
  strcpy(aireciudadano_device_nameTemp, eepromConfig.aireciudadano_device_name);
#endif
  Read_EEPROM(); // Read EEPROM config values //MIRAR SDflag   ////////////////////TEST
#if PreProgSensor
  strncpy(eepromConfig.aireciudadano_device_name, aireciudadano_device_nameTemp, sizeof(eepromConfig.aireciudadano_device_name));
  Serial.print(F("T2:"));
  Serial.println(eepromConfig.aireciudadano_device_name);
#endif

  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);

  aireciudadano_device_id = eepromConfig.aireciudadano_device_name;

  float Floatver = sw_version.toFloat();
  Swver = Floatver * 10;
  Serial.print(F("SW version: "));
  Serial.println(sw_version);

  // Get device id
  Get_AireCiudadano_DeviceId();

  delay(100);

  MQTT_send_topic = "measurement"; // measurement are sent to this topic
  MQTT_receive_topic = "config/" + aireciudadano_device_id; // Config messages will be received in config/id

  // Print initial configuration
  Print_Config();

  // Set Latitude and Longitude
  latitudef = atof(eepromConfig.sensor_lat);
  longitudef = atof(eepromConfig.sensor_lon);

  // Start Captive Portal for 60 seconds
  if (ResetFlag == true)
  {
    Serial.println("Test_Sensor");
    Test_Sensor();

#if !Rosver3
    Start_Captive_Portal();
#endif
    delay(100);
  }
    // Attempt to connect to WiFi network:
    Connect_WiFi();

    // Attempt to connect to MQTT broker
    if (!err_wifi)
    {
      Init_MQTT();
    }
  // Initialize and warm up PM25 sensor
  Setup_Sensor();  

  // Init control loops
  measurements_loop_start = millis();
  errors_loop_start = millis();
  MQTT_loop_start = millis();

  // Get device id
  IDn = 0;
  Aireciudadano_Characteristics();

  Serial.println(F(""));
  Serial.println(F("### Configuración del medidor AireCiudadano finalizada ###\n"));

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // If a firmware update is in progress do not do anything else
  if (updating)
  {
    return;
  }

  // Measurement loop
  if ((millis() - measurements_loop_start) >= measurements_loop_duration)
  {

    // New timestamp for the loop start time
    measurements_loop_start = millis();

    // Read sensors

    Read_Sensor();

    if (FlagLED == true)
      FlagLED = false;
    else
      digitalWrite(LEDPIN, HIGH);
    if (NoSensor == false)
    {
      if (PM25_value >= 0)
      {
        // Accumulates samples
        PM25_accumulated += PM25_value;
        if (AdjPMS == true)
          PM25_accumulated_ori += PM25_value_ori;
        PM25_samples++;
        Con_loop_times++;
      }
    }
    else
    {
      Serial.println(F("Medidor No configurado"));
  }

    // MQTT loop
    // if ((millis() - MQTT_loop_start) >= (eepromConfig.PublicTime * 60000))
      if ((millis() - MQTT_loop_start) >= (eepromConfig.PublicTime * 5000))
    //  if ((millis() - MQTT_loop_start) >= (1 * 60000))
    {
      // New timestamp for the loop start time
      MQTT_loop_start = millis();

      // Message the MQTT broker in the cloud app to send the measured values
      if ((!err_wifi) && (PM25_samples > 0))
      {
        Send_Message_Cloud_App_MQTT();
      }
      // Reset samples after sending them to the MQTT server
      PM25_accumulated = 0.0;
      PM25_accumulated_ori = 0.0;
      PM25_samples = 0.0;
    }
  }

  // Errors loop
  if ((millis() - errors_loop_start) >= errors_loop_duration)
  {

    // New timestamp for the loop start time
    errors_loop_start = millis();

    // Try to recover error conditions
    if (err_sensor)
    {
      Serial.println(F("--- err_sensor"));
      // Setup_Sensor();  // Init pm25 sensors
    }

      if (WiFi.status() != WL_CONNECTED)
      {
        Serial.println(F("--- err_wifi"));
        err_wifi = true;
        WiFi.reconnect();
      }
      else
      {
        err_wifi = false;
      }

      // Reconnect MQTT if needed
      if ((!MQTT_client.connected()) && (!err_wifi))
      {
        Serial.println(F("--- err_mqtt"));
        err_MQTT = true;
        FlagDATAicon = false;
      }

      // Reconnect MQTT if needed
      if ((err_MQTT) && (!err_wifi))
      {
        Serial.println(F("--- MQTT reconnect"));
        // Attempt to connect to MQTT broker
        Init_MQTT();
      }

    // From here, all other tasks performed outside of measurements, MQTT and error loops

    // if not there are not connectivity errors, receive MQTT messages
    if ((!err_MQTT) && (!err_wifi))
    {
      MQTT_client.loop();
    }

    // Process wifi server requests
    Check_WiFi_Server();
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

// Print wifi status on serial monitor
// ESP8266

void Print_WiFi_Status_ESP8266()
{

  // Get current status
  //  WL_CONNECTED: assigned when connected to a WiFi network;
  //  WL_NO_SHIELD: assigned when no WiFi shield is present;
  //  WL_IDLE_STATUS: it is a temporary status assigned when WiFi.begin() is called and remains active until the number of attempts expires (resulting in WL_CONNECT_FAILED) or a connection is established (resulting in WL_CONNECTED);
  //  WL_NO_SSID_AVAIL: assigned when no SSID are available;
  //  WL_SCAN_COMPLETED: assigned when the scan networks is completed;
  //  WL_CONNECT_FAILED: assigned when the connection fails for all the attempts;
  //  WL_CONNECTION_LOST: assigned when the connection is lost;
  //  WL_DISCONNECTED: assigned when disconnected from a network;

  // wifi_status = WiFi.status();
  Serial.print(F("wifi_status: "));
  Serial.println(WiFi.status());

  // Print the SSID of the network you're attached to:
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address:
  Serial.print(F("IP Address: "));
  Serial.println(WiFi.localIP());

  // Print your WiFi shield's MAC address:
  Serial.print(F("MAC Adress: "));
  Serial.println(WiFi.macAddress());

  // Print the received signal strength:
  Serial.print(F("Signal strength (RSSI):"));

  Serial.print(WiFi.RSSI());

  Serial.println(F(" dBm"));
}

void Connect_WiFi()
{ // Connect to WiFi

  WiFi.disconnect(true); // disconnect from wifi to set new wifi connection
  WiFi.mode(WIFI_STA);   // init wifi mode

  //  If there are not wifi identity and wifi password defined, proceed to traight forward configuration

    String wifi_ssid = WiFi.SSID(); // your network SSID (name)
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
    Print_WiFi_Status_ESP8266();
}

void Check_WiFi_Server()                       // Server access by http when you put the ip address in a web browser !!!!!!!!!!!!!!!!!!!!!!!!!!!
{                                              // Wifi server
  WiFiClient client = wifi_server.available(); // listen for incoming clients
  if (client)
  {                                  // if you get a client,
    Serial.println(F("new client")); // print a message out the serial port
    String currentLine = "";         // make a String to hold incoming data from the client
    while (client.connected())
    { // loop while the client's connected
      if (client.available())
      {                         // if there's bytes to read from the client,
        char c = client.read(); // read a byte, then
        Serial.write(c);        // print it out the serial monitor
        if (c == '\n')
        { // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0)
          {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            // Print current info
            client.print("Medidor AireCiudadano");
            client.println("<br>");
            client.print("SW version: ");
            client.print(sw_version);
            client.println("<br>");
            client.println("------");
            client.println("<br>");
            client.print("AireCiudadano Device ID: ");
            client.print(aireciudadano_device_id);
            client.println("<br>");
            client.print("AireCiudadano custom name: ");
            client.print(eepromConfig.aireciudadano_device_name);
            client.println("<br>");
            client.print("SSID: ");
            client.print(String(WiFi.SSID()));
            client.println("<br>");
            client.print("IP Adress: ");
            client.print(WiFi.localIP());
            client.println("<br>");
            client.print("MAC Adress: ");
            client.print(WiFi.macAddress());
            client.println("<br>");
            client.print("RSSI: ");
            client.print(WiFi.RSSI());
            client.println("<br>");
            client.println("------");
            client.println("<br>");
            client.print("Publication Time: ");
            client.println(eepromConfig.PublicTime);
            client.println("<br>");
            client.print("MQTT Server: ");
            client.print("sensor.aireciudadano.com");
            client.println("<br>");
            client.print("MQTT Port: ");
            client.print("80");
            client.println("<br>");
            client.print("Sensor latitude: ");
            client.print(eepromConfig.sensor_lat);
            client.println("<br>");
            client.print("Sensor longitude: ");
            client.print(eepromConfig.sensor_lon);
            client.println("<br>");
            client.println("------");
            client.println("<br>");
            client.print("PM2.5: ");
            client.print(PM25_value); //!!!!!!!!!!!!!!!!!!!!!!!!!!!
            client.println("<br>");
            client.print("Temperature: ");
            client.print(temp);
            client.println("<br>");
            client.print("Humidity: ");
            client.print(humi);
            client.println("<br>");
            client.println("------");
            client.println("<br>");

            // Captive portal:
            client.print("Click <a href=\"/3\">here</a> to launch captive portal to set up WiFi and sensor configuration.<br>");
            client.println("<br>");
            // Suspend:
            // client.print("Click <a href=\"/4\">here</a> to suspend the device.<br>");
            client.print("Click <a href=\"/4\">here</a> to Firmware update.<br>");
            client.println("<br>");
            // Restart:
            client.print("Click <a href=\"/5\">here</a> to restart the device.<br>");
            client.println("<br>");

            // The HTTP response ends with another blank line:
            client.println();

            // break out of the while loop:
            break;
          }
          else
          { // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r')
        {                   // if you got anything else but a carriage return character,
          currentLine += c; // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /1" to calibrate the sensor:

        // Check to see if the client request was "GET /3" to launch captive portal:
        if (currentLine.endsWith("GET /3"))
        {
          PortalFlag = true;
#if !Rosver3
          Start_Captive_Portal();
#endif
        }
        if (currentLine.endsWith("GET /4"))
        {
        }
        // Check to see if the client request was "GET /5" to restart the device:
        if (currentLine.endsWith("GET /5"))
        {
          ESP.restart();
        }
      }
    }

    // close the connection:
    client.stop();
    Serial.println(F("client disconnected"));
  }
}

void Start_Captive_Portal()
{ // Run a captive portal to configure WiFi and MQTT
  InCaptivePortal = true;
  String wifiAP;
  int captiveportaltime = 0;

  Serial.println(F("Start_Captive_Portal"));

//    captiveportaltime = 60;
    captiveportaltime = 25;

  wifiAP = aireciudadano_device_id;
  Serial.println(wifiAP);

  wifi_server.stop();

  wifiManager.setDebugOutput(true);
  wifiManager.disconnect();

  WiFi.mode(WIFI_AP); // explicitly set mode, esp defaults to STA+AP

  // Captive portal parameters

  WiFiManagerParameter custom_wifi_html("<p>Set WPA2 Enterprise</p>"); // only custom html
  WiFiManagerParameter custom_wifi_user("User", "WPA2 Enterprise identity", eepromConfig.wifi_user, 24);
  WiFiManagerParameter custom_wpa2_pass;
  WiFiManagerParameter custom_wifi_html2("<hr><br/>"); // only custom html

  WiFiManagerParameter custom_id_name("CustomName", "Set Station Name (25 characters max):", eepromConfig.aireciudadano_device_name, 25);

  WiFiManagerParameter custom_sensor_latitude("Latitude", "Latitude (5-4 dec digits are enough)", eepromConfig.sensor_lat, 10);
  WiFiManagerParameter custom_sensor_longitude("Longitude", "Longitude (5-4 dec)", eepromConfig.sensor_lon, 10);
  WiFiManagerParameter custom_outin_type;
  WiFiManagerParameter custom_endhtml("<p></p>"); // only custom html
  WiFiManagerParameter custom_endhtmlup2("<hr>"); // only custom html


  const char *custom_wpa2_pw = "<label for='p'>Password</label><input id='p' name='p' maxlength='64' type='password' placeholder='{p}'><input type='checkbox' onclick='f()'> Show Password";
  new (&custom_wpa2_pass) WiFiManagerParameter(custom_wpa2_pw); // REVISAR!!!!!!!!!!!

#if !Rosver2

  // Sensor Location menu

  if (eepromConfig.ConfigValues[3] == '0')
  {
    const char *custom_outin_str = "<br/><br/><label for='customOutIn'>Location:</label><br/><input type='radio' name='customOutIn' value='1'> Indoors - sensor measures indoors air<br><input type='radio' name='customOutIn' value='0' checked> Outdoors - sensor measures outdoors air";
    new (&custom_outin_type) WiFiManagerParameter(custom_outin_str);
  }
  else if (eepromConfig.ConfigValues[3] == '1')
  {
    const char *custom_outin_str = "<br/><br/><label for='customOutIn'>Location:</label><br/><input type='radio' name='customOutIn' value='1' checked> Indoors - sensor measures indoors air<br><input type='radio' name='customOutIn' value='0'> Outdoors - sensor measures outdoors air";
    new (&custom_outin_type) WiFiManagerParameter(custom_outin_str);
  }

#endif

  // Add parameters

  wifiManager.addParameter(&custom_wifi_user);
  wifiManager.addParameter(&custom_wpa2_pass);
#if !Rosver2
  wifiManager.addParameter(&custom_wifi_html2);
#endif

#if !Rosver2

  wifiManager.addParameter(&custom_id_name);
  wifiManager.addParameter(&custom_sensor_latitude);
  wifiManager.addParameter(&custom_sensor_longitude);
  wifiManager.addParameter(&custom_outin_type);
#endif
  wifiManager.addParameter(&custom_endhtml);

  wifiManager.setSaveParamsCallback(saveParamCallback);

  wifiManager.setConfigPortalTimeout(captiveportaltime);

  const char *menu[] = {"wifi", "wifinoscan", "info", "exit", "sep", "update"};
  wifiManager.setMenu(menu, 6);

  // it starts an access point
  // and goes into a blocking loop awaiting configuration
  // wifiManager.resetSettings(); // reset previous configurations
  ConfigPortalSave = false;

  bool res = wifiManager.startConfigPortal(wifiAP.c_str());
  if (!res)
  {
    Serial.println(F("Not able to start captive portal"));
  }
  else
  {
    // if you get here you have connected to the WiFi
    Serial.println(F("Captive portal operative"));
  }

  // Save parameters to EEPROM only if any of them changed

  if (ConfigPortalSave == true)
  {
    ConfigPortalSave = false;

    strncpy(eepromConfig.wifi_user, custom_wifi_user.getValue(), sizeof(eepromConfig.wifi_user));
    eepromConfig.wifi_user[sizeof(eepromConfig.wifi_user) - 1] = '\0';
    Serial.println(F("Wifi Identity write_eeprom = true"));

    strncpy(eepromConfig.wifi_password, wifi_passwpa2, sizeof(eepromConfig.wifi_password));
    eepromConfig.wifi_password[sizeof(eepromConfig.wifi_password) - 1] = '\0';
    Serial.println(F("Wifi pass write_eeprom = true"));

#if !Rosver2

    strncpy(eepromConfig.aireciudadano_device_name, custom_id_name.getValue(), sizeof(eepromConfig.aireciudadano_device_name));
    eepromConfig.aireciudadano_device_name[sizeof(eepromConfig.aireciudadano_device_name) - 1] = '\0';
    Serial.println(F("Devname write_eeprom = true"));

    strncpy(eepromConfig.sensor_lat, custom_sensor_latitude.getValue(), sizeof(eepromConfig.sensor_lat));
    eepromConfig.sensor_lat[sizeof(eepromConfig.sensor_lat) - 1] = '\0';
    Serial.println(F("Lat write_eeprom = true"));
    latitudef = atof(eepromConfig.sensor_lat); // Cambiar de string a float

    strncpy(eepromConfig.sensor_lon, custom_sensor_longitude.getValue(), sizeof(eepromConfig.sensor_lon));
    eepromConfig.sensor_lon[sizeof(eepromConfig.sensor_lon) - 1] = '\0';
    Serial.println(F("Lon write_eeprom = true"));
    longitudef = atof(eepromConfig.sensor_lon); // Cambiar de string a float

#endif

    CustomValTotalString[9] = {0};
    sprintf(CustomValTotalString, "%8d", CustomValtotal);
    if (CustomValTotalString[0] == ' ')
      CustomValTotalString[0] = '0';
    if (CustomValTotalString[1] == ' ')
      CustomValTotalString[1] = '0';
    if (CustomValTotalString[2] == ' ')
      CustomValTotalString[2] = '0';
    if (CustomValTotalString[3] == ' ')
      CustomValTotalString[3] = '0';
    if (CustomValTotalString[4] == ' ')
      CustomValTotalString[4] = '0';
    if (CustomValTotalString[5] == ' ')
      CustomValTotalString[5] = '0';
    if (CustomValTotalString[6] == ' ')
      CustomValTotalString[6] = '0';
    if (CustomValTotalString[7] == ' ')
      CustomValTotalString[7] = '0';
    if (CustomValTotalString[8] == ' ')
      CustomValTotalString[8] = '0';

    Serial.print(F("CustomValTotalString: "));
    Serial.println(CustomValTotalString);

    if (CustomValtotal == 0)
    {
      Serial.println(F("No configuration sensor values ​​chosen, no changes will be stored"));
    }
    else
    {
      strncpy(eepromConfig.ConfigValues, CustomValTotalString, sizeof(eepromConfig.ConfigValues));
      eepromConfig.ConfigValues[sizeof(eepromConfig.ConfigValues) - 1] = '\0';
      Serial.println(F("CustomVal write_eeprom = true"));
    }
    Write_EEPROM();
    Serial.println(F("write_eeprom = true Final"));
  }
  ESP.restart(); // REVISAR!!!!!!!!!!!!!
}

String getParam(String name)
{
  // read parameter from server, for custom hmtl input
  String value;

  if (wifiManager.server->hasArg(name))
  {
    value = wifiManager.server->arg(name);
  }
  CustomValue = atoi(value.c_str());
  return value;
}

String getParamstring(String name)
{
  // read parameter from server, for customhmtl input
  String value = "";
  if (wifiManager.server->hasArg(name))
  {
    value = wifiManager.server->arg(name);
  }
  return value;
}

void saveParamCallback()
{
  Serial.println(F("[CALLBACK] saveParamCallback fired"));
  Serial.println("Value customSenPM = " + getParam("customSenPM"));
  CustomValtotal = CustomValue;
  Serial.println("Value customSenHYT = " + getParam("customSenHYT"));
  CustomValtotal = CustomValtotal + (CustomValue * 10);
  Serial.println("Value customDisplay = " + getParam("customDisplay"));
  CustomValtotal = CustomValtotal + (CustomValue * 100);
  Serial.println("Value customSD = " + getParam("customSD"));
  CustomValtotal = CustomValtotal + (CustomValue * 1000);
  Serial.println("Value customOutIn = " + getParam("customOutIn"));
  CustomValtotal = CustomValtotal + (CustomValue * 10000);
  Serial.print(F("CustomValtotal: "));
  Serial.println(CustomValtotal);
  strncpy(wifi_passwpa2, getParam("p").c_str(), sizeof(wifi_passwpa2)); // REVISAR!!!!!!!!!!!

    Serial.println("No SD & RTC");
  ConfigPortalSave = true;
}

void Init_MQTT()
{ // MQTT Init function
  Serial.print(F("Attempting to connect to the MQTT broker "));
  Serial.print(F("sensor.aireciudadano.com"));
  Serial.print(F(":"));
  Serial.println(F("80"));

  MQTT_client.setBufferSize(1024);

  MQTT_client.setServer("sensor.aireciudadano.com", 80);
  MQTT_client.setCallback(Receive_Message_Cloud_App_MQTT);
  MQTT_client.connect(aireciudadano_device_id.c_str());

  if (!MQTT_client.connected())
  {
    err_MQTT = true;
    MQTT_Reconnect();
  }
  else
  {
    err_MQTT = false;
    lastReconnectAttempt = 0;
    // Once connected resubscribe
    MQTT_client.subscribe(MQTT_receive_topic.c_str());
    Serial.print(F("MQTT connected - Receive topic: "));
    Serial.println(MQTT_receive_topic);
  }
}

void MQTT_Reconnect()
{ // MQTT reconnect function
  // Try to reconnect only if it has been more than 5 sec since last attemp
  unsigned long now = millis();
  if (now - lastReconnectAttempt > 5000)
  {
    lastReconnectAttempt = now;
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (MQTT_client.connect(aireciudadano_device_id.c_str()))
    {
      err_MQTT = false;
      Serial.println(F("MQTT connected"));
      lastReconnectAttempt = 0;
      // Once connected resubscribe
      MQTT_client.subscribe(MQTT_receive_topic.c_str());
      Serial.print(F("MQTT connected - Receive topic: "));
      Serial.println(MQTT_receive_topic);
    }
    else
    {
      err_MQTT = true;
      Serial.print(F("failed, rc="));
      Serial.print(MQTT_client.state());
      Serial.println(F(" try again in 5 seconds"));
    }
  }
}

void Send_Message_Cloud_App_MQTT()
{ // Send measurements to the cloud application by MQTT
  // Print info
  float pm25f;
  float pm25fori;
  int8_t RSSI;
  int8_t inout;

  Serial.print(F("Sending MQTT message to the send topic: "));
  Serial.println(MQTT_send_topic);
  pm25f = PM25_accumulated / PM25_samples;
  pm25int = round(pm25f);
  pm25fori = PM25_accumulated_ori / PM25_samples;
  pm25intori = round(pm25fori);
  ReadHyT();

  RSSI = WiFi.RSSI();

  Serial.print(F("Signal strength (RSSI):"));
  Serial.print(RSSI);
  Serial.println(F(" dBm"));

    if (AmbInOutdoors)
      inout = 1;
    else
      inout = 0;

    if (AdjPMS == true)
    sprintf(MQTT_message, "{id: %s, PM25: %d, PM25raw: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId);

    else
     sprintf(MQTT_message, "{id: %s, PM25: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d}", aireciudadano_device_id.c_str(), pm25int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId);

  Serial.print(MQTT_message);
  Serial.println();

  // send message, the Print interface can be used to set the message contents

  MQTT_client.publish(MQTT_send_topic.c_str(), MQTT_message);

  digitalWrite(LEDPIN, LOW);
  FlagLED = true;
}

void Receive_Message_Cloud_App_MQTT(char *topic, byte *payload, unsigned int length)
{                               // callback function to receive configuration messages from the cloud application by MQTT
  boolean write_eeprom = false; // to track if writing the eeprom is required
  uint16_t tempcustom = 0;
  uint16_t CustomValtotal2 = 0;
  memcpy(received_payload, payload, length);
  Serial.print(F("Message arrived: "));
  Serial.println(received_payload);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(jsonBuffer, received_payload);

  // Test if parsing succeeds.
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Update name
  if (jsonBuffer["name"] != "")
  {
    strncpy(eepromConfig.aireciudadano_device_name, jsonBuffer["name"].as<const char *>(), sizeof(eepromConfig.aireciudadano_device_name));
    eepromConfig.aireciudadano_device_name[sizeof(eepromConfig.aireciudadano_device_name) - 1] = '\0';
    Serial.print(F("AireCiudadano custom name (json buffer): "));
    Serial.println(eepromConfig.aireciudadano_device_name);
    write_eeprom = true;
  }

  // Publication Time

  tempcustom = uint16_t(jsonBuffer["warning"]);

  if (tempcustom != 0)
  {
    eepromConfig.PublicTime = (uint16_t)jsonBuffer["warning"];
    Serial.print(F("PublicTime write_eeprom = true, value: "));
    Serial.println((uint16_t)jsonBuffer["warning"]);
    write_eeprom = true;
  }

  // Latitude

  latitudef = atof(jsonBuffer["caution"]);
  if (latitudef != 0)
  {
    strncpy(eepromConfig.sensor_lat, jsonBuffer["caution"], sizeof(eepromConfig.sensor_lat));
    eepromConfig.sensor_lat[sizeof(eepromConfig.sensor_lat) - 1] = '\0';
    Serial.print(F("Lat write_eeprom = true, value: "));
    Serial.println(eepromConfig.sensor_lat);
    write_eeprom = true;
  }

  // Longitude

  longitudef = atof(jsonBuffer["temperature_offset"]);
  if (longitudef != 0)
  {
    strncpy(eepromConfig.sensor_lon, jsonBuffer["temperature_offset"], sizeof(eepromConfig.sensor_lon));
    eepromConfig.sensor_lon[sizeof(eepromConfig.sensor_lon) - 1] = '\0';
    Serial.print(F("Lon write_eeprom = true, value: "));
    Serial.println(eepromConfig.sensor_lon);
    write_eeprom = true;
  }

  // CustomSenPM

    CustomValtotal2 = ((int)(eepromConfig.ConfigValues[7]) - 48);

    // CustomSenHYT

    CustomValtotal2 = CustomValtotal2 + ((int)eepromConfig.ConfigValues[6] - 48) * 10;

  // CustomOutIn

  tempcustom = ((uint16_t)jsonBuffer["MQTT_port"]);

  if (tempcustom != 0)
  {
    tempcustom = tempcustom - 1;
    Serial.print("Value customSenOutIn = ");
    Serial.println(tempcustom);
    CustomValtotal2 = CustomValtotal2 + (tempcustom * 10000);
  }
  else
    CustomValtotal2 = CustomValtotal2 + ((int)eepromConfig.ConfigValues[3] - 48) * 10000;

  CustomValTotalString[9] = {0};
  sprintf(CustomValTotalString, "%8d", CustomValtotal2);
  if (CustomValTotalString[0] == ' ')
    CustomValTotalString[0] = '0';
  if (CustomValTotalString[1] == ' ')
    CustomValTotalString[1] = '0';
  if (CustomValTotalString[2] == ' ')
    CustomValTotalString[2] = '0';
  if (CustomValTotalString[3] == ' ')
    CustomValTotalString[3] = '0';
  if (CustomValTotalString[4] == ' ')
    CustomValTotalString[4] = '0';
  if (CustomValTotalString[5] == ' ')
    CustomValTotalString[5] = '0';
  if (CustomValTotalString[6] == ' ')
    CustomValTotalString[6] = '0';
  if (CustomValTotalString[7] == ' ')
    CustomValTotalString[7] = '0';
  if (CustomValTotalString[8] == ' ')
    CustomValTotalString[8] = '0';

  Serial.print(F("CustomValTotalString: "));
  Serial.println(CustomValTotalString);

  if (CustomValtotal2 == 0)
  {
    Serial.println(F("No configuration sensor values ​​chosen, no changes will be stored"));
  }
  else
  {
    strncpy(eepromConfig.ConfigValues, CustomValTotalString, sizeof(eepromConfig.ConfigValues));
    eepromConfig.ConfigValues[sizeof(eepromConfig.ConfigValues) - 1] = '\0';
    write_eeprom = true;
    Serial.println(F("CustomVal write_eeprom = true"));
    Serial.print(F("Configuration Values: "));
    Serial.println(eepromConfig.ConfigValues);
  }

  Aireciudadano_Characteristics();

  // print info
  Serial.println(F("MQTT update - message processed"));
  //  Print_Config();

  // if update flag has been enabled, update to latest bin
  // It has to be the last option, to allow to save EEPROM if required
  if (((jsonBuffer["update"]) && (jsonBuffer["update"] == "ON")))
  {
    // Update firmware to latest bin
    Serial.println(F("Update firmware to latest bin"));
    Firmware_Update();
  }

  // If factory reset has been enabled, just do it
  if ((jsonBuffer["factory_reset"]) && (jsonBuffer["factory_reset"] == "ON"))
  {
    Wipe_EEPROM(); // Wipe EEPROM
    ESP.restart();
  }

  // save the new values if the flag was set
  if (write_eeprom)
  {
    Serial.println(F("write_eeprom = true Final"));
    Write_EEPROM();
    ESP.restart();
  }

  // If reboot, just do it, without cleaning the EEPROM
  //  if ((jsonBuffer["reboot"]) && (jsonBuffer["reboot"] == "ON"))
  //    ESP.restart();
}

void Firmware_Update()
{
  // ESP8266 Firmware Update

  // For remote firmware update
  BearSSL::WiFiClientSecure UpdateClient;
  int freeheap = ESP.getFreeHeap();

  Serial.println(F("### FIRMWARE UPGRADE ###"));

  ESPhttpUpdate.setLedPin(LEDPIN, LOW);

  // Add optional callback notifiers
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);
  UpdateClient.setInsecure();

  // Try to set a smaller buffer size for BearSSL update
  bool mfln = UpdateClient.probeMaxFragmentLength("raw.githubusercontent.com", 443, 512);
  Serial.printf("\nConnecting to https://raw.githubusercontent.com\n");
  Serial.printf("MFLN supported: %s\n", mfln ? "yes" : "no");
  if (mfln)
  {
    UpdateClient.setBufferSizes(512, 512);
  }
  UpdateClient.connect("raw.githubusercontent.com", 443);
  if (UpdateClient.connected())
  {
    Serial.printf("MFLN status: %s\n", UpdateClient.getMFLNStatus() ? "true" : "false");
    Serial.printf("Memory used: %d\n", freeheap - ESP.getFreeHeap());
    freeheap -= ESP.getFreeHeap();
  }
  else
  {
    Serial.printf("Unable to connect\n");
  }

  Serial.println("Firmware ESP8266WISP_WPA2Rosver");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/ESP8266WISP_WPA2Rosver.bin");

  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println(F("HTTP_UPDATE_NO_UPDATES"));
    break;

  case HTTP_UPDATE_OK:
    Serial.println(F("HTTP_UPDATE_OK"));
    break;
  }
}

void update_started()
{
  Serial.println(F("CALLBACK:  HTTP update process started"));
  updating = true;
}

void update_finished()
{
  Serial.println(F("CALLBACK:  HTTP update process finished"));
  Serial.println(F("### FIRMWARE UPGRADE COMPLETED - REBOOT ###"));
  updating = false;
}

void update_progress(int cur, int total)
{
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err)
{
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
  updating = false;
}

void Test_Sensor()
{
  Serial.println(F("Test Plantower Sensor"));

  pmsSerial.begin(9600); // Software serial begin for PMS sensor
  delay(100);

  if (pms.readUntil(data))
  {
    Serial.println(F("Test Plantower sensor found!"));
    digitalWrite(LEDPIN, LOW); // turn the LED on by making the voltage LOW
    delay(400);                // wait for a 400 ms
    digitalWrite(LEDPIN, HIGH);
    delay(400);
    digitalWrite(LEDPIN, LOW);
    delay(400);
    digitalWrite(LEDPIN, HIGH);
    delay(400);
    digitalWrite(LEDPIN, LOW);
    delay(400);
    digitalWrite(LEDPIN, HIGH);
  }
  else
  {
    Serial.println(F("Could not find Plantower sensor!"));
  }

  Serial.print(F("Test SHT4x: "));
  if (!sht4.begin())
  { // Set to 0x45 for alternate i2c addr
    Serial.println(F("none"));
  }
  else
  {
    Serial.println(F("OK"));
    delay(200);
    digitalWrite(LEDPIN, LOW); // turn the LED on by making the voltage LOW
    delay(200);                // wait for a 200 ms
    digitalWrite(LEDPIN, HIGH);
    delay(200);
    digitalWrite(LEDPIN, LOW);
    delay(200);
    digitalWrite(LEDPIN, HIGH);
    delay(200);
    digitalWrite(LEDPIN, LOW);
    delay(200);
    digitalWrite(LEDPIN, HIGH);
  }
}

void Setup_Sensor()
{ // Identify and initialize PM25, temperature and humidity sensor

// PMS7003 PMSA003
    Serial.println(F("Test Plantower Sensor"));

  pmsSerial.begin(9600); // Software serial begin for PMS sensor

    delay(1000);

    if (pms.readUntil(data))
    {
      Serial.println(F("Plantower sensor found!"));
      PMSsen = true;
      AdjPMS = true; // Por defecto se deja con ajuste, REVISAR!!!!!!
    }
    else
    {
      Serial.println(F("Could not find Plantower sensor!"));
    }

    Serial.print(F("SHT4x test: "));
    if (!sht4.begin())
    { // Set to 0x45 for alternate i2c addr
      Serial.println(F("none"));
    }
    else
    {
      Serial.println(F("OK"));
      SHT31sen = true;
    }

    sht4.setPrecision(SHT4X_MED_PRECISION);
    Serial.println(F("SHT4x Med precision"));
    sht4.setHeater(SHT4X_NO_HEATER);
    Serial.println(F("SHT4x No heater"));
}

void Read_Sensor()
{ // Read PM25, temperature and humidity values

    if (pms.readUntil(data))
    {
      PM25_value = data.PM_AE_UG_2_5;
      Serial.print(F("PMS PM2.5: "));
      Serial.print(PM25_value);
      Serial.print(F(" ug/m3   "));
      if (AdjPMS == true)
      {
        PM25_value_ori = PM25_value;
        // PM25_value = ((562 * PM25_value_ori) / 1000) - 1; // Ecuación de ajuste resultado de 13 intercomparaciones entre PMS7003 y SPS30 por meses
        // PM25_value = ((553 * PM25_value_ori) / 1000) + 1.3; // Segundo ajuste
        PM25_value = ((630 * PM25_value_ori) / 1000) + 1.56; // Tercer ajuste a los que salio en Lima y pruebas aqui
        Serial.print(F("Adjust: "));
        Serial.print(PM25_value);
        Serial.println(F(" ug/m3"));
      }
      else
        Serial.println(F(""));
    }
    else
    {
      Serial.println(F("PM25 dummy  = 10"));                      // TEST PMS7003 dummy
      PM25_value = 10;                                            // TEST PMS7003 dummy
    }
}

void ReadHyT()
{
  //  SHT31 or SHT4x
  if (SHT31sen == true)
  {
    temperature = 0.0;
    humidity = 0.0;

  sensors_event_t humidity2, temp2;
  sht4.getEvent(&humidity2, &temp2);// populate temp and humidity objects with fresh data
  humidity = humidity2.relative_humidity;
  temperature = temp2.temperature;

    if (!isnan(humidity))
    { // check if 'is not a number'
      failh = 0;
      Serial.print(F("SHT4x Humi % = "));
      Serial.print(humidity);
      humi = round(humidity);
    }
    else
    {
      Serial.println(F("Failed to read humidity SHT4x"));
      humi = 255;
      if (failh == 5)
      {
        failh = 0;
        sht4.begin();
      }
      else
        failh = failh + 1;
    }

    if (!isnan(temperature))
    { // check if 'is not a number'
      Serial.print(F("   Temp *C = "));
      Serial.println(temperature);
      temp = round(temperature);
    }
    else
    {
      Serial.println(F("   Failed to read temperature SHT4x"));
      temp = 255;
    }
  }

}

///////////////////////////////////////////////////////////////////////////////

void Print_Config()
{ // print AireCiudadano device settings

  Serial.println(F("#######################################"));
  Serial.print(F("Device id: "));
  Serial.println(aireciudadano_device_id);
  Serial.print(F("AireCiudadano custom name: "));
  Serial.println(eepromConfig.aireciudadano_device_name);
  Serial.print(F("SW version: "));
  Serial.println(sw_version);
  Serial.print(F("Publication Time: "));
  Serial.println(eepromConfig.PublicTime);
  Serial.print(F("Sensor latitude: "));
  Serial.println(eepromConfig.sensor_lat);
  Serial.print(F("Sensor longitude: "));
  Serial.println(eepromConfig.sensor_lon);
  Serial.print(F("Configuration values: "));
  Serial.println(eepromConfig.ConfigValues);
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());
  Serial.print(F("WiFi Identity for WPA enterprise: "));
  Serial.println(eepromConfig.wifi_user);
  Serial.print(F("WiFi identity's password for WPA enterprise: "));
  Serial.println(eepromConfig.wifi_password);
  Serial.println(F("#######################################"));
}

void Get_AireCiudadano_DeviceId()
{ // Get TTGO T-Display info and fill up aireciudadano_device_id with last 6 digits (in HEX) of WiFi mac address or Custom_Name + 6 digits
  //  uint32_t chipId = 0;
  char aireciudadano_device_id_endframe[10];

  chipId = ESP.getChipId();
  chipIdHEX = String(ESP.getChipId(), HEX);
  strncpy(aireciudadano_device_id_endframe, chipIdHEX.c_str(), sizeof(aireciudadano_device_id_endframe));
#if !Rosver
  Aireciudadano_Characteristics();
#endif
  Serial.print(F("ESP8266 Chip ID = "));
  Serial.print(chipIdHEX);
  Serial.print(F(", ESP CoreVersion: "));
  Serial.println(ESP.getCoreVersion());

  Serial.print(F("AireCiudadano Device ID: "));
  if (String(aireciudadano_device_id).isEmpty())
    aireciudadano_device_id = String("AireCiudadano_") + aireciudadano_device_id_endframe;
  else
    aireciudadano_device_id = String(eepromConfig.aireciudadano_device_name) + "_" + aireciudadano_device_id_endframe;
  Serial.println(aireciudadano_device_id);
}

void Aireciudadano_Characteristics()
{

  Serial.print(F("eepromConfig.ConfigValues: "));
  Serial.println(eepromConfig.ConfigValues);
  Serial.print(F("eepromConfig.ConfigValues[3]: "));
  Serial.println(eepromConfig.ConfigValues[3]);
  if (eepromConfig.ConfigValues[3] == '0')
  {
    AmbInOutdoors = false;
    Serial.println(F("Outdoors"));
  }
  else
  {
    AmbInOutdoors = true;
    Serial.println(F("Indoors"));
  }

  if (AdjPMS == true)
    Serial.println(F("PMS sensor with stadistical adjust"));
  else if (PMSsen == true)
    Serial.println(F("PMS sensor"));
  else
    Serial.println(F("No PMS sensor"));

  if (SHT31sen == true)
    Serial.println(F("SHT4x sensor"));
  else
    Serial.println("No SHT4x sensor");

  Serial.println(F("WPA2 security"));

  // SPS30sen = 1
  // SEN5Xsen = 2
  // PMSsen = 4
  // AdjPMS = 8
  // SHT31sen = 16
  // AM2320sen =32
  // SDflag = 64
  // SPmeter = 128
  // TDisplay = 256
  // OLED66 = 512
  // OLED96 = 1024
  //        = 2048
  // AmbInOutdoors (Indoors) = 4096
  // WPA2 = 8192
  // ESP8266 = 16384
  // Rosver = 32768
  // Swver * 65536

  if (PMSsen)
    IDn = IDn + 4;
  if (AdjPMS)
    IDn = IDn + 8;
  if (SHT31sen)
    IDn = IDn + 16;
  if (AmbInOutdoors)
    IDn = IDn + 4096;
// WPA2
    IDn = IDn + 8192; 
// ESP8266
  IDn = IDn + 16384;
// Rosver
  IDn = IDn + 32768;
  IDn = IDn + (Swver * 65536);
  Serial.print(F("IDn: "));
  Serial.println(IDn);
}

void Read_EEPROM()
{
  // The begin() call will find the data previously saved in EEPROM if the same size
  // as was previously committed. If the size is different then the EEEPROM data is cleared.
  // Note that this is not made permanent until you call commit();
  EEPROM.begin(sizeof(MyConfigStruct));
  // Check if the EEPROM contains valid data from another run
  // If so, overwrite the 'default' values set up in our struct
  if (EEPROM.percentUsed() >= 0)
  {
    Serial.println(F("EEPROM has data from a previous run."));
    Serial.print(EEPROM.percentUsed());
    Serial.println(F("% of ESP flash space currently used"));

    // Read saved data
    EEPROM.get(0, eepromConfig);
    //    Print_Config();
  }
  else
  {
    aireciudadano_device_id.toCharArray(eepromConfig.aireciudadano_device_name, sizeof(eepromConfig.aireciudadano_device_name)); // Initialize aireciudadano_device_name with aireciudadano_device_id
    Serial.println(F("No EEPROM data - using default config values"));
  }
}

void Write_EEPROM()
{
  // The begin() call will find the data previously saved in EEPROM if the same size
  // as was previously committed. If the size is different then the EEEPROM data is cleared.
  // Note that this is not made permanent until you call commit();
  EEPROM.begin(sizeof(MyConfigStruct));

  // set the EEPROM data ready for writing
  EEPROM.put(0, eepromConfig);

  // write the data to EEPROM
  boolean ok = EEPROM.commit();
  Serial.println((ok) ? "EEPROM Commit OK" : "EEPROM Commit failed");
}

void Wipe_EEPROM()
{ // Wipe AireCiudadano device persistent info to reset config data
  boolean result = EEPROM.wipe();
  if (result)
  {
    Serial.println(F("All EEPROM data wiped"));
  }
  else
  {
    Serial.println(F("EEPROM data could not be wiped from flash store"));
  }
}