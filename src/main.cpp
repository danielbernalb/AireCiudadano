//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AireCiudadano medidor Fijo - Medidor de PM2.5 abierto, medición opcional de humedad y temperatura.
// Más información en: aireciudadano.com
// Este firmware es un fork del proyecto Anaire (https://www.anaire.org/) recomendado para la medición de CO2.
// 21/03/2024 info@aireciudadano.com
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Novedades:
// 1. Reporte del PM1 al servidor
// 2. Posibilidad de uso de dos sensores PMS7003:
// PM25: promedio de la medición de los dos sensores PMS7003
// PM25raw: promedio de las dos medición RAW de los dos sensores
// PM251: valor de la medición (sin ajuste o con ajuste) del sensor 1 PMS7003
// PM251: valor de la medición (sin ajuste o con ajuste) del sensor 2 PMS7003
// 3. SoundMeter integrado con opcion SoundAM (Aeropuerto)
// 4. Probar medicion HyT de PMSx003T con TwoPMS true
// 5. ESP32S3 board
// 6. SHT31 y SHT4X funcionales sin seleccion de cada modelo
// 7. LedNeoPixel multicolor
// 8. Compatabilidad con ESP32C3 AirGrad soble PMS5003T
// 9. SoundMeter Bluetooth
// 10. LTR390 UV sensor
// 11. Ajuste de SPS30 agregado, envia en PM25 el ajuste y en PM25raw el original
// 12. AdjPMS para PMS7003 eliminado, siempre envia ajuste por PM25 y el original por PM25raw
// 13. En Bluetooth cuando se bajan los datos del SEN55 genera reset con todas las librerias
// 14. MinVer es como Rosver pero para cualquier sensor y sin SD
// 15. Json lib actualizada a v7
// 16. ESP8285 flag en platformio.ini
// 17. En Test_Sensor se identifica SEN54 y 55 con HyT y PMS7003T para parpadeo de HyT
// 18. Ver 2.6
// 19. SPS30 ajuste resultado de intercomparacion con SEN y PMS
// 20. MinVer con SD
// 21. Conectividad movil, FlagMobData
// 22. mqtt.loop ser realiza ahora con la duracion  de MQTT_loop_review en milisegundos
// 23. Wifi Power max con flag MaxWifiTX SOLO programada desde web mqtt AireCiudadano: Resultado no concluyente de incremento de cobertura
// 24. ZH10 sensor para ESP32
// 25- SDS011 sensor para ESP8266 y ESP32
// OK: Revisar BUG cuando una simcard no se ha usado y no ingresa rapido a la red el ESP8266 se resetea y no sigue preguntando la red para ingresar, a diferencia del codigo Arduino y que si lo logra.
// OK: MONTiny: fail y no sigue y se reinicia y sale 0:0

// Constantes de Ajuste de sensores programables: pendiente e intercepto. ANALIZAR MAS
// Verificar nueva libreria Bluetooth que parece compatible con Sensirion UPT Core@^0.3.0, Sigue el error con lectura de nox y en algunos modelos es lento
// La rutina Button2 falla pero con https://github.com/LennartHennigs/Button2.git#2.3.3 OK, no se porque el sensor viejo falla en eso

#include <Arduino.h>
#include "main.hpp"

////////////////////////////////
// Modo de comunicaciones del sensor:
#define Wifi true       // Set to true in case Wifi if desired, Bluetooth off and SDyRTCsave optional
#define WPA2 false       // Set to true to WPA2 enterprise networks (IEEE 802.1X)
#define Bluetooth false   // Set to true in case Bluetooth if desired, Wifi off and SDyRTCsave optional
#define SDyRTC false     // Set to true in case SD card and RTC (Real Time clock) if desired, Wifi and Bluetooth off
#define SaveSDyRTC false // Set to true in case SD card and RTC (Real Time clock) if desired to save data in Wifi or Bluetooth mode
#define TwoPMS false     // Set to true if you want 2 PMS7003 sensors
#define SoundMeter false // set to true for Sound Meter
#define SoundAM false    // Set to true to Sound meter airplane mode
#define Influxver false  // Set to true for InfluxDB version

#define ZH10sen true     // Set to true for ZH10 instead PMSX003
#define SDS011sen false  // Set to true for SDS011 instead PMSX003
#define LedNeo false     // Set to true for Led Neo multicolor
#define LTR390UV false
#define NoxVoxTd false

// Seleccion de operador de telefonia movil
#define TigoKalleyExito false
#define MovistarVirgin false
#define Claro false
#define Wom false

#define A7670 false
#define SIM7070 false
#define SIM800 false

// Escoger modelo de pantalla (pasar de false a true) o si no hay escoger ninguna (todas false):
#define Tdisplaydisp false    // TTGO T Display
#define OLED66display false   // Pantalla OLED 0.66"
#define OLED96display false   // Pantalla OLED 0.96"

#define CO2sensor false          // Set to true for CO2 sensors: SCD30 and SenseAir S8
#define SiteAltitude 0         // IMPORTANT for CO2 measurement: Put the site altitude of the measurement, it affects directly the value
//define SiteAltitude 2600   // 2600 meters above sea level: Bogota, Colombia

// Boards diferentes
#define TTGO_TQ false

// Definiciones opcionales para version Wifi
#define BrownoutOFF false   // Colocar en true en boards con problemas de RESET por Brownout o bajo voltaje
#define ESP8266SH false     // Colocar para PMS en pin 0 - Hardware Serial
#define PreProgSensor false // Variables de sensor preprogramadas:
                            // Latitude: char sensor_lat[10] = "xx.xxxx";
                            // Longitude: char sensor_lon[10] = "xx.xxxx";
                            // Valores de configuración: char ConfigValues[9] = "000xxxxx";
                            // Nombre estación: char aireciudadano_device_name[36] = "xxxxxxxxxxxxxx";

#ifdef ESP32S3def
#define ESP32S3 true      // Set to true in case you use an ESP32S3
#else
#define ESP32S3 false     // Set to true in case you use an ESP32S3
#endif

#ifdef ESP32C3AGdef
#define ESP32C3AG true    // Set to true in case you use an ESP32C3 version AirGrad
#else
#define ESP32C3AG false   // Set to true in case you use an ESP32C3 version AirGrad
#endif

#ifdef ESP8285def
#define ESP8285 true      // Set to true in case you use an ESP8285 switch
#else
#define ESP8285 false
#endif

#ifdef Rosverdef
#define Rosver true       // Set to true in case you use an ESP32S3
#else
#define Rosver false      // Set to true in case you use an ESP32S3
#endif

#ifdef MinVerdef
#define MinVer true       // Set to true for a minimum version like Rosver but without SD support and any sensor
#else
#define MinVer false
#endif

#ifdef MobDatadef
#define MobData true      // Opcion para datos mobiles
#else
#define MobData false
#endif

#ifdef MinVerSDdef
#define MinVerSD true     // Version minima con SD
#else
#define MinVerSD false
#endif

#ifdef MobDataSPver
#define MobDataSP true
#else
#define MobDataSP false
#endif

// Fin definiciones opcionales Wifi

bool SPS30sen = false;  // Sensor Sensirion SPS30
bool SEN5Xsen = false;  // Sensor Sensirion SEN5X
bool PMSsen = false;    // Sensor Plantower PMS
bool SHTsen = false;    // Sensor SHT31/SHT4x humedad y temperatura
bool SHT31sen = false;  // Sensor SHT31 humedad y temperatura
bool SHT4xsen = false;  // Sensor SHT4x humedad y temperatura
bool AM2320sen = false; // Sensor AM2320 humedad y temperatura
bool SCD30sen = false;  // Sensor CO2 SCD30 Sensirion
bool S8sen = false;     // Sensor CO2 SenseAir S8
bool TDisplay = false;  // Board TTGO T-Display is used
bool OLED66 = false;    // OLED Diplay 0.66 inch 64x48
bool OLED96 = false;    // OLED Diplay 0.96 inch 128x64
bool AmbInOutdoors = false; // Indoors measuring outside environment, false if is outdoors
bool SDflag = false;
bool FlagMobData = false;
bool FlagSENHyT = false;
bool FlagpmsHyT = false;
bool FlagMQTTcon = false;
bool FlagPoweroff = false;
bool MaxWifiTX = false;
bool FlagAdjustSensor;

uint8_t Contacon = 0;

uint8_t CustomValue = 0;
uint32_t CustomValtotal = 0;
char CustomValTotalString[9] = "00000000";
uint32_t IDn = 0;
String chipIdHEX;

#if Rosver
uint64_t chipId;
#else
uint32_t chipId = 0;
#endif

// device id, automatically filled by concatenating the last three fields of the wifi mac address, removing the ":" in betweeen, in HEX format. Example: ChipId (HEX) = 85e646, ChipId (DEC) = 8775238, macaddress = E0:98:06:85:E6:46
String sw_version = "2.9";
String aireciudadano_device_id;
uint8_t Swver;

// Init to default values; if they have been chaged they will be readed later, on initialization
struct MyConfigStruct
{
#if Bluetooth
#if CO2sensor
  uint16_t BluetoothTime = 10; // Bluetooth Time
#elif (SoundMeter || LTR390UV)
  uint16_t BluetoothTime = 2; // Bluetooth Time
#else
  uint16_t BluetoothTime = 10; // Bluetooth Time
#endif
  char aireciudadano_device_name[30]; // Device name; default to aireciudadano_device_id
#elif Wifi
#if !MobData
  uint16_t PublicTime = 1; // Publication Time
#else
  uint16_t PublicTime = 2; // Publication Time
#endif
  //  uint16_t MQTT_port = 80;                           // MQTT port; Default Port on 80
  //  char MQTT_server[30] = "sensor.aireciudadano.com"; // MQTT server url or public IP address.
#if !PreProgSensor
  char sensor_lat[10] = "0.0"; // Sensor latitude  GPS
  char sensor_lon[10] = "0.0"; // Sensor longitude GPS
  char ConfigValues[10] = "000100000";
  char aireciudadano_device_name[30]; // Device name; default to aireciudadano_device_id
#else
  char sensor_lat[10] = "4.6987";   // Aquí colocar la Latitud del sensor
  char sensor_lon[10] = "-74.0987"; // Colocar la Longitud del sensor
  char ConfigValues[10] = "000100000";
  char aireciudadano_device_name[30] = "AireCiudadano_Test01"; // Nombre de la estacion
#endif
#endif
#if (WPA2 || Rosver)
  char wifi_user[24];     // WiFi user to be used on WPA Enterprise. Default to null (not used)
  char wifi_password[24]; // WiFi password to be used on WPA Enterprise. Default to null (not used)
#endif
} eepromConfig;

char wifi_passwpa2[24];
bool ConfigPortalSave = false;

#if PreProgSensor
const char *ssid = "TPxxx";
const char *password = "aptxxxxx";
char aireciudadano_device_nameTemp[30] = {0};
#endif

#if BrownoutOFF
// OFF BROWNOUT/////////////////////
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

#if ESP8266
// Save config values to EEPROM
#include <ESP_EEPROM.h>
#else
// to store data on nvs partition
#include <Preferences.h>
Preferences preferences;
#endif

// Measurements
float PM25_value = 0;     // PM25 measured value
float PM25_valueold;
float PM251_value = 0;    // PM25 sensor 1 measured value
float PM252_value = 0;    // PM25 sensor 2 measured value
float PM25_value_ori = 0; // PM25 original measured value in PMS adjust TRUE
float PM251_value_ori = 0;
float PM252_value_ori = 0;
float PM25_valuesam = 0;    // PM25 firware SoundAM
float PM25_accumulated = 0; // Accumulates pm25 measurements for a MQTT period
float PM251_accumulated = 0;
float PM252_accumulated = 0;
float PM25_accumulated_ori = 0; // Accumulates pm25 measurements for a MQTT period in PMS Adjust TRUE
float PM251_accumulated_ori = 0;
float PM252_accumulated_ori = 0;
float PM25_accumulatedsam = 0; // Accumulates pm25 measurements for a MQTT period of Sam samples
float PM1_value = 0;           // PM1 measured value
float PM11_value = 0;
float PM12_value = 0;
float PM1_accumulated = 0;
float PM11_accumulated = 0;
float PM12_accumulated = 0;
float temperature;    // Read temperature as Celsius
float humidity;       // Read humidity in %
int PM25_samples = 0; // Counts de number of samples for a MQTT period
int SP_samples = 0;   // Counts de number of samples for a MQTT period
int pm25int;          // PM25 publicado
int pm25intori;
int pm251int;
int pm252int;
int pm251intori;
int pm252intori;
int pm1int;
int pm11int;
int pm12int;
float dBAmax = 0;
float dBAmaxsam = 0;

int temp;
int humi;

float latitudef = 0.0;
float longitudef = 0.0;

bool err_wifi = false;
bool err_MQTT = false;
bool err_sensor = false;
bool FlagDATAicon = false;
bool NoSensor = false;

bool MQTT_toggle = false;
bool MQTT_token = false;

// Measurements loop: time between measurements
unsigned int measurements_loop_duration = 1000; // 1 second
unsigned long measurements_loop_start;          // holds a timestamp for each control loop start

unsigned int Bluetooth_loop_time;
unsigned int Con_loop_times = 0;

unsigned int SDyRTC_loop_time;

// MQTT loop: time between MQTT measurements sent to the cloud
unsigned long MQTT_loop_start; // holds a timestamp for each cloud loop start
unsigned long MQTT_loop_startsam;
unsigned long MQTT_loop_review;
unsigned int MQTT_loop_review_duration = 15000;
unsigned long lastReconnectAttempt = 0; // MQTT reconnections

// Errors loop: time between error condition recovery
unsigned int errors_loop_duration = 60000; // 60 seconds
unsigned long errors_loop_start;           // holds a timestamp for each error loop start

byte cont = 0;

#if !ESP8266
// TTGO ESP32 board
#include "esp_timer.h"
#include <esp_system.h>
#endif
#include <Wire.h>

// OLED display
unsigned int mcount, ecode = 0;
int lastDrawedLine = 0;
unsigned int inthumi = 0;
unsigned int inttemp = 0;
unsigned int cursor = 0;
bool toggleLive;
int dw = 0; // display width
int dh = 0; // display height

#if !ESP8266

#if OLED66display
#include <U8g2lib.h>
#include "Iconos.h"
U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
#elif OLED96display
#include <U8g2lib.h>
#include "Iconos.h"
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, U8X8_PIN_NONE, U8X8_PIN_NONE);
#elif Tdisplaydisp
#include "Iconos.h"
#endif

#else

#if OLED66display
#include <U8g2lib.h>
#include "Iconos.h"
U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 5, 4);
#elif OLED96display
#include <U8g2lib.h>
#include "Iconos.h"
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 5, 4);
#endif

#endif

#if Tdisplaydisp
// Display and fonts
#include <TFT_eSPI.h>
#include <SPI.h>
#include "ArchivoNarrow_Regular50pt7b.h"
#include "ArimoBoldFont16.h"
#include "ArimoBoldFont20.h"
#include "FreeSansBold30pt7b.h"

#define GFXFF 1
#define FF90 &ArimoBoldFont16
#define FF92 &ArimoBoldFont20
#define FF95 &ArchivoNarrow_Regular50pt7b
#define FF97 &FreeSansBold30pt7b

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke library, pins defined in User_Setup.h

// Customized AireCiudadano splash screen
#include "Icono_AireCiudadano.h"

// Buttons: Top and bottom considered when USB connector is positioned on the right of the board

#include "Button2.h"
#define BUTTON_TOP 35
#define BUTTON_BOTTOM 0
Button2 button_top(BUTTON_TOP);
Button2 button_bottom(BUTTON_BOTTOM);

// Define ADC PIN for battery voltage measurement
#define ADC_PIN 34
float battery_voltage;
int vref = 1100;

// Define voltage threshold
#define USB_Voltage 4.0
#define Voltage_Threshold_1 3.9
#define Voltage_Threshold_2 3.7
#define Voltage_Threshold_3 3.5
#define Voltage_Threshold_4 3.3

#else

#define BUTTON_TOP 35   // ??????????
#define BUTTON_BOTTOM 0 // ??????????

#endif

#if !ESP32S3
#define Sensor_SDA_pin 21 // Define the SDA pin used
#define Sensor_SCL_pin 22 // Define the SCL pin used
#else
#define Sensor_SDA_pin 4 // Define the SDA pin used
#define Sensor_SCL_pin 5 // Define the SCL pin used
#endif

byte failpm = 0;

#if !MobDataSP
#if !Rosver

#include <sps30.h>
SPS30 sps30;
#define SP30_COMMS Wire
#define DEBUG 0

#include <SensirionI2CSen5x.h>

// The used commands use up to 48 bytes. On some Arduino's the default buffer space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

SensirionI2CSen5x sen5x;
// bool SEN5Xflag = false;
float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;

#if SDS011sen

#include <SDS011.h>
SDS011 my_sds;
float p10, p25;
int errSDS011;

#if ESP32
HardwareSerial port(2);     // PIN 17 y 16 ESP32
#endif

#if ESP8266
#include <SoftwareSerial.h>
#define SDS_TX 14 // PMS TX pin
#define SDS_RX 12 // PMS RX pin
#endif

#endif

#if ZH10sen     // Funciona para ESP32, falta ESP8266

#define RX_PIN_ZH10 17  // Pin RX del ESP32
#define TX_PIN_ZH10 15  // Pin TX del ESP32
HardwareSerial mySerial(2);
uint8_t command[] = {0xFF, 0x01, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCA}; // Comando para leer datos
uint8_t response[22];
uint8_t checksum = 0;
float temperatureZH10;
float vocZH10;
uint16_t vocsint;
uint16_t pm1_0;
uint16_t pm2_5;
uint16_t pm10ZH10;
uint16_t rawTemp;
uint16_t humiZH10;

#endif

#endif

#include "PMS.h"
#endif

#if !ESP8266

#if !TTGO_TQ

#if !TwoPMS

PMS pms(Serial1);
PMS::DATA data;
// bool PMSflag = false;
#if !Tdisplaydisp
#if !ESP32C3AG
#define PMS_TX 17 // PMS TX pin
#define PMS_RX 16 // PMS RX pin
#else
#include <HardwareSerial.h>
#define PMS_TX 20 // PMS TX pin
#define PMS_RX 21 // PMS RX pin
#endif
#else
#define PMS_TX 17 // PMS TX pin
#define PMS_RX 15 // PMS RX pin
#endif

#else     // esTwoPMS test AirGrad

#include <SoftwareSerial.h>
//#include <HardwareSerial.h>

#define PMS_TX1 20 // PMS TX pin
#define PMS_RX1 21 // PMS RX pin
#define PMS_TX2 0 // PMS TX pin      
#define PMS_RX2 1  // PMS RX pin

SoftwareSerial pmsSerial1(PMS_TX1, PMS_RX1);    //SoftwareSerial
//PMS pms1(Serial0);    // HardwareSerial
PMS pms1(pmsSerial1);   //SoftwareSerial
PMS::DATA data;

SoftwareSerial pmsSerial2(PMS_TX2, PMS_RX2);    //SoftwareSerial
//PMS pms2(Serial1);    // HardwareSerial
PMS pms2(pmsSerial2);   // SoftwareSerial
PMS::DATA data2;

#endif

#else   // Es TTGO_TQ

PMS pms(Serial2);
PMS::DATA data;
// bool PMSflag = false;
#define PMS_TX 19 // PMS TX pin
#define PMS_RX 18 // PMS RX pin

#endif

#else   // es ESP8266

#if !ESP8266SH

#include <SoftwareSerial.h>

#if !ESP8285
// #define PMS_TX 0 // PMS TX pin  --- A veces no programa en ESP8266mini
// #define PMS_TX 2 // PMS TX pin  --- Bien pero conectado al Onboard Led del ESP8266
// #define PMS_TX 16 // PMS TX pin --- No hace nada, no lee
// #define PMS_TX 14 // PMS TX pin --- Bien pero SPI de SD card usa ese pin
#if !(SaveSDyRTC || SDyRTC || Rosver)
#if !TwoPMS
#define PMS_TX 14 // PMS TX pin
#define PMS_RX 16 // PMS RX pin
#else
#if !PurpleVer
#define PMS_TX1 14 // PMS TX pin      // D5
#else
#define PMS_TX1 13 // PMS TX pin      
#endif
#define PMS_RX1 16 // PMS RX pin
#define PMS_TX2 12 // PMS TX pin      // D6
#define PMS_RX2 2  // PMS RX pin
#endif
#else
#define PMS_TX 0  // PMS TX pin
#define PMS_RX 16 // PMS RX pin
#endif
#else
#define PMS_TX 3 // PMS TX pin
#define PMS_RX 2 // PMS RX pin
#endif

#if !(TwoPMS || SoundMeter)
SoftwareSerial pmsSerial(PMS_TX, PMS_RX); // SoftwareSerial(rxPin, txPin)
PMS pms(pmsSerial);
PMS::DATA data;
#elif TwoPMS
SoftwareSerial pmsSerial1(PMS_TX1, PMS_RX1);
PMS pms1(pmsSerial1);
PMS::DATA data;

SoftwareSerial pmsSerial2(PMS_TX2, PMS_RX2);
PMS pms2(pmsSerial2);
PMS::DATA data2;
#endif

#else // ESP8266 Hardware Serial

PMS pms(Serial);
PMS::DATA data;

#endif

#endif

#if SoundMeter
#if !ESP8266
#if !Bluetooth
#if Tdisplaydisp
#define ESP32_RX 27 // ESP_MEMS TX pin
#else
#define ESP32_RX 16 // ESP_MEMS TX pin
#endif
#else
#if !Tdisplaydisp
#define ESP32_RX 16 // ESP_MEMS TX pin
#else
#define ESP32_RX 33 // ESP_MEMS TX pin
#endif
#endif
#else
#define ESP8266_RX 14 // ESP_MEMS TX pin, D5
SoftwareSerial SerialESP(ESP8266_RX, 16);
#endif
#endif

#if !MobDataSP
#include <Adafruit_SHT31.h>
Adafruit_SHT31 sht31;
byte failh = 0;
#include "Adafruit_SHT4x.h"
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
#endif

#if !(Rosver || MinVer || MobData || MinVerSD)

#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"
Adafruit_AM2320 am2320 = Adafruit_AM2320();
bool AM2320flag = false;

#endif

#if CO2sensor
bool CO2measure = false;

#include "SparkFun_SCD30_Arduino_Library.h"
SCD30 airSensor;

#include "s8_uart.h"
#define S8_UART_PORT 1 // Change UART port if it is needed
HardwareSerial S8_serial(S8_UART_PORT);
S8_UART *sensor_S8;
S8_sensor sensorS8;
float hpa;

#endif

// Bluetooth in TTGO T-Display
#if Bluetooth
#include <Sensirion_Gadget_BLE.h> // to connect to Sensirion MyAmbience Android App available on Google Play
//#include <BLE2902.h>
// #include <BLEDevice.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>
NimBLELibraryWrapper lib;

#if (LTR390 || SoundMeter)
DataProvider provider(lib, DataType::PM10_PM25_PM40_PM100);
#elif CO2sensor
DataProvider provider(lib, DataType::T_RH_CO2);
#elif NoxVoxTd
//DataProvider provider(lib, DataType::T_RH_VOC_NOX_PM25); // Por error al bajar los datos con nox: assert failed: void ByteArray<SIZE>
DataProvider provider(lib, DataType::T_RH_VOC_PM25_V2);
#else
//DataProvider provider(lib, DataType::PM10_PM25_PM40_PM100);
DataProvider provider(lib, DataType::T_RH_VOC_PM25_V2);
#endif
#endif

#if !ESP8266
#define OUT_EN 26 // Enable del elevador de voltaje
#else
#define OUT_EN 12 // Enable del elevador de voltaje
#endif

#if !ESP8266

#if Wifi
// WiFi
// #define WM_DEBUG_LEVEL DEBUG_DEV
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

#if WPA2
#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
#endif

const int WIFI_CONNECT_TIMEOUT = 10000; // 10 seconds
WiFiServer wifi_server(80);
WiFiClient wifi_client;
bool PortalFlag = false;

WiFiManager wifiManager;

#endif

#else
#if Wifi

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

WiFiManager wifiManager;

// WiFi
#include <ESP8266WiFi.h> // Wifi ESP8266
extern "C"
{
#include "user_interface.h"

#if WPA2
#include "wpa2_enterprise.h"
#endif

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

#endif

#endif

#if Wifi

// MQTT
#include <PubSubClient.h>
char MQTT_message[256];
#if !MobData
PubSubClient MQTT_client(wifi_client);
#endif
char received_payload[384];
String MQTT_send_topic;
String MQTT_send_topicsam;
String MQTT_receive_topic;

byte contmqtt = 0;

//#if SoundMeter
#if Influxver
byte Influxseconds = 60;
byte Influxsecondssam = 60;
#endif

// #define MQTT_VERSION MQTT_VERSION_3_1

// JSON
#include <ArduinoJson.h>
//StaticJsonDocument<384> jsonBuffer;
JsonDocument jsonBuffer;

#if !ESP8266
// OTA Update
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#else
// For http binary updates
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#endif

#endif

#if !ESP8266
#include "rom/rtc.h"
#endif

bool ResetFlag = false;
bool DeepSleepFlag = false;
bool NoiseBUTTONFlag = false;
bool ResetFlagMobData = false;
bool ResetFlagMobDataTemp = false;

#if !ESP8266
void print_reset_reason(RESET_REASON reason);
#endif

// to know when there is an updating process in place
bool updating = false;

// To know when the device is in the following states
bool InCaptivePortal = false;
bool Calibrating = false;

// uint16_t SDyRTCtime = 15;       // Valor de Sample Time de SD y RTC
uint16_t SDyRTCtime = 60; // Valor de Sample Time de SD y RTC

//#if (Wifi || SDyRTC || SaveSDyRTC || Rosver)
#if (SDyRTC || SaveSDyRTC || Rosver || MinVerSD)
#if !WPA2

#include <SPI.h>
#include <SD.h>
#include "RTClib.h"

const int chipSelect = 10;
uint16_t SDreset = 0; // Valor en el que se resetea el ESP para verificar que la SD este conectada

#if SaveSDyRTC
#define ValSDreset 720
#else
#define ValSDreset 180
#endif

File dataFile;

RTC_DS1307 rtc;

String Valdate_time_id;

const char *customHtml = R"(
  <label for="date_time_id">Browser's Date & Hour for RTC clock:</label><br/>
  <input type="text" id="date_time_id" name="date_time_id" value="" readonly><br/>
  <script>
    function addZero(i) {
    if (i < 10) {i = "0" + i}
    return i;
    }
    const date = new Date();
    let year = date.getFullYear();
    let month = addZero(date.getMonth() + 1);
    let day = addZero(date.getDate());
    let hour = addZero(date.getHours());
    let minutes = addZero(date.getMinutes());
    let seconds = addZero(date.getSeconds());
    let datestring = year + ":" + month + ":" + day + "_" + hour + ":" + minutes + ":" + seconds;
    document.getElementById("date_time_id").value = datestring;
  </script>   
     )";

#endif
#endif
#if ESP8285
#define LEDPIN 13
#elif ESP32C3AG
#define LEDPIN 10
#else
#define LEDPIN 2
#endif

bool FlagLED = false;

#if LedNeo
#define PinLedNeo 48
#define LedBrightness 100       // Min 00, normal 150, max 255
#include <Adafruit_NeoPixel.h> //Libreria necesaria, instalarla desde el gestor de librerias
Adafruit_NeoPixel LED_RGB(1, PinLedNeo, NEO_GRBW + NEO_KHZ800);  // Creamos el objeto que manejará el led rgb PinLedNeo
#endif

#if LTR390UV
#include <Wire.h>
#include <LTR390.h>
#define I2C_ADDRESS 0x53
LTR390 ltr390(I2C_ADDRESS);
float getUVIval;
uint32_t rawUVS;

#define EnLTR390 13
//#define EnLTR390 26

#endif

// MOBILE DATA TRANSMISION

#if MobData

// Select your modem:
#if A7670
#define TINY_GSM_MODEM_A7670
#elif SIM7070
#define TINY_GSM_MODEM_SIM7070
#define PowerSIM7070 0
#elif SIM800
#define TINY_GSM_MODEM_SIM800
#endif

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// SoftwareSerial pmsSerial2(PMS_TX2, PMS_RX2);
SoftwareSerial SerialAT(13, 12);  // D7 RX, D6 TX en la board, conectar al SIM     // Si se usan 2 PMSx003 habria conflicto con el pin 12

// Use Hardware Serial on Mega, Leonardo, Micro
//#define SerialAT Serial2

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Your GPRS credentials, if any
#if TigoKalleyExito
const char apn[] = "web.colombiamovil.com.co";
#elif MovistarVirgin
const char apn[] = "internet.movistar.com.co";
#elif Claro
const char apn[] = "internet.comcel.com.co";
#elif Wom
const char apn[] = "internet.wom.co";
#endif

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient MQTT_client(client);

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // Initialize serial port for serial monitor in Arduino IDE
#if !ESP8266SH
  Serial.begin(115200);
#else
  Serial.begin(9600);
#endif
  delay(100);
#if Tdisplaydisp
  // Out for power on and off sensors
  pinMode(OUT_EN, OUTPUT);
  // Off sensors
  digitalWrite(OUT_EN, LOW); // step-up off

#if LTR390UV
  pinMode(EnLTR390, OUTPUT);

  // Configurar el pin con resistencia pulldown interna
  gpio_pulldown_en((gpio_num_t)EnLTR390);
  gpio_pullup_dis((gpio_num_t)EnLTR390);
  
  // Opcionalmente, configurar el estado del pin durante deep sleep
  //gpio_hold_en((gpio_num_t)EnLTR390);


  // Off sensors
  digitalWrite(EnLTR390, LOW); // LTR390 off
#endif

#endif

#if SIM7070
  pinMode(PowerSIM7070, INPUT);
#endif

  while (!Serial)
  {
    delay(500); // wait 0.5 seconds for connection
  }

  Serial.setDebugOutput(true);
  Serial.println(F(""));

  ResetFlagMobDataTemp = true;

#if Wifi

#if !ESP8266
  Serial.print(F("CPU0 reset reason:"));
  print_reset_reason(rtc_get_reset_reason(0));
#else
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
#endif

#endif

#if Bluetooth
  if (DeepSleepFlag == true)
  {
    delay(100);
    if (digitalRead(BUTTON_TOP) == false)
    {
      delay(900);
      if (digitalRead(BUTTON_TOP) == false)
      {
        NoiseBUTTONFlag = false;
        Serial.println(F("NoiseBUTTONFlag = false"));
      }
      else
      {
        NoiseBUTTONFlag = true;
        Serial.println(F("NoiseBUTTONFlag = true"));
        Suspend_Device();
      }
    }
    else
    {
      NoiseBUTTONFlag = true;
      Serial.println(F("NoiseBUTTONFlag = true"));
      Suspend_Device();
    }
  }
#endif

  // print info
  Serial.println();
  Serial.println(F("##### Inicializando Medidor Aire Ciudadano #####"));

#if !ESP8266
  // init preferences to handle persitent config data
  preferences.begin("config"); // use "config" namespace
#endif

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
#if !ESP8266
  digitalWrite(LEDPIN, LOW);
#if ESP32C3AG
  digitalWrite(LEDPIN, HIGH);
  delay(500);
  digitalWrite(LEDPIN, LOW);
#endif
#else
  digitalWrite(LEDPIN, HIGH);
#endif

#if LedNeo
  LED_RGB.begin();            // Inicia el objeto que hemos creado asociado a la librería NeoPixel
  LED_RGB.setBrightness(LedBrightness)    ; // Para el brillo del led

  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(0, 255, 0)));   // Encendemos el led verde
  LED_RGB.show(); // Enciende el color
  delay(500);

  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(180, 180, 0))); // Encendemos el led amarillo
  LED_RGB.show(); // Enciende el color
  delay(500);

  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(255, 125, 0))); // Encendemos el led naranja
  LED_RGB.show(); // Enciende el color
  delay(500);

  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(255, 0, 0)));   // Encendemos el led rojo
  LED_RGB.show(); // Enciende el color
  delay(500);

  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(128, 0, 255))); // Encendemos el led morado
  LED_RGB.show(); // Enciende el color
  delay(500);

  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(128, 128, 128))); // Encendemos el led blanco bajo brillo
  LED_RGB.show(); // Enciende el color
  delay(500);

  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(0, 0, 0)));     // Apagado
  LED_RGB.show(); // Enciende el color
  delay(500);

#endif

  aireciudadano_device_id = eepromConfig.aireciudadano_device_name;

  float Floatver = sw_version.toFloat();
  Swver = Floatver * 10;
  Serial.print(F("SW version: "));
  Serial.println(sw_version);

  // Get device id
  Get_AireCiudadano_DeviceId();

#if Bluetooth
  Bluetooth_loop_time = eepromConfig.BluetoothTime;
#endif

#if Tdisplaydisp
  TDisplay = true;
  OLED66 = false;
  OLED96 = false;
#elif OLED66display
  TDisplay = false;
  OLED66 = true;
  OLED96 = false;
#elif OLED96display
  TDisplay = false;
  OLED66 = false;
  OLED96 = true;
#else
  TDisplay = false;
  OLED66 = false;
  OLED96 = false;
#endif

#if BrownoutOFF
  // OFF BROWNOUT/////////////////////
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable   detector
#endif

#if (Tdisplaydisp || OLED96display || OLED66display)

  if (TDisplay == true)
  {
#if !ESP8266
    // Initialize TTGO Display and show AireCiudadano splash screen
    Button_Init();
    Display_Init();
    Display_Splash_Screen();

    for (int i = 0; i < 10; i++)
    {
#if Tdisplaydisp
      button_top.loop();
#endif
      delay(500);
    }
#endif
  }
  else if (OLED66 == true || OLED96 == true)
  {
    pinMode(BUTTON_BOTTOM, INPUT_PULLUP);
#if !Tdisplaydisp
    displayInit();
    pageStart();
    showWelcome();
    delay(100);
    u8g2.drawXBM(16, 18, 32, 32, IconoAC);
    pageEnd();
#endif
    delay(3000);
  }

#else

  pinMode(BUTTON_BOTTOM, INPUT_PULLUP);
  delay(100);

#endif

#if Tdisplaydisp
  // On sensors
  digitalWrite(OUT_EN, HIGH); // step-up on

#if EnLTR390
  digitalWrite(EnLTR390, HIGH); // LTR390 on
#endif

  delay(100);
#endif

#if Wifi
  // Set MQTT topics
#if !Influxver
  MQTT_send_topic = "measurement"; // measurement are sent to this topic
#else
  MQTT_send_topic = "measurementinflux"; // topic para InfluxDB
#if SoundAM
  MQTT_send_topicsam = "measurementinfsam"; // topic para InfluxDB
#endif
#endif                                                      // measurementfix are sent to this topic
  MQTT_receive_topic = "config/" + aireciudadano_device_id; // Config messages will be received in config/id
#endif

  // Print initial configuration
  Print_Config();

#if Wifi
  // Set Latitude and Longitude
  latitudef = atof(eepromConfig.sensor_lat);
  longitudef = atof(eepromConfig.sensor_lon);

  // Initialize the GadgetBle Library for Bluetooth
#elif Bluetooth
  provider.begin(Bluetooth_loop_time);
  Serial.print(F("Sensirion Provider Lib initialized with deviceId = "));
  Serial.println(provider.getDeviceIdString());

#endif

#if Wifi

  // Start Captive Portal for 60 seconds
  if (ResetFlag == true)
  {
#if !MobDataSP
#if (Rosver || MinVer || MobData || MinVerSD)
    Serial.println(F("Test_Sensor"));
    Test_Sensor();
#endif
#endif
    Start_Captive_Portal();
    delay(100);
  }
  if (SDflag == false)
  {
    if (FlagMobData == false)
    {
      // Attempt to connect to WiFi network:
      Connect_WiFi();
      Serial.println("Connect_Wifi");

      // Attempt to connect to MQTT broker
      if (!err_wifi)
      {
        Serial.println("Init_MQTT");
        Init_MQTT();

#if ESP8285
        digitalWrite(LEDPIN, LOW); // turn the LED off by making the voltage LOW
        delay(750);                // wait for a 750 msecond
        digitalWrite(LEDPIN, HIGH);
#endif
      }
    }
  }
  if (FlagMobData == true)
  {
#if MobData
    SerialAT.begin(19200);
    delay(10);

#if SIM7070
    if (ResetFlag == true)
    {
      Serial.println("Power ON SIM7070 routine");
      pinMode(PowerSIM7070, OUTPUT);
      digitalWrite(PowerSIM7070, LOW);
      delay(3000);
      pinMode(PowerSIM7070, INPUT);
      delay(7000);
    }
#endif

    // Attempt to connect to Mobile Data network:
connectstart:
    Serial.println("Connect MobData routine");
    Connect_MobData();                // Setup Connect Mob Data
    // Attempt to connect to MQTT broker
    Serial.println("Init MQTT routine");
    FlagMQTTcon = false;
    Init_MQTT();
    if (FlagMQTTcon == false)
      goto connectstart;
#endif
  }
#endif

  // Initialize sensors

#if CO2sensor
  Setup_CO2sensor();
#elif SoundMeter
  Setup_SoundMeter();
#elif LTR390UV
  Setup_UV();
#else
  Setup_Sensor();
#endif

  // Init control loops
  measurements_loop_start = millis();
  errors_loop_start = millis();
  MQTT_loop_review = millis();
#if Wifi
  MQTT_loop_start = millis();
  MQTT_loop_startsam = millis();
#endif

#if (SDyRTC || SaveSDyRTC || Rosver || MinVerSD)
#if !WPA2
#if !SaveSDyRTC
  if (SDflag == true)
  {
#endif
    SDreset = ValSDreset;

    Serial.print(F("Initializing SD card: "));
    // make sure that the default chip select pin is set to output, even if you don't use it:
    pinMode(SS, OUTPUT);

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect))
    {
      Serial.println(F("Card failed, or not present"));
      Serial.println(F("Review SD card or connections"));
    }
    else
    {
      Serial.println(F("OK, card initialized"));

      // Open up the file we're going to log to!
      // dataFile = SD.open("datalog.txt", FILE_WRITE);
      dataFile = SD.open(aireciudadano_device_id + ".txt", FILE_WRITE);
      if (!dataFile)
      {
        Serial.println(F("error opening aireciudadano_device_id.txt"));
        Serial.println(F("Review the SD card"));
      }
      else
      {
        dataFile.println("");
        dataFile.println(aireciudadano_device_id);
        dataFile.println("Date_Hour_PM2.5_Hum(optional)_Temp(Optional)_RESET(Optional)");
        dataFile.close();
        // print to the serial port too:
        Serial.println(F("OK, SD card file open"));
        digitalWrite(LEDPIN, LOW);  // turn the LED on (HIGH is the voltage level)
        delay(500);                 // wait for a 500 msecond
        digitalWrite(LEDPIN, HIGH); // turn the LED off by making the voltage LOW
        delay(500);                 // wait for a 500 msecond
        digitalWrite(LEDPIN, LOW);  // turn the LED on (HIGH is the voltage level)
        delay(500);                 // wait for a 500 msecond
        digitalWrite(LEDPIN, HIGH); // turn the LED off by making the voltage LOW
        delay(500);                 // wait for a 500 msecond
      }
    }

    Serial.print(F("Initializing RTC ds1307: "));

    if (!rtc.begin())
    {
      Serial.println(F("Couldn't find RTC"));
      Serial.flush();
    }
    else
    {
      Serial.println(F("OK, ds1307 init"));
      digitalWrite(LEDPIN, LOW);  // turn the LED on (HIGH is the voltage level)
      delay(200);                 // wait for a 500 msecond
      digitalWrite(LEDPIN, HIGH); // turn the LED off by making the voltage LOW
      delay(200);                 // wait for a 500 msecond
      digitalWrite(LEDPIN, LOW);  // turn the LED on (HIGH is the voltage level)
      delay(200);                 // wait for a 500 msecond
      digitalWrite(LEDPIN, HIGH); // turn the LED off by making the voltage LOW
    }

    if (!rtc.isrunning())
      Serial.println(F("RTC is NOT running"));
    else
      Serial.println(F("ds1307 is running, no changes"));
#if !SaveSDyRTC
  }
#endif
#endif
#endif

  // Get device id
#if (Rosver || SoundMeter || MinVer || MinVerSD)
  IDn = 0;
  Aireciudadano_Characteristics();
#endif

  Serial.println(F(""));
  Serial.println("### Configuración del medidor AireCiudadano finalizada ###\n");

#if (Tdisplaydisp || OLED96display || OLED66display)

  if (TDisplay == true)
  {
#if !ESP8266
#if Tdisplaydisp
    tft.fillScreen(TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.setTextDatum(6); // bottom left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Medidor", tft.width() / 2, (tft.height() / 2) - 30);
    tft.drawString("AireCiudadano", tft.width() / 2, tft.height() / 2);
    tft.drawString("ver: " + String(), tft.width() / 2 - 20, (tft.height() / 2) + 30);
    tft.drawString(sw_version.c_str(), tft.width() / 2 + 20, (tft.height() / 2) + 30);
    delay(2000);
    // Update display with new values
    Update_Display();
#endif
#endif
  }
  else if (OLED66 == true || OLED96 == true)
  {
#if !Tdisplaydisp
    pageStart();
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.setCursor(0, (dh / 2 - 4));
    u8g2.print("Medidor Listo");
    delay(1000);
    pageEnd();
#endif
  }

#endif

  delay(1000);
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
#if Bluetooth
#if Tdisplaydisp
  // Measure the battery voltage
  battery_voltage = ((float)analogRead(ADC_PIN) / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
#endif
#endif

  // Measurement loop
  if ((millis() - measurements_loop_start) >= measurements_loop_duration)
  {

    // New timestamp for the loop start time
    measurements_loop_start = millis();

    // Read sensors

#if CO2sensor
    Read_CO2sensor();
#elif SoundMeter
    Read_SoundMeter();
#elif LTR390UV
    Read_UV();
#else
    Read_Sensor();
#endif

    if (FlagLED == true)
      FlagLED = false;
    else
#if !ESP8266
      digitalWrite(LEDPIN, LOW);
#else
      digitalWrite(LEDPIN, HIGH);
#endif

    if (NoSensor == false)
    {
      if (PM25_value >= 0)
      {

#if (Tdisplaydisp || OLED96display || OLED66display)

        // Update display with new values
        if (TDisplay == true)
        {
#if !ESP8266
#if Tdisplaydisp
          Update_Display();
#endif
#endif
        }
        else if (OLED66 == true || OLED96 == true)
        {
#if !Tdisplaydisp
          UpdateOLED();
#endif
        }

#else
#if Bluetooth
        TimeConfig();
#endif
#endif

        // Accumulates samples
#if !TwoPMS
        PM25_accumulated += PM25_value;
        PM1_accumulated += PM1_value;
#else
        PM251_accumulated += PM251_value;
        PM252_accumulated += PM252_value;
        PM11_accumulated += PM11_value;
        PM12_accumulated += PM12_value;
#endif
#if SoundAM
        PM25_accumulatedsam += PM25_valuesam;
#endif
#if !TwoPMS
        PM25_accumulated_ori += PM25_value_ori;
#else
        PM251_accumulated_ori += PM251_value_ori;
        PM252_accumulated_ori += PM252_value_ori;
#endif
        PM25_samples++;
#if SoundAM
        SP_samples++;
#endif
        Con_loop_times++;
      }
    }
    else
    {
      // Rutina Test para enviar datos sin sensor conectado
      /*
        PM25_value = random(1, 99);
        PM25_accumulated += PM25_value;
        PM25_samples++;
        Con_loop_times++;
        Serial.print(F("Valor random: "));
        Serial.println(PM25_value);
      */
      Serial.println(F("Medidor No configurado"));

#if (Tdisplaydisp || OLED96display || OLED66display)

      if (TDisplay == true)
      {
#if !ESP8266
#if Tdisplaydisp
        tft.fillScreen(TFT_BLUE);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        tft.setTextDatum(6); // bottom left
        tft.setTextSize(1);
        tft.setFreeFont(FF90);
        tft.setTextDatum(MC_DATUM);
        if (Bluetooth == true)
        {
          tft.drawString("Sensores", tft.width() / 2, (tft.height() / 2) - 30);
          tft.drawString("No conectados", tft.width() / 2, (tft.height() / 2) + 20);
        }
        else
        {
          tft.drawString("Medidor", tft.width() / 2, (tft.height() / 2) - 30);
          tft.drawString("No configurado", tft.width() / 2, (tft.height() / 2) + 20);
        }
        delay(1000);
#endif
#endif
      }
      else if (OLED66 == true || OLED96 == true)
      {

#if !Tdisplaydisp
        pageStart();
        u8g2.setFont(u8g2_font_5x8_tf);
        if (Bluetooth == true)
        {
          u8g2.setCursor(4, (dh / 2 - 7));
          u8g2.print("Sensores No");
          u8g2.setCursor(8, (dh / 2 + 7));
          u8g2.print("conectados");
        }
        else
        {
          u8g2.setCursor(8, (dh / 2 - 7));
          u8g2.print("Medidor No");
          u8g2.setCursor(5, (dh / 2 + 7));
          u8g2.print("configurado");
        }
        pageEnd();
#endif
        delay(2000);
      }

#endif
    }
  }

#if Bluetooth

  // Bluetooth loop

  if (Con_loop_times >= eepromConfig.BluetoothTime)
  {
    float PM25f;
    PM25f = PM25_accumulated / PM25_samples;
    pm25int = round(PM25f);
#if CO2sensor
    Serial.print(F("CO2: "));
    Serial.print(pm25int);
    Serial.println(F(" ppm"));
#elif SoundMeter
    Serial.print(F("SPL: "));
    Serial.print(PM25f, 1);
    Serial.print(F(" dBA    Max: "));
    Serial.print(dBAmax, 1);
    Serial.println(F(" dBA"));
#elif LTR390UV
    Serial.print(F("UV Index int: "));
    Serial.println(pm25int);
#else
    Serial.print(F("PM2.5: "));
    Serial.print(pm25int);
    Serial.print(F(" ug/m3"));
    Serial.print(F("   "));
#endif
#if !(SoundMeter || LTR390UV)
    ReadHyT();
#endif
    Write_Bluetooth();
#if SaveSDyRTC
    Write_SD();
#endif
    PM25_accumulated = 0.0;
    PM25_samples = 0.0;
    Con_loop_times = 0;
    dBAmax = 0.0;
  }

  //#elif (Wifi || SDyRTC || Rosver)
#elif (Wifi || SDyRTC || MobData)

  if (SDflag == true)
  {
#if !(WPA2 || MinVer)     // Deberia incluir SoundMeter y depronto otras q no hagan Write_SD?
    // SDyRTC loop

    if (Con_loop_times >= SDyRTCtime)
    {
      float PM25f;

      PM25f = PM25_accumulated / PM25_samples;
      pm25int = round(PM25f);
      Serial.print(F("PM2.5: "));
      Serial.print(pm25int);
      Serial.print("   ");
#if !SoundMeter
      ReadHyT();
#endif
#if SDyRTC
      Write_SD();
#endif
      PM25_accumulated = 0.0;
      PM25_samples = 0.0;
      Con_loop_times = 0;
      dBAmax = 0.0;
    }
#endif
  }
  else if (FlagMobData == true)
  {
    // MQTT loop
#if !Influxver
    if ((millis() - MQTT_loop_start) >= (eepromConfig.PublicTime * 60000))
      //  if ((millis() - MQTT_loop_start) >= (eepromConfig.PublicTime * 15000))
      //  if ((millis() - MQTT_loop_start) >= (1 * 60000))
#else
    if ((millis() - MQTT_loop_start) >= (eepromConfig.PublicTime * 1000 * Influxseconds))
#endif
    {
      // New timestamp for the loop start time
      MQTT_loop_start = millis();

#if MobData
      MobDataConnected();     // MobDataConnected revision!!!!!!!!
#endif

      // Message the MQTT broker in the cloud app to send the measured values
      if (PM25_samples > 0)  //!!!!!!!!!!!
      {
        MQTT_client.loop();               // Ocurre cada lectura de Sensor, osea cada 1seg
        Serial.println("MQTT_client.loop");
        Send_Message_Cloud_App_MQTT();
      }
#if SaveSDyRTC
      Write_SD();
#endif

#if LedNeo
      LedNeoAverage(pm25int);
#endif

      // Reset samples after sending them to the MQTT server
      PM25_accumulated = 0.0;
      PM25_accumulated_ori = 0.0;
      PM251_accumulated = 0.0;
      PM251_accumulated_ori = 0.0;
      PM252_accumulated = 0.0;
      PM252_accumulated_ori = 0.0;
      PM1_accumulated = 0.0;
      PM11_accumulated = 0.0;
      PM12_accumulated = 0.0;
      PM25_samples = 0.0;
      dBAmax = 0.0;
    }
#if SoundAM
    if ((millis() - MQTT_loop_startsam) >= (1000 * Influxsecondssam))
    {
      // New timestamp for the loop start time
      MQTT_loop_startsam = millis();

      // Message the MQTT broker in the cloud app to send the measured values
      if (SP_samples > 0)
      {
        Send_Message_Cloud_App_MQTTsam();
      }

      // Reset samples after sending them to the MQTT server
      PM25_accumulatedsam = 0.0;
      SP_samples = 0.0;
      dBAmaxsam = 0.0;
    }
#endif
  }
  else
  {
    // MQTT loop
#if !Influxver
    if ((millis() - MQTT_loop_start) >= (eepromConfig.PublicTime * 60000))
      //  if ((millis() - MQTT_loop_start) >= (eepromConfig.PublicTime * 3000))
      //  if ((millis() - MQTT_loop_start) >= (1 * 60000))
#else
    if ((millis() - MQTT_loop_start) >= (eepromConfig.PublicTime * 1000 * Influxseconds))
#endif
    {
      // New timestamp for the loop start time
      MQTT_loop_start = millis();

      // Message the MQTT broker in the cloud app to send the measured values
      if ((!err_wifi) && (PM25_samples > 0))
      {
        Send_Message_Cloud_App_MQTT();
      }

#if SaveSDyRTC
      Write_SD();
#endif

#if LedNeo
      LedNeoAverage(pm25int);
#endif

      // Reset samples after sending them to the MQTT server
      PM25_accumulated = 0.0;
      PM25_accumulated_ori = 0.0;
      PM251_accumulated = 0.0;
      PM251_accumulated_ori = 0.0;
      PM252_accumulated = 0.0;
      PM252_accumulated_ori = 0.0;
      PM1_accumulated = 0.0;
      PM11_accumulated = 0.0;
      PM12_accumulated = 0.0;
      PM25_samples = 0.0;
      dBAmax = 0.0;
    }
#if SoundAM
    if ((millis() - MQTT_loop_startsam) >= (1000 * Influxsecondssam))
    {
      // New timestamp for the loop start time
      MQTT_loop_startsam = millis();

      // Message the MQTT broker in the cloud app to send the measured values
      if ((!err_wifi) && (SP_samples > 0))
      {
        Send_Message_Cloud_App_MQTTsam();
      }

      // Reset samples after sending them to the MQTT server
      PM25_accumulatedsam = 0.0;
      SP_samples = 0.0;
      dBAmaxsam = 0.0;
    }
#endif
  }
#endif

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

#if Wifi

    if (SDflag == false)
    {
      if (FlagMobData == false)
      {

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
#if !ESP8266
          MQTT_Reconnect();
#endif
          Init_MQTT();
        }
      }
    }
#endif
  }

#if Wifi

  if (SDflag == false)
  {
    if (FlagMobData == false)
    {
      if ((millis() - MQTT_loop_review) >= MQTT_loop_review_duration)     // Revisar cada x milisegundos
      {
        // New timestamp for the loop start time
        MQTT_loop_review = millis();
        // From here, all other tasks performed outside of measurements, MQTT and error loops

        // if not there are not connectivity errors, receive MQTT messages
        if ((!err_MQTT) && (!err_wifi))
        {
          MQTT_client.loop();                     // Ocurre cada lectura de Sensor, osea cada 1seg
          Serial.println("MQTT_client.loop");
        }
        // Process wifi server requests
        Check_WiFi_Server();
      }
    }
  }
#endif

  // Process Bluetooth events
#if Bluetooth
  provider.handleDownload();
  delay(3);
#endif

#if !ESP8266
#if Tdisplaydisp
  if (TDisplay == true)
  {
    // Process buttons events
    button_top.loop();
    button_bottom.loop();
  }
#endif
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////

#if (Wifi || MobData)

#if !ESP8266

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d - ", event);

  switch (event)
  {
    case SYSTEM_EVENT_WIFI_READY:
      Serial.println(F("WiFi interface ready"));
      break;
    case SYSTEM_EVENT_SCAN_DONE:
      Serial.println(F("Completed scan for access points"));
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println(F("WiFi client started"));
      break;
    case SYSTEM_EVENT_STA_STOP:
      Serial.println(F("WiFi clients stopped"));
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println(F("Connected to access point"));
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println(F("Disconnected from WiFi access point"));
      break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
      Serial.println(F("Authentication mode of access point has changed"));
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print(F("Obtained IP address: "));
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.println(F("Lost IP address and IP address is reset to 0"));
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.println(F("WiFi Protected Setup (WPS): succeeded in enrollee mode"));
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      Serial.println(F("WiFi Protected Setup (WPS): failed in enrollee mode"));
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      Serial.println(F("WiFi Protected Setup (WPS): timeout in enrollee mode"));
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.println(F("WiFi Protected Setup (WPS): pin code in enrollee mode"));
      break;
    case SYSTEM_EVENT_AP_START:
      Serial.println(F("WiFi access point started"));
      break;
    case SYSTEM_EVENT_AP_STOP:
      Serial.println(F("WiFi access point  stopped"));
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println(F("Client connected"));
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println(F("Client disconnected"));
      break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      Serial.println(F("Assigned IP address to client"));
      break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
      Serial.println(F("Received probe request"));
      break;
    case SYSTEM_EVENT_GOT_IP6:
      Serial.println(F("IPv6 is preferred"));
      break;
    case SYSTEM_EVENT_ETH_START:
      Serial.println(F("Ethernet started"));
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println(F("Ethernet stopped"));
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println(F("Ethernet connected"));
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println(F("Ethernet disconnected"));
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.println(F("Obtained IP address"));
      break;
    default:
      break;
  }
}

#else
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

#endif

void Connect_WiFi()
{ // Connect to WiFi

  WiFi.disconnect(true); // disconnect from wifi to set new wifi connection
  WiFi.mode(WIFI_STA);   // init wifi mode

  // Highest RF power output
  if (MaxWifiTX == true)
#if !ESP8266
  {
    WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Default 14dB
    Serial.println("MaxWifiTX = true");
  }
#else
  {
    WiFi.setOutputPower(20.5);        // Default 17dB
    Serial.println("MaxWifiTX = true");
  }
#endif

#if !ESP8266

  WiFi.onEvent(WiFiEvent);

#if !PreProgSensor
  wifi_config_t conf;
  esp_wifi_get_config(WIFI_IF_STA, &conf); // Get WiFi configuration
  Serial.print(F("Attempting to connect to WiFi network: "));
  Serial.println(String(reinterpret_cast<const char *>(conf.sta.ssid))); // WiFi.SSID() is not filled up until the connection is established
#endif

#endif

#if !WPA2

#if ESP8266
#if !PreProgSensor
  WiFi.begin();
#else
  WiFi.begin(ssid, password);
#endif
#endif

#else
  // #if WPA2
  //  If there are not wifi identity and wifi password defined, proceed to traight forward configuration

  if ((strlen(eepromConfig.wifi_user) == 0) && (strlen(eepromConfig.wifi_password) == 0))
  {
    Serial.println(F("Attempting to authenticate..."));

#if ESP8266

#if !PreProgSensor
    WiFi.begin();
#else
    WiFi.begin(ssid, password);
#endif

#endif
  }
  else
  {
#if WPA2
    // set up wpa2 enterprise
#if !ESP8266
    Serial.println(F("Attempting to authenticate using WPA2 Enterprise..."));
    Serial.print(F("Identity: "));
    Serial.println(eepromConfig.wifi_user);
    Serial.print(F("Password: "));
    Serial.println(eepromConfig.wifi_password);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));         // provide identity
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));         // provide username --> identity and username is same
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)eepromConfig.wifi_password, strlen(eepromConfig.wifi_password)); // provide password
    esp_wifi_sta_wpa2_ent_enable();

#else

    String wifi_ssid = WiFi.SSID(); // your network SSID (name)
    // String wifi_password = WiFi.psk()); // your network psk password
    Serial.println(F("Attempting to authenticate with WPA2 Enterprise "));
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
    wifi_station_set_wpa2_enterprise_auth(1);
    // Clean up to be sure no old data is still inside
    wifi_station_clear_cert_key();
    wifi_station_clear_enterprise_ca_cert();
    wifi_station_clear_enterprise_identity();
    wifi_station_clear_enterprise_username();
    wifi_station_clear_enterprise_password();
    wifi_station_clear_enterprise_new_password();

    // Set up authentication
    wifi_station_set_enterprise_identity((uint8 *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));
    wifi_station_set_enterprise_username((uint8 *)eepromConfig.wifi_user, strlen(eepromConfig.wifi_user));
    wifi_station_set_enterprise_password((uint8 *)eepromConfig.wifi_password, strlen((char *)eepromConfig.wifi_password));
    wifi_station_connect();

#endif

#endif
  }
  // #endif
#endif

#if ESP32

#if !PreProgSensor
  WiFi.begin();
#else
  WiFi.begin(ssid, password);
#endif

#endif

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
  }

#if ESP8266
  Print_WiFi_Status_ESP8266();
#else
  Print_WiFi_Status();
#endif
}

void Print_WiFi_Status()
{ // Print wifi status on serial monitor

  // Get current status
  //  WL_CONNECTED: assigned when connected to a WiFi network;
  //  WL_NO_SHIELD: assigned when no WiFi shield is present;
  //  WL_IDLE_STATUS: it is a temporary status assigned when WiFi.begin() is called and remains active until the number of attempts expires (resulting in WL_CONNECT_FAILED) or a connection is established (resulting in WL_CONNECTED);
  //  WL_NO_SSID_AVAIL: assigned when no SSID are available;
  //  WL_SCAN_COMPLETED: assigned when the scan networks is completed;
  //  WL_CONNECT_FAILED: assigned when the connection fails for all the attempts;
  //  WL_CONNECTION_LOST: assigned when the connection is lost;
  //  WL_DISCONNECTED: assigned when disconnected from a network;

  Serial.print(F("wifi_status: "));
#if (OLED66 == true || OLED96 == true)
  pageStart();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(10, dh / 2);
#endif
  switch (WiFi.status())
  {
    case WL_CONNECTED:
      Serial.println(F(" WIFI CONNECTED!!!"));
#if (OLED66 == true || OLED96 == true)
      u8g2.print("OK WIFI :)");
#endif
      break;
    case WL_NO_SHIELD:
      Serial.println(F("No WiFi HW detected"));
      break;
    case WL_IDLE_STATUS:
      Serial.println(F("Attempting..."));
      break;
    case WL_NO_SSID_AVAIL:
      Serial.println(F("No SSID available"));
#if (OLED66 == true || OLED96 == true)
      u8g2.print("NO WIFI :(");
#endif
      break;
    case WL_SCAN_COMPLETED:
      Serial.println(F("Networks scan completed"));
      break;
    case WL_CONNECT_FAILED:
      Serial.println(F("Connect failed"));
#if (OLED66 == true || OLED96 == true)
      u8g2.print("NO WIFI :(");
#endif
      break;
    case WL_CONNECTION_LOST:
      Serial.println(F("Connection lost"));
#if (OLED66 == true || OLED96 == true)
      u8g2.print("NO WIFI :(");
#endif
      break;
    case WL_DISCONNECTED:
      Serial.println(F("Disconnected"));
#if (OLED66 == true || OLED96 == true)
      u8g2.print("NO WIFI :(");
#endif
      break;
    default:
      Serial.println(F("Unknown status"));
      break;
  }
  Serial.println(F(""));
  delay(3000);
#if (OLED66 == true || OLED96 == true)
  pageEnd();
#endif

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
  Serial.print(F("Signal strength (RSSI): "));

  Serial.print(WiFi.RSSI());
  Serial.println(F(" dBm"));
}

void Check_WiFi_Server()                    // Server access by http when you put the ip address in a web browser !!!!!!!!!!!!!!!!!!!!!!!!!!!
{ // Wifi server
  WiFiClient client = wifi_server.accept(); // listen for incoming clients
  if (client)
  { // if you get a client,
    Serial.println(F("new client"));        // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected())
    { // loop while the client's connected
      if (client.available())
      { // if there's bytes to read from the client,
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
#if !Influxver
            client.print("80");
//            client.print("30183");
#else
            client.print("30183");
#endif
            client.println("<br>");
            client.print("Sensor latitude: ");
            client.print(eepromConfig.sensor_lat);
            client.println("<br>");
            client.print("Sensor longitude: ");
            client.print(eepromConfig.sensor_lon);
            client.println("<br>");
            client.println("------");
            client.println("<br>");
#if SoundMeter
            client.print("SPL: ");
            client.print(PM25_value); //!!!!!!!!!!!!!!!!!!!!!!!!!!!
#else
            client.print("PM2.5: ");
            client.print(PM25_value); //!!!!!!!!!!!!!!!!!!!!!!!!!!!
            client.println("<br>");
            client.print("Temperature: ");
            client.print(temp);
            client.println("<br>");
            client.print("Humidity: ");
            client.print(humi);
            client.println("<br>");
#endif
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
        { // if you got anything else but a carriage return character,
          currentLine += c; // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /1" to calibrate the sensor:

        // Check to see if the client request was "GET /3" to launch captive portal:
        if (currentLine.endsWith("GET /3"))
        {
          PortalFlag = true;
          Start_Captive_Portal();
        }
        // #if !ESP8266        // Check to see if the client request was "GET /4" to suspend the device:
        if (currentLine.endsWith("GET /4"))
        {
          Firmware_Update();
        }
        // #endif

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

  Serial.println(F("Start Captive Portal"));
  Serial.println(F("Timeout to login: 60 seconds -> timeout in portal: 300 seconds"));

  if (SDflag == false)
    captiveportaltime = 60;
  //    captiveportaltime = 15;
  else
    captiveportaltime = 30; // captiveportaltime = 15;

  wifiAP = aireciudadano_device_id;
  Serial.println(wifiAP);

#if (Tdisplaydisp || OLED96display || OLED66display)
#if !ESP8266
#if Tdisplaydisp

  if (TDisplay == true)
  {
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Portal Cautivo", tft.width() / 2, (tft.height() / 2 - 20));
    tft.drawString(wifiAP, tft.width() / 2, tft.height() / 2 + 10);
  }
#endif
#endif

  if (OLED66 == true || OLED96 == true)
  {
#if !Tdisplaydisp
    pageStart();
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.setCursor(2, (dh / 2) - 10);
    u8g2.print("Portal cautivo");

    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setCursor(5, dh / 2);
    u8g2.print(captiveportaltime);
    u8g2.setCursor(20, dh / 2);
    u8g2.print(" segundos");
    //    delay(2000);
    pageEnd();

#endif
  }

#endif

  wifi_server.stop();

  wifiManager.setDebugOutput(true);

  wifiManager.disconnect();

  WiFi.mode(WIFI_AP); // explicitly set mode, esp defaults to STA+AP

  // Captive portal parameters

#if WPA2
  WiFiManagerParameter custom_wifi_html("<p>Set WPA2 Enterprise</p>"); // only custom html
  WiFiManagerParameter custom_wifi_user("User", "WPA2 Enterprise identity", eepromConfig.wifi_user, 24);
  WiFiManagerParameter custom_wpa2_pass;
#if !(Rosver || SoundMeter || MinVer || MinVerSD)
  WiFiManagerParameter custom_wifi_html2("<p></p>"); // only custom html
#else
  WiFiManagerParameter custom_wifi_html2("<hr><br/>"); // only custom html
#endif
#endif

#if MobData
#if A7670
  WiFiManagerParameter custom_mobdata("<p>Mobile Data Version with A7670:</p>"); // only custom html
#elif SIM7070
  WiFiManagerParameter custom_mobdata("<p>Mobile Data Version with SIM7070:</p>"); // only custom html
#elif SIM800
  WiFiManagerParameter custom_mobdata("<p>Mobile Data Version with SIM800:</p>"); // only custom html
#endif
#endif

#if !ESP8266
  WiFiManagerParameter custom_id_name("CustomName", "Set Station Name (29 characters max):", eepromConfig.aireciudadano_device_name, 29);
#else
  WiFiManagerParameter custom_id_name("CustomName", "Set Station Name (25 characters max):", eepromConfig.aireciudadano_device_name, 25);
#endif

#if !(Rosver || SoundMeter || Minver || MinVerSD)
  char Ptime[5];
  itoa(eepromConfig.PublicTime, Ptime, 10);
  WiFiManagerParameter custom_public_time("Ptime", "Set Publication Time in minutes:", Ptime, 4);
  WiFiManagerParameter custom_sensor_html("<p></p>"); // only custom html
#endif
  WiFiManagerParameter custom_sensor_latitude("Latitude", "Latitude (5-4 dec digits are enough)", eepromConfig.sensor_lat, 10);
  WiFiManagerParameter custom_sensor_longitude("Longitude", "Longitude (5-4 dec)", eepromConfig.sensor_lon, 10);
#if !(Rosver || SoundMeter || MinVer || MinVerSD)
  WiFiManagerParameter custom_sensorPM_type;
  WiFiManagerParameter custom_sensorHYT_type;
  WiFiManagerParameter custom_display_type;
#endif
  WiFiManagerParameter custom_outin_type;
#if (Rosver || MinVerSD)
  WiFiManagerParameter custom_endhtmlup("<hr><p></p>"); // only custom html
  WiFiManagerParameter custom_sd_type;
  WiFiManagerParameter date_time;
#endif
  WiFiManagerParameter custom_endhtml("<p></p>"); // only custom html

#if !(Rosver || SoundMeter || MinVer || MinVerSD)
  // Sensor PM menu

  if (eepromConfig.ConfigValues[7] == '0')
  {
#if SDS011sen
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM:</label><br/><input type='radio' name='customSenPM' value='0' checked> None<br><input type='radio' name='customSenPM' value='1'> Sensirion SPS30 adjusted<br><input type='radio' name='customSenPM' value='2'> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3'> SDS011";
#else
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM:</label><br/><input type='radio' name='customSenPM' value='0' checked> None<br><input type='radio' name='customSenPM' value='1'> Sensirion SPS30 adjusted<br><input type='radio' name='customSenPM' value='2'> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3'> Plantower PMS adjusted";
#endif
    new (&custom_sensorPM_type) WiFiManagerParameter(custom_senPM_str);
  }
  else if (eepromConfig.ConfigValues[7] == '1')
  {
#if SDS011sen   
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM:</label><br/><input type='radio' name='customSenPM' value='0'> None<br><input type='radio' name='customSenPM' value='1' checked> Sensirion SPS30 adjusted<br><input type='radio' name='customSenPM' value='2'> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3'> SDS011";
#else
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM:</label><br/><input type='radio' name='customSenPM' value='0'> None<br><input type='radio' name='customSenPM' value='1' checked> Sensirion SPS30 adjusted<br><input type='radio' name='customSenPM' value='2'> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3'> Plantower PMS adjusted";
#endif
    new (&custom_sensorPM_type) WiFiManagerParameter(custom_senPM_str);
  }
  else if (eepromConfig.ConfigValues[7] == '2')
  {
#if SDS011sen
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM:</label><br/><input type='radio' name='customSenPM' value='0'> None<br><input type='radio' name='customSenPM' value='1'> Sensirion SPS30 adjusted<br><input type='radio' name='customSenPM' value='2' checked> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3'> SDS011";
#else
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM:</label><br/><input type='radio' name='customSenPM' value='0'> None<br><input type='radio' name='customSenPM' value='1'> Sensirion SPS30 adjusted<br><input type='radio' name='customSenPM' value='2' checked> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3'> Plantower PMS adjusted";
#endif
    new (&custom_sensorPM_type) WiFiManagerParameter(custom_senPM_str);
  }
  else if (eepromConfig.ConfigValues[7] == '3')
  {
#if SDS011sen
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM:</label><br/><input type='radio' name='customSenPM' value='0'> None<br><input type='radio' name='customSenPM' value='1'> Sensirion SPS30 adjusted<br><input type='radio' name='customSenPM' value='2'> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3' checked> SDS011";
#else
    const char *custom_senPM_str = "<br/><br/><label for='customSenPM'>Sensor PM:</label><br/><input type='radio' name='customSenPM' value='0'> None<br><input type='radio' name='customSenPM' value='1'> Sensirion SPS30 adjusted<br><input type='radio' name='customSenPM' value='2'> Sensirion SEN5X<br><input type='radio' name='customSenPM' value='3' checked> Plantower PMS adjusted";
#endif
    new (&custom_sensorPM_type) WiFiManagerParameter(custom_senPM_str);
  }

  // Sensor HYT menu

  if (eepromConfig.ConfigValues[6] == '0')
  {
    const char *custom_senHYT_str = "<br/><br/><label for='customSenHYT'>Sensor HyT:</label><br/><input type='radio' name='customSenHYT' value='0' checked> None or integrated (SEN54/5-PMSx003T)<br><input type='radio' name='customSenHYT' value='1'> Sensirion SHT31/SHT4x<br><input type='radio' name='customSenHYT' value='2'> AM2320";
    new (&custom_sensorHYT_type) WiFiManagerParameter(custom_senHYT_str);
  }
  else if (eepromConfig.ConfigValues[6] == '1')
  {
    const char *custom_senHYT_str = "<br/><br/><label for='customSenHYT'>Sensor HyT:</label><br/><input type='radio' name='customSenHYT' value='0'> None or integrated (SEN54/5-PMSx003T)<br><input type='radio' name='customSenHYT' value='1' checked> Sensirion SHT31/SHT4x<br><input type='radio' name='customSenHYT' value='2'> AM2320";
    new (&custom_sensorHYT_type) WiFiManagerParameter(custom_senHYT_str);
  }
  else if (eepromConfig.ConfigValues[6] == '2')
  {
    const char *custom_senHYT_str = "<br/><br/><label for='customSenHYT'>Sensor HyT:</label><br/><input type='radio' name='customSenHYT' value='0'> None or integrated (SEN54/5-PMSx003T)<br><input type='radio' name='customSenHYT' value='1'> Sensirion SHT31/SHT4x<br><input type='radio' name='customSenHYT' value='2' checked> AM2320";
    new (&custom_sensorHYT_type) WiFiManagerParameter(custom_senHYT_str);
  }

  // Sensor Display menu

  if (eepromConfig.ConfigValues[5] == '0')
  {
    const char *custom_display_str = "<br/><br/><label for='customDisplay'>Display:</label><br/><input type='radio' name='customDisplay' value='0' checked> None<br><input type='radio' name='customDisplay' value='1'> TTGO T-Display<br><input type='radio' name='customDisplay' value='2'> OLED 0.96 inch - 128x64p<br><input type='radio' name='customDisplay' value='3'> OLED 0.66 inch - 64x48p";
    new (&custom_display_type) WiFiManagerParameter(custom_display_str);
  }
  else if (eepromConfig.ConfigValues[5] == '1')
  {
    const char *custom_display_str = "<br/><br/><label for='customDisplay'>Display:</label><br/><input type='radio' name='customDisplay' value='0'> None<br><input type='radio' name='customDisplay' value='1' checked> TTGO T-Display<br><input type='radio' name='customDisplay' value='2'> OLED 0.96 inch - 128x64p<br><input type='radio' name='customDisplay' value='3'> OLED 0.66 inch - 64x48p";
    new (&custom_display_type) WiFiManagerParameter(custom_display_str);
  }
  else if (eepromConfig.ConfigValues[5] == '2')
  {
    const char *custom_display_str = "<br/><br/><label for='customDisplay'>Display:</label><br/><input type='radio' name='customDisplay' value='0'> None<br><input type='radio' name='customDisplay' value='1'> TTGO T-Display<br><input type='radio' name='customDisplay' value='2' checked> OLED 0.96 inch - 128x64p<br><input type='radio' name='customDisplay' value='3'> OLED 0.66 inch - 64x48p";
    new (&custom_display_type) WiFiManagerParameter(custom_display_str);
  }
  else if (eepromConfig.ConfigValues[5] == '3')
  {
    const char *custom_display_str = "<br/><br/><label for='customDisplay'>Display:</label><br/><input type='radio' name='customDisplay' value='0'> None<br><input type='radio' name='customDisplay' value='1'> TTGO T-Display<br><input type='radio' name='customDisplay' value='2'> OLED 0.96 inch - 128x64p<br><input type='radio' name='customDisplay' value='3' checked> OLED 0.66 inch - 64x48p";
    new (&custom_display_type) WiFiManagerParameter(custom_display_str);
  }

#endif

#if WPA2
  const char *custom_wpa2_pw = "<label for='p'>Password</label><input id='p' name='p' maxlength='64' type='password' placeholder='{p}'><input type='checkbox' onclick='f()'> Show Password";
  new (&custom_wpa2_pass) WiFiManagerParameter(custom_wpa2_pw); // REVISAR!!!!!!!!!!!
#endif

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

#if (Rosver || MinVerSD)
#if !WPA2
  // SD option
  if (eepromConfig.ConfigValues[4] == '0')
  {
    const char *custom_sd_str = "<label for='customSD'>SD version:</label><br/><input type='radio' name='customSD' value='0' checked> No SD & RTC<br><input type='radio' name='customSD' value='1'> SD & RTC mode";
    new (&custom_sd_type) WiFiManagerParameter(custom_sd_str);
  }
  else if (eepromConfig.ConfigValues[4] == '1')
  {
    const char *custom_sd_str = "<label for='customSD'>SD version:</label><br/><input type='radio' name='customSD' value='0'> No SD & RTC<br><input type='radio' name='customSD' value='1' checked> SD & RTC mode";
    new (&custom_sd_type) WiFiManagerParameter(custom_sd_str);
  }

  new (&date_time) WiFiManagerParameter(customHtml);

#endif
#endif

  // Add parameters

#if WPA2
  wifiManager.addParameter(&custom_wifi_user);
  wifiManager.addParameter(&custom_wpa2_pass);
  wifiManager.addParameter(&custom_wifi_html2);
#endif

#if MobData
  wifiManager.addParameter(&custom_mobdata);
#endif

  wifiManager.addParameter(&custom_id_name);
#if !(Rosver || SoundMeter || MinVer || MinVerSD)
  wifiManager.addParameter(&custom_public_time);
  wifiManager.addParameter(&custom_sensor_html);
#endif

  wifiManager.addParameter(&custom_sensor_latitude);
  wifiManager.addParameter(&custom_sensor_longitude);
#if !(Rosver || SoundMeter || MinVer || MinVerSD)
  wifiManager.addParameter(&custom_sensorPM_type);
  wifiManager.addParameter(&custom_sensorHYT_type);
  wifiManager.addParameter(&custom_display_type);
#endif
  wifiManager.addParameter(&custom_outin_type);
#if (Rosver || MinVerSD)
#if !WPA2
  wifiManager.addParameter(&custom_endhtmlup);
  wifiManager.addParameter(&custom_sd_type);
  wifiManager.addParameter(&custom_endhtml);
  wifiManager.addParameter(&date_time);
#endif
#else
  wifiManager.addParameter(&custom_endhtml);
#endif

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

#if WPA2
    strncpy(eepromConfig.wifi_user, custom_wifi_user.getValue(), sizeof(eepromConfig.wifi_user));
    eepromConfig.wifi_user[sizeof(eepromConfig.wifi_user) - 1] = '\0';
    Serial.println(F("Wifi Identity write_eeprom = true"));

    strncpy(eepromConfig.wifi_password, wifi_passwpa2, sizeof(eepromConfig.wifi_password));
    eepromConfig.wifi_password[sizeof(eepromConfig.wifi_password) - 1] = '\0';
    Serial.println(F("Wifi pass write_eeprom = true"));
#endif

    strncpy(eepromConfig.aireciudadano_device_name, custom_id_name.getValue(), sizeof(eepromConfig.aireciudadano_device_name));
    eepromConfig.aireciudadano_device_name[sizeof(eepromConfig.aireciudadano_device_name) - 1] = '\0';
    Serial.println(F("Devname write_eeprom = true"));

#if !(Rosver || SoundMeter || MinVer || MinVerSD)
    eepromConfig.PublicTime = atoi(custom_public_time.getValue());
    Serial.println(F("PublicTime write_eeprom = true"));
#endif

    strncpy(eepromConfig.sensor_lat, custom_sensor_latitude.getValue(), sizeof(eepromConfig.sensor_lat));
    eepromConfig.sensor_lat[sizeof(eepromConfig.sensor_lat) - 1] = '\0';
    Serial.println(F("Lat write_eeprom = true"));
    latitudef = atof(eepromConfig.sensor_lat); // Cambiar de string a float

    strncpy(eepromConfig.sensor_lon, custom_sensor_longitude.getValue(), sizeof(eepromConfig.sensor_lon));
    eepromConfig.sensor_lon[sizeof(eepromConfig.sensor_lon) - 1] = '\0';
    Serial.println(F("Lon write_eeprom = true"));
    longitudef = atof(eepromConfig.sensor_lon); // Cambiar de string a float

    CustomValTotalString[8] = {0};
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

#if !(Rosver || MinVer || MinVerSD)
    if (CustomValtotal == 0)
    {
      Serial.println(F("No configuration sensor values ​​chosen, no changes will be stored"));
    }
    else
    {
#endif
      strncpy(eepromConfig.ConfigValues, CustomValTotalString, sizeof(eepromConfig.ConfigValues));
      eepromConfig.ConfigValues[sizeof(eepromConfig.ConfigValues) - 1] = '\0';
      Serial.println(F("CustomVal write_eeprom = true"));
#if !(Rosver || MinVer || MinVerSD)
    }
#endif
    Write_EEPROM();
    Serial.println(F("write_eeprom = true Final"));
  }
  ESP.restart();
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
  Serial.println("Value customMobData = " + getParam("customMD"));
  CustomValtotal = CustomValtotal + (CustomValue * 100000);
  Serial.print(F("CustomValtotal: "));
  Serial.println(CustomValtotal);
  strncpy(wifi_passwpa2, getParam("p").c_str(), sizeof(wifi_passwpa2)); // REVISAR!!!!!!!!!!!

#if (Rosver || MinVerSD)
#if !WPA2
  if (getParam("customSD") == "1")
  {
    Serial.println("SD & RTC selected");
    SDflag = true;
    Valdate_time_id = getParamstring("date_time_id");
    Serial.println("PARAM customfieldid = " + Valdate_time_id);
    RTCadjustTime();
  }
  else
  {
    Serial.println("No SD & RTC");
    SDflag = false;
  }
#endif
#endif

#if MobData
  FlagMobData = true;
  Serial.println(F("Mobile Data mode"));
#else
  FlagMobData = false;
  Serial.println(F("NO Mobile Data mode"));
#endif

  ConfigPortalSave = true;
}

#if (Rosver || MinVerSD)
#if !WPA2
void RTCadjustTime()
{
  String Valdate_year = Valdate_time_id.substring(0, 4);
  String Valdate_month = Valdate_time_id.substring(5, 7);
  String Valdate_day = Valdate_time_id.substring(8, 10);
  String Valdate_hour = Valdate_time_id.substring(11, 13);
  String Valdate_min = Valdate_time_id.substring(14, 16);
  String Valdate_sec = Valdate_time_id.substring(17, 19);

  uint16_t year = Valdate_year.toInt();
  uint8_t month = Valdate_month.toInt();
  uint8_t day = Valdate_day.toInt();
  uint8_t hour = Valdate_hour.toInt();
  uint8_t min = Valdate_min.toInt();
  uint8_t sec = Valdate_sec.toInt();

  Serial.print("year = ");
  Serial.print(year);
  Serial.print(", month = ");
  Serial.print(month);
  Serial.print(", day = ");
  Serial.print(day);
  Serial.print(", hour = ");
  Serial.print(hour);
  Serial.print(", min = ");
  Serial.print(min);
  Serial.print(", sec = ");
  Serial.println(sec);

  Serial.print(F("Initializing RTC ds1307: "));

  if (!rtc.begin())
  {
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
  }
  else
  {
    Serial.println(F("OK, ds1307 init"));
    Serial.print(F("RTC let's set the time!: "));
    Serial.println(Valdate_time_id);

    rtc.adjust(DateTime(year, month, day, hour, min, sec));

    digitalWrite(LEDPIN, LOW);  // turn the LED on (HIGH is the voltage level)
    delay(200);                 // wait for a 500 msecond
    digitalWrite(LEDPIN, HIGH); // turn the LED off by making the voltage LOW
    delay(200);                 // wait for a 500 msecond
    digitalWrite(LEDPIN, LOW);  // turn the LED on (HIGH is the voltage level)
    delay(200);                 // wait for a 500 msecond
    digitalWrite(LEDPIN, HIGH); // turn the LED off by making the voltage LOW
  }
}

#endif
#endif

void Init_MQTT()
{ // MQTT Init function
  Serial.print(F("Attempting to connect to the MQTT broker "));
  //  Serial.print(eepromConfig.MQTT_server);
  Serial.print(F("sensor.aireciudadano.com"));
  Serial.print(F(":"));
  //  Serial.println(eepromConfig.MQTT_port);
#if !Influxver
  Serial.println(F("80"));
#else
  Serial.println(F("30183")); // InfluxDB port
#endif

#if !ESP8266
  MQTT_client.setBufferSize(512); // to receive messages up to 512 bytes length (default is 256)
#else
  MQTT_client.setBufferSize(1024);    // PROBAR!!!!!!!!!!!!!!!!!!!!!
#endif

#if !MobData
  MQTT_client.setKeepAlive(120);
#else
  MQTT_client.setKeepAlive(500);
#endif

  //  MQTT_client.setServer(eepromConfig.MQTT_server, eepromConfig.MQTT_port);
#if !Influxver
  MQTT_client.setServer("sensor.aireciudadano.com", 80);
#else
  MQTT_client.setServer("sensor.aireciudadano.com", 30183);
#endif
  MQTT_client.setCallback(Receive_Message_Cloud_App_MQTT);

  MQTT_client.connect(aireciudadano_device_id.c_str());

#if MobData
  delay(1000);
#endif

  if (!MQTT_client.connected())
  {
    err_MQTT = true;
    Serial.println("Fail, MQTT_Reconnect");
#if MobData
    delay(5000);
#endif
    MQTT_Reconnect();
  }
  else
  {
    err_MQTT = false;
    lastReconnectAttempt = 0;
    // Once connected resubscribe
    MQTT_client.subscribe(MQTT_receive_topic.c_str());
    Serial.print(F("Init MQTT connected - Receive topic: "));
    Serial.println(MQTT_receive_topic);
    FlagMQTTcon = true;
    contmqtt = 0;
#if !ESP8266
    digitalWrite(LEDPIN, HIGH);
    delay(1000);
    digitalWrite(LEDPIN, LOW);
    delay(1000);
    digitalWrite(LEDPIN, HIGH);
#else
    digitalWrite(LEDPIN, LOW);
    delay(1000);
    digitalWrite(LEDPIN, HIGH);
    delay(1000);
    digitalWrite(LEDPIN, LOW);
#endif
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
      Serial.print(F("Reconnect MQTT connected - Receive topic: "));
      Serial.println(MQTT_receive_topic);
//      FlagMQTTcon = true;
      contmqtt = 0;
#if !ESP8266
      digitalWrite(LEDPIN, HIGH);
      delay(1000);
      digitalWrite(LEDPIN, LOW);
      delay(1000);
      digitalWrite(LEDPIN, HIGH);
#else
      digitalWrite(LEDPIN, LOW);
      delay(1000);
      digitalWrite(LEDPIN, HIGH);
      delay(1000);
      digitalWrite(LEDPIN, LOW);
#endif
    }
    else
    {
#if !MobData
      err_MQTT = true;
      Serial.print(F("failed, rc="));
      Serial.print(MQTT_client.state());
      Serial.println(F(" try again in 5 seconds"));
#else
      Serial.print("TinyGSM: Retry MQQT connect ");
      Serial.println(apn);
      MQTT_client.setServer("sensor.aireciudadano.com", 80);
      MQTT_client.connect(aireciudadano_device_id.c_str());
      cont = 0;
      while (!MQTT_client.connected()) {
        cont++;
        Serial.print("TinyGSM: Retry MQQT ");
        Serial.println(cont);
        delay(5000);
        if (cont > 5)
        {
          cont = 0;
          FlagMQTTcon = true;
          break;
//          ESP.restart();    // CASO sin MQTT_client.loop();
        }
        MQTT_client.setServer("sensor.aireciudadano.com", 80);
        MQTT_client.connect(aireciudadano_device_id.c_str());
      }
      if (MQTT_client.connected())
        Serial.println("TinyGSM:  success to apn");
      else
      {
        Serial.println("TinyGSM:  no connection to apn");
        ResetMobDataConn();
      }
#endif
    }
  }
}

void Send_Message_Cloud_App_MQTT()
{ // Send measurements to the cloud application by MQTT
  // Print info
  float pm25f;
  float pm25fori;
  float pm1f;
#if TwoPMS
  float pm251f;
  float pm252f;
  float pm251fori;
  float pm252fori;
  float pm11f;
  float pm12f;
#endif
  int8_t RSSI = 0;
  int8_t inout;
#if SoundMeter
  int8_t dBAmaxint;
#endif

#if !TwoPMS
  Serial.print(F("Sending MQTT message to the send topic: "));
  Serial.println(MQTT_send_topic);
  pm25f = PM25_accumulated / PM25_samples;
  pm25int = round(pm25f);
  pm25fori = PM25_accumulated_ori / PM25_samples;
  pm25intori = round(pm25fori);
  pm1f = PM1_accumulated / PM25_samples;
  pm1int = round(pm1f);
#else
  pm251f = PM251_accumulated / PM25_samples;
  pm251int = round(pm251f);
  pm251fori = PM251_accumulated_ori / PM25_samples;
  pm251intori = round(pm251fori);
  pm252f = PM252_accumulated / PM25_samples;
  pm252int = round(pm252f);
  pm252fori = PM252_accumulated_ori / PM25_samples;
  pm252intori = round(pm252fori);
  pm25f = (pm251f + pm252f) / 2;
  pm25int = round(pm25f);
  pm25fori = (pm251fori + pm252fori) / 2;
  pm25intori = round(pm25fori);
  pm11f = PM11_accumulated / PM25_samples;
  pm11int = round(pm11f);
  pm12f = PM12_accumulated / PM25_samples;
  pm12int = round(pm12f);
  pm1f = (pm11f + pm12f) / 2;
  pm1int = round(pm1f);
#endif

#if SoundMeter
  dBAmaxint = round(dBAmax);
#else
  ReadHyT();
#endif

  if (FlagMobData == true)
  {
#if MobData
    RSSI = modem.getSignalQuality();
    Serial.print(F("Signal Quality: "));
    Serial.println(RSSI);
#endif
  }
  else
  {
    RSSI = WiFi.RSSI();
    Serial.print(F("Signal strength (RSSI): "));
    Serial.print(RSSI);
    Serial.println(F(" dBm"));
  }

  if (AmbInOutdoors)
    inout = 1;
  else
    inout = 0;

  if (SEN5Xsen == true)
  {
#if !(Rosver || SoundMeter)
    uint8_t voc;
    uint8_t nox;

    if (isnan(ambientHumidity))
    {
      if (humi == 0)
        humi = 0;
    }
    else
      humi = round(ambientHumidity);
    if (isnan(ambientTemperature))
    {
      if (temp == 0)
        temp = 0;
    }
    else
      temp = round(ambientTemperature);
    if (isnan(vocIndex))
      voc = 0;
    else
      voc = round(vocIndex);
    if (isnan(noxIndex))
      nox = 0;
    else
      nox = round(noxIndex);

    pm25intori = pm25int;   // RAW = Original

#if !Influxver
    // TEST PARA CONTAR RESET MOBDATA!!!!!!!!!!!!!!!!!
    //sprintf(MQTT_message, "{id: %s, PM25: %d, PM25raw: %d, PM1: %d, VOC: %d, NOx: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm1int, voc, nox, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId);
    if (ResetFlagMobDataTemp == true)
    {
      byte temp1 = 1;
      sprintf(MQTT_message, "{id: %s, PM25: %d, PM25raw: %d, PM1: %d, VOC: %d, NOx: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d, datavar2: %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm1int, voc, nox, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId, temp1);
      ResetFlagMobDataTemp = false;
    }
    else
    {
      byte temp1 = 0;
      sprintf(MQTT_message, "{id: %s, PM25: %d, PM25raw: %d, PM1: %d, VOC: %d, NOx: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d, datavar2: %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm1int, voc, nox, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId, temp1);
    }
#else
    sprintf(MQTT_message, "{\"id\": \"%s\", \"PM25\": %d, \"PM25raw\": %d, \"PM1\": %d, \"VOC\": %d, \"NOx\": %d, \"humidity\": %d, \"temperature\": %d, \"RSSI\": %d, \"latitude\": %f, \"longitude\": %f, \"inout\": %d, \"configval\": %d, \"datavar1\": %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm1int, voc, nox, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId); // for Telegraf
#endif
#endif
  }
  else
  {
#if !Rosver
#if !TwoPMS
#if !Influxver

    // TEST PARA CONTAR RESET MOBDATA!!!!!!!!!!!!!!!!!
    //sprintf(MQTT_message, "{id: %s, PM25: %d, PM25raw: %d, PM1: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm1int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId);

    if (ResetFlagMobDataTemp == true)
    {
      byte temp1 = 1;
      sprintf(MQTT_message, "{id: %s, PM25: %d, PM25raw: %d, PM1: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d, datavar2: %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm1int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId, temp1);
      ResetFlagMobDataTemp = false;
    }
    else
    {
      byte temp1 = 0;
      sprintf(MQTT_message, "{id: %s, PM25: %d, PM25raw: %d, PM1: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d, datavar2: %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm1int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId, temp1);
    }

#else
    sprintf(MQTT_message, "{\"id\": \"%s\", \"PM25\": %d, \"PM25raw\": %d, \"PM1\": %d, \"humidity\": %d, \"temperature\": %d, \"RSSI\": %d, \"latitude\": %f, \"longitude\": %f, \"inout\": %d, \"configval\": %d, \"datavar1\": %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm1int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId); // for Telegraf
#endif
#else
#if !Influxver
    sprintf(MQTT_message, "{id: %s, PM25: %d, PM25raw: %d, PM251: %d, PM252: %d, PM1: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm251int, pm252int, pm1int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId);
#else
    sprintf(MQTT_message, "{\"id\": \"%s\", \"PM25\": %d, \"PM25raw\": %d, \"PM251\": %d, \"PM252\": %d, \"PM1\": %d, \"humidity\": %d, \"temperature\": %d, \"RSSI\": %d, \"latitude\": %f, \"longitude\": %f, \"inout\": %d, \"configval\": %d, \"datavar1\": %d}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm251int, pm252int, pm1int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId); // for Telegraf
#endif
#endif
#else   // Rosver
#if !Influxver
    sprintf(MQTT_message, "{id: %s, PM25: %d, PM25raw: %d, PM1: %d, humidity: %d, temperature: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, datavar1: %llu}", aireciudadano_device_id.c_str(), pm25int, pm25intori, pm1int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId);
#else
    sprintf(MQTT_message, "{\"id\": \"%s\", \"PM25\": %d, \"PM25raw\": %d, \"PM1\": %d, \"humidity\": %d, \"temperature\": %d, \"RSSI\": %d, \"latitude\": %f, \"longitude\": %f, \"inout\": %d, \"configval\": %d, \"datavar1\": %llu}", pm25int, pm25intori, pm1int, humi, temp, RSSI, latitudef, longitudef, inout, IDn, chipId); // for Telegraf
#endif
#endif
#if SoundMeter
#if !Influxver
    sprintf(MQTT_message, "{id: %s, noisedba: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, noisepeak: %d, datavar1: %d}", aireciudadano_device_id.c_str(), pm25int, RSSI, latitudef, longitudef, inout, IDn, dBAmaxint, chipId);
#else
    sprintf(MQTT_message, "{\"id\": \"%s\", \"noisedba\": %d, \"RSSI\": %d, \"latitude\": %f, \"longitude\": %f, \"inout\": %d, \"configval\": %d, \"noisepeak\": %d, \"datavar1\": %d}", aireciudadano_device_id.c_str(), pm25int, RSSI, latitudef, longitudef, inout, IDn, dBAmaxint, chipId); // for Telegraf
#endif
#endif
  }
  Serial.println(MQTT_message);

#if ESP8285
  digitalWrite(LEDPIN, LOW); // turn the LED off by making the voltage LOW
  delay(750);                // wait for a 750 msecond
  digitalWrite(LEDPIN, HIGH);
#endif

  if (OLED66 == true || OLED96 == true || TDisplay == true)
    FlagDATAicon = true;

  // send message, the Print interface can be used to set the message contents
  // Publicar un mensaje MQTT

  int responsepublish = 0;
  responsepublish = MQTT_client.publish(MQTT_send_topic.c_str(), MQTT_message);
  Serial.print("response: ");
  Serial.print(responsepublish);

  // Revisar el código de retorno
  if (responsepublish == 0) {
    Serial.println(", ERROR Mensaje no publicado");
    delay(5000);
#if MobData
    ResetMobDataConn();
#endif
    responsepublish = MQTT_client.publish(MQTT_send_topic.c_str(), MQTT_message);
    if (responsepublish == 0)
      Serial.println("Reintento fallido, mensaje no publicado");
    else
      Serial.println("Reintento OK, mensaje publicado");
  }
  else {
    Serial.println(", OK Mensaje publicado");
#if !ESP8266
    digitalWrite(LEDPIN, HIGH);
#else
    digitalWrite(LEDPIN, LOW);
#endif
    FlagLED = true;
#if Influxver
    Influxseconds = 60;
    Influxsecondssam = 60;
#endif
  }
}

#if SoundAM
void Send_Message_Cloud_App_MQTTsam()
{ // Send measurements to the cloud application by MQTT
  // Print info
  float pm25f;
  int8_t RSSI;
  int8_t inout;
  int8_t dBAmaxintsam;

  Serial.print(F("Sending MQTT message to the send topic: "));
  Serial.println(MQTT_send_topicsam);
  pm25f = PM25_accumulatedsam / SP_samples;

  pm25int = round(pm25f);

  dBAmaxintsam = round(dBAmaxsam);

  RSSI = WiFi.RSSI();

  Serial.print(F("Signal strength (RSSI):"));
  Serial.print(RSSI);
  Serial.println(F(" dBm"));

  if (AmbInOutdoors)
    inout = 1;
  else
    inout = 0;

#if !Influxver
  sprintf(MQTT_message, "{id: %s, noisedba: %d, RSSI: %d, latitude: %f, longitude: %f, inout: %d, configval: %d, noisepeak: %d, datavar1: %d}", aireciudadano_device_id.c_str(), pm25int, RSSI, latitudef, longitudef, inout, IDn, dBAmaxintsam, chipId);
#else
  sprintf(MQTT_message, "{\"id\": \"%s\", \"noisedba\": %d, \"RSSI\": %d, \"latitude\": %f, \"longitude\": %f, \"inout\": %d, \"configval\": %d, \"noisepeak\": %d, \"datavar1\": %d}", aireciudadano_device_id.c_str(), pm25int, RSSI, latitudef, longitudef, inout, IDn, dBAmaxintsam, chipId); // for Telegraf
#endif

  Serial.print(MQTT_message);
  Serial.println();

#if ESP8285
  digitalWrite(LEDPIN, LOW); // turn the LED off by making the voltage LOW
  delay(750);                // wait for a 750 msecond
  digitalWrite(LEDPIN, HIGH);
#endif

  if (OLED66 == true || OLED96 == true || TDisplay == true)
    FlagDATAicon = true;

  // send message, the Print interface can be used to set the message contents
  // Publicar un mensaje MQTT

  int responsepublish = 0;
  responsepublish = MQTT_client.publish(MQTT_send_topicsam.c_str(), MQTT_message);
  Serial.print("response: ");
  Serial.println(responsepublish);

  // Revisar el código de retorno
  if (responsepublish == 0) {
    Serial.println("ERROR Mensaje no publicado");
  }
  else {
    Serial.println("OK Mensaje publicado");
#if !ESP8266
    digitalWrite(LEDPIN, HIGH);
#else
    digitalWrite(LEDPIN, LOW);
#endif
    FlagLED = true;
#if Influxver
    Influxseconds = 60;
    Influxsecondssam = 60;
#endif
  }
}
#endif

void Receive_Message_Cloud_App_MQTT(char *topic, byte *payload, unsigned int length)
{ // callback function to receive configuration messages from the cloud application by MQTT
  boolean write_eeprom = false; // to track if writing the eeprom is required
  uint16_t tempcustom = 0;
  uint64_t CustomValtotal2 = 0;
  memcpy(received_payload, payload, length);
  Serial.print(F("Message arrived: "));
  Serial.println(received_payload);

  /* POSIBLES ESPACIOS DE ENVIO DE CONFIG

    {"warning": "", "caution": "4.7835", "alarm": "ON", "name": "", "update": "OFF", "factory_reset": "OFF", "FRC": "OFF", "FRC_value": "", "MQTT_server": "", "MQTT_port": "", "ABC": "OFF", "reboot": "OFF", "temperature_offset": "-74.7654", "altitude_compensation": ""}

    "warning": ""            // Publication Time
    "caution": "4.7835"     // Latitude
    "alarm": "ON"
    "name": ""              // Update name
    "update": "OFF"         // Firmware Update
    "factory_reset": "OFF"  // Factory Reset
    "FRC": "OFF"
    "FRC_value": ""         // CustomSenHYT
    "MQTT_server": ""
    "MQTT_port": ""        // CustomOutIn
    "ABC": "OFF"
    "reboot": "OFF"
    "temperature_offset": "-74.7654"   // Longitude
    "altitude_compensation": ""        // CustomSenPM

  */

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

#if !(Rosver || SoundMeter || MinVer || MinVerSD)

  tempcustom = ((uint16_t)jsonBuffer["altitude_compensation"]);
  if (tempcustom != 0)
  {
    tempcustom = tempcustom - 1;
    Serial.print("Value customSenPM = ");
    Serial.println(tempcustom);
    CustomValtotal2 = tempcustom;
  }
  else
#endif
    CustomValtotal2 = ((int)(eepromConfig.ConfigValues[7]) - 48);

  // CustomSenHYT OR MaxWifiTX

#if !(Rosver || SoundMeter || MinVer || MinVerSD)

  tempcustom = ((uint16_t)jsonBuffer["FRC_value"]);

  if (tempcustom != 0)
  {
    tempcustom = tempcustom - 1;
    Serial.print("Value customSenHYT = ");
    Serial.println(tempcustom);
    CustomValtotal2 = CustomValtotal2 + (tempcustom * 10);
  }
  else
    CustomValtotal2 = CustomValtotal2 + ((int)eepromConfig.ConfigValues[6] - 48) * 10;
#else

  tempcustom = ((uint16_t)jsonBuffer["FRC_value"]);

  if (tempcustom != 0)
  {
    Serial.print("MaxWifiTX activated: ");
    Serial.println(tempcustom);
    CustomValtotal2 = CustomValtotal2 + (tempcustom * 10);
  }
  else
  {
    Serial.print("Normal Wifi TX: ");
    Serial.println(tempcustom);
    //CustomValtotal2 = CustomValtotal2 + ((int)eepromConfig.ConfigValues[6] - 48) * 10;
  }
#endif
  //  CustomValtotal2 = CustomValtotal2 + ((int)eepromConfig.ConfigValues[6] - 48) * 10;

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

  // CustomMobileData

#if MobData
  Serial.println(F("Mobile Data mode"));
  CustomValtotal2 = CustomValtotal2 + 100000;
#else
  Serial.println(F("NO Mobile Data mode"));
#endif

  CustomValTotalString[8] = {0};
  sprintf(CustomValTotalString, "%8lld", CustomValtotal2);
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

#if !(Rosver || MinVer || MinVerSD)
  if (CustomValtotal2 == 0)
  {
    Serial.println(F("No configuration sensor values ​​chosen, no changes will be stored"));
  }
  else
  {
#endif
    strncpy(eepromConfig.ConfigValues, CustomValTotalString, sizeof(eepromConfig.ConfigValues));
    eepromConfig.ConfigValues[sizeof(eepromConfig.ConfigValues) - 1] = '\0';
    write_eeprom = true;
    Serial.println(F("CustomVal write_eeprom = true"));
    Serial.print(F("Configuration Values: "));
    Serial.println(eepromConfig.ConfigValues);
#if !(Rosver || MinVer || MinVerSD)
  }
#endif
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

  // save the new values if the flag was set
  if (write_eeprom)
  {
    Serial.println(F("write_eeprom = true Final"));
    Write_EEPROM();
    ESP.restart();
  }
}

void Firmware_Update()
{

  // ESP32 Firmware Update

#if !ESP8266

  Serial.println(F("### FIRMWARE UPDATE ###"));

  // For remote firmware update
  WiFiClientSecure UpdateClient;
  UpdateClient.setInsecure();

  // Reading data over SSL may be slow, use an adequate timeout
  UpdateClient.setTimeout(30); // timeout argument is defined in seconds for setTimeout
  Serial.println(F("ACTUALIZACION EN CURSO"));

#if Tdisplaydisp
  // Update display
  tft.fillScreen(TFT_ORANGE);
  tft.setTextColor(TFT_BLACK, TFT_ORANGE);
  tft.setTextSize(1);
  tft.setFreeFont(FF90);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("ACTUALIZACION EN CURSO", tft.width() / 2, tft.height() / 2);
#elif (OLED66 == true || OLED96 == true)
  pageStart();
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.setCursor(0, (dh / 2 - 4));
  u8g2.print("Actualizacion");
  delay(1000);
  pageEnd();
#endif

  httpUpdate.setLedPin(LEDPIN, HIGH);

  httpUpdate.onStart(update_started);
  httpUpdate.onEnd(update_finished);
  httpUpdate.onProgress(update_progress);
  httpUpdate.onError(update_error);

#if !WPA2
#if Tdisplaydisp
  Serial.println("Firmware WITD");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WITD.bin");
#elif OLED96display
  Serial.println("Firmware WI96");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WI96.bin");
#elif OLED66display
  Serial.println("Firmware WI66");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WI66.bin");
#elif SoundMeter
#if !SoundAM
  Serial.println("Firmware WISP SoundMeter");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WISMeter.bin");
#else
  Serial.println("Firmware WISP SoundMeter AM");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WISMeteram.bin");
#endif
#elif Minver
  Serial.println("Firmware WISP MinVer");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WISPMV.bin");
#else
  Serial.println("Firmware WISP");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WISP.bin");
#endif
#else
#if Tdisplaydisp
  Serial.println("Firmware WITD WPA2");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WITDWPA2.bin");
#elif OLED96display
  Serial.println("Firmware WI96 WPA2");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WI96WPA2.bin");
#elif OLED66display
  Serial.println("Firmware WI66 WPA2");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WI66WPA2.bin");
#elif SoundMeter
  Serial.println("Firmware WISP SoundMeter");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WISMeterWPA2.bin");
#else
  Serial.println("Firmware WISP WPA2");
  t_httpUpdate_return ret = httpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/WISPWPA2.bin");
#endif
#endif

  switch (ret)
  {

    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());

#if Tdisplaydisp
      tft.fillScreen(TFT_ORANGE);
      tft.drawString("ACTUALIZACION FALLIDA", tft.width() / 2, tft.height() / 2);
#elif (OLED66 == true || OLED96 == true)
      pageStart();
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.setCursor(0, (dh / 2 - 4));
      u8g2.print("Act Fallo");
      pageEnd();
#endif
      delay(1000);
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F("HTTP_FIRMWARE_UPDATE_NO_UPDATES"));
      break;

    case HTTP_UPDATE_OK:
      Serial.println(F("HTTP_FIRMWARE_UPDATE_OK"));

#if Tdisplaydisp
      tft.fillScreen(TFT_ORANGE);
      tft.drawString("ACTUALIZACION COMPLETA", tft.width() / 2, tft.height() / 2);
#elif (OLED66 == true || OLED96 == true)
      pageStart();
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.setCursor(0, (dh / 2 - 4));
      u8g2.print("Act OK");
      pageEnd();
#endif
      delay(1000);
      break;
  }

  // ESP8266 Firmware Update

#else

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

  // Run http update
  // CAMBIAR A MAIN no en branch

#if WPA2
#if Rosver
  Serial.println("Firmware ESP8266WISP_WPA2Rosver");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/ESP8266WISP_WPA2Rosver.bin");
#else
  Serial.println("Firmware ESP8266WISP_WPA2");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/ESP8266WISP_WPA2.bin");
#endif
#else
#if Rosver
  Serial.println("Firmware ESP8266WISP_Rosver");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/ESP8266WISP_Rosver.bin");
#elif MinVer
  Serial.println("Firmware ESP8266WISP_MinVer");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/ESP8266WISPMV.bin");
#elif MinVerSD
  Serial.println("Firmware ESP8266WISP_MinVerSD");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/ESP8266WISPMVSD.bin");
#elif MobData
  Serial.println("Firmware ESP8266WISP_MobData");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/ESP8266WISPMobData.bin");
#elif SoundMeter
#if !Influxver
  Serial.println("Firmware ESP8266WISP_SoundMeter");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/ESP8266WISMeter.bin");
#else
  Serial.println("Firmware ESP8266WISP_SoundMeter_InfluxDB");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/ESP8266WISMeterInflux.bin");
#endif
#else
  Serial.println("Firmware ESP8266WISP");
  t_httpUpdate_return ret = ESPhttpUpdate.update(UpdateClient, "https://raw.githubusercontent.com/danielbernalb/AireCiudadano/main/bin/ESP8266WISP.bin");
#endif
#endif

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

#endif
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

#if MobData

void Connect_MobData()
{
  Serial.println("");
  Serial.println("Connect_MobData");
  Serial.println("TinyGSM: Wait...");
revini:

  if (FlagPoweroff == true)
  { // Power off
    SerialMon.println("MONTiny: Reinit Power off - on");
#if A7670
    SerialAT.println("AT+CPOF");
#elif SIM7070
    SerialAT.println("AT+CPOWD");
#elif SIM800
    SerialAT.println("AT+CPOWD");
#endif
  }
  else
  { // Restart
    SerialMon.println("MONTiny: Initializing modem...");
    modem.restart();
  }
  delay(15000);

  String modemInfo = modem.getModemInfo();
  SerialMon.print("MONTiny: Modem Info: ");
  SerialMon.println(modemInfo);

revini2:
  SerialMon.print("MONTiny: Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println("MONTiny: fail");
    delay(10000);
    if (modem.waitForNetwork()) {
      goto revexit;
    }
    Contacon ++;
    if (Contacon < 60)
      goto revini2;
    else
      goto revini;
  }
revexit:
  Contacon = 0;
  SerialMon.println("MONTiny: success waiting for network");

  if (modem.isNetworkConnected()) {
    SerialMon.println("MONTiny: Network connected");
  }
  else
  {
    Serial.println("TinyGSM: NO Network connected");
    delay(10);
    goto revini;
  }


  // GPRS connection parameters are usually set after network registration
  SerialMon.print("MONTiny: Connecting to ");
  SerialMon.print(apn);
  //  if (!modem.gprsConnect(apn, user1, pw1)) {
  if (!modem.gprsConnect(apn)) {
    SerialMon.println("MONTiny: fail apn");
    delay(10000);
    //    return;
    goto revini;
  }
  SerialMon.println("MONTiny:  success to apn");

  if (modem.isGprsConnected()) {
    SerialMon.println("MONTiny: GPRS connected");
  }

  // CASI SE REPITE 2 VECES !!!!!!!!!!!!!!!!
  // Make sure we're still registered on the network.
  if (!modem.isNetworkConnected()) {
    SerialMon.println("MONTiny: Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      SerialMon.println("MONTiny: fail1");
      delay(10000);
      //      return;
      goto revini;
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println("MONTiny: Network re-connected");
    }
    if (!modem.isGprsConnected()) {
      SerialMon.println("MONTiny: GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn)) {
        SerialMon.println("MONTiny: fail2");
        delay(10000);
        //        return;
        goto revini;
      }
      if (modem.isGprsConnected()) {
        SerialMon.println("MONTiny: GPRS reconnected");
      }
      Serial.println("MonTiny: NEW IP ADDRESS : " + modem.getLocalIP());
    }
  }
}

void MobDataConnected()
{
  Serial.println("MobDataConnected");
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) {
    Serial.println("TinyGSM: Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      Serial.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected())
      Serial.println("TinyGSM:: Network re-connected");
    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      Serial.print("TinyGSM: GPRS disconnected!");
      Serial.print(F("Connecting to "));
      Serial.print(apn);
      if (!modem.gprsConnect(apn)) {
        Serial.println("  fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected())
        Serial.println("TinyGSM: GPRS reconnected");
    }
  }
  else
    Serial.println("TinyGSM: Network connected ok");

  Serial.println("TinyGSM: IP ADDRESS : " + modem.getLocalIP());
  Serial.println("Signal Quality: " + modem.getSignalQuality());//  delay(5);   // TEST

  if (!MQTT_client.connected()) {
    Serial.println("TinyGSM: MQTT not connected");
    SerialMon.print("Disconnecting from: ");
    SerialMon.println("sensor.aireciudadano.com");
    MQTT_client.disconnect(); // Disconnect from MQTT  //added 08/04/2023
    delay(500);
    SerialMon.print("Network State is: ");
    SerialMon.println(modem.isGprsConnected()); //added 08/04/2023
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (!modem.isGprsConnected()) {
        // Reconnect to GPRS network if not connected
        SerialMon.println("NEW ROUTINE: GPRS not connected, reconnecting...");
        if (!modem.gprsConnect(apn)) {
          SerialMon.println("GPRS reconnect failed");
          delay(10000);
          return;
        }
      }
      if (MqttConnectok()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
  else
    Serial.println("TinyGSM: MQTT connected");
}

boolean MqttConnectok()
{
  Serial.println("MqttConnectok");
  Serial.println("Re Connecting to: sensor.aireciudadano.com");

  ResetMobDataConn();

  boolean status = MQTT_client.connect(aireciudadano_device_id.c_str());

  if (status == false) {
    Serial.println("Fail MqttConnectok");
    return false;
  }
  Serial.println("Success MqttConnectok");
  MQTT_client.subscribe(MQTT_receive_topic.c_str());
  return MQTT_client.connected();
}

void ResetMobDataConn()
{
  Serial.println("ResetMobDataCon");
  if (ResetFlagMobData == false)
  {
    if (contmqtt < 4)
    {
      Serial.println("Reset Mobile Data Connection, first Connect_MobData");
      Connect_MobData();
      delay(100);
      Serial.println("second Init MQTT routine 1");
      Init_MQTT();
      delay(100);
      contmqtt ++;
      FlagPoweroff = false;
    }
    else
    {
      Serial.println("Reset Mobile Data Connection, first Power off and Connect_MobData");
      FlagPoweroff = true;
      ResetFlagMobData = true;
      Connect_MobData();
      delay(100);
      Serial.println("second Init MQTT routine 2");
      Init_MQTT();
      delay(100);
      contmqtt = 0;
    }
  }
  else
  {
    if (contmqtt < 4)
    {
      Serial.println("Reset Mobile Data Connection, first Power off and Connect_MobData");
      FlagPoweroff = true;
      ResetFlagMobData = true;
      Connect_MobData();
      delay(100);
      Serial.println("second Init MQTT routine 3");
      Init_MQTT();
      delay(100);
      contmqtt ++;
    }
    else
    {
#if A7670
      Serial.println("Reset module A7670");
#elif SIM7070
      Serial.println("Reset module SIM7070");
#elif SIM800
      Serial.println("Reset module SIM800");
#endif
      FlagPoweroff = false;
      ResetFlagMobData = false;
      contmqtt = 0;
      ESP.restart();
    }
  }
}
#endif

#endif

#if !MobDataSP

#if (Rosver || MinVer || MobData || MinVerSD)
void Test_Sensor()
{
  byte ErrorFlag = 0;

#if (MinVer || MinVerSD)
  Serial.println(F("Test Sensirion SPS30 sensor"));
#if ESP8266
  Wire.begin();
#else
  Wire.begin(Sensor_SDA_pin, Sensor_SCL_pin);
#endif
  sps30.EnableDebugging(DEBUG);
  SP30_COMMS.begin();
  if (sps30.begin(&SP30_COMMS) == false)
    SPS30sen = false;
  // check for SPS30 connection
  if (!sps30.probe())
  {
    Serial.println(F("Not SPS30 detected"));
    SPS30sen = false;
  }
  else
  {
    Serial.println(F("Detected I2C Sensirion Sensor"));
    GetDeviceInfo();
    ErrorFlag ++;
  }
  Serial.println(F("Test Sensirion SEN5X sensor"));
  delay(10);
  sen5x.begin(Wire);
  delay(10);
  uint16_t error;
  error = sen5x.deviceReset();
  if (error)
  {
    Serial.println(F("Not SEN5X detected"));
    SEN5Xsen = false;
  }
  else
  {
    printModuleVersions();
    ErrorFlag ++;
  }
#endif
  Serial.println(F("Test Plantower Sensor"));
#if !ESP8266
  Serial1.begin(PMS::BAUD_RATE, SERIAL_8N1, PMS_TX, PMS_RX);
#else
  pmsSerial.begin(9600); // Software serial begin for PMS sensor
#endif
  delay(100);

  if (pms.readUntil(data))
  {
    Serial.println(F("Test Plantower sensor found!"));
    PMSsen = true;
    ErrorFlag ++;
    if (data.HUMI != 0)
    {
      if (!isnan(humidity))
      {
        FlagpmsHyT = true;
        Serial.println("FlagpmsHyT = true");
      }
      else
      {
        FlagpmsHyT = false;
        Serial.println("FlagpmsHyT = false");
      }
    }
  }
  else
  {
    Serial.println(F("Could not find Plantower sensor!"));
    PMSsen = false;
    FlagpmsHyT = false;
  }

  if (ErrorFlag > 0)
  {
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
  ErrorFlag = 0;
  Serial.print(F("SHT31 test: "));
  if (!sht31.begin(0x44))
  {
    Serial.println(F("no SHTsen"));
    SHTsen = false;
  }
  else
  {
    ErrorFlag = false;
    humidity = sht31.readHumidity();
    if (!isnan(humidity))
    {
      Serial.println(F("OK SHT31"));
      SHT31sen = true;
      SHTsen = true;
      ErrorFlag ++;
    }
    else
      Serial.println(F("no SHT31"));
  }
  Serial.print(F("SHT4x test: "));
  if (!sht4.begin())
    Serial.println(F("no SHT4x"));
  else
  {
    sensors_event_t humidity2, temp2;
    sht4.getEvent(&humidity2, &temp2);
    humidity = humidity2.relative_humidity;

    if (!isnan(humidity))
    { // check if 'is not a number'
      Serial.println(F("OK SHT4x"));
      SHT4xsen = true;
      SHTsen = true;
      ErrorFlag ++;
    }
    else
      Serial.println(F("no SHT4x"));
  }

  if (FlagSENHyT == true)         // Revision de sensores SEN54 y SEN55 que miden HyT
    ErrorFlag ++;

  if (FlagpmsHyT == true)         // Revision de sensor PMSx003T
    ErrorFlag ++;

  if (ErrorFlag > 0)
  {
    Serial.println(F("OK SHT3X, SHT4x, SEN54, SEN55 o PMSx003T"));
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
#endif

void Setup_Sensor()
{ // Identify and initialize PM25, temperature and humidity sensor

#if !Rosver

  // Test PM2.5 SPS30

#if !(MinVer || MinVerSD)
#if Wifi

  if (SPS30sen == true)
  {
#endif
#endif
    Serial.println(F("Test Sensirion SPS30 sensor"));

#if ESP8266
    Wire.begin();
#else
    Wire.begin(Sensor_SDA_pin, Sensor_SCL_pin);
#endif

    sps30.EnableDebugging(DEBUG);
    // Begin communication channel
    SP30_COMMS.begin();
    if (sps30.begin(&SP30_COMMS) == false)
    {
      Errorloop((char *)"Could not set I2C communication channel.", 0);
    }
    // check for SPS30 connection
    if (!sps30.probe())
      Errorloop((char *)"could not probe / connect with SPS30.", 0);
    else
    {
      Serial.println(F("Detected I2C Sensirion Sensor"));
      GetDeviceInfo();
    }
    if (SPS30sen == true)
    {
      // start measurement
      if (sps30.start())
        Serial.println(F("Measurement started"));
      else
        Errorloop((char *)"Could NOT start measurement", 0);
    }
#if !(MinVer || MinVerSD)
#if Wifi
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  // Test PM2.5 SEN5X

  if (SEN5Xsen == true)

  {
#endif
#endif
#if (Bluetooth || SDyRTC || MinVer || MinVerSD)
    if (SPS30sen == false)
    {
#endif
      Serial.println(F("Test Sensirion SEN5X sensor"));

#if ESP8266
      Wire.begin();
#else
      Wire.begin(Sensor_SDA_pin, Sensor_SCL_pin);
#endif

      delay(10);
      sen5x.begin(Wire);
      delay(10);

      uint16_t error;
      char errorMessage[256];
      error = sen5x.deviceReset();
      if (error)
      {
        Serial.print(F("Error trying to execute deviceReset(): "));
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
      }
      else
      {
        // Print SEN55 module information if i2c buffers are large enough
        Serial.println(F("SEN5X sensor found!"));
        SEN5Xsen = true;
        printSerialNumber();
        printModuleVersions();

        // Start Measurement
        error = sen5x.startMeasurement();
        if (error)
        {
          Serial.print(F("Error trying to execute startMeasurement(): "));
          errorToString(error, errorMessage, 256);
          Serial.println(errorMessage);
          // ESP.restart();
        }
        else
          Serial.println(F("SEN5X measurement OK"));
      }
#if (Bluetooth || SDyRTC || MinVer || MinVerSD)
    }
#else
    }

    ///////////////////////////////////////////////////////////////////////////////////////////

    // PMS7003 PMSA003
    if (PMSsen == true)
    {
#endif
#endif

#if SDS011sen
    Serial.println(F("Test SDS011 Sensor"));
#elif ZH10sen
    Serial.println(F("Test ZH10 Sensor"));
#else
    Serial.println(F("Test Plantower Sensor"));
#endif

#if !ESP8266

#if !TwoPMS

#if !TTGO_TQ

#if SDS011sen
	  my_sds.begin(&port);
#elif ZH10sen
    mySerial.begin(9600, SERIAL_8N1, RX_PIN_ZH10, TX_PIN_ZH10); // Comunicación con el sensor
#else
    Serial1.begin(PMS::BAUD_RATE, SERIAL_8N1, PMS_TX, PMS_RX);
#endif

#else
    Serial2.begin(PMS::BAUD_RATE, SERIAL_8N1, PMS_TX, PMS_RX);
#endif

#else
    pmsSerial1.begin(PMS::BAUD_RATE); // SoftwareSerial PMS1
    pmsSerial2.begin(PMS::BAUD_RATE); // SoftwareSerial PMS2
    delay(100);
    pms1.activeMode();
    pms2.activeMode();
    delay(100);
#endif

#else

#if !SoundMeter
#if !ESP8266SH
#if !TwoPMS
//ESP8266
#if SDS011sen
    my_sds.begin(SDS_TX, SDS_RX); // SoftwareSerial SDS011 9600 baudrate  
#else
    pmsSerial.begin(9600); // Software serial begin for PMS sensor
#endif
#else
    pmsSerial1.begin(9600); // Software serial begin for PMS1 sensor
    pmsSerial2.begin(9600); // Software serial begin for PMS2 sensor

#if PurpleVer
    pms1.activeMode();
    pms2.activeMode();
#endif

#endif
#endif
#endif
#endif

#if !(TwoPMS || SoundMeter)
    delay(1000);

#if SDS011sen
	  errSDS011 = my_sds.read(&p25, &p10);
	  if (!errSDS011) {
		  Serial.println(F("SDS011 sensor found!"));

#elif ZH10sen

    mySerial.write(command, sizeof(command)); // Enviar comando al sensor
    delay(100); // Esperar respuesta del sensor

    if (mySerial.available() >= 22) { // Se esperan 22 bytes en total
      Serial.print("Trama recibida: ");

      // Leer y mostrar la trama recibida
      for (int i = 0; i < 22; i++) {
        response[i] = mySerial.read();
        Serial.printf("%02X ", response[i]);
      }
      Serial.println();

      // Validar cabecera
      if (response[0] == 0xFF && response[1] == 0x01 && response[2] == 0x35) {
        // Extraer datos
        vocsint = (response[3] << 8) | response[4];
        pm1_0 = (response[7] << 8) | response[8];
        pm2_5 = (response[9] << 8) | response[10];
        pm10ZH10 = (response[11] << 8) | response[12];
        rawTemp = (response[13] << 8) | response[14];
        humiZH10 = (response[15] << 8) | response[16];

        // Convertir datos
        temperatureZH10 = (rawTemp - 500) / 10.0;
        vocZH10 = vocsint / 10.0;

        // Calcular checksum
        uint8_t checksum = 0;
        for (int i = 1; i < 21; i++) { // Sumar bytes del 1 al 20
          checksum += response[i];
        }
        checksum = ~checksum + 1; // Complemento a uno

        // Comparar con el byte de checksum recibido
        if (checksum == response[21]) {
          PM25_value = pm2_5;
          PM1_value = pm1_0;
          humi = humiZH10;
          temp = round(temperatureZH10);
          vocIndex = vocZH10;

        } else {
          Serial.printf("Error: Checksum inválido. Calculado: %02X, Recibido: %02X\n", checksum, response[21]);
        }
      } else {
        Serial.println("Error: Respuesta inválida (cabecera incorrecta)");
      }

#else
    if (pms.readUntil(data))
    {
      Serial.println(F("Plantower sensor found!"));
#endif

      PMSsen = true;
#if ESP8285
      digitalWrite(LEDPIN, LOW); // turn the LED off by making the voltage LOW
      delay(750);                // wait for a 750 msecond
      digitalWrite(LEDPIN, HIGH);
#endif
    }
    else
    {

#if SDS011sen
      Serial.println(F("Could not find SDS011 sensor!"));
#elif ZH10sen
      Serial.println(F("Could not find ZH10 sensor!")); 
#else
      Serial.println(F("Could not find Plantower sensor!"));
#endif

    }
#elif TwoPMS
    delay(500);

    if (pms1.readUntil(data))
    {
      Serial.println(F("Plantower1 sensor found!"));
      PMSsen = true;
#if ESP8285
      digitalWrite(LEDPIN, LOW); // turn the LED off by making the voltage LOW
      delay(750);                // wait for a 750 msecond
      digitalWrite(LEDPIN, HIGH);
#endif
    }
    else
      Serial.println(F("Could not find Plantower1 sensor!"));

    delay(500);

    if (pms2.readUntil(data))
      Serial.println(F("Plantower2 sensor found!"));
    else
      Serial.println(F("Could not find Plantower2 sensor!"));
#endif

#if !(Rosver || MinVer || MinVerSD)
#if Wifi
  }

  if (SHTsen == true)
  {
#endif
#endif

    Serial.print(F("SHT31 test: "));
    if (!sht31.begin(0x44))
      Serial.println(F("no SHT31"));
    else
    {
      humidity = sht31.readHumidity();
      if (!isnan(humidity))
      {
        Serial.println(F("OK SHT31"));
        SHT31sen = true;
        SHTsen = true;
        Serial.print(F("Heater Enabled State: "));
        if (sht31.isHeaterEnabled())
          Serial.println(F("ENABLED"));
        else
          Serial.println(F("DISABLED"));
      }
      else
        Serial.println(F("no SHT31"));
    }

    if (SHT31sen == false)
    {
      Serial.print(F("SHT4x test: "));
      if (!sht4.begin())
        Serial.println(F("no SHT4x"));
      else
      {
        sensors_event_t humidity2, temp2;
        sht4.getEvent(&humidity2, &temp2);
        humidity = humidity2.relative_humidity;

        if (!isnan(humidity))
        { // check if 'is not a number'
          Serial.println(F("OK SHT4x"));
          SHT4xsen = true;
          SHTsen = true;
          sht4.setPrecision(SHT4X_MED_PRECISION);
          Serial.println(F("SHT4x Med precision"));
          sht4.setHeater(SHT4X_NO_HEATER);
          Serial.println(F("SHT4x No heater"));
        }
        else
          Serial.println(F("no SHT4x"));
      }
    }

#if !(Rosver || MinVer || MobData || MinVerSD)
#if Wifi
  }

  if (AM2320sen == true)
  {
#endif
    Serial.print(F("AM2320 test: "));
    am2320.begin();
    delay(1);
    humidity = am2320.readHumidity();
    temperature = am2320.readTemperature();
    if (!isnan(humidity))
    {
      Serial.println(F("OK"));
      AM2320sen = true;
    }
    else
      Serial.println(F("none"));
#if Wifi
  }
#endif
#endif
}

#endif

#if !SoundMeter

void Read_Sensor()
{ // Read PM25, temperature and humidity values

#if !Rosver
  if (SPS30sen == true)
  {
    uint8_t ret, error_cnt = 0;
    struct sps_values val;
    // loop to get data
    do
    {
      ret = sps30.GetValues(&val);
      // data might not have been ready
      if (ret == SPS30_ERR_DATALENGTH)
      {
        if (error_cnt++ > 3)
        {
          ErrtoMess((char *)"Error during reading values: ", ret);
          Serial.print("Error during reading values: ");
          Serial.println(ret);
        }
      }
      // if other error
      else if (ret != SPS30_ERR_OK)
      {
        ErrtoMess((char *)"Error during reading values: ", ret);
        Serial.print("Error during reading values: ");
        Serial.println(ret);
      }
      if (failpm > 120)
      {
        failpm = 0;
        Serial.print(F("Reset lectura sensor erronea 1"));
        ESP.restart();
      }
      else
        failpm = failpm + 1;
      delay(1000);
    } while (ret != SPS30_ERR_OK);

    PM25_valueold = PM25_value;
    PM25_value = val.MassPM2;
    PM1_value = val.MassPM1;

    if (!err_sensor)
    {
      if ((PM25_value == 0 && PM1_value == 0) || (PM25_value == PM25_valueold))
      { // Recuperación de error en 0
        if (failpm > 120)
        {
          failpm = 0;
          Serial.print(F("Reset lectura sensor erronea 2"));
          ESP.restart();
        }
        else
          failpm = failpm + 1;
      }
      else
        failpm = 0;
      // Provide the sensor values for Tools -> Serial Monitor or Serial Plotter
      Serial.print(F("SPS30 PM2.5: "));
      Serial.print(PM25_value);
      Serial.print(F(" ug/m3   "));
      PM25_value_ori = PM25_value;

#if Bluetooth
      if (FlagAdjustSensor == 0){
        Serial.print("No ");
      }
      else{
        //Serial.println("Adjust PM2.5 SPS30");
        PM25_value = ((1207 * PM25_value_ori) / 1000) - 1.01; // Ajuste propuesto por paper USA, falta calibracion propia  
      }
#else
      // PM25_value = ((1280 * PM25_value_ori) / 1000) + 1.78; // Ajuste propuesto por paper USA, falta calibracion propia
      PM25_value = ((1207 * PM25_value_ori) / 1000) - 1.01; // Ajuste propuesto por paper USA, falta calibracion propia
#endif
      if (PM25_value < 0)
        PM25_value = 0;
      Serial.print(F("Adjust: "));
      Serial.print(PM25_value);
      Serial.println(F(" ug/m3"));
    }
  }
  else if (SEN5Xsen == true)
  {
    uint16_t error;
    char errorMessage[256];

    error = sen5x.readMeasuredValues(
              massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
              massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
              noxIndex);

    if (error)
    {
      Serial.print(F("Error trying to execute readMeasuredValues(): "));
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
      delay(10);
      Setup_Sensor();
      Serial.println(F("Reinit I2C"));
      delay(10);
      if (failpm > 120)
      {
        failpm = 0;
        ESP.restart();
      }
      else
        failpm = failpm + 1;
    }
    else
    {
      if (PM25_value == 0 && PM1_value == 0)
      { // Recuperación de error en 0
        if (failpm > 120)
        {
          failpm = 0;
          ESP.restart();
        }
        else
          failpm = failpm + 1;
      }
      else
        failpm = 0;
      PM25_value = massConcentrationPm2p5;
      Serial.print(F("SEN5X PM2.5: "));
      Serial.print(PM25_value);
      Serial.print(F(" ug/m3   "));
      PM1_value = massConcentrationPm1p0;
      Serial.print(F(" Humi % = "));
      if (isnan(ambientHumidity))
        Serial.print(F(" n/a"));
      else
        Serial.print(ambientHumidity);
      humi = round(ambientHumidity);

      Serial.print(F("   Temp *C = "));
      if (isnan(ambientTemperature))
        Serial.print(F(" n/a"));
      else
        Serial.print(ambientTemperature);
      temp = round(ambientTemperature);
      Serial.print(F("   VocIndex:"));
      if (isnan(vocIndex))
      {
        Serial.print(F(" n/a"));
        vocIndex = 0;
      }
      else
        Serial.print(vocIndex);
      Serial.print(F("   NoxIndex:"));
      if (isnan(noxIndex))
      {
        Serial.println(F(" n/a"));
        noxIndex = 0;
      }
      else
        Serial.println(noxIndex);
    }
  }
#endif
#if !Rosver
  else if (PMSsen == true)
#else
  if (PMSsen == true)
#endif
  {
#if !TwoPMS

#if SDS011sen
    errSDS011 = my_sds.read(&p25, &p10);
	  if (!errSDS011) 
    {
      failpm = 0;
      PM25_value = p25;
      Serial.print(F("PMS PM2.5: "));
      Serial.print(PM25_value);
      Serial.print(F(" ug/m3   "));

#elif ZH10sen

    mySerial.write(command, sizeof(command)); // Enviar comando al sensor
    delay(100); // Esperar respuesta del sensor

    if (mySerial.available() >= 22) { // Se esperan 22 bytes en total
      // Leer y mostrar la trama recibida
      for (int i = 0; i < 22; i++) {
      response[i] = mySerial.read();
    }

    // Validar cabecera
    if (response[0] == 0xFF && response[1] == 0x01 && response[2] == 0x35) {
      // Extraer datos
      vocsint = (response[3] << 8) | response[4];
      pm1_0 = (response[7] << 8) | response[8];
      pm2_5 = (response[9] << 8) | response[10];
      pm10ZH10 = (response[11] << 8) | response[12];
      rawTemp = (response[13] << 8) | response[14];
      humiZH10 = (response[15] << 8) | response[16];

      // Convertir datos
      temperatureZH10 = (rawTemp - 500) / 10.0;
      vocZH10 = vocsint / 10.0;

      // Calcular checksum
      uint8_t checksum = 0;
      for (int i = 1; i < 21; i++) { // Sumar bytes del 1 al 20
        checksum += response[i];
      }
      checksum = ~checksum + 1; // Complemento a uno

      // Comparar con el byte de checksum recibido
      if (checksum == response[21]) {
        // Mostrar datos en el monitor serie
        PM25_value = pm2_5;
        PM1_value = pm1_0;
        humi = humiZH10;
        temp = round(temperatureZH10);
        vocIndex = vocZH10;
      }
      else {
      Serial.printf("Error: Checksum inválido. Calculado: %02X, Recibido: %02X\n", checksum, response[21]);
      }
    } 
    else {
      Serial.println("Error: Respuesta inválida (cabecera incorrecta)");
    }

#else
    if (pms.readUntil(data))
    {
      failpm = 0;
      PM25_value = data.PM_AE_UG_2_5;
      Serial.print(F("PMS PM2.5: "));
      Serial.print(PM25_value);
      Serial.print(F(" ug/m3   "));
      PM1_value = data.PM_AE_UG_1_0;
#endif

      PM25_value_ori = PM25_value;

#if Bluetooth
      if (FlagAdjustSensor == 0){
        Serial.print("No ");
      }
      else{
        //Serial.println("Adjust PM2.5 PMSx003");
        // PM25_value = ((562 * PM25_value_ori) / 1000) - 1; // Ecuación de ajuste resultado de 13 intercomparaciones entre PMS7003 y SPS30 por meses
        // PM25_value = ((553 * PM25_value_ori) / 1000) + 1.3; // Segundo ajuste
        PM25_value = ((630 * PM25_value_ori) / 1000) + 1.56; // Tercer ajuste a los que salio en Lima y pruebas aqui
      }
#else

#if SDS011sen
      PM25_value = PM25_value;
#else
      // PM25_value = ((562 * PM25_value_ori) / 1000) - 1; // Ecuación de ajuste resultado de 13 intercomparaciones entre PMS7003 y SPS30 por meses
      // PM25_value = ((553 * PM25_value_ori) / 1000) + 1.3; // Segundo ajuste
      PM25_value = ((630 * PM25_value_ori) / 1000) + 1.56; // Tercer ajuste a los que salio en Lima y pruebas aqui
#endif

#endif
      if (PM25_value < 0)
        PM25_value = 0;
      Serial.print(F("Adjust: "));
      Serial.print(PM25_value);
      Serial.println(F(" ug/m3"));
    }
    else
    {

#if SDS011sen
      Serial.println(F("No data by SDS011 sensor!"));
#else
      Serial.println(F("No data by Plantower sensor!"));
#endif

      if (failpm > 120)
      {
        failpm = 0;
        ESP.restart();
      }
      else
        failpm = failpm + 1;
    }
#else
    if (pms1.readUntil(data))
    {
      failpm = 0;
      PM251_value = data.PM_AE_UG_2_5;
      Serial.print(F("PMS1 PM2.5: "));
      Serial.print(PM251_value);
      Serial.print(F(" ug/m3   "));
      PM11_value = data.PM_AE_UG_1_0;
      PM251_value_ori = PM251_value;
      PM251_value = ((630 * PM251_value_ori) / 1000) + 1.56; // Tercer ajuste a los que salio en Lima y pruebas aqui
      Serial.print(F("Adjust1: "));
      Serial.print(PM251_value);
      Serial.print(F(" ug/m3   //   "));
    }
    else
    {
      Serial.println(F("No data by Plantower1 sensor!"));
      if (failpm > 120)
      {
        failpm = 0;
        ESP.restart();
      }
      else
        failpm = failpm + 1;
    }

    if (pms2.readUntil(data))
    {
      failpm = 0;
      PM252_value = data.PM_AE_UG_2_5;
      Serial.print(F("PMS2 PM2.5: "));
      Serial.print(PM252_value);
      Serial.print(F(" ug/m3   "));
      PM12_value = data.PM_AE_UG_1_0;
      PM252_value_ori = PM252_value;
      PM252_value = ((630 * PM252_value_ori) / 1000) + 1.56; // Tercer ajuste a los que salio en Lima y pruebas aqui
      Serial.print(F("Adjust2: "));
      Serial.print(PM252_value);
      Serial.println(F(" ug/m3"));
    }
    else
    {
      Serial.println(F("No data by Plantower2 sensor!"));
      if (failpm > 120)
      {
        failpm = 0;
        ESP.restart();
      }
      else
        failpm = failpm + 1;
    }
#endif
  }
  else
    NoSensor = true;
}

#endif

#if CO2sensor
void Setup_CO2sensor()
{
  // Test Sensirion SCD30

  Serial.println(F("Test Sensirion SCD30 sensor"));

  Wire.begin(Sensor_SDA_pin, Sensor_SCL_pin);

  if (airSensor.begin(Wire, false) == false)
  {
    Serial.println("Air sensor not detected. Please check wiring");
  }
  else
  {
    Serial.println(F("SCD30 sensor found!"));
    SCD30sen = true;
    airSensor.setMeasurementInterval(2); // Change number of seconds between measurements: 2 to 1800 (30 minutes), stored in non-volatile memory of SCD30

    // While the setting is recorded, it is not immediately available to be read.
    delay(200);

    Serial.print("Auto calibration set to ");
    if (airSensor.getAutoSelfCalibration() == true)
      Serial.println("true");
    else
      Serial.println("false");

    // meters above sealevel
    airSensor.setAltitudeCompensation(SiteAltitude); // Set altitude of the sensor in m, stored in non-volatile memory of SCD30

    // Read altitude compensation value
    unsigned int altitude = airSensor.getAltitudeCompensation();
    Serial.print("Current altitude: ");
    Serial.print(altitude);
    Serial.println("m");

    // Read temperature offset
    float offset = airSensor.getTemperatureOffset();
    Serial.print("Current temp offset: ");
    Serial.print(offset, 2);
    Serial.println("C");
  }

  // Test SenseAir S8

  Serial.println(F("Test Sensirion SenseAir S8 sensor"));

  // Initialize S8 sensor
  S8_serial.begin(S8_BAUDRATE, SERIAL_8N1, PMS_TX, PMS_RX);
  sensor_S8 = new S8_UART(S8_serial);

  // Check if S8 is available
  sensor_S8->get_firmware_version(sensorS8.firm_version);
  int len = strlen(sensorS8.firm_version);
  if (len == 0)
  {
    Serial.println("SenseAir S8 CO2 sensor not found!");
    S8sen = false;
  }
  else
  {
    S8sen = true;
    // Show basic S8 sensor info
    Serial.println("SenseAir S8 sensor found!");
    printf("Firmware version: %s\n", sensorS8.firm_version);
    sensorS8.sensor_id = sensor_S8->get_sensor_ID();
    Serial.print("Sensor ID: 0x");
    printIntToHex(sensorS8.sensor_id, 4);
    Serial.println("");

    // meters above sealevel
    Serial.print("Current altitude: ");
    Serial.print(SiteAltitude);
    Serial.println("m");

    hpa = 1013 - 0.118 * SiteAltitude + 0.00000473 * SiteAltitude * SiteAltitude; // Cuadratic regresion formula obtained PA (hpa) from high above the sea
    Serial.print(F("Atmospheric pressure calculated by the sea level inserted (hPa): "));
    Serial.println(hpa);

    Serial.println("S8 Disabling ABC period");
    sensor_S8->set_ABC_period(0);
    delay(100);
    sensorS8.abc_period = sensor_S8->get_ABC_period();

    if (sensorS8.abc_period > 0)
    {
      Serial.print("ABC (automatic background calibration) period: ");
      Serial.print(sensorS8.abc_period);
      Serial.println(" hours");
    }
    else
      Serial.println("ABC (automatic calibration) is disabled");

    Serial.println("Setup done!");
  }
}

void Read_CO2sensor()
{
  if (CO2measure == false)
  {
    CO2measure = true;

    if (SCD30sen == true)
    {
      if (airSensor.dataAvailable())
      {
        PM25_value = airSensor.getCO2();
        Serial.print("CO2:");
        Serial.print(PM25_value);

        temp = round(airSensor.getTemperature());
        Serial.print(", temp:");
        Serial.print(temp);

        humi = round(airSensor.getHumidity());
        Serial.print(", humidity:");
        Serial.println(humi);
      }
    }

    if (S8sen == true)
    {
      // Get CO2 measure
      float CO2cor;
      sensorS8.co2 = sensor_S8->get_co2();
      // Adjust by altitude above the sea level
      CO2cor = sensorS8.co2 + (0.016 * ((1013 - hpa) / 10) * (sensorS8.co2 - 400)); // Increment of 1.6% for every hPa of difference at sea level
      PM25_value = round(CO2cor);
      Serial.print("CO2 orignal:");
      Serial.print(sensorS8.co2);
      Serial.print("    CO2 adjust:");
      Serial.println(PM25_value);
    }
  }
  else
    CO2measure = false;
}

#endif

#if SoundMeter

void Setup_SoundMeter()
{
  Serial.println("Config Sound Meter ESP_MEMS");
#if !ESP8266
#if !Bluetooth
  Serial2.begin(9600, SERIAL_8N1, ESP32_RX, 17);
#else
#if !Tdisplaydisp
  Serial2.begin(9600, SERIAL_8N1, ESP32_RX, 17);
#else
  Serial2.begin(9600, SERIAL_8N1, ESP32_RX, 25);    // Pin RX 33 para Bluetooth, RX no puede ser pin 17 porque así no funciona Serial2
#endif
#endif
#else
  SerialESP.begin(9600);
#endif
#if !Bluetooth
#if !(SoundAM || LTR390UV)
  Influxseconds = 60;
#endif
#endif
}

void Read_SoundMeter()
{
  // Serial.println("Read Sound Meter ESP_MEMS");
  String frame;
#if SoundAM
  byte dBActual;
#endif

#if !ESP8266
  if (Serial2.available() != 0)
  {
    frame = Serial2.readStringUntil('\n');
    PM25_value = frame.toFloat();
    if (PM25_value > 255)
      PM25_value = 255;
    if (PM25_value > dBAmax)
      dBAmax = PM25_value;
    Serial.print("SPL: ");
    Serial.print(PM25_value);
    Serial.print(" dBA    ");
    Serial.print("Max: ");
    Serial.print(dBAmax);
#if !SoundAM
    Serial.println(" dBA");
#else
    Serial.print(" dBA    ");
#endif
  }
#else
  frame = SerialESP.readStringUntil('\n');
  PM25_value = frame.toFloat();
  if (PM25_value > 255)
    PM25_value = 255;
  if (PM25_value > dBAmax)
    dBAmax = PM25_value;
  Serial.print("SPL: ");
  Serial.print(PM25_value);
  Serial.print(" dBA    ");
  Serial.print("Max: ");
  Serial.print(dBAmax);
#if !SoundAM
  Serial.println(" dBA");
#else
  Serial.print(" dBA    ");
#endif

#endif

#if SoundAM

  PM25_valuesam = PM25_value;
  if (PM25_valuesam > 255)
    PM25_valuesam = 255;
  if (PM25_valuesam > dBAmaxsam)
    dBAmaxsam = PM25_valuesam;
  Serial.print("Max AM: ");
  Serial.print(dBAmaxsam);
  Serial.println(" dBA");

  //    Serial.print("PM25_valuesam: ");
  //    Serial.print(PM25_valuesam);
  //    Serial.println(" dBA");

  if (PM25_valuesam > 70)
  {
    dBActual = 2;
    if (dBActual < Influxsecondssam)
      Influxsecondssam = 2;
  }
  else if (PM25_valuesam > 65)
  {
    dBActual = 5;
    if (dBActual < Influxsecondssam)
      Influxsecondssam = 8;
  }
  else if (PM25_valuesam > 60)
  {
    dBActual = 10;
    if (dBActual < Influxsecondssam)
      Influxsecondssam = 30;
  }
  else
  {
    dBActual = 60;
    if (dBActual < Influxsecondssam)
      Influxsecondssam = 60;
  }
  Serial.print("Influxsecondssam = ");
  Serial.println(Influxsecondssam);

#endif
}

#endif

#if LTR390UV

void Setup_UV()
{
  Wire.begin();
  if(!ltr390.init())
  {
    Serial.println("Couldn't find LTR sensor!");
  }
  else
  {
    Serial.println("Found LTR sensor!");

  ltr390.setMode(LTR390_MODE_UVS);
  if (ltr390.getMode() == LTR390_MODE_ALS) {
    Serial.println("In ALS mode");
  } else {
    Serial.println("In UVS mode");
  }

  ltr390.setGain(LTR390_GAIN_18);
  Serial.print("Gain: ");
  switch (ltr390.getGain()) {
    case LTR390_GAIN_1: Serial.println(1); break;
    case LTR390_GAIN_3: Serial.println(3); break;
    case LTR390_GAIN_6: Serial.println(6); break;
    case LTR390_GAIN_9: Serial.println(9); break;
    case LTR390_GAIN_18: Serial.println(18); break;
  }

  ltr390.setResolution(LTR390_RESOLUTION_20BIT);
  Serial.print("Resolution: ");
  switch (ltr390.getResolution()) {
    case LTR390_RESOLUTION_13BIT: Serial.println(13); break;
    case LTR390_RESOLUTION_16BIT: Serial.println(16); break;
    case LTR390_RESOLUTION_17BIT: Serial.println(17); break;
    case LTR390_RESOLUTION_18BIT: Serial.println(18); break;
    case LTR390_RESOLUTION_19BIT: Serial.println(19); break;
    case LTR390_RESOLUTION_20BIT: Serial.println(20); break;
  }
  //ltr390.setThresholds(100, 1000);
  //ltr390.configInterrupt(true, LTR390_MODE_UVS);
  }
}

void Read_UV()
{
  if (ltr390.newDataAvailable()) 
  { 
//    getUVIval = ltr390.getUVI();
    getUVIval = ltr390.getUVI();
    PM25_value = round(getUVIval);
//    PM25_value = int(getUVIval / 2.7f);
    Serial.print("UV index: ");
    Serial.print(getUVIval);
    Serial.print("   intori: ");
    Serial.println(PM25_value, 0);
    rawUVS = ltr390.readUVS();
    Serial.print("UVraw: ");
    Serial.println(rawUVS);

/*
    Serial.print("Gain: ");
    switch (ltr390.getGain()) {
      case LTR390_GAIN_1: Serial.println(1); break;
      case LTR390_GAIN_3: Serial.println(3); break;
      case LTR390_GAIN_6: Serial.println(6); break;
      case LTR390_GAIN_9: Serial.println(9); break;
      case LTR390_GAIN_18: Serial.println(18); break;
    }
    Serial.print("Resolution: ");
    switch (ltr390.getResolution()) {
      case LTR390_RESOLUTION_13BIT: Serial.println(13); break;
      case LTR390_RESOLUTION_16BIT: Serial.println(16); break;
      case LTR390_RESOLUTION_17BIT: Serial.println(17); break;
      case LTR390_RESOLUTION_18BIT: Serial.println(18); break;
      case LTR390_RESOLUTION_19BIT: Serial.println(19); break;
      case LTR390_RESOLUTION_20BIT: Serial.println(20); break;
    }
*/
  
  }
  else
    Serial.print("LTR390 not connected");
}
#endif

/**
   @brief : read and display device info
*/

#if !MobDataSP
#if !Rosver
void GetDeviceInfo()
{
  char buf[32];
  uint8_t ret;
  SPS30_version v;
  // try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == SPS30_ERR_OK)
  {
    Serial.print(F("Serial number : "));
    if (strlen(buf) > 0)
      Serial.println(buf);
    else
      Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *)"could not get serial number. ", ret);
  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == SPS30_ERR_OK)
  {
    Serial.print(F("Product name  : "));

    if (buf[7] == '0')
      if (buf[6] == '0')
        if (buf[5] == '0')
          if (buf[4] == '0')
            if (buf[3] == '8')
            {
              Serial.println(buf);
              Serial.println(F("Detected SPS30"));
              SPS30sen = true;
            }
            else
              NotAvailableSPS30();
          else
            NotAvailableSPS30();
        else
          NotAvailableSPS30();
      else
        NotAvailableSPS30();
    else
      NotAvailableSPS30();
  }
  else
    ErrtoMess((char *)"could not get product name. ", ret);
  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != SPS30_ERR_OK)
  {
    Serial.println(F("Can not read version info."));
    return;
  }

  if (SPS30sen == true)
  {
    Serial.print(F("Firmware level: "));
    Serial.print(v.major);
    Serial.print(F("."));
    Serial.println(v.minor);

    Serial.print(F("Library level : "));
    Serial.print(v.DRV_major);
    Serial.print(F("."));
    Serial.println(v.DRV_minor);
  }
}

void NotAvailableSPS30()
{
  Serial.println(F("NO SPS30"));
  SPS30sen = false;
}

void Errorloop(char *mess, uint8_t r)
{
  if (r)
    ErrtoMess(mess, r);
  else
    Serial.println(mess);
  Serial.println(F("No SPS30 connected"));
}

/**
    @brief : display error message
    @param mess : message to display
    @param r : error code

*/
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];
  Serial.print(mess);
  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}

void printModuleVersions()
{
  uint16_t error;
  char errorMessage[256];

  unsigned char productName[32];
  uint8_t productNameSize = 32;

  error = sen5x.getProductName(productName, productNameSize);

  if (error)
  {
    Serial.print(F("Error trying to execute getProductName(): "));
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
#if (MinVer || MinVerSD)
    SEN5Xsen = false;  // Sensor Sensirion SEN5X
#endif
  }
  else
  {
    Serial.print(F("ProductName: "));
    Serial.println((char *)productName);
    FlagSENHyT = false;
    if (String(((char *)productName)) == "SEN54")
      FlagSENHyT = true;
    if (String(((char *)productName)) == "SEN55")
      FlagSENHyT = true;
#if (MinVer || MinVerSD)
    SEN5Xsen = true;  // Sensor Sensirion SEN5X
#endif
  }

  if (SEN5Xsen == true)
  {
    bool firmwareDebug;
    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error)
    {
      Serial.print(F("Error trying to execute getVersion(): "));
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }
    else
    {
      Serial.print(F("Firmware: "));
      Serial.print(firmwareMajor);
      Serial.print(F("."));
      Serial.print(firmwareMinor);
      Serial.print(F(", "));

      Serial.print(F("Hardware: "));
      Serial.print(hardwareMajor);
      Serial.print(F("."));
      Serial.println(hardwareMinor);
    }
  }
}

void printSerialNumber()
{
  uint16_t error;
  char errorMessage[256];
  unsigned char serialNumber[32];
  uint8_t serialNumberSize = 32;

  error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
  if (error)
  {
    Serial.print(F("Error trying to execute getSerialNumber(): "));
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    Serial.print(F("SerialNumber: "));
    Serial.println((char *)serialNumber);
  }
}
#endif

#endif

///////////////////////////////////////////////////////////////////////////////

#if !SoundMeter

void ReadHyT()
{
  //  SHT31
  if (SHT31sen == true)
  {
    humidity = 0.0;
    temperature = 0.0;
    humidity = sht31.readHumidity();
    temperature = sht31.readTemperature();

    if (!isnan(humidity))
    { // check if 'is not a number'
      failh = 0;
      Serial.print(F("SHT31 Humi % = "));
      Serial.print(humidity);
      humi = round(humidity);
    }
    else
    {
      Serial.println(F("Failed to read humidity SHT31"));
      humi = 255;
      if (failh == 5)
      {
        failh = 0;
        sht31.begin(0x44);
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
      Serial.println(F("   Failed to read temperature SHT31"));
      temp = 255;
    }
  }

  // SHT4x
  else if (SHT4xsen == true)
  {
    temperature = 0.0;
    humidity = 0.0;

    sensors_event_t humidity2, temp2;
    sht4.getEvent(&humidity2, &temp2); // populate temp and humidity objects with fresh data
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

#if !(Rosver || MinVer || MobData || MinVerSD)
  // AM2320//
  else if (AM2320sen == true)
  {
    temperature = 0.0;
    humidity = 0.0;
    humidity = am2320.readHumidity();
    temperature = am2320.readTemperature();

    if (!isnan(humidity))
    {
      failh = 0;
      Serial.print(F("AM2320 Humi % = "));
      Serial.print(humidity);
      humi = round(humidity);
    }
    else
    {
      Serial.println(F("   Failed to read humidity AM2320"));
      if (failh == 5)
      {
        failh = 0;
        am2320.begin();
        Serial.println(F("   Reinit AM2320"));
      }
      else
        failh = failh + 1;
    }

    if (!isnan(temperature))
    {
      Serial.print(F("   Temp *C = "));
      Serial.println(temperature);
      temp = round(temperature);
    }
    else
      Serial.println(F("   Failed to read temperature AM2320"));
  }
#endif
  else if (data.HUMI != 0)
  {
    temperature = 0.0;
    humidity = 0.0;
    humidity = data.HUMI;
    temperature = data.TEMP;

    if (!isnan(humidity))
    {
      Serial.print(F("PMSx003T Humi % = "));
      Serial.print(humidity);
      humi = round(humidity);
    }

    if (!isnan(temperature))
    {
      Serial.print(F("   Temp *C = "));
      Serial.println(temperature);
      temp = round(temperature);
    }
  }
  else if (FlagSENHyT == true)
  {
    Serial.print(F("SEN5x Humi % = "));
    Serial.print(humi);
    Serial.print(F("   Temp *C = "));
    Serial.println(temp);
  }
}
#endif

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
#if Bluetooth
  Serial.print(F("Bluetooth Time: "));
  Serial.println(eepromConfig.BluetoothTime);
#endif
#if (SDyRTC || Rosver || MinVerSD)
#if !WPA2
  if (SDflag == true)
  {
    Serial.print(F("SDyRTC Time: "));
    Serial.println(SDyRTCtime);
  }
#endif
#endif
#if Wifi
#if SaveSDyRTC
  Serial.println("SDyRTC enabled: save data and date on SD Card");
#endif
  Serial.print(F("Publication Time: "));
  Serial.println(eepromConfig.PublicTime);
  Serial.print(F("Sensor latitude: "));
  Serial.println(eepromConfig.sensor_lat);
  Serial.print(F("Sensor longitude: "));
  Serial.println(eepromConfig.sensor_lon);
  Serial.print(F("Configuration values: "));
  Serial.println(eepromConfig.ConfigValues);
#if WPA2
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());
  Serial.print(F("WiFi Identity for WPA enterprise: "));
  Serial.println(eepromConfig.wifi_user);
  Serial.print(F("WiFi identity's password for WPA enterprise: "));
  Serial.println(eepromConfig.wifi_password);
#else
#if !MobData
  Serial.print(F("Wifi SSID: "));
  Serial.println(WiFi.SSID());
  Serial.print(F("Wifi password: "));
  Serial.println(wifiManager.getWiFiPass());
#else
  Serial.print(F("Mobile Data apn: "));
  Serial.println(apn);
#endif
#endif
#endif
  Serial.println(F("#######################################"));
}

#if !ESP8266
void espDelay(int ms)
{ //! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}
#endif

#if (Tdisplaydisp || OLED96display || OLED66display)

#if !ESP8266
#if Tdisplaydisp

void Button_Init()
{ // Manage TTGO T-Display board buttons

  // Buttons design:
  //   - Top button short click: info about the device
  //   - Top button long click: sleep
  //   - Bottom button short click: buttons usage
  //   - Bottom button long click: config device

  // Long clicks: keep pressing more than 2 second
  button_top.setLongClickTime(2000);
  button_bottom.setLongClickTime(2000);
  //  Serial.println(F("Button_Init");

  // Top button short click: show info about the device
  button_top.setClickHandler([](Button2 & b)
  {
    Serial.println(F("Top button short click"));
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setTextDatum(TL_DATUM); // top left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
#if Wifi
    tft.drawString("ID " + aireciudadano_device_id, 8, 5);         //!!!Arreglar por nuevo tamaño String
#elif Bluetooth
//    tft.drawString("ID MyAm 00:" + provider.getDeviceIdString(), 8, 5);         //!!!Arreglar por nuevo tamaño String
    tft.drawString("ID MyAm:" + provider.getDeviceIdString(), 8, 5);         //!!!Arreglar por nuevo tamaño String
#endif
    tft.drawString("SW ver: " + sw_version, 8, 22);
#if Bluetooth
    tft.drawString("Sam int: " + String(Bluetooth_loop_time), 8, 39);
    tft.drawString("Bluetooth ver", 8, 56);
#elif SDyRTC
    tft.drawString("SDyRTC ver", 8, 39);
    tft.drawString("Log int: " + String(SDyRTCtime), 8, 56);
#elif Wifi
    tft.drawString("Wifi ver", 8, 39);
    tft.drawString("Pubtime min: " + String(eepromConfig.PublicTime), 8, 56);
    tft.drawString("SSID " + String(WiFi.SSID()), 8, 73);
    tft.drawString("IP " + WiFi.localIP().toString(), 8, 90);
    tft.drawString("MAC " + String(WiFi.macAddress()), 8, 107);
    tft.drawString("RSSI " + String(WiFi.RSSI()), 8, 124);
#endif

    delay(5000); // keep the info in the display for 5s
    Update_Display();
  });

  // Top button long click: toggle acoustic alarm
  button_top.setLongClickDetectedHandler([](Button2 & b)
  {
    Serial.println(F("Top button long click"));

    Suspend_Device();
  });

  // Bottom button short click: show buttons info
  button_bottom.setClickHandler([](Button2 & b)
  {

///////////////
/*
    if (GainLTR390 == 1) {
        GainLTR390 = 3;
    }
    else if (GainLTR390 == 3) {
        GainLTR390 = 6;
    }
    else if (GainLTR390 == 6) {
        GainLTR390 = 9;
    }
    else if (GainLTR390 == 9) {
        GainLTR390 = 18;
    }
    else if (GainLTR390 == 18) {
        GainLTR390 = 1;
    }
   
    ltr.setGain(GainLTR390);
*/
//////////////

    Serial.println(F("Bottom button short click"));
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLUE, TFT_WHITE);
    tft.setTextDatum(TL_DATUM); // top left
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.drawString("Left button", 3, 5);
    tft.drawString("  Short: Menu", 3, 21);
#if !CO2sensor
    tft.drawString("  Long: SampTi", 3, 37);
#else
    tft.drawString("  Long: Calibr", 3, 37);
#endif
    tft.drawString("Right button", 3, 75);
    tft.drawString("  Short: Info", 3, 91);
    tft.drawString("  Long: Sleep", 3, 107);
    if (FlagAdjustSensor == 0)
      tft.drawString("NO Adjust Sensor", 3, 150);
    else
      tft.drawString("Adjust Sensor", 3, 150);
    delay(2500);
    if (digitalRead(BUTTON_TOP) == false){
      delay(100);
      if (digitalRead(BUTTON_TOP) == false){
        delay(100);
        if (digitalRead(BUTTON_TOP) == false){
          if (FlagAdjustSensor == 0){
            FlagAdjustSensor = 1;
            Serial.println("Adjust Sensor");
            tft.drawString("Adjust Sens        ", 3, 150);
          }
          else{
            FlagAdjustSensor = 0;
            Serial.println("No Adjust Sensor");
            tft.drawString("NO Adjust Sens  ", 3, 150);
          }
        }
      }
    }
    delay(2500);
    Update_Display();
  });

  // Bottom button long click: deactivate self calibration and perform sensor forced recalibration
  button_bottom.setLongClickDetectedHandler([](Button2 & b)
  {
    Serial.println(F("Bottom button long click"));
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setTextSize(1);
    tft.setFreeFont(FF90);
    tft.setTextDatum(MC_DATUM);
#if Bluetooth
#if !CO2sensor
    tft.drawString("Eval Time:", tft.width() / 2, tft.height() / 2 - 15);
    tft.drawString(String(eepromConfig.BluetoothTime) + " seg", tft.width() / 2, tft.height() / 2 + 15);
    delay(1000);
    Bluetooth_loop_time = eepromConfig.BluetoothTime;

    while (digitalRead(BUTTON_BOTTOM) == false)
    {
      if (Bluetooth_loop_time == 2)
        Bluetooth_loop_time = 10;
      else if (Bluetooth_loop_time == 10)
        Bluetooth_loop_time = 60;
      else if (Bluetooth_loop_time == 60)
        Bluetooth_loop_time = 120;
      else if (Bluetooth_loop_time == 120)
        Bluetooth_loop_time = 300;
      else if (Bluetooth_loop_time == 300)
        Bluetooth_loop_time = 600;
      else if (Bluetooth_loop_time == 600)
        Bluetooth_loop_time = 3600;
      else if (Bluetooth_loop_time == 3600)
        Bluetooth_loop_time = 10800;
      else
        Bluetooth_loop_time = 2;
      tft.drawString("                    ", tft.width() / 2, tft.height() / 2 + 15);
      tft.drawString(String(Bluetooth_loop_time) + " seg", tft.width() / 2, tft.height() / 2 + 15);
      delay(1000);
    }
    tft.drawString(String(Bluetooth_loop_time) + " seg", tft.width() / 2, tft.height() / 2 + 15);
    delay(1000);
    FlashBluetoothTime();
#else
    Serial.print("CALIBRATION:");
    for (int i = 180; i > -1; i--)
    { // loop from 0 to 180
      tft.drawString("Calib Time:", tft.width() / 2, tft.height() / 2 - 15);
      tft.drawString("                    ", tft.width() / 2, tft.height() / 2 + 15);
      tft.drawString(String(i) + " seg", tft.width() / 2, tft.height() / 2 + 15);
      delay(1000); // wait 1000 ms

      if (toggleLive == false)
      {
        if (SCD30sen == true)
        {
          pm25int = airSensor.getCO2();
          Serial.print(i);
          Serial.print(" CO2(ppm):");
          Serial.println(pm25int);
          toggleLive = true;
        }
        else if (S8sen == true)
        {
          // Get CO2 measure
          pm25int = sensor_S8->get_co2();
          Serial.print(i);
          Serial.print(" CO2(ppm):");
          Serial.println(pm25int);
          toggleLive = true;
        }
      }
      else
        toggleLive = false;
    }
    if (SCD30sen == true)
      airSensor.setForcedRecalibrationFactor(400);
    else if (S8sen == true)
      sensor_S8->manual_calibration();
    Serial.println("Resetting forced calibration factor to 400: done");
    tft.drawString("Reset calib:", tft.width() / 2, tft.height() / 2 - 15);
    tft.drawString("400 ppm", tft.width() / 2, tft.height() / 2 + 15);
    delay(5000);
#endif

#else
    tft.drawString("Reiniciando", tft.width() / 2, tft.height() / 2 - 5);
    delay(2000);
    ESP.restart();
#endif
  });
}

void Display_Init()
{ // TTGO T-Display init
  tft.init();
  tft.setRotation(0);
}

void Display_Splash_Screen()
{ // Display AireCiudadano splash screen
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, 135, 240, Icono_AireCiudadano);
}

void Update_Display()
{ // Update display

  tft.setTextDatum(TL_DATUM); // top left

  // Set screen and text colours based on PM25 value

  displayAverage(pm25int);

  if (FlagDATAicon == true)
  {
    if (pm25int < 57)
      tft.drawXBitmap(5, 194, Icono_data_on_BIG, 19, 19, TFT_BLACK);
    else
      tft.drawXBitmap(5, 194, Icono_data_on_BIG, 19, 19, TFT_WHITE);
    FlagDATAicon = false;
  }
  if (toggleLive)
  {
#if Bluetooth
#if SoundMeter
    if (pm25int < 86)
      tft.drawXBitmap(6, 192, Icono_bt_on_BIG, 19, 19, TFT_BLACK);
    else
      tft.drawXBitmap(6, 192, Icono_bt_on_BIG, 19, 19, TFT_WHITE);
#elif CO2sensor
    if (pm25int < 801)
      tft.drawXBitmap(6, 192, Icono_bt_on_BIG, 19, 19, TFT_BLACK);
    else
      tft.drawXBitmap(6, 192, Icono_bt_on_BIG, 19, 19, TFT_WHITE);
#elif LTR390UV
    if (pm25int < 8)
      tft.drawXBitmap(6, 192, Icono_bt_on_BIG, 19, 19, TFT_BLACK);
    else
      tft.drawXBitmap(6, 192, Icono_bt_on_BIG, 19, 19, TFT_WHITE);
#else
    if (pm25int < 57)
      tft.drawXBitmap(6, 192, Icono_bt_on_BIG, 19, 19, TFT_BLACK);
    else
      tft.drawXBitmap(6, 192, Icono_bt_on_BIG, 19, 19, TFT_WHITE);
#endif
#endif
  }
  toggleLive = !toggleLive;
}

#endif
#endif

#if (OLED96display || OLED66display)

void UpdateOLED()
{
  //  Serial.println(F("Sensor Read Update OLED");
  pageStart();
  displaySensorAverage(pm25int);
#if Wifi

  displaySensorData(round(PM25_value), humi, temp, WiFi.RSSI());

  if (FlagDATAicon == true)
  {
    u8g2.drawBitmap(dw - 25, dh - 7, 1, 8, Icono_data_on);
    FlagDATAicon = false;
  }
#else
  displaySensorData(round(PM25_value), humi, temp, 0);
  TimeConfig();
#endif
  if (toggleLive)
#if Bluetooth
    u8g2.drawBitmap(dw - 19, dh - 8, 1, 8, Icono_bt_on);
#else
    u8g2.drawBitmap(dw - 15, dh - 7, 1, 8, Icono_sensor_live);
#endif
  toggleLive = !toggleLive;
  pageEnd();
}
#endif
#endif

#if Bluetooth
void TimeConfig()
{
  if (digitalRead(BUTTON_BOTTOM) == false)
  {
    delay(500);
    if (digitalRead(BUTTON_BOTTOM) == false)
    {
#if !CO2sensor
#if (OLED96display || OLED66display)

      pageStart();
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, dh / 2 - 7);
      u8g2.print("Eval time:");
      u8g2.setCursor(10, dh / 2 + 7);
      u8g2.print(String(eepromConfig.BluetoothTime) + " seg");
      pageEnd();

#endif

      delay(1000);

      Bluetooth_loop_time = eepromConfig.BluetoothTime;

      while (digitalRead(BUTTON_BOTTOM) == false)
      {
        if (Bluetooth_loop_time == 2)
          Bluetooth_loop_time = 10;
        else if (Bluetooth_loop_time == 10)
          Bluetooth_loop_time = 60;
        else if (Bluetooth_loop_time == 60)
          Bluetooth_loop_time = 120;
        else if (Bluetooth_loop_time == 120)
          Bluetooth_loop_time = 300;
        else if (Bluetooth_loop_time == 300)
          Bluetooth_loop_time = 600;
        else if (Bluetooth_loop_time == 600)
          Bluetooth_loop_time = 3600;
        else if (Bluetooth_loop_time == 3600)
          Bluetooth_loop_time = 10800;
        else
          Bluetooth_loop_time = 2;

#if (OLED96display || OLED66display)

        pageStart();
        u8g2.setCursor(0, dh / 2 - 7);
        u8g2.print("Eval time");
        u8g2.setCursor(8, dh / 2 + 7);
        u8g2.print(String(Bluetooth_loop_time) + " seg");
        pageEnd();

#endif
        Serial.print(F("Evaluation time: "));
        Serial.print(Bluetooth_loop_time);
        Serial.println(F(" seg"));
        delay(1000);
      }
      FlashBluetoothTime();
#else
#if (OLED96display || OLED66display)
      u8g2.setFont(u8g2_font_6x10_tf);
#endif
      Serial.print("CALIBRATION:");
      for (int i = 180; i > -1; i--)
      { // loop from 0 to 180
#if (OLED96display || OLED66display)
        pageStart();
        u8g2.setCursor(0, dh / 2 - 7);
        u8g2.print("Calib time:");
        u8g2.setCursor(8, dh / 2 + 7);
        u8g2.print(String(i) + " seg");
        pageEnd();
#endif
        delay(1000); // wait 1000 ms

        if (toggleLive == false)
        {
          if (SCD30sen == true)
          {
            pm25int = airSensor.getCO2();
            Serial.print(i);
            Serial.print(" CO2(ppm):");
            Serial.println(pm25int);
            toggleLive = true;
          }
          else if (S8sen == true)
          {
            // Get CO2 measure
            pm25int = sensor_S8->get_co2();
            Serial.print(i);
            Serial.print(" CO2(ppm):");
            Serial.println(pm25int);
            toggleLive = true;
          }
        }
        else
          toggleLive = false;
      }
      if (SCD30sen == true)
        airSensor.setForcedRecalibrationFactor(400);
      else if (S8sen == true)
        sensor_S8->manual_calibration();
      Serial.println("Resetting forced calibration factor to 400: done");
#if (OLED96display || OLED66display)
      pageStart();
      u8g2.setCursor(0, dh / 2 - 2);
      u8g2.print("Reset calib:");
      u8g2.setCursor(8, dh / 2 + 7);
      u8g2.print("400 ppm");
      pageEnd();
#endif
      delay(5000);
#endif
    }
  }
}

void FlashBluetoothTime()
{
  //  gadgetBle.setSampleIntervalMs(Bluetooth_loop_time * 1000); // Rutina para configurar el tiempo de muestreo del sensor y la app

  if (eepromConfig.BluetoothTime != Bluetooth_loop_time)
  {
    eepromConfig.BluetoothTime = Bluetooth_loop_time;
    Serial.print(F("Bluetooth time: "));
    Serial.println(eepromConfig.BluetoothTime);
    Write_EEPROM();
  }
}

#endif

void Get_AireCiudadano_DeviceId()
{ // Get TTGO T-Display info and fill up aireciudadano_device_id with last 6 digits (in HEX) of WiFi mac address or Custom_Name + 6 digits
  char aireciudadano_device_id_endframe[10];

#if !ESP8266

  for (int i = 0; i < 17; i = i + 8)
  {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  chipIdHEX = String(chipId, HEX);
  strncpy(aireciudadano_device_id_endframe, chipIdHEX.c_str(), sizeof(aireciudadano_device_id_endframe));
#if Wifi
  Aireciudadano_Characteristics();
#endif
  Serial.printf("ESP32 Chip model = %s Rev %d.\t", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores and %dMB Flash.\n", ESP.getChipCores(), ESP.getFlashChipSize() / (1024 * 1024));

#else

#if Rosver
  String macAddress = WiFi.macAddress(); // Get the MAC address as a string
  uint64_t decimalNumber = 0;
  int shiftBits = 40; // Start from the leftmost byte

  for (int i = 0; i < 6; i++)
  {
    String hexComponent = macAddress.substring(i * 3, i * 3 + 2);
    uint64_t decimalComponent = strtol(hexComponent.c_str(), NULL, 16);
    decimalNumber |= (decimalComponent << shiftBits);
    shiftBits -= 8;
  }
  chipId = decimalNumber;
#else
  chipId = ESP.getChipId();
#endif

  chipIdHEX = String(ESP.getChipId(), HEX);
  strncpy(aireciudadano_device_id_endframe, chipIdHEX.c_str(), sizeof(aireciudadano_device_id_endframe));
#if Wifi
#if !(Rosver || MinVer || MobData || MinVerSD)
  Aireciudadano_Characteristics();
#else
  if (eepromConfig.ConfigValues[4] == '0')
  {
    SDflag = false;
    Serial.println(F("Mode: Wifi or Mobile Data"));
  }
  else
  {
    SDflag = true;
    Serial.println(F("Mode: SD & RTC"));
  }

#if MobData
  FlagMobData = true;
  Serial.println(F("Mobile Data mode"));
#else
  FlagMobData = false;
  Serial.println(F("NO Mobile Data mode"));
#endif

#endif
#endif
  Serial.print(F("ESP8266 Chip ID = "));
  Serial.print(chipIdHEX);
  Serial.print(F(", ESP CoreVersion: "));
  Serial.println(ESP.getCoreVersion());

#endif

  Serial.print(F("AireCiudadano Device ID: "));
  if (String(aireciudadano_device_id).isEmpty())
    aireciudadano_device_id = String("AireCiudadano_") + aireciudadano_device_id_endframe;
  else
    aireciudadano_device_id = String(eepromConfig.aireciudadano_device_name) + "_" + aireciudadano_device_id_endframe;
  Serial.println(aireciudadano_device_id);
}

void Aireciudadano_Characteristics()
{
#if !Bluetooth
#if !(Rosver || SoundMeter || MinVer || MobData || MinVerSD)
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

  Serial.print(F("eepromConfig.ConfigValues[5]: "));
  Serial.println(eepromConfig.ConfigValues[5]);

  TDisplay = false;
  OLED66 = false;
  OLED96 = false;
  if (eepromConfig.ConfigValues[5] == '0')
    Serial.println(F("None Display"));
  else if (eepromConfig.ConfigValues[5] == '1')
  {
    TDisplay = true;
    Serial.println(F("TTGO TDisplay board"));
  }
  else if (eepromConfig.ConfigValues[5] == '2')
  {
    OLED96 = true;
    Serial.println(F("OLED 0.96 inch display 128x64"));
  }
  else if (eepromConfig.ConfigValues[5] == '3')
  {
    OLED66 = true;
    Serial.println(F("OLED 0.66 inch display 64x48"));
  }

  Serial.print(F("eepromConfig.ConfigValues[6]: "));
  Serial.println(eepromConfig.ConfigValues[6]);
  SHTsen = false;
  AM2320sen = false;
  if (eepromConfig.ConfigValues[6] == '0')
    Serial.println(F("None sensor HYT"));
  else if (eepromConfig.ConfigValues[6] == '1')
  {
    SHTsen = true;
    Serial.println(F("SHT31/SEN4x sensor"));
  }
  else if (eepromConfig.ConfigValues[6] == '2')
  {
    AM2320sen = true;
    Serial.println(F("AM2320 sensor"));
  }

  Serial.print(F("eepromConfig.ConfigValues[7]: "));
  Serial.println(eepromConfig.ConfigValues[7]);
  SPS30sen = false;
  SEN5Xsen = false;
  PMSsen = false;
  if (eepromConfig.ConfigValues[7] == '0')
    Serial.println(F("None PM sensor"));
  else if (eepromConfig.ConfigValues[7] == '1')
  {
    SPS30sen = true;
    Serial.println(F("SPS30 sensor"));
  }
  else if (eepromConfig.ConfigValues[7] == '2')
  {
    SEN5Xsen = true;
    Serial.println(F("SEN5X sensor"));
  }
  else if (eepromConfig.ConfigValues[7] == '3')
  {
    PMSsen = true;
    Serial.println(F("PMS sensor"));
  }

  Serial.print(F("eepromConfig.ConfigValues[4]: "));
  Serial.println(eepromConfig.ConfigValues[4]);
  if (eepromConfig.ConfigValues[4] == '0')
  {
    SDflag = false;
    Serial.println(F("No SD & RTC"));
  }
  else
  {
    SDflag = true;
    Serial.println(F("SD & RTC mode"));
  }

#if !WPA2
  Serial.println(F("No WPA2"));
#else
  Serial.println(F("WPA2 security"));
#endif

#elif Rosver
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

  if (PMSsen == true)
    Serial.println(F("PMS sensor"));
  else
    Serial.println(F("No PMS sensor"));

  if (SHTsen == true)
    Serial.println(F("SHT31/SHT4x sensor"));
  else
    Serial.println("No SHT31/SHT4x sensor");

  Serial.print(F("eepromConfig.ConfigValues[4]: "));
  Serial.println(eepromConfig.ConfigValues[4]);
  if (eepromConfig.ConfigValues[4] == '0')
  {
    SDflag = false;
    Serial.println(F("No SD & RTC"));
  }
  else
  {
    SDflag = true;
    Serial.println(F("SD & RTC mode"));
  }

  Serial.print(F("eepromConfig.ConfigValues[6]: "));
  Serial.println(eepromConfig.ConfigValues[6]);
  if (eepromConfig.ConfigValues[6] == '0')
  {
    MaxWifiTX = false;
    Serial.println(F("Normal Wifi power TX"));
  }

  if (eepromConfig.ConfigValues[6] == '1')
  {
    MaxWifiTX = true;
    Serial.println(F("MaxWifiTX activated"));
  }
#if !WPA2
  Serial.println(F("No WPA2"));
#else
  Serial.println(F("WPA2 security"));
#endif

#elif MinVer

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

  Serial.print(F("eepromConfig.ConfigValues[6]: "));
  Serial.println(eepromConfig.ConfigValues[6]);
  if (eepromConfig.ConfigValues[6] == '0')
  {
    MaxWifiTX = false;
    Serial.println(F("Normal Wifi power TX"));
  }

  if (eepromConfig.ConfigValues[6] == '1')
  {
    MaxWifiTX = true;
    Serial.println(F("MaxWifiTX activated"));
  }

  if (SPS30sen == true)
    Serial.println(F("SPS30 sensor"));
  else
    Serial.println(F("No SPS30 sensor"));

  if (SEN5Xsen == true)
    Serial.println(F("SEN5X sensor"));
  else
    Serial.println(F("No SEN5X sensor"));

  if (PMSsen == true)
    Serial.println(F("PMS sensor"));
  else
    Serial.println(F("No PMS sensor"));

  if (SHTsen == true)
    Serial.println(F("SHT31/SHT4x sensor"));
  else
    Serial.println("No SHT31/SHT4x sensor");

#elif MinVerSD

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

  Serial.print(F("eepromConfig.ConfigValues[6]: "));
  Serial.println(eepromConfig.ConfigValues[6]);
  if (eepromConfig.ConfigValues[6] == '0')
  {
    MaxWifiTX = false;
    Serial.println(F("Normal Wifi power TX"));
  }

  if (eepromConfig.ConfigValues[6] == '1')
  {
    MaxWifiTX = true;
    Serial.println(F("MaxWifiTX activated"));
  }

  if (SPS30sen == true)
    Serial.println(F("SPS30 sensor"));
  else
    Serial.println(F("No SPS30 sensor"));

  if (SEN5Xsen == true)
    Serial.println(F("SEN5X sensor"));
  else
    Serial.println(F("No SEN5X sensor"));

  if (PMSsen == true)
    Serial.println(F("PMS sensor"));
  else
    Serial.println(F("No PMS sensor"));

  if (SHTsen == true)
    Serial.println(F("SHT31/SHT4x sensor"));
  else
    Serial.println("No SHT31/SHT4x sensor");

  Serial.print(F("eepromConfig.ConfigValues[4]: "));
  Serial.println(eepromConfig.ConfigValues[4]);
  if (eepromConfig.ConfigValues[4] == '0')
  {
    SDflag = false;
    Serial.println(F("No SD & RTC"));
  }
  else
  {
    SDflag = true;
    Serial.println(F("SD & RTC mode"));
  }

#elif MobData

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

  if (SPS30sen == true)
    Serial.println(F("SPS30 sensor"));
  else
    Serial.println(F("No SPS30 sensor"));

  if (SEN5Xsen == true)
    Serial.println(F("SEN5X sensor"));
  else
    Serial.println(F("No SEN5X sensor"));

  if (PMSsen == true)
    Serial.println(F("PMS sensor"));
  else
    Serial.println(F("No PMS sensor"));

  if (SHTsen == true)
    Serial.println(F("SHT31/SHT4x sensor"));
  else
    Serial.println("No SHT31/SHT4x sensor");

#if MobData
  FlagMobData = true;
  Serial.println(F("Mobile Data mode"));
#else
  FlagMobData = false;
  Serial.println(F("NO Mobile Data mode"));
#endif

#else // SoundMeter

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

  Serial.print(F("eepromConfig.ConfigValues[6]: "));
  Serial.println(eepromConfig.ConfigValues[6]);
  if (eepromConfig.ConfigValues[6] == '0')
  {
    MaxWifiTX = false;
    Serial.println(F("Normal Wifi power TX"));
  }

  if (eepromConfig.ConfigValues[6] == '1')
  {
    MaxWifiTX = true;
    Serial.println(F("MaxWifiTX activated"));
  }

  Serial.println(F("Sound Meter MEMS sensor"));

#if !WPA2
  Serial.println(F("No WPA2"));
#else
  Serial.println(F("WPA2 security"));
#endif

#endif

  // SPS30sen = 1
  // SEN5Xsen = 2
  // PMSsen = 4
  // AdjPMS = 8
  // SHTsen = 16
  // AM2320sen =32
  // SDflag = 64
  // MobData = 128
  // TDisplay = 256
  // OLED66 = 512
  // OLED96 = 1024
  // TwoSensor = 2048
  // AmbInOutdoors (Indoors) = 4096
  // WPA2 = 8192
  // ESP8266 = 16384
  // Rosver = 32768
  // Swver * 65536

  if (SPS30sen)
    IDn = IDn + 1;
  if (SEN5Xsen)
    IDn = IDn + 2;
  if (PMSsen)
    IDn = IDn + 4;
  //  if (AdjPMS)
  //    IDn = IDn + 8;
  if (SHTsen)
    IDn = IDn + 16;
  if (AM2320sen)
    IDn = IDn + 32;
  //  if (SDflag)
  //    IDn = IDn + 64;
#if SaveSDyRTC
  IDn = IDn + 64;
#endif
#if SoundMeter
  IDn = IDn + 128;
#endif
  if (TDisplay)
    IDn = IDn + 256;
  if (OLED66 || OLED96)
    IDn = IDn + 512;
  //  if (OLED96)
  //    IDn = IDn + 1024;
  if (FlagMobData)
    IDn = IDn + 1024;
#if TwoPMS
  IDn = IDn + 2048; // Solo para PMS, si se quiere para SEN5X pasarla a var y setearla en codigo
#endif
  if (AmbInOutdoors)
    IDn = IDn + 4096;
#if WPA2
  IDn = IDn + 8192;
#endif
#if ESP8266
  IDn = IDn + 16384;
#endif
#if Rosver
  IDn = IDn + 32768;
#endif
  IDn = IDn + (Swver * 65536);
  Serial.print(F("IDn: "));
  Serial.println(IDn);
#endif
}

void Read_EEPROM()
{
#if ESP8266

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

#else
  // Read AireCiudadano device persistent info
  if (preferences.getBytesLength("config") > 0)
  {
    boolean result = preferences.getBytes("config", &eepromConfig, sizeof(eepromConfig));
    if (result)
    {
      Serial.println(F("Config data read from flash"));
    }
    else
    {
      Serial.println(F("Config data could not be read from flash"));
    }
  }
  else
  {
    aireciudadano_device_id.toCharArray(eepromConfig.aireciudadano_device_name, sizeof(eepromConfig.aireciudadano_device_name)); // Initialize aireciudadano_device_name with aireciudadano_device_id
    Serial.println(F("No EEPROM data - using default config values"));
  }

#endif
}

void Write_EEPROM()
{

#if ESP8266
  // The begin() call will find the data previously saved in EEPROM if the same size
  // as was previously committed. If the size is different then the EEEPROM data is cleared.
  // Note that this is not made permanent until you call commit();
  EEPROM.begin(sizeof(MyConfigStruct));

  // set the EEPROM data ready for writing
  EEPROM.put(0, eepromConfig);

  // write the data to EEPROM
  boolean ok = EEPROM.commit();
  Serial.println((ok) ? "EEPROM Commit OK" : "EEPROM Commit failed");

#else
  // Write AireCiudadano device persistent info
  boolean result = preferences.putBytes("config", &eepromConfig, sizeof(eepromConfig));
  if (result)
  {
    Serial.println(F("Config data written to flash"));
  }
  else
  {
    Serial.println(F("Config data could not be written to flash"));
  }

#endif
}

void Wipe_EEPROM()
{ // Wipe AireCiudadano device persistent info to reset config data

#if ESP8266
  boolean result = EEPROM.wipe();
  if (result)
  {
    Serial.println(F("All EEPROM data wiped"));
  }
  else
  {
    Serial.println(F("EEPROM data could not be wiped from flash store"));
  }

#else

  boolean result = preferences.clear();
  if (result)
  {
    Serial.println(F("All EEPROM data wiped"));
  }
  else
  {
    Serial.println(F("EEPROM data could not be wiped from flash store"));
  }

#endif
}

#if Tdisplaydisp

#if Bluetooth
void displayBatteryLevel(int colour)
{ // Draw a battery showing the level of charge

  // Measure the battery voltage
  battery_voltage = ((float)analogRead(ADC_PIN) / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);

  Serial.print(F("Battery voltage: "));
  Serial.print(battery_voltage);
  Serial.println(F(" V"));

  //  tft.drawString(String(battery_voltage), 42, 218);

  // If battery voltage is up 4.5 then external power supply is working and battery is charging
  if (battery_voltage > USB_Voltage)
  {
    tft.drawRect(3, 216, 34, 22, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);
    tft.fillRect(7, 220, 5, 14, colour);
    tft.fillRect(14, 220, 5, 14, colour);
    tft.fillRect(21, 220, 5, 14, colour);
    tft.fillRect(28, 220, 5, 14, colour);
  }
  else if (battery_voltage >= Voltage_Threshold_1)
  {
    tft.drawRect(5, 218, 30, 18, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);
    tft.fillRect(7, 220, 5, 14, colour);
    tft.fillRect(14, 220, 5, 14, colour);
    tft.fillRect(21, 220, 5, 14, colour);
    tft.fillRect(28, 220, 5, 14, colour);
  }
  else if (battery_voltage >= Voltage_Threshold_2)
  {
    tft.drawRect(5, 218, 30, 18, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);
    tft.fillRect(7, 220, 5, 14, colour);
    tft.fillRect(14, 220, 5, 14, colour);
    tft.fillRect(21, 220, 5, 14, colour);
  }
  else if (battery_voltage >= Voltage_Threshold_3)
  {
    tft.drawRect(5, 218, 30, 18, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);
    tft.fillRect(7, 220, 5, 14, colour);
    tft.fillRect(14, 220, 5, 14, colour);
  }
  else if (battery_voltage >= Voltage_Threshold_4)
  {
    tft.drawRect(5, 218, 30, 18, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);
    tft.fillRect(7, 220, 5, 14, colour);
  }
  else
  {
    tft.drawRect(5, 218, 30, 18, colour);
    tft.drawRect(4, 217, 32, 20, colour);
    tft.fillRect(35, 223, 5, 8, colour);

    // Measurements are not trustable with this battery level
    Serial.println(F("Battery level too low"));
  }
}
#endif

#endif

#if !ESP8266

void Suspend_Device()
{
  if (NoiseBUTTONFlag == false)
  {
    Serial.println(F("Presiona de nuevo el boton para despertar"));
    // Off sensors
    digitalWrite(OUT_EN, LOW); // step-up off

#if LTR390UV
    digitalWrite(EnLTR390, LOW); // LTR390 off
#endif  

#if Tdisplaydisp

    if (TDisplay == true)
    {
      // #if !ESP8266
      //  int r = digitalRead(TFT_BL);
      tft.fillScreen(TFT_BLACK);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("Presiona boton", tft.width() / 2, tft.height() / 2 - 15);
      tft.drawString("para despertar", tft.width() / 2, tft.height() / 2 + 15);
      espDelay(3000);
      // digitalWrite(TFT_BL, !r);
      tft.writecommand(TFT_DISPOFF);
      tft.writecommand(TFT_SLPIN);
      // #endif
    }

#else
    espDelay(3000);
#endif

    // After using light sleep, you need to disable timer wake, because here use external IO port to wake up
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    // esp_sleep_enable_ext1_wakeup(GPIO_SEL_0, ESP_EXT1_WAKEUP_ALL_LOW);
  }
  else
    Serial.println(F("Tecla fallida, presione 1 segundo para despertar"));

  // After using light sleep, you need to disable timer wake, because here use external IO port to wake up
  //  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  // esp_sleep_enable_ext1_wakeup(GPIO_SEL_0, ESP_EXT1_WAKEUP_ALL_LOW);

  // set top button for wake up
#if !ESP32C3AG        // REVISAR!!! para el caso ESP32C3AG
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // Top button
  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0); // Bottom button

  // Antes de entrar en deep sleep:

#if LTR390UV
  digitalWrite(EnLTR390, LOW);
  gpio_deep_sleep_hold_en();  // Mantener el estado de los pines durante deep sleep
#endif

  delay(200);
  esp_deep_sleep_start();
#endif
}

void print_reset_reason(RESET_REASON reason)
{
  switch (reason)
  {
    case 1:
      Serial.println(F("POWERON_RESET"));
      ResetFlag = true;
      DeepSleepFlag = false;
      break; /**<1,  Vbat power on reset*/
    case 3:
      Serial.println(F("SW_RESET"));
      break; /**<3,  Software reset digital core*/
    case 4:
      Serial.println(F("OWDT_RESET"));
      break; /**<4,  Legacy watch dog reset digital core*/
    case 5:
      Serial.println(F("DEEPSLEEP_RESET"));
      DeepSleepFlag = true;
      ResetFlag = false;
      break; /**<5,  Deep Sleep reset digital core*/
    case 6:
      Serial.println(F("SDIO_RESET"));
      break; /**<6,  Reset by SLC module, reset digital core*/
    case 7:
      Serial.println(F("TG0WDT_SYS_RESET"));
      break; /**<7,  Timer Group0 Watch dog reset digital core*/
    case 8:
      Serial.println(F("TG1WDT_SYS_RESET"));
      break; /**<8,  Timer Group1 Watch dog reset digital core*/
    case 9:
      Serial.println(F("RTCWDT_SYS_RESET"));
      break; /**<9,  RTC Watch dog Reset digital core*/
    case 10:
      Serial.println(F("INTRUSION_RESET"));
      break; /**<10, Instrusion tested to reset CPU*/
    case 11:
      Serial.println(F("TGWDT_CPU_RESET"));
      break; /**<11, Time Group reset CPU*/
    case 12:
      Serial.println(F("SW_CPU_RESET"));
      ResetFlag = false;
      DeepSleepFlag = false;
      break; /**<12, Software reset CPU*/
    case 13:
      Serial.println(F("RTCWDT_CPU_RESET"));
      break; /**<13, RTC Watch dog Reset CPU*/
    case 14:
      Serial.println(F("EXT_CPU_RESET"));
      break; /**<14, for APP CPU, reseted by PRO CPU*/
    case 15:
      Serial.println(F("RTCWDT_BROWN_OUT_RESET"));
      break; /**<15, Reset when the vdd voltage is not stable*/
    case 16:
      Serial.println(F("RTCWDT_RTC_RESET"));
      break; /**<16, RTC Watch dog reset digital core and rtc module*/
    default:
      Serial.println(F("NO_MEAN"));
  }
}

#endif

#if (OLED96display || OLED66display)

void displayInit()
{
  u8g2.setBusClock(100000);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setContrast(255);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  u8g2.setFontMode(0);
  dw = u8g2.getDisplayWidth();
  dh = u8g2.getDisplayHeight();
  //  Serial.println(F("OLED display ready"));
}

void showWelcome()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 0, "AireCiudadano");
  u8g2.sendBuffer();
  u8g2.drawStr(0, 8, "ver: ");
  u8g2.sendBuffer();
  u8g2.drawStr(22, 8, sw_version.c_str());
  u8g2.sendBuffer();
  lastDrawedLine = 10;
  Serial.println(F("OLED display ready"));
  u8g2.sendBuffer();
}

void welcomeAddMessage(String msg)
{
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, lastDrawedLine, msg.c_str());
  lastDrawedLine = lastDrawedLine + 7;
  u8g2.sendBuffer();
}

void AddMessage(String msg)
{
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(7, lastDrawedLine, msg.c_str());
  u8g2.sendBuffer();
}

void displayCenterBig(String msg)
{
#if !CO2sensor
  if (dw > 64)
  {
    u8g2.setCursor(dw - 64, 6);
    u8g2.setFont(u8g2_font_inb24_mn);
  }
  else
  {
    u8g2.setCursor(dw - 28, 9);
    u8g2.setFont(u8g2_font_9x18B_tf);
  }
  u8g2.print(msg.c_str());

  u8g2.setCursor(100, 37);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.print("ug/m3");
#else
  if (dw > 64)
  {
    u8g2.setCursor(dw - 62, 10);
    u8g2.setFont(u8g2_font_inb19_mn);
  }
  else
  {
    u8g2.setCursor(dw - 27, 9);
    u8g2.setFont(u8g2_font_7x13B_tf);
  }
  u8g2.print(msg.c_str());

  u8g2.setCursor(100, 37);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.print("ppm");
#endif
}

void displayBottomLine(String msg)
{
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.print(msg);
}

void displayEmoticonLabel(int cursor, String msg)
{
  u8g2.setFont(u8g2_font_unifont_t_emoticons);
  u8g2.drawGlyph(76, 12, cursor);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.setCursor(77, 17);
  u8g2.print(msg);
}

void displayTextLevel(String msg)
{
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(29, 31); //(35, 26);; (25, 29); (30, 29); (29, 28); (25, 30)(30, 29)
  u8g2.print(msg);        // 4 8 7 6 7 6
}

void displayColorLevel(int cursor, String msg)
{
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.setCursor(35, 22);
  u8g2.print(msg);
}

#endif

#if !ESP8266
#if Tdisplaydisp

void displayAverage(int average)
{
  tft.setTextSize(1);

#if !(SoundMeter || CO2sensor || LTR390UV)
  if (average < 13)
  {
    tft.fillScreen(TFT_GREEN);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceGoodBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF92);
    tft.drawString("GOOD", 34, 97);
  }
  else if (average < 36)
  {
    tft.fillScreen(TFT_YELLOW);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    delay(50);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceModerateBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF90);
    tft.drawString("MODERATE", 18, 97);
  }
  else if (average < 56)
  {
    tft.fillScreen(TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceUnhealthySGroupsBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF90);
    tft.drawString("UNHEALT SEN", 5, 97);
  }
  else if (average < 151)
  {
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceUnhealthyBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("UNHEALTHY", 13, 97);
  }
  else if (average < 251)
  {
    tft.fillScreen(TFT_VIOLET);
    tft.setTextColor(TFT_WHITE, TFT_VIOLET);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceVeryUnhealthyBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("VERY UNHEAL", 5, 97);
  }
  else
  {
    tft.fillScreen(TFT_BROWN);
    tft.setTextColor(TFT_WHITE, TFT_BROWN);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceHazardousBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("HAZARDOUS", 13, 97);
  }

  // Draw PM25 number
  tft.setTextSize(1);
#if !NoxVoxTd
  tft.setFreeFont(FF95);

  if (average < 10)
    tft.drawString(String(average), 45, 116);
  else if (average < 100)
    tft.drawString(String(average), 21, 116);
  else
    tft.drawString(String(average), 0, 116);

  // Draw PM25 units
  tft.setTextSize(1);
  tft.setFreeFont(FF90);
  if (FlagAdjustSensor == 0)
    tft.drawString("PM2.5: ", 30, 197);
  else
    tft.drawString("PM2.5. ", 30, 197);
  tft.drawString(String(round(PM25_value), 0), 90, 197);

  if (temp != 0 || humi != 0)
  {
    // Draw temperature
    tft.drawString("T" + String(temp), 60, 220);

    // Draw humidity
    tft.drawString("H" + String(humi), 95, 220);
  }
  else
    tft.drawString("ug/m3", 72, 218);

#else
  tft.setFreeFont(FF97);

  if (average < 10)
    tft.drawString(String(average), 55, 115);
  else if (average < 100)
    tft.drawString(String(average), 35, 115);
  else
    tft.drawString(String(average), 18, 115);

  // Draw PM25 units
  tft.setTextSize(1);
  tft.setFreeFont(FF92);
  tft.drawString("PM2.5: ", 10, 165);
  tft.drawString(String(round(PM25_value), 0), 90, 165);
#if ZH10sen
tft.drawString("Voc: ", 34, 190);
  tft.drawString(String(vocIndex), 90, 190);
#else
  tft.drawString("Voc: ", 34, 181);
  tft.drawString(String(round(vocIndex), 0), 90, 181);
#endif
#if !ZH10sen
  tft.drawString("Nox: ", 33, 197);
  tft.drawString(String(round(noxIndex), 0), 90, 197);
#endif
  tft.setTextSize(1);
  tft.setFreeFont(FF90);

  if (temp != 0 || humi != 0)
  {
    // Draw temperature
    tft.drawString("T" + String(temp), 60, 220);

    // Draw humidity
    tft.drawString("H" + String(humi), 95, 220);
  }
  else
    tft.drawString("ug/m3", 72, 218);
#endif

#if Wifi
  int rssi;
  rssi = WiFi.RSSI();

  if (rssi != 0)
  {
    if (pm25int < 57)
      tft.drawXBitmap(5, 215, Icono_wifi_on_BIG, 20, 20, TFT_BLACK);
    else
      tft.drawXBitmap(5, 215, Icono_wifi_on_BIG, 20, 20, TFT_WHITE);
    Serial.print(F(" RSSI: "));
    Serial.print(rssi);
    rssi = rssi + 130;
    Serial.print(F("  norm: "));
    Serial.println(rssi);
    tft.drawString(String(rssi), 30, 220);
  }
#endif

#elif CO2sensor

  if (average < 600)
  {
    tft.fillScreen(TFT_GREEN);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceGoodBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF92);
    tft.drawString("GOOD", 34, 100);
  }
  else if (average < 800)
  {
    tft.fillScreen(TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
    delay(50);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceModerateBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF90);
    tft.drawString("MODERATE", 18, 100);
  }
  else if (average < 1000)
  {
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceUnhealthyBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("UNHEALTHY", 13, 100);
  }
  else if (average < 1400)
  {
    tft.fillScreen(TFT_VIOLET);
    tft.setTextColor(TFT_WHITE, TFT_VIOLET);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceVeryUnhealthyBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("VERY UNHEAL", 5, 100);
  }
  else
  {
    tft.fillScreen(TFT_BROWN);
    tft.setTextColor(TFT_WHITE, TFT_BROWN);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceHazardousBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("HAZARDOUS", 13, 100);
  }

  // Draw CO2 number
  tft.setTextSize(1);
  tft.setFreeFont(FF97); // CO2 grande

  if (average == 0)
    tft.drawString(String(average), 52, 132);
  else if (average < 1000)
    tft.drawString(String(average), 18, 132);
  else
    tft.drawString(String(average), 2, 132);

  // Draw PM25 units
  tft.setTextSize(1);
  tft.setFreeFont(FF90);
  tft.drawString("CO2: ", 30, 197);
  tft.drawString(String(round(PM25_value), 0), 80, 197);

  if (temp != 0 || humi != 0)
  {
    // Draw temperature
    tft.drawString("T" + String(temp), 60, 220);

    // Draw humidity
    tft.drawString("H" + String(humi), 95, 220);
  }
  else
    tft.drawString("ppm", 72, 218);

#if Wifi
  int rssi;
  rssi = WiFi.RSSI();

  if (rssi != 0)
  {
    if (pm25int < 57)
      tft.drawXBitmap(5, 215, Icono_wifi_on_BIG, 20, 20, TFT_BLACK);
    else
      tft.drawXBitmap(5, 215, Icono_wifi_on_BIG, 20, 20, TFT_WHITE);
    Serial.print(F(" RSSI: "));
    Serial.print(rssi);
    rssi = rssi + 130;
    Serial.print(F("  norm: "));
    Serial.println(rssi);
    tft.drawString(String(rssi), 30, 220);
  }
#endif

#elif LTR390UV

  if (average < 3)
  {
    tft.fillScreen(TFT_GREEN);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceGoodBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF92);
    tft.drawString("GOOD", 34, 97);
  }
  else if (average < 6)
  {
    tft.fillScreen(TFT_YELLOW);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    delay(50);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceModerateBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF90);
    tft.drawString("MODERATE", 18, 97);
  }
  else if (average < 8)
  {
    tft.fillScreen(TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceUnhealthySGroupsBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF92);
    tft.drawString("HIGH", 34, 97);
  }
  else if (average < 11)
  {
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceUnhealthyBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("VERY HIGH", 13, 97);
  }
  else
  {
    tft.fillScreen(TFT_VIOLET);
    tft.setTextColor(TFT_WHITE, TFT_VIOLET);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceVeryUnhealthyBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("EXTREME", 23, 97);
  }

  // Draw PM25 number
  tft.setTextSize(1);
  tft.setFreeFont(FF95);

  if (average < 10)
    tft.drawString(String(average), 45, 116);
  else if (average < 100)
    tft.drawString(String(average), 21, 116);
  else
    tft.drawString(String(average), 0, 116);

  // Draw PM25 units
  tft.setTextSize(1);
  tft.setFreeFont(FF90);
  tft.drawString("UVindex", 25, 197);
  tft.drawString(String(getUVIval, 1), 99, 197);
//  tft.drawString("r: " + String(ltr.uv()), 60, 220);
  tft.drawString("r: " + String(rawUVS), 58, 220);

#if Wifi
  int rssi;
  rssi = WiFi.RSSI();

  if (rssi != 0)
  {
    if (pm25int < 8)
      tft.drawXBitmap(5, 215, Icono_wifi_on_BIG, 20, 20, TFT_BLACK);
    else
      tft.drawXBitmap(5, 215, Icono_wifi_on_BIG, 20, 20, TFT_WHITE);
    Serial.print(F(" RSSI: "));
    Serial.print(rssi);
    rssi = rssi + 130;
    Serial.print(F("  norm: "));
    Serial.println(rssi);
    tft.drawString(String(rssi), 30, 220);
  }
#endif

#else
  if (average < 55)
  {
    tft.fillScreen(TFT_GREEN);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceGoodBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF92);
    tft.drawString("GOOD", 34, 97);
  }
  else if (average < 70)
  {
    tft.fillScreen(TFT_YELLOW);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    delay(50);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceModerateBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF90);
    tft.drawString("MODERATE", 18, 97);
  }
  else if (average < 85)
  {
    tft.fillScreen(TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
#if Bluetooth
    displayBatteryLevel(TFT_BLACK);
#endif
    tft.drawXBitmap(27, 10, SmileFaceUnhealthySGroupsBig, 80, 80, TFT_BLACK);
    tft.setFreeFont(FF92);
    tft.drawString("LOUD", 37, 97);
  }
  else if (average < 95)
  {
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceUnhealthyBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("UNHEALTHY", 13, 97);
  }
  else if (average < 105)
  {
    tft.fillScreen(TFT_VIOLET);
    tft.setTextColor(TFT_WHITE, TFT_VIOLET);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceVeryUnhealthyBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("VERY UNHEAL", 5, 97);
  }
  else
  {
    tft.fillScreen(TFT_BROWN);
    tft.setTextColor(TFT_WHITE, TFT_BROWN);
#if Bluetooth
    displayBatteryLevel(TFT_WHITE);
#endif
    tft.drawXBitmap(27, 10, SmileFaceHazardousBig, 80, 80, TFT_WHITE);
    tft.setFreeFont(FF90);
    tft.drawString("HAZARDOUS", 13, 97);
  }

  // Draw PM25 number
  tft.setTextSize(1);
  tft.setFreeFont(FF95);

  if (average < 10)
    tft.drawString(String(average), 45, 116);
  else if (average < 100)
    tft.drawString(String(average), 21, 116);
  else
    tft.drawString(String(average), 0, 116);

  // Draw PM25 units
  tft.setTextSize(1);
  tft.setFreeFont(FF90);
  tft.drawString("SPL: ", 40, 197);
  tft.drawString(String(round(PM25_value), 0), 93, 197);

  tft.drawString("dBA", 85, 218);

#if Wifi
  int rssi;
  rssi = WiFi.RSSI();

  if (rssi != 0)
  {
    if (pm25int < 57)
      tft.drawXBitmap(5, 215, Icono_wifi_on_BIG, 20, 20, TFT_BLACK);
    else
      tft.drawXBitmap(5, 215, Icono_wifi_on_BIG, 20, 20, TFT_WHITE);
    Serial.print(F(" RSSI: "));
    Serial.print(rssi);
    rssi = rssi + 130;
    Serial.print(F("  norm: "));
    Serial.println(rssi);
    tft.drawString(String(rssi), 30, 220);
  }
#endif

#endif
}
#endif
#endif

#if (OLED96display || OLED66display)

void displaySensorAverage(int average)
{
#if !CO2sensor
  if (average < 13)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceGood);
    displayColorLevel(0, " green");
    displayTextLevel("  GOOD");
  }
  else if (average < 36)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceModerate);
    displayColorLevel(0, "yellow");
    displayTextLevel("MODERATE");
  }
  else if (average < 56)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceUnhealthySGroups);
    displayColorLevel(0, "orange");
    displayTextLevel("UNH SEN");
  }
  else if (average < 151)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceUnhealthy);
    displayColorLevel(0, "  red");
    displayTextLevel("UNHEALT");
  }
  else if (average < 251)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceVeryUnhealthy);
    displayColorLevel(0, "violet");
    displayTextLevel("V UNHEA");
  }
  else
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceHazardous);
    displayColorLevel(0, " brown");
    displayTextLevel(" HAZARD");
  }
#else
  if (average < 600)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceGood);
    displayColorLevel(0, " green");
    displayTextLevel("  GOOD");
  }
  else if (average < 800)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceModerate);
    displayColorLevel(0, "yellow");
    displayTextLevel("MODERATE");
  }
  else if (average < 1000)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceUnhealthy);
    displayColorLevel(0, "  red");
    displayTextLevel("UNHEALT");
  }
  else if (average < 1400)
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceVeryUnhealthy);
    displayColorLevel(0, "violet");
    displayTextLevel("V UNHEA");
  }
  else
  {
    u8g2.drawXBM(1, 5, 32, 32, SmileFaceHazardous);
    displayColorLevel(0, " brown");
    displayTextLevel(" HAZARD");
  }
#endif
  char output[4];
  sprintf(output, "%03d", average);
  displayCenterBig(output);
}

// TODO: separate this function, format/display
void displaySensorData(int pm25, int humi, int temp, int rssi)
{
  char output[22];
  sprintf(output, "%03d H%02d T%02d", pm25, humi, temp); // 000 E00 H00% T00°C
  u8g2.setCursor(dw / 2 + 16, 48);
  displayBottomLine(String(output));

  u8g2.setFont(u8g2_font_4x6_tf);
#if !CO2sensor
  u8g2.setCursor(43, 1);
#else
  u8g2.setCursor(42, 1);
#endif
  sprintf(output, "%04d", pm25); // PM25 instantaneo fuente pequeña
  u8g2.print(output);

#if Wifi
  u8g2.setCursor(20, dh - 6);

  if (rssi == 0)
  {
    u8g2.print("   ");
    Serial.println(F(""));
  }
  else
  {
    u8g2.drawBitmap(5, dh - 8, 1, 8, Icono_wifi_on);
    Serial.print(F(" RSSI: "));
    Serial.print(rssi);
    //    rssi = abs(rssi);
    sprintf(output, "%02d", rssi);
    rssi = rssi + 130;
    Serial.print(F("  norm: "));
    Serial.println(rssi);
    u8g2.print(rssi);
  }
#endif
}

void pageStart()
{
  u8g2.firstPage();
}

void pageEnd()
{
  u8g2.nextPage();
}

#endif

#if LedNeo

void LedNeoAverage(int average)
{
  if (average < 13)
  {
    LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(0, 255, 0))); // Encendemos el led verde
    Serial.println("Led RGB Verde");
  }
  else if (average < 36)
  {
    LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(180, 180, 0))); // Encendemos el led amarillo
    Serial.println("Led RGB Amarillo");
  }
  else if (average < 56)
  {
    LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(255, 125, 0))); // Encendemos el led naranja
    Serial.println("Led RGB Naranja");
  }
  else if (average < 151)
  {
    LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(255, 0, 0))); // Encendemos el led rojo
    Serial.println("Led RGB Rojo");
  }
  else if (average < 251)
  {
    LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(128, 0, 255))); // Encendemos el led morado
    Serial.println("Led RGB Morado");
  }
  else
  {
    LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(128, 128, 128))); // Encendemos el led blanco bajo brillo
    Serial.println("Led RGB Cafe = Blanco");
  }
  LED_RGB.show(); // Enciende el color
}
#endif

#if Bluetooth
void Write_Bluetooth()
{ // Write measurements to Bluetooth

#if SoundMeter
  provider.writeValueToCurrentSample(pm25int, SignalType::PM2P5_MICRO_GRAMM_PER_CUBIC_METER);
  provider.commitSample(Bluetooth_loop_time);
  Serial.print("Bluetooth frame SPL(dBA): ");
  Serial.println(pm25int);
#elif LTR390UV
  provider.writeValueToCurrentSample(pm25int, SignalType::PM2P5_MICRO_GRAMM_PER_CUBIC_METER);
  provider.commitSample(Bluetooth_loop_time);
  Serial.print("Bluetooth frame UV Index: ");
  Serial.println(pm25int);
#elif CO2sensor
  provider.writeValueToCurrentSample(temp, SignalType::TEMPERATURE_DEGREES_CELSIUS);
  provider.writeValueToCurrentSample(humi, SignalType::RELATIVE_HUMIDITY_PERCENTAGE);
  provider.writeValueToCurrentSample(pm25int, SignalType::CO2_PARTS_PER_MILLION);
  provider.commitSample(Bluetooth_loop_time);
  Serial.print("Bluetooth frame CO2(ppm): ");
  Serial.print(pm25int);
  Serial.print(", temp(°C): ");
  Serial.print(temp);
  Serial.print(", humidity(%): ");
  Serial.println(humi);
#elif NoxVoxTd
#if ZH10sen
  uint16_t vocint = vocIndex;
#else
  uint16_t vocint = round(vocIndex);
#endif
  uint16_t noxint = round(noxIndex);
  provider.writeValueToCurrentSample(temp, SignalType::TEMPERATURE_DEGREES_CELSIUS);
  provider.writeValueToCurrentSample(humi, SignalType::RELATIVE_HUMIDITY_PERCENTAGE);
  provider.writeValueToCurrentSample(vocZH10, SignalType::VOC_INDEX);
  provider.writeValueToCurrentSample(noxint, SignalType::NOX_INDEX);
  provider.writeValueToCurrentSample(pm25int, SignalType::PM2P5_MICRO_GRAMM_PER_CUBIC_METER);
  provider.commitSample(Bluetooth_loop_time);
  Serial.print("Bluetooth frame PM2.5(ug/m3): ");
  Serial.print(pm25int);
#if ZH10sen
  Serial.print(", Voc(ppb): ");
  Serial.print(vocZH10);
#else
  Serial.print(", Voc Index: ");
  Serial.print(vocint);
  Serial.print(", Nox index: ");
  Serial.print(noxint);
#endif
  Serial.print(", temp(°C): ");
  Serial.print(temp);
  Serial.print(", humidity(%): ");
  Serial.println(humi);
#else
  if (SHT31sen == true || SHT4xsen == true || AM2320sen == true || data.HUMI != 0 || FlagSENHyT == true)
  {
    provider.writeValueToCurrentSample(pm25int, SignalType::PM2P5_MICRO_GRAMM_PER_CUBIC_METER);
    provider.writeValueToCurrentSample(temp, SignalType::TEMPERATURE_DEGREES_CELSIUS);
    provider.writeValueToCurrentSample(humi, SignalType::RELATIVE_HUMIDITY_PERCENTAGE);
    provider.commitSample(Bluetooth_loop_time);
    Serial.print("Bluetooth frame PM2.5(ug/m3): ");
    Serial.print(pm25int);
    Serial.print(", temp(°C): ");
    Serial.print(temp);
    Serial.print(", humidity(%): ");
    Serial.println(humi);
  }
  else
  {
    provider.writeValueToCurrentSample(pm25int, SignalType::PM2P5_MICRO_GRAMM_PER_CUBIC_METER);
    provider.writeValueToCurrentSample(0, SignalType::TEMPERATURE_DEGREES_CELSIUS);
    provider.writeValueToCurrentSample(0, SignalType::RELATIVE_HUMIDITY_PERCENTAGE);
    provider.commitSample(Bluetooth_loop_time);
    Serial.print("Bluetooth frame PM2.5(ug/m3): ");
    Serial.println(pm25int);
  }
#endif
}
#endif

#if (SDyRTC || SaveSDyRTC || Rosver || MinVerSD)
#if !WPA2
void Write_SD()
{ // Write date - time and measurements to SD card

  DateTime now = rtc.now();

  char buf1[] = "YYYY/MM/DD_hh:mm:ss";
  String dataString = "";

  // make a string for assembling the data to log:

  dataString = now.toString(buf1);
  dataString += "_";
  dataString += pm25int;
  if (SHT31sen == true || SHT4xsen == true || AM2320sen == true || data.HUMI != 0 || FlagSENHyT == true)
  {
    dataString += "_";
    dataString += humi;
    dataString += "_";
    dataString += temp;
  }

  Serial.print(F("SDreset: "));
  Serial.println(SDreset);
  SDreset = SDreset - 1;

  if (SDreset == 0)
    dataString += "_RESET";

  // dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile = SD.open(aireciudadano_device_id + ".txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile)
  {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.print(F("SD write: "));
    Serial.println(dataString);
    // LED Routine
    digitalWrite(LEDPIN, LOW);
    FlagLED = true;
  }
  // if the file isn't open, pop up an error:
  else
  {
    Serial.println("SD write: error opening file");
  }

  if (SDreset == 0)
  {
    Serial.print(F("SD reset cycles: "));
    Serial.println(ValSDreset);
    ESP.restart();
  }
}
#endif
#endif
