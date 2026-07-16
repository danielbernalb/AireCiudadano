/**
 * @file types.h
 * @brief Definición de tipos y variables globales del proyecto AireCiudadano
 * 
 * Este archivo contiene todas las variables globales y estructuras de configuración.
 * Está organizado por categorías para facilitar su comprensión.
 */

#ifndef TYPES_H
#define TYPES_H

#include "config.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
// VARIABLES DE ESTADO DEL SISTEMA
///////////////////////////////////////////////////////////////////////////////////////////////////

extern bool SPS30sen;      // Sensor Sensirion SPS30 activo
extern bool SEN5Xsen;      // Sensor Sensirion SEN5X activo
extern bool PMSsen;        // Sensor Plantower PMS activo
extern bool SHTsen;        // Sensor SHT31/SHT4x activo
extern bool SHT31sen;      // Sensor SHT31 activo
extern bool SHT4xsen;      // Sensor SHT4x activo
extern bool AM2320sen;     // Sensor AM2320 activo
extern bool SCD30sen;      // Sensor CO2 SCD30 activo
extern bool S8sen;         // Sensor CO2 SenseAir S8 activo
extern bool TDisplay;      // Usando TTGO T-Display
extern bool OLED66;        // Usando OLED 0.66"
extern bool OLED96;        // Usando OLED 0.96"
extern bool AmbInOutdoors; // Midiendo en interiores (false) o exteriores (true)
extern bool SDflag;        // Tarjeta SD activa
extern bool FlagMobData;   // Datos móviles activos
extern bool FlagSENHyT;    // Sensor SEN con HyT
extern bool FlagpmsHyT;    // Sensor PMS con HyT
extern bool FlagMQTTcon;   // MQTT conectado
extern bool FlagPoweroff;  // Apagado pendiente
extern bool MaxWifiTX;     // Máxima potencia WiFi
extern bool FlagAdjustSensor; // Ajuste de sensor activo

extern uint8_t Contacon;   // Contador de conexiones
extern uint8_t CustomValue;
extern uint32_t CustomValtotal;
extern char CustomValTotalString[9];
extern uint32_t IDn;
extern String chipIdHEX;

#if Rosver
extern uint64_t chipId;
#else
extern uint32_t chipId;
#endif

extern String sw_version;                    // Versión del firmware
extern String aireciudadano_device_id;       // ID único del dispositivo
extern uint8_t Swver;

///////////////////////////////////////////////////////////////////////////////////////////////////
// ESTRUCTURA DE CONFIGURACIÓN (EEPROM)
///////////////////////////////////////////////////////////////////////////////////////////////////

struct MyConfigStruct
{
#if Bluetooth
  #if CO2sensor
    uint16_t BluetoothTime = 10;
  #elif (SoundMeter || LTR390UV)
    uint16_t BluetoothTime = 2;
  #else
    uint16_t BluetoothTime = 10;
  #endif
  char aireciudadano_device_name[30];
#elif Wifi
  #if !MobData
    uint16_t PublicTime = 1;
  #else
    uint16_t PublicTime = 2;
  #endif
  
  #if !PreProgSensor
    char sensor_lat[10] = "0.0";
    char sensor_lon[10] = "0.0";
    char ConfigValues[10] = "000100000";
    char aireciudadano_device_name[30];
  #else
    char sensor_lat[10] = "4.6987";
    char sensor_lon[10] = "-74.0987";
    char ConfigValues[10] = "000100000";
    char aireciudadano_device_name[30] = "AireCiudadano_Test01";
  #endif
#endif

#if (WPA2 || Rosver)
  char wifi_user[24];
  char wifi_password[24];
#endif
};

extern MyConfigStruct eepromConfig;
extern char wifi_passwpa2[24];
extern bool ConfigPortalSave;

///////////////////////////////////////////////////////////////////////////////////////////////////
// VARIABLES DE MEDICIÓN - PARTÍCULAS (PM)
///////////////////////////////////////////////////////////////////////////////////////////////////

extern float PM25_value;         // Valor medido PM2.5
extern float PM25_valueold;      // Valor anterior PM2.5
extern float PM251_value;        // Valor sensor 1 PM2.5
extern float PM252_value;        // Valor sensor 2 PM2.5
extern float PM25_value_ori;     // Valor original PM2.5 (sin ajuste)
extern float PM251_value_ori;    
extern float PM252_value_ori;    
extern float PM25_valuesam;      // Valor firmware SoundAM
extern float PM25_accumulated;   // Acumulado para período MQTT
extern float PM251_accumulated;  
extern float PM252_accumulated;  
extern float PM25_accumulated_ori;
extern float PM251_accumulated_ori;
extern float PM252_accumulated_ori;
extern float PM25_accumulatedsam;

extern float PM1_value;          // Valor medido PM1
extern float PM11_value;         
extern float PM12_value;         
extern float PM1_accumulated;    
extern float PM11_accumulated;   
extern float PM12_accumulated;   

extern int pm25int;              // PM2.5 publicado (entero)
extern int pm25intori;
extern int pm251int;
extern int pm252int;
extern int pm251intori;
extern int pm252intori;
extern int pm1int;
extern int pm11int;
extern int pm12int;

extern int PM25_samples;         // Contador de muestras para período MQTT
extern int SP_samples;

///////////////////////////////////////////////////////////////////////////////////////////////////
// VARIABLES DE MEDICIÓN - AMBIENTALES
///////////////////////////////////////////////////////////////////////////////////////////////////

extern float temperature;        // Temperatura en Celsius
extern float humidity;           // Humedad en %
extern int temp;                 // Temperatura entera
extern int humi;                 // Humedad entera

extern float latitudef;          // Latitud GPS
extern float longitudef;         // Longitud GPS

///////////////////////////////////////////////////////////////////////////////////////////////////
// VARIABLES DE MEDICIÓN - SONIDO
///////////////////////////////////////////////////////////////////////////////////////////////////

extern float dBAmax;             // Máximo dBA
extern float dBAmaxsam;          // Máximo dBA (muestreo)

///////////////////////////////////////////////////////////////////////////////////////////////////
// VARIABLES DE ESTADO Y ERROR
///////////////////////////////////////////////////////////////////////////////////////////////////

extern bool err_wifi;            // Error de WiFi
extern bool err_MQTT;            // Error de MQTT
extern bool err_sensor;          // Error de sensor
extern bool FlagDATAicon;        // Icono de datos
extern bool NoSensor;            // Sin sensor detectado

extern bool MQTT_toggle;
extern bool MQTT_token;

///////////////////////////////////////////////////////////////////////////////////////////////////
// TEMPORIZADORES Y CONTROL DE TIEMPO
///////////////////////////////////////////////////////////////////////////////////////////////////

extern unsigned int measurements_loop_duration;  // Duración del loop de medición (ms)
extern unsigned long measurements_loop_start;    // Timestamp inicio loop medición

extern unsigned int Bluetooth_loop_time;
extern unsigned int Con_loop_times;
extern unsigned int SDyRTC_loop_time;

extern unsigned long MQTT_loop_start;            // Timestamp inicio loop MQTT
extern unsigned long MQTT_loop_startsam;
extern unsigned long MQTT_loop_review;
extern unsigned int MQTT_loop_review_duration;   // Duración review MQTT (ms)
extern unsigned long lastReconnectAttempt;       // Último intento de reconexión MQTT

extern unsigned int errors_loop_duration;        // Duración loop de errores (ms)
extern unsigned long errors_loop_start;          // Timestamp inicio loop errores

extern byte cont;

///////////////////////////////////////////////////////////////////////////////////////////////////
// VARIABLES DE PANTALLA
///////////////////////////////////////////////////////////////////////////////////////////////////

extern unsigned int mcount, ecode;
extern int lastDrawedLine;
extern unsigned int inthumi;
extern unsigned int inttemp;
extern unsigned int cursor;
extern bool toggleLive;
extern int dw;  // Ancho de pantalla
extern int dh;  // Alto de pantalla

///////////////////////////////////////////////////////////////////////////////////////////////////
// VARIABLES ESPECÍFICAS POR SENSOR
///////////////////////////////////////////////////////////////////////////////////////////////////

// --- SPS30 ---
extern float massConcentrationPm1p0;
extern float massConcentrationPm2p5;
extern float massConcentrationPm4p0;
extern float massConcentrationPm10p0;
extern float ambientHumidity;
extern float ambientTemperature;
extern float vocIndex;
extern float noxIndex;

// --- SDS011 ---
extern float p10, p25;
extern int errSDS011;

// --- ZH10 ---
extern float temperatureZH10;
extern float vocZH10;
extern uint16_t vocsint;
extern uint16_t pm1_0;
extern uint16_t pm2_5;
extern uint16_t pm10ZH10;
extern uint16_t rawTemp;
extern uint16_t humiZH10;

// --- CO2 ---
extern bool AM2320flag;
extern bool CO2measure;
extern float hpa;

// --- WiFi ---
extern bool PortalFlag;
extern int wifi_status;

// --- MQTT ---
extern String MQTT_send_topic;
extern String MQTT_send_topicsam;
extern String MQTT_receive_topic;

// --- Reset y Power ---
extern bool ResetFlag;
extern bool DeepSleepFlag;
extern bool NoiseBUTTONFlag;
extern bool ResetFlagMobData;
extern bool ResetFlagMobDataTemp;

// --- Firmware Update ---
extern bool updating;
extern bool InCaptivePortal;
extern bool Calibrating;

// --- SD y RTC ---
extern uint16_t SDyRTCtime;
extern uint16_t SDreset;

// --- Fecha y Hora ---
extern String Valdate_time_id;

// --- LED NeoPixel ---
extern bool FlagLED;

// --- UV ---
extern float getUVIval;
extern uint32_t rawUVS;

// --- Lluvia ---
extern float lluvia1min;
extern float lluviaTotal;

// --- Inclinómetro ---
extern float sum_x, sum_y, sum_z;

// --- Acelerómetro LSM9DS1 ---
extern float sum_ax, sum_ay, sum_az;
extern float sum_mx, sum_my, sum_mz;

// --- Nivel ultrasónico ---
extern int conteoLecturas;
extern bool esperandoRespuesta;
extern int bytesLeidos;
extern int distance;
extern int historico[];
extern int indiceHistorico;
extern int countHistorico;
extern int ultimaLecturaRechazada;
extern int vecesRechazadaSimilar;

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONSTANTES
///////////////////////////////////////////////////////////////////////////////////////////////////

#define VENTANA_HISTORICO 10  // Tamaño de ventana histórica para nivel

#endif // TYPES_H
