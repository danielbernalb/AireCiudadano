/**
 * @file config.h
 * @brief Configuración principal del proyecto AireCiudadano
 * 
 * Este archivo contiene todas las definiciones y banderas (flags) de configuración
 * del firmware. Modifica estos valores según tu hardware y necesidades.
 * 
 * Para voluntarios: Solo necesitas cambiar true/false en las secciones marcadas
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONFIGURACIÓN DE COMUNICACIONES
///////////////////////////////////////////////////////////////////////////////////////////////////

// --- WiFi ---
#define Wifi true        // true: activa WiFi, false: desactiva
#define WPA2 false       // true: para redes empresariales WPA2, false: redes normales

// --- Bluetooth ---
#define Bluetooth false  // true: activa Bluetooth (requiere WiFi en false)

// --- Tarjeta SD y Reloj Tiempo Real (RTC) ---
#define SDyRTC false     // true: activa SD y RTC (requiere WiFi y Bluetooth en false)
#define SaveSDyRTC false // true: guarda datos en SD incluso con WiFi o Bluetooth activo

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONFIGURACIÓN DE SENSORES DE PARTÍCULAS (PM)
///////////////////////////////////////////////////////////////////////////////////////////////////

// --- Selección de sensor de partículas ---
#define TwoPMS false     // true: usa 2 sensores PMS7003, false: usa 1 solo
#define ZH10sen false    // true: usa sensor ZH10 en lugar de PMS
#define SDS011sen false  // true: usa sensor SDS011 en lugar de PMS
#define NoxVoxTd false   // true: lectura de sensor NoxVox

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONFIGURACIÓN DE SENSORES AMBIENTALES
///////////////////////////////////////////////////////////////////////////////////////////////////

// --- Sensores especiales ---
#define Influxver true   // true: versión para InfluxDB (SP - Rain - Incli - Nivel)
#define SoundMeter true  // true: activa medidor de sonido
#define SoundAM false    // true: modo avión para medidor de sonido
#define Rain false       // true: activa pluviómetro (medidor de lluvia)
#define Incli false      // true: activa inclinómetros
#define ADXL false       // true: usa acelerómetro ADXL345
#define LSM9 false       // true: usa sensor LSM9DS1
#define Nivel false      // true: activa medidores de nivel
#define NivPin false     // true: medidor de nivel ultrasónico por pines (JSN-SR04M)
#define NivSer false     // true: medidor de nivel ultrasónico serial (JSN-SR04M)
#define Niv485 false     // true: medidor de nivel RS485 (SeedStudio)

// --- Otros sensores ---
#define LTR390UV false   // true: sensor UV LTR390 (solo ESP32)
#define LedNeo false     // true: LED NeoPixel multicolor
#define Relay false      // true: usa relé para sensor móvil

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONFIGURACIÓN DE PANTALLA
///////////////////////////////////////////////////////////////////////////////////////////////////

// Activa SOLO UNA de las siguientes opciones (o ninguna si no hay pantalla):
#define Tdisplaydisp false    // true: TTGO T Display
#define OLED66display false   // true: Pantalla OLED 0.66"
#define OLED96display false   // true: Pantalla OLED 0.96"

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONFIGURACIÓN DE DATOS MÓVILES (SIM CARD)
///////////////////////////////////////////////////////////////////////////////////////////////////

// --- Operador de telefonía móvil (activar solo uno) ---
#define TigoKalleyExito false
#define MovistarVirgin false
#define Claro false
#define Wom false

// --- Tipo de placa SIM (activar solo uno) ---
#define A7670 false
#define SIM7070 false
#define SIM800 false

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONFIGURACIÓN DE SENSOR CO2
///////////////////////////////////////////////////////////////////////////////////////////////////

#define CO2sensor false       // true: activa sensores de CO2 (SCD30 o SenseAir S8)
#define SiteAltitude 0        // IMPORTANTE: altitud del sitio en metros sobre el nivel del mar
                              // Ejemplo: Bogotá = 2600m
//#define SiteAltitude 2600   // Descomentar y ajustar para Bogotá

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONFIGURACIÓN DE PLACAS ESPECIALES
///////////////////////////////////////////////////////////////////////////////////////////////////

#define TTGO_TQ false        // true: placa TTGO T-Q

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONFIGURACIONES ADICIONALES
///////////////////////////////////////////////////////////////////////////////////////////////////

#define BrownoutOFF false    // true: desactiva brownout (para placas con problemas de voltaje)
#define ESP8266SH false      // true: PMS en pin 0 - Hardware Serial (ESP8266)
#define PreProgSensor false  // true: usa variables de sensor preprogramadas

// Si PreProgSensor es true, configura aquí:
// Latitude: char sensor_lat[10] = "xx.xxxx";
// Longitude: char sensor_lon[10] = "xx.xxxx";
// ConfigValues: char ConfigValues[9] = "000xxxxx";
// DeviceName: char aireciudadano_device_name[36] = "xxxxxxxxxxxxxx";

///////////////////////////////////////////////////////////////////////////////////////////////////
// DEFINICIÓN DE VERSIONES DE PLATAFORMA
// (No modificar - se definen desde platformio.ini)
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef ESP32S3def
#define ESP32S3 true
#else
#define ESP32S3 false
#endif

#ifdef ESP32C3AGdef
#define ESP32C3AG true
#else
#define ESP32C3AG false
#endif

#ifdef ESP8285def
#define ESP8285 true
#else
#define ESP8285 false
#endif

#ifdef Rosverdef
#define Rosver true
#else
#define Rosver false
#endif

#ifdef MinVerdef
#define MinVer true
#else
#define MinVer false
#endif

#ifdef MobDatadef
#define MobData true
#else
#define MobData false
#endif

#ifdef MinVerSDdef
#define MinVerSD true
#else
#define MinVerSD false
#endif

#ifdef MobDataSPver
#define MobDataSP true
#else
#define MobDataSP false
#endif

#endif // CONFIG_H
