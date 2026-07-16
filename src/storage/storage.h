/**
 * @file storage.h
 * @brief Funciones para almacenamiento de datos
 * 
 * Este archivo contiene todas las funciones relacionadas con:
 * - EEPROM (configuración)
 * - Tarjeta SD (guardado de datos)
 * - RTC (reloj tiempo real)
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
// EEPROM - ALMACENAMIENTO DE CONFIGURACIÓN
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Lee configuración almacenada en EEPROM
 * 
 * Recupera:
 * - Nombre del dispositivo
 * - Coordenadas GPS
 * - Parámetros de publicación
 * - Credenciales WiFi (si aplica)
 */
void Read_EEPROM();

/**
 * @brief Escribe configuración actual en EEPROM
 * 
 * Guarda todos los parámetros configurables
 */
void Write_EEPROM();

/**
 * @brief Borra toda la configuración de EEPROM
 * 
 * Restaura valores por defecto
 * ¡Usar con cuidado!
 */
void Wipe_EEPROM();

/**
 * @brief Actualiza tiempo de Bluetooth en EEPROM
 */
void FlashBluetoothTime();

///////////////////////////////////////////////////////////////////////////////////////////////////
// TARJETA SD - GUARDADO DE DATOS
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Escribe medición actual en tarjeta SD
 * 
 * Guarda en archivo CSV:
 * - Fecha y hora (si hay RTC)
 * - Valores de PM2.5, PM1
 * - Temperatura y humedad
 * - Otros sensores activos
 */
void Write_SD();

/**
 * @brief Ajusta hora del RTC
 * 
 * Sincroniza con servidor NTP o GPS
 */
void RTCadjustTime();

///////////////////////////////////////////////////////////////////////////////////////////////////
// UTILIDADES DE IMPRESIÓN
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Imprime configuración actual por serial
 * 
 * Útil para diagnóstico y debugging
 */
void Print_Config();

/**
 * @brief Imprime versiones de módulos
 * 
 * Muestra información de firmware y librerías
 */
void printModuleVersions();

/**
 * @brief Imprime número de serie del dispositivo
 */
void printSerialNumber();

#endif // STORAGE_H
