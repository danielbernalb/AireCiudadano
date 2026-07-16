/**
 * @file utils.h
 * @brief Funciones utilitarias del proyecto AireCiudadano
 * 
 * Este archivo contiene funciones de uso general:
 * - Información del dispositivo
 * - Actualización de firmware
 * - Manejo de errores
 * - Utilidades varias
 */

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
// INFORMACIÓN DEL DISPOSITIVO
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Obtiene ID único del dispositivo AireCiudadano
 * 
 * El ID se genera a partir de la dirección MAC WiFi
 */
void Get_AireCiudadano_DeviceId();

/**
 * @brief Imprime razón del reset del ESP32
 * @param reason Código de razón del reset
 */
void print_reset_reason(RESET_REASON reason);

///////////////////////////////////////////////////////////////////////////////////////////////////
// ACTUALIZACIÓN DE FIRMWARE (OTA)
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Inicia proceso de actualización de firmware OTA
 * 
 * Permite actualizar firmware por WiFi sin conexión física
 */
void Firmware_Update();

/**
 * @brief Callback: inicio de actualización
 */
void update_started();

/**
 * @brief Callback: finalización de actualización
 */
void update_finished();

/**
 * @brief Callback: progreso de actualización
 * @param cur Bytes actualizados
 * @param total Total de bytes
 */
void update_progress(int cur, int total);

/**
 * @brief Callback: error en actualización
 * @param err Código de error
 */
void update_error(int err);

///////////////////////////////////////////////////////////////////////////////////////////////////
// MANEJO DE ERRORES
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Maneja loop de error crítico
 * @param mess Mensaje de error
 * @param r Código de error para blink
 * 
 * Esta función detiene el dispositivo y muestra error
 */
void Errorloop(char *mess, uint8_t r);

/**
 * @brief Convierte código de error a mensaje legible
 * @param mess Buffer para mensaje
 * @param r Código de error
 */
void ErrtoMess(char *mess, uint8_t r);

///////////////////////////////////////////////////////////////////////////////////////////////////
// UTILIDADES GENERALES
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Genera delay compatible con watchdog
 * @param ms Milisegundos de delay
 * 
 * Usa espDelay() en lugar de delay() para evitar reset
 */
void espDelay(int ms);

/**
 * @brief Configura tiempo y fecha
 * 
 * Sincroniza con servidor NTP
 */
void TimeConfig();

/**
 * @brief Suspende dispositivo (deep sleep)
 * 
 * Ahorra energía apagando componentes
 */
void Suspend_Device();

/**
 * @brief Muestra estado de conexión WiFi
 */
void Print_WiFi_Status();

/**
 * @brief Verifica evento WiFi
 * @param event Evento WiFi ocurrido
 */
void WiFiEvent(WiFiEvent_t event);

///////////////////////////////////////////////////////////////////////////////////////////////////
// LED NEOPIXEL
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Calcula promedio para LED NeoPixel
 * @param average Valor promedio
 */
void LedNeoAverage(int average);

///////////////////////////////////////////////////////////////////////////////////////////////////
// INTERRUPCIONES
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Interrupción para contador de lluvia
 * 
 * Se ejecuta en cada pulso del pluviómetro
 */
void IRAM_ATTR contarPulso();

#endif // UTILS_H
