/**
 * @file mobile.h
 * @brief Funciones para comunicación de datos móviles (SIM)
 * 
 * Este archivo contiene todas las funciones relacionadas con:
 * - Conexión a red celular (2G/3G/4G)
 * - Envío de datos por GPRS/LTE
 * - Gestión de módulos GSM (A7670, SIM7070, SIM800)
 */

#ifndef MOBILE_H
#define MOBILE_H

#include <Arduino.h>

/**
 * @brief Inicia conexión de datos móviles
 * 
 * Esta función:
 * 1. Inicializa módulo GSM
 * 2. Configura APN del operador
 * 3. Establece conexión de datos
 * 4. Verifica señal y registro en red
 */
void Connect_MobData();

/**
 * @brief Maneja evento de conexión móvil exitosa
 * 
 * Se ejecuta cuando se establece conexión con la red celular
 */
void MobDataConnected();

/**
 * @brief Reset de conexión de datos móviles
 * 
 * Reinicia módulo GSM y reintenta conexión
 */
void ResetMobDataConn();

#endif // MOBILE_H
