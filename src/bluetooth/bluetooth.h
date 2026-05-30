/**
 * @file bluetooth.h
 * @brief Funciones para comunicación Bluetooth
 * 
 * Este archivo contiene todas las funciones relacionadas con:
 * - Conexión Bluetooth BLE
 * - Envío de datos por Bluetooth
 * - Características GATT
 */

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>

/**
 * @brief Inicializa botones para control Bluetooth
 * 
 * Configura botones físicos para:
 * - Encender/apagar Bluetooth
 * - Iniciar envío de datos
 */
void Button_Init();

/**
 * @scribe Envía datos por Bluetooth
 * 
 * Esta función:
 * 1. Verifica conexión Bluetooth
 * 2. Empaqueta datos de sensores
 * 3. Envía a dispositivo conectado (celular/tablet)
 */
void Write_Bluetooth();

/**
 * @brief Configura características del dispositivo AireCiudadano
 * 
 * Define servicios y características BLE:
 * - Servicio de medición
 * - Servicio de configuración
 * - UUIDs personalizados
 */
void Aireciudadano_Characteristics();

#endif // BLUETOOTH_H
