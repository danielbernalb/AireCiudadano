/**
 * @file wifi.h
 * @brief Funciones para gestión de WiFi
 * 
 * Este archivo contiene todas las funciones relacionadas con:
 * - Conexión WiFi
 * - Portal cautivo
 * - Servidor web
 */

#ifndef WIFI_H
#define WIFI_H

#include <Arduino.h>

/**
 * @brief Verifica el estado de la conexión WiFi e imprime información
 */
void Print_WiFi_Status();

/**
 * @brief Verifica el estado de la conexión WiFi (ESP8266)
 */
void Print_WiFi_Status_ESP8266();

/**
 * @brief Intenta conectar a la red WiFi configurada
 * 
 * Esta función:
 * 1. Lee las credenciales de EEPROM
 * 2. Intenta conectar usando WiFi.begin()
 * 3. Espera hasta establecer conexión o timeout
 * 4. Activa portal cautivo si falla la conexión
 */
void Connect_WiFi();

/**
 * @brief Inicia y verifica la conexión con el servidor HTTP
 * 
 * Se usa cuando se accede por navegador a la IP del dispositivo
 */
void Check_WiFi_Server();

/**
 * @brief Inicia el portal cautivo para configuración WiFi
 * 
 * El portal cautivo permite:
 * - Configurar SSID y contraseña WiFi
 * - Configurar nombre del dispositivo
 * - Configurar coordenadas GPS
 * - Ajustar parámetros de publicación
 */
void Start_Captive_Portal();

/**
 * @brief Obtiene un parámetro del formulario web
 * @param name Nombre del parámetro
 * @return Valor del parámetro como String
 */
String getParam(String name);

/**
 * @brief Obtiene un parámetro del formulario web como string
 * @param name Nombre del parámetro
 * @return Valor del parámetro
 */
String getParamstring(String name);

/**
 * @brief Callback para guardar parámetros del portal cautivo
 * 
 * Se ejecuta cuando el usuario guarda la configuración en el portal
 */
void saveParamCallback();

#endif // WIFI_H
