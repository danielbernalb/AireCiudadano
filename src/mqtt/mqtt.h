/**
 * @file mqtt.h
 * @brief Funciones para gestión de comunicación MQTT
 * 
 * Este archivo contiene todas las funciones relacionadas con:
 * - Conexión al broker MQTT
 * - Publicación de mensajes
 * - Recepción de comandos
 * - Reconexión automática
 */

#ifndef MQTT_H
#define MQTT_H

#include <Arduino.h>

/**
 * @brief Inicializa el cliente MQTT
 * 
 * Configura:
 * - Servidor y puerto MQTT
 * - Callback de recepción
 * - Parámetros de conexión
 */
void Init_MQTT();

/**
 * @brief Intenta reconectar al broker MQTT
 * 
 * Esta función:
 * 1. Verifica si hay conexión WiFi
 * 2. Intenta conectar con credenciales almacenadas
 * 3. Suscribe a tópicos de recepción
 * 4. Maneja errores de conexión
 */
void MQTT_Reconnect();

/**
 * @brief Envía mensaje de medición al servidor MQTT
 * 
 * Publica los valores de:
 * - PM2.5, PM1
 * - Temperatura y humedad
 * - Coordenadas GPS
 * - Estado del dispositivo
 */
void Send_Message_Cloud_App_MQTT();

/**
 * @brief Envía mensaje de medición (versión SoundAM)
 * 
 * Similar a Send_Message_Cloud_App_MQTT pero para modo avión
 */
void Send_Message_Cloud_App_MQTTsam();

/**
 * @brief Procesa mensajes recibidos del servidor MQTT
 * 
 * @param topic Tópico recibido
 * @param payload Datos del mensaje
 * @param length Longitud del payload
 * 
 * Comandos soportados:
 * - Cambio de intervalo de publicación
 * - Reset del dispositivo
 * - Actualización de configuración
 */
void Receive_Message_Cloud_App_MQTT(char *topic, byte *payload, unsigned int length);

/**
 * @brief Verifica si MQTT está conectado
 * @return true si está conectado, false en caso contrario
 */
boolean MqttConnectok();

#endif // MQTT_H
