/**
 * @file display.h
 * @brief Funciones para gestión de pantallas
 * 
 * Este archivo contiene todas las funciones relacionadas con:
 * - Inicialización de pantallas (OLED, TTGO)
 * - Mostrar información de medición
 * - Animaciones y splash screens
 * - Actualización de datos en tiempo real
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
// INICIALIZACIÓN Y CONFIGURACIÓN
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Inicializa la pantalla configurada
 * 
 * Detecta y configura:
 * - TTGO T-Display
 * - OLED 0.66"
 * - OLED 0.96"
 */
void Display_Init();

/**
 * @brief Muestra pantalla de bienvenida (splash screen)
 * 
 * Muestra logo de AireCiudadano durante el inicio
 */
void Display_Splash_Screen();

/**
 * @brief Actualiza toda la pantalla con datos actuales
 * 
 * Esta función se ejecuta periódicamente y muestra:
 * - Valores de PM2.5, PM1
 * - Temperatura y humedad
 * - Iconos de estado
 * - Barra de señal WiFi
 */
void Update_Display();

/**
 * @brief Actualiza pantalla OLED específica
 * 
 * Versión especializada para pantallas OLED
 */
void UpdateOLED();

/**
 * @brief Inicializa sistema de visualización genérico
 */
void displayInit();

/**
 * @brief Muestra mensaje de bienvenida
 */
void showWelcome();

/**
 * @brief Agrega mensaje a pantalla de bienvenida
 * @param msg Mensaje a mostrar
 */
void welcomeAddMessage(String msg);

/**
 * @brief Agrega mensaje genérico a pantalla
 * @param msg Mensaje a mostrar
 */
void AddMessage(String msg);

///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCIONES DE VISUALIZACIÓN - TEXTOS
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Muestra mensaje grande centrado
 * @param msg Mensaje a centrar
 */
void displayCenterBig(String msg);

/**
 * @brief Muestra mensaje en línea inferior
 * @param msg Mensaje en línea baja
 */
void displayBottomLine(String msg);

/**
 * @brief Muestra emoticón con etiqueta
 * @param numsmle Número del emoticón
 * @param msg Etiqueta del emoticón
 */
void displayEmoticonLabel(int numsmle, String msg);

/**
 * @brief Muestra nivel como texto
 * @param msg Texto del nivel
 */
void displayTextLevel(String msg);

/**
 * @brief Muestra nivel con color
 * @param cursor Posición del cursor
 * @param msg Texto del nivel
 */
void displayColorLevel(int cursor, String msg);

///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCIONES DE VISUALIZACIÓN - DATOS DE SENSORES
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Muestra promedio de sensor
 * @param average Valor promedio a mostrar
 */
void displayAverage(int average);

/**
 * @brief Muestra promedio de sensor de partículas
 * @param average Valor PM2.5 promedio
 */
void displaySensorAverage(int average);

/**
 * @brief Muestra datos completos del sensor
 * @param pm25 Valor PM2.5
 * @param humi Humedad (%)
 * @param temp Temperatura (°C)
 * @param rssi Intensidad de señal WiFi
 */
void displaySensorData(int pm25, int humi, int temp, int rssi);

/**
 * @brief Muestra nivel de batería con color
 * @param colour Color según nivel de batería
 */
void displayBatteryLevel(int colour);

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL DE PÁGINA
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Inicia nueva página de visualización
 */
void pageStart();

/**
 * @brief Finaliza página de visualización
 */
void pageEnd();

#endif // DISPLAY_H
