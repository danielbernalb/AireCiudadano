/**
 * @file sensors.h
 * @brief Funciones para gestión de sensores
 * 
 * Este archivo contiene todas las funciones relacionadas con:
 * - Inicialización de sensores
 * - Lectura de mediciones
 * - Calibración y ajuste
 * - Detección de errores
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCIONES GENERALES DE SENSORES
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Prueba e identifica los sensores conectados
 * 
 * Esta función:
 * 1. Escanea el bus I2C
 * 2. Identifica cada sensor por su dirección
 * 3. Configura banderas (flags) para sensores detectados
 * 4. Imprime información de diagnóstico
 */
void Test_Sensor();

/**
 * @brief Inicializa todos los sensores detectados
 * 
 * Configura:
 * - Parámetros de comunicación (I2C, UART)
 * - Rangos de medición
 * - Frecuencia de muestreo
 */
void Setup_Sensor();

/**
 * @brief Lee mediciones de todos los sensores activos
 * 
 * Esta función se ejecuta periódicamente y:
 * 1. Lee cada sensor activo
 * 2. Almacena valores en variables globales
 * 3. Maneja errores de lectura
 * 4. Actualiza acumulados para promedios
 */
void Read_Sensor();

/**
 * @brief Lee humedad y temperatura de sensores HyT
 * 
 * Función específica para sensores con medición de humedad/temperatura
 */
void ReadHyT();

///////////////////////////////////////////////////////////////////////////////////////////////////
// SENSORES DE CO2
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Inicializa sensores de CO2
 * 
 * Soporta:
 * - Sensirion SCD30
 * - SenseAir S8
 */
void Setup_CO2sensor();

/**
 * @brief Lee medición de sensor de CO2
 * 
 * Actualiza variables globales de CO2, temperatura y humedad
 */
void Read_CO2sensor();

///////////////////////////////////////////////////////////////////////////////////////////////////
// MEDIDOR DE SONIDO (SoundMeter)
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Inicializa medidor de sonido
 * 
 * Configura:
 * - Micrófono (ICS43434 o INMP441)
 * - Parámetros de muestreo
 * - Filtros de frecuencia
 */
void Setup_SoundMeter();

/**
 * @brief Lee medición de nivel de sonido
 * 
 * Calcula:
 * - Nivel dBA actual
 * - Máximos del período
 * - Promedios
 */
void Read_SoundMeter();

///////////////////////////////////////////////////////////////////////////////////////////////////
// SENSOR UV (LTR390)
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Inicializa sensor UV LTR390
 */
void Setup_UV();

/**
 * @brief Lee medición de índice UV
 * 
 * Calcula índice UV a partir de lectura raw
 */
void Read_UV();

///////////////////////////////////////////////////////////////////////////////////////////////////
// PLUVIÓMETRO (Rain Gauge)
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Inicializa pluviómetro
 * 
 * Configura:
 * - Pin de interrupción para contar pulsos
 * - Constante de mm por pulso
 */
void Setup_Rain();

/**
 * @brief Lee acumulado de lluvia
 * 
 * Procesa:
 * - Conteo de pulsos
 * - Conversión a mm
 * - Acumulado total
 */
void Read_Rain();

///////////////////////////////////////////////////////////////////////////////////////////////////
// INCLINÓMETROS
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Inicializa inclinómetros
 * 
 * Soporta:
 * - ADXL345
 * - LSM9DS1
 */
void Setup_Incli();

/**
 * @brief Lee medición de inclinómetros (loop principal)
 * 
 * Calcula:
 * - Ángulos X, Y, Z
 * - Aceleraciones
 */
void Read_Incli();

/**
 * @brief Lee medición de inclinómetros (cada 1 segundo)
 * 
 * Versión de alta frecuencia para mayor precisión
 */
void Read_Incli_1s();

///////////////////////////////////////////////////////////////////////////////////////////////////
// MEDIDORES DE NIVEL
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Inicializa medidores de nivel
 * 
 * Soporta:
 * - Ultrasónico por pines (JSN-SR04M)
 * - Ultrasónico serial
 * - RS485 SeedStudio
 */
void Setup_Nivel();

/**
 * @brief Lee medidor de nivel por pines
 */
void Read_Nivel_Pin();

/**
 * @brief Lee medidor de nivel serial
 */
void Read_Nivel_Ser();

/**
 * @brief Lee medidor de nivel serial (cada 1 segundo)
 */
void Read_Nivel_Ser_1s();

/**
 * @brief Lee medidor de nivel RS485
 */
void Read_Nivel_485();

/**
 * @brief Lee medidor de nivel RS485 (cada 1 segundo)
 */
void Read_Nivel_485_1s();

/**
 * @brief Lee nivel ultrasónico genérico
 * 
 * Incluye filtrado de outliers
 */
void LeerNivel();

/**
 * @brief Detecta lecturas atípicas (outliers)
 * @param nuevaLectura Nueva lectura a verificar
 * @return true si es outlier, false si es válida
 */
bool detectarOutlier(int nuevaLectura);

///////////////////////////////////////////////////////////////////////////////////////////////////
// SENSORES ESPECÍFICOS - FUNCIONES INTERNAS
///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Obtiene información del dispositivo SPS30
 */
void GetDeviceInfo_SPS30();

/**
 * @brief Maneja error de sensor SPS30 no disponible
 */
void NotAvailableSPS30();

#endif // SENSORS_H
