//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AireCiudadano medidor Fijo - Medidor de PM2.5 abierto, medición opcional de humedad y temperatura.
// Más información en: aireciudadano.com
// Este firmware es un fork del proyecto Anaire (https://www.anaire.org/) recomendado para la medición de CO2.
// 26/03/2023 info@aireciudadano.com
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Pendientes:
// Revisar todas las versiones y pruebas para pasar a v2.0
// MODIFICACIONES EXTERNAS:
// Modificado libreria WifiManager para compatibilidad
// Modificado PubSubClient.cpp : para quitar warning

#include <Arduino.h>

////////////////////////////////

#ifdef ESP32S3def
#define ESP32S3 true     // Set to true in case you use an ESP32S3
#else
#define ESP32S3 false     // Set to true in case you use an ESP32S3
#endif

#ifdef ESP32C3def
#define ESP32C3 true     // Set to true in case you use an ESP32S3
#else
#define ESP32C3 false     // Set to true in case you use an ESP32S3
#endif

#ifdef Rosverdef
#define Rosver true     // Set to true in case you use an ESP32S3
#else
#define Rosver false     // Set to true in case you use an ESP32S3
#endif

#define LEDPIN 10


///////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // Initialize serial port for serial monitor in Arduino IDE
#if !ESP8266SH
  Serial.begin(115200);
#else
  Serial.begin(9600);
#endif
  delay(100);
  while (!Serial)
  {
    delay(500); // wait 0.5 seconds for connection
  }
  pinMode(LEDPIN, OUTPUT);

  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  digitalWrite(LEDPIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("HIGH");
  delay(1000);                       // wait for a second
  digitalWrite(LEDPIN, LOW);    // turn the LED off by making the voltage LOW
  Serial.println("LOW");
  delay(1000);                       // wait for a second
}

