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
#include "main.hpp"

////////////////////////////////
bool FlagLED = false;

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
  pinMode(10, OUTPUT);

#if !ESP8266
  Serial.println(F("CPU0 reset reason:"));
  print_reset_reason(rtc_get_reset_reason(0));
#else
  uint16_t Resetvar = 0;
  Serial.print(F("CPU reset reason: "));
  rst_info *rinfo = ESP.getResetInfoPtr();
  Serial.println(rinfo->reason);
  Resetvar = rinfo->reason;
  ResetFlag = true;
  if (Resetvar == 1 || Resetvar == 2 || Resetvar == 3 || Resetvar == 4)
  {
    ResetFlag = false;
    Serial.print(F("Resetvar: false"));
  }
  Serial.print(F("Resetvar: "));
  Serial.println(Resetvar);
#endif

  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  Serial.println("HIGH");
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  Serial.println("LOW");
  delay(1000);                       // wait for a second
}

