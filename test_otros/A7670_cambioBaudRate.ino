// =========================================================================
//  UTILIDAD: Cambiar el baudrate del A7670 (generico, cualquier velocidad)
// =========================================================================
//  Sketch independiente, para correr UNA sola vez desde el ESP8266.
//  Solo usa SoftwareSerial (ya viene con el core de ESP8266, no hay que
//  instalar nada nuevo). No depende de TinyGSM.
//
//  Para usarlo: solo cambia OLD_BAUD y NEW_BAUD aqui abajo segun lo que
//  necesites (por ejemplo 115200 -> 9600, o 9600 -> 115200, etc).
//
//  Que hace:
//   1) Abre el puerto al baudrate ACTUAL del modem (OLD_BAUD) y verifica
//      comunicacion con "AT".
//   2) Envia "AT+IPREX=<NEW_BAUD>" -> fija el baudrate PERMANENTE (el que
//      el A7670 usara despues de cada reinicio/apagado).
//   3) Envia "AT+CRESET" para reiniciar el modulo por software y que
//      el cambio quede aplicado.
//   4) Reabre el puerto ya a NEW_BAUD y confirma con "AT" que el cambio
//      funciono.
//
//  IMPORTANTE: una vez confirmado el cambio, en tu proyecto principal
//  debes cambiar SerialAT.begin(OLD_BAUD) por SerialAT.begin(NEW_BAUD),
//  o el modem y el ESP quedaran hablando a velocidades distintas.
// =========================================================================

#include <SoftwareSerial.h>

// Mismos pines que usas en tu proyecto principal (D7=RX, D6=TX)
SoftwareSerial SerialAT(13, 12);

// -------- UNICO LUGAR QUE HAY QUE CAMBIAR SEGUN LO QUE NECESITES --------
const uint32_t OLD_BAUD = 115200;   // baudrate actual del modem
const uint32_t NEW_BAUD = 9600;    // baudrate al que quieres cambiarlo
// --------------------------------------------------------------------------


// Envia un comando AT y espera una respuesta que contenga "expected"
// dentro de timeoutMs. Devuelve true si la encontro.
bool sendAT(const String &cmd, const char *expected, uint32_t timeoutMs = 3000) {
  while (SerialAT.available()) SerialAT.read();  // limpiar basura pendiente

  Serial.print(">> ");
  Serial.println(cmd);
  SerialAT.print(cmd);
  SerialAT.print("\r\n");

  String resp = "";
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      resp += (char)SerialAT.read();
    }
    if (resp.indexOf(expected) != -1) {
      Serial.print("<< ");
      Serial.println(resp);
      return true;
    }
    yield();  // alimenta el watchdog del ESP8266 mientras esperamos
  }
  Serial.print("<< (timeout) ");
  Serial.println(resp);
  return false;
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.print("=== Cambiar baudrate A7670: ");
  Serial.print(OLD_BAUD);
  Serial.print(" -> ");
  Serial.print(NEW_BAUD);
  Serial.println(" ===");

  // ---------- Paso 1: verificar comunicacion al baudrate actual ----------
  Serial.print("\nPaso 1: verificando modem a ");
  Serial.print(OLD_BAUD);
  Serial.println(" baudios...");
  SerialAT.begin(OLD_BAUD);
  delay(200);

  bool ok = false;
  for (int i = 0; i < 5 && !ok; i++) {
    ok = sendAT("AT", "OK", 2000);
    if (!ok) delay(500);
  }

  if (!ok) {
    Serial.print("ERROR: el modem no respondio a ");
    Serial.print(OLD_BAUD);
    Serial.println(" baudios. Revisa cableado/alimentacion.");
    Serial.println("Si ya hubiera quedado en NEW_BAUD por un intento previo, ajusta OLD_BAUD y reintenta.");
    return;
  }
  Serial.print("Modem responde OK a ");
  Serial.print(OLD_BAUD);
  Serial.println(" baudios.");

  // ---------- Paso 2: fijar el nuevo baudrate permanente ----------
  Serial.print("\nPaso 2: fijando baudrate permanente a ");
  Serial.print(NEW_BAUD);
  Serial.println(" (AT+IPREX)...");
  String cmdIprex = "AT+IPREX=" + String(NEW_BAUD);
  if (!sendAT(cmdIprex, "OK", 3000)) {
    Serial.println("ERROR: el modem no acepto el comando AT+IPREX.");
    return;
  }
  Serial.println("AT+IPREX aceptado.");

  // ---------- Paso 3: reiniciar el modulo para aplicar el cambio ----------
  Serial.println("\nPaso 3: reiniciando el modulo (AT+CRESET) para aplicar el cambio...");
  SerialAT.print("AT+CRESET\r\n");
  Serial.println("Esperando a que el modulo reinicie (10s)...");
  delay(10000);

  // ---------- Paso 4: verificar que ya responde al nuevo baudrate ----------
  Serial.print("\nPaso 4: verificando modem a ");
  Serial.print(NEW_BAUD);
  Serial.println(" baudios...");
  SerialAT.begin(NEW_BAUD);
  delay(200);

  ok = false;
  for (int i = 0; i < 8 && !ok; i++) {
    ok = sendAT("AT", "OK", 2000);
    if (!ok) delay(1000);
  }

  if (ok) {
    Serial.print("\n*** EXITO: el modem ya responde a ");
    Serial.print(NEW_BAUD);
    Serial.println(" baudios. ***");
    Serial.println("Ahora en tu proyecto principal cambia: SerialAT.begin(OLD_BAUD) -> SerialAT.begin(NEW_BAUD)");
  } else {
    Serial.print("\nERROR: el modem no respondio a ");
    Serial.print(NEW_BAUD);
    Serial.println(" baudios tras el reinicio por AT+CRESET.");
    Serial.println("Prueba desconectar y reconectar fisicamente la alimentacion del modem,");
    Serial.println("y vuelve a correr este sketch (ajusta OLD_BAUD/NEW_BAUD segun lo que quieras probar).");
  }
}

void loop() {
  // Sketch de una sola pasada, no hace falta nada aqui.
}