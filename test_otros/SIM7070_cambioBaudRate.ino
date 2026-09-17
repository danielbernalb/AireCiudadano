// =========================================================================
//  UTILIDAD: Cambiar el baudrate del SIM7070 (generico, cualquier velocidad)
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
//   2) Envia "AT+IPR=<NEW_BAUD>" -> fija el baudrate (se guarda solo,
//      AUTO_SAVE) y toma efecto de inmediato tras el "OK".
//   3) Reabre el puerto ya a NEW_BAUD y confirma con "AT" que el cambio
//      funciono (sin reiniciar el modulo).
//   4) (Opcional pero recomendado) Envia "AT+CREBOOT" para reiniciar el
//      modulo y confirmar que el nuevo baudrate persiste tras el reinicio.
//
//  IMPORTANTE: una vez confirmado el cambio, en tu proyecto principal
//  debes cambiar SerialAT.begin(OLD_BAUD) por SerialAT.begin(NEW_BAUD),
//  o el modem y el ESP quedaran hablando a velocidades distintas.
// =========================================================================

#include <SoftwareSerial.h>

// Mismos pines que usas en tu proyecto principal (D7=RX, D6=TX)
SoftwareSerial SerialAT(13, 12);

// -------- UNICO LUGAR QUE HAY QUE CAMBIAR SEGUN LO QUE NECESITES --------
const uint32_t OLD_BAUD = 19200;   // baudrate actual del modem
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
  Serial.print("=== Cambiar baudrate SIM7070: ");
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

  // ---------- Paso 2: fijar el nuevo baudrate (AT+IPR, AUTO_SAVE) ----------
  Serial.print("\nPaso 2: fijando baudrate a ");
  Serial.print(NEW_BAUD);
  Serial.println(" (AT+IPR)...");
  String cmdIpr = "AT+IPR=" + String(NEW_BAUD);
  if (!sendAT(cmdIpr, "OK", 3000)) {
    Serial.println("ERROR: el modem no acepto el comando AT+IPR.");
    Serial.println("Verifica que NEW_BAUD sea uno de los valores soportados (300, 600, 1200, 2400,");
    Serial.println("4800, 9600, 19200, 38400, 57600, 115200, 230400, ...).");
    return;
  }
  Serial.println("AT+IPR aceptado. El modem ya cambio de velocidad (y quedo guardado en NVRAM).");

  // ---------- Paso 3: reabrir el puerto YA al nuevo baudrate ----------
  // A diferencia del A7670, aqui NO hay que esperar un reset: el cambio
  // de baudrate se aplica justo despues del "OK" anterior.
  Serial.print("\nPaso 3: verificando modem a ");
  Serial.print(NEW_BAUD);
  Serial.println(" baudios (sin reiniciar)...");
  SerialAT.begin(NEW_BAUD);
  delay(200);

  ok = false;
  for (int i = 0; i < 5 && !ok; i++) {
    ok = sendAT("AT", "OK", 2000);
    if (!ok) delay(500);
  }

  if (!ok) {
    Serial.print("ERROR: el modem no respondio a ");
    Serial.print(NEW_BAUD);
    Serial.println(" baudios inmediatamente despues del AT+IPR.");
    Serial.println("Prueba desconectar y reconectar fisicamente la alimentacion del modem,");
    Serial.println("y vuelve a correr este sketch con OLD_BAUD = NEW_BAUD para verificar si ya quedo aplicado.");
    return;
  }
  Serial.print("*** Modem ya responde a ");
  Serial.print(NEW_BAUD);
  Serial.println(" baudios. ***");

  // ---------- Paso 4 (opcional): reiniciar y confirmar que persiste ----------
  Serial.println("\nPaso 4: reiniciando el modulo (AT+CREBOOT) para confirmar persistencia...");
  SerialAT.print("AT+CREBOOT\r\n");
  Serial.println("Esperando a que el modulo reinicie (10s)...");
  delay(10000);

  SerialAT.begin(NEW_BAUD);
  delay(200);

  ok = false;
  for (int i = 0; i < 8 && !ok; i++) {
    ok = sendAT("AT", "OK", 2000);
    if (!ok) delay(1000);
  }

  if (ok) {
    Serial.print("\n*** EXITO: tras reiniciar, el modem sigue respondiendo a ");
    Serial.print(NEW_BAUD);
    Serial.println(" baudios. El cambio quedo guardado de forma permanente. ***");
    Serial.println("Ahora en tu proyecto principal cambia: SerialAT.begin(OLD_BAUD) -> SerialAT.begin(NEW_BAUD)");
  } else {
    Serial.print("\nAVISO: el modem no respondio a ");
    Serial.print(NEW_BAUD);
    Serial.println(" baudios tras el AT+CREBOOT.");
    Serial.println("Esto puede ser solo un timeout del reinicio (el SIM7070 a veces tarda mas de 10s en");
    Serial.println("volver a responder AT). Prueba subir el delay de 10000 a 15000-20000 ms, o vuelve a");
    Serial.println("correr el sketch con OLD_BAUD = NEW_BAUD para confirmar si el cambio ya quedo aplicado.");
  }
}

void loop() {
  // Sketch de una sola pasada, no hace falta nada aqui.
}
