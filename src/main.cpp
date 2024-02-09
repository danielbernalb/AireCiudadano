
#include <Arduino.h>
#include "main.hpp"

////////////////////////////////

// Select your modem:
// #define TINY_GSM_MODEM_SIM800
#define TINY_GSM_MODEM_SIM7600

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

#include <SoftwareSerial.h>
SoftwareSerial SerialAT(13, 12); // RX, TX

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

#define PWRKEY 16 // GPIO pin used for PWRKEY
#define LED_PIN 2

// Your GPRS credentials, if any
const char apn[] = "web.colombiamovil.com.co";
// const char apn[] = "internet.comcel.com.co";
// const char apn[] = "internet.movistar.com.co";

// MQTT details
const char *broker = "sensor.aireciudadano.com";

const char *topicac = "measurement";
const char *topic = "config/";

String MQTT_send_topic;
String MQTT_receive_topic;

#include <TinyGsmClient.h>

// MQTT
#include <PubSubClient.h>
char MQTT_message[256];
// PubSubClient MQTT_client(wifi_client);
char received_payload[384];

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient MQTT_client(client);

String aireciudadano_device_id = "AireCiudadano_TestMQTT";
int pm25int = 4;
float latitudef = 4.6987;
float longitudef = -74.0987;
int inout = 1;

void setup()
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);

  // Set GSM module baud rate
  SerialAT.begin(115200);

  SerialMon.println("");
  SerialMon.println("Wait...");

  // Restart takes quite some time
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  delay(5000);

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected())
  {
    SerialMon.println("Network connected");
  }

  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn))
  {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected())
  {
    SerialMon.println("GPRS connected");
  }

  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected())
  {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true))
    {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected())
    {
      SerialMon.println("Network re-connected");
    }

    if (!modem.isGprsConnected())
    {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn))
      {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected())
      {
        SerialMon.println("GPRS reconnected");
      }
    }
  }
  MQTT_send_topic = "measurement";                          // measurement are sent to this topic
  MQTT_receive_topic = "config/" + aireciudadano_device_id; // Config messages will be received in config/id

  Init_MQTT();

  SerialMon.println("Start Loop:");
  delay(1000);
}

void loop()
{
  if (modem.isNetworkConnected())
    SerialMon.println("Network connected");
  else
    SerialMon.println("Network disconnected");

  if (modem.isGprsConnected())
    SerialMon.println("GPRS connected!");
  else
    SerialMon.println("GPRS disconnected!");

  if (!MQTT_client.connected())
  {
    SerialMon.println("MQTT disconnected!");
    Reconnect_MQTT();
  }
  else
  {
    SerialMon.println("MQTT connected!");
    //    Reconnect_MQTT();                   // No funciono
    Send_Message_Cloud_App_MQTT();
    delay(60000);
  }
}

void Send_Message_Cloud_App_MQTT()
{ // Send measurements to the cloud application by MQTT
  if (pm25int == 61)
    pm25int = 0;
  else
    pm25int = pm25int + 1;

  sprintf(MQTT_message, "{id: %s, PM25: %d, latitude: %f, longitude: %f, inout: %d}", aireciudadano_device_id.c_str(), pm25int, latitudef, longitudef, inout);

  MQTT_client.publish(MQTT_send_topic.c_str(), MQTT_message);

  Serial.print("Mensaje enviado: ");
  Serial.print(MQTT_send_topic.c_str());
  Serial.print(", ");
  Serial.println(MQTT_message);
  Serial.println("");

  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
}

void Init_MQTT()
{ // MQTT Init function
  Serial.print(F("Attempting to connect to the MQTT broker "));
  Serial.print(F("sensor.aireciudadano.com"));
  Serial.print(F(":"));
  Serial.println(F("80"));

  //  MQTT_client.setBufferSize(1024);

  MQTT_client.setServer("sensor.aireciudadano.com", 80);
  //  MQTT_client.setCallback(Receive_Message_Cloud_App_MQTT);

  MQTT_client.connect(aireciudadano_device_id.c_str());

  if (!MQTT_client.connected())
  {
    Serial.println("MQTT_Reconnect");
    Reconnect_MQTT();
  }
  else
  {
    // Once connected resubscribe
    MQTT_client.subscribe(MQTT_receive_topic.c_str());
    Serial.print(F("MQTT connected - Receive topic: "));
    Serial.println(MQTT_receive_topic);
  }
}

void Reconnect_MQTT()
{ // MQTT Init function
  MQTT_client.setServer("sensor.aireciudadano.com", 80);
  MQTT_client.connect(aireciudadano_device_id.c_str());

  while (!MQTT_client.connected())
  {
    delay(5000);
    MQTT_client.setServer("sensor.aireciudadano.com", 80);
    MQTT_client.connect(aireciudadano_device_id.c_str());
  }
  // Once connected resubscribe
  MQTT_client.subscribe(MQTT_receive_topic.c_str());
  Serial.print(F("MQTT connected - Receive topic: "));
  Serial.println(MQTT_receive_topic);
}

void Receive_Message_Cloud_App_MQTT(char *topic, byte *payload, unsigned int length)
{ // callback function to receive configuration messages from the cloud application by MQTT
  memcpy(received_payload, payload, length);
  Serial.print(F("Message arrived: "));
  Serial.println(received_payload);
}