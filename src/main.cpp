
#include <Arduino.h>
#include "main.hpp"

/*
Developed by Nandu, Anandu, Unnikrishnan 
Company: Elementz Engineers Guild Pvt Ltd
*/
//#include<SoftwareSerial.h>
//SoftwareSerial mySerial(3,2);
#define mySerial Serial2
//#define sw 5
#define led 18
int flag = 1;
int flag1 = 0;
int state=0;
String Publish = "measurement"; //Publish Topic
String Subscribe = "config/";   //Subscribe Topic
String mensaje1 = "id: AireCiudadano_TestMQTT, PM25: 3, latitude: 4.698700, longitude: -74.098701, inout: 1";
String mensaje2 = "id: AireCiudadano_TestMQTT, PM25: 4, latitude: 4.698700, longitude: -74.098701, inout: 1";
String mensaje3 = "id: AireCiudadano_TestMQTT, PM25: 5, latitude: 4.698700, longitude: -74.098701, inout: 1";
String mensaje4 = "id: AireCiudadano_TestMQTT, PM25: 6, latitude: 4.698700, longitude: -74.098701, inout: 1";

void setup() 
{
  Serial.begin(115200);
  mySerial.begin(115200);
  //pinMode(sw, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  //AT Commands for setting up the client id and Server
  //Need to be executed once -- Open serial terminal doe seeing the debug messages
  Serial.println("Connecting To Server........");
  mySerial.println("ATE0");
  delay(2000);
  mySerial.println("AT+CMQTTSTART"); //Establishing MQTT Connection
  delay(2000); 
  mySerial.println("AT+CMQTTACCQ=0,\"test0\""); //Client ID - change this for each client as this need to be unique
  delay(2000);
  mySerial.println("AT+CMQTTCONNECT=0,\"sensor.aireciudadano.com:80\",60,1"); //MQTT Server Name for connecting this client
  delay(2000);

  //SUBSCRIBE MESSAGE
  //Need to be executed once
  mySerial.println("AT+CMQTTSUBTOPIC=0,Subscribe.length(),1"); //AT Command for Setting up the Subscribe Topic Name 
  delay(2000);
  mySerial.println(Subscribe); //Topic Name
  delay(2000);
  mySerial.println("AT+CMQTTSUB=0,mensaje1.length(),1,1"); //Length of message
  delay(2000);
  mySerial.println(mensaje1); //message
  delay(2000);
  Serial.println("Done");
}

void loop() 
{
  String a;
      //PUBLISH MESSAGE
      flag1 = 1;
      digitalWrite(led, HIGH);
      Serial.println("Publishing Message: mensaje n");
      mySerial.println("AT+CMQTTTOPIC=0,Publish.length()"); //AT Command for Setting up the Publish Topic Name
      delay(1000);
      mySerial.println(Publish); //Topic Name
      delay(1000);
      mySerial.println("AT+CMQTTPAYLOAD=0,mensaje2.length()"); //Payload length
      delay(1000);
      mySerial.println(mensaje2); //Payload message
      delay(1000);
      mySerial.println("AT+CMQTTPUB=0,1,60"); //Acknowledgment
      delay(60000);
  }