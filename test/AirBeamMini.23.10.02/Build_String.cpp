#include "Arduino.h"
#include "Build_String.h"

extern String post_data;
extern String ssid;
extern String pass;
extern String uuid;
extern String uuidauth;
extern String longitude;
extern String latitude;
extern String BLEmac;
extern String Aircastingserver;

String post = F("POST /api/realtime/measurements.json HTTP/1.1\nHost: ");
String post_1 = F("\nAuthorization: Basic ");
String post_2 = F("\nContent-Type: application/json\nAccept: application/json\nContent-Length: ");

//Data_1
String data1 = F("{\"data\":\"{\\\"measurement_type\\\":\\\"");
//Add Particulate Matter || Temperature || Humidity
String data1_1 = F("\\\", \\\"measurements\\\":[{\\\"longitude\\\":");
//Add Longitude
String data1_2 = F(",\\\"latitude\\\":");
//Add Latitude
String data1_3 = F(",\\\"time\\\":\\\"");

//Data_2
String data2 = F("}], \\\"sensor_package_name\\\":\\\"");
//Add Sensor Package
String data2_1 = F("\\\", \\\"sensor_name\\\":\\\"AirBeamMini-");
//Add F || RH || PM
String data2_2 = F("\\\", \\\"session_uuid\\\":\\\"");
//Add UUID
String data2_3 = F("\\\", \\\"measurement_short_type\\\":\\\"");
//Add F || RH || PM
String data2_4 = F("\\\", \\\"unit_symbol\\\":\\\"");
//Add F || ug/m3 || %
String data2_5 = F("\\\", \\\"threshold_high\\\":40, \\\"threshold_low\\\":10, \\\"threshold_medium\\\":20, \\\"threshold_very_high\\\":60, \\\"threshold_very_low\\\":-10, \\\"unit_name\\\":\\\"");         //Use this for temperature celsius
String data2_6 = F("\\\", \\\"threshold_high\\\":105, \\\"threshold_low\\\":45, \\\"threshold_medium\\\":75, \\\"threshold_very_high\\\":135, \\\"threshold_very_low\\\":15, \\\"unit_name\\\":\\\"");        //Use this for temperature fahrenheit
String data2_7 = F("\\\", \\\"threshold_high\\\":314, \\\"threshold_low\\\":280, \\\"threshold_medium\\\":297, \\\"threshold_very_high\\\":330, \\\"threshold_very_low\\\":264, \\\"unit_name\\\":\\\"");     //Use this for temperature kelvin
String data2_8 = F("\\\", \\\"threshold_high\\\":75, \\\"threshold_low\\\":25, \\\"threshold_medium\\\":50, \\\"threshold_very_high\\\":100, \\\"threshold_very_low\\\":0, \\\"unit_name\\\":\\\"");          //Use this for humidity
String data2_9 = F("\\\", \\\"threshold_high\\\":55, \\\"threshold_low\\\":12, \\\"threshold_medium\\\":35, \\\"threshold_very_high\\\":150, \\\"threshold_very_low\\\":0, \\\"unit_name\\\":\\\"");          //Use this for PM
String data2_10 = F("\\\", \\\"threshold_high\\\":3000, \\\"threshold_low\\\":1000, \\\"threshold_medium\\\":2000, \\\"threshold_very_high\\\":4000, \\\"threshold_very_low\\\":0, \\\"unit_name\\\":\\\"");  //Use this for PM different heat legend
String data2_11 = F("\\\", \\\"threshold_high\\\":100, \\\"threshold_low\\\":20, \\\"threshold_medium\\\":50, \\\"threshold_very_high\\\":200, \\\"threshold_very_low\\\":0, \\\"unit_name\\\":\\\"");        //Use this for PM10
//Add microgram per cubic meter || fahrenheit || percent
String data2_12 = F("\\\"}\", \"compression\":false}");

//Data_3
String data3 = F("\\\",\\\"timezone_offset\\\":0,\\\"milliseconds\\\":0,\\\"measured_value\\\":");

/*Build post String*/
String getPost() {
  post_data = F("");
  post_data += post;
  post_data += Aircastingserver;
  post_data += post_1;
  post_data += uuidauth;
  post_data += post_2;
  //Serial.println(post_data);
  return post_data;
}

/*Build Data 1 String*/
String getData_1(int stream) {
  post_data = F("");
  post_data += data1;
  switch (stream) {
    case 0:
      break;

    case 1:
      post_data += F("Temperature");
      break;

    case 2:
      post_data += F("Temperature");
      break;

    case 3:
      post_data += F("Temperature");
      break;

    case 4:
      post_data += F("Humidity");
      break;

    case 5:
      post_data += F("Particulate Matter");
      break;

    case 6:
      post_data += F("Particulate Matter");
      break;

    case 7:
      post_data += F("Particulate Matter");
      break;

    case 8:
      post_data += F("Particulate Matter");
      break;

    case 9:
      post_data += F("Particulate Matter");
      break;

    case 10:
      post_data += F("Particulate Matter");
      break;

    case 11:
      post_data += F("Particulate Matter");
      break;

    case 12:
      post_data += F("Particulate Matter");
      break;

    case 13:
      post_data += F("Particulate Matter");
      break;

    case 14:
      post_data += F("Particulate Matter");
      break;

    case 15:
      post_data += F("Particulate Matter");
      break;

    case 16:
      post_data += F("Particulate Matter");
      break;

    case 17:
      post_data += F("Particulate Matter");
      break;

    case 18:
      post_data += F("Particulate Matter");
      break;
  }

  post_data += data1_1;
  post_data += longitude;
  post_data += data1_2;
  post_data += latitude;
  post_data += data1_3;
  //Serial.print(post_data);
  return post_data;
}

/*Build Data2 String */
String getData_2(int stream) {
  post_data = F("");

  post_data += data2;
  post_data += F("AirBeamMini-");
  post_data += BLEmac;

  post_data += data2_1;
  switch (stream) {
    case 0:
      break;

    case 1:
      post_data += F("C");
      break;

    case 2:
      post_data += F("F");
      break;

    case 3:
      post_data += F("K");
      break;

    case 4:
      post_data += F("RH");
      break;

    case 5:
      post_data += F("PM1");
      break;

    case 6:
      post_data += F("PM2.5");
      break;

    case 7:
      post_data += F("PM10");
      break;

    case 8:
      post_data += F("PM-Count 0.3");
      break;

    case 9:
      post_data += F("PM-Count 0.5");
      break;

    case 10:
      post_data += F("PM-Count 1");
      break;

    case 11:
      post_data += F("PM-Count 2.5");
      break;

    case 12:
      post_data += F("PM-Count 5");
      break;

    case 13:
      post_data += F("PM-Count 10");
      break;

    case 14:
      post_data += F("PM-Bam 2.5");
      break;

    case 15:
      post_data += F("PM-Bam 10");
      break;

    case 16:
      post_data += F("PM-Grimm 1");
      break;

    case 17:
      post_data += F("PM-Grimm 2.5");
      break;

    case 18:
      post_data += F("PM-Grimm 10");
      break;
  }

  post_data += data2_2;
  post_data += uuid;

  post_data += data2_3;
  switch (stream) {
    case 0:
      break;

    case 1:
      post_data += F("C");
      break;

    case 2:
      post_data += F("F");
      break;

    case 3:
      post_data += F("K");
      break;

    case 4:
      post_data += F("RH");
      break;

    case 5:
      post_data += F("PM");
      break;

    case 6:
      post_data += F("PM");
      break;

    case 7:
      post_data += F("PM");
      break;

    case 8:
      post_data += F("PM");
      break;

    case 9:
      post_data += F("PM");
      break;

    case 10:
      post_data += F("PM");
      break;

    case 11:
      post_data += F("PM");
      break;

    case 12:
      post_data += F("PM");
      break;

    case 13:
      post_data += F("PM");
      break;

    case 14:
      post_data += F("PM");
      break;

    case 15:
      post_data += F("PM");
      break;

    case 16:
      post_data += F("PM");
      break;

    case 17:
      post_data += F("PM");
      break;

    case 18:
      post_data += F("PM");
      break;
  }

  post_data += data2_4;
  switch (stream) {
    case 0:
      break;

    case 1:
      post_data += F("C");
      break;

    case 2:
      post_data += F("F");
      break;

    case 3:
      post_data += F("K");
      break;

    case 4:
      post_data += F("%");
      break;

    case 5:
      post_data += F("µg/m³");
      break;

    case 6:
      post_data += F("µg/m³");
      break;

    case 7:
      post_data += F("µg/m³");
      break;

    case 8:
      post_data += F("ppl");
      break;

    case 9:
      post_data += F("ppl");
      break;

    case 10:
      post_data += F("ppl");
      break;

    case 11:
      post_data += F("ppl");
      break;

    case 12:
      post_data += F("ppl");
      break;

    case 13:
      post_data += F("ppl");
      break;

    case 14:
      post_data += F("µg/m³");
      break;

    case 15:
      post_data += F("µg/m³");
      break;

    case 16:
      post_data += F("µg/m³");
      break;

    case 17:
      post_data += F("µg/m³");
      break;

    case 18:
      post_data += F("µg/m³");
      break;
  }
  if (stream == 1) {
    post_data += data2_5;
  }
  if (stream == 2) {
    post_data += data2_6;
  }
  if (stream == 3) {
    post_data += data2_7;
  }
  if (stream == 4) {
    post_data += data2_8;
  }
  if (stream == 18 || stream == 17 || stream == 16 || stream == 15 || stream == 14 || stream == 6 || stream == 5) {
    post_data += data2_9;
  }
  if (stream == 13 || stream == 12 || stream == 11 || stream == 10 || stream == 9 || stream == 8) {
    post_data += data2_10;
  }
  if (stream == 7) {
    post_data += data2_11;
  }

  switch (stream) {
    case 0:
      break;

    case 1:
      post_data += F("celsius");
      break;

    case 2:
      post_data += F("fahrenheit");
      break;

    case 3:
      post_data += F("kelvin");
      break;

    case 4:
      post_data += F("percent");
      break;

    case 5:
      post_data += F("microgram per cubic meter");
      break;

    case 6:
      post_data += F("microgram per cubic meter");
      break;

    case 7:
      post_data += F("microgram per cubic meter");
      break;

    case 8:
      post_data += F("particles per .1 liter");
      break;

    case 9:
      post_data += F("particles per .1 liter");
      break;

    case 10:
      post_data += F("particles per .1 liter");
      break;

    case 11:
      post_data += F("particles per .1 liter");
      break;

    case 12:
      post_data += F("particles per .1 liter");
      break;

    case 13:
      post_data += F("particles per .1 liter");
      break;

    case 14:
      post_data += F("microgram per cubic meter");
      break;

    case 15:
      post_data += F("microgram per cubic meter");
      break;

    case 16:
      post_data += F("microgram per cubic meter");
      break;

    case 17:
      post_data += F("microgram per cubic meter");
      break;

    case 18:
      post_data += F("microgram per cubic meter");
      break;
  }
  post_data += data2_12;
  //Serial.print(post_data);
  return post_data;
}

/*Build Data3 String*/
String getData_3(int years, int months, int days, int hours, int mins, int secs, int sensor_value) {
  post_data = F("");
  post_data += printDigitsYear(years);
  post_data += F("-");
  post_data += printDigits(months);
  post_data += F("-");
  post_data += printDigits(days);
  post_data += F("T");
  post_data += printDigits(hours);
  post_data += F(":");
  post_data += printDigits(mins);
  post_data += F(":");
  post_data += printDigits(secs);
  post_data += data3;
  post_data += sensor_value;
  post_data += F(",\\\"value\\\":");
  post_data += sensor_value;
  //Serial.print(post_data);
  return post_data;
}
