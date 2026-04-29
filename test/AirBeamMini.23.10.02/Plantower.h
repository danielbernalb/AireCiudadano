#ifndef PLANTOWER_HEADER
#define PLANTOWER_HEADER

byte plantower();
void sensor_average();
void stream_BLE_data();

String printDigits(byte digits);
String printDigitsYear(byte digits);

String Vmeas(byte batloop);

#endif
