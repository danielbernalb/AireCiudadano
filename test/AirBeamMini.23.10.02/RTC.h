#ifndef RTC_HEADER
#define RTC_HEADER

void RTCsettime(int hours, int mins, int secs, int days, int months, int years);
byte RTCgettime();

String printDigitsYear(byte digits);

#endif
