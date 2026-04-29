#include "Arduino.h"
#include "Color.h"

/*
  red_led 2    channel 0
  green_led 1  channel 1
  blue_led 0   channel 2

  Common Anode+ RGB LED
  0    Cathode- is fully on
  255  Cathode- is fully off
*/

byte LEDoff = 255;

byte LEDon(float bright) {
  return round(LEDoff - (bright * 2.55));
}

void red(byte bright) {
  ledcWrite(0, LEDon(bright));
  ledcWrite(1, LEDoff);
  ledcWrite(2, LEDoff);
}

void green(byte bright) {
  ledcWrite(0, LEDoff);
  ledcWrite(1, LEDon(bright));
  ledcWrite(2, LEDoff);
}

void blue(byte bright) {
  ledcWrite(0, LEDoff);
  ledcWrite(1, LEDoff);
  ledcWrite(2, LEDon(bright));
}

void magenta(byte bright) {
  ledcWrite(0, LEDon(bright));
  ledcWrite(1, LEDoff);
  ledcWrite(2, LEDon(bright));
}

void cyan(byte bright) {
  ledcWrite(0, LEDoff);
  ledcWrite(1, LEDon(bright));
  ledcWrite(2, LEDon(bright));
}

void yellow(byte bright) {
  ledcWrite(0, LEDon(bright));
  ledcWrite(1, LEDon(bright));
  ledcWrite(2, LEDoff);
}

void orange(byte bright) {
  ledcWrite(0, LEDon(bright));
  ledcWrite(1, LEDon(bright * 0.25));  //0.25
  ledcWrite(2, LEDoff);
}

void purple(byte bright) {
  ledcWrite(0, LEDon(bright * 0.125)); //0.125
  ledcWrite(1, LEDoff);
  ledcWrite(2, LEDon(bright * 0.75));  //0.75
}

void white(byte bright) {
  ledcWrite(0, LEDon(bright));
  ledcWrite(1, LEDon(bright));
  ledcWrite(2, LEDon(bright));
}

void off() {
  ledcWrite(0, LEDoff);
  ledcWrite(1, LEDoff);
  ledcWrite(2, LEDoff);
}
