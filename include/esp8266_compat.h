#ifndef ESP8266_COMPAT_H
#define ESP8266_COMPAT_H

#if defined(ESP8266)
  // Redefinimos las macros de log de ESP32 (ESP-IDF) para que usen Serial en ESP8266
  
  // Macros con TAG (usadas en TinyGsmMqttA76xx.h)
  #define ESP_LOGE(tag, format, ...) Serial.printf("[E][%s]: " format "\n", tag, ##__VA_ARGS__)
  #define ESP_LOGW(tag, format, ...) Serial.printf("[W][%s]: " format "\n", tag, ##__VA_ARGS__)
  #define ESP_LOGI(tag, format, ...) Serial.printf("[I][%s]: " format "\n", tag, ##__VA_ARGS__)
  #define ESP_LOGD(tag, format, ...) Serial.printf("[D][%s]: " format "\n", tag, ##__VA_ARGS__)
  #define ESP_LOGV(tag, format, ...) Serial.printf("[V][%s]: " format "\n", tag, ##__VA_ARGS__)
  
  // Macros sin TAG (usadas en TinyGsmHttpsComm.h y TinyGsmFSComm.tpp)
  #define log_e(format, ...) Serial.printf("[E]: " format "\n", ##__VA_ARGS__)
  #define log_w(format, ...) Serial.printf("[W]: " format "\n", ##__VA_ARGS__)
  #define log_i(format, ...) Serial.printf("[I]: " format "\n", ##__VA_ARGS__)
  #define log_d(format, ...) Serial.printf("[D]: " format "\n", ##__VA_ARGS__)
  #define log_v(format, ...) Serial.printf("[V]: " format "\n", ##__VA_ARGS__)
#endif

#endif // ESP8266_COMPAT_H