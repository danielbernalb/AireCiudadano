#ifndef SDCARD_HEADER
#define SDCARD_HEADER

void Soft_Reset_ESP();

void readFile(const char* path);
void refreshFile(const char* path);
void appendFile(const char* path, const char* message);
void deleteFile(const char* path);

String compFile(String SD_index, const char* path, const char* path1);

String printDigitsYear(byte digits);

#endif
