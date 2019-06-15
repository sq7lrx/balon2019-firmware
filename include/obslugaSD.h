#ifndef obslugaSD
#define obslugaSD
#include <SPI.h>
#include <SD.h>

#define SD_CS 3
#define SD_FILENAME "telemetria.txt"

void SDinit();
void zapisznaSD(String linijka);

#endif