#include "obslugaSD.h"

void SDinit()
{
    Sd2Card card;
    SdVolume volume;
    SdFile root;
    Serial.print("Inicjalizacja SD ");
    SD.begin(SD_CS);                                    
    Serial.print(" . ");
    if(!( SD.exists("plik5.txt"))) Serial.println("Tworze plik ...");
    Serial.println("OK");
}
void zapisznaSD(String linijka)
{
    File myFile;
   myFile = SD.open("plik5.txt", FILE_WRITE);
   if (myFile) {
    myFile.println(linijka);
    myFile.close();
    Serial.println("Zapis");
  } else {
    Serial.println("Nie zapisalo sie");
  }

}
