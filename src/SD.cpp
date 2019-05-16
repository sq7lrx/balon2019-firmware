
#include <SPI.h>
#include <SD.h>

Sd2Card card;
SdVolume volume;
SdFile root;

void zapisznaSD(String linijka)
{
  Serial.begin(9600);
  File myFile;
  if (!SD.begin(3)) {
    Serial.println("nie chce siem menda wlaczyc");
    while (1);
  }
   myFile = SD.open("dane.txt", FILE_WRITE);
   if (myFile) {
    myFile.println(linijka);
    myFile.close();
  } else {
    Serial.println("Nie zapisalo sie");
  }

}
void setup()
{
 
}


void loop() {
  zapisznaSD("hahaha1");
  zapisznaSD("haha2");
}