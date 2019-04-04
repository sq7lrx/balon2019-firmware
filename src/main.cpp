#include <Arduino.h>
#include "DHT.h"

#define DHT11_PIN 4 //do zmiany

DHT dht;

float wilgotnosc;
float temperatura;

void setup() {
  // Tutaj umieszczamy wszystkie funkcje, ktore
  // Arduino ma wykonac zaraz po starcie.
  // UWAGA: Pamietajcie o kolejnosci
 Serial.begin(9600);
  dht.setup(DHT11_PIN);
}

//funkcja pomiaru temperatury i wilgotnosci
void pomiardht()
{
 wilgotnosc = dht.getHumidity();
 temperatura = dht.getTemperature();
  
  if (dht.getStatusString() == "OK") {
    Serial.print("Wilgotnosc: ");
    Serial.print(wilgotnosc);
    Serial.print("% | ");
    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.println("°C");
  }
}

void loop() {
  // tutaj umieszczamy wszystkie funkcje, ktore
  // program ma wykonywac w petli przez caly czas
  // dzialania.
pomiardht();
delay (1000);
}
