#include <Arduino.h>
#include "DHT.h"

#define DHT11_PIN 4 //do zmiany

DHT dht;
 
void setup()
{
  Serial.begin(9600);
  dht.setup(DHT11_PIN);
}
 
void loop()
{
  float wilgotnosc = dht.getHumidity();
  float temperatura = dht.getTemperature();
  
  if (dht.getStatusString() == "OK") {
    Serial.print("Wilgotnosc: ");
    Serial.print(wilgotnosc);
    Serial.print("% | ");
    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.println("°C");
  }
  delay (1000);
}
