#include <Arduino.h>
#include "DHT.h"

#define DHT11_PIN //tu zmieniamy na pin DHT

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
    Serial.print(wilgotnosc);
    Serial.print("% | ");
    Serial.print(temperatura);
    Serial.println("°C");
  }
  delay (1000);
}
