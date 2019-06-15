#include <Arduino.h>
#include <stdlib.h>
#include <Adafruit_BMP085.h>
#include "DHT.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MPU6050.h>
 
#define DHTPIN 7
#define ONEWIRE_PIN 5

OneWire oneWire(5); //Podłączenie do 5
DallasTemperature termometry(&oneWire); //Przekazania informacji do biblioteki
MPU6050 mpu;

float zewnetrzna_temperatura()
{
    DHT dht;
    float t = dht.getTemperature();  // odczyt temperatury
     
    return t;
}

float wilgotnosc()
{
    DHT dht;
    float h = dht.getHumidity();     // odczyt wilgotności powietrza
     
    return h;
}

float barometr()
{ 
    Adafruit_BMP085 bmp;
    return bmp.readPressure();
}

float wewnetrzna_temperaturaDS() //DS
{
  termometry.requestTemperatures(); //Pobranie temperatury czujnika
  return termometry.getTempCByIndex(0);  //Wyswietlenie informacji
}

float zewnetrzna_temperaturaDS() //DS
{
  termometry.requestTemperatures(); //Pobranie temperatury czujnika
  return termometry.getTempCByIndex(1);  //Wyswietlenie informacji
}



void mpuInit()
{
  Serial.print("Inicjalizacja MPU6050 ");
  mpu.initialize();
  Serial.print(" . ");
  if(mpu.testConnection()) Serial.println("OK");
}

int mpuX()
{
  return mpu.getAccelerationX();
}
int mpuY()
{
  return mpu.getAccelerationY();
}
int mpuZ()
{
  return mpu.getAccelerationZ();
}
