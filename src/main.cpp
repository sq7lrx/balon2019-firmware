#include <Arduino.h>
#include "DHT.h"
#include <SoftwareSerial.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
 
Adafruit_BMP085 bmp;
#define DHTPIN 7

DHT dht;

float wilgotnosc;
float temperatura;

SoftwareSerial gpsSerial(3,4);//rx,tx  

void setup() {
  // Tutaj umieszczamy wszystkie funkcje, ktore
  // Arduino ma wykonac zaraz po starcie.
  // UWAGA: Pamietajcie o kolejnosci
  gpsSerial.begin(9600); // connect gps sensor 
  Serial.begin(9600);
  dht.setup(DHTPIN, dht.AM2302);
  // DO ZROBIENIA:
  // - wlaczenie GPS
  // - ustawienie GPS w tryb duzych predkosci
  // - wlaczenie innych sensorow
}

//funkcja pomiaru temperatury i wilgotnosci

String th()
{
    float t = dht.getTemperature();  // odczyt temperatury
    float h = dht.getHumidity();     // odczyt wilgotności powietrza
     
    String wynik = String(t) + "," + String(h);
    return wynik;
}

String nastepny (int* poczatek, int* koniec, String linijka)
{
  String wycinek;

  *poczatek=linijka.indexOf(*koniec);
  *koniec=linijka.indexOf(',', *(poczatek)+1);

  wycinek = linijka.substring(*poczatek+1, *koniec);

  return wycinek;
}

String gps( String linijka)
{

  int poczatek=0;
  int koniec=0;
  String av;
  String wynik="";

  if(linijka.substring(0,6)=="$GPRMC")
    {
      koniec=linijka.indexOf(',');
      
      wynik+=(nastepny(&poczatek, &koniec, linijka)) + ","; //godzina
      av=nastepny(&poczatek, &koniec, linijka); //a/v
      wynik+=(nastepny(&poczatek, &koniec, linijka)) + ","; //dl
      nastepny(&poczatek, &koniec, linijka); // N
      wynik+=(nastepny(&poczatek, &koniec, linijka)) + ","; // szer
      nastepny(&poczatek, &koniec, linijka); // E
      wynik+=(nastepny(&poczatek, &koniec, linijka)) + ","; // Speed over the ground in knots
      nastepny(&poczatek, &koniec, linijka); // Track angle in degrees True
      nastepny(&poczatek, &koniec, linijka); //  Date - 23rd of March 1994
      nastepny(&poczatek, &koniec, linijka); //  Magnetic Variation

    }
    else if(linijka.substring(0,6)=="$GPGGA")
    {
      koniec=linijka.indexOf(',');

      nastepny(&poczatek, &koniec, linijka); //Fix taken at 12:35:19 UTC
      nastepny(&poczatek, &koniec, linijka); //Latitude 48 deg 07.038' N
      nastepny(&poczatek, &koniec, linijka); //N
      nastepny(&poczatek, &koniec, linijka); // Longitude 11 deg 31.000
      nastepny(&poczatek, &koniec, linijka); // E
      nastepny(&poczatek, &koniec, linijka); // Fix quality: 0 = invalid
                                            // 1 = GPS fix (SPS)
                                            // 2 = DGPS fix
                                            // 3 = PPS fix
                                            // 4 = Real Time Kinematic
                                            // 5 = Float RTK
                                            // 6 = estimated (dead reckoning) (2.3 feature)
                                            // 7 = Manual input mode
                                            // 8 = Simulation modeE
      nastepny(&poczatek, &koniec, linijka); // Speed over the ground in knots
      nastepny(&poczatek, &koniec, linijka); // Horizontal dilution of position
      wynik+=(nastepny(&poczatek, &koniec, linijka)) + ","; //  Altitude, Meters, above mean sea level
      nastepny(&poczatek, &koniec, linijka); //  M
      nastepny(&poczatek, &koniec, linijka); // Height of geoid above WGS84 ellipsoid
      nastepny(&poczatek, &koniec, linijka); // M
      nastepny(&poczatek, &koniec, linijka); //  empty
    }
  if(av=="A")
    return wynik;
  else
    return "Nie ma";
}

String barometr()
{ 
    String wynik="";
    wynik+=String(bmp.readPressure());
    // Obliczamy wysokosc dla domyslnego cisnienia przy pozimie morza
    // p0 = 1013.25 millibar = 101325 Pascal
    wynik+=String(bmp.readAltitude());
    return wynik;
}

//void input_data(int g,int m,int s,int wysokosc,int cisnienie,int uv, float dlugosc,float szerokosc,float temperatura,float wilgotnosc)
//{
 //   String ident = "identyfikator_nasz";
 //   String output=ident+","+String(g)+","+String(m)+","+String(s)+","+String(dlugosc)+String(dlugosc_kier)+","+String(szerokosc)+String(szerokosc_kier)+","+String(wysokosc)+","+String(temperatura)+","+String(cisnienie)+","+String(wilgotnosc)+","+String(co2)+","+String(ozon)+","+String(uv)+","+String(pyl);
 //   Serial.println(output);
//}

void loop() {
  // tutaj umieszczamy wszystkie funkcje, ktore
  // program ma wykonywac w petli przez caly czas
  // dzialania.
  String ramka;

  // Uruchomienie funkcji odczytujacych dane z sensorow
  String linijka=gpsSerial.readStringUntil('/n');
  Serial.println("GPS");
  Serial.println(gps(linijka));
  Serial.println("TH");
  Serial.println(th());
  Serial.println("CISNIENIE");
  Serial.println(barometr());
  

  // stworzy ramke z wszytkich pomiarow
  // DO ZROBIENIA
  
  // wyslac ramke przez radio na ziemie
  // DO ZROBIENIA

  delay (1000);
}
