#include <Arduino.h>
#include <String.h>



String nastepny (int* poczatek, int* koniec, String linijka)
{
  String wycinek;

  *poczatek=linijka.indexOf(*koniec);
  *koniec=linijka.indexOf(',', *(poczatek)+1);

  wycinek = linijka.substring(*poczatek+1, *koniec);

  return wycinek;
}

void setup() {
  Serial.begin(9600);

}

void loop() {
  int poczatek=0;
  int koniec=0;

  //String linijka=Serial.readStringUntil('/n');
  String linijka="$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";


  if(linijka.substring(0,6)=="$GPRMC")
  {
    koniec=linijka.indexOf(',');

    nastepny(&poczatek, &koniec, linijka); //godzina
    nastepny(&poczatek, &koniec, linijka); //a/v
    nastepny(&poczatek, &koniec, linijka); //dl
    nastepny(&poczatek, &koniec, linijka); // N
    nastepny(&poczatek, &koniec, linijka); // szer
    nastepny(&poczatek, &koniec, linijka); // E
    nastepny(&poczatek, &koniec, linijka); // Speed over the ground in knots
    nastepny(&poczatek, &koniec, linijka); // Track angle in degrees True
    nastepny(&poczatek, &koniec, linijka); //  Date - 23rd of March 1994
    nastepny(&poczatek, &koniec, linijka); //  Magnetic Variation

    delay(5000);
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
    nastepny(&poczatek, &koniec, linijka); //  Altitude, Meters, above mean sea level
    nastepny(&poczatek, &koniec, linijka); //  M
    nastepny(&poczatek, &koniec, linijka); // Height of geoid above WGS84 ellipsoid
    nastepny(&poczatek, &koniec, linijka); // M
    nastepny(&poczatek, &koniec, linijka); //  empty
  }
}