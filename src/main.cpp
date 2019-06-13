#include <Arduino.h>
//#include "debug.h"
#include <Arduino.h>
#include "gps.h"
#include <avr/power.h>
#include <avr/sleep.h>
#include <SPI.h>
#include "RFM22.h"

#define GPS_BAUDRATE 9600
#define RTTY_PERIOD 1000UL
#define DEBUG_RESET // AVR reset
#define DEBUG_RTTY  //RTTY packet dump
#define RADIO_FREQUENCY 434.700
/*
 0x02  5db (3mW)
 0x03  8db (6mW)
 0x04 11db (12mW)
 0x05 14db (25mW)
 0x06 17db (50mW)
 0x07 20db (100mW)
 */
#define RADIO_POWER 0x05
#define RADIO_REBOOT 20
#define BATTVOLTPIN 1

rfm22 radio1(10);
char datastring[200];
unsigned int pkt_num;
unsigned int numerRamki;
float battvolt = 0;
float tempin = 0;
unsigned long rtty_next_tx_millis;
elapsedMillis czasOdOstatniegoTX;
int gpsPowerSave = 0;

elapsedMillis sinceMSG_poll;

void power_save();
void rtty_txbit(int bit);
void rtty_txbyte(char c);
void rtty_txstring(char *string);
uint16_t gps_CRC16_checksum(char *string);
void rtty_send();
void sendUBX(uint8_t *MSG, uint8_t len);
void resetGPS();
void setupGPS();
void setupGPSpower();
void setGPS_GSVoff();
void setGPS_GLLoff();
void setGPS_VTGoff();
void setGPS_GSAoff();
void setGPS_NMEAoff();
void setGPS_rate();
void setGPS_DynamicModel6();
void setGps_MaxPerformanceMode();
boolean getUBX_ACK(uint8_t *MSG);
void setupRADIO();
void sensors();


void power_save()
{
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();

  //digitalWrite(LED_PIN, HIGH);
  sleep_mode(); // Go to sleep

  sleep_disable(); // Resume after wake up
}

void rtty_txbit(int bit)
{
  if (bit)
  {
    radio1.write(0x73, 0x03); // High0x03
  }
  else
  {
    radio1.write(0x73, 0x00); // Low
  }

  /*
  Baud    bit length  char length  char/sec char/min words/min
  45.5 Bd	22 ms       0.165 s      6	      364	     60
  50 Bd	  20 ms       0.15 s       6.67     400      66
  75 Bd	  13.33 ms    0.1 s        10       600      100
  100 Bd  10 ms       0.075 s      13.33    800      133
  */
  delayMicroseconds(10000); // 100 baud (or 9750)
  //delayMicroseconds(1685); // 300 baud
  //delayMicroseconds(843); // 600 baud
  // delayMicroseconds(20150); // 50 Baud 
}

void rtty_txbyte(char c)
{
  /* Simple function to sent each bit of a char to 
   	** rtty_txbit function. 
   	** NB The bits are sent Least Significant Bit first
   	**
   	** All chars should be preceded with a 0 and 
   	** proceded with a 1. 0 = Start bit; 1 = Stop bit
   	**
   	*/

  int i;

  rtty_txbit(0); // Start bit

  // Send bits for for char LSB first

  for (i = 0; i < 8; i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1)
      rtty_txbit(1);

    else
      rtty_txbit(0);

    c = c >> 1;
  }

  rtty_txbit(1); // Stop bit
  rtty_txbit(1); // Stop bit
}

void rtty_txstring(char *string)
{

  /* Simple function to sent a char at a time to 
   	** rtty_txbyte function. 
   	** NB Each char is one byte (8 Bits)
   	*/

  char c;

  c = *string++;

  while (c != '\0')
  {
    rtty_txbyte(c);
    c = *string++;
  }
}

static inline uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data) __attribute__((always_inline, unused));
static inline uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data)
{
	unsigned int i;

	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++) {
		if (crc & 0x8000) {
			crc = (crc << 1) ^ 0x1021;
		} else {
			crc <<= 1;
		}
	}
	return crc;
}

uint16_t gps_CRC16_checksum(char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;
  // Calculate checksum ignoring the first four $s
  for (i = 4; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update(crc, c);
  }

  return crc;
}

void rtty_send()
{

  char temp1[12];
  char h_str[3];
  char m_str[3];
  char s_str[3];
  strncpy(h_str, gps_time + 0, 2);
  strncpy(m_str, gps_time + 2, 2);
  strncpy(s_str, gps_time + 4, 2);

  pkt_num++;

  strcpy(datastring, "$$$$SP7TECH-1,"); //CALLSIGN

  sprintf(temp1, "%u", pkt_num);
  strcat(datastring, temp1);

  strcat(datastring, ",");

  snprintf(temp1, 3, "%s", h_str);
  strcat(datastring, temp1);

  strcat(datastring, ":");
  snprintf(temp1, 3, "%s", m_str);
  strcat(datastring, temp1);

  strcat(datastring, ":");
  snprintf(temp1, 3, "%s", s_str);
  strcat(datastring, temp1);
  strcat(datastring, ",");

  dtostrf(gps_lat, 7, 6, temp1); // DD.DDDDDD
  strcat(datastring, temp1);
  strcat(datastring, ",");
  dtostrf(gps_lon, 7, 6, temp1); // DD.DDDDDD
  strcat(datastring, temp1);
  strcat(datastring, ",");
  //sprintf(temp1,"%s", gps_altitude);
  snprintf(temp1, 6, "%05ld", (long)(gps_altitude + 0.5)); // wysokosc
  strcat(datastring, temp1);
  strcat(datastring, ",");
  snprintf(temp1, 4, "%03d", (int)(gps_speed + 0.5)); // predkosc
  strcat(datastring, temp1);
  strcat(datastring, ",");
  //snprintf(temp1, 4, "%03d", (int)(gps_course + 0.5));
  //strcat(datastring,temp1);
  //strcat(datastring, ",");

  dtostrf(battvolt, 4, 2, temp1); // napiecie baterii
  strcat(datastring, temp1);
  strcat(datastring, ",");
  dtostrf(tempin, 3, 1, temp1); // temp_in
  strcat(datastring, temp1);
  strcat(datastring, ",");
  snprintf(temp1, 3, "%02d", (int)gps_sat); // ilosc satelitow
  strcat(datastring, temp1);
  strcat(datastring, ",");
  snprintf(temp1, 2, "%01d", (int)gpsPowerSave); // 0 - pelna moc, 1 - energooszczedny
  strcat(datastring, temp1);
  strcat(datastring, ",");
  snprintf(temp1, 2, "%c", active ? 'A' : 'I'); // A - active, I - invalid
  strcat(datastring, temp1);

  // suma kontrolna
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring);
  char checksum_str[6];
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring, checksum_str);

#ifdef DEBUG_RTTY
  Serial.print("RTTY: ");
  Serial.print(datastring);
#endif

  noInterrupts();
  rtty_txstring(datastring);
  interrupts();

  numerRamki++;

  if (numerRamki >= RADIO_REBOOT)
  {
    Serial.println("RADIO: reset");
    numerRamki = 0;
    radio1.write(0x07, 0x80); // radio soft restart
    delay(500);
    setupRADIO();
    delay(500);
    radio1.write(0x07, 0x08); // turn tx on
    delay(2000);
  }
}

void sendUBX(uint8_t *MSG, uint8_t len)
{
  Serial1.flush();
  Serial1.write(0xFF); //preco toto na zaciatku spravy ?
  delay(100);
  for (int i = 0; i < len; i++)
  {
    Serial1.write(MSG[i]);
  }
}

void resetGPS()
{
  /*
  Wymuszony (Watchdog) Zimny start
  */
  uint8_t set_reset[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5};
  sendUBX(set_reset, sizeof(set_reset) / sizeof(uint8_t));
}

void setupGPS()
{
  Serial.print("GPS: Konfiguracja");
  //setGPS_NMEAoff() // test
  setGPS_GLLoff();
  Serial.print('.');
  delay(30);
  setGPS_GSVoff();
  Serial.print('.');
  delay(30);
  setGPS_VTGoff();
  Serial.print('.');
  delay(30);
  setGPS_GSAoff();
  Serial.print('.');
  delay(30);
  setGPS_rate();
  Serial.print('.');
  delay(30);
  setGPS_DynamicModel6();
  Serial.print('.');
  delay(30);
  setGps_MaxPerformanceMode();
  Serial.print('.');
  delay(30);
  Serial.println("OK");
}

void setupGPSpower()
{
  //Set GPS ot Power Save Mode
  uint8_t setPSM[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92}; // Setup for Power Save Mode (Default Cyclic 1s)

  sendUBX(setPSM, sizeof(setPSM) / sizeof(uint8_t));
}

void setGPS_GSVoff()
{
  int gps_set_sucess = 0;
  uint8_t setGSVoff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};

  while (!gps_set_sucess)
  {
    sendUBX(setGSVoff, sizeof(setGSVoff) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setGSVoff);
  }
}

void setGPS_GLLoff()
{
  int gps_set_sucess = 0;
  uint8_t setGLLoff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};

  while (!gps_set_sucess)
  {
    sendUBX(setGLLoff, sizeof(setGLLoff) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setGLLoff);
  }
}

void setGPS_VTGoff()
{

  int gps_set_sucess = 0;
  uint8_t setVTGoff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

  while (!gps_set_sucess)
  {
    sendUBX(setVTGoff, sizeof(setVTGoff) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setVTGoff);
  }
}

void setGPS_GSAoff()
{

  int gps_set_sucess = 0;
  uint8_t setGSAoff[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};

  while (!gps_set_sucess)
  {
    sendUBX(setGSAoff, sizeof(setGSAoff) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setGSAoff);
  }
}

void setGPS_NMEAoff()
{
  int gps_set_sucess = 0;
  uint8_t setNMEAoff[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};

  while (!gps_set_sucess)
  {
    sendUBX(setNMEAoff, sizeof(setNMEAoff) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setNMEAoff);
  }
}

void setGPS_rate()
{

  int gps_set_sucess = 0;
  uint8_t setrate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4, 0x01, 0x01, 0x00, 0x01, 0x00, 0x0B, 0x77};

  while (!gps_set_sucess)
  {
    sendUBX(setrate, sizeof(setrate) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setrate);
  }
}

void setGPS_DynamicModel6()
{
  /*
  CFG-NAV5
  Header: 0xB5, 0x62, 
  ID: 0x06, 0x24, 
  Length 0x24, 0x00, 
  mask 0xFF, 0xFF, 
  dynModel:  0x06, (Airborne <1g)
  fixMode: 0x03, 
  fixedAlt: 0x00, 0x00, 0x00, 0x00, 
  fixedAltVar: 0x10, 0x27, 0x00, 0x00, 
  minElev 0x05, 
  drLimit 0x00, 
  pDop 0xFA, 0x00, 
  tDop 0xFA, 0x00, 
  pAcc 0x64, 0x00, 
  tAcc 0x2C, 0x01, 
  staticHoldThresh 0x00, 
  dgpsTimeOut 0x00, 
  0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00,
  CK_A 0x16, 
  CK_B 0xDC  
  */

  int gps_set_sucess = 0;
  uint8_t setdm6[] = {
      0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
      0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
      0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
      0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  while (!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setdm6);
  }
}

/**
UBX-CFG-RMX - 0 Continuous Mode (Max Performance Mode)
*/
void setGps_MaxPerformanceMode()
{
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = {
      0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91}; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax) / sizeof(uint8_t));
}

boolean getUBX_ACK(uint8_t *MSG)
{
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0;      // CK_A
  ackPacket[9] = 0;      // CK_B

  // Calculate the checksums
  for (uint8_t ubxi = 2; ubxi < 8; ubxi++)
  {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1)
  {

    // Test for success
    if (ackByteID > 9)
    {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000)
    {
      return false;
    }

    // Make sure data is available to read
    if (Serial1.available())
    {
      b = Serial1.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID])
      {
        ackByteID++;
      }
      else
      {
        ackByteID = 0; // Reset and look again, invalid order
      }
    }
  }
}

void setupRADIO()
{
  Serial.print("RADIO: konfiguracja... ");
  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71, 0x00); // niezmodulowana nosna
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b, 0x12);
  radio1.write(0x0c, 0x15);

  // sensor temperatury radia
  radio1.write(0x0f, 0x00);
  radio1.write(0x12, 0x20); //tsrange
  radio1.write(0x13, 0x00); //tvoffs

  radio1.setFrequency(RADIO_FREQUENCY);
  radio1.write(0x6D, RADIO_POWER);
  Serial.println("OK");
}

void sensors()
{
  char temp1[12];
  Serial.println("SENSOR: Odczyt");
  // bateria
  battvolt = analogRead(BATTVOLTPIN);
  battvolt = (battvolt / 1024) * 3.30;
  dtostrf(battvolt, 4, 2, temp1);
  Serial.print("batt=");
  Serial.println(temp1);
  // radio temp sensor
  radio1.write(0x0f, 0x80); // pomiar
  delay(100);
  tempin = (radio1.read(0x11)) * 0.5 - 64; // odczyt
  dtostrf(tempin, 3, 1, temp1); // temperatura radia
  Serial.print("temp_in=");
  Serial.println(temp1);
  Serial.flush();
}

void blink(int times) {
  digitalWrite(LED_BUILTIN, LOW);
  delay(20);
  for (int i = 0; i < times; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(20);               
    digitalWrite(LED_BUILTIN, LOW);  
    delay(80);  
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  blink(8);

  // Porrt USB
  Serial.begin(115200); // Teensy Serial zawsze komunikuje sie z predkoscia 12 Mbit/sec
  while (!Serial && (millis() < 10000))  ; // czekaj az port bedzie otwarty lub 10 sekund

  Serial.println("AMT High Altitude Balloon 2019");
  Serial.println("Starting...");

  // Ustaw predkosc portu GPS
  Serial1.begin(GPS_BAUDRATE);
  while (!Serial1 && (millis() < 10000))  ; // czekaj az port bedzie otwarty lub 10 sekund
  
  setupRADIO();
  resetGPS();
  setupGPS();
  gps_parser_setup();
  delay(100);

  radio1.write(0x07, 0x08); // turn tx on
  czasOdOstatniegoTX = 0;
}

void loop()
{
  int cc;

  // czy czas na kolejne nadawanie?
  if (czasOdOstatniegoTX >= RTTY_PERIOD)
  {
    blink(active ? 1 : 2);
    sensors();
    rtty_send();
    czasOdOstatniegoTX = 0;
  }

  while (Serial1.available())
  {
    cc = Serial1.read();
    gps_decode(cc);

    if (gpsPowerSave == 0 && gps_sat > 5)
    {
      setupGPSpower();
      gpsPowerSave = 1;
      Serial.println("GPS: oszczedzanie baterii");
    }
    if (gpsPowerSave == 1 && gps_sat < 5)
    {
      setGps_MaxPerformanceMode();
      gpsPowerSave = 0;
      Serial.println("GPS: wysoka wydajnosc");
    }
  }

  power_save();
}