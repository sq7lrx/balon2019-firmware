#ifndef __GPS_H__
#define __GPS_H__

extern char gps_time[7];       // HHMMSS
extern char gps_date[7];       // DDMMYY
extern float gps_lat;
extern float gps_lon;
extern char gps_rtty_lat[10];
extern char gps_rtty_lon[11];
extern float gps_course;
extern float gps_speed;
extern float gps_altitude;
extern char gps_sat;
extern bool active;

void gps_parser_setup();
bool gps_decode(char c);

#endif