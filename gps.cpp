/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include "config.h"
#include "gps.h"


#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>


PROGMEM const prog_uchar ubx_setNav[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 
  0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 
  0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x16, 0xDC 
};

PROGMEM const prog_uchar ubx_powerOff[] = {
  0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B
};

PROGMEM const prog_uchar ubx_powerOn[] = {
  0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x01, 0x00, 0x00, 0x00, 0x4C, 0x37
};

//Max Performance Mode (default)
PROGMEM const prog_uchar ubx_setMaxPerformance[] = {
  0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91
};

//Power Save Mode
PROGMEM const prog_uchar ubx_setPowerSaveMode[] = {
  0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92
};

// The ISS Kepplerian Elements calculated dataset (starting at Feb. 6th 00:00:00 UTC)
PROGMEM const prog_uint16_t ISSData[] = {
	12565, 11545, 10525
};

#define ISSData_Length 2   // 2 days will stay below 32 Mbyte
// Remember to get ISS data:
// Lookup the UNIX epoch time: http://www.epochconverter.com/
// cd /home/thomas/Documents/hamradio/balloon/ISS/
// perl iss.pl 1335009600 5 2012-04-21_1200UTC.dat > 2012-04-21_1200UTC.tsv




#define ISS_CUT_OFF 2230.0  // Horizont of the ISS in km
//#define getISSLatitude(x)  ((ISSData[x] >> 9) - 64)
//#define getISSLongitude(x) ((ISSData[x] & 511) - 180)

/*
// GPS PUBX request string
#define PUBX "$PUBX,00*33"

// NMEA commands
#define GLL_OFF "$PUBX,40,GLL,0,0,0,0*5C"
#define ZDA_OFF "$PUBX,40,ZDA,0,0,0,0*44"
#define GSV_OFF "$PUBX,40,GSV,0,0,0,0*59"
#define GSA_OFF "$PUBX,40,GSA,0,0,0,0*4E"
#define RMC_OFF "$PUBX,40,RMC,0,0,0,0*47"
#define GGA_OFF "$PUBX,40,GGA,0,0,0,0*5A"
#define VTG_OFF "$PUBX,40,VTG,0,0,0,0*5E"

// configures the GPS to only use the strings we want
void configNMEA() {
  Serial.println(GLL_OFF);
  Serial.println(ZDA_OFF);
  Serial.println(GSV_OFF);
  Serial.println(GSA_OFF);
  Serial.println(RMC_OFF);
  Serial.println(GGA_OFF);
  Serial.println(VTG_OFF);

  // Set the navigation mode (Airborne, 1G)
  Serial.println("Trying to set the GPS to airborne mode:");
  unsigned int setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  sendUBX(setNav, sizeof(setNav)/sizeof(unsigned int));
  getUBX_ACK(setNav);
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(unsigned int  *MSG, unsigned int  len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
bool getUBX_ACK(unsigned int  *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  int startTime = millis();

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;	// CK_A
  ackPacket[9] = 0;	// CK_B

  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
      }
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}

*/

// Module declarations
static void parse_sentence_type(const char * token);
static void parse_time(const char *token);
static void parse_sats(const char *token);
static void parse_date(const char *token);
static void parse_status(const char *token);
static void parse_lat(const char *token);
static void parse_lat_hemi(const char *token);
static void parse_lon(const char *token);
static void parse_lon_hemi(const char *token);
static void parse_speed(const char *token);
static void parse_course(const char *token);
static void parse_altitude(const char *token);



// Module types
typedef void (*t_nmea_parser)(const char *token);

enum t_sentence_type {
  SENTENCE_UNK,
  SENTENCE_GGA,
  SENTENCE_RMC
};


// Module constants
static const t_nmea_parser unk_parsers[] = {
  parse_sentence_type,    // $GPxxx
};

static const t_nmea_parser gga_parsers[] = {
  NULL,             // $GPGGA
  parse_time,       // Time
  NULL,             // Latitude
  NULL,             // N/S
  NULL,             // Longitude
  NULL,             // E/W
  NULL,             // Fix quality 
  parse_sats,       // Number of satellites
  NULL,             // Horizontal dilution of position
  parse_altitude,   // Altitude
  NULL,             // "M" (mean sea level)
  NULL,             // Height of GEOID (MSL) above WGS84 ellipsoid
  NULL,             // "M" (mean sea level)
  NULL,             // Time in seconds since the last DGPS update
  NULL              // DGPS station ID number
};

static const t_nmea_parser rmc_parsers[] = {
  NULL,             // $GPRMC
  parse_time,       // Time
  parse_status,     // A=active, V=void
  parse_lat,        // Latitude,
  parse_lat_hemi,   // N/S
  parse_lon,        // Longitude
  parse_lon_hemi,   // E/W
  parse_speed,      // Speed over ground in knots
  parse_course,     // Track angle in degrees (true)
  parse_date,       // Date (DDMMYY)
  NULL,             // Magnetic variation
  NULL              // E/W
};

static const int NUM_OF_UNK_PARSERS = (sizeof(unk_parsers) / sizeof(t_nmea_parser));
static const int NUM_OF_GGA_PARSERS = (sizeof(gga_parsers) / sizeof(t_nmea_parser));
static const int NUM_OF_RMC_PARSERS = (sizeof(rmc_parsers) / sizeof(t_nmea_parser));

// Module variables
static t_sentence_type sentence_type = SENTENCE_UNK;
static bool at_checksum = false;
static unsigned char our_checksum = '$';
static unsigned char their_checksum = 0;
static char token[16];
static int num_tokens = 0;
static unsigned int offset = 0;
static bool active = false;
static char gga_time[7], rmc_time[7], rmc_date[7];
static char new_time[7], new_date[7];
static int new_sats, gga_sats;
static float new_lat;
static float new_lon;
static char new_aprs_lat[9];
static char new_aprs_lon[10];
static float new_course;
static float new_speed;
static float new_altitude;

// Public (extern) variables, readable from other modules
char gps_time[7];       // HHMMSS
char gps_date[7];       // DDMMYY
int gps_sats = 0;
float gps_lat = 0;
float gps_lon = 0;
char gps_aprs_lat[10];
char gps_aprs_lon[11];
float gps_course = 0;
float gps_speed = 0;
float gps_altitude = -1000.0; //The dead sea at -420 m is theoretically the deepest usable spot on earth where you could use a GPS
                              //Here we define -1000 m as our invalid gps altitude 
unsigned long doppler_frequency = 145825000UL;
bool satInView = false;
int iss_lat;
int iss_lon;
int time2lock;
unsigned int iss_datapoint;

// Module functions
unsigned char from_hex(char a) 
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else if (a >= '0' && a <= '9')
    return a - '0';
  else
    return 0;
}

void parse_sentence_type(const char *token)
{
  if (strcmp(token, "$GPGGA") == 0) {
    sentence_type = SENTENCE_GGA;
  } else if (strcmp(token, "$GPRMC") == 0) {
    sentence_type = SENTENCE_RMC;
  } else {
    sentence_type = SENTENCE_UNK;
  }
}

void parse_time(const char *token)
{
  // Time can have decimals (fractions of a second), but we only take HHMMSS
  strncpy(new_time, token, 6);
}

void parse_date(const char *token)
{
  // Date in DDMMYY
  strncpy(new_date, token, 6);
}

void parse_sats(const char *token)
{
  // Date in DDMMYY
  new_sats = atoi(token);
}


void parse_status(const char *token)
{
  // "A" = active, "V" = void. We shoud disregard void sentences
  if (strcmp(token, "A") == 0)
    active = true;
  else
    active = false;
}

void parse_lat(const char *token)
{
  // Parses latitude in the format "DD" + "MM" (+ ".M{...}M")
  char degs[3];
  if (strlen(token) >= 4) {
    degs[0] = token[0];
    degs[1] = token[1];
    degs[2] = '\0';
    new_lat = atof(degs) + atof(token + 2) / 60;
  }
  // APRS-ready latitude
  strncpy(new_aprs_lat, token, 7);
}

void parse_lat_hemi(const char *token)
{
  if (token[0] == 'S')
    new_lat = -new_lat;
  new_aprs_lat[7] = token[0];
  new_aprs_lon[8] = '\0';
}

void parse_lon(const char *token)
{
  // Longitude is in the format "DDD" + "MM" (+ ".M{...}M")
  char degs[4];
  if (strlen(token) >= 5) {
    degs[0] = token[0];
    degs[1] = token[1];
    degs[2] = token[2];
    degs[3] = '\0';
    new_lon = atof(degs) + atof(token + 3) / 60;
  }
  // APRS-ready longitude
  strncpy(new_aprs_lon, token, 8);
}

void parse_lon_hemi(const char *token)
{
  if (token[0] == 'W')
    new_lon = -new_lon;
  new_aprs_lon[8] = token[0];
  new_aprs_lon[9] = '\0';
}

void parse_speed(const char *token)
{
  new_speed = atof(token);
}

void parse_course(const char *token)
{
  new_course = atof(token);
}

void parse_altitude(const char *token)
{
  new_altitude = atof(token);
}


//
// Exported functions
//
void gps_setup() {
  strcpy(gps_time, "000000");
  strcpy(gps_date, "000000");
  strcpy(gps_aprs_lat, "0000.00N");
  strcpy(gps_aprs_lon, "00000.00E");
  
  
  // Set uBlox navigation mode
  for(int i=0; i<44; i++) {
    Serial.write(pgm_read_byte_near(ubx_setNav + i ));
    
  }
  
  time2lock = 0;  
  
  
}

void gps_off() 
{
  for(int i=0; i<16; i++) {
    Serial.write(pgm_read_byte_near(ubx_powerOff + i ));
  }
  
}

void gps_on() 
{
  for(int i=0; i<16; i++) {
    Serial.write(pgm_read_byte_near(ubx_powerOn + i ));
  }
  
}

void gps_setMaxPerformance() 
{
  for(int i=0; i<10; i++) {
    Serial.write(pgm_read_byte_near(ubx_setMaxPerformance + i ));
  }
  
}

void gps_setPowerSaveMode() 
{
  for(int i=0; i<10; i++) {
    Serial.write(pgm_read_byte_near(ubx_setPowerSaveMode + i ));
  }
  
}


bool gps_decode(char c)
{
  int ret = false;

  switch(c) {
    case '\r':
    case '\n':
      // End of sentence

      if (num_tokens && our_checksum == their_checksum) {
#ifdef DEBUG_GPS
        Serial.print(" (OK!)");
#endif
        // Return a valid position only when we've got two rmc and gga
        // messages with the same timestamp.
        switch (sentence_type) {
          case SENTENCE_UNK:
            break;    // Keeps gcc happy
          case SENTENCE_GGA:
            strcpy(gga_time, new_time);
            gga_sats = new_sats;
            break;
          case SENTENCE_RMC:
            strcpy(rmc_time, new_time);
            strcpy(rmc_date, new_date);
            break;
        }

        // Valid position scenario:
        //
        // 1. The timestamps of the two previous GGA/RMC sentences must match.
        //
        // 2. We just processed a known (GGA/RMC) sentence. Suppose the
        //    contrary: after starting up this module, gga_time and rmc_time
        //    are both equal (they're both initialized to ""), so (1) holds
        //    and we wrongly report a valid position.
        //
        // 3. The GPS has a valid fix. For some reason, the Venus 634FLPX
        //    reports 24 deg N, 121 deg E (the middle of Taiwan) until a valid
        //    fix is acquired:
        //
        //    $GPGGA,120003.000,2400.0000,N,12100.0000,E,0,00,0.0,0.0,M,0.0,M,,0000**69 (OK!)
        //    $GPGSA,A,1,,,,,,,,,,,,,0.0,0.0,0.0**30 (OK!)
        //    $GPRMC,120003.000,V,2400.0000,N,12100.0000,E,000.0,000.0,280606,,,N**78 (OK!)
        //    $GPVTG,000.0,T,,M,000.0,N,000.0,K,N**02 (OK!)
        //
        // 4.) We also want a clean altitude being reported.
        //     When is an altitude clean? Theoretically we could launch a balloon from the dead sea
        //     420 m below sea level. Therefore we define -1000 m as our invalid altitude and
        //     expect to find an altitude above -420 m when we've decoded a valid GGA statement.

        if (sentence_type != SENTENCE_UNK &&      // Known sentence?
            strcmp(gga_time, rmc_time) == 0 &&    // RMC/GGA times match?
            active &&                             // Valid fix?
            gga_sats > 4 &&                       // 5 sats or more?
            new_altitude > -1000.0) {             // Valid new altitude?              
          // Atomically merge data from the two sentences
          strcpy(gps_time, new_time);
          strcpy(gps_date, new_date);
          gps_lat = new_lat;
          gps_lon = new_lon;
          strcpy(gps_aprs_lat, new_aprs_lat);
          strcpy(gps_aprs_lon, new_aprs_lon);
          gps_course = new_course;
          gps_speed = new_speed;
          gps_sats = gga_sats;
          gps_altitude = new_altitude;
          new_altitude = -1000.0; // Invalidate new_altitude so that we are sure to get a valid one next time
          ret = true;
        }
      }
#ifdef DEBUG_GPS
      if (num_tokens)
        Serial.println();
#endif
      at_checksum = false;        // CR/LF signals the end of the checksum
      our_checksum = '$';         // Reset checksums
      their_checksum = 0;
      offset = 0;                 // Prepare for the next incoming sentence
      num_tokens = 0;
      sentence_type = SENTENCE_UNK;
      break;
    
    case '*':
      // Begin of checksum and process token (ie. do not break)
      at_checksum = true;
      our_checksum ^= c;
#ifdef DEBUG_GPS
      Serial.print(c);
#endif

    case ',':
      // Process token
      token[offset] = '\0';
      our_checksum ^= c;  // Checksum the ',', undo the '*'

      // Parse token
      switch (sentence_type) {
        case SENTENCE_UNK:
          if (num_tokens < NUM_OF_UNK_PARSERS && unk_parsers[num_tokens])
            unk_parsers[num_tokens](token);
          break;
        case SENTENCE_GGA:
          if (num_tokens < NUM_OF_GGA_PARSERS && gga_parsers[num_tokens])
            gga_parsers[num_tokens](token);
          break;
        case SENTENCE_RMC:
          if (num_tokens < NUM_OF_RMC_PARSERS && rmc_parsers[num_tokens])
            rmc_parsers[num_tokens](token);
          break;
      }

      // Prepare for next token
      num_tokens++;
      offset = 0;
#ifdef DEBUG_GPS
      Serial.print(c);
#endif
      break;

    default:
      // Any other character
      if (at_checksum) {
        // Checksum value
        their_checksum = their_checksum * 16 + from_hex(c);
      } else {
        // Regular NMEA data
        if (offset < 15) {  // Avoid buffer overrun (tokens can't be > 15 chars)
          token[offset] = c;
          offset++;
          our_checksum ^= c;
        }
      }
#ifdef DEBUG_GPS
      Serial.print(c);
#endif
  }
  return ret;
}


unsigned long gps_get_region_frequency()
{
  unsigned long frequency = RADIO_FREQUENCY;
  if(-168 < gps_lon && gps_lon < -34) frequency = RADIO_FREQUENCY_REGION2;
  if(-34 <  gps_lon && gps_lon <  71) frequency = RADIO_FREQUENCY_REGION1;
  if(-34.95 < gps_lat && gps_lat < 7.18  && -73.13 < gps_lon && gps_lon < -26.46) frequency = RADIO_FREQUENCY_BRAZIL;      // Brazil
  if( 29.38 < gps_lat && gps_lat < 47.10 && 127.16 < gps_lon && gps_lon < 153.61) frequency = RADIO_FREQUENCY_JAPAN;       // Japan
  if( 19.06 < gps_lat && gps_lat < 53.74 &&  72.05 < gps_lon && gps_lon < 127.16) frequency = RADIO_FREQUENCY_CHINA;       // China
  if( -0.30 < gps_lat && gps_lat < 20.42 &&  93.06 < gps_lon && gps_lon < 105.15) frequency = RADIO_FREQUENCY_THAILAND;    // Thailand
  if(-54.54 < gps_lat && gps_lat <-32.43 && 161.62 < gps_lon && gps_lon < 179.99) frequency = RADIO_FREQUENCY_NEWZEALAND;  // New Zealand
  if(-50.17 < gps_lat && gps_lat < -8.66 && 105.80 < gps_lon && gps_lon < 161.62) frequency = RADIO_FREQUENCY_AUSTRALIA;   // Australia
  
  // Note: These regions are a super simplified approach to define rectangles on the world map, representing regions where we may consider at least some
  // chance that an APRS Digipeter or an Igate is listening. They have absolutely NO political relevance. I was just trying to identify
  // regions with APRS activity on APRS.fi and look up the used frequencies on the web. Any corrections or additions are welcome. Please email
  // your suggestions to thomas@tkrahn.com 
  
  // use your own coordinates for testing; Comment out when testing is finished!
//  if(29.7252 < gps_lat && gps_lat < 29.7261 && -95.5082 < gps_lon && gps_lon < -95.5074) frequency = MX146_FREQUENCY_TESTING; // KT5TK home
  //if(29.7353 < gps_lat && gps_lat < 29.7359 && -95.5397 < gps_lon && gps_lon < -95.5392) frequency = MX146_FREQUENCY_TESTING; // Gessner BBQ

  // Note: Never define a region that spans the date line! Use two regions instead.
  if(gps_lat == 0 && gps_lon == 0) frequency = RADIO_FREQUENCY; // switch to default when we don't have a GPS lease
  
  return frequency;
}

bool gps_check_satellite()
{
  int latdiff;
  int londiff;

  long igpsdate = atol(gps_date);
  long igpstime = atol(gps_time);
  
  int gpstime_day     =        igpsdate / 10000;  
  int gpstime_month   =        (igpsdate % 10000) / 100;
  int gpstime_year    =        (igpsdate % 10000) % 100 + 2000;
  int gpstime_hour    =        igpstime / 10000;
  int gpstime_minute  =        (igpstime % 10000) / 100; 
  int gpstime_second  =        (igpstime % 10000) % 100;

  
//  int gpstime_year    = 2012;  
//  int gpstime_month   = 2;
//  int gpstime_day     = 11;
//  int gpstime_hour    = 21;
//  int gpstime_minute  = 0;
//  int gpstime_second  = 0;


// The start time must match with the dataset above!
  long gpslaunchminutes = (gpstime_day - 19 ) * 1440 + (gpstime_hour - 0) * 60 + gpstime_minute - 0;



// look 2 minutes into the future so that you see the constellation for the next TX cycle
  gpslaunchminutes += 2;
  
  // make sure we're in the bounds of the array.
  if ((gpslaunchminutes < 0) || (gpslaunchminutes > ISSData_Length)) // make sure we're in the bounds of the array.
  {
    gpslaunchminutes = 0; 
  }
  
  iss_datapoint = pgm_read_word_near(ISSData + gpslaunchminutes);
  
  // unmerge the datapoint into its components
  
  iss_lat = ((iss_datapoint >> 9) - 64); 
  iss_lon = ((iss_datapoint & 511) - 180);
  
  latdiff = abs(iss_lat - (int)gps_lat);
  londiff = abs(iss_lon - (int)gps_lon);
  
  // ISS nearby? 
  float delLat = abs(iss_lat-gps_lat)*111194.9;
  float delLong = 111194.9*abs(iss_lon-gps_lon)*cos(radians((iss_lat+gps_lat)/2));
  float distance = sqrt(pow(delLat,2)+pow(delLong,2))/1000.0; // Distance between balloon and ISS in km (on the surface of the earth)
  
  
  if (distance < ISS_CUT_OFF) 
  {
    satInView = true;
    
  }
  else
  {
    satInView = false;

  }  
  return satInView;  
}





float gps_get_lat()
{
  return gps_lat; 
}


float gps_get_lon()
{
  return gps_lon; 
}

float gps_get_altitude()
{
  return gps_altitude; 
}


long gps_get_time()
{
  return atol(gps_time);
}

long gps_get_date()
{
  return atol(gps_date);
}

void gpsSetTime2lock( int periods )
{
  time2lock = periods; 
}



