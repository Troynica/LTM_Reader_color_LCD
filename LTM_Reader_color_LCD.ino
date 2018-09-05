// Modified from https://github.com/DzikuVx/ltm_telemetry_reader/blob/master/ltm_telemetry_reader.ino


#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
//#include <SoftwareSerial.h>
//#include "printf.h"

#define TFT_CS     10
#define TFT_RST    8  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     9
#define TFT_BL     6

#define DISPL_MAX 15
#define DISPL_DIM 245

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

//SoftwareSerial ltmSerial(8, 9);

String fixTypes[3] = {
  " NO ",
  " 2D ",
  " 3D "
};

uint16_t color1, color2;

void setup() {
  //Serial.begin(4800);
  Serial.begin(100000,SERIAL_8E2);
  //ltmSerial.begin(9600);

  tft.initR(INITR_GREENTAB);      // initialize a ST7735S chip
  tft.setRotation(3);
  tft.fillScreen(rgbval(0,0,0));
  tft.setTextColor(ST7735_WHITE);
  tft.setTextWrap(true);

  tft.fillScreen(rgbval(0,0,0));
  color1= rgbval(76,76,76);
  color2 = rgbval(120,120,120);
  tft.fillRect(0,0,127,45, color1);
  tft.drawRect(0,0,127,45, color2);

  tft.setCursor(4,4);
  tft.print("Lat:");
  tft.setCursor(4,14);
  tft.print("Lon:");
  tft.setCursor(4,24);
  tft.print("HDOP:");
  tft.setCursor(72, 24);
  tft.print("Sat:");
  tft.setCursor(4, 34);
  tft.print("Fix:");

  color1= rgbval(100,100,10);
  color2 = rgbval(200,200,20);
  tft.fillRect(0,49,42,31, color1);
  tft.drawRect(0,49,42,31, color2);
  tft.setCursor(4, 51);
  tft.print("GS:");

  color1= rgbval(100,20,100);
  color2 = rgbval(200,40,200);
  tft.fillRect(44,49,82,31, color1);
  tft.drawRect(44,49,82,31, color2);
  tft.setCursor(49, 51);
  tft.print("Alt:");

  
  tft.setCursor(4, 82);
  tft.print("Roll:");
  tft.setCursor(4, 92);
  tft.print("Ptch:");
  tft.setCursor(4, 102);
  tft.print("Hdg :");
  //tft.setCursor(4,118);
  //tft.print("Mode:"); 
  tft.setCursor(72, 92);
  tft.print("RSSI:");
  tft.setCursor(72, 102);
  tft.print("Vbat:");

//  printf_begin();

  analogWrite(TFT_BL, DISPL_MAX);
}

enum ltmStates {
  IDLE,
  HEADER_START1,
  HEADER_START2,
  HEADER_MSGTYPE,
  HEADER_DATA
};

#define LONGEST_FRAME_LENGTH 18

/*
 * LTM based on https://github.com/KipK/Ghettostation/blob/master/GhettoStation/LightTelemetry.cpp implementation
 */

#define GFRAMELENGTH 18
#define AFRAMELENGTH 10
#define SFRAMELENGTH 11
#define OFRAMELENGTH 18
#define NFRAMELENGTH 10
#define XFRAMELENGTH 10

const char* flightModes[] = {
  "    MANUAL    ",
  "     RATE     ",
  "    ANGLE     ",
  "   HORIZON    ",
  "     ACRO     ",
  " STABILIZED 1 ",
  " STABILIZED 2 ",
  " STABILIZED 3 ",
  "   ALT HOLD   ",
  "   GPS HOLD   ",
  "   WAYPOINS   ",
  "   HEADFREE   ",
  "    CIRCLE    ",
  "      RTH     ",
  "  FOLLOW ME   ",
  "     LAND     ",
  " FLY BY WIRE A",
  " FLY BY WIRE B",
  "    CRUISE    ",
  "   UNKNOWN    "
};

typedef struct remoteData_s {
  int pitch;
  int roll;
  int heading;
  uint16_t voltage;
  byte rssi;
  bool armed;
  bool failsafe;
  byte flightmode;

  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  uint8_t groundSpeed; 
  int16_t hdop;
  uint8_t gpsFix;
  uint8_t gpsSats;

  int32_t homeLatitude;
  int32_t homeLongitude;

  uint8_t sensorStatus;
} remoteData_t;

remoteData_t remoteData;

uint8_t serialBuffer[LONGEST_FRAME_LENGTH];
uint8_t state = IDLE;
char frameType;
byte frameLength;
byte receiverIndex;

byte readByte(uint8_t offset) {
  return serialBuffer[offset];
}

int readInt(uint8_t offset) {
  return (int) serialBuffer[offset] + ((int) serialBuffer[offset + 1] << 8);
}

int32_t readInt32(uint8_t offset) {
  return (int32_t) serialBuffer[offset] + ((int32_t) serialBuffer[offset + 1] << 8) + ((int32_t) serialBuffer[offset + 2] << 16) + ((int32_t) serialBuffer[offset + 3] << 24);
}

uint32_t nextDisplay = 0;


/*************************************************************************
 * //Function to calculate the distance between two waypoints
 *************************************************************************/
float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
    float dist_calc=0;
    float dist_calc2=0;
    float diflat=0;
    float diflon=0;

    //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
    diflat=radians(flat2-flat1);
    flat1=radians(flat1);
    flat2=radians(flat2);
    diflon=radians((flon2)-(flon1));

    dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
    dist_calc2= cos(flat1);
    dist_calc2*=cos(flat2);
    dist_calc2*=sin(diflon/2.0);
    dist_calc2*=sin(diflon/2.0);
    dist_calc +=dist_calc2;

    dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

    dist_calc*=6371000.0; //Converting to meters
    //Serial.println(dist_calc);
    return dist_calc;
}


void loop() {

  if (millis() >= nextDisplay) {
    writeDisplay();
    nextDisplay = millis() + 200;
  }
  
  if (Serial.available()) {

    char data = Serial.read();

    if (state == IDLE) {
      if (data == '$') {
        state = HEADER_START1;
      }
    } else if (state == HEADER_START1) {
      if (data == 'T') {
        state = HEADER_START2;
      } else {
        state = IDLE;
      }
    } else if (state == HEADER_START2) {
      frameType = data;
      state = HEADER_MSGTYPE;
      receiverIndex = 0;

      switch (data) {

        case 'G':
          frameLength = GFRAMELENGTH;
          break;
        case 'A':
          frameLength = AFRAMELENGTH;
          break;
        case 'S':
          frameLength = SFRAMELENGTH;
          break;
        case 'O':
          frameLength = OFRAMELENGTH;
          break;
        case 'N':
          frameLength = NFRAMELENGTH;
          break;
        case 'X':
          frameLength = XFRAMELENGTH;
          break;
        default:
          state = IDLE;
      }

    } else if (state == HEADER_MSGTYPE) {

      /*
       * Check if last payload byte has been received.
       */
      if (receiverIndex == frameLength - 4) {
        /*
         * If YES, check checksum and execute data processing
         */

        if (frameType == 'A') {
            remoteData.pitch = readInt(0);
            remoteData.roll = readInt(2);
            remoteData.heading = readInt(4);
        }

        if (frameType == 'S') {
            remoteData.voltage = readInt(0);
            remoteData.rssi = readByte(4);

            byte raw = readByte(6);
            remoteData.flightmode = raw >> 2;
        }

        if (frameType == 'G') {
            remoteData.latitude = readInt32(0);
            remoteData.longitude = readInt32(4);
            remoteData.groundSpeed = readByte(8);
            remoteData.altitude = readInt32(9);

            uint8_t raw = readByte(13);
            remoteData.gpsSats = raw >> 2;
            remoteData.gpsFix = raw & 0x03;
        }

        if (frameType == 'X') {
            remoteData.hdop = readInt(0);
            remoteData.sensorStatus = readByte(2);
        }
        
        state = IDLE;
        memset(serialBuffer, 0, LONGEST_FRAME_LENGTH);

      } else {
        /*
         * If no, put data into buffer
         */
        serialBuffer[receiverIndex++] = data;
      }

    }

  }

}


void writeDisplay() {

  int32_t  latitude_    = remoteData.latitude;
  int32_t  longitude_   = remoteData.longitude;
  int16_t  hdop_        = remoteData.hdop;
  byte     gpsFix_      = remoteData.gpsFix;
  int      gpsSats_     = remoteData.gpsSats;
  uint8_t  groundSpeed_ = remoteData.groundSpeed;
  int16_t  altitude_    = remoteData.altitude;
  int16_t  altitude_e   = abs(altitude_) / 100;
  int16_t  altitude_d   = abs(altitude_) % 100;
  int      roll_        = remoteData.roll;
  int      pitch_       = remoteData.pitch;
  int      heading_     = remoteData.heading;
  uint16_t rssi_        = (remoteData.rssi*100)/230;
  uint16_t voltage_     = remoteData.voltage;
  byte     voltage_e    = voltage_/1000;
  uint16_t voltage_d    = (voltage_%1000)/100;
  byte     flightmode_  = remoteData.flightmode;
  bool     armed_       = remoteData.armed;
  bool     failsafe_    = remoteData.failsafe;


//
// Sanitize
//

//if (latitude_  > 180  || latitude_  < -180 ) latitude_  = 0;
//if (longitude_ > 180  || longitude_ < -180 ) longitude_ = 0;
  if (hdop_      > 9999 || hdop_      <    0 ) hdop_      = 9999;
  if (gpsFix_    > 2                         ) gpsFix_    = 0;
  if (gpsSats_   > 99   || gpsSats_   <    0 ) gpsSats_   = 0;
  if (roll_      > 180  || roll_      < -180 ) roll_      = 0;
  if (pitch_     > 180  || pitch_     < -180 ) pitch_     = 0;
  if (heading_   > 360  || heading_   <    0 ) heading_   = 0;
  if (voltage_e  > 30                        ) voltage_e  = 99;
  if (voltage_d  > 9                         ) voltage_d  = 9;
  if (flightmode_ > 19                       ) flightmode_ = 19;

  color1= rgbval(76,76,76);

  // Print lat / lon
  tft.setTextColor(ST7735_WHITE,20);
  tft.setCursor( 38, 4);
  tft.fillRect(  38, 3,80,9,20);
  tft.print(latitude_);
  tft.setCursor( 38,14);
  tft.fillRect(  38,13,80,9,20);
  tft.print(longitude_);

  // Print HDOP value
  tft.setTextColor(ST7735_BLUE,color1);
  tft.setCursor( 38,24);
  if ( hdop_ > 3 ) {
    tft.setTextColor(ST7735_YELLOW,ST7735_RED);
  } else {
    tft.setTextColor(ST7735_GREEN,color1);
  }
  if ( hdop_ < 10 ) tft.print(" ");
  if ( hdop_ < 100 ) tft.print(" ");
  if ( hdop_ < 1000 ) tft.print(" ");
  tft.print(hdop_);
  tft.print(" ");

  // Print GPS fix
  tft.setCursor( 38,34);
  if (gpsFix_ != 2) {
    tft.setTextColor(ST7735_YELLOW,ST7735_RED);
  } else {
    tft.setTextColor(ST7735_GREEN,color1);
  }
  tft.print(fixTypes[gpsFix_]);

  // Print # of GPS sats
  tft.setCursor(100,24);
  if (gpsSats_ < 6) {
    tft.setTextColor(ST7735_YELLOW,ST7735_RED);
  } else {
    tft.setTextColor(ST7735_GREEN,color1);
  }
  tft.print(" ");
  tft.print(gpsSats_);
  tft.print(" ");


  color1= rgbval(100,100,10);
  color2 = rgbval(250,250,20);
  tft.setTextColor(color2,color1);
  tft.setTextSize(2);
  tft.setCursor( 4,62);
  if ( groundSpeed_ < 10   ) tft.print(" ");
  if ( groundSpeed_ < 100  ) tft.print(" ");
  tft.print(groundSpeed_);

  color1= rgbval(100,20,100);
  color2 = rgbval(250,40,250);
  tft.setTextColor(color2,color1);
  tft.setCursor(46,62);
  
  if ( altitude_e < 10  ) tft.print(" ");
  if ( altitude_e < 100 ) tft.print(" ");
  if ( altitude_e < 1000) tft.print(" ");
  if ( altitude_ >= 0 ) tft.print("+") ; else tft.print("-");
  tft.print(altitude_e);
  tft.setTextSize(1);
  tft.setCursor(106,69);
  tft.print(",") ;
  if ( altitude_d < 10 ) tft.print("0");
  tft.print(altitude_d);


  // Print roll/pitch/yaw
  tft.setTextColor(ST7735_YELLOW,0);
  tft.setCursor( 32,82);
  if ( roll_ >= 0 ) tft.print(" ");
  if ( roll_ < 10 && roll_ > -10) tft.print(" ");
  if ( roll_ < 100 && roll_ > -100) tft.print(" ");
  tft.print(roll_);
  tft.setCursor( 32,92);
  if ( pitch_ >= 0 ) tft.print(" ");
  if ( pitch_ < 10 && pitch_  >  -10) tft.print(" ");
  if ( pitch_ < 100 && pitch_ > -100) tft.print(" ");
  tft.print(pitch_);
  tft.setCursor( 38,102);
  if ( heading_ < 10 ) tft.print(" ");
  if ( heading_ < 100 ) tft.print(" ");
  tft.print(heading_);

  tft.setCursor(100, 92);
  if ( rssi_ < 10  ) {
    tft.setTextColor(ST7735_YELLOW,ST7735_RED);
    tft.print(" ");
  } else {
    tft.setTextColor(ST7735_GREEN,ST7735_BLACK);
  }
  if ( rssi_ < 100 ) tft.print(" ");
  tft.print(rssi_);
  tft.print("%");


  tft.setCursor(100, 102);
  if (voltage_ < 13400) {
    tft.setTextColor(ST7735_YELLOW,ST7735_RED);
  } else if ( voltage_ < 14000) {
    tft.setTextColor(ST7735_RED,ST7735_YELLOW);
  } else {
    tft.setTextColor(ST7735_GREEN,ST7735_BLACK);
  }
  if ( voltage_e < 10  ) tft.print(" ");
  tft.print(voltage_e);
  tft.print(",");
  tft.print(voltage_d);

  tft.setTextColor(ST7735_YELLOW,ST7735_BLUE);
  tft.setCursor( 25,118);
  tft.print(flightModes[flightmode_]);

  tft.setCursor( 2,118);
  if (armed_) {
    tft.setTextColor(ST7735_WHITE,ST7735_BLUE);
  } else {
    tft.setTextColor(ST7735_YELLOW,0);
  }
  tft.print("ARM");

  tft.setCursor( 115,118);
  if (failsafe_) {
    tft.setTextColor(ST7735_YELLOW,ST7735_RED);
  } else {
    tft.setTextColor(ST7735_GREEN,0);
  }
  tft.print("FS");


}


uint16_t rgbval(byte r, byte g, byte b) {
  return( ((r/8)*2048) + ((g/4)*32) + (b/8) );
}

