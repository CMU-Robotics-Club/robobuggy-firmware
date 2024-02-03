/*
  Read NMEA sentences over I2C using Ublox module SAM-M8Q, NEO-M8P, etc
  By: Nathan Seidle
  SparkFun Electronics
  Date: August 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example reads the NMEA characters over I2C and pipes them to MicroNMEA
  This example will output your current long/lat and satellites in view

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  For more MicroNMEA info see https://github.com/stevemarple/MicroNMEA

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
  Go outside! Wait ~25 seconds and you should see your lat/long
*/
#include <stdstrings.h>
#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GPS
#include <SD.h>

#include <Adafruit_BNO08x.h>

#define BNO_085_INT 20

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

//#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
//SFE_UBLOX_GPS myGPS;

HardwareSerial& gpsSerial = Serial4;

#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

/**  @file

 @brief Universal Transverse Mercator transforms.

 Functions to convert (spherical) latitude and longitude to and
 from (Euclidean) UTM coordinates.

 @author Chuck Gantz- chuck.gantz@globalstar.com
 */

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
// #include "ofMathConstants.h"

namespace UTM
{
  // Grid granularity for rounding UTM coordinates to generate MapXY.
  const double grid_size = 100000.0; ///< 100 km grid

// WGS84 Parameters
#define WGS84_A 6378137.0        ///< major axis
#define WGS84_B 6356752.31424518 ///< minor axis
#define WGS84_F 0.0033528107     ///< ellipsoid flattening
#define WGS84_E 0.0818191908     ///< first eccentricity
#define WGS84_EP 0.0820944379    ///< second eccentricity

  // UTM Parameters
#define UTM_K0 0.9996                   ///< scale factor
#define UTM_FE 500000.0                 ///< false easting
#define UTM_FN_N 0.0                    ///< false northing, northern hemisphere
#define UTM_FN_S 10000000.0             ///< false northing, southern hemisphere
#define UTM_E2 (WGS84_E * WGS84_E)      ///< e^2
#define UTM_E4 (UTM_E2 * UTM_E2)        ///< e^4
#define UTM_E6 (UTM_E4 * UTM_E2)        ///< e^6
#define UTM_EP2 (UTM_E2 / (1 - UTM_E2)) ///< e'^2

  /**
   * Determine the correct UTM letter designator for the
   * given latitude
   *
   * @returns 'Z' if latitude is outside the UTM limits of 84N to 80S
   *
   * Written by Chuck Gantz- chuck.gantz@globalstar.com
   */
  static inline char UTMLetterDesignator(double Lat)
  {
    char LetterDesignator;

    if ((84 >= Lat) && (Lat >= 72))
      LetterDesignator = 'X';
    else if ((72 > Lat) && (Lat >= 64))
      LetterDesignator = 'W';
    else if ((64 > Lat) && (Lat >= 56))
      LetterDesignator = 'V';
    else if ((56 > Lat) && (Lat >= 48))
      LetterDesignator = 'U';
    else if ((48 > Lat) && (Lat >= 40))
      LetterDesignator = 'T';
    else if ((40 > Lat) && (Lat >= 32))
      LetterDesignator = 'S';
    else if ((32 > Lat) && (Lat >= 24))
      LetterDesignator = 'R';
    else if ((24 > Lat) && (Lat >= 16))
      LetterDesignator = 'Q';
    else if ((16 > Lat) && (Lat >= 8))
      LetterDesignator = 'P';
    else if ((8 > Lat) && (Lat >= 0))
      LetterDesignator = 'N';
    else if ((0 > Lat) && (Lat >= -8))
      LetterDesignator = 'M';
    else if ((-8 > Lat) && (Lat >= -16))
      LetterDesignator = 'L';
    else if ((-16 > Lat) && (Lat >= -24))
      LetterDesignator = 'K';
    else if ((-24 > Lat) && (Lat >= -32))
      LetterDesignator = 'J';
    else if ((-32 > Lat) && (Lat >= -40))
      LetterDesignator = 'H';
    else if ((-40 > Lat) && (Lat >= -48))
      LetterDesignator = 'G';
    else if ((-48 > Lat) && (Lat >= -56))
      LetterDesignator = 'F';
    else if ((-56 > Lat) && (Lat >= -64))
      LetterDesignator = 'E';
    else if ((-64 > Lat) && (Lat >= -72))
      LetterDesignator = 'D';
    else if ((-72 > Lat) && (Lat >= -80))
      LetterDesignator = 'C';
    // 'Z' is an error flag, the Latitude is outside the UTM limits
    else
      LetterDesignator = 'Z';
    return LetterDesignator;
  }

  /**
   * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
   *
   * East Longitudes are positive, West longitudes are negative.
   * North latitudes are positive, South latitudes are negative
   * Lat and Long are in fractional degrees
   *
   * Written by Chuck Gantz- chuck.gantz@globalstar.com
   */
  static inline void LLtoUTM(const double Lat, const double Long,
                             double &UTMNorthing, double &UTMEasting,
                             char *UTMZone)
  {
    double a = WGS84_A;
    double eccSquared = UTM_E2;
    double k0 = UTM_K0;

    double LongOrigin;
    double eccPrimeSquared;
    double N, T, C, A, M;

    // Make sure the longitude is between -180.00 .. 179.9
    double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180;

    double LatRad = Lat * DEG_TO_RAD;
    double LongRad = LongTemp * DEG_TO_RAD;
    double LongOriginRad;
    int ZoneNumber;

    ZoneNumber = int((LongTemp + 180) / 6) + 1;

    if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
      ZoneNumber = 32;

    // Special zones for Svalbard
    if (Lat >= 72.0 && Lat < 84.0)
    {
      if (LongTemp >= 0.0 && LongTemp < 9.0)
        ZoneNumber = 31;
      else if (LongTemp >= 9.0 && LongTemp < 21.0)
        ZoneNumber = 33;
      else if (LongTemp >= 21.0 && LongTemp < 33.0)
        ZoneNumber = 35;
      else if (LongTemp >= 33.0 && LongTemp < 42.0)
        ZoneNumber = 37;
    }
    // +3 puts origin in middle of zone
    LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
    LongOriginRad = LongOrigin * DEG_TO_RAD;

    // compute the UTM Zone from the latitude and longitude
    sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));

    eccPrimeSquared = (eccSquared) / (1 - eccSquared);

    N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
    T = tan(LatRad) * tan(LatRad);
    C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
    A = cos(LatRad) * (LongRad - LongOriginRad);

    M = a * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256) * LatRad - (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(2 * LatRad) + (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(4 * LatRad) - (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

    UTMEasting = (double)(k0 * N * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120) + 500000.0);

    UTMNorthing = (double)(k0 * (M + N * tan(LatRad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720)));

    if (Lat < 0)
    {
      // 10000000 meter offset for southern hemisphere
      UTMNorthing += 10000000.0;
    }
  }

  /**
   * Converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
   *
   * East Longitudes are positive, West longitudes are negative.
   * North latitudes are positive, South latitudes are negative
   * Lat and Long are in fractional degrees.
   *
   * Written by Chuck Gantz- chuck.gantz@globalstar.com
   */
  static inline void UTMtoLL(const double UTMNorthing, const double UTMEasting,
                             const char *UTMZone, double &Lat, double &Long)
  {
    double k0 = UTM_K0;
    double a = WGS84_A;
    double eccSquared = UTM_E2;
    double eccPrimeSquared;
    double e1 = (1 - sqrt(1 - eccSquared)) / (1 + sqrt(1 - eccSquared));
    double N1, T1, C1, R1, D, M;
    double LongOrigin;
    double mu, phi1Rad;
    double x, y;
    int ZoneNumber;
    char *ZoneLetter;

    x = UTMEasting - 500000.0; // remove 500,000 meter offset for longitude
    y = UTMNorthing;

    ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
    if ((*ZoneLetter - 'N') < 0)
    {
      // remove 10,000,000 meter offset used for southern hemisphere
      y -= 10000000.0;
    }

    //+3 puts origin in middle of zone
    LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
    eccPrimeSquared = (eccSquared) / (1 - eccSquared);

    M = y / k0;
    mu = M / (a * (1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256));

    phi1Rad = mu + ((3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu) + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu) + (151 * e1 * e1 * e1 / 96) * sin(6 * mu));

    N1 = a / sqrt(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad));
    T1 = tan(phi1Rad) * tan(phi1Rad);
    C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
    R1 = a * (1 - eccSquared) / pow(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad), 1.5);
    D = x / (N1 * k0);

    Lat = phi1Rad - ((N1 * tan(phi1Rad) / R1) * (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * D * D * D * D / 24 + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * eccPrimeSquared - 3 * C1 * C1) * D * D * D * D * D * D / 720));

    Lat = Lat * RAD_TO_DEG;

    Long = ((D - (1 + 2 * T1 + C1) * D * D * D / 6 + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * eccPrimeSquared + 24 * T1 * T1) * D * D * D * D * D / 120) / cos(phi1Rad));
    Long = LongOrigin + Long * RAD_TO_DEG;
  }
} // end namespace UTM

//@requires power>=0;
uint64_t positivePow(uint64_t base, uint64_t power)
{

  uint64_t result = 1;
  while (power > 0)
  {
    result *= base;
    power -= 1;
  }
  return result;
}

// converts day:hour:minute:second:nanosecond to absolute time in nanoseconds
// warning: will break if you run the buggy at midNight on the end of a month;
uint64_t getTimeMillis()
{

  uint64_t n_hun  = nmea.getHundredths();
  uint64_t n_sec  = nmea.getSecond();
  uint64_t n_min  = nmea.getMinute();
  uint64_t n_hour = nmea.getHour();
  uint64_t n_day  = nmea.getDay(); 

  uint64_t total_time =
    n_hun +
    n_sec * 100ull +
    n_min * (100ull * 60ull) +
    n_hour * (100ull * 60ull * 60ull) +
    n_day * (100ull * 60ull * 60ull * 24ull);

  return 10ull * total_time;
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GRAVITY)) {
    Serial.println("Could not enable gravity vector");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
    Serial.println("Could not enable geomagnetic rotation vector");
  }
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game rotation vector");
  }
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    Serial.println("Could not enable raw magnetometer");
  }
}


void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun Ublox Example");

  gpsSerial.begin(38400);

  Wire.begin();

  if (!bno08x.begin_I2C()) {
    while (1)
      Serial.println("BNO085 not detected over I2C. Freezing");
  }

  /*if (!myGPS.begin())
  {
    while (1)
      Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
  }*/

  if (!SD.begin(BUILTIN_SDCARD)) {
    while (1)
      Serial.println("SD card not detected. Freezing");
  }

  setReports();
}

bool led_state = false;

uint64_t last_gps_time = 0;
uint64_t last_local_time = millis();

void loop()
{
  int fileNum = 0;
  char fileName[100];
  while (true) {
    snprintf(fileName, 100, "log%d.txt", fileNum);

    if (!SD.exists(fileName)) {
      break;
    }

    ++fileNum;
  }
  File f = SD.open(fileName, FILE_WRITE);

  if (!f) {
    while (1)
      Serial.println("File not created. Freezing");
  }

  f.printf("------- BEGIN NEW LOG --------\n");

  while (1) {
    //myGPS.checkUblox(); // See if new data is available. Process bytes as they come in.
    while (gpsSerial.available()) {
      if (nmea.process(gpsSerial.read())) {
        bool isGGA = std::strcmp(nmea.getMessageType(),"GGA")==0;
        bool isRMC = std::strcmp(nmea.getMessageType(),"RMC")==0
        if (nmea.isValid() && (isGGA || isRMC)) {
          /*long latitude_mdeg = myGPS.getLatitude();
          long longitude_mdeg = myGPS.getLongitude();*/
          
          long latitude_mdeg = nmea.getLatitude();
          long longitude_mdeg = nmea.getLongitude();
          uint64_t currTimeMillis = getTimeMillis();
          last_gps_time = currTimeMillis;
          last_local_time = millis();

          // Serial.print("Latitude (deg): ");
          // Serial.println(latitude_mdeg / 1000000., 6);
          // Serial.print("Longitude (deg): ");
          // Serial.println(longitude_mdeg / 1000000., 6);
          // Serial.println("Time: ");
          Serial.println(currTimeMillis);

          double x = 0;
          double y = 0;
          char r[] = "T";

          UTM::LLtoUTM(latitude_mdeg / 1000000.0, longitude_mdeg / 1000000.0, x, y, r);

          

          f.printf("x: %f, y: %f, time: %llu\n", x, y, currTimeMillis);

          digitalWrite(LED_BUILTIN, led_state);
          led_state = !led_state;
        }
        else
        {
          Serial.printf("No fix - Num. Satellites: %hhu Time: %d\n", nmea.getNumSatellites(), millis());
          f.printf("No fix - Num. Satellites: %hhu\n", nmea.getNumSatellites());
        }
      }
    }

    if (bno08x.wasReset()) {
      Serial.print("sensor was reset ");
      setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
      Serial.println("Logging IMU event");

      f.printf("IMU %u ", millis() - last_local_time + last_gps_time);
      switch (sensorValue.sensorId) {

      case SH2_ACCELEROMETER:
        f.printf(
          "Accelerometer - x: %f y: %f z: %f\n",
          (double)sensorValue.un.accelerometer.x,
          (double)sensorValue.un.accelerometer.y,
          (double)sensorValue.un.accelerometer.z
        );
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        f.printf(
          "Gyro - x: %f y: %f z: %f\n",
          (double)sensorValue.un.gyroscope.x,
          (double)sensorValue.un.gyroscope.y,
          (double)sensorValue.un.gyroscope.z
        );
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        f.printf(
          "Magnetic Field - x: %f y: %f z: %f\n",
          (double)sensorValue.un.magneticField.x,
          (double)sensorValue.un.magneticField.y,
          (double)sensorValue.un.magneticField.z
        );
        break;
      case SH2_LINEAR_ACCELERATION:
        f.printf(
          "Linear Acceleration - x: %f y: %f z: %f\n",
          (double)sensorValue.un.linearAcceleration.x,
          (double)sensorValue.un.linearAcceleration.y,
          (double)sensorValue.un.linearAcceleration.z
        );
        break;
      case SH2_GRAVITY:
        f.printf(
          "Gravity - x: %f y: %f z: %f\n",
          (double)sensorValue.un.gravity.x,
          (double)sensorValue.un.gravity.y,
          (double)sensorValue.un.gravity.z
        );
        break;
      case SH2_ROTATION_VECTOR:
        f.printf(
          "Rotation Vector - r: %f i: %f j: %f k: %f\n",
          (double)sensorValue.un.rotationVector.real,
          (double)sensorValue.un.rotationVector.i,
          (double)sensorValue.un.rotationVector.j,
          (double)sensorValue.un.rotationVector.k
        );
        break;
      case SH2_GEOMAGNETIC_ROTATION_VECTOR:
        f.printf(
          "Geo-Magnetic Rotation Vector - r: %f i: %f j: %f k: %f\n",
          (double)sensorValue.un.geoMagRotationVector.real,
          (double)sensorValue.un.geoMagRotationVector.i,
          (double)sensorValue.un.geoMagRotationVector.j,
          (double)sensorValue.un.geoMagRotationVector.k
        );
        break;
      case SH2_GAME_ROTATION_VECTOR:
        f.printf(
          "Game Rotation Vector - r: %f i: %f j: %f k: %f\n",
          (double)sensorValue.un.gameRotationVector.real,
          (double)sensorValue.un.gameRotationVector.i,
          (double)sensorValue.un.gameRotationVector.j,
          (double)sensorValue.un.gameRotationVector.k
        );
        break;
      case SH2_RAW_ACCELEROMETER:
        f.printf(
          "Raw Accelerometer - x: %f y: %f z: %f\n",
          (double)sensorValue.un.rawAccelerometer.x,
          (double)sensorValue.un.rawAccelerometer.y,
          (double)sensorValue.un.rawAccelerometer.z
        );
        break;
      case SH2_RAW_GYROSCOPE:
        f.printf(
          "Raw Gyro - x: %f y: %f z: %f\n",
          (double)sensorValue.un.rawGyroscope.x,
          (double)sensorValue.un.rawGyroscope.y,
          (double)sensorValue.un.rawGyroscope.z
        );
        break;
      case SH2_RAW_MAGNETOMETER:
        f.printf(
          "Raw Magnetic Field - x: %f y: %f z: %f\n",
          (double)sensorValue.un.rawMagnetometer.x,
          (double)sensorValue.un.rawMagnetometer.y,
          (double)sensorValue.un.rawMagnetometer.z
        );
        break;
      default:
        break;
      }
    }

    static int flush_cnt = 0;

    if (++flush_cnt >= 100) {
      flush_cnt = 0;
      f.flush();
    }


    delay(5); // Don't pound too hard on the I2C bus
  }
}

// This function gets called from the SparkFun Ublox Arduino Library
// As each NMEA character comes in you can specify what to do with it
// Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
// a buffer, radio, etc.

/*
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  // Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  // for sentence cracking
  nmea.process(incoming);
}
*/