/* Asset tracker example, for Butterfly
    Uses the GNSS add-on with CAM M8Q found here:

   https://www.tindie.com/products/TleraCorp/gps-add-on-for-dragonfly-and-butterfly/

   Idea is ultra-low power for longest LiPo battery life so I would run this with
   1 - 4 MHz clock speed; this reduction plus use of STM32 stop mode means no serial
   through the USB. That's why there is a low power Sharp TFT display here.

    This example code is in the public domain.
*/
#include <Arduino.h>
#include <Wire.h>
#include "GNSS.h"
#include <RTC.h>
#include <SPI.h>
#include "USFS.h"
#include "SPIFlash.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

// define pins for Sharp LCD display, any pins can be used
uint8_t DSCK  = 12;
uint8_t DMOSI = 11;
uint8_t DSS   = 9;

Adafruit_SharpMem display(DSCK, DMOSI, DSS);

#define BLACK 0
#define WHITE 1

// Butterfly
#define myLed1 A5 // blue led 
#define pps 39
#define enableESP8285 10

bool SerialDebug = true;

uint16_t Hour = 0, Minute = 0, Second = 0, Millisec, Year = 0, Month = 0, Day = 0, Alt = 0;
uint16_t hour = 0, minute = 0, second = 0, year = 0, month = 0, day = 0, millisec;
bool ppsFlag = false, firstSync = false, alarmFlag = false;
uint8_t count = 0, fixType = 0, fixQuality, latBytes[4], longBytes[4], tempBytes[4], pressBytes[4];
uint8_t yawBytes[2], pitchBytes[2], rollBytes[2];
int32_t latOut;

float Temperature, Long, Lat;

// battery voltage monitor definitions
uint16_t rawVbat;
float VDDA, VBAT;
#define VbatMon A3

// MPU9250 variables
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
int16_t tempCount, rawPressure, rawTemperature;            // temperature raw count output
float   temperature, pressure, altitude; // Stores the MPU9250 internal chip temperature in degrees Celsius
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float _beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float _zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float _Kp = 2.0f * 5.0f; // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
float _Ki = 0.0f;

float pitch, yaw, roll, Yaw, Pitch, Roll;
float a12, a22, a31, a32, a33;          // rotation matrix coefficients for Euler angles and gravity components
float A12, A22, A31, A32, A33;            // rotation matrix coefficients for Hardware Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float lin_Ax, lin_Ay, lin_Az;             // Hardware linear acceleration (acceleration with gravity component subtracted)
float _q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // hardware quaternion data register

const uint8_t USFS_intPin = 7;
bool passThru  = false, newEM7180Data = false;

/* Choose EM7180, MPU9250 and MS5637 sample rates and bandwidths
   Choices are:
   accBW, gyroBW 0x00 = 250 Hz, 0x01 = 184 Hz, 0x02 = 92 Hz, 0x03 = 41 Hz, 0x04 = 20 Hz, 0x05 = 10 Hz, 0x06 = 5 Hz, 0x07 = no filter (3600 Hz)
   QRtDiv 0x00, 0x01, 0x02, etc quat rate = gyroRt/(1 + QRtDiv)
   magRt 8 Hz = 0x08 or 100 Hz 0x64
   accRt, gyroRt 1000, 500, 250, 200, 125, 100, 50 Hz enter by choosing desired rate
   and dividing by 10, so 200 Hz would be 200/10 = 20 = 0x14
   sample rate of barometer is baroRt/2 so for 25 Hz enter 50 = 0x32
*/
uint8_t accBW = 0x03, gyroBW = 0x03, QRtDiv = 0x09, magRt = 0x08, accRt = 0x0A, gyroRt = 0x0A, baroRt = 0x32;
/*
   Choose MPU9250 sensor full ranges
   Choices are 2, 4, 8, 16 g for accFS, 250, 500, 1000, and 2000 dps for gyro FS and 1000 uT for magFS expressed as HEX values
*/
uint16_t accFS = 0x08, gyroFS = 0x7D0, magFS = 0x3E8;

USFS USFS(USFS_intPin, passThru);

// 128 MBit (16 MByte) SPI Flash 65,536, 256-byte pages
#define csPin 6 // SPI Flash chip select pin

uint16_t page_number = 0;     // set the page number for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write

SPIFlash SPIFlash(csPin);


void setup()
{
  Serial.begin(38400);
  delay(2000);
  Serial.println("Serial enabled!");

  Wire.begin(TWI_PINS_20_21); // set master mode on pins 20/21, I2C frequency at 400 kHz
  Wire.setClock(400000);

  USFS.I2Cscan();

  pinMode(myLed1, OUTPUT);
  digitalWrite(myLed1, LOW);  // start with blue led off, since active HIGH

  pinMode(enableESP8285, OUTPUT);
  digitalWrite(enableESP8285, LOW);  // disable ESP8285 to start

  // Voltage divider 27K/100K to monitor LiPo battery voltage
  pinMode(VbatMon, INPUT);
  analogReadResolution(12); // take advantage of 12-bit ADCs

  // Set up for data display
  display.begin(); // Initialize the display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer

  // Start device display with ID of sensor
  display.setCursor(0, 10); display.print("Butterfly");
  display.setCursor(0, 20); display.print("CAM M8Q");
  display.setCursor(0, 40); display.print("Concurrent");
  display.setCursor(0, 60); display.print("GNSS");
  display.setCursor(0, 80); display.print("EM7180 + MPU9250");
  display.setCursor(0, 100); display.print("Abs.Orientation");
  display.refresh();
  delay(1000);

  pinMode(pps, INPUT);

  //   while (!Serial) { }

  GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ); // Start GNSS
  while (!GNSS.done()) { } // wait for begin to complete

  GNSS.setConstellation(GNSS.CONSTELLATION_GPS); // choose satellites
  while (!GNSS.done()) { } // wait for set to complete

  GNSS.setSBAS(true); // choose satellites
  while (!GNSS.done()) { } // wait for set to complete

  GNSS.setPeriodic(5, 60, true);  // set periodic wake and sleep mode
  while (!GNSS.done()) { } // wait for set to complete

  USFS.getChipID();        // check ROM/RAM version of EM7180
  USFS.loadfwfromEEPROM(); // load EM7180 firmware from EEPROM
  USFS.initEM7180(accBW, gyroBW, accFS, gyroFS, magFS, QRtDiv, magRt, accRt, gyroRt, baroRt); // set MPU and MS5637 sensor parameters

  // check SPI Flash ID
  SPIFlash.SPIFlashinit();
  SPIFlash.getChipID();
  rawVbat = analogRead(VbatMon);
  VBAT = (127.0f / 100.0f) * 3.30f * ((float)rawVbat) / 4095.0f;
  if (VBAT > 3.70) SPIFlash.flash_chip_erase(true); // full erase only if the battery is still good

  // Set the RTC time
  RTC.setHours(hour);
  RTC.setMinutes(minute);
  RTC.setSeconds(second);
  RTC.setMinutes(minute);

  // Set the RTC date
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);

  int16_t calib = 0;
  RTC.setCalibration(calib);  // clock slow, add pulses, clock fast, subtract pulses

  // Check calibration
  int16_t calreg = RTC.getCalibration();
  Serial.print("Calibration pulses = "); Serial.println(calreg);

  // set alarm to update the RTC every second
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second

  RTC.attachInterrupt(alarmMatch);

  attachInterrupt(pps, CAMM8QintHandler, RISING);

  attachInterrupt(USFS_intPin, EM7180intHandler, RISING);  // define interrupt for INT pin output of EM7180

  USFS.checkEM7180Status();

}

void loop()
{
  /*GNSS*/
  //    if (GNSS.available()) // check if new GNSS data is available
  if (ppsFlag == true)
  {
    ppsFlag = false;
    //    while(!GNSS.available()) { STM32.sleep();}
    delay(50); // delay a bit to allow GNSS data to become available

    GNSSLocation myLocation = GNSS.read(); // read available GNSS data

    if (myLocation) // if there is a fix
    {
      Lat = myLocation.latitude();
      int32_t_float_to_bytes(Lat, &latBytes[0]);
      Long = myLocation.longitude();
      int32_t_float_to_bytes(Long, &longBytes[0]);
      Alt = myLocation.altitude();
      Serial.print("latitude = ");
      Serial.print(Lat, 7);
      Serial.print(", longitude = ");
      Serial.print(Long, 7);
      Serial.print(", altitude = ");
      Serial.print(Alt, 1);
      Serial.print(", satellites = ");
      Serial.print(myLocation.satellites());
      Serial.print(", pdop = ");
      Serial.print(myLocation.pdop());
      Serial.print(", fixType = ");
      fixType = myLocation.fixType();
      if (fixType == 0) Serial.print("none");
      if (fixType == 1) Serial.print("time");
      if (fixType == 2) Serial.print("2D");
      if (fixType == 3) Serial.print("3D");
      Serial.print(", fixQuality = ");
      fixQuality = myLocation.fixQuality();
      if (fixQuality == 0) Serial.print("none");
      if (fixQuality == 1) Serial.print("auto");
      if (fixQuality == 2) Serial.print("diff");
      if (fixQuality == 3) Serial.print("prec");
      if (fixQuality == 4) Serial.print("rtk_fixed");
      if (fixQuality == 5) Serial.print("rtk_float");
      if (fixQuality == 6) Serial.print("est");
      if (fixQuality == 7) Serial.print("man");
      if (fixQuality == 8) Serial.print("sim");
      Serial.println();

      Hour   = myLocation.hour();
      Minute = myLocation.minute();
      Second = myLocation.second();
      Millisec = myLocation.millis();
      Serial.print("GNSS Time = ");
      if (Hour < 10)   {
        Serial.print("0");
        Serial.print(Hour);
      } else Serial.print(Hour);
      Serial.print(":");
      if (Minute < 10) {
        Serial.print("0");
        Serial.print(Minute);
      } else Serial.print(Minute);
      Serial.print(":");
      if (Second < 10) {
        Serial.print("0");
        Serial.print(Second);
      } else Serial.print(Second);
      Serial.print(":");
      if (Millisec < 10) {
        Serial.print("0");
        Serial.println(Millisec);
      } else Serial.println(Millisec);

      Year = myLocation.year();
      Month = myLocation.month();
      Day = myLocation.day();
      Serial.print("GNSS Date = ");
      Serial.print(Year); Serial.print(":"); Serial.print(Month); Serial.print(":"); Serial.println(Day);
      Serial.println();

      // Test if the RTC has been synced after GNSS time available
      if (firstSync == false)
      {
        firstSync = true;
        syncRTC();  // just need to sync once
      }

      // Send some data to the SPI flash
      if (sector_number < 8 && page_number < 0xFFFF) { // 65,536 256-byte pages in a 16 MByte flash
        flashPage[sector_number * 32 + 0]  = latBytes[0];  // latitude in bytes
        flashPage[sector_number * 32 + 1]  = latBytes[1];
        flashPage[sector_number * 32 + 2]  = latBytes[2];
        flashPage[sector_number * 32 + 3]  = latBytes[3];
        flashPage[sector_number * 32 + 4]  = longBytes[0]; // longitude in bytes
        flashPage[sector_number * 32 + 5]  = longBytes[1];
        flashPage[sector_number * 32 + 6]  = longBytes[2];
        flashPage[sector_number * 32 + 7]  = longBytes[3];
        flashPage[sector_number * 32 + 8]  = yawBytes[0];   // heading
        flashPage[sector_number * 32 + 9]  = yawBytes[1];
        flashPage[sector_number * 32 + 10] = pitchBytes[0]; // pitch
        flashPage[sector_number * 32 + 11] = pitchBytes[1];
        flashPage[sector_number * 32 + 12] = rollBytes[0]; // roll
        flashPage[sector_number * 32 + 13] = rollBytes[1];
        flashPage[sector_number * 32 + 14] = pressBytes[0];
        flashPage[sector_number * 32 + 15] = pressBytes[1];
        flashPage[sector_number * 32 + 16] = pressBytes[2];
        flashPage[sector_number * 32 + 17] = pressBytes[3];
        flashPage[sector_number * 32 + 18] = tempBytes[0];
        flashPage[sector_number * 32 + 19] = tempBytes[1];
        flashPage[sector_number * 32 + 20] = tempBytes[2];
        flashPage[sector_number * 32 + 21] = tempBytes[3];
        flashPage[sector_number * 32 + 22] = Second;
        flashPage[sector_number * 32 + 23] = Minute;
        flashPage[sector_number * 32 + 24] = Hour;
        flashPage[sector_number * 32 + 25] = Day;
        flashPage[sector_number * 32 + 26] = Month;
        flashPage[sector_number * 32 + 27] = (uint8_t) (Year - 2000);
        flashPage[sector_number * 32 + 28] = (Alt & 0xFF00) >> 8; // MSB GPS altitude
        flashPage[sector_number * 32 + 29] =  Alt & 0x00FF;       // LSB GPS altitude
        flashPage[sector_number * 32 + 30] = (rawVbat & 0xFF00) >> 8; // battery voltage
        flashPage[sector_number * 32 + 31] =  rawVbat & 0x00FF;
        sector_number++;
      }
      else if (sector_number == 8 && page_number < 0xFFFF)
      {
        SPIFlash.flash_page_program(flashPage, page_number);
        Serial.print("Wrote flash page: "); Serial.println(page_number);
        sector_number = 0;
        page_number++;
      }
      else
      {
        Serial.println("Reached last page of SPI flash!"); Serial.println("Data logging stopped!");
      }

    }
  }


  /*EM7180*/
  // If intpin goes high, all data registers have new data
  if (newEM7180Data == true) { // On interrupt, read data
    newEM7180Data = false;  // reset newData flag

    // Check event status register, way to chech data ready by polling rather than interrupt
    uint8_t eventStatus = USFS.checkEM7180Status(); // reading clears the register

    // Check for errors
    if (eventStatus & 0x02) { // error detected, what is it?

      uint8_t errorStatus = USFS.checkEM7180Errors();
      if (errorStatus != 0x00) { // is there an error?
        Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
        if (errorStatus == 0x11) Serial.print("Magnetometer failure!");
        if (errorStatus == 0x12) Serial.print("Accelerometer failure!");
        if (errorStatus == 0x14) Serial.print("Gyro failure!");
        if (errorStatus == 0x21) Serial.print("Magnetometer initialization failure!");
        if (errorStatus == 0x22) Serial.print("Accelerometer initialization failure!");
        if (errorStatus == 0x24) Serial.print("Gyro initialization failure!");
        if (errorStatus == 0x30) Serial.print("Math error!");
        if (errorStatus == 0x80) Serial.print("Invalid sample rate!");
      }

      // Handle errors ToDo

    }

    // if no errors, see if new data is ready
    if (eventStatus & 0x10) { // new acceleration data available
      USFS.readSENtralAccelData(accelCount);

      // Now we'll calculate the accleration value into actual g's
      ax = (float)accelCount[0] * 0.000488f; // get actual g value
      ay = (float)accelCount[1] * 0.000488f;
      az = (float)accelCount[2] * 0.000488f;
    }

    if (eventStatus & 0x20) { // new gyro data available
      USFS.readSENtralGyroData(gyroCount);

      // Now we'll calculate the gyro value into actual dps's
      gx = (float)gyroCount[0] * 0.153f; // get actual dps value
      gy = (float)gyroCount[1] * 0.153f;
      gz = (float)gyroCount[2] * 0.153f;
    }

    if (eventStatus & 0x08) { // new mag data available
      USFS.readSENtralMagData(magCount);

      // Now we'll calculate the mag value into actual G's
      mx = (float)magCount[0] * 0.305176f; // get actual G value
      my = (float)magCount[1] * 0.305176f;
      mz = (float)magCount[2] * 0.305176f;
    }

    if (eventStatus & 0x04) { // new quaternion data available
      USFS.readSENtralQuatData(Q);
    }

    // get MS5637 pressure
    if (eventStatus & 0x40) { // new baro data available
      rawPressure = USFS.readSENtralBaroData();
      pressure = (float)rawPressure * 0.01f + 1013.25f; // pressure in mBar
      int32_t_float_to_bytes(pressure - 1013.25f, &pressBytes[0]);

      // get MS5637 temperature
      rawTemperature = USFS.readSENtralTempData();
      temperature = (float) rawTemperature * 0.01f; // temperature in degrees C
      int32_t_float_to_bytes(temperature, &tempBytes[0]);
    }
  }


  /*RTC*/
  if (alarmFlag) { // update RTC output whenever there is a GNSS pulse
    alarmFlag = false;

    VDDA = STM32.getVREF();
    Temperature = STM32.getTemperature();

    Serial.print("VDDA = "); Serial.println(VDDA, 2);
    Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);

    rawVbat = analogRead(VbatMon);
    VBAT = (127.0f / 100.0f) * 3.30f * ((float)rawVbat) / 4095.0f;
    Serial.print("VBAT = "); Serial.println(VBAT, 2);

    hour   = RTC.getHours();
    minute = RTC.getMinutes();
    second = RTC.getSeconds();
    millisec = RTC.getTicks();
    Serial.print("RTC Time = ");
    if (hour < 10)   {
      Serial.print("0");
      Serial.print(hour);
    } else Serial.print(hour);
    Serial.print(":");
    if (minute < 10) {
      Serial.print("0");
      Serial.print(minute);
    } else Serial.print(minute);
    Serial.print(":");
    if (second < 10) {
      Serial.print("0");
      Serial.print(second);
    } else Serial.print(second);
    Serial.print(":");
    if (millisec < 10) {
      Serial.print("0");
      Serial.println(millisec);
    } else Serial.println(millisec);

    year = RTC.getYear();
    month = RTC.getMonth();
    day = RTC.getDay();
    Serial.print("RTC Date = ");
    Serial.print(year); Serial.print(":"); Serial.print(month); Serial.print(":"); Serial.println(day);
    Serial.println();
    //     }

    if (SerialDebug) {
      Serial.print("ax = "); Serial.print((int)1000 * ax);
      Serial.print(" ay = "); Serial.print((int)1000 * ay);
      Serial.print(" az = "); Serial.print((int)1000 * az); Serial.println(" mg");
      Serial.print("gx = "); Serial.print( gx, 2);
      Serial.print(" gy = "); Serial.print( gy, 2);
      Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
      Serial.print("mx = "); Serial.print( (int)mx);
      Serial.print(" my = "); Serial.print( (int)my);
      Serial.print(" mz = "); Serial.print( (int)mz); Serial.println(" mG");

      Serial.println("Hardware quaternions:");
      Serial.print("Q0 = "); Serial.print(Q[0]);
      Serial.print(" Qx = "); Serial.print(Q[1]);
      Serial.print(" Qy = "); Serial.print(Q[2]);
      Serial.print(" Qz = "); Serial.println(Q[3]);
    }

    //Hardware AHRS:
    A12 =   2.0f * (Q[1] * Q[2] + Q[0] * Q[3]);
    A22 =   Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
    A31 =   2.0f * (Q[0] * Q[1] + Q[2] * Q[3]);
    A32 =   2.0f * (Q[1] * Q[3] - Q[0] * Q[2]);
    A33 =   Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];
    Pitch = -asinf(A32);
    Roll  = atan2f(A31, A33);
    Yaw   = atan2f(A12, A22);
    Pitch *= 180.0f / pi;
    int16_t_float_to_bytes(Pitch, &pitchBytes[0]);
    Yaw   *= 180.0f / pi;
    Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if (Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
    int16_t_float_to_bytes(Yaw, &yawBytes[0]);
    Roll  *= 180.0f / pi;
    int16_t_float_to_bytes(Roll, &rollBytes[0]);
    lin_Ax = ax + a31;
    lin_Ay = ay + a32;
    lin_Az = az - a33;

    if (SerialDebug) {
      Serial.print("Hardware Yaw, pitch, Roll: ");
      Serial.print(Yaw, 2);
      Serial.print(", ");
      Serial.print(Pitch, 2);
      Serial.print(", ");
      Serial.println(Roll, 2);

      Serial.print("Hardware Grav_x, Grav_y, Grav_z: ");
      Serial.print(-A31 * 1000, 2);
      Serial.print(", ");
      Serial.print(-A32 * 1000, 2);
      Serial.print(", ");
      Serial.print(A33 * 1000, 2);  Serial.println(" mg");
      Serial.print("Hardware Lin_ax, Lin_ay, Lin_az: ");
      Serial.print(lin_Ax * 1000, 2);
      Serial.print(", ");
      Serial.print(lin_Ay * 1000, 2);
      Serial.print(", ");
      Serial.print(lin_Az * 1000, 2);  Serial.println(" mg");

      Serial.println("MS5637:");
      Serial.print("Altimeter temperature = ");
      Serial.print( temperature, 2);
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = ");
      Serial.print(9.0f * temperature / 5.0f + 32.0f, 2);
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = ");
      Serial.print(pressure, 2);
      Serial.println(" mbar");// pressure in millibar
      altitude = 145366.45f * (1.0f - pow(((pressure) / 1013.25f), 0.190284f));
      Serial.print("Altitude = ");
      Serial.print(altitude, 2);
      Serial.println(" feet");
      Serial.println(" ");
    }

    // set up TFT display data
    display.clearDisplay();
    display.setCursor(0, 10); display.print("lat "); display.print(Lat, 5);
    display.setCursor(0, 20); display.print("lon "); display.print(Long, 5);
    display.setCursor(0, 30); display.print("alt "); display.print(Alt, 1); display.print(" m");
    display.print(" fix "); display.print(fixType);
    display.setCursor(0, 40); display.print("GPS ");
    if (Hour < 10)   {
      display.print("0");
      display.print(Hour);
    } else display.print(Hour);
    display.print(":");
    if (Minute < 10) {
      display.print("0");
      display.print(Minute);
    } else display.print(Minute);
    display.print(":");
    if (Second < 10) {
      display.print("0");
      display.print(Second);
    } else display.print(Second);
    display.setCursor(0, 50);
    display.print(Year); display.print(":"); display.print(Month); display.print(":"); display.print(Day);
    display.print(" q "); display.print(fixQuality);
    display.setCursor(0, 60); display.print("RTC ");
    if (hour < 10)   {
      display.print("0");
      display.print(hour);
    } else display.print(hour);
    display.print(":");
    if (minute < 10) {
      display.print("0");
      display.print(minute);
    } else display.print(minute);
    display.print(":");
    if (second < 10) {
      display.print("0");
      display.print(second);
    } else display.print(second);

    display.setCursor(0, 70);
    display.print("Y"); display.print(Yaw, 1);
    display.print(" P"); display.print(Pitch, 1);
    display.print(" R"); display.print(Roll, 1);

    display.setCursor(0, 80);
    display.print("P "); display.print(pressure, 1); display.print(" mb ");
    display.print(" T "); display.print(temperature, 1); display.print(" C ");
    display.setCursor(0, 90);
    display.print("alt "); display.print(altitude, 1); display.print(" ft");

    display.setCursor(0, 100);
    display.print("VDDA "); display.print(VDDA, 2); display.print(" V");
    display.print(" T "); display.print(Temperature, 1); display.print(" C");

    display.setCursor(0, 110);
    display.print("VBAT "); display.print(VBAT, 2); display.print(" V");

    display.refresh();
    digitalWrite(myLed1, HIGH); delay(1); digitalWrite(myLed1, LOW);
  }

  STM32.stop();  // sleep until next interrupt
}
/* end of loop*/

/* Useful functions */
void EM7180intHandler()
{
  newEM7180Data = true;
}

void CAMM8QintHandler()
{
  ppsFlag = true;
}

void alarmMatch()
{
  alarmFlag = true;
}

void syncRTC()
{
  // Set the time
  RTC.setSeconds(Second);
  RTC.setMinutes(Minute);
  if (Hour < 7) {
    RTC.setHours(Hour + 17);
  } else RTC.setHours(Hour - 7);
  RTC.setMinutes(Minute);

  // Set the date
  if (Hour < 7) {
    RTC.setDay(Day - 1);
  } else RTC.setDay(Day);
  RTC.setMonth(Month);
  RTC.setYear(Year - 2000);
}

void int32_t_float_to_bytes(float temp, uint8_t * dest)
{
  int32_t tempOut = temp * 10000000;
  dest[0] = (tempOut & 0xFF000000) >> 24;
  dest[1] = (tempOut & 0x00FF0000) >> 16;
  dest[2] = (tempOut & 0x0000FF00) >> 8;
  dest[3] = (tempOut & 0x000000FF);
}

void int16_t_float_to_bytes(float temp, uint8_t * dest)
{
  int32_t tempOut = temp * 50;
  dest[0] = (tempOut & 0xFF00) >> 8;
  dest[1] = (tempOut & 0x00FF);
}

