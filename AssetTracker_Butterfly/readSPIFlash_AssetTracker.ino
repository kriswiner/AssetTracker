/* SPIFlash_Ladybug.ino
Sketch by Kris Winer December 16. 2016

License: Use this sketch any way you choose; if you like it, buy me a beer sometime

Purpose: Checks function of a variety of SPI NOR flash memory chips hosted by the STM32L4
Dragonfly (STM32L476), Butterfly (STM32L433), and Ladybug (STML432) development boards or their variants.

Sketch takes advantage of the SPI.beginTransaction/SPI.EndTransaction protocol for efficiency
and maximum speed.

Sketch based on the work of Pete (El Supremo) as follows:
 * Copyright (c) 2014, Pete (El Supremo), el_supremo@shaw.ca
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 */

#include <SPI.h>
#include "SPIFlash.h"

// Highest page number is 0xFFFF = 65535 for 128 Mbit flash
// Highest page number is 0x0EFF =  4095 for   8 Mbit flash
uint16_t max_page_number = 0xFFFF;
unsigned char flashPage[256];
uint16_t page_number = 0;
uint8_t sector_number = 0;
uint8_t buffer[4] = {0, 0, 0, 0};

uint32_t compHumidity, compTemp, compPress;   // pressure, humidity, and temperature raw count output for BME280
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
uint16_t rawVbat;
float VDDA, VBAT;
uint16_t gpsAlt;
float   temperature;         // Stores the real internal chip temperature in degrees Celsius
float latitude, longitude, yaw, pitch, roll;   // Stores CAM M8Q GPS position output
#define csPin 6

SPIFlash SPIFlash(csPin);

void setup(void)
{ 
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  // check SPI Flash ID
  SPIFlash.SPIFlashinit();
  SPIFlash.getChipID();
 
  // read Sensor Tile SPI flash
  for(page_number = 0; page_number < 1000; page_number++)  {

   //  Serial.print("Read Page 0x"); Serial.println(page_number, HEX);
   SPIFlash.flash_read_pages(flashPage, page_number, 1);
      
   for(sector_number = 0; sector_number < 8; sector_number++) {
    
    // reconstruct latitude
    buffer[0] = flashPage[sector_number*32 + 0];
    buffer[1] = flashPage[sector_number*32 + 1];
    buffer[2] = flashPage[sector_number*32 + 2];
    buffer[3] = flashPage[sector_number*32 + 3];

    latitude = (float) ( (int32_t) (buffer[0] << 24) | (int32_t) (buffer[1] << 16) | (int32_t) (buffer[2] << 8) | (int32_t) buffer[4]);
    latitude /= 10000000.0f;
    
    // reconstruct longitude
    buffer[0] = flashPage[sector_number*32 + 4];
    buffer[1] = flashPage[sector_number*32 + 5];
    buffer[2] = flashPage[sector_number*32 + 6];
    buffer[3] = flashPage[sector_number*32 + 7];

    longitude = (float) ( (int32_t) (buffer[0] << 24) | (int32_t) (buffer[1] << 16) | (int32_t) (buffer[2] << 8) | (int32_t) buffer[4]);
    longitude /= 10000000.0f;

    // reconstruct yaw
    buffer[0] = flashPage[sector_number*32 + 8];
    buffer[1] = flashPage[sector_number*32 + 9];

    yaw = (float) ( (int16_t) (buffer[0] << 8) | (int16_t) (buffer[1] ) );
    yaw /= 50.0f;

    // reconstruct pitch
    buffer[0] = flashPage[sector_number*32 + 10];
    buffer[1] = flashPage[sector_number*32 + 11];

    pitch = (float) ( (int16_t) (buffer[0] << 8) | (int16_t) (buffer[1] ) );
    pitch /= 50.0f;

    // reconstruct roll
    buffer[0] = flashPage[sector_number*32 + 12];
    buffer[1] = flashPage[sector_number*32 + 13];

    roll = (float) ( (int16_t) (buffer[0] << 8) | (int16_t) (buffer[1] ) );
    roll /= 50.0f;

    
    // reconstruct pressure
    buffer[0] = flashPage[sector_number*32 + 14];
    buffer[1] = flashPage[sector_number*32 + 15];
    buffer[2] = flashPage[sector_number*32 + 16];
    buffer[3] = flashPage[sector_number*32 + 17];

    pressure = (float) ( (int32_t) (buffer[0] << 24) | (int32_t) (buffer[1] << 16) | (int32_t) (buffer[2] << 8) | (int32_t) buffer[4]);
    pressure /= 10000000.0f;
    pressure = pressure + 1013.25f;  // Pressure in mbar
  
    // reconstruct temperature
    buffer[0] = flashPage[sector_number*32 + 18];
    buffer[1] = flashPage[sector_number*32 + 19];
    buffer[2] = flashPage[sector_number*32 + 20];
    buffer[3] = flashPage[sector_number*32 + 21];

    temperature_C = (float) ( (int32_t) (buffer[0] << 24) | (int32_t) (buffer[1] << 16) | (int32_t) (buffer[2] << 8) | (int32_t) buffer[4]);
    temperature_C /= 10000000.0f;
   
    Seconds = flashPage[sector_number*32 + 22];
    Minutes = flashPage[sector_number*32 + 23];
    Hours =   flashPage[sector_number*32 + 24];
    Day =     flashPage[sector_number*32 + 25];
    Month =   flashPage[sector_number*32 + 26];
    Year =    flashPage[sector_number*32 + 27];

    gpsAlt = ((uint16_t) flashPage[sector_number*32 + 28] << 8) |  flashPage[sector_number*32 + 29];
    
    rawVbat = ((uint16_t) flashPage[sector_number*32 + 30] << 8) |  flashPage[sector_number*32 + 31];

    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
    
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   

    VBAT = (127.0f/100.0f) * 3.30f * ((float)rawVbat)/4095.0f;

    // Output for spreadsheet analysis
    if(Month < 10) {Serial.print("0"); Serial.print(Month);} else Serial.print(Month);
    Serial.print("/");Serial.print(Day); Serial.print("/");Serial.print(Year); Serial.print(" ");
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.print(Seconds);} else Serial.print(Seconds); Serial.print(",");      
 
    Serial.print(pressure, 2); Serial.print(","); Serial.print(temperature_C, 2); Serial.print(",");Serial.print(temperature_F, 2); Serial.print(",");
    Serial.print(latitude, 7); Serial.print(","); Serial.print(longitude, 7); Serial.print(","); 
    Serial.print(yaw, 2); Serial.print(","); Serial.print(pitch, 2); Serial.print(","); Serial.print(roll, 2); Serial.print(","); 
    Serial.print(VBAT, 2);  Serial.print(","); Serial.print(altitude, 2); Serial.print(","); Serial.println(gpsAlt);  
    }
  
  }


}

void loop(void)
{
}
