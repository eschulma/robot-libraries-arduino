/*
 2-16-2013
 Spark Fun Electronics
 Nathan Seidle
 
 This code is heavily based on maxbot's and IlBaboomba's code: http://arduino.cc/forum/index.php?topic=126244
 They didn't have a license on it so I'm hoping it's public domain.
 
 This example shows how to read and calculate the 64 temperatures for the 64 pixels of the MLX90620 thermopile sensor.
 
 alpha_ij array is specific to every sensor and needs to be calculated separately. Please see the 
 'MLX90620_alphaCalculator' sketch to get these values. If you choose not to calculate these values
 this sketch will still work but the temperatures shown will be very inaccurate.
 
 Don't get confused by the bottom view of the device! The GND pin is connected to the housing.
 
 To get this code to work, attached a MLX90620 to an Arduino Uno using the following pins:
 A5 to 330 ohm to SCL -- use different pins for Mega, connect the SDA and SCL pins to 20 and 21
 A4 to 330 ohm to SDA -- see above
 3.3V to VDD
 GND to VSS
 
 I used the internal pull-ups on the SDA/SCL lines. Normally you should use ~4.7k pull-ups for I2C.
 ESK: must use 4.7k pullups to VDD!

 */

#include <i2cmaster.h>
//i2cmaster comes from here: http://www.cheap-thermocam.bplaced.net/software/I2Cmaster.rar

#include "Arduino.h"
#include <math.h>

#include "MLX90620_registers.h"
#include "MLX90620.h"

const float MLX90620::alpha_ij[64] = {
		  1.85901E-8, 1.98125E-8, 1.82409E-8, 1.56797E-8, 2.09766E-8, 2.21408E-8, 2.09766E-8, 1.78334E-8,
		  2.27229E-8, 2.44691E-8, 2.29557E-8, 1.99871E-8, 2.42945E-8, 2.64481E-8, 2.48765E-8, 2.13841E-8,
		  2.56914E-8, 2.80198E-8, 2.62735E-8, 2.28975E-8, 2.66228E-8, 2.84272E-8, 2.74377E-8, 2.42945E-8,
		  2.72048E-8, 2.95914E-8, 2.87765E-8, 2.54586E-8, 2.76123E-8, 2.93585E-8, 2.90093E-8, 2.58661E-8,
		  2.78451E-8, 2.97660E-8, 2.87765E-8, 2.54586E-8, 2.76123E-8, 2.99988E-8, 2.90093E-8, 2.64481E-8,
		  2.68556E-8, 2.95914E-8, 2.95914E-8, 2.60407E-8, 2.62735E-8, 2.91839E-8, 2.90093E-8, 2.62735E-8,
		  2.54586E-8, 2.87765E-8, 2.84272E-8, 2.60407E-8, 2.48765E-8, 2.74377E-8, 2.76123E-8, 2.56914E-8,
		  2.33049E-8, 2.66228E-8, 2.66228E-8, 2.50512E-8, 2.17333E-8, 2.44691E-8, 2.44691E-8, 2.25482E-8,
		};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Begin Program code

void MLX90620::setup()
{
  loopCount = 0;

  i2c_init(); //Init the I2C pins
  // PORTC = (1 << PORTC4) | (1 << PORTC5); //Enable pull-ups // ESK: irrelevant as these are the wrong pins anyway
  
  delay(5); //Init procedure calls for a 5ms delay after power-on

  read_EEPROM_MLX90620(); //Read the entire EEPROM

  setConfiguration(refreshRate); //Configure the MLX sensor with the user's choice of refresh rate

  calculate_TA(); //Calculate the current Tambient
}

void MLX90620::fillTemps() {
    calculate_TA(); //Calculate the new Tambient

    if(checkConfig_MLX90620()) // check that the POR flag is not set
    {
      // Serial.println("POR Detected!");
      setConfiguration(refreshRate); //Re-write the configuration bytes to the MLX
    }

    readIR_MLX90620(); //Get the 64 bytes of raw pixel data into the irData array

    calculate_TO(); //Run all the large calculations to get the temperature data for each pixel
}

//From the 256 bytes of EEPROM data, initialize 
void MLX90620::varInitialization(byte calibration_data[])
{
  v_th = 256 * calibration_data[VTH_H] + calibration_data[VTH_L];
  k_t1 = (256 * calibration_data[KT1_H] + calibration_data[KT1_L]) / 1024.0; //2^10 = 1024
  k_t2 = (256 * calibration_data[KT2_H] + calibration_data[KT2_L]) / 1048576.0; //2^20 = 1,048,576
  emissivity = ((unsigned int)256 * calibration_data[CAL_EMIS_H] + calibration_data[CAL_EMIS_L]) / 32768.0;
  
  a_cp = calibration_data[CAL_ACP];
  if(a_cp > 127) a_cp -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

  b_cp = calibration_data[CAL_BCP];
  if(b_cp > 127) b_cp -= 256;

  tgc = calibration_data[CAL_TGC];
  if(tgc > 127) tgc -= 256;

  b_i_scale = calibration_data[CAL_BI_SCALE];

  for(int i = 0 ; i < 64 ; i++)
  {
    //Read the individual pixel offsets
    a_ij[i] = calibration_data[i]; 
    if(a_ij[i] > 127) a_ij[i] -= 256; //These values are stored as 2's compliment. This coverts it if necessary.

    //Read the individual pixel offset slope coefficients
    b_ij[i] = calibration_data[0x40 + i]; //Bi(i,j) begins 64 bytes into EEPROM at 0x40
    if(b_ij[i] > 127) b_ij[i] -= 256;
  }
  
}

//Receives the refresh rate for sensor scanning
//Sets the two byte configuration registers
//This function overwrites what is currently in the configuration registers
//The MLX doesn't seem to mind this (flags are read only)
void MLX90620::setConfiguration(int irRefreshRateHZ)
{
  byte Hz_LSB;

  switch(irRefreshRateHZ)
  {
  case 0:
    Hz_LSB = 0b00001111;
    break;
  case 1:
    Hz_LSB = 0b00001110;
    break;
  case 2:
    Hz_LSB = 0b00001101;
    break;
  case 4:
    Hz_LSB = 0b00001100;
    break;
  case 8:
    Hz_LSB = 0b00001011;
    break;
  case 16:
    Hz_LSB = 0b00001010;
    break;
  case 32:
    Hz_LSB = 0b00001001;
    break;
  default:
    Hz_LSB = 0b00001110;
  }

  byte defaultConfig_H = 0b01110100; // x111.01xx, Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz

  i2c_start_wait(MLX90620_WRITE);
  i2c_write(0x03); //Command = configuration value
  i2c_write((byte)Hz_LSB - 0x55);
  i2c_write(Hz_LSB);
  i2c_write(defaultConfig_H - 0x55); //Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz
  i2c_write(defaultConfig_H);
  i2c_stop();
}

//Read the 256 bytes from the MLX EEPROM and setup the various constants (*lots* of math)
//Note: The EEPROM on the MLX has a different I2C address from the MLX. I've never seen this before.
void MLX90620::read_EEPROM_MLX90620()
{
  i2c_start_wait(MLX90620_EEPROM_WRITE);
  i2c_write(0x00); //EEPROM info starts at location 0x00
  i2c_rep_start(MLX90620_EEPROM_READ);

  //Read all 256 bytes from the sensor's EEPROM
  for(int i = 0 ; i <= 255 ; i++)
    eepromData[i] = i2c_readAck();

  i2c_stop(); //We're done talking

  varInitialization(eepromData); //Calculate a bunch of constants from the EEPROM data

  writeTrimmingValue(eepromData[OSC_TRIM_VALUE]);
}

//Given a 8-bit number from EEPROM (Slave address 0x50), write value to MLX sensor (Slave address 0x60)
void MLX90620::writeTrimmingValue(byte val)
{
  i2c_start_wait(MLX90620_WRITE); //Write to the sensor
  i2c_write(0x04); //Command = write oscillator trimming value
  i2c_write((byte)val - 0xAA);
  i2c_write(val);
  i2c_write(0x56); //Always 0x56
  i2c_write(0x00); //Always 0x00
  i2c_stop();
}

//Gets the latest PTAT (package temperature ambient) reading from the MLX
//Then calculates a new Tambient
//Many of these values (k_t1, v_th, etc) come from varInitialization and EEPROM reading
//This has been tested to match example 7.3.2
void MLX90620::calculate_TA(void)
{
   float lk_t1 = k_t1;	// if I don't do this, the compile fails! ESK

   unsigned int ptat = readPTAT_MLX90620();
   Tambient = (-lk_t1 + sqrt(square(lk_t1) - (4 * k_t2 * (v_th - (float)ptat)))) / (2*k_t2) + 25; //it's much more simple now, isn't it? :)
}

//Reads the PTAT data from the MLX
//Returns an unsigned int containing the PTAT
unsigned int MLX90620::readPTAT_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read PTAT
  i2c_write(0x90); //Start address is 0x90
  i2c_write(0x00); //Address step is 0
  i2c_write(0x01); //Number of reads is 1
  i2c_rep_start(MLX90620_READ);

  byte ptatLow = i2c_readAck(); //Grab the lower and higher PTAT bytes
  byte ptatHigh = i2c_readAck();

  i2c_stop();
  
  return( (unsigned int)(ptatHigh << 8) | ptatLow); //Combine bytes and return
}

//Calculate the temperatures seen for each pixel
//Relies on the raw irData array
//Returns an 64-int array called temperatures
void MLX90620::calculate_TO()
{
  float v_ir_off_comp;
  float v_ir_tgc_comp;
  float v_ir_comp;

  //Calculate the offset compensation for the one compensation pixel
  //This is a constant in the TO calculation, so calculate it here.
  int cpix = readCPIX_MLX90620(); //Go get the raw data of the compensation pixel
  float v_cp_off_comp = (float)cpix - (a_cp + (b_cp/pow(2, b_i_scale)) * (Tambient - 25)); 

  for (int i = 0 ; i < 64 ; i++)
  {
    v_ir_off_comp = irData[i] - (a_ij[i] + (float)(b_ij[i]/pow(2, b_i_scale)) * (Tambient - 25)); //#1: Calculate Offset Compensation 

    v_ir_tgc_comp = v_ir_off_comp - ( ((float)tgc/32) * v_cp_off_comp); //#2: Calculate Thermal Gradien Compensation (TGC)

    v_ir_comp = v_ir_tgc_comp / emissivity; //#3: Calculate Emissivity Compensation

    temperatures[i] = sqrt( sqrt( (v_ir_comp/alpha_ij[i]) + pow(Tambient + 273.15, 4) )) - 273.15;
  }
}

//Reads 64 bytes of pixel data from the MLX
//Loads the data into the irData array
void MLX90620::readIR_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read a register
  i2c_write(0x00); //Start address = 0x00
  i2c_write(0x01); //Address step = 1
  i2c_write(0x40); //Number of reads is 64
  i2c_rep_start(MLX90620_READ);

  for(int i = 0 ; i < 64 ; i++)
  {
    byte pixelDataLow = i2c_readAck();
    byte pixelDataHigh = i2c_readAck();
    irData[i] = (int)(pixelDataHigh << 8) | pixelDataLow;
  }

  i2c_stop();
}

//Read the compensation pixel 16 bit data
int MLX90620::readCPIX_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read register
  i2c_write(0x91);
  i2c_write(0x00);
  i2c_write(0x01);
  i2c_rep_start(MLX90620_READ);

  byte cpixLow = i2c_readAck(); //Grab the two bytes
  byte cpixHigh = i2c_readAck();
  i2c_stop();

  return ( (int)(cpixHigh << 8) | cpixLow);
}

//Reads the current configuration register (2 bytes) from the MLX
//Returns two bytes
unsigned int MLX90620::readConfig_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE); //The MLX configuration is in the MLX, not EEPROM
  i2c_write(CMD_READ_REGISTER); //Command = read configuration register
  i2c_write(0x92); //Start address
  i2c_write(0x00); //Address step of zero
  i2c_write(0x01); //Number of reads is 1

    i2c_rep_start(MLX90620_READ);

  byte configLow = i2c_readAck(); //Grab the two bytes
  byte configHigh = i2c_readAck();

  i2c_stop();

  return( (unsigned int)(configHigh << 8) | configLow); //Combine the configuration bytes and return as one unsigned int
}

//Poll the MLX for its current status
//Returns true if the POR/Brown out bit is set
boolean MLX90620::checkConfig_MLX90620()
{
  if ( (readConfig_MLX90620() & (unsigned int)1<<POR_TEST) == 0)
    return true;
  else
    return false;
}

//Prints the temperatures in a way that's more easily viewable in the terminal window
void MLX90620::prettyPrintTemperatures()
{
  Serial.println();
  for(int i = 0 ; i < 64 ; i++)
  {
    if(i % 16 == 0) Serial.println();
    Serial.print(convertToFahrenheit(temperatures[i]));
    //Serial.print(irData[i]);
    Serial.print(", ");
  }
}

//Prints the temperatures in a way that's more easily parsed by a Processing app
//Each line starts with '$' and ends with '*'
void MLX90620::rawPrintTemperatures()
{
  Serial.print("$");
  for(int i = 0 ; i < 64 ; i++)
  {
    Serial.print(convertToFahrenheit(temperatures[i]));
    Serial.print(","); //Don't print comma on last temperature
  }
  Serial.println("*");
}

//Given a Celsius float, converts to Fahrenheit
float MLX90620::convertToFahrenheit (float Tc)
{
  float Tf = (9/5) * Tc + 32;

  return(Tf);
}

/**
 * Returns the average of the 3 highest temperature readings.
 */
float MLX90620::getMaxTemperature() {
	float maxTemp[3] = { 0 };

	fillTemps();

	for(int i = 0; i < 64; i++) {
		for(int j = 0; j < 3; j++) {
			if(temperatures[i] > maxTemp[j]) {
				maxTemp[j] = temperatures[i];
				break;
			}
		}
	}

	float avgMax = (maxTemp[0] + maxTemp[1] + maxTemp[2])/3.0;
	return avgMax;
}
