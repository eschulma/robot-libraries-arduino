#ifndef MLX90620_H
#define MLX90620_H

#include "Arduino.h"

class MLX90620 {
private:
	const static int refreshRate = 16; //Set this value to your desired refresh frequency
	byte loopCount; //Used in main loop
	const static float alpha_ij[64];

	float Tambient; //Tracks the changing ambient temperature of the sensor
	int irData[64]; //Contains the raw IR data from the sensor
	float temperatures[64]; //Contains the calculated temperatures of each pixel in the array (Celsius)
	byte eepromData[256]; //Contains the full EEPROM reading from the MLX (Slave 0x50)

	//These are constants calculated from the calibration data stored in EEPROM
	//See varInitialize and section 7.3 for more information
	int v_th, a_cp, b_cp, tgc, b_i_scale;
	float k_t1;
	float k_t2;
	float emissivity;
	int a_ij[64];
	int b_ij[64];

	void varInitialization(byte calibration_data[]);
	void setConfiguration(int irRefreshRateHZ);
	void read_EEPROM_MLX90620();
	void writeTrimmingValue(byte val);
	void calculate_TA(void);
	unsigned int readPTAT_MLX90620();
	void calculate_TO();
	void readIR_MLX90620();
	int readCPIX_MLX90620();
	unsigned int readConfig_MLX90620();
	boolean checkConfig_MLX90620();

public:
	MLX90620() {};	// caller is responsible for calling the setup function before use!
	void setup();

	void fillTemps();
	float getMaxTemperature();
	void prettyPrintTemperatures();
	void rawPrintTemperatures();
	float convertToFahrenheit (float Tc);
};

#endif
