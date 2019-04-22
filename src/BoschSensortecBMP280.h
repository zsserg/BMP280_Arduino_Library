
#ifndef __BMP280_H__
#define __BMP280_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <SPI.h>

//Uncomment the following line to enable software I2C
//You will need to have the SoftwareWire library installed
//#include <SoftwareWire.h>

//Communication mode
#define I2C_MODE 0
#define SPI_MODE 1

// I2C sub-mode
#define HARD_WIRE 1
#define SOFT_WIRE 2

// Sensor operation mode
#define MODE_SLEEP 0b00
#define MODE_FORCED 0b01
#define MODE_NORMAL 0b11

//----------Register names------------------
//Calibration registers
#define BMP280_DIG_T1_LSB_REG			0x88
#define BMP280_DIG_T1_MSB_REG			0x89
#define BMP280_DIG_T2_LSB_REG			0x8A
#define BMP280_DIG_T2_MSB_REG			0x8B
#define BMP280_DIG_T3_LSB_REG			0x8C
#define BMP280_DIG_T3_MSB_REG			0x8D
#define BMP280_DIG_P1_LSB_REG			0x8E
#define BMP280_DIG_P1_MSB_REG			0x8F
#define BMP280_DIG_P2_LSB_REG			0x90
#define BMP280_DIG_P2_MSB_REG			0x91
#define BMP280_DIG_P3_LSB_REG			0x92
#define BMP280_DIG_P3_MSB_REG			0x93
#define BMP280_DIG_P4_LSB_REG			0x94
#define BMP280_DIG_P4_MSB_REG			0x95
#define BMP280_DIG_P5_LSB_REG			0x96
#define BMP280_DIG_P5_MSB_REG			0x97
#define BMP280_DIG_P6_LSB_REG			0x98
#define BMP280_DIG_P6_MSB_REG			0x99
#define BMP280_DIG_P7_LSB_REG			0x9A
#define BMP280_DIG_P7_MSB_REG			0x9B
#define BMP280_DIG_P8_LSB_REG			0x9C
#define BMP280_DIG_P8_MSB_REG			0x9D
#define BMP280_DIG_P9_LSB_REG			0x9E
#define BMP280_DIG_P9_MSB_REG			0x9F
//General IO registers
#define BMP280_CHIP_ID_REG				0xD0 //Chip ID; 0x58 for BMP280
#define BMP280_RST_REG					0xE0 //Softreset Reg; write 0xB6 to reset the device
#define BMP280_STAT_REG					0xF3 //Status Reg
#define BMP280_CTRL_MEAS_REG			0xF4 //Ctrl Measure Reg
#define BMP280_CONFIG_REG				0xF5 //Configuration Reg
#define BMP280_PRESSURE_MSB_REG			0xF7 //Pressure MSB
#define BMP280_PRESSURE_LSB_REG			0xF8 //Pressure LSB
#define BMP280_PRESSURE_XLSB_REG		0xF9 //Pressure XLSB
#define BMP280_TEMPERATURE_MSB_REG		0xFA //Temperature MSB
#define BMP280_TEMPERATURE_LSB_REG		0xFB //Temperature LSB
#define BMP280_TEMPERATURE_XLSB_REG		0xFC //Temperature XLSB

//Class SensorSettings.  This object is used to hold settings data.  The application
//uses this classes' data directly.  The settings are adopted and sent to the sensor
//
//This is a kind of bloated way to do this.  The trade-off is that the user doesn't
//need to deal with #defines or enums with bizarre names.
//
//A power user would strip out SensorSettings entirely, and send specific read and
//write command directly to the IC. (ST #defines below)
//
struct SensorSettings
{
  public:
	
	//Main Interface and mode settings
    uint8_t commInterface;
    uint8_t I2CAddress;
    uint8_t chipSelectPin;
	
	uint8_t runMode;
	uint8_t tStandby;
	uint8_t filter;
	uint8_t tempOverSample;
	uint8_t pressOverSample;
    float tempCorrection; // correction of temperature - added to the result
};

//Used to hold the calibration constants. These are used
//toc alculate pressure/temperature settings from raw ADC values.
struct SensorCalibration
{
  public:
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
};

//This is the main operational class of the driver.

class BMP280
{
  public:
    //settings
    SensorSettings settings;
	SensorCalibration calibration;
	
	//Constructor generates default SensorSettings.
	//(over-ride after construction if desired)
    BMP280(void);
	
	//Call to apply SensorSettings.
	//This also reads the SensorCalibration constants
    uint8_t begin(void);
    
	//Communication settings
	bool beginSPI(uint8_t csPin); //Communicate using SPI
    bool beginI2C(TwoWire &wirePort = Wire); //Called when user provides Wire port
#ifdef SoftwareWire_h
	bool beginI2C(SoftwareWire &wirePort); //Called when user provides a softwareWire port
#endif //SoftwareWire_h

	void setI2CAddress(uint8_t i2caddress); //Set the address the library should use to communicate. Use if address jumper is closed (0x76).

    //Register read/write functions
	//Reads a chunk of memory of size <length> starting from <offset> into uint8_t array starting at <data_pointer>.
    void readRegisterRegion(uint8_t* data_pointer, uint8_t offset, uint8_t length);
	//Reads one register
    uint8_t readRegister(uint8_t offset);
	//Writes a byte into register
    void writeRegister(uint8_t offset, uint8_t data);

	//Device configuration functions
	uint8_t getMode(void); //Get the current mode: sleep, forced, or normal
	void setMode(uint8_t mode); //Set the current mode
	void reset(void); //Software reset routine
	void setStandbyTime(uint8_t timeSetting); //Set the standby time between measurements
	void setTempOverSample(uint8_t overSampleAmount); //Set the temperature sample mode
	void setPressureOverSample(uint8_t overSampleAmount); //Set the pressure sample mode
	void setFilter(uint8_t filterSetting); //Set the filter

	//Measurement functions
	bool isMeasuring(void); //Returns true while the device is taking measurement
	void setReferencePressure(float refPressure); //Allows user to set local sea level reference pressure
	float getReferencePressure();
	float readPressure(void);
	float readAltitudeMeters(void);
	float readAltitudeFeet(void);
    float readTempC(void);
    float readTempF(void);
	
private:
	uint8_t checkOversampleValue(uint8_t userValue); //Checks for valid over sample values
    uint8_t _wireType = HARD_WIRE; //Default to Wire.h
    TwoWire *_hardPort; //The generic connection to user's chosen I2C hardware
    
#ifdef SoftwareWire_h
	SoftwareWire *_softPort; //Or, the generic connection to software wire port
#endif //SoftwareWire_h
	
	float _referencePressure = 101325.0; //Default but is changeable
	int32_t _t_fine; //Raw temperature value used for pressure calculation formula
};

#endif  // End of __BMP280_H__ definition check