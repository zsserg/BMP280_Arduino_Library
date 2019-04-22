
#include "BoschSensortecBMP280.h"

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//

//Constructor -- Specifies default configuration
BMP280::BMP280(void)
{
	//Construct with these default settings

	settings.commInterface = I2C_MODE; //Default to I2C

	settings.I2CAddress = 0x77; //Default, jumper open is 0x77
	_hardPort = &Wire; //Default to Wire port

	settings.chipSelectPin = 10; //Select CS pin for SPI
	
	settings.runMode = 3; //Normal/Run
	settings.tStandby = 0; //0.5ms
	settings.filter = 0; //Filter off
	settings.tempOverSample = 1;
	settings.pressOverSample = 1;
    settings.tempCorrection = 0.0; // correction of temperature - added to the result
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to 
//  configure before calling .begin();
//
//****************************************************************************//
uint8_t BMP280::begin()
{
	delay(2);  //Make sure sensor had enough time to turn on. BMP280 requires 2ms to start up.

	//Check the settings structure values to determine how to setup the device
	switch (settings.commInterface)
	{

	case I2C_MODE:
		
		switch(_wireType)
		{
			case(HARD_WIRE):
				_hardPort->begin(); //The caller can begin their port and set the speed. We just confirm it here otherwise it can be hard to debug.
				break;
			case(SOFT_WIRE):
			#ifdef SoftwareWire_h
				_softPort->begin(); //The caller can begin their port and set the speed. We just confirm it here otherwise it can be hard to debug.
			#endif
				break;
		}
		break;

	case SPI_MODE:
		// start the SPI library:
		SPI.begin();
		#ifdef ARDUINO_ARCH_ESP32
		SPI.setFrequency(1000000);
		// Data is read and written MSb first.
		SPI.setBitOrder(SPI_MSBFIRST);
		// Like the standard arduino/teensy comment below, mode0 seems wrong according to standards
		// but conforms to the timing diagrams when used for the ESP32
		SPI.setDataMode(SPI_MODE0);
		#else
		// Maximum SPI frequency is 10MHz, could divide by 2 here:
		SPI.setClockDivider(SPI_CLOCK_DIV32);
		// Data is read and written MSb first.
		SPI.setBitOrder(MSBFIRST);
		// Data is captured on rising edge of clock (CPHA = 0)
		// Base value of the clock is HIGH (CPOL = 1)
		// This was SPI_MODE3 for RedBoard, but I had to change to
		// MODE0 for Teensy 3.1 operation
		SPI.setDataMode(SPI_MODE3);
		#endif
		// initialize the  data ready and chip select pins:
		pinMode(settings.chipSelectPin, OUTPUT);
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}

	//Check communication with IC before anything else
	uint8_t chipID = readRegister(BMP280_CHIP_ID_REG); //Should return 0x58
	if(chipID != 0x58) return(false);

	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	calibration.dig_T1 = ((uint16_t)((readRegister(BMP280_DIG_T1_MSB_REG) << 8) + readRegister(BMP280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((readRegister(BMP280_DIG_T2_MSB_REG) << 8) + readRegister(BMP280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((readRegister(BMP280_DIG_T3_MSB_REG) << 8) + readRegister(BMP280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((readRegister(BMP280_DIG_P1_MSB_REG) << 8) + readRegister(BMP280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((readRegister(BMP280_DIG_P2_MSB_REG) << 8) + readRegister(BMP280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((readRegister(BMP280_DIG_P3_MSB_REG) << 8) + readRegister(BMP280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((readRegister(BMP280_DIG_P4_MSB_REG) << 8) + readRegister(BMP280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((readRegister(BMP280_DIG_P5_MSB_REG) << 8) + readRegister(BMP280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((readRegister(BMP280_DIG_P6_MSB_REG) << 8) + readRegister(BMP280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((readRegister(BMP280_DIG_P7_MSB_REG) << 8) + readRegister(BMP280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((readRegister(BMP280_DIG_P8_MSB_REG) << 8) + readRegister(BMP280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((readRegister(BMP280_DIG_P9_MSB_REG) << 8) + readRegister(BMP280_DIG_P9_LSB_REG)));

	//Initialize sensor with default values
	setStandbyTime(settings.tStandby);
	setFilter(settings.filter);
	setPressureOverSample(settings.pressOverSample); //Default of 1x oversample
	setTempOverSample(settings.tempOverSample); //Default of 1x oversample
	setMode(MODE_NORMAL); //Go!
	
	return(true);
}

//Begin comm with BMP280 over SPI
bool BMP280::beginSPI(uint8_t csPin)
{
	settings.chipSelectPin = csPin;
	settings.commInterface = SPI_MODE;
	
	begin();
}

//Begin comm with BMP280 over I2C
bool BMP280::beginI2C(TwoWire &wirePort)
{
	_hardPort = &wirePort;
	_wireType = HARD_WIRE;

	settings.commInterface = I2C_MODE;
	
	//settings.I2CAddress = 0x77; //We assume user has set the I2C address using setI2CAddress()
	begin();
}

//Begin comm with BMP280 over software I2C
#ifdef SoftwareWire_h
bool BMP280::beginI2C(SoftwareWire& wirePort)
{
	_softPort = &wirePort;
	_wireType = SOFT_WIRE;

	settings.commInterface = I2C_MODE;
	//settings.I2CAddress = 0x77; //We assume user has set the I2C address using setI2CAddress()

	if(begin() == 0) return(true); //Begin normal init with these settings.
	return(false);
}
#endif

//Set the global setting for the I2C address we want to communicate with
//Default is 0x77
void BMP280::setI2CAddress(uint8_t address)
{
	settings.I2CAddress = address; //Set the I2C address for this device
}


/***********************************************************************************
								Communication functions
***********************************************************************************/


void BMP280::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	//define pointer that will point to the external space
	uint8_t i = 0;
	char c = 0;

	switch (settings.commInterface)
	{

	case I2C_MODE:
		switch(_wireType)
		{
			case(HARD_WIRE):
				_hardPort->beginTransmission(settings.I2CAddress);
				_hardPort->write(offset);
				_hardPort->endTransmission();

				// request bytes from slave device
				_hardPort->requestFrom(settings.I2CAddress, length);
				while ( (_hardPort->available()) && (i < length))  // slave may send less than requested
				{
					c = _hardPort->read(); // receive a byte as character
					*outputPointer = c;
					outputPointer++;
					i++;
				}
				break;
			case(SOFT_WIRE):
			#ifdef SoftwareWire_h
				_softPort->beginTransmission(settings.I2CAddress);
				_softPort->write(offset);
				_softPort->endTransmission();

				// request bytes from slave device
				_softPort->requestFrom(settings.I2CAddress, length);
				while ( (_softPort->available()) && (i < length))  // slave may send less than requested
				{
					c = _softPort->read(); // receive a byte as character
					*outputPointer = c;
					outputPointer++;
					i++;
				}
			#endif
				break;
		}
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(settings.chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		while ( i < length ) // slave may send less than requested
		{
			c = SPI.transfer(0x00); // receive a byte as character
			*outputPointer = c;
			outputPointer++;
			i++;
		}
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}

}

uint8_t BMP280::readRegister(uint8_t offset)
{
	//Return value
	uint8_t result = 0;
	uint8_t numBytes = 1;
	switch (settings.commInterface) {

	case I2C_MODE:
		switch(_wireType)
		{
			case(HARD_WIRE):
				_hardPort->beginTransmission(settings.I2CAddress);
				_hardPort->write(offset);
				_hardPort->endTransmission();

				_hardPort->requestFrom(settings.I2CAddress, numBytes);
				while ( _hardPort->available() ) // slave may send less than requested
				{
					result = _hardPort->read(); // receive a byte as a proper uint8_t
				}
				break;
			
			case(SOFT_WIRE):
			#ifdef SoftwareWire_h
				_softPort->beginTransmission(settings.I2CAddress);
				_softPort->write(offset);
				_softPort->endTransmission();

				_softPort->requestFrom(settings.I2CAddress, numBytes);
				while ( _softPort->available() ) // slave may send less than requested
				{
					result = _softPort->read(); // receive a byte as a proper uint8_t
				}
			#endif
				break;
		}
		
		break;

	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(settings.chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset | 0x80);  //Ored with "read request" bit
		// send a value of 0 to read the first byte returned:
		result = SPI.transfer(0x00);
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}
	return result;
}

void BMP280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	switch (settings.commInterface)
	{
	case I2C_MODE:
		//Write the byte

		switch(_wireType)
		{
			case(HARD_WIRE):
				_hardPort->beginTransmission(settings.I2CAddress);
				_hardPort->write(offset);
				_hardPort->write(dataToWrite);
				_hardPort->endTransmission();
				break;
			case(SOFT_WIRE):
			#ifdef SoftwareWire_h
				_softPort->beginTransmission(settings.I2CAddress);
				_softPort->write(offset);
				_softPort->write(dataToWrite);
				_softPort->endTransmission();
			#endif
				break;
		}
		break;
		
	case SPI_MODE:
		// take the chip select low to select the device:
		digitalWrite(settings.chipSelectPin, LOW);
		// send the device the register you want to read:
		SPI.transfer(offset & 0x7F);
		// send a value of 0 to read the first byte returned:
		SPI.transfer(dataToWrite);
		// decrement the number of bytes left to read:
		// take the chip select high to de-select:
		digitalWrite(settings.chipSelectPin, HIGH);
		break;

	default:
		break;
	}
}


/***********************************************************************************
								Configuration functions
***********************************************************************************/


// Get the current mode bits [0:1] in the ctrl_meas register
// Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
uint8_t BMP280::getMode()
{
	uint8_t controlData = readRegister(BMP280_CTRL_MEAS_REG);
	return(controlData & 0b00000011); //Keep only bits [0:1]
}

// Set the mode bits [0:1] in the ctrl_meas register
// Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
void BMP280::setMode(uint8_t mode)
{
	if(mode > 0b11) mode = 0; //Error check. Default to sleep mode
	
	uint8_t controlData = getMode();
	controlData |= mode;
	writeRegister(BMP280_CTRL_MEAS_REG, controlData);
}

//Strictly resets.  Run .begin() afterwards
void BMP280::reset( void )
{
	writeRegister(BMP280_RST_REG, 0xB6);
	
}

//Set the standby time (tStandby) in the config register
//tStandby can be:
//  0	0.5 ms
//  1	62.5 ms
//  2	125 ms
//  3	250 ms
//  4	500 ms
//  5	1000 ms
//  6	2000 ms
//  7	4000 ms
void BMP280::setStandbyTime(uint8_t timeSetting)
{
	if(timeSetting > 0b111) timeSetting = 0; //Error check. Default to 0.5ms
	
	uint8_t controlData = readRegister(BMP280_CONFIG_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear the 7/6/5 bits
	controlData |= (timeSetting << 5); //Align with bits 7/6/5
	writeRegister(BMP280_CONFIG_REG, controlData);
}

//Set the IIR filter setting in the config register
//filter can be off or number of FIR coefficients to use:
//  0	filter off
//  1	coefficients = 2
//  2	coefficients = 4
//  3	coefficients = 8
//  4	coefficients = 16
void BMP280::setFilter(uint8_t filterSetting)
{
	if(filterSetting > 0b111) filterSetting = 0; //Error check. Default to filter off
	
	uint8_t controlData = readRegister(BMP280_CONFIG_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear the 4/3/2 bits
	controlData |= (filterSetting << 2); //Align with bits 4/3/2
	writeRegister(BMP280_CONFIG_REG, controlData);
}

//Set the temperature oversample value
//0 turns off temp sensing
//1 to 16 are valid over sampling values
void BMP280::setTempOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkOversampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_t bits (7, 6, 5) to overSampleAmount
	uint8_t controlData = readRegister(BMP280_CTRL_MEAS_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear bits 765
	controlData |= overSampleAmount << 5; //Align overSampleAmount to bits 7/6/5
	writeRegister(BMP280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the pressure oversample value
//0 turns off pressure sensing
//1 to 16 are valid over sampling values
void BMP280::setPressureOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkOversampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_p bits (4, 3, 2) to overSampleAmount
	uint8_t controlData = readRegister(BMP280_CTRL_MEAS_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear bits 432
	controlData |= overSampleAmount << 2; //Align overSampleAmount to bits 4/3/2
	writeRegister(BMP280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Validates an over sample value
//Allowed values are 0 to 16
//These are used in the pressure, and temp oversample functions
uint8_t BMP280::checkOversampleValue(uint8_t userValue)
{
	switch(userValue) 
	{
		case(0): 
			return 0;
			break; //Valid
		case(1): 
			return 1;
			break; //Valid
		case(2): 
			return 2;
			break; //Valid
		case(4): 
			return 3;
			break; //Valid
		case(8): 
			return 4;
			break; //Valid
		case(16): 
			return 5;
			break; //Valid
		default: 
			return 1; //Default to 1x
			break; //Good
	}
}


/***********************************************************************************
								Measurement functions
***********************************************************************************/


//Check the measuring bit and return true while device is taking measurement
bool BMP280::isMeasuring(void)
{
	uint8_t stat = readRegister(BMP280_STAT_REG);
	return(stat & (1<<3)); //If the measuring bit (3) is set, return true
}

//Sets the internal variable _referencePressure so the 
void BMP280::setReferencePressure(float refPressure)
{
	_referencePressure = refPressure;
}

//Return the local reference pressure
float BMP280::getReferencePressure()
{
	return(_referencePressure);
}


// Reads ADC pressure value, than calculates and returns absolute pressure in Pa. Calculation algorithm is provided in datasheet.
float BMP280::readPressure(void)
{

	// Get raw ADC pressure value
	uint8_t buffer[3];
	int32_t adc_P;
	readRegisterRegion(buffer, BMP280_PRESSURE_MSB_REG, 3);
	adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	
	var1 = ((int64_t)_t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
	
	return (float)p_acc / 256.0;
	
}

float BMP280::readAltitudeMeters(void)
{
	float heightOutput = 0;
	
	heightOutput = ((float)-44330.77)*(pow(((float)readPressure()/(float)_referencePressure), 0.190263) - (float)1); //Corrected, see issue 30 at Sparkfun repo
	return heightOutput;
	
}

float BMP280::readAltitudeFeet(void)
{
	float heightOutput = 0;
	
	heightOutput = readAltitudeMeters() * 3.28084;
	return heightOutput;
	
}

// Reads ADC temperature value, than calculates and returns absolute pressure in Celsius. Calculation algorithm is taken from BMP280 datasheet.
float BMP280::readTempC( void )
{

	// Get raw ADC temperature value
	uint8_t buffer[3];
	int32_t adc_T;
	readRegisterRegion(buffer, BMP280_TEMPERATURE_MSB_REG, 3);
	adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

	int64_t var1, var2;
	float output;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) * ((int32_t)calibration.dig_T3)) >> 14;
	_t_fine = var1 + var2;
	output = (_t_fine * 5 + 128) >> 8;
	output = output / 100 + settings.tempCorrection;
	return output;
}

float BMP280::readTempF( void )
{
	float output = readTempC();
	output = (output * 9) / 5 + 32;
	return output;
}
