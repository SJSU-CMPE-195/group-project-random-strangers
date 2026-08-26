/*
 * MT6816.h
 *
 *  Created on: 25th of August, 2026
 *      Author: Brendan
 */

#ifndef MT6816_H_
#define MT6816_H_

#include "Arduino.h"
#include "SPI.h"


union MT6816SPIReg {
	struct {
		uint16_t angle:14;
        uint8_t  no_mag_error:1;
        uint8_t  even_parity:1;
	};
	uint16_t reg;
};


#define MT6816_CPR 16384
#define MT6816_REG_ADDR1 0x03
#define MT6816_REG_ADDR2 0x04
#define MT6816_BITORDER MSBFIRST
#define MT6816_RW 0b10000000

static SPISettings MT6816SPISettings(8000000, MT6816_BITORDER, SPI_MODE3); // @suppress("Invalid arguments")


class MT6816 {
public:
	MT6816(SPISettings settings = MT6816SPISettings, int nCS = -1);
	virtual ~MT6816();

	virtual void init(SPIClass* _spi = &SPI);

	float getCurrentAngle(); // angle in radians, return current value
	float getFastAngle();	 // angle in radians, return last value and read new

	uint16_t readRawAngle(); // 14bit angle value

private:

	uint8_t spi_transfer8(uint8_t outdata);
	SPIClass* spi;
	SPISettings settings;
	bool errorflag = false;
	int nCS = -1;
	uint16_t lastAngle = 0;

};

#endif /* AS5047U_H_ */
