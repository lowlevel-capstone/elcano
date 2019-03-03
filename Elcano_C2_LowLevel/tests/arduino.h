#pragma once
#ifndef arduino
#define arduino
#include <stdexcept>
#include <cstdint>


enum pMode {INPUT, OUTPUT, INPUT_PULLUP};
enum pVal {LOW, HIGH};
enum intMode { CHANGE, RISING, FALLING };
enum spiMode { SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3 };
enum spiOrder { LSBFIRST, MSBFIRST };

uint32_t returnMillis = 0;

struct PinState{
		pMode mode;
		pVal val;
		int16_t angle;
}dPins[55];

void pinMode(int pinNum, pMode p) {
	 dPins[pinNum].mode = p;
}

uint32_t millis() {
	return returnMillis;
}

void digitalWrite(int pinNum, pVal p) {
	dPins[pinNum].val = p;
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class serial {
public:
	void println(std::string msg) {
		std::cout << msg << std::endl;
	};
	void print(std::string msg) {
		std::cout << msg;
	};
	void print(int msg) {
		std::cout << msg;
	};
	void println(int msg) {
		std::cout << msg << std::endl;
	};

}Serial;

class Servo {
public:
	int pNum;
	uint16_t servoVal;
	void attach(int pinNum) {
		pNum = pinNum;
	};
	//Sets angle in degrees 
	//can be from 0 to 180
	void write(uint16_t angle) {
		if (angle < 0 || angle > 180) {
			throw std::out_of_range ("Not between 0 and 180");
		}
		dPins[pNum].angle = angle;
	};

	//Sets angle based on microseconds
	//should be between 700ish and 2300ish
	void writeMicroseconds(int16_t uS) {
		if (uS < 700 || uS > 2300) {
			throw std::out_of_range("Not between 700 and 2300");
		}

		dPins[pNum].angle = map(uS,0,180,700,2300);
	};
};

class spi {
public:
	void setDataMode(spiMode Dmode) { Dmode = mode; };
	void setBitOrder(spiOrder bitOrder) { order = bitOrder; };
	void begin() { val = 0; even = true; };
	int16_t transfer(int16_t data) { 
		int16_t val;
		if ((even && order == MSBFIRST) || (!even && order == LSBFIRST) )
			val = transferB1(data);
		else
			val = transferB2(data);
		return val;
	};
	
	bool even;
	spiMode mode;
	spiOrder order;
	bool AorB;
	bool shutdown;
	bool gain;
	int16_t val;
private:
	int16_t transferB1(int16_t data) {
		/*char b1;
		char b2;
		data = data & 0xbff0;
		if (order == LSBFIRST) {
			b1 = (data & 0x00ff) >> 4;
			b2 = (data & 0xff00) >> 8;
		}
		else {
			b2 = (data & 0x00ff) >> 4;
			b1 = (data & 0xff00) >> 8;
		}
		AorB = b1 & 0x80;
		gain = b1 & 0x20;
		shutdown = b1 & 0x10;
		val = b1 & 0x0F;
		val = val << 4;
		val = val | b2;

		*/
		return 0;
	};
	int16_t transferB2(int16_t data) {
		char b2;
		data = data & 0xf0;
		b2 = data >> 4;
		val = 0 | b2;
	};
}SPI;

typedef void(*functiontype)();
class interrupt {
public:
	int16_t intNum;
	functiontype ISR;
	char mode;
	bool enabled;
}Interrupt;

void attachInterrupt(int intNum, functiontype f, intMode iMode) {
	Interrupt.intNum = intNum;
	Interrupt.ISR = f;
	switch (iMode) {
	case CHANGE: Interrupt.mode = 'c';
		break;
	case FALLING: Interrupt.mode = 'f';
		break;
	case RISING: Interrupt.mode = 'r';
		break;
	default:
		Interrupt.mode = '0';
	}
}
void attachInterrupt(int intNum, functiontype f, pVal p) {
	Interrupt.intNum = intNum;
	Interrupt.ISR = f;
	switch (p) {
	case LOW: Interrupt.mode = 'l';
		break;
	case HIGH: Interrupt.mode = 'h';
		break;
	default:
		Interrupt.mode = '0';
	}
}

void noInterrupts() {
	Interrupt.enabled = false;
}

void interrupts() {
	Interrupt.enabled = true;
}

int digitalPinToInterrupt(int pNum) {
	int intNum = -1;
#if aBoard == Mega
	switch (pNum)
	{
	case 2: intNum = 0;
		break;
	case 3: intNum = 1;
		break;
	case 21: intNum = 2;
		break;
	case 20: intNum = 3;
		break;
	case 19: intNum = 4;
		break;
	case 18: intNum = 5;
		break;
	default:
		break;
	}
#endif
	return intNum;
}

typedef char byte;

#endif