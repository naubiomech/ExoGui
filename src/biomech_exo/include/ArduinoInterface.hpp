#ifndef ARDUINO_INTERFACE_HEADER
#define ARDUINO_INTERFACE_HEADER

#include <stdio.h>
#include <stdlib.h>
#include "Pin.hpp"


class SoftwareSerial{
public:
	const char* readStr;
	int strLen;
	int charIndex;
public:
	SoftwareSerial(int, int);
	bool begin(int);
	void write(char character);
	void write(const char* str);
	void print(double);
	void print(const char[]);
	void println();
	void println(const char[]);
	void println(double);
	void setReadString(const char* str);

	int read();
	bool available();
};

#define REVERSE 1
#define AUTOMATIC 1

#define I2C_MASTER 1
#define I2C_PULLUP_EXT 1
#define I2C_RATE_100 1
#define I2C_OP_MODE_ISR 1

enum i2c_pins {I2C_PINS_3_4 = 0,
               I2C_PINS_7_8,
               I2C_PINS_37_38};
enum i2c_bus {WIRE_BUS, WIRE1_BUS};

struct sensors_event_t{
	struct {
		double x;
		double y;
		double z;
	} orientation;
};

class Adafruit_BNO055{
public:
	Adafruit_BNO055(int, int, unsigned int, int, i2c_pins, int, int, int);
	bool begin();
	void getEvent(sensors_event_t*);
	void getCalibration(uint8_t*, uint8_t*, uint8_t*, uint8_t*);
	bool isFullyCalibrated();

};

class Metro{
public:
	Metro(unsigned long interval);
	void reset();
	bool check();
};

class PID{
public:
	PID(double*, double*, double*, double, double, double, int);
	void SetMode(int);
	void SetOutputLimits(int,int);
	void SetSampleTime(int);
	void Compute_KF(double);
	double GetKp();
	double GetKi();
	double GetKd();
	void SetTunings(double,double,double);

};

#define INPUT 0
#define OUTPUT 1

#define LOW 0
#define HIGH 1

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21
#define A8 22
#define A9 23
#define A12 31
#define A13 32
#define A14 33
#define A15 34
#define A16 35
#define A17 36
#define A18 37
#define A19 38
#define A20 39
#define A21 40
#define A22 41

void delay(double);
double pow(double, double);
double exp(double);
double round(double);
double max(double, double);
double min(double, double);
unsigned long millis();
void pinMode(int, int);
void analogReadResolution(int);
void analogWriteResolution(int);
int analogRead(int);
void analogWrite(int, int);
int digitalRead(int);
void digitalWrite(int, int);

extern SoftwareSerial Serial;

#endif
