#ifndef ARDUINO_INTERFACE_HEADER
#define ARDUINO_INTERFACE_HEADER

#include "Pin.hpp"

class SoftwareSerial{
public:
	int rx;
	int tx;
public:
	SoftwareSerial();
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
#endif
