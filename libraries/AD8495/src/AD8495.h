#pragma once

#include <Arduino.h>

class AD8495{
public:
	AD8495(int pin);
	double tempC();
	double tempF();
	void poll();
	void init();
private:
	static const int bufSize=30;
	static const int maxFluc=4;
	static const int maxRej=30;
	int apin;
	unsigned int aBuf[bufSize];
	unsigned int ptr;
	unsigned int lastAvg;
	unsigned int rejects;
	
};