#include "AD8495.h"
AD8495::AD8495(int pin){
	apin=pin;
	ptr=0;
	
	rejects=0;

}
void AD8495::init(){
	for(int i=0;i<bufSize;i++){
		aBuf[i]=analogRead(apin);
	}
	lastAvg=aBuf[bufSize-1];
}
void AD8495::poll(){
	if(ptr>=bufSize)ptr=0;
	analogRead(apin);
	int aVal=analogRead(apin);
	int diff=aVal-lastAvg;
	if(diff<0)diff=-diff;
	if(diff<maxFluc){
		aBuf[ptr]=aVal;
		Serial.println(aBuf[ptr]);
		ptr++;
		rejects=0;
	}
	else{
		rejects++;
		if(rejects>maxRej){
			rejects=0;
			aBuf[ptr]=aVal;
			lastAvg=aVal;
			for(int i=0;i<bufSize;i++){
				aBuf[i]=aVal;
			}
			Serial.print("acc:");
			Serial.println(aBuf[ptr]);
			ptr++;
		}else{
			Serial.print("rej:");
			Serial.print(aVal);
			Serial.print(",   ");
			Serial.println(lastAvg);
		}
	}
}
double AD8495::tempC(){
	int aVal=0;
	for(int i=0;i<bufSize;i++)aVal+=aBuf[i];
	aVal/=bufSize;
	lastAvg=aVal;
	double mVolt=(double)map(aVal,0,1023,0,5000);
	double temp=(mVolt-1250.0)/5.0;
	return temp;
}
double AD8495::tempF(){
	return tempC()*1.8+32.0;
}