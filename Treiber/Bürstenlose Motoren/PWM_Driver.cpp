#include "PWM_Driver.h"
#include <wiringPiI2C.h> 
#include <wiringPi.h> 




PWM_Driver::PWM_Driver()
{
	 
	
}

PWM_Driver::~PWM_Driver()
{
	
}

//Setzt PWM-Signal, abhänig von übergebenen Wert 
void PWM_Driver::setDuty(int output, float wert)
{
	
	int outReg= Reg_OUT+4*output;
	int ausgabe= 4096*wert/100;
	int msb= ausgabe/256;
	int lsb= ausgabe-256*msb;
	
	wiringPiI2CWriteReg8(fd, outReg, lsb);
	wiringPiI2CWriteReg8(fd, outReg+1, msb);
	
	
}
