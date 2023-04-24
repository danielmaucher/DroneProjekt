#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h> 
#include <iostream>
#include <stdlib.h>
#include "Motor.h"


//Konstruktor
//PWM-Erweiterung wird über I²C angesprochen
Motor::Motor(int iMotor, int ifd)
{
	motor= iMotor;
	fd=ifd;

	//Setup und PWM-Frequenz wird gesetzt
	wiringPiI2CWriteReg8(fd, 0, 16);
	wiringPiI2CWriteReg8(fd, REG_FREQ, PWM_FREQ);
	wiringPiI2CWriteReg8(fd, 1, 4);		
	wiringPiI2CWriteReg8(fd, 0, 0);
	wiringPiI2CWriteReg8(fd, 0xFA, 0);
}

Motor::~Motor()
{
	
}

//Rechnet einen Wert von 0-100 in Eingabe für PWM-Signal um
//Eingabe wird an PWM_Erweiterung übergeben
void Motor::setDrehzahl(float power_proz)
{
	
	if(power_proz<=100&&power_proz>=0)
	{
		
		act_power= power_proz;
		float duty= 58+power_proz/2;

		PWM_Driver::setDuty(motor, duty);
	}
}

//Gibt aktulles Powerlevel zurück
float Motor::getPower()
{
	return act_power;
}
