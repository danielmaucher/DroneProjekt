#include <stdio.h>
#include <wiringPi.h>
#include <iostream>
#include <stdlib.h>
#include "Servo.h"

#define MIN_ANG 0
#define MAX_ANG 180

//Konstruktor
//Übergebener Pin wird aktiviert
//PWM-Frequenz wird gesetzt auf 331Hz
Servo::Servo(int ipin)
{
	pin=ipin;	
	pinMode(pin,PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(1000);
	pwmSetClock(58);
	
}

Servo::~Servo()
{
	
}


//Übergabewert in Grad
//PWM-Signal wird entsprechend gesetzt
//Servoposition wird in lokaler Variable gespeichert
void Servo::pos(int input)
{
	if(input<MIN_ANG)
	{
		input=MIN_ANG;
	}
	else if(input>MAX_ANG)
	{
		input=MAX_ANG;
	}
	aktPos = input;
	int del = (int) 168+ input*700/180.0;
	pwmWrite(pin,del);
}


//Aktuelle Position des Servomotors wird zurückgegeben
int Servo::getPos()
{
	return aktPos;
}


