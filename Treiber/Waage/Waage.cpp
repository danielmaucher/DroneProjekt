#include <stdio.h>
#include <wiringPi.h>
#include <iostream>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include "Waage.h"

#define WAAGE_DT 26
#define WAAGE_CK 22

#define HIGH 1
#define LOW 0

#define AVG_COUNT 32
#define TOLERANZ 0.001
#define RESET_TIME 0.06


using namespace::std;

//Konstruktor
Waage::Waage()
{
	cal=0;
}

Waage::~Waage()
{
	
}

//Waage wird initialisiert
//Waagenwert ohne Gewicht/Schub wird ermittelt
void Waage::init_waage()
{
	
	int z=0;
	cal=0;
	
	unsigned long avg[AVG_COUNT] ;
	unsigned long average=0;
	unsigned long average2=0;
	unsigned long low, high;
	

	reset();
	getWeight();
	getWeight();
	
	
	for(int j=0; j<AVG_COUNT; j++)
	{		
		avg[j]=this->getWeight();
		average+= avg[j];
		
	}
	
	low = (long) average/AVG_COUNT*(1-TOLERANZ);
	high = (long) average/AVG_COUNT*(1+TOLERANZ);
	
	for(int i=0; i<AVG_COUNT; i++)
	{
		if((avg[i] <= high && avg[i] >=  low))
		{
			average2 += avg[i];
			z++;
		}		
	}
	if(z<AVG_COUNT*0.75)
	{
		cout<<"Waage sehr ungenau.\tErneute Kalibrierung empfohlen.\n";
		
	}
	try
	{
	cal= average2/z;
	}
	catch(const exception& e)
	{
		cout<<"Keine Kalibrierung durchgeführt. Muss erneut durchgeführt werden.\n";
	}
	
	
	cout<<"Nullwert Waage: "<< cal<< endl;
	
	
}

//Wartet bis A/D-Wandler bereit ist
void Waage::check()
{
	while(digitalRead(WAAGE_DT)==1)
	{
	}
}


//Gibt rohen Waagenwert aus
//Dazu werden 26 Impulse auf der CLK gesendet
//und die Datenleitung ausgelesen
unsigned long Waage::getWeight()
{
	int b;
	unsigned long x=0;
	bool neg=false;
	
	reset();
	
	check();
	for(int i=0; i<26; i++)
	{
	
		digitalWrite(WAAGE_CK,HIGH);
		delay(0.005);
		digitalWrite(WAAGE_CK, LOW);
		x= x << 1;
		
		x+=digitalRead(WAAGE_DT);
				
	}

	reset();
	x=x/1024;
	
	return  x;
}

//Gibt den Skalierungsfaktor zurück
unsigned long Waage::getCal()
{
	return cal;
}


//HX711 (A/D-Wandler) wird zurückgesetzt
void Waage::reset()
{
	digitalWrite(WAAGE_CK, HIGH);
	delay(RESET_TIME);
	digitalWrite(WAAGE_CK, LOW);

}