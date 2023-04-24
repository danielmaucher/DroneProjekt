#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h> 
#include <thread>
#include <fstream>
#include <time.h>
#include <string.h>
#include "Lagesensor.h"

#define PWM_I2C_ADDR 0x40 	
#define MOTOR_FAC 3
#define KP 1.8				//Regelparameter
#define TN 0.001			//
#define TV 0.5				//

#define REG_FREQ 0xFE		// Register to change frequenz
#define PWM_FREQ 10 		// round(25MHZ/4096/freq)-1

using namespace::std;

Lagesensor lagesensor;
bool reglung=true;

int fd = wiringPiI2CSetup(PWM_I2C_ADDR);
int motorbase;

//Thread, der die Lageerkennung startet
void tRotate()
{
	
	lagesensor.lageerkennung();
	
	
}

//Thread, der die Motor bei Betätigung der meißten Tasten ausschält
//bei drücken der Tasten 'm' und 'n' wird die Drehzahl verändert
void tSafe()
{
	char a;
	while(reglung)
	{
		std::cin >>a;
		switch(a)
		{
			case 'm':
			motorbase++;
			break;
			case 'n':
			motorbase--;
			break;
			default:
			reglung=false;
			wiringPiI2CWriteReg8(fd, 0x08, 7);
			wiringPiI2CWriteReg8(fd, 0x09, 8);
			wiringPiI2CWriteReg8(fd, 0x0C, 7);
			wiringPiI2CWriteReg8(fd, 0x0D, 8);
			break;
		}
	}
}


//Main-Funktion und Einstigspunkt des Programms
int main()
{
	
	//I²C und WiringPi Setup Funktionen
	//PWM_Frequenz der PWM_Erweiterung wird gesetzt
	if(wiringPiSetup()==-1)
	{
		std::cout<<"WiringPi Setup fehlgeschlagen";
		return 1;
	}
	wiringPiI2CWriteReg8(fd, 0, 16);
	wiringPiI2CWriteReg8(fd, REG_FREQ, PWM_FREQ);
	wiringPiI2CWriteReg8(fd, 1, 4);		
	wiringPiI2CWriteReg8(fd, 0, 0);
	wiringPiI2CWriteReg8(fd, 0xFA, 0);
	
	//Starten der Lageerkennung
	std::thread rot(tRotate);
	motorbase=0;
	
	//Txt-Datei wird erstellt
	struct tm *t;
	time_t tnow;
	time(&tnow);
	t=localtime(&tnow);
	char datum[12];
	FILE *datei;
	sprintf(datum, "%i-%i-%i-%i-%i",t->tm_mday, t->tm_mon+1, t->tm_year+1900, t->tm_hour, t->tm_min);
	ofstream ofFile(strcat(datum, "Regler.txt"),ios::trunc);
	
	double phi;
	double phi_sum;
	double phi_alt;
	double y_alt;
	double phi_set;
	double y_phi;
	int msb, lsb;
	int ausgabe;
	double wert;
	

	//Motordrehzahl wird auf Null gesetzt
	wiringPiI2CWriteReg8(fd, 0x08, 7);
	wiringPiI2CWriteReg8(fd, 0x09, 8);
	wiringPiI2CWriteReg8(fd, 0x0C, 7);
	wiringPiI2CWriteReg8(fd, 0x0D, 8);
	
	
	
	char a;
	std::cin>>a;
	motorbase+=9;
	delay(100);
	std::thread saf(tSafe);

	//Regelschleife
	while(reglung)
	{
		phi= lagesensor.get_phi();
		
		
			phi_sum+=phi;

			//PID-Regler
			y_phi=KP*(phi+ TN*phi_sum+TV*(lagesensor.getp()));
					
			wert=(y_phi*MOTOR_FAC);
			
			//Stellgröße wird der Motordrehzahl hinzugefügt
			//Die Drehzahl eines Motors wird erhöht und die des 
			//anderen wird verringert, so wird ein Drehmoment erzeugt
			ausgabe= 4096*(wert+50+motorbase)/100;
			msb= ausgabe/256;
			lsb= ausgabe-256*msb;
	
			wiringPiI2CWriteReg8(fd, 0x08, lsb);
			wiringPiI2CWriteReg8(fd, 0x09, msb);
			
			
			ausgabe= 4096*(-wert+50+motorbase)/100;
			msb= ausgabe/256;
			lsb= ausgabe-256*msb;
	
			wiringPiI2CWriteReg8(fd, 0x0C, lsb);
			wiringPiI2CWriteReg8(fd, 0x0D, msb);
			

			//Lagewinkel, Stellgröße und Rollrate wird in die Txt-Datei geschrieben
			ofFile<<phi;
			ofFile<<"\t";
			ofFile<<y_phi;
			ofFile<<"\t";
			ofFile<<phi_sum;
			ofFile<<"\t";
			ofFile<<lagesensor.getp();
			ofFile<<endl;
			
		
	
		delay(0.5);
	}
	
	//Motordrehzahl wird auf Null gesetzt
	wiringPiI2CWriteReg8(fd, 0x08, 7);
	wiringPiI2CWriteReg8(fd, 0x09, 8);
	wiringPiI2CWriteReg8(fd, 0x0C, 7);
	wiringPiI2CWriteReg8(fd, 0x0D, 8);
	
	
	return 0;
	

}
