#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h> 
#include <thread>
#include <fstream>
#include <time.h>
#include <string.h>
#include "Lagesensor.h"
#include <cmath>

#define PWM_I2C_ADDR 0x40 	
#define MOTOR_FAC 3

#define REG_FREQ 0xFE		// Register to change frequenz
#define PWM_FREQ 10 		// round(25MHZ/4096/freq)-1

using namespace::std;

Lagesensor lagesensor;
bool reglung=true;
double tn;
double kp;
double tv;
double phi;
int fd = wiringPiI2CSetup(PWM_I2C_ADDR);
int motorbase;

//Thread, der die Lageerkennung startet
void tRotate()
{
	
	lagesensor.lageerkennung();
	
	
}

//Erste Regelparameter werden erstellt
//Parameter sind nicht gut, aber gute Ausgangslage 
//für den Parameteralgorithmus
void parameter1()
{
	kp=0;
	tv=0;
	tn=0;
	bool b=true;
	int q=0;
	double phi_ref=phi;
	while (b)
	{
		
		if(q==0)
		{
			kp+=0.05;
			std::cout<< kp<< std::endl;
		}
		
		delay(200);
		
		if(abs(phi)<abs(phi_ref*0.95))
		{
			q++;
			std::cout<<"phidiff"<<phi-phi_ref<<std::endl;
		}
		else 
		{
			q=0;
		}
		
		if(q==5)
		{
			std::cout<<"q "<<q<<endl;
			b=false;
			
		}	
	}
	kp*=1;
	tv=0.5*kp;
	tn=0.001;
	std::cout<< "Regelparameter gesetzt "<<kp<<std::endl;
}

//Parameteralgorithmus
//Regelverhalten wird aufgezeichnet
//Regelparameter werden angepasst
void parameter2()
{
	int avgWerte=200;
	for(int u=0; u<=10;u++)
	{
		double p_avg=0;
		double phi_avg=0;
		double phi_sigma=0;
		double phi_avgW[avgWerte];
		for(int i=0; i<=avgWerte; i++)
		{
			phi_avgW[i]=phi;
			phi_avg+=phi_avgW[i];
			p_avg+=abs(lagesensor.getp());		
			delay(20);
		}
		phi_avg=phi_avg/avgWerte;
		p_avg=p_avg/avgWerte;
		
		cout<< "Phi_avg " << phi_avg << endl;
		cout<< "p_avg " << p_avg << endl;
		for(int i=0; i<=avgWerte; i++)
		{
			phi_sigma+=abs(phi_avgW[i]-phi_avg);
		}
		phi_sigma=phi_sigma/avgWerte;
		cout<< "Phi_sigma "<< phi_sigma<<endl;
		
		if(phi_avg>0.1||phi_avg<-0.1)
		{
			if(p_avg>0.1)
			{
				kp=kp/1.05;
				tv*=1.05;
			}
			else 
			if(phi_sigma>0.015)
			{
				kp*=1.07;
				tv=tv/1.05;
			}
			else
			{
				tn*=1.05;
			}
		}
		else
		{
			if(phi_sigma>0.01)
			{
				if(phi_sigma>0.01)
				{
					tv*=1.05;
					kp=kp/1.05;
				}
				else
				{
					tn=tn/1.05;
				}
									
			}
			else
			{
				cout<<"Regelparameter gut "<< endl;
			}
			
		}
		
		cout<< "Regelparameter: kp "<< kp<< " tn "<< tn << " tv "<< tv<< endl;
		
	}
	cout<<"Parameter gesetzt"<<endl;
	
}

//Thread, der die Motor bei Betätigung der meißten Tasten ausschält
//bei drücken der Tasten 'm' und 'n' wird die Drehzahl verändert
//'p' und 'o' passen die Regelparameter an
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
			case 'p':
				parameter1();
				break;
			case 'o':
			
				parameter2();
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
		
		
			phi_sum+=0.006*phi;

			//PID-Regler
			y_phi=kp*(phi+ tn*phi_sum+tv*(lagesensor.getp()));
			
			
			//Stellgröße wird der Motordrehzahl hinzugefügt
			//Die Drehzahl eines Motors wird erhöht und die des 
			//anderen wird verringert, so wird ein Drehmoment erzeugt
			wert=(y_phi*MOTOR_FAC);
			
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
