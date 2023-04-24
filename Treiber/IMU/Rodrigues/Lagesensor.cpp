#include "Lagesensor.h"
#include <wiringPiI2C.h> 
#include <wiringPi.h> 
#include <stdio.h> 
#include <math.h>  
#include <iostream> 
#include <sys/time.h>

using namespace::std;

//Konstruktor
//I²C-Bus wird aktiviert
//Setup funktionen der IMU werden aufgerufen
Lagesensor::Lagesensor()
{
	fd = wiringPiI2CSetup (I2C_ADDR); 
	wiringPiI2CWriteReg8 (fd,0x6B,0x00);  	//disable sleep mode 
	wiringPiI2CWriteReg8 (fd,0x1A,0x00);	//Set ACC scaling (16384.0)
	wiringPiI2CWriteReg8 (fd,0x1B,0x08);	//Set Gyro scaling (65.5)
		
	//Körperfestes Koordinatensystem wird definiert
	x_vec= {1,0,0};
	y_vec= {0,1,0};
	z_vec= {0,0,1};
}

Lagesensor::~Lagesensor()
{
	
}	


//Beschleunigung oder Drehraten werden ausgelesen und zurückgegeben
int Lagesensor::read_word_i2c(const int &addr) 
{ 

val = wiringPiI2CReadReg8(fd, addr); 
val = val << 8; 
val += wiringPiI2CReadReg8(fd, addr+1); 
if (val >= 0x8000) 
val = -(65536 - val);   
return val; 
}

//Normiert übergebenen Vektor
void Lagesensor::vec_skal(vektor &a)
{
	double length= 1/sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
	a.x=a.x*length;
	a.y= a.y*length;
	a.z= a.z*length;
}


 

//Berechnet aus den Beschleunigungen die Lagewinkel Phi und Theta
void Lagesensor::getRotation(const double &x,const double &y,const double &z) 
{ 
	phiacc = atan2(y, z);
	thetaacc = asin(x);
	
}  
	 
  
//Berechnet aus dem Körperfesten Koordinatensystem die Lagewinkel Phi, Theta und Psi
//Wendet Kalman-Filter oder Komplementärfilter an
void Lagesensor::angels()
{
	double dif_phi=phi3;
	double dif_theta=theta3;
	
	phi3 =atan2(y_vec.z, z_vec.z);
	theta3 =asin(-x_vec.z);
	psi =atan2(x_vec.y, x_vec.x);
	
	phi += phi3 -dif_phi ;
	theta += theta3 - dif_theta;

	//Kalman Filter
	/*
	kal_t=var_t/(var_t+VAR_MES_T);
	var_t=(1-kal_t)*var_t+PROZESSRAUSCHEN;
	kal_p=var_p/(var_p+VAR_MES_P);
	var_p=(1-kal_p)*var_p+PROZESSRAUSCHEN;
	
	phi= phi*(1-kal_p)+kal_p*phiacc/57.3;
	theta = theta*(1-kal_t)+kal_t*thetaacc/57.3;
	
	*/

	//Komplementärfilter
	phi = phi*COMP_FAC + (1-COMP_FAC)*phiacc*GRAD_RAD;
	theta = theta*COMP_FAC + (1-COMP_FAC)*thetaacc*GRAD_RAD;
					
	
}


//Körperfestes Koordinatensystem wird gedreht
//phi2, theta2, psi2 sind Winkel um die gedreht wird
void Lagesensor::set_newCoord()
{
	//Rodrigues-Rotationsformel
	double cosp=cos(phi2);
	double sinp=sin(phi2);
	
	y_vec=(y_vec*cosp)+(x_vec%y_vec*sinp);	
	vec_skal(y_vec);
	z_vec=(z_vec*cosp)+(x_vec%z_vec*sinp);
	vec_skal(z_vec);
	
	cosp=cos(theta2);
	sinp=sin(theta2);
	x_vec=(x_vec*cosp)+(y_vec%x_vec*sinp);
	vec_skal(x_vec);
	z_vec=(z_vec*cosp)+(y_vec%z_vec*sinp);
	vec_skal(z_vec);
	
	cosp=cos(psi2);
	sinp=sin(psi2);
	y_vec=(y_vec*cosp)+(z_vec%y_vec*sinp);
	vec_skal(y_vec);
	x_vec=(x_vec*cosp)+(z_vec%x_vec*sinp);
	vec_skal(x_vec);
	
}



 
//Hauptfunktion
//Zuerst werden Rollraten gemittelt, damit die Berechnung verbessert wird
//Endlosschleife in der Daten ausgelesen werden und Funktionen aufgerufen 
//werden, damit die Lage bestimmt wird
void Lagesensor::lageerkennung() 
{ 

	int anzahl=1000;
		  
	double p_mittel=0;
	double q_mittel=0;
	double r_mittel=0;
	   
	for(int i=0; i<anzahl; i++)
	{
		p_mittel += read_word_i2c(X_ROT) / DREH_SKAL;
		q_mittel +=read_word_i2c(Y_ROT) / DREH_SKAL;
		r_mittel +=read_word_i2c(Z_ROT) / DREH_SKAL;
		
	} 
	
	p_mittel = p_mittel/anzahl;
	q_mittel = q_mittel/anzahl; 
	r_mittel = r_mittel/anzahl;

	std::cout<<"Lageerkennung bereit"<<std::endl;
	
	while(1) 
	{   
		acc1.x = read_word_i2c(X_ACC)/ ACC_SKAL; 
		acc1.y = read_word_i2c(Y_ACC)/ ACC_SKAL; 
		acc1.z = read_word_i2c(Z_ACC)/ ACC_SKAL;  
		p= GRAD_RAD*(read_word_i2c(X_ROT) / DREH_SKAL -p_mittel);
		q= GRAD_RAD*(read_word_i2c(Y_ROT) / DREH_SKAL -q_mittel);
		r= GRAD_RAD*(read_word_i2c(Z_ROT) / DREH_SKAL -r_mittel);			
		phi2 = 0.02*p;
		theta2 = 0.02*q;
		psi2 = 0.02*r; 
	
		getRotation( acc1.y,acc1.x, acc1.z);
		
	
		set_newCoord();
		angels();
		
		delay(1);
	} 
	
}
	 
//Gibt den Lagewinkel phi zurück	 
double Lagesensor::get_phi()
{
	return phi;
}	 

//Gibt den Lagewinkel theta zurück
double Lagesensor::get_theta()
{
	return theta;
}

//Gibt den Lagewinkel psi zurück
double Lagesensor::get_psi()
{
	return psi;
}

//Gibt die Rollrate zurück
double Lagesensor::getp()
{
	return p;
}

//Gibt die Nickrate zurück
double Lagesensor::getq()
{
	return q;
}

//Gibt die Gierrate zurück
double Lagesensor::getr()
{
	return r;
}
