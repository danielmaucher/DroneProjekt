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
	
	//Körperfestes Koordinatensystem wird in einer Quaternion definiert
	quat={1,0,0,0};
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

//Rotationsmatrix wird aus Quaternion erstellt
void rotmat(quatern q)
{
	
	matR[0][0]=1-2*(q.c*q.c+q.d*q.d);
	matR[0][1]=2*(q.b*q.c-q.a*q.d);
	matR[0][2]=2*(q.b*q.d+q.a*q.c);
	matR[1][0]=2*(q.b*q.c+q.a*q.d);
	matR[1][1]=1-2*(q.b*q.b+q.d*q.d);
	matR[1][2]=2*(q.c*q.d-q.a*q.b);
	matR[2][0]=2*(q.b*q.d-q.a*q.c);
	matR[2][1]=2*(q.c*q.d+q.a*q.b);
	matR[2][2]=1-2*(q.c*q.c+q.b*q.b);
	
}

//Quaternion wird normiert
void normQuat(quatern& q)
{
	double length=sqrt(q.a*q.a+q.b*q.b+q.c*q.c+q.d*q.d);
	
	
}

//Rotiert Quaternion
quatern rotate( quatern& q, double dPhi, double dTheta, double dPsi)
{
	double sinp=sin(dPhi/2);
	quatern qphi(cos(dPhi/2), matR[0][0]*sinp, matR[1][0]*sinp, matR[2][0]*sinp);
	sinp=sin(dTheta/2);
	quatern qtheta(cos(dTheta/2),matR [0][1]*sinp, matR[1][1]*sinp, matR[2][1]*sinp);
	sinp=sin(dPsi/2);
	quatern qpsi(cos(dPsi/2), matR[0][2]*sinp, matR[1][2]*sinp, matR[2][2]*sinp);
	quatern q_kon={q.a,-q.b, -q.c, -q.d};
	return (q*qphi*qtheta*qpsi);
}

//Berechnet aus der Rotationsmatrix die Lagewinkel Phi, Theta und Psi
//Wendet Kalman-Filter oder Komplementärfilter an
void angels()
{
	double dif_phi=phi3;
	double dif_theta=theta3;
	
	
	phi3 = atan2(matR[2][1], matR[2][2]);
	theta3=asin(-matR[2][0]);
	psi3 = atan2(matR[1][0], matR[0][0]);
	
	phi += phi3 -dif_phi ;
	theta += theta3 - dif_theta;
	
	
	/*
	//Kalman-Filter
	kal_t=var_t/(var_t+VAR_MES_T);
	var_t=(1-kal_t)*var_t+PROZESSRAUSCHEN;
	kal_p=var_p/(var_p+VAR_MES_P);
	var_p=(1-kal_p)*var_p+PROZESSRAUSCHEN;
	
	phi= phi*(1-kal_p)+kal_p*phiacc/57.3;
	theta = theta*(1-kal_t)+kal_t*thetaacc/57.3;
	*/

	//Komplementärfilter
	phi = phi * COMP_FAC + (1 - COMP_FAC)*phiacc*GRAD_RAD;
	theta = theta * COMP_FAC + (1 - COMP_FAC)*thetaacc*GRAD_RAD;
	
}

 
//Berechnet aus den Beschleunigungen die Lagewinkel Phi und Theta
void Lagesensor::getRotation(const double &x,const double &y,const double &z) 
{ 
	phiacc = atan2(y, z);
	thetaacc = asin(x);
}  

//Hauptfunktion
//Zuerst werden Rollraten gemittelt, damit die Berechnung verbessert wird
//Endlosschleife in der Daten ausgelesen werden und Funktionen aufgerufen 
//werden, damit die Lage bestimmt wird
void Lagesensor::lageerkennung() 
{ 

	int anzahl=1000;
	
	double acc1x, acc1y, acc1z;	  
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
		acc1x = read_word_i2c(X_ACC)/ ACC_SKAL; 
		acc1y = read_word_i2c(Y_ACC)/ ACC_SKAL; 
		acc1z = read_word_i2c(Z_ACC)/ ACC_SKAL;  
		p= GRAD_RAD*(read_word_i2c(X_ROT) / DREH_SKAL -p_mittel);
		q= GRAD_RAD*(read_word_i2c(Y_ROT) / DREH_SKAL -q_mittel);
		r= GRAD_RAD*(read_word_i2c(Z_ROT) / DREH_SKAL -r_mittel);			
		phi2 = 0.02*p;
		theta2 = 0.02*q;
		psi2 = 0.02*r; 
	
		phiacc= getRotation( acc1y,acc1x, acc1z);
		thetaacc= getRotation(acc1x,acc1y, acc1z);
		
		quat=rotate(quat,phi2,  theta2 ,psi2);
		rotmat(quat);
		angels();
		normQuat(quat);
	
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
