#pragma once

#define X_ACC 0x3B
#define Y_ACC 0x3D
#define Z_ACC 0x3F
#define X_ROT 0x43
#define Y_ROT 0x45
#define Z_ROT 0x47
#define I2C_ADDR 0x68

#define COMP_FAC 0.9
#define ACC_SKAL 16384.0
#define DREH_SKAL 65.5
#define RAD_GRAD 57.3
#define GRAD_RAD 0.01745

#define VAR_MES_P 0.10
#define VAR_MES_T 0.10
#define PROZESSRAUSCHEN 0.01

	
class Lagesensor
{
	
		
	public: 
		Lagesensor();
		~Lagesensor();
		void lageerkennung();
		double get_phi();
		double get_theta();
		double get_psi();
		double getp();
		double getq();
		double getr();
		

		//Vektor wird definiert
		//Vektoroperationen werden definiert
		struct vektor 
		{
			double x;
			double y;
			double z;
			
			vektor(double x=0, double y=0, double z=0)
				:x(x), y(y), z(z)
			{					
			}
			
			//Addition zweier Vektoren
			vektor operator+(const vektor& a)
			{
				return vektor(a.x+x, a.y+y, a.z+z);
			}
			
			//Produkt mit Skalar
			vektor operator*(const double& a)
			{
				return vektor(a*x, a*y, a*z);
			}
			
			//Skalarprodukt
			double operator*(const vektor& a)
			{
				return (x*a.x+y*a.y+z*a.z);
			}
			
			//Kreuzprodukt
			vektor operator%(const vektor& a)
			{
				return vektor(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x);
			}
			
		};

		
	private:
		
		int read_word_i2c(const int &addr);
		void vec_skal(vektor &a);
		void getRotation(const double &x,const double &y,const double &z);
		void angels();
		void set_newCoord();
		int fd;
		double x_rot, y_rot, z_rot;
		double phi, theta, psi;
		double phi2, theta2, psi2;
		double phi3, theta3;
		double p, q, r;
		double phiacc, thetaacc;
		double var_t, kal_t, kal_p, var_p;
		int val; 
		vektor x_vec,y_vec,z_vec;
		vektor pos, acc, vel;
		vektor acc1;
		

};
