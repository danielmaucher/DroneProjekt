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
		
		//Quaternionen und dazugehörige Operationen werden definiert
		struct quatern
		{
			double a;
			double b;
			double c;
			double d;
			
			quatern(double a=0, double b=0, double c=0, double d=0)
				:a(a), b(b), c(c), d(d)
				{
				}
				
			quatern operator*(quatern q)
			{
				return quatern(	a*q.a-b*q.b-c*q.c-d*q.d,
								a*q.b+b*q.a+c*q.d-d*q.c,
								a*q.c-b*q.d+c*q.a+d*q.b,
								a*q.d+b*q.c-c*q.b+d*q.a);
			}	
		};

		
	private:
		
		
		
		int fd;
		double x_rot, y_rot, z_rot;
		double matR[3][3];
		quatern quat;
		double phi, theta, psi;
		double phi2, theta2, psi2;
		double phi3, theta3;
		double p, q, r;
		double phiacc, thetaacc;
		double var_t, kal_t, kal_p, var_p;
		int val; 

		void rotmat(quatern q);
		void normQuat(quatern& q);
		quatern rotate( quatern q, double dPhi, double dTheta, double dPsi);
		int read_word_i2c(const int &addr);	
		double dist(const double& a,const double& b);	
		double getRotation(const double &x,const double &y,const double &z);
		void angels();
};
