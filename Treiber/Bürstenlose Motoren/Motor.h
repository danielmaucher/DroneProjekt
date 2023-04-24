#pragma once
#include "PWM_Driver.h"

class Motor : public PWM_Driver
{
	public:
		Motor(int iMotor, int ifd);
		~Motor();
		void setDrehzahl(float power_proz);
		float getPower();
		
	private:
		float act_power;
		int motor;
};
