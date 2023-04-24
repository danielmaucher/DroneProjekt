#pragma once

#define REG_FREQ 0xFE		// Register to change frequenz
#define PWM_FREQ 10 		// round(25MHZ/4096/freq)-1
#define Reg_OUT 0x08		// Register of first channel


class PWM_Driver
{
	public: 
		PWM_Driver();
		~PWM_Driver();
		void setDuty(int output, float wert);
	
	protected:
		int fd;
	
};
