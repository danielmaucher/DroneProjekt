#pragma once

class Servo
{
	public:
		Servo(int ipin);
		~Servo();
		void pos(int input);
		int getPos();
	
	private:
		
		int input;
		int pin;
		int aktPos;

};
