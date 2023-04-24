#pragma once

class Waage
{
	public:
	Waage();
	~Waage();
	
	void init_waage();
	unsigned long getCal();
	unsigned long getWeight();
	
	private:
	unsigned long cal;
	void check();
	void reset();
	
};
