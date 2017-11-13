#include "CSonar.h"

#define SONAR_START 0
#define SONAR_END 1
uint32_t EchoPin = 88;
uint32_t TrigPin = 89;
uint32_t timer_us_start = 0;
volatile int sonar_state = SONAR_END;
volatile uint32_t timer_us = 0;

extern "C"{
void sonar_handle(void)
{
	if(sonar_state == SONAR_START )
	{
		timer_us = micros();
		sonar_state = SONAR_END;
	}
}
}


CSonar::CSonar()
{

}

CSonar::~CSonar()
{
	
}

void CSonar::Init(void)
{
	pinMode(EchoPin,INPUT);
	pinMode(TrigPin,OUTPUT);
	//attachInterrupt(8, sonar_handle, FALLING);
}

float CSonar::update(void)
{
	
	float sonar_out;

	//if(sonar_state == SONAR_END || (micros() - timer_us_start) > 60000)
	{
		//if((micros() - timer_us_start) > 60000) this->output = 0.0;
		//else this->output = (timer_us - timer_us_start)/1000.0;
		digitalWrite(TrigPin, HIGH);
		delay_us(50);
		digitalWrite(TrigPin, LOW);
		timer_us = pulseIn(EchoPin, HIGH, 3000);	
		this->output = timer_us*17/100000.0;
		timer_us_start = micros();
		sonar_state = SONAR_START;
	}


	
	return sonar_out;
	
}


