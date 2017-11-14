#ifndef _CMOTOR_H
#define _CMOTOR_H

#include "drv_include.h"

#define RIGHT_CH 0
#define LEFT_CH 2
#define BACK_CH 1

#define FORWARD 0
#define BACKWARD 1

typedef struct
{
  
double output;

double target;
double feedback;
double last_feedback;

double e_0;     //this error
double e_1;     //last error
double e_2;     //pre  error

double Proportion;      //this error
double Integral;      //last error
double Differential;      //pre  error


double Kp;
double Ki;
double Kd;

int motorA;
int motorB;

}MOTOR_PID;

typedef struct
{
  long counter;
  int count;
  char dir;
  char encoder_a;
  char encoder_b;
  char isPass;
}Wheel;

typedef struct
{
  float v_motor1;
  float v_motor2;
  float v_motor3;
}Speed;

typedef struct{
  int SV_pwm;
  int PG_exti;
  int EN;
  int FR;
  int BK;
  int target;
}MOTOR_CTL;


extern "C" void Count_and_Direction(Wheel *wheel);
extern "C" float velocity_calculate(Wheel *omni) ;

class CMotor
{

public:
	CMotor();
	~CMotor();
	void Init();
	void controller(double s1,double s2);
	bool readEncoder(int &s1,int &s2);
	void setSpeed(double s1,double s2);
	void FeedbackSpeed(double s1,double s2);

  MOTOR_CTL m_ctl[2];
  Wheel m_wheel[2];
  double speed[2];
  double target_speed[2];
private:
	void setDir(int target,Wheel &wheel);
};


#endif

