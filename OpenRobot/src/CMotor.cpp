#include "CMotor.h"

#define SPEED_COMPARE(x,y) ((x*y)<0 ? 1:0)
#define KP  2.2
#define KI  0.36
#define KD  0.8
#define ROTATE_SPEED  6.0606


// right
#define MOTOR1_SV MOTOR_CH7
#define MOTOR1_EN 18 //PB2
#define MOTOR1_PG 26 //PB10
#define MOTOR1_FR 17 //PB1

#define MOTOR_BK 16 //PB0

// left
#define MOTOR2_SV MOTOR_CH8
#define MOTOR2_EN 79 //PB2
#define MOTOR2_PG 27 //PB10
#define MOTOR2_FR 76 //PB1

// right
#define MOTOR1_A_CH MOTOR_CH3
#define MOTOR1_B_CH MOTOR_CH4
#define ENCODER1_A  EXTI_PIN_2
#define ENCODER1_B  EXTI_PIN_3

// back
#define MOTOR2_A_CH MOTOR_CH5
#define MOTOR2_B_CH MOTOR_CH6
#define ENCODER2_A  EXTI_PIN_4
#define ENCODER2_B  EXTI_PIN_5

// left
#define MOTOR3_A_CH MOTOR_CH7
#define MOTOR3_B_CH MOTOR_CH8
#define ENCODER3_A  EXTI_PIN_6
#define ENCODER3_B  EXTI_PIN_7




#define EXTI_PIN_0 80
#define EXTI_PIN_1 81
#define EXTI_PIN_2 82
#define EXTI_PIN_3 83
#define EXTI_PIN_4 84
#define EXTI_PIN_5 85
#define EXTI_PIN_6 86
#define EXTI_PIN_7 87
//the pin number of encoder, ENCODERX is for motorX




extern "C"{
void Count_and_Direction(Wheel *wheel) {
			
	wheel->count += 1;
	if (wheel->dir == FORWARD) {
	   //forward
	  wheel->counter ++;
	}
	else {
	  //backward
	  wheel->counter --;
	}	

}

float velocity_calculate(Wheel *omni) {
  float ang;
  ang = float(omni->count * 1.0);
  if (omni->dir == 1) {
    ang = -ang;
  }
  omni->count = 0;
  return ang;
}

}


void MOTOR_controller(MOTOR_CTL &motor_ctl)
{
 if(abs(motor_ctl.target) <= 250 && abs(motor_ctl.target) >= 0)
  {
    if(abs(motor_ctl.target) >= 1)
    {
      if (motor_ctl.target >= 0) 
      {
        digitalWrite(motor_ctl.EN, LOW);
        digitalWrite(motor_ctl.FR, LOW);
      }
      else 
      {
        digitalWrite(motor_ctl.EN, LOW);
        digitalWrite(motor_ctl.FR, HIGH);
      }
      drv_pwm_set_duty(motor_ctl.SV_pwm, (uint8_t)abs(motor_ctl.target));
    }
    else
    {
      digitalWrite(motor_ctl.EN, HIGH);       
    }
  }
  else
  {
      digitalWrite(motor_ctl.EN, HIGH);
  }   
}


CMotor::CMotor()
{
  m_ctl[0].EN = MOTOR1_EN;
  m_ctl[0].SV_pwm= MOTOR1_SV;
  m_ctl[0].FR = MOTOR1_FR;
  m_ctl[0].PG_exti = MOTOR1_PG;
  m_ctl[0].BK = MOTOR_BK;
  m_ctl[0].target = 0;

  m_ctl[1].EN = MOTOR2_EN;
  m_ctl[1].SV_pwm= MOTOR2_SV;
  m_ctl[1].FR = MOTOR2_FR;
  m_ctl[1].PG_exti = MOTOR2_PG;
  m_ctl[1].BK = MOTOR_BK;
  m_ctl[1].target = 0;



  m_wheel[0].encoder_a = ENCODER1_A;
  m_wheel[0].encoder_b = ENCODER1_B;
  m_wheel[0].counter = 0;
  m_wheel[0].count = 0;
  m_wheel[0].dir = 0;

  m_wheel[1].encoder_a = ENCODER2_A;
  m_wheel[1].encoder_b = ENCODER2_B;
  m_wheel[1].counter = 0;
  m_wheel[1].count = 0;
  m_wheel[1].dir = 0;
}

CMotor::~CMotor()
{
}

void CMotor::Init()
{
  pinMode(m_ctl[0].EN, OUTPUT);
  pinMode(m_ctl[0].FR, OUTPUT);
  pinMode(m_ctl[0].PG_exti, INPUT);

  pinMode(m_ctl[1].EN, OUTPUT);
  pinMode(m_ctl[1].FR, OUTPUT);
  pinMode(m_ctl[1].PG_exti, INPUT);

  pinMode(m_ctl[1].BK, OUTPUT);
  digitalWrite(m_ctl[1].BK,HIGH);

}

void CMotor::controller(double s1,double s2)
{
    if(SPEED_COMPARE(target_speed[0], s1) == 1 && speed[0] != 0.0)
    {
    	m_ctl[0].target = 0;
	MOTOR_controller(m_ctl[0]);
    }
    else
    {
       target_speed[0] = s1;
	m_ctl[0].target = s1;
	MOTOR_controller(m_ctl[0]);
    }

    if(SPEED_COMPARE(target_speed[1], s2) == 1 && speed[1] != 0.0)
    {
    	m_ctl[1].target = 0;
	MOTOR_controller(m_ctl[1]);
    }
    else
    {
       target_speed[1] = s2;
	m_ctl[1].target = s2;
	MOTOR_controller(m_ctl[1]);
    }
	
    this->setDir(m_ctl[0].target,m_wheel[0]);
    // The right wheel is the opposite direction
    this->setDir((0-m_ctl[1].target),m_wheel[1]);
}

bool CMotor::readEncoder(int &s1,int &s2)
{
	__disable_irq();
	s1 = m_wheel[0].counter;
	s2 = m_wheel[1].counter;
	__enable_irq();
	return true;
}

void CMotor::setSpeed(double s1,double s2)
{
  m_ctl[0].target = s1;
  m_ctl[1].target = s2;
}
void CMotor::FeedbackSpeed(double s1,double s2)
{
	return;
}

void CMotor::setDir(int target,Wheel &wheel)
{
    if(target > 0)
      wheel.dir = FORWARD;
    else if(target == 0)
      return;
    else
      wheel.dir = BACKWARD;
}


