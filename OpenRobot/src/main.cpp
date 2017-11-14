#include <stdlib.h>
#include <math.h>

#include "sys.h"
#include "delay.h"
#include "drv_include.h"
#include "drv_led.h"
#include "IMU.h"

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <turtlebot3_msgs/SensorState.h>

#include "CPackage.h"
#include "CMotor.h"
#include "drv_exti.h"
#include "UARTClass.h"
#include "main.h"
#include "USBSerial.h"
#include "mpu.h"
#include "mpu9250_iic.h"
#include "24cxx.h"
#include "DataScope_DP.h"
#include "CKalman.h"
#include "CSonar.h"

#define MAIN_DEBUG
#ifdef MAIN_DEBUG
#define fprintf(f) uart1_print(f)
#else
#define fprintf(f)
#endif

//static bool flag_odom = false;
//static char flag_sensor = 0;
//USBSerial Serial;
volatile int ttt_mp = 0;

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
* Subscriber
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& init_pose_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<geometry_msgs::PoseWithCovarianceStamped> init_pose_sub("initialpose", InitPoseCallback);
/*******************************************************************************
* Publisher
*******************************************************************************/
// Bumpers, cliffs, buttons, encoders, battery of Turtlebot3
turtlebot3_msgs::SensorState sensor_state_msg;
ros::Publisher sensor_state_pub("sensor_state", &sensor_state_msg);

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped tfs_msg;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tfbroadcaster;

/*******************************************************************************
* Declaration for IMU
*******************************************************************************/
cIMU imu;
CKalman kalman;

/*******************************************************************************
* Declaration for MOTOR
*******************************************************************************/
double goal_linear_x_velocity  = 0.0;
double goal_linear_y_velocity  = 0.0;
double goal_angular_velocity = 0.0;
CMotor motor;
Speed speed_motor = {0};


static int32_t old_wheel_encoder[3] = {0};
static int32_t diff_wheel_encoder[3] = {0};
double odometry_x=0;
double odometry_y=0;
double odometry_th=0;
static double last_theta = 0.0;

bool init_encoder_[2]  = {false, false};
int32_t last_diff_tick_[2];
int32_t last_tick_[2];
double last_rad_[2];
double last_velocity_[2];

/*******************************************************************************
* Declaration for Sonar
*******************************************************************************/
CSonar sonar;
/*******************************************************************************
* Declaration for SYSTERM
*******************************************************************************/
static unsigned long prev_update_time = 0;
unsigned long watch_dog = 0, last_time = 0;
char ex_log_msg[256];
USBSerial Serial;
extern UARTClass Serial1;
extern UARTClass Serial2;   // Arduino Serial
uint32_t stateLed_Hz = 2;

robot_serial ros_serial;
IMU_MSG imu_info = {0};
ODOM_MSG odom_info = {0};
uint8_t ros_readbuffer[256] = {0};



/*-------------------------------------------------------------------------------------------------------------*/
/*        
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好        
*/
double ProcessNiose_Q=0.002;
double MeasureNoise_R=0.02;
double KalmanFilter(const double ResrcData)
{
        double R = MeasureNoise_R;
        double Q = ProcessNiose_Q;

        static        double x_last;

        double x_mid = x_last;
        double x_now;

        static        double p_last;

        double p_mid ;
        double p_now;
        double kg;        

        x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
        kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
        x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
        p_now=(1-kg)*p_mid;//最优值对应的covariance        

        p_last = p_now; //更新covariance值
        x_last = x_now; //更新系统状态值

        return x_now;                
}

/*-------------------------------------------------------------------------------------------------------------*/



void system_Init(void)
{

  Cache_Enable();                 //打开L1-Cache
  HAL_Init();				        //初始化HAL库
  Stm32_Clock_Init(432,25,2,9);   //设置时钟,216Mhz 
  delay_init(216);                //延时初始化

	__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
	
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{

  double wheel_speed_cmd[2];
  double lin_vel1;
  double lin_vel2;
//  char rosmsg[256] = {0};

  wheel_speed_cmd[LEFT]  = cmd_vel_msg.linear.x- (cmd_vel_msg.angular.z * WHEEL_SEPARATION / 2);
  wheel_speed_cmd[RIGHT] = cmd_vel_msg.linear.x + (cmd_vel_msg.angular.z * WHEEL_SEPARATION / 2);

  lin_vel1 = wheel_speed_cmd[LEFT] * 312.5;
  if (lin_vel1 > 256)
  {
    lin_vel1 =  256;
  }
  else if (lin_vel1 < -256)
  {
    lin_vel1 = -256;
  }

  lin_vel2 = wheel_speed_cmd[RIGHT] * 312.5;
  if (lin_vel2 > 256)
  {
    lin_vel2 =  256;
  }
  else if (lin_vel2 < -256)
  {
    lin_vel2 = -256;
  }

	speed_motor.v_motor1 = (int)lin_vel1;
	speed_motor.v_motor2 = (int)-lin_vel2;

//   sprintf(ex_log_msg, "M1:%f   M2:%f  M3:%f",speed_motor.v_motor1,speed_motor.v_motor2,speed_motor.v_motor3);
//   nh.loginfo(ex_log_msg);
//  pidflag = true;
  watch_dog = millis();
}


void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& init_pose_msg)
{
	odometry_x = init_pose_msg.pose.pose.position.x;
	odometry_y = init_pose_msg.pose.pose.position.y;
	odometry_th = atan2f(init_pose_msg.pose.pose.orientation.y*init_pose_msg.pose.pose.orientation.z + init_pose_msg.pose.pose.orientation.x*init_pose_msg.pose.pose.orientation.w,
                     0.5f -init_pose_msg.pose.pose.orientation.z*init_pose_msg.pose.pose.orientation.z - init_pose_msg.pose.pose.orientation.w*init_pose_msg.pose.pose.orientation.w);

	imu.filter.q0 = init_pose_msg.pose.pose.orientation.w;
	imu.filter.q1 = init_pose_msg.pose.pose.orientation.y;
	imu.filter.q2 = init_pose_msg.pose.pose.orientation.y;
	imu.filter.q3 = init_pose_msg.pose.pose.orientation.z;

	last_theta = odometry_th;

}


/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(void)
{
  static bool gyro_cali = false;
  uint32_t pre_time;
  uint32_t t_time;

  char log_msg[50];

  if (nh.connected())
  {
    if (gyro_cali == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      imu.SEN.gyro_cali_start();

      t_time   = millis();
      pre_time = millis();
      while(!imu.SEN.gyro_cali_get_done())
      {
        imu.update();

        if (millis()-pre_time > 5000)
        {
          break;
        }
        if (millis()-t_time > 100)
        {
          t_time = millis();
          //LED_TOGGLE(1);
        }
      }
      gyro_cali = true;

      sprintf(log_msg, "Calibrattion End");
      nh.loginfo(log_msg);
    }
  }
  else
  {
    gyro_cali = false;
  }
}



/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
   bool dxl_comm_result = false;

  int32_t current_tick;

  sensor_state_msg.stamp = nh.now();
//  sensor_state_msg.battery = checkVoltage();

  dxl_comm_result = motor.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  if (dxl_comm_result == true)
  {
    sensor_state_pub.publish(&sensor_state_msg);
  }
  else
  {
    return;
  }

  current_tick = sensor_state_msg.left_encoder;

  if (!init_encoder_[LEFT])
  {
    last_tick_[LEFT] = current_tick;
    init_encoder_[LEFT] = true;
  }

  last_diff_tick_[LEFT] = current_tick - last_tick_[LEFT];
  last_tick_[LEFT] = current_tick;
  last_rad_[LEFT] += TICK2RAD * (double)last_diff_tick_[LEFT];

  current_tick = sensor_state_msg.right_encoder;

  if (!init_encoder_[RIGHT])
  {
    last_tick_[RIGHT] = current_tick;
    init_encoder_[RIGHT] = true;
  }

  last_diff_tick_[RIGHT] = current_tick - last_tick_[RIGHT];
  last_tick_[RIGHT] = current_tick;
  last_rad_[RIGHT] += TICK2RAD * (double)last_diff_tick_[RIGHT];
}


/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool updateOdometry(double diff_time)
{
  
  float angle = 0.0;
  float angle_kalman = 0.0;

 double odom_vel[3];

  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick_[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick_[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  angle = atan2f(imu.quat[1]*imu.quat[2] + imu.quat[0]*imu.quat[3],
                     0.5f - imu.quat[2]*imu.quat[2] - imu.quat[3]*imu.quat[3]);
  angle_kalman = kalman.update(angle);
  delta_theta = angle_kalman - last_theta;


  v = delta_s / step_time;
  w = delta_theta / step_time;

  last_velocity_[LEFT]  = wheel_l / step_time;
  last_velocity_[RIGHT] = wheel_r / step_time;

  // compute odometric pose
  odometry_x += delta_s * cos(odometry_th + (delta_theta / 2.0));
  odometry_y += delta_s * sin(odometry_th + (delta_theta / 2.0));
  odometry_th += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  odom.pose.pose.position.x = odometry_x;
  odom.pose.pose.position.y = odometry_y;
  odom.pose.pose.position.z = odometry_th*57.59;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odometry_th);

  // We should update the twist of the odometry
  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];

  last_theta = atan2f(imu.quat[1]*imu.quat[2] + imu.quat[0]*imu.quat[3],
                      0.5f - imu.quat[2]*imu.quat[2] - imu.quat[3]*imu.quat[3]);

  return true;
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom.header.frame_id = "odom";
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation = odom.pose.pose.orientation;
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;
  prev_update_time = time_now;
  ros::Time stamp_now = nh.now();

  // odom
  updateOdometry((double)(step_time * 0.001));
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);


  // tf
  updateTF(odom_tf);
  tfbroadcaster.sendTransform(odom_tf);
}

void publishImuMsg(void)
{
  //short ax,ay,az;
  //short gx,gy,gz;
  //MPU_Get_Accelerometer(&ax,&ay,&az);
  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.angular_velocity.x = imu.SEN.gyroADC[0];
  imu_msg.angular_velocity.y = imu.SEN.gyroADC[1];
  imu_msg.angular_velocity.z = imu.SEN.gyroADC[2];
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

#if 1
  imu_msg.linear_acceleration.x = imu.SEN.accADC[0];
  imu_msg.linear_acceleration.y = imu.SEN.accADC[1];
  imu_msg.linear_acceleration.z = imu.SEN.accADC[2];
#else
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
#endif
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation.w = imu.quat[0];
  imu_msg.orientation.x = imu.quat[1];
  imu_msg.orientation.y = imu.quat[2];
  imu_msg.orientation.z = imu.quat[3];

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu_pub.publish(&imu_msg);

	tfs_msg.header.stamp    = nh.now();
	tfs_msg.header.frame_id = "base_link";
	tfs_msg.child_frame_id  = "imu_link";
	tfs_msg.transform.rotation.w = imu.quat[0];
	tfs_msg.transform.rotation.x = imu.quat[1];
	tfs_msg.transform.rotation.y = imu.quat[2];
	tfs_msg.transform.rotation.z = imu.quat[3];

	tfs_msg.transform.translation.x = -0.032;
	tfs_msg.transform.translation.y = 0.0;
	tfs_msg.transform.translation.z = 0.068;

	tfbroadcaster.sendTransform(tfs_msg);
}



void loop()
{
	static uint64_t f2_timer[3] = {0};

/*********************************************************************************

				Recv Fail for a long time

*********************************************************************************/
#if 1
  last_time = millis() - watch_dog;
  if (last_time <= 1300) {
    //pidflag = true;
  }
  else {
    speed_motor.v_motor1 = 0;
    speed_motor.v_motor2 = 0;
    speed_motor.v_motor3 = 0;
  }
#endif
/*********************************************************************************

			Odom

*********************************************************************************/
#if 1
	if( (millis()- f2_timer[0]) >= (1000/30) )// 30Hz
	{
		publishSensorStateMsg();
		publishDriveInformation();
		f2_timer[0] = millis();
	}
#endif
/*********************************************************************************

			IMU

*********************************************************************************/
	if( (millis()- f2_timer[1]) >= (1000/200) )// 200Hz
	{
		publishImuMsg();
		f2_timer[1] = millis();
	}
	if( (millis()- f2_timer[2]) >= 1000/stateLed_Hz )// stateLed_Hz Hz
	{
		LED_TOGGLE(0);
		Serial.write((uint8_t*)"OK",2);
		f2_timer[2] = millis();
	}

	//sonar.update();
	imu.update();
	//updateGyroCali();

	
	nh.spinOnce();

}



void timer_Handle(void)
{

#if 1
	__disable_irq();
	motor.speed[0] = velocity_calculate(&motor.m_wheel[0]);
	motor.speed[1] = velocity_calculate(&motor.m_wheel[1]);
	__enable_irq();
#endif
	//motor.setSpeed(speed_motor.v_motor1, speed_motor.v_motor2);
	motor.FeedbackSpeed(motor.speed[0], motor.speed[1]);
	motor.controller(speed_motor.v_motor1, speed_motor.v_motor2);

#if 0
	float ang = atan2f(imu.quat[1]*imu.quat[2] + imu.quat[0]*imu.quat[3],
                       0.5f - imu.quat[2]*imu.quat[2] - imu.quat[3]*imu.quat[3]);//*57.29578f;
      
	//kalman_out = KalmanFilter(ang);
	float out;
	out = sonar.output;
	DataScope_Get_Channel_Data((ang*57.29578f),1);
	DataScope_Get_Channel_Data((out),2);
	
	Serial1.write(DataScope_OutPut_Buffer,DataScope_Data_Generate(2));
#endif
}

extern "C"{
//interrupt function of encoder1
void Encoder_Counter_1() {
  Count_and_Direction(&motor.m_wheel[0]);
}

//interrupt function of encoder2
void Encoder_Counter_2() {
  Count_and_Direction(&motor.m_wheel[1]);
}
}
int main(void)
{
/*************************************************************
						Hardware Init
***************************************************************/
	bsp_init();
	drv_uart_init();
	
	drv_spi_init();
	drv_micros_init();
	drv_Led_Init();                     //初始化LED
	USB_Init();
/*************************************************************
						Ros Init
***************************************************************/
#if 1
	nh.getHardware()->setBaud(115200);
	nh.initNode();
	
	nh.subscribe(cmd_vel_sub);
	nh.subscribe(init_pose_sub);
	nh.advertise(imu_pub);
	nh.advertise(odom_pub);
	nh.advertise(sensor_state_pub);
	tfbroadcaster.init(nh);
#endif
/*************************************************************
						Serial Init
***************************************************************/
      //Serial1.begin((uint32_t)115200);
/*************************************************************
						Motor Drive Init
***************************************************************/	
#if 1
	drv_motor_pwm_init();
	motor.Init();
	speed_motor.v_motor1 = 0.0;
	speed_motor.v_motor2 = 0.0;
	speed_motor.v_motor3 = 0.0;
#endif
/*************************************************************
						Motor Encoder Init
***************************************************************/	
#if 1
	drv_exti_init();
	pinMode(80,INPUT);
	pinMode(81,INPUT);
	pinMode(82,INPUT);
	pinMode(83,INPUT);
	pinMode(84,INPUT);
	pinMode(85,INPUT);
	pinMode(86,INPUT);
	pinMode(87,INPUT);

	attachInterrupt(10, Encoder_Counter_1, RISING);
	attachInterrupt(11, Encoder_Counter_2, RISING);
#endif

#if 1
	if(imu.begin() == IMU_OK)
	{
		kalman.setMeasureNoise_R(0.002);
		kalman.setMeasureNoise_R(0.2);
		BEEP_LONG;
	}
	else
	{
		BEEP_ONCE;
	}
#else
	if(MPU9250_Init()==0) 
	{
		BEEP_LONG;
	}
	else
	{
		BEEP_ONCE;
	}
#endif	
/*************************************************************
						Timer Init
***************************************************************/
#if 1
  	drv_timer_init();
	drv_timer_pause(TIMER_CH2);
	drv_timer_set_period(TIMER_CH2,60000);
	drv_timer_attachInterrupt(TIMER_CH2,timer_Handle);
	drv_timer_resume(TIMER_CH2);
#endif

/*************************************************************
						Sonar Init
***************************************************************/
#if 0
	sonar.Init();
#endif

       prev_update_time = millis();


	while(1)
	{
		loop();
	}
}


