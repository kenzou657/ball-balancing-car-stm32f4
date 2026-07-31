#include "balance.h"

//int Time_count=0; //Time variable //计时变量 

u8 Lidar_Detect = Lidar_Detect_ON;			//电磁巡线模式雷达检测障碍物，默认开启
int RC_Lidar_Avoid_FLAG = RC_Avoid_OFF;			//遥控模式雷达避障，默认关闭
u8 Mode;
float RC_Velocity_CCD=350,RC_Velocity_ELE=350; 
float PS2_Velocity=350,PS2_Turn_Velocity;			//遥控控制的速度
Encoder OriginalEncoder; //Encoder raw data //编码器原始数据    
u8 Lidar_Success_Receive_flag=0,Lidar_flag_count=0;//雷达在线标志位，雷达掉线计数器

// Robot mode is wrong to detect flag bits
//机器人模式是否出错检测标志位
int robot_mode_check_flag=0; 

short test_num;
u8 command_lost_count=0;//

Encoder OriginalEncoder; //Encoder raw data //编码器原始数据     

//========== PWM清除使用变量 ==========//
u8 start_check_flag = 0;//标记是否需要清空PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //标记开始清除PWM
u8 clear_done_once = 0; //清除完成标志位
u16 clear_again_times = 0;
float debug_show_diff = 0;
//void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/


/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //全向移动小车才开启速度平滑处理
	if((Mode == PS2_Control_Mode||Mode == APP_Control_Mode)&&RC_Lidar_Avoid_FLAG&&Lidar_Success_Receive_flag)//app以及ps2控制下，判断是否开启遥控避障
	{
		RC_Lidar_Aviod(&Vx,&Vy,&Vz);
	}
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
  
      //Get the smoothed data 
			//获取平滑处理后的数据			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
		//Mecanum wheel car
	  //麦克纳姆轮小车
	  if (Car_Mode==Mec_Car) 
    {
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//Omni car
		//全向轮小车
		else if (Car_Mode==Omni_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=0;	//Out of use //没有使用到
		}
		
		//Ackermann structure car
		//阿克曼小车
		else if (Car_Mode==Akm_Car) 
		{
			//Ackerman car specific related variables //阿克曼小车专用相关变量
			float R, Ratio=636.56, AngleR, Angle_Servo;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//对于阿克曼小车Vz代表右前轮转向角度
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
			AngleR=target_limit_float(AngleR,-0.49f,0.32f);
			
			//Inverse kinematics //运动学逆解
			if(AngleR!=0)
			{
				MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
				MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;			
			}
			else 
			{
				MOTOR_A.Target = Vx;
				MOTOR_B.Target = Vx;
			}
			// The PWM value of the servo controls the steering Angle of the front wheel
			//舵机PWM值，舵机控制前轮转向角度
			//Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
			Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.755f;
			Servo=SERVO_INIT + (Angle_Servo - 1.755f)*Ratio;

			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
			Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //舵机PWM值限幅
			}
		
		//Differential car
		//差速小车
		else if (Car_Mode==Diff_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_D.Target = Vx - Vz * Wheel_spacing / 2.0f; //计算出左轮 MotorD 的目标速度
			MOTOR_A.Target = Vx + Vz * Wheel_spacing / 2.0f; //计算出右轮 MotorA 的目标速度
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude);
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude);
			MOTOR_B.Target=0; //Out of use //没有使用到
			MOTOR_C.Target=0; //Out of use //没有使用到
			
		}
		
		//FourWheel car
		//四驱车
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
					
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 		
		}
		
		//Tank Car
		//履带车
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //计算出右轮的目标速度
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
		}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
//	static u8 Count_CCD = 0;								//调节CCD控制频率
	static u8 last_mode = 0;
	u8 contest_key = key_stateless;
	u32 lastWakeTime = getSysTickCnt();

	while(1)
	{	
		// This task runs at a frequency of 100Hz (10ms control once)
		//此任务以100Hz的频率运行（10ms控制一次）
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
		//Time count is no longer needed after 30 seconds
		//时间计数，30秒后不再需要
		if(SysVal.Time_count<3000)SysVal.Time_count++;
		//Get the encoder data, that is, the real time wheel speed, 
		//and convert to transposition international units
		//获取编码器数据，即车轮实时速度，并转换位国际单位
		Get_Velocity_Form_Encoder();   

		if(++Lidar_flag_count==10) Lidar_Success_Receive_flag=0,Lidar_flag_count=0,RC_Lidar_Avoid_FLAG=RC_Avoid_OFF; //100ms清零雷达接收到数据标志位
		Contest_Task_Update(CONTEST_TRACK_PERIOD_MS);
		contest_key = KEY_Scan(RATE_100_HZ, 0);
		Contest_Task_KeyAction(contest_key);
		 
		if(last_mode != Mode)  //切换模式时屏幕清零
		{
			last_mode++;
			OLED_Clear();
			if(last_mode>6)
			{
//				OLED_Clear();
				last_mode = 0;
			}
		} 
				
		if(Mode != ELE_Line_Patrol_Mode)
			Buzzer_Alarm(0);
		if(Contest_Task_IsRunning() == 0)
		{
			//Contest mode only: keep the fixed differential chassis stopped until long press starts a task.
			//赛题模式专用：长按启动任务前，固定差速车保持停止，不再进入旧遥控/巡线模式。
			Move_X = 0.0f;
			Move_Y = 0.0f;
			Move_Z = 0.0f;
			MOTOR_A.Target = 0.0f;
			MOTOR_B.Target = 0.0f;
			MOTOR_C.Target = 0.0f;
			MOTOR_D.Target = 0.0f;
		}
		//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
		//and the software failure flag is 0
		//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
		if(Turn_Off(Voltage)==0) 
		 { 			
			 //Speed closed-loop control to calculate the PWM value of each motor, 
			 //PWM represents the actual wheel speed					 
			 //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
			 Limit_Pwm(16500) ;
			 //Set different PWM control polarity according to different car models
			 //根据不同小车型号设置不同的PWM控制极性
			 switch(Car_Mode)
			 {
					case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  -MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //麦克纳姆轮小车
					case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Omni car                //全向轮小车
					case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //阿克曼小车
					case Diff_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  0,  0, -MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //两轮差速小车：右轮MotorA，左轮MotorD
					case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //四驱车 
					case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //履带车
			 }
		 }
		 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
		 //如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
		 else	Set_Pwm(0,0,0,0,0); 
			 	
	}  
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			PWMA2=16799,PWMA1=16799+motor_a;
	else 	            PWMA1=16799,PWMA2=16799-motor_a;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c<0)			PWMC2=16799,PWMC1=16799+motor_c;
	else 	            PWMC1=16799,PWMC2=16799-motor_c;
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            	PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//舵机控制。滚球稳定启用时覆盖普通舵机输入，PC9/TIM8_CH4 输出滚球舵机 PWM。
	if(Ball_Control_IsServoOverrideEnabled())
	{
		Servo_PWM = Ball_Control_GetServoPwm();
	}
	else
	{
		Servo_PWM = servo;
	}
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC1=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>14000)Pwm=14000;
	 if(Pwm<-14000)Pwm=-14000;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>14000)Pwm=14000;
	 if(Pwm<-14000)Pwm=-14000;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>14000)Pwm=14000;
	 if(Pwm<-14000)Pwm=-14000;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>14000)Pwm=14000;
	 if(Pwm<-14000)Pwm=-14000;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
	 switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //如果无方向控制指令，检查转向控制状态
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转
		 else 		               Move_Z=0;                       //stop           //停止
	 }
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:     Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	
	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_Control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作
	static u8 acc_dec_filter = 0;		
	
	if(PS2_ON_Flag==1)
	{
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
		
	  //Ignore small movements of the joystick //忽略摇杆小幅度动作
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
	if(++acc_dec_filter==15)
	{
		acc_dec_filter=0;
	  if (PS2_KEY==11)	    RC_Velocity+=5;  //To accelerate//加速
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //减速	
	}
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //对PS2手柄控制命令进行处理
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
		Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //阿克曼结构小车转换为前轮转向角度
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
	}
		//Control target value is obtained and kinematics analysis is performed
	  //得到控制目标值，进行运动学分析
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 
/**************************************************************************
函数功能：CCD巡线，采集3个电感的数据并提取中线 
入口参数：无
返回  值：无
**************************************************************************/
void  Get_RC_CCD(void)
{
	static float Bias,Last_Bias;
	float move_z=0;
									
			Move_X=RC_Velocity_CCD;													//CCD巡线模式线速度
			Bias=CCD_Median-64;  //提取偏差，64为巡线的中心点
	    if(Car_Mode == Omni_Car)
			  move_z=-Bias*Omni_Car_CCD_KP*0.1f-(Bias-Last_Bias)*Omni_Car_CCD_KI*0.1f; //PD控制，原理就是使得小车保持靠近巡线的中心点
			else if(Car_Mode == Tank_Car)
				move_z=-Bias*Tank_Car_CCD_KP*0.1f-(Bias-Last_Bias)*Tank_Car_CCD_KI*0.1f;
			else
				move_z=-Bias*CCD_KP*0.1f-(Bias-Last_Bias)*CCD_KI*0.1f;
			Last_Bias=Bias;   //保存上一次的偏差
			
			if(Car_Mode==Mec_Car)															
			{
				Move_Z=move_z*RC_Velocity_CCD/50000;							//差速控制原理需要经过此处处理
			}

			else if(Car_Mode==Omni_Car)											
			{
				Move_Z=move_z*RC_Velocity_CCD/21000;							//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Akm_Car)												
			{
				Move_Z=move_z/450;																//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Diff_Car)		
			{	
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/67000;					//差速控制原理需要经过此处处理	
			}
			else if(Car_Mode==Tank_Car)		
			{	
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/50000;					//差速控制原理需要经过此处处理	
			}
			else if(Car_Mode==FourWheel_Car)									
			{
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/20100;					//差速控制原理需要经过此处处理
			}			
		
			//Z-axis data conversion //Z轴数据转化	
			//Unit conversion, mm/s -> m/s
			//单位转换，mm/s -> m/s
			Move_X=Move_X/1000;
			Move_Z=Move_Z;
			Move_Y=0;
			//Control target value is obtained and kinematics analysis is performed
			//得到控制目标值，进行运动学分析
			Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
函数功能：电磁巡线，采集3个电感的数据并提取中线 
入口参数：无
返回  值：无
**************************************************************************/
void  Get_RC_ELE(void)
{
	static float Bias,Last_Bias;
	float move_z=0;
	
	if(Detect_Barrier() == No_Barrier)
	{
			Move_X=RC_Velocity_ELE;				//电磁巡线模式的速度
			Bias=100-Sensor;  //提取偏差	
      if(Car_Mode == Omni_Car)		
			  move_z=-Bias* Omni_Car_ELE_KP*0.08f-(Bias-Last_Bias)* Omni_Car_ELE_KI*0.05f; 
			else if(Car_Mode == Tank_Car)
				move_z=-Bias*Tank_Car_ELE_KP*0.1f-(Bias-Last_Bias)*Tank_Car_ELE_KI*0.1f;
			else
				move_z=-Bias* ELE_KP*0.08f-(Bias-Last_Bias)* ELE_KI*0.05f; 
			Last_Bias=Bias; 
		  Buzzer_Alarm(0);

			if(Car_Mode==Mec_Car)															
			{		
				Move_Z=move_z*RC_Velocity_ELE/50000;					//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Omni_Car)											
			{
				Move_Z=move_z*RC_Velocity_ELE/10800;					//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Diff_Car)		
			{
				if(Move_X<0) move_z=-move_z;			
				Move_Z=move_z*RC_Velocity_ELE/45000;					//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Tank_Car)		
			{
				if(Move_X<0) move_z=-move_z;			
				Move_Z=move_z*RC_Velocity_ELE/28000;					//差速控制原理需要经过此处处理
			}
			else if(Car_Mode==FourWheel_Car)									
			{
				if(Move_X<0) move_z=-move_z;
				Move_Z=move_z*RC_Velocity_ELE/20100;					//差速控制原理需要经过此处处理
			}
			
			else if(Car_Mode==Akm_Car)											
			{
				Move_Z=move_z/450;														//差速控制原理需要经过此处处理
			}
		}
	
	else									//有障碍物
	{
		Buzzer_Alarm(100);				//当电机使能的时候，有障碍物则蜂鸣器报警
		Move_X = 0;
		Move_Z = 0;
	}

			//Z-axis data conversion //Z轴数据转化	
			//Unit conversion, mm/s -> m/s
			//单位转换，mm/s -> m/s
			Move_X=Move_X/1000;
			Move_Z=Move_Z;
	
			//Control target value is obtained and kinematics analysis is performed
			//得到控制目标值，进行运动学分析
	    Move_Y=0;
			Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
函数功能：检测前方是否有障碍物
入口参数：无
返回  值：无
**************************************************************************/
u8 Detect_Barrier(void)
{
	int i;
	u8 point_count = 0;
	
	if(Lidar_Detect == Lidar_Detect_ON)
	{
		for(i=0;i<1152;i++)	//检测是否有障碍物 
		{
			if((Dataprocess[i].angle>300)||(Dataprocess[i].angle<60))
			{
				if(0<Dataprocess[i].distance&&Dataprocess[i].distance<700)//700mm内是否有障碍物
					point_count++;
		  }
	}
		if(point_count > 0)//有障碍物
			return Barrier_Detected;
		else
			return No_Barrier;
	}
	else
		return No_Barrier;
}

/**************************************************************************
函数功能：小车避障模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Avoid(void)
{
	int i = 0; 
	u8 calculation_angle_cnt = 0;	//用于判断225个点中需要做避障的点
	float angle_sum = 0;			//粗略计算障碍物位于左或者右
	u8 distance_count = 0;			//距离小于某值的计数
	for(i=0;i<1152;i++)				//遍历120度范围内的距离数据，共120个点左右的数据
	{
		if((Dataprocess[i].angle>300)||(Dataprocess[i].angle<60))  //避障角度在300-60之间
		{
			if((0<Dataprocess[i].distance)&&(Dataprocess[i].distance<Avoid_Distance))	//距离小于450mm需要避障,只需要120度范围内点
			{
				calculation_angle_cnt++;						 			//计算距离小于避障距离的点个数
				if(Dataprocess[i].angle<60)		
					angle_sum += Dataprocess[i].angle;
				else if(Dataprocess[i].angle>300)
					angle_sum += (Dataprocess[i].angle-360);	//300度到60度转化为-60度到60度
				if(Dataprocess[i].distance<Avoid_Min_Distance)				//记录小于200mm的点的计数
					distance_count++;
			}
	  }
	}
	Move_X = forward_velocity;
  if(calculation_angle_cnt == 0)//不需要避障
	 {
		Move_Z = 0;
	 }
	else                          //当距离小于200mm，小车往后退
	{
		if(distance_count>8)
		{
			Move_X = -forward_velocity;
			Move_Z = 0;
		}
		else
		{
			Move_X = 0;
			if(angle_sum > 0)//障碍物偏右
			{
				if(Car_Mode == Mec_Car)  //麦轮转弯需要把前进速度降低
					Move_X = 0;
				else                     //其他车型保持原有车速
				  Move_X = forward_velocity;
				
				if(Car_Mode == Akm_Car)
					Move_Z = PI/4;
				else if(Car_Mode == Omni_Car)
					Move_Z=corner_velocity;
				else
				  Move_Z=other_corner_velocity;//左转
			}
			else //偏左
			{
				if(Car_Mode == Mec_Car)
					Move_X = 0;
				else
				  Move_X = forward_velocity;
				
				if(Car_Mode == Akm_Car)
					Move_Z = -PI/4;
				else if(Car_Mode == Omni_Car)
					Move_Z=-corner_velocity;//右转
				else
					Move_Z=-other_corner_velocity;
			}
	  }
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}


/**************************************************************************
函数功能：小车跟随模式 
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Follow(void)
{
	static u16 cnt = 0;
	int i;
	int calculation_angle_cnt = 0;
	static float angle = 0;				//避障的角度
	static float last_angle = 0;		//
	u16 mini_distance = 65535;
	static u8 data_count = 0;			//用于滤除一写噪点的计数变量
	//需要找出跟随的那个点的角度
	for(i = 0; i < 1152; i++)
	{
			if((0<Dataprocess[i].distance)&&(Dataprocess[i].distance<Follow_Distance))
			{
				calculation_angle_cnt++;
				if(Dataprocess[i].distance<mini_distance)
				{
					mini_distance = Dataprocess[i].distance;
					angle = Dataprocess[i].angle;
				}
			}
	}
	if(angle > 180)  //0--360度转换成0--180；-180--0（顺时针）
		angle -= 360;
	if((angle-last_angle > 10)||(angle-last_angle < -10))   //做一定消抖，波动大于10度的需要做判断
	{
		if(++data_count > 30)   //连续30次采集到的值(300ms后)和上次的比大于10度，此时才是认为是有效值
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else    //波动小于10度的可以直接认为是有效值
	{
		if(++data_count > 10)   //连续10次采集到的值(100ms后)，此时才是认为是有效值
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	if(calculation_angle_cnt < 8)  //跟随距离小于8且当cnt>40的时候，认为在1600内没有跟随目标
	{
		if(cnt < 40)
			cnt++;
		if(cnt >= 40)
		{
			Move_X = 0;
			Move_Z = 0;
		}
	}
	else
	{
		cnt = 0;
		if(Move_X > 0.06f || Move_X < -0.06f)  //当Move_X有速度时，转向PID开始调整
		{
			if(mini_distance < 700 && (last_angle > 60 || last_angle < -60))
			{
				Move_Z = -0.0098f*last_angle;  //当距离偏小且角度差距过大直接快速转向
			}
			else
			{
				  Move_Z = -Follow_Turn_PID(last_angle,0);		//转向PID，车头永远对着跟随物品
			}
		}
		else
		{
			Move_Z = 0;
		}
		if(angle>150 || angle<-150)  //如果小车在后方60°需要反方向运动以及快速转弯
		{
			Move_X = -Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);
			Move_Z = -0.0098f*last_angle;
		}
		else
		{
		  Move_X = Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);  //保持距离保持在500mm
		}
		Move_X = target_limit_float(Move_X,-amplitude_limiting,amplitude_limiting);   //对前进速度限幅
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
函数功能：小车走直线模式
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_along_wall(void)
{
	static u32 target_distance=0;
	static int i=0;

	u32 distance, last_distance;
	u8 data_count = 0;			//用于滤除一写噪点的计数变量
	
	Move_X = forward_velocity;  //初始速度
	
	for(int j=0;j<1152;j++) //225
	{
		if(Dataprocess[j].angle>268 && Dataprocess[j].angle<272)   //取雷达的4度的点
		{
			if(i==0)
			{
				target_distance=Dataprocess[j].distance;  //雷达捕获第一个距离
				i++;
			}
			 if(Dataprocess[j].distance<(target_distance+limit_distance))//限制一下雷达的探测距离
			 {
				 data_count++;
				 distance=Dataprocess[j].distance;//实时距离
				 if(distance==0)		distance = last_distance;
				 last_distance = distance;
			 }
		}
	}
	if(Car_Mode == Mec_Car || Car_Mode == Omni_Car)  //只有麦轮和全向可以用Move_Y
	{
		Move_Y=Along_Adjust_PID(distance,target_distance);
		Move_X = forward_velocity;
		Move_Z = 0;
	}
	else   //其他车型使用Move_Z保持走直线状态
	{
		Move_Z=Along_Adjust_PID(distance,target_distance);
		Move_X = forward_velocity;
		Move_Y = 0;
	}
	if(data_count == 0)  //当data_count等于0，只有前进速度
	{
		Move_Y = 0;
		Move_Z = 0;
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
函数功能：小车遥控避障功能
入口参数：无
返回  值：无
**************************************************************************/
void RC_Lidar_Aviod(float *Vx,float *Vy,float *Vz)
{
	int i;
	int avoid_distance=450;//避障距离
	u16 avoid_point_count=0;//范围内需要壁障的点数
	int left_angle_sum=0, right_angle_sum=0;
	int mini_distance_count=0;//最小转弯距离的点数
	for(i=0;i<1152;i++)
	{
		if(Dataprocess[i].angle>300||Dataprocess[i].angle<60)
		{//车前避障角度 300°-60°
			if(0<Dataprocess[i].distance&&Dataprocess[i].distance<avoid_distance)
			{
				avoid_point_count++;//需要避障的点数
				if(Dataprocess[i].angle<60)
				{
					right_angle_sum+=Dataprocess[i].angle;
				}
				else if(Dataprocess[i].angle>300)
				{
					left_angle_sum+=(Dataprocess[i].angle-360);//300°到359°转化为-60°到-1°
				}
				if(Dataprocess[i].distance<Avoid_Min_Distance)
				{//小于最小避障距离
					mini_distance_count++;
				}
			}
		}
	
	}
	if(*Vx>0&&avoid_point_count>8){//需要避障
		if(mini_distance_count>=8)//距离小于最小转弯距离要求
		{
			*Vx=*Vx*-0.75f;//后退
			*Vz=0;
		}
		else
		{
			if(right_angle_sum+left_angle_sum>0)
			{//往左避让
				*Vx=*Vx*0.5f;//避障的X轴速度是前进速度的一半
				if(Car_Mode==Akm_Car){
					*Vz=(PI/2)*4/9;
				}else{
					*Vz=1;
				}
			}else{//往右避让
				*Vx=*Vx*0.5f;//避障的X轴速度是前进速度的一半
				if(Car_Mode==Akm_Car){
					*Vz=(-PI/2)*4/9;
				}else{
					*Vz=-1;
				}
				
			}
		}
	}
}

/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= -OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;
	
	if(PS2_ON_Flag)
	{
		step=0.05;
	}
	else
	{
		step=0.01;
	}
	
	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else temp = a;
	return temp;
}

/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转。已停止使用
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//如果连续6次接近满幅输出，判断为电机乱转，让电机失能	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：走直线雷达距离pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/	 	
//走直线雷达距离调整pid

float Along_Adjust_PID(float Current_Distance,float Target_Distance)//距离调整PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//计算偏差
	Integral_bias+=Bias;	                                 			//求出偏差的积分
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Mode == FourWheel_Car)
		OutPut=FourWheel_Along_Distance_KP*Bias/100000+FourWheel_Along_Distance_KI*Integral_bias/100000+FourWheel_Along_Distance_KD*(Bias-Last_Bias)/100000;//位置式PID控制器
	else if(Car_Mode == Akm_Car)
		OutPut=Akm_Along_Distance_KP*Bias/100000+Akm_Along_Distance_KI*Integral_bias/100000+Akm_Along_Distance_KD*(Bias-Last_Bias)/1000;//位置式PID控制器
	else if(Car_Mode == Diff_Car || Car_Mode == Tank_Car)
		OutPut=Diff_Along_Distance_KP*Bias/100+Diff_Along_Distance_KI*Integral_bias/100+Diff_Along_Distance_KD*(Bias-Last_Bias);//位置式PID控制器
	else
	  OutPut=-Along_Distance_KP*Bias/100000-Along_Distance_KI*Integral_bias/100000-Along_Distance_KD*(Bias-Last_Bias)/1000;//位置式PID控制器
	Last_Bias=Bias;                                       		 			//保存上一次偏差
	if(Turn_Off(Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                          	
}


/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
函数功能：跟随雷达距离pid
入口参数: 当前距离和目标距离
返回  值：电机目标速度
**************************************************************************/	 	
//跟随雷达距离调整pid
float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//距离调整PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//计算偏差
	Integral_bias+=Bias;	                                 			//求出偏差的积分
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	OutPut=Distance_KP*Bias/100+Distance_KI*Integral_bias/100+Distance_KD*(Bias-Last_Bias)/100;//位置式PID控制器
	Last_Bias=Bias;                                       		 			//保存上一次偏差
	if(Turn_Off(Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                          	
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
函数功能：跟随雷达转向pid
入口参数: 当前角度和目标角度
返回  值：电机转向速度
**************************************************************************/	 	
//跟随雷达转向pid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //计算偏差
	Integral_bias+=Bias;	                                 				 //求出偏差的积分
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Mode == Akm_Car || Car_Mode == Omni_Car)
		OutPut=(Follow_KP_Akm/100)*Bias+(Follow_KI_Akm/100)*Integral_bias+(Follow_KD_Akm/100)*(Bias-Last_Bias);	//位置式PID控制器
	else
	  OutPut=(Follow_KP/100)*Bias+(Follow_KI/100)*Integral_bias+(Follow_KD/100)*(Bias-Last_Bias);	//位置式PID控制器
	Last_Bias=Bias;                                       					 		//保存上一次偏差
	if(Turn_Off(Voltage)== 1)								//电机关闭，此时积分清零
		Integral_bias = 0;
	return OutPut;                                           					 	//输出
	
}

