/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __speedcontrol_h_
#define __speedcontrol_h_


/*******伺服方向***********/
#define DIRECTION_POSITIVE	0
#define DIRECTION_NEGATIVE	1

/**---------伺服电机发送脉冲判断运行模式---------**/
#define AUTORUNNINGMODE       0x00	  //自动运行模式
#define JOGRUNNINGMODE        0x01	  //手动寸动运行模式
#define LINKEDRUNNINGMODE	  0x02    //手动连续运行模式
#define BACKTOORIGIN		  0x03	  //回原点模式

#define SPEED_UP		0x01        //增加速度
#define SPEED_HOLD		0x02        //保持速度
#define SPEED_DOWN		0x03        //降低速度

//#define PLUS_ACCEL		0x01        //加加速度
//#define HOLD_ACCEL		0x02        //匀加速度
//#define MINUS_ACCEL		0x03        //减加速度
//#define ZERO_ACCEL		0x04        //最大速度-保持段

//36000:1k 360:100k
#define PULSE_MINHALFPERIOD 180	   //最大速度对应最小半周期



extern u16 *SpeedUp_Jerk;
extern u16 *SpeedUp_Max_Acceleration;
extern u16 SpeedUp_Jerk1[10];
extern u16 SpeedUp_Max_Acceleration1[10];
extern u16 SpeedUp_Jerk2[10];
extern u16 SpeedUp_Max_Acceleration2[10];
extern u16 SpeedUp_Jerk3[10];
extern u16 SpeedUp_Max_Acceleration3[10];
extern u16 Present_SpeedUp_Jerk[6];               
extern u16 Present_SpeedUp_Max_Acceleration[6]; 
extern u32 Present_Pulse_SpeedControlStep_Counter[6];
// extern u16 SpeedUp_Max_A;6.29
//--6.29
extern u16 SpeedUp_Max_A[6];
extern u16 SpeedUp_Max_A_Z;
extern u16 SpeedUp_Max_A_X;
extern u16 SpeedUp_Max_A_O;
extern u16 SpeedUp_Max_A_Y;

extern u16 SpeedUp_Max_Acceleration_Sheet[81];

extern u8  Timer_Enable_X;
extern u8  Timer_Enable_Z;
extern u8  Timer_Enable_L;
extern u8  Timer_Enable_O;
extern u8  Timer_Enable_A;
extern u8  Timer_Enable_B;
extern u32 Servo_Pulse_Count[6];	      //给伺服电机发生脉冲个数
extern u8  Servo_Control_Mode ; 		  //伺服器运行模式状态
extern u32 Pulse_Counter[6] ;
extern u32 Pulse_ControlStep[6];
extern u32 SpeedUp_Pulse_Counter[6];
//extern u32 Pulse_MinHalfPeriod;		  // 最大速度脉冲值
extern u32 Pulse_Maxspeed_HalfPeriod[6]; // 最大速度对应半周期
extern u32 PULSE_MINHALF_PERIOD;
extern u32 Pulse_InitialHalfPeriod[6] ;
extern u32 Pulse_TotalCounter[6];
extern u16 Pulse_PresentAcceleration[6];
extern u32 AUTO_PULSE_MINHALF_PERIOD,MANUAL_PULSE_MINHALF_PERIOD;

extern void SendPulse(u8);  //发送脉冲函数
void  ClosePulseReset(u8 Axis);


#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

