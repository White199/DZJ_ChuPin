/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : main.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "SpeedControl.h" 
#include "StatusControl.h"
#include "Auto.h"
#include "Manual.h"
#include "Parameter.h"
#include "Error.h"
#include "SpeedControl_CAN.h"


#define SET_X_PULSE_PERIOD_HALF(X)		{TIM3->ARR = (X + X - 1); TIM3->CCR1 = X; TIM3->CCR2 = X;}	   //X-TIM3
#define CLOSE_X_PULSE					TIM_Cmd(TIM3, DISABLE)

#define SET_Z_PULSE_PERIOD_HALF(X)		{TIM2->ARR = (X + X - 1); TIM2->CCR1 = X; TIM2->CCR2 = X;}	   //Z-TIM2
#define CLOSE_Z_PULSE					TIM_Cmd(TIM2, DISABLE)

#define SET_L_PULSE_PERIOD_HALF(X)		{TIM1->ARR = (X + X - 1); TIM1->CCR3 = X; TIM1->CCR4 = X;}	   //L-TIM1-CH3
#define CLOSE_L_PULSE					TIM_Cmd(TIM1, DISABLE)

#define SET_O_PULSE_PERIOD_HALF(X)		{TIM4->ARR = (X + X - 1); TIM4->CCR3 = X; TIM4->CCR3 = X;}	   //O-TIM4-CH3
#define CLOSE_O_PULSE					TIM_Cmd(TIM4, DISABLE)

#define SET_X_DIRECTION_H	     	GPIO_SetBits  (GPIOA, GPIO_Pin_7);		//PA7
#define SET_X_DIRECTION_L			GPIO_ResetBits(GPIOA, GPIO_Pin_7);

#define SET_Z_DIRECTION_H			GPIO_SetBits  (GPIOA, GPIO_Pin_1);		//PA1
#define SET_Z_DIRECTION_L			GPIO_ResetBits(GPIOA, GPIO_Pin_1);

#define SET_L_DIRECTION_H	     	GPIO_SetBits  (GPIOE, GPIO_Pin_14);		//PE14
#define SET_L_DIRECTION_L			GPIO_ResetBits(GPIOE, GPIO_Pin_14);

#define SET_O_DIRECTION_H			GPIO_SetBits  (GPIOB, GPIO_Pin_9);		//PB9
#define SET_O_DIRECTION_L			GPIO_ResetBits(GPIOB, GPIO_Pin_9);



/*------伺服器运行模式状态------*/
u8 Servo_Control_Mode = LINKEDRUNNINGMODE;

u32 Pulse_TotalCounter[6] = {MINROBOTPOSITION, MINROBOTPOSITION ,MINROBOTPOSITION ,MINROBOTPOSITION,MINROBOTPOSITION,MINROBOTPOSITION};				//X轴、Z轴、L轴脉冲总数量

#define SPEED_CONTROL_TIME		18000                //每隔2ms(36000)加一次脉冲，如果每个脉冲周期大于2ms，则每个脉冲周期都加
#define MAX_SPEED	       	    100000 				 //最大速度100k
/*---需要发送的脉冲个数---*/				
u32 Servo_Pulse_Count[6] = {0};			
u32 AUTO_PULSE_MINHALF_PERIOD=PULSE_MINHALFPERIOD; //自动运行经过速度等级变换后的最大速度对应的最小半周期 -*/
u32 PULSE_MINHALF_PERIOD=PULSE_MINHALFPERIOD; //经过速度等级变换后的最大速度对应的最小半周期 -*/



/*---单次脉冲发送过程中发送的个数---*/
u32 Pulse_Counter[6] = {0};						//一个控制过程的脉冲发送总个数
u32 Pulse_PresentHalfPeriod[6] = {18000,18000,18000,18000}; //当前（半）周期---*/	
u32 Pulse_InitialHalfPeriod[6] = {18000,18000,18000,18000};  //脉冲半周期初始值：初始速度值---*/ 
u32 Pulse_Maxspeed_HalfPeriod[6] = {180,180,180,180,180,180}; //最大速度对应最小半周期 -*/
u32 Pulse_Maxspeed_HalfPeriod_i[6]={0};     

       u8  SpeedUp_Min_Acceleration=20;           //加速度初始值  50-->20
       u16 Present_SpeedUp_Jerk[6]={10,10,10,10};               //当前加加速度值 
       u16 Present_SpeedUp_Max_Acceleration[6]={200,200,200,200,200,200};  //当前最大加速度值
       u8  Pulse_Record_Flag[6]={FALSE,FALSE,FALSE,FALSE};			      //加加速度记录标志位
       u16 Pulse_PresentAcceleration[6] ={20,20,20,20};	      //当前加速度
//	   u16 Pulse_LastAcceleration[4] ={20,20,20,20};	      //记录加速到最大速度时的最后一次加速度
       u32 Delta_Present_MaxSpeed[6]={0};              //当前最大速度的补偿误差
       u32 Pulse_Present_MaxSpeed[6]={100000,100000,100000,100000};         //100k
static u32 Pulse_MinSpeed[6] = {1000,1000,1000,1000};			      //初始最小速度 与 初始速度保持一致
static u32 Present_SpeedUp_Min_Acceleration[6] = {20,20,20,20}; //初始最小加速度 与 初始速度保持一致
       u32 Period_Counter_Acceleration[6]={0};  //最小加速度累加到最大加速度需要的周期数 (550-50)/10=50
static u32 Period_Counter[6]={0};               //加速阶段周期数
static u32 Period_Counter2[6]={0};              //加速阶段-Down周期数
static u32 Period_Counter_UpA[6]={0};           //加速阶段加速度变化的周期数
static u32 Period_Counter_HoldA[6]={0};         //加速阶段匀速度变化的周期数
static u32 Period_Counter_DownA[6]={0};         //加速阶段减速度变化的周期数
       u32 Delta_Speed_Frequency[6]={0};        //从最小加速度加速到最大加速度所需要的脉冲
       u32 Amin_Counter[6]={0};

/*---速度参数---*/	
       u32 Present_Pulse_SpeedControlStep_Counter[6]={0}; //记录当前的速度规划周期脉冲步数
static u32 NextSpeed[6] = {0};                   	     //修改之后为下个中断对应的速度值
static u32 Pulse_InitialSpeed[6] = {1000,1000,1000,1000};			     //当前周期内的初始速度-实时更新
static u32 Pulse_SpeedControlStep_Counter[6] = {0};	     //每隔2ms加减速所使用的脉冲数计数
static u32 Remain_SpeedControlStep_Counter[6] = {0};	   //每隔2ms加减速所使用的脉冲数计数

/*---加减速步骤---*/	
u32 Pulse_ControlStep[6] = {SPEED_UP,SPEED_UP,SPEED_UP,SPEED_UP};	
/*---加速使用脉冲数---*/	
u32 SpeedUp_Pulse_Counter[6] = {0}; 			  //记录加速度使用的脉冲个数，计算得出速度各段的使用脉冲数

//----------------------------------//  
u8 A_List[6]={0};//加速度表下标

                         
/***********************************************************
**  功能：速度规划   
**	时间：2016-07-18
**	作者：wx
**	描述：将速度规划问题转变为加加速度的变化问题
**  加加速度：   SpeedUp_Jerk   最大加速度： SpeedUp_Max_Acceleration
**  18000:1k    3600:5K   1800:10k  360:50k  180:100k
% ***************************2016-07-24-01*********************************
%     Vmin=1k=1000;HP=36000;  Vmax=100K=100000; HP=180; Amin=50;
%        V           HP(min)  J       Amax         Ta(max)     Sa(max) 
%     1%~9%(Vmax)    18000    3     140(Amax)-30  	  t= 86*2ms   S=  9.95mm  
% 	  10%~19%(Vmax)  1800     4     170(Amax)-30      t=137*2ms   S= 29.85mm
%     20%~29%(Vmax)  900      5     200(Amax)-30      t=170*2ms   S= 53.71mm
% 	  30%~39%(Vmax)  600      6     230(Amax)-30  	  t=198*2ms   S= 83.37mm
%     40%~49%(Vmax)  450      6     260(Amax)-35      t=221*2ms   S=115.05mm  
%     50%~59%(Vmax)  360      8     330(Amax)-35	    t=213*2ms   S=132.57mm
% 	  60%~69%(Vmax)  300      10    400(Amax)-35      t=212*2ms   S=154.62mm	
% 	  70%~79%(Vmax)  257      10    450(Amax)-40      t=221*2ms   S=186.31mm	
% 	  80%~89%(Vmax)  225      10    500(Amax)-45      t=224*2ms   S=208.40mm	
% 	  90%~100%(Vmax) 200      10    550(Amax)-50      t=231*2ms   S=238.15mm
% ******************************************* ******************************/
//u16 SpeedUp_Jerk[10]             = {3,  4,  5,  6,  6,  8,  10, 10 ,10, 10 };
//u16 SpeedUp_Max_Acceleration[10] = {140,170,200,230,260,330,400,450,500,550};

//2016-09-14-wtmx
u16 *SpeedUp_Jerk=0;
u16 *SpeedUp_Max_Acceleration=0;

u16 SpeedUp_Jerk1[10]             = {10,  10,  5,  5,  5,  5,  5,  5,  5,  5};
u16 SpeedUp_Max_Acceleration1[10] = {200,220,300,320,340,360,380,400,420,440};

u16 SpeedUp_Jerk2[10]             = {15,  15, 10, 10, 10, 10, 10, 10, 10, 10};
u16 SpeedUp_Max_Acceleration2[10] = {200,260,300,340,380,420,460,500,540,580};

u16 SpeedUp_Jerk3[10]             = {15,  15, 10, 10, 10, 10, 10, 10, 10, 10};
u16 SpeedUp_Max_Acceleration3[10] = {200,260,300,340,380,420,460,500,540,580};

//2017-06-22-S_Style Sheet
//200ms  210ms 220ms ....990ms 1000ms
// u16 SpeedUp_Max_A=0;  6.29注释
//---6.29每个轴对应一个最大加速度
u16 SpeedUp_Max_A_X=0;
u16 SpeedUp_Max_A_Z=0;
u16 SpeedUp_Max_A_L=0;
u16 SpeedUp_Max_A_O=0;
//---6.29
u16 SpeedUp_Max_A[6]={0};

u16 SpeedUp_Max_Acceleration_Sheet[81]=
{ 
  780, 670, 600, 550, 510, 480, 450, 420,
  400, 380, 370, 350, 340, 330, 320, 300,
  290, 280, 270, 270, 260, 250,	250, 240,
  240, 230, 220, 220, 210, 210, 200, 200,
  190, 190, 190, 180, 180, 180, 170, 170, 
  170, 160, 160, 160, 160, 150, 150, 150,
  150, 140, 140, 140, 140, 140,	130, 130,
  130, 130, 130, 120, 120, 120, 120, 120,
  120, 120, 110, 110, 110, 110, 110, 110,
  110, 100, 100, 100, 100, 100, 100, 100,
	100
};


u8 Timer_Enable_X = DISABLE;						 //脉冲控制定时器是否使能
u8 Timer_Enable_Z = DISABLE;						 //脉冲控制定时器是否使能
u8 Timer_Enable_L = DISABLE;						 //脉冲控制定时器是否使能
u8 Timer_Enable_O = DISABLE;						 //脉冲控制定时器是否使能
u8 Timer_Enable_A = DISABLE;						 //脉冲控制定时器是否使能
u8 Timer_Enable_B = DISABLE;						 //脉冲控制定时器是否使能
//---2017.8.1
u32 pulse_Present_MaxSpeed;
u32 servo_Pulse_Count;

//------------------------------------------//
u8 Sheet_Position[6]={0};//遥杆加减速对应的加速度在表中的位置
u8 Speed_Up_Flag[6]={0},Speed_Down_Flag[6]={0};//加速还是减速的标志位


extern u8 A_List[6];//加速度表下标

/**************************************************************************************************
**  函数名：  SpeedPlanningReset()
**	输入参数：无
**	输出参数：无
**	函数功能：复位相关标志位
**	备注：	  			  
**  作者：    吴祥     
**  开发日期：2016/10/11 
***************************************************************************************************/
void  SpeedPlanningReset(u8 Axis)
{
	 Period_Counter[Axis]=0;
	 Period_Counter2[Axis]=0;
	 Amin_Counter[Axis]=0;
	 Pulse_Record_Flag[Axis]=FALSE;
	 Period_Counter_UpA[Axis]=0;
	 Period_Counter_HoldA[Axis]=0;
	 Period_Counter_DownA[Axis]=0;
	 Pulse_Maxspeed_HalfPeriod_i[Axis]=0;
	 NextSpeed[X_Axsis] = 1000;
}
/**************************************************************************************************
**  函数名：  MaxSpeedSection()
**	输入参数：无
**	输出参数：无
**	函数功能：根据最大速度值来分段计算加速度
**	备注：	  			  
**  作者：    吴祥     
**  开发日期：2013/1/23 
***************************************************************************************************/
void  MaxSpeedSection(u8 Axis)
{
	  
	  /*
	  if(Pulse_Maxspeed_HalfPeriod[Axis]>18000)  //1%=36000 *2 =72000>65536 超出范围
	  {
	  	  Pulse_Maxspeed_HalfPeriod[Axis]=18000; //1%	
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[0];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[0];
	  }
	  //1%<=最大速度<10%
	  if(Pulse_Maxspeed_HalfPeriod[Axis]>1800)	//最大速度<10%
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[0];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[0];
	  }
	  //10%<=最大速度<20%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>900 && Pulse_Maxspeed_HalfPeriod[Axis]<=1800)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[1];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[1];
	  }	
	  //20%<=最大速度<30%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>600 && Pulse_Maxspeed_HalfPeriod[Axis]<=900)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[2];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[2];
	  }	
	  //30%<=最大速度<40%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>450 && Pulse_Maxspeed_HalfPeriod[Axis]<=600)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[3];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[3];
	  }	
	  //40%<=最大速度<50%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>360 && Pulse_Maxspeed_HalfPeriod[Axis]<=450)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[4];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[4];
	  }	
	  //50<=最大速度<60%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>300 && Pulse_Maxspeed_HalfPeriod[Axis]<=360)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[5];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[5];
	  }	
	  //60%<=最大速度<70%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>257 && Pulse_Maxspeed_HalfPeriod[Axis]<=300)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[6];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[6];
	  }	
	  //70%<=最大速度<80%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>225 && Pulse_Maxspeed_HalfPeriod[Axis]<=257)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[7];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[7];
	  }	
	  //80%<=最大速度<90%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>200 && Pulse_Maxspeed_HalfPeriod[Axis]<=225)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[8];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[8];
	  }		  
	  //90%<=最大速度<=100%
	  else if(Pulse_Maxspeed_HalfPeriod[Axis]>=180 && Pulse_Maxspeed_HalfPeriod[Axis]<=200)  
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[9];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[9];
	  }	
	  else
	  {
	  	  Present_SpeedUp_Jerk[Axis]=SpeedUp_Jerk[0];
		  Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_Acceleration[0];
	  }
	  */
// 		Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_A;  6.29注释

    Present_SpeedUp_Max_Acceleration[Axis]=SpeedUp_Max_A[Axis];
		
	    //计算当前周期的最大速度值Vmax
		Pulse_Present_MaxSpeed[Axis]=18000000/Pulse_Maxspeed_HalfPeriod[Axis]; 
		//计算从最小加速度到最大加速度值的脉冲周期
		Period_Counter_Acceleration[Axis]=(Present_SpeedUp_Max_Acceleration[Axis]-SpeedUp_Min_Acceleration)/Present_SpeedUp_Jerk[Axis];
		//计算从最小加速度到最大加速度值的脉冲距离
		Delta_Speed_Frequency[Axis]=(Present_SpeedUp_Max_Acceleration[Axis]+SpeedUp_Min_Acceleration)*(Period_Counter_Acceleration[Axis]+1)/2;
		
		//最大速度补偿计算误差 如期望速度为10000,实际的可达到为9980
		if((Pulse_Present_MaxSpeed[Axis]-Pulse_MinSpeed[Axis])>(Delta_Speed_Frequency[Axis]+Delta_Speed_Frequency[Axis]))
		{
			Delta_Present_MaxSpeed[Axis]=(Pulse_Present_MaxSpeed[Axis]-Pulse_MinSpeed[Axis]-Delta_Speed_Frequency[Axis]-Delta_Speed_Frequency[Axis])%Present_SpeedUp_Max_Acceleration[Axis];
			Pulse_Present_MaxSpeed[Axis]=Pulse_Present_MaxSpeed[Axis]-Delta_Present_MaxSpeed[Axis];
			Pulse_Maxspeed_HalfPeriod[Axis]=18000000/Pulse_Present_MaxSpeed[Axis];
		}		
}


		
/**************************************************************************************************
**  函数名：  SendPulse()
**	输入参数：无
**	输出参数：无
**	函数功能：给伺服器发送脉冲
**	备注：	  根据全局变量Servo_Control_Mode，Servo_Pulse_Count，Axsis_Chosen
**			  来确定发送脉冲的脉冲个数和控制轴。
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void SendPulse(u8 Axis)	 //主要变量ControlMode,Axsis,PulseCount
{
	 switch(Servo_Control_Mode)
	 {	 	 
			 case BACKTOORIGIN:				 //回原点模式
				  Servo_Pulse_Count[Axis] = MINROBOTPOSITION;
				  break;
			 
			 case AUTORUNNINGMODE:			 //自动运行模式
			  	  Servo_Pulse_Count[Axis] = Auto_Pulse_Count;
				  break;

			 case JOGRUNNINGMODE:			 //寸动运行模式
			      Servo_Pulse_Count[Axis] = Jog_Pulse_Count;
				  break;

			 case LINKEDRUNNINGMODE:		 //连动运行模式
				  Servo_Pulse_Count[Axis] = Linked_Pulse_Count;
				  break;

			 default:
				  break;	  
	 }
	 
	 /*- 根据最大速度分段计算加速度 -*/
   SpeedPlanningReset(Axis);
   MaxSpeedSection(Axis);
	 /*-- 判断伺服器运动方向 --*/
   DirectionJudge(Axis);		 
	 switch(Axis)
	 {
	 	 case X_Axsis: 
		  	  if( (Axsis_Move_Direction[Axis]!= NONE)&&(Servo_Pulse_Count[Axis]!=0) )
			  {
				  TIM_Cmd(TIM3, ENABLE);
				  Timer_Enable_X = ENABLE;
			  }
			  break;
		 
		 case Z_Axsis:
		  	  if( (Axsis_Move_Direction[Axis]!= NONE)&&(Servo_Pulse_Count[Axis]!=0) )
			  {
			      TIM_Cmd(TIM2, ENABLE);
		  	    Timer_Enable_Z = ENABLE;
			  }
			  break;

	 	 case L_Axsis: 		  //料仓轴
		  	  if( (Axsis_Move_Direction[Axis]!= NONE)&&(Servo_Pulse_Count[Axis]!=0) )
			  {
			      TIM_Cmd(TIM1, ENABLE);
			      Timer_Enable_L = ENABLE;
			  }
			  break;
	 	 case O_Axsis: 		  //料仓轴
		  	  if( (Axsis_Move_Direction[Axis]!= NONE)&&(Servo_Pulse_Count[Axis]!=0) )
			  {
			      TIM_Cmd(TIM4, ENABLE);
			      Timer_Enable_O = ENABLE;
			  }
			  break;			  				 
		 default:
		      break;		     	 
	 }
	  
	 
}



/**************************************************************************************************
**  函数名：  SpeedPlanning()
**	输入参数：无
**	输出参数：无
**	函数功能：由加加速度和最大加速度以及当前速度来确定加速度值
**	备注：	  			  
**  作者：    吴祥     
**  开发日期：2016/7/18 
***************************************************************************************************/
void SpeedPlanning(u8 Axis)
{
	if(Error_Status != NO_ERROR)//错误状态检测
	{
		if(Pulse_PresentHalfPeriod[Axis]>=180 && Pulse_PresentHalfPeriod[Axis]<=450)
		{
			Pulse_PresentAcceleration[Axis] = SpeedUp_Max_Acceleration[9];
		}
		else
		{
			Pulse_PresentAcceleration[Axis] = SpeedUp_Max_Acceleration[5];
		}
	}
	else
	{
		Period_Counter[Axis]++;//对应轴周期+1 		  
		/*----速度加速度阶段进行规划--------*/
		if(Pulse_ControlStep[Axis] == SPEED_UP)//当前轴处于加速模式
		{
			//经过匀加速段后,进入减加速阶段a=a-j			//还处于匀加速周期 && 下个速度还未到最大速度 && 最大速度-下一个周期的速度 <= ？？
			if( (Period_Counter_HoldA[Axis]!=0) && (Pulse_Present_MaxSpeed[Axis]>NextSpeed[Axis]) && 
				((Pulse_Present_MaxSpeed[Axis]-NextSpeed[Axis])<=(Delta_Speed_Frequency[Axis]-Present_SpeedUp_Max_Acceleration[Axis])) )//？？？
			{
				//		减加速阶段周期数	  =	   加速阶段周期数   -（   加加速阶段周期数     +	匀加速阶段周期数  		）
				Period_Counter_DownA[Axis]=Period_Counter[Axis]-(Period_Counter_UpA[Axis]+Period_Counter_HoldA[Axis]);
				if(Pulse_PresentAcceleration[Axis]==Present_SpeedUp_Min_Acceleration[Axis])//若当前加速度与最小加速度一样（<50）
				{
					Amin_Counter[Axis]++;          //加速段达到最小速度的周期数
				}
				else
				{	//当前加速度=当前最大加速度-减加速周期*当前加加速度
					Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Max_Acceleration[Axis]-Period_Counter_DownA[Axis]*Present_SpeedUp_Jerk[Axis];
					if(Pulse_PresentAcceleration[Axis]<Present_SpeedUp_Min_Acceleration[Axis])//若计算出的当前加速度小于最小加速度
					{
						Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Min_Acceleration[Axis];//当前加速度=最小加速度
					}
				}
			//Period_Counter_DownA=Period_Counter-(Period_Counter_UpA+Period_Counter_HoldA);
			}
			//加加速度阶段 ： a=a+j		//加加速度周期<从最小加到最大的周期
			else if((Pulse_Record_Flag[Axis]==FALSE)&&(Period_Counter_UpA[Axis]<=Period_Counter_Acceleration[Axis]))
			{
				Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Min_Acceleration[Axis]+Period_Counter_UpA[Axis]*Present_SpeedUp_Jerk[Axis];
				Period_Counter_UpA[Axis]=Period_Counter[Axis];
				//加加速到最大速度时,记录相关数据
				if((Pulse_Record_Flag[Axis]==FALSE)&&(Pulse_PresentAcceleration[Axis]>=Present_SpeedUp_Max_Acceleration[Axis]))
				{
					Pulse_Record_Flag[Axis]=TRUE;
					Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Max_Acceleration[Axis];
					Period_Counter_UpA[Axis]=Period_Counter[Axis]-1;
				}
			}
			//匀加速度阶段 ： a=Amax
			else if(Period_Counter_DownA[Axis]==0)
			{
				Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Max_Acceleration[Axis];
				Period_Counter_HoldA[Axis]=Period_Counter[Axis]-Period_Counter_UpA[Axis];
			}
			else
			{
			}
		}
		/*----速度匀速度阶段进行规划--不会进去此条件------*/
		else if(Pulse_ControlStep[Axis] == SPEED_HOLD)//当前轴处于匀速模式
		{
			if(Period_Counter_DownA[Axis]>0)
			{
				Pulse_PresentAcceleration[Axis]=Pulse_PresentAcceleration[Axis]+Present_SpeedUp_Jerk[Axis];
			}					  
		}
		/*----速度减速阶段进行规划--------*/
		else if(Pulse_ControlStep[Axis] == SPEED_DOWN)//当前轴处于减速模式
		{
			//加加速度阶段 ： a=a+j
			if((Period_Counter_DownA[Axis]>0)&&(Period_Counter2[Axis]<=(Period_Counter_DownA[Axis]-1)) )
			{					  	  						   
				if( Period_Counter2[Axis]>Amin_Counter[Axis])
				{
					Pulse_PresentAcceleration[Axis]=Pulse_PresentAcceleration[Axis]+Present_SpeedUp_Jerk[Axis];
					if(Pulse_PresentAcceleration[Axis]>Present_SpeedUp_Max_Acceleration[Axis])
					{
						Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Max_Acceleration[Axis];
					}
				}
			}
			//减加速度阶段 ： a=a-j
			else if((Period_Counter_HoldA[Axis]==0)||
			((Period_Counter_HoldA[Axis]>0) && (Period_Counter2[Axis]>=(Period_Counter_HoldA[Axis]+Period_Counter_DownA[Axis]-1)))) 
			{
				Pulse_PresentAcceleration[Axis]=Pulse_PresentAcceleration[Axis]-Present_SpeedUp_Jerk[Axis];
				if(Pulse_PresentAcceleration[Axis]<Present_SpeedUp_Min_Acceleration[Axis])
				{  
					Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Min_Acceleration[Axis];
				}
			}
			//匀加速度阶段 ： a=Amax
			else
			{
				Pulse_PresentAcceleration[Axis]=Present_SpeedUp_Max_Acceleration[Axis];
			}	
			Period_Counter2[Axis]++;	
		}//else if(Pulse_ControlStep == SPEED_DOWN)
		else
		{
			Pulse_PresentAcceleration[Axis]=SpeedUp_Max_Acceleration[0];		  
		}
	}
}

/**************************************************************************************************
**  函数名：  ClosePulseReset()
**	输入参数：无
**	输出参数：无
**	函数功能：脉冲发送结束后复位标志位
**	备注：	  			  
**  作者：    吴祥     
**  开发日期：2016/7/18 
***************************************************************************************************/
void  ClosePulseReset(u8 Axis)
{
	 Pulse_ControlStep[Axis] = SPEED_UP;	
	 Pulse_Maxspeed_HalfPeriod[Axis] = 180;	
	 Pulse_PresentHalfPeriod[Axis] = 18000;
	 SpeedUp_Pulse_Counter[Axis] = 0;
	 Pulse_Counter[Axis] = 0;
	 Pulse_SpeedControlStep_Counter[Axis] = 0;
	 Pulse_InitialSpeed[Axis] = 1000;
	 NextSpeed[Axis]=0;		
	 Period_Counter[Axis]=0;
	 Period_Counter2[Axis]=0;
	 Amin_Counter[Axis]=0;
	 Pulse_Record_Flag[Axis]=FALSE;
	 Period_Counter_UpA[Axis]=0;
	 Period_Counter_HoldA[Axis]=0;
	 Period_Counter_DownA[Axis]=0;
	 Present_Pulse_SpeedControlStep_Counter[Axis]=0;
	 Servo_Stop_Done[Axis]=FALSE;   //伺服器停止动作标志位复位	 		
   Axsis_Move_Direction[Axis] = NONE;
}
/**************************************************************************************************
**  函数名：  TIM3_IRQHandler()-X
**	输入参数：无
**	输出参数：无
**	函数功能：定时器3中断函数，X轴脉冲控制函数
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void TIM3_IRQHandler(void)
{	    
    if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
    {    
         TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	 
// 			 NextSpeed[X_Axsis] = 0;
			 switch(Pulse_ControlStep[X_Axsis])
			 {
				  case SPEED_UP:       
					   if( (u32)(Pulse_SpeedControlStep_Counter[X_Axsis]*Pulse_PresentHalfPeriod[X_Axsis]) >= SPEED_CONTROL_TIME )
					   {
						  Present_Pulse_SpeedControlStep_Counter[X_Axsis]=Pulse_SpeedControlStep_Counter[X_Axsis];
							Pulse_SpeedControlStep_Counter[X_Axsis] = 0;   
							SpeedPlanning(X_Axsis);		 //计算加速度20160718	
							NextSpeed[X_Axsis] = Pulse_InitialSpeed[X_Axsis] + Pulse_PresentAcceleration[X_Axsis];  //真是速度值 NextSpeed=fi  ===  Vi  
							if(NextSpeed[X_Axsis]>MAX_SPEED)
							{
								 NextSpeed[X_Axsis]=MAX_SPEED-Pulse_PresentAcceleration[X_Axsis];
							}
							Pulse_PresentHalfPeriod[X_Axsis] = 18000000 / NextSpeed[X_Axsis]; 
							Pulse_InitialSpeed[X_Axsis] = NextSpeed[X_Axsis];
							if(NextSpeed[X_Axsis]>=Pulse_Present_MaxSpeed[X_Axsis])
							{
						     SpeedUp_Pulse_Counter[X_Axsis] = Pulse_Counter[X_Axsis];                              
								 Pulse_ControlStep[X_Axsis] =  SPEED_HOLD;							
							}
					  }
					  if((Pulse_Counter[X_Axsis] + Pulse_Counter[X_Axsis] + Present_Pulse_SpeedControlStep_Counter[X_Axsis]+2) >= Servo_Pulse_Count[X_Axsis])
					  {  
							Pulse_ControlStep[X_Axsis] = SPEED_HOLD; 
							Remain_SpeedControlStep_Counter[X_Axsis]=Pulse_SpeedControlStep_Counter[X_Axsis];
							Pulse_SpeedControlStep_Counter[X_Axsis] = 0;
							SpeedUp_Pulse_Counter[X_Axsis] = Pulse_Counter[X_Axsis];				
					  }
					  break;

				 case SPEED_HOLD:			         
					  if(Pulse_SpeedControlStep_Counter[X_Axsis]>300)
					  {
					  	  Pulse_SpeedControlStep_Counter[X_Axsis]=0;
					  }
					  if(Servo_Pulse_Count[X_Axsis] - Pulse_Counter[X_Axsis] <= (SpeedUp_Pulse_Counter[X_Axsis]-Remain_SpeedControlStep_Counter[X_Axsis]+2) )
					  {				  						  						  
						  Pulse_SpeedControlStep_Counter[X_Axsis] = 0; 
						  Remain_SpeedControlStep_Counter[X_Axsis]=0;
						  //减加速一次进入减速段
						  NextSpeed[X_Axsis]=NextSpeed[X_Axsis]-Pulse_PresentAcceleration[X_Axsis];  
						  Pulse_PresentHalfPeriod[X_Axsis] = 18000000 / NextSpeed[X_Axsis];
						  Pulse_InitialSpeed[X_Axsis] = NextSpeed[X_Axsis];
						  SpeedPlanning(X_Axsis);
						  Pulse_ControlStep[X_Axsis] = SPEED_DOWN; 
					  }
					  break;

				 case SPEED_DOWN:
					  if((u32)(Pulse_SpeedControlStep_Counter[X_Axsis] * Pulse_PresentHalfPeriod[X_Axsis]) >= SPEED_CONTROL_TIME)
					  {			   
							Pulse_SpeedControlStep_Counter[X_Axsis] = 0;
						    SpeedPlanning(X_Axsis);		 //计算加速度20160718					  			
							NextSpeed[X_Axsis] = Pulse_InitialSpeed[X_Axsis] - Pulse_PresentAcceleration[X_Axsis];
							if(NextSpeed[X_Axsis]<Pulse_MinSpeed[X_Axsis])
							{
							    NextSpeed[X_Axsis]=Pulse_MinSpeed[X_Axsis];
							}
							Pulse_PresentHalfPeriod[X_Axsis] = 18000000 / NextSpeed[X_Axsis];
							Pulse_InitialSpeed[X_Axsis] = NextSpeed[X_Axsis];	
					  }
					  break;

				default:
					  break;
			 }  
			if(Pulse_PresentHalfPeriod[X_Axsis]>18000)
			{
				Pulse_PresentHalfPeriod[X_Axsis]=18000;
			}
			else if(Pulse_PresentHalfPeriod[X_Axsis]<180)
			{
				Pulse_PresentHalfPeriod[X_Axsis]=180;
			}
			SET_X_PULSE_PERIOD_HALF(Pulse_PresentHalfPeriod[X_Axsis]);
			Pulse_SpeedControlStep_Counter[X_Axsis]++;
			Pulse_Counter[X_Axsis]++;
			
			//记录X轴的总脉冲坐标数：
			if(Axsis_Move_Direction[X_Axsis] == POSITIVE)
			{ 
				 Pulse_TotalCounter[0] ++;
			}
			else
			{
				 Pulse_TotalCounter[0] --;
			}
			if(Error_Enable_Flag == TRUE)
			{					
				 CLOSE_X_PULSE;  
				 ClosePulseReset(X_Axsis);
				 Timer_Enable_X = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;	
			}		 

			/*=======================脉冲计数完成，停止脉冲，并清空数据=========================*/
			if( Pulse_Counter[X_Axsis] == Servo_Pulse_Count[X_Axsis])
			{
				 CLOSE_X_PULSE; 
				 ClosePulseReset(X_Axsis);
				 Timer_Enable_X = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;							 
			 }	   	  
	}
}

/**************************************************************************************************
**  函数名：  TIM2_IRQHandler()-Z
**	输入参数：无
**	输出参数：无
**	函数功能：定时器3中断函数，X轴脉冲控制函数
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void TIM2_IRQHandler(void)
{	    
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
    {    
             TIM_ClearITPendingBit(TIM2,TIM_IT_Update);	 
// 			 NextSpeed[Z_Axsis] = 0;
			 switch(Pulse_ControlStep[Z_Axsis])
			 {
				  case SPEED_UP:       
					   if( (u32)(Pulse_SpeedControlStep_Counter[Z_Axsis]*Pulse_PresentHalfPeriod[Z_Axsis]) >= SPEED_CONTROL_TIME )
					   {
						    Present_Pulse_SpeedControlStep_Counter[Z_Axsis]=Pulse_SpeedControlStep_Counter[Z_Axsis];
							Pulse_SpeedControlStep_Counter[Z_Axsis] = 0;   
							SpeedPlanning(Z_Axsis);		 //计算加速度20160718	
							NextSpeed[Z_Axsis] = Pulse_InitialSpeed[Z_Axsis] + Pulse_PresentAcceleration[Z_Axsis];  //真是速度值 NextSpeed=fi  ===  Vi  
							if(NextSpeed[Z_Axsis]>MAX_SPEED)
							{
								 NextSpeed[Z_Axsis]=MAX_SPEED-Pulse_PresentAcceleration[Z_Axsis];
							}
							Pulse_PresentHalfPeriod[Z_Axsis] = 18000000 / NextSpeed[Z_Axsis]; 
							Pulse_InitialSpeed[Z_Axsis] = NextSpeed[Z_Axsis];
							if(NextSpeed[Z_Axsis]>=Pulse_Present_MaxSpeed[Z_Axsis])
							{
						         SpeedUp_Pulse_Counter[Z_Axsis] = Pulse_Counter[Z_Axsis];                              
								 Pulse_ControlStep[Z_Axsis] =  SPEED_HOLD;							
							}
					  }
					  if((Pulse_Counter[Z_Axsis] + Pulse_Counter[Z_Axsis] + Present_Pulse_SpeedControlStep_Counter[Z_Axsis]+2) >= Servo_Pulse_Count[Z_Axsis])
					  {  
							Pulse_ControlStep[Z_Axsis] = SPEED_HOLD; 
							Remain_SpeedControlStep_Counter[Z_Axsis]=Pulse_SpeedControlStep_Counter[Z_Axsis];
							Pulse_SpeedControlStep_Counter[Z_Axsis] = 0;
							SpeedUp_Pulse_Counter[Z_Axsis] = Pulse_Counter[Z_Axsis];				
					  }
					  break;

				 case SPEED_HOLD:			         
					  if(Pulse_SpeedControlStep_Counter[Z_Axsis]>300)
					  {
					  	  Pulse_SpeedControlStep_Counter[Z_Axsis]=0;
					  }
					  if(Servo_Pulse_Count[Z_Axsis] - Pulse_Counter[Z_Axsis] <= (SpeedUp_Pulse_Counter[Z_Axsis]-Remain_SpeedControlStep_Counter[Z_Axsis]+2) )
					  {				  						  						  
						  Pulse_SpeedControlStep_Counter[Z_Axsis] = 0; 
						  Remain_SpeedControlStep_Counter[Z_Axsis]=0;
						  //减加速一次进入减速段
						  NextSpeed[Z_Axsis]=NextSpeed[Z_Axsis]-Pulse_PresentAcceleration[Z_Axsis];  
						  Pulse_PresentHalfPeriod[Z_Axsis] = 18000000 / NextSpeed[Z_Axsis];
						  Pulse_InitialSpeed[Z_Axsis] = NextSpeed[Z_Axsis];
						  SpeedPlanning(Z_Axsis);
						  Pulse_ControlStep[Z_Axsis] = SPEED_DOWN; 
					  }
					  break;

				 case SPEED_DOWN:
					  if((u32)(Pulse_SpeedControlStep_Counter[Z_Axsis] * Pulse_PresentHalfPeriod[Z_Axsis]) >= SPEED_CONTROL_TIME)
					  {			   
							Pulse_SpeedControlStep_Counter[Z_Axsis] = 0;
						    SpeedPlanning(Z_Axsis);		 //计算加速度20160718					  			
							NextSpeed[Z_Axsis] = Pulse_InitialSpeed[Z_Axsis] - Pulse_PresentAcceleration[Z_Axsis];
							if(NextSpeed[Z_Axsis]<Pulse_MinSpeed[Z_Axsis])
							{
							    NextSpeed[Z_Axsis]=Pulse_MinSpeed[Z_Axsis];
							}
							Pulse_PresentHalfPeriod[Z_Axsis] = 18000000 / NextSpeed[Z_Axsis];
							Pulse_InitialSpeed[Z_Axsis] = NextSpeed[Z_Axsis];	
					  }
					  break;

				default:
					  break;
			 }   
			if(Pulse_PresentHalfPeriod[Z_Axsis]>18000)
			{
				Pulse_PresentHalfPeriod[Z_Axsis]=18000;
			}
			else if(Pulse_PresentHalfPeriod[Z_Axsis]<180)
			{
				Pulse_PresentHalfPeriod[Z_Axsis]=180;
			}
			SET_Z_PULSE_PERIOD_HALF(Pulse_PresentHalfPeriod[Z_Axsis]);
			Pulse_SpeedControlStep_Counter[Z_Axsis]++;
			Pulse_Counter[Z_Axsis]++;
			
			//记录Z轴的总脉冲坐标数：
			if(Axsis_Move_Direction[Z_Axsis] == POSITIVE)
			{ 
				Pulse_TotalCounter[1] ++;
			}
			else
			{
				Pulse_TotalCounter[1] --;
			}
			if(Error_Enable_Flag == TRUE)
			{					
				 CLOSE_Z_PULSE;  
				 ClosePulseReset(Z_Axsis);
				 Timer_Enable_Z = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;	
			}		 

			/*=======================脉冲计数完成，停止脉冲，并清空数据=========================*/
			if( Pulse_Counter[Z_Axsis] == Servo_Pulse_Count[Z_Axsis])
			{
				 CLOSE_Z_PULSE; 
				 ClosePulseReset(Z_Axsis);
				 Timer_Enable_Z = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;							 
			}		   	  
	}
}


/**************************************************************************************************
**  函数名：  TIM1_UP_IRQChannel()-L
**	输入参数：无
**	输出参数：无
**	函数功能：定时器1中断函数，L轴脉冲控制函数
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void TIM1_UP_IRQHandler(void)
{	    
    if(TIM_GetITStatus(TIM1,TIM_IT_Update) == SET)
    {    
       TIM_ClearITPendingBit(TIM1,TIM_IT_Update);	 
// 			 NextSpeed[L_Axsis] = 0;
			 switch(Pulse_ControlStep[L_Axsis])
			 {
				  case SPEED_UP:       
					   if( (u32)(Pulse_SpeedControlStep_Counter[L_Axsis]*Pulse_PresentHalfPeriod[L_Axsis]) >= SPEED_CONTROL_TIME )
					   {
						  Present_Pulse_SpeedControlStep_Counter[L_Axsis]=Pulse_SpeedControlStep_Counter[L_Axsis];
							Pulse_SpeedControlStep_Counter[L_Axsis] = 0;   
							SpeedPlanning(L_Axsis);		 //计算加速度20160718	
							NextSpeed[L_Axsis] = Pulse_InitialSpeed[L_Axsis] + Pulse_PresentAcceleration[L_Axsis];  //真是速度值 NextSpeed=fi  ===  Vi  
							if(NextSpeed[L_Axsis]>MAX_SPEED)
							{
								 NextSpeed[L_Axsis]=MAX_SPEED-Pulse_PresentAcceleration[L_Axsis];
							}
							Pulse_PresentHalfPeriod[L_Axsis] = 18000000 / NextSpeed[L_Axsis]; 
							Pulse_InitialSpeed[L_Axsis] = NextSpeed[L_Axsis];
							if(NextSpeed[L_Axsis]>=Pulse_Present_MaxSpeed[L_Axsis])
							{
						     SpeedUp_Pulse_Counter[L_Axsis] = Pulse_Counter[L_Axsis];                              
								 Pulse_ControlStep[L_Axsis] =  SPEED_HOLD;							
							}
					  }
					  if((Pulse_Counter[L_Axsis] + Pulse_Counter[L_Axsis] + Present_Pulse_SpeedControlStep_Counter[L_Axsis]+2) >= Servo_Pulse_Count[L_Axsis])
					  {  
							Pulse_ControlStep[L_Axsis] = SPEED_HOLD; 
							Remain_SpeedControlStep_Counter[L_Axsis]=Pulse_SpeedControlStep_Counter[L_Axsis];
							Pulse_SpeedControlStep_Counter[L_Axsis] = 0;
							SpeedUp_Pulse_Counter[L_Axsis] = Pulse_Counter[L_Axsis];				
					  }
					  break;

				 case SPEED_HOLD:			         
					  if(Pulse_SpeedControlStep_Counter[L_Axsis]>300)
					  {
					  	 Pulse_SpeedControlStep_Counter[L_Axsis]=0;
					  }
					  if(Servo_Pulse_Count[L_Axsis] - Pulse_Counter[L_Axsis] <= (SpeedUp_Pulse_Counter[L_Axsis]-Remain_SpeedControlStep_Counter[L_Axsis]+2) )
					  {				  						  						  
						  Pulse_SpeedControlStep_Counter[L_Axsis] = 0; 
						  Remain_SpeedControlStep_Counter[L_Axsis]=0;
						  //减加速一次进入减速段
						  NextSpeed[L_Axsis]=NextSpeed[L_Axsis]-Pulse_PresentAcceleration[L_Axsis];  
						  Pulse_PresentHalfPeriod[L_Axsis] = 18000000 / NextSpeed[L_Axsis];
						  Pulse_InitialSpeed[L_Axsis] = NextSpeed[L_Axsis];
						  SpeedPlanning(L_Axsis);
						  Pulse_ControlStep[L_Axsis] = SPEED_DOWN; 
					  }
					  break;

				 case SPEED_DOWN:
					  if((u32)(Pulse_SpeedControlStep_Counter[L_Axsis] * Pulse_PresentHalfPeriod[L_Axsis]) >= SPEED_CONTROL_TIME)
					  {			   
							Pulse_SpeedControlStep_Counter[L_Axsis] = 0;
						  SpeedPlanning(L_Axsis);		 //计算加速度20160718					  			
							NextSpeed[L_Axsis] = Pulse_InitialSpeed[L_Axsis] - Pulse_PresentAcceleration[L_Axsis];
							if(NextSpeed[L_Axsis]<Pulse_MinSpeed[L_Axsis])
							{
							   NextSpeed[L_Axsis]=Pulse_MinSpeed[L_Axsis];
							}
							Pulse_PresentHalfPeriod[L_Axsis] = 18000000 / NextSpeed[L_Axsis];
							Pulse_InitialSpeed[L_Axsis] = NextSpeed[L_Axsis];	
					  }
					  break;

				default:
					  break;
			 } 
			 if(Pulse_PresentHalfPeriod[L_Axsis]>18000)
			 {
				  Pulse_PresentHalfPeriod[L_Axsis]=18000;
			 }
			 else if(Pulse_PresentHalfPeriod[L_Axsis]<180)
			 {
				  Pulse_PresentHalfPeriod[L_Axsis]=180;
			 }
			 SET_L_PULSE_PERIOD_HALF(Pulse_PresentHalfPeriod[L_Axsis]);
			 Pulse_SpeedControlStep_Counter[L_Axsis]++;
			 Pulse_Counter[L_Axsis]++;
			
			 //记录Z轴的总脉冲坐标数：
			 if(Axsis_Move_Direction[L_Axsis] == POSITIVE)
			 { 
				  Pulse_TotalCounter[L_Axsis] ++;
			 }
			 else
			 {
				  Pulse_TotalCounter[L_Axsis] --;
			 }
			 if(Error_Enable_Flag == TRUE)
			 {					
				  CLOSE_L_PULSE;  
				  ClosePulseReset(L_Axsis);
				  Timer_Enable_L = DISABLE;
				  Linked_Move_Enable=DISABLE;
				  Jog_Move_Enable= DISABLE;	
			 }		 

			 /*=======================脉冲计数完成，停止脉冲，并清空数据=========================*/
			 if(Pulse_Counter[L_Axsis] == Servo_Pulse_Count[L_Axsis])
			 {
				  CLOSE_L_PULSE; 
				  ClosePulseReset(L_Axsis);
				  Timer_Enable_L = DISABLE;
				  Linked_Move_Enable=DISABLE;
				  Jog_Move_Enable= DISABLE;							 
			 }	   	  
	}
}
/**************************************************************************************************
**  函数名：  TIM4_UP_IRQChannel()-O
**	输入参数：无
**	输出参数：无
**	函数功能：定时器4中断函数，O轴脉冲控制函数
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2016/11/26 
***************************************************************************************************/
void TIM4_IRQHandler(void)
{	    
    if(TIM_GetITStatus(TIM4,TIM_IT_Update) == SET)
    {    
             TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	 
// 			 NextSpeed[O_Axsis] = 0;
			 switch(Pulse_ControlStep[O_Axsis])
			 {
				  case SPEED_UP:       
					   if( (u32)(Pulse_SpeedControlStep_Counter[O_Axsis]*Pulse_PresentHalfPeriod[O_Axsis]) >= SPEED_CONTROL_TIME )
					   {
						    Present_Pulse_SpeedControlStep_Counter[O_Axsis]=Pulse_SpeedControlStep_Counter[O_Axsis];
							Pulse_SpeedControlStep_Counter[O_Axsis] = 0;   
							SpeedPlanning(O_Axsis);		 //计算加速度20160718	
							NextSpeed[O_Axsis] = Pulse_InitialSpeed[O_Axsis] + Pulse_PresentAcceleration[O_Axsis];  //真是速度值 NextSpeed=fi  ===  Vi  
							if(NextSpeed[O_Axsis]>MAX_SPEED)
							{
								 NextSpeed[O_Axsis]=MAX_SPEED-Pulse_PresentAcceleration[O_Axsis];
							}
							Pulse_PresentHalfPeriod[O_Axsis] = 18000000 / NextSpeed[O_Axsis]; 
							Pulse_InitialSpeed[O_Axsis] = NextSpeed[O_Axsis];
							if(NextSpeed[O_Axsis]>=Pulse_Present_MaxSpeed[O_Axsis])
							{
						         SpeedUp_Pulse_Counter[O_Axsis] = Pulse_Counter[O_Axsis];                              
								 Pulse_ControlStep[O_Axsis] =  SPEED_HOLD;							
							}
					  }
					  if((Pulse_Counter[O_Axsis] + Pulse_Counter[O_Axsis] + Present_Pulse_SpeedControlStep_Counter[O_Axsis]+2) >= Servo_Pulse_Count[O_Axsis])
					  {  
							Pulse_ControlStep[O_Axsis] = SPEED_HOLD; 
							Remain_SpeedControlStep_Counter[O_Axsis]=Pulse_SpeedControlStep_Counter[O_Axsis];
							Pulse_SpeedControlStep_Counter[O_Axsis] = 0;
							SpeedUp_Pulse_Counter[O_Axsis] = Pulse_Counter[O_Axsis];				
					  }
					  break;

				 case SPEED_HOLD:			         
					  if(Pulse_SpeedControlStep_Counter[O_Axsis]>300)
					  {
					  	  Pulse_SpeedControlStep_Counter[O_Axsis]=0;
					  }
					  if(Servo_Pulse_Count[O_Axsis] - Pulse_Counter[O_Axsis] <= (SpeedUp_Pulse_Counter[O_Axsis]-Remain_SpeedControlStep_Counter[O_Axsis]+2) )
					  {				  						  						  
						  Pulse_SpeedControlStep_Counter[O_Axsis] = 0; 
						  Remain_SpeedControlStep_Counter[O_Axsis]=0;
						  //减加速一次进入减速段
						  NextSpeed[O_Axsis]=NextSpeed[O_Axsis]-Pulse_PresentAcceleration[O_Axsis];  
						  Pulse_PresentHalfPeriod[O_Axsis] = 18000000 / NextSpeed[O_Axsis];
						  Pulse_InitialSpeed[O_Axsis] = NextSpeed[O_Axsis];
						  SpeedPlanning(O_Axsis);
						  Pulse_ControlStep[O_Axsis] = SPEED_DOWN; 
					  }
					  break;

				 case SPEED_DOWN:
					  if((u32)(Pulse_SpeedControlStep_Counter[O_Axsis] * Pulse_PresentHalfPeriod[O_Axsis]) >= SPEED_CONTROL_TIME)
					  {			   
							Pulse_SpeedControlStep_Counter[O_Axsis] = 0;
						    SpeedPlanning(O_Axsis);		 //计算加速度20160718					  			
							NextSpeed[O_Axsis] = Pulse_InitialSpeed[O_Axsis] - Pulse_PresentAcceleration[O_Axsis];
							if(NextSpeed[O_Axsis]<Pulse_MinSpeed[O_Axsis])
							{
							    NextSpeed[O_Axsis]=Pulse_MinSpeed[O_Axsis];
							}
							Pulse_PresentHalfPeriod[O_Axsis] = 18000000 / NextSpeed[O_Axsis];
							Pulse_InitialSpeed[O_Axsis] = NextSpeed[O_Axsis];	
					  }
					  break;

				default:
					  break;
			 }
			if(Pulse_PresentHalfPeriod[O_Axsis]>18000)
			{
				Pulse_PresentHalfPeriod[O_Axsis]=18000;
			}
			else if(Pulse_PresentHalfPeriod[O_Axsis]<180)
			{
				Pulse_PresentHalfPeriod[O_Axsis]=180;
			}
			SET_O_PULSE_PERIOD_HALF(Pulse_PresentHalfPeriod[O_Axsis]);
			Pulse_SpeedControlStep_Counter[O_Axsis]++;
			Pulse_Counter[O_Axsis]++;
			
			//记录Z轴的总脉冲坐标数：
			if(Axsis_Move_Direction[O_Axsis] == POSITIVE)
			{ 
				Pulse_TotalCounter[3]++;
			}
			else
			{
				Pulse_TotalCounter[3]--;
			}
			if(Error_Enable_Flag == TRUE)
			{					
				 CLOSE_O_PULSE;  
				 ClosePulseReset(O_Axsis);
				 Timer_Enable_O = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;	
			}		 

			/*=======================脉冲计数完成，停止脉冲，并清空数据=========================*/
			if( Pulse_Counter[O_Axsis] == Servo_Pulse_Count[O_Axsis])
			{
				 CLOSE_O_PULSE; 
				 ClosePulseReset(O_Axsis);
				 Timer_Enable_O = DISABLE;
				 Linked_Move_Enable=DISABLE;
				 Jog_Move_Enable= DISABLE;							 
			}	   	  
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/
