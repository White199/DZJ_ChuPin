/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#ifndef __statuscontrol_h_
#define __statuscontrol_h_

#define WAIT_MODE			  0x00     //等待模式
#define AUTO_WORK_MODE        0x01	   //选择自动工作模式模式
#define MANUAL_WORK_MODE      0x02	   //选择手动工作模式模式
#define FREE_PROGRAM_MODE	  0x03     //选择自由编程模式
#define IO_DEBUG_MODE		  0x04	   //选择IO调试模式


/**----------------电机移动方向------------------**/
#define NEGATIVE              0x00	   //反向
#define	POSITIVE              0x01	   //正向
#define NONE                  0x02	   //

#define X_Axsis				  0x00	   //X轴
#define Z_Axsis				  0x01	   //Z轴
#define L_Axsis				  0x02	   //L轴
#define O_Axsis				  0x03	   //O轴
#define A_Axsis				  0x04	   //V轴
#define B_Axsis				  0x05	   //W轴
//回原点方式
//#define Z_X_TYPE			  0x00	  //Z_X
//#define Z_X_L_TYPE			  0x01	  //Z_X_L
//回原点方式
#define FOM_Z_X     0
#define FOM_X_Z     1
#define FOM_O_X     2
#define FOM_Z_X_L   3
#define FOM_Z_L_X   4
#define FOM_Z_L_X_O 5
#define FOM_Z_X_L_O 6

//加速度等级
#define FA_RANK1    0
#define FA_RANK2    1
#define FA_RANK3    2

extern u8  Synchronize;				  //与手控器同步标志位
extern u8  Origin_Backed;			  //回原点完成
extern u8  X_Origin_Backed; 		  //X轴达到原点标志位
extern u8  Z_Origin_Backed;   		  //Z轴达到原点标志位
extern u8  L_Origin_Backed;   		  //L轴达到原点标志位
extern u8  O_Origin_Backed;   		  //O轴达到原点标志位
extern u8  Back_Origin_Flag; 		  //回原点命令标志位
extern u8  Origin_Speed_Down;         //回原点检测到信号后，状态为减速状态标志位
extern u8  Initialize_Finished;		  //初始化完成
extern u32 Back_Origin_Speed;		  //回原点速度
extern u8  Robot_Enable_Flag;		  //机械手使能
//extern u8  Origin_Type;				  //回原点方式

//extern u8 Servo_Is_Run ;			  //伺服器正在运行标志位
extern u8 Work_Status ;				  //工作状态
extern u32 Input_Detect_Time;
extern u8 Input_Detect_Enable;
extern u8 Action_Detect_Flag;
extern u8 Jog_Pause_Enable; 
extern u8 Position_Handle_Flag;
//extern u8 Linked_Running_FLAG ;     //连动运行模式
extern u8 SafeArea_Is_On;
extern u8 X_In_SafeArea_Flag;
extern u8 Safe_Area_Flag;
extern u8 Safe_Area_Alarm;		      //安全区域报警，超出安全区
extern u8 g_Current_SafeArea_Num;	  //安全区编号
extern u8 Input15_16_Detect;
extern u8 Axis_Manul_BackToOrigin;    //手动回零
extern u8 Axsis_Chosen;				  //运动轴选择
extern u8 Axsis_Move_Direction[6] ;	  //运动轴方向选择
extern u8 Input_Count15;
extern u8 Input_Count16;
extern u8 Servo_Stop_Done[6];
extern u8 Test_ServoStop;
extern u8 Timer_Enable_X;
extern u8 Timer_Enable_Z;
extern u8 Timer_Enable_L;
extern u8 Timer_Enable_O;
extern u8 g_Auto_Reset_Flag;
extern u8 Robot_Auto_Reset;
extern u8 g_Auto_LOrigin_Flag;
extern u32 g_USART_Delay_Timer;
void WorkMode(void);
extern void ActionControl(void);	  //动作控制
extern void DirectionJudge(u8);	  //判断机械轴动作方向
extern void StatusControl(void);	  //输入状态检测
extern void Robot_Reset(void);
extern void SafeAreaDetection(void);
extern void Servo_Stop(u8);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

