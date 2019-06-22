/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __auto_h_
#define __auto_h_


#define ONCE_MODE   0x01	  //单次模式
#define SINGLE_MODE 0x02    //单步模式
#define LOOP_MODE   0x03	  //循环模式
#define AUTO_START  0x04	  //启动
#define AUTO_STOP   0x05	  //停止

#define RUN    0x00	   //运行
#define PAUSE  0x01	   //暂停
#define STOP   0x02    //停止


//#define INTERNAL_POSITION  5000 //料仓上取料和摆放成品间隔距离

extern u32 Auto_Pulse_Count;
extern u8 Auto_Run_Enable ;		  //自动模式使能标志位
extern u8 Auto_Mode;			  //自动模式下的模式选择
extern u8 Stop_Time;
extern u8 Stop_Status;
extern u8 Single_Mode_Enable;
extern u8 Once_Mode_Enable;
extern u8 Loop_Mode_Enable;
extern u8 Section_Action_Complete;
extern u8 JHRobot_Section;
extern u8 Last_JHRobot_Section;
extern u32 JHRobot_Section_Time;
extern u8 JHRobot_Section_Delay;
extern u8 Section_Step;
extern u8 Manchine_Door_Flag;
extern u8 JHRobot_Warning;
extern u32 Max_Robot_Position;
extern u32 Min_Robot_Position;


extern u32 Action_Delay_Time;         //机械手每次动作完成之后的延时，因为伺服器动作是跟随动作
extern u8  Puls_Delay_Time[50];		 //检测输入信号延时
extern u8  Puls_Delay_Enable[50];	     //检测输入延时使能标志位
extern u8  Puls_Delay_Num;
extern u8  Action_Done_Flag ;		 //伺服器动作完成标志
extern u8  Action_Delay_Flag;		 //动作延时完成
extern u8  Auto_Action_Delay;	 //卸料结束最后次动作延时
extern u32 Interval_Position;
extern u8  Load_Material_Row_Number;
extern u32 Total_Product_Number;
extern u32 Product_Number;		  //产品加工数量
extern u32 Check_Number;			  //产品检测数量
extern u32 Test_Pulse_Xaxsis;	 //测试X轴丢脉冲
extern u8  Test_Pulse_Xaxsis_Flag;

/* 产品取原料和放成品位置 */
extern u32 Product_Position[10];
//extern u32 Interval_Position;	
extern u8  Holder_Delay_Flag;
extern u16 Holder_Delay_Time;		      //夹具延时
extern u8  Holder_Delay_Done;		  //夹具延时完成标志位
extern u16 Holder_Delay;				  //夹具设定延时时间

extern u8  Claw_Delay_Flag;		  //夹具延时标志位
extern u16 Claw_Delay_Time;		      //夹具延时
extern u8  Claw_Delay_Done;		  //夹具延时完成标志位
extern u16 Claw_Delay;				  //夹具设定延时时间

//输出脉冲信号延时参数
extern u16 Output_Delay_Time;		      //输出脉冲延时

extern u8 g_Robot_Has_Debuged_Flag;
extern u8 g_Program_Is_Debuging;


void Read_SaveProgram_IIC_Address(void);
void AutoRun(void);
extern void AutoMode(void);
extern void AutoRunning(void);
extern void AutoReset(void);
extern void AutoStart(void);
extern void AutoPause(void);
extern void AutoStop(void);
extern void SafeAreaJudge(void);
extern void SetSingle(u8,u8,u32);
void MoveToTarget(u32,u8,u32);
void RobotAutoReset(void);

#endif


/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/


