/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#ifndef __PARAMETER_H_
#define __PARAMETER_H_

#include "FreeProgramme.h"

/**------------命令--------------**/
#define   ROBOT_ENABLE_A_ORIGIN  0x0A	//机械手使能和回原点
#define   WORK_MODE              0x0B	//机械手工作模式
#define   AUTO_RUN               0x0C	//机械手自动运行
#define   FREE_PROGRAM_SEND      0x0D	//自由编程程序下发
#define   WATCH_COMMAND          0x0E	//监视命令
#define   READ_IIC	             0x0F	//开机读IIC
#define   IO_DEBUG_OUTPUT1       0x1A	//IO调试-输出1
#define   IO_DEBUG_OUTPUT2       0x1B	//IO调试-输出2
#define   IO_DEBUG_OUTPUT3       0x1C	//IO调试-输出3
#define   MANUL_OPERATE          0x1D	//手动调试
#define   PARAMETER_ORDER        0x1E	//参数命令

/**-------0x0A-----使能和回原点--------------**/
#define   ROBOT_ORIGIN   	     0x01	//机械手回原点
#define   X_AXSIS_ORIGIN	     0x02	//X轴回原点
#define   Z_AXSIS_ORIGIN	     0x03	//Z轴回原点
#define   ORIGIN_SPEED	         0x04	//机械手回原点速度
#define   ROBOT_ENABLE	         0x05	//机械手是否使能

/**-------0x0B-----工作模式--------------**/
#define   WAIT_MODE			     0x00   //等待模式
#define   AUTO_MODE              0x01	//自动模式
#define   FREE_PROGRAM           0x02	//自由编程
#define   IO_MODE                0x03	//IO调试
#define   MANUL_MODE             0x04	//手动调试

/**-------0x0C-----自动运行--------------**/
#define   CYCLE_MODE             0x01	//循环模式
#define   SINGLE_MODE            0x02	//单步模式
#define   ACTION_RUN             0x03	//启动
#define   ACTION_PAUSE           0x04	//暂停
#define   ACTION_STOP            0x05	//停止
#define   ACTION_PROGRAM         0x06	//选择运行的程序
#define   ACTION_RESET           0x07	//机械手复位
#define   ACTION_DEBUG           0x08	//机械手调试-自由编程的调试按钮
#define   ACTION_LORIGIN         0x09	//机械手L轴回零

/**-------0x0D-----自由编程--------------**/
#define   PROGRAM_START          0xE1	//程序开始
#define   PROGRAM_INFO           0xE2	//程序信息描述
#define   PROGRAM_CONT           0xE3	//程序内容
#define   PROGRAM_DELETE         0xE4	//程序删除
#define   PROGRAM_FROM_USB_START 0xE5	//USB程序拷贝发送开始
#define   PROGRAM_FROM_USB_END   0xE6	//USB程序拷贝发送结束
#define   PROGRAM_END            0xED	//程序结束

#define   P_PROGRAM_RENAME				 0xE7 //重命名
#define   P_PROGRAM_CHOOSE         0xE8	//产品选择
#define   P_PROGRAM_DELETE         0xE9	//产品删除
#define   P_PROGRAM_RENAME_COVER   0xEA //产品重命名覆盖
#define   P_PROGRAM_NEW						 0xEB //产品新建
#define   P_PROGRAM_NEWNAME_COVER	 0xEC //新建同名覆盖
#define   P_PROGRAM_RECEIVE				 0xED //接收主控板程序

/**-------0x0E-----监视命令--------------**/
#define   IO_DEBUG_LOCAL         0x01	//IO调试-本地IO
#define   IO_DEBUG_INPUT1        0x02	//IO调试-输入1
#define   IO_DEBUG_INPUT2        0x03	//IO调试-输入2
#define   IO_DEBUG_INPUT3        0x04	//IO调试-输入3
#define   IO_DEBUG_OUTPUT1_LCD   0x05	//IO调试-输出1界面
#define   IO_DEBUG_OUTPUT2_LCD   0x06	//IO调试-输出2界面
#define   IO_DEBUG_OUTPUT3_LCD   0x07	//IO调试-输出3界面
#define   ROBOT_ORIGINED		 0x08	//机械手是否已回原点
#define   ACTION_RESET_SCAN	     0x09	//机械手复位状态查询
#define   ALARM_CLEAR	         0x0A	//清除当前报警信息
#define   ROBOT_STATUS      	 0x0B	//查询机械手状态
#define   AUTO_PARAMETER         0x0C	//自动运行参数-当前行-产量
#define   X_AXSIS_POSITION	     0x0D	//X轴坐标
#define   Z_AXSIS_POSITION       0x0E	//Z轴坐标
#define   ROBOT_PRE_STATUS       0x0F	//机械手运行状态
#define   ROBOT_DEBUG_STATUS     0x10	//机械手调试状态
#define   L_AXSIS_POSITION       0x11	//L轴坐标
#define   O_AXSIS_POSITION       0x12	//O轴坐标
#define   A_AXSIS_POSITION       0x13	//V轴坐标
#define   B_AXSIS_POSITION       0x14	//W轴坐标

/**-------0x0F-----开机读IIC--------------**/
#define   READ_IIC_1	         0x01	//
#define   READ_IIC_2	         0x02	//
#define   READ_IIC_3	         0x03	//
#define   READ_IIC_4	         0x04	//
#define   READ_IIC_5	         0x05	//
#define   READ_IIC_6	         0x06	//
#define   READ_IIC_7	         0x07	//
///**-------0x1A-----IO调试-输出1--------------**/
//#define   STORAGE_LOAD_FINISH    0x01	//料仓-上料完成
//#define   STORAGE_UNLOAD_FINISH  0x02	//料仓-卸料完成
//#define   STORAGE_LOAD_ALARM     0x03	//料仓-上料仓报警
//#define   STORAGE_UNLOAD_ALARM   0x04	//料仓-卸料仓报警
//#define   STORAGE_RESERVE1       0x05	//料仓-预留1
//
//#define   MACHINE1_KAPA_OPEN     0x06	//机床1卡盘A开
//#define   MACHINE1_KAPA_CLOSE    0x07	//机床1卡盘A关
//#define   MACHINE1_CUIQI         0x08	//机床1吹气
//#define   MACHINE1_ALLOW_START   0x09	//机床1允许启动
//#define   MACHINE1_START_PROCESS 0x0A	//机床1开始加工
//#define   MACHINE1_DOOR_OPEN     0x0B	//机床1顶门开
//#define   MACHINE1_DOOR_CLOSE    0x0C	//机床1顶门关
//#define   MACHINE1_RESERVE1      0x0D	//机床1预留1	 
//#define   MACHINE1_RESERVE2      0x0E	//机床1预留2
//#define   MACHINE1_RESERVE3      0x0F	//机床1预留3

/**-------0x1A-----IO调试-输出1---0x01 Y0--0x0F Y14--------**/
#define   IODEBUG_OUTPUT1_1      0x01	//
#define   IODEBUG_OUTPUT1_2      0x02	//
#define   IODEBUG_OUTPUT1_3      0x03	//
#define   IODEBUG_OUTPUT1_4      0x04	//
#define   IODEBUG_OUTPUT1_5      0x05	//
#define   IODEBUG_OUTPUT1_6      0x06	//
#define   IODEBUG_OUTPUT1_7      0x07	//
#define   IODEBUG_OUTPUT1_8      0x08	//
#define   IODEBUG_OUTPUT1_9      0x09	//
#define   IODEBUG_OUTPUT1_10     0x0A	//
#define   IODEBUG_OUTPUT1_11     0x0B	//
#define   IODEBUG_OUTPUT1_12     0x0C	//
#define   IODEBUG_OUTPUT1_13     0x0D	//	 
#define   IODEBUG_OUTPUT1_14     0x0E	//
#define   IODEBUG_OUTPUT1_15     0x0F	//


/**-------0x1B-----IO调试-输出2--- 0X01 Y15---0X0F Y29--------**/
#define   IODEBUG_OUTPUT2_1      0x01	//
#define   IODEBUG_OUTPUT2_2      0x02	//
#define   IODEBUG_OUTPUT2_3      0x03	//
#define   IODEBUG_OUTPUT2_4      0x04	//
#define   IODEBUG_OUTPUT2_5      0x05	//
#define   IODEBUG_OUTPUT2_6      0x06	//
#define   IODEBUG_OUTPUT2_7      0x07	//
#define   IODEBUG_OUTPUT2_8      0x08	//
#define   IODEBUG_OUTPUT2_9      0x09	//
#define   IODEBUG_OUTPUT2_10     0x0A	//
#define   IODEBUG_OUTPUT2_11     0x0B	//
#define   IODEBUG_OUTPUT2_12     0x0C	//
#define   IODEBUG_OUTPUT2_13     0x0D	//	 
#define   IODEBUG_OUTPUT2_14     0x0E	//
#define   IODEBUG_OUTPUT2_15     0x0F	//

/**-------0x1C-----IO调试-输出3--- 0X01 Y30---0X0F Y35--------**/
#define   IODEBUG_OUTPUT3_1      0x01	//
#define   IODEBUG_OUTPUT3_2      0x02	//
#define   IODEBUG_OUTPUT3_3      0x03	//
#define   IODEBUG_OUTPUT3_4      0x04	//
#define   IODEBUG_OUTPUT3_5      0x05	//
#define   IODEBUG_OUTPUT3_6      0x06	//



/**-------0x1B-----IO调试-输出2--------------**/
#define   MACHINE2_KAPA_OPEN     0x01	//机床1卡盘A开
#define   MACHINE2_KAPA_CLOSE    0x02	//机床1卡盘A关
#define   MACHINE2_CUIQI         0x03	//机床1吹气
#define   MACHINE2_ALLOW_START   0x04	//机床1允许启动
#define   MACHINE2_START_PROCESS 0x05	//机床1开始加工
#define   MACHINE2_DOOR_OPEN     0x06	//机床1顶门开
#define   MACHINE2_DOOR_CLOSE    0x07	//机床1顶门关
#define   MACHINE2_RESERVE1      0x08	//机床1预留1
#define   MACHINE2_RESERVE2      0x09	//机床1预留2
#define   MACHINE2_RESERVE3      0x0A	//机床1预留3

#define   ROLL_OVER_UPRIGHT      0x0B	//翻转台-正摆
#define   ROLL_OVER_REVERSE      0x0C	//翻转台-反摆
#define   ROLL_OVER_OPEN         0x0D	//翻转台-松开
#define   ROLL_OVER_CLOSE        0x0E	//翻转台-夹紧
#define   ROLL_OVER_RESERVE1     0x10	//翻转台-预留1

/**-------0x1C-----IO调试-输出3--------------**/
#define   CLAW_LOAD_OPEN         0x01	//料爪-上料爪松
#define   CLAW_LOAD_CLOSE        0x02	//料爪-上料爪紧
#define   CLAW_UNLOAD_OPEN       0x03	//料爪-卸料爪松
#define   CLAW_UNLOAD_CLOSE      0x04	//料爪-卸料爪紧
#define   CLAW_BAIGANG_UPRIGHT   0x05	//料爪-摆缸正摆
#define   CLAW_BAIGANG_RESET     0x06	//料爪-摆缸复位
#define   CLAW_YCYLINDER_STRETCH 0x07	//料爪-Y向气缸伸出
#define   CLAW_YCYLINDER_RETRACT 0x08	//料爪-Y向气缸缩回
#define   CLAW_BLOW              0x09	//料爪-手抓吹气
#define   CLAW_RESERVE1          0x0A	//料爪-预留1

#define   OTHERS_OILING_ENABLE   0x0B	//其他-润滑使能
#define   OTHERS_RUN_LIGHT       0x0C	//其他-运行灯
#define   OTHERS_WAIT_LIGHT      0x0D	//其他-待机灯
#define   OTHERS_ALARM_LIGHT     0x0E	//其他-报警灯
#define   OTHERS_RESERVE1        0x10	//其他-预留1


/**-------0x1D-----手动调试-----------------**/
#define	  XAXIS_MANUL_SPEED		 0x01	//X轴-手动速度值
#define   XAXIS_MOVE_LEFT        0x02	//X轴-左移
#define   XAXIS_MOVE_RIGHT       0x03	//X轴-右移
#define   XAXIS_STEP_STOP        0x04	//X轴-寸动停止
#define   XAXIS_SAVE_POINT       0x05	//X轴-保存
#define	  ZAXIS_MANUL_SPEED		 0x06	//Z轴-手动速度值
#define   ZAXIS_MOVE_LEFT        0x07	//Z轴-左移
#define   ZAXIS_MOVE_RIGHT       0x08	//Z轴-右移
#define   ZAXIS_STEP_STOP        0x09	//Z轴-寸动停止
#define   ZAXIS_SAVE_POINT       0x0A	//Z轴-保存
#define   XZAXIS_SAVE_POINT      0x0B	//XZ轴-保存
#define   XZAXIS_DELETE_POINT   0x0C	//XZ轴-删除
#define	  XAXIS_STEP_MM		     0x0D	//X轴-寸动距离mm为单位
#define	  ZAXIS_STEP_MM		     0x0E	//Z轴-寸动距离mm为单位
#define	  XAXIS_BACK_ORIGIN		 0x0F	//X轴-回零
#define	  ZAXIS_BACK_ORIGIN		 0x10	//Z轴-回零
#define   MANUL_FINE_ADJUST		 0x11	//微调命令

#define	  LAXIS_MANUL_SPEED		 0x12	//L轴-手动速度值
#define   LAXIS_MOVE_LEFT        0x13	//L轴-左移
#define   LAXIS_MOVE_RIGHT       0x14	//L轴-右移
#define   LAXIS_STEP_STOP        0x15	//L轴-寸动停止
#define	  LAXIS_STEP_MM		     0x16	//L轴-寸动距离mm为单位
#define   LAXIS_SAVE_POINT       0x17	//L轴-保存
#define	  LAXIS_BACK_ORIGIN		 0x18	//L轴-回零

#define	  OAXIS_MANUL_SPEED		 0x19	//O轴-手动速度值
#define   OAXIS_MOVE_LEFT        0x1A	//O轴-左移
#define   OAXIS_MOVE_RIGHT       0x1B	//O轴-右移
#define   OAXIS_STEP_STOP        0x1C	//O轴-寸动停止
#define	  OAXIS_STEP_MM		     0x1D	//O轴-寸动距离mm为单位
#define   OAXIS_SAVE_POINT       0x1E	//O轴-保存
#define	  OAXIS_BACK_ORIGIN		 0x1F	//O轴-回零
#define   LOAXIS_DELETE_POINT   0x20	//LO轴-删除

#define	  AAXIS_MANUL_SPEED		 0x21	//V轴-手动速度值
#define   AAXIS_MOVE_LEFT        0x22	//V轴-左移
#define   AAXIS_MOVE_RIGHT       0x23	//V轴-右移
#define   VAXIS_STEP_STOP        0x24	//V轴-寸动停止
#define	  VAXIS_STEP_MM		     0x25	//V轴-寸动距离mm为单位
#define   VAXIS_SAVE_POINT       0x26	//V轴-保存
#define	  VAXIS_BACK_ORIGIN		 0x27	//V轴-回零

#define	  WAXIS_MANUL_SPEED		 0x28	//W轴-手动速度值
#define   BAXIS_MOVE_LEFT        0x29	//W轴-左移
#define   BAXIS_MOVE_RIGHT       0x2A	//W轴-右移
#define   WAXIS_STEP_STOP        0x2B	//W轴-寸动停止
#define	  WAXIS_STEP_MM		     0x2C	//W轴-寸动距离mm为单位
#define   WAXIS_SAVE_POINT       0x2D	//W轴-保存
#define	  WAXIS_BACK_ORIGIN		 0x2E	//W轴-回零

#define   E_GEARRATIO_SET			  0x2F//电子齿轮比设置
#define   XYAXIS_MANUL_SPEED    0x30//示教界面XY联动速度设置
#define   ZOAXIS_MANUL_SPEED    0x31//示教界面ZO联动速度设置
#define   ABAXIS_MANUL_SPEED    0x32//示教界面AB联动速度设置
#define   XYAXIS_STEP_MM				0x33//示教界面XY寸动速度设置
#define   ZOAXIS_STEP_MM				0x34//示教界面ZO寸动速度设置
#define   ABAXIS_STEP_MM				0x35//示教界面AB寸动速度设置
#define   P_ACTION_SPEEDADJ      0x36	//遥杆速度调节20181205-L

/**-------0x1E-----参数命令-----------------**/
#define	  PARAMETER_SOFT_LIMIT		   0x01	//软限位
#define   PARAMETER_SAFE_AREA        0x02	//安全区
#define   PARAMETER_FUCTION_SET      0x03	//功能参数
#define   PARAMETER_IO_DETECT_SET    0x04	//功能参数


#define   ALARMHISTORY_CLEAR         0x07	//历史报警清除


#define   JOG_MOVE_MODE          0x01	//寸动模式
#define   LINK_MOVE_MODE         0x02	//连动模式

#define   NO_AXIS    0x00
#define   X_AXIS     0x01
#define   Z_AXIS     0x02
#define   L_AXIS     0x03
#define   O_AXIS     0x04
#define   V_AXIS     0x05
#define   W_AXIS     0x06

#define CURRENT_PROGRAM_ADR1  0x1200//屏幕上处理时保存的程序地址1
#define CURRENT_PROGRAM_ADR2  0x7000//屏幕上处理时保存的程序地址2

#define ORDER_LENTH  30       //指令长度字节

#define LARGESAVEORDERNUM  32//存储程序的数目上限(数组数据错乱)
#define SAVEPROGRAMNUM  104	 //存储程序的数目上限
#define SAVEPROGRAMNUM_2  8	 //存储程序的数目上限-料仓程序
#define SAVEPROGRAMNUM_4  6	 //存储程序的数目上限-单次、复位、料仓程序
#define LC_MAXNUM       10    //料仓数目上限



#define X_Axsis				  0x00	   //X轴
#define Z_Axsis				  0x01	   //Z轴
#define L_Axsis				  0x02	   //L轴
#define O_Axsis				  0x03	   //O轴
#define A_Axsis				  0x04	   //V轴
#define B_Axsis				  0x05	   //W轴


extern u8  X_Axsis_Origin_Enable ;	 //X轴回原点标识位
extern u8  Z_Axsis_Origin_Enable ;	 //Z轴回原点标识位
extern u8  Robot_Origining;
//extern u8  X_Axsis_Origining;
//extern u8  Z_Axsis_Origining ;
extern u8  X_Axsis_Origned;	   //X轴已回原点
extern u8  Z_Axsis_Origned;	   //Z轴已回原点
extern u8  Robot_Origined;	   //机械手回原点
extern u8  Robot_Enable;
extern u8  Robot_Enable_Switch_On;	  
extern u8  Robot_Enable_Pre_Switch;
extern u8  Program_Ready;
extern u8  Data_Syc_Ready;
extern u8  ProgramInf_Syc_Ready;
extern u8  Program_Syc_Ready;
extern u8  SoftLimit_Syc_Ready;
extern u8  SafeArea_Syc_Ready;
extern u8  Point_Syc_Ready;
extern u8  Setting_Syc_Ready;
extern u8  IODetect_Syc_Ready;
extern u8  Main_Init_Done;		 //主控板开机完成
extern u8  Robot_Status ;      //机械手状态
extern u8  Robot_Auto_Mode;

extern u32 g_XAxis_Position ;    //X轴的位置
extern u32 g_ZAxis_Position ;		 //Z轴的位置
extern u32 g_LAxis_Position ;		 //L轴的位置
extern u32 g_OAxis_Position ;		 //O轴的位置
extern u32 g_AAxis_Position ;		 //V轴的位置
extern u32 g_BAxis_Position ;		 //W轴的位置
extern u32 g_Pre_XAxis_Position;   //X轴之前时刻的位置
extern u32 g_Pre_ZAxis_Position ;	 //Z轴之前时刻的位置
extern u32 g_Pre_LAxis_Position ;	 //L轴之前时刻的位置
extern u32 g_Pre_OAxis_Position ;	 //O轴之前时刻的位置
extern u32 g_Pre_VAxis_Position ;	 //V轴之前时刻的位置
extern u32 g_Pre_WAxis_Position ;	 //W轴之前时刻的位置
extern u8 Axsis_Move_Direction[6];//轴运动方向
extern u8  g_Servo_Is_Run,g_Axis_Is_Run;
extern u8  g_Axis_Choose ;
//extern u16 g_Robot_Error;
extern u8  Robot_Error_Data[5],Robot_Axis_State[6];
extern u8 g_Robot_Error_Flag;

//点存储
typedef struct 
{
	u8  Flag;		//是否有点
	u32 Name;		//点的名称
	u32 Point;		//X的值
}SavePoint;

//程序信息存储
typedef struct 
{
	u8  Flag;	   //是否有程序
	u8  Code;	   //程序的编号，下发给主控制板是识别用
	u32 Name;	   //N
	u16  Num;	   //S:Scale保存程序的规模大小，用于运行时显示判断
	u16 Address;   //IIC的存储地址
}SaveProgramInf;   //7

//程序存储
typedef struct 
{
	u8  Flag;	   //是否有程序
	u8  Code;	   //程序的编号，下发给主控制板是识别用
	u32 Name;	   //N
	u16  Num;	   //S:Scale保存程序的规模大小，用于运行时显示判断
    FreeProgram	Program[LARGESTPROGRAMNUM];
}SaveProgram;	   //7+1500

/***************保存程序信息*********************************/
typedef struct 
{
	u8  Flag;	   //是否有程序
	u8  Code;	   //程序的编号，下发给主控制板是识别用
	u32 Name;	   //N
	u16  Order_Num;//指令数目
	u32 Time;			//时间
}SavedProgramInf;   //13

typedef struct
{
	u8  Flag;	   //是否有程序 1有 0无
	u8  Code;	   //程序的编号，下发给主控制板是识别用
	u16 XuHao;
	u32 Name;	   //名字
	u16  Order_Num;//指令数目
	u32 time;      //日期
}Save_Program;	   
extern Save_Program OperateProgram[SAVEPROGRAMNUM];
extern SavePoint Manul_Save_PointX[25];
extern SavePoint Manul_Save_PointZ[25];
extern SavePoint Manul_Save_PointL[25];
extern SavePoint Manul_Save_PointO[25];
extern SaveProgramInf Free_Program_Save[10];
extern SaveProgram Current_Operate_Program;
extern SaveProgram Free_Program_Operate,Free_Program_Operate;
void USART1_ReceiveProgram(u16 Num,u8 Code);	    //发送程序函数
u8 USART1_ReadProgram(u8);


extern u8  UnLock_Forever_flag;	//是否长期有效标志位
extern u16 Unlock_LastDay;		//解密时间
extern u8  Reset_Flag;
extern u32 MachineQueueNum;		//由时间和机器码生成的机器编码
extern u8  Check_battery;

extern SavedProgramInf Saved_Program[];


void InitLockTime(void);
void CheckTime(void);
void UnlockCodeInput(void);

void ReadIICInitPar(void);
void ReadIICAlarm(void);
void LinkMoveDataInit(void);
void Read_Point(void);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

