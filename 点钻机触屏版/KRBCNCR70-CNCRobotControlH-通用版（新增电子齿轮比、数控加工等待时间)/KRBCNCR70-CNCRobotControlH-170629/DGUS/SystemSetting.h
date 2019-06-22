/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : ***.h
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 05/28/2015
* Description        : This is the ....
***************************************************************************************/

#ifndef __SYSTEMSETTING_H_
#define __SYSTEMSETTING_H_

//轴选择
#define FAN_X_Z_L   0
#define FAN_X_Z_L_A_B 1
#define FAN_X_Z_L_O_A_B 2

//回原点方式
#define FOM_Z_L_X   0
#define FOM_Z_A_B_L_X 1
#define FOM_Z_A_B_L_X_O 2
//加速度等级
#define FA_RANK1    0
#define FA_RANK2    1
#define FA_RANK3    2
//---加速度等级时间，轴的选择6.28
#define FAN_X    0
#define FAN_Z    1
#define FAN_L    2
#define FAN_O    3
#define FAN_A    4
#define FAN_B    5
//---加速度类型 S/T  6.28
#define FAN_S    0
#define FAN_T    1
//---6.29
// #define X_AXIS				  0x00	   //X轴
// #define Z_AXIS				  0x01	   //Z轴
// #define Y_AXIS				  0x02	   //L轴
// #define O_AXIS				  0x03	   //O轴

//---伺服报警信号类型 常闭/常开 12.10
#define FAN_C    0
#define FAN_K    1

//原点方向
#define OR_LEFT     0
#define OR_RIGHT    1

//U盘拷贝程序
#define USBTOCONTROLER 0	  //U盘到控制器
#define CONTROLERTOUSB 1	  //控制器到U盘

#define USBHSTATUS_NOUDISK 		   0
#define USBHSTATUS_UDISKATTACHED   1
#define USBHSTATUS_ISCOPING        2
#define USBHSTATUS_FINISHEDCOPY    3
#define USBHSTATUS_FAILEDCOPY      4
#define USBHSTATUS_UDISKPULLOUT    5
#define USBHSTATUS_UDISKERROR      6
#define USBHSTATUS_UDISKWIRTEPRO   7
#define USBHSTATUS_UDISKNOFILES    8
#define USBHSTATUS_UDISKFAMMAT     9
#define USBHSTATUS_PROGRAMDOWNLOAD 10


/****************TWS****************/
#define Handset     2    //---2018.5.28手持器
#define Controller  3    //---2018.5.28控制器

#define USBSTATUS_NOUDISK         0
#define USBSTATUS_UDISKATTACHED   1
#define USBSTATUS_UDISKPULLOUT    2
#define USBSTATUS_UDISKERROR      3
#define USBSTATUS_UDISKWIRTEPRO   4
#define USBSTATUS_UDISKNOFILES    5
#define USBSTATUS_UDISKFAMMAT     6

/***************TWS*****************/





typedef struct 
{
	u32 Left_Limit  ; //-
	u32 Right_Limit ; //+
	u8  Switch_Limit; //开关
}SoftLimit;

typedef struct 
{
	u32 X_Left ;
	u32 Z_Left ;
	u32 X_Right ;
	u32 Z_Right ;
	u8  SafeArea_Switch;
}SafeArea;

typedef struct //--6.29
{
	u8  Type[6];
	u16 Time[6];
}ACC;  

typedef struct //---2018.3.21
{
	u16  Ratio[6];
}GR; 

typedef struct 
{
	u8  Axis ;
	u8  Origin[6] ;
	ACC Accelerate ;//--- 由8位改成了16位 
	//---加速度中的轴、类型 选 择 6.28
	GR  GearRatio;
	//---运行速度
	u8  SpeedLevel;
	//---回原点速度
	u8  OriginSpeed[6];
	//---手动速度
	u8  ManualSpeed;
	u8  OriginDir[6];//XZYABO
	u8  OriginPosition[6];
	u32 PulseTime;
	u8  ServoAlarm;
	u8  NcComplete;
}JXSParameter;

typedef struct 
{
	u8  ManualSpeed_H[3];//XYZO  A  B  
	u8  ManualSpeed_L[3];//XYZO  A  B  
	u8  AutoSpeed_H[3];//XYZO  A  B  
	u8  AutoSpeed_L[3];//XYZO  A  B  
	u8  Acceleration_UP[3];//XYZO  A  B  
	u8  Acceleration_DOWN[3];//XYZO  A  B  
}DZJSpeed;
//LC点示教存储
typedef struct 
{
	u32 XPoint;	
	u32 YPoint;
}LCPosition;

typedef struct 
{
	u8  ID;
	u8  Switch ;
	u8  H_Num ;	
	u8  V_Num ;	
	u16 Glue_Time;
	u16 Diamonds_Time;
	u32 HT_Hight;
	u32 LC_Hight;
	LCPosition Position1;
	LCPosition Position2;
	LCPosition Position3;
}LCParameter;



typedef struct 
{
	u32  RW_Num ;	//任务计划
	u32  SC_Num ;	//生产
	u32  CJ_Num ;	//抽检
	u32  JG_Num ;	//间隔
	u32  LJ_Num ;	//累计
}SCParameter;

extern JXSParameter JXS_Parameter;
extern LCParameter LC_Parameter_Load[];
extern LCPosition LC_Set_Position[3];
extern LCPosition LC_Load_Position[20][20];
extern LCPosition LC_UnLoad_Position[20][20];
extern u32 LC_Load_OAxis_Position[50];
extern u32 LC_UnLoad_OAxis_Position[50];
extern u8 LC_Load_Pre_PositionX;			
extern u8 LC_Load_Pre_PositionY;			
extern u8 LC_UnLoad_Pre_PositionX;			
extern u8 LC_UnLoad_Pre_PositionY;	
extern u8 LC_UnLoad_Pre_PositionZ;
extern u8 LC_Load_Pre_ListX;			   	   
extern u8 LC_Load_Pre_ListY;			   	   
extern u8 LC_UnLoad_Pre_ListX;			   	   
extern u8 LC_UnLoad_Pre_ListY;
extern u8 LC_UnLoad_Pre_ListZ;	
extern u8 g_Calculate_LC_Load_Parameter_Flag;
extern u8 g_Calculate_LC_UnLoad_Parameter_Flag;	
extern u8 g_LC_Load_Position_Flag;
extern u8 g_LC_UnLoad_Position_Flag;	  	
extern LCParameter LC_Parameter_UnLoad;
extern u8 Set_LCPosition_XYO;         
extern SCParameter SC_Parameter;
extern SoftLimit  Robot_SoftLimit[6];
extern SafeArea Robot_Safe_Area[7];
extern u8  g_Current_Choose_Area ;     //当前选择区域
extern u8  gs_Pre_Choose_Area;
extern u8  gs_Program_Display_Refresh;
extern u8  gs_Point_Display_Refresh;
extern u8  g_FucSet_Axis_Num;
extern u8  g_USART1_DeletePoint_Flag;
extern u8 IO_Detect_Parameter[40];
extern u8 Temp_IO_Detect_Parameter[40];

void LCDSystemSettingProcess(void);
void LCDSystemSettingDisply(void);

void SavePointManager(void);
void SaveProgramManager(void);
void SafeAreaSetting(void);
void FunctionSetting(void);
void IODetectionSetting(void);
void Robot_Init(void);
void Read_LC_List(u8);
void Calculate_LC_Load_Parameter(void);
void Calculate_LC_UnLoad_Parameter(void);
void Calculate_LC_UnLoad_OAxis_Parameter(void);

extern u16 gs_Current_Choose_IO1;
extern u16 gs_Current_Choose_IO2;
extern u16 gs_Current_Choose_IO3;
extern u8  g_IO_DetectAck_Flag;

#endif

