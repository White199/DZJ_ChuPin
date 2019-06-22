/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __parameter_h_
#define __parameter_h_


/******************参数命令定义***************************/
#define PROGRAMSTARTACTION 1    //是否有开机动作

#define LARGESTPROGRAMNUM 32	//编程最多指令数
#define SAVEPROGRAMNUM 104			//当前保存的程序
#define SAVEPROGRAMNUM_2 8			//除去料仓程序后保存的程序
#define SAVEPROINT 100				//当前保存的点
#define SAVESAFEAREA 3				//当前保存的安全区
#define SAVESOFTLIMIT 6				//当前保存的软限位

#define MINROBOTPOSITION   1000000    //
#define MAXROBOTPOSITION   2000000    //

#define LC_MAX_NUM   10 				//料仓数目上限
#define ORDER_LENTH  30       //指令长度字节

//自由编程
typedef struct 
{
	u8  Flag;  //标志位
	u8  List;  //指令编号
	u8  Order; //指令类型
	u8  Key;   //子命令
	u32 Value1;//参数1-坐标XYZVW
	u32 Value2;//参数2-
	u32 Value3;//参数3-
	u32 Value4;//参数4-
	u32 Value5;//参数5-
	u8 LC_ID;//料仓选择
	u32 IO_Group;//点胶点钻头IO控制
	u16 Delay_Time;//延时时间
	
}FreeProgram;
//extern FreeProgram Current_Process_Program[LARGESTPROGRAMNUM];
//程序存储
typedef struct 
{
	u8  Flag;	   //是否有程序
	u8  Code;	   //程序的编号，下发给主控制板是识别用
	u32 Name;	   //程序名称
	u8  Num;	   //保存程序的规模大小，用于运行时显示判断
    FreeProgram	Program[LARGESTPROGRAMNUM];
}SaveProgram;

//程序存储
typedef struct 
{
	u8  Flag;	   //是否有程序
	u8  Code;	   //程序编号
	u32 Name;	   //程序名称
	u8  Num;	   //程序规模，程序的条数
	u16 Address;   //IIC的存储地址
}SaveProgramIICAddress;






//点存储
//typedef struct 
//{
//	u8  Flag;	   //是否有程序
//	u8  Code;	   //程序规模，程序的条数
//	u16 Address;   //IIC的存储地址
//}SavePointIICAddress;

extern SaveProgramIICAddress Program_IIC_Address[SAVEPROGRAMNUM];
//extern SavePointIICAddress Point_IIC_Address[SAVEPROINT];
//extern SaveProgram Free_Program_Save[SAVEPROGRAMNUM];
extern SaveProgram Free_Program_Operate;
extern SaveProgram LC_Load_Program_Operate;
extern SaveProgram LC_UnLoad_Program_Operate;
extern SaveProgram Start_Action_Program_Operate;
extern SaveProgram Reset_Action_Program_Operate;

extern u8 Start_Recieve_Program_Flag;
extern u8 Current_Delete_Program;	        //当前删除程序


//-----------------------------------------------------------//
/***************保存程序信息*********************************/
typedef struct 
{
	u8  Flag;	          //是否有程序
	u8  Code;	          //程序的编号，下发给主控制板是识别用
	u32 Name;	          //N
	u32 Address_Order;  //指令存储地址
	u16  Order_Num;			//指令数目
	u32 Time;					  //时间
}SavedProgramInf;   //13
//LC点示教存储
typedef struct 
{
	u32 XPoint;	
	u32 YPoint;
}LCPosition;
//料仓表
typedef struct 
{
	LCPosition LC_Load_Position[22][20];
}LCTable;




typedef struct 
{
	u32 Left_Limit  ; //-
	u32 Right_Limit ; //+
	u8  Switch_Limit; //开关
}SoftLimit;

typedef struct 
{
	u32 X_Left ;
	u32 Z_Up ;
	u32 X_Right ;
	u32 Z_Down ;
	u8  SafeArea_Switch;
}SafeAreas;


//点存储
typedef struct 
{
	u8  Code;		//编号
	u8  Flag;
	u32 Name;		//点的名称
	u32 Point;		//X的值
}SavePoint;

typedef struct //--6.29
{
	u8  Type[4];
	u16 Time[4];
}ACC;  //Accelerate

typedef struct 
{
	u8  Axis;
	u8  Origin[6];
// 	u16  Accelerate ;  6.29 注释
	ACC Accelerate;  //--6.29
	u8  Auto_SpeedLevel;//自动运行最大速度
	u8  Manual_SpeedLevel;//手动示教最大速度
	u8  Manual_Prescaler;
	u16 SPEED_CONTROL_TIME_MANUAL[3];         //18000每隔2ms()加一次脉冲，如果每个脉冲周期大于2ms，则每个脉冲周期都加
	u32 OriginSpeed[6];
	u8  OriginPosition[6];
	u8  OriginDir[6];
	u32 PulseTime;
	u16 GearRatio[6];//X Z L O A B
	u8  ServoAlarm;
	//低四位代表数控加工输入IO，0001——>X32,0010——>X33,0100——>X34,1000——>X35默认0（未配置）
	//高四位代表等待时间（min）0000表示不检测超时永久等待，0001-1111表示1-16min
	u8	NcComplete;
	u8  ManualSpeed_H[3];//XYZO  A  B  
	u8  ManualSpeed_L[3];//XYZO  A  B  
	u8  AutoSpeed_H[3];//XYZO  A  B  
	u8  AutoSpeed_L[3];//XYZO  A  B  
	u8  Acceleration_UP[3];//XYZO  A  B  
	u8  Acceleration_DOWN[3];//XYZO  A  B  
}JXSParameter;





typedef struct 
{
	u8  Switch ;
	u8  Layer ;
	u32 O_PositionS;
	u32 O_PositionE;
	u8  H_Num ;	
	u8  V_Num ;	
	LCPosition Position1;
	LCPosition Position2;
	LCPosition Position3;
	
	u8  ID;
	u32 LC_Hight;
	u32 HT_Hight;
	u16 Glue_Time;
	u16 Diamonds_Time;
}LCParameter;

typedef struct 
{
	u32  RW_Num ;	//任务计划
	u32  SC_Num ;	//生产
	u32  CJ_Num ;	//抽检
	u32  JG_Num ;	//间隔
	u32  LJ_Num ;	//累计
}SCParameter;





extern JXSParameter JXS_Parameter;						  //料仓参数
extern LCPosition LC_Load_Position[20][20];	              //计算存储的位置表
extern LCPosition LC_UnLoad_Position[20][20];	          //计算存储的位置表
extern u32 LC_Load_OAxis_Position[50] ;			          //计算垂直轴位置表
extern u32 LC_UnLoad_OAxis_Position[50] ;		          //计算垂直轴位置表
extern LCParameter LC_Parameter_Load[];					  //料仓参数
//extern LCParameter LC_Parameter_UnLoad;					  //料仓参数
extern SCParameter SC_Parameter ;						  //生产参数
extern SoftLimit  Robot_SoftLimit[4];		              //XZ软限位参数
extern SafeAreas Robot_Safe_Area[SAVESAFEAREA];		      //安全区域参数
extern u8 LC_Load_Pre_ListX;
extern u8 LC_Load_Pre_ListY;
extern u8 LC_Load_Pre_ListO_Get;
extern u8 LC_Load_Pre_ListO_Place;
extern u8 LC_UnLoad_Pre_ListX;
extern u8 LC_UnLoad_Pre_ListY;
extern u8 LC_UnLoad_Pre_ListO;
extern u8 LC_UnLoad_Pre_ListZ;
extern u8 LC_UnLoad_ZAxis_Num;
extern u8 g_DD_ResetAction;

extern SavePoint Manul_Save_PointX;
extern SavePoint Manul_Save_PointZ;
extern SavePoint Manul_Save_PointL;
extern SavePoint Manul_Save_PointO;
extern u8 Current_Delete_XZPoint;
extern u8 Current_Delete_LOPoint;
extern u8 g_Run_Program_Num ;
extern u16 SaveProgram_IIC_Address;
extern u8  SaveProgram_IIC_Num ;
extern u8 g_LDOperation_Pause;


//------------------------------------//
extern LCTable LC_Load_Table[LC_MAX_NUM];

//extern  ManualOperate Manual_Operate;
//extern  ProductCount  Product_Count;
//extern  ParmeterTime  Parameter_Time;

//extern AxsisSet Machine_Wait_Position;   //待机点
//extern AxsisSet Load_Wait_Position;      //取料待机点
//extern AxsisSet Load_Material_Position;  //取料点
//extern AxsisSet Unload_Wait_Position;    //放料等待点
//extern AxsisSet Unload_Material_Position;//放料点
//extern AxsisSet Product_Place_Position;  //成品摆放点
//extern AxsisSet Product_Check_Position;  //检测点
//extern SafeArea Material_Safe_Area;
//extern SafeArea Machine_Safe_Area;
extern u32 X_Axsis_Maxlength;
extern u32 Z_Axsis_Maxlength;
extern u32 L_Axsis_Maxlength;
extern u32 O_Axsis_Maxlength;
extern u32 A_Axsis_Maxlength;
extern u32 B_Axsis_Maxlength;
extern u32 X_Axsis_Minlength;						
extern u32 Z_Axsis_Minlength;
extern u32 L_Axsis_Minlength;
extern u32 O_Axsis_Minlength;
extern u32 A_Axsis_Minlength;
extern u32 B_Axsis_Minlength;
//---0821
extern u32 X_Axsis_Minlength_Can ;
extern u32 Z_Axsis_Minlength_Can ;
extern u32 L_Axsis_Minlength_Can ;
extern u32 O_Axsis_Minlength_Can ;
extern u32 A_Axsis_Minlength_Can ;
extern u32 B_Axsis_Minlength_Can ;
extern u32 X_Axsis_Maxlength_Can ;
extern u32 Z_Axsis_Maxlength_Can ;
extern u32 L_Axsis_Maxlength_Can ;
extern u32 O_Axsis_Maxlength_Can ;		    
extern u32 A_Axsis_Maxlength_Can ;
extern u32 B_Axsis_Maxlength_Can ;


//extern u8 SoftLimit_Chang_Done;				
extern u32 Robot_Soft_Limit[2][2];

extern u8  PVD_data[4];
extern u8  IIC_Parameter[100];



extern SavedProgramInf Saved_Program[];
extern SaveProgram Free_Program_Operate1;



void ParameterOrder(void);
void FreeProgramSend(void);
void ReadIICData(void);
void ReadIICSysParameter(void);
void ReadIIC_Program(void);
void ReadIIC_One_Program(void);
void Read_LC_List(u8);
void Calculate_LC_Load_Parameter(u8 m);
void Calculate_LC_UnLoad_Parameter(void);
void Calculate_LC_Load_OAxis_Parameter(void);
void Calculate_LC_UnLoad_OAxis_Parameter(void);

//-----------------------------//
void Program_Send(void);

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

