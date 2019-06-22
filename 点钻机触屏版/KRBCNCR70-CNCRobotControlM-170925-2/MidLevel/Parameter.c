/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Parameter.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/25/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Parameter.h"
#include "Usart.h"
#include "Manual.h"
#include "w25qxx.h" 
#include "SpeedControl.h"
#include "StatusControl.h"
#include "Auto.h"
#include "Auto_2.h"
#include "SignalWatch.h"
#include "BackToOrigin.h" 


u32 X_Axsis_Maxlength = MAXROBOTPOSITION;						    //X轴最大软件限位行程
u32 Z_Axsis_Maxlength = MAXROBOTPOSITION;							//Z轴最大软件限位行程
u32 L_Axsis_Maxlength = MAXROBOTPOSITION;							//L轴最大软件限位行程
u32 O_Axsis_Maxlength = MAXROBOTPOSITION;							//L轴最大软件限位行程
u32 A_Axsis_Maxlength = MAXROBOTPOSITION;							//L轴最大软件限位行程
u32 B_Axsis_Maxlength = MAXROBOTPOSITION;							//L轴最大软件限位行程

u32 X_Axsis_Minlength = MINROBOTPOSITION;							//X轴最小软件限位行程
u32 Z_Axsis_Minlength = MINROBOTPOSITION;							//Z轴最小软件限位行程
u32 L_Axsis_Minlength = MINROBOTPOSITION;							//L轴最小软件限位行程
u32 O_Axsis_Minlength = MINROBOTPOSITION;							//L轴最小软件限位行程
u32 A_Axsis_Minlength = MINROBOTPOSITION;							//L轴最小软件限位行程
u32 B_Axsis_Minlength = MINROBOTPOSITION;							//L轴最小软件限位行程
//---0821
u32 X_Axsis_Minlength_Can = 0;
u32 Z_Axsis_Minlength_Can = 0;
u32 L_Axsis_Minlength_Can = 0;
u32 O_Axsis_Minlength_Can = 0;
u32 A_Axsis_Minlength_Can = 0;
u32 B_Axsis_Minlength_Can = 0;
u32 X_Axsis_Maxlength_Can = MINROBOTPOSITION;						    
u32 Z_Axsis_Maxlength_Can = MINROBOTPOSITION;						    
u32 L_Axsis_Maxlength_Can = MINROBOTPOSITION;						    
u32 O_Axsis_Maxlength_Can = MINROBOTPOSITION;						    
u32 A_Axsis_Maxlength_Can = MINROBOTPOSITION;						    
u32 B_Axsis_Maxlength_Can = MINROBOTPOSITION;

//u8 SoftLimit_Chang_Done = FALSE;
SoftLimit  Robot_SoftLimit[4]={0};		        //XZ软限位参数
// Area[0] 表示料仓安全区 Area[1]表示机床1安全区  Area[2]表示机床2安全区
SafeAreas Robot_Safe_Area[SAVESAFEAREA]={0};	//安全区域参数
JXSParameter JXS_Parameter ;		//料仓参数

u8 LC_Load_Pre_ListX=0;
u8 LC_Load_Pre_ListY=0;
u8 LC_Load_Pre_ListO_Get=0;     //钩取料盘指令行
u8 LC_Load_Pre_ListO_Place=0;   //放置料盘指令行
u8 LC_UnLoad_Pre_ListX=0;
u8 LC_UnLoad_Pre_ListY=0;
u8 LC_UnLoad_Pre_ListO=0;
u8 LC_UnLoad_Pre_ListZ=0;
u8 LC_UnLoad_ZAxis_Num=0;
u8 g_DD_ResetAction=FALSE;
LCPosition LC_Load_Position[20][20]={0};	    //计算存储的位置表
LCPosition LC_UnLoad_Position[20][20]={0};	    //计算存储的位置表
u32 LC_Load_OAxis_Position[50]={0};			    //计算垂直轴位置表
u32 LC_UnLoad_OAxis_Position[50]={0};		    //计算垂直轴位置表
//LCParameter LC_Parameter_UnLoad = {0};			//料仓参数
SCParameter SC_Parameter = {0};					//生产参数
u32 Parameter_Temp=0;                           //临时存放接收到的数据
//u8  Usart_Parameter[60]={0};
u8  IIC_Parameter[100]={0};
u8  PVD_data[4]={0};
u8  Temp_Code = 0;		                        //程序编号的临时变量
u8  Temp_Num = 0;		                        //程序编号的临时规模计数
//static u8 gs_Save_Program_Num = 0;            //保存的程序数目
static u8 i=0;

 //程序IIC存储的地址
SaveProgramIICAddress Program_IIC_Address[SAVEPROGRAMNUM]={	  //0x700一个程序
                                    {0,0,0,0,0x4100}, {0,0,0,0,0x4800}, {0,0,0,0,0x4F00}, {0,0,0,0,0x5600},
									{0,0,0,0,0x5D00}, {0,0,0,0,0x6400}, {0,0,0,0,0x6B00}, {0,0,0,0,0x7200},
									{0,9,0,0,0x7900}, {0,10,0,0,0x8000}
								  };  //合计90个字节

u16 SaveProgram_IIC_Address = 0; //记录当前程序要保存的地址
u8  SaveProgram_IIC_Num = 0; //记录当前程序要保存的地址
SaveProgram Free_Program_Operate={0};
SaveProgram LC_Load_Program_Operate={0};
SaveProgram LC_UnLoad_Program_Operate={0};
SaveProgram Start_Action_Program_Operate={0};
SaveProgram Reset_Action_Program_Operate={0};

u8 Program_Data_Temp[1600]={0};
u8 adrees_temp[10]={0};					//写数组IIC缓存
u8 Start_Recieve_Program_Flag = FALSE;  //开始接收程序标志位
u8 Program_From_UDisk_Flag = FALSE;     //程序来自U盘
u8 Current_Delete_Program = 0;	        //当前删除程序
u8 Delete_temp[20]={0};
SavePoint Manul_Save_PointX={0};	  //保存点的数组，Flag、Name、P-X，其中Flag表示是否有点保存
SavePoint Manul_Save_PointZ={0};	  //保存点的数组，Flag、Name、P-Z，其中Flag表示是否有点保存
SavePoint Manul_Save_PointL={0};	  //保存点的数组，Flag、Name、P-Z，其中Flag表示是否有点保存
SavePoint Manul_Save_PointO={0};	  //保存点的数组，Flag、Name、P-Z，其中Flag表示是否有点保存
u8 Current_Delete_XZPoint = 0;	     //当前删除XZ点
u8 Current_Delete_LOPoint = 0;	     //当前删除LO点
u8 g_Run_Program_Num = 0;			 //当前运行的程序编号,0表示无选择程序，-1可做数组下标

u8 Search_Time[6]={0}; //记录搜索次数
u8 Send_Time=0;        //单个程序发送次数
//u8 Count_P=0;
u32 temp_Program_Code=0;

//----------------------------------------------------//
SavedProgramInf Saved_Program[104]={0};
u8 Product_Num=0,Program_Flag[SAVEPROGRAMNUM]={0};
LCTable LC_Load_Table[LC_MAX_NUM]={0};                        // 料仓存储表
SaveProgram Free_Program_Operate1={0};
LCParameter LC_Parameter_Load[LC_MAX_NUM] = {0};			//料仓参数

u8 LC_ID=0;
u16 Pluse_Div[10]={24,23,22,21,20,19,18,17,16,15};//手动调试最小速度
extern  u32 Pulse_Present_MaxSpeed[6];         //100k
extern u16 Confirm_Order_Num;
extern u8 Product_Num,Program_Flag[SAVEPROGRAMNUM],Order_Read_Mode;
extern u16 Confirm_Order_Num,Read_Point_Num,ShuZu_Count;
/**************************************************************************************************
**  函数名：  ParameterTemp()
**	输入参数：无
**	输出参数：无
**	函数功能：在参数命令下，处理接收到的数据
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void ParameterTemp()
{
	Parameter_Temp = 0;
	Parameter_Temp = Parameter_Temp | (UsartReceiveData[3] & 0x000000ff);
	Parameter_Temp = Parameter_Temp | (UsartReceiveData[4] & 0x000000ff)<<8;
	Parameter_Temp = Parameter_Temp | (UsartReceiveData[5] & 0x000000ff)<<16;
	Parameter_Temp = Parameter_Temp | (UsartReceiveData[6] & 0x000000ff)<<24;
}



/**************************************************************************************************
**  函数名：  ParameterOrder()
**	输入参数：无
**	输出参数：无
**	函数功能：参数命令：软限位、安全区、功能设定参数
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2015/11/08 
***************************************************************************************************/
void ParameterOrder()
{
	u8 address1=0;
	u8 address2=0;
	u8 Code=0;
	//--6.29
	u8 i;
	switch(UsartReceiveData[1])
	{
		  /**-- 最大软件限位行程设定 0x2000--**/
		  case P_PARAMETER_SOFT_LIMIT:{
		     if(UsartReceiveData[2]==0x01)                 //X	
			   {
				   Robot_SoftLimit[0].Left_Limit  = (u32)( ((u32)UsartReceiveData[3]) |((u32)UsartReceiveData[4]<<8) |((u32)UsartReceiveData[5]<<16) |((u32)UsartReceiveData[6]<<24) );
				   Robot_SoftLimit[0].Right_Limit = (u32)( ((u32)UsartReceiveData[7]) |((u32)UsartReceiveData[8]<<8) |((u32)UsartReceiveData[9]<<16) |((u32)UsartReceiveData[10]<<24) );
				   Robot_SoftLimit[0].Switch_Limit= UsartReceiveData[11];
				   W25QXX_Write(&UsartReceiveData[3],0x2000,9);			   
				   X_Axsis_Minlength = Robot_SoftLimit[0].Left_Limit*+MINROBOTPOSITION;
				   X_Axsis_Maxlength = Robot_SoftLimit[0].Right_Limit+MINROBOTPOSITION;
			   }
		     else if(UsartReceiveData[2]==0x02)            //Z	
			   {
				   Robot_SoftLimit[1].Left_Limit  = (u32)( ((u32)UsartReceiveData[3]) |((u32)UsartReceiveData[4]<<8) |((u32)UsartReceiveData[5]<<16) |((u32)UsartReceiveData[6]<<24) );
				   Robot_SoftLimit[1].Right_Limit = (u32)( ((u32)UsartReceiveData[7]) |((u32)UsartReceiveData[8]<<8) |((u32)UsartReceiveData[9]<<16) |((u32)UsartReceiveData[10]<<24) );
				   Robot_SoftLimit[1].Switch_Limit= UsartReceiveData[11];
				   W25QXX_Write(&UsartReceiveData[3],0x2009,9);			   
				   Z_Axsis_Minlength = Robot_SoftLimit[1].Left_Limit+MINROBOTPOSITION;
				   Z_Axsis_Maxlength = Robot_SoftLimit[1].Right_Limit+MINROBOTPOSITION;
			   }
		     else if(UsartReceiveData[2]==0x03)            //L	
			   {
				   Robot_SoftLimit[2].Left_Limit  = (u32)( ((u32)UsartReceiveData[3]) |((u32)UsartReceiveData[4]<<8) |((u32)UsartReceiveData[5]<<16) |((u32)UsartReceiveData[6]<<24) );
				   Robot_SoftLimit[2].Right_Limit = (u32)( ((u32)UsartReceiveData[7]) |((u32)UsartReceiveData[8]<<8) |((u32)UsartReceiveData[9]<<16) |((u32)UsartReceiveData[10]<<24) );
				   Robot_SoftLimit[2].Switch_Limit= UsartReceiveData[11];
				   W25QXX_Write(&UsartReceiveData[3],0x2012,9);			   
				   L_Axsis_Minlength = Robot_SoftLimit[2].Left_Limit+MINROBOTPOSITION;
				   L_Axsis_Maxlength = Robot_SoftLimit[2].Right_Limit+MINROBOTPOSITION;
			   }
		     else if(UsartReceiveData[2]==0x04)            //O	
			   {
				   Robot_SoftLimit[3].Left_Limit  = (u32)( ((u32)UsartReceiveData[3]) |((u32)UsartReceiveData[4]<<8) |((u32)UsartReceiveData[5]<<16) |((u32)UsartReceiveData[6]<<24) );
				   Robot_SoftLimit[3].Right_Limit = (u32)( ((u32)UsartReceiveData[7]) |((u32)UsartReceiveData[8]<<8) |((u32)UsartReceiveData[9]<<16) |((u32)UsartReceiveData[10]<<24) );
				   Robot_SoftLimit[3].Switch_Limit= UsartReceiveData[11];
				   W25QXX_Write(&UsartReceiveData[3],0x201B,9);			   
				   O_Axsis_Minlength = Robot_SoftLimit[3].Left_Limit+MINROBOTPOSITION;
				   O_Axsis_Maxlength = Robot_SoftLimit[3].Right_Limit+MINROBOTPOSITION;
			   }
		     else if(UsartReceiveData[2]==0x05)            //A	
			   {
				   Robot_SoftLimit[4].Left_Limit  = (u32)( ((u32)UsartReceiveData[3]) |((u32)UsartReceiveData[4]<<8) |((u32)UsartReceiveData[5]<<16) |((u32)UsartReceiveData[6]<<24) );
				   Robot_SoftLimit[4].Right_Limit = (u32)( ((u32)UsartReceiveData[7]) |((u32)UsartReceiveData[8]<<8) |((u32)UsartReceiveData[9]<<16) |((u32)UsartReceiveData[10]<<24) );
				   Robot_SoftLimit[4].Switch_Limit= UsartReceiveData[11];
				   W25QXX_Write(&UsartReceiveData[3],0x2024,9);			   
				   A_Axsis_Minlength = Robot_SoftLimit[4].Left_Limit+MINROBOTPOSITION;
				   A_Axsis_Maxlength = Robot_SoftLimit[4].Right_Limit+MINROBOTPOSITION;
			   }
		     else if(UsartReceiveData[2]==0x06)            //B	
			   {
				   Robot_SoftLimit[5].Left_Limit  = (u32)( ((u32)UsartReceiveData[3]) |((u32)UsartReceiveData[4]<<8) |((u32)UsartReceiveData[5]<<16) |((u32)UsartReceiveData[6]<<24) );
				   Robot_SoftLimit[5].Right_Limit = (u32)( ((u32)UsartReceiveData[7]) |((u32)UsartReceiveData[8]<<8) |((u32)UsartReceiveData[9]<<16) |((u32)UsartReceiveData[10]<<24) );
				   Robot_SoftLimit[5].Switch_Limit= UsartReceiveData[11];
				   W25QXX_Write(&UsartReceiveData[3],0x202D,9);			   
				   B_Axsis_Minlength = Robot_SoftLimit[5].Left_Limit+MINROBOTPOSITION;
				   B_Axsis_Maxlength = Robot_SoftLimit[5].Right_Limit+MINROBOTPOSITION;
			   }
		   }break;
	
		  /**-- 安全区域参数 0x2100--**/
		  case P_PARAMETER_SAFE_AREA:{
			   address1 = (UsartReceiveData[2]&0xf0)>>4;
			   address2 =  UsartReceiveData[2]&0x0f;
			   Code = UsartReceiveData[3]-1;
			   if(address1 == 0x01)	                         //左上
			   {
			   	if(address2 == 0x01)                         //X
					{
						Robot_Safe_Area[Code].X_Left = (u32)( ((u32)UsartReceiveData[4]) |((u32)UsartReceiveData[5]<<8) |((u32)UsartReceiveData[6]<<16) |((u32)UsartReceiveData[7]<<24) );
						Robot_Safe_Area[Code].X_Left = Robot_Safe_Area[Code].X_Left+MINROBOTPOSITION;
						W25QXX_Write(&UsartReceiveData[4],0x2100+0x20*Code,4);
					}
					else			                                   //Z
					{
						Robot_Safe_Area[Code].Z_Up = (u32)( ((u32)UsartReceiveData[4]) |((u32)UsartReceiveData[5]<<8) |((u32)UsartReceiveData[6]<<16) |((u32)UsartReceiveData[7]<<24) );
						Robot_Safe_Area[Code].Z_Up = Robot_Safe_Area[Code].Z_Up+MINROBOTPOSITION;
						W25QXX_Write(&UsartReceiveData[4],0x2100+0x20*Code+0x04,4);
					}
			   }
			   else if(address1 == 0x02)	                   //=2 右下
			   {
			   	if(address2 == 0x01)                         //X
					{
						Robot_Safe_Area[Code].X_Right = (u32)( ((u32)UsartReceiveData[4]) |((u32)UsartReceiveData[5]<<8) |((u32)UsartReceiveData[6]<<16) |((u32)UsartReceiveData[7]<<24) );
						Robot_Safe_Area[Code].X_Right = Robot_Safe_Area[Code].X_Right+MINROBOTPOSITION;
						W25QXX_Write(&UsartReceiveData[4],0x2100+0x20*Code+0x08,4);
					}
					else			                                   //Z
					{
						Robot_Safe_Area[Code].Z_Down = (u32)( ((u32)UsartReceiveData[4]) |((u32)UsartReceiveData[5]<<8) |((u32)UsartReceiveData[6]<<16) |((u32)UsartReceiveData[7]<<24) );
						Robot_Safe_Area[Code].Z_Down = Robot_Safe_Area[Code].Z_Down+MINROBOTPOSITION;
						W25QXX_Write(&UsartReceiveData[4],0x2100+0x20*Code+0x0C,4);
					}			   
			   }
			   else	                                         //=3 启用-关闭
			   {
			   	Robot_Safe_Area[Code].SafeArea_Switch=UsartReceiveData[4];
					W25QXX_Write(&UsartReceiveData[4],0x2100+0x20*Code+0x10,1);
			   }
		   }break;
				 
		  /**-- 功能设定参数 0x1000--**/	
		  case P_PARAMETER_FUCTION_SET:{
			   switch(UsartReceiveData[2])
			   {
			    	case 0x01:	//机械手参数
//						JXS_Parameter.Axis      =UsartReceiveData[3];
//						JXS_Parameter.Origin    =UsartReceiveData[4];
//						W25QXX_Write(&UsartReceiveData[3],0x10B0,2);//6.29改成22	12/8改成26	 3.21改成28
//						
//						JXS_Parameter.Auto_SpeedLevel=UsartReceiveData[5];
////						JXS_Parameter.OriginSpeed = UsartReceiveData[6];
//						JXS_Parameter.Manual_SpeedLevel = UsartReceiveData[7];//手动速度
//						W25QXX_Write(&UsartReceiveData[5],0x10B8,3);//6.29改成22	12/8改成26	 3.21改成28
//						
//						 //---设定回原点速度，
//						 Origin_Speed[0]=JXS_Parameter.OriginSpeed[0];
//						
//						 AUTO_PULSE_MINHALF_PERIOD=PULSE_MINHALFPERIOD*100/JXS_Parameter.Auto_SpeedLevel;		
						 break;

			    	case 0x02:	//上料仓参数1
						 LC_ID  =	UsartReceiveData[3]+1;
						 LC_Parameter_Load[LC_ID-1].ID			   = UsartReceiveData[3];
					 	 LC_Parameter_Load[LC_ID-1].Switch           = UsartReceiveData[4];
						 LC_Parameter_Load[LC_ID-1].H_Num            = UsartReceiveData[5];
						 LC_Parameter_Load[LC_ID-1].V_Num            = UsartReceiveData[6];	
						 LC_Parameter_Load[LC_ID-1].Glue_Time        = ( ((u32)UsartReceiveData[7]) |((u32)UsartReceiveData[8]<<8) );
						 LC_Parameter_Load[LC_ID-1].Diamonds_Time    = ( ((u32)UsartReceiveData[9]) |((u32)UsartReceiveData[10]<<8) );
						 LC_Parameter_Load[LC_ID-1].HT_Hight = (u32)( ((u32)UsartReceiveData[11]) |((u32)UsartReceiveData[12]<<8) |((u32)UsartReceiveData[13]<<16) |((u32)UsartReceiveData[14]<<24) );
						 W25QXX_Write(&UsartReceiveData[3],0x1100+(LC_ID-1)*40,12);
				     break;

			    	case 0x03:	//上料仓参数2
						 LC_Parameter_Load[LC_ID-1].LC_Hight         = (u32)( ((u32)UsartReceiveData[3]) |((u32)UsartReceiveData[4]<<8) |((u32)UsartReceiveData[5]<<16) |((u32)UsartReceiveData[6]<<24) );
						 LC_Parameter_Load[LC_ID-1].Position1.XPoint = (u32)( ((u32)UsartReceiveData[7]) |((u32)UsartReceiveData[8]<<8) |((u32)UsartReceiveData[9]<<16) |((u32)UsartReceiveData[10]<<24) );
						 LC_Parameter_Load[LC_ID-1].Position1.YPoint = (u32)( ((u32)UsartReceiveData[11]) |((u32)UsartReceiveData[12]<<8) |((u32)UsartReceiveData[13]<<16) |((u32)UsartReceiveData[14]<<24) );						
						 LC_Parameter_Load[LC_ID-1].Position2.XPoint = (u32)( ((u32)UsartReceiveData[15]) |((u32)UsartReceiveData[16]<<8) |((u32)UsartReceiveData[17]<<16) |((u32)UsartReceiveData[18]<<24) );
						 LC_Parameter_Load[LC_ID-1].Position2.YPoint = (u32)( ((u32)UsartReceiveData[19]) |((u32)UsartReceiveData[20]<<8) |((u32)UsartReceiveData[21]<<16) |((u32)UsartReceiveData[22]<<24) );
						 LC_Parameter_Load[LC_ID-1].Position3.XPoint = (u32)( ((u32)UsartReceiveData[23]) |((u32)UsartReceiveData[24]<<8) |((u32)UsartReceiveData[25]<<16) |((u32)UsartReceiveData[26]<<24) );
						 LC_Parameter_Load[LC_ID-1].Position3.YPoint = (u32)( ((u32)UsartReceiveData[27]) |((u32)UsartReceiveData[28]<<8) |((u32)UsartReceiveData[29]<<16) |((u32)UsartReceiveData[30]<<24) );						 
						 W25QXX_Write(&UsartReceiveData[3],0x1100+(LC_ID-1)*40+12,28);
						 Calculate_LC_Load_Parameter(LC_ID-1);
						 Calculate_LC_Load_OAxis_Parameter();
				     break;

			    	case 0x06:	//生产参数
						 SC_Parameter.RW_Num = (u32)( ((u32)UsartReceiveData[3]) |((u32)UsartReceiveData[4]<<8) |((u32)UsartReceiveData[5]<<16) |((u32)UsartReceiveData[6]<<24) );
						 SC_Parameter.CJ_Num = (u32)( ((u32)UsartReceiveData[7]) |((u32)UsartReceiveData[8]<<8) |((u32)UsartReceiveData[9]<<16) |((u32)UsartReceiveData[10]<<24) );
						 SC_Parameter.JG_Num = (u32)( ((u32)UsartReceiveData[11])|((u32)UsartReceiveData[12]<<8)|((u32)UsartReceiveData[13]<<16)|((u32)UsartReceiveData[14]<<24) );
						 SC_Parameter.SC_Num = (u32)( ((u32)UsartReceiveData[15])|((u32)UsartReceiveData[16]<<8)|((u32)UsartReceiveData[17]<<16)|((u32)UsartReceiveData[18]<<24) );
						 SC_Parameter.LJ_Num = (u32)( ((u32)UsartReceiveData[19])|((u32)UsartReceiveData[20]<<8)|((u32)UsartReceiveData[21]<<16)|((u32)UsartReceiveData[22]<<24) );
						 W25QXX_Write(&UsartReceiveData[3],0x1090,20);
						 break;
					case 0x07://电子齿轮比
						switch(UsartReceiveData[3])
						{
							case 0:
								JXS_Parameter.GearRatio[0] = UsartReceiveData[4]+UsartReceiveData[5]<<8;//X电子齿轮比
								W25QXX_Write(&UsartReceiveData[4],0x10D0,2);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 1:
								JXS_Parameter.GearRatio[1] = UsartReceiveData[4]+UsartReceiveData[5]<<8;//Z电子齿轮比
								W25QXX_Write(&UsartReceiveData[4],0x10D2,2);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 2:
								JXS_Parameter.GearRatio[2] = UsartReceiveData[4]+UsartReceiveData[5]<<8;//Y电子齿轮比
								W25QXX_Write(&UsartReceiveData[4],0x10D4,2);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 3:
								JXS_Parameter.GearRatio[3] = UsartReceiveData[4]+UsartReceiveData[5]<<8;//O电子齿轮比
								W25QXX_Write(&UsartReceiveData[4],0x10D6,12);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 4:
								JXS_Parameter.GearRatio[4] = UsartReceiveData[4]+UsartReceiveData[5]<<8;//A电子齿轮比
								W25QXX_Write(&UsartReceiveData[4],0x10D8,12);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 5:
								JXS_Parameter.GearRatio[5] = UsartReceiveData[4]+UsartReceiveData[5]<<8;//B电子齿轮比
								W25QXX_Write(&UsartReceiveData[4],0x10DA,12);//6.29改成22	12/8改成26	 3.21改成28
								break;
						}
						break;
					case 0x08://复位参数
						if(UsartReceiveData[3]==1)
						{
							JXS_Parameter.OriginDir[X_Axsis]=UsartReceiveData[5];
							JXS_Parameter.OriginDir[Z_Axsis]=UsartReceiveData[6];
							JXS_Parameter.OriginDir[L_Axsis]=UsartReceiveData[7];
							JXS_Parameter.OriginDir[O_Axsis]=UsartReceiveData[8];
							JXS_Parameter.OriginDir[A_Axsis]=UsartReceiveData[9];
							JXS_Parameter.OriginDir[B_Axsis]=UsartReceiveData[10];							
							W25QXX_Write(&UsartReceiveData[5],0x10B2,6);//6.29改成22	12/8改成26	 3.21改成28
						}
						if(UsartReceiveData[3]==2)
						{
							JXS_Parameter.OriginSpeed[X_Axsis]=UsartReceiveData[4];
							JXS_Parameter.OriginSpeed[Z_Axsis]=UsartReceiveData[5];
							JXS_Parameter.OriginSpeed[L_Axsis]=UsartReceiveData[6];
							JXS_Parameter.OriginSpeed[O_Axsis]=UsartReceiveData[7];
							JXS_Parameter.OriginSpeed[A_Axsis]=UsartReceiveData[8];
							JXS_Parameter.OriginSpeed[B_Axsis]=UsartReceiveData[9];							
							W25QXX_Write(&UsartReceiveData[4],0x10B8,6);//6.29改成22	12/8改成26	 3.21改成28
							Origin_Speed[X_Axsis]=JXS_Parameter.OriginSpeed[X_Axsis];
							Origin_Speed[Z_Axsis]=JXS_Parameter.OriginSpeed[Z_Axsis];
							Origin_Speed[L_Axsis]=JXS_Parameter.OriginSpeed[L_Axsis];
							Origin_Speed[O_Axsis]=JXS_Parameter.OriginSpeed[O_Axsis];
							Origin_Speed[A_Axsis]=JXS_Parameter.OriginSpeed[A_Axsis];
							Origin_Speed[B_Axsis]=JXS_Parameter.OriginSpeed[B_Axsis];
						}
						if(UsartReceiveData[3]==3)
						{
							JXS_Parameter.OriginPosition[X_Axsis]=UsartReceiveData[4];
							JXS_Parameter.OriginPosition[Z_Axsis]=UsartReceiveData[5];
							JXS_Parameter.OriginPosition[L_Axsis]=UsartReceiveData[6];
							JXS_Parameter.OriginPosition[O_Axsis]=UsartReceiveData[7];
							JXS_Parameter.OriginPosition[A_Axsis]=UsartReceiveData[8];
							JXS_Parameter.OriginPosition[B_Axsis]=UsartReceiveData[9];						
							W25QXX_Write(&UsartReceiveData[4],0x10BE,6);//6.29改成22	12/8改成26	 3.21改成28
						}
						if(UsartReceiveData[3]==4)
						{
							JXS_Parameter.Origin[X_Axsis] = UsartReceiveData[4];
							JXS_Parameter.Origin[L_Axsis] = UsartReceiveData[5];
							JXS_Parameter.Origin[Z_Axsis] = UsartReceiveData[6];
							JXS_Parameter.Origin[O_Axsis] = UsartReceiveData[7];
							JXS_Parameter.Origin[A_Axsis] = UsartReceiveData[8];
							JXS_Parameter.Origin[B_Axsis] = UsartReceiveData[9];					
							W25QXX_Write(&UsartReceiveData[4],0x10C4,6);//6.29改成22	12/8改成26	 3.21改成28
						}								
						break;
					case 0x09:
						switch(UsartReceiveData[3])
						{
							case 0x01:
								JXS_Parameter.ManualSpeed_L[0]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x0500,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x02:
								JXS_Parameter.ManualSpeed_H[0]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x0501,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x03:
								JXS_Parameter.AutoSpeed_L[0]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x0502,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x04:
								JXS_Parameter.AutoSpeed_H[0]=UsartReceiveData[4];
								Pulse_Present_MaxSpeed[X_Axsis]=JXS_Parameter.AutoSpeed_H[0]*1000;
								Pulse_Present_MaxSpeed[Z_Axsis]=JXS_Parameter.AutoSpeed_H[0]*1000;
								Pulse_Present_MaxSpeed[L_Axsis]=JXS_Parameter.AutoSpeed_H[0];
								Pulse_Present_MaxSpeed[O_Axsis]=JXS_Parameter.AutoSpeed_H[0]*1000;
								W25QXX_Write(&UsartReceiveData[4],0x0503,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x05:
								JXS_Parameter.Acceleration_UP[0]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x0504,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x06:
								JXS_Parameter.Acceleration_DOWN[0]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x0505,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x07:
								JXS_Parameter.ManualSpeed_L[1]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x0506,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x08:
								JXS_Parameter.ManualSpeed_H[1]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x0507,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x09:
								JXS_Parameter.AutoSpeed_L[1]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x0508,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x0A:
								JXS_Parameter.AutoSpeed_H[1]=UsartReceiveData[4];
								Pulse_Present_MaxSpeed[A_Axsis]=JXS_Parameter.AutoSpeed_H[1]*1000;
								W25QXX_Write(&UsartReceiveData[4],0x0509,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x0B:
								JXS_Parameter.Acceleration_UP[1]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x050A,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x0C:
								JXS_Parameter.Acceleration_DOWN[1]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x050B,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x0D:
								JXS_Parameter.ManualSpeed_L[2]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x050C,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x0E:
								JXS_Parameter.ManualSpeed_H[2]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x050D,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x0F:
								JXS_Parameter.AutoSpeed_L[2]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x050E,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x10:
								JXS_Parameter.AutoSpeed_H[2]=UsartReceiveData[4];
								Pulse_Present_MaxSpeed[B_Axsis]=JXS_Parameter.AutoSpeed_H[2]*1000;
								W25QXX_Write(&UsartReceiveData[4],0x050F,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x11:
								JXS_Parameter.Acceleration_UP[2]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x0510,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							case 0x12:
								JXS_Parameter.Acceleration_DOWN[2]=UsartReceiveData[4];
								W25QXX_Write(&UsartReceiveData[4],0x0511,1);//6.29改成22	12/8改成26	 3.21改成28
								break;
							
						}
						break;
			   }
		   }break;
				 
		  default:
		     break;
	}
}



/**************************************************************************************************
**  函数名：  FreeProgramSend()
**	输入参数：无
**	输出参数：无
**	函数功能：自由编程发送
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void FreeProgramSend()
{
	switch(UsartReceiveData[1])
	{
		case PROGRAM_FROM_USB_START:		  //本段程序下发来自U盘 开始
		     Program_From_UDisk_Flag=TRUE;
			 //将地址数组清零
			 for(i=0;i<SAVEPROGRAMNUM;i++)
			 {
			 	 Program_IIC_Address[i].Flag = 0;
				 Program_IIC_Address[i].Code = 0;
				 Program_IIC_Address[i].Name = 0;
				 Program_IIC_Address[i].Num  = 0;
			 }
			 break;	

		case PROGRAM_FROM_USB_END:		  //本段程序下发来自U盘 结束
		     Program_From_UDisk_Flag=FALSE;
			 g_Run_Program_Num=0;
//			 g_Load_LC_Key=0;
//			 g_UnLoad_LC_Key=0;
			 W25QXX_Write(&g_Run_Program_Num,0x40E0,1);
			 //将地址数组保存
			 for(i=0;i<SAVEPROGRAMNUM;i++)
			 {
			 	adrees_temp[0] = Program_IIC_Address[i].Flag;
				adrees_temp[1] = Program_IIC_Address[i].Code;
				adrees_temp[2] = Program_IIC_Address[i].Name;
			    adrees_temp[3] = Program_IIC_Address[i].Name>>8;
				adrees_temp[4] = Program_IIC_Address[i].Name>>16;
				adrees_temp[5] = Program_IIC_Address[i].Name>>24;
				adrees_temp[6] = Program_IIC_Address[i].Num;			 
			 	W25QXX_Write(adrees_temp,Program_IIC_Address[i].Address,7);
			 }
			 //读取料仓指令行
			 Read_LC_List(g_Run_Program_Num);
			 break;	


		case P_PROGRAM_START:		  //开始接收本次程序
		     Start_Recieve_Program_Flag=TRUE;
			 Temp_Num=0;
			 SaveProgram_IIC_Address=0;
			 Free_Program_Operate.Flag = 0;
			 Free_Program_Operate.Code = 0;   
			 Free_Program_Operate.Name = 0;
			 Free_Program_Operate.Num  = 0;
			 break;	

		case PROGRAM_INFO:			  //本次程序信息
				Free_Program_Operate1.Flag = 1;
				Free_Program_Operate1.Code = UsartReceiveData[3];   
				Free_Program_Operate1.Name = (u32)( ((u32)UsartReceiveData[4]) |((u32)UsartReceiveData[5]<<8) |((u32)UsartReceiveData[6]<<16) |((u32)UsartReceiveData[7]<<24) );
				Free_Program_Operate1.Num  = UsartReceiveData[8]|UsartReceiveData[9]<<8;
				g_Run_Program_Num=UsartReceiveData[3];
				Saved_Program[g_Run_Program_Num-1].Flag=1;
				Saved_Program[g_Run_Program_Num-1].Name=Free_Program_Operate1.Name;
				Saved_Program[g_Run_Program_Num-1].Order_Num=Free_Program_Operate1.Num;
				Saved_Program[g_Run_Program_Num-1].Time=(u32)( ((u32)UsartReceiveData[10]) |((u32)UsartReceiveData[11]<<8) |((u32)UsartReceiveData[12]<<16) |((u32)UsartReceiveData[13]<<24) );
				Program_Flag[g_Run_Program_Num-1]=1;
				W25QXX_Write(Program_Flag,0x40E0+g_Run_Program_Num-1,1);				
				W25QXX_Write(&UsartReceiveData[2],0x5000+14*(UsartReceiveData[3]-1),14);
			 break;	

		case PROGRAM_CONT:			  //本次程序内容
		    if(Start_Recieve_Program_Flag)
			 {
				if(Temp_Num<LARGESTPROGRAMNUM)
				{
					Free_Program_Operate1.Program[Temp_Num].Flag   = 1;
					Free_Program_Operate1.Program[Temp_Num].List   = UsartReceiveData[2];
//					Free_Program_Operate1.Program[Temp_Num].Order  = OR_AXISORDER;
					Free_Program_Operate1.Program[Temp_Num].Key    = UsartReceiveData[20];
					Free_Program_Operate1.Program[Temp_Num].Value1 = (u32)( ((u32)UsartReceiveData[3])|((u32)UsartReceiveData[4]<<8)|((u32)UsartReceiveData[5]<<16)|((u32)UsartReceiveData[6]<<24) );
					Free_Program_Operate1.Program[Temp_Num].Value2 = (u32)( ((u32)UsartReceiveData[7])|((u32)UsartReceiveData[8]<<8)|((u32)UsartReceiveData[9]<<16)|((u32)UsartReceiveData[10]<<24) );	
					Free_Program_Operate1.Program[Temp_Num].Value3 = (u32)( ((u32)UsartReceiveData[11])|((u32)UsartReceiveData[12]<<8)|((u32)UsartReceiveData[13]<<16)|((u32)UsartReceiveData[14]<<24) ); 					
					Free_Program_Operate1.Program[Temp_Num].Value1 = Free_Program_Operate1.Program[Temp_Num].Value1 & 0x0fffffff;
					Free_Program_Operate1.Program[Temp_Num].Value2 = Free_Program_Operate1.Program[Temp_Num].Value2 & 0x0fffffff;	
					Free_Program_Operate1.Program[Temp_Num].Value3 = Free_Program_Operate1.Program[Temp_Num].Value3 & 0x0fffffff;	
					if( K_ZAXISPOSITION<=Free_Program_Operate1.Program[Temp_Num].Key && Free_Program_Operate1.Program[Temp_Num].Key<=K_OAXIS)	//轴命令
					{
						Free_Program_Operate1.Program[Temp_Num].Value1 = Free_Program_Operate1.Program[Temp_Num].Value1+MINROBOTPOSITION;
						Free_Program_Operate1.Program[Temp_Num].Value2 = Free_Program_Operate1.Program[Temp_Num].Value1+MINROBOTPOSITION;
						Free_Program_Operate1.Program[Temp_Num].Value3 = Free_Program_Operate1.Program[Temp_Num].Value1+MINROBOTPOSITION;
					}
				}
				W25QXX_Write(&UsartReceiveData[2],Saved_Program[g_Run_Program_Num-1].Address_Order+Temp_Num*ORDER_LENTH,ORDER_LENTH);	 //每次写入一条语句，写的条数由Num决定
				Temp_Num++;
				if(Temp_Num==Free_Program_Operate1.Num)
				{
					Start_Recieve_Program_Flag=FALSE;
					Temp_Num=0;
				}
			 }
			 break;	

		case P_PROGRAM_RENAME:		  //(产品重命名)
				for(i=0;i<10;i++)
					if(Saved_Program[i].Code==UsartReceiveData[6])
					{
						Saved_Program[i].Name = UsartReceiveData[2]+(UsartReceiveData[3]<<8)+(UsartReceiveData[4]<<16)+(UsartReceiveData[5]<<24);
						W25QXX_Write(&UsartReceiveData[2],0x5000+i*14+2,4);
					}
			 break;	
	
		case P_PROGRAM_NEW:	//产品新建
			for(i=0;i<10;i++)
				if(Saved_Program[i].Code==UsartReceiveData[3])
				{
					g_Run_Program_Num = UsartReceiveData[3];
					W25QXX_Write(&g_Run_Program_Num,0x40C0,1);//当前运行程序切换				
					
					Program_Flag[i]=1;
					W25QXX_Write(Program_Flag,0x40E0,SAVEPROGRAMNUM);
					
					Free_Program_Operate1.Flag = 1;
					Saved_Program[i].Flag = 1;
					Saved_Program[i].Name = UsartReceiveData[4]|UsartReceiveData[5]<<8|UsartReceiveData[6]<<16|UsartReceiveData[7]<<24;
					Saved_Program[i].Order_Num = UsartReceiveData[8]|UsartReceiveData[9]<<8;
					Saved_Program[i].Time 		 = UsartReceiveData[10]|UsartReceiveData[11]<<8|UsartReceiveData[12]<<16|UsartReceiveData[13]<<24;
					W25QXX_Write(&UsartReceiveData[2],0x5000+i*14,14);
					Confirm_Order_Num  = 0;
					g_Auto_PresentLine = 0;
					Action_Step_Run_Num= 0;	
				}				
			break;

		case P_PROGRAM_CHOOSE:			  //(产品选择)
			for(i=0;i<10;i++)
				if(Saved_Program[i].Code==UsartReceiveData[2])
				{
					Free_Program_Operate1.Flag = 1;
					Free_Program_Operate1.Code = Saved_Program[i].Code;
					Free_Program_Operate1.Name = Saved_Program[i].Name;
					Free_Program_Operate1.Num  = Saved_Program[i].Order_Num;
					Confirm_Order_Num=0;
					g_Auto_PresentLine=0;
					Action_Step_Run_Num=0;
					g_Run_Program_Num	= UsartReceiveData[2];	
					W25QXX_Write(&g_Run_Program_Num,0x40C0,1);				
					
					Program_Send();//产品选择将控制板程序上传
				}
				UsartReceiveData[1]=0;
			 break;	

		case P_PROGRAM_DELETE:		 //删除程序		 
			 for(i=0;i<10;i++)
				if(Saved_Program[i].Code==UsartReceiveData[2])
				{
					Saved_Program[i].Flag = 0;
					Program_Flag[i] = 0;
					W25QXX_Write(Program_Flag,0x40E0,SAVEPROGRAMNUM);
					Product_Num=Product_Num-1;
					W25QXX_Write(&Product_Num,0x40D0,1);
					if(g_Run_Program_Num==Saved_Program[i].Code)//删除当前程序
					{
						g_Run_Program_Num=0;
						W25QXX_Write(&g_Run_Program_Num,0x40C0,1);				
					}
		
				}
			 break;		
		case P_PROGRAM_SEND://程序载入
				
			Program_Send();
		default:
		     break;
	}
}



/**************************************************************************************************
**  函数名：  ReadIIC_Point()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC存储点数据。
**	备注：	  
**  作者：    吴祥     
**  开发日期：2015/11/03 
***************************************************************************************************/
void ReadIIC_Point()
{
	u8 data_temp[12]={0};
	//完成整个程序搜索及发送，发送同步完成标志位
    if(Search_Time[1] == SAVEPROINT)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[1]+1;
		USART1_SendData(2,0xF2,data_temp);
		Search_Time[1] = 0;
	}
	else
	{
		data_temp[0]=0x11;
		if(Search_Time[1]<=24)		 //X
		{
			W25QXX_Read(&data_temp[1],0x3100+Search_Time[1]*0x10,10);
			data_temp[11]=0;	
			USART1_SendData(12,0xF2,data_temp);
		}
		else if(Search_Time[1]<=49)				    //Z
		{
			W25QXX_Read(&data_temp[1],0x3400+(Search_Time[1]-25)*0x10,10);	
			data_temp[11]=1;	
			USART1_SendData(12,0xF2,data_temp);		
		}
		else if(Search_Time[1]<=74)			    //L
		{
			W25QXX_Read(&data_temp[1],0x3700+(Search_Time[1]-50)*0x10,10);	
			data_temp[11]=2;	
			USART1_SendData(12,0xF2,data_temp);		
		}
		else								  //O
		{
			W25QXX_Read(&data_temp[1],0x3A00+(Search_Time[1]-75)*0x10,10);	
			data_temp[11]=3;	
			USART1_SendData(12,0xF2,data_temp);			
		}
		Search_Time[1]++;
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_Program_Inf()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC程序数据。
**	备注：	  
**  作者：    吴祥     
**  开发日期：2015/11/03 
***************************************************************************************************/
void ReadIIC_Program_Inf()
{
	u8 data_temp[20]={0};
    if(Search_Time[2] == (SAVEPROGRAMNUM))
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[2]+1;
		USART1_SendData(2,0xF3,data_temp);
		Search_Time[2] = 0;		
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[2]+1;
		data_temp[2] = Saved_Program[Search_Time[2]].Flag;
		data_temp[3] = Saved_Program[Search_Time[2]].Code;
		data_temp[4] = Saved_Program[Search_Time[2]].Name;
		data_temp[5] = Saved_Program[Search_Time[2]].Name>>8;
		data_temp[6] = Saved_Program[Search_Time[2]].Name>>16;
		data_temp[7] = Saved_Program[Search_Time[2]].Name>>24;
		data_temp[8] = Saved_Program[Search_Time[2]].Order_Num;
		data_temp[9] = Saved_Program[Search_Time[2]].Order_Num>>8;
		data_temp[10]= Saved_Program[Search_Time[2]].Time;
		data_temp[11]= Saved_Program[Search_Time[2]].Time>>8;
		data_temp[12]= Saved_Program[Search_Time[2]].Time>>16;
		data_temp[13]= Saved_Program[Search_Time[2]].Time>>24;		
		USART1_SendData(14,0xF3,data_temp);		
		Search_Time[2]++;
	}
}
/**************************************************************************************************
**  函数名：  ReadIIC_Program()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC程序数据。
**	备注：	  
**  作者：    吴祥     
**  开发日期：2015/11/03 
***************************************************************************************************/
void ReadIIC_Program()
{
	u8 data_temp[20]={0};
	//完成整个程序搜索及发送，发送同步完成标志位
    if(Search_Time[3] == 1)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[3]+1;
		USART1_SendData(2,0xF4,data_temp);
		Search_Time[3] = 0;
		Send_Time = 0;
	}
	else
	{
		if(UsartReceiveData[2]<=0x02 && Send_Time==0)//开机同步-打开程序
		{
		    Read_SaveProgram_IIC_Address();
		}
		else if(UsartReceiveData[2]>0x02 && Send_Time==0)
		{
		    SaveProgram_IIC_Address=Program_IIC_Address[UsartReceiveData[2]-3].Address;
			SaveProgram_IIC_Num    =Program_IIC_Address[UsartReceiveData[2]-3].Num ;
		}
		if(SaveProgram_IIC_Address>0 && SaveProgram_IIC_Num>0)
		{
			Send_Time++;
			data_temp[0]=0x11;					
			data_temp[1]=Send_Time;	
			if(Send_Time==(SaveProgram_IIC_Num+2))
			{
				//结束本条程序的发送，回复命令
				Search_Time[3]++; 
				data_temp[1]=0x00;
				data_temp[2]=0x00;
				Send_Time =0;
				USART1_SendData(3,0xF4,data_temp);			
			}
			else if(Send_Time==1)	//本条程序第一次查询发送：程序信息
			{
				W25QXX_Read(&data_temp[2],SaveProgram_IIC_Address,7);  //读取每个程序的前7位程序信息						
				USART1_SendData(9,0xF4,data_temp);	   //程序编码-发送次数-七位程序信息			
			}
			else				//：程序内容
			{
				W25QXX_Read(&data_temp[2],SaveProgram_IIC_Address+0x07+(Send_Time-2)*0x10,16);			
				USART1_SendData(18,0xF4,data_temp);	   //程序编码-发送次数-三位程序信息
			}
		}
		else
		{
			data_temp[1]=0x00;
			data_temp[2]=0x00;
			USART1_SendData(3,0xF4,data_temp);
			Search_Time[3]++;			
		}
	}
}

/**************************************************************************************************
**  函数名：  ReadIIC_SoftLimit()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC存储软限位参数。
**	备注：	  
**  作者：    吴祥     
**  开发日期：2015/11/08 
***************************************************************************************************/
void ReadIIC_SoftLimit()
{
	u8 data_temp[12]={0};
	//完成整个软限位搜索及发送，发送同步完成标志位
    if(Search_Time[4] == SAVESOFTLIMIT)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[4]+1;
		USART1_SendData(2,0xF5,data_temp);
		Search_Time[4] = 0;
		 X_Axsis_Minlength = Robot_SoftLimit[0].Left_Limit+MINROBOTPOSITION;
		 X_Axsis_Maxlength = Robot_SoftLimit[0].Right_Limit+MINROBOTPOSITION;
		 Z_Axsis_Minlength = Robot_SoftLimit[1].Left_Limit+MINROBOTPOSITION;
		 Z_Axsis_Maxlength = Robot_SoftLimit[1].Right_Limit+MINROBOTPOSITION;
		 L_Axsis_Minlength = Robot_SoftLimit[2].Left_Limit+MINROBOTPOSITION;
		 L_Axsis_Maxlength = Robot_SoftLimit[2].Right_Limit+MINROBOTPOSITION;
		 O_Axsis_Minlength = Robot_SoftLimit[3].Left_Limit+MINROBOTPOSITION;
		 O_Axsis_Maxlength = Robot_SoftLimit[3].Right_Limit+MINROBOTPOSITION;
		 A_Axsis_Minlength = Robot_SoftLimit[4].Left_Limit+MINROBOTPOSITION;
		 A_Axsis_Maxlength = Robot_SoftLimit[4].Right_Limit+MINROBOTPOSITION;
		 B_Axsis_Minlength = Robot_SoftLimit[5].Left_Limit+MINROBOTPOSITION;
		 B_Axsis_Maxlength = Robot_SoftLimit[5].Right_Limit+MINROBOTPOSITION;
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[4]+1;
		W25QXX_Read(&data_temp[2],0x2000+Search_Time[4]*0x09,9);	
		USART1_SendData(11,0xF5,data_temp);
		Robot_SoftLimit[Search_Time[4]].Left_Limit  = (u32)( ((u32)data_temp[2]) |((u32)data_temp[3]<<8) |((u32)data_temp[4]<<16) |((u32)data_temp[5]<<24) );
		Robot_SoftLimit[Search_Time[4]].Right_Limit = (u32)( ((u32)data_temp[6]) |((u32)data_temp[7]<<8) |((u32)data_temp[8]<<16) |((u32)data_temp[9]<<24) );
		Robot_SoftLimit[Search_Time[4]].Switch_Limit= data_temp[10];
		Search_Time[4]++;
	}

}

/**************************************************************************************************
**  函数名：  ReadIIC_SafeArea()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC存储安全区参数。
**	备注：	  
**  作者：    吴祥     
**  开发日期：2015/11/08 
***************************************************************************************************/
void ReadIIC_SafeArea()
{
	u8 data_temp[20]={0};
	//完成整个程序搜索及发送，发送同步完成标志位
    if(Search_Time[5] == SAVESAFEAREA)
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[5]+1;
		USART1_SendData(2,0xF6,data_temp);
		Search_Time[5] = 0;
		//
		GPIO_SetBits(GPIOA,GPIO_Pin_5);		  //机械手伺服使能
		Robot_Enable = TRUE;
		//
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[5]+1;
		W25QXX_Read(&data_temp[2],0x2100+Search_Time[5]*0x20,17);	
		USART1_SendData(19,0xF6,data_temp);
	    Robot_Safe_Area[Search_Time[5]].X_Left  = (u32)( ((u32)data_temp[2]) |((u32)data_temp[3]<<8) |((u32)data_temp[4]<<16) |((u32)data_temp[5]<<24) );
		Robot_Safe_Area[Search_Time[5]].X_Left  = Robot_Safe_Area[Search_Time[5]].X_Left+MINROBOTPOSITION;
		Robot_Safe_Area[Search_Time[5]].Z_Up    = (u32)( ((u32)data_temp[6]) |((u32)data_temp[7]<<8) |((u32)data_temp[8]<<16) |((u32)data_temp[9]<<24) );
		Robot_Safe_Area[Search_Time[5]].Z_Up    = Robot_Safe_Area[Search_Time[5]].Z_Up+MINROBOTPOSITION;
		Robot_Safe_Area[Search_Time[5]].X_Right = (u32)( ((u32)data_temp[10]) |((u32)data_temp[11]<<8)|((u32)data_temp[12]<<16)|((u32)data_temp[13]<<24));
		Robot_Safe_Area[Search_Time[5]].X_Right = Robot_Safe_Area[Search_Time[5]].X_Right+MINROBOTPOSITION;
		Robot_Safe_Area[Search_Time[5]].Z_Down  = (u32)( ((u32)data_temp[14])|((u32)data_temp[15]<<8)|((u32)data_temp[16]<<16)|((u32)data_temp[17]<<24));
		Robot_Safe_Area[Search_Time[5]].Z_Down  = Robot_Safe_Area[Search_Time[5]].Z_Down+MINROBOTPOSITION;
		Robot_Safe_Area[Search_Time[5]].SafeArea_Switch = data_temp[18];
		Search_Time[5]++;
	}
}


/**************************************************************************************************
**  函数名：  ReadIIC_Setting()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC机械手设置参数。
**	备注：	  
**  作者：    吴祥     
**  开发日期：2015/11/13 
***************************************************************************************************/
void ReadIIC_Setting()
{
	u8 data_temp[50]={0};//---由23改成25  6.29//由25改成29 12.11  3.21改成30
//	u8 Data_Clear[8]={0};
//---6.29
	u8 i;
	//完成整个程序搜索及发送，发送同步完成标志位
  if(Search_Time[0] == 24)	  //依次上传机械手、料仓、生产参数
	{
		data_temp[0] = 0xAA;
		data_temp[1] = Search_Time[0]+1;
		USART1_SendData(2,0xF1,data_temp);
		Search_Time[0] = 0;
	}
	else
	{
		data_temp[0] = 0x11;
		data_temp[1] = Search_Time[0]+1;	
		if(Search_Time[0] == 0)				 //机械手参数 2+1 //0x1000
		{
			
			W25QXX_Read(&data_temp[2],0x10B0,26);//	6.29	//12.8改成26   3.21改成28
// 			W25QXX_Read(0x1000,&data_temp[2],14);//---由12改成14    6.28
//			JXS_Parameter.Axis       =data_temp[2];
//			JXS_Parameter.Origin     =data_temp[3];
			JXS_Parameter.OriginDir[X_Axsis] =data_temp[4];
			JXS_Parameter.OriginDir[Z_Axsis] =data_temp[5];
			JXS_Parameter.OriginDir[L_Axsis] =data_temp[6];
			JXS_Parameter.OriginDir[O_Axsis] =data_temp[7];
			JXS_Parameter.OriginDir[A_Axsis] =data_temp[8];
			JXS_Parameter.OriginDir[B_Axsis] =data_temp[9];
			JXS_Parameter.OriginSpeed[X_Axsis] =data_temp[10];
			JXS_Parameter.OriginSpeed[Z_Axsis] =data_temp[11];
			JXS_Parameter.OriginSpeed[L_Axsis] =data_temp[12];
			JXS_Parameter.OriginSpeed[O_Axsis] =data_temp[13];
			JXS_Parameter.OriginSpeed[A_Axsis] =data_temp[14];
			JXS_Parameter.OriginSpeed[B_Axsis] =data_temp[15];		
			JXS_Parameter.OriginPosition[X_Axsis] =data_temp[16];
			JXS_Parameter.OriginPosition[Z_Axsis] =data_temp[17];
			JXS_Parameter.OriginPosition[L_Axsis] =data_temp[18];
			JXS_Parameter.OriginPosition[O_Axsis] =data_temp[19];
			JXS_Parameter.OriginPosition[A_Axsis] =data_temp[20];
			JXS_Parameter.OriginPosition[B_Axsis] =data_temp[21];
			JXS_Parameter.Origin[X_Axsis]     = data_temp[22];			
			JXS_Parameter.Origin[L_Axsis]     = data_temp[23];
			JXS_Parameter.Origin[Z_Axsis]     = data_temp[24];
			JXS_Parameter.Origin[O_Axsis]     = data_temp[25];
			JXS_Parameter.Origin[A_Axsis]     = data_temp[26];
			JXS_Parameter.Origin[B_Axsis]     = data_temp[27];			
			USART1_SendData(28,0xF1,data_temp);//---由14改成16---6.29  改成24//改成28   3.21改成30
			Origin_Speed[X_Axsis]=JXS_Parameter.OriginSpeed[X_Axsis];				
			Origin_Speed[Z_Axsis]=JXS_Parameter.OriginSpeed[Z_Axsis];				
			Origin_Speed[L_Axsis]=JXS_Parameter.OriginSpeed[L_Axsis];				
			Origin_Speed[O_Axsis]=JXS_Parameter.OriginSpeed[O_Axsis];				
			Origin_Speed[A_Axsis]=JXS_Parameter.OriginSpeed[A_Axsis];				
			Origin_Speed[B_Axsis]=JXS_Parameter.OriginSpeed[B_Axsis];					
		    SpeedUp_Max_A[L_Axsis]=SpeedUp_Max_Acceleration_Sheet[(200-200)/10];	//2019.6.17 JXS_Parameter.Accelerate.Time[i]
		}
		else if(Search_Time[0] == 1||Search_Time[0] == 2||Search_Time[0] == 3||Search_Time[0] == 4||Search_Time[0] == 5
			||Search_Time[0] == 6||Search_Time[0] == 7||Search_Time[0] == 8||Search_Time[0] == 9||Search_Time[0] == 10)			  //上料仓参数1  2+1
		{
				i=Search_Time[0]-1;
				W25QXX_Read(&data_temp[2],0x1100+i*40,12);
				LC_Parameter_Load[i].ID     = data_temp[2];
				LC_Parameter_Load[i].Switch = data_temp[3];
				LC_Parameter_Load[i].H_Num = data_temp[4];
				LC_Parameter_Load[i].V_Num = data_temp[5];
				LC_Parameter_Load[i].Glue_Time =(u16)(data_temp[6]|data_temp[7]<<8);
				LC_Parameter_Load[i].Diamonds_Time = (u16)(data_temp[8]|data_temp[9]<<8);
				LC_Parameter_Load[i].HT_Hight = (u32)( ((u32)data_temp[10]) |((u32)data_temp[11]<<8) |((u32)data_temp[12]<<16)|((u32)data_temp[13]<<24) );
				USART1_SendData(14,0xF1,data_temp);		
		}
		else if(Search_Time[0] == 11||Search_Time[0] == 12||Search_Time[0] == 13||Search_Time[0] == 14||Search_Time[0] == 15
			||Search_Time[0] == 16||Search_Time[0] == 17||Search_Time[0] == 18||Search_Time[0] == 19||Search_Time[0] == 20)			  //上料仓参数2 3+1
		{
				i=Search_Time[0]-11;
				W25QXX_Read(&data_temp[2],0x1100+i*40+12,28);
				LC_Parameter_Load[i].LC_Hight         = (u32)( ((u32)data_temp[2]) |((u32)data_temp[3]<<8) |((u32)data_temp[4]<<16) |((u32)data_temp[5]<<24) );
				LC_Parameter_Load[i].Position1.XPoint = (u32)( ((u32)data_temp[6]) |((u32)data_temp[7]<<8) |((u32)data_temp[8]<<16) |((u32)data_temp[9]<<24) );
				LC_Parameter_Load[i].Position1.YPoint = (u32)( ((u32)data_temp[10]) |((u32)data_temp[11]<<8) |((u32)data_temp[12]<<16) |((u32)data_temp[13]<<24) );
				LC_Parameter_Load[i].Position2.XPoint = (u32)( ((u32)data_temp[14]) |((u32)data_temp[15]<<8) |((u32)data_temp[16]<<16) |((u32)data_temp[17]<<24) );
				LC_Parameter_Load[i].Position2.YPoint = (u32)( ((u32)data_temp[18]) |((u32)data_temp[19]<<8) |((u32)data_temp[20]<<16) |((u32)data_temp[21]<<24) );
				LC_Parameter_Load[i].Position3.XPoint = (u32)( ((u32)data_temp[22]) |((u32)data_temp[23]<<8) |((u32)data_temp[24]<<16) |((u32)data_temp[25]<<24) );
				LC_Parameter_Load[i].Position3.YPoint = (u32)( ((u32)data_temp[26]) |((u32)data_temp[27]<<8) |((u32)data_temp[28]<<16) |((u32)data_temp[29]<<24) );	
				LC_ID=i;
			  if(LC_ID<=9)
				Calculate_LC_Load_Parameter(LC_ID);
//				Calculate_LC_Load_OAxis_Parameter();
				USART1_SendData(30,0xF1,data_temp);
		}
		else if(Search_Time[0] == 21)			 //生产参数 6+1
		{
			W25QXX_Read(&data_temp[2],0x1090,20);
			//当前任务计划数
			SC_Parameter.RW_Num = (u32)( ((u32)data_temp[2]) |((u32)data_temp[3]<<8) |((u32)data_temp[4]<<16) |((u32)data_temp[5]<<24) );
			//抽检数
			SC_Parameter.CJ_Num = (u32)( ((u32)data_temp[6]) |((u32)data_temp[7]<<8) |((u32)data_temp[8]<<16) |((u32)data_temp[9]<<24) );
			//抽检间隔数
			SC_Parameter.JG_Num = (u32)( ((u32)data_temp[10]) |((u32)data_temp[11]<<8)|((u32)data_temp[12]<<16)|((u32)data_temp[13]<<24) );
			//生产的量
			SC_Parameter.SC_Num = (u32)( ((u32)data_temp[14]) |((u32)data_temp[15]<<8)|((u32)data_temp[16]<<16)|((u32)data_temp[17]<<24) );
			//2016-12-28 //2017-12-7
			//SC_Parameter.SC_Num=0;
//			W25QXX_Write(0x1090+0x0C,Data_Clear,8);
			//累计生产量
			SC_Parameter.LJ_Num = (u32)( ((u32)data_temp[18]) |((u32)data_temp[19]<<8)|((u32)data_temp[20]<<16)|((u32)data_temp[21]<<24) );
			USART1_SendData(22,0xF1,data_temp);	
					
		}
		else if(Search_Time[0] == 22)			 //生产参数 6+1
		{
			W25QXX_Read(&data_temp[2],0x10D0,13);
			JXS_Parameter.GearRatio[0] = data_temp[2]|data_temp[3]<<8;//X电子齿轮比
			JXS_Parameter.GearRatio[2] = data_temp[4]|data_temp[5]<<8;//Y电子齿轮比
			JXS_Parameter.GearRatio[1] = data_temp[6]|data_temp[7]<<8;//Z电子齿轮比
			JXS_Parameter.GearRatio[3] = data_temp[8]|data_temp[9]<<8;//O电子齿轮比
			JXS_Parameter.GearRatio[4] = data_temp[10]|data_temp[11]<<8;//V电子齿轮比
			JXS_Parameter.GearRatio[5] = data_temp[12]|data_temp[13]<<8;//W电子齿轮比
			USART1_SendData(14,0xF1,data_temp);			
		}
		else if(Search_Time[0] == 23)			 //速度参数 6+1
		{
			W25QXX_Read(&data_temp[2],0x0500,18);
			JXS_Parameter.ManualSpeed_L[0] = data_temp[2];
			JXS_Parameter.ManualSpeed_H[0] = data_temp[3];
			JXS_Parameter.AutoSpeed_L[0]   = data_temp[4];
			JXS_Parameter.AutoSpeed_H[0]   = data_temp[5];
			JXS_Parameter.Acceleration_UP[0]   = data_temp[6];
			JXS_Parameter.Acceleration_DOWN[0] = data_temp[7];
			JXS_Parameter.ManualSpeed_L[1]     = data_temp[8];
			JXS_Parameter.ManualSpeed_H[1]     = data_temp[9];
			JXS_Parameter.AutoSpeed_L[1]       = data_temp[10];
			JXS_Parameter.AutoSpeed_H[1]       = data_temp[11];
			JXS_Parameter.Acceleration_UP[1]   = data_temp[12];
			JXS_Parameter.Acceleration_DOWN[1] = data_temp[13];
			JXS_Parameter.ManualSpeed_L[2]     = data_temp[14];
			JXS_Parameter.ManualSpeed_H[2]     = data_temp[15];
			JXS_Parameter.AutoSpeed_L[2]       = data_temp[16];
			JXS_Parameter.AutoSpeed_H[2]       = data_temp[17];
			JXS_Parameter.Acceleration_UP[2]   = data_temp[18];
			JXS_Parameter.Acceleration_DOWN[2] = data_temp[19];
			Pulse_Present_MaxSpeed[X_Axsis]=JXS_Parameter.AutoSpeed_H[0]*1000;
			Pulse_Present_MaxSpeed[Z_Axsis]=JXS_Parameter.AutoSpeed_H[0]*1000;
			Pulse_Present_MaxSpeed[L_Axsis]=JXS_Parameter.AutoSpeed_H[0];
			Pulse_Present_MaxSpeed[O_Axsis]=JXS_Parameter.AutoSpeed_H[0]*1000;
			Pulse_Present_MaxSpeed[A_Axsis]=JXS_Parameter.AutoSpeed_H[1]*1000;
			Pulse_Present_MaxSpeed[B_Axsis]=JXS_Parameter.AutoSpeed_H[2]*1000;
			JXS_Parameter.SPEED_CONTROL_TIME_MANUAL[1]=36000/(Pluse_Div[(JXS_Parameter.ManualSpeed_L[1]-1)]);//手动示教加速周期A
			JXS_Parameter.SPEED_CONTROL_TIME_MANUAL[2]=36000/(Pluse_Div[(JXS_Parameter.ManualSpeed_L[2]-1)]);//手动示教加速周期B
			USART1_SendData(20,0xF1,data_temp);			
		}
		Search_Time[0]++;
	}
}

/**************************************************************************************************
**  函数名：  ReadIICData()
**	输入参数：无
**	输出参数：无
**	函数功能：串口读取IIC数据，开机数据从IIC读取之后，接收到串口命令则将数据发送给手控器
**	备注：	  分为六种类型数据读取
**  作者：    吴祥     
**  开发日期：20135/11/03 
***************************************************************************************************/
void ReadIICData()
{
	  //修改IIC读取协议,第一字节为协议标识位,当前数据查询结束为0xAA ,否则为0x11
	  switch(UsartReceiveData[1])
	  {

	  	case 0x01:				//读取设置参数
		     ReadIIC_Setting();
			 break;

	  	case 0x02:				//读取保存点
		     ReadIIC_Point();
			 break;
	  		 
	  	case 0x03:				//读取保存程序
		     ReadIIC_Program_Inf();		 
			 break;

	  	case 0x04:				//读取保存程序
		     ReadIIC_Program();		 
			 break;

	  	case 0x05:				//读取软限位
		     ReadIIC_SoftLimit();
			 break;

	  	case 0x06:				//读取安全区
			 ReadIIC_SafeArea();
		     break;

	  	case 0x07:				//读取报警数据
		     //ReadIIC_Alarm();
			 break;

//	  	case 0x07:				//读取保存程序
//		     ReadIIC_One_Program();		 
//			 break;

  		default:
		     break;
	  }

}

/**************************************************************************************************
**  函数名：  ReadIICSysParameter()
**	输入参数：无
**	输出参数：无
**	函数功能：开机读取IIC-同步参数
**	备注：	  存取都是地位在先
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void ReadIICSysParameter()
{
	u8 i=0;
	for(i=0;i<100;i++)
	{
		IIC_Parameter[i]=0;
	}

	for(i=1;i<SAVEPROGRAMNUM;i++)
	{
		Saved_Program[i].Flag    = 0;
		Saved_Program[i].Code    = 0;
		Saved_Program[i].Name    = 0;
		Saved_Program[i].Order_Num = 0;
	}
	for(i=0;i<7;i++)
	{
		IIC_Parameter[i]=0;
	}
	
  if(*SpeedUp_Jerk==0)
	{
		 SpeedUp_Jerk=SpeedUp_Jerk1;
		 SpeedUp_Max_Acceleration=SpeedUp_Max_Acceleration1;
	}
	if(JXS_Parameter.PulseTime==0)
	{
	   JXS_Parameter.PulseTime=50;
	}
	
	/*********************2018.6.9 WHHITE_L*****************************/
	W25QXX_Read(&g_Run_Program_Num,0x40C0,1);//断电前运行程序	
//	Program_Flag[0]=1;	
//	W25QXX_Write(Program_Flag,0x40E0,SAVEPROGRAMNUM);	//读取有效程序
	W25QXX_Read(Program_Flag,0x40E0,SAVEPROGRAMNUM);	//读取有效程序

	for(i=0;i<SAVEPROGRAMNUM;i++)
	{
		if(Program_Flag[i]==1)
		{
			W25QXX_Read(IIC_Parameter,0x5000+14*i, 14);
			Saved_Program[i].Flag    = IIC_Parameter[0];
			Saved_Program[i].Name    = (u32)(((u32)IIC_Parameter[2])|((u32)IIC_Parameter[3]<<8)|((u32)IIC_Parameter[4]<<16)|((u32)IIC_Parameter[5])<<24);
			Saved_Program[i].Order_Num     = IIC_Parameter[6]|IIC_Parameter[7]<<8;
			Saved_Program[i].Time      = (u32)(((u32)IIC_Parameter[8])|((u32)IIC_Parameter[9]<<8)|((u32)IIC_Parameter[10]<<16)|((u32)IIC_Parameter[11])<<24);
			Product_Num++;
		}
		Saved_Program[i].Code    = i+1;
		Saved_Program[i].Address_Order = 0xFFFF+i*0x8000;	
	}
	if(Product_Num>0)
	{
		Free_Program_Operate1.Flag = 1;
		Free_Program_Operate1.Code = Saved_Program[g_Run_Program_Num-1].Code;
		Free_Program_Operate1.Name = Saved_Program[g_Run_Program_Num-1].Name;
		Free_Program_Operate1.Num  = Saved_Program[g_Run_Program_Num-1].Order_Num; 
  }	
}



/*************************************************************************
**  函数名：  Read_LC_List(u8)
**	输入参数：程序编号
**	输出参数：无
**	函数功能：读取料仓指令的参数行
**	备注：	  无
**  作者：    wx   
**  开发日期：2016/12/05
**************************************************************************/
void Read_LC_List(u8 Run_Program_Num)
{
	u8 i=0;
	LC_Load_Pre_ListX=0;
	LC_Load_Pre_ListY=0;
	LC_UnLoad_Pre_ListX=0;
	LC_UnLoad_Pre_ListY=0;
	LC_UnLoad_Pre_ListZ=0;
	if((Run_Program_Num>0)&&(Free_Program_Operate.Code==Run_Program_Num)&&(Free_Program_Operate.Program[0].Key==K_PROGRAMSTART))
	{
		for(i=0;i<Free_Program_Operate.Num;i++)
		{
		   if(Free_Program_Operate.Program[i].Key==K_LOADPOSITION)
		   {
			   if(Free_Program_Operate.Program[i].Value3==11) //上料X
			   {
			   	   LC_Load_Pre_ListX=i;
			   }
			   else												 //上料Y
			   {
			   	   LC_Load_Pre_ListY=i;
			   }
		   }
		   else if(Free_Program_Operate.Program[i].Key==K_UNLOADPOSITION)
		   {
			   if(Free_Program_Operate.Program[i].Value3==11) //卸料X
			   {
			   	   LC_UnLoad_Pre_ListX=i;
			   }												 
			   else												 //卸料Y
			   {
			   	   LC_UnLoad_Pre_ListY=i;
			   }
		   }		
		   else if(Free_Program_Operate.Program[i].Key==K_ZAXISPOSITION)
		   {
 			   LC_UnLoad_Pre_ListZ=i;
		   }
		}
	}

}


/*************************************************************************
**  函数名：  Calculate_LC_Load_Parameter(u8)
**	输入参数： 
**	输出参数：无
**	函数功能： 
**	备注：	  无
**  作者：    wx   
**  开发日期：2016/12/05
**************************************************************************/
void Calculate_LC_Load_Parameter(u8 m)
{
  u32 HD=0,HDL=0,VD=0,VDL=0;          //横向间距   纵向间距
	u32 DeltaH=0,DeltaV=0;	            //横向补偿   纵向补偿
	u8 i=0,j=0,k=1;					    //i表示横向X  j表示纵向Y		
		if((LC_Parameter_Load[m].Switch==FALSE) || ((LC_Parameter_Load[m].H_Num==1)&&(LC_Parameter_Load[m].V_Num==1)))
		{
			LC_Load_Table[m].LC_Load_Position[0][0].XPoint=LC_Parameter_Load[m].Position1.XPoint;
			LC_Load_Table[m].LC_Load_Position[0][0].YPoint=LC_Parameter_Load[m].Position1.YPoint;	
		}
		else if( (LC_Parameter_Load[m].Switch==TRUE)&&(LC_Parameter_Load[m].H_Num>=1)&&(LC_Parameter_Load[m].V_Num>=1)
				&&(LC_Parameter_Load[m].Position1.XPoint<=LC_Parameter_Load[m].Position2.XPoint)
				&&(LC_Parameter_Load[m].Position2.YPoint<=LC_Parameter_Load[m].Position3.YPoint) )
		{	
				if(LC_Parameter_Load[m].H_Num==1)
			{}
			else
			{
					HD =(LC_Parameter_Load[m].Position2.XPoint-LC_Parameter_Load[m].Position1.XPoint)/(LC_Parameter_Load[m].H_Num+1);  //取整
					HDL=(LC_Parameter_Load[m].Position2.XPoint-LC_Parameter_Load[m].Position1.XPoint)%(LC_Parameter_Load[m].H_Num+1);  //取余
			}
				if(LC_Parameter_Load[m].H_Num==1)
			{}
			else
			{
					VD =(LC_Parameter_Load[m].Position3.YPoint-LC_Parameter_Load[m].Position2.YPoint)/(LC_Parameter_Load[m].V_Num-1);
					VDL=(LC_Parameter_Load[m].Position3.YPoint-LC_Parameter_Load[m].Position2.YPoint)%(LC_Parameter_Load[m].V_Num-1);	 //取余
			}
			//计算平均数
			for(j=0;j<LC_Parameter_Load[m].V_Num;j++)
			{
				for(i=0;i<LC_Parameter_Load[m].H_Num;i++)
				{	//以X1/Y1为参考点进行计算的
					LC_Load_Table[m].LC_Load_Position[i][j].XPoint=LC_Parameter_Load[m].Position1.XPoint+i*HD;
					LC_Load_Table[m].LC_Load_Position[i][j].YPoint=LC_Parameter_Load[m].Position1.YPoint+j*VD;
				}
			}
			//补偿余数
			k=1;
			for(i=(LC_Parameter_Load[m].H_Num-HDL);i<LC_Parameter_Load[m].H_Num;i++)
			{
				for(j=0;j<LC_Parameter_Load[m].V_Num;j++)
				{
					LC_Load_Table[m].LC_Load_Position[i][j].XPoint=LC_Load_Table[m].LC_Load_Position[i][j].XPoint+k*1;
				}
				k++;
			}
			k=1;
			for(j=(LC_Parameter_Load[m].V_Num-VDL);j<LC_Parameter_Load[m].V_Num;j++)
			{
				for(i=0;i<LC_Parameter_Load[m].H_Num;i++)
				{
					LC_Load_Table[m].LC_Load_Position[i][j].YPoint=LC_Load_Table[m].LC_Load_Position[i][j].YPoint+k*1;
				}
				k++;
			}
			//计算偏移量
			if(LC_Parameter_Load[m].Position2.YPoint>LC_Parameter_Load[m].Position1.YPoint)	 //Y向外倾斜 X向右倾斜
			{	//以右边为第一行第一列
				if(LC_Parameter_Load[m].V_Num==1)
				{	}
				else
				{
					DeltaH=(LC_Parameter_Load[m].Position2.XPoint-LC_Parameter_Load[m].Position3.XPoint)/(LC_Parameter_Load[m].V_Num-1); //第二行的偏移
				}
				if(LC_Parameter_Load[m].H_Num==1)
				{	}
				else
				{
					DeltaV=(LC_Parameter_Load[m].Position2.YPoint-LC_Parameter_Load[m].Position1.YPoint)/(LC_Parameter_Load[m].H_Num-1); //第二列的偏移
				}
				for(j=0;j<LC_Parameter_Load[m].V_Num;j++)
				{
					for(i=0;i<LC_Parameter_Load[m].H_Num;i++)
					{
						LC_Load_Table[m].LC_Load_Position[i][j].XPoint=LC_Load_Table[m].LC_Load_Position[i][j].XPoint-j*DeltaH;
						LC_Load_Table[m].LC_Load_Position[i][j].YPoint=LC_Load_Table[m].LC_Load_Position[i][j].YPoint+i*DeltaV;
					}
				}
			}
			else if(LC_Parameter_Load[m].Position2.YPoint<LC_Parameter_Load[m].Position1.YPoint)	 //Y向内倾斜 X向左倾斜
			{	//以右边为第一行第一列
				if(LC_Parameter_Load[m].V_Num==1)
				{	}
				else
				{
					DeltaH=(LC_Parameter_Load[m].Position3.XPoint-LC_Parameter_Load[m].Position2.XPoint)/(LC_Parameter_Load[m].V_Num-1); //第二行的偏移
				}
				if(LC_Parameter_Load[m].H_Num==1)
				{	}
				else
				{
					DeltaV=(LC_Parameter_Load[m].Position1.YPoint-LC_Parameter_Load[m].Position2.YPoint)/(LC_Parameter_Load[m].H_Num-1); //第二列的偏移
				}
				for(j=0;j<LC_Parameter_Load[m].V_Num;j++)
				{
					for(i=0;i<LC_Parameter_Load[m].H_Num;i++)
					{
						LC_Load_Table[m].LC_Load_Position[i][j].XPoint=LC_Load_Table[m].LC_Load_Position[i][j].XPoint+j*DeltaH;
						LC_Load_Table[m].LC_Load_Position[i][j].YPoint=LC_Load_Table[m].LC_Load_Position[i][j].YPoint-i*DeltaV;
					}
				}
			}
		}
}


/*************************************************************************
**  函数名：  Calculate_LC_UnLoad_Parameter()
**	输入参数： 
**	输出参数：无
**	函数功能：  
**	备注：	  无
**  作者：    wx   
**  开发日期：2016/12/05
**************************************************************************/
void Calculate_LC_UnLoad_Parameter()
{
//    u32 HD=0,HDL=0,VD=0,VDL=0;          //横向间距   纵向间距
//	u32 DeltaH=0,DeltaV=0;	            //横向补偿   纵向补偿
//	u8 i=0,j=0,k=1;					    //i表示横向X  j表示纵向Y
//	if((LC_Parameter_UnLoad.Switch==FALSE) || ((LC_Parameter_UnLoad.H_Num==1)&&(LC_Parameter_UnLoad.V_Num==1)))
//	{
//	    LC_UnLoad_Position[0][0].XPoint=LC_Parameter_UnLoad.Position1.XPoint;
//		LC_UnLoad_Position[0][0].YPoint=LC_Parameter_UnLoad.Position1.YPoint;	
//	}
//	else if( (LC_Parameter_UnLoad.Switch==TRUE)&&(LC_Parameter_UnLoad.H_Num>=1)&&(LC_Parameter_UnLoad.V_Num>=1)
//	    &&(LC_Parameter_UnLoad.Position1.XPoint<=LC_Parameter_UnLoad.Position2.XPoint)
//	    &&(LC_Parameter_UnLoad.Position2.YPoint<=LC_Parameter_UnLoad.Position3.YPoint) )
//	{	
//	    if(LC_Parameter_UnLoad.H_Num==1)
//		{}
//		else
//		{
//		    HD =(LC_Parameter_UnLoad.Position2.XPoint-LC_Parameter_UnLoad.Position1.XPoint)/(LC_Parameter_UnLoad.H_Num-1);
//	    	HDL=(LC_Parameter_UnLoad.Position2.XPoint-LC_Parameter_UnLoad.Position1.XPoint)%(LC_Parameter_UnLoad.H_Num-1);	 //取余
//		}
//	    if(LC_Parameter_UnLoad.V_Num==1)
//		{}
//		else
//		{
//	        VD =(LC_Parameter_UnLoad.Position3.YPoint-LC_Parameter_UnLoad.Position2.YPoint)/(LC_Parameter_UnLoad.V_Num-1);
//	    	VDL=(LC_Parameter_UnLoad.Position3.YPoint-LC_Parameter_UnLoad.Position2.YPoint)%(LC_Parameter_UnLoad.V_Num-1);	 //取余
//		}
//		//计算平均值
//		for(j=0;j<LC_Parameter_UnLoad.V_Num;j++)
//		{
//			for(i=0;i<LC_Parameter_UnLoad.H_Num;i++)
//			{	//以X1/Y1为参考点进行计算的
//			    LC_UnLoad_Position[i][j].XPoint=LC_Parameter_UnLoad.Position1.XPoint+i*HD;
//				LC_UnLoad_Position[i][j].YPoint=LC_Parameter_UnLoad.Position1.YPoint+j*VD;
//			}
//		}
//		//补偿余数
//		k=1;
//		for(i=(LC_Parameter_UnLoad.H_Num-HDL);i<LC_Parameter_UnLoad.H_Num;i++)
//		{
//			for(j=0;j<LC_Parameter_UnLoad.V_Num;j++)
//			{
//			    LC_UnLoad_Position[i][j].XPoint=LC_UnLoad_Position[i][j].XPoint+k*1;
//			}
//			k++;
//		}
//		k=1;
//		for(j=(LC_Parameter_UnLoad.V_Num-VDL);j<LC_Parameter_UnLoad.V_Num;j++)
//		{
//			for(i=0;i<LC_Parameter_UnLoad.H_Num;i++)
//			{
//				LC_UnLoad_Position[i][j].YPoint=LC_UnLoad_Position[i][j].YPoint+k*1;
//			}
//			k++;
//		}
//		//计算偏移量
//		if(LC_Parameter_UnLoad.Position2.YPoint>LC_Parameter_UnLoad.Position1.YPoint)	 //Y向外倾斜 X向右倾斜
//		{	//以右边为第一行第一列
//		    if(LC_Parameter_UnLoad.V_Num==1)
//			{}
//			else
//			{
//		        DeltaH=(LC_Parameter_UnLoad.Position2.XPoint-LC_Parameter_UnLoad.Position3.XPoint)/(LC_Parameter_UnLoad.V_Num-1); //第二行的偏移
//			}
//		    if(LC_Parameter_UnLoad.H_Num==1)
//			{}
//			else
//			{
//			    DeltaV=(LC_Parameter_UnLoad.Position2.YPoint-LC_Parameter_UnLoad.Position1.YPoint)/(LC_Parameter_UnLoad.H_Num-1); //第二列的偏移
//		    }
//			for(j=0;j<LC_Parameter_UnLoad.V_Num;j++)
//			{
//				for(i=0;i<LC_Parameter_UnLoad.H_Num;i++)
//				{
//				    LC_UnLoad_Position[i][j].XPoint=LC_UnLoad_Position[i][j].XPoint-j*DeltaH;
//					LC_UnLoad_Position[i][j].YPoint=LC_UnLoad_Position[i][j].YPoint+i*DeltaV;
//				}
//			}
//		}
//		else if(LC_Parameter_UnLoad.Position2.YPoint<LC_Parameter_UnLoad.Position1.YPoint)	 //Y向内倾斜 X向左倾斜
//		{	//以右边为第一行第一列
//		    if(LC_Parameter_UnLoad.V_Num==1)
//			{}
//			else
//			{
//		        DeltaH=(LC_Parameter_UnLoad.Position3.XPoint-LC_Parameter_UnLoad.Position2.XPoint)/(LC_Parameter_UnLoad.V_Num-1); //第二行的偏移
//			}
//		    if(LC_Parameter_UnLoad.V_Num==1)
//			{}
//			else
//			{
//			    DeltaV=(LC_Parameter_UnLoad.Position1.YPoint-LC_Parameter_UnLoad.Position2.YPoint)/(LC_Parameter_UnLoad.H_Num-1); //第二列的偏移
//		    }
//			for(j=0;j<LC_Parameter_UnLoad.V_Num;j++)
//			{
//				for(i=0;i<LC_Parameter_UnLoad.H_Num;i++)
//				{
//				    LC_UnLoad_Position[i][j].XPoint=LC_UnLoad_Position[i][j].XPoint+j*DeltaH;
//					LC_UnLoad_Position[i][j].YPoint=LC_UnLoad_Position[i][j].YPoint-i*DeltaV;
//				}
//			}
//		}
//	}

}


/*************************************************************************
**  函数名：  Calculate_LC_OAxis_Parameter()
**	输入参数： 
**	输出参数：无
**	函数功能：  
**	备注：	  无
**  作者：    wx   
**  开发日期：2016/12/05
**************************************************************************/
void Calculate_LC_Load_OAxis_Parameter()
{
//	u32 OD=0,ODL=0;
//	u8 i=0,k=1;
//	if((LC_Parameter_Load.Switch==FALSE) || (LC_Parameter_Load.Layer<=1))
//	{}
//	else if(LC_Parameter_Load.O_PositionE>=LC_Parameter_Load.O_PositionS)
//	{
//		OD =(LC_Parameter_Load.O_PositionE-LC_Parameter_Load.O_PositionS)/(LC_Parameter_Load.Layer-1); //取整
//		ODL=(LC_Parameter_Load.O_PositionE-LC_Parameter_Load.O_PositionS)%(LC_Parameter_Load.Layer-1); //取余
//		for(i=0;i<LC_Parameter_Load.Layer;i++)
//		{
//		    LC_Load_OAxis_Position[i]=LC_Parameter_Load.O_PositionS+i*OD;
//		}		
//	}
//	//补偿余数
//	for(i=(LC_Parameter_Load.Layer-ODL);i<LC_Parameter_Load.Layer;i++)
//	{
//		LC_Load_OAxis_Position[i]=LC_Load_OAxis_Position[i]+k*1;
//		k++;
//	}
}

/*************************************************************************
**  函数名：  Calculate_LC_UnLoad_OAxis_Parameter()
**	输入参数： 
**	输出参数：无
**	函数功能：  
**	备注：	  无
**  作者：    wx   
**  开发日期：2016/12/05
**************************************************************************/
void Calculate_LC_UnLoad_OAxis_Parameter()
{
//	u32 OD=0,ODL=0;
//	u8 i=0,k=1;
//	if((LC_Parameter_UnLoad.Switch==FALSE) || (LC_Parameter_UnLoad.Layer<=1))
//	{}
//	else if(LC_Parameter_UnLoad.O_PositionE>=LC_Parameter_UnLoad.O_PositionS)
//	{
//		OD =(LC_Parameter_UnLoad.O_PositionE-LC_Parameter_UnLoad.O_PositionS)/(LC_Parameter_UnLoad.Layer-1); //取整
//		ODL=(LC_Parameter_UnLoad.O_PositionE-LC_Parameter_UnLoad.O_PositionS)%(LC_Parameter_UnLoad.Layer-1); //取余
//		for(i=0;i<LC_Parameter_UnLoad.Layer;i++)
//		{
//		    LC_UnLoad_OAxis_Position[i]=LC_Parameter_UnLoad.O_PositionS+i*OD;
//		}		
//	}
//	//补偿余数
//	for(i=(LC_Parameter_UnLoad.Layer-ODL);i<LC_Parameter_UnLoad.Layer;i++)
//	{
//		LC_UnLoad_OAxis_Position[i]=LC_UnLoad_OAxis_Position[i]+k*1;
//		k++;
//	}
}

/**************************************************************************************************
**  函数名：  Program_Send()
**	输入参数：无
**	输出参数：无
**	函数功能：上传程序
**	备注：	  无
**  作者：    White_L
**  开发日期：2018/8/20 
***************************************************************************************************/
void Program_Send(void)
{
	u8 data_temp[60];
	if(UsartReceiveData[2]==0xBB)
	{
		g_Run_Program_Num = UsartReceiveData[3];
		W25QXX_Read(data_temp,Saved_Program[g_Run_Program_Num-1].Address_Order+((UsartReceiveData[4]|UsartReceiveData[5]<<8)-1)*ORDER_LENTH,ORDER_LENTH);
		Start_Recieve_Program_Flag=FALSE;
		if((UsartReceiveData[4]|UsartReceiveData[5]<<8)==Saved_Program[g_Run_Program_Num-1].Order_Num)
		{
			W25QXX_Write(&g_Run_Program_Num,0x40C0,1);//断电前运行程序							
			Free_Program_Operate1.Flag = 1;
			Free_Program_Operate1.Code = g_Run_Program_Num;
			Free_Program_Operate1.Name = Saved_Program[g_Run_Program_Num-1].Name;
			Free_Program_Operate1.Num  = Saved_Program[g_Run_Program_Num-1].Order_Num;
			Read_Point_Num=0;
			Order_Read_Mode=0;
			ShuZu_Count=0;
			Order_Package();
		}
		USART1_SendData(ORDER_LENTH,0xE0,data_temp);     //发送程序
	}
	else if(UsartReceiveData[2]==0xAA)
	{
		g_Run_Program_Num = UsartReceiveData[3];
		W25QXX_Write(&UsartReceiveData[4],Saved_Program[g_Run_Program_Num-1].Address_Order+((UsartReceiveData[4]|UsartReceiveData[5]<<8)-1)*ORDER_LENTH,ORDER_LENTH);
		Start_Recieve_Program_Flag=FALSE;		
		if((UsartReceiveData[4]|UsartReceiveData[5]<<8)==Saved_Program[g_Run_Program_Num-1].Order_Num)
		{
			W25QXX_Write(&g_Run_Program_Num,0x40C0,1);//断电前运行程序				
			Free_Program_Operate1.Flag = 1;
			Free_Program_Operate1.Code = g_Run_Program_Num;
			Free_Program_Operate1.Name = Saved_Program[g_Run_Program_Num-1].Name;
			Free_Program_Operate1.Num  = Saved_Program[g_Run_Program_Num-1].Order_Num;
			Read_Point_Num=0;
			Order_Read_Mode=0;
			ShuZu_Count=0;
			Order_Package();
		}
		data_temp[0]=0xE0;
		USART1_SendData(1,0xE0,data_temp);     //接收程序
	}
}
/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

