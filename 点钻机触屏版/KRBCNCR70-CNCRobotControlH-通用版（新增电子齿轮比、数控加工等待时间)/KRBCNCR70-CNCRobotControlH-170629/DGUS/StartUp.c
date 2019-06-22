/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ****************
* File Name          : 
* Author             : 
* Version            : 
* Date               : 
* Description        : 
******************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "DGUS.h"
#include "key.h"

u8  Landing_Success = FALSE;		   //密码登陆标识位
u8  Change_User = FALSE;			     //切换用户
u8  PassWord_Input_Done = FALSE;
u32 Pass_Word_Temp = 0;		         //输入的密码缓存

/*----------------Start_Up--界面1操作标志位----------------*/
//static u8 gs_Start_Up_Page = 0;	             //第一主页面下的页面编号
static u8 gs_Welcom_Flag = 1;	                 //显示欢迎-初始显示登录
//static u8 gs_Current_Alarm = 1;	             //报警显示
static u8 gs_Robot_Enable_Flag = 0;	           //使能
static u8 gs_Emergency_Stop_Flag = 0;	         //急停
static u32 gs_SC_Parameter_SC_Num = 0;		     //加工参数
static u8 gs_Cycle_Run_Flag = 0;	 		         //0:循环  1：单步
//static u8 gs_X_Position_Flag = 0;	 		       //X位置刷新
//static u8 gs_Z_Position_Flag = 0;	 		       //Z位置刷新
//static u8 gs_Robot_Origin_Flag = 0;	 		     //回零
static u8 gs_Auto_Status = AUTO_ERROR;
//static u8 gs_Robot_Run_Flag = 0;			       //启动
//static u8 gs_Robot_Pause_Flag = 0;			     //暂停
//static u8 gs_Robot_Stop_Flag = 0;			       //停止
 u8 gs_Program_Refresh_Flag = FALSE;	   //显示程序刷新

/*-----------剪切图标定义--------------*/
//回零
//static PageCut PageCut_Origin_Y={0x1180,74,509,181,589,246,509,181};	 //回零图标剪切有效果
//static PageCut PageCut_Origin_N={0x1180,1 ,509,181,589,246,509,181};	 //回零图标剪切无效果
//回零中提示
//static PageCut PageCut_Origin_Remind={0x1190,38,209,106,614,349,209,106};//回零中提示

//---原来的 注释掉
//循环-单步
// static PageCut PageCut_CyCle1={0x1130, 1,690,48 ,776,86 ,690,48 };	 //循环图标剪切
// static PageCut PageCut_CyCle2={0x1170, 1,643,341,732,417,643,341};	 //循环图标剪切
// static PageCut PageCut_Sigle1={0x1130,74,690,48 ,776,86 ,690,48 };	 //单步图标剪切---因为0x1130在第一页，所有剪切74页坐标为690,48，776,86 到第一页坐标690，48
// static PageCut PageCut_Sigle2={0x1170,74,643,341,732,417,643,341};	 //单步图标剪切
// //使能-失能
// static PageCut PageCut_Enable ={0x1110,74,460,48 ,540,86 ,460,48 };	 //使能图标剪切
// static PageCut PageCut_Disable={0x1110, 1,460,48 ,540,86 ,460,48 };	 //失能图标剪切
// //急停
// static PageCut PageCut_Emerg_Y={0x1120,74,580,48 ,666,86 ,580,48 };	 
// static PageCut PageCut_Emerg_N={0x1120, 1,580,48 ,666,86 ,580,48 };	 
// //启动
// static PageCut PageCut_Run_Y={0x1140,74,508,255,589,336,508,255};	 
// static PageCut PageCut_Run_N={0x1140, 1,508,255,589,336,508,255};	
// //暂停
// static PageCut PageCut_Pause_Y={0x1150,74,644,254,733,335,644,254};	 
// static PageCut PageCut_Pause_N={0x1150, 1,644,254,733,335,644,254};
// //停止
// static PageCut PageCut_Stop_Y={0x1160,74,506,339,587,419,506,339};	 
// static PageCut PageCut_Stop_N={0x1160, 1,506,339,587,419,506,339};

//---改后的
//循环-单步
//static PageCut PageCut_CyCle1={0x1130, 1,690,48 ,776,86 ,690,48 };	 //循环图标剪切
static PageCut PageCut_CyCle2={0x1170, 1,640,310,730,390,640,310};	   //循环图标剪切
//static PageCut PageCut_Sigle1={0x1130,74,690,48 ,776,86 ,690,48 };	 //单步图标剪切---因为0x1130在第一页，所有剪切74页坐标为690,48，776,86 到第一页坐标690，48
static PageCut PageCut_Sigle2={0x1170,74,640,310,730,390,574,310};	   //单步图标剪切
//使能-失能
//static PageCut PageCut_Enable ={0x1110,74,460,48 ,540,86 ,460,48 };	   //使能图标剪切
//static PageCut PageCut_Disable={0x1110, 1,460,48 ,540,86 ,460,48 };	   //失能图标剪切
//急停
static PageCut PageCut_Emerg_Y={0x1120,74,580,48 ,666,86 ,580,48 };	 
static PageCut PageCut_Emerg_N={0x1120, 1,580,48 ,666,86 ,580,48 };	 
//启动
static PageCut PageCut_Run_Y={0x1140,74,478,193,571,281,467,210};	 
static PageCut PageCut_Run_N={0x1140, 1,490,200,580,280,490,200};	
//暂停
static PageCut PageCut_Pause_Y={0x1150,74,636,200,716,280,569,208};	 
static PageCut PageCut_Pause_N={0x1150, 1,640,200,730,280,640,200};
//停止
static PageCut PageCut_Stop_Y={0x1160,74,490,310,567,390,470,310};	 
static PageCut PageCut_Stop_N={0x1160, 1,490,310,580,390,490,310};


//每一行的背景色
static PageCut Run_Program_Color[9] = {
									  {0x1100,74,10 ,100,418,110,10 ,100}, //不显示时用
									  {0x1100,74,10 ,250,418,277,10 ,151}, //第一行
									  {0x1100,74,10 ,250,418,277,10 ,184}, //第二行
									  {0x1100,74,10 ,250,418,277,10 ,217}, //第三行
									  {0x1100,74,10 ,250,418,277,10 ,250}, //第四行
									  {0x1100,74,10 ,250,418,277,10 ,283}, //第五行
									  {0x1100,74,10 ,250,418,277,10 ,316}, //第六行
									  {0x1100,74,10 ,250,418,277,10 ,349}, //第七行
									  {0x1100,74,10 ,250,418,277,10 ,382}, //第八行	
									}; 

//-----------显示中文字符定义---命名方式：Send_String_XX--------------//
u8 Send_String_HY[]={"欢迎"};
u8 Send_String_DL[]={"登录"};

static u32 gs_StartUp_XAxis_Position = 0;  //X轴位置
static u32 gs_StartUp_ZAxis_Position = 0;  //Z轴位置
static u32 gs_StartUp_LAxis_Position = 0;  //L轴位置
static u32 gs_StartUp_OAxis_Position = 0;  //O轴位置

FreeProgram Run_Program[LARGESTPROGRAMNUM]={0};            //选择运行的程序

u8 gs_Run_Program_List[9]={1,2,3,4,5,1,2,3};	                     //当前显示的8条指令
u8 gs_Run_Program_Color = 1;				                 //当前背景色
u16 gs_Run_Program_Code[8]={0};	                     //当前显示的8个编码
static u32  gs_Run_Program_Parameter_Display[9][2]={0};		 //编码命令对应的两位参数及显示
       u8 gs_Run_Program_Num = 0;				                   //当前选择运行程序的编号
static u16 gs_Run_Program_Operater_List = 0;		             //当前选择运行程序正在运行的指令条：由主控板返回
// static u32 gs_Origin_Speed = 10;				                   //回零速度
// static u8 gs_Refresh_Origin_Speed = TRUE;	               //回零速度刷新
       u8 g_Auto_Status = AUTO_WAITE;
u32 gs_Run_Program_Name=0,g_Run_Program_Name=0;

u8  SuZhu_Count1=8;
u16 Read_Program_Count=0;//两个数组读取程序的计数
u16 ShuZu_Count=0;//指令保存在数组位置
u8  Waite_Reflesh=0;//
extern u8 LCD_Control_Flag;
extern u16 E_GearRatio[6];
extern u8 LC_Empty_Flag;
extern u8 Product_Num;
extern u8 LC_ID,Display_Flag;
extern u8 LCD_Main_Page_Pre;
/*************************************************************************
**  函数名：  LCDStartUpProcess()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void LCDStartUpProcess(void)
{
	 //如果不是当前页面，则切换到当前界面
	if( (LCD_Var_Address == 0x1000) && (LCD_Main_Page != START_UP) )
	{
		LCD_Main_Page_Pre = LCD_Main_Page;
		LCD_Main_Page = START_UP;                         //LCD主页面为开启界面
		LCD_Current_Page = 	LCD_Main_Page;                //LCD当前页面为开启界面
		g_Watch_IO_Control = 0; 	                        //实时IO监视状态
	  Order_Temp[0]=AUTO_MODE;                          //设置为自动模式
	  USART1_SendData(1,WORK_MODE,&Order_Temp[0]);      //串口发送工作模式中的自动模式到主控板
//		g_Cycle_Run_Flag = 1;
//		Robot_Auto_Mode = CYCLE_MODE;							  
	}
	if(Robot_Origined == TRUE)                          //如果机械手回原点
	{
		//LCDStartUpDisply();	                              //开机界面的显示	
		switch(LCD_Var_Address)
		{
			case 0x1007:  				                          //回零操作
				 if(Robot_Status == ACTION_STOP)              //如果机械手状态为停止
				 {
					   Order_Temp[0]=ROBOT_ORIGIN;              //设置机械手回原点
					   USART1_SendData(1,ROBOT_ENABLE_A_ORIGIN,&Order_Temp[0]);  //串口发送机械手使能及回原点中的机械手回原点到主控板
					   Robot_Origined = FALSE;                  //机械手回原点
					   g_Robot_Stop_Flag = 0;                   //机械手停止
					   g_Run_Program_Operater_List = 0;         //当前选择运行程序正在运行的指令条：由主控板返回             			   				   
				 }
				 else
				 {
				   	 LCD_Popup_Windows = PW_1_STOPROBOT;      //弹出提示框-请先停止机器
				 }
				 break;

			case 0x1009: 								 	                  //启动
	       if(gs_Run_Program_Num)				 
			   {
				   	 if(g_Robot_Auto_Reset == FALSE)
					   {
					   	LCD_Current_Page = 88;
							Robot_Status = ACTION_RESET;            //复位状态
					   }
					   else
					   {
						   if(Robot_Status != ACTION_RUN)
						   {
							   Order_Temp[0]=ACTION_RUN;            //设置机械手启动
							   USART1_SendData(1,AUTO_RUN,&Order_Temp[0]);//串口发送机械手自动运行中的启动到主控板
						   }
					   }				   	   	   
			     }
			     else
			     {
			   		  LCD_Popup_Windows = PW_1_CHOOSEPROGRAM;	//弹出提示框-请先加载程序		
			     }
			     break;	

			case 0x100A: 				                            //暂停
					   if(Robot_Status == ACTION_RUN)
					   {
						   Order_Temp[0]=ACTION_PAUSE;            //设置机械手暂停
						   USART1_SendData(1,AUTO_RUN,&Order_Temp[0]);//串口发送机械手自动运行中的暂停到主控板
						   Auto_Run_Flag = FALSE;					   
					   }							   
					   break;	

			case 0x100B: 				                            //停止
					   if((Robot_Status == ACTION_RUN) || (Robot_Status == ACTION_PAUSE))
					   {
					   	 Order_Temp[0]=ACTION_STOP;             //设置机械手停止
						   USART1_SendData(1,AUTO_RUN,&Order_Temp[0]);//串口发送机械手自动运行中的停止到主控板
							 if(Robot_Status == ACTION_RUN)
								g_Run_Program_Operater_List = 0;
						   Auto_Run_Flag = FALSE;
						   g_Robot_Auto_Reset = FALSE;
						 g_Auto_Status=0;
					   }					   				   
					   break;	

			case 0x100C: 				                            //单步-循环
					   if((Robot_Status == ACTION_STOP)||(Robot_Status == ACTION_PAUSE))
					   {
						   if(Robot_Auto_Mode == CYCLE_MODE)
						   {
						   	 Order_Temp[0]=SINGLE_MODE;           //设置机械手为单步模式
							   USART1_SendData(1,AUTO_RUN,&Order_Temp[0]);//串口发送机械手自动运行中的单步模式到主控板
							 }
						   else
						   {
						   	 Order_Temp[0]=CYCLE_MODE;            //设置机械手为循环模式
							   USART1_SendData(1,AUTO_RUN,&Order_Temp[0]);//串口发送机械手自动运送中的循环模式到主控板						   						   
						   }
					   }
					   else
					   {
					   	   LCD_Popup_Windows = PW_1_STOPROBOT;  //弹出提示框-请先停止机器
					   }
					   break;

// 			case 0x1008:	  //回零速度值
// 				 //按下的是加减号
// 				 if(LCD_Key_Value>=0x01)
// 				 {
// 					 gs_Origin_Speed=LCD_Key_Value;
// 				 }
// 				 //按下的是数字输入
// 				 else
// 				 {
// 					 gs_Origin_Speed=LCD_Var_Data;			 
// 				 }
// 				 Order_Temp[0] = ORIGIN_SPEED;
// 				 Order_Temp[1] = gs_Origin_Speed;
// 				 Order_Temp[2] = gs_Origin_Speed>>8;
// 				 Order_Temp[3] = gs_Origin_Speed>>16;
// 				 Order_Temp[4] = gs_Origin_Speed>>24;
// 				 USART1_SendData(5,ROBOT_ENABLE_A_ORIGIN,&Order_Temp[0]);//----ROBOT_ENABLE_A_ORIGIN:0x0A	//机械手使能和回原点
// 				 gs_Refresh_Origin_Speed = TRUE;
// 				 break;

			case 0x1040: 				                             //L轴回零中-确定		   	     
					LCD_Current_Page = 	START_UP;		
					LC_Empty_Flag=0;//料仓空
			     break;

			case 0x1041: 				                             //L轴回零中-取消				 
				    LCD_Current_Page = START_UP;			           //LCD当前页面为开启界面		   
					LC_Empty_Flag=0;//料仓空
			     break;

			case 0x10A0: 				                             //复位-确定		   	     
				   Order_Temp[0]=ACTION_RESET;                 //设置机械手复位
			     USART1_SendData(1,AUTO_RUN,&Order_Temp[0]); //串口发送机械手自动运行中的复位到主控板
				   LCD_Current_Page = 89;						   
			     break;

			case 0x10A1: 				                             //复位-取消
				 LCD_Current_Page = START_UP;                  //LCD当前页面为开启界面
				 Robot_Status = ACTION_STOP;		               //机械手状态为停止
			     break;
			case 0x1EE1:				//工件偏移
				 LCD_Current_Page = 143;
				break;
			
			case 0x1EE2:
				LCD_Current_Page=1;
				break;
			default:
			     break;		   	
		}
	}
	else                                                 //回零操作
	{
// 		if(LCD_Var_Address == 0x1008 )//回零速度值
// 		{	 
// 			 //按下的是加减号
// 			 if(LCD_Key_Value>=0x01)
// 			 {
// 				 gs_Origin_Speed=LCD_Key_Value;
// 			 }
// 			 //按下的是数字输入
// 			 else
// 			 {
// 				 gs_Origin_Speed=LCD_Var_Data;			 
// 			 }
// 			 Order_Temp[0] = ORIGIN_SPEED;
// 			 Order_Temp[1] = gs_Origin_Speed;
// 			 Order_Temp[2] = gs_Origin_Speed>>8;
// 			 Order_Temp[3] = gs_Origin_Speed>>16;
// 			 Order_Temp[4] = gs_Origin_Speed>>24;
// 			 USART1_SendData(5,ROBOT_ENABLE_A_ORIGIN,&Order_Temp[0]);
// 			 gs_Refresh_Origin_Speed = TRUE;
// 		 }
		 if(LCD_Var_Address == 0x1007 )	                    //回原点命令
		 {			
			if(Robot_Enable)
			{
				Order_Temp[0]=ROBOT_ORIGIN;                     //设置机械手回原点
				USART1_SendData(1,ROBOT_ENABLE_A_ORIGIN,&Order_Temp[0]);//---下发回原点到主控板
				Robot_Origined = FALSE;
				g_Robot_Stop_Flag = 0;			
			}
			else
			{
				LCD_Popup_Windows = PW_1_ENABLEFIRST;	          //弹出提示框-请先使能
			}						 
		 }
		 if(Robot_Origining)                                //回零中
		 {
		 }
		 else if((LCD_Var_Address == 0x1009) || (LCD_Var_Address == 0x100A) || (LCD_Var_Address == 0x100B) || (LCD_Var_Address == 0x100C))
		 {
		 	 LCD_Popup_Windows = PW_1_HOMEFIRST;	            //弹出提示框-请先回零		 
		 }
	
	}
	LCD_Key_Value=0;                                      //键值清零
	LCD_Var_Data=0;                                       //变量数据清零
}


void LandingProcess(void)
{
	if(LCD_Current_Page==START_UP && LCD_Var_Address != 0x1001)
	{
		LCD_Popup_Windows = PW_1_LOADFISRT ;	              //弹出提示框-请先登录	
	}
	if(LCD_Var_Address == 0x1001)		                      //按下登录按钮
	{		
		LCD_Current_Page=22;	                              //切换到登陆界面
		Refresh_Admin_Flag=TRUE;	                          //显示权限
	}
	//按下登录权限
	if(LCD_Var_Address == 0x1021)  
	{
	  LCD_Current_Page = 103;	                            //切换到权限选择
	}
	if(LCD_Var_Address == 0x1022)  
	{
	  LCD_Current_Page = 22;	                            //切换到登陆界面
		Refresh_Admin_Flag=TRUE;	                          //显示权限
		Admin_Authority=0;                                  //选择“普通权限”
	}
	if(LCD_Var_Address == 0x1023)  
	{
	  LCD_Current_Page = 22;	                            //切换到登陆界面
		Refresh_Admin_Flag=TRUE;                            //显示权限
		Admin_Authority=1;	                                //选择“管理员权限”
	}

	if(LCD_Var_Address == 0x1004)                         //按下取消按钮
	{
	  if(Change_User == FALSE)                            //不是用户权限切换
		{
			LCD_Current_Page = START_UP;	                    //切换到登陆界面
		}
		else					                                      //是用户权限切换
		{
			LCD_Current_Page = 49;	                          //切换到用户设置
			Change_User=FALSE;
		}
		Refresh_Admin_Flag=TRUE;                            //显示权限
		if(Robot_Admin_Authority==MANAGE_AUTHORITY)
		{
			Admin_Authority=1;
		}
		else
		{
			Admin_Authority=0;
		}
	}
	if(LCD_Var_Address == 0x1005)                          //按下ok,输入密码
	{
		PassWord_Input_Done = TRUE;
		Pass_Word_Temp = LCD_Var_Data;	
	}

	if(LCD_Var_Address==0x1003 && PassWord_Input_Done==TRUE)
	{
		PassWord_Input_Done=FALSE;
		if((Admin_Authority==0) && (Pass_Word[0] == Pass_Word_Temp))//“普通权限”登录密码正确
		{
			Landing_Success = TRUE;                             //登录成功
			Robot_Admin_Authority=ORDINARY_AUTHORITY;           //普通权限
			Pass_Word_Temp=0;
			if(Change_User==FALSE)                              //不是用户权限切换
			{
			  LCD_Current_Page = START_UP;	                    //LCD当前页面为开启界面
				LCD_Main_Page = START_UP;                         //LCD主页面为开启界面
				g_Welcom_Flag = TRUE;                             //显示欢迎
				g_Reflash_Progress_Num = TRUE;                    //刷新产量标志位
				TIM_Cmd(TIM4,ENABLE);
				LCD_Key_Value=0;                                  //键值清零
				LCD_Var_Data=0;                                   //变量数据清零
				g_Key_Scan=FALSE;
				g_BackLight_Dark = FALSE;
			}
	    else
			{			
			  LCD_Current_Page = 49;	                          //切换到用户设置
				Change_User=FALSE;
			}								
		}
		else if((Admin_Authority==1) && (Pass_Word[1] == Pass_Word_Temp))//“管理员权限”登录密码正确
		{
			Landing_Success = TRUE;                             //登录成功
			Robot_Admin_Authority=MANAGE_AUTHORITY;             //管理员权限
			Pass_Word_Temp=0;
			if(Change_User==FALSE)                              //不是用户权限切换
			{
			  LCD_Current_Page = START_UP;	                    //LCD当前页面为开启界面
				LCD_Main_Page = START_UP;                         //LCD主页面为开启界面
				g_Welcom_Flag = TRUE;                             //显示欢迎
				g_Reflash_Progress_Num = TRUE;                    //刷新产量标志位
				TIM_Cmd(TIM4,ENABLE);
				LCD_Key_Value=0;                                  //键值清零
				LCD_Var_Data=0;                                   //变量数据清零
				g_Key_Scan=FALSE;
				g_BackLight_Dark = FALSE;
			}
	    else
			{			
			  LCD_Current_Page = 49;	                          //切换到用户设置
				Change_User=FALSE;
			}	
		}
		else
		{
		  if(Change_User==FALSE)
			{
				Landing_Success = FALSE;                          //登录未成功
				g_Welcom_Flag = FALSE;                            //不显示欢迎
			}
			LCD_Popup_Windows = PW_1_PASSWORDERROR ;	          //弹出提示框-密码错误			
		}			
	}
}


/*************************************************************************
**  函数名：  LCDStartUpDisply()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
extern u8 Point_String[6][8];
void LCDStartUpDisply()
{
	u8 i=0;
	//--界面切换--//
	if(LCD_Pre_Page != LCD_Current_Page )	                   //之前显示界面不是当前界面
	{
		LCD_Pre_Page = LCD_Current_Page;                       //设置之前界面为当前界面
		PageChange(LCD_Pre_Page);
//		gs_Run_Program_Num=0;
	}
	//生产参数显示
	if(g_Reflash_Progress_Num)
	{
		WriteNum4(0x5350,SC_Parameter.RW_Num);                 //任务计划：生产-加工数量
		WriteNum4(0x535A,SC_Parameter.SC_Num);                 //生产-完成数量
		g_Reflash_Progress_Num = FALSE;		
	}
	if((Landing_Success == TRUE)&&(Robot_Origined == TRUE)&&(gs_SC_Parameter_SC_Num != SC_Parameter.SC_Num) )
	{
		WriteNum4(0x535A,SC_Parameter.SC_Num);                 //生产-完成数量
		gs_SC_Parameter_SC_Num = SC_Parameter.SC_Num;          //生产参数为生产-完成数量
		//刷新上料仓位置
		//if(LC_Parameter_Load.Switch==TRUE && LC_Parameter_Load.H_Num>0 && LC_Parameter_Load.V_Num>0)	//料盘计算完成
		if((gs_Run_Program_Num>0) && (g_Calculate_LC_Load_Parameter_Flag==TRUE) )
		{
			LC_Load_Pre_PositionX=SC_Parameter.SC_Num%LC_Parameter_Load[LC_ID].H_Num;	
			LC_Load_Pre_PositionY=SC_Parameter.SC_Num/LC_Parameter_Load[LC_ID].H_Num%LC_Parameter_Load[LC_ID].V_Num;
			if(LC_Load_Pre_ListX>0)
			{
			    Free_Program_Operate.Program[LC_Load_Pre_ListX].Value1=LC_Load_Position[LC_Load_Pre_PositionX][LC_Load_Pre_PositionY].XPoint|0xf0000000;
			}
			if(LC_Load_Pre_ListY>0)
			{
			    Free_Program_Operate.Program[LC_Load_Pre_ListY].Value1=LC_Load_Position[LC_Load_Pre_PositionX][LC_Load_Pre_PositionY].YPoint|0xf0000000;
			}
		}
		//刷新卸料仓位置
		//if(LC_Parameter_UnLoad.Switch==TRUE && LC_Parameter_UnLoad.H_Num>0 && LC_Parameter_UnLoad.V_Num>0)	//料盘计算完成
		if((gs_Run_Program_Num>0) && (g_Calculate_LC_UnLoad_Parameter_Flag==TRUE)) 
		{
		  LC_UnLoad_Pre_PositionX=SC_Parameter.SC_Num%LC_Parameter_UnLoad.H_Num;	
			LC_UnLoad_Pre_PositionY=SC_Parameter.SC_Num/LC_Parameter_UnLoad.H_Num%LC_Parameter_UnLoad.V_Num;
			if(LC_UnLoad_Pre_ListX>0)
			{
			    Free_Program_Operate.Program[LC_UnLoad_Pre_ListX].Value1=LC_UnLoad_Position[LC_UnLoad_Pre_PositionX][LC_UnLoad_Pre_PositionY].XPoint|0xf0000000;
			}
			if(LC_UnLoad_Pre_ListY>0)
			{
			    Free_Program_Operate.Program[LC_UnLoad_Pre_ListY].Value1=LC_UnLoad_Position[LC_UnLoad_Pre_PositionX][LC_UnLoad_Pre_PositionY].YPoint|0xf0000000;
			}
		}
	}
  //--欢迎-登录--//
	if(gs_Welcom_Flag != g_Welcom_Flag)			   
	{
		gs_Welcom_Flag = g_Welcom_Flag;
		if(gs_Welcom_Flag)
		{
			DisplayChineseString(0x100E,Send_String_HY,4);		     //显示欢迎 
		}
		else
		{
			DisplayChineseString(0x100E,Send_String_DL,4);         //显示登录
		}	
	}
	//--使能-失能--//
	if(gs_Robot_Enable_Flag != g_Robot_Enable_Flag)	 
	{
		gs_Robot_Enable_Flag = g_Robot_Enable_Flag;	
		if(gs_Robot_Enable_Flag)
		{
//			Display_PageCut(PageCut_Enable);                       //显示使能
		}
		else
		{
//			Display_PageCut(PageCut_Disable);                      //显示失能
		}	
	}
	//--急停--//
	if(gs_Emergency_Stop_Flag != g_Emergency_Stop_Flag)	 
	{
		gs_Emergency_Stop_Flag = g_Emergency_Stop_Flag;
		if(gs_Emergency_Stop_Flag)
		{
			Display_PageCut(PageCut_Emerg_Y);                      //显示急停
		}
		else
		{
			Display_PageCut(PageCut_Emerg_N);		
		}		
	}
	//--循环-单步--//
	if(gs_Cycle_Run_Flag != g_Cycle_Run_Flag)	 
	{
		gs_Cycle_Run_Flag = g_Cycle_Run_Flag;
		if(gs_Cycle_Run_Flag)	                                   //循环
		{
			//---Display_PageCut(PageCut_CyCle1);
			Display_PageCut(PageCut_CyCle2);                       //显示循环   
			gs_Run_Program_Code[0]=1;
			gs_Run_Program_Code[1]=2;
			gs_Run_Program_Code[2]=3;
			gs_Run_Program_Code[3]=4;
			gs_Run_Program_Code[4]=5;
			gs_Run_Program_Code[5]=6;
			gs_Run_Program_Code[6]=7;
			gs_Run_Program_Code[7]=8;
			gs_Run_Program_List[0]=1; 		
			gs_Run_Program_List[1]=2; 		
			gs_Run_Program_List[2]=3; 		
			gs_Run_Program_List[3]=4; 		
			gs_Run_Program_List[4]=5; 		
			gs_Run_Program_List[5]=1; 		
			gs_Run_Program_List[6]=2; 		
			gs_Run_Program_List[7]=3; 			
		}
		else					                                           //单步
		{
			//---Display_PageCut(PageCut_Sigle1);
			Display_PageCut(PageCut_Sigle2);                       //显示单步		
		}				
	}
	//--X位置刷新--//
	if(gs_StartUp_XAxis_Position != g_XAxis_Position)	 
	{
		gs_StartUp_XAxis_Position = g_XAxis_Position;
		WriteNum4(0x101A,gs_StartUp_XAxis_Position);		
	}
	//--Z位置刷新--//
	if(gs_StartUp_ZAxis_Position != g_ZAxis_Position)	 
	{
		gs_StartUp_ZAxis_Position = g_ZAxis_Position;
		WriteNum4(0x101C,gs_StartUp_ZAxis_Position);		
	}
	//--L位置刷新--//
	if(gs_StartUp_LAxis_Position != g_LAxis_Position)	 
	{
		gs_StartUp_LAxis_Position = g_LAxis_Position;
		WriteNum4(0x101E,gs_StartUp_LAxis_Position);		
	}
	//--O位置刷新--//
	if(gs_StartUp_OAxis_Position != g_OAxis_Position)	 
	{
		gs_StartUp_OAxis_Position = g_OAxis_Position;
		WriteNum4(0x1018,gs_StartUp_OAxis_Position);		
	}
	//程序运行，刷新显示&&Free_Program_Operate.Num<=LARGESTPROGRAMNUM
	if(gs_Run_Program_Operater_List != g_Run_Program_Operater_List)
	{
		Waite_Reflesh=0;
		if(gs_Cycle_Run_Flag)//循环显示固定数据
		{
			gs_Run_Program_List[8]=gs_Run_Program_List[0];
			gs_Run_Program_Parameter_Display[8][0] = gs_Run_Program_Parameter_Display[0][0];
			gs_Run_Program_Parameter_Display[8][1] = gs_Run_Program_Parameter_Display[0][1];
			for(i=0;i<8;i++)
			{
				gs_Run_Program_List[i] = gs_Run_Program_List[i+1];
				gs_Run_Program_Parameter_Display[i][0] = gs_Run_Program_Parameter_Display[i+1][0];
				gs_Run_Program_Parameter_Display[i][1] = gs_Run_Program_Parameter_Display[i+1][1];
			}
			for(i=0;i<8;i++)
			{
				gs_Run_Program_Code[i] = g_Run_Program_Operater_List+i;
			}
		}
		else//单步显示真实数据
		{
			for(i=0;i<8;i++)
			{
				AT24CXX_Read(CURRENT_PROGRAM_ADR1+ORDER_LENTH*(g_Run_Program_Operater_List+i-1),&Order_Temp[3],ORDER_LENTH);
				if(Free_Program_Operate.Num>=g_Run_Program_Operater_List+i)
				{
					gs_Run_Program_Code[i] = g_Run_Program_Operater_List+i;
					gs_Run_Program_List[i] = Order_Temp[26];
					gs_Run_Program_Parameter_Display[i][0] = Order_Temp[5]|Order_Temp[6]<<8|Order_Temp[7]<<16|Order_Temp[8]<<24;
					gs_Run_Program_Parameter_Display[i][1] = Order_Temp[9]|Order_Temp[10]<<8|Order_Temp[11]<<16|Order_Temp[12]<<24;
				}
				else
				{
					gs_Run_Program_Code[i] = 0;
					gs_Run_Program_List[i] = 0;
					gs_Run_Program_Parameter_Display[i][0] = 0;
					gs_Run_Program_Parameter_Display[i][1] = 0;
				}					
			}
		}
			gs_Run_Program_Operater_List = g_Run_Program_Operater_List;
			gs_Program_Refresh_Flag =TRUE;

	}
	//--启动效果--//
	if((g_Auto_Status!=gs_Auto_Status) && (g_Auto_Status == AUTO_RUNNING) )	 
	{
		Display_PageCut(PageCut_Run_Y);                          //显示启动
		Display_PageCut(PageCut_Pause_N);
		Display_PageCut(PageCut_Stop_N);
		gs_Auto_Status=g_Auto_Status;	
		gs_Run_Program_Code[0]=1;
		gs_Run_Program_Code[1]=2;
		gs_Run_Program_Code[2]=3;
		gs_Run_Program_Code[3]=4;
		gs_Run_Program_Code[4]=5;
		gs_Run_Program_Code[5]=6;
		gs_Run_Program_Code[6]=7;
		gs_Run_Program_Code[7]=8;
		gs_Run_Program_List[0]=1; 		
		gs_Run_Program_List[1]=2; 		
		gs_Run_Program_List[2]=3; 		
		gs_Run_Program_List[3]=4; 		
		gs_Run_Program_List[4]=5; 		
		gs_Run_Program_List[5]=1; 		
		gs_Run_Program_List[6]=2; 		
		gs_Run_Program_List[7]=3;
	}
	//--暂停效果刷新--//
	if((g_Auto_Status!=gs_Auto_Status) && (g_Auto_Status == AUTO_PAUSE) )	 
	{
		Waite_Reflesh=0;
		Robot_Status=ACTION_PAUSE;
		Display_PageCut(PageCut_Run_N);            
		Display_PageCut(PageCut_Pause_Y);                        //显示暂停
		Display_PageCut(PageCut_Stop_N);
		gs_Auto_Status=g_Auto_Status;	
		
		for(i=0;i<8;i++)
		{
			AT24CXX_Read(CURRENT_PROGRAM_ADR1+ORDER_LENTH*(g_Run_Program_Operater_List+i-1),&Order_Temp[3],ORDER_LENTH);
			if(Free_Program_Operate.Num>=g_Run_Program_Operater_List+i)
			{
				gs_Run_Program_Code[i] = g_Run_Program_Operater_List+i;
				gs_Run_Program_List[i] = Order_Temp[26];
				gs_Run_Program_Parameter_Display[i][0] = Order_Temp[5]|Order_Temp[6]<<8|Order_Temp[7]<<16|Order_Temp[8]<<24;
				gs_Run_Program_Parameter_Display[i][1] = Order_Temp[9]|Order_Temp[10]<<8|Order_Temp[11]<<16|Order_Temp[12]<<24;
			}
			else
			{
				gs_Run_Program_Code[i] = 0;
				gs_Run_Program_List[i] = 0;
				gs_Run_Program_Parameter_Display[i][0] = 0;
				gs_Run_Program_Parameter_Display[i][1] = 0;
			}				
		}
		gs_Program_Refresh_Flag=TRUE;
	}
	//打开新程序，确定前八行
	if((Landing_Success == TRUE) && ((gs_Run_Program_Num != g_Run_Program_Num)||(gs_Run_Program_Name!=g_Run_Program_Name)))
	{	
		Waite_Reflesh=0;
		gs_Run_Program_Num = g_Run_Program_Num;
		g_Run_Program_List_Num = Free_Program_Operate.Num;
		gs_Run_Program_Name = g_Run_Program_Name;
		//显示程序名
		if(gs_Run_Program_Name==0)
		{
			Temp_Display_Data[0]=0x20;
			Temp_Display_Data[1]=0x20;
			Temp_Display_Data[2]=0x20;
			Temp_Display_Data[3]=0x20;
			DisplayChineseString(0x11F0,Temp_Display_Data,4);
		}
		else
		{
			Temp_Display_Data[0]=gs_Run_Program_Name>>24;
			Temp_Display_Data[1]=gs_Run_Program_Name>>16;
			Temp_Display_Data[2]=gs_Run_Program_Name>>8;
			Temp_Display_Data[3]=gs_Run_Program_Name;
			DisplayChineseString(0x11F0,Temp_Display_Data,4);
		}
		//确定显示编码-程序-参数

		for(i=0;i<8;i++)
		{
			if(Free_Program_Operate.Program[i].Flag !=0)
			{
				gs_Run_Program_Code[i] = Free_Program_Operate.Program[i].List;
				gs_Run_Program_List[i] = Free_Program_Operate.Program[i].Key;
				gs_Run_Program_Parameter_Display[i][0] = Free_Program_Operate.Program[i].Value1;
				gs_Run_Program_Parameter_Display[i][1] = Free_Program_Operate.Program[i].Value2;
			}
			else
			{
				gs_Run_Program_Code[i] = 0;
				gs_Run_Program_List[i] = 0;
				gs_Run_Program_Parameter_Display[i][0] = 0;
				gs_Run_Program_Parameter_Display[i][1] = 0;
			}
		}
		gs_Program_Refresh_Flag =TRUE;
	}
	//--停止效果--//
	else if((g_Auto_Status!=gs_Auto_Status) && (g_Auto_Status == AUTO_WAITE) )	 
	{
		Waite_Reflesh=0;
	    Display_PageCut(PageCut_Run_N);
		Display_PageCut(PageCut_Pause_N);
		Display_PageCut(PageCut_Stop_Y);                         //显示停止
		gs_Auto_Status=g_Auto_Status;
		g_Run_Program_Operater_List = 0;
		Display_Flag=0;
	}
	else if(g_Auto_Status == AUTO_WAITE&&Waite_Reflesh==0)//程序停止状态刷新
	{
		Waite_Reflesh++;
		for(i=0;i<8;i++)
		{
			AT24CXX_Read(CURRENT_PROGRAM_ADR1+ORDER_LENTH*i,&Order_Temp[3],ORDER_LENTH);
			if(Free_Program_Operate.Num>=Order_Temp[3])
			{
				gs_Run_Program_Code[i] = i+1;
				gs_Run_Program_List[i] = Order_Temp[26];
				gs_Run_Program_Parameter_Display[i][0] = Order_Temp[5]|Order_Temp[6]<<8|Order_Temp[7]<<16|Order_Temp[8]<<24;
				gs_Run_Program_Parameter_Display[i][1] = Order_Temp[9]|Order_Temp[10]<<8|Order_Temp[11]<<16|Order_Temp[12]<<24;
			}
			else
			{
				gs_Run_Program_Code[i] = 0;
				gs_Run_Program_List[i] = 0;
				gs_Run_Program_Parameter_Display[i][0] = 0;
				gs_Run_Program_Parameter_Display[i][1] = 0;
			}		
		}
		gs_Program_Refresh_Flag=TRUE;
	}
	//--程序刷新--//
	if(gs_Program_Refresh_Flag)	 
	{
		//显示背景
		Display_PageCut(Run_Program_Color[gs_Run_Program_Color]);
		//显示编码-程序
	    for(i=0;i<8;i++)
	    {
			//显示编码
			DisplayChineseString(0x1200+0x40*i,ProgramCode_String[gs_Run_Program_Code[i]],4);		
			//显示程序
			if(gs_Run_Program_List[i]!=0)
			{
				DisplayChineseString(0x1210+0x40*i,Point_String[gs_Run_Program_List[i]],8);
			}
			else
			{
				DisplayChineseString(0x1210+0x40*i,Point_String[0],8);	
			} 	
			//显示参数
			if((gs_Run_Program_Parameter_Display[i][0] & 0xf0000000) != 0)
			{
				ValueToString(gs_Run_Program_Parameter_Display[i][0]);
				DisplayChineseString(0x1220+0x40*i,Parameter_Value,10);
			}
			else
			{			
				DisplayChineseString(0x1220+0x40*i,Parameter_String[0],10);
			}
			if((gs_Run_Program_Parameter_Display[i][1] & 0xf0000000) != 0)
			{
				ValueToString(gs_Run_Program_Parameter_Display[i][1]);
				DisplayChineseString(0x1230+0x40*i,Parameter_Value,10);
			}
			else
			{
				DisplayChineseString(0x1230+0x40*i,Parameter_String[0],10);
			}
	    }		
		gs_Program_Refresh_Flag = FALSE;		
	}
	//回零速度显示刷新
// 	if(gs_Refresh_Origin_Speed)
// 	{
// 		WriteNum2(0x1008,gs_Origin_Speed); 
// 		gs_Refresh_Origin_Speed = FALSE;	
// 	}
	//权限显示
	if(Refresh_Admin_Flag)
	{
  	 DisplayChineseString(0x1030,Admin_Authority_Dis[Admin_Authority],10);
	   Refresh_Admin_Flag=FALSE;	
	}
	//手动调试-L轴回零完成
	if((LCD_Current_Page==117)&&(g_LAxis_Position==0))
	{
		LCD_Current_Page=1;
	}

}



