/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ****************
* File Name          : 
* Author             : 
* Version            : 
* Date               : 
* Description        : 
******************************************************************************/
#include "stm32f10x.h"
#include "integer.h"
//#include "stm32f10x_lib.h"
#include "DGUS.h"
#include "Delay.h"

u8  LCD_Main_Page_Pre=0;					//记录是从那个主页进入到另一个主页
u8  LCD_Main_Page = 0 ;                     //LCD当前主目录1~6
u8  LCD_Current_Page = 0 ;                  //LCD当前显示界面（主目录1~6）
u8  LCD_Pre_Page = 0;				        //之前显示界面
u16 LCD_Var_Address = 0;					//变量地址
u16 LCD_Key_Value = 0;					    //键值
u16 LCD_GearRatio_Distance[6]={100,100,100,100,100,100};	//坐标校准输入距离 默认100mm
u32 LCD_Var_Data = 0;				        //变量数据
u16 LCD_RTC_Reg_len = 0;					//实时时钟寄存器及数据长度
u32 LCD_RTC_Day = 0;					    //年-月-日-周
u32 LCD_RTC_Time = 0;					    //时-分-秒
u8	LCD_New_Order=FALSE;			        //接收到新的数据
u8  LCD_Data_Decode_Flag = FALSE;		    //
u8  LCD_Popup_Windows = 0x00;				//页面弹窗标识位
u8  LCD_Page_Refresh = FALSE;				//页面刷新标识位
u8  LCD_Display_ChineseString = FALSE;      //显示中文标识位
u8  LCD_Page_Cut = FALSE;					//显示剪切图标
u8  LCD_Hidden_Display = FALSE;			    //隐藏显示指针
u8  LCD_UnHidden_Display = FALSE;			//显示隐藏指针

u8  Dispaly_Order_Num = 0;		 //单次显示命令条数
u8  Dispaly_Order[20] = {0};	 //单次显示命令内容

u8  temp_data[26] = {0};
u8  Order_Temp[60] = {0};			 //指令缓存，用于串口发送
u8  IIC_Temp[60] = {0};			 //
static u8 gs_LCD_Pre_Page = 0;		 //关闭背光时记录之前界面参数
static u8  i = 0;

ChineseString Display_ChineseString = {0x0000,0,0};
//PageCut  Display_PageCut = {0x0000,0,0,0,0,0,0,0};
UnHidden UnHidden_Display = {0x0000,0x0000};
Hidden Hidden_Display = {0x0000};



/*----------------Start_Up--界面1操作标志位----------------*/
u32 Pass_Word[2] = {123456,111111};		         //开机密码:普通权限,管理员权限
u8  Refresh_Admin_Flag=FALSE;
u8  Admin_Authority=0;					   //0:普通权限 1:管理员权限

u8  Admin_Authority_Dis[2][10]={"普通权限",
                                "管理员权限"};
u8  Robot_Admin_Authority = ORDINARY_AUTHORITY;
u32 g_Pass_Word_Count = 0;
u32 g_Pass_Word_Delay = 1;
u8 g_Alarm_Refresh_Flag = 1;	     //报警显示
u8 g_Welcom_Flag = 0;	             //显示欢迎
u8 g_Robot_Enable_Flag = 0;	         //使能
u8 g_Emergency_Stop_Flag = 0;	     //急停
u8 g_Cycle_Run_Flag = 1;	 		 //1:循环  0：单步
u8 g_X_Position_Flag  = 0;	 		 //X位置刷新
u8 g_Z_Position_Flag = 0 ;	 		 //Z位置刷新
u8 g_Robot_Origin_Flag = 0 ;	 	 //回零
u8 g_Robot_Run_Flag = 0 ;			 //启动
u8 g_Robot_Pause_Flag = 0 ;			 //暂停
u8 g_Robot_Stop_Flag = 0 ;			 //停止
u8 g_Program_Refresh_Flag = 0 ;	     //显示程序刷新
u8 g_Run_Program_Num = 0;			 //当前运行的程序编号
u16 g_Run_Program_Operater_List = 0;	 //当前选择运行程序正在运行的指令条：由主控板返回
u8 g_Run_Step = 0;					 //从串口接收回来的数据，调试用！
u8 Auto_Run_Flag = FALSE;
u8 Auto_Run_Count = 0;
u16 g_Run_Program_List_Num = 0;	     //当前程序规模大小；
u8 g_Reflash_Progress_Num = FALSE;	 //刷新产量标志位
u8 g_Robot_Auto_Reset = FALSE;
u8 g_Robot_Auto_Reset_Scan = FALSE;
u8 Axis_Position_Scan = FALSE;		 //自动运行时，轴位置查询

u16 g_Run_Program_Operater_List1=0;//当前选择运行程序正在运行的指令条：由主控板返回
u8 Display_Flag=0;
/*---------------------------------------------------------*/


/*----------------Free_Program--界面2操作标志位------------*/
u8 g_Background_Color = 1;			      //编辑背景色
u8 gs_Background_Color= 0;
u8 gs_Current_Operate_List=1;	         //当前蓝色背景所在的指令编号
u8 gs_Current_Program_List=1;
/*---------------点选择界面-----------------------*/
//u8 g_Background_Color_PointEdit = 1;			      //编辑背景色
//u8 gs_Background_Color_PointEdit= 0;
//u8 gs_Current_Operate_List_PointEdit=1;				//点选择界面蓝色背景所在的点编号
/*---------------点编辑界面-------------------*/
u8 g_Background_Color_PointEdit = 1;			      //编辑背景色
u8 gs_Background_Color_PointEdit= 0;
u8 gs_Current_Operate_List_PointEdit=1;				//点选择界面蓝色背景所在的点编号
/*---------------程序编辑界面-------------------*/
u8 g_Background_Color_ProgramEdit = 1;			      //编辑背景色
u8 gs_Background_Color_ProgramEdit= 0;
u8 gs_Current_Operate_List_ProgramEdit=1;				//点选择界面蓝色背景所在的点编号


u8 g_Background_Color_Blink = FALSE;  //背光闪烁标志位
u8 g_Background_Blink_Delay = FALSE;  //背景闪烁延时标志位
u8 g_Background_Blink_Count = 0    ;  //背景闪烁延时计数
u8 g_Event_Done_Flag = FALSE;         //当前事件是否处理-背光闪烁
u8 g_SaveProgram_Flag = 0;            //程序保存
u32 Axis_Parameter_Speed = 50;        //轴速度参数百分比
u32 Delay_Parameter_Time = 100;				  //延时时间参数（由默认0修改为10white_L）
u8 g_Current_Paramter_Num = 0;		    //参数设置行
u8 g_Current_Paramter_Key = 0;		    //参数设置指令编号
u8 g_Pre_SafeArea = 0;			          //参数设置时-保存当前选择安全区域
u8 g_FreeProgram_Setting_XZ = FALSE;  //自由编程X-Z轴设置参数标志位
u8 gs_FreeProgram_Setting_XZ=FALSE;
/*---------------------------------------------------------*/


/*----------------IO_Control--界面3操作标志位--------------*/
u8 g_Watch_IO_Control = 0; //实时IO监视状态
u16 g_LocalIO_Status = 0;	 //本地IO状态
u16 g_Input1_Status  = 0;	 //输入1状态
u16 g_Input2_Status  = 0;	 //输入2状态
u16 g_Input3_Status  = 0;	 //输入3状态
u16 g_Output1_Status = 0;	 //输出1状态
u16 g_Output2_Status = 0;	 //输出2状态
u16 g_Output3_Status = 0;	 //输出3状态
u8  g_Output1Ack_Flag = FALSE;//输入状态通信应答成功标志位
u8  g_Output2Ack_Flag = FALSE;//输入状态通信应答成功标志位
u8  g_Output3Ack_Flag = FALSE;//输入状态通信应答成功标志位
/*---------------------------------------------------------*/


/*----------------Manul_Operate--界面4操作标志位------------*/
u8  g_ManulOperate_Flag = FALSE;
u8  g_X_Origin_Flag = FALSE ;			     //X轴回零
u8  g_Z_Origin_Flag = FALSE ;			     //z轴回零
u8  g_L_Origin_Flag = FALSE ;			     //z轴回零
u8  g_O_Origin_Flag = FALSE ;			     //z轴回零
u8  g_X_SavePoint_Flag = FALSE ;	         //X保存
u8  g_Z_SavePoint_Flag = FALSE ;	         //Z保存
u8  g_L_SavePoint_Flag = FALSE ;	         //L保存
u8  g_O_SavePoint_Flag = FALSE ;	         //O保存
u8  g_Current_Manul_Axis = NO_AXIS;			 //手动微调时当前选择轴
u16 g_Manule_Pulse = 0 ;					 //手动微调脉冲值
u16 gs_Manule_Pulse = 0 ;					 //手动微调脉冲值
/*---------------------------------------------------------*/


/*----------------SystemSetting--界面5操作标志位------------*/
u8  g_Buzzer_Switch_Flag = TRUE;
u32 g_BackLight_Time = 120;		   //背光时间
u32 g_BackLight_Count=0;		   //背光时间计数
u8  g_BackLight_Dark = TRUE;
//u8  g_X_SoftLimit_flag = FALSE; 
//u8  g_Z_SoftLimit_flag = FALSE;
//u8  g_L_SoftLimit_flag = FALSE;
u8  USBH_Process_Flag=FALSE;       //U盘页面标志位
u8  USBH_Direction=USBTOCONTROLER; //U盘和控制器拷贝方向

u8  USB_Update_Flag=0;             //---2018.5.28 U盘升级选择
u8  USB_Dispaly_Fresh_Flag=FALSE;  //---2018.5.28 U盘显示刷新标志位

u8  USBH_Dispaly_Fresh_Flag=0;
u8  USBH_And_Control[2][14]={
                             {"U盘-->控制器  "},
			           	     {"控制器-->U盘  "}
			                };
u8  USBH_CopyStatus_String[12][22]={
                                    {"      请插入U盘       "},
			           	            {"      检测到U盘       "},
									{"    程序拷贝中...     "},
									{" 拷贝已完成,请拔出U盘 "},
									{"   拷贝失败,请检查！  "},
									{"      U盘已拔出       "},
									{"   U盘错误,请检查！   "},
									{"  U盘写保护,请设置！  "},
									{"    未找到程序文件!   "},
									{"      U盘格式化...    "},
									{"     程序下发中...    "},
			                       };
u8 USB_Update_Choice[4][14]={//---2018.5.28
	                           {"              "},
														 {"              "},
                             {"手持器系统更新"},
			           	           {"控制器系统更新"}	
                            };
u8 USB_UpdateStatus_String[8][24]={//---2018.5.28
                                    {"       请插入U盘        "},
			           	                  {"       检测到U盘        "},
																		{"       U盘已拔出        "},
																		{"    U盘错误,请检查！    "},
																		{"   U盘写保护,请设置！   "},
																		{"     未找到程序文件!    "},
																		{"       U盘格式化...     "},
                                  };
u8 USB_Update_Status = USBSTATUS_NOUDISK;    //---2018.5.28 U盘升级状态
u8 USBH_Copy_Status = USBHSTATUS_NOUDISK;		 //U盘拷贝状态
u8 USB_Send_Program=FALSE;						 //U盘拷贝程序发送到主控制器
/*---------------------------------------------------------*/

/*************************************************************************
**  函数名：  LCDDisplayFuction()
**	输入参数：LCD显示功能实现函数
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void LCDDisplayFuction()
{
  /*-----------显示当前页面-------------LCD_Page_Refresh-----------*/
	if(LCD_Page_Refresh == TRUE)
	{
		PageChange(LCD_Current_Page);		//显示当前页	
		LCD_Page_Refresh = FALSE;
		LCD_Current_Page = 0;
	}

 /*-----------显示剪切图标-------------LCD_Page_Cut-----------*/
	if(LCD_Page_Cut)
	{
		
		//DisplayPageCut(Display_PageCut.Vary_addr,Display_PageCut.Pic_ID,Display_PageCut.Left_X,Display_PageCut.Up_Y,Display_PageCut.Right_X,Display_PageCut.Down_Y,Display_PageCut.New_X,Display_PageCut.New_Y);
		LCD_Page_Cut = FALSE;	
	}

 /*-----------显示中文字符-------------LCD_Display_ChineseString-----------*/
	if(LCD_Display_ChineseString)
	{
		DisplayChineseString(Display_ChineseString.addr,Display_ChineseString.data,Display_ChineseString.len);	
		LCD_Display_ChineseString = FALSE;
	}

 /*-----------显示弹窗-------------LCD_Popup_Windows-----------*/
	if(LCD_Popup_Windows)
	{
		Popup_Windows(LCD_Popup_Windows);
		LCD_Popup_Windows = 0;	
	}

 /*-----------隐藏显示内容-------------LCD_Hidden_Display-----------*/
	if(LCD_Hidden_Display)
	{
		HiddenDisplay(Hidden_Display.Des_Point);
		LCD_Hidden_Display = FALSE;	
	}	
	
	
 /*-----------显示隐藏内容-------------LCD_UnHidden_Display-----------*/
	if(LCD_UnHidden_Display)
	{
		UnHiddenDisplay(UnHidden_Display.Des_Point,UnHidden_Display.Vary_addr);
		LCD_UnHidden_Display = FALSE;	
	}
}
		


/*************************************************************************
**  函数名：  LCDDisplay()
**	输入参数：LCD显示处理程序
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void LCDDisplay()
{
	if((Landing_Success)&&(g_BackLight_Dark))
	{
		if(LCD_Current_Page!=85)
		{
			gs_LCD_Pre_Page  = 	LCD_Current_Page;
			LCD_Current_Page = 	85;
		}
	}
	switch(LCD_Main_Page)
	{
	 	case START_UP:	
		     LCDStartUpDisply();	 
	 	 	 break;
	 	case FREE_PROGRAMME:
//		     LCDFreeProgramDisply();
			LCDProductManagementDisply();
	 	 	 break;	
	 	case IO_CONTROL:
			 LCDIOControlDisply();
	 	 	 break;	
	 	case MANUL_DEBUG:
	 	 	 LCDManulOperateDisply();
			 break;	
	 	case SYSTEM_SETTING:
	 	 	 LCDSystemSettingDisply();
			 break;
	 	case ALARM_HISTORY:
	 	 	 LCDAlarmHistoryDisply();
			 break;
		default:
			 break;	  			  	
	}
	/*-----------显示弹窗-------------LCD_Popup_Windows-----------*/
	if(LCD_Popup_Windows)
	{
		Popup_Windows(LCD_Popup_Windows);
		LCD_Popup_Windows = 0;	
	}

}


/*************************************************************************
**  函数名：  LCDProcess()
**	输入参数：LCD触摸按键处理程序
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
extern u8 Program_Or_Point;
extern u8 Point_Num,LC_Empty_Flag;
void LCDProcess(void)
{
		if(LCD_Current_Page==116&&LC_Empty_Flag)//更换料仓报警
		{
	       g_Alarm_Buzzer_Count++;
		   if(g_Alarm_Buzzer_Count>=1000)
		   {
			g_Alarm_Buzzer_Count=0;
			BuzzerRing(10);
		   }
		}
		if(LCD_New_Order)
		{
			if(g_Buzzer_Switch_Flag==TRUE)
			{
				if(g_Manul_Link_Flag)
				{
				}
				else
				{
					BuzzerRing(10);
				}
			}
			//g_Manul_Link_Flag = FALSE;
			//背光控制
			g_BackLight_Count=0;
			if(LCD_Var_Address == 0x1EE0)
			{
				LCD_Current_Page = gs_LCD_Pre_Page;
				BackLight(0x40);
				g_BackLight_Dark = FALSE;
				LCD_Var_Address = 0;
			}
			if((LCD_Var_Address != 0) &&(g_Program_Is_Debuging==FALSE))
			{
				if(g_Robot_Error_Flag==FALSE)  //报警之后无下发指令！此处需测试- 测试可行20161015
				{
				//if(Robot_Error_Data[1]==0x00 && Robot_Error_Data[2]==0x00 && Robot_Error_Data[3]==0x00 && Robot_Error_Data[4]==0x00)  //无机械手错误发生
				if(Robot_Error_Data[1]==0x00)
				{
					if((Landing_Success==FALSE) || (Change_User==TRUE)) //未成功登录
					{
						LandingProcess();
					}
					else if(Landing_Success==TRUE)	  //已成功登录
					{
							switch((LCD_Var_Address>>12)&0x000f)
							{
								case 1:	LCDStartUpProcess();						//0x1000			
												break;	 
								case 2:	LCDFreeProgramProcess();			  //0x2000
												break;
								case 3:	LCDIOControlProcess();          //0x3000
												break;
								case 4:	LCDManulOperateProcess();	      //0x4000手动调试有回零操														
										break;
								case 5:	LCDSystemSettingProcess();		  //0x5000
												break;
//								case 6:	LCDAlarmHistoryProcess();       //0x6000
//												break;
								default:
												break;	
							}
					}
				}//if()  //无机械手错误发生
				}//if(g_Robot_Error_Flag==FALSE)
			}//if(LCD_Var_Address != 0)

			for(i=0;i<20;i++)
			{
				LCDReceiveDataBuffer[i]=0;
			}
			LCD_Var_Address = 0;
			LCD_New_Order = FALSE;
		}//if(LCD_New_Order)
    //LCD_New_Order=FALSE
	  else 
		{
			if((LCD_Current_Page==14||LCD_Current_Page==91||LCD_Current_Page==67 ||LCD_Current_Page==69
		       ||LCD_Current_Page==98||LCD_Current_Page==99||LCD_Current_Page==113||LCD_Current_Page==121)&&(Robot_Status == ACTION_STOP ))
		  {
				//读取使能信号 PC1
				if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1))  //低电平 按下脉冲电位器开关按钮
				{
					DelayNms(2);
					if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1))  //低电平 按下脉冲电位器开关按钮
					{
						if(LCD_Current_Page==14 || LCD_Current_Page==98)
						{
							g_Current_Manul_Axis=X_AXIS;	
						}
						else if(LCD_Current_Page==91 || LCD_Current_Page==99 )
						{
							g_Current_Manul_Axis=Z_AXIS;
						}	
						else if(LCD_Current_Page==67 || LCD_Current_Page==113 )
						{
							g_Current_Manul_Axis=L_AXIS;
						}
						else 
						{
							g_Current_Manul_Axis=O_AXIS;
						}
						TIM_Cmd(TIM3, ENABLE);//使能定时器  
						TIM3->CNT =0x7FFF;
						g_Manule_Pulse=0x7FFF;
						gs_Manule_Pulse=0x7FFF;
						//g_Manule_Pulse=TIM_GetCounter(TIM3); //读取脉冲
						Order_Temp[0] = MANUL_FINE_ADJUST;
						Order_Temp[1] = 1;
						USART1_SendData(2,MANUL_OPERATE,&Order_Temp[0]);
						LCD_Current_Page=109;	//跳转到脉冲电子齿轮界面
						Manual_Pulse_Watch=FALSE;
						Manual_Pulse_Count=0;
					}
				}	
		 }
	 }			
	/*
	//伺服器使能-失能
	if(Robot_Enable_Switch_On!=Robot_Enable_Pre_Switch)
	{
		if((Robot_Enable_Switch_On==TRUE) && (Robot_Enable==FALSE))
		{
		    Order_Temp[0]=ROBOT_ENABLE;
		    Order_Temp[1]=1;
		    USART1_SendData(2,ROBOT_ENABLE_A_ORIGIN,&Order_Temp[0]);
			Robot_Enable_Pre_Switch = Robot_Enable_Switch_On;
		}
		//伺服器失能
		else
		{    
			if(LCD_Current_Page != START_UP)
			{
				//左上角状态栏，产生报警信息：失能无效，请在首页进行失能操作

				Robot_Error_Data[0] = Robot_Error_Data[0]|0x0040;	//失能错误
//				LCD_Popup_Windows = PW_1_NOINFISRTPAGE;	//弹出提示框-只有在首界面才能失能机械手
//				LCD_Main_Page = START_UP;
//				LCD_Current_Page = START_UP;
			}
			else if(Robot_Status != ACTION_STOP)
			{
				LCD_Popup_Windows = PW_1_STOPROBOT;	//弹出提示框-请先停止机械手
			}
		    else if(Robot_Status == ACTION_STOP )
		    { 
			    Order_Temp[0]=ROBOT_ENABLE;
			    Order_Temp[1]=0;
			    USART1_SendData(2,ROBOT_ENABLE_A_ORIGIN,&Order_Temp[0]);
				Robot_Enable_Pre_Switch = Robot_Enable_Switch_On;				
		    }
		}		
	}
	*/
}

/*************************************************************************
**  函数名：  BuzzerRing()
**	输入参数：蜂鸣器鸣叫
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/7/10
**************************************************************************/
void BuzzerRing(u8 time)
{
   temp_data[0] = 0xA5 ;
   temp_data[1] = 0x5A ;
   temp_data[2] = 0x03 ;
   temp_data[3] = 0x80 ;
   temp_data[4] = 0x02 ;
   temp_data[5] = time;
   USART3_SendData(temp_data,6);   
}

/*************************************************************************
**  函数名：  BackLight()
**	输入参数：背光时间
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/7/10
**************************************************************************/
void BackLight(u8 Light)
{
   temp_data[0] = 0xA5 ;
   temp_data[1] = 0x5A ;
   temp_data[2] = 0x03 ;
   temp_data[3] = 0x80 ;
   temp_data[4] = 0x01 ;
   temp_data[5] = Light;
   USART3_SendData(temp_data,6);   
}


/*************************************************************************
**  函数名：  ReadRTC()
**	输入参数：读取RTC
**	输出参数：无
**	函数功能：
**	备注：	  返回数据：A5 5A 0A 81 20 07 96 01 01 05 00 00 52
**  作者：    wx   
**  开发日期：2015/7/10
**************************************************************************/
void ReadRTC()
{
   temp_data[0] = 0xA5 ;
   temp_data[1] = 0x5A ;
   temp_data[2] = 0x03 ;
   temp_data[3] = 0x81 ;
   temp_data[4] = 0x20 ;
   temp_data[5] = 0x07;
   USART3_SendData(temp_data,6);   
}

/*************************************************************************
**  函数名：  PageChange()
**	输入参数：LCD界面切换程序
**	输出参数：无
**	函数功能：判断触控指令属于哪个主界面
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void PageChange(u8 PageID)
{
   temp_data[0] = 0xA5 ;
   temp_data[1] = 0x5A ;
   temp_data[2] = 0x04 ;
   temp_data[3] = 0x80 ;
   temp_data[4] = 0x03 ;
   temp_data[5] = 0x00 ;
   temp_data[6] = PageID;
   USART3_SendData(temp_data,7);   
}

/*************************************************************************
**  函数名：  Popup_Windows()
**	输入参数：LCD界面切换程序
**	输出参数：无
**	函数功能：判断触控指令属于哪个主界面
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void Popup_Windows(u8 KeyVal)
{
    temp_data[0] = 0xA5 ;
    temp_data[1] = 0x5A ;
    temp_data[2] = 0x03 ;
    temp_data[3] = 0x80 ;
	  temp_data[4] = 0x4F;
	  temp_data[5] = KeyVal&0xff;
    USART3_SendData(temp_data,6);//----?????????????????????
}


/*************************************************************************
//函数名称：Display_ChineseString
//函数功能：写文本到DGUS屏上
//参数说明：
//addr ：屏中的显示地址；data：数据所在指针；
//作者：我不怕
//时间：9月20日
//备注：这个弹出就是弹出菜单的指令控制方式
**************************************************************************/
void DisplayChineseString(u16 addr,u8 *data,u8 len)
{

	temp_data[0] = 0xA5;
	temp_data[1] = 0x5A;
	temp_data[2] = (u8)(len+3);//发送的数据长度
	temp_data[3] = 0x82;
	temp_data[4] = (u8)((addr>>8)&0xff);
	temp_data[5] = (u8)( addr&0xff);
	USART3_SendData(temp_data,6); 
	for(i=0;i<len;i++)
	{			
		USART3->DR = data[i];
		while((USART3->SR&0X40)==0);	
	}	
}


/******************************图片区域、剪切 粘贴*********
 Pic_addr:基本图形显示设置的地址   ID addr：剪切的来源图片ID号   
 row1，column1：左上角坐标 (剪切的区域左上角)
 row2，column2: 右下角坐标 (剪切的区域右下角)
 row3，column3：粘贴到目的地址时的左上角坐标
***************************************************/
//作者：我不怕
//时间：9月20日
//备注：此函数仅仅是将图片剪切放到某一页，不能保留剪切图片上所带有的跳转属性
void DisplayPageCut(u16 Vary_addr,u16 Pic_ID,u16 row1,u16 column1,u16 row2,u16 column2,u16 row3,u16 column3)
{
	temp_data[0] = 0xA5 ;
	temp_data[1] = 0x5A ;
	temp_data[2] = 0x17 ;
	temp_data[3] = 0x82 ;

	temp_data[4]=(u8)(Vary_addr>>8);
	temp_data[5]=(u8)(Vary_addr&0xff);

	temp_data[6] = 0x00 ;
	temp_data[7] = 0x06 ;
	temp_data[8] = 0x00 ;
	temp_data[9] = 0x01 ;

	temp_data[10]=(u8)(Pic_ID>>8);
	temp_data[11]=(u8)(Pic_ID&0xff);

	temp_data[12]=(u8)(row1>>8);
	temp_data[13]=(u8)(row1&0xff);
	temp_data[14]=(u8)(column1>>8);
	temp_data[15]=(u8)(column1&0xff);
	
	temp_data[16]=(u8)(row2>>8);
	temp_data[17]=(u8)(row2&0xff);
	temp_data[18]=(u8)(column2>>8);
	temp_data[19]=(u8)(column2&0xff);

	temp_data[20]=(u8)(row3>>8);
	temp_data[21]=(u8)(row3&0xff);
	temp_data[22]=(u8)(column3>>8);
	temp_data[23]=(u8)(column3&0xff);

	temp_data[24] = 0xFF ;
	temp_data[25] = 0x00 ;
	USART3_SendData(temp_data,26);
}



void Display_PageCut(PageCut Icon)
{
	DisplayPageCut(Icon.Vary_addr,Icon.Pic_ID,Icon.Left_X,Icon.Up_Y,Icon.Right_X,Icon.Down_Y,Icon.New_X,Icon.New_Y);
}




//函数名称：WriteNum2
//函数功能：写2个字节的数值到DGUS屏上地址
//参数说明：
//addr ：屏中的地址；num：数据值；
//作者：我不怕
//时间：9月25日
//备注：
void WriteNum2(u16 addr,u16 num)
{
	temp_data[0] = 0xA5 ;
	temp_data[1] = 0x5A ;
	temp_data[2] = 0x05 ;
	temp_data[3] = 0x82 ;	
	temp_data[4]=(u8)((addr>>8)&0xff);
	temp_data[5]=(u8)( addr&0xff);
	temp_data[6]=(u8)((num>>8)&0xff);
	temp_data[7]=(u8)( num&0xff);
	USART3_SendData(temp_data,8); 	
}
//函数名称：WriteNum4
//函数功能：写4个字节的数值到DGUS屏上地址
//参数说明：
//addr ：屏中的地址；num：数据值；
//作者：我不怕
//时间：9月25日
//备注：
void WriteNum4(u16 addr,u32 num)
{
	temp_data[0] = 0xA5 ;
	temp_data[1] = 0x5A ;
	temp_data[2] = 0x07 ;
	temp_data[3] = 0x82 ;
	temp_data[4]=(u8)((addr>>8)&0xff);
	temp_data[5]=(u8)( addr&0xff);
	temp_data[6]=(u8)((num>>24)&0xff);
	temp_data[7]=(u8)((num>>16)&0xff);
	temp_data[8]=(u8)((num>>8)&0xff);
	temp_data[9]=(u8)( num&0xff);
	USART3_SendData(temp_data,10); 	
}

//函数名称：WriteNum8
//函数功能：写8个字节的数值到DGUS屏上地址
//参数说明：
//addr ：屏中的地址；num：数据值；
//作者：wx
//时间：
//备注：
//void WriteNum8(u16 addr,u64 num)
//{
//	temp_data[0] = 0xA5 ;
//	temp_data[1] = 0x5A ;
//	temp_data[2] = 0x07 ;
//	temp_data[3] = 0x82 ;
//	temp_data[4]=(u8)((addr>>8)&0xff);
//	temp_data[5]=(u8)( addr&0xff);
//	temp_data[6]=(u8)((num>>24)&0xff);
//	temp_data[7]=(u8)((num>>16)&0xff);
//	temp_data[8]=(u8)((num>>8)&0xff);
//	temp_data[9]=(u8)( num&0xff);
//	USART3_SendData(temp_data,10); 	
//}

/**********************************************************************************************************
**点动初始化0x05,0x50,0x06,0x83,0x00,0x00,0x01,0x00,0x01
**********************************************************************************************************/
void WriteNum9(u16 adr)
{
	u8 data[15]={0xA5,0x5A,0x0c,0x82,0x00,0x00,0xA5,0x5A,0x06,0x83,0x00,0x00,0x01,0x00,0x01};
	data[4]=(u8)((adr>>8)&0xff);
	data[5]=(u8)(adr&0xff);
	data[10]=(u8)((adr>>8)&0xff);
	data[11]=(u8)(adr&0xff);
	USART3_SendData(data,15); 	
}


//函数名称：HiddenDisplay
//函数功能：隐藏文本显示
//参数说明：
//point_addr ：隐藏文本的描述指针；
//作者：我不怕
//时间：9月28日
//备注：
void HiddenDisplay(u16 point_addr)
{
	temp_data[0] = 0xA5 ;
	temp_data[1] = 0x5A ;
	temp_data[2] = 0x05 ;
	temp_data[3] = 0x82 ;
	temp_data[4]=(u8)((point_addr>>8)&0xff);
	temp_data[5]=(u8)( point_addr&0xff);
	temp_data[6] = 0xFF ;
	temp_data[7] = 0x00 ;
	USART3_SendData(temp_data,8); 
}
//函数名称：UnHiddenDisplay
//函数功能：打开文本显示
//参数说明：
//point_addr ：隐藏文本的描述指针；add 文本显示的变量地址
//作者：我不怕
//时间：9月28日
//备注：
void UnHiddenDisplay(u16 point_addr,u16 addr)
{
	temp_data[0] = 0xA5 ;
	temp_data[1] = 0x5A ;
	temp_data[2] = 0x05 ;
	temp_data[3] = 0x82 ;
	temp_data[4]=(u8)((point_addr>>8)&0xff);
	temp_data[5]=(u8)( point_addr&0xff);
	temp_data[6]=(u8)((addr>>8)&0xff);
	temp_data[7]=(u8)( addr&0xff);
	USART3_SendData(temp_data,8); 
}


//函数名称：Read_Number
//函数功能：写4个字节的数值到DGUS屏上地址
//参数说明：
//addr ：屏中的地址；num：数据值；
//作者：我不怕
//时间：9月25日
//备注：
//void Read_Number(u16 addr,u32 num)
//{
//	temp_data[0] = 0x5A ;
//	temp_data[1] = 0xA5 ;
//	temp_data[2] = 0x07 ;
//	temp_data[3] = 0x82 ;
//	temp_data[4]=(u8)((addr>>8)&0xff);
//	temp_data[5]=(u8)( addr&0xff);
//	temp_data[6]=(u8)((num>>24)&0xff);
//	temp_data[7]=(u8)((num>>16)&0xff);
//	temp_data[8]=(u8)((num>>8)&0xff);
//	temp_data[9]=(u8)( num&0xff);
//	USART3_SendData(temp_data,10); 	
//}


