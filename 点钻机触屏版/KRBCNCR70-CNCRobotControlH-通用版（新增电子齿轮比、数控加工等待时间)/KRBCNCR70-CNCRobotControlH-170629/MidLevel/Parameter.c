  /*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : 
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 
* Description        : This is the ....
***************************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "Parameter.h"
#include "PlatformInit.h"
#include "Delay.h"
#include "24Cxx.h"
#include "DGUS.h"
#include "manuloperate.h"

u8  Robot_Status = ACTION_STOP;      //机械手状态
u8  Robot_Auto_Mode = CYCLE_MODE;	 //机械手自动运行模式：循环-单步

/*-----开机界面变量---StartUp-----*/
//u8  X_Axsis_Origining = FALSE;
//u8  Z_Axsis_Origining = FALSE;
u8  X_Axsis_Origin_Enable = FALSE;
u8  Z_Axsis_Origin_Enable = FALSE;
u8  X_Axsis_Origned	=FALSE;	   //X轴已回原点
u8  Z_Axsis_Origned	=FALSE;	   //Z轴已回原点
u8  Robot_Origining = FALSE;
u8  Robot_Origined = FALSE;	   //机械手回原点
u8  Robot_Enable = FALSE;      //机械手使能
u8  Robot_Enable_Switch_On = FALSE;	//机械手使能钥匙开关
u8  Robot_Enable_Pre_Switch = FALSE;

u8  Program_Ready = FALSE;		     //程序准备OK
u8  Data_Syc_Ready = FALSE;	       //是否读取开机程序
u8  Setting_Syc_Ready = FALSE;	   //开机读取设定参数完成
u8  Point_Syc_Ready = FALSE;	     //开机读取点完成
u8  ProgramInf_Syc_Ready = FALSE;	 //开机读取程序信息参数完成
u8  Program_Syc_Ready = FALSE;	   //开机读取程序完成
u8  SoftLimit_Syc_Ready = FALSE;   //开机读取软限位完成
u8  SafeArea_Syc_Ready = FALSE;    //开机读取安全区域完成
u8  IODetect_Syc_Ready = FALSE;    //开机读取IO检测完成
u8  Main_Init_Done = FALSE;		     //主控板开机完成
u32 g_XAxis_Position = 0;          		//X轴的位置
u32 g_ZAxis_Position = 0;		       //Z轴的位置
u32 g_LAxis_Position = 0;		       //L轴的位置
u32 g_OAxis_Position = 0;		       //O轴的位置
u32 g_AAxis_Position = 0;		       //V轴的位置
u32 g_BAxis_Position = 0;		       //W轴的位置

u32 g_Pre_XAxis_Position = 0;      //X轴之前时刻的位置
u32 g_Pre_ZAxis_Position = 0;	     //Z轴之前时刻的位置
u32 g_Pre_LAxis_Position = 0;	     //L轴之前时刻的位置
u32 g_Pre_OAxis_Position = 0;	     //O轴之前时刻的位置
u32 g_Pre_VAxis_Position = 0;	     //V轴之前时刻的位置
u32 g_Pre_WAxis_Position = 0;	     //W轴之前时刻的位置

u8 Axsis_Move_Direction[6]={1,1,1,1,1,1};//当前运动方向
u8  g_Servo_Is_Run = FALSE;	  
u8  g_Axis_Is_Run=0;					//   B  A  O Y Z X 机械手正在运行标志位，用于自动和手动时查询机械手实时位置
															//	 20 10 8 4 2 1
u8  g_Axis_Choose = NO_AXIS;
SavePoint Manul_Save_PointX[25]={0};	  //保存点的数组，Flag、Name、P-X，其中Flag表示是否有点保存
SavePoint Manul_Save_PointZ[25]={0};	  //保存点的数组，Flag、Name、P-Z，其中Flag表示是否有点保存
SavePoint Manul_Save_PointL[25]={0};	  //保存点的数组，Flag、Name、P-Z，其中Flag表示是否有点保存
SavePoint Manul_Save_PointO[25]={0};	  //保存点的数组，Flag、Name、P-Z，其中Flag表示是否有点保存
SaveProgram Current_Operate_Program={0};  //当前操作的程序
SaveProgramInf Free_Program_Save[10]={	  //保存程序的数组，Flag，Code，Num,其中Code表示序号，Num表示规模大小
                                    {0,1,0,0,0}, {0,2,0,0,0}, {0,3,0,0,0}, {0,4,0,0,0},
									{0,5,0,0,0}, {0,6,0,0,0}, {0,7,0,0,0}, {0,8,0,0,0},
									{0,9,0,0,0}, {0,10,0,0,0}
								  };
SaveProgram Free_Program_Operate={0};	  //当前选中运行的程序
/*- [15:8] 机械手典型报警                      -*/
/* 15:急停  14:安全区 13:软限位 12:产量完成 11:缺料 10:满料 9:动作超时  8:机床故障 */
/*- [7:0] 手持器操作报警                      -*/
/*  7:通信 6:失能  5:预留                    */
//u16 g_Robot_Error = 0;               //机械手错误标志位	  -当需要加入详细信号检测报警时,可将参数修改成数组

//Robot_Error_Data[0]示教器操作错误
//Robot_Error_Data[1-4]与控制板错误信息相对应
u8 Robot_Error_Data[5] = {0};
u8 g_Robot_Error_Flag=FALSE; 
u8 Robot_Axis_State[6]={0};//每个轴的当前运行状态 加速 匀速 减速

//--------加密锁相关参数-------//
u8  InitDataFlag = 0xa4;	//加密锁初始时间
u8  InitDataBuf[7]={0}; 	//初始数据缓存
u8  InitData[7]={0x00,0x0e,0x04,0x0b,0x00,0x00,0x00};//2014年4月11日0时0分0秒
u8  DateTime[7]={0};		//
u8  UnLock_Forever_flag=0;	//是否长期有效标志位
u16 Unlock_LastDay=0;		//解密时间
u8  Reset_Flag=0;
u32 System_time=0;			//系统时间
u8  Check_battery=0;        //
u32 TimesData[3]={0,0,0}; 
u32 EndTimesData[3]={0};
u32 TimesDataEE[3]={0x00140411,0x00140516,0x00140411};  //起始时间，截止时间，上次开机时间
u8  EE_Temp[4] = {0};
u32 MachineNum=0x12345678; 	//随机给定的机器码
u32 MachineQueueNum=0;		//由时间和机器码生成的机器编码
u8  Battery_Unset[]={":电池未安装!  "};
u8  Battery_black[]={"              "};
u8  Beyond_LastDay[]={": 超过使用期限,请联系厂家解密! "};
u8  Beyond_Black[]=  {"                               "};
u32 MachineCommenTemp=0;    //输入的解锁密码缓存   u32 Unlock_Code_Temp=0;
u32 MachineNumTemp=0;		//解密算法得到的数据
u32 TimeDataNum=0; 			//解密算法得到的数据
/***************2018.6.26 White_L*************************************/
//{0,1,0,0,0}, {0,2,0,0,0}, {0,3,0,0,0}, {0,4,0,0,0},{0,5,0,0,0}, {0,6,0,0,0}, {0,7,0,0,0}, {0,8,0,0,0},{0,9,0,0,0}, {0,10,0,0,0},
//{0,11,0,0,0}, {0,12,0,0,0}, {0,13,0,0,0}, {0,14,0,0,0},{0,15,0,0,0}, {0,16,0,0,0}, {0,17,0,0,0}, {0,18,0,0,0},{0,19,0,0,0}, {0,20,0,0,0},
//{0,21,0,0,0}, {0,22,0,0,0}, {0,23,0,0,0}, {0,24,0,0,0},{0,25,0,0,0}, {0,26,0,0,0}, {0,27,0,0,0}, {0,28,0,0,0},{0,29,0,0,0}, {0,30,0,0,0},
//{0,31,0,0,0}, {0,32,0,0,0}, {0,33,0,0,0}, {0,34,0,0,0},{0,35,0,0,0}, {0,36,0,0,0}, {0,37,0,0,0}, {0,38,0,0,0},{0,39,0,0,0}, {0,40,0,0,0},
//{0,41,0,0,0}, {0,42,0,0,0}, {0,43,0,0,0}, {0,44,0,0,0},{0,45,0,0,0}, {0,46,0,0,0}, {0,47,0,0,0}, {0,48,0,0,0},{0,49,0,0,0}, {0,50,0,0,0},
//{0,51,0,0,0}, {0,52,0,0,0}, {0,53,0,0,0}, {0,54,0,0,0},{0,55,0,0,0}, {0,56,0,0,0}, {0,57,0,0,0}, {0,58,0,0,0},{0,59,0,0,0}, {0,60,0,0,0},
//{0,61,0,0,0}, {0,62,0,0,0}, {0,63,0,0,0}, {0,64,0,0,0},{0,65,0,0,0}, {0,66,0,0,0}, {0,67,0,0,0}, {0,68,0,0,0},{0,69,0,0,0}, {0,70,0,0,0},
//{0,71,0,0,0}, {0,72,0,0,0}, {0,73,0,0,0}, {0,74,0,0,0},{0,75,0,0,0}, {0,76,0,0,0}, {0,77,0,0,0}, {0,78,0,0,0},{0,79,0,0,0}, {0,80,0,0,0},
//{0,81,0,0,0}, {0,82,0,0,0}, {0,83,0,0,0}, {0,84,0,0,0},{0,85,0,0,0}, {0,86,0,0,0}, {0,87,0,0,0}, {0,88,0,0,0},{0,89,0,0,0}, {0,90,0,0,0},
//{0,91,0,0,0}, {0,92,0,0,0}, {0,93,0,0,0}, {0,94,0,0,0},{0,95,0,0,0}, {0,96,0,0,0}, {0,97,0,0,0}, {0,98,0,0,0},{0,99,0,0,0}, {0,100,0,0,0},
//{0,101,0,0,0}, {0,102,0,0,0}, {0,103,0,0,0}, {0,104,0,0,0}
SavedProgramInf Saved_Program[SAVEPROGRAMNUM]={0};
Save_Program OperateProgram[SAVEPROGRAMNUM] = {0};  //新版本存程序
u8 Product_Num=0,Program_Flag[SAVEPROGRAMNUM]={0};
extern u32 gs_Run_Program_Name,g_Run_Program_Name;
extern u16 Current_Select_Program;
extern u16 Current_Program_Adress;
extern u8 Select_Program_Flag;
extern u16 Order_Num,Point_Num,Read_Program_Count;
extern u16 Rod_X_Positive, Rod_X_Negative,Rod_Y_Positive,Rod_Y_Negative,Rod_Centry_X,Rod_Centry_Y;
extern u16 Rod_Step_X[8],Rod_Step_Y[8];	   
/*************************************************************************
**  函数名：  WriteTimeData()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：      
**  开发日期：
**************************************************************************/
u8 WriteTimeData(u8 *buf)
{ 
  u8 year,moth,day,hour,min,sec;
  u32 temp;
  year=*buf++;
  year=*buf++;
  moth=*buf++;
  day=*buf++;
  hour=*buf++;
  min=*buf++;
  sec=*buf;
  temp = year*360*24*3600+(moth-1)*30*24*3600+(day-1)*24*3600+hour*3600+min*60+sec;//获得设置的值
  RTC_Configuration();
 /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask(); 
  BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);  
  /* Change the current time */
  RTC_SetCounter(temp);	//  从9时22分11秒开始计时	 14年3月5日
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask(); 
  return 1; 
}

/*************************************************************************
**  函数名：  ReadTimeData()
**	输入参数：
**	输出参数：无
**	函数功能：设置加密锁初始时间
**	备注：	  无
**  作者：      
**  开发日期：
**************************************************************************/
u8 ReadTimeData(u8 *buf)		 //写在datatime数组  
{   
     u8 year,month,day;
     u32 Time_Value;
     u16 Day_Value;
     
	 Time_Value = RTC_GetCounter();//读取计数器的值
     
	 Day_Value = Time_Value/(24*3600);//得到天数
     year = Day_Value/360;//得到年
     month = (Day_Value - year*360)/30;//得到月
     day = (Day_Value - year*360)%30;//得到日
	 if(day == 30)
	 {
     	day = 1;
		month += 1;
	 }
	 else
	 {
	 	day += 1;
	 } 
     if(month==12)
	 {
     	month = 1;
		year += 1;
	 }
	 else
	 {
	 	month += 1;
	 } 
     buf[2] =  (Time_Value - Day_Value *24*3600)/3600;//得到小时      
     buf[1] =   (Time_Value - Day_Value *24*3600 - buf[2]*3600)/60;//得到分钟
	 buf[0] =    Time_Value - Day_Value *24*3600 - buf[2]*3600 - buf[1]*60;//得到秒钟 
	 buf[3]=day;
	 buf[5]=month;
	 buf[6]=year;
	 return 1;
}

/*************************************************************************
**  函数名：  Change_BCD()
**	输入参数：
**	输出参数：无
**	函数功能：BCD转十进制
**	备注：	  无
**  作者：      
**  开发日期：
**************************************************************************/
u8 Change_BCD(u8 j)
{ 
	 u8 m;
	 m=j/10;
	 return(m*6+j);
}


u8 BCD_Changeto_D(u8 j)
{ 
	 u8 a,b;
	 a=j/16;
	 b=j%16;
	 return(a*10+b);
}

/*************************************************************************
**  函数名：  TimeProcess()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：      
**  开发日期：
**************************************************************************/
void TimeProcess(unsigned char datetime[],unsigned int timedatanum)
{
        unsigned int time[3];
        time[0]=Change_BCD(datetime[3]); //day
        time[1]=Change_BCD(datetime[5]); //month
        time[2]=Change_BCD(datetime[6]); //year
        time[0]=(time[0]>>4)*10+(time[0]&0x0f);
        time[1]=(time[1]>>4)*10+(time[1]&0x0f);
        time[2]=(time[2]>>4)*10+(time[2]&0x0f);
        
        time[0]=time[2]*365+(time[1]-1)*30+time[0]+timedatanum;
        time[2]=time[0]/365;        
        time[1]=(time[0]%365)/30+1;
        if(time[1]>12)time[1]=12;
        time[0]=(time[0]%365)%30; 
        if(time[1]==2&&time[0]>28)time[0]=time[0]-28;     
         
        EndTimesData[0]=((time[0]/10)<<4)+(time[0]%10);  //day
        EndTimesData[1]=((time[1]/10)<<4)+(time[1]%10);  //month
        EndTimesData[2]=((time[2]/10)<<4)+(time[2]%10);  //year
}


/*************************************************************************
**  函数名：  InitLockTime()
**	输入参数：
**	输出参数：无
**	函数功能：设置加密锁初始时间
**	备注：	  无
**  作者：     
**  开发日期：
**************************************************************************/
void InitLockTime(void)
{
    u8 i,result=0;
    if(InitDataFlag == 0xa4) 
    {
        for(i=0;i<7;i++)
        {
            InitDataBuf[i]=InitData[i];
        } 
        result = WriteTimeData(InitDataBuf); 
		DelayNms(10);
		ReadTimeData(DateTime);
        DelayNms(10);
        if(result)
        {
            InitDataFlag++; 
			AT24CXX_Write(0x0100,&InitDataFlag,1);                          
        }
        else
        {
            while(1);
        }     
    }
}


//*************时间转换函数*************** flag=0:BCD到正常数据，flag=1:正常数据到BCD，
u32 TimeTransform(u32 timedatanum,u8 flag)
{
        u32 time[3];
        if(flag==0)
        {
                time[0]=(timedatanum>>16)&0x00ff; //year
                time[1]=(timedatanum>>8)&0x00ff;  //month
                time[2]=timedatanum&0x00ff;       //day 
                
                time[0]=((time[0]>>4)&0x0f)*10+(time[0]&0x0f);//year
                time[1]=((time[1]>>4)&0x0f)*10+(time[1]&0x0f);//month
                time[2]=((time[2]>>4)&0x0f)*10+(time[2]&0x0f);//day 
                
                time[2]=time[0]*365+(time[1]-1)*30+time[2];
                return time[2];
        }
        else
        {   
                time[0]=timedatanum/365; 
                time[1]=timedatanum%365/30+1; 
                if(time[1]>12)time[1]=12;
                time[2]=(timedatanum%365)%30;
                if(time[1]==2&&time[2]>28)time[2]=time[2]-28;  
                
                time[0]=((time[0]/10)<<4)|(time[0]%10); //year
                time[1]=((time[1]/10)<<4)|(time[1]%10);   //month
                time[2]=((time[2]/10)<<4)|(time[2]%10);        //day  
                
                time[0]<<=8;
                time[0]|=time[1];
                time[0]<<=8;
                time[0]|=time[2]; 
                return time[0];      
        }       
}


/*************************************************************************
**  函数名：  InitLockTime()
**	输入参数：
**	输出参数：无
**	函数功能：设置加密锁初始时间
**	备注：	  无
**  作者：      
**  开发日期：
**************************************************************************/
void CheckTime(void)
{
    u8 result=0;
    u16 time[2]={0,0}; 
    while(result==0)
    {
        result=ReadTimeData(DateTime);
        DelayNms(10);
    }
    System_time=0;
    System_time<<=8;
    System_time|=Change_BCD(DateTime[6]);	   //年
    System_time<<=8;
    System_time|=Change_BCD(DateTime[5]);	   //月
    System_time<<=8;
    System_time|=Change_BCD(DateTime[3]);   		//日

	if(Check_battery==1)	//电池被拔掉
    {  
		LCD_Current_Page = 105;
		LCD_Pre_Page = LCD_Current_Page;
		PageChange(LCD_Pre_Page);
		//PageChange(105);    //显示电池未安装提示界面
		Check_battery=0;
        if(TimesData[2]<TimesData[1])
        {
                TimesData[2]=TimeTransform(TimesData[2],0)+1;  //电池拔掉,扣除1*2天
                TimesData[2]=TimeTransform(TimesData[2],1);
                TimesDataEE[2]=TimesData[2];
				EE_Temp[0] = TimesDataEE[2];
				EE_Temp[1] = TimesDataEE[2]>>8;
				EE_Temp[2] = TimesDataEE[2]>>16;
				EE_Temp[3] = TimesDataEE[2]>>24;               
				AT24CXX_Write(0x0118,EE_Temp,4);
                
				InitData[1]=(TimesData[2]>>16)&0x00ff; //year
				InitData[1]=BCD_Changeto_D(InitData[1]);
				AT24CXX_Write(0x0102,&InitData[1],1);

                InitData[2]=(TimesData[2]>>8)&0x00ff;  //month
				InitData[2]=BCD_Changeto_D(InitData[2]);
				AT24CXX_Write(0x0103,&InitData[2],1);

                InitData[3]=TimesData[2]&0x00ff;       //day 
				InitData[3]=BCD_Changeto_D(InitData[3]);
				AT24CXX_Write(0x0104,&InitData[3],1);
        }
        InitDataFlag = 0xa4;
		//Write_one_byte(InitDataFlag, 26);
		AT24CXX_Write(0x0100,&InitDataFlag,1);
        InitLockTime();
		//DispMode=LCD_LOCKALARM;  提示电池被拔掉
        Reset_Flag=0;
    }
    else if((TimesData[2]>System_time)||(System_time>=TimesData[1])) 
    {         
        //当前时间比上次读出时间更早,错误  生成密钥并显示

		//--显示刷新-----------------------------------------//
		 LCD_Current_Page = 104;
		 LCD_Pre_Page = LCD_Current_Page;
		 PageChange(LCD_Pre_Page);		 
		 //PageChange(104);    //显示解锁界面		 
		 DisplayChineseString(0x5540,Beyond_LastDay,30);  	 //显示已超期
         Reset_Flag=0;   
    } 
    else
    {
        TimesData[2]=System_time;
        TimesDataEE[2]=TimesData[2];
		EE_Temp[0] = TimesDataEE[2];
		EE_Temp[1] = TimesDataEE[2]>>8;
		EE_Temp[2] = TimesDataEE[2]>>16;
		EE_Temp[3] = TimesDataEE[2]>>24;               
		AT24CXX_Write(0x0118,EE_Temp,4);
		DisplayChineseString(0x5540,Beyond_Black,30);  	 //显示空格
        Reset_Flag=1;
    }
    time[0]=TimeTransform(TimesData[2],0);
    time[1]=TimeTransform(TimesData[1],0);
    Unlock_LastDay=time[1]-time[0];
	
	//--根据锁定时间和机器码产生密钥MachineQueueNum -//
     MachineQueueNum=0;
     MachineQueueNum=MachineQueueNum<<8;
     MachineQueueNum=MachineQueueNum|DateTime[6];
     MachineQueueNum=MachineQueueNum<<8;
     MachineQueueNum=MachineQueueNum|DateTime[1];
     MachineQueueNum=MachineQueueNum<<8;
     MachineQueueNum=MachineQueueNum|DateTime[0];
     MachineQueueNum=MachineQueueNum^MachineNum;
	 WriteNum4(0x5510,MachineQueueNum);

} 


void  Decrypt(void)
{
    u8 i=8;
    u16 sum=0xF1B8,d=0x9E37;
    u16 y,z;
    u16 a,b;
    z=(u16)(MachineCommenTemp&0x0000ffff);
    y=(u16)(MachineCommenTemp>>16);
    a=1234;b=5678;
    while ( i-- > 0 )
    {
        z -= ( y << 4 ) + a ^ y + sum ^ ( y >> 5 ) + b;
        y -= ( z << 4 ) + a ^ z + sum ^ ( z >> 5 ) + b;
        sum -= d;
    } 
    MachineNumTemp=y;
    TimeDataNum=z;
   
}

/*************************************************************************
**  函数名：  Unlock_Code_Input()
**	输入参数：
**	输出参数：无
**	函数功能：解锁密码输入
**	备注：	  无
**  作者：      
**  开发日期：
**************************************************************************/
void UnlockCodeInput()
{
	if(LCD_Var_Address == 0x5520)	//点击ok
	{
		MachineCommenTemp = LCD_Var_Data;
		Decrypt();	
		LCD_Var_Address=0;	
		LCD_New_Order = FALSE;
	}
	if((MachineCommenTemp!=0) && (LCD_Var_Address == 0x5530))	//点击确定
	{
		  if(MachineNumTemp==(MachineQueueNum%10000))
	      {
	            TimesData[0]=0;
	            TimesData[0]<<=8;
	            TimesData[0]|=Change_BCD(DateTime[6]);
	            TimesData[0]<<=8;
	            TimesData[0]|=Change_BCD(DateTime[5]);
	            TimesData[0]<<=8;
	            TimesData[0]|=Change_BCD(DateTime[3]);  
	            TimeProcess(DateTime,TimeDataNum);
	            TimesData[1]=0;
	            TimesData[1]<<=8;
	            TimesData[1]|=EndTimesData[2];
	            TimesData[1]<<=8;
	            TimesData[1]|=EndTimesData[1];
	            TimesData[1]<<=8;
	            TimesData[1]|=EndTimesData[0];
	            TimesDataEE[0]=TimesData[0];

				EE_Temp[0] = TimesDataEE[0];
				EE_Temp[1] = TimesDataEE[0]>>8;
				EE_Temp[2] = TimesDataEE[0]>>16;
				EE_Temp[3] = TimesDataEE[0]>>24;               
				AT24CXX_Write(0x0110,EE_Temp,4);

	            TimesDataEE[1]=TimesData[1];
				EE_Temp[0] = TimesDataEE[1];
				EE_Temp[1] = TimesDataEE[1]>>8;
				EE_Temp[2] = TimesDataEE[1]>>16;
				EE_Temp[3] = TimesDataEE[1]>>24;               
				AT24CXX_Write(0x0114,EE_Temp,4);
    
	            if(TimeDataNum>=999)
				{
				   UnLock_Forever_flag=1;
				   AT24CXX_Write(0x010A,&UnLock_Forever_flag,1);
				}  //长期有效 
	            else 
				{
					UnLock_Forever_flag=0;
					AT24CXX_Write(0x010A,&UnLock_Forever_flag,1);
				}
				if(LCD_Main_Page == SYSTEM_SETTING)
				{
					LCD_Current_Page = 49;
					LCD_Pre_Page = LCD_Current_Page;
					PageChange(LCD_Pre_Page);
					//PageChange(49);    //切换到设置界面
				}
				else
				{
					LCD_Current_Page = 0;
					LCD_Pre_Page = LCD_Current_Page;
					PageChange(LCD_Pre_Page);
					//PageChange(0);    //切换到主界面
				}
				WriteNum4(0x5520,0);
				DisplayChineseString(0x5540,Beyond_Black,30);  	 //显示空格
				Reset_Flag=1;
	      }
	      else	//解锁密码错误-弹出报警窗
		  {
		  		Popup_Windows(PW_5_UNLOCKCODEEROOR);  //0x65
				Reset_Flag=0;
		  }	
		  LCD_Var_Address=0;
		  LCD_New_Order = FALSE;
	}
	if((Reset_Flag==0) && (LCD_New_Order==TRUE))
	{
		LCD_New_Order=FALSE;
	}
}

/*************************************************************************
**  函数名：  ReadIICInitPar()
**	输入参数：
**	输出参数：无
**	函数功能：开机初始IIC
**	备注：	  无
**  作者：      
**  开发日期：
**************************************************************************/
void ReadIICInitPar()
{
//	u8 Data_Temp_P[100]={0};
//	AT24CXX_Write(0x0260,Data_Temp_P,100);

	//读取IIC参数,起始地址0x0000;普通密码:0x0000; 管理员密码:0x0004;背光0x0008;	
	AT24CXX_Read(0x0000, IIC_Temp, 12);
	Pass_Word[0] = (u32)(((u32)IIC_Temp[0])|((u32)IIC_Temp[1]<<8)|((u32)IIC_Temp[2]<<16)|((u32)IIC_Temp[3])<<24);
	Pass_Word[1] = (u32)(((u32)IIC_Temp[4])|((u32)IIC_Temp[5]<<8)|((u32)IIC_Temp[6]<<16)|((u32)IIC_Temp[7])<<24);
	g_BackLight_Time = (u32)(((u32)IIC_Temp[8])|((u32)IIC_Temp[9]<<8)|((u32)IIC_Temp[10]<<16)|((u32)IIC_Temp[11])<<24);
	if(g_BackLight_Time<10)
	{
		g_BackLight_Time = 120;
	}
//	Pass_Word[0] = 123456;
//	Pass_Word[1] = 111111;

	/*----第一次烧写IIC时启用----*/	
//	IIC_Temp[0]=0xa4;
//	AT24CXX_Write(0x0100,IIC_Temp,1);
//	IIC_Temp[0]=0x0e;
//	IIC_Temp[1]=0x04;
//	IIC_Temp[2]=0x0b;
//	AT24CXX_Write(0x0102,IIC_Temp,3);
//	Data_Temp_P[0]=0x11;
//	Data_Temp_P[1]=0x04;
//	Data_Temp_P[2]=0x14;
//	Data_Temp_P[3]=0x00;
//	AT24CXX_Write(0x0110,Data_Temp_P,4);
//	Data_Temp_P[0]=0x16;
//	Data_Temp_P[1]=0x05;
//	Data_Temp_P[2]=0x14;
//	Data_Temp_P[3]=0x00;
//	AT24CXX_Write(0x0114,Data_Temp_P,4);
//	Data_Temp_P[0]=0x11;
//	Data_Temp_P[1]=0x04;
//	Data_Temp_P[2]=0x14;
//	Data_Temp_P[3]=0x00;
//	AT24CXX_Write(0x0118,Data_Temp_P,4);  	
   /*----第一次烧写IIC时启用----*/

	//读取加密锁相关参数,起始地址0x0100；
	AT24CXX_Read(0x0100, IIC_Temp, 8);
	InitDataFlag=IIC_Temp[0];
	InitData[0]=IIC_Temp[1];
	InitData[1]=IIC_Temp[2];  //nian
	InitData[2]=IIC_Temp[3];  //yue
	InitData[3]=IIC_Temp[4];  //ri
	InitData[4]=IIC_Temp[5];
	InitData[5]=IIC_Temp[6];
	InitData[6]=IIC_Temp[7];
	//永久保存标志位
	AT24CXX_Read(0x010A, IIC_Temp,1);
	UnLock_Forever_flag=IIC_Temp[0];

	//保存相应时间参数
	AT24CXX_Read(0x0110, IIC_Temp,4);
	TimesDataEE[0] = (u32)(((u32)IIC_Temp[0])|((u32)IIC_Temp[1]<<8)|((u32)IIC_Temp[2]<<16)|((u32)IIC_Temp[3])<<24);
	AT24CXX_Read(0x0114, IIC_Temp,4);
	TimesDataEE[1] = (u32)(((u32)IIC_Temp[0])|((u32)IIC_Temp[1]<<8)|((u32)IIC_Temp[2]<<16)|((u32)IIC_Temp[3])<<24);
	AT24CXX_Read(0x0118, IIC_Temp,4);
	TimesDataEE[2] = (u32)(((u32)IIC_Temp[0])|((u32)IIC_Temp[1]<<8)|((u32)IIC_Temp[2]<<16)|((u32)IIC_Temp[3])<<24);
  TimesData[0]=TimesDataEE[0];
  TimesData[1]=TimesDataEE[1];
  TimesData[2]=TimesDataEE[2];

}

/*************************************************************************
**  函数名：  LinkMoveDataInit()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：      
**  开发日期：
**************************************************************************/
void LinkMoveDataInit()
{
   //Y-前移
   WriteNum9(0x4940);
   WriteNum9(0x4960);
   //Y-后移
   WriteNum9(0x4950);
   WriteNum9(0x4960);
   //V-前移
   WriteNum9(0x49D0);
   WriteNum9(0x49F0);
   //V-后移
   WriteNum9(0x49E0);
   WriteNum9(0x49F0);
   //W-前移
   WriteNum9(0x4A00);
   WriteNum9(0x4A20);
   //W-后移
   WriteNum9(0x4A10);
   WriteNum9(0x4A20);
}

/**************************************************************************************************
**  函数名：  Order_Package()
**	输入参数：无
**	输出参数：无
**	函数功能：点胶指令封装
**	备注：	  无
**  作者：    White_L     
**  开发日期：2018/7/1 
***************************************************************************************************/
void Read_Point(void)
{
	u8 j=0;
	u16 m=0;
		if(Free_Program_Operate.Num>0)
		{
			if(Free_Program_Operate.Num>LARGESTPROGRAMNUM)
			{
				m=LARGESTPROGRAMNUM;
			}
			else if(Free_Program_Operate.Num>0)
			{
				m=Free_Program_Operate.Num;			
			}
		 for(j=0;j<m;j++)
		 {
			 /*List Value1 2 3 4  5 LC_ID Key IO_Group Delay_Time*/
			AT24CXX_Read(CURRENT_PROGRAM_ADR1+ORDER_LENTH*j,&Order_Temp[3],ORDER_LENTH);
			Free_Program_Operate.Program[j].Flag  = 1;
			Free_Program_Operate.Program[j].List  = Order_Temp[3]|Order_Temp[4]<<8;
			Free_Program_Operate.Program[j].Value1= Order_Temp[5]|Order_Temp[6]<<8|Order_Temp[7]<<16|Order_Temp[8]<<24;
			Free_Program_Operate.Program[j].Value2= Order_Temp[9]|Order_Temp[10]<<8|Order_Temp[11]<<16|Order_Temp[12]<<24;
			Free_Program_Operate.Program[j].Value3= Order_Temp[13]|Order_Temp[14]<<8|Order_Temp[15]<<16|Order_Temp[16]<<24;
			Free_Program_Operate.Program[j].Value4= Order_Temp[17]|Order_Temp[18]<<8|Order_Temp[19]<<16|Order_Temp[20]<<24;	
			Free_Program_Operate.Program[j].Value5= Order_Temp[21]|Order_Temp[22]<<8|Order_Temp[23]<<16|Order_Temp[24]<<24;						 
			Free_Program_Operate.Program[j].LC_ID	= Order_Temp[25];	
			Free_Program_Operate.Program[j].Key   = Order_Temp[26];
			 if(1==Order_Temp[26]||2==Order_Temp[26]||3==Order_Temp[26]||5==Order_Temp[26])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
			 {
				Free_Program_Operate.Program[j].IO_Group  = Order_Temp[27]|Order_Temp[28]<<8|Order_Temp[29]<<16|Order_Temp[30]<<24;	
			 }
			else if(4==IIC_Temp[26])//延时指令
			{
				Free_Program_Operate.Program[j].Delay_Time  = Order_Temp[29]|Order_Temp[30]<<8;				
			}	
			Read_Program_Count++;
		 }
	 }
}
/*************************************************************************
**  函数名：  ReadIICAlarm()
**	输入参数：
**	输出参数：无
**	函数功能：开机初始IIC
**	备注：	  无
**  作者：      
**  开发日期：
**************************************************************************/
void ReadIICAlarm()
{
	u8 i=0;
	u16 temp=0;
	for(i=0;i<20;i++)
	{
		AT24CXX_Read(0x0200+i*21, IIC_Temp, 21);
	  Alarm_History_Display[i].Flag    = IIC_Temp[0] ;
		Alarm_History_Display[i].Day[0]  = IIC_Temp[1] ;
		Alarm_History_Display[i].Day[1]  = IIC_Temp[2] ;
		Alarm_History_Display[i].Day[2]  = IIC_Temp[3] ;
		Alarm_History_Display[i].Day[3]  = IIC_Temp[4] ;
		Alarm_History_Display[i].Day[4]  = IIC_Temp[5] ;
		Alarm_History_Display[i].Day[5]  = IIC_Temp[6] ;
		Alarm_History_Display[i].Day[6]  = IIC_Temp[7] ;
		Alarm_History_Display[i].Day[7]  = IIC_Temp[8] ;
		Alarm_History_Display[i].Day[8]  = IIC_Temp[9] ;
		Alarm_History_Display[i].Day[9]  = IIC_Temp[10] ;
		Alarm_History_Display[i].Time[0] = IIC_Temp[11] ;
		Alarm_History_Display[i].Time[1] = IIC_Temp[12] ;
		Alarm_History_Display[i].Time[2] = IIC_Temp[13] ;
		Alarm_History_Display[i].Time[3] = IIC_Temp[14] ;
		Alarm_History_Display[i].Time[4] = IIC_Temp[15] ;
		Alarm_History_Display[i].Time[5] = IIC_Temp[16] ;
		Alarm_History_Display[i].Time[6] = IIC_Temp[17] ;
		Alarm_History_Display[i].Time[7] = IIC_Temp[18] ;
		Alarm_History_Display[i].Value   = 0x30 ;
		Alarm_History_Display[i].Type    = IIC_Temp[20] ;

		Alarm_History_Time[i].Flag  = Alarm_History_Display[i].Flag;
		Alarm_History_Time[i].year  = 0x2000+(u16)( ((u16)(Alarm_History_Display[i].Day[2]-0x30)<<4)|((u16)(Alarm_History_Display[i].Day[3]-0x30)) );
	  Alarm_History_Time[i].month = ((Alarm_History_Display[i].Day[5]-0x30)<<4)|(Alarm_History_Display[i].Day[6]-0x30) ;
		Alarm_History_Time[i].day   = ((Alarm_History_Display[i].Day[8]-0x30)<<4)|(Alarm_History_Display[i].Day[9]-0x30) ;
		Alarm_History_Time[i].hour  = ((Alarm_History_Display[i].Time[0]-0x30)<<4)|(Alarm_History_Display[i].Time[1]-0x30) ;
		Alarm_History_Time[i].minute= ((Alarm_History_Display[i].Time[3]-0x30)<<4)|(Alarm_History_Display[i].Time[4]-0x30) ;
		Alarm_History_Time[i].second= ((Alarm_History_Display[i].Time[6]-0x30)<<4)|(Alarm_History_Display[i].Time[7]-0x30) ;
		Alarm_History_Time[i].Type  = Alarm_History_Display[i].Type;
		if(Alarm_History_Display[i].Flag==0)
		{
			break;
		}
	}
		AT24CXX_Read(0x0800,IIC_Temp,12);
		Rod_X_Positive=IIC_Temp[0]|IIC_Temp[1]<<8;
		Rod_X_Negative=IIC_Temp[2]|IIC_Temp[3]<<8;
		Rod_Centry_X  =IIC_Temp[4]|IIC_Temp[5]<<8;
		Rod_Centry_Y  =IIC_Temp[6]|IIC_Temp[7]<<8;
		Rod_Y_Negative=IIC_Temp[8]|IIC_Temp[9]<<8;
		Rod_Y_Positive=IIC_Temp[10]|IIC_Temp[11]<<8;
		 //X轴
		 temp=Rod_X_Positive-Rod_Centry_X-100;
		 temp=temp>>3;
		 Rod_Step_X[4]=Rod_Centry_X+50;
		 Rod_Step_X[5]=Rod_Centry_X+50+temp*3;
		 Rod_Step_X[6]=Rod_Centry_X+50+temp*6;
		 Rod_Step_X[7]=Rod_X_Positive-50;
		 temp=Rod_Centry_X-Rod_X_Negative-100;
		 temp=temp>>3;
		 Rod_Step_X[0]=Rod_X_Negative+50;
		 Rod_Step_X[1]=Rod_X_Negative+50+temp*3;
		 Rod_Step_X[2]=Rod_X_Negative+50+temp*6;
		 Rod_Step_X[3]=Rod_Centry_X-50;
		//Y轴
		 temp=Rod_Y_Positive-Rod_Centry_Y-100;
		 temp=temp>>3;
		 Rod_Step_Y[4]=Rod_Centry_Y+50;
		 Rod_Step_Y[5]=Rod_Centry_Y+50+temp*3;
		 Rod_Step_Y[6]=Rod_Centry_Y+50+temp*6;
		 Rod_Step_Y[7]=Rod_Y_Positive-50;
		 temp=Rod_Centry_Y-Rod_Y_Negative-100;
		 temp=temp>>3;
		 Rod_Step_Y[0]=Rod_Y_Negative+50;
		 Rod_Step_Y[1]=Rod_Y_Negative+50+temp*3;
		 Rod_Step_Y[2]=Rod_Y_Negative+50+temp*6;
		 Rod_Step_Y[3]=Rod_Centry_Y-50;		
	/******************2018.6.25White_L 读取保存程序信息*******************************/
		AT24CXX_Read(0x1015,&g_Run_Program_Num,1);//当前运行程序编号
	/**Flag Code Name Order_Num Time***/
	  for(i=0;i<SAVEPROGRAMNUM;i++)
		{
			if(Program_Flag[i]==1)
			{
				OperateProgram[Product_Num].Flag = Saved_Program[i].Flag;
				OperateProgram[Product_Num].Code = Saved_Program[i].Code;
				OperateProgram[Product_Num].XuHao = Product_Num+1;
				OperateProgram[Product_Num].Name = Saved_Program[i].Name;
				OperateProgram[Product_Num].Order_Num = Saved_Program[i].Order_Num;
				OperateProgram[Product_Num].time = Saved_Program[i].Time;
				Product_Num++;
			}
		}
		if(Product_Num!=0)
		{
			Free_Program_Operate.Flag = 1;
			Free_Program_Operate.Code = g_Run_Program_Num;
			Free_Program_Operate.Name = Saved_Program[g_Run_Program_Num-1].Name;
			Free_Program_Operate.Num  = Saved_Program[g_Run_Program_Num-1].Order_Num;
			Order_Num                 = Free_Program_Operate.Num;
			Current_Operate_Program.Flag  = 1;
			Current_Operate_Program.Code  = g_Run_Program_Num;
			Current_Operate_Program.Name  = Saved_Program[g_Run_Program_Num-1].Name;
			Current_Operate_Program.Num   = Saved_Program[g_Run_Program_Num-1].Order_Num;	
			gs_Run_Program_Name = Saved_Program[g_Run_Program_Num-1].Name;
			g_Run_Program_Name  = Saved_Program[g_Run_Program_Num-1].Name;
			Read_Point();//读取点坐标
			Select_Program_Flag=TRUE;
//			Current_Select_Program=g_Run_Program_Num;
	  }
		else
		{
		 for(i=0;i<LARGESTPROGRAMNUM;i++)
		 {
			Free_Program_Operate.Program[i].List   = 0;
			Free_Program_Operate.Program[i].Key    = 0;
			Free_Program_Operate.Program[i].Value1 = 0;
			Free_Program_Operate.Program[i].Value2 = 0;	
			Free_Program_Operate.Program[i].Value3 = 0;
			Free_Program_Operate.Program[i].LC_ID	 = 0;
		 }			
		}
}
/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team ******** END OF FILE ************************/
