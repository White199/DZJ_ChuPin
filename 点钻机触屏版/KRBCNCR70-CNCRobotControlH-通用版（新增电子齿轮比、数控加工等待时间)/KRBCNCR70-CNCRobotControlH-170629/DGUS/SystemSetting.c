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
#include "Delay.h"
#include "usbh_usr.h"

//static u8 gs_System_Setting_Page = 0;
static u8 gs_Pre_Choose_Point = 0;	              //先前选择点
static u8 gs_Current_Choose_Point = 0;	          //当前选择点
       u8 gs_Point_Display_Refresh = FALSE;	      //点显示刷新

//---新定义
//static u32 gs_Origin_Speed = 10;				          //回零速度
//static u8 gs_Refresh_Origin_Speed = TRUE;	        //回零速度刷新

static u8 gs_Pre_Choose_Program = 0;	            //先前选择程序
u8 gs_Current_Choose_Program = 0;	        //当前选择程序
       u8 gs_Program_Display_Refresh = FALSE;	    //程序显示刷新
			 
u16 gs_Pre_Choose_IO1 = 0;                   //先前选择IO口
u16 gs_Current_Choose_IO1= 0;	              //当前选择IO口	

u16 gs_Pre_Choose_IO2 = 0;                   //先前选择IO口
u16 gs_Current_Choose_IO2 = 0;	              //当前选择IO口	

u16 gs_Pre_Choose_IO3 = 0;                   //先前选择IO口
u16 gs_Current_Choose_IO3= 0;	              //当前选择IO口
u8  g_IO_DetectAck_Flag = FALSE;              //输入状态通信应答成功标志位
u8 IO_Detect_Parameter[40]={0};
u8 Temp_IO_Detect_Parameter[40]={0};
u8 OriginDir_Flag=0;

extern u8 Firmware_Name_Table[9][11];//lin
extern UINT   File_Num;				//lin


static PageCut Save_Point_List[25] ={ //第一行
                                      {0x5400,21,100,95 ,120,105,100,97 }, 
                                      {0x5400,21,275,95 ,295,105,275,97 },	 
									                    {0x5400,21,430,95 ,450,105,430,97 },
                                      {0x5400,21,585,95 ,605,105,585,97 },	 
									                    {0x5400,21,740,95 ,760,105,740,97 },
									                    //第二行
                                      {0x5400,21,100,160,120,170,100,162}, 
                                      {0x5400,21,275,160,295,170,275,162},	 
									                    {0x5400,21,430,160,450,170,430,162},
                                      {0x5400,21,585,160,605,170,585,162},	 
									                    {0x5400,21,740,160,760,170,740,162},
  									                  //第三行
                                      {0x5400,21,100,220,120,230,100,222}, 
                                      {0x5400,21,275,220,295,230,275,222},	 
									                    {0x5400,21,430,220,450,230,430,222},
                                      {0x5400,21,585,220,605,230,585,222},	 
									                    {0x5400,21,740,220,760,230,740,222},  
									                    //第四行
                                      {0x5400,21,100,282,120,292,100,285}, 
                                      {0x5400,21,275,282,295,292,275,285},	 
									                    {0x5400,21,430,282,450,292,430,285},
                                      {0x5400,21,585,282,605,292,585,285},	 
									                    {0x5400,21,740,282,760,292,740,285},
									                    //第五行
                                      {0x5400,21,100,346,120,356,100,350}, 
                                      {0x5400,21,275,346,295,356,275,350},	 
									                    {0x5400,21,430,346,450,356,430,350},
                                      {0x5400,21,585,346,605,356,585,350},	 
									                    {0x5400,21,740,346,760,356,740,350},
									                 };
//存储程序选中效果
static PageCut Save_Program_List[20] ={ //第一行
                                        {0x5410,46,115,100,130,115,115,100}, 
                                        {0x5410,46,275,100,290,115,275,100},	 
									                      {0x5410,46,430,100,445,115,430,100},
                                        {0x5410,46,585,100,600,115,585,100},	 
									                      {0x5410,46,742,100,757,115,742,100},
									                      //第二行
                                        {0x5410,46,115,180,130,195,115,180}, 
                                        {0x5410,46,275,180,290,195,275,180},	 
									                      {0x5410,46,430,180,445,195,430,180},
                                        {0x5410,46,585,180,600,195,585,180},	 
									                      {0x5410,46,742,180,757,195,742,180},
 									                      //第三行
                                        {0x5410,46,115,260,130,275,115,260}, 
                                        {0x5410,46,275,260,290,275,275,260},	 
									                      {0x5410,46,430,260,445,275,430,260},
                                        {0x5410,46,585,260,600,275,585,260},	 
									                      {0x5410,46,742,260,757,275,742,260},
									                      //第四行
                                        {0x5410,46,115,335,130,350,115,335}, 
                                        {0x5410,46,275,335,290,350,275,335},	 
									                      {0x5410,46,430,335,445,350,430,335},
                                        {0x5410,46,585,335,600,350,585,335},	 
									                      {0x5410,46,742,335,757,350,742,335},
									                    };

static PageCut Save_IO_List[40]  =  { //第一行
                                      {0x5440,21,80,95 ,90,105,80,97 }, 
                                      {0x5440,21,180,95 ,190,105,180,97 },	 
									                    {0x5440,21,280,95 ,290,105,280,97 },
                                      {0x5440,21,375,95 ,385,105,375,97 },	 
									                    {0x5440,21,475,95 ,485,105,475,97 },
																			{0x5440,21,575,95 ,585,105,575,97 },
																			{0x5440,21,675,95 ,685,105,675,97 },
																			{0x5440,21,770,95 ,780,105,770,97 },
									                    //第二行
                                      {0x5440,21,80,160,90,170,80,162}, 
                                      {0x5440,21,180,160,190,170,180,162},	 
									                    {0x5440,21,280,160,290,170,280,162},
                                      {0x5440,21,375,160,385,170,375,162},	 
									                    {0x5440,21,475,160,485,170,475,162},
																			{0x5440,21,575,160,585,170,575,162},
																			{0x5440,21,675,160,685,170,675,162},
																			{0x5440,21,770,160,780,170,770,162},
  									                  //第三行
                                      {0x5440,21,80,220,90,230,80,222}, 
                                      {0x5440,21,180,220,190,230,180,222},	 
									                    {0x5440,21,280,220,290,230,280,222},
                                      {0x5440,21,375,220,385,230,375,222},	 
									                    {0x5440,21,475,220,485,230,475,222},
																			{0x5440,21,575,220,585,230,575,222},
																			{0x5440,21,675,220,685,230,675,222},
																			{0x5440,21,770,220,780,230,770,222},																			
									                    //第四行
                                      {0x5440,21,80,282,90,292,80,285}, 
                                      {0x5440,21,180,282,190,292,180,285},	 
									                    {0x5440,21,280,282,290,292,280,285},
                                      {0x5440,21,375,282,385,292,375,285},	 
									                    {0x5440,21,475,282,485,292,475,285},
																			{0x5440,21,575,282,585,292,575,285},
																			{0x5440,21,675,282,685,292,675,285},
																			{0x5440,21,770,282,780,292,770,285},	
									                    //第五行
                                      {0x5440,21,80,346,90,356,80,350}, 
                                      {0x5440,21,180,346,190,356,180,350},	 
									                    {0x5440,21,280,346,290,356,280,350},
                                      {0x5440,21,375,346,385,356,375,350},	 
									                    {0x5440,21,475,346,485,356,475,350},
																			{0x5440,21,575,346,585,356,575,350},
																			{0x5440,21,675,346,685,356,675,350},
																			{0x5400,21,770,346,780,356,770,350},	
									                 };

static PageCut Save_Pre_IO_List[40]  =  { //第一行
                                      {0x5E40,129,80,95 ,90,105,80,97 }, 
                                      {0x5E40,129,180,95 ,190,105,180,97 },	 
									                    {0x5E40,129,280,95 ,290,105,280,97 },
                                      {0x5E40,129,375,95 ,385,105,375,97 },	 
									                    {0x5E40,129,475,95 ,485,105,475,97 },
																			{0x5E40,129,575,95 ,585,105,575,97 },
																			{0x5E40,129,675,95 ,685,105,675,97 },
																			{0x5E40,129,770,95 ,780,105,770,97 },
									                    //第二行
                                      {0x5E40,129,80,160,90,170,80,162}, 
                                      {0x5E40,129,180,160,190,170,180,162},	 
									                    {0x5E40,129,280,160,290,170,280,162},
                                      {0x5E40,129,375,160,385,170,375,162},	 
									                    {0x5E40,129,475,160,485,170,475,162},
																			{0x5E40,129,575,160,585,170,575,162},
																			{0x5E40,129,675,160,685,170,675,162},
																			{0x5E40,129,770,160,780,170,770,162},
  									                  //第三行
                                      {0x5E40,129,80,220,90,230,80,222}, 
                                      {0x5E40,129,180,220,190,230,180,222},	 
									                    {0x5E40,129,280,220,290,230,280,222},
                                      {0x5E40,129,375,220,385,230,375,222},	 
									                    {0x5E40,129,475,220,485,230,475,222},
																			{0x5E40,129,575,220,585,230,575,222},
																			{0x5E40,129,675,220,685,230,675,222},
																			{0x5E40,129,770,220,780,230,770,222},																			
									                    //第四行
                                      {0x5E40,129,80,282,90,292,80,285}, 
                                      {0x5E40,129,180,282,190,292,180,285},	 
									                    {0x5E40,129,280,282,290,292,280,285},
                                      {0x5E40,129,375,282,385,292,375,285},	 
									                    {0x5E40,129,475,282,485,292,475,285},
																			{0x5E40,129,575,282,585,292,575,285},
																			{0x5E40,129,675,282,685,292,675,285},
																			{0x5E40,129,770,282,780,292,770,285},	
									                    //第五行
                                      {0x5E40,129,80,346,90,356,80,350}, 
                                      {0x5E40,129,180,346,190,356,180,350},	 
									                    {0x5E40,129,280,346,290,356,280,350},
                                      {0x5E40,129,375,346,385,356,375,350},	 
									                    {0x5E40,129,475,346,485,356,475,350},
																			{0x5E40,129,575,346,585,356,575,350},
																			{0x5E40,129,675,346,685,356,675,350},
																			{0x5E40,129,770,346,780,356,770,350},	
									                 };

static u8 i = 0;
static u8 j = 0;
static u8 gs_Save_Program_Num = 0;             //记录查询得到的已保存程序的个数
SoftLimit  Temp_Soft_Limit[6]={0};				//XZYOAB
SoftLimit  Temp_Soft_Limit1[6]={0};				//XZYOAB
SoftLimit  Robot_SoftLimit[6]={0};		         //XZL软限位参数

static u8  gs_Pre_Buzzer_switch = FALSE;       //蜂鸣器之前状态
static u8  Buzzer_Switch_On[]={"开"};	         //蜂鸣器：开
static u8  Buzzer_Switch_Off[]={"关"};	       //蜂鸣器：关
static u8  gs_BackLight_Refresh = 1;

static u8  gs_Pass_Word_String[]={"******"};   //显示的隐藏字符
static u8  gs_Pass_Word_Refresh = 0;	         //显示刷新
static u32 gs_Pass_Word_Temp[3] = {0};	       //原密码、新密码、确认密码缓存

       u8  gs_Pre_Choose_Area = 0;             //先前选择区域
       u8  g_Current_Choose_Area = 0;          //当前选择区域

//安全区域效果
PageCut Safe_Area_List[7] ={ 
                             {0x5420,52,11 ,98 ,170,148,11 ,98 }, 
                             {0x5420,52,11 ,151,170,201,11 ,151},	 
									           {0x5420,52,11 ,204,170,254,11 ,204},
                             {0x5420,52,11 ,257,170,307,11 ,257},	 
									           {0x5420,52,11 ,310,170,360,11 ,310},
									           {0x5420,52,11 ,363,170,413,11 ,363},
									           {0x5420,52,11 ,416,170,466,11 ,416},
									         };
SafeArea Robot_Safe_Area[7]={0};

/*******************************TWS***************************************/
/******************为复位顺序定义图片剪切*********************************/
static PageCut row1_X_Y = {0x5660,128,85,135,120,165,85,135};	 //第一行
static PageCut row1_X_N = {0x5660,129,85,135,120,165,85,135};	

static PageCut row1_Y_Y = {0x5670,128,132,135,167,165,132,135};	 
static PageCut row1_Y_N = {0x5670,129,132,135,167,165,132,135};	

static PageCut row1_Z_Y = {0x5680,128,179,135,214,165,179,135};	 
static PageCut row1_Z_N = {0x5680,129,179,135,214,165,179,135};	

static PageCut row1_O_Y = {0x5690,128,226,135,261,165,226,135};	 
static PageCut row1_O_N = {0x5690,129,226,135,261,165,226,135};	

static PageCut row1_A_Y = {0x56A0,128,273,135,308,165,273,135};	 
static PageCut row1_A_N = {0x56A0,129,273,135,308,165,273,135};	

static PageCut row1_B_Y = {0x56B0,128,320,135,355,165,320,135};	 
static PageCut row1_B_N = {0x56B0,129,320,135,355,165,320,135};	

static PageCut row2_X_Y = {0x56C0,128,85,175,120,205,85,175};	 //第二行
static PageCut row2_X_N = {0x56C0,129,85,175,120,205,85,175};	

static PageCut row2_Y_Y = {0x56D0,128,132,175,167,205,132,175};	 
static PageCut row2_Y_N = {0x56D0,129,132,175,167,205,132,175};	

static PageCut row2_Z_Y = {0x56E0,128,179,175,214,205,179,175};	 
static PageCut row2_Z_N = {0x56E0,129,179,175,214,205,179,175};	

static PageCut row2_O_Y = {0x56F0,128,226,175,261,205,226,175};	 
static PageCut row2_O_N = {0x56F0,129,226,175,261,205,226,175};	

static PageCut row2_A_Y = {0x5700,128,273,175,308,205,273,175};	 
static PageCut row2_A_N = {0x5700,129,273,175,308,205,273,175};	

static PageCut row2_B_Y = {0x5710,128,320,175,355,205,320,175};	 
static PageCut row2_B_N = {0x5710,129,320,175,355,205,320,175};	

static PageCut row3_X_Y = {0x5720,128,85,215,120,245,85,215};	 //第三行
static PageCut row3_X_N = {0x5720,129,85,215,120,245,85,215};	

static PageCut row3_Y_Y = {0x5730,128,132,215,167,245,132,215};	 
static PageCut row3_Y_N = {0x5730,129,132,215,167,245,132,215};	

static PageCut row3_Z_Y = {0x5740,128,179,215,214,245,179,215};	 
static PageCut row3_Z_N = {0x5740,129,179,215,214,245,179,215};	

static PageCut row3_O_Y = {0x5750,128,226,215,261,245,226,215};	 
static PageCut row3_O_N = {0x5750,129,226,215,261,245,226,215};	

static PageCut row3_A_Y = {0x5760,128,273,215,308,245,273,215};	 
static PageCut row3_A_N = {0x5760,129,273,215,308,245,273,215};	

static PageCut row3_B_Y = {0x5770,128,320,215,355,245,320,215};	 
static PageCut row3_B_N = {0x5770,129,320,215,355,245,320,215};

static PageCut row4_X_Y = {0x5780,128,85,255,120,285,85,255};	 //第四行
static PageCut row4_X_N = {0x5780,129,85,255,120,285,85,255};	

static PageCut row4_Y_Y = {0x5790,128,132,255,167,285,132,255};	 
static PageCut row4_Y_N = {0x5790,129,132,255,167,285,132,255};	

static PageCut row4_Z_Y = {0x57A0,128,179,255,214,285,179,255};	 
static PageCut row4_Z_N = {0x57A0,129,179,255,214,285,179,255};	

static PageCut row4_O_Y = {0x57B0,128,226,255,261,285,226,255};	 
static PageCut row4_O_N = {0x57B0,129,226,255,261,285,226,255};	

static PageCut row4_A_Y = {0x57C0,128,273,255,308,285,273,255};	 
static PageCut row4_A_N = {0x57C0,129,273,255,308,285,273,255};	

static PageCut row4_B_Y = {0x57D0,128,320,255,355,285,320,255};	 
static PageCut row4_B_N = {0x57D0,129,320,255,355,285,320,255};	

static PageCut row5_X_Y = {0x57E0,128,85,295,120,325,85,295};	 //第五行
static PageCut row5_X_N = {0x57E0,129,85,295,120,325,85,295};	

static PageCut row5_Y_Y = {0x57F0,128,132,295,167,325,132,295};	 
static PageCut row5_Y_N = {0x57F0,129,132,295,167,325,132,295};	

static PageCut row5_Z_Y = {0x5800,128,179,295,214,325,179,295};	 
static PageCut row5_Z_N = {0x5800,129,179,295,214,325,179,295};	

static PageCut row5_O_Y = {0x5810,128,226,295,261,325,226,295};	 
static PageCut row5_O_N = {0x5810,129,226,295,261,325,226,295};	

static PageCut row5_A_Y = {0x5820,128,273,295,308,325,273,295};	 
static PageCut row5_A_N = {0x5820,129,273,295,308,325,273,295};	

static PageCut row5_B_Y = {0x5830,128,320,295,355,325,320,295};	 
static PageCut row5_B_N = {0x5830,129,320,295,355,325,320,295};	

static PageCut row6_X_Y = {0x5840,128,85,335,120,365,85,335};	 //第六行
static PageCut row6_X_N = {0x5840,129,85,335,120,365,85,335};	

static PageCut row6_Y_Y = {0x5850,128,132,335,167,365,132,335};	 
static PageCut row6_Y_N = {0x5850,129,132,335,167,365,132,335};	

static PageCut row6_Z_Y = {0x5860,128,179,335,214,365,179,335};	 
static PageCut row6_Z_N = {0x5860,129,179,335,214,365,179,335};	

static PageCut row6_O_Y = {0x5870,128,226,335,261,365,226,335};	 
static PageCut row6_O_N = {0x5870,129,226,335,261,365,226,335};	

static PageCut row6_A_Y = {0x5880,128,273,335,308,365,273,335};	 
static PageCut row6_A_N = {0x5880,129,273,335,308,365,273,335};	

static PageCut row6_B_Y = {0x5890,128,320,335,355,365,320,335};	 
static PageCut row6_B_N = {0x5890,129,320,335,355,365,320,335};


u8 row1_X=0,row1_Y=0,row1_Z=0,row1_O=0,row1_A=0,row1_B=0,
   row2_X=0,row2_Y=0,row2_Z=0,row2_O=0,row2_A=0,row2_B=0,
   row3_X=0,row3_Y=0,row3_Z=0,row3_O=0,row3_A=0,row3_B=0,
   row4_X=0,row4_Y=0,row4_Z=0,row4_O=0,row4_A=0,row4_B=0,
   row5_X=0,row5_Y=0,row5_Z=0,row5_O=0,row5_A=0,row5_B=0,
   row6_X=0,row6_Y=0,row6_Z=0,row6_O=0,row6_A=0,row6_B=0;

/*******************************TWS***************************************/

//软限位显示
static u8  gs_Reflash_SoftLimit = FALSE;
static u8  gs_Refresh_SoftLimit_Temp = FALSE; //点软限位开关时
static u8  gs_Refresh_SafeArea_Temp = TRUE;	  //点安全区域开关时
static u8  gs_Refresh_ORigin_Temp = TRUE;	  //复位设置时
static u8  gs_Refresh_Speed_Temp = TRUE;	  	 //速度设置
static u8  X_SoftLimit_On[]={"开"};	          //X轴限位：开
static u8  X_SoftLimit_Off[]={"关"};	        //X轴限位：关
static u8  L_SoftLimit_On[]={"开"};	          //L轴限位：开
static u8  L_SoftLimit_Off[]={"关"};	        //L轴限位：关
static u8  O_SoftLimit_On[]={"开"};	          //O轴限位：开
static u8  O_SoftLimit_Off[]={"关"};	        //O轴限位：关
static u8  Z_SoftLimit_On[]={"开"};	          //Z轴限位：开
static u8  Z_SoftLimit_Off[]={"关"};	        //Z轴限位：关
static u8  X_SafeArea_On[]={"启用"};	        //X轴安全区：启用
static u8  X_SafeArea_Off[]={"关闭"};	        //X轴安全区：关闭
static u8  LC_Num[10][6]={{"料仓1"},
						  {"料仓2"},
						  {"料仓3"},
						  {"料仓4"},
						  {"料仓5"},
						  {"料仓6"},
						  {"料仓7"},
						  {"料仓8"},
						  {"料仓9"},
						  {"料仓10"}};	        //X轴安全区：关闭

       u8 gs_Reflash_Fuction_JXS  = 0;	              //机械手功能显示刷新
       u8 gs_Reflash_Fuction_LC   = 0;	              //料仓功能显示刷新
       u8 gs_Reflash_Fuction_SC   = 0;	              //生产功能显示刷新

static u8  FucSet_Axis[6][12]={	//轴选择  
											  {"X-Z-Y    "},	  //3
							                  {"X-Y-Z-A-B"},	    //5
											  {"X-Y-Z-A-B-O"}	    //6
						                   };
static u8  FucSet_Origin[9][12]={ //回原点方式
							                    {"Z-Y-X    "},    //4
												{"Z-A-B-Y-X"},	  //8		
												{"Z-A-B-Y-X-O"}	  //9
						                     };
// static u8  FucSet_Accelerate[3][10]={	 //回原点方式	
// 							{"等级一  "},	  //1 
// 							{"等级二  "},	  //2
// 							{"等级三  "}	  //3
// 					      };

//---6.28 加速度类型 S  T 
static u8 FucSet_Acc_Type[2][1]={ //用于加速度时间处的 加速度类型选择
										              {"S"},
												          {"T"}												 
                                };
//----2017.6.28
static u8 FucSet_Acc_Axis[6][1]={ //用于加速度时间处的轴选择
										              {"X"},       //0
												          {"Z"},       //1
                                  {"Y"},       //2
                                  {"O"},		   //3											 
                                  {"A"},	     //4											 
                                  {"B"}		     //5											 
                                 };
//---12.20 伺服报警信号
static u8 FucSet_ServoAlarm[2][1]={ //用于伺服报警信号处常闭常开类型选择
										              {"0"},       //0
												          {"1"}				 //1								 
                                };
//---2018.3.20 数控加工等待时间-IO口
static u8 FucSet_NumericalControl[5][3]={ //用于数控加工等待时间IO口选择
	                                {"关 "},     //0
										              {"X32"},     //1
												          {"X33"},     //2
										              {"X34"},     //3
												          {"X35"},     //4         																	
                                };
static u8  FucSet_GearRatio[4][1]={ //用于电子齿轮比处的轴选择
										              {"X"},       //0
												          {"Z"},       //1
                                  {"Y"},       //2
                                  {"O"}		     //3		
                                };
static u8  Origin_DirectionX[6][4]={ //回原点轴X		
							                       {"X"},	  //0
							                       {"Y"},	  //1 
							                       {"Z"},	  //2
							                       {"A"},	  //3
							                       {"B"},	  //4
							                       {"O"},	  //5
					                         };
static u8  Origin_DirectionZ[2][4]={ //回原点方向Z		
							                       {" 上 "},	  //0
							                       {" 下 "}	    //1 
					                         };
static u8  LC_Switch[2][4]={ //料仓开关		
							               {"关闭"},	          //0
							               {"启用"}	            //1 
					                 };
//static u8  LC_Mode[2][6]={	     //料仓开关		
//							{"模式一"},	  //0
//							{"模式二"},	  //1 
//					      };
//static u32 gs_Robot_Speed_Level = 100;		//回零速度

static u8  gs_Refresh_Axis_Num = FALSE;          //机械手轴个数
static u8  gs_Refresh_Origin_Dir = FALSE;        //原点方向选择
static u8  gs_Refresh_Origin_Method = FALSE;     //回原点方式
static u8  gs_Refresh_Accelerate_Level = FALSE;  //加速度等级
static u8  gs_Refresh_ServoAlarm = FALSE;        //伺服报警信号类型
static u8  gs_Refresh_NumericalControl = FALSE;  //数控加工等待时间 2018.3.20
static u8  gs_Refresh_GearRatio = FALSE;  //电子齿轮比  2018.3.21

									 
		
									 
u8 Reset_row1=0,Reset_row2=0,Reset_row3=0,
   Reset_row4=0,Reset_row5=0,Reset_row6=0;
u8 Reset_num[6] = {0x20,0x10,0x08,0x04,0x02,0x01};
u8 First=0,Second=0,Third=0,Fourth=0,Fifth=0,Sixth=0;
									 
JXSParameter JXS_Parameter = {0};
JXSParameter Temp_JXS_Parameter = {0};
DZJSpeed DZJSpeed1;
u8 Temp_ACC_Axis=X_AXIS-1;
/************2018.6.19white_L **********************/
u8 LC_Count_X[10],LC_Count_Y[10];

LCParameter LC_Parameter_Load[LC_MAXNUM] = {0};		         //上料仓参数
LCParameter LC_Parameter_UnLoad = {0};		       //卸料仓参数
LCParameter Temp_LC_Parameter_Load[LC_MAXNUM] = {0};        //上料仓
LCParameter Temp_LC_Parameter_UnLoad = {0};	     //卸料仓
LCPosition LC_Load_Position[20][20]={0};	       //上料仓计算存储的位置表
LCPosition LC_UnLoad_Position[20][20]={0};	     //卸料仓计算存储的位置表
u32 LC_Load_OAxis_Position[50]={0};			         //上料仓计算垂直轴位置表
u32 LC_UnLoad_OAxis_Position[50]={0};		         //卸料仓计算垂直轴位置表
u8  LC_Load_Pre_PositionX=0;				             //上料仓X轴当前位置
u8  LC_Load_Pre_PositionY=0;				             //上料仓L轴当前位置
u8  LC_UnLoad_Pre_PositionX=0;				           //卸料仓X轴当前位置
u8  LC_UnLoad_Pre_PositionY=0;				           //卸料仓L轴当前位置
u8  LC_UnLoad_Pre_PositionZ=0;				           //卸料仓Z轴当前位置
u8  LC_Load_Pre_ListX=0;			   	               //上料仓X轴当前指令行数标识
u8  LC_Load_Pre_ListY=0;			   	               //上料仓L轴当前指令行数标识
u8  LC_UnLoad_Pre_ListX=0;			   	             //卸料仓X轴当前指令行数标识
u8  LC_UnLoad_Pre_ListY=0;			   	             //卸料仓L轴当前指令行数标识
u8  LC_UnLoad_Pre_ListZ=0;			   	             //卸料仓Z轴当前指令行数标识
u8  g_Calculate_LC_Load_Parameter_Flag=FALSE;
u8  g_Calculate_LC_UnLoad_Parameter_Flag=FALSE;
u8  g_LC_Load_Position_Flag=FALSE;
u8  g_LC_UnLoad_Position_Flag=FALSE;
static u8  gs_Refresh_LC_Switch = FALSE;         //料仓开关显示
static u8  gs_Refresh_LC_Position = FALSE;       //料仓位置显示
       u8 Set_LCPosition_XYO=0;                  //示教点选择标识位

SCParameter SC_Parameter = {0};		               //生产参数
SCParameter Temp_SC_Parameter = {0};
static u32 gs_LJProgress_Num = 0;
static u8  gs_Clear_SC_Parametr = FALSE;
static u8  gs_Clear_LJ_Parametr = FALSE;

       u8  g_USART1_DeletePoint_Flag=FALSE;			 //标志开始删除
static u8  gs_USBH_Copy_Status=USBHSTATUS_NOUDISK;

static PageCut PageCut_SavePointX={0x5430,41 ,8,45,792,85,8,45};   //X
static PageCut PageCut_SavePointZ={0x5430,44 ,8,45,792,85,8,45};   //Z
static PageCut PageCut_SavePointL={0x5430,115,8,45,792,85,8,45};   //Y
static PageCut PageCut_SavePointO={0x5430,123,8,45,792,85,8,45};   //O
       u8  Beyond_Forever[]={": 当前使用期限为永久解密！     "};
u8 LC_ID=0;
			 
			 u32  Admin_Pass_Word = 0;
static u8 gs_Refresh_Admin_Pass_Word=FALSE;
			 
extern u8 LCD_Main_Page_Pre;
	   
extern u16 Rod_X_Positive, Rod_X_Negative,Rod_Y_Positive,Rod_Y_Negative,Rod_Centry_X,Rod_Centry_Y;
extern vu16  After_filter[4];    //用来存放求平均值之后的结果 
extern u16 Rod_Step_X[8],Rod_Step_Y[8];	 
extern int Rod_Display_X,Rod_Display_Y;	   
/*************************************************************************
**  函数名：  Robot_Init()
**	输入参数：机械手开机初始化参数
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/11/03
**************************************************************************/
void Robot_Init()
{
	//软限位参数初始化
	Temp_Soft_Limit[0].Left_Limit  =Robot_SoftLimit[0].Left_Limit;
	Temp_Soft_Limit[0].Right_Limit =Robot_SoftLimit[0].Right_Limit;
	Temp_Soft_Limit[0].Switch_Limit=Robot_SoftLimit[0].Switch_Limit;
	Temp_Soft_Limit[1].Left_Limit  =Robot_SoftLimit[1].Left_Limit;
	Temp_Soft_Limit[1].Right_Limit =Robot_SoftLimit[1].Right_Limit;
	Temp_Soft_Limit[1].Switch_Limit=Robot_SoftLimit[1].Switch_Limit;
	Temp_Soft_Limit[2].Left_Limit  =Robot_SoftLimit[2].Left_Limit;
	Temp_Soft_Limit[2].Right_Limit =Robot_SoftLimit[2].Right_Limit;
	Temp_Soft_Limit[2].Switch_Limit=Robot_SoftLimit[2].Switch_Limit;
	Temp_Soft_Limit[3].Left_Limit  =Robot_SoftLimit[3].Left_Limit;
	Temp_Soft_Limit[3].Right_Limit =Robot_SoftLimit[3].Right_Limit;
	Temp_Soft_Limit[3].Switch_Limit=Robot_SoftLimit[3].Switch_Limit;
	Temp_Soft_Limit[4].Left_Limit  =Robot_SoftLimit[4].Left_Limit;
	Temp_Soft_Limit[4].Right_Limit =Robot_SoftLimit[4].Right_Limit;
	Temp_Soft_Limit[4].Switch_Limit=Robot_SoftLimit[4].Switch_Limit;	
	Temp_Soft_Limit[5].Left_Limit  =Robot_SoftLimit[5].Left_Limit;
	Temp_Soft_Limit[5].Right_Limit =Robot_SoftLimit[5].Right_Limit;
	Temp_Soft_Limit[5].Switch_Limit=Robot_SoftLimit[5].Switch_Limit;
	//功能设定参数初始化
	Temp_JXS_Parameter.Axis = JXS_Parameter.Axis;
	Temp_JXS_Parameter.Origin[0] = JXS_Parameter.Origin[0];
	Temp_JXS_Parameter.Origin[1] = JXS_Parameter.Origin[1];
	Temp_JXS_Parameter.Origin[2] = JXS_Parameter.Origin[2];
	Temp_JXS_Parameter.Origin[3] = JXS_Parameter.Origin[3];
	Temp_JXS_Parameter.Origin[4] = JXS_Parameter.Origin[4];
	Temp_JXS_Parameter.Origin[5] = JXS_Parameter.Origin[5];
	Temp_JXS_Parameter.OriginDir[0] = JXS_Parameter.OriginDir[0];
	Temp_JXS_Parameter.OriginDir[1] = JXS_Parameter.OriginDir[1];
	Temp_JXS_Parameter.OriginDir[2] = JXS_Parameter.OriginDir[2];
	Temp_JXS_Parameter.OriginDir[3] = JXS_Parameter.OriginDir[3];
	Temp_JXS_Parameter.OriginDir[4] = JXS_Parameter.OriginDir[4];
	Temp_JXS_Parameter.OriginDir[5] = JXS_Parameter.OriginDir[5];
	Temp_JXS_Parameter.SpeedLevel = JXS_Parameter.SpeedLevel;
	Temp_JXS_Parameter.OriginSpeed[0] = JXS_Parameter.OriginSpeed[0];
	Temp_JXS_Parameter.OriginSpeed[1] = JXS_Parameter.OriginSpeed[1];
	Temp_JXS_Parameter.OriginSpeed[2] = JXS_Parameter.OriginSpeed[2];
	Temp_JXS_Parameter.OriginSpeed[3] = JXS_Parameter.OriginSpeed[3];
	Temp_JXS_Parameter.OriginSpeed[4] = JXS_Parameter.OriginSpeed[4];
	Temp_JXS_Parameter.OriginSpeed[5] = JXS_Parameter.OriginSpeed[5];
	Temp_JXS_Parameter.OriginPosition[0] = JXS_Parameter.OriginPosition[0];
	Temp_JXS_Parameter.OriginPosition[1] = JXS_Parameter.OriginPosition[1];
	Temp_JXS_Parameter.OriginPosition[2] = JXS_Parameter.OriginPosition[2];
	Temp_JXS_Parameter.OriginPosition[3] = JXS_Parameter.OriginPosition[3];
	Temp_JXS_Parameter.OriginPosition[4] = JXS_Parameter.OriginPosition[4];
	Temp_JXS_Parameter.OriginPosition[5] = JXS_Parameter.OriginPosition[5];
	Temp_JXS_Parameter.ManualSpeed	= JXS_Parameter.ManualSpeed ;
	
	
	Temp_LC_Parameter_Load[LC_ID].Switch      = LC_Parameter_Load[LC_ID].Switch;
	Temp_LC_Parameter_Load[LC_ID].H_Num       = LC_Parameter_Load[LC_ID].H_Num;
	Temp_LC_Parameter_Load[LC_ID].V_Num       = LC_Parameter_Load[LC_ID].V_Num;
	Temp_LC_Parameter_Load[LC_ID].Position1.XPoint = LC_Parameter_Load[LC_ID].Position1.XPoint;
	Temp_LC_Parameter_Load[LC_ID].Position1.YPoint = LC_Parameter_Load[LC_ID].Position1.YPoint;
	Temp_LC_Parameter_Load[LC_ID].Position2.XPoint = LC_Parameter_Load[LC_ID].Position2.XPoint;
	Temp_LC_Parameter_Load[LC_ID].Position2.YPoint = LC_Parameter_Load[LC_ID].Position2.YPoint;
	Temp_LC_Parameter_Load[LC_ID].Position3.XPoint = LC_Parameter_Load[LC_ID].Position3.XPoint;
	Temp_LC_Parameter_Load[LC_ID].Position3.YPoint = LC_Parameter_Load[LC_ID].Position3.YPoint;
	
	Temp_LC_Parameter_UnLoad.Switch     = LC_Parameter_UnLoad.Switch;
	Temp_LC_Parameter_UnLoad.H_Num      = LC_Parameter_UnLoad.H_Num;
	Temp_LC_Parameter_UnLoad.V_Num      = LC_Parameter_UnLoad.V_Num;
	Temp_LC_Parameter_UnLoad.Position1.XPoint = LC_Parameter_UnLoad.Position1.XPoint;
	Temp_LC_Parameter_UnLoad.Position1.YPoint = LC_Parameter_UnLoad.Position1.YPoint;
	Temp_LC_Parameter_UnLoad.Position2.XPoint = LC_Parameter_UnLoad.Position2.XPoint;
	Temp_LC_Parameter_UnLoad.Position2.YPoint = LC_Parameter_UnLoad.Position2.YPoint;
	Temp_LC_Parameter_UnLoad.Position3.XPoint = LC_Parameter_UnLoad.Position3.XPoint;
	Temp_LC_Parameter_UnLoad.Position3.YPoint = LC_Parameter_UnLoad.Position3.YPoint;

	Temp_SC_Parameter.RW_Num = SC_Parameter.RW_Num;
	Temp_SC_Parameter.CJ_Num = SC_Parameter.CJ_Num;
	Temp_SC_Parameter.JG_Num = SC_Parameter.JG_Num;
	Temp_SC_Parameter.SC_Num = SC_Parameter.SC_Num;
	Temp_SC_Parameter.LJ_Num = SC_Parameter.LJ_Num;
	
	//IO检测参数初始化
	Temp_IO_Detect_Parameter[0]=IO_Detect_Parameter[0];
	Temp_IO_Detect_Parameter[1]=IO_Detect_Parameter[1];
	Temp_IO_Detect_Parameter[2]=IO_Detect_Parameter[2];
	Temp_IO_Detect_Parameter[3]=IO_Detect_Parameter[3];
	Temp_IO_Detect_Parameter[4]=IO_Detect_Parameter[4];
	Temp_IO_Detect_Parameter[5]=IO_Detect_Parameter[5];
	Temp_IO_Detect_Parameter[6]=IO_Detect_Parameter[6];
	Temp_IO_Detect_Parameter[7]=IO_Detect_Parameter[7];
	Temp_IO_Detect_Parameter[8]=IO_Detect_Parameter[8];
	Temp_IO_Detect_Parameter[9]=IO_Detect_Parameter[9];
	Temp_IO_Detect_Parameter[10]=IO_Detect_Parameter[10];
	Temp_IO_Detect_Parameter[11]=IO_Detect_Parameter[11];
	Temp_IO_Detect_Parameter[12]=IO_Detect_Parameter[12];
	Temp_IO_Detect_Parameter[13]=IO_Detect_Parameter[13];
	Temp_IO_Detect_Parameter[14]=IO_Detect_Parameter[14];
	Temp_IO_Detect_Parameter[15]=IO_Detect_Parameter[15];
	Temp_IO_Detect_Parameter[16]=IO_Detect_Parameter[16];
	Temp_IO_Detect_Parameter[17]=IO_Detect_Parameter[17];

	Temp_IO_Detect_Parameter[18]=IO_Detect_Parameter[18];
	Temp_IO_Detect_Parameter[19]=IO_Detect_Parameter[19];
	Temp_IO_Detect_Parameter[20]=IO_Detect_Parameter[20];
	Temp_IO_Detect_Parameter[21]=IO_Detect_Parameter[21];
	Temp_IO_Detect_Parameter[22]=IO_Detect_Parameter[22];
	Temp_IO_Detect_Parameter[23]=IO_Detect_Parameter[23];
	Temp_IO_Detect_Parameter[24]=IO_Detect_Parameter[24];
	Temp_IO_Detect_Parameter[25]=IO_Detect_Parameter[25];
	Temp_IO_Detect_Parameter[26]=IO_Detect_Parameter[26];
	Temp_IO_Detect_Parameter[27]=IO_Detect_Parameter[27];
	Temp_IO_Detect_Parameter[28]=IO_Detect_Parameter[28];
	Temp_IO_Detect_Parameter[29]=IO_Detect_Parameter[29];
	Temp_IO_Detect_Parameter[30]=IO_Detect_Parameter[30];
	Temp_IO_Detect_Parameter[31]=IO_Detect_Parameter[31];
	Temp_IO_Detect_Parameter[32]=IO_Detect_Parameter[32];
	Temp_IO_Detect_Parameter[33]=IO_Detect_Parameter[33];
	Temp_IO_Detect_Parameter[34]=IO_Detect_Parameter[34];
	Temp_IO_Detect_Parameter[35]=IO_Detect_Parameter[35];

	//首页显示
	LCD_Main_Page = START_UP;
	LCD_Current_Page = START_UP;
	PageChange(START_UP);
	DisplayChineseString(0x11F0,Temp_Display_Data,4);
	
	DelayNms(100);
	WriteNum2(0x2630,50); //自由编程-参数-速度设置	

	//伺服器使能
	Robot_Enable_Switch_On = TRUE;
	Robot_Enable = TRUE;
	g_Robot_Enable_Flag = 1;
}

/*************************************************************************
**  函数名：  SavePointManager()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void SavePointManager()
{
	switch(LCD_Var_Address)
	{
		//储存点管理-X轴
		case 0x500C:
			 if(LCD_Current_Page != 41)
			 {
				 LCD_Current_Page = 41;		
				 gs_Current_Choose_Point=1;
				 gs_Pre_Choose_Point=0;
				 gs_Point_Display_Refresh=TRUE;	
			 }			 
			 break;	
			 
		//储存点管理-Z轴
		case 0x500D:
			 if(LCD_Current_Page != 44)
			 {
				 LCD_Current_Page = 44;	
				 gs_Current_Choose_Point=1;
				 gs_Pre_Choose_Point=0;
				 gs_Point_Display_Refresh=TRUE;	
			 }					 
			 break;
		
		//储存点管理-L轴	 
		case 0x500E:
			 if(LCD_Current_Page != 115)
			 {
				 LCD_Current_Page = 115;	
				 gs_Current_Choose_Point=1;
				 gs_Pre_Choose_Point=0;
				 gs_Point_Display_Refresh=TRUE;	
			 }					 
			 break;
			 
		//储存点管理-O轴
		case 0x500F:
			 if(LCD_Current_Page != 123)
			 {
				 LCD_Current_Page = 123;	
				 gs_Current_Choose_Point=1;
				 gs_Pre_Choose_Point=0;
				 gs_Point_Display_Refresh=TRUE;	
			 }					 
			 break;

    //X轴坐标点			 
		case 0x5010:
			 gs_Current_Choose_Point = 1;						 
			 break;	
		case 0x5011:
			 gs_Current_Choose_Point = 2; 				         
			 break;	
		case 0x5012:
		   gs_Current_Choose_Point = 3; 				         
			 break;
		case 0x5013:
			 gs_Current_Choose_Point = 4; 				         
			 break;
		case 0x5014: 						 
			 gs_Current_Choose_Point = 5; 
			 break;	
		case 0x5015: 				        
			 gs_Current_Choose_Point = 6; 
			 break;	
		case 0x5016: 				       
			 gs_Current_Choose_Point = 7; 
			 break;
		case 0x5017: 				       
			 gs_Current_Choose_Point = 8; 
			 break;
		case 0x5018: 				      
 			 gs_Current_Choose_Point = 9; 
 			 break;	
		case 0x5019: 				        
			 gs_Current_Choose_Point = 10; 
		   break;
		case 0x501A: 				        
			 gs_Current_Choose_Point = 11; 
		   break;
		case 0x501B: 				       
			 gs_Current_Choose_Point = 12; 
			 break;
		case 0x501C: 				       
			 gs_Current_Choose_Point = 13; 
			 break;
		case 0x501D: 				      
 			 gs_Current_Choose_Point = 14; 
 			 break;	
		case 0x501E: 				        
			 gs_Current_Choose_Point = 15; 
		   break;
		case 0x501F: 				        
			 gs_Current_Choose_Point = 16; 
		     break;
		case 0x5020:
			 gs_Current_Choose_Point = 17;						 
			 break;	
		case 0x5021:
			 gs_Current_Choose_Point = 18; 				         
			 break;	
		case 0x5022:
		   gs_Current_Choose_Point = 19; 				         
			 break;
		case 0x5023:
			 gs_Current_Choose_Point = 20; 				         
			 break;
		case 0x5024: 						 
			 gs_Current_Choose_Point = 21; 
			 break;	
		case 0x5025: 				        
			 gs_Current_Choose_Point = 22; 
			 break;	
		case 0x5026: 				       
			 gs_Current_Choose_Point = 23; 
			 break;
		case 0x5027: 				       
			 gs_Current_Choose_Point = 24; 
			 break;
		case 0x5028: 				      
 			 gs_Current_Choose_Point = 25; 
 			 break;	
		
		//坐标点-删除
		case 0x5029: 				       
			 if(LCD_Main_Page == SYSTEM_SETTING && Set_LCPosition_XYO==0)
			 {
			 	if(LCD_Current_Page==41||LCD_Current_Page==0x73)   	                      //X轴点删除
				{
					if(Manul_Save_PointX[gs_Current_Choose_Point-1].Flag==1)
					{
						for(i=gs_Current_Choose_Point-1;i<25;i++)
						{
							if(i==24)	                                  //如果删除的是最后一个点
							{
								Manul_Save_PointX[i].Flag=0;
								Manul_Save_PointX[i].Name=0;
								Manul_Save_PointX[i].Point=0;
							}
							else
							{
								Manul_Save_PointX[i].Flag  = Manul_Save_PointX[i+1].Flag;
								Manul_Save_PointX[i].Name  = Manul_Save_PointX[i+1].Name;
								Manul_Save_PointX[i].Point = Manul_Save_PointX[i+1].Point;
					 	  }
					   }										
						//给主控板发送删除指令
						 //程序载入中
						 LCD_Current_Page = 66;
						 LCD_Pre_Page = LCD_Current_Page;
						 PageChange(LCD_Pre_Page);
						Display_PageCut(PageCut_SavePointX);
						g_USART1_DeletePoint_Flag=TRUE;
						Order_Temp[0]=XZAXIS_DELETE_POINT;
						Order_Temp[1]=0;					                     //0删除X点  1删除Z点  
						Order_Temp[2]=gs_Current_Choose_Point;         //删除程序的编号,-1可当数组下标
						USART1_SendData(3,MANUL_OPERATE,Order_Temp);
						g_USART1_DeletePoint_Flag=FALSE;

			  }	
				if(Manul_Save_PointL[gs_Current_Choose_Point-1].Flag==1)
				{
					for(i=gs_Current_Choose_Point-1;i<25;i++)
					{
						if(i==24)	                                     //如果删除的是最后一个点
						{
							Manul_Save_PointL[i].Flag=0;
							Manul_Save_PointL[i].Name=0;
							Manul_Save_PointL[i].Point=0;
						}
						else
						{
							Manul_Save_PointL[i].Flag  = Manul_Save_PointL[i+1].Flag;
							Manul_Save_PointL[i].Name  = Manul_Save_PointL[i+1].Name;
							Manul_Save_PointL[i].Point = Manul_Save_PointL[i+1].Point;
					  }
				   }
					//给主控板发送删除指令
					 //程序载入中
					 LCD_Current_Page = 66;
					 LCD_Pre_Page = LCD_Current_Page;
					 PageChange(LCD_Pre_Page);
					Display_PageCut(PageCut_SavePointL);
					g_USART1_DeletePoint_Flag=TRUE;
					Order_Temp[0]=LOAXIS_DELETE_POINT;
					Order_Temp[1]=0;					                       //0删除L点  1删除O点 
					Order_Temp[2]=gs_Current_Choose_Point;           //删除程序的编号,-1可当数组下标
					USART1_SendData(3,MANUL_OPERATE,Order_Temp);
					g_USART1_DeletePoint_Flag=FALSE;
					LCD_Current_Page=41;			
					gs_Point_Display_Refresh = TRUE;  
				}					
			}  
				else if(LCD_Current_Page==44)                       //Z轴点删除
				{
					if(Manul_Save_PointZ[gs_Current_Choose_Point-1].Flag==1)
					{
						for(i=gs_Current_Choose_Point-1;i<25;i++)
						{
							if(i==24)	                                    //如果删除的是最后一个点
							{
								Manul_Save_PointZ[i].Flag=0;
								Manul_Save_PointZ[i].Name=0;
								Manul_Save_PointZ[i].Point=0;
							}
							else
							{
								Manul_Save_PointZ[i].Flag  = Manul_Save_PointZ[i+1].Flag;
								Manul_Save_PointZ[i].Name  = Manul_Save_PointZ[i+1].Name;
								Manul_Save_PointZ[i].Point = Manul_Save_PointZ[i+1].Point;
					 	  }
					   }
						//给主控板发送删除指令
						 //程序载入中
						 LCD_Current_Page = 66;
						 LCD_Pre_Page = LCD_Current_Page;
						 PageChange(LCD_Pre_Page);
						Display_PageCut(PageCut_SavePointZ);
						g_USART1_DeletePoint_Flag=TRUE;
						Order_Temp[0]=XZAXIS_DELETE_POINT;
					  Order_Temp[1]=1;					                       //0删除X点  1删除Z点  
						Order_Temp[2]=gs_Current_Choose_Point;           //删除程序的编号,-1可当数组下标
						USART1_SendData(3,MANUL_OPERATE,Order_Temp);		
						g_USART1_DeletePoint_Flag=FALSE;
						LCD_Current_Page=44;
						gs_Point_Display_Refresh = TRUE; 
					}				
				}  //if(LCD_Current_Page=44)  //Z轴点删除
       }
			 break;
			 
		//坐标点-返回
		case 0x502A: 				      
 			 if(LCD_Main_Page == SYSTEM_SETTING && Set_LCPosition_XYO==0)
			 {		 	
				LCD_Current_Page = 17;
 			 }
 			 else if(LCD_Main_Page == FREE_PROGRAMME)
			 {		 	
				LCD_Current_Page = 2;
 			 }
			 else if(Set_LCPosition_XYO!=0)
			 {
			 	if((Set_LCPosition_XYO>>8)==0)                          //上料仓
				{
				    LCD_Current_Page = 78;
				}
				else
				{
				    LCD_Current_Page = 120;	                            //卸料仓
				}
				Set_LCPosition_XYO=0;
			 }
			 gs_Current_Choose_Point = 0;
			 break;
 	  default:
		   break;		   	
	}
	if(LCD_Main_Page == FREE_PROGRAMME && gs_Current_Choose_Point!=0)
	{
		switch(g_Current_Paramter_Key)
		{
			case K_XAXIS:
				 Current_Operate_Program.Program[g_Current_Paramter_Num].Value1 = Manul_Save_PointX[gs_Current_Choose_Point-1].Point |0xf0000000 ;
				 break;

			case K_ZAXIS:
				 Current_Operate_Program.Program[g_Current_Paramter_Num].Value1 = Manul_Save_PointZ[gs_Current_Choose_Point-1].Point |0xf0000000 ;
				 break;

			case K_LAXIS:
				 Current_Operate_Program.Program[g_Current_Paramter_Num].Value1 = Manul_Save_PointL[gs_Current_Choose_Point-1].Point |0xf0000000 ;
				 break;

			case K_OAXIS:
				 Current_Operate_Program.Program[g_Current_Paramter_Num].Value1 = Manul_Save_PointO[gs_Current_Choose_Point-1].Point |0xf0000000 ;
				 break;
			
			default:
			   break;
		}
		g_Current_Paramter_Num = 0;
		g_Current_Paramter_Key = 0;
		LCD_Current_Page = 2;
		gs_Current_Choose_Point=0;
		g_Refresh_Dispaly_List_Flag = TRUE;
	}
	else if(Set_LCPosition_XYO!=0 && gs_Current_Choose_Point!=0)	  //料仓示教坐标点设置
	{
		switch(Set_LCPosition_XYO)
		{
			case 11:
				 Temp_LC_Parameter_Load[LC_ID].Position1.XPoint = Manul_Save_PointX[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 12:
				 Temp_LC_Parameter_Load[LC_ID].Position1.YPoint = Manul_Save_PointL[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 13:
				 Temp_LC_Parameter_Load[LC_ID].Position2.XPoint = Manul_Save_PointX[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 14:
				 Temp_LC_Parameter_Load[LC_ID].Position2.YPoint = Manul_Save_PointL[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 15:
				 Temp_LC_Parameter_Load[LC_ID].Position3.XPoint = Manul_Save_PointX[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 16:
				 Temp_LC_Parameter_Load[LC_ID].Position3.YPoint = Manul_Save_PointL[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 17:
				 Temp_LC_Parameter_Load[LC_ID].HT_Hight = Manul_Save_PointZ[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 18:
				 Temp_LC_Parameter_Load[LC_ID].LC_Hight = Manul_Save_PointZ[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			
			case 41:
				 Temp_LC_Parameter_UnLoad.Position1.XPoint = Manul_Save_PointX[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 42:
				 Temp_LC_Parameter_UnLoad.Position1.YPoint = Manul_Save_PointL[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 51:
				 Temp_LC_Parameter_UnLoad.Position2.XPoint = Manul_Save_PointX[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 52:
				 Temp_LC_Parameter_UnLoad.Position2.YPoint = Manul_Save_PointL[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 61:
				 Temp_LC_Parameter_UnLoad.Position3.XPoint = Manul_Save_PointX[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			case 62:
				 Temp_LC_Parameter_UnLoad.Position3.YPoint = Manul_Save_PointL[gs_Current_Choose_Point-1].Point;
				 LCD_Current_Page = 78;
				 break;
			default:
			   break;
		}
		gs_Refresh_LC_Position = TRUE;
		gs_Current_Choose_Point=0;
		Set_LCPosition_XYO=0;
	}
}

/*************************************************************************
**  函数名：  SaveProgramManager()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void SaveProgramManager()
{
	switch(LCD_Var_Address)
	{
		//程序管理-选择
		case 0x5030:
			 gs_Current_Choose_Program = 1;						 
			 break;	
		case 0x5031:
			 gs_Current_Choose_Program = 2; 				         
			 break;	
		case 0x5032:
		   gs_Current_Choose_Program = 3; 				         
			 break;
		case 0x5033:
			 gs_Current_Choose_Program = 4; 				         
			 break;
		case 0x5034: 						 
			 gs_Current_Choose_Program = 5; 
			 break;	
		case 0x5035: 				        
			 gs_Current_Choose_Program = 6; 
			 break;	
		case 0x5036: 				       
			 gs_Current_Choose_Program = 7; 
			 break;
		case 0x5037: 				       
			 gs_Current_Choose_Program = 8; 
			 break;
		case 0x5038: 				      
 			 gs_Current_Choose_Program = 9; 
 			 break;	
		case 0x5039: 				        
			 gs_Current_Choose_Program = 10; 
		   break;
		case 0x503A: 				        
			 //gs_Current_Choose_Program = 11; 
		   break;
		case 0x503B: 				       
			 //gs_Current_Choose_Program = 12; 
			 break;
		case 0x503C: 				       
			 //gs_Current_Choose_Program = 13; 
			 break;
		case 0x503D: 				      
 			 //gs_Current_Choose_Program = 14; 
 			 break;	
		case 0x503E: 				        
			 //gs_Current_Choose_Program = 15; 
		   break;
		case 0x503F: 				        
			 //gs_Current_Choose_Program = 16; 
		   break;
		case 0x5040:
			 //gs_Current_Choose_Program = 17;						 
			 break;	
		case 0x5041:
			 //gs_Current_Choose_Program = 18; 				         
			 break;	
		case 0x5042:
		   //gs_Current_Choose_Program = 19; 				         
			 break;
		case 0x5043:
			 //gs_Current_Choose_Program = 20; 				         
			 break;
		
		//程序-删除
		case 0x5044: 				       
			 if(LCD_Main_Page == FREE_PROGRAMME)              //自由编程调用程序时，删除失效
			 {
			 }
			 else
			 {
				if(Free_Program_Save[gs_Current_Choose_Program-1].Flag==0)
				{
				}
				else
				{			
				  //程序删除中
					 LCD_Current_Page = 73;
					 LCD_Pre_Page = LCD_Current_Page;
					 PageChange(LCD_Pre_Page);
				  //删除料仓程序
					if(gs_Current_Choose_Program>SAVEPROGRAMNUM_4)	
					{
						Free_Program_Save[gs_Current_Choose_Program-1].Flag=0;
						Free_Program_Save[gs_Current_Choose_Program-1].Name=0;
						Free_Program_Save[gs_Current_Choose_Program-1].Num=0;
						if(g_Run_Program_Num == gs_Current_Choose_Program)
						{
						  g_Run_Program_Num=0;
							g_Current_Program_Debug=FALSE;
						}											    
					}
				  //删除主程序
					else
					{
						//记录最大的保存程序个数
						if(Free_Program_Save[SAVEPROGRAMNUM_4-1].Flag == 1)
						{
						 	 gs_Save_Program_Num=SAVEPROGRAMNUM_4;
						}
						else
						{
							 for(i=0;i<SAVEPROGRAMNUM_4;i++)
							 {				 		
							    if(Free_Program_Save[i].Flag==0)
								{
									gs_Save_Program_Num=i;
									break; 
								}
							 }				 
						 }					
						//删除最后那个保存的程序
					 	if(gs_Current_Choose_Program==gs_Save_Program_Num)	  
						{
							Free_Program_Save[gs_Current_Choose_Program-1].Flag=0;
							Free_Program_Save[gs_Current_Choose_Program-1].Name=0;
							Free_Program_Save[gs_Current_Choose_Program-1].Num=0;							
						}
						//删除中间保存程序
						else
						{
							 for(i=gs_Current_Choose_Program-1;i<gs_Save_Program_Num-1;i++)
							 {
								Free_Program_Save[i].Flag = Free_Program_Save[i+1].Flag;	 //编号Code不发生修改
							  Free_Program_Save[i].Name = Free_Program_Save[i+1].Name;
								Free_Program_Save[i].Num  = Free_Program_Save[i+1].Num;			
						 	 }
							  Free_Program_Save[i].Flag = 0;
							  Free_Program_Save[i].Name = 0;
							  Free_Program_Save[i].Num  = 0;			
						}
					 }
					 //给主控板发送删除指令					 					 
					 Order_Temp[0]=PROGRAM_DELETE;
					 Order_Temp[1]=gs_Current_Choose_Program;           //删除程序的编号  -1才可当数组下标
					 g_USART1_SendProgram_Flag=TRUE;
					 USART1_SendData(2,FREE_PROGRAM_SEND,Order_Temp);					 
					 g_USART1_SendProgram_Flag=FALSE;
					 //删除了当前在自由编程中选择的程序，则调试失效，但程序仍然显示
					 if(Current_Operate_Program.Code==gs_Current_Choose_Program)
					 {
						 g_Current_Program_Debug=FALSE;
						 Current_Operate_Program.Code=0;						 					 
					 }
					 if(g_Run_Program_Num == gs_Current_Choose_Program)	  //删除程序正是在运行的程序
					 {
					 	 g_Run_Program_Num = 0;
						 g_Current_Program_Debug=FALSE;									
						 Free_Program_Operate.Flag = 0;
						 Free_Program_Operate.Code = 0;
						 Free_Program_Operate.Name = 0;
						 Free_Program_Operate.Num  = 0;
						 for(j=0;j<LARGESTPROGRAMNUM;j++)
						 {
							 Free_Program_Operate.Program[j].Flag  =0;
							 Free_Program_Operate.Program[j].List  =0;
							 Free_Program_Operate.Program[j].Order =0;
							 Free_Program_Operate.Program[j].Key   =0;
							 Free_Program_Operate.Program[j].Value1=0;
							 Free_Program_Operate.Program[j].Value2=0;
							 Free_Program_Operate.Program[j].Value3=0;
							 Free_Program_Operate.Program[j].Value4=0;
						 }
					 }
					 else if(g_Run_Program_Num<=SAVEPROGRAMNUM_4 && g_Run_Program_Num>gs_Current_Choose_Program)
					 {
					 	 g_Run_Program_Num--;
					 }
					 LCD_Current_Page = 45;
					 gs_Program_Display_Refresh = TRUE;
				   }
			 } 
			 break;
			 
		//程序-返回 	 
		case 0x5045: 				     
 			 if(LCD_Main_Page == FREE_PROGRAMME)
			 {
				 LCD_Current_Page = 2;
			 }
			 else
			 {
				 LCD_Current_Page = 17;
			 }
			 gs_Current_Choose_Program=0;
 			 break;

		//USB拷贝 
		case 0x5046: 				     
 			 if(LCD_Main_Page == FREE_PROGRAMME)
			 {				 
			 }
			 else
			 {
			 	 LCD_Current_Page = 107;
			 }
 			 break;
			
    //USB拷贝：U盘-->控制器     0 			 
		case 0x5047: 				   
			 USBH_Process_Flag=TRUE;
			 USBH_Direction=USBTOCONTROLER;
			 LCD_Current_Page = 108;
			 USBH_Copy_Status = USBHSTATUS_NOUDISK;
			 USBH_Dispaly_Fresh_Flag=1;
			 if(USBH_USR_ApplicationState == USH_USR_FS_NULL)
			 {
			 	USBH_USR_ApplicationState = USH_USR_FS_INIT;
			 }
			 USB_Button_Ok=TRUE;
			 USB_Button_Cancle=TRUE;
 			 break;
			 
		//USB拷贝：控制器-->U盘     1 
		case 0x5048: 				   
			 USBH_Process_Flag=TRUE;
 			 USBH_Direction=CONTROLERTOUSB;
			 LCD_Current_Page = 108;
			 USBH_Copy_Status = USBHSTATUS_NOUDISK;
			 USBH_Dispaly_Fresh_Flag=1;
			 if(USBH_USR_ApplicationState == USH_USR_FS_NULL)
			 {
			 	USBH_USR_ApplicationState = USH_USR_FS_INIT;
			 }
			 USB_Button_Ok=TRUE;
			 USB_Button_Cancle=TRUE;
			 break;
			 
		//USB拷贝：返回	 
		case 0x5049: 				     
			 LCD_Current_Page = 45;
			 gs_Program_Display_Refresh = TRUE;
			 USBH_Process_Flag=FALSE;
 			 USB_Button_Ok=FALSE;
			 USB_Button_Cancle=FALSE;
 			 break;
		
		//U盘状态：确定     拷贝完成或者出错之后才可确定
		case 0x504A: 				   
			 if(USB_Button_Ok)
			 {
			 	LCD_Current_Page = 107;
			 }
 			 break;
			 
		//U盘状态：取消     拷贝完成或者出错之后才可取消
		case 0x504B: 				   
			 if(USB_Button_Cancle)
			 {
			 	LCD_Current_Page = 107;
			 }
 			 break;
 	   default:
		   break;		   	
	}
	if(LCD_Main_Page == FREE_PROGRAMME && gs_Current_Choose_Program!=0)
	{	
		 g_Background_Color=1;
		 gs_Current_Operate_List=1;
		 Current_Operate_Program.Flag= 0;
		 Current_Operate_Program.Code= 0;
		 Current_Operate_Program.Name= 0;
		 Current_Operate_Program.Num = 0;
		 for(i=0;i<LARGESTPROGRAMNUM;i++)
		 {
			Current_Operate_Program.Program[i].Flag   =0;
			Current_Operate_Program.Program[i].List   =0;
			Current_Operate_Program.Program[i].Order  =0;
			Current_Operate_Program.Program[i].Key    =0;
			Current_Operate_Program.Program[i].Value1 =0;
			Current_Operate_Program.Program[i].Value2 =0;
			Current_Operate_Program.Program[i].Value3 =0;
			Current_Operate_Program.Program[i].Value4 =0;				 
		 }		 
		 if(Free_Program_Save[gs_Current_Choose_Program-1].Flag)
		 {
			 //---------程序打开中----------
			 //切换到程序打开提示界面
			 LCD_Current_Page = 125;
			 LCD_Pre_Page = LCD_Current_Page;
			 PageChange(LCD_Pre_Page);
			 //发送当前操作程序编号
			 Order_Temp[0] =ACTION_PROGRAM;
			 Order_Temp[1] =Free_Program_Save[gs_Current_Choose_Program-1].Code;
			 USART1_SendData(2,AUTO_RUN,Order_Temp);
			 i=0x02;                                                 //key
			 if(USART1_ReadProgram(i)==0)                          	 //程序打开中
			 {					 	 
				 g_Run_Program_Num = Current_Operate_Program.Code;
				 //读料仓指令行
			 	 Read_LC_List(g_Run_Program_Num);
				 g_Run_Program_List_Num  = Current_Operate_Program.Num;
				 g_Run_Program_Operater_List = 0;
				 g_Current_Program_Debug=TRUE;
			 }
			 else                                                    //程序打开失败
			 {
			   Robot_Error_Data[0] = Robot_Error_Data[0]|0x20;
			 	 g_Run_Program_Num = 0;
				 //发送当前操作程序编号
				 Order_Temp[0] =ACTION_PROGRAM;
				 Order_Temp[1] =g_Run_Program_Num;
				 USART1_SendData(2,AUTO_RUN,Order_Temp);
			 }
		 }
		 else
		 {
		 	 g_Run_Program_Num = 0;
			 //发送当前操作程序编号
			 Order_Temp[0] =ACTION_PROGRAM;
			 Order_Temp[1] =g_Run_Program_Num;
			 USART1_SendData(2,AUTO_RUN,Order_Temp);		 
		 } 
		 //切换回自由编程界面
		 LCD_Current_Page = 2;
		 g_Refresh_Dispaly_List_Flag = TRUE;
	}
}

/*************************************************************************
**  函数名：  SoftLimitSetting()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
u8 Soft_Limit_Flag=0;//是否对软限位进行修改
void SoftLimitSetting()
{
	switch(LCD_Var_Address)
	{
		case 0x5050://输入实际距离
			 Soft_Limit_Flag=1;
			 Temp_Soft_Limit1[0].Left_Limit = LCD_Var_Data;		//X轴 软限位- 
			 Temp_Soft_Limit[0].Left_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[0];		//X轴 软限位- 
			 break;	
		case 0x5052:
			 Soft_Limit_Flag=1;
			 Temp_Soft_Limit1[0].Right_Limit = LCD_Var_Data;		//X轴 软限位- 
			 Temp_Soft_Limit[0].Right_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[0];		//X轴 软限位+	         
			 break;	
		case 0x5054:
			 Soft_Limit_Flag=1;
			 Temp_Soft_Limit1[1].Left_Limit = LCD_Var_Data;		//X轴 软限位- 
			 Temp_Soft_Limit[1].Left_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[1];		//Z轴 软限位-	         
			 break;
		case 0x5056:
			 Soft_Limit_Flag=1;
			 Temp_Soft_Limit1[1].Right_Limit = LCD_Var_Data;		//X轴 软限位- 
			 Temp_Soft_Limit[1].Right_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[1];		//Z轴 软限位+	         
			 break;
		case 0x5250:
			 Soft_Limit_Flag=1;
			 Temp_Soft_Limit1[2].Left_Limit = LCD_Var_Data;		//X轴 软限位- 
			 Temp_Soft_Limit[2].Left_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[2];		//L轴 软限位-	         
			 break;
		case 0x5252:
			 Soft_Limit_Flag=1;
			 Temp_Soft_Limit1[2].Right_Limit = LCD_Var_Data;		//X轴 软限位- 
			 Temp_Soft_Limit[2].Right_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[2];		//L轴 软限位+	         
			 break;
//		case 0x5254:
//			 Soft_Limit_Flag=1;
//			 Temp_Soft_Limit[3].Left_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[3];		//O轴 软限位-	         
//			 break;
//		case 0x5256:
//			 Soft_Limit_Flag=1;
//			 Temp_Soft_Limit[3].Right_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[3];		//O轴 软限位+	         
//			 break;
		
		//A轴正向行程软限位
		case 0x5058: 						 					 
			 Soft_Limit_Flag=1;
			 Temp_Soft_Limit1[4].Right_Limit = LCD_Var_Data;		//X轴 软限位- 
			 Temp_Soft_Limit[4].Right_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[4];		//A轴 软限位+	         	
			 break;
		//A轴反向行程软限位
		case 0x505A: 				        
			 Soft_Limit_Flag=1;
			 Temp_Soft_Limit1[4].Left_Limit = LCD_Var_Data;		//X轴 软限位- 
			 Temp_Soft_Limit[4].Left_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[4];		//A轴 软限位-	         			
			 break;					 
		//B轴正向行程软限位	 
		case 0x5258: 			        
			 Soft_Limit_Flag=1;
			 Temp_Soft_Limit1[5].Right_Limit = LCD_Var_Data;		//X轴 软限位- 
			 Temp_Soft_Limit[5].Right_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[5];		//B轴 软限位+	         	
			 break;	
		//B轴反向行程软限位	 
		case 0x525A: 				        
			 Soft_Limit_Flag=1;
			 Temp_Soft_Limit1[5].Left_Limit = LCD_Var_Data;		//X轴 软限位- 
			 Temp_Soft_Limit[5].Left_Limit = LCD_Var_Data*JXS_Parameter.GearRatio.Ratio[5];		//B轴 软限位-	         			
			 break;			
    //软限位-取消			 
		case 0x5059: 				        
			 LCD_Current_Page = 17;					    
		if(((Temp_Soft_Limit[0].Left_Limit>Temp_Soft_Limit[0].Right_Limit)||(Temp_Soft_Limit[0].Right_Limit<g_XAxis_Position))) 
			 {
			 	LCD_Popup_Windows = PW_5_SOFTLIMITERROR ;	//弹出提示框
				gs_Reflash_SoftLimit=TRUE;
			 }
			 else if(((Temp_Soft_Limit[1].Left_Limit>Temp_Soft_Limit[1].Right_Limit)||(Temp_Soft_Limit[1].Right_Limit<g_ZAxis_Position))) 
			 {
			 	LCD_Popup_Windows = PW_5_SOFTLIMITERROR ;	//弹出提示框
				gs_Reflash_SoftLimit=TRUE;
			 }
			 else if(((Temp_Soft_Limit[2].Left_Limit>Temp_Soft_Limit[2].Right_Limit)||(Temp_Soft_Limit[2].Right_Limit<g_LAxis_Position))) 
			 {
			 	LCD_Popup_Windows = PW_5_SOFTLIMITERROR ;	//弹出提示框
				gs_Reflash_SoftLimit=TRUE;
			 }			 		 
//			 else if(((Temp_Soft_Limit[3].Left_Limit>=Temp_Soft_Limit[3].Right_Limit)||(Temp_Soft_Limit[3].Right_Limit<g_OAxis_Position))) 
//			 {
//			 	LCD_Popup_Windows = PW_5_SOFTLIMITERROR ;	//弹出提示框
//				gs_Reflash_SoftLimit=TRUE;
//			 }	
			 else if(((Temp_Soft_Limit[4].Left_Limit>=Temp_Soft_Limit[4].Right_Limit)||(Temp_Soft_Limit[4].Right_Limit<g_AAxis_Position))) 
			 {
			 	LCD_Popup_Windows = PW_5_SOFTLIMITERROR ;	//弹出提示框
				gs_Reflash_SoftLimit=TRUE;
			 }
			 else if(((Temp_Soft_Limit[5].Left_Limit>=Temp_Soft_Limit[5].Right_Limit)||(Temp_Soft_Limit[5].Right_Limit<g_BAxis_Position))) 
			 {
			 	LCD_Popup_Windows = PW_5_SOFTLIMITERROR ;	//弹出提示框
				gs_Reflash_SoftLimit=TRUE;
			 }
			 else if(Soft_Limit_Flag)
			 {
				 Soft_Limit_Flag=0;
				 Robot_SoftLimit[0].Left_Limit  =Temp_Soft_Limit[0].Left_Limit;	     //X-
				 Robot_SoftLimit[0].Right_Limit =Temp_Soft_Limit[0].Right_Limit;	   //X+
				 Robot_SoftLimit[1].Left_Limit  =Temp_Soft_Limit[1].Left_Limit;	     //Z-
				 Robot_SoftLimit[1].Right_Limit =Temp_Soft_Limit[1].Right_Limit;	   //Z+
				 Robot_SoftLimit[2].Left_Limit  =Temp_Soft_Limit[2].Left_Limit;	     //L-
				 Robot_SoftLimit[2].Right_Limit =Temp_Soft_Limit[2].Right_Limit;	   //L+
				 Robot_SoftLimit[3].Left_Limit  =Temp_Soft_Limit[3].Left_Limit;	     //O-
				 Robot_SoftLimit[3].Right_Limit =Temp_Soft_Limit[3].Right_Limit;	   //O+
				 Robot_SoftLimit[4].Right_Limit =Temp_Soft_Limit[4].Right_Limit;	   //A+
				 Robot_SoftLimit[4].Left_Limit  =Temp_Soft_Limit[4].Left_Limit;	     //A-
				 Robot_SoftLimit[5].Right_Limit =Temp_Soft_Limit[5].Right_Limit;	   //B+
				 Robot_SoftLimit[5].Left_Limit  =Temp_Soft_Limit[5].Left_Limit;	     //B-
				 //数据下发
				 Order_Temp[0] =PARAMETER_SOFT_LIMIT;	                               //命令
				 Order_Temp[1] =0x01;	                                               //X
				 Order_Temp[2] =Robot_SoftLimit[0].Left_Limit;
				 Order_Temp[3] =Robot_SoftLimit[0].Left_Limit>>8;
				 Order_Temp[4] =Robot_SoftLimit[0].Left_Limit>>16;
				 Order_Temp[5] =Robot_SoftLimit[0].Left_Limit>>24;
				 Order_Temp[6] =Robot_SoftLimit[0].Right_Limit;
				 Order_Temp[7] =Robot_SoftLimit[0].Right_Limit>>8;
				 Order_Temp[8] =Robot_SoftLimit[0].Right_Limit>>16;
				 Order_Temp[9] =Robot_SoftLimit[0].Right_Limit>>24;
				 Order_Temp[10]=Robot_SoftLimit[0].Switch_Limit;
				 USART1_SendData(11,PARAMETER_ORDER,Order_Temp);
				 
				 Order_Temp[0] =PARAMETER_SOFT_LIMIT;	                               //命令
				 Order_Temp[1] =0x02;	                                               //Z
				 Order_Temp[2] =Robot_SoftLimit[1].Left_Limit ;
				 Order_Temp[3] =Robot_SoftLimit[1].Left_Limit >>8;
				 Order_Temp[4] =Robot_SoftLimit[1].Left_Limit >>16;
				 Order_Temp[5] =Robot_SoftLimit[1].Left_Limit >>24;
				 Order_Temp[6] =Robot_SoftLimit[1].Right_Limit ;
				 Order_Temp[7] =Robot_SoftLimit[1].Right_Limit >>8;
				 Order_Temp[8] =Robot_SoftLimit[1].Right_Limit >>16;
				 Order_Temp[9] =Robot_SoftLimit[1].Right_Limit >>24;
				 Order_Temp[10]=Robot_SoftLimit[1].Switch_Limit;
				 USART1_SendData(11,PARAMETER_ORDER,Order_Temp);
				 
				 Order_Temp[0] =PARAMETER_SOFT_LIMIT;	                               //命令
				 Order_Temp[1] =0x03;	                                               //L
				 Order_Temp[2] =Robot_SoftLimit[2].Left_Limit ;
				 Order_Temp[3] =Robot_SoftLimit[2].Left_Limit >>8;
				 Order_Temp[4] =Robot_SoftLimit[2].Left_Limit >>16;
				 Order_Temp[5] =Robot_SoftLimit[2].Left_Limit >>24;
				 Order_Temp[6] =Robot_SoftLimit[2].Right_Limit ;
				 Order_Temp[7] =Robot_SoftLimit[2].Right_Limit >>8;
				 Order_Temp[8] =Robot_SoftLimit[2].Right_Limit >>16;
				 Order_Temp[9] =Robot_SoftLimit[2].Right_Limit >>24;
				 Order_Temp[10]=Robot_SoftLimit[2].Switch_Limit;
				 USART1_SendData(11,PARAMETER_ORDER,Order_Temp);

				 Order_Temp[0] =PARAMETER_SOFT_LIMIT;	                               //命令
				 Order_Temp[1] =0x04;	                                               //O
				 Order_Temp[2] =Robot_SoftLimit[3].Left_Limit ;
				 Order_Temp[3] =Robot_SoftLimit[3].Left_Limit >>8;
				 Order_Temp[4] =Robot_SoftLimit[3].Left_Limit >>16;
				 Order_Temp[5] =Robot_SoftLimit[3].Left_Limit >>24;
				 Order_Temp[6] =Robot_SoftLimit[3].Right_Limit ;
				 Order_Temp[7] =Robot_SoftLimit[3].Right_Limit >>8;
				 Order_Temp[8] =Robot_SoftLimit[3].Right_Limit >>16;
				 Order_Temp[9] =Robot_SoftLimit[3].Right_Limit >>24;
				 Order_Temp[10]=Robot_SoftLimit[3].Switch_Limit;
				 USART1_SendData(11,PARAMETER_ORDER,Order_Temp);

				 Order_Temp[0] =PARAMETER_SOFT_LIMIT;	                               //命令
				 Order_Temp[1] =0x05;	                                               //A
				 Order_Temp[2] =Robot_SoftLimit[4].Left_Limit ;
				 Order_Temp[3] =Robot_SoftLimit[4].Left_Limit >>8;
				 Order_Temp[4] =Robot_SoftLimit[4].Left_Limit >>16;
				 Order_Temp[5] =Robot_SoftLimit[4].Left_Limit >>24;
				 Order_Temp[6] =Robot_SoftLimit[4].Right_Limit ;
				 Order_Temp[7] =Robot_SoftLimit[4].Right_Limit >>8;
				 Order_Temp[8] =Robot_SoftLimit[4].Right_Limit >>16;
				 Order_Temp[9] =Robot_SoftLimit[4].Right_Limit >>24;
				 Order_Temp[10]=Robot_SoftLimit[4].Switch_Limit;
				 USART1_SendData(11,PARAMETER_ORDER,Order_Temp);
				
				 Order_Temp[0] =PARAMETER_SOFT_LIMIT;	                               //命令
				 Order_Temp[1] =0x06;	                                               //B
				 Order_Temp[2] =Robot_SoftLimit[5].Left_Limit ;
				 Order_Temp[3] =Robot_SoftLimit[5].Left_Limit >>8;
				 Order_Temp[4] =Robot_SoftLimit[5].Left_Limit >>16;
				 Order_Temp[5] =Robot_SoftLimit[5].Left_Limit >>24;
				 Order_Temp[6] =Robot_SoftLimit[5].Right_Limit ;
				 Order_Temp[7] =Robot_SoftLimit[5].Right_Limit >>8;
				 Order_Temp[8] =Robot_SoftLimit[5].Right_Limit >>16;
				 Order_Temp[9] =Robot_SoftLimit[5].Right_Limit >>24;
				 Order_Temp[10]=Robot_SoftLimit[5].Switch_Limit;
				 USART1_SendData(11,PARAMETER_ORDER,Order_Temp);
			 }
			 break;	 
 	  default:
		   break;		   	
	}
}

/*************************************************************************
**  函数名：  UserSetting()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void UserSetting()
{
	switch(LCD_Var_Address)
	{
		//切换用户
		case 0x5500:	
       LCD_Current_Page = 22;	                   //切换到登陆界面
			 Change_User=TRUE;
			 WriteNum4(0x1005,0);                      //清除登录密码显示		 
			 break;	

		//蜂鸣器
		case 0x5060:	
			 if(g_Buzzer_Switch_Flag == TRUE)
			 {
			 	g_Buzzer_Switch_Flag = FALSE;
			 }
			 else
			 {
			 	g_Buzzer_Switch_Flag = TRUE;
			 }
			 break;	
			 
		//背光控制
		case 0x5062:   
	     g_BackLight_Time = LCD_Var_Data;
			 if(g_BackLight_Time<10)
			 {
			 	g_BackLight_Time = 10;
			 }
			 if(g_BackLight_Time>3600)
			 {
			 	g_BackLight_Time = 3600;
			 }
			 IIC_Temp[0] = g_BackLight_Time;
			 IIC_Temp[1] = g_BackLight_Time>>8;
			 IIC_Temp[2] = g_BackLight_Time>>16;
			 IIC_Temp[3] = g_BackLight_Time>>24;
			 AT24CXX_Write(0x0008,IIC_Temp,4);
			 gs_BackLight_Refresh = 1;
			 break;
			 
		//修改密码	 
		case 0x5066:
       LCD_Current_Page = 53;		
			 gs_Pass_Word_Refresh=1;
			 break;
		
		//用户设置-返回
		case 0x5067:
       LCD_Current_Page = 17;		
			 break;
		
		//修改密码-原密码
 		case 0x5068:				 	
       gs_Pass_Word_Temp[0]=LCD_Var_Data; 	
			 break;	
		
		//修改密码-新密码
		case 0x506A:					       	 
			 if(LCD_Var_Data<100000)
			 {
			 	//弹框提示-请输入有效的六位密码
				LCD_Popup_Windows = 0x90;		
			 } 
			 else
			 {
			 	gs_Pass_Word_Temp[1]=LCD_Var_Data;
			 }		
			 break;
		
    //修改密码-确认新密码			 
		case 0x506C:					
       gs_Pass_Word_Temp[2]=LCD_Var_Data; 
			 if(gs_Pass_Word_Temp[2]!=gs_Pass_Word_Temp[1])
			 {
				//弹框提示-两次输入新密码不一致
				LCD_Popup_Windows = 0xA0;	 
			 }		
			 break;

    //修改密码-确定			 
		case 0x506E:
		   if((Robot_Admin_Authority==ORDINARY_AUTHORITY) && (gs_Pass_Word_Temp[0] == Pass_Word[0])) //普通权限
			 {
        LCD_Current_Page = 49;		
				Pass_Word[0]=gs_Pass_Word_Temp[1];
				IIC_Temp[0] = Pass_Word[0];
				IIC_Temp[1] = Pass_Word[0]>>8;
				IIC_Temp[2] = Pass_Word[0]>>16;
				IIC_Temp[3] = Pass_Word[0]>>24;
				AT24CXX_Write(0x0000,IIC_Temp,4);
			 }
			 else if((Robot_Admin_Authority==MANAGE_AUTHORITY)&&(gs_Pass_Word_Temp[0] == Pass_Word[1])) //管理员权限
			 {
        LCD_Current_Page = 49;		
				Pass_Word[1]=gs_Pass_Word_Temp[1];
				IIC_Temp[0] = Pass_Word[1];
				IIC_Temp[1] = Pass_Word[1]>>8;
				IIC_Temp[2] = Pass_Word[1]>>16;
				IIC_Temp[3] = Pass_Word[1]>>24;
				AT24CXX_Write(0x0004,IIC_Temp,4);			 
			 }
			 else
			 {
			 	//弹框提示-原密码错误
				LCD_Popup_Windows = 0x80;	
			 }
			 break;	
		
    //修改密码-取消			 
		case 0x506F:
       LCD_Current_Page = 49;		
			 break;	
		
		//修改加密锁
		case 0x5501:	
       if(Robot_Admin_Authority==MANAGE_AUTHORITY)
			 {
			 	 LCD_Current_Page = 104;	                        //切换到加密锁界面
			   if(UnLock_Forever_flag==1) 
				 {
					 CheckTime();
					 DisplayChineseString(0x5540,Beyond_Forever,30);//显示开机超期
				 }	
			 }
			 break;	
			 
		//修改加密锁-确定		 
		case 0x5530:		 
			 break;	
		
		//修改加密锁-返回
		case 0x5531:	
			 LCD_Current_Page = 49;	
			 WriteNum4(0x5520,0);
			 break;
 	  default:
		   break;		   	
	}
	if(LCD_Current_Page == 104)
	{
		UnlockCodeInput();
	}
}



/*************************************************************************
**  函数名：  SafeAreaSetting()
**	输入参数：
**	输出参数：无
**	函数功能：安全区域设定
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void SafeAreaSetting()
{
	u16 temp=0;
	switch(LCD_Var_Address)
	{
		case 0x5073://设为顶部
			 Rod_X_Positive = After_filter[0];
			 Order_Temp[0]=Rod_X_Positive;
			 Order_Temp[1]=Rod_X_Positive>>8;
			 AT24CXX_Write(0x0800,Order_Temp,2);
			 //X轴
			 temp=Rod_X_Positive-Rod_Centry_X-100;
			 temp=temp>>3;
			 Rod_Step_X[4]=Rod_Centry_X+100;
			 Rod_Step_X[5]=Rod_Centry_X+50+temp*3;
			 Rod_Step_X[6]=Rod_Centry_X+50+temp*6;
			 Rod_Step_X[7]=Rod_X_Positive-50;
			 temp=Rod_Centry_X-Rod_X_Negative-100;
			 temp=temp>>3;
			 Rod_Step_X[0]=Rod_X_Negative+50;
			 Rod_Step_X[1]=Rod_X_Negative+50+temp*3;
			 Rod_Step_X[2]=Rod_X_Negative+50+temp*6;
			 Rod_Step_X[3]=Rod_Centry_X-100;
			//Y轴
			 temp=Rod_Y_Positive-Rod_Centry_Y-100;
			 temp=temp>>3;
			 Rod_Step_Y[4]=Rod_Centry_Y+100;
			 Rod_Step_Y[5]=Rod_Centry_Y+50+temp*3;
			 Rod_Step_Y[6]=Rod_Centry_Y+50+temp*6;
			 Rod_Step_Y[7]=Rod_Y_Positive-50;
			 temp=Rod_Centry_Y-Rod_Y_Negative-100;
			 temp=temp>>3;
			 Rod_Step_Y[0]=Rod_Y_Negative+100;
			 Rod_Step_Y[1]=Rod_Y_Negative+50+temp*3;
			 Rod_Step_Y[2]=Rod_Y_Negative+50+temp*6;
			 Rod_Step_Y[3]=Rod_Centry_Y-100;			 
			 break;	
		case 0x5074://设为底部        
			 Rod_X_Negative = After_filter[0];
			 Order_Temp[0]=Rod_X_Negative;
			 Order_Temp[1]=Rod_X_Negative>>8;
			 AT24CXX_Write(0x0802,Order_Temp,2);
			 //X轴
			 temp=Rod_X_Positive-Rod_Centry_X-100;
			 temp=temp>>3;
			 Rod_Step_X[4]=Rod_Centry_X+100;
			 Rod_Step_X[5]=Rod_Centry_X+50+temp*3;
			 Rod_Step_X[6]=Rod_Centry_X+50+temp*6;
			 Rod_Step_X[7]=Rod_X_Positive-50;
			 temp=Rod_Centry_X-Rod_X_Negative-100;
			 temp=temp>>3;
			 Rod_Step_X[0]=Rod_X_Negative+100;
			 Rod_Step_X[1]=Rod_X_Negative+50+temp*3;
			 Rod_Step_X[2]=Rod_X_Negative+50+temp*6;
			 Rod_Step_X[3]=Rod_Centry_X-100;
			//Y轴
			 temp=Rod_Y_Positive-Rod_Centry_Y-100;
			 temp=temp>>3;
			 Rod_Step_Y[4]=Rod_Centry_Y+100;
			 Rod_Step_Y[5]=Rod_Centry_Y+50+temp*3;
			 Rod_Step_Y[6]=Rod_Centry_Y+50+temp*6;
			 Rod_Step_Y[7]=Rod_Y_Positive-50;
			 temp=Rod_Centry_Y-Rod_Y_Negative-100;
			 temp=temp>>3;
			 Rod_Step_Y[0]=Rod_Y_Negative+50;
			 Rod_Step_Y[1]=Rod_Y_Negative+50+temp*3;
			 Rod_Step_Y[2]=Rod_Y_Negative+50+temp*6;
			 Rod_Step_Y[3]=Rod_Centry_Y-100;
			 break;
		case 0x5071://设为中心        
			 Rod_Centry_X     = After_filter[0];
			 Rod_Centry_Y     = After_filter[1];
			 Order_Temp[0]=Rod_Centry_X;
			 Order_Temp[1]=Rod_Centry_X>>8;
			 Order_Temp[2]=Rod_Centry_Y;
			 Order_Temp[3]=Rod_Centry_Y>>8;
			 AT24CXX_Write(0x0804,Order_Temp,4);
			 //X轴
			 temp=Rod_X_Positive-Rod_Centry_X-100;
			 temp=temp>>3;
			 Rod_Step_X[4]=Rod_Centry_X+100;
			 Rod_Step_X[5]=Rod_Centry_X+50+temp*3;
			 Rod_Step_X[6]=Rod_Centry_X+50+temp*6;
			 Rod_Step_X[7]=Rod_X_Positive-50;
			 temp=Rod_Centry_X-Rod_X_Negative-100;
			 temp=temp>>3;
			 Rod_Step_X[0]=Rod_X_Negative+50;
			 Rod_Step_X[1]=Rod_X_Negative+50+temp*3;
			 Rod_Step_X[2]=Rod_X_Negative+50+temp*6;
			 Rod_Step_X[3]=Rod_Centry_X-100;
			//Y轴
			 temp=Rod_Y_Positive-Rod_Centry_Y-100;
			 temp=temp>>3;
			 Rod_Step_Y[4]=Rod_Centry_Y+100;
			 Rod_Step_Y[5]=Rod_Centry_Y+50+temp*3;
			 Rod_Step_Y[6]=Rod_Centry_Y+50+temp*6;
			 Rod_Step_Y[7]=Rod_Y_Positive-50;
			 temp=Rod_Centry_Y-Rod_Y_Negative-100;
			 temp=temp>>3;
			 Rod_Step_Y[0]=Rod_Y_Negative+50;
			 Rod_Step_Y[1]=Rod_Y_Negative+50+temp*3;
			 Rod_Step_Y[2]=Rod_Y_Negative+50+temp*6;
			 Rod_Step_Y[3]=Rod_Centry_Y-100;
			 break;
		case 0x5070://设为左侧
			 Rod_Y_Negative = After_filter[1];
			 Order_Temp[0]=Rod_Y_Negative;
			 Order_Temp[1]=Rod_Y_Negative>>8;
			 AT24CXX_Write(0x0808,Order_Temp,2);
			 //X轴
			 temp=Rod_X_Positive-Rod_Centry_X-100;
			 temp=temp>>3;
			 Rod_Step_X[4]=Rod_Centry_X+100;
			 Rod_Step_X[5]=Rod_Centry_X+50+temp*3;
			 Rod_Step_X[6]=Rod_Centry_X+50+temp*6;
			 Rod_Step_X[7]=Rod_X_Positive-50;
			 temp=Rod_Centry_X-Rod_X_Negative-100;
			 temp=temp>>3;
			 Rod_Step_X[0]=Rod_X_Negative+50;
			 Rod_Step_X[1]=Rod_X_Negative+50+temp*3;
			 Rod_Step_X[2]=Rod_X_Negative+50+temp*6;
			 Rod_Step_X[3]=Rod_Centry_X-100;
			//Y轴
			 temp=Rod_Y_Positive-Rod_Centry_Y-100;
			 temp=temp>>3;
			 Rod_Step_Y[4]=Rod_Centry_Y+100;
			 Rod_Step_Y[5]=Rod_Centry_Y+50+temp*3;
			 Rod_Step_Y[6]=Rod_Centry_Y+50+temp*6;
			 Rod_Step_Y[7]=Rod_Y_Positive-50;
			 temp=Rod_Centry_Y-Rod_Y_Negative-100;
			 temp=temp>>3;
			 Rod_Step_Y[0]=Rod_Y_Negative+50;
			 Rod_Step_Y[1]=Rod_Y_Negative+50+temp*3;
			 Rod_Step_Y[2]=Rod_Y_Negative+50+temp*6;
			 Rod_Step_Y[3]=Rod_Centry_Y-100;
			 break;
 		case 0x5072://设为右侧         
			 Rod_Y_Positive = After_filter[1];
			 Order_Temp[0]=Rod_Y_Positive;
			 Order_Temp[1]=Rod_Y_Positive>>8;
			 AT24CXX_Write(0x080A,Order_Temp,2);
			 //X轴
			 temp=Rod_X_Positive-Rod_Centry_X-100;
			 temp=temp>>3;
			 Rod_Step_X[4]=Rod_Centry_X+100;
			 Rod_Step_X[5]=Rod_Centry_X+50+temp*3;
			 Rod_Step_X[6]=Rod_Centry_X+50+temp*6;
			 Rod_Step_X[7]=Rod_X_Positive-50;
			 temp=Rod_Centry_X-Rod_X_Negative-100;
			 temp=temp>>3;
			 Rod_Step_X[0]=Rod_X_Negative+50;
			 Rod_Step_X[1]=Rod_X_Negative+50+temp*3;
			 Rod_Step_X[2]=Rod_X_Negative+50+temp*6;
			 Rod_Step_X[3]=Rod_Centry_X-100;
			//Y轴
			 temp=Rod_Y_Positive-Rod_Centry_Y-100;
			 temp=temp>>3;
			 Rod_Step_Y[4]=Rod_Centry_Y+100;
			 Rod_Step_Y[5]=Rod_Centry_Y+50+temp*3;
			 Rod_Step_Y[6]=Rod_Centry_Y+50+temp*6;
			 Rod_Step_Y[7]=Rod_Y_Positive-50;
			 temp=Rod_Centry_Y-Rod_Y_Negative-100;
			 temp=temp>>3;
			 Rod_Step_Y[0]=Rod_Y_Negative+50;
			 Rod_Step_Y[1]=Rod_Y_Negative+50+temp*3;
			 Rod_Step_Y[2]=Rod_Y_Negative+50+temp*6;
			 Rod_Step_Y[3]=Rod_Centry_Y-100;
			 break;	 
		//摇杆校准-返回
		case 0x5080:
			 LCD_Current_Page = 17;		                 //返回	
			 break;	
 	   default:
		   break;		   	
	}
}

/*************************************************************************
**  函数名：  FunctionSetting()
**	输入参数：
**	输出参数：无
**	函数功能：功能设定
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/11/11
**************************************************************************/
void FunctionSetting()
{
	switch(LCD_Var_Address)
	{
		case 0x5330://--示教低速度
			DZJSpeed1.ManualSpeed_L[0]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x01;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.ManualSpeed_L[0];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x5332://--示教高速度
			DZJSpeed1.ManualSpeed_H[0]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x02;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.ManualSpeed_H[0];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);
		break;
		case 0x5334://--开始速度
			DZJSpeed1.AutoSpeed_L[0]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x03;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.AutoSpeed_L[0];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);
		break;
		case 0x5336://--最高速度
			DZJSpeed1.AutoSpeed_H[0]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x04;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.AutoSpeed_H[0];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);
			break;
		case 0x5338://--加速度
			DZJSpeed1.Acceleration_UP[0]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x05;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.Acceleration_UP[0];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);
			break;
		case 0x533A://--减速度
			DZJSpeed1.Acceleration_DOWN[0]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x06;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.Acceleration_DOWN[0];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);
			break;
		case 0x5340://--A轴示教低速度
			DZJSpeed1.ManualSpeed_L[1]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x07;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.ManualSpeed_L[1];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x5342://--A轴示教高速度
			DZJSpeed1.ManualSpeed_H[1]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x08;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.ManualSpeed_H[1];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x5344://--A轴开始速度
			DZJSpeed1.AutoSpeed_L[1]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x09;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.AutoSpeed_L[1];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x5346://--A轴最高速度
			DZJSpeed1.AutoSpeed_H[1]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x0A;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.AutoSpeed_H[1];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x5348://--A轴加速度
			DZJSpeed1.Acceleration_UP[1]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x0B;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.Acceleration_UP[1];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x534A://--A轴减速度
			DZJSpeed1.Acceleration_DOWN[1]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x0C;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.Acceleration_DOWN[1];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x5350://--B轴示教低速度
			DZJSpeed1.ManualSpeed_L[2]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x0D;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.ManualSpeed_L[2];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x5352://--B轴示教高速度
			DZJSpeed1.ManualSpeed_H[2]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x0E;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.ManualSpeed_H[2];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x5354://--B轴开始速度
			DZJSpeed1.AutoSpeed_L[2]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x0F;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.AutoSpeed_L[2];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x5356://--B轴最高速度
			DZJSpeed1.AutoSpeed_H[2]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x10;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.AutoSpeed_H[2];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x5358://--B轴加速度
			DZJSpeed1.Acceleration_UP[2]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x11;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.Acceleration_UP[2];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		case 0x535A://--B轴减速度
			DZJSpeed1.Acceleration_DOWN[2]=LCD_Var_Data/100;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x09;						               //料仓参数
			 Order_Temp[2] =0x12;						               //料仓参数
			 Order_Temp[3] =DZJSpeed1.Acceleration_DOWN[2];
			 USART1_SendData(4,PARAMETER_ORDER,Order_Temp);			
			break;
		//---功能设定--机械手
		case 0x5300: 
		   LCD_Current_Page= 76;
//			 Temp_JXS_Parameter.Axis       = JXS_Parameter.Axis;
//			 Temp_JXS_Parameter.Origin     = JXS_Parameter.Origin;
//			 Temp_JXS_Parameter.SpeedLevel = JXS_Parameter.SpeedLevel;
//			 Temp_JXS_Parameter.ManualSpeed = JXS_Parameter.ManualSpeed;
//			 Temp_JXS_Parameter.PulseTime  = JXS_Parameter.PulseTime;
//			 Temp_JXS_Parameter.ServoAlarm  = JXS_Parameter.ServoAlarm;   //---2017.12.10设置伺服报警信号
//			 Temp_JXS_Parameter.NcComplete  = JXS_Parameter.NcComplete;//---2018.3.20设置数控加工等待时间
			 gs_Reflash_Fuction_JXS = TRUE;
			 break;

    //--- 功能设定--料仓
		case 0x5301:         
			 LCD_Current_Page= 78;
			 Temp_LC_Parameter_Load[LC_ID].ID   			 = LC_Parameter_Load[LC_ID].ID;
			 Temp_LC_Parameter_Load[LC_ID].Switch     = LC_Parameter_Load[LC_ID].Switch;
			 Temp_LC_Parameter_Load[LC_ID].H_Num      = LC_Parameter_Load[LC_ID].H_Num;
			 Temp_LC_Parameter_Load[LC_ID].V_Num      = LC_Parameter_Load[LC_ID].V_Num;
			 Temp_LC_Parameter_Load[LC_ID].Glue_Time     = LC_Parameter_Load[LC_ID].Glue_Time;
			 Temp_LC_Parameter_Load[LC_ID].Diamonds_Time = LC_Parameter_Load[LC_ID].Diamonds_Time;			 
			 Temp_LC_Parameter_Load[LC_ID].HT_Hight      = LC_Parameter_Load[LC_ID].HT_Hight;
			 Temp_LC_Parameter_Load[LC_ID].LC_Hight      = LC_Parameter_Load[LC_ID].LC_Hight;			 	 
			 Temp_LC_Parameter_Load[LC_ID].Position1.XPoint = LC_Parameter_Load[LC_ID].Position1.XPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position1.YPoint = LC_Parameter_Load[LC_ID].Position1.YPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position2.XPoint = LC_Parameter_Load[LC_ID].Position2.XPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position2.YPoint = LC_Parameter_Load[LC_ID].Position2.YPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position3.XPoint = LC_Parameter_Load[LC_ID].Position3.XPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position3.YPoint = LC_Parameter_Load[LC_ID].Position3.YPoint;
			 gs_Reflash_Fuction_LC = TRUE;
			 break;
		
		//---功能设定--生产
		case 0x5302:
			 LCD_Current_Page= 80;
			 Temp_SC_Parameter.RW_Num = SC_Parameter.RW_Num;
			 Temp_SC_Parameter.CJ_Num = SC_Parameter.CJ_Num;
			 Temp_SC_Parameter.JG_Num = SC_Parameter.JG_Num;
			 Temp_SC_Parameter.SC_Num = SC_Parameter.SC_Num;
			 Temp_SC_Parameter.LJ_Num = SC_Parameter.LJ_Num;
			 g_Reflash_Progress_Num=TRUE;
			 gs_Reflash_Fuction_SC=TRUE;
			 break;
		
		//---机械手--轴个数
 		case 0x5303:  
		   if(Admin_Authority) //管理员权限
			 {
//				 Temp_JXS_Parameter.Axis++;
//				if(Temp_JXS_Parameter.Axis==3)
//				 {
//				 	  Temp_JXS_Parameter.Axis=FAN_X_Z_L;
//				 }
//				 //跟随修改回原点方式
//				if(Temp_JXS_Parameter.Axis==FAN_X_Z_L)
//				 {
//				    Temp_JXS_Parameter.Origin=FOM_Z_L_X;          //2  ---这个应该是3吧？？？
//				 }
//				 else if(Temp_JXS_Parameter.Axis==FAN_X_Z_L_A_B)
//				 {
//					 Temp_JXS_Parameter.Origin=FOM_Z_A_B_L_X;			//4   ---6
//				 }
//				 else 
//				 {
//					 Temp_JXS_Parameter.Origin=FOM_Z_A_B_L_X_O;		//5  ----7
//				 }
//				 gs_Refresh_Axis_Num=TRUE; 
//				 gs_Refresh_Origin_Method=TRUE;
			 }
			 break;
			 
		//---机械手--原点方式	 
		case 0x5304:  
		    if(Admin_Authority)                               //管理员权限
			 {
//				 if(Temp_JXS_Parameter.Origin==FOM_Z_L_X)
//				 {
//					   Temp_JXS_Parameter.Origin=FOM_Z_L_X;
//				 } 
//				 else if(Temp_JXS_Parameter.Origin==FOM_Z_A_B_L_X)
//				 {
//					 Temp_JXS_Parameter.Origin=FOM_Z_A_B_L_X;			//4   ---7					 
//				 }
//				 else if(Temp_JXS_Parameter.Origin==FOM_Z_A_B_L_X_O)
//				 {
//					 Temp_JXS_Parameter.Origin=FOM_Z_A_B_L_X_O;
//				 }
//				 gs_Refresh_Origin_Method=TRUE;
			 }
			 break;
			 
		//---机械手--加速度时间	 
		case 0x5305:  
		   if(Admin_Authority)
			 { 
 				  Temp_JXS_Parameter.Accelerate.Time[Temp_ACC_Axis]=LCD_Var_Data;
			 }
			 else
			 {
				  Temp_JXS_Parameter.Accelerate.Time[Temp_ACC_Axis]=JXS_Parameter.Accelerate.Time[Temp_ACC_Axis];
			 }
		   gs_Refresh_Accelerate_Level=TRUE;	 			 
			 break;
			 
		//---机械手--确定	 
		case 0x5306:				 
//			 JXS_Parameter.Axis = Temp_JXS_Parameter.Axis;
//			 JXS_Parameter.Origin = Temp_JXS_Parameter.Origin;			
//			 JXS_Parameter.SpeedLevel = Temp_JXS_Parameter.SpeedLevel;
//			 JXS_Parameter.PulseTime  = Temp_JXS_Parameter.PulseTime;
//			 JXS_Parameter.ServoAlarm = Temp_JXS_Parameter.ServoAlarm;   //---2017.12.10设置伺服报警信号
//			 JXS_Parameter.NcComplete = Temp_JXS_Parameter.NcComplete;//---2018.3.20设置数控加工等待时间
//			 JXS_Parameter.ManualSpeed = Temp_JXS_Parameter.ManualSpeed;
//			 
//			 Order_Temp[0]=PARAMETER_FUCTION_SET;  //---对应控制板parameter.c  129行的中UsartReceiveData[1]，下面以此类推
//			 Order_Temp[1]=0x01;			 	
//			 Order_Temp[2]=JXS_Parameter.Axis;           
//			 Order_Temp[3]=JXS_Parameter.Origin;			 
//			 Order_Temp[4]=JXS_Parameter.SpeedLevel;		
//       //---加一个原点速度
//			 Order_Temp[5]=JXS_Parameter.OriginSpeed[0];
//			 Order_Temp[6]=JXS_Parameter.ManualSpeed;///手动速度

       //---2018.3.21加四个电子齿轮比,一个伺服报警			 

			 USART1_SendData(7,PARAMETER_ORDER,Order_Temp);//---数据长度改成了13----下发到控制板parameter.c  129--245行---改成14--6.28号改成16   6.29改成24   12.10改成29
			 gs_Reflash_Fuction_JXS=TRUE;
			 break;
			 
		//---机械手--返回
 		case 0x5307:     
		   //---回零顺序
			Temp_JXS_Parameter.Origin[0] = JXS_Parameter.Origin[0];
			Temp_JXS_Parameter.Origin[1] = JXS_Parameter.Origin[1];
			Temp_JXS_Parameter.Origin[2] = JXS_Parameter.Origin[2];
			Temp_JXS_Parameter.Origin[3] = JXS_Parameter.Origin[3];
			Temp_JXS_Parameter.Origin[4] = JXS_Parameter.Origin[4];
			Temp_JXS_Parameter.Origin[5] = JXS_Parameter.Origin[5];
		//原点方向
			 Temp_JXS_Parameter.OriginDir[0] = JXS_Parameter.OriginDir[0];
			 Temp_JXS_Parameter.OriginDir[1] = JXS_Parameter.OriginDir[1];
			 Temp_JXS_Parameter.OriginDir[2] = JXS_Parameter.OriginDir[2];
			 Temp_JXS_Parameter.OriginDir[3] = JXS_Parameter.OriginDir[3];
			 Temp_JXS_Parameter.OriginDir[4] = JXS_Parameter.OriginDir[4];
			 Temp_JXS_Parameter.OriginDir[5] = JXS_Parameter.OriginDir[5];
			 Temp_JXS_Parameter.SpeedLevel = JXS_Parameter.SpeedLevel;
		   //---原点速度
			 Temp_JXS_Parameter.OriginSpeed[0] = JXS_Parameter.OriginSpeed[0];
			 Temp_JXS_Parameter.OriginSpeed[1] = JXS_Parameter.OriginSpeed[1];
			 Temp_JXS_Parameter.OriginSpeed[2] = JXS_Parameter.OriginSpeed[2];
			 Temp_JXS_Parameter.OriginSpeed[3] = JXS_Parameter.OriginSpeed[3];
			 Temp_JXS_Parameter.OriginSpeed[4] = JXS_Parameter.OriginSpeed[4];
			 Temp_JXS_Parameter.OriginSpeed[5] = JXS_Parameter.OriginSpeed[5];	
		//手动速度
			 Temp_JXS_Parameter.ManualSpeed = JXS_Parameter.ManualSpeed;

			 Temp_JXS_Parameter.PulseTime  = JXS_Parameter.PulseTime;
		     Temp_JXS_Parameter.ServoAlarm  = JXS_Parameter.ServoAlarm;   //---2017.12.10设置伺服报警信号
			 Temp_JXS_Parameter.NcComplete  = JXS_Parameter.NcComplete;//---2018.3.20设置数控加工等待时间
// 		 Temp_JXS_Parameter.Accelerate = JXS_Parameter.Accelerate;//---6.29注释
		   //---6.29  
			 LCD_Current_Page= 17;
			 break;
		
    //---机械手--速度等级		
 		case 0x5308:     
		   Temp_JXS_Parameter.SpeedLevel = LCD_Key_Value;
			 break;
		
		//---增加回原点速度大小
		case 0x5309:       
				if(LCD_Key_Value<1||LCD_Key_Value>20)  
				{
					LCD_Key_Value=20;
				}
//			 Temp_JXS_Parameter.OriginSpeed = LCD_Key_Value;  
			 break;   
			
    //---机械手--原点方向X Y Z A B O轴选择				
 		case 0x530A:     
		   if(Admin_Authority)                               //管理员权限
			 {
				 OriginDir_Flag++;
				 if(OriginDir_Flag==6)
				 {
			       OriginDir_Flag = 0;
				 }
				 gs_Refresh_Origin_Dir=TRUE;
			 }
			 break;
			 
		//---机械手--原点方向选择X Y Z A B O	 
 		case 0x530B:     
		   if(Admin_Authority)                               //管理员权限
			 {
				 if(Temp_JXS_Parameter.OriginDir[OriginDir_Flag])
				 {
			       Temp_JXS_Parameter.OriginDir[OriginDir_Flag] = 0;
				 }
				 else
				 {
				     Temp_JXS_Parameter.OriginDir[OriginDir_Flag] = 1;
				 }
				 gs_Refresh_Origin_Dir=TRUE;
			 }
			 break;
			 
	  //---6.28
		//---机械手--加速度--时间轴的选择
		case 0x5313:       
			 if(Admin_Authority)
			 {
				 Temp_ACC_Axis++;
				 if(Temp_ACC_Axis==6)
				 {
					 Temp_ACC_Axis=FAN_X;
				 }
				 gs_Refresh_Accelerate_Level=TRUE;
			 }
			 break;
				 
		//---6.28
		//---机械手--加速类型选择 S\T
		case 0x530E:       
       if(Admin_Authority)
			 {
				 Temp_JXS_Parameter.Accelerate.Type[Temp_ACC_Axis]++;
				 if(Temp_JXS_Parameter.Accelerate.Type[Temp_ACC_Axis]>=2)
				 {
						Temp_JXS_Parameter.Accelerate.Type[Temp_ACC_Axis]=FAN_S;
				 }
				gs_Refresh_Accelerate_Level=TRUE;
			 }
			 break;				 
				 
		case 0x536A://手动速度调节
			if(LCD_Key_Value<10||LCD_Key_Value>100)  
			{
				LCD_Key_Value=10;
			}			
		   Temp_JXS_Parameter.ManualSpeed = LCD_Key_Value;	
			break;
		
		
		case 0x5320:         
			 break;
		
		//脉冲时间
 		case 0x532C:    
		   Temp_JXS_Parameter.PulseTime = LCD_Var_Data;
			 break;
// 		case 0x5330:
// 			 break;

		//---2018.3.21
		//---增加电子齿轮比设置
		case 0x5341: 
       if(Admin_Authority)
			 {      
//				 Temp_JXS_Parameter.GearRatio_Axis++;
//				 if(Temp_JXS_Parameter.GearRatio_Axis>=4)
//				 {
//						Temp_JXS_Parameter.GearRatio_Axis=0;
//				 }
         gs_Refresh_GearRatio=TRUE;
		   }
			 break; 

//		case 0x5342: 
//       if(Admin_Authority)
//			 {      
////				 Temp_JXS_Parameter.GearRatio.Ratio[Temp_JXS_Parameter.GearRatio_Axis] = LCD_Var_Data;
//				 gs_Refresh_GearRatio=TRUE;
//		   }			 
//			 break;

		case 0x5343:			 
			 break;

//		case 0x5344:			 
//			 break;
			 
		//---2017.12.06
		//---增加伺服报警信号设置
		case 0x5345:
       if(Admin_Authority)
			 {				 
				 if(Temp_JXS_Parameter.ServoAlarm == FAN_K)  
				 {
					 Temp_JXS_Parameter.ServoAlarm =FAN_C;
				 }
				 else
				 {
					 Temp_JXS_Parameter.ServoAlarm =FAN_K;
				 }
				 gs_Refresh_ServoAlarm = TRUE;
       }				 
			 break;
			 
	  //---2018.3.20
		//---增加数控加工等待时间-IO口
		case 0x5347:       
			 if(Admin_Authority)
			 {
				 Temp_JXS_Parameter.NcComplete=1+(Temp_JXS_Parameter.NcComplete);
				 if((Temp_JXS_Parameter.NcComplete&0x0f)==5)
				 {
					 Temp_JXS_Parameter.NcComplete&=0xf0;
				 }
				 gs_Refresh_NumericalControl=TRUE;
			 }
			 break;
			 
	  //---2018.3.20
		//---增加数控加工等待时间-时间	 
//		case 0x5348:  
//		   if(Admin_Authority)
//			 { 
//				  Temp_JXS_Parameter.NcComplete&=0x0f;
//				  Temp_JXS_Parameter.NcComplete|=(LCD_Var_Data<<4);
//			 }
//			 else
//			 {
//				  Temp_JXS_Parameter.NcComplete|=(JXS_Parameter.NcComplete&0xf0);
//			 }
//		   gs_Refresh_NumericalControl=TRUE;	 			 
//			 break;			 
		case 0x53B1:
			 LCD_Current_Page= 151;		
			break;
	   //LC参数设定-料仓1选择
		case 0x53A0: 
			 LCD_Current_Page= 78;	
		     LC_ID=0;
			 Temp_LC_Parameter_Load[0].Switch = LC_Parameter_Load[0].Switch; 
			 Temp_LC_Parameter_Load[0].H_Num = LC_Parameter_Load[0].H_Num;
			 Temp_LC_Parameter_Load[0].V_Num = LC_Parameter_Load[0].V_Num;
			 Temp_LC_Parameter_Load[0].Position1.XPoint = LC_Parameter_Load[0].Position1.XPoint;
			 Temp_LC_Parameter_Load[0].Position1.YPoint = LC_Parameter_Load[0].Position1.YPoint;
			 Temp_LC_Parameter_Load[0].Position2.XPoint = LC_Parameter_Load[0].Position2.XPoint;
			 Temp_LC_Parameter_Load[0].Position2.YPoint = LC_Parameter_Load[0].Position2.YPoint;
			 Temp_LC_Parameter_Load[0].Position3.XPoint = LC_Parameter_Load[0].Position3.XPoint;
			 Temp_LC_Parameter_Load[0].Position3.YPoint = LC_Parameter_Load[0].Position3.YPoint;
			 Temp_LC_Parameter_Load[0].Glue_Time = LC_Parameter_Load[0].Diamonds_Time;
			 Temp_LC_Parameter_Load[0].HT_Hight = LC_Parameter_Load[0].HT_Hight;
			 Temp_LC_Parameter_Load[0].LC_Hight = LC_Parameter_Load[0].LC_Hight;
			 gs_Reflash_Fuction_LC=TRUE;
			 break;
		
	   //LC参数设定-料仓2选择
		case 0x53A1: 
			 LCD_Current_Page= 78;	
			 LC_ID=1;
			 Temp_LC_Parameter_Load[1].Switch = LC_Parameter_Load[1].Switch; 
			 Temp_LC_Parameter_Load[1].H_Num = LC_Parameter_Load[1].H_Num;
			 Temp_LC_Parameter_Load[1].V_Num = LC_Parameter_Load[1].V_Num;
			 Temp_LC_Parameter_Load[1].Position1.XPoint = LC_Parameter_Load[1].Position1.XPoint;
			 Temp_LC_Parameter_Load[1].Position1.YPoint = LC_Parameter_Load[1].Position1.YPoint;
			 Temp_LC_Parameter_Load[1].Position2.XPoint = LC_Parameter_Load[1].Position2.XPoint;
			 Temp_LC_Parameter_Load[1].Position2.YPoint = LC_Parameter_Load[1].Position2.YPoint;
			 Temp_LC_Parameter_Load[1].Position3.XPoint = LC_Parameter_Load[1].Position3.XPoint;
			 Temp_LC_Parameter_Load[1].Position3.YPoint = LC_Parameter_Load[1].Position3.YPoint;
			 Temp_LC_Parameter_Load[1].Glue_Time = LC_Parameter_Load[1].Diamonds_Time;
			 Temp_LC_Parameter_Load[1].HT_Hight = LC_Parameter_Load[1].HT_Hight;
			 Temp_LC_Parameter_Load[1].LC_Hight = LC_Parameter_Load[1].LC_Hight;
			 gs_Reflash_Fuction_LC=TRUE;
			 break;

	   //LC参数设定-料仓3选择
		case 0x53A2: 
			 LCD_Current_Page= 78;	
			 LC_ID=2;
			 Temp_LC_Parameter_Load[2].Switch = LC_Parameter_Load[2].Switch; 
			 Temp_LC_Parameter_Load[2].H_Num = LC_Parameter_Load[2].H_Num;
			 Temp_LC_Parameter_Load[2].V_Num = LC_Parameter_Load[2].V_Num;
			 Temp_LC_Parameter_Load[2].Position1.XPoint = LC_Parameter_Load[2].Position1.XPoint;
			 Temp_LC_Parameter_Load[2].Position1.YPoint = LC_Parameter_Load[2].Position1.YPoint;
			 Temp_LC_Parameter_Load[2].Position2.XPoint = LC_Parameter_Load[2].Position2.XPoint;
			 Temp_LC_Parameter_Load[2].Position2.YPoint = LC_Parameter_Load[2].Position2.YPoint;
			 Temp_LC_Parameter_Load[2].Position3.XPoint = LC_Parameter_Load[2].Position3.XPoint;
			 Temp_LC_Parameter_Load[2].Position3.YPoint = LC_Parameter_Load[2].Position3.YPoint;
			 Temp_LC_Parameter_Load[2].Glue_Time = LC_Parameter_Load[2].Diamonds_Time;
			 Temp_LC_Parameter_Load[2].HT_Hight = LC_Parameter_Load[2].HT_Hight;
			 Temp_LC_Parameter_Load[2].LC_Hight = LC_Parameter_Load[2].LC_Hight;		
			 gs_Reflash_Fuction_LC=TRUE;
			 break;

	   //LC参数设定-料仓4选择
		case 0x53A3: 
			 LCD_Current_Page= 78;	
			 LC_ID=3;
			 Temp_LC_Parameter_Load[3].Switch = LC_Parameter_Load[3].Switch; 
			 Temp_LC_Parameter_Load[3].H_Num = LC_Parameter_Load[3].H_Num;
			 Temp_LC_Parameter_Load[3].V_Num = LC_Parameter_Load[3].V_Num;
			 Temp_LC_Parameter_Load[3].Position1.XPoint = LC_Parameter_Load[3].Position1.XPoint;
			 Temp_LC_Parameter_Load[3].Position1.YPoint = LC_Parameter_Load[3].Position1.YPoint;
			 Temp_LC_Parameter_Load[3].Position2.XPoint = LC_Parameter_Load[3].Position2.XPoint;
			 Temp_LC_Parameter_Load[3].Position2.YPoint = LC_Parameter_Load[3].Position2.YPoint;
			 Temp_LC_Parameter_Load[3].Position3.XPoint = LC_Parameter_Load[3].Position3.XPoint;
			 Temp_LC_Parameter_Load[3].Position3.YPoint = LC_Parameter_Load[3].Position3.YPoint;
			 Temp_LC_Parameter_Load[3].Glue_Time = LC_Parameter_Load[3].Diamonds_Time;
			 Temp_LC_Parameter_Load[3].HT_Hight = LC_Parameter_Load[3].HT_Hight;
			 Temp_LC_Parameter_Load[3].LC_Hight = LC_Parameter_Load[3].LC_Hight;
			 gs_Reflash_Fuction_LC=TRUE;
			 break;	

	   //LC参数设定-料仓5选择
		case 0x539F: 
			 LCD_Current_Page= 78;	
			 LC_ID=4;
			 Temp_LC_Parameter_Load[4].Switch = LC_Parameter_Load[4].Switch; 
			 Temp_LC_Parameter_Load[4].H_Num = LC_Parameter_Load[4].H_Num;
			 Temp_LC_Parameter_Load[4].V_Num = LC_Parameter_Load[4].V_Num;
			 Temp_LC_Parameter_Load[4].Position1.XPoint = LC_Parameter_Load[4].Position1.XPoint;
			 Temp_LC_Parameter_Load[4].Position1.YPoint = LC_Parameter_Load[4].Position1.YPoint;
			 Temp_LC_Parameter_Load[4].Position2.XPoint = LC_Parameter_Load[4].Position2.XPoint;
			 Temp_LC_Parameter_Load[4].Position2.YPoint = LC_Parameter_Load[4].Position2.YPoint;
			 Temp_LC_Parameter_Load[4].Position3.XPoint = LC_Parameter_Load[4].Position3.XPoint;
			 Temp_LC_Parameter_Load[4].Position3.YPoint = LC_Parameter_Load[4].Position3.YPoint;
			 Temp_LC_Parameter_Load[4].Glue_Time = LC_Parameter_Load[4].Diamonds_Time;
			 Temp_LC_Parameter_Load[4].HT_Hight = LC_Parameter_Load[4].HT_Hight;
			 Temp_LC_Parameter_Load[4].LC_Hight = LC_Parameter_Load[4].LC_Hight;
			 gs_Reflash_Fuction_LC=TRUE;
			 break;	
	   //LC参数设定-料仓6选择
		case 0x53A6: 
			 LCD_Current_Page= 78;	
			 LC_ID=5;
			 Temp_LC_Parameter_Load[5].Switch = LC_Parameter_Load[5].Switch; 
			 Temp_LC_Parameter_Load[5].H_Num = LC_Parameter_Load[5].H_Num;
			 Temp_LC_Parameter_Load[5].V_Num = LC_Parameter_Load[5].V_Num;
			 Temp_LC_Parameter_Load[5].Position1.XPoint = LC_Parameter_Load[5].Position1.XPoint;
			 Temp_LC_Parameter_Load[5].Position1.YPoint = LC_Parameter_Load[5].Position1.YPoint;
			 Temp_LC_Parameter_Load[5].Position2.XPoint = LC_Parameter_Load[5].Position2.XPoint;
			 Temp_LC_Parameter_Load[5].Position2.YPoint = LC_Parameter_Load[5].Position2.YPoint;
			 Temp_LC_Parameter_Load[5].Position3.XPoint = LC_Parameter_Load[5].Position3.XPoint;
			 Temp_LC_Parameter_Load[5].Position3.YPoint = LC_Parameter_Load[5].Position3.YPoint;
			 Temp_LC_Parameter_Load[5].Glue_Time = LC_Parameter_Load[5].Diamonds_Time;
			 Temp_LC_Parameter_Load[5].HT_Hight = LC_Parameter_Load[5].HT_Hight;
			 Temp_LC_Parameter_Load[5].LC_Hight = LC_Parameter_Load[5].LC_Hight;
			 gs_Reflash_Fuction_LC=TRUE;
			 break;	
	   //LC参数设定-料仓7选择
		case 0x53A7: 
			 LCD_Current_Page= 78;	
			 LC_ID=6;
			 Temp_LC_Parameter_Load[6].Switch = LC_Parameter_Load[6].Switch; 
			 Temp_LC_Parameter_Load[6].H_Num = LC_Parameter_Load[6].H_Num;
			 Temp_LC_Parameter_Load[6].V_Num = LC_Parameter_Load[6].V_Num;
			 Temp_LC_Parameter_Load[6].Position1.XPoint = LC_Parameter_Load[6].Position1.XPoint;
			 Temp_LC_Parameter_Load[6].Position1.YPoint = LC_Parameter_Load[6].Position1.YPoint;
			 Temp_LC_Parameter_Load[6].Position2.XPoint = LC_Parameter_Load[6].Position2.XPoint;
			 Temp_LC_Parameter_Load[6].Position2.YPoint = LC_Parameter_Load[6].Position2.YPoint;
			 Temp_LC_Parameter_Load[6].Position3.XPoint = LC_Parameter_Load[6].Position3.XPoint;
			 Temp_LC_Parameter_Load[6].Position3.YPoint = LC_Parameter_Load[6].Position3.YPoint;
			 Temp_LC_Parameter_Load[6].Glue_Time = LC_Parameter_Load[6].Diamonds_Time;
			 Temp_LC_Parameter_Load[6].HT_Hight = LC_Parameter_Load[6].HT_Hight;
			 Temp_LC_Parameter_Load[6].LC_Hight = LC_Parameter_Load[6].LC_Hight;
			 gs_Reflash_Fuction_LC=TRUE;
			 break;	
	   //LC参数设定-料仓8选择
		case 0x53A8: 
			 LCD_Current_Page= 78;	
			 LC_ID=7;
			 Temp_LC_Parameter_Load[7].Switch = LC_Parameter_Load[7].Switch; 
			 Temp_LC_Parameter_Load[7].H_Num = LC_Parameter_Load[7].H_Num;
			 Temp_LC_Parameter_Load[7].V_Num = LC_Parameter_Load[7].V_Num;
			 Temp_LC_Parameter_Load[7].Position1.XPoint = LC_Parameter_Load[7].Position1.XPoint;
			 Temp_LC_Parameter_Load[7].Position1.YPoint = LC_Parameter_Load[7].Position1.YPoint;
			 Temp_LC_Parameter_Load[7].Position2.XPoint = LC_Parameter_Load[7].Position2.XPoint;
			 Temp_LC_Parameter_Load[7].Position2.YPoint = LC_Parameter_Load[7].Position2.YPoint;
			 Temp_LC_Parameter_Load[7].Position3.XPoint = LC_Parameter_Load[7].Position3.XPoint;
			 Temp_LC_Parameter_Load[7].Position3.YPoint = LC_Parameter_Load[7].Position3.YPoint;
			 Temp_LC_Parameter_Load[7].Glue_Time = LC_Parameter_Load[7].Diamonds_Time;
			 Temp_LC_Parameter_Load[7].HT_Hight = LC_Parameter_Load[7].HT_Hight;
			 Temp_LC_Parameter_Load[7].LC_Hight = LC_Parameter_Load[7].LC_Hight;
			 gs_Reflash_Fuction_LC=TRUE;
			 break;	
	   //LC参数设定-料仓9选择
		case 0x53A9: 
			 LCD_Current_Page= 78;	
			 LC_ID=8;
			 Temp_LC_Parameter_Load[8].Switch = LC_Parameter_Load[8].Switch; 
			 Temp_LC_Parameter_Load[8].H_Num = LC_Parameter_Load[8].H_Num;
			 Temp_LC_Parameter_Load[8].V_Num = LC_Parameter_Load[8].V_Num;
			 Temp_LC_Parameter_Load[8].Position1.XPoint = LC_Parameter_Load[8].Position1.XPoint;
			 Temp_LC_Parameter_Load[8].Position1.YPoint = LC_Parameter_Load[8].Position1.YPoint;
			 Temp_LC_Parameter_Load[8].Position2.XPoint = LC_Parameter_Load[8].Position2.XPoint;
			 Temp_LC_Parameter_Load[8].Position2.YPoint = LC_Parameter_Load[8].Position2.YPoint;
			 Temp_LC_Parameter_Load[8].Position3.XPoint = LC_Parameter_Load[8].Position3.XPoint;
			 Temp_LC_Parameter_Load[8].Position3.YPoint = LC_Parameter_Load[8].Position3.YPoint;
			 Temp_LC_Parameter_Load[8].Glue_Time = LC_Parameter_Load[8].Diamonds_Time;
			 Temp_LC_Parameter_Load[8].HT_Hight = LC_Parameter_Load[8].HT_Hight;
			 Temp_LC_Parameter_Load[8].LC_Hight = LC_Parameter_Load[8].LC_Hight;
			 gs_Reflash_Fuction_LC=TRUE;
			 break;	
	   //LC参数设定-料仓10选择
		case 0x53AA: 
			 LCD_Current_Page= 78;	
			 LC_ID=9;
			 Temp_LC_Parameter_Load[9].Switch = LC_Parameter_Load[9].Switch; 
			 Temp_LC_Parameter_Load[9].H_Num = LC_Parameter_Load[9].H_Num;
			 Temp_LC_Parameter_Load[9].V_Num = LC_Parameter_Load[9].V_Num;
			 Temp_LC_Parameter_Load[9].Position1.XPoint = LC_Parameter_Load[9].Position1.XPoint;
			 Temp_LC_Parameter_Load[9].Position1.YPoint = LC_Parameter_Load[9].Position1.YPoint;
			 Temp_LC_Parameter_Load[9].Position2.XPoint = LC_Parameter_Load[9].Position2.XPoint;
			 Temp_LC_Parameter_Load[9].Position2.YPoint = LC_Parameter_Load[9].Position2.YPoint;
			 Temp_LC_Parameter_Load[9].Position3.XPoint = LC_Parameter_Load[9].Position3.XPoint;
			 Temp_LC_Parameter_Load[9].Position3.YPoint = LC_Parameter_Load[9].Position3.YPoint;
			 Temp_LC_Parameter_Load[9].Glue_Time = LC_Parameter_Load[9].Diamonds_Time;
			 Temp_LC_Parameter_Load[9].HT_Hight = LC_Parameter_Load[9].HT_Hight;
			 Temp_LC_Parameter_Load[9].LC_Hight = LC_Parameter_Load[9].LC_Hight;
			 gs_Reflash_Fuction_LC=TRUE;
			 break;			
		case 0x53A4://料仓功能开关	 
				if(Temp_LC_Parameter_Load[LC_ID].Switch == 0)
				 {
					  Temp_LC_Parameter_Load[LC_ID].Switch = 1;
				 } 
				 else
				 {
					  Temp_LC_Parameter_Load[LC_ID].Switch = 0;
				 }
				 gs_Refresh_LC_Switch=TRUE;
		     break;
		
		
//		case 0x53B1: //示教坐标-O起点
//			 if(LCD_Current_Page == 78)
//				 Set_LCPosition_XYO=71;
//			 else if(LCD_Current_Page == 136)
//				 Set_LCPosition_XYO=72;
//			 else if(LCD_Current_Page == 137)
//				 Set_LCPosition_XYO=73;
//			 else if(LCD_Current_Page == 138)
//				 Set_LCPosition_XYO=74;
//			 LCD_Current_Page = 123;
//			 gs_Current_Choose_Point = 1;
//			 gs_Point_Display_Refresh = TRUE;
//			 break;

		
		case 0x53B4: //示教坐标-O终点
//			 if(LC_ID == 0)
//				 Set_LCPosition_XYO=75;
//			 else if(LC_ID == 1)
//				 Set_LCPosition_XYO=76;
//			 else if(LC_ID == 2)
//				 Set_LCPosition_XYO=77;
//			 else if(LC_ID == 3)
//				 Set_LCPosition_XYO=78;
//			 LCD_Current_Page = 123;
//			 gs_Current_Choose_Point = 1;
//			 gs_Point_Display_Refresh = TRUE;
			 break;	
			 
		case 0x53AD://横向个数
			 Temp_LC_Parameter_Load[LC_ID].H_Num =  LCD_Var_Data;
			 gs_Refresh_LC_Position = TRUE;
			 break;
			 
		case 0x53AB://纵向个数
			 Temp_LC_Parameter_Load[LC_ID].V_Num =  LCD_Var_Data;
			 gs_Refresh_LC_Position = TRUE;			 
			 break;	 

		case 0x53AF://出胶时间		 
			 Temp_LC_Parameter_Load[LC_ID].Glue_Time =  LCD_Var_Data*10;
			 gs_Refresh_LC_Position = TRUE;
			 break;	 			 
			 
		case 0x52A0://取放时间			 
			 Temp_LC_Parameter_Load[LC_ID].Diamonds_Time =  LCD_Var_Data*10;
			 gs_Refresh_LC_Position = TRUE;
			 break;	 				 
		
		case 0x53B8: //回抬高度
			 Temp_LC_Parameter_Load[LC_ID].HT_Hight =  LCD_Var_Data;
			 gs_Refresh_LC_Position = TRUE;
			 break;

		
		case 0x53BA: //料仓高度
			Set_LCPosition_XYO=18;
			 LCD_Current_Page = 44;
			 gs_Current_Choose_Point = 1;
			 gs_Point_Display_Refresh = TRUE;
			 break;	
			 
		//LC-保存
		case 0x53BD: 
			 LC_Parameter_Load[LC_ID].ID = LC_ID;
			 LC_Parameter_Load[LC_ID].Switch   = Temp_LC_Parameter_Load[LC_ID].Switch;
			 LC_Parameter_Load[LC_ID].H_Num      = Temp_LC_Parameter_Load[LC_ID].H_Num;
			 LC_Parameter_Load[LC_ID].V_Num      = Temp_LC_Parameter_Load[LC_ID].V_Num;
			 LC_Parameter_Load[LC_ID].Position1.XPoint = Temp_LC_Parameter_Load[LC_ID].Position1.XPoint;
			 LC_Parameter_Load[LC_ID].Position1.YPoint = Temp_LC_Parameter_Load[LC_ID].Position1.YPoint;
			 LC_Parameter_Load[LC_ID].Position2.XPoint = Temp_LC_Parameter_Load[LC_ID].Position2.XPoint;
			 LC_Parameter_Load[LC_ID].Position2.YPoint = Temp_LC_Parameter_Load[LC_ID].Position2.YPoint;
			 LC_Parameter_Load[LC_ID].Position3.XPoint = Temp_LC_Parameter_Load[LC_ID].Position3.XPoint;
			 LC_Parameter_Load[LC_ID].Position3.YPoint = Temp_LC_Parameter_Load[LC_ID].Position3.YPoint;
			 LC_Parameter_Load[LC_ID].Glue_Time= Temp_LC_Parameter_Load[LC_ID].Glue_Time;
			 LC_Parameter_Load[LC_ID].Diamonds_Time= Temp_LC_Parameter_Load[LC_ID].Diamonds_Time;
			 LC_Parameter_Load[LC_ID].HT_Hight      = Temp_LC_Parameter_Load[LC_ID].HT_Hight;
			 LC_Parameter_Load[LC_ID].LC_Hight      = Temp_LC_Parameter_Load[LC_ID].LC_Hight;
			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x02;						               //料仓参数
			 Order_Temp[2] =LC_Parameter_Load[LC_ID].ID;
			 Order_Temp[3] =LC_Parameter_Load[LC_ID].Switch;                //参数值
			 Order_Temp[4]=LC_Parameter_Load[LC_ID].H_Num;                 //参数值
			 Order_Temp[5]=LC_Parameter_Load[LC_ID].V_Num;                 //参数值 
			 Order_Temp[6]=LC_Parameter_Load[LC_ID].Glue_Time;               //参数值
			 Order_Temp[7]=LC_Parameter_Load[LC_ID].Glue_Time>>8;                 //参数值 
			 Order_Temp[8]=LC_Parameter_Load[LC_ID].Diamonds_Time;                 //参数值
			 Order_Temp[9]=LC_Parameter_Load[LC_ID].Diamonds_Time>>8;                 //参数值 
			 Order_Temp[10]=LC_Parameter_Load[LC_ID].HT_Hight;                 //参数值
			 Order_Temp[11]=LC_Parameter_Load[LC_ID].HT_Hight>>8;                 //参数值 
			 Order_Temp[12]=LC_Parameter_Load[LC_ID].HT_Hight>>16;                 //参数值
			 Order_Temp[13]=LC_Parameter_Load[LC_ID].HT_Hight>>24;                 //参数值 
			 USART1_SendData(14,PARAMETER_ORDER,Order_Temp);

			 Order_Temp[0] =PARAMETER_FUCTION_SET;
			 Order_Temp[1] =0x03;	
			 Order_Temp[2]=LC_Parameter_Load[LC_ID].LC_Hight;                 //参数值
			 Order_Temp[3]=LC_Parameter_Load[LC_ID].LC_Hight>>8;                 //参数值 
			 Order_Temp[4]=LC_Parameter_Load[LC_ID].LC_Hight>>16;                 //参数值
			 Order_Temp[5]=LC_Parameter_Load[LC_ID].LC_Hight>>24;                 //参数值 			 
			 Order_Temp[6]=LC_Parameter_Load[LC_ID].Position1.XPoint;      //参数值
			 Order_Temp[7]=LC_Parameter_Load[LC_ID].Position1.XPoint>>8;   //参数值
			 Order_Temp[8]=LC_Parameter_Load[LC_ID].Position1.XPoint>>16;  //参数值
			 Order_Temp[9]=LC_Parameter_Load[LC_ID].Position1.XPoint>>24;  //参数值
			 Order_Temp[10]=LC_Parameter_Load[LC_ID].Position1.YPoint;      //参数值
			 Order_Temp[11]=LC_Parameter_Load[LC_ID].Position1.YPoint>>8;   //参数值
			 Order_Temp[12]=LC_Parameter_Load[LC_ID].Position1.YPoint>>16;  //参数值
			 Order_Temp[13]=LC_Parameter_Load[LC_ID].Position1.YPoint>>24;  //参数值			 
			 Order_Temp[14] =LC_Parameter_Load[LC_ID].Position2.XPoint;      //参数值
			 Order_Temp[15] =LC_Parameter_Load[LC_ID].Position2.XPoint>>8;   //参数值
			 Order_Temp[16] =LC_Parameter_Load[LC_ID].Position2.XPoint>>16;  //参数值
			 Order_Temp[17] =LC_Parameter_Load[LC_ID].Position2.XPoint>>24;  //参数值
			 Order_Temp[18] =LC_Parameter_Load[LC_ID].Position2.YPoint;      //参数值
			 Order_Temp[19] =LC_Parameter_Load[LC_ID].Position2.YPoint>>8;   //参数值
			 Order_Temp[20] =LC_Parameter_Load[LC_ID].Position2.YPoint>>16;  //参数值
			 Order_Temp[21] =LC_Parameter_Load[LC_ID].Position2.YPoint>>24;  //参数值
			 Order_Temp[22]=LC_Parameter_Load[LC_ID].Position3.XPoint;      //参数值
			 Order_Temp[23]=LC_Parameter_Load[LC_ID].Position3.XPoint>>8;   //参数值
			 Order_Temp[24]=LC_Parameter_Load[LC_ID].Position3.XPoint>>16;  //参数值
			 Order_Temp[25]=LC_Parameter_Load[LC_ID].Position3.XPoint>>24;  //参数值
			 Order_Temp[26]=LC_Parameter_Load[LC_ID].Position3.YPoint;      //参数值
			 Order_Temp[27]=LC_Parameter_Load[LC_ID].Position3.YPoint>>8;   //参数值
			 Order_Temp[28]=LC_Parameter_Load[LC_ID].Position3.YPoint>>16;  //参数值
			 Order_Temp[29]=LC_Parameter_Load[LC_ID].Position3.YPoint>>24;  //参数值
			 USART1_SendData(30,PARAMETER_ORDER,Order_Temp);

			 //计算上料仓参数
			 Calculate_LC_Load_Parameter();
			 gs_Reflash_Fuction_LC=TRUE;
			 break;
			 
		//LC-返回
		case 0x53BE:  
			 Temp_LC_Parameter_Load[LC_ID].Switch     = LC_Parameter_Load[LC_ID].Switch;
			 Temp_LC_Parameter_Load[LC_ID].H_Num      = LC_Parameter_Load[LC_ID].H_Num;
			 Temp_LC_Parameter_Load[LC_ID].V_Num      = LC_Parameter_Load[LC_ID].V_Num;
			 Temp_LC_Parameter_Load[LC_ID].Glue_Time     = LC_Parameter_Load[LC_ID].Glue_Time;
			 Temp_LC_Parameter_Load[LC_ID].Diamonds_Time = LC_Parameter_Load[LC_ID].Diamonds_Time;			 
			 Temp_LC_Parameter_Load[LC_ID].HT_Hight      = LC_Parameter_Load[LC_ID].HT_Hight;
			 Temp_LC_Parameter_Load[LC_ID].LC_Hight      = LC_Parameter_Load[LC_ID].LC_Hight;			 	 
			 Temp_LC_Parameter_Load[LC_ID].Position1.XPoint = LC_Parameter_Load[LC_ID].Position1.XPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position1.YPoint = LC_Parameter_Load[LC_ID].Position1.YPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position2.XPoint = LC_Parameter_Load[LC_ID].Position2.XPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position2.YPoint = LC_Parameter_Load[LC_ID].Position2.YPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position3.XPoint = LC_Parameter_Load[LC_ID].Position3.XPoint;
			 Temp_LC_Parameter_Load[LC_ID].Position3.YPoint = LC_Parameter_Load[LC_ID].Position3.YPoint;
			 LCD_Current_Page= 17;   
  
			 Set_LCPosition_XYO=0;
			 break;

		//LC-示教坐标1-X	 
		case 0x53C0: 
			Set_LCPosition_XYO=11;
			 LCD_Current_Page = 41;
			 gs_Current_Choose_Point = 1;
			 gs_Point_Display_Refresh = TRUE;
			 break;
			 
		//示教坐标1-Y	 
		case 0x53C1: 
			Set_LCPosition_XYO=12;
			 LCD_Current_Page = 115;
			 gs_Current_Choose_Point = 1;
			 gs_Point_Display_Refresh = TRUE;
			 break;
			 
		//示教坐标2-X	 
		case 0x53C2: 
			Set_LCPosition_XYO=13;
			 LCD_Current_Page = 41;
			 gs_Current_Choose_Point = 1;
			 gs_Point_Display_Refresh = TRUE;
			 break;
			 
		//示教坐标2-Y
		case 0x53C3: 
			Set_LCPosition_XYO=14;
			 LCD_Current_Page = 115;
			 gs_Current_Choose_Point = 1;
			 gs_Point_Display_Refresh = TRUE;
			 break;
			 
		//示教坐标3-X
		case 0x53C4: 
			Set_LCPosition_XYO=15;
			 LCD_Current_Page = 41;
			 gs_Current_Choose_Point = 1;
			 gs_Point_Display_Refresh = TRUE;				  
			 break;
			
		//示教坐标3-Y
		case 0x53C5: 
			Set_LCPosition_XYO=16;
			 LCD_Current_Page = 115;
			 gs_Current_Choose_Point = 1;
			 gs_Point_Display_Refresh = TRUE;
			 break;

	  //生产-加工任务 
//		case 0x5350: 
//		   Temp_SC_Parameter.RW_Num = LCD_Var_Data;  
//			 //Temp_SC_Parameter.SC_Num=0;
//			 break;
		
		//生产-抽检产品
//		case 0x5352: 
//		   if(LCD_Var_Data<=Temp_SC_Parameter.RW_Num)
//			 {
//			  	Temp_SC_Parameter.CJ_Num = LCD_Var_Data; 
//			 }
//			 else
//			 {
//			 	  WriteNum4(0x5352,SC_Parameter.CJ_Num);
//			 }
//			 break;
			 
//		//生产-抽检间隔
// 		case 0x5354:  
//		   if(LCD_Var_Data<=Temp_SC_Parameter.RW_Num)
//			 {
//		     	Temp_SC_Parameter.JG_Num = LCD_Var_Data; 
//			 } 
//			 else
//			 {
//			 	  WriteNum4(0x5354,SC_Parameter.JG_Num);
//			 }
//			 break;
//		case 0x5356:    		    
//			 break;
			
    //生产-保存			 
//		case 0x5358:    
//			 SC_Parameter.RW_Num = Temp_SC_Parameter.RW_Num;
//			 SC_Parameter.CJ_Num = Temp_SC_Parameter.CJ_Num;
//			 SC_Parameter.JG_Num = Temp_SC_Parameter.JG_Num;
//			 SC_Parameter.SC_Num = Temp_SC_Parameter.SC_Num;
//			 SC_Parameter.LJ_Num = Temp_SC_Parameter.LJ_Num;
//			 Order_Temp[0] =PARAMETER_FUCTION_SET;
//			 Order_Temp[1] =0x06;				              //生产参数
//			 Order_Temp[2] =SC_Parameter.RW_Num;      //参数值
//			 Order_Temp[3] =SC_Parameter.RW_Num>>8;   //参数值
//			 Order_Temp[4] =SC_Parameter.RW_Num>>16;  //参数值
//			 Order_Temp[5] =SC_Parameter.RW_Num>>24;  //参数值
//			 Order_Temp[6] =SC_Parameter.CJ_Num;      //参数值
//			 Order_Temp[7] =SC_Parameter.CJ_Num>>8;   //参数值
//			 Order_Temp[8] =SC_Parameter.CJ_Num>>16;  //参数值
//			 Order_Temp[9] =SC_Parameter.CJ_Num>>24;  //参数值
//			 Order_Temp[10]=SC_Parameter.JG_Num;      //参数值
//			 Order_Temp[11]=SC_Parameter.JG_Num>>8;   //参数值
//			 Order_Temp[12]=SC_Parameter.JG_Num>>16;  //参数值
//			 Order_Temp[13]=SC_Parameter.JG_Num>>24;  //参数值
//			 Order_Temp[14]=SC_Parameter.SC_Num;      //参数值
//			 Order_Temp[15]=SC_Parameter.SC_Num>>8;   //参数值
//			 Order_Temp[16]=SC_Parameter.SC_Num>>16;  //参数值
//			 Order_Temp[17]=SC_Parameter.SC_Num>>24;  //参数值
//			 Order_Temp[18]=SC_Parameter.LJ_Num;      //参数值
//			 Order_Temp[19]=SC_Parameter.LJ_Num>>8;   //参数值
//			 Order_Temp[20]=SC_Parameter.LJ_Num>>16;  //参数值
//			 Order_Temp[21]=SC_Parameter.LJ_Num>>24;  //参数值
//			 USART1_SendData(22,PARAMETER_ORDER,Order_Temp);
//			 g_Reflash_Progress_Num = TRUE;
//			 gs_Reflash_Fuction_SC = TRUE;
//			 break;
			 
		//生产-返回
		case 0x5359:  
			 Temp_SC_Parameter.RW_Num = SC_Parameter.RW_Num;
			 Temp_SC_Parameter.CJ_Num = SC_Parameter.CJ_Num;
			 Temp_SC_Parameter.JG_Num = SC_Parameter.JG_Num;
			 Temp_SC_Parameter.SC_Num = SC_Parameter.SC_Num;
			 Temp_SC_Parameter.LJ_Num = SC_Parameter.LJ_Num;
		   LCD_Current_Page= 17;
			 //g_Reflash_Progress_Num=TRUE;       
			 break;

		//生产-累计数量-清除
		case 0x535C:  
			 Temp_SC_Parameter.LJ_Num =0;
			 gs_Clear_LJ_Parametr = TRUE;
     		 //g_Reflash_Progress_Num=TRUE;
			 break;

		//生产-完成数量-清除
		case 0x535D:  
			 Temp_SC_Parameter.SC_Num =0;
     		 gs_Clear_SC_Parametr = TRUE;
			 //g_Reflash_Progress_Num=TRUE;
			 break;
	}
// 	//---加入速度刷新
// 	if(gs_Refresh_Origin_Speed)
// 	{
// 		WriteNum2(0x5309,gs_Origin_Speed); 
// 		gs_Refresh_Origin_Speed = FALSE;	
// 	}
}

/*************************************************************************
**  函数名：  ResetSetting();
**	输入参数：
**	输出参数：无
**	函数功能
**	备注：	  无
**  作者：    zj   
**  开发日期：2018/01/05
**************************************************************************/		 
void ResetSetting()
{

	if(Admin_Authority)                               //管理员权限
	{
			 
	/****************TWS**************************/
	/****************复位顺序选择*****************/
	if((LCD_Var_Address >= 0x5601 && LCD_Var_Address <= 0x5606))  //第一行
			{
				switch(LCD_Var_Address)
				{
					case 0x5601:
						if(0x20 ==Temp_JXS_Parameter.Origin[0] )
							Temp_JXS_Parameter.Origin[0] = 0x00;
						else
							Temp_JXS_Parameter.Origin[0] = 0x20;
						Temp_JXS_Parameter.Origin[1] &= 0x1F;
						Temp_JXS_Parameter.Origin[2] &= 0x1F;
						Temp_JXS_Parameter.Origin[3] &= 0x1F;
						Temp_JXS_Parameter.Origin[4] &= 0x1F;
						Temp_JXS_Parameter.Origin[5] &= 0x1F;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
					case 0x5602:
						if(0x10 ==Temp_JXS_Parameter.Origin[0] )
							Temp_JXS_Parameter.Origin[0] = 0x00;
						else
							Temp_JXS_Parameter.Origin[0] = 0x10;
						Temp_JXS_Parameter.Origin[1] &= 0x2F;
						Temp_JXS_Parameter.Origin[2] &= 0x2F;
						Temp_JXS_Parameter.Origin[3] &= 0x2F;
						Temp_JXS_Parameter.Origin[4] &= 0x2F;
						Temp_JXS_Parameter.Origin[5] &= 0x2F;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
					case 0x5603:
						if(0x08 ==Temp_JXS_Parameter.Origin[0] )
							Temp_JXS_Parameter.Origin[0] = 0x00;
						else
							Temp_JXS_Parameter.Origin[0] = 0x08;
						Temp_JXS_Parameter.Origin[1] &= 0x37;
						Temp_JXS_Parameter.Origin[2] &= 0x37;
						Temp_JXS_Parameter.Origin[3] &= 0x37;
						Temp_JXS_Parameter.Origin[4] &= 0x37;
						Temp_JXS_Parameter.Origin[5] &= 0x37;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
					case 0x5604:
						if(0x04 ==Temp_JXS_Parameter.Origin[0] )
							Temp_JXS_Parameter.Origin[0] = 0x00;
						else
							Temp_JXS_Parameter.Origin[0] = 0x04;
						Temp_JXS_Parameter.Origin[1] &= 0x3B;
						Temp_JXS_Parameter.Origin[2] &= 0x3B;
						Temp_JXS_Parameter.Origin[3] &= 0x3B;
						Temp_JXS_Parameter.Origin[4] &= 0x3B;
						Temp_JXS_Parameter.Origin[5] &= 0x3B;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
					case 0x5605:
						if(0x02 ==Temp_JXS_Parameter.Origin[0] )
							Temp_JXS_Parameter.Origin[0] = 0x00;
						else
							Temp_JXS_Parameter.Origin[0] = 0x02;
						Temp_JXS_Parameter.Origin[1] &= 0x3D;
						Temp_JXS_Parameter.Origin[2] &= 0x3D;
						Temp_JXS_Parameter.Origin[3] &= 0x3D;
						Temp_JXS_Parameter.Origin[4] &= 0x3D;
						Temp_JXS_Parameter.Origin[5] &= 0x3D;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
					case 0x5606:
						if(0x01 ==Temp_JXS_Parameter.Origin[0] )
							Temp_JXS_Parameter.Origin[0] = 0x00;
						else
							Temp_JXS_Parameter.Origin[0] = 0x01;
						Temp_JXS_Parameter.Origin[1] &= 0x3E;
						Temp_JXS_Parameter.Origin[2] &= 0x3E;
						Temp_JXS_Parameter.Origin[3] &= 0x3E;
						Temp_JXS_Parameter.Origin[4] &= 0x3E;
						Temp_JXS_Parameter.Origin[5] &= 0x3E;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
			   }
			}	

			 
	if((LCD_Var_Address >= 0x5640 && LCD_Var_Address <= 0x5645))  //第二行
					{
						switch(LCD_Var_Address)
						{
							case 0x5640:
						Temp_JXS_Parameter.Origin[0] &= 0x1F;
						if(0x20 ==Temp_JXS_Parameter.Origin[1] )
							Temp_JXS_Parameter.Origin[1] = 0x00;
						else
							Temp_JXS_Parameter.Origin[1] = 0x20;						
						Temp_JXS_Parameter.Origin[2] &= 0x1F;
						Temp_JXS_Parameter.Origin[3] &= 0x1F;
						Temp_JXS_Parameter.Origin[4] &= 0x1F;
						Temp_JXS_Parameter.Origin[5] &= 0x1F;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
							case 0x5641:
						Temp_JXS_Parameter.Origin[0] &= 0x2F;
						if(0x10 ==Temp_JXS_Parameter.Origin[1] )
							Temp_JXS_Parameter.Origin[1] = 0x00;
						else
							Temp_JXS_Parameter.Origin[1] = 0x10;
						Temp_JXS_Parameter.Origin[2] &= 0x2F;
						Temp_JXS_Parameter.Origin[3] &= 0x2F;
						Temp_JXS_Parameter.Origin[4] &= 0x2F;
						Temp_JXS_Parameter.Origin[5] &= 0x2F;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
							case 0x5642:
						Temp_JXS_Parameter.Origin[0] &= 0x37;
						if(0x08 ==Temp_JXS_Parameter.Origin[1] )
							Temp_JXS_Parameter.Origin[1] = 0x00;
						else
							Temp_JXS_Parameter.Origin[1] = 0x08;
						Temp_JXS_Parameter.Origin[2] &= 0x37;
						Temp_JXS_Parameter.Origin[3] &= 0x37;
						Temp_JXS_Parameter.Origin[4] &= 0x37;
						Temp_JXS_Parameter.Origin[5] &= 0x37;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
							case 0x5643:
						Temp_JXS_Parameter.Origin[0] &= 0x3B;
						if(0x04 ==Temp_JXS_Parameter.Origin[1] )
							Temp_JXS_Parameter.Origin[1] = 0x00;
						else
							Temp_JXS_Parameter.Origin[1] = 0x04;
						Temp_JXS_Parameter.Origin[2] &= 0x3B;
						Temp_JXS_Parameter.Origin[3] &= 0x3B;
						Temp_JXS_Parameter.Origin[4] &= 0x3B;
						Temp_JXS_Parameter.Origin[5] &= 0x3B;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
							case 0x5644:
						Temp_JXS_Parameter.Origin[0] &= 0x3D;
						if(0x02 ==Temp_JXS_Parameter.Origin[1] )
							Temp_JXS_Parameter.Origin[1] = 0x00;
						else
							Temp_JXS_Parameter.Origin[1] = 0x02;
						Temp_JXS_Parameter.Origin[2] &= 0x3D;
						Temp_JXS_Parameter.Origin[3] &= 0x3D;
						Temp_JXS_Parameter.Origin[4] &= 0x3D;
						Temp_JXS_Parameter.Origin[5] &= 0x3D;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
							case 0x5645:
						Temp_JXS_Parameter.Origin[0] &= 0x3E;
						if(0x01 ==Temp_JXS_Parameter.Origin[1] )
							Temp_JXS_Parameter.Origin[1] = 0x00;
						else
							Temp_JXS_Parameter.Origin[1] = 0x01;
						Temp_JXS_Parameter.Origin[2] &= 0x3E;
						Temp_JXS_Parameter.Origin[3] &= 0x3E;
						Temp_JXS_Parameter.Origin[4] &= 0x3E;
						Temp_JXS_Parameter.Origin[5] &= 0x3E;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
						 }
					}					
								 
					
	if((LCD_Var_Address >= 0x5646 && LCD_Var_Address <= 0x564B))  //第三行
					{
						switch(LCD_Var_Address)
						{
							case 0x5646:
						Temp_JXS_Parameter.Origin[0] &= 0x1F;
						Temp_JXS_Parameter.Origin[1] &= 0x1F;
						if(0x20 ==Temp_JXS_Parameter.Origin[2] )
							Temp_JXS_Parameter.Origin[2] = 0x00;
						else
							Temp_JXS_Parameter.Origin[2] = 0x20;
						Temp_JXS_Parameter.Origin[3] &= 0x1F;
						Temp_JXS_Parameter.Origin[4] &= 0x1F;
						Temp_JXS_Parameter.Origin[5] &= 0x1F;
						gs_Refresh_ORigin_Temp=TRUE;
						 break;
							case 0x5647:
						Temp_JXS_Parameter.Origin[0] &= 0x2F;
						Temp_JXS_Parameter.Origin[1] &= 0x2F;
						if(0x10 ==Temp_JXS_Parameter.Origin[2] )
							Temp_JXS_Parameter.Origin[2] = 0x00;
						else
							Temp_JXS_Parameter.Origin[2] = 0x10;
						Temp_JXS_Parameter.Origin[3] &= 0x2F;
						Temp_JXS_Parameter.Origin[4] &= 0x2F;
						Temp_JXS_Parameter.Origin[5] &= 0x2F;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
							case 0x5648:
						Temp_JXS_Parameter.Origin[0] &= 0x37;
						Temp_JXS_Parameter.Origin[1] &= 0x37;
						if(0x08 ==Temp_JXS_Parameter.Origin[2] )
							Temp_JXS_Parameter.Origin[2] = 0x00;
						else
							Temp_JXS_Parameter.Origin[2] = 0x08;
						Temp_JXS_Parameter.Origin[3] &= 0x37;
						Temp_JXS_Parameter.Origin[4] &= 0x37;
						Temp_JXS_Parameter.Origin[5] &= 0x37;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
							case 0x5649:
						Temp_JXS_Parameter.Origin[0] &= 0x3B;
						Temp_JXS_Parameter.Origin[1] &= 0x3B;
						if(0x04 ==Temp_JXS_Parameter.Origin[2] )
							Temp_JXS_Parameter.Origin[2] = 0x00;
						else
							Temp_JXS_Parameter.Origin[2] = 0x04;
						Temp_JXS_Parameter.Origin[3] &= 0x3B;
						Temp_JXS_Parameter.Origin[4] &= 0x3B;
						Temp_JXS_Parameter.Origin[5] &= 0x3B;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
							case 0x564A:
						Temp_JXS_Parameter.Origin[0] &= 0x3D;
						Temp_JXS_Parameter.Origin[1] &= 0x3D;
						if(0x02 ==Temp_JXS_Parameter.Origin[2] )
							Temp_JXS_Parameter.Origin[2] = 0x00;
						else
							Temp_JXS_Parameter.Origin[2] = 0x02;
						Temp_JXS_Parameter.Origin[3] &= 0x3D;
						Temp_JXS_Parameter.Origin[4] &= 0x3D;
						Temp_JXS_Parameter.Origin[5] &= 0x3D;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
							case 0x564B:
						Temp_JXS_Parameter.Origin[0] &= 0x3E;
						Temp_JXS_Parameter.Origin[1] &= 0x3E;
						if(0x01 ==Temp_JXS_Parameter.Origin[2] )
							Temp_JXS_Parameter.Origin[2] = 0x00;
						else
							Temp_JXS_Parameter.Origin[2] = 0x01;
						Temp_JXS_Parameter.Origin[3] &= 0x3E;
						Temp_JXS_Parameter.Origin[4] &= 0x3E;
						Temp_JXS_Parameter.Origin[5] &= 0x3E;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
						}
					}
				
	if((LCD_Var_Address >= 0x564C && LCD_Var_Address <= 0x5651))  //第四行
					{
						switch(LCD_Var_Address)
						{
							case 0x564C:
						Temp_JXS_Parameter.Origin[0] &= 0x1F;
						Temp_JXS_Parameter.Origin[1] &= 0x1F;
						Temp_JXS_Parameter.Origin[2] &= 0x1F;
						if(0x20 ==Temp_JXS_Parameter.Origin[3] )
							Temp_JXS_Parameter.Origin[3] = 0x00;
						else
							Temp_JXS_Parameter.Origin[3] = 0x20;
						Temp_JXS_Parameter.Origin[4] &= 0x1F;
						Temp_JXS_Parameter.Origin[5] &= 0x1F;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
							case 0x564D:
						Temp_JXS_Parameter.Origin[0] &= 0x2F;
						Temp_JXS_Parameter.Origin[1] &= 0x2F;
						Temp_JXS_Parameter.Origin[2] &= 0x2F;
						if(0x10 ==Temp_JXS_Parameter.Origin[3] )
							Temp_JXS_Parameter.Origin[3] = 0x00;
						else
							Temp_JXS_Parameter.Origin[3] = 0x10;
						Temp_JXS_Parameter.Origin[4] &= 0x2F;
						Temp_JXS_Parameter.Origin[5] &= 0x2F;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
							case 0x564E:
						Temp_JXS_Parameter.Origin[0] &= 0x37;
						Temp_JXS_Parameter.Origin[1] &= 0x37;
						Temp_JXS_Parameter.Origin[2] &= 0x37;
						if(0x08 ==Temp_JXS_Parameter.Origin[3] )
							Temp_JXS_Parameter.Origin[3] = 0x00;
						else
							Temp_JXS_Parameter.Origin[3] = 0x08;
						Temp_JXS_Parameter.Origin[4] &= 0x37;
						Temp_JXS_Parameter.Origin[5] &= 0x37;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
							case 0x564F:
						Temp_JXS_Parameter.Origin[0] &= 0x3B;
						Temp_JXS_Parameter.Origin[1] &= 0x3B;
						Temp_JXS_Parameter.Origin[2] &= 0x3B;
						if(0x04 ==Temp_JXS_Parameter.Origin[3] )
							Temp_JXS_Parameter.Origin[3] = 0x00;
						else
							Temp_JXS_Parameter.Origin[3] = 0x04;
						Temp_JXS_Parameter.Origin[4] &= 0x3B;
						Temp_JXS_Parameter.Origin[5] &= 0x3B;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
							case 0x5650:
						Temp_JXS_Parameter.Origin[0] &= 0x3D;
						Temp_JXS_Parameter.Origin[1] &= 0x3D;
						Temp_JXS_Parameter.Origin[2] &= 0x3D;
						if(0x02 ==Temp_JXS_Parameter.Origin[3] )
							Temp_JXS_Parameter.Origin[3] = 0x00;
						else
							Temp_JXS_Parameter.Origin[3] = 0x02;
						Temp_JXS_Parameter.Origin[4] &= 0x3D;
						Temp_JXS_Parameter.Origin[5] &= 0x3D;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
							case 0x5651:
						Temp_JXS_Parameter.Origin[0] &= 0x3E;
						Temp_JXS_Parameter.Origin[1] &= 0x3E;
						Temp_JXS_Parameter.Origin[2] &= 0x3E;
						if(0x01 ==Temp_JXS_Parameter.Origin[3] )
							Temp_JXS_Parameter.Origin[3] = 0x00;
						else
							Temp_JXS_Parameter.Origin[3] = 0x01;
						Temp_JXS_Parameter.Origin[4] &= 0x3E;
						Temp_JXS_Parameter.Origin[5] &= 0x3E;
						gs_Refresh_ORigin_Temp=TRUE;
								break;
						}		
					}
	//				
	if((LCD_Var_Address >= 0x5652 && LCD_Var_Address <= 0x5657))  //第五行
					{
						switch(LCD_Var_Address)
							{
								case 0x5652:
							Temp_JXS_Parameter.Origin[0] &= 0x1F;
							Temp_JXS_Parameter.Origin[1] &= 0x1F;
							Temp_JXS_Parameter.Origin[2] &= 0x1F;
							Temp_JXS_Parameter.Origin[3] &= 0x1F;
							if(0x20 ==Temp_JXS_Parameter.Origin[4] )
								Temp_JXS_Parameter.Origin[4] = 0x00;
							else
								Temp_JXS_Parameter.Origin[4] = 0x20;
							Temp_JXS_Parameter.Origin[5] &= 0x1F;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
							  case 0x5653:
							Temp_JXS_Parameter.Origin[0] &= 0x2F;
							Temp_JXS_Parameter.Origin[1] &= 0x2F;
							Temp_JXS_Parameter.Origin[2] &= 0x2F;
							Temp_JXS_Parameter.Origin[3] &= 0x2F;
							Temp_JXS_Parameter.Origin[4] = 0x10;
							Temp_JXS_Parameter.Origin[5] &= 0x2F;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
							  case 0x5654:
							Temp_JXS_Parameter.Origin[0] &= 0x37;
							Temp_JXS_Parameter.Origin[1] &= 0x37;
							Temp_JXS_Parameter.Origin[2] &= 0x37;
							Temp_JXS_Parameter.Origin[3] &= 0x37;
							if(0x08 ==Temp_JXS_Parameter.Origin[4] )
								Temp_JXS_Parameter.Origin[4] = 0x00;
							else
								Temp_JXS_Parameter.Origin[4] = 0x08;
							Temp_JXS_Parameter.Origin[5] &= 0x37;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
							  case 0x5655:
							Temp_JXS_Parameter.Origin[0] &= 0x3B;
							Temp_JXS_Parameter.Origin[1] &= 0x3B;
							Temp_JXS_Parameter.Origin[2] &= 0x3B;
							Temp_JXS_Parameter.Origin[3] &= 0x3B;
							if(0x04 ==Temp_JXS_Parameter.Origin[4] )
								Temp_JXS_Parameter.Origin[4] = 0x00;
							else
								Temp_JXS_Parameter.Origin[4] = 0x04;
							Temp_JXS_Parameter.Origin[5] &= 0x3B;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
							  case 0x5656:
							Temp_JXS_Parameter.Origin[0] &= 0x3D;
							Temp_JXS_Parameter.Origin[1] &= 0x3D;
							Temp_JXS_Parameter.Origin[2] &= 0x3D;
							Temp_JXS_Parameter.Origin[3] &= 0x3D;
							if(0x02 ==Temp_JXS_Parameter.Origin[4] )
								Temp_JXS_Parameter.Origin[4] = 0x00;
							else
								Temp_JXS_Parameter.Origin[4] = 0x02;
							Temp_JXS_Parameter.Origin[5] &= 0x3D;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
							  case 0x5657:
							Temp_JXS_Parameter.Origin[0] &= 0x3E;
							Temp_JXS_Parameter.Origin[1] &= 0x3E;
							Temp_JXS_Parameter.Origin[2] &= 0x3E;
							Temp_JXS_Parameter.Origin[3] &= 0x3E;
							if(0x01 ==Temp_JXS_Parameter.Origin[4] )
								Temp_JXS_Parameter.Origin[4] = 0x00;
							else
								Temp_JXS_Parameter.Origin[4] = 0x01;
							Temp_JXS_Parameter.Origin[5] &= 0x3E;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
						}
					}
	//			
	if((LCD_Var_Address >= 0x5658 && LCD_Var_Address <= 0x565D))  //第六行
					{
						switch(LCD_Var_Address)
						{
						case 0x5658:
							Temp_JXS_Parameter.Origin[0] &= 0x1F;
							Temp_JXS_Parameter.Origin[1] &= 0x1F;
							Temp_JXS_Parameter.Origin[2] &= 0x1F;
							Temp_JXS_Parameter.Origin[3] &= 0x1F;
							Temp_JXS_Parameter.Origin[4] &= 0x1F;
							if(0x20 ==Temp_JXS_Parameter.Origin[5] )
								Temp_JXS_Parameter.Origin[5] = 0x00;
							else
								Temp_JXS_Parameter.Origin[5] = 0x20;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
						case 0x5659:
							Temp_JXS_Parameter.Origin[0] &= 0x2F;
							Temp_JXS_Parameter.Origin[1] &= 0x2F;
							Temp_JXS_Parameter.Origin[2] &= 0x2F;
							Temp_JXS_Parameter.Origin[3] &= 0x2F;
							Temp_JXS_Parameter.Origin[4] &= 0x2F;
							if(0x10 ==Temp_JXS_Parameter.Origin[5] )
								Temp_JXS_Parameter.Origin[5] = 0x00;
							else
								Temp_JXS_Parameter.Origin[5] = 0x10;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
						case 0x565A:
							Temp_JXS_Parameter.Origin[0] &= 0x37;
							Temp_JXS_Parameter.Origin[1] &= 0x37;
							Temp_JXS_Parameter.Origin[2] &= 0x37;
							Temp_JXS_Parameter.Origin[3] &= 0x37;
							Temp_JXS_Parameter.Origin[4] &= 0x37;
							if(0x08 ==Temp_JXS_Parameter.Origin[5] )
								Temp_JXS_Parameter.Origin[5] = 0x00;
							else
								Temp_JXS_Parameter.Origin[5] = 0x08;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
						case 0x565B:
							Temp_JXS_Parameter.Origin[0] &= 0x3B;
							Temp_JXS_Parameter.Origin[1] &= 0x3B;
							Temp_JXS_Parameter.Origin[2] &= 0x3B;
							Temp_JXS_Parameter.Origin[3] &= 0x3B;
							Temp_JXS_Parameter.Origin[4] &= 0x3B;
							if(0x04 ==Temp_JXS_Parameter.Origin[5] )
								Temp_JXS_Parameter.Origin[5] = 0x00;
							else
								Temp_JXS_Parameter.Origin[5] = 0x04;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
						case 0x565C:
							Temp_JXS_Parameter.Origin[0] &= 0x3D;
							Temp_JXS_Parameter.Origin[1] &= 0x3D;
							Temp_JXS_Parameter.Origin[2] &= 0x3D;
							Temp_JXS_Parameter.Origin[3] &= 0x3D;
							Temp_JXS_Parameter.Origin[4] &= 0x3D;
							if(0x02 ==Temp_JXS_Parameter.Origin[5] )
								Temp_JXS_Parameter.Origin[5] = 0x00;
							else
								Temp_JXS_Parameter.Origin[5] = 0x02;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
						case 0x565D:
							Temp_JXS_Parameter.Origin[0] &= 0x3E;
							Temp_JXS_Parameter.Origin[1] &= 0x3E;
							Temp_JXS_Parameter.Origin[2] &= 0x3E;
							Temp_JXS_Parameter.Origin[3] &= 0x3E;
							Temp_JXS_Parameter.Origin[4] &= 0x3E;
							if(0x01 ==Temp_JXS_Parameter.Origin[5] )
								Temp_JXS_Parameter.Origin[5] = 0x00;
							else
								Temp_JXS_Parameter.Origin[5] = 0x01;
							gs_Refresh_ORigin_Temp=TRUE;
									break;
						 }
			  }		
					
	/****************TWS*****************/			 		 

		switch(LCD_Var_Address)
		{
			case 0x5600: //返回     复位设置参数            //X0
				LCD_Current_Page = 17;		
				JXS_Parameter.Origin[0]=	Temp_JXS_Parameter.Origin[0];	
				JXS_Parameter.Origin[1]=	Temp_JXS_Parameter.Origin[1];	
				JXS_Parameter.Origin[2]=	Temp_JXS_Parameter.Origin[2];	
				JXS_Parameter.Origin[3]=	Temp_JXS_Parameter.Origin[3];	
				JXS_Parameter.Origin[4]=	Temp_JXS_Parameter.Origin[4];	
				JXS_Parameter.Origin[5]=	Temp_JXS_Parameter.Origin[5];//复位顺序		
				JXS_Parameter.OriginDir[0]=	Temp_JXS_Parameter.OriginDir[0];	
				JXS_Parameter.OriginDir[1]=	Temp_JXS_Parameter.OriginDir[1];	
				JXS_Parameter.OriginDir[2]=	Temp_JXS_Parameter.OriginDir[2];	
				JXS_Parameter.OriginDir[3]=	Temp_JXS_Parameter.OriginDir[3];	
				JXS_Parameter.OriginDir[4]=	Temp_JXS_Parameter.OriginDir[4];	
				JXS_Parameter.OriginDir[5]=	Temp_JXS_Parameter.OriginDir[5];//复位方向
				JXS_Parameter.OriginSpeed[0]=	Temp_JXS_Parameter.OriginSpeed[0];	
				JXS_Parameter.OriginSpeed[1]=	Temp_JXS_Parameter.OriginSpeed[1];	
				JXS_Parameter.OriginSpeed[2]=	Temp_JXS_Parameter.OriginSpeed[2];	
				JXS_Parameter.OriginSpeed[3]=	Temp_JXS_Parameter.OriginSpeed[3];	
				JXS_Parameter.OriginSpeed[4]=	Temp_JXS_Parameter.OriginSpeed[4];	
				JXS_Parameter.OriginSpeed[5]=	Temp_JXS_Parameter.OriginSpeed[5];//复位速度
				JXS_Parameter.OriginPosition[0]=	Temp_JXS_Parameter.OriginPosition[0];	
				JXS_Parameter.OriginPosition[1]=	Temp_JXS_Parameter.OriginPosition[1];	
				JXS_Parameter.OriginPosition[2]=	Temp_JXS_Parameter.OriginPosition[2];	
				JXS_Parameter.OriginPosition[3]=	Temp_JXS_Parameter.OriginPosition[3];	
				JXS_Parameter.OriginPosition[4]=	Temp_JXS_Parameter.OriginPosition[4];	
				JXS_Parameter.OriginPosition[5]=	Temp_JXS_Parameter.OriginPosition[5];//复位零点
				Order_Temp[0] =PARAMETER_FUCTION_SET;
				Order_Temp[1] =0x08;
				Order_Temp[2] =1;
				Order_Temp[3] =0;
				Order_Temp[4] =Temp_JXS_Parameter.OriginDir[0];
				Order_Temp[5] =Temp_JXS_Parameter.OriginDir[1];
				Order_Temp[6] =Temp_JXS_Parameter.OriginDir[2];
				Order_Temp[7] =Temp_JXS_Parameter.OriginDir[3];
				Order_Temp[8] =Temp_JXS_Parameter.OriginDir[4];
				Order_Temp[9] =Temp_JXS_Parameter.OriginDir[5];//复位方向
				USART2_SendDataArray(10,PARAMETER_ORDER,Order_Temp);			
				DelayNms(200);
				
				Order_Temp[0] =PARAMETER_FUCTION_SET;
				Order_Temp[1] =0x08;
				Order_Temp[2] =2;				
				Order_Temp[3] =Temp_JXS_Parameter.OriginSpeed[0];
				Order_Temp[4] =Temp_JXS_Parameter.OriginSpeed[1];
				Order_Temp[5] =Temp_JXS_Parameter.OriginSpeed[2];
				Order_Temp[6] =Temp_JXS_Parameter.OriginSpeed[3];
				Order_Temp[7] =Temp_JXS_Parameter.OriginSpeed[4];
				Order_Temp[8] =Temp_JXS_Parameter.OriginSpeed[5];//复位速度
				USART2_SendDataArray(9,PARAMETER_ORDER,Order_Temp);
				DelayNms(200);
				
				Order_Temp[0] =PARAMETER_FUCTION_SET;
				Order_Temp[1] =0x08;
				Order_Temp[2] =3;
				Order_Temp[3] =Temp_JXS_Parameter.OriginPosition[0];
				Order_Temp[4] =Temp_JXS_Parameter.OriginPosition[1];
				Order_Temp[5] =Temp_JXS_Parameter.OriginPosition[2];
				Order_Temp[6] =Temp_JXS_Parameter.OriginPosition[3];
				Order_Temp[7] =Temp_JXS_Parameter.OriginPosition[4];
				Order_Temp[8] =Temp_JXS_Parameter.OriginPosition[5];//原点位置
				USART2_SendDataArray(9,PARAMETER_ORDER,Order_Temp);
				DelayNms(200);

				Order_Temp[0] =PARAMETER_FUCTION_SET;
				Order_Temp[1] =0x08;
				Order_Temp[2] =4;
				Order_Temp[3] =JXS_Parameter.Origin[0];
				Order_Temp[4] =JXS_Parameter.Origin[1];
				Order_Temp[5] =JXS_Parameter.Origin[2];
				Order_Temp[6] =JXS_Parameter.Origin[3];
				Order_Temp[7] =JXS_Parameter.Origin[4];
				Order_Temp[8] =JXS_Parameter.Origin[5];//复位顺序
				USART2_SendDataArray(9,PARAMETER_ORDER,Order_Temp);			
				 break;	

			case 0x5624://B轴复位速度  1-20      
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<21)
					Temp_JXS_Parameter.OriginSpeed[5]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginSpeed[5]=10;
				gs_Refresh_ORigin_Temp=TRUE;
				 break;
			case 0x5622://A轴复位速度                  
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<21)
					Temp_JXS_Parameter.OriginSpeed[4]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginSpeed[4]=10;						 
				gs_Refresh_ORigin_Temp=TRUE;
				 break;	
			case 0x5620://O轴复位速度                  
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<21)
					Temp_JXS_Parameter.OriginSpeed[3]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginSpeed[3]=10;				         
				gs_Refresh_ORigin_Temp=TRUE;
				 break;	
			case 0x560E://Z轴复位速度                  
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<21)
					Temp_JXS_Parameter.OriginSpeed[1]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginSpeed[1]=10;					         
				gs_Refresh_ORigin_Temp=TRUE;
				 break;
			case 0x560C://Y轴复位速度                  
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<21)
					Temp_JXS_Parameter.OriginSpeed[2]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginSpeed[2]=10;				         
				gs_Refresh_ORigin_Temp=TRUE;
				 break;
			case 0x560A://X轴复位速度        
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<21)
					Temp_JXS_Parameter.OriginSpeed[0]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginSpeed[0]=10;			
				gs_Refresh_ORigin_Temp=TRUE;
				 break;
			case 0x5610://X轴原点方向	
			   if(Admin_Authority)                               //管理员权限
				 {
					 if(Temp_JXS_Parameter.OriginDir[0])
					 {
					   Temp_JXS_Parameter.OriginDir[0] = 0;
					 }
					 else
					 {
						 Temp_JXS_Parameter.OriginDir[0] = 1;
					 }
				gs_Refresh_ORigin_Temp=TRUE;
				 }
			break;
			case 0x5612://Y轴原点方向
			   if(Admin_Authority)                               //管理员权限
				 {
					 if(Temp_JXS_Parameter.OriginDir[2])
					 {
					   Temp_JXS_Parameter.OriginDir[2] = 0;
					 }
					 else
					 {
						 Temp_JXS_Parameter.OriginDir[2] = 1;
					 }
				gs_Refresh_ORigin_Temp=TRUE;
				 }			 break;
			case 0x5614://Z轴原点方向
			   if(Admin_Authority)                               //管理员权限
				 {
					 if(Temp_JXS_Parameter.OriginDir[1])
					 {
					   Temp_JXS_Parameter.OriginDir[1] = 0;
					 }
					 else
					 {
						 Temp_JXS_Parameter.OriginDir[1] = 1;
					 }
				gs_Refresh_ORigin_Temp=TRUE;
				 }			break;
			case 0x5626://O轴原点方向         
			   if(Admin_Authority)                               //管理员权限
				 {
					 if(Temp_JXS_Parameter.OriginDir[3])
					 {
					   Temp_JXS_Parameter.OriginDir[3] = 0;
					 }
					 else
					 {
						 Temp_JXS_Parameter.OriginDir[3] = 1;
					 }
				gs_Refresh_ORigin_Temp=TRUE;
				 }			 break;
			case 0x5628://A轴原点方向                  				         
			   if(Admin_Authority)                               //管理员权限
				 {
					 if(Temp_JXS_Parameter.OriginDir[4])
					 {
					   Temp_JXS_Parameter.OriginDir[4] = 0;
					 }
					 else
					 {
						 Temp_JXS_Parameter.OriginDir[4] = 1;
					 }
				gs_Refresh_ORigin_Temp=TRUE;
				 }			 break;
			case 0x562A://B轴原点方向                  		         
			   if(Admin_Authority)                               //管理员权限
				 {
					 if(Temp_JXS_Parameter.OriginDir[5])
					 {
					   Temp_JXS_Parameter.OriginDir[5] = 0;
					 }
					 else
					 {
						 Temp_JXS_Parameter.OriginDir[5] = 1;
					 }
				gs_Refresh_ORigin_Temp=TRUE;
				 }			 break;
			case 0x5616: //X轴原点位置                 //X1
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<100)
					Temp_JXS_Parameter.OriginPosition[0]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginPosition[0]=10;				
				gs_Refresh_ORigin_Temp=TRUE;
				 break;	
			case 0x5618: //Y轴原点位置                 //X2
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<100)
					Temp_JXS_Parameter.OriginPosition[2]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginPosition[2]=10; 				         
				gs_Refresh_ORigin_Temp=TRUE;
				 break;
			case 0x561A:  //Z轴原点位置                //X3
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<100)
					Temp_JXS_Parameter.OriginPosition[1]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginPosition[1]=10;
				gs_Refresh_ORigin_Temp=TRUE;
				 break;
			case 0x562C:  //O轴原点位置                //X4
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<100)
					Temp_JXS_Parameter.OriginPosition[3]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginPosition[3]=10;			
				gs_Refresh_ORigin_Temp=TRUE;
				 break;	
			case 0x562E:  //A轴原点位置                //X5				        
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<100)
					Temp_JXS_Parameter.OriginPosition[4]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginPosition[4]=10;		
				gs_Refresh_ORigin_Temp=TRUE;
				 break;	
			case 0x5630:   //B轴原点位置                //X6 				       
				if(LCD_Var_Data/100>0&&LCD_Var_Data/100<100)
					Temp_JXS_Parameter.OriginPosition[5]=LCD_Var_Data/100;
				else
					Temp_JXS_Parameter.OriginPosition[5]=10;	
				gs_Refresh_ORigin_Temp=TRUE;
				 break;
			default:
				break;

		}
	}
}




/*************************************************************************
**  函数名：  SystemUpdate()
**	输入参数：
**	输出参数：无
**	函数功能：系统更新
**	备注：	  无
**  作者：    zj   
**  开发日期：2018/05/24
**************************************************************************/
void SystemUpdate()
{
	switch(LCD_Var_Address)
	{
		//USB更新：手持器系统更新
		case 0x5900:
			USBH_Process_Flag=TRUE;
			USB_Update_Flag=TRUE;
			USBH_Direction = Handset;
			LCD_Current_Page = 164;
			USB_Update_Status = USBSTATUS_NOUDISK;
			USB_Dispaly_Fresh_Flag=TRUE;
			if(USBH_USR_ApplicationState == USH_USR_FS_NULL)
			{
				USBH_USR_ApplicationState = USH_USR_FS_INIT;
			}
			USB_Button_Ok=TRUE;
			USB_Button_Cancle=TRUE;
			break;
		
		//USB更新：控制器系统更新		
    case 0x5901:
			USBH_Process_Flag=TRUE;
			USB_Update_Flag=TRUE;
			USBH_Direction = Controller;
			LCD_Current_Page = 164;
		  USB_Update_Status = USBSTATUS_NOUDISK;
		  USB_Dispaly_Fresh_Flag=TRUE;
			if(USBH_USR_ApplicationState == USH_USR_FS_NULL)
			{
				USBH_USR_ApplicationState = USH_USR_FS_INIT;
			}
			USB_Button_Ok=TRUE;
			USB_Button_Cancle=TRUE;
			break;
		
		//USB更新：返回		
    case 0x5902:
			LCD_Current_Page = 17;
			USBH_Process_Flag=FALSE;
 			USB_Button_Ok=FALSE;
			USB_Button_Cancle=FALSE;
			break;
		
		//USB状态：确定
    case 0x5903:
			if(USB_Button_Ok)
			{
				if(LCD_Current_Page == 165)
					USB_Dispaly_Fresh_Flag=TRUE;
				LCD_Current_Page = 165;
			}
			break;
			
		//USB状态：取消
    case 0x5904:
			if(USB_Button_Cancle)
			{
				LCD_Current_Page = 163;
				for(i=0;i<9;i++)
				{
					DisplayChineseString(0x5960+12*i,"            ",12);
				}
			}		
			break;

	case 0x5905:
			LCD_Current_Page = 166;
			File_Num = 0;
			USB_Dispaly_Fresh_Flag=TRUE;
			break;
		
    case 0x5906:
			LCD_Current_Page = 166;
			File_Num = 1;
			USB_Dispaly_Fresh_Flag=TRUE;
			break;
		
    case 0x5907:
			LCD_Current_Page = 166;
			File_Num = 2;
			USB_Dispaly_Fresh_Flag=TRUE;
			break;
		
    case 0x5908:
			LCD_Current_Page = 166;
			File_Num = 3;
			USB_Dispaly_Fresh_Flag=TRUE;
			break;

    case 0x5909:
			LCD_Current_Page = 166;
			File_Num = 4;
			USB_Dispaly_Fresh_Flag=TRUE;
			break;

		case 0x590A:
			LCD_Current_Page = 166;
			File_Num = 5;
			USB_Dispaly_Fresh_Flag=TRUE;
			break;
		
    case 0x590B:
			LCD_Current_Page = 166;
			File_Num = 6;
			USB_Dispaly_Fresh_Flag=TRUE;
			break;
		
    case 0x590C:
			LCD_Current_Page = 166;
			File_Num = 7;
			USB_Dispaly_Fresh_Flag=TRUE;
			break;
		
    case 0x590D:
			LCD_Current_Page = 166;
			File_Num = 8;
			USB_Dispaly_Fresh_Flag=TRUE;
			break;
		
		//USB更新：输入管理员密码	
    case 0x590E:
			Admin_Pass_Word = LCD_Var_Data;
		  gs_Refresh_Admin_Pass_Word = TRUE;
		  break;
		
		//USB更新：管理员密码正确并开始更新			
    case 0x590F:
			if(Pass_Word[1] == Admin_Pass_Word)
			{
				//LCD_Current_Page = 145;
				Admin_Pass_Word = 0;
				if(USBH_Direction == Handset)//手持器更新
				{
					AT24CXX_WriteOneByte(0xf0,0x0700);
				}
				else if(USBH_Direction == Controller)//控制板更新
				{
					AT24CXX_WriteOneByte(0x0f,0x0700);
				}
				AT24CXX_Write(0x0701,&Firmware_Name_Table[0][0] + File_Num*12,12);
				LCD_Current_Page = 169;
			}
			else
			{
				LCD_Popup_Windows = PW_5_PASSWORDERROR;
			}
			gs_Refresh_Admin_Pass_Word = TRUE;
			break;

		//USB更新：取消更新	
    case 0x5910:
			LCD_Current_Page = 165;
			break;
		
		//USB更新：固件列表返回			
    case 0x5911:
			LCD_Current_Page = 164;
			break;
	}
}

/****************TWS*****************/


/*************************************************************************
**  函数名：  LCDSystemSettingProcess()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void LCDSystemSettingProcess(void)
{
	if( Robot_Status != ACTION_RUN )
	{
		 //如果不是当前页面，则切换到当前界面
		if( (LCD_Var_Address == 0x5000) && (LCD_Main_Page != SYSTEM_SETTING))
		{
			LCD_Main_Page_Pre = LCD_Main_Page;
			LCD_Main_Page = SYSTEM_SETTING;
			LCD_Current_Page = 	LCD_Main_Page;
			g_Watch_IO_Control = 0; 						  
		    Order_Temp[0]=WAIT_MODE;
		    USART1_SendData(1,WORK_MODE,&Order_Temp[0]);
		}
		//存储点管理
		if((0x500C<=LCD_Var_Address) && (LCD_Var_Address<0x5030))
		{
			SavePointManager();	
		}
		//存储程序管理
		else if((0x5030<=LCD_Var_Address) && (LCD_Var_Address<0x5050))
		{
			SaveProgramManager();	
		}
		//软限位设定
		else if(((0x5050<=LCD_Var_Address) && (LCD_Var_Address<0x5060)) || ((0x5250<=LCD_Var_Address) && (LCD_Var_Address<0x5260)))
		{
			SoftLimitSetting();	
		}
		//用户设置
		else if(((0x5060<=LCD_Var_Address) && (LCD_Var_Address<0x5070)) || ((0x5500<=LCD_Var_Address) && (LCD_Var_Address<0x5550)))
		{
			UserSetting();	
		}
		//摇杆校准设定
		else if((0x5070<=LCD_Var_Address) && (LCD_Var_Address<0x5090))
		{
			SafeAreaSetting();	
		}
		//功能设定
		else if(LCD_Var_Address==0x52A0||((0x5300<=LCD_Var_Address) && (LCD_Var_Address<0x5370)) ||((0x5370<=LCD_Var_Address) && (LCD_Var_Address<0x53D2)))
		{
			FunctionSetting();	
		}
		//复位设置
		else if(((0x5600<=LCD_Var_Address) && (LCD_Var_Address<0x5900)))
		{
			ResetSetting();
		}
				//系统更新
		else if(((0x5900<=LCD_Var_Address) && (LCD_Var_Address<0x5A00)))
		{
			SystemUpdate();
		}		
		else
		{
			switch(LCD_Var_Address)
			{
				case 0x5001: 						 //用户设置
					 LCD_Current_Page = 49;
					 gs_Pass_Word_Refresh = 1;
					 break;							 //存储点管理
				case 0x5002: 				         
					 LCD_Current_Page = 41;
					 gs_Current_Choose_Point = 1;
					 gs_Point_Display_Refresh = TRUE;
					 break;	
				case 0x5003: 				         //程序管理
					 LCD_Current_Page = 45;
					 gs_Current_Choose_Program = 1;
					 gs_Program_Display_Refresh = TRUE;				 
					 break;
				case 0x5004: 				         		 
					 LCD_Current_Page = 47;			 //软限位设定
					 gs_Reflash_SoftLimit = TRUE;
					 break;
				case 0x5005: 						
					 LCD_Current_Page = 51;			 //安全区域设定
					 g_Current_Choose_Area= 1;
					 break;	
				case 0x5006: 				         //功能设定：机械手-料仓-生产
					 LCD_Current_Page= 76;
					 Temp_JXS_Parameter.Axis        = JXS_Parameter.Axis;
//					 Temp_JXS_Parameter.Origin      = JXS_Parameter.Origin;
// 					 Temp_JXS_Parameter.Accelerate = JXS_Parameter.Accelerate;  //---6.29
					 Temp_JXS_Parameter.OriginDir[0]  = JXS_Parameter.OriginDir[0];
					 Temp_JXS_Parameter.OriginDir[1]  = JXS_Parameter.OriginDir[1];
					 Temp_JXS_Parameter.OriginDir[2]  = JXS_Parameter.OriginDir[2];
					 Temp_JXS_Parameter.OriginDir[3]  = JXS_Parameter.OriginDir[3];
					 Temp_JXS_Parameter.OriginDir[4]  = JXS_Parameter.OriginDir[4];
					 Temp_JXS_Parameter.OriginDir[5]  = JXS_Parameter.OriginDir[5];
					 Temp_JXS_Parameter.SpeedLevel  =  JXS_Parameter.SpeedLevel;
					 Temp_JXS_Parameter.OriginSpeed[0] = JXS_Parameter.OriginSpeed[0];
					 Temp_JXS_Parameter.OriginSpeed[1] = JXS_Parameter.OriginSpeed[1];
					 Temp_JXS_Parameter.OriginSpeed[2] = JXS_Parameter.OriginSpeed[2];
					 Temp_JXS_Parameter.OriginSpeed[3] = JXS_Parameter.OriginSpeed[3];
					 Temp_JXS_Parameter.OriginSpeed[4] = JXS_Parameter.OriginSpeed[4];
					 Temp_JXS_Parameter.OriginSpeed[5] = JXS_Parameter.OriginSpeed[5];
					 Temp_JXS_Parameter.PulseTime  = JXS_Parameter.PulseTime;
					 Temp_JXS_Parameter.ServoAlarm  = JXS_Parameter.ServoAlarm;   //---2017.12.10设置伺服报警信号
					 Temp_JXS_Parameter.NcComplete = JXS_Parameter.NcComplete;    //---2017.3.20设置数据加工等待时间
					 gs_Reflash_Fuction_JXS = TRUE;
					 break;	
				case 0x5007:
					 LCD_Current_Page = 129;
					 gs_Refresh_ORigin_Temp=TRUE;
					 Temp_IO_Detect_Parameter[0]=IO_Detect_Parameter[0];
					 Temp_IO_Detect_Parameter[1]=IO_Detect_Parameter[1];
					 Temp_IO_Detect_Parameter[2]=IO_Detect_Parameter[2];
					 Temp_IO_Detect_Parameter[3]=IO_Detect_Parameter[3];
					 Temp_IO_Detect_Parameter[4]=IO_Detect_Parameter[4];
					 Temp_IO_Detect_Parameter[5]=IO_Detect_Parameter[5];
					 Temp_IO_Detect_Parameter[6]=IO_Detect_Parameter[6];
					 Temp_IO_Detect_Parameter[7]=IO_Detect_Parameter[7];
					 Temp_IO_Detect_Parameter[8]=IO_Detect_Parameter[8];
					 Temp_IO_Detect_Parameter[9]=IO_Detect_Parameter[9];
					 Temp_IO_Detect_Parameter[10]=IO_Detect_Parameter[10];
					 Temp_IO_Detect_Parameter[11]=IO_Detect_Parameter[11];
					 Temp_IO_Detect_Parameter[12]=IO_Detect_Parameter[12];
					 Temp_IO_Detect_Parameter[13]=IO_Detect_Parameter[13];
					 Temp_IO_Detect_Parameter[14]=IO_Detect_Parameter[14];
					 Temp_IO_Detect_Parameter[15]=IO_Detect_Parameter[15];
					 Temp_IO_Detect_Parameter[16]=IO_Detect_Parameter[16];
					 Temp_IO_Detect_Parameter[17]=IO_Detect_Parameter[17];

					 Temp_IO_Detect_Parameter[18]=IO_Detect_Parameter[18];
					 Temp_IO_Detect_Parameter[19]=IO_Detect_Parameter[19];
					 Temp_IO_Detect_Parameter[20]=IO_Detect_Parameter[20];
					 Temp_IO_Detect_Parameter[21]=IO_Detect_Parameter[21];
					 Temp_IO_Detect_Parameter[22]=IO_Detect_Parameter[22];
					 Temp_IO_Detect_Parameter[23]=IO_Detect_Parameter[23];
					 Temp_IO_Detect_Parameter[24]=IO_Detect_Parameter[24];
					 Temp_IO_Detect_Parameter[25]=IO_Detect_Parameter[25];
					 Temp_IO_Detect_Parameter[26]=IO_Detect_Parameter[26];
					 Temp_IO_Detect_Parameter[27]=IO_Detect_Parameter[27];
					 Temp_IO_Detect_Parameter[28]=IO_Detect_Parameter[28];
					 Temp_IO_Detect_Parameter[29]=IO_Detect_Parameter[29];
					 Temp_IO_Detect_Parameter[30]=IO_Detect_Parameter[30];
					 Temp_IO_Detect_Parameter[31]=IO_Detect_Parameter[31];
					 Temp_IO_Detect_Parameter[32]=IO_Detect_Parameter[32];
					 Temp_IO_Detect_Parameter[33]=IO_Detect_Parameter[33];
					 Temp_IO_Detect_Parameter[34]=IO_Detect_Parameter[34];
					 Temp_IO_Detect_Parameter[35]=IO_Detect_Parameter[35];
					 break;
				case 0x5008: 				         //系统更新
           LCD_Current_Page= 163;		
				case 0x5009: 				      
		 			 break;	
				case 0x500A: 				        
				     break;
				case 0x500B: 				        
				     break;
		 	       default:
				     break;		   	
			}	
		}
	}
	else
	{
//		LCD_Popup_Windows = 0x50;	//弹出提示框-请先停止机械手
	}	
}

/*************************************************************************
**  函数名：  IO1DetectDisplay()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    zj   
**  开发日期：2018/1/18
**************************************************************************/
void IO1DetectDisplay()
{
	//X0
	if((gs_Current_Choose_IO1&0x0001) != (gs_Pre_Choose_IO1&0x0001) )	
	{
		if(gs_Current_Choose_IO1&0x0001)	  //当前位为1，则显示效果
		{
			Display_PageCut(Save_IO_List[0]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[0]);
		}
	}
	//X1 
	if((gs_Current_Choose_IO1&0x0002) != (gs_Pre_Choose_IO1&0x0002) )	
	{
		if(gs_Current_Choose_IO1&0x0002)	  
		{
			Display_PageCut(Save_IO_List[1]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[1]);
		}
	}
	//X2 
	if((gs_Current_Choose_IO1&0x0004) != (gs_Pre_Choose_IO1&0x0004) )	
	{
		if(gs_Current_Choose_IO1&0x0004)	  
		{
			Display_PageCut(Save_IO_List[2]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[2]);
		}
	}
	//X3 
	if((gs_Current_Choose_IO1&0x0008) != (gs_Pre_Choose_IO1&0x0008) )	
	{
		if(gs_Current_Choose_IO1&0x0008)	  
		{
			Display_PageCut(Save_IO_List[3]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[3]);
		}
	}
	//X4
	if((gs_Current_Choose_IO1&0x0010) != (gs_Pre_Choose_IO1&0x0010) )	
	{
		if(gs_Current_Choose_IO1&0x0010)	  //当前位为1，则显示效果
		{
			Display_PageCut(Save_IO_List[4]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[4]);
		}
	}
	//X5 
	if((gs_Current_Choose_IO1&0x0020) != (gs_Pre_Choose_IO1&0x0020) )	
	{
		if(gs_Current_Choose_IO1&0x0020)	  
		{
			Display_PageCut(Save_IO_List[5]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[5]);
		}
	}
	//X6 
	if((gs_Current_Choose_IO1&0x0040) != (gs_Pre_Choose_IO1&0x0040) )	
	{
		if(gs_Current_Choose_IO1&0x0040)	  
		{
			Display_PageCut(Save_IO_List[6]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[6]);
		}
	}
	//X7 
	if((gs_Current_Choose_IO1&0x0080) != (gs_Pre_Choose_IO1&0x0080) )	
	{
		if(gs_Current_Choose_IO1&0x0080)	  
		{
			Display_PageCut(Save_IO_List[7]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[7]);
		}
	}
	//X8
	if((gs_Current_Choose_IO1&0x0100) != (gs_Pre_Choose_IO1&0x0100) )	
	{
		if(gs_Current_Choose_IO1&0x0100)	  //当前位为1，则显示效果
		{
			Display_PageCut(Save_IO_List[8]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[8]);
		}
	}
	//X9 
	if((gs_Current_Choose_IO1&0x0200) != (gs_Pre_Choose_IO1&0x0200) )	
	{
		if(gs_Current_Choose_IO1&0x0200)	  
		{
			Display_PageCut(Save_IO_List[9]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[9]);
		}
	}
	//X10 
	if((gs_Current_Choose_IO1&0x0400) != (gs_Pre_Choose_IO1&0x0400) )	
	{
		if(gs_Current_Choose_IO1&0x0400)	  
		{
			Display_PageCut(Save_IO_List[10]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[10]);
		}
	}
	//X11 
	if((gs_Current_Choose_IO1&0x0800) != (gs_Pre_Choose_IO1&0x0800) )	
	{
		if(gs_Current_Choose_IO1&0x0800)	  
		{
			Display_PageCut(Save_IO_List[11]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[11]);
		}
	}
	//X12
	if((gs_Current_Choose_IO1&0x1000) != (gs_Pre_Choose_IO1&0x1000) )	
	{
		if(gs_Current_Choose_IO1&0x1000)	  //当前位为1，则显示效果
		{
			Display_PageCut(Save_IO_List[12]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[12]);
		}
	}
	//X13 
	if((gs_Current_Choose_IO1&0x2000) != (gs_Pre_Choose_IO1&0x2000) )	
	{
		if(gs_Current_Choose_IO1&0x2000)	  
		{
			Display_PageCut(Save_IO_List[13]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[13]);
		}
	}
	//X14
	if((gs_Current_Choose_IO1&0x4000) != (gs_Pre_Choose_IO1&0x4000) )	
	{
		if(gs_Current_Choose_IO1&0x4000)	  
		{
			Display_PageCut(Save_IO_List[14]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[14]);
		}
	}
	//X15 
	if((gs_Current_Choose_IO1&0x8000) != (gs_Pre_Choose_IO1&0x8000) )	
	{
		if(gs_Current_Choose_IO1&0x8000)	  
		{
			Display_PageCut(Save_IO_List[15]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[15]);
		}
	}
}

/*************************************************************************
**  函数名：  IO2DetectDisplay()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    zj   
**  开发日期：2018/1/18
**************************************************************************/
void IO2DetectDisplay()
{
	//X16
	if((gs_Current_Choose_IO2&0x0001) != (gs_Pre_Choose_IO2&0x0001) )	
	{
		if(gs_Current_Choose_IO2&0x0001)	  //当前位为1，则显示效果
		{
			Display_PageCut(Save_IO_List[16]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[16]);
		}
	}
	//X17 
	if((gs_Current_Choose_IO2&0x0002) != (gs_Pre_Choose_IO2&0x0002) )	
	{
		if(gs_Current_Choose_IO2&0x0002)	  
		{
			Display_PageCut(Save_IO_List[17]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[17]);
		}
	}
	//X18 
	if((gs_Current_Choose_IO2&0x0004) != (gs_Pre_Choose_IO2&0x0004) )	
	{
		if(gs_Current_Choose_IO2&0x0004)	  
		{
			Display_PageCut(Save_IO_List[18]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[18]);
		}
	}
	//X19 
	if((gs_Current_Choose_IO2&0x0008) != (gs_Pre_Choose_IO2&0x0008) )	
	{
		if(gs_Current_Choose_IO2&0x0008)	  
		{
			Display_PageCut(Save_IO_List[19]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[19]);
		}
	}
	//X20
	if((gs_Current_Choose_IO2&0x0010) != (gs_Pre_Choose_IO2&0x0010) )	
	{
		if(gs_Current_Choose_IO2&0x0010)	  //当前位为1，则显示效果
		{
			Display_PageCut(Save_IO_List[20]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[20]);
		}
	}
	//X21 
	if((gs_Current_Choose_IO2&0x0020) != (gs_Pre_Choose_IO2&0x0020) )	
	{
		if(gs_Current_Choose_IO2&0x0020)	  
		{
			Display_PageCut(Save_IO_List[21]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[21]);
		}
	}
	//X22 
	if((gs_Current_Choose_IO2&0x0040) != (gs_Pre_Choose_IO2&0x0040) )	
	{
		if(gs_Current_Choose_IO2&0x0040)	  
		{
			Display_PageCut(Save_IO_List[22]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[22]);
		}
	}
	//X23 
	if((gs_Current_Choose_IO2&0x0080) != (gs_Pre_Choose_IO2&0x0080) )	
	{
		if(gs_Current_Choose_IO2&0x0080)	  
		{
			Display_PageCut(Save_IO_List[23]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[23]);
		}
	}
	//X24
	if((gs_Current_Choose_IO2&0x0100) != (gs_Pre_Choose_IO2&0x0100) )	
	{
		if(gs_Current_Choose_IO2&0x0100)	  //当前位为1，则显示效果
		{
			Display_PageCut(Save_IO_List[24]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[24]);
		}
	}
	//X25 
	if((gs_Current_Choose_IO2&0x0200) != (gs_Pre_Choose_IO2&0x0200) )	
	{
		if(gs_Current_Choose_IO2&0x0200)	  
		{
			Display_PageCut(Save_IO_List[25]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[25]);
		}
	}
	//X26
	if((gs_Current_Choose_IO2&0x0400) != (gs_Pre_Choose_IO2&0x0400) )	
	{
		if(gs_Current_Choose_IO2&0x0400)	  
		{
			Display_PageCut(Save_IO_List[26]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[26]);
		}
	}
	//X27 
	if((gs_Current_Choose_IO2&0x0800) != (gs_Pre_Choose_IO2&0x0800) )	
	{
		if(gs_Current_Choose_IO2&0x0800)	  
		{
			Display_PageCut(Save_IO_List[27]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[27]);
		}
	}
	//X28
	if((gs_Current_Choose_IO2&0x1000) != (gs_Pre_Choose_IO2&0x1000) )	
	{
		if(gs_Current_Choose_IO2&0x1000)	  //当前位为1，则显示效果
		{
			Display_PageCut(Save_IO_List[28]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[28]);
		}
	}
	//X29 
	if((gs_Current_Choose_IO2&0x2000) != (gs_Pre_Choose_IO2&0x2000) )	
	{
		if(gs_Current_Choose_IO2&0x2000)	  
		{
			Display_PageCut(Save_IO_List[29]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[29]);
		}
	}
	//X30
	if((gs_Current_Choose_IO2&0x4000) != (gs_Pre_Choose_IO2&0x4000) )	
	{
		if(gs_Current_Choose_IO2&0x4000)	  
		{
			Display_PageCut(Save_IO_List[30]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[30]);
		}
	}
	//X31
	if((gs_Current_Choose_IO2&0x8000) != (gs_Pre_Choose_IO2&0x8000) )	
	{
		if(gs_Current_Choose_IO2&0x8000)	  
		{
			Display_PageCut(Save_IO_List[31]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[31]);
		}
	}
}

/*************************************************************************
**  函数名：  IO3DetectDisplay()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    zj   
**  开发日期：2018/1/18
**************************************************************************/
void IO3DetectDisplay()
{
	//X32
	if((gs_Current_Choose_IO3&0x0001) != (gs_Pre_Choose_IO3&0x0001) )	
	{
		if(gs_Current_Choose_IO3&0x0001)	  //当前位为1，则显示效果
		{
			Display_PageCut(Save_IO_List[32]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[32]);
		}
	}
	//X33 
	if((gs_Current_Choose_IO3&0x0002) != (gs_Pre_Choose_IO3&0x0002) )	
	{
		if(gs_Current_Choose_IO3&0x0002)	  
		{
			Display_PageCut(Save_IO_List[33]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[33]);
		}
	}
	//X34 
	if((gs_Current_Choose_IO3&0x0004) != (gs_Pre_Choose_IO3&0x0004) )	
	{
		if(gs_Current_Choose_IO3&0x0004)	  
		{
			Display_PageCut(Save_IO_List[34]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[34]);
		}
	}
	//X35 
	if((gs_Current_Choose_IO3&0x0008) != (gs_Pre_Choose_IO3&0x0008) )	
	{
		if(gs_Current_Choose_IO3&0x0008)	  
		{
			Display_PageCut(Save_IO_List[35]);
		}
		else
		{
			Display_PageCut(Save_Pre_IO_List[35]);
		}
	}
}
	
/*************************************************************************
**  函数名：  LCDSystemSettingDisply()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void LCDSystemSettingDisply()
{
	u8 i=0;
	u8 *firmware_add = &Firmware_Name_Table[0][0];
	//--界面切换--//
	if(LCD_Pre_Page != LCD_Current_Page )	   
	{
		LCD_Pre_Page = LCD_Current_Page;
		PageChange(LCD_Pre_Page);
	}
	//--存储点管理--//
	if(gs_Current_Choose_Point != gs_Pre_Choose_Point )
	{
		if(gs_Current_Choose_Point == 0)
		{
			HiddenDisplay(0x5E10);
		}
		if(gs_Pre_Choose_Point == 0)
		{
			UnHiddenDisplay(0x5E10,0x5400);
		}
		if(gs_Current_Choose_Point>=1)
		{
			Display_PageCut(Save_Point_List[gs_Current_Choose_Point-1]); //图像剪切		
		}
		gs_Pre_Choose_Point=gs_Current_Choose_Point;	
	}
	if(gs_Point_Display_Refresh)
	{
		if(LCD_Current_Page==41)	                                     //X轴
		{
			for(i=0;i<25;i++)
			{
				Temp_Display_Data[0]=Manul_Save_PointX[i].Name>>24;
				Temp_Display_Data[1]=Manul_Save_PointX[i].Name>>16;
				Temp_Display_Data[2]=Manul_Save_PointX[i].Name>>8;
				Temp_Display_Data[3]=Manul_Save_PointX[i].Name;
				DisplayChineseString(0x5100+i*4,Temp_Display_Data,4);
				WriteNum4(0x5102+i*4,Manul_Save_PointX[i].Point);	
			}		
		}
		else if(LCD_Current_Page==44)	                                  //Z轴
		{
			for(i=0;i<25;i++)
			{
				Temp_Display_Data[0]=Manul_Save_PointZ[i].Name>>24;
				Temp_Display_Data[1]=Manul_Save_PointZ[i].Name>>16;
				Temp_Display_Data[2]=Manul_Save_PointZ[i].Name>>8;
				Temp_Display_Data[3]=Manul_Save_PointZ[i].Name;
				DisplayChineseString(0x5100+i*4,Temp_Display_Data,4);
				WriteNum4(0x5102+i*4,Manul_Save_PointZ[i].Point);
			}		
		}
		else if(LCD_Current_Page==115)	                                 //L轴
		{
			for(i=0;i<25;i++)
			{
				Temp_Display_Data[0]=Manul_Save_PointL[i].Name>>24;
				Temp_Display_Data[1]=Manul_Save_PointL[i].Name>>16;
				Temp_Display_Data[2]=Manul_Save_PointL[i].Name>>8;
				Temp_Display_Data[3]=Manul_Save_PointL[i].Name;
				DisplayChineseString(0x5100+i*4,Temp_Display_Data,4);
				WriteNum4(0x5102+i*4,Manul_Save_PointL[i].Point);
			}		
		}
		else if(LCD_Current_Page==123)	                                  //O轴
		{
			for(i=0;i<25;i++)
			{
				Temp_Display_Data[0]=Manul_Save_PointO[i].Name>>24;
				Temp_Display_Data[1]=Manul_Save_PointO[i].Name>>16;
				Temp_Display_Data[2]=Manul_Save_PointO[i].Name>>8;
				Temp_Display_Data[3]=Manul_Save_PointO[i].Name;
				DisplayChineseString(0x5100+i*4,Temp_Display_Data,4);
				WriteNum4(0x5102+i*4,Manul_Save_PointO[i].Point);
			}		
		}
		gs_Point_Display_Refresh = FALSE;	
	}
	//--程序管理--//
	if(gs_Current_Choose_Program != gs_Pre_Choose_Program )
	{	
		if(gs_Current_Choose_Program == 0)
		{
			HiddenDisplay(0x5E20);
		}
		if(gs_Pre_Choose_Program == 0)
		{
			UnHiddenDisplay(0x5E20,0x5410);
		}		
		Display_PageCut(Save_Program_List[gs_Current_Choose_Program-1]);  //图像剪切	
		gs_Pre_Choose_Program=gs_Current_Choose_Program;	
	}
	//--程序显示--//
	if(gs_Program_Display_Refresh)
	{
		for(i=0;i<SAVEPROGRAMNUM;i++)
		{
			Temp_Display_Data[0]=Free_Program_Save[i].Name>>24;
			Temp_Display_Data[1]=Free_Program_Save[i].Name>>16;
			Temp_Display_Data[2]=Free_Program_Save[i].Name>>8;
			Temp_Display_Data[3]=Free_Program_Save[i].Name;
			DisplayChineseString(0x5170+4*i,Temp_Display_Data,4);
			WriteNum2(0x5172+4*i,Free_Program_Save[i].Num);
		}
		gs_Program_Display_Refresh = FALSE;	
	}
	//--IO管理--//
	if((gs_Current_Choose_IO1 != gs_Pre_Choose_IO1 ) && (g_IO_DetectAck_Flag == TRUE))
	{
		IO1DetectDisplay();		
		gs_Pre_Choose_IO1=gs_Current_Choose_IO1;
    g_IO_DetectAck_Flag = FALSE;		
	}
		
	if((gs_Current_Choose_IO2 != gs_Pre_Choose_IO2 ) && (g_IO_DetectAck_Flag == TRUE))
	{
		IO2DetectDisplay();		
		gs_Pre_Choose_IO2=gs_Current_Choose_IO2;
    g_IO_DetectAck_Flag = FALSE;		
	}
	
	if((gs_Current_Choose_IO3 != gs_Pre_Choose_IO3 ) && (g_IO_DetectAck_Flag == TRUE))
	{
		IO3DetectDisplay();		
		gs_Pre_Choose_IO3=gs_Current_Choose_IO3;
		g_IO_DetectAck_Flag = FALSE;
    		
	}

	//--蜂鸣器 开-关--//
	if(gs_Pre_Buzzer_switch != g_Buzzer_Switch_Flag)
	{
	 	if(g_Buzzer_Switch_Flag)
		{
			DisplayChineseString(0x5061,Buzzer_Switch_On,2);
		}
		else
		{
		  DisplayChineseString(0x5061,Buzzer_Switch_Off,2);
		}
		gs_Pre_Buzzer_switch = g_Buzzer_Switch_Flag;
	}
	//--背光时间设定--//
	if(gs_BackLight_Refresh)
	{
		WriteNum4(0x5062,g_BackLight_Time);
		gs_BackLight_Refresh=0;
	}
	//--密码显示--//
	if(gs_Pass_Word_Refresh)
	{
		DisplayChineseString(0x5504,gs_Pass_Word_String,6);
		DisplayChineseString(0x5508,gs_Pass_Word_String,6);
		WriteNum4(0x5068,0);
		WriteNum4(0x506A,0);
		WriteNum4(0x506C,0);
		gs_Pass_Word_Refresh = 0;
	}
	//--安全区域设定--//
	if(g_Current_Choose_Area!= gs_Pre_Choose_Area )
	{	
		if(g_Current_Choose_Area== 0)
		{
			HiddenDisplay(0x5E30);
		}
		if(gs_Pre_Choose_Area == 0)
		{
			UnHiddenDisplay(0x5E30,0x5420);
		}		
		Display_PageCut(Safe_Area_List[g_Current_Choose_Area-1]);//图像剪切	
		gs_Pre_Choose_Area=g_Current_Choose_Area;
		WriteNum4(0x5078,Robot_Safe_Area[gs_Pre_Choose_Area-1].X_Left);
		WriteNum4(0x507A,Robot_Safe_Area[gs_Pre_Choose_Area-1].Z_Left);	
		WriteNum4(0x507C,Robot_Safe_Area[gs_Pre_Choose_Area-1].X_Right);
		WriteNum4(0x507E,Robot_Safe_Area[gs_Pre_Choose_Area-1].Z_Right);
		gs_Refresh_SafeArea_Temp=TRUE;
	}
	//--安全区功能 启用-关闭--//
	if(gs_Refresh_SafeArea_Temp)
	{
		if(Robot_Safe_Area[g_Current_Choose_Area-1].SafeArea_Switch)
		{
			DisplayChineseString(0x5082,X_SafeArea_On,4);
		}
		else
		{
		  DisplayChineseString(0x5082,X_SafeArea_Off,4);
		}
		gs_Refresh_SafeArea_Temp = FALSE;
	}
	//--摇杆校准--//
	if(LCD_Current_Page==0x33)
	{
		WriteNum4(0x507E,Rod_Display_X*100);//上下方向		
		WriteNum4(0x5078,Rod_Display_Y*100);//左右方向				
	}
	//--复位参数L--//
	if(gs_Refresh_ORigin_Temp)
	{
		/************************4.8***************************/
		for(i=0;i<6;i++)
		{
			if(Temp_JXS_Parameter.Origin[0]>>i==1)
			{
				switch(i)
				{
					case 5:Display_PageCut(row1_X_Y);						
					break;
					case 4:Display_PageCut(row1_Y_Y);						
					break;
					case 3:Display_PageCut(row1_Z_Y);						
					break;
					case 2:Display_PageCut(row1_O_Y);						
					break;
					case 1:Display_PageCut(row1_A_Y);						
					break;
					case 0:Display_PageCut(row1_B_Y);						
					break;
				}
			}
			else
			{
				switch(i)
				{
					case 5:Display_PageCut(row1_X_N);						
					break;
					case 4:Display_PageCut(row1_Y_N);						
					break;
					case 3:Display_PageCut(row1_Z_N);						
					break;
					case 2:Display_PageCut(row1_O_N);						
					break;
					case 1:Display_PageCut(row1_A_N);						
					break;
					case 0:Display_PageCut(row1_B_N);						
					break;
				}
			}
			
			if(Temp_JXS_Parameter.Origin[1]>>i==1)
			{
				switch(i)
				{
					case 5:Display_PageCut(row2_X_Y);						
					break;
					case 4:Display_PageCut(row2_Y_Y);						
					break;
					case 3:Display_PageCut(row2_Z_Y);						
					break;
					case 2:Display_PageCut(row2_O_Y);						
					break;
					case 1:Display_PageCut(row2_A_Y);						
					break;
					case 0:Display_PageCut(row2_B_Y);						
					break;
				}
			}
			else
			{
				switch(i)
				{
					case 5:Display_PageCut(row2_X_N);						
					break;
					case 4:Display_PageCut(row2_Y_N);						
					break;
					case 3:Display_PageCut(row2_Z_N);						
					break;
					case 2:Display_PageCut(row2_O_N);						
					break;
					case 1:Display_PageCut(row2_A_N);						
					break;
					case 0:Display_PageCut(row2_B_N);						
					break;
				}
			}

			if(Temp_JXS_Parameter.Origin[2]>>i==1)
			{
				switch(i)
				{
					case 5:Display_PageCut(row3_X_Y);						
					break;
					case 4:Display_PageCut(row3_Y_Y);						
					break;
					case 3:Display_PageCut(row3_Z_Y);						
					break;
					case 2:Display_PageCut(row3_O_Y);						
					break;
					case 1:Display_PageCut(row3_A_Y);						
					break;
					case 0:Display_PageCut(row3_B_Y);						
					break;
				}
			}
			else
			{
				switch(i)
				{
					case 5:Display_PageCut(row3_X_N);						
					break;
					case 4:Display_PageCut(row3_Y_N);						
					break;
					case 3:Display_PageCut(row3_Z_N);						
					break;
					case 2:Display_PageCut(row3_O_N);						
					break;
					case 1:Display_PageCut(row3_A_N);						
					break;
					case 0:Display_PageCut(row3_B_N);						
					break;
				}
			}

			if(Temp_JXS_Parameter.Origin[3]>>i==1)
			{
				switch(i)
				{
					case 5:Display_PageCut(row4_X_Y);						
					break;
					case 4:Display_PageCut(row4_Y_Y);						
					break;
					case 3:Display_PageCut(row4_Z_Y);						
					break;
					case 2:Display_PageCut(row4_O_Y);						
					break;
					case 1:Display_PageCut(row4_A_Y);						
					break;
					case 0:Display_PageCut(row4_B_Y);						
					break;
				}
			}
			else
			{
				switch(i)
				{
					case 5:Display_PageCut(row4_X_N);						
					break;
					case 4:Display_PageCut(row4_Y_N);						
					break;
					case 3:Display_PageCut(row4_Z_N);						
					break;
					case 2:Display_PageCut(row4_O_N);						
					break;
					case 1:Display_PageCut(row4_A_N);						
					break;
					case 0:Display_PageCut(row4_B_N);						
					break;
				}
			}
			if(Temp_JXS_Parameter.Origin[4]>>i==1)
			{
				switch(i)
				{
					case 5:Display_PageCut(row5_X_Y);						
					break;
					case 4:Display_PageCut(row5_Y_Y);						
					break;
					case 3:Display_PageCut(row5_Z_Y);						
					break;
					case 2:Display_PageCut(row5_O_Y);						
					break;
					case 1:Display_PageCut(row5_A_Y);						
					break;
					case 0:Display_PageCut(row5_B_Y);						
					break;
				}
			}
			else
			{
				switch(i)
				{
					case 5:Display_PageCut(row5_X_N);						
					break;
					case 4:Display_PageCut(row5_Y_N);						
					break;
					case 3:Display_PageCut(row5_Z_N);						
					break;
					case 2:Display_PageCut(row5_O_N);						
					break;
					case 1:Display_PageCut(row5_A_N);						
					break;
					case 0:Display_PageCut(row5_B_N);						
					break;
				}
			}
			if(Temp_JXS_Parameter.Origin[5]>>i==1)
			{
				switch(i)
				{
					case 5:Display_PageCut(row6_X_Y);						
					break;
					case 4:Display_PageCut(row6_Y_Y);						
					break;
					case 3:Display_PageCut(row6_Z_Y);						
					break;
					case 2:Display_PageCut(row6_O_Y);						
					break;
					case 1:Display_PageCut(row6_A_Y);						
					break;
					case 0:Display_PageCut(row6_B_Y);						
					break;
				}
			}
			else
			{
				switch(i)
				{
					case 5:Display_PageCut(row6_X_N);						
					break;
					case 4:Display_PageCut(row6_Y_N);						
					break;
					case 3:Display_PageCut(row6_Z_N);						
					break;
					case 2:Display_PageCut(row6_O_N);						
					break;
					case 1:Display_PageCut(row6_A_N);						
					break;
					case 0:Display_PageCut(row6_B_N);						
					break;
				}
			}
		}
			
		/*************************4.8**************************/		
		DisplayChineseString(0x5610,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[0]],4);		
		DisplayChineseString(0x5612,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[2]],4);		
		DisplayChineseString(0x5614,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[1]],4);		
		DisplayChineseString(0x5626,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[3]],4);		
		DisplayChineseString(0x5628,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[4]],4);		
		DisplayChineseString(0x562A,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[5]],4);		
		WriteNum4(0x560A,Temp_JXS_Parameter.OriginSpeed[0]*100);		
		WriteNum4(0x560C,Temp_JXS_Parameter.OriginSpeed[2]*100);		
		WriteNum4(0x560E,Temp_JXS_Parameter.OriginSpeed[1]*100);		
		WriteNum4(0x5620,Temp_JXS_Parameter.OriginSpeed[3]*100);		
		WriteNum4(0x5622,Temp_JXS_Parameter.OriginSpeed[4]*100);		
		WriteNum4(0x5624,Temp_JXS_Parameter.OriginSpeed[5]*100);	
		WriteNum4(0x5616,Temp_JXS_Parameter.OriginPosition[0]*100);		
		WriteNum4(0x5618,Temp_JXS_Parameter.OriginPosition[2]*100);		
		WriteNum4(0x561A,Temp_JXS_Parameter.OriginPosition[1]*100);		
		WriteNum4(0x562C,Temp_JXS_Parameter.OriginPosition[3]*100);		
		WriteNum4(0x562E,Temp_JXS_Parameter.OriginPosition[4]*100);		
		WriteNum4(0x5630,Temp_JXS_Parameter.OriginPosition[5]*100);		
		gs_Refresh_ORigin_Temp = FALSE;
	}
	//--XZ轴软限位参数显示--//
	if(gs_Reflash_SoftLimit)
	{
		WriteNum4(0x5050,Temp_Soft_Limit1[0].Left_Limit);
		WriteNum4(0x5052,Temp_Soft_Limit1[0].Right_Limit);
		WriteNum4(0x5054,Temp_Soft_Limit1[1].Left_Limit);
		WriteNum4(0x5056,Temp_Soft_Limit1[1].Right_Limit);
		WriteNum4(0x5250,Temp_Soft_Limit1[2].Left_Limit);
		WriteNum4(0x5252,Temp_Soft_Limit1[2].Right_Limit);
		WriteNum4(0x5254,Temp_Soft_Limit1[3].Left_Limit);
		WriteNum4(0x5256,Temp_Soft_Limit1[3].Right_Limit);
		WriteNum4(0x5058,Temp_Soft_Limit1[4].Right_Limit);
		WriteNum4(0x505A,Temp_Soft_Limit1[4].Left_Limit);
		WriteNum4(0x5258,Temp_Soft_Limit1[5].Right_Limit);
		WriteNum4(0x525A,Temp_Soft_Limit1[5].Left_Limit);
	 	//--软限位 开-关--//E_GearRatio
		WriteNum4(0x505C,JXS_Parameter.GearRatio.Ratio[0]*100);		
		WriteNum4(0x505E,JXS_Parameter.GearRatio.Ratio[1]*100);		
		WriteNum4(0x525C,JXS_Parameter.GearRatio.Ratio[2]*100);		
		WriteNum4(0x525E,JXS_Parameter.GearRatio.Ratio[3]*100);		
		WriteNum4(0x5060,JXS_Parameter.GearRatio.Ratio[4]*100);		
		WriteNum4(0x5260,JXS_Parameter.GearRatio.Ratio[5]*100);	
		gs_Reflash_SoftLimit = FALSE;
	}
	//--速度参数显示--//
	if(gs_Refresh_Speed_Temp&&LCD_Current_Page==76)
	{
		WriteNum4(0x5330,DZJSpeed1.ManualSpeed_L[0]*100);
		WriteNum4(0x5332,DZJSpeed1.ManualSpeed_H[0]*100);
		WriteNum4(0x5334,DZJSpeed1.AutoSpeed_L[0]*100);
		WriteNum4(0x5336,DZJSpeed1.AutoSpeed_H[0]*100);
		WriteNum4(0x5338,DZJSpeed1.Acceleration_UP[0]*100);
		WriteNum4(0x533A,DZJSpeed1.Acceleration_DOWN[0]*100);
		
		WriteNum4(0x5340,DZJSpeed1.ManualSpeed_L[1]*100);
		WriteNum4(0x5342,DZJSpeed1.ManualSpeed_H[1]*100);
		WriteNum4(0x5344,DZJSpeed1.AutoSpeed_L[1]*100);
		WriteNum4(0x5346,DZJSpeed1.AutoSpeed_H[1]*100);
		WriteNum4(0x5348,DZJSpeed1.Acceleration_UP[1]*100);
		WriteNum4(0x534A,DZJSpeed1.Acceleration_DOWN[1]*100);

		WriteNum4(0x5350,DZJSpeed1.ManualSpeed_L[2]*100);
		WriteNum4(0x5352,DZJSpeed1.ManualSpeed_H[2]*100);
		WriteNum4(0x5354,DZJSpeed1.AutoSpeed_L[2]*100);
		WriteNum4(0x5356,DZJSpeed1.AutoSpeed_H[2]*100);
		WriteNum4(0x5358,DZJSpeed1.Acceleration_UP[2]*100);
		WriteNum4(0x535A,DZJSpeed1.Acceleration_DOWN[2]*100);		
		gs_Refresh_Speed_Temp=FALSE;
	}
	//--功能设定--//
	if(gs_Reflash_Fuction_JXS)
	{		
//		DisplayChineseString(0x5310,FucSet_Axis[JXS_Parameter.Axis],12);
//		DisplayChineseString(0x5320,FucSet_Origin[JXS_Parameter.Origin],12);
//		//---显示X,Z,Y,O  6.28	
//		DisplayChineseString(0x5330,FucSet_Acc_Axis[Temp_ACC_Axis],1);
//		DisplayChineseString(0x530F,FucSet_Acc_Type[JXS_Parameter.Accelerate.Type[Temp_ACC_Axis]],1);
//		WriteNum2(0x5305,JXS_Parameter.Accelerate.Time[Temp_ACC_Axis]);
//		
//		DisplayChineseString(0x530C,Origin_DirectionX[OriginDir_Flag],4);
//		WriteNum2(0x5308,(u16)JXS_Parameter.SpeedLevel);
//		//---写回原点速度
//		WriteNum2(0x5309,(u16)JXS_Parameter.OriginSpeed);
//		//---手动速度
//		WriteNum2(0x536A,JXS_Parameter.ManualSpeed);
//		
//		//---2017.12.10显示伺服报警信号
//		DisplayChineseString(0x5346,FucSet_ServoAlarm[JXS_Parameter.ServoAlarm],1);
		
		//---2018.3.21写回电子齿轮比
//		DisplayChineseString(0x5343,FucSet_GearRatio[JXS_Parameter.GearRatio_Axis],1);
//		WriteNum2(0x5342,JXS_Parameter.GearRatio.Ratio[JXS_Parameter.GearRatio_Axis]);
		
	  //---2018.3.20写回数控加工等待时间
//		DisplayChineseString(0x5360,FucSet_NumericalControl[(JXS_Parameter.NcComplete&0x0f)],3);
//		WriteNum2(0x5348,(JXS_Parameter.NcComplete>>4));
		
    gs_Reflash_Fuction_JXS=FALSE;
	}
	if(gs_Reflash_Fuction_LC)//第一次进来料仓刷新
	{
		if(0<=LC_ID<=9)
		{
			DisplayChineseString(0x53A5,LC_Switch[LC_Parameter_Load[LC_ID].Switch],4);		
			DisplayChineseString(0x53B2,LC_Num[LC_ID],6);//料仓选择
			WriteNum2(0x53AD,LC_Parameter_Load[LC_ID].H_Num);
			WriteNum2(0x53AB,LC_Parameter_Load[LC_ID].V_Num);
			WriteNum4(0x53C6,LC_Parameter_Load[LC_ID].Position1.XPoint);
			WriteNum4(0x53C8,LC_Parameter_Load[LC_ID].Position1.YPoint);
			WriteNum4(0x53CA,LC_Parameter_Load[LC_ID].Position2.XPoint);
			WriteNum4(0x53CC,LC_Parameter_Load[LC_ID].Position2.YPoint);		    
			WriteNum4(0x53CE,LC_Parameter_Load[LC_ID].Position3.XPoint);
			WriteNum4(0x53D0,LC_Parameter_Load[LC_ID].Position3.YPoint);
			WriteNum4(0x53AF,LC_Parameter_Load[LC_ID].Glue_Time/10);
			WriteNum4(0x52A0,LC_Parameter_Load[LC_ID].Diamonds_Time/10);
			WriteNum2(0x53B8,LC_Parameter_Load[LC_ID].HT_Hight);
			WriteNum4(0x53BB,LC_Parameter_Load[LC_ID].LC_Hight);
		}
		gs_Reflash_Fuction_LC=FALSE;
	}
	if(gs_Reflash_Fuction_SC)
	{
		WriteNum4(0x5350,SC_Parameter.RW_Num);
		WriteNum4(0x5352,SC_Parameter.CJ_Num);
		WriteNum4(0x5354,SC_Parameter.JG_Num);
		WriteNum4(0x535A,SC_Parameter.SC_Num);
		WriteNum4(0x5356,SC_Parameter.LJ_Num);
		gs_LJProgress_Num = SC_Parameter.LJ_Num;
		gs_Reflash_Fuction_SC = FALSE;
	}
//	//临时显示刷新  机械手-轴个数
//	if(gs_Refresh_Axis_Num)
//	{		
//		DisplayChineseString(0x5310,FucSet_Axis[Temp_JXS_Parameter.Axis],12);
//		gs_Refresh_Axis_Num=FALSE;
//	}
//	//临时显示刷新  机械手-回原点方式
//	if(gs_Refresh_Origin_Method)
//	{
//		DisplayChineseString(0x5320,FucSet_Origin[Temp_JXS_Parameter.Origin],12);
//		gs_Refresh_Origin_Method=FALSE;
//	}
	//临时显示刷新  机械手-原点方向
	if(gs_Refresh_Origin_Dir)
	{
		DisplayChineseString(0x5628,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[0]],2);
		DisplayChineseString(0x5629,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[2]],2);
		DisplayChineseString(0x562A,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[1]],2);
		DisplayChineseString(0x5633,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[3]],2);
		DisplayChineseString(0x5634,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[4]],2);
		DisplayChineseString(0x5635,Origin_DirectionZ[Temp_JXS_Parameter.OriginDir[5]],2);
		gs_Refresh_Origin_Dir=FALSE;
	}	

	//临时显示刷新  料仓-开关
	if(gs_Refresh_LC_Switch)
	{
		switch(LC_ID)
		{
			case 0:
				DisplayChineseString(0x53A5,LC_Switch[Temp_LC_Parameter_Load[0].Switch],4);	//料仓1		
				break;
			case 1:
				DisplayChineseString(0x53A5,LC_Switch[Temp_LC_Parameter_Load[1].Switch],4);	//料仓2			
				break;
			case 2:
				DisplayChineseString(0x53A5,LC_Switch[Temp_LC_Parameter_Load[2].Switch],4);				
				break;
			case 3:
				DisplayChineseString(0x53A5,LC_Switch[Temp_LC_Parameter_Load[3].Switch],4);				
				break;
			case 4:
				DisplayChineseString(0x53A5,LC_Switch[Temp_LC_Parameter_Load[4].Switch],4);				
				break;
			case 5:
				DisplayChineseString(0x53A5,LC_Switch[Temp_LC_Parameter_Load[5].Switch],4);				
				break;
			case 6:
				DisplayChineseString(0x53A5,LC_Switch[Temp_LC_Parameter_Load[6].Switch],4);				
				break;
			case 7:
				DisplayChineseString(0x53A5,LC_Switch[Temp_LC_Parameter_Load[7].Switch],4);				
				break;
			case 8:
				DisplayChineseString(0x53A5,LC_Switch[Temp_LC_Parameter_Load[8].Switch],4);				
				break;
			case 9:
				DisplayChineseString(0x53A5,LC_Switch[Temp_LC_Parameter_Load[9].Switch],4);				
				break;
		}	
		gs_Refresh_LC_Switch=FALSE;
	}
	if(gs_Refresh_LC_Position)
	{
		if(0<=LC_ID<=9)
		{
			DisplayChineseString(0x53A5,LC_Switch[LC_Parameter_Load[LC_ID].Switch],4);		
			DisplayChineseString(0x53B2,LC_Num[LC_ID],6);//料仓选择
			WriteNum2(0x53AD,Temp_LC_Parameter_Load[LC_ID].H_Num);
			WriteNum2(0x53AB,Temp_LC_Parameter_Load[LC_ID].V_Num);
//			ValueToString(Temp_LC_Parameter_Load[LC_ID].Glue_Time);
			WriteNum4(0x53AF,Temp_LC_Parameter_Load[LC_ID].Glue_Time/10);
//			ValueToString(Temp_LC_Parameter_Load[LC_ID].Diamonds_Time);
			WriteNum4(0x52A0,Temp_LC_Parameter_Load[LC_ID].Diamonds_Time/10);
			WriteNum4(0x53C6,Temp_LC_Parameter_Load[LC_ID].Position1.XPoint);
			WriteNum4(0x53C8,Temp_LC_Parameter_Load[LC_ID].Position1.YPoint);
			WriteNum4(0x53CA,Temp_LC_Parameter_Load[LC_ID].Position2.XPoint);
			WriteNum4(0x53CC,Temp_LC_Parameter_Load[LC_ID].Position2.YPoint);
			WriteNum4(0x53CE,Temp_LC_Parameter_Load[LC_ID].Position3.XPoint);
			WriteNum4(0x53D0,Temp_LC_Parameter_Load[LC_ID].Position3.YPoint);
			WriteNum2(0x53B8,Temp_LC_Parameter_Load[LC_ID].HT_Hight);
			WriteNum4(0x53BB,Temp_LC_Parameter_Load[LC_ID].LC_Hight);
			i=0;
		}
		gs_Refresh_LC_Position=FALSE;	
	}	
//	if(gs_SCProgress_Num != SC_Parameter.LJ_Num) //累计加工数量在刷新
//	{
//		WriteNum4(0x535A,SC_Parameter.LJ_Num);
//		gs_LJProgress_Num = SC_Parameter.LJ_Num;
//	}
	//累计加工数量在刷新
	if(gs_LJProgress_Num != SC_Parameter.LJ_Num) 
	{
		WriteNum4(0x5356,SC_Parameter.LJ_Num);
		gs_LJProgress_Num = SC_Parameter.LJ_Num;
		Temp_SC_Parameter.LJ_Num = SC_Parameter.LJ_Num;
		Temp_SC_Parameter.SC_Num = SC_Parameter.SC_Num;
	}
	if(gs_Clear_LJ_Parametr)
	{
		WriteNum4(0x5356,Temp_SC_Parameter.LJ_Num);
		gs_Clear_LJ_Parametr = FALSE;
	}
	if(gs_Clear_SC_Parametr)
	{
		WriteNum4(0x535A,Temp_SC_Parameter.SC_Num);
		gs_Clear_SC_Parametr = FALSE;
	}
  //权限显示
	if(Refresh_Admin_Flag)
	{
  	  DisplayChineseString(0x1030,Admin_Authority_Dis[Admin_Authority],10);
	    Refresh_Admin_Flag=FALSE;	
	}
	//U盘拷贝显示
	if((USBH_Dispaly_Fresh_Flag==1) || (USBH_Copy_Status!=gs_USBH_Copy_Status))
	{
		USBH_Dispaly_Fresh_Flag=0;
		DisplayChineseString(0x5600,USBH_And_Control[USBH_Direction],14);
		DisplayChineseString(0x5610,USBH_CopyStatus_String[USBH_Copy_Status],22);
		gs_USBH_Copy_Status=USBH_Copy_Status;
	}
	//U盘更新显示
	if((LCD_Pre_Page != 166)&&(USB_Dispaly_Fresh_Flag==TRUE))
	{
		USB_Dispaly_Fresh_Flag=FALSE;
		DisplayChineseString(0x5920,USB_Update_Choice[USBH_Direction],14);
		DisplayChineseString(0x5930,USB_UpdateStatus_String[USB_Update_Status],24);
	}
	
	if((LCD_Pre_Page == 166)&&(USB_Dispaly_Fresh_Flag==TRUE))
	{
		USB_Dispaly_Fresh_Flag=FALSE;
		firmware_add = &Firmware_Name_Table[0][0] + File_Num*12;
		DisplayChineseString(0x5A10,firmware_add,10);//显示选择的固件名
	}
	
	//U盘更新 管理员密码显示
	if(gs_Refresh_Admin_Pass_Word)
	{
		WriteNum4(0x590E,Admin_Pass_Word);
		gs_Refresh_Admin_Pass_Word=FALSE;		
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
	if((Run_Program_Num>0)&&(Current_Operate_Program.Code==Run_Program_Num)&&(Current_Operate_Program.Program[0].Key==K_PROGRAMSTART))
	{
		for(i=0;i<Current_Operate_Program.Num;i++)
		{
		   if(Current_Operate_Program.Program[i].Key==K_LOADPOSITION)
		   {
			   if(Current_Operate_Program.Program[i].Value3==11) //上料X
			   {
			   	   LC_Load_Pre_ListX=i;
			   }
			   else												 //上料Y
			   {
			   	   LC_Load_Pre_ListY=i;
			   }
		   }
		   else if(Current_Operate_Program.Program[i].Key==K_UNLOADPOSITION)
		   {
			   if(Current_Operate_Program.Program[i].Value3==11) //卸料X
			   {
			   	   LC_UnLoad_Pre_ListX=i;
			   }												 
			   else												 //卸料Y
			   {
			   	   LC_UnLoad_Pre_ListY=i;
			   }
		   }
		   else if(Current_Operate_Program.Program[i].Key==K_ZAXISPOSITION)
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
void Calculate_LC_Load_Parameter()
{
  u32 HD=0,HDL=0,VD=0,VDL=0;  //横向间距   纵向间距
	u32 DeltaH=0,DeltaV=0;	    //横向补偿   纵向补偿
	u8 i=0,j=0,k=1;					    //i表示横向X  j表示纵向Y			    //i表示横向X  j表示纵向Y

	g_Calculate_LC_Load_Parameter_Flag=FALSE;
	if((LC_Parameter_Load[LC_ID].Switch==FALSE) || ((LC_Parameter_Load[LC_ID].H_Num==1)&&(LC_Parameter_Load[LC_ID].V_Num==1)))
	{
	  LC_Load_Position[0][0].XPoint=LC_Parameter_Load[LC_ID].Position1.XPoint;
		LC_Load_Position[0][0].YPoint=LC_Parameter_Load[LC_ID].Position1.YPoint;	
	}
	else if( (LC_Parameter_Load[LC_ID].Switch==TRUE)&&(LC_Parameter_Load[LC_ID].H_Num>=1)&&(LC_Parameter_Load[LC_ID].V_Num>=1)
	    &&(LC_Parameter_Load[LC_ID].Position1.XPoint<=LC_Parameter_Load[LC_ID].Position2.XPoint)
	    &&(LC_Parameter_Load[LC_ID].Position2.YPoint<=LC_Parameter_Load[LC_ID].Position3.YPoint) )
	{	
	    if(LC_Parameter_Load[LC_ID].H_Num==1)
		{}
		else
		{
	        HD =(LC_Parameter_Load[LC_ID].Position2.XPoint-LC_Parameter_Load[LC_ID].Position1.XPoint)/(LC_Parameter_Load[LC_ID].H_Num-1);  //取整
	    	HDL=(LC_Parameter_Load[LC_ID].Position2.XPoint-LC_Parameter_Load[LC_ID].Position1.XPoint)%(LC_Parameter_Load[LC_ID].H_Num-1);  //取余
		}
	    if(LC_Parameter_Load[LC_ID].H_Num==1)
		{}
		else
		{
		    VD =(LC_Parameter_Load[LC_ID].Position3.YPoint-LC_Parameter_Load[LC_ID].Position2.YPoint)/(LC_Parameter_Load[LC_ID].V_Num-1);	 //取整
	    	VDL=(LC_Parameter_Load[LC_ID].Position3.YPoint-LC_Parameter_Load[LC_ID].Position2.YPoint)%(LC_Parameter_Load[LC_ID].V_Num-1);	 //取余
		}
		for(j=0;j<LC_Parameter_Load[LC_ID].V_Num;j++)
		{
			for(i=0;i<LC_Parameter_Load[LC_ID].H_Num;i++)
			{	//以X1/Y1为参考点进行计算的
			    LC_Load_Position[i][j].XPoint=LC_Parameter_Load[LC_ID].Position1.XPoint+i*HD;
				LC_Load_Position[i][j].YPoint=LC_Parameter_Load[LC_ID].Position1.YPoint+j*VD;
			}
		}
		//补偿余数
		k=1;
		for(i=(LC_Parameter_Load[LC_ID].H_Num-HDL);i<LC_Parameter_Load[LC_ID].H_Num;i++)
		{
			for(j=0;j<LC_Parameter_Load[LC_ID].V_Num;j++)
			{
			    LC_Load_Position[i][j].XPoint=LC_Load_Position[i][j].XPoint+k*1;
			}
			k++;
		}
		k=1;
		for(j=(LC_Parameter_Load[LC_ID].V_Num-VDL);j<LC_Parameter_Load[LC_ID].V_Num;j++)
		{
			for(i=0;i<LC_Parameter_Load[LC_ID].H_Num;i++)
			{
				LC_Load_Position[i][j].YPoint=LC_Load_Position[i][j].YPoint+k*1;
			}
			k++;
		}
		//计算偏移量
		if(LC_Parameter_Load[LC_ID].Position2.YPoint>LC_Parameter_Load[LC_ID].Position1.YPoint)	 //Y向外倾斜 X向右倾斜
		{	//以右边为第一行第一列
		    if(LC_Parameter_Load[LC_ID].V_Num==1)
			{}
			else
			{
		        DeltaH=(LC_Parameter_Load[LC_ID].Position2.XPoint-LC_Parameter_Load[LC_ID].Position3.XPoint)/(LC_Parameter_Load[LC_ID].V_Num-1); //第二行的偏移
			}
		    if(LC_Parameter_Load[LC_ID].H_Num==1)
			{}
			else
			{
			    DeltaV=(LC_Parameter_Load[LC_ID].Position2.YPoint-LC_Parameter_Load[LC_ID].Position1.YPoint)/(LC_Parameter_Load[LC_ID].H_Num-1); //第二列的偏移
		    }
			for(j=0;j<LC_Parameter_Load[LC_ID].V_Num;j++)
			{
				for(i=0;i<LC_Parameter_Load[LC_ID].H_Num;i++)
				{
				    LC_Load_Position[i][j].XPoint=LC_Load_Position[i][j].XPoint-j*DeltaH;
					LC_Load_Position[i][j].YPoint=LC_Load_Position[i][j].YPoint+i*DeltaV;
				}
			}
		}
		else if(LC_Parameter_Load[LC_ID].Position2.YPoint<LC_Parameter_Load[LC_ID].Position1.YPoint)	 //Y向内倾斜 X向左倾斜
		{	//以右边为第一行第一列
		    if(LC_Parameter_Load[LC_ID].V_Num==1)
			{}
			else
			{
		        DeltaH=(LC_Parameter_Load[LC_ID].Position3.XPoint-LC_Parameter_Load[LC_ID].Position2.XPoint)/(LC_Parameter_Load[LC_ID].V_Num-1); //第二行的偏移
		    }
			if(LC_Parameter_Load[LC_ID].H_Num==1)
			{}
			else
			{
			    DeltaV=(LC_Parameter_Load[LC_ID].Position1.YPoint-LC_Parameter_Load[LC_ID].Position2.YPoint)/(LC_Parameter_Load[LC_ID].H_Num-1); //第二列的偏移
		    }
			for(j=0;j<LC_Parameter_Load[LC_ID].V_Num;j++)
			{
				for(i=0;i<LC_Parameter_Load[LC_ID].H_Num;i++)
				{
				    LC_Load_Position[i][j].XPoint=LC_Load_Position[i][j].XPoint+j*DeltaH;
					LC_Load_Position[i][j].YPoint=LC_Load_Position[i][j].YPoint-i*DeltaV;
				}
			}
		}
		g_Calculate_LC_Load_Parameter_Flag=TRUE;
		g_LC_Load_Position_Flag=TRUE;
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
    u32 HD=0,HDL=0,VD=0,VDL=0;//横向间距   纵向间距
	u32 DeltaH=0,DeltaV=0;	  //横向补偿   纵向补偿
	u8 i=0,j=0,k=1;					    //i表示横向X  j表示纵向Y

	g_Calculate_LC_UnLoad_Parameter_Flag=FALSE;
	if((LC_Parameter_UnLoad.Switch==FALSE) || ((LC_Parameter_UnLoad.H_Num==1)&&(LC_Parameter_UnLoad.V_Num==1)))
	{
	    LC_UnLoad_Position[0][0].XPoint=LC_Parameter_UnLoad.Position1.XPoint;
		LC_UnLoad_Position[0][0].YPoint=LC_Parameter_UnLoad.Position1.YPoint;	
	}
	else if( (LC_Parameter_UnLoad.Switch==TRUE)&&(LC_Parameter_UnLoad.H_Num>=1)&&(LC_Parameter_UnLoad.V_Num>=1)
	    &&(LC_Parameter_UnLoad.Position1.XPoint<=LC_Parameter_UnLoad.Position2.XPoint)
	    &&(LC_Parameter_UnLoad.Position2.YPoint<=LC_Parameter_UnLoad.Position3.YPoint) )
	{	
	    if(LC_Parameter_UnLoad.H_Num==1)
		{}
		else
		{
		    HD =(LC_Parameter_UnLoad.Position2.XPoint-LC_Parameter_UnLoad.Position1.XPoint)/(LC_Parameter_UnLoad.H_Num-1);   //取整
		    HDL=(LC_Parameter_UnLoad.Position2.XPoint-LC_Parameter_UnLoad.Position1.XPoint)%(LC_Parameter_UnLoad.H_Num-1);	 //取余
	    }
	    if(LC_Parameter_UnLoad.V_Num==1)
		{}
		else
		{
	        VD =(LC_Parameter_UnLoad.Position3.YPoint-LC_Parameter_UnLoad.Position2.YPoint)/(LC_Parameter_UnLoad.V_Num-1);	 //取整
			VDL=(LC_Parameter_UnLoad.Position3.YPoint-LC_Parameter_UnLoad.Position2.YPoint)%(LC_Parameter_UnLoad.V_Num-1);	 //取余
	    }
		//计算平均值
		for(j=0;j<LC_Parameter_UnLoad.V_Num;j++)
		{
			for(i=0;i<LC_Parameter_UnLoad.H_Num;i++)
			{	//以X1/Y1为参考点进行计算的
			    LC_UnLoad_Position[i][j].XPoint=LC_Parameter_UnLoad.Position1.XPoint+i*HD;
				LC_UnLoad_Position[i][j].YPoint=LC_Parameter_UnLoad.Position1.YPoint+j*VD;
			}
		}
		//补偿余数
		k=1;
		for(i=(LC_Parameter_UnLoad.H_Num-HDL);i<LC_Parameter_UnLoad.H_Num;i++)
		{
			for(j=0;j<LC_Parameter_UnLoad.V_Num;j++)
			{
			    LC_UnLoad_Position[i][j].XPoint=LC_UnLoad_Position[i][j].XPoint+k*1;
			}
			k++;
		}
		k=1;
		for(j=(LC_Parameter_UnLoad.V_Num-VDL);j<LC_Parameter_UnLoad.V_Num;j++)
		{
			for(i=0;i<LC_Parameter_UnLoad.H_Num;i++)
			{
				LC_UnLoad_Position[i][j].YPoint=LC_UnLoad_Position[i][j].YPoint+k*1;
			}
			k++;
		}
		//计算偏移量
		if(LC_Parameter_UnLoad.Position2.YPoint>LC_Parameter_UnLoad.Position1.YPoint)	 //Y向外倾斜 X向右倾斜
		{	//以右边为第一行第一列
		    if(LC_Parameter_UnLoad.V_Num==1)
			{}
			else
			{
		        DeltaH=(LC_Parameter_UnLoad.Position2.XPoint-LC_Parameter_UnLoad.Position3.XPoint)/(LC_Parameter_UnLoad.V_Num-1); //第二行的偏移
			}
		    if(LC_Parameter_UnLoad.H_Num==1)
			{}
			else
			{
			    DeltaV=(LC_Parameter_UnLoad.Position2.YPoint-LC_Parameter_UnLoad.Position1.YPoint)/(LC_Parameter_UnLoad.H_Num-1); //第二列的偏移
		    }
			for(j=0;j<LC_Parameter_UnLoad.V_Num;j++)
			{
				for(i=0;i<LC_Parameter_UnLoad.H_Num;i++)
				{
				    LC_UnLoad_Position[i][j].XPoint=LC_UnLoad_Position[i][j].XPoint-j*DeltaH;
					LC_UnLoad_Position[i][j].YPoint=LC_UnLoad_Position[i][j].YPoint+i*DeltaV;
				}
			}
		}
		else if(LC_Parameter_UnLoad.Position2.YPoint<LC_Parameter_UnLoad.Position1.YPoint)	 //Y向内倾斜 X向左倾斜
		{	//以右边为第一行第一列
		    if(LC_Parameter_UnLoad.V_Num==1)
			{}
			else
			{
		        DeltaH=(LC_Parameter_UnLoad.Position3.XPoint-LC_Parameter_UnLoad.Position2.XPoint)/(LC_Parameter_UnLoad.V_Num-1); //第二行的偏移
			}
		    if(LC_Parameter_UnLoad.V_Num==1)
			{}
			else
			{
			    DeltaV=(LC_Parameter_UnLoad.Position1.YPoint-LC_Parameter_UnLoad.Position2.YPoint)/(LC_Parameter_UnLoad.H_Num-1); //第二列的偏移
		    }
			for(j=0;j<LC_Parameter_UnLoad.V_Num;j++)
			{
				for(i=0;i<LC_Parameter_UnLoad.H_Num;i++)
				{
				    LC_UnLoad_Position[i][j].XPoint=LC_UnLoad_Position[i][j].XPoint+j*DeltaH;
					LC_UnLoad_Position[i][j].YPoint=LC_UnLoad_Position[i][j].YPoint-i*DeltaV;
				}
			}
		}
		g_Calculate_LC_UnLoad_Parameter_Flag=TRUE;
		g_LC_UnLoad_Position_Flag=TRUE;
	}

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
