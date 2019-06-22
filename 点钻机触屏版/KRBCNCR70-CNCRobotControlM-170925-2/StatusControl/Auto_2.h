/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : Auto_2.h
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 21/10/2015
* Description        : 自动运行头文件
***************************************************************************************/
#ifndef _Auto_2_h_
#define _Auto_2_h_

#define AUTO_WAITE				0
#define AUTO_RUNNING			1
#define AUTO_PAUSE				2
#define AUTO_ERROR				3


typedef struct 
{
	u8  Flag;	   //
	u8  Line;	   //
}ActionStepList;
#define LISTNUM 6


extern u8  g_AutoStatue;
extern u16  g_Auto_PresentLine;
extern u32 g_Auto_Circle_Timer;
extern u32 g_Auto_ProduceCounter;
extern u8  g_LC_Program_Step_Run;
extern u8  g_LC_Step_Control;
extern u8  g_LC_Load_Finish;			
extern u8  g_LC_Load_Permit;		
extern u8  g_LC_UnLoad_Finish;	
extern u8  g_LC_UnLoad_Permit;	  
extern u8  g_Read_LC_Load_Program;
extern u8  g_Read_LC_UnLoad_Program;
extern u8  g_LC_PresentLine;
extern u8  g_Start_Action_PresentLine;
extern u8  g_Start_Step_Control;
extern u8  g_Reset_Action_PresentLine;
extern u8  g_Reset_Step_Control;

extern u8  g_Auto_Order_Start;
extern u8  g_Auto_Order_Pause;
extern u8  g_Auto_Order_Stop;
extern u8  g_Auto_Order_StackReset;


extern u32 g_Auto_ActionRun_Timer;
extern u8  g_Auto_ActionRun_Step;
extern u32 g_Auto_ActionTimeOut_Time;
extern u8  g_Auto_ActionTimeOut_Flag;
extern u8  g_Auto_ActionError_Flag;
extern u8  g_Auto_WorkFinished_Flag;
extern u8  g_Material_Load_Disk_Finished;
extern u8  g_Material_UnLoad_Disk_Finished;

extern u32 g_LC_ActionRun_Timer;
extern u8  g_LC_ActionRun_Step;
extern u32 g_LC_ActionTimeOut_Time;
//extern u8  g_LC_ActionTimeOut_Flag;
extern u8  g_Start_ActionRun_Step;
extern u8  g_Reset_ActionRun_Step;

extern ActionStepList Action_Step_List[LISTNUM];
extern u8  Action_Step_List_Num;
extern u8  Action_Step_Run_Num;
extern u8  Action_Step_Confirm_Num;
extern u8  g_Material_Disk_Mode;
extern u8  g_Read_Load_LC_Position;
extern u8  g_Read_UnLoad_LC_Position;
extern u16 g_Load_LC_Key;
extern u32 g_Load_LC_Present_Position_X;		
extern u32 g_Load_LC_Present_Position_L;	
extern u8  g_Load_LC_Position_XKey;
extern u8  g_Load_LC_Position_LKey;

extern u16 g_UnLoad_LC_Key;
extern u32 g_UnLoad_LC_Present_Position_X;		
extern u32 g_UnLoad_LC_Present_Position_L;	
extern u8  g_UnLoad_LC_Position_XKey;
extern u8  g_UnLoad_LC_Position_LKey;

void AutoModeControl(void);
void AutoPauseOperate(void);



//#define LARGESTPROGRAMNUM 100	//编程最多指令数

//主要指令--order------//
#define   OR_BASICORDER      0x01  //基本指令
#define   OR_AXISORDER       0x02  //轴控指令
#define   OR_IOORDER         0x03  //IO指令

//子命令---key------//
#define   K_PROGRAMSTART           0x01	 //主程序开始
#define   K_PROGRAMEND             0x02	 //主程序结束
#define   K_DELAY                  0x03	 //延时
#define   K_LOADPROGRAM            0x04  //上料仓程序
#define   K_UNLOADPROGRAM          0x05  //卸料仓程序
#define   K_STARTACTION            0x06  //启动单次动作
#define   K_OAXISRESET             0x07  //堆叠完成动作

#define   K_ZAXISPOSITION          0x0C	 //垂直轴坐标
#define   K_OAXISPOSITION          0x0D	 //料仓轴坐标
#define   K_LOADPOSITION           0x0E	 //上料坐标
#define   K_UNLOADPOSITION         0x0F	 //卸料坐标
#define   K_XAXIS                  0x10	 //X轴
#define   K_ZAXIS                  0x11	 //Z轴
#define   K_LAXIS                  0x12	 //L轴
#define   K_OAXIS                  0x13	 //O轴


//---20161111------
#define   K_IOINSTRUCT_OUTPUT1     0x14	//输出指令1-置位Y0
#define   K_IOINSTRUCT_OUTPUT2     0x15	//输出指令2-复位Y0
#define   K_IOINSTRUCT_OUTPUT3     0x16	//输出指令3
#define   K_IOINSTRUCT_OUTPUT4     0x17	//输出指令4
#define   K_IOINSTRUCT_OUTPUT5     0x18	//输出指令5
#define   K_IOINSTRUCT_OUTPUT6     0x19	//输出指令6
#define   K_IOINSTRUCT_OUTPUT7     0x1A	//输出指令7
#define   K_IOINSTRUCT_OUTPUT8     0x1B	//输出指令8
#define   K_IOINSTRUCT_OUTPUT9     0x1C	//输出指令9
#define   K_IOINSTRUCT_OUTPUT10    0x1D	//输出指令10
#define   K_IOINSTRUCT_OUTPUT11    0x1E	//输出指令11
#define   K_IOINSTRUCT_OUTPUT12    0x1F	//输出指令12
#define   K_IOINSTRUCT_OUTPUT13    0x20	//输出指令13
#define   K_IOINSTRUCT_OUTPUT14    0x21	//输出指令14
#define   K_IOINSTRUCT_OUTPUT15    0x22	//输出指令15
#define   K_IOINSTRUCT_OUTPUT16    0x23	//输出指令16
#define   K_IOINSTRUCT_OUTPUT17    0x24	//输出指令17
#define   K_IOINSTRUCT_OUTPUT18    0x25	//输出指令18
#define   K_IOINSTRUCT_OUTPUT19    0x26	//输出指令19
#define   K_IOINSTRUCT_OUTPUT20    0x27	//输出指令20
#define   K_IOINSTRUCT_OUTPUT21    0x28	//输出指令21-置位Y10
#define   K_IOINSTRUCT_OUTPUT22    0x29	//输出指令22-复位Y10
#define   K_IOINSTRUCT_OUTPUT23    0x2A	//输出指令23
#define   K_IOINSTRUCT_OUTPUT24    0x2B	//输出指令24
#define   K_IOINSTRUCT_OUTPUT25    0x2C	//输出指令25
#define   K_IOINSTRUCT_OUTPUT26    0x2D	//输出指令26
#define   K_IOINSTRUCT_OUTPUT27    0x2E	//输出指令27
#define   K_IOINSTRUCT_OUTPUT28    0x2F	//输出指令28
#define   K_IOINSTRUCT_OUTPUT29    0x30	//输出指令29
#define   K_IOINSTRUCT_OUTPUT30    0x31	//输出指令30
#define   K_IOINSTRUCT_OUTPUT31    0x32	//输出指令31
#define   K_IOINSTRUCT_OUTPUT32    0x33	//输出指令32
#define   K_IOINSTRUCT_OUTPUT33    0x34	//输出指令33
#define   K_IOINSTRUCT_OUTPUT34    0x35	//输出指令34
#define   K_IOINSTRUCT_OUTPUT35    0x36	//输出指令35
#define   K_IOINSTRUCT_OUTPUT36    0x37	//输出指令36
#define   K_IOINSTRUCT_OUTPUT37    0x38	//输出指令37
#define   K_IOINSTRUCT_OUTPUT38    0x39	//输出指令38
#define   K_IOINSTRUCT_OUTPUT39    0x3A	//输出指令39-置位Y19
#define   K_IOINSTRUCT_OUTPUT40    0x3B	//输出指令40-复位Y19

//此段指令为单路和双路预留指令区间
#define   K_IOINSTRUCT_OUTPUT41    0x3C	//输出指令41
#define   K_IOINSTRUCT_OUTPUT42    0x3D	//输出指令42
#define   K_IOINSTRUCT_OUTPUT43    0x3E	//输出指令43
#define   K_IOINSTRUCT_OUTPUT44    0x3F	//输出指令44
//此段指令为单路和双路预留指令区间

#define   K_IOINSTRUCT_OUTPUT45    0x40	//输出指令45-置位Y20-复位Y21
#define   K_IOINSTRUCT_OUTPUT46    0x41	//输出指令46-复位Y20-置位Y21
#define   K_IOINSTRUCT_OUTPUT47    0x42	//输出指令47-置位Y22-复位Y23
#define   K_IOINSTRUCT_OUTPUT48    0x43	//输出指令48-复位Y22-置位Y23
#define   K_IOINSTRUCT_OUTPUT49    0x44	//输出指令39-置位Y24-复位Y25
#define   K_IOINSTRUCT_OUTPUT50    0x45	//输出指令40-复位Y24-置位Y25
#define   K_IOINSTRUCT_OUTPUT51    0x46	//输出指令41-置位Y26-复位Y27
#define   K_IOINSTRUCT_OUTPUT52    0x47	//输出指令42-复位Y26-置位Y27
#define   K_IOINSTRUCT_OUTPUT53    0x48	//输出指令43-置位Y28-复位Y29
#define   K_IOINSTRUCT_OUTPUT54    0x49	//输出指令44-复位Y28-置位Y29
#define   K_IOINSTRUCT_OUTPUT55    0x4A	//输出指令45-置位Y30-复位Y31
#define   K_IOINSTRUCT_OUTPUT56    0x4B	//输出指令46-复位Y30-置位Y31
#define   K_IOINSTRUCT_OUTPUT57    0x4C	//输出指令47-置位Y32-复位Y33
#define   K_IOINSTRUCT_OUTPUT58    0x4D	//输出指令48-复位Y32-置位Y33
#define   K_IOINSTRUCT_OUTPUT59    0x4E	//输出指令49-置位Y34-复位Y35
#define   K_IOINSTRUCT_OUTPUT60    0x4F	//输出指令50-复位Y34-置位Y35


#define   K_IOINSTRUCT_INPUT1      0x50	//输入指令1
#define   K_IOINSTRUCT_INPUT2      0x51	//输入指令2
#define   K_IOINSTRUCT_INPUT3      0x52	//输入指令3
#define   K_IOINSTRUCT_INPUT4      0x53	//输入指令4
#define   K_IOINSTRUCT_INPUT5      0x54	//输入指令5
#define   K_IOINSTRUCT_INPUT6      0x55	//输入指令6
#define   K_IOINSTRUCT_INPUT7      0x56	//输入指令7
//#define   K_IOINSTRUCT_INPUT8      0x57	//输入指令8
//#define   K_IOINSTRUCT_INPUT9      0x58	//输入指令9
//#define   K_IOINSTRUCT_INPUT10     0x59	//输入指令10



#define   A_QuLiao								 0x03  //取料
#define   A_DianJiao							 0x01  //点胶
#define   A_DianZuan							 0x02  //点钻
#define   A_KongYi								 0x05  //空移
#define   A_Delay								 0x04  //延时

#endif

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/

