/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : ***.h
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 05/28/2015
* Description        : This is the ....
***************************************************************************************/

#ifndef __FREEPROGRAMME_H_
#define __FREEPROGRAMME_H_

#define LARGESTPROGRAMNUM 32	//编程最多指令数
#define LARGESTPOINTMNUM 104		//编程最多点数目


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
#define   K_OAXISRESET             0x07  //料仓动作复位

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
#define   K_IOINSTRUCT_OUTPUT23    0x2A	//输出指令23-置位Y11
#define   K_IOINSTRUCT_OUTPUT24    0x2B	//输出指令24-复位Y11
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
#define   K_IOINSTRUCT_OUTPUT49    0x44	//输出指令49-置位Y24-复位Y25
#define   K_IOINSTRUCT_OUTPUT50    0x45	//输出指令50-复位Y24-置位Y25
#define   K_IOINSTRUCT_OUTPUT51    0x46	//输出指令51-置位Y26-复位Y27
#define   K_IOINSTRUCT_OUTPUT52    0x47	//输出指令52-复位Y27-置位Y26
#define   K_IOINSTRUCT_OUTPUT53    0x48	//输出指令53-置位Y28-复位Y29
#define   K_IOINSTRUCT_OUTPUT54    0x49	//输出指令54-复位Y28-置位Y29
#define   K_IOINSTRUCT_OUTPUT55    0x4A	//输出指令55-置位Y30-复位Y31
#define   K_IOINSTRUCT_OUTPUT56    0x4B	//输出指令56-复位Y30-置位Y31
#define   K_IOINSTRUCT_OUTPUT57    0x4C	//输出指令57-置位Y32-复位Y33
#define   K_IOINSTRUCT_OUTPUT58    0x4D	//输出指令58-复位Y32-置位Y33
#define   K_IOINSTRUCT_OUTPUT59    0x4E	//输出指令59-置位Y34-复位Y35
#define   K_IOINSTRUCT_OUTPUT60    0x4F	//输出指令60-复位Y34-置位Y35


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

 
//自由编程
typedef struct 
{
	u8  Flag;  //标志位
	u8  List;  //程序编号
	u8  Order; //指令类型
	u8  Key;   //子命令
	u32 Value1;//参数1-坐标XYZOVW
	u32 Value2;//参数2-
	u32 Value3;//参数3-
	u32 Value4;//参数4-
	u32 Value5;//参数5-
	u32 IO_Group;//IO控制
	u8  LC_ID;//料仓选择
	u16 Delay_Time;//延时指令延时
}FreeProgram;

//extern u8 g_Free_Program_Num;		 //当前选择自由编程程序
//extern FreeProgram Current_Process_Program[LARGESTPROGRAMNUM];

extern u8 g_Refresh_Dispaly_List_Flag ;	 //刷新程序显示列表
//extern u8 gs_Current_Program_Num;
extern u8 gs_Current_Operate_List;
extern u8 ProgramCode_String[1001][4];
extern u8 Program_String[87][16];
extern u8 Parameter_Value[10];
extern u8 Parameter_String[24][10];
extern u8 g_Current_Program_Debug;			 //当前程序是否可以调试
extern u8 g_Program_Is_Debuging;		     //程序调试中,所有按键失效
extern u8 g_USART1_SendProgram_Flag;

void LCDFreeProgramProcess(void);
void LCDFreeProgramDisply(void);
void ValueToString(u32);
void ProcessProgram(u8,u8);
void Delete_Order(void);
void Search_Order(void);
void Modify_Order(void);

void Delete_Point(void);
void Search_Point(void);
void Choose_Point(void);

void Select_Product(void);
void New_Product(void);
void Delete_Product(void);

void LCDProductManagementDisply(void);
void ValueToNumber(u16 Temp_Value);



#endif

