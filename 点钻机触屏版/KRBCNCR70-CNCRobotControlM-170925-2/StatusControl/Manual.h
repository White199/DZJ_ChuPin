/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __manual_h_
#define __manual_h_


#define  M_REVERSING_UP              0x01     //换向升
#define  M_REVERSING_DOWN            0x02     //换向降
#define  M_REVERSING_CLAMP           0x03     //换向夹紧
#define  M_REVERSING_RELEASE         0x04     //换向放松
#define  M_REVERSING_TURN_FRONT      0x05     //换向前翻
#define  M_REVERSING_TURN_BACK       0x06     //换向后翻
#define  M_TRASPOSITION1             0x07     //转位1
#define  M_TRASPOSITION2             0x08     //转位2
#define  M_CLAW1_CLAMP               0x09     //料抓1紧
#define  M_CLAW1_RELEASE             0x0a     //料抓1松
#define  M_CLAW2_CLAMP               0x0b     //料抓2紧
#define  M_CLAW2_RELEASE             0x0c     //料抓2松
#define  M_MACHINE1_HOLDER_CLAMP     0x0d     //1号机夹具紧
#define  M_MACHINE1_HOLDER_RELEASE   0x0e     //1号机夹具松
#define  M_MACHINE1_DOOR_OPEN        0x0f     //1号机移门开
#define  M_MACHINE1_DOOR_CLOSE       0x10     //1号机移门关
#define  M_MACHINE1_SEND_GAS         0x11     //1号机吹气
#define  M_MACHINE1_SEND_MATERIAL    0x12     //1号机送料
#define  M_MACHINE2_HOLDER_CLAMP     0x13     //2号机夹具紧
#define  M_MACHINE2_HOLDER_RELEASE   0x14     //2号机夹具松
#define  M_MACHINE2_DOOR_OPEN        0x15     //2号机移门开
#define  M_MACHINE2_DOOR_CLOSE       0x16     //2号机移门关
#define  M_MACHINE2_SEND_GAS         0x17     //2号机吹气
#define  M_MACHINE2_SEND_MATERIAL    0x18     //2号机送料
#define  M_LOAD_MARERIAL_FINISH      0x19     //取料结束信号
#define  M_UNLOAD_MATERIAL_FINISH    0x1a     //卸料结束信号
#define  M_X_CHOSEN                  0x1b     //x轴选中
#define  M_Z_CHOSEN                  0x1c     //z轴选中
#define  M_JOG_MODE                  0x1d     //寸动模式
#define  M_LINKED_MODE               0x1e     //连动模式
#define  M_POSITIVE_MOVE             0x1f     //正向移动
#define  M_NEGATIVE_MOVE             0x20    //反向移动

									   
extern u8 X_Axsis_Chosen ;			  //X轴选中标志位
extern u8 Z_Axsis_Chosen ;			  //Z轴选中标志位

extern u8 Manual_Mode_Enable ;		  //手动模式使能标志位
extern u8 Jog_Move_Enable ;			  //寸动运行标志位
extern u8 Jog_Mode_Enable ;			  //寸动模式使能标志位
extern u8 Linked_Mode_Enable;		  //连动模式使能标志位
extern u8 Linked_Move_Enable;		  //连动运行标志位
extern u8 g_Link_Move_Delay;
extern u8 g_Link_Move_Count;

extern u8  Send_Material_Finish_Flag;
extern u32 Send_Material_Finish_Time ;       //送料定时参数
extern u32 Load_Material_Finish_Time ;		//取料定时参数
extern u8  Load_Material_Finish_Flag	;		//取料结束标志位
extern u32 Unload_Material_Finish_Time ;		//卸料定时参数
extern u8  Unload_Material_Finish_Flag ;		//卸料结束标志位

extern u8 Manual_Mode_Error_Flag;	  //手动操作模式错误标志位
extern u32 Jog_Pulse_Count ;		  //寸动模式的每个动作的脉冲数
extern u32 Jog_Pulse_Count_Init ;
extern u32 Linked_Pulse_Count ;		  //连动模式的每次发送脉冲数
extern u32 Linked_Pulse;
extern u8  Test_Key_Flag;
extern u8  XAxis_Manul_Speed;			  //X轴手动速度
extern u8  ZAxis_Manul_Speed;			  //Z轴手动速度
extern u8  LAxis_Manul_Speed;			  //Z轴手动速度
extern u8  OAxis_Manul_Speed;			  //Z轴手动速度
extern u8  AAxis_Manul_Speed;			  //Z轴手动速度
extern u8  BAxis_Manul_Speed;			  //Z轴手动速度
extern u8  XAxis_Step_Distance;		  //X轴寸动距离5-100,默认50mm
extern u8  ZAxis_Step_Distance;		  //Z轴寸动距离5-100,默认50mm
extern u8  LAxis_Step_Distance;		  //Z轴寸动距离5-100,默认50mm
extern u8  OAxis_Step_Distance;		  //Z轴寸动距离5-100,默认50mm
extern u8  g_Manul_Fine_Adjust;		  //手动微调

void IODebugOutput1(void);
void IODebugOutput2(void);
void IODebugOutput3(void);
extern void ManulDebug(void);		  //手动调试
extern void ManualMode(void);		  //手动模式
extern void ManualJogRunnig(void);	  //手动操作点动模式
extern void ManualLinkedRunning(void);//手动操作连动模式
//---
extern u32 Real_Time_Position[6];	  //�
extern vu32 Real_Time_Position_1[6];


void Order_Package(void);//指令封装

#endif

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/

