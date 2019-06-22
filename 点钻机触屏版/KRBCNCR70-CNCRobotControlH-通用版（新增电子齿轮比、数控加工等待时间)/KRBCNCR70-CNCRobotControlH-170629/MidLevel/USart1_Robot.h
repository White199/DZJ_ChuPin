/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : 
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 
* Description        : This is the ....
***************************************************************************************/

#ifndef __USART1_ROBOT_H_
#define __USART1_ROBOT_H_

/*-------控制器回复命令宏定义-MainBoard Reply-------------*/
#define M_ROBOT_ENABLE_A_ORIGIN  0xA0	//机械手使能和回原点
#define M_WORK_MODE              0xB0	//机械手工作模式
#define M_AUTO_RUN               0xC0	//机械手自动运行
#define M_FREE_PROGRAM_SEND      0xD0	//自由编程程序下发
#define M_FREE_PROGRAM_RECEIVE	 0xE0 //产品选择程序接收

//输出点回复
#define M_IO_DEBUG_OUTPUT1        0xA1	//IO调试-输出回复
#define M_IO_DEBUG_OUTPUT2        0xB1	//IO调试-输出回复
#define M_IO_DEBUG_OUTPUT3        0xC1	//IO调试-输出回复
#define M_MANUL_OPERATE			  0xD1	//手动调试回复
#define M_PARAMETER_ORDER		  0xE1	//参数命令回复

//监视命令回复
#define M_IO_DEBUG_LOCAL         0xE1	//IO调试-本地IO
#define M_IO_DEBUG_INPUT1        0xE2	//IO调试-输入1
#define M_IO_DEBUG_INPUT2        0xE3	//IO调试-输入2
#define M_IO_DEBUG_INPUT3        0xE4	//IO调试-输入3
#define M_IO_DEBUG_OUTPUT1_LCD   0xE5	//IO调试-输出1界面
#define M_IO_DEBUG_OUTPUT2_LCD   0xE6	//IO调试-输出2界面
#define M_IO_DEBUG_OUTPUT3_LCD   0xE7	//IO调试-输出3界面
#define M_ROBOT_ORIGINED		 0xE8	//机械手是否已回原点
#define M_ACTION_RESET_SCAN	     0xE9	//机械手复位查询
#define M_ALARM_CLEAR    	     0xEA	//清除当前报警标志位
#define M_ROBOT_STATUS      	 0xEB	//状态查询:机械手状态、机械手运行状态、报警信息
#define M_AUTO_PARAMETER         0xEC	//
#define M_X_AXSIS_POSITION	     0xED	//X轴坐标
#define M_Z_AXSIS_POSITION       0xEE	//Z轴坐标
#define M_ROBOT_PRE_STATUS       0xEF	//机械手运行状态
#define M_ROBOT_DEBUG_STATUS     0xF0	//机械手调试状态
#define M_L_AXSIS_POSITION       0xF1	//L轴坐标
#define M_O_AXSIS_POSITION       0xF2	//O轴坐标
#define M_A_AXSIS_POSITION       0xF3	//V轴坐标
#define M_B_AXSIS_POSITION       0xF4	//W轴坐标
#define M_KEY_RUN					       0xF5	//控制板按键启动
#define M_KEY_PAUSE					     0xF6	//控制板按键暂停
#define M_KEY_RESET					     0xF7	//控制板按键复位


//读取IIC回复
#define   M_READ_IIC_1	         0xF1	//
#define   M_READ_IIC_2	         0xF2	//
#define   M_READ_IIC_3	         0xF3	//
#define   M_READ_IIC_4	         0xF4	//
#define   M_READ_IIC_5	         0xF5	//
#define   M_READ_IIC_6	         0xF6	//
#define   M_READ_IIC_7	         0xF7	//



extern u8 LCD_RealTime_Count;
extern u8 LCD_RealTime_Watch;
extern u8 LCD_Position_Watch;
extern u8 Manual_Pulse_Count;
extern u8 Manual_Pulse_Watch;
extern u8 Read_IIC_Done;
extern u8 Read_IIC_Count ;
extern u8 Read_IIC_Flag ;
							    
void LCDRealTimeWatch(void);	    //实时监视参数
void USART1RecieveDataDecode(void);	//串口1指令解码函数
void USART1Acknowledge(void);		//串口1主板应答处理函数
u8   DataSyc(void);
void MainBoard_InitDone(void);
void Program_Receive(void);//程序接收函数

#endif

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/

