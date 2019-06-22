/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __signalwatch_h_
#define __signalwatch_h_

/******************USART命令类型Command***************************/
#define WORK_MODE         0x0a	   //工作模式
#define AUTO_MODE         0x0b	   //自动模式
#define MANUAL_OPERATE	  0x0c	   //手动操作动作命令
#define PARAMETER_COMMAND 0x0d	   //参数命令
#define WATCH_COMMAND     0x0e	   //监视命令
#define READ_IIC		  0x0f	   //开机读取IIC命令
#define REAL_TIME_WATCH   0x10	   //实时监测命令
#define BACK_TO_ORIGIN    0x11	   //回原点命令
#define CANCLE_ALARM      0x12	   //取消报警
#define INITIALIZE_FINISH 0xbd     //初始化完成
#define ORIGIN_BACK		  0xbe     //回原点完成

/**-------0x0E-----监视命令--------------**/
#define   IO_DEBUG_LOCAL         0x01	//IO调试-本地IO
#define   IO_DEBUG_INPUT1        0x02	//IO调试-输入1
#define   IO_DEBUG_INPUT2        0x03	//IO调试-输入2
#define   IO_DEBUG_INPUT3        0x04	//IO调试-输入3
#define   IO_DEBUG_OUTPUT1_LCD   0x05	//IO调试-输出1界面
#define   IO_DEBUG_OUTPUT2_LCD   0x06	//IO调试-输出2界面
#define   IO_DEBUG_OUTPUT3_LCD   0x07	//IO调试-输出3界面
#define   ROBOT_ORIGINED		 0x08	//机械手是否已回原点
#define   ACTION_RESET_SCAN	     0x09	//复位扫描
#define   ALARM_CLEAR    	     0x0A	//消除报警
#define   ROBOT_STATUS      	 0x0B	//机械手状态
#define   AUTO_PARAMETER         0x0C	//自动运行参数-当前行-产量
#define   X_AXSIS_POSITION	     0x0D	//X轴坐标
#define   Z_AXSIS_POSITION       0x0E	//Z轴坐标
#define   ROBOT_PRE_STATUS       0x0F	//机械手运行状态
#define   ROBOT_DEBUG_STATUS     0x10	//机械手调试状态
#define   L_AXSIS_POSITION       0x11	//L轴坐标
#define   O_AXSIS_POSITION       0x12	//L轴坐标
#define   A_AXSIS_POSITION       0x13	//V轴坐标
#define   B_AXSIS_POSITION       0x14	//W轴坐标

/******************监视命令***************************/
#define IN_PAGE1		    0x01   //输入页一
#define IN_PAGE2			0x02   //输入页二
#define	IN_PAGE3			0x03   //输入页三
#define	OUT_PAGE1			0x04   //输出页一
#define	OUT_PAGE2			0x05   //输出页二
#define	MANUAL_PARAMETER	0x06   //手动参数
#define	PROCESS				0x07   //加工
#define	TIME_POINTS_OTHERS	0x08   //时间-点-其他


/**------------P_XX手持器与主机通信参数--------------**/
/**------------P_XX手持器与主机通信参数--------------**/
/**------------参数命令--------------**/
#define   P_ROBOT_ENABLE_A_ORIGIN  0x0A	//机械手使能和回原点
#define   P_WORK_MODE              0x0B	//机械手工作模式
#define   P_AUTO_RUN               0x0C	//机械手自动运行
#define   P_FREE_PROGRAM_SEND      0x0D	//自由编程程序下发
#define   P_WATCH_COMMAND          0x0E	//监视命令
#define   P_READ_IIC	             0x0F	//开机读IIC
#define   P_IO_DEBUG_OUTPUT1       0x1A	//IO调试-输出1
#define   P_IO_DEBUG_OUTPUT2       0x1B	//IO调试-输出2
#define   P_IO_DEBUG_OUTPUT3       0x1C	//IO调试-输出3
#define   P_MANUL_DEBUG            0x1D	//手动调试
#define   P_PARAMETER_ORDER        0x1E	//参数命令
#define   P_AUTO_RUN_LCD           0x1F	//机械手自动运行

/**-------0x0A-----使能和回原点--------------**/
#define   P_ROBOT_ORIGIN   	     0x01	//机械手回原点
#define   P_X_AXSIS_ORIGIN	     0x02	//X轴回原点
#define   P_Z_AXSIS_ORIGIN	     0x03	//Z轴回原点
#define   P_ORIGIN_SPEED	     0x04	//机械手回原点速度
#define   P_ROBOT_ENABLE	     0x05	//机械手是否使能

/**-------0x0B-----工作模式--------------**/
#define   P_AUTO_MODE              0x01	//自动模式
#define   P_FREE_PROGRAM           0x02	//自由编程
#define   P_IO_MODE                0x03	//IO调试
#define   P_MANUL_MODE             0x04	//手动调试

/**-------0x0C-----自动运行--------------**/
#define   P_CYCLE_MODE             0x01	//循环模式
#define   P_SINGLE_MODE            0x02	//单步模式
#define   P_ACTION_RUN             0x03	//启动
#define   P_ACTION_PAUSE           0x04	//暂停
#define   P_ACTION_STOP            0x05	//停止
#define   P_ACTION_PROGRAM         0x06	//选择运行的程序
#define   P_ACTION_RESET           0x07	//机械手复位
#define   P_ACTION_DEBUG           0x08	//机械手调试-自由编程的调试按钮
#define   P_ACTION_LORIGIN         0x09	//机械手L轴回零

/**-------0x0D-----自由编程--------------**/
#define   P_PROGRAM_START          0xE1	//程序开始
#define   PROGRAM_INFO             0xE2	//程序信息描述
#define   PROGRAM_CONT             0xE3	//程序内容
#define   PROGRAM_DELETE           0xE4	//程序删除
#define   PROGRAM_FROM_USB_START   0xE5	//USB程序拷贝发送开始
#define   PROGRAM_FROM_USB_END     0xE6	//USB程序拷贝发送结束
//#define   P_PROGRAM_END            0xED	//程序结束

#define   P_PROGRAM_RENAME			0xE7 //重命名
#define   P_PROGRAM_CHOOSE         0xE8	//产品选择
#define   P_PROGRAM_DELETE         0xE9	//产品删除
#define   P_PROGRAM_RENAME_COVER   0xEA //产品重名覆盖
#define   P_PROGRAM_NEW				0xEB	//产品新建
#define   P_PROGRAM_NEWNAME_COVER	0xEC //新建同名覆盖-确定
#define   P_PROGRAM_SEND			0xED //发送主控板程序给上面
/**-------0x0E-----监视命令--------------**/
#define   P_IO_DEBUG_LOCAL         0x01	//IO调试-本地IO
#define   P_IO_DEBUG_INPUT1        0x02	//IO调试-输入1
#define   P_IO_DEBUG_INPUT2        0x03	//IO调试-输入2
#define   P_IO_DEBUG_INPUT3        0x04	//IO调试-输入3
#define   P_IO_DEBUG_OUTPUT1_LCD   0x05	//IO调试-输出1界面
#define   P_IO_DEBUG_OUTPUT2_LCD   0x06	//IO调试-输出2界面
#define   P_IO_DEBUG_OUTPUT3_LCD   0x07	//IO调试-输出3界面
#define   P_ROBOT_ORIGINED		   0x08	//机械手是否已回原点
#define   P_X_AXSIS_ORIGINED	   0x09	//X轴是否已回原点
#define   P_Z_AXSIS_ORIGINED	   0x0A	//Z轴是否已回原点
#define   P_ALARM_STATUS      	   0x0B	//报警标志位
#define   P_PRE_PROGRAM_NUMBER     0x0C	//Z轴是否已回原点
#define   P_X_AXSIS_POSITION	   0x0D	//X轴坐标
#define   P_Z_AXSIS_POSITION       0x0E	//Z轴坐标

/**-------0x0F-----开机读IIC--------------**/
#define   P_READ_IIC_1	         0x01	//
#define   P_READ_IIC_2	         0x02	//
#define   P_READ_IIC_3	         0x03	//
#define   P_READ_IIC_4	         0x04	//
#define   P_READ_IIC_5	         0x05	//
#define   P_READ_IIC_6	         0x06	//


/**-------0x1A-----IO调试-输出1--- 0x01 Y0--0x0F Y14---------**/
#define   P_IODEBUG_OUTPUT1_1      0x01	//
#define   P_IODEBUG_OUTPUT1_2      0x02	//
#define   P_IODEBUG_OUTPUT1_3      0x03	//
#define   P_IODEBUG_OUTPUT1_4      0x04	//
#define   P_IODEBUG_OUTPUT1_5      0x05	//
#define   P_IODEBUG_OUTPUT1_6      0x06	//
#define   P_IODEBUG_OUTPUT1_7      0x07	//
#define   P_IODEBUG_OUTPUT1_8      0x08	//
#define   P_IODEBUG_OUTPUT1_9      0x09	//
#define   P_IODEBUG_OUTPUT1_10     0x0A	//
#define   P_IODEBUG_OUTPUT1_11     0x0B	//
#define   P_IODEBUG_OUTPUT1_12     0x0C	//
#define   P_IODEBUG_OUTPUT1_13     0x0D	//	 
#define   P_IODEBUG_OUTPUT1_14     0x0E	//
#define   P_IODEBUG_OUTPUT1_15     0x0F	//

/**-------0x1B-----IO调试-输出2--- 0X01 Y15---0X0F Y29--------**/
#define   P_IODEBUG_OUTPUT2_1      0x01	//
#define   P_IODEBUG_OUTPUT2_2      0x02	//
#define   P_IODEBUG_OUTPUT2_3      0x03	//
#define   P_IODEBUG_OUTPUT2_4      0x04	//
#define   P_IODEBUG_OUTPUT2_5      0x05	//
#define   P_IODEBUG_OUTPUT2_6      0x06	//
#define   P_IODEBUG_OUTPUT2_7      0x07	//
#define   P_IODEBUG_OUTPUT2_8      0x08	//
#define   P_IODEBUG_OUTPUT2_9      0x09	//
#define   P_IODEBUG_OUTPUT2_10     0x0A	//
#define   P_IODEBUG_OUTPUT2_11     0x0B	//
#define   P_IODEBUG_OUTPUT2_12     0x0C	//
#define   P_IODEBUG_OUTPUT2_13     0x0D	//	 
#define   P_IODEBUG_OUTPUT2_14     0x0E	//
#define   P_IODEBUG_OUTPUT2_15     0x0F	//

/**-------0x1C-----IO调试-输出2--- 0X01 Y30---0X0A Y39--------**/
#define   P_IODEBUG_OUTPUT3_1      0x01	//
#define   P_IODEBUG_OUTPUT3_2      0x02	//
#define   P_IODEBUG_OUTPUT3_3      0x03	//
#define   P_IODEBUG_OUTPUT3_4      0x04	//
#define   P_IODEBUG_OUTPUT3_5      0x05	//
#define   P_IODEBUG_OUTPUT3_6      0x06	//
#define   P_IODEBUG_OUTPUT3_7      0x07	//
#define   P_IODEBUG_OUTPUT3_8      0x08	//
#define   P_IODEBUG_OUTPUT3_9      0x09	//
#define   P_IODEBUG_OUTPUT3_10     0x0A	//

/**-------0x1B-----IO调试-输出2--------------**/
#define   P_MACHINE2_KAPA_OPEN     0x01	//机床1卡盘A开
#define   P_MACHINE2_KAPA_CLOSE    0x02	//机床1卡盘A关
#define   P_MACHINE2_CUIQI         0x03	//机床1吹气
#define   P_MACHINE2_ALLOW_START   0x04	//机床1允许启动
#define   P_MACHINE2_START_PROCESS 0x05	//机床1开始加工
#define   P_MACHINE2_DOOR_OPEN     0x06	//机床1顶门开
#define   P_MACHINE2_DOOR_CLOSE    0x07	//机床1顶门关
#define   P_MACHINE2_OBLIGATE1     0x08	//机床1预留1
#define   P_MACHINE2_OBLIGATE2     0x09	//机床1预留2
#define   P_MACHINE2_OBLIGATE3     0x0A	//机床1预留3

#define   P_ROLL_OVER_UPRIGHT      0x0B	//翻转台-正摆
#define   P_ROLL_OVER_REVERSE      0x0C	//翻转台-反摆
#define   P_ROLL_OVER_OPEN         0x0D	//翻转台-松开
#define   P_ROLL_OVER_CLOSE        0x0E	//翻转台-夹紧
#define   P_ROLL_OVER_OBLIGATE1    0x10	//翻转台-预留1

/**-------0x1C-----IO调试-输出3--------------**/
#define   P_CLAW_LOAD_OPEN         0x01	//料爪-上料爪松
#define   P_CLAW_LOAD_CLOSE        0x02	//料爪-上料爪紧
#define   P_CLAW_UNLOAD_OPEN       0x03	//料爪-卸料爪松
#define   P_CLAW_UNLOAD_CLOSE      0x04	//料爪-卸料爪紧
#define   P_CLAW_BAIGANG_UPRIGHT   0x05	//料爪-摆缸正摆
#define   P_CLAW_BAIGANG_RESET     0x06	//料爪-摆缸复位
#define   P_CLAW_YCYLINDER_STRETCH 0x07	//料爪-Y向气缸伸出
#define   P_CLAW_YCYLINDER_RETRACT 0x08	//料爪-Y向气缸缩回
#define   P_CLAW_BLOW              0x09	//料爪-手抓吹气
#define   P_CLAW_OBLIGATE1         0x0A	//料爪-预留1

#define   P_OTHERS_OILING_ENABLE   0x0B	//其他-润滑使能
#define   P_OTHERS_RUN_LIGHT       0x0C	//其他-运行灯
#define   P_OTHERS_WAIT_LIGHT      0x0D	//其他-待机灯
#define   P_OTHERS_ALARM_LIGHT     0x0E	//其他-报警灯
#define   P_OTHERS_OBLIGATE1       0x10	//其他-预留1	
	  
/**-------0x1D-----手动调试-----------------**/
#define	  P_XAXIS_MANUL_SPEED	   0x01	//X轴-手动速度值
#define   P_XAXIS_MOVE_LEFT        0x02	//X轴-左移
#define   P_XAXIS_MOVE_RIGHT       0x03	//X轴-右移
#define   P_XAXIS_STEP_STOP        0x04	//X轴-停止
#define   P_XAXIS_SAVE_POINT       0x05	//X轴-保存
#define	  P_ZAXIS_MANUL_SPEED	   0x06	//Z轴-手动速度值
#define   P_ZAXIS_MOVE_LEFT        0x07	//Z轴-左移
#define   P_ZAXIS_MOVE_RIGHT       0x08	//Z轴-右移
#define   P_ZAXIS_STEO_STOP        0x09	//Z轴-停止
#define   P_ZAXIS_SAVE_POINT       0x0A	//Z轴-保存
#define   P_XZAXIS_SAVE_POINT      0x0B	//XZ轴-保存
#define   P_XZAXIS_DELETE_POINT   0x0C	//XZ轴-删除
#define	  P_XAXIS_STEP_MM	       0x0D	//X轴-寸动倍率
#define	  P_ZAXIS_STEP_MM	       0x0E	//Z轴-寸动倍率
#define	  P_XAXIS_BACK_ORIGIN	   0x0F	//X轴-回零
#define	  P_ZAXIS_BACK_ORIGIN	   0x10	//Z轴-回零
#define   P_MANUL_FINE_ADJUST	   0x11	//微调命令

#define	  P_LAXIS_MANUL_SPEED	   0x12	//L轴-手动速度值
#define   P_LAXIS_MOVE_LEFT        0x13	//L轴-左移
#define   P_LAXIS_MOVE_RIGHT       0x14	//L轴-右移
#define   P_LAXIS_STEP_STOP        0x15	//L轴-寸动停止
#define	  P_LAXIS_STEP_MM		   0x16	//L轴-寸动距离mm为单位
#define   P_LAXIS_SAVE_POINT       0x17	//L轴-保存
#define	  P_LAXIS_BACK_ORIGIN	   0x18	//L轴-回零

#define	  P_OAXIS_MANUL_SPEED	   0x19	//O轴-手动速度值
#define   P_OAXIS_MOVE_LEFT        0x1A	//O轴-左移
#define   P_OAXIS_MOVE_RIGHT       0x1B	//O轴-右移
#define   P_OAXIS_STEP_STOP        0x1C	//O轴-寸动停止
#define	  P_OAXIS_STEP_MM		   0x1D	//O轴-寸动距离mm为单位
#define   P_OAXIS_SAVE_POINT       0x1E	//O轴-保存
#define	  P_OAXIS_BACK_ORIGIN	   0x1F	//O轴-回零
#define   P_LOAXIS_DELETE_POINT   0x20	//LO轴-删除


#define	  P_VAXIS_MANUL_SPEED	   0x21	//V轴-手动速度值
#define   P_VAXIS_MOVE_LEFT        0x22	//V轴-左移
#define   P_VAXIS_MOVE_RIGHT       0x23	//V轴-右移
#define   P_VAXIS_STEP_STOP        0x24	//V轴-寸动停止
#define	  P_VAXIS_STEP_MM		   0x25	//V轴-寸动距离mm为单位
#define   P_VAXIS_SAVE_POINT       0x26	//V轴-保存
#define	  P_VAXIS_BACK_ORIGIN	   0x27	//V轴-回零

#define	  P_WAXIS_MANUL_SPEED	   0x28	//W轴-手动速度值
#define   P_WAXIS_MOVE_LEFT        0x29	//W轴-左移
#define   P_WAXIS_MOVE_RIGHT       0x2A	//W轴-右移
#define   P_WAXIS_STEP_STOP        0x2B	//W轴-寸动停止
#define	  P_WAXIS_STEP_MM		   0x2C	//W轴-寸动距离mm为单位
#define   P_WAXIS_SAVE_POINT       0x2D	//W轴-保存
#define	  P_WAXIS_BACK_ORIGIN	   0x2E	//W轴-回零

#define   E_GEARRATIO_SET			0x2F//电子齿轮比设置
#define   XYAXIS_MANUL_SPEED    	0x30//示教界面XY联动速度设置
#define   ZOAXIS_MANUL_SPEED    	0x31//示教界面ZO联动速度设置
#define   ABAXIS_MANUL_SPEED    	0x32//示教界面AB联动速度设置
#define   XYAXIS_STEP_MM			0x33//示教界面XY寸动速度设置
#define   ZOAXIS_STEP_MM			0x34//示教界面ZO寸动速度设置
#define   ABAXIS_STEP_MM			0x35//示教界面AB寸动速度设置
#define   P_ACTION_SPEEDADJ     0x36//总速度调节20181120-lin
/**-------0x1E-----参数命令-----------------**/
#define	  P_PARAMETER_SOFT_LIMIT	   0x01	//软限位
#define   P_PARAMETER_SAFE_AREA        0x02	//安全区
#define   P_PARAMETER_FUCTION_SET      0x03	//功能参数


extern u8 Watch_Data;				   //实时监视相关数据

extern void WatchFunction(void);
extern void OrderDecoding(void);


#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

