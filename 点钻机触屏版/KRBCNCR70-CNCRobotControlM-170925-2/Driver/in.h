//******************************************************//
//*****************  输入信号检测  *********************//
//**   输入信号全部JHR210的输入，后面标注为JHR105     **//
//**   的输入信号为小型号机床使用的输入               **//
//******************************************************//
#ifndef __in_h_
#define __in_h_

#define  DETECT_REVERSING_UP              0       //换向升检测
#define  DETECT_REVERSING_DOWN            1       //换向降检测
#define  DETECT_REVERSING_CLAMP           2       //换向夹紧检测
#define  DETECT_REVERSING_RELEASE         3       //换向放松检测
#define  DETECT_REVERSING_TURN_FRONT      4       //换向前翻检测
#define  DETECT_REVERSING_TURN_BACK       5       //换向后翻检测

#define  DETECT_TRASPOSITION1             6       //回转位1检测				   JHR105
#define  DETECT_TRASPOSITION2             7       //回转位2检测				   JHR105
#define  DETECT_CLAW1_CLAMP               8       //料抓1紧检测				   JHR105
#define  DETECT_CLAW1_RELEASE             9       //料抓1松检测				   JHR105
#define  DETECT_CLAW2_CLAMP               10      //料抓2紧检测				   JHR105
#define  DETECT_CLAW2_RELEASE             11      //料抓2松检测				   JHR105
#define  DETECT_X_ORIGIN				  12	  //送料X轴回零				   JHR105
#define  DETECT_Z_ORIGIN				  13	  //送料Z轴回零				   JHR105
#define  DETECT_MACHINE_BREAKDOWN		  14	  //机床故障				   JHR105
#define  DETECT_ROBOT_START				  15	  //机器人启动				   JHR105
#define  DETECT_ROBOT_STOP				  16	  //机器人停止				   JHR105
#define  DETECT_MACHINE1_DOOR_BUTTON	  17	  //1号机移门按钮			   JHR105
#define	 DETECT_MACHINE1_HOLDER_BUTTON	  18	  //1号机夹具按钮			   JHR105
#define	 DETECT_MACHINE1_NEED_MATERIAL	  19	  //1号机要料按钮			   JHR105
#define  DETECT_MACHINE1_HOLDER_CLAMP     20      //1号机夹具紧检测			   JHR105
#define  DETECT_MACHINE1_HOLDER_RELEASE   21      //1号机夹具松检测			   JHR105
#define  DETECT_MACHINE1_DOOR_OPEN        22      //1号机移门开检测			   JHR105
#define  DETECT_MACHINE1_DOOR_CLOSE       23      //1号机移门关检测			   JHR105

#define  DETECT_MACHINE2_DOOR_BUTTON	  24	  //1号机移门按钮
#define	 DETECT_MACHINE2_HOLDER_BUTTON	  25	  //1号机夹具按钮
#define	 DETECT_MACHINE2_NEED_MATERIAL	  26	  //1号机要料按钮
#define  DETECT_MACHINE2_HOLDER_CLAMP     27      //2号机夹具紧检测
#define  DETECT_MACHINE2_HOLDER_RELEASE   28      //2号机夹具松检测
#define  DETECT_MACHINE2_DOOR_OPEN        29      //2号机移门开检测
#define  DETECT_MACHINE2_DOOR_CLOSE       30      //2号机移门关检测

/*------------料仓输入信号----------------*/
#define	 ALLOW_LOAD_MATERIAL			  31	  //允许取料				  JHR105
#define	 ALLOW_UNLOAD_MARERIAL			  32	  //允许卸料				  JHR105
#define	 LACK_MATERIAL_ALARM			  33	  //缺料报警				  JHR105
#define	 FULL_MATERIAL_ALARM			  34	  //满料报警				  JHR105
#define  EMERGENCY_STOP_ALARM	          35	  //急停报警				  JHR105

/*------------伺服器极限报警输入信号----------------*/
#define	 X_LEFT_LIMIT			          36	  //X左极限				  JHR105
#define	 X_RIGHT_LIMIT			          37	  //X右极限				  JHR105
#define	 Z_LEFT_LIMIT			          38	  //Z左极限				  JHR105
#define	 Z_RIGHT_LIMIT			          39	  //Z右极限				  JHR105
		
#define  I_DETECT_B_ORIGIN				  16	  //送料W轴回零				   JR
#define  I_DETECT_A_ORIGIN				  17	  //送料V轴回零				   JR
#define  I_DETECT_O_ORIGIN				  18	  //送料O轴回零				   JR
#define  I_DETECT_L_ORIGIN				  19	  //送料L轴回零				   JR
#define  I_DETECT_X_ORIGIN				  20	  //送料X轴回零				   JR
#define  I_DETECT_Z_ORIGIN				  21	  //送料Z轴回零				   JR

extern u8 ReadInput(u8 IO_Num);

extern u8 ReadEmergencyStop(void);
extern u8 ReadXAxsisError(void) ;
extern u8 ReadZAxsisError(void);
extern u8 ReadAxsisError(u8) ;
extern u8 ReadIOPort(GPIO_TypeDef* GPIOx, u16 PortNum, u8 PortPreviousStatus);

extern u8 Local_IO_Status[2];
extern u8 Input_Detect_Status[6];

#endif
/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/
