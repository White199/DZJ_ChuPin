/******************** (C) COPYRIGHT 2011 zjut嵌入式开发组  ********************
* File Name          : out.h
* Author             :  
* Version            :  
* Date               :  
* Description        : This file 
*******************************************************************************/
#ifndef __out_h_
#define __out_h_

#define  OUT_REVERSING_UP                   0      //换向升
#define  OUT_REVERSING_DOWN                 1      //换向降
#define  OUT_REVERSING_CLAMP                2      //换向夹紧
#define  OUT_REVERSING_RELEASE              3      //换向放松
#define  OUT_REVERSING_TURN_FRONT           4      //换向前翻
#define  OUT_REVERSING_TURN_BACK            5      //换向后翻

#define  OUT_TRASPOSITION_1                 6      //转位1				  JHR105
#define  OUT_TRASPOSITION_2                 7      //转位2				  JHR105
#define  OUT_CLAW1_CLAMP                    8      //料抓1紧			  JHR105
#define  OUT_CLAW1_RELEASE                  9      //料抓1松			  JHR105
#define  OUT_CLAW2_CLAMP                    10     //料抓2紧			  JHR105
#define  OUT_CLAW2_RELEASE                  11     //料抓2松			  JHR105
#define  OUT_BREAKOUT_OUTLIGHT		        12	   //故障输出，点报警灯	  JHR105
#define  OUT_MACHINE1_HOLDER_CLAMP          13     //1号机夹具紧		  JHR105
#define  OUT_MACHINE1_HOLDER_RELEASE        14     //1号机夹具松		  JHR105
#define  OUT_MACHINE1_DOOR_OPEN             15     //1号机移门开		  JHR105
#define  OUT_MACHINE1_DOOR_CLOSE            16     //1号机移门关		  JHR105
#define  OUT_MACHINE1_SEND_GAS              17     //1号机吹气			  JHR105
#define  OUT_MACHINE1_SEND_MATERIAL_FINISH  18     //1号机送料完成		  JHR105

#define  OUT_MACHINE2_HOLDER_CLAMP          19     //2号机夹具紧
#define  OUT_MACHINE2_HOLDER_RELEASE        20     //2号机夹具松
#define  OUT_MACHINE2_DOOR_OPEN             21     //2号机移门开
#define  OUT_MACHINE2_DOOR_CLOSE            22     //2号机移门关
#define  OUT_MACHINE2_SEND_GAS              23     //2号机吹气
#define  OUT_MACHINE2_SEND_MATERIAL_FINISH  24     //2号机送料完成

/*------------料仓输出信号----------------*/
#define  OUT_LOAD_MATERIAL_FINISH           26     //取料结束信号
#define  OUT_UNLOAD_MATERIAL_FINISH         27     //卸料结束信号


/**-------0x1A-----IO调试-输出1--------------**/
#define   O_IODEBUG_OUTPUT_0       0	//
#define   O_IODEBUG_OUTPUT_1       1	//
#define   O_IODEBUG_OUTPUT_2       2	//
#define   O_IODEBUG_OUTPUT_3       3	//
#define   O_IODEBUG_OUTPUT_4       4	//
#define   O_IODEBUG_OUTPUT_5       5	//
#define   O_IODEBUG_OUTPUT_6       6	//
#define   O_IODEBUG_OUTPUT_7       7	//
#define   O_IODEBUG_OUTPUT_8       8	//
#define   O_IODEBUG_OUTPUT_9       9	//
#define   O_IODEBUG_OUTPUT_10      10	//
#define   O_IODEBUG_OUTPUT_11      11	//
#define   O_IODEBUG_OUTPUT_12      12	//
#define   O_IODEBUG_OUTPUT_13      13	//
#define   O_IODEBUG_OUTPUT_14      14	//
#define   O_IODEBUG_OUTPUT_15      15	//
#define   O_IODEBUG_OUTPUT_16      16	//
#define   O_IODEBUG_OUTPUT_17      17	//
#define   O_IODEBUG_OUTPUT_18      18	//
#define   O_IODEBUG_OUTPUT_19      19	//
#define   O_IODEBUG_OUTPUT_20      20	//
#define   O_IODEBUG_OUTPUT_21      21	//
#define   O_IODEBUG_OUTPUT_22      22	//
#define   O_IODEBUG_OUTPUT_23      23	//
#define   O_IODEBUG_OUTPUT_24      24	//
#define   O_IODEBUG_OUTPUT_25      25	//
#define   O_IODEBUG_OUTPUT_26      26	//
#define   O_IODEBUG_OUTPUT_27      27	//
#define   O_IODEBUG_OUTPUT_28      28	//
#define   O_IODEBUG_OUTPUT_29      29	//
#define   O_IODEBUG_OUTPUT_30      30	//
#define   O_IODEBUG_OUTPUT_31      31	//
#define   O_IODEBUG_OUTPUT_32      32	//
#define   O_IODEBUG_OUTPUT_33      33	//
#define   O_IODEBUG_OUTPUT_34      34	//
#define   O_IODEBUG_OUTPUT_35      35	//
#define   O_IODEBUG_OUTPUT_36      36	//
#define   O_IODEBUG_OUTPUT_37      37	//
#define   O_IODEBUG_OUTPUT_38      38	//
#define   O_IODEBUG_OUTPUT_39      39	//
/**-------0x1A-----IO调试-输出1--------------**/
#define   O_STORAGE_LOAD_FINISH    0	//料仓-上料完成
#define   O_STORAGE_UNLOAD_FINISH  1	//料仓-卸料完成
#define   O_STORAGE_LOAD_ALARM     2	//料仓-上料仓报警
#define   O_STORAGE_UNLOAD_ALARM   3	//料仓-卸料仓报警

#define   O_MACHINE1_KAPA_OPEN     4	//机床1卡盘A开
#define   O_MACHINE1_KAPA_CLOSE    5	//机床1卡盘A关
#define   O_MACHINE1_CUIQI         6	//机床1吹气
#define   O_MACHINE1_ALLOW_START   7	//机床1允许启动
#define   O_MACHINE1_START_PROCESS 8	//机床1开始加工
#define   O_MACHINE1_DOOR_OPEN     9	//机床1顶门开
#define   O_MACHINE1_DOOR_CLOSE    10	//机床1顶门关
#define   O_MACHINE1_OBLIGATE1     11	//机床1预留1

/**-------0x1B-----IO调试-输出2--------------**/
#define   O_MACHINE2_KAPA_OPEN     12	//机床2卡盘A开
#define   O_MACHINE2_KAPA_CLOSE    13	//机床2卡盘A关
#define   O_MACHINE2_CUIQI         14	//机床2吹气
#define   O_MACHINE2_ALLOW_START   15	//机床2允许启动
#define   O_MACHINE2_START_PROCESS 16	//机床2开始加工
#define   O_MACHINE2_DOOR_OPEN     17	//机床2顶门开
#define   O_MACHINE2_DOOR_CLOSE    18	//机床2顶门关
#define   O_MACHINE2_OBLIGATE1     19	//机床2预留1

#define   O_ROLL_OVER_UPRIGHT      20	//翻转台-正摆
#define   O_ROLL_OVER_REVERSE      21	//翻转台-反摆
#define   O_ROLL_OVER_OPEN         22   //翻转台-松开
#define   O_ROLL_OVER_CLOSE        23	//翻转台-夹紧


/**-------0x1C-----IO调试-输出3--------------**/
#define   O_CLAW_LOAD_OPEN         24	//料爪-上料爪松
#define   O_CLAW_LOAD_CLOSE        25	//料爪-上料爪紧
#define   O_CLAW_UNLOAD_OPEN       26	//料爪-卸料爪松
#define   O_CLAW_UNLOAD_CLOSE      27	//料爪-卸料爪紧
#define   O_CLAW_BAIGANG_UPRIGHT   28	//料爪-摆缸正摆
#define   O_CLAW_BAIGANG_RESET     29	//料爪-摆缸复位
#define   O_CLAW_YCYLINDER_STRETCH 30	//料爪-Y向气缸伸出
#define   O_CLAW_YCYLINDER_RETRACT 31	//料爪-Y向气缸缩回
#define   O_CLAW_BLOW              32	//料爪-手抓吹气

#define   O_OTHERS_OILING_ENABLE   33	//其他-润滑使能
#define   O_OTHERS_RUN_LIGHT       34	//其他-运行灯
#define   O_OTHERS_WAIT_LIGHT      35	//其他-待机灯
#define   O_OTHERS_ALARM_LIGHT     36	//其他-报警灯



/*------------电机方向控制信号----------------*/
#define X_AXSIS_DIRECTION           44	   //
#define Z_AXSIS_DIRECTION			42	   //
#define L_AXSIS_DIRECTION           43	   //
#define O_AXSIS_DIRECTION			41	   //
#define A_AXSIS_DIRECTION           45	   //
#define B_AXSIS_DIRECTION		    46	   //

extern void SetOutput(u8 IO_Num);
extern void ResetOutput(u8 IO_Num);
extern u8 Output_Status[5];



#endif

/******************* (C) COPYRIGHT 2011 zjut嵌入式开发组 *****END OF FILE****/
