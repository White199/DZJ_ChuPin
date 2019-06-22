/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __manual_h_
#define __manual_h_


#define  M_REVERSING_UP              0x01     //»»ÏòÉı
#define  M_REVERSING_DOWN            0x02     //»»Ïò½µ
#define  M_REVERSING_CLAMP           0x03     //»»Ïò¼Ğ½ô
#define  M_REVERSING_RELEASE         0x04     //»»Ïò·ÅËÉ
#define  M_REVERSING_TURN_FRONT      0x05     //»»ÏòÇ°·­
#define  M_REVERSING_TURN_BACK       0x06     //»»Ïòºó·­
#define  M_TRASPOSITION1             0x07     //×ªÎ»1
#define  M_TRASPOSITION2             0x08     //×ªÎ»2
#define  M_CLAW1_CLAMP               0x09     //ÁÏ×¥1½ô
#define  M_CLAW1_RELEASE             0x0a     //ÁÏ×¥1ËÉ
#define  M_CLAW2_CLAMP               0x0b     //ÁÏ×¥2½ô
#define  M_CLAW2_RELEASE             0x0c     //ÁÏ×¥2ËÉ
#define  M_MACHINE1_HOLDER_CLAMP     0x0d     //1ºÅ»ú¼Ğ¾ß½ô
#define  M_MACHINE1_HOLDER_RELEASE   0x0e     //1ºÅ»ú¼Ğ¾ßËÉ
#define  M_MACHINE1_DOOR_OPEN        0x0f     //1ºÅ»úÒÆÃÅ¿ª
#define  M_MACHINE1_DOOR_CLOSE       0x10     //1ºÅ»úÒÆÃÅ¹Ø
#define  M_MACHINE1_SEND_GAS         0x11     //1ºÅ»ú´µÆø
#define  M_MACHINE1_SEND_MATERIAL    0x12     //1ºÅ»úËÍÁÏ
#define  M_MACHINE2_HOLDER_CLAMP     0x13     //2ºÅ»ú¼Ğ¾ß½ô
#define  M_MACHINE2_HOLDER_RELEASE   0x14     //2ºÅ»ú¼Ğ¾ßËÉ
#define  M_MACHINE2_DOOR_OPEN        0x15     //2ºÅ»úÒÆÃÅ¿ª
#define  M_MACHINE2_DOOR_CLOSE       0x16     //2ºÅ»úÒÆÃÅ¹Ø
#define  M_MACHINE2_SEND_GAS         0x17     //2ºÅ»ú´µÆø
#define  M_MACHINE2_SEND_MATERIAL    0x18     //2ºÅ»úËÍÁÏ
#define  M_LOAD_MARERIAL_FINISH      0x19     //È¡ÁÏ½áÊøĞÅºÅ
#define  M_UNLOAD_MATERIAL_FINISH    0x1a     //Ğ¶ÁÏ½áÊøĞÅºÅ
#define  M_X_CHOSEN                  0x1b     //xÖáÑ¡ÖĞ
#define  M_Z_CHOSEN                  0x1c     //zÖáÑ¡ÖĞ
#define  M_JOG_MODE                  0x1d     //´ç¶¯Ä£Ê½
#define  M_LINKED_MODE               0x1e     //Á¬¶¯Ä£Ê½
#define  M_POSITIVE_MOVE             0x1f     //ÕıÏòÒÆ¶¯
#define  M_NEGATIVE_MOVE             0x20    //·´ÏòÒÆ¶¯

									   
extern u8 X_Axsis_Chosen ;			  //XÖáÑ¡ÖĞ±êÖ¾Î»
extern u8 Z_Axsis_Chosen ;			  //ZÖáÑ¡ÖĞ±êÖ¾Î»

extern u8 Manual_Mode_Enable ;		  //ÊÖ¶¯Ä£Ê½Ê¹ÄÜ±êÖ¾Î»
extern u8 Jog_Move_Enable ;			  //´ç¶¯ÔËĞĞ±êÖ¾Î»
extern u8 Jog_Mode_Enable ;			  //´ç¶¯Ä£Ê½Ê¹ÄÜ±êÖ¾Î»
extern u8 Linked_Mode_Enable;		  //Á¬¶¯Ä£Ê½Ê¹ÄÜ±êÖ¾Î»
extern u8 Linked_Move_Enable;		  //Á¬¶¯ÔËĞĞ±êÖ¾Î»
extern u8 g_Link_Move_Delay;
extern u8 g_Link_Move_Count;

extern u8  Send_Material_Finish_Flag;
extern u32 Send_Material_Finish_Time ;       //ËÍÁÏ¶¨Ê±²ÎÊı
extern u32 Load_Material_Finish_Time ;		//È¡ÁÏ¶¨Ê±²ÎÊı
extern u8  Load_Material_Finish_Flag	;		//È¡ÁÏ½áÊø±êÖ¾Î»
extern u32 Unload_Material_Finish_Time ;		//Ğ¶ÁÏ¶¨Ê±²ÎÊı
extern u8  Unload_Material_Finish_Flag ;		//Ğ¶ÁÏ½áÊø±êÖ¾Î»

extern u8 Manual_Mode_Error_Flag;	  //ÊÖ¶¯²Ù×÷Ä£Ê½´íÎó±êÖ¾Î»
extern u32 Jog_Pulse_Count ;		  //´ç¶¯Ä£Ê½µÄÃ¿¸ö¶¯×÷µÄÂö³åÊı
extern u32 Jog_Pulse_Count_Init ;
extern u32 Linked_Pulse_Count ;		  //Á¬¶¯Ä£Ê½µÄÃ¿´Î·¢ËÍÂö³åÊı
extern u32 Linked_Pulse;
extern u8  Test_Key_Flag;
extern u8  XAxis_Manul_Speed;			  //XÖáÊÖ¶¯ËÙ¶È
extern u8  ZAxis_Manul_Speed;			  //ZÖáÊÖ¶¯ËÙ¶È
extern u8  LAxis_Manul_Speed;			  //ZÖáÊÖ¶¯ËÙ¶È
extern u8  OAxis_Manul_Speed;			  //ZÖáÊÖ¶¯ËÙ¶È
extern u8  AAxis_Manul_Speed;			  //ZÖáÊÖ¶¯ËÙ¶È
extern u8  BAxis_Manul_Speed;			  //ZÖáÊÖ¶¯ËÙ¶È
extern u8  XAxis_Step_Distance;		  //XÖá´ç¶¯¾àÀë5-100,Ä¬ÈÏ50mm
extern u8  ZAxis_Step_Distance;		  //ZÖá´ç¶¯¾àÀë5-100,Ä¬ÈÏ50mm
extern u8  LAxis_Step_Distance;		  //ZÖá´ç¶¯¾àÀë5-100,Ä¬ÈÏ50mm
extern u8  OAxis_Step_Distance;		  //ZÖá´ç¶¯¾àÀë5-100,Ä¬ÈÏ50mm
extern u8  g_Manul_Fine_Adjust;		  //ÊÖ¶¯Î¢µ÷

void IODebugOutput1(void);
void IODebugOutput2(void);
void IODebugOutput3(void);
extern void ManulDebug(void);		  //ÊÖ¶¯µ÷ÊÔ
extern void ManualMode(void);		  //ÊÖ¶¯Ä£Ê½
extern void ManualJogRunnig(void);	  //ÊÖ¶¯²Ù×÷µã¶¯Ä£Ê½
extern void ManualLinkedRunning(void);//ÊÖ¶¯²Ù×÷Á¬¶¯Ä£Ê½
//---
extern u32 Real_Time_Position[6];	  //÷
extern vu32 Real_Time_Position_1[6];


void Order_Package(void);//Ö¸Áî·â×°

#endif

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/

