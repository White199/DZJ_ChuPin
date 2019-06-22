/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : AutoOperate.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 28/10/2015
* Description        : 各个动作操作函数
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Parameter.h"
#include "Auto_2.h"
#include "StatusControl.h"
#include "Auto.h"
#include "in.h"
#include "out.h"
#include "SpeedControl.h" 
#include "24Cxx.h"
#include "ActionOperate.h"

#include "PDO.h"
#include "Delay.h"


//----------------------------------//
u8  g_ActionDelay_Step = 0;
u32	g_ActionDelay_Timer = 0;
u8 Action_Delay_Finish = 0;
u8 g_DianJiao_Delay_Step = 0;
u32 g_DianJiao_Delay_Timer = 0;
u16 g_Delay_Timer = 0;
u8  g_Delay_Timer_Flag =0;
u32 HT_Position=0;
u32 Z_Position=0;
u8 IO_Flag=0;//提前打开IO


extern u8 HT_Flag,LC_Empty_Flag;
extern u32 HT_Height,LC_Height,Glue_Time,Delay_Time;
extern u16 Confirm_Order_Num,ShuZu_Count;
extern u16 X_Speed,Z_Speed,Y_Speed,O_Speed,A_Speed,B_Speed;

/**************************************************************************************************
**  函数名：  AXisMove()
**	输入参数：无
**	输出参数：无
**	函数功能：轴移动
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2015/11/21 
***************************************************************************************************/
u8  g_Key_Delay_Flag = FALSE;
u32 g_Key_Delay_Timer = 0;
u8 Key_Delay(u32 Delay_Time)
{
	if(g_Key_Delay_Timer>=Delay_Time)
	{
		 g_Key_Delay_Flag = FALSE;
		 g_Key_Delay_Timer = 0;
		 return 1;	
	}
	return 0;
}

/**************************************************************************************************
**  函数名：  LC_Key_Delay()
**	输入参数：无
**	输出参数：无
**	函数功能：轴移动
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2015/11/21 
***************************************************************************************************/
u8  g_LC_Key_Delay_Flag = FALSE;
u32 g_LC_Key_Delay_Timer = 0;
u8 LC_Key_Delay(u32 Delay_Time)
{
	if(g_LC_Key_Delay_Timer>=Delay_Time)
	{
		 g_LC_Key_Delay_Flag = FALSE;
		 g_LC_Key_Delay_Timer = 0;
		 return 1;	
	}
	return 0;
}

//延时命令相关变量参数
u8  g_Axis_Delay_Flag = FALSE;
u8  g_Axis_Delay_Done = FALSE;
u32 g_Axis_Delay_Timer = 0;
/**************************************************************************************************
**  函数名：  AXisMove()
**	输入参数：无
**	输出参数：无
**	函数功能：轴移动
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2015/11/21 
***************************************************************************************************/
void Action_Delay(u32 Delay_Time)
{

}

/**************************************************************************************************
**  函数名：  AXisMove()
**	输入参数：无
**	输出参数：无
**	函数功能：轴移动
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2015/11/21 
***************************************************************************************************/
u8 AXisMove(u8 Axsis,u32 Axsis_Position,u32 Axsis_Speed)
{
	 u8 Result=0;
	 switch(Axsis)
	 {
		  case X_Axsis:
			   if(Axsis_Position==Pulse_TotalCounter[0])
			   {
				    Result=1;
			   }
		     break;

		  case Z_Axsis:
			   if(Axsis_Position==Pulse_TotalCounter[1])
			   {
				    Result=1;
			   }
		     break;
				 
		  case L_Axsis:
			   if(Axsis_Position==Pulse_TotalCounter[2])
			   {
				    Result=1;
			   }
		     break;
				 
		  case O_Axsis:
			   if(Axsis_Position==Pulse_TotalCounter[3])
			   {
				    Result=1;
			   }		       
				 break;

		  case A_Axsis:
			   if(Axsis_Position==Pulse_TotalCounter[4])
			   {
				    Result=1;
			   }		       
				 break;
			   
		  case B_Axsis:
			   if(Axsis_Position==Pulse_TotalCounter[5])
			   {
				    Result=1;
			   }		       
				 break;			   
		  default:
				 Result=0;
				 break;
	 }
	 if(!Result)			 //执行动作
	 {
		 //---2017.9.27			 
		 //---2017.8.1  		  
		 MoveToTarget(Axsis_Position,Axsis,Axsis_Speed);//--加了一个速度参数0801			 
		 Result = 1;		  
	 }
	 return	Result;
}

/**************************************************************************************************
**  函数名：  LC_Parameter_Load_Calculate()
**	输入参数：
**	输出参数：
**	函数功能：
**	备注：	  
**  作者：       
**  开发日期：2016/11/09
***************************************************************************************************/
void LC_Parameter_Load_Calculate()
{
//	 u8 LC_Load_XNum=0;
//	 u8 LC_Load_YNum=0;
//	 if(((LC_Load_Pre_ListX>0)||(LC_Load_Pre_ListY>0))&&(LC_Parameter_Load.H_Num>0)&&(LC_Parameter_Load.V_Num>0))
//	 {
//	   LC_Load_XNum=SC_Parameter.SC_Num%LC_Parameter_Load.H_Num;	 //横向取余
//		 LC_Load_YNum=(SC_Parameter.SC_Num/LC_Parameter_Load.H_Num)%LC_Parameter_Load.V_Num;	 //横向取整再纵向取余
//		 Free_Program_Operate.Program[LC_Load_Pre_ListX].Value1=LC_Load_Position[LC_Load_XNum][LC_Load_YNum].XPoint+MINROBOTPOSITION;
//		 Free_Program_Operate.Program[LC_Load_Pre_ListY].Value1=LC_Load_Position[LC_Load_XNum][LC_Load_YNum].YPoint+MINROBOTPOSITION;
//	 }
}

/**************************************************************************************************
**  函数名：  LC_Parameter_Load_OAxis_Calculate()
**	输入参数：
**	输出参数：
**	函数功能：
**	备注：	  
**  作者：       
**  开发日期：2016/12/14
***************************************************************************************************/
void LC_Parameter_Load_OAxis_Calculate()
{
//	 u8 LC_Load_ONum_Place=0;
//	 u8 LC_Load_ONum_Get=0;
//	 if((LC_Parameter_Load.Switch>0)&&(LC_Parameter_Load.Layer>0)&&(LC_Load_Pre_ListO_Place>0))
//	 {
//	     LC_Load_ONum_Place=SC_Parameter.SC_Num/(LC_Parameter_Load.H_Num*LC_Parameter_Load.V_Num)%LC_Parameter_Load.Layer;
//		   LC_Load_Program_Operate.Program[LC_Load_Pre_ListO_Place].Value1=LC_Load_OAxis_Position[LC_Load_ONum_Place]+MINROBOTPOSITION;
//	     if(LC_Load_Pre_ListO_Get>0)
//		   {
//		     LC_Load_ONum_Get=LC_Load_ONum_Place+1;
//			   if(LC_Load_ONum_Get>LC_Parameter_Load.Layer)
//			   {
//			     LC_Load_ONum_Get=2;    
//			   }
//			   LC_Load_Program_Operate.Program[LC_Load_Pre_ListO_Get].Value1=LC_Load_OAxis_Position[LC_Load_ONum_Get]+MINROBOTPOSITION;
//		   }
//	 }
}

/**************************************************************************************************
**  函数名：  LC_Parameter_UnLoad_ZAxis_Calculate()
**	输入参数：
**	输出参数：
**	函数功能：
**	备注：	  
**  作者：       
**  开发日期：2016/12/14
***************************************************************************************************/
u32 LC_Parameter_UnLoad_ZAxis_Calculate()
{
//	 LC_UnLoad_ZAxis_Num=1;
//	 if((LC_Parameter_UnLoad.Switch==TRUE)&&(LC_Parameter_UnLoad.Layer>0)&&(LC_UnLoad_Pre_ListZ>0))
//	 {
//	     LC_UnLoad_ZAxis_Num=SC_Parameter.SC_Num%LC_Parameter_UnLoad.Layer;
//		   Free_Program_Operate.Program[LC_UnLoad_Pre_ListZ].Value1=LC_UnLoad_OAxis_Position[LC_UnLoad_ZAxis_Num]+MINROBOTPOSITION;	 	 
//	 }
//	 return Free_Program_Operate.Program[LC_UnLoad_Pre_ListZ].Value1;
}

/**************************************************************************************************
**  函数名：  LC_Parameter_UnLoad_OAxis_Calculate()
**	输入参数：
**	输出参数：
**	函数功能：
**	备注：	  
**  作者：       
**  开发日期：2016/12/14
***************************************************************************************************/
void LC_Parameter_UnLoad_OAxis_Calculate()
{
//	 u8 LC_UnLoad_ONum=0;
//	 if((LC_Parameter_UnLoad.Switch>0)&&(LC_Parameter_UnLoad.Layer>0)&&(LC_UnLoad_Pre_ListO>0))
//	 {
//	     LC_UnLoad_ONum=(SC_Parameter.SC_Num/(LC_Parameter_UnLoad.H_Num*LC_Parameter_UnLoad.V_Num))%LC_Parameter_UnLoad.Layer;
//		   LC_UnLoad_Program_Operate.Program[LC_UnLoad_Pre_ListO].Value1=LC_UnLoad_OAxis_Position[LC_UnLoad_ONum]+MINROBOTPOSITION;
//	 }
}

/**************************************************************************************************
**  函数名：  LC_Parameter_Load_Calculate()
**	输入参数：
**	输出参数：
**	函数功能：
**	备注：	  
**  作者：       
**  开发日期：2016/11/09
***************************************************************************************************/
void LC_Parameter_UnLoad_Calculate()
{
//	 u8 LC_UnLoad_XNum=0;
//	 u8 LC_UnLoad_YNum=0;
//	 if(((LC_UnLoad_Pre_ListX>0)||(LC_UnLoad_Pre_ListY>0))&&(LC_Parameter_UnLoad.H_Num>0)&&(LC_Parameter_UnLoad.V_Num>0))
//	 {
//	   LC_UnLoad_XNum=SC_Parameter.SC_Num%LC_Parameter_UnLoad.H_Num;	 //横向取余
//		 LC_UnLoad_YNum=(SC_Parameter.SC_Num/LC_Parameter_UnLoad.H_Num)%LC_Parameter_UnLoad.V_Num;	 //横向取整再纵向取余
//		 Free_Program_Operate.Program[LC_UnLoad_Pre_ListX].Value1=LC_UnLoad_Position[LC_UnLoad_XNum][LC_UnLoad_YNum].XPoint+MINROBOTPOSITION;
//		 Free_Program_Operate.Program[LC_UnLoad_Pre_ListY].Value1=LC_UnLoad_Position[LC_UnLoad_XNum][LC_UnLoad_YNum].YPoint+MINROBOTPOSITION;
//	 }
}

/**************************************************************************************************
**  函数名：  AutoActionStepList()
**	输入参数：需要运行的行号
**	输出参数： 
**	函数功能： 
**	备注：	   
**  作者：       
**  开发日期：2016/12/06
***************************************************************************************************/
u8 AutoActionStepList(u8 ActionLine)
{
	u8 result=0,i=0;
	if(ActionLine < Free_Program_Operate.Num)
	{
    while(Free_Program_Operate.Program[ActionLine+result].List==Free_Program_Operate.Program[ActionLine].List)
		{
		  Action_Step_List[result].Flag=TRUE;
			Action_Step_List[result].Line=i;
			result++;
			if((ActionLine+result)>=Free_Program_Operate.Num)
			{
				 break;
			}
			if(result==LISTNUM)	//如果指令等于６条
			{
			   break;
			}
		}		
	}
	return result;	
}

/**************************************************************************************************
**  函数名：  Program_Start_Action()
**	输入参数：需要运行的行号
**	输出参数： 
**	函数功能： 
**	备注：	   
**  作者：       
**  开发日期：2016/12/06
***************************************************************************************************/
u8 g_Program_Start_Action=FALSE;
u8 g_Program_Start_Action_Done=FALSE;
void Program_Start_Action()
{
	if(g_Program_Start_Action==FALSE)
	{
 	    Start_Action_Program_Operate.Flag  = 1;
 	    Start_Action_Program_Operate.Code  = 0;
	    Start_Action_Program_Operate.Name  = 0;
			Start_Action_Program_Operate.Num   = 18;
			//1-毛坯钩下降
			Start_Action_Program_Operate.Program[0].Flag   = 1;
			Start_Action_Program_Operate.Program[0].List   = 1;
			Start_Action_Program_Operate.Program[0].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[0].Key    = K_IOINSTRUCT_OUTPUT5; //Y2-1
			Start_Action_Program_Operate.Program[0].Value1 = 0;
			Start_Action_Program_Operate.Program[0].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[0].Value3 = 10;   //延时 100ms					
			//2-成品钩下降
			Start_Action_Program_Operate.Program[1].Flag   = 1;
			Start_Action_Program_Operate.Program[1].List   = 2;
			Start_Action_Program_Operate.Program[1].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[1].Key    = K_IOINSTRUCT_OUTPUT7; //Y3-1
			Start_Action_Program_Operate.Program[1].Value1 = 0;
			Start_Action_Program_Operate.Program[1].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[1].Value3 = 10;   //延时
			//3-毛坯钩缩回
			Start_Action_Program_Operate.Program[2].Flag   = 1;
			Start_Action_Program_Operate.Program[2].List   = 3;
			Start_Action_Program_Operate.Program[2].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[2].Key    = K_IOINSTRUCT_OUTPUT50; //Y25
			Start_Action_Program_Operate.Program[2].Value1 = 0;
			Start_Action_Program_Operate.Program[2].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[2].Value3 = 10;   //延时					
			//4-成品钩缩回
			Start_Action_Program_Operate.Program[3].Flag   = 1;
			Start_Action_Program_Operate.Program[3].List   = 4;
			Start_Action_Program_Operate.Program[3].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[3].Key    = K_IOINSTRUCT_OUTPUT52; //Y27
			Start_Action_Program_Operate.Program[3].Value1 = 0;
			Start_Action_Program_Operate.Program[3].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[3].Value3 = 10;   //延时	
			//5-垂直轴下降到第二个料盘位置
			Start_Action_Program_Operate.Program[4].Flag   = 1;
			Start_Action_Program_Operate.Program[4].List   = 5;
			Start_Action_Program_Operate.Program[4].Order  = OR_AXISORDER;
			Start_Action_Program_Operate.Program[4].Key    = K_OAXIS;   //O轴
			Start_Action_Program_Operate.Program[4].Value1 = LC_Load_OAxis_Position[1]+MINROBOTPOSITION;;
			Start_Action_Program_Operate.Program[4].Value2 = 50;	    //速度
			Start_Action_Program_Operate.Program[4].Value3 = 10;         //延时
			//6-毛坯钩伸出
			Start_Action_Program_Operate.Program[5].Flag   = 1;
			Start_Action_Program_Operate.Program[5].List   = 6;
			Start_Action_Program_Operate.Program[5].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[5].Key    = K_IOINSTRUCT_OUTPUT49; //Y24
			Start_Action_Program_Operate.Program[5].Value1 = 0;
			Start_Action_Program_Operate.Program[5].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[5].Value3 = 10;   //延时					
			//7-成品钩伸出
			Start_Action_Program_Operate.Program[6].Flag   = 1;
			Start_Action_Program_Operate.Program[6].List   = 7;
			Start_Action_Program_Operate.Program[6].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[6].Key    = K_IOINSTRUCT_OUTPUT51; //Y26
			Start_Action_Program_Operate.Program[6].Value1 = 0;
			Start_Action_Program_Operate.Program[6].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[6].Value3 = 10;   //延时
			//8-毛坯钩上升
			Start_Action_Program_Operate.Program[7].Flag   = 1;
			Start_Action_Program_Operate.Program[7].List   = 8;
			Start_Action_Program_Operate.Program[7].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[7].Key    = K_IOINSTRUCT_OUTPUT6; //Y2-0
			Start_Action_Program_Operate.Program[7].Value1 = 0;
			Start_Action_Program_Operate.Program[7].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[7].Value3 = 10;   //延时					
			//9-成品钩上升
			Start_Action_Program_Operate.Program[8].Flag   = 1;
			Start_Action_Program_Operate.Program[8].List   = 9;
			Start_Action_Program_Operate.Program[8].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[8].Key    = K_IOINSTRUCT_OUTPUT8; //Y3-0
			Start_Action_Program_Operate.Program[8].Value1 = 0;
			Start_Action_Program_Operate.Program[8].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[8].Value3 = 10;   //延时
			//10-毛坯钩缩回
			Start_Action_Program_Operate.Program[9].Flag   = 1;
			Start_Action_Program_Operate.Program[9].List   = 10;
			Start_Action_Program_Operate.Program[9].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[9].Key    = K_IOINSTRUCT_OUTPUT50; //Y25
			Start_Action_Program_Operate.Program[9].Value1 = 0;
			Start_Action_Program_Operate.Program[9].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[9].Value3 = 10;   //延时					
			//11-成品钩缩回
			Start_Action_Program_Operate.Program[10].Flag   = 1;
			Start_Action_Program_Operate.Program[10].List   = 11;
			Start_Action_Program_Operate.Program[10].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[10].Key    = K_IOINSTRUCT_OUTPUT52; //Y27
			Start_Action_Program_Operate.Program[10].Value1 = 0;
			Start_Action_Program_Operate.Program[10].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[10].Value3 = 10;   //延时
			//12-垂直轴上升到第一个料盘位置
			Start_Action_Program_Operate.Program[11].Flag   = 1;
			Start_Action_Program_Operate.Program[11].List   = 12;
			Start_Action_Program_Operate.Program[11].Order  = OR_AXISORDER;
			Start_Action_Program_Operate.Program[11].Key    = K_OAXIS;   //O轴
			Start_Action_Program_Operate.Program[11].Value1 = LC_Load_OAxis_Position[0]+MINROBOTPOSITION;
			Start_Action_Program_Operate.Program[11].Value2 = 50;	    //速度
			Start_Action_Program_Operate.Program[11].Value3 = 10;         //延时
			//13-毛坯钩伸出
			Start_Action_Program_Operate.Program[12].Flag   = 1;
			Start_Action_Program_Operate.Program[12].List   = 13;
			Start_Action_Program_Operate.Program[12].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[12].Key    = K_IOINSTRUCT_OUTPUT49; //Y24
			Start_Action_Program_Operate.Program[12].Value1 = 0;
			Start_Action_Program_Operate.Program[12].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[12].Value3 = 10;   //延时					
			//14-成品钩伸出
			Start_Action_Program_Operate.Program[13].Flag   = 1;
			Start_Action_Program_Operate.Program[13].List   = 14;
			Start_Action_Program_Operate.Program[13].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[13].Key    = K_IOINSTRUCT_OUTPUT51; //Y26
			Start_Action_Program_Operate.Program[13].Value1 = 0;
			Start_Action_Program_Operate.Program[13].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[13].Value3 = 10;   //延时
			//15-毛坯钩下降
			Start_Action_Program_Operate.Program[14].Flag   = 1;
			Start_Action_Program_Operate.Program[14].List   = 15;
			Start_Action_Program_Operate.Program[14].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[14].Key    = K_IOINSTRUCT_OUTPUT5; //Y2-1
			Start_Action_Program_Operate.Program[14].Value1 = 0;
			Start_Action_Program_Operate.Program[14].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[14].Value3 = 10;   //延时					
			//16-成品钩下降
			Start_Action_Program_Operate.Program[15].Flag   = 1;
			Start_Action_Program_Operate.Program[15].List   = 16;
			Start_Action_Program_Operate.Program[15].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[15].Key    = K_IOINSTRUCT_OUTPUT7; //Y3-1
			Start_Action_Program_Operate.Program[15].Value1 = 0;
			Start_Action_Program_Operate.Program[15].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[15].Value3 = 10;   //延时
			//17-毛坯钩缩回
			Start_Action_Program_Operate.Program[16].Flag   = 1;
			Start_Action_Program_Operate.Program[16].List   = 17;
			Start_Action_Program_Operate.Program[16].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[16].Key    = K_IOINSTRUCT_OUTPUT50; //Y25
			Start_Action_Program_Operate.Program[16].Value1 = 0;
			Start_Action_Program_Operate.Program[16].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[16].Value3 = 10;   //延时					
			//18-成品钩缩回
			Start_Action_Program_Operate.Program[17].Flag   = 1;
			Start_Action_Program_Operate.Program[17].List   = 18;
			Start_Action_Program_Operate.Program[17].Order  = OR_IOORDER;
			Start_Action_Program_Operate.Program[17].Key    = K_IOINSTRUCT_OUTPUT52; //Y27
			Start_Action_Program_Operate.Program[17].Value1 = 0;
			Start_Action_Program_Operate.Program[17].Value2 = 8;	  //检测开
			Start_Action_Program_Operate.Program[17].Value3 = 10;   //延时

			g_Program_Start_Action=TRUE;
			g_Program_Start_Action_Done=FALSE;
			g_Start_ActionRun_Step=0;
			g_Start_Action_PresentLine=0;
	}
}

/**************************************************************************************************
**  函数名：  Program_Reset_Action()
**	输入参数：需要运行的行号
**	输出参数： 
**	函数功能： 
**	备注：	   
**  作者：       
**  开发日期：2016/12/06
***************************************************************************************************/
u8 g_Program_Reset_Action=FALSE;
u8 g_Program_Reset_Action_Done=FALSE;
void Program_Reset_Action()
{
	if(g_Program_Reset_Action==FALSE)
	{
 	    Reset_Action_Program_Operate.Flag  = 1;
 	    Reset_Action_Program_Operate.Code  = 0;
	    Reset_Action_Program_Operate.Name  = 0;
	    Reset_Action_Program_Operate.Num   = 2;
			//1-轴复位
			Reset_Action_Program_Operate.Program[0].Flag   = 1;
			Reset_Action_Program_Operate.Program[0].List   = 1;
			Reset_Action_Program_Operate.Program[0].Order  = OR_AXISORDER;
			Reset_Action_Program_Operate.Program[0].Key    = K_OAXISPOSITION; //O轴->0
			Reset_Action_Program_Operate.Program[0].Value1 = MINROBOTPOSITION;//位置
			Reset_Action_Program_Operate.Program[0].Value2 = 20;   //速度
			Reset_Action_Program_Operate.Program[0].Value3 = 10;    //延时					
			//2-输出复位信号
			Reset_Action_Program_Operate.Program[1].Flag   = 1;
			Reset_Action_Program_Operate.Program[1].List   = 2;
			Reset_Action_Program_Operate.Program[1].Order  = OR_IOORDER;
			Reset_Action_Program_Operate.Program[1].Key    = K_IOINSTRUCT_OUTPUT23; //Y11-1
			Reset_Action_Program_Operate.Program[1].Value1 = 0;
			Reset_Action_Program_Operate.Program[1].Value2 = 8;	  //检测开
			Reset_Action_Program_Operate.Program[1].Value3 = 10;   //延时

			g_Program_Reset_Action=TRUE;
			g_Program_Reset_Action_Done=FALSE;
			g_Reset_ActionRun_Step=0;
			g_Reset_Action_PresentLine=0;
	}
}


/**************************************************************************************************
**  函数名：  AutoActionOutControl()
**	输入参数：需要运行的行号
**	输出参数：动作是否成功输出
**	函数功能：自动模式控制函数
**	备注：	  控制设备自动运行
**  作者：    周海波    
**  开发日期：2015/10/28
***************************************************************************************************/
//static u32 gs_SC_Parameter_SC_Num=0;
u8 AutoActionOutControl(u16 ActionLine)
{
	u8 result = 0,i=0;
	if(g_LC_Step_Control)
	{}
	else if(g_Start_Step_Control)
	{}
	else
	{
	   result = Action_Step_List_Num;
	}
if(LC_Empty_Flag == 0)
{
	if(ActionLine < Free_Program_Operate1.Num)	  //.Num = 10 (0~9存在程序)
	{
			if(ActionAllowJudge(Free_Program_Operate1,ShuZu_Count) == 0)	//动作执行前进行动作安全合法性判断
			{
						switch(Free_Program_Operate1.Program[ShuZu_Count].Key)
						{
							case A_DianJiao:{
								if(HT_Flag)//只要当前位置大于回抬高度则回抬
								{
								 Z_Position=Pulse_TotalCounter[1]-HT_Height;
								 AXisMove(Z_Axsis,Z_Position,Z_Speed);								
								 AXisMove(X_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value1,X_Speed);
								 AXisMove(L_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value2,Y_Speed);
								 AXisMove(A_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value4,A_Speed);
								 AXisMove(B_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value5,B_Speed);
									for(i=3;i<6;i++)
									{
										if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
											ResetOutput(i+1);//先让胶头下移		
									}	
								}
								else
								{
								 //位置移动: 位置 , 速度				//点胶模式
								 AXisMove(X_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value1,X_Speed);
								 AXisMove(L_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value2,Y_Speed);
								 AXisMove(Z_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value3,Z_Speed);
								 AXisMove(A_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value4,A_Speed);
								 AXisMove(B_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value5,B_Speed);
									for(i=3;i<6;i++)
									{
										if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
											ResetOutput(i+1);//先让胶头下移		
									}	
								}
								result=0;
							}
							break;
							
							case A_QuLiao:{
								//取料  先回抬再下去取料
								if(HT_Flag)//只要当前位置大于回抬高度则回抬
								{
								 Z_Position=Pulse_TotalCounter[1]-HT_Height;
								 AXisMove(Z_Axsis,Z_Position,Z_Speed);	
								 AXisMove(X_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value1,X_Speed);
								 AXisMove(L_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value2,Y_Speed);
								 AXisMove(A_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value4,A_Speed);
								 AXisMove(B_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value5,B_Speed);									
									for(i=13;i<21;i++)
									{
										if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
											ResetOutput(i+1);//先让钻头下移		
									}
								}
								else
								{
								 AXisMove(X_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value1,X_Speed);
								 AXisMove(L_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value2,Y_Speed);
								 AXisMove(Z_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value3,Z_Speed);
								 AXisMove(A_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value4,A_Speed);
								 AXisMove(B_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value5,B_Speed);
									for(i=13;i<21;i++)
									{
										if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
											ResetOutput(i+1);//先让钻头下移		
									}
								}
								result=0;
							}
								break;
								
							case A_DianZuan:{
								//点钻 先回抬再下去放料
								if(HT_Flag)//只要当前位置大于回抬高度则回抬
								{
								 Z_Position=Pulse_TotalCounter[1]-HT_Height;
								 AXisMove(Z_Axsis,Z_Position,Z_Speed);								
								 AXisMove(X_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value1,X_Speed);
								 AXisMove(L_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value2,Y_Speed);
								 AXisMove(A_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value4,A_Speed);
								 AXisMove(B_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value5,B_Speed);
									for(i=13;i<21;i++)
									{
										if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
											ResetOutput(i+1);//先让钻头下移		
									}
								}
								else
								{
								 AXisMove(X_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value1,X_Speed);
								 AXisMove(L_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value2,Y_Speed);
								 AXisMove(Z_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value3,Z_Speed);
								 AXisMove(A_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value4,A_Speed);
								 AXisMove(B_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value5,B_Speed);
									for(i=13;i<21;i++)
									{
										if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
											ResetOutput(i+1);//先让钻头下移		
									}
								}
								result=0;								
							}								
								break;
							case A_KongYi:{
								//空移 
								 AXisMove(X_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value1,X_Speed);
								 AXisMove(L_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value2,Y_Speed);
								 AXisMove(Z_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value3,Z_Speed);
								 AXisMove(A_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value4,A_Speed);
								 AXisMove(B_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value5,B_Speed);
								result=0;								
							}								
								break;	
							case A_Delay:
									g_Delay_Timer_Flag=1;//开始计时
									g_Delay_Timer=0;
									result=0;
								break;
							default:{
								 result = 13;	//轴控指令类型异常
							}break;
						}//switch(order) 
				if(g_LC_Step_Control) //执行料仓程序
				{	
				}
				else if(g_Start_Step_Control) //执行启动程序
				{	
				}
				else
				{
					if(result<9)
					{
						Action_Step_Run_Num++;
					}
				}
			}
			else
			{
				result = 11; //主要指令类型异常
			}
		}
		else
		{
			result = 10;				//当前行号长度异常
		}
	}
	else//料仓料钻取完空移
	{
		 AXisMove(Z_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[Z_Axsis]*100);
		 AXisMove(L_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[L_Axsis]*100);
		 AXisMove(X_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[X_Axsis]*100);
		 result = 0;		
	}
	return result;		
}

/**************************************************************************************************
**  函数名：  AutoActionOutConfirm()
**	输入参数：需要运行的行号
**	输出参数：动作是否已确认
**	函数功能：自动模式控制函数
**	备注：	  控制设备自动运行
**  作者：    周海波    
**  开发日期：2015/10/28
***************************************************************************************************/
u8 Action_Flag=0;
u8 AutoActionOutConfirm(SaveProgram Program_Operate1,u16 ShuZu_Count)
{
	u8 result = 0,i=0;
	if(LC_Empty_Flag==0)
	{
	switch(Free_Program_Operate1.Program[ShuZu_Count].Key)
	{
		case A_DianJiao:{ // 点胶 是否移动到目标位置 X、Y、Z是否移动到位 
				if(HT_Flag==1)
				{
					if(Action_Flag==0&&Z_Position==Pulse_TotalCounter[Z_Axsis])
					{
						 //位置移动: 位置 , 速度				//点胶模式
						 Action_Flag=1;
						 AXisMove(Z_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value3,Z_Speed);				 
					}
					 if(Action_Flag==1&&(Timer_Enable_Z == DISABLE)&&(Timer_Enable_X == DISABLE) && (Timer_Enable_L == DISABLE)&& (Timer_Enable_A == DISABLE)&& (Timer_Enable_B == DISABLE)
						 &&(Free_Program_Operate1.Program[ShuZu_Count].Value1==Pulse_TotalCounter[X_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value2==Pulse_TotalCounter[L_Axsis])
						 &&(Free_Program_Operate1.Program[ShuZu_Count].Value3==Pulse_TotalCounter[Z_Axsis])
						 &&(Free_Program_Operate1.Program[ShuZu_Count].Value4==Pulse_TotalCounter[A_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value5==Pulse_TotalCounter[B_Axsis]))//除Z轴外其他轴达到目标位置
					 {
						for(i=0;i<3;i++)
						{
							if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
								ResetOutput(i+1);//出胶		
						}						
						g_DianJiao_Delay_Step=1;
						 Action_Flag=2;
					 }
					 if(g_DianJiao_Delay_Step==1&&g_DianJiao_Delay_Timer>=Glue_Time)//点胶结束关闭IO
					 {
							 IO_Flag=2;//点胶结束
							 g_DianJiao_Delay_Step=0;
							 g_DianJiao_Delay_Timer=0;
							for(i=0;i<3;i++)
							{
								if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
									SetOutput(i+1);//出胶结束		
							}
							g_LCDelay_Step=1;
							 g_LCDelay_Timer=0;
					 }
					 if(g_LCDelay_Step==1&&g_LCDelay_Timer>=Delay_Time)//出胶后移动延时
					 {
							 Action_Flag=0;
							 g_LCDelay_Step=0;
							 g_LCDelay_Timer=0;
							 HT_Flag==0;
							 result = 1; 	
					 }
				}
				else
				{
				 if(Action_Flag==0&&(Timer_Enable_Z == DISABLE)&&(Timer_Enable_X == DISABLE) && (Timer_Enable_L == DISABLE)&& (Timer_Enable_A == DISABLE)&& (Timer_Enable_B == DISABLE)
					 &&(Free_Program_Operate1.Program[ShuZu_Count].Value1==Pulse_TotalCounter[X_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value2==Pulse_TotalCounter[L_Axsis])
					 &&(Free_Program_Operate1.Program[ShuZu_Count].Value3==Pulse_TotalCounter[Z_Axsis])
				 &&(Free_Program_Operate1.Program[ShuZu_Count].Value4==Pulse_TotalCounter[A_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value5==Pulse_TotalCounter[B_Axsis]))//除Z轴外其他轴达到目标位置
				 {
					 Action_Flag=1;
					for(i=0;i<3;i++)
					{
						if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
							ResetOutput(i+1);//出胶		
					}	
					g_DianJiao_Delay_Step=1;						 
					 Action_Flag=2;
				 }
				 if(g_DianJiao_Delay_Step==1&&g_DianJiao_Delay_Timer>=Glue_Time)//点胶结束关闭IO
				 {
						 IO_Flag=2;//点胶结束
						 g_DianJiao_Delay_Step=0;
						for(i=0;i<3;i++)
						{
							if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
								SetOutput(i+1);//点胶结束		
						}
						g_LCDelay_Step=1;
						 g_LCDelay_Timer=0;
				 }
				 if(g_LCDelay_Step==1&&g_LCDelay_Timer>=Delay_Time)
				 {
						 Action_Flag=0;
						 g_LCDelay_Step=0;
						 g_LCDelay_Timer=0;
						Action_Flag = 0;
						 g_DianJiao_Delay_Step=0;
						 g_DianJiao_Delay_Timer=0;
						result = 1; 	
				 }
			 }
			}
		 break;
		case A_QuLiao:{ //取料
				if(HT_Flag==1)
				{
					if(Action_Flag==0&&Z_Position==Pulse_TotalCounter[Z_Axsis])
					{
						 //位置移动: 位置 , 速度				//点胶模式
						 Action_Flag=1;
						 AXisMove(Z_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value3,Z_Speed);					 
					}
					 if(Action_Flag==1&&(Timer_Enable_Z == DISABLE)&&(Timer_Enable_X == DISABLE) && (Timer_Enable_L == DISABLE)&& (Timer_Enable_A == DISABLE)&& (Timer_Enable_B == DISABLE)
						 &&(Free_Program_Operate1.Program[ShuZu_Count].Value1==Pulse_TotalCounter[X_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value2==Pulse_TotalCounter[L_Axsis])
						 &&(Free_Program_Operate1.Program[ShuZu_Count].Value3==Pulse_TotalCounter[Z_Axsis])
						 &&(Free_Program_Operate1.Program[ShuZu_Count].Value4==Pulse_TotalCounter[A_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value5==Pulse_TotalCounter[B_Axsis]))//除Z轴外其他轴达到目标位置
					 {
						 Action_Flag=2;
						for(i=5;i<13;i++)
						{
							if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
								ResetOutput(i+1);//吸钻	
						}
						 g_LCDelay_Step=1;
						 g_LCDelay_Timer=0;
					 }
					 if(g_LCDelay_Step==1&&g_LCDelay_Timer>=Delay_Time)//取料结束
					 {
						 Action_Flag=0;
						 HT_Flag=0;
						 g_LCDelay_Timer=0;
						 g_LCDelay_Step=0;
						 result = 1;				 
					 }
				}
				else
				{
				 if(Action_Flag==0&&(Timer_Enable_Z == DISABLE)&&(Timer_Enable_X == DISABLE) && (Timer_Enable_L == DISABLE)&& (Timer_Enable_A == DISABLE)&& (Timer_Enable_B == DISABLE)
					 &&(Free_Program_Operate1.Program[ShuZu_Count].Value1==Pulse_TotalCounter[X_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value2==Pulse_TotalCounter[L_Axsis])
					 &&(Free_Program_Operate1.Program[ShuZu_Count].Value3==Pulse_TotalCounter[Z_Axsis])
					 &&(Free_Program_Operate1.Program[ShuZu_Count].Value4==Pulse_TotalCounter[A_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value5==Pulse_TotalCounter[B_Axsis]))//除Z轴外其他轴达到目标位置
				 {
					 Action_Flag=1;
					for(i=5;i<13;i++)
					{
						if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
							ResetOutput(i+1);//取料	
					}
					 Action_Flag=2;
					g_LCDelay_Step=1;
					 g_LCDelay_Timer=0;					 
				 }
				 if(g_LCDelay_Step==1&&g_LCDelay_Timer>=Delay_Time)//取料稳定
				 {
					 Action_Flag=0;
					 g_LCDelay_Timer=0;
					 g_LCDelay_Step=0;
					 result = 1;				 
				 }
			 }
		 }break;
		case A_DianZuan:{ //放料
				if(HT_Flag==1)
				{
					if(Action_Flag==0&&Z_Position==Pulse_TotalCounter[Z_Axsis])
					{
						 //位置移动: 位置 , 速度				//点胶模式
						 Action_Flag=1;
						 AXisMove(Z_Axsis,Free_Program_Operate1.Program[ShuZu_Count].Value3,Z_Speed);					
					}
					 if(Action_Flag==1&&(Timer_Enable_Z == DISABLE)&&(Timer_Enable_X == DISABLE) && (Timer_Enable_L == DISABLE)&& (Timer_Enable_A == DISABLE)&& (Timer_Enable_B == DISABLE)
						 &&(Free_Program_Operate1.Program[ShuZu_Count].Value1==Pulse_TotalCounter[X_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value2==Pulse_TotalCounter[L_Axsis])
						 &&(Free_Program_Operate1.Program[ShuZu_Count].Value3==Pulse_TotalCounter[Z_Axsis])
						 &&(Free_Program_Operate1.Program[ShuZu_Count].Value4==Pulse_TotalCounter[A_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value5==Pulse_TotalCounter[B_Axsis]))//除Z轴外其他轴达到目标位置
					 {
						 Action_Flag=2;
						for(i=5;i<13;i++)
						{
							if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
								SetOutput(i+1);//放料		
						}
						g_LCDelay_Step=1;
						 g_LCDelay_Timer=0;
					 }
					 if(g_LCDelay_Step==1&&g_LCDelay_Timer>=Delay_Time)//放料结束
					 {
						 Action_Flag=0;
						 HT_Flag=0;
						 g_LCDelay_Timer=0;
						 g_LCDelay_Step=0;
						 result = 1;				 
					 }
				}
				else
				{
				 if(Action_Flag==0&&(Timer_Enable_Z == DISABLE)&&(Timer_Enable_X == DISABLE) && (Timer_Enable_L == DISABLE)&& (Timer_Enable_A == DISABLE)&& (Timer_Enable_B == DISABLE)
					 &&(Free_Program_Operate1.Program[ShuZu_Count].Value1==Pulse_TotalCounter[X_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value2==Pulse_TotalCounter[L_Axsis])
					 &&(Free_Program_Operate1.Program[ShuZu_Count].Value3==Pulse_TotalCounter[Z_Axsis])
					 &&(Free_Program_Operate1.Program[ShuZu_Count].Value4==Pulse_TotalCounter[A_Axsis])&& (Free_Program_Operate1.Program[ShuZu_Count].Value5==Pulse_TotalCounter[B_Axsis]))//除Z轴外其他轴达到目标位置
				 {
					 Action_Flag=1;
					for(i=5;i<13;i++)
					{
						if(Free_Program_Operate1.Program[ShuZu_Count].IO_Group&(0x00000001<<i))
							SetOutput(i+1);//放料		
					}
					g_LCDelay_Step=1;
					 g_LCDelay_Timer=0;						 
				 }						 
				 if(g_LCDelay_Step==1&&g_LCDelay_Timer>=Delay_Time)//放料稳定
				 {
					 Action_Flag=0;
					 g_LCDelay_Timer=0;
					 g_LCDelay_Step=0;
					 result = 1;
				 }
			 }				 
				 
		 }break;
		case A_KongYi:{ //空移 不需要回抬
				 if((Timer_Enable_X == DISABLE) && (Timer_Enable_L == DISABLE)&&(Timer_Enable_Z == DISABLE) && (Timer_Enable_A == DISABLE)&&(Timer_Enable_B == DISABLE)
					 &&(Free_Program_Operate1.Program[ShuZu_Count].Value1==Pulse_TotalCounter[0])&& (Free_Program_Operate1.Program[ShuZu_Count].Value2==Pulse_TotalCounter[2])
				 &&(Free_Program_Operate1.Program[ShuZu_Count].Value3==Pulse_TotalCounter[1])&& (Free_Program_Operate1.Program[ShuZu_Count].Value4==Pulse_TotalCounter[4])
				 &&(Free_Program_Operate1.Program[ShuZu_Count].Value5==Pulse_TotalCounter[5]))//除Z轴外其他轴达到目标位置
				 {
					 result = 1; 								 
				 }			 
		 }break;	
		case A_Delay:{ //延时
			if(g_Delay_Timer>=Free_Program_Operate1.Program[ShuZu_Count].Delay_Time)
			{
					g_Delay_Timer_Flag=0;
					g_Delay_Timer=0;
				  result = 1; 
			 }			 
		 }break;
		default:
			break;
	 }		
	}
	else
	{
		if((Timer_Enable_O == DISABLE) && (Timer_Enable_L == DISABLE)&& (Timer_Enable_Z == DISABLE)&&
			(MINROBOTPOSITION==Pulse_TotalCounter[3])&& (MINROBOTPOSITION==Pulse_TotalCounter[2])&& (MINROBOTPOSITION==Pulse_TotalCounter[1]))
		{
				g_Auto_ActionRun_Step = 2;
				result = 1; 
		}			
	}
	return result;		
}

/**************************************************************************************************
**  函数名：  AutoActionOutDelay()
**	输入参数：需要运行的行号
**	输出参数：延时是否完成
**	函数功能：自动运行动作延时控制
**	备注：	  控制设备自动运行
**  作者：    周海波    
**  开发日期：2015/10/28
***************************************************************************************************/
u8 AutoActionOutDelay(SaveProgram Program_Operate,u8 ActionLine)
{
	u8 result = FALSE;
	if((K_ZAXISPOSITION<=Program_Operate.Program[ActionLine].Key)&&(Program_Operate.Program[ActionLine].Key<=K_UNLOADPOSITION))
	{
	    result = TRUE;
	}
	else if(Program_Operate.Program[ActionLine].Value3==0)
	{
	    result = TRUE;
	}
	else
	{
		switch(g_ActionDelay_Step)
		{
			case 0:
				 g_ActionDelay_Timer = 0;
				 g_ActionDelay_Step = 1;
				 break;
			case 1:
				 if(g_ActionDelay_Timer >= Program_Operate.Program[ActionLine].Value3 )
				 { 
					g_ActionDelay_Step = 0;
					g_ActionDelay_Timer=0;
					result = TRUE;
				 }
				 break;
			default:
				 g_ActionDelay_Step = 0;
				 break;
		}
	}
	return result;		
}

/**************************************************************************************************
**  函数名：  AutoActionOutDelay()
**	输入参数：需要运行的行号
**	输出参数：延时是否完成
**	函数功能：自动运行动作延时控制
**	备注：	  控制设备自动运行
**  作者：    周海波    
**  开发日期：2015/10/28
***************************************************************************************************/
u8  g_LCDelay_Step = 0;
u32	g_LCDelay_Timer = 0;
u8 LCActionOutDelay(SaveProgram Program_Operate,u8 ActionLine)
{
	u8 result = FALSE;
	if((K_ZAXISPOSITION<=Program_Operate.Program[ActionLine].Key)&&(Program_Operate.Program[ActionLine].Key<=K_UNLOADPOSITION))
	{
	    result = TRUE;
	}
	else if(Program_Operate.Program[ActionLine].Value3==0)
	{
	    result = TRUE;
	}
	else
	{
		switch(g_LCDelay_Step)
		{
			case 0:
				 g_LCDelay_Timer = 0;
				 g_LCDelay_Step = 1;
				 break;
			case 1:
				 if(g_LCDelay_Timer >= Program_Operate.Program[ActionLine].Value3 )
				 { 
					g_LCDelay_Step = 0;
					g_LCDelay_Timer=0;
					result = TRUE;
				 }
				 break;
			default:
				 g_LCDelay_Step = 0;
				 break;
		}
	}
	return result;		
}

/**************************************************************************************************
**  函数名：  ActionAllowJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：动作执行前，合法性判断
**	备注：	  在动作执行前，先判断动作的合法性，动作合法
**            即执行动作，动作有异常，这停机报警提示
**  作者：    周海波    
**  开发日期：2015/10/21
********************************** *****************************************************************/
u8 ActionAllowJudge(SaveProgram Program_Operate, u8 ActionLine)
{
	u8 result = TRUE;
	switch(Program_Operate.Program[ActionLine].Key)
	{
		case A_QuLiao:
//			 if((K_PROGRAMSTART<=Program_Operate.Program[ActionLine].Key)&&(Program_Operate.Program[ActionLine].Key<=K_OAXISRESET))
//			 {
//			     result = FALSE;
//			 }
//			 else
//			 {
//			     result = 12;	//基本指令类型异常
//			 }
			     result = FALSE;
			 break;
		case A_DianJiao:
//			 if((K_ZAXISPOSITION<=Program_Operate.Program[ActionLine].Key)&&(Program_Operate.Program[ActionLine].Key<=K_OAXIS))
//			 {
//			     result = FALSE;
//			 }
//			 else
//			 {
//			     result = 13;	//轴控指令类型异常
//			 }
			     result = FALSE;
			 break;
		case A_DianZuan: 
//			 if((K_IOINSTRUCT_OUTPUT1<=Program_Operate.Program[ActionLine].Key)&&(Program_Operate.Program[ActionLine].Key<=K_IOINSTRUCT_INPUT7))
//			 {
//			     result = FALSE;
//			 }
//			 else
//			 {
//			     result = 14;	//IO控制指令类型异常
//			 }
			     result = FALSE;
			 break;
		case A_KongYi:
			     result = FALSE;			
			break;
		case A_Delay:
			     result = FALSE;			
			break;
		default:
			 result = 11;	//主要指令类型异常
			 break;
	}
	return result;
}
/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/
