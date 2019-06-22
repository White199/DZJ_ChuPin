#include "stm32f10x.h"
#include "integer.h"
#include "DGUS.h"
//#include "IOCommand.h"

/*----------------IO指令--hyd---20161129---------*/
void IO_Command(void)
{
	switch(LCD_Var_Address)
	{
	  //单向阀
	  	case 0x2150:		   //IO指令-27	 跳转到第2页
			 LCD_Current_Page = 2;
		     break;

		case 0x2151:		   //Y0-1
			 ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT1);
			 LCD_Current_Page = 2;
		     break;
		case 0x2152:		   //Y0-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT2);
			 LCD_Current_Page = 2;
			 break;

		case 0x2153:		   //Y1-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT3);
			 LCD_Current_Page = 2;
			 break;
		case 0x2154:		   //Y1-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT4);
			 LCD_Current_Page = 2;
			 break;

		case 0x2155:		   //Y2-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT5);
			 LCD_Current_Page = 2;
			 break;
		case 0x2156:		   //Y2-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT6);
			 LCD_Current_Page = 2;
			 break;

		case 0x2157:		   //Y3-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT7);
			 LCD_Current_Page = 2;
			 break;
		case 0x2158:		   //Y3-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT8);
			 LCD_Current_Page = 2;
			 break;

		case 0x2159:		   //Y4-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT9);
			 LCD_Current_Page = 2;
			 break;
		case 0x215A:		   //Y4-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT10);
			 LCD_Current_Page = 2;
			 break;

		case 0x215B:		   //Y5-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT11);
			 LCD_Current_Page = 2;
			 break;
		case 0x215C:		   //Y5-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT12);
			 LCD_Current_Page = 2;
			 break;

		case 0x215D:		  //Y6-1
			 ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT13);
			 LCD_Current_Page = 2;   
		     break;
		case 0x215E:		  //Y6-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT14);
			 LCD_Current_Page = 2;  
			 break;

		case 0x215F:		  //Y7-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT15);
			 LCD_Current_Page = 2;  
			 break;
		case 0x2160:		  //Y7-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT16);
			 LCD_Current_Page = 2;  
			 break;

		case 0x2161:		  //Y8-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT17);
			 LCD_Current_Page = 2;  
			 break;
		case 0x2162:		  //Y8-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT18);
			 LCD_Current_Page = 2;
			 break;

		case 0x2163:		 //Y9-1		     
			 ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT19);
			 LCD_Current_Page = 2;
			 break;
		case 0x2164:		 //Y9-0
			 ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT20);
			 LCD_Current_Page = 2;
		     break;

		case 0x2165:		 //Y10-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT21);
			 LCD_Current_Page = 2;
		     break;
		case 0x2166:		 //Y10-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT22);
			 LCD_Current_Page = 2;
		     break;

		case 0x2167:		  //Y11-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT23);
			 LCD_Current_Page = 2;  
			 break;
		case 0x2168:		  //Y11-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT24);
			 LCD_Current_Page = 2;  
			 break;

		case 0x2169:		  //Y12-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT25);
			 LCD_Current_Page = 2;  
			 break;
		case 0x216A:		  //Y12-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT26);
			 LCD_Current_Page = 2;
			 break;

		case 0x216B:		 //Y13-1		     
			 ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT27);
			 LCD_Current_Page = 2;
			 break;
		case 0x216C:		 //Y13-0
			 ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT28);
			 LCD_Current_Page = 2;
		     break;

		case 0x216D:		 //Y14-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT29);
			 LCD_Current_Page = 2;
		     break;
		case 0x216E:		 //Y14-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT30);
			 LCD_Current_Page = 2;
		     break;

		case 0x216F:		 //Y15-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT31);
			 LCD_Current_Page = 2;
		     break;
		case 0x2170:		 //Y15-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT32);
			 LCD_Current_Page = 2;
		     break;

		case 0x2171:		  //Y16-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT33);
			 LCD_Current_Page = 2;  
			 break;
		case 0x2172:		  //Y16-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT34);
			 LCD_Current_Page = 2;  
			 break;

		case 0x2173:		  //Y17-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT35);
			 LCD_Current_Page = 2;  
			 break;
		case 0x2174:		  //Y17-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT36);
			 LCD_Current_Page = 2;
			 break;

		case 0x2175:		 //Y18-1		     
			 ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT37);
			 LCD_Current_Page = 2;
			 break;
		case 0x2176:		 //Y18-0
			 ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT38);
			 LCD_Current_Page = 2;
		     break;

		case 0x2177:		 //Y19-1
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT39);
			 LCD_Current_Page = 2;
		     break;
		case 0x2178:		 //Y19-0
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT40);
			 LCD_Current_Page = 2;
		     break;

	//双向阀
		case 0x2179:		 //Y20-Y21
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT45);
			 LCD_Current_Page = 2;
		     break;
		case 0x217A:		 //Y21-Y20
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT46);
			 LCD_Current_Page = 2;
		     break;

		case 0x217B:		 //Y22-Y23
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT47);
			 LCD_Current_Page = 2;
		     break;
		case 0x217C:		 //Y23-Y22
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT48);
			 LCD_Current_Page = 2;
		     break;

		case 0x217D:		 //Y24-Y25
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT49);
			 LCD_Current_Page = 2;
		     break;
		case 0x217E:		 //Y25-Y24
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT50);
			 LCD_Current_Page = 2;
		     break;

		case 0x217F:		 //Y26-Y27
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT51);
			 LCD_Current_Page = 2;
		     break;
		case 0x2180:		 //Y27-Y26
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT52);
			 LCD_Current_Page = 2;
		     break;

		case 0x2181:		 //Y28-Y29
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT53);
			 LCD_Current_Page = 2;
		     break;
		case 0x2182:		 //Y29-Y28
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT54);
			 LCD_Current_Page = 2;
		     break;

		case 0x2183:		 //Y30-Y31
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT55);
			 LCD_Current_Page = 2;
		     break;
		case 0x2184:		 //Y31-Y30
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT56);
			 LCD_Current_Page = 2;
		     break;

 		case 0x2185:		 //Y32-Y33
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT57);
			 LCD_Current_Page = 2;
		     break;
		case 0x2186:		 //Y33-Y32
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT58);
			 LCD_Current_Page = 2;
		     break;

		case 0x2187:		 //Y34-Y35
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT59);
			 LCD_Current_Page = 2;
		     break;
		case 0x2188:		 //Y35-Y34
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_OUTPUT60);
			 LCD_Current_Page = 2;
		     break;

		case 0x2189:		 //
		     break;
		case 0x218A:		 //
		     break;
		case 0x218B:		 //
		     break;
		case 0x218C:		 //
		     break;


   //输入检测
		case 0x218D:		 //X30
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_INPUT6);
			 LCD_Current_Page = 2;
		     break;

		case 0x218E:		 //X31
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_INPUT7);
			 LCD_Current_Page = 2;
		     break;

		case 0x218F:		 //X32
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_INPUT1);
			 LCD_Current_Page = 2;
		     break;

		case 0x2190:		 //X33
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_INPUT2);
			 LCD_Current_Page = 2;
		     break;

		case 0x2191:		 //X34
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_INPUT3);
			 LCD_Current_Page = 2;
		     break;

		case 0x2192:		 //X35
		     ProcessProgram(OR_IOORDER,K_IOINSTRUCT_INPUT4);
			 LCD_Current_Page = 2;
		     break;

 		case 0x2193:		 //
		     break;
 		case 0x2194:		 //
		     break;
 		case 0x2195:		 //
		     break;
 		case 0x2196:		 //
		     break;
	   default:
	         break;

	}
}


