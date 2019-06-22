/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ****************
* File Name          : 
* Author             : 
* Version            : 
* Date               : 
* Description        : 
******************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "DGUS.h"
#include "parameter.h"
#include "freeprogramme.h"
#include "delay.h"
#include "manuloperate.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
static PageCut Backgroud_Color[9] = {
									  {0x2300,21,136 ,95,650,128,136 ,95 }, //第一行
									  {0x2300,21,136 ,95,650,128,136 ,135}, //第二行
									  {0x2300,21,136 ,95,650,128,136 ,175}, //第三行
									  {0x2300,21,136 ,95,650,128,136 ,215}, //第四行
									  {0x2300,21,136 ,95,650,128,136 ,255}, //第五行
									  {0x2300,21,136 ,95,650,128,136 ,295}, //第六行
									  {0x2300,21,136 ,95,650,128,136 ,335}, //第七行
									  {0x2300,21,136 ,95,650,128,136 ,375}, //第八行
									  {0x2300,21,136 ,95,650,128,136 ,415}  //第九行
									};  	
static PageCut Backgroud_Color_FreeProgram[8] = {
									  {0x2300,21,18 ,375,650,410,18 ,135}, //第1行
									  {0x2300,21,18 ,375,650,410,18 ,175}, //第2行
									  {0x2300,21,18 ,375,650,410,18 ,215}, //第3行
									  {0x2300,21,18 ,375,650,410,18 ,255}, //第4行
									  {0x2300,21,18 ,375,650,410,18 ,295}, //第5行
									  {0x2300,21,18 ,375,650,410,18 ,335}, //第6行
									  {0x2300,21,18 ,375,650,410,18 ,375}, //第7行
									  {0x2300,21,18 ,375,650,410,18 ,415}  //第8行
									};

//u8 gs_Current_Program_Num = 0;         //当前编程的程序条数
//u8 g_Current_Program_Num_Temp = 0;     //当前编程的程序条数缓存，用于--
//FreeProgram Current_Process_Program[LARGESTPROGRAMNUM]={0}; //当前编程的100条指令，如果直接从保存程序读取的话，则直接将保存程序的对应值赋值

static u8 gs_Current_Dispaly_List[8]={0};//当前显示的8条指令

static u8 gs_Current_Program_Firstline=0;//显示框第一条指令的行数
u8 g_Refresh_Dispaly_List_Flag=FALSE;	 //刷新程序显示列表
u8 gs_Parameter_Unset = FALSE;			 //程序参数未设置

u8  Point_String[6][8]={
							{"        "},	  //0
							{"  点胶  "},	  //1
							{"  点钻  "},	  //2
							{"  取料  "},		//3
							{"  延时  "},	  //4
							{"  空移  "}		//5
};
u8 LC_String[11][6]={
							{"      "},	  //0
							{"料仓1 "},	  //1
							{"料仓2 "},	  //2
							{"料仓3 "},	  //3
							{"料仓4 "},		//4
							{"料仓5 "},		//5
							{"料仓6 "},		//6
							{"料仓7 "},		//7
							{"料仓8 "},		//8
							{"料仓9 "},		//9
							{"料仓10"}		//10
};
u8  Program_String[87][16]={	
							{"             "},	  //0
							{"点胶         "},	  //	  
							{"点钻         "},	  //2
							{"取料         "},
							{"联动模式     "},	  //4 
							{"寸动模式     "},
							{"启动动作     "},	  //6
							{"堆叠完成动作 "},
							{""},{""},{" "},{" "},
							{"垂直轴位置   "},	  //12
							{"料仓轴位置   "},	  //13
							{"上料仓位置   "},    //14
							{"卸料仓位置   "},	  //15
							{"X轴-移动     "},	  //16
							{"Z轴-移动     "},	  //17
							{"Y轴-移动     "},	  //18
							{"O轴-移动     "},	  //19

							//输出指令
						    //-----单阀-----//
							//20--指令1  ~~~  27--指令8
							{"Y0置位       "},{"Y0复位       "},{"Y1置位       "},{"Y1复位       "},
							{"Y2置位       "},{"Y2复位       "},{"Y3置位       "},{"Y3复位       "},
						    //28--指令9  ~~~  35--指令16
							{"Y4置位       "},{"Y4复位       "},{"Y5置位       "},{"Y5复位       "},
							{"Y6置位       "},{"Y6复位       "},{"Y7置位       "},{"Y7复位       "},
							//36--指令17 ~~~  43--指令24
							{"Y8置位       "},{"Y8复位       "},{"Y9置位       "},{"Y9复位       "},
							{"Y10置位      "},{"Y10复位      "},{"Y11置位      "},{"Y11复位      "},
							//44--指令25 ~~~  51--指令32
							{"Y12置位      "},{"Y12复位      "},{"Y13置位      "},{"Y13复位      "},
							{"Y14置位      "},{"Y14复位      "},{"Y15置位      "},{"Y15复位      "},
							//52--指令33 ~~~  59--指令40
							{"Y16置位      "},{"Y16复位      "},{"Y17置位      "},{"Y17复位      "},
							{"Y18置位      "},{"Y18复位      "},{"Y19置位      "},{"Y19复位      "},
							
							//预留切换区
							{" "},{" "},{" "},{" "},//输出指令41--44

							//-----双阀-----//
							//64--指令45~~~  71--指令52
							{"Y20置-Y21复位"},{"Y21置-Y20复位"},{"Y22置-Y23复位"},{"Y23置-Y22复位"},
							{"Y24置-Y25复位"},{"Y25置-Y24复位"},{"Y26置-Y27复位"},{"Y27置-Y26复位"},
							//72--指令53~~~  79--指令60
							{"Y28置-Y29复位"},{"Y29置-Y28复位"},{"Y30置-Y31复位"},{"Y31置-Y30复位"},
							{"Y32置-Y33复位"},{"Y33置-Y32复位"},{"Y34置-Y35复位"},{"Y35置-Y34复位"},

							//输入指令
							{"X32-输入检测 "}, //80--输入指令1
							{"X33-输入检测 "},	  //	  
							{"X34-输入检测 "},	  
							{"X35-输入检测 "},
							{"X36-输入检测 "}, 
							{"X30-输入检测 "}, 
							{"X31-输入检测 "} //86--输入指令7
					      };

u8  SafeArea_String[14][16]={	
							{"机床安全区域1始"},		  
						   	{"机床安全区域1末"},
							{"机床安全区域2始"},		  
						   	{"机床安全区域2末"},
							{"料仓安全区域始 "},		  
						   	{"料仓安全区域末 "},
							{"其他安全区域1始"},		  
						   	{"其他安全区域1末"},
							{"其他安全区域2始"},		  
						   	{"其他安全区域2末"},
							{"其他安全区域3始"},		  
						   	{"其他安全区域3末"},
							{"其他安全区域4始"},		  
						   	{"其他安全区域4末"}
						   };

u8  Parameter_String[24][10]={	
								{"          "},  //0
								{"  取料完成"},	//1	  
							   	{"  允许取料"},	//2
								{"  卸料完成"},	//3	  
							   	{"  允许卸料"},	//4
								{"  延时时间"},	//5	  
							   	{"  移动坐标"},	//6
								{"     速度 "},	//7
								{" 检测 : 开"},	//8	  
							   	{" 检测 : 关"},	//9
								{"      0.00"},	//10
								{"   X轴坐标"},	//11
								{"   Y轴坐标"},	//12
								{"  放置料盘"},	//13
								{"  钩取料盘"},	//14
								{"  脉冲信号"},	//15
								{"  程序开始"},	//16
								{"  程序结束"},	//17
								{"  单次执行"}, //18						
								{"   Z轴坐标"},	//19
								{"  条件执行"},	//20
								{"  默认执行"},	//21
								{"  动作执行"},	//22
								{"  动作确认"}	//23
						    };

u8  gs_Current_Dispaly_Code[8]={0};	     //当前显示的8个编码
u32  gs_Code_Parameter_Display[8][4]={0};//编码命令对应的两位参数及显示
u16  gs_code_num[8][4] = {0};
u8 gs_name[8][4] = {0};
u32 gs_time[8] = {0};


u8  ProgramCode_String[1001][4]={{"   "},
								{"1: "},{"2: "},{"3: "},{"4: "},{"5: "},{"6: "},{"7: "},{"8: "},{"9: "},{"10:"},
								{"11:"},{"12:"},{"13:"},{"14:"},{"15:"},{"16:"},{"17:"},{"18:"},{"19:"},{"20:"},
								{"21:"},{"22:"},{"23:"},{"24:"},{"25:"},{"26:"},{"27:"},{"28:"},{"29:"},{"30:"},
								{"31:"},{"32:"},{"33:"},{"34:"},{"35:"},{"36:"},{"37:"},{"38:"},{"39:"},{"40:"},
								{"41:"},{"42:"},{"43:"},{"44:"},{"45:"},{"46:"},{"47:"},{"48:"},{"49:"},{"50:"},
								{"51:"},{"52:"},{"53:"},{"54:"},{"55:"},{"56:"},{"57:"},{"58:"},{"59:"},{"60:"},
								{"61:"},{"62:"},{"63:"},{"64:"},{"65:"},{"66:"},{"67:"},{"68:"},{"69:"},{"70:"},
								{"71:"},{"72:"},{"73:"},{"74:"},{"75:"},{"76:"},{"77:"},{"78:"},{"79:"},{"80:"},
								{"81:"},{"82:"},{"83:"},{"84:"},{"85:"},{"86:"},{"87:"},{"88:"},{"89:"},{"90:"},
								{"91:"},{"92:"},{"93:"},{"94:"},{"95:"},{"96:"},{"97:"},{"98:"},{"99:"},{"100:"},
								{"101:"},{"102:"},{"103:"},{"104:"},{"105:"},{"106:"},{"107:"},{"108:"},{"109:"},{"110:"},
								{"111:"},{"112:"},{"113:"},{"114:"},{"115:"},{"116:"},{"117:"},{"118:"},{"119:"},{"120:"},
								{"121:"},{"122:"},{"123:"},{"124:"},{"125:"},{"126:"},{"127:"},{"128:"},{"129:"},{"130:"},
								{"131:"},{"132:"},{"133:"},{"134:"},{"135:"},{"136:"},{"137:"},{"138:"},{"139:"},{"140:"},
								{"141:"},{"142:"},{"143:"},{"144:"},{"145:"},{"146:"},{"147:"},{"148:"},{"149:"},{"150:"},
								{"151:"},{"152:"},{"153:"},{"154:"},{"155:"},{"156:"},{"157:"},{"158:"},{"159:"},{"160:"},
								{"161:"},{"162:"},{"163:"},{"164:"},{"165:"},{"166:"},{"167:"},{"168:"},{"169:"},{"170:"},
								{"171:"},{"172:"},{"173:"},{"174:"},{"175:"},{"176:"},{"177:"},{"178:"},{"179:"},{"180:"},
								{"181:"},{"182:"},{"183:"},{"184:"},{"185:"},{"186:"},{"187:"},{"188:"},{"189:"},{"190:"},
								{"191:"},{"192:"},{"193:"},{"194:"},{"195:"},{"196:"},{"197:"},{"198:"},{"199:"},{"200:"},	
								{"201:"},{"202:"},{"203:"},{"204:"},{"205:"},{"206:"},{"207:"},{"208:"},{"209:"},{"210:"},
								{"211:"},{"212:"},{"213:"},{"214:"},{"215:"},{"216:"},{"217:"},{"218:"},{"219:"},{"220:"},
								{"221:"},{"222:"},{"223:"},{"224:"},{"225:"},{"226:"},{"227:"},{"228:"},{"229:"},{"230:"},
								{"231:"},{"232:"},{"233:"},{"234:"},{"235:"},{"236:"},{"237:"},{"238:"},{"239:"},{"240:"},
								{"241:"},{"242:"},{"243:"},{"244:"},{"245:"},{"246:"},{"247:"},{"248:"},{"249:"},{"250:"},
								{"251:"},{"252:"},{"253:"},{"254:"},{"255:"},{"256:"},{"257:"},{"258:"},{"259:"},{"260:"},
								{"261:"},{"262:"},{"263:"},{"264:"},{"265:"},{"266:"},{"267:"},{"268:"},{"269:"},{"270:"},
								{"271:"},{"272:"},{"273:"},{"274:"},{"275:"},{"276:"},{"277:"},{"278:"},{"279:"},{"280:"},
								{"281:"},{"282:"},{"283:"},{"284:"},{"285:"},{"286:"},{"287:"},{"288:"},{"289:"},{"290:"},
								{"291:"},{"292:"},{"293:"},{"294:"},{"295:"},{"296:"},{"297:"},{"298:"},{"299:"},{"300:"},	
								{"301:"},{"302:"},{"303:"},{"304:"},{"305:"},{"306:"},{"307:"},{"308:"},{"309:"},{"310:"},
								{"311:"},{"312:"},{"313:"},{"314:"},{"315:"},{"316:"},{"317:"},{"318:"},{"319:"},{"320:"},
								{"321:"},{"322:"},{"323:"},{"324:"},{"325:"},{"326:"},{"327:"},{"328:"},{"329:"},{"330:"},
								{"331:"},{"332:"},{"333:"},{"334:"},{"335:"},{"336:"},{"337:"},{"338:"},{"339:"},{"340:"},
								{"341:"},{"342:"},{"343:"},{"344:"},{"345:"},{"346:"},{"347:"},{"348:"},{"349:"},{"350:"},
								{"351:"},{"352:"},{"353:"},{"354:"},{"355:"},{"356:"},{"357:"},{"358:"},{"359:"},{"360:"},
								{"361:"},{"362:"},{"363:"},{"364:"},{"365:"},{"366:"},{"367:"},{"368:"},{"369:"},{"370:"},
								{"371:"},{"372:"},{"373:"},{"374:"},{"375:"},{"376:"},{"377:"},{"378:"},{"379:"},{"380:"},
								{"381:"},{"382:"},{"383:"},{"384:"},{"385:"},{"386:"},{"387:"},{"388:"},{"389:"},{"390:"},
								{"391:"},{"392:"},{"393:"},{"394:"},{"395:"},{"396:"},{"397:"},{"398:"},{"399:"},{"400:"},	
								{"401:"},{"402:"},{"403:"},{"404:"},{"405:"},{"406:"},{"407:"},{"408:"},{"409:"},{"410:"},
								{"411:"},{"412:"},{"413:"},{"414:"},{"415:"},{"416:"},{"417:"},{"418:"},{"419:"},{"420:"},
								{"421:"},{"422:"},{"423:"},{"424:"},{"425:"},{"426:"},{"427:"},{"428:"},{"429:"},{"430:"},
								{"431:"},{"432:"},{"433:"},{"434:"},{"435:"},{"436:"},{"437:"},{"438:"},{"439:"},{"440:"},
								{"441:"},{"442:"},{"443:"},{"444:"},{"445:"},{"446:"},{"447:"},{"448:"},{"449:"},{"450:"},
								{"451:"},{"452:"},{"453:"},{"454:"},{"455:"},{"456:"},{"457:"},{"458:"},{"459:"},{"460:"},
								{"461:"},{"462:"},{"463:"},{"464:"},{"465:"},{"466:"},{"467:"},{"468:"},{"469:"},{"470:"},
								{"471:"},{"472:"},{"473:"},{"474:"},{"475:"},{"476:"},{"477:"},{"478:"},{"479:"},{"480:"},
								{"481:"},{"482:"},{"483:"},{"484:"},{"485:"},{"486:"},{"487:"},{"488:"},{"489:"},{"490:"},
								{"491:"},{"492:"},{"493:"},{"494:"},{"495:"},{"496:"},{"497:"},{"498:"},{"499:"},{"500:"},	
								{"501:"},{"502:"},{"503:"},{"504:"},{"505:"},{"506:"},{"507:"},{"508:"},{"509:"},{"510:"},
								{"511:"},{"512:"},{"513:"},{"514:"},{"515:"},{"516:"},{"517:"},{"518:"},{"519:"},{"520:"},
								{"521:"},{"522:"},{"523:"},{"524:"},{"525:"},{"526:"},{"527:"},{"528:"},{"529:"},{"530:"},
								{"531:"},{"532:"},{"533:"},{"534:"},{"535:"},{"536:"},{"537:"},{"538:"},{"539:"},{"540:"},
								{"541:"},{"542:"},{"543:"},{"544:"},{"545:"},{"546:"},{"547:"},{"548:"},{"549:"},{"550:"},
								{"551:"},{"552:"},{"553:"},{"554:"},{"555:"},{"556:"},{"557:"},{"558:"},{"559:"},{"560:"},
								{"561:"},{"562:"},{"563:"},{"564:"},{"565:"},{"566:"},{"567:"},{"568:"},{"569:"},{"570:"},
								{"571:"},{"572:"},{"573:"},{"574:"},{"575:"},{"576:"},{"577:"},{"578:"},{"579:"},{"580:"},
								{"581:"},{"582:"},{"583:"},{"584:"},{"585:"},{"586:"},{"587:"},{"588:"},{"589:"},{"590:"},
								{"591:"},{"592:"},{"593:"},{"594:"},{"595:"},{"596:"},{"597:"},{"598:"},{"599:"},{"600:"},	
								{"601:"},{"602:"},{"603:"},{"604:"},{"605:"},{"606:"},{"607:"},{"608:"},{"609:"},{"710:"},
								{"611:"},{"612:"},{"613:"},{"614:"},{"615:"},{"616:"},{"617:"},{"618:"},{"619:"},{"720:"},
								{"621:"},{"622:"},{"623:"},{"624:"},{"625:"},{"626:"},{"627:"},{"628:"},{"629:"},{"730:"},
								{"631:"},{"632:"},{"633:"},{"634:"},{"635:"},{"636:"},{"637:"},{"638:"},{"639:"},{"740:"},
								{"641:"},{"642:"},{"643:"},{"644:"},{"645:"},{"646:"},{"647:"},{"648:"},{"649:"},{"750:"},
								{"651:"},{"652:"},{"653:"},{"654:"},{"655:"},{"656:"},{"657:"},{"658:"},{"659:"},{"760:"},
								{"661:"},{"662:"},{"663:"},{"664:"},{"665:"},{"666:"},{"667:"},{"668:"},{"669:"},{"770:"},
								{"671:"},{"672:"},{"673:"},{"674:"},{"675:"},{"676:"},{"677:"},{"678:"},{"679:"},{"780:"},
								{"681:"},{"682:"},{"683:"},{"684:"},{"685:"},{"686:"},{"687:"},{"688:"},{"689:"},{"790:"},
								{"691:"},{"692:"},{"693:"},{"694:"},{"695:"},{"696:"},{"697:"},{"698:"},{"699:"},{"700:"},	
								{"701:"},{"702:"},{"703:"},{"704:"},{"705:"},{"706:"},{"707:"},{"708:"},{"709:"},{"710:"},
								{"711:"},{"712:"},{"713:"},{"714:"},{"715:"},{"716:"},{"717:"},{"718:"},{"719:"},{"720:"},
								{"721:"},{"722:"},{"723:"},{"724:"},{"725:"},{"726:"},{"727:"},{"728:"},{"729:"},{"730:"},
								{"731:"},{"732:"},{"733:"},{"734:"},{"735:"},{"736:"},{"737:"},{"738:"},{"739:"},{"740:"},
								{"741:"},{"742:"},{"743:"},{"744:"},{"745:"},{"746:"},{"747:"},{"748:"},{"749:"},{"750:"},
								{"751:"},{"752:"},{"753:"},{"754:"},{"755:"},{"756:"},{"757:"},{"758:"},{"759:"},{"760:"},
								{"761:"},{"762:"},{"763:"},{"764:"},{"765:"},{"766:"},{"767:"},{"768:"},{"769:"},{"770:"},
								{"771:"},{"772:"},{"773:"},{"774:"},{"775:"},{"776:"},{"777:"},{"778:"},{"779:"},{"780:"},
								{"781:"},{"782:"},{"783:"},{"784:"},{"785:"},{"786:"},{"787:"},{"788:"},{"789:"},{"790:"},
								{"791:"},{"792:"},{"793:"},{"794:"},{"795:"},{"796:"},{"797:"},{"798:"},{"719:"},{"800:"},
								{"801:"},{"802:"},{"803:"},{"804:"},{"805:"},{"806:"},{"807:"},{"808:"},{"809:"},{"810:"},
								{"811:"},{"812:"},{"813:"},{"814:"},{"815:"},{"816:"},{"817:"},{"818:"},{"819:"},{"820:"},
								{"821:"},{"822:"},{"823:"},{"824:"},{"825:"},{"826:"},{"827:"},{"828:"},{"829:"},{"830:"},
								{"831:"},{"832:"},{"833:"},{"834:"},{"835:"},{"836:"},{"837:"},{"838:"},{"839:"},{"840:"},
								{"841:"},{"842:"},{"843:"},{"844:"},{"845:"},{"846:"},{"847:"},{"848:"},{"849:"},{"850:"},
								{"851:"},{"852:"},{"853:"},{"854:"},{"855:"},{"856:"},{"857:"},{"858:"},{"859:"},{"860:"},
								{"861:"},{"862:"},{"863:"},{"864:"},{"865:"},{"866:"},{"867:"},{"868:"},{"869:"},{"870:"},
								{"871:"},{"872:"},{"873:"},{"874:"},{"875:"},{"876:"},{"877:"},{"878:"},{"879:"},{"880:"},
								{"881:"},{"882:"},{"883:"},{"884:"},{"885:"},{"886:"},{"887:"},{"888:"},{"889:"},{"890:"},
								{"891:"},{"892:"},{"893:"},{"894:"},{"895:"},{"896:"},{"897:"},{"898:"},{"899:"},{"900:"},
								{"901:"},{"902:"},{"903:"},{"904:"},{"905:"},{"906:"},{"907:"},{"908:"},{"909:"},{"910:"},
								{"911:"},{"912:"},{"913:"},{"914:"},{"915:"},{"916:"},{"917:"},{"918:"},{"919:"},{"920:"},
								{"921:"},{"922:"},{"923:"},{"924:"},{"925:"},{"926:"},{"927:"},{"928:"},{"929:"},{"930:"},
								{"931:"},{"932:"},{"933:"},{"934:"},{"935:"},{"936:"},{"937:"},{"938:"},{"939:"},{"940:"},
								{"941:"},{"942:"},{"943:"},{"944:"},{"945:"},{"946:"},{"947:"},{"948:"},{"949:"},{"950:"},
								{"951:"},{"952:"},{"953:"},{"954:"},{"955:"},{"956:"},{"957:"},{"958:"},{"959:"},{"960:"},
								{"961:"},{"962:"},{"963:"},{"964:"},{"965:"},{"966:"},{"967:"},{"968:"},{"969:"},{"970:"},
								{"971:"},{"972:"},{"973:"},{"974:"},{"975:"},{"976:"},{"977:"},{"978:"},{"979:"},{"980:"},
								{"981:"},{"982:"},{"983:"},{"984:"},{"985:"},{"986:"},{"987:"},{"988:"},{"989:"},{"990:"},
								{"991:"},{"992:"},{"993:"},{"994:"},{"995:"},{"996:"},{"997:"},{"998:"},{"999:"},								
							  };


u8 Parameter_Value[10]={0};

u8 g_Current_Program_Debug=FALSE;			 //当前程序是否可以调试
u8 g_Program_Is_Debuging=FALSE;			     //程序调试中,所有按键失效
u8 g_USART1_SendProgram_Flag=FALSE;			 //标志开始发送程序
u8 g_Current_Program_Modify=FALSE; 			 //修改
static u8 gs_Current_Program_Modify=TRUE;	 //调试
static PageCut Free_Program_Modify={0x2320,100,691,270,784,312,691,270};
static u8 gs_Refresh_Speed_Input = FALSE;	 //参数-速度刷新显示
static u8 gs_Refresh_Delay_Input = FALSE;	 //参数-时间刷新显示
//static u8 gs_LDAction_flag = 0;	             //联动动作开始结束标识位
static u8 gs_Load_LCMark_flag = 0;	             //上料仓注释
static u8 gs_UnLoad_LCMark_flag = 0;	         //卸料仓注释
static u8 gs_Load_Program_flag = 0;	             //上料仓程序
static u8 gs_UnLoad_Program_flag = 0;	         //卸料仓程序
static u8 gs_OAxis_Position_flag = 0;	         //O轴位置



u16 Current_Select_Program = 0;           //当前选中的程序在数组中的位置编号-1
u8  Select_Program_Flag = 1;              //选中程序页面刷新标志位

u8 name_temp[4]={0};                      //命名临时变量
u32 name_save=0;		
u16 Same_Name_Num=0;											//重名程序对应序列
u16 Current_Program_Adress=0;
u32  Point_Parameter_Display[8][5]={0};		 //编码命令对应的两位参数及显示
u8 Point_Action_Flag=0;										//点编辑时对应点编号
u8 Program_Save_OrNot=0;									
//程序是否修改保存标志位  1程序保存 2程序不保存 3程序载入 4点示教载入 5示教保存  6示教不保存
u8 Program_Edit_OrNot=0;									//程序是否修改  1修改 0未修改
u8 Program_Transport_Flag=0;							//程序传输标志位
u8 Program_Transport_Count=0;							//程序传输计时
u8 Click_Count=0,Click_Count1=0;													//界面敲击计数（用于双击进入自由编程）
/********************************指令选择标志位********************************************************/
u8 DianJiao_Choose=1,DianZuan_Choose=0,QuLiao_Choose=0,Order_Type=1;//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
/*********************************IO选择标志位*********************************************************/																			
u8 ChuJiao1=0,ChuJiao2=0,ChuJiao3=0,JiaoTouXiaYi1=0,JiaoTouXiaYi2=0,JiaoTouXiaYi3=0,
XiZuan1=0,XiZuan2=0,XiZuan3=0,XiZuan4=0,XiZuan5=0,XiZuan6=0,XiZuan7=0,XiZuan8=0,
ZuanTouXiaYi1=0,ZuanTouXiaYi2=0,ZuanTouXiaYi3=0,ZuanTouXiaYi4=0,ZuanTouXiaYi5=0,ZuanTouXiaYi6=0,ZuanTouXiaYi7=0,ZuanTouXiaYi8=0;
u32 IO_Choose=0;//高八位控制胶头或者钻头下移  低八位控制出胶和吸放钻
u16 Delay_Time=1;//延时指令默认1ms
/********************************料仓选择标志位********************************************************/
u8 Now_LC_Choose_1=1,Now_LC_Choose_2=0,Now_LC_Choose_3=0,Now_LC_Choose_4=0,Now_LC_Choose_5=0,Now_LC_Choose_6=0,
Now_LC_Choose_7=0,Now_LC_Choose_8=0,Now_LC_Choose_9=0,Now_LC_Choose_10=0;
u8 Oeder_Edit_Flag=0;
u8 Link_Step_Refresh=1;
extern u8 LC_ID,Parameter_Refresh,Display_Flag;
extern u8 g_Run_Program_Num;
extern u32 g_Run_Program_Name;
extern u8 Product_Num,Program_Flag[SAVEPROGRAMNUM];
extern u32 seccount;
extern u8 Program_Or_Point;
extern u8 Point_Num,Order_Num;
extern u32 LC_IO_Axis_Code;
extern u8 gs_Background_Color,gs_Current_Program_List;
extern u8 gs_Current_Operate_List_PointEdit,g_Background_Color_PointEdit,gs_Background_Color_PointEdit;
extern u8 g_Background_Color_PointEdit,gs_Background_Color_PointEdit,gs_Current_Operate_List_PointEdit;
extern u8 g_Background_Color_ProgramEdit,gs_Background_Color_ProgramEdit,gs_Current_Operate_List_ProgramEdit;
extern u16 gs_Run_Program_Code[8];
extern u8 gs_Run_Program_List[9];
extern u8 Link_Step_Flag;//寸动联动模式
extern u8 LCD_Main_Page_Pre;
/*************************************************************************
**  函数名：  CurrentProcessProgramValue()
**	输入参数：
**	输出参数：无
**	函数功能：指令对应参数值
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void CurrentProcessProgramValue(u8 Num,u8 Key)
{
		//延时
		if(Key==K_DELAY)
		{
			Current_Operate_Program.Program[Num].Value1=0;
			Current_Operate_Program.Program[Num].Value2=5;
			Current_Operate_Program.Program[Num].Value3=0;		    
		}
		//上料仓程序
		else if(Key==K_LOADPROGRAM)
		{
			if(gs_Load_Program_flag)
			{
			    Current_Operate_Program.Program[Num].Value1=2;
				gs_Load_Program_flag=0;
			}
			else
			{
			    Current_Operate_Program.Program[Num].Value1=1;
				gs_Load_Program_flag=1;
			}	
			Current_Operate_Program.Program[Num].Value2=0;
			Current_Operate_Program.Program[Num].Value3=0;
		}
		//卸料仓程序
		else if(Key==K_UNLOADPROGRAM)
		{
			if(gs_UnLoad_Program_flag)
			{
			    Current_Operate_Program.Program[Num].Value1=4;
				gs_UnLoad_Program_flag=0;
			}
			else
			{
			    Current_Operate_Program.Program[Num].Value1=3;
				gs_UnLoad_Program_flag=1;
			}	
			Current_Operate_Program.Program[Num].Value2=0;
			Current_Operate_Program.Program[Num].Value3=0;		
		}
		//启动单次运行
		else if(Key==K_STARTACTION)
		{
			Current_Operate_Program.Program[Num].Value1=18;
			Current_Operate_Program.Program[Num].Value2=0;
			Current_Operate_Program.Program[Num].Value3=0;	
		}
		//堆叠完成动作
		else if(Key==K_OAXISRESET)
		{
			Current_Operate_Program.Program[Num].Value1=20;
			Current_Operate_Program.Program[Num].Value2=22;
			Current_Operate_Program.Program[Num].Value3=0;
		}
		//轴指令
		else if(K_XAXIS<=Key && Key<=K_OAXIS)
		{
			Current_Operate_Program.Program[Num].Value1=6;
			Current_Operate_Program.Program[Num].Value2=7;
			Current_Operate_Program.Program[Num].Value3=5;
		}
		//垂直轴位置
		else if(Key==K_ZAXISPOSITION)
		{
			Current_Operate_Program.Program[Num].Value1=6;
			Current_Operate_Program.Program[Num].Value2=7;
			Current_Operate_Program.Program[Num].Value3=19;	    
		}
		//料仓轴位置
		else if(Key==K_OAXISPOSITION)
		{
			Current_Operate_Program.Program[Num].Value1=6;
			Current_Operate_Program.Program[Num].Value2=7;
			if(gs_OAxis_Position_flag)
			{
			    Current_Operate_Program.Program[Num].Value3=14;
				gs_OAxis_Position_flag=0;	
			}
			else
			{
			    Current_Operate_Program.Program[Num].Value3=13;
				gs_OAxis_Position_flag=1;			
			}	    
		}
		//上料仓位置
		else if(Key==K_LOADPOSITION)
		{
			Current_Operate_Program.Program[Num].Value1=6;
			Current_Operate_Program.Program[Num].Value2=7;
			if(gs_Load_LCMark_flag)
			{
			    Current_Operate_Program.Program[Num].Value3=12;
				gs_Load_LCMark_flag=0;
			}
			else
			{
			    Current_Operate_Program.Program[Num].Value3=11;
				gs_Load_LCMark_flag=1;
			}						    
		}
		//卸料仓位置
		else if(Key==K_UNLOADPOSITION)
		{
			Current_Operate_Program.Program[Num].Value1=6;
			Current_Operate_Program.Program[Num].Value2=7;
			if(gs_UnLoad_LCMark_flag)
			{
			    Current_Operate_Program.Program[Num].Value3=12;
				gs_UnLoad_LCMark_flag=0;
			}
			else
			{
			    Current_Operate_Program.Program[Num].Value3=11;
				gs_UnLoad_LCMark_flag=1;
			}						    
		}

		//IO指令-输出
		else if(K_IOINSTRUCT_OUTPUT1<=Key && Key<=K_IOINSTRUCT_OUTPUT60)//(0x09<=Key && Key<=0x2C)
		{
			Current_Operate_Program.Program[Num].Value1=0;
			Current_Operate_Program.Program[Num].Value2=8;
			Current_Operate_Program.Program[Num].Value3=5;	
//			if(Key==K_IOINSTRUCT_OUTPUT23)		//机床启动-脉冲信号	 送料带
//			{
//				Current_Operate_Program.Program[Num].Value1=15;
//				Current_Operate_Program.Program[Num].Value2=9;
//				Current_Operate_Program.Program[Num].Value3=0;
//			}	
		}
		//IO指令-输入
		else if(K_IOINSTRUCT_INPUT1<=Key && Key<=K_IOINSTRUCT_INPUT7)//
		{
			Current_Operate_Program.Program[Num].Value1=0;
			Current_Operate_Program.Program[Num].Value2=8;
			Current_Operate_Program.Program[Num].Value3=5;		
		}
		else
		{
			Current_Operate_Program.Program[Num].Value1=0;
			Current_Operate_Program.Program[Num].Value2=0;
			Current_Operate_Program.Program[Num].Value3=0;
		}
}

/*************************************************************************
**  函数名：  ProcessProgram()
**	输入参数：
**	输出参数：无
**	函数功能：处理点击按钮时程序
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void ProcessProgram(u8 Order, u8 Key)
{
		 if(gs_Current_Operate_List>Current_Operate_Program.Num)	//新的指令
		 {
			 if(Current_Operate_Program.Program[gs_Current_Operate_List-1].Flag==0)
			 {
				 Current_Operate_Program.Program[Current_Operate_Program.Num].Flag=1;
				 if(Current_Operate_Program.Num==0)	  //第一条指令
				 {
				 	 Current_Operate_Program.Program[Current_Operate_Program.Num].List=1;	//编号
				 }
				 else								 //不是第一条
				 {
				     Current_Operate_Program.Program[Current_Operate_Program.Num].List=Current_Operate_Program.Program[Current_Operate_Program.Num-1].List+1;	//编号
				 }
				 Current_Operate_Program.Program[Current_Operate_Program.Num].Order = Order;
				 Current_Operate_Program.Program[Current_Operate_Program.Num].Key = Key;	
				 CurrentProcessProgramValue(Current_Operate_Program.Num,Key);   			 			
				 Current_Operate_Program.Num++;
				 if(gs_Current_Operate_List<LARGESTPROGRAMNUM)	//如果程序指令没有超过最大限度才++
				 {
				 	gs_Current_Operate_List++;
				 }
			 }
			 if(g_Background_Color<8)
			 {
			 	g_Background_Color++;
			 }
		 }
		 else												 //插入的指令
		 {
			 if(Current_Operate_Program.Program[gs_Current_Operate_List-1].Flag==0)
			 {
				 Current_Operate_Program.Program[gs_Current_Operate_List-1].Flag=1;
				 Current_Operate_Program.Program[gs_Current_Operate_List-1].Order = Order;
				 Current_Operate_Program.Program[gs_Current_Operate_List-1].Key = Key;
				 CurrentProcessProgramValue(gs_Current_Operate_List-1,Key);	
			 }	 		 
		 }
		 g_Refresh_Dispaly_List_Flag = TRUE;	 
}


/*************************************************************************
**  函数名：  Modify_Order()
**	输入参数：
**	输出参数：无
**	函数功能：修改指令
**	备注：	  无
**  作者：    White_L   
**  开发日期：2018/10/11
**************************************************************************/
void Modify_Order(void)
{
	
}
/*************************************************************************
**  函数名：  Delete_Order()
**	输入参数：
**	输出参数：无
**	函数功能：删除指令
**	备注：	  无
**  作者：    White_L   
**  开发日期：2018/10/11
**************************************************************************/
void Delete_Order(void)
{
	u16 i=0,j=0,n=0,k=0;
	 if(gs_Current_Operate_List_ProgramEdit-1>Order_Num)
	 {
	 }
	 else
	 {
		//删除程序以后所有后面程序前移//
		if(Free_Program_Operate.Program[(gs_Current_Operate_List_ProgramEdit-1)%LARGESTPROGRAMNUM].List!=0)
		{
				Order_Num--;//指令数目减一
				Free_Program_Operate.Num=Order_Num;
				OperateProgram[gs_Current_Operate_List-1].Order_Num--;
				j=((gs_Current_Operate_List_ProgramEdit-1)/LARGESTPROGRAMNUM)*LARGESTPROGRAMNUM;//判断删除点位于哪一段104
				if(j>0)
				{
					if(Order_Num-j>LARGESTPROGRAMNUM)
						n = j+LARGESTPROGRAMNUM;
					else
						n = Order_Num; 
				}
				else
				{
						n = LARGESTPROGRAMNUM; 
				}
				//程序保存中
				LCD_Current_Page = 66;
				LCD_Pre_Page = LCD_Current_Page;
				PageChange(LCD_Pre_Page);
				for(i=gs_Current_Operate_List_ProgramEdit-1;i<Order_Num;i++)//所有指令前移
				{
					AT24CXX_Read(CURRENT_PROGRAM_ADR2+ORDER_LENTH*(i+1),IIC_Temp,ORDER_LENTH);
					IIC_Temp[0]=(i+1);
					IIC_Temp[1]=(i+1)>>8;
					AT24CXX_Write(CURRENT_PROGRAM_ADR2+ORDER_LENTH*i,IIC_Temp,ORDER_LENTH);
				}
			for(i=j;i<n;i++)
			 {
				 /*List Value1 2 3 4  5 LC_ID Key IO_Group Delay_Time*/
					AT24CXX_Read(CURRENT_PROGRAM_ADR2+i*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Flag  = 1;
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
					if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
					 {
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
					 }
					else if(4==IIC_Temp[26])//延时指令
					{
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
					}				 
			 }
		 if(i%LARGESTPROGRAMNUM!=0)
		 {
			for(k=i%LARGESTPROGRAMNUM;k<LARGESTPROGRAMNUM;k++)
			{
				 /*List Value1 2 3 4  5 LC_ID Key IO_Group Delay_Time*/
				Free_Program_Operate.Program[k].Flag  = 0;
				Free_Program_Operate.Program[k].List  = 0;
				Free_Program_Operate.Program[k].Value1= 0;
				Free_Program_Operate.Program[k].Value2= 0;
				Free_Program_Operate.Program[k].Value3= 0;
				Free_Program_Operate.Program[k].Value4= 0;	
				Free_Program_Operate.Program[k].Value5= 0;			 
				Free_Program_Operate.Program[k].LC_ID	= 0;	
				Free_Program_Operate.Program[k].Key   = 0;
				Free_Program_Operate.Program[k].IO_Group= 0;	
				Free_Program_Operate.Program[k].Delay_Time= 0;				 				
			 }
		 }
		}
//		if(g_Background_Color_ProgramEdit>Order_Num%LARGESTPROGRAMNUM)
//		{
//			g_Background_Color_ProgramEdit=Order_Num%LARGESTPROGRAMNUM;
			if(g_Background_Color_ProgramEdit==0)
				g_Background_Color_ProgramEdit=1;
//			gs_Current_Operate_List_ProgramEdit=Order_Num;
			if(gs_Current_Operate_List_ProgramEdit==0)
			{
				gs_Current_Operate_List_ProgramEdit=1;
			}
//		}
		g_Current_Program_Debug=FALSE;
		g_Refresh_Dispaly_List_Flag = TRUE;		 
		Program_Edit_OrNot=1;//程序修改标志位
		gs_Run_Program_Num=0;
	}						
	LCD_Current_Page = 8;	
}
/*************************************************************************
**  函数名：  Search_Order()
**	输入参数：
**	输出参数：无
**	函数功能：查询指令
**	备注：	  无
**  作者：    White_L   
**  开发日期：2018/10/11
**************************************************************************/
void Search_Order(void)
{
	u16 n=0,i=0,k=0,j=0;
	name_save = name_temp[0]<<24 | name_temp[1]<<16 | name_temp[2]<<8 | name_temp[3];
	if(name_save == 0x20202020)
	{
		LCD_Current_Page = 141;	
	}
	else
	{
		name_save=0;
		n=0;
		for(i=0;i<4;i++)
		{
			if(name_temp[i]>=0x30)
			{
				name_temp[i]=(name_temp[i]-0x30);
				n++;
			}
		}
		for(i=0;i<n;i++)
			name_save+=name_temp[i]*pow(10,n-1-i);//求得点的序号
		
		if(name_save>Order_Num)
			LCD_Current_Page = 142;	//不存在该指令
		else
		{
			gs_Current_Operate_List_ProgramEdit = name_save;
			g_Background_Color_ProgramEdit = name_save%8;
			if(g_Background_Color_ProgramEdit==0)
				g_Background_Color_ProgramEdit=8;
			g_Refresh_Dispaly_List_Flag = TRUE;	
			//判断name_save处在哪一个104段内
			name_save=(name_save/LARGESTPROGRAMNUM)*LARGESTPROGRAMNUM;
			if(Order_Num-name_save>LARGESTPROGRAMNUM)
				n=name_save+LARGESTPROGRAMNUM;
			else
				n=Order_Num;			
			for(i=name_save;i<n;i++)
			 {
				 /*List Value1 2 3 4  5 LC_ID Key IO_Group*/
					AT24CXX_Read(CURRENT_PROGRAM_ADR2+i*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Flag  = 1;
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
					Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
					 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
					 {
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
					 }
					else if(4==IIC_Temp[26])//延时指令
					{
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
					}				 
			 }
			 if(i%LARGESTPROGRAMNUM!=0)
			 {
				for(k=i%LARGESTPROGRAMNUM;k<LARGESTPROGRAMNUM;k++)
				{
					 /*List Value1 2 3 4  5 LC_ID Key IO_Group*/
					Free_Program_Operate.Program[k].Flag  = 0;
					Free_Program_Operate.Program[k].List  = 0;
					Free_Program_Operate.Program[k].Value1= 0;
					Free_Program_Operate.Program[k].Value2= 0;
					Free_Program_Operate.Program[k].Value3= 0;
					Free_Program_Operate.Program[k].Value4= 0;	
					Free_Program_Operate.Program[k].Value5= 0;			 
					Free_Program_Operate.Program[k].LC_ID	= 0;	
					Free_Program_Operate.Program[k].Key   = 0;
					Free_Program_Operate.Program[k].IO_Group= 0;										
				 }
			 }
		}
			g_Refresh_Dispaly_List_Flag = TRUE;
	}
}
/*************************************************************************
**  函数名：  Select_Product()
**	输入参数：
**	输出参数：无
**	函数功能：产品选择
**	备注：	  无
**  作者：    White_L   
**  开发日期：2018/10/11
**************************************************************************/
void Select_Product(void)
{
	 Select_Program_Flag = TRUE;
	 Current_Select_Program = gs_Current_Operate_List - 1;
	 if(g_Run_Program_Num == OperateProgram[Current_Select_Program].Code)
	 {
		 Order_Num = OperateProgram[Current_Select_Program].Order_Num;
		 LCD_Current_Page = 2;				 
	 }
	 else if(g_Run_Program_Num != OperateProgram[Current_Select_Program].Code)//选择运行程序用程序Code识别
	 {
		 g_Run_Program_Num  = OperateProgram[Current_Select_Program].Code;
		 g_Run_Program_Name = OperateProgram[Current_Select_Program].Name;
		 Free_Program_Operate.Flag = 1;
		 Free_Program_Operate.Code = g_Run_Program_Num;
		 Free_Program_Operate.Name = OperateProgram[Current_Select_Program].Name;
		 Free_Program_Operate.Num  = OperateProgram[Current_Select_Program].Order_Num;	
		 Current_Operate_Program.Flag  = 1;
		 Current_Operate_Program.Code  = g_Run_Program_Num;
		 Current_Operate_Program.Name  = OperateProgram[Current_Select_Program].Name;
		 Current_Operate_Program.Num   = OperateProgram[Current_Select_Program].Order_Num;	
		 g_Run_Program_List_Num 			 = Free_Program_Operate.Num;
		 g_Run_Program_Operater_List   = 0;
		 Order_Num                     = OperateProgram[Current_Select_Program].Order_Num;
		 Display_Flag=0;
		 //程序保存中
		 LCD_Current_Page = 66;
		 LCD_Pre_Page = LCD_Current_Page;
		 PageChange(LCD_Pre_Page);
		 Read_IIC_Done=1;
		 LCD_New_Order=FALSE;
		 ReadRTC();	
		 DelayNms(10);
		 IIC_Temp[0] =PROGRAM_INFO;
		 IIC_Temp[1] =1;//flag
		 IIC_Temp[2] =Free_Program_Operate.Code;
		 IIC_Temp[3] =Free_Program_Operate.Name;
		 IIC_Temp[4] =Free_Program_Operate.Name>>8;
		 IIC_Temp[5] =Free_Program_Operate.Name>>16;
		 IIC_Temp[6] =Free_Program_Operate.Name>>24;
		 IIC_Temp[7] =Free_Program_Operate.Num;
		 IIC_Temp[8]	=Free_Program_Operate.Num>>8;
		 IIC_Temp[9] =seccount;
		 IIC_Temp[10] =seccount>>8;
		 IIC_Temp[11] =seccount>>16;
		 IIC_Temp[12] =seccount>>24;
		 IIC_Temp[13] =Point_Num;
		 IIC_Temp[14] =Point_Num>>8;
		AT24CXX_Write(0x1100+(g_Run_Program_Num-1)*14,&IIC_Temp[1],14);	
		AT24CXX_Write(0x1015,&g_Run_Program_Num,1);//当前运行程序编号

		Saved_Program[Current_Select_Program].Order_Num = Order_Num;
		Saved_Program[Current_Select_Program].Time      = seccount;
		USART2_SendDataArray(15,FREE_PROGRAM_SEND,IIC_Temp);  //先发本条程序的信息变量
		DelayNms(1000);
		/**********-------------接收上传程序-----------------***************/
		Program_Save_OrNot=3;////程序载入
		Program_Transport_Count=0;
		Order_Num=0;
		USART1_ReceiveProgram(Free_Program_Operate.Num,g_Run_Program_Num);
		g_Background_Color_ProgramEdit = 1;			      //编辑背景色
		gs_Background_Color_ProgramEdit= 0;
		gs_Current_Operate_List_ProgramEdit=1;				//点选择界面蓝色背景所在的点编号
	 }
		gs_Run_Program_Code[0]=1;
		gs_Run_Program_Code[1]=2;
		gs_Run_Program_Code[2]=3;
		gs_Run_Program_Code[3]=4;
		gs_Run_Program_Code[4]=5;
		gs_Run_Program_Code[5]=6;
		gs_Run_Program_Code[6]=7;
		gs_Run_Program_Code[7]=8;
		gs_Run_Program_List[0]=1; 		
		gs_Run_Program_List[1]=2; 		
		gs_Run_Program_List[2]=3; 		
		gs_Run_Program_List[3]=4; 		
		gs_Run_Program_List[4]=5; 		
		gs_Run_Program_List[5]=1; 		
		gs_Run_Program_List[6]=2; 		
		gs_Run_Program_List[7]=3;
}

/*************************************************************************
**  函数名：  New_Product()
**	输入参数：
**	输出参数：无
**	函数功能：新建产品
**	备注：	  无
**  作者：    White_L   
**  开发日期：2018/10/11
**************************************************************************/
void New_Product(void)
{
	u16 n=0;
	 LCD_New_Order=FALSE;
	 //程序载入中
	 LCD_Current_Page = 66;
	 LCD_Pre_Page = LCD_Current_Page;
	 PageChange(LCD_Pre_Page);
	 for(n=0;n<8;n++)
	 {
		Free_Program_Operate.Program[n].List   = 0;
		Free_Program_Operate.Program[n].Key    = 0;
		Free_Program_Operate.Program[n].Value1 = 0;
		Free_Program_Operate.Program[n].Value2 = 0;	
		Free_Program_Operate.Program[n].Value3 = 0;
		Free_Program_Operate.Program[n].LC_ID	 = 0;
	 }	 
	 ReadRTC();
	 DelayNms(10);
	 for(n=0;n<SAVEPROGRAMNUM;n++)
		if(Program_Flag[n]==0)
			break;
		
	 Program_Flag[n]=1;
	 Saved_Program[n].Flag = 1;
	 Saved_Program[n].Name = name_save;						
	 Saved_Program[n].Time = seccount;//读取计数器的值

	 Free_Program_Operate.Flag     = 1;
	 Free_Program_Operate.Code		 = Saved_Program[n].Code;
	 Free_Program_Operate.Name     = name_save;
	 Free_Program_Operate.Num			 = 0;
	 Order_Num=0;
	 OperateProgram[Product_Num].Flag = 1;
	 OperateProgram[Product_Num].Code = Saved_Program[n].Code;
	 OperateProgram[Product_Num].XuHao= Product_Num+1;			
	 OperateProgram[Product_Num].Name = name_save;
	 OperateProgram[Product_Num].Order_Num  = 0;
	 OperateProgram[Product_Num].time = seccount;		
		
	 g_Run_Program_Num=Saved_Program[n].Code;//新建文件即为当前运行程序
	 g_Run_Program_Name=Saved_Program[n].Name;
	 AT24CXX_Write(0x1015,&g_Run_Program_Num,1);//当前运行程序编号
		
	 Order_Temp[0] = 0xEB;
	 Order_Temp[1] = 1;	//Flag				 
	 Order_Temp[2] = Saved_Program[n].Code;
	 Order_Temp[3] = name_save;	 
	 Order_Temp[4] = name_save>>8;//程序名称
	 Order_Temp[5] = name_save>>16;
	 Order_Temp[6] = name_save>>24;
	 Order_Temp[7] = 0;
	 Order_Temp[8] = 0;//点个数
	 Order_Temp[9] = seccount;
	 Order_Temp[10] = seccount>>8;
	 Order_Temp[11] = seccount>>16;
	 Order_Temp[12]= seccount>>24;
	 Order_Temp[13]= 0;
	 Order_Temp[14]= 0;
	 AT24CXX_Write(0x1100+n*14,&Order_Temp[1],14);	//保存每个产品信息
	 USART2_SendDataArray(15,FREE_PROGRAM_SEND,&Order_Temp[0]);
	 Product_Num++;
	 seccount  = 0;					 
	 g_Current_Program_Debug     = TRUE;
	 g_Refresh_Dispaly_List_Flag = TRUE;
	 gs_Current_Operate_List 		 = Product_Num;
	 g_Background_Color 				 = Product_Num%8;
	 if(g_Background_Color==0)
		 g_Background_Color=8;
	/*---------------程序编辑界面-------------------*/
	 g_Background_Color_ProgramEdit = 1;			      //编辑背景色
	 gs_Background_Color_ProgramEdit= 0;
	 gs_Current_Operate_List_ProgramEdit=1;				//点选择界面蓝色背景所在的点编号	
}
/*************************************************************************
**  函数名：  Product_Delete()
**	输入参数：
**	输出参数：无
**	函数功能：删除产品
**	备注：	  无
**  作者：    White_L   
**  开发日期：2018/10/11
**************************************************************************/
void Delete_Product(void)
{
	u16 i=0;
	 if(gs_Current_Operate_List>Product_Num)
	 {
	 }
	 else
	 {
		if(Product_Num>0)//删除程序以后所有后面程序前移//
		{
			if(OperateProgram[gs_Current_Operate_List-1].Code==g_Run_Program_Num)
			{
					gs_Run_Program_Code[0]=0;
					gs_Run_Program_Code[1]=0;
					gs_Run_Program_Code[2]=0;
					gs_Run_Program_Code[3]=0;
					gs_Run_Program_Code[4]=0;
					gs_Run_Program_Code[5]=0;
					gs_Run_Program_Code[6]=0;
					gs_Run_Program_Code[7]=0;
					gs_Run_Program_List[0]=0; 		
					gs_Run_Program_List[1]=0; 		
					gs_Run_Program_List[2]=0; 		
					gs_Run_Program_List[3]=0; 		
					gs_Run_Program_List[4]=0; 		
					gs_Run_Program_List[5]=0; 		
					gs_Run_Program_List[6]=0; 		
					gs_Run_Program_List[7]=0;
				 Free_Program_Operate.Flag     = 0;
				 Free_Program_Operate.Code		 = 0;
				 Free_Program_Operate.Name     = 0;
				 Free_Program_Operate.Num			 = 0;
				 Current_Operate_Program.Flag  = 0;
				 Current_Operate_Program.Code  = 0;
				 Current_Operate_Program.Name  = 0;
				 Current_Operate_Program.Num   = 0;
				 AT24CXX_Write(0x1015,&g_Run_Program_Num,1);
				 /*---------------点选择界面-----------------------*/
				 g_Background_Color_PointEdit = 1;			      //编辑背景色
				 gs_Background_Color_PointEdit= 0;
				 gs_Current_Operate_List_PointEdit=1;				//点选择界面蓝色背景所在的点编号
				 /*---------------点编辑界面-------------------*/
				 g_Background_Color_PointEdit = 1;			      //编辑背景色
				 gs_Background_Color_PointEdit= 0;
				 gs_Current_Operate_List_PointEdit=1;				//点选择界面蓝色背景所在的点编号
				 /*---------------程序编辑界面-------------------*/
				 g_Background_Color_ProgramEdit = 1;			      //编辑背景色
				 gs_Background_Color_ProgramEdit= 0;
				 gs_Current_Operate_List_ProgramEdit=1;				//点选择界面蓝色背景所在的点编号

				 g_Run_Program_Num=0;
				 g_Run_Program_Name=0;//主界面刷新 
			}
			Program_Flag[OperateProgram[gs_Current_Operate_List-1].Code-1]=0;//对应产品标志位置0
			Product_Num--;//产品数减一
			Order_Temp[0] = P_PROGRAM_DELETE;
			Order_Temp[1]=OperateProgram[gs_Current_Operate_List-1].Code;//对应程序编号
			USART2_SendDataArray(2,FREE_PROGRAM_SEND,&Order_Temp[0]);	
			g_Current_Program_Debug=FALSE;
			for(i=gs_Current_Operate_List-1;i<Product_Num;i++)
			{
				OperateProgram[i].Flag  = OperateProgram[i+1].Flag;
				OperateProgram[i].Code  = OperateProgram[i+1].Code;
				OperateProgram[i].Order_Num   = OperateProgram[i+1].Order_Num;
				OperateProgram[i].Name  = OperateProgram[i+1].Name;
				OperateProgram[i].time  = OperateProgram[i+1].time;
			}
			Saved_Program[OperateProgram[gs_Current_Operate_List-1].Code-1].Flag=0;
			Saved_Program[OperateProgram[gs_Current_Operate_List-1].Code-1].Name=0;
			Saved_Program[OperateProgram[gs_Current_Operate_List-1].Code-1].Order_Num=0;
			Saved_Program[OperateProgram[gs_Current_Operate_List-1].Code-1].Time=0;
			OperateProgram[Product_Num].Flag = 0;
			OperateProgram[Product_Num].Code = 0;
			OperateProgram[Product_Num].XuHao= 0;
			OperateProgram[Product_Num].Order_Num  = 0;
			OperateProgram[Product_Num].Name = 0;
			OperateProgram[Product_Num].time = 0;
		}										
	 }
}
/*************************************************************************
**  函数名：  ValueToString()
**	输入参数：
**	输出参数：无
**	函数功能：函数参数值转变为文本
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void ValueToString(u32 Temp_Value)
{
	u8 j=0;
	u8 k=0;
	for(j=0;j<10;j++)
	{
		Parameter_Value[j]=0;	
	}
	//坐标设置 
	if(Temp_Value>>28 == 0x0f)
	{
		Temp_Value = Temp_Value & 0x0fffffff;	
		j=9;
	    if(Temp_Value==0)
		{
			Parameter_Value[0]=0x20;
			Parameter_Value[1]=0x20;
			Parameter_Value[2]=0x20;
			Parameter_Value[3]=0x20;
			Parameter_Value[4]=0x20;
			Parameter_Value[5]=0x20;
			Parameter_Value[6]=0x30;
			Parameter_Value[7]=0x2E;
			Parameter_Value[8]=0x30;
			Parameter_Value[9]=0x30;		
		}
		else
		{
			while((Temp_Value!=0) && (j!=0))
		    {						
				if(j==7)
				{
					Parameter_Value[j]=0x2E;
				}		
				else
				{
					Parameter_Value[j]=Temp_Value%10+0x30;
					Temp_Value=Temp_Value/10;
				}
				j--;
			}
			for(k=0;k<=j;k++)
			{
				Parameter_Value[k]=0x20;
			}
		}
	}
	//速度设置
	else if(Temp_Value>>28 == 0x0e)
	{
		Temp_Value = Temp_Value & 0x0fffffff;			
	    Parameter_Value[9]=0x20;
		Parameter_Value[8]=0x25;		   //%
		j=7;
		while((Temp_Value!=0) && (j!=0))
	    {						
			Parameter_Value[j]=Temp_Value%10+0x30;
			Temp_Value=Temp_Value/10;
			j--;
		}
		for(k=0;k<=j;k++)
		{
			Parameter_Value[k]=0x20;
		}	
	}
	//时间设置
	else if(Temp_Value>>28 == 0x0d)
	{
		Temp_Value = Temp_Value & 0x0fffffff;	
		Parameter_Value[9]=0x20;
		Parameter_Value[8]=0x53;			//S
		if(Temp_Value<100)
		{
			//Parameter_Value[0]=0x20;
			Parameter_Value[0]=0x20;
			Parameter_Value[1]=0x20;
			Parameter_Value[2]=0x20;
			Parameter_Value[3]=0x20;
			Parameter_Value[4]=0x30;
			Parameter_Value[5]=0x2E;
			Parameter_Value[6]=Temp_Value/10+0x30;
			Parameter_Value[7]=Temp_Value%10+0x30;
				
		}
		else
		{	
			j=7;
		    while((Temp_Value!=0) && (j!=0))
		    {						
				if(j==5)
				{
					Parameter_Value[j]=0x2E;
				}		
				else
				{
					Parameter_Value[j]=Temp_Value%10+0x30;
					Temp_Value=Temp_Value/10;
				}
				j--;
			}
			for(k=0;k<=j;k++)
			{
				Parameter_Value[k]=0x20;
			}
		}
	}

}

/*************************************************************************
**  函数名：  ValueToName()
**	输出参数：无
**	函数功能：函数参数值转变为名字文本
**	备注：	  无
**  作者：    DH   
**  开发日期：2018/7/4
**************************************************************************/
void ValueToName(u8 a)
{
	u8 i=0;
	int b;
	for(b = 0; b<4; b++)
	{
		if(gs_name[i][b] == 0x00 || gs_name[i][b] == 0xFF)
			gs_name[i][b] = 0x20;
	}
}


/*************************************************************************
**  函数名：  ValueToNumber()
**	输出参数：无
**	函数功能：函数参数值转变为数字文本
**	备注：	  无
**  作者：    DH   
**  开发日期：2018/7/4
**************************************************************************/
void ValueToNumber(u16 Temp_Value)
{
	Parameter_Value[0]=(Temp_Value)/1000 + 0x30;
	Parameter_Value[1]=(Temp_Value%1000)/100 + 0x30;
	Parameter_Value[2]=(Temp_Value%100)/10 + 0x30;
	Parameter_Value[3]=Temp_Value%10 + 0x30;
	if(Parameter_Value[0]==0x30)
		Parameter_Value[0]=0x20;
	else if(Parameter_Value[0]==0x30&&Parameter_Value[1]==0x30)
	{
		Parameter_Value[0]=0x20;
		Parameter_Value[1]=0x20;
	}

}
/*************************************************************************
**  函数名：  ValueToTime()
**	输入参数：
**	输出参数：无
**	函数功能：函数参数值转变为文本时间
**	备注：	  无
**  作者：    DH   
**  开发日期：2018/7/4
**************************************************************************/
void ValueToTime(u32 Temp_Value)
{
	int table[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
	int year = 0;
	int month = 0;
	int day = 0;
	int a=0,b=0,c=0,i=0;
	if(Temp_Value==0)
		{
			Parameter_Value[0]=0x20;
			Parameter_Value[1]=0x20;
			Parameter_Value[2]=0x20;
			Parameter_Value[3]=0x20;
			Parameter_Value[4]=0x2D;
			Parameter_Value[5]=0x20;
			Parameter_Value[6]=0x20;
			Parameter_Value[7]=0x2D;
			Parameter_Value[8]=0x20;
			Parameter_Value[9]=0x20;		
		}
	 else
		{//时间换算早于1792年有BUG
			day = (Temp_Value - 57600)/ 86400;
			a = (day - 365*2 - 1)/1461;          //闰年数
			b = (day - 365*2 - 1 - a*1461)/365;
			year = a*4 + b + 1972;
			c = day - 365 - 366 - a*1461 - b*365 + 1; //除去年的天数
			if (c != 365)
			{
				for (i = 0;i < 12;i++)
				{	
					if (c <= table[i])
						break;
					c = c - table[i];
				}
				month = i + 1;
				day = c + 1;
			}
			else
			{
				year += 1;
				month = 1;
				day = 1;
			}
			Parameter_Value[0]=year/1000 + 0x30;
			Parameter_Value[1]=(year%1000)/100 + 0x30;
			Parameter_Value[2]=(year%100)/10 + 0x30;
			Parameter_Value[3]=(year%10) + 0x30;
			Parameter_Value[4]=0x2D;
			Parameter_Value[5]=month/10 + 0x30;
			Parameter_Value[6]=month%10 + 0x30;
			Parameter_Value[7]=0x2D;
			Parameter_Value[8]=day/10 + 0x30;
			Parameter_Value[9]=day%10 + 0x30;
		 }		
}
/*----------------基本指令--------------*/
void Basic_Command()
{
	switch(LCD_Var_Address)
	{
		case 0x200C:		  //基本指令-20	  跳转到第2页
			 LCD_Current_Page = 2; 
		     break;

		case 0x200D:		  //主程序开始
			 ProcessProgram(OR_BASICORDER,K_PROGRAMSTART);
		     LCD_Current_Page = 2;  
			 break;

		case 0x200E:		  //主程序结束
		     ProcessProgram(OR_BASICORDER,K_PROGRAMEND);
			 LCD_Current_Page = 2;  
			 break;

		case 0x2011:		  //延时
		     ProcessProgram(OR_BASICORDER,K_DELAY);
			 LCD_Current_Page = 2;  
			 break;
		case 0x201B:		  //上料仓程序
		     ProcessProgram(OR_BASICORDER,K_LOADPROGRAM);
			 LCD_Current_Page = 2;  
			 break;
		case 0x201C:		  //卸料仓程序
	     	 ProcessProgram(OR_BASICORDER,K_UNLOADPROGRAM);
		     LCD_Current_Page = 2;  
			 break;
		case 0x201D:		  //启动单次动作
	     	 ProcessProgram(OR_BASICORDER,K_STARTACTION);
		     LCD_Current_Page = 2;  
			 break;
		case 0x201E:		  //启动单次动作
	     	 ProcessProgram(OR_BASICORDER,K_OAXISRESET);
		     LCD_Current_Page = 2;  
			 break;
		default:
		     break;
	}
}

/*----------------轴控指令--------------*/
void Axis_Command()
{
	switch(LCD_Var_Address)
	{
		case 0x2012:		 //轴控指令-26	  跳转到第2页			     				 
			 LCD_Current_Page = 2;
			 break;

		case 0x2013:		 //X轴-水平
			 ProcessProgram(OR_AXISORDER,K_XAXIS);
			 LCD_Current_Page = 2;
		     break;

		case 0x2014:		 //Z轴-垂直
		     ProcessProgram(OR_AXISORDER,K_ZAXIS);
			 LCD_Current_Page = 2;
		     break;

		case 0x2015:		 //L轴-前后
//		     if(JXS_Parameter.Axis == FAN_X_Z_L)
//			 {
		         ProcessProgram(OR_AXISORDER,K_LAXIS);
			     LCD_Current_Page = 2;
//			 }
			 break;
		case 0x2016:		 //O轴-料仓
		     ProcessProgram(OR_AXISORDER,K_OAXIS);
			 LCD_Current_Page = 2;
		     break;
		case 0x2017:		  //上料仓位置
	     	 ProcessProgram(OR_AXISORDER,K_LOADPOSITION);
		     LCD_Current_Page = 2;  
			 break;
		case 0x2018:		  //卸料仓位置
	     	 ProcessProgram(OR_AXISORDER,K_UNLOADPOSITION);
		     LCD_Current_Page = 2;  
			 break;
		case 0x2019:		  //垂直轴位置
	     	 ProcessProgram(OR_AXISORDER,K_OAXISPOSITION);
		     LCD_Current_Page = 2;  
			 break;
		case 0x201A:		  //垂直轴位置
	     	 ProcessProgram(OR_AXISORDER,K_ZAXISPOSITION);
		     LCD_Current_Page = 2;  
			 break;		default:
		     break;
	}
}

/*----------------指令编号--------------*/
void List_Setting()
{
	u8 PressList=0;
	u8 i=0;
	if((gs_Current_Operate_List>0)&&(Current_Operate_Program.Program[gs_Current_Operate_List-1].Flag==TRUE))
	{
		if(((u8)(LCD_Var_Address&0x000f)+0x01)==gs_Background_Color) //点击当前编号按钮在选中的操作行
		{
			if(LCD_Var_Address==0x2210)
			{	     				 
				 if(gs_Current_Operate_List==1)	//操作第一条指令
				 {
				 }	
				 else
				 {
				     if(Current_Operate_Program.Program[gs_Current_Operate_List-1].List==Current_Operate_Program.Program[gs_Current_Operate_List-2].List)
					 { //当前行与上一行相同
					     Current_Operate_Program.Program[gs_Current_Operate_List-1].List++; 
						 PressList=2;

					 }
					 else
					 {
					     Current_Operate_Program.Program[gs_Current_Operate_List-1].List--;
						 PressList=1;
					 }
				 }
            }
			else
			{
			     if(Current_Operate_Program.Program[gs_Current_Operate_List-1].List==Current_Operate_Program.Program[gs_Current_Operate_List-2].List)
				 { //当前行与上一行相同
				     Current_Operate_Program.Program[gs_Current_Operate_List-1].List++; 
					 PressList=2;

				 }
				 else
				 {
				     Current_Operate_Program.Program[gs_Current_Operate_List-1].List--;
					 PressList=1;
				 }				
						
			}
			if(PressList==1)   //当前编号减1
			{
				for(i=gs_Current_Operate_List;i<Current_Operate_Program.Num;i++)
				{
					Current_Operate_Program.Program[i].List--;
				}
			}
			else if(PressList==2)  //当前编号加1
			{
				for(i=gs_Current_Operate_List;i<Current_Operate_Program.Num;i++)
				{
					Current_Operate_Program.Program[i].List++;
				}		
			}
			g_Refresh_Dispaly_List_Flag = TRUE;
		}
	}
}

/*************************************************************************
**  函数名：  LCDFreeProgramProcess()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void LCDFreeProgramProcess(void)
{
	u8 i=0,n=0,j=0,k=0;
		if( (Robot_Origined == TRUE)&&(Robot_Status == ACTION_STOP))	//机械手已回零，且处于停止状态
	{
		 //如果不是当前页面，则切换到当前界面
		if( (LCD_Var_Address == 0x2000) && (LCD_Main_Page != FREE_PROGRAMME))
		{
				LCD_Main_Page_Pre = LCD_Main_Page;
				LCD_Main_Page = FREE_PROGRAMME;
				LCD_Current_Page = 	LCD_Main_Page;
				g_Watch_IO_Control = 0; 						          //IO口实时监控关
				g_Refresh_Dispaly_List_Flag=TRUE;                         //刷新程序显示列表开
				g_Robot_Auto_Reset = FALSE;                               //机械手自动复位关
				gs_Background_Color = 0;
		}
		else
		{
			if(Program_Or_Point==0||Program_Or_Point==1)
			{
/********************************************产品编辑界面*********************************************************/
			switch(LCD_Var_Address)
			{
				case 0x2F00://寸动联动切换
					Link_Step_Flag=!Link_Step_Flag;
					Link_Step_Refresh=1;
					break;
				case 0x2807://文件新建
						LCD_Current_Page=124;//加个条件
						break;
				
				case 0x26BD://新建确定
					if(Product_Num<SAVEPROGRAMNUM)
					{
						New_Product();
						Program_Or_Point       = 2;
						Current_Select_Program = gs_Current_Operate_List-1;
						gs_Background_Color_ProgramEdit=0;
						g_Refresh_Dispaly_List_Flag = TRUE;
						Select_Program_Flag         = TRUE;
						LCD_Current_Page       = 8;//自由编程界面	
					}						
					break;
					
				case 0x26BE://新建取消
					 name_temp[0] = 0x20;name_temp[1] = 0x20;name_temp[2] = 0x20;name_temp[3] = 0x20;
					 DisplayChineseString(0x2130,name_temp,4);
					 gs_Background_Color=0;
					 LCD_Current_Page = 2;//文件管理界面						
					break;		
				
				case 0x2801://产品载入  是否载入（文件载入后才可以双击编辑）
					 LCD_Current_Page = 133;	 
					 break;	
			  case 0x2620://产品载入
					Select_Product();
					LCD_Current_Page = 2;
					 break;
				
				case 0x2803://重命名
							LCD_Current_Page = 131;
					break;
				
				case 0x2802://重命名 重名检测
				     if(LCD_Key_Value < 0x01)                       
				     {
							 for(i=0;i<Product_Num;i++)
							 {
								 if(OperateProgram[i].Name==LCD_Var_Data)
									 break;
							 }
							 if(i==Product_Num)//没有重名程序
							 {
								 name_temp[3] = LCD_Var_Data;
								 name_temp[2] = LCD_Var_Data>>8;
								 name_temp[1] = LCD_Var_Data>>16;
								 name_temp[0] = LCD_Var_Data>>24;
								 name_save = name_temp[0]<<24|name_temp[1]<<16|name_temp[2]<<8|name_temp[3];
								 DisplayChineseString(0x2130,name_temp,4);								 
							 }
							 else
							 {
								 name_temp[3] = LCD_Var_Data;
								 name_temp[2] = LCD_Var_Data>>8;
								 name_temp[1] = LCD_Var_Data>>16;
								 name_temp[0] = LCD_Var_Data>>24;
								 name_save = name_temp[0]<<24|name_temp[1]<<16|name_temp[2]<<8|name_temp[3];
								 Same_Name_Num = i+1;
								 LCD_Current_Page = 135;								 //重名提示
							 }
						 }
							break;
				case 0x2631://重命名确定
					 OperateProgram[gs_Current_Operate_List - 1].Name                       = name_save;
					 Saved_Program[OperateProgram[gs_Current_Operate_List - 1].Code-1].Name = name_save;
					 AT24CXX_Write(0x1100+(OperateProgram[gs_Current_Operate_List - 1].Code-1)*11+2,(u8*)&name_save,4);
					 g_Refresh_Dispaly_List_Flag = TRUE;
				
					 if(OperateProgram[gs_Current_Operate_List - 1].Code==g_Run_Program_Num)//当前重命名产品为当前选择产品
					 {
						 g_Run_Program_Name          = name_save;						 
						 Select_Program_Flag=TRUE;
					 }
					 
					 Order_Temp[0] = P_PROGRAM_RENAME;
					 Order_Temp[1] = name_save;//程序名称
					 Order_Temp[2] = name_save>>8;
					 Order_Temp[3] = name_save>>16;
					 Order_Temp[4] = name_save>>24;
					 Order_Temp[5] = OperateProgram[gs_Current_Operate_List - 1].Code;//对应程序编号
					 USART2_SendDataArray(6,FREE_PROGRAM_SEND,&Order_Temp[0]);
					 LCD_Current_Page = 2;
					break;
					 
				case 0x2632:		 //重命名设置-取消
					 name_temp[0] = 0x20;name_temp[1] = 0x20;name_temp[2] = 0x20;name_temp[3] = 0x20;
					 DisplayChineseString(0x2130,name_temp,4);
					 LCD_Current_Page = 2;
					 break;
				case 0x2637:   //重名提示×返回
					 LCD_Current_Page = 2;
					break;
				
				case 0x2008:		  //删除产品
					if(Program_Flag[OperateProgram[gs_Current_Operate_List-1].Code-1]!=0)
						LCD_Current_Page = 132;				
				     break;
				case 0x2622:		 //删除产品
					 Delete_Product();
					 g_Refresh_Dispaly_List_Flag = TRUE;
					 gs_Background_Color=0;
					 LCD_Current_Page = 2;				 
					 break;
					 
				case 0x2623:		 //取消删除
				   g_SaveProgram_Flag=0;
					 LCD_Current_Page = 2;
					 break;					 

				case 0x2820://产品查询 
					name_save = name_temp[0]<<24 | name_temp[1]<<16 | name_temp[2]<<8 | name_temp[3];
					if(name_save == 0x20202020)
					{
						LCD_Current_Page = 141;	
					}
					else
					{
					  for(i = 0; i<Product_Num; i++)
					  {
							if(name_save == OperateProgram[i].Name)
							{
								if(OperateProgram[i].XuHao>8)
								{
									gs_Current_Operate_List = i + 1;
									g_Background_Color = 1;
									g_Refresh_Dispaly_List_Flag = TRUE;
								}
								else
								{
									gs_Current_Operate_List = i + 1;
									g_Background_Color = i + 1;
									g_Refresh_Dispaly_List_Flag = TRUE;									
								}
								break;
							}
							else if(i == Product_Num-1)
							{
								LCD_Current_Page = 142;	
							}
					  }
					}

					break;
				
				case 0x2830://产品查询输入名字 
					 if(LCD_Key_Value < 0x01) 
				     {
						 name_temp[3] = LCD_Var_Data;
						 name_temp[2] = LCD_Var_Data>>8;
						 name_temp[1] = LCD_Var_Data>>16;
						 name_temp[0] = LCD_Var_Data>>24;
						 DisplayChineseString(0x2830,name_temp,4);
					 }
					 break;
					 
				case 0x2805://产品查询返回（点叉）
					 LCD_Current_Page = 2;
					 gs_Current_Operate_List = 1;
					 g_Background_Color = 1;
					 g_Refresh_Dispaly_List_Flag = TRUE;						
				     break;
				
				case 0x2201:		 //上一页
					 if(Product_Num!=0)
					 {
						 if(gs_Current_Operate_List==g_Background_Color)	//程序的前八行
						 {
						 }
						 //不在前八行
						 else if(gs_Current_Operate_List-g_Background_Color<=8) //当前操作行前面没有8行，即不足单页显示
						 {
						 	gs_Current_Operate_List=1;
							g_Background_Color=1;
						 }
						 else
						 {
						 	gs_Current_Operate_List=gs_Current_Operate_List-g_Background_Color+1-8;
							g_Background_Color = 1;
						 }
						 g_Refresh_Dispaly_List_Flag=TRUE;
					 }
				     break;
	
				case 0x2202:		 //下一页
					 if(Product_Num!=0)
					 {
						 if(Product_Num>(gs_Current_Operate_List-g_Background_Color+8)) //后面还有程序，则翻页
						 {
						 	gs_Current_Operate_List = gs_Current_Operate_List-g_Background_Color+8+1;
	   						g_Background_Color = 1;
							if(gs_Current_Operate_List+7-g_Background_Color>=LARGESTPROGRAMNUM)
							{
								gs_Current_Operate_List=LARGESTPROGRAMNUM-g_Background_Color-7+1;  //93
							}
						 }	
						 g_Refresh_Dispaly_List_Flag=TRUE;		   
					 }
					 break;
				case 0x2328://提示载入界面×返回
						Program_Or_Point=1;
						LCD_Current_Page       = 2;//产品编程界面
					break;
				case 0x2204:		 //第一行
					 if(gs_Current_Operate_List-g_Background_Color+1>Product_Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List = gs_Current_Operate_List-g_Background_Color+1;
						 g_Background_Color=1;
						 g_Current_Program_Modify=FALSE;
						 if(Current_Operate_Program.Program[gs_Current_Operate_List-1].Order==OR_AXISORDER)
						 {
							g_Current_Program_Modify=TRUE;
						 }
						if(Product_Num!=0&&OperateProgram[gs_Current_Operate_List-1].Code==g_Run_Program_Num)
						{
							Click_Count++;
							if(Click_Count==2)
							{
							 Click_Count 						= 0;
							 Program_Or_Point       = 2;
							 Current_Select_Program = gs_Current_Operate_List-1;
							 gs_Background_Color_ProgramEdit=0;
							 g_Refresh_Dispaly_List_Flag = TRUE;
							 Select_Program_Flag         = TRUE;
							 LCD_Current_Page       = 8;//自由编程界面
							}
						}
						else 
						{
							Click_Count1++;
							if(Click_Count1==2)
							{
								Click_Count1=0;
								LCD_Current_Page       = 161;//提示先载入文件
							}
						}
					 }
					 break;
					 
				case 0x2205:		 //第二行
					 if(gs_Current_Operate_List-g_Background_Color+2>Product_Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List = gs_Current_Operate_List-g_Background_Color+2;
						 g_Background_Color=2;
						 g_Current_Program_Modify=FALSE;
						 if(Current_Operate_Program.Program[gs_Current_Operate_List-1].Order==OR_AXISORDER)
						 {
							g_Current_Program_Modify=TRUE;
						 }
						if(Product_Num!=0&&OperateProgram[gs_Current_Operate_List-1].Code==g_Run_Program_Num)
						{
							Click_Count++;
							if(Click_Count==2)
							{
							 Click_Count 						= 0;
							 Program_Or_Point       = 2;
							 Current_Select_Program = gs_Current_Operate_List-1;
							 gs_Background_Color_ProgramEdit=0;
							 g_Refresh_Dispaly_List_Flag = TRUE;
							 Select_Program_Flag         = TRUE;
							 LCD_Current_Page       = 8;//自由编程界面
							}
						}
						else
						{
							 LCD_Current_Page       = 161;//提示先载入文件
						}
					 }
				     break;
	
				case 0x2206:		 //第三行
					 if(gs_Current_Operate_List-g_Background_Color+3>Product_Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List = gs_Current_Operate_List-g_Background_Color+3;
						 g_Background_Color=3;
						 g_Current_Program_Modify=FALSE;
						 if(Current_Operate_Program.Program[gs_Current_Operate_List-1].Order==OR_AXISORDER)
						 {
							g_Current_Program_Modify=TRUE;
						 }
						if(Product_Num!=0&&OperateProgram[gs_Current_Operate_List-1].Code==g_Run_Program_Num)
						{
							Click_Count++;
							if(Click_Count==2)
							{
							 Click_Count 						= 0;
							 Program_Or_Point       = 2;
							 Current_Select_Program = gs_Current_Operate_List-1;
							 gs_Background_Color_ProgramEdit=0;
							 g_Refresh_Dispaly_List_Flag = TRUE;
							 Select_Program_Flag         = TRUE;
							 LCD_Current_Page       = 8;//自由编程界面
							}
						}
						else
						{
							 LCD_Current_Page       = 161;//提示先载入文件
						}
					 }
					 break;
					 
				case 0x2207:		 //第四行
					 if(gs_Current_Operate_List-g_Background_Color+4>Product_Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List = gs_Current_Operate_List-g_Background_Color+4;
						 g_Background_Color=4;
						 g_Current_Program_Modify=FALSE;
						 if(Current_Operate_Program.Program[gs_Current_Operate_List-1].Order==OR_AXISORDER)
						 {
							g_Current_Program_Modify=TRUE;
						 }
						if(Product_Num!=0&&OperateProgram[gs_Current_Operate_List-1].Code==g_Run_Program_Num)
						{
							Click_Count++;
							if(Click_Count==2)
							{
							 Click_Count 						= 0;
							 Program_Or_Point       = 2;
							 Current_Select_Program = gs_Current_Operate_List-1;
							 gs_Background_Color_ProgramEdit=0;
							 g_Refresh_Dispaly_List_Flag = TRUE;
							 Select_Program_Flag         = TRUE;
							 LCD_Current_Page       = 8;//自由编程界面
							}
						}
						else
						{
							 LCD_Current_Page       = 161;//提示先载入文件
						}
				   }
					 break;
	
				case 0x2208:		 //第五行
					 if(gs_Current_Operate_List-g_Background_Color+5>Product_Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List = gs_Current_Operate_List-g_Background_Color+5;
						 g_Background_Color=5;
						 g_Current_Program_Modify=FALSE;
						 if(Current_Operate_Program.Program[gs_Current_Operate_List-1].Order==OR_AXISORDER)
						 {
							g_Current_Program_Modify=TRUE;
						 }
						if(Product_Num!=0&&OperateProgram[gs_Current_Operate_List-1].Code==g_Run_Program_Num)
						{
							Click_Count++;
							if(Click_Count==2)
							{
							 Click_Count 						= 0;
							 Program_Or_Point       = 2;
							 Current_Select_Program = gs_Current_Operate_List-1;
							 gs_Background_Color_ProgramEdit=0;
							 g_Refresh_Dispaly_List_Flag = TRUE;
							 Select_Program_Flag         = TRUE;
							 LCD_Current_Page       = 8;//自由编程界面
							}
						}
						else
						{
							 LCD_Current_Page       = 161;//提示先载入文件
						}
				  }
					 break;	
					 
				case 0x2209:		 //第六行
					 if(gs_Current_Operate_List-g_Background_Color+6>Product_Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List = gs_Current_Operate_List-g_Background_Color+6;
						 g_Background_Color=6;
						 g_Current_Program_Modify=FALSE;
						 if(Current_Operate_Program.Program[gs_Current_Operate_List-1].Order==OR_AXISORDER)
						 {
							g_Current_Program_Modify=TRUE;
						 }
						if(Product_Num!=0&&OperateProgram[gs_Current_Operate_List-1].Code==g_Run_Program_Num)
						{
							Click_Count++;
							if(Click_Count==2)
							{
							 Click_Count 						= 0;
							 Program_Or_Point       = 2;
							 Current_Select_Program = gs_Current_Operate_List-1;
							 gs_Background_Color_ProgramEdit=0;
							 g_Refresh_Dispaly_List_Flag = TRUE;
							 Select_Program_Flag         = TRUE;
							 LCD_Current_Page       = 8;//自由编程界面
							}
						}
						else
						{
							 LCD_Current_Page       = 161;//提示先载入文件
						}
				   }
				     break;
	
				case 0x220A:		 //第七行
					 if(gs_Current_Operate_List-g_Background_Color+7>Product_Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List = gs_Current_Operate_List-g_Background_Color+7;
						 g_Background_Color=7;
						 g_Current_Program_Modify=FALSE;
						 if(Current_Operate_Program.Program[gs_Current_Operate_List-1].Order==OR_AXISORDER)
						 {
							g_Current_Program_Modify=TRUE;
						 }
						if(Product_Num!=0&&OperateProgram[gs_Current_Operate_List-1].Code==g_Run_Program_Num)
						{
							Click_Count++;
							if(Click_Count==2)
							{
							 Click_Count 						= 0;
							 Program_Or_Point       = 2;
							 Current_Select_Program = gs_Current_Operate_List-1;
							 gs_Background_Color_ProgramEdit=0;
							 g_Refresh_Dispaly_List_Flag = TRUE;
							 Select_Program_Flag         = TRUE;
							 LCD_Current_Page       = 8;//自由编程界面
							}
						}
						else
						{
							 LCD_Current_Page       = 161;//提示先载入文件
						}
				   }
					 break;					 

				case 0x220B:		 //第八行
					 if(gs_Current_Operate_List-g_Background_Color+8>Product_Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List = gs_Current_Operate_List-g_Background_Color+8;
						 g_Background_Color=8;
						 g_Current_Program_Modify=FALSE;
						 if(Current_Operate_Program.Program[gs_Current_Operate_List-1].Order==OR_AXISORDER)
						 {
							g_Current_Program_Modify=TRUE;
						 }
						if(Product_Num!=0&&OperateProgram[gs_Current_Operate_List-1].Code==g_Run_Program_Num)//双击进入自由编程界面
						{
							Click_Count++;
							if(Click_Count==2)
							{
							 Click_Count 						= 0;
							 Program_Or_Point       = 2;
							 Current_Select_Program = gs_Current_Operate_List-1;
							 gs_Background_Color_ProgramEdit=0;
							 g_Refresh_Dispaly_List_Flag = TRUE;
							 Select_Program_Flag         = TRUE;
							 LCD_Current_Page       = 8;//自由编程界面
							}
						}
						else
						{
							 LCD_Current_Page       = 161;//提示先载入文件
						}
				   }
				     break;
				 			
				case 0x2621:		 //取消保存
				     g_SaveProgram_Flag=0;
					 LCD_Current_Page = 2;
					 break;	
				default:
				    break;
				}
			}//switch(LCD_Var_Address)
/**************************************自由编程界面**************************************************/
		else if(Program_Or_Point==2)
		{
			switch(LCD_Var_Address)
			{
				case 0x2F00://寸动联动切换
					Link_Step_Flag=!Link_Step_Flag;
					Link_Step_Refresh=1;
					break;
				case 0x2331://点胶指令
					 Order_Type=1;
					 Point_Action_Flag   = 0;
					 LCD_Main_Page       = 4;                	//LCD主界面为手动调试                    
					 LCD_Current_Page    = LCD_Main_Page;          			//LCD当前界面为手动调试
					 g_Watch_IO_Control  = 0; 						  	    				//实时IO监视状态
					 Order_Temp[0]       = MANUL_MODE;                   //设置为手动模式
					 USART2_SendDataArray(1,WORK_MODE,&Order_Temp[0]);//串口发送工作模式中的手动模式到主控板
					 g_Robot_Auto_Reset = FALSE;
					 LC_IO_Axis_Code = 0x4001;
					break;
				
				case 0x2332://点钻指令
					 Order_Type=2;
					 Point_Action_Flag   = 0;
					 LCD_Main_Page       = 4;                	//LCD主界面为手动调试                    
					 LCD_Current_Page    = LCD_Main_Page;          			//LCD当前界面为手动调试
					 g_Watch_IO_Control  = 0; 						  	    				//实时IO监视状态
					 Order_Temp[0]       = MANUL_MODE;                   //设置为手动模式
					 USART2_SendDataArray(1,WORK_MODE,&Order_Temp[0]);//串口发送工作模式中的手动模式到主控板
					 g_Robot_Auto_Reset = FALSE;
					 LC_IO_Axis_Code = 0x4001;
					break;
				case 0x2333://取料指令				
					 Order_Type=3;
					 Point_Action_Flag   = 0;
					 LCD_Main_Page       = MANUL_DEBUG;                	//LCD主界面为手动调试                    
					 LCD_Current_Page    = 6;          			//LCD当前界面为手动调试
					 Parameter_Refresh   = TRUE;
					 g_Watch_IO_Control  = 0; 						  	    				//实时IO监视状态
					 Order_Temp[0]       = MANUL_MODE;                   //设置为手动模式
					 USART2_SendDataArray(1,WORK_MODE,&Order_Temp[0]);//串口发送工作模式中的手动模式到主控板
					 g_Robot_Auto_Reset = FALSE;
					 LC_IO_Axis_Code = 0x4010;
					break;
				case 0x2334://延时指令
					 Order_Type=4;
					 LCD_Current_Page    = 162;          			//延时输入界面
					break;
				case 0x2C10://延时输入确认
					 Order_Type=4;
					 if(Oeder_Edit_Flag==0)
					 {
						 Point_Action_Flag   = 0;
						 //直接让用户输入延时
						 if(Order_Check(Order_Type))
							Insert_Order(Order_Type);
						 else
							LCD_Current_Page=8;
					 }
					 else
					 {
						 if(Order_Check(Order_Type))
							Insert_Order(Order_Type);	
						else
							LCD_Current_Page=8;
						 Point_Action_Flag   = 0;						 
					 }

					break;
				case 0x2C11://延时输入取消
					 Order_Type=0;
					 LCD_Current_Page    = 8;          			//延时输入界面
					break;
				case 0x2329://空移指令
					 Order_Type=5;
					 Point_Action_Flag   = 0;
					 LCD_Main_Page       = MANUL_DEBUG;                	//LCD主界面为手动调试                    
					 LCD_Current_Page    = LCD_Main_Page;          			//LCD当前界面为手动调试
					 g_Watch_IO_Control  = 0; 						  	    				//实时IO监视状态
					 Order_Temp[0]       = MANUL_MODE;                   //设置为手动模式
					 USART2_SendDataArray(1,WORK_MODE,&Order_Temp[0]);//串口发送工作模式中的手动模式到主控板
					 g_Robot_Auto_Reset = FALSE;
					 LC_IO_Axis_Code = 0x4001;
					break;
				
				case 0x2853://删除指令
					 Delete_Order();
				     break;		
				case 0x2851://修改指令
					 Oeder_Edit_Flag = 1;
					 Order_Type          = Free_Program_Operate.Program[(gs_Current_Operate_List_ProgramEdit-1)%LARGESTPROGRAMNUM].Key;
					 Point_Action_Flag   = Free_Program_Operate.Program[(gs_Current_Operate_List_ProgramEdit-1)%LARGESTPROGRAMNUM].List;
					 if(Order_Type==4)//延时指令
					 {
							LCD_Current_Page    = 162;          			//延时输入界面						 
					 }
					 else if(Order_Type==3)//取料指令
					 {
						 Order_Type=3;
						 LCD_Main_Page       = MANUL_DEBUG;                	//LCD主界面为手动调试                    
						 LCD_Current_Page    = 6;          			//LCD当前界面为手动调试
						 g_Watch_IO_Control  = 0; 						  	    				//实时IO监视状态
						 Order_Temp[0]       = MANUL_MODE;                   //设置为手动模式
						 USART2_SendDataArray(1,WORK_MODE,&Order_Temp[0]);//串口发送工作模式中的手动模式到主控板
						 g_Robot_Auto_Reset = FALSE;
						 LC_IO_Axis_Code = 0x4010;	
					 }
					 else
					 {
						 LCD_Main_Page       = MANUL_DEBUG;                	//LCD主界面为手动调试                    
						 LCD_Current_Page    = LCD_Main_Page;          			//LCD当前界面为手动调试
						 g_Watch_IO_Control  = 0; 						  	    				//实时IO监视状态
						 Order_Temp[0]       = MANUL_MODE;                   //设置为手动模式
						 USART2_SendDataArray(1,WORK_MODE,&Order_Temp[0]);//串口发送工作模式中的手动模式到主控板
						 g_Robot_Auto_Reset = FALSE;
						 LC_IO_Axis_Code = 0x4001;				
					 }						 
					break;
				case 0x2854://返回提示是否保存程序
					if(Program_Edit_OrNot==0)//程序未修改返回
					{
						OperateProgram[gs_Current_Operate_List-1].Order_Num = Saved_Program[g_Run_Program_Num-1].Order_Num;
						Order_Num									= Saved_Program[g_Run_Program_Num-1].Order_Num;
						Free_Program_Operate.Num  = Saved_Program[g_Run_Program_Num-1].Order_Num;
						g_Refresh_Dispaly_List_Flag=TRUE;//刷新程序显示列表开
						Program_Or_Point    = 1;
						gs_Background_Color = 0;
						LCD_Current_Page    = 2;
					}
					else
						LCD_Current_Page = 126;
					break;

				case 0x26C2://程序保存确认
					Program_Save_OrNot=1;//是否保存
					g_Refresh_Dispaly_List_Flag=TRUE;//刷新程序显示列表开
				  gs_Background_Color         = 0;			
				 //程序载入中
				 LCD_Current_Page = 66;
				 LCD_Pre_Page = LCD_Current_Page;
				 PageChange(LCD_Pre_Page);
				 Program_Save();
					break;
				
				case 0x26C3://不保存(这边需要重新读一下原来的程序)
					Program_Save_OrNot=2;		
					LCD_Current_Page = 2;
					Program_Or_Point=1;
					g_Refresh_Dispaly_List_Flag=TRUE;                         //刷新程序显示列表开
				  gs_Background_Color         = 0;
				 //程序载入中
				 LCD_Current_Page = 66;
				 LCD_Pre_Page = LCD_Current_Page;
				 PageChange(LCD_Pre_Page);
				 Program_Save();				
					break;					 
				
				case 0x2820://指令查询 
				 Search_Order();
					break;
				
				case 0x2830://点查询输入点编号
					 if(LCD_Key_Value < 0x01) 
				     {
							 name_temp[3] = LCD_Var_Data;
							 name_temp[2] = LCD_Var_Data>>8;
							 name_temp[1] = LCD_Var_Data>>16;
							 name_temp[0] = LCD_Var_Data>>24;
							 DisplayChineseString(0x2830,name_temp,4);
						 }
					 break;
					 
				case 0x2805://点查询返回（点叉）
					 LCD_Current_Page = 3;
					 gs_Background_Color_ProgramEdit = 0;
					 g_Refresh_Dispaly_List_Flag = TRUE;					
				     break;	
				
				case 0x2357://取料参数设置返回
					 LCD_Current_Page = 3;
					 gs_Background_Color_ProgramEdit = 0;
					 g_Refresh_Dispaly_List_Flag = TRUE;					
				     break;						 
				
				case 0x2201:		 //上一页
					 if(Order_Num!=0)
					 {
						 if(gs_Current_Operate_List_ProgramEdit%LARGESTPROGRAMNUM==g_Background_Color_ProgramEdit&&Free_Program_Operate.Program[0].List>1)	//程序的前八行
						 {
								j=Free_Program_Operate.Program[0].List-1;
								for(i=j;i>j-LARGESTPROGRAMNUM;i--)
								{
									AT24CXX_Read(CURRENT_PROGRAM_ADR2+(i-1)*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
									/*List Value1 2 3 4  5 LC_ID Key IO_Group*/
									Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Flag  = 1;
									Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
									Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
									Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
									Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
									Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
									Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
									Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
									Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
									 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
									 {
										Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
									 }
									else if(4==IIC_Temp[26])//延时指令
									{
										Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
									}								
								}
								g_Background_Color_ProgramEdit = 1;
								gs_Current_Operate_List_ProgramEdit=j-7;							 
						 }
						 //不在前八行
						 else if(gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit<=8) //当前操作行前面没有8行，即不足单页显示
						 {
						 	gs_Current_Operate_List_ProgramEdit=1;
							g_Background_Color_ProgramEdit=1;
						 }
						 else
						 {
						 	gs_Current_Operate_List_ProgramEdit=gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+1-8;
							g_Background_Color_ProgramEdit = 1;
						 }
						 g_Refresh_Dispaly_List_Flag=TRUE;
					 }
				     break;
	
				case 0x2202:		 //下一页
					 if(Order_Num!=0)
					 {
						 if(Order_Num>(((gs_Current_Operate_List_ProgramEdit-1)/8+1)*8)) //后面还有程序，则翻页
						 {
							 if(LARGESTPROGRAMNUM-15<=gs_Current_Operate_List_ProgramEdit%LARGESTPROGRAMNUM&&gs_Current_Operate_List_ProgramEdit%LARGESTPROGRAMNUM<=LARGESTPROGRAMNUM-8)//倒数第二页
							 {
								gs_Current_Operate_List_ProgramEdit = Free_Program_Operate.Program[LARGESTPROGRAMNUM-8].List;	
								g_Background_Color_ProgramEdit = 1;
								g_Refresh_Dispaly_List_Flag=TRUE;		   								
							 }
							 else if(LARGESTPROGRAMNUM-8<=(gs_Current_Operate_List_ProgramEdit-1)%LARGESTPROGRAMNUM)//最后一页
							 {
								 if(Order_Num>LARGESTPROGRAMNUM&&(Order_Num>(Order_Num/LARGESTPROGRAMNUM)*LARGESTPROGRAMNUM))
								 {
									j=Free_Program_Operate.Program[LARGESTPROGRAMNUM-1].List;
									if(Order_Num-j>LARGESTPROGRAMNUM)
										n=j+LARGESTPROGRAMNUM;
									else
										n=Order_Num;
									for(i=j;i<n;i++)
									{
										AT24CXX_Read(CURRENT_PROGRAM_ADR2+i*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
										/*List Value1 2 3 4  5 LC_ID Key IO_Group*/
										Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Flag  = 1;
										Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
										Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
										Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
										Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
										Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
										Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
										Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
										Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
										 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
										 {
											Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
										 }
										else if(4==IIC_Temp[26])//延时指令
										{
											Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
										}										
									}
									if(i%LARGESTPROGRAMNUM!=0)
									{
										for(k=i%LARGESTPROGRAMNUM;k<LARGESTPROGRAMNUM;k++)
										{
										/*List key  value1 2 3 4 5*/
											Free_Program_Operate.Program[k].Flag  = 0;
											Free_Program_Operate.Program[k].List  = 0;
											Free_Program_Operate.Program[k].Key   = 0;
											Free_Program_Operate.Program[k].Value1= 0;
											Free_Program_Operate.Program[k].Value2= 0;
											Free_Program_Operate.Program[k].Value3= 0;
											Free_Program_Operate.Program[k].Value4= 0;
											Free_Program_Operate.Program[k].Value5= 0;										
										}
									}
									g_Background_Color_ProgramEdit = 1;
									gs_Current_Operate_List_ProgramEdit=j+1;
									g_Refresh_Dispaly_List_Flag=TRUE;		   
								}									 
							 }
							 else
							 {
								gs_Current_Operate_List_ProgramEdit = gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+8+1;
	   						g_Background_Color_ProgramEdit = 1;			
								g_Refresh_Dispaly_List_Flag=TRUE;		   								 
							 }
						 }	
					 }
					 break;
				case 0x2204:		 //第一行
					 if(gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+1>Free_Program_Operate.Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List_ProgramEdit = gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+1;
						 g_Background_Color_ProgramEdit=1;
						 g_Current_Program_Modify=FALSE;
					 }
					 break;
					 
				case 0x2205:		 //第二行
					 if(gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+2>Free_Program_Operate.Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List_ProgramEdit = gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+2;
						 g_Background_Color_ProgramEdit=2;
						 g_Current_Program_Modify=FALSE;
					 }
				     break;
	
				case 0x2206:		 //第三行
					 if(gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+3>Free_Program_Operate.Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List_ProgramEdit = gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+3;
						 g_Background_Color_ProgramEdit=3;
						 g_Current_Program_Modify=FALSE;
					 }
					 break;
					 
				case 0x2207:		 //第四行
					 if(gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+4>Free_Program_Operate.Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List_ProgramEdit = gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+4;
						 g_Background_Color_ProgramEdit=4;
						 g_Current_Program_Modify=FALSE;
				     }
					 break;
	
				case 0x2208:		 //第五行
					 if(gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+5>Free_Program_Operate.Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List_ProgramEdit = gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+5;
						 g_Background_Color_ProgramEdit=5;
						 g_Current_Program_Modify=FALSE;
				     }
					 break;	
					 
				case 0x2209:		 //第六行
					 if(gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+6>Free_Program_Operate.Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List_ProgramEdit = gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+6;
						 g_Background_Color_ProgramEdit=6;
						 g_Current_Program_Modify=FALSE;
				     }
				     break;
	
				case 0x220A:		 //第七行
					 if(gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+7>Free_Program_Operate.Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List_ProgramEdit = gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+7;
						 g_Background_Color_ProgramEdit=7;
						 g_Current_Program_Modify=FALSE;
				     }
					 break;					 

				case 0x220B:		 //第八行
					 if(gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+8>Free_Program_Operate.Num)
					 {
					 }
					 else
					 {
						 gs_Current_Operate_List_ProgramEdit = gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+8;
						 g_Background_Color_ProgramEdit=8;
						 g_Current_Program_Modify=FALSE;
				   }
				     break;	
				 			
				case 0x2621:		 //取消保存
				     g_SaveProgram_Flag=0;
					 LCD_Current_Page = 2;
					 break;	
				default:
				    break;
				}				
		 }
	}

	}
	else
	{
		if(LCD_Current_Page == START_UP)	  //主页面
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_1_HOMEFIRST;	//弹出提示框-请先回零
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	//弹出提示框-请先停止机械手
			}
		}
		else if(LCD_Current_Page == 16)	  //报警第一页
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_6_HOMEFIRST1;	//弹出提示框-请先回零
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	//弹出提示框-请先停止机械手
			}
		}	
		else if(LCD_Current_Page == 34)	  //报警第二页
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_6_HOMEFIRST2;	//弹出提示框-请先回零
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	//弹出提示框-请先停止机械手
			}
		}	
		else if(LCD_Current_Page == SYSTEM_SETTING)	  //系统设置
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_4_HOMEFIRST;	//弹出提示框-请先回零
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	//弹出提示框-请先停止机械手
			}
		}
	}
	LCD_Key_Value=0;
	LCD_Var_Data=0;
}




/*************************************************************************
**  函数名：  LCDFreeProgramDisply()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void LCDFreeProgramDisply()
{
	u8 i=0;
	//--界面切换--//
	if(LCD_Pre_Page != LCD_Current_Page )	   
	{
		LCD_Pre_Page = LCD_Current_Page;
		PageChange(LCD_Pre_Page);
	}
   /*---编辑背景---*/ 
    if(g_Background_Color != gs_Background_Color)	 //
	{
		Display_PageCut(Backgroud_Color[g_Background_Color-1]);
		gs_Background_Color = g_Background_Color;
//		LocalIOStatusDisplay();
//		gs_LocalIO_Status = g_LocalIO_Status;
	}
	/*---编辑背景-闪烁--*/
	if((gs_Current_Program_List<=Product_Num) && (Current_Operate_Program.Program[gs_Current_Program_List-1].Flag==0) && (g_Event_Done_Flag==FALSE))
	{
		Display_PageCut(Backgroud_Color[g_Background_Color-1]);
		g_Background_Blink_Delay = TRUE;
		g_Event_Done_Flag = TRUE;
		g_Background_Blink_Count=0;	
	}
	if((gs_Current_Program_List<=Product_Num) && (Current_Operate_Program.Program[gs_Current_Program_List-1].Flag==1) && (g_Event_Done_Flag==TRUE))
	{
		g_Background_Blink_Delay = FALSE;
		g_Event_Done_Flag = FALSE;	
		UnHiddenDisplay(0x2E00,0x2300);
		g_Background_Blink_Count=0;
	}
	if(Current_Operate_Program.Program[gs_Current_Program_List-1].Flag==0)
	{
		if(g_Background_Color_Blink)
		{
			HiddenDisplay(0x2E00);
		}
		else
		{
			UnHiddenDisplay(0x2E00,0x2300);
		}
	}
	/*---刷新程序列表---*/ 
	if(g_Refresh_Dispaly_List_Flag)
	{
		gs_Current_Program_Firstline=gs_Current_Operate_List+1-g_Background_Color;
		for(i=0;i<8;i++)
		{
			gs_Current_Dispaly_Code[i]=0;
		}		
		if(Product_Num<8)		 //程序总数少于8条
		{
			//获取程序显示值  --- 三位参数显示值
			for(i=0;i<Product_Num;i++)
			{
				gs_Current_Dispaly_Code[i]=OperateProgram[i+gs_Current_Program_Firstline-1].Code;//编号
				gs_Code_Parameter_Display[i][0]=OperateProgram[i+gs_Current_Program_Firstline-1].Name;//程序名
				gs_Code_Parameter_Display[i][1]=OperateProgram[i+gs_Current_Program_Firstline-1].Order_Num;//程序大小
				gs_Code_Parameter_Display[i][2]=OperateProgram[i+gs_Current_Program_Firstline-1].time;//程序保存时间
			}
//		    //获取编码显示值
//			for(i=0;i<(Current_Operate_Program.Num+1-gs_Current_Program_Firstline);i++)
//			{
//				gs_Current_Dispaly_Code[i]=Current_Operate_Program.Program[gs_Current_Operate_List+i+1-g_Background_Color].List;	
//			}		
		}
		else
		{
			//获取程序显示值
			for(i=0;i<8;i++)
			{
				gs_Current_Dispaly_Code[i]=OperateProgram[i+gs_Current_Program_Firstline-1].Code;//编号
				gs_Code_Parameter_Display[i][0]=OperateProgram[i+gs_Current_Program_Firstline-1].Name;//程序名
				gs_Code_Parameter_Display[i][1]=OperateProgram[i+gs_Current_Program_Firstline-1].Order_Num;//程序大小
				gs_Code_Parameter_Display[i][2]=OperateProgram[i+gs_Current_Program_Firstline-1].time;//程序保存时间
			}
//			//获取编码显示值
//			for(i=0;i<8;i++)
//			{
//				gs_Current_Dispaly_Code[i]=gs_Current_Operate_List+i+1-g_Background_Color;
//			}	
						
			if(gs_Current_Program_List-g_Background_Color+1+8>Product_Num)
			{
				for(i=Product_Num+g_Background_Color-gs_Current_Program_List;i<8;i++)
				{
					gs_Current_Dispaly_Code[i]=0;
				}
			}
		}

		//显示程序
	    for(i=0;i<8;i++)
	    {
			if(gs_Current_Dispaly_List[i]!=0)
			{
				DisplayChineseString(0x2020+0x20*i,Program_String[gs_Current_Dispaly_List[i]],16);
			}
			else
			{
				DisplayChineseString(0x2020+0x20*i,Program_String[0],16);	
			} 	
	    }
		//显示编码
		for(i=0;i<Product_Num;i++)
		{
			DisplayChineseString(0x2520+0x10*i,ProgramCode_String[gs_Current_Dispaly_Code[i]],4);
		}
		//显示参数值
		for(i=0;i<Product_Num;i++)
		{
			//显示参数1-程序名称
			if(gs_Code_Parameter_Display[i][0] != 0)
			{
				ValueToString(gs_Code_Parameter_Display[i][0]);
				DisplayChineseString(0x2030+0x20*i,Parameter_Value,10);
			}
			else
			{			
				DisplayChineseString(0x2030+0x20*i,Parameter_String[gs_Code_Parameter_Display[i][0]],10);
			}
		}
		for(i=0;i<Product_Num;i++)
		{
			//显示参数2 程序大小
			if(gs_Code_Parameter_Display[i][1] != 0)
			{
				ValueToString(gs_Code_Parameter_Display[i][1]);
				DisplayChineseString(0x2035+0x20*i,Parameter_Value,10);
			}
			else
			{
				DisplayChineseString(0x2035+0x20*i,Parameter_String[gs_Code_Parameter_Display[i][1]],10);
			}
		}
		for(i=0;i<Product_Num;i++)
		{
			//显示参数3-时间
			if(gs_Code_Parameter_Display[i][2] != 0)
			{
				ValueToString(gs_Code_Parameter_Display[i][2]);
				DisplayChineseString(0x203A+0x20*i,Parameter_Value,10);
			}
			else
			{
				DisplayChineseString(0x203A+0x20*i,Parameter_String[gs_Code_Parameter_Display[i][2]],10);
			}
		}
		g_Refresh_Dispaly_List_Flag = FALSE;
	}
	/*---刷新程序列表---*/

    //--程序-保存--//
	if(g_SaveProgram_Flag)			   
	{		
		Temp_Display_Data[0]=Current_Operate_Program.Name>>24;
		Temp_Display_Data[1]=Current_Operate_Program.Name>>16;
		Temp_Display_Data[2]=Current_Operate_Program.Name>>8;
		Temp_Display_Data[3]=Current_Operate_Program.Name;
		DisplayChineseString(0x2600,Temp_Display_Data,4);
		WriteNum2(0x2610,Current_Operate_Program.Num);
		g_SaveProgram_Flag=0;
	}

	//--安全区域设定--//
	if(g_Current_Choose_Area!= gs_Pre_Choose_Area )
	{	
		if(g_Current_Choose_Area== 0)
		{
			HiddenDisplay(0x5E30);
		}
		if(gs_Pre_Choose_Area == 0)
		{
			UnHiddenDisplay(0x5E30,0x5420);
		}		
		Display_PageCut(Safe_Area_List[g_Current_Choose_Area-1]);//图像剪切	
		gs_Pre_Choose_Area=g_Current_Choose_Area;
		WriteNum4(0x5078,Robot_Safe_Area[gs_Pre_Choose_Area-1].X_Left);
		WriteNum4(0x507A,Robot_Safe_Area[gs_Pre_Choose_Area-1].Z_Left);	
		WriteNum4(0x507C,Robot_Safe_Area[gs_Pre_Choose_Area-1].X_Right);
		WriteNum4(0x507E,Robot_Safe_Area[gs_Pre_Choose_Area-1].Z_Right);
	}
	//--程序管理--//
	if(gs_Program_Display_Refresh)
	{
		for(i=0;i<SAVEPROGRAMNUM;i++)
		{
			Temp_Display_Data[0]=Free_Program_Save[i].Name>>24;
			Temp_Display_Data[1]=Free_Program_Save[i].Name>>16;
			Temp_Display_Data[2]=Free_Program_Save[i].Name>>8;
			Temp_Display_Data[3]=Free_Program_Save[i].Name;
			DisplayChineseString(0x5170+4*i,Temp_Display_Data,4);
			WriteNum2(0x5172+4*i,Free_Program_Save[i].Num);
		}
		gs_Program_Display_Refresh = FALSE;	
	}

	if(g_FreeProgram_Setting_XZ)
	{
		LCDManulOperateDisply();
	}

	//--修改--//
	if(gs_Current_Program_Modify!=g_Current_Program_Modify)
	{
		if(g_Current_Program_Modify)
		{
			HiddenDisplay(0x2E02);
		}
		else
		{
		    Display_PageCut(Free_Program_Modify);
			UnHiddenDisplay(0x2E02,0x2320);			
		}
		gs_Current_Program_Modify=g_Current_Program_Modify;
	}
	//参数-速度 显示刷新
	if(gs_Refresh_Speed_Input)
	{
		WriteNum2(0x2630,Axis_Parameter_Speed); //参数-速度
		gs_Refresh_Speed_Input = FALSE;
	}
	//参数-延时 显示刷新
	if(gs_Refresh_Delay_Input)
	{
		WriteNum4(0x2633,Delay_Parameter_Time); //参数-速度
		gs_Refresh_Delay_Input = FALSE;
	}

	if((g_FreeProgram_Setting_XZ==FALSE)&&(gs_FreeProgram_Setting_XZ==TRUE))
	{
		HiddenDisplay(0x4E00);
		HiddenDisplay(0x4E01);
		HiddenDisplay(0x4E02);
		HiddenDisplay(0x4E03);
		gs_FreeProgram_Setting_XZ=g_FreeProgram_Setting_XZ;
	}

}
/*************************************************************************
**  函数名：  LCDProductManagementDisply()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    DH   
**  开发日期：2018/7/02
**************************************************************************/
u8 Program_Or_Point=0;//0 手动调试界面 1 程序管理界面 2 自由编程界面
void LCDProductManagementDisply()
{
	u8 i=0,j=0,Temp_String3[10]="       ms",Temp_String4[3]=" . ";
	//--界面切换--//
		LCD_Pre_Page = LCD_Current_Page;
		PageChange(LCD_Pre_Page);
	
	/***********************程序管理界面********************************/
	if(Program_Or_Point==0||Program_Or_Point==1)
	{
		 /*---编辑背景---*/ 
		if(g_Background_Color != gs_Background_Color)	 //
		{
			Display_PageCut(Backgroud_Color[g_Background_Color-1]);
			gs_Background_Color = g_Background_Color;
//		LocalIOStatusDisplay();
//		gs_LocalIO_Status = g_LocalIO_Status;
		}
		
		/*---刷新程序列表---*/ 
		if(g_Refresh_Dispaly_List_Flag)
		{
			gs_Current_Program_Firstline=gs_Current_Operate_List+1-g_Background_Color;
			if(Product_Num <= 8)		 //程序总数少于8条
			{
				//获取程序显示值  --- 三位参数显示值
				for(i=0;i<8;i++)
				{
					gs_code_num[i][0] = OperateProgram[i].Flag;
					gs_code_num[i][1] = OperateProgram[i].XuHao;
					gs_code_num[i][2] = OperateProgram[i].Order_Num;
					gs_name[i][3] = OperateProgram[i].Name;
					gs_name[i][2] = OperateProgram[i].Name>>8;
					gs_name[i][1] = OperateProgram[i].Name>>16;
					gs_name[i][0] = OperateProgram[i].Name>>24;
					gs_time[i] = OperateProgram[i].time;
				}	
			}
			else//获取程序显示值
			{								
				for(i=0;i<8;i++)
				{
					if(gs_Current_Operate_List+i-g_Background_Color<SAVEPROGRAMNUM)//显示赋值前先判断是否超出范围
					{
						gs_code_num[i][0] = OperateProgram[gs_Current_Operate_List+i-g_Background_Color].Flag;
						gs_code_num[i][1] = OperateProgram[gs_Current_Operate_List+i-g_Background_Color].XuHao;
						gs_code_num[i][2] = OperateProgram[gs_Current_Operate_List+i-g_Background_Color].Order_Num;
						gs_name[i][3] = OperateProgram[gs_Current_Operate_List+i-g_Background_Color].Name;
						gs_name[i][2] = OperateProgram[gs_Current_Operate_List+i-g_Background_Color].Name>>8;
						gs_name[i][1] = OperateProgram[gs_Current_Operate_List+i-g_Background_Color].Name>>16;
						gs_name[i][0] = OperateProgram[gs_Current_Operate_List+i-g_Background_Color].Name>>24;
						gs_time[i] = OperateProgram[gs_Current_Operate_List+i-g_Background_Color].time;
					}
					else
					{
						gs_code_num[i][0] = 0;
						gs_code_num[i][1] = 0;
						gs_code_num[i][2] = 0;
						gs_name[i][3] = 0;
						gs_name[i][2] = 0;
						gs_name[i][1] = 0;
						gs_name[i][0] = 0;
						gs_time[i] = 0;					
					}
				}	
				if(gs_Current_Operate_List-g_Background_Color+1+8>Product_Num)//翻页查看
				{
					for(i=Product_Num+g_Background_Color-gs_Current_Operate_List;i<8;i++)
					{
						gs_Current_Dispaly_Code[i]=0;
					}
				}
			}
			
			//显示编号
			for(i=0;i<8;i++)
			{
				if(gs_code_num[i][0] != 0)
				{
					DisplayChineseString(0x2520+0x10*i,ProgramCode_String[gs_code_num[i][1]],4);
				}
				else
				{
					DisplayChineseString(0x2520+0x10*i,Program_String[0],16);	
				} 
			}
			//显示程序名字
			for(i=0;i<8;i++)
			{
				if(gs_code_num[i][0] != 0)
				{
					ValueToName(i);
					DisplayChineseString(0x2020+0x20*i,gs_name[i],4);
				}
				else
				{
					DisplayChineseString(0x2020+0x20*i,Program_String[0],16);	
				} 	
			}
			//显示长度
			for(i=0;i<8;i++)
			{
				if(gs_code_num[i][0] != 0)
				{
					ValueToNumber(gs_code_num[i][2]);
					DisplayChineseString(0x2030+0x20*i,Parameter_Value,4); 
					for(j=0;j<10;j++)
						Parameter_Value[j]=0;
				}
				else
				{
					DisplayChineseString(0x2030+0x20*i,Program_String[0],16);	
				}
			}
			//显示日期
			for(i=0;i<8;i++)
			{
				if(gs_code_num[i][0] != 0)
				{
					ValueToTime(gs_time[i]);
					DisplayChineseString(0x2035+0x20*i,Parameter_Value,12); 
					for(j=0;j<10;j++)
						Parameter_Value[j]=0;
				}
				else
				{
					DisplayChineseString(0x2035+0x20*i,Program_String[0],16);	
				}
			}
			g_Refresh_Dispaly_List_Flag = FALSE;
		}
		/*---刷新程序列表--White_L---*/
		
		//--程序-产品选择--//
		if(Select_Program_Flag)
		{
			DisplayChineseString(0x2120,gs_name[Current_Select_Program%8],4);
			Select_Program_Flag = FALSE;
		}
		
			//--程序-保存--//
		if(g_SaveProgram_Flag)			   
		{		
			Temp_Display_Data[0]=Current_Operate_Program.Name>>24;
			Temp_Display_Data[1]=Current_Operate_Program.Name>>16;
			Temp_Display_Data[2]=Current_Operate_Program.Name>>8;
			Temp_Display_Data[3]=Current_Operate_Program.Name;
			DisplayChineseString(0x2600,Temp_Display_Data,4);
			WriteNum2(0x2610,Current_Operate_Program.Num);
			g_SaveProgram_Flag=0;
		}
		//--程序管理--//
		if(gs_Program_Display_Refresh)
		{
			for(i=0;i<SAVEPROGRAMNUM;i++)
			{
				Temp_Display_Data[0]=Free_Program_Save[i].Name>>24;
				Temp_Display_Data[1]=Free_Program_Save[i].Name>>16;
				Temp_Display_Data[2]=Free_Program_Save[i].Name>>8;
				Temp_Display_Data[3]=Free_Program_Save[i].Name;
				DisplayChineseString(0x5170+4*i,Temp_Display_Data,4);
				WriteNum2(0x5172+4*i,Free_Program_Save[i].Num);
			}
			gs_Program_Display_Refresh = FALSE;	
		}
	}
	/*********************************************自由编程界面********************************************/
	else if(Program_Or_Point==2)
	{
		
		 /*---编辑背景---*/ 
		if(g_Background_Color_ProgramEdit != gs_Background_Color_ProgramEdit)	 //
		{
			Display_PageCut(Backgroud_Color_FreeProgram[g_Background_Color_ProgramEdit-1]);
			gs_Background_Color_ProgramEdit = g_Background_Color_ProgramEdit;
		}
		/*---刷新指令列表---*/ 
		if(g_Refresh_Dispaly_List_Flag)
		{
			gs_Current_Program_Firstline=((gs_Current_Operate_List_ProgramEdit-1)/8)*8;
			if(Free_Program_Operate.Num <= 8)		 //程序总数少于8条
			{
				//获取程序显示值  --- 三位参数显示值
				for(i=0;i<Free_Program_Operate.Num;i++)
				{
					gs_code_num[i][0] = Free_Program_Operate.Program[i].List;
					gs_code_num[i][1] = Free_Program_Operate.Program[i].Key;
					gs_code_num[i][2] = Free_Program_Operate.Program[i].LC_ID;
					gs_code_num[i][3] = LC_Parameter_Load[gs_code_num[i][2]].Glue_Time;
					if(Free_Program_Operate.Program[i].Key==4)//延时指令
						Point_Parameter_Display[i][0] = Free_Program_Operate.Program[i].Delay_Time;						
					else
					{
						Point_Parameter_Display[i][0] = (Free_Program_Operate.Program[i].Value1|0xf0<<24);///JXS_Parameter.GearRatio.Ratio[0]
						Point_Parameter_Display[i][1] = (Free_Program_Operate.Program[i].Value2|0xf0<<24);///JXS_Parameter.GearRatio.Ratio[1]
						Point_Parameter_Display[i][2] = (Free_Program_Operate.Program[i].Value3|0xf0<<24);///JXS_Parameter.GearRatio.Ratio[2]
						Point_Parameter_Display[i][3] = (Free_Program_Operate.Program[i].Value4|0xf0<<24);///JXS_Parameter.GearRatio.Ratio[4]
						Point_Parameter_Display[i][4] = (Free_Program_Operate.Program[i].Value5|0xf0<<24);///JXS_Parameter.GearRatio.Ratio[5]
					}
				}	
				for(j=i;j<8;j++)
				{
						gs_code_num[j][0] = 0;
						gs_code_num[j][1] = 0;
						gs_code_num[j][2] = 0;
						gs_code_num[j][3] = 0;
						Point_Parameter_Display[j][0] = 0;					
						Point_Parameter_Display[j][1] = 0;					
						Point_Parameter_Display[j][2] = 0;					
						Point_Parameter_Display[j][3] = 0;					
						Point_Parameter_Display[j][4] = 0;					
						Point_Parameter_Display[j][5] = 0;					
				}
			}
			else//获取程序显示值
			{								
				for(i=0;i<8;i++)
				{
					if(gs_Current_Program_Firstline+i<Free_Program_Operate.Num)//显示赋值前先判断是否超出范围
					{
						gs_code_num[i][0] = Free_Program_Operate.Program[(gs_Current_Program_Firstline+i)%LARGESTPROGRAMNUM].List;
						gs_code_num[i][1] = Free_Program_Operate.Program[(gs_Current_Program_Firstline+i)%LARGESTPROGRAMNUM].Key;
						gs_code_num[i][2] = Free_Program_Operate.Program[(gs_Current_Program_Firstline+i)%LARGESTPROGRAMNUM].LC_ID;
						gs_code_num[i][3] = LC_Parameter_Load[gs_code_num[i][2]].Glue_Time;
						if(gs_code_num[i][1]==4)//延时指令
							Point_Parameter_Display[i][0] = Free_Program_Operate.Program[(gs_Current_Program_Firstline+i)%LARGESTPROGRAMNUM].Delay_Time;						
						else
						{
							Point_Parameter_Display[i][0] = (u32)(Free_Program_Operate.Program[(gs_Current_Program_Firstline+i)%LARGESTPROGRAMNUM].Value1|0xf0<<24);
							Point_Parameter_Display[i][1] = (u32)(Free_Program_Operate.Program[(gs_Current_Program_Firstline+i)%LARGESTPROGRAMNUM].Value2|0xf0<<24);
							Point_Parameter_Display[i][2] = (u32)(Free_Program_Operate.Program[(gs_Current_Program_Firstline+i)%LARGESTPROGRAMNUM].Value3|0xf0<<24);
							Point_Parameter_Display[i][3] = (u32)(Free_Program_Operate.Program[(gs_Current_Program_Firstline+i)%LARGESTPROGRAMNUM].Value4|0xf0<<24);
							Point_Parameter_Display[i][4] = (u32)(Free_Program_Operate.Program[(gs_Current_Program_Firstline+i)%LARGESTPROGRAMNUM].Value5|0xf0<<24);
						}
					}
					else
					{
						gs_code_num[i][0] = 0;
						gs_code_num[i][1] = 0;
						gs_code_num[i][2] = 0;
						gs_code_num[i][3] = 0;
						Point_Parameter_Display[i][0] = 0;				
						Point_Parameter_Display[i][1] = 0;				
						Point_Parameter_Display[i][2] = 0;				
						Point_Parameter_Display[i][3] = 0;				
						Point_Parameter_Display[i][4] = 0;				
					}
				}	
//				if(gs_Current_Operate_List_ProgramEdit-g_Background_Color_ProgramEdit+1+8>Point_Num)//翻页查看
//				{
//					for(i=Point_Num+g_Background_Color_ProgramEdit-gs_Current_Operate_List_ProgramEdit;i<8;i++)
//					{
//						gs_Current_Dispaly_Code[i]=0;
//					}
//				}
			}
			//显示指令编号
			for(i=0;i<8;i++)
			{
				if(gs_code_num[i][0] != 0)
				{
					DisplayChineseString(0x2150+4*i,ProgramCode_String[gs_code_num[i][0]],4);
				}
				else
				{
					DisplayChineseString(0x2150+4*i,Program_String[0],4);	
				} 
			}
			//显示指令类型
			for(i=0;i<8;i++)
			{
				if(gs_code_num[i][0] != 0)
				{
					DisplayChineseString(0x2900+0x20*i,Point_String[gs_code_num[i][1]],8);
					for(j=0;j<10;j++)
						Parameter_Value[j]=0;
				}
				else
				{
					DisplayChineseString(0x2900+0x20*i,"    ",8);
				} 	
			}
			//显示料仓ID
			for(i=0;i<8;i++)
			{
				if(gs_code_num[i][0] != 0)
				{
					DisplayChineseString(0x2910+0x20*i,LC_String[gs_code_num[i][2]],6);
					for(j=0;j<10;j++)
						Parameter_Value[j]=0;
				}
				else
				{
					DisplayChineseString(0x2910+0x20*i,LC_String[0],6);
				} 	
			}
			//显示出胶量单位为时间
			for(i=0;i<8;i++)
			{
				if(gs_code_num[i][3] != 0)
				{
					Temp_String4[0]=(char)(gs_code_num[i][3]/1000+'0');
					Temp_String4[2]=(char)((gs_code_num[i][3]/100)%10+'0');
					DisplayChineseString(0x2D10+0x10*i,Temp_String4,10);
					Temp_String4[0]=0x20;
					Temp_String4[2]=0x20;
				}
				else
				{
					DisplayChineseString(0x2D10+0x10*i,"          ",10);
				} 	
			}
			//显示坐标参数
			for(i=0;i<8;i++)
			{
				if(gs_code_num[i][0] != 0)
				{
					if(gs_code_num[i][1]==4)//延时指令
					{
						Temp_String3[4]=(char)(Point_Parameter_Display[i][0]/100+'0');
						Temp_String3[5]=(char)((Point_Parameter_Display[i][0]/10)%10+'0');
						Temp_String3[6]=(char)(Point_Parameter_Display[i][0]%10+'0');
						if(Temp_String3[4]==0x30)
							Temp_String3[4]=0x20;
						else if(Temp_String3[4]==0x30&&Temp_String3[5]==0x30)
						{
							Temp_String3[4]=0x20;
							Temp_String3[5]=0x20;							
						}
						DisplayChineseString(0x2A00+0x10*i,Temp_String3,10);
						for(j=0;j<10;j++)
							Parameter_Value[j]=0;
						DisplayChineseString(0x2D10+0x10*i,"   ",3);
						DisplayChineseString(0x2A80+0x10*i,"   ",10);
						DisplayChineseString(0x2B00+0x10*i,"   ",10);
						DisplayChineseString(0x2C20+0x10*i,"   ",10);
						DisplayChineseString(0x2E10+0x10*i,"   ",10);						
					}
					else
					{
						ValueToString(Point_Parameter_Display[i][0]);
						DisplayChineseString(0x2A00+0x10*i,Parameter_Value,10);
						for(j=0;j<10;j++)
							Parameter_Value[j]=0;						
						ValueToString(Point_Parameter_Display[i][1]);
						DisplayChineseString(0x2A80+0x10*i,Parameter_Value,10);
						for(j=0;j<10;j++)
							Parameter_Value[j]=0;
						ValueToString(Point_Parameter_Display[i][2]);
						DisplayChineseString(0x2B00+0x10*i,Parameter_Value,10);
						for(j=0;j<10;j++)
							Parameter_Value[j]=0;
						ValueToString(Point_Parameter_Display[i][3]);
						DisplayChineseString(0x2C20+0x10*i,Parameter_Value,10);
						for(j=0;j<10;j++)
							Parameter_Value[j]=0;
						ValueToString(Point_Parameter_Display[i][4]);
						DisplayChineseString(0x2E10+0x10*i,Parameter_Value,10);
					}
					for(j=0;j<10;j++)
						Parameter_Value[j]=0;
				}
				else
				{
					DisplayChineseString(0x2A00+0x10*i,"    ",10); 
					DisplayChineseString(0x2A80+0x10*i,"    ",10);
					DisplayChineseString(0x2B00+0x10*i,"    ",10);
					DisplayChineseString(0x2C20+0x10*i,"    ",10);
					DisplayChineseString(0x2E10+0x10*i,"    ",10);
				}
			}	
			g_Refresh_Dispaly_List_Flag = FALSE;
		}			
	}
	
}
