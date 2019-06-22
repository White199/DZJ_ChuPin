/***********************************
 *           SDO Protocol          *
 ***********************************/
#define INITIATE_SDO_DOWNLOAD   0x20             //SDO启动下载：写入Servo数据[4:7]:data-reponse:0x60 
#define INITIATE_SDO_UPLOAD     0x40             //SDO启动上传：读取Servo数据-reponse:0x40 [4:7]:data
//#define DOWNLOAD_SDO_SEGMENT                   //SDO段下载
//#define UPLOAD_SDO_SEGMENT                     //SDO段上传
#define ABORT_SDO_TRANSFER      0x80             //中止SDO通信   [4:7]:abort codes

void Initiate_SDO_Download(void);
void Initiate_SDO_Upload(void);      
void Abort_SDO_Transfer(void);
void OD_Write(u8*,u16,u8);           //data,index,subindex
void SDO_Protocol(u8);               //SDO通信协议
void SDO_Process(u8);              //SDO消息处理函数
void Set_Servo_PP_Mode(u8);          //伺服器Profile Position位置模式配置	  
void Set_Servo_Homing_Mode(u8 );	 //伺服器Homing Mode回原点模式配置
void Set_Guard_Time(u8);
void Set_PUU(u8,u32,u32);
void Servo_Off(u8);
void Fault_Reset(u8);
void Servo_Off(u8);
void Servo_On(u8);
void Query_Servo_parameter(u8,u16,u8);
void Change_Polarity_Positive(u8);
void Change_Polarity_Negtive(u8);
//---
void SDO_Process_PositionMode(u8);
void SDO_Process_HomingMode(u8);

