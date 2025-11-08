/*************  功能说明    **************

下载时, 选择时钟 12MHZ (用户可自行修改频率)。

1.用RTC定时唤醒MCU，1秒唤醒1次，唤醒后用比较器判断外部电压：1，正常，正常工作；2，如电压偏低，继续休眠，主时钟停止震荡，RTC继续工作.

2.比较器正极通过可调电阻分压后输入到P3.7口，比较器负极使用内部1.19V参考电压.

3.该分压电路的地用I/O(P3.5)控制，I/O设置为开漏，不比较时，对外设置为1，I/O口浮空，省电；比较时，对外输出0，就是地！

4.中景园2.25inch-76x284-LCD-ZJY225KP屏幕驱动


******************************************/
#include	"\..\library\config.h"
#include	"\..\library\STC32G_Timer.h"
#include	"\..\library\STC32G_GPIO.h"
#include	"\..\library\STC32G_UART.h"
//#include	"\..\library\STC32G_NVIC.h"//中断
#include	"\..\library\STC32G_Switch.h"//功能脚切换
#include	"\..\library\STC32G_CAN.h"
#include	"\..\library\STC32G_Clock.h"
#include    "\..\library\STC32G_EEPROM.h"
#include    "\..\library\STC32G_Delay.h"
#include	"\..\library\STC32G_RTC.h"
#include	"\..\library\STC32G_Compare.h"
#include    "ntc_sampling.h"
#include 	"\..\HARDWARE\LCD\lcd.h"
#include 	"\..\HARDWARE\LCD\pic.h"



/*************  本地常量声明    **************/
#define     EE_ADDRESS  0x000000  //保存的EEPROM起始地址
#define     LargeCapacitor  0   //0: 滤波电容比较小,  1: 滤波电容比较大
//extern u16 	cnt_ms;
extern bit T0_1ms;
extern bit T0_1S;//1s标志
extern bit T0_5S;//5s标志

/*************  IO口定义    **************/


/*************  本地变量声明    **************/


u8  sava_data[2];     //存储数组
u8  hour,minute,second; //RTC变量
u16 msecond;

char code *STCRTC  = "@STCRTC#";    //= "@STCRTC#";  命令头   
char indexrtc=0;                    //当前的命令头索引
char length =0;                     //长度
char rtctime[12] ;               //rtc时间数据
bit  Rec_OK = 0;                    //rtc时间获取完成标志

/*************  本地函数声明    **************/

void	Timer_config(void);
void    GPIO_config(void);
void	UART_config(void);
void    PWM_capture(void);
void    CAN_config(void);
void 	RTC_config(void);
void 	CMP_config(void);
void 	CMP_disable(void);
void	Ext_Vcc_Det(void);
void 	WriteRTC(void);
void    LCDShowTest(void);
void    RX1_Check(void);
/****************  外部函数声明和外部变量声明 *****************/

extern bit B_1S;//RTC
extern bit B_Alarm;//RTC

/******************** 主函数 **************************/
void main(void)
{
   
    u8  n,i,j;
    u8  sr;
    WTST = 0;  //设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
    EAXFR = 1; //扩展寄存器(XFR)访问使能
    CKCON = 0; //提高访问XRAM速度

	GPIO_config();
	Timer_config();
//	UART1_config(2);//寄存器串口
	UART_config();//函数版串口
	PWM_capture();
	ADC_config();
	RTC_config();
	CMP_config();
//	CAN_config();

    // 启用外部12MHz晶振，分频系数为0（即12MHz）
    //XOSCClkConfig(0);
	
    EA = 1;     //打开总中断
//	EEPROM_read_n(EE_ADDRESS,sava_data,2);        //读出2字节
//	pinly_base = ((u32)sava_data[0] << 24) + ((u32)sava_data[1]<< 16) + ((u16)sava_data[2]<< 8)  + ((u8)sava_data[3]);
//	pinly_base = ((u16)sava_data[0]<< 8)  + ((u8)sava_data[1]);
//	if(pinly_base > 4000 && pinly_base < 30000) 
//    {
//        printf("读取pinly_base:%hd \r\n", pinly_base);
//    }
//    else 
//    {
//        pinly_base = 6160; // 默认值
//        printf("读取pinly_base错误,使用默认值:%hd \r\n", pinly_base);
//    }
//	#if (LargeCapacitor == 0)   //滤波电容比较小，电容保持时间比较短，则先擦除
//        EEPROM_SectorErase(EE_ADDRESS); //擦除一个扇区
//    #endif
	//====初始化数据=====
	V_JRP = 0;
    while (1)
    {

		 if(B_1S)
        {
            B_1S = 0;
            printf("Year=20%d,Month=%d,Day=%d,Hour=%d,Minute=%d,Second=%d\r\n",YEAR,MONTH,DAY,HOUR,MIN,SEC);
            Ext_Vcc_Det();  //每秒钟检测一次外部电源，如果外部电源连接则工作，外部电源断开则进入休眠模式
        }

        if(B_Alarm)
        {
            B_Alarm = 0;
            printf("RTC Alarm!\r\n");
        }
        
    }
}

/******************* PWM1P输入捕获 *******************/
void    PWM_capture(void)
{

	PWMA_CCER1=0x00;
	PWMA_CCMR1=0x01;//CC1为输入模式,且映射到TI1FP1 上
	PWMA_CCMR2=0x02;//CC2为输入模式,且映射到TI1FP2 上
	PWMA_CCER1=0x11;//使能CC1，CC2上的捕获功能
	PWMA_CCER1|=0x00;//设置捕获极性为CC1的上升沿
	PWMA_CCER1|=0x20;//设置捕获极性为CC1的下降沿
	PWMA_SMCR=0x54;//TS=TI1FP1,SMS=TI1上升沿复位模式
	PWMA_CR1=0x01;
	
	PWMA_IER=0x06;	 //使能CC1，CC2的捕获中断
//	PWMA_CCER2=0x00;
//	PWMA_CCMR3=0x01;//CC3为输入模式,且映射到TI3FP3 上
//	PWMA_CCMR4=0x02;//CC4为输入模式,且映射到TI3FP4 上
//	PWMA_CCER2=0x11;//使能CC3，CC4上的捕获功能
//	PWMA_CCER2|=0x00;//设置捕获极性为CC3的上升沿
//	PWMA_CCER2|=0x20;//设置捕获极性为CC4的下降沿
////	PWMA_SMCR=0x54;//TS=TI1FP1,SMS=TI1上升沿复位模式
//	PWMA_CR1=0x01;
//	PWMA_IER=0x1e;	 //使能CC1,CC2,CC3,CC4的捕获中断
	
}
/******************* IO配置函数 *******************/
void	GPIO_config(void)
{
	P0_MODE_IO_PU(GPIO_Pin_All);//准双向口
	P1_MODE_IO_PU(GPIO_Pin_All);//准双向口
//	P1_MODE_IO_PU(GPIO_Pin_LOW| GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);//准双向口
//	P1_MODE_OUT_PP(GPIO_Pin_4);//P14电热膜开关，推挽输出	
	P2_MODE_IO_PU(GPIO_Pin_All);//准双向口  
	P3_MODE_IO_PU(GPIO_Pin_LOW);//准双向口  P30P31烧录口
	P3_MODE_OUT_OD(GPIO_Pin_5);//P35开漏输出
	P3_MODE_IN_HIZ(GPIO_Pin_7);//P37高阻输入
	P4_MODE_IO_PU(GPIO_Pin_All);//准双向口  
	P5_MODE_IO_PU(GPIO_Pin_All);//准双向口  
	P6_MODE_IO_PU(GPIO_Pin_All);//准双向口 
	P7_MODE_IO_PU(GPIO_Pin_All);//准双向口  

	//CAN1_SW(CAN1_P00_P01);				//CAN1_P00_P01,CAN1_P50_P51,CAN1_P42_P45,CAN1_P70_P71


}

/***************  串口初始化函数 *****************/
void	UART_config(void)
{
	COMx_InitDefine		COMx_InitStructure;					//结构定义
	COMx_InitStructure.UART_Mode      = UART_8bit_BRTx;		//模式,   UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
	COMx_InitStructure.UART_BRT_Use   = BRT_Timer1;			//选择波特率发生器, BRT_Timer1,BRT_Timer2 (注意: 串口2固定使用BRT_Timer2)
	COMx_InitStructure.UART_BaudRate  = 115200ul;			//波特率,     110 ~ 115200
	COMx_InitStructure.UART_RxEnable  = ENABLE;				//接收允许,   ENABLE或DISABLE
	UART_Configuration(UART1, &COMx_InitStructure);		//初始化串口 UART1,UART2,UART3,UART4
	NVIC_UART1_Init(ENABLE,Priority_1);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3

	UART1_SW(UART1_SW_P30_P31);		//UART1_SW_P30_P31,UART1_SW_P36_P37,UART1_SW_P16_P17,UART1_SW_P43_P44
}

/************************ 定时器配置 ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;						//结构定义
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));		//中断频率, 1000次/秒
	TIM_InitStructure.TIM_PS        = 0;					//8位预分频器(n+1), 0~255
	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//初始化Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer0_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}

/******************** CAN 配置 ********************/
void CAN_config(void)
{
    CAN_InitTypeDef	CAN_InitStructure;			//结构定义

    CAN_InitStructure.CAN_Enable = ENABLE;		//CAN功能使能   ENABLE或DISABLE
    // CAN_InitStructure.CAN_IMR    = CAN_ALLIM;	//CAN中断寄存器 	CAN_DOIM,CAN_BEIM,CAN_TIM,CAN_RIM,CAN_EPIM,CAN_EWIM,CAN_ALIM,CAN_ALLIM,DISABLE
      // 关闭所有CAN中断（包括接收中断）
    CAN_InitStructure.CAN_IMR    = DISABLE;	//CAN中断寄存器 	禁用所有中断
    CAN_InitStructure.CAN_SJW    = 0;			//重新同步跳跃宽度  0~3
    CAN_InitStructure.CAN_SAM    = 0;			//总线电平采样次数  0:采样1次; 1:采样3次

    //CAN总线波特率=Fclk/((1+(TSG1+1)+(TSG2+1))*(BRP+1)*2)
    CAN_InitStructure.CAN_TSG1   = 2;			//同步采样段1       0~15
    CAN_InitStructure.CAN_TSG2   = 1;			//同步采样段2       1~7 (TSG2 不能设置为0)
    CAN_InitStructure.CAN_BRP    = 3;			//波特率分频系数    0~63
    //示例24000000/((1+3+2)*4*2)=500KHz
	//CAN总线函数  12000000/((1+3+2)*4*2)=250KHz

    CAN_InitStructure.CAN_ListenOnly = DISABLE;	//Listen Only模式   ENABLE,DISABLE
    CAN_InitStructure.CAN_Filter  = SINGLE_FILTER;//滤波选择  DUAL_FILTER(双滤波),SINGLE_FILTER(单滤波)
    CAN_InitStructure.CAN_ACR0    = 0x00;		//总线验收代码寄存器 0~0xFF
    CAN_InitStructure.CAN_ACR1    = 0x00;
    CAN_InitStructure.CAN_ACR2    = 0x00;
    CAN_InitStructure.CAN_ACR3    = 0x00;
    CAN_InitStructure.CAN_AMR0    = 0xff;		//总线验收屏蔽寄存器 0~0xFF
    CAN_InitStructure.CAN_AMR1    = 0xff;
    CAN_InitStructure.CAN_AMR2    = 0xff;
    CAN_InitStructure.CAN_AMR3    = 0xff;
    CAN_Inilize(CAN1,&CAN_InitStructure);		//CAN1 初始化

    // //只接收 ID 为 0x280 ~ 0x2ff 的标准帧报文
    // CAN_InitStructure.CAN_ACR0    = 0x50;		//总线验收代码寄存器 0~0xFF
    // CAN_InitStructure.CAN_ACR1    = 0x00;
    // CAN_InitStructure.CAN_ACR2    = 0x00;
    // CAN_InitStructure.CAN_ACR3    = 0x00;
    // CAN_InitStructure.CAN_AMR0    = 0x0f;		//总线验收屏蔽寄存器 0~0xFF
    // CAN_InitStructure.CAN_AMR1    = 0xef;
    // CAN_InitStructure.CAN_AMR2    = 0xff;
    // CAN_InitStructure.CAN_AMR3    = 0xff;
    // CAN_Inilize(CAN2,&CAN_InitStructure);		//CAN2 初始化

    // NVIC_CAN_Init(CAN1,ENABLE,Priority_1);		//中断使能, CAN1/CAN2; ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
    // NVIC_CAN_Init(CAN2,ENABLE,Priority_1);		//中断使能, CAN1/CAN2; ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
     // 关闭CAN接收中断
    NVIC_CAN_Init(CAN1,DISABLE,Priority_1);		//中断禁用
}

/****************  RTC初始化函数 *****************/
void	RTC_config(void)
{
	RTC_InitTypeDef		RTC_InitStructure;
	RTC_InitStructure.RTC_Clock  = RTC_X32KCR;	//RTC 时钟, RTC_IRC32KCR, RTC_X32KCR
	RTC_InitStructure.RTC_Enable = ENABLE;			//RTC功能使能,   ENABLE, DISABLE
	RTC_InitStructure.RTC_Year   = 25;					//RTC 年, 00~99, 对应2000~2099年
	RTC_InitStructure.RTC_Month  = 12;					//RTC 月, 01~12
	RTC_InitStructure.RTC_Day    = 31;					//RTC 日, 01~31
	RTC_InitStructure.RTC_Hour   = 23;					//RTC 时, 00~23
	RTC_InitStructure.RTC_Min    = 59;					//RTC 分, 00~59
	RTC_InitStructure.RTC_Sec    = 55;					//RTC 秒, 00~59
	RTC_InitStructure.RTC_Ssec   = 00;					//RTC 1/128秒, 00~127

	RTC_InitStructure.RTC_ALAHour= 00;					//RTC 闹钟时, 00~23
	RTC_InitStructure.RTC_ALAMin = 00;					//RTC 闹钟分, 00~59
	RTC_InitStructure.RTC_ALASec = 00;					//RTC 闹钟秒, 00~59
	RTC_InitStructure.RTC_ALASsec= 00;					//RTC 闹钟1/128秒, 00~127
	RTC_Inilize(&RTC_InitStructure);
	NVIC_RTC_Init(RTC_ALARM_INT|RTC_SEC_INT,Priority_0);		//中断使能, RTC_ALARM_INT/RTC_DAY_INT/RTC_HOUR_INT/RTC_MIN_INT/RTC_SEC_INT/RTC_SEC2_INT/RTC_SEC8_INT/RTC_SEC32_INT/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}

/************************ 比较器配置 ****************************/
void	CMP_config(void)
{
	CMP_InitDefine CMP_InitStructure;						//结构定义
	CMP_InitStructure.CMP_EN = ENABLE;								//允许比较器		ENABLE,DISABLE
	CMP_InitStructure.CMP_P_Select     = CMP_P_P37;		//比较器输入正极选择, CMP_P_P37/CMP_P_P50/CMP_P_P51, CMP_P_ADC: 由ADC模拟输入端做正输入.
	CMP_InitStructure.CMP_N_Select     = CMP_N_GAP;		//比较器输入负极选择, CMP_N_GAP: 选择内部BandGap经过OP后的电压做负输入, CMP_N_P36: 选择P3.6做负输入.
	CMP_InitStructure.CMP_InvCMPO      = DISABLE;			//比较器输出取反, 	ENABLE,DISABLE
	CMP_InitStructure.CMP_100nsFilter  = ENABLE;			//内部0.1us滤波,  	ENABLE,DISABLE
	CMP_InitStructure.CMP_Outpt_En     = ENABLE;			//允许比较结果输出,ENABLE,DISABLE
	CMP_InitStructure.CMP_OutDelayDuty = 16;					//比较结果变化延时周期数, 0~63
	CMP_Inilize(&CMP_InitStructure);				//初始化比较器
	NVIC_CMP_Init(RISING_EDGE|FALLING_EDGE,Priority_0);	//中断使能, RISING_EDGE/FALLING_EDGE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}
/************************ 比较器配置 ****************************/
void	CMP_disable(void)
{
	CMP_InitDefine CMP_InitStructure;						//结构定义
	CMP_InitStructure.CMP_EN = DISABLE;								//允许比较器		ENABLE,DISABLE
	CMP_InitStructure.CMP_P_Select     = CMP_P_P37;		//比较器输入正极选择, CMP_P_P37/CMP_P_P50/CMP_P_P51, CMP_P_ADC: 由ADC模拟输入端做正输入.
	CMP_InitStructure.CMP_N_Select     = CMP_N_GAP;		//比较器输入负极选择, CMP_N_GAP: 选择内部BandGap经过OP后的电压做负输入, CMP_N_P36: 选择P3.6做负输入.
	CMP_InitStructure.CMP_InvCMPO      = DISABLE;			//比较器输出取反, 	ENABLE,DISABLE
	CMP_InitStructure.CMP_100nsFilter  = ENABLE;			//内部0.1us滤波,  	ENABLE,DISABLE
	CMP_InitStructure.CMP_Outpt_En     = ENABLE;			//允许比较结果输出,ENABLE,DISABLE
	CMP_InitStructure.CMP_OutDelayDuty = 16;					//比较结果变化延时周期数, 0~63
	CMP_Inilize(&CMP_InitStructure);				//初始化比较器
	NVIC_CMP_Init(RISING_EDGE|FALLING_EDGE,Priority_0);	//中断使能, RISING_EDGE/FALLING_EDGE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}

void PWMA_ISR() interrupt 26
{
	unsigned int cnt;
	if(PWMA_SR1 & 0x02)
	{
		PWMA_SR1 &= ~0x02;
		cnt = (PWMA_CCR1H <<8) + PWMA_CCR1L;  //CC1捕获周期宽度
//		zhouqi = cnt;
		
	}
	
	if(PWMA_SR1 & 0x04)
	{
		PWMA_SR1 &= ~0x04;
		cnt = (PWMA_CCR2H <<8) + PWMA_CCR2L;  //CC2捕获占空比（高电平宽度）
//		zhankongbi = cnt;
	}
//	if(PWMA_SR1 & 0x08)
//	{
//		PWMA_SR1 &= ~0x08;
//		cnt = (PWMA_CCR3H <<8) + PWMA_CCR3L;  //CC3捕获周期宽度
//		zhouqi = cnt;
//	}
//	
//	if(PWMA_SR1 & 0x10)
//	{
//		PWMA_SR1 &= ~0x10;
//		cnt = (PWMA_CCR4H <<8) + PWMA_CCR4L;  //CC4捕获占空比（高电平宽度）
//		zhankongbi = cnt;
//	}
}

/********************** 写RTC函数 ************************/
void WriteRTC(void)
{
//	printf("%02d%02d年%d月%d日\t",(int)rtctime[0],(int)rtctime[1],(int)rtctime[2],(int)rtctime[3]);
//	printf("%d：%d：%d.%d\t",(int)rtctime[5],(int)rtctime[6],(int)rtctime[7],(int)((u16)(rtctime[9]*256)|(u8)rtctime[10]));
//	printf("%d\r\n",(int)rtctime[8]);
//            INIYEAR = rtctime[1];     //Y:2021            //单片机内部rtc时钟初始化的办法
//            INIMONTH = rtctime[2];    //M:12
//            INIDAY = rtctime[3];      //D:31
//            INIHOUR = rtctime[5];     //H:23
//            INIMIN =rtctime[6];      //M:59
//            INISEC = rtctime[7];      //S:50
//            INISSEC = rtctime[8];      //S/128:0
//            RTCCFG |= 0x01;   //触发RTC寄存器初始化
            
    INIYEAR = rtctime[1];   
    INIMONTH = rtctime[2];
    INIDAY = rtctime[3];
    INIHOUR = rtctime[5];   //修改时分秒
    INIMIN = rtctime[6];
    INISEC = rtctime[7];
    INISSEC = rtctime[8];
    RTCCFG |= 0x01;   //触发RTC寄存器初始化
}

//========================================================================
// 函数: void Ext_Vcc_Det(void)
// 描述: 外部电源检测函数。
// 参数: 无.
// 返回: 无.
// 版本: V1.0, 2022-10-10
//========================================================================
void Ext_Vcc_Det(void)
{
    P35 = 0;        //比较时，对外输出0，做比较电路的地线
    CMP_config();    //使能比较器模块
    _nop_();
    _nop_();
    _nop_();
    if(CMPRES)     //判断是否CMP+电平高于CMP-，外部电源连接
    {
        P40 = 0;		//LED Power On
		printf("外部电源连接\r\n"); 
		if(T0_5S)
		{
			printf("T0_5S:%d\r\n",T0_5S);  
			T0_5S =0;		
			LCD_Init();	
//			V_JRP = 0;	
			printf("V_JRP:%d\r\n",V_JRP); 
		}
		if(T0_1S)
		{
            printf("T0_1S:%d\r\n",T0_1S); 
            T0_1S =0;
      
//			if(P32==0)   printf("P32:%d\r\n",P32); 
//			else printf("P32:%d\r\n",P32); 
//			if(P74==0)   printf("P74:%d\r\n",P74); 
//			else printf("P74:%d\r\n",P74); 
//			if(P75==0)   printf("P75:%d\r\n",P75); 
//			else printf("P75:%d\r\n",P75); 
//			if(P76==0)   printf("P76:%d\r\n",P76); 
//			else printf("P76:%d\r\n",P76); 
//			if(P77==0)   printf("P77:%d\r\n",P77); 
//			else printf("P77:%d\r\n",P77); 	
		if( Rec_OK==1 )
			{
			WriteRTC();
            Rec_OK = 0;            
			}   			
		
		}
		if(T0_1ms)
		{
			T0_1ms = 0;
			if(COM1.RX_TimeOut > 0)		//超时计数
			{
				if(--COM1.RX_TimeOut == 0)
				{
					if(COM1.RX_Cnt > 0)
					{
						RX1_Check();

					}
					COM1.RX_Cnt = 0;
				}
			}
		}
    }
    else
    {
		printf("休眠\r\n"); 
		CMP_disable();
//        CMPEN = 0;      //关闭比较器模块
//        P35 = 1;        //不比较时，对外设置为1，I/O口浮空，省电(浮空后MCU无法唤醒)
        P40 = 1;		//LED Power Off
        _nop_();
        _nop_();
        PCON = 0x02;  //STC32G 芯片使用内部32K时钟，休眠无法唤醒
        _nop_();
        _nop_();
        _nop_();
        _nop_();
        _nop_();
        _nop_();
    }
}
void    LCDShowTest(void)
{
		LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
        LCD_ShowPicture(86, 0, 105, 56, gImage_1);
        LCD_ShowString(13, 60, "2.25 TFT RESOLUTION:284x76 DRIVER IC:ST7789", RED, WHITE, 12, 0);
        delay_ms(1000);
        LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
        LCD_Fill(0, 0, 37, LCD_H, RED);
        LCD_Fill(37, 0, 72, LCD_H, GREEN);
        LCD_Fill(72, 0, 107, LCD_H, BLUE);
        LCD_Fill(107, 0, 142, LCD_H, YELLOW);
        LCD_Fill(142, 0, 177, LCD_H, WHITE);
        LCD_Fill(177, 0, 209, LCD_H, GRAY);
        LCD_Fill(209, 0, 244, LCD_H, BLACK);
        LCD_Fill(244, 0, 284, LCD_H, MAGENTA);
        delay_ms(1000);
        LCD_Fill(0, 0, LCD_W / 2, LCD_H / 2, RED);
        LCD_Fill(LCD_W / 2, 0, LCD_W, LCD_H / 2, GREEN);
        LCD_Fill(0, LCD_H / 2, LCD_W / 2, LCD_H, BLUE);
        LCD_Fill(LCD_W / 2, LCD_H / 2, LCD_W, LCD_H, WHITE);
        delay_ms(1000);
		LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
		LCD_ShowPicture(5, 5, 76, 62, tuolaji);
		LCD_ShowChinese(100,35,"欢迎使用东方红农业装备",RED,WHITE,16,0);
		
		delay_ms(5000);
}

/**************** 串口1处理函数 ****************************/

void RX1_Check(void)
{
    u8  i,dat;
//	printf("收到内容如下：   %d",COM1.RX_Cnt);
//	for(i=0; i<COM1.RX_Cnt; i++)    printf("%d,%c\r\n",i, RX1_Buffer[i]);    //把收到的数据原样返回,用于测试
//	printf("\r\n");
	
	//-------------------------------串口RTC对时 -------------------------------   
	for(i=0; i<COM1.RX_Cnt; i++)
	{
	
//		printf("%d,%c\r\n",i, (int)RX1_Buffer[i]); 
//		
		//数据接收
		 if( length>0 )
        {     
            rtctime[length-1]=RX1_Buffer[i];
            length++;
            if( length>=12 )
            {
                length = 0;
                Rec_OK = 1;
                indexrtc=0;
            }
        }      
		
		//头部接收
		 if (RX1_Buffer[i] == STCRTC[ indexrtc])
        {
            indexrtc++;
            if(STCRTC[indexrtc] == '\0')
            {
                length = 1;   //开启接收
                indexrtc=0;
            }
        }
        else
        {
            indexrtc = 0;
            if (RX1_Buffer[i] ==STCRTC[ indexrtc])
                indexrtc++;
        }         
	}
    
}