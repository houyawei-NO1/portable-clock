/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/* --- Web: www.STCMCUDATA.com  ---------------------------------------*/
/* --- BBS: www.STCAIMCU.com  -----------------------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序            */
/*---------------------------------------------------------------------*/

#ifndef		__CONFIG_H
#define		__CONFIG_H

/*********************************************************/

//#define MAIN_Fosc		22118400L	//定义主时钟
#define MAIN_Fosc		12000000L	//定义主时钟
//#define MAIN_Fosc		11059200L	//定义主时钟
//#define MAIN_Fosc		 5529600L	//定义主时钟
//#define MAIN_Fosc		24000000L	//定义主时钟

//==========================================================================

//#define Timer0_Reload   (MAIN_Fosc / 1000)      //Timer 0 中断频率, 1000次/秒
//#define Timer1_Reload   (MAIN_Fosc / 10000)     //Timer 1 中断频率, 10000次/秒

/*********************************************************/

#include "type_def.h"
#include "stc32g.h"
#include <stdlib.h>
#include <stdio.h>

//void Timer0_init(void);
//void Timer1_init(void);
//void Timer3_init(void);

//extern bit one_f,two_f,three_f,four_f,five_f,six_f,low_f,high_f;
extern void	PWM_config(u16 Fre);

/* 定义LCD管脚端口 */
sbit LCD_SCK = P2 ^ 0;  // 时钟管脚
sbit LCD_MOSI = P2 ^ 1; // 数据管脚
sbit LCD_RES = P2 ^ 2;  // 复位管脚
sbit LCD_DC = P2 ^ 3;   // 数据/命令管脚
sbit LCD_CS = P2 ^ 4;   // 片选信号线
sbit LCD_BLK = P2 ^ 5;  // 背光控制管脚

sbit V_JRP = P1^4;

sbit left_key=P7^4;
sbit red_key=P7^5;
sbit add_key=P7^6;
sbit right_key=P7^7;

/********************精确延时函数************/
#if (MAIN_Fosc >= 40000000L)
    #define		usrNOP()    _nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_()
#elif (MAIN_Fosc >= 36000000L)
    #define		usrNOP()    _nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_()
#elif (MAIN_Fosc >= 30000000L)
    #define		usrNOP()    _nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();_nop_()
#elif (MAIN_Fosc >= 24000000L)
    #define		usrNOP()    _nop_();_nop_();_nop_();_nop_();_nop_();_nop_()
#elif (MAIN_Fosc >= 20000000L)
    #define		usrNOP()    _nop_();_nop_();_nop_();_nop_();_nop_()
#elif (MAIN_Fosc >= 18000000L)
    #define		usrNOP()    _nop_();_nop_();_nop_();_nop_()
#elif (MAIN_Fosc >= 12000000L)
    #define		usrNOP()    _nop_();_nop_();_nop_()
#elif (MAIN_Fosc >= 6000000L)
    #define		usrNOP()    _nop_();_nop_()
#else
    #define		usrNOP()    _nop_()
#endif
//***************延时函数***********//
//void delay_us(u8 us);
//void DelayXms(u16 xMs);

//----------------串口打印--------------------------------------------//
//寄存器版串口打印函数
//#define Baudrate1   (65536 - MAIN_Fosc / 115200 / 4)


//#define UART1_BUF_LENGTH    128

//void UartPutc(unsigned char dat);
//char putchar(char c);
//void UART1_config(u8 brt);

#endif
