#ifndef	__NTC_SAMPLING_H
#define	__NTC_SAMPLING_H

#include	"\..\library\STC32G_ADC.h"
#include	"\..\library\STC32G_NVIC.h"//中断
#include 	<math.h>

#define SUM_LENGTH	16	/* 平均值采样次数 最大值16（不需要的话可将定义值改为 1 ）*/

#define BUBBLE_SORT     //设置使用冒泡排序，去掉最高值、最低值，求中间平均值（不需要的话屏蔽此行）



void  	ADC_config(void);
extern float    NTC_ADC_convert(u8 chn);	//chn=0~7对应P1.0~P1.7, chn=8~14对应P0.0~P0.6, chn=15对应BandGap电压
void 	BubbleSort(u16* pDataArry, u8 DataNum);
float 	CalculationTemperature(u16 adc);
float   CalculationPTC100Temperature(u16 adc);
void 	DataSwap(u16* data1, u16* data2);
void 	BubbleSort(u16* pDataArry, u8 DataNum);

#endif