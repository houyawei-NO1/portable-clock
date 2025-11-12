#include    "ntc_sampling.h"

#ifdef BUBBLE_SORT  //使用冒泡排序，去掉最高值、最低值，求中间平均值
u16 ADC_Buffer[16];
#endif

/******************* AD配置函数 *******************/
void	ADC_config(void)
{
	ADC_InitTypeDef		ADC_InitStructure;		//结构定义
	ADC_InitStructure.ADC_SMPduty   = 31;		//ADC 模拟信号采样时间控制, 0~31（注意： SMPDUTY 一定不能设置小于 10）
	ADC_InitStructure.ADC_CsSetup   = 0;		//ADC 通道选择时间控制 0(默认),1
	ADC_InitStructure.ADC_CsHold    = 1;		//ADC 通道选择保持时间控制 0,1(默认),2,3
	ADC_InitStructure.ADC_Speed     = ADC_SPEED_2X16T;		//设置 ADC 工作时钟频率	ADC_SPEED_2X1T~ADC_SPEED_2X16T
	ADC_InitStructure.ADC_AdjResult = ADC_RIGHT_JUSTIFIED;	//ADC结果调整,	ADC_LEFT_JUSTIFIED,ADC_RIGHT_JUSTIFIED
	ADC_Inilize(&ADC_InitStructure);		//初始化
	ADC_PowerControl(ENABLE);						//ADC电源开关, ENABLE或DISABLE
	NVIC_ADC_Init(DISABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}

/***********************************
查询方式做一次ADC, chn为通道号, chn=0~7对应P1.0~P1.7, chn=8~14对应P0.0~P0.6, chn=15对应BandGap电压.
***********************************/
float NTC_ADC_convert(u8 chn)
{
	u16	j;
	u8	k;		//平均值滤波时使用
    float vADC;

	Get_ADCResult(chn);		//参数i=0~15,查询方式做一次ADC, 切换通道后第一次转换结果丢弃. 避免采样电容的残存电压影响.
	Get_ADCResult(chn);    //参数i=0~15,查询方式做一次ADC, 切换通道后第二次转换结果丢弃. 避免采样电容的残存电压影响.

#ifdef BUBBLE_SORT  //使用冒泡排序，去掉最高值、最低值，求中间平均值

	for(k=0; k<16; k++)	ADC_Buffer[k] = Get_ADCResult(chn);
	BubbleSort(ADC_Buffer,16);  //冒泡排序
	for(k=4, j=0; k<12; k++) j += ADC_Buffer[k];  //取中间8个数据
	j >>= 3;		// 右移3位(除8)，求平均

#else   //采样累加，求平均值（不需要的话可将 SUM_LENGTH 定义值改为 1 ）

	for(k=0, j=0; k<SUM_LENGTH; k++)	j += Get_ADCResult(chn);	// 采样累加和 参数0~15,查询方式做一次ADC, 返回值就是结果
	j = j / SUM_LENGTH;		// 求平均

#endif
    
    printf("12bit: ADC%02d=%04u  ",chn,j);  //输出ADC值
    
//    vADC = ((float)j * 2.8 / 4096);  //计算NTC电压, Vref=5.0V
//    printf("电压:%fV  ",vADC);
    
    vADC = CalculationTemperature(j); //计算温度值
//    printf("T=%f °C\r\n",vADC);
	return vADC;

    //过采样例子
//	for(k=0, j=0; k<16; k++)	j += Get_ADCResult(chn);	// 采样累加和 参数0~15,查询方式做一次ADC, 返回值就是结果
//	vADC = j / 4;		// 12位ADC，采样16次数值累加后除以4，结果便为14位过采样ADC数值
//    printf("14bit: ADC=%f  ",vADC);
//    vADC = (vADC * 5.0 / 16384);  //计算NTC电压, Vref=2.5V
//    printf("P13=%fV  ",vADC);
	
}


/******************** 计算NTC温度 *********************/
#define ADC_VREF 2.89f     // ADC 参考，测得 2.8V
#define DIV_VSUP 3.12f     // 分压上端电压 (R20 接的 VCC)。请确认并替换为实测值
#define R_SERIES 6040.0f  // 固定串联电阻 R1 = 10k
#define NTC_R25 10000.0f   // NTC 在 25°C 名义阻值 R0 = 10k
#define NTC_B   3950.0f    // NTC Beta 值

float CalculationTemperature(u16 adc)
{
    const float T0 = 25.0f + 273.15f; // 25°C in Kelvin
    const float B = NTC_B;
    float v_adc;
    float r_ntc;
    float tempK, tempC;

    if (adc == 0) {
        // ADC 读到 0，表示 Vadc=0，NTC 可能短路或测量无效，返回一个极低温度或特殊值
        return -100.0f;
    }
    if (adc >= 4095) {
        // ADC 接近满量程，分母将趋近于0，认为 NTC 阻值极大（开路/超量程）
        return 150.0f; // 返回一个上限温度（根据传感器范围调整）
    }

    // ADC -> 电压 (使用 4095 作为满量程)
    v_adc = (float)adc * (ADC_VREF / 4095.0f);
	printf("电压:%fV  ",v_adc);

    // 计算 NTC 电阻：假设电路为 Vref -- R_SERIES -- Vadc -- NTC -- GND
    // r_ntc = (v_adc * R_SERIES) / (VREF - v_adc);
	r_ntc = (v_adc * R_SERIES) / (DIV_VSUP - v_adc);
	printf("电阻:%f欧  ",r_ntc);
    // Beta 公式（自然对数）
    tempK = 1.0f / ( (1.0f / T0) + (log(r_ntc / NTC_R25) / B) );
    tempC = tempK - 273.15f;

    return tempC;
}

/******************** 计算PTC100温度 *********************/

float CalculationPTC100Temperature(u16 adc)
{
	float V_TAD,RT, Temperature;
	V_TAD = ((float)adc * 5.0f / 4096.0f);      // ADC采样电压，参考电压5.0V，范围0.14-2.2V
	printf("V_TAD=%f v\r\n",V_TAD);
	// RT  = (2000* V_TAD + 4337) / (57.84f - V_TAD); //RT范围80.31-157.33
	// RT  = (2000* V_TAD + 4320) / (56.97f - V_TAD); //RT范围80.31-157.33
	// RT  = (4.15* V_TAD + 9) / (120.0f - 2.075f*V_TAD)*1000; //RT范围80.31-157.33
	RT  = (4.15* V_TAD + 9) / (120.0f - 2.075f*V_TAD)*1024; //RT范围80.31-157.33
	printf("RT=%f Ω\r\n",RT);
	if (RT < 80.31f) Temperature = -50.0f; //最小值
	if (RT >= 80.31f && RT < 100.0f) Temperature = (RT-100.0f)/0.3938f; 
	if (RT >= 100.0f && RT < 119.4f) Temperature = (RT-100.0f)/0.388f; 
	if (RT >= 119.4f && RT < 138.51f) Temperature = (RT-100.29f)/0.3822f; 
	if (RT >= 138.51f && RT < 157.33f) Temperature = (RT-100.87f)/0.3764f; 
	if (RT > 157.0f) Temperature = 150.0f; //
	return Temperature;
}
#ifdef BUBBLE_SORT  //使用冒泡排序
//========================================================================
// 函数: void DataSwap(u16* data1, u16* data2)
// 描述: 数据交换函数。
// 参数: data1,data2 要交换的数据.
// 返回: none.
// 版本: VER1.0
// 日期: 2021-9-27
// 备注: 
//========================================================================
void DataSwap(u16* data1, u16* data2)
{
	u16 temp;
	temp = *data1;
	*data1 = *data2;
	*data2 = temp;
}

//========================================================================
// 函数: void BubbleSort(u16* pDataArry, u8 DataNum)
// 描述: 冒泡排序函数。
// 参数: pDataArry需要排序的数组，DataNum需要排序的数据个数.
// 返回: none.
// 版本: VER1.0
// 日期: 2021-9-27
// 备注: 
//========================================================================
void BubbleSort(u16* pDataArry, u8 DataNum)
{
	bit flag;
	u8 i,j;
	for(i=0;i<DataNum-1;i++)
	{
		flag = 0;
		for(j=0;j<DataNum-i-1;j++)
		{
			if(pDataArry[j] > pDataArry[j+1])
			{
				flag = 1;
				DataSwap(&pDataArry[j], &pDataArry[j+1]);
			}
		}
		if(!flag)  //上一轮比较中不存在数据交换，则退出排序
		{
			break;
		}
	}
}
#endif