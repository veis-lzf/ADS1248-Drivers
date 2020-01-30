#include "ads1248.h"

// 初始化ADC芯片
void initADS()
{
    int i;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);            // 开启AFIO时钟
//  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);            // 改变指定管脚的映射 GPIO_Remap_SWJ_Disable SWJ 完全禁用（JTAG+SW-DP）
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);       // 改变指定管脚的映射 GPIO_Remap_SWJ_JTAGDisable ，JTAG-DP 禁用 + SW-DP 使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  START_Pin |  RST2_Pin |  RST1_Pin  | SDO_Pin |  SCLK_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1248_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin =  CS_Pin  | SEL2_Pin | SEL1_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SEL_CS_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  DRDY_Pin |  INT_Pin |  SDI_Pin ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADS1248_GPIO_PORT, &GPIO_InitStructure);

    SPICS_H;
    SPIDO_H;
    SPICK_L;

    START_L;
    SEL1_L;
    SEL2_L;

    GPIO_WriteBit(ADS1248_GPIO_PORT, RST2_Pin, Bit_RESET);
    GPIO_WriteBit(ADS1248_GPIO_PORT, RST1_Pin, Bit_RESET);
    for(i = 0; i < 100000; i++);
    START_H;
    GPIO_WriteBit(ADS1248_GPIO_PORT, RST2_Pin, Bit_SET);
    GPIO_WriteBit(ADS1248_GPIO_PORT, RST1_Pin, Bit_SET);

#ifdef DELAY_FUN_MAP
    Delay_ms(16);
#else
    for(i=0;i<1600000;i++);   // 16MS
#endif
    ADS_REG_WR(ADS_RESET, 0, 0, 0);
#ifdef DELAY_FUN_MAP
    Delay_ms(16);
#else
    for (i = 0; i < 1600000; i++);
#endif
    SEL1_L;
    SEL2_L; // 选择模拟开关通道
    ADS1248_SetInputChannel(P_AIN0, N_AIN1); // 选择输入通道
#ifdef  USE_INTER_VREF
    ADS1248_SetReference(REF_Inter_AlwaysOn, SELT_REF1); // 设置内部基准为参考源
#else  /*   USE REF0  */
    ADS1248_SetReference(REF_Inter_AlwaysOn, SELT_REF1); // 设置外部REF0为参考源
#endif
    ADS1248_SetIDAC(IDAC1_IEXT1, IDAC2_IEXT1, IMAG_1500); // 设置电流源，输出到外部  OUT1  OUT2脚
    ADS1248_SetPGAGainAndDataRate(PGAGain_1, DataRate_10); // 设置PGA倍数和转换速率
}

// 选择ADC采样通道
void ADS_sel_chn(int i)
{
    if(i == 0)
    {
        SEL2_L;
        SEL1_L;   // 选择模拟开关通道
        ADS1248_SetInputChannel(P_AIN0, N_AIN1);    // 选择输入通道
    }
    if(i == 1)
    {
        SEL2_L;
        SEL1_H;   // 选择模拟开关通道
        ADS1248_SetInputChannel(P_AIN4, N_AIN5);    // 选择输入通道
    }
    if(i == 2)
    {
        SEL2_H;
        SEL1_L;   // 选择模拟开关通道
        ADS1248_SetInputChannel(P_AIN6, N_AIN7);    // 选择输入通道
    }
    if(i == 3)
    {
        SEL2_H;
        SEL1_H;   // 选择模拟开关通道
        ADS1248_SetInputChannel(P_AIN2, N_AIN3);    // 选择输入通道
    }

}

// 写ADC芯片寄存器
void ADS_REG_WR(int d1, int d2, int d3, int d4)
{
    int i;
    uint8_t dat;
    SPICS_H;
    SPIDO_H;
    SPICK_L;
    SPICS_L;
    dat = d1;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        if(dat & 0x80)
        {
            SPIDO_H;
        }
        else 
        {
            SPIDO_L;
        }
        SPICK_L;
        dat <<= 1;
    }
    dat = d2;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        if(dat & 0x80)
        {
            SPIDO_H;
        }
        else 
        {
            SPIDO_L;
        }
        SPICK_L;
        dat <<= 1;
    }
    dat = d3;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        if(dat & 0x80)
        {
            SPIDO_H;
        }
        else 
        {
            SPIDO_L;
        }
        SPICK_L;
        dat <<= 1;
    }
    if(d2 == 1)
    {
        dat = d4;
        for(i = 0; i < 8; i++)
        {
            SPICK_H;
            if(dat & 0x80)
            {
                SPIDO_H;
            }
            else
            {
                SPIDO_L;
            }
            SPICK_L;
            dat <<= 1;
        }
    }

    for(i = 0; i < 10; i++);
    SPICS_H;
    SPIDO_H;
    SPICK_H;
    for(i = 0; i < 10; i++)
    {
        SPICK_L;
        SPICK_H;
    }
}

// 读取ADC芯片寄存器的值
void ADS_REG_RD(int STARTaddr, int *reg1, int *reg2)
{
    int i;
    uint8_t  dat;
    SPICS_H;
    SPIDO_H;
    SPICK_L;
    SPICS_L;
    dat = (STARTaddr & 0x0f) | 0x20;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        if(dat & 0x80)
        {
            SPIDO_H;
        }
        else 
        {
            SPIDO_L;
        }
        SPICK_L;
        dat <<= 1;
    }
    dat = 1;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        if(dat & 0x80)
        {
            SPIDO_H;
        }
        else 
        {
            SPIDO_L;
        }
        SPICK_L;
        dat <<= 1;
    }

    dat = 0;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        dat <<= 1;
        if(GPIO_ReadInputDataBit(ADS1248_GPIO_PORT, SDI_Pin))
        {
            dat |= 1;
        }
        SPICK_L;
    }
    *reg1 = dat;
    dat = 0;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        dat <<= 1;
        if(GPIO_ReadInputDataBit(ADS1248_GPIO_PORT, SDI_Pin))
        {
            dat |= 1;
        }
        SPICK_L;
    }
    *reg2 = dat;
    for(i = 0; i < 10; i++);
    SPICS_H;
    SPIDO_H;
    SPICK_H;
    for(i = 0; i < 10; i++)
    {
        SPICK_L;
        SPICK_H;
    }
}

// 读取ADC的值
int ADS_READ_VAL(void)
{
    int i, j, t;
    uint32_t adcval = 0, dat;

    t = 0;
    i = GPIO_ReadInputDataBit(ADS1248_GPIO_PORT, DRDY_Pin);
    while(1)
    {
        j = GPIO_ReadInputDataBit(ADS1248_GPIO_PORT, DRDY_Pin);
        if((i == 1) && (j == 0))
        {
            break;
        }
        i = j;
        t++;
        if(t > 0x4ffffff)
        {
            return 0;
        }
    }
    
    SPICS_H;
    SPIDO_H;
    SPICK_L;
    SPICS_L;
    dat = 0x12;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        if(dat & 0x80)
        {
            SPIDO_H;
        }
        else 
        {
            SPIDO_L;
        }
        SPICK_L;
        dat <<= 1;
    }
    for(i = 0; i < 100000; i++);
    adcval = 0;
    dat = 0;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        dat <<= 1;
        if(GPIO_ReadInputDataBit(ADS1248_GPIO_PORT, SDI_Pin))
        {
            dat |= 1;
        }
        SPICK_L;
    }
    adcval |= dat << 16;
    dat = 0;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        dat <<= 1;
        if(GPIO_ReadInputDataBit(ADS1248_GPIO_PORT, SDI_Pin))
        {
            dat |= 1;
        }
        SPICK_L;
    }
    adcval |= dat << 8;
    dat = 0;
    for(i = 0; i < 8; i++)
    {
        SPICK_H;
        dat <<= 1;
        if(GPIO_ReadInputDataBit(ADS1248_GPIO_PORT, SDI_Pin))
        {
            dat |= 1;
        }
        SPICK_L;
    }
    adcval |= dat ;
    for(i = 0; i < 10; i++);
    SPICS_H;
    SPIDO_H;
    SPICK_H;
    for(i = 0; i < 10; i++)
    {
        SPICK_L;
        SPICK_H;
    }
    
//    return  adcval;
    adcval<<=8;           // 左移8位，放大256倍识别正负号
    return (adcval >> 8); // 右移8位恢复原值
}

// 设置ADC的输出电流
void ADS1248_SetIDAC(uint8_t  idac1, uint8_t  idac2, uint8_t  idacImage)
{
    ADS_REG_WR (ADS_WREG | ADS_IDAC0, 1, idacImage, idac1 | idac2);
}

// 设置PGA倍数和传输速率
void ADS1248_SetPGAGainAndDataRate(uint8_t  pgaGain, uint8_t  dataRate)
{
    ADS_REG_WR (ADS_WREG | ADS_SYS0, 0, pgaGain | dataRate, 0);
}

// 设置ADC输入通道
void ADS1248_SetInputChannel(uint8_t  positiveChannel, uint8_t  negativeChannel)
{
    ADS_REG_WR(ADS_WREG | ADS_MUX0, 0, positiveChannel | negativeChannel, 0);
}

// 设置ADC的基准源
void ADS1248_SetReference(uint8_t  interVrefOnOff, uint8_t  refSelected)  // 设置内部基准为参考源
{
    ADS_REG_WR(ADS_WREG | ADS_MUX1, 0, interVrefOnOff | refSelected, 0);
}
