// Author: Joshua Southerland (2015)

#define USE_CROSS_STUDIO_DEBUG

// STM32F405RG (see stm32f4xx.h to change target device)
// you have to uncomment one of the options, I chose:
//#define STM32F427_437xx

// assert_param undefined reference errors are solved by defining USE_STDPERIPH_DRIVER
//#define USE_STDPERIPH_DRIVER

// for startup file, make sure  STARTUP_FROM_RESET is defined

// also have to specify the external crystal speed
//#define HSE_VALUE=16000000

#include "stm32f4xx.h"

// use a board definition file
// makes it easier to switch things like processor model, board revision, etc 
#include "wallaby_r1.h"
#include "wallaby_spi_r1.h"

#ifdef USE_CROSS_STUDIO_DEBUG
    #include <__cross_studio_io.h>
#else
    int debug_printf(const char *format, ...){};
    void debug_exit(uint8_t val){};
#endif

#define BUFFERSIZE                       REG_READABLE_COUNT

volatile uint8_t aTxBuffer[REG_ALL_COUNT]; // =  {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};  //"SPI Master/Slave : Communication between two SPI using DMA";
__IO uint8_t aRxBuffer [REG_ALL_COUNT];
__IO uint8_t ubRxIndex = 0;
__IO uint8_t ubTxIndex = 0;
__IO uint32_t TimeOut = 0;
 
SPI_InitTypeDef  SPI_InitStructure;


// TODO: timings based on a 32 bit usec clock would overflow and maybe glitch every 1.19 hours
// Maybe have a second clock too?  If so usCount would just be  time % 1_second 
// and we would want helper functions that accept  sec/usec timing pairs
static volatile uint32_t usCount;


typedef struct pid_struct_
{
    float kP;
    float kI;
    float kD;
    float prevErr;
    float iErr;

} pid_struct;

void initSystick()
{
    usCount=0;
    SysTick_Config(SystemCoreClock/1000000);
}
 
 
void delay_us(uint32_t delay)
{
    uint32_t target;
    target=usCount+delay;
    while(usCount<target);
}
 
void SysTick_Handler(void) 
{
    usCount++;
}

void configMotorPin(uint32_t pin, GPIO_TypeDef* port)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(port, &GPIO_InitStructure);
}

void configServoPin(uint32_t pin, GPIO_TypeDef* port)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(port, &GPIO_InitStructure);
}


void configBEMFPin(uint32_t pin, GPIO_TypeDef* port) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(port, &GPIO_InitStructure);
}


void configAnalogInPin(uint32_t pin, GPIO_TypeDef* port, uint8_t pullup_enable) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    
    if (pullup_enable)
    {
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    }
    else
    {
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    }

    GPIO_Init(port, &GPIO_InitStructure);
}

void configDigitalInPin(uint32_t pin, GPIO_TypeDef* port)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;		
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(port, &GPIO_InitStructure);
}

void configDigitalOutPin(uint32_t pin, GPIO_TypeDef* port)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(port, &GPIO_InitStructure);
}



uint16_t slow_adc(ADC_TypeDef * bus, uint8_t channel)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // ADC Common Init
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1 Channel
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // Manual
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(bus, &ADC_InitStructure);

    // ADC1 regular channel 11 configuration
    ADC_RegularChannelConfig(bus, channel, 1, ADC_SampleTime_28Cycles); // PC1

    // Enable ADC1
    ADC_Cmd(bus, ENABLE);
    delay_us(10);

    ADC_SoftwareStartConv(bus);
    while(ADC_GetFlagStatus(bus, ADC_FLAG_EOC) == RESET);
    uint16_t val = ADC_GetConversionValue(bus);

    // Disable ADC1
    ADC_Cmd(bus, DISABLE);
    delay_us(10);
    return val;
}



void initSPI3()
{
    // SPI clock should be enabled below with RCC_AHB1PeriphClockCmd


    // set up pins for  clock, miso, mosi
    {
        // Note: this assumes CLK, MISO, MOSI are on the same port... but they always should be
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI3_CLK | SPI3_MISO | SPI3_MOSI;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(SPI3_CLK_PORT, &GPIO_InitStruct);
    }
    
    // use the alternate function (SPI3)
    GPIO_PinAFConfig(SPI3_CLK_PORT,  SPI3_CLK_SOURCE,  GPIO_AF_SPI3);
    GPIO_PinAFConfig(SPI3_MISO_PORT, SPI3_MISO_SOURCE, GPIO_AF_SPI3);
    GPIO_PinAFConfig(SPI3_MOSI_PORT, SPI3_MOSI_SOURCE, GPIO_AF_SPI3);


    // set up chip select pins, not expected to be on the same port
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI3_CS0;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(SPI3_CS0_PORT, &GPIO_InitStruct);     
    }
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI3_CS1;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(SPI3_CS1_PORT, &GPIO_InitStruct);   
    }


    // set chip selects high  (they are active low)
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0;
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

    // configure SPI3 in Mode 0
    // CPOL 0  (clock low when idle)
    // CPHA 0  (data sampled at first edge)
    {
        SPI_InitTypeDef SPI_InitStruct;
        SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
        SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_Init(SPI3, &SPI_InitStruct);
    }

    SPI_Cmd(SPI3, ENABLE);
}



void initSPI4()
{
    // SPI clock should be enabled below with RCC_AHB1PeriphClockCmd


    // set up pins for  clock, miso, mosi
    {
        // Note: this assumes CLK, MISO, MOSI are on the same port... but they always should be
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI4_CLK | SPI4_MISO | SPI4_MOSI;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(SPI4_CLK_PORT, &GPIO_InitStruct);
    }
    
    // use the alternate function (SPI3)
    GPIO_PinAFConfig(SPI4_CLK_PORT,  SPI4_CLK_SOURCE,  GPIO_AF_SPI4);
    GPIO_PinAFConfig(SPI4_MISO_PORT, SPI4_MISO_SOURCE, GPIO_AF_SPI4);
    GPIO_PinAFConfig(SPI4_MOSI_PORT, SPI4_MOSI_SOURCE, GPIO_AF_SPI4);


    // set up chip select and nirq pins, not expected to be on the same port
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI4_CS0;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(SPI4_CS0_PORT, &GPIO_InitStruct);     
    }
    {
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.GPIO_Pin = SPI4_NIRQ;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(SPI4_NIRQ_PORT, &GPIO_InitStruct);   
    }


    // set chip select and nirq high  (they are active low)
    SPI4_CS0_PORT->BSRRL |= SPI4_CS0;
    SPI4_NIRQ_PORT->BSRRL |= SPI4_NIRQ;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI4, ENABLE);

    // configure SPI3 in Mode 0
    // CPOL 0  (clock low when idle)
    // CPHA 0  (data sampled at first edge)
    {
        SPI_InitTypeDef SPI_InitStruct;
        SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
        SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
        SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
        SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
        SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_Init(SPI4, &SPI_InitStruct);
    }

    SPI_Cmd(SPI4, ENABLE);
}




uint8_t SPI3_write(uint8_t data)
{
	SPI3->DR = data; // write data
	while( !(SPI3->SR & SPI_I2S_FLAG_TXE) ); // wait for transmit
	while( !(SPI3->SR & SPI_I2S_FLAG_RXNE) ); // wait for receive
	while( SPI3->SR & SPI_I2S_FLAG_BSY ); // wait unit SPI is done
	return SPI3->DR; // return data
}


uint8_t SPI4_write(uint8_t data)
{
	SPI4->DR = data; // write data
	while( !(SPI4->SR & SPI_I2S_FLAG_TXE) ); // wait for transmit
	while( !(SPI4->SR & SPI_I2S_FLAG_RXNE) ); // wait for receive
	while( SPI4->SR & SPI_I2S_FLAG_BSY ); // wait unit SPI is done
	return SPI4->DR; // return data
}


void setupAccelMag()
{
    configDigitalInPin(ACCEL_INT1, ACCEL_INT1_PORT);  
    configDigitalInPin(ACCEL_INT2, ACCEL_INT2_PORT);  

    // select Accel/Mag
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    delay_us(100); 

    uint8_t regval;
    // request "Who am I)
    SPI3_write(0x8F); // write register
    regval = SPI3_write(0x00); // write dummy val to get contents
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // chip select low

    if (regval == 73){
        debug_printf("Accel/Magn identified itself\n");
    }else{
        debug_printf("Accel/Magn did not respond/identify itself (regval=%d)\n",regval);
        return;
    }

    // accel
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x20);
    SPI3_write(0x57); // 50hz continuous, all axes
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x21);
    SPI3_write(0x00);  // 773Hz filter, +/- 2g scale, no self test, full duplex SPI
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    // accel data ready on int1
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0;
    SPI3_write(0x22);
    SPI3_write(0b00000100);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    // mag data ready on int1
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0;
    SPI3_write(0x23);
    SPI3_write(0b00000100);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    // magnetometer
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x24);
    SPI3_write(0x70); // temp disabled, high res, 50Hz, no interrupts
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x25);
    SPI3_write(0x20);  // +/- 4g
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x26);
    SPI3_write(0x00);  // normal
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
}

void readAccel()
{
    uint16_t accel_x, accel_y, accel_z;

    //x  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xA8);
    accel_x = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xA9);
    accel_x |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip


    //Y  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xAA);
    accel_y = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xAB);
    accel_y |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip


    //Z  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xAC);
    accel_z = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0xAD);
    accel_z |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip

    aTxBuffer[REG_RW_ACCEL_X_H]  = (accel_x & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ACCEL_X_L] = (accel_x & 0x00FF);
    aTxBuffer[REG_RW_ACCEL_Y_H] = (accel_y & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ACCEL_Y_L] = (accel_y & 0x00FF);
    aTxBuffer[REG_RW_ACCEL_Z_H] = (accel_z & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ACCEL_Z_L] = (accel_z & 0x00FF);

    //float ax = (float)((int16_t)accel_x) / 16000.0;
    //float ay = (float)((int16_t)accel_y) / 16000.0;
    //float az = (float)((int16_t)accel_z) / 16000.0;
    //debug_printf("accel %d %d %d \n", (int16_t)accel_x, (int16_t)accel_y, (int16_t)accel_z);
    //debug_printf("accel %f %f %f\n", ax, ay, az);
}



void readMag()
{
    uint16_t mag_x, mag_y, mag_z;

    uint32_t int1 = ACCEL_INT1_PORT->IDR & ACCEL_INT1;
    uint32_t int2 = ACCEL_INT2_PORT->IDR & ACCEL_INT2;

    //x  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x88);
    mag_x = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x89);
    mag_x |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip


    //Y  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x8A);
    mag_y = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x8B);
    mag_y |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip


    //Z  TODO high or low first
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x8C);
    mag_z = SPI3_write(0x00);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip
  
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    SPI3_write(0x8D);
    mag_z |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // done with chip


    aTxBuffer[REG_RW_MAG_X_H] = (mag_x & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MAG_X_L] = (mag_x & 0x00FF);
    aTxBuffer[REG_RW_MAG_Y_H] = (mag_y & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MAG_Y_L] = (mag_y & 0x00FF);
    aTxBuffer[REG_RW_MAG_Z_H] = (mag_z & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MAG_Z_L] = (mag_z & 0x00FF);

    //float mx = (float)((int16_t)mag_x) / 16000.0;
    //float my = (float)((int16_t)mag_y) / 16000.0;
    //float mz = (float)((int16_t)mag_z) / 16000.0;
    //debug_printf("accel %d %d %d \n", (int16_t)accel_x, (int16_t)accel_y, (int16_t)accel_z);
    //debug_printf("mag %f %f %f  (int1 %d   int2 %d)\n", mx, my, mz, int1, int2);
}



void setupUART2()
{
    // UART2 to external header
    
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    // set up serial on UART2 to main processor
    // PD5 TX   PD6 RX
    USART_InitTypeDef usartConfig;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // RCC_APB1Periph_AFIO
    usartConfig.USART_BaudRate = 115200;
    usartConfig.USART_WordLength = USART_WordLength_8b;
    usartConfig.USART_StopBits = USART_StopBits_1;
    usartConfig.USART_Parity = GPIO_PuPd_NOPULL;
    usartConfig.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usartConfig.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &usartConfig);

    USART_Cmd(USART2, ENABLE);
    //USART_ITConfig(USART2, USART_IT_RXNE |USART_IT_TXE, ENABLE);
}

void setupUART3()
{
    // UART3 to main proc
    
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect USART pins to AF */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    // set up serial on UART3 to main processor
    // PB10 TX   PB11 RX
    USART_InitTypeDef usartConfig;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // RCC_APB1Periph_AFIO
    usartConfig.USART_BaudRate = 115200;
    usartConfig.USART_WordLength = USART_WordLength_8b;
    usartConfig.USART_StopBits = USART_StopBits_1;
    usartConfig.USART_Parity = USART_Parity_No;
    usartConfig.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usartConfig.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &usartConfig);

    USART_Cmd(USART3, ENABLE);
    //USART_ITConfig(USART3, USART_IT_RXNE |USART_IT_TXE, ENABLE);
}


void setupGyro()
{
    configDigitalInPin(GYRO_DATA_READY, GYRO_DATA_READY_PORT);    

    // select Accel/Mag
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    delay_us(100); 

    uint8_t regval;
    // request "Who am I)
    SPI3_write(0x80 | 0x0F); // write register
    regval = SPI3_write(0x00); // write dummy val to get contents
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // chip select low

    if (regval == 211){
      debug_printf("Gyro identified itself\n");
    }else{
      debug_printf("Gyro did not respond/identify itself (regval=%d)\n",regval);
      return;
    }

    // ctrl1
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1;
    SPI3_write(0x20);
    SPI3_write(0b00001111);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;

    // ctrl2
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1;
    SPI3_write(0x21);
    SPI3_write(0b00000000);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;

    // ctrl3
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1;
    SPI3_write(0x22);
    SPI3_write(0b00001000); // use data ready pin
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;

    // ctrl4
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1;
    SPI3_write(0x23);
    SPI3_write(0b00000000);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;

    // ctrl5
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1;
    SPI3_write(0x24);
    SPI3_write(0b00000000);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1;
}


void readGyro()
{
    uint16_t gyro_x, gyro_y, gyro_z;

    uint32_t gyro_data_ready = GYRO_DATA_READY_PORT->IDR & GYRO_DATA_READY;

    //x  TODO high or low first
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x28);
    gyro_x = SPI3_write(0x00);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip
  
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x29);
    gyro_x |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip


    //y  TODO high or low first
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x2A);
    gyro_y = SPI3_write(0x00);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip
  
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x2B);
    gyro_y |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip



    //z  TODO high or low first
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x2C);
    gyro_z = SPI3_write(0x00);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip
  
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    SPI3_write(0x80 | 0x2D);
    gyro_z |= ((uint16_t)SPI3_write(0x00) << 8);
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // done with chip


    aTxBuffer[REG_RW_GYRO_X_H] = (gyro_x & 0xFF00) >> 8;
    aTxBuffer[REG_RW_GYRO_X_L] = (gyro_x & 0x00FF);
    aTxBuffer[REG_RW_GYRO_Y_H] = (gyro_y & 0xFF00) >> 8;
    aTxBuffer[REG_RW_GYRO_Y_L] = (gyro_y & 0x00FF);
    aTxBuffer[REG_RW_GYRO_Z_H] = (gyro_z & 0xFF00) >> 8;
    aTxBuffer[REG_RW_GYRO_Z_L] = (gyro_z & 0x00FF);
}



void spi4_demo()
{

    SPI4_CS0_PORT->BSRRH |= SPI4_CS0; // chip select low
    delay_us(100); 


    uint8_t regval;
    // request "Who am I)
    SPI4_write(0x1F); // write register
    regval = SPI4_write(0x00); // write dummy val to get contents
    debug_printf("Read a %x\n", regval);

    SPI4_CS0_PORT->BSRRL |= SPI4_CS0; // chip select high
}


// yellow led for use by cortex m4
void yellow_led_demo(unsigned int seconds)
{
      debug_printf("yellow led demo\n");
      for (unsigned int i = 0; i < seconds; ++i)
      {
              // LED off (set pin high)
              LED1_PORT->BSRRL |= LED1_PIN;
              delay_us(100000);

              // LED on (set pin low)
              LED1_PORT->BSRRH |= LED1_PIN;
              delay_us(100000);
      }
}



// serial communication over external uart header
void uart2_demo()
{
    debug_printf("uart2 demo\n");
    // Wait until a byte is received 
    //while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET)
    //{
    //}

    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
    {
        unsigned char rd = USART_ReceiveData(USART2);
        debug_printf("got %c\n", rd);
    }

    // read byte and print via usart2
    //unsigned char rd = USART_ReceiveData(USART2);
    //debug_printf("got %c\n", rd);
    debug_printf("sending\n");
    USART_SendData(USART2, 'D');        
}


// serial communication with main processor
void uart3_demo()
{
    debug_printf("uart3 demo\n");
    // Wait until a byte is received 
    while(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
    {
    }

    // read byte and print via usart2
    unsigned char rd = USART_ReceiveData(USART3);
    debug_printf("got %c\n", rd);
    USART_SendData(USART3, 'D');        
}

void adc_update()
{
    //debug_printf("adc_demo\n");

    int32_t adc_in[6];
    int32_t adc_batt = 0;

    adc_in[0] = slow_adc(AIN0_ADX, AIN0_CHAN);
    adc_in[1] = slow_adc(AIN1_ADX, AIN1_CHAN);
    adc_in[2] = slow_adc(AIN2_ADX, AIN2_CHAN);
    adc_in[3] = slow_adc(AIN3_ADX, AIN3_CHAN);
    adc_in[4] = slow_adc(AIN4_ADX, AIN4_CHAN);
    adc_in[5] = slow_adc(AIN5_ADX, AIN5_CHAN);

    adc_batt = slow_adc(ADC_BATT_ADX, ADC_BATT_CHAN);

    aTxBuffer[REG_RW_ADC_0_H] = (adc_in[0] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_0_L] = (adc_in[0] & 0x00FF);
    aTxBuffer[REG_RW_ADC_1_H] = (adc_in[1] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_1_L] = (adc_in[1] & 0x00FF);
    aTxBuffer[REG_RW_ADC_2_H] = (adc_in[2] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_2_L] = (adc_in[2] & 0x00FF);
    aTxBuffer[REG_RW_ADC_3_H] = (adc_in[3] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_3_L] = (adc_in[3] & 0x00FF);
    aTxBuffer[REG_RW_ADC_4_H] = (adc_in[4] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_4_L] = (adc_in[4] & 0x00FF);
    aTxBuffer[REG_RW_ADC_5_H] = (adc_in[5] & 0xFF00) >> 8;
    aTxBuffer[REG_RW_ADC_5_L] = (adc_in[5] & 0x00FF);

    aTxBuffer[REG_RW_BATT_H] = (adc_batt & 0xFF00) >> 8;
    aTxBuffer[REG_RW_BATT_L] = (adc_batt & 0x00FF);
}




void setup_I2C1(void)
{
    debug_printf("Setup I2C1\n");
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // TODO: already did this
    
    GPIO_InitStruct.GPIO_Pin = I2C1_SDA | I2C1_SCL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;// GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_PinAFConfig(I2C1_SDA_PORT, I2C1_SDA_SOURCE, GPIO_AF_I2C1);	
    GPIO_PinAFConfig(I2C1_SCL_PORT, I2C1_SCL_SOURCE, GPIO_AF_I2C1);
    GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct); // TODO: assumes both pins on same port

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    I2C_DeInit(I2C1);
    I2C_Cmd(I2C1, DISABLE);


    I2C_InitStruct.I2C_ClockSpeed = 100000;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0xA1;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &I2C_InitStruct);
}



void update_motor_mode(uint32_t dir1_pin, uint32_t dir2_pin, GPIO_TypeDef* dir1_port, GPIO_TypeDef* dir2_port, uint8_t drive_code)
{

    switch(drive_code)
    {
        case 0:
            dir1_port->BSRRH |= dir1_pin; 
            dir2_port->BSRRH |= dir2_pin;
            break;
        case 1:
            dir1_port->BSRRL |= dir1_pin; 
            dir2_port->BSRRH |= dir2_pin;
            break;
        case 2:
            dir1_port->BSRRH |= dir1_pin; 
            dir2_port->BSRRL |= dir2_pin;
            break;
        case 3:
            dir1_port->BSRRL |= dir1_pin; 
            dir2_port->BSRRL |= dir2_pin;
            break;
        default:
            break;
    }
}

void update_motor_modes()
{
    uint8_t motor_0_mode = (aTxBuffer[REG_RW_MOT_DIRS] & 0b00000011);
    uint8_t motor_1_mode = (aTxBuffer[REG_RW_MOT_DIRS] & 0b00001100) >> 2;
    uint8_t motor_2_mode = (aTxBuffer[REG_RW_MOT_DIRS] & 0b00110000) >> 4;
    uint8_t motor_3_mode = (aTxBuffer[REG_RW_MOT_DIRS] & 0b11000000) >> 6;

    update_motor_mode(MOT0_DIR1, MOT0_DIR2, MOT0_DIR1_PORT, MOT0_DIR2_PORT, motor_0_mode);
    update_motor_mode(MOT1_DIR1, MOT1_DIR2, MOT1_DIR1_PORT, MOT1_DIR2_PORT, motor_1_mode);
    update_motor_mode(MOT2_DIR1, MOT2_DIR2, MOT2_DIR1_PORT, MOT2_DIR2_PORT, motor_2_mode);
    update_motor_mode(MOT3_DIR1, MOT3_DIR2, MOT3_DIR1_PORT, MOT3_DIR2_PORT, motor_3_mode);
}

void idle_motor_dirs()
{
    aTxBuffer[REG_RW_MOT_MODES] = 0;
}


//https://github.com/devthrash/STM32F4-examples/blob/master/I2C%20Master/main.c#L40
/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy anymore
        debug_printf("a\n");
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
          debug_printf("b\n");

	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	          debug_printf("c\n");

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
		        debug_printf("d\n");

	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	  
	/* wait for I2C1 EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
        debug_printf("e\n");

	if(direction == I2C_Direction_Transmitter){
                debug_printf("transmitter\n");
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
                debug_printf("receiver\n");
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}

        I2C_Cmd(I2Cx, ENABLE);


        debug_printf("f\n");

}


void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

void I2C_stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition 
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void spi4_cs0_nirq_test(void)
{
    debug_printf("setting cs0 and nirq low for spi4\n");
    SPI4_CS0_PORT->BSRRH |= SPI4_CS0;
    SPI4_NIRQ_PORT->BSRRL |= SPI4_NIRQ;
}

void i2c1_test(void)
{
    debug_printf("i2c1_test\n");
    const uint8_t SLAVE_ADDRESS = 0xAA; // FIXME:0xA0;
    uint8_t received_data[2];

   while(1)
   {
		
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		I2C_write(I2C1, 0x20); // write one byte to the slave
		//I2C_write(I2C1, 0x03); // write another byte to the slave
		I2C_stop(I2C1); // stop the transmission
		
		I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
		//received_data[0] = I2C_read_ack(I2C1); // read one byte and request another byte
		received_data[1] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
                debug_printf("Read %x, %x\n", received_data[0], received_data[1]);
  }
}



static void SPI_DMA_Config(unsigned int buffer_len)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    // Peripheral Clock Enable -------------------------------------------------
    // Enable the SPI clock 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    // Enable GPIO clocks 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Enable DMA clock 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);


    // SPI GPIO Configuration --------------------------------------------------
    // GPIO Deinitialisation
    GPIO_DeInit(SPI2_CLK_PORT);
    GPIO_DeInit(SPI2_MISO_PORT);
    GPIO_DeInit(SPI2_MOSI_PORT);
    GPIO_DeInit(SPI2_CS0_PORT);
  
    // Connect SPI pins to AF
    GPIO_PinAFConfig(SPI2_CLK_PORT, SPI2_CLK_SOURCE, GPIO_AF_SPI2);
    GPIO_PinAFConfig(SPI2_MISO_PORT, SPI2_MISO_SOURCE, GPIO_AF_SPI2);   
    GPIO_PinAFConfig(SPI2_MOSI_PORT, SPI2_MOSI_SOURCE, GPIO_AF_SPI2);
    GPIO_PinAFConfig(SPI2_CS0_PORT, SPI2_CS0_SOURCE, GPIO_AF_SPI2);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

    // SPI SCK pin configuration
    GPIO_InitStructure.GPIO_Pin = SPI2_CLK;
    GPIO_Init(SPI2_CLK_PORT, &GPIO_InitStructure);

    // SPI  MISO pin configuration
    GPIO_InitStructure.GPIO_Pin =  SPI2_MISO;
    GPIO_Init(SPI2_MISO_PORT, &GPIO_InitStructure); 

    // SPI  MOSI pin configuration
    GPIO_InitStructure.GPIO_Pin =  SPI2_MOSI;
    GPIO_Init(SPI2_MOSI_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  SPI2_CS0;
    GPIO_Init(SPI2_CS0_PORT, &GPIO_InitStructure);

    // SPI configuration -------------------------------------------------------
    SPI_I2S_DeInit(SPI2);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; // | SPI_NSSInternalSoft_Set;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;

    // DMA configuration -------------------------------------------------------
    // Deinitialize DMA Streams
    DMA_DeInit(DMA1_Stream4); // TX stream
    DMA_DeInit(DMA1_Stream3); // RX stream



    // Configure DMA Initialization Structure 
    DMA_InitStructure.DMA_BufferSize = buffer_len;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI2->DR));
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // Configure TX DMA
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aTxBuffer;
    DMA_Init(DMA1_Stream4, &DMA_InitStructure);
    // Configure RX DMA
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aRxBuffer;
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);
   
}
 

void init180MHz()
{
    RCC_DeInit();

    // Enable HSE
    RCC_HSEConfig(RCC_HSE_ON);
    //RCC_HSEConfig(RCC_HSE_Bypass);
    while(RCC_WaitForHSEStartUp() != SUCCESS);

    FLASH_PrefetchBufferCmd(ENABLE);
    FLASH_SetLatency(FLASH_Latency_5);

    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
    RCC_PCLK1Config(RCC_HCLK_Div4);
    RCC_PCLK2Config(RCC_HCLK_Div2); 

    uint32_t PLL_M = 24;  // ( (24 MHz / 24) * 360 ) / 2 = 180 MHz
    uint32_t PLL_N = 360;
    uint32_t PLL_P = 2;
    uint32_t PLL_Q = 7;

    RCC_PLLConfig(RCC_PLLSource_HSE,PLL_M,PLL_N,PLL_P,PLL_Q);
    RCC_PLLCmd(ENABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    // Wait till PLL is used as system clock source
    while(RCC_GetSYSCLKSource() != 0x08);
}



void TIM1_Configuration(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 
  memset(&TIM_OCInitStructure, 0, sizeof TIM_OCInitStructure);
  memset(&TIM_TimeBaseStructure, 0, sizeof TIM_TimeBaseStructure);

  /* Time base configuration - SystemCoreClock = 168000000 for 168 MHz board */
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (((SystemCoreClock / 10000000)) - 1); // Shooting for 1 MHz, (1us)
  TIM_TimeBaseStructure.TIM_Period = 400 - 1; // 1 MHz / 500 = 2 kHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0xFF;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
 
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM1, ENABLE);
 
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 200; // Servo Top-Center
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
 
  /* Output Compare PWM1 Mode configuration: Channel2 PD.13 */
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
 
  /* Output Compare PWM1 Mode configuration: Channel3 PD.14 */
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
 
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
 
  /* TIM4 enable counter */
  TIM_Cmd(TIM1, ENABLE);

  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


void TIM3_Configuration(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 
  /* Time base configuration - SystemCoreClock = 168000000 for 168 MHz board */
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (((SystemCoreClock / 1000000) / 2) - 1); // Shooting for 1 MHz, (1us)
  TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 1 MHz / 20000 = 50 Hz (20ms)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM3, ENABLE);
 
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1500; // Servo Top-Center
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  /* Output Compare PWM1 Mode configuration: Channel2 PD.13 */
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
 
  /* Output Compare PWM1 Mode configuration: Channel3 PD.14 */
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
 
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
 
  /* TIM4 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}


void TIM8_Configuration(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  memset(&TIM_OCInitStructure, 0, sizeof TIM_OCInitStructure);
  memset(&TIM_TimeBaseStructure, 0, sizeof TIM_TimeBaseStructure);

  /* Time base configuration - SystemCoreClock = 168000000 for 168 MHz board */
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (((SystemCoreClock / 10000000)) - 1); // Shooting for 1 MHz, (1us)
  TIM_TimeBaseStructure.TIM_Period = 400 - 1; // 1 MHz / 20000 = 50 Hz (20ms)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0xFF;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
 
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM8, ENABLE);
 
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 200; // Servo Top-Center
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
 
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);
 
  /* TIM4 enable counter */
  TIM_Cmd(TIM8, ENABLE);

  TIM_CtrlPWMOutputs(TIM8, ENABLE);
}



void TIM9_Configuration(void)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 
  /* Time base configuration - SystemCoreClock = 168000000 for 168 MHz board */
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (((SystemCoreClock / 1000000)) - 1); // Shooting for 1 MHz, (1us)
  TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 1 MHz / 20000 = 50 Hz (20ms)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
 
  /* Enable TIM4 Preload register on ARR */
  TIM_ARRPreloadConfig(TIM9, ENABLE);
 
  /* TIM PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1500; // Servo Top-Center
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 
  /* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
  TIM_OC1Init(TIM9, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
 
  /* Output Compare PWM1 Mode configuration: Channel2 PD.13 */
  TIM_OC2Init(TIM9, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

  /* TIM Interrupts enable */
  TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);
 
  /* TIM4 enable counter */
  TIM_Cmd(TIM9, ENABLE);
}

void init()
{
    init180MHz(); // switch over to 180 MHz via the external 24 MHz crystal and a PLL

    initSystick(); // get a wallclock started, ticking at 1ms

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_DMA1 , ENABLE);

    // TIM1,8 for Motors
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    // TIM3,9 for Servos
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

    SPI_DMA_Config(BUFFERSIZE);

    // Slave board configuration
    // Initializes the SPI communication
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    SPI_Init(SPI2, &SPI_InitStructure);


    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

    // Enable DMA SPI TX Stream 
    DMA_Cmd(DMA1_Stream4,ENABLE);

    // Enable DMA SPI RX Stream 
    DMA_Cmd(DMA1_Stream3,ENABLE); 

    {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM1 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
        {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM1 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    // Enable SPI DMA TX Requsts 
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

    // Enable SPI DMA RX Requsts 
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);

    debug_printf("enable spi\n");
    // Enable the SPI peripheral
    SPI_Cmd(SPI2, ENABLE);

    // motor 0 
    configMotorPin(MOT0_DIR1, MOT0_DIR1_PORT);
    configMotorPin(MOT0_DIR2, MOT0_DIR2_PORT);
    //configMotorPin(MOT0_PWM, MOT0_PWM_PORT);
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = MOT0_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(MOT0_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(MOT0_PWM_PORT, MOT0_PWM_PinSource, GPIO_AF_TIM1);
    }

    // motor 1
    configMotorPin(MOT1_DIR1, MOT1_DIR1_PORT);
    configMotorPin(MOT1_DIR2, MOT1_DIR2_PORT);
    //configMotorPin(MOT1_PWM, MOT1_PWM_PORT);
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = MOT1_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(MOT1_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(MOT1_PWM_PORT, MOT1_PWM_PinSource, GPIO_AF_TIM1);
    }

    // motor 2
    configMotorPin(MOT2_DIR1, MOT2_DIR1_PORT);
    configMotorPin(MOT2_DIR2, MOT2_DIR2_PORT);
    //configMotorPin(MOT2_PWM, MOT2_PWM_PORT);
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = MOT2_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(MOT2_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(MOT2_PWM_PORT, MOT2_PWM_PinSource, GPIO_AF_TIM1);
    }

    // motor 3
    configMotorPin(MOT3_DIR1, MOT3_DIR1_PORT);
    configMotorPin(MOT3_DIR2, MOT3_DIR2_PORT);
    //configMotorPin(MOT3_PWM, MOT3_PWM_PORT);
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = MOT3_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(MOT3_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(MOT3_PWM_PORT, MOT3_PWM_PinSource, GPIO_AF_TIM8);
    }

    // motor bemf adc sensing
    configBEMFPin(MOT0_BEMF_H, MOT0_BEMF_H_PORT);
    configBEMFPin(MOT0_BEMF_L, MOT0_BEMF_L_PORT);
    configBEMFPin(MOT1_BEMF_H, MOT1_BEMF_H_PORT);
    configBEMFPin(MOT1_BEMF_L, MOT1_BEMF_L_PORT);

    configBEMFPin(MOT2_BEMF_H, MOT2_BEMF_H_PORT);
    configBEMFPin(MOT2_BEMF_L, MOT2_BEMF_L_PORT);
    configBEMFPin(MOT3_BEMF_H, MOT3_BEMF_H_PORT);
    configBEMFPin(MOT3_BEMF_L, MOT3_BEMF_L_PORT);

    // TODO: cleaner servo init and all 4
    // TODO for now just 0,1 on TIM3
    // TODO: later, 2,3 on TIM9
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = SRV0_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(SRV0_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(SRV0_PWM_PORT, SRV0_PWM_PinSource, GPIO_AF_TIM3);
    }

    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = SRV1_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(SRV1_PWM_PORT, &GPIO_InitStructure);
        
        GPIO_PinAFConfig(SRV1_PWM_PORT, SRV1_PWM_PinSource, GPIO_AF_TIM3);
    }

    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = SRV2_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(SRV2_PWM_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(SRV2_PWM_PORT, SRV2_PWM_PinSource, GPIO_AF_TIM9);
    }

    {
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = SRV3_PWM;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(SRV3_PWM_PORT, &GPIO_InitStructure);
        
        GPIO_PinAFConfig(SRV3_PWM_PORT, SRV3_PWM_PinSource, GPIO_AF_TIM9);
    }
    
    TIM1_Configuration(); // motors 0,1,2

    // TODO move tim1 interrupt setup for motor control
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM1 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    TIM8_Configuration(); // motor 3
    
    // TODO move tim8 interrupt setup for motor control
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM8 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }    

    TIM3_Configuration(); // servos 0,1
        // TODO move tim3 interrupt setup for servo control
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM3 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    TIM9_Configuration(); // servos 2,3

        // TODO move tim9 interrupt setup for servo control
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        // Enable the TIM9 gloabal Interrupt
        NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }



    // Button S1 is also a digital in
    // TODO: make this another digital pin
    configDigitalInPin(BUTTON_S1_PIN, BUTTON_S1_PORT);


    // Init SPI3 (plus chip selects) for the IMU sensors
    initSPI3(); 

    // Init SPI4 (SPI header)
    initSPI4(); 
  
    setup_I2C1();

    // wait a bit
    delay_us(5000);

    setupAccelMag();

    setupGyro();

    setupUART2();
    setupUART3();
    

    // wait a bit
    delay_us(5);
}

void spi2_dma_cleanup()
{
    //dma cleanup
    // Clear DMA Transfer Complete Flags66
    DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4); // tx
    DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3); // rx

    // Disable DMA SPI TX Stream
    DMA_Cmd(DMA1_Stream4,DISABLE);

    // Disable DMA SPI RX Stream
    DMA_Cmd(DMA1_Stream3,DISABLE);

    // Disable SPI DMA TX Requsts
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);

    // Disable SPI DMA RX Requsts
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, DISABLE);

    // Disable the SPI peripheral
    SPI_Cmd(SPI2, DISABLE);
}



void updateDigIn(void)
{
        uint16_t dig_in = 0;
        // IDR is high when pin is low
        if ((BUTTON_S1_PORT->IDR & BUTTON_S1_PIN) == 0) dig_in |= 0b0000010000000000;

        if ((DIG0_PORT->IDR & DIG0_PIN) == 0)           dig_in |= 0b0000001000000000;
        if ((DIG1_PORT->IDR & DIG1_PIN) == 0)           dig_in |= 0b0000000100000000;
        if ((DIG2_PORT->IDR & DIG2_PIN) == 0)           dig_in |= 0b0000000010000000;
        if ((DIG3_PORT->IDR & DIG3_PIN) == 0)           dig_in |= 0b0000000001000000;
        if ((DIG4_PORT->IDR & DIG4_PIN) == 0)           dig_in |= 0b0000000000100000;
        if ((DIG5_PORT->IDR & DIG5_PIN) == 0)           dig_in |= 0b0000000000010000;
        if ((DIG6_PORT->IDR & DIG6_PIN) == 0)           dig_in |= 0b0000000000001000;
        if ((DIG7_PORT->IDR & DIG7_PIN) == 0)           dig_in |= 0b0000000000000100;
        if ((DIG8_PORT->IDR & DIG8_PIN) == 0)           dig_in |= 0b0000000000000010;
        if ((DIG9_PORT->IDR & DIG9_PIN) == 0)           dig_in |= 0b0000000000000001;

        //debug_printf("dig in: %d\n", dig_in);
        aTxBuffer[REG_RW_DIG_IN_H] = (dig_in & 0xFF00) >> 8;
        aTxBuffer[REG_RW_DIG_IN_L] = (dig_in & 0x00FF);
}


void TIM1_CC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1  ) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        uint32_t mot0_cmd = (((uint32_t)(aTxBuffer[REG_RW_MOT_0_PWM_H])) << 8) | ((uint32_t)(aTxBuffer[REG_RW_MOT_0_PWM_L]));
        uint32_t mot1_cmd = (((uint32_t)aTxBuffer[REG_RW_MOT_1_PWM_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_MOT_1_PWM_L]);
        uint32_t mot2_cmd = (((uint32_t)aTxBuffer[REG_RW_MOT_2_PWM_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_MOT_2_PWM_L]);
        
        TIM_SetCompare1(TIM1, mot0_cmd);
        TIM_SetCompare2(TIM1, mot1_cmd);      
        TIM_SetCompare3(TIM1, mot2_cmd);
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        uint32_t servo0_cmd = (((uint32_t)aTxBuffer[REG_RW_SERVO_0_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_SERVO_0_L]);
        uint32_t servo1_cmd = (((uint32_t)aTxBuffer[REG_RW_SERVO_1_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_SERVO_1_L]);

        TIM_SetCompare1(TIM3, servo0_cmd);
        TIM_SetCompare2(TIM3, servo1_cmd);
    }
}

void TIM8_CC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET)
    {
        TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);

        uint32_t mot3_cmd = (((uint32_t)aTxBuffer[REG_RW_MOT_3_PWM_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_MOT_3_PWM_L]);

        TIM_SetCompare1(TIM8, mot3_cmd);
    }
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM9, TIM_IT_Update);

        uint32_t servo2_cmd = (((uint32_t)aTxBuffer[REG_RW_SERVO_2_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_SERVO_2_L]);
        uint32_t servo3_cmd = (((uint32_t)aTxBuffer[REG_RW_SERVO_3_H]) << 8) | ((uint32_t)aTxBuffer[REG_RW_SERVO_3_L]);

        TIM_SetCompare1(TIM9, servo2_cmd);
        TIM_SetCompare2(TIM9, servo3_cmd);
    }
}




void motor_update(int16_t bemf_val, pid_struct * pids, uint8_t channel, uint8_t motor_mode)
{
    static const uint8_t MOT_SP_REG_STRIDE = 2;
    static const uint8_t MOT_PWM_REG_STRIDE = 2;
    static const uint8_t MOT_POS_REG_STRIDE = 4; // 32 bit position
    static const uint8_t MOT_POS_GOAL_REG_STRIDE = 4;
    
    if ((bemf_val < 8) && (bemf_val > -8)) bemf_val = 0;

    const uint16_t mot_pos_address = REG_RW_MOT_0_B3 + MOT_POS_REG_STRIDE * channel;
    int32_t pos = ((int32_t)aTxBuffer[mot_pos_address] << 24) | ((int32_t)aTxBuffer[mot_pos_address+1] << 16) | ((int32_t)aTxBuffer[mot_pos_address+2] << 8) | ((int32_t)aTxBuffer[mot_pos_address+3]);
    pos += bemf_val;
    aTxBuffer[mot_pos_address]   = (pos & 0xFF000000) >> 24;
    aTxBuffer[mot_pos_address+1] = (pos & 0x00FF0000) >> 16;
    aTxBuffer[mot_pos_address+2] = (pos & 0x0000FF00) >> 8;
    aTxBuffer[mot_pos_address+3] = (pos & 0x000000FF);


    uint8_t motor_done = aRxBuffer[REG_RW_MOT_DONE] & (1 << channel);

    if (motor_mode == 0)
    {
        // PWM mode
    }
    else if (motor_done ==0)
    {
        // PID Control Needed   (MAV, MTP, MRP)
        const uint16_t goal_address = REG_RW_MOT_0_SP_H + MOT_SP_REG_STRIDE * channel;
        int32_t goal = (((int16_t)(aTxBuffer[goal_address])) << 8) | ((int16_t)(aTxBuffer[goal_address+1]));
        if (aTxBuffer[goal_address] & 0b10000000) goal |= 0xFFFF0000;// TODO: cleanup negative goal handling
        const int32_t current = (int32_t)bemf_val;
    
        const float pErr =  (float)(goal-current); //(float)(goal - current);
        const float dErr = pErr - pids->prevErr;

        pids->prevErr = pErr;
        pids->iErr += pErr;

        // clamp error integral
        static const float max_iErr = 10000.0f; 
        if (pids->iErr > max_iErr)
        {
            pids->iErr = max_iErr;
        }
        else if (pids->iErr < -max_iErr)
        {
            pids->iErr = -max_iErr;
        }

        int32_t cmd = (int32_t)(pids->kP*pErr + pids->kI*pids->iErr + pids->kD*dErr);

        // Move To Position or  Move Relative Position goal checks
        if (motor_mode == MOT_MODE_MTP || motor_mode == MOT_MODE_MRP)
        {
            const uint16_t mot_pos_goal_address = REG_W_MOT_0_GOAL_B3 + MOT_POS_GOAL_REG_STRIDE * channel;
            int32_t pos_goal = ((int32_t)aTxBuffer[mot_pos_goal_address] << 24) | ((int32_t)aTxBuffer[mot_pos_goal_address+1] << 16) | ((int32_t)aTxBuffer[mot_pos_goal_address+2] << 8) | ((int32_t)aTxBuffer[mot_pos_goal_address+3]);

            if (goal < 0)
            {
                if(pos < pos_goal){
                    cmd = 0;
                    aRxBuffer[REG_RW_MOT_DONE] |= (1 << channel);
                }
            }
            else
            {
                if(pos > pos_goal){
                    cmd = 0;
                    aRxBuffer[REG_RW_MOT_DONE] |= (1 << channel);
                }
            }
        }

        // clamp output signal
        static const uint16_t max_cmd = 400;
        if (cmd > max_cmd) cmd = max_cmd;
        if (cmd < -max_cmd) cmd = -max_cmd;

        // TODO: more concise way of this?
        // something like   ~(0b11 << (6- 2*channel))
        uint8_t dir_mask = 0xff;
        switch(channel)
        {
            case 0:
                dir_mask = 0b11111100;
                break;
            case 1:
                dir_mask = 0b11110011;
                break;
            case 2:
                dir_mask = 0b11001111;
                break;
            case 3:
                dir_mask = 0b00111111;
                break;
            default:
                break;
        }

        // handle direction
        if (cmd < 0)
        {
            cmd = -cmd;
            //motor_mode[0] = 2; // reverse
            aTxBuffer[REG_RW_MOT_DIRS] =  (aTxBuffer[REG_RW_MOT_DIRS] & dir_mask) | (0b10 << (2*channel));
        }
        else
        {
            //motor_mode[0] = 1; //forward
            aTxBuffer[REG_RW_MOT_DIRS] = (aTxBuffer[REG_RW_MOT_DIRS] & dir_mask) | (0b01 << (2*channel));
        }

        // TODO: adjust for a physical deadband?

        // TODO: adjust or nonlinearity?

        // TODO: add simulated deadband?




        // set command
        uint16_t ucmd = (uint16_t)cmd;

        // TODO multi channel is a little messy here
        aTxBuffer[REG_RW_MOT_0_PWM_H + MOT_PWM_REG_STRIDE * channel] = (ucmd & 0xFF00) >> 8;
        aTxBuffer[REG_RW_MOT_0_PWM_L + MOT_PWM_REG_STRIDE * channel] = (ucmd & 0x00FF);
    }

    update_motor_modes();
}


void clear_rx_buffer()
{
    for(uint16_t i=0;i<BUFFERSIZE;i++)
    {
        aRxBuffer[i]=0x00;
    }
}


void clear_tx_buffer()
{
    for(uint16_t i=0;i<BUFFERSIZE;i++)
    {
        aTxBuffer[i]=0x00;
    }
}


void init_rx_buffer()
{
    clear_rx_buffer();
}


void set_pid_reg_defaults(uint8_t channel)
{
    uint16_t default_p = 2000;
    uint16_t default_pd = 1000;
    uint16_t default_i = 200;
    uint16_t default_id = 1000;
    uint16_t default_d = 1;
    uint16_t default_dd = 1000;

    const uint8_t reg_addys_per_motor = 3*4; // 3 coeffs, each are 4 bytes (2 for numerator, 2 denominator)

    uint8_t p_addr = REG_W_PID_0_P_H + channel * reg_addys_per_motor;
    uint8_t pd_addr = p_addr + 2;
    uint8_t i_addr = pd_addr + 2;
    uint8_t id_addr = i_addr + 2;
    uint8_t d_addr = id_addr + 2;
    uint8_t dd_addr = d_addr + 2;

    aTxBuffer[p_addr]     = (uint8_t)((default_p & 0xFF00) >> 8);
    aTxBuffer[p_addr+1]   = (uint8_t)(default_p & 0x00FF);
    aTxBuffer[pd_addr]    = (uint8_t)((default_pd & 0xFF00) >> 8);
    aTxBuffer[pd_addr+1]  = (uint8_t)(default_pd & 0x00FF);

    aTxBuffer[i_addr]     = (uint8_t)((default_i & 0xFF00) >> 8);
    aTxBuffer[i_addr+1]   = (uint8_t)(default_i & 0x00FF);
    aTxBuffer[id_addr]    = (uint8_t)((default_id & 0xFF00) >> 8);
    aTxBuffer[id_addr+1]  = (uint8_t)(default_id & 0x00FF);

    aTxBuffer[d_addr]     = (uint8_t)((default_d & 0xFF00) >> 8);
    aTxBuffer[d_addr+1]   = (uint8_t)(default_d & 0x00FF);
    aTxBuffer[dd_addr]    = (uint8_t)((default_dd & 0xFF00) >> 8);
    aTxBuffer[dd_addr+1]  = (uint8_t)(default_dd & 0x00FF);
}


void init_tx_buffer()
{
    clear_tx_buffer();
    
    aTxBuffer[REG_R_START] = 'J';

    const uint16_t init_servo_cmd = 1500;
    aTxBuffer[REG_RW_SERVO_0_H] = (init_servo_cmd & 0xFF00) >> 8; 
    aTxBuffer[REG_RW_SERVO_0_L] = (init_servo_cmd & 0x00FF); 
    aTxBuffer[REG_RW_SERVO_1_H] = (init_servo_cmd & 0xFF00) >> 8; 
    aTxBuffer[REG_RW_SERVO_1_L] = (init_servo_cmd & 0x00FF); 
    aTxBuffer[REG_RW_SERVO_2_H] = (init_servo_cmd & 0xFF00) >> 8; 
    aTxBuffer[REG_RW_SERVO_2_L] = (init_servo_cmd & 0x00FF); 
    aTxBuffer[REG_RW_SERVO_3_H] = (init_servo_cmd & 0xFF00) >> 8; 
    aTxBuffer[REG_RW_SERVO_3_L] = (init_servo_cmd & 0x00FF); 

    uint8_t i;
    for (i = 0; i < 4; ++i) set_pid_reg_defaults(i);
}


void update_bemfs(int32_t * bemf_vals)
{
    int32_t adc_motor_vals[8];

    adc_motor_vals[0] = slow_adc(MOT0_BEMF_H_ADX, MOT0_BEMF_H_CHAN);
    adc_motor_vals[1] = slow_adc(MOT0_BEMF_L_ADX, MOT0_BEMF_L_CHAN);
    adc_motor_vals[2] = slow_adc(MOT1_BEMF_H_ADX, MOT1_BEMF_H_CHAN);
    adc_motor_vals[3] = slow_adc(MOT1_BEMF_L_ADX, MOT1_BEMF_L_CHAN);
    adc_motor_vals[4] = slow_adc(MOT2_BEMF_H_ADX, MOT2_BEMF_H_CHAN);
    adc_motor_vals[5] = slow_adc(MOT2_BEMF_L_ADX, MOT2_BEMF_L_CHAN);
    adc_motor_vals[6] = slow_adc(MOT3_BEMF_H_ADX, MOT3_BEMF_H_CHAN);
    adc_motor_vals[7] = slow_adc(MOT3_BEMF_L_ADX, MOT3_BEMF_L_CHAN);

    bemf_vals[0] = adc_motor_vals[0] - adc_motor_vals[1];
    bemf_vals[1] = adc_motor_vals[2] - adc_motor_vals[3];
    bemf_vals[2] = adc_motor_vals[4] - adc_motor_vals[5];
    bemf_vals[3] = adc_motor_vals[6] - adc_motor_vals[7];
}


void update_pid_coeffs_from_reg(pid_struct * pids, uint8_t channel)
{
        const uint8_t reg_addys_per_motor = 3*4; // 3 coeffs, each are 4 bytes (2 for numerator, 2 denominator)

        uint8_t p_addr = REG_W_PID_0_P_H + channel * reg_addys_per_motor;
        uint8_t pd_addr = p_addr + 2;
        uint8_t i_addr = pd_addr + 2;
        uint8_t id_addr = i_addr + 2;
        uint8_t d_addr = id_addr + 2;
        uint8_t dd_addr = d_addr + 2;

        uint16_t p =  (((uint16_t)(aTxBuffer[p_addr]))  << 8) | ((uint16_t)(aTxBuffer[p_addr+1]));
        uint16_t pd = (((uint16_t)(aTxBuffer[pd_addr])) << 8) | ((uint16_t)(aTxBuffer[pd_addr+1]));
        uint16_t i =  (((uint16_t)(aTxBuffer[i_addr]))  << 8) | ((uint16_t)(aTxBuffer[i_addr+1]));
        uint16_t id = (((uint16_t)(aTxBuffer[id_addr])) << 8) | ((uint16_t)(aTxBuffer[id_addr+1]));
        uint16_t d =  (((uint16_t)(aTxBuffer[d_addr]))  << 8) | ((uint16_t)(aTxBuffer[d_addr+1]));
        uint16_t dd = (((uint16_t)(aTxBuffer[dd_addr])) << 8) | ((uint16_t)(aTxBuffer[dd_addr+1]));

        pids->kP = (float)p / (float)pd;
        pids->kI = (float)i / (float)id;
        pids->kD = (float)d / (float)dd;
}


void update_dig_pin_from_reg(uint32_t pin, GPIO_TypeDef* port, uint8_t output_enable, uint8_t pullup_enable)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;		
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    if (output_enable)
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    }
    else
    {
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    }

    if (pullup_enable)
    {
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    }
    else
    {
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    }

    GPIO_Init(port, &GPIO_InitStructure);
}


void update_dig_pin_configs()
{
    update_dig_pin_from_reg(DIG0_PIN, DIG0_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 1, aTxBuffer[REG_RW_DIG_PE_L] & 1);
    update_dig_pin_from_reg(DIG1_PIN, DIG1_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 2, aTxBuffer[REG_RW_DIG_PE_L] & 2);
    update_dig_pin_from_reg(DIG2_PIN, DIG2_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 4, aTxBuffer[REG_RW_DIG_PE_L] & 4);
    update_dig_pin_from_reg(DIG3_PIN, DIG3_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 8, aTxBuffer[REG_RW_DIG_PE_L] & 8);
    update_dig_pin_from_reg(DIG4_PIN, DIG4_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 16, aTxBuffer[REG_RW_DIG_PE_L] & 16);
    update_dig_pin_from_reg(DIG5_PIN, DIG5_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 32, aTxBuffer[REG_RW_DIG_PE_L] & 32);
    update_dig_pin_from_reg(DIG6_PIN, DIG6_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 64, aTxBuffer[REG_RW_DIG_PE_L] & 64);
    update_dig_pin_from_reg(DIG7_PIN, DIG7_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 128, aTxBuffer[REG_RW_DIG_PE_L] & 128);

    update_dig_pin_from_reg(DIG8_PIN, DIG8_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 1, aTxBuffer[REG_RW_DIG_PE_H] & 1);
    update_dig_pin_from_reg(DIG9_PIN, DIG9_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 2, aTxBuffer[REG_RW_DIG_PE_H] & 2);
    update_dig_pin_from_reg(DIG10_PIN, DIG10_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 4, aTxBuffer[REG_RW_DIG_PE_H] & 4);
    update_dig_pin_from_reg(DIG11_PIN, DIG11_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 8, aTxBuffer[REG_RW_DIG_PE_H] & 8);
    update_dig_pin_from_reg(DIG12_PIN, DIG12_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 16, aTxBuffer[REG_RW_DIG_PE_H] & 16);
    update_dig_pin_from_reg(DIG13_PIN, DIG13_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 32, aTxBuffer[REG_RW_DIG_PE_H] & 32);
}

void update_dig_pin(uint32_t pin, GPIO_TypeDef* port, uint8_t output_enable, uint8_t out_val, uint8_t pin_num)
{
    if (output_enable)
    {
        if (out_val)
        {
            // set pin high
            port->BSRRL |= pin;
        }
        else
        {
            // set pin low
            port->BSRRH |= pin;
        }
    }
    else
    {
        uint8_t dig_in_addr;
        uint8_t shift;
        if (pin_num > 7)
        {   
            dig_in_addr = REG_RW_DIG_IN_H;
            shift = pin_num-8;
        }
        else
        {
            dig_in_addr = REG_RW_DIG_IN_L;
            shift = pin_num;
        }

        // IDR is low when pin is high
        if ((port->IDR & pin) == 0)
        {
            // pin is high
            aTxBuffer[dig_in_addr] |= (1<<shift);
        }
        else
        {
            // pin is low
            aTxBuffer[dig_in_addr] &= ~(1<<shift);
        }
    }
}

void update_dig_pins()
{

    update_dig_pin(DIG0_PIN, DIG0_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 1, aTxBuffer[REG_RW_DIG_OUT_L] & 1, 0);
    update_dig_pin(DIG1_PIN, DIG1_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 2, aTxBuffer[REG_RW_DIG_OUT_L] & 2, 1);
    update_dig_pin(DIG2_PIN, DIG2_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 4, aTxBuffer[REG_RW_DIG_OUT_L] & 4, 2);
    update_dig_pin(DIG3_PIN, DIG3_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 8, aTxBuffer[REG_RW_DIG_OUT_L] & 8, 3);
    update_dig_pin(DIG4_PIN, DIG4_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 16, aTxBuffer[REG_RW_DIG_OUT_L] & 16, 4);
    update_dig_pin(DIG5_PIN, DIG5_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 32, aTxBuffer[REG_RW_DIG_OUT_L] & 32, 5);
    update_dig_pin(DIG6_PIN, DIG6_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 64, aTxBuffer[REG_RW_DIG_OUT_L] & 64, 6);
    update_dig_pin(DIG7_PIN, DIG7_PORT, aTxBuffer[REG_RW_DIG_OE_L] & 128, aTxBuffer[REG_RW_DIG_OUT_L] & 128, 7);

    update_dig_pin(DIG8_PIN, DIG8_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 1, aTxBuffer[REG_RW_DIG_OUT_H] & 1, 8);
    update_dig_pin(DIG9_PIN, DIG9_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 2, aTxBuffer[REG_RW_DIG_OUT_H] & 2, 9);
    update_dig_pin(DIG10_PIN, DIG10_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 4, aTxBuffer[REG_RW_DIG_OUT_H] & 4, 10);
    update_dig_pin(DIG11_PIN, DIG11_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 8, aTxBuffer[REG_RW_DIG_OUT_H] & 8, 11);
    update_dig_pin(DIG12_PIN, DIG12_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 16, aTxBuffer[REG_RW_DIG_OUT_H] & 16, 12);
    update_dig_pin(DIG13_PIN, DIG13_PORT, aTxBuffer[REG_RW_DIG_OE_H] & 32, aTxBuffer[REG_RW_DIG_OUT_H] & 32, 13);
}


void init_pid_struct(pid_struct * pids, uint8_t channel)
{
    pids->prevErr = 0.0f;
    pids->iErr = 0.0f;
    update_pid_coeffs_from_reg(pids, channel);
}



void config_adc_in_from_regs()
{
    // analog in pins
    configAnalogInPin(AIN0_PIN, AIN0_PORT, aTxBuffer[REG_RW_ADC_PE] & 1);
    configAnalogInPin(AIN1_PIN, AIN1_PORT, aTxBuffer[REG_RW_ADC_PE] & 2);
    configAnalogInPin(AIN2_PIN, AIN2_PORT, aTxBuffer[REG_RW_ADC_PE] & 4);
    configAnalogInPin(AIN3_PIN, AIN3_PORT, aTxBuffer[REG_RW_ADC_PE] & 8);
    configAnalogInPin(AIN4_PIN, AIN4_PORT, aTxBuffer[REG_RW_ADC_PE] & 16);
    configAnalogInPin(AIN5_PIN, AIN5_PORT, aTxBuffer[REG_RW_ADC_PE] & 32);

    // adc also used for battery level sensing
    configAnalogInPin(ADC_BATT_PIN, ADC_BATT_PORT, 0);
}

void handle_dma()
{
    static uint8_t expected = 0;

    if ( (DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET) &&  (DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET))
    {
        // TODO: make this interrupt-based
        if (aRxBuffer[0] == 'J'  && aRxBuffer[REG_READABLE_COUNT-1] == 'S' && aRxBuffer[1] == WALLABY_SPI_VERSION)
        {
            expected += 1;
            if (aRxBuffer[2] != expected) debug_printf("Missed packet(s) got ID %d expected %d\n", aRxBuffer[2], expected);

            // handle recently compled DMA transfer
            //debug_printf("%x %x %x %x %x %x %x %x %x %x\n", aRxBuffer[0], aRxBuffer[1], aRxBuffer[2], aRxBuffer[3], aRxBuffer[4], aRxBuffer[5], aRxBuffer[6], aRxBuffer[7], aRxBuffer[8], aRxBuffer[9]);    
            uint8_t num_regs = aRxBuffer[3];
            uint8_t j;
            for (j = 0; j < num_regs; j+=2)
            {
                // TODO: register/value checks before assignment
                aTxBuffer[aRxBuffer[4+j]] = aRxBuffer[4+j+1];
            }
        }
        else
        {
            debug_printf("got bad spi packet\n");
            if (aRxBuffer[1] == WALLABY_SPI_VERSION)
            {
                debug_printf("SPI protocol version mismatch\n");

            }
        }
        // Clear DMA Transfer Complete Flags so we can notice when a transfer happens again
        DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4); // tx
        DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3); // rx
    }
}

void DMA1_Stream3_IRQHandler()
{
    handle_dma();
}


void DMA1_Stream4_IRQHandler()
{
    handle_dma();
}


int main()
{
    int32_t bemf_vals[4];

    init();

    // set up SPI buffers
    init_rx_buffer();
    init_tx_buffer();

    // TODO: remove
    
    aTxBuffer[REG_RW_MOT_0_SP_H] = ((-500) & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MOT_0_SP_L] = ((-500) & 0x00FF);
    aTxBuffer[REG_RW_MOT_1_SP_H] = ((500) & 0xFF00) >> 8;
    aTxBuffer[REG_RW_MOT_1_SP_L] = ((500) & 0x00FF);
    aTxBuffer[REG_RW_MOT_MODES] = 0b00001111;
    aTxBuffer[REG_W_MOT_0_GOAL_B3] = (-420000 & 0xFF000000) >> 24;
    aTxBuffer[REG_W_MOT_0_GOAL_B2] = (-420000 & 0x00FF0000) >> 16;
    aTxBuffer[REG_W_MOT_0_GOAL_B1] = (-420000 & 0x0000FF00) >> 8;
    aTxBuffer[REG_W_MOT_0_GOAL_B0] = (-420000 & 0x000000FF);

    // turn low bit on
    aTxBuffer[REG_RW_DIG_OE_H] = 0b00010000;
    aTxBuffer[REG_RW_DIG_OE_L] = 0xFF;
    aTxBuffer[REG_RW_DIG_OUT_H] = 16; // led high
    aTxBuffer[REG_RW_DIG_OUT_L] = 1; // pin 0 high
    

    // set up pid structs
    pid_struct pid_structs[4];
    {
        uint8_t i;
        for (i = 0; i < 4; ++i) init_pid_struct(&(pid_structs[i]), i);
    }
    
    debug_printf("starting\n");
    
    update_dig_pin_configs(); // TODO: also in loop, or when reg is updated
    config_adc_in_from_regs();// TODO: also in loop, or when reg is updated

    // Loop until button is pressed
    uint32_t count = 0;
    while (1)
    {
        count += 1;
        {
            // only sample motor backemf 1/4 of the time
            const uint8_t bemf_update_time =  (count % 4 == 1);
            if (bemf_update_time)
            {
                // idle breaking
                MOT0_DIR1_PORT->BSRRH |= MOT0_DIR1; 
                MOT0_DIR2_PORT->BSRRH |= MOT0_DIR2;
                MOT1_DIR1_PORT->BSRRH |= MOT1_DIR1; 
                MOT1_DIR2_PORT->BSRRH |= MOT1_DIR2;
                MOT2_DIR1_PORT->BSRRH |= MOT2_DIR1; 
                MOT2_DIR2_PORT->BSRRH |= MOT2_DIR2;
                MOT3_DIR1_PORT->BSRRH |= MOT3_DIR1; 
                MOT3_DIR2_PORT->BSRRH |= MOT3_DIR2;
            }
            // let the motor coast
            // delay_us(700);
            // now I squeeze in most sensor updates instead of just sleeping
            uint32_t before = usCount;
            update_dig_pins();
            adc_update();
            readAccel();
            readMag();
            readGyro();   
            uint32_t sensor_update_time = usCount - before;
            const uint16_t us_delay_needed = 700;
            const uint8_t got_time_to_burn = sensor_update_time < us_delay_needed;

            if (got_time_to_burn)
            {
                // sleep the remainder of the coasting period before sampling bemf
                delay_us(us_delay_needed-sensor_update_time); // TODO ideally, we are servicing SPI requests
            }

            if (bemf_update_time)
            {
                update_bemfs(bemf_vals);

                // set the motor directions back up
                update_motor_modes();

                uint8_t channel;
                for (channel = 0; channel < 4; ++channel)
                {
                    uint8_t shift = 2*channel;
                    uint8_t motor_mode = (aTxBuffer[REG_RW_MOT_MODES] & (0b11 << shift)) >> shift;
                    // TODO:  MRP, MRP... for now we just do MAV
                    motor_update(bemf_vals[channel], &pid_structs[channel], channel, motor_mode);
                }

            }
            else
            {
                // updating sensors doesnt' take as long as all of the adc for backemf updates
                // sleep a bit so that our time through the loop is consistent for PID control
                delay_us(222); // TODO ideally, we are servicing SPI requests
            }
        }

        // close program if button is pushed
        // TODO: remove this developer functionality
        if (aTxBuffer[REG_RW_DIG_IN_H] & 0b00100000) break; // button

    } 

    // set all motor pwms to 0
    aTxBuffer[REG_RW_MOT_0_PWM_H] = 0;
    aTxBuffer[REG_RW_MOT_0_PWM_L] = 0;
    aTxBuffer[REG_RW_MOT_1_PWM_H] = 0;
    aTxBuffer[REG_RW_MOT_1_PWM_L] = 0;
    aTxBuffer[REG_RW_MOT_2_PWM_H] = 0;
    aTxBuffer[REG_RW_MOT_2_PWM_L] = 0;
    aTxBuffer[REG_RW_MOT_3_PWM_H] = 0;
    aTxBuffer[REG_RW_MOT_3_PWM_L] = 0;

    // set all motors to idle state
    idle_motor_dirs();

    spi2_dma_cleanup();

    debug_printf("done\n");
}