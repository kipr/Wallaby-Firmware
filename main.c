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
#include "wallaby_r0.h"


#ifdef USE_CROSS_STUDIO_DEBUG
    #include <__cross_studio_io.h>
#else
    int debug_printf(const char *format, ...){};
    void debug_exit(uint8_t val){};
#endif


static volatile uint32_t usCount;


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


// TODO: remove/replace, just for benchmarking
float pid()
{
    float err = 0.0;
    unsigned int i;
    float kp = 1.2;
    float ki = 1.4;
    float kd = 1.6;

    float P = 1.5;
    float I = 1.3;
    float D = 2.5;
    for (i = 0; i < 400000; ++i){ 
      //err = 0.1*(5.4-2.3) + 0.4*(4.3-2.4) + 0.3*(5.6+7.5);
     // 0.1*(5.4-2.3) + 0.4*(4.3-2.4) + 0.3*(5.6+7.5); err =  0.1*(2.3) + 0.4*(4.3-2.4) + 0.3*(5.6+7.5);
      //err = 0.1;// + 0.4 + 0.3;
      err = kp*(P+2.4) + ki*(I+4.5) + kd*(D+2.3);
    }
    return err;
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


void configAnalogInPin(uint32_t pin, GPIO_TypeDef* port) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(port, &GPIO_InitStructure);
}

void configDigitalInPin(uint32_t pin, GPIO_TypeDef* port)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;		
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(port, &GPIO_InitStructure);
}

void configDigitalOutPin(uint32_t pin, GPIO_TypeDef* port)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(port, &GPIO_InitStructure);
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
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
}


uint16_t slow_adc(ADC_TypeDef * bus, uint8_t channel)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // ADC Common Init
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
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

    ADC_SoftwareStartConv(bus);
    while(ADC_GetFlagStatus(bus, ADC_FLAG_EOC) == RESET);
    uint16_t val = ADC_GetConversionValue(bus);

    // Disable ADC1
    ADC_Cmd(bus, DISABLE);

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
        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
        SPI_Init(SPI3, &SPI_InitStruct);
    }


    SPI_Cmd(SPI3, ENABLE);
    
}


uint8_t SPI3_write(uint8_t data)
{
	SPI3->DR = data; // write data
	while( !(SPI3->SR & SPI_I2S_FLAG_TXE) ); // wait for transmit
	while( !(SPI3->SR & SPI_I2S_FLAG_RXNE) ); // wait for receive
	while( SPI3->SR & SPI_I2S_FLAG_BSY ); // wait unit SPI is done
	return SPI3->DR; // return data
}


void setupAccelMag()
{

    // select Accel/Mag
    SPI3_CS0_PORT->BSRRH |= SPI3_CS0; // chip select low
    delay_us(100); 

    uint8_t regval;
    // request "Who am I)
    SPI3_write(0x8F); // write register
    regval = SPI3_write(0x00); // write dummy val to get contents
    SPI3_CS0_PORT->BSRRL |= SPI3_CS0; // chip select low

    if (regval == 73)
    {
      debug_printf("Accel/Magn identified itself\n");
    } else
    {
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



    debug_printf("accel %d %d %d \n", accel_x, accel_y, accel_z);
}


void setupGyro()
{
    // select Accel/Mag
    SPI3_CS1_PORT->BSRRH |= SPI3_CS1; // chip select low
    delay_us(100); 

    uint8_t regval;
    // request "Who am I)
    SPI3_write(0x80 | 0x0F); // write register
    regval = SPI3_write(0x00); // write dummy val to get contents
    SPI3_CS1_PORT->BSRRL |= SPI3_CS1; // chip select low

    if (regval == 211)
    {
      debug_printf("Gyro identified itself\n");
    } else
    {
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
    SPI3_write(0b00000000);
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


    debug_printf("gyro %d %d %d \n", gyro_x, gyro_y, gyro_z);
}


void main(void)
{
    init180MHz(); // switch over to 180 MHz via the external 24 MHz crystal and a PLL

    debug_printf("Starting up...\n"); 

    initSystick(); // get a wallclock started, ticking at 1ms

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

    configDigitalOutPin(LED1_PIN, LED1_PORT);

    // motor 0 
    configMotorPin(MOT0_DIR1, MOT0_DIR1_PORT);
    configMotorPin(MOT0_DIR2, MOT0_DIR2_PORT);
    configMotorPin(MOT0_PWM, MOT0_PWM_PORT);

    // motor 1
    configMotorPin(MOT1_DIR1, MOT1_DIR1_PORT);
    configMotorPin(MOT1_DIR2, MOT1_DIR2_PORT);
    configMotorPin(MOT1_PWM, MOT1_PWM_PORT);

    // motor 2
    configMotorPin(MOT2_DIR1, MOT2_DIR1_PORT);
    configMotorPin(MOT2_DIR2, MOT2_DIR2_PORT);
    configMotorPin(MOT2_PWM, MOT2_PWM_PORT);

    // motor 3
    configMotorPin(MOT3_DIR1, MOT3_DIR1_PORT);
    configMotorPin(MOT3_DIR2, MOT3_DIR2_PORT);
    configMotorPin(MOT3_PWM, MOT3_PWM_PORT);


    // motor bemf adc sensing
    configBEMFPin(MOT0_BEMF_H, MOT0_BEMF_H_PORT);
    configBEMFPin(MOT0_BEMF_L, MOT0_BEMF_L_PORT);
    configBEMFPin(MOT1_BEMF_H, MOT1_BEMF_H_PORT);
    configBEMFPin(MOT1_BEMF_L, MOT1_BEMF_L_PORT);

    configBEMFPin(MOT2_BEMF_H, MOT2_BEMF_H_PORT);
    configBEMFPin(MOT2_BEMF_L, MOT2_BEMF_L_PORT);
    configBEMFPin(MOT3_BEMF_H, MOT3_BEMF_H_PORT);
    configBEMFPin(MOT3_BEMF_L, MOT3_BEMF_L_PORT);

    // servos
    configServoPin(SRV0_PWM, SRV0_PWM_PORT);
    configServoPin(SRV1_PWM, SRV1_PWM_PORT);
    configServoPin(SRV2_PWM, SRV2_PWM_PORT);
    configServoPin(SRV3_PWM, SRV3_PWM_PORT);
    
    // analog in pins
    configAnalogInPin(AIN0_PIN, AIN0_PORT);
    configAnalogInPin(AIN1_PIN, AIN1_PORT);
    configAnalogInPin(AIN2_PIN, AIN2_PORT);
    configAnalogInPin(AIN3_PIN, AIN3_PORT);
    configAnalogInPin(AIN4_PIN, AIN4_PORT);
    configAnalogInPin(AIN5_PIN, AIN5_PORT);

    // adc also used for battery level sensing
    configAnalogInPin(ADC_BATT_PIN, ADC_BATT_PORT);
    
    // digital inputs
    configDigitalInPin(DIG0_PIN, DIG0_PORT);
    configDigitalInPin(DIG1_PIN, DIG1_PORT);
    configDigitalInPin(DIG2_PIN, DIG2_PORT);
    configDigitalInPin(DIG3_PIN, DIG3_PORT);
    configDigitalInPin(DIG4_PIN, DIG4_PORT);
    configDigitalInPin(DIG5_PIN, DIG5_PORT);
    configDigitalInPin(DIG6_PIN, DIG6_PORT);
    configDigitalInPin(DIG7_PIN, DIG7_PORT);
    configDigitalInPin(DIG8_PIN, DIG8_PORT);
    configDigitalInPin(DIG9_PIN, DIG9_PORT);


    int32_t adc_motor_vals[8];
    int32_t adc_in[6];
    int16_t dig_in = 0;
    int32_t adc_batt = 0;


    // Init SPI3 (plus chip selects) for the IMU sensors
    initSPI3(); 

    // wait a bit
    delay_us(5000);

    setupAccelMag();

    setupGyro();
/*
    for (unsigned int i = 0; i < 1000; ++i){
        delay_us(20000);
        //readAccel();
        readGyro();
    }
*/

/*

    for (unsigned int cycles = 0; cycles < 100; ++cycles)
    {
        SRV0_PWM_PORT->BSRRL |= SRV0_PWM;
        SRV1_PWM_PORT->BSRRL |= SRV1_PWM;
        SRV2_PWM_PORT->BSRRL |= SRV2_PWM;
        SRV3_PWM_PORT->BSRRL |= SRV3_PWM;
        delay_us(1000);
        SRV0_PWM_PORT->BSRRH |= SRV0_PWM;
        SRV1_PWM_PORT->BSRRH |= SRV1_PWM;
        SRV2_PWM_PORT->BSRRH |= SRV2_PWM;
        SRV3_PWM_PORT->BSRRH |= SRV3_PWM;
        delay_us(19000);
    }
*/    
    

/*
    // Turn a motor on at full speed
    MOT0_DIR1_PORT->BSRRH |= MOT0_DIR1; 
    MOT0_DIR2_PORT->BSRRL |= MOT0_DIR2;
    MOT0_PWM_PORT->BSRRL |= MOT0_PWM;

    // Turn a motor on at full speed
    MOT1_DIR1_PORT->BSRRH |= MOT1_DIR1; 
    MOT1_DIR2_PORT->BSRRL |= MOT1_DIR2;
    MOT1_PWM_PORT->BSRRL |= MOT1_PWM;

    // Turn a motor on at full speed
    MOT2_DIR1_PORT->BSRRH |= MOT2_DIR1; 
    MOT2_DIR2_PORT->BSRRL |= MOT2_DIR2;
    MOT2_PWM_PORT->BSRRL |= MOT2_PWM;

    // Turn a motor on at full speed
    MOT3_DIR1_PORT->BSRRH |= MOT3_DIR1; 
    MOT3_DIR2_PORT->BSRRL |= MOT3_DIR2;
    MOT3_PWM_PORT->BSRRL |= MOT3_PWM;
*/



    for (unsigned int cycles = 0; cycles < 100; ++cycles)
    {
    
        // LED off (set pin high)
        LED1_PORT->BSRRL |= LED1_PIN;
    

        //adc_in[0] = slow_adc(AIN0_ADX, AIN0_CHAN);

        //adc_in[0] = (adc_in[0] > 100) ? adc_in[0] : 0;
        //uint16_t wait = 45.0;//50.0 * ((float)adc_in[0] * 0.00024420024); // 0-50

        // Turn a motor on at full speed
        //MOT0_DIR1_PORT->BSRRH |= MOT0_DIR1; 
        //MOT0_DIR2_PORT->BSRRL |= MOT0_DIR2;  

        /*
        if (wait > 0){
            MOT0_PWM_PORT->BSRRL |= MOT0_PWM;
            MOT1_PWM_PORT->BSRRL |= MOT1_PWM;
            delay_us(wait);
        }
        */
      
        /*
        if (cycles < 100000){
            delay_us(45);
        }else{
          delay_us(25);
        }
        */


        // turn motor off
        //MOT0_DIR1_PORT->BSRRH |= MOT0_DIR1; 
        //MOT0_DIR2_PORT->BSRRH |= MOT0_DIR2;

        /*
        if (wait < 50){
              MOT0_PWM_PORT->BSRRH |= MOT0_PWM;
              MOT1_PWM_PORT->BSRRH |= MOT1_PWM;
              delay_us(50 - wait);
        }
        */



        // LED on (set pin low)
        LED1_PORT->BSRRH |= LED1_PIN;

        //uint32_t button = GPIOA->IDR & 0x0001;

        int32_t before = usCount;

        adc_motor_vals[0] = slow_adc(MOT0_BEMF_H_ADX, MOT0_BEMF_H_CHAN);
        adc_motor_vals[1] = slow_adc(MOT0_BEMF_L_ADX, MOT0_BEMF_L_CHAN);
        adc_motor_vals[2] = slow_adc(MOT1_BEMF_H_ADX, MOT1_BEMF_H_CHAN);
        adc_motor_vals[3] = slow_adc(MOT1_BEMF_L_ADX, MOT1_BEMF_L_CHAN);
        adc_motor_vals[4] = slow_adc(MOT2_BEMF_H_ADX, MOT2_BEMF_H_CHAN);
        adc_motor_vals[5] = slow_adc(MOT2_BEMF_L_ADX, MOT2_BEMF_L_CHAN);
        adc_motor_vals[6] = slow_adc(MOT3_BEMF_H_ADX, MOT3_BEMF_H_CHAN);
        adc_motor_vals[7] = slow_adc(MOT3_BEMF_L_ADX, MOT3_BEMF_L_CHAN);

        adc_in[0] = slow_adc(AIN0_ADX, AIN0_CHAN);
        adc_in[1] = slow_adc(AIN1_ADX, AIN1_CHAN);
        adc_in[2] = slow_adc(AIN2_ADX, AIN2_CHAN);
        adc_in[3] = slow_adc(AIN3_ADX, AIN3_CHAN);
        adc_in[4] = slow_adc(AIN4_ADX, AIN4_CHAN);
        adc_in[5] = slow_adc(AIN5_ADX, AIN5_CHAN);

        adc_batt = slow_adc(ADC_BATT_ADX, ADC_BATT_CHAN);

        int32_t after = usCount;

      
        //debug_printf("ADC: %d - %d = %d batt = %d in %d ms\n", adc_motor_vals[0], adc_motor_vals[1], (adc_motor_vals[0] - adc_motor_vals[1]), adc_batt, (after-before));
        debug_printf("ADC %d %d %d %d %d %d   BAT %d \n", adc_in[0],  adc_in[1],  adc_in[2],  adc_in[3],  adc_in[4],  adc_in[5], adc_batt);
        
        uint16_t dig_in = 0;
        // IDR is high when pin is low
        if ((DIG0_PORT->IDR & DIG0_PIN) == 0) dig_in |= 0b1000000000;
        if ((DIG1_PORT->IDR & DIG1_PIN) == 0) dig_in |= 0b0100000000;
        if ((DIG2_PORT->IDR & DIG2_PIN) == 0) dig_in |= 0b0010000000;
        if ((DIG3_PORT->IDR & DIG3_PIN) == 0) dig_in |= 0b0001000000;
        if ((DIG4_PORT->IDR & DIG4_PIN) == 0) dig_in |= 0b0000100000;
        if ((DIG5_PORT->IDR & DIG5_PIN) == 0) dig_in |= 0b0000010000;
        if ((DIG6_PORT->IDR & DIG6_PIN) == 0) dig_in |= 0b0000001000;
        if ((DIG7_PORT->IDR & DIG7_PIN) == 0) dig_in |= 0b0000000100;
        if ((DIG8_PORT->IDR & DIG8_PIN) == 0) dig_in |= 0b0000000010;
        if ((DIG9_PORT->IDR & DIG9_PIN) == 0) dig_in |= 0b0000000001;
        
        debug_printf("dig in: %d\n", dig_in);

        delay_us(200000);
 
        // wait
        /*
        if (cycles < 100000){
            delay_us(5);
        }else{
            delay_us(25);
        }
        */
    }

    /*
    for (unsigned int cycles = 0; cycles < 100; ++cycles){
        SRV0_PWM_PORT->BSRRL |= SRV0_PWM;
        SRV1_PWM_PORT->BSRRL |= SRV1_PWM;
        SRV2_PWM_PORT->BSRRL |= SRV2_PWM;
        SRV3_PWM_PORT->BSRRL |= SRV3_PWM;
        delay_us(1500);
        SRV0_PWM_PORT->BSRRH |= SRV0_PWM;
        SRV1_PWM_PORT->BSRRH |= SRV1_PWM;
        SRV2_PWM_PORT->BSRRH |= SRV2_PWM;
        SRV3_PWM_PORT->BSRRH |= SRV3_PWM;
        delay_us(18500);
    }
   */

    debug_printf("Shutting down...\n");

    // Turn motors off 
    MOT0_PWM_PORT->BSRRH |= MOT0_PWM;
    MOT1_PWM_PORT->BSRRH |= MOT1_PWM;
    MOT2_PWM_PORT->BSRRH |= MOT2_PWM;
    MOT3_PWM_PORT->BSRRH |= MOT3_PWM;

    //debug_exit(0);
}
