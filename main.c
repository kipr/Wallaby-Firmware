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

#include "wallaby.h"

volatile uint8_t aTxBuffer[REG_ALL_COUNT]; // =  {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};  //"SPI Master/Slave : Communication between two SPI using DMA";
__IO uint8_t aRxBuffer [REG_ALL_COUNT];

 

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


int main()
{
    int32_t bemf_vals[4];

    init();

    // set up SPI buffers
    init_rx_buffer();
    init_tx_buffer();

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
                delay_us(us_delay_needed-sensor_update_time);
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
                    motor_update(bemf_vals[channel], &pid_structs[channel], channel, motor_mode);
                }

            }
            else
            {
                // updating sensors doesnt' take as long as all of the adc for backemf updates
                // sleep a bit so that our time through the loop is consistent for PID control
                delay_us(222);
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