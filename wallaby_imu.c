#include "wallaby_imu.h"
#include "wallaby.h"


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
        //debug_printf("Accel/Magn identified itself\n");
    }else{
        //debug_printf("Accel/Magn did not respond/identify itself (regval=%d)\n",regval);
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

    if (regval == GYRO_ID){
      //debug_printf("Gyro identified itself\n");
    }else{
      //debug_printf("Gyro did not respond/identify itself (regval=%d)\n",regval);
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


