#include <stdint.h>
#include "my_library.h"
#include "my_SPI1.h"
#include "my_I2C1.h"
#include "my_LCD1602.h"
#include "my_MPU6050.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void) {
    systick_init(); // SysTick初始化
    blinkblink_init(); // 設定板子上的自帶的燈炮，用以閃燈
    SPI1_Slave_Init(); // SPI1 Slave模式初始化
    I2C1_Master_Init(); // I2C1 Master模式初始化
    LCD1602_Init(); // LCD1602初始化 (LCD1602由I2C1控制)
    MPU6050_Init(); // MPU6050初始化 (MPU6050由I2C1控制)

    union {
        float f;
        uint8_t bytes[4];
    } T; // 用以紀錄溫度

    char T_string[20]; // 用以紀錄溫度(字串形式)

    int mpu6050_error = 0; // 用以紀錄MPU6050是否故障
    int lcd1602_error = 0; // 用以紀錄LCD1602是否故障

    delay_ms(1000);
    blinkblink(5, 100); // 閃燈5次

    ADC1_Init(); // ADC1初始化
    unsigned int adc1_value = ADC1_Read(); // ADC1讀取電壓
    unsigned int TIM3_ms_time = 100 + ((adc1_value * 1900) / 4095); // 依讀取之電壓值設定Timer3計時時間

    LCD1602_Put_Cursor(0,0); // LCD1602設定游標位置
    LCD1602_Send_String("Temperature:"); // LCD1602顯示字串

    TIM3_ms_Init(TIM3_ms_time); // Timer3初始化，計時時間初始值設為TIM3_ms_time
    TIM3_ms_Enable(); // Timer3啟用


    while (1) {
        if(Timer3_ms_flag) { // 若Timer3計時時間到
            blinkblink(1, 50); // 快速閃燈一次

            T.f = MPU6050_ReadTemperature(); // MPU6050讀取溫度，並寫入float T
            mpu6050_error = i2c1_error; // 紀錄MPU6050是否發生故障

            LCD1602_Put_Cursor(1,0); // LCD1602設定游標位置

            if(mpu6050_error == 0) { // 若MPU6050沒有發生故障
                // 將溫度資料float T轉換成字串T_string，並在LCD1602上顯示
                float_to_string(T.f, T_string, 2, 2);
                LCD1602_Send_String(T_string);
                LCD1602_Send_String("        ");
            }
            else { // 若MPU6050發生故障
                LCD1602_Send_String("MPU6050 Error"); // LCD1602顯示故障訊息
            }
            lcd1602_error = i2c1_error; // 紀錄LCD1602是否發生故障


            if(mpu6050_error == 1) { // 若MPU6050發生故障
                SPI1_TX_Buf_Write(0, '1'); // spi1_tx_buf[0]寫入為'1'，每次SPI通訊後雙方以收到的第一個資料作為故障代碼，'1'表示MPU6050發生故障
            }
            else if(lcd1602_error == 1) { // 若LCD1602發生故障
                SPI1_TX_Buf_Write(0, '2'); // 寫入故障代碼為'2'，表示LCD1602發生故障
            }
            else { // 若MPU6050、LCD1602皆沒有發生故障
                SPI1_TX_Buf_Write(0, '0'); // 寫入故障代碼為'0'，表示一切正常
            }

            // 將溫度資料float T寫入spi1_tx_buf[1:4]，當SPI Master發起通訊時會被傳送出去
            SPI1_TX_Buf_Write(1, T.bytes[0]);
            SPI1_TX_Buf_Write(2, T.bytes[1]);
            SPI1_TX_Buf_Write(3, T.bytes[2]);
            SPI1_TX_Buf_Write(4, T.bytes[3]);

            adc1_value = ADC1_Read(); // ADC1讀取電壓
            TIM3_ms_time = 100 + ((adc1_value * 1900) / 4095); // 依讀取之電壓值設定Timer3計時時間
            TIM3_ms_ChangeTime(TIM3_ms_time); // 改變Timer3計時時間為TIM3_ms_time

            Timer3_ms_flag = 0;
        }
    }
}
