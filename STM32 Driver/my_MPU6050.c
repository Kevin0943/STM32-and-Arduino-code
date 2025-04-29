#include "my_library.h"
#include "my_I2C1.h"
#include "my_MPU6050.h"


/**
===== 簡述 =====
MPU6050 Library
需搭配my_I2C1.c使用


===== 使用說明 =====
使用前需先執行I2C1_Master_Init()和MPU6050_Init()進行初始化
若同時使用my_MPU6050.c和my_LCD1620.c，I2C1_Master_Init()只需執行一次

 */


volatile float temperature = 0;
volatile int16_t raw_temp = 0;

// ===== MPU6050 Function =====
// --- MPU6050 寫入 ---
void MPU6050_Write(uint8_t reg_addr, uint8_t data) {
    I2C1_TX_Buf_Write(0, reg_addr);
    I2C1_TX_Buf_Write(1, data);

    I2C1_Master_Write(MPU6050_DevADDR, i2c1_tx_buf, 2);
    delay_ms(100);
}

// --- MPU6050 讀取 ---
void MPU6050_Read(uint8_t reg_addr, int len) {
    I2C1_TX_Buf_Write(0, reg_addr);

    I2C1_Master_Read(MPU6050_DevADDR, i2c1_tx_buf, i2c1_rx_buf, 1, len);
    delay_ms(20);
}

// --- MPU6050 初始化 ---
void MPU6050_Init(void) {
    MPU6050_Write(0x6B, 0x00);
    delay_ms(400);
}

// --- MPU6050 讀取溫度 ---
float MPU6050_ReadTemperature(void) {
    MPU6050_Read(TEMP_RegADDR, 2);
    raw_temp = ((int16_t)I2C1_RX_Buf_Read(0) << 8) | I2C1_RX_Buf_Read(1);
    temperature = raw_temp / 340.0f + 36.53f;

    return temperature;
}
