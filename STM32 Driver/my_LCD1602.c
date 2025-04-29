#include "my_library.h"
#include "my_I2C1.h"
#include "my_LCD1602.h"


/**
===== 簡述 =====
LCD1602 Library
需搭配my_I2C1.c使用


===== 使用說明 =====
使用前需先執行I2C1_Master_Init()和LCD1602_Init()進行初始化
若同時使用my_MPU6050.c和my_LCD1620.c，I2C1_Master_Init()只需執行一次

 */


// ===== LCD1602 Function =====
// --- LCD1602 傳送指令 ---
void LCD1602_Send_Cmd(char data) {
	uint8_t data_h = data & 0xF0;
	uint8_t data_l = (data << 4) & 0xF0;

	I2C1_TX_Buf_Write(0, data_h | 0x0C);
	I2C1_TX_Buf_Write(1, data_h | 0x08);
	I2C1_TX_Buf_Write(2, data_l | 0x0C);
	I2C1_TX_Buf_Write(3, data_l | 0x08);

	I2C1_Master_Write(LCD6050_DevADDR, i2c1_tx_buf, 4);
	delay_ms(1);
}

// --- LCD1602 傳送資料 ---
void LCD1602_Send_Data(char data) {
	uint8_t data_h = data & 0xF0;
	uint8_t data_l = (data << 4) & 0xF0;

	I2C1_TX_Buf_Write(0, data_h | 0x0D);
	I2C1_TX_Buf_Write(1, data_h | 0x09);
	I2C1_TX_Buf_Write(2, data_l | 0x0D);
	I2C1_TX_Buf_Write(3, data_l | 0x09);

	I2C1_Master_Write(LCD6050_DevADDR, i2c1_tx_buf, 4);
	delay_ms(1);
}

// --- LCD1602 清除螢幕 ---
void LCD1602_Clear(void) {
    LCD1602_Send_Cmd(0x01);
    delay_ms(1);
}

// --- LCD1602 初始化 ---
void LCD1602_Init(void) {
	delay_ms(1000);

    LCD1602_Send_Cmd(0x30);
    delay_ms(5);
    LCD1602_Send_Cmd(0x30);
    delay_ms(1);
    LCD1602_Send_Cmd(0x30);
    delay_ms(1);
    LCD1602_Send_Cmd(0x20);
    delay_ms(1);

    LCD1602_Send_Cmd(0x28);
    delay_ms(1);
    LCD1602_Send_Cmd(0x08);
    delay_ms(1);
    LCD1602_Send_Cmd(0x01);
    delay_ms(1);
    LCD1602_Send_Cmd(0x06);
    delay_ms(1);
    LCD1602_Send_Cmd(0x0C);
    delay_ms(1);

    delay_ms(500);

    LCD1602_Clear();
    delay_ms(300);
}

// --- LCD1602 顯示字串 ---
void LCD1602_Send_String (char *str) {
    while(*str) {
        LCD1602_Send_Data(*str++);
    }
    delay_ms(1);
}

// --- LCD1602 設定游標位置 ---
void LCD1602_Put_Cursor(uint8_t row, uint8_t col) {
    LCD1602_Send_Cmd(0x80 | (col + (0x40 * row)));
}
