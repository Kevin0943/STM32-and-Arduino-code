#ifndef MY_LCD1602_H
#define MY_LCD1602_H


#include <stdint.h>


// ===== LCD1602 I2C Slave address =====
#define LCD6050_DevADDR  0x27

// ===== LCD1602 Function =====
void LCD1602_Send_Cmd(char data);
void LCD1602_Send_Data(char data);
void LCD1602_Clear(void);
void LCD1602_Init(void);
void LCD1602_Send_String (char *str);
void LCD1602_Put_Cursor(uint8_t row, uint8_t col);


#endif // MY_LCD1602_H
