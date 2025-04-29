#ifndef MY_MPU6050_H
#define MY_MPU6050_H


#include <stdint.h>


// ===== MPU6050 I2C Slave address =====
#define MPU6050_DevADDR  0x68

// ===== MPU6050 Register address =====
#define TEMP_RegADDR  0x41

volatile float temperature;
volatile int16_t raw_temp;

// ===== MPU6050 Function =====
void MPU6050_Write(uint8_t reg_addr, uint8_t data);
void MPU6050_Read(uint8_t reg_addr, int len);
void MPU6050_Init(void);
float MPU6050_ReadTemperature(void);


#endif // MY_MPU6050_H
