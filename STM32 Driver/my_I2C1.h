#ifndef MY_I2C1_H
#define MY_I2C1_H


#include <stdint.h>


volatile int rw_state;
volatile int reStarted;
extern volatile int i2c1_error;
volatile uint8_t slaveAddress;
volatile int i2c1_tx_dataLen;
volatile int i2c1_rx_dataLen;

// ===== I2C1 Buffer Size =====
#define I2C1_BUF_SIZE 128

// ===== I2C1 RX Buffer =====
extern volatile uint8_t i2c1_rx_buf[I2C1_BUF_SIZE];

// ===== I2C1 TX Buffer =====
extern volatile uint8_t i2c1_tx_buf[I2C1_BUF_SIZE];

// ===== I2C1 Function =====
void I2C1_Master_Init(void);
void I2C1_TX_Buf_Write(int idx, uint8_t val);
uint8_t I2C1_RX_Buf_Read(int idx);
void I2C1_Master_Start(void);
void I2C1_Master_Send_SlaveAddress(void);
void I2C1_Master_Send_Data(void);
void I2C1_Master_Receive_Data(void);
void I2C1_Master_Stop(void);
void I2C1_Master_Write(uint8_t slave_Address, const volatile uint8_t *tx_buf, int txLen);
void I2C1_Master_Read(uint8_t slave_Address, const volatile uint8_t *tx_buf, const volatile uint8_t *rx_buf, int txLen, int rxLen);


#endif // MY_I2C1_H
