#ifndef MY_SPI1_H
#define MY_SPI1_H


#include <stdint.h>


volatile int isMaster;

// ===== SPI1 Buffer Size =====
#define SPI1_BUF_SIZE 128

// ===== SPI1 RX Buffer =====
extern volatile uint8_t spi1_rx_buf[SPI1_BUF_SIZE];
extern volatile int  spi1_rx_dataLen;

// ===== SPI1 TX Buffer =====
extern volatile uint8_t spi1_tx_buf[SPI1_BUF_SIZE];

// ===== SPI1 RX Flag =====
volatile uint8_t spi1_rx_done;

// ===== SPI1 TX Flag =====
volatile uint8_t spi1_tx_done;

// ===== SPI1 Function =====
void SPI1_Master_Init(void);
void SPI1_Slave_Init(void);
void SPI1_TX_Buf_Write(int idx, uint8_t val);
uint8_t SPI1_RX_Buf_Read(int idx);
void SPI1_Master_TransmitReceive(int SlaveNumber, const volatile uint8_t *tx_buf, const volatile uint8_t *rx_buf, int len);


#endif // MY_SPI1_H
