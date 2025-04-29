#ifndef MY_UART1_H
#define MY_UART1_H


#include <stdint.h>


// ===== UART1 Buffer Size =====
#define UART1_BUF_SIZE 128

// ===== UART1 RX Buffer =====
extern volatile uint8_t uart1_rx_buf[UART1_BUF_SIZE];
extern volatile int  uart1_rx_dataLen;

// ===== UART1 TX Buffer =====
extern volatile uint8_t uart1_tx_buf[UART1_BUF_SIZE];

// ===== UART1 Function =====
void UART1_Init(uint32_t brr);
void UART1_TX_Buf_Write(int idx, uint8_t val);
void UART1_Send_Data(const volatile uint8_t *tx_buf, int len);
void UART1_Send_String(const char *s);


#endif // MY_UART1_H
