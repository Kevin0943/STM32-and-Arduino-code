#include <stdint.h>
#include "my_library.h"
#include "my_UART1.h"
#include "my_SPI1.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void) {
    systick_init(); // SysTick初始化
    blinkblink_init(); // 設定板子上的自帶的燈炮，用以閃燈
    UART1_Init(9600); // UART1初始化，波特率9600
    SPI1_Master_Init(); // SPI1 Master模式初始化

    union {
        float f;
        uint8_t bytes[4];
    } T; // 用以紀錄溫度

    char T_string[20]; // 用以紀錄溫度(字串形式)

    UART1_Send_String("\n\nThermometer Master Ready");// 用UART傳開機確認訊息給PC

    delay_ms(1000);
    blinkblink(5, 100); // 閃燈5次

    SPI1_TX_Buf_Write(0, '0'); // spi1_tx_buf[0]寫入為'0'，每次SPI通訊後雙方以收到的第一個資料作為故障代碼，'0'表示一切正常
    uint8_t slave0_state, slave1_state; // 用以紀錄SPI Slave0、Slave1的故障代碼

    unsigned int TIM3_ms_time = 1000; // 用以紀錄Timer3計時時間，初始值為1000 ms
    TIM3_ms_Init(TIM3_ms_time); // Timer3初始化，計時時間初始值設為TIM3_ms_time
    TIM3_ms_Enable(); // Timer3啟用


    while (1) {
        if(uart1_rx_dataLen) { // 若PC傳來UART訊息
            TIM3_ms_time = string_to_uint(uart1_rx_buf, uart1_rx_dataLen); // 將接收到的字串轉換成unsigned int
            TIM3_ms_ChangeTime(TIM3_ms_time); // 改變Timer3計時時間

            // 回傳UART確認訊息給PC
            UART1_Send_String("\n\nSet timer time: ");
            UART1_Send_Data(uart1_rx_buf, uart1_rx_dataLen);
            UART1_Send_String(" ms");

            uart1_rx_dataLen = 0;
        }

        if(Timer3_ms_flag) { // 若Timer3計時時間到
            blinkblink(1, 50); // 快速閃燈一次

            SPI1_Master_TransmitReceive(0, spi1_tx_buf, spi1_rx_buf, 5); // 向SPI Slave0發起通訊，接收溫度資料
            slave0_state = SPI1_RX_Buf_Read(0); // 紀錄SPI Slave0的故障代碼

            if(slave0_state == '0') { // 若SPI Slave0故障代碼為'0'，表示一切正常
                // 將SPI Slave0傳來的溫度資料從spi1_rx_buf[1:4]讀取出來，並寫入float T
                T.bytes[0] = SPI1_RX_Buf_Read(1);
                T.bytes[1] = SPI1_RX_Buf_Read(2);
                T.bytes[2] = SPI1_RX_Buf_Read(3);
                T.bytes[3] = SPI1_RX_Buf_Read(4);

                // 將溫度資料float T轉換成字串T_string，並用UART傳給PC
                float_to_string(T.f, T_string, 2, 2);
                UART1_Send_String("\n\nTemperature: ");
                UART1_Send_String(T_string);

                // 將溫度資料float T寫入spi1_tx_buf[1:4]，準備傳送給SPI Slave1
                SPI1_TX_Buf_Write(1, T.bytes[0]);
                SPI1_TX_Buf_Write(2, T.bytes[1]);
                SPI1_TX_Buf_Write(3, T.bytes[2]);
                SPI1_TX_Buf_Write(4, T.bytes[3]);
            }
            else if(slave0_state == '1') { // 若SPI Slave0故障代碼為'1'，表示Slave0的MPU6050發生故障
                UART1_Send_String("\n\nSlave0: MPU6050 Error"); // 用UART傳故障訊息給PC
            }
            else if(slave0_state == '2') { // 若SPI Slave0故障代碼為'2'，表示Slave0的LCD1602發生故障
                UART1_Send_String("\n\nSlave0: LCD1602 Error"); // 用UART傳故障訊息給PC
            }
            else { // 若SPI Slave0故障代碼為其他任何值，表示Slave0本身故障
                UART1_Send_String("\n\nSlave0 Error"); // 用UART傳故障訊息給PC
            }


            SPI1_Master_TransmitReceive(1, spi1_tx_buf, spi1_rx_buf, 5); // 向SPI Slave1發起通訊，傳送溫度資料
            slave1_state = SPI1_RX_Buf_Read(0); // 紀錄SPI Slave1的故障代碼

            if(slave1_state != '0') { // 若SPI Slave1故障代碼不為'0'，表示Slave1故障
                // 用UART傳故障訊息給PC
                UART1_Send_String("\nSlave1 Error");
            }

            Timer3_ms_flag = 0;
        }
    }
}
