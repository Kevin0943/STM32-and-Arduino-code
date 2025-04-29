#include <stdint.h>
#include "my_library.h"
#include "my_SPI1.h"
#include "my_SG90.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void) {
    systick_init(); // SysTick初始化
    blinkblink_init(); // 設定板子上的自帶的燈炮，用以閃燈
    SPI1_Slave_Init(); // SPI1 Slave模式初始化

    unsigned int angle = 90; // 用以紀錄SG90角度，初始值為90度
    SG90_Init(angle); // SG90初始化，初始角度設為angle

    union {
        float f;
        uint8_t bytes[4];
    } T; // 用以紀錄溫度

    delay_ms(1000);
    blinkblink(5, 100); // 閃燈5次

    SPI1_TX_Buf_Write(0, '0'); // spi1_tx_buf[0]寫入為'0'，每次SPI通訊後雙方以收到的第一個資料作為故障代碼，'0'表示一切正常


    while (1) {
        if(spi1_rx_dataLen) { // 若SPI Master有發起過通訊且已結束
            blinkblink(1, 50); // 快速閃燈一次

            // 將SPI Master傳來的溫度資料從spi1_rx_buf[1:4]讀取出來，並寫入float T
            T.bytes[0] = SPI1_RX_Buf_Read(1);
            T.bytes[1] = SPI1_RX_Buf_Read(2);
            T.bytes[2] = SPI1_RX_Buf_Read(3);
            T.bytes[3] = SPI1_RX_Buf_Read(4);

            if(SPI1_RX_Buf_Read(0) == '0') { // 若SPI Master故障代碼為'0'，表示一切正常
                // 計算SG90角度並設置，(角度0 ~ 180) 對應 (溫度20 ~ 30)
                T.f = (float)((int)(T.f * 10)) / 10; // 小數點後只保留第一位
                if(T.f < 20.0f) { T.f = 20.0f;} // 限制溫度範圍
                else if(T.f > 30.0f) { T.f = 30.0f;}
                angle = (unsigned int)(((T.f - 20.0f) / 10.0f) * 180.0f);
                SG90_ChangeAngle(angle); // 改變SG90角度
            }

            spi1_rx_dataLen = 0;
        }
    }
}
