#include "my_library.h"
#include "my_UART1.h"


/**
===== 簡述 =====
Simple hardware-based UART driver
點對點全雙工通訊，可同時收發不定長資料互不干擾
使用DMA和中斷實現非阻塞式通訊

適用型號：STM32F103C8T6
使用USART1 Peripheral


===== Pin 配置 =====
TX: PA9
RX: PA10


===== 使用說明 =====
使用前需先執行UART1_Init()初始化
不可直接寫入uart1_tx_buf，需使用UART1_TX_Buf_Write()

傳送：
使用UART1_TX_Buf_Write()將欲傳送資料寫入uart1_tx_buf，再使用UART1_Send_Data()傳送
或使用UART1_Send_String()直接傳送字串

接收：
資料傳入時DMA自動將其搬運至uart1_rx_buf，而後uart1_rx_dataLen會被設為所接收資料的長度，檢查uart1_rx_dataLen值以確認是否有資料傳入
uart1_rx_dataLen需手動重設為0


===== Function 說明 =====
void UART1_Init(uint32_t brr):
UART1初始化，輸入欲使用的波特率

void UART1_TX_Buf_Write(int idx, uint8_t val):
uart1_tx_buf寫入保護，避免在通訊進行中時修改uart1_tx_buf造成通訊錯誤
uart1_tx_buf[idx] = val

void UART1_Send_Data(const volatile uint8_t *tx_buf, int len):
傳送tx_buf[0:len]存放的資料

void UART1_Send_String(const char *s):
傳送字串s (結尾需有結束字元'\0')

void USART1_IRQHandler(void):
USART1中斷處理函式，在接收完數據後閒置一段時間便會觸發IDLE中斷
將uart1_rx_dataLen設為所接收資料的長度

 */


// ===== UART1 RX Buffer =====
volatile uint8_t uart1_rx_buf[UART1_BUF_SIZE]; // 資料接收Buffer
volatile int  uart1_rx_dataLen = 0; // 紀錄接收到多長的資料

// ===== UART1 TX Buffer =====
volatile uint8_t uart1_tx_buf[UART1_BUF_SIZE]; // 資料傳送Buffer

// ===== UART1 Function =====
// --- UART1 初始化 ---
void UART1_Init(uint32_t brr){ // UART1初始化，輸入欲使用的波特率
    // 啟用APB2相關Clock
    RCC_APB2ENR |= (1 << 2); // 啟用GPIOA Clock
    RCC_APB2ENR |= (1 << 14); // 啟用USART1 Clock

    // 啟用AHB相關Clock
    RCC_AHBENR  |= (1 << 0); // 啟用DMA1 Clock

    // 設PA9為Alternate function output Push-pull輸出(USART1 TX)
    GPIOA_CRH &= ~(0xF << 4); // PA9清除設定
    GPIOA_CRH |=  (0xB << 4); // CNF=10，MODE=11

    // 設PA10為Floating input(USART1 RX)
    GPIOA_CRH &= ~(0xF << 8);  // PA10清除設定
    GPIOA_CRH |=  (0x4 << 8);  // CNF=01，MODE=00

    // 設定USART1波特率
    float usart_div = 8000000.0f / (16.0f * (float)brr);
    uint32_t usart_div_mantissa = (uint32_t)usart_div;
    uint32_t usart_div_fraction = (uint32_t)((usart_div - usart_div_mantissa) * 16.0f);
    USART1_BRR = (usart_div_mantissa << 4) | (usart_div_fraction & 0xF);

    // 啟用USART1相關功能
    USART1_CR1 = 0; // USART1_CR1清空設定
    USART1_CR1 |= (1 << 2);  // 啟用RX
    USART1_CR1 |= (1 << 3);  // 啟用TX
    USART1_CR1 |= (1 << 4); // 啟用IDLE中斷

    // 啟用USART1的DMA相關功能
    USART1_CR3 = 0; // USART1_CR3清空設定
    USART1_CR3 |= (1 << 6); // 啟用USART1的DMA接收功能
    USART1_CR3 |= (1 << 7); // 啟用USART1的DMA傳輸功能

    // 啟用USART1
    USART1_CR1 |= (1 << 13);

    // 啟用DMA1_Channel4相關參數(USART1 TX)
    DMA1_CCR4 = 0; // DMA1_CCR4清空設定
    DMA1_CCR4 &= ~1; // 設定前先禁用DMA1_Channel4，要搬運欲傳輸資料時才啟用
    DMA1_CPAR4 = (uint32_t)&USART1_DR; // 設定周邊暫存器位址
    DMA1_CCR4 |= (1 << 4); // 設定搬運方向: 記憶體 >> 周邊
    DMA1_CCR4 |= (1 << 7); // 傳送後記憶體位址遞增

    // 啟用DMA1_Channel5相關參數(USART1 RX)
    DMA1_CCR5 = 0; // DMA1_CCR5清空設定
    DMA1_CCR5 &= ~1; // 設定前先禁用DMA1_Channel5
    DMA1_CPAR5 = (uint32_t)&USART1_DR; // 設定周邊暫存器位址
    DMA1_CMAR5 = (uint32_t)uart1_rx_buf; // 設定記憶體位址
    DMA1_CNDTR5 = UART1_BUF_SIZE; // 設定搬運資料長度
    DMA1_CCR5 &= ~(1 << 4); // 設定搬運方向: 周邊 >> 記憶體
    DMA1_CCR5 |= (1 << 7); // 傳送後記憶體位址遞增
    DMA1_CCR5 |= 1; // 啟用DMA1_Channel5

    // 設定中斷向量
    NVIC_ISER1 |= (1 << (37 - 32)); // 設定USART1中斷向量(IRQ 37)
}

// --- UART1 寫入uart1_tx_buf ---
void UART1_TX_Buf_Write(int idx, uint8_t val) {
    while (DMA1_CNDTR4); // 等待上一批的資料搬運完成，不需要等待TXE = 1 (Data register 空)
    uart1_tx_buf[idx] = val;
}

// --- UART1 傳送多個byte ---
void UART1_Send_Data(const volatile uint8_t *tx_buf, int len) {
    if(len > UART1_BUF_SIZE){ return;} // 傳送長度違法

    while (DMA1_CNDTR4); // 等待上一批的資料搬運完成
    // 等待TXE = 1 (Data register 空) (DMA1_CNDTR4為0只代表DMA已把最後的數據搬到USART1_DR中，但不代表USART1_DR中的數據已被USART1硬體送出去)
    while (!(USART1_SR & (1 << 7))); // Bit 7: TXE

    if (tx_buf != uart1_tx_buf){ // 若傳入的tx_buf不是uart1_tx_buf，則把tx_buf的內容複製到uart1_tx_buf
        for(int i = 0; i < len; i++){
            uart1_tx_buf[i] = tx_buf[i];
        }
    }

    DMA1_CCR4 &= ~1; // 設定前先禁用DMA1_Channel4
    DMA1_CMAR4 = (uint32_t)uart1_tx_buf; // 設定記憶體位址
    DMA1_CNDTR4 = len; // 設定搬運資料長度
    DMA1_CCR4 |= 1; // 啟用DMA1_Channel4，啟用後開始自動搬運直到搬完
}

// --- UART1 傳送string ---
void UART1_Send_String(const char *s) {
    while (DMA1_CNDTR4); // 等待上一批的資料搬運完成
    // 等待TXE = 1 (Data register 空) (DMA1_CNDTR4為0只代表DMA已把最後的數據搬到USART1_DR中，但不代表USART1_DR中的數據已被USART1硬體送出去)
    while (!(USART1_SR & (1 << 7))); // Bit 7: TXE

    int len = 0;
    while (s[len] && (len < UART1_BUF_SIZE)) { // 計算string長度，最多可傳送UART1_BUF_SIZE個字元
        uart1_tx_buf[len] = s[len];
        len++;
    }

    DMA1_CCR4 &= ~1; // 設定前先禁用DMA1_Channel4
    DMA1_CMAR4 = (uint32_t)uart1_tx_buf; // 設定記憶體位址
    DMA1_CNDTR4 = len; // 設定搬運資料長度
    DMA1_CCR4 |= 1; // 啟用DMA1_Channel4，啟用後開始自動搬運直到搬完
}

// --- UART1 中斷處理函式 --- // USART1在接收完數據後閒置一段時間便會觸發IDLE中斷
void USART1_IRQHandler(void) {
    if (USART1_SR & (1 << 4)) { // IDLE flag (確認是IDLE中斷)
        (void)USART1_SR; // 先讀SR
        (void)USART1_DR; // 再讀DR以清除IDLE flag
        uart1_rx_dataLen = UART1_BUF_SIZE - DMA1_CNDTR5; // 接收資料的長度 = uart1_rx_buf長度 - DMA1_Channel5剩餘可搬運長度

        DMA1_CCR5 &= ~1; // 設定前先禁用DMA1_Channel5
        DMA1_CMAR5 = (uint32_t)uart1_rx_buf; // 設定記憶體位址
        DMA1_CNDTR5 = UART1_BUF_SIZE; // 設定最大搬運資料長度
        DMA1_CCR5 |= 1; // 啟用DMA1_Channel5，啟用後一旦料傳入便會自動搬運，最大可搬運次數:UART1_BUF_SIZE (下次觸發IDLE中斷時刷新剩餘可搬運次數)
    }
}
