#include "my_library.h"
#include "my_I2C1.h"


/**
===== 簡述 =====
Simple hardware-based I2C driver
使用DMA和中斷實現非阻塞式通訊
支援Master模式，可連接多個Slave
支援故障偵測

適用型號：STM32F103C8T6
使用I2C1 Peripheral


===== Pin 配置 =====
SCL: PB6
SDA: PB7


===== 使用說明 =====
使用前需先執行I2C1_Master_Init()初始化
不可直接寫入i2c1_tx_buf，需使用I2C1_TX_Buf_Write()
不可直接讀取i2c1_rx_buf，需使用I2C1_RX_Buf_Read()

寫入：
使用I2C1_TX_Buf_Write()將欲傳送數據(register address和data)寫入i2c1_tx_buf
再使用I2C1_Master_Write()向指定Slave發送寫入訊號

讀取：
使用I2C1_TX_Buf_Write()將register address寫入i2c1_tx_buf
再使用I2C1_Master_Read()向指定Slave發送讀取訊號
再使用I2C1_RX_Buf_Read()取得讀取到的資料


===== Function 說明 =====
void I2C1_Master_Init(void):
I2C1 Master模式初始化

void I2C1_TX_Buf_Write(int idx, uint8_t val):
i2c1_tx_buf寫入保護，避免在通訊進行中時修改i2c1_tx_buf造成通訊錯誤
i2c1_tx_buf[idx] = val

uint8_t I2C1_RX_Buf_Read(int idx):
i2c1_rx_buf讀取保護，避免通訊還未結束就讀取i2c1_rx_buf造成讀取到錯誤資料
return i2c1_rx_buf[idx]

void I2C1_Master_Start(void):
產生start bit，開始通訊
start bit成功產生時觸發Event中斷以接續下一步

void I2C1_Master_Send_SlaveAddress(void):
傳送slave address + R/W bit
slave address成功傳送且收到ACK回應時觸發Event中斷以接續下一步

void I2C1_Master_Send_Data(void):
傳送數據，包含register address和data
數據傳送完畢時觸發Event中斷以接續下一步

void I2C1_Master_Receive_Data(void):
接收數據
接收到指定長度資料後觸發DMA接收通道搬運完成中斷以接續下一步

void I2C1_Master_Stop(void):
產生stop bit，結束通訊

void I2C1_Master_Write(uint8_t slave_Address, const volatile uint8_t *tx_buf, int txLen):
向位址slave_Address的設備發送完整I2C寫入訊號
tx_buf[0:txLen]存放欲傳送數據，包含register address和data
會執行副程式I2C1_Master_Start()開始通訊，後續步驟皆由中斷觸發

void I2C1_Master_Read(uint8_t slave_Address, const volatile uint8_t *tx_buf, const volatile uint8_t *rx_buf, int txLen, int rxLen):
向位址slave_Address的設備發送完整I2C讀取訊號
tx_buf[0:txLen]存放register address
讀取長度rxLen的資料存放至i2c1_rx_buf[0:rxLen]
會執行副程式I2C1_Master_Start()開始通訊，後續步驟皆由中斷觸發

void I2C1_EV_IRQHandler(void):
I2C1 Event中斷處理函式，I2C通訊各階段結束後觸發此中斷以接續下一步(除了接收資料階段)

void DMA1_Channel7_IRQHandler(void):
DMA接收通道搬運完成中斷處理函式，I2C接收資料階段接收到指定長度資料後觸發此中斷以接續下一步

void I2C1_ER_IRQHandler(void):
I2C1 Error中斷處理函式，發生錯誤時觸發此中斷以進行處理(例如Slave未正常回傳ACK)

 */


volatile int rw_state = 0; // 紀錄當前模式為寫入(0) or 讀取(1)
volatile int reStarted = 0; // 紀錄是否已產生過restart bit
volatile int i2c1_error = 0; // Error flag，紀錄是否發生錯誤
volatile uint8_t slaveAddress; // 紀錄當前通訊的Slave的設備位址
volatile int i2c1_tx_dataLen = 0; // 紀錄欲傳送數據的長度，包含register address和data
volatile int i2c1_rx_dataLen = 0; // 紀錄欲接收資料的長度

// ===== I2C1 RX Buffer =====
volatile uint8_t i2c1_rx_buf[I2C1_BUF_SIZE]; // 資料接收Buffer

// ===== I2C1 TX Buffer =====
volatile uint8_t i2c1_tx_buf[I2C1_BUF_SIZE]; // 資料傳送Buffer

// ===== I2C1 Function =====
// --- I2C1 Master 初始化 ---
void I2C1_Master_Init(void) {
    // 啟用APB2相關Clock
    RCC_APB2ENR |= (1 << 3); // 啟用GPIOB Clock

    // 啟用APB1相關Clock
    RCC_APB1ENR |= (1 << 21); // 啟用I2C1 Clock

    // 啟用AHB相關Clock
    RCC_AHBENR  |= (1 << 0); // 啟用DMA1 Clock

    // 設PB6為Alternate function output Open-drain(I2C SCL)
    GPIOB_CRL &= ~(0xF << 24); // PB6清除設定
    GPIOB_CRL |=  (0xF << 24); // CNF=11，MODE=11

    // 設PB7為Alternate function output Open-drain(I2C SDA)
    GPIOB_CRL &= ~(0xF << 28);  // PB7清除設定
    GPIOB_CRL |=  (0xF << 28);  // CNF=11，MODE=11

    // 拉高SCL(PB6)、SDA(PB7)
    GPIOB_ODR |= (1 << 6);
    GPIOB_ODR |= (1 << 7);

    // I2C1相關設定
    I2C1_CR1 = 0; // I2C1_CR1清空設定
    I2C1_CR2 = 0; // I2C1_CR2清空設定
    I2C1_CR1 &= ~1; // Disable I2C
    I2C1_CR2 |= 8;          // 設I2C Clock頻率為8MHz，此值必須和APB1 Clock頻率相同 (APB1 Clock頻率使用預設值8MHz)
    I2C1_CCR = 40;         // 設SCL Clock頻率為100kHz
    I2C1_TRISE = 9;        // 設SCL Clock為HIGH的最長時間(單位為APB1 Clock)
    I2C1_CR2 |= (1 << 9) | // ITEVTEN = 1，啟用Event中斷
                (1 << 8);  // ITERREN = 1，啟用Error中斷

    // I2C1的DMA相關設定
    I2C1_CR2 |= (1 << 11); // 啟用I2C1的DMA傳輸和接收功能
    I2C1_CR2 |= (1 << 12); // LAST = 1，DMA接收通道搬運完最後一個byte後自動送NACK，在搬運完之前則是送ACK

    // 啟用I2C1
    I2C1_CR1 |= 1;

    // 啟用DMA1_Channel6相關參數(I2C1 TX)
    DMA1_CCR6 = 0; // DMA1_CCR6清空設定
    DMA1_CCR6 &= ~1; // 設定前先禁用DMA1_Channel6
    DMA1_CPAR6 = (uint32_t)&I2C1_DR; // 設定周邊暫存器位址
    DMA1_CCR6 |= (1 << 4); // 設定搬運方向: 記憶體 >> 周邊
    DMA1_CCR6 |= (1 << 7); // 搬運後記憶體位址遞增

    // 啟用DMA1_Channel7相關參數(I2C1 RX)
    DMA1_CCR7 = 0; // DMA1_CCR7清空設定
    DMA1_CCR7 &= ~1; // 設定前先禁用DMA1_Channel7
    DMA1_CPAR7 = (uint32_t)&I2C1_DR; // 設定周邊暫存器位址
    DMA1_CCR7 |= (1 << 1); // 啟用搬運完成中斷
    DMA1_CCR7 &= ~(1 << 4); // 設定搬運方向: 周邊 >> 記憶體
    DMA1_CCR7 |= (1 << 7); // 搬運後記憶體位址遞增

    // 設定I2C1中斷向量
    NVIC_ISER0 |= (1 << 31); // 設定I2C1 Event中斷向量(IRQ 31)
    NVIC_ISER1 |= (1 << (32 - 32)); // 設定I2C1 Error中斷向量(IRQ 32)

    // 設定DMA中斷向量
    NVIC_ISER0 |= (1 << 17); // 設定DMA1_Channel7中斷向量(IRQ 17)
}

// --- I2C1 寫入i2c1_tx_buf ---
void I2C1_TX_Buf_Write(int idx, uint8_t val) {
    while (I2C1_SR2 & (1 << 1)); // 等待BUSY = 0 (等待先前的通訊結束)
    i2c1_tx_buf[idx] = val;
}

// --- I2C1 讀取i2c1_rx_buf ---
uint8_t I2C1_RX_Buf_Read(int idx) {
    while (I2C1_SR2 & (1 << 1)); // 等待BUSY = 0 (等待先前的通訊結束)
    return i2c1_rx_buf[idx];
}

// --- I2C1 產生start bit ---
void I2C1_Master_Start(void) {
    // 產生start bit
    I2C1_CR1 |= (1 << 8); // START

    // 等待SB = 1時觸發Event中斷以接續下一步
}

// --- I2C1 傳送slave address + R/W bit ---
void I2C1_Master_Send_SlaveAddress(void) {
    if(reStarted == 0) {
        // 傳送slave address，同時清除SB：先讀取SR1，再寫入DR
        (void)I2C1_SR1; // 先讀取SR1
        I2C1_DR = slaveAddress << 1; // 再寫入DR，I2C1_DR = 7bit slave address + R/W bit
    }
    else {
        // 傳送slave address，同時清除SB：先讀取SR1，再寫入DR
        (void)I2C1_SR1; // 先讀取SR1
        I2C1_DR = (slaveAddress << 1) | 1; // 再寫入DR，I2C1_DR = 7bit slave address + R/W bit
    }

    // 等待ADDR = 1時觸發Event中斷以接續下一步
}

// --- I2C1 傳送多個byte --- // 用於位在slave address之後的多個byte，包含register address和data
void I2C1_Master_Send_Data() {
    DMA1_CCR6 &= ~1; // 設定前先禁用DMA1_Channel6
    DMA1_CMAR6 = (uint32_t)i2c1_tx_buf; // 設定記憶體位址
    DMA1_CNDTR6 = i2c1_tx_dataLen; // 設定搬運資料長度
    DMA1_CCR6 |= 1; // 啟用DMA1_Channel6，啟用後開始自動搬運直到搬完

    // 清除ADDR：先讀取SR1，再讀取SR2
    (void)I2C1_SR1;
    (void)I2C1_SR2;

    // 等待BTF = 1時觸發Event中斷以接續下一步
}

// --- I2C1 接收多個byte ---
void I2C1_Master_Receive_Data() {
    DMA1_CCR7 &= ~1; // 設定前先禁用DMA1_Channel7
    DMA1_CMAR7 = (uint32_t)i2c1_rx_buf; // 設定記憶體位址
    DMA1_CNDTR7 = i2c1_rx_dataLen; // 設定搬運資料長度
    DMA1_CCR7 |= 1; // 啟用DMA1_Channel7，啟用後開始自動搬運直到搬完

    if(i2c1_rx_dataLen <= 1) {
        I2C1_CR1 &= ~(1 << 10); // 下次接收完資料回傳NACK，若沒這行將會回傳ACK代表叫Slave繼續傳資料
    }
    else {
        I2C1_CR1 |= (1 << 10); // 下次接收完資料回傳ACK，DMA搬運到最後一個數據時自動改為回傳NACK
    }

    // 清除ADDR：先讀取SR1，再讀取SR2
    (void)I2C1_SR1;
    (void)I2C1_SR2;

    // 等待DMA接收通道搬運完成中斷觸發以接續下一步
}

// --- I2C1 產生stop bit ---
void I2C1_Master_Stop(void) {
    // 產生stop bit
    I2C1_CR1 |= (1 << 9); // STOP
}

// --- I2C1 Write ---
void I2C1_Master_Write(uint8_t slave_Address, const volatile uint8_t *tx_buf, int txLen) {
    if(txLen > I2C1_BUF_SIZE){ return;} // 傳送長度違法

    i2c1_error = 0; // 刷新Error flag

    while (I2C1_SR2 & (1 << 1)); // 等待BUSY = 0 (等待先前的通訊結束)
    rw_state = 0; // 紀錄當前模式為寫入
    reStarted = 0; // 刷新reStarted
    slaveAddress = slave_Address; // 紀錄當前通訊的Slave的設備位址
    i2c1_tx_dataLen = txLen; // 紀錄欲傳送數據的長度，包含register address和data

    if (tx_buf != i2c1_tx_buf){ // 若傳入的tx_buf不是i2c1_tx_buf，則把tx_buf的內容複製到i2c1_tx_buf
        for(int i = 0; i < i2c1_tx_dataLen; i++){
            i2c1_tx_buf[i] = tx_buf[i];
        }
    }

    // 產生start bit
    I2C1_Master_Start();

    // 後續步驟皆由中斷觸發
}

// --- I2C1 Read ---
void I2C1_Master_Read(uint8_t slave_Address, const volatile uint8_t *tx_buf, const volatile uint8_t *rx_buf, int txLen, int rxLen) {
    if(txLen > I2C1_BUF_SIZE){ return;} // 傳送長度違法
    if(rxLen > I2C1_BUF_SIZE){ return;} // 接收長度違法

    i2c1_error = 0; // 刷新Error flag

    while (I2C1_SR2 & (1 << 1)); // 等待BUSY = 0 (等待先前的通訊結束)
    rw_state = 1; // 紀錄當前模式為讀取
    reStarted = 0; // 刷新reStarted
    slaveAddress = slave_Address; // 紀錄當前通訊的Slave的設備位址
    i2c1_tx_dataLen = txLen; // 紀錄欲傳送數據的長度，只包含register address
    i2c1_rx_dataLen = rxLen; // 紀錄欲接收數據的長度

    if (tx_buf != i2c1_tx_buf){ // 若傳入的tx_buf不是i2c1_tx_buf，則把tx_buf的內容複製到i2c1_tx_buf
        for(int i = 0; i < i2c1_tx_dataLen; i++){
            i2c1_tx_buf[i] = tx_buf[i];
        }
    }

    // 產生start bit
    I2C1_Master_Start();

    // 後續步驟皆由中斷觸發
}

// --- I2C1 Event中斷處理函式 ---
void I2C1_EV_IRQHandler(void) {
    if (I2C1_SR1 & (1 << 0)) { // SB = 1，表示start bit成功產生
        I2C1_Master_Send_SlaveAddress();
    }
    else if (I2C1_SR1 & (1 << 1)) { // ADDR = 1，表示slave address成功傳送且收到ACK回應
        if(reStarted == 0) { // 未產生過restart bit
            I2C1_Master_Send_Data();
        }
        else { // 已產生過restart bit
            I2C1_Master_Receive_Data();
        }
    }
    else if (I2C1_SR1 & (1 << 2)) { // BTF = 1，表示數據傳送完畢
        // 清除BTF：先讀取SR1，再讀取DR
        (void)I2C1_SR1;
        (void)I2C1_DR;
        if(rw_state == 0) { // 寫入模式
            I2C1_Master_Stop();
        }
        else { // 讀取模式
            if(reStarted == 0) { // 未產生過restart bit
                reStarted = 1;
                I2C1_Master_Start();
            }
        }
    }
}

// --- DMA1_Channel7 中斷處理函式(I2C1 RX) ---
void DMA1_Channel7_IRQHandler(void) {
    if (DMA1_ISR & (1 << 25)) { // DMA1_Channel7搬運完成flag
        DMA1_IFCR |= (1 << 25); // 清除DMA1_Channel7搬運完成flag，避免此中斷一直連續觸發
        DMA1_CCR7 &= ~1; // 禁用DMA1_Channel7
        I2C1_Master_Stop();
    }
}

// --- I2C1 Error中斷處理函式 ---
void I2C1_ER_IRQHandler(void) {
    i2c1_error = 1; // 紀錄發生錯誤

    I2C1_SR1 &= ~( (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 14) | (1 << 15) ); // 清除flag，避免此中斷一直連續觸發
    I2C1_Master_Stop(); // 產生stop bit提早結束通訊
}
