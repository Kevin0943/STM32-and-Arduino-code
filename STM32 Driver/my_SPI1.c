#include "my_library.h"
#include "my_SPI1.h"


/**
===== 簡述 =====
Simple hardware-based SPI driver
一主二從全雙工通訊，每次通訊皆同時收發不定長資料
使用DMA和中斷實現非阻塞式通訊
支援Master模式(可連接2個Slave)、Slave模式
每次通訊由Master決定收發之資料長度，Slave可自動應對

適用型號：STM32F103C8T6
使用SPI1 Peripheral


===== Pin 配置 =====
Master模式：
NSS0: PA4
NSS1: PA3
SCK:  PA5
MISO: PA6
MOSI: PA7

Slave模式：
NSS:  PA4
SCK:  PA5
MISO: PA6
MOSI: PA7


===== 使用說明 =====
Master模式：
使用前需先執行SPI1_Master_Init()初始化
使用SPI1_TX_Buf_Write()將欲傳送資料寫入spi1_tx_buf
再使用SPI1_Master_TransmitReceive()向指定Slave發起通訊
再使用SPI1_RX_Buf_Read()讀取接收到的資料

Slave模式：
使用前需先執行SPI1_Slave_Init()初始化
使用SPI1_TX_Buf_Write()將欲傳送資料寫入spi1_tx_buf，當Master發起通訊時會被傳送出去
Master發起通訊時DMA自動處理資料收發，通訊結束後spi1_rx_dataLen會被設為所接收資料的長度，檢查spi1_rx_dataLen值以確認是否有通訊已發生
再使用SPI1_RX_Buf_Read()讀取接收到的資料
spi1_rx_dataLen需手動重設為0


===== Function 說明 =====
void SPI1_Master_Init(void):
SPI1 Master模式初始化

void SPI1_Slave_Init(void):
SPI1 Slave模式初始化

void SPI1_TX_Buf_Write(int idx, uint8_t val):
spi1_tx_buf寫入保護，避免在通訊進行中時修改spi1_tx_buf造成通訊錯誤
spi1_tx_buf[idx] = val

uint8_t SPI1_RX_Buf_Read(int idx):
spi1_rx_buf讀取保護，避免通訊還未結束就讀取spi1_rx_buf造成讀取到錯誤資料
return spi1_rx_buf[idx]

void SPI1_Master_TransmitReceive(int SlaveNumber, const volatile uint8_t *tx_buf, const volatile uint8_t *rx_buf, int len):
Master向編號SlaveNumber的Slave發起通訊(拉低其對應的NSS引腳)，同時收發長度len的資料
tx_buf[0:len]存放欲傳送資料
接收的資料被存放至spi1_rx_buf[0:len]

void DMA1_Channel3_IRQHandler(void): // Master模式才會啟用此中斷
DMA傳送通道搬運完成中斷處理函式，觸發時設置spi1_tx_done = 1
若spi1_tx_done和spi1_rx_done都為1，代表資料收發都已結束，Master拉高NSS引腳結束當前通訊

void DMA1_Channel2_IRQHandler(void): // Master模式才會啟用此中斷
DMA接收通道搬運完成中斷處理函式，觸發時設置spi1_rx_done = 1
若spi1_tx_done和spi1_rx_done都為1，代表資料收發都已結束，Master拉高NSS引腳結束當前通訊

void EXTI4_IRQHandler(void): // Slave模式才會啟用此中斷
NSS引腳被Master拉低或拉高時觸發以進行對應設置
若是通訊結束觸發，將spi1_rx_dataLen設為所接收資料的長度

 */


volatile int isMaster = 1; // 紀錄當前模式為Master or Slave

// ===== SPI1 RX Buffer =====
volatile uint8_t spi1_rx_buf[SPI1_BUF_SIZE]; // 資料接收Buffer
volatile int  spi1_rx_dataLen = 0; // Slave模式使用，當一次通訊結束後spi1_rx_dataLen會紀錄和Master交換了多少長度的資料

// ===== SPI1 TX Buffer =====
volatile uint8_t spi1_tx_buf[SPI1_BUF_SIZE]; // 資料傳送Buffer

// ===== SPI1 RX Flag =====
volatile uint8_t spi1_rx_done = 0; // 紀錄DMA接收通道(DMA1_Channel2)搬運是否完成

// ===== SPI1 TX Flag =====
volatile uint8_t spi1_tx_done = 0; // 紀錄DMA傳送通道(DMA1_Channel3)搬運是否完成

// ===== SPI1 Function =====
// --- SPI1 Master 初始化 ---
void SPI1_Master_Init(void) {
    isMaster = 1; // 紀錄當前為Master模式

    // 啟用 GPIOA 和 SPI1 時鐘
    RCC_APB2ENR |= (1 << 2) | (1 << 12); // RCC_APB2ENR: IOPAEN 和 SPI1EN

    // 啟用AHB相關Clock
    RCC_AHBENR  |= (1 << 0); // 啟用DMA1 Clock

    // 配置 GPIOA 引腳
    GPIOA_CRL &= ~((0xF << 16) |(0xF << 12) | (0xF << 20) | (0xF << 24) | (0xF << 28));
    GPIOA_CRL |=  (0x3 << 16) | // PA4 (NSS0) - General purpose output Push-pull, 50MHz
                  (0x3 << 12) | // PA3 (NSS1) - General purpose output Push-pull, 50MHz
                  (0xB << 20) | // PA5 (SCK)  - Alternate function output Push-pull, 50MHz
                  (0x8 << 24) | // PA6 (MISO) - Input with pull-up / pull-down
                  (0xB << 28);  // PA7 (MOSI) - Alternate function output Push-pull, 50MHz

    // 拉高 NSS0(PA4)、NSS1(PA3)
    GPIOA_ODR |= (1 << 4);
    GPIOA_ODR |= (1 << 3);

    // 將PA6(MISO)設定為Input with pull-down
    GPIOA_ODR &= ~(1 << 6);

    // 配置 SPI1
    SPI1_CR1 = 0; // SPI1_CR1清空設定
    SPI1_CR1 = (1 << 2)  | // MSTR: Master模式
               (7 << 3)  | // BR[2:0]: fPCLK/256 = 8MHz/256 = 31.25kHz，週期32微秒
               (1 << 9)  | // SSM: 軟體管理NSS，NSS Pin電位設定為手動控制
               (1 << 8)  | // SSI: 當SSM = 1時，SSI需設為1，避免MODF fault
               (0 << 7)  | // LSBFIRST: MSB 優先
               (0 << 11);  // DFF: 8 位元資料格式

    SPI1_CR1 &= ~(1 << 1); // CPOL = 0
    SPI1_CR1 |=  (1 << 0); // CPHA = 1  → Mode 1

    SPI1_CR2 = 0; // SPI1_CR2清空設定
    SPI1_CR2 |= (1 << 1) | (1 << 0); // TXDMAEN, RXDMAEN：啟用 DMA 傳輸

    // 啟用 SPI1
    SPI1_CR1 |= (1 << 6); // SPE: SPI 啟用

    // 啟用DMA1_Channel3相關參數(SPI1 TX)
    DMA1_CCR3 = 0; // DMA1_CCR3清空設定
    DMA1_CCR3 &= ~1; // 設定前先禁用DMA1_Channel3
    DMA1_CPAR3 = (uint32_t)&SPI1_DR; // 設定周邊暫存器位址
    DMA1_CCR3 |= (1 << 1); // 啟用搬運完成中斷
    DMA1_CCR3 |= (1 << 4); // 設定搬運方向: 記憶體 >> 周邊
    DMA1_CCR3 |= (1 << 7); // 搬運後記憶體位址遞增

    // 啟用DMA1_Channel2相關參數(SPI1 RX)
    DMA1_CCR2 = 0; // DMA1_CCR7清空設定
    DMA1_CCR2 &= ~1; // 設定前先禁用DMA1_Channel2
    DMA1_CPAR2 = (uint32_t)&SPI1_DR; // 設定周邊暫存器位址
    DMA1_CCR2 |= (1 << 1); // 啟用搬運完成中斷
    DMA1_CCR2 &= ~(1 << 4); // 設定搬運方向: 周邊 >> 記憶體
    DMA1_CCR2 |= (1 << 7); // 搬運後記憶體位址遞增

    // 設定DMA中斷向量
    NVIC_ISER0 |= (1 << 12); // 設定DMA1_Channel2中斷向量(IRQ 12)
    NVIC_ISER0 |= (1 << 13); // 設定DMA1_Channel3中斷向量(IRQ 13)
}

// --- SPI1 Slave 初始化 --
void SPI1_Slave_Init(void) {
    isMaster = 0; // 紀錄當前為Slave模式

    // 啟用 GPIOA 和 SPI1 時鐘
    RCC_APB2ENR |= (1 << 2) | (1 << 12) | (1 << 0); // RCC_APB2ENR: IOPAEN 和 SPI1EN 和 AFIOEN

    // 啟用AHB相關Clock
    RCC_AHBENR  |= (1 << 0); // 啟用DMA1 Clock

    // 配置 GPIOA 引腳
    GPIOA_CRL &= ~((0xF << 16) | (0xF << 20) | (0xF << 24) | (0xF << 28));
    GPIOA_CRL |=  (0x8 << 16) | // PA4 (NSS)  - Input with pull-up / pull-down
                  (0x4 << 20) | // PA5 (SCK)  - Floating input
                  (0x4 << 24) | // PA6 (MISO) - Floating input
                  (0x4 << 28);  // PA7 (MOSI) - Floating input

    // PA4 (NSS)設定為Input with pull-up
    GPIOA_ODR |= (1 << 4);

    // 配置 SPI1
    SPI1_CR1 = (0 << 2)  | // MSTR: Slave模式
               (0 << 9)  | // SSM: 硬體管理NSS，NSS被拉低SPI硬體才能啟用
               (0 << 7)  | // LSBFIRST: MSB 優先
               (0 << 11);  // DFF: 8 位元資料格式

    SPI1_CR1 &= ~(1 << 1); // CPOL = 0
    SPI1_CR1 |=  (1 << 0); // CPHA = 1  → Mode 1

    SPI1_CR2 = 0; // SPI1_CR2清空設定
    SPI1_CR2 |= (1 << 1) | (1 << 0); // TXDMAEN, RXDMAEN：啟用 DMA 傳輸

    // 啟用 SPI1
    SPI1_CR1 |= (1 << 6); // SPE: SPI 啟用


    // 啟用DMA1_Channel3相關參數(SPI1 TX)
    DMA1_CCR3 = 0; // DMA1_CCR3清空設定
    DMA1_CCR3 &= ~1; // 設定前先禁用DMA1_Channel3
    DMA1_CPAR3 = (uint32_t)&SPI1_DR; // 設定周邊暫存器位址
    DMA1_CMAR3 = (uint32_t)spi1_tx_buf; // 設定記憶體位址
    DMA1_CNDTR3 = SPI1_BUF_SIZE; // 設定搬運資料長度
    DMA1_CCR3 |= (1 << 4); // 設定搬運方向: 記憶體 >> 周邊
    DMA1_CCR3 |= (1 << 7); // 搬運後記憶體位址遞增
    DMA1_CCR3 |= 1; // 啟用DMA1_Channel3


    // 啟用DMA1_Channel2相關參數(SPI1 RX)
    DMA1_CCR2 = 0; // DMA1_CCR7清空設定
    DMA1_CCR2 &= ~1; // 設定前先禁用DMA1_Channel2
    DMA1_CPAR2 = (uint32_t)&SPI1_DR; // 設定周邊暫存器位址
    DMA1_CMAR2 = (uint32_t)spi1_rx_buf; // 設定記憶體位址
    DMA1_CNDTR2 = SPI1_BUF_SIZE; // 設定搬運資料長度
    DMA1_CCR2 &= ~(1 << 4); // 設定搬運方向: 周邊 >> 記憶體
    DMA1_CCR2 |= (1 << 7); // 搬運後記憶體位址遞增
    DMA1_CCR2 |= 1; // 啟用DMA1_Channel2

    //設定NSS外部中斷
    AFIO_EXTICR2 &= ~(0xF << 0); // Connect EXTI4 to PA4
    EXTI_IMR  |= (1 << 4); // 啟用 EXTI4

    EXTI_RTSR |= (1 << 4);
    EXTI_FTSR |= (1 << 4); // 雙邊沿觸發

    // 設定EXTI4中斷向量
    NVIC_ISER0 |= (1 << 10);
}

// --- SPI1 寫入spi1_tx_buf ---
void SPI1_TX_Buf_Write(int idx, uint8_t val) {
    if(isMaster) {
        // 等待NSS0(PA4)、NSS1(PA3)拉高，確保所有通訊都已結束
        while ( !(GPIOA_IDR & (1 << 4)) || !(GPIOA_IDR & (1 << 3)) );
    }
    else {
        // 等待NSS(PA4)拉高，確保通訊已結束
        while (!(GPIOA_IDR & (1 << 4)));
    }

    spi1_tx_buf[idx] = val;
}

// --- SPI1 讀取spi1_rx_buf ---
uint8_t SPI1_RX_Buf_Read(int idx) {
    if(isMaster) {
        // 等待NSS0(PA4)、NSS1(PA3)拉高，確保所有通訊都已結束
        while ( !(GPIOA_IDR & (1 << 4)) || !(GPIOA_IDR & (1 << 3)) );
    }
    else {
        // 等待NSS(PA4)拉高，確保通訊已結束
        while (!(GPIOA_IDR & (1 << 4)));
    }

    return spi1_rx_buf[idx];
}

// --- SPI1 Master 傳送and接收 --- // 由Master決定此次通訊交換多長資料(len)，Slave可自動應對
void SPI1_Master_TransmitReceive(int SlaveNumber, const volatile uint8_t *tx_buf, const volatile uint8_t *rx_buf, int len) {
    if(!isMaster) { return;} // 防止Slave模式誤用
    if(len > SPI1_BUF_SIZE){ return;} // 傳送長度違法

    // 等待NSS0(PA4)、NSS1(PA3)拉高，確保所有通訊都已結束
    while ( !(GPIOA_IDR & (1 << 4)) || !(GPIOA_IDR & (1 << 3)) );

    if (tx_buf != spi1_tx_buf){ // 若傳入的tx_buf不是spi1_tx_buf，則把tx_buf的內容複製到spi1_tx_buf
        for(int i = 0; i < len; i++){
            spi1_tx_buf[i] = tx_buf[i];
        }
    }

    if(len > 1) { spi1_tx_done = 0;}
    else if(len == 1) { spi1_tx_done = 1;} // 若len == 1，DMA1_Channel3的搬運資料長度將為0，會造成DMA1_Channel3不觸發搬運完成中斷，故手動設定spi1_tx_done = 1
    else { return;}
    spi1_rx_done = 0;

    SPI1_DR = spi1_tx_buf[0]; // 覆蓋上一次通訊殘留的未傳送的無用資料，避免此無用資料被誤傳 (每次通訊的資料長度若不足SPI1_BUF_SIZE便會在SPI1_DR殘留無用資料)
    DMA1_CCR3 &= ~1; // 設定前先禁用DMA1_Channel3
    DMA1_CMAR3 = (uint32_t)(spi1_tx_buf + 1); // 設定記憶體位址
    DMA1_CNDTR3 = len - 1; // 設定搬運資料長度
    DMA1_CCR3 |= 1; // 啟用DMA1_Channel3

    (void)SPI1_DR; // 清空可能存在的殘留資料避免誤讀
    DMA1_CCR2 &= ~1; // 設定前先禁用DMA1_Channel2
    DMA1_CMAR2 = (uint32_t)spi1_rx_buf; // 設定記憶體位址
    DMA1_CNDTR2 = len; // 設定搬運資料長度
    DMA1_CCR2 |= 1; // 啟用DMA1_Channel2

    if(SlaveNumber == 0) {
        // 控制NSS選擇Slave 0，開始和Slave 0 通訊
        GPIOA_ODR |= (1 << 3); // 拉高 NSS1(PA3)，確保Slave 1沒被選擇到
        GPIOA_ODR &= ~(1 << 4); // 拉低 NSS0(PA4)，選擇Slave 0
    }
    else if(SlaveNumber == 1) {
        // 控制NSS選擇Slave 1，開始和Slave 1 通訊
        GPIOA_ODR |= (1 << 4); // 拉高 NSS0(PA4)，確保Slave 0沒被選擇到
        GPIOA_ODR &= ~(1 << 3); // 拉低 NSS1(PA3)，選擇Slave 1
    }
}

// --- DMA1_Channel3 中斷處理函式(SPI1 TX) --- // Master模式才會啟用此中斷
void DMA1_Channel3_IRQHandler(void) {
    if (DMA1_ISR & (1 << 9)) { // DMA1_Channel3搬運完成flag
        DMA1_IFCR |= (1 << 9); // 清除DMA1_Channel3搬運完成flag，避免此中斷一直連續觸發

        spi1_tx_done = 1; // 紀錄DMA1_Channel3搬運完成
        if(spi1_tx_done && spi1_rx_done) { // 若DMA1_Channel3、DMA1_Channel2都搬運完成
            // 拉高 NSS0(PA4)、NSS1(PA3)
            GPIOA_ODR |= (1 << 4);
            GPIOA_ODR |= (1 << 3);
        }
    }
}

// --- DMA1_Channel2 中斷處理函式(SPI1 RX) --- // Master模式才會啟用此中斷
void DMA1_Channel2_IRQHandler(void) {
    if (DMA1_ISR & (1 << 5)) { // DMA1_Channel2搬運完成flag
        DMA1_IFCR |= (1 << 5); // 清除DMA1_Channel2搬運完成flag，避免此中斷一直連續觸發

        spi1_rx_done = 1; // 紀錄DMA1_Channel2搬運完成
        if(spi1_tx_done && spi1_rx_done) { // 若DMA1_Channel3、DMA1_Channel2都搬運完成
            // 拉高 NSS0(PA4)、NSS1(PA3)
            GPIOA_ODR |= (1 << 4);
            GPIOA_ODR |= (1 << 3);
        }
    }
}

// --- EXTI4外部中斷處理函式 --- // Slave模式才會啟用此中斷，NSS引腳(PA4)被Master拉低或拉高時觸發
void EXTI4_IRQHandler(void) {
    if (EXTI_PR & (1 << 4)) { // 確認是EXTI4外部中斷所觸發

        // 讀取NSS(PA4)電位，確認此次外部中斷觸發是因為被拉低還是拉高
        if(!(GPIOA_IDR & (1 << 4))) { // 若為拉低，代表開始通訊，則將MISO轉為輸出模式
            GPIOA_CRL &= ~(0xF << 24); // PA6 (MISO) - 清空
            GPIOA_CRL |=  (0xB << 24); // PA6 (MISO) - Alternate function output Push-pull, 50MHz
        }
        else { // 若為拉高，代表通訊結束，則將MISO轉為Floating input模式(高阻抗)，避免在MISO線路上和其餘Slave產生短路
            GPIOA_CRL &= ~(0xF << 24); // PA6 (MISO) - 清空
            GPIOA_CRL |=  (0x4 << 24); // PA6 (MISO) - Floating input

            spi1_rx_dataLen = SPI1_BUF_SIZE - DMA1_CNDTR2; // 接收資料的長度 = spi1_rx_buf長度 - DMA1_Channel2剩餘可搬運長度

            SPI1_DR = spi1_tx_buf[0]; // 覆蓋上一次通訊殘留的未傳送的無用資料，避免此無用資料被誤傳 (每次通訊的資料長度若不足SPI1_BUF_SIZE便會在SPI1_DR殘留無用資料)
            DMA1_CCR3 &= ~1; // 設定前先禁用DMA1_Channel3
            DMA1_CMAR3 = (uint32_t)(spi1_tx_buf + 1); // 設定記憶體位址
            DMA1_CNDTR3 = SPI1_BUF_SIZE - 1; // 設定搬運資料長度
            DMA1_CCR3 |= 1; // 啟用DMA1_Channel3

            (void)SPI1_DR; // 清空可能存在的殘留資料避免誤讀
            DMA1_CCR2 &= ~1; // 設定前先禁用DMA1_Channel2
            DMA1_CMAR2 = (uint32_t)spi1_rx_buf; // 設定記憶體位址
            DMA1_CNDTR2 = SPI1_BUF_SIZE; // 設定搬運資料長度
            DMA1_CCR2 |= 1; // 啟用DMA1_Channel2
        }
        EXTI_PR |= (1 << 4); // 清除Pending bit，否則此中斷會馬上再次觸發 (寫入1可清除此Reg該位元)
    }
}
