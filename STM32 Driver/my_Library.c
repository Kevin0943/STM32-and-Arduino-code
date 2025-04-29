#include "my_library.h"


/**
適用型號：STM32F103C8T6


===== delay_ms 說明 =====
volatile unsigned int ms_ticks:
紀錄SysTick中斷觸發次數

void systick_init(void):
SysTick初始化，使SysTick中斷每毫秒觸發一次
執行後才可使用delay_ms()和blinkblink()

void SysTick_Handler(void):
SysTick中斷處理函式，每次觸發執行ms_ticks++

void delay_ms(unsigned int ms):
迴圈檢查ms_ticks值實現delay ms 毫秒


===== blinkblink 說明 =====
void blinkblink_init(void):
設定板子上的自帶的燈炮(PC13)

void blinkblink(uint8_t k, unsigned int s):
閃燈炮k次，週期2*s ms


===== PWM1(with Timer2) 說明 =====
void PWM1_Init(unsigned int us1, unsigned int us2):
PWM1 初始化，設置PA0為PWM輸出引腳
輸入 us1：PWM週期(微秒)  us2：PWM高電位時間(微秒)

void PWM1_ChangePeriod(unsigned int us):
PWM1 改變PWM週期

void PWM1_ChangeHighTime(unsigned int us):
PWM1 改變PWM高電位時間

void PWM1_Enable(void):
PWM1 啟用

void PWM1_Disable(void):
PWM1 關閉


===== Timer3(ms) 說明 =====
volatile unsigned int Timer3_ms_flag:
Timer3更新中斷觸發時設置Timer3_ms_flag = 1，需手動重設為0

void TIM3_ms_Init(unsigned int ms):
Timer3 初始化
輸入ms：幾毫秒觸發一次Timer3更新中斷

void TIM3_ms_ChangeTime(unsigned int ms):
Timer3 改變觸發Timer3更新中斷的時間

void TIM3_ms_Enable(void):
Timer3 啟用

void TIM3_ms_Disable(void):
Timer3 關閉

void TIM3_IRQHandler(void):
Timer3 更新中斷處理函式，觸發時設置Timer3_ms_flag = 1


===== ADC1 說明 =====
void ADC1_Init(void):
ADC1 初始化，設置PA1為類比輸入引腳

uint16_t ADC1_Read(void):
ADC1 讀取電壓


===== string to unsigned int 說明 =====
unsigned int string_to_uint(const volatile uint8_t *str, int len):
將字串str[0:len]轉換成unsigned int並回傳


===== float to string 說明 =====
void float_to_string(float num, char *buf, int int_digits, int frac_digits):
將float num轉換成字串放入buf，限制範圍為：小數點前int_digits位、小數點後frac_digits位

 */


// ===== delay_ms =====
volatile unsigned int ms_ticks = 0; // 紀錄SysTick中斷觸發次數

// --- SysTick 初始化 ---
void systick_init(void) {
    SYST_RVR = 8000 - 1; // 設定Reload Value(計數器每次倒數的初始)，HCLK頻率維持系統預設值8MHz，8MHz / 8000 = 1kHz，每1ms觸發一次SysTick中斷
    SYST_CVR = 0; // 清除目前的計數值，寫入任意值都會將目前的計數器清為0，確保下個clock reload成SYST_RVR值
    SYST_CSR = 0x07; // SYST_CSR[2:0] = 111，使用系統主時脈(HCLK)、啟用SysTick中斷、開始倒數
    NVIC_ISER0 |= (1 << 15); // 設定SysTick中斷向量(IRQ15)
    SCB_SHPR3 &= ~(0xFF << 24);
    SCB_SHPR3 |= (15 << 24); // 設SysTick中斷的優先權為Level 15(最低)，避免干擾其他更重要的中斷
}

void delay_ms(unsigned int ms) { // 等待ms次SysTick中斷(每秒1000次SysTick中斷)
    unsigned int start = ms_ticks;
    while ((ms_ticks - start) < ms); // ms_ticks 溢位不影響
}

void SysTick_Handler(void) { // SysTick中斷處理函式
    ms_ticks++;
}

// ===== blinkblink =====
// --- 設定板子上的自帶的燈炮(PC13) ---
void blinkblink_init(void){
    // 開啟GPIOC時鐘
    RCC_APB2ENR |= (1 << 4);

    // 設定PC13為General purpose output Push-pull，最大 2MHz
    GPIOC_CRH &= ~(0xF << 20); // 清除原設定
    GPIOC_CRH |=  (0x2 << 20); // CNF13[1:0]=00(General purpose output Push-pull)，MODE13[1:0]=10(2MHz)

    GPIOC_ODR |= (1 << 13); // 將PC13預設為HIGH(燈泡不亮)
}

// --- 閃板子上的自帶的燈炮(PC13) ---
void blinkblink(uint8_t k, unsigned int s){ // 閃燈炮k次，週期2*s ms
    for(int i=0; i<k; i++){
        GPIOC_ODR ^= (1 << 13);
        delay_ms(s);
        GPIOC_ODR ^= (1 << 13);
        delay_ms(s);
    }
}

// ===== PWM1(with Timer2) =====
// --- PWM1 初始化 ---
void PWM1_Init(unsigned int us1, unsigned int us2) { // 輸入 us1：PWM週期(微秒)  us2：PWM高電位時間(微秒)
    // 啟用 GPIOA 時鐘
    RCC_APB2ENR |= (1 << 2); // RCC_APB2ENR: IOPAEN

    // 配置 PA0 引腳(PWM輸出引腳)
    GPIOA_CRL &= ~(0xF << 0);
    GPIOA_CRL |=  (0xB << 0); // Alternate function output Push-pull, 50MHz
    GPIOA_ODR &= ~(1 << 0); // PA0拉低

    // 啟用TIM2時鐘
    RCC_APB1ENR |= (1 << 0);  // TIM2EN

    // 關閉CC1通道輸出
    TIM2_CCER &= ~(1 << 0); // 關閉CC1通道輸出

    // 設定計數器
    TIM2_PSC = 8 - 1; // 8MHz / 8 = 1MHz，一秒計數1M次
    TIM2_ARR = us1 - 1; // us1微秒觸發一次Timer2更新，PWM週期為us1微秒

    // PWM設定
    TIM2_CCMR1 &= ~(3 << 0);  // CC1S = 00，CC1通道設為輸出
    TIM2_CCMR1 |= (6 << 4);  // OC1M = 110: PWM mode 1
    TIM2_CCMR1 |= (1 << 3);  // OC1PE: 啟用TIM2_CCR1更新緩衝器
    TIM2_CR1 |= (1 << 7); // 啟用TIM2_ARR更新緩衝器
    TIM2_CCR1 = us2; // 設定PWM高電位時間us2微秒

    // 啟用計數器
    TIM2_CR1 |= (1 << 0);  // CEN啟用計數器
}

// --- PWM1 改變PWM週期 ---
void PWM1_ChangePeriod(unsigned int us) { // 輸入us：PWM週期(微秒)
    TIM2_ARR = us - 1; // us1微秒觸發一次Timer2更新
}

// --- PWM1 改變PWM高電位時間 ---
void PWM1_ChangeHighTime(unsigned int us) { // 輸入us：PWM高電位時間(微秒)
    TIM2_CCR1 = us; // 設定PWM高電位時間為us微秒
}

// --- PWM1 啟用 ---
void PWM1_Enable(void) { // 輸入us：PWM高電位時間(微秒)
    TIM2_CCER |= (1 << 0); // 啟用CC1通道輸出
}

// --- PWM1 關閉 ---
void PWM1_Disable(void) {
    TIM2_CCER &= ~(1 << 0); // 關閉CC1通道輸出
    GPIOA_ODR &= ~(1 << 0); // PA0拉低
}

// ===== Timer3(ms) =====
volatile unsigned int Timer3_ms_flag = 0;
// --- Timer3 初始化 ---
void TIM3_ms_Init(unsigned int ms) { // 輸入ms：幾毫秒觸發一次Timer3更新中斷
    // 啟用TIM3時鐘
    RCC_APB1ENR |= (1 << 1); // TIM3EN

    // 關閉計數器
    TIM3_CR1 &= ~(1 << 0); // CEN關閉計數器

    // 設定計數器
    TIM3_PSC = 8000 - 1; // 8MHz / 8000 = 1kHz，一秒計數1000次
    TIM3_ARR = ms - 1; // ms毫秒觸發一次Timer3更新中斷
    TIM3_CR1 |= (1 << 7); // 啟用TIM3_ARR更新緩衝器

    // 啟用更新中斷
    TIM3_DIER |= (1 << 0); // UIE

    // 設定NVIC TIM3更新中斷向量
    NVIC_ISER0 |= (1 << 29);

    Timer3_ms_flag = 0;
}

// --- Timer3 改變觸發Timer3更新中斷的時間 ---
void TIM3_ms_ChangeTime(unsigned int ms) {
    TIM3_ARR = ms - 1; // ms毫秒觸發一次Timer3更新中斷
}

// --- Timer3 啟用 ---
void TIM3_ms_Enable(void) {
    TIM3_CR1 |= (1 << 0); // CEN啟用計數器
    Timer3_ms_flag = 0;
}

// --- Timer3 關閉 ---
void TIM3_ms_Disable(void) {
    TIM3_CR1 &= ~(1 << 0); // CEN關閉計數器
    Timer3_ms_flag = 0;
}

// --- Timer3 更新中斷處理函式 ---
void TIM3_IRQHandler(void) {
    if (TIM3_SR & (1 << 0)) {  // UIF
        TIM3_SR &= ~(1 << 0);  // 清除中斷flag

        Timer3_ms_flag = 1;
    }
}

// ===== ADC1 =====
// --- ADC1 初始化 ---
void ADC1_Init(void) {
    // 啟用 GPIOA 與 ADC1 時鐘
    RCC_APB2ENR |= (1 << 2); // GPIOAEN
    RCC_APB2ENR |= (1 << 9); // ADC1EN

    // 配置 PA1 引腳
    GPIOA_CRL &= ~(0xF << 4); // PA1 類比輸入 (CNF=00, MODE=00)

    // 選擇轉換通道為Channel 1 (PA1)
    ADC1_SQR3 &= ~(0x1F << 0);
    ADC1_SQR3 |=  (1 << 0); // SQ1[4:0] = 00001

    // 設定Channel 1取樣時間 = 55.5 cycles
    ADC1_SMPR2 |= (0x5 << 3); // SMP1[5:3] = 101

    // 設定通道數為 1
    ADC1_SQR1 &= ~(0xF << 20); // L[3:0] = 0000

    // 選擇觸發來源為 SWSTART
    ADC1_CR2 |= (0x7 << 17); // EXTSEL=111
    ADC1_CR2 |= (1 << 20); // EXTTRIG=1

    // 開啟 ADC
    ADC1_CR2 |= (1 << 0); // ADON=1 (Power Up)
    delay_ms(10); // tSTAB 時間等待

    // 啟動校準
    ADC1_CR2 |= (1 << 2); // CAL=1
    while (ADC1_CR2 & (1 << 2)); // 等待校準完成
}

// --- ADC1 讀取電壓 ---
uint16_t ADC1_Read(void) {
    // 觸發轉換
    ADC1_CR2 |= (1 << 22); // SWSTART=1

    // 等待轉換完成
    while (!(ADC1_SR & (1 << 1))); // EOC=1

    // return 讀取值
    return (uint16_t)ADC1_DR;
}

// --- string to unsigned int ---
unsigned int string_to_uint(const volatile uint8_t *str, int len) {
    unsigned int result = 0;

    for(int i=0; i<len; i++) {
        if (*str >= '0' && *str <= '9') {
            result = result * 10 + (*str - '0');
        } else {
            break; // 遇到非數字字元就停止
        }
        str++;
    }

    return result;
}

// --- float to string ---
void float_to_string(float num, char *buf, int int_digits, int frac_digits) {
    char *buf_0 = buf;

    if (num < 0) {
        *buf++ = '-';
        num = -num;
    }

    // 分離整數與小數部分
    int int_part = (int)num;
    float frac_part = num - (float)int_part;

    // 整數轉字串，先存進 tmp 再反轉
    int i = 0;
    char tmp[12];
    while (int_part && i < sizeof(tmp)) {
        tmp[i++] = '0' + (int_part % 10);
        int_part /= 10;
    }

    // 若原本就是 0，還是要補一個 0
    if (i == 0) tmp[i++] = '0';

    // 補前導 0，直到達到 int_digits
    while (i < int_digits) {
        tmp[i++] = '0';
    }

    // 反轉輸出整數部分
    while (i--) *buf++ = tmp[i];

    if(frac_digits < 1) {
        *buf = '\0';  // 結尾
        *(buf_0 + int_digits) = '\0'; // 防止string長度過長
        return;
    }

    *buf++ = '.';  // 小數點

    // 小數轉字串
    for (int j = 0; j < frac_digits; j++) {
        frac_part *= 10.0f;
        int digit = (int)frac_part;
        *buf++ = '0' + digit;
        frac_part -= digit;
    }

    *buf = '\0';  // 結尾

    *(buf_0 + int_digits + frac_digits + 1) = '\0'; // 防止string長度過長
}
