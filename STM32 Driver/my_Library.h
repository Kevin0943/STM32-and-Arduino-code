#ifndef MY_LIBRARY_H
#define MY_LIBRARY_H


#include <stdint.h>


// ===== Base addresses =====
#define SYSTICK_BASE    0xE000E010
#define RCC_BASE        0x40021000
#define GPIOA_BASE      0x40010800
#define GPIOB_BASE      0x40010C00
#define GPIOC_BASE      0x40011000
#define TIM2_BASE       0x40000000
#define TIM3_BASE       0x40000400
#define ADC1_BASE       0x40012400
#define NVIC_BASE       0xE000E100
#define NVIC_IPR_BASE   0xE000E400
#define AFIO_BASE       0x40010000
#define EXTI_BASE       0x40010400
#define USART1_BASE     0x40013800
#define I2C1_BASE       0x40005400
#define SPI1_BASE       0x40013000
#define DMA1_BASE       0x40020000

// ===== SysTick registers =====
#define SYST_CSR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYST_RVR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYST_CVR        (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))
#define SCB_SHPR3        (*(volatile uint32_t *)0xE000ED20)

// ===== RCC registers =====
#define RCC_APB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0x18))
#define RCC_APB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x1C))
#define RCC_AHBENR      (*(volatile uint32_t *)(RCC_BASE + 0x14))

// ===== GPIOA registers =====
#define GPIOA_CRL       (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_CRH       (*(volatile uint32_t *)(GPIOA_BASE + 0x04))
#define GPIOA_IDR       (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_ODR       (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))

// ===== GPIOB registers =====
#define GPIOB_CRL       (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_CRH       (*(volatile uint32_t *)(GPIOB_BASE + 0x04))
#define GPIOB_ODR       (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))

// ===== GPIOC registers =====
#define GPIOC_CRH       (*(volatile uint32_t *)(GPIOC_BASE + 0x04))
#define GPIOC_IDR       (*(volatile uint32_t *)(GPIOC_BASE + 0x08))
#define GPIOC_ODR       (*(volatile uint32_t *)(GPIOC_BASE + 0x0C))

// ===== Timer registers =====
#define TIM2_CR1        (*(volatile uint16_t *)(TIM2_BASE + 0x00))
#define TIM2_DIER       (*(volatile uint16_t *)(TIM2_BASE + 0x0C))
#define TIM2_SR         (*(volatile uint16_t *)(TIM2_BASE + 0x10))
#define TIM2_EGR        (*(volatile uint16_t *)(TIM2_BASE + 0x14))
#define TIM2_CNT        (*(volatile uint16_t *)(TIM2_BASE + 0x24))
#define TIM2_PSC        (*(volatile uint16_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR        (*(volatile uint16_t *)(TIM2_BASE + 0x2C))
#define TIM2_CCMR1      (*(volatile uint16_t *)(TIM2_BASE + 0x18))
#define TIM2_CCER       (*(volatile uint16_t *)(TIM2_BASE + 0x20))
#define TIM2_CCR1       (*(volatile uint16_t *)(TIM2_BASE + 0x34))

#define TIM3_CR1        (*(volatile uint16_t *)(TIM3_BASE + 0x00))
#define TIM3_DIER       (*(volatile uint16_t *)(TIM3_BASE + 0x0C))
#define TIM3_SR         (*(volatile uint16_t *)(TIM3_BASE + 0x10))
#define TIM3_EGR        (*(volatile uint16_t *)(TIM3_BASE + 0x14))
#define TIM3_CNT        (*(volatile uint16_t *)(TIM3_BASE + 0x24))
#define TIM3_PSC        (*(volatile uint16_t *)(TIM3_BASE + 0x28))
#define TIM3_ARR        (*(volatile uint16_t *)(TIM3_BASE + 0x2C))

// ===== ADC1 registers =====
#define ADC1_SR         (*(volatile uint32_t *)(ADC1_BASE + 0x00))
#define ADC1_CR2        (*(volatile uint32_t *)(ADC1_BASE + 0x08))
#define ADC1_SMPR2      (*(volatile uint32_t *)(ADC1_BASE + 0x10))
#define ADC1_SQR1       (*(volatile uint32_t *)(ADC1_BASE + 0x2C))
#define ADC1_SQR3       (*(volatile uint32_t *)(ADC1_BASE + 0x34))
#define ADC1_DR         (*(volatile uint32_t *)(ADC1_BASE + 0x4C))

// ===== NVIC registers =====
#define NVIC_ISER0      (*(volatile uint32_t *)(NVIC_BASE + 0x00))
#define NVIC_ISER1      (*(volatile uint32_t *)(NVIC_BASE + 0x04))

// ===== AFIO registers =====
#define AFIO_EXTICR2    (*(volatile uint32_t *)(AFIO_BASE + 0x0C))

// ===== EXTI registers =====
#define EXTI_IMR        (*(volatile uint32_t*)(EXTI_BASE + 0x00))
#define EXTI_EMR        (*(volatile uint32_t*)(EXTI_BASE + 0x04))
#define EXTI_RTSR       (*(volatile uint32_t*)(EXTI_BASE + 0x08))
#define EXTI_FTSR       (*(volatile uint32_t*)(EXTI_BASE + 0x0C))
#define EXTI_PR         (*(volatile uint32_t*)(EXTI_BASE + 0x14))

// ===== USART1 registers =====
#define USART1_SR       (*(volatile uint32_t *)(USART1_BASE + 0x00))
#define USART1_DR       (*(volatile uint32_t *)(USART1_BASE + 0x04))
#define USART1_BRR      (*(volatile uint32_t *)(USART1_BASE + 0x08))
#define USART1_CR1      (*(volatile uint32_t *)(USART1_BASE + 0x0C))
#define USART1_CR3      (*(volatile uint32_t *)(USART1_BASE + 0x14))

// ===== I2C1 registers =====
#define I2C1_CR1        (*(volatile uint32_t *)(I2C1_BASE + 0x00))
#define I2C1_CR2        (*(volatile uint32_t *)(I2C1_BASE + 0x04))
#define I2C1_OAR1       (*(volatile uint32_t *)(I2C1_BASE + 0x08))
#define I2C1_CCR        (*(volatile uint32_t *)(I2C1_BASE + 0x1C))
#define I2C1_TRISE      (*(volatile uint32_t *)(I2C1_BASE + 0x20))
#define I2C1_DR         (*(volatile uint32_t *)(I2C1_BASE + 0x10))
#define I2C1_SR1        (*(volatile uint32_t *)(I2C1_BASE + 0x14))
#define I2C1_SR2        (*(volatile uint32_t *)(I2C1_BASE + 0x18))

// ===== SPI1 registers =====
#define SPI1_CR1        (*(volatile uint32_t *)(SPI1_BASE + 0x00))
#define SPI1_CR2        (*(volatile uint32_t *)(SPI1_BASE + 0x04))
#define SPI1_SR         (*(volatile uint32_t *)(SPI1_BASE + 0x08))
#define SPI1_DR         (*(volatile uint32_t *)(SPI1_BASE + 0x0C))

// ===== DMA1 registers =====
#define DMA1_ISR        (*(volatile uint32_t *)(DMA1_BASE + 0x00))
#define DMA1_IFCR       (*(volatile uint32_t *)(DMA1_BASE + 0x04))

#define DMA1_CCR2       (*(volatile uint32_t *)(DMA1_BASE + 0x1C))
#define DMA1_CNDTR2     (*(volatile uint32_t *)(DMA1_BASE + 0x20))
#define DMA1_CPAR2      (*(volatile uint32_t *)(DMA1_BASE + 0x24))
#define DMA1_CMAR2      (*(volatile uint32_t *)(DMA1_BASE + 0x28))

#define DMA1_CCR3       (*(volatile uint32_t *)(DMA1_BASE + 0x30))
#define DMA1_CNDTR3     (*(volatile uint32_t *)(DMA1_BASE + 0x34))
#define DMA1_CPAR3      (*(volatile uint32_t *)(DMA1_BASE + 0x38))
#define DMA1_CMAR3      (*(volatile uint32_t *)(DMA1_BASE + 0x3C))

#define DMA1_CCR4       (*(volatile uint32_t *)(DMA1_BASE + 0x44))
#define DMA1_CNDTR4     (*(volatile uint32_t *)(DMA1_BASE + 0x48))
#define DMA1_CPAR4      (*(volatile uint32_t *)(DMA1_BASE + 0x4C))
#define DMA1_CMAR4      (*(volatile uint32_t *)(DMA1_BASE + 0x50))

#define DMA1_CCR5       (*(volatile uint32_t *)(DMA1_BASE + 0x58))
#define DMA1_CNDTR5     (*(volatile uint32_t *)(DMA1_BASE + 0x5C))
#define DMA1_CPAR5      (*(volatile uint32_t *)(DMA1_BASE + 0x60))
#define DMA1_CMAR5      (*(volatile uint32_t *)(DMA1_BASE + 0x64))

#define DMA1_CCR6       (*(volatile uint32_t *)(DMA1_BASE + 0x6C))
#define DMA1_CNDTR6     (*(volatile uint32_t *)(DMA1_BASE + 0x70))
#define DMA1_CPAR6      (*(volatile uint32_t *)(DMA1_BASE + 0x74))
#define DMA1_CMAR6      (*(volatile uint32_t *)(DMA1_BASE + 0x78))

#define DMA1_CCR7       (*(volatile uint32_t *)(DMA1_BASE + 0x80))
#define DMA1_CNDTR7     (*(volatile uint32_t *)(DMA1_BASE + 0x84))
#define DMA1_CPAR7      (*(volatile uint32_t *)(DMA1_BASE + 0x88))
#define DMA1_CMAR7      (*(volatile uint32_t *)(DMA1_BASE + 0x8C))

// ===== delay_ms =====
extern volatile unsigned int ms_ticks;
void systick_init(void);
void delay_ms(unsigned int ms);

// ===== blinkblink =====
void blinkblink_init(void);
void blinkblink(uint8_t k, unsigned int s);

// ===== PWM1(with Timer2) =====
void PWM1_Init(unsigned int us1, unsigned int us2);
void PWM1_ChangePeriod(unsigned int us);
void PWM1_ChangeHighTime(unsigned int us);
void PWM1_Enable(void);
void PWM1_Disable(void);

// ===== Timer3(ms) =====
extern volatile unsigned int Timer3_ms_flag;
void TIM3_ms_Init(unsigned int ms);
void TIM3_ms_ChangeTime(unsigned int ms);
void TIM3_ms_Enable(void);
void TIM3_ms_Disable(void);

// ===== ADC1 =====
void ADC1_Init(void);
uint16_t ADC1_Read(void);

// ===== string to unsigned int =====
unsigned int string_to_uint(const volatile uint8_t *str, int len);

// ===== float to string =====
void float_to_string(float num, char *buf, int int_digits, int frac_digits);


#endif // MY_LIBRARY_H
