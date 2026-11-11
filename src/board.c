/**
 * @file board.c
 * @brief Board bring-up for STM32F030F4P6 - see board.h
 *
 * Clock: 48 MHz from HSI/2 (4 MHz) through the x12 PLL, 1 flash wait state.
 *
 * Pin map (TSSOP-20; only PA0-PA7, PA9-PA10, PA13-PA14, PB1, PF0-PF1 exist):
 *   PA10  1-Wire DQ          (owned by the 1-Wire library, AF2 TIM1_CH3)
 *   PA9   strong pull-up switch  gate-driver input, active-high, push-pull
 *   PA0   status LED         push-pull output
 *   PA1   error LED          push-pull output
 *   PB1   config jumper 0    input, pull-up
 *   PF0   config jumper 1    input, pull-up
 *   PF1   spare              input, pull-up
 *   PA13/PA14  SWD           left at their reset alternate function
 *
 * Host pins are variant-dependent:
 *   UART: PA2 = TX, PA3 = RX (AF1)
 *   SPI:  PA4 = CS (GPIO input), PA5 = SCK, PA6 = MISO, PA7 = MOSI (AF0)
 *   I2C:  PA4 = SCL (input, pull-up), PA5 = SDA (open-drain)
 */

#include "board.h"

#include "stm32f0xx.h"

#ifndef OW_SYSCLK_MHZ
#error "OW_SYSCLK_MHZ must be defined (the CMake project sets it)"
#endif
#if (OW_SYSCLK_MHZ) != 48
#error "board.c implements the 48 MHz HSI+PLL clock only"
#endif

#define CONSOLE_BAUD 115200u
#define CONSOLE_BRR ((((uint32_t)OW_SYSCLK_MHZ * 1000000u) + (CONSOLE_BAUD / 2u)) / CONSOLE_BAUD)

static uint32_t ms_count;

/* --- GPIO helpers --------------------------------------------------------- */

static void gpio_mode(GPIO_TypeDef *port, uint8_t pin, uint32_t mode)
{
    port->MODER = (port->MODER & ~(3u << (2u * pin))) | (mode << (2u * pin));
}

__attribute__((unused)) static void gpio_af(GPIO_TypeDef *port, uint8_t pin, uint8_t af)
{
    volatile uint32_t *afr = (pin < 8u) ? &port->AFR[0] : &port->AFR[1];
    const uint8_t shift = (uint8_t)((pin & 7u) * 4u);
    *afr = (*afr & ~(0xFu << shift)) | ((uint32_t)af << shift);
}

static void gpio_push_pull(GPIO_TypeDef *port, uint8_t pin, uint8_t od)
{
    if (od) {
        port->OTYPER |= (1u << pin);
    } else {
        port->OTYPER &= ~(1u << pin);
    }
}

static void gpio_pull_up(GPIO_TypeDef *port, uint8_t pin)
{
    port->PUPDR = (port->PUPDR & ~(3u << (2u * pin))) | (1u << (2u * pin));
}

static void gpio_set(GPIO_TypeDef *port, uint8_t pin, uint8_t high)
{
    port->BSRR = (uint32_t)(1u << pin) << (high ? 0u : 16u);
}

/* --- Clock ---------------------------------------------------------------- */

static void board_clock_init(void)
{
    /* HSI/2 = 4 MHz is the PLL input; x12 = 48 MHz. Write PLLMUL before
       enabling the PLL so it locks on a valid configuration (RM0360). */
    RCC->CFGR = RCC_CFGR_PLLMUL12;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    }
    /* 1 wait state covers 48 MHz, prefetch on. */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }
}

/* --- GPIO ----------------------------------------------------------------- */

static void board_gpio_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    /* GPIOF is on AHB for the oscillator pins; enable it so PF0/PF1 are GPIOs. */
    RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

#if defined(HOST_IF_UART)
    /* USART1 TX/RX (PA2/PA3, AF1) - host UART. */
    gpio_mode(GPIOA, 2, 2u);
    gpio_af(GPIOA, 2, 1u);
    gpio_push_pull(GPIOA, 2, 0u);
    gpio_mode(GPIOA, 3, 2u);
    gpio_af(GPIOA, 3, 1u);
    gpio_push_pull(GPIOA, 3, 0u);
#else
    gpio_mode(GPIOA, 2, 0u);
    gpio_pull_up(GPIOA, 2);
    gpio_mode(GPIOA, 3, 0u);
    gpio_pull_up(GPIOA, 3);
#endif

#if defined(HOST_IF_SPI)
    /* SPI1 (PA5 SCK, PA6 MISO, PA7 MOSI, AF0). PA4 is a software CS input. */
    gpio_mode(GPIOA, 4, 0u);
    gpio_pull_up(GPIOA, 4);
    gpio_mode(GPIOA, 5, 2u);
    gpio_af(GPIOA, 5, 0u);
    gpio_push_pull(GPIOA, 5, 0u);
    gpio_mode(GPIOA, 6, 2u);
    gpio_af(GPIOA, 6, 0u);
    gpio_mode(GPIOA, 7, 2u);
    gpio_af(GPIOA, 7, 0u);
    gpio_push_pull(GPIOA, 7, 0u);
#elif defined(HOST_IF_I2C)
    /* Software I2C: PA4 = SCL (input, pull-up), PA5 = SDA (open-drain). */
    gpio_mode(GPIOA, 4, 0u);
    gpio_pull_up(GPIOA, 4);
    gpio_mode(GPIOA, 5, 1u);
    gpio_push_pull(GPIOA, 5, 1u); /* open-drain */
    gpio_pull_up(GPIOA, 5);
    gpio_set(GPIOA, 5, 1u);       /* release SDA */
    gpio_mode(GPIOA, 6, 0u);
    gpio_pull_up(GPIOA, 6);
    gpio_mode(GPIOA, 7, 0u);
    gpio_pull_up(GPIOA, 7);
#else
    gpio_mode(GPIOA, 4, 0u);
    gpio_pull_up(GPIOA, 4);
    gpio_mode(GPIOA, 5, 0u);
    gpio_pull_up(GPIOA, 5);
    gpio_mode(GPIOA, 6, 0u);
    gpio_pull_up(GPIOA, 6);
    gpio_mode(GPIOA, 7, 0u);
    gpio_pull_up(GPIOA, 7);
#endif

    /* Strong pull-up gate (PA9), off at reset. */
    gpio_mode(GPIOA, 9, 1u);
    gpio_push_pull(GPIOA, 9, 0u);
    gpio_set(GPIOA, 9, 0u);

    /* LEDs (PA0/PA1), off at reset. */
    gpio_mode(GPIOA, 0, 1u);
    gpio_push_pull(GPIOA, 0, 0u);
    gpio_set(GPIOA, 0, 0u);
    gpio_mode(GPIOA, 1, 1u);
    gpio_push_pull(GPIOA, 1, 0u);
    gpio_set(GPIOA, 1, 0u);

    /* Config jumpers (PB1/PF0), input pull-up. */
    gpio_mode(GPIOB, 1, 0u);
    gpio_pull_up(GPIOB, 1);
    gpio_mode(GPIOF, 0, 0u);
    gpio_pull_up(GPIOF, 0);

    /* Spare (PF1), input pull-up. */
    gpio_mode(GPIOF, 1, 0u);
    gpio_pull_up(GPIOF, 1);
}

/* --- Console helpers (UART build only) ----------------------------------- */
/* USART1 is owned by host_uart.c on PA2/PA3; these helpers are not a separate
   console. They are compiled only in the UART build, after host_uart_init(). */

#if defined(HOST_IF_UART)

void board_putc(char c)
{
    while (!(USART1->ISR & USART_ISR_TXE)) {
    }
    USART1->TDR = (uint8_t)c;
}

void board_puts(const char *s)
{
    while (*s) {
        board_putc(*s++);
    }
}

void board_put_int(int32_t value)
{
    char buf[12];
    char *p = buf + sizeof(buf) - 1u;
    *p = '\0';

    if (value == 0) {
        *(--p) = '0';
    } else {
        uint32_t uvalue;
        uint8_t negative = 0u;
        if (value < 0) {
            negative = 1u;
            uvalue = (uint32_t)-(value + 1) + 1u;
        } else {
            uvalue = (uint32_t)value;
        }
        do {
            *(--p) = (char)('0' + (uvalue % 10u));
            uvalue /= 10u;
        } while (uvalue);
        if (negative) {
            *(--p) = '-';
        }
    }
    board_puts(p);
}

void board_put_hex(uint8_t b)
{
    static const char hex[] = "0123456789ABCDEF";
    board_putc(hex[b >> 4]);
    board_putc(hex[b & 0x0Fu]);
}

#endif /* HOST_IF_UART */

/* --- Millisecond tick (SysTick, polled, no interrupt) --------------------- */

static void board_tick_init(void)
{
    ms_count = 0u;
    SysTick->LOAD = ((uint32_t)OW_SYSCLK_MHZ * 1000u) - 1u;
    SysTick->VAL = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

uint32_t board_millis(void)
{
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
        (void)SysTick->CTRL; /* read clears COUNTFLAG */
        ms_count++;
    }
    return ms_count;
}

/* --- Indicators / actuator ------------------------------------------------ */

void board_led_status(uint8_t on)
{
    gpio_set(GPIOA, 0, on ? 1u : 0u);
}

void board_led_error(uint8_t on)
{
    gpio_set(GPIOA, 1, on ? 1u : 0u);
}

void board_gate_pullup(uint8_t on)
{
    gpio_set(GPIOA, 9, on ? 1u : 0u);
}

/* --- Watchdog ------------------------------------------------------------- */

void board_watchdog_init(void)
{
    /* Start the watchdog first: the IWDG clock (LSI) is enabled by the
       KR=0xCCCC write. Unlocking before that leaves the PVU/RVU update flags
       stuck and the firmware hangs in the wait loop below. */
    IWDG->KR = 0xCCCCu;  /* enable (starts the LSI) */
    IWDG->KR = 0x5555u;  /* unlock PR/RLR */
    IWDG->PR = 0x4u;     /* /64 */
    IWDG->RLR = 625u;    /* ~1 s with the 40 kHz LSI */
    while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) {
    }
    IWDG->KR = 0xAAAAu;  /* reload */
}

void board_watchdog_refresh(void)
{
    IWDG->KR = 0xAAAAu;
}

/* --- Jumpers -------------------------------------------------------------- */

uint8_t board_read_jumpers(void)
{
    uint8_t v = 0u;
    if (!(GPIOB->IDR & (1u << 1))) {
        v |= 0x01u;
    }
    if (!(GPIOF->IDR & (1u << 0))) {
        v |= 0x02u;
    }
    return v;
}

/* --- Public init ---------------------------------------------------------- */

void board_init(void)
{
    board_clock_init();
    board_gpio_init();
    board_tick_init();
    board_watchdog_init();
}

void board_reset(void)
{
    NVIC_SystemReset();
}
