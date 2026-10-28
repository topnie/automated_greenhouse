/*
 * periodic_service_manager.c
 *
 *  Created on: 23 wrz 2026
 *      Author: Tomasz Pniewski
 */


#include "periodic_service_manager.h"

#include "stm32g431xx.h"

static volatile bool service_needed = true;

void RTC_WKUP_IRQHandler(void) {
	service_needed = true;
	// Unlock write to the RTC.
	PWR->CR1 |= PWR_CR1_DBP;
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	// Clear wake-up flags.
    RTC->SCR = RTC_SCR_CWUTF;
    EXTI->PR1 = EXTI_PR1_PIF20;
    RTC->WPR = 0x00;
    PWR->CR1 &= ~(PWR_CR1_DBP);
}

void initialize_RTC(void) {
	// Enable PWR peripheral clock.
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;

	// Enable access to the backup domain.
	PWR->CR1 |= PWR_CR1_DBP;

	// Force a clean backup-domain state so RTCSEL can actually be changed.
	RCC->BDCR |= RCC_BDCR_BDRST;
	RCC->BDCR &= ~RCC_BDCR_BDRST;

	// Start LSE.
	RCC->BDCR |= RCC_BDCR_LSEON;
	while (!(RCC->BDCR & RCC_BDCR_LSERDY))
	    ;

	// Disable RTC.
	RCC->BDCR &= ~(RCC_BDCR_RTCEN);

	// Select LSE as RTC clock.
	RCC->BDCR &= ~(RCC_BDCR_RTCSEL);
	RCC->BDCR |= (0b01UL << RCC_BDCR_RTCSEL_Pos);

	// Enable RTC.
	RCC->BDCR |= RCC_BDCR_RTCEN;

    // Lock backup domain again.
    PWR->CR1 &= ~PWR_CR1_DBP;
}

void initialize_timer(uint16_t timer_seconds) {
	if (timer_seconds == 0) {
		return;
	}
	NVIC_DisableIRQ(RTC_WKUP_IRQn);
	NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
	// Enable clock to PWR registers.
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	// Set DBP to enable writes to RTC registers.
	PWR->CR1 |= PWR_CR1_DBP;
	// Unlock write to the RTC.
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	// Disable the wake-up timer.
	RTC->CR &= ~(RTC_CR_WUTE);
	// Clear old RTC wake-up flag.
	RTC->SCR = RTC_SCR_CWUTF;

	// Wait until we can write the new timer values (2 RTCCLK cycles).
	while (!(RTC->ICSR & RTC_ICSR_WUTWF)) {
		;
	}
    RTC->PRER =
        (127U << RTC_PRER_PREDIV_A_Pos) |
        (255U << RTC_PRER_PREDIV_S_Pos);

	// Set the ck_spre clock for the timer.
	RTC->CR &= ~(RTC_CR_WUCKSEL);
	RTC->CR |= (0b100UL << RTC_CR_WUCKSEL_Pos);

	// Set the amount of seconds for the timer wake-up.
	RTC->WUTR = timer_seconds - 1;


	// Enable the wake-up timer.
	RTC->CR |= RTC_CR_WUTE;
	RTC->CR |= RTC_CR_WUTIE;

	// Disable write access.
	RTC->WPR = 0x00;
	PWR->CR1 &= ~(PWR_CR1_DBP);

	// Configure EXTI line 20 for rising edge.
	EXTI->IMR1 |= EXTI_IMR1_IM20;
	EXTI->RTSR1 |= EXTI_RTSR1_RT20;
	EXTI->FTSR1 &= ~(EXTI_FTSR1_FT20);
	// Clear any old EXTI20 pending flag.
	EXTI->PR1 = EXTI_PR1_PIF20;

	NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

bool is_service_needed(void) {
	return service_needed;
}

void service_done(void) {
	service_needed = false;
}

