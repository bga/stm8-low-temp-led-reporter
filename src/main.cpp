/*
  Copyright 2021 Bga <bga.email@gmail.com>

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

//#define F_CPU 8000000UL

#include <stdint.h>

#include <stm8s.h>
#include <intrinsics.h>
#include <eeprom.h>
#include <delay.h>

//#define ENABLE_STATIC_PRINT

#include <!cpp/bitManipulations.h>
#include <!cpp/Binary_values_8_bit.h>
#include <!cpp/RunningAvg.h>
#include <!cpp/newKeywords.h>
//#include <!cpp/debug.h>

//# TODO timer based delay_ms instead dumb loop

using namespace Stm8Hal;

#define delay_ms TIM4_delay_ms

enum { adcMaxBufferSize = 16 };

void wfiLowPowerMode() {
//	CLK_CKDIVR = (0x3 << 3) | 0x7;
	__wait_for_interrupt();
}
void enterHighSpeedMode() {
//	CLK_CKDIVR = (0x3 << 3);
}

typedef FU16 Adc_Value;

enum {
	//# TODO calculate real RC time
	Adc_delay_us = 24,
	Adc_maxValue = 1024,
};

void Adc_init() {
	/* right-align data */
	setBit(ADC1_CR2, ADC1_CR2_ALIGN);

	//# ADC clock = fMasterClock / 18
//	setBitMaskedValues(ADC1_CR1, 4, 0x07, 7);

	/* wake ADC from power down */
	setBit(ADC1_CR1, ADC1_CR1_ADON);
}

void inline Adc_initChannel(FU8 channelNo) {
	#if 0
	if(channelNo < 8) {
		setBit(ADC2_TDRL, channelNo);
	}
	else {
		setBit(ADC2_TDRH, channelNo - 8);
	}
	#else
		setBit(((U16 *)&ADC2_TDRH)[0], channelNo);
	#endif
}

void Adc_setChannel(FU8 channelNo) {
	setBitMaskedValues(ADC1_CSR, 0, 0x0F, channelNo);
}

void Adc_readStart() {
	setBit(ADC1_CR1, ADC1_CR1_ADON);
}

Adc_Value Adc_read() {
	while (!(ADC1_CSR & _BV(ADC1_CSR_EOC)));
	U8 adcL = ADC1_DRL;
	U8 adcH = ADC1_DRH;
	clearBit(ADC1_CSR, ADC1_CSR_EOC);
	return (adcL | (adcH << 8));
}
Adc_Value Adc_readSync(FU8 channelNo) {
	Adc_setChannel(channelNo);
	Adc_readStart();
	return Adc_read();
}

enum {
	AdcUser_minValue = 100,
	AdcUser_maxValue = 1000,
};

Bool Adc_checkWrongValue(Adc_Value v) {
	return AdcUser_minValue <= v && v <= AdcUser_maxValue;
}


template<int portAddrArg, int bitNoArg>
struct Pin_PushPull {
	enum {
		portAddr = portAddrArg,
		bitNo = bitNoArg,
	};

	GPIO_TypeDef* const m_gpioPort = (GPIO_TypeDef*)portAddr;
	void init() {
		setBit(m_gpioPort->DDR, bitNo);
		setBit(m_gpioPort->CR1, bitNo);
	}
	void on() {
		setBit(m_gpioPort->ODR, bitNo);
	}
	void off() {
		clearBit(m_gpioPort->ODR, bitNo);
	}
	void toggle() {
		toggleBit(m_gpioPort->ODR, bitNo);
	}
};

template<int portAddrArg, int bitNoArg>
struct Pin_PullHiZ {
	enum {
		portAddr = portAddrArg,
		bitNo = bitNoArg,
	};

	GPIO_TypeDef* const m_gpioPort = (GPIO_TypeDef*)portAddr;
	void init() {
	}
	void hiZ() {
		clearBit(m_gpioPort->DDR, bitNo);
	}
	void off() {
		setBit(m_gpioPort->DDR, bitNo);
	}
	void toggle() {
		toggleBit(m_gpioPort->DDR, bitNo);
	}
};

template<int portAddrArg, int bitNoArg>
struct Pin_PushHiZ {
	enum {
		portAddr = portAddrArg,
		bitNo = bitNoArg,
	};

	GPIO_TypeDef* const m_gpioPort = (GPIO_TypeDef*)portAddr;
	void init() {
		setBit(m_gpioPort->ODR, bitNo);
	}
	void hiZ() {
		clearBit(m_gpioPort->DDR, bitNo);
	}
	void on() {
		setBit(m_gpioPort->DDR, bitNo);
	}
	void toggle() {
		toggleBit(m_gpioPort->DDR, bitNo);
	}
};

template<int portAddrArg, int bitNoArg>
struct Pin_PushPullHiZ {
	enum {
		portAddr = portAddrArg,
		bitNo = bitNoArg,
	};

	GPIO_TypeDef* const m_gpioPort = (GPIO_TypeDef*)portAddr;
	void init() {
	}
	void hiZ() {
		clearBit(m_gpioPort->DDR, bitNo);
		clearBit(m_gpioPort->CR1, bitNo);
		clearBit(m_gpioPort->ODR, bitNo);
	}
	void on() {
		setBit(m_gpioPort->DDR, bitNo);
		setBit(m_gpioPort->CR1, bitNo);
		setBit(m_gpioPort->ODR, bitNo);
	}
	void off() {
		setBit(m_gpioPort->DDR, bitNo);
		setBit(m_gpioPort->CR1, bitNo);
		clearBit(m_gpioPort->ODR, bitNo);
	}
	void toggle() {
		setBit(m_gpioPort->DDR, bitNo);
		setBit(m_gpioPort->CR1, bitNo);
		toggleBit(m_gpioPort->ODR, bitNo);
	}
};




template<
	int adcPortAddrArg, int adcBitNoArg,
	int adcChannelNoArg>
struct Pin_AdcWithEnable {
	enum {
		adcPortAddr = adcPortAddrArg,
		adcBitNo = adcBitNoArg,
		adcChannelNo = adcChannelNoArg,
	};

	Pin_PullHiZ<adcPortAddr, adcBitNo> m_adcPin;

	void init() {
	}

	Adc_Value readSync() {
		// HiZ
		m_adcPin.hiZ();

//		m_vccPin.on();
		delay_us(Adc_delay_us);
		Adc_Value ret = Adc_readSync(adcChannelNo);
//		m_vccPin.off();

		m_adcPin.off();
		return ret;
	}
};

Pin_PushPullHiZ<PC_BASE_ADDRESS, 7> tempDividerVccPin;
Pin_AdcWithEnable<PD_BASE_ADDRESS, 2, 3> tempDividerAdcPin;
Pin_PushPull<PC_BASE_ADDRESS, 5> batteryVccPin;
Pin_AdcWithEnable<PC_BASE_ADDRESS, 4, 2> batteryAdcPin;
Pin_PushPull<PA_BASE_ADDRESS, 2> statusLedPin;



typedef U16 U1_15;
typedef U16 U3_13;

struct Settings {
	U1_15 tempDividerMultipier;
	U16 tempHysteresisLow;
	U16 tempHysteresisHigh;
	U16 vcc_ref_mV;
	U16 vccAdcLow_mV;
	U16 UNIQUE_NAME; //# for backward compatibility
	U16 statusLedBlinkShortDelay_ms;
	U16 statusLedBlinkLongDelay_ms;
	U8 AWU_period_s;
};

EEMEM const Settings defaultSettings = {
	.tempDividerMultipier = U1_15(1 * (1 << 15)),
	.tempHysteresisLow = U3_13(1.1 * (Adc_maxValue / 2)),
	.tempHysteresisHigh = U3_13(0.9 * (Adc_maxValue / 2)),
	.vcc_ref_mV = 2500,
	.vccAdcLow_mV = 3200,
	.statusLedBlinkShortDelay_ms = 150,
	.statusLedBlinkLongDelay_ms = 500,
	.AWU_period_s = 5,
};
Settings const& settings = ((Settings*)(&defaultSettings))[0];

RunningAvg<FU16[adcMaxBufferSize], FU32> vccRunningAvg;
RunningAvg<FU16[adcMaxBufferSize], FU32> TempDivider_runningAvg;

enum {
	ticksCountPerSAprox = 1000UL,

	TIM4_prescaler = 7,
	TIM4_arr = F_CPU / (1UL << TIM4_prescaler) / ticksCountPerSAprox,

	ticksCountPerSReal = F_CPU / (1UL << TIM4_prescaler) / TIM4_arr,
};

static_assert_lt(0, TIM4_arr);
static_assert_lte(TIM4_arr, U8(-1));

#define msToTicksCount(msArg) (FU32(ticksCountPerSReal) * (msArg) / 1000UL)

void Timer_init() {
	TIM4_PSCR = TIM4_prescaler;
	TIM4_ARR = TIM4_arr;

	setBit(TIM4_IER, TIM4_IER_UIE); // Enable Update Interrupt
}

volatile FU16 Timer_counter;
ISR(TIM4_ISR) {
	clearBit(TIM4_SR, TIM4_SR_UIF);
	Timer_counter -= 1;
}

void TIM4_delay_ms(FU16 ms){
	Timer_counter = ms;
	setBit(TIM4_CR1, TIM4_CR1_CEN); // Enable TIM4
	while(Timer_counter != 0) {
		wfiLowPowerMode();
	}
	clearBit(TIM4_CR1, TIM4_CR1_CEN); // Disable TIM4
}

void timerThread();
ISR(AWU_ISR) {
	//# Use __BRES intrisic instead clearBit. [http://www.count-zero.ru/2018/stm8_wakeup/]
	__BRES((unsigned char __near*)(&AWU_CSR), AWU_CSR_AWUF);

	timerThread();
}

struct Reporter {
	enum {
		ReportCount_lowVcc = 2,
		ReportCount_hwError = 4,
	};

	enum {
		Report_lowVcc = 0,
		Report_lowTemp,
		Report_hwError,
	};

	FU8 m_lastReport;
	FU8 m_status;

	Reporter() {
	}

	void clearStatus() {
		m_status = 0;
	}
	void pushLowVccReport() {
		setBit(m_status, Report_lowVcc);
	}
	void pushLowTempReport() {
		setBit(m_status, Report_lowTemp);
	}
	void pushHwErrorReport() {
		setBit(m_status, Report_hwError);
	}

	void blink(FU8 n, FU16 delayTime_ms) {
		statusLedPin.on();
		forInc(FU8, i, 0, 2 * n - 1) {
			delay_ms(delayTime_ms);
			statusLedPin.toggle();
		}
	}
	void blinkShort(FU8 n) {
		blink(n, settings.statusLedBlinkShortDelay_ms);
	}
	void blinkLong(FU8 n) {
		blink(n, settings.statusLedBlinkLongDelay_ms);
	}

	void reportLowVcc() {
		m_lastReport = Report_lowVcc;

		blinkShort(ReportCount_lowVcc);
	}
	void reportHwError() {
		m_lastReport = Report_hwError;

		blinkShort(ReportCount_hwError);
	}

	void reportLowTemp() {
		m_lastReport = Report_lowTemp;
		#if 0
		//# leave status led on
		statusLedPin.on();
		#else
		blinkLong(1);
		#endif // 0
	}
	void report() {
		if(m_status & _BV(Report_hwError)) {
			reportHwError();
		}
		else switch(m_status) {
			//# nothing to report
			case(0): {
				blinkShort(1);
			} break;
			//# report low vcc
			case(1): {
				reportLowVcc();
			} break;
			case(2): {
				reportLowTemp();
			} break;
			case(3): {
				if(m_lastReport == Report_lowTemp) {
					reportLowVcc();
				}
				else {
					reportLowTemp();
				}
			} break;
			default: {
				//# should not be here
			}
		}
	}

} reporter;

FU16 rrToRh(FU16 adc, FU16 rL) {
	return FU32(rL) * (Adc_maxValue - adc) / adc;
//	return rL * (Adc_maxValue / adc - 1);
}

Bool Temp_isLow = false;

void timerThread() {
	enterHighSpeedMode();

	//# fix. ADC is deinitializating after halt
	Adc_init();

	#if 0
	statusLedPin.toggle();
	delay_ms(200);
	return;
	#endif // 0

	#if 0
	reporter.clearStatus();

//	reporter.pushLowVccReport();
//	reporter.pushLowTempReport();
	reporter.pushHwErrorReport();

	reporter.report();

	return;
	#endif // 0

	#if 1
	statusLedPin.on();

	Bool isHwError = false;
	Adc_Value temp;

	batteryVccPin.on();
	vccRunningAvg.add(batteryAdcPin.readSync());
	batteryVccPin.off();

	tempDividerVccPin.on();
	Adc_checkWrongValue((temp = tempDividerAdcPin.readSync())) ? TempDivider_runningAvg.add(temp), 1 : (isHwError = true);
	tempDividerVccPin.off();

	reporter.clearStatus();

	#if 1
	if(isHwError) {
		reporter.pushHwErrorReport();
	};
	#endif // 1

	#endif // 1

	#if 1
	FU16 batteryVcc = FU32(settings.vcc_ref_mV) * Adc_maxValue / vccRunningAvg.computeAvg();
	if(batteryVcc <= settings.vccAdcLow_mV) {
		reporter.pushLowVccReport();
	};
	#endif // 1

	#if 1
	FU16 rDivider = ((FU32(TempDivider_runningAvg.computeAvg()) * settings.tempDividerMultipier) >> 15);

	FU16 rDividerLowThr = settings.tempHysteresisLow;
	FU16 rDividerHighThr = settings.tempHysteresisHigh;
	Bool isInvert = 1;
	if(rDividerHighThr < rDividerLowThr) {
		swap(rDividerHighThr, rDividerLowThr);
		isInvert = 0;
	};

	if(rDividerHighThr < rDivider && !Temp_isLow) {
		Temp_isLow = 1;
	};
	if(rDivider < rDividerLowThr && Temp_isLow) {
		Temp_isLow = 0;
	};
	if(Temp_isLow ^ isInvert) {
		reporter.pushLowTempReport();
	};
	#endif // 0

	reporter.report();
}

static void switch_to_lsi_auto() {
	// Enables Clock switch
	CLK_SWCR |= _BV(CLK_SWCR_SWEN);
	// Enable LSI
	CLK_SWR = 0xD2;

	while((CLK_SWCR & _BV(CLK_SWCR_SWBSY)) != 0 );
}
static void switch_to_lsi_manual() {
	// Enable LSI
	CLK_SWR = 0xD2;
	while((CLK_SWCR & _BV(CLK_SWCR_SWIF)) != 0 );
	// Enables Clock switch
	CLK_SWCR |= _BV(CLK_SWCR_SWEN);

}



void Clock_switchToLSI() {
	//# TODO enable it
//	setBit(CLK_ICKR, CLK_ICKR_REGAH);
//	setBit(CLK_ICKR, CLK_ICKR_LSIEN);
//	while((CLK_ICKR & _BV(CLK_ICKR_LSIRDY)) == 0);

	switch_to_lsi_manual();
	return;

	enum {
		CLK_SWCR_SWITCH_TO_HSI = 0xE1,
		CLK_SWCR_SWITCH_TO_LSI = 0xD2,
		CLK_SWCR_SWEN_BIT = 0x08,
	};

	CLK_SWCR = CLK_SWCR_SWITCH_TO_LSI;
	while(!(CLK_SWCR & (1 << CLK_SWCR_SWIF)));
	setBit(CLK_SWCR, CLK_SWCR_SWEN);
	wait(CLK_SWCR & _BV(CLK_SWCR_SWBSY));

	if(CLK_CMSR == CLK_SWCR_SWITCH_TO_HSI) {
		statusLedPin.on();
		while(1);
	};
	clearBit(CLK_ICKR, CLK_ICKR_HSIEN);
}

enum {
//	AWU_period_s = 5,
	AWU_freq_hz = 128000UL,
	//# see table #25 at [https://www.st.com/resource/en/reference_manual/cd00190271-stm8s-series-and-stm8af-series-8bit-microcontrollers-stmicroelectronics.pdf]
	AWU_clockSelectorValue = B1110,
	AWU_prescaler = 5LU * (1 << 11),
};

void AWU_init() {
	//# [http://www.count-zero.ru/2018/stm8_wakeup/]
	if(settings.AWU_period_s <= 5) {
		FU8 AWU_period_s = Math_max(settings.AWU_period_s, 2);

		AWU_TBR = B1110;
		//# TODO use only mul and shift. Try to use FU16 instead FU32
		//# TODO fix extra 2
		AWU_APR = (FU32(AWU_period_s) * AWU_freq_hz) / (5LU * (1 << 11));
	}
	else /* 6 - 30s */ {
		FU8 AWU_period_s = Math_min(settings.AWU_period_s, 30);

		AWU_TBR = B1111;
		//# TODO use only mul and shift. Try to use FU16 instead FU32
		AWU_APR = (FU32(AWU_period_s) * AWU_freq_hz) / (30LU * (1 << 11));
	}
	setBit_noBV(AWU_CSR, _BV(AWU_CSR_AWUEN));

	//# disable internal ldo in active-halt mode
	setBit(CLK_ICKR, CLK_ICKR_REGAH);
	//# turn off flash in active-halt mode
	setBit(FLASH_CR1, FLASH_CR1_AHALT);
}

static_print((FU32(10) * AWU_freq_hz) / (30LU * (1 << 11)));

void Hw_allPortsToZero() {
	//# TODO memset it
	(((GPIO_TypeDef*)PA_BASE_ADDRESS)->DDR =
	((GPIO_TypeDef*)PB_BASE_ADDRESS)->DDR =
	((GPIO_TypeDef*)PC_BASE_ADDRESS)->DDR =
	((GPIO_TypeDef*)PD_BASE_ADDRESS)->DDR =
		0xFF
	);

	#if 0
	(((GPIO_TypeDef*)PA_BASE_ADDRESS)->CR1 =
	((GPIO_TypeDef*)PB_BASE_ADDRESS)->CR1 =
	((GPIO_TypeDef*)PC_BASE_ADDRESS)->CR1 =
	((GPIO_TypeDef*)PD_BASE_ADDRESS)->CR1 =
		0xFF
	);
	#endif // 0

	//# done by default
	#if 1
	(((GPIO_TypeDef*)PA_BASE_ADDRESS)->ODR =
	((GPIO_TypeDef*)PB_BASE_ADDRESS)->ODR =
	((GPIO_TypeDef*)PC_BASE_ADDRESS)->ODR =
	((GPIO_TypeDef*)PD_BASE_ADDRESS)->ODR =
		0xFF
	);
	#endif // 0
}

#if 0
void Hw_configureInputPins() {
	((GPIO_TypeDef*)PD_BASE_ADDRESS)->DDR
}
#endif // 0

void Hw_enable() {
	enum {
		CLK_PCKENR1_TIM4 = 4
	};
  CLK_PCKENR1 = _BV(CLK_PCKENR1_TIM4);

	enum {
		CLK_PCKENR12_AWU = 2,
		CLK_PCKENR12_ADC = 3
	};
	CLK_PCKENR2 = _BV(CLK_PCKENR12_AWU) | _BV(CLK_PCKENR12_ADC);
}


void main() {
	Hw_enable();

	//# to avoid current leakage
	Hw_allPortsToZero();

//	Clock_switchToLSI();

	tempDividerVccPin.init();
	tempDividerAdcPin.init();
	batteryVccPin.init();
	batteryAdcPin.init();
	statusLedPin.init();

	#if 0
	statusLedPin.on();
	__halt();
	#endif // 0

	Adc_init();

	//# for debug purposes
	statusLedPin.on();

	Timer_init();
	enable_interrupts();

	#if 0
	while(1) {
//		statusLed.toggle();
		timerThread();
		delay_ms(1000);
	}
	#endif // 0

	AWU_init();

	while(1) {
//		wfiLowPowerMode();
		__halt();
	}
}
