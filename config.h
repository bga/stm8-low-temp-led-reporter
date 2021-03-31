//#pragma once

#define STM8S103
#define STM8S103F3

#if defined(STM8S001J3) || defined(STM8S101) || defined(STM8S103F3) || defined(STM8S003F3)
	#define F_CPU_PRESCALER 1
	// #define F_CPU 128000UL
	#define F_CPU 16000000UL /* HSI */ / 8 /* HSIDIV */
#elif defined(STM8L001) || defined(STM8L101)
	#define F_CPU_PRESCALER 1
	#define F_CPU 38000UL
#else
	#error Unsupported MCU 
#endif

#ifdef __ICCSTM8__
	#define override
#endif
