Little beacon compares target temperature vs ambient (both NTC thermistors). Result is feed to Schmitt trigger(hysteresis). User get 1 long blink or 1 short depends from result. Can be used to report when some object is colder or hotter than ambient. This project also attempt to reduce stm8s103f3/stm8s003f3 power as low as possible. Result is around 10 - 30mkA (2 years from single CR2032).
Extra features.
	report low battery voltage - 2 short blinks
	report sensor failure - 4 short blinks

Requires IAR STM8 for build
