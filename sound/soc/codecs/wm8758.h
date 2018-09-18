/*
 * wm8758.h		--  codec driver for WM8758
 *
 * Copyright 2009 Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __WM8758_H__
#define __WM8758_H__

/*
 * Register values.
 */
#define WM8758_RESET				0x00
#define WM8758_POWER_MANAGEMENT_1		0x01
#define WM8758_POWER_MANAGEMENT_2		0x02
#define WM8758_POWER_MANAGEMENT_3		0x03
#define WM8758_AUDIO_INTERFACE			0x04
#define WM8758_COMPANDING_CONTROL		0x05
#define WM8758_CLOCKING				0x06
#define WM8758_ADDITIONAL_CONTROL		0x07
#define WM8758_GPIO_CONTROL			0x08
#define WM8758_JACK_DETECT_CONTROL_1		0x09
#define WM8758_DAC_CONTROL			0x0A
#define WM8758_LEFT_DAC_DIGITAL_VOLUME		0x0B
#define WM8758_RIGHT_DAC_DIGITAL_VOLUME		0x0C
#define WM8758_JACK_DETECT_CONTROL_2		0x0D
#define WM8758_ADC_CONTROL			0x0E
#define WM8758_LEFT_ADC_DIGITAL_VOLUME		0x0F
#define WM8758_RIGHT_ADC_DIGITAL_VOLUME		0x10
#define WM8758_EQ1				0x12
#define WM8758_EQ2				0x13
#define WM8758_EQ3				0x14
#define WM8758_EQ4				0x15
#define WM8758_EQ5				0x16
#define WM8758_DAC_LIMITER_1			0x18
#define WM8758_DAC_LIMITER_2			0x19
#define WM8758_NOTCH_FILTER_1			0x1b
#define WM8758_NOTCH_FILTER_2			0x1c
#define WM8758_NOTCH_FILTER_3			0x1d
#define WM8758_NOTCH_FILTER_4			0x1e
#define WM8758_ALC_CONTROL_1			0x20
#define WM8758_ALC_CONTROL_2			0x21
#define WM8758_ALC_CONTROL_3			0x22
#define WM8758_NOISE_GATE			0x23
#define WM8758_PLL_N				0x24
#define WM8758_PLL_K1				0x25
#define WM8758_PLL_K2				0x26
#define WM8758_PLL_K3				0x27
#define WM8758_3D_CONTROL			0x29
#define WM8758_OUT4_ADC				0x2a
#define WM8758_BEEP_CONTROL			0x2b
#define WM8758_INPUT_CONTROL			0x2c
#define WM8758_LEFT_INP_PGA_CONTROL		0x2d
#define WM8758_RIGHT_INP_PGA_CONTROL		0x2e
#define WM8758_LEFT_ADC_BOOST_CONTROL		0x2f
#define WM8758_RIGHT_ADC_BOOST_CONTROL		0x30
#define WM8758_OUTPUT_CONTROL			0x31
#define WM8758_LEFT_MIXER_CONTROL		0x32
#define WM8758_RIGHT_MIXER_CONTROL		0x33
#define WM8758_LOUT1_HP_CONTROL			0x34
#define WM8758_ROUT1_HP_CONTROL			0x35
#define WM8758_LOUT2_SPK_CONTROL		0x36
#define WM8758_ROUT2_SPK_CONTROL		0x37
#define WM8758_OUT3_MIXER_CONTROL		0x38
#define WM8758_OUT4_MIXER_CONTROL		0x39

#define WM8758_CACHEREGNUM			59

/* Clock divider Id's */
enum wm8758_clk_id {
	WM8758_OPCLKRATE,
	WM8758_BCLKDIV,
};

enum wm8758_sysclk_src {
	WM8758_PLL,
	WM8758_MCLK
};

extern struct snd_soc_dai wm8758_dai;
extern struct snd_soc_codec_device soc_codec_dev_wm8758;

#endif	/* __WM8758_H__ */
