/*
 * imx-3stack-wm8758.c  --  i.MX 3Stack Driver for Wolfson WM8758 Codec
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Copyright 2007-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    19th Jun 2007   Initial version.
 *
 */
/*
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/dma.h>
#include <mach/clock.h>

#include "../codecs/sgtl5000.h"
#include "imx-ssi.h"
#include "imx-pcm.h"
*/
#define DBG(...) printk("%s,%s,%d\n",__FILE__,__FUNCTION__,__LINE__);printk(__VA_ARGS__)
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
//#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/switch.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/slab.h>
#include <mach/dma.h>
//xnn debug #include <mach/spba.h>
#include <mach/clock.h>
#include <mach/mxc.h>

#include "../codecs/wm8758.h"
#include "imx-ssi.h"
#include "imx-pcm.h"

/* SSI BCLK and LRC master */
#define WM8758_SSI_MASTER	1

#define H2W_NO_DEVICE	0
#define H2W_HEADSET	1
static struct switch_dev *psdev;

struct imx_3stack_priv {
	int active;
	struct platform_device *pdev;
	int sysclk;
};

static struct imx_3stack_priv machine_priv;

extern void wm8758_mic_control(struct snd_soc_codec *codec,unsigned int reg,unsigned int value);
extern struct snd_soc_device *wm8758_socdev;

static int imx_3stack_audio_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{  
	printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct imx_3stack_priv *priv = &machine_priv;
	int ret = 0;
	//snd_pcm_format_t format = params_format(params);
	unsigned int channels = params_channels(params);
       printk("  %s(%d)   channels= %d   %s \n",__FILE__,__LINE__,channels,__func__);

        // add by panzidong
        struct imx_ssi *prive = (struct imx_ssi *)cpu_dai->private_data;
        printk("change manuel priv->netmode =1   %s(%d)  %s \n",__FILE__,__LINE__,__func__);
        if(channels ==2)
         prive->network_mode =1;
        else
         prive->network_mode =0;
        prive->sync_mode =1;
   
   
	u32 dai_format;
	/* only need to do this once as capture and playback are sync */
	if (priv->active)
		return 0;
	priv->active = 1;

#if WM8758_SSI_MASTER
	/* codec PLL input is 26MHz from MCLK */
        printk(" priv->sysclk = %d    %s(%d)    change by panzidong here for sample rates %s \n",priv->sysclk,__FILE__,__LINE__,__func__);
 //	snd_soc_dai_set_sysclk(codec_dai, WM8758_PLL, priv->sysclk/2, 0);
        snd_soc_dai_set_sysclk(codec_dai, WM8758_PLL, priv->sysclk, 0);
#endif

#if WM8758_SSI_MASTER
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM; 
	    //xnn debug | SND_SOC_DAIFMT_SYNC;
//	if (channels == 2)    //delete by panzidong
//		dai_format |= SND_SOC_DAIFMT_PDM; //xnn debug SND_SOC_DAIFMT_TDM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0){
                printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
		return ret;
         }
	/* set cpu DAI configuration */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		dai_format &= ~SND_SOC_DAIFMT_INV_MASK;
		/* Invert frame to switch mic from right channel to left */
                printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
		dai_format |= SND_SOC_DAIFMT_NB_IF;
	}
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0){
                printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
		return ret;
         }
	/* set codec BCLK to fs*64 */
	snd_soc_dai_set_clkdiv(codec_dai, WM8758_BCLKDIV, 0x2);

#else
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_SYNC;
	if (channels == 2)
		dai_format |= SND_SOC_DAIFMT_TDM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0){
                printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
		return ret;
        }
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0){
                printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
		return ret;
         }
#endif

	/* set i.MX active slot mask */
#if  1
//xnn debug 
        snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 2,32);   //change by panzidong;   2:32);   ORG;
#else
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels);
#endif

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

        // add by panzidong for mic
        printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
        //  wm8758_mic_control(wm8758_socdev->card->codec,0x01,0x3e);
        wm8758_mic_control(wm8758_socdev->card->codec,0x01,(1<<5)|(3<<0)|(1<<3)|(1<<4));
        wm8758_mic_control(wm8758_socdev->card->codec,0x02,0x1bf);
        wm8758_mic_control(wm8758_socdev->card->codec,0x2c,(1<<4)|(1<<5));
        wm8758_mic_control(wm8758_socdev->card->codec,0x30,(1<<8));
        wm8758_mic_control(wm8758_socdev->card->codec,0x2e,(1<<7)|(1<<8)|(0x3f));
        wm8758_mic_control(wm8758_socdev->card->codec,0x20,0xb8);
        //  add by panzidong for output two channels;
        wm8758_mic_control(wm8758_socdev->card->codec,0x31,0x62);
/*
        snd_soc_write(codec, 0x2c, (1<<4)|(1<<5));
        snd_soc_write(codec, 0x30, (1<<8));
        snd_soc_write(codec, 0x2e, (1<<7)|(1<<8)|(0x3f));
        snd_soc_write(codec,0x2c,0xbb);

*/

        printk("Freescale:  hw_paraw ok!!!! %s(%d)  %s \n",__FILE__,__LINE__,__func__);
	return 0;
}

static void imx_3stack_shutdown(struct snd_pcm_substream *substream)
{
	struct imx_3stack_priv *priv = &machine_priv;
	priv->active = 0;
}

/*
 * imx_3stack WM8758 HiFi DAI operations.
 */
static struct snd_soc_ops imx_3stack_ops = {
	.shutdown = imx_3stack_shutdown,
	.hw_params = imx_3stack_audio_hw_params,
};

static void imx_3stack_init_dam(int ssi_port, int dai_port)
{
	
	unsigned int ssi_ptcr = 0;
	unsigned int dai_ptcr = 0;
	unsigned int ssi_pdcr = 0;
	unsigned int dai_pdcr = 0;
	/* WM8758 uses SSI1 or SSI2 via AUDMUX port dai_port for audio */
        printk("  %s(%d)  ENTER   %s \n",__FILE__,__LINE__,__func__);
	/* reset port ssi_port & dai_port */
	__raw_writel(0, DAM_PTCR(ssi_port));
	__raw_writel(0, DAM_PTCR(dai_port));
	__raw_writel(0, DAM_PDCR(ssi_port));
	__raw_writel(0, DAM_PDCR(dai_port));

	/* set to synchronous */
	ssi_ptcr |= AUDMUX_PTCR_SYN;
	dai_ptcr |= AUDMUX_PTCR_SYN;

#if WM8758_SSI_MASTER
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TFSDIR;
	ssi_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, dai_port);

	/* set Tx Clock direction and source dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TCLKDIR;
	ssi_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, dai_port);
#else
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  ssi_port --> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TFSDIR;
	dai_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, ssi_port);

	/* set Tx Clock direction and source ssi_port--> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TCLKDIR;
	dai_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, ssi_port);
#endif

	__raw_writel(ssi_ptcr, DAM_PTCR(ssi_port));
	__raw_writel(dai_ptcr, DAM_PTCR(dai_port));
	__raw_writel(ssi_pdcr, DAM_PDCR(ssi_port));
	__raw_writel(dai_pdcr, DAM_PDCR(dai_port));
        printk("  %s(%d)  EXIT   %s \n",__FILE__,__LINE__,__func__);

}

static const struct snd_soc_dapm_route audio_map[] = {
	/* Mic 1 Jack --> MIC_IN_P/N */
	{"MIC_IN_P", NULL, "LMICP"},
	{"MIC_IN_N", NULL, "LMICN"},

	/* Mic 2 Jack --> MIC_IN_2 */
	{"MIC_IN_2", NULL, "RMICP"},

	/* Line in Jack --> AUX (L+R) */
	{"L2", NULL, "Line In Left"},
	{"R2", NULL, "Line In Right"},

	/* Out1 --> Headphone Jack */
	{"Headphone Jack", NULL, "RHP"},
	{"Headphone Jack", NULL, "LHP"},

	/* Out2 --> Line Out Jack */
	{"Line Out Jack", NULL, "RSPK"},
	{"Line Out Jack", NULL, "LSPK"},
};

static int wm8758_jack_func;
static int wm8758_spk_func;
/*
static void headphone_detect_handler(struct work_struct *work)
{
	struct imx_3stack_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	int hp_status;
	char *envp[3];
	char *buf;

	sysfs_notify(&pdev->dev.kobj, NULL, "headphone");
	hp_status = plat->hp_status();

*/
//extern void wm8758_mic_control(struct snd_soc_codec *codec,unsigned int reg,unsigned int value);
//extern struct snd_soc_device *wm8758_socdev;
static int cnt=0;
static void headphone_detect_handler(struct work_struct *work)
{
  

	struct imx_3stack_priv *priv = &machine_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	int hp_status;
	sysfs_notify(&pdev->dev.kobj, NULL, "headphone");
	hp_status =plat->hp_status();


//printk("cnt=%d,hp_status=%d\n",cnt,hp_status);
//cnt++;

//	if (hp_status) {  //  org    //change by panzidong
	if (!hp_status) {
		set_irq_type(plat->hp_irq, IRQ_TYPE_EDGE_FALLING);
        //      change by panzidong
        //        wm8758_spk_func = 1;
                 wm8758_mic_control(wm8758_socdev->card->codec,0x03,0x6f);
                  // add for  wm8758 record;
               //  wm8758_mic_control(wm8758_socdev->card->codec,0x2e,0x190);
                  
		switch_set_state(psdev, H2W_HEADSET);
	//	switch_set_state(psdev, H2W_NO_DEVICE);
	} else {
		set_irq_type(plat->hp_irq, IRQ_TYPE_EDGE_RISING);
        //      change by panzidong
                 wm8758_mic_control(wm8758_socdev->card->codec,0x03,0x0f);
		switch_set_state(psdev, H2W_NO_DEVICE);
        //    wm8758_spk_func = 0;
	//	switch_set_state(psdev, H2W_HEADSET);
	}
         // add by panzidong  
        if(1){
        char *envp[3];
        char *buf;
        buf = kmalloc(32, GFP_ATOMIC);
        if (!buf) {
                pr_err("%s kmalloc failed\n", __func__);
                return;
        	}
	 printk("  %s(%d)  %s  \n",__FILE__,__LINE__,__func__);

        if(hp_status ==1 ) 
        envp[0] = "NAME=headphone";
        else 
        envp[0] = "NAME=speaker";
        snprintf(buf, 32, "STATE=%d", hp_status);
        envp[1] = buf;
        envp[2] = NULL;
        kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
        }

    // deleted by panzidong      
   //	enable_irq(plat->hp_irq);
        printk("  %s(%d)  %s  \n\n\n",__FILE__,__LINE__,__func__);  
}

static DECLARE_DELAYED_WORK(hp_event, headphone_detect_handler);

static irqreturn_t imx_headphone_detect_handler(int irq, void *data)
{
//	printk("#############irqreturn_t imx_headphone_detect_handler\n");
//    deleted by panzidong
//	disable_irq(irq);
        printk("  %s(%d)  %s  \n",__FILE__,__LINE__,__func__);  
	 schedule_delayed_work(&hp_event, msecs_to_jiffies(200));
        printk("  %s(%d)  %s  \n",__FILE__,__LINE__,__func__);  
	return IRQ_HANDLED;
}

static ssize_t show_headphone(struct device_driver *dev, char *buf)
{
	struct imx_3stack_priv *priv = &machine_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	u16 hp_status;
	//printk("###########ssize_t show_headphone########\n");
  //      printk("  %s(%d)  %s  \n",__FILE__,__LINE__,__func__);
	/* determine whether hp is plugged in */
	hp_status = plat->hp_status();

	if (hp_status == 0)
		strcpy(buf, "speaker\n");
	else
		strcpy(buf, "headphone\n");

	return strlen(buf);
}

static DRIVER_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);

extern unsigned int wm8758_read_cache(unsigned int reg);
static ssize_t show_codec_regs(struct device_driver *dev, char *buf)
{
	char *p = buf;
	int i;
//	printk("#########show_codec_regs...........");
	for (i = 0; i < WM8758_CACHEREGNUM; i++) {
		p += sprintf(p, "[%02x]:%03x ", i, wm8758_read_cache(i));
		if (i % 6 == 0)
			p += sprintf(p, "\n");
	}
	*p = '\0';

	return strlen(buf);
}

static DRIVER_ATTR(regs, S_IRUGO | S_IWUSR, show_codec_regs, NULL);

static const char *jack_function[] = { "off", "on" };

static const char *spk_function[] = { "off", "on" };

static const struct soc_enum wm8758_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static int wm8758_get_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	printk("#########get jack...");
	ucontrol->value.enumerated.item[0] = wm8758_jack_func;
	return 0;
}

static int wm8758_set_jack(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	printk("#########set jack...");
	if (wm8758_jack_func == ucontrol->value.enumerated.item[0])
		return 0;

	wm8758_jack_func = ucontrol->value.enumerated.item[0];
	if (wm8758_jack_func)
		snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Headphone Jack");

	snd_soc_dapm_sync(codec);
	return 1;
}

static int wm8758_get_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	printk("#########get spk...");
	ucontrol->value.enumerated.item[0] = wm8758_spk_func;
	return 0;
}

static int wm8758_set_spk(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	printk("#########set spk...");
	if (wm8758_spk_func == ucontrol->value.enumerated.item[0])
		return 0;

	wm8758_spk_func = ucontrol->value.enumerated.item[0];
	if (wm8758_spk_func)
		snd_soc_dapm_enable_pin(codec, "Line Out Jack");
	else
		snd_soc_dapm_disable_pin(codec, "Line Out Jack");

	snd_soc_dapm_sync(codec);
	return 1;
}

static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("MIC_IN_P", NULL),
	SND_SOC_DAPM_MIC("MIC_IN_N", NULL),
	SND_SOC_DAPM_MIC("MIC_IN_2", NULL),
	SND_SOC_DAPM_LINE("Line In Left", NULL),
	SND_SOC_DAPM_LINE("Line In Right", NULL),
	SND_SOC_DAPM_SPK("Line Out Jack", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static const struct snd_kcontrol_new wm8758_machine_controls[] = {
	SOC_ENUM_EXT("Jack Function", wm8758_enum[0], wm8758_get_jack,
		     wm8758_set_jack),
	SOC_ENUM_EXT("Speaker Function", wm8758_enum[1], wm8758_get_spk,
		     wm8758_set_spk),
};

static int imx_3stack_wm8758_init(struct snd_soc_codec *codec)
{
	int i, ret;
//	printk("@@@@@@@@@@@@@@@@@imx_3stack_wm8758_init\n");
	/* Add imx_3stack specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8758_machine_controls); i++) {
		ret = snd_ctl_add(codec->card,
				  snd_soc_cnew(&wm8758_machine_controls[i],
					       codec, NULL));
		if (ret < 0)
			return ret;
	}

	/* Add imx_3stack specific widgets */
	snd_soc_dapm_new_controls(codec, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	/* Set up imx_3stack specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;

}
static struct snd_soc_dai *imx_3stack_cpu_dai;

/* imx_3stack digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "WM8758",
	.stream_name = "WM8758",
//	.cpu_dai = imx_3stack_cpu_dai,
	.codec_dai = &wm8758_dai,
	.init = imx_3stack_wm8758_init,
	.ops = &imx_3stack_ops,
};

/* imx_3stack audio machine driver */
//xnn debug
#if 1
static struct snd_soc_card snd_soc_card_imx_3stack = {
	.name = "imx-3stack",
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
	.platform = &imx_soc_platform,
};
/*static struct snd_soc_machine snd_soc_machine_imx_3stack = {
        .name = "imx-3stack",  //add
        .dai_link = &imx_3stack_dai,//  add
        .num_links = 1,    //add
};
*/
static struct snd_soc_device imx_3stack_snd_devdata = {
  //      .machine = &snd_soc_machine_imx_3stack,   //add 
	.card = &snd_soc_card_imx_3stack,
	.codec_dev = &soc_codec_dev_wm8758,
};
#else
static struct snd_soc_machine snd_soc_machine_imx_3stack = {
	.name = "imx-3stack",
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
};
static struct snd_soc_device imx_3stack_snd_devdata = {
	.machine = &snd_soc_machine_imx_3stack,
	.platform = &imx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8758,
};
#endif


ssize_t	h2w_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case H2W_NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case H2W_HEADSET:
		return sprintf(buf, "Headset\n");
	}
	return -EINVAL;
}
/*
static int __devinit imx_3stack_wm8580_probe(struct platform_device *pdev)
{
	struct wm8580_setup_data *setup;

	imx_3stack_dai.cpu_dai = &imx_esai_dai[2];
	imx_3stack_dai.cpu_dai->dev = &pdev->dev;

	setup = kzalloc(sizeof(struct wm8580_setup_data), GFP_KERNEL);
	setup->spi = 1;
	imx_3stack_snd_devdata.codec_data = setup;

	return 0;
}
*/
static int __devinit imx_3stack_wm8758_probe(struct platform_device *pdev)
{
	
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	
	struct imx_3stack_priv *priv = &machine_priv;
	int ret = 0;
	 
	priv->pdev = pdev;
	priv->sysclk = plat->sysclk;
        printk(" %s   %s(%d)  %s   \n",imx_ssi_dai[2]->name,__FILE__,__LINE__,__func__);
        //  init  imx_3stack_cpu_dai  and  imx_3stack_dai
        imx_3stack_cpu_dai = imx_ssi_dai[2];

        imx_3stack_dai.cpu_dai = imx_3stack_cpu_dai;
        //NOTE:  don't change  cpu_dai->priveate_date =  struct imx_ssi * 
//	imx_3stack_cpu_dai->private_data = plat;     

	imx_3stack_init_dam(plat->src_port, plat->ext_port);   
	if (plat->src_port == 2){
	      
                imx_3stack_cpu_dai->dev=&pdev->dev;
                
               printk(" imx_3stack_cpu_dai.name = %s     %s(%d)  %s \n",imx_3stack_cpu_dai->name, __FILE__,__LINE__,__func__);
               }
	else
		imx_3stack_cpu_dai->name =  "imx-ssi-1";
	ret = driver_create_file(pdev->dev.driver, &driver_attr_headphone);
	ret = driver_create_file(pdev->dev.driver, &driver_attr_regs);
	if (ret < 0) {
		pr_err("%s:failed to create driver_attr_headphone\n", __func__);
                printk("failed to create driver_attr_headphome  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
		return ret;
	}

	if (plat->init && plat->init())
		goto err_plat_init;
        
        printk("######### plat->hp_irq= %d , irq->name= %s   %s(%d)  %s   \n",plat->hp_irq,pdev->name,__FILE__,__LINE__,__func__);
	if (!plat->hp_status()){  //key point : change by panzidong   add (!)
		ret = request_irq(plat->hp_irq,
				  imx_headphone_detect_handler,
				  IRQ_TYPE_EDGE_FALLING, pdev->name, priv);
 //           wm8758_mic_control(wm8758_socdev->card->codec,0x03,0x6f);
        }
	else{
		ret = request_irq(plat->hp_irq,
				  imx_headphone_detect_handler,
				  IRQ_TYPE_EDGE_RISING, pdev->name, priv);
   //         wm8758_mic_control(wm8758_socdev->card->codec,0x03,0x0f);
         }
	if (ret < 0) {

                printk(" request irq failded  hp_detect ######  %s(%d)  %s   \n",__FILE__,__LINE__,__func__);
		pr_err("%s: request irq failed\n", __func__);
		goto err_irq;
	}

	wm8758_jack_func = 1;
	wm8758_spk_func = 1;
	psdev = kzalloc(sizeof(struct switch_dev), GFP_KERNEL);
	if (psdev == NULL) {
		ret = -ENOMEM;
		goto err_switch;
	}
	psdev->name = "h2w";
	psdev->print_name = h2w_print_name;
	ret = switch_dev_register(psdev);
	if (ret < 0) {
		pr_err("%s:failed to register switch device\n", __func__);
		goto err_switchdev;
	}

	if (plat->hp_status())
		switch_set_state(psdev, H2W_HEADSET);
	else
		switch_set_state(psdev, H2W_NO_DEVICE);
        printk(" imx-3stack-wm8758_probe OK! OK!!!!!!!!  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
	return 0;

err_switchdev:
        printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
	kfree(psdev);
err_switch:
        printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
	free_irq(plat->hp_irq, priv);
err_irq:
        printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
	if (plat->finit)
		plat->finit();
err_plat_init:
        printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);

	return ret;
}

static int imx_3stack_wm8758_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_3stack_priv *priv = &machine_priv;

	free_irq(plat->hp_irq, priv);

	if (plat->finit)
		plat->finit();

	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);

	switch_dev_unregister(psdev);
	kfree(psdev);

	return 0;
}

static struct platform_driver imx_3stack_wm8758_audio_driver = {
	.probe = imx_3stack_wm8758_probe,
	.remove = __devexit_p(imx_3stack_wm8758_remove),
	.driver = {
		   .name = "imx-3stack-wm8758",
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_init(void)
{
	int ret;
	
	ret = platform_driver_register(&imx_3stack_wm8758_audio_driver);
	if (ret)
		return -ENOMEM;

	imx_3stack_snd_device = platform_device_alloc("soc-audio", 2);
	if (!imx_3stack_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_3stack_snd_device, &imx_3stack_snd_devdata);
	imx_3stack_snd_devdata.dev = &imx_3stack_snd_device->dev;
	ret = platform_device_add(imx_3stack_snd_device);
       printk("  %s(%d)  %s  \n",__FILE__,__LINE__,__func__);
	if (ret)
		platform_device_put(imx_3stack_snd_device);
	if (ret)
               printk("register  soc-audio failed add bypanzidong  in  imx_3stack_init \n");
        printk("  %s(%d)  %s \n",__FILE__,__LINE__,__func__);
	return ret;
}

static void __exit imx_3stack_exit(void)
{
	platform_driver_unregister(&imx_3stack_wm8758_audio_driver);
	platform_device_unregister(imx_3stack_snd_device);
}

module_init(imx_3stack_init);
module_exit(imx_3stack_exit);

MODULE_AUTHOR("Liam Girdwood");
MODULE_DESCRIPTION("PMIC WM8758 Driver for i.MX 3STACK");
MODULE_LICENSE("GPL");
