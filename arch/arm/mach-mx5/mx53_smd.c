/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c/mpr.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/ahci_platform.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/da9052/da9052.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <linux/spi/flash.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/iomux-mx53.h>
#include <mach/i2c.h>
#include <mach/mxc_iim.h>
#include <mach/mxc_rfkill.h>
#include <mach/check_fuse.h>

#include "crm_regs.h"
#include "devices.h"
#include "usb.h"
#include "pmic.h"

/*!
 * @file mach-mx5/mx53_smd.c
 *
 * @brief This file contains MX53 smd board specific initialization routines.
 *
 * @ingroup MSL_MX53
 */
//xnn, use the macro to control zedi board driver     //add by panzidong     
#define MX53_ZEDI_VERSION1 1
#ifdef MX53_ZEDI_VERSION1
 #define WM8325_PMIC 1
//#define MX53_ZEDI_IRDA 0
#define MX53_ZEDI_X7 1      // add by panzidong
#endif


// add by panzidong #####start                 
#ifdef MX53_ZEDI_X7
/* MX53 ZEDI X7 GPIO PIN configurations */
#define MX53_SMD_GPIO_0			(0*32 + 0)	/* GPIO1_0 */
#define MX53_SMD_LCD_CONTRAST			(0*32 + 1)	/* GPIO1_1 */
#define MX53_SMD_HDMI_CEC_D			(0*32 + 2)	/* GPIO1_2 */
#define MX53_SMD_SATA_CLK_GPEN		(0*32 + 4)	/* GPIO1_4 */
#define MX53_SMD_PMIC_FAULT			(0*32 + 5)	/* GPIO1_5 */
#define MX53_SMD_SYS_ON_OFF_CTL		(0*32 + 7)	/* GPIO1_7 */
#define MX53_SMD_PMIC_ON_OFF_REQ	(0*32 + 8)	/* GPIO1_8 */

#define MX53_SMD_FEC_INT			(1*32 + 4)	/* GPIO2_4 */
#define MX53_SMD_LEFT_LED		(1*32 + 5)	/* GPIO2_5 */
#define MX53_SMD_RIGHT_LED			(1*32 + 6)	/* GPIO2_6 */
#define MX53_SMD_OSC_CHIH1_EN		(1*32 + 7)	/* GPIO2_7 */
#define MX53_SMD_MOD12V_CTL	(1*32 + 12)	/* GPIO2_12 */
#define MX53_SMD_WIFI_LED_G		(1*32 + 13)	/* GPIO2_13 */
#define MX53_SMD_SD1_CD		(1*32 + 14)	/* GPIO2_14 */
#define MX53_SMD_SD1_WP			(1*32 + 15)	/* GPIO2_15 */
#define MX53_SMD_FEC_PWR_EN			(1*32 + 16)	/* GPIO_2_16 */
#define MX53_SMD_GPIO2_RSV		(1*32 + 23)	/* GPIO_2_23 */
#define MX53_SMD_GPIO3_RSV		(1*32 + 24)	/* GPIO_2_24 */

#define MX53_SMD_LCD5V_EN		(1*32 + 25)	/* GPIO_2_25 */
#define MX53_SMD_TP_INT		(1*32 + 26)	/* GPIO_2_26 */
#define MX53_SMD_GPIO1_RSV		(1*32 + 30)	/* GPIO_2_30 */
#define MX53_SMD_GPIO2_31		(1*32 + 31)	/* GPIO_2_31 */

#define MX53_SMD_DCDC1V8_EN			(2*32 + 1)	/*GPIO_3_1 */
#define MX53_SMD_AUD_AMP_STBY_B		(2*32 + 2)	/*GPIO_3_2 */
#define MX53_SMD_SATA_PWR_EN		(2*32 + 3)	/*GPIO_3_3 */
#define MX53_SMD_TPM_OSC_EN			(2*32 + 4)	/*GPIO_3_4 */
// #define MX53_SMD_WLAN_PD			(2*32 + 5)	/*GPIO_3_5 */

#define MX53_SMD_WiFi_BT_PWR_EN		(0*32 + 8)	/*GPIO_1_8 */

#define MX53_SMD_RECOVERY_MODE_SW	(2*32 + 11)	/*GPIO_3_11 */
#define MX53_SMD_USB_OTG_OC			(2*32 + 12)	/*GPIO_3_12 */

#define MX53_SMD_SD1_CD				(1*32 + 14)	/*GPIO_2_14 */
#define MX53_SMD_USB_RST	(2*32 + 1)	/*GPIO_3_1 */
#define MX53_SMD_USB_HUB_RESET_B	(2*32 + 14)	/*GPIO_3_14 */
#define MX53_SMD_eCOMPASS_INT		(2*32 + 15)	/*GPIO_3_15 */

#define MX53_SMD_GPIO4_RSV		(2*32 + 16)	/* GPIO_3_16 */
#define MX53_SMD_GPIO5_RSV		(2*32 + 17)	/* GPIO_3_17 */
#define MX53_SMD_GPIO6_RSV		(2*32 + 18)	/* GPIO_3_18 */
#define MX53_SMD_CHR_STAT1		(2*32 + 19)	/* GPIO_3_19 */
#define MX53_SMD_CHR_STAT2	(2*32 + 20)	/* GPIO_3_20 */
#define MX53_SMD_BT_PRIORITY		(2*32 + 21)	/* GPIO_3_21 */
#define MX53_SMD_GPIO3_22			(2*32 + 22)	/* GPIO_3_22 */
// delete by panzidong  for uart2
//#define MX53_SMD_TPM_INT			(2*32 + 26)	/* GPIO_3_26 */
//#define MX53_SMD_MODEM_WKUP			(2*32 + 27)	/* GPIO_3_27 */
// delete by panzidong  for uart2
#define MX53_SMD_WLAN_HOST_WAKE			(2*32 + 28)	/* GPIO_3_28 */
#define MX53_SMD_CHR_PG			(2*32 + 29)	/* GPIO_3_29 */
#define MX53_SMD_3V3DCDC_EN		(2*32 + 30)	/* GPIO_3_30 */
#define MX53_SMD_USB_HOST_OC		(2*32 + 31)	/* GPIO_3_31 */

#define MX53_SMD_CHA_ISET			(3*32 + 2)	/* GPIO_4_2 */
#define MX53_SMD_SYS_EJECT			(3*32 + 3)	/* GPIO_4_3 */
#define MX53_SMD_BT_HOST_WAKE			(3*32 + 5)	/* GPIO_4_5 */
// #define MX53_SMD_HDMI_CEC_D			(3*32 + 4)	/* GPIO_4_4 */

#define MX53_SMD_GPIO4_15		(3*32 + 15)	/* GPIO_4_15 */

#define MX53_SMD_MODEM_DISABLE_B	(3*32 + 10)	/* GPIO_4_10 */

#define MX53_SMD_SD1_WP				(1*32 + 15)	/* GPIO_2_15 */

#define MX53_SMD_DCDC5V_BB_EN		(3*32 + 14)	/* GPIO_4_14 */
// #define MX53_SMD_WLAN_HOST_WAKE		(3*32 + 15)	/* GPIO_4_15 */

#define MX53_SMD_LCD3V3_EN	(4*32 + 0)	/* GPIO_5_0 */
#define MX53_SMD_3G_CARD_RST		(4*32 + 2)	/* GPIO_5_2 */
#define MX53_SMD_3G_CARD_EN			(4*32 + 4)	/* GPIO_5_4 */

#define MX53_SMD_CAP_TCH_FUN0		(5*32 + 6)	/* GPIO_6_6 */
#define MX53_SMD_WLAN_PD			(5*32 + 7)	/* GPIO_6_7 */
#define MX53_SMD_PANIC_KEY			(5*32 + 8)	/* GPIO_6_8 */
#define MX53_SMD_GPIO10_RSV			(5*32 + 9)	/* GPIO_6_9 */
#define MX53_SMD_WIFI3V3_EN			(5*32 + 10)	/* GPIO_6_10 */
#define MX53_SMD_DC_PLUGOUT	(5*32 + 11)	/* GPIO_6_11 */
#define MX53_SMD_AUD_AMP_STBYB			(5*32 + 12)	/* GPIO_6_12 */
#define MX53_SMD_3G3V3_EN			(5*32 + 13)	/* GPIO_6_13 */
#define MX53_SMD_MIC_DETECT			(5*32 + 14)	/* GPIO_6_14 */
#define MX53_SMD_USB_HUB_RST		(5*32 + 15)	/* GPIO_6_15 */
#define MX53_SMD_HEADPHONE_DETECT		(5*32 + 16)	/* GPIO_6_16 */
#define MX53_SMD_AC_IN				(5*32 + 17)	/* GPIO_6_17 */
#define MX53_SMD_PWR_GOOD			(5*32 + 18)	/* GPIO_6_18 */
#define MX53_SMD_GPIO7_RSV			(6*32 + 1)	/* GPIO_7_1 */
#define MX53_SMD_CABC_EN0			(6*32 + 2)	/* GPIO_7_2 */
#define MX53_SMD_DOCK_DECTECT		(6*32 + 3)	/* GPIO_7_3 */
#define MX53_SMD_FEC_RST			(6*32 + 6)	/* GPIO_7_6 */
#define MX53_SMD_USER_DEG_CHG_NONE	(6*32 + 7)	/* GPIO_7_7 */
#define MX53_SMD_USB_OTG_PWR_EN		(6*32 + 8)	/* GPIO_7_8 */
#define MX53_SMD_GPIO8_RSV		(6*32 + 9)	/* GPIO_7_9 */
#define MX53_SMD_PMIC_STBY		(6*32 + 11)	/* GPIO_7_11 */
#define MX53_SMD_GPIO8_RSV	(6*32 + 9)	/* GPIO_7_9 */
#define MX53_SMD_CAN_STBY			(6*32 + 10)	/* GPIO_7_10 */

#define MX53_SMD_PMIC_INT			(4*32 + 20)	/* GPIO_5_20 */



#define MX53_SMD_CAP_TCH_FUN1		(6*32 + 13)	/* GPIO_7_13 */

#else
// add by panzidong  #####end


/* MX53 SMD GPIO PIN configurations */
#define MX53_SMD_KEY_RESET			(0*32 + 2)	/* GPIO1_2 */
#define MX53_SMD_SATA_CLK_GPEN		(0*32 + 4)	/* GPIO1_4 */
#define MX53_SMD_PMIC_FAULT			(0*32 + 5)	/* GPIO1_5 */
#define MX53_SMD_SYS_ON_OFF_CTL		(0*32 + 7)	/* GPIO1_7 */
#define MX53_SMD_PMIC_ON_OFF_REQ	(0*32 + 8)	/* GPIO1_8 */

#define MX53_SMD_FEC_INT			(1*32 + 4)	/* GPIO2_4 */
#define MX53_SMD_HEADPHONE_DEC		(1*32 + 5)	/* GPIO2_5 */
#define MX53_SMD_ZIGBEE_INT			(1*32 + 6)	/* GPIO2_6 */
#define MX53_SMD_ZIGBEE_RESET_B		(1*32 + 7)	/* GPIO2_7 */


#define MX53_SMD_GPS_RESET_B		(1*32 + 12)	/* GPIO2_12 */
#define MX53_SMD_WAKEUP_ZIGBEE		(1*32 + 13)	/* GPIO2_13 */


#define MX53_SMD_KEY_VOL_DOWN		(1*32 + 14)	/* GPIO2_14 */
#define MX53_SMD_KEY_VOL_UP			(1*32 + 15)	/* GPIO2_15 */
#define MX53_SMD_FEC_PWR_EN			(1*32 + 16)	/* GPIO_2_16 */
#define MX53_SMD_LID_OPN_CLS_SW		(1*32 + 23)	/* GPIO_2_23 */
#define MX53_SMD_GPS_PPS		(1*32 + 24)	/* GPIO_2_24 */

#ifdef MX53_ZEDI_VERSION1 //xnn
#define MX53_SMD_GPIO2_25		(1*32 + 25)	/* GPIO_2_25 */
#define MX53_SMD_GPIO2_26		(1*32 + 26)	/* GPIO_2_26 */
#define MX53_SMD_GPIO2_30		(1*32 + 30)	/* GPIO_2_30 */
#define MX53_SMD_GPIO2_31		(1*32 + 31)	/* GPIO_2_31 */
#endif

#define MX53_SMD_DCDC1V8_EN			(2*32 + 1)	/*GPIO_3_1 */
#define MX53_SMD_AUD_AMP_STBY_B		(2*32 + 2)	/*GPIO_3_2 */
#define MX53_SMD_SATA_PWR_EN		(2*32 + 3)	/*GPIO_3_3 */
#define MX53_SMD_TPM_OSC_EN			(2*32 + 4)	/*GPIO_3_4 */
#define MX53_SMD_WLAN_PD			(2*32 + 5)	/*GPIO_3_5 */

//xnn
#ifdef MX53_ZEDI_VERSION1
#define MX53_SMD_WiFi_BT_PWR_EN		(0*32 + 8)	/*GPIO_1_8 */
#else
#define MX53_SMD_WiFi_BT_PWR_EN		(2*32 + 10)	/*GPIO_3_10 */
#endif

#define MX53_SMD_RECOVERY_MODE_SW	(2*32 + 11)	/*GPIO_3_11 */
#define MX53_SMD_USB_OTG_OC			(2*32 + 12)	/*GPIO_3_12 */

#ifdef MX53_ZEDI_VERSION1 //xnn
#define MX53_SMD_SD1_CD				(1*32 + 14)	/*GPIO_2_14 */
#else
#define MX53_SMD_SD1_CD				(2*32 + 13)	/*GPIO_3_13 */
#endif

#define MX53_SMD_USB_HUB_RESET_B	(2*32 + 14)	/*GPIO_3_14 */
#define MX53_SMD_eCOMPASS_INT		(2*32 + 15)	/*GPIO_3_15 */
#define MX53_SMD_CAP_TCH_INT1		(2*32 + 20)	/* GPIO_3_20 */
#define MX53_SMD_BT_PRIORITY		(2*32 + 21)	/* GPIO_3_21 */
#define MX53_SMD_ALS_INT			(2*32 + 22)	/* GPIO_3_22 */
//#define MX53_SMD_TPM_INT			(2*32 + 26)	/* GPIO_3_26 */
//#define MX53_SMD_MODEM_WKUP			(2*32 + 27)	/* GPIO_3_27 */
#define MX53_SMD_BT_RESET			(2*32 + 28)	/* GPIO_3_28 */
#define MX53_SMD_TPM_RST_B			(2*32 + 29)	/* GPIO_3_29 */
#define MX53_SMD_CHRG_OR_CMOS		(2*32 + 30)	/* GPIO_3_30 */
#define MX53_SMD_CAP_TCH_INT0		(2*32 + 31)	/* GPIO_3_31 */

#define MX53_SMD_CHA_ISET			(3*32 + 2)	/* GPIO_4_2 */
#define MX53_SMD_SYS_EJECT			(3*32 + 3)	/* GPIO_4_3 */
#define MX53_SMD_HDMI_CEC_D			(3*32 + 4)	/* GPIO_4_4 */

#ifdef MX53_ZEDI_VERSION1 //xnn
#define MX53_SMD_GPIO4_15		(3*32 + 15)	/* GPIO_4_15 */
#endif

#define MX53_SMD_MODEM_DISABLE_B	(3*32 + 10)	/* GPIO_4_10 */

#ifdef MX53_ZEDI_VERSION1 //xnn
#define MX53_SMD_SD1_WP				(1*32 + 15)	/* GPIO_2_15 */
#else
#define MX53_SMD_SD1_WP				(3*32 + 11)	/* GPIO_4_11 */
#endif

#define MX53_SMD_DCDC5V_BB_EN		(3*32 + 14)	/* GPIO_4_14 */
#define MX53_SMD_WLAN_HOST_WAKE		(3*32 + 15)	/* GPIO_4_15 */

#define MX53_SMD_HDMI_RESET_B		(4*32 + 0)	/* GPIO_5_0 */
#define MX53_SMD_MODEM_RESET_B		(4*32 + 2)	/* GPIO_5_2 */
#define MX53_SMD_KEY_INT			(4*32 + 4)	/* GPIO_5_4 */

#define MX53_SMD_CAP_TCH_FUN0		(5*32 + 6)	/* GPIO_6_6 */

#ifdef MX53_ZEDI_VERSION1
#define MX53_SMD_LED_G			(5*32 + 9)	/* GPIO_6_9 */
#else
#define MX53_SMD_CSI0_RST			(5*32 + 9)	/* GPIO_6_9 */
#endif

#define MX53_SMD_CSI0_PWN			(5*32 + 10)	/* GPIO_6_10 */
#define MX53_SMD_OSC_CKIH1_EN		(5*32 + 11)	/* GPIO_6_11 */
#define MX53_SMD_HDMI_INT			(5*32 + 12)	/* GPIO_6_12 */

#ifdef MX53_ZEDI_VERSION1
#define MX53_SMD_LED_R			(5*32 + 13)	/* GPIO_6_13 */
#else
#define MX53_SMD_LCD_PWR_EN			(5*32 + 13)	/* GPIO_6_13 */
#endif

#define MX53_SMD_ACCL_INT1_IN		(5*32 + 15)	/* GPIO_6_15 */
#define MX53_SMD_ACCL_INT2_IN		(5*32 + 16)	/* GPIO_6_16 */
#define MX53_SMD_AC_IN				(5*32 + 17)	/* GPIO_6_17 */
#define MX53_SMD_PWR_GOOD			(5*32 + 18)	/* GPIO_6_18 */


#define MX53_SMD_CABC_EN0			(6*32 + 2)	/* GPIO_7_2 */
#define MX53_SMD_DOCK_DECTECT		(6*32 + 3)	/* GPIO_7_3 */
#define MX53_SMD_FEC_RST			(6*32 + 6)	/* GPIO_7_6 */
#define MX53_SMD_USER_DEG_CHG_NONE	(6*32 + 7)	/* GPIO_7_7 */
#define MX53_SMD_USB_OTG_PWR_EN		(6*32 + 8)	/* GPIO_7_8 */
#define MX53_SMD_DEVELOP_MODE_SW	(6*32 + 9)	/* GPIO_7_9 */
#define MX53_SMD_CABC_EN1			(6*32 + 10)	/* GPIO_7_10 */

#ifdef MX53_ZEDI_VERSION1 //xnn
#define MX53_SMD_PMIC_INT			(5*32 + 0)	/* GPIO_6_0 */
#else
#define MX53_SMD_PMIC_INT			(6*32 + 11)	/* GPIO_7_11 */
#endif

#define MX53_SMD_CAP_TCH_FUN1		(6*32 + 13)	/* GPIO_7_13 */

#ifdef MX53_ZEDI_IRDA
#define MX53_ZEDI_IRDA_EN		(2*32 + 22)	/* GPIO_3_22 */
#endif
#endif     //add by panzidong
#define MX53_OFFSET					(0x20000000)
#define TZIC_WAKEUP0_OFFSET         (0x0E00)
#define TZIC_WAKEUP1_OFFSET         (0x0E04)
#define TZIC_WAKEUP2_OFFSET         (0x0E08)
#define TZIC_WAKEUP3_OFFSET         (0x0E0C)
#define GPIO7_0_11_IRQ_BIT			(0x1<<11)

#ifdef WM8325_PMIC
extern int __init mx53_smd_init_wm8325(void);
extern void wm831x_poweroff_system(void);
#else
extern int __init mx53_smd_init_da9052(void);
#endif

extern void pm_i2c_init(u32 base_addr);

// add by panzidong for sim9000a txd to gpio&txd switch
iomux_v3_cfg_t mx53_EIM_D24_gpio[] = {
	MX53_PAD_EIM_D24__GPIO3_24,
};

iomux_v3_cfg_t mx53_EIM_D24_uart3_txd[] = {
	MX53_PAD_EIM_D24__UART3_TXD_MUX,
};

iomux_v3_cfg_t mx53_EIM_D26_gpio[] = {
        MX53_PAD_EIM_D26__GPIO3_26,
};

iomux_v3_cfg_t mx53_EIM_D26_uart2_txd[] = {
       MX53_PAD_EIM_D26__UART2_TXD_MUX,
};

void  EIM_D24_GPIO_MODE(){
    mxc_iomux_v3_setup_multiple_pads(mx53_EIM_D24_gpio,ARRAY_SIZE(mx53_EIM_D24_gpio));
    #define EIM_D24_GPIO3_24   ( 2*32 + 24 )   /* GPIO_3_24 */
    gpio_request(EIM_D24_GPIO3_24,"GPIO3_24");
    gpio_direction_output(EIM_D24_GPIO3_24, 0);
    gpio_set_value(EIM_D24_GPIO3_24, 0);
    gpio_free(EIM_D24_GPIO3_24);
}

void  EIM_D24_UART_MODE(){
    mxc_iomux_v3_setup_multiple_pads(mx53_EIM_D24_uart3_txd,ARRAY_SIZE(mx53_EIM_D24_uart3_txd));

}

void  EIM_D26_GPIO_MODE(){
    mxc_iomux_v3_setup_multiple_pads(mx53_EIM_D26_gpio,ARRAY_SIZE(mx53_EIM_D26_gpio));
    #define EIM_D26_GPIO3_26  ( 2*32 + 26 )  /*GPIO_3_26*/
    gpio_request(EIM_D26_GPIO3_26,"GPIO3_26");
    gpio_direction_output(EIM_D26_GPIO3_26, 0);
    gpio_set_value(EIM_D26_GPIO3_26, 0);
    gpio_free(EIM_D26_GPIO3_26);
}

void  EIM_D26_UART_MODE(){
    mxc_iomux_v3_setup_multiple_pads(mx53_EIM_D26_uart2_txd,ARRAY_SIZE(mx53_EIM_D26_uart2_txd));
}


#ifdef MX53_ZEDI_VERSION1 //xnn
#ifdef MX53_ZEDI_X7     //add by panzidong   #####start
static iomux_v3_cfg_t mx53_smd_pads[] = {
      /*LCD5V_EN*/
       MX53_PAD_EIM_OE__GPIO2_25,
	/* LCD3v3_EN */
	MX53_PAD_EIM_WAIT__GPIO5_0,
	/*TP_INT*/
      	MX53_PAD_EIM_RW__GPIO2_26,

	/* GPIO1_RSV */
	//MX53_PAD_EIM_EB2__GPIO2_30,
        MX53_PAD_EIM_EB2__I2C2_SCL,

	/* GPIO2_RSV */
	MX53_PAD_EIM_CS0__GPIO2_23,
	/* GPIO3_RSV */
	MX53_PAD_EIM_CS1__GPIO2_24,
	/*3g card enable*/
	MX53_PAD_EIM_A24__GPIO5_4,
	/*3g card reset*/
	MX53_PAD_EIM_A25__GPIO5_2,

	/*GPIO4_RSV*/
         //MX53_PAD_EIM_D16__GPIO3_16,
         MX53_PAD_EIM_D16__I2C2_SDA,

	/*GPIO5_RSV*/
	MX53_PAD_EIM_D17__GPIO3_17,
	/*GPIO6_RSV*/
	MX53_PAD_EIM_D18__GPIO3_18,
	/*charger station 1*/
	MX53_PAD_EIM_D19__GPIO3_19,
	/*charger station 2*/
	MX53_PAD_EIM_D20__GPIO3_20,
	/*bluetooth priority*/
	MX53_PAD_EIM_D21__GPIO3_21,
	/*backup*/
	MX53_PAD_EIM_D22__GPIO3_22,
       /*wlan_host_wake*/
       MX53_PAD_EIM_D28__GPIO3_28,
	/*charger pg*/
	MX53_PAD_EIM_D29__GPIO3_29,
	/*3v3 dcdc en*/
	MX53_PAD_EIM_D30__GPIO3_30,
	/*usb_host_oc*/
	MX53_PAD_EIM_D31__GPIO3_31,
       /*22.5792 PCM crystal enable*/
	MX53_PAD_EIM_DA1__GPIO3_1,
	 /*usb_otg_oc*/
	MX53_PAD_EIM_DA12__GPIO3_12, 
       /*3G 3v3en */
	MX53_PAD_NANDF_RE_B__GPIO6_13,

	/* panic key keypad ,power key,by ljs*/
	MX53_PAD_NANDF_ALE__GPIO6_8,

	/* wlan_power suspend/resume */
	MX53_PAD_NANDF_CLE__GPIO6_7,
	/*GPIO10_RSV*/
	MX53_PAD_NANDF_WP_B__GPIO6_9,
	/*WIFI 3V3_EN*/
	MX53_PAD_NANDF_RB0__GPIO6_10,
	/* dc_plugout */
	MX53_PAD_NANDF_RE_B__GPIO6_13,
	/* mic_detect */
	MX53_PAD_NANDF_CS1__GPIO6_14,
	/* usb_hub_reset*/
	MX53_PAD_NANDF_CS2__GPIO6_15,
	/* headphone_detect */
	MX53_PAD_NANDF_CS3__GPIO6_16, 
	
       /*I2S AUD3*/
         //change by panzidog for 8758  i2s pin;
//	MX53_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC,  
//	MX53_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD, 
//	MX53_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS, 
//	MX53_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD, 
        /*I2S AUD5*/
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,
	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,


	/* UART1 */
	MX53_PAD_CSI0_DAT10__UART1_TXD_MUX,
	MX53_PAD_CSI0_DAT11__UART1_RXD_MUX,
	/* UART4 */
	MX53_PAD_CSI0_DAT12__UART4_TXD_MUX,
	MX53_PAD_CSI0_DAT13__UART4_RXD_MUX,
       /* UART5 */
        MX53_PAD_CSI0_DAT14__UART5_TXD_MUX,
	MX53_PAD_CSI0_DAT15__UART5_RXD_MUX,


	/*PMIC_INT*/
	MX53_PAD_CSI0_DATA_EN__GPIO5_20,
       /* UART2 */
	MX53_PAD_EIM_D27__UART2_RXD_MUX,
	MX53_PAD_EIM_D26__UART2_TXD_MUX,
        // delete by panzidong
        MX53_PAD_EIM_D28__UART2_CTS,
//        MX53_PAD_EIM_D29__UART2_RTS,
        /*UART3*/
	MX53_PAD_EIM_D24__UART3_TXD_MUX,
	MX53_PAD_EIM_D25__UART3_RXD_MUX,
	MX53_PAD_EIM_EB3__UART3_RTS,
	MX53_PAD_EIM_D23__UART3_CTS,
	/* DISPLAY */


	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,


	/* FEC */
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
	/*I2S AUD5*/
       MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,

	/*KEYPAD 配置6个键盘，可以相应3x3,by ljs*/
       MX53_PAD_KEY_COL2__KPP_COL_2,
	MX53_PAD_KEY_ROW2__KPP_ROW_2,
	MX53_PAD_KEY_COL3__KPP_COL_3,
	MX53_PAD_KEY_ROW3__KPP_ROW_3,
	MX53_PAD_KEY_COL4__KPP_COL_4,
	MX53_PAD_KEY_ROW4__KPP_ROW_4,


		/* SD1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
	
       MX53_PAD_PATA_DATA14__GPIO2_14,
       MX53_PAD_PATA_DATA15__GPIO2_15,
	/* I2C2 */
     //	MX53_PAD_KEY_COL3__I2C2_SCL,
//	MX53_PAD_KEY_ROW3__I2C2_SDA,
	/* SD2 */
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
	/*GPIO7_RSV*/
	MX53_PAD_PATA_BUFFER_EN__GPIO7_1,
	/*GPIO8_RSV*/
	MX53_PAD_PATA_CS_0__GPIO7_9,
       /*CAN_STBY*/
	MX53_PAD_PATA_CS_1__GPIO7_10,
	/*FEC_NRST*/
	MX53_PAD_PATA_DA_0__GPIO7_6,
       /*USB_OTG_PWR_EN*/
	MX53_PAD_PATA_DA_1__GPIO7_7,


	/* SD3 */
	MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
	MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
	MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
	MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
	MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
	MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
	MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
	MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
	MX53_PAD_PATA_IORDY__ESDHC3_CLK,
	MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
	/* FEC_nINT */
	MX53_PAD_PATA_DATA4__GPIO2_4,
	/* LEFT_LED */
	MX53_PAD_PATA_DATA5__GPIO2_5,
	/* RIGHT_LED */
	MX53_PAD_PATA_DATA6__GPIO2_6,
	/*MOD12V_CTL */
	MX53_PAD_PATA_DATA12__GPIO2_12,
	/* WIFI_LED_G */
	MX53_PAD_PATA_DATA13__GPIO2_13,


	/* DI1_GPIO */
	MX53_PAD_PATA_DIOR__GPIO7_3,
	/*DI2_GPIO  */
	MX53_PAD_PATA_DIOW__GPIO6_17,
	/* DI3_GPIO */
	MX53_PAD_PATA_DMACK__GPIO6_18,
	/* DO_GPIO */
	MX53_PAD_PATA_DMARQ__GPIO7_0,



	/*WATCHDOG CHIPSET*/
	MX53_PAD_GPIO_0__GPIO1_0,
       /*BACKLIGHT PWM*/
         // change by panzidong for backlight
	//xnn debug  MX53_PAD_GPIO_1__PWM2_PWMO,
        MX53_PAD_GPIO_1__PWM2_PWMO,
       // MX53_PAD_GPIO_1__GPIO1_1,
	/* HDMI_CEC_D */
	MX53_PAD_GPIO_2__GPIO1_2,
	/*BACKUP LOCK OUT FOR AUDIO CODEC*/
	MX53_PAD_GPIO_5__CCM_CLKO,
	/*CAN1*/
	MX53_PAD_GPIO_7__CAN1_TXCAN,
	MX53_PAD_GPIO_8__CAN1_RXCAN,
//        MX53_PAD_KEY_COL2__CAN1_TXCAN,
//        MX53_PAD_KEY_ROW2__CAN1_RXCAN,
	/* I2C3 */
	MX53_PAD_GPIO_3__I2C3_SCL,
	MX53_PAD_GPIO_6__I2C3_SDA,
	/* GPIO9_PWM */
	MX53_PAD_GPIO_9__PWM1_PWMO,
	/* BACKUP FOR PMIC SUSPEND */
	MX53_PAD_GPIO_16__GPIO7_11,
      /*SPDIF*/
	MX53_PAD_GPIO_17__SPDIF_OUT1,
     /*USB_HOST_WAKE,BACKUP FOR BT*/
	MX53_PAD_GPIO_19__GPIO4_5,



};
#else    //  add by panzidong   #####mid
static iomux_v3_cfg_t mx53_smd_pads[] = {
	/* HDMI reset */
	MX53_PAD_EIM_WAIT__GPIO5_0,
	/* CSPI1 */
	MX53_PAD_EIM_EB2__ECSPI1_SS0,
	MX53_PAD_EIM_D16__ECSPI1_SCLK,
	MX53_PAD_EIM_D17__ECSPI1_MISO,
	MX53_PAD_EIM_D18__ECSPI1_MOSI,
	MX53_PAD_EIM_D19__ECSPI1_SS1,
	/* WLAN_PD */
	//MX53_PAD_EIM_DA5__GPIO3_5,
	/* DCDC5V_PWR_EN */
	MX53_PAD_EIM_DA10__GPIO3_10,
	/* RECOVERY_MODE_SW */
	//MX53_PAD_EIM_DA11__GPIO3_11,
	/* USB_OTG_OC */
	MX53_PAD_EIM_DA12__GPIO3_12,
	/* HDMI_INT */
	MX53_PAD_NANDF_WE_B__GPIO6_12,
	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
	/* UART1 */
	MX53_PAD_CSI0_DAT10__UART1_TXD_MUX,
	MX53_PAD_CSI0_DAT11__UART1_RXD_MUX,
        /* UART2 */
//      delete by panzidong
//	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
//	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,
#ifdef MX53_ZEDI_VERSION1
        MX53_PAD_EIM_D28__UART2_CTS,
	MX53_PAD_EIM_D29__UART2_RTS,
#endif
        /*UART3*/
	MX53_PAD_EIM_D24__UART3_TXD_MUX,
	MX53_PAD_EIM_D25__UART3_RXD_MUX,
	/* DISPLAY */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
	/* FEC */
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
	/* I2C2 */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
	/* SD2 */
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
	/* DEVELOP_MODE_SW */
	//MX53_PAD_PATA_CS_0__GPIO7_9,
	/* FEC_nRST */
	MX53_PAD_PATA_DA_0__GPIO7_6,
	/* USER_DEBUG_OR_CHARGER_DONE */
	//MX53_PAD_PATA_DA_1__GPIO7_7,
	/* USB_OTG_PWR_EN */
	MX53_PAD_PATA_DA_2__GPIO7_8,
	/* SD3 */
	MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
	MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
	MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
	MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
	MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
	MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
	MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
	MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
	MX53_PAD_PATA_IORDY__ESDHC3_CLK,
	MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
	/* FEC_nINT */
	MX53_PAD_PATA_DATA4__GPIO2_4,
	/* PWR_GOOD */
	//MX53_PAD_PATA_DMACK__GPIO6_18,
	/* I2C3 */
	MX53_PAD_GPIO_3__I2C3_SCL,
	MX53_PAD_GPIO_6__I2C3_SDA,
	/* wifi wow */
	MX53_PAD_GPIO_7__GPIO1_7,
	/* wifi on */
	MX53_PAD_GPIO_8__GPIO1_8,
	/* HDMI_CEC_D */
	//MX53_PAD_GPIO_14__GPIO4_4,
#ifdef MX53_ZEDI_VERSION1
	/* PMIC_INT */
	MX53_PAD_CSI0_DAT14__GPIO6_0,
#else
	/* PMIC_INT */
	MX53_PAD_GPIO_16__GPIO7_11,
#endif
	/* owire */
	MX53_PAD_GPIO_18__OWIRE_LINE,		
	MX53_PAD_GPIO_17__SPDIF_OUT1,
       //xnn
#ifdef MX53_ZEDI_VERSION1
        //COMMON gpio
       /*3.3V PWR en*/
	MX53_PAD_EIM_D30__GPIO3_30,
	/*usb host pwr en*/
	MX53_PAD_EIM_DA14__GPIO3_14,
	/*usb host oc*/
	MX53_PAD_EIM_D31__GPIO3_31,
	/*led_r*/
	MX53_PAD_NANDF_RE_B__GPIO6_13,
	/* led_g */
	MX53_PAD_NANDF_WP_B__GPIO6_9,
	/*common GPIO 4_14*/
	MX53_PAD_KEY_COL4__GPIO4_14,
	/*mod12V pwr ctrl*/
	MX53_PAD_PATA_DATA12__GPIO2_12,
	/*wifi_led_g*/
	MX53_PAD_PATA_DATA13__GPIO2_13,
        /*irda gpio*/
        MX53_PAD_EIM_D22__GPIO3_22,
        /*GPIO5_CLKO*/
        MX53_PAD_GPIO_5__CCM_CLKO,
        /*common gpio 6_17*/
        MX53_PAD_PATA_DIOW__GPIO6_17,
	/* common gpio 4_15 */
	MX53_PAD_KEY_ROW4__GPIO4_15,
        /* common gpio 2_16 */
	MX53_PAD_EIM_RW__GPIO2_26,
        /* common gpio 3_20 */
	MX53_PAD_EIM_D20__GPIO3_20,
        /* common gpio 3_21 */
	MX53_PAD_EIM_D21__GPIO3_21,
        /* common gpio 4_10 */
	MX53_PAD_KEY_COL2__GPIO4_10,
        /* common gpio 7_3 */
	MX53_PAD_PATA_DIOR__GPIO7_3,
         /* common gpio 2_24 */
	MX53_PAD_EIM_CS1__GPIO2_24,
        /* GPIO9_PWM */
	MX53_PAD_GPIO_9__PWM1_PWMO,
         /* common gpio 2_25 */
	MX53_PAD_EIM_OE__GPIO2_25,
         /* common gpio 4_15 */
	MX53_PAD_KEY_ROW4__GPIO4_15,
         /* common gpio 2_30 */
	MX53_PAD_EIM_EB2__GPIO2_30,
         /* common gpio 2_31 */
	MX53_PAD_EIM_EB3__GPIO2_31,
         /* common gpio 2_23 */
	MX53_PAD_EIM_CS0__GPIO2_23,
         /* common gpio 4_11 */
	MX53_PAD_KEY_ROW2__GPIO4_11,

        //sd1
	/* SD1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,

	MX53_PAD_PATA_DATA14__GPIO2_14,
       MX53_PAD_PATA_DATA15__GPIO2_15,
#endif
       //end
};
#endif  //add by panzidong #####end
#else
static iomux_v3_cfg_t mx53_smd_pads[] = {
	/* DI_VGA_HSYNC */
	MX53_PAD_EIM_OE__IPU_DI1_PIN7,
	/* HDMI reset */
	MX53_PAD_EIM_WAIT__GPIO5_0,
	/* DI_VGA_VSYNC */
	MX53_PAD_EIM_RW__IPU_DI1_PIN8,
	/* CSPI1 */
	MX53_PAD_EIM_EB2__ECSPI1_SS0,
	MX53_PAD_EIM_D16__ECSPI1_SCLK,
	MX53_PAD_EIM_D17__ECSPI1_MISO,
	MX53_PAD_EIM_D18__ECSPI1_MOSI,
	MX53_PAD_EIM_D19__ECSPI1_SS1,
	/* BT: UART3*/
	MX53_PAD_EIM_D24__UART3_TXD_MUX,
	MX53_PAD_EIM_D25__UART3_RXD_MUX,
	MX53_PAD_EIM_EB3__UART3_RTS,
	MX53_PAD_EIM_D23__UART3_CTS,
	/* LID_OPN_CLS_SW*/
	MX53_PAD_EIM_CS0__GPIO2_23,
	/* GPS_PPS */
	MX53_PAD_EIM_CS1__GPIO2_24,
	/* FEC_PWR_EN */
	MX53_PAD_EIM_A22__GPIO2_16,
	/* CAP_TCH_FUN0*/
	MX53_PAD_EIM_A23__GPIO6_6,
	/* KEY_INT */
	MX53_PAD_EIM_A24__GPIO5_4,
	/* MODEM_RESET_B */
	MX53_PAD_EIM_A25__GPIO5_2,
	/* CAP_TCH_INT1 */
	MX53_PAD_EIM_D20__GPIO3_20,
	/* BT_PRIORITY */
	MX53_PAD_EIM_D21__GPIO3_21,
	/* ALS_INT */
	MX53_PAD_EIM_D22__GPIO3_22,
	/* TPM_INT */
	MX53_PAD_EIM_D26__GPIO3_26,
	/* MODEM_WKUP */
//	MX53_PAD_EIM_D27__GPIO3_27,
	/* BT_RESET */
	MX53_PAD_EIM_D28__GPIO3_28,
	/* TPM_RST_B */
	MX53_PAD_EIM_D29__GPIO3_29,
	/* CHARGER_NOW_OR_CMOS_RUN */
	MX53_PAD_EIM_D30__GPIO3_30,
	/* CAP_TCH_INT0 */
	MX53_PAD_EIM_D31__GPIO3_31,
	/* DCDC1V8_EN */
	MX53_PAD_EIM_DA1__GPIO3_1,
	/* AUD_AMP_STBY_B */
	MX53_PAD_EIM_DA2__GPIO3_2,
	/* SATA_PWR_EN */
	MX53_PAD_EIM_DA3__GPIO3_3,
	/* TPM_OSC_EN */
	MX53_PAD_EIM_DA4__GPIO3_4,
	/* WLAN_PD */
	MX53_PAD_EIM_DA5__GPIO3_5,
	/* WiFi_BT_PWR_EN */
	MX53_PAD_EIM_DA10__GPIO3_10,
	/* RECOVERY_MODE_SW */
	MX53_PAD_EIM_DA11__GPIO3_11,
	/* USB_OTG_OC */
	MX53_PAD_EIM_DA12__GPIO3_12,
	/* SD1_CD */
	MX53_PAD_EIM_DA13__GPIO3_13,
	/* USB_HUB_RESET_B */
	MX53_PAD_EIM_DA14__GPIO3_14,
	/* eCOMPASS_IN */
	MX53_PAD_EIM_DA15__GPIO3_15,
	/* HDMI_INT */
	MX53_PAD_NANDF_WE_B__GPIO6_12,
	/* LCD_PWR_EN */
	MX53_PAD_NANDF_RE_B__GPIO6_13,
	/* CSI0_RST */
	MX53_PAD_NANDF_WP_B__GPIO6_9,
	/* CSI0_PWN */
	MX53_PAD_NANDF_RB0__GPIO6_10,
	/* OSC_CKIH1_EN */
	MX53_PAD_NANDF_CS0__GPIO6_11,
	/* ACCL_INT1_IN */
	MX53_PAD_NANDF_CS2__GPIO6_15,
	/* ACCL_INT2_IN */
	MX53_PAD_NANDF_CS3__GPIO6_16,
	/* AUDMUX */
	MX53_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC,
	MX53_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD,
	MX53_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS,
	MX53_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD,
	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
	/* UART1 */
	MX53_PAD_CSI0_DAT10__UART1_TXD_MUX,
	MX53_PAD_CSI0_DAT11__UART1_RXD_MUX,
	/* CSI0 */
	MX53_PAD_CSI0_DAT12__IPU_CSI0_D_12,
	MX53_PAD_CSI0_DAT13__IPU_CSI0_D_13,
	MX53_PAD_CSI0_DAT14__IPU_CSI0_D_14,
	MX53_PAD_CSI0_DAT15__IPU_CSI0_D_15,
	MX53_PAD_CSI0_DAT16__IPU_CSI0_D_16,
	MX53_PAD_CSI0_DAT17__IPU_CSI0_D_17,
	MX53_PAD_CSI0_DAT18__IPU_CSI0_D_18,
	MX53_PAD_CSI0_DAT19__IPU_CSI0_D_19,
	MX53_PAD_CSI0_VSYNC__IPU_CSI0_VSYNC,
	MX53_PAD_CSI0_MCLK__IPU_CSI0_HSYNC,
	MX53_PAD_CSI0_PIXCLK__IPU_CSI0_PIXCLK,
	/* DISPLAY */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
	/* FEC */
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
	/* AUDMUX5 */
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,
	/* MODEM_DISABLE_B */
	MX53_PAD_KEY_COL2__GPIO4_10,
	/* SD1_WP */
	MX53_PAD_KEY_ROW2__GPIO4_11,
	/* I2C2 */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
	/* DCDC5V_BB_EN */
	MX53_PAD_KEY_COL4__GPIO4_14,
	/* WLAN_HOST_WAKE */
	MX53_PAD_KEY_ROW4__GPIO4_15,
	/* SD1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
	/* SD2 */
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
	/* UART2 */
//	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
//	MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,
	/* DEVELOP_MODE_SW */
	MX53_PAD_PATA_CS_0__GPIO7_9,
	/* CABC_EN1 */
	MX53_PAD_PATA_CS_1__GPIO7_10,
	/* FEC_nRST */
	MX53_PAD_PATA_DA_0__GPIO7_6,
	/* USER_DEBUG_OR_CHARGER_DONE */
	MX53_PAD_PATA_DA_1__GPIO7_7,
	/* USB_OTG_PWR_EN */
	MX53_PAD_PATA_DA_2__GPIO7_8,
	/* SD3 */
	MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
	MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
	MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
	MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
	MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
	MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
	MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
	MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
	MX53_PAD_PATA_IORDY__ESDHC3_CLK,
	MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
	/* FEC_nINT */
	MX53_PAD_PATA_DATA4__GPIO2_4,
	/* HEADPHONE DET*/
	MX53_PAD_PATA_DATA5__GPIO2_5,
	/* ZigBee_INT*/
	MX53_PAD_PATA_DATA6__GPIO2_6,
	/* ZigBee_RESET_B */
	MX53_PAD_PATA_DATA7__GPIO2_7,
	/* GPS_RESET_B*/
	MX53_PAD_PATA_DATA12__GPIO2_12,
	/* WAKEUP_ZigBee */
	MX53_PAD_PATA_DATA13__GPIO2_13,
	/* KEY_VOL- */
	MX53_PAD_PATA_DATA14__GPIO2_14,
	/* KEY_VOL+ */
	MX53_PAD_PATA_DATA15__GPIO2_15,
	/* DOCK_DECTECT */
	MX53_PAD_PATA_DIOR__GPIO7_3,
	/* AC_IN */
	MX53_PAD_PATA_DIOW__GPIO6_17,
	/* PWR_GOOD */
	MX53_PAD_PATA_DMACK__GPIO6_18,
	/* CABC_EN0 */
	MX53_PAD_PATA_INTRQ__GPIO7_2,
	MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK,
	MX53_PAD_GPIO_1__PWM2_PWMO,
	/* KEY_RESET */
	MX53_PAD_GPIO_2__GPIO1_2,
	/* I2C3 */
	MX53_PAD_GPIO_3__I2C3_SCL,
	MX53_PAD_GPIO_6__I2C3_SDA,
	/* SATA_CLK_GPEN */
	MX53_PAD_GPIO_4__GPIO1_4,
	/* PMIC_FAULT */
	MX53_PAD_GPIO_5__GPIO1_5,
	/* SYS_ON_OFF_CTL */
	MX53_PAD_GPIO_7__GPIO1_7,
	/* PMIC_ON_OFF_REQ */
	MX53_PAD_GPIO_8__GPIO1_8,
	/* CHA_ISET */
	MX53_PAD_GPIO_12__GPIO4_2,
	/* SYS_EJECT */
	MX53_PAD_GPIO_13__GPIO4_3,
	/* HDMI_CEC_D */
	MX53_PAD_GPIO_14__GPIO4_4,
	/* PMIC_INT */
	MX53_PAD_GPIO_16__GPIO7_11,
	MX53_PAD_GPIO_17__SPDIF_OUT1,
	/* CAP_TCH_FUN1 */
	MX53_PAD_GPIO_18__GPIO7_13,
	/* LVDS */
	MX53_PAD_LVDS0_TX3_P__LDB_LVDS0_TX3,
	MX53_PAD_LVDS0_CLK_P__LDB_LVDS0_CLK,
	MX53_PAD_LVDS0_TX2_P__LDB_LVDS0_TX2,
	MX53_PAD_LVDS0_TX1_P__LDB_LVDS0_TX1,
	MX53_PAD_LVDS0_TX0_P__LDB_LVDS0_TX0,
	MX53_PAD_LVDS1_TX3_P__LDB_LVDS1_TX3,
	MX53_PAD_LVDS1_TX2_P__LDB_LVDS1_TX2,
	MX53_PAD_LVDS1_CLK_P__LDB_LVDS1_CLK,
	MX53_PAD_LVDS1_TX1_P__LDB_LVDS1_TX1,
	MX53_PAD_LVDS1_TX0_P__LDB_LVDS1_TX0,
};
#endif

#ifndef WM8325_PMIC //xnn
static void smd_da9053_irq_wakeup_only_fixup(void)
{
	void __iomem *tzic_base;
	tzic_base = ioremap(MX53_TZIC_BASE_ADDR, SZ_4K);
	if (NULL == tzic_base) {
		pr_err("fail to map MX53_TZIC_BASE_ADDR\n");
		return;
	}
	__raw_writel(0, tzic_base + TZIC_WAKEUP0_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP1_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP2_OFFSET);
	/* only enable irq wakeup for da9053 */
	__raw_writel(GPIO7_0_11_IRQ_BIT, tzic_base + TZIC_WAKEUP3_OFFSET);
	iounmap(tzic_base);
	pr_info("only da9053 irq is wakeup-enabled\n");
}
#endif

static void smd_suspend_enter(void)
{
#ifdef WM8325_PMIC //xnn
#else
	if (board_is_rev(BOARD_REV_4)) {
		smd_da9053_irq_wakeup_only_fixup();
		da9053_suspend_cmd_sw();
	} else {
		if (da9053_get_chip_version() !=
			DA9053_VERSION_BB)
			smd_da9053_irq_wakeup_only_fixup();

		da9053_suspend_cmd_hw();
	}
#endif
}

static void smd_suspend_exit(void)
{
#ifdef WM8325_PMIC //xnn
#else
	if (da9053_get_chip_version())
		da9053_restore_volt_settings();
#endif
}

static struct mxc_pm_platform_data smd_pm_data = {
	.suspend_enter = smd_suspend_enter,
	.suspend_exit = smd_suspend_exit,
};

static struct fb_videomode video_modes[] = {
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 27MHz */
	// "CLAA-WVGA", 57, 800, 480, 37037, 40, 60, 10, 10, 20, 10,     //change by panzidong   #####old
         "CLAA-WVGA", 60, 800, 480, 33300, 45, 211, 22, 23, 1, 1,       //change by panzidong   #####new
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	/* 800x480 @ 60 Hz , pixel clk @ 32MHz */
	"SEIKO-WVGA", 60, 800, 480, 29850, 89, 164, 23, 10, 10, 10,
	FB_SYNC_CLK_LAT_FALL,
	FB_VMODE_NONINTERLACED,
	0,},
	{
	/* 1600x1200 @ 60 Hz 162M pixel clk*/
	"UXGA", 60, 1600, 1200, 6172,
	304, 64,
	1, 46,
	192, 3,
	FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	0,},
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

#ifdef MX53_ZEDI_X7     //add by panzidong  #####start

#if 1
//by ljs
static u16 keymapping[25] = {
	//r0
	//c0,c1,c2,c3,c4
	KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,

	//r1  
	//c0,c1,c2,c3,c4 
	KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,

	//r2
	//c0,c1,c2,c3,c4
	KEY_RESERVED,
       KEY_RESERVED,
       KEY_HOME,
       KEY_RESERVED,
       KEY_VOLUMEUP,

	//r3
	//c0,c1,c2,c3,c4
	KEY_RESERVED,
       KEY_RESERVED,
       KEY_UP,
       KEY_MENU,
       KEY_VOLUMEDOWN,

	//r4
	//c0,c1,c2,c3,c4
	KEY_RESERVED,
       KEY_RESERVED,
       KEY_DOWN,
       KEY_RESERVED,
       KEY_BACK,
};
#else
//之前的配置。和原理图上的按键位置不匹配
static u16 keymapping[25] = {
	//r0
	//c0,c1,c2,c3,c4
	KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,

	//r1  
	//c0,c1,c2,c3,c4 
	KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,
       KEY_RESERVED,

	//r2
	//c0,c1,c2,c3,c4
	KEY_RESERVED,
       KEY_RESERVED,
       KEY_BACK,
       KEY_RESERVED,
       KEY_HOME,

	//r3
	//c0,c1,c2,c3,c4
       KEY_RESERVED,
       KEY_RESERVED,
       KEY_UP,
       KEY_MENU,
       KEY_VOLUMEUP,

	//r4
	//c0,c1,c2,c3,c4
       KEY_RESERVED,
       KEY_RESERVED,
       KEY_DOWN,
       KEY_RESERVED,
       KEY_VOLUMEDOWN,
};
#endif


static struct resource mxc_kpp_resources[] = {
	[0] = {
	       .start = MXC_INT_KPP,
	       .end = MXC_INT_KPP,
	       .flags = IORESOURCE_IRQ,
	       }
};

static struct keypad_data keypad_plat_data = {
	.rowmax = 5,
	.colmax = 5,
	.irq = MXC_INT_KPP,
	.learning = 0,
	.delay = 2,
	.matrix = keymapping,
};

#endif      //add by panzidong #####end

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 3,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.iram_enable = true,
//	.iram_size = 0x14000,   //   panzidong  debug   release space for soc pcm.
        .iram_size =13000,
	.reset = mx5_vpu_reset,
};

static struct fec_platform_data fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};


static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = NULL,
	.chipselect_inactive = NULL,
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
#ifdef WM8325_PMIC //xnn

        .reg_id = "DCDC2",
#else
	.reg_id = "DA9052_BUCK_CORE",
#endif
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
};

static struct mxc_bus_freq_platform_data bus_freq_data = {
#ifdef WM8325_PMIC //xnn
        .gp_reg_id = "DCDC2",
	.lp_reg_id = "DCDC1",
#else
	.gp_reg_id = "DA9052_BUCK_CORE",
	.lp_reg_id = "DA9052_BUCK_PRO",
#endif
};

static struct tve_platform_data tve_data = {
#ifdef WM8325_PMIC //xnn
        .dac_reg = "LDO1",
#else
	.dac_reg = "DA9052_LDO7",
#endif
	
};

static struct ldb_platform_data ldb_data = {
	.ext_ref = 1,
	.boot_enable = MXC_LDBDI1,
};

static void mxc_iim_enable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;

	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void mxc_iim_disable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;
	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}

static struct mxc_iim_data iim_data = {
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
        //add by panzidong   #####start
        [1] = {
	       .flags = IORESOURCE_MEM,
	       },
        [2] = {
	       .flags = IORESOURCE_MEM,
	       },
        // add by panzidong #####end
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	// .mode_str = "1024x768M-16@60",    // change by panzidong  #####old
         .mode_str =  "CLAA-WVGA",           // change by panzidong  #####new
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,   // change by panzidong   old=RGB666
	 .mode_str = "1024x768-16@60",             // change by panziodng   old="XGA"
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

extern int primary_di;
static int __init mxc_init_fb(void)
{
	if (!machine_is_mx53_smd())
		return 0;

	/*for zedi board, set default display as CLAA-WVGA*/
	if (primary_di < 0)
	//	primary_di = 1;      // change by panzidong #####old
                primary_di = 0;      // add by panzidong    #####new

	if (primary_di) {
		printk(KERN_INFO "DI1 is primary\n");
		/* DI1 -> DP-BG channel: */
		mxc_fb_devices[1].num_resources = 1;   // change by panzidong  old=ARRAY_SIZE(mxcfb_resources)
		mxc_fb_devices[1].resource = &mxcfb_resources[0];    // change by panzidong old=mxcfb_resource
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);

		/* DI0 -> DC channel: */
		mxc_fb_devices[0].num_resources = 1;   // add by panzidong
		mxc_fb_devices[0].resource = &mxcfb_resources[1];  // add by panzidong
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);
	} else {
		printk(KERN_INFO "DI0 is primary\n");

		/* DI0 -> DP-BG channel: */
		mxc_fb_devices[0].num_resources = 1;//   change by panzidong 0ld=ARRAY_SIZE(mxcfb_resources)
		mxc_fb_devices[0].resource =&mxcfb_resources[0];        // change by panzidong old=mxcfb_resources
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);

		/* DI1 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);
	}

	/*
	 * DI0/1 DP-FG channel:
	 */
      mxc_fb_devices[2].num_resources = 1;            // add by panzidong
	mxc_fb_devices[2].resource = &mxcfb_resources[2];   // add by panzidong
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
device_initcall(mxc_init_fb);

static void sii902x_hdmi_reset(void)
{
	#ifndef MX53_ZEDI_X7
	gpio_set_value(MX53_SMD_HDMI_RESET_B, 0);
	msleep(10);
	gpio_set_value(MX53_SMD_HDMI_RESET_B, 1);
	msleep(10);
	#endif
}

static struct mxc_lcd_platform_data sii902x_hdmi_data = {
	.reset = sii902x_hdmi_reset,
#ifdef WM8325_PMIC //xnn
        .analog_reg = "LDO4",  
#else
	.analog_reg = "DA9052_LDO2",
#endif
	.fb_id = "DISP3 BG",
	.boot_enable = 1,
};

static struct imxi2c_platform_data mxci2c_data = {
       .bitrate = 100000,
};

static struct mxc_camera_platform_data camera_data = {
#ifdef WM8325_PMIC //xnn
    	.analog_regulator = "LDO1",
	.core_regulator = "LDO9",
#else
	.analog_regulator = "DA9052_LDO7",
	.core_regulator = "DA9052_LDO9",
#endif
	.mclk = 24000000,
	.csi = 0,
};

static struct mxc_lightsensor_platform_data ls_data = {
	.rext = 700,	/* calibration: 499K->700K */
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	.type = "mma8451",
	.addr = 0x1C,
	 },
	{ 
	.type = "bq27x00-battery",
	.addr = 0x21,
	},
	{
	.type = "ov5642",
	.addr = 0x3C,
	.platform_data = (void *)&camera_data,
	 },
};

static u16 smd_touchkey_martix[4] = {
	KEY_BACK, KEY_HOME, KEY_MENU, KEY_POWER,    // add by panzidong
//EY_BACK, KEY_HOME, KEY_MENU, KEY_SEARCH,   // delete by panzidong
};

static struct mpr121_platform_data mpr121_keyboard_platdata = {
	.keycount = ARRAY_SIZE(smd_touchkey_martix),
	.vdd_uv = 3300000,
	.matrix = smd_touchkey_martix,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
// add by panzidong for MX53_SMD_ZEDI_VERSION_2  pcf6583
        {
         .type = "pcf8563",
         .addr = 0x51,   //0xa3 read and 0xa2 write
        },
// add by panzidong for MX53_SMD_ZEDI_VERSION_2  pcf8563
/*
	{
	 .type = "sgtl5000-i2c",
	 .addr = 0x0a,
	 },
	{
	.type = "mpr121_touchkey",
	.addr = 0x5a,
#ifndef MX53_ZEDI_X7           // add by panzidong #####start
	.irq = gpio_to_irq(MX53_SMD_KEY_INT),
#endif                        // add by panzidong  #####end
	.platform_data = &mpr121_keyboard_platdata,
	},
	{
	.type = "mag3110",
	.addr = 0x0e,
	.irq = gpio_to_irq(MX53_SMD_eCOMPASS_INT),
	},*/
};

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
}


static struct gpio_keys_button smd_buttons[] = {
#ifdef MX53_ZEDI_X7     // add by panzidong   #####start
	GPIO_BUTTON(MX53_SMD_PANIC_KEY, KEY_POWER, 1, "panic key", 0),
	//GPIO_BUTTON(MX53_SMD_DC_PLUGOUT, KEY_STOP, 1, "dc plug out", 0),
	#else           // add by panzidong  ######middle
	GPIO_BUTTON(MX53_SMD_KEY_VOL_UP, KEY_VOLUMEUP, 1, "volume-up", 0),
	GPIO_BUTTON(MX53_SMD_KEY_VOL_DOWN, KEY_VOLUMEDOWN, 1, "volume-down", 0),
	#endif          // add by panzidong  #####end
};


static struct gpio_keys_platform_data smd_button_data = {
	.buttons	= smd_buttons,
	.nbuttons	= ARRAY_SIZE(smd_buttons),
};

static struct platform_device smd_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &smd_button_data,
	}
};

static void __init smd_add_device_buttons(void)
{
	platform_device_register(&smd_button_device);
}
#else
static void __init smd_add_device_buttons(void) {}
#endif

static int p1003_ts_hw_status(void)
{
      #ifdef MX53_ZEDI_X7      // add by panzidong
      return 0;
      #else
	return gpio_get_value(MX53_SMD_CAP_TCH_INT1);
      #endif
}

static struct p1003_ts_platform_data p1003_ts_data = {
	.hw_status = p1003_ts_hw_status,
};
#if defined(CONFIG_INPUT_ADXL34X) || defined(CONFIG_INPUT_ADXL34X_MODULE)
#include <linux/input/adxl34x.h>
static const struct adxl34x_platform_data adxl34x_info = {
	.x_axis_offset = 0,
	.y_axis_offset = 0,
	.z_axis_offset = 0,
	.tap_threshold = 0x31,
	.tap_duration = 0x10,
	.tap_latency = 0x60,
	.tap_window = 0xF0,
	.tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN,
	.act_axis_control = 0xFF,
	.activity_threshold = 5,
	.inactivity_threshold = 3,
	.inactivity_time = 4,
	.free_fall_threshold = 0x7,
	.free_fall_time = 0x20,
	.data_rate = 0x8,
	.data_range = ADXL_FULL_RES,
 
	.ev_type = EV_ABS,
	.ev_code_x = ABS_X,		/* EV_REL */
	.ev_code_y = ABS_Y,		/* EV_REL */
	.ev_code_z = ABS_Z,		/* EV_REL */
 
	.ev_code_tap = {BTN_TOUCH, BTN_TOUCH, BTN_TOUCH}, /* EV_KEY x,y,z */
 
/*	.ev_code_ff = KEY_F,*/		/* EV_KEY */
/*	.ev_code_act_inactivity = KEY_A,*/	/* EV_KEY */
	.power_mode = ADXL_AUTO_SLEEP | ADXL_LINK,
	.fifo_mode = ADXL_FIFO_STREAM,
	.orientation_enable = ADXL_EN_ORIENTATION_3D,
	.deadzone_angle = ADXL_DEADZONE_ANGLE_10p8,
	.divisor_length = ADXL_LP_FILTER_DIVISOR_16,
	/* EV_KEY {+Z, +Y, +X, -X, -Y, -Z} */
	.ev_codes_orient_3d = {BTN_Z, BTN_Y, BTN_X, BTN_A, BTN_B, BTN_C},
};
#endif



static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
#ifdef MX53_ZEDI_X7      // add by panzidong  #####start
	{
	 .type = "wm8758",
	 .addr = 0x1a,
	 },
	{
	 .type = "max11801",
	 .addr = 0x48, // 0x49, //hardware pull down a0a1 to 0, so the address are: 10010a1a0
	 .irq  = gpio_to_irq(MX53_SMD_TP_INT),
	},
	{
		I2C_BOARD_INFO("adxl34x", 0x53),
		.irq = gpio_to_irq(MX53_SMD_HDMI_CEC_D),
		.platform_data = (void *)&adxl34x_info,
	},
#endif                   // add by panzidong #####end
	{
	.type = "sii902x",
#ifdef MX53_ZEDI_VERSION1
        .addr = 0x3b,
#else
	.addr = 0x39,
#endif
#ifndef MX53_ZEDI_X7
	.irq = gpio_to_irq(MX53_SMD_HDMI_INT),
#endif
	.platform_data = &sii902x_hdmi_data,
	},
	{
	.type = "p1003_fwv33",
	.addr = 0x41,
#ifndef MX53_ZEDI_X7
	.irq  = gpio_to_irq(MX53_SMD_CAP_TCH_INT1),
#endif
	.platform_data = &p1003_ts_data,
	},
	{
		.type = "egalax_ts",
		.addr = 0x4,
#ifndef MX53_ZEDI_X7
		.irq = gpio_to_irq(MX53_SMD_CAP_TCH_INT1),
#endif
	},
	{
	.type = "isl29023",
	.addr = 0x44,
#ifndef MX53_ZEDI_X7
	//.irq  = gpio_to_irq(MX53_SMD_ALS_INT),
#endif
	.platform_data = &ls_data,
	},
};

#if defined(CONFIG_MTD_MXC_M25P80) || defined(CONFIG_MTD_MXC_M25P80_MODULE)
static struct mtd_partition m25p32_partitions[] = {
	{
	.name = "bootloader",
	.offset = 0,
	.size = 0x00040000,
	},
	{
	.name = "kernel",
	.offset = MTDPART_OFS_APPEND,
	.size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data m25p32_spi_flash_data = {
	.name = "m25p32",
	.parts = m25p32_partitions,
	.nr_parts = ARRAY_SIZE(m25p32_partitions),
	.type = "m25p32",
};
#endif

static struct spi_board_info m25p32_spi1_board_info[] __initdata = {
#if defined(CONFIG_MTD_MXC_M25P80) || defined(CONFIG_MTD_MXC_M25P80_MODULE)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p32",           /* Name of spi_driver for this device */
		.max_speed_hz = 20000000,       /* max spi SCK clock speed in HZ */
		.bus_num = 1,                   /* Framework bus number */
		.chip_select = 1,               /* Framework chip select. */
		.platform_data = &m25p32_spi_flash_data,
	}
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(m25p32_spi1_board_info,
				ARRAY_SIZE(m25p32_spi1_board_info));
}

static void mx53_gpio_usbotg_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(MX53_SMD_USB_OTG_PWR_EN, 1);
	else
		gpio_set_value(MX53_SMD_USB_OTG_PWR_EN, 0);
}

#ifdef MX53_ZEDI_VERSION1 //xnn
static void mx53_gpio_usbhost1_driver_vbus(bool on)
{
	if (on)
		gpio_set_value(MX53_SMD_USB_HUB_RESET_B, 1);
	else
		gpio_set_value(MX53_SMD_USB_HUB_RESET_B, 0);
}
#endif

static int sdhc_write_protect(struct device *dev)
{
	int ret = 0;
#ifdef MX53_ZEDI_VERSION1 //xnn
	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(MX53_SMD_SD1_WP);
#else
	if (to_platform_device(dev)->id == 0)
		ret = 0;
#endif


	return ret;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;
#ifdef MX53_ZEDI_VERSION1 //xnn
	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(MX53_SMD_SD1_CD);
#else
	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(MX53_SMD_KEY_VOL_DOWN);
#endif

	return ret;
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA | MMC_CAP_DATA_DDR,  //
//        .caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,    // change to SDR mode  
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.clock_mmc = "esdhc_clk",
};


static int mxc_sgtl5000_amp_enable(int enable)
{
	gpio_request(MX53_SMD_AUD_AMP_STBY_B, "amp-standby");
	if (enable)
		gpio_direction_output(MX53_SMD_AUD_AMP_STBY_B, 1);
	else
		gpio_direction_output(MX53_SMD_AUD_AMP_STBY_B, 0);
	gpio_free(MX53_SMD_AUD_AMP_STBY_B);
	return 0;
}

static int headphone_det_status(void)
{
        return (gpio_get_value(MX53_SMD_HEADPHONE_DETECT) == 0);  // add by panzidong #####new

//eturn (gpio_get_value(MX53_SMD_HEADPHONE_DEC) == 0);            // change by panzidong #####old
}
#ifdef MX53_ZEDI_VERSION1       //  add by panzidong  #####start
static struct mxc_audio_platform_data wm8758_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,   //change by panzidong org ext_port =3;
	.hp_irq = gpio_to_irq(MX53_SMD_HEADPHONE_DETECT),
	.hp_status = headphone_det_status,
 	.sysclk = 24000000, 
        //add for ext_ram ;
        .ext_ram_rx=1,   
        .ext_ram_tx=1, 
    //   .ext_ram_clk = clk_get(NULL, "emi_fast_clk");  
};

static struct platform_device mxc_wm8758_device = {
	.name = "imx-3stack-wm8758",
	.dev = {
		// .release = mxc_nop_release,
		.platform_data = &wm8758_data,
		},
};

static void mxc_init_wm8758(void)
{
	platform_device_register(&mxc_wm8758_device);
}
#endif               //end by panzidong  #####end

static int mxc_sgtl5000_init(void);

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
#ifdef MX53_ZEDI_X7     // add by panzidong   #####start
	.hp_irq = gpio_to_irq(MX53_SMD_HEADPHONE_DETECT),
#endif                 // add by panzidong   #####end
//hp_irq = gpio_to_irq(MX53_SMD_HEADPHONE_DEC),     //delete by panzidong;
	.hp_status = headphone_det_status,
	.amp_enable = mxc_sgtl5000_amp_enable,
	.init = mxc_sgtl5000_init,
	.ext_ram_rx = 1,
};

static int mxc_sgtl5000_init(void)
{
	sgtl5000_data.sysclk = 22579200;
	return 0;
}

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
};

static struct mxc_asrc_platform_data mxc_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* Souce from CKIH1 for 44.1K */
	/* Source from CCM spdif_clk (24M) for 48k and 32k
	 * It's not accurate
	 */
	.spdif_clk_48000 = 1,
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct mxc_audio_platform_data spdif_audio_data = {
	.ext_ram_tx = 1,
};

static struct platform_device mxc_spdif_audio_device = {
	.name = "imx-spdif-audio-device",
};

static void mx53_smd_bt_reset(void)
{
#ifndef MX53_ZEDI_X7
	gpio_request(MX53_SMD_BT_RESET, "bt-reset");
	gpio_direction_output(MX53_SMD_BT_RESET, 0);
	/* pull down reset pin at least >5ms */
	mdelay(6);
	/* pull up after power supply BT */
	gpio_set_value(MX53_SMD_BT_RESET, 1);
	gpio_free(MX53_SMD_BT_RESET);
	msleep(100);
	/* Bluetooth need some time to reset */
#endif
}

static int mx53_smd_bt_power_change(int status)
{
	if (status)
		mx53_smd_bt_reset();

	return 0;
}

static struct platform_device mxc_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct mxc_bt_rfkill_platform_data mxc_bt_rfkill_data = {
	.power_change = mx53_smd_bt_power_change,
};

#if defined(CONFIG_BATTERY_MAX17085) || defined(CONFIG_BATTERY_MAX17085_MODULE)
static struct resource smd_batt_resource[] = {
	/*{
	.flags = IORESOURCE_IO,
	.name = "pwr-good",
	.start = MX53_SMD_PWR_GOOD,
	.end = MX53_SMD_PWR_GOOD,
	},
	{
	.flags = IORESOURCE_IO,
	.name = "ac-in",
	.start = MX53_SMD_AC_IN,
	.end = MX53_SMD_AC_IN,
	},*/
	{
	.flags = IORESOURCE_IO,
	.name = "charge-now",
#ifndef MX53_ZEDI_X7
	.start = MX53_SMD_CHRG_OR_CMOS,
	.end = MX53_SMD_CHRG_OR_CMOS,
#endif
	},
	{
	.flags = IORESOURCE_IO,
	.name = "charge-done",
	.start = MX53_SMD_USER_DEG_CHG_NONE,
	.end = MX53_SMD_USER_DEG_CHG_NONE,
	},
};

static struct platform_device smd_battery_device = {
	.name           = "max17085_bat",
	.resource	= smd_batt_resource,
	.num_resources  = ARRAY_SIZE(smd_batt_resource),
};

static void __init smd_add_device_battery(void)
{
	platform_device_register(&smd_battery_device);
}
#else
static void __init smd_add_device_battery(void)
{
}
#endif

#ifdef MX53_ZEDI_IRDA //xnn
static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

#define MX53_SMD_IRDA_GPIO	IOMUX_TO_GPIO(MX53_ZEDI_IRDA_EN)
static struct mxc_irda_platform_data mxc_irda_data = {
	.gpio = MX53_SMD_IRDA_GPIO,
	.irda_irq = gpio_to_irq(MX53_ZEDI_IRDA_EN),
	.active_low = 1,
};

struct platform_device mxc_irda_device = {
	.name = "mxc_irda",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_irda_data,
	},
};

static void mxc_init_irda_device()
{
	platform_device_register(&mxc_irda_device);
}

#endif

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_1G;
	int left_mem = 0;
	int gpu_mem = SZ_128M;
	int fb_mem = SZ_32M;
	char *str;

	mxc_set_cpu_type(MXC_CPU_MX53);

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			break;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_nommu");
			if (str != NULL)
				gpu_data.enable_mmu = 0;

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (gpu_data.enable_mmu)
		gpu_mem = 0;

	if (left_mem == 0 || left_mem > total_mem)
		left_mem = total_mem - gpu_mem - fb_mem;

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		if (!gpu_data.enable_mmu) {
			gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
			gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
		}
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			mxcfb_resources[0].start =
				gpu_data.enable_mmu ?
				mem_tag->u.mem.start + left_mem :
				gpu_device.resource[5].end + 1;
			mxcfb_resources[0].end =
				mxcfb_resources[0].start + fb_mem - 1;
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
		}
#endif
	}
}

static void __init mx53_smd_io_init(void)
{

        // add for WXCJ by panzidong
        //EIM_D24_GPIO_MODE();
        //EIM_D26_GPIO_MODE();

#ifdef MX53_ZEDI_VERSION1
     //  int i; //xnn test
#endif
	mxc_iomux_v3_setup_multiple_pads(mx53_smd_pads,
					ARRAY_SIZE(mx53_smd_pads));
//#ifdef MX53_ZEDI_VERSION1     // change by panzidong ######start
#ifdef MX53_ZEDI_X7      
        /*xnn  3.3V enable*/
	//gpio_request(MX53_SMD_CHRG_OR_CMOS, "3v3-en");
	//gpio_direction_output(MX53_SMD_CHRG_OR_CMOS, 1);
	//gpio_set_value(MX53_SMD_CHRG_OR_CMOS, 1);
        gpio_request(MX53_SMD_3V3DCDC_EN, "3v3-en");
	gpio_direction_output(MX53_SMD_3V3DCDC_EN, 1);
	gpio_set_value(MX53_SMD_3V3DCDC_EN, 1);
	mdelay(5);
#endif
#ifdef MX53_ZEDI_X7
        /*xnn  enable  lcd 5V analog, 3v3 digital, contrast as gpio high*/
	//gpio_request(MX53_SMD_LCD5V_EN, "lcd5v-en");
	//gpio_direction_output(MX53_SMD_LCD5V_EN, 1);
	//gpio_set_value(MX53_SMD_LCD5V_EN, 1);
	//mdelay(1);
	//gpio_request(MX53_SMD_LCD3V3_EN, "lcd3v3-en");
	//gpio_direction_output(MX53_SMD_LCD3V3_EN, 1);
	//gpio_set_value(MX53_SMD_LCD3V3_EN, 1);
	mdelay(1);
	gpio_request(MX53_SMD_LCD_CONTRAST,"lcdc-en");
	gpio_direction_output(MX53_SMD_LCD_CONTRAST, 1);
	gpio_set_value(MX53_SMD_LCD_CONTRAST, 1);
#endif                         // change by panzidong #####end
#ifdef MX53_ZEDI_VERSION1      
	/* SD1 CD */
	gpio_request(MX53_SMD_SD1_CD, "sd1-cd");
	gpio_direction_input(MX53_SMD_SD1_CD);
	/* SD1 WP */
	gpio_request(MX53_SMD_SD1_WP, "sd1-wp");
	gpio_direction_input(MX53_SMD_SD1_WP);
#endif
	/* reset FEC PHY */
	gpio_request(MX53_SMD_FEC_RST, "fec-rst");
	gpio_direction_output(MX53_SMD_FEC_RST, 0);
	gpio_set_value(MX53_SMD_FEC_RST, 0);
	msleep(1);
	gpio_set_value(MX53_SMD_FEC_RST, 1);

//#ifndef MX53_ZEDI_VERSION1//xnn disable     // add by panzidong #####old
	/* headphone_det_b */
//	gpio_request(MX53_SMD_HEADPHONE_DEC, "headphone-dec");
//	gpio_direction_input(MX53_SMD_HEADPHONE_DEC);
//#endif
        /* headphone_det_b */                 // add by panzidong #####new   
        //gpio_request(MX53_SMD_HEADPHONE_DETECT, "headphone-dec");
	//gpio_direction_input(MX53_SMD_HEADPHONE_DETECT);
        // add by panzidong for irq  dead;
        //gpio_free(MX53_SMD_HEADPHONE_DETECT);
	/* USB PWR enable */
	//gpio_request(MX53_SMD_USB_OTG_PWR_EN, "usb-pwr");  // add by panzidong #####old
        gpio_request(MX53_SMD_USB_OTG_PWR_EN, "usbotg-pwr");// add by panzidong #####new 
	gpio_direction_output(MX53_SMD_USB_OTG_PWR_EN, 0);

#ifdef MX53_ZEDI_VERSION1 
	//xnn usb host power
	gpio_request(MX53_SMD_USB_HUB_RESET_B, "usbhost-pwr");
	gpio_direction_output(MX53_SMD_USB_HUB_RESET_B, 0);
	//end
#endif

#ifndef MX53_ZEDI_VERSION1 //xnn disable  
	/* Enable MX53_SMD_DCDC1V8_EN */
	gpio_request(MX53_SMD_DCDC1V8_EN, "dcdc1v8-en");
	gpio_direction_output(MX53_SMD_DCDC1V8_EN, 1);
	gpio_set_value(MX53_SMD_DCDC1V8_EN, 1);
#endif
#ifndef MX53_ZEDI_X7  // add by panzidong #####start
	/* Enable OSC_CKIH1_EN for audio */
	gpio_request(MX53_SMD_OSC_CKIH1_EN, "osc-en");
	gpio_direction_output(MX53_SMD_OSC_CKIH1_EN, 1);
	gpio_set_value(MX53_SMD_OSC_CKIH1_EN, 1);
#endif
#ifndef MX53_ZEDI_X7
	/* Sii902x HDMI controller */
	gpio_request(MX53_SMD_HDMI_RESET_B, "disp0-pwr-en");
	gpio_direction_output(MX53_SMD_HDMI_RESET_B, 0);
	gpio_request(MX53_SMD_HDMI_INT, "disp0-det-int");
	gpio_direction_input(MX53_SMD_HDMI_INT);
#endif                //add by panzidong  #####end
#ifndef MX53_ZEDI_VERSION1 //xnn disable
	/* MPR121 capacitive button */
	//gpio_request(MX53_SMD_KEY_INT, "cap-button-irq");
	//gpio_direction_input(MX53_SMD_KEY_INT);
	//gpio_free(MX53_SMD_KEY_INT);

	/* Camera reset */
	gpio_request(MX53_SMD_CSI0_RST, "cam-reset");
	gpio_direction_output(MX53_SMD_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(MX53_SMD_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(MX53_SMD_CSI0_PWN, 1);
	msleep(1);
	gpio_set_value(MX53_SMD_CSI0_PWN, 0);
#endif
if(0){
	/* Enable WiFi/BT Power*/                           // add by panzidong  #####old
	//gpio_request(MX53_SMD_WiFi_BT_PWR_EN, "bt-wifi-pwren");
	//gpio_direction_output(MX53_SMD_WiFi_BT_PWR_EN, 1);
        gpio_request(MX53_SMD_WIFI3V3_EN, "wifi-3v3-pwren");// add by panzidong  #####new
	gpio_direction_output(MX53_SMD_WIFI3V3_EN, 1);
	gpio_set_value(MX53_SMD_WIFI3V3_EN, 1);  
//#ifndef MX53_ZEDI_VERSION1 //xnn disable   // change by panzidong  #####old
#ifdef MX53_ZEDI_X7                          // change by panzidong  #####new
	/* WiFi Power up sequence */
	gpio_request(MX53_SMD_WLAN_PD, "wifi-pd");
	gpio_direction_output(MX53_SMD_WLAN_PD, 1);
	mdelay(1);
	gpio_set_value(MX53_SMD_WLAN_PD, 0);
	mdelay(5);
	gpio_set_value(MX53_SMD_WLAN_PD, 1);
	gpio_free(MX53_SMD_WLAN_PD);
#endif
        }
         //add by panzidong for wifi power up sequence;
                 
        gpio_request(MX53_SMD_WIFI3V3_EN, "wifi-3v3-pwren");// wifi3v3_en  pull down
	gpio_direction_output(MX53_SMD_WIFI3V3_EN, 1);
	gpio_set_value(MX53_SMD_WIFI3V3_EN, 0);  

	gpio_request(MX53_SMD_WLAN_PD, "wifi-pd");     //wlan_pd   pull down
	gpio_direction_output(MX53_SMD_WLAN_PD, 1);
	gpio_set_value(MX53_SMD_WLAN_PD, 0);
        mdelay(1000);
        
	gpio_set_value(MX53_SMD_WIFI3V3_EN, 1);  //wifi3v3_en pull up
        mdelay(1000);
	gpio_set_value(MX53_SMD_WLAN_PD, 1);   // wlan_pd    pull up

#ifndef MX53_ZEDI_VERSION1 //xnn disable
	/* battery */
	gpio_request(MX53_SMD_AC_IN, "ac-in");
	gpio_direction_input(MX53_SMD_AC_IN);
	gpio_request(MX53_SMD_PWR_GOOD, "pwr-good");
	gpio_direction_input(MX53_SMD_PWR_GOOD);
	gpio_request(MX53_SMD_CHRG_OR_CMOS, "charger now");
	gpio_direction_output(MX53_SMD_CHRG_OR_CMOS, 0);
	gpio_request(MX53_SMD_USER_DEG_CHG_NONE, "charger done");
	gpio_direction_output(MX53_SMD_USER_DEG_CHG_NONE, 0);

	/* ambient light sensor */
	//gpio_request(MX53_SMD_ALS_INT, "als int");
	//gpio_direction_input(MX53_SMD_ALS_INT);

	gpio_request(MX53_SMD_LCD_PWR_EN, "lcd-pwr-en");
	gpio_direction_output(MX53_SMD_LCD_PWR_EN, 1);

	/* mag3110 magnetometer sensor */
        //delete by panzidong
//	gpio_request(MX53_SMD_eCOMPASS_INT, "ecompass int");
//	gpio_direction_input(MX53_SMD_eCOMPASS_INT);
#endif
#ifdef MX53_ZEDI_VERSION1   // add by panzidong  #####start
       //xnn
#if 0
	/*usb host oc*/
       //gpio3_31
	gpio_request(MX53_SMD_CAP_TCH_INT0, "usbhost-oc-int");
	gpio_direction_input(MX53_SMD_CAP_TCH_INT0);
#endif
	/*led_right*/
	//gpio_request(MX53_SMD_RIGHT_LED, "led-right-int");
	//gpio_direction_output(MX53_SMD_RIGHT_LED, 1);
	//gpio_set_value(MX53_SMD_RIGHT_LED, 1);
	/*led_left*/
	//gpio_request(MX53_SMD_LEFT_LED, "led-left-int");
	//gpio_direction_output(MX53_SMD_LEFT_LED, 1);
	//gpio_set_value(MX53_SMD_LEFT_LED, 1);
	/*wifi_wow*/
	//gpio4_15           // add by panzidong  #####end
	gpio_request(MX53_SMD_WLAN_HOST_WAKE, "wifi-wow-int");
	gpio_direction_input(MX53_SMD_WLAN_HOST_WAKE);
#endif
#ifdef MX53_ZEDI_IRDA
	gpio_request(MX53_ZEDI_IRDA_EN, "mxc_irda");
	gpio_direction_input(MX53_ZEDI_IRDA_EN);
#endif


//#ifdef MX53_ZEDI_VERSION1
#ifdef MX53_ZEDI_X7

	gpio_request(MX53_SMD_CAN_STBY,"can_stby");
	gpio_direction_output(MX53_SMD_CAN_STBY,1);
	gpio_set_value(MX53_SMD_CAN_STBY,1);
	gpio_free(MX53_SMD_CAN_STBY);



	// add by panzidong for uart422-uart485
	gpio_request(MX53_SMD_CHR_STAT1,"chr_stat1");
	gpio_direction_output(MX53_SMD_CHR_STAT1,0);
	gpio_set_value(MX53_SMD_CHR_STAT1,0);
	gpio_free(MX53_SMD_CHR_STAT1);

	// add by panzidong for uart422-uart485
	gpio_request(MX53_SMD_CHR_PG,"chr_pg");
	gpio_direction_output(MX53_SMD_CHR_PG,0);
	gpio_set_value(MX53_SMD_CHR_PG,0);
	gpio_free(MX53_SMD_CHR_PG);


	//GPIO1,
	// gpio_request(MX53_SMD_GPIO1_RSV, "GPIO1");
	//gpio_direction_output(MX53_SMD_GPIO1_RSV, 0);
	//gpio_set_value(MX53_SMD_GPIO1_RSV, 0);
	// add by panzidong for 422 switch 485
	// gpio_free(MX53_SMD_GPIO1_RSV);
	//GPIO2,
	//gpio_request(MX53_SMD_GPIO2_RSV, "GPIO2");
	//gpio_direction_output(MX53_SMD_GPIO2_RSV, 0);
	//gpio_set_value(MX53_SMD_GPIO2_RSV, 0);
	//GPIO3,
	//gpio_request(MX53_SMD_GPIO3_RSV, "GPIO3");
	//gpio_direction_output(MX53_SMD_GPIO3_RSV, 1);
	//gpio_set_value(MX53_SMD_GPIO3_RSV, 1);

	//GPIO4,
	// gpio_request(MX53_SMD_GPIO4_RSV, "GPIO4");
	//	gpio_direction_output(MX53_SMD_GPIO4_RSV, 0);
	//gpio_set_value(MX53_SMD_GPIO4_RSV, 0);
	// add by panzidong for 422 switch 485
	//  gpio_free(MX53_SMD_GPIO4_RSV);
	//GPIO5,

	//gpio_request(MX53_SMD_GPIO5_RSV, "GPIO5");
	//gpio_direction_output(MX53_SMD_GPIO5_RSV, 1);
	//gpio_set_value(MX53_SMD_GPIO5_RSV, 1);
        
	//GPIO6,
	gpio_request(MX53_SMD_GPIO6_RSV, "GPIO6");
	gpio_direction_output(MX53_SMD_GPIO6_RSV, 0);
	gpio_set_value(MX53_SMD_GPIO6_RSV, 0);
	//GPIO7,
	//gpio_request(MX53_SMD_GPIO7_RSV, "GPIO7");
	//gpio_direction_output(MX53_SMD_GPIO7_RSV, 1);
	//gpio_set_value(MX53_SMD_GPIO7_RSV, 1);
	//GPIO8,
	//change by panzidong  for usb host GPIO8_RSV  direction  input; 
	//gpio_request(MX53_SMD_GPIO8_RSV, "GPIO8");
	//gpio_direction_input(MX53_SMD_GPIO8_RSV);
	//gpio_free(MX53_SMD_GPIO8_RSV); 
	//	gpio_direction_output(MX53_SMD_GPIO8_RSV, 0);
	//	gpio_set_value(MX53_SMD_GPIO8_RSV, 0);
	//GPIO3_22,
	//gpio_request(MX53_SMD_GPIO3_22, "GPIO3_22");
	//gpio_direction_output(MX53_SMD_GPIO3_22, 1);
	//gpio_set_value(MX53_SMD_GPIO3_22, 1);
	//GPIO10,
	gpio_request(MX53_SMD_GPIO10_RSV, "GPIO10");
	gpio_direction_output(MX53_SMD_GPIO10_RSV, 0);
	gpio_set_value(MX53_SMD_GPIO10_RSV, 0);
	//GPIO6_13
	gpio_request(MX53_SMD_3G3V3_EN, "GPIO_3v3En");
	gpio_direction_output(MX53_SMD_3G3V3_EN, 1);
	gpio_set_value(MX53_SMD_3G3V3_EN, 1);

	//add by panzidong for MX53_BT_HOST_WAKE
	gpio_request(MX53_SMD_BT_HOST_WAKE,"BT_HOST_WAKE");
	gpio_direction_output(MX53_SMD_BT_HOST_WAKE,1);
	gpio_set_value(MX53_SMD_BT_HOST_WAKE,1);
	// add by panzidong for MX53 Touchscreen  irq gpio  direction input.
	gpio_request(MX53_SMD_TP_INT,"TouchScreen_Irq");
	gpio_direction_input(MX53_SMD_TP_INT);
	gpio_free(MX53_SMD_TP_INT);
#endif

}

// add by panzidong for can0
static void flexcan_xcvr_enable(int id, int en)
{
        static int pwdn;
        if (id < 0 || id > 1)
                return;
/*
        if (en) {
                if (!(pwdn++))
                        gpio_set_value(MX53_12V_EN, 1);

                if (id == 0) {
                        gpio_set_value(MX53_CAN1_EN1, 1);
                        gpio_set_value(MX53_CAN1_EN2, 1);
                } else {
                        gpio_set_value(MX53_CAN2_EN1, 1);
                        gpio_set_value(MX53_CAN2_EN2, 1);
                }

        } else {
                if (!(--pwdn))
                        gpio_set_value(MX53_12V_EN, 0);

                if (id == 0) {
                        gpio_set_value(MX53_CAN1_EN1, 0);
                        gpio_set_value(MX53_CAN1_EN2, 0);
                } else {
                        gpio_set_value(MX53_CAN2_EN1, 0);
                        gpio_set_value(MX53_CAN2_EN2, 0);
                }
        }*/
}


static struct flexcan_platform_data flexcan0_data = {
        .core_reg = NULL,
        .io_reg = NULL,
        .root_clk_id = "lp_apm", /*lp_apm is 24MHz */
      //    .root_clk_id = "can_clk", /*lp_apm is 24MHz */
        .xcvr_enable = flexcan_xcvr_enable,
        .br_clksrc = 1,
        .br_rjw = 2,
        .br_presdiv = 5,
        .br_propseg = 5,
        .br_pseg1 = 4,
        .br_pseg2 = 7,
        .bcc = 1,
        .srx_dis = 1,
        .smp = 1,
        .boff_rec = 1,
        .ext_msg = 1,
        .std_msg = 1,
};


// end by panzidong for can0

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "ssi_ext1_clk");
	/*
	 *ssi_ext1_clk was enbled in arch/arm/mach-mx5/clock.c, and it was kept
	 *open to provide clock for audio codec on i.Mx53 Quickstart, but MX53
	 *SMD board have no needs to do that, so we close it here
	 */
	clk_disable(mxc_ipu_data.csi_clk[0]);
	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);

	mxcsdhc1_device.resource[2].start = gpio_to_irq(MX53_SMD_SD1_CD);
	mxcsdhc1_device.resource[2].end = gpio_to_irq(MX53_SMD_SD1_CD);

	mxc_cpu_common_init();
	mx53_smd_io_init();

//#ifdef WM8325_PMIC         // change by panzidong   #####old
#ifdef MX53_ZEDI_VERSION1    // change by panzidong   #####new

        pm_power_off = wm831x_poweroff_system;//need to add WM8325 patch
#else
	/* power off by sending shutdown command to da9053*/
	pm_power_off = da9053_power_off;
#endif
        // add by panzidong for flxcan
        mxc_register_device(&mxc_flexcan0_device,&flexcan0_data);
        // add by panzidong for flxcan
	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c_data);

//#ifdef WM8325_PMIC        // change by panzidong   #####old
#ifdef MX53_ZEDI_VERSION1   // change by panzidong   #####new

        mx53_smd_init_wm8325();
#else
	mx53_smd_init_da9052();
#endif

	mxc_register_device(&mxc_rtc_device, NULL);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);

//#ifndef MX53_ZEDI_VERSION1 //xnn disable     // change by panzidong   #####old
#ifndef MX53_ZEDI_X7 //xnn disable             // change by panzidong   #####new

	mxc_register_device(&mxc_ldb_device, &ldb_data);
	mxc_register_device(&mxc_tve_device, &tve_data);
#endif

	if (!mxc_fuse_get_vpu_status())
		mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	if (!mxc_fuse_get_gpu_status())
		mxc_register_device(&gpu_device, &gpu_data);
	mxc_register_device(&mxcscc_device, NULL);
	mxc_register_device(&pm_device, &smd_pm_data);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&mxc_iim_device, &iim_data);

#ifndef MX53_ZEDI_VERSION1 //xnn disable

	mxc_register_device(&mxc_pwm2_device, NULL);
#endif
        
	mxc_register_device(&mxc_pwm2_device, NULL);
//      change by panzidong   for   backlight;
//	mxc_register_device(&mxc_pwm1_backlight_device, &mxc_pwm_backlight_data);
	mxc_register_device(&mxc_pwm2_backlight_device, &mxc_pwm_backlight_data);
#ifdef MX53_ZEDI_X7
	mxc_register_device(&mxc_keypad_device, &keypad_plat_data);
#endif
/* Register mmc3(eMMC) first, make it's device number be 0 to
	 * avoid device number change by hotplug in SD(mmc1) card */
        mxc_register_device(&mxcsdhc3_device, &mmc3_data);
	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	//mxc_register_device(&mxcsdhc3_device, &mmc3_data);

#ifdef MX53_ZEDI_VERSION1 //xnn disable

	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
#endif
#ifndef MX53_ZEDI_X7
      mxc_register_device(&mxc_ssi3_device, NULL);
#endif
	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);

#ifndef MX53_ZEDI_VERSION1 //xnn disable

	mxc_register_device(&ahci_fsl_device, &sata_data);
	//mxc_register_device(&imx_ahci_device_hwmon, NULL);  // delete new ;
	/* AHCI SATA PWR EN(DCDC_5V, DCDC_3V3_BB) on SATA bus */
	gpio_request(MX53_SMD_SATA_PWR_EN, "sata-pwr-en");
	gpio_direction_output(MX53_SMD_SATA_PWR_EN, 1);
#endif

	mxc_register_device(&mxc_fec_device, &fec_data);
	mxc_register_device(&mxc_ptp_device, NULL);
#ifndef MX53_ZEDI_VERSION1
	/* ASRC is only available for MX53 TO2.0 */
	if (mx53_revision() >= IMX_CHIP_REVISION_2_0) {
		mxc_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
		clk_put(mxc_asrc_data.asrc_core_clk);
		mxc_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
		clk_put(mxc_asrc_data.asrc_audio_clk);
		mxc_register_device(&mxc_asrc_device, &mxc_asrc_data);
	}
#endif
	spi_device_init();

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

#ifndef MX53_ZEDI_VERSION1 //xnn disable

	sgtl5000_data.ext_ram_clk = clk_get(NULL, "emi_fast_clk");
	clk_put(sgtl5000_data.ext_ram_clk);
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);
#else
//add wolfson driver here

      wm8758_data.ext_ram_clk = clk_get(NULL, "emi_fast_clk"); 
      mxc_register_device(&mxc_wm8758_device, &wm8758_data);  // add by panzidong #####new
#endif

	spdif_audio_data.ext_ram_clk = clk_get(NULL, "emi_fast_clk");
	clk_put(spdif_audio_data.ext_ram_clk);
	mxc_register_device(&mxc_spdif_audio_device, &spdif_audio_data);
        // change by panzidong
	//mx5_set_otghost_vbus_func(mx53_gpio_usbotg_driver_vbus);
        mx53_gpio_usbotg_driver_vbus(1);
	// end change by panzidong 
	mx5_usb_dr_init();

#ifdef MX53_ZEDI_VERSION1
       mx5_set_host1_vbus_func(mx53_gpio_usbhost1_driver_vbus);
#endif

	mx5_usbh1_init();
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);

//#ifndef MX53_ZEDI_VERSION1 //xnn disable
	mxc_register_device(&mxc_bt_rfkill, &mxc_bt_rfkill_data);

//not used the method

	smd_add_device_buttons();


#ifndef MX53_ZEDI_VERSION1
	smd_add_device_battery();
#endif
#ifdef MX53_ZEDI_IRDA
	mxc_init_irda_device();
#endif
#ifndef WM8325_PMIC
//	pm_i2c_init(I2C1_BASE_ADDR - MX53_OFFSET);
#endif
}

static void __init mx53_smd_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 22579200, 0);

	uart_clk = clk_get_sys("mxcintuart.0", NULL);
	early_console_setup(MX53_BASE_ADDR(UART1_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx53_smd_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_SMD data structure.
 */
MACHINE_START(MX53_SMD, "Freescale MX53 SMD Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
