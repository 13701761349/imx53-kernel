/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


/*
 * mx53_smd_pmic_wm8325.c  --  i.MX53 SMD driver for pmic wm8325
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/wm831x/core.h>
#include <linux/mfd/wm831x/pdata.h>
#include <mach/irqs.h>
#include <mach/iomux-mx53.h>
#include <mach/gpio.h>


#define MX53_SMD_WM_8325_GPIO3		(2*32 + 29)	/* GPIO_3_29, DVS1 function */

/* VCC DC1 */
static struct regulator_init_data wm8325_dc1 = {
	.constraints = {
		.name = "DCDC1",
		.min_uV = 600000,
		.max_uV = 1300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* VDDGP DC2 */
static struct regulator_init_data wm8325_dc2 = {
	.constraints = {
		.name = "DCDC2",
		.min_uV =  600000,
		.max_uV = 1300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* VDDAL 1V3 DC3 */
static struct regulator_init_data wm8325_dc3 = {
	.constraints = {
		.name = "DCDC3",
		.min_uV = 850000,
		.max_uV = 3400000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* DDR_1V5 DC4 */
static struct regulator_init_data wm8325_dc4 = {
	.constraints = {
		.name = "DCDC4",
		.min_uV = 850000,
		.max_uV = 3400000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* TVDAC_2V75 LDO1 */
static struct regulator_init_data wm8325_ldo1 = {
	.constraints = {
		.name = "LDO1",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* VDD_REG_2V5 LDO2 */
static struct regulator_init_data wm8325_ldo2 = {
	.constraints = {
		.name = "LDO2",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* VUSB_2V5/LVDS_2V5/NVCC_XTAL_2V5 LDO3 */
static struct regulator_init_data wm8325_ldo3 = {
	.constraints = {
		.name = "LDO3",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* VDD_DIG_PLL 1V3 LDO4 */
static struct regulator_init_data wm8325_ldo4 = {
	.constraints = {
		.name = "LDO4",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* SATA_PHY_2V5 LDO5 */
static struct regulator_init_data wm8325_ldo5 = {
	.constraints = {
		.name = "LDO5",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* VDD_FUSE 3V3 LDO6 */
static struct regulator_init_data wm8325_ldo6 = {
	.constraints = {
		.name = "LDO6",
		.min_uV = 900000,
		.max_uV = 3300000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/*
   NVCC_EIM_MAIN/NVCC_SD1&2/
   NVCC_PATA/NVCC_GPIO/NVCC_FEC
   3V3 LDO7
*/
static struct regulator_init_data wm8325_ldo7 = {
	.constraints = {
		.name = "LDO7",
		.min_uV = 1000000,
		.max_uV = 3500000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/*
   NVCC_NANDF/NVCC_RESET/NVCC_CSI
   NVCC_JTAG/NVCC_CKIH/VDD_ANA_PLL
   1V8 LDO8
*/
static struct regulator_init_data wm8325_ldo8 = {
	.constraints = {
		.name = "LDO8",
		.min_uV = 1000000,
		.max_uV = 3500000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* SATA_1V3 LDO9 */
static struct regulator_init_data wm8325_ldo9 = {
	.constraints = {
		.name = "LDO9",
		.min_uV = 1000000,
		.max_uV = 3500000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* NVCC_LCD 2V775 LDO10*/
static struct regulator_init_data wm8325_ldo10 = {
	.constraints = {
		.name = "LDO10",
		.min_uV = 1000000,
		.max_uV = 3500000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};

/* NVCC_SRTC 1V3 LDO11*/
static struct regulator_init_data wm8325_ldo11 = {
	.constraints = {
		.name = "LDO11",
		.min_uV = 800000,
		.max_uV = 1550000,
		.always_on = 1,
		.boot_on = 1,
		.valid_modes_mask = 0,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
	},
};
// change by panzidong   
#define MX53_SMD_WM8325_IRQ			(4*32 + 20)	/* GPIO5_20 */
//#define MX53_SMD_WM8325_IRQ			(5*32 + 0)	/* GPIO6_0 */
//#define MX53_SMD_WM8325_IRQ			(2*32 + 14)	/* GPIO3_14 */
#define WM8325_I2C_ADDR				(0x68)
#define WM8325_I2C_DEVICE_NAME      "wm8325"

#define WM8325_GPIO1_CONFIG_VAL     (0x848a)
//0b 1000,0100,1000,1010: 
//input,no pull,single edge triggered,
//DBVDD power domain,active high,output comos,
//enable tri stated,
//hw enable1 input

#define WM8325_GPIO2_CONFIG_VAL     (0x848c)
//0b 1000,0100,1000,1100: 
//input,no pull,single edge triggered,
//DBVDD power domain,active high,output comos,
//enable tri stated,
//hw control1 input

#define WM8325_GPIO3_CONFIG_VAL     (0x8488)
//0b 1000,0100,1000,1000: 
//input,no pull,single edge triggered,
//DBVDD power domain,active high,output comos,
//enable tri stated,
//hw dvs1 input

#define WM831X_DC1_CONTROL_2_VAL    (0x0b00)
//0b 0000,1011,0000,0000
//DC1 hardware control source=hardware control 1 
//DC1 hardware control operationg mode=hysteretice mode(default,150mA)

#define WM831X_DC1_DVS_CONTROL_VAL  (0x1024)
//0b 0001,0000,0010,0100
//DC1 DVS control source=hardware DVS1
//0x24=0.6+(0x24-0x8)*0.0125V=0.95V

#define WM831X_DC2_CONTROL_2_VAL    (0x0b00)
//0b 0000,1011,0000,0000
//DC2 hardware control source=hardware control 1 
//DC2 hardware control operationg mode=hysteretice mode(default, 150mA)

#define WM831X_DC2_DVS_CONTROL_VAL  (0x101c)
//0b 0001,0000,0010,0100
//DC1 DVS control source=hardware DVS1
//0x1c=0.6+(0x1c-0x8)*0.0125V=0.85V

#define WM831X_DC3_CONTROL_2_VAL    (0x0b00)
//0b 0000,1011,0000,0000
//DC3 hardware control source=hardware control 1 
//DC3 hardware control operationg mode=hysteretice mode(default, 200mA)

#define WM831X_DC4_CONTROL_2_VAL    (0x0b00)
//0b 0000,1011,0000,0000
//DC4 hardware control source=hardware control 1 
//DC4 hardware control operationg mode=hysteretice mode(default, 200mA)

//#define WM831X_LDO1_CONTROL_VAL     (0x0A00)
#define WM831X_LDO1_CONTROL_VAL     (0x0900)
//0b 0000,1001,0000,0000
//LDO1 hardware control Source=hardware control 1
//LDO1 hardware control operating mode=turn converter off(ldo1<->tvdac_2V75, do not use it in zedi design)

#define WM831X_LDO2_CONTROL_VAL     (0x0A01)
//0b 0000,1010,0000,0001
//LDO2 hardware control Source=hardware control 1
//LDO2 hardware control operating mode=low power mode(ldo2<->vdd_reg_2V5)
//LDO2 low power mode select=1=20ma minimum current

#define WM831X_LDO3_CONTROL_VAL     (0x0A01)
//0b 0000,1010,0000,0001
//LDO3 hardware control Source=hardware control 1
//LDO3 hardware control operating mode=low power mode(ldo3<->vldo3out(vusb_2v5,lvds_2v5,nvcc_xtal_2v5))
//LDO3 low power mode select=1=20ma minimum current

#define WM831X_LDO4_CONTROL_VAL     (0x0900)
//0b 0000,1001,0000,0000
//LDO4 hardware control Source=hardware control 1
//LDO4 hardware control operating mode=turn converter off(ldo4<->vldo4out(wifi),shut down in suspend mode)

#define WM831X_LDO5_CONTROL_VAL     (0x0900)
//0b 0000,1001,0000,0000
//LDO5 hardware control Source=hardware control 1
//LDO5 hardware control operating mode=turn converter off(ldo5<->sata_phy_2v5, do not use it in zedi design)

#define WM831X_LDO6_CONTROL_VAL     (0x0900)
//0b 0000,1001,0000,0000
//LDO6 hardware control Source=hardware control 1
//LDO6 hardware control operating mode=turn converter off(ldo6<->vdd_fuse, can shut down in suspend mode)

#define WM831X_LDO7_CONTROL_VAL     (0x0A01)
//0b 0000,1010,0000,0001
//LDO7 hardware control Source=hardware control 1
//LDO7 hardware control operating mode=low power mode(ldo7<->vldo7_3v3(nvcc_nandf,nvcc_eim_main_1/2,nvcc_eim_sec,nvcc_sd1/2,nvcc_pata,nvcc_fec,nvcc_gpio,nvcc_keypad),vusb_3v3, do not use it in zedi design)
//LDO7 low power mode select=1=20ma minimum current

#define WM831X_LDO8_CONTROL_VAL     (0x0A01)
//0b 0000,1010,0000,0001
//LDO8 hardware control Source=hardware control 1
//LDO8 hardware control operating mode=turn converter off(ldo8<->vldo8_1V8(nvcc_reset,nvcc_csi,nvcc_jtag,nvcc_ckih))
//LDO8 low power mode select=1=20ma minimum current

#define WM831X_LDO9_CONTROL_VAL     (0x0900)
//0b 0000,1001,0000,0000
//LDO9 hardware control Source=hardware control 1
//LDO9 hardware control operating mode=turn converter off(ldo9<->sata_1V3, do not use it in zedi design)

#define WM831X_LDO10_CONTROL_VAL     (0x0900)
//0b 0000,1001,0000,0000
//LDO10 hardware control Source=hardware control 1
//LDO10 hardware control operating mode=turn converter off(ldo10<->vldo10_2V75(nvcc_lcd_1/2), do not use it in zedi design)



static int wm8325_post_init(struct wm831x *wm831x)
{
	int ret;

	/* Set GPIO1 as input ,active high, Hardware Enable1, Function */
	ret = wm831x_reg_write(wm831x, WM831X_GPIO1_CONTROL, \
			WM8325_GPIO1_CONFIG_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x gpio1 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set GPIO2 as input ,active high, Hardware Control1 Function */
	ret = wm831x_reg_write(wm831x, WM831X_GPIO2_CONTROL, \
			WM8325_GPIO2_CONFIG_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x gpio2 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set GPIO3 as input ,active high, DVS1 Function */
	ret = wm831x_reg_write(wm831x, WM831X_GPIO3_CONTROL, \
			WM8325_GPIO3_CONFIG_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x gpio3 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set DCDC1 hardware controlled by Hardware Control1 */
	ret = wm831x_reg_write(wm831x, WM831X_DC1_CONTROL_2, \
			WM831X_DC1_CONTROL_2_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x dc1 ctrl2 reg error.\n", __func__);
		goto out;
	}

	/* Set DCDC2 hardware controlled by Hardware Control1 */
	ret = wm831x_reg_write(wm831x, WM831X_DC2_CONTROL_2, \
			WM831X_DC2_CONTROL_2_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x dc2 ctrl2 reg error.\n", __func__);
		goto out;
	}

//xnn add
	/* Set DCDC3 hardware controlled by Hardware Control1 */
	ret = wm831x_reg_write(wm831x, WM831X_DC3_CONTROL_2, \
			WM831X_DC3_CONTROL_2_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x dc3 ctrl2 reg error.\n", __func__);
		goto out;
	}

	/* Set DCDC4 hardware controlled by Hardware Control1 */
	ret = wm831x_reg_write(wm831x, WM831X_DC4_CONTROL_2, \
			WM831X_DC4_CONTROL_2_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x dc4 ctrl2 reg error.\n", __func__);
		goto out;
	}
//xnn add end

	/* Set DCDC1 controlled by DVS1, DC1_DVS_VSEL=0.95V */
	ret = wm831x_reg_write(wm831x, WM831X_DC1_DVS_CONTROL, \
			WM831X_DC1_DVS_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x dc1 dvs reg error.\n", __func__);
		goto out;
	}

	/* Set DCDC2 controlled by DVS1, DC2_DVS_VSEL=0.85V */
	ret = wm831x_reg_write(wm831x, WM831X_DC2_DVS_CONTROL, \
			WM831X_DC2_DVS_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x dc2 dvs reg error.\n", __func__);
		goto out;
	}

	/* Set other LDOs hardware controlled by Hardware Control1,
	   and set other LDOs hardware enable function, accroding to board design
	*/
	/* Set LDO1 hardware controlled by Hardware Control1:shut down mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO1_CONTROL, \
			WM831X_LDO1_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x ldo1 ctrl reg error.\n", __func__);
		goto out;
	}
//xnn add for ldo suspend control

	/* Set LDO2 hardware controlled by Hardware Control1:low-pwr mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO2_CONTROL, \
			WM831X_LDO2_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x ldo2 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set LDO3 hardware controlled by Hardware Control1:low-pwr mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO3_CONTROL, \
			WM831X_LDO3_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x ldo3 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set LDO4 hardware controlled by Hardware Control1:shut down mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO4_CONTROL, \
			WM831X_LDO4_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x ldo4 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set LDO5 hardware controlled by Hardware Control1:shut down mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO5_CONTROL, \
			WM831X_LDO5_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x ldo5 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set LDO6 hardware controlled by Hardware Control1:shut down mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO6_CONTROL, \
			WM831X_LDO6_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x ldo6 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set LDO7 hardware controlled by Hardware Control1:low-pwr mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO7_CONTROL, \
			WM831X_LDO7_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x ldo7 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set LDO8 hardware controlled by Hardware Control1:low-pwr mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO8_CONTROL, \
			WM831X_LDO8_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x ldo8 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set LDO9 hardware controlled by Hardware Control1:shut down mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO9_CONTROL, \
			WM831X_LDO9_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x ldo9 ctrl reg error.\n", __func__);
		goto out;
	}

	/* Set LDO10 hardware controlled by Hardware Control1:shut down mode */
	ret = wm831x_reg_write(wm831x, WM831X_LDO10_CONTROL, \
			WM831X_LDO10_CONTROL_VAL); 
	if (0 > ret) {
		printk("func:%s, write wm831x ldo10 ctrl reg error.\n", __func__);
		goto out;
	}
//xnn add end
	return 0;

out:
	return ret;
}


static struct wm831x_pdata wm8325_plat= {
	.post_init = wm8325_post_init,
	.irq_base = MXC_BOARD_IRQ_START,

	.dcdc = {
		&wm8325_dc1,
		&wm8325_dc2,
		&wm8325_dc3,
		&wm8325_dc4,
	},
	.ldo = {
		 &wm8325_ldo1,        
		 &wm8325_ldo2,        
		 &wm8325_ldo3,        
		 &wm8325_ldo4,        
		 &wm8325_ldo5,        
		 &wm8325_ldo6,        
		 &wm8325_ldo7,        
		 &wm8325_ldo8,        
		 &wm8325_ldo9,        
		 &wm8325_ldo10,        
		 &wm8325_ldo11,        
	},
};

static struct i2c_board_info __initdata wm8325_i2c_device = {
	I2C_BOARD_INFO(WM8325_I2C_DEVICE_NAME, WM8325_I2C_ADDR >> 1),
	.irq = gpio_to_irq(MX53_SMD_WM8325_IRQ),
	.platform_data = &wm8325_plat,
};

int __init mx53_smd_init_wm8325(void)
{
	return i2c_register_board_info(0, &wm8325_i2c_device, 1);
}
