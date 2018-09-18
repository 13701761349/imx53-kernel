/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file drivers/serial/mxc_uart.c
 *
 * @brief Driver for the Freescale Semiconductor MXC serial ports based on
 * drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 * @ingroup UART
 */

/*
 * Include Files
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/platform_device.h>
#include <linux/sysrq.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/div64.h>
#include <mach/mxc_uart.h>
// add by panzidong
#include <mach/iomux-mx53.h>
#include <linux/delay.h>   /*mdalay*/
#include <mach/gpio.h>  /*gpio_ */
// end by panzidong

#define MX53_SMD_GPIO3_27                       (2*32 + 27) 
#define MX53_SMD_GPIO4_RSV              (2*32 + 16)     /* GPIO_3_16 */

int __init mxcuart_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Serial: MXC Internal UART2 IOMUX driver\n");
        static u64 uart_pads[]={
// 	MX53_PAD_EIM_D27__UART2_RXD_MUX,
 	MX53_PAD_EIM_D27__GPIO3_27,
 //	MX53_PAD_EIM_D26__UART2_TXD_MUX,
	};
        mxc_iomux_v3_setup_multiple_pads(uart_pads,ARRAY_SIZE(uart_pads));
        gpio_free(MX53_SMD_GPIO4_RSV);
        gpio_request(MX53_SMD_GPIO4_RSV, "GPIO4");
        gpio_direction_output(MX53_SMD_GPIO4_RSV, 1);
        gpio_set_value(MX53_SMD_GPIO4_RSV, 1);


 
        while(1)
          {
    /*
           mxc_iomux_v3_setup_multiple_pads(uart_pads,ARRAY_SIZE(uart_pads));
           gpio_free(MX53_SMD_GPIO4_RSV);
           gpio_request(MX53_SMD_GPIO4_RSV, "GPIO4");
           gpio_direction_output(MX53_SMD_GPIO4_RSV, 1);
           gpio_set_value(MX53_SMD_GPIO4_RSV, 1);

      */
          gpio_request(MX53_SMD_GPIO3_27,"GPIO3_27");
          gpio_direction_output(MX53_SMD_GPIO3_27,1);
          gpio_set_value(MX53_SMD_GPIO3_27,1);
          printk("high \n");
          mdelay(1000);
          gpio_set_value(MX53_SMD_GPIO3_27,0);
           printk("low\n");
          mdelay(1000);
          }
	return ret;
}

/*!
 * This function is used to cleanup all resources before the driver exits.
 */
static void __exit mxcuart_exit(void)
{
//	platform_driver_unregister(&mxcuart_driver);
//	uart_unregister_driver(&mxc_reg);
}

module_init(mxcuart_init);
module_exit(mxcuart_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC serial port driver");
MODULE_LICENSE("GPL");


