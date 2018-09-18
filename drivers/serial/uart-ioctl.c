/*
 * uart-ioctl.c
 *
 * ioctl example
 *
 * Copyright (C) 2005 Farsight
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/security.h>
#include <asm/uaccess.h>
#include <mach/iomux-mx53.h>
#include <linux/delay.h>   /*mdalay*/
#include <mach/gpio.h>  /*gpio_ */

#define UART_MAGIC 'k'

#define UART_422       _IO(UART_MAGIC, 1)
#define UART_485_READ  _IO(UART_MAGIC, 2)
#define UART_485_WRITE _IO(UART_MAGIC, 3)

#define EIM_D24_GPIO_MODE_CMD  _IO(UART_MAGIC,4)
#define EIM_D24_UART_MODE_CMD  _IO(UART_MAGIC,5)
#define EIM_D26_GPIO_MODE_CMD  _IO(UART_MAGIC,6)
#define EIM_D26_UART_MODE_CMD  _IO(UART_MAGIC,7)

extern void  EIM_D24_GPIO_MODE();
extern void  EIM_D24_UART_MODE();
extern void  EIM_D26_GPIO_MODE();
extern void  EIM_D26_UART_MODE();



//#include "uart.h"
//#define  UART_422       1
//#define  UART_485_READ  2
//#define  UART_485_WRITE 3


#define MX53_SMD_CHR_STAT1              (2*32 + 19)     /* GPIO_3_19 */
#define MX53_SMD_CHR_PG                 (2*32 + 29)     /* GPIO_3_29 */




MODULE_LICENSE ("GPL");

int uart_major = 250;
int uart_minor = 0;
int number_of_devices = 1;
char data[128]="\0";

static struct class  *uart_class;

struct cdev cdev;
dev_t dev = 0;

static int uart_open (struct inode *inode, struct file *file)
{
  printk (KERN_INFO "/dev/UART2_SWITCH! device opened\n");
  return 0;
}

static int uart_release (struct inode *inode, struct file *file)
{
  printk (KERN_INFO "/dev/UART2_SWITCH! device closed\n");
  return 0;
}

int uart_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
  int ret=0;
   printk("cmd = %d\n",cmd);
  switch (cmd) {    
  case UART_422:
        printk(KERN_INFO "UART_422 called\n");
        //GPIO1_RSV =1
        gpio_request(MX53_SMD_CHR_STAT1, "GPIO1");
        gpio_direction_output(MX53_SMD_CHR_STAT1, 1);
        gpio_set_value(MX53_SMD_CHR_STAT1, 1);
        gpio_free(MX53_SMD_CHR_STAT1);
        printk("GPIO1_RSV=1\n");

        //GPIO4_RSV=0
        gpio_request(MX53_SMD_CHR_PG, "GPIO4");
        gpio_direction_output(MX53_SMD_CHR_PG, 0);
        gpio_set_value(MX53_SMD_CHR_PG, 0);
        gpio_free(MX53_SMD_CHR_PG);
        printk("GPIO4_RSV=0\n");
        printk("422 MODE\n");
    break;

  case UART_485_WRITE:
    printk(KERN_INFO  "UART_485_WRITE call\n");
        //GPIO1_RSV =1
        gpio_request(MX53_SMD_CHR_STAT1, "GPIO1");
        gpio_direction_output(MX53_SMD_CHR_STAT1, 1);
        gpio_set_value(MX53_SMD_CHR_STAT1, 1);
        gpio_free(MX53_SMD_CHR_STAT1);
        printk("GPIO1_RSV=1\n");
  
        //GPIO4_RSV=1
        gpio_request(MX53_SMD_CHR_PG, "GPIO4");
        gpio_direction_output(MX53_SMD_CHR_PG, 1);
        gpio_set_value(MX53_SMD_CHR_PG, 1);
        gpio_free(MX53_SMD_CHR_PG);
        printk("GPIO4_RSV=1\n");
        printk("485 WRITE MODE\n");

    break;

   case UART_485_READ:
        printk(KERN_INFO "UART_485_READ called\n");
 
        //GPIO1_RSV =0
        gpio_request(MX53_SMD_CHR_STAT1, "GPIO1");
        gpio_direction_output(MX53_SMD_CHR_STAT1, 0);
        gpio_set_value(MX53_SMD_CHR_STAT1, 0);
        gpio_free(MX53_SMD_CHR_STAT1);
        printk("GPIO1_RSV=0\n");
  
        //GPIO4_RSV=0
        gpio_request(MX53_SMD_CHR_PG, "GPIO4");
        gpio_direction_output(MX53_SMD_CHR_PG, 0);
        gpio_set_value(MX53_SMD_CHR_PG, 0);
        gpio_free(MX53_SMD_CHR_PG);
        printk("GPIO4_RSV=0\n");
        printk("485 READ MODE\n");
    break;

    case EIM_D24_GPIO_MODE_CMD:
	 EIM_D24_GPIO_MODE();
    break;

    case EIM_D24_UART_MODE_CMD:
	 EIM_D24_UART_MODE();
    break;

    case EIM_D26_GPIO_MODE_CMD:
	 EIM_D26_GPIO_MODE();
    break;

    case EIM_D26_UART_MODE_CMD:
	 EIM_D26_UART_MODE();
    break;
    

  default:
  
    break;
  
  }

  return ret;
}


struct file_operations uart_fops = {
  .owner = THIS_MODULE,
  .open  = uart_open,
  .release = uart_release,
  .ioctl = uart_ioctl,
};

static void char_reg_setup_cdev (void)
{
  int error, devno = MKDEV (uart_major, uart_minor);
  cdev_init (&cdev, &uart_fops);
  cdev.owner = THIS_MODULE;
  cdev.ops = &uart_fops;
  error = cdev_add (&cdev, devno , 1);
  if (error)
    printk (KERN_NOTICE "Error %d adding char_reg_setup_cdev", error);

}

static int __init uart_2_init (void)
{
  int result;

  dev = MKDEV (uart_major, uart_minor);
  result = register_chrdev_region (dev, number_of_devices, "uart");
  if (result<0) {
    printk (KERN_WARNING "uart: can't get major number %d\n", uart_major);
    return result;
  }

  char_reg_setup_cdev();


   uart_class = class_create(THIS_MODULE, "UART2_SWITCH");
//class_create和device_create  自动在/dev目录下创建设备节点gps_sleep
   if(IS_ERR(uart_class)) 
      {
       printk("Err: failed in creating class./n");
       return -1; 
      }
   device_create(uart_class, NULL, MKDEV(uart_major, 0),  NULL, "UART2_SWITCH");
   printk("creat /dev/UART2_SWITCH\n");
//在这里需要注意了，由于内核版本不同，导致device_create的参数可能有变化，

//我就遇到了，在网上找了一个，一用就出问题了，系统会跑死，现在这个的内核是2.6.35的
  

  printk (KERN_INFO "uart_ioctl driver done\n");
  return 0;
}

static void __exit uart_2_exit (void)
{
  dev_t devno = MKDEV (uart_major, uart_minor);
  
  cdev_del (&cdev);

  device_destroy(uart_class, MKDEV(uart_major, 0));//delete device node under /dev
  class_destroy(uart_class); //delete class created by us


  unregister_chrdev_region (devno, number_of_devices);

  printk (KERN_INFO "uart_ioctl cleaned up\n");
}

module_init (uart_2_init);
module_exit (uart_2_exit);
