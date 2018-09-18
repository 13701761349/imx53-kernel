/*************************************

NAME:zed_watchdog.c
COPYRIGHT:www.zed.com

*************************************/
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/unistd.h>


#include <linux/gpio.h>
#include <linux/io.h>

#define DEVICE_NAME 	"watchdog_zed"
//#define DEBUG
#ifdef DEBUG      
#define DBG(...) printk(" DBG(%s, %s(), %d): ", __FILE__, __FUNCTION__, __LINE__); printk(__VA_ARGS__)      
#else      
#define DBG(...)      
#endif      
  
#define HARD_WATCHDOG_PORT    (0*32+4)
#define HARD_WATCHDOG_DELAY   (HZ*8)


#define FEED_MAGIC 'k'
#define FEED_CMD	 _IO(FEED_MAGIC,0x1a)


static bool b_busy=false;

static struct timer_list harddog_timer;
static void feed();
static void harddog_timer_callback(unsigned long arg);
static void start_timer();
static void stop_timer();

static void feed()
{
DBG("\n");
		gpio_direction_output(HARD_WATCHDOG_PORT,1);
		gpio_direction_output(HARD_WATCHDOG_PORT,0);
}

//auto feed
static void harddog_timer_callback(unsigned long arg)
{
	feed();
	start_timer();   	
}

static void stop_timer()
{
	del_timer(&harddog_timer); 
}

static void start_timer()
{
	harddog_timer.expires=jiffies+HARD_WATCHDOG_DELAY;
	harddog_timer.function=&harddog_timer_callback;
	add_timer(&harddog_timer);  
}





static int harddog_open(struct inode *inode, struct file *filp)    
{       

	if(b_busy==true)
		return -EBUSY;

	b_busy=true;
	feed();
	stop_timer();
	
	return 0;    
}    

static int harddog_release(struct inode *inode, struct file *filp)    
{  

	b_busy=false;
	//feed();
	//start_timer();

	return 0;     
}    


static long harddog_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
DBG("cmd=%d\n",cmd);
DBG("arg=%d\n",arg);
	switch(cmd) {
	case FEED_CMD:
		DBG("cmd=%d\n",cmd);
		feed();
		break;
	default:
		return -EINVAL;
	}
}



static struct file_operations dev_fops = {    
	.owner  =   THIS_MODULE,    
	.open   =   harddog_open,        
	.release=   harddog_release, 
	.unlocked_ioctl	= harddog_ioctl,
};    


static struct miscdevice misc = {
	.minor = WATCHDOG_MINOR,/*MISC_DYNAMIC_MINOR*/
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};


static int __init dev_init(void)
{
	b_busy=false;
	gpio_request(HARD_WATCHDOG_PORT, "watchdog_zed");
	feed();

	init_timer(&harddog_timer);   
	start_timer(); 

	int ret = misc_register(&misc);
	printk (DEVICE_NAME" initialized\n");

	return ret;
}

static void __exit dev_exit(void)
{
	stop_timer();
	misc_deregister(&misc);
	gpio_free(HARD_WATCHDOG_PORT);
	printk (DEVICE_NAME" exit\n");
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("liujiansong@zed.com");


