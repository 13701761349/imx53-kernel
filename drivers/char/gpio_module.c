/********************************misc_test.c**************/    
#include <linux/miscdevice.h>  
#include <linux/delay.h>  
#include <asm/irq.h>  
#include <linux/kernel.h>  
#include <linux/module.h>  
#include <linux/init.h>  
#include <linux/mm.h>  
#include <linux/fs.h>  
#include <linux/types.h>  
#include <linux/time.h>    
#include <linux/timer.h>   
#include <linux/moduleparam.h>  
#include <linux/slab.h>  
#include <linux/errno.h>  
#include <linux/ioctl.h>  
#include <linux/cdev.h>  
#include <linux/string.h>  
#include <linux/list.h>  
#include <linux/gpio.h>  
#include <asm/uaccess.h>  
#include <asm/atomic.h>  
#include <asm/unistd.h>  

#define DEBUG  
#ifdef DEBUG    
#define DBG(...) printk(" DBG(%s, %s(), %d): ", __FILE__, __FUNCTION__, __LINE__); printk(__VA_ARGS__)    
#else    
#define DBG(...)    
#endif    

#define  MISC_HIGH     1
#define  MISC_LOW	  0

#define MX53_SMD_GPIO7_0	(6*32+0)

#define GPIO_TEST	MX53_SMD_GPIO7_0


#define DEVICE_NAME "misc_test_dev"  

int ret;
#define NUM_BYTES 32
ssize_t misc_write(struct file *filp, const char __user *buf, size_t count,  loff_t *f_pos)  
{   	
	char *kbuf=kmalloc(count+1,GFP_KERNEL);//need print it using %s, so plus 1 byte for '\0' 
	if (count == 0) return count; 
	DBG("to copy from user %d bytes\n", count);
	
  	ret = copy_from_user(kbuf, buf, count);//buf->kbuf,if success,ret=0  
	DBG("copied %d bytes of %s\n", count-ret,kbuf);

	kfree(kbuf);
	return count-ret; //return the bytes quantity have copied
}  
ssize_t misc_read(struct file *filp, char __user *buf, size_t count,  loff_t *f_pos)  
{  
	char *kbuf=kmalloc(NUM_BYTES,GFP_KERNEL);
	if (count == 0) return count; 
 	kbuf="hello evryone";
	DBG("to copy to user %d bytes\n", count);

	ret = copy_to_user(buf, kbuf,  count);//kbuf->buf,if success,ret=0  
	DBG("copied %d bytes of %s\n", count-ret,kbuf);
	
	kfree(kbuf);
 	return count-ret ;  //return the bytes quantity have copied
}  


static int misc_release(struct inode *inode, struct file *filp)  
{
DBG("release \n");   
return 0;  
}  

static int misc_open(struct inode *inode, struct file *filp)  
{     
DBG("open \n");  
return 0;  
}  


int uart_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
  int ret=0;

  switch (cmd) {

  case MISC_HIGH:
	DBG("MISC_HIGH \n"); 

        gpio_direction_output(GPIO_TEST, 0);
        gpio_set_value(GPIO_TEST, 0);
    break;

  case MISC_LOW:
	DBG("MISC_LOW \n"); 
       // gpio_request(GPIO_TEST, "GPIO7_0");
        gpio_direction_output(GPIO_TEST, 0);
        gpio_set_value(GPIO_TEST, 1);
    break;

  
  default:
    break;
  
  }

  return ret;
}



static struct file_operations dev_fops = {  
.owner  =   THIS_MODULE,  
.open   =   misc_open,  
.read   =   misc_read,  
.write  =   misc_write,  
.release=   misc_release, 
 .ioctl = uart_ioctl, 
};  

static struct miscdevice misc = {  
.minor = MISC_DYNAMIC_MINOR,  
.name = DEVICE_NAME,  
.fops = &dev_fops,  
};  

static int __init dev_init(void)  
{  
int ret;  
ret = misc_register(&misc);  
DBG (DEVICE_NAME"\tinit\n");  

    gpio_request(GPIO_TEST, "GPIO7_0");
return ret;  
}  

static void __exit dev_exit(void)  
{  
gpio_free(GPIO_TEST);
DBG (DEVICE_NAME"\texit\n");  
misc_deregister(&misc);  
}  

module_init(dev_init);  
module_exit(dev_exit);  
MODULE_LICENSE("GPL");  
MODULE_AUTHOR("Song.");  
