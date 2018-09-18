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
#include <linux/io.h>
#include <linux/ioport.h>


// add by panzdiong
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/uaccess.h>
#include <asm/mach-types.h>

// add by panzidong



#define LICENSES_MAGIC 'k'
#define LICENSES_READ  _IO(LICENSES_MAGIC, 1)

MODULE_LICENSE ("GPL");

//extern struct mxc_iim_data *iim_data;
void __iomem *cpuid_base=NULL;

static int licenses_major = 250;
static int licenses_minor = 2;
static int number_of_devices = 1;
static char data[128]="\0";
// add by panzdiong
static unsigned  char   kernel_licenses[32+1];
static unsigned  char   kernel_root[16];   // root=/dev/mmcblock0p4
static int  root_flags=1;
static int  licenses_flags=0;
// add by panzidong    
static struct class  *licenses_class;

static struct cdev cdev;
static dev_t dev = 0;


int linux_Encrypt(unsigned char* encrypt, unsigned char* licenses){
  unsigned  char KEY[18]={'z',
                  'e',
                  'd',
                  'c',
                  'h',
                  'a',
                  'o',
                  'r',
                  'e',
                  'n',
                  '2',
                  '0',
                  '1',
                  '3',
                  '1',
                  '0',
                  '1',
                  '0',
                  };
  unsigned  char KEY1[18]={
                   'a',
                   'b',
                   'd',
                   'c',
                   'h',
                   'a',
                   'z',
                   'z',
                   '2',
                   '0',
                   '1',
                   '3',
                   '4',
                   '5',
                   '6',
                   '7',
                   '1',
                   '0',
                  };
  unsigned  char KEY2[18]={1,2,3,4,5,6,7,8,
                           9,6,7,5,3,2,4,7,
                           1,2,};
    int i;
    for(i=0;i<16;i++)
      {
         encrypt[i]=(unsigned char)((encrypt[i]^KEY[i])+KEY1[i]+KEY2[i]);     
      }


    for(i=1;i<15;i++)
      {
         encrypt[i]=(unsigned char)(encrypt[i-1]*KEY2[i-1] + encrypt[i+1]*KEY2[i+1] + encrypt[i]*KEY2[i]); 
      }


      encrypt[0]=(unsigned char)(encrypt[15]*KEY2[16] + encrypt[0]*KEY2[0] + encrypt[1]*KEY2[1]);
      encrypt[15]=(unsigned char)(encrypt[14]*KEY2[14] +encrypt[0]*KEY2[0] + encrypt[15]*KEY2[15]);



    return 0;

}





static int licenses_open (struct inode *inode, struct file *file)
{
  printk (KERN_INFO "/dev/licenses! device opened\n");
  return 0;
}

static int licenses_release (struct inode *inode, struct file *file)
{
  printk (KERN_INFO "/dev/licenses! device closed\n");
  return 0;
}

int licenses_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
  int ret=0;
   printk("cmd = %d\n",cmd);
  switch (cmd) {    
  case  LICENSES_READ:
        printk(KERN_INFO "LICENSES_READ called\n");
  

    break;
  default:
  
    break;
  
  }

  return ret;
}

ssize_t  licenses_read(struct file *filp,char *buff,size_t count,loff_t *offp){
  
   if(count!=16){
      printk("cpuid length error \n");
      return -EFAULT;
   }
       

   cpuid_base = ioremap(0x63f98820,0x20);
    if(cpuid_base == NULL){
       printk("IOREMAP ERROR\n");
       return -EFAULT ;
     }else{
       printk("IOREMAP OK\n");
     }

    char hexTchar[16]={'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f',};
    char cpuid[17]={'0',};
    char tmp;
    int i=0;

    for(i=0;i<8;i++){

    tmp=(char)((readl(cpuid_base+4*i) & 0xf0)>>4);
    cpuid[2*i]=hexTchar[tmp];

    tmp=(char)((readl(cpuid_base+4*i)& 0xf));
    cpuid[2*i+1]=hexTchar[tmp];

    }

    cpuid[16]='\0';
    printk("imx53 cpu id : %s \n",cpuid);
     


    ssize_t result = 0;
    if(copy_to_user(buff,cpuid,sizeof(cpuid)-1)){
       iounmap(cpuid_base);
       result = -EFAULT;
    }
    else{
       printk(KERN_INFO,"read %d  bytes \n",count);
    }

    iounmap(cpuid_base);
    return result;
  
}

struct file_operations licenses_fops = {
  .owner = THIS_MODULE,
  .open  = licenses_open,
  .release = licenses_release,
  .ioctl =  licenses_ioctl,
  .read = licenses_read,
};

static void char_reg_setup_cdev(void)
{
  int error, devno = MKDEV (licenses_major, licenses_minor);
  cdev_init (&cdev, &licenses_fops);
  cdev.owner = THIS_MODULE;
  cdev.ops = &licenses_fops;
  error = cdev_add (&cdev, devno , 1);
  if (error)
    printk (KERN_NOTICE "Error %d adding char_reg_setup_cdev", error);

}

static int __init licenses_init (void)
{
/*
    if(root_flags == 0){
      printk("Kernel  boot mode is  into recovery....................\n");
      goto  SkipLicensesCheck;
    }
*/

/*
    printk("Kernel Start licenses  check  \n");
    if(licenses_flags == 0 )
      {
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        printk("bootargs not contain  licenses=xxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
        while(1);
      }
   int i;
   unsigned char encrypt[16+1];
   unsigned char decrypt[16];
   unsigned char licenses[32+1];
   char hexTchar[16]={'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f',};


    cpuid_base = ioremap(0x63f98820,0x20);
    if(cpuid_base == NULL){
       printk("IOREMAP ERROR\n");
       return -EFAULT ;
     }else{
       printk("IOREMAP OK\n");
     }
    
    unsigned char tmp;
   
    for(i=0;i<8;i++){

    tmp=(char)((readl(cpuid_base+4*i) & 0xf0)>>4);
    encrypt[2*i]=hexTchar[tmp];

    tmp=(char)((readl(cpuid_base+4*i)& 0xf));
    encrypt[2*i+1]=hexTchar[tmp];

    }

    encrypt[16]='\0';
    printk("imx53 cpu id : %s \n",encrypt);

   printk("before encrypt:%s\n after encrypt:\n",encrypt);

   linux_Encrypt(encrypt,licenses);
      for(i=0;i<16;i++)
      {
         licenses[2*i]=hexTchar[(encrypt[i]&0xf0)>>4];
         licenses[2*i+1]=hexTchar[encrypt[i]&0xf];
       
      }
      licenses[32]='\0';
   //   printk("!!!!!!!!!!!!!! licenses=%s\n",licenses);
   //   printk("!!!!!!!!!!!!!! kernel licenses=%s\n",kernel_licenses);

      if(strncmp(licenses,kernel_licenses,32)!=0)
        {
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        printk("licenses strncmp  kernel_licenses  error xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        while(1);
        }

      iounmap(cpuid_base);
//SkipLicensesCheck:
*/
  int result;

  dev = MKDEV (licenses_major, licenses_minor);
  result = register_chrdev_region (dev, number_of_devices, "licenses");
  if (result<0) {
    printk (KERN_WARNING "uart: can't get major number %d\n", licenses_major);
    return result;
  }

  char_reg_setup_cdev();


   licenses_class = class_create(THIS_MODULE, "licenses");
//class_create和device_create  自动在/dev目录下创建设备节点
   if(IS_ERR(licenses_class)) 
      {
       printk("Err: failed in creating class./n");
       return -1; 
      }
   device_create(licenses_class, NULL, MKDEV(licenses_major,licenses_minor ),  NULL, "licenses");
   printk("creat /dev/licenses\n");
//在这里需要注意了，由于内核版本不同，导致device_create的参数可能有变化，

//我就遇到了，在网上找了一个，一用就出问题了，系统会跑死，现在这个的内核是2.6.35的
  

   printk (KERN_INFO "licenses_ioctl driver done\n");


    return 0;
}

static void __exit licenses_exit (void)
{
  dev_t devno = MKDEV (licenses_major, licenses_minor);
  
  cdev_del (&cdev);

  device_destroy(licenses_class, MKDEV(licenses_major, licenses_minor));//delete device node under /dev
  class_destroy(licenses_class); //delete class created by us


  unregister_chrdev_region (devno, number_of_devices);

  printk (KERN_INFO "licenses_ioctl cleaned up\n");
}

static int __init enable_licenses_setup(char *options)
{
    //为了读取内核传过来的licenses=xxxxxxxxxxxxxxxxxxx;
    strncpy(kernel_licenses,options,32);
	kernel_licenses[32]='\0';
    licenses_flags = 1 ;
	return 1;
}
__setup("licenses=", enable_licenses_setup);
static int __init enable_recovery_setup(char *options)
{
    //为了读取内核传过来的root=/dev/mmcblock0p4,判断是否是进入recovery mode;
    root_flags=strncmp("/dev/mmcblock0p4",options,16);
    printk("root_flags =  %d  if==0 into recovery if==1 into normal boot mode;   \n",root_flags);
	return 1;
}
__setup("root=", enable_recovery_setup);


module_init (licenses_init);
module_exit (licenses_exit);
