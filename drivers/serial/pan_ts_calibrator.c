#include <linux/module.h>  
#include <linux/init.h>  
#include <linux/version.h>  
#include <linux/proc_fs.h>  
#include <asm/uaccess.h>  
#include <linux/gpio.h>  
//#include <mach/regs-gpio.h>  
   
#define USER_ROOT_DIR "pan_ts_calibrator"  
#define USER_ENTRY1 "pan_ts_calibrator_entry"  
  
static struct proc_dir_entry *uart_en_root;  
static struct proc_dir_entry *uart_en_entry1;  
static char msg[255];  
extern unsigned long  calibration[7];

int proc_write_information(struct file *file,const char *buffer,unsigned long count,void *data);  
   
static int proc_uart_en_init(void)  
{  
   
     uart_en_root = proc_mkdir(USER_ROOT_DIR,NULL);  
     if(NULL == uart_en_root)  
     {  
         printk(KERN_ALERT "Create dir /proc/%s error!\n",USER_ROOT_DIR);  
         return -1;  
     }  
     printk(KERN_INFO "Create dir /proc/%s\n",USER_ROOT_DIR);  
   
     uart_en_entry1 = create_proc_entry(USER_ENTRY1,0666,uart_en_root);  
     if(NULL == uart_en_entry1)  
     {  
         printk(KERN_ALERT "Create entry %s under /proc/%s error!\n",USER_ENTRY1,USER_ROOT_DIR);  
         goto err_out;  
     }  
     printk(KERN_INFO "Create /proc/%s/%s\n",USER_ROOT_DIR,USER_ENTRY1);  
   
     uart_en_entry1->write_proc = proc_write_information;  
   
     return 0;  
   
     err_out: remove_proc_entry(USER_ROOT_DIR,uart_en_root);  
     return -1;  
 }  
   
 static void proc_uart_en_exit(void)  
 {  
     remove_proc_entry(USER_ENTRY1,uart_en_root);  
     remove_proc_entry(USER_ROOT_DIR,NULL);  
     printk(KERN_INFO "All proc entry removed !\n");  
 }  
//extern int  calibratre_para[7];   
int proc_write_information(struct file *file,const char *buffer,unsigned long count,void *data)  
 {  
     int count2 = 0;  
     int i=0;
     int flag=1;
//     int tmp=1; 
     unsigned long tmp=1;
     int firstchar=0;
     int calibrator[7]={0};
     printk("in proc_write_information\n");

     if(copy_from_user((void *)msg,(const void __user *)buffer,count))  
         return -EFAULT;  
      printk("msg=%s  \n",msg);

    while(i != 7 ) {

      while(msg[count2] != ',' &&  count2 <=(count-1) ){

        if(msg[count2]=='-'){
            flag=-1;
            count2++;
       }else if(firstchar == 0){
          tmp=msg[count2]-'0';
          printk("i=%d,count2=%d,tmp=%ld \n",i,count2,tmp);
          firstchar =1;
          count2++;
       }else if(firstchar == 1){
          tmp = tmp*10+(msg[count2]-'0');
          count2++;
          printk("i=%d,count2=%d,tmp=%ld \n",i,count2,tmp);
      }

     }
      tmp = flag * tmp;
      printk("i=%d,tmp=%ld \n",i,tmp);
      calibrator[i]=tmp;
      firstchar=0;
      flag=1;
      count2++;
      i++;
      tmp=1;
     }

        for(i=0;i<7;i++)
         {
        calibration[i]=calibrator[i];
        printk("calibrate[%d]=%ld \n",i,calibration[i]);
         }
    /*  while(count<7){
        printk("calibrate_para[%d]=%s \n",count,msg[count]);
        count++;
       }*/
             return count;  
  }  
   
module_init(proc_uart_en_init);  
module_exit(proc_uart_en_exit);  
MODULE_LICENSE("GPL");  
MODULE_AUTHOR("BQL"); 

