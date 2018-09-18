/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
// add by panzidong
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
// end by panzidong

// add by panzidong
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/power_supply.h>

// end by panzidong



#define DRIVER_VERSION			"1.1.0"

#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_FLAGS		0x0A
#define BQ27x00_REG_TTE			0x16
#define BQ27x00_REG_TTF			0x18
#define BQ27x00_REG_TTECP		0x26

#define BQ27000_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27000_FLAG_CHGS		BIT(7)

#define BQ27500_REG_SOC			0x2c
#define BQ27500_FLAG_DSC		BIT(0)
#define BQ27500_FLAG_FC			BIT(9)

#define STAT2      (2*32 + 20)     /* GPIO_3_20 */
#define STAT1          (5*32 + 17)     /* GPIO_6_17 */
#define PG       (5*32 + 18)     /* GPIO_6_18 */



/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27x00_device_info *di);
};

enum bq27x00_chip { BQ27000, BQ27500 ,BQ24610};

struct bq27x00_device_info {
	struct device 		*dev;
	int			id;
	struct bq27x00_access_methods	*bus;
	struct power_supply	bat;
	enum bq27x00_chip	chip;

	struct i2c_client	*client;
        // add bypanzidong
          struct workqueue_struct *monitor_wqueue;
        struct delayed_work monitor_work;

        int old_pe_status;
        int old_stat1_status;
        int old_stat2_status;

};
// add by panzidong
static void bq27x00_battery_update_status(struct bq27x00_device_info *di)
{
        //int old_charge_status = di->charge_status;

       // ds2438_battery_read_status(di);

       // if (di->charge_status != old_charge_status)
        int pe_status,stat1_status,stat2_status;
            pe_status=gpio_get_value(PG);
           stat1_status=gpio_get_value(STAT1);
           stat2_status=gpio_get_value(STAT2);

        if(di->old_pe_status != pe_status || di->old_stat1_status != stat1_status || di->old_stat2_status != stat2_status )
        {
                di->old_pe_status = pe_status;
                di->old_stat1_status = stat1_status;
                di->old_stat2_status = stat2_status;
                power_supply_changed(&di->bat);
         }
}

// add by panzidong
static void bq27x00_battery_work(struct work_struct *work)
{
        struct bq27x00_device_info *di = container_of(work,
                                                     struct bq27x00_device_info,
                                                     monitor_work.work);
      //    const int interval = HZ * 60;
          const int interval = HZ * 6;

      //  dev_dbg(di->w1_dev, "%s\n", __func__);

        bq27x00_battery_update_status(di);
        queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}






static enum power_supply_property bq27x00_battery_props[] = {


// add by panzidong
//     POWER_SUPPLY_PROP_ONLINE,
       POWER_SUPPLY_PROP_TECHNOLOGY,
       POWER_SUPPLY_PROP_HEALTH,
// add by panzidong
	POWER_SUPPLY_PROP_STATUS,//状态

	POWER_SUPPLY_PROP_VOLTAGE_NOW,//电压
	POWER_SUPPLY_PROP_CAPACITY,//电量
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,//电池极限电压 最大值
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,//电池极限电压 最小值

};

/*
 * Common code for BQ27x00 devices
 */

static int bq27x00_read(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di)
{
	int ret;
	int temp = 0;


         return 30;
/*
	ret = bq27x00_read(BQ27x00_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	if (di->chip == BQ27500)
		return temp - 2731;
	else
		return ((temp >> 2) - 273) * 10;*/

       return 0;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */

#include <linux/mfd/wm831x/auxadc.h>
//电压，ljsong
extern  int battery_data_read();
static int bq27x00_battery_voltage(struct bq27x00_device_info *di)
{

        int volt = 6400;
	int data = battery_data_read() & 0xfff;
        if(data == 1){ // error
               volt =  6400;
	       return volt;
        }


        volt =(int)(((long)data*1465)*122/22)/1000;
	//printk("bq27x00_battery_voltage : data=0x%x,volt=%d\n",data,volt);

	if(volt>8400)
	volt=8400;

	if(volt<6400)
	volt=6400;


        return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di)
{
	int ret;
	int curr = 0;
	int flags = 0;
        return 3200;

	//return curr;
}


/*
最大电压，ljsong
 */
static int bq27x00_battery_min_volt(struct bq27x00_device_info *di)
{

	int volt= 6400;

       return volt;
}

/*
最小电压，ljsong
 */
static int bq27x00_battery_max_volt(struct bq27x00_device_info *di)
{
	int volt= 8400;
	
       return volt;
}


/*
百分比，ljsong
 */
static int bq27x00_battery_rsoc(struct bq27x00_device_info *di)
{

        
        int rsoc = 0 ;
        int data = battery_data_read() & 0xfff;
        if(data == 1){ // error
               rsoc =  0;
	       return rsoc;
        }

        int volt =(int)(((long)data*1465)*122/22)/1000;
	//printk("bq27x00_battery_rsoc : data=0x%x,volt=%d\n",data,volt);
        if(volt <= 6400)
             return 0;
   
        rsoc=(volt-6400)*100/(8400-6400);
 
	if(rsoc>100 || rsoc >99 )
	rsoc=100;

	if(rsoc<0)
	rsoc=0;

       return rsoc;

}

static int bq27x00_battery_status(struct bq27x00_device_info *di,
				  union power_supply_propval *val)
{
	int flags = 0;
	int status=0;
	int ret;
        int stat1_status=0;
        int stat2_status=0;
        int pe_status=0;
/*
IC STATE           PG
valid    vcc       off
invalid  vcc       on
*/
        pe_status=gpio_get_value(PG);
        //printk("pe_status=%d \n",pe_status);
        if(pe_status){//invalid  vcc
 //         val->intval=status;
  //        val->intval=POWER_SUPPLY_STATUS_UNKNOWN;
        val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
          return 0;
        }

/*
charge state                stat1       stat2
charge in progress           on          off
charge complete              off         on
charge suppend               off         off*/
      stat1_status=gpio_get_value(STAT1);
      stat2_status=gpio_get_value(STAT2);

      //printk("stat1_status=%d \n",stat1_status);
      //printk("stat2_status=%d \n",stat2_status);
 //     if (stat1_status==0 &&  stat2_status==1 )
      if (stat1_status==0  )
          status = POWER_SUPPLY_STATUS_CHARGING;
 //     else if(stat1_status==1 && stat2_status==0 )
      else if(stat1_status==1  )
          status = POWER_SUPPLY_STATUS_FULL;
//      else if(stat1_status ==0 && stat2_status==0 )
//          status = POWER_SUPPLY_STATUS_NOT_CHARGING;
//      else
//          status = POWER_SUPPLY_STATUS_DISCHARGING;



/*
	ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}

	if (di->chip == BQ27500) {
		if (flags & BQ27500_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (flags & BQ27500_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (flags & BQ27000_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
*/
	val->intval = status;
	return 0;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27x00_battery_time(struct bq27x00_device_info *di, int reg,
				union power_supply_propval *val)
{
	int tval = 0;
	int ret;

        val->intval = 10*60;
        return 0;

/*
	ret = bq27x00_read(reg, &tval, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading register %02x\n", reg);
		return ret;
	}

	if (tval == 65535)
		return -ENODATA;
*/
	val->intval = tval * 60;
	return 0;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	switch (psp) {


//  add by panzidong
/*        case POWER_SUPPLY_PROP_ONLINE:
                printk("pe_status = %d \n",gpio_get_value(PG));
                val->intval = gpio_get_value(PG);
                break;*/
        case POWER_SUPPLY_PROP_TECHNOLOGY:
                printk("TECHNOLOGY  NiMH \n");
                val->intval = 1;   // "NiMH"
                break;
        case POWER_SUPPLY_PROP_HEALTH:
                printk("HEALTH GOOD \n");
                val->intval = 1;
                break;
//  end by panzidong



	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN://ljsong
		val->intval  = bq27x00_battery_max_volt(di);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN://ljsong
		val->intval  = bq27x00_battery_min_volt(di);
		break;



	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27x00_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27x00_battery_voltage(di);
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27x00_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27x00_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27x00_battery_temperature(di);
		break;
/*	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27x00_battery_time(di, BQ27x00_REG_TTE, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27x00_battery_time(di, BQ27x00_REG_TTECP, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27x00_battery_time(di, BQ27x00_REG_TTF, val);
		break;*/
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = NULL;
}

/*
 * i2c specific code
 */

static int bq27x00_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;
/*
	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}*/
	return err;
}

static int bq27x00_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27x00_device_info *di;
	struct bq27x00_access_methods *bus;
	int num;
	int retval = 0;
        printk("1111111111111111\n");
	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
        printk("22222222222222222222222\n");
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

        printk("3333333333333333333331111111111111111\n");
	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

        printk("444444444444444444444441111111111111111\n");
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;
	di->chip = id->driver_data;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

        printk("555555555555555555555551111111111111111\n");
	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = name;
	bus->read = &bq27x00_read_i2c;
	di->bus = bus;
	di->client = client;

	bq27x00_powersupply_init(di);

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto batt_failed_4;
	}

        printk("66666666666666666661111111111111111\n");
	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

        // add by panzidong
        INIT_DELAYED_WORK(&di->monitor_work, bq27x00_battery_work);
        di->monitor_wqueue = create_singlethread_workqueue(
                                "bq27x00-battery-monitor");
        if (!di->monitor_wqueue) {
                retval = -ESRCH;
                goto batt_failed_4;
        }
         queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ / 2);


	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	power_supply_unregister(&di->bat);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

// add by panzidong
        cancel_delayed_work(&di->monitor_work);
        queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ);
// end by panzidong



	kfree(di);

	return 0;
}

/*
 * Module stuff
 */

static const struct i2c_device_id bq27x00_id[] = {
	{ "bq27200", BQ27000 },	/* bq27200 is same as bq27000, but with i2c */
	{ "bq27500", BQ27500 },
        { "bq27x00-battery",BQ24610},
	{},
};

static struct i2c_driver bq27x00_battery_driver = {
	.driver = {
		.name = "bq27x00-battery",
	},
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.id_table = bq27x00_id,
};

static int __init bq27x00_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27x00_battery_driver);
//
        gpio_request(STAT1, "STAT1");
        gpio_direction_input(STAT1);
        gpio_request(STAT2, "STAT2");
        gpio_direction_input(STAT2);
        gpio_request(PG, "PE");
        gpio_direction_input(PG);

	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 driver\n");
        else
		printk(KERN_ERR "register BQ27x00 driver\n");

	return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
