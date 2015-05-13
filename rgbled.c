/*
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include "rgbled.h"
#include "axp209_event.h"
#include "mt7620a_reg.h"

#define GPIO_IRQ_NUM    26
//struct file_operations rgbled_fops;
const struct ledcfg   init_cfg = {
    .LCFG = {
        .value = 0x13,
    },
    .LCTR = {
        .value = 0x00,
    },
    .T1_T2 = {
        .value = 0x00, 
    },
    .T3_T4 = {
        .value = 0x00,
    },
    .T0 = {
        .value = 0x00,
    },
    .PWM = 0xff,
};

struct {
    struct i2c_client *client;
    struct semaphore sem;
}rgbled_info;

#define DECLARE_LEDCFG(name)    struct ledcfg name = init_cfg;

int rgbled_read_a8_d8(struct i2c_client *client, unsigned char addr,unsigned char *value);

int rgbled_write_a8_d8(struct i2c_client *client, unsigned char addr,unsigned char value);

static int rgbled_readData(struct i2c_client *client, unsigned char reg,unsigned char *value);

void rgbled_write_config(struct i2c_client *client,struct ledcfg cfg);

static int irq_status_chk(void *dummy)
{
    int i = 0;
    uint8_t value = 0x00;
    while(!value)
    {
        rgbled_readData(rgbled_info.client, 0x02, &value);
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(10);
    }
    printk("%s\t%d\t%x\n", __func__, __LINE__, value);
    return 0;
}

int axp209_event(struct notifier_block *nb, unsigned long event, void *v)
{
    uint8_t value = 0x00;
    printk("%s\t%d\t%d\n", __func__, __LINE__, event);
    switch(event)
    {
        case ELEC80_100:
            {
                DECLARE_LEDCFG(r);
                r.LCTR.lctr.LED0 = 0x01;
                r.LCFG.lcfg.MD = 0x00;
                rgbled_write_config(rgbled_info.client, r);
                break;
            }
        case ELEC50_80:
            {
                DECLARE_LEDCFG(r);
                r.LCTR.lctr.LED0 = 0x01;
                r.T3_T4.t3_t4.T4 = 0x06;
                rgbled_write_config(rgbled_info.client, r);
                break;
            }
        case ELEC20_50:
            {
                DECLARE_LEDCFG(r);
                r.LCTR.lctr.LED0 = 0x01;
                r.T3_T4.t3_t4.T4 = 0x05;
                rgbled_write_config(rgbled_info.client, r);
                break;
            }
        case ELEC0_20:
            {
                DECLARE_LEDCFG(r);
                r.LCTR.lctr.LED0 = 0x01;
                r.T3_T4.t3_t4.T4 = 0x04;
                rgbled_write_config(rgbled_info.client, r);
                break;
            }
        case ELEC80_100_C:
            {
                DECLARE_LEDCFG(r);
                r.LCFG.lcfg.MD = 0x00;
                r.LCTR.lctr.LED1 = 0x01;
                rgbled_write_config(rgbled_info.client, r);
                break;
            }
        case ELEC50_80_C:
            {
                DECLARE_LEDCFG(r);
                r.LCTR.lctr.LED1 = 0x01;
                r.T3_T4.t3_t4.T4 = 0x06;
                rgbled_write_config(rgbled_info.client, r);
                break;
            }
        case ELEC20_50_C:
            {
                DECLARE_LEDCFG(r);
                r.LCTR.lctr.LED1 = 0x01;
                r.T3_T4.t3_t4.T4 = 0x05;
                rgbled_write_config(rgbled_info.client, r);
                break;
            }
        case ELEC0_20_C:
            {
                DECLARE_LEDCFG(r);
                r.LCTR.lctr.LED1 = 0x01;
                r.T3_T4.t3_t4.T4 = 0x04;
                rgbled_write_config(rgbled_info.client, r);
                break;
            }
    }
    //kthread_run(irq_status_chk, NULL, "rgbled_thread");
    return NOTIFY_DONE;
}

static int rgbled_sendData(struct i2c_client *client, unsigned char reg,unsigned char value)
{
    int ret = 0;
    int times = 0;
    ret = rgbled_write_a8_d8(client,reg,value);
    while(ret != 0 && times < 2)
    {
        ret = rgbled_write_a8_d8(client,reg,value);
        times ++;
    }
    return ret;
}
static int rgbled_readData(struct i2c_client *client, unsigned char reg,unsigned char *value)
{
    int ret = 0;
    int times = 0;

    ret = rgbled_read_a8_d8(client,reg,value);
    while(ret != 0 && times < 2)
    {
        ret = rgbled_read_a8_d8(client,reg,value);
        times ++;
    }
    return ret;
}
int rgbled_read_a8_d8(struct i2c_client *client, unsigned char addr,unsigned char *value)
{
    unsigned char data[2];
    struct i2c_msg msg[2];
    int ret;

    data[0] = addr;
    data[1] = 0xee;
    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &data[0];
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = &data[1];

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret >= 0) 
    {
        *value = data[1];
        ret = 0;
    } 
    return ret;
}
int rgbled_write_a8_d8(struct i2c_client *client, unsigned char addr,unsigned char value)
{
    struct i2c_msg msg;
    unsigned char data[2];
    int ret;

    data[0] = addr;
    data[1] = value;

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = 2;
    msg.buf = data;

    ret = i2c_transfer(client->adapter, &msg, 1);
    return ret;
}

void rgbled_write_config(struct i2c_client *client,struct ledcfg cfg)
{
    uint8_t val = 0;
    rgbled_sendData(client, CTR, cfg.LCTR.value);
    printk("%s\t%d\t%d\n", __func__, __LINE__, cfg.LCTR.value);
    if(cfg.LCTR.lctr.LED0)
    {
        printk("%s\t%d\n", __func__, __LINE__);
        rgbled_sendData(client, LCFG0, cfg.LCFG.value);
        rgbled_sendData(client, LED0T0, cfg.T1_T2.value);
        rgbled_sendData(client, LED0T1, cfg.T3_T4.value);
        rgbled_sendData(client, LED0T2, cfg.T0.value);
        rgbled_sendData(client, PWM0, cfg.PWM);
    }
    if(cfg.LCTR.lctr.LED1)
    {
        rgbled_sendData(client, LCFG1, cfg.LCFG.value);
        rgbled_sendData(client, LED1T0, cfg.T1_T2.value);
        rgbled_sendData(client, LED1T1, cfg.T3_T4.value);
        rgbled_sendData(client, LED1T2, cfg.T0.value);
        rgbled_sendData(client, PWM1, cfg.PWM);
    }
    if(cfg.LCTR.lctr.LED2)
    {
        rgbled_sendData(client, LCFG2, cfg.LCFG.value);
        rgbled_sendData(client, LED2T0, cfg.T1_T2.value);
        rgbled_sendData(client, LED2T1, cfg.T3_T4.value);
        rgbled_sendData(client, LED2T2, cfg.T0.value);
        rgbled_sendData(client, PWM2, cfg.PWM);
    }
    rgbled_sendData(client, 0x01, 0x01);
}

static struct notifier_block rgbled_notifier = {
    .notifier_call = axp209_event,
};

EXPORT_SYMBOL(rgbled_notifier);

static int rgbled_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
    int ret, value;
    if(!i2c_check_functionality(client->adapter,I2C_FUNC_I2C))
        return -ENODEV;
    rgbled_info.client = client;
    sema_init(&rgbled_info.sem, 0);
    //gpio26_irq_init();

    rgbled_sendData(client, 0x01, 0xE1);
    rgbled_sendData(client, 0x02, 0xF0);
    //r g b enable
    //rgbled_sendData(client, CTR, 0x07);

    //电流
    //蓝
    //绿
    /*
       rgbled_sendData(client, 0x32, 0x0);
    //红
    rgbled_sendData(client, 0x33, 0x0);
    rgbled_sendData(client, 0x34, 0xFF);
    rgbled_sendData(client, 0x35, 0xFF);
    rgbled_sendData(client, 0x36, 0xFF);
    */
    //rgbled_sendData(client, 0x01, 0x01 << 4);
    return 0;
}

static int rgbled_remove(struct i2c_client *client)
{
    rgbled_sendData(client, CTR, 0x00);
    return 0;
}

static const struct i2c_device_id rgbled_id[] = {
    {"rgbled",0},
    { },
};
MODULE_DEVICE_TABLE(i2c,rgbled_id);

static const struct of_device_id rgbled_of_match[] = {
    {.compatible = "aw,rgbled"},
    { }
};
MODULE_DEVICE_TABLE(of,rgbled_of_match);

static struct i2c_driver rgbled_driver = {
    .driver          = {
        .name    = "rgbled",
        .owner	 = THIS_MODULE,
        .of_match_table = of_match_ptr(rgbled_of_match)
    },
    .probe		 = rgbled_probe,
    .remove          = rgbled_remove,
    .id_table        = rgbled_id
};

/*
   struct file_operations rgbled_fops = {
   .open = rgbled_open,
   .release = rgbled_release,
   .unlocked_ioctl = rgbled_ioctl,
   };
   */
module_i2c_driver(rgbled_driver);

MODULE_AUTHOR("mrdong <mrdong@focalcrest.com>");
MODULE_DESCRIPTION("rgbled I2C bus driver");
MODULE_LICENSE("GPL");
