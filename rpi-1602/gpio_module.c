#include <linux/miscdevice.h>  
#include <linux/delay.h>  
#include <asm/irq.h>  
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
#include <asm/io.h>  
#include <asm/uaccess.h>  
#include <linux/ioport.h>    
#include <linux/delay.h>
#include <linux/gpio.h>
#include "bcm2835.h"  

#define RS_1602 RPI_BPLUS_GPIO_J8_12
#define RW_1602 RPI_BPLUS_GPIO_J8_16
#define EN_1602 RPI_BPLUS_GPIO_J8_18

#define D0_1602 RPI_BPLUS_GPIO_J8_11
#define D1_1602 RPI_BPLUS_GPIO_J8_13
#define D2_1602 RPI_BPLUS_GPIO_J8_15
#define D3_1602 RPI_BPLUS_GPIO_J8_29
#define D4_1602 RPI_BPLUS_GPIO_J8_31
#define D5_1602 RPI_BPLUS_GPIO_J8_33
#define D6_1602 RPI_BPLUS_GPIO_J8_35
#define D7_1602 RPI_BPLUS_GPIO_J8_37

void data_1602(unsigned char data_v);
int bcm2835_gpio_fsel(uint8_t pin, uint8_t mode) ;
int bcm2835_gpio_set(uint8_t pin) ;
int bcm2835_gpio_clr(uint8_t pin);
void command_1602(unsigned char cmd);
void write_1602(unsigned char cmd);
void init_1602(void);
void L1602_char(unsigned char row, unsigned char pos, unsigned char sign);
void L1602_string(unsigned char row, unsigned char pos, char *p);

struct globalmem_dev 
{
	unsigned char mem[20];
};
struct globalmem_dev dev_1602; 


void L1602_string(unsigned char row, unsigned char pos, char *p)
{
	unsigned a;
	if(row == 1) a = 0x80;
	if(row == 2) a = 0xC0;
	a = a + pos - 1;
	command_1602(a);
	while(*p != '\0')
	{
		write_1602(*p);
		p++;
	}
}

void L1602_char(unsigned char row, unsigned char pos, unsigned char sign)
{
	unsigned a;
	if(row == 1) a = 0x80;
	if(row == 2) a = 0xC0;
	a = a + pos - 1;
	command_1602(a);
	write_1602(sign);
}

void init_1602(void)
{
	command_1602(0x01);
	command_1602(0x38);
	command_1602(0x0C);
	command_1602(0x06);
	command_1602(0xD0);
}

void write_1602(unsigned char cmd)
{
	data_1602(cmd);
	bcm2835_gpio_set(RS_1602);
	bcm2835_gpio_clr(RW_1602);
	bcm2835_gpio_clr(EN_1602);
	/*gpio_set_value(RS_1602, 1);
	gpio_set_value(RW_1602, 0);
	gpio_set_value(EN_1602, 0);*/
	udelay(200);
	bcm2835_gpio_set(EN_1602);
	//gpio_set_value(EN_1602, 1);
	udelay(200);
}

void command_1602(unsigned char cmd)
{
	data_1602(cmd);
	bcm2835_gpio_clr(RS_1602);
	bcm2835_gpio_clr(RW_1602);
	bcm2835_gpio_clr(EN_1602);
	udelay(200);
	bcm2835_gpio_set(EN_1602);
	udelay(200);
}

void data_1602(unsigned char data_v)
{
	if(data_v & 0x01)	bcm2835_gpio_set(D0_1602);
	else				bcm2835_gpio_clr(D0_1602);
	if(data_v & 0x02)	bcm2835_gpio_set(D1_1602);
	else				bcm2835_gpio_clr(D1_1602);
	if(data_v & 0x04)	bcm2835_gpio_set(D2_1602);
	else				bcm2835_gpio_clr(D2_1602);
	if(data_v & 0x08)	bcm2835_gpio_set(D3_1602);
	else				bcm2835_gpio_clr(D3_1602);
	if(data_v & 0x10)	bcm2835_gpio_set(D4_1602);
	else				bcm2835_gpio_clr(D4_1602);
	if(data_v & 0x20)	bcm2835_gpio_set(D5_1602);
	else				bcm2835_gpio_clr(D5_1602);
	if(data_v & 0x40)	bcm2835_gpio_set(D6_1602);
	else				bcm2835_gpio_clr(D6_1602);
	if(data_v & 0x80)	bcm2835_gpio_set(D7_1602);
	else				bcm2835_gpio_clr(D7_1602);	
	
}

int bcm2835_gpio_fsel(uint8_t pin, uint8_t mode)  
{   
    volatile uint32_t * bcm2835_gpio = (volatile uint32_t *)ioremap(BCM2835_GPIO_BASE, 16);  
    volatile uint32_t * bcm2835_gpio_fsel = bcm2835_gpio + BCM2835_GPFSEL0/4 + (pin/10);  
    uint8_t   shift = (pin % 10) * 3;  
    uint32_t  value = mode << shift; 
    uint32_t  cleanv = 0b111 << shift;
    *bcm2835_gpio_fsel = *bcm2835_gpio_fsel & ~cleanv; 
    *bcm2835_gpio_fsel = *bcm2835_gpio_fsel | value;  
    return 0;  
}  
  
int bcm2835_gpio_set(uint8_t pin)  
{  
    volatile uint32_t * bcm2835_gpio = (volatile uint32_t *)ioremap(BCM2835_GPIO_BASE, 16);  
    volatile uint32_t * bcm2835_gpio_set = bcm2835_gpio + BCM2835_GPSET0/4 + pin/32;  
    uint8_t   shift = pin % 32;  
    uint32_t  value = 1 << shift;  
    *bcm2835_gpio_set = value;  
    return 0;  
}  
  
int bcm2835_gpio_clr(uint8_t pin)  
{  
   //GPIO清除功能物理地址  
    volatile uint32_t * bcm2835_gpio = (volatile uint32_t *)ioremap(BCM2835_GPIO_BASE, 16);  
    volatile uint32_t * bcm2835_gpio_clr = bcm2835_gpio + BCM2835_GPCLR0/4 + pin/32;  
    uint8_t   shift = pin % 32;  
    uint32_t  value = 1 << shift;  
    *bcm2835_gpio_clr = value;  
    return 0;  
}  

static ssize_t device_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int i;
	char mem[20];
	
	ret = copy_from_user(mem, buf, count);
	if(ret < 0)
	{
		printk("Error copy from user! \n");
	}
	for( i=0 ; i<count-2 ; i++)
	{
		L1602_char(mem[0], mem[1]+i, mem[i+2]);
	}
	printk("In tht write function.\n");
	printk("%d %d %c\n", mem[0], mem[1], mem[2]);
	return ret;
}

static int leds_open(struct inode *inode, struct file *filp)  
{  
   return 0;   
}  
  
static long leds_ioctl(struct file*filp, unsigned int cmd, unsigned long arg)  
{  
    switch(cmd)    
    {    
        case 0:     
        	data_1602(0xAA);
            printk("Output 0xAA\n");  
            break;    
        case 1:    
        	data_1602(0x55); 
            printk("Output 0x55\n");  
            break;   
        case 2:    
  
            break; 
  
        default:    
            return-EINVAL;    
    }    
  
    return 0;  
}  
  
static int leds_release(struct inode *inode, struct file *filp)  
{  
    return 0;
}  
  
static const struct file_operations leds_fops = {  
    .owner = THIS_MODULE,  
    .write = device_write,
    .open = leds_open,  
    .unlocked_ioctl = leds_ioctl,  
    .release = leds_release,  
};  
  
static struct miscdevice misc = {  
    .minor =MISC_DYNAMIC_MINOR,  
    .name ="rpi_1602",  
    .fops =&leds_fops,  
};  
  
  
static int __init leds_init(void)  
{  
    int ret;  
   
    ret =misc_register(&misc);  
  	if(ret < 0)
  	{
  		printk("Unable to register misc!\n");
  	}

  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_11, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_13, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_15, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_12, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_29, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_31, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_33, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_35, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_16, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_18, BCM2835_GPIO_FSEL_OUTP);
  	bcm2835_gpio_fsel(RPI_BPLUS_GPIO_J8_37, BCM2835_GPIO_FSEL_OUTP);
  	
  	init_1602();
  	
  	L1602_string(1, 1, "Wait for command");
  	
    printk("ledsinit.\n");  
    return ret;  
}  
  
static void leds_exit(void)  
{    
    misc_deregister(&misc);          
    printk("rpi_1602_exit\n");  
}  
  
module_init(leds_init);  
module_exit(leds_exit);  
  
MODULE_AUTHOR("Hu Chunxu");  
MODULE_LICENSE("GPL");  
