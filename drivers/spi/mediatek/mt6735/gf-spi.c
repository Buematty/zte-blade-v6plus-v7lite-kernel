#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/delay.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/usb.h>
//#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/power_supply.h>

#include <linux/gpio.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include "gf-spi.h"
#include "mt_spi.h"

#define NAVIGATION		1

/*spi device name*/
#define SPI_DEV_NAME   "spidev"
/*device name after register in charater*/
#define DEV_NAME "goodix_fp"
#define GF_INPUT_NAME       "goodix_fp"

#define GF_X18M_TARGET_FW
#define GF_318M_TARGET_CFG

#if defined GF_X16M_TARGET_FW
#define GF_PID "GFx16M"
#define GF_NAV_FRAME_LEN         (1680)    //navigation data
#elif defined GF_X18M_TARGET_FW
#define GF_PID "GFx18M"
#define GF_NAV_FRAME_LEN        (2160)     //navigation data len
#else
#error "have't define any product ID"
#endif
#define GF_PID_LEN 6


struct gf_key_map
{
    char *name;
    unsigned short val;
};

struct gf_key_map key_map[] = 
{
      {  "POWER",  KEY_FINGER_PRINT  },
      {  "HOME" ,  KEY_HOME   },
      {  "MENU" ,  KEY_MENU   },
      {  "BACK" ,  KEY_BACK   },
      {  "UP"   ,  KEY_FINGER_UP     },
      {  "DOWN" ,  KEY_FINGER_DOWN   },
      {  "LEFT" ,  KEY_FINGER_LEFT   },
      {  "RIGHT",  KEY_FINGER_RIGHT  },
      {  "FORCE",  KEY_F9     },
      {  "CLICK",  KEY_FINGER_CLICK   },
};



#define GF_KEY_FF        KEY_FINGER_PRINT
#define GF_KEY_HOME     KEY_HOME
#define GF_KEY_MENU     KEY_MENU
#define GF_KEY_BACK     KEY_BACK
#if (NAVIGATION == 1)
#define GF_UP_KEY       KEY_UP
#define GF_DOWN_KEY KEY_DOWN
#define GF_LEFT_KEY KEY_LEFT
#define GF_RIGHT_KEY    KEY_RIGHT
#define GF_KEY_FORCE    KEY_F9
#define GF_APP_SWITCH   KEY_F19
#endif

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define	CHRD_DRIVER_NAME		"goodix"
#define	CLASS_NAME			    "goodix-spi"
#define SPIDEV_MAJOR			156	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
static DECLARE_BITMAP(minors, N_SPI_MINORS);
#define FW_UPDATE               1
#define ESD_PROTECT             1
#define CFG_UPDATE              1

/**************************debug******************************/
//#define SPI_ASYNC   1

#define DEFAULT_DEBUG   (0x1<<0)
#define SUSPEND_DEBUG   (0x1<<1)
#define SPI_DEBUG       (0x1<<2)
#define TIME_DEBUG      (0x1<<3)
#define FLOW_DEBUG      (0x1<<4)
#define gf_debug(level, fmt, args...) do{ \
    if(g_debug & level) {\
	pr_info("gf:" fmt, ##args); \
    } \
}while(0)

#define FUNC_ENTRY()  gf_debug(FLOW_DEBUG, "gf:%s, entry\n", __func__)
#define FUNC_EXIT()  gf_debug(FLOW_DEBUG,"gf:%s, exit\n", __func__)


static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
/*************************data stream***********************
 *	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
 *     1B      |   1B    |  1B     |    2048B     |  2B      |
 ************************************************************/
static unsigned bufsiz = 8 * (2048+5);
unsigned long g_debug = DEFAULT_DEBUG;
static unsigned char g_mode_switch_flag = 0;
static void gf_esd_switch(struct gf_dev *gf_dev, s32 on);
int chip_version_config=0;

static unsigned char GF_FW[]=
{
#if defined GF_X16M_TARGET_FW
#include "inc/gf_fw_x16M.i"
#elif defined GF_X18M_TARGET_FW
#include "inc/gf_fw_x18M.i"
#else
#error "have't define any product for fw"
#endif
};

static unsigned char GF_CFG1[]=
{
#if defined GF_318M_TARGET_CFG
#include "inc/gf318m_cfg_kaier.i"
#elif defined GF_316M_TARGET_CFG
#include "inc/gf316m_cfg.i"
#elif defined GF_516M_TARGET_CFG
#include "inc/gf516m_cfg.i"
#elif defined GF_518M_TARGET_CFG
#include "inc/gf518m_cfg.i"
#elif defined GF_816M_TARGET_CFG
#include "inc/gf816m_cfg.i"
#else
#error "have't define any product for cfg"
#endif
};
static unsigned char GF_CFG2[]=
{
#include "inc/gf318m_cfg_helitai.i"
};
static unsigned char GF_CFG3[]=
{
#include "inc/gf318m_cfg_syx.i"
};
static unsigned char GF_CFG4[]=
{
#include "inc/gf318m_cfg_oufei.i"
};

#define FW_LENGTH (42*1024)

unsigned char* fw = GF_FW;
unsigned char* cfg = GF_CFG1;

static struct pinctrl *pc;	
static struct pinctrl_state *ps_pwr_on;	
static struct pinctrl_state *ps_pwr_off;	
static struct pinctrl_state *ps_rst_low;	
static struct pinctrl_state *ps_rst_high;	
static struct pinctrl_state *ps_irq_init;	
static struct pinctrl_state *ps_irq_in;	
static struct pinctrl_state *ps_irq_dis;	
static struct pinctrl_state *ps_irq_en;	
static struct pinctrl_state *ps_miso_spi;
static struct pinctrl_state *ps_miso_pullup;

#ifdef CONFIG_FINGERPRINT_ID_GPIO
static unsigned long id_gpio;
static struct pinctrl_state *ps_id_up;	
static struct pinctrl_state *ps_id_down;
#endif
int finger_updown = 0;

struct of_device_id fp_of_match[] = {	
	{ .compatible = "gf,gf3118", },	
	{},
};
static struct task_struct *gf_irq_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int irq_flag = 0;

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

enum
{
	KAIER = 0,
	HELITAI,
	SYX,
	OUFEI_A31,
	KAIER_A31,
	MAX_MANUFACTOR_ID,
};
int manufactory_id = MAX_MANUFACTOR_ID;

void print_16hex(u8 *config, u8 len)
{
    u8 i,j = 0;
    printk("dump hex \n");
    for(i = 0 ; i< len ; i++) {
        printk("0x%x " , config[i]);
        if(j++ == 15) {
            j = 0;
            printk("\n");
        }
    }
    printk("\n");
}

unsigned short checksum(u8 *data, u32 len)
{
    u16 checksum = 0;
    u32 i = 0;         
    for(i =0; i<len; i++) {
        checksum += data[i];
    }
    return checksum;
}


/* -------------------------------------------------------------------- */
/* devfs                                */
/* -------------------------------------------------------------------- */
static ssize_t gf_debug_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    printk("gf:Show.\n");
    return 0;
}
static ssize_t gf_debug_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int debug_level = 0;
    sscanf(buf, "%d", &debug_level);
    printk("gf:Store. debug_level = %d\n", debug_level);
    return strnlen(buf, count);
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, gf_debug_show, gf_debug_store);

static ssize_t gf_chipid_show(struct device *dev, 	
	struct device_attribute *attr, char *buf)
{	
	//u8 reg_value[10+GF_RDATA_OFFSET] = {0};
	//struct spi_device *spi = to_spi_device(dev);
	//struct gf_dev *gf_dev = spi_get_drvdata(spi);
	int ret = 0;	
	//memset(reg_value, 0xFF, 10);	
	//gf_spi_read_bytes(gf_dev, 0x4220, 10, reg_value);	
	//printk("%s,chip version is 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", __func__, reg_value[GF_RDATA_OFFSET], reg_value[GF_RDATA_OFFSET+1], reg_value[GF_RDATA_OFFSET+2], reg_value[GF_RDATA_OFFSET+3]);	
	//if ((0x00 == reg_value[GF_RDATA_OFFSET+3])&& (0x90 == reg_value[GF_RDATA_OFFSET+2]) && (0x08 == reg_value[GF_RDATA_OFFSET+1])){	
	printk("%s,chip version check pass!\n", __func__);	
	ret = sprintf(buf, "0x02,0x08,0x90\n");	
	return ret;
}
static DEVICE_ATTR(chipid, S_IRUGO, gf_chipid_show, NULL);

static ssize_t gf_manufactory_show(struct device *dev, 	
	struct device_attribute *attr, char *buf)
{	
	int ret = 0;
	
	switch(manufactory_id)
	{
	case KAIER:
		ret = sprintf(buf, "kaier\n");
		break;
	case HELITAI:
		ret = sprintf(buf, "helitai\n");
		break;
	case SYX:
		ret = sprintf(buf, "syx\n");
		break;
	case KAIER_A31:
		ret = sprintf(buf, "kaier_a31\n");
		break;
	case OUFEI_A31:
		ret = sprintf(buf, "oufei_a31\n");
		break;
	default:
		ret = sprintf(buf, "unknown\n");
		break;
	}
	
	return ret;
}
static DEVICE_ATTR(manufactor_id, S_IRUGO, gf_manufactory_show, NULL);

static ssize_t gf_updown_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	return (sprintf(buf, "%d", finger_updown));
}
static DEVICE_ATTR(finger_up_down, S_IRUGO, gf_updown_show, NULL);

static struct attribute *gf_debug_attrs[] = {
    &dev_attr_debug.attr,
	&dev_attr_chipid.attr,
	&dev_attr_finger_up_down.attr,
	&dev_attr_manufactor_id.attr,
    NULL
};

static const struct attribute_group gf_debug_attr_group = {
    .attrs = gf_debug_attrs,
    .name = "debug"
};



/*power management*/
int gf_power_on(struct gf_dev* gf_dev)
{
    int rc = 0;

    if (gf_dev->isPowerOn == 0){
    	pinctrl_select_state(gf_dev->pc, gf_dev->irq_init);						
		/*Reset GPIO Output-low before poweron*/			
		pinctrl_select_state(gf_dev->pc, gf_dev->rst_low);			
		msleep(5);			
		/*power on*/		 	
		pinctrl_select_state(gf_dev->pc, gf_dev->pwr_on);						
		msleep(20);			
		/*INT GPIO set floating after poweron and controlled by GFX1XM*/			
		pinctrl_select_state(gf_dev->pc, gf_dev->irq_in);			
		msleep(1);						
		/*Reset GPIO Output-high, GFX1XM works*/			
		pinctrl_select_state(gf_dev->pc, gf_dev->rst_high); 				
		msleep(20);
		gf_dev->isPowerOn = 1;
    }

    pr_info("gf:---- power on ok ----\n");
    
    return rc;
}

int gf_power_off(struct gf_dev* gf_dev)
{
    int rc = 0;			
	if (gf_dev->isPowerOn == 1){
		pinctrl_select_state(gf_dev->pc, gf_dev->irq_dis); 			
		msleep(10);			
		pinctrl_select_state(gf_dev->pc, gf_dev->pwr_off);						
		//msleep(50);
    	gf_dev->isPowerOn = 0;
    }
    pr_info("gf:---- power off ----\n");
    return rc;
}

static void gf_enable_irq(struct gf_dev *gf_dev)
{
        if (gf_dev->irq_enabled) {
                pr_warn("gf:IRQ has been enabled.\n");
        } else {
                enable_irq(gf_dev->spi->irq);
                gf_dev->irq_enabled = 1;
        }
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
        if (gf_dev->irq_enabled) {
                gf_dev->irq_enabled = 0;
                disable_irq(gf_dev->spi->irq);
        } else {
                pr_warn("gf:IRQ has been disabled.\n");
        }
}


#ifdef SPI_ASYNC
static void gf_spi_complete(void *arg)
{
    complete(arg);
}
#endif //SPI_ASYNC

typedef enum {	
	SPEED_500KHZ=0,	
	SPEED_1MHZ,	
	SPEED_2MHZ,	
	SPEED_3MHZ,	
	SPEED_4MHZ,	
	SPEED_6MHZ,	
	SPEED_8MHZ,	
	SPEED_KEEP,	
	SPEED_UNSUPPORTED
}SPI_SPEED;
static struct mt_chip_conf spi_conf_mt65xx = {	
	.setuptime = 15,	
	.holdtime = 15,	
	.high_time = 21, 	
	.low_time = 21,		
	.cs_idletime = 20,	
	.ulthgh_thrsh = 0,	
	.cpol = 0,	
	.cpha = 0,	
	.rx_mlsb = 1,	
	.tx_mlsb = 1,	
	.tx_endian = 0,	
	.rx_endian = 0,	
	.com_mod = FIFO_TRANSFER,	
	.pause = 0,	
	.finish_intr = 1,	
	.deassert = 0,	
	.ulthigh = 0,	
	.tckdly = 0,
};
static void gf_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag)
{	
	struct mt_chip_conf *mcc = &spi_conf_mt65xx;	
	if(flag == 0) {		
		mcc->com_mod = FIFO_TRANSFER;	
	} else {		
		mcc->com_mod = DMA_TRANSFER;	
	}	
	switch(speed)	
	{		
		case SPEED_500KHZ:			
			mcc->high_time = 120;			
			mcc->low_time = 120;			
			break;		
		case SPEED_1MHZ:			
			mcc->high_time = 60;			
			mcc->low_time = 60;			
			break;		
		case SPEED_2MHZ:			
			mcc->high_time = 30;			
			mcc->low_time = 30;			
			break;		
		case SPEED_3MHZ:			
			mcc->high_time = 20;			
			mcc->low_time = 20;			
			break;		
		case SPEED_4MHZ:			
			mcc->high_time = 15;			
			mcc->low_time = 15;			
			break;		
		case SPEED_6MHZ:			
			mcc->high_time = 10;			
			mcc->low_time = 10;			
			break;		
		case SPEED_8MHZ:		    
			mcc->high_time = 8;			
			mcc->low_time = 8;			
			break;  		
		case SPEED_KEEP:		
		case SPEED_UNSUPPORTED:			
			break;	
		}	
	if(spi_setup(spi) < 0){		
		pr_err("gf:Failed to set spi.");	
	}
}

void gf_spi_setup(struct gf_dev *gf_dev, int max_speed_hz)
{

    gf_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
    gf_dev->spi->max_speed_hz = max_speed_hz;
    gf_dev->spi->bits_per_word = 8;
	gf_dev->spi->controller_data  = (void*)&spi_conf_mt65xx;
    spi_setup(gf_dev->spi);
}

/**********************************************************
 *Message format:
 *	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
 *    1B         |   1B    |  1B    |  length       |
 *
 * read buffer length should be 1 + 1 + 1 + data_length
 ***********************************************************/
int gf_spi_write_bytes(struct gf_dev *gf_dev,
        u16 addr, u32 data_len, u8 *tx_buf)
{
#ifdef SPI_ASYNC
    DECLARE_COMPLETION_ONSTACK(read_done);
#endif
    struct spi_message msg;
    struct spi_transfer *xfer;
    int ret = 0;
	u32  package_num = (data_len + 2*GF_WDATA_OFFSET)>>MTK_SPI_ALIGN_MASK_NUM;
    u32  reminder = (data_len + 2*GF_WDATA_OFFSET) & MTK_SPI_ALIGN_MASK;
    u8 *reminder_buf = NULL;
    u8   twice = 0;


	if ((data_len + GF_WDATA_OFFSET) > 32) {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	
    if((package_num > 0) && (reminder != 0)) {
        twice = 1; 
        /*copy the reminder data to temporarity buffer.*/
        reminder_buf = kzalloc(reminder + GF_WDATA_OFFSET, GFP_KERNEL);
        if(reminder_buf == NULL ) {
            pr_err("gf:No memory for exter data.");
            return -ENOMEM;
        }    
        memcpy(reminder_buf + GF_WDATA_OFFSET, tx_buf + 2*GF_WDATA_OFFSET+data_len - reminder, reminder);
        gf_debug(SPI_DEBUG,"gf:w-reminder:0x%x-0x%x,0x%x", reminder_buf[GF_WDATA_OFFSET],reminder_buf[GF_WDATA_OFFSET+1],
                reminder_buf[GF_WDATA_OFFSET + 2]); 
        xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
    } else {
        twice = 0; 
        xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
    }  	
    if( xfer == NULL){
        pr_warn("gf:No memory for command.\n");
		 if(reminder_buf != NULL)
            kfree(reminder_buf);
        return -ENOMEM;
    }

    /*send gf command to device.*/
    spi_message_init(&msg);
    tx_buf[0] = GF_W;
    tx_buf[1] = (u8)((addr >> 8)&0xFF);
    tx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = tx_buf;
    if(twice == 1) {
        xfer[0].len = package_num << MTK_SPI_ALIGN_MASK_NUM;
        spi_message_add_tail(&xfer[0], &msg);
        addr += (data_len - reminder + GF_WDATA_OFFSET);
        reminder_buf[0] = GF_W;
        reminder_buf[1] = (u8)((addr >> 8)&0xFF);
        reminder_buf[2] = (u8)(addr & 0xFF);
        xfer[1].tx_buf = reminder_buf;
        xfer[1].len = reminder + 2*GF_WDATA_OFFSET;
        //xfer[1].delay_usecs = 5;
        spi_message_add_tail(&xfer[1], &msg);
    } else {
    	xfer[0].len = data_len + 3;
    	spi_message_add_tail(&xfer[0], &msg);
    }
#ifdef SPI_ASYNC
    msg.complete = gf_spi_complete;
    msg.context = &read_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);
    if(ret == 0) {
        wait_for_completion(&read_done);
        if(msg.status == 0)
            ret = msg.actual_length - GF_WDATA_OFFSET;
    }
#else
    ret = spi_sync(gf_dev->spi, &msg);
    if(ret == 0) {
		if(twice == 1)
            ret = msg.actual_length - 2*GF_WDATA_OFFSET;
        else
        	ret = msg.actual_length - GF_WDATA_OFFSET;
    }else  {
        gf_debug(DEFAULT_DEBUG,"write async failed. ret = %d", ret);
    }
#endif
    gf_debug(SPI_DEBUG, "ret = %d, actual_length = %d\n", ret, msg.actual_length);

    if(xfer != NULL){
		kfree(xfer);
        xfer = NULL;
    }
    if(reminder_buf != NULL) {
        kfree(reminder_buf);
        reminder_buf = NULL;
    }
    return ret;
}

/*************************************************************
 *First message:
 *	write cmd   |  ADDR_H |ADDR_L  |
 *    1B         |   1B    |  1B    |
 *Second message:
 *	read cmd   |  data stream  |
 *    1B        |   length    |
 *
 * read buffer length should be 1 + 1 + 1 + 1 + data_length
 **************************************************************/
int gf_spi_read_bytes(struct gf_dev *gf_dev,
        u16 addr, u32 data_len, u8 *rx_buf)
{
#ifdef SPI_ASYNC
    DECLARE_COMPLETION_ONSTACK(write_done);
#endif //SPI_ASYNC
    struct spi_message msg;
    struct spi_transfer *xfer;
	u32  package_num = (data_len + 1 + 1)>>MTK_SPI_ALIGN_MASK_NUM;
    u32  reminder = (data_len + 1 + 1) & MTK_SPI_ALIGN_MASK;
    u8 *reminder_buf = NULL;
    u8   twice = 0;
    int ret = 0;

    if((package_num > 0) && (reminder != 0)) {
        twice = 1;
        reminder_buf = kzalloc(reminder + GF_RDATA_OFFSET, GFP_KERNEL);
        if(reminder_buf == NULL ) {
            pr_err("gf:No memory for exter data.");
            return -ENOMEM;
        }
        xfer = kzalloc(sizeof(*xfer)*4, GFP_KERNEL);
    } else {
        twice = 0;
        xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
    }

    if( xfer == NULL){
        pr_warn("gf:No memory for command.\n");
		if(reminder_buf != NULL)
            kfree(reminder_buf);
        return -ENOMEM;
    }
	if ((data_len + GF_WDATA_OFFSET) > 32) {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	
    /*send gf command to device.*/
    spi_message_init(&msg);
    rx_buf[0] = GF_W;
    rx_buf[1] = (u8)((addr >> 8)&0xFF);
    rx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = rx_buf;
    xfer[0].len = 3;
    spi_message_add_tail(&xfer[0], &msg);

    /*if wanted to read data from gf.
     *Should write Read command to device
     *before read any data from device.
     */
    //memset(rx_buf, 0xff, data_len);
    spi_sync(gf_dev->spi, &msg);
    spi_message_init(&msg);
    rx_buf[4] = GF_R;
    xfer[1].tx_buf = &rx_buf[4];

    xfer[1].rx_buf = &rx_buf[4];
    if(twice == 1)
        xfer[1].len = (package_num << MTK_SPI_ALIGN_MASK_NUM);
    else
        xfer[1].len = data_len + 1;
    spi_message_add_tail(&xfer[1], &msg);

    if(twice == 1) {
        addr += data_len - reminder + 1;
        reminder_buf[0] = GF_W;
        reminder_buf[1] = (u8)((addr >> 8)&0xFF);
        reminder_buf[2] = (u8)(addr & 0xFF);
        xfer[2].tx_buf = reminder_buf;
        xfer[2].len = 3;
        spi_message_add_tail(&xfer[2], &msg);
        spi_sync(gf_dev->spi, &msg);
        spi_message_init(&msg);
        reminder_buf[4] = GF_R;
        xfer[3].tx_buf = &reminder_buf[4];
        xfer[3].rx_buf = &reminder_buf[4];
        xfer[3].len = reminder + 1 + 1;
        spi_message_add_tail(&xfer[3], &msg);
    }

#ifdef SPI_ASYNC
    msg.complete = gf_spi_complete;
    msg.context = &write_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);
    if(ret == 0) {
        wait_for_completion(&write_done);
        if(msg.status == 0)
            ret = msg.actual_length - 1;//GF_RDATA_OFFSET;
    }
#else
    ret = spi_sync(gf_dev->spi, &msg);
    if(ret == 0) {
        if(twice == 1) {
            gf_debug(SPI_DEBUG,"reminder:0x%x:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x", reminder_buf[0], reminder_buf[1],
                    reminder_buf[2], reminder_buf[3],reminder_buf[4],reminder_buf[5],reminder_buf[6],reminder_buf[7]);
            memcpy(rx_buf + GF_RDATA_OFFSET + data_len - reminder + 1, reminder_buf + GF_RDATA_OFFSET, reminder);
            ret = data_len;//msg.actual_length - 1; //8 
        } else {
            ret = data_len;//msg.actual_length - 1; //4
        }
    }else {
        pr_err("gf: read failed. ret = %d", ret);
    }
#endif
    gf_debug(SPI_DEBUG, "ret = %d, actual_length = %d\n", ret, msg.actual_length);
    kfree(xfer);
    if(xfer != NULL)
        xfer = NULL;
    if(reminder_buf != NULL) {
        kfree(reminder_buf);
        reminder_buf = NULL;
    }

    return ret;
}

static int gf_spi_read_byte(struct gf_dev *gf_dev, u16 addr, u8 *value)
{
    int status = 0;
    mutex_lock(&gf_dev->buf_lock);

    status = gf_spi_read_bytes(gf_dev, addr, 1, gf_dev->buffer);
    *value = gf_dev->buffer[GF_RDATA_OFFSET];
    gf_debug(SPI_DEBUG, "value = 0x%x, buffer[3] = 0x%x\n", *value, gf_dev->buffer[3]);
    mutex_unlock(&gf_dev->buf_lock);
    return status;
}
static int gf_spi_write_byte(struct gf_dev *gf_dev, u16 addr, u8 value)
{
    int status = 0;
    mutex_lock(&gf_dev->buf_lock);
    gf_dev->buffer[GF_WDATA_OFFSET] = value;
    status = gf_spi_write_bytes(gf_dev, addr, 1, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);
    return status;
}

/**************************nagv***********************************/
int get_nav_buf(struct gf_dev *gf_dev, struct gf_nagv *gf_nagv)
{
    unsigned char lbstatus = 0;
    int ret = 0;
    unsigned char frameNum = 0;
    unsigned char pNavFrameNum;

    if(gf_nagv->buf == NULL) {
        gf_debug(DEFAULT_DEBUG, "get_package_data buf is NULL.\n");
        return -EINVAL;
    }
    /* read status and frame num buffered*/
    gf_spi_read_byte(gf_dev, GF_LBSTATUS_ADDR, &lbstatus);
    gf_spi_read_byte(gf_dev, GF_NAV_FRAMENUM_ADDR, &pNavFrameNum);
    gf_debug(DEFAULT_DEBUG, "NAV: lbstatus: 0x%x, frame_num:0x%x\n", lbstatus, pNavFrameNum);

    frameNum = pNavFrameNum & 0x0F;
    if(frameNum)
    {/* read navigation data */
        gf_debug(DEFAULT_DEBUG, "NAV: read %d frame data.\n", frameNum);
        mutex_lock(&gf_dev->buf_lock);
        gf_dev->spi->max_speed_hz=4*1000*1000;
		gf_spi_set_mode(gf_dev->spi, SPEED_4MHZ, 0);
        if(lbstatus & GF_NAV_BUF_MASK)
            ret = gf_spi_read_bytes(gf_dev, GF_NAV_BUF1_ADDR, (frameNum*GF_NAV_FRAME_LEN), gf_dev->buffer);
        else
            ret = gf_spi_read_bytes(gf_dev, GF_NAV_BUF0_ADDR, (frameNum*GF_NAV_FRAME_LEN), gf_dev->buffer);

        gf_dev->spi->max_speed_hz=1*1000*1000;
		gf_spi_set_mode(gf_dev->spi, SPEED_1MHZ, 0);
        mutex_unlock(&gf_dev->buf_lock);

        if(!ret) {
            gf_debug(DEFAULT_DEBUG, "NAV:Failed to read data. len = %d. ret = %d, lbstatus = %d, nav_frm_num = %d\n",
                               GF_NAV_FRAME_LEN, ret, lbstatus, pNavFrameNum);
                return -EFAULT;
        }
/*
        unsigned int checksum16 = 0;
        unsigned char ck_r10 = 0;
        unsigned char ck_r11 = 0;
        unsigned char ck_r20 = 0;
        unsigned char ck_r21 = 0;

        checksum16 = checksum((unsigned char*)(gf_dev->buffer + GF_RDATA_OFFSET), 
                            (unsigned int)(GF_NAV_FRAME_LEN*frameNum));
        gf_spi_read_byte(gf_dev, 0x9A9A, &ck_r10);
        gf_spi_read_byte(gf_dev, 0x9A9B, &ck_r11);
        gf_spi_read_byte(gf_dev, 0xB49A, &ck_r20);
        gf_spi_read_byte(gf_dev, 0xB49B, &ck_r21);
        pr_info("NAV: checksum:0x%x, ck_r1:[0x%x-0x%x], ck_r2:[0x%x-0x%x].\n", checksum16, ck_r10, ck_r11, ck_r20, ck_r21);
*/     
        if(copy_to_user(gf_nagv->buf, (gf_dev->buffer + GF_RDATA_OFFSET), GF_NAV_FRAME_LEN*frameNum)){
            pr_err("gf:Failed to copy data from kernel to user.\n");
            return -EFAULT;
        }
        if(copy_to_user(gf_nagv->frame_num, &pNavFrameNum, 1)){
            pr_err("gf:Failed to copy data from kernel to user.\n");
            return -EFAULT;
        }
    }
    else
    {
        gf_debug(DEFAULT_DEBUG, "NAV: no data to read.\n");
    }
 
    /*tell the hardware. Data has been read already.*/
    gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, 0);
    return 0;
}


int gf_read_nav_base(struct gf_dev *gf_dev, struct gf_nagv *gf_nagv)
{
    unsigned char status = 0;
    unsigned char lbstatus = 0;
    int retry = 0;
    int ret = 0;
    if(gf_nagv->buf == NULL)  {
        gf_debug(DEFAULT_DEBUG, "get_nav_base is NULL.\n");
        return -EINVAL;
    }


    disable_irq(gf_dev->spi->irq);
    gf_spi_write_byte(gf_dev, GF_MODE_STATUS, GF_PRENAV_MODE);
    do
    {
        /* read status and frame num buffered*/
        gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &status);
        gf_spi_read_byte(gf_dev, GF_LBSTATUS_ADDR, &lbstatus);
        //pr_info("NAV: retry: %d, status: 0x%x, lbstatus:0x%x\n", retry, status, lbstatus);

        if( (status & GF_BUF_STA_MASK) && (lbstatus & GF_NAV_MASK))
        {/* read navigation base */
            mutex_lock(&gf_dev->buf_lock);
            ret = gf_spi_read_bytes(gf_dev, GF_NAV_BUF0_ADDR, GF_NAV_FRAME_LEN, gf_dev->buffer);
            mutex_unlock(&gf_dev->buf_lock);

            if(!ret) {
                gf_debug(DEFAULT_DEBUG, "NAV:Failed to read nav base. len = %d. ret = %d\n", GF_NAV_FRAME_LEN, ret);
                gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, status&0x7F);
                enable_irq(gf_dev->spi->irq);
                return -EFAULT;
            }
            else
            {
                gf_spi_write_byte(gf_dev, GF_PAUSE_ADDR, 1);
                if(copy_to_user(gf_nagv->buf, (gf_dev->buffer + GF_RDATA_OFFSET), GF_NAV_FRAME_LEN)){
                    pr_err("gf:Failed to copy data from kernel to user.\n");
                    enable_irq(gf_dev->spi->irq);
                    return -EFAULT;
                }
                gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, status&0x7F);
                break;
            }
        }
        else
        {
            mdelay(3);
        }
    }while( retry++  < 20);
    gf_debug(DEFAULT_DEBUG, " NAV base out: retry : %d\n", retry);
    gf_spi_write_byte(gf_dev, GF_MODE_STATUS, GF_IMAGE_MODE);
    enable_irq(gf_dev->spi->irq);
    return 0;
}


/**************************nagv end***********************************/

/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct gf_dev *gf_dev = filp->private_data;
    ssize_t			status = 0;
    //long int t1, t2;
    FUNC_ENTRY();
    pr_info("%s\n", __func__);
    if ((count > bufsiz)||(count == 0)) {
        pr_warn("gf:Max size for write buffer is %d. wanted length is %d\n", bufsiz, (int)count);
        FUNC_EXIT();
        return -EMSGSIZE;
    }

    gf_dev = filp->private_data;
    	   
    mutex_lock(&gf_dev->fb_lock);
    mutex_lock(&gf_dev->buf_lock);
    gf_dev->spi->max_speed_hz=4*1000*1000;
	gf_spi_set_mode(gf_dev->spi, SPEED_4MHZ, 0);
    //t1 = ktime_to_us(ktime_get());
    status = gf_spi_read_bytes(gf_dev, GF_BUFFER_DATA, count, gf_dev->buffer);
    gf_dev->spi->max_speed_hz=1*1000*1000;
	gf_spi_set_mode(gf_dev->spi, SPEED_1MHZ, 0);
    if(status > 0) {
        unsigned long missing = 0;
        missing = copy_to_user(buf, gf_dev->buffer + GF_RDATA_OFFSET, status);
        if(missing == status)
            status = -EFAULT;
    } else {
        pr_err("gf:Failed to read data from SPI device.\n");
        status = -EFAULT;
    }
    // t2 = ktime_to_us(ktime_get());
    //pr_info("read time use: %ld\n", t2-t1);
    mutex_unlock(&gf_dev->buf_lock);			   
    mutex_unlock(&gf_dev->fb_lock);

    return status;
}

/* Write-only message with current device setup */
static ssize_t gf_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
    struct gf_dev *gf_dev = filp->private_data;
    ssize_t			status = 0;
    FUNC_ENTRY();
    if(count > bufsiz) {
        pr_warn("gf:Max size for write buffer is %d\n", bufsiz);
        return -EMSGSIZE;
    }

    mutex_lock(&gf_dev->fb_lock);
    mutex_lock(&gf_dev->buf_lock);
    status = copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET, buf, count);
    if(status == 0) {
        gf_dev->spi->max_speed_hz=2*1000*1000;
		gf_spi_set_mode(gf_dev->spi, SPEED_2MHZ, 0);
        status = gf_spi_write_bytes(gf_dev, GF_BUFFER_DATA, count, gf_dev->buffer);
    } else {
        pr_err("gf:Failed to xfer data through SPI bus.\n");
        status = -EFAULT;
    }
    mutex_unlock(&gf_dev->buf_lock);		      
    mutex_unlock(&gf_dev->fb_lock);
    FUNC_EXIT();
    return status;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gf_dev *gf_dev = NULL;
    struct gf_key gf_key = { 0 };
    struct gf_nagv gf_nagv = { 0 };
    struct gf_ioc_transfer ioc;
    int			err = 0;
    u32			tmp = 0;
    int 		retval = 0;
    int i;

    FUNC_ENTRY();
    if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
        return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,
                (void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,
                (void __user *)arg, _IOC_SIZE(cmd));
    if (err)
        return -EFAULT;

    gf_dev = (struct gf_dev *)filp->private_data;

    switch(cmd) {
        case GF_IOC_CMD:
            /*copy command data from user to kernel.*/
            if(copy_from_user((struct gf_ioc_transfer *)(&ioc),
                        (struct gf_ioc_transfer*)arg, sizeof(struct gf_ioc_transfer))){
                pr_err("Failed to copy command from user to kernel.\n");
                retval = -EFAULT;
                break;
            }

            if((ioc.len > bufsiz)||(ioc.len == 0)) {
                pr_warn("gf:The request length[%d] is longer than supported maximum buffer length[%d].\n",
                        ioc.len, bufsiz);
                retval = -EMSGSIZE;
                break;
            }

 
            mutex_lock(&gf_dev->fb_lock);
            mutex_lock(&gf_dev->buf_lock);
            gf_dev->spi->max_speed_hz=1*1000*1000;
			gf_spi_set_mode(gf_dev->spi, SPEED_1MHZ, 0);
            if(ioc.cmd == GF_R) {
                /*if want to read data from hardware.*/
                pr_info("gf: Read data from 0x%x, len = 0x%x buf = 0x%p\n", ioc.addr, ioc.len, ioc.buf);
                gf_spi_read_bytes(gf_dev, ioc.addr, ioc.len, gf_dev->buffer);
                if(copy_to_user(ioc.buf, gf_dev->buffer + GF_RDATA_OFFSET, ioc.len)) {
                    pr_err("gf: Failed to copy data from kernel to user.\n");
                    retval = -EFAULT;
                    mutex_unlock(&gf_dev->buf_lock);
                    mutex_unlock(&gf_dev->fb_lock);
                    break;
                }
                //print_16hex(ioc.buf, ioc.len);
            } else if (ioc.cmd == GF_W) {
                /*if want to read data from hardware.*/
                //print_16hex(ioc.buf, ioc.len);
                //pr_info("Write data from 0x%x, len = 0x%x\n", ioc.addr, ioc.len);

                if(ioc.addr == 0x8043)
                {
                    if (gf_dev->mode == GF_FF_MODE){
                        g_mode_switch_flag = 1;
                    }
                    gf_dev->mode = ioc.buf[0];

                    //if(gf_dev->fb_black) {
                    //    pr_info("In screen off status, chip mode control by driver");
                    //    mutex_unlock(&gf_dev->buf_lock);
                    //    mutex_unlock(&gf_dev->fb_lock);
                    //    break;
                    //}

                    gf_debug(DEFAULT_DEBUG, "gf set mode 0x%x \n", gf_dev->mode);
                }

                if(copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET, ioc.buf, ioc.len)){
                    pr_err("gf: Failed to copy data from user to kernel.\n");
                    retval = -EFAULT;
                    mutex_unlock(&gf_dev->buf_lock);
                    mutex_unlock(&gf_dev->fb_lock);
                    break;
                }

                gf_spi_write_bytes(gf_dev, ioc.addr, ioc.len, gf_dev->buffer);
            } else {
                pr_warn("gf: Error command for gf.\n");
            }

            mutex_unlock(&gf_dev->buf_lock);
            mutex_unlock(&gf_dev->fb_lock);
			if ((ioc.cmd == GF_W) && (ioc.addr == 0x8043)) {
				if (gf_dev->mode == GF_IMAGE_MODE)
					gf_esd_switch(gf_dev, 1);
				else				
					gf_esd_switch(gf_dev, 0);
			}
			
            break;
        case GF_IOC_REINIT:
            disable_irq(gf_dev->spi->irq);
            gf_hw_reset(gf_dev);
            enable_irq(gf_dev->spi->irq);
            gf_dev->irq_enabled = 1;

            gf_debug(FLOW_DEBUG,"wake-up gf\n");
            break;
        case GF_IOC_SETSPEED:
            retval = __get_user(tmp, (u32 __user*)arg);
            if(tmp > 8*1000*1000) {
                pr_warn("gf: The maximum SPI speed is 8MHz.\n");
                retval = -EMSGSIZE;
                break;
            }
            if(retval == 0) {
				SPI_SPEED speed;
                mutex_lock(&gf_dev->fb_lock);		
     	        mutex_lock(&gf_dev->buf_lock);
                gf_dev->spi->max_speed_hz=tmp;
				speed = (tmp == 500*1000) ? SPEED_500KHZ : (tmp == 1000*1000) ? SPEED_1MHZ : 
					(tmp == 2*1000*1000) ? SPEED_2MHZ : (tmp == 3*1000*1000) ? SPEED_3MHZ :
					(tmp == 4*1000*1000) ? SPEED_4MHZ : (tmp == 6*1000*1000) ? SPEED_6MHZ :
					(tmp == 8*1000*1000) ? SPEED_8MHZ : SPEED_KEEP;
				gf_spi_set_mode(gf_dev->spi, speed, 0);
	        mutex_unlock(&gf_dev->buf_lock);
                mutex_unlock(&gf_dev->fb_lock);
    
                gf_debug(DEFAULT_DEBUG, "spi speed changed to %d\n", tmp);
            }
            break;
        case GF_IOC_POWER_OFF:
            //disable_irq(gf_dev->spi->irq);
            //mutex_lock(&gf_dev->fb_lock);	
            //POWER OFF	
            //gf_power_off(gf_dev);
            //mutex_unlock(&gf_dev->fb_lock);		

            gf_debug(DEFAULT_DEBUG,"gf device disabled\n");
            break;

        case GF_IOC_POWER_ON:
            //disable_irq(gf_dev->spi->irq);
            //mutex_lock(&gf_dev->fb_lock);	
            //POWER ON	
            //gf_power_on(gf_dev);
            //mutex_unlock(&gf_dev->fb_lock);	

            //enable_irq(gf_dev->spi->irq);
            //gf_dev->irq_enabled = 1;
            gf_debug(DEFAULT_DEBUG,"gf device enabled\n");
            break;
        case GF_IOC_DISABLE_IRQ:
			gf_debug(DEFAULT_DEBUG,"IOCTL disable irq");
            gf_disable_irq(gf_dev);
            break;
        case GF_IOC_ENABLE_IRQ:
			gf_debug(DEFAULT_DEBUG,"IOCTL enable irq");
            gf_enable_irq(gf_dev);
            break;
        case GF_IOC_SENDKEY:
            if (copy_from_user
                    (&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
                pr_warn("gf: Failed to copy data from user space.\n");
                retval = -EFAULT;
                break;
            }

            for(i = 0; i< ARRAY_SIZE(key_map); i++) {
                if(key_map[i].val == gf_key.key){
                    input_report_key(gf_dev->input, gf_key.key, gf_key.value);
                    input_sync(gf_dev->input);
                    break;
                }   
            }   

            if(i == ARRAY_SIZE(key_map)) {
                pr_warn("gf: key %d not support yet \n", gf_key.key);
                retval = -EFAULT; 
            } 

            break;
        case GF_IOC_GET_NAGV_BASE:
            if (copy_from_user
                    (&gf_nagv, (struct gf_nagv *)arg, sizeof(struct gf_nagv))) {
                pr_warn("gf: Failed to copy data from user space.\n");
                retval = -EFAULT;
                break;
            }
            retval = gf_read_nav_base(gf_dev, &gf_nagv);
            break;
        case GF_IOC_GET_NAGV_DATA:
            if (copy_from_user
                    (&gf_nagv, (struct gf_nagv *)arg, sizeof(struct gf_nagv))) {
                pr_warn("gf: Failed to copy data from user space.\n");
                retval = -EFAULT;
                break;
            }
            retval = get_nav_buf(gf_dev, &gf_nagv);
            break;
        default:
            pr_warn("gf: doesn't support this command(%d)\n", cmd);
            break;
    }
    FUNC_EXIT();
    return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
     return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct gf_dev *gf_dev = filp->private_data;
    gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &gf_dev->buf_status);
    if((gf_dev->buf_status & GF_BUF_STA_MASK) == GF_BUF_STA_READY) {
        return (POLLIN|POLLRDNORM);
    } else {
        gf_debug(DEFAULT_DEBUG, "Poll no data.\n");
    }
    return 0;
}

static bool hw_config(struct gf_dev *gf_dev)
{
#ifdef CONFIG_FINGERPRINT_ID_GPIO
	int id1 = -1;	
	int id2 = -1;	
	gpio_request(gf_dev->id_gpio, "fp_vendor_id");	
	pinctrl_select_state(gf_dev->pc, gf_dev->id_up);	
	id1= gpio_get_value(gf_dev->id_gpio);	
	pinctrl_select_state(gf_dev->pc, gf_dev->id_down);	
	id2= gpio_get_value(gf_dev->id_gpio);	
	gf_debug(DEFAULT_DEBUG, "id = %lu id_up = %d, id_down = %d", gf_dev->id_gpio, id1, id2);	
	//gpio_free(gf_dev->id_gpio);
	
	if(id1 != id2) {			
		gf_debug(DEFAULT_DEBUG, "%s, send kaier config\n", __func__);			
		cfg = GF_CFG1;	
		manufactory_id = KAIER;
	} else if (id1 == 1) {			
		gf_debug(DEFAULT_DEBUG, "%s, send helitai config\n", __func__);			
		cfg = GF_CFG2;	
		manufactory_id = HELITAI;
	} else if (id1 == 0) {			
		gf_debug(DEFAULT_DEBUG, "send syx config\n");			
		cfg = GF_CFG3;	
		manufactory_id = SYX;
	} else {	
		//default config is kaier			
		gf_debug(DEFAULT_DEBUG, "Vendor ID not matched!!!  send kaier\n");			
		cfg = GF_CFG1;		
		manufactory_id = MAX_MANUFACTOR_ID;
	}	
#else
	cfg = GF_CFG4;
	manufactory_id = OUFEI_A31;

#endif
    mutex_lock(&gf_dev->buf_lock);
    memcpy(gf_dev->buffer + GF_WDATA_OFFSET, cfg, GF_CFG_LEN);
    gf_spi_write_bytes(gf_dev, GF_CFG_ADDR, GF_CFG_LEN, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);

    return true;
}

static int isUpdate(struct gf_dev *gf_dev)
{
    unsigned char version[16];
    unsigned int ver_fw = 0;
    unsigned int ver_file = 0;
    unsigned char* fw = GF_FW;
    unsigned char fw_running = 0;
    const unsigned char OFFSET = 7;

    msleep(300);
    gf_spi_read_byte(gf_dev, 0x41e4, &fw_running);
    gf_debug(DEFAULT_DEBUG, "%s: 0x41e4 = 0x%x\n", __func__, fw_running);
    if(fw_running == 0xbe) {
        /*firmware running*/
        ver_file = (int)(fw[12] & 0xF0) <<12;
        ver_file |= (int)(fw[12] & 0x0F)<<8;
        ver_file |= fw[13];	//get the fw version in the i file;

        /*In case we want to upgrade to a special firmware. Such as debug firmware.*/
        if(ver_file != 0x5a5a) {
            mutex_lock(&gf_dev->buf_lock);
            gf_spi_read_bytes(gf_dev,0x8000,16,gf_dev->buffer);
            memcpy(version, gf_dev->buffer + GF_RDATA_OFFSET, 16);
            mutex_unlock(&gf_dev->buf_lock);
            if(memcmp(version, GF_PID, GF_PID_LEN)) {
                gf_debug(DEFAULT_DEBUG, "version: 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", version[0], version[1],
                        version[2], version[3], version[4], version[5]);
                return 1;
            }
            if((version[OFFSET]>9) || ((version[OFFSET + 1])>9)) {
                gf_debug(DEFAULT_DEBUG, "version: 8-0x%x; 9-0x%x\n", version[OFFSET], version[OFFSET + 1]);
                return 1;
            }

            //get the current fw version
            ver_fw  = (unsigned int)version[OFFSET] << 16;
            ver_fw |= (unsigned int)version[OFFSET + 1] << 8;
            ver_fw |= (unsigned int)version[OFFSET + 2];
            gf_debug(DEFAULT_DEBUG, "ver_fw: 0x%06x; ver_file:0x%06x\n", ver_fw, ver_file);

            if(ver_fw == ver_file){
                /*If the running firmware is or ahead of the file's firmware. No need to do upgrade.*/
                return 0;
            }
        }
        gf_debug(DEFAULT_DEBUG, "Current Ver: 0x%x, Upgrade to Ver: 0x%x\n", ver_fw, ver_file);
    }else {
        /*no firmware.*/
        gf_debug(DEFAULT_DEBUG, "No running firmware. Value = 0x%x\n", fw_running);
    }
    return 1;
}


static u8 is_9p_ready_ok(struct gf_dev *gf_dev)
{
    u8 tmpBuf[16] = {0};
    u8 *ptr =NULL;
    u16 time_out = 0;
    gf_spi_read_bytes(gf_dev, 0x4220, 4, tmpBuf);

    ptr = &tmpBuf[GF_RDATA_OFFSET];


    while(ptr[0] !=0x02 || ptr[1] !=0x08 || ptr[2] !=0x90 || ptr[3] !=0x00)
    {
        time_out++;
        if (time_out > 3)
        {
            return 0;
        }
		msleep(10);
        gf_spi_read_bytes(gf_dev, 0x4220, 4, tmpBuf);
        ptr = &tmpBuf[GF_RDATA_OFFSET];
    }  
    gf_debug(DEFAULT_DEBUG, "%s , timeout = %d\n",__func__,  time_out);
    return 1;
}


#if 0
static int gf_fb_state_chg_callback(struct notifier_block *nb,
        unsigned long val, void *data)
{
    struct gf_dev *gf_dev;
    struct fb_event *evdata = data;
    unsigned int blank;

    if (val != FB_EARLY_EVENT_BLANK)
        return 0;
    pr_info("[info] call back %s value = %d\n", __func__, (int)val);

    gf_dev = container_of(nb, struct gf_dev, notifier);
    if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && gf_dev) {
        blank = *(int *)(evdata->data);
        switch (blank) {
            case FB_BLANK_POWERDOWN:
                /*set mode*/
                pr_info("fbcall back, set mode to FF mode \n");
                mutex_lock(&gf_dev->fb_lock);
                gf_spi_write_byte(gf_dev, GF_MODE_STATUS, GF_FF_MODE);
                gf_dev->mode = GF_FF_MODE;
                mutex_unlock(&gf_dev->fb_lock);
                gf_dev->fb_black = 1;
                break;
            case FB_BLANK_UNBLANK:
                /*set mode*/
                //pr_info("fbcall back, recover mode to %d \n", gf_dev->mode);
                //gf_spi_write_byte(gf_dev, GF_MODE_STATUS, gf_dev->mode);
                gf_dev->fb_black = 0;
                break;
            default:
                pr_info("%s defalut\n", __func__);
                break;
        }
    }
    return NOTIFY_OK;
}


static struct notifier_block gf_noti_block = {
    .notifier_call = gf_fb_state_chg_callback,
};
#endif

static void gf_reg_key_kernel(struct gf_dev *gf_dev)
{
    int i;

    set_bit(EV_KEY, gf_dev->input->evbit); //tell the kernel is key event
    for(i = 0; i< ARRAY_SIZE(key_map); i++) {
        set_bit(key_map[i].val, gf_dev->input->keybit);
    }

    gf_dev->input->name = GF_INPUT_NAME;
    if (input_register_device(gf_dev->input)) {
		input_free_device(gf_dev->input);
        pr_warn("gf:Failed to register GF as input device.\n");
    }
}



static int gf_fw_update_init(struct gf_dev *gf_dev)
{
    u8 retry_cnt = 5;
    u8 value;

    //    disable_irq(gf_dev);
    //    gf_hw_reset(gf_dev);    
    //    enable_irq(gf_dev);

    while(retry_cnt--)
    {
        //set spi miso input pull up
        gf_miso_pullup(gf_dev);

        //reset and delay 5ms
        pinctrl_select_state(gf_dev->pc, gf_dev->rst_low);
        mdelay(5);
        pinctrl_select_state(gf_dev->pc, gf_dev->rst_high);
        mdelay(1);

        //recover miso back spi
        gf_miso_backnal(gf_dev);


        gf_spi_setup(gf_dev, 1000*1000);
		gf_spi_set_mode(gf_dev->spi, SPEED_1MHZ, 0);

        if(!is_9p_ready_ok(gf_dev)){
            pr_err("gf:check 9p ver fail \n");
            retry_cnt = 0xFF;
            break;
        }


        mdelay(10);
        gf_spi_write_byte(gf_dev, 0x5081, 0x00);

        gf_spi_write_byte(gf_dev, 0x4180, 0x0C);
        gf_spi_read_byte(gf_dev, 0x4180, &value);
        if (value == 0x0C)   
        {
            gf_debug(DEFAULT_DEBUG, "########hold SS51 and DSP successfully!\n");
            break;
        }
    }

    if(retry_cnt == 0xFF) {
        gf_debug(DEFAULT_DEBUG, "Faile to hold SS51 and DSP.\n");
        return 0;
    } else {
        gf_debug(DEFAULT_DEBUG, "Hold retry_cnt=%d\n",retry_cnt);
        gf_spi_write_byte(gf_dev, 0x4010, 0);
        return 1;
    }
}

static void gf_esd_work(struct work_struct *work)
{
    unsigned char value[4];
#if FW_UPDATE
    unsigned char* p_fw = GF_FW;
#endif
    struct gf_dev *gf_dev;
    int ret = 0;
    u8 mode = 0xFF;
    FUNC_ENTRY();
    if(work == NULL)
    {
        gf_debug(DEFAULT_DEBUG, "[info] %s wrong work\n",__func__);
        return;
    }
    gf_dev = container_of((struct delayed_work *)work, struct gf_dev, spi_work);   

    mutex_lock(&gf_dev->fb_lock);

    if ((gf_dev->isPowerOn == 0) || (gf_dev->mode == GF_FF_MODE)) {
        goto exit;
    }

    if(g_mode_switch_flag == 1) {
        g_mode_switch_flag = 0;
        goto exit;
    }

   

    mutex_lock(&gf_dev->buf_lock);
    //ret = power_supply_is_system_supplied();
    //pr_info("BEN: power_supply ret = %d\n", ret);

    gf_dev->spi->max_speed_hz= 1000*1000;//SPI_SPEED_MIN;
	gf_spi_set_mode(gf_dev->spi, SPEED_1MHZ, 0);
    mutex_unlock(&gf_dev->buf_lock);

    gf_spi_read_byte(gf_dev, 0x8040, &value[0]);
    gf_spi_read_byte(gf_dev, 0x8000, &value[1]);
    gf_spi_read_byte(gf_dev, 0x8043, &value[2]); //&& value[1] == 0x47 && value[2] == 0x56
    //pr_info("timer_work: value[0] = 0x%x, 1:0x%x, 2:0x%x\n", value[0], value[1], value[2]);
    if(value[0] == 0xC6 && value[1] == 0x47){
        gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
        mdelay(1);
    }else{
        gf_spi_read_byte(gf_dev, 0x8040, &value[0]);
        gf_spi_read_byte(gf_dev, 0x8000, &value[1]);
        gf_spi_read_byte(gf_dev, 0x8043, &value[2]); //&& value[1] == 0x47 && value[2] == 0x56
        if(value[0] == 0xC6 && value[1] == 0x47){
            gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
            mdelay(1);
        }else{
            gf_debug(DEFAULT_DEBUG, "##Jason hardware works abnormal,do reset! 0x8040=0x%x 0x8000=0x%x 0x8046=0x%x\n"
                    ,value[0],value[1],value[2]);
            disable_irq(gf_dev->spi->irq);
            gf_hw_reset(gf_dev);

            gf_spi_read_byte(gf_dev, 0x8000, &value[0]);
            pr_info("[info] %s read 0x8000 finish value = 0x%x\n", __func__,value[0]);
#if FW_UPDATE
            if(value[0] != 0x47) {
                gf_spi_read_byte(gf_dev, 0x8000, &value[0]);
                if(value[0] != 0x47) {
                    /***********************************firmware update*********************************/
                    gf_debug(DEFAULT_DEBUG, "[info] %s firmware update start\n", __func__);
                    if(gf_fw_update_init(gf_dev)) {
                        gf_fw_update(gf_dev, p_fw, FW_LENGTH);
                        gf_hw_reset(gf_dev);
                    }
                }
            }
            /***************************************update config********************************/
            //pr_info("[info] %s write 0xaa \n", __func__);
            ret = gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
            if(!ret)
                gf_debug(DEFAULT_DEBUG, "[info] %s write 0x8040 fail\n", __func__);

            if(!hw_config(gf_dev))
                gf_debug(DEFAULT_DEBUG, "[info] %s write config fail\n", __func__);
#endif
            enable_irq(gf_dev->spi->irq);
            gf_dev->irq_enabled = 1;
        }
    }
    /*if mode was changed by reset, we should set the mode  back to the primary mode*/
    gf_spi_read_byte(gf_dev, GF_MODE_STATUS,&mode);
    if(mode != gf_dev->mode) {
        gf_debug(DEFAULT_DEBUG, "[info] %s set mode back\n", __func__);
        gf_spi_write_byte(gf_dev, GF_MODE_STATUS, gf_dev->mode);
        gf_spi_read_byte(gf_dev, GF_MODE_STATUS, &mode);
        gf_debug(DEFAULT_DEBUG, "[info] %s mode444 = %d\n", __func__, mode);
    }

exit:
    mutex_unlock(&gf_dev->fb_lock);
	queue_delayed_work(gf_dev->spi_wq, &gf_dev->spi_work, gf_dev->clk_tick_cnt);
    FUNC_EXIT();
}

static void gf_esd_switch(struct gf_dev *gf_dev, s32 on)
{
	//	spin_lock_irq(&gf_dev->spi_lock);	
	if (1 == on) {
		//switch on esd		
		if (0 == gf_dev->esd_running){			
			queue_delayed_work(gf_dev->spi_wq, &gf_dev->spi_work, gf_dev->clk_tick_cnt);			
			gf_dev->esd_running = 1;			
			gf_debug(DEFAULT_DEBUG,"start esd work");					
		}	
	} else {
		//switch off esd		
		if (1 == gf_dev->esd_running) {			
			cancel_delayed_work_sync(&gf_dev->spi_work);			
			gf_dev->esd_running = 0;			
			gf_debug(DEFAULT_DEBUG,"stop esd work");		
		}	
	}
	//	spin_unlock_irq(&gf_dev->spi_lock);
}

static irqreturn_t gf_irq(int irq, void* handle)
{    
	irq_flag = 1;    
	gf_debug(DEFAULT_DEBUG, "gf: %s ", __func__);    
	wake_up_interruptible(&waiter);	
	return IRQ_HANDLED;
}

static int gf_event_handler(void *para)
{
    struct gf_dev *gf_dev = (struct gf_dev *)para;
    u8 mode = 0x80;
    u8	status = 0, lbstatus = 0;

#if (NAVIGATION == 1)
    u8  ngv;
    u8  force_val;
#endif
	do {
		mode = 0x80;
		status = 0;
		lbstatus = 0;
#if (NAVIGATION == 1)
		ngv = 0;
		force_val = 0;
#endif
		gf_debug(DEFAULT_DEBUG, "waiter event");		
		wait_event_interruptible(waiter, irq_flag != 0);
		mutex_lock(&gf_dev->fb_lock);
		irq_flag = 0;
    	gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &status);
    	gf_spi_read_byte(gf_dev, GF_LBSTATUS_ADDR, &lbstatus);
    	gf_spi_read_byte(gf_dev, GF_MODE_STATUS, &mode);

    	if (lbstatus & (1<<7) ) {
        	//senser reset
        	gf_spi_write_byte(gf_dev, GF_LBSTATUS_ADDR, lbstatus & 0x7F);
        	gf_spi_write_byte(gf_dev, GF_MODE_STATUS, gf_dev->mode);
        	gf_debug(DEFAULT_DEBUG, "Sensor rest, set mode %d\n", gf_dev->mode);
			mutex_unlock(&gf_dev->fb_lock);
	    	continue;
    	}
 
   
    	if(!(status & GF_BUF_STA_MASK)) {
        	gf_debug(DEFAULT_DEBUG, "Invalid IRQ status = 0x%x, lbstatus = 0x%x, mode = %d\n", status, lbstatus,mode);
			mutex_unlock(&gf_dev->fb_lock);
			continue;
    	}
 
    	gf_debug(DEFAULT_DEBUG, "IRQ status = 0x%x, lbstatus = 0x%x, mode = %d\n", status, lbstatus, mode);

    	switch(mode)
    	{
       		case GF_FF_MODE:
				#if 0
            	if((status & GF_HOME_KEY_MASK) && (status & GF_HOME_KEY_STA)){
                	gf_debug(DEFAULT_DEBUG, "gf: wake device.\n");
                	//gf_spi_write_byte(gf_dev, GF_MODE_STATUS, 0x00);
                	input_report_key(gf_dev->input, GF_KEY_FF, 1);
                	input_sync(gf_dev->input);
                	input_report_key(gf_dev->input, GF_KEY_FF, 0);
                	input_sync(gf_dev->input);
            	} else {
               	 	break;
            	}
				#endif

        	case GF_IMAGE_MODE:
				if ((status & 0xf0) ==0xb0){				
					finger_updown = 1;			
					wake_lock_timeout(&gf_dev->wl, 2*HZ);
				} else if ((status & 0xf0) ==0xa0){
					finger_updown = 0;
				}
#ifdef GF_FASYNC
            	if(gf_dev->async) {
                	//pr_info("async \n");
                	kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
            	}
#endif
            	break;
        	case GF_KEY_MODE:
#if (NAVIGATION == 1)
            	gf_spi_read_byte(gf_dev, GF_LBSTATUS_ADDR, &lbstatus); 
            	gf_debug(DEFAULT_DEBUG, "gf:Key mode: status = 0x%x lbstatus = 0x%x\n", status, lbstatus);

            	if(lbstatus & GF_NGKEY_MASK) {
                	if(lbstatus & GF_NVG_KEY_MASK) { //Navigator is enabled
                    	ngv = (lbstatus & GF_NVG_KEY_STA) >> 4;
                    	if (NGV_VALUE_UP == ngv) {
                        	gf_debug(DEFAULT_DEBUG, "UP");
                        	input_report_key(gf_dev->input, GF_UP_KEY, 1);
                        	input_sync(gf_dev->input);
                        	input_report_key(gf_dev->input, GF_UP_KEY, 0);
                        	input_sync(gf_dev->input);
                    	} else if (NGV_VALUE_DOWN == ngv) {
                        	gf_debug(DEFAULT_DEBUG, "DOWN");
                        	input_report_key(gf_dev->input, GF_DOWN_KEY, 1);
                        	input_sync(gf_dev->input);
                        	input_report_key(gf_dev->input, GF_DOWN_KEY, 0);
                        	input_sync(gf_dev->input);
                    	} else if (NGV_VALUE_LEFT == ngv) {
                        	/*
                        	pr_info("KEY_BACK");
                         	gf_send_key(KEY_BACK, 1);   
                           	gf_send_key(KEY_BACK, 0);   
                         	*/
                        	gf_debug(DEFAULT_DEBUG, "LEFT");
                        	input_report_key(gf_dev->input, GF_KEY_MENU, 1);
                        	input_sync(gf_dev->input);
                        	input_report_key(gf_dev->input, GF_KEY_MENU, 0);
                        	input_sync(gf_dev->input);
                    	} else if (NGV_VALUE_RIGHT == ngv) {
                        	gf_debug(DEFAULT_DEBUG, "RIGHT");
                        	/*gf_send_key(KEY_F17, 1);  
                          	gf_send_key(KEY_F17, 0);    
                         	*/
                        	input_report_key(gf_dev->input, GF_KEY_BACK, 1);
                        	input_sync(gf_dev->input);
                        	input_report_key(gf_dev->input, GF_KEY_BACK, 0);
                        	input_sync(gf_dev->input);
                    	}
                	} 

                	if (lbstatus&GF_FCE_KEY_MASK) { // force value is enabled.
                    	gf_spi_read_byte(gf_dev, GF_FORCE_VALUE_ADDR, &force_val);
                    	gf_debug(DEFAULT_DEBUG, "FORCE VALUE. force_val : %d\n", force_val);
                    	input_report_key(gf_dev->input, GF_KEY_FORCE, force_val);
                    	input_sync(gf_dev->input);
                	}
            	} else if (status & GF_KEY_MASK) {
                	gf_debug(DEFAULT_DEBUG, "input home key \n");
                	if (status & GF_HOME_KEY_MASK) {
                    	input_report_key(gf_dev->input, GF_KEY_HOME, (status & GF_HOME_KEY_STA)>>4);
                    	input_sync(gf_dev->input);
                	}
            	}
            	//gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, (status & 0x7F));
#else
	    		gf_debug(DEFAULT_DEBUG, "gf:Key mode: status = 0x%x\n", status);
	    		if  ((status & GF_KEY_MASK) && (status & GF_BUF_STA_MASK)) {
					if (status & GF_HOME_KEY_MASK) {
		    			input_report_key(gf_dev->input, GF_KEY_HOME, (status & GF_HOME_KEY_STA)>>4);
		    			input_sync(gf_dev->input);
					}
					else if (status & GF_MENU_KEY_MASK){
		    			input_report_key(gf_dev->input, GF_KEY_MENU, (status & GF_MENU_KEY_STA)>>2);
		    			input_sync(gf_dev->input);
					}else if (status & GF_BACK_KEY_MASK){
		    			input_report_key(gf_dev->input, GF_KEY_BACK, (status & GF_BACK_KEY_STA));
		    			input_sync(gf_dev->input);
					}
	    		}
#endif
	    		if(gf_dev->async) {
    				kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
	    		}   
            	break;
        	case GF_SLEEP_MODE:
            	pr_warn("gf:Should not happen in sleep mode.\n");
            	break;
        	case GF_DEBUG_MODE:
#ifdef GF_FASYNC
            	if(gf_dev->async) {
               		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
            	}
#endif
            	break;
        	case GF_NAV_MODE:
            	gf_debug(DEFAULT_DEBUG, "Macky:IRQ status = 0x%x, lbstatus = 0x%x, mode = %d\n", status, lbstatus, mode);
            	sendnlmsg("nagv");
            	break;
        	default:
            	pr_warn("gf:Unknown mode. mode = 0x%x\n", mode);
            	break;
    	}
		mutex_unlock(&gf_dev->fb_lock);
	}while(!kthread_should_stop());

    return 0;
}


static int gf_open(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int			status = -ENXIO;

    FUNC_ENTRY();
    mutex_lock(&device_list_lock);

    list_for_each_entry(gf_dev, &device_list, device_entry) {
        if(gf_dev->devt == inode->i_rdev) {
            gf_debug(DEFAULT_DEBUG, "Found\n");
            status = 0;
            break;
        }
    }

    if(status == 0){
        mutex_lock(&gf_dev->buf_lock);
        if( gf_dev->buffer == NULL) {
            gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
            if(gf_dev->buffer == NULL) {
                dev_dbg(&gf_dev->spi->dev, "open/ENOMEM\n");
                status = -ENOMEM;
            }
        }
        mutex_unlock(&gf_dev->buf_lock);

        if(status == 0) {
            gf_dev->users++;
            filp->private_data = gf_dev;
            nonseekable_open(inode, filp);
            gf_debug(DEFAULT_DEBUG, "Succeed to open device. irq = %d\n", gf_dev->spi->irq);
            enable_irq(gf_dev->spi->irq);
            gf_dev->irq_enabled = 1;
        }
    } else {
        pr_err("gf:No device for minor %d\n", iminor(inode));
    }
    //gf_power_on(gf_dev);
    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
    struct gf_dev *gf_dev = filp->private_data;
    int ret;

    FUNC_ENTRY();
    ret = fasync_helper(fd, filp, mode, &gf_dev->async);
    FUNC_EXIT();
    return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int    status = 0;
    FUNC_ENTRY();
    mutex_lock(&device_list_lock);
    gf_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close??*/
    gf_dev->users --;
    if(!gf_dev->users) {

        gf_debug(DEFAULT_DEBUG, "disble_irq. irq = %d\n", gf_dev->spi->irq);
        disable_irq(gf_dev->spi->irq);
       // gf_power_off(gf_dev);
    }
    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}

static const struct file_operations gf_fops = {
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =	gf_write,
    .read =		gf_read,
    .unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT    
    .compat_ioctl = gf_compat_ioctl,
#endif        
    .open =		gf_open,
    .release =	gf_release,
    .poll   = gf_poll,
#ifdef GF_FASYNC
    .fasync = gf_fasync,
#endif
};



/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *gf_spi_class;

/*-------------------------------------------------------------------------*/

static int gf_probe(struct spi_device *spi)
{
    struct gf_dev	*gf_dev;
    int			status;
    int ret;
    unsigned long		minor;
    int err = 0;
    unsigned char version[16] = {0};
	struct device_node *node = NULL;	
	u32 ints[2] = { 0, 0 };
    FUNC_ENTRY();

    /* Allocate driver data */
    gf_dev = kzalloc(sizeof(*gf_dev), GFP_KERNEL);
    if (!gf_dev){
        pr_warn("gf:Failed to alloc memory for gf device.\n");
        FUNC_EXIT();
        return -ENOMEM;
    }

	gf_dev->pc = pc;
	gf_dev->pwr_on = ps_pwr_on;
	gf_dev->pwr_off = ps_pwr_off;
	gf_dev->rst_low = ps_rst_low;
	gf_dev->rst_high = ps_rst_high;
	gf_dev->irq_init = ps_irq_init;
	gf_dev->irq_in = ps_irq_in;
	gf_dev->irq_dis = ps_irq_dis;
	gf_dev->irq_en = ps_irq_en;
	gf_dev->miso_spi = ps_miso_spi;
	gf_dev->miso_pullup = ps_miso_pullup;
#ifdef CONFIG_FINGERPRINT_ID_GPIO
	gf_dev->id_up = ps_id_up;
	gf_dev->id_down = ps_id_down;
	gf_dev->id_gpio = id_gpio;
	gpio_request(gf_dev->id_gpio, "fp_id");
#endif
	
    gf_power_on(gf_dev);
    g_mode_switch_flag = 0; 
    /* Initialize the driver data */
    gf_dev->spi = spi;
    gf_dev->irq_enabled = 0;
    spin_lock_init(&gf_dev->spi_lock);
    mutex_init(&gf_dev->buf_lock);
    mutex_init(&gf_dev->fb_lock);
    INIT_LIST_HEAD(&gf_dev->device_entry);
	wake_lock_init(&gf_dev->wl, WAKE_LOCK_SUSPEND, "gf_fp_wl");

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
        struct device *dev;

        status = sysfs_create_group(&spi->dev.kobj,&gf_debug_attr_group);
        if(status){
			mutex_unlock(&device_list_lock);
            pr_err("gf:Failed to create sysfs file.\n");
            goto err;
        }

        gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
        dev = device_create(gf_spi_class, &spi->dev, gf_dev->devt,
                gf_dev, DEV_NAME);
        status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    } else {
        dev_dbg(&spi->dev, "gf:no minor number available!\n");
        status = -ENODEV;
    }
    if (status == 0) {
        set_bit(minor, minors);
        list_add(&gf_dev->device_entry, &device_list);
    }
    mutex_unlock(&device_list_lock);

    if (status == 0){
        gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
        if(gf_dev->buffer == NULL) {
            kfree(gf_dev);
            status = -ENOMEM;
            goto err_alloc_buf;
        }
        spi_set_drvdata(spi, gf_dev);


        gf_dev->fb_black = 0;
        //gf_dev->notifier = gf_noti_block;
        //fb_register_client(&gf_dev->notifier);


        /*setup gf configurations.*/
        gf_debug(DEFAULT_DEBUG, "Setting gf device configuration.\n");
        /*SPI parameters.*/
        gf_spi_setup(gf_dev, 1000*1000);
		gf_spi_set_mode(gf_dev->spi, SPEED_1MHZ, 0);
        //gf_irq_cfg();
        //gf_debug(DEFAULT_DEBUG, "interrupt NO. = %d\n", gf_dev->spi->irq);

#if FW_UPDATE
        if(isUpdate(gf_dev)) {
            unsigned char* fw = GF_FW;
            /*Do upgrade action.*/     
            if(gf_fw_update_init(gf_dev)) {
                gf_fw_update(gf_dev, fw, FW_LENGTH);
                gf_hw_reset(gf_dev);
                //mdelay(100);
            }
        }
        /*write config*/
        if(!hw_config(gf_dev))
            gf_debug(DEFAULT_DEBUG, "[info] %s write config fail\n", __func__);
#endif
		ret = gf_spi_read_bytes(gf_dev,0x8000,16,gf_dev->buffer);
        memcpy(version, gf_dev->buffer + GF_RDATA_OFFSET, 16);
        for(ret = 0; ret <16; ret++)
            gf_debug(DEFAULT_DEBUG, "version[%d] = %x \n", ret,version[ret]);
		chip_version_config = version[4];
		gf_debug(DEFAULT_DEBUG,"goodix chip_version_config=%x ",chip_version_config);
		if (chip_version_config != 0x38)
			goto err_check_id;

        /*register device within input system.*/
        gf_dev->input = input_allocate_device();
        if(gf_dev->input == NULL) {
            pr_err("gf:Failed to allocate input device.\n");
            status = -ENOMEM;
            goto err_input;
        }
        gf_reg_key_kernel(gf_dev);

		node = of_find_matching_node(node, fp_of_match);	
		if(node) {		
			of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));			
			gpio_request(ints[0], "gf_irq");		
			gpio_set_debounce(ints[0], ints[1]);		
			gf_dev->spi->irq =  irq_of_parse_and_map(node, 0);		
			gf_debug(DEFAULT_DEBUG,"irq number %d", gf_dev->spi->irq);				
			if (!gf_dev->spi->irq) {			
					pr_err("gf:Failed to get irq number.");			
					goto err_get_irq;		
				}		
			gf_irq_thread = kthread_run(gf_event_handler, (void *)gf_dev, "gf-thread");		
			if (IS_ERR(gf_irq_thread))		
			{			
				pr_err("gf:Failed to create kernel thread: %ld", PTR_ERR(gf_irq_thread));	
				goto err_get_irq;
			}     	   
#if 0
        err = request_threaded_irq(spi->irq, NULL, gf_irq,
                IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                dev_name(&spi->dev), gf_dev);
#else
		
        err = request_irq(gf_dev->spi->irq, gf_irq,
                IRQF_TRIGGER_RISING,//IRQ_TYPE_LEVEL_HIGH,
                dev_name(&gf_dev->spi->dev), gf_dev);
#endif
		}
        if(!err) {
            disable_irq(gf_dev->spi->irq);
        }

#if ESD_PROTECT
		gf_dev->clk_tick_cnt = 2*HZ;   //2*HZ is 2 seconds	
		INIT_DELAYED_WORK(&gf_dev->spi_work, gf_esd_work);	
		gf_dev->spi_wq = create_workqueue("gf_esd_check");	
		gf_esd_switch(gf_dev, 1);
#endif // ESD_PROTECT

        gf_debug(DEFAULT_DEBUG, "GF installed.\n");
    }
    else
        kfree(gf_dev);

	return 0;
err_get_irq:
	gpio_free(ints[0]);
	input_unregister_device(gf_dev->input);
err_input:
err_check_id:
	kfree(gf_dev->buffer);
err_alloc_buf:
    list_del(&gf_dev->device_entry);
    clear_bit(MINOR(gf_dev->devt), minors);
	device_destroy(gf_spi_class, gf_dev->devt);
	sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);
err:
	wake_lock_destroy(&gf_dev->wl);
    gf_power_off(gf_dev);
#ifdef CONFIG_FINGERPRINT_ID_GPIO
	gpio_free(gf_dev->id_gpio);
#endif
	kfree(gf_dev);
    FUNC_EXIT();
	gf_debug(DEFAULT_DEBUG,"probe failed");
    return status;
}

static int gf_remove(struct spi_device *spi)
{
    struct gf_dev	*gf_dev = spi_get_drvdata(spi);
    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
    if(gf_dev->spi->irq) {
        free_irq(gf_dev->spi->irq, gf_dev);
    }

#if ESD_PROTECT
    destroy_workqueue(gf_dev->spi_wq);
#endif

    spin_lock_irq(&gf_dev->spi_lock);
    gf_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&gf_dev->spi_lock);
	wake_lock_destroy(&gf_dev->wl);
    /*
       if(gf_dev->spi_wq != NULL) {
       flush_workqueue(gf_dev->spi_wq);
       destroy_workqueue(gf_dev->spi_wq);
       }
     */
    /* prevent new opens */
    mutex_lock(&device_list_lock);
    sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);
    list_del(&gf_dev->device_entry);
    device_destroy(gf_spi_class, gf_dev->devt);
    clear_bit(MINOR(gf_dev->devt), minors);
    if (gf_dev->users == 0) {
        if(gf_dev->input != NULL)
            input_unregister_device(gf_dev->input);

        if(gf_dev->buffer != NULL)
            kfree(gf_dev->buffer);
        kfree(gf_dev);
    }
    mutex_unlock(&device_list_lock);

    FUNC_EXIT();
    return 0;
}

static int gf_suspend_test(struct device *dev)
{
    printk(KERN_ERR"gf_suspend_test.\n");
    g_debug |= SUSPEND_DEBUG;
    return 0;
}

static int gf_resume_test(struct device *dev)
{
    printk(KERN_ERR"gf_resume_test.\n");
    g_debug &= ~SUSPEND_DEBUG;
    return 0;
}
static const struct dev_pm_ops gf_pm = {
    .suspend = gf_suspend_test,
    .resume = gf_resume_test
};

static struct spi_driver gf_spi_driver = {
    .driver = {
        .name =		SPI_DEV_NAME,
        .owner =	THIS_MODULE,
        .pm = &gf_pm,
    },
    .probe =	gf_probe,
    .remove =	gf_remove,
    //.suspend = gf_suspend_test,
    //.resume = gf_resume_test,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};
static struct spi_board_info spi_board_devs[] __initdata = {	
	[0] = {	    
		.modalias=SPI_DEV_NAME,		
			.bus_num = 0,		
			.chip_select=0,		
			.mode = SPI_MODE_0,		
			.controller_data = &spi_conf_mt65xx,	
	},
};

static int fp_probe(struct platform_device *pdev)
{		
	struct pinctrl_state *spi_state;
	
	pc = devm_pinctrl_get(&pdev->dev);    
	if (IS_ERR(pc)) {        
		pr_err("fp:get pinctrl config failed");        
		return PTR_ERR(pc);    
		}	
	ps_pwr_on = pinctrl_lookup_state(pc, "pwr_on");	
	ps_pwr_off = pinctrl_lookup_state(pc, "pwr_off");	
	ps_rst_low = pinctrl_lookup_state(pc, "rst_low");	
	ps_rst_high = pinctrl_lookup_state(pc, "rst_high");	
	ps_irq_init = pinctrl_lookup_state(pc, "irq_init");	
	ps_irq_in = pinctrl_lookup_state(pc, "irq_in");	
	ps_irq_dis = pinctrl_lookup_state(pc, "irq_dis");	
	ps_irq_en = pinctrl_lookup_state(pc, "irq_en");	
#ifdef CONFIG_FINGERPRINT_ID_GPIO
	ps_id_up = pinctrl_lookup_state(pc, "id_up");	
	ps_id_down = pinctrl_lookup_state(pc, "id_down");	
#endif
	if (IS_ERR(ps_pwr_on) || IS_ERR(ps_pwr_off)		 
		|| IS_ERR(ps_rst_low) || IS_ERR(ps_rst_high) || IS_ERR(ps_irq_init)		 
		|| IS_ERR(ps_irq_in) || IS_ERR(ps_irq_dis) || IS_ERR(ps_irq_en)		 
#ifdef CONFIG_FINGERPRINT_ID_GPIO
		|| IS_ERR(ps_id_up) || IS_ERR(ps_id_down) 
#endif
	   ){		
		pr_err("fp:lookup pinctrl state failed");		
		return -1;	
	}
	spi_state = pinctrl_lookup_state(pc, "spi1_cs_set");
	pinctrl_select_state(pc, spi_state);
	spi_state = pinctrl_lookup_state(pc, "spi1_clk_set");
	pinctrl_select_state(pc, spi_state);
	ps_miso_spi = pinctrl_lookup_state(pc, "spi1_miso_set");
	pinctrl_select_state(pc, ps_miso_spi);
	spi_state = pinctrl_lookup_state(pc, "spi1_mosi_set");
	pinctrl_select_state(pc, spi_state);
	ps_miso_pullup = pinctrl_lookup_state(pc, "spi1_miso_pullup");
	
#ifdef CONFIG_FINGERPRINT_ID_GPIO
	id_gpio = of_get_gpio(pdev->dev.of_node, 0);	
#endif
	pr_info("fp:platform_driver probe success");	
	return 0;
}

static int fp_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver fp_driver = {     
	.remove = fp_remove,   
	.shutdown = NULL,    
	.probe = fp_probe,    
	.driver = {             
		.name = "fp_config",            
		.owner = THIS_MODULE,            
		.of_match_table = fp_of_match,    
	},  
};
/*-------------------------------------------------------------------------*/

static int __init gf_init(void)
{
    int status;
    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
    if (status < 0){
        pr_warn("gf:Failed to register char device!\n");
        FUNC_EXIT();
        return status;
    }

    gf_spi_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gf_spi_class)) {
        unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
        pr_warn("gf:Failed to create class.\n");
        FUNC_EXIT();
        return PTR_ERR(gf_spi_class);
    }
	status = platform_driver_register(&fp_driver);
	if (status < 0){
		class_destroy(gf_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
		pr_err("gf:Failed to register SPI platform driver.");
		return status;
	}

	spi_register_board_info(spi_board_devs,ARRAY_SIZE(spi_board_devs));
    status = spi_register_driver(&gf_spi_driver);
    if (status < 0) {
        class_destroy(gf_spi_class);
        unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
        pr_warn("gf:Failed to register SPI driver.\n");
    }

    /*init netlink for kernel and user space*/
    netlink_init();

    return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
    spi_unregister_driver(&gf_spi_driver);
    class_destroy(gf_spi_class);
    unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
    netlink_exit();
	platform_driver_unregister(&fp_driver);
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");


