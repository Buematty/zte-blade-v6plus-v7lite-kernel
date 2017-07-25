#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/gpio.h>
#include <linux/pinctrl/devinfo.h>
#include <linux/wakelock.h>
/********************GF Mapping**********************/
#define GF_BASE             (0x8000)
#define GF_OFFSET(x)        (GF_BASE + x)

#define GF_VERSION	        GF_OFFSET(0)
#define GF_CONFIG_DATA      GF_OFFSET(0x40)
#define GF_CFG_ADDR	        GF_OFFSET(0x47)
#define GF_PAUSE_ADDR       GF_OFFSET(0x44)
#define GF_MODE_STATUS      GF_OFFSET(0x043)
//#define GF_MIXER_DATA     GF_OFFSET(0x140)
#define GF_BUFFER_STATUS	GF_OFFSET(0x140)
#define GF_KEY_DATA         GF_OFFSET(0x142)
#define GF_NOISE_DATA       GF_OFFSET(0x144)
#define GF_LONG_PRESS_STDP  GF_OFFSET(0x146)
#define GF_BUFFER_DATA      GF_OFFSET(0x141)
#define GF_FORCE_VALUE_ADDR GF_OFFSET(0x146)
#define GF_LBSTATUS_ADDR    GF_OFFSET(0x141)
#define GF_NAV_FRAMENUM_ADDR    GF_OFFSET(0x146)
#define GF_NAV_BUF0_ADDR   GF_OFFSET(0x148)
#define GF_NAV_BUF1_ADDR       0x9B48



#define GF_BUF_STA_MASK     (0x1<<7)
#define	GF_BUF_STA_READY	(0x1<<7)
#define	GF_BUF_STA_BUSY     (0x0<<7)

#define	GF_IMAGE_MASK       (0x1<<6)
#define	GF_IMAGE_ENABLE     (0x1)
#define	GF_IMAGE_DISABLE	(0x0)

#define GF_LONGPRESS_MASK   (0x1<<2)
#define GF_FCE_KEY_MASK (0x1<<3)
#define GF_NVG_KEY_MASK (0x1<<6)
#define GF_NVG_KEY_STA  (0x3<<4)
#define GF_NGKEY_MASK   (GF_NVG_KEY_MASK | GF_FCE_KEY_MASK | GF_LONGPRESS_MASK)


#define	GF_KEY_MASK	        (GF_HOME_KEY_MASK | \
                                 GF_MENU_KEY_MASK | \
                                 GF_BACK_KEY_MASK )

//home key
#define	GF_HOME_KEY_MASK	(0x1<<5)
#define	GF_HOME_KEY_ENABL   (0x1)
#define	GF_HOME_KEY_DISABLE (0x0)

#define	GF_HOME_KEY_STA     (0x1<<4)
//menu key
#define	GF_MENU_KEY_MASK    (0x1<<3)
#define	GF_MENU_KEY_ENABLE	(0x1)
#define	GF_MENU_KEY_DISABLE	(0x0)

#define	GF_MENU_KEY_STA	(0x1<<2)
//back key
#define	GF_BACK_KEY_MASK    (0x1<<1)
#define	GF_BACK_KEY_ENABLE  (0x1)
#define	GF_BACK_KEY_DISABLE (0x0)

#define	GF_BACK_KEY_STA     (0x1<<0)


#define NGV_VALUE_UP        0x00 
#define NGV_VALUE_DOWN  0x01
#define NGV_VALUE_LEFT  0x02
#define NGV_VALUE_RIGHT     0x03



#define	GF_IMAGE_MODE       (0x00)
#define	GF_KEY_MODE	    (0x01)
#define GF_SLEEP_MODE       (0x02)
#define GF_FF_MODE	    (0x03)
#define GF_NAV_MODE         (0x10)
#define GF_PRENAV_MODE      (0x11)
#define GF_DEBUG_MODE       (0x56)


/*low 8-bit status*/
#define GF_RST_MASK     (0x1<<7)
#define GF_NAV_MASK     (0x1<<6)
#define GF_NAV_BUF_MASK     (0x1<<5)
#define GF_LONG_PRESS_MASK     (0x1<<2)
#define GF_LONG_PRESS_STA     (0x1<<1)


/**********************GF ops****************************/
#define GF_W                0xF0
#define GF_R                0xF1
#define GF_WDATA_OFFSET         (0x3)
#define GF_RDATA_OFFSET         (0x5)
#define GF_CFG_LEN                  (249)   /*config data length*/
#define MTK_SPI_ALIGN_MASK_NUM 10
#define  MTK_SPI_ALIGN_MASK  ((0x1 << MTK_SPI_ALIGN_MASK_NUM) - 1)

/**********************************************************/

#define GF_FASYNC 		1//If support fasync mechanism.
//#undef GF_FASYNC

/*************************************************************/
struct gf_dev {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct input_dev        *input;

	struct workqueue_struct *spi_wq;
	struct delayed_work     spi_work;
	int clk_tick_cnt;
	int esd_running;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned		users;
	u8			*buffer;
	u8  		buf_status;
	u8           mode;
	u8           isPowerOn;
	struct timer_list   gf_timer;
#ifdef GF_FASYNC
	struct  fasync_struct *async;
#endif
    struct notifier_block notifier;
    char fb_black;
    struct mutex fb_lock;
    int irq_enabled;
	
	int irq_pin;
	int rst_pin;
	int miso_pin;
	struct pinctrl *pc;	
	struct pinctrl_state *pwr_on;	
	struct pinctrl_state *pwr_off;	
	struct pinctrl_state *rst_low;	
	struct pinctrl_state *rst_high;	
	struct pinctrl_state *irq_init;	
	struct pinctrl_state *irq_in;	
	struct pinctrl_state *irq_dis;	
	struct pinctrl_state *irq_en;	
	struct pinctrl_state *id_up;	
	struct pinctrl_state *id_down;
	struct pinctrl_state *miso_nopull;
	struct pinctrl_state *miso_up;
	struct pinctrl_state *miso_spi;
	struct pinctrl_state *miso_pullup;
	unsigned long id_gpio;

	struct wake_lock wl;
};

struct gf_key {
        unsigned int key;
        int value;
};


/**********************IO Magic**********************/
#define  GF_IOC_MAGIC    'g'  //define magic number
struct gf_ioc_transfer {
	u8	cmd;
	u8 reserve;
	u16	addr;
	u32 len;
	u8 *buf;
};
struct gf_nagv {
	u8 *buf;
	u32 len;
        u8 *frame_num;
};
//define commands
/*read/write GF registers*/
#define  GF_IOC_CMD	_IOWR(GF_IOC_MAGIC, 1, struct gf_ioc_transfer)
#define  GF_IOC_REINIT	_IO(GF_IOC_MAGIC, 0)
#define  GF_IOC_SETSPEED	_IOW(GF_IOC_MAGIC, 2, u32)
#define  GF_IOC_POWER_OFF   _IO(GF_IOC_MAGIC, 3)
#define  GF_IOC_POWER_ON    _IO(GF_IOC_MAGIC, 4)
#define  GF_IOC_DISABLE_IRQ     _IO(GF_IOC_MAGIC, 5)
#define  GF_IOC_ENABLE_IRQ      _IO(GF_IOC_MAGIC, 6)
#define  GF_IOC_SENDKEY  _IOW(GF_IOC_MAGIC, 7, struct gf_key)
#define  GF_IOC_GET_NAGV_BASE  _IOW(GF_IOC_MAGIC, 8, struct gf_nagv)
#define  GF_IOC_GET_NAGV_DATA  _IOW(GF_IOC_MAGIC, 9, struct gf_nagv)


#define  GF_IOC_MAXNR    10

/*******************Refering to hardware platform*****************************/
//#define 	GF_RST_PIN   	EXYNOS4_GPX1(5)
//#define 	GF_IRQ_PIN   	EXYNOS4_GPX1(4)
//#define 	GF_IRQ_NUM   	gpio_to_irq(GF_IRQ_PIN)
//#define		GF_MISO_PIN	EXYNOS4_GPB(2)

/*Confure the IRQ pin for GF irq if necessary*/
inline static void gf_irq_cfg(struct gf_dev *gf_dev)
{
	/*Config IRQ pin, referring to platform.*/
	gpio_request_one(gf_dev->irq_pin, GPIOF_IN, "gf_irq");
	pinctrl_select_state(gf_dev->pc, gf_dev->irq_in);
	gpio_free(gf_dev->irq_pin);
}


inline static void gf_miso_pullup(struct gf_dev *gf_dev)
{
	/*Config MISO pin, referring to platform.*/
	pinctrl_select_state(gf_dev->pc, gf_dev->miso_pullup);
}



inline static void gf_miso_backnal(struct gf_dev *gf_dev)
{
	/*Config IRQ pin, referring to platform.*/
	pinctrl_select_state(gf_dev->pc, gf_dev->miso_spi);
}


/********************************************************************
*CPU output low level in RST pin to reset GF. This is the MUST action for GF.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
inline static void gf_hw_reset(struct gf_dev *gf_dev)
{		/*rst pin referring to samsung KIT.*/
	pinctrl_select_state(gf_dev->pc, gf_dev->rst_high);
	mdelay(1);
	pinctrl_select_state(gf_dev->pc, gf_dev->rst_low);			 	
	msleep(6);  //delay for power to reset  typical:10ms max:50ms	
	pinctrl_select_state(gf_dev->pc, gf_dev->rst_high);
	mdelay(100);
}

int gf_spi_read_bytes(struct gf_dev *gf_dev,
                                u16 addr, u32 data_len, u8 *rx_buf);

int gf_spi_write_bytes(struct gf_dev *gf_dev,
                                u16 addr, u32 data_len, u8 *tx_buf);
int gf_fw_update(struct gf_dev* gf_dev, unsigned char *buf, unsigned short len);
int netlink_init(void);
void netlink_exit(void);
void sendnlmsg(char *message);
#endif
