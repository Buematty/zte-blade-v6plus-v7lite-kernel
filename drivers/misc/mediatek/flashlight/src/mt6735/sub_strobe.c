
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
#include <mt-plat/mt_gpio.h>
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif
static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);
static struct work_struct workTimeOut;
static void work_timeOutFunc(struct work_struct *data);


#define GPIO_CAMERA_FLASH_EN_PIN    83
#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0


int FL_Sub_Enable(void)
{
	if (g_duty < 0)
		g_duty = 0;
	else if (g_duty > 16)
		g_duty = 16;
	PK_DBG("gaoyangyang add test Front flash g_duty=%d\n", g_duty);
	if (g_duty) {
		//g_duty=1
		if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	} else {
		//g_duty=0	
		if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	}
	PK_DBG(" FL_Sub_Enable line=%d\n", __LINE__);
	return 0;
}

int FL_Sub_Disable(void)
{
	//Disable GPIO
	if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
	PK_DBG(" FL_Sub_Disable line=%d\n", __LINE__);
	return 0;
}

int FL_Sub_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_Sub_dim_duty line=%d\n", __LINE__);
	g_duty = duty;
	return 0;
}
int FL_Sub_Init(void)
{
    if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN,GPIO_MODE_00))    {PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}

    FL_Sub_Disable(); 
	return 0;
}
int FL_Sub_Uninit(void)
{
	FL_Sub_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Sub_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart SubledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void SubtimerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = SubledTimeOutCallback;
}


static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
	{
		int i4RetValue = 0;
		int ior_shift;
		int iow_shift;
		int iowr_shift;
	
		ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
		iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
		iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
		switch (cmd) {
	
		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
			g_timeOutTimeMs = arg;
			break;
	
	
		case FLASH_IOC_SET_DUTY:
			PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
			FL_Sub_dim_duty(arg);
			break;
	
	
		case FLASH_IOC_SET_STEP:
			PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);
	
			break;
	
		case FLASH_IOC_SET_ONOFF:
			PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
			if (arg == 1) {
	
				int s;
				int ms;
	
				if (g_timeOutTimeMs > 1000) {
					s = g_timeOutTimeMs / 1000;
					ms = g_timeOutTimeMs - s * 1000;
				} else {
					s = 0;
					ms = g_timeOutTimeMs;
				}
	
				if (g_timeOutTimeMs != 0) {
					ktime_t ktime;
	
					ktime = ktime_set(s, ms * 1000000);
					hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
				}
				FL_Sub_Enable();
			} else {
				FL_Sub_Disable();
				hrtimer_cancel(&g_timeOutTimer);
			}
			break;
		default:
			PK_DBG(" No such command\n");
			i4RetValue = -EPERM;
			break;
		}
		return i4RetValue;
	}
static int sub_strobe_open(void *pArg)
	{
		int i4RetValue = 0;
	
		PK_DBG("sub_strobe_open line=%d\n", __LINE__);
		if (0 == strobe_Res) {
			FL_Sub_Init();
			SubtimerInit();
		}
		PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
		spin_lock_irq(&g_strobeSMPLock);
	
	
		if (strobe_Res) {
			PK_DBG(" busy!\n");
			i4RetValue = -EBUSY;
		} else {
			strobe_Res += 1;
		}
	
	
		spin_unlock_irq(&g_strobeSMPLock);
		PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	
		return i4RetValue;
	
	}


static int sub_strobe_release(void *pArg)
	{
		PK_DBG(" sub_strobe_release\n");
	
		if (strobe_Res) {
			spin_lock_irq(&g_strobeSMPLock);
	
			strobe_Res = 0;
			strobe_Timeus = 0;
	
			/* LED On Status */
			g_strobe_On = FALSE;
	
			spin_unlock_irq(&g_strobeSMPLock);
	
			FL_Sub_Uninit();
		}
	
		PK_DBG(" Done\n");
	
		return 0;
	
	}


FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}
