/*
 * BU6429AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "BU6429AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

/* if use ISRC mode, should modify variables in init_setting */
#define USE_ISRC_MODE_S5K2P8_SENSOR

static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;


static int s4AF_ReadReg(unsigned short *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff[2];

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, pBuff, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C read failed!!\n");
		return -1;
	}

	*a_pu2Result = (((u16)(pBuff[0] & 0x03)) << 8) + pBuff[1];  //rohm

	return 0;
}

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	#ifdef USE_ISRC_MODE_S5K2P8_SENSOR
	char puSendCmd[2] = {(char)(((a_u2Data >> 8) & 0x03) | 0xF4), (char)(a_u2Data & 0xFF)};
	#else
	char puSendCmd[2] = {(char)(((a_u2Data >> 8) & 0x03) | 0xC0), (char)(a_u2Data & 0xFF)};
	#endif

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

#ifdef USE_ISRC_MODE_S5K2P8_SENSOR
static int init_setting(void)
{
	int  i4RetValue = 0;
	//LOG_INF("init_setting start 151201!!\n");
char puSendCmd1[2]={(char)(0x94),(char)(0x32)};//A point=50d 

	/* following 1 variables should be modified by module */
	/* resonant_frequency = 106; 1 ~ 255   0.4*106=72.4Hz */
	/* following 1 variables for ISRC mode */

	/* following 1 variables for Q fact,modified by module */


	/* convert frequency to register setting */

	/* step1: power on */
char puSendCmd2[2]={(char)(0x9C),(char)(0x64)};//B point=100d 
char puSendCmd3[2]={(char)(0xA4),(char)(0x84)};//A-B point step mode:200us/4Lsd 
char puSendCmd4[2]={(char)(0x8C),(char)(0x32)};//73Hz,Fast mode
 i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd1, 2);	
    if (i4RetValue < 0) 
    {
        LOG_INF("[BU6424AF]I2C send failed!!\n");
		return -1;
	}

	/* step2: set Q fact adjustment */


 i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
    if (i4RetValue < 0) 
    {
        LOG_INF("[BU6424AF]I2C send failed!!\n");
				return -1;
			}


	/* step3: resonant_frequency */

	/* puSendCmd[2] = (char)((MSB) & (Wx << 3)), (char)(resonant_frequency & 0xff); */
 i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
    if (i4RetValue < 0) 
    {
        LOG_INF("[BU6424AF]I2C send failed!!\n");
		return -1;
	}

	/* step4: set ISRC mode */

	/* puSendCmd[2] = (char)((MSB) & (Wx << 3)), (char)(Mode & 0xff); */
 i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd4, 2);
    if (i4RetValue < 0) 
    {
        LOG_INF("[BU6424AF]I2C send failed!!\n");
		return -1;
	}

	return 0;
}
#endif

static inline int getAFInfo(__user stAF_MotorInfo *pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		unsigned short InitPos;

		ret = s4AF_ReadReg(&InitPos);
		#ifdef USE_ISRC_MODE_S5K2P8_SENSOR
		init_setting();
		#endif

		if (ret == 0) {
			LOG_INF("gaoyangyang add Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	/* LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); */


	if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
	}

	return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long BU6429AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int BU6429AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		char puSendCmd[2];

		puSendCmd[0] = (char)(0x00);
		puSendCmd[1] = (char)(0x00);
		i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);
		LOG_INF("Wait\n");
		/*s4AF_WriteReg(200);
		msleep(20);
		s4AF_WriteReg(100);
		msleep(20);*/
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

void BU6429AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}
