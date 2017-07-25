/* drivers/i2c/chips/lis3dh.c - LIS3DH motion sensor driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <cust_acc.h>
#include "lis3dsh.h"
#include "accel.h"

//#define STEP_TEST

#ifdef STEP_TEST
#include <step_counter.h>
#endif

#define FIFO_I2C_LIMIT_8
#define CONFIG_LIS3DSH_LOWPASS   /*apply low pass filter on output*/       

#define LIS3DSH_DEV_NAME	"LIS3DSH"

#define DEBUG	1
#define GSE_TAG                 	 "[Gsensor] "
#define GSE_FUN(f)               	printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)	printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)	printk(KERN_ERR GSE_TAG fmt, ##args)

#define LIS3DSH_AXIS_X		0
#define LIS3DSH_AXIS_Y		1
#define LIS3DSH_AXIS_Z		2
#define LIS3DSH_AXES_NUM	3
#define LIS3DSH_DATA_LEN	6
#define C_MAX_FIR_LENGTH	32

typedef enum {
	LIS3DSH_ACC	= 1,
	LIS3DSH_STEP_C	= 2,
	LIS3DSH_TILT	= 3,
}LIS3DSH_INIT_TYPE;

typedef enum {
	ADX_TRC_FILTER  = 0x01,
	ADX_TRC_RAWDATA = 0x02,
	ADX_TRC_IOCTL   = 0x04,
	ADX_TRC_CALI	= 0X08,
	ADX_TRC_INFO	= 0X10,
} ADX_TRC;

struct scale_factor{
	u8  whole;
	u8  fraction;
};

struct data_resolution {
	struct scale_factor scalefactor;
	int	sensitivity;
};

#if defined(CONFIG_LIS3DSH_LOWPASS)
struct data_filter {
	s16	raw[C_MAX_FIR_LENGTH][LIS3DSH_AXES_NUM];
	int	sum[LIS3DSH_AXES_NUM];
	int	num;
	int	idx;
};
#endif

struct lis3dsh_i2c_data {
	struct i2c_client	*client;
	struct acc_hw	*hw;
	struct hwmsen_convert	cvt;

	struct data_resolution	*reso;
	struct work_struct		eint_work;
	atomic_t	trace;
	atomic_t	suspend;
	atomic_t	selftest;
	atomic_t	filter;
	s16		cali_sw[LIS3DSH_AXES_NUM+1];

	s8		offset[LIS3DSH_AXES_NUM+1];  /*+1: for 4-byte alignment*/
	s16		data[LIS3DSH_AXES_NUM+1];

#if defined(CONFIG_LIS3DSH_LOWPASS)
	atomic_t	firlen;
	atomic_t	fir_en;
	struct data_filter	fir;
#endif 

	atomic_t	enabled;
#ifdef STEP_TEST
	atomic_t	pedo_enabled;
#endif
};

static struct data_resolution lis3dsh_data_resolution[] = {
     /* combination by {FULL_RES,RANGE}*/
    {{ 1, 0}, 1024},   // dataformat +/-2g  in 12-bit resolution;  { 1, 0} = 1.0 = (2*2*1000)/(2^12);  1024 = (2^12)/(2*2) 
    {{ 1, 9}, 512},   // dataformat +/-4g  in 12-bit resolution;  { 1, 9} = 1.9 = (2*4*1000)/(2^12);  512 = (2^12)/(2*4) 
};

static struct data_resolution lis3dsh_offset_resolution = {{15, 6}, 64};

static bool sensor_power = false;
static bool acc_en  = false;
static struct GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
static bool enable_status = false;
static int sensor_suspend = 0;
static int lis3dsh_init_flag = 1;

static struct i2c_client *lis3dsh_i2c_client = NULL;
static struct lis3dsh_i2c_data *obj_i2c_data = NULL;
struct acc_hw lis3dsh_accel_cust;
static struct acc_hw *hw = &lis3dsh_accel_cust;

static DEFINE_MUTEX(lis3dsh_init_mutex);
static DEFINE_MUTEX(lis3dsh_i2c_mutex);
static DEFINE_MUTEX(lis3dsh_op_mutex);

static int  lis3dsh_local_init(void);
static int  lis3dsh_remove(void);
static struct acc_init_info lis3dsh_init_info = {
		.name	= LIS3DSH_DEV_NAME,
		.init		= lis3dsh_local_init,
		.uninit	= lis3dsh_remove,
};

#ifdef STEP_TEST
#define PEDOMETERMAX	0x7FFF
#define SIX_STEP			0x7FF9

/* SM_DATA[0] is  addr, SM_DATA[1,2,....] is data */
u8 Globe_stepnum = 0;
u8 SM_DATA[3] = {LIS3DSH_CTRL_REG1,
	0x01,0x01
}; //statemache#1,#2 enable  from reg 21 -22,2 byte

u8 SM_DATA_A[6] = {LIS3DSH_VFC_1,
	0x7D,0x72,0x4C, 0x26,0x00
};// from REG 0x1B  5 byte data

u8 SM_DATA_B[3] = {0x19, 
	0x00,0x00                            
}; // from reg 0x19-0x1A 2BYTE

u8 SM_DATA_C[9] = {0x50, 
	0x00,
	0x00,
	0x1C,
	0x00,
	0x12,
	0x00,
	0x00,
	0x0D
}; // from reg 0x50-0x57 8byte

u8 SM_DATA_D[8] ={0x59,
	0x00,
	0x03,
	0x21,
	0x00,
	0x1F,
	0x00,
	0x00
}; // from reg 0x59-0x5F 7byte

u8 SM_DATA_E[17] ={0X70,
	0x00,
	0x00,
	0x2D,
	0x00,
	0x5A,
	0x00,
	0x00,
	0x02,
	0x00,
	0x00,
	0x03,
	0x21,
	0x00,
	0x07,
	0x00,
	0x00
}; // from reg 0x70-0x7F 16byte     

u8 SM_DATA_F[17] = {0X40,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x11,
	0x0B
}; // from reg 0x40-0x4F 16byte

u8 SM_DATA_G[17] = {0X60,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0xBB,
	0xBB,
	0xBB,
	0x02,
	0x22,
	0x15,
	0x0B
}; // from reg 0x40-0x4F 16byte
u8 SM2_DATA_C[9] = {0x50, 
	0x00,
	0x00,
	0x00,
	0x00,
	0x48,
	0x00,
	0x00,
	0x0A
}; // from reg 0x50-0x57 8byte

u8 SM2_DATA_E[17] ={0X70,
	0x00,
	0x00,
	0x19,
	0x00,
	0x52,
	0x00,
	0x00,
	0x0B, //TH1 FOR RUNNING MODE
	0x00,
	0x00,
	0x03,
	0x21,
	0x00,
	0x07,
	0x00,
	0x00
}; // from reg 0x70-0x7F 16byte

u8 SM2_DATA_F[17] = {0X40,
	0x51,
	0x11,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x15,
	0x02,
	0x11,
	0x0B
}; // from reg 0x60-0x6F 16byte 
static bool STM_WALKING_MODE = TRUE;

extern struct acc_hw* lis3dsh_get_cust_acc_hw(void);
extern int step_c_register_data_path(struct step_c_data_path *data);
extern int step_c_driver_add(struct step_c_init_info* obj);
extern int step_c_register_control_path(struct step_c_control_path *ctl);
static int lis3dsh_step_c_local_init(void);
static int lis3dsh_step_c_local_uninit(void);

static struct step_c_init_info  lis3dsh_step_c_init_info = {
	.name	= "LIS3DSH_STEP_C",
	.init		= lis3dsh_step_c_local_init,
	.uninit	= lis3dsh_step_c_local_uninit,
};
#endif

struct acc_hw *lis3dsh_get_cust_acc(void)
{
	return &lis3dsh_accel_cust;
}

static int lis_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2]={{0},{0}};
	
	mutex_lock(&lis3dsh_i2c_mutex);
	
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len =1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len =len;
	msgs[1].buf = data;
	
	if (!client)
	{
		mutex_unlock(&lis3dsh_i2c_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE) 
	{
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&lis3dsh_i2c_mutex);
		return -EINVAL;
	}
	
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err < 0) 
	{
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
		err = -EIO;
	} 
	else 
		err = 0;

	mutex_unlock(&lis3dsh_i2c_mutex);
	return err;
}

static int lis_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err, idx, num;
	char buf[C_I2C_FIFO_SIZE];
	err =0;
	
	mutex_lock(&lis3dsh_i2c_mutex);
	
	if (!client)
	{
		mutex_unlock(&lis3dsh_i2c_mutex);
		return -EINVAL;
	}
	else if (len >= C_I2C_FIFO_SIZE) 
	{        
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&lis3dsh_i2c_mutex);
		return -EINVAL;
	}    

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
	    buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0)
	{
		GSE_ERR("send command error!!\n");
		mutex_unlock(&lis3dsh_i2c_mutex);
		return -EFAULT;
	}
	
	mutex_unlock(&lis3dsh_i2c_mutex);
	return err;
}

static void LIS3DSH_power(struct acc_hw *hw, unsigned int on) 
{
}

static int LIS3DSH_ReadData(struct i2c_client *client, s16 data[LIS3DSH_AXES_NUM])
{
	struct lis3dsh_i2c_data *priv = i2c_get_clientdata(client);        
	u8 buf[LIS3DSH_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
		err = -EINVAL;
	else
	{
		if((lis_i2c_read_block(client, LIS3DSH_REG_OUT_X, buf, 0x01))<0)
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		if((lis_i2c_read_block(client, LIS3DSH_REG_OUT_X+1, &buf[1], 0x01))<0)
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		
		data[LIS3DSH_AXIS_X] = (s16)((buf[0]+(buf[1]<<8))>>4);
		
		if((lis_i2c_read_block(client, LIS3DSH_REG_OUT_Y, &buf[2], 0x01))<0)
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		
		if((lis_i2c_read_block(client, LIS3DSH_REG_OUT_Y+1, &buf[3], 0x01))<0)
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		
		data[LIS3DSH_AXIS_Y] =  (s16)((s16)(buf[2] +( buf[3]<<8))>>4);

		if((lis_i2c_read_block(client, LIS3DSH_REG_OUT_Z, &buf[4], 0x01))<0)
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}

		if((lis_i2c_read_block(client, LIS3DSH_REG_OUT_Z+1, &buf[5], 0x01))<0)
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		
		data[LIS3DSH_AXIS_Z] =(s16)((buf[4]+(buf[5]<<8))>>4);
		data[LIS3DSH_AXIS_X] &= 0xfff;
		data[LIS3DSH_AXIS_Y] &= 0xfff;
		data[LIS3DSH_AXIS_Z] &= 0xfff;

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LIS3DSH_AXIS_X], data[LIS3DSH_AXIS_Y], data[LIS3DSH_AXIS_Z],
		                           data[LIS3DSH_AXIS_X], data[LIS3DSH_AXIS_Y], data[LIS3DSH_AXIS_Z]);
		}

		if(data[LIS3DSH_AXIS_X] & 0x800)
		{
			data[LIS3DSH_AXIS_X] = ~data[LIS3DSH_AXIS_X];
			data[LIS3DSH_AXIS_X] &= 0xfff;
			data[LIS3DSH_AXIS_X] +=1;
			data[LIS3DSH_AXIS_X] = -data[LIS3DSH_AXIS_X];
		}
		if(data[LIS3DSH_AXIS_Y] & 0x800)
		{
			data[LIS3DSH_AXIS_Y] = ~data[LIS3DSH_AXIS_Y];
			data[LIS3DSH_AXIS_Y] &= 0xfff;
			data[LIS3DSH_AXIS_Y] +=1;
			data[LIS3DSH_AXIS_Y] = -data[LIS3DSH_AXIS_Y];
		}
		if(data[LIS3DSH_AXIS_Z] & 0x800)
		{
			data[LIS3DSH_AXIS_Z] = ~data[LIS3DSH_AXIS_Z];
			data[LIS3DSH_AXIS_Z] &= 0xfff;
			data[LIS3DSH_AXIS_Z] +=1;
			data[LIS3DSH_AXIS_Z] = -data[LIS3DSH_AXIS_Z];
		}

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[LIS3DSH_AXIS_X], data[LIS3DSH_AXIS_Y], data[LIS3DSH_AXIS_Z],
					data[LIS3DSH_AXIS_X], data[LIS3DSH_AXIS_Y], data[LIS3DSH_AXIS_Z]);
		}
		
#ifdef CONFIG_LIS3DSH_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][LIS3DSH_AXIS_X] = data[LIS3DSH_AXIS_X];
					priv->fir.raw[priv->fir.num][LIS3DSH_AXIS_Y] = data[LIS3DSH_AXIS_Y];
					priv->fir.raw[priv->fir.num][LIS3DSH_AXIS_Z] = data[LIS3DSH_AXIS_Z];
					priv->fir.sum[LIS3DSH_AXIS_X] += data[LIS3DSH_AXIS_X];
					priv->fir.sum[LIS3DSH_AXIS_Y] += data[LIS3DSH_AXIS_Y];
					priv->fir.sum[LIS3DSH_AXIS_Z] += data[LIS3DSH_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][LIS3DSH_AXIS_X], priv->fir.raw[priv->fir.num][LIS3DSH_AXIS_Y], priv->fir.raw[priv->fir.num][LIS3DSH_AXIS_Z],
							priv->fir.sum[LIS3DSH_AXIS_X], priv->fir.sum[LIS3DSH_AXIS_Y], priv->fir.sum[LIS3DSH_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[LIS3DSH_AXIS_X] -= priv->fir.raw[idx][LIS3DSH_AXIS_X];
					priv->fir.sum[LIS3DSH_AXIS_Y] -= priv->fir.raw[idx][LIS3DSH_AXIS_Y];
					priv->fir.sum[LIS3DSH_AXIS_Z] -= priv->fir.raw[idx][LIS3DSH_AXIS_Z];
					priv->fir.raw[idx][LIS3DSH_AXIS_X] = data[LIS3DSH_AXIS_X];
					priv->fir.raw[idx][LIS3DSH_AXIS_Y] = data[LIS3DSH_AXIS_Y];
					priv->fir.raw[idx][LIS3DSH_AXIS_Z] = data[LIS3DSH_AXIS_Z];
					priv->fir.sum[LIS3DSH_AXIS_X] += data[LIS3DSH_AXIS_X];
					priv->fir.sum[LIS3DSH_AXIS_Y] += data[LIS3DSH_AXIS_Y];
					priv->fir.sum[LIS3DSH_AXIS_Z] += data[LIS3DSH_AXIS_Z];
					priv->fir.idx++;
					data[LIS3DSH_AXIS_X] = priv->fir.sum[LIS3DSH_AXIS_X]/firlen;
					data[LIS3DSH_AXIS_Y] = priv->fir.sum[LIS3DSH_AXIS_Y]/firlen;
					data[LIS3DSH_AXIS_Z] = priv->fir.sum[LIS3DSH_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][LIS3DSH_AXIS_X], priv->fir.raw[idx][LIS3DSH_AXIS_Y], priv->fir.raw[idx][LIS3DSH_AXIS_Z],
						priv->fir.sum[LIS3DSH_AXIS_X], priv->fir.sum[LIS3DSH_AXIS_Y], priv->fir.sum[LIS3DSH_AXIS_Z],
						data[LIS3DSH_AXIS_X], data[LIS3DSH_AXIS_Y], data[LIS3DSH_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
}

static int LIS3DSH_ResetCalibration(struct i2c_client *client)
{
	struct lis3dsh_i2c_data *obj = i2c_get_clientdata(client);	
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;     
}

static int LIS3DSH_ReadCalibration(struct i2c_client *client, int dat[LIS3DSH_AXES_NUM])
{
    struct lis3dsh_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[LIS3DSH_AXIS_X]] = obj->cvt.sign[LIS3DSH_AXIS_X]*obj->cali_sw[LIS3DSH_AXIS_X];
    dat[obj->cvt.map[LIS3DSH_AXIS_Y]] = obj->cvt.sign[LIS3DSH_AXIS_Y]*obj->cali_sw[LIS3DSH_AXIS_Y];
    dat[obj->cvt.map[LIS3DSH_AXIS_Z]] = obj->cvt.sign[LIS3DSH_AXIS_Z]*obj->cali_sw[LIS3DSH_AXIS_Z];                        
                                       
    return 0;
}

static int LIS3DSH_WriteCalibration(struct i2c_client *client, int dat[LIS3DSH_AXES_NUM])
{
	struct lis3dsh_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	GSE_FUN();
	
	if(!obj || ! dat)
	{
		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	}
	else
	{        
		s16 cali[LIS3DSH_AXES_NUM];
		cali[obj->cvt.map[LIS3DSH_AXIS_X]] = obj->cvt.sign[LIS3DSH_AXIS_X]*obj->cali_sw[LIS3DSH_AXIS_X];
		cali[obj->cvt.map[LIS3DSH_AXIS_Y]] = obj->cvt.sign[LIS3DSH_AXIS_Y]*obj->cali_sw[LIS3DSH_AXIS_Y];
		cali[obj->cvt.map[LIS3DSH_AXIS_Z]] = obj->cvt.sign[LIS3DSH_AXIS_Z]*obj->cali_sw[LIS3DSH_AXIS_Z]; 
		cali[LIS3DSH_AXIS_X] += dat[LIS3DSH_AXIS_X];
		cali[LIS3DSH_AXIS_Y] += dat[LIS3DSH_AXIS_Y];
		cali[LIS3DSH_AXIS_Z] += dat[LIS3DSH_AXIS_Z];

		obj->cali_sw[LIS3DSH_AXIS_X] += obj->cvt.sign[LIS3DSH_AXIS_X]*dat[obj->cvt.map[LIS3DSH_AXIS_X]];
		obj->cali_sw[LIS3DSH_AXIS_Y] += obj->cvt.sign[LIS3DSH_AXIS_Y]*dat[obj->cvt.map[LIS3DSH_AXIS_Y]];
		obj->cali_sw[LIS3DSH_AXIS_Z] += obj->cvt.sign[LIS3DSH_AXIS_Z]*dat[obj->cvt.map[LIS3DSH_AXIS_Z]];
	} 

	return err;
}

static int LIS3DSH_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2]={0};    
	u8 addr = LIS3DSH_REG_DEVID;

	if(lis_i2c_read_block(client, addr, databuf, 0x01))
	{
		GSE_ERR("read DeviceID register err!\n");
		return LIS3DSH_ERR_I2C;
	}
	
	GSE_LOG("LIS3DSH Device ID=0x%x,respect=0x%x\n",databuf[0],WHO_AM_I);
	
/*DO Not meet the datasheet!!! abort   */
	if(databuf[0]!= WHO_AM_I)
		return LIS3DSH_ERR_IDENTIFICATION;

	return LIS3DSH_SUCCESS;
}

static int LIS3DSH_SetDataResolution(struct lis3dsh_i2c_data *obj)
{
	int err;
	u8  dat, reso;

	if((err = lis_i2c_read_block(obj->client, LIS3DSH_REG_CTL_REG_5, &dat,0x01))<0)
	{
		GSE_ERR("write data format fail!!\n");
		return err;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
	reso  = (dat & 0x38)>>3;
	if(reso >= 0x3)
		reso = 0x3;//do not use +/- 16g
	
	if(reso < sizeof(lis3dsh_data_resolution)/sizeof(lis3dsh_data_resolution[0]))
	{        
		obj->reso = &lis3dsh_data_resolution[reso];
		return 0;
	}
	else
		return -EINVAL;
}

static int LIS3DSH_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];
	u8 addr = LIS3DSH_REG_CTL_REG_4;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	
	if((lis_i2c_read_block(client, addr, databuf, 0x01))<0)
	{
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS3DSH_ERR_I2C;
	}

	GSE_LOG("LIS3DSH_SetBWRate read from REG5 is 0x%x\n",databuf[0]);

	databuf[0] &= ~0xF0;
	databuf[0] |= bwrate;

	res = lis_i2c_write_block(client, LIS3DSH_REG_CTL_REG_4, databuf, 0x1);
	if(res < 0)
		return LIS3DSH_ERR_I2C;
	
	return LIS3DSH_SUCCESS;    
}

static int LIS3DSH_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];    
	int res = 0;
	u8 addr = LIS3DSH_REG_CTL_REG_4;
	struct lis3dsh_i2c_data *obj = i2c_get_clientdata(client);
       GSE_FUN();
	   
	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return LIS3DSH_SUCCESS;
	}

	if(enable == true)
	{
#ifdef STEP_TEST	   
		if(!atomic_read(&obj->pedo_enabled))
#endif			
			res = LIS3DSH_SetBWRate(client, LIS3DSH_BW_100HZ);//400 or 100 no other choice
	}
	else
	{
		GSE_ERR("TRY TO POWEROFF\n");
#ifdef  STEP_TEST 
		if(!atomic_read(&obj->pedo_enabled))
#endif
		{
			if(lis_i2c_read_block(client, addr, databuf, 0x01) <0)
			{
				GSE_ERR("read power ctl register err!\n");
				return LIS3DSH_ERR_I2C;
			}

			databuf[1] = 0x07;
			databuf[0] = LIS3DSH_REG_CTL_REG_4;
			
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				GSE_ERR("set power mode failed!\n");
				return LIS3DSH_ERR_I2C;
			}
       	}
	}
	
	if(atomic_read(&obj->trace) & ADX_TRC_INFO)
		GSE_LOG("set power mode ok %d!\n", databuf[1]);

	sensor_power = enable;
	enable_status = sensor_power;
	GSE_LOG("set power mode sensor_power %d!\n",sensor_power);
	return LIS3DSH_SUCCESS;    
}

static int LIS3DSH_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct lis3dsh_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	u8 addr = LIS3DSH_REG_CTL_REG_5;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);

	if((lis_i2c_read_block(client, addr, databuf, 0x01))<0)
	{
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS3DSH_ERR_I2C;
	}

	databuf[0] &= ~0x38;
	databuf[0] |=dataformat;
	
	res = lis_i2c_write_block(client, LIS3DSH_REG_CTL_REG_5, databuf, 0x1);
	if(res < 0)
		return LIS3DSH_ERR_I2C;

	return LIS3DSH_SetDataResolution(obj);    
}

#ifdef STEP_TEST
static void dumpReg(struct i2c_client *client)
{
	int i=0;
	u8 addr = 0x10;
	u8 regdata=0;
	for(i=0; i<(0x80-0x10) ; i++)
	{
		lis_i2c_read_block(client,addr,&regdata,1);
		GSE_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
		addr++;
	}
}

static int LIS3DSH_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[2];
	u8 addr = LIS3DSH_REG_CTL_REG3;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2); 

	if((lis_i2c_read_block(client, addr, databuf, 0x01))<0)
	{
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS3DSH_ERR_I2C;
	}

	databuf[0] = 0x00;

	res = lis_i2c_write_block(client, LIS3DSH_REG_CTL_REG_3, databuf, 0x01);
	if(res < 0)
	{
		return LIS3DSH_ERR_I2C;
	}
	
	return LIS3DSH_SUCCESS;    
}
#endif

static int LIS3DSH_Init(struct i2c_client *client, int reset_cali)
{
	struct lis3dsh_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	u8 databuf[2] = {0, 0};
	GSE_FUN();

	res = LIS3DSH_CheckDeviceID(client); 
	if(res != LIS3DSH_SUCCESS)
		return res;

	databuf[0] = 0x0f;
	res = lis_i2c_write_block(client, LIS3DSH_REG_CTL_REG_4, databuf, 0x01);
	if(res < 0)
	{
		GSE_ERR("LIS3DSH_Init step 1!\n");
		return res;
	}

	res = LIS3DSH_SetBWRate(client, LIS3DSH_BW_100HZ); //400 or 100 no other choice
	if(res < 0)
	{
		GSE_ERR("LIS3DSH_Init step 2!\n");
		return res;
	}

	res = LIS3DSH_SetDataFormat(client, LIS3DSH_RANGE_4G);//4g no oher choise
	if(res < 0) 
	{
		GSE_ERR("LIS3DSH_Init step 3!\n");
		return res;
	}
	
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = LIS3DSH_SetPowerMode(client, enable_status);
	if(res < 0)
	{
		GSE_ERR("LIS3DSH_Init step 5!\n");
		return res;
	}

	if(0 != reset_cali)
	{ 
		//reset calibration only in power on
		res = LIS3DSH_ResetCalibration(client);
		if(res < 0)
			return res;
	}

#ifdef CONFIG_LIS3DSH_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return LIS3DSH_SUCCESS;
}

static int LIS3DSH_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
		return -1;
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "LIS3DSH Chip");
	return 0;
}

static int LIS3DSH_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct lis3dsh_i2c_data *obj = (struct lis3dsh_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[LIS3DSH_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
		return -1;

	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_suspend == 1)
	{
		//GSE_LOG("sensor in suspend read not data!\n");
		return 0;
	}

	if((res = LIS3DSH_ReadData(client, obj->data)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		obj->data[LIS3DSH_AXIS_X] += obj->cali_sw[LIS3DSH_AXIS_X];
		obj->data[LIS3DSH_AXIS_Y] += obj->cali_sw[LIS3DSH_AXIS_Y];
		obj->data[LIS3DSH_AXIS_Z] += obj->cali_sw[LIS3DSH_AXIS_Z];
		
		/*remap coordinate*/
		acc[obj->cvt.map[LIS3DSH_AXIS_X]] = obj->cvt.sign[LIS3DSH_AXIS_X]*obj->data[LIS3DSH_AXIS_X];
		acc[obj->cvt.map[LIS3DSH_AXIS_Y]] = obj->cvt.sign[LIS3DSH_AXIS_Y]*obj->data[LIS3DSH_AXIS_Y];
		acc[obj->cvt.map[LIS3DSH_AXIS_Z]] = obj->cvt.sign[LIS3DSH_AXIS_Z]*obj->data[LIS3DSH_AXIS_Z];

		//Out put the mg
		acc[LIS3DSH_AXIS_X] = acc[LIS3DSH_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LIS3DSH_AXIS_Y] = acc[LIS3DSH_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LIS3DSH_AXIS_Z] = acc[LIS3DSH_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		

		sprintf(buf, "%04x %04x %04x", acc[LIS3DSH_AXIS_X], acc[LIS3DSH_AXIS_Y], acc[LIS3DSH_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		//	dumpReg(client);
		}
	}
	
	return 0;
}

static int LIS3DSH_ReadRawData(struct i2c_client *client, char *buf)
{
	struct lis3dsh_i2c_data *obj = (struct lis3dsh_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
		return EINVAL;
	
	if((res = LIS3DSH_ReadData(client, obj->data)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "%04x %04x %04x", obj->data[LIS3DSH_AXIS_X], 
			obj->data[LIS3DSH_AXIS_Y], obj->data[LIS3DSH_AXIS_Z]);
	}
	
	return 0;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dsh_i2c_client;
	char strbuf[LIS3DSH_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LIS3DSH_ReadChipInfo(client, strbuf, LIS3DSH_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dsh_i2c_client;
	char strbuf[LIS3DSH_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LIS3DSH_ReadSensorData(client, strbuf, LIS3DSH_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dsh_i2c_client;
	struct lis3dsh_i2c_data *obj;
	int err, len, mul;
	int tmp[LIS3DSH_AXES_NUM];	
	len = 0;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);

	if((err = LIS3DSH_ReadCalibration(client, tmp)))
		return -EINVAL;
	else
	{    
		mul = obj->reso->sensitivity/lis3dsh_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[LIS3DSH_AXIS_X], obj->offset[LIS3DSH_AXIS_Y], obj->offset[LIS3DSH_AXIS_Z],
			obj->offset[LIS3DSH_AXIS_X], obj->offset[LIS3DSH_AXIS_Y], obj->offset[LIS3DSH_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[LIS3DSH_AXIS_X], obj->cali_sw[LIS3DSH_AXIS_Y], obj->cali_sw[LIS3DSH_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[LIS3DSH_AXIS_X]*mul + obj->cali_sw[LIS3DSH_AXIS_X],
			obj->offset[LIS3DSH_AXIS_Y]*mul + obj->cali_sw[LIS3DSH_AXIS_Y],
			obj->offset[LIS3DSH_AXIS_Z]*mul + obj->cali_sw[LIS3DSH_AXIS_Z],
			tmp[LIS3DSH_AXIS_X], tmp[LIS3DSH_AXIS_Y], tmp[LIS3DSH_AXIS_Z]);
		
		return len;
    }
}

static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = lis3dsh_i2c_client;  
	int err, x, y, z;
	int dat[LIS3DSH_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if((err = LIS3DSH_ResetCalibration(client)))
			GSE_ERR("reset offset err = %d\n", err);
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[LIS3DSH_AXIS_X] = x;
		dat[LIS3DSH_AXIS_Y] = y;
		dat[LIS3DSH_AXIS_Z] = z;
		if((err = LIS3DSH_WriteCalibration(client, dat)))
			GSE_ERR("write calibration err = %d\n", err);
	}
	else
		GSE_ERR("invalid format\n");
	
	return count;
}

static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dsh_i2c_client;
	struct lis3dsh_i2c_data *obj;
	u8 data;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	lis_i2c_read_block(client,LIS3DSH_REG_CTL_REG_4,&data,0x01);
    	return snprintf(buf, PAGE_SIZE, "%x\n", data);
}

static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LIS3DSH_LOWPASS
	struct i2c_client *client = lis3dsh_i2c_client;
	struct lis3dsh_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][LIS3DSH_AXIS_X], obj->fir.raw[idx][LIS3DSH_AXIS_Y], obj->fir.raw[idx][LIS3DSH_AXIS_Z]);
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[LIS3DSH_AXIS_X], obj->fir.sum[LIS3DSH_AXIS_Y], obj->fir.sum[LIS3DSH_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[LIS3DSH_AXIS_X]/len, obj->fir.sum[LIS3DSH_AXIS_Y]/len, obj->fir.sum[LIS3DSH_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_LIS3DSH_LOWPASS
	struct i2c_client *client = lis3dsh_i2c_client;  
	struct lis3dsh_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
		GSE_ERR("invallid format\n");
	else if(firlen > C_MAX_FIR_LENGTH)
		GSE_ERR("exceeds maximum filter length\n");
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(0 == firlen)
			atomic_set(&obj->fir_en, 0);
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lis3dsh_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lis3dsh_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
	{
		//GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;    
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct lis3dsh_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");

	return len;    
}

static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(power, S_IRUGO, show_power_status, NULL);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);

static struct driver_attribute *lis3dsh_attr_list[] = {
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_cali,
	&driver_attr_power,
	&driver_attr_firlen,
	&driver_attr_trace,
	&driver_attr_status,        
};

static int lis3dsh_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(lis3dsh_attr_list)/sizeof(lis3dsh_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, lis3dsh_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", lis3dsh_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

static int lis3dsh_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(lis3dsh_attr_list)/sizeof(lis3dsh_attr_list[0]));

	if(driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
		driver_remove_file(driver, lis3dsh_attr_list[idx]);

	return err;
}

static int lis3dsh_open(struct inode *inode, struct file *file)
{
	file->private_data = lis3dsh_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int lis3dsh_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long lis3dsh_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)

{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct lis3dsh_i2c_data *obj = (struct lis3dsh_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[LIS3DSH_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			LIS3DSH_Init(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			LIS3DSH_ReadChipInfo(client, strbuf, LIS3DSH_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			LIS3DSH_SetPowerMode(client,true);
			LIS3DSH_ReadSensorData(client, strbuf, LIS3DSH_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_OFFSET:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			if(copy_to_user(data, &gsensor_offset, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			LIS3DSH_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				cali[LIS3DSH_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[LIS3DSH_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[LIS3DSH_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = LIS3DSH_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = LIS3DSH_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if((err = LIS3DSH_ReadCalibration(client, cali)))
			{
				break;
			}
			
			sensor_data.x = cali[LIS3DSH_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[LIS3DSH_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[LIS3DSH_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}

#ifdef CONFIG_COMPAT
static long lis3dsh_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;
	void __user *arg32 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
	switch (cmd)
	{
		case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
			if (arg32 == NULL)
			{
			    err = -EINVAL;
			    break;    
			}
		
			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
			if (err){
				GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
				return err;
			}
			break;
			
		case COMPAT_GSENSOR_IOCTL_SET_CALI:
			if (arg32 == NULL)
			{
				err = -EINVAL;
				break;    
			}

			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
			if (err){
				GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
				return err;
			}
			break;

		case COMPAT_GSENSOR_IOCTL_GET_CALI:
			if (arg32 == NULL)
			{
				err = -EINVAL;
				break;    
			}

			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
			if (err){
				GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
				return err;
			}
			break;
			
		case COMPAT_GSENSOR_IOCTL_CLR_CALI:
			if (arg32 == NULL)
			{
				err = -EINVAL;
				break;    
			}

			err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
			if (err){
				GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
				return err;
			}
			break;

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
		break;
	}

	return err;
}
#endif

static struct file_operations lis3dsh_fops = {
	.owner	= THIS_MODULE,
	.open	= lis3dsh_open,
	.release	= lis3dsh_release,
	.unlocked_ioctl = lis3dsh_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= lis3dsh_compat_ioctl,
#endif
};

static struct miscdevice lis3dsh_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &lis3dsh_fops,
};

static int lis3dsh_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct lis3dsh_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	GSE_FUN();   
	
	mutex_lock(&lis3dsh_op_mutex);
	
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{	
			mutex_unlock(&lis3dsh_op_mutex);
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		if((err = LIS3DSH_SetPowerMode(obj->client, false)))
		{
			GSE_ERR("write power control fail!!\n");
			mutex_unlock(&lis3dsh_op_mutex);
			return -EINVAL;  
		}
		
		atomic_set(&obj->suspend, 1);
		LIS3DSH_power(obj->hw, 0);
	}
	
	sensor_suspend = 1;
	mutex_unlock(&lis3dsh_op_mutex);
	return err;
}

static int lis3dsh_resume(struct i2c_client *client)
{
	struct lis3dsh_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();
	
	mutex_lock(&lis3dsh_op_mutex);
	
	if(obj == NULL)
	{
		mutex_unlock(&lis3dsh_op_mutex);
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	LIS3DSH_power(obj->hw, 1);
      err = LIS3DSH_Init(client, 0);
	if(err)
	{
		mutex_unlock(&lis3dsh_op_mutex);
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->suspend, 0);
	sensor_suspend = 0;	
	mutex_unlock(&lis3dsh_op_mutex);
	return 0;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int lis3dsh_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int lis3dsh_enable_nodata(int en)
{
	int res =0;
	bool power = false;
	
	if(1==en)
	{   
		acc_en =true;
		power = true;
	}
	
	if(0==en)
	{
		power = false;
		acc_en =false;
	}
	
	res = LIS3DSH_SetPowerMode(obj_i2c_data->client, power);
	if(res != LIS3DSH_SUCCESS)
	{
		GSE_ERR("LIS3DSH_SetPowerMode fail!\n");
		return -1;
	}
	
	GSE_LOG("lis3dsh_enable_nodata OK!\n");
	return 0;
}

static int lis3dsh_set_delay(u64 ns)
{
	int value =0;
	int sample_delay=0;
	int err;
	
	value = (int)ns/1000/1000;
	if(value <= 5)
	{
		sample_delay = LIS3DSH_BW_100HZ;
	}
	else if(value <= 10)
	{
		sample_delay = LIS3DSH_BW_100HZ;
	}
	else
	{
		sample_delay = LIS3DSH_BW_100HZ;
	}
	
	mutex_lock(&lis3dsh_op_mutex);
	err = LIS3DSH_SetBWRate(obj_i2c_data->client, sample_delay);
	if(err != LIS3DSH_SUCCESS ) 
	{
		GSE_ERR("Set delay parameter error!\n");
	}
	mutex_unlock(&lis3dsh_op_mutex);
	
	if(value >= 50)
	{
		atomic_set(&obj_i2c_data->filter, 0);
	}
	else
	{					
		obj_i2c_data->fir.num = 0;
		obj_i2c_data->fir.idx = 0;
		obj_i2c_data->fir.sum[LIS3DSH_AXIS_X] = 0;
		obj_i2c_data->fir.sum[LIS3DSH_AXIS_Y] = 0;
		obj_i2c_data->fir.sum[LIS3DSH_AXIS_Z] = 0;
		atomic_set(&obj_i2c_data->filter, 1);
	}
	
	GSE_LOG("lis3dsh_set_delay (%d)\n",value);
	return 0;
}

static int lis3dsh_get_data(int* x ,int* y,int* z, int* status)
{
	char buff[LIS3DSH_BUFSIZE];
	LIS3DSH_ReadSensorData(obj_i2c_data->client, buff, LIS3DSH_BUFSIZE);
	
	sscanf(buff, "%x %x %x", x, y, z);		
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

#ifdef STEP_TEST
/******************************************
  motion sensor pedometer INIT
*******************************************/
void lis3dsh_pedometer_init(struct lis3dsh_i2c_data *acc)
{
	int i;
	int ii = 0;
	int err = -1;
	u8 buf[2] = { 0 };
	GSE_FUN();

	buf[0] = LIS3DSH_CTRL_REG3;
	buf[1] = 0x4C;
	err = lis_i2c_write_block(acc->client, LIS3DSH_CTRL_REG3, &buf[1], 0x01);

	err=  LIS3DSH_SetPowerMode(acc->client, true);

	buf[0] = LIS3DSH_CTRL_REG6;
	buf[1] = 0x10;
	err = lis_i2c_write_block(acc->client,LIS3DSH_CTRL_REG6, &buf[1], 1);
}

void lis3dsh_StateMachine_2_init(struct lis3dsh_i2c_data *acc)
{
	int i;
	int ii = 0;
	int err = -1;
	u8 buf[17] = { 0 };
	u8 bufread[17] = { 0 };
	GSE_FUN();

#ifdef FIFO_I2C_LIMIT_8	
	buf[0] = SM2_DATA_C[0];
	for(i = 1; i <= 8; i++){
		buf[i] = SM2_DATA_C[i];
		err = lis_i2c_write_block(acc->client, buf[0],&buf[i], 1); 
		buf[0] ++;
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
#else
	//err = STLIS3DSH_ConWriteBytes(I2C_AUTO_INCREMENT |SM_DATA_C[0],&SM_DATA_C[1],8);
	buf[0] = (I2C_AUTO_INCREMENT |SM2_DATA_C[0]);
	for(i = 1; i <= 8; i++){
		buf[i] = SM2_DATA_C[i];
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = lis_i2c_write_block(acc->client, buf[0],&buf[1], 8); 
#endif	
	
#ifdef FIFO_I2C_LIMIT_8
	buf[0] = SM2_DATA_E[0];
	for(i = 1; i <= 16; i++){
		buf[i] = SM2_DATA_E[i];
		err = lis_i2c_write_block(acc->client, buf[0],&buf[i], 1); 
		buf[0] ++;
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
#else
	// err = STLIS3DSH_ConWriteBytes(I2C_AUTO_INCREMENT |SM_DATA_E[0],&SM_DATA_E[1],16);
	buf[0] = (I2C_AUTO_INCREMENT |SM2_DATA_E[0]);
	for(i = 1; i <= 16; i++){
		buf[i] = SM2_DATA_E[i];
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err =  lis_i2c_write_block(acc->client, buf[0],&buf[1], 16); 
#endif	

#ifdef FIFO_I2C_LIMIT_8
	buf[0] = SM2_DATA_F[0];
	for(i = 1; i <= 16; i++){
		buf[i] = SM2_DATA_F[i];
		err = lis_i2c_write_block(acc->client, buf[0],&buf[i], 1); 
		buf[0] ++;
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
#else
	//err =  STLIS3DSH_ConWriteBytes(I2C_AUTO_INCREMENT |SM_DATA_F[0],&SM_DATA_F[1],16);
	buf[0] = (I2C_AUTO_INCREMENT |SM2_DATA_F[0]);
		for(i = 1; i <= 16; i++){
		buf[i] = SM2_DATA_F[i];
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = lis_i2c_write_block(acc->client, buf[0],&buf[1], 16); 
#endif
}

void lis3dsh_StateMachine_init(struct lis3dsh_i2c_data *acc)
{
	int i;
	int ii = 0;
	int err = -1;
	u8 buf[17] = { 0 };
	u8 bufread[17] = { 0 };
	
	buf[0] = (I2C_AUTO_INCREMENT | SM_DATA[0]);
	for(i = 1; i <= 2; i++){
		buf[i] = SM_DATA[i];
		GSE_LOG( "lis3dsh_StateMachine_init write SM_DATA i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = lis_i2c_write_block(acc->client, buf[0],&buf[1], 2);
	
	bufread[0] = (I2C_AUTO_INCREMENT | SM_DATA[0]);
	err = lis_i2c_read_block(acc->client,bufread[0],&bufread[1],2);
	for(ii = 1 ; ii <= 2 ; ii++)
	{
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init read SM_DATA ii=%d,buf[%d]=0x%02x\n", ii, ii, bufread[ii]);
	}	

	buf[0] = (I2C_AUTO_INCREMENT |SM_DATA_A[0]);
	for(i = 1; i <= 5; i++){
		buf[i] = SM_DATA_A[i];
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = lis_i2c_write_block(acc->client, buf[0], &buf[1], 5);

	buf[0] = (I2C_AUTO_INCREMENT |SM_DATA_B[0]);
	for(i = 1; i <= 2; i++){
		buf[i] = SM_DATA_B[i];
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err =err = lis_i2c_write_block(acc->client, buf[0],&buf[1], 2);

#ifdef FIFO_I2C_LIMIT_8	
	buf[0] = SM_DATA_C[0];
	for(i = 1; i <= 8; i++){
		buf[i] = SM_DATA_C[i];
		err = lis_i2c_write_block(acc->client, buf[0],&buf[i], 1); 
		buf[0] ++;
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
#else
	buf[0] = (I2C_AUTO_INCREMENT |SM_DATA_C[0]);
	for(i = 1; i <= 8; i++){
		buf[i] = SM_DATA_C[i];
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = lis_i2c_write_block(acc->client, buf[0],&buf[1], 8); 
#endif

	buf[0] = (I2C_AUTO_INCREMENT |SM_DATA_D[0]);
	for(i = 1; i <= 7; i++){
		buf[i] = SM_DATA_D[i];
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err =  lis_i2c_write_block(acc->client, buf[0],&buf[1], 7); 

#ifdef FIFO_I2C_LIMIT_8
	buf[0] = SM_DATA_E[0];
	for(i = 1; i <= 16; i++){
		buf[i] = SM_DATA_E[i];
		err = lis_i2c_write_block(acc->client, buf[0],&buf[i], 1); 
		buf[0] ++;
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
#else
	buf[0] = (I2C_AUTO_INCREMENT |SM_DATA_E[0]);
	for(i = 1; i <= 16; i++){
		buf[i] = SM_DATA_E[i];
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err =  lis_i2c_write_block(acc->client, buf[0],&buf[1], 16); 
#endif	

#ifdef FIFO_I2C_LIMIT_8
	buf[0] = SM_DATA_F[0];
	for(i = 1; i <= 16; i++){
		buf[i] = SM_DATA_F[i];
		err = lis_i2c_write_block(acc->client, buf[0],&buf[i], 1); 
		buf[0] ++;
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
#else
	buf[0] = (I2C_AUTO_INCREMENT |SM_DATA_F[0]);
	for(i = 1; i <= 16; i++){
		buf[i] = SM_DATA_F[i];
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = lis_i2c_write_block(acc->client, buf[0],&buf[1], 16); 
#endif

#ifdef FIFO_I2C_LIMIT_8
	buf[0] = SM_DATA_G[0];
	for(i = 1; i <= 16; i++){
		buf[i] = SM_DATA_G[i];
		err = lis_i2c_write_block(acc->client, buf[0],&buf[i], 1); 
		buf[0] ++;
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
#else
	buf[0] = (I2C_AUTO_INCREMENT |SM_DATA_G[0]);
	for(i = 1; i <= 16; i++){
		buf[i] = SM_DATA_G[i];
		//GSE_LOG(&acc->client->dev, "lis3dsh_StateMachine_init i=%d,buf[%d]=0x%02x\n", i, i, buf[i]);
	};
	err = lis_i2c_write_block(acc->client, buf[0],&buf[1], 16); 
#endif	 
}

/******************************************
	motion sensor start_pedometer
*******************************************/
static void lis3dsh_start_pedometer(struct lis3dsh_i2c_data *acc,u16 value)
{
	u8 err = -1;
	u8 config[2] = { 0 };
	
	config[0] = LIS3DSH_LC_L;
	config[1] = (u8) (value & 0x00FF);
	err = lis_i2c_write_block(acc->client, config[0],&config[1],1);
	
	config[0] = LIS3DSH_LC_H;
	config[1] = (u8) (value >> 8);
	err = lis_i2c_write_block(acc->client, config[0],&config[1], 1);
	
	Globe_stepnum  = 0;
}

/******************************************
	motion sensor pedometer read
*******************************************/
static void lis3dsh_acc_get_pedometer_step(struct lis3dsh_i2c_data *acc,u16 *step_Num)
{	
	int err = -1;
	u16 temp_step =0x0000;
	u8 reg_addr;
	u8 acc_step[3];
	GSE_FUN();  

	//dumpReg(acc->client);

	reg_addr = (I2C_AUTO_INCREMENT | LIS3DSH_LC_L);
	err = lis_i2c_read_block(acc->client, reg_addr,acc_step, 2);

	if(-1 == err)
	{
		GSE_LOG("Motion Sensor get data fail!\n\r");
	}else{
		temp_step =  ((u16) ((acc_step[1] << 8) | acc_step[0]));
		if ((0xff ==acc_step[1]) ||(temp_step ==0)){
			lis3dsh_start_pedometer(obj_i2c_data,0x7FFF);
			lis_i2c_read_block(acc->client, reg_addr,acc_step, 2);		 
		}
		
		temp_step = PEDOMETERMAX - ((u16) ((acc_step[1] << 8) | acc_step[0]));
		
		GSE_LOG("lis3dsh step %d , raw data,H,L  %x,%x, STM_MODE %d\n",temp_step,acc_step[1] ,acc_step[0], STM_WALKING_MODE );
		//Globe_stepnum = Globe_stepnum + temp_step;
		//*step_Num = Globe_stepnum;
		*step_Num = temp_step;	
	}
}

static int LIS3DSH_pedometer_enable(struct lis3dsh_i2c_data *acc)
{
	GSE_FUN();
	
	if (!atomic_cmpxchg(&acc->pedo_enabled, 0, 1)) {
		lis3dsh_pedometer_init(acc);
		lis3dsh_StateMachine_init(acc);
		//lis3dsh_start_pedometer(acc);
		GSE_LOG("lis3dsh dump register pedometer enable phase\n");
		//dumpReg(acc->client);
	}
	else
	{
		if(atomic_read(&acc->pedo_enabled))
			GSE_LOG("pedo_enable already enable 1\n");
	}
	return 0;
}
static int LIS3DSH_pedometer_disable(struct lis3dsh_i2c_data *acc)
{
	u8 buf[3] =  {I2C_AUTO_INCREMENT|LIS3DSH_CTRL_REG1,  0x00,0x00};                     // disable ST1,ST2
 	int err =0;
	GSE_FUN();

	return err;
}

static int lis3dsh_step_c_open_report_data(int open)
{
	return LIS3DSH_SUCCESS;
}

static int lis3dsh_step_c_enable_nodata(int en)
{
	int res =0;
	int value = en;
	int err = 0;
	struct lis3dsh_i2c_data *priv = obj_i2c_data;

   	printk("Kaka lis3dsh_step_c_enable_nodata OK en==%d!\n",en);

	if(priv == NULL)
	{
		GSE_ERR("%s obj_i2c_data is NULL!\n", __func__);
		return -1;
	}

	if(value == 1)
	{
		LIS3DSH_pedometer_enable(priv);
		mt_eint_unmask(CUST_EINT_GSE_1_NUM);	
	}
	else
	{
		mt_eint_mask(CUST_EINT_GSE_1_NUM);	
		LIS3DSH_pedometer_disable(priv);
	}
	
	printk("Kaka lis3dsh_step_c_enable_nodata end OK!\n");
	return err;
}

static int lis3dsh_step_c_enable_step_detect(int en)
{
	return lis3dsh_step_c_enable_nodata(en);
}

static int lis3dsh_step_c_sigmotion(int en)
{
	return 0;
}

static int lis3dsh_step_c_set_delay(u64 delay)
{
	return 0;
}

static int lis3dsh_step_c_get_data(u64 *value, int *status)
{
	int err = 0;
	u16 pedo_data = 0;
	
	struct lis3dsh_i2c_data *priv = obj_i2c_data;
	lis3dsh_acc_get_pedometer_step(priv, &pedo_data);
	*value = (u64)pedo_data;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	
	return err;
}

static int lis3dsh_step_c_get_data_step_d(u64 *value, int *status)
{
	return 0;
}

static int lis3dsh_step_c_get_data_significant(u64 *value, int *status)
{
	return 0;
}

static void lis3dsh_eint_func(void)
{
	struct lis3dsh_i2c_data *priv = obj_i2c_data;
	GSE_FUN();
	
	if(!priv)
		return;

	mt_eint_mask(CUST_EINT_GSE_1_NUM);
	GSE_ERR("%s SCHEDULE WORK\n", __FUNCTION__);
       if(&priv->eint_work == NULL)
		GSE_ERR("%s NULL WORK\n", __FUNCTION__);
	   
	schedule_work(&priv->eint_work);
}

static int lis3dsh_setup_eint(void)
{
#ifdef CUST_EINT_GSE_1_TYPE
	mt_set_gpio_dir(GPIO_GSE_1_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_GSE_1_EINT_PIN, GPIO_GSE_1_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_GSE_1_EINT_PIN, true);
	mt_set_gpio_pull_select(GPIO_GSE_1_EINT_PIN, GPIO_PULL_DOWN);

	mt_eint_set_hw_debounce(CUST_EINT_GSE_1_NUM, CUST_EINT_GSE_1_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_GSE_1_NUM, EINTF_TRIGGER_RISING, lis3dsh_eint_func, 0);

	mt_eint_mask(CUST_EINT_GSE_1_NUM);
#endif
	return 0;
}

static void lis3dsh_eint_work(struct work_struct *work)
{
	struct lis3dsh_i2c_data *priv = obj_i2c_data;
	u8 databuf;
	u8 addr = 0x18;
	int res = 0;
	u16 pedo_data = 0;
	u16 pedo_write =0;
	GSE_FUN();
	
	if((lis_i2c_read_block(priv->client, addr, &databuf, 0x01))<0)
	{
		GSE_ERR("read OUTS1 register err!\n");
		return LIS3DSH_ERR_I2C;
	}
	
	if(databuf & 0x80){
		// handle pedometer overrun 		
		GSE_ERR("pedometer over run err!\n");
		lis3dsh_start_pedometer(priv,0x7FFF);
		if (STM_WALKING_MODE ==TRUE)
			lis3dsh_StateMachine_init(priv);
		else
			lis3dsh_StateMachine_2_init(priv);
	}
	else {
		if (STM_WALKING_MODE == TRUE)
		{
			GSE_ERR("START CONFIG RUNNING MODE!\n");
			lis3dsh_StateMachine_2_init(priv);

			lis3dsh_acc_get_pedometer_step(priv, &pedo_data);
			pedo_data = pedo_data +1;
			pedo_write = PEDOMETERMAX -pedo_data;
			lis3dsh_start_pedometer(priv,pedo_write);

			STM_WALKING_MODE = FALSE;
		}
		else{
			GSE_ERR("SCONFIG WALKING MODE!\n");
			lis3dsh_StateMachine_init(priv);
			STM_WALKING_MODE = TRUE;
		}
		//dumpReg(priv->client);
	}
	
	GSE_ERR("CLEAR int 1, read OUTS1 register 0x%x\n",databuf);
lis3dsh_eint_work_exit:
	mt_eint_unmask(CUST_EINT_GSE_1_NUM);
}

static int lis3dsh_step_c_local_init(void)
{
	int res = 0;
	struct step_c_control_path step_ctl={0};
	struct step_c_data_path step_data={0};	
	struct acc_hw *hw = lis3dsh_get_cust_acc_hw();	
	printk("lis3dsh_step_c_local_init \n");

	if (-1 == lis3dsh_init_flag)
	{
		GSE_ERR("lis3dsh_local_init   lis3dsh_init_flag=-1\n");
	   	return -1;
	}

	if (1==lis3dsh_init_flag )
	{
		mutex_lock(&lis3dsh_init_mutex);		
		LIS3DSH_power(hw, 1);
		if(i2c_add_driver(&lis3dsh_i2c_driver))
		{
			LIS3DSH_power(hw, 0);
			GSE_ERR("add driver error\n");
			mutex_unlock(&lis3dsh_init_mutex);
			return -1;
		}
	}
	
	if (-1 == lis3dsh_init_flag)
	{
		GSE_ERR("lis3dsh_local_init   lis3dsh_init_flag2=-1\n");
	   	return -1;
	}
	
       lis3dsh_setup_eint();

	step_ctl.open_report_data= lis3dsh_step_c_open_report_data;
	step_ctl.enable_nodata = lis3dsh_step_c_enable_nodata;
	step_ctl.enable_step_detect  = lis3dsh_step_c_enable_step_detect;
	step_ctl.set_delay = lis3dsh_step_c_set_delay;
	step_ctl.is_report_input_direct = false;
	step_ctl.is_support_batch = false;		
       step_ctl.enable_significant = lis3dsh_step_c_sigmotion; 
	res = step_c_register_control_path(&step_ctl);
	if(res)
	{
		 GSE_ERR("Kaka register step counter control path err\n");
		goto lis3dsh_step_c_local_init_failed;
	}
	
	step_data.get_data = lis3dsh_step_c_get_data;
	step_data.get_data_step_d = lis3dsh_step_c_get_data_step_d;
	step_data.get_data_significant = lis3dsh_step_c_get_data_significant;
	step_data.vender_div = 1;
	res = step_c_register_data_path(&step_data);
	if(res)
	{
		GSE_ERR("register step counter data path err= %d\n", res);
		goto lis3dsh_step_c_local_init_failed;
	}
	mutex_unlock(&lis3dsh_init_mutex);
	return 0;
	
lis3dsh_step_c_local_init_failed:
	mutex_unlock(&lis3dsh_init_mutex);
	GSE_ERR("%s init failed!\n", __FUNCTION__);
	return res;
}

static int lis3dsh_step_c_local_uninit(void)
{
	return 0;
}

#endif

static int lis3dsh_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct lis3dsh_i2c_data *obj;
	struct acc_control_path ctl={0};
	struct acc_data_path data={0};
	int err = 0;
	int retry = 0;
	
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(struct lis3dsh_i2c_data));
	obj->hw = hw;
	
	if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
#ifdef CONFIG_LIS3DSH_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);
	
	if(atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);
#endif

	lis3dsh_i2c_client = new_client;	

	for(retry = 0; retry < 3; retry++)
	{
		if((err = LIS3DSH_Init(new_client, 1))==0)			
			break;
		GSE_ERR("lis3dsh_device init cilent fail time: %d\n", retry);
	}
	
	if(err != 0)
		goto exit_init_failed;

#ifdef STEP_TEST	
	INIT_WORK(&obj->eint_work, lis3dsh_eint_work);
	lis3dsh_start_pedometer(obj_i2c_data, 0x7FFF);
#endif	

	if((err = misc_register(&lis3dsh_device)))
	{
		GSE_ERR("lis3dsh_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	
	if((err = lis3dsh_create_attr(&(lis3dsh_init_info.platform_diver_addr->driver))))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data = lis3dsh_open_report_data;
	ctl.enable_nodata = lis3dsh_enable_nodata;
	ctl.set_delay = lis3dsh_set_delay;
	ctl.is_report_input_direct = false;
	err = acc_register_control_path(&ctl);
	if(err)
	{
	 	GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = lis3dsh_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
	 	GSE_ERR("register acc data path err\n");
		goto exit_kfree;
	}

	GSE_LOG("%s: OK\n", __func__);
	lis3dsh_init_flag = 0;    
	return 0;

exit_create_attr_failed:
	misc_deregister(&lis3dsh_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	lis3dsh_init_flag = -1;        
	return err;
}

static int lis3dsh_i2c_remove(struct i2c_client *client)
{
	int err = 0;	

	err = lis3dsh_delete_attr(&(lis3dsh_init_info.platform_diver_addr->driver));
	if(err)
		GSE_ERR("lis3dsh_delete_attr fail: %d\n", err);
	
	err = misc_deregister(&lis3dsh_device);
	if(err)
		GSE_ERR("misc_deregister fail: %d\n", err);

	lis3dsh_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor_lis3dsh"},
	{},
};
#endif

static const struct i2c_device_id lis3dsh_i2c_id[] = 
{
	{LIS3DSH_DEV_NAME,0},
	{}
};

static struct i2c_driver lis3dsh_i2c_driver = {
	.driver = {
		.name = LIS3DSH_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = accel_of_match,
#endif		
	},
	.probe	= lis3dsh_i2c_probe,
	.remove	= lis3dsh_i2c_remove,
	.suspend	= lis3dsh_suspend,
	.resume	= lis3dsh_resume,
	.id_table	= lis3dsh_i2c_id,
};


static int  lis3dsh_local_init(void)
{
	if(i2c_add_driver(&lis3dsh_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	
	if(-1 == lis3dsh_init_flag)
	   return -1;
	
	return 0;
}

static int  lis3dsh_remove(void)
{
	GSE_FUN();    
	i2c_del_driver(&lis3dsh_i2c_driver);
	return 0;
}

static int __init lis3dsh_init(void)
{
	const char *name = "mediatek,lis3dsh";

	hw = get_accel_dts_func(name, hw);
	if (!hw)
		GSE_ERR("get dts info fail\n");

	acc_driver_add(&lis3dsh_init_info);

#ifdef STEP_TEST
	step_c_driver_add(&lis3dsh_step_c_init_info);
#endif

	return 0;    
}

static void __exit lis3dsh_exit(void)
{
	GSE_FUN();
}

module_init(lis3dsh_init);
module_exit(lis3dsh_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LIS3DSH I2C driver");
MODULE_AUTHOR("Chunlei.Wang@mediatek.com");
