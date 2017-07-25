/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *	
 * MediaTek Inc. (C) 2012. All rights reserved. 
 * 
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 *
 * Version: V2.5
 * Release Date: 2015/01/21
 */
 #include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include "tpd.h"
#include "goodix_driver.h"

#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif
#if GTP_SUPPORT_I2C_DMA
#include <linux/dma-mapping.h>
#endif
#ifdef CONFIG_MTK_BOOT
#include "mt_boot_common.h"
#endif

#if GTP_GESTURE_WAKEUP
typedef enum
{
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct i2c_client *client);
#endif

#if GTP_CHARGER_SWITCH
#ifdef MT6573
#define CHR_CON0	  (0xF7000000+0x2FA00)
#else
extern kal_bool upmu_is_chr_det(void);
#endif
static void gtp_charger_switch(s32 dir_update);
#endif 

#if GTP_HAVE_TOUCH_KEY
const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM ( sizeof( touch_key_array )/sizeof( touch_key_array[0] ) )
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]	= TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
//static int tpd_calmat_local[8]	 = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#if GTP_SUPPORT_I2C_DMA
s32 i2c_dma_write(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len);
s32 i2c_dma_read(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len);

static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
#endif

s32 gtp_send_cfg(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
static int touch_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
s32 gtp_read_version(struct i2c_client *client, u16 *version);//zhangjian add
extern struct tpd_device *tpd;
extern u8 gtp_loading_fw;

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client *);
extern void uninit_wr_node(void);
#endif
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_rw_mutex);

static int tpd_flag = 0; 
int tpd_halt = 0;
static int g_tpd_debug_point = 0;

static struct task_struct *thread = NULL;
struct i2c_client *i2c_client_point = NULL;

#define GTP_DRIVER_NAME  "goodix_touch"
static const struct i2c_device_id goodix_i2c_id[] = { {GTP_DRIVER_NAME, 0}, {} };
//static unsigned short force[] = { 0, GTP_I2C_ADDRESS, I2C_CLIENT_END, I2C_CLIENT_END };
//static const unsigned short *const forces[] = { force, NULL };

static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
};
static struct i2c_driver tpd_i2c_driver = {
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.detect = tpd_i2c_detect,
	.driver.name = GTP_DRIVER_NAME,
	.driver = {
		   .name = GTP_DRIVER_NAME,
		   .of_match_table = tpd_of_match,
		   },
	.id_table = goodix_i2c_id,
	//.address_list = (const unsigned short *)forces,
};

static u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
	= {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
#if GTP_CHARGER_SWITCH
static u8 gtp_charger_config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
	= {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
#endif

#pragma pack(1)
typedef struct
{
	u16 pid;				 //product id	//
	u16 vid;				 //version id	//
} st_tpd_info;
#pragma pack()

st_tpd_info tpd_info;
u8 goodix_int_type = 0;
u32 abs_x_max = 0;
u32 abs_y_max = 0;
u8 gtp_rawdiff_mode = 0;
u8 cfg_len = 0;
u8 pnl_init_error = 0;
//zhangjian add for gesture
#if GTP_GESTURE_WAKEUP
static struct kobject *mtk_gt9xx_tp_wake_gesture_kobj;
static ssize_t gt9xx_wake_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
       struct goodix_ts_data *goodix_data = i2c_get_clientdata(i2c_client_point);
	 
	return snprintf(buf, PAGE_SIZE, "%u,\n", goodix_data->enable_wakeup_gesture);
}

static ssize_t gt9xx_wake_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;
    struct goodix_ts_data *goodix_data = i2c_get_clientdata(i2c_client_point);
    if (sscanf(buf, "%u", &input) != 1)
    return -EINVAL;
    
    input = input > 0 ? 1 : 0;
    printk(KERN_INFO "wake_gesture input is %d \n",input);
    goodix_data->enable_wakeup_gesture = input;
    return count;
}
static DEVICE_ATTR(wake_gesture,  S_IRUGO|S_IWUSR|S_IWGRP,  gt9xx_wake_gesture_show,  gt9xx_wake_gesture_store); 
/* add your attr in here*/
static struct attribute *gt9xx_wake_gesture_attributes[] = {
	&dev_attr_wake_gesture.attr,	
	NULL
};

static struct attribute_group gt9xx_wake_gesture_attribute_group = {
	.attrs = gt9xx_wake_gesture_attributes
};


int gt9xx_create_wake_gesture_sysfs(struct i2c_client * client)
{
    int err;
    mtk_gt9xx_tp_wake_gesture_kobj = kobject_create_and_add("tp_wake_gesture", NULL);
    if (!mtk_gt9xx_tp_wake_gesture_kobj)
    {
        err = -EINVAL;
        dev_err(&client->dev, "%s() - ERROR Unable to create shtsc_wake_gesture_kobj\n",__func__);
    }
    err = sysfs_create_group(mtk_gt9xx_tp_wake_gesture_kobj, &gt9xx_wake_gesture_attribute_group);
    if (0 != err) 
    {
        dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed.\n", __func__);
        sysfs_remove_group(&client->dev.kobj, &gt9xx_wake_gesture_attribute_group);
        kobject_put(mtk_gt9xx_tp_wake_gesture_kobj);
        return -EIO;
    } 
    else 
    {
        pr_info("GTP:%s() - sysfs_create_group() succeeded.\n",__func__);
    }
    return err;
}

int gt9xx_remove_wake_gesture_sysfs(struct i2c_client * client)
{
	sysfs_remove_group(&client->dev.kobj, &gt9xx_wake_gesture_attribute_group);
	kobject_put(mtk_gt9xx_tp_wake_gesture_kobj);
	return 0;
}
#endif
static int gtp_read_cfg_version(struct i2c_client *client, u16* version)
{
	int ret = 0;
	u8 buf[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

	GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	if (version) {
		*version = buf[2];
	}
	GTP_INFO("cfg version: 0x%x", *version);

	return ret;
}
static ssize_t gt9xx_tpd_read(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    int len = 0;
    uint8_t buffer_tpd[800];
    struct goodix_ts_data *ts_data = i2c_get_clientdata(i2c_client_point);
    u16 version_info = 0x00;
    u16 version_cfg_info = 0x00;
    gtp_read_version(ts_data->i2c_client, &version_info);
    gtp_read_cfg_version(ts_data->i2c_client, &version_cfg_info);
    ts_data->fw_version = version_info;
    ts_data->config_version = version_cfg_info;
    len += sprintf(buffer_tpd + len, "Goodix Touchscreen Yashi module.\n");
    len += sprintf(buffer_tpd+len, "I2C address: 0x%x\n", 0x5d); 
    len += sprintf(buffer_tpd + len, "chip name:Goodix, chip id:%d \n", ts_data->chip_part_id);
    len += sprintf(buffer_tpd + len, "Goodix  firmware version:0x%x\n", ts_data->fw_version);
    len += sprintf(buffer_tpd + len, "Goodix  cfg version:0x%x\n", ts_data->config_version);
    len += sprintf(buffer_tpd + len, "Goodix  sensor id:0x%x\n", ts_data->sensor_id);
    return simple_read_from_buffer(page, size, ppos, buffer_tpd, len);	
}

static const struct file_operations gt9xx_tpd_id = {
        .owner = THIS_MODULE,
        .read = gt9xx_tpd_read,
        .write = NULL,
    };
int  gt9xx_create_proc(void)	
{
    struct proc_dir_entry *mt_entry = NULL;
    mt_entry = proc_create("driver/ts_information", 0644, NULL, &gt9xx_tpd_id);
    if (mt_entry)
    {
        printk("create tsc_id success\n");
        return 1;
    }
    else
    {
        printk("create tsc_id fail\n");
        return 0;
    }
}
//zhangjian add end
/* proc file system */
s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t, loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *, size_t, loff_t *);

static struct proc_dir_entry *gt91xx_config_proc = NULL;
static const struct file_operations config_proc_ops = {
	.owner = THIS_MODULE,
	.read = gt91xx_config_read_proc,
	.write = gt91xx_config_write_proc,
};

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, "mtk-tpd");
	return 0;
}

static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
	char *ptr = page;
	char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = {0};
	int i;
	
	if (*ppos)	// CMD call again
	{
		return 0;
	}
	
	ptr += sprintf(ptr, "==== GT9XX config init value====\n");

	for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
	{
		ptr += sprintf(ptr, "0x%02X ", config[i + 2]);

		if (i % 8 == 7)
			ptr += sprintf(ptr, "\n");
	}

	ptr += sprintf(ptr, "\n");

	ptr += sprintf(ptr, "==== GT9XX config real value====\n");
	i2c_read_bytes(i2c_client_point, GTP_REG_CONFIG_DATA, temp_data, GTP_CONFIG_MAX_LENGTH);

	for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
	{
		ptr += sprintf(ptr, "0x%02X ", temp_data[i]);

		if (i % 8 == 7)
			ptr += sprintf(ptr, "\n");
	}
	*ppos += ptr - page;
	return (ptr - page);
}

static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
	s32 ret = 0;

	//GTP_DEBUG("write count %ld\n", count);

	if (count > GTP_CONFIG_MAX_LENGTH)
	{
		//GTP_ERROR("size not match [%d:%ld]\n", GTP_CONFIG_MAX_LENGTH, count);
		return -EFAULT;
	}

	if (copy_from_user(&config[2], buffer, count))
	{
		GTP_ERROR("copy from user fail\n");
		return -EFAULT;
	}

	ret = gtp_send_cfg(i2c_client_point);
	abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
	abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
	goodix_int_type = (config[TRIGGER_LOC]) & 0x03;

	if (ret < 0)
	{
		GTP_ERROR("send config failed.");
	}

	return count;
}

#if GTP_SUPPORT_I2C_DMA
s32 i2c_dma_read(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len)
{
	int ret;
	s32 retry = 0;
	u8 buffer[2];

	struct i2c_msg msg[2] =
	{
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = buffer,
			.len = 2,
			.timing = I2C_MASTER_CLOCK
		},
		{
			.addr = (client->addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = (u8*)gpDMABuf_pa,	 
			.len = len,
			.timing = I2C_MASTER_CLOCK
		},
	};
	
	buffer[0] = (addr >> 8) & 0xFF;
	buffer[1] = addr & 0xFF;

	if (rxbuf == NULL)
		return -1;

	//GTP_DEBUG("dma i2c read: 0x%04X, %d bytes(s)", addr, len);
	for (retry = 0; retry < I2C_RETRY_TIMES; ++retry) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0) {
			continue;
		}
		memcpy(rxbuf, gpDMABuf_va, len);
		return 0;
	}
	GTP_ERROR("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
	return ret;
}


s32 i2c_dma_write(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len)
{
	int ret;
	s32 retry = 0;
	u8 *wr_buf = gpDMABuf_va;
	
	struct i2c_msg msg =
	{
		.addr = (client->addr & I2C_MASK_FLAG),
		.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.flags = 0,
		.buf = (u8*)gpDMABuf_pa,
		.len = 2 + len,
		.timing = I2C_MASTER_CLOCK
	};
	
	wr_buf[0] = (u8)((addr >> 8) & 0xFF);
	wr_buf[1] = (u8)(addr & 0xFF);

	if (txbuf == NULL)
		return -1;
	
	//GTP_DEBUG("dma i2c write: 0x%04X, %d bytes(s)", addr, len);
	memcpy(wr_buf+2, txbuf, len);
	for (retry = 0; retry < I2C_RETRY_TIMES; ++retry) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			continue;
		}
		return 0;
	}
	GTP_ERROR("Dma I2C Write Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
	return ret;
}

s32 i2c_read_bytes_dma(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len)
{
	s32 left = len;
	s32 read_len = 0;
	u8 *rd_buf = rxbuf;
	s32 ret = 0;

	mutex_lock(&i2c_rw_mutex);
	//GTP_DEBUG("Read bytes dma: 0x%04X, %d byte(s)", addr, len);
	while (left > 0) {
		if (left > GTP_DMA_MAX_TRANSACTION_LENGTH) {
			read_len = GTP_DMA_MAX_TRANSACTION_LENGTH;
		} else {
			read_len = left;
		}
		ret = i2c_dma_read(client, addr, rd_buf, read_len);
		if (ret < 0) {
			GTP_ERROR("dma read failed");
			ret = -1;
			goto out;
		}
		
		left -= read_len;
		addr += read_len;
		rd_buf += read_len;
	}
	ret = 0;
out:
	mutex_unlock(&i2c_rw_mutex);
	return ret;
}

s32 i2c_write_bytes_dma(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len)
{

	s32 ret = 0;
	s32 write_len = 0;
	s32 left = len;
	u8 *wr_buf = txbuf;

	mutex_lock(&i2c_rw_mutex);
	//GTP_DEBUG("Write bytes dma: 0x%04X, %d byte(s)", addr, len);
	while (left > 0) {
		if (left > GTP_DMA_MAX_I2C_TRANSFER_SIZE)
		{
			write_len = GTP_DMA_MAX_I2C_TRANSFER_SIZE;
		} else {
			write_len = left;
		}
		ret = i2c_dma_write(client, addr, wr_buf, write_len);
		if (ret < 0) {
			GTP_ERROR("dma i2c write failed!");
			ret = -1;
			goto out;
		}
		
		left -= write_len;
		addr += write_len;
		wr_buf += write_len;
	}
	ret = 0;
out:
	mutex_unlock(&i2c_rw_mutex);
	return ret;
}
#endif


int i2c_read_bytes_non_dma(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
	u8 buffer[GTP_ADDR_LENGTH];
	u8 retry;
	u16 left = len;
	u16 offset = 0;
	int retval = -1;
	struct i2c_msg msg[2] =
	{
		{
			.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
			//.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
			.flags = 0,
			.buf = buffer,
			.len = GTP_ADDR_LENGTH,
			.timing = I2C_MASTER_CLOCK
		},
		{
			.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
			//.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
			.flags = I2C_M_RD,
			.timing = I2C_MASTER_CLOCK
		},
	};

	mutex_lock(&i2c_rw_mutex);

	if (rxbuf == NULL) {
		retval = -1;
		goto out;
	}

	//GTP_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

	while (left > 0) {
		buffer[0] = ((addr + offset) >> 8) & 0xFF;
		buffer[1] = (addr + offset) & 0xFF;

		msg[1].buf = &rxbuf[offset];

		if (left > MAX_TRANSACTION_LENGTH) {
			msg[1].len = MAX_TRANSACTION_LENGTH;
			left -= MAX_TRANSACTION_LENGTH;
			offset += MAX_TRANSACTION_LENGTH;
		} else {
			msg[1].len = left;
			left = 0;
		}

		retry = 0;
		while (i2c_transfer(client->adapter, &msg[0], 2) != 2) {
			retry++;
			//if (retry == 20)
			if (retry == I2C_RETRY_TIMES) {
				GTP_ERROR("I2C read 0x%X length=%d failed\n", addr + offset, len);
				retval = -1;
				goto out;
			}
		}
	}
	retval = 0;
out:
	mutex_unlock(&i2c_rw_mutex);
	return retval;
}

int i2c_write_bytes_non_dma(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
	u8 buffer[MAX_TRANSACTION_LENGTH];
	u16 left = len;
	u16 offset = 0;
	u8 retry = 0;
	int retval = -1;
	struct i2c_msg msg =
	{
		.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
		//.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
		.flags = 0,
		.buf = buffer,
		.timing = I2C_MASTER_CLOCK,
	};

	mutex_lock(&i2c_rw_mutex);
	
	if (txbuf == NULL){
		retval = -1;
		goto out;
	}
	//GTP_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

	while (left > 0)
	{
		retry = 0;

		buffer[0] = ((addr + offset) >> 8) & 0xFF;
		buffer[1] = (addr + offset) & 0xFF;

		if (left > MAX_I2C_TRANSFER_SIZE) {
			memcpy(&buffer[GTP_ADDR_LENGTH], &txbuf[offset], MAX_I2C_TRANSFER_SIZE);
			msg.len = MAX_TRANSACTION_LENGTH;
			left -= MAX_I2C_TRANSFER_SIZE;
			offset += MAX_I2C_TRANSFER_SIZE;
		} else {
			memcpy(&buffer[GTP_ADDR_LENGTH], &txbuf[offset], left);
			msg.len = left + GTP_ADDR_LENGTH;
			left = 0;
		}

		//GTP_DEBUG("byte left %d offset %d\n", left, offset);
		while (i2c_transfer(client->adapter, &msg, 1) != 1) {
			retry++;

			//if (retry == 20)
			if (retry == I2C_RETRY_TIMES) {
				GTP_ERROR("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
				retval = -1;
				goto out;
			}
		}
	}
	retval = 0;
out:
	mutex_unlock(&i2c_rw_mutex);
	return retval;
}

int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
#if GTP_SUPPORT_I2C_DMA
	return i2c_write_bytes_dma(client, addr, txbuf, len);
#else
	return i2c_write_bytes_non_dma(client, addr, txbuf, len);
#endif
}

int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
#if GTP_SUPPORT_I2C_DMA
	return i2c_read_bytes_dma(client, addr, rxbuf, len);
#else
	return i2c_read_bytes_non_dma(client, addr, rxbuf, len);
#endif
}

s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
	s32 ret = -1;
	u16 addr = (buf[0] << 8) + buf[1];

	ret = i2c_write_bytes(client, addr, &buf[2], len - 2);
	if (!ret) {
		return 1;
	} else {
#if GTP_GESTURE_WAKEUP
		if (DOZE_ENABLED == doze_status) {
			return ret;
		}
#endif
		{
			gtp_reset_guitar(client, 20);
		}
		return ret;
	}
}

s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
	s32 ret = -1;
	u16 addr = (buf[0] << 8) + buf[1];

	ret = i2c_read_bytes(client, addr, &buf[2], len - 2);
	if (!ret) {
		return 2;
	} else {
#if GTP_GESTURE_WAKEUP
		if (DOZE_ENABLED == doze_status) {
			return ret;
		}
#endif
		{
			gtp_reset_guitar(client, 20);
		}
		return ret;
	}
}

s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	u8 retry = 0;
	
	while (retry++ < 3)
	{
		memset(buf, 0xAA, 16);
		buf[0] = (u8)(addr >> 8);
		buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, buf, len + 2);
		
		memset(confirm_buf, 0xAB, 16);
		confirm_buf[0] = (u8)(addr >> 8);
		confirm_buf[1] = (u8)(addr & 0xFF);
		gtp_i2c_read(client, confirm_buf, len + 2);
		
		if (!memcmp(buf, confirm_buf, len+2))
		{
			memcpy(rxbuf, confirm_buf+2, len);
			return SUCCESS;
		}
	}	 
	GTP_ERROR("i2c read 0x%04X, %d bytes, double check failed!", addr, len);
	return FAIL;
}

/*static int tpd_get_tplcd_res(struct goodix_ts_data *goodix_data)
{ 
	int tp_resx = 0, tp_resy = 0;
	u8 buf[8] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
	int ret = -1;
	
	ret = gtp_i2c_read(goodix_data->i2c_client, buf, sizeof(buf));
	if(ret < 0) {
		GTP_ERROR("Read tpd res failed.");
		tp_resx = TPD_RES_X;
		tp_resy = TPD_RES_Y;
	} else {
		tp_resx = buf[4] << 8 | buf[3];
		tp_resy = buf[6] << 8 | buf[5];
	}
	printk("tpd %s tp (%d, %d) lcd(%d, %d).\n", __func__, tp_resx, tp_resy, TPD_RES_X, TPD_RES_Y);
	goodix_data->tp_resx = tp_resx;
	goodix_data->tp_resy = tp_resy;
	goodix_data->lcd_resx = TPD_RES_X;
	goodix_data->lcd_resy = TPD_RES_Y;

	return 0;
}*/
int gtp_disable_irq(void)
{
	struct goodix_ts_data *goodix_data = i2c_get_clientdata(i2c_client_point);
	disable_irq(goodix_data->touch_irq);
	return 0;
}
int gtp_enable_irq(void)
{
	struct goodix_ts_data *goodix_data = i2c_get_clientdata(i2c_client_point);
	enable_irq(goodix_data->touch_irq);
	return 0;
}

/*******************************************************
Function:
	Send config Function.

Input:
	client: i2c client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
	s32 ret = 1;

#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;

	if (pnl_init_error)
	{
		GTP_INFO("Error occurred in init_panel, no config send!");
		return 0;
	}
	
	GTP_INFO("Driver Send Config");
	for (retry = 0; retry < 5; retry++)
	{
		ret = gtp_i2c_write(client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);

		if (ret > 0)
		{
			break;
		}
	}
#endif
	return ret;
}
#if GTP_CHARGER_SWITCH
static int gtp_send_chr_cfg(struct i2c_client *client)
{
	s32 ret = 1;
#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;

	if (pnl_init_error) {
		GTP_INFO("Error occurred in init_panel, no config sent!");
		return 0;
	}
	
	GTP_INFO("Driver Send Config");
	for (retry = 0; retry < 5; retry++) {
		ret = gtp_i2c_write(client, gtp_charger_config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		if (ret > 0) {
			break;
		}
	}
#endif	
	return ret;
}
#endif
static u8 ascii2hex(u8 a)
{
	s8 value = 0;

	if(a >= '0' && a <= '9') {
		value = a - '0';
	} else if (a >= 'A' && a <= 'F') {
		value = a - 'A' + 0x0A;
	} else if (a >= 'a' && a <= 'f') {
		value = a - 'a' + 0x0A;
	} else {
		value = 0xff;
	}
	
	return value;
}
/*******************************************************
Function:
	Read goodix touchscreen version function.

Input:
	client: i2c client struct.
	version:address to store version info

Output:
	Executive outcomes.0---succeed.
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16 *version)
{
	s32 ret = -1;
	s32 i;
	u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};
	struct goodix_ts_data *data = i2c_get_clientdata(client);

	//GTP_DEBUG_FUNC();
	GTP_INFO("Enter  %s,%d",__func__,__LINE__);
	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	if (version) {
		*version = (buf[7] << 8) | buf[6];
	}
	data->fw_version = *version;

	tpd_info.vid = *version;
	tpd_info.pid = 0x00;

	for (i = 0; i < 4; i++) {
		if (buf[i + 2] < 0x30)break;

		tpd_info.pid |= ((buf[i + 2] - 0x30) << ((3 - i) * 4));
	}

	if (buf[5] == 0x00) {
		data->chip_part_id = ascii2hex(buf[2]) * 100 + ascii2hex(buf[3]) * 10 + ascii2hex(buf[4]);
		GTP_INFO("IC VERSION: %c%c%c_%02x%02x",
			 buf[2], buf[3], buf[4], buf[7], buf[6]);  
	} else {
		data->chip_part_id = ascii2hex(buf[2]) * 1000 + ascii2hex(buf[3]) * 100 + ascii2hex(buf[4]) * 10 + ascii2hex(buf[5]);
		GTP_INFO("IC VERSION:%c%c%c%c_%02x%02x",
			 buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
	}
	return ret;
}

/*******************************************************
Function:
	GTP initialize function.

Input:
	client: i2c client private struct.

Output:
	Executive outcomes.0---succeed.
*******************************************************/
static s32 gtp_init_panel(struct i2c_client *client)
{
	s32 ret = 0;

#if GTP_DRIVER_SEND_CFG
	s32 i;
	u8 check_sum = 0;
	u8 opr_buf[16];
	u8 sensor_id = 0;
	 u8 drv_cfg_version;
	 u8 flash_cfg_version;

	u8 cfg_info_group0[] = CTP_CFG_GROUP0;
	u8 cfg_info_group1[] = CTP_CFG_GROUP1;
	u8 cfg_info_group2[] = CTP_CFG_GROUP2;
	u8 cfg_info_group3[] = CTP_CFG_GROUP3;
	u8 cfg_info_group4[] = CTP_CFG_GROUP4;
	u8 cfg_info_group5[] = CTP_CFG_GROUP5;
	u8 *send_cfg_buf[] = {cfg_info_group0, cfg_info_group1, cfg_info_group2,
						cfg_info_group3, cfg_info_group4, cfg_info_group5};
	u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group0), 
						  CFG_GROUP_LEN(cfg_info_group1),
						  CFG_GROUP_LEN(cfg_info_group2),
						  CFG_GROUP_LEN(cfg_info_group3), 
						  CFG_GROUP_LEN(cfg_info_group4),
						  CFG_GROUP_LEN(cfg_info_group5)};

	struct goodix_ts_data *data = i2c_get_clientdata(client);
	
	GTP_INFO("Enter  %s,%d",__func__,__LINE__);
	GTP_INFO("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d", 
		cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
		cfg_info_len[4], cfg_info_len[5]);

	ret = gtp_i2c_read_dbl_check(client, GTP_REG_SENSOR_ID, &sensor_id, 1);
	if (SUCCESS == ret) {
		if (sensor_id >= 0x06) {
			GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
		}
	} else {
		GTP_ERROR("Failed to get sensor_id, No config sent!");
	}
	data->sensor_id = sensor_id;
	GTP_INFO("Sensor_ID: %d", sensor_id);

	if ((!cfg_info_len[1]) && (!cfg_info_len[2]) && 
		(!cfg_info_len[3]) && (!cfg_info_len[4]) && 
		(!cfg_info_len[5]))
	{
		sensor_id = 0; 
	} else {
		ret = gtp_i2c_read_dbl_check(client, GTP_REG_SENSOR_ID, &sensor_id, 1);
		if (SUCCESS == ret) {
			if (sensor_id >= 0x06) {
				GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
				pnl_init_error = 1;
				return -1;
			}
		} else {
			GTP_ERROR("Failed to get sensor_id, No config sent!");
			pnl_init_error = 1;
			return -1;
		}
		GTP_INFO("Sensor_ID: %d", sensor_id);
	}
	
	cfg_len = cfg_info_len[sensor_id];
	
	GTP_INFO("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id, cfg_len);
	
	if (cfg_len < GTP_CONFIG_MIN_LENGTH)
	{
		GTP_ERROR("CTP_CONFIG_GROUP%d is INVALID CONFIG GROUP! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id);
		pnl_init_error = 1;
		return -1;
	}
		{
		ret = gtp_i2c_read_dbl_check(client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);	  
		if (ret == SUCCESS)
		{
			GTP_DEBUG("CFG_CONFIG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id, 
						send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);
	
			flash_cfg_version = opr_buf[0];
			drv_cfg_version = send_cfg_buf[sensor_id][0];		// backup  config version
			
			if (flash_cfg_version < 90 && flash_cfg_version > drv_cfg_version) {
				send_cfg_buf[sensor_id][0] = 0x00;
			}
		}
		else
		{
			GTP_ERROR("Failed to get ic config version!No config sent!");
			return -1;
		}
	}  
	memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], cfg_len);

	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < cfg_len; i++)
	{
		check_sum += config[i];
	}
	config[cfg_len] = (~check_sum) + 1;  
#else // DRIVER NOT SEND CONFIG
	cfg_len = GTP_CONFIG_MAX_LENGTH;
	ret = gtp_i2c_read(client, config, cfg_len + GTP_ADDR_LENGTH);
	if (ret < 0)
	{
		GTP_ERROR("Read Config Failed, Using DEFAULT Resolution & INT Trigger!");
		abs_x_max = GTP_MAX_WIDTH;
		abs_y_max = GTP_MAX_HEIGHT;
		goodix_int_type = GTP_INT_TRIGGER;
	}
#endif // GTP_DRIVER_SEND_CFG

	GTP_DEBUG_FUNC();
	if ((abs_x_max == 0) && (abs_y_max == 0))
	{
		abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
		abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
		goodix_int_type = (config[TRIGGER_LOC]) & 0x03; 
	}
		{
#if GTP_DRIVER_SEND_CFG
		ret = gtp_send_cfg(client);
		if (ret < 0)
		{
			GTP_ERROR("Send config error.");
		}
	  {
		/* for resume to send config */
		if (flash_cfg_version < 90 && flash_cfg_version > drv_cfg_version) {		
			config[GTP_ADDR_LENGTH] = drv_cfg_version;
			check_sum = 0;
			for (i = GTP_ADDR_LENGTH; i < cfg_len; i++)
			{
				check_sum += config[i];
			}
			config[cfg_len] = (~check_sum) + 1;
		}
	  }
#endif
		GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
			abs_x_max,abs_y_max,goodix_int_type);
	}
	
	msleep(10);
	return 0;
}

static s8 gtp_i2c_test(struct i2c_client *client)
{

	u8 retry = 0;
	s8 ret = -1;
	u32 hw_info = 0;

	GTP_DEBUG_FUNC();

	while (retry < 2) {
		ret = i2c_read_bytes(client, GTP_REG_HW_INFO, (u8 *)&hw_info, sizeof(hw_info));
		if ((!ret) && (hw_info == 0x00900600)) {				//20121212
			return ret;
		}

		GTP_ERROR("GTP_REG_HW_INFO : %08X", hw_info);
		GTP_ERROR("GTP i2c test failed time %d.", retry);
		msleep(10);
		retry++;
	}

	return -1;
}



/*******************************************************
Function:
	Set INT pin  as input for FW sync.

Note:
  If the INT is high, It means there is pull up resistor attached on the INT pin.
  Pull low the INT pin manaully for FW sync.
*******************************************************/
void gtp_int_sync(s32 ms)
{
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(ms);
	GTP_GPIO_AS_INT(GTP_INT_PORT);
}

void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
	GTP_INFO("GTP RESET!,%d\n",(client->addr == 0x14));
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(ms);
	GTP_GPIO_OUTPUT(GTP_INT_PORT, client->addr == 0x14);

	msleep(2);
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

	msleep(6);						//must >= 6ms


	gtp_int_sync(50); 
}

static int tpd_power_on(struct i2c_client *client)
{
	int retval = 0;
	//int reset_count = 0;

	GTP_INFO("Enter %s, %d,client-address=0x%x\n", __FUNCTION__, __LINE__,client->addr);

//reset_proc:
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);	
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(10);

	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		TPD_DMESG("Failed to enable reg-vgp1: %d\n", retval);

	gtp_reset_guitar(client, 20);
	{
		retval = gtp_i2c_test(client);
		if (retval < 0) {
			GTP_ERROR("I2C communication ERROR!");
			//if (reset_count < TPD_MAX_RESET_COUNT) {
			//	reset_count++;
			//	goto reset_proc;
			//}
		}
	}
	 GTP_INFO("Exit %s, %d\n", __FUNCTION__, __LINE__);
	return retval;
}

static int tpd_i2c_remove(struct i2c_client *client)
{
#if GTP_CREATE_WR_NODE
	uninit_wr_node();
#endif
#if GTP_GESTURE_WAKEUP
      gt9xx_remove_wake_gesture_sysfs(i2c_client_point);//zhangjian add for gesture
#endif
#if GTP_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

	return 0;
}
#if (GTP_ESD_PROTECT || GTP_COMPATIBLE_MODE)
static void force_reset_guitar(void)
{
	s32 i = 0;
	s32 ret = 0;
	int retval = -1;

	GTP_INFO("force_reset_guitar");
	
	gtp_disable_irq();

	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);	
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	
	regulator_disable(tpd->reg);
	msleep(15);
	retval = regulator_enable(tpd->reg);
	if (retval != 0)
		GTP_INFO("Failed to enable reg-vgp1: %d\n", retval);

	for (i = 0; i < 5; i++) {
		//Reset Guitar
		gtp_reset_guitar(i2c_client_point, 20);
		msleep(50);
		//Send config
		ret = gtp_send_cfg(i2c_client_point);

		if (ret < 0)
		{
			continue;
		}
		break;
	}
	gtp_enable_irq();
	
	if (i >= 5) {
		GTP_ERROR("Failed to reset guitar.");
		return;
	}
	GTP_INFO("Esd recovery successful");
	return;
}
#endif


static void tpd_down(s32 x, s32 y, s32 size, s32 id)
{
	if ((!size) && (!id))
	{
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
	}
	else
	{
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, size);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
		/* track id Start 0 */
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	}

	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, id, 1);

#ifdef CONFIG_MTK_BOOT
	if (tpd_dts_data.use_tpd_button) {
		if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()) {	
			tpd_button(x, y, 1); 
		}
	}
#endif
}

static void tpd_up(s32 x, s32 y, s32 id)
{
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
	TPD_EM_PRINT(x, y, x, y, id, 0);

#ifdef CONFIG_MTK_BOOT
	if (tpd_dts_data.use_tpd_button) {
		if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()) {	
			tpd_button(x, y, 0); 
		}
	}
#endif
}

#if GTP_GESTURE_WAKEUP
static void tpd_blackwakeup_func(struct goodix_ts_data *data, u8 gesture)
{
	unsigned int code = KEY_RESERVED;

	//a,b,c,d,e,g,h,m,o,q,s,v,w,y,z,^
	if ((gesture == 'c') || (gesture == 'e') || (gesture == 'm') || (gesture == 'o') ||
		(gesture == 's') || (gesture == 'w') ) {
		
		if (gesture == 'c'){
			code = KEY_GESTURE_C;
		}
		if (gesture == 'e'){
			code = KEY_GESTURE_E;                    
		}  
		if (gesture == 'm'){
			code = KEY_GESTURE_M;
		}
		if (gesture == 'o'){
			code = KEY_GESTURE_O;
		}
		if (gesture == 's'){
			code = KEY_GESTURE_S;
		}
		if (gesture == 'w'){
			code = KEY_GESTURE_W;
		}
	}else if ( (gesture == 0xAA) || (gesture == 0xBB) ||
		(gesture == 0xAB) || (gesture == 0xBA) ) {
		
		char *direction[4] = {"Right", "Down", "Up", "Left"};
		u8 type = ((gesture & 0x0F) - 0x0A) + (((gesture >> 4) & 0x0F) - 0x0A) * 2;

		GTP_INFO("%s slide to light up the screen!", direction[type]);
		if (type == 0){
			code = KEY_GESTURE_RIGHT;
		}
		if (type == 1){
			code = KEY_GESTURE_DOWN;
		}
		if (type == 2){
			code = KEY_GESTURE_UP;
		}
		if (type == 3){
			code = KEY_GESTURE_LEFT;
		}
	} else if (0xCC == gesture) {
		code = KEY_GESTURE_DOUBLE_CLICK;
	}
	input_report_key(tpd->dev, code, 1);
	input_sync(tpd->dev);
	input_report_key(tpd->dev, code, 0);
	input_sync(tpd->dev);
}
#endif
/*static void to_lcd_point(int *x, int *y)
{
	struct goodix_ts_data *data = i2c_get_clientdata(i2c_client_point);
	if(0 == data->tp_resx || 0 == data->tp_resy) {
		return;
	}

	*x = (*x) * data->lcd_resx / data->tp_resx;
	*y = (*y) * data->lcd_resy / data->tp_resy;
}*/
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 4 };
	u8	end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
	u8	point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
	u8	touch_num = 0;
	u8	finger = 0;
	static u8 pre_touch = 0;
	static u8 pre_key = 0;
	u8	key_value = 0;
	u8 *coor_data = NULL;
	s32 input_x = 0;
	s32 input_y = 0;
	s32 input_w = 0;
	s32 id = 0;
	s32 i  = 0;
	s32 ret = -1;
	
#if GTP_GESTURE_WAKEUP
	struct goodix_ts_data *goodix_data = i2c_get_clientdata(i2c_client_point);
	u8 doze_buf[3] = {0x81, 0x4B};
#endif

	sched_setscheduler(current, SCHED_RR, &param);
	do
	{
		set_current_state(TASK_INTERRUPTIBLE);
		
		while (tpd_halt)
		{
#if GTP_GESTURE_WAKEUP
			if (DOZE_ENABLED == doze_status) {
				break;
			}
#endif
			tpd_flag = 0;
			msleep(20);
		}

		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);

#if GTP_CHARGER_SWITCH
		gtp_charger_switch(0);
#endif
		//GTP_INFO("Enter %s, %d\n", __FUNCTION__, __LINE__);
#if GTP_GESTURE_WAKEUP
		if (DOZE_ENABLED == doze_status)
		{
			ret = gtp_i2c_read(i2c_client_point, doze_buf, 3);
			GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
			if (ret > 0)
			{				
				if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b') || (doze_buf[2] == 'c') ||
					(doze_buf[2] == 'd') || (doze_buf[2] == 'e') || (doze_buf[2] == 'g') || 
					(doze_buf[2] == 'h') || (doze_buf[2] == 'm') || (doze_buf[2] == 'o') ||
					(doze_buf[2] == 'q') || (doze_buf[2] == 's') || (doze_buf[2] == 'v') || 
					(doze_buf[2] == 'w') || (doze_buf[2] == 'y') || (doze_buf[2] == 'z') ||
					(doze_buf[2] == 0x5E) /* ^ */
					)
				{
					if (doze_buf[2] != 0x5E) {
						printk("Wakeup by gesture(%c), light up the screen!", doze_buf[2]);
					} else {
						printk("Wakeup by gesture(^), light up the screen!");
					}
					doze_status = DOZE_WAKEUP;
					tpd_blackwakeup_func(goodix_data, doze_buf[2]);
					// clear 0x814B
					doze_buf[2] = 0x00;
					gtp_i2c_write(i2c_client_point, doze_buf, 3);
				} else if ( (doze_buf[2] == 0xAA) || (doze_buf[2] == 0xBB) ||
					(doze_buf[2] == 0xAB) || (doze_buf[2] == 0xBA) )
				{
					char *direction[4] = {"Right", "Down", "Up", "Left"};
					u8 type = ((doze_buf[2] & 0x0F) - 0x0A) + (((doze_buf[2] >> 4) & 0x0F) - 0x0A) * 2;
					
					printk("%s slide to light up the screen!", direction[type]);
					doze_status = DOZE_WAKEUP;
					tpd_blackwakeup_func(goodix_data, doze_buf[2]);
					// clear 0x814B
					doze_buf[2] = 0x00;
					gtp_i2c_write(i2c_client_point, doze_buf, 3);
				} else if (0xCC == doze_buf[2]) {
					printk("Double click to light up the screen!");
					doze_status = DOZE_WAKEUP;
					tpd_blackwakeup_func(goodix_data, doze_buf[2]);
					// clear 0x814B
					doze_buf[2] = 0x00;
					gtp_i2c_write(i2c_client_point, doze_buf, 3);
				} else {
					printk("tpd unkown blackscreen gesture.");
					// clear 0x814B
					doze_buf[2] = 0x00;
					gtp_i2c_write(i2c_client_point, doze_buf, 3);
					
				}
				gtp_enter_doze(i2c_client_point);
			}
			continue;
		}
#endif
		ret = gtp_i2c_read(i2c_client_point, point_data, 12);
		if (ret < 0) {
			GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
			continue;
		}
		finger = point_data[GTP_ADDR_LENGTH];
		if (finger == 0x00) 	{
			printk("tpd no finger\n");
			continue;
		}
		if ((finger & 0x80) == 0) {
			printk("tpd status error\n");
			goto exit_work_func;
		}
		
		touch_num = finger & 0x0f;
		if (touch_num > GTP_MAX_TOUCH) {
			goto exit_work_func;
		}
		if (touch_num > 1) {
			u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

			ret = gtp_i2c_read(i2c_client_point, buf, 2 + 8 * (touch_num - 1));
			memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
		}

#if (GTP_HAVE_TOUCH_KEY || GTP_PEN_HAVE_BUTTON)
		key_value = point_data[3 + 8 * touch_num];

		if (key_value || pre_key) {
#if GTP_PEN_HAVE_BUTTON
			if (key_value == 0x40) {
				GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Down.");
				input_report_key(pen_dev, BTN_STYLUS, 1);
				input_report_key(pen_dev, BTN_STYLUS2, 1);
				pen_active = 1;
			} else if (key_value == 0x10) {
				GTP_DEBUG("BTN_STYLUS Down, BTN_STYLUS2 Up.");
				input_report_key(pen_dev, BTN_STYLUS, 1);
				input_report_key(pen_dev, BTN_STYLUS2, 0);
				pen_active = 1;
			} else if (key_value == 0x20) {
				GTP_DEBUG("BTN_STYLUS Up, BTN_STYLUS2 Down.");
				input_report_key(pen_dev, BTN_STYLUS, 0);
				input_report_key(pen_dev, BTN_STYLUS2, 1);
				pen_active = 1;
			} else {
				GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Up.");
				input_report_key(pen_dev, BTN_STYLUS, 0);
				input_report_key(pen_dev, BTN_STYLUS2, 0);
				if ( (pre_key == 0x40) || (pre_key == 0x20) ||(pre_key == 0x10) ) {
					pen_active = 1;
				}
			}
			if (pen_active) {
				touch_num = 0;		// shield pen point
				//pre_touch = 0;	// clear last pen status
			}
#endif
#if GTP_HAVE_TOUCH_KEY
			if (!pre_touch) {
				printk("tpd gtp keyvalue = %d.\n", key_value);
				for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
					input_report_key(tpd->dev, touch_key_array[i], key_value & (0x01 << i));
				}
				touch_num = 0;	// shiled fingers
			}
#endif
		}
#endif
		pre_key = key_value;

		//GTP_DEBUG("pre_touch:%02x, finger:%02x.touch_num=0x%x\n", pre_touch, finger,touch_num);
		
		if (touch_num) {
			for (i = 0; i < touch_num; i++) {
				coor_data = &point_data[i * 8 + 3];

				id = coor_data[0] & 0x0F;	   
				input_x  = coor_data[1] | coor_data[2] << 8;
				input_y  = coor_data[3] | coor_data[4] << 8;
				input_w  = coor_data[5] | coor_data[6] << 8;

				input_x = TPD_WARP_X(abs_x_max, input_x);
				input_y = TPD_WARP_Y(abs_y_max, input_y);
				//to_lcd_point(&input_x, &input_y);

				//GTP_DEBUG(" (%d)(%d, %d)[%d]", id, input_x, input_y, input_w);
				tpd_down(input_x, input_y, input_w, id);

				if((g_tpd_debug_point - 1) % 100 == 0) {
					printk("tpd:%d%d %d,%d\n", id, input_w, input_x, input_y);
				}
			}
		} else {
			if (pre_touch)	{
				//GTP_DEBUG("Touch Release!");
				tpd_up(0, 0, 0);
			}
		}
		
		pre_touch = touch_num;
		input_sync(tpd->dev);

exit_work_func:

		if (!gtp_rawdiff_mode) {
			ret = gtp_i2c_write(i2c_client_point, end_cmd, 3);
			if (ret < 0) {
				GTP_INFO("I2C write end_cmd  error!");
			}
		}
	} while (!kthread_should_stop());

	return 0;
}

static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
	TPD_DEBUG_PRINT_INT;
	
	tpd_flag = 1;
	g_tpd_debug_point ++;

	wake_up_interruptible(&waiter);

	return IRQ_HANDLED;
}

static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };
	struct goodix_ts_data *goodix_data = i2c_get_clientdata(i2c_client_point);

	GTP_INFO("Device Tree Tpd_irq_registration!");

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		goodix_data->touch_irq = irq_of_parse_and_map(node, 0);
		GTP_INFO("Device goodix_int_type = %d!", goodix_int_type);
		if (!goodix_int_type) {/*EINTF_TRIGGER*/
			ret = request_irq(goodix_data->touch_irq, tpd_eint_interrupt_handler, IRQF_TRIGGER_RISING,
					"goodix-eint", NULL);
			if (ret > 0) {
				ret = -1;
				GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		} else {
			ret = request_irq(goodix_data->touch_irq, tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING,
					"goodix-eint", NULL);
			if (ret > 0) {
				ret = -1;
				GTP_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		}
	} else {
		GTP_ERROR("tpd request_irq can not find touch eint device node!.");
		ret = -1;
	}
	GTP_INFO("[%s]irq:%d, debounce:%d-%d:", __func__, goodix_data->touch_irq, ints[0], ints[1]);
	GTP_INFO("%s irq_get_trigger_type is %d .\n", __func__, irq_get_trigger_type(goodix_data->touch_irq));
	return ret;
}

static s32 tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 err = 0;
	s32 ret = 0;
	u16 version_info;
#if GTP_HAVE_TOUCH_KEY
	int idx = 0;
#endif
	struct goodix_ts_data *goodix_data;

	GTP_INFO("Enter %s, %d\n", __FUNCTION__, __LINE__);

	goodix_data = kzalloc(sizeof(*goodix_data), GFP_KERNEL);
	if (!goodix_data) {
		dev_err(&client->dev,
				"%s: Failed to alloc mem for goodix_data\n",
				__func__);
		return -ENOMEM;
	}
	goodix_data->i2c_client = client;
	i2c_set_clientdata(client, goodix_data);
	i2c_client_point = client;
	
#if GTP_SUPPORT_I2C_DMA
	tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
	if(!gpDMABuf_va){
		GTP_INFO("[Error] Allocate DMA I2C Buffer failed!\n");
	}
	memset(gpDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);
#endif

	//tpd_power_already_on = 1;
	ret = tpd_power_on(client);
	if (ret < 0) {
		GTP_ERROR("I2C communication ERROR!");
		goto error_i2c_failed;
	}
	
	ret = gtp_read_version(client, &version_info);
	if (ret < 0) {
		GTP_ERROR("Read version failed.");
	}	 
	
	ret = gtp_init_panel(client);
	if (ret < 0) {
		GTP_ERROR("GTP init panel failed.");
	}
	
	goodix_data->need_stay_awake = false;
	goodix_data->enable_wakeup_gesture = false;
	goodix_data->tpd_suspend = false;

	//tpd_get_tplcd_res(goodix_data);
	
#if GTP_HAVE_TOUCH_KEY
	for (idx = 0; idx < GTP_MAX_KEY_NUM; idx++) {
		input_set_capability(tpd->dev, EV_KEY, touch_key_array[idx]);
	}
#endif

#if GTP_GESTURE_WAKEUP
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_DOUBLE_CLICK); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_UP); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_LEFT); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_RIGHT); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_C); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_E); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_M); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_U); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_Z);
#endif

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		GTP_INFO(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}
	
	tpd_irq_registration();

	// Create proc file system
	gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0640, NULL, &config_proc_ops);
	if (gt91xx_config_proc == NULL) {
		GTP_ERROR("create_proc_entry %s failed\n", GT91XX_CONFIG_PROC_FILE);
	} else {
		GTP_INFO("create proc entry %s success", GT91XX_CONFIG_PROC_FILE);
	}
      //zhangjian add for gesture and TP info
      #if GTP_GESTURE_WAKEUP
      gt9xx_create_wake_gesture_sysfs(i2c_client_point);
      #endif
      gt9xx_create_proc();
      //zhangjian add end
#if GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif
#if GTP_AUTO_UPDATE
	ret = gup_init_update_proc(client);
	if (ret < 0) {
		GTP_ERROR("Create update thread error.");
	}
#endif

	tpd_load_status = 1;
	GTP_INFO("Exit %s, %d\n", __FUNCTION__, __LINE__);
	return 0;

error_i2c_failed:
	kfree(goodix_data);
	if(gpDMABuf_va){
		dma_free_coherent(&tpd->dev->dev, GTP_DMA_MAX_TRANSACTION_LENGTH,  gpDMABuf_va, gpDMABuf_pa);
		gpDMABuf_va = NULL;
		gpDMABuf_pa = 0;
		TPD_DMESG("[DMA][release] Allocate DMA I2C Buffer release!\n");
	}
	return -1;
}


static int tpd_local_init(void)
{
	int retval = -1;
	
	GTP_INFO("Enter %s, %d\n", __FUNCTION__, __LINE__);
	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
	retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
	if (retval != 0) {
		TPD_DMESG("Failed to set reg-vgp1 voltage: %d\n", retval);
		return -1;
	}
	
#if GTP_ESD_PROTECT
	clk_tick_cnt = 2 * HZ;	 // HZ: clock ticks in 1 second generated by system
	GTP_DEBUG("Clock ticks for an esd cycle: %d", clk_tick_cnt);
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
	spin_lock_init(&esd_lock);			// 2.6.39 & later
	// esd_lock = SPIN_LOCK_UNLOCKED;	// 2.6.39 & before
#endif

	if (i2c_add_driver(&tpd_i2c_driver) != 0) {
		GTP_INFO("unable to add i2c driver.\n");
		return -1;
	}

	if(tpd_load_status == 0) {
		TPD_DMESG("goodix add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
		tpd_dts_data.tpd_key_dim_local);
	}

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif

	// set vendor string
	tpd->dev->id.vendor = 0x00;
	tpd->dev->id.product = tpd_info.pid;
	tpd->dev->id.version = tpd_info.vid;

	GTP_INFO("Exit %s, %d\n", __FUNCTION__, __LINE__);
	
	tpd_type_cap = 1;

	return 0;
}

#if GTP_GESTURE_WAKEUP
static s8 gtp_enter_doze(struct i2c_client *client)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

	GTP_DEBUG_FUNC();

	GTP_DEBUG("Entering gesture mode...");
	while(retry++ < 5)
	{
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x46;
		ret = gtp_i2c_write(client, i2c_control_buf, 3);
		if (ret < 0)
		{
			GTP_DEBUG("Failed to set gesture flag into 0x8046, %d", retry);
			continue;
		}
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x40;
		ret = gtp_i2c_write(client, i2c_control_buf, 3);
		if (ret > 0)
		{
			doze_status = DOZE_ENABLED;
			GTP_INFO("Gesture mode enabled.");
			return ret;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send gesture cmd failed.");
	return ret;
}
#endif
/*******************************************************
Function:
	Eter sleep function.

Input:
	client:i2c_client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_enter_sleep(struct i2c_client *client)
{
#if GTP_COMPATIBLE_MODE
	if (CHIP_TYPE_GT9F == gtp_chip_type)
	{
		u8 i2c_status_buf[3] = {0x80, 0x44, 0x00};
		s32 ret = 0;
	  
		ret = gtp_i2c_read(client, i2c_status_buf, 3);
		if(ret <= 0)
		{
			 GTP_ERROR("[gtp_enter_sleep]Read ref status reg error.");
		}
		
		if (i2c_status_buf[2] & 0x80)
		{
			//Store bak ref
			ret = gtp_bak_ref_proc(client, GTP_BAK_REF_STORE);
			if(FAIL == ret)
			{
				GTP_ERROR("[gtp_enter_sleep]Store bak ref failed.");
			}		 
		}
	}
#endif
#if GTP_POWER_CTRL_SLEEP

	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);	
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(10);

	regulator_disable(tpd->reg);
   
	GTP_INFO("GTP enter sleep by poweroff!");
	return 0;
	
#else
	{
		s8 ret = -1;
		s8 retry = 0;
		u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};
		
		
		GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
		msleep(5);
	
		while (retry++ < 5) {
			ret = gtp_i2c_write(client, i2c_control_buf, 3);
			if (ret > 0) {
				GTP_INFO("GTP enter sleep!");
				return ret;
			}
	
			msleep(10);
		}
	
		GTP_ERROR("GTP send sleep cmd failed.");
		return ret;
	}
#endif
}

/*******************************************************
Function:
	Wakeup from sleep mode Function.

Input:
	client:i2c_client.

Output:
	Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_wakeup_sleep(struct i2c_client *client)
{
	u8 retry = 0;
	s8 ret = -1;
	
#if GTP_GESTURE_WAKEUP
	struct goodix_ts_data *goodix_data = i2c_get_clientdata(i2c_client_point);
#endif
	GTP_DEBUG("GTP wakeup begin.");

#if (GTP_POWER_CTRL_SLEEP)	 
	while (retry++ < 5)
	{
		ret = tpd_power_on(client);

		if (ret < 0)
		{
			GTP_ERROR("I2C Power on ERROR!");
			continue;
		}
		GTP_INFO("Ic wakeup by poweron");
		return 0;
	}
#else
	while (retry++ < 10)
	{
#if GTP_GESTURE_WAKEUP
		if(goodix_data->enable_wakeup_gesture) {
			if (DOZE_WAKEUP != doze_status) {
				GTP_INFO("Powerkey wakeup.");
			} else {
				GTP_INFO("Gesture wakeup.");
			}
			doze_status = DOZE_DISABLED;

			disable_irq(goodix_data->touch_irq);
			gtp_reset_guitar(client, 20);
			enable_irq(goodix_data->touch_irq);
		} else {
			GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
			msleep(30);
		}
#else
		GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
		msleep(30);
#endif
		ret = gtp_i2c_test(client);
		if (ret >= 0) {
			GTP_INFO("GTP wakeup sleep.");
#if GTP_GESTURE_WAKEUP
			if(!goodix_data->enable_wakeup_gesture) {
				gtp_int_sync(25);
			}
#if GTP_ESD_PROTECT
			gtp_init_ext_watchdog(client);
#endif
#else
			gtp_int_sync(25);
#endif
			return ret;
		}
		gtp_reset_guitar(client, 20);
	}
#endif
	GTP_ERROR("GTP wakeup sleep failed.");
	return ret;
}

/* Function to manage low power suspend */
static void tpd_suspend(struct device *h)
{
	s32 ret = -1;
	struct goodix_ts_data *goodix_data = i2c_get_clientdata(i2c_client_point);
	
	GTP_INFO("TPD goodix ts suspend.");
	if(goodix_data->need_stay_awake) {
		printk("%s need_stay_awake return.\n", __func__);
		return;
	}
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1) {
		return ;
	}
#endif

	tpd_halt = 1;
	goodix_data->tpd_suspend = true;
#if GTP_ESD_PROTECT
	gtp_esd_switch(i2c_client_point, SWITCH_OFF);
#endif
	
#if GTP_GESTURE_WAKEUP
	if(goodix_data->enable_wakeup_gesture) {
		ret = gtp_enter_doze(i2c_client_point);
	} else {
		disable_irq(goodix_data->touch_irq);
		ret = gtp_enter_sleep(i2c_client_point);
	}
#else
	disable_irq(goodix_data->touch_irq);
	ret = gtp_enter_sleep(i2c_client_point);
#endif
	if (ret < 0)
	{
		GTP_ERROR("GTP early suspend failed.");
	}
	// to avoid waking up while not sleeping, delay 48 + 10ms to ensure reliability 
	msleep(58);
}

/* Function to manage power-on resume */
static void tpd_resume(struct device *h)
{
	s32 ret = -1;
	struct goodix_ts_data *goodix_data = i2c_get_clientdata(i2c_client_point);

	GTP_INFO("TPD goodix ts resume.");

	//ret = regulator_enable(tpd->reg);
	//if (ret != 0)
	//	TPD_DMESG("Failed to enable reg-vgp1: %d\n", ret);
	
	if(goodix_data->need_stay_awake) {
		printk("%s need_stay_awake return.\n", __func__);
		goodix_data->tpd_suspend = false;
		return;
	}
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1) {
		return ;
	}
#endif
	ret = gtp_wakeup_sleep(i2c_client_point);
	if (ret < 0) {
		GTP_ERROR("GTP later resume failed.");
	}
	{
		gtp_send_cfg(i2c_client_point);
	}
	
#if GTP_CHARGER_SWITCH
	gtp_charger_switch(1);	// force update
#endif

	tpd_halt = 0;
	goodix_data->tpd_suspend = false;
#if GTP_GESTURE_WAKEUP
	doze_status = DOZE_DISABLED;
	if(!goodix_data->enable_wakeup_gesture) {
		enable_irq(goodix_data->touch_irq);
	}
#else 
	enable_irq(goodix_data->touch_irq);
#endif

}

static struct tpd_driver_t tpd_device_driver =
{
	.tpd_device_name = "goodix",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	GTP_INFO("MediaTek gt91xx touch panel driver init\n");

	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		GTP_INFO("add goodix driver failed\n");

	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	GTP_INFO("MediaTek gt91xx touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

