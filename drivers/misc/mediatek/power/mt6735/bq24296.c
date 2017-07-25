#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <mt-plat/charging.h>
#include "bq24296.h"
#include <linux/proc_fs.h>//zhangjian add
#include <asm/uaccess.h> //zhangjian add
#include <linux/delay.h>	//doumm add

/**********************************************************
  *
  *   [I2C Slave Setting]
  *
  *********************************************************/
#define bq24296_SLAVE_ADDR_WRITE   0xD6
#define bq24296_SLAVE_ADDR_READ    0xD7

static struct i2c_client *new_client;
static const struct i2c_device_id bq24296_i2c_id[] = { {"bq24296", 0}, {} };

kal_bool chargin_hw_init_done = KAL_FALSE;
static int bq24296_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

#ifdef CONFIG_OF
static const struct of_device_id bq24296_of_match[] = {
	{.compatible = "mediatek,ext_buck",},
	{.compatible = "mediatek,swithing_charger",},
	{},
};

MODULE_DEVICE_TABLE(of, bq24296_of_match);
#endif

static struct i2c_driver bq24296_driver = {
	.driver = {
		   .name = "bq24296",
#ifdef CONFIG_OF
		   .of_match_table = bq24296_of_match,
#endif
		   },
	.probe = bq24296_driver_probe,
	.id_table = bq24296_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable]
  *
  *********************************************************/
unsigned char bq24296_reg[bq24296_REG_NUM] = { 0 };

static DEFINE_MUTEX(bq24296_i2c_access);

int g_bq24296_hw_exist = 0;

/**********************************************************
  *
  *   [I2C Function For Read/Write bq24296]
  *
  *********************************************************/
int bq24296_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[1] = { 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&bq24296_i2c_access);

	/* new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG; */
	new_client->ext_flag =
	    ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], (1 << 8 | 1));
	if (ret < 0) {
		/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
		new_client->ext_flag = 0;

		mutex_unlock(&bq24296_i2c_access);
		return 0;
	}

	readData = cmd_buf[0];
	*returnData = readData;

	/* new_client->addr = new_client->addr & I2C_MASK_FLAG; */
	new_client->ext_flag = 0;

	mutex_unlock(&bq24296_i2c_access);
	return 1;
}

int bq24296_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	mutex_lock(&bq24296_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	new_client->ext_flag = ((new_client->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		new_client->ext_flag = 0;
		mutex_unlock(&bq24296_i2c_access);
		return 0;
	}

	new_client->ext_flag = 0;
	mutex_unlock(&bq24296_i2c_access);
	return 1;
}

/**********************************************************
  *
  *   [Read / Write Function]
  *
  *********************************************************/
unsigned int bq24296_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT)
{
	unsigned char bq24296_reg = 0;
	int ret = 0;

	battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	ret = bq24296_read_byte(RegNum, &bq24296_reg);

	battery_log(BAT_LOG_FULL, "[bq24296_read_interface] Reg[%x]=0x%x\n", RegNum, bq24296_reg);

	bq24296_reg &= (MASK << SHIFT);
	*val = (bq24296_reg >> SHIFT);

	battery_log(BAT_LOG_FULL, "[bq24296_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int bq24296_config_interface(unsigned char RegNum, unsigned char val, unsigned char MASK,
				    unsigned char SHIFT)
{
	unsigned char bq24296_reg = 0;
	int ret = 0;

	battery_log(BAT_LOG_FULL, "--------------------------------------------------\n");

	ret = bq24296_read_byte(RegNum, &bq24296_reg);
	battery_log(BAT_LOG_FULL, "[bq24296_config_interface] Reg[%x]=0x%x\n", RegNum, bq24296_reg);

	bq24296_reg &= ~(MASK << SHIFT);
	bq24296_reg |= (val << SHIFT);

	ret = bq24296_write_byte(RegNum, bq24296_reg);
	battery_log(BAT_LOG_FULL, "[bq24296_config_interface] write Reg[%x]=0x%x\n", RegNum, bq24296_reg);

	/* Check */
	/* bq24296_read_byte(RegNum, &bq24296_reg); */
	/* battery_log(BAT_LOG_FULL, "[bq24296_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq24296_reg); */

	return ret;
}

/* write one register directly */
unsigned int bq24296_reg_config_interface(unsigned char RegNum, unsigned char val)
{
	unsigned int ret = 0;

	ret = bq24296_write_byte(RegNum, val);

	return ret;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
/* CON0---------------------------------------------------- */

void bq24296_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
	    );
}

void bq24296_set_vindpm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_VINDPM_MASK),
				       (unsigned char) (CON0_VINDPM_SHIFT)
	    );
}

void bq24296_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
	    );
}

/* CON1---------------------------------------------------- */

void bq24296_set_reg_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_REG_RST_MASK),
				       (unsigned char) (CON1_REG_RST_SHIFT)
	    );
}

void bq24296_set_wdt_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_WDT_RST_MASK),
				       (unsigned char) (CON1_WDT_RST_SHIFT)
	    );
}

void bq24296_set_otg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OTG_CONFIG_MASK),
				       (unsigned char) (CON1_OTG_CONFIG_SHIFT)
	    );
}

void bq24296_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CHG_CONFIG_MASK),
				       (unsigned char) (CON1_CHG_CONFIG_SHIFT)
	    );
}

void bq24296_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_SYS_MIN_MASK),
				       (unsigned char) (CON1_SYS_MIN_SHIFT)
	    );
}

void bq24296_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_BOOST_LIM_MASK),
				       (unsigned char) (CON1_BOOST_LIM_SHIFT)
	    );
}

/* CON2---------------------------------------------------- */

void bq24296_set_ichg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_ICHG_MASK), (unsigned char) (CON2_ICHG_SHIFT)
	    );
}

void bq24296_set_bcold(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_BCOLD_MASK), (unsigned char) (CON2_BCOLD_SHIFT)
	    );
}

void bq24296_set_force_20pct(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_FORCE_20PCT_MASK),
				       (unsigned char) (CON2_FORCE_20PCT_SHIFT)
	    );
}

/* CON3---------------------------------------------------- */

void bq24296_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_IPRECHG_MASK),
				       (unsigned char) (CON3_IPRECHG_SHIFT)
	    );
}

void bq24296_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_ITERM_MASK), (unsigned char) (CON3_ITERM_SHIFT)
	    );
}

/* CON4---------------------------------------------------- */

void bq24296_set_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VREG_MASK), (unsigned char) (CON4_VREG_SHIFT)
	    );
}

void bq24296_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_BATLOWV_MASK),
				       (unsigned char) (CON4_BATLOWV_SHIFT)
	    );
}

void bq24296_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VRECHG_MASK),
				       (unsigned char) (CON4_VRECHG_SHIFT)
	    );
}

/* CON5---------------------------------------------------- */

void bq24296_set_en_term(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TERM_MASK),
				       (unsigned char) (CON5_EN_TERM_SHIFT)
	    );
}

void bq24296_set_watchdog(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_WATCHDOG_MASK),
				       (unsigned char) (CON5_WATCHDOG_SHIFT)
	    );
}

void bq24296_set_en_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TIMER_MASK),
				       (unsigned char) (CON5_EN_TIMER_SHIFT)
	    );
}

void bq24296_set_chg_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_CHG_TIMER_MASK),
				       (unsigned char) (CON5_CHG_TIMER_SHIFT)
	    );
}

/* CON6---------------------------------------------------- */

void bq24296_set_treg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_TREG_MASK), (unsigned char) (CON6_TREG_SHIFT)
	    );
}

void bq24296_set_boostv(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BOOSTV_MASK),
				       (unsigned char) (CON6_BOOSTV_SHIFT)
	    );
}

void bq24296_set_bhot(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BHOT_MASK), (unsigned char) (CON6_BHOT_SHIFT)
	    );
}

/* CON7---------------------------------------------------- */

void bq24296_set_tmr2x_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_TMR2X_EN_MASK),
				       (unsigned char) (CON7_TMR2X_EN_SHIFT)
	    );
}

void bq24296_set_batfet_disable(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_Disable_MASK),
				       (unsigned char) (CON7_BATFET_Disable_SHIFT)
	    );
}

void bq24296_set_int_mask(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq24296_config_interface((unsigned char) (bq24296_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_INT_MASK_MASK),
				       (unsigned char) (CON7_INT_MASK_SHIFT)
	    );
}

/* CON8---------------------------------------------------- */

unsigned int bq24296_get_system_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296_read_interface((unsigned char) (bq24296_CON8),
				     (&val), (unsigned char) (0xFF), (unsigned char) (0x0)
	    );
	return val;
}

unsigned int bq24296_get_vbus_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296_read_interface((unsigned char) (bq24296_CON8),
				     (&val),
				     (unsigned char) (CON8_VBUS_STAT_MASK),
				     (unsigned char) (CON8_VBUS_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq24296_get_chrg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296_read_interface((unsigned char) (bq24296_CON8),
				     (&val),
				     (unsigned char) (CON8_CHRG_STAT_MASK),
				     (unsigned char) (CON8_CHRG_STAT_SHIFT)
	    );
	return val;
}

unsigned int bq24296_get_vsys_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296_read_interface((unsigned char) (bq24296_CON8),
				     (&val),
				     (unsigned char) (CON8_VSYS_STAT_MASK),
				     (unsigned char) (CON8_VSYS_STAT_SHIFT)
	    );
	return val;
}

/**********************************************************
  *
  *   [Internal Function]
  *
  *********************************************************/
void bq24296_hw_component_detect(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq24296_read_interface(0x0A, &val, 0xFF, 0x0);

	if (val == 0)
		g_bq24296_hw_exist = 0;
	else
		g_bq24296_hw_exist = 1;

	battery_log(BAT_LOG_CRTI, "[bq24296_hw_component_detect] exist=%d, Reg[0x03]=0x%x\n", g_bq24296_hw_exist, val);
}

int is_bq24296_exist(void)
{
	battery_log(BAT_LOG_CRTI, "[is_bq24296_exist] g_bq24296_hw_exist=%d\n", g_bq24296_hw_exist);

	return g_bq24296_hw_exist;
}

void bq24296_dump_register(void)
{
	int i = 0;

	battery_log(BAT_LOG_FULL, "[bq24296] ");
	for (i = 0; i < bq24296_REG_NUM; i++) {
		bq24296_read_byte(i, &bq24296_reg[i]);
		battery_log(BAT_LOG_FULL, "[0x%x]=0x%x ", i, bq24296_reg[i]);
	}
	battery_log(BAT_LOG_FULL, "\n");
}
//zhangjian add for bq24296 ship mode control
static ssize_t Bq24296_contrl_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{

  unsigned char buf[10]={0};
  int ret = 0;
  unsigned short addr=0;
  unsigned char data=0;
  char  c;
  int tmp1,tmp2;
  char keyword[5]={0};
  unsigned char readdata;
  static int j = 0;
  keyword[4]='\0';
  
  printk("Bq24296_ship_mode_contrl_proc in\n");
  if (copy_from_user(buf, buff, 9))
  {
      printk("%s, copy_from_user error\n", __func__);
      return -EFAULT;
   }
  if (buf[0]=='O' && buf[1]=='F' &&buf[2]=='F')
  {
    while(j < 300)
    {
      msleep(500);
//    	if(bq24296_get_chrg_stat() == 0x00)
    	if(bq24296_get_vbus_stat() != 0x01)
    	{
      j = 300;
      printk("Bq24296 set batfet disable \n");
      bq24296_set_watchdog(0x0);
      bq24296_set_batfet_disable(0x1);
    	}
	else
	j = j+1;
    }
     return len;
  }
  else if(buf[0]=='w')
  {
      sscanf(buf, "%c %x %x",&c,&tmp1,&tmp2);
      printk("cmd=%c address=%02x data=%02x \n",c ,tmp1,tmp2);
      addr=tmp1 & 0xff;
      data=tmp2 & 0xff;
      bq24296_write_byte(addr,data);
      return len;
  }
  else if(buf[0]=='r')
  {      
      sscanf(buf, "%c %x %x",&c,&tmp1,&tmp2);
      printk(" cmd=%c address=%02x data=%02x \n",c ,tmp1,tmp2);
      addr=tmp1 & 0xff;
      ret = bq24296_read_byte(addr,&readdata);
      printk("****** readdata=0x%02x",readdata);
      return len;  
  }  
  else
  {
	printk("cmd error\n");
	return len;
  }

 }
 
static const struct file_operations ir_proc_fops = {
	.owner		= THIS_MODULE,	
	.read       = seq_read,	
	.write      = Bq24296_contrl_proc,
};

static void create_Bq24296_proc_file(void)
{
  struct proc_dir_entry *ir_proc_file = proc_create("driver/Bq24296", 0777, NULL,& ir_proc_fops);
  if (ir_proc_file == NULL) 
  {
    printk(KERN_INFO "proc file create failed!\n");
  } 
  
}
//zhangjian add end

static int bq24296_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;

	battery_log(BAT_LOG_CRTI, "[bq24296_driver_probe]\n");

	new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);

	if (!new_client) {
		err = -ENOMEM;
		goto exit;
	}
	memset(new_client, 0, sizeof(struct i2c_client));

	new_client = client;

      /* --------------------- */
      bq24296_hw_component_detect();
      bq24296_dump_register();
      chargin_hw_init_done = KAL_TRUE;       
      create_Bq24296_proc_file();//zhangjian add for bq24296 ship mode control
      return 0;

exit:
	return err;

}

/**********************************************************
  *
  *   [platform_driver API]
  *
  *********************************************************/
unsigned char g_reg_value_bq24296 = 0;
static ssize_t show_bq24296_access(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_log(BAT_LOG_CRTI, "[show_bq24296_access] 0x%x\n", g_reg_value_bq24296);
	return sprintf(buf, "%u\n", g_reg_value_bq24296);
}

static ssize_t store_bq24296_access(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	battery_log(BAT_LOG_CRTI, "[store_bq24296_access]\n");

	if (buf != NULL && size != 0) {
		battery_log(BAT_LOG_CRTI, "[store_bq24296_access] buf is %s and size is %zu\n", buf, size);
		/*reg_address = kstrtoul(buf, 16, &pvalue);*/

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16, (unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);

			battery_log(BAT_LOG_CRTI,
			    "[store_bq24296_access] write bq24296 reg 0x%x with value 0x%x !\n",
			     reg_address, reg_value);
			ret = bq24296_config_interface(reg_address, reg_value, 0xFF, 0x0);
		} else {
			ret = bq24296_read_interface(reg_address, &g_reg_value_bq24296, 0xFF, 0x0);
			battery_log(BAT_LOG_CRTI,
			    "[store_bq24296_access] read bq24296 reg 0x%x with value 0x%x !\n",
			     reg_address, g_reg_value_bq24296);
			battery_log(BAT_LOG_CRTI,
			    "[store_bq24296_access] Please use \"cat bq24296_access\" to get value\r\n");
		}
	}
	return size;
}

static DEVICE_ATTR(bq24296_access, 0664, show_bq24296_access, store_bq24296_access);	/* 664 */

static int bq24296_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	battery_log(BAT_LOG_CRTI, "******** bq24296_user_space_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq24296_access);

	return 0;
}

struct platform_device bq24296_user_space_device = {
	.name = "bq24296-user",
	.id = -1,
};

static struct platform_driver bq24296_user_space_driver = {
	.probe = bq24296_user_space_probe,
	.driver = {
		   .name = "bq24296-user",
		   },
};

static int __init bq24296_subsys_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&bq24296_driver) != 0)
		battery_log(BAT_LOG_CRTI, "[bq24261_init] failed to register bq24261 i2c driver.\n");
	else
		battery_log(BAT_LOG_CRTI, "[bq24261_init] Success to register bq24261 i2c driver.\n");

	/* bq24296 user space access interface */
	ret = platform_device_register(&bq24296_user_space_device);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq24296_init] Unable to device register(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&bq24296_user_space_driver);
	if (ret) {
		battery_log(BAT_LOG_CRTI, "****[bq24296_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

	return 0;
}

static void __exit bq24296_exit(void)
{
	i2c_del_driver(&bq24296_driver);
}

/* module_init(bq24296_init); */
/* module_exit(bq24296_exit); */
subsys_initcall(bq24296_subsys_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq24296 Driver");
MODULE_AUTHOR("YT Lee<yt.lee@mediatek.com>");
