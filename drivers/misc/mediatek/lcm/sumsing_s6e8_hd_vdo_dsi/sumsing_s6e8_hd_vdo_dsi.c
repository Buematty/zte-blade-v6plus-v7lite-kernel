#ifdef BUILD_LK
#include <string.h>
#include <mt_gpio.h>
#include <platform/mt_pmic.h>
#include <cust_gpio_usage.h>
#include <platform/gpio_const.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <linux/string.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
//#include <mach/mt_pm_ldo.h>
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH (720) // pixel
#define FRAME_HEIGHT (1280) // pixel

#define REGFLAG_DELAY 0xFFAB
#define REGFLAG_END_OF_TABLE 0xFFAA // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	 lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)					 lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		 lcm_util.dsi_write_regs(addr, pdata, byte_nums)
//#define read_reg                                          lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define GPIO_LCD_DRV_EN_PIN  (GPIO76 | 0x80000000)

#define GPIO_LCD_ID0_PIN  (GPIO58 | 0x80000000)
#define GPIO_LCD_ID1_PIN  (GPIO57 | 0x80000000)
/*ZSW_MODIFY begin,add for A31 set backlight level,yangchaofeng,20150218*/
#define BRIGHT_TO_BL(out, v, bl_max, max_bright) do {\
					out = (2 * (v) * (bl_max) + max_bright)\
					/ (2 * max_bright) ;\
					} while (0)
/*ZSW_MODIFY end,add for A31 set backlight level,yangchaofeng,20150218*/
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(0, fmt)
#else
#define LCD_DEBUG(fmt, args...)  pr_err("[KERNEL/LCM]"fmt, ##args)
#endif

//#define LCM_DSI_CMD_MODE

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

#if  1  /*new sample*/
static struct LCM_setting_table lcm_sleep_out_setting[] = { 
{0x11, 0, {0x00}},
{REGFLAG_DELAY, 120, {}},
{0x29, 0, {0x00}},
{0x53, 1, {0x28}},
{0x51, 1, {0x00}},
 {REGFLAG_END_OF_TABLE, 0x00, {}} 
}; 
#endif

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
 {REGFLAG_DELAY, 40, {}},
    // Sleep Mode On
	{0x10, 0, {0x00}},
 {REGFLAG_DELAY, 130, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {

		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}

}


/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if 0
		params->dsi.mode   = CMD_MODE;
#else
		//params->dsi.mode   = SYNC_EVENT_VDO_MODE; //BURST_VDO_MODE;
		params->dsi.mode   =BURST_VDO_MODE; 
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
            //The following defined the fomat for data coming from LCD engine.
            params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
            params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
            params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
            params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

            // Highly depends on LCD driver capability.
                 params->dsi.packet_size=256;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
              	params->dsi.vertical_sync_active				=2; // 6; //4   
		params->dsi.vertical_backporch				       = 5;  //14  
		params->dsi.vertical_frontporch				       = 9; //14;  //16  
		params->dsi.vertical_active_line				       = FRAME_HEIGHT;     
		params->dsi.horizontal_sync_active				= 40; // 60;   //4
		params->dsi.horizontal_backporch				= 40; //100;  //60  
		params->dsi.horizontal_frontporch				= 200; //100;    //60
		params->dsi.horizontal_blanking_pixel				= 60;   
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;  

		params->dsi.HS_TRAIL = 5;
		//params->dsi.CLK_TRAIL=5;
		params->dsi.LPX=5;
		params->dsi.HS_PRPR=5;
		params->dsi.cont_clock=1;
                params->dsi.ssc_disable=1;
		params->dsi.PLL_CLOCK = 244 ; //212;   //245
		params->dsi.esd_check_enable = 0;
		params->dsi.customization_esd_check_enable = 0;
		params->dsi.lcm_esd_check_table[0].cmd 			= 0x0a;
		params->dsi.lcm_esd_check_table[0].count 		= 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
		//params->dsi.vertical_vfp_lp = 100;
		//params->dsi.clk_lp_per_line_enable=1;
}

static unsigned int lcm_compare_id(void)
{
	int pin_lcd_id0=0;	  //58  
	int pin_lcd_id1=0;	  //57
	mt_set_gpio_mode(GPIO_LCD_ID0_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_ID0_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_LCD_ID0_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_mode(GPIO_LCD_ID1_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_ID1_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_LCD_ID1_PIN, GPIO_PULL_ENABLE);
        MDELAY(10);
	
	pin_lcd_id0= mt_get_gpio_in(GPIO_LCD_ID0_PIN);
	pin_lcd_id1= mt_get_gpio_in(GPIO_LCD_ID1_PIN);
	
#ifdef BUILD_LK
	printf("%s,sumsing , pin_lcd_id0= %d pin_lcd_id1=%d\n", __func__, pin_lcd_id0,pin_lcd_id1);
#else
	printk("%s,sumsing , pin_lcd_id0= %d pin_lcd_id1=%d\n", __func__, pin_lcd_id0,pin_lcd_id1);
#endif
	if((0==pin_lcd_id0)&&(1==pin_lcd_id1))
	{
		return 1;
	}	
	else 
		return 0;
}
#if 1
static void set_lcd_id_pin(void)
{
	int ret=-1;
#if 0
	ret = mt_set_gpio_mode(GPIO_LCD_ID0_PIN, GPIO_MODE_00);
	if (0 != ret)
		LCD_DEBUG("ID0 mt_set_gpio_mode fail\n");

	ret = mt_set_gpio_dir(GPIO_LCD_ID0_PIN, GPIO_DIR_IN);
	if (0 != ret)
		LCD_DEBUG("ID0 mt_set_gpio_dir fail\n");

	ret = mt_set_gpio_pull_enable(GPIO_LCD_ID0_PIN, GPIO_PULL_ENABLE);
	if (0 != ret)
		LCD_DEBUG("ID0 mt_set_gpio_pull_enable fail\n");

	ret = mt_set_gpio_mode(GPIO_LCD_ID1_PIN, GPIO_MODE_00);
	if (0 != ret)
		LCD_DEBUG("ID1 mt_set_gpio_mode fail\n");

	ret = mt_set_gpio_dir(GPIO_LCD_ID1_PIN, GPIO_DIR_IN);
	if (0 != ret)
		LCD_DEBUG("ID1 mt_set_gpio_dir fail\n");

	ret = mt_set_gpio_pull_enable(GPIO_LCD_ID1_PIN, GPIO_PULL_ENABLE);
	if (0 != ret)
		LCD_DEBUG("ID1 mt_set_gpio_pull_enable fail\n");
#endif
	ret=mt_set_gpio_pull_select(GPIO_LCD_ID0_PIN, GPIO_PULL_DOWN);
	if(0!=ret)
		LCD_DEBUG("[LCD] sumsing set_lcd_id0_pin fail \n");

	ret=mt_set_gpio_pull_select(GPIO_LCD_ID1_PIN, GPIO_PULL_UP);
	if(0!=ret)
		LCD_DEBUG("[LCD] sumsing set_lcd_id1_pin fail \n");

}
#endif

static void lcm_init(void)
{
	mt_set_gpio_mode(GPIO_LCD_DRV_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_DRV_EN_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_DRV_EN_PIN, 1);
	MDELAY(10);
	SET_RESET_PIN(1);
        MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	set_lcd_id_pin();
#ifdef BUILD_LK
	printf("uboot %s\n", __func__);
#else
	printk("kernel %s\n", __func__);
#endif
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
#if 1
	mt_set_gpio_mode(GPIO_LCD_DRV_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_DRV_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_LCD_DRV_EN_PIN, GPIO_PULL_ENABLE);
       mt_set_gpio_out(GPIO_LCD_DRV_EN_PIN, 0);
	mt_set_gpio_pull_select(GPIO_LCD_DRV_EN_PIN, GPIO_PULL_DOWN);
#endif
	SET_RESET_PIN(0);
}

static void lcm_resume(void)
{
	lcm_init();
}
static struct LCM_setting_table lcm_backlight_level_setting[] = {	
	{0X51, 1, {0Xff}},	//Write Display Brightness Value
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_setbacklight(unsigned int level)
{
/*ZSW_MODIFY begin,add for A31 set backlight level,yangchaofeng,20150218*/
	unsigned int bl_lvl;
	static unsigned int last_bl_lvl;
	BRIGHT_TO_BL(bl_lvl,level,215,255);//255 is android max value
	if(level==0)
		bl_lvl=0;
	else if ( (bl_lvl <= 10) && (bl_lvl != 0))
		bl_lvl = 4;

	if(last_bl_lvl==bl_lvl)
		return;
	last_bl_lvl=bl_lvl;
       lcm_backlight_level_setting[0].para_list[0] = bl_lvl;
/*ZSW_MODIFY end,add for A31 set backlight level,yangchaofeng,20150218*/
#ifdef BUILD_LK
	printf("[LCD] sumsing lcm_setbacklight level=%d ---> bl_lvl=%d\n",level,bl_lvl);
#else
	printk("[LCD] sumsing lcm_setbacklight level=%d ---> bl_lvl=%d\n",level,bl_lvl);
#endif
	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);

}
LCM_DRIVER s6e8aa4_dsi_cmd_lcm_drv = {

	.name = "sumsing_s6e8_hd_vdo_dsi",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.compare_id     = lcm_compare_id,
	.set_backlight = lcm_setbacklight,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
};
