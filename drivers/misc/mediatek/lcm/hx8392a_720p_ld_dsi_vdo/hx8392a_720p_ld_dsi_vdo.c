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
					/ (2 * max_bright) +25;\
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


static struct LCM_setting_table lcm_initialization_setting[] = {
{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
{0xC0,8,{0x80,0x00,0x01,0x00,0x0C,0x00,0x50,0x00}},
{0xC1,8,{0xC7,0x00,0x00,0x00,0x0B,0x00,0xC0,0xC0}},
{0xC2,8,{0xC0,0x02,0x00,0x00,0x0C,0x00,0x50,0xC7}},
{0xC3,8,{0xC0,0x02,0x00,0x00,0x0D,0x00,0x50,0xC4}},
{0xC4,8,{0xC0,0x02,0x00,0x00,0x0D,0x26,0x50,0xDC}},
{0xC5,8,{0xC0,0x02,0x00,0x00,0x0E,0x26,0x50,0xDC}},
{0xB6,1,{0x00}},
{0xC8,1,{0xF3}},
{0xBA,1,{0x20}},
{0xBB,7,{0x33,0x33,0x33,0x33,0x33,0x33,0x33}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},
{0xE1,1,{0x01}},/*disable dimming*/
{0xCA,1,{0x04}},
{0xE2,1,{0x0A}},
{0xE3,1,{0x00}},
{0xE7,1,{0x00}},
{0xED,8,{0x48,0x00,0xE0,0x13,0x08,0x00,0x92,0x08}},
{0xFD,6,{0x00,0x08,0x1C,0x00,0x00,0x01}},
{0xC3,11,{0x11,0x24,0x04,0x0A,0x02,0x04,0x00,0x1C,0x10,0xF0,0x00}},
{0xEA,5,{0x7F,0x20,0x00,0x00,0x00}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
{0xB0,3,{0x05,0x05,0x05}},
{0xB1,3,{0x05,0x05,0x05}},
{0xB2,3,{0xD0,0xD0,0xD0}},
{0xB4,3,{0xAA,0xAA,0xAA}},
{0xB5,3,{0x33,0x33,0x33}},
{0xB6,3,{0x44,0x44,0x44}},
{0xB7,3,{0x24,0x24,0x24}},
{0xB8,3,{0x24,0x24,0x24}},
{0xB9,3,{0x14,0x14,0x14}},
{0xBA,3,{0x14,0x14,0x14}},
{0xBE,3,{0x23,0x78,0x78}},
{0xCA,1,{0x80}},
{0xCB,12,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xCC,12,{0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},
{0xF1,6,{0x10,0x00,0x00,0x00,0x01,0x2F}},
{0xF6,1,{0x0A}},
{0xCA,1,{0x21}},//open and set acl 
{0xCB,1,{0x61}}, 
{0xCD,1,{0x00}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},
{0xC0,7,{0x06,0x02,0x02,0x22,0x00,0x00,0x01}},
{0xC5,2,{0x00,0xA0}},
{0x51,1,{0x00}},
{0x35,1,{0x01}},
{0xD4,1,{0x02}},/*2:8setps,3:16stesps;4:32 steps;*/
 {0x11,	1,		{0x00}},
 {REGFLAG_DELAY, 120, {}},		
 {0x29,	1,		{0x00}},	
 {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
 {REGFLAG_DELAY, 40, {}},
    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 40, {}},
	{0x4F, 1, {0x01}},
 {REGFLAG_DELAY, 150, {}},
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

		params->dsi.vertical_sync_active				=4; // 6; //4   
		params->dsi.vertical_backporch				       = 8;  //14  
		params->dsi.vertical_frontporch				       = 10; //14;  //16  
		params->dsi.vertical_active_line				       = FRAME_HEIGHT;     
		params->dsi.horizontal_sync_active				= 8; // 60;   //4
		params->dsi.horizontal_backporch				= 24; //100;  //60  
		params->dsi.horizontal_frontporch				= 24; //100;    //60
		params->dsi.horizontal_blanking_pixel				= 60;   
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;  
		//params->dsi.HS_TRAIL = 12;
		//params->dsi.CLK_TRAIL=5;
		params->dsi.ssc_disable=1;
		params->dsi.cont_clock=0;
		params->dsi.PLL_CLOCK = 190; //212;   //245
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
	printf("%s,tianma , pin_lcd_id0= %d pin_lcd_id1=%d\n", __func__, pin_lcd_id0,pin_lcd_id1);
#else
	printk("%s,tianma , pin_lcd_id0= %d pin_lcd_id1=%d\n", __func__, pin_lcd_id0,pin_lcd_id1);
#endif
        if((0==pin_lcd_id0)&&(0==pin_lcd_id1))
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

	ret=mt_set_gpio_pull_select(GPIO_LCD_ID0_PIN, GPIO_PULL_DOWN);
	if(0!=ret)
		LCD_DEBUG("[LCD] tinama set_lcd_id0_pin fail \n");

	ret=mt_set_gpio_pull_select(GPIO_LCD_ID1_PIN, GPIO_PULL_DOWN);
	if(0!=ret)
		LCD_DEBUG("[LCD] tianma set_lcd_id1_pin fail \n");

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
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting,
		   sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
#if 1
	mt_set_gpio_mode(GPIO_LCD_DRV_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_DRV_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_LCD_DRV_EN_PIN, GPIO_PULL_ENABLE);
       mt_set_gpio_out(GPIO_LCD_DRV_EN_PIN, 0);
	mt_set_gpio_pull_select(GPIO_LCD_DRV_EN_PIN, GPIO_PULL_DOWN);
#endif
}

static void lcm_resume(void)
{
	lcm_init();
}
static struct LCM_setting_table lcm_backlight_level_setting[] = {	
	{0X51, 1, {0Xff}},	//Write Display Brightness Value
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting_acl[] = {
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},
	{REGFLAG_DELAY, 16, {}},
	{0xCA,1,{0x21}},
	{REGFLAG_DELAY, 16, {}},
	{0X51, 1, {0Xff}},	//Write Display Brightness Value
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_setbacklight(unsigned int level)
{
/*ZSW_MODIFY begin,add for A31 set backlight level,yangchaofeng,20150218*/
	unsigned int bl_lvl;
	static unsigned int last_bl_lvl;
	static bool acl_is_disable=false;
	BRIGHT_TO_BL(bl_lvl,level,225,255);//255 is android max value
	if(level==0)
		bl_lvl=0;
	else if ( (bl_lvl <= 35) && (bl_lvl != 0))
		bl_lvl = 35;

	if(last_bl_lvl==bl_lvl)
		return;
	last_bl_lvl=bl_lvl;

	if(bl_lvl>=250 && acl_is_disable==false){
		acl_is_disable=true;
		lcm_backlight_level_setting_acl[2].para_list[0] = 0x20;//disable ACL
		lcm_backlight_level_setting_acl[4].para_list[0] = bl_lvl;

		push_table(lcm_backlight_level_setting_acl, sizeof(lcm_backlight_level_setting_acl) / sizeof(struct LCM_setting_table), 1);
	}
	else if(bl_lvl <250 && acl_is_disable==true){
		acl_is_disable=false;
		lcm_backlight_level_setting_acl[2].para_list[0] = 0x21;//disable ACL
		lcm_backlight_level_setting_acl[4].para_list[0] = bl_lvl;

		push_table(lcm_backlight_level_setting_acl, sizeof(lcm_backlight_level_setting_acl) / sizeof(struct LCM_setting_table), 1);
	}
	else {
		lcm_backlight_level_setting[0].para_list[0] = bl_lvl;
		push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
	}
/*ZSW_MODIFY end,add for A31 set backlight level,yangchaofeng,20150218*/
#ifdef BUILD_LK
	printf("[LCD] tianma lcm_setbacklight level=%d ---> bl_lvl=%d\n",level,bl_lvl);
#else
	printk("[LCD] tianma lcm_setbacklight level=%d ---> bl_lvl=%d acl_value=0x%x\n",level,bl_lvl,lcm_backlight_level_setting_acl[2].para_list[0]);
#endif
	
}
LCM_DRIVER hx8392a_720p_ld_dsi_vdo_drv = {

	.name = "hx8392a_720p_ld_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.compare_id     = lcm_compare_id,
	.set_backlight = lcm_setbacklight,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
};
