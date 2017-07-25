#ifndef BUILD_LK
#include <linux/string.h>
#endif

#include "lcm_drv.h"
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH										(720)
#define FRAME_HEIGHT										(1280)

#define REGFLAG_DELAY								0xFE
#define REGFLAG_END_OF_TABLE							0xFF	/* END OF REGISTERS MARKER */

#define LCM_DSI_CMD_MODE									0
/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	 lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)					 lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		 lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg					 lcm_util.dsi_read_reg()


struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};






static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
		// Display off sequence
		{0x28, 0, {0x00}},
		{REGFLAG_DELAY, 20, {}},
		// Sleep Mode On
		{0x10, 0, {0x00}},
		{REGFLAG_DELAY, 120, {}},

		{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{ 0x51, 1, {0xFF} },
	{ REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void lcm_init_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//mt6325_upmu_set_rg_vgp1_en(1);  //liuyi
#else
	printk("%s, begin\n", __func__);
	//hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	//liuyi
	printk("%s, end\n", __func__);
#endif
#endif
}

static void lcm_suspend_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//mt6325_upmu_set_rg_vgp1_en(0);//liuyi
#else
	printk("%s, begin\n", __func__);
	//hwPowerDown(MT6325_POWER_LDO_VGP1, "LCM_DRV");	//liuyi
	printk("%s, end\n", __func__);
#endif
#endif
}
static void lcm_resume_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//mt6325_upmu_set_rg_vgp1_en(1);  //liuyi
#else
	printk("%s, begin\n", __func__);
	//hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	 //liuyi
	printk("%s, end\n", __func__);
#endif
#endif
}





static void lcm_init_registers_yushun(void)
{
	unsigned int data_array[16];

	/*register write switch  on*/
	data_array[0] = 0x00022902;
	data_array[1] = 0x000004B0;
	dsi_set_cmdq(data_array, 2, 1);		

	/*otp switch  on*/
	data_array[0] = 0x00022902;
	data_array[1] = 0x000001E3;	
	dsi_set_cmdq(data_array, 2, 1);
	
	/*porch*/
	data_array[0] = 0x00072902;
	data_array[1] = 0x10B223C0;
	//data_array[1] = changcmd(0xC0,0x23,0xB2,0x10); 
	data_array[2] =0x007F0210;
	//data_array[2] = changcmd(0x10,0x02,0x7F);	
	dsi_set_cmdq(data_array, 3, 1);

	//gama 2.2
	data_array[0] = 0x001F2902; 
	data_array[1] = 0x090500CA; 
	data_array[2] = 0x201B150D; 
	data_array[3] = 0x1B1E2021;
	data_array[4] = 0x00080D13;
	data_array[5] = 0x0D090500;
	data_array[6] = 0x21201B15;
	data_array[7] = 0x131B1E20;
	data_array[8] = 0x0000080D;
	dsi_set_cmdq(data_array, 9, 1);
	
	//digital gamma ¦Ì¡Â¨¦??¦Ì
	data_array[0] = 0x00142902; 
	data_array[1] = 0x000001CB; 
	data_array[2] = 0x00DC0000; 
	data_array[3] = 0x00000000;
	data_array[4] = 0x000000FC;
	data_array[5] = 0x00FC0000;
	dsi_set_cmdq(data_array, 6,1);

	
 	//flick
	data_array[0] = 0x00022902; 
	data_array[1] = 0x000021C6; 
	dsi_set_cmdq(data_array, 2, 1);
 
	//¡ä¨°?aCE1|?¨¹
	data_array[0] = 0x00022902; 
	data_array[1] = 0x000021BB; 
	dsi_set_cmdq(data_array, 2, 1);
 
	//CE1|?¨¹(??¨¦?????)    
	data_array[0] = 0x000B2902; 
	data_array[1] = 0x808068BD; //o¨¬¨¦?0x602?¡¤???¨¨??¦Ì?a0x80¡ê??¦Ì?a0x60?¨¦¨º1??¨¦?2????¡ä?T¨¤?¡ê??¨¦¡Á?DD¡À¨¨??D¡ì1??¡ê
	data_array[2] = 0x80806868; 
	data_array[3] = 0x00802008;
	dsi_set_cmdq(data_array, 4, 1);
 
	//CE1|?¨¹¡ê¡§??¨¦?????¡ê?¡ã¨¹o?¨¨?¨¢3¨º?¡Àe¡ê?
	data_array[0] = 0x00162902; 
	data_array[1] = 0x4A0AFFBE; 
	data_array[2] = 0xF855A037;
	data_array[3] = 0x10200C0C;
	data_array[4] = 0xE0003F3F;  //0xe0??¡ä??¡Â??¨®|¦Ì??aece¡ê??¨¦¨°?¨ª¡§1y??????¡ä??¡Â¦Ì¡Â??¨¨?¨¢3?????¨º¨¬a?¡ê0xe0???a0xD8¨¨?¨¢3¨¦??¡é??o¨¬¡ê?0xe0???a0xF0??¨¦?¨¦??¡é??¡ã¡Á¡ê???¡Á?DD????¡ê?¨¨¡¤¨¨?D¡ì1??¡ê
	data_array[5] = 0x3F3F1010;
	data_array[6] = 0x00003F3F;
	dsi_set_cmdq(data_array, 7, 1);
}
static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
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

	params->type = LCM_TYPE_DSI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
		params->dsi.mode   = SYNC_EVENT_VDO_MODE;
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	/* Not support in MT6573 */
	params->dsi.packet_size = 256;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 13;
		params->dsi.vertical_frontporch					= 17;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 12;
		params->dsi.horizontal_backporch				= 40;
		params->dsi.horizontal_frontporch				= 100;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

#ifndef CONFIG_FPGA_EARLY_PORTING
                params->dsi.PLL_CLOCK = 220; //this value must be in MTK suggested table
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
		params->dsi.esd_check_enable = 0;
		params->dsi.customization_esd_check_enable = 0;
		params->dsi.lcm_esd_check_table[0].cmd 			= 0x0a;
		params->dsi.lcm_esd_check_table[0].count 		= 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
		params->dsi.vertical_vfp_lp = 100;
}


static void lcm_init(void)
{
	
	printk("%s, yushun begin\n", __func__);
	SET_RESET_PIN(1);
	MDELAY(20);//5
	SET_RESET_PIN(0);
	MDELAY(20);//50
	SET_RESET_PIN(1);
	MDELAY(30);//100
	lcm_init_registers_yushun();
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
	{
	unsigned int data_array[16];
	printk("%s, begin\n", __func__);

	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);

	data_array[0] = 0x00B02300;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00000500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00000500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x01B12300;//Deep standby
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(10);
}


static void lcm_resume(void)
{
	lcm_init();
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);	
}
#endif
static void lcm_setbacklight(unsigned int level)
{
#ifdef BUILD_LK
	dprintf(0,"%s,lk nt35595 backlight: level = %d\n", __func__, level);
#else
	printk("%s, kernel nt35595 backlight: level = %d\n", __func__, level);
#endif
	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = level;
	
	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}



LCM_DRIVER hx8392a_dsi_vdo_3lane_lcm_drv = 
{
    .name			= "hx8392a_dsi_vdo_3lane",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power		= lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
	.set_backlight	= lcm_setbacklight,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
