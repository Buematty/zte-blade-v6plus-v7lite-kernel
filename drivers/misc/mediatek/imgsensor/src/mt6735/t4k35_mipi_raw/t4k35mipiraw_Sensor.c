/*****************************************************************************
 *
 * Filename:
 * ---------
 *     t4k35mipiraw_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
/*#include <linux/xlog.h>*/
#include <linux/types.h>
#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "t4k35mipiraw_Sensor.h"

#define T4K35_write_cmos_sensor(addr, para) write_cmos_sensor(addr, para)
#define T4K35_read_cmos_sensor(addr) read_cmos_sensor(addr)

#define T4K35_DEBUG
#define T4K35_DEBUG_SOFIA

#ifdef T4K35_DEBUG
	#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#else
	#define T4K35DB(fmt, arg...)
#endif

#ifdef T4K35_DEBUG_SOFIA
	#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#else
	#define T4K35DBSOFIA(fmt, arg...)
#endif

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

/****************************Modify Following Strings for Debug****************************/
#define PFX "t4k35_camera_sensor"
#define LOG_1 LOG_INF("t4k35,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 1632*1224@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/



static DEFINE_SPINLOCK(imgsensor_drv_lock);

static int flag = 0;

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = T4K35_SENSOR_ID,        //record sensor id defined in Kd_imgsensor.h

    .checksum_value = 0xeab06ccb,        //checksum value for Camera Auto Test

    .pre = {
        .pclk = 264000000,                //record different mode's pclk
        .linelength = 3508,                //record different mode's linelength
        .framelength = 2488,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 3264,        //record different mode's width of grabwindow
        .grabwindow_height = 2448,        //record different mode's height of grabwindow
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        /*     following for GetDefaultFramerateByScenario()    */
        .max_framerate = 300,
    },
    .cap = {
        .pclk = 264000000,
        .linelength = 3508,
        .framelength = 2488,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 264000000,
        .linelength = 3508,
        .framelength = 2488,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 240,    //less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
    },
    .normal_video = {
        .pclk = 264000000,
        .linelength = 3508,
        .framelength = 2488,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1836,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 264000000,
        .linelength = 3498,
        .framelength = 786,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,
        .grabwindow_height = 480,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 264000000,
        .linelength = 3508,
        .framelength = 2488,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 1224,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .margin = 6,            //sensor framelength & shutter margin
    .min_shutter = 1,        //min shutter
    .max_frame_length = 0x7fff,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 1,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 1,      //1, support; 0,not support
    .ihdr_le_firstline = 1,  //1,le first ; 0, se first
    .sensor_mode_num = 5,      //support sensor mode num

    .cap_delay_frame = 2,        //enter capture delay frame num
    .pre_delay_frame = 2,         //enter preview delay frame num
    .video_delay_frame = 2,        //enter video delay frame num
    .hs_video_delay_frame = 2,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 2,//enter slim video delay frame num

    .isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
    .i2c_addr_table = {0x6c, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x4EA,                    //current shutter
    .gain = 0x40,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x6c,//record current sensor's i2c write id
};


/* Sensor output window information */

static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = 
{{ 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0, 0, 3264, 2448}, // Preview 
 { 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0, 0, 3264, 2448}, // capture 
 { 3264, 2448, 0, 0, 3264, 2448, 3264, 1836, 0, 0, 3264, 1836,  0, 0, 3264, 1836}, // video 
 { 3264, 2448, 0, 0, 3232, 2448, 3264, 2448, 0, 0, 640, 480,  0, 0, 640, 480}, //hight speed video 
 { 3264, 2448, 0, 0, 3264, 2448, 3264, 2448, 0, 0, 1632, 1224, 0, 0, 1632, 1224}};// slim video

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

#define Sleep(ms) mdelay(ms)


/*******************************************************************************
*
********************************************************************************/
#define OTP_TEST


#ifdef OTP_TEST

#define SET_T4K35_REG(addr, para)  T4K35_write_cmos_sensor((u16)addr, (u8)(para))
#define GET_T4K35_REG(addr, para)  para=T4K35_read_cmos_sensor((u16)addr)

typedef struct t4k35_otp_struct 
{
  uint8_t LSC[53];              /* LSC */
  uint8_t AWB[8];               /* AWB */
  uint8_t Module_Info[9];
  uint8_t AF_Macro[2];
  uint8_t AF_Inifity[5];
} st_t4k35_otp;

#define T4K35_OTP_PSEL 0x3502
#define T4K35_OTP_CTRL 0x3500
#define T4K35_OTP_DATA_BEGIN_ADDR 0x3504
#define T4K35_OTP_DATA_END_ADDR 0x3543

static uint16_t t4k35_otp_data[T4K35_OTP_DATA_END_ADDR - T4K35_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
static uint16_t t4k35_otp_data_backup[T4K35_OTP_DATA_END_ADDR - T4K35_OTP_DATA_BEGIN_ADDR + 1] = {0x00};
static bool firstOpenCamer = true;
static st_t4k35_otp t4k35_data;
static uint16_t t4k35_r_golden_value=0x68; //0x91
static uint16_t t4k35_g_golden_value=0x90; //0xA6
static uint16_t t4k35_b_golden_value=0x5F; //0x81

static void t4k35_otp_set_page(uint16_t page)
{
   	//set page
    SET_T4K35_REG(T4K35_OTP_PSEL, page);
}
static void t4k35_otp_access(void)
{
	uint16_t reg_val;
	//OTP access
	GET_T4K35_REG(T4K35_OTP_CTRL, reg_val);
	SET_T4K35_REG(T4K35_OTP_CTRL, reg_val | 0x80);
	Sleep(30);
}
static void t4k35_otp_read_enble(uint8_t enble)
{
    if (enble)
        SET_T4K35_REG(T4K35_OTP_CTRL, 0x01);
    else
        SET_T4K35_REG(T4K35_OTP_CTRL, 0x00);
}



static int32_t t4k35_otp_read_data(uint16_t* otp_data)
{
    uint16_t i = 0;
    //uint16_t data = 0;
	
    for (i = 0; i <= (T4K35_OTP_DATA_END_ADDR - T4K35_OTP_DATA_BEGIN_ADDR); i++)
	{
        GET_T4K35_REG(T4K35_OTP_DATA_BEGIN_ADDR+i,otp_data[i]);
    }

    return 0;
}

static void t4k35_update_awb(struct t4k35_otp_struct *p_otp)
{
  //return;
  uint16_t rg,bg,r_otp,g_otp,b_otp;

  r_otp=p_otp->AWB[1]+(p_otp->AWB[0]<<8);
  g_otp=(p_otp->AWB[3]+(p_otp->AWB[2]<<8)+p_otp->AWB[5]+(p_otp->AWB[4]<<8))/2;
  b_otp=p_otp->AWB[7]+(p_otp->AWB[6]<<8);
  
  rg = 256*(t4k35_r_golden_value *g_otp)/(r_otp*t4k35_g_golden_value);
  bg = 256*(t4k35_b_golden_value*g_otp)/(b_otp*t4k35_g_golden_value);

  printk("r_golden=0x%x,g_golden=0x%x, b_golden=0x%0x\n", t4k35_r_golden_value,t4k35_g_golden_value,t4k35_b_golden_value);
  printk("r_otp=0x%x,g_opt=0x%x, b_otp=0x%0x\n", r_otp,g_otp,b_otp);
  printk("rg=0x%x, bg=0x%0x\n", rg,bg);

  //R
  SET_T4K35_REG(0x0212, rg >> 8);
  SET_T4K35_REG(0x0213, rg & 0xff);

  //B
  SET_T4K35_REG(0x0214, bg >> 8);
  SET_T4K35_REG(0x0215, bg & 0xff);
  /*T4K35DB("T4K35_OTP_12=%x \n ",(T4K35_read_cmos_sensor(0x0212)));
  T4K35DB("T4K35_OTP_13=%x \n ",(T4K35_read_cmos_sensor(0x0213)));
  T4K35DB("T4K35_OTP_14=%x \n ",(T4K35_read_cmos_sensor(0x0214)));
  T4K35DB("T4K35_OTP_15=%x \n ",(T4K35_read_cmos_sensor(0x0215)));*/
}

static void t4k35_update_lsc(struct t4k35_otp_struct *p_otp)
{
  uint16_t addr;
  int i;

  //set lsc parameters
  addr = 0x323A;
  SET_T4K35_REG(addr, p_otp->LSC[0]);
  addr = 0x323E;
  for(i = 1; i < 53; i++) 
  {
    printk("SET LSC[%d], addr:0x%0x, val:0x%0x\n", i, addr, p_otp->LSC[i]);
    SET_T4K35_REG(addr++, p_otp->LSC[i]);
  }

  //turn on lsc
  SET_T4K35_REG(0x3237,0x80);
}

static int32_t t4k35_otp_init_lsc_awb(struct t4k35_otp_struct *p_otp)
{
  int i,j;
  //uint16_t reg_val;
  uint16_t check_sum=0x00;

  //read OTP LSC and AWB data
  for(i = 3; i >= 0; i--) 
  {
  	//otp enable
  	t4k35_otp_read_enble(1);
  	//read data area
  	//set page
	t4k35_otp_set_page(i);
	//OTP access
    t4k35_otp_access();

	printk("otp lsc data area data:%d\n",i);
    t4k35_otp_read_data(t4k35_otp_data);


	//read data backup area
	printk("otp lsc backup area data:%d\n",i+6);
	//set page
	t4k35_otp_set_page(i+6);
	//OTP access
    t4k35_otp_access();
	
	t4k35_otp_read_data(t4k35_otp_data_backup);
	//otp disable
  	t4k35_otp_read_enble(0);

	//get final OTP data;
    for(j = 0; j < 64; j++) 
	{
        t4k35_otp_data[j]=t4k35_otp_data[j]|t4k35_otp_data_backup[j];
    }

	//check program flag
    if (0 == t4k35_otp_data[0]) 
	{
      continue;
    } 
	else 
	{
	  //checking check sum
	  for(j = 2; j < 64; j++) 
	  {
        check_sum=check_sum+t4k35_otp_data[j];
      }
	  
	  if((check_sum&0xFF)==t4k35_otp_data[1])
	  {
	  	printk("otp lsc checksum ok!\n");
		for(j=3;j<=55;j++)
		{
			p_otp->LSC[j-3]=t4k35_otp_data[j];
		}
		for(j=56;j<=63;j++)
		{
			p_otp->AWB[j-56]=t4k35_otp_data[j];
		}
		return 0;
	  }
	  else
	  {
		printk("otp lsc checksum error!\n");
		return -1;
	  }
    }
  }

  if (i < 0) 
  {
    return -1;
    printk("No otp lsc data on sensor t4k35\n");
  }
  else 
  {
    return 0;
  }
}


static int32_t t4k35_otp_init_module_info(struct t4k35_otp_struct *p_otp)
{
  int i,pos;
  uint16_t check_sum=0x00;

  //otp enable
  t4k35_otp_read_enble(1);
  //set page
  t4k35_otp_set_page(4);
  //OTP access
  t4k35_otp_access();
  printk("data area data:\n");
  t4k35_otp_read_data(t4k35_otp_data);


  //set page
  t4k35_otp_set_page(10);
  //OTP access
  t4k35_otp_access();
  t4k35_otp_read_data(t4k35_otp_data_backup);
  //otp disable
  t4k35_otp_read_enble(0);		
  
  //get final OTP data;
  for(i = 0; i < 64; i++) 
  {
	  t4k35_otp_data[i]=t4k35_otp_data[i]|t4k35_otp_data_backup[i];
  }

  //check flag
  if(t4k35_otp_data[32])
  {
	  pos=32;
  }
  else if(t4k35_otp_data[0])
  {
  	  pos=0;
  }
  else
  {
  	  printk("otp no module information!\n");
  	  return -1;
  }
  

  //checking check sum
  for(i = pos+2; i <pos+32; i++) 
  {
     check_sum=check_sum+t4k35_otp_data[i];
  }
	  
  if((check_sum&0xFF)==t4k35_otp_data[pos+1])
  {
	  	printk("otp module info checksum ok!\n");
		if((t4k35_otp_data[pos+15]==0x00)&&(t4k35_otp_data[pos+16]==0x00)
			&&(t4k35_otp_data[pos+17]==0x00)&&(t4k35_otp_data[pos+18]==0x00)
			&&(t4k35_otp_data[pos+19]==0x00)&&(t4k35_otp_data[pos+20]==0x00)
			&&(t4k35_otp_data[pos+21]==0x00)&&(t4k35_otp_data[pos+22]==0x00))
			return 0;
		
			
		t4k35_r_golden_value=t4k35_otp_data[pos+16]+(t4k35_otp_data[pos+15]<<8);
		t4k35_g_golden_value=(t4k35_otp_data[pos+18]+(t4k35_otp_data[pos+17]<<8)+t4k35_otp_data[pos+20]+(t4k35_otp_data[pos+19]<<8))/2;
		t4k35_b_golden_value=t4k35_otp_data[pos+22]+(t4k35_otp_data[pos+21]<<8);
		return 0;
  }
  else
  {
	printk("otp module info checksum error!\n");
	return -1;
  }

}

static int32_t t4k35_otp_init_setting(void)
{
    int32_t rc = 0;
	
if(firstOpenCamer)
	{
	rc=t4k35_otp_init_module_info(&t4k35_data);
	rc=t4k35_otp_init_lsc_awb(&t4k35_data);
	printk("this is first time enter ,need to read otp zx!\n");
	firstOpenCamer=false;
}
	if(rc==0x00)
	{
		//check module information
	}
	
    
	if(rc==0x00)
	{
		t4k35_update_lsc(&t4k35_data);
		t4k35_update_awb(&t4k35_data);
	}

	
    return rc;
}

#endif


static void set_dummy(void)
{
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	T4K35_write_cmos_sensor(0x0104, 1); 
	T4K35_write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	T4K35_write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	T4K35_write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	T4K35_write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);
	T4K35_write_cmos_sensor(0x0104, 0); 	
}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{

    return ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
#if 1
    //kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

    LOG_INF("framerate = %d, min framelength enable = %d \n", framerate,min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    //dummy_line = frame_length - imgsensor.min_frame_length;
    //if (dummy_line < 0)
        //imgsensor.dummy_line = 0;
    //else
        //imgsensor.dummy_line = dummy_line;
    //imgsensor.frame_length = frame_length + imgsensor.dummy_line;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
#endif
}    /*    set_max_framerate  */



/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    //kal_uint32 frame_length = 0;
	
	//shutter = 752;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    //write_shutter(shutter);
    /* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
    /* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

    // OV Recommend Solution
    // if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;//get bigger one
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;	//get smaller one
#if 1
    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        // Extend frame length
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
        }
    } else {
        // Extend frame length
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
    }
#endif

	
    T4K35_write_cmos_sensor(0x0202, (shutter >> 8) & 0xFF);
   	T4K35_write_cmos_sensor(0x0203, shutter & 0xFF);
	LOG_INF("Exit! autoflicker_en =%d, margin =%d, min_shutter =%d, min_frame_length =%d, max_frame_length =%d\n",imgsensor.autoflicker_en, imgsensor_info.margin, imgsensor_info.min_shutter, imgsensor.min_frame_length, imgsensor_info.max_frame_length);
    LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    /*    set_shutter */



/*static kal_uint16 gain2reg(const kal_uint16 gain) //not use
{
    kal_uint16 reg_gain = 0x0000;

    reg_gain = ((gain / BASEGAIN) << 4) + ((gain % BASEGAIN) * 16 / BASEGAIN);
    reg_gain = reg_gain & 0xFFFF;
    return (kal_uint16)reg_gain;
}*/

/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 Gain;
	kal_uint8  Senosr_base=0x2E;
	Gain=(gain*Senosr_base)/64;
    T4K35_write_cmos_sensor(0x0204, (Gain >> 8) & 0x0F);
    T4K35_write_cmos_sensor(0x0205, Gain  & 0xFF);
	LOG_INF("gain:0x%x, Gain:0x%x\n",gain, Gain);
	
    //T4K35_write_cmos_sensor(0x0104, 0); 
    return gain;

}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
#if 1
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    if (imgsensor.ihdr_en) {
		//write_cmos_sensor(0x0230, 0x01);
		//write_cmos_sensor(0x0230, imgsensor.ihdr_en);
        spin_lock(&imgsensor_drv_lock);
        if (le > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = le + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
        if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


        // Extend frame length first
        write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);

		T4K35_write_cmos_sensor(0x0202, (le >> 8) & 0xFF);
   		T4K35_write_cmos_sensor(0x0203, le & 0xFF);

		write_cmos_sensor(0x0234, ((se) >> 8) & 0xFF);
        write_cmos_sensor(0x0235, (se) & 0xFF);

        set_gain(gain);
    }
#endif
}


#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d\n", image_mirror);

    /********************************************************
       *
       *   0x3820[2] ISP Vertical flip
       *   0x3820[1] Sensor Vertical flip
       *
       *   0x3821[2] ISP Horizontal mirror
       *   0x3821[1] Sensor Horizontal mirror
       *
       *   ISP and Sensor flip or mirror register bit should be the same!!
       *
       ********************************************************/

    switch (image_mirror) {
        case IMAGE_NORMAL:
            write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
            write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x00));
            write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
            write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x06));
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xF9) | 0x06));
            write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xF9) | 0x00));
            break;
        default:
            LOG_INF("Error image_mirror setting\n");
    }


}
#endif
/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}    /*    night_mode    */

static void sensor_init(void)
{
    LOG_INF("E\n");

	T4K35_write_cmos_sensor(0x0101,0x03);//no flip & mirror
	T4K35_write_cmos_sensor(0x0103,0x00);//not reset
	T4K35_write_cmos_sensor(0x0104,0x00);//not hold
	T4K35_write_cmos_sensor(0x0105,0x01);//mask corrupted frames
	T4K35_write_cmos_sensor(0x0110,0x00);//---------???????
	T4K35_write_cmos_sensor(0x0111,0x02);//MIPI
	T4K35_write_cmos_sensor(0x0112,0x0A);//RAW10
	T4K35_write_cmos_sensor(0x0113,0x0A);
	
	T4K35_write_cmos_sensor(0x0114,0x03);//4   //4 lane

	T4K35_write_cmos_sensor(0x0130,0x18);//EXTCLK?????
	T4K35_write_cmos_sensor(0x0131,0x00);
	T4K35_write_cmos_sensor(0x0141,0x00);//context A
	T4K35_write_cmos_sensor(0x0142,0x00);//mask corrupt
	T4K35_write_cmos_sensor(0x0143,0x00);//frame count value
	T4K35_write_cmos_sensor(0x0202,0x09);//exposure time = 2482
	T4K35_write_cmos_sensor(0x0203,0xB2);
	T4K35_write_cmos_sensor(0x0204,0x00);//gain = 46
	T4K35_write_cmos_sensor(0x0205,0x2E);
	T4K35_write_cmos_sensor(0x0210,0x01);
	T4K35_write_cmos_sensor(0x0211,0x00);
	T4K35_write_cmos_sensor(0x0212,0x01);
	T4K35_write_cmos_sensor(0x0213,0x00);
	T4K35_write_cmos_sensor(0x0214,0x01);
	T4K35_write_cmos_sensor(0x0215,0x00);
	T4K35_write_cmos_sensor(0x0216,0x01);
	T4K35_write_cmos_sensor(0x0217,0x00);
	
	//T4K35_write_cmos_sensor(0x0230,0x00);//HDR
	write_cmos_sensor(0x0230, imgsensor.ihdr_en);
	T4K35_write_cmos_sensor(0x0301,0x01);//VT_PIX_CLK_DIV = 1
	
	T4K35_write_cmos_sensor(0x0303,0x05);//clk	//VT_SYS_CLK_DIV = 1/5
	T4K35_write_cmos_sensor(0x0305,0x03);//PRE_PLL_CLK_DIV = 1/4
	T4K35_write_cmos_sensor(0x0306,0x00);//PLL_MULTIPLIER = 110
	T4K35_write_cmos_sensor(0x0307,0x6E);
	T4K35_write_cmos_sensor(0x030B,0x00);//OP_SYS_CLK_DIV = 1
	T4K35_write_cmos_sensor(0x030D,0x03);//PRE_PLL_ST_CLK_DIV = 1/4
	T4K35_write_cmos_sensor(0x030E,0x00);//PLL_MULT_ST = 110
	T4K35_write_cmos_sensor(0x030F,0x6E);
	T4K35_write_cmos_sensor(0x0310,0x00);//OPCK_PLLSEL = PLL_LG
	//MIPI CLK = 660MHz. Internal CLK = 132MHz
	
	T4K35_write_cmos_sensor(0x0340,0x09);//frame and line length
	T4K35_write_cmos_sensor(0x0341,0xB8);
	T4K35_write_cmos_sensor(0x0342,0x0D);
	T4K35_write_cmos_sensor(0x0343,0xB4);
	
	T4K35_write_cmos_sensor(0x0344,0x00);//-/-/-/-/H_CROP[3:0];
	T4K35_write_cmos_sensor(0x0346,0x00);//Y_ADDR_START[15:8];
	T4K35_write_cmos_sensor(0x0347,0x00);//Y_ADDR_START[7:0];
	T4K35_write_cmos_sensor(0x034A,0x09);//Y_ADDR_END[15:8];
	T4K35_write_cmos_sensor(0x034B,0x9f);//Y_ADDR_END[7:0];
	T4K35_write_cmos_sensor(0x034C,0x06);//X_OUTPUT_SIZE[15:8];
	T4K35_write_cmos_sensor(0x034D,0x60);//X_OUTPUT_SIZE[7:0];
	T4K35_write_cmos_sensor(0x034E,0x04);//Y_OUTPUT_SIZE[15:8];
	T4K35_write_cmos_sensor(0x034F,0xc8);//Y_OUTPUT_SIZE[7:0];
	T4K35_write_cmos_sensor(0x0401,0x02);//-/-/-/-/-/-/SCALING_MODE[1:0];
	T4K35_write_cmos_sensor(0x0403,0x00);//-/-/-/-/-/-/SPATIAL_SAMPLING[1:0];
	T4K35_write_cmos_sensor(0x0404,0x20);//SCALE_M[7:0];
	T4K35_write_cmos_sensor(0x0408,0x00);//DCROP_XOFS[15:8];
	T4K35_write_cmos_sensor(0x0409,0x08);//DCROP_XOFS[7:0];
	T4K35_write_cmos_sensor(0x040A,0x00);//DCROP_YOFS[15:8];
	T4K35_write_cmos_sensor(0x040B,0x08);//DCROP_YOFS[7:0];
	T4K35_write_cmos_sensor(0x040C,0x0c);//DCROP_WIDTH[15:8];
	T4K35_write_cmos_sensor(0x040D,0xc0);//DCROP_WIDTH[7:0];
	T4K35_write_cmos_sensor(0x040E,0x09);//DCROP_HIGT[15:8];
	T4K35_write_cmos_sensor(0x040F,0x90);//DCROP_HIGT[7:0];

	T4K35_write_cmos_sensor(0x0601,0x00);//test pattern = no test
	T4K35_write_cmos_sensor(0x0800,0x88);//CSI2 clk delay set
	T4K35_write_cmos_sensor(0x0801,0x38);
	T4K35_write_cmos_sensor(0x0802,0x78);
	T4K35_write_cmos_sensor(0x0803,0x48);
	T4K35_write_cmos_sensor(0x0804,0x48);
	T4K35_write_cmos_sensor(0x0805,0x40);
	T4K35_write_cmos_sensor(0x0806,0x00);
	T4K35_write_cmos_sensor(0x0807,0x48);
	T4K35_write_cmos_sensor(0x0808,0x01);
	T4K35_write_cmos_sensor(0x0820,0x0A);//CSI2 trams target bit rate = 2592
	T4K35_write_cmos_sensor(0x0821,0x20);
	T4K35_write_cmos_sensor(0x0822,0x00);
	T4K35_write_cmos_sensor(0x0823,0x00);
	T4K35_write_cmos_sensor(0x0900,0x00);//no horizon binning
	T4K35_write_cmos_sensor(0x0901,0x00);//no vertical binning
	T4K35_write_cmos_sensor(0x0902,0x00);//vertical binning mode = average binning
	T4K35_write_cmos_sensor(0x0A05,0x01);//mapped couplet correction enabled
	T4K35_write_cmos_sensor(0x0A06,0x01);//ADPC on
	T4K35_write_cmos_sensor(0x0A07,0x98);//ADPC weight
	T4K35_write_cmos_sensor(0x0A0A,0x01);//ADPC on
	T4K35_write_cmos_sensor(0x0A0B,0x98);//ADPC weight
	T4K35_write_cmos_sensor(0x0F00,0x00);//ABF(Auto Bracket Function) setting disabled
	T4K35_write_cmos_sensor(0x300A,0x00);
	T4K35_write_cmos_sensor(0x301A,0x44);
	T4K35_write_cmos_sensor(0x301B,0x44);
	T4K35_write_cmos_sensor(0x3024,0x00);//Reserved ;//zh modify 20140208 Purpose: Reduce gain calibration variability from 2% to 0.7% when turn on.
	T4K35_write_cmos_sensor(0x3025,0x65);//Reserved ;//zh modify 20140208 Purpose: Reduce gain calibration variability from 2% to 0.7% when turn on.
	T4K35_write_cmos_sensor(0x3037,0x06);
	T4K35_write_cmos_sensor(0x3038,0x06);
	T4K35_write_cmos_sensor(0x3039,0x06);
	T4K35_write_cmos_sensor(0x303A,0x06);
	T4K35_write_cmos_sensor(0x303B,0x0B);
	T4K35_write_cmos_sensor(0x303C,0x03);
	T4K35_write_cmos_sensor(0x303D,0x03);
	T4K35_write_cmos_sensor(0x3053,0xC0);
	T4K35_write_cmos_sensor(0x305D,0x10);
	T4K35_write_cmos_sensor(0x305E,0x06);
	T4K35_write_cmos_sensor(0x306B,0x08);
	T4K35_write_cmos_sensor(0x3073,0x1C);
	T4K35_write_cmos_sensor(0x3074,0x0F);
	T4K35_write_cmos_sensor(0x3075,0x03);
	T4K35_write_cmos_sensor(0x3076,0x7F);
	T4K35_write_cmos_sensor(0x307E,0x02);
	T4K35_write_cmos_sensor(0x308D,0x03);
	T4K35_write_cmos_sensor(0x308E,0x20);
	T4K35_write_cmos_sensor(0x3091,0x04);
	T4K35_write_cmos_sensor(0x3096,0x75);
	T4K35_write_cmos_sensor(0x3097,0x7E);
	T4K35_write_cmos_sensor(0x3098,0x20);
	T4K35_write_cmos_sensor(0x30A0,0x82);
	T4K35_write_cmos_sensor(0x30AB,0x30);
	T4K35_write_cmos_sensor(0x30CE,0x60);//Reserved ;
	T4K35_write_cmos_sensor(0x30CF,0x75);
	T4K35_write_cmos_sensor(0x30D2,0xB3);
	T4K35_write_cmos_sensor(0x30D5,0x09);
	T4K35_write_cmos_sensor(0x3134,0x01);
	T4K35_write_cmos_sensor(0x314D,0x80);
	T4K35_write_cmos_sensor(0x315B,0x22);
	T4K35_write_cmos_sensor(0x315C,0x22);
	T4K35_write_cmos_sensor(0x315D,0x02);
	T4K35_write_cmos_sensor(0x3165,0x67);
	T4K35_write_cmos_sensor(0x3168,0xF1);
	T4K35_write_cmos_sensor(0x3169,0x77);
	T4K35_write_cmos_sensor(0x316A,0x77);
	T4K35_write_cmos_sensor(0x3173,0x30);
	T4K35_write_cmos_sensor(0x31B1,0x40);//Reserved ;//zh modify 20140208 Purpose: Reduce gain calibration variability from 2% to 0.7% when turn on.
	T4K35_write_cmos_sensor(0x31C1,0x27);
	T4K35_write_cmos_sensor(0x31DB,0x15);
	T4K35_write_cmos_sensor(0x31DC,0xE0);
	T4K35_write_cmos_sensor(0x3204,0x00);
	T4K35_write_cmos_sensor(0x3231,0x00);	//Preset WB for R gain = x1
	T4K35_write_cmos_sensor(0x3232,0x00);	//Preset WB for Gr gain = x1
	T4K35_write_cmos_sensor(0x3233,0x00);	//Preset WB for Gb gain = x1
	T4K35_write_cmos_sensor(0x3234,0x00);	//Preset WB for B gain = x1
	T4K35_write_cmos_sensor(0x3237,0x00);	//Lens shading setting switch = off
	T4K35_write_cmos_sensor(0x3238,0x00);
	T4K35_write_cmos_sensor(0x3239,0x80);
	T4K35_write_cmos_sensor(0x323A,0x80);
	T4K35_write_cmos_sensor(0x323B,0x00);
	T4K35_write_cmos_sensor(0x323C,0x81);
	T4K35_write_cmos_sensor(0x323D,0x00);
	T4K35_write_cmos_sensor(0x323E,0x02);
	T4K35_write_cmos_sensor(0x323F,0x00);
	T4K35_write_cmos_sensor(0x3240,0x00);
	T4K35_write_cmos_sensor(0x3241,0x00);
	T4K35_write_cmos_sensor(0x3242,0x00);
	T4K35_write_cmos_sensor(0x3243,0x00);
	T4K35_write_cmos_sensor(0x3244,0x00);
	T4K35_write_cmos_sensor(0x3245,0x00);
	T4K35_write_cmos_sensor(0x3246,0x00);
	T4K35_write_cmos_sensor(0x3247,0x00);
	T4K35_write_cmos_sensor(0x3248,0x00);
	T4K35_write_cmos_sensor(0x3249,0x00);
	T4K35_write_cmos_sensor(0x324A,0x00);
	T4K35_write_cmos_sensor(0x324B,0x00);
	T4K35_write_cmos_sensor(0x324C,0x00);
	T4K35_write_cmos_sensor(0x324D,0x00);
	T4K35_write_cmos_sensor(0x324E,0x00);
	T4K35_write_cmos_sensor(0x324F,0x00);
	T4K35_write_cmos_sensor(0x3250,0x00);
	T4K35_write_cmos_sensor(0x3251,0x80);
	T4K35_write_cmos_sensor(0x3252,0x80);
	T4K35_write_cmos_sensor(0x3253,0x80);
	T4K35_write_cmos_sensor(0x3254,0x80);
	T4K35_write_cmos_sensor(0x3255,0x80);
	T4K35_write_cmos_sensor(0x3256,0x80);
	T4K35_write_cmos_sensor(0x3257,0x80);
	T4K35_write_cmos_sensor(0x3258,0x80);
	T4K35_write_cmos_sensor(0x3259,0x80);
	T4K35_write_cmos_sensor(0x325A,0x80);
	T4K35_write_cmos_sensor(0x325B,0x80);
	T4K35_write_cmos_sensor(0x325C,0x80);
	T4K35_write_cmos_sensor(0x325D,0x80);
	T4K35_write_cmos_sensor(0x325E,0x80);
	T4K35_write_cmos_sensor(0x325F,0x80);
	T4K35_write_cmos_sensor(0x3260,0x80);
	T4K35_write_cmos_sensor(0x3261,0x00);
	T4K35_write_cmos_sensor(0x3262,0x00);
	T4K35_write_cmos_sensor(0x3263,0x00);
	T4K35_write_cmos_sensor(0x3264,0x00);
	T4K35_write_cmos_sensor(0x3265,0x00);
	T4K35_write_cmos_sensor(0x3266,0x00);
	T4K35_write_cmos_sensor(0x3267,0x00);
	T4K35_write_cmos_sensor(0x3268,0x00);
	T4K35_write_cmos_sensor(0x3269,0x00);
	T4K35_write_cmos_sensor(0x326A,0x00);
	T4K35_write_cmos_sensor(0x326B,0x00);
	T4K35_write_cmos_sensor(0x326C,0x00);
	T4K35_write_cmos_sensor(0x326D,0x00);
	T4K35_write_cmos_sensor(0x326E,0x00);
	T4K35_write_cmos_sensor(0x326F,0x00);
	T4K35_write_cmos_sensor(0x3270,0x00);
	T4K35_write_cmos_sensor(0x3271,0x80);
	T4K35_write_cmos_sensor(0x3272,0x00);
	T4K35_write_cmos_sensor(0x3273,0x80);	//above are LSSC for lens shading correction
	T4K35_write_cmos_sensor(0x3274,0x01);
	T4K35_write_cmos_sensor(0x3275,0x00);
	T4K35_write_cmos_sensor(0x3276,0x00);
	T4K35_write_cmos_sensor(0x3282,0xC0);	//DPC switch?
	T4K35_write_cmos_sensor(0x3284,0x06);
	T4K35_write_cmos_sensor(0x3285,0x03);
	T4K35_write_cmos_sensor(0x3286,0x02);
	T4K35_write_cmos_sensor(0x328A,0x03);
	T4K35_write_cmos_sensor(0x328B,0x02);
	T4K35_write_cmos_sensor(0x3290,0x20);
	T4K35_write_cmos_sensor(0x3294,0x10);	//memory selection for DPC mode = HDPC
	T4K35_write_cmos_sensor(0x32A8,0x84);	//CNR switch = on
	T4K35_write_cmos_sensor(0x32A9,0x02);	//updating frame data for CNR calculation
	T4K35_write_cmos_sensor(0x32B3,0x10);
	T4K35_write_cmos_sensor(0x32B4,0x1F);
	T4K35_write_cmos_sensor(0x32B7,0x3B);
	T4K35_write_cmos_sensor(0x32BB,0x0F);
	T4K35_write_cmos_sensor(0x32BC,0x0F);
	T4K35_write_cmos_sensor(0x32BE,0x04);
	T4K35_write_cmos_sensor(0x32BF,0x0F);
	T4K35_write_cmos_sensor(0x32C0,0x0F);
	T4K35_write_cmos_sensor(0x32C6,0x50);
	T4K35_write_cmos_sensor(0x32C8,0x0E);
	T4K35_write_cmos_sensor(0x32C9,0x0E);
	T4K35_write_cmos_sensor(0x32CA,0x0E);
	T4K35_write_cmos_sensor(0x32CB,0x0E);
	T4K35_write_cmos_sensor(0x32CC,0x0E);
	T4K35_write_cmos_sensor(0x32CD,0x0E);
	T4K35_write_cmos_sensor(0x32CE,0x08);
	T4K35_write_cmos_sensor(0x32CF,0x08);
	T4K35_write_cmos_sensor(0x32D0,0x08);
	T4K35_write_cmos_sensor(0x32D1,0x0F);
	T4K35_write_cmos_sensor(0x32D2,0x0F);
	T4K35_write_cmos_sensor(0x32D3,0x0F);
	T4K35_write_cmos_sensor(0x32D4,0x08);
	T4K35_write_cmos_sensor(0x32D5,0x08);
	T4K35_write_cmos_sensor(0x32D6,0x08);
	T4K35_write_cmos_sensor(0x32D8,0x00);	//CNR intensity gain setting
	T4K35_write_cmos_sensor(0x32D9,0x00);
	T4K35_write_cmos_sensor(0x32DA,0x00);
	T4K35_write_cmos_sensor(0x32DD,0x02);
	T4K35_write_cmos_sensor(0x32E0,0x20);
	T4K35_write_cmos_sensor(0x32E1,0x20);
	T4K35_write_cmos_sensor(0x32E2,0x20);
	T4K35_write_cmos_sensor(0x32F2,0x04);	//adjustment of CNR intensity = x1
	T4K35_write_cmos_sensor(0x32F3,0x04);	//CNR intensity ratio = 2^4 = 16
	T4K35_write_cmos_sensor(0x3307,0x2E);	//min setting for analog gain
	T4K35_write_cmos_sensor(0x3308,0x2D);	//max setting for analog gain = (0x2D x 16) + 15
	T4K35_write_cmos_sensor(0x3309,0x0D);
	T4K35_write_cmos_sensor(0x3384,0x10);
	T4K35_write_cmos_sensor(0x3424,0x00);	//MIPI clk ULP request and escape mode = off
	T4K35_write_cmos_sensor(0x3425,0x78);	//escape mode transfer data
	T4K35_write_cmos_sensor(0x3427,0xC0);
	T4K35_write_cmos_sensor(0x3430,0xA7);
	T4K35_write_cmos_sensor(0x3431,0x60);
	T4K35_write_cmos_sensor(0x3432,0x11);
	
	mdelay(40);
	T4K35_write_cmos_sensor(0x0100,0x00);
	//T4K35_write_cmos_sensor(0x0100,0x01);	//software standby mode:streaming
	//mdelay(40);
	//mdelay(100);
}    /*    sensor_init  */


static void preview_setting(void)
{

	T4K35_write_cmos_sensor(0x0100,0x00);
	//T4K35_write_cmos_sensor(0x0104,0x01);	//hoid grouped parameter
	T4K35_write_cmos_sensor(0x0344,0x00);////-/-/-/-/H_CROP[3:0];
	T4K35_write_cmos_sensor(0x0346,0x00);////Y_ADDR_START[15:8];
	T4K35_write_cmos_sensor(0x0347,0x00);////Y_ADDR_START[7:0];
	T4K35_write_cmos_sensor(0x034A,0x09);////Y_ADDR_END[15:8];
	T4K35_write_cmos_sensor(0x034B,0x9f);////Y_ADDR_END[7:0];
	T4K35_write_cmos_sensor(0x034C,0x0c);////X_OUTPUT_SIZE[15:8];
	T4K35_write_cmos_sensor(0x034D,0xc0);////X_OUTPUT_SIZE[7:0];
	T4K35_write_cmos_sensor(0x034E,0x09);////Y_OUTPUT_SIZE[15:8];
	T4K35_write_cmos_sensor(0x034F,0x90);////Y_OUTPUT_SIZE[7:0];
	T4K35_write_cmos_sensor(0x0401,0x00);////-/-/-/-/-/-/SCALING_MODE[1:0];
	T4K35_write_cmos_sensor(0x0403,0x00);////-/-/-/-/-/-/SPATIAL_SAMPLING[1:0];
	T4K35_write_cmos_sensor(0x0404,0x10);////SCALE_M[7:0];
	T4K35_write_cmos_sensor(0x0408,0x00);////DCROP_XOFS[15:8];
	T4K35_write_cmos_sensor(0x0409,0x08);////DCROP_XOFS[7:0];
	T4K35_write_cmos_sensor(0x040A,0x00);////DCROP_YOFS[15:8];
	T4K35_write_cmos_sensor(0x040B,0x08);////DCROP_YOFS[7:0];
	T4K35_write_cmos_sensor(0x040C,0x0c);////DCROP_WIDTH[15:8];
	T4K35_write_cmos_sensor(0x040D,0xc0);////DCROP_WIDTH[7:0];
	T4K35_write_cmos_sensor(0x040E,0x09);////DCROP_HIGT[15:8];
	T4K35_write_cmos_sensor(0x040F,0x90);////DCROP_HIGT[7:0];
	T4K35_write_cmos_sensor(0x0900,0x00);////-/-/-/-/-/-/H_BIN[1:0];
	T4K35_write_cmos_sensor(0x0901,0x00);////-/-/-/-/-/-/V_BIN_MODE[1:0];
	T4K35_write_cmos_sensor(0x0902,0x00);////-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	//T4K35_write_cmos_sensor(0x0104,0x00);	//consume as normal


	if(imgsensor.ihdr_en)
		T4K35_write_cmos_sensor(0x230,(1<<4)|(1));
	else
		T4K35_write_cmos_sensor(0x0230,0x00);

	mdelay(40);
	T4K35_write_cmos_sensor(0x0100,0x01);
	mdelay(40);
}    /*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	T4K35_write_cmos_sensor(0x0100,0x00);
	//T4K35_write_cmos_sensor(0x0104,0x01);	//hoid grouped parameter
	T4K35_write_cmos_sensor(0x0344,0x00);////-/-/-/-/H_CROP[3:0];
	T4K35_write_cmos_sensor(0x0346,0x00);////Y_ADDR_START[15:8];
	T4K35_write_cmos_sensor(0x0347,0x00);////Y_ADDR_START[7:0];
	T4K35_write_cmos_sensor(0x034A,0x09);////Y_ADDR_END[15:8];
	T4K35_write_cmos_sensor(0x034B,0x9f);////Y_ADDR_END[7:0];
	T4K35_write_cmos_sensor(0x034C,0x0c);////X_OUTPUT_SIZE[15:8];
	T4K35_write_cmos_sensor(0x034D,0xc0);////X_OUTPUT_SIZE[7:0];
	T4K35_write_cmos_sensor(0x034E,0x09);////Y_OUTPUT_SIZE[15:8];
	T4K35_write_cmos_sensor(0x034F,0x90);////Y_OUTPUT_SIZE[7:0];
	T4K35_write_cmos_sensor(0x0401,0x00);////-/-/-/-/-/-/SCALING_MODE[1:0];
	T4K35_write_cmos_sensor(0x0403,0x00);////-/-/-/-/-/-/SPATIAL_SAMPLING[1:0];
	T4K35_write_cmos_sensor(0x0404,0x10);////SCALE_M[7:0];
	T4K35_write_cmos_sensor(0x0408,0x00);////DCROP_XOFS[15:8];
	T4K35_write_cmos_sensor(0x0409,0x08);////DCROP_XOFS[7:0];
	T4K35_write_cmos_sensor(0x040A,0x00);////DCROP_YOFS[15:8];
	T4K35_write_cmos_sensor(0x040B,0x08);////DCROP_YOFS[7:0];
	T4K35_write_cmos_sensor(0x040C,0x0c);////DCROP_WIDTH[15:8];
	T4K35_write_cmos_sensor(0x040D,0xc0);////DCROP_WIDTH[7:0];
	T4K35_write_cmos_sensor(0x040E,0x09);////DCROP_HIGT[15:8];
	T4K35_write_cmos_sensor(0x040F,0x90);////DCROP_HIGT[7:0];
	T4K35_write_cmos_sensor(0x0900,0x00);////-/-/-/-/-/-/H_BIN[1:0];
	T4K35_write_cmos_sensor(0x0901,0x00);////-/-/-/-/-/-/V_BIN_MODE[1:0];
	T4K35_write_cmos_sensor(0x0902,0x00);////-/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	//T4K35_write_cmos_sensor(0x0104,0x00);	//consume as normal


	if(imgsensor.ihdr_en)
		T4K35_write_cmos_sensor(0x230,(1<<4)|(1));
	else
		T4K35_write_cmos_sensor(0x0230,0x00);

	mdelay(40);
	T4K35_write_cmos_sensor(0x0100,0x01);
	mdelay(40);

}

static void normal_video_setting(kal_uint16 currefps)
{

	T4K35_write_cmos_sensor(0x0100,0x00);

	T4K35_write_cmos_sensor(0x0104,0x01);	//hold grouped parameter
	T4K35_write_cmos_sensor(0x0340,0x09);	//framelength = 2488
	T4K35_write_cmos_sensor(0x0341,0xB8);
	T4K35_write_cmos_sensor(0x0342,0x0D);	//linelength = 3508
	T4K35_write_cmos_sensor(0x0343,0xB4);

	T4K35_write_cmos_sensor(0x0346,0x01);	//y_addr_start = 314
	T4K35_write_cmos_sensor(0x0347,0x3A);
	T4K35_write_cmos_sensor(0x034A,0x08);	//y_addr_end = 2150
	T4K35_write_cmos_sensor(0x034B,0x66);

	T4K35_write_cmos_sensor(0x034C,0x0C);	//x_output_size = 3264
	T4K35_write_cmos_sensor(0x034D,0xC0);
	T4K35_write_cmos_sensor(0x034E,0x07);	//y_output_size = 1836
	T4K35_write_cmos_sensor(0x034F,0x2C);

	T4K35_write_cmos_sensor(0x0301,0x01);	//video timing clk divider =1
	T4K35_write_cmos_sensor(0x0401,0x02);	//scale mode = horizon+vertical
	T4K35_write_cmos_sensor(0x0404,0x10);	//scale factor = 1
	T4K35_write_cmos_sensor(0x0900,0x00);	//no horizon binning
	T4K35_write_cmos_sensor(0x0901,0x00);	//no vertical binning
	T4K35_write_cmos_sensor(0x0104,0x00);	//consume as normal

	if(imgsensor.ihdr_en)
		T4K35_write_cmos_sensor(0x230,(1<<4)|(1));
	else
		T4K35_write_cmos_sensor(0x0230,0x00);

	mdelay(40);
	T4K35_write_cmos_sensor(0x0100,0x01);
	mdelay(40);


}
static void hs_video_setting(void)
{
    LOG_INF("E\n");
	T4K35_write_cmos_sensor(0x0100,0x00);
	
	T4K35_write_cmos_sensor(0x0104,0x0001);	//hold grouped parameter
	T4K35_write_cmos_sensor(0x0340,0x0003);	//framelength = 786
	T4K35_write_cmos_sensor(0x0341,0x0012);
	T4K35_write_cmos_sensor(0x0342,0x000D);	//linelength = 3498
	T4K35_write_cmos_sensor(0x0343,0x00AA);
	T4K35_write_cmos_sensor(0x0346,0x0000);	//y_addr_start = 32
	T4K35_write_cmos_sensor(0x0347,0x0020);
	T4K35_write_cmos_sensor(0x034A,0x0009);	//y_addr_end = 2464
	T4K35_write_cmos_sensor(0x034B,0x00A0);
	T4K35_write_cmos_sensor(0x034C,0x0002);	//y_output_size = 640
	T4K35_write_cmos_sensor(0x034D,0x0080);
	T4K35_write_cmos_sensor(0x034E,0x0001);	//y_output_size = 480
	T4K35_write_cmos_sensor(0x034F,0x00E0);
	T4K35_write_cmos_sensor(0x0301,0x0001);	//video timing clk divider =1
	T4K35_write_cmos_sensor(0x0303,0x0004);	//video timing system clk divider value = 1/4
	T4K35_write_cmos_sensor(0x0401,0x0002);	//scale mode = horizon+vertical
	T4K35_write_cmos_sensor(0x0404,0x0014);	//scale factor = 16/20
	T4K35_write_cmos_sensor(0x0900,0x0002);	//1/4 horizon binning
	T4K35_write_cmos_sensor(0x0901,0x0002);	//1/4 vertical binning
	T4K35_write_cmos_sensor(0x0104,0x0000);	//consume as norma

	if(imgsensor.ihdr_en)
		T4K35_write_cmos_sensor(0x230,(1<<4)|(1));
	else
		T4K35_write_cmos_sensor(0x0230,0x00);

	mdelay(40);
	T4K35_write_cmos_sensor(0x0100,0x01);
	mdelay(40);


}

static void slim_video_setting(void)
{
    LOG_INF("E\n");
    //@@video_720p_30fps_800Mbps
   	T4K35_write_cmos_sensor(0x0100,0x00);

	T4K35_write_cmos_sensor(0x0104,0x01);	//hoid grouped parameter
	T4K35_write_cmos_sensor(0x0340,0x09);	//set framelength=2488
	T4K35_write_cmos_sensor(0x0341,0xB8);
	T4K35_write_cmos_sensor(0x0342,0x0D);	//set linelength=3508
	T4K35_write_cmos_sensor(0x0343,0xB4);

	T4K35_write_cmos_sensor(0x0346,0x00);	//Y_addr_start=8
	T4K35_write_cmos_sensor(0x0347,0x08);	//shijiaoyizhixing
	T4K35_write_cmos_sensor(0x034A,0x09);	//Y_addr_end=2456
	T4K35_write_cmos_sensor(0x034B,0x98);

	T4K35_write_cmos_sensor(0x034C,0x06);	//x_output_size=1632
	T4K35_write_cmos_sensor(0x034D,0x60);
	T4K35_write_cmos_sensor(0x034E,0x04);	//y_output_size=1224
	T4K35_write_cmos_sensor(0x034F,0xC8);	//C8
	T4K35_write_cmos_sensor(0x0301,0x01);	//video timing clk divider =1
	T4K35_write_cmos_sensor(0x0401,0x02);	//scale mode=horizon+vertical
	T4K35_write_cmos_sensor(0x0404,0x10);	//scale factor = 1
	T4K35_write_cmos_sensor(0x0900,0x01);	//1/2 horizon binning
	T4K35_write_cmos_sensor(0x0901,0x01);	//1/2 vertical binning
	T4K35_write_cmos_sensor(0x0104,0x00);	//consume as normal

	if(imgsensor.ihdr_en)
		T4K35_write_cmos_sensor(0x230,(1<<4)|(1));
	else
		T4K35_write_cmos_sensor(0x0230,0x00);

	mdelay(40);
	//write_cmos_sensor(0x0230, 0x00);

	T4K35_write_cmos_sensor(0x0100,0x01);
	LOG_INF("preview_setting() done!\n");
	mdelay(40);
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);

    if (enable) {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0601, 0x02);
    } else {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0601, 0x00);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("--lhl---ok-t4k35-i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                return ERROR_NONE;
            }
            LOG_INF("--lhl--error--t4k35--Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;
    LOG_2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    mDELAY(2);
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();

	#ifdef OTP_TEST
	t4k35_otp_init_setting();
	printk("no otp zx!\n");
	#endif

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}    /*    open  */



/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");

    /*No Need to implement this function*/

    return ERROR_NONE;
}    /*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
	//mDELAY(40);
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	LOG_INF("preview() done!\n");
    return ERROR_NONE;
}    /*    preview   */

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
	
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
	LOG_INF("imgsensor.current_fps=%d, imgsensor.pclk =%d, imgsensor.line_length =%d, imgsensor.frame_length =%d, imgsensor.min_frame_length =%d\n",
				imgsensor.current_fps, imgsensor.pclk, imgsensor.line_length, imgsensor.frame_length, imgsensor.min_frame_length);
    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);	
	//mDELAY(40);
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();	
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);

    return ERROR_NONE;
}    /*    slim_video     */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            preview(image_window, sensor_config_data);
			flag = 0;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
			flag = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            normal_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(image_window, sensor_config_data);
            break;
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{//This Function not used after ROME
    LOG_INF("framerate = %d\n ", framerate);
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
        // Dynamic frame rate
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
#if 1
	if (KAL_TRUE == imgsensor.test_pattern) return ERROR_NONE;	//add myself

    LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) //enable auto flicker
        imgsensor.autoflicker_en = KAL_TRUE;
    else //Cancel Auto flick
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
#endif
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;

}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}



static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    //LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            //LOG_INF("current fps :%d\n", *feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            //LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            //LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data);

            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            //LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 T4K35_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    /*    T4K35_MIPI_RAW_SensorInit    */
