/*
 * A V4L2 driver for tw2866 cameras.
 * author: bill
 * wits/merrii
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>
#include <linux/err.h>

#include "camera.h"


MODULE_AUTHOR("raymonxiu");
MODULE_DESCRIPTION("A low-level driver for tw2866 sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN      1 
#if(DEV_DBG_EN == 1)    
#define vfe_dev_dbg(x,arg...) printk("[tw2866]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...) 
#endif
#define vfe_dev_err(x,arg...) printk("[tw2866]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[tw2866]"x,##arg)

#define LOG_ERR_RET(x)  { \
                          int ret;  \
                          ret = x; \
                          if(ret < 0) {\
                            vfe_dev_err("error at %s\n",__func__);  \
                            return ret; \
                          } \
                        }

//define module timing
#define MCLK              (27*1000*1000)
#define V4L2_IDENT_SENSOR 0x2866

//define the voltage level of control signal
#define CSI_STBY_ON     1
#define CSI_STBY_OFF    0
#define CSI_RST_ON      0
#define CSI_RST_OFF     1
#define CSI_PWR_ON      1
#define CSI_PWR_OFF     0
#define CSI_AF_PWR_ON   1
#define CSI_AF_PWR_OFF  0
#define regval_list reg_list_a16_d8

#define REG_DLY  0xffff

#define  TW2866_PAL  0
#define  TW2866_NTSC  1
#define  TW2866_NORMAL  0

/*
* Basic window sizes.  These probably belong somewhere more globally 
* useful. 
*/
#define VGA_WIDTH	704
#define VGA_HEIGHT	576
#define PAL_WIDTH		704
#define PAL_HEIGHT		576

//#define NTSC_WIDTH		720
//#define	NTSC_HEIGHT		480

///extern void  set_aldo1(void);



#define MSG_I2CADDR    (0xb8>>1)

//#define MSG_I2CADDR2    (I2C_ADDR2>>1)

/*
 * Information we maintain about a known sensor.
 */
struct sensor_format_struct;  /* coming later */

static struct class *i2c_dev_class;

struct cfg_array { /* coming later */
	struct regval_list * regs;
	int size;
};

static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
  return container_of(sd, struct sensor_info, sd);
}


/*
 * The default register settings
 *
 */



static struct regval_list sensor_comm_regs[] = {
#if 1
	/* Software Reset*/
	{0x03, 0x09},
	{0x11,0x04},
	{0x12,0x00},
	{0x13,0x04},
	{0x14,0x00}, 
	{0xa0,0x55},
	{0xa1,0xaa},
	{0x69,0x40},
	{0x6d,0x90},

	/* Clock PLL Control */
#endif
};
static struct regval_list sensor_video1_regs[] = {
#if 0
	{0x01, 0x00},/* brightness. */
	{0x02, 0x64},/* luminance contrast gain */	
	{0x03, 0x11},/* sharpness */
	{0x04, 0x80},/* Chroma (U) */
	{0x05, 0x80},/* Chroma (V) */
	{0x06, 0x00}/* hue */
#endif
};
static struct regval_list sensor_video2_regs[] = {
#if 0
	{0x43, 0x08},   //ENCCLK video encoder clock and delay control
	//{0xfa, 0x4a},   //CLKOCTL output enable control and clock output control 108MHz
	{0xfa, 0x40},   //CLKOCTL output enable control and clock output control 27MHz
	{0x9e, 0x52},   //NOVID 
	//{0xca, 0xaa},   // CHMD video channel output control
	//{0xcd, 0x08},   // MAINCH 1st channel slection
	//{0xcc, 0x0d},   // SELCH 2nd chanel slection
	{0xca, 0x00},   // CHMD video channel output control
	{0xcd, 0x00},   // MAINCH 1st channel slection
	//{0xcc, 0x0d},   // SELCH 2nd chanel slection
	{0xaa, 0x00},   //AGCEN video AGC control
	{0xf9, 0x51},   // VMISC video Miscellaneous Function Control 4:2:2
	{0x96, 0xe6},   //MISC2 Miscellaneous Control II
	{0xaf, 0x22},   //VSHP21 vertical peaking level control
	{0xb0, 0x22},   //VSHP43 vertical peaking level control
	{0x41, 0xd4},   //ENCCTL video encoder standard control PAL-N
	{0x43, 0x00},   //ENCCLK video encoder clock and delay control
	{0x4b, 0x00},   //ENCTEST2 video DAC control
#endif
};

static struct regval_list sensor_audio_regs[] = {
#if 0
	/* 0xD0, 0xD1, 0x7F - Analog Audio Input Gain */
	{0x7f, 0x80},/* [7:4] AIGAIN5 [3:0] MIXRATIO5 */
	{0xD0, 0x88},/* [7:4] AIGAIN2 [3:0] AIGAIN1 */
	{0xD1, 0x88},/* [7:4] AIGAIN4 [3:0] AIGAIN3 */
	/* Number of Audio to be Recorded */
	{0xD2, 0x01},/* recode: I2S, 4 chn */
	/* 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA ¨C Sequence of Audio to be Recorded*/
	{0xD3, 0x10},/* default value: 1 0 */
	{0xD7, 0x32},/* default value: 9 8 */
	/* playback: I2S, master, 16bit, ACLKR pin is output */
	{0xDB, 0xE1},
	/* u-Law/A-Law Output and Mix Mute Control */
	{0xDC, 0x00},/* PCM output */
	/* Mix Output Selection */
	{0xE0, 0x15},/* Select playback audio of the first stage chips*/
	//spring
	/* Audio Detection Threshold, 8K typical setting value */
	{0xE1, 0xc0},
	{0xE2, 0xaa},
	{0xE3, 0xaa},
	/* Audio Clock Increment, ACKI[21:0]: 09B583h for fs = 8kHz is default. */
	{0xF0, 0x83},/* ACKI[7:0] */
	{0xF1, 0xb5},/* ACKI[15:8] */
	{0xF2, 0x09},/* ACKI[21:16] */
	/* [3] ACKI control is automatically set up by AFMD register values */
	/* [2:0] AFAUTO control mode. 0: 8kHz setting(default). */
	{0x70, 0x08},/* [2:0] 0:8k, 1:16k, 2:32k, 3:44.1k 4:48k */
	/* Audio Clock Control */
	{0xF8, 0xc4},/* bit2: Loop open in ACKG block */
	/* Enable state register updating and interrupt request of audio AIN5 detection for each input*/
	{0x73, 0x01},
	/* ADATM I2S Output Select (default value)*/
	{0x7B, 0x15},/* Select record audio of channel 51(AIN51)*/
	{0x7C, 0x15},/* Select record audio of channel 51(AIN51)*/
	/* MIX_MUTE_A5 ?????? */
	{0x7E, 0x03},//twll
	/* Audio Fs Mode Control */
	{0x89, 0x0d}/* AIN5MD=1, AFS384=0 *///twll  
#endif
};

static struct regval_list sensor_pal_regs[] = {
#if 0
	{0x0e, 0x01},/* PAL (B,D,G,H,I) */
	//{0x07, 0x12},/* Cropping Register, High */
	{0x07, 0x16},/* Cropping Register, High */
	{0x08, 0x17},/* Vertical Delay Register, Low  */
	{0x09, 0x20},/* Vertical Active Register, Low */
	{0x0a, 0x0c},/* Horizontal Delay Register, Low */
	{0x0b, 0xd0},/* Horizontal Active Register, Low */

	{0x1e, 0x01},/* PAL (B,D,G,H,I) */
	{0x17, 0x12},/* Cropping Register, High */
	{0x18, 0x17},/* Vertical Delay Register, Low  */
	{0x19, 0x20},/* Vertical Active Register, Low */
	{0x1a, 0x0c},/* Horizontal Delay Register, Low */
	{0x1b, 0xd0},/* Horizontal Active Register, Low */ 

	{0x2e, 0x01},/* PAL (B,D,G,H,I) */
	{0x27, 0x12},/* Cropping Register, High */
	{0x28, 0x17},/* Vertical Delay Register, Low  */
	{0x29, 0x20},/* Vertical Active Register, Low */
	{0x2a, 0x0c},/* Horizontal Delay Register, Low */
	{0x2b, 0xd0},/* Horizontal Active Register, Low */ 

	{0x3e, 0x01},/* PAL (B,D,G,H,I) */
	{0x37, 0x12},/* Cropping Register, High */
	{0x38, 0x17},/* Vertical Delay Register, Low  */
	{0x39, 0x20},/* Vertical Active Register, Low */
	{0x3a, 0x0c},/* Horizontal Delay Register, Low */
	{0x3b, 0xd0},/* Horizontal Active Register, Low */ 
	
	{0x41, 0xd4}/* PAL-B -->  HZ50:1 FSCSEL:1 PHALT:1 PED:0 */
#endif

};
static struct regval_list sensor_ntsc_regs[] = {
#if 0
	{0x0e, 0x00},/* NTSC(M) */
	{0x07, 0x02},/* Cropping Register, High */
	{0x08, 0x15},/* Vertical Delay Register, Low  */
	{0x09, 0xf0},/* Vertical Active Register, Low */
	{0x0a, 0x0c},/* Horizontal Delay Register, Low */
	{0x0b, 0xd0},/* Horizontal Active Register, Low */

	{0x1e, 0x00},/* NTSC(M) */
	{0x17, 0x02},/* Cropping Register, High */
	{0x18, 0x15},/* Vertical Delay Register, Low  */
	{0x19, 0xf0},/* Vertical Active Register, Low */
	{0x1a, 0x0c},/* Horizontal Delay Register, Low */
	{0x1b, 0xd0},/* Horizontal Active Register, Low */

	{0x2e, 0x00},/* NTSC(M) */
	{0x27, 0x02},/* Cropping Register, High */
	{0x28, 0x15},/* Vertical Delay Register, Low  */
	{0x29, 0xf0},/* Vertical Active Register, Low */
	{0x2a, 0x0c},/* Horizontal Delay Register, Low */
	{0x2b, 0xd0},/* Horizontal Active Register, Low */

	{0x3e, 0x00},/* NTSC(M) */
	{0x37, 0x02},/* Cropping Register, High */
	{0x38, 0x15},/* Vertical Delay Register, Low  */
	{0x39, 0xf0},/* Vertical Active Register, Low */
	{0x3a, 0x0c},/* Horizontal Delay Register, Low */
	{0x3b, 0xd0},/* Horizontal Active Register, Low */
	
	{0x41, 0x40}/* NTSC-M -->  HZ50:0 FSCSEL:0 PHALT:0 PED:0 */
#endif
};

/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 * 
 */


static struct regval_list sensor_fmt_yuv422_yuyv[] = {	

};


static struct regval_list sensor_fmt_yuv422_yvyu[] = {

};

static struct regval_list sensor_fmt_yuv422_vyuy[] = {

};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {

};

static struct regval_list sensor_fmt[] = {	

};
static unsigned short glb_reg;

/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int sensor_read(struct v4l2_subdev *sd, unsigned short reg,
    unsigned char *value)
{
	int ret=0;
	int cnt=0;
	
  struct i2c_client *client = v4l2_get_subdevdata(sd);

  //hanbiao
   client->addr = MSG_I2CADDR;

   
  ret = cci_read_a8_d8(client,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_read_a8_d8(client,reg,value);
  	cnt++;
  }
  if(cnt>0)
  	vfe_dev_dbg("sensor read retry=%d\n",cnt);
  
  return ret;
}


static int sensor_write(struct v4l2_subdev *sd, unsigned short reg,
    unsigned char value)
{
	int ret=0;
	int cnt=0;
	
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  
  //hanbiao
   client->addr = MSG_I2CADDR;

  ret = cci_write_a8_d8(client,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_write_a8_d8(client,reg,value);
  	cnt++;
  }
  if(cnt>0)
  	vfe_dev_dbg("sensor write retry=%d\n",cnt);
  
  return ret;
}

/*
 * Write a list of register settings;
 */
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *regs, int array_size)
{
	int i=0;

	 //client->addr = I2C_ADDR;
	
  if(!regs)
  	return -EINVAL;
  
  while(i<array_size)
  {
	if(regs->addr == REG_DLY) {
      msleep(regs->data);
    } 
    else {  
      LOG_ERR_RET(sensor_write(sd, regs->addr, regs->data))
    }
    i++;
    regs++;
  }
  return 0;
}

static ssize_t register_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    u8 value;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    
    sensor_read(sd,glb_reg,&value);
    return sprintf(buf, "reg:0x%x,value:0x%x\n",glb_reg,value);
}
static ssize_t register_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    u8 reg,value;
    struct i2c_client *client = container_of(dev, struct i2c_client, dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    if (sscanf(buf, "0x%x,0x%x", &reg,&value) != 2)
        return -EINVAL;
    printk("%s,line:%d,reg:0x%x,value:0x%x\n",__func__,__LINE__,reg,value);
    if(reg != 0xff){
        glb_reg = reg;
        sensor_write(sd,reg,value);
    }else{
        glb_reg = value;
    }
    return size;
}
static DEVICE_ATTR(register, S_IRWXU | S_IRWXG, register_show, register_store);

static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	vfe_dev_dbg("%s,line:%d\n",__func__,__LINE__);
	return 0;
}
static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);
	vfe_dev_dbg("%s,line:%d\n",__func__,__LINE__);
	return 0;
}
static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	vfe_dev_dbg("%s,line:%d\n",__func__,__LINE__);
	return 0;
}
static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	struct sensor_info *info = to_state(sd);
	vfe_dev_dbg("%s,line:%d\n",__func__,__LINE__);

	return 0;
}

static int sensor_s_fps(struct v4l2_subdev *sd)
{
  vfe_dev_dbg("%s,line:%d\n",__func__,__LINE__);
  return 0;
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	vfe_dev_dbg("sensor_get_exposure = %d\n", info->exp);
	return 0;
}

static int sensor_s_exp_gain(struct v4l2_subdev *sd, unsigned int exp_gain)
{
	vfe_dev_dbg("%s,line:%d\n",__func__,__LINE__);

  return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	unsigned char explow,expmid,exphigh;
	struct sensor_info *info = to_state(sd);

	vfe_dev_dbg("%s,line:%d\n",__func__,__LINE__);

	return 0;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	vfe_dev_dbg("sensor_get_gain = %d\n", info->gain);
	return 0;
}


static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);
	vfe_dev_dbg("%s,line:%d\n",__func__,__LINE__);
	
	return 0;
}
static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	int ret;
	unsigned char rdval;
	
	ret=0;
	if(ret!=0)
		return ret;
	
	if(on_off==CSI_STBY_ON)//sw stby on
	{
		ret=0;
	}
	else//sw stby off
	{
		ret=0;
	}
	return ret;
}


/*
 * Stuff that knows about the sensor.
 */
 static int sensor_power(struct v4l2_subdev *sd, int on)
{
  struct i2c_client *client = v4l2_get_subdevdata(sd);
  printk("%s,line:%d\n",__func__,__LINE__);
  switch(on)
  {
    case CSI_SUBDEV_STBY_ON:
      vfe_dev_dbg("CSI_SUBDEV_STBY_ON!\n");
      break;
    case CSI_SUBDEV_STBY_OFF:
      vfe_dev_dbg("CSI_SUBDEV_STBY_OFF!\n");      
      break;
    case CSI_SUBDEV_PWR_ON:
      vfe_dev_dbg("CSI_SUBDEV_PWR_ON!\n");
      i2c_lock_adapter(client->adapter);
      //active mclk before power on
      vfe_set_mclk_freq(sd,MCLK);
      vfe_set_mclk(sd,ON);
      i2c_unlock_adapter(client->adapter);
      break;
    case CSI_SUBDEV_PWR_OFF:
      vfe_dev_dbg("CSI_SUBDEV_PWR_OFF!\n");
      i2c_lock_adapter(client->adapter);
      //inactive mclk before power off
      vfe_set_mclk(sd,OFF);  
      i2c_unlock_adapter(client->adapter);  
      break;
    default:
      return -EINVAL;
  }   

  return 0;
}
 
 static int sensor_reset(struct v4l2_subdev *sd, u32 val)
 {
   printk("%s,line:%d\n",__func__,__LINE__);
   switch(val)
   {
	 case 0:
	   vfe_dev_dbg("%s,line:%d\n",__func__,__LINE__);
	   break;
	 case 1:
	   vfe_dev_dbg("%s,line:%d\n",__func__,__LINE__);
	   break;
	 default:
	   return -EINVAL;
   }
	 
   return 0;
 }

static int sensor_detect(struct v4l2_subdev *sd)
{
#if 1
	unsigned char rdval = 0;

do
{

	sensor_read(sd, 0x80, &rdval);

	msleep(10);

	printk("==tw2866 detect the 0x80 is %d=============\n", rdval);
	
      //return -ENODEV;

	sensor_read(sd, 0x81, &rdval);
	  
      msleep(10);
	
	printk("===tw2866 detect the 0x81 is %d=============\n", rdval);


}while(0);
	//return -ENODEV;
#endif

	return 0;
}
static int tw2866_comm_init(struct v4l2_subdev *sd)
{
	return sensor_write_array(sd, sensor_comm_regs, ARRAY_SIZE(sensor_comm_regs));
}



static int tw2866_init(struct v4l2_subdev *sd)
{
    int ret;
	vfe_dev_err("tw2866_init called\n");
	
	ret = tw2866_comm_init(sd);  
	if(ret < 0) {
		vfe_dev_err("tw2866_comm_init failed\n");
		return ret;
	}

	return 0;
}
static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	
	struct sensor_info *info = to_state(sd);

       printk("%s,line:%d\n",__func__,__LINE__);
	vfe_dev_dbg("tw2866 sensor_init\n");
#if 0
	 set_aldo1();

      //make sure that no device can access i2c bus during sensor initial or power down
      //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
      i2c_lock_adapter(client->adapter);
      //power on reset
      vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
      vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
      //power down io
      vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
      //reset on io
      vfe_gpio_write(sd,RESET,CSI_RST_ON);
      usleep_range(1000,1200);
      //active mclk before power on
      vfe_set_mclk_freq(sd,MCLK);
      vfe_set_mclk(sd,ON);
      usleep_range(10000,12000);
      //power supply
      vfe_gpio_write(sd,POWER_EN,CSI_PWR_ON);
      vfe_set_pmu_channel(sd,IOVDD,ON);
      vfe_set_pmu_channel(sd,AVDD,ON);
      vfe_set_pmu_channel(sd,DVDD,ON);
      vfe_set_pmu_channel(sd,AFVDD,ON);
 
      //remember to unlock i2c adapter, so the device can access the i2c bus again
      i2c_unlock_adapter(client->adapter);

	

#endif

  vfe_gpio_write(sd,RESET,CSI_RST_OFF);

   vfe_gpio_write(sd,POWER_EN,CSI_PWR_OFF);
msleep(20);
	
vfe_gpio_write(sd,POWER_EN,CSI_PWR_ON);  
msleep(10);

	/*Make sure it is a target sensor*/
	ret = sensor_detect(sd);
	if (ret) {
		vfe_dev_err("chip found is not an target chip.\n");
		return ret;
	}
	
    vfe_get_standby_mode(sd,&info->stby_mode);
  
    if((info->stby_mode == HW_STBY || info->stby_mode == SW_STBY) \
        && info->init_first_flag == 0) {
        vfe_dev_print("stby_mode and init_first_flag = 0\n");
        return 0;
    } 
  
	info->focus_status = 0;
	info->low_speed = 0;
	info->width = PAL_WIDTH;
	info->height = PAL_HEIGHT;
	info->gain = 0;

	info->tpf.numerator = 1;            
	info->tpf.denominator = 25;    /* 25fps */    

    ret = tw2866_init(sd);
    if(ret < 0){
        vfe_dev_err("tw2866 init failed.\n");
		return ret;
    }

    if(info->stby_mode == 0)
        info->init_first_flag = 0;
  
    info->preview_first_flag = 1;
    printk("%s,line:%d\n",__func__,__LINE__);
	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
  int ret=0;
    return ret;
}


/*
 * Store information about the video data format. 
 */
static struct sensor_format_struct {
  __u8 *desc;
  //__u32 pixelformat;
  enum v4l2_mbus_pixelcode mbus_code;
  struct regval_list *regs;
  int regs_size;
  int bpp;   /* Bytes per pixel */
}sensor_formats[] = {
	{
		.desc		= "UYVY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
	
#if 0
	{
		.desc		= "VYUY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
	{
		.desc		= "YVYU 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
#endif

};
#define N_FMTS ARRAY_SIZE(sensor_formats)

  

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */


static struct sensor_win_size sensor_win_sizes[] = {
	/* PAL */
	{
		.width			= PAL_WIDTH,
		.height			= PAL_HEIGHT,
        .hoffset    = 0,
        .voffset    = 0,
		.regs				= sensor_pal_regs,
		.regs_size	= ARRAY_SIZE(sensor_pal_regs),
		.set_size		= NULL,
	},
	/* NTSC */
	/*
	{
		.width			= NTSC_WIDTH,
		.height			= NTSC_HEIGHT,
		.regs				= sensor_ntsc_regs,
		.regs_size	= ARRAY_SIZE(sensor_ntsc_regs),
		.set_size		= NULL,
	}
	*/
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))


static int tvp5150_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned index,
						enum v4l2_mbus_pixelcode *code)
{
	if (index)
		return -EINVAL;

	*code = V4L2_MBUS_FMT_YUYV8_2X8;
	return 0;
}

static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)
{
    printk("%s,line:%d\n",__func__,__LINE__);
	if (index >= N_FMTS)
		return -EINVAL;

	*code = sensor_formats[index].mbus_code;
	return 0;
}

static int sensor_enum_size(struct v4l2_subdev *sd,
                            struct v4l2_frmsizeenum *fsize)
{
    printk("%s,line:%d\n",__func__,__LINE__);
  if(fsize->index > N_WIN_SIZES-1)
  	return -EINVAL;
  
  fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
  fsize->discrete.width = sensor_win_sizes[fsize->index].width;
  fsize->discrete.height = sensor_win_sizes[fsize->index].height;
  
  return 0;
}

static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
    struct v4l2_mbus_framefmt *fmt,
    struct sensor_format_struct **ret_fmt,
    struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);
    printk("%s,line:%d,fmt->code = 0x%x\n",__func__,__LINE__,fmt->code);
	for (index = 0; index < N_FMTS; index++)
	if (sensor_formats[index].mbus_code == fmt->code)
		break;

	if (index >= N_FMTS) 
		return -EINVAL;
    printk("%s,line:%d\n",__func__,__LINE__);
	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;

	/*
	* Fields: the sensor devices claim to be progressive.
	*/

  fmt->field = V4L2_FIELD_INTERLACED;//////////////=============>in bt656 set to V4L2_FIELD_INTERLACED/V4L2_FIELD_INTERLACED_TB/V4L2_FIELD_INTERLACED_BT

	/*
	* Round requested image size down to the nearest
	* we support, but not below the smallest.
	*/
	printk("%s,line:%d,width=%d,height=%d\n",__func__,__LINE__,fmt->width,fmt->height);
	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)
			break;
    printk("%s,line:%d\n",__func__,__LINE__);
	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	* Note the size we'll actually handle.
	*/
	fmt->width = wsize->width;
	fmt->height = wsize->height;
//	info->current_wins = wsize;
	printk("%s,line:%d,width=%d,height=%d\n",__func__,__LINE__,wsize->width,wsize->height);
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;
	//pix->sizeimage = pix->height*pix->bytesperline;
	return 0;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)
{
    printk("%s,line:%d\n",__func__,__LINE__);
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}
static int sensor_g_mbus_config(struct v4l2_subdev *sd,
           struct v4l2_mbus_config *cfg)
{
    printk("%s,line:%d\n",__func__,__LINE__);

    cfg->type = V4L2_MBUS_BT656;
  cfg->flags = 0 ;//no use here
    return 0;
}

/*
 * Set a format.
 */
static int sensor_s_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)
{
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);


    printk("%s,line:%d\n",__func__,__LINE__);
	vfe_dev_dbg("sensor_s_fmt\n");

	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
		return ret;

	if(info->capture_mode == V4L2_MODE_VIDEO)
	{
		//video
	}
	else if(info->capture_mode == V4L2_MODE_IMAGE)
	{
		//image 

	}

	sensor_write_array(sd, sensor_fmt->regs, sensor_fmt->regs_size);

	ret = 0;
	if (wsize->regs)
		LOG_ERR_RET(sensor_write_array(sd, wsize->regs, wsize->regs_size))

	if (wsize->set_size)
		LOG_ERR_RET(wsize->set_size(sd))

#if 0
	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;
#else
       info->fmt = sensor_fmt;
	info->width = 704;
	info->height = 576;
#endif

	vfe_dev_print("s_fmt set width = %d, height = %d\n",wsize->width,wsize->height);


	return 0;
}



static int tvp5150_mbus_fmt(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *f)
{
	///struct tvp5150 *decoder = to_tvp5150(sd);
	v4l2_std_id std;

	if (f == NULL)
		return -EINVAL;

	///tvp5150_reset(sd, 0);

	/* Calculate height and width based on current standard */
	//if (decoder->norm == V4L2_STD_ALL)
		//std = tvp5150_read_std(sd);
	///else
		std = V4L2_STD_PAL;

	f->width = 704;
	if (std & V4L2_STD_525_60)
		f->height = 480;
	else
		f->height = 576;

	f->code = V4L2_MBUS_FMT_YUYV8_2X8;
	f->field = V4L2_FIELD_SEQ_TB;
	f->colorspace = V4L2_COLORSPACE_SMPTE170M;

	///v4l2_dbg(1, debug, sd, "width = %d, height = %d\n", f->width,
			///f->height);
	return 0;
}


/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct sensor_info *info = to_state(sd);

    printk("%s,line:%d\n",__func__,__LINE__);
	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->capturemode = info->capture_mode;
	 
	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct sensor_info *info = to_state(sd);
	printk("%s,line:%d\n",__func__,__LINE__);
	vfe_dev_dbg("sensor_s_parm\n");

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (info->tpf.numerator == 0)
		return -EINVAL;

	info->capture_mode = cp->capturemode;

	return 0;
}

static int sensor_queryctrl(struct v4l2_subdev *sd,    struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */
	printk("%s,line:%d\n",__func__,__LINE__);
	switch (qc->id) {
		case V4L2_CID_VFLIP:
		case V4L2_CID_HFLIP:
		    printk("%s,line:%d,V4L2_CID_HFLIP\n",__func__,__LINE__);
			return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
		case V4L2_CID_GAIN:	
		    printk("%s,line:%d,V4L2_CID_GAIN\n",__func__,__LINE__);
			return v4l2_ctrl_query_fill(qc, 1*16, 64*16-1, 1, 1*16);
		case V4L2_CID_EXPOSURE:
		    printk("%s,line:%d,V4L2_CID_EXPOSURE\n",__func__,__LINE__);
			return v4l2_ctrl_query_fill(qc, 0, 65535*16, 1, 0);
		case V4L2_CID_FRAME_RATE:
		    printk("%s,line:%d,V4L2_CID_FRAME_RATE\n",__func__,__LINE__);
			return v4l2_ctrl_query_fill(qc, 15, 120, 1, 120);
	}
	return -EINVAL;
}

static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    printk("%s,line:%d\n",__func__,__LINE__);
	switch (ctrl->id) {
		case V4L2_CID_VFLIP:
		    printk("%s,line:%d,V4L2_CID_VFLIP\n",__func__,__LINE__);
			return sensor_g_vflip(sd, &ctrl->value);
		case V4L2_CID_HFLIP:
		    printk("%s,line:%d,V4L2_CID_HFLIP\n",__func__,__LINE__);
			return sensor_g_hflip(sd, &ctrl->value);
		case V4L2_CID_GAIN:
		    printk("%s,line:%d,V4L2_CID_GAIN\n",__func__,__LINE__);
			return sensor_g_gain(sd, &ctrl->value);
		case V4L2_CID_EXPOSURE:
		    printk("%s,line:%d,V4L2_CID_EXPOSURE\n",__func__,__LINE__);
			return sensor_g_exp(sd, &ctrl->value);
	}
	return -EINVAL;
}


static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct v4l2_queryctrl qc;
	int ret;

    printk("%s,line:%d\n",__func__,__LINE__);
	
	qc.id = ctrl->id;
	ret = sensor_queryctrl(sd, &qc);
	if (ret < 0) {
		return ret;
	}
	if (ctrl->value < qc.minimum || ctrl->value > qc.maximum) {
		return -ERANGE;
	}
	switch (ctrl->id) {
		case V4L2_CID_VFLIP:
		    printk("%s,line:%d,V4L2_CID_VFLIP\n",__func__,__LINE__);
			return sensor_s_vflip(sd, ctrl->value);
		case V4L2_CID_HFLIP:
		    printk("%s,line:%d,V4L2_CID_HFLIP\n",__func__,__LINE__);
			return sensor_s_hflip(sd, ctrl->value);
		case V4L2_CID_GAIN:
			return sensor_s_gain(sd, ctrl->value);
			printk("%s,line:%d,V4L2_CID_GAIN\n",__func__,__LINE__);
		case V4L2_CID_EXPOSURE:
		    printk("%s,line:%d,V4L2_CID_EXPOSURE\n",__func__,__LINE__);
			return sensor_s_exp(sd, ctrl->value);
		case V4L2_CID_FRAME_RATE:
		    printk("%s,line:%d,V4L2_CID_FRAME_RATE\n",__func__,__LINE__);
			return sensor_s_fps(sd);
	}
	return -EINVAL;
}



static int sensor_g_chip_ident(struct v4l2_subdev *sd,
    struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
    printk("%s,line:%d\n",__func__,__LINE__);
	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.g_chip_ident = sensor_g_chip_ident,
	.g_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.queryctrl = sensor_queryctrl,
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.enum_mbus_fmt = sensor_enum_fmt,
	.enum_framesizes = sensor_enum_size,
  .try_mbus_fmt = sensor_try_fmt,
	.s_mbus_fmt = sensor_s_fmt,
	.s_parm = sensor_s_parm,
	.g_parm = sensor_g_parm,
	.g_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */
static int sensor_probe(struct i2c_client *client,
      const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
//	int ret;

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &sensor_ops);

	info->fmt = &sensor_formats[0];
	info->brightness = 0;
	info->contrast = 0;
	info->saturation = 0;
	info->hue = 0;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->autogain = 0;
	info->exp = 0;
	info->autoexp = 0;
	info->autowb = 0;
	info->wb = 0;
	info->clrfx = 0;
	
	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "tw2866", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);


static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tw2866",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
static __init int init_sensor(void)
{
	return i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);

