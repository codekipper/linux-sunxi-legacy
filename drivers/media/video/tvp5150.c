/*
 * tvp5150 - Texas Instruments TVP5150A/AM1 video decoder driver
 *
 * Copyright (c) 2005,2006 Mauro Carvalho Chehab (mchehab@infradead.org)
 * This code is placed under the terms of the GNU General Public License v2
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <media/v4l2-device.h>
#include <media/tvp5150.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>

#include "tvp5150_reg.h"

enum gpio_type {
  POWER_EN,
  PWDN,
  RESET,
  AF_PWDN,
  FLASH_EN,
  FLASH_MODE,
};



//#include "camera.h"
extern int vfe_gpio_write(struct v4l2_subdev *sd, enum gpio_type gpio_type, unsigned int status);

MODULE_DESCRIPTION("Texas Instruments TVP5150A video decoder driver");
MODULE_AUTHOR("Mauro Carvalho Chehab");
MODULE_LICENSE("GPL");
///extern void  set_aldo1(void);

//define the voltage level of control signal
#define CSI_STBY_ON     1
#define CSI_STBY_OFF    0
#define CSI_RST_ON      0
#define CSI_RST_OFF     1
#define CSI_PWR_ON      1
#define CSI_PWR_OFF     0
#define CSI_AF_PWR_ON   1
#define CSI_AF_PWR_OFF  0



#define MSG_I2CADDR    (0xb8>>1)


/* Structure for register values */
struct i2c_reg_value {
	u8 reg;
	u8 value;
};



static int debug;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

struct tvp5150 {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;

	v4l2_std_id norm;	/* Current set standard */
	u32 input;
	u32 output;
	int enable;
};

static inline struct tvp5150 *to_tvp5150(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tvp5150, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct tvp5150, hdl)->sd;
}

static int tvp5150_read(struct v4l2_subdev *sd, unsigned char addr)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	unsigned char buffer[1];
	int rc;
      c->addr = MSG_I2CADDR;

	buffer[0] = addr;
	if (1 != (rc = i2c_master_send(c, buffer, 1)))
		v4l2_dbg(0, debug, sd, "i2c i/o error: rc == %d (should be 1)\n", rc);

	msleep(10);

	if (1 != (rc = i2c_master_recv(c, buffer, 1)))
		v4l2_dbg(0, debug, sd, "i2c i/o error: rc == %d (should be 1)\n", rc);

	v4l2_dbg(2, debug, sd, "tvp5150: read 0x%02x = 0x%02x\n", addr, buffer[0]);

	return (buffer[0]);
}

static inline void tvp5150_write(struct v4l2_subdev *sd, unsigned char addr,
				 unsigned char value)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	unsigned char buffer[2];
	int rc;

	c->addr = MSG_I2CADDR;

	buffer[0] = addr;
	buffer[1] = value;
	v4l2_dbg(2, debug, sd, "tvp5150: writing 0x%02x 0x%02x\n", buffer[0], buffer[1]);
	if (2 != (rc = i2c_master_send(c, buffer, 2)))
		v4l2_dbg(0, debug, sd, "i2c i/o error: rc == %d (should be 2)\n", rc);
}


/****************************************************************************
			Basic functions
 ****************************************************************************/

static inline void tvp5150_selmux(struct v4l2_subdev *sd)
{
    return;
}



/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_default[] = {

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

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_enable[] = {

};

struct tvp5150_vbi_type {
	unsigned int vbi_type;
	unsigned int ini_line;
	unsigned int end_line;
	unsigned int by_field :1;
};

struct i2c_vbi_ram_value {
	u16 reg;
	struct tvp5150_vbi_type type;
	unsigned char values[16];
};

/* This struct have the values for each supported VBI Standard
 * by
 tvp5150_vbi_types should follow the same order as vbi_ram_default
 * value 0 means rom position 0x10, value 1 means rom position 0x30
 * and so on. There are 16 possible locations from 0 to 15.
 */

static struct i2c_vbi_ram_value vbi_ram_default[] =
{
	/* FIXME: Current api doesn't handle all VBI types, those not
	   yet supported are placed under #if 0 */
#if 0
	{0x010, /* Teletext, SECAM, WST System A */
		{V4L2_SLICED_TELETEXT_SECAM,6,23,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x26,
		  0xe6, 0xb4, 0x0e, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
#endif
	{0x030, /* Teletext, PAL, WST System B */
		{V4L2_SLICED_TELETEXT_B,6,22,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0x27, 0x2e, 0x20, 0x2b,
		  0xa6, 0x72, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
#if 0
	{0x050, /* Teletext, PAL, WST System C */
		{V4L2_SLICED_TELETEXT_PAL_C,6,22,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x22,
		  0xa6, 0x98, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x070, /* Teletext, NTSC, WST System B */
		{V4L2_SLICED_TELETEXT_NTSC_B,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0x27, 0x2e, 0x20, 0x23,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x090, /* Tetetext, NTSC NABTS System C */
		{V4L2_SLICED_TELETEXT_NTSC_C,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x22,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x15, 0x00 }
	},
	{0x0b0, /* Teletext, NTSC-J, NABTS System D */
		{V4L2_SLICED_TELETEXT_NTSC_D,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xa7, 0x2e, 0x20, 0x23,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x0d0, /* Closed Caption, PAL/SECAM */
		{V4L2_SLICED_CAPTION_625,22,22,1},
		{ 0xaa, 0x2a, 0xff, 0x3f, 0x04, 0x51, 0x6e, 0x02,
		  0xa6, 0x7b, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00 }
	},
#endif
	{0x0f0, /* Closed Caption, NTSC */
		{V4L2_SLICED_CAPTION_525,21,21,1},
		{ 0xaa, 0x2a, 0xff, 0x3f, 0x04, 0x51, 0x6e, 0x02,
		  0x69, 0x8c, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00 }
	},
	{0x110, /* Wide Screen Signal, PAL/SECAM */
		{V4L2_SLICED_WSS_625,23,23,1},
		{ 0x5b, 0x55, 0xc5, 0xff, 0x00, 0x71, 0x6e, 0x42,
		  0xa6, 0xcd, 0x0f, 0x00, 0x00, 0x00, 0x3a, 0x00 }
	},
#if 0
	{0x130, /* Wide Screen Signal, NTSC C */
		{V4L2_SLICED_WSS_525,20,20,1},
		{ 0x38, 0x00, 0x3f, 0x00, 0x00, 0x71, 0x6e, 0x43,
		  0x69, 0x7c, 0x08, 0x00, 0x00, 0x00, 0x39, 0x00 }
	},
	{0x150, /* Vertical Interval Timecode (VITC), PAL/SECAM */
		{V4l2_SLICED_VITC_625,6,22,0},
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x6d, 0x49,
		  0xa6, 0x85, 0x08, 0x00, 0x00, 0x00, 0x4c, 0x00 }
	},
	{0x170, /* Vertical Interval Timecode (VITC), NTSC */
		{V4l2_SLICED_VITC_525,10,20,0},
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x6d, 0x49,
		  0x69, 0x94, 0x08, 0x00, 0x00, 0x00, 0x4c, 0x00 }
	},
#endif
	{0x190, /* Video Program System (VPS), PAL */
		{V4L2_SLICED_VPS,16,16,0},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xba, 0xce, 0x2b, 0x0d,
		  0xa6, 0xda, 0x0b, 0x00, 0x00, 0x00, 0x60, 0x00 }
	},
	/* 0x1d0 User programmable */

	/* End of struct */
	{ (u16)-1 }
};

static int tvp5150_write_inittab(struct v4l2_subdev *sd,
				const struct i2c_reg_value *regs)
{
	while (regs->reg != 0xff) {
		tvp5150_write(sd, regs->reg, regs->value);
		regs++;
	}
	return 0;
}

static int tvp5150_vdp_init(struct v4l2_subdev *sd,
				const struct i2c_vbi_ram_value *regs)
{
	unsigned int i;

	/* Disable Full Field */
	
	return 0;
}

/* Fills VBI capabilities based on i2c_vbi_ram_value struct */
static int tvp5150_g_sliced_vbi_cap(struct v4l2_subdev *sd,
				struct v4l2_sliced_vbi_cap *cap)
{
	const struct i2c_vbi_ram_value *regs = vbi_ram_default;
	int line;

	v4l2_dbg(1, debug, sd, "g_sliced_vbi_cap\n");
	memset(cap, 0, sizeof *cap);

	while (regs->reg != (u16)-1 ) {
		for (line=regs->type.ini_line;line<=regs->type.end_line;line++) {
			cap->service_lines[0][line] |= regs->type.vbi_type;
		}
		cap->service_set |= regs->type.vbi_type;

		regs++;
	}
	return 0;
}

/* Set vbi processing
 * type - one of tvp5150_vbi_types
 * line - line to gather data
 * fields: bit 0 field1, bit 1, field2
 * flags (default=0xf0) is a bitmask, were set means:
 *	bit 7: enable filtering null bytes on CC
 *	bit 6: send data also to FIFO
 *	bit 5: don't allow data with errors on FIFO
 *	bit 4: enable ECC when possible
 * pix_align = pix alignment:
 *	LSB = field1
 *	MSB = field2
 */
static int tvp5150_set_vbi(struct v4l2_subdev *sd,
			const struct i2c_vbi_ram_value *regs,
			unsigned int type,u8 flags, int line,
			const int fields)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	v4l2_std_id std = decoder->norm;
	u8 reg;
	int pos=0;

	if (std == V4L2_STD_ALL) {
		v4l2_err(sd, "VBI can't be configured without knowing number of lines\n");
		return 0;
	} else if (std & V4L2_STD_625_50) {
		/* Don't follow NTSC Line number convension */
		line += 3;
	}

	if (line<6||line>27)
		return 0;

	while (regs->reg != (u16)-1 ) {
		if ((type & regs->type.vbi_type) &&
		    (line>=regs->type.ini_line) &&
		    (line<=regs->type.end_line)) {
			type=regs->type.vbi_type;
			break;
		}

		regs++;
		pos++;
	}
	if (regs->reg == (u16)-1)
		return 0;

	type=pos | (flags & 0xf0);
	reg=((line-6)<<1)+TVP5150_LINE_MODE_INI;

	if (fields&1) {
		///tvp5150_write(sd, reg, type);
	}

	if (fields&2) {
		///tvp5150_write(sd, reg+1, type);
	}

	return type;
}

static int tvp5150_get_vbi(struct v4l2_subdev *sd,
			const struct i2c_vbi_ram_value *regs, int line)
{

	return 0;
}

static int tvp5150_set_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	int fmt = 0;

	decoder->norm = std;

	/* First tests should be against specific std */

	if (std == V4L2_STD_ALL) {
		fmt = VIDEO_STD_AUTO_SWITCH_BIT;	/* Autodetect mode */
	} else if (std & V4L2_STD_NTSC_443) {
		fmt = VIDEO_STD_NTSC_4_43_BIT;
	} else if (std & V4L2_STD_PAL_M) {
		fmt = VIDEO_STD_PAL_M_BIT;
	} else if (std & (V4L2_STD_PAL_N | V4L2_STD_PAL_Nc)) {
		fmt = VIDEO_STD_PAL_COMBINATION_N_BIT;
	} else {
		/* Then, test against generic ones */
		if (std & V4L2_STD_NTSC)
			fmt = VIDEO_STD_NTSC_MJ_BIT;
		else if (std & V4L2_STD_PAL)
			fmt = VIDEO_STD_PAL_BDGHIN_BIT;
		else if (std & V4L2_STD_SECAM)
			fmt = VIDEO_STD_SECAM_BIT;
	}

	v4l2_dbg(1, debug, sd, "Set video std register to %d.\n", fmt);
	////tvp5150_write(sd, TVP5150_VIDEO_STD, fmt);
	return 0;
}

static int tvp5150_s_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct tvp5150 *decoder = to_tvp5150(sd);

	if (decoder->norm == std)
		return 0;

	return tvp5150_set_std(sd, std);
}

static int tvp5150_reset(struct v4l2_subdev *sd, u32 val)
{
	struct tvp5150 *decoder = to_tvp5150(sd);

	
	/* Initializes TVP5150 to its default values */
	tvp5150_write_inittab(sd, tvp5150_init_default);

	/* Initializes VDP registers */
	///tvp5150_vdp_init(sd, vbi_ram_default);

	/* Selects decoder input */
	///tvp5150_selmux(sd);

	/* Initializes TVP5150 to stream enabled values */
	///tvp5150_write_inittab(sd, tvp5150_init_enable);

	/* Initialize image preferences */
	v4l2_ctrl_handler_setup(&decoder->hdl);

	///tvp5150_set_std(sd, decoder->norm);
	return 0;
};

static int tvp5150_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		///tvp5150_write(sd, TVP5150_BRIGHT_CTL, ctrl->val);
		return 0;
	case V4L2_CID_CONTRAST:
		///tvp5150_write(sd, TVP5150_CONTRAST_CTL, ctrl->val);
		return 0;
	case V4L2_CID_SATURATION:
		///tvp5150_write(sd, TVP5150_SATURATION_CTL, ctrl->val);
		return 0;
	case V4L2_CID_HUE:
		///tvp5150_write(sd, TVP5150_HUE_CTL, ctrl->val);
		return 0;
	}
	return -EINVAL;
}

static v4l2_std_id tvp5150_read_std(struct v4l2_subdev *sd)
{
	//int val = tvp5150_read(sd, TVP5150_STATUS_REG_5);
	int val = 0x03; //hanbiao for test.

	switch (val & 0x0F) {
	case 0x01:
		return V4L2_STD_NTSC;
	case 0x03:
		return V4L2_STD_PAL;
	case 0x05:
		return V4L2_STD_PAL_M;
	case 0x07:
		return V4L2_STD_PAL_N | V4L2_STD_PAL_Nc;
	case 0x09:
		return V4L2_STD_NTSC_443;
	case 0xb:
		return V4L2_STD_SECAM;
	default:
		return V4L2_STD_UNKNOWN;
	}
}

static int tvp5150_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned index,
						enum v4l2_mbus_pixelcode *code)
{
	if (index)
		return -EINVAL;

	*code = V4L2_MBUS_FMT_YUYV8_2X8;
	return 0;
}

static int tvp5150_mbus_fmt(struct v4l2_subdev *sd,
			    struct v4l2_mbus_framefmt *f)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	v4l2_std_id std;

	if (f == NULL)
		return -EINVAL;

	tvp5150_reset(sd, 0);

	/* Calculate height and width based on current standard */
	if (decoder->norm == V4L2_STD_ALL)
		std = tvp5150_read_std(sd);
	else
		std = decoder->norm;

	f->width = 720;
	if (std & V4L2_STD_525_60)
		f->height = 480;
	else
		f->height = 576;

	f->code = V4L2_MBUS_FMT_YUYV8_2X8;
	f->field = V4L2_FIELD_SEQ_TB;
	f->colorspace = V4L2_COLORSPACE_SMPTE170M;

	v4l2_dbg(1, debug, sd, "width = %d, height = %d\n", f->width,
			f->height);
	return 0;
}

/****************************************************************************
			I2C Command
 ****************************************************************************/

static int tvp5150_s_routing(struct v4l2_subdev *sd,
			     u32 input, u32 output, u32 config)
{
	struct tvp5150 *decoder = to_tvp5150(sd);

	decoder->input = input;
	decoder->output = output;
	tvp5150_selmux(sd);
	return 0;
}

static int tvp5150_s_raw_fmt(struct v4l2_subdev *sd, struct v4l2_vbi_format *fmt)
{
	/* this is for capturing 36 raw vbi lines
	   if there's a way to cut off the beginning 2 vbi lines
	   with the tvp5150 then the vbi line count could be lowered
	   to 17 lines/field again, although I couldn't find a register
	   which could do that cropping */
	if (fmt->sample_format == V4L2_PIX_FMT_GREY)
		///tvp5150_write(sd, TVP5150_LUMA_PROC_CTL_1, 0x70);
	if (fmt->count[0] == 18 && fmt->count[1] == 18) {
		///tvp5150_write(sd, TVP5150_VERT_BLANKING_START, 0x00);
		///tvp5150_write(sd, TVP5150_VERT_BLANKING_STOP, 0x01);
	}
	return 0;
}

static int tvp5150_s_sliced_fmt(struct v4l2_subdev *sd, struct v4l2_sliced_vbi_format *svbi)
{

	return 0;
}

static int tvp5150_g_sliced_fmt(struct v4l2_subdev *sd, struct v4l2_sliced_vbi_format *svbi)
{
	int i, mask = 0;

	memset(svbi, 0, sizeof(*svbi));

	for (i = 0; i <= 23; i++) {
		svbi->service_lines[0][i] =
			tvp5150_get_vbi(sd, vbi_ram_default, i);
		mask |= svbi->service_lines[0][i];
	}
	svbi->service_set = mask;
	return 0;
}




#ifdef CONFIG_VIDEO_ADV_DEBUG
static int tvp5150_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	reg->val = tvp5150_read(sd, reg->reg & 0xff);
	reg->size = 1;
	return 0;
}

static int tvp5150_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	////tvp5150_write(sd, reg->reg & 0xff, reg->val & 0xff);
	return 0;
}
#endif

static int tvp5150_g_tuner(struct v4l2_subdev *sd, struct v4l2_tuner *vt)
{
	int status = tvp5150_read(sd, 0x88);

	vt->signal = ((status & 0x04) && (status & 0x02)) ? 0xffff : 0x0;
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_ctrl_ops tvp5150_ctrl_ops = {
	.s_ctrl = tvp5150_s_ctrl,
};

static const struct v4l2_subdev_core_ops tvp5150_core_ops = {
	//.log_status = tvp5150_log_status,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.queryctrl = v4l2_subdev_queryctrl,
	.querymenu = v4l2_subdev_querymenu,
	.s_std = tvp5150_s_std,
	.reset = tvp5150_reset,
	//.g_chip_ident = tvp5150_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = tvp5150_g_register,
	.s_register = tvp5150_s_register,
#endif
};

static const struct v4l2_subdev_tuner_ops tvp5150_tuner_ops = {
	.g_tuner = tvp5150_g_tuner,
};

static const struct v4l2_subdev_video_ops tvp5150_video_ops = {
	.s_routing = tvp5150_s_routing,
	.enum_mbus_fmt = tvp5150_enum_mbus_fmt,
	.s_mbus_fmt = tvp5150_mbus_fmt,
	.try_mbus_fmt = tvp5150_mbus_fmt,
};

static const struct v4l2_subdev_vbi_ops tvp5150_vbi_ops = {
	.g_sliced_vbi_cap = tvp5150_g_sliced_vbi_cap,
	.g_sliced_fmt = tvp5150_g_sliced_fmt,
	.s_sliced_fmt = tvp5150_s_sliced_fmt,
	.s_raw_fmt = tvp5150_s_raw_fmt,
};

static const struct v4l2_subdev_ops tvp5150_ops = {
	.core = &tvp5150_core_ops,
	.tuner = &tvp5150_tuner_ops,
	.video = &tvp5150_video_ops,
	.vbi = &tvp5150_vbi_ops,
};



/****************************************************************************
			I2C Client & Driver
 ****************************************************************************/

static int tvp5150_probe(struct i2c_client *c,
			 const struct i2c_device_id *id)
{
	struct tvp5150 *core;
	struct v4l2_subdev *sd;
	u8 msb_id, lsb_id, msb_rom, lsb_rom;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(c->adapter,
	     I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -EIO;

	core = kzalloc(sizeof(struct tvp5150), GFP_KERNEL);
	if (!core) {
		return -ENOMEM;
	}
	sd = &core->sd;
	v4l2_i2c_subdev_init(sd, c, &tvp5150_ops);
	v4l_info(c, "chip found @ 0x%02x (%s)\n",
		 c->addr << 1, c->adapter->name);

	  vfe_gpio_write(sd,RESET,CSI_RST_OFF);

     vfe_gpio_write(sd,POWER_EN,CSI_PWR_OFF);
    msleep(20);
	
   vfe_gpio_write(sd,POWER_EN,CSI_PWR_ON);  
   msleep(10);



	msb_id = tvp5150_read(sd, TVP5150_MSB_DEV_ID);
	lsb_id = tvp5150_read(sd, TVP5150_LSB_DEV_ID);

	
	v4l2_info(sd, "tvp%02x%02xam1 detected.\n", msb_id, lsb_id);

	/* ITU-T BT.656.4 timing */
		
	core->norm = V4L2_STD_ALL;	/* Default is autodetect */
	core->input = TVP5150_COMPOSITE1;
	core->enable = 1;

	v4l2_ctrl_handler_init(&core->hdl, 4);
	v4l2_ctrl_new_std(&core->hdl, &tvp5150_ctrl_ops,
			V4L2_CID_BRIGHTNESS, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&core->hdl, &tvp5150_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&core->hdl, &tvp5150_ctrl_ops,
			V4L2_CID_SATURATION, 0, 255, 1, 128);
	v4l2_ctrl_new_std(&core->hdl, &tvp5150_ctrl_ops,
			V4L2_CID_HUE, -128, 127, 1, 0);
	sd->ctrl_handler = &core->hdl;
	if (core->hdl.error) {
		int err = core->hdl.error;

		v4l2_ctrl_handler_free(&core->hdl);
		kfree(core);
		return err;
	}
	v4l2_ctrl_handler_setup(&core->hdl);
	
	return 0;
}

static int tvp5150_remove(struct i2c_client *c)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(c);
	struct tvp5150 *decoder = to_tvp5150(sd);

	v4l2_dbg(1, debug, sd,
		"tvp5150.c: removing tvp5150 adapter on address 0x%x\n",
		c->addr << 1);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&decoder->hdl);
	kfree(to_tvp5150(sd));
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id tvp5150_id[] = {
	{ "tvp5150", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tvp5150_id);

static struct i2c_driver tvp5150_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tvp5150",
	},
	.probe		= tvp5150_probe,
	.remove		= tvp5150_remove,
	.id_table	= tvp5150_id,
};

static __init int init_tvp5150(void)
{
	return i2c_add_driver(&tvp5150_driver);
}

static __exit void exit_tvp5150(void)
{
	i2c_del_driver(&tvp5150_driver);
}

module_init(init_tvp5150);
module_exit(exit_tvp5150);
