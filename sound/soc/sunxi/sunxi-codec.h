/*
 * sound/soc/sunxi/sunxi-codec.h
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#ifndef _SUNXI_CODEC_H
#define _SUNXI_CODEC_H

/* Codec Register */
#define CODEC_BASEADDRESS	(0x01c22c00)
#define SUNXI_DAC_DPC		(0x00)
	#define SUNXI_DAC_DPC_DAC_EN			(1 << 31)
	#define SUNXI_DAC_DPC_DIGITAL_VOL		(1 << 12)

#define SUNXI_DAC_FIFOC		(0x04)
	#define SUNXI_DAC_FIFOC_SAMPLE_RATE_96KHZ	(7 << 29)
	#define SUNXI_DAC_FIFOC_SAMPLE_RATE_192KHZ	(6 << 29)
	#define SUNXI_DAC_FIFOC_SAMPLE_RATE_8KHZ	(5 << 29)
	#define SUNXI_DAC_FIFOC_SAMPLE_RATE_12KHZ	(4 << 29)
	#define SUNXI_DAC_FIFOC_SAMPLE_RATE_16KHZ	(3 << 29)
	#define SUNXI_DAC_FIFOC_SAMPLE_RATE_24KHZ	(2 << 29)
	#define SUNXI_DAC_FIFOC_SAMPLE_RATE_32KHZ	(1 << 29)
	#define SUNXI_DAC_FIFOC_SAMPLE_RATE_48KHZ	(0 << 29)
	#define SUNXI_DAC_FIFOC_SAMPLE_RATE_MASK	(7 << 29)
	#define SUNXI_DAC_FIFOC_32_TAP_FIR		(1 << 28)
	#define SUNXI_DAC_FIFOC_64_TAP_FIR		(0 << 28)
	#define SUNXI_DAC_FIFOC_LAST_SE			(1 << 26)
	#define SUNXI_DAC_FIFOC_TX_FIFO_MODE		(1 << 24)
	#define SUNXI_DAC_FIFOC_DRA_LEVEL		(1 << 21)
	#define SUNXI_DAC_FIFOC_TX_TRI_LEVEL		(1 << 8)
	#define SUNXI_DAC_FIFOC_DAC_MODE		(1 << 6)
	#define SUNXI_DAC_FIFOC_TASR			(1 << 5)
	#define SUNXI_DAC_FIFOC_DAC_DRQ			(1 << 4)
	#define SUNXI_DAC_FIFOC_DAC_FIFO_FLUSH		(1 << 0)

#define SUNXI_DAC_FIFOS		(0x08)
#define SUNXI_DAC_TXDATA	(0x0c)
#define SUNXI_DAC_ACTL		(0x10)
	#define SUNXI_DAC_ACTL_DACAEN_R			(1 << 31)
	#define SUNXI_DAC_ACTL_DACAEN_L			(1 << 30)
	#define SUNXI_DAC_ACTL_MIXEN			(1 << 29)
	#define SUNXI_DAC_ACTL_LINE_OUT_VOLUME		(1 << 26)
	#define SUNXI_DAC_ACTL_FM_VOLUME		(7 << 23)
	#define SUNXI_DAC_ACTL_MIC_OUT_VOLUME		(7 << 20)
	#define SUNXI_DAC_ACTL_LINE_SWITCH_L		(1 << 19)
	#define SUNXI_DAC_ACTL_LINE_SWITCH_R		(1 << 18)
	#define SUNXI_DAC_ACTL_FM_SWITCH_L		(1 << 17)
	#define SUNXI_DAC_ACTL_FM_SWITCH_R		(1 << 16)
	#define SUNXI_DAC_ACTL_LDAC_LMIXER		(1 << 15)
	#define SUNXI_DAC_ACTL_RDAC_RMIXER		(1 << 14)
	#define SUNXI_DAC_ACTL_LDAC_RMIXER		(1 << 13)
	#define SUNXI_DAC_ACTL_MIC_INPUT_MUX		(0xf << 9)
	#define SUNXI_DAC_ACTL_DACPAS			(1 << 8)
	#define SUNXI_DAC_ACTL_MIXPAS			(1 << 7)
	#define SUNXI_DAC_ACTL_PA_MUTE			(1 << 6)
	#define SUNXI_DAC_ACTL_VOLUME			(0x3f << 0)

#define SUNXI_DAC_TUNE		(0x14)
	#define SUNXI_DAC_TUNE_SUN7I			(1 << 3)

#define SUNXI_DAC_DEBUG		(0x18)
	#define SUNXI_DAC_DEBUG_DAC_CHANNEL		(1 << 6)

#define SUNXI_ADC_FIFOC		(0x1c)
	#define SUNXI_ADC_FIFOC_SAMPLE_RATE_96KHZ	(7 << 29)
	#define SUNXI_ADC_FIFOC_SAMPLE_RATE_192KHZ	(6 << 29)
	#define SUNXI_ADC_FIFOC_SAMPLE_RATE_8KHZ	(5 << 29)
	#define SUNXI_ADC_FIFOC_SAMPLE_RATE_12KHZ	(4 << 29)
	#define SUNXI_ADC_FIFOC_SAMPLE_RATE_16KHZ	(3 << 29)
	#define SUNXI_ADC_FIFOC_SAMPLE_RATE_24KHZ	(2 << 29)
	#define SUNXI_ADC_FIFOC_SAMPLE_RATE_32KHZ	(1 << 29)
	#define SUNXI_ADC_FIFOC_SAMPLE_RATE_48KHZ	(0 << 29)
	#define SUNXI_ADC_FIFOC_SAMPLE_RATE_MASK	(7 << 29)
	#define SUNXI_ADC_FIFOC_ADC_DIG_EN		(1 << 28)
	#define SUNXI_ADC_FIFOC_RX_FIFO_MODE		(1 << 24)
	#define SUNXI_ADC_FIFOC_RX_TRI_LEVEL		(1 << 8)
	#define SUNXI_ADC_FIFOC_ADC_MODE		(1 << 7)
	#define SUNXI_ADC_FIFOC_RASR			(1 << 6)
	#define SUNXI_ADC_FIFOC_ADC_DRQ			(1 << 4)
	#define SUNXI_ADC_FIFOC_ADC_FIFO_FLUSH		(1 << 0)

#define SUNXI_ADC_FIFOS		(0x20)
#define SUNXI_ADC_RXDATA	(0x24)
#define SUNXI_ADC_ACTL		(0x28)
	#define  SUNXI_ADC_ACTL_ADC_EN			(3 << 30)
	#define  SUNXI_ADC_ACTL_MIC1_EN			(1 << 29)
	#define  SUNXI_ADC_ACTL_MIC2_EN			(1 << 28)
	#define  SUNXI_ADC_ACTL_VMIC_EN			(1 << 27)
	#define  SUNXI_ADC_ACTL_MIC_GAIN		(1 << 25)
	#define  SUNXI_ADC_ACTL_ADC_SELECT		(1 << 17)
	#define  SUNXI_ADC_ACTL_BIT8			(1 << 8)
	#define  SUNXI_ADC_ACTL_PA_ENABLE		(1 << 4)
	#define  SUNXI_ADC_ACTL_HP_DIRECT		(1 << 3)

#define SUNXI_ADC_DEBUG		(0x2c)
#define SUNXI_DAC_TXCNT		(0x30)
#define SUNXI_ADC_RXCNT		(0x34)
#define SUNXI_BIAS_CRT		(0x38)
#define SUNXI_MIC_CRT		(0x3c)
	#define  SUNXI_MIC_CRT_MIC1_GAIN_VOL		(3 << 29)
	#define  SUNXI_MIC_CRT_MIC2_GAIN_VOL		(7 << 26)
	#define  SUNXI_MIC_CRT_LINEIN_APM_VOL		(7 << 13)

#define SUNXI_CODEC_REGS_NUM	(13)

#define DAIFMT_16BITS		(16)
#define DAIFMT_20BITS		(20)

#define DAIFMT_BS_MASK		(~(1 << 16))	/* FIFO big small mode mask */
#define DAIFMT_BITS_MASK	(~(1 << 5))	/* FIFO Bits select mask */
#define SAMPLE_RATE_MASK	(~(7 << 29))	/* Sample Rate slect mask */

#define DAC_EN			(31)
#define DIGITAL_VOL		(12)
/* For Old version of the codec */
#define DAC_VERSION		(23)

#define DAC_CHANNEL		(6)
#define FIR_VERSION		(28)
#define LAST_SE			(26)
#define TX_FIFO_MODE		(24)
#define DRA_LEVEL		(21)
#define TX_TRI_LEVEL		(8)
#define DAC_MODE		(6)
#define TASR			(5)
#define DAC_DRQ			(4)
#define DAC_FIFO_FLUSH		(0)

#define VOLUME			(0)
#define PA_MUTE			(6)
#define MIXPAS			(7)
#define DACPAS			(8)
#define MIXEN			(29)
#define DACAEN_L		(30)
#define DACAEN_R		(31)

#define ADC_DIG_EN		(28)
#define RX_FIFO_MODE		(24)
#define RX_TRI_LEVEL		(8)
#define ADC_MODE		(7)
#define RASR			(6)
#define ADC_DRQ			(4)
#define ADC_FIFO_FLUSH		(0)

#define  ADC_LF_EN		(31)
#define  ADC_RI_EN		(30)
#define  ADC_EN			(30)
#define  MIC1_EN		(29)
#define  MIC2_EN		(28)
#define  VMIC_EN		(27)
#define  MIC_GAIN		(25)
#define  ADC_SELECT		(17)
#define  PA_ENABLE		(4)
#define  HP_DIRECT		(3)

enum m1_codec_config {
	CMD_MIC_SEL = 0,
	CMD_ADC_SEL,
};

void  __iomem *baseaddr;

#define AUDIO_RATE_DEFAULT	44100

#define codec_rdreg(reg)	readl((baseaddr + (reg)))
#define codec_wrreg(reg, val)	writel((val), (baseaddr + (reg)))

/*
 * Convenience kcontrol builders
 */
#define CODEC_SINGLE_VALUE(xreg, xshift, xmax, xinvert) \
		((unsigned long)&(struct codec_mixer_control) \
		{.reg = xreg, .shift = xshift, .rshift = xshift, .max = xmax, \
		.invert = xinvert})

#define CODEC_SINGLE(xname, reg, shift, max, invert) \
{	.iface	= SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info	= snd_codec_info_volsw, .get = snd_codec_get_volsw, \
	.put	= snd_codec_put_volsw, \
	.private_value = CODEC_SINGLE_VALUE(reg, shift, max, invert)}

/* mixer control */
struct	codec_mixer_control {
	int		min;
	int		max;
	int		where;
	unsigned int	mask;
	unsigned int	reg;
	unsigned int	rreg;
	unsigned int	shift;
	unsigned int	rshift;
	unsigned int	invert;
	unsigned int	value;
};

extern int __init snd_chip_codec_mixer_new(struct snd_card *card);
#endif
