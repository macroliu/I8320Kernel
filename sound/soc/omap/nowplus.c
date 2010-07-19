/*
 * nowplus.c  --  SoC audio for the Samsung NOWPLUS board.
 *
 * Derived from sound/soc/omap/omap3beagle.c
 *
 * Author: Joerie de Gram <j.de.gram@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>

#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/twl4030.h"
#include "../codecs/max9877.h"

static int nowplus_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int fmt;
	int ret;

	switch (params_channels(params)) {
	case 2: /* Stereo I2S mode */
		fmt =	SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	case 4: /* Four channel TDM mode */
		fmt =	SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_IB_NF |
			SND_SOC_DAIFMT_CBM_CFM;
		break;
	default:
		return -EINVAL;
	}

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops nowplus_ops = {
	.hw_params = nowplus_hw_params,
};

static int nowplus_twl4030_init(struct snd_soc_codec *codec)
{
	int err = max9877_add_controls(codec);

	if (err < 0)
		return err;

	return 0;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link nowplus_dai = {
	.name = "TWL4030",
	.stream_name = "TWL4030",
	.cpu_dai = &omap_mcbsp_dai[0],
	.codec_dai = &twl4030_dai[TWL4030_DAI_HIFI],
	.ops = &nowplus_ops,
	.init = &nowplus_twl4030_init,
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_nowplus = {
	.name = "nowplus",
	.platform = &omap_soc_platform,
	.dai_link = &nowplus_dai,
	.num_links = 1,
};

/* Audio subsystem */
static struct snd_soc_device nowplus_snd_devdata = {
	.card = &snd_soc_nowplus,
	.codec_dev = &soc_codec_dev_twl4030,
};

static struct platform_device *nowplus_snd_device;

static int __init nowplus_soc_init(void)
{
	int ret;

	pr_info("NOWPLUS SoC init\n");

	nowplus_snd_device = platform_device_alloc("soc-audio", -1);
	if (!nowplus_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(nowplus_snd_device, &nowplus_snd_devdata);
	nowplus_snd_devdata.dev = &nowplus_snd_device->dev;
	*(unsigned int *)nowplus_dai.cpu_dai->private_data = 1; /* McBSP2 */

	ret = platform_device_add(nowplus_snd_device);
	if (ret)
		goto err1;

	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(nowplus_snd_device);

	return ret;
}

static void __exit nowplus_soc_exit(void)
{
	platform_device_unregister(nowplus_snd_device);
}

module_init(nowplus_soc_init);
module_exit(nowplus_soc_exit);

MODULE_AUTHOR("Joerie de Gram <j.de.gram@gmail.com>");
MODULE_DESCRIPTION("ALSA SoC NOWPLUS");
MODULE_LICENSE("GPL");
