/*
 * Generic I2S codec driver
 *
 * Copyright (C) 2024 comma.ai
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

// Only supports 48kHz, 16-bit audio
static struct snd_soc_dai_driver generic_dai = {
  .name = "HiFi",
  .playback = {
    .stream_name = "HiFi Playback",
    .channels_min = 2,
    .channels_max = 2,
    .rates = SNDRV_PCM_RATE_48000,
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
  },
  .capture = {
    .stream_name = "HiFi Capture",
    .channels_min = 2,
    .channels_max = 2,
    .rates = SNDRV_PCM_RATE_48000,
    .formats = SNDRV_PCM_FMTBIT_S16_LE,
  },
};

static const struct snd_soc_dapm_widget generic_dapm_widgets[] = {
  SND_SOC_DAPM_INPUT("IN_L"),
  SND_SOC_DAPM_INPUT("IN_R"),

  SND_SOC_DAPM_OUTPUT("OUT_L"),
  SND_SOC_DAPM_OUTPUT("OUT_R"),
};

static const struct snd_soc_dapm_route generic_dapm_routes[] = {
	{ "HiFi Playback", NULL, "IN_L" },
	{ "HiFi Playback", NULL, "IN_R" },

	{ "OUT_L", NULL, "HiFi Capture" },
	{ "OUT_R", NULL, "HiFi Capture" },
};

static struct snd_soc_codec_driver soc_generic_codec = {
	.component_driver = {
		.dapm_widgets		= generic_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(generic_dapm_widgets),
		.dapm_routes		= generic_dapm_routes,
		.num_dapm_routes	= ARRAY_SIZE(generic_dapm_routes),
	},
};

static int generic_codec_dev_probe(struct platform_device *pdev)
{
  int ret;
	ret = snd_soc_register_codec(&pdev->dev, &soc_generic_codec, &generic_dai, 1);
  if (ret < 0) {
    dev_err(&pdev->dev, "Failed to register generic codec: %d\n", ret);
    return ret;
  }

  dev_err(&pdev->dev, "Registered generic codec\n");
  return 0;
}

static int generic_codec_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static const struct of_device_id generic_codec_of_match[] = {
	{ .compatible = "generic-codec", },
	{},
};
MODULE_DEVICE_TABLE(of, generic_codec_of_match);

static struct platform_driver generic_codec_driver = {
	.driver = {
		.name = "generic-codec",
 		.of_match_table = of_match_ptr(generic_codec_of_match),
	},
	.probe = generic_codec_dev_probe,
	.remove = generic_codec_dev_remove,
};

module_platform_driver(generic_codec_driver);

MODULE_DESCRIPTION("Generic codec driver");
MODULE_AUTHOR("Robbe Derks <robbe@comma.ai>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:generic-codec");
