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

static int generic_codec_set_sysclk(struct snd_soc_dai *dai, int clk_id,
                                    unsigned int freq, int dir) {
  return 0;
}

static int generic_codec_set_fmt(struct snd_soc_dai *dai, unsigned int fmt) {
  return 0;
}

static int generic_codec_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *params,
                                  struct snd_soc_dai *dai) {
  return 0;
}

static const struct snd_soc_dai_ops generic_codec_ops = {
  .set_sysclk = generic_codec_set_sysclk,
  .set_fmt = generic_codec_set_fmt,
  .hw_params = generic_codec_hw_params,
};

static struct snd_soc_dai_driver generic_dai = {
  .name = "HiFi",
  .playback = {
    .stream_name = "HiFi Playback",
    .channels_min = 1,
    .channels_max = 2,
    .rates = SNDRV_PCM_RATE_8000_96000,
    .formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE),
  },
  .capture = {
    .stream_name = "HiFi Capture",
    .channels_min = 1,
    .channels_max = 2,
    .rates = SNDRV_PCM_RATE_8000_96000,
    .formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE),
  },
  .ops = &generic_codec_ops,
};

static const struct snd_soc_dapm_widget generic_dapm_widgets[] = {
  SND_SOC_DAPM_ADC("ADCL", NULL, SND_SOC_NOPM, 0, 0),
  SND_SOC_DAPM_ADC("ADCR", NULL, SND_SOC_NOPM, 0, 0),

  SND_SOC_DAPM_OUTPUT("SPKL"),
  SND_SOC_DAPM_OUTPUT("SPKR"),

  SND_SOC_DAPM_INPUT("MIC1"),
  SND_SOC_DAPM_INPUT("MIC2"),
};

static const struct snd_soc_dapm_route intercon[] = {
	// {"SPKL", NULL, "DACL1"},
  // {"SPKR", NULL, "DACL2"},
  // {"ADCL", NULL, "MIC1"},
  // {"ADCR", NULL, "MIC2"},

  /* Left headphone output mixer */
  {"Left HP Mixer", "Left DAC1 Switch", "DACL1"},
  {"Left HP Mixer", "Left DAC2 Switch", "DACL2"},
  {"Left HP Mixer", "Right DAC1 Switch", "DACR1"},
  {"Left HP Mixer", "Right DAC2 Switch", "DACR2"},
  {"Left HP Mixer", "MIC1 Switch", "MIC1 Input"},
  {"Left HP Mixer", "MIC2 Switch", "MIC2 Input"},
  {"Left HP Mixer", "INA1 Switch", "INA1 Input"},
  {"Left HP Mixer", "INA2 Switch", "INA2 Input"},
  {"Left HP Mixer", "INB1 Switch", "INB1 Input"},
  {"Left HP Mixer", "INB2 Switch", "INB2 Input"},

  /* Right headphone output mixer */
  {"Right HP Mixer", "Left DAC1 Switch", "DACL1"},
  {"Right HP Mixer", "Left DAC2 Switch", "DACL2"  },
  {"Right HP Mixer", "Right DAC1 Switch", "DACR1"},
  {"Right HP Mixer", "Right DAC2 Switch", "DACR2"},
  {"Right HP Mixer", "MIC1 Switch", "MIC1 Input"},
  {"Right HP Mixer", "MIC2 Switch", "MIC2 Input"},
  {"Right HP Mixer", "INA1 Switch", "INA1 Input"},
  {"Right HP Mixer", "INA2 Switch", "INA2 Input"},
  {"Right HP Mixer", "INB1 Switch", "INB1 Input"},
  {"Right HP Mixer", "INB2 Switch", "INB2 Input"},

  /* Left speaker output mixer */
  {"Left SPK Mixer", "Left DAC1 Switch", "DACL1"},
  {"Left SPK Mixer", "Left DAC2 Switch", "DACL2"},
  {"Left SPK Mixer", "Right DAC1 Switch", "DACR1"},
  {"Left SPK Mixer", "Right DAC2 Switch", "DACR2"},
  {"Left SPK Mixer", "MIC1 Switch", "MIC1 Input"},
  {"Left SPK Mixer", "MIC2 Switch", "MIC2 Input"},
  {"Left SPK Mixer", "INA1 Switch", "INA1 Input"},
  {"Left SPK Mixer", "INA2 Switch", "INA2 Input"},
  {"Left SPK Mixer", "INB1 Switch", "INB1 Input"},
  {"Left SPK Mixer", "INB2 Switch", "INB2 Input"},

  /* Right speaker output mixer */
  {"Right SPK Mixer", "Left DAC1 Switch", "DACL1"},
  {"Right SPK Mixer", "Left DAC2 Switch", "DACL2"},
  {"Right SPK Mixer", "Right DAC1 Switch", "DACR1"},
  {"Right SPK Mixer", "Right DAC2 Switch", "DACR2"},
  {"Right SPK Mixer", "MIC1 Switch", "MIC1 Input"},
  {"Right SPK Mixer", "MIC2 Switch", "MIC2 Input"},
  {"Right SPK Mixer", "INA1 Switch", "INA1 Input"},
  {"Right SPK Mixer", "INA2 Switch", "INA2 Input"},
  {"Right SPK Mixer", "INB1 Switch", "INB1 Input"},
  {"Right SPK Mixer", "INB2 Switch", "INB2 Input"},

  /* Earpiece/Receiver output mixer */
  {"Left REC Mixer", "Left DAC1 Switch", "DACL1"},
  {"Left REC Mixer", "Left DAC2 Switch", "DACL2"},
  {"Left REC Mixer", "Right DAC1 Switch", "DACR1"},
  {"Left REC Mixer", "Right DAC2 Switch", "DACR2"},
  {"Left REC Mixer", "MIC1 Switch", "MIC1 Input"},
  {"Left REC Mixer", "MIC2 Switch", "MIC2 Input"},
  {"Left REC Mixer", "INA1 Switch", "INA1 Input"},
  {"Left REC Mixer", "INA2 Switch", "INA2 Input"},
  {"Left REC Mixer", "INB1 Switch", "INB1 Input"},
  {"Left REC Mixer", "INB2 Switch", "INB2 Input"},

  /* Earpiece/Receiver output mixer */
  {"Right REC Mixer", "Left DAC1 Switch", "DACL1"},
  {"Right REC Mixer", "Left DAC2 Switch", "DACL2"},
  {"Right REC Mixer", "Right DAC1 Switch", "DACR1"},
  {"Right REC Mixer", "Right DAC2 Switch", "DACR2"},
  {"Right REC Mixer", "MIC1 Switch", "MIC1 Input"},
  {"Right REC Mixer", "MIC2 Switch", "MIC2 Input"},
  {"Right REC Mixer", "INA1 Switch", "INA1 Input"},
  {"Right REC Mixer", "INA2 Switch", "INA2 Input"},
  {"Right REC Mixer", "INB1 Switch", "INB1 Input"},
  {"Right REC Mixer", "INB2 Switch", "INB2 Input"},

  {"HP Left Out", NULL, "Left HP Mixer"},
  {"HP Right Out", NULL, "Right HP Mixer"},
  {"SPK Left Out", NULL, "Left SPK Mixer"},
  {"SPK Right Out", NULL, "Right SPK Mixer"},
  {"REC Left Out", NULL, "Left REC Mixer"},
  {"REC Right Out", NULL, "Right REC Mixer"},

  {"HPL", NULL, "HP Left Out"},
  {"HPR", NULL, "HP Right Out"},
  {"SPKL", NULL, "SPK Left Out"},
  {"SPKR", NULL, "SPK Right Out"},
  {"RECL", NULL, "REC Left Out"},
  {"RECR", NULL, "REC Right Out"},

  /* Left ADC input mixer */
  {"Left ADC Mixer", "MIC1 Switch", "MIC1 Input"},
  {"Left ADC Mixer", "MIC2 Switch", "MIC2 Input"},
  {"Left ADC Mixer", "INA1 Switch", "INA1 Input"},
  {"Left ADC Mixer", "INA2 Switch", "INA2 Input"},
  {"Left ADC Mixer", "INB1 Switch", "INB1 Input"},
  {"Left ADC Mixer", "INB2 Switch", "INB2 Input"},

  /* Right ADC input mixer */
  {"Right ADC Mixer", "MIC1 Switch", "MIC1 Input"},
  {"Right ADC Mixer", "MIC2 Switch", "MIC2 Input"},
  {"Right ADC Mixer", "INA1 Switch", "INA1 Input"},
  {"Right ADC Mixer", "INA2 Switch", "INA2 Input"},
  {"Right ADC Mixer", "INB1 Switch", "INB1 Input"},
  {"Right ADC Mixer", "INB2 Switch", "INB2 Input"},

  /* Inputs */
  {"ADCL", NULL, "Left ADC Mixer"},
  {"ADCR", NULL, "Right ADC Mixer"},
  {"INA1 Input", NULL, "INA1"},
  {"INA2 Input", NULL, "INA2"},
  {"INB1 Input", NULL, "INB1"},
  {"INB2 Input", NULL, "INB2"},
  {"MIC1 Input", NULL, "MIC1"},
  {"MIC2 Input", NULL, "MIC2"},
};

static int generic_codec_probe(struct snd_soc_codec *codec) {
  return 0;
}

static int generic_codec_remove(struct snd_soc_codec *codec) {
  return 0;
}

static int generic_codec_set_bias_level(struct snd_soc_codec *codec,
                                        enum snd_soc_bias_level level) {
  return 0;
}

static struct snd_soc_codec_driver soc_generic_codec = {
  .probe   = generic_codec_probe,
	.remove  = generic_codec_remove,
	.set_bias_level = generic_codec_set_bias_level,
	.suspend_bias_off = true,

	.component_driver = {
		.dapm_widgets		= generic_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(generic_dapm_widgets),
		.dapm_routes		= intercon,
		.num_dapm_routes	= ARRAY_SIZE(intercon),
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
