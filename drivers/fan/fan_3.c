/************************************
speed node path:
/sys/devices/soc/800f000.qcom,spmi/spmi-0/spmi0-02/800f000.qcom,spmi:qcom,pmi8998@2:qcom,fan@c400/
************************************/

#include <linux/spmi.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/pwm.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define GPIO_MODE_CTL			0xc440
#define GPIO_DIG_OUT_SOURCE_CTL 	0xc444
#define GPIO_EN_CTL			0xc446
#define DEFAULT_DUTY_NS			0
#define DEFAULT_PERIOD_NS		40000

static struct kobject   *fan_kobj;
static struct fan_dev *fan = NULL;

struct fan_dev {
	struct platform_device		*pdev;
	struct regmap			*regmap;
	struct pwm_device		*fan_pwm_device;
};

static int fan_config2(struct fan_dev *fan)
{
	int rc = 0;
	u8 reg = 0;
	struct device_node *node = fan->pdev->dev.of_node;

	// configure gpio mode
	reg = 0x01;
	rc = regmap_write(fan->regmap, GPIO_MODE_CTL, reg);
	if(rc) {
		pr_err("%s: write %s failed. rc = %d.\n", __func__, "GPIO_MODE_CTL", rc);
		return rc;
	}

	reg = 0x02;
	rc = regmap_write(fan->regmap, GPIO_DIG_OUT_SOURCE_CTL, reg);
	if(rc) {
		pr_err("%s: write %s failed. rc = %d.\n", __func__, "GPIO_DIG_OUT_SOURCE_CTL", rc);
		return rc;
	}

	reg = 0x80;
	rc = regmap_write(fan->regmap, GPIO_EN_CTL, reg);
	if(rc) {
		pr_err("%s: write %s failed. rc = %d.\n", __func__, "GPIO_EN_CTL", rc);
		return rc;
	}

	// configure pwm device
	fan->fan_pwm_device = of_pwm_get(node, NULL);
	if(IS_ERR(fan->fan_pwm_device)) {
		rc = PTR_ERR(fan->fan_pwm_device);
		pr_err("%s: failed to get pwm device.\n", __func__);
		fan->fan_pwm_device = NULL;
		return rc;
	}

	pwm_config(fan->fan_pwm_device, DEFAULT_DUTY_NS, DEFAULT_PERIOD_NS);
	pwm_enable(fan->fan_pwm_device);

	return rc;
}

static ssize_t fan_speed_adjust(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int rc = 0;
	unsigned long speed_level;
	int duty_ns = DEFAULT_DUTY_NS, period_ns = DEFAULT_PERIOD_NS;

	if(NULL == fan) {
		pr_err("%s: invalid fan device.\n", __func__);
		return -1;
	}

	rc = kstrtoul(buf, 10, &speed_level);
	switch(speed_level) {
		case 0:
			duty_ns = 0;
			break;
		case 1:
			duty_ns = 1000;
			break;
		case 2:
			duty_ns = 10000;
			break;
		case 3:
			duty_ns = 40000;
			break;
		default:
			pr_err("%s: wrong level. (level 0 - 3)\n", __func__);
			return len;
	}

	pwm_disable(fan->fan_pwm_device);
	pwm_config(fan->fan_pwm_device, duty_ns, period_ns);
	pwm_enable(fan->fan_pwm_device);

	return len;
}

static DEVICE_ATTR(speed, 0664, NULL, fan_speed_adjust);

static struct attribute *fan_attrs[] = {
	&dev_attr_speed.attr,
	NULL,
};

static struct attribute_group fan_attr_group = {
	.attrs = fan_attrs,
};

static int fan_probe(struct platform_device *pdev)
{
	int rc = 0;

	fan = devm_kzalloc(&pdev->dev, sizeof(*fan), GFP_KERNEL);
	if(IS_ERR(fan)) {
		pr_err("%s: can't alloc memery for fan device.\n", __func__);
		return -1;
	}

	fan->regmap = dev_get_regmap(pdev->dev.parent, NULL);

	dev_set_drvdata(&pdev->dev, fan);
	fan->pdev = pdev;

	rc = fan_config2(fan);
	if(rc) {
		pr_err("%s: fan config2 failed.\n", __func__);
		goto fail;
	}

	fan_kobj = kobject_create_and_add("fan", kernel_kobj);
	if (!fan_kobj){
		dev_err(&pdev->dev, "kobject fan ENOMEM\n");
		goto fail;
	}

	rc = sysfs_create_group(&pdev->dev.kobj, &fan_attr_group);
	rc = sysfs_create_group(fan_kobj, &fan_attr_group);
	if(rc) {
		pr_err("%s: sysfs create failed. rc = %d.\n", __func__, rc);
		goto fail;
	}

	pr_info("%s: fan_probe done.\n", __func__);
	return 0;

fail:
	return rc;
}

static struct of_device_id fan_match_table[] = {
	{
		.compatible = "qcom,qpnp-fan",
	},
	{},
};

static struct platform_driver fan_driver = {
	.driver = {
		.name = "fan_driver",
		.owner = THIS_MODULE,
		.of_match_table = fan_match_table,
	},
	.probe = fan_probe,
};

static int __init fan_init(void)
{
	return platform_driver_register(&fan_driver);
}

static void __exit fan_exit(void)
{
	return platform_driver_unregister(&fan_driver);
}

module_init(fan_init);
module_exit(fan_exit);
MODULE_LICENSE("GPL v2");
