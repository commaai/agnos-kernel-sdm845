#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/gpio/consumer.h>
#include <linux/sysfs.h>

struct gpio_som_id {
	struct platform_device *pdev;
  struct pinctrl *pinctrl;
  struct gpio_desc *gpio[4];
};

static ssize_t som_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  struct gpio_som_id *som_id = dev_get_drvdata(dev);
  int i, id = 0;

  if (!som_id)
    goto error;

  for (i=0; i<sizeof(som_id->gpio)/sizeof(som_id->gpio[0]); i++) {
    id |= gpiod_get_value(som_id->gpio[i]) << i;
  }

  return sprintf(buf, "%d\n", id);
error:
  return -EINVAL;
}

static DEVICE_ATTR_RO(som_id);

static struct attribute *som_id_attrs[] = {
  &dev_attr_som_id.attr,
  NULL
};

static struct attribute_group som_id_attr_group = {
  .attrs = som_id_attrs,
};

static int gpio_som_id_probe(struct platform_device *pdev)
{
	struct gpio_som_id *som_id;
	int i, rc = 0;

	som_id = devm_kzalloc(&pdev->dev, sizeof(*som_id), GFP_KERNEL);
	if (!som_id)
		return -ENOMEM;

	som_id->pdev = pdev;

  // you need to set a pinctrl for the GPIOs to have pull-ups
  som_id->pinctrl = devm_pinctrl_get(&pdev->dev);
  if (IS_ERR(som_id->pinctrl)) {
    dev_err(&pdev->dev, "Failed to get pinctrl: %ld\n", PTR_ERR(som_id->pinctrl));
    rc = PTR_ERR(som_id->pinctrl);
    goto error;
  }

  // get the GPIOs
  for (i=0; i<sizeof(som_id->gpio)/sizeof(som_id->gpio[0]); i++) {
    som_id->gpio[i] = devm_gpiod_get_index(&pdev->dev, NULL, i, GPIOD_IN);
    if (IS_ERR(som_id->gpio[i])) {
      dev_err(&pdev->dev, "Failed to get GPIO: %ld\n", PTR_ERR(som_id->gpio[i]));
      rc = PTR_ERR(som_id->gpio[i]);
      goto error;
    }
  }

  // make a sysfs entry for the SOM ID
  rc = sysfs_create_group(&pdev->dev.kobj, &som_id_attr_group);
  if (rc) {
    dev_err(&pdev->dev, "Failed to create sysfs entry: %d\n", rc);
    goto error;
  }

  platform_set_drvdata(pdev, som_id);
error:
	return rc;
}

static int gpio_som_id_remove(struct platform_device *pdev)
{
	struct gpio_som_id *som_id = dev_get_drvdata(&pdev->dev);

  sysfs_remove_group(&pdev->dev.kobj, &som_id_attr_group);
	return 0;
}

static const struct of_device_id of_match_table[] = {
	{ .compatible = "comma,gpio-som-id", },
	{}
};

static struct platform_driver gpio_som_id_driver = {
	.driver		= {
		.name	= "comma,gpio-som-id",
		.of_match_table = of_match_table,
	},
	.probe		= gpio_som_id_probe,
	.remove		= gpio_som_id_remove,
};

module_driver(gpio_som_id_driver, platform_driver_register, platform_driver_unregister);

MODULE_DESCRIPTION("GPIO SOM ID driver");
MODULE_LICENSE("MIT");