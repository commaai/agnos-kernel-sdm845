#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of.h>

static int panda_spi_probe(struct spi_device *spi)
{
    dev_info(&spi->dev, "Panda SPI device probed\n");
    return 0;
}

static int panda_spi_remove(struct spi_device *spi)
{
    dev_info(&spi->dev, "Panda SPI device removed\n");
    return 0;
}

static const struct of_device_id panda_spi_of_match[] = {
    { .compatible = "commaai,panda" },
    {}
};
MODULE_DEVICE_TABLE(of, panda_spi_of_match);

static struct spi_driver panda_spi_driver = {
    .driver = {
        .name = "panda-spi",
        .of_match_table = panda_spi_of_match,
    },
    .probe = panda_spi_probe,
    .remove = panda_spi_remove,
};

module_spi_driver(panda_spi_driver);

MODULE_LICENSE("MIT");
MODULE_AUTHOR("Comma.ai");
MODULE_DESCRIPTION("SPI protocol driver for Comma.ai Panda");