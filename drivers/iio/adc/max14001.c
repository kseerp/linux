// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices MAX14001 Peripheral Module ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 */
#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/bitrev.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/types.h>

/* MAX14001 Registers Address */
#define MAX14001_ADC		0x00
#define MAX14001_FADC		0x01
#define MAX14001_FLAGS		0x02
#define MAX14001_FLTEN		0x03
#define MAX14001_THL		0x04
#define MAX14001_THU		0x05
#define MAX14001_INRR		0x06
#define MAX14001_INRT		0x07
#define MAX14001_INRP		0x08
#define MAX14001_CFG		0x09
#define MAX14001_ENBL		0x0A
#define MAX14001_ACT		0x0B
#define MAX14001_WEN		0x0C
#define MAX14001_FLTV		0x13
#define MAX14001_THLV		0x14
#define MAX14001_THUV		0x15
#define MAX14001_INRRV		0x16
#define MAX14001_INRTV		0x17
#define MAX14001_INRPV		0x18
#define MAX14001_CFGV		0x19
#define MAX14001_ENBLV		0x1A

#define IIO_DMA_MINALIGN	ARCH_KMALLOC_MINALIGN

#define MAX14001_LSB_DIV	BIT(10)
#define MAX14001_SET_WRITE_BIT	BIT(10)

#define MAX14001_WRITE_WEN	0x294
#define MAX14001_DATA_MASK	GENMASK(9, 0)

#define MAX14001_REG_ADDR_MASK(reg_addr) \
	FIELD_PREP(GENMASK(15, 11), (u16)(reg_addr))

#define MAX14001_READ_REG(reg_addr) \
	bitrev16(MAX14001_REG_ADDR_MASK(reg_addr))

#define MAX14001_WRITE_REG(reg_addr, data) \
	bitrev16((u16)(MAX14001_REG_ADDR_MASK(reg_addr) | \
	((u16)data) | MAX14001_SET_WRITE_BIT))

struct max14001_state {
	struct spi_device *spi;
	struct regulator *vref;
	/* lock protect agains multiple concurrent accesses */
	struct mutex lock;
	int vref_mv;
	bool use_fadc;
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	__be16 spi_tx_buffer __aligned(IIO_DMA_MINALIGN);
	u16 spi_rx_buffer;
};

static const struct iio_chan_spec max14001_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) |
		BIT(IIO_CHAN_INFO_OFFSET),
	}
};

static int max14001_read(struct max14001_state *st,
			 unsigned int reg_addr, int *val)
{
	int ret;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = &st->spi_tx_buffer,
			.len = 2,
			.cs_change = 1,
		}, {
			.rx_buf = &st->spi_rx_buffer,
			.len = 2,
		},
	};

	put_unaligned_be16(MAX14001_READ_REG(reg_addr),
			   &st->spi_tx_buffer);

	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
	if (ret)
		return ret;

	*val = bitrev16(get_unaligned_be16(&st->spi_rx_buffer));

	return 0;
}

static int max14001_write(struct max14001_state *st,
			  unsigned int reg_addr, int data)
{
	put_unaligned_be16(MAX14001_WRITE_REG(reg_addr, data),
			   &st->spi_tx_buffer);

	return spi_write(st->spi, &st->spi_tx_buffer, 2);
}

static void max14001_regulator_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

static int max14001_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct max14001_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = max14001_read(st, st->use_fadc ?
				    MAX14001_FADC : MAX14001_ADC, val);
		mutex_unlock(&st->lock);
		if (ret < 0)
			return ret;

		*val &= MAX14001_DATA_MASK;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv / 1000;
		*val2 = MAX14001_LSB_DIV;

		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_OFFSET:
		*val = (st->vref_mv / 1000) / 2;

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static const struct iio_info max14001_info = {
	.read_raw = max14001_read_raw,
};

static int max14001_cfg_reg(struct max14001_state *st)
{
	struct device_node *np = st->spi->dev.of_node;
	int reg_cfg;
	int filter;
	int ret;

	ret = max14001_read(st, MAX14001_CFG, &reg_cfg);
	if (ret)
		return ret;

	if (of_property_read_bool(np, "use-fadc"))
		st->use_fadc = true;
	else
		st->use_fadc = false;

	if (of_property_read_bool(np, "inrush-mode"))
		reg_cfg |= BIT(1);
	else
		reg_cfg &= ~BIT(1);

	if (of_property_read_u32(np, "filter", &filter)) {
		dev_err(&st->spi->dev, "Failed to get filter property");
		return -EINVAL;
	}

	switch (filter) {
	case 0:
		reg_cfg &= BIT(2) & BIT(3);
		break;

	case 1:
		reg_cfg &= BIT(2);
		reg_cfg |= BIT(3);
		break;

	case 2:
		reg_cfg |= BIT(2);
		reg_cfg &= BIT(3);
		break;

	case 3:
		reg_cfg |= BIT(2) | BIT(3);
		break;
	}

	if (of_property_read_bool(np, "current-source"))
		reg_cfg |= BIT(4);
	else
		reg_cfg &= ~BIT(4);

	/* Enable SPI Registers Write */
	ret = max14001_write(st, MAX14001_WEN, MAX14001_WRITE_WEN);
	if (ret)
		return ret;

	return max14001_write(st, MAX14001_CFG, reg_cfg);
}

static int max14001_config_init(struct max14001_state *st)
{
	int i, data;
	int reg_flten;
	int reg_fltv;
	int ret;

	/* Enable SPI Registers Write */
	ret = max14001_write(st, MAX14001_WEN, MAX14001_WRITE_WEN);
	if (ret)
		return ret;

	/* Power on reset */
	ret = max14001_write(st, MAX14001_ACT, BIT(7));
	if (ret)
		return ret;

	/* Disable SPI Registers Write */
	ret = max14001_write(st, MAX14001_WEN, 0x000);
	if (ret)
		return ret;

	msleep(100);

	/* Read FLAGS */
	ret = max14001_read(st, MAX14001_FLAGS, &data);
	if (ret)
		return ret;

	/* Enable SPI Registers Write */
	max14001_write(st, MAX14001_WEN, MAX14001_WRITE_WEN);
	if (ret)
		return ret;

	/* Clear Faults */
	for (i = MAX14001_FLTEN; i <= MAX14001_ACT; i++) {
		max14001_write(st, i, 0x000);
		if (ret)
			return ret;
	}

	for (i = MAX14001_FLTV; i <= MAX14001_ENBLV; i++) {
		max14001_write(st, i, 0x000);
		if (ret)
			return ret;
	}

	/* read FLTEN and FLTV and store on local variables */
	max14001_read(st, MAX14001_FLTEN, &reg_flten);
	if (ret)
		return ret;

	max14001_read(st, MAX14001_FLTV, &reg_fltv);
	if (ret)
		return ret;

	/* Enable Memory Validation Flag */
	max14001_write(st, MAX14001_FLTEN, reg_flten | BIT(7));
	if (ret)
		return ret;

	max14001_write(st, MAX14001_FLTV, reg_fltv | BIT(7));
	if (ret)
		return ret;

	/* Disable SPI Registers Write */
	max14001_write(st, MAX14001_WEN, 0x000);
	if (ret)
		return ret;

	/* Read FLAGS */
	max14001_read(st, MAX14001_FLAGS, &data);
	if (ret)
		return ret;


	ret = max14001_cfg_reg(st);
	if (ret)
		return ret;

	/* Enable field side current sink */
	max14001_write(st, MAX14001_ENBL, 0x010);
	if (ret)
		return ret;

	max14001_write(st, MAX14001_ENBLV, 0x010);
	if (ret)
		return ret;

	/* Read FLAGS */
	ret = max14001_read(st, MAX14001_FLAGS, &data);
	if (ret)
		return ret;

	/* Disable SPI Registers Write */
	return max14001_write(st, MAX14001_WEN, 0x000);
}

static int max14001_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct max14001_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &max14001_info;
	indio_dev->channels = max14001_channels;
	indio_dev->num_channels = ARRAY_SIZE(max14001_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	st->vref = devm_regulator_get_optional(&spi->dev, "vref");
	if (IS_ERR(st->vref)) {
		if (PTR_ERR(st->vref) != -ENODEV)
			return dev_err_probe(&spi->dev, PTR_ERR(st->vref),
					     "Failed to get vref regulator");

		/* internal reference */
		st->vref_mv = 1250;
	} else {
		ret = regulator_enable(st->vref);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
					     "Failed to enable vref regulators\n");

		ret = devm_add_action_or_reset(&spi->dev,
					       max14001_regulator_disable,
					       st->vref);
		if (ret)
			return ret;

		ret = regulator_get_voltage(st->vref);
		if (ret < 0)
			return dev_err_probe(&spi->dev, ret,
					     "Failed to get vref\n");

		st->vref_mv = ret / 1000;
	}

	ret = max14001_config_init(st);
	if (ret)
		return ret;

	mutex_init(&st->lock);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id max14001_id[] = {
	{ "max14001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, max14001_id);

static const struct of_device_id max14001_of_match[] = {
	{ .compatible = "adi,max14001" },
	{ }
};
MODULE_DEVICE_TABLE(of, max14001_of_match);

static struct spi_driver max14001_driver = {
	.driver = {
		.name = "max14001",
		.of_match_table = max14001_of_match,
	},
	.probe = max14001_probe,
	.id_table = max14001_id,
};
module_spi_driver(max14001_driver);

MODULE_AUTHOR("Kim Seer Paller");
MODULE_DESCRIPTION("MAX14001 ADC driver");
MODULE_LICENSE("GPL v2");
