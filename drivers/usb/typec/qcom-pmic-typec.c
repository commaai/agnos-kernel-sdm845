// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/role.h>
#include <linux/usb/typec_mux.h>

#define PM8150B_TYPEC_MISC_STATUS		0xb
#define PM8150B_CC_ATTACHED			BIT(0)
#define PM8150B_CC_ORIENTATION			BIT(1)
#define PM8150B_SNK_SRC_MODE			BIT(6)
#define PM8150B_TYPEC_MODE_CFG			0x44
#define PM8150B_TYPEC_DISABLE_CMD		BIT(0)
#define PM8150B_EN_SNK_ONLY			BIT(1)
#define PM8150B_EN_SRC_ONLY			BIT(2)
#define PM8150B_TYPEC_VCONN_CONTROL		0x46
#define PM8150B_VCONN_EN_SRC			BIT(0)
#define PM8150B_VCONN_EN_VAL			BIT(1)
#define PM8150B_TYPEC_EXIT_STATE_CFG		0x50
#define PM8150B_SEL_SRC_UPPER_REF		BIT(2)
#define PM8150B_TYPEC_INTR_EN_CFG_1		0x5e

#define PMI8998_TYPE_C_STATUS_4			0xE
#define PMI8998_CC_ATTACHED			BIT(0)
#define PMI8998_CC_ORIENTATION			BIT(1)
#define PMI8998_UFP_DFP_MODE_STATUS		BIT(7)
#define PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL	0x68
#define PMI8998_TYPEC_DISABLE_CMD		BIT(0)
#define PMI8998_DFP_EN_CMD			BIT(1)
#define PMI8998_UFP_EN_CMD			BIT(2)
#define PMI8998_VCONN_EN_VALUE			BIT(3)
#define PMI8998_VCONN_EN_SRC			BIT(4)
#define PMI8998_TYPE_C_CFG_3_REG		0x5A
#define PMI8998_TVBUS_DEBOUNCE			BIT(7)
#define PMI8998_TYPEC_LEGACY_CABLE_INT_EN	BIT(6)
#define PMI8998_TYPEC_NONCOMPLIANT_LEGACY_CABLE_INT_EN	BIT(5)
#define PMI8998_TYPEC_TRYSOURCE_DETECT_INT_EN	BIT(4)
#define PMI8998_TYPEC_TRYSINK_DETECT_INT_EN	BIT(3)
#define PMI8998_EN_TRYSINK_MODE			BIT(2)
#define PMI8998_EN_LEGACY_CABLE_DETECTION	BIT(1)
#define PMI8998_ALLOW_PD_DRING_UFP_TCCDB	BIT(0)
#define PMI8998_TYPE_C_INTRPT_ENB_REG		0x67

struct register_cfg {
	u16 addr;
	u8 mask;
	u8 val;
};

struct qcom_typec_data {
	struct qcom_typec_status {
		unsigned int addr;
		unsigned char cc_attached_bit;
		unsigned char cc_orientation_bit;
		unsigned char snk_src_mode_bit;
	} status;
	struct qcom_typec_mode {
		unsigned int addr;
		unsigned char disable_bit;
		unsigned char sink_only_bit;
		unsigned char source_only_bit;
	} mode;
	int init_seq_len;
	struct register_cfg init_seq[];

};

struct qcom_pmic_typec {
	struct device		*dev;
	struct regmap		*regmap;
	u32			base;
	const struct qcom_typec_data	*data;

	struct typec_port	*port;
	struct usb_role_switch *role_sw;

	struct regulator	*vbus_reg;
	bool			vbus_enabled;
};

static struct qcom_pmic_typec *qcom_usb_g;

static struct qcom_typec_data pm8150b_data = {
	.status = {
		.addr = PM8150B_TYPEC_MISC_STATUS,
		.cc_attached_bit = PM8150B_CC_ATTACHED,
		.cc_orientation_bit = PM8150B_CC_ORIENTATION,
		.snk_src_mode_bit = PM8150B_SNK_SRC_MODE,
	},
	.mode = {
		.addr = PM8150B_TYPEC_MODE_CFG,
		.disable_bit = PM8150B_TYPEC_DISABLE_CMD,
		.sink_only_bit = PM8150B_EN_SNK_ONLY,
		.source_only_bit = PM8150B_EN_SRC_ONLY,
	},
	.init_seq_len = 3,
	.init_seq = {
		{ .addr = PM8150B_TYPEC_INTR_EN_CFG_1,
		  .mask = GENMASK(7, 0),
		  .val = 0, },
		{ .addr = PM8150B_TYPEC_EXIT_STATE_CFG,
		  .mask = PM8150B_SEL_SRC_UPPER_REF,
		  .val = PM8150B_SEL_SRC_UPPER_REF, },
		{ .addr = PM8150B_TYPEC_VCONN_CONTROL,
		  .mask = PM8150B_VCONN_EN_SRC | PM8150B_VCONN_EN_VAL,
		  .val = PM8150B_VCONN_EN_SRC, },
	},
};

static struct qcom_typec_data pmi8998_data = {
	.status = {
		.addr = PMI8998_TYPE_C_STATUS_4,
		.cc_attached_bit = PMI8998_CC_ATTACHED,
		.cc_orientation_bit = PMI8998_CC_ORIENTATION,
		.snk_src_mode_bit = PMI8998_UFP_DFP_MODE_STATUS,
	},
	.mode = {
		.addr = PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL,
		.disable_bit = PMI8998_TYPEC_DISABLE_CMD,
		// TODO: are these in the wrong order??
		.sink_only_bit = PMI8998_UFP_EN_CMD,
		.source_only_bit = PMI8998_DFP_EN_CMD,
	},
	.init_seq_len = 3,
	.init_seq = {
		{ .addr = PMI8998_TYPE_C_INTRPT_ENB_REG, .mask = GENMASK(7, 0),
			.val = 0, },
		{ .addr = PMI8998_TYPE_C_CFG_3_REG,
		  .mask = PMI8998_EN_TRYSINK_MODE |
			  PMI8998_TYPEC_NONCOMPLIANT_LEGACY_CABLE_INT_EN |
			  PMI8998_TYPEC_LEGACY_CABLE_INT_EN,
		.val = 0, },
		{ .addr = PMI8998_TYPE_C_INTRPT_ENB_SOFTWARE_CTRL,
		  .mask = PMI8998_VCONN_EN_SRC | PMI8998_VCONN_EN_VALUE,
		  .val = PMI8998_VCONN_EN_SRC, },
	},
};

static void qcom_pmic_typec_enable_vbus_regulator(struct qcom_pmic_typec
							*qcom_usb, bool enable)
{
	int ret;

	if (enable == qcom_usb->vbus_enabled)
		return;

	if (enable) {
		ret = regulator_enable(qcom_usb->vbus_reg);
		if (ret)
			return;
	} else {
		ret = regulator_disable(qcom_usb->vbus_reg);
		if (ret)
			return;
	}
	qcom_usb->vbus_enabled = enable;
}

void qcom_pmic_typec_check_connection(struct qcom_pmic_typec *qcom_usb)
{
	enum typec_orientation orientation;
	enum usb_role role;
	unsigned int stat;
	bool enable_vbus;
	const struct qcom_typec_status *stat_reg;
	
	if (!qcom_usb)
		qcom_usb = qcom_usb_g;

	dev_info(qcom_usb->dev, "%s\n", __func__);

	stat_reg = &qcom_usb->data->status;

	regmap_read(qcom_usb->regmap, qcom_usb->base + stat_reg->addr,
		    &stat);

	if (stat & stat_reg->cc_attached_bit) {
		orientation = (stat & stat_reg->cc_orientation_bit) ?
				TYPEC_ORIENTATION_REVERSE :
				TYPEC_ORIENTATION_NORMAL;
		typec_set_orientation(qcom_usb->port, orientation);

		role = (stat & stat_reg->snk_src_mode_bit) ? USB_ROLE_HOST : USB_ROLE_DEVICE;
		if (role == USB_ROLE_HOST)
			enable_vbus = true;
		else
			enable_vbus = false;
	} else {
		role = USB_ROLE_NONE;
		enable_vbus = false;
	}

	qcom_pmic_typec_enable_vbus_regulator(qcom_usb, enable_vbus);
	if (IS_ERR(qcom_usb->role_sw) && PTR_ERR(qcom_usb->role_sw) == -EPROBE_DEFER)
		qcom_usb->role_sw = fwnode_usb_role_switch_get(dev_fwnode(qcom_usb->dev));

	usb_role_switch_set_role(qcom_usb->role_sw, role);
}
EXPORT_SYMBOL_GPL(qcom_pmic_typec_check_connection);

// TODO: what IRQ should this be???
static irqreturn_t qcom_pmic_typec_interrupt(int irq, void *_qcom_usb)
{
	struct qcom_pmic_typec *qcom_usb = _qcom_usb;

	pr_info("TYPEC IRQ!\n");

	qcom_pmic_typec_check_connection(qcom_usb);
	return IRQ_HANDLED;
}

static void qcom_pmic_typec_typec_hw_init(struct qcom_pmic_typec *qcom_usb,
					  enum typec_port_type type)
{
	u8 mode = 0;
	int i = 0;
	const struct qcom_typec_mode *mode_reg = &qcom_usb->data->mode;

	if (type == TYPEC_PORT_SRC)
		mode = mode_reg->source_only_bit;
	else if (type == TYPEC_PORT_SNK)
		mode = mode_reg->sink_only_bit;
	
	regmap_update_bits(qcom_usb->regmap, qcom_usb->base + mode_reg->addr,
			   mode_reg->source_only_bit | mode_reg->sink_only_bit,
			   mode);

	for (i = 0; i < qcom_usb->data->init_seq_len; i++) {
		regmap_update_bits(qcom_usb->regmap,
				   qcom_usb->base + qcom_usb->data->init_seq[i].addr,
				   qcom_usb->data->init_seq[i].mask,
				   qcom_usb->data->init_seq[i].val);
	}
}

static int qcom_pmic_typec_probe(struct platform_device *pdev)
{
	struct qcom_pmic_typec *qcom_usb;
	struct device *dev = &pdev->dev;
	struct fwnode_handle *fwnode;
	struct typec_capability cap;
	const char *buf;
	int ret, irq, role;
	u32 reg;

	dev_info(dev, "Start probe!\n");

	ret = device_property_read_u32(dev, "reg", &reg);
	if (ret < 0) {
		dev_err(dev, "missing base address\n");
		return ret;
	}

	qcom_usb = devm_kzalloc(dev, sizeof(*qcom_usb), GFP_KERNEL);
	if (!qcom_usb)
		return -ENOMEM;

	qcom_usb->dev = dev;
	qcom_usb->base = reg;

	qcom_usb->data = device_get_match_data(dev);

	qcom_usb->regmap = dev_get_regmap(dev->parent, NULL);
	if (!qcom_usb->regmap) {
		dev_err(dev, "Failed to get regmap\n");
		return -EINVAL;
	}

	qcom_usb->vbus_reg = devm_regulator_get(qcom_usb->dev, "usb_vbus");
	if (IS_ERR(qcom_usb->vbus_reg))
		return PTR_ERR(qcom_usb->vbus_reg);

	fwnode = device_get_named_child_node(dev, "connector");
	if (!fwnode)
		return -EINVAL;

	ret = fwnode_property_read_string(fwnode, "power-role", &buf);
	if (!ret) {
		role = typec_find_port_power_role(buf);
		if (role < 0)
			role = TYPEC_PORT_SNK;
	} else {
		role = TYPEC_PORT_SNK;
	}
	cap.type = role;

	ret = fwnode_property_read_string(fwnode, "data-role", &buf);
	if (!ret) {
		role = typec_find_port_data_role(buf);
		if (role < 0)
			role = TYPEC_PORT_UFP;
	} else {
		role = TYPEC_PORT_UFP;
	}
	cap.data = role;

	cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
	cap.fwnode = fwnode;
	qcom_usb->port = typec_register_port(dev, &cap);
	if (IS_ERR(qcom_usb->port)) {
		ret = PTR_ERR(qcom_usb->port);
		dev_err(dev, "Failed to register type c port %d\n", ret);
		goto err_put_node;
	}
	fwnode_handle_put(fwnode);

	dev_info(dev, "Before fwnode_usb_role_switch_get\n");
	qcom_usb->role_sw = fwnode_usb_role_switch_get(dev_fwnode(qcom_usb->dev));
	if (IS_ERR(qcom_usb->role_sw) && PTR_ERR(qcom_usb->role_sw) != -EPROBE_DEFER) {
		dev_err(dev, "failed to get role switch\n");
		ret = PTR_ERR(qcom_usb->role_sw);
		goto err_typec_port;
	}

	dev_info(dev, "Before platform_get_irq\n");
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		goto err_usb_role_sw;

	dev_info(dev, "Before devm_request_threaded_irq\n");
	ret = devm_request_threaded_irq(qcom_usb->dev, irq, NULL,
					qcom_pmic_typec_interrupt, IRQF_ONESHOT,
					"qcom-pmic-typec", qcom_usb);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		goto err_usb_role_sw;
	}

	platform_set_drvdata(pdev, qcom_usb);
	qcom_pmic_typec_typec_hw_init(qcom_usb, cap.type);
	qcom_pmic_typec_check_connection(qcom_usb);

	dev_info(qcom_usb->dev, "Probed!\n");
	qcom_usb_g = qcom_usb;

	return 0;

err_usb_role_sw:
	usb_role_switch_put(qcom_usb->role_sw);
err_typec_port:
	typec_unregister_port(qcom_usb->port);
err_put_node:
	fwnode_handle_put(fwnode);

	return ret;
}

static int qcom_pmic_typec_remove(struct platform_device *pdev)
{
	struct qcom_pmic_typec *qcom_usb = platform_get_drvdata(pdev);

	usb_role_switch_set_role(qcom_usb->role_sw, USB_ROLE_NONE);
	qcom_pmic_typec_enable_vbus_regulator(qcom_usb, 0);

	typec_unregister_port(qcom_usb->port);
	usb_role_switch_put(qcom_usb->role_sw);

	return 0;
}

static const struct of_device_id qcom_pmic_typec_table[] = {
	{ .compatible = "qcom,pm8150b-usb-typec", .data = &pm8150b_data },
	{ .compatible = "qcom,pmi8998-usb-typec", .data = &pmi8998_data },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_pmic_typec_table);

static struct platform_driver qcom_pmic_typec = {
	.driver = {
		.name = "qcom,pmic-typec",
		.of_match_table = qcom_pmic_typec_table,
	},
	.probe = qcom_pmic_typec_probe,
	.remove = qcom_pmic_typec_remove,
};
module_platform_driver(qcom_pmic_typec);

MODULE_DESCRIPTION("QCOM PMIC USB type C driver");
MODULE_LICENSE("GPL v2");
