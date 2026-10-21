// SPDX-License-Identifier: GPL-2.0
// CAN bus driver for Bosch M_CAN controller
// Copyright (C) 2014 Freescale Semiconductor, Inc.
//      Dong Aisheng <b29396@freescale.com>
// Copyright (C) 2018-19 Texas Instruments Incorporated - http://www.ti.com/

/* Bosch M_CAN user manual can be obtained from:
 * http://www.bosch-semiconductors.de/media/pdf_1/ipmodules_1/m_can/
 * mcan_users_manual_v302.pdf
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/iopoll.h>
#include <linux/can/dev.h>
#include <linux/pinctrl/consumer.h>

#include "m_can.h"

/* registers definition */
enum m_can_reg {
	M_CAN_CREL	= 0x0,
	M_CAN_ENDN	= 0x4,
	M_CAN_CUST	= 0x8,
	M_CAN_DBTP	= 0xc,
	M_CAN_TEST	= 0x10,
	M_CAN_RWD	= 0x14,
	M_CAN_CCCR	= 0x18,
	M_CAN_NBTP	= 0x1c,
	M_CAN_TSCC	= 0x20,
	M_CAN_TSCV	= 0x24,
	M_CAN_TOCC	= 0x28,
	M_CAN_TOCV	= 0x2c,
	M_CAN_ECR	= 0x40,
	M_CAN_PSR	= 0x44,
/* TDCR Register only available for version >=3.1.x */
	M_CAN_TDCR	= 0x48,
	M_CAN_IR	= 0x50,
	M_CAN_IE	= 0x54,
	M_CAN_ILS	= 0x58,
	M_CAN_ILE	= 0x5c,
	M_CAN_GFC	= 0x80,
	M_CAN_SIDFC	= 0x84,
	M_CAN_XIDFC	= 0x88,
	M_CAN_XIDAM	= 0x90,
	M_CAN_HPMS	= 0x94,
	M_CAN_NDAT1	= 0x98,
	M_CAN_NDAT2	= 0x9c,
	M_CAN_RXF0C	= 0xa0,
	M_CAN_RXF0S	= 0xa4,
	M_CAN_RXF0A	= 0xa8,
	M_CAN_RXBC	= 0xac,
	M_CAN_RXF1C	= 0xb0,
	M_CAN_RXF1S	= 0xb4,
	M_CAN_RXF1A	= 0xb8,
	M_CAN_RXESC	= 0xbc,
	M_CAN_TXBC	= 0xc0,
	M_CAN_TXFQS	= 0xc4,
	M_CAN_TXESC	= 0xc8,
	M_CAN_TXBRP	= 0xcc,
	M_CAN_TXBAR	= 0xd0,
	M_CAN_TXBCR	= 0xd4,
	M_CAN_TXBTO	= 0xd8,
	M_CAN_TXBCF	= 0xdc,
	M_CAN_TXBTIE	= 0xe0,
	M_CAN_TXBCIE	= 0xe4,
	M_CAN_TXEFC	= 0xf0,
	M_CAN_TXEFS	= 0xf4,
	M_CAN_TXEFA	= 0xf8,
};

/* napi related */
#define M_CAN_NAPI_WEIGHT	64

/* Core Release Register (CREL) */
#define CREL_REL_SHIFT		28
#define CREL_REL_MASK		(0xF << CREL_REL_SHIFT)
#define CREL_STEP_SHIFT		24
#define CREL_STEP_MASK		(0xF << CREL_STEP_SHIFT)
#define CREL_SUBSTEP_SHIFT	20
#define CREL_SUBSTEP_MASK	(0xF << CREL_SUBSTEP_SHIFT)

/* Data Bit Timing & Prescaler Register (DBTP) */
#define DBTP_TDC		BIT(23)
#define DBTP_DBRP_SHIFT		16
#define DBTP_DBRP_MASK		(0x1f << DBTP_DBRP_SHIFT)
#define DBTP_DTSEG1_SHIFT	8
#define DBTP_DTSEG1_MASK	(0x1f << DBTP_DTSEG1_SHIFT)
#define DBTP_DTSEG2_SHIFT	4
#define DBTP_DTSEG2_MASK	(0xf << DBTP_DTSEG2_SHIFT)
#define DBTP_DSJW_SHIFT		0
#define DBTP_DSJW_MASK		(0xf << DBTP_DSJW_SHIFT)

/* Transmitter Delay Compensation Register (TDCR) */
#define TDCR_TDCO_SHIFT		8
#define TDCR_TDCO_MASK		(0x7F << TDCR_TDCO_SHIFT)
#define TDCR_TDCF_SHIFT		0
#define TDCR_TDCF_MASK		(0x7F << TDCR_TDCF_SHIFT)

/* Test Register (TEST) */
#define TEST_LBCK		BIT(4)

/* CC Control Register(CCCR) */
#define CCCR_CMR_MASK		0x3
#define CCCR_CMR_SHIFT		10
#define CCCR_CMR_CANFD		0x1
#define CCCR_CMR_CANFD_BRS	0x2
#define CCCR_CMR_CAN		0x3
#define CCCR_CME_MASK		0x3
#define CCCR_CME_SHIFT		8
#define CCCR_CME_CAN		0
#define CCCR_CME_CANFD		0x1
#define CCCR_CME_CANFD_BRS	0x2
#define CCCR_TXP		BIT(14)
#define CCCR_TEST		BIT(7)
#define CCCR_MON		BIT(5)
#define CCCR_CSR		BIT(4)
#define CCCR_CSA		BIT(3)
#define CCCR_ASM		BIT(2)
#define CCCR_CCE		BIT(1)
#define CCCR_INIT		BIT(0)
#define CCCR_CANFD		0x10
/* for version >=3.1.x */
#define CCCR_EFBI		BIT(13)
#define CCCR_PXHD		BIT(12)
#define CCCR_BRSE		BIT(9)
#define CCCR_FDOE		BIT(8)
/* only for version >=3.2.x */
#define CCCR_NISO		BIT(15)

/* Nominal Bit Timing & Prescaler Register (NBTP) */
#define NBTP_NSJW_SHIFT		25
#define NBTP_NSJW_MASK		(0x7f << NBTP_NSJW_SHIFT)
#define NBTP_NBRP_SHIFT		16
#define NBTP_NBRP_MASK		(0x1ff << NBTP_NBRP_SHIFT)
#define NBTP_NTSEG1_SHIFT	8
#define NBTP_NTSEG1_MASK	(0xff << NBTP_NTSEG1_SHIFT)
#define NBTP_NTSEG2_SHIFT	0
#define NBTP_NTSEG2_MASK	(0x7f << NBTP_NTSEG2_SHIFT)

/* Error Counter Register(ECR) */
#define ECR_RP			BIT(15)
#define ECR_REC_SHIFT		8
#define ECR_REC_MASK		(0x7f << ECR_REC_SHIFT)
#define ECR_TEC_SHIFT		0
#define ECR_TEC_MASK		0xff

/* Protocol Status Register(PSR) */
#define PSR_BO		BIT(7)
#define PSR_EW		BIT(6)
#define PSR_EP		BIT(5)
#define PSR_LEC_MASK	0x7

/* Interrupt Register(IR) */
#define IR_ALL_INT	0xffffffff

/* Renamed bits for versions > 3.1.x */
#define IR_ARA		BIT(29)
#define IR_PED		BIT(28)
#define IR_PEA		BIT(27)

/* Bits for version 3.0.x */
#define IR_STE		BIT(31)
#define IR_FOE		BIT(30)
#define IR_ACKE		BIT(29)
#define IR_BE		BIT(28)
#define IR_CRCE		BIT(27)
#define IR_WDI		BIT(26)
#define IR_BO		BIT(25)
#define IR_EW		BIT(24)
#define IR_EP		BIT(23)
#define IR_ELO		BIT(22)
#define IR_BEU		BIT(21)
#define IR_BEC		BIT(20)
#define IR_DRX		BIT(19)
#define IR_TOO		BIT(18)
#define IR_MRAF		BIT(17)
#define IR_TSW		BIT(16)
#define IR_TEFL		BIT(15)
#define IR_TEFF		BIT(14)
#define IR_TEFW		BIT(13)
#define IR_TEFN		BIT(12)
#define IR_TFE		BIT(11)
#define IR_TCF		BIT(10)
#define IR_TC		BIT(9)
#define IR_HPM		BIT(8)
#define IR_RF1L		BIT(7)
#define IR_RF1F		BIT(6)
#define IR_RF1W		BIT(5)
#define IR_RF1N		BIT(4)
#define IR_RF0L		BIT(3)
#define IR_RF0F		BIT(2)
#define IR_RF0W		BIT(1)
#define IR_RF0N		BIT(0)
#define IR_ERR_STATE	(IR_BO | IR_EW | IR_EP)

/* Interrupts for version 3.0.x */
#define IR_ERR_LEC_30X	(IR_STE	| IR_FOE | IR_ACKE | IR_BE | IR_CRCE)
#define IR_ERR_BUS_30X	(IR_ERR_LEC_30X | IR_WDI | IR_ELO | IR_BEU | \
			 IR_BEC | IR_TOO | IR_MRAF | IR_TSW | IR_TEFL | \
			 IR_RF1L | IR_RF0L)
#define IR_ERR_ALL_30X	(IR_ERR_STATE | IR_ERR_BUS_30X)
/* Interrupts for version >= 3.1.x */
#define IR_ERR_LEC_31X	(IR_PED | IR_PEA)
#define IR_ERR_BUS_31X      (IR_ERR_LEC_31X | IR_WDI | IR_ELO | IR_BEU | \
			 IR_BEC | IR_TOO | IR_MRAF | IR_TSW | IR_TEFL | \
			 IR_RF1L | IR_RF0L)
#define IR_ERR_ALL_31X	(IR_ERR_STATE | IR_ERR_BUS_31X)

/* Interrupt Line Select (ILS) */
#define ILS_ALL_INT0	0x0
#define ILS_ALL_INT1	0xFFFFFFFF

/* Interrupt Line Enable (ILE) */
#define ILE_EINT1	BIT(1)
#define ILE_EINT0	BIT(0)

/* Rx FIFO 0/1 Configuration (RXF0C/RXF1C) */
#define RXFC_FWM_SHIFT	24
#define RXFC_FWM_MASK	(0x7f << RXFC_FWM_SHIFT)
#define RXFC_FS_SHIFT	16
#define RXFC_FS_MASK	(0x7f << RXFC_FS_SHIFT)

/* Rx FIFO 0/1 Status (RXF0S/RXF1S) */
#define RXFS_RFL	BIT(25)
#define RXFS_FF		BIT(24)
#define RXFS_FPI_SHIFT	16
#define RXFS_FPI_MASK	0x3f0000
#define RXFS_FGI_SHIFT	8
#define RXFS_FGI_MASK	0x3f00
#define RXFS_FFL_MASK	0x7f

/* Rx Buffer / FIFO Element Size Configuration (RXESC) */
#define M_CAN_RXESC_8BYTES	0x0
#define M_CAN_RXESC_64BYTES	0x777

/* Tx Buffer Configuration(TXBC) */
#define TXBC_NDTB_SHIFT		16
#define TXBC_NDTB_MASK		(0x3f << TXBC_NDTB_SHIFT)
#define TXBC_TFQS_SHIFT		24
#define TXBC_TFQS_MASK		(0x3f << TXBC_TFQS_SHIFT)

/* Tx FIFO/Queue Status (TXFQS) */
#define TXFQS_TFQF		BIT(21)
#define TXFQS_TFQPI_SHIFT	16
#define TXFQS_TFQPI_MASK	(0x1f << TXFQS_TFQPI_SHIFT)
#define TXFQS_TFGI_SHIFT	8
#define TXFQS_TFGI_MASK		(0x1f << TXFQS_TFGI_SHIFT)
#define TXFQS_TFFL_SHIFT	0
#define TXFQS_TFFL_MASK		(0x3f << TXFQS_TFFL_SHIFT)

/* Tx Buffer Element Size Configuration(TXESC) */
#define TXESC_TBDS_8BYTES	0x0
#define TXESC_TBDS_64BYTES	0x7

/* Tx Event FIFO Configuration (TXEFC) */
#define TXEFC_EFS_SHIFT		16
#define TXEFC_EFS_MASK		(0x3f << TXEFC_EFS_SHIFT)

/* Tx Event FIFO Status (TXEFS) */
#define TXEFS_TEFL		BIT(25)
#define TXEFS_EFF		BIT(24)
#define TXEFS_EFGI_SHIFT	8
#define	TXEFS_EFGI_MASK		(0x1f << TXEFS_EFGI_SHIFT)
#define TXEFS_EFFL_SHIFT	0
#define TXEFS_EFFL_MASK		(0x3f << TXEFS_EFFL_SHIFT)

/* Tx Event FIFO Acknowledge (TXEFA) */
#define TXEFA_EFAI_SHIFT	0
#define TXEFA_EFAI_MASK		(0x1f << TXEFA_EFAI_SHIFT)

/* Message RAM Configuration (in bytes) */
#define SIDF_ELEMENT_SIZE	4
#define XIDF_ELEMENT_SIZE	8
#define TXE_ELEMENT_SIZE	8
/* RX FIFO, RX buffer and TX buffer elements: ID, DLC and the data field */
#define M_CAN_ELEMENT_SIZE(data_len)	(8 + (data_len))

/* the most words to read or write at once, a run of FIFO elements */
#define M_CAN_FIFO_BATCH_WORDS	64

/* the most FIFO elements the registers can configure */
#define M_CAN_RXF_MAX		64
#define M_CAN_TXE_MAX		32
#define M_CAN_TXB_MAX		32

/* Message RAM Elements */
#define M_CAN_FIFO_ID		0x0
#define M_CAN_FIFO_DLC		0x4
#define M_CAN_FIFO_DATA(n)	(0x8 + ((n) << 2))

/* Rx Buffer Element */
/* R0 */
#define RX_BUF_ESI		BIT(31)
#define RX_BUF_XTD		BIT(30)
#define RX_BUF_RTR		BIT(29)
/* R1 */
#define RX_BUF_ANMF		BIT(31)
#define RX_BUF_FDF		BIT(21)
#define RX_BUF_BRS		BIT(20)

/* Tx Buffer Element */
/* T0 */
#define TX_BUF_ESI		BIT(31)
#define TX_BUF_XTD		BIT(30)
#define TX_BUF_RTR		BIT(29)
/* T1 */
#define TX_BUF_EFC		BIT(23)
#define TX_BUF_FDF		BIT(21)
#define TX_BUF_BRS		BIT(20)
#define TX_BUF_MM_SHIFT		24
#define TX_BUF_MM_MASK		(0xff << TX_BUF_MM_SHIFT)

/* Tx event FIFO Element */
/* E1 */
#define TX_EVENT_MM_SHIFT	TX_BUF_MM_SHIFT
#define TX_EVENT_MM_MASK	(0xff << TX_EVENT_MM_SHIFT)

static inline u32 m_can_read(struct m_can_classdev *cdev, enum m_can_reg reg)
{
	return cdev->ops->read_reg(cdev, reg);
}

static inline void m_can_write(struct m_can_classdev *cdev, enum m_can_reg reg,
			       u32 val)
{
	cdev->ops->write_reg(cdev, reg, val);
}

static void m_can_fifo_write(struct m_can_classdev *cdev,
			     u32 fpi, unsigned int offset, u32 val)
{
	u32 addr_offset = cdev->mcfg[MRAM_TXB].off + fpi * cdev->elem_size +
			  offset;

	cdev->ops->write_fifo(cdev, addr_offset, &val, 1);
}

static inline void m_can_fifo_write_no_off(struct m_can_classdev *cdev,
					   u32 fpi, u32 val)
{
	cdev->ops->write_fifo(cdev, fpi, &val, 1);
}

static inline bool m_can_tx_fifo_full(struct m_can_classdev *cdev)
{
		return !!(m_can_read(cdev, M_CAN_TXFQS) & TXFQS_TFQF);
}

void m_can_config_endisable(struct m_can_classdev *cdev, bool enable)
{
	u32 cccr = m_can_read(cdev, M_CAN_CCCR);
	u32 timeout = 10;
	u32 val = 0;

	/* Clear the Clock stop request if it was set */
	if (cccr & CCCR_CSR)
		cccr &= ~CCCR_CSR;

	if (enable) {
		/* Clear the Clock stop request if it was set */
		if (cccr & CCCR_CSR)
			cccr &= ~CCCR_CSR;

		/* enable m_can configuration */
		m_can_write(cdev, M_CAN_CCCR, cccr | CCCR_INIT);
		udelay(5);
		/* CCCR.CCE can only be set/reset while CCCR.INIT = '1' */
		m_can_write(cdev, M_CAN_CCCR, cccr | CCCR_INIT | CCCR_CCE);
	} else {
		m_can_write(cdev, M_CAN_CCCR, cccr & ~(CCCR_INIT | CCCR_CCE));
	}

	/* there's a delay for module initialization */
	if (enable)
		val = CCCR_INIT | CCCR_CCE;

	while ((m_can_read(cdev, M_CAN_CCCR) & (CCCR_INIT | CCCR_CCE)) != val) {
		if (timeout == 0) {
			netdev_warn(cdev->net, "Failed to init module\n");
			return;
		}
		timeout--;
		udelay(1);
	}
}

static inline void m_can_enable_all_interrupts(struct m_can_classdev *cdev)
{
	/* Only interrupt line 0 is used in this driver */
	m_can_write(cdev, M_CAN_ILE, ILE_EINT0);
}

static inline void m_can_disable_all_interrupts(struct m_can_classdev *cdev)
{
	m_can_write(cdev, M_CAN_ILE, 0x0);
}

static void m_can_clean(struct net_device *net)
{
	struct m_can_classdev *cdev = netdev_priv(net);

	if (cdev->tx_skb) {
		int putidx = 0;

		net->stats.tx_errors++;
		if (cdev->version > 30)
			putidx = ((m_can_read(cdev, M_CAN_TXFQS) &
				   TXFQS_TFQPI_MASK) >> TXFQS_TFQPI_SHIFT);

		can_free_echo_skb(cdev->net, putidx);
		cdev->tx_skb = NULL;
	}

	if (cdev->is_peripheral) {
		int i;

		/* the frames in flight are gone, free their echo skbs, or
		 * their FIFO elements can't be used again
		 */
		cancel_work_sync(&cdev->tx_work);
		for (i = 0; i < cdev->can.echo_skb_max; i++)
			can_free_echo_skb(net, i);
		net->stats.tx_errors += skb_queue_len(&cdev->tx_skbs);
		skb_queue_purge(&cdev->tx_skbs);
	}
}

/* receive the RX FIFO element in elem */
static void m_can_read_fifo(struct net_device *dev, const u32 *elem)
{
	struct net_device_stats *stats = &dev->stats;
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u32 id = elem[0], dlc = elem[1];

	if (dlc & RX_BUF_FDF)
		skb = alloc_canfd_skb(dev, &cf);
	else
		skb = alloc_can_skb(dev, (struct can_frame **)&cf);
	if (!skb) {
		stats->rx_dropped++;
		return;
	}

	if (dlc & RX_BUF_FDF)
		cf->len = can_dlc2len((dlc >> 16) & 0x0F);
	else
		cf->len = get_can_dlc((dlc >> 16) & 0x0F);

	if (id & RX_BUF_XTD)
		cf->can_id = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		cf->can_id = (id >> 18) & CAN_SFF_MASK;

	if (id & RX_BUF_ESI) {
		cf->flags |= CANFD_ESI;
		netdev_dbg(dev, "ESI Error\n");
	}

	if (!(dlc & RX_BUF_FDF) && (id & RX_BUF_RTR)) {
		cf->can_id |= CAN_RTR_FLAG;
	} else {
		if (dlc & RX_BUF_BRS)
			cf->flags |= CANFD_BRS;

		memcpy(cf->data, &elem[2], cf->len);
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->len;

	netif_receive_skb(skb);
}

static int m_can_do_rx_poll(struct net_device *dev, int quota)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	u32 words = cdev->elem_size / 4, num = cdev->mcfg[MRAM_RXF0].num;
	u32 buf[M_CAN_FIFO_BATCH_WORDS];
	u32 pkts = 0, rxfs, fgi, n, i;

	rxfs = m_can_read(cdev, M_CAN_RXF0S);
	if (!(rxfs & RXFS_FFL_MASK)) {
		netdev_dbg(dev, "no messages in fifo0\n");
		return 0;
	}

	while ((rxfs & RXFS_FFL_MASK) && (quota > 0)) {
		if (rxfs & RXFS_RFL)
			netdev_warn(dev, "Rx FIFO 0 Message Lost\n");

		/* read the elements up to the end of the fifo at once */
		fgi = (rxfs & RXFS_FGI_MASK) >> RXFS_FGI_SHIFT;
		n = min3(rxfs & RXFS_FFL_MASK, num - fgi,
			 (u32)ARRAY_SIZE(buf) / words);
		n = min_t(u32, n, quota);
		cdev->ops->read_fifo(cdev, cdev->mcfg[MRAM_RXF0].off +
				     fgi * cdev->elem_size, buf, n * words);
		for (i = 0; i < n; i++)
			m_can_read_fifo(dev, &buf[i * words]);

		/* acking an element acks the ones before it too */
		m_can_write(cdev, M_CAN_RXF0A, fgi + n - 1);

		quota -= n;
		pkts += n;

		/* a peripheral's ISR acked RF0N before getting here, so newer
		 * frames interrupt again
		 */
		if (cdev->is_peripheral && n == (rxfs & RXFS_FFL_MASK))
			break;
		rxfs = m_can_read(cdev, M_CAN_RXF0S);
	}

	if (pkts)
		can_led_event(dev, CAN_LED_EVENT_RX);

	return pkts;
}

static int m_can_handle_lost_msg(struct net_device *dev)
{
	struct net_device_stats *stats = &dev->stats;
	struct sk_buff *skb;
	struct can_frame *frame;

	netdev_err(dev, "msg lost in rxf0\n");

	stats->rx_errors++;
	stats->rx_over_errors++;

	skb = alloc_can_err_skb(dev, &frame);
	if (unlikely(!skb))
		return 0;

	frame->can_id |= CAN_ERR_CRTL;
	frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;

	netif_receive_skb(skb);

	return 1;
}

static int m_can_handle_lec_err(struct net_device *dev,
				enum m_can_lec_type lec_type)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;

	cdev->can.can_stats.bus_error++;
	stats->rx_errors++;

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	/* check for 'last error code' which tells us the
	 * type of the last error to occur on the CAN bus
	 */
	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	switch (lec_type) {
	case LEC_STUFF_ERROR:
		netdev_dbg(dev, "stuff error\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		break;
	case LEC_FORM_ERROR:
		netdev_dbg(dev, "form error\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
		break;
	case LEC_ACK_ERROR:
		netdev_dbg(dev, "ack error\n");
		cf->data[3] = CAN_ERR_PROT_LOC_ACK;
		break;
	case LEC_BIT1_ERROR:
		netdev_dbg(dev, "bit1 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		break;
	case LEC_BIT0_ERROR:
		netdev_dbg(dev, "bit0 error\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		break;
	case LEC_CRC_ERROR:
		netdev_dbg(dev, "CRC error\n");
		cf->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
		break;
	default:
		break;
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);

	return 1;
}

static int __m_can_get_berr_counter(const struct net_device *dev,
				    struct can_berr_counter *bec)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	unsigned int ecr;

	ecr = m_can_read(cdev, M_CAN_ECR);
	bec->rxerr = (ecr & ECR_REC_MASK) >> ECR_REC_SHIFT;
	bec->txerr = (ecr & ECR_TEC_MASK) >> ECR_TEC_SHIFT;

	return 0;
}

static int m_can_clk_start(struct m_can_classdev *cdev)
{
	int err;

	if (cdev->pm_clock_support == 0)
		return 0;

	err = pm_runtime_get_sync(cdev->dev);
	if (err < 0) {
		pm_runtime_put_noidle(cdev->dev);
		return err;
	}

	return 0;
}

static void m_can_clk_stop(struct m_can_classdev *cdev)
{
	if (cdev->pm_clock_support)
		pm_runtime_put_sync(cdev->dev);
}

static int m_can_get_berr_counter(const struct net_device *dev,
				  struct can_berr_counter *bec)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	int err;

	err = m_can_clk_start(cdev);
	if (err)
		return err;

	__m_can_get_berr_counter(dev, bec);

	m_can_clk_stop(cdev);

	return 0;
}

static int m_can_handle_state_change(struct net_device *dev,
				     enum can_state new_state)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	struct can_berr_counter bec;
	unsigned int ecr;

	switch (new_state) {
	case CAN_STATE_ERROR_ACTIVE:
		/* error warning state */
		cdev->can.can_stats.error_warning++;
		cdev->can.state = CAN_STATE_ERROR_WARNING;
		break;
	case CAN_STATE_ERROR_PASSIVE:
		/* error passive state */
		cdev->can.can_stats.error_passive++;
		cdev->can.state = CAN_STATE_ERROR_PASSIVE;
		break;
	case CAN_STATE_BUS_OFF:
		/* bus-off state */
		cdev->can.state = CAN_STATE_BUS_OFF;
		m_can_disable_all_interrupts(cdev);
		cdev->can.can_stats.bus_off++;
		can_bus_off(dev);
		break;
	default:
		break;
	}

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	__m_can_get_berr_counter(dev, &bec);

	switch (new_state) {
	case CAN_STATE_ERROR_ACTIVE:
		/* error warning state */
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = (bec.txerr > bec.rxerr) ?
			CAN_ERR_CRTL_TX_WARNING :
			CAN_ERR_CRTL_RX_WARNING;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case CAN_STATE_ERROR_PASSIVE:
		/* error passive state */
		cf->can_id |= CAN_ERR_CRTL;
		ecr = m_can_read(cdev, M_CAN_ECR);
		if (ecr & ECR_RP)
			cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		if (bec.txerr > 127)
			cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case CAN_STATE_BUS_OFF:
		/* bus-off state */
		cf->can_id |= CAN_ERR_BUSOFF;
		break;
	default:
		break;
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);

	return 1;
}

static int m_can_handle_state_errors(struct net_device *dev, u32 psr)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	int work_done = 0;

	if (psr & PSR_EW && cdev->can.state != CAN_STATE_ERROR_WARNING) {
		netdev_dbg(dev, "entered error warning state\n");
		work_done += m_can_handle_state_change(dev,
						       CAN_STATE_ERROR_WARNING);
	}

	if (psr & PSR_EP && cdev->can.state != CAN_STATE_ERROR_PASSIVE) {
		netdev_dbg(dev, "entered error passive state\n");
		work_done += m_can_handle_state_change(dev,
						       CAN_STATE_ERROR_PASSIVE);
	}

	if (psr & PSR_BO && cdev->can.state != CAN_STATE_BUS_OFF) {
		netdev_dbg(dev, "entered error bus off state\n");
		work_done += m_can_handle_state_change(dev,
						       CAN_STATE_BUS_OFF);
	}

	return work_done;
}

static void m_can_handle_other_err(struct net_device *dev, u32 irqstatus)
{
	if (irqstatus & IR_WDI)
		netdev_err(dev, "Message RAM Watchdog event due to missing READY\n");
	if (irqstatus & IR_ELO)
		netdev_err(dev, "Error Logging Overflow\n");
	if (irqstatus & IR_BEU)
		netdev_err(dev, "Bit Error Uncorrected\n");
	if (irqstatus & IR_BEC)
		netdev_err(dev, "Bit Error Corrected\n");
	if (irqstatus & IR_TOO)
		netdev_err(dev, "Timeout reached\n");
	if (irqstatus & IR_MRAF)
		netdev_err(dev, "Message RAM access failure occurred\n");
}

static inline bool is_lec_err(u32 psr)
{
	psr &= LEC_UNUSED;

	return psr && (psr != LEC_UNUSED);
}

static int m_can_handle_bus_errors(struct net_device *dev, u32 irqstatus,
				   u32 psr)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	int work_done = 0;

	if (irqstatus & IR_RF0L)
		work_done += m_can_handle_lost_msg(dev);

	/* handle lec errors on the bus */
	if ((cdev->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING) &&
	    is_lec_err(psr))
		work_done += m_can_handle_lec_err(dev, psr & LEC_UNUSED);

	/* other unproccessed error interrupts */
	m_can_handle_other_err(dev, irqstatus);

	return work_done;
}

static int m_can_rx_handler(struct net_device *dev, int quota)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	int work_done = 0;
	u32 irqstatus, psr = 0;

	/* a peripheral's ISR just read IR, anything newer interrupts again */
	irqstatus = cdev->irqstatus;
	if (!cdev->is_peripheral)
		irqstatus |= m_can_read(cdev, M_CAN_IR);
	if (!irqstatus)
		goto end;

	/* Errata workaround for issue "Needless activation of MRAF irq"
	 * During frame reception while the MCAN is in Error Passive state
	 * and the Receive Error Counter has the value MCAN_ECR.REC = 127,
	 * it may happen that MCAN_IR.MRAF is set although there was no
	 * Message RAM access failure.
	 * If MCAN_IR.MRAF is enabled, an interrupt to the Host CPU is generated
	 * The Message RAM Access Failure interrupt routine needs to check
	 * whether MCAN_ECR.RP = ’1’ and MCAN_ECR.REC = 127.
	 * In this case, reset MCAN_IR.MRAF. No further action is required.
	 */
	if (cdev->version <= 31 && irqstatus & IR_MRAF &&
	    m_can_read(cdev, M_CAN_ECR) & ECR_RP) {
		struct can_berr_counter bec;

		__m_can_get_berr_counter(dev, &bec);
		if (bec.rxerr == 127) {
			m_can_write(cdev, M_CAN_IR, IR_MRAF);
			irqstatus &= ~IR_MRAF;
		}
	}

	if (irqstatus & (IR_ERR_STATE | IR_ERR_BUS_30X))
		psr = m_can_read(cdev, M_CAN_PSR);

	if (irqstatus & IR_ERR_STATE)
		work_done += m_can_handle_state_errors(dev, psr);

	if (irqstatus & IR_ERR_BUS_30X)
		work_done += m_can_handle_bus_errors(dev, irqstatus, psr);

	if (irqstatus & IR_RF0N)
		work_done += m_can_do_rx_poll(dev, (quota - work_done));
end:
	return work_done;
}

static int m_can_rx_peripheral(struct net_device *dev)
{
	m_can_rx_handler(dev, INT_MAX);

	return 0;
}

static int m_can_poll(struct napi_struct *napi, int quota)
{
	struct net_device *dev = napi->dev;
	struct m_can_classdev *cdev = netdev_priv(dev);
	int work_done;

	work_done = m_can_rx_handler(dev, quota);
	if (work_done < quota) {
		napi_complete_done(napi, work_done);
		m_can_enable_all_interrupts(cdev);
	}

	return work_done;
}

static void m_can_echo_tx_event(struct net_device *dev)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	u32 num = cdev->mcfg[MRAM_TXE].num;
	u32 buf[M_CAN_FIFO_BATCH_WORDS];
	u32 txefs, fgi, count, n, i;

	txefs = m_can_read(cdev, M_CAN_TXEFS);
	fgi = (txefs & TXEFS_EFGI_MASK) >> TXEFS_EFGI_SHIFT;
	count = (txefs & TXEFS_EFFL_MASK) >> TXEFS_EFFL_SHIFT;

	while (count) {
		/* read the events up to the end of the fifo at once */
		n = min3(count, num - fgi, (u32)ARRAY_SIZE(buf) / 2);
		cdev->ops->read_fifo(cdev, cdev->mcfg[MRAM_TXE].off +
				     fgi * TXE_ELEMENT_SIZE, buf, n * 2);

		/* acking an element acks the ones before it too */
		m_can_write(cdev, M_CAN_TXEFA, (TXEFA_EFAI_MASK &
						((fgi + n - 1) << TXEFA_EFAI_SHIFT)));

		/* the message marker is the echo skb index */
		for (i = 0; i < n; i++) {
			stats->tx_bytes += can_get_echo_skb(dev,
				(buf[2 * i + 1] & TX_EVENT_MM_MASK) >>
				TX_EVENT_MM_SHIFT);
			stats->tx_packets++;
		}

		count -= n;
		fgi = (fgi + n) % num;
	}
}

static irqreturn_t m_can_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct m_can_classdev *cdev = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	u32 ir;

	ir = m_can_read(cdev, M_CAN_IR);
	if (!ir) {
		/* the level IRQ of a peripheral can also be its own, clear
		 * those too or it never deasserts
		 */
		if (cdev->is_peripheral && cdev->ops->clear_interrupts) {
			cdev->ops->clear_interrupts(cdev);
			return IRQ_HANDLED;
		}
		return IRQ_NONE;
	}

	/* ACK all irqs */
	if (ir & IR_ALL_INT)
		m_can_write(cdev, M_CAN_IR, ir);

	if (cdev->ops->clear_interrupts)
		cdev->ops->clear_interrupts(cdev);

	/* schedule NAPI in case of
	 * - rx IRQ
	 * - state change IRQ
	 * - bus error IRQ and bus error reporting
	 */
	if ((ir & IR_RF0N) || (ir & IR_ERR_ALL_30X)) {
		cdev->irqstatus = ir;
		if (!cdev->is_peripheral) {
			m_can_disable_all_interrupts(cdev);
			napi_schedule(&cdev->napi);
		} else {
			/* the threaded IRQ is oneshot, the line stays masked
			 * until this returns
			 */
			m_can_rx_peripheral(dev);
		}
	}

	if (cdev->version == 30) {
		if (ir & IR_TC) {
			/* Transmission Complete Interrupt*/
			stats->tx_bytes += can_get_echo_skb(dev, 0);
			stats->tx_packets++;
			can_led_event(dev, CAN_LED_EVENT_TX);
			netif_wake_queue(dev);
		}
	} else  {
		if (ir & IR_TEFN) {
			/* New TX FIFO Element arrived */
			m_can_echo_tx_event(dev);
			can_led_event(dev, CAN_LED_EVENT_TX);
			if (cdev->is_peripheral)
				/* the TX work wakes the queue */
				queue_work(cdev->tx_wq, &cdev->tx_work);
			else if (netif_queue_stopped(dev) &&
				 !m_can_tx_fifo_full(cdev))
				netif_wake_queue(dev);
		}
	}

	return IRQ_HANDLED;
}

static const struct can_bittiming_const m_can_bittiming_const_30X = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 64,
	.tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 16,
	.sjw_max = 16,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

static const struct can_bittiming_const m_can_data_bittiming_const_30X = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 16,
	.tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 32,
	.brp_inc = 1,
};

static const struct can_bittiming_const m_can_bittiming_const_31X = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 256,
	.tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 128,
	.sjw_max = 128,
	.brp_min = 1,
	.brp_max = 512,
	.brp_inc = 1,
};

static const struct can_bittiming_const m_can_data_bittiming_const_31X = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 1,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 32,
	.tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 16,
	.sjw_max = 16,
	.brp_min = 1,
	.brp_max = 32,
	.brp_inc = 1,
};

static int m_can_set_bittiming(struct net_device *dev)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	const struct can_bittiming *bt = &cdev->can.bittiming;
	const struct can_bittiming *dbt = &cdev->can.data_bittiming;
	u16 brp, sjw, tseg1, tseg2;
	u32 reg_btp;

	brp = bt->brp - 1;
	sjw = bt->sjw - 1;
	tseg1 = bt->prop_seg + bt->phase_seg1 - 1;
	tseg2 = bt->phase_seg2 - 1;
	reg_btp = (brp << NBTP_NBRP_SHIFT) | (sjw << NBTP_NSJW_SHIFT) |
		(tseg1 << NBTP_NTSEG1_SHIFT) | (tseg2 << NBTP_NTSEG2_SHIFT);
	m_can_write(cdev, M_CAN_NBTP, reg_btp);

	if (cdev->can.ctrlmode & CAN_CTRLMODE_FD) {
		reg_btp = 0;
		brp = dbt->brp - 1;
		sjw = dbt->sjw - 1;
		tseg1 = dbt->prop_seg + dbt->phase_seg1 - 1;
		tseg2 = dbt->phase_seg2 - 1;

		/* TDC is only needed for bitrates beyond 2.5 MBit/s.
		 * This is mentioned in the "Bit Time Requirements for CAN FD"
		 * paper presented at the International CAN Conference 2013
		 */
		if (dbt->bitrate > 2500000) {
			u32 tdco, ssp;

			/* Use the same value of secondary sampling point
			 * as the data sampling point
			 */
			ssp = dbt->sample_point;

			/* Equation based on Bosch's M_CAN User Manual's
			 * Transmitter Delay Compensation Section
			 */
			tdco = (cdev->can.clock.freq / 1000) *
			       ssp / dbt->bitrate;

			/* Max valid TDCO value is 127 */
			if (tdco > 127) {
				netdev_warn(dev, "TDCO value of %u is beyond maximum. Using maximum possible value\n",
					    tdco);
				tdco = 127;
			}

			reg_btp |= DBTP_TDC;
			m_can_write(cdev, M_CAN_TDCR,
				    tdco << TDCR_TDCO_SHIFT);
		}

		reg_btp |= (brp << DBTP_DBRP_SHIFT) |
			   (sjw << DBTP_DSJW_SHIFT) |
			   (tseg1 << DBTP_DTSEG1_SHIFT) |
			   (tseg2 << DBTP_DTSEG2_SHIFT);

		m_can_write(cdev, M_CAN_DBTP, reg_btp);
	}

	return 0;
}

/* Configure M_CAN chip:
 * - set rx buffer/fifo element size
 * - configure rx fifo
 * - accept non-matching frame into fifo 0
 * - configure tx buffer
 *		- >= v3.1.x: TX FIFO is used
 * - configure mode
 * - setup bittiming
 */
static u32 m_can_of_parse_mram(struct m_can_classdev *cdev,
			       const u32 *mram_config_vals, u32 scale);

static void m_can_chip_config(struct net_device *dev)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	u32 cccr, test, end;
	bool small;

	m_can_config_endisable(cdev, true);

	/* without CAN FD, 8 byte data fields keep consecutive elements close
	 * enough for peripherals to read and write a run of them at once
	 */
	small = cdev->is_peripheral && !(cdev->can.ctrlmode & CAN_CTRLMODE_FD);
	cdev->elem_size = M_CAN_ELEMENT_SIZE(CANFD_MAX_DLEN);
	end = m_can_of_parse_mram(cdev, cdev->mram_cfg_vals, 1);
	if (small) {
		/* and four of them take less room than a CAN FD element, so
		 * use 4x the FIFO elements if the TX event FIFO still fits
		 */
		cdev->elem_size = M_CAN_ELEMENT_SIZE(8);
		if (m_can_of_parse_mram(cdev, cdev->mram_cfg_vals, 4) > end)
			m_can_of_parse_mram(cdev, cdev->mram_cfg_vals, 1);
	}
	cdev->can.echo_skb_max = cdev->mcfg[MRAM_TXB].num;

	/* RX Buffer/FIFO Element Size */
	m_can_write(cdev, M_CAN_RXESC,
		    small ? M_CAN_RXESC_8BYTES : M_CAN_RXESC_64BYTES);

	/* Accept Non-matching Frames Into FIFO 0 */
	m_can_write(cdev, M_CAN_GFC, 0x0);

	if (cdev->version == 30) {
		/* only support one Tx Buffer currently */
		m_can_write(cdev, M_CAN_TXBC, (1 << TXBC_NDTB_SHIFT) |
				cdev->mcfg[MRAM_TXB].off);
	} else {
		/* TX FIFO is used for newer IP Core versions */
		m_can_write(cdev, M_CAN_TXBC,
			    (cdev->mcfg[MRAM_TXB].num << TXBC_TFQS_SHIFT) |
			    (cdev->mcfg[MRAM_TXB].off));
	}

	/* TX Buffer Element Size */
	m_can_write(cdev, M_CAN_TXESC,
		    small ? TXESC_TBDS_8BYTES : TXESC_TBDS_64BYTES);

	/* TX Event FIFO */
	if (cdev->version == 30) {
		m_can_write(cdev, M_CAN_TXEFC, (1 << TXEFC_EFS_SHIFT) |
				cdev->mcfg[MRAM_TXE].off);
	} else {
		/* Full TX Event FIFO is used */
		m_can_write(cdev, M_CAN_TXEFC,
			    ((cdev->mcfg[MRAM_TXE].num << TXEFC_EFS_SHIFT)
			     & TXEFC_EFS_MASK) |
			    cdev->mcfg[MRAM_TXE].off);
	}

	/* rx fifo configuration, blocking mode, fifo size 1 */
	m_can_write(cdev, M_CAN_RXF0C,
		    (cdev->mcfg[MRAM_RXF0].num << RXFC_FS_SHIFT) |
		     cdev->mcfg[MRAM_RXF0].off);

	m_can_write(cdev, M_CAN_RXF1C,
		    (cdev->mcfg[MRAM_RXF1].num << RXFC_FS_SHIFT) |
		     cdev->mcfg[MRAM_RXF1].off);

	cccr = m_can_read(cdev, M_CAN_CCCR);
	test = m_can_read(cdev, M_CAN_TEST);
	test &= ~TEST_LBCK;
	if (cdev->version == 30) {
	/* Version 3.0.x */

		cccr &= ~(CCCR_TEST | CCCR_MON |
			(CCCR_CMR_MASK << CCCR_CMR_SHIFT) |
			(CCCR_CME_MASK << CCCR_CME_SHIFT));

		if (cdev->can.ctrlmode & CAN_CTRLMODE_FD)
			cccr |= CCCR_CME_CANFD_BRS << CCCR_CME_SHIFT;

	} else {
	/* Version 3.1.x or 3.2.x */
		cccr &= ~(CCCR_TEST | CCCR_MON | CCCR_BRSE | CCCR_FDOE |
			  CCCR_NISO);

		/* Only 3.2.x has NISO Bit implemented */
		if (cdev->can.ctrlmode & CAN_CTRLMODE_FD_NON_ISO)
			cccr |= CCCR_NISO;

		if (cdev->can.ctrlmode & CAN_CTRLMODE_FD)
			cccr |= (CCCR_BRSE | CCCR_FDOE);
	}

	/* Loopback Mode */
	if (cdev->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		cccr |= CCCR_TEST | CCCR_MON;
		test |= TEST_LBCK;
	}

	/* Enable Monitoring (all versions) */
	if (cdev->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		cccr |= CCCR_MON;

	/* Write config */
	m_can_write(cdev, M_CAN_CCCR, cccr);
	m_can_write(cdev, M_CAN_TEST, test);

	/* Enable interrupts */
	m_can_write(cdev, M_CAN_IR, IR_ALL_INT);
	if (!(cdev->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING))
		if (cdev->version == 30)
			m_can_write(cdev, M_CAN_IE, IR_ALL_INT &
				    ~(IR_ERR_LEC_30X));
		else
			m_can_write(cdev, M_CAN_IE, IR_ALL_INT &
				    ~(IR_ERR_LEC_31X));
	else
		m_can_write(cdev, M_CAN_IE, IR_ALL_INT);

	/* every interrupt costs bus accesses on peripherals, so don't take
	 * the ones that aren't handled: TX complete, TX FIFO empty, timestamp
	 * wraparound, the FIFO watermarks/full and the unused RX FIFO 1/buffers
	 */
	if (cdev->is_peripheral)
		m_can_write(cdev, M_CAN_IE, m_can_read(cdev, M_CAN_IE) &
			    ~(IR_TC | IR_TCF | IR_TFE | IR_TSW | IR_HPM |
			      IR_TEFW | IR_TEFF | IR_RF0W | IR_RF0F |
			      IR_RF1N | IR_RF1W | IR_RF1F | IR_RF1L | IR_DRX));

	/* route all interrupts to INT0 */
	m_can_write(cdev, M_CAN_ILS, ILS_ALL_INT0);

	/* set bittiming params */
	m_can_set_bittiming(dev);

	m_can_config_endisable(cdev, false);

	if (cdev->ops->init)
		cdev->ops->init(cdev);
}

static void m_can_start(struct net_device *dev)
{
	struct m_can_classdev *cdev = netdev_priv(dev);

	/* basic m_can configuration */
	m_can_chip_config(dev);

	cdev->can.state = CAN_STATE_ERROR_ACTIVE;

	m_can_enable_all_interrupts(cdev);
}

static int m_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		m_can_clean(dev);
		m_can_start(dev);
		netif_wake_queue(dev);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/* Checks core release number of M_CAN
 * returns 0 if an unsupported device is detected
 * else it returns the release and step coded as:
 * return value = 10 * <release> + 1 * <step>
 */
static int m_can_check_core_release(struct m_can_classdev *cdev)
{
	u32 crel_reg;
	u8 rel;
	u8 step;
	int res;

	/* Read Core Release Version and split into version number
	 * Example: Version 3.2.1 => rel = 3; step = 2; substep = 1;
	 */
	crel_reg = m_can_read(cdev, M_CAN_CREL);
	rel = (u8)((crel_reg & CREL_REL_MASK) >> CREL_REL_SHIFT);
	step = (u8)((crel_reg & CREL_STEP_MASK) >> CREL_STEP_SHIFT);

	if (rel == 3) {
		/* M_CAN v3.x.y: create return value */
		res = 30 + step;
	} else {
		/* Unsupported M_CAN version */
		res = 0;
	}

	return res;
}

/* Selectable Non ISO support only in version 3.2.x
 * This function checks if the bit is writable.
 */
static bool m_can_niso_supported(struct m_can_classdev *cdev)
{
	u32 cccr_reg, cccr_poll = 0;
	int niso_timeout = -ETIMEDOUT;
	int i;

	m_can_config_endisable(cdev, true);
	cccr_reg = m_can_read(cdev, M_CAN_CCCR);
	cccr_reg |= CCCR_NISO;
	m_can_write(cdev, M_CAN_CCCR, cccr_reg);

	for (i = 0; i <= 10; i++) {
		cccr_poll = m_can_read(cdev, M_CAN_CCCR);
		if (cccr_poll == cccr_reg) {
			niso_timeout = 0;
			break;
		}

		usleep_range(1, 5);
	}

	/* Clear NISO */
	cccr_reg &= ~(CCCR_NISO);
	m_can_write(cdev, M_CAN_CCCR, cccr_reg);

	m_can_config_endisable(cdev, false);

	/* return false if time out (-ETIMEDOUT), else return true */
	return !niso_timeout;
}

static int m_can_dev_setup(struct m_can_classdev *m_can_dev)
{
	struct net_device *dev = m_can_dev->net;
	int m_can_version;

	m_can_version = m_can_check_core_release(m_can_dev);
	/* return if unsupported version */
	if (!m_can_version) {
		dev_err(m_can_dev->dev, "Unsupported version number: %2d",
			m_can_version);
		return -EINVAL;
	}

	if (!m_can_dev->is_peripheral)
		netif_napi_add(dev, &m_can_dev->napi,
			       m_can_poll, M_CAN_NAPI_WEIGHT);

	/* Shared properties of all M_CAN versions */
	m_can_dev->version = m_can_version;
	m_can_dev->can.do_set_mode = m_can_set_mode;
	m_can_dev->can.do_get_berr_counter = m_can_get_berr_counter;

	/* Set M_CAN supported operations */
	m_can_dev->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
					CAN_CTRLMODE_LISTENONLY |
					CAN_CTRLMODE_BERR_REPORTING |
					CAN_CTRLMODE_FD;

	/* Set properties depending on M_CAN version */
	switch (m_can_dev->version) {
	case 30:
		/* CAN_CTRLMODE_FD_NON_ISO is fixed with M_CAN IP v3.0.x */
		can_set_static_ctrlmode(dev, CAN_CTRLMODE_FD_NON_ISO);
		m_can_dev->can.bittiming_const = m_can_dev->bit_timing ?
			m_can_dev->bit_timing : &m_can_bittiming_const_30X;

		m_can_dev->can.data_bittiming_const = m_can_dev->data_timing ?
						m_can_dev->data_timing :
						&m_can_data_bittiming_const_30X;
		break;
	case 31:
		/* CAN_CTRLMODE_FD_NON_ISO is fixed with M_CAN IP v3.1.x */
		can_set_static_ctrlmode(dev, CAN_CTRLMODE_FD_NON_ISO);
		m_can_dev->can.bittiming_const = m_can_dev->bit_timing ?
			m_can_dev->bit_timing : &m_can_bittiming_const_31X;

		m_can_dev->can.data_bittiming_const = m_can_dev->data_timing ?
						m_can_dev->data_timing :
						&m_can_data_bittiming_const_31X;
		break;
	case 32:
		m_can_dev->can.bittiming_const = m_can_dev->bit_timing ?
			m_can_dev->bit_timing : &m_can_bittiming_const_31X;

		m_can_dev->can.data_bittiming_const = m_can_dev->data_timing ?
						m_can_dev->data_timing :
						&m_can_data_bittiming_const_31X;

		m_can_dev->can.ctrlmode_supported |=
						(m_can_niso_supported(m_can_dev)
						? CAN_CTRLMODE_FD_NON_ISO
						: 0);
		break;
	default:
		dev_err(m_can_dev->dev, "Unsupported version number: %2d",
			m_can_dev->version);
		return -EINVAL;
	}

	if (m_can_dev->ops->init)
		m_can_dev->ops->init(m_can_dev);

	return 0;
}

static void m_can_stop(struct net_device *dev)
{
	struct m_can_classdev *cdev = netdev_priv(dev);

	/* disable all interrupts */
	m_can_disable_all_interrupts(cdev);

	/* set the state as STOPPED */
	cdev->can.state = CAN_STATE_STOPPED;
}

static int m_can_close(struct net_device *dev)
{
	struct m_can_classdev *cdev = netdev_priv(dev);

	netif_stop_queue(dev);

	if (!cdev->is_peripheral)
		napi_disable(&cdev->napi);

	m_can_stop(dev);
	m_can_clk_stop(cdev);
	free_irq(dev->irq, dev);

	if (cdev->is_peripheral) {
		cdev->tx_skb = NULL;
		destroy_workqueue(cdev->tx_wq);
		cdev->tx_wq = NULL;
		skb_queue_purge(&cdev->tx_skbs);
	}

	close_candev(dev);
	can_led_event(dev, CAN_LED_EVENT_STOP);

	return 0;
}

static int m_can_next_echo_skb_occupied(struct net_device *dev, int putidx)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	/*get wrap around for loopback skb index */
	unsigned int wrap = cdev->can.echo_skb_max;
	int next_idx;

	/* calculate next index */
	next_idx = (++putidx >= wrap ? 0 : putidx);

	/* check if occupied */
	return !!cdev->can.echo_skb[next_idx];
}

static netdev_tx_t m_can_tx_handler(struct m_can_classdev *cdev)
{
	struct canfd_frame *cf = (struct canfd_frame *)cdev->tx_skb->data;
	struct net_device *dev = cdev->net;
	struct sk_buff *skb = cdev->tx_skb;
	u32 id, cccr, fdflags;
	int i;
	int putidx;

	/* Generate ID field for TX buffer Element */
	/* Common to all supported M_CAN versions */
	if (cf->can_id & CAN_EFF_FLAG) {
		id = cf->can_id & CAN_EFF_MASK;
		id |= TX_BUF_XTD;
	} else {
		id = ((cf->can_id & CAN_SFF_MASK) << 18);
	}

	if (cf->can_id & CAN_RTR_FLAG)
		id |= TX_BUF_RTR;

	if (cdev->version == 30) {
		netif_stop_queue(dev);

		/* message ram configuration */
		m_can_fifo_write(cdev, 0, M_CAN_FIFO_ID, id);
		m_can_fifo_write(cdev, 0, M_CAN_FIFO_DLC,
				 can_len2dlc(cf->len) << 16);

		for (i = 0; i < cf->len; i += 4)
			m_can_fifo_write(cdev, 0,
					 M_CAN_FIFO_DATA(i / 4),
					 *(u32 *)(cf->data + i));

		can_put_echo_skb(skb, dev, 0);

		if (cdev->can.ctrlmode & CAN_CTRLMODE_FD) {
			cccr = m_can_read(cdev, M_CAN_CCCR);
			cccr &= ~(CCCR_CMR_MASK << CCCR_CMR_SHIFT);
			if (can_is_canfd_skb(skb)) {
				if (cf->flags & CANFD_BRS)
					cccr |= CCCR_CMR_CANFD_BRS <<
						CCCR_CMR_SHIFT;
				else
					cccr |= CCCR_CMR_CANFD <<
						CCCR_CMR_SHIFT;
			} else {
				cccr |= CCCR_CMR_CAN << CCCR_CMR_SHIFT;
			}
			m_can_write(cdev, M_CAN_CCCR, cccr);
		}
		m_can_write(cdev, M_CAN_TXBTIE, 0x1);
		m_can_write(cdev, M_CAN_TXBAR, 0x1);
		/* End of xmit function for version 3.0.x */
	} else {
		/* Transmit routine for version >= v3.1.x */

		/* Check if FIFO full */
		if (m_can_tx_fifo_full(cdev)) {
			/* This shouldn't happen */
			netif_stop_queue(dev);
			netdev_warn(dev,
				    "TX queue active although FIFO is full.");

			if (cdev->is_peripheral) {
				kfree_skb(skb);
				dev->stats.tx_dropped++;
				return NETDEV_TX_OK;
			} else {
				return NETDEV_TX_BUSY;
			}
		}

		/* get put index for frame */
		putidx = ((m_can_read(cdev, M_CAN_TXFQS) & TXFQS_TFQPI_MASK)
				  >> TXFQS_TFQPI_SHIFT);
		/* Write ID Field to FIFO Element */
		m_can_fifo_write(cdev, putidx, M_CAN_FIFO_ID, id);

		/* get CAN FD configuration of frame */
		fdflags = 0;
		if (can_is_canfd_skb(skb)) {
			fdflags |= TX_BUF_FDF;
			if (cf->flags & CANFD_BRS)
				fdflags |= TX_BUF_BRS;
		}

		/* Construct DLC Field. Also contains CAN-FD configuration
		 * use put index of fifo as message marker
		 * it is used in TX interrupt for
		 * sending the correct echo frame
		 */
		m_can_fifo_write(cdev, putidx, M_CAN_FIFO_DLC,
				 ((putidx << TX_BUF_MM_SHIFT) &
				  TX_BUF_MM_MASK) |
				 (can_len2dlc(cf->len) << 16) |
				 fdflags | TX_BUF_EFC);

		for (i = 0; i < cf->len; i += 4)
			m_can_fifo_write(cdev, putidx, M_CAN_FIFO_DATA(i / 4),
					 *(u32 *)(cf->data + i));

		/* Push loopback echo.
		 * Will be looped back on TX interrupt based on message marker
		 */
		can_put_echo_skb(skb, dev, putidx);

		/* Enable TX FIFO element to start transfer  */
		m_can_write(cdev, M_CAN_TXBAR, (1 << putidx));

		/* stop network queue if fifo full */
		if (m_can_tx_fifo_full(cdev) ||
		    m_can_next_echo_skb_occupied(dev, putidx))
			netif_stop_queue(dev);
	}

	return NETDEV_TX_OK;
}

/* peripherals: write the queued frames to the TX FIFO, consecutive elements
 * at once, and request them all with one TXBAR write
 */
static void m_can_tx_work_queue(struct work_struct *ws)
{
	struct m_can_classdev *cdev = container_of(ws, struct m_can_classdev,
						tx_work);
	struct net_device *dev = cdev->net;
	u32 words = cdev->elem_size / 4, num = cdev->mcfg[MRAM_TXB].num;
	u32 buf[M_CAN_FIFO_BATCH_WORDS];
	u32 txfqs, free, putidx, start, n = 0, txbar = 0;
	struct canfd_frame *cf;
	struct sk_buff *skb;

	txfqs = m_can_read(cdev, M_CAN_TXFQS);
	free = (txfqs & TXFQS_TFFL_MASK) >> TXFQS_TFFL_SHIFT;
	putidx = (txfqs & TXFQS_TFQPI_MASK) >> TXFQS_TFQPI_SHIFT;
	start = putidx;

	/* an element's echo skb is freed when its TX event is handled */
	while (free && !cdev->can.echo_skb[putidx] &&
	       (skb = skb_dequeue(&cdev->tx_skbs))) {
		u32 *elem = &buf[n * words];

		cf = (struct canfd_frame *)skb->data;
		if (cf->can_id & CAN_EFF_FLAG)
			elem[0] = (cf->can_id & CAN_EFF_MASK) | TX_BUF_XTD;
		else
			elem[0] = (cf->can_id & CAN_SFF_MASK) << 18;
		if (cf->can_id & CAN_RTR_FLAG)
			elem[0] |= TX_BUF_RTR;

		/* the put index is the message marker, to find the echo skb
		 * when the TX event comes back
		 */
		elem[1] = ((putidx << TX_BUF_MM_SHIFT) & TX_BUF_MM_MASK) |
			  (can_len2dlc(cf->len) << 16) | TX_BUF_EFC;
		if (can_is_canfd_skb(skb)) {
			elem[1] |= TX_BUF_FDF;
			if (cf->flags & CANFD_BRS)
				elem[1] |= TX_BUF_BRS;
		}
		memset(&elem[2], 0, (words - 2) * 4);
		memcpy(&elem[2], cf->data, cf->len);

		can_put_echo_skb(skb, dev, putidx);

		txbar |= BIT(putidx);
		putidx = (putidx + 1) % num;
		free--;
		n++;

		if (putidx == 0 || (n + 1) * words > ARRAY_SIZE(buf)) {
			cdev->ops->write_fifo(cdev, cdev->mcfg[MRAM_TXB].off +
					      start * cdev->elem_size, buf,
					      n * words);
			start = putidx;
			n = 0;
		}
	}
	if (n)
		cdev->ops->write_fifo(cdev, cdev->mcfg[MRAM_TXB].off +
				      start * cdev->elem_size, buf, n * words);
	if (txbar)
		m_can_write(cdev, M_CAN_TXBAR, txbar);

	if (skb_queue_len(&cdev->tx_skbs) < cdev->can.echo_skb_max)
		netif_wake_queue(dev);
}

static netdev_tx_t m_can_start_xmit(struct sk_buff *skb,
				    struct net_device *dev)
{
	struct m_can_classdev *cdev = netdev_priv(dev);

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	if (cdev->is_peripheral) {
		if (cdev->can.state == CAN_STATE_BUS_OFF) {
			kfree_skb(skb);
			dev->stats.tx_dropped++;
			return NETDEV_TX_OK;
		}

		/* register access can sleep, so the TX work writes the frames,
		 * a TX FIFO's worth at a time
		 */
		skb_queue_tail(&cdev->tx_skbs, skb);
		if (skb_queue_len(&cdev->tx_skbs) >= cdev->can.echo_skb_max)
			netif_stop_queue(dev);
		queue_work(cdev->tx_wq, &cdev->tx_work);
	} else {
		cdev->tx_skb = skb;
		return m_can_tx_handler(cdev);
	}

	return NETDEV_TX_OK;
}

static int m_can_open(struct net_device *dev)
{
	struct m_can_classdev *cdev = netdev_priv(dev);
	int err;

	err = m_can_clk_start(cdev);
	if (err)
		return err;

	/* open the can device */
	err = open_candev(dev);
	if (err) {
		netdev_err(dev, "failed to open can device\n");
		goto exit_disable_clks;
	}

	/* register interrupt handler */
	if (cdev->is_peripheral) {
		cdev->tx_skb = NULL;
		skb_queue_head_init(&cdev->tx_skbs);
		cdev->tx_wq = alloc_workqueue("mcan_wq",
					      WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
		if (!cdev->tx_wq) {
			err = -ENOMEM;
			goto out_wq_fail;
		}

		INIT_WORK(&cdev->tx_work, m_can_tx_work_queue);

		err = request_threaded_irq(dev->irq, NULL, m_can_isr,
					   IRQF_ONESHOT,
					   dev->name, dev);
	} else {
		err = request_irq(dev->irq, m_can_isr, IRQF_SHARED, dev->name,
				  dev);
	}

	if (err < 0) {
		netdev_err(dev, "failed to request interrupt\n");
		goto exit_irq_fail;
	}

	/* start the m_can controller */
	m_can_start(dev);

	can_led_event(dev, CAN_LED_EVENT_OPEN);

	if (!cdev->is_peripheral)
		napi_enable(&cdev->napi);

	netif_start_queue(dev);

	return 0;

exit_irq_fail:
	if (cdev->is_peripheral)
		destroy_workqueue(cdev->tx_wq);
out_wq_fail:
	close_candev(dev);
exit_disable_clks:
	m_can_clk_stop(cdev);
	return err;
}

static const struct net_device_ops m_can_netdev_ops = {
	.ndo_open = m_can_open,
	.ndo_stop = m_can_close,
	.ndo_start_xmit = m_can_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int register_m_can_dev(struct net_device *dev)
{
	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->netdev_ops = &m_can_netdev_ops;

	return register_candev(dev);
}

/* lay out the message RAM with scale times the DT's FIFO elements, returns
 * the end of the layout
 */
static u32 m_can_of_parse_mram(struct m_can_classdev *cdev,
			       const u32 *mram_config_vals, u32 scale)
{
	cdev->mcfg[MRAM_SIDF].off = mram_config_vals[0];
	cdev->mcfg[MRAM_SIDF].num = mram_config_vals[1];
	cdev->mcfg[MRAM_XIDF].off = cdev->mcfg[MRAM_SIDF].off +
			cdev->mcfg[MRAM_SIDF].num * SIDF_ELEMENT_SIZE;
	cdev->mcfg[MRAM_XIDF].num = mram_config_vals[2];
	cdev->mcfg[MRAM_RXF0].off = cdev->mcfg[MRAM_XIDF].off +
			cdev->mcfg[MRAM_XIDF].num * XIDF_ELEMENT_SIZE;
	cdev->mcfg[MRAM_RXF0].num = min_t(u32, scale * (mram_config_vals[3] &
			(RXFC_FS_MASK >> RXFC_FS_SHIFT)), M_CAN_RXF_MAX);
	cdev->mcfg[MRAM_RXF1].off = cdev->mcfg[MRAM_RXF0].off +
			cdev->mcfg[MRAM_RXF0].num * cdev->elem_size;
	cdev->mcfg[MRAM_RXF1].num = min_t(u32, scale * (mram_config_vals[4] &
			(RXFC_FS_MASK >> RXFC_FS_SHIFT)), M_CAN_RXF_MAX);
	cdev->mcfg[MRAM_RXB].off = cdev->mcfg[MRAM_RXF1].off +
			cdev->mcfg[MRAM_RXF1].num * cdev->elem_size;
	cdev->mcfg[MRAM_RXB].num = mram_config_vals[5];
	cdev->mcfg[MRAM_TXE].off = cdev->mcfg[MRAM_RXB].off +
			cdev->mcfg[MRAM_RXB].num * cdev->elem_size;
	cdev->mcfg[MRAM_TXE].num = min_t(u32, scale * mram_config_vals[6],
					 M_CAN_TXE_MAX);
	cdev->mcfg[MRAM_TXB].off = cdev->mcfg[MRAM_TXE].off +
			cdev->mcfg[MRAM_TXE].num * TXE_ELEMENT_SIZE;
	cdev->mcfg[MRAM_TXB].num = min_t(u32, scale * (mram_config_vals[7] &
			(TXBC_NDTB_MASK >> TXBC_NDTB_SHIFT)), M_CAN_TXB_MAX);

	dev_dbg(cdev->dev,
		"sidf 0x%x %d xidf 0x%x %d rxf0 0x%x %d rxf1 0x%x %d rxb 0x%x %d txe 0x%x %d txb 0x%x %d\n",
		cdev->mcfg[MRAM_SIDF].off, cdev->mcfg[MRAM_SIDF].num,
		cdev->mcfg[MRAM_XIDF].off, cdev->mcfg[MRAM_XIDF].num,
		cdev->mcfg[MRAM_RXF0].off, cdev->mcfg[MRAM_RXF0].num,
		cdev->mcfg[MRAM_RXF1].off, cdev->mcfg[MRAM_RXF1].num,
		cdev->mcfg[MRAM_RXB].off, cdev->mcfg[MRAM_RXB].num,
		cdev->mcfg[MRAM_TXE].off, cdev->mcfg[MRAM_TXE].num,
		cdev->mcfg[MRAM_TXB].off, cdev->mcfg[MRAM_TXB].num);

	return cdev->mcfg[MRAM_TXB].off +
	       cdev->mcfg[MRAM_TXB].num * cdev->elem_size;
}

void m_can_init_ram(struct m_can_classdev *cdev)
{
	int end, i, start;

	/* initialize the entire Message RAM in use to avoid possible
	 * ECC/parity checksum errors when reading an uninitialized buffer
	 */
	start = cdev->mcfg[MRAM_SIDF].off;
	end = cdev->mcfg[MRAM_TXB].off +
		cdev->mcfg[MRAM_TXB].num * cdev->elem_size;

	for (i = start; i < end; i += 4)
		m_can_fifo_write_no_off(cdev, i, 0x0);
}
EXPORT_SYMBOL_GPL(m_can_init_ram);

int m_can_class_get_clocks(struct m_can_classdev *m_can_dev)
{
	int ret = 0;

	m_can_dev->hclk = devm_clk_get(m_can_dev->dev, "hclk");
	m_can_dev->cclk = devm_clk_get(m_can_dev->dev, "cclk");

	if (IS_ERR(m_can_dev->cclk)) {
		dev_err(m_can_dev->dev, "no clock found\n");
		ret = -ENODEV;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(m_can_class_get_clocks);

struct m_can_classdev *m_can_class_allocate_dev(struct device *dev)
{
	struct m_can_classdev *class_dev = NULL;
	u32 mram_config_vals[MRAM_CFG_LEN];
	struct net_device *net_dev;
	u32 tx_fifo_size;
	int ret;

	ret = fwnode_property_read_u32_array(dev_fwnode(dev),
					     "bosch,mram-cfg",
					     mram_config_vals,
					     sizeof(mram_config_vals) / 4);
	if (ret) {
		dev_err(dev, "Could not get Message RAM configuration.");
		goto out;
	}

	/* Get TX FIFO size
	 * Defines the total amount of echo buffers for loopback
	 */
	tx_fifo_size = mram_config_vals[7];

	/* allocate the m_can device */
	/* with echo skbs for the biggest TX FIFO a layout can use */
	net_dev = alloc_candev(sizeof(*class_dev),
			       max_t(u32, tx_fifo_size, M_CAN_TXB_MAX));
	if (!net_dev) {
		dev_err(dev, "Failed to allocate CAN device");
		goto out;
	}

	class_dev = netdev_priv(net_dev);
	if (!class_dev) {
		dev_err(dev, "Failed to init netdev cdevate");
		goto out;
	}

	class_dev->net = net_dev;
	class_dev->dev = dev;
	SET_NETDEV_DEV(net_dev, dev);

	memcpy(class_dev->mram_cfg_vals, mram_config_vals,
	       sizeof(mram_config_vals));
	class_dev->elem_size = M_CAN_ELEMENT_SIZE(CANFD_MAX_DLEN);
	m_can_of_parse_mram(class_dev, mram_config_vals, 1);
	class_dev->can.echo_skb_max = class_dev->mcfg[MRAM_TXB].num;
out:
	return class_dev;
}
EXPORT_SYMBOL_GPL(m_can_class_allocate_dev);

int m_can_class_register(struct m_can_classdev *m_can_dev)
{
	int ret;

	if (m_can_dev->pm_clock_support) {
		pm_runtime_enable(m_can_dev->dev);
		ret = m_can_clk_start(m_can_dev);
		if (ret)
			goto pm_runtime_fail;
	}

	ret = m_can_dev_setup(m_can_dev);
	if (ret)
		goto clk_disable;

	ret = register_m_can_dev(m_can_dev->net);
	if (ret) {
		dev_err(m_can_dev->dev, "registering %s failed (err=%d)\n",
			m_can_dev->net->name, ret);
		goto clk_disable;
	}

	devm_can_led_init(m_can_dev->net);

	dev_info(m_can_dev->dev, "%s device registered (irq=%d, version=%d)\n",
		 KBUILD_MODNAME, m_can_dev->net->irq, m_can_dev->version);

	/* Probe finished
	 * Stop clocks. They will be reactivated once the M_CAN device is opened
	 */
clk_disable:
	m_can_clk_stop(m_can_dev);
pm_runtime_fail:
	if (ret) {
		if (m_can_dev->pm_clock_support)
			pm_runtime_disable(m_can_dev->dev);
		free_candev(m_can_dev->net);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(m_can_class_register);

int m_can_class_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct m_can_classdev *cdev = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
		m_can_stop(ndev);
		m_can_clk_stop(cdev);
	}

	pinctrl_pm_select_sleep_state(dev);

	cdev->can.state = CAN_STATE_SLEEPING;

	return 0;
}
EXPORT_SYMBOL_GPL(m_can_class_suspend);

int m_can_class_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct m_can_classdev *cdev = netdev_priv(ndev);

	pinctrl_pm_select_default_state(dev);

	cdev->can.state = CAN_STATE_ERROR_ACTIVE;

	if (netif_running(ndev)) {
		int ret;

		ret = m_can_clk_start(cdev);
		if (ret)
			return ret;

		m_can_init_ram(cdev);
		m_can_start(ndev);
		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(m_can_class_resume);

void m_can_class_unregister(struct m_can_classdev *m_can_dev)
{
	unregister_candev(m_can_dev->net);

	m_can_clk_stop(m_can_dev);

	free_candev(m_can_dev->net);
}
EXPORT_SYMBOL_GPL(m_can_class_unregister);

MODULE_AUTHOR("Dong Aisheng <b29396@freescale.com>");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for Bosch M_CAN controller");
