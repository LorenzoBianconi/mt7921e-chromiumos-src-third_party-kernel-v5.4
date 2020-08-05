/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#ifndef _MTK_MCUPM_H
#define _MTK_MCUPM_H

#include <linux/platform_device.h>
#include <linux/rpmsg/mtk_rpmsg.h>

struct mtk_mcupm;

/**
 * enum ipi_id - the id of inter-processor interrupt
 *
 * @MCUPM_IPI_PLATFORM:	 The interrupt from mcupm is to notfiy kernel
 *			 reveived request and reply ack.
 *			 For other IPI below, AP should send the request
 *			 to MCUPM to trigger the interrupt.
 * @MCUPM_IPI_MAX:	 The maximum IPI number
 */

enum mcupm_ipi_id {
	MCUPM_IPI_PLATFORM = 0,
	MCUPM_IPI_CPU_DVFS,
	MCUPM_IPI_FHCTL,
	MCUPM_IPI_MCDI,
	MCUPM_IPI_SUSPEND,
	MCUPM_IPI_IPIR_MET,
	MCUPM_IPI_IPIS_MET,
	MCUPM_IPI_EEMSN,
	MCUPM_IPI_NUM,
	MCUPM_IPI_INIT_SERVICE = 0xFF,
	MCUPM_IPI_MAX = 0x100,
};

#define MCUPM_MBOX_TOTAL 8
#define SMEM_SIZE_80B			0x00000014      //80 Bytes
#define PIN_S_SIZE			SMEM_SIZE_80B
#define PIN_R_SIZE			SMEM_SIZE_80B
#define MBOX_TABLE_SIZE			(PIN_S_SIZE + PIN_R_SIZE)

#define MBOX_SLOT_SIZE 4

/*
 * mbox return value definition
 */
enum MBOX_RETURN {
	MBOX_READ_SZ_ERR  = -6,
	MBOX_WRITE_SZ_ERR = -5,
	MBOX_PARA_ERR     = -4,
	MBOX_CONFIG_ERR   = -3,
	MBOX_IRQ_ERR      = -2,
	MBOX_PLT_ERR      = -1,
	MBOX_DONE         = 0,
	MBOX_PIN_BUSY     = 1,
};

/* MCUPM RESERVED MEM */
#define MCUPM_RESERVED_DEBUG		(1)
#define MCUPM_PLT_LOGGER_BUF_LEN	0x100000

#if !defined(CONFIG_MTK_GMO_RAM_OPTIMIZE) && !defined(CONFIG_MTK_MET_MEM_ALLOC)
#define MCUPM_MET_LOGGER_BUF_LEN	0x400000
#endif
#define MCUPM_PLT_EEMSN_BUF_LEN		0x1000
#define MCUPM_BRISKET_BUF_LEN		0x1000

enum {
	MCUPM_MEM_ID = 0,
#if !defined(CONFIG_MTK_GMO_RAM_OPTIMIZE) && !defined(CONFIG_MTK_MET_MEM_ALLOC)
	MCUPM_MET_ID,
#endif
	MCUPM_EEMSN_MEM_ID,
	MCUPM_BRISKET_ID,
	NUMS_MCUPM_MEM_ID,
};

/*
 * mbox record information
 *
 * write_count    :mbox write success count
 * busy_count     :mbox read success count
 * trig_irq_count :mbox trigger irq success count
 */
struct mtk_mbox_record {
	uint32_t write_count;
	uint32_t busy_count;
	uint32_t trig_irq_count;
};

/*
 * mbox information
 *
 * mbdev  :mbox device
 * irq_num:identity of mbox irq
 * id     :mbox id
 * slot   :how many slots that mbox used
 * opt    :option for tx mode, 0:mbox, 1:share memory 2:queue
 * enable :mbox status, 0:disable, 1: enable
 * is64d  :mbox is64d status, 0:32d, 1: 64d
 * base   :mbox base address
 * set_irq_reg  :mbox set irq register
 * clr_irq_reg  :mbox clear irq register
 * init_base_reg:mbox initialize register
 * mbox lock    :lock of mbox
 * record       :mbox record information
 */
struct mtk_mcupm_mbox_info {
	struct mtk_mcupm *mcupm;
	int irq_num;
	unsigned int id;
	unsigned int slot;
	unsigned int opt;
	bool enable;
	bool is64d;
	void __iomem *base;
	void __iomem *set_irq_reg;
	void __iomem *clr_irq_reg;
	void __iomem *init_base_reg;
	void __iomem *send_status_reg;
	void __iomem *recv_status_reg;
	spinlock_t mbox_lock;
	struct mtk_mbox_record record;
};

struct mcupm_ipi_desc {
	/* For protecting handler. */
	struct mutex lock;
	ipi_handler_t handler;
	void *priv;
};

struct mcupm_reserve_mblock {
	u32 num;
	u64 start_phys;
	u64 start_virt;
	u64 size;
};

struct mtk_mcupm {
	struct rproc_subdev *rpmsg_subdev;
	struct device *dev;
	struct rproc *rproc;

	struct mtk_mcupm_mbox_info *info_table;

	/* To prevent multiple ipi_send run concurrently. */
	struct mutex send_lock;
	struct mcupm_ipi_desc ipi_desc[MCUPM_IPI_MAX];
	bool ipi_id_ack[MCUPM_IPI_MAX];
	wait_queue_head_t ack_wq;
};

#endif
