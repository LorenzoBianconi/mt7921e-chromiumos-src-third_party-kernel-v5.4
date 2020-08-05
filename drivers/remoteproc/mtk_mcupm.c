/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/iopoll.h>
#include "mtk_mcupm.h"

char* mcupm_device_name[] = {
	"mcupm-plt-rpmsg",
	"mcupm-dvfs-rpmsg",
	"mcupm-fhctl-rpmsg",
	"mcupm-mcdi-rpmsg",
	"mcupm-suspend-rpmsg",
	"mcupm-ipir_met-rpmsg",
	"mcupm-ipis_met-rpmsg",
	"mcupm-eemsn-rpmsg",
};

struct mtk_mcupm_mbox_info mcupm_mbox_table[MCUPM_MBOX_TOTAL] = {
	{0, 0, 0, MBOX_TABLE_SIZE, 1, 1, 0, 0, 0, 0, 0, 0, 0,
		{ { { __ARCH_SPIN_LOCK_UNLOCKED } } }, {0, 0, 0} },
	{0, 0, 1, MBOX_TABLE_SIZE, 1, 1, 0, 0, 0, 0, 0, 0, 0,
		{ { { __ARCH_SPIN_LOCK_UNLOCKED } } }, {0, 0, 0} },
	{0, 0, 2, MBOX_TABLE_SIZE, 1, 1, 0, 0, 0, 0, 0, 0, 0,
		{ { { __ARCH_SPIN_LOCK_UNLOCKED } } }, {0, 0, 0} },
	{0, 0, 3, MBOX_TABLE_SIZE, 1, 1, 0, 0, 0, 0, 0, 0, 0,
		{ { { __ARCH_SPIN_LOCK_UNLOCKED } } }, {0, 0, 0} },
	{0, 0, 4, MBOX_TABLE_SIZE, 1, 1, 0, 0, 0, 0, 0, 0, 0,
		{ { { __ARCH_SPIN_LOCK_UNLOCKED } } }, {0, 0, 0} },
	{0, 0, 5, MBOX_TABLE_SIZE, 1, 1, 0, 0, 0, 0, 0, 0, 0,
		{ { { __ARCH_SPIN_LOCK_UNLOCKED } } }, {0, 0, 0} },
	{0, 0, 6, MBOX_TABLE_SIZE, 1, 1, 0, 0, 0, 0, 0, 0, 0,
		{ { { __ARCH_SPIN_LOCK_UNLOCKED } } }, {0, 0, 0} },
	{0, 0, 7, MBOX_TABLE_SIZE, 1, 1, 0, 0, 0, 0, 0, 0, 0,
		{ { { __ARCH_SPIN_LOCK_UNLOCKED } } }, {0, 0, 0} },
};

/*
 * memory copy to tiny
 * @param dest: dest address
 * @param src: src address
 * @param size: memory size
 */
void mtk_memcpy_to_tinysys(void __iomem *dest, const void *src, int size)
{
	int i;
	u32 __iomem *t = dest;
	const u32 *s = src;

	for (i = 0; i < ((size + 3) >> 2); i++)
		*t++ = *s++;
}
/*
 * memory copy from tiny
 * @param dest: dest address
 * @param src: src address
 * @param size: memory size
 */
void mtk_memcpy_from_tinysys(void *dest, const void __iomem *src, int size)
{
	int i;
	u32 *t = dest;
	const u32 __iomem *s = src;

	for (i = 0; i < ((size + 3) >> 2); i++)
		*t++ = *s++;
}
/*
 * check mbox 32bits clr irq reg status
 * with read/write function must in critical context
 * @return irq status 0: not triggered , other: irq triggered
 */
unsigned int mtk_mbox_read_recv_irq(struct mtk_mcupm_mbox_info *mbox_info,
		unsigned int id)
{
	unsigned int reg;

	if (!mbox_info)
		return 0;

	if (mbox_info->recv_status_reg)
		reg = readl(mbox_info->recv_status_reg);
	else
		reg = readl(mbox_info->clr_irq_reg);

	return reg;
}

static int mcupm_mbox_trigger_irq(struct mtk_mcupm_mbox_info *mbox_info,
		unsigned int irq)
{
	unsigned long flags;

	if (!mbox_info)
		return -EINVAL;

	spin_lock_irqsave(&mbox_info->mbox_lock, flags);
	writel(irq, mbox_info->set_irq_reg);
	mbox_info->record.trig_irq_count++;
	spin_unlock_irqrestore(&mbox_info->mbox_lock, flags);

	return MBOX_DONE;

}
/*
 * clear mbox irq,
 * with read/write function must in critical context
 */
int mtk_mbox_clr_irq(struct mtk_mcupm_mbox_info *mbox_info,
		unsigned int irq)
{
	if (!mbox_info)
		return MBOX_PLT_ERR;

	writel(irq, mbox_info->clr_irq_reg);

	return MBOX_DONE;
}

int mcupm_mbox_write(struct mtk_mcupm_mbox_info *mbox, int slot, void *data, unsigned int len)
{
	unsigned int slot_ofs, size;
	struct mtk_mcupm_mbox_info *minfo;
	void __iomem *base;
	unsigned long flags;

	if (!mbox) {
		pr_notice("[MBOX]write fail, dev or ptr null");
		return MBOX_PLT_ERR;
	}

	if (slot >= mbox->slot || !data)
		return MBOX_PARA_ERR;

	minfo = mbox;
	base = minfo->base;
	slot_ofs = slot * MBOX_SLOT_SIZE;
	size = minfo->slot;

	if (slot > size)
		return MBOX_WRITE_SZ_ERR;

	spin_lock_irqsave(&mbox->mbox_lock, flags);
	mtk_memcpy_to_tinysys((void __iomem *)(base + slot_ofs),
			data, len);
	minfo->record.write_count++;
	spin_unlock_irqrestore(&mbox->mbox_lock, flags);

	return MBOX_DONE;

}

/**
 * mcupm_ipi_send() - send data from AP to MCUPM.
 *
 * @mcupm:	mtk_mcupm structure
 * @id:		IPI ID
 * @buf:	the data buffer
 * @len:	the data buffer length
 * @wait:	number of msecs to wait for ack. 0 to skip waiting.
 *
 * This function is thread-safe. When this function returns,
 * MCUPM has received the data and starts the processing.
 * When the processing completes, IPI handler registered
 * by scp_ipi_register will be called in interrupt context.
 *
 * Returns 0 if sending data successfully, -error on error.
 **/
int mcupm_ipi_send(struct mtk_mcupm *mcupm, u32 id, void *buf, unsigned int len,
		 unsigned int wait)
{
	struct mtk_mcupm_mbox_info *mbox_info;
	unsigned long flags;
	unsigned int reg, irq_state;
	int ret;

	if(!mcupm) {
		pr_info("%s: mcupm is null pointer\n", __func__);
		return -EINVAL;
	}

	dev_err(mcupm->dev, "%s: (%d) len=%d\n", __func__, id, len);

	if (WARN_ON(id < MCUPM_IPI_PLATFORM) || WARN_ON(id >= MCUPM_IPI_MAX) ||
	    WARN_ON(!buf))
		return -EINVAL;

	mbox_info = &mcupm->info_table[id];

	spin_lock_irqsave(&mbox_info->mbox_lock, flags);
	irq_state = 0;
	if (mbox_info->send_status_reg)
		reg = readl(mbox_info->send_status_reg);
	else
		reg = readl(mbox_info->set_irq_reg);

	irq_state = (reg & (0x1 << id));

	if (irq_state) {
		mbox_info->record.busy_count++;
		spin_unlock_irqrestore(&mbox_info->mbox_lock, flags);
		return -EBUSY;
	}
	spin_unlock_irqrestore(&mbox_info->mbox_lock, flags);

	ret = mcupm_mbox_write(mbox_info, 0, buf, len);
	if (ret != MBOX_DONE)
		return ret;

	/*
	 * Ensure that all writes to SRAM are committed before sending the
	 * interrupt to mbox.
	 */
	mb();

	ret = mcupm_mbox_trigger_irq(mbox_info, (0x1 << id));

	if (mbox_info->send_status_reg) {
		ret = readl_poll_timeout_atomic(mbox_info->send_status_reg, reg,
										!(reg & (0x1 << id)), 10, 20 *1000);
	} else {
		ret = readl_poll_timeout_atomic(mbox_info->set_irq_reg, reg,
										!(reg & (0x1 << id)), 10, 20 *1000);
	}

	if (ret != MBOX_DONE)
		return ret;

	return ret;

}
EXPORT_SYMBOL_GPL(mcupm_ipi_send);

static void mcupm_ipi_handler(struct mtk_mcupm *mcupm, int id)
{
	struct mcupm_ipi_desc *pIPI_desc;
	struct mtk_mcupm_mbox_info *pMBOX_info;
	ipi_handler_t handler;
	u8 tmp_data[160];

	if (!mcupm) {
		dev_err(mcupm->dev, "mcupm pointer is NULL\n");
		return;
	}
	if (id >= MCUPM_IPI_MAX) {
		dev_err(mcupm->dev, "No such ipi id = %d\n", id);
		return;
	}

	pIPI_desc = &mcupm->ipi_desc[id];
	pMBOX_info = &mcupm->info_table[id];

	if (!pIPI_desc->handler) {
		dev_err(mcupm->dev, "ipi_desc pointer is NULL\n");
		return;
	} else {
		dev_err(mcupm->dev, "Call ipi handler\n");
	}
	//scp_ipi_lock(scp, id);
	handler = pIPI_desc->handler;
	if (!handler) {
		dev_err(mcupm->dev, "No such ipi id = %d\n", id);
		//scp_ipi_unlock(scp, id);
		return;
	}
#define SMEM_SIZE_80B			0x00000014      //80 Bytes
#define MBOX_SLOT_SIZE 4
	mtk_memcpy_from_tinysys(tmp_data, (void __iomem *)(pMBOX_info->base + (SMEM_SIZE_80B * MBOX_SLOT_SIZE)), (SMEM_SIZE_80B * MBOX_SLOT_SIZE));
	handler(tmp_data, (SMEM_SIZE_80B * MBOX_SLOT_SIZE), pIPI_desc->priv);
	//scp_ipi_unlock(scp, id);
}
static irqreturn_t mcupm_irq_handler(int irq, void *priv)
{
	struct mtk_mcupm_mbox_info *mbox_info = priv;
	unsigned long flags, irq_state, reg;
	unsigned int mbox_id = mbox_info->id;

	pr_info("mcupm_irq_handlerl reve (%d) mbox id %d\n", irq, mbox_id);

	/*check bit*/
	spin_lock_irqsave(&mbox_info->mbox_lock, flags);
	reg = mtk_mbox_read_recv_irq(mbox_info, mbox_id);
	irq_state = (reg & (0x1 << mbox_id));
	spin_unlock_irqrestore(&mbox_info->mbox_lock, flags);

	/*copy data*/
	mcupm_ipi_handler(mbox_info->mcupm, mbox_id);

	/*clear bit*/
	spin_lock_irqsave(&mbox_info->mbox_lock, flags);
	mtk_mbox_clr_irq(mbox_info, irq_state);
	spin_unlock_irqrestore(&mbox_info->mbox_lock, flags);

	return IRQ_HANDLED;
}
int mcupm_ipi_register(struct platform_device *pdev,
			 struct mtk_mcupm *mcupm,
		     u32 id,
		     ipi_handler_t handler,
		     void *priv)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct mtk_mcupm_mbox_info *mbox_info;
	if (!mcupm) {
		dev_err(mcupm->dev, "mcupm device is not ready\n");
		return -EPROBE_DEFER;
	}

	if (WARN_ON(id >= MCUPM_IPI_MAX) || WARN_ON(handler == NULL))
		return -EINVAL;

	mbox_info = &mcupm->info_table[id];
	spin_lock_init(&mbox_info->mbox_lock);

	//scp_ipi_lock(scp, id);
	if(mcupm->ipi_desc[id].handler) {
		dev_err(mcupm->dev, "mcupm IPI handler(%d) is already exist\n", id);
		return -EEXIST;
	}

	mcupm->ipi_desc[id].handler = handler;
	mcupm->ipi_desc[id].priv = priv;
	//scp_ipi_unlock(scp, id);

	ret = devm_request_threaded_irq(dev, platform_get_irq(pdev, id), mcupm_irq_handler,
						NULL, IRQF_TRIGGER_NONE,
						pdev->name, (void *)mbox_info);

	return 0;
}
/**
 * scp_ipi_unregister() - unregister an ipi function
 *
 * @scp:	mtk_scp structure
 * @id:		IPI ID
 *
 * Unregister an ipi function to receive ipi interrupt from SCP.
 */
static void mcupm_ipi_unregister(struct mtk_mcupm *mcupm, u32 id)
{
	if (!mcupm)
		return;

	if (WARN_ON(id >= MCUPM_IPI_MAX))
		return;

	//scp_ipi_lock(scp, id);
	mcupm->ipi_desc[id].handler = NULL;
	mcupm->ipi_desc[id].priv = NULL;
	//scp_ipi_unlock(scp, id);
}
static int mcupm_send_ipi(struct platform_device *pdev, u32 id, void *buf,
			unsigned int len, unsigned int wait)
{
	struct mtk_mcupm *mcupm = platform_get_drvdata(pdev);

	return mcupm_ipi_send(mcupm, id, buf, len, wait);
}
static int mcupm_register_ipi(struct platform_device *pdev, u32 id,
			    ipi_handler_t handler, void *priv)
{
	struct mtk_mcupm *mcupm = platform_get_drvdata(pdev);

	return mcupm_ipi_register(pdev, mcupm, id, handler, priv);
}
static void mcupm_unregister_ipi(struct platform_device *pdev, u32 id)
{
	struct mtk_mcupm *mcupm = platform_get_drvdata(pdev);

	mcupm_ipi_unregister(mcupm, id);
}

static struct mtk_rpmsg_info mtk_mcupm_rpmsg_info = {
	.send_ipi = mcupm_send_ipi,
	.register_ipi = mcupm_register_ipi,
	.unregister_ipi = mcupm_unregister_ipi,
	.ns_ipi_id = MCUPM_IPI_INIT_SERVICE,
};

static void mcupm_add_rpmsg_subdev(struct mtk_mcupm *mcupm)
{
	mcupm->rpmsg_subdev =
		mtk_rpmsg_create_rproc_subdev(to_platform_device(mcupm->dev),
					      &mtk_mcupm_rpmsg_info);

	if (mcupm->rpmsg_subdev)
		rproc_add_subdev(mcupm->rproc, mcupm->rpmsg_subdev);
}

static void mcupm_remove_rpmsg_subdev(struct mtk_mcupm *mcupm)
{
	if (mcupm->rpmsg_subdev) {
		rproc_remove_subdev(mcupm->rproc, mcupm->rpmsg_subdev);
		mtk_rpmsg_destroy_rproc_subdev(mcupm->rpmsg_subdev);
		mcupm->rpmsg_subdev = NULL;
	}
}

/*
 * mtk_mbox_probe , porbe and initial mbox
 *
 */
static int mtk_parser_mbox_properties(struct platform_device *pdev, struct mtk_mcupm *mcupm, struct mtk_mcupm_mbox_info *mbox_table, int mbox_size)
{
	int mbox = 0;
	char name[32];
	struct resource *res;
	struct device *dev = &pdev->dev;
	/*set irq reg*/
	snprintf(name, sizeof(name), "mbox%d_set", mbox);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	mbox_table[mbox].set_irq_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR((void const *) mbox_table[mbox].set_irq_reg)) {
		pr_err("MBOX %d can't find set reg(%s)\n", mbox, name);
		goto mtk_mbox_probe_fail;
	}

	/*clear reg*/
	snprintf(name, sizeof(name), "mbox%d_clr", mbox);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	mbox_table[mbox].clr_irq_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR((void const *) mbox_table[mbox].clr_irq_reg)) {
		pr_err("MBOX %d can't find clr reg(%s)\n", mbox, name);
		goto mtk_mbox_probe_fail;
	}

	/*send status reg*/
	snprintf(name, sizeof(name), "mbox%d_send", mbox);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	mbox_table[mbox].send_status_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR((void const *) mbox_table[mbox].send_status_reg)) {
		pr_notice("MBOX %d can't find send status reg(%s)\n", mbox, name);
		mbox_table[mbox].send_status_reg = NULL;
	}

	/*recv status reg*/
	snprintf(name, sizeof(name), "mbox%d_recv", mbox);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	mbox_table[mbox].recv_status_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR((void const *) mbox_table[mbox].recv_status_reg)) {
		pr_notice("MBOX %d can't find recv status reg(%s)\n", mbox, name);
		mbox_table[mbox].recv_status_reg = NULL;
	}

	for (mbox = 0; mbox < mbox_size; mbox++) {
		pr_info("[MCUPM]  mbox-%d, probe\n", mbox);
		if (pdev) {
			snprintf(name, sizeof(name), "mbox%d_base", mbox);
			res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
			mbox_table[mbox].base = devm_ioremap_resource(dev, res);
			if (IS_ERR((void const *) mbox_table[mbox].base)) {
				pr_err("MBOX %d can't find base reg(%s)\n", mbox, name);
				goto mtk_mbox_probe_fail;
			}

			snprintf(name, sizeof(name), "mbox%d", mbox);
			mbox_table[mbox].irq_num = platform_get_irq_byname(pdev, name);
			if (mbox_table[mbox].irq_num < 0) {
				pr_err("MBOX %d can't find IRQ\n", mbox);
				goto mtk_mbox_probe_fail;
			}
		}
		mbox_table[mbox].set_irq_reg = mbox_table[0].set_irq_reg;
		mbox_table[mbox].clr_irq_reg = mbox_table[0].clr_irq_reg;
		mbox_table[mbox].send_status_reg = mbox_table[0].send_status_reg;
		mbox_table[mbox].recv_status_reg = mbox_table[0].recv_status_reg;
		mbox_table[mbox].mcupm = mcupm;
	}

mtk_mbox_probe_fail:
	return -3;

}

static int mcupm_ipi_init(struct mtk_mcupm *mcupm)
{
	int i;

	//mcupm register mcupm ipi
	mutex_init(&mcupm->send_lock);
	for (i = 0; i < MCUPM_IPI_MAX; i++)
		mutex_init(&mcupm->ipi_desc[i].lock);

	return 0;

}
static int mcupm_start(struct rproc *rproc)
{
	WARN(1, "%s: start from preloader should not run here!\n", __func__);
    return 0;
}
static int mcupm_stop(struct rproc *rproc)
{
	WARN(1, "%s: load from preloader should not run here!\n", __func__);
	return 0;
}
static int mcupm_load(struct rproc *rproc, const struct firmware *fw)
{
	WARN(1, "%s: load from preloader should not run here!\n", __func__);
	return 0;
}

static const struct rproc_ops mcupm_ops = {
	.start		= mcupm_start,
	.stop		= mcupm_stop,
	.load		= mcupm_load,
};

static int mcupm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mtk_mcupm *mcupm;
	struct rproc *rproc;
	char *fw_name = NULL;
	int ret;

	rproc = rproc_alloc(dev,
			    np->name,
			    &mcupm_ops,
			    fw_name,
			    sizeof(*mcupm));
	if (!rproc) {
		dev_err(dev, "unable to allocate remoteproc\n");
		return -ENOMEM;
	}
    rproc->auto_boot = 0;
	mcupm = (struct mtk_mcupm *)rproc->priv;
	mcupm->rproc = rproc;
	mcupm->dev = dev;
	platform_set_drvdata(pdev, mcupm);

    mcupm->info_table = &mcupm_mbox_table[0];

	mtk_parser_mbox_properties(pdev, mcupm, mcupm->info_table, MCUPM_MBOX_TOTAL);

	ret = mcupm_ipi_init(mcupm);

	init_waitqueue_head(&mcupm->ack_wq);
	dev_err(dev, "mtk add MCUPM Name Service\n");

	mcupm_add_rpmsg_subdev(mcupm);

	if (ret) {
		dev_err(dev, "failed to request irq\n");
		goto remove_subdev;
	}

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc add failed\n");
		goto remove_subdev;
	}

	return 0;

remove_subdev:
	mcupm_remove_rpmsg_subdev(mcupm);

	return ret;
}
static int mcupm_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id mtk_mcupm_of_match[] = {
	{ .compatible = "mediatek,mt6873-mcupm"},
	{ .compatible = "mediatek,mcupm"},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_scp_of_match);

static struct platform_driver mtk_mcupm_driver = {
	.probe = mcupm_probe,
	.remove = mcupm_remove,
	.driver = {
		.name = "mtk-mcupm",
		.of_match_table = of_match_ptr(mtk_mcupm_of_match),
	},
};

module_platform_driver(mtk_mcupm_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek MCUPM control driver");

