// SPDX-License-Identifier: GPL-2.0-only
/*
 * Dynamic DMA mapping support.
 *
 * This implementation is a fallback for platforms that do not support
 * I/O TLBs (aka DMA address translation hardware).
 * Copyright (C) 2000 Asit Mallick <Asit.K.Mallick@intel.com>
 * Copyright (C) 2000 Goutham Rao <goutham.rao@intel.com>
 * Copyright (C) 2000, 2003 Hewlett-Packard Co
 *	David Mosberger-Tang <davidm@hpl.hp.com>
 *
 * 03/05/07 davidm	Switch from PCI-DMA to generic device DMA API.
 * 00/12/13 davidm	Rename to swiotlb.c and add mark_clean() to avoid
 *			unnecessary i-cache flushing.
 * 04/07/.. ak		Better overflow handling. Assorted fixes.
 * 05/09/10 linville	Add support for syncing ranges, support syncing for
 *			DMA_BIDIRECTIONAL mappings, miscellaneous cleanup.
 * 08/12/11 beckyb	Add highmem support
 */

#define pr_fmt(fmt) "software IO TLB: " fmt

#include <linux/cache.h>
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/mm.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/swiotlb.h>
#include <linux/pfn.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/highmem.h>
#include <linux/gfp.h>
#include <linux/scatterlist.h>
#include <linux/mem_encrypt.h>
#include <linux/set_memory.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/slab.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#include <asm/io.h>
#include <asm/dma.h>

#include <linux/init.h>
#include <linux/memblock.h>
#include <linux/iommu-helper.h>

#define CREATE_TRACE_POINTS
#include <trace/events/swiotlb.h>

#define OFFSET(val,align) ((unsigned long)	\
	                   ( (val) & ( (align) - 1)))

#define SLABS_PER_PAGE (1 << (PAGE_SHIFT - IO_TLB_SHIFT))

/*
 * Minimum IO TLB size to bother booting with.  Systems with mainly
 * 64bit capable cards will only lightly use the swiotlb.  If we can't
 * allocate a contiguous 1MB, we're probably in trouble anyway.
 */
#define IO_TLB_MIN_SLABS ((1<<20) >> IO_TLB_SHIFT)
#define INVALID_PHYS_ADDR (~(phys_addr_t)0)

enum swiotlb_force swiotlb_force;

struct io_tlb_mem io_tlb_default_mem;

/*
 * Max segment that we can provide which (if pages are contingous) will
 * not be bounced (unless SWIOTLB_FORCE is set).
 */
static unsigned int max_segment;

static int late_alloc;

static int __init
setup_io_tlb_npages(char *str)
{
	struct io_tlb_mem *mem = &io_tlb_default_mem;

	if (isdigit(*str)) {
		mem->nslabs = simple_strtoul(str, &str, 0);
		/* avoid tail segment of size < IO_TLB_SEGSIZE */
		mem->nslabs = ALIGN(mem->nslabs, IO_TLB_SEGSIZE);
	}
	if (*str == ',')
		++str;
	if (!strcmp(str, "force")) {
		swiotlb_force = SWIOTLB_FORCE;
	} else if (!strcmp(str, "noforce")) {
		swiotlb_force = SWIOTLB_NO_FORCE;
		mem->nslabs = 1;
	}

	return 0;
}
early_param("swiotlb", setup_io_tlb_npages);

static bool no_iotlb_memory;

unsigned long swiotlb_nr_tbl(void)
{
	return unlikely(no_iotlb_memory) ? 0 : io_tlb_default_mem.nslabs;
}
EXPORT_SYMBOL_GPL(swiotlb_nr_tbl);

unsigned int swiotlb_max_segment(void)
{
	return unlikely(no_iotlb_memory) ? 0 : max_segment;
}
EXPORT_SYMBOL_GPL(swiotlb_max_segment);

void swiotlb_set_max_segment(unsigned int val)
{
	if (swiotlb_force == SWIOTLB_FORCE)
		max_segment = 1;
	else
		max_segment = rounddown(val, PAGE_SIZE);
}

unsigned long swiotlb_size_or_default(void)
{
	unsigned long size;

	size = io_tlb_default_mem.nslabs << IO_TLB_SHIFT;

	return size ? size : (IO_TLB_DEFAULT_SIZE);
}

void __init swiotlb_adjust_size(unsigned long new_size)
{
	struct io_tlb_mem *mem = &io_tlb_default_mem;
	unsigned long size;

	/*
	 * If swiotlb parameter has not been specified, give a chance to
	 * architectures such as those supporting memory encryption to
	 * adjust/expand SWIOTLB size for their use.
	 */
	if (!mem->nslabs) {
		size = ALIGN(new_size, 1 << IO_TLB_SHIFT);
		mem->nslabs = size >> IO_TLB_SHIFT;
		mem->nslabs = ALIGN(mem->nslabs, IO_TLB_SEGSIZE);

		pr_info("SWIOTLB bounce buffer size adjusted to %luMB", size >> 20);
	}
}

void swiotlb_print_info(void)
{
	struct io_tlb_mem *mem = &io_tlb_default_mem;
	unsigned long bytes = mem->nslabs << IO_TLB_SHIFT;

	if (no_iotlb_memory) {
		pr_warn("No low mem\n");
		return;
	}

	pr_info("mapped [mem %pa-%pa] (%luMB)\n", &mem->start, &mem->end,
	       bytes >> 20);
}

/*
 * Early SWIOTLB allocation may be too early to allow an architecture to
 * perform the desired operations.  This function allows the architecture to
 * call SWIOTLB when the operations are possible.  It needs to be called
 * before the SWIOTLB memory is used.
 */
void __init swiotlb_update_mem_attributes(void)
{
	struct io_tlb_mem *mem = &io_tlb_default_mem;
	void *vaddr;
	unsigned long bytes;

	if (no_iotlb_memory || late_alloc)
		return;

	vaddr = phys_to_virt(mem->start);
	bytes = PAGE_ALIGN(mem->nslabs << IO_TLB_SHIFT);
	set_memory_decrypted((unsigned long)vaddr, bytes >> PAGE_SHIFT);
	memset(vaddr, 0, bytes);
}

int __init swiotlb_init_with_tbl(char *tlb, unsigned long nslabs, int verbose)
{
	struct io_tlb_mem *mem = &io_tlb_default_mem;
	unsigned long i, bytes;
	size_t alloc_size;

	bytes = nslabs << IO_TLB_SHIFT;

	mem->nslabs = nslabs;
	mem->start = __pa(tlb);
	mem->end = mem->start + bytes;

	/*
	 * Allocate and initialize the free list array.  This array is used
	 * to find contiguous free memory regions of size up to IO_TLB_SEGSIZE
	 * between mem->start and mem->end.
	 */
	alloc_size = PAGE_ALIGN(mem->nslabs * sizeof(int));
	mem->list = memblock_alloc(alloc_size, PAGE_SIZE);
	if (!mem->list)
		panic("%s: Failed to allocate %zu bytes align=0x%lx\n",
		      __func__, alloc_size, PAGE_SIZE);

	alloc_size = PAGE_ALIGN(mem->nslabs * sizeof(phys_addr_t));
	mem->orig_addr = memblock_alloc(alloc_size, PAGE_SIZE);
	if (!mem->orig_addr)
		panic("%s: Failed to allocate %zu bytes align=0x%lx\n",
		      __func__, alloc_size, PAGE_SIZE);

	for (i = 0; i < mem->nslabs; i++) {
		mem->list[i] = IO_TLB_SEGSIZE - OFFSET(i, IO_TLB_SEGSIZE);
		mem->orig_addr[i] = INVALID_PHYS_ADDR;
	}
	mem->index = 0;

	if (verbose)
		swiotlb_print_info();

	swiotlb_set_max_segment(mem->nslabs << IO_TLB_SHIFT);
	spin_lock_init(&mem->lock);

	return 0;
}

/*
 * Statically reserve bounce buffer space and initialize bounce buffer data
 * structures for the software IO TLB used to implement the DMA API.
 */
void  __init
swiotlb_init(int verbose)
{
	struct io_tlb_mem *mem = &io_tlb_default_mem;
	size_t default_size = IO_TLB_DEFAULT_SIZE;
	unsigned char *vstart;
	unsigned long bytes;

	if (!mem->nslabs) {
		mem->nslabs = (default_size >> IO_TLB_SHIFT);
		mem->nslabs = ALIGN(mem->nslabs, IO_TLB_SEGSIZE);
	}

	bytes = mem->nslabs << IO_TLB_SHIFT;

	/* Get IO TLB memory from the low pages */
	vstart = memblock_alloc_low(PAGE_ALIGN(bytes), PAGE_SIZE);
	if (vstart && !swiotlb_init_with_tbl(vstart, mem->nslabs, verbose))
		return;

	if (mem->start) {
		memblock_free_early(mem->start,
				    PAGE_ALIGN(mem->nslabs << IO_TLB_SHIFT));
		mem->start = 0;
	}
	pr_warn("Cannot allocate buffer");
	no_iotlb_memory = true;
}

/*
 * Systems with larger DMA zones (those that don't support ISA) can
 * initialize the swiotlb later using the slab allocator if needed.
 * This should be just like above, but with some error catching.
 */
int
swiotlb_late_init_with_default_size(size_t default_size)
{
	struct io_tlb_mem *mem = &io_tlb_default_mem;
	unsigned long bytes, req_nslabs = mem->nslabs;
	unsigned char *vstart = NULL;
	unsigned int order;
	int rc = 0;

	if (!mem->nslabs) {
		mem->nslabs = (default_size >> IO_TLB_SHIFT);
		mem->nslabs = ALIGN(mem->nslabs, IO_TLB_SEGSIZE);
	}

	/*
	 * Get IO TLB memory from the low pages
	 */
	order = get_order(mem->nslabs << IO_TLB_SHIFT);
	mem->nslabs = SLABS_PER_PAGE << order;
	bytes = mem->nslabs << IO_TLB_SHIFT;

	while ((SLABS_PER_PAGE << order) > IO_TLB_MIN_SLABS) {
		vstart = (void *)__get_free_pages(GFP_DMA | __GFP_NOWARN,
						  order);
		if (vstart)
			break;
		order--;
	}

	if (!vstart) {
		mem->nslabs = req_nslabs;
		return -ENOMEM;
	}
	if (order != get_order(bytes)) {
		pr_warn("only able to allocate %ld MB\n",
			(PAGE_SIZE << order) >> 20);
		mem->nslabs = SLABS_PER_PAGE << order;
	}
	rc = swiotlb_late_init_with_tbl(vstart, mem->nslabs);
	if (rc)
		free_pages((unsigned long)vstart, order);

	return rc;
}

static void swiotlb_cleanup(void)
{
	struct io_tlb_mem *mem = &io_tlb_default_mem;

	mem->end = 0;
	mem->start = 0;
	mem->nslabs = 0;
	max_segment = 0;
}

static int swiotlb_init_io_tlb_mem(struct io_tlb_mem *mem, phys_addr_t start,
				   size_t size)
{
	unsigned long i;
	void *vaddr = phys_to_virt(start);

	size = ALIGN(size, 1 << IO_TLB_SHIFT);
	mem->nslabs = size >> IO_TLB_SHIFT;
	mem->nslabs = ALIGN(mem->nslabs, IO_TLB_SEGSIZE);

	mem->start = start;
	mem->end = mem->start + size;

	set_memory_decrypted((unsigned long)vaddr, size >> PAGE_SHIFT);
	memset(vaddr, 0, size);

	/*
	 * Allocate and initialize the free list array.  This array is used
	 * to find contiguous free memory regions of size up to IO_TLB_SEGSIZE
	 * between mem->start and mem->end.
	 */
	mem->list = (unsigned int *)__get_free_pages(GFP_KERNEL,
	                              get_order(mem->nslabs * sizeof(int)));
	if (!mem->list)
		goto cleanup3;

	mem->orig_addr = (phys_addr_t *)
		__get_free_pages(GFP_KERNEL,
				 get_order(mem->nslabs *
					   sizeof(phys_addr_t)));
	if (!mem->orig_addr)
		goto cleanup4;

	for (i = 0; i < mem->nslabs; i++) {
		mem->list[i] = IO_TLB_SEGSIZE - OFFSET(i, IO_TLB_SEGSIZE);
		mem->orig_addr[i] = INVALID_PHYS_ADDR;
	}
	mem->index = 0;

	spin_lock_init(&mem->lock);

	return 0;

cleanup4:
	free_pages((unsigned long)mem->list,
		   get_order(mem->nslabs * sizeof(int)));
	mem->list = NULL;
cleanup3:
	swiotlb_cleanup();
	return -ENOMEM;
}

int swiotlb_late_init_with_tbl(char *tlb, unsigned long nslabs)
{
	struct io_tlb_mem *mem = &io_tlb_default_mem;
	unsigned long bytes = nslabs << IO_TLB_SHIFT;
	int ret;

	ret = swiotlb_init_io_tlb_mem(mem, virt_to_phys(tlb), bytes);
	if (ret)
		return ret;

	no_iotlb_memory = false;

	swiotlb_print_info();

	late_alloc = 1;

	swiotlb_set_max_segment(bytes);

	return 0;
}

void __init swiotlb_exit(void)
{
	struct io_tlb_mem *mem = &io_tlb_default_mem;

	if (!mem->orig_addr)
		return;

	if (late_alloc) {
		free_pages((unsigned long)mem->orig_addr,
			   get_order(mem->nslabs * sizeof(phys_addr_t)));
		free_pages((unsigned long)mem->list,
			   get_order(mem->nslabs * sizeof(int)));
		free_pages((unsigned long)phys_to_virt(mem->start),
			   get_order(mem->nslabs << IO_TLB_SHIFT));
	} else {
		memblock_free_late(__pa(mem->orig_addr),
				   PAGE_ALIGN(mem->nslabs * sizeof(phys_addr_t)));
		memblock_free_late(__pa(mem->list),
				   PAGE_ALIGN(mem->nslabs * sizeof(int)));
		memblock_free_late(mem->start,
				   PAGE_ALIGN(mem->nslabs << IO_TLB_SHIFT));
	}
	swiotlb_cleanup();
}

/*
 * Bounce: copy the swiotlb buffer from or back to the original dma location
 */
static void swiotlb_bounce(phys_addr_t orig_addr, phys_addr_t tlb_addr,
			   size_t size, enum dma_data_direction dir)
{
	unsigned long pfn = PFN_DOWN(orig_addr);
	unsigned char *vaddr = phys_to_virt(tlb_addr);

	if (PageHighMem(pfn_to_page(pfn))) {
		/* The buffer does not have a mapping.  Map it in and copy */
		unsigned int offset = orig_addr & ~PAGE_MASK;
		char *buffer;
		unsigned int sz = 0;
		unsigned long flags;

		while (size) {
			sz = min_t(size_t, PAGE_SIZE - offset, size);

			local_irq_save(flags);
			buffer = kmap_atomic(pfn_to_page(pfn));
			if (dir == DMA_TO_DEVICE)
				memcpy(vaddr, buffer + offset, sz);
			else
				memcpy(buffer + offset, vaddr, sz);
			kunmap_atomic(buffer);
			local_irq_restore(flags);

			size -= sz;
			pfn++;
			vaddr += sz;
			offset = 0;
		}
	} else if (dir == DMA_TO_DEVICE) {
		memcpy(vaddr, phys_to_virt(orig_addr), size);
	} else {
		memcpy(phys_to_virt(orig_addr), vaddr, size);
	}
}

static int swiotlb_tbl_find_free_region(struct device *hwdev,
					dma_addr_t tbl_dma_addr,
					size_t alloc_size,
					unsigned long attrs)
{
	struct io_tlb_mem *mem = get_io_tlb_mem(hwdev);
	unsigned long flags;
	unsigned int nslots, stride, index, wrap;
	int i;
	unsigned long mask;
	unsigned long offset_slots;
	unsigned long max_slots;
	unsigned long tmp_io_tlb_used;

	if (no_iotlb_memory && !hwdev->dma_io_tlb_mem)
		panic("Can not allocate SWIOTLB buffer earlier and can't now provide you with the DMA bounce buffer");

	mask = dma_get_seg_boundary(hwdev);

	tbl_dma_addr &= mask;

	offset_slots = ALIGN(tbl_dma_addr, 1 << IO_TLB_SHIFT) >> IO_TLB_SHIFT;

	/*
	 * Carefully handle integer overflow which can occur when mask == ~0UL.
	 */
	max_slots = mask + 1
		    ? ALIGN(mask + 1, 1 << IO_TLB_SHIFT) >> IO_TLB_SHIFT
		    : 1UL << (BITS_PER_LONG - IO_TLB_SHIFT);

	/*
	 * For mappings greater than or equal to a page, we limit the stride
	 * (and hence alignment) to a page size.
	 */
	nslots = ALIGN(alloc_size, 1 << IO_TLB_SHIFT) >> IO_TLB_SHIFT;
	if (alloc_size >= PAGE_SIZE)
		stride = (1 << (PAGE_SHIFT - IO_TLB_SHIFT));
	else
		stride = 1;

	BUG_ON(!nslots);

	/*
	 * Find suitable number of IO TLB entries size that will fit this
	 * request and allocate a buffer from that IO TLB pool.
	 */
	spin_lock_irqsave(&mem->lock, flags);

	if (unlikely(nslots > mem->nslabs - mem->used))
		goto not_found;

	index = ALIGN(mem->index, stride);
	if (index >= mem->nslabs)
		index = 0;
	wrap = index;

	do {
		while (iommu_is_span_boundary(index, nslots, offset_slots,
					      max_slots)) {
			index += stride;
			if (index >= mem->nslabs)
				index = 0;
			if (index == wrap)
				goto not_found;
		}

		/*
		 * If we find a slot that indicates we have 'nslots' number of
		 * contiguous buffers, we allocate the buffers from that slot
		 * and mark the entries as '0' indicating unavailable.
		 */
		if (mem->list[index] >= nslots) {
			int count = 0;

			for (i = index; i < (int) (index + nslots); i++)
				mem->list[i] = 0;
			for (i = index - 1; (OFFSET(i, IO_TLB_SEGSIZE) != IO_TLB_SEGSIZE - 1) && mem->list[i]; i--)
				mem->list[i] = ++count;

			/*
			 * Update the indices to avoid searching in the next
			 * round.
			 */
			mem->index = ((index + nslots) < mem->nslabs
				      ? (index + nslots) : 0);

			goto found;
		}
		index += stride;
		if (index >= mem->nslabs)
			index = 0;
	} while (index != wrap);

not_found:
	tmp_io_tlb_used = mem->used;

	spin_unlock_irqrestore(&mem->lock, flags);
	if (!(attrs & DMA_ATTR_NO_WARN) && printk_ratelimit())
		dev_warn(hwdev, "swiotlb buffer is full (sz: %zd bytes), total %lu (slots), used %lu (slots)\n",
			 alloc_size, mem->nslabs, tmp_io_tlb_used);
	return -ENOMEM;

found:
	mem->used += nslots;
	spin_unlock_irqrestore(&mem->lock, flags);

	return index;
}

static void swiotlb_tbl_release_region(struct device *hwdev, int index,
				       size_t size)
{
	struct io_tlb_mem *mem = get_io_tlb_mem(hwdev);
	unsigned long flags;
	int i, count, nslots = ALIGN(size, 1 << IO_TLB_SHIFT) >> IO_TLB_SHIFT;

	/*
	 * Return the buffer to the free list by setting the corresponding
	 * entries to indicate the number of contiguous entries available.
	 * While returning the entries to the free list, we merge the entries
	 * with slots below and above the pool being returned.
	 */
	spin_lock_irqsave(&mem->lock, flags);
	{
		count = ((index + nslots) < ALIGN(index + 1, IO_TLB_SEGSIZE) ?
			 mem->list[index + nslots] : 0);
		/*
		 * Step 1: return the slots to the free list, merging the
		 * slots with superceeding slots
		 */
		for (i = index + nslots - 1; i >= index; i--) {
			mem->list[i] = ++count;
			mem->orig_addr[i] = INVALID_PHYS_ADDR;
		}
		/*
		 * Step 2: merge the returned slots with the preceding slots,
		 * if available (non zero)
		 */
		for (i = index - 1; (OFFSET(i, IO_TLB_SEGSIZE) != IO_TLB_SEGSIZE -1) && mem->list[i]; i--)
			mem->list[i] = ++count;

		mem->used -= nslots;
	}
	spin_unlock_irqrestore(&mem->lock, flags);
}

phys_addr_t swiotlb_tbl_map_single(struct device *hwdev, phys_addr_t orig_addr,
		size_t mapping_size, size_t alloc_size,
		enum dma_data_direction dir, unsigned long attrs)
{
	struct io_tlb_mem *mem = get_io_tlb_mem(hwdev);
	dma_addr_t tbl_dma_addr = phys_to_dma_unencrypted(hwdev, mem->start);
	phys_addr_t tlb_addr;
	unsigned int nslots, index;
	int i;

	if (mem_encrypt_active())
		pr_warn_once("Memory encryption is active and system is using DMA bounce buffers\n");

	if (mapping_size > alloc_size) {
		dev_warn_once(hwdev, "Invalid sizes (mapping: %zd bytes, alloc: %zd bytes)",
			      mapping_size, alloc_size);
		return (phys_addr_t)DMA_MAPPING_ERROR;
	}

	index = swiotlb_tbl_find_free_region(hwdev, tbl_dma_addr, alloc_size,
					     attrs);
	if (index < 0)
		return (phys_addr_t)DMA_MAPPING_ERROR;

	tlb_addr = mem->start + (index << IO_TLB_SHIFT);

	/*
	 * Save away the mapping from the original address to the DMA address.
	 * This is needed when we sync the memory.  Then we sync the buffer if
	 * needed.
	 */
	nslots = ALIGN(alloc_size, 1 << IO_TLB_SHIFT) >> IO_TLB_SHIFT;
	for (i = 0; i < nslots; i++)
		mem->orig_addr[index+i] = orig_addr + (i << IO_TLB_SHIFT);
	if (!(attrs & DMA_ATTR_SKIP_CPU_SYNC) &&
	    (dir == DMA_TO_DEVICE || dir == DMA_BIDIRECTIONAL))
		swiotlb_bounce(orig_addr, tlb_addr, mapping_size, DMA_TO_DEVICE);

	return tlb_addr;
}

/*
 * tlb_addr is the physical address of the bounce buffer to unmap.
 */
void swiotlb_tbl_unmap_single(struct device *hwdev, phys_addr_t tlb_addr,
			      size_t mapping_size, size_t alloc_size,
			      enum dma_data_direction dir, unsigned long attrs)
{
	struct io_tlb_mem *mem = get_io_tlb_mem(hwdev);
	int index = (tlb_addr - mem->start) >> IO_TLB_SHIFT;
	phys_addr_t orig_addr = mem->orig_addr[index];

	/*
	 * First, sync the memory before unmapping the entry
	 */
	if (orig_addr != INVALID_PHYS_ADDR &&
	    !(attrs & DMA_ATTR_SKIP_CPU_SYNC) &&
	    ((dir == DMA_FROM_DEVICE) || (dir == DMA_BIDIRECTIONAL)))
		swiotlb_bounce(orig_addr, tlb_addr, mapping_size, DMA_FROM_DEVICE);

	swiotlb_tbl_release_region(hwdev, index, alloc_size);
}

void swiotlb_tbl_sync_single(struct device *hwdev, phys_addr_t tlb_addr,
			     size_t size, enum dma_data_direction dir,
			     enum dma_sync_target target)
{
	struct io_tlb_mem *mem = get_io_tlb_mem(hwdev);
	int index = (tlb_addr - mem->start) >> IO_TLB_SHIFT;
	phys_addr_t orig_addr = mem->orig_addr[index];

	if (orig_addr == INVALID_PHYS_ADDR)
		return;
	orig_addr += (unsigned long)tlb_addr & ((1 << IO_TLB_SHIFT) - 1);

	switch (target) {
	case SYNC_FOR_CPU:
		if (likely(dir == DMA_FROM_DEVICE || dir == DMA_BIDIRECTIONAL))
			swiotlb_bounce(orig_addr, tlb_addr,
				       size, DMA_FROM_DEVICE);
		else
			BUG_ON(dir != DMA_TO_DEVICE);
		break;
	case SYNC_FOR_DEVICE:
		if (likely(dir == DMA_TO_DEVICE || dir == DMA_BIDIRECTIONAL))
			swiotlb_bounce(orig_addr, tlb_addr,
				       size, DMA_TO_DEVICE);
		else
			BUG_ON(dir != DMA_FROM_DEVICE);
		break;
	default:
		BUG();
	}
}

/*
 * Create a swiotlb mapping for the buffer at @paddr, and in case of DMAing
 * to the device copy the data into it as well.
 */
dma_addr_t swiotlb_map(struct device *dev, phys_addr_t paddr, size_t size,
		enum dma_data_direction dir, unsigned long attrs)
{
	phys_addr_t swiotlb_addr;
	dma_addr_t dma_addr;

	trace_swiotlb_bounced(dev, phys_to_dma(dev, paddr), size,
			      swiotlb_force);

	swiotlb_addr = swiotlb_tbl_map_single(dev, paddr, size, size, dir,
			attrs);
	if (swiotlb_addr == (phys_addr_t)DMA_MAPPING_ERROR)
		return DMA_MAPPING_ERROR;

	/* Ensure that the address returned is DMA'ble */
	dma_addr = phys_to_dma_unencrypted(dev, swiotlb_addr);
	if (unlikely(!dma_capable(dev, dma_addr, size, true))) {
		swiotlb_tbl_unmap_single(dev, swiotlb_addr, size, size, dir,
			attrs | DMA_ATTR_SKIP_CPU_SYNC);
		dev_WARN_ONCE(dev, 1,
			"swiotlb addr %pad+%zu overflow (mask %llx, bus limit %llx).\n",
			&dma_addr, size, *dev->dma_mask, dev->bus_dma_limit);
		return DMA_MAPPING_ERROR;
	}

	if (!dev_is_dma_coherent(dev) && !(attrs & DMA_ATTR_SKIP_CPU_SYNC))
		arch_sync_dma_for_device(swiotlb_addr, size, dir);
	return dma_addr;
}

void *swiotlb_alloc(struct device *dev, size_t size, dma_addr_t *dma_handle,
		    unsigned long attrs)
{
	struct io_tlb_mem *mem = dev->dma_io_tlb_mem;
	int index;
	void *vaddr;
	phys_addr_t tlb_addr;

	size = PAGE_ALIGN(size);
	index = swiotlb_tbl_find_free_region(dev, mem->start, size, attrs);
	if (index < 0)
		return NULL;

	tlb_addr = mem->start + (index << IO_TLB_SHIFT);
	*dma_handle = phys_to_dma_unencrypted(dev, tlb_addr);

	if (!dev_is_dma_coherent(dev)) {
		unsigned long pfn = PFN_DOWN(tlb_addr);

		/* remove any dirty cache lines on the kernel alias */
		arch_dma_prep_coherent(pfn_to_page(pfn), size);

		/* create a coherent mapping */
		vaddr = dma_common_contiguous_remap(
			pfn_to_page(pfn), size,
			dma_pgprot(dev, PAGE_KERNEL, attrs),
			__builtin_return_address(0));
		if (!vaddr) {
			swiotlb_tbl_release_region(dev, index, size);
			return NULL;
		}
	} else {
		vaddr = phys_to_virt(tlb_addr);
	}

	memset(vaddr, 0, size);

	return vaddr;
}

void swiotlb_free(struct device *dev, size_t size, void *vaddr,
		  dma_addr_t dma_addr, unsigned long attrs)
{
	struct io_tlb_mem *mem = dev->dma_io_tlb_mem;
	unsigned int index;

	if (!dev_is_dma_coherent(dev))
		vunmap(vaddr);

	index = (dma_addr - mem->start) >> IO_TLB_SHIFT;
	swiotlb_tbl_release_region(dev, index, PAGE_ALIGN(size));
}

size_t swiotlb_max_mapping_size(struct device *dev)
{
	return ((size_t)1 << IO_TLB_SHIFT) * IO_TLB_SEGSIZE;
}

bool is_swiotlb_active(struct device *dev)
{
	/*
	 * When SWIOTLB is initialized, even if mem->start points to physical
	 * address zero, mem->end surely doesn't.
	 */
	return io_tlb_default_mem.end != 0 || dev->dma_io_tlb_mem;
}

#ifdef CONFIG_DEBUG_FS

static void swiotlb_create_debugfs(struct io_tlb_mem *mem, const char *name,
				   struct dentry *node)
{
	mem->debugfs = debugfs_create_dir(name, node);
	debugfs_create_ulong("io_tlb_nslabs", 0400, mem->debugfs, &mem->nslabs);
	debugfs_create_ulong("io_tlb_used", 0400, mem->debugfs, &mem->used);
}

static int __init swiotlb_create_default_debugfs(void)
{
	swiotlb_create_debugfs(&io_tlb_default_mem, "swiotlb", NULL);

	return 0;
}

late_initcall(swiotlb_create_default_debugfs);

#endif

static int rmem_swiotlb_device_init(struct reserved_mem *rmem,
				    struct device *dev)
{
	struct io_tlb_mem *mem = rmem->priv;
	int ret;

	if (dev->dma_io_tlb_mem)
		return -EBUSY;

	if (!mem) {
		mem = kzalloc(sizeof(*mem), GFP_KERNEL);
		if (!mem)
			return -ENOMEM;

		if (!memremap(rmem->base, rmem->size, MEMREMAP_WB)) {
			ret = -EINVAL;
			goto cleanup;
		}

		ret = swiotlb_init_io_tlb_mem(mem, rmem->base, rmem->size);
		if (ret)
			goto cleanup;

		rmem->priv = mem;
	}

#ifdef CONFIG_DEBUG_FS
	swiotlb_create_debugfs(mem, rmem->name, io_tlb_default_mem.debugfs);
#endif

	dev->dma_io_tlb_mem = mem;

	return 0;

cleanup:
	kfree(mem);

	return ret;
}

static void rmem_swiotlb_device_release(struct reserved_mem *rmem,
					struct device *dev)
{
	if (!dev)
		return;

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(dev->dma_io_tlb_mem->debugfs);
#endif
	dev->dma_io_tlb_mem = NULL;
}

static const struct reserved_mem_ops rmem_swiotlb_ops = {
	.device_init	= rmem_swiotlb_device_init,
	.device_release	= rmem_swiotlb_device_release,
};

static int __init rmem_swiotlb_setup(struct reserved_mem *rmem)
{
	unsigned long node = rmem->fdt_node;

	if (of_get_flat_dt_prop(node, "reusable", NULL) ||
	    of_get_flat_dt_prop(node, "linux,cma-default", NULL) ||
	    of_get_flat_dt_prop(node, "linux,dma-default", NULL) ||
	    of_get_flat_dt_prop(node, "no-map", NULL))
		return -EINVAL;

	rmem->ops = &rmem_swiotlb_ops;
	pr_info("Reserved memory: created device swiotlb memory pool at %pa, size %ld MiB\n",
		&rmem->base, (unsigned long)rmem->size / SZ_1M);
	return 0;
}

RESERVEDMEM_OF_DECLARE(dma, "restricted-dma-pool", rmem_swiotlb_setup);
