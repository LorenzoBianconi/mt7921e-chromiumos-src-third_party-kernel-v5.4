/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_SWIOTLB_H
#define __LINUX_SWIOTLB_H

#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/limits.h>

struct page;
struct scatterlist;

enum swiotlb_force {
	SWIOTLB_NORMAL,		/* Default - depending on HW DMA mask etc. */
	SWIOTLB_FORCE,		/* swiotlb=force */
	SWIOTLB_NO_FORCE,	/* swiotlb=noforce */
};

/*
 * Maximum allowable number of contiguous slabs to map,
 * must be a power of 2.  What is the appropriate value ?
 * The complexity of {map,unmap}_single is linearly dependent on this value.
 */
#define IO_TLB_SEGSIZE	128

/*
 * log of the size of each IO TLB slab.  The number of slabs is command line
 * controllable.
 */
#define IO_TLB_SHIFT 11

/* default to 64MB */
#define IO_TLB_DEFAULT_SIZE (64UL<<20)

extern void swiotlb_init(int verbose);
int swiotlb_init_with_tbl(char *tlb, unsigned long nslabs, int verbose);
extern unsigned long swiotlb_nr_tbl(void);
unsigned long swiotlb_size_or_default(void);
extern int swiotlb_late_init_with_tbl(char *tlb, unsigned long nslabs);
extern int swiotlb_late_init_with_default_size(size_t default_size);
extern void __init swiotlb_update_mem_attributes(void);

/*
 * Enumeration for sync targets
 */
enum dma_sync_target {
	SYNC_FOR_CPU = 0,
	SYNC_FOR_DEVICE = 1,
};

phys_addr_t swiotlb_tbl_map_single(struct device *hwdev, phys_addr_t phys,
		size_t mapping_size, size_t alloc_size,
		enum dma_data_direction dir, unsigned long attrs);

extern void swiotlb_tbl_unmap_single(struct device *hwdev,
				     phys_addr_t tlb_addr,
				     size_t mapping_size,
				     size_t alloc_size,
				     enum dma_data_direction dir,
				     unsigned long attrs);

extern void swiotlb_tbl_sync_single(struct device *hwdev,
				    phys_addr_t tlb_addr,
				    size_t size, enum dma_data_direction dir,
				    enum dma_sync_target target);

dma_addr_t swiotlb_map(struct device *dev, phys_addr_t phys,
		size_t size, enum dma_data_direction dir, unsigned long attrs);

void *swiotlb_alloc(struct device *dev, size_t size, dma_addr_t *dma_handle,
		    unsigned long attrs);

void swiotlb_free(struct device *dev, size_t size, void *vaddr,
		  dma_addr_t dma_addr, unsigned long attrs);

#ifdef CONFIG_SWIOTLB
extern enum swiotlb_force swiotlb_force;

/**
 * struct io_tlb_mem - IO TLB Memory Pool Descriptor
 *
 * @start:	The start address of the swiotlb memory pool. Used to do a quick
 *		range check to see if the memory was in fact allocated by this
 *		API. For restricted DMA pool, this is device tree adjustable.
 * @end:	The end address of the swiotlb memory pool. Used to do a quick
 *		range check to see if the memory was in fact allocated by this
 *		API. For restricted DMA pool, this is device tree adjustable.
 * @nslabs:	The number of IO TLB blocks (in groups of 64) between @start and
 *		@end. For default swiotlb, this is command line adjustable via
 *		setup_io_tlb_npages.
 * @used:	The number of used IO TLB block.
 * @list:	The free list describing the number of free entries available
 *		from each index.
 * @index:	The index to start searching in the next round.
 * @orig_addr:	The original address corresponding to a mapped entry for the
 *		sync operations.
 * @lock:	The lock to protect the above data structures in the map and
 *		unmap calls.
 * @debugfs:	The dentry to debugfs.
 */
struct io_tlb_mem {
	phys_addr_t start;
	phys_addr_t end;
	unsigned long nslabs;
	unsigned long used;
	unsigned int *list;
	unsigned int index;
	phys_addr_t *orig_addr;
	spinlock_t lock;
	struct dentry *debugfs;
};
extern struct io_tlb_mem io_tlb_default_mem;

static inline struct io_tlb_mem *get_io_tlb_mem(struct device *dev)
{
	return dev->dma_io_tlb_mem ? dev->dma_io_tlb_mem : &io_tlb_default_mem;
}

static inline bool is_swiotlb_buffer(struct device *dev, phys_addr_t paddr)
{
	struct io_tlb_mem *mem = get_io_tlb_mem(dev);

	return paddr >= mem->start && paddr < mem->end;
}

void __init swiotlb_exit(void);
unsigned int swiotlb_max_segment(void);
size_t swiotlb_max_mapping_size(struct device *dev);
bool is_swiotlb_active(struct device *dev);
void __init swiotlb_adjust_size(unsigned long new_size);
#else
#define swiotlb_force SWIOTLB_NO_FORCE
static inline bool is_swiotlb_buffer(struct device *dev, phys_addr_t paddr)
{
	return false;
}
static inline void swiotlb_exit(void)
{
}
static inline unsigned int swiotlb_max_segment(void)
{
	return 0;
}
static inline size_t swiotlb_max_mapping_size(struct device *dev)
{
	return SIZE_MAX;
}

static inline bool is_swiotlb_active(struct device *dev)
{
	return false;
}

static inline void swiotlb_adjust_size(unsigned long new_size)
{
}
#endif /* CONFIG_SWIOTLB */

extern void swiotlb_print_info(void);
extern void swiotlb_set_max_segment(unsigned int);

#endif /* __LINUX_SWIOTLB_H */
