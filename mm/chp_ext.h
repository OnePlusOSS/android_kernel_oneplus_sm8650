#ifndef _CHP_EXT_H_
#define _CHP_EXT_H_
// prepare for kernel module

#ifdef CONFIG_CONT_PTE_HUGEPAGE
#include <linux/mm.h>
struct chp_ext_order {
	union {
		struct {
			unsigned int order : 8;
			unsigned int magic : 16;
			unsigned int type : 8;
		};
		unsigned int nr;
	};
};

enum chp_ext_type {
	CHP_EXT_ZSMALLOC = 0,
	CHP_EXT_GPU,
	CHP_EXT_DMABUF,

	NR_CHP_EXT_TYPES,
};

enum chp_ext_cmd {
	CHP_EXT_CMD_KERNEL_SUPPORT_CHP = 0,
	CHP_EXT_CMD_CHP_POOL,
	CHP_EXT_CMD_POOL_TOTAL_CMA_COUNT,
	CHP_EXT_CMD_POOL_CMA_COUNT,
	CHP_EXT_CMD_POOL_BUDDY_COUNT,
	CHP_EXT_CMD_POOL_PAGES,
	CHP_EXT_CMD_POOL_WM_HIGH,
};

static inline struct page *alloc_chp_ext_wrapper(gfp_t gfp_mask, int type)
{
	struct chp_ext_order ceo = {
		.order = HPAGE_CONT_PTE_ORDER,
		.magic = THP_SWAP_PRIO_MAGIC,
		.type = type,
	};

	return alloc_pages(gfp_mask, ceo.nr);
}

static inline void __free_pages_ext(struct page *page, unsigned int order)
{
	if (unlikely(order == HPAGE_CONT_PTE_ORDER && PageContExtAlloc(page)))
		put_page(page);
	else
		__free_pages(page, order);
}

static inline bool is_chp_ext_pages(struct page *page, unsigned int order)
{
	return (order == HPAGE_CONT_PTE_ORDER && PageContExtAlloc(page));
}

static inline long chp_read_info_ext(int cmd)
{
	struct sysinfo si = {
		.procs		= THP_SWAP_PRIO_MAGIC,
		.totalhigh	= cmd,
	};

	si_meminfo(&si);
	return (long)si.freehigh;
}
#endif /* CONFIG_CONT_PTE_HUGEPAGE */
#endif /* _CHP_EXT_H_ */
