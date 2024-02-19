// SPDX-License-Identifier: GPL-2.0-only
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */
#include <asm/pgtable-types.h>
#include <asm/pgalloc.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/pagemap.h>
#include <linux/rmap.h>
#include <linux/init.h>
#include <linux/mmu_notifier.h>
#include <linux/gfp.h>
#include <linux/string.h>
#include <linux/mm_inline.h>
#include <linux/backing-dev.h>
#include <linux/proc_fs.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/page_owner.h>
#include <linux/swapops.h>
#include <uapi/linux/sched/types.h>

#include <linux/mm_types.h>
#include <linux/huge_mm.h>
#include <linux/shmem_fs.h>
#include <linux/cma.h>
#include <linux/memblock.h>
#include <linux/sched/mm.h>
#include <linux/oom.h>
#include <linux/psi.h>
#if CONFIG_CHP_ABNORMAL_PTES_DEBUG
#include <linux/sched.h>
#endif

#include "cma.h"
#include "chp_ext.h"
#include "internal.h"
#include <trace/hooks/sched.h>

#define endio_spinlock android_kabi_reserved2

DEFINE_STATIC_KEY_FALSE(cont_pte_huge_page_enabled_key);

struct vm_chp_event_state {
	unsigned long event[NR_VM_CHP_EVENT_ITEMS];
};
DEFINE_PER_CPU(struct vm_chp_event_state, vm_chp_event_states) = {{0}};

static DEFINE_SPINLOCK(uid_blacklist_lock);
#define MAX_UID_BLACKLIST_SIZE 96
struct uid_blacklist {
	uid_t array[MAX_UID_BLACKLIST_SIZE];
	short size;
};
struct uid_blacklist *ub;

static bool config_anon_enable = true;
static bool config_alloc_oom = false;
bool config_bug_on = false;
bool supported_oat_hugepage = true;

struct cma *cont_pte_cma;
static struct huge_page_pool g_cont_pte_pool;
struct cont_pte_huge_page_stat perf_stat;

static unsigned long sysctl_max_chp_buddy_count;

/* cmdline */
unsigned long cont_pte_pool_cma_size;
static unsigned long cont_pte_sup_mem;
static bool cont_pte_sup_prjname;
/* true by default, false when cont_pte_hugepage=off in cmdline */
static bool cont_pte_hugepage_enable = false;
bool cma_chunk_refill_ready;

#define M2N(SZ) ((SZ) / HPAGE_CONT_PTE_SIZE)
#define HIGH_ORDER_GFP ((GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN \
			 | __GFP_NORETRY) & ~__GFP_RECLAIM & ~__GFP_MOVABLE)
#define CONT_PTE_SUP_MEM_SIZE (8ul * SZ_1G)
#define MEM_SIZE_8G  (8ul * SZ_1G)
#define MEM_SIZE_12G (12ul * SZ_1G)
/* The default chunk size is 64M, avg cma chunk refill time 747ms */
#define CONT_PTE_CMA_CHUNK_ORDER	14
#define CONT_PTE_CMA_CHUNK_SIZE		(1 << (CONT_PTE_CMA_CHUNK_ORDER + PAGE_SHIFT))
#define CONT_PTE_PAGES_PER_CHUNK	(1 << CONT_PTE_CMA_CHUNK_ORDER)
/* map up to 8 hugepages in case we are eligible for hugepage mapping */
#define MAX_HUGEPAGE_MAPAROUND 8
#define NR_FAULT_AROUND_STAT_ITEMS (MAX_HUGEPAGE_MAPAROUND + 1)

#define ROOT_APP_UID KUIDT_INIT(0)
#define SYSTEM_APP_UID KUIDT_INIT(1000)

static atomic64_t fault_around_stat[NR_FAULT_AROUND_STAT_ITEMS] __cacheline_aligned_in_smp;

unsigned long swap_cluster_double_mapped;
atomic_long_t cont_pte_double_map_count;
atomic64_t thp_swpin_hit_swapcache;
atomic64_t thp_cow;
atomic64_t thp_cow_fallback;

char *vm_chp_event_text[NR_VM_CHP_EVENT_ITEMS] = {
	"page_alloc_slow_path",
	"page_alloc_failed",

	"madv_free",
	"madv_dont_need_unaligned",

	"refill_kworker_wake_up",
	"refill_kworker_alloc_success",

	"refill_extalloc",
	"zsmalloc",
	"gpu",
	"dmabuf",
	"alloc_from_pool_buddy",

	"thp_do_anon_pages",
	"thp_do_anon_pages_fallback",

	"thp_swpin_no_swapcache_entry",
	"thp_swpin_no_swapcache_alloc_success",
	"thp_swpin_no_swapcache_alloc_fail",
	"thp_swpin_no_swapcache_fallback_entry",
	"thp_swpin_no_swapcache_fallback_alloc_success",
	"thp_swpin_no_swapcache_fallback_alloc_fail",
	"thp_swpin_swapcache_entry",
	"thp_swpin_swapcache_alloc_success",
	"thp_swpin_swapcache_prepare_fail",
	"thp_swpin_swapcache_fallback_entry",
	"thp_swpin_swapcache_fallback_alloc_success",
	"thp_swpin_swapcache_fallback_alloc_fail",

	"thp_file_entry",
	"thp_file_alloc_success",
	"thp_file_alloc_fail",

	"thp_swpin_critical_entry",
	"thp_swpin_critical_fallback"
};

char *thp_read_swpcache_ret_status_string[RET_STATUS_NR] = {
	"ret_status_alloc_thp_success",
	"ret_status_no_swp_info",
	"ret_status_hit_swpcache",
	"ret_status_no_cluster_info",
	"ret_status_zero_swpcount",
	"ret_status_alloc_thp_fail",
	"ret_status_swpcache_prepare_fail",
	"ret_status_add_to_swpcache_fail",
	"ret_status_memcg_charge_fail",
	"ret_status_other_fail",
};

char *wp_reuse_fail_text[WP_REUSE_FAIL_NR] = {
	"wp_reuse_fail_total",
	"pte_no_same",
	"pte_no_readonly",
	"zero_ref_count",
};

#if CONFIG_REUSE_SWP_ACCOUNT_DEBUG
char *reuse_swp_text[REUSE_SWP_NR] = {
	"normal_reuse_swp_wb",
	"normal_reuse_swp_no_wb",
	"normal_reuse_swp_wb_err",
	"chp_reuse_swp_wb",
	"chp_reuse_swp_no_wb",
	"chp_reuse_swp_wb_err",
};
#endif

enum huge_page_pool_flags {
	HPP_WORKER_RUNNING,
};

#if CONFIG_POOL_ASYNC_RECLAIM
wait_queue_head_t pool_direct_reclaim_wait[MAX_NUMNODES];
#endif

#define DEFINE_CHP_SYSFS_ATTRIBUTE(__name)				\
static ssize_t __name ## _show(struct kobject *kobj,			\
			       struct kobj_attribute *attr, char *buf)	\
{									\
	return scnprintf(buf, PAGE_SIZE, "%d\n", config_ ## __name);	\
}									\
									\
static ssize_t __name ## _store(struct kobject *kobj,			\
				struct kobj_attribute *attr,		\
				const char *buf, size_t count)		\
{									\
	int val, ret;							\
									\
	ret = kstrtoint(buf, 10, &val);					\
	if (ret)							\
		return ret;						\
									\
	config_ ## __name = !!val;					\
	chp_logi("write val:%d\n", config_ ## __name);			\
	return count;							\
}									\
									\
static struct kobj_attribute __name ## _attr =				\
	__ATTR(__name, 0644, __name ## _show,  __name ## _store);	\

inline bool current_is_hybridswapd(void)
{
	if (unlikely(!(current->flags & PF_KTHREAD)))
		return false;

	return strncmp(current->comm, "hybridswapd",
		       sizeof("hybridswapd") - 1) == 0;
}

static bool find_uid_in_blacklist(uid_t uid);

inline bool cont_pte_huge_page_enabled(void)
{
	return static_branch_likely(&cont_pte_huge_page_enabled_key);
}

inline void count_vm_chp_events(enum vm_chp_event_item item, long delta)
{
	this_cpu_add(vm_chp_event_states.event[item], delta);
}

inline void count_vm_chp_event(enum vm_chp_event_item item)
{
	count_vm_chp_events(item, 1);
}

static void all_vm_chp_events(unsigned long *ret)
{
	int cpu;
	int i;

	memset(ret, 0, NR_VM_CHP_EVENT_ITEMS * sizeof(unsigned long));

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		struct vm_chp_event_state *this = &per_cpu(vm_chp_event_states,
							   cpu);

		for (i = 0; i < NR_VM_CHP_EVENT_ITEMS; i++)
			ret[i] += this->event[i];
	}
	cpus_read_unlock();
}

inline void mod_chp_page_state(struct page *page, long delta)
{
	int inx = HPAGE_POOL_CMA;

#if !CONFIG_CONT_PTE_EXT_MIGRATETYPE
	if (!within_cont_pte_cma(page_to_pfn(page)))
		inx = HPAGE_POOL_BUDDY;
#else
	if (!is_migrate_ext(get_pageblock_migratetype(page)))
		inx = HPAGE_POOL_BUDDY;
#endif
	atomic64_add(delta, &perf_stat.usage[inx]);
}

inline unsigned long chp_page_state(enum hpage_type t)
{
	return atomic64_read(&perf_stat.usage[t]);
}

inline bool is_thp_swap(struct swap_info_struct *si)
{
	return si && si->prio == THP_SWAP_PRIO_MAGIC;
}

inline bool is_cont_pte_cma(struct cma *cma)
{
	return cma == cont_pte_cma;
}

inline bool within_cont_pte_cma(unsigned long pfn)
{
	return cont_pte_cma && pfn >= cont_pte_cma->base_pfn &&
		pfn < cont_pte_cma->base_pfn + cont_pte_cma->count;
}

static inline void huge_page_pool_cnt_update(struct huge_page_pool *pool)
{
#if CONFIG_CONT_PTE_EXT_MIGRATETYPE
	pool->count[HPAGE_POOL_CMA] = ext_mt_order4_nr_free;
#endif
}

int huge_page_pool_count(struct huge_page_pool *pool, int inx)
{
	if (inx != HPAGE_POOL_BUDDY)
		huge_page_pool_cnt_update(pool);

	if (inx >= NR_HPAGE_POOL_TYPE)
		return pool->count[HPAGE_POOL_CMA] +
			pool->count[HPAGE_POOL_BUDDY];

	return pool->count[inx];
}

static void huge_page_pool_add(struct huge_page_pool *pool, struct page *page, int inx)
{
	CHP_BUG_ON(!IS_ALIGNED(page_to_pfn(page), HPAGE_CONT_PTE_NR));

#ifdef CONFIG_CONT_PTE_HUGEPAGE_DEBUG_VERBOSE
	int i;

	for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
		CHP_BUG_ON(atomic_read(&page[i]._refcount) != 1 && !refill);
		CHP_BUG_ON(atomic_read(&page[i]._mapcount) + 1);
		CHP_BUG_ON(PageCompound(&page[i]));
		WARN_ON_ONCE(page[i].flags); /* some new pages have 4000 0000 0000 0000, why? */
		WARN_ON_ONCE(page[i].private);
	}
#endif

	spin_lock(&pool->spinlock);
	list_add_tail(&page->lru, &pool->items[inx]);
	pool->count[inx]++;
	spin_unlock(&pool->spinlock);
}

static struct page *huge_page_pool_remove(struct huge_page_pool *pool, int inx)
{
	struct page *page;

	if (!cont_pte_huge_page_enabled())
		return NULL;

#if CONFIG_CONT_PTE_EXT_MIGRATETYPE
	/*
	 * so-called "CMA" pool has been mapped to ext_migratetype, we use
	 * ___GFP_CHP to tell buddy to get pages from those ext trip&quad
	 * migratetype
	 */
	if (inx == HPAGE_POOL_CMA) {
		page = alloc_pages(HIGH_ORDER_GFP | ___GFP_CHP, HPAGE_CONT_PTE_ORDER);
		if (page) {
			int i;
			split_page(page, HPAGE_CONT_PTE_ORDER);
			/* alloc pages only set the 1st page's private to 0 */
			for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
				set_page_private(&page[i], 0);
		}
		return page;
	}
#endif

	spin_lock(&pool->spinlock);
	page = list_first_entry_or_null(&pool->items[inx], struct page, lru);
	if (page) {
		pool->count[inx]--;
		list_del(&page->lru);
	}
	spin_unlock(&pool->spinlock);

	if (page && inx == HPAGE_POOL_BUDDY)
		count_vm_chp_event(CHP_ALLOC_FROM_BUDDY_POOL);

	if (chp_page_state(HPAGE_POOL_BUDDY) < sysctl_max_chp_buddy_count &&
	    pool->count[HPAGE_POOL_BUDDY] < pool->min_buddy / 2 &&
	    !test_bit(HPP_WORKER_RUNNING, &pool->flags)) {
		wake_up_process(pool->refill_worker);
	}
	return page;
}

#if !CONFIG_POOL_ASYNC_RECLAIM
static struct page *huge_page_pool_fetch(struct huge_page_pool *pool)
{
	struct page *page;

	page = huge_page_pool_remove(pool, HPAGE_POOL_CMA);
	if (!page)
		page = huge_page_pool_remove(pool, HPAGE_POOL_BUDDY);
	return page;
}
#endif

static unsigned long peak_chp_nr;
static struct page *alloc_chp_from_buddy(void)
{
	struct page *page;
	int i;
	unsigned long nr = chp_page_state(HPAGE_POOL_BUDDY);
	static unsigned long peak_jiffies;

	if (nr > sysctl_max_chp_buddy_count)
		return NULL;

#define STABLE_MAX_BUDDY_USAGE M2N(300 * SZ_1M)
	/*
	 * at boot stage, we allow more buddy memory to be used as hugepages.
	 * once it reaches the peak, we lower the size to the number a stable
	 * fragmentized buddy can lend to hugepages
	 */
	if (sysctl_max_chp_buddy_count != STABLE_MAX_BUDDY_USAGE) {
		if (nr > STABLE_MAX_BUDDY_USAGE && nr > peak_chp_nr) {
			peak_jiffies = jiffies;
			peak_chp_nr = nr;
		}

		if ((nr >= sysctl_max_chp_buddy_count) || /* we have used up the budget for boot */
		    (peak_jiffies != 0 && (jiffies - peak_jiffies) > 10 * HZ)) { /* peak durates for 10s */
			sysctl_max_chp_buddy_count = STABLE_MAX_BUDDY_USAGE;
			pr_info("cont_pte_hugepage: lower max buddy usage to %ld from peak %ld\n",
				sysctl_max_chp_buddy_count, peak_chp_nr);
		}
	}

	/*
	 * The slow path of the pool doesn't do any reclamation. gfp has no __GFP_CHP
	 * it won't use ext_migratetype
	 */
	page = alloc_pages(HIGH_ORDER_GFP, HPAGE_CONT_PTE_ORDER);
	if (page) {
		split_page(page, HPAGE_CONT_PTE_ORDER);
		/* alloc pages only set the 1st page's private to 0 */
		for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
			set_page_private(&page[i], 0);
	}

	return page;
}

static inline void huge_page_pool_chunk_refill(struct huge_page_pool *pool)
{
	int i;
	struct page *page;

#if !CONFIG_CONT_PTE_EXT_MIGRATETYPE
	unsigned long nr_chunk;
	int j;

	nr_chunk = cont_pte_pool_cma_size / CONT_PTE_CMA_CHUNK_SIZE;
	for (i = 0; i < nr_chunk; i++) {
		page = cma_alloc(cont_pte_cma, CONT_PTE_PAGES_PER_CHUNK,
				 HPAGE_CONT_PTE_ORDER,
				 GFP_KERNEL | __GFP_NOWARN);
		if (!page) {
			if (!atomic64_read(&perf_stat.chunk_refill_fail_count))
				perf_stat.chunk_refill_first_fail_num = i + 1; /* num + 1, differentiate init value */
			atomic64_inc(&perf_stat.chunk_refill_fail_count);
			pr_err("@@@%s fail to cma_alloc:%d\n", __func__, i);
			continue;
		}

		/*
		 * Fill in the list by cutting 64k page.
		 * For cma, split_page and set_page_private
		 * are already done.
		 */
		for (j = 0; j < CONT_PTE_PAGES_PER_CHUNK; j += HPAGE_CONT_PTE_NR)
			huge_page_pool_add(pool, &page[j], HPAGE_POOL_CMA);
	}
#endif
	cma_chunk_refill_ready = true;

	for (i = 0; i < pool->min_buddy; i++) {
		page = alloc_chp_from_buddy();
		if (!page)
			break;
		huge_page_pool_add(pool, page, HPAGE_POOL_BUDDY);
	}

	huge_page_pool_cnt_update(pool);
	chp_logi("pool_cma: %d pool_buddy: %d\n",
		  pool->count[HPAGE_POOL_CMA],
		  pool->count[HPAGE_POOL_BUDDY]);
}

static int huge_page_pool_refill_worker(void *data)
{
	struct huge_page_pool *pool = data;
	struct page *page;
	s64 time;
	int expect, i, retries, max_retries = 100;

	time = ktime_to_ms(ktime_get());
	huge_page_pool_chunk_refill(pool);
	perf_stat.chunk_refill_time = ktime_to_ms(ktime_get()) - time;

	for (;;) {
		time = ktime_to_ms(ktime_get());
		expect = max(pool->min_buddy -
			     huge_page_pool_count(pool, HPAGE_POOL_BUDDY), 0);
		retries = i = 0;
		count_vm_chp_event(CHP_REFILL_WORKER_WAKE_UP);

		set_bit(HPP_WORKER_RUNNING, &pool->flags);
		while (i < expect) {
			page = alloc_chp_from_buddy();
			if (!page) {
				if (retries++ >= max_retries) {
					pr_err("refill page timeout\n");
					break;
				}

				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(msecs_to_jiffies(100));
				continue;
			}

			huge_page_pool_add(pool, page, HPAGE_POOL_BUDDY);
			retries = 0;
			i++;
		}

		if (i > expect / 2)
			count_vm_chp_event(CHP_REFILL_WORKER_ALLOC_SUCCESS);

		set_current_state(TASK_INTERRUPTIBLE);
		if (unlikely(kthread_should_stop())) {
			set_current_state(TASK_RUNNING);
			break;
		}
		clear_bit(HPP_WORKER_RUNNING, &pool->flags);
		schedule();

		set_current_state(TASK_RUNNING);
	}
	return 0;
}

static int huge_page_pool_init(struct huge_page_pool *pool)
{
	struct sched_attr attr = { .sched_nice = 10 };
	const char *kworker_name = "khpage_poold";
	struct cpumask cpu_mask = { CPU_BITS_NONE };
	int ret, i;

	for (i = 0; i < NR_HPAGE_POOL_TYPE; i++) {
		pool->count[i] = 0;
		INIT_LIST_HEAD(&pool->items[i]);
	}

	spin_lock_init(&pool->spinlock);
	pool->cma_count = (cont_pte_pool_cma_size >> PAGE_SHIFT) / HPAGE_CONT_PTE_NR;

	if (cont_pte_pool_cma_size > 256 * SZ_1M)
		pool->high = M2N(90 * SZ_1M);
	else /* likely a small qemu system */
		pool->high = M2N(SZ_4M);

	pool->min_buddy = M2N(32 * SZ_1M);
	sysctl_max_chp_buddy_count = M2N(3 * SZ_512M);
	/* wakeup kthread on count < low, low = 3/4 high */
	pool->low = pool->high * 3 / 4;
#if CONFIG_POOL_ASYNC_RECLAIM
	pool->wmark[POOL_WMARK_MIN] = pool->high * 3 / 10; /* trigger direct reclaim at min/2 */
	pool->wmark[POOL_WMARK_LOW] = pool->high * 7 / 10; /* trigger kswapd */
	pool->wmark[POOL_WMARK_HIGH] = pool->high; /* stop reclaim order 4 pages */
#endif

	pool->refill_worker = kthread_run(huge_page_pool_refill_worker, pool,
					  kworker_name);
	/* TDOO if failed */
	if (IS_ERR(pool->refill_worker)) {
		chp_loge("failed to start %s\n", kworker_name);
		return -ENOMEM;
	}

	ret = sched_setattr(pool->refill_worker, &attr);
	if (ret)
		chp_loge("failed to set priority for %s ret=%d\n",
			 kworker_name, ret);

	/* TODO: move this to userspace, debug only */
	for (i = 0; i < 4; i++)
		cpumask_set_cpu(i, &cpu_mask);
	set_cpus_allowed_ptr(pool->refill_worker, &cpu_mask);

	chp_logi("success.\n");
	return 0;
}

struct huge_page_pool *cont_pte_pool(void)
{
	return &g_cont_pte_pool;
}

int cont_pte_pool_total_pages(void)
{
	struct huge_page_pool *pool = cont_pte_pool();

	huge_page_pool_cnt_update(pool);

	return (pool->count[HPAGE_POOL_CMA] +
		pool->count[HPAGE_POOL_BUDDY]) * HPAGE_CONT_PTE_NR;
}

int cont_pte_pool_high(void)
{
	struct huge_page_pool *pool = cont_pte_pool();

	return pool->high * HPAGE_CONT_PTE_NR;
}

bool cont_pte_pool_add(struct page *page)
{
	struct huge_page_pool *pool = cont_pte_pool();
	int inx;

	if (within_cont_pte_cma(page_to_pfn(page)))
		inx = HPAGE_POOL_CMA;
	else
		inx = HPAGE_POOL_BUDDY;

	huge_page_pool_add(pool, page, inx);
	return true;
}

/* copied from set_pte_at */
static inline void cset_pte_at(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep, pte_t pte)
{
	if (pte_present(pte) && pte_user_exec(pte) && !pte_special(pte))
		__sync_icache_dcache(pte);

	if (system_supports_mte() &&
	    pte_present(pte) && pte_tagged(pte) && !pte_special(pte))
		mte_sync_tags(pte); /* 6.1 use "void mte_sync_tags(pte_t pte)" */

	__check_racy_pte_update(mm, ptep, pte);

	set_pte(ptep, pte);
}

inline bool transhuge_cont_pte_vma_suitable(struct vm_area_struct *vma,
						   unsigned long haddr)
{
	if (!vma_is_anonymous(vma)) {
#if !CONFIG_CONT_PTE_FILE_HUGEPAGE_DISABLE
		struct inode *inode;

		if (!vma->vm_file || !vma->vm_file->f_inode)
			return false;

		inode = vma->vm_file->f_inode;

		if (!S_ISREG(inode->i_mode))
			return false;

		if (!inode->may_cont_pte)
			return false;

		/* vm_account means the vma was for writing */
		if (vma->vm_flags & (VM_WRITE | VM_ACCOUNT))
			return false;

		if (shmem_file(vma->vm_file))
			return false;

		if (!transhuge_cont_pte_vma_aligned(vma))
			return false;

		if (inode_is_open_for_write(inode))
			return false;
#else
		return false;
#endif
	} else {
#ifndef  CONFIG_CONT_PTE_HUGEPAGE_ON_QEMU
		if (!vma_is_chp_anonymous(vma))
			return false;
#endif

		if (vma->vm_start >= vma->vm_mm->start_brk &&
			vma->vm_end <= vma->vm_mm->brk)
			return false;

		if (vma->vm_flags & (VM_GROWSDOWN | VM_GROWSUP))
			return false;
	}

	return transhuge_cont_pte_addr_suitable(vma, haddr);
}

void __free_cont_pte_hugepages(struct page *page)
{
	int i;

	CHP_BUG_ON(page_ref_count(page) != 1);

	mod_chp_page_state(page, -1);
	/* try to refill pool before releasing */
	if (within_cont_pte_cma(page_to_pfn(page))) {
		struct huge_page_pool *pool = cont_pte_pool();
		huge_page_pool_add(pool, page, HPAGE_POOL_CMA);
	} else {
		for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
			__free_page(&page[i]);
	}
}

#if CONFIG_POOL_ASYNC_RECLAIM

#define POOL_ALLOC_WMARK_MIN         POOL_WMARK_MIN
#define POOL_ALLOC_WMARK_LOW         POOL_WMARK_LOW
#define POOL_ALLOC_WMARK_HIGH        POOL_WMARK_HIGH
#define POOL_ALLOC_NO_WATERMARKS     0x04 /* don't check watermarks at all */
/* Mask to get the watermark bits */
#define POOL_ALLOC_WMARK_MASK        (POOL_ALLOC_NO_WATERMARKS-1)


#define POOL_ALLOC_OOM		0x08 /* from oom path */
#define POOL_ALLOC_HARDER	0x10 /* try to alloc harder */
#define POOL_ALLOC_HIGH		0x20 /* eg: from rt thread */
#define POOL_ALLOC_ALL		0x40 /* alloc till 0 */

#define MAX_LEN_CHP_VMA_NAME (sizeof(DALVIK_MAIN_HEAP) + 1)
#define DALVIK_MAIN_HEAP_BIT (1ul << 63)

#define LIBC_SO "libc.so"
#define LIBC_64BIT_PREFIX "lib64"

static inline bool is_64bit_libc_so(struct file *vm_file)
{
	if (unlikely(!vm_file))
		return false;

	/*
	 * uid == 0 /system/lib64/bootstrap/libc.so
	 * uid == 1000 /apex/com.android.runtime/lib64/bionic/libc.so
	 */
	if (i_uid_read(vm_file->f_inode) != 1000 &&
	    i_uid_read(vm_file->f_inode) != 0)
		return false;

	/*
	 * MT6989 use hbt_translator to simulate 32 bit process results in
	 * TIF_32BIT flag could not work as expect.
	 */
	if (strncmp(vm_file->f_path.dentry->d_name.name,
		    LIBC_SO, sizeof(LIBC_SO) - 1) == 0 &&
	    vm_file->f_path.dentry->d_parent &&
	    vm_file->f_path.dentry->d_parent->d_parent &&
	    strncmp(vm_file->f_path.dentry->d_parent->d_parent->d_name.name,
		    LIBC_64BIT_PREFIX, sizeof(LIBC_64BIT_PREFIX) - 1) == 0)
		return true;
	return false;
}

enum chp_vma_type chp_handle_prctl_set_anon_name(const char __user *name,
						 char *kname,
						 unsigned long len)
{
	u64 rsv = current->mm->android_kabi_reserved1;
	unsigned long name_addr = untagged_addr((unsigned long)name);
	unsigned long native_addr = rsv & ~(DALVIK_MAIN_HEAP_BIT);
	struct mm_struct *mm = current->mm;

	if (!cont_pte_huge_page_enabled())
		return CHP_VMA_NONE;

	if (unlikely(!config_anon_enable || !name_addr ||
		     test_thread_flag(TIF_32BIT)))
		return CHP_VMA_NONE;

	if (len < SZ_2M)
		return CHP_VMA_NONE;

	if (find_uid_in_blacklist(from_kuid(&init_user_ns, task_uid(current))))
		return CHP_VMA_NONE;

	if (native_addr) {
		if (native_addr == name_addr)
			return CHP_VMA_NATIVE;

		if (DALVIK_MAIN_HEAP_BIT & rsv)
			return CHP_VMA_NONE;
	}

	if (strcmp(NATIVE_HEAP, kname) == 0) {
		struct vm_area_struct *vma;
		bool ret = CHP_VMA_NONE;

		if (unlikely(native_addr && native_addr != name_addr))
			return ret;

		mmap_read_lock(mm);
		/*
		 * uid == 0 /system/lib64/bootstrap/libc.so
		 * uid == 1000 /apex/com.android.runtime/lib64/bionic/libc.so
		 */
		vma = find_vma(mm, name_addr);
		if (vma && !vma_is_anonymous(vma) &&
		    is_64bit_libc_so(vma->vm_file)) {
			current->mm->android_kabi_reserved1 |= name_addr;
			ret = CHP_VMA_NATIVE;
		} else {
			if (vma && !(vma_is_anonymous(vma) && vma->vm_file))
				chp_loge("vma %s ineligible %lx %s\n",
					 NATIVE_HEAP,
					 (unsigned long)name_addr,
					 vma->vm_file->f_path.dentry->d_name.name);
		}
		mmap_read_unlock(mm);
		return ret;
	} else if (strcmp(DALVIK_MAIN_HEAP, kname) == 0) {
		current->mm->android_kabi_reserved1 |= DALVIK_MAIN_HEAP_BIT;
		return CHP_VMA_DALVIK;
	}
	return CHP_VMA_NONE;
}

/*
 * for gki or oki, use si->procs == THP_SWAP_PRIO_MAGIC as a magic,
 * si->totalhigh as a cmd, si->freehigh as a result
 */
bool handle_chp_ext_cmd(struct sysinfo *si)
{
	struct huge_page_pool *pool;

	if (likely(si->procs != THP_SWAP_PRIO_MAGIC))
		return false;

	si->freehigh = 0;
	if (unlikely(!cont_pte_huge_page_enabled()))
		return true;

	pool = cont_pte_pool();
	switch(si->totalhigh) {
	case CHP_EXT_CMD_KERNEL_SUPPORT_CHP:
		si->freehigh = true;
		break;
	case CHP_EXT_CMD_CHP_POOL:
		si->freehigh = (u64)&g_cont_pte_pool;
		break;
	case CHP_EXT_CMD_POOL_CMA_COUNT:
		si->freehigh = huge_page_pool_count(pool, HPAGE_POOL_CMA);
		break;
	case CHP_EXT_CMD_POOL_BUDDY_COUNT:
		si->freehigh = huge_page_pool_count(pool, HPAGE_POOL_BUDDY);
		break;
	case CHP_EXT_CMD_POOL_PAGES:
		si->freehigh = huge_page_pool_count(pool, NR_HPAGE_POOL_TYPE) *
			HPAGE_CONT_PTE_NR;
		si->freehigh = min((int)si->freehigh / 2,
				   pool->high * HPAGE_CONT_PTE_NR);
		break;
	case CHP_EXT_CMD_POOL_TOTAL_CMA_COUNT:
		si->freehigh = pool->cma_count;
		break;
	case CHP_EXT_CMD_POOL_WM_HIGH:
		si->freehigh = pool->high;
		break;
	default:
		break;
	}
	return true;
}

inline void handle_chp_get_unmapped_area(struct vm_unmapped_area_info *info,
					 struct file *filp, unsigned long pgoff)
{
	if (!cont_pte_huge_page_enabled()) {
		info->align_mask = 0;
		return;
	}

	/* don't change alignment for 32bit non-file pages */
	if (test_thread_flag(TIF_32BIT) &&
	    (!filp || CONFIG_CONT_PTE_FILE_HUGEPAGE_DISABLE))
		info->align_mask = 0;
	else
		info->align_mask = CONT_PTE_SIZE - 1;
}

static inline unsigned int get_pool_alloc_flags(gfp_t gfp_mask)
{
	unsigned int alloc_flags = POOL_ALLOC_WMARK_MIN;
	struct task_struct *tsk = current;

	/*
	 * we are only setting direct reclaim in tough cases like swapcache swapin,
	 * we'd like to try our best to get hugepages to decrease fallbacks
	 */
	if (gfp_mask & ___GFP_DIRECT_RECLAIM)
		alloc_flags |= POOL_ALLOC_HARDER;

	if (rt_task(tsk) || oplus_mm_vh_current_is_fg() || (gfp_mask & __GFP_HIGH))
		alloc_flags |= POOL_ALLOC_HIGH;

	if (gfp_mask & __GFP_MEMALLOC)
		alloc_flags |= POOL_ALLOC_ALL;

	return alloc_flags;
}

static inline unsigned long prepare_pool_wmark(struct huge_page_pool *pool,
		unsigned int alloc_flags,
		gfp_t gfp_mask)
{
	unsigned long wmark;
	const bool alloc_harder = (alloc_flags & (POOL_ALLOC_HARDER | POOL_ALLOC_OOM));

	wmark = pool->wmark[alloc_flags & POOL_ALLOC_WMARK_MASK];

	if (alloc_harder)
		wmark -= wmark / 2;

	if (alloc_flags & POOL_ALLOC_HIGH)
		wmark -= wmark / 2;

	if (alloc_flags & POOL_ALLOC_ALL)
		wmark = 0;

	return wmark;
}

static struct page *get_page_from_huge_pool(struct huge_page_pool *pool,
				     unsigned int alloc_flags,
				     gfp_t gfp_mask)
{
	unsigned long wmark;
	struct page *page = NULL;

	wmark = prepare_pool_wmark(pool, alloc_flags, gfp_mask);

	if (huge_page_pool_count(pool, HPAGE_POOL_CMA) > wmark)
		page = huge_page_pool_remove(pool, HPAGE_POOL_CMA);

	if (!page)
		page = huge_page_pool_remove(pool, HPAGE_POOL_BUDDY);

	return page;
}

/* from mm/page_alloc.c */
static void wake_all_kswapds(unsigned int order, gfp_t gfp_mask,
		const struct alloc_context *ac)
{
	struct zoneref *z;
	struct zone *zone;
	pg_data_t *last_pgdat = NULL;
	enum zone_type highest_zoneidx = ac->highest_zoneidx;

	for_each_zone_zonelist_nodemask(zone, z, ac->zonelist, highest_zoneidx,
			ac->nodemask) {
		if (last_pgdat != zone->zone_pgdat)
			wakeup_kswapd(zone, gfp_mask, order, highest_zoneidx);
		last_pgdat = zone->zone_pgdat;
	}
}

/* For use only in non-NUMA system mobile scenarios */
static inline void pool_try_to_wakeup_kswapd(struct huge_page_pool *pool, gfp_t gfp_mask)
{
	int order;
	struct alloc_context ac = { };

	/* Tell kswapd we're from the cont-pte hugepage pool */
	order = HPAGE_CONT_PTE_ORDER;
	gfp_mask |= POOL_USER_ALLOC;

	ac.highest_zoneidx = gfp_zone(gfp_mask);
	ac.zonelist = node_zonelist(numa_node_id(), gfp_mask);
	ac.nodemask = NULL;
	ac.migratetype = gfp_migratetype(gfp_mask);
	ac.preferred_zoneref = first_zones_zonelist(ac.zonelist,
			ac.highest_zoneidx, ac.nodemask);
	ac.spread_dirty_pages = false;

	wake_all_kswapds(order, gfp_mask, &ac);

	atomic64_add(1, &perf_stat.wmark_count[POOL_WMARK_LOW]);
}

extern unsigned long try_to_free_cont_pte_hugepages(struct zonelist *zonelist,
		gfp_t gfp_mask, nodemask_t *nodemask, unsigned long nr_reclaim);

static struct page *__try_to_cont_pte_hugepages_direct_reclaim(unsigned long *did_some_progress,
							struct huge_page_pool *pool,
							unsigned int alloc_flags,
							gfp_t gfp_mask)
{
	struct zonelist *zonelist = node_zonelist(numa_node_id(), gfp_mask);
	nodemask_t *nodemask = NULL;
	struct page *page = NULL;
	unsigned long nr_reclaim = POOL_DIRECT_RECLAIM_NR * HPAGE_CONT_PTE_NR;
	unsigned long pflags; /* FIXME: add psi for lmdk? */
	s64 time;
	s64 reclaim_seq;

	time = ktime_to_ms(ktime_get());
	atomic64_add(1, &perf_stat.wmark_count[POOL_WMARK_MIN]);
	reclaim_seq = atomic64_add_return(1, &perf_stat.reclaim_seq[POOL_DIRECT_RECLAIM]);

	psi_memstall_enter(&pflags);
	*did_some_progress = try_to_free_cont_pte_hugepages(zonelist, gfp_mask, nodemask, nr_reclaim);

	atomic64_set(&perf_stat.reclaim_count[POOL_DIRECT_RECLAIM][reclaim_seq % POOL_RECLAIM_SEQ_ITEM], *did_some_progress);
	perf_stat.reclaim_time[POOL_DIRECT_RECLAIM][reclaim_seq % POOL_RECLAIM_SEQ_ITEM] = ktime_to_ms(ktime_get()) - time;

	if (*did_some_progress < HPAGE_CONT_PTE_NR) {
		pr_err_ratelimited("@%s:%d fail to try_to_free_cont_pte_hugepages @\n",
				__func__, __LINE__);
		goto out;
	}

	page = get_page_from_huge_pool(pool, alloc_flags, gfp_mask);
out:
	psi_memstall_leave(&pflags);
	return page;
}

static inline struct page *
huge_page_pool_removes_may_oom(struct huge_page_pool *pool,
				    gfp_t gfp_mask,
				    unsigned long *did_some_progress)
{
	struct zonelist *zonelist = node_zonelist(numa_node_id(), gfp_mask);
	nodemask_t *nodemask = NULL;
	struct page *page = NULL;
	struct oom_control oc = {
		.zonelist = zonelist,
		.nodemask = nodemask,
		.memcg = NULL,
		.gfp_mask = gfp_mask,
		.order = HPAGE_CONT_PTE_ORDER,
	};

	*did_some_progress = 0;
	atomic64_add(1, &perf_stat.oom_stat[POOL_OOM_ENTER]);
	/*
	 * Acquire the oom lock.  If that fails, somebody else is
	 * making progress for us.
	*/
	if (!mutex_trylock(&oom_lock)) {
		*did_some_progress = 1;
		schedule_timeout_uninterruptible(1);
		return NULL;
	}

	if (out_of_memory(&oc)) {
		if (tsk_is_oom_victim(current))
			goto out;

		*did_some_progress = 1;
		page = get_page_from_huge_pool(pool, POOL_ALLOC_WMARK_MIN | POOL_ALLOC_OOM, gfp_mask);
	}

out:
	mutex_unlock(&oom_lock);
	return page;
}
#endif

static struct page *__alloc_cont_pte_hugepage(gfp_t gfp_mask)
{
	bool can_direct_reclaim = false;
	struct huge_page_pool *pool = cont_pte_pool();
	gfp_t gfp_zero = gfp_mask & __GFP_ZERO;
	struct page *page = NULL;
	int i;

	/* fast path: count > wmark_low */
#if CONFIG_POOL_ASYNC_RECLAIM
	int retry_count = 0;
	unsigned long did_some_progress = 0;
	unsigned int alloc_flags = POOL_ALLOC_WMARK_LOW;

	page = get_page_from_huge_pool(pool, alloc_flags, gfp_mask);
	if (page)
		goto get;
#else
	page = huge_page_pool_fetch(pool);
#endif

#if CONFIG_POOL_DIRECT_RECLAIM
	can_direct_reclaim = gfp_mask & __GFP_DIRECT_RECLAIM;
#endif

	/*
	 * slow path 1: count < wmark_low
	 * wakeup kswapd & refill_worker
	 */
#if CONFIG_POOL_ASYNC_RECLAIM
	/*
	 * Apply scoped allocation constraints. This is mainly about GFP_NOFS
	 * resp. GFP_NOIO which has to be inherited for all allocation requests
	 * from a particular context which has been marked by
	 * memalloc_no{fs,io}_{save,restore}.
	 */
	gfp_mask = current_gfp_context(gfp_mask);
	alloc_flags = get_pool_alloc_flags(gfp_mask);
retry:
	if ((gfp_mask & __GFP_KSWAPD_RECLAIM) && oplus_mm_vh_free_zram_is_ok())
		pool_try_to_wakeup_kswapd(pool, gfp_mask);

	page = get_page_from_huge_pool(pool, alloc_flags, gfp_mask);
	if (page)
		goto get;

	if (!can_direct_reclaim)
		return NULL;
	/*
	 * slow path 2: count < wmark_min
	 * direct reclaim
	 */
	page = __try_to_cont_pte_hugepages_direct_reclaim(&did_some_progress, pool, alloc_flags, gfp_mask);
	if (!page) {
		atomic64_add(1, &perf_stat.direct_reclaim_stat[POOL_DIRECT_RECLAIM_FAIL]);
		pr_err_ratelimited("@%s:%d fail to __try_to_cont_pte_hugepages_direct_reclaim@\n",
				__func__, __LINE__);
	} else {
		atomic64_add(1, &perf_stat.direct_reclaim_stat[POOL_DIRECT_RECLAIM_SUCCESS]);
		goto get;
	}

	if (!config_alloc_oom)
		return NULL;
	page = huge_page_pool_removes_may_oom(pool, gfp_mask, &did_some_progress);
	if (!page) {
		atomic64_add(1, &perf_stat.oom_stat[POOL_OOM_FAIL]);
		pr_err_ratelimited("@%s:%d fail to @huge_page_pool_removes_may_oom\n",
				__func__, __LINE__);

		if (did_some_progress) {
			did_some_progress = 0;
			/* Only one attempt is allowed */
			if (retry_count)
				return NULL;
			retry_count++;
			alloc_flags |= POOL_ALLOC_HARDER;
			pr_err_ratelimited("@%s:%d retry slow path! @\n", __func__, __LINE__);
			goto retry;
		}
		return NULL;
	}
	atomic64_add(1, &perf_stat.oom_stat[POOL_OOM_SUCCESS]);

get:
#endif

	/* a refilled hugepage from free_compound_page */
	if (page) {
		if (TestClearPageContRefill(page)) {
			for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
				post_alloc_hook(&page[i], 0, GFP_KERNEL | gfp_zero);
		} else if (!static_branch_unlikely(&init_on_alloc) && gfp_zero) {
			for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
				clear_highpage(&page[i]);
		}
	}

	return page;
}

/*
 * For the passed gfp_mask, the following is an example:
 * <- application ->
 * 1) Allow kswapd and direct reclaim
 * gfp_t gfp_mask = (GFP_TRANSHUGE_LIGHT | __GFP_RECLAIM) & ~__GFP_MOVABLE & ~__GFP_COMP;
 *
 * 2) Allow kswapd reclaim
 * gfp_mask = (GFP_TRANSHUGE_LIGHT  | __GFP_KSWAPD_RECLAIM) & ~__GFP_MOVABLE & ~__GFP_COMP;
 *
 * 3) Not allowed to reclaim
 * gfp_mask = GFP_TRANSHUGE_LIGHT & ~__GFP_MOVABLE & ~__GFP_COMP;
 *
 * <- kernel ->
 * NOTE: Without __GFP_COMP
 * gfp_mask = xxx; (no __GFP_COMP)
 * page = alloc_cont_pte_hugepage(gfp_mask);
 * __SetPageUptodate(page);
 * ...
 * __free_cont_pte_hugepages(page); //page_ref_count must be 1
 */
struct page *alloc_cont_pte_hugepage(gfp_t gfp_mask)
{
	struct page *page = NULL;
	static bool first_alloc = true;

	if (unlikely(first_alloc)) {
		pr_info("%s: Initial hugepage pool size: %lu MiB\n", __func__,
			huge_page_pool_count(cont_pte_pool(), NR_HPAGE_POOL_TYPE) *
			HPAGE_CONT_PTE_SIZE / SZ_1M);
		first_alloc = false;
	}

	/* enter fast path or slow path 1/2 */
	page = __alloc_cont_pte_hugepage(gfp_mask);
	/*
	 * slow path 3: use buddy
	 */
	if (!page) {
		/* slow path almost always fails once system gets stable */
		if (sysctl_max_chp_buddy_count != STABLE_MAX_BUDDY_USAGE) {
			count_vm_chp_event(CHP_PAGE_ALLOC_SLOWPATH);
			page = alloc_chp_from_buddy();
		}
		if (!page)
			count_vm_chp_event(CHP_PAGE_ALLOC_FAIELD);
	}

	if (page)
		mod_chp_page_state(page, 1);

	return page;
}

/* folio version of alloc_cont_pte_hugepage */
struct folio *alloc_cont_pte_folio(gfp_t gfp_mask)
{
	return (struct folio *)alloc_cont_pte_hugepage(gfp_mask);
}

static bool is_zygote_process(struct task_struct *t)
{
	const struct cred *tcred = __task_cred(t);
	if (!strcmp(t->comm, "main") && (tcred->uid.val == 0) &&
		(t->parent != 0 && !strcmp(t->parent->comm, "init")))
		return true;
	else
		return false;
}

static inline bool is_native_task(struct task_struct *tsk)
{
	return (tsk->signal->oom_score_adj == OOM_SCORE_ADJ_MIN);
}

static bool is_critical_system_task(struct task_struct *tsk)
{
	kuid_t uid;

	uid = current_uid();
	if (!uid_eq(uid, ROOT_APP_UID) &&
		!uid_eq(uid, SYSTEM_APP_UID))
		return false;

	if (!strcmp(tsk->comm, "system_server") ||
		!strcmp(tsk->comm, "surfaceflinger")||
		!strcmp(tsk->comm, "servicemanager")||
		!strcmp(tsk->comm, "init")||
		is_zygote_process(tsk))
		return true;

	return false;
}

void update_task_hugepage_critical_flag(struct task_struct *tsk)
{
	if (is_critical_system_task(tsk))
		tsk->signal->flags |= SIGNAL_HUGEPAGE_CRITICAL;
	else
		tsk->signal->flags |= SIGNAL_HUGEPAGE_NOT_CRITICAL;
}

bool is_critical_native(struct task_struct *tsk)
{
	if (!is_native_task(tsk))
		return false;

	if (tsk->signal->flags & SIGNAL_HUGEPAGE_CRITICAL)
		return true;
	else if (tsk->signal->flags & SIGNAL_HUGEPAGE_NOT_CRITICAL)
		return false;

	if (is_critical_system_task(tsk)) {
		tsk->signal->flags |= SIGNAL_HUGEPAGE_CRITICAL;
		return true;
	}

	return false;
}

bool allow_3order_use_extmigratetype_later(void)
{
	struct huge_page_pool *pool = cont_pte_pool();
	/* we have free 4order or we have splitted 3order */
	return huge_page_pool_count(pool, HPAGE_POOL_CMA) > M2N(SZ_128M) * 2 ||
		ext_free_area[HPAGE_CONT_PTE_ORDER - 1].nr_free > 2;
}

bool allow_3order_use_extmigratetype_early(void)
{
	struct huge_page_pool *pool = cont_pte_pool();
	/* we have lots of free 4order or we have many splitted 3order */
	return huge_page_pool_count(pool, HPAGE_POOL_CMA) > M2N(SZ_128M) * 4 ||
		ext_free_area[HPAGE_CONT_PTE_ORDER - 1].nr_free > 20;
}

/*
 * This function will be call by alloc_pages when order >= MAX_ORDER
 * It support alloc page from huge_page_pool for driver as zsmalloc/dma-buf/gpu.
 * the following is an example:
 *
 *	 struct chp_ext_order ce_order = {
 *		 .order = HPAGE_CONT_PTE_ORDER,
 *		 .magic = THP_SWAP_PRIO_MAGIC,
 *		 .type = CHP_ZSMALLOC,
 *	 };
 *   page = alloc_pages(gfp_mask | __GFP_COMP, ce_order.nr);
 *   ......
 *   put_page(page);
 */
struct page *alloc_chp_ext(gfp_t gfp_mask, int *order)
{
	struct page *page = NULL;
	struct huge_page_pool *pool = cont_pte_pool();
	gfp_t gfp_zero = gfp_mask & __GFP_ZERO;
	struct chp_ext_order ceo = { .nr = *order, };
	int i;

	/* sanity check */
	if (ceo.magic != THP_SWAP_PRIO_MAGIC ||
	    ceo.order != HPAGE_CONT_PTE_ORDER ||
	    ceo.type >= NR_CHP_EXT_TYPES ||
	     !(gfp_mask & __GFP_COMP))
		return NULL;

	/* decode to real order */
	*order = ceo.order;
	/* zsmalloc can use both cma and buddy */
	if(ceo.type == CHP_EXT_ZSMALLOC) {
		page = alloc_cont_pte_hugepage(gfp_mask);
	} else if (huge_page_pool_count(pool, HPAGE_POOL_CMA) > M2N(SZ_128M)) {
		page = huge_page_pool_remove(pool, HPAGE_POOL_CMA);
		if (page) {
			mod_chp_page_state(page, 1);
			if (TestClearPageContRefill(page)) {
				for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
					post_alloc_hook(&page[i], 0, GFP_KERNEL | gfp_zero);
			} else if (!static_branch_unlikely(&init_on_alloc) && gfp_zero) {
				for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
					clear_highpage(&page[i]);
			}
		}
	}

	if (page) {
		for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
			SetPageCont(&page[i]);
		prep_compound_page(page, ceo.order);
		/*
		 * 5.10 set subpages' refcount to 0 in prep_compound_page,
		 * but 6.1 doesn't
		 */
		for (i = 1; i < HPAGE_CONT_PTE_NR; i++)
			page_ref_add_unless(&page[i], -1, 0);
		SetPageContExtAlloc(page);
		SetPageUptodate(page);
		count_vm_chp_event(CHP_ALLOC_ZSMALLOC + ceo.type);
	}
	return page;
}

/**********************from *arch/arm64/mm/hugetlbpage.c******************************************/

static pte_t get_clear_flush(struct mm_struct *mm,
			     unsigned long addr,
			     pte_t *ptep,
			     unsigned long pgsize,
			     unsigned long ncontig,
			     bool flush)
{
	pte_t orig_pte = ptep_get(ptep);
	bool valid = pte_valid(orig_pte);
	unsigned long i, saddr = addr;

	for (i = 0; i < ncontig; i++, addr += pgsize, ptep++) {
		pte_t pte = ptep_get_and_clear(mm, addr, ptep);

		if (pte_dirty(pte))
			orig_pte = pte_mkdirty(orig_pte);

		if (pte_young(pte))
			orig_pte = pte_mkyoung(orig_pte);
	}

	if (valid && flush) {
		struct vm_area_struct vma = TLB_FLUSH_VMA(mm, 0);

		flush_tlb_range(&vma, saddr, addr);
	}
	return orig_pte;
}

static inline pte_t __cont_pte_huge_ptep_get_and_clear_flush(struct mm_struct *mm,
				       unsigned long addr,
				       pte_t *ptep,
				       bool flush)
{
	pte_t orig_pte = ptep_get(ptep);

	CHP_BUG_ON(!pte_cont(orig_pte));
	CHP_BUG_ON(!IS_ALIGNED(addr, HPAGE_CONT_PTE_SIZE));
	CHP_BUG_ON(!IS_ALIGNED(pte_pfn(orig_pte), HPAGE_CONT_PTE_NR));

	return get_clear_flush(mm, addr, ptep, PAGE_SIZE, CONT_PTES, flush);
}

pte_t cont_pte_huge_ptep_get_and_clear_flush(struct mm_struct *mm,
					     unsigned long addr,
					     pte_t *ptep)
{
	return __cont_pte_huge_ptep_get_and_clear_flush(mm, addr, ptep, true);
}

pte_t cont_pte_huge_ptep_get_and_clear(struct mm_struct *mm,
				       unsigned long addr,
				       pte_t *ptep)
{
	return __cont_pte_huge_ptep_get_and_clear_flush(mm, addr, ptep, false);
}

static void clear_flush(struct mm_struct *mm,
			unsigned long addr,
			pte_t *ptep,
			unsigned long pgsize, unsigned long ncontig)
{
	struct vm_area_struct vma = TLB_FLUSH_VMA(mm, 0);
	unsigned long i, saddr = addr;

	for (i = 0; i < ncontig; i++, addr += pgsize, ptep++)
		pte_clear(mm, addr, ptep);

	flush_tlb_range(&vma, saddr, addr);
}

void cont_pte_set_huge_pte_at(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep, pte_t pte)
{
	size_t pgsize = PAGE_SIZE;
	int i;
	unsigned long pfn, dpfn;
	pgprot_t hugeprot;

	pfn = pte_pfn(pte);
	dpfn = pgsize >> PAGE_SHIFT;
	hugeprot = pte_pgprot(pte);

	CHP_BUG_ON(!IS_ALIGNED(pfn, HPAGE_CONT_PTE_NR));
	CHP_BUG_ON(!IS_ALIGNED(addr, HPAGE_CONT_PTE_SIZE));
	CHP_BUG_ON(!pte_cont(pte));

	clear_flush(mm, addr, ptep, pgsize, CONT_PTES);

	for (i = 0; i < CONT_PTES; i++, ptep++, addr += pgsize, pfn += dpfn)
		cset_pte_at(mm, addr, ptep, pfn_pte(pfn, hugeprot));
}

void cont_pte_set_huge_pte_wrprotect(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep)
{
	size_t pgsize = PAGE_SIZE;
	unsigned long pfn, dpfn;
	pgprot_t hugeprot;
	int i;
	pte_t pte;

	CHP_BUG_ON (!pte_cont(READ_ONCE(*ptep)));

	pte = get_clear_flush(mm, addr, ptep, PAGE_SIZE, CONT_PTES, true);
	pte = pte_wrprotect(pte);

	hugeprot = pte_pgprot(pte);
	pfn = pte_pfn(pte);
	dpfn = pgsize >> PAGE_SHIFT;

	for (i = 0; i < CONT_PTES; i++, ptep++, addr += PAGE_SIZE, pfn += dpfn)
		cset_pte_at(mm, addr, ptep, pfn_pte(pfn, hugeprot));
}

void cont_pte_set_huge_pte_clean(struct mm_struct *mm, unsigned long addr,
			      pte_t *ptep)
{
	size_t pgsize = PAGE_SIZE;
	unsigned long pfn, dpfn;
	pgprot_t hugeprot;
	int i;
	pte_t pte;

	CHP_BUG_ON (!pte_cont(READ_ONCE(*ptep)));

	/* is hugepage clean ? */
	for (i = 0; i < CONT_PTES; i++) {
		pte_t pte = READ_ONCE(*(ptep + i));

		if (pte_dirty(pte) || pte_young(pte))
			break;
	}
	if (i >= CONT_PTES)
		return;

	pte = get_clear_flush(mm, addr, ptep, PAGE_SIZE, CONT_PTES, true);
#ifdef CONFIG_CONT_PTE_HUGEPAGE
	/*
	 * we have seen a swapentry's bit3, which is AF bit(10) in pte,
	 * is incorrectly cleared. This sets swapoffset to 0-1-2-3-0-1
	 * -2-3-8-9-10-11-8-9-10-11 for a hugepage.
	 */
	if (!pte_present(pte))
		UNALIGNED_CONT_PTE_WARN(1);
	else
#endif
	pte = pte_mkold(pte);
	pte = pte_mkclean(pte);

	hugeprot = pte_pgprot(pte);
	pfn = pte_pfn(pte);
	dpfn = pgsize >> PAGE_SHIFT;

	for (i = 0; i < CONT_PTES; i++, ptep++, addr += PAGE_SIZE, pfn += dpfn)
		cset_pte_at(mm, addr, ptep, pfn_pte(pfn, hugeprot));
}

static inline void __do_set_cont_pte_with_addr(struct vm_fault *vmf,
		struct page *page,
		unsigned long addr)
{
	pte_t entry;
	struct vm_area_struct *vma = vmf->vma;
	unsigned long haddr = ALIGN_DOWN(addr, HPAGE_CONT_PTE_SIZE);
	pte_t *ptep = vmf->pte - (addr - haddr) / PAGE_SIZE;

	entry = mk_pte(page, vma->vm_page_prot);
	entry = pte_mkyoung(entry);
	entry = pte_mkcont(entry);

	add_mm_counter(vma->vm_mm, mm_counter_file(page), HPAGE_CONT_PTE_NR);
	page_add_file_rmap(page, vmf->vma, true);

	cont_pte_set_huge_pte_at(vma->vm_mm, haddr, ptep, entry);

	count_vm_event(THP_FILE_MAPPED);
}

void do_set_cont_pte(struct vm_fault *vmf, struct page *page)
{
	__do_set_cont_pte_with_addr(vmf, page, vmf->address);
}

void do_set_cont_pte_with_addr(struct vm_fault *vmf,
			       struct page *page,
			       unsigned long addr)
{
	__do_set_cont_pte_with_addr(vmf, page, addr);
}

void change_huge_cont_pte(struct vm_area_struct *vma, pte_t * pte,
			  unsigned long addr, pgprot_t newprot,
			  unsigned long cp_flags)
{
	struct mm_struct *mm = vma->vm_mm;
	pte_t oldpte, ptent;

	oldpte = cont_pte_huge_ptep_get_and_clear(mm, addr, pte);

	ptent = pte_modify(oldpte, newprot);

	cont_pte_set_huge_pte_at(mm, addr, pte, ptent);
}

void __split_huge_zero_page_pte(struct vm_area_struct *vma,
		unsigned long haddr, pte_t *pte)
{
	int i;
	struct mm_struct *mm = vma->vm_mm;

	cont_pte_huge_ptep_get_and_clear_flush(vma->vm_mm, haddr, pte);
	for (i = 0; i < HPAGE_CONT_PTE_NR; i++, haddr += PAGE_SIZE, pte++) {
		pte_t entry;
		entry = pfn_pte(my_zero_pfn(haddr), vma->vm_page_prot);
		entry = pte_mkspecial(entry);
		CHP_BUG_ON(!pte_none(*pte));
		cset_pte_at(mm, haddr, pte, entry);
	}
	smp_wmb();
}

extern unsigned long huge_zero_pfn;

static inline bool is_huge_zero_cont_pte(pte_t pte)
{
        return READ_ONCE(huge_zero_pfn) == pte_pfn(pte) && pte_present(pte);
}

static void __split_huge_cont_pte_locked(struct vm_area_struct *vma, pte_t *pte,
		unsigned long haddr, bool freeze)
{
	int i;
	struct mm_struct *mm = vma->vm_mm;
	struct page *head = compound_head(pte_page(*pte));
	pte_t ptent = *pte;
	bool write;
	pte_t old_ptes[HPAGE_CONT_PTE_NR];

	CHP_BUG_ON(haddr & ~HPAGE_CONT_PTE_MASK);
	VM_BUG_ON_VMA(vma->vm_start > haddr, vma);
	VM_BUG_ON_VMA(vma->vm_end < haddr + HPAGE_CONT_PTE_SIZE, vma);
	CHP_BUG_ON(!pte_cont(*pte));

#define THP_SPLIT_CONT_PTE THP_SPLIT_PMD       /* we are leveraging PMD count for CONT_PTE */
	count_vm_event(THP_SPLIT_CONT_PTE);

	if (!vma_is_anonymous(vma)) {
		cont_pte_huge_ptep_get_and_clear_flush(vma->vm_mm, haddr, pte);
		page_remove_rmap(head, vma, true);
		put_page(head);

		add_mm_counter(mm, mm_counter_file(head), -HPAGE_CONT_PTE_NR);
		return;
	}

	if (is_huge_zero_cont_pte(*pte)) {
		return __split_huge_zero_page_pte(vma, haddr, pte);
	}

	write = pte_write(*pte);
	page_ref_add(head, HPAGE_CONT_PTE_NR - 1);

	for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
		old_ptes[i] = READ_ONCE(*(pte + i));
	cont_pte_huge_ptep_get_and_clear_flush(vma->vm_mm, haddr, pte);
	for (i = 0; i < HPAGE_CONT_PTE_NR; i++, haddr += PAGE_SIZE, pte++) {
		if (freeze) {
			swp_entry_t swp_entry;
			if (write)
				swp_entry = make_writable_migration_entry(page_to_pfn(head + i));
			else
				swp_entry = make_readable_migration_entry(page_to_pfn(head + i));
			ptent = swp_entry_to_pte(swp_entry);
		} else {
			ptent = old_ptes[i];
			ptent = pte_mknoncont(ptent);
		}
		cset_pte_at(vma->vm_mm, haddr, pte, ptent);
		atomic_inc(&head[i]._mapcount);
	}

	if (compound_mapcount(head) > 1 && !TestSetPageDoubleMap(head)) {
		for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
			atomic_inc(&head[i]._mapcount);
		atomic_long_inc(&cont_pte_double_map_count);
	}

	lock_page_memcg(head);
	if (atomic_add_negative(-1, compound_mapcount_ptr(head))) {
		/* Last compound_mapcount is gone. */
		__mod_lruvec_page_state(head, NR_ANON_THPS, -HPAGE_CONT_PTE_NR);
		if (TestClearPageDoubleMap(head)) {
			atomic_long_dec(&cont_pte_double_map_count);
			/* No need in mapcount reference anymore */
			for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
				atomic_dec(&head[i]._mapcount);
		}
	}
	unlock_page_memcg(head);
	munlock_vma_page(head, vma, true);

	if (freeze) {
		for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
			page_remove_rmap(head + i, vma, false);
			put_page(head + i);
		}
	}
}

/* FIXME: move the parameter from page to folio */
void __split_huge_cont_pte_double_ptl(struct vm_area_struct *vma, pte_t *pte,
			   unsigned long address, bool freeze,
			   struct page *page, spinlock_t *src_ptl, spinlock_t *dst_ptl)
{
	bool do_unlock_page = false;
	unsigned long haddr = address & HPAGE_CONT_PTE_MASK;
	struct folio *folio = page ? page_folio(page) : NULL; /* fixme: use folio directly */
	pte_t _pte;

	CHP_BUG_ON(!src_ptl);

	if (address != haddr)
		pte -= (address - haddr)/PAGE_SIZE;

	CHP_BUG_ON(freeze && !page);
	if (page) {
		VM_WARN_ON_ONCE(!PageLocked(page));
		if (page != pte_page(*pte))
			goto out;
	}
repeat:
	if (pte_cont(*pte)) {
		/* FIXME: Probes whether all 16 ptes are pte_cont */
#if CONFIG_CHP_ABNORMAL_PTES_DEBUG
		check_cont_pte_trans_huge(pte, CORRUPT_CONT_PTE_REASON_SPLIT_CONT_PTE);
#endif

		if (!page) {
			page = pte_page(*pte);
			folio = page_folio(page);
			CHP_BUG_ON(!page);
			if (PageAnon(page)) {
				if (unlikely(!trylock_page(page))) {
					get_page(page);
					_pte = *pte;
					spin_unlock(src_ptl);
					if (dst_ptl)
						spin_unlock(dst_ptl);
					lock_page(page);
					if (dst_ptl)
						spin_lock(dst_ptl);
					spin_lock(src_ptl);
					if (unlikely(!pte_same(*pte, _pte))) {
						unlock_page(page);
						put_page(page);
						page = NULL;
						goto repeat;
					}
					put_page(page);
				}

				do_unlock_page = true;
			}
		}
	} else {
		/*
		 * we don't have migration entry on cont pte, this migration entry is likely to
		 * be a basepage
		 */
		WARN_ON(is_migration_entry(pte_to_swp_entry(*pte)));
		goto out;
	}

	__split_huge_cont_pte_locked(vma, pte, haddr, freeze);
out:
	if (do_unlock_page)
		unlock_page(page);
}

/* FIXME: move the parameter from page to folio */
void __split_huge_cont_pte(struct vm_area_struct *vma, pte_t *pte,
			   unsigned long address, bool freeze,
			   struct page *page, spinlock_t *ptl)
{
	__split_huge_cont_pte_double_ptl(vma, pte, address, freeze, page, ptl, NULL);
}

void split_huge_cont_pte_address(struct vm_area_struct *vma,
				 unsigned long address,
				 bool freeze, struct page *page)
{
	unsigned long haddr = address & HPAGE_CONT_PTE_MASK;
	struct mmu_notifier_range range;
	spinlock_t *ptl;
	pgd_t *pgdp;
	p4d_t *p4dp;
	pud_t *pudp;
	pmd_t *pmdp;
	pte_t *ptep;

	if (vma_is_special_huge(vma))
		return;

	pgdp = pgd_offset(vma->vm_mm, haddr);
	if (!pgd_present(*pgdp))
		return;

	p4dp = p4d_offset(pgdp, haddr);
	if (!p4d_present(*p4dp))
		return;

	pudp = pud_offset(p4dp, haddr);
	if (!pud_present(*pudp))
		return;

	pmdp = pmd_offset(pudp, haddr);
	if (!pmd_present(*pmdp))
		return;

	ptep = pte_offset_map(pmdp, haddr);
	if (!pte_present(*ptep))
		return;

	mmu_notifier_range_init(&range, MMU_NOTIFY_CLEAR, 0, vma, vma->vm_mm,
			haddr, haddr + HPAGE_CONT_PTE_SIZE);
	mmu_notifier_invalidate_range_start(&range);

	ptl = pte_lockptr(vma->vm_mm, pmdp);
	spin_lock(ptl);

	__split_huge_cont_pte(vma, ptep, haddr, freeze, page, ptl);

	spin_unlock(ptl);
	mmu_notifier_invalidate_range_only_end(&range);
}

bool set_cont_pte_huge_zero_page(struct mm_struct *mm,
		struct vm_area_struct *vma, unsigned long faddr, pte_t *pte,
		struct page *zero_page)
{
	pte_t entry;
	unsigned long haddr = faddr & HPAGE_CONT_PTE_MASK;
	pte_t *ptep = pte - (faddr - haddr)/PAGE_SIZE;

	entry = mk_pte(zero_page, vma->vm_page_prot);
	entry = pte_mkyoung(entry);
	entry = pte_mkhuge(entry);
	entry = pte_mkcont(entry);
	cont_pte_set_huge_pte_at(vma->vm_mm, haddr, ptep, entry);

	return true;
}

#if !CONFIG_CONT_PTE_FILE_HUGEPAGE_DISABLE
static struct page *build_cont_pte_thp(struct address_space *mapping,
				struct page *page)
{
	int i;

	XA_STATE_ORDER(xas, &mapping->i_pages, page->index,
			HPAGE_CONT_PTE_ORDER);

	mapping_set_update(&xas, mapping);

	spin_lock_irq(&mapping->i_pages.xa_lock);
	if (!PageHead(page)) {
#ifdef CONFIG_CONT_PTE_HUGEPAGE_DEBUG_VERBOSE
		/* if there are mapped basepages, we fallback to basepage */
		for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
			if (atomic_read(&page[i]._mapcount) != -1) {
				pr_err("@ FIXME: _mapcount!=-1  %s:%d _mapcount=%d paref%lx !!!@\n",
				     __func__, __LINE__,
				     atomic_read(&page[i]._mapcount),
				     (unsigned long)&page[i]);
				CHP_BUG_ON(1);
				spin_unlock_irq(&mapping->i_pages.xa_lock);
				return ERR_PTR(-EBUSY);
			}
			if (atomic_read(&page[i]._refcount) < 1) {
				pr_err("@ FIXME: _refcount < 1  %s:%d _refcount=%d page=%lx pfn:%lx cma:%d !!!@\n",
				     __func__, __LINE__,
				     atomic_read(&page[i]._refcount),
				     (unsigned long)&page[i],
				     page_to_pfn(&page[i]),
				     within_cont_pte_cma(page_to_pfn(&page[i])));
				CHP_BUG_ON(1);
				spin_unlock_irq(&mapping->i_pages.xa_lock);
				return ERR_PTR(-EBUSY);
			}
		}
#endif
		get_page(page);
		prep_compound_page(page, HPAGE_CONT_PTE_ORDER);
		prep_transhuge_page(page);

		/* The reference count for all sub-pages is moved to the head page */
		page_ref_add(page, HPAGE_CONT_PTE_NR - 1);
		/*
		 * 5.10 set subpages' refcount to 0 in prep_compound_page,
		 * but 5.15 doesn't
		 */
		for (i = 1; i < HPAGE_CONT_PTE_NR; i++)
			page_ref_add_unless(&page[i], -1, 0);

		__inc_node_page_state(page, NR_FILE_THPS);
		__mod_lruvec_page_state(page, NR_FILE_PAGES, HPAGE_CONT_PTE_NR);
		xas_store(&xas, page);
		spin_unlock_irq(&mapping->i_pages.xa_lock);

		filemap_nr_thps_inc(mapping);
		lru_cache_add(page);

		count_vm_event(THP_FAULT_ALLOC);
	} else {
		spin_unlock_irq(&mapping->i_pages.xa_lock);
		pr_err("@@@FIXME: endio on THP %s page:%pK pfn:%lx flags:%lx\n",
			__func__, page, page_to_pfn(page), page->flags);
	}

	return page;
}

static struct page *find_get_entry_nolocked(struct address_space *mapping,
					    pgoff_t index)
{
	XA_STATE(xas, &mapping->i_pages, index);
	struct page *page;

 repeat:
	xas_reset(&xas);
	page = xas_load(&xas);
	if (xas_retry(&xas, page))
		goto repeat;

	if (!page || xa_is_value(page))
		goto out;

	if (!page_cache_get_speculative(page))
		goto repeat;

	if (unlikely(page != xas_reload(&xas))) {
		put_page(page);
		goto repeat;
	}
 out:
	return page;
}

/* return head if xa has cont_pte pages, otherwise, return the page in index */
struct page *find_get_entry_may_cont_pte(struct address_space *mapping,
		pgoff_t index)
{
	pgoff_t head = ALIGN_DOWN(index, HPAGE_CONT_PTE_NR);
	struct page *page;

	spin_lock_irq(&mapping->i_pages.xa_lock);
	page = find_get_entry_nolocked(mapping, head);

	/* we happen to search the head, no matter what it is, return */
	if (head == index)
		goto out;

	/* we get nothing at head, further search the exact index */
	if (!page || xa_is_value(page)) {
		page = find_get_entry_nolocked(mapping, index);
		goto out;
	}

	if (!PageCont(page)) {
		/* head is an ordinary page, search the exact index */
		put_page(page);
		page = find_get_entry_nolocked(mapping, index);
	}

out:
	spin_unlock_irq(&mapping->i_pages.xa_lock);

	/* almost always true */
	if (page && !xa_is_value(page) && likely(!in_atomic()))
		wait_on_page_locked(page);
	/* almost always false, to be compatible with potential atomic context */
	while (page && !xa_is_value(page) && PageCont(page) && !PageTransCompound(page))
		cpu_relax();
	CHP_BUG_ON(page && !xa_is_value(page) && PageCont(page) && !PageHead(page));

	return page;
}

static struct page *find_get_page_nolocked(struct address_space *mapping,
					   pgoff_t offset)
{
	struct page *page;

	page = find_get_entry_nolocked(mapping, offset);
	if (xa_is_value(page))
		page = NULL;

	if (!page)
		return NULL;

	page = find_subpage(page, offset);

	return page;
}

static int __find_get_cont_pte_pages(struct address_space *mapping, pgoff_t offset,
		struct page **ret_page)
{
	int i;
	struct page *page;
	pgoff_t start = ALIGN_DOWN(offset, HPAGE_CONT_PTE_NR);
	*ret_page = NULL;

	for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
		if (i == 0) {
			page = find_get_page_nolocked(mapping, start);
			if (!page)
				continue;
			if (ContPteHugePageHead(page)) {
				*ret_page = page;
				return HIT_THP;
			} else if (PageCont(page)) {
				*ret_page = page;
				return HIT_CONT;
			}

			if (start == offset) {
				*ret_page = page;
				return HIT_BASEPAGE;
			} else {
				put_page(page);
				*ret_page = find_get_page_nolocked(mapping, offset);
				return HIT_BASEPAGE;
			}
		} else {
			page = find_get_page_nolocked(mapping, start + i);

			if (page) {
				if (start + i == offset) {
					*ret_page = page;
					return HIT_BASEPAGE;
				}

				put_page(page);
				if (start + i < offset)
					*ret_page = find_get_page_nolocked(mapping, offset);
				return HIT_BASEPAGE;
			}
		}
	}

	return HIT_NOTHING;
}


/* return either the head page(if hit cont_pte) or the basepage located at offset */
int find_get_cont_pte_pages(struct address_space *mapping, pgoff_t offset,
		struct page **ret_page)
{
	int ret;

	spin_lock_irq(&mapping->i_pages.xa_lock);
	ret = __find_get_cont_pte_pages(mapping, offset, ret_page);
	spin_unlock_irq(&mapping->i_pages.xa_lock);

	return ret;
}

static int cont_add_to_page_cache_locked(struct page *page,
					   struct address_space *mapping,
					   pgoff_t offset, gfp_t gfp,
					   void **shadowp)
{
	XA_STATE(xas, &mapping->i_pages, offset);
	unsigned int order;
	void *entry, *old = NULL;
	int error;
	int i, j;

	mapping_set_update(&xas, mapping);

	xas_set_order(&xas, offset, HPAGE_CONT_PTE_ORDER);
	do {
		xas_lock_irq(&xas);
		xas_create_range(&xas);
		if (!xas_error(&xas))
			break;
		xas_unlock_irq(&xas);
		if (!xas_nomem(&xas, GFP_KERNEL)) {
			pr_err("%s %d EINVAL start:%lx mapping:%lx", __func__, __LINE__,
				(unsigned long)offset, (unsigned long)mapping);
			return -EINVAL;
		}
	} while (1);
	xas_unlock_irq(&xas);

	gfp &= GFP_RECLAIM_MASK;
	for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
		xas_set_order(&xas, offset + i, 0);

		get_page(&page[i]);
		page[i].mapping = mapping;
		page[i].index = offset + i;

		order = xa_get_order(xas.xa, xas.xa_index);
		/* Allocate memory for splitting an entry */
		if (order > thp_order(&page[i]))
			xas_split_alloc(&xas, xa_load(xas.xa, xas.xa_index),
					order, gfp);
	}

	xas_lock_irq(&xas);

	/* make sure all of 16 entries are empty before inserting */
	do {
		struct page *page;
		int ret = __find_get_cont_pte_pages(mapping, ALIGN_DOWN(offset, HPAGE_CONT_PTE_NR), &page);

		j = 0;
		if (page)
			put_page(page);
		if (ret != HIT_NOTHING) {
			xas_set_err(&xas, -EEXIST);
			goto error;
		}
	} while (0);

	for (j = 0; j < HPAGE_CONT_PTE_NR; j++) {
		xas_set_order(&xas, offset + j, 0);
		xas_for_each_conflict(&xas, entry) {
			old = entry;
			/*
			 * NOTE: Only get the shadow of the header
			 * entry for cont-pte huegpage workingset!
			 */
			if (!j && shadowp)
				*shadowp = old;
			if (!xa_is_value(entry)) {
				xas_set_err(&xas, -EEXIST);
				pr_err("%s %d EEXIST start:%lx off:%d mapping:%lx  old=%lx PageCompound=%d PageCont=%d PageHead=%d\n",
						__func__, __LINE__, (unsigned long)offset, j, (unsigned long)mapping,
						(unsigned long)old, PageCompound((struct page *)old),
						PageCont((struct page *)old), PageHead((struct page *)old));
				CHP_BUG_ON(1);
				goto error;
			}
		}

		if (old) {
			/* entry may have been split before we acquired lock */
			order = xa_get_order(xas.xa, xas.xa_index);
			if (order > thp_order(&page[j])) {
				/*  NOTE: Split a multi-index entry into smaller entries */
				xas_split(&xas, old, order);
				xas_reset(&xas);
			}
		}

		xas_store(&xas, &page[j]);
		if (xas_error(&xas)) {
			pr_err("@%s:%d FIXME: Failed to xas_store @\n", __func__, __LINE__);
			goto error;
		}
		mapping->nrpages++;
	}

error:
	if (xas_error(&xas)) {
		error = xas_error(&xas);

		if (j) {
			for (j--; j >= 0; j--) {
				xas_set(&xas, offset + j);
				xas_store(&xas, NULL);
				xas_init_marks(&xas);
				mapping->nrpages--;
			}
		}
		xas_unlock_irq(&xas);

		for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
			page[i].mapping = NULL;
			/* Leave page->index set: truncation relies upon it */
			put_page(&page[i]);
		}

		return error;
	}

	xas_unlock_irq(&xas);
	return 0;
}

static bool alloc_contpages_and_add_xa(struct address_space *mapping,
		      unsigned long index,
		      gfp_t gfp_mask,
		      struct list_head *page_pool, struct page **ret_page)
{
	int i;
	struct page *page;
	void *shadow = NULL;
	int ret;

	/*
	 * we only support fs without readpages which are real for
	 * ext4, erofs and f2fs
	 */
	CHP_BUG_ON(mapping->a_ops->readpages);

	count_vm_chp_event(THP_FILE_ENTRY);
	page = alloc_cont_pte_hugepage((GFP_TRANSHUGE_LIGHT | __GFP_RECLAIM | __GFP_HIGH) & ~__GFP_MOVABLE & ~__GFP_COMP);
	if (!page) {
		count_vm_chp_event(THP_FILE_ALLOC_FAIL);
		goto fail_alloc;
	}
	count_vm_chp_event(THP_FILE_ALLOC_SUCCESS);

	for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
		/* FIXME: we need this before we fix endio on THP */
		ClearPageContUptodate(&page[i]);
		ClearPageContIODoing(&page[i]);
		SetPageCont(&page[i]);
		__SetPageLocked(&page[i]);
	}

	if (unlikely(mem_cgroup_charge(page, NULL, gfp_mask))) {
		pr_err("@%s:%d mem_cgroup_charge\n", __func__, __LINE__);
		goto fail_charge;
	}

	ret = cont_add_to_page_cache_locked(page, mapping, index, gfp_mask, &shadow);
	if (ret < 0) {
		goto fail_xa;
	} else {
		/* NOTE: just do workingset for header page! */
		WARN_ON_ONCE(PageActive(page));
		if (!(gfp_mask & __GFP_WRITE) && shadow)
			workingset_refault(page, shadow);
	}

	*ret_page = page;
	return true;
fail_xa:
	mem_cgroup_uncharge(page);
fail_charge:
	for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
		__ClearPageLocked(&page[i]);
		ClearPageCont(&page[i]);
	}

	__free_cont_pte_hugepages(page);

fail_alloc:
	*ret_page = NULL;
	return false;
}

static void read_pages(struct readahead_control *rac,
		       struct list_head *pages, bool skip_page)
{
	const struct address_space_operations *aops = rac->mapping->a_ops;
	struct page *page;
	struct blk_plug plug;

	if (!readahead_count(rac))
		goto out;

	blk_start_plug(&plug);

	if (aops->readahead) {
		aops->readahead(rac);
		/* Clean up the remaining pages */
		while ((page = readahead_page(rac))) {
			if (PageCont(page)) {
				pr_err("@@@FIXME: %s readahead_page page:%pK pfn:%lx flags:%lx ref:%d index:%lx process:%s %d compound_head:%lx %pK cma:%d\n",
						__func__, page, page_to_pfn(page), page->flags, atomic_read(&page->_refcount),
						page->index, current->comm, current->pid, page->compound_head, compound_head(page),
						within_cont_pte_cma(page_to_pfn(page)));
				dump_page(page, "THP readahead_page");
				dump_page(compound_head(page), "THP readahead_page head");
				CHP_BUG_ON(1);
			}
			unlock_page(page);
			put_page(page);
		}
	} else if (aops->readpages) {
		aops->readpages(rac->file, rac->mapping, pages,
				readahead_count(rac));
		/* Clean up the remaining pages */
		put_pages_list(pages);
		rac->_index += rac->_nr_pages;
		rac->_nr_pages = 0;
	} else {
		while ((page = readahead_page(rac))) {
			aops->readpage(rac->file, page);
			put_page(page);
		}
	}

	blk_finish_plug(&plug);

	CHP_BUG_ON(!list_empty(pages));
	CHP_BUG_ON(readahead_count(rac));

 out:
	if (skip_page)
		rac->_index++;
}

static void cont_pte_hugepage_ra(struct readahead_control *ractl,
					   unsigned long nr_to_read,
					   unsigned long lookahead_size,
					   struct vm_fault *vmf)
{
	unsigned long i;
	struct address_space *mapping = ractl->mapping;
	unsigned long index = readahead_index(ractl);
	LIST_HEAD(page_pool);
	gfp_t gfp_mask = readahead_gfp_mask(mapping);
	int ret;
	unsigned int nofs = memalloc_nofs_save();

	for (i = 0; i < nr_to_read; i += HPAGE_CONT_PTE_NR) {
		struct page *page = NULL;

		ret = find_get_cont_pte_pages(mapping, index + i, &page);
		if (ret != HIT_NOTHING) {
			if (page)
				put_page(page);

			read_pages(ractl, &page_pool, false);
			ractl->_index = index + i + HPAGE_CONT_PTE_NR;
			ractl->_nr_pages = 0;
			continue;
		}

		ret = alloc_contpages_and_add_xa(mapping, index + i, gfp_mask, &page_pool,
				     &page);
		if (!ret) {
			read_pages(ractl, &page_pool, false);
			ractl->_index = index + i + HPAGE_CONT_PTE_NR;
			ractl->_nr_pages = 0;
			continue;
		}

		/* Set the read-ahead flag for the last hugepage */
		if (i == nr_to_read - HPAGE_CONT_PTE_NR)
			SetPageReadahead(page);

		ractl->_nr_pages += HPAGE_CONT_PTE_NR;
	}

	read_pages(ractl, &page_pool, false);
	memalloc_nofs_restore(nofs);
}

struct file *do_cont_pte_sync_mmap_readahead(struct vm_fault *vmf)
{
	struct file *file = vmf->vma->vm_file;
	struct file_ra_state *ra = &file->f_ra;
	struct address_space *mapping = file->f_mapping;
	unsigned long haddr = vmf->address & HPAGE_CONT_PTE_MASK;
	DEFINE_READAHEAD(ractl, file, mapping, vmf->pgoff);
	struct file *fpin = NULL;

	fpin = maybe_unlock_mmap_for_io(vmf, fpin);

	if (vmf->address <= haddr + HPAGE_CONT_PTE_SIZE / 2) {
		/* Readahead a hugepage forward */
		if (haddr - HPAGE_CONT_PTE_SIZE < vmf->vma->vm_start)
			goto no_readahead;
		ra->start = ALIGN_DOWN(vmf->pgoff, HPAGE_CONT_PTE_NR) - HPAGE_CONT_PTE_NR;
	} else {
		/* Readahead a hugepage backwards */
		if (haddr + 2 * HPAGE_CONT_PTE_SIZE > vmf->vma->vm_end)
			goto no_readahead;
		ra->start = ALIGN_DOWN(vmf->pgoff, HPAGE_CONT_PTE_NR);
	}
	ra->size = 2 * HPAGE_CONT_PTE_NR;
	ra->async_size = 0;	/* we are ignoring it */
	ractl._index = ra->start;

	cont_pte_hugepage_ra(&ractl, ra->size, ra->async_size, vmf);
	return fpin;
 no_readahead:
	ractl._index = ALIGN_DOWN(vmf->pgoff, HPAGE_CONT_PTE_NR);
	ra->size = HPAGE_CONT_PTE_NR;
	ra->async_size = 0;
	cont_pte_hugepage_ra(&ractl, ra->size, ra->async_size, vmf);

	return fpin;
}

struct file *do_cont_pte_async_mmap_readahead(struct vm_fault *vmf, struct page *page)
{
	struct file *file = vmf->vma->vm_file;
	struct file_ra_state *ra = &file->f_ra;
	struct address_space *mapping = file->f_mapping;
	unsigned long haddr = vmf->address & HPAGE_CONT_PTE_MASK;
	DEFINE_READAHEAD(ractl, file, mapping, vmf->pgoff);
	struct file *fpin = NULL;
	unsigned int size = 0;
	int i;

	CHP_BUG_ON(PageWriteback(page));

	ClearPageReadahead(page);

	if (inode_read_congested(mapping->host))
		return fpin;

	if (blk_cgroup_congested())
		return fpin;

	if (haddr + 2 * HPAGE_CONT_PTE_SIZE > vmf->vma->vm_end)
		goto out;

	fpin = maybe_unlock_mmap_for_io(vmf, fpin);

	/* we read up to 3 hugepages */
	for (i = 0; i < 3; i++) {
		if (haddr + (i + 2) * HPAGE_CONT_PTE_SIZE > vmf->vma->vm_end)
			break;
		size += HPAGE_CONT_PTE_NR;
	}

	ra->start = ALIGN_DOWN(vmf->pgoff, HPAGE_CONT_PTE_NR) + HPAGE_CONT_PTE_NR;
	ra->size = size;
	ra->async_size = 0;	/* we are ignoring it */
	ractl._index = ra->start;
	cont_pte_hugepage_ra(&ractl, ra->size, ra->async_size, vmf);

out:
	return fpin;
}

static inline void __build_thp(struct address_space *mapping, struct page *head)
{
	bool error = TestClearPageError(head);
	int i;

	if (!error)
		goto done;

	/* we defer 150ms and re-read pages with IO errors */
	for (i = PG_cont_ioredo_s; i <= PG_cont_ioredo_e; i++) {
		if (test_and_set_bit(i, &head->flags))
			continue;
		msleep(150);
		break;
	}
	/* we have used up our retries, HW probably has broken */
	if (i > PG_cont_ioredo_e)
		goto done;

	for (i = 0; i < HPAGE_CONT_PTE_NR; i++)
		ClearPageContIODoing(&head[i]);

	for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
		/*
		 * if fs is struggling with getting memory, memory reclamation might
		 * need long time to be done, give retry one more chance
		 */
		if (mapping->a_ops->readpage(NULL, &head[i]) == -ENOMEM)
			clear_bit(PG_cont_ioredo_s, &head->flags);
	}
	return;

done:
	build_cont_pte_thp(mapping, head);
	if (!error) {
		for (i = PG_cont_ioredo_s; i <= PG_cont_ioredo_e; i++)
			clear_bit(i, &head->flags);
		SetPageUptodate(head);
	} else {
		ClearPageUptodate(head);
		SetPageError(head);
		for (i = PG_cont_ioredo_s; i <= PG_cont_ioredo_e; i++)
			clear_bit(i, &head->flags);
	}

	unlock_page(head);

	for (i = 1; i < HPAGE_CONT_PTE_NR; i++)
		clear_bit(PG_locked, &head[i].flags);
	put_page(head);
}

static LIST_HEAD(thp_queue);
static DEFINE_SPINLOCK(thp_queue_lock);

static void thp_queue_add(struct page *page)
{
	unsigned long flags;

	spin_lock_irqsave(&thp_queue_lock, flags);
	list_add_tail(&page->lru, &thp_queue);
	spin_unlock_irqrestore(&thp_queue_lock, flags);
}

static struct page *thp_queue_remove(void)
{
	unsigned long flags;
	struct page *page;

	spin_lock_irqsave(&thp_queue_lock, flags);
	page = list_first_entry_or_null(&thp_queue, struct page, lru);
	if (page)
		list_del(&page->lru);
	spin_unlock_irqrestore(&thp_queue_lock, flags);

	return page;
}

static void build_thp(struct work_struct *work)
{
	struct page *head;

	while ((head = thp_queue_remove()))
		__build_thp(head->mapping, head);
}
static DECLARE_WORK(thp_queue_work, build_thp);
static struct workqueue_struct *build_thp_wq;

#if defined(CONFIG_DEBUG_LOCK_ALLOC)
static DEFINE_SPINLOCK(cont_endio_lock);
#endif

void init_cont_endio_spinlock(struct inode *inode)
{
	/*
	 * with debug, spinlock will take 64bytes rather than 4 bytes
	 * (or 24 with DEBUG_SPINLOCK)
	 */
#if !defined(CONFIG_DEBUG_LOCK_ALLOC)
	spinlock_t *sp = (spinlock_t *)&inode->i_mapping->endio_spinlock;
	/* we are using android reserve2-4 */
	CHP_BUG_ON(sizeof(spinlock_t) > 3 * sizeof(unsigned long));
	spin_lock_init(sp);
#endif
}

void set_cont_pte_uptodate_and_unlock(struct page *page)
{
	struct address_space *mapping;
	bool page_uptodate  = true;
	bool page_error = false;
	unsigned long pfn;
	struct page *head;
	unsigned long flags;
	spinlock_t *sp;
	int i;

	pfn = page_to_pfn(page);
	head = (struct page *)pfn_to_page(ALIGN_DOWN(pfn, HPAGE_CONT_PTE_NR));
	CHP_BUG_ON(!PageCont(head) || !PageCont(page));

	mapping = page->mapping;

	/* TODO: figure out why this is happening */
	if (PageCompound(page)) {
		pr_err("@@@FIXME: %s endio on THP: cont:%d %d index:%lx %lx mapping:%lx %lx pfn:%lx within:%d compound:%d update:%d\n",
				__func__, PageCont(head), PageCont(page), head->index, page->index, (unsigned long)mapping, (unsigned long)
				head->mapping, pfn, within_cont_pte_cma(pfn), PageTransCompound(page), PageUptodate(head));
		return;
	}
	CHP_BUG_ON(!mapping || ((mapping != head->mapping) && !PageCompound(page)));
	SetPageContUptodate(page);

#if defined(CONFIG_DEBUG_LOCK_ALLOC)
	sp = &cont_endio_lock;
#else
	sp = (spinlock_t *)&head->mapping->endio_spinlock;
#endif
	spin_lock_irqsave(sp, flags);
	for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
		CHP_BUG_ON(!PageCont(&head[i]));
		if (!PageContUptodate(&head[i])) {
			page_uptodate = false;
			break;
		}
	}

	if (page_uptodate) {
		for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
			/* users can re-read once */
			bool error = TestClearPageError(&head[i]);

			ClearPageContUptodate(&head[i]);
			if (error)
				page_error = error;
		}
	}
	spin_unlock_irqrestore(sp, flags);

	if (!page_uptodate)
		return;

	if (page_error)
		SetPageError(head);

	if (in_interrupt() || page_error) {
		thp_queue_add(head);
		queue_work(build_thp_wq, &thp_queue_work);
	} else {
		__build_thp(head->mapping, head);
	}
}

vm_fault_t cont_pte_filemap_around(struct vm_fault *vmf, pgoff_t start_pgoff, pgoff_t end_pgoff)
{
	struct vm_area_struct *vma = vmf->vma;
	struct file *file = vma->vm_file;
	struct address_space *mapping = file->f_mapping;
	struct page *page;
	vm_fault_t ret = 0;
	unsigned long addr = vmf->address;
	unsigned long haddr = addr & HPAGE_CONT_PTE_MASK;
	unsigned long end = min(vma->vm_end - 1, ALIGN_DOWN(haddr, SZ_2M) +  SZ_2M - 1);
	int hit, i;
	pgoff_t pgoff, last_pgoff, hoff;
	unsigned long around_cont = 0;

	hoff = ALIGN_DOWN(vmf->pgoff, HPAGE_CONT_PTE_NR);
	pgoff = last_pgoff = hoff;

	/*
	 * we are unable to map hugepage, but someone else might have read hugepage or
	 * basepages, map us as basepages from their pages(either hugepage or basepage)
	 */
	if (!transhuge_cont_pte_vma_suitable(vmf->vma, haddr)) {
		hit = find_get_cont_pte_pages(mapping, hoff, &page);
		/* try to map around base pages */
		if (hit != HIT_NOTHING) {
			if (page)
				put_page(page);
			if (hit != HIT_CONT)
				return vmf->vma->vm_ops->map_pages(vmf, start_pgoff, end_pgoff);
		}
		return 0;
	}

	if (pmd_none(*vmf->pmd)) {
		struct mm_struct *mm = vmf->vma->vm_mm;

		CHP_BUG_ON(pmd_trans_huge(*vmf->pmd));

		if (vmf->flags & FAULT_FLAG_SPECULATIVE)
			return VM_FAULT_RETRY;

		vmf->ptl = pmd_lock(mm, vmf->pmd);
		if (likely(pmd_none(*vmf->pmd))) {
			mm_inc_nr_ptes(mm);
			pmd_populate(mm, vmf->pmd, vmf->prealloc_pte);
			vmf->prealloc_pte = NULL;
		}
		spin_unlock(vmf->ptl);

		/* See comment in handle_pte_fault() */
		if (pmd_devmap_trans_unstable(vmf->pmd))
			return VM_FAULT_NOPAGE;
	}

	addr = haddr;
	if (!pte_map_lock_addr(vmf, addr)) {
		ret = VM_FAULT_RETRY;
		goto out;
	}

	for (i = 0; i < MAX_HUGEPAGE_MAPAROUND; i++) {
		int hit;
		unsigned long max_idx;

		if (haddr + (i + 1) * HPAGE_CONT_PTE_SIZE > end)
			break;

		pgoff = hoff + i * HPAGE_CONT_PTE_NR;
		hit = find_get_cont_pte_pages(mapping, pgoff, &page);
		if (hit == HIT_THP) {
			if (PageLocked(page))
				goto put;

			if (!PageUptodate(page) || PageReadahead(page))
				goto put;

			if (PageHWPoison(page))
				goto put;

			if (!trylock_page(page))
				goto put;

			if (page->mapping != mapping)
				goto unlock;

			if (!PageUptodate(page))
				goto unlock;

			max_idx = DIV_ROUND_UP(i_size_read(mapping->host), PAGE_SIZE);
			if (pgoff >= max_idx)
				goto unlock;

			addr += (pgoff - last_pgoff) << PAGE_SHIFT;
			vmf->pte += pgoff - last_pgoff;
			last_pgoff = pgoff;

			if (!cont_pte_none(vmf->pte))
				goto unlock;

			/* We're about to handle the fault */
			if (haddr == addr)
				ret = VM_FAULT_NOPAGE;

			do_set_cont_pte_with_addr(vmf, compound_head(page), addr);

			around_cont++;

			unlock_page(page);
			continue;
unlock:
			unlock_page(page);
put:
			put_page(page);
		} else {
			if (page)
				put_page(page);
		}
	}
	pte_unmap_unlock(vmf->pte, vmf->ptl);
out:

#if CONFIG_CONT_PTE_HUGEPAGE_DEBUG
	atomic_long_inc(&fault_around_stat[around_cont]);
#endif
	return ret;
}
#endif /* !CONFIG_CONT_PTE_FILE_HUGEPAGE_DISABLE */

#if CONFIG_CHP_ABNORMAL_PTES_DEBUG
static inline int chp_find_uid_in_record(uid_t uid)
{
	int i;

	for (i = 0; i < CHP_ABMORMAL_PTES_SEQ; i++) {
		if (perf_stat.abps[i].uid == uid &&
		    perf_stat.abps[i].reason)
			return i;
	}

	return -1;
}

inline void commit_chp_abnormal_ptes_record(unsigned long reason)
{
	uid_t uid;
	u64 seq;
	int idx;

	seq = atomic64_read(&perf_stat.chp_abnormal_ptes_uid_cnt);
	if (seq <= CHP_ABMORMAL_PTES_SEQ - 1) {
		uid = from_kuid(&init_user_ns, task_uid(current));
		idx = chp_find_uid_in_record(uid);
		/* uid already record? */
		if (idx != -1 && perf_stat.abps[seq].reason & reason)
			return;

		seq = (idx == -1) ? seq : idx;
		perf_stat.abps[seq].uid = uid;
		get_task_comm(perf_stat.abps[seq].comm, current->group_leader);
		perf_stat.abps[seq].pid = current->group_leader->pid;
		perf_stat.abps[seq].reason |= reason;

		/* uid first record? */
		if (idx == -1) {
			pr_debug_ratelimited("@%s:%d idx:%d, seq:%llu, uid:%d comm:%s pid:%d reason:%lx @\n", __func__, __LINE__,
					idx, seq, uid, perf_stat.abps[seq].comm, perf_stat.abps[seq].pid,
					perf_stat.abps[seq].reason);
			atomic64_inc(&perf_stat.chp_abnormal_ptes_uid_cnt);
		}
	}

	return;
}

inline void check_cont_pte_trans_huge(pte_t *ptep, unsigned long reason)
{
	int i, nr = 0;

	if (unlikely(!IS_ALIGNED((unsigned long)ptep, CONT_PTES * sizeof(pte_t))))
		ptep = (pte_t *)((unsigned long)ptep & ~((unsigned long)CONT_PTES * sizeof(pte_t) - 1));

	for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
		if (pte_cont(*(ptep + i)))
			nr++;
	}

	if (nr > 0 && nr < HPAGE_CONT_PTE_NR) {
		for (i = 0; i < HPAGE_CONT_PTE_NR; i++) {
			printk(KERN_ERR "@@@Fixme - corrupted cont_pte %s %s-%d i:%d pte:%llx\n",
					__func__, current->comm, current->pid, i, pte_val(*(ptep + i)));
		}

		commit_chp_abnormal_ptes_record(reason);

		CHP_BUG_ON(1);
	}
}
#endif /* CONFIG_CHP_ABNORMAL_PTES_DEBUG */

#if CONFIG_POOL_ASYNC_RECLAIM
static int proc_pool_async_reclaim_stat_show(struct seq_file *s, void *v)
{
	int i, j;
	struct huge_page_pool *pool = cont_pte_pool();
	s64 reclaim_count[POOL_RECLAIM_ITEM][POOL_RECLAIM_SEQ_ITEM];
	s64 reclaim_time[POOL_RECLAIM_ITEM][POOL_RECLAIM_SEQ_ITEM];
	s64 avg_reclaim_count[POOL_RECLAIM_ITEM] = {0};
	s64 avg_reclaim_time[POOL_RECLAIM_ITEM] = {0};

	huge_page_pool_cnt_update(pool);

	seq_puts(s, "*************************** Pool Async Statistics ******************************\n");
	for (i = 0; i < POOL_RECLAIM_ITEM; i++) {
		for (j = 0; j < POOL_RECLAIM_SEQ_ITEM; j++) {
			if (!j) {
				seq_printf(s, "\n%s\n", i ? "direct reclaim count: " : "kswapd reclaim count: ");
				seq_printf(s, " reclaim_seq: %llu real_seq:%llu\n",
						atomic64_read(&perf_stat.reclaim_seq[i]),
						atomic64_read(&perf_stat.reclaim_seq[i]) % POOL_RECLAIM_SEQ_ITEM);
			}

			reclaim_time[i][j] = perf_stat.reclaim_time[i][j];
			reclaim_count[i][j] = atomic64_read(&perf_stat.reclaim_count[i][j]);
			seq_printf(s, "  count[%d]:%llu page(%llu hpage) time:%llums\n",
					j, reclaim_count[i][j], reclaim_count[i][j] / HPAGE_CONT_PTE_NR,
					reclaim_time[i][j]);
			avg_reclaim_count[i] += reclaim_count[i][j];
			avg_reclaim_time[i] += reclaim_time[i][j];
		}

		if (atomic64_read(&perf_stat.reclaim_seq[i]) > POOL_RECLAIM_SEQ_ITEM) {
			avg_reclaim_count[i] /= POOL_RECLAIM_SEQ_ITEM;
			avg_reclaim_time[i] /= POOL_RECLAIM_SEQ_ITEM;
		} else {
			avg_reclaim_count[i] /= atomic64_read(&perf_stat.reclaim_seq[i]);
			avg_reclaim_time[i] /= atomic64_read(&perf_stat.reclaim_seq[i]);
		}
	}

	seq_printf(s, "cma pageblock count: %u\n", pool->count[HPAGE_POOL_CMA]);
	seq_printf(s, "buddy pageblock count: %u\n", pool->count[HPAGE_POOL_BUDDY]);
	seq_printf(s, "wmark_min: %lu\n", pool->wmark[POOL_WMARK_MIN]);
	seq_printf(s, "wmark_low: %lu\n", pool->wmark[POOL_WMARK_LOW]);
	seq_printf(s, "wmark_high: %lu\n", pool->wmark[POOL_WMARK_HIGH]);
	seq_printf(s, "entry wmark_min count: %llu\n",
			atomic64_read(&perf_stat.wmark_count[POOL_WMARK_MIN]));
	seq_printf(s, "entry wmark_low count: %llu\n",
			atomic64_read(&perf_stat.wmark_count[POOL_WMARK_LOW]));
	seq_printf(s, "entry direct reclaim count: %llu\n",
			atomic64_read(&perf_stat.direct_reclaim_stat[POOL_DIRECT_RECLAIM_ENTER]));
	seq_printf(s, "direct reclaim success count: %llu\n",
			atomic64_read(&perf_stat.direct_reclaim_stat[POOL_DIRECT_RECLAIM_SUCCESS]));
	seq_printf(s, "direct reclaim fail count: %llu\n",
			atomic64_read(&perf_stat.direct_reclaim_stat[POOL_DIRECT_RECLAIM_FAIL]));
	seq_printf(s, "entry oom count: %llu\n",
			atomic64_read(&perf_stat.oom_stat[POOL_OOM_ENTER]));
	seq_printf(s, "oom success count: %llu\n",
			atomic64_read(&perf_stat.oom_stat[POOL_OOM_SUCCESS]));
	seq_printf(s, "oom fail count: %llu\n",
			atomic64_read(&perf_stat.oom_stat[POOL_OOM_FAIL]));
	seq_printf(s, "kswapd_wakeup_count: %llu\n",
			atomic64_read(&perf_stat.kswapd_wakeup_count));

	seq_printf(s, "\nnumber of observations: %d\n"
			"direct reclaim threshold: %d page(%d hpage)\n"
			"direct reclaim avg reclaim count: %llu page(%llu hpage)\n"
			"direct reclaim avg reclaim time: %llu ms\n"
			"direct reclaim efficiency: %llu (0-10000)\n"
			"kswapd reclaim threshold: %lu page(%lu hpage)\n"
			"kswapd reclaim avg reclaim count: %llu page(%llu hpage)\n"
			"kswapd reclaim avg reclaim time: %llu ms\n"
			"kswapd reclaim efficiency: %llu (0-10000)\n",
			POOL_RECLAIM_SEQ_ITEM,
			POOL_DIRECT_RECLAIM_NR * HPAGE_CONT_PTE_NR, POOL_DIRECT_RECLAIM_NR,
			avg_reclaim_count[POOL_DIRECT_RECLAIM], avg_reclaim_count[POOL_DIRECT_RECLAIM] / HPAGE_CONT_PTE_NR,
			avg_reclaim_time[POOL_DIRECT_RECLAIM],
			(avg_reclaim_count[POOL_DIRECT_RECLAIM] / HPAGE_CONT_PTE_NR) * 10000 / POOL_DIRECT_RECLAIM_NR,
			(pool->wmark[POOL_WMARK_HIGH] * HPAGE_CONT_PTE_NR) / 2, pool->wmark[POOL_WMARK_HIGH] / 2,
			avg_reclaim_count[POOL_KSWAPD_RECLAIM], avg_reclaim_count[POOL_KSWAPD_RECLAIM] / HPAGE_CONT_PTE_NR,
			avg_reclaim_time[POOL_KSWAPD_RECLAIM],
			(avg_reclaim_count[POOL_KSWAPD_RECLAIM] / HPAGE_CONT_PTE_NR) * 10000 / pool->wmark[POOL_WMARK_HIGH]);

	return 0;
}
#endif

static int proc_stat_show(struct seq_file *s, void *v)
{
	int i, j;
	struct huge_page_pool *pool = cont_pte_pool();
	unsigned long events[NR_VM_CHP_EVENT_ITEMS];
#if CONFIG_MAPPED_WALK_MIDDLE_CONT_PTE_DEBUG || CONFIG_NON_SPF_FAULT_RETRY_DEBUG || CONFIG_CHP_ABNORMAL_PTES_DEBUG
		u64 cnt;
#endif
#if CONFIG_CHP_ABNORMAL_PTES_DEBUG
		unsigned long reason;
		char *reason_str;
#endif


	huge_page_pool_cnt_update(pool);
	all_vm_chp_events(events);

	seq_printf(s, "cont_pte_cma_size %lu\n", cont_pte_pool_cma_size);
	seq_printf(s, "cont_pte_sup_mem %lu\n", cont_pte_sup_mem);
	seq_printf(s, "cont_page_flag 0x%lx\n",
		   1ul << PG_cont | 1ul << PG_cont_uptodate);

	seq_printf(s, "pool_low %d\n", pool->low * HPAGE_CONT_PTE_NR);
	seq_printf(s, "pool_high %d\n", pool->high * HPAGE_CONT_PTE_NR);
	seq_printf(s, "pool_cma_count %d\n",
		   pool->count[HPAGE_POOL_CMA] * HPAGE_CONT_PTE_NR);
	seq_printf(s, "pool_buddy_count %d\n",
		   pool->count[HPAGE_POOL_BUDDY] * HPAGE_CONT_PTE_NR);

	seq_printf(s, "usage_cma %lu\n",
		   chp_page_state(HPAGE_POOL_CMA) * HPAGE_CONT_PTE_NR);
	seq_printf(s, "usage_buddy %lu\n",
		   chp_page_state(HPAGE_POOL_BUDDY) * HPAGE_CONT_PTE_NR);
	seq_printf(s, "peak_usage_buddy %lu\n", peak_chp_nr * HPAGE_CONT_PTE_NR);

#if CONFIG_CONT_PTE_EXT_MIGRATETYPE
	seq_printf(s, "ext_mt_order3_nr_early %lu\n", ext_mt_order3_nr_early);
	seq_printf(s, "ext_mt_order3_nr_later %lu\n", ext_mt_order3_nr_later);
#endif

	seq_printf(s, "thp_swpin_swapcache_hit %llu\n",
		   atomic64_read(&thp_swpin_hit_swapcache));
	seq_printf(s, "swap_cluster_double_mapped %lu\n",
			swap_cluster_double_mapped);
	seq_printf(s, "thp_cow %llu\n", atomic64_read(&thp_cow));
	seq_printf(s, "thp_cow_fallback %llu\n", atomic64_read(&thp_cow_fallback));

	for (i = 0; i < THP_SWPIN_NO_SWAPCACHE_ENTRY; i++)
		seq_printf(s, "%s %lu\n", vm_chp_event_text[i], events[i]);

	for (; i < NR_VM_CHP_EVENT_ITEMS; i++) {
		if (i != THP_SWPIN_NO_SWAPCACHE_ENTRY &&
		    i != THP_SWPIN_NO_SWAPCACHE_FALLBACK_ENTRY &&
		    i != THP_SWPIN_SWAPCACHE_ENTRY &&
		    i != THP_SWPIN_SWAPCACHE_FALLBACK_ENTRY &&
		    i != THP_FILE_ENTRY &&
		    i != THP_SWPIN_CRITICAL_ENTRY)
			seq_puts(s, "  ");

		seq_printf(s, "%s %lu\n", vm_chp_event_text[i], events[i]);

		if (i == THP_SWPIN_SWAPCACHE_PREPARE_FAIL) {
			for (j = 1; j < RET_STATUS_NR; j++) {
				seq_printf(s, "    %s %llu\n", thp_read_swpcache_ret_status_string[j],
						atomic64_read(&perf_stat.thp_read_swpcache_ret_status_stat[j]));
			}
		}
	}

	seq_printf(s, "chunk_refill_info\n  time: %llums\n  fail_count: %llu\n  first_fail_num: %llu\n  cma_steal_count: %llu\n",
			perf_stat.chunk_refill_time,
			atomic64_read(&perf_stat.chunk_refill_fail_count),
			perf_stat.chunk_refill_first_fail_num,
			atomic64_read(&perf_stat.cma_steal_count));

	for (i = 0; i < WP_REUSE_FAIL_NR; i++)
		seq_printf(s, "%s%s: %llu\n", i ? "  " : "", wp_reuse_fail_text[i],
				atomic64_read(&perf_stat.wp_reuse_fail_count[i]));

#if CONFIG_REUSE_SWP_ACCOUNT_DEBUG
	seq_puts(s, "reuse_swp_page_stat\n");
	for (i = 0; i < REUSE_SWP_NR; i++)
		seq_printf(s, "  %s: %llu\n", reuse_swp_text[i],
				atomic64_read(&perf_stat.reuse_swp_count[i]));
#endif

	seq_printf(s, "truncate_hit_middle_page_cnt %llu\n",
		atomic64_read(&perf_stat.truncate_hit_middle_page_cnt));

	seq_printf(s, "cp_cont_pte_split_count: %llu\n",
		atomic64_read(&perf_stat.cp_cont_pte_split_count));

#if CONFIG_MAPPED_WALK_MIDDLE_CONT_PTE_DEBUG
		cnt = atomic64_read(&perf_stat.mapped_walk_middle_cont_pte_cnt);

		seq_puts(s, "mapped_walk_middle_cont_pte_stat\n");
		seq_printf(s, "  mapped_walk_middle_cont_pte_cnt: %llu\n", cnt);
		for (i = 0; i < MAPPED_WALK_HIT_SEQ; i++) {
			if (perf_stat.mapped_walk_stat[i].ori_addr) {
				seq_printf(s, "  seq: %d  ori_addr:%lx addr:%lx page:%lx page_pfn:%lx pte_pfn:%lx\n",
						i, perf_stat.mapped_walk_stat[i].ori_addr,
						perf_stat.mapped_walk_stat[i].addr,
						(unsigned long)perf_stat.mapped_walk_stat[i].page,
						perf_stat.mapped_walk_stat[i].page_pfn,
						perf_stat.mapped_walk_stat[i].pte_pfn);
				for (j = 0; j < HPAGE_CONT_PTE_NR; j++)
					seq_printf(s, "    pte%d: 0x%llx\n", j, perf_stat.mapped_walk_stat[i].pte[j]);
			}
		}

		cnt = atomic64_read(&perf_stat.mapped_walk_start_from_non_head);
		seq_printf(s, "mapped_walk_start_from_non_head: %llu\n", cnt);
		cnt = atomic64_read(&perf_stat.mapped_walk_lastmoment_doublemap);
		seq_printf(s, "mapped_walk_lastmoment_doublemap: %llu\n", cnt);
#endif

#if CONFIG_NON_SPF_FAULT_RETRY_DEBUG
		seq_puts(s, "non_sfp_fault_retry\n");
		cnt = atomic64_read(&perf_stat.non_sfp_fault_retry_cnt[SWPIN_CHP_FAULT_RETRY]);
		seq_printf(s, "  swpin_chp_fault_retry: %llu\n", cnt);
		cnt = atomic64_read(&perf_stat.non_sfp_fault_retry_cnt[SWPIN_FALLBACK_FAULT_RETRY]);
		seq_printf(s, "  swpin_fallback_fault_retry: %llu\n", cnt);
#endif

#if CONFIG_CHP_ABNORMAL_PTES_DEBUG
		seq_puts(s, "chp_abnormal_ptes_stat\n");
		cnt = atomic64_read(&perf_stat.chp_abnormal_ptes_uid_cnt);
		seq_printf(s, " chp_abnormal_ptes_uid_cnt: %llu\n", cnt);
		for (i = 0; i <= cnt; i++) {
			if (i != CHP_ABMORMAL_PTES_SEQ && perf_stat.abps[i].reason) {
				reason = perf_stat.abps[i].reason;

				if (reason & CORRUPT_CONT_PTE_REASON_MASK &&
				    reason & DOUBLE_MAP_REASON_MASK)
					reason_str = "CORRUPT_CONT_PTE | DOUBLE_MAP";
				else if (reason & CORRUPT_CONT_PTE_REASON_MASK)
					reason_str = "CORRUPT_CONT_PTE";
				else
					reason_str = "DOUBLE_MAP";

				seq_printf(s, "    cnt:%d uid:%d comm:%s pid:%d reason:(%s) 0x%lx \n", i,
						perf_stat.abps[i].uid, perf_stat.abps[i].comm,
						perf_stat.abps[i].pid, reason_str, reason);
			}
		}
#endif /* CONFIG_CHP_ABNORMAL_PTES_DEBUG */

#if CONFIG_CHP_LAZYFREE_DEBUG
	seq_puts(s, "chp_lazyfree_debug_stat\n");
	seq_printf(s, " chp_lazyfree_redirty_cnt: %llu\n",
		   atomic64_read(&perf_stat.chp_lazyfree_redirty_cnt));
	seq_printf(s, " chp_lazyfree_discard_cnt: %llu\n",
		   atomic64_read(&perf_stat.chp_lazyfree_discard_cnt));
#endif

	return 0;
}

static int proc_csv_stat_show(struct seq_file *s, void *v)
{
	unsigned long events[NR_VM_EVENT_ITEMS];
	unsigned long pages[NR_LRU_LISTS];
	int lru;
	struct huge_page_pool *pool = cont_pte_pool();

	huge_page_pool_cnt_update(pool);
	all_vm_events(events);
	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_node_page_state(NR_LRU_BASE + lru);
	events[PGPGIN] /= 2;		/* sectors -> kbytes */
	events[PGPGOUT] /= 2;		/* sectors -> kbytes */

	seq_printf(s, "%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s,%10s\n",
		   "mem_avail", "file", "chp_file", "anon", "chp_anon",
		   "cma", "buddy",
		   "zram0", "zram1",
		   "pool",
		   "pgpgin", "pgpgout");
	seq_printf(s, "%10lu,%10lu,%10lu,%10lu,%10lu,%10lu,%10lu,%10d,%10d,%10d,%10lu,%10lu\n",
		   si_mem_available(),
		   pages[LRU_ACTIVE_FILE] + pages[LRU_INACTIVE_FILE],
		   global_node_page_state(NR_FILE_PMDMAPPED) * HPAGE_CONT_PTE_NR,
		   pages[LRU_ACTIVE_ANON] + pages[LRU_INACTIVE_ANON],
		   global_node_page_state(NR_ANON_THPS) * HPAGE_CONT_PTE_NR,
		   chp_page_state(HPAGE_POOL_CMA) * HPAGE_CONT_PTE_NR,
		   chp_page_state(HPAGE_POOL_BUDDY) * HPAGE_CONT_PTE_NR,
		   0,
		   0,
		   pool->count[HPAGE_POOL_CMA] * HPAGE_CONT_PTE_NR +
		   pool->count[HPAGE_POOL_BUDDY] * HPAGE_CONT_PTE_NR,
		   events[PGPGIN], events[PGPGOUT]);
	return 0;
}

#if CONFIG_CONT_PTE_HUGEPAGE_DEBUG
static int proc_fault_around_stat_show(struct seq_file *s, void *v)
{
	int i = 0;
	s64 counter, total = 0;

	seq_puts(s, "fault around stat:\n");
	for (; i < NR_FAULT_AROUND_STAT_ITEMS; i++) {
		counter = atomic64_read(&fault_around_stat[i]);
		total += counter;
		seq_printf(s, "fault-around[%d] %llu\n", i, counter);
	}
	seq_printf(s, "fault-around total %llu\n", total);

	return 0;
}
#endif

static int __init cmdline_parse_disable(char *p)
{
	int ret;

	ret = kstrtobool(p, &cont_pte_hugepage_enable);
	if (!ret && !cont_pte_hugepage_enable)
		chp_logi("cont_pte_hugepage is disabled\n");
	return 0;
}
early_param("cont_pte_hugepage", cmdline_parse_disable);

static int __init cmdline_parse_cont_pte_cma(char *p)
{
	cont_pte_pool_cma_size = ALIGN_DOWN(memparse(p, &p), CONT_PTE_CMA_CHUNK_SIZE);
	return 0;
}
early_param("cont_pte_cma", cmdline_parse_cont_pte_cma);

static int __init cmdline_parse_cont_pte_sup_mem(char *p)
{
	cont_pte_sup_mem = memparse(p, &p);
	return 0;
}
early_param("cont_pte_sup_mem", cmdline_parse_cont_pte_sup_mem);

static int __init cmdline_parse_prjname(char *p)
{
	static const char *prjs[] = {
		"22825", "22877", NULL,
	};
	static const char *ext_prjs[] = {
		"22877", NULL, /* FIXME: add projects which don't support art alignment */
	};

	int i = 0;

	for (i = 0; prjs[i] && p; i++) {
		if (!strcmp(p, prjs[i])) {
			chp_logi("%s support\n", prjs[i]);
			cont_pte_sup_prjname = true;
			break;
		}
	}

	for (i = 0; ext_prjs[i] && p; i++) {
		if (!strcmp(p, ext_prjs[i])) {
			supported_oat_hugepage = false;
			break;
		}
	}

	return 0;
}
early_param("oplusboot.prjname", cmdline_parse_prjname);

void __init cont_pte_cma_reserve(void)
{
	unsigned long phys_mem_sub_reserved_size = memblock_phys_mem_size() - memblock_reserved_size();
#if !CONFIG_CONT_PTE_EXT_MIGRATETYPE
	int res;
#endif

	if (cont_pte_pool_cma_size == 0) {
		/*setup cont_pte_pool_cma_size by ddr size*/
		if ((phys_mem_sub_reserved_size >  MEM_SIZE_8G) && (phys_mem_sub_reserved_size <  MEM_SIZE_12G)) {
			if (supported_oat_hugepage) {
				cont_pte_pool_cma_size = ALIGN_DOWN(phys_mem_sub_reserved_size * 1 / 4,
								CONT_PTE_CMA_CHUNK_SIZE);
			} else {
				cont_pte_pool_cma_size = ALIGN_DOWN(phys_mem_sub_reserved_size * 1 / 4 - SZ_512M,
								CONT_PTE_CMA_CHUNK_SIZE);
			}
		} else {
			cont_pte_pool_cma_size = ALIGN_DOWN(phys_mem_sub_reserved_size * 1 / 4,
							CONT_PTE_CMA_CHUNK_SIZE);
		}
	}

	if (cont_pte_sup_mem == 0)
		cont_pte_sup_mem = CONT_PTE_SUP_MEM_SIZE;

	if (!cont_pte_sup_prjname || !cont_pte_hugepage_enable ||
	    phys_mem_sub_reserved_size < cont_pte_sup_mem) {
		chp_logi("device does not support cont_pte_huge_page\n");
		return;
	}

#if !CONFIG_CONT_PTE_EXT_MIGRATETYPE
	res = cma_declare_contiguous(0, cont_pte_pool_cma_size, 0, 0,
			HPAGE_CONT_PTE_ORDER, false, "cont_pte",
			&cont_pte_cma);
	if (unlikely(res)) {
		pr_warn("cont_pte_cma: reservation failed: err %d", res);
		return;
	}

	chp_logi("cont_pte_cma: reserved %lu MiB\n",
		 cont_pte_pool_cma_size / SZ_1M);
#endif

	static_branch_enable(&cont_pte_huge_page_enabled_key);
}

static bool __find_uid_in_blacklist(uid_t uid)
{
	int left, right;

	if (ub == NULL || ub->size == 0)
		return false;

	left = 0;
	right = ub->size - 1;

	while (left <= right) {
		int mid = left + (right - left) / 2;

		if (ub->array[mid] == uid) {
			return true;
		} else if (ub->array[mid] < uid)
			left = mid + 1;
		else
			right = mid - 1;
	}
	return false;
}

static bool find_uid_in_blacklist(uid_t uid)
{
	bool ret;

	spin_lock(&uid_blacklist_lock);
	ret = __find_uid_in_blacklist(uid);
	spin_unlock(&uid_blacklist_lock);
	return ret;
}

static void uid_blacklist_update(size_t uid)
{
	bool update = false;
	int i;

	spin_lock(&uid_blacklist_lock);
	if (unlikely(!ub)) {
		ub = kzalloc(sizeof(struct uid_blacklist), GFP_ATOMIC);
		if (!ub) {
			chp_loge("failed to alloc uid_blacklist");
			goto unlock;
		}
	}

	if (ub->size == MAX_UID_BLACKLIST_SIZE) {
		chp_logi("uid_blacklist array oversize\n");
		goto unlock;
	}

	if (ub->size == 0) {
		update = true;
		ub->array[ub->size] = uid;
		ub->size += 1;
		goto unlock;
	}

	if (__find_uid_in_blacklist(uid))
		goto unlock;

	i = ub->size - 1;
	while (i >= 0 && ub->array[i] > uid) {
		ub->array[i + 1] = ub->array[i];
		i -= 1;
	}
	update = true;
	ub->array[i + 1] = uid;
	ub->size += 1;
unlock:
	spin_unlock(&uid_blacklist_lock);
	if (update)
		chp_logi("update uid_t:%lu\n", uid);
}

void chp_uid_blacklist_update(void)
{
	uid_t uid = from_kuid(&init_user_ns, task_uid(current));

	if (uid < 10000) {
		chp_logi("kernel space do not block any uid:%d below 10000\n",
			 uid);
		return;
	}
	uid_blacklist_update(uid);
}

static ssize_t uid_blacklist_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	struct uid_blacklist *p;
	size_t off = 0, size = PAGE_SIZE;
	int i;

	spin_lock(&uid_blacklist_lock);
	if (!ub) {
		spin_unlock(&uid_blacklist_lock);
		return scnprintf(buf, size, "<none>\n");
	}
	p = kzalloc(sizeof(struct uid_blacklist), GFP_ATOMIC);
	if (!p) {
		spin_unlock(&uid_blacklist_lock);
		return -ENOMEM;
	}
	memcpy(p, ub, sizeof(struct uid_blacklist));
	spin_unlock(&uid_blacklist_lock);

	for (i = 0; i < p->size; i++) {
		off += scnprintf(buf + off, size - off, "%d\n", p->array[i]);
		if (off >= size)
			break;
	}
	buf[off] = '\0';
	kfree(p);
	return off;
}

static ssize_t uid_blacklist_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long uid;
	int ret;

	ret = kstrtoul(buf, 10, &uid);
	if (ret)
		return ret;

	if (uid < 0)
		return -EINVAL;

	uid_blacklist_update((uid_t)uid);
	return count;
}

static ssize_t enabled_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	size_t off = 0, size = PAGE_SIZE;

	if (CONFIG_CONT_PTE_FILE_HUGEPAGE_DISABLE)
		off += scnprintf(buf + off, size - off, "file");
	else
		off += scnprintf(buf + off, size - off, "[file]");


	off += scnprintf(buf + off, size - off, " [anon]\n");
	return off;
}

DEFINE_CHP_SYSFS_ATTRIBUTE(alloc_oom);
DEFINE_CHP_SYSFS_ATTRIBUTE(anon_enable);
DEFINE_CHP_SYSFS_ATTRIBUTE(bug_on);

static struct kobj_attribute uid_blacklist_attr =
	__ATTR(uid_blacklist, 0644, uid_blacklist_show, uid_blacklist_store);
static struct kobj_attribute enabled_attr =
	__ATTR(enabled, 0444, enabled_show, NULL);

static struct attribute *chp_attr[] = {
	&uid_blacklist_attr.attr,
	&anon_enable_attr.attr,
	&alloc_oom_attr.attr,
	&bug_on_attr.attr,
	&enabled_attr.attr,
	NULL,
};

static const struct attribute_group chp_attr_group = {
	.attrs = chp_attr,
};

static int __init proc_fs_pte_huge_page_init(void)
{
	struct proc_dir_entry *root_dir;
	struct kobject *chp_kobj;
	int err;

	if (!cont_pte_huge_page_enabled())
		return -ENOMEM;

	/* create base info */
	root_dir = proc_mkdir(KBUILD_MODNAME, NULL);

	if (!root_dir)
		return -ENOMEM;

	proc_create_single("stat", 0, root_dir, proc_stat_show);
	proc_create_single("csv_stat", 0, root_dir, proc_csv_stat_show);
#if CONFIG_CONT_PTE_HUGEPAGE_DEBUG
	proc_create_single("fault_around_stat", 0, root_dir, proc_fault_around_stat_show);
#endif

#if CONFIG_POOL_ASYNC_RECLAIM
	proc_create_single("pool_async_reclaim_stat", 0, root_dir, proc_pool_async_reclaim_stat_show);
#endif

	chp_kobj = kobject_create_and_add("chp", mm_kobj);
	if (unlikely(!chp_kobj)) {
		chp_loge("failed to create chp kobject\n");
		return -ENOMEM;
	}

	err = sysfs_create_group(chp_kobj, &chp_attr_group);
	if (err) {
		chp_loge("failed to register chp group\n");
		kobject_put(chp_kobj);
		return -ENOMEM;
	}
	return 0;
}
fs_initcall(proc_fs_pte_huge_page_init);

static int __init cont_pte_huge_page_init(void)
{
	if (!cont_pte_huge_page_enabled())
		return -ENOMEM;

	if (huge_page_pool_init(cont_pte_pool())) {
		chp_loge("failed to create huge_page_pool\n");
		return -ENOMEM;
	}

#if !CONFIG_CONT_PTE_FILE_HUGEPAGE_DISABLE
	build_thp_wq = create_singlethread_workqueue("build_thp");
	if (!build_thp_wq) {
		pr_warn("failed to create build_thp workqueue\n");
		return -ENOMEM;
	}
#endif

	return 0;
}
core_initcall(cont_pte_huge_page_init);
