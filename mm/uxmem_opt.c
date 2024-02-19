// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "uxmem_opt: " fmt

#include <linux/module.h>
#include <trace/hooks/vmscan.h>
#include <trace/hooks/mm.h>
#include <linux/proc_fs.h>
#include <linux/gfp.h>
#include <linux/types.h>
#include <linux/printk.h>
#include <linux/mm.h>
#include <linux/vmstat.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <uapi/linux/sched/types.h>
#include <linux/mm_inline.h>
#include <linux/page-flags.h>
#include <linux/pageblock-flags.h>
#include <linux/memcontrol.h>
#include <linux/mm_types.h>
#include <linux/page_ref.h>
#include <linux/mmzone.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include "internal.h"

#define UX_PAGE_POOL_NAME "ux_page_pool_fillthread"
#define MAX_UXMEM_POOL_ALLOC_RETRIES (5)

static const unsigned int orders[] = {0, 1};
/* 32M for order 0, 8M  for order1 by default */
static const unsigned int page_pool_nr_pages[] = {(SZ_32M >> PAGE_SHIFT), (SZ_8M >> PAGE_SHIFT)};
#define NUM_ORDERS ARRAY_SIZE(orders)
static struct ux_page_pool *pools[NUM_ORDERS];
static struct task_struct *ux_page_pool_tsk = NULL;
static wait_queue_head_t kworkthread_waitq;
static unsigned int kworkthread_wait_flag;
static bool ux_page_pool_enabled = false;
static bool fillthread_enabled = false;

static unsigned long ux_pool_alloc_fail = 0;
/* true by default, false when uxmem_opt=N in cmdline */
bool uxmem_enable = true;
static bool inited = false;

#define PARA_BUF_LEN 512

static int __init uxmem_opt_enable(char *p)
{
	int ret;

	ret = kstrtobool(p, &uxmem_enable);
	pr_info("uxmem_opt is %s!\n",
			uxmem_enable ? "enabled" : "disabled");
	return 0;
}
early_param("uxmem_opt", uxmem_opt_enable);

static int page_pool_fill(struct ux_page_pool *pool, int migratetype);

static int order_to_index(unsigned int order)
{
	int i;

	for (i = 0; i < NUM_ORDERS; i++) {
		if (order == orders[i])
			return i;
	}
	return -1;
}

static void page_pool_wakeup_process(struct ux_page_pool *pool)
{
	if (unlikely(!ux_page_pool_enabled))
		return;

	if (pool == NULL) {
		pr_err("%s: boost_pool is NULL!\n", __func__);
		return;
	}

	if (fillthread_enabled) {
		kworkthread_wait_flag = 1;
		wake_up_interruptible(&kworkthread_waitq);
	}
}

void set_ux_page_pool_fillthread_cpus(void)
{
	struct cpumask mask;
	struct cpumask *cpumask = &mask;
	pg_data_t *pgdat = NODE_DATA(0);
	unsigned int cpu = 0, cpufreq_max_tmp = 0;
	struct cpufreq_policy *policy_max = NULL;

	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (policy == NULL)
			continue;

		if (policy->cpuinfo.max_freq >= cpufreq_max_tmp) {
			cpufreq_max_tmp = policy->cpuinfo.max_freq;
			policy_max = policy;
		}
	}

	cpumask_copy(cpumask, cpumask_of_node(pgdat->node_id));
	if (policy_max)
		cpumask_andnot(cpumask, cpumask, policy_max->related_cpus);

	if (!cpumask_empty(cpumask))
		set_cpus_allowed_ptr(current, cpumask);
}

static int ux_page_pool_fillthread(void *p)
{
	struct ux_page_pool *pool;
	int i, j;
	int ret;

	if (unlikely(!ux_page_pool_enabled))
		return -1;

	set_ux_page_pool_fillthread_cpus();

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(kworkthread_waitq,
						       (kworkthread_wait_flag == 1));
		if (ret < 0)
			continue;

		kworkthread_wait_flag = 0;

		for (i = 0; i < NUM_ORDERS; i++) {
			pool = pools[i];
			for (j = 0; j < POOL_MIGRATETYPE_TYPES_SIZE; j++) {
				while (pool->count[j] < pool->high[j]) {
					if (page_pool_fill(pool, j) < 0) {
						/* sleep for 20ms if alloc fail */
						msleep(20);
					}
				}
			}
		}
	}
	return 0;
}

struct ux_page_pool *ux_page_pool_create(gfp_t gfp_mask, unsigned int order, unsigned int nr_pages)
{
	struct ux_page_pool *pool;
	int i;

	pool = kmalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		return ERR_PTR(-ENOMEM);

	pool->gfp_mask = gfp_mask;
	pool->order = order;
	for (i = 0; i < POOL_MIGRATETYPE_TYPES_SIZE; i++) {
		pool->count[i] = 0;
		/* MIGRATETYPE: UNMOVABLE & MOVABLE */
		pool->high[i] = nr_pages/POOL_MIGRATETYPE_TYPES_SIZE;
		/* wakeup kthread on count < low*/
		pool->low[i]  = pool->high[i]/2;
		INIT_LIST_HEAD(&pool->items[i]);

		pr_info("%s order:%d migratetype:%d low: %d high: %d count:%d.\n",
			__func__, pool->order, i, pool->low[i], pool->high[i], pool->count[i]);
	}

	spin_lock_init(&pool->lock);
	return pool;
}

static void page_pool_add(struct ux_page_pool *pool, struct page *page, int migratetype)
{
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);
	list_add_tail(&page->lru, &pool->items[migratetype]);
	pool->count[migratetype]++;
	spin_unlock_irqrestore(&pool->lock, flags);
}

static struct page *page_pool_remove(struct ux_page_pool *pool, int migratetype)
{
	struct page *page;
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);
	page = list_first_entry_or_null(&pool->items[migratetype], struct page, lru);
	if (page) {
		pool->count[migratetype]--;
		list_del(&page->lru);
	}
	spin_unlock_irqrestore(&pool->lock, flags);

	/* wakeup kthread on count < low*/
	if (pool->count[migratetype] < pool->low[migratetype])
		page_pool_wakeup_process(pool);

	return page;
}

static int page_pool_fill(struct ux_page_pool *pool, int migratetype)
{
	struct page *page;
	gfp_t gfp_refill = pool->gfp_mask;
	/* unsigned long pfn; */

	if (pool == NULL) {
		pr_err("%s: pool is NULL!\n", __func__);
		return -1;
	}

	page = alloc_pages(gfp_refill, pool->order);
	if (page == NULL)
		return -1;

	page_pool_add(pool, page, migratetype);
	return 1;
}

/* fast path */
static struct page *ux_page_pool_alloc_pages(unsigned int order, int migratetype)
{
	struct page *page = NULL;
	int retries = 0;
	struct ux_page_pool *pool = NULL;
	int order_ind = order_to_index(order);

	if (unlikely(!ux_page_pool_enabled) || (order_ind == -1))
		return NULL;

	if (migratetype > MIGRATE_MOVABLE)
		return NULL;

	pool = pools[order_ind];
	if (pool == NULL)
		return NULL;

retry:
	/* Fast-path: Get a page from cache */
	page = page_pool_remove(pool, migratetype);
	if (!page && retries < MAX_UXMEM_POOL_ALLOC_RETRIES) {
		retries++;
		goto retry;
	}

	if (!page)
		ux_pool_alloc_fail += 1;

	return page;
}

static int ux_page_pool_refill(struct page *page, unsigned int order, int migratetype)
{
	struct ux_page_pool *pool;
	int order_ind = order_to_index(order);

	if (unlikely(!ux_page_pool_enabled) || (order_ind == -1))
		return false;

	pool = pools[order_ind];
	if (pool == NULL)
		return false;

	if (pool->count[migratetype] >= pool->high[migratetype])
		return false;

	/* set_page_count(page, 1); */
	page_pool_add(pool, page, migratetype);
	return true;
}

static ssize_t ux_page_pool_write(struct file *file,
		const char __user *buff, size_t len, loff_t *ppos)
{
	char kbuf[PARA_BUF_LEN] = {'\0'};
	char *str;
	int high_0, high_1;
	int i, ret;
	struct ux_page_pool *pool;
	unsigned long flags;

	if (len > PARA_BUF_LEN - 1) {
		pr_err("len %ld is too long\n", len);
		return -EINVAL;
	}

	if (copy_from_user(&kbuf, buff, len))
		return -EFAULT;
	kbuf[len] = '\0';

	str = strstrip(kbuf);
	if (!str) {
		pr_err("buff %s is invalid\n", kbuf);
		return -EINVAL;
	}

	ret = sscanf(str, "%d %d", &high_0, &high_1);

	if (ret == 2) {
		for (i = 0; i < POOL_MIGRATETYPE_TYPES_SIZE; i++) {
			pool = pools[0];
			spin_lock_irqsave(&pool->lock, flags);
			/* MIGRATETYPE: UNMOVABLE & MOVABLE */
			pool->high[i] = high_0/POOL_MIGRATETYPE_TYPES_SIZE;
			pool->low[i]  = pool->high[i]/2;
			spin_unlock_irqrestore(&pool->lock, flags);
			pr_info("%s order:%d migratetype:%d low: %d high: %d count:%d.\n",
				__func__, pool->order, i,
				pool->low[i], pool->high[i], pool->count[i]);

			pool = pools[1];
			spin_lock_irqsave(&pool->lock, flags);
			/* MIGRATETYPE: UNMOVABLE & MOVABLE */
			pool->high[i] = high_1/POOL_MIGRATETYPE_TYPES_SIZE;
			pool->low[i]  = pool->high[i]/2;
			spin_unlock_irqrestore(&pool->lock, flags);
			pr_info("%s order:%d migratetype:%d low: %d high: %d count:%d.\n",
				__func__, pool->order, i,
				pool->low[i], pool->high[i], pool->count[i]);
		}
		return len;
	}

	if (strstr(str, "fillthread_pause")) {
		fillthread_enabled = false;
		return len;
	}

	if (strstr(str, "fillthread_resume")) {
		fillthread_enabled = true;
		return len;
	}

	return -EINVAL;
}

static ssize_t ux_page_pool_read(struct file *file,
		char __user *buffer, size_t count, loff_t *off)
{
	char kbuf[PARA_BUF_LEN] = {'\0'};
	int len = 0;
	int i, j;
	struct ux_page_pool *pool;

	for (i = 0; i < NUM_ORDERS; i++) {
		pool = pools[i];
		for (j = 0; j < POOL_MIGRATETYPE_TYPES_SIZE; j++) {
			len += snprintf(kbuf + len, PARA_BUF_LEN - len,
					"order:%d migratetype:%d low: %d high: %d count:%d.\n",
					pool->order, j,
					pool->low[j], pool->high[j], pool->count[j]);
		}
	}

	len += snprintf(kbuf + len, PARA_BUF_LEN - len,
			"page_pool alloc fail count:%ld\n", ux_pool_alloc_fail);
	len += snprintf(kbuf + len, PARA_BUF_LEN - len,
			"page_pool fillthread status:%s\n",
			fillthread_enabled ? "running" : "not running");

	if (len == PARA_BUF_LEN)
		kbuf[len - 1] = '\0';

	if (len > *off)
		len -= *off;
	else
		len = 0;

	if (copy_to_user(buffer, kbuf + *off, (len < count ? len : count)))
		return -EFAULT;

	*off += (len < count ? len : count);
	return (len < count ? len : count);
}

static const struct proc_ops ux_page_pool_fops = {
	.proc_write = ux_page_pool_write,
	.proc_read = ux_page_pool_read,
};

static struct proc_dir_entry *ux_page_pool_entry;
static int ux_page_pool_init(void)
{
	int i;
	struct proc_dir_entry *root_dir_entry = proc_mkdir("oplus_mem", NULL);

	for (i = 0; i < NUM_ORDERS; i++) {
		pools[i] = ux_page_pool_create((GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN |
			   __GFP_NORETRY) & ~__GFP_RECLAIM, orders[i], page_pool_nr_pages[i]);
	}
	ux_page_pool_enabled = true;

	init_waitqueue_head(&kworkthread_waitq);
	ux_page_pool_tsk = kthread_run(ux_page_pool_fillthread, NULL, UX_PAGE_POOL_NAME);
	if (IS_ERR_OR_NULL(ux_page_pool_tsk))
		pr_err("%s:run ux_page_pool_fillthread failed!\n", __func__);
	fillthread_enabled = true;
	page_pool_wakeup_process(pools[0]);


	ux_page_pool_entry = proc_create((root_dir_entry ?
				"ux_page_pool" : "oplus_mem/ux_page_pool"),
				0666, root_dir_entry, &ux_page_pool_fops);

	return 0;
}

bool current_is_key_task(void)
{
	unsigned long ret = 0;

#ifdef CONFIG_CONT_PTE_HUGEPAGE
	trace_android_vh_si_meminfo_adjust((unsigned long *)OPLUS_MM_VH_CURRENT_IS_KEY, &ret);
#endif
	return (bool)ret;
}

struct page *
get_page_from_uxmempool(gfp_t gfp_mask, unsigned int order, int migratetype)
{
	struct page *page = NULL;

	if (!inited)
		return NULL;

	if (current_is_key_task() && !(gfp_mask & __GFP_DMA32)) {
		page = ux_page_pool_alloc_pages(order, migratetype);
		/* a refilled from __free_pages */
		if (page && !page_count(page)) {
			/* prep_new_page(page, order, gfp_mask, alloc_flags);
			 */
			post_alloc_hook(page, order, gfp_mask);
		}
	}
	return page;
}

bool uxmempool_refill(struct page *page, unsigned int order, int migratetype)
{
	struct zone *zone = page_zone(page);
	unsigned long mark = zone->_watermark[WMARK_LOW];
	long free_pages = zone_page_state(zone, NR_FREE_PAGES);

	free_pages -= zone->nr_reserved_highatomic;

	if (!inited)
		return false;

	if ((migratetype <= MIGRATE_MOVABLE) && (zone_idx(zone) == ZONE_NORMAL)
			&& free_pages > mark) {
		if (ux_page_pool_refill(page, order, migratetype))
			return true;
	}

	return false;
}

#define KMALLOC_MAX_PAGES 8
#define UXMEM_POOL_MAX_PAGES 2

bool uxmem_kvmalloc_check_use_vmalloc(size_t size, gfp_t *kmalloc_flags)
{
	bool key_task;

	if (!inited)
		return false;

	key_task = current_is_key_task();

	if (key_task && (size > UXMEM_POOL_MAX_PAGES * PAGE_SIZE)) {
		*kmalloc_flags &= ~__GFP_DIRECT_RECLAIM;
		*kmalloc_flags |= __GFP_KSWAPD_RECLAIM;
		return false;
	} else if (!key_task && (size >= KMALLOC_MAX_PAGES * PAGE_SIZE))
		return true;

	return false;
}

bool uxmem_should_alloc_pages_retry(gfp_t gfp_mask, unsigned int *alloc_flags,
		struct zone *preferred_zone)
{
	if (!inited)
		return false;

	if (unlikely(current_is_key_task()) && !in_interrupt() &&
		(preferred_zone->nr_reserved_highatomic >= (SZ_8M >> PAGE_SHIFT)) &&
		!(*alloc_flags & (ALLOC_HARDER|ALLOC_OOM)) && !(gfp_mask & __GFP_NORETRY)) {
		*alloc_flags |= ALLOC_HARDER;
		return true;
	} else
		return false;
}

void fill_pcplist_from_uxmempool(struct zone *zone, unsigned int order,
		struct per_cpu_pages *pcp, int migratetype, struct list_head *list)
{
	struct page *page = NULL;

	if (!inited)
		return;

	if (current_is_key_task() && zone_idx(zone) != ZONE_DMA32) {
		page = ux_page_pool_alloc_pages(order,
				migratetype == get_cma_migrate_type() ?
				MIGRATE_MOVABLE : migratetype);
	}

	if (page) {
		list_add_tail(&page->lru, list);
		pcp->count += 1 << order;
	}
}

static int __init uxmem_opt_init(void)
{
	int ret = 0;

	if (!uxmem_enable || IS_ENABLED(CONFIG_PAGE_POISONING)) {
		pr_err("oplus_bsp_uxmem_opt is disabled in cmdline\n");
		return -EINVAL;
	}

	ret = ux_page_pool_init();
	if (ret != 0) {
		pr_err("uxmem_opt init failed!\n");
		return ret;
	}

	pr_info("uxmem_opt_init succeed!\n");
	inited = true;
	return 0;
}

static void __exit uxmem_opt_exit(void)
{
	proc_remove(ux_page_pool_entry);
	pr_info("uxmem_opt_exit succeed!\n");
}

module_init(uxmem_opt_init);
module_exit(uxmem_opt_exit);

MODULE_LICENSE("GPL v2");
