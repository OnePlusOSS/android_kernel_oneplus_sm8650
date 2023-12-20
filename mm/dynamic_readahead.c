// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "dynamic_readahead: " fmt

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/mm_inline.h>
#include <linux/mm_types.h>
#include <linux/mmzone.h>
#include <linux/sched.h>
#include <linux/vmstat.h>
#include <linux/fs.h>
#include "internal.h"

static unsigned long long high_wm = 0;
#ifdef CONFIG_OPLUS_FEATURE_UXMEM_OPT
/* borrow from uxmem */
extern bool current_is_key_task(void);
#endif

/* true by default, false when dynamic_readahead=N in cmdline */
static bool enable = true;
static int __init dynamic_readahead_enable(char *p)
{
	int ret;

	ret = kstrtobool(p, &enable);
	pr_info("dynamic_readahead is %s!\n",
			enable ? "enabled" : "disabled");
	return 0;
}
early_param("dynamic_readahead", dynamic_readahead_enable);

static inline bool is_lowmem(void)
{
	return global_zone_page_state(NR_FREE_PAGES) < high_wm;
}

void adjust_readaround(struct file_ra_state *ra, pgoff_t pgoff)
{
	unsigned int ra_pages = ra->ra_pages / 2;

	if (enable && !current_is_key_task() && is_lowmem()) {
		ra->start = max_t(long, 0, pgoff - ra_pages / 2);
		ra->size = ra_pages;
		ra->async_size = ra_pages / 4;
	}
}

unsigned long adjust_readahead(struct file_ra_state *ra, unsigned long max_pages)
{
	if (enable && !current_is_key_task() && is_lowmem())
		max_pages = min_t(long, max_pages, ra->ra_pages / 2);
	return max_pages;
}

static int __init dynamic_readahead_init(void)
{
	struct zone *zone;

	for_each_zone(zone) {
		high_wm += high_wmark_pages(zone);
	}
	return 0;
}

module_init(dynamic_readahead_init);

MODULE_LICENSE("GPL v2");

