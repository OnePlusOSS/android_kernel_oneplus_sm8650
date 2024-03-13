// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2023 Oplus. All rights reserved.
 */

#ifndef __OPLUS_BENCH_MARK_SCHED_H__
#define __OPLUS_BENCH_MARK_SCHED_H__

#define TARGET_COMM "Thread-"
#define TOP_CHILDS_NORMAL (8)
#define TOP_CHILDS_BENCH  (7)
#define TOP_CHILDS (TOP_CHILDS_NORMAL + TOP_CHILDS_BENCH)

enum bm_task_type {
	BENCH_NONE = 0,
	BENCH_MAIN = 1,
	BENCH_NORMAL = 2,
	BENCH_MAX_TYPES
};

extern unsigned int sysctl_multi_thread;
extern struct cpumask bm_normal_task_mask;

void benchmark_init(void);
int test_benchmark_task(struct task_struct *p);
void bm_wake_up_new_task(struct task_struct *new);
int bm_set_cpus_allowed_by_task(const struct cpumask *cpu_valid_mask, const struct cpumask *new_mask,
					struct task_struct *p, unsigned int *dest_cpu);
void bm_bonus_bench_task(struct task_struct *p);
void bm_select_task_rq_fair(void *unused, struct task_struct *p,
					int prev_cpu, int sd_flag, int wake_flags, int *new_cpu);
void bm_select_task_rq_rt(struct task_struct *p, struct cpumask *lowest_mask,
					int ret, int *target_cpu);
void bm_wake_up_new_task(struct task_struct *new);
void bm_scheduler_tick(struct rq *rq);

static inline unsigned int bm_enter(void)
{
	return sysctl_multi_thread;
}
#endif /* __OPLUS_BENCH_AMRK_SCHED_H__ */
