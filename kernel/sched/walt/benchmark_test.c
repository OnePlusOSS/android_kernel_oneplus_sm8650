// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#include <linux/sched/cputime.h>
#include <linux/sched.h>
#include <linux/cgroup.h>
#include <linux/kmemleak.h>
#include <uapi/linux/sched/types.h>
#include <trace/hooks/sched.h>

#include "../sched.h"
#include "benchmark_test.h"
#include "walt.h"

#define TRACE_DEBUG (0)
#define CGROUP_TOP_APP (4)

unsigned int sysctl_multi_thread;
struct cpumask bm_normal_task_mask;
struct cpumask top_backup_mask;

/* pid of top main control thread */
atomic_t top_pid = ATOMIC_INIT(0);
/* counts of new tasks which forks from top main control thread,
include bench-threads and other threads */
atomic_t top_childs = ATOMIC_INIT(0);
/* pids of bench-threads in top_childs */
int top_concur_tasks[TOP_CHILDS_BENCH] = {0};

int test_benchmark_task(struct task_struct *p)
{
	int i;

	if (likely(sysctl_multi_thread == 0))
		return BENCH_NONE;

	if (p->pid == 0)
		return BENCH_NONE;

	/* Wait for all bench tasks have found */
	if (atomic_read(&top_childs) != TOP_CHILDS)
		return BENCH_NONE;

	if (atomic_read(&top_pid) == p->pid)
		return BENCH_MAIN;

	for (i = 0; i < TOP_CHILDS_BENCH; ++i) {
		if (top_concur_tasks[i] == p->pid) {
			return BENCH_NORMAL;
		}
	}

	return BENCH_NONE;
}

void bm_wake_up_new_task(struct task_struct *new) {
	struct cgroup_subsys_state *css;

	if (likely(!bm_enter()))
		return;

	if (atomic_read(&top_pid) && (atomic_read(&top_childs) == TOP_CHILDS))
		return;

	rcu_read_lock();
	css = task_css(current, cpu_cgrp_id);
	if (!css || css->id != CGROUP_TOP_APP) {
		rcu_read_unlock();
		return;
	}
	rcu_read_unlock();

	if (atomic_read(&top_pid) && current->pid == atomic_read(&top_pid)) {
		int nums, bench_idx;

		atomic_inc(&top_childs);
		nums = atomic_read(&top_childs);
		bench_idx = nums - TOP_CHILDS_NORMAL;
		if (bench_idx > 0 && bench_idx <= TOP_CHILDS_BENCH)
			top_concur_tasks[bench_idx - 1] = new->pid;

#if TRACE_DEBUG
		trace_printk("top_childs update to %d, bench_idx=%d(%d %d %d %d %d %d %d)\n",
			nums, bench_idx, top_concur_tasks[0], top_concur_tasks[1], top_concur_tasks[2],
			top_concur_tasks[3], top_concur_tasks[4], top_concur_tasks[5], top_concur_tasks[6]);
#endif

		/* The main-loop-task(top_pid) has found, and it have create enough parallel-childs */
		if (atomic_read(&top_childs) == TOP_CHILDS) {
			struct rq_flags rf;
			struct rq *rq;
			const struct cpumask *new_mask = cpumask_of(7);

			rq = task_rq_lock(current, &rf);
			cpumask_copy(&current->cpus_mask, new_mask);
			current->nr_cpus_allowed = cpumask_weight(new_mask);
			task_rq_unlock(rq, current, &rf);
#if TRACE_DEBUG
			trace_printk("current=%-12s pid=%d affinity=(%*pbl %*pbl)\n",
				current->comm, current->pid, cpumask_pr_args(current->cpus_ptr),
				cpumask_pr_args(&current->cpus_mask));
#endif
		}
	}

	if (atomic_read(&top_pid) == 0) {
		atomic_set(&top_pid, current->pid);
		atomic_inc(&top_childs);
#if TRACE_DEBUG
		trace_printk("top_pid update to %d, top_childs update to %d\n",
			atomic_read(&top_pid), atomic_read(&top_childs));
#endif
	}
}

int bm_set_cpus_allowed_by_task(const struct cpumask *cpu_valid_mask, const struct cpumask *new_mask,
			struct task_struct *p, unsigned int *dest_cpu)
{
	if (atomic_read(&top_pid) && p->pid == atomic_read(&top_pid)) {
		const struct cpumask *bm_new_mask = cpumask_of(7);

		if (!cpumask_equal(new_mask, bm_new_mask)) {
			*dest_cpu = cpumask_any_and_distribute(bm_new_mask, new_mask);
#if TRACE_DEBUG
			trace_printk("comm=%-12s pid=%d affinity=(%*pbl %*pbl) new_mask=%*pbl dest=%d\n",
				p->comm, p->pid, cpumask_pr_args(p->cpus_ptr), cpumask_pr_args(&p->cpus_mask),
				cpumask_pr_args(new_mask), *dest_cpu);
#endif
			return 1;
		}
	}

	return 0;
}

static void bm_set_cpus_allowed_comm(void *unused, struct task_struct *p, const struct cpumask *new_mask)
{
	if (atomic_read(&top_pid) && p->pid == atomic_read(&top_pid)) {
		const struct cpumask *top_new_mask = cpumask_of(7);

		cpumask_copy(&p->cpus_mask, top_new_mask);
		p->nr_cpus_allowed = cpumask_weight(top_new_mask);
		cpumask_copy(&top_backup_mask, new_mask);
#ifdef TRACE_DEBUG
		trace_printk("comm=%-12s pid=%d affinity=(%*pbl %*pbl) backup_mask=%*pbl\n",
			p->comm, p->pid, cpumask_pr_args(p->cpus_ptr), cpumask_pr_args(&p->cpus_mask),
			cpumask_pr_args(&top_backup_mask));
#endif
	}
}

void bm_select_task_rq_fair(void *unused, struct task_struct *p,
	int prev_cpu, int sd_flag, int wake_flags, int *new_cpu)
{
	int i, lowest_nr_cpu = -1, lowest_nr = INT_MAX;

	if (likely(!bm_enter()))
		goto out;

	if (test_benchmark_task(p))
		goto out;

	if (is_migration_disabled(p))
		return;

	for_each_cpu(i, &bm_normal_task_mask) {
		bool valid = cpumask_test_cpu(i, p->cpus_ptr) &&
						cpu_active(i) && !cpu_halted(i);

		if (valid && cpu_rq(i)->nr_running < lowest_nr) {
			lowest_nr_cpu = i;
			lowest_nr = cpu_rq(i)->nr_running;
		}
	}

	if (lowest_nr_cpu != -1)
		*new_cpu = lowest_nr_cpu;

out:
#ifdef TRACE_DEBUG
	trace_printk("comm=%-12s pid=%d new_cpu=%d affinity=%*pbl\n",
		p->comm, p->pid, *new_cpu, cpumask_pr_args(p->cpus_ptr));
#endif
}

void bm_select_task_rq_rt(struct task_struct *p, struct cpumask *lowest_mask,
			int ret, int *target_cpu)
{
	int i, lowest_nr_cpu = -1, lowest_nr = INT_MAX;

	if (!ret)
		return;

	if (likely(!bm_enter()))
		return;

	if (is_migration_disabled(p))
		return;

	for_each_cpu_and(i, lowest_mask, &bm_normal_task_mask) {
		bool valid = cpumask_test_cpu(i, p->cpus_ptr) &&
						cpu_active(i) && !cpu_halted(i);

		if (valid && cpu_rq(i)->nr_running < lowest_nr) {
			lowest_nr_cpu = i;
			lowest_nr = cpu_rq(i)->nr_running;
		}
	}

	if (lowest_nr_cpu != -1)
		*target_cpu = lowest_nr_cpu;

#if TRACE_DEBUG
	trace_printk("comm=%-12s pid=%d target_cpu=%d affinity=%*pbl lowest_mask=%*pbl\n",
		p->comm, p->pid, *target_cpu, cpumask_pr_args(p->cpus_ptr), cpumask_pr_args(lowest_mask));
#endif
}

static int proc_multi_thread_handler(struct ctl_table *table,
	int write, void __user *buffer, size_t *lenp,
	loff_t *ppos)
{
	int ret, pid;

	ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (!write)
		goto out;

	if (sysctl_multi_thread) {
	} else {
		/* Notify we have leave BenchMark(Multi-Threads), restore here */
		pid = atomic_read(&top_pid);
		if (pid != 0) {
			struct task_struct *p;
			struct rq_flags rf;
			struct rq *rq;

			rcu_read_lock();
			p = find_task_by_vpid(pid);
			if (p != NULL)
				get_task_struct(p);
			rcu_read_unlock();

			if (p != NULL) {
				rq = task_rq_lock(p, &rf);
				cpumask_copy(&p->cpus_mask, &top_backup_mask);
				p->nr_cpus_allowed = cpumask_weight(&top_backup_mask);
				task_rq_unlock(rq, p, &rf);
#if TRACE_DEBUG
				trace_printk("comm=%-12s pid=%d backup_mask=%*pbl affinity=%*pbl\n",
					p->comm, p->pid, cpumask_pr_args(&top_backup_mask), cpumask_pr_args(p->cpus_ptr));
#endif
				put_task_struct(p);
			}
		}

		atomic_set(&top_pid, 0);
		atomic_set(&top_childs, 0);
		memset(top_concur_tasks, 0, sizeof(top_concur_tasks));
	}

out:
	return ret;
}

struct ctl_table bm_table[] = {
	{
		.procname	= "multi_thread",
		.data		= &sysctl_multi_thread,
		.maxlen 	= sizeof(unsigned int),
		.mode		= 0644,
		.proc_handler	= proc_multi_thread_handler,
		.extra1 	= SYSCTL_ZERO,
		.extra2 	= SYSCTL_INT_MAX,
	},
	{ }
};

struct ctl_table bm_base_table[] = {
	{
		.procname	= "bmcpu",
		.mode		= 0555,
		.child		= bm_table,
	},
	{ },
};

static int bm_proc_init(void)
{
	struct ctl_table_header *hdr;

	hdr = register_sysctl_table(bm_base_table);
	kmemleak_not_leak(hdr);

	return 0;
}

void benchmark_init(void)
{
	int ret;

	ret = bm_proc_init();
	if (ret) {
		pr_err("bm_proc_init failed, ret=%d\n", ret);
		return;
	}

	cpumask_clear(&top_backup_mask);
	/* Task prefer core 0-3 except those bench threads */
	cpumask_clear(&bm_normal_task_mask);
	cpumask_set_cpu(0, &bm_normal_task_mask);
	cpumask_set_cpu(1, &bm_normal_task_mask);
	cpumask_set_cpu(2, &bm_normal_task_mask);
	cpumask_set_cpu(3, &bm_normal_task_mask);

	ret = register_trace_android_rvh_set_cpus_allowed_comm(bm_set_cpus_allowed_comm, NULL);
	if (ret)
		pr_err("register set_cpus_allowed_comm hooks failed, ret=%d\n", ret);
}

void benchmark_exit(void)
{
}
