// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "mpam_test: " fmt

#include <linux/module.h>
#include <linux/io.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <trace/hooks/mpam.h>
#include <soc/qcom/mpam.h>
#include "mpam_regs.h"

#define RESERVED_PARTID	16
/*
 * Use the last byte of android_vendor_data1 to
 * log the task's part_id.
 */
#define PART_ID_OFFSET	(64 * sizeof(u64) - 1)

enum part_id_enum {
	DEFAULT_PARTID = 0,
	HIGH_PRIO_PARTID,
	LOW_PRIO_PARTID,
};

static struct kobject *mpam_test_kobj;

static int mpam_enable = 1;

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", mpam_enable);
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1) {
		pr_err("invalid param\n");
		return count;
	}

	mpam_enable = val;

	return count;
}

static inline void set_cpbm(int part_id, const char *buf)
{
	int ret;
	u32 cpbm_val;
	u64 zone_val;

	if (!mpam_enable)
		return;

	if (sscanf(buf, "%d %ld", &cpbm_val, &zone_val) != 2) {
		printk("invalid param\n");
		return;
	}
	printk("%d %ld\n",cpbm_val,zone_val);
	part_id += RESERVED_PARTID;
	ret = qcom_mpam_set_cache_portion(part_id, cpbm_val, zone_val);
	if (ret)
		printk("set cache portion failed ret %d\n", ret);
}

static inline u32 get_cpbm(int part_id)
{
	int ret;
	u64 cpbm_val;

	part_id += RESERVED_PARTID;
	ret = qcom_mpam_get_cache_portion(part_id, &cpbm_val);
	if (ret) {
		printk("get cache portion failed ret %d\n", ret);
		return 0;
	}
	printk("set cache portion ok %ld %d\n", cpbm_val, ret);
	return (u32)(cpbm_val & 0xFFFF);
}

static ssize_t low_prio_cpbm_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%x\n", get_cpbm(LOW_PRIO_PARTID));
}

static ssize_t low_prio_cpbm_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	set_cpbm(LOW_PRIO_PARTID, buf);
	return count;
}

static ssize_t normal_cpbm_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%x\n", get_cpbm(DEFAULT_PARTID));
}

static ssize_t normal_cpbm_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	set_cpbm(DEFAULT_PARTID, buf);
	return count;
}

static ssize_t high_prio_cpbm_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%x\n", get_cpbm(HIGH_PRIO_PARTID));
}

static ssize_t high_prio_cpbm_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	set_cpbm(HIGH_PRIO_PARTID, buf);
	return count;
}

static struct kobj_attribute attr_enable = __ATTR_RW(enable);
static struct kobj_attribute attr_normal_cpbm = __ATTR_RW(normal_cpbm);
static struct kobj_attribute attr_low_prio_cpbm = __ATTR_RW(low_prio_cpbm);
static struct kobj_attribute attr_high_prio_cpbm = __ATTR_RW(high_prio_cpbm);

static ssize_t pids_show(char *buf, u8 part_id)
{
	ssize_t len = 0;
	struct task_struct *p, *t;
	u8 *task_struct_vendor_hook;

	rcu_read_lock();
	for_each_process_thread(p, t) {
		task_struct_vendor_hook = (u8 *) t->android_vendor_data1;
		if (task_struct_vendor_hook[PART_ID_OFFSET] == part_id)
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d ", t->pid);
	}
	rcu_read_unlock();
	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

	return len;
}

static void mpam_find_target(void)
{
	struct task_struct *p, *t;
	u8 *task_struct_vendor_hook;

	rcu_read_lock();
	for_each_process(p) {
		if (!strncmp(p->comm, "tutu.ABenchMark", 15)) {
			for_each_thread(p, t) {
				if (!strncmp(t->comm, "Thread-", 7)) {
					task_struct_vendor_hook = (u8 *) t->android_vendor_data1;
					task_struct_vendor_hook[PART_ID_OFFSET] = HIGH_PRIO_PARTID;
				}
			}
			break; // after dealing with target process, break it
		}
	}
	rcu_read_unlock();
}

static void pids_store(const char *buf, u8 part_id)
{
	int ret;
	pid_t pid_input;
	char *kbuf, *token;
	struct task_struct *p;
	u8 *task_struct_vendor_hook;

	if (!mpam_enable)
		return;

	kbuf = (char *)buf;
	while ((token = strsep(&kbuf, " ")) != NULL) {
		ret = kstrtouint(token, 10, &pid_input);
		if (ret < 0) {
			pr_err("invalid argument\n");
			return;
		}

		if (pid_input == 0) {
			pr_err("find target\n");
			mpam_find_target();
			continue;
		}

		p = find_task_by_vpid(pid_input);
		if (IS_ERR_OR_NULL(p)) {
			pr_err("pid %d not exist\n", pid_input);
			continue;
		}

		task_struct_vendor_hook = (u8 *) p->android_vendor_data1;
		task_struct_vendor_hook[PART_ID_OFFSET] = part_id;
	}
}

static ssize_t normal_pids_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return pids_show(buf, DEFAULT_PARTID);
}

static ssize_t normal_pids_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	pids_store(buf, DEFAULT_PARTID);
	return count;
}

static ssize_t low_prio_pids_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return pids_show(buf, LOW_PRIO_PARTID);
}

static ssize_t low_prio_pids_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	pids_store(buf, LOW_PRIO_PARTID);
	return count;
}

static ssize_t high_prio_pids_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return pids_show(buf, HIGH_PRIO_PARTID);
}

static ssize_t high_prio_pids_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	pids_store(buf, HIGH_PRIO_PARTID);
	return count;
}

static struct kobj_attribute attr_normal_pids = __ATTR_RW(normal_pids);
static struct kobj_attribute attr_low_prio_pids = __ATTR_RW(low_prio_pids);
static struct kobj_attribute attr_high_prio_pids = __ATTR_RW(high_prio_pids);

static void mpam_write_partid(u8 part_id, pid_t next_pid)
{
	u64 reg;

	part_id += RESERVED_PARTID;
	reg = (part_id << PARTID_I_SHIFT) | (part_id << PARTID_D_SHIFT);

	write_sysreg_s(reg, SYS_MPAM0_EL1);
	write_sysreg_s(reg, SYS_MPAM1_EL1);
}

static void mpam_test_switch_task(void *unused, struct task_struct *prev,
							struct task_struct *next)
{
	u8 *task_struct_vendor_hook;

	task_struct_vendor_hook = (u8 *) next->android_vendor_data1;
	mpam_write_partid(task_struct_vendor_hook[PART_ID_OFFSET], next->pid);
}

static struct attribute *mpam_test_attrs[] = {
	&attr_enable.attr,
	&attr_normal_cpbm.attr,
	&attr_low_prio_cpbm.attr,
	&attr_high_prio_cpbm.attr,
	&attr_normal_pids.attr,
	&attr_low_prio_pids.attr,
	&attr_high_prio_pids.attr,
	NULL
};

static const struct attribute_group mpam_test_group = {
	.name = "parameters",
	.attrs = mpam_test_attrs,
};

static int __init mpam_test_init(void)
{
	int ret;

	mpam_test_kobj = kobject_create_and_add("mpam_test", kernel_kobj);
	if (!mpam_test_kobj)
		pr_err("kobj created failed\n");
	ret = sysfs_create_group(mpam_test_kobj, &mpam_test_group);
	if (ret)
		pr_err("sysfs created failed %d\n", ret);

	register_trace_android_vh_mpam_set(mpam_test_switch_task, NULL);

	return 0;
}
module_init(mpam_test_init);

static void __exit mpam_test_exit(void)
{
	unregister_trace_android_vh_mpam_set(mpam_test_switch_task, NULL);
	sysfs_remove_group(mpam_test_kobj, &mpam_test_group);
	kobject_del(mpam_test_kobj);
	kobject_put(mpam_test_kobj);
}
module_exit(mpam_test_exit);

MODULE_LICENSE("GPL");
