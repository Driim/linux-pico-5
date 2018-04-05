// SPDX-License-Identifier: GPL-2.0
/*
 *  linux/drivers/thermal/cpu_cooling.c
 *
 *  Copyright (C) 2012	Samsung Electronics Co., Ltd(http://www.samsung.com)
 *
 *  Copyright (C) 2012-2018 Linaro Limited.
 *
 *  Authors:	Amit Daniel <amit.kachhap@linaro.org>
 *		Viresh Kumar <viresh.kumar@linaro.org>
 *
 */
#define pr_fmt(fmt) "CPU cooling: " fmt
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/cpuidle.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/idr.h>
#include <linux/kthread.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/sched/prio.h>
#include <linux/sched/rt.h>
#include <linux/smpboot.h>
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>

#include <linux/ratelimit.h>

#include <linux/platform_device.h>
#include <linux/of_platform.h>

#include <trace/events/thermal.h>

#include <uapi/linux/sched/types.h>

#ifdef CONFIG_CPU_FREQ_THERMAL
/*
 * Cooling state <-> CPUFreq frequency
 *
 * Cooling states are translated to frequencies throughout this driver and this
 * is the relation between them.
 *
 * Highest cooling state corresponds to lowest possible frequency.
 *
 * i.e.
 *	level 0 --> 1st Max Freq
 *	level 1 --> 2nd Max Freq
 *	...
 */

/**
 * struct freq_table - frequency table along with power entries
 * @frequency:	frequency in KHz
 * @power:	power in mW
 *
 * This structure is built when the cooling device registers and helps
 * in translating frequency to power and vice versa.
 */
struct freq_table {
	u32 frequency;
	u32 power;
};

/**
 * struct time_in_idle - Idle time stats
 * @time: previous reading of the absolute time that this cpu was idle
 * @timestamp: wall time of the last invocation of get_cpu_idle_time_us()
 */
struct time_in_idle {
	u64 time;
	u64 timestamp;
};

/**
 * struct cpufreq_cooling_device - data for cooling device with cpufreq
 * @id: unique integer value corresponding to each cpufreq_cooling_device
 *	registered.
 * @last_load: load measured by the latest call to cpufreq_get_requested_power()
 * @cpufreq_state: integer value representing the current state of cpufreq
 *	cooling	devices.
 * @clipped_freq: integer value representing the absolute value of the clipped
 *	frequency.
 * @max_level: maximum cooling level. One less than total number of valid
 *	cpufreq frequencies.
 * @freq_table: Freq table in descending order of frequencies
 * @cdev: thermal_cooling_device pointer to keep track of the
 *	registered cooling device.
 * @policy: cpufreq policy.
 * @node: list_head to link all cpufreq_cooling_device together.
 * @idle_time: idle time stats
 *
 * This structure is required for keeping information of each registered
 * cpufreq_cooling_device.
 */
struct cpufreq_cooling_device {
	int id;
	u32 last_load;
	unsigned int cpufreq_state;
	unsigned int clipped_freq;
	unsigned int max_level;
	struct freq_table *freq_table;	/* In descending order */
	struct cpufreq_policy *policy;
	struct list_head node;
	struct time_in_idle *idle_time;
};

static DEFINE_IDA(cpufreq_ida);
static DEFINE_MUTEX(cooling_list_lock);
static LIST_HEAD(cpufreq_cdev_list);

/* Below code defines functions to be used for cpufreq as cooling device */

/**
 * get_level: Find the level for a particular frequency
 * @cpufreq_cdev: cpufreq_cdev for which the property is required
 * @freq: Frequency
 *
 * Return: level corresponding to the frequency.
 */
static unsigned long get_level(struct cpufreq_cooling_device *cpufreq_cdev,
			       unsigned int freq)
{
	struct freq_table *freq_table = cpufreq_cdev->freq_table;
	unsigned long level;

	for (level = 1; level <= cpufreq_cdev->max_level; level++)
		if (freq > freq_table[level].frequency)
			break;

	return level - 1;
}

/**
 * cpufreq_thermal_notifier - notifier callback for cpufreq policy change.
 * @nb:	struct notifier_block * with callback info.
 * @event: value showing cpufreq event for which this function invoked.
 * @data: callback-specific data
 *
 * Callback to hijack the notification on cpufreq policy transition.
 * Every time there is a change in policy, we will intercept and
 * update the cpufreq policy with thermal constraints.
 *
 * Return: 0 (success)
 */
static int cpufreq_thermal_notifier(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned long clipped_freq;
	struct cpufreq_cooling_device *cpufreq_cdev;

	if (event != CPUFREQ_ADJUST)
		return NOTIFY_DONE;

	mutex_lock(&cooling_list_lock);
	list_for_each_entry(cpufreq_cdev, &cpufreq_cdev_list, node) {
		/*
		 * A new copy of the policy is sent to the notifier and can't
		 * compare that directly.
		 */
		if (policy->cpu != cpufreq_cdev->policy->cpu)
			continue;

		/*
		 * policy->max is the maximum allowed frequency defined by user
		 * and clipped_freq is the maximum that thermal constraints
		 * allow.
		 *
		 * If clipped_freq is lower than policy->max, then we need to
		 * readjust policy->max.
		 *
		 * But, if clipped_freq is greater than policy->max, we don't
		 * need to do anything.
		 */
		clipped_freq = cpufreq_cdev->clipped_freq;

		if (policy->max > clipped_freq)
			cpufreq_verify_within_limits(policy, 0, clipped_freq);
		break;
	}
	mutex_unlock(&cooling_list_lock);

	return NOTIFY_OK;
}

/**
 * update_freq_table() - Update the freq table with power numbers
 * @cpufreq_cdev:	the cpufreq cooling device in which to update the table
 * @capacitance: dynamic power coefficient for these cpus
 *
 * Update the freq table with power numbers.  This table will be used in
 * cpu_power_to_freq() and cpu_freq_to_power() to convert between power and
 * frequency efficiently.  Power is stored in mW, frequency in KHz.  The
 * resulting table is in descending order.
 *
 * Return: 0 on success, -EINVAL if there are no OPPs for any CPUs,
 * or -ENOMEM if we run out of memory.
 */
static int update_freq_table(struct cpufreq_cooling_device *cpufreq_cdev,
			     u32 capacitance)
{
	struct freq_table *freq_table = cpufreq_cdev->freq_table;
	struct dev_pm_opp *opp;
	struct device *dev = NULL;
	int num_opps = 0, cpu = cpufreq_cdev->policy->cpu, i;

	dev = get_cpu_device(cpu);
	if (unlikely(!dev)) {
		pr_warn("No cpu device for cpu %d\n", cpu);
		return -ENODEV;
	}

	num_opps = dev_pm_opp_get_opp_count(dev);
	if (num_opps < 0)
		return num_opps;

	/*
	 * The cpufreq table is also built from the OPP table and so the count
	 * should match.
	 */
	if (num_opps != cpufreq_cdev->max_level + 1) {
		dev_warn(dev, "Number of OPPs not matching with max_levels\n");
		return -EINVAL;
	}

	for (i = 0; i <= cpufreq_cdev->max_level; i++) {
		unsigned long freq = freq_table[i].frequency * 1000;
		u32 freq_mhz = freq_table[i].frequency / 1000;
		u64 power;
		u32 voltage_mv;

		/*
		 * Find ceil frequency as 'freq' may be slightly lower than OPP
		 * freq due to truncation while converting to kHz.
		 */
		opp = dev_pm_opp_find_freq_ceil(dev, &freq);
		if (IS_ERR(opp)) {
			dev_err(dev, "failed to get opp for %lu frequency\n",
				freq);
			return -EINVAL;
		}

		voltage_mv = dev_pm_opp_get_voltage(opp) / 1000;
		dev_pm_opp_put(opp);

		/*
		 * Do the multiplication with MHz and millivolt so as
		 * to not overflow.
		 */
		power = (u64)capacitance * freq_mhz * voltage_mv * voltage_mv;
		do_div(power, 1000000000);

		/* power is stored in mW */
		freq_table[i].power = power;
	}

	return 0;
}

static u32 cpu_freq_to_power(struct cpufreq_cooling_device *cpufreq_cdev,
			     u32 freq)
{
	int i;
	struct freq_table *freq_table = cpufreq_cdev->freq_table;

	for (i = 1; i <= cpufreq_cdev->max_level; i++)
		if (freq > freq_table[i].frequency)
			break;

	return freq_table[i - 1].power;
}

static u32 cpu_power_to_freq(struct cpufreq_cooling_device *cpufreq_cdev,
			     u32 power)
{
	int i;
	struct freq_table *freq_table = cpufreq_cdev->freq_table;

	for (i = 1; i <= cpufreq_cdev->max_level; i++)
		if (power > freq_table[i].power)
			break;

	return freq_table[i - 1].frequency;
}

/**
 * get_load() - get load for a cpu since last updated
 * @cpufreq_cdev:	&struct cpufreq_cooling_device for this cpu
 * @cpu:	cpu number
 * @cpu_idx:	index of the cpu in time_in_idle*
 *
 * Return: The average load of cpu @cpu in percentage since this
 * function was last called.
 */
static u32 get_load(struct cpufreq_cooling_device *cpufreq_cdev, int cpu,
		    int cpu_idx)
{
	u32 load;
	u64 now, now_idle, delta_time, delta_idle;
	struct time_in_idle *idle_time = &cpufreq_cdev->idle_time[cpu_idx];

	now_idle = get_cpu_idle_time(cpu, &now, 0);
	delta_idle = now_idle - idle_time->time;
	delta_time = now - idle_time->timestamp;

	if (delta_time <= delta_idle)
		load = 0;
	else
		load = div64_u64(100 * (delta_time - delta_idle), delta_time);

	idle_time->time = now_idle;
	idle_time->timestamp = now;

	return load;
}

/**
 * get_dynamic_power() - calculate the dynamic power
 * @cpufreq_cdev:	&cpufreq_cooling_device for this cdev
 * @freq:	current frequency
 *
 * Return: the dynamic power consumed by the cpus described by
 * @cpufreq_cdev.
 */
static u32 get_dynamic_power(struct cpufreq_cooling_device *cpufreq_cdev,
			     unsigned long freq)
{
	u32 raw_cpu_power;

	raw_cpu_power = cpu_freq_to_power(cpufreq_cdev, freq);
	return (raw_cpu_power * cpufreq_cdev->last_load) / 100;
}

/* cpufreq cooling device callback functions are defined below */

/**
 * cpufreq_get_max_state - callback function to get the max cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the max cooling state.
 *
 * Callback for the thermal cooling device to return the cpufreq
 * max cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int cpufreq_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct cpufreq_cooling_device *cpufreq_cdev = cdev->devdata;

	*state = cpufreq_cdev->max_level;
	return 0;
}

/**
 * cpufreq_get_cur_state - callback function to get the current cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: fill this variable with the current cooling state.
 *
 * Callback for the thermal cooling device to return the cpufreq
 * current cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int cpufreq_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct cpufreq_cooling_device *cpufreq_cdev = cdev->devdata;

	*state = cpufreq_cdev->cpufreq_state;

	return 0;
}

/**
 * cpufreq_set_cur_state - callback function to set the current cooling state.
 * @cdev: thermal cooling device pointer.
 * @state: set this variable to the current cooling state.
 *
 * Callback for the thermal cooling device to change the cpufreq
 * current cooling state.
 *
 * Return: 0 on success, an error code otherwise.
 */
static int cpufreq_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state)
{
	struct cpufreq_cooling_device *cpufreq_cdev = cdev->devdata;
	unsigned int clip_freq;

	/* Request state should be less than max_level */
	if (WARN_ON(state > cpufreq_cdev->max_level))
		return -EINVAL;

	/* Check if the old cooling action is same as new cooling action */
	if (cpufreq_cdev->cpufreq_state == state)
		return 0;

	clip_freq = cpufreq_cdev->freq_table[state].frequency;
	cpufreq_cdev->cpufreq_state = state;
	cpufreq_cdev->clipped_freq = clip_freq;

	cpufreq_update_policy(cpufreq_cdev->policy->cpu);

	return 0;
}

/**
 * cpufreq_get_requested_power() - get the current power
 * @cdev:	&thermal_cooling_device pointer
 * @tz:		a valid thermal zone device pointer
 * @power:	pointer in which to store the resulting power
 *
 * Calculate the current power consumption of the cpus in milliwatts
 * and store it in @power.  This function should actually calculate
 * the requested power, but it's hard to get the frequency that
 * cpufreq would have assigned if there were no thermal limits.
 * Instead, we calculate the current power on the assumption that the
 * immediate future will look like the immediate past.
 *
 * We use the current frequency and the average load since this
 * function was last called.  In reality, there could have been
 * multiple opps since this function was last called and that affects
 * the load calculation.  While it's not perfectly accurate, this
 * simplification is good enough and works.  REVISIT this, as more
 * complex code may be needed if experiments show that it's not
 * accurate enough.
 *
 * Return: 0 on success, -E* if getting the static power failed.
 */
static int cpufreq_get_requested_power(struct thermal_cooling_device *cdev,
				       struct thermal_zone_device *tz,
				       u32 *power)
{
	unsigned long freq;
	int i = 0, cpu;
	u32 total_load = 0;
	struct cpufreq_cooling_device *cpufreq_cdev = cdev->devdata;
	struct cpufreq_policy *policy = cpufreq_cdev->policy;
	u32 *load_cpu = NULL;

	freq = cpufreq_quick_get(policy->cpu);

	if (trace_thermal_power_cpu_get_power_enabled()) {
		u32 ncpus = cpumask_weight(policy->related_cpus);

		load_cpu = kcalloc(ncpus, sizeof(*load_cpu), GFP_KERNEL);
	}

	for_each_cpu(cpu, policy->related_cpus) {
		u32 load;

		if (cpu_online(cpu))
			load = get_load(cpufreq_cdev, cpu, i);
		else
			load = 0;

		total_load += load;
		if (load_cpu)
			load_cpu[i] = load;

		i++;
	}

	cpufreq_cdev->last_load = total_load;

	*power = get_dynamic_power(cpufreq_cdev, freq);

	if (load_cpu) {
		trace_thermal_power_cpu_get_power(policy->related_cpus, freq,
						  load_cpu, i, *power);

		kfree(load_cpu);
	}

	return 0;
}

/**
 * cpufreq_state2power() - convert a cpu cdev state to power consumed
 * @cdev:	&thermal_cooling_device pointer
 * @tz:		a valid thermal zone device pointer
 * @state:	cooling device state to be converted
 * @power:	pointer in which to store the resulting power
 *
 * Convert cooling device state @state into power consumption in
 * milliwatts assuming 100% load.  Store the calculated power in
 * @power.
 *
 * Return: 0 on success, -EINVAL if the cooling device state could not
 * be converted into a frequency or other -E* if there was an error
 * when calculating the static power.
 */
static int cpufreq_state2power(struct thermal_cooling_device *cdev,
			       struct thermal_zone_device *tz,
			       unsigned long state, u32 *power)
{
	unsigned int freq, num_cpus;
	struct cpufreq_cooling_device *cpufreq_cdev = cdev->devdata;

	/* Request state should be less than max_level */
	if (WARN_ON(state > cpufreq_cdev->max_level))
		return -EINVAL;

	num_cpus = cpumask_weight(cpufreq_cdev->policy->cpus);

	freq = cpufreq_cdev->freq_table[state].frequency;
	*power = cpu_freq_to_power(cpufreq_cdev, freq) * num_cpus;

	return 0;
}

/**
 * cpufreq_power2state() - convert power to a cooling device state
 * @cdev:	&thermal_cooling_device pointer
 * @tz:		a valid thermal zone device pointer
 * @power:	power in milliwatts to be converted
 * @state:	pointer in which to store the resulting state
 *
 * Calculate a cooling device state for the cpus described by @cdev
 * that would allow them to consume at most @power mW and store it in
 * @state.  Note that this calculation depends on external factors
 * such as the cpu load or the current static power.  Calling this
 * function with the same power as input can yield different cooling
 * device states depending on those external factors.
 *
 * Return: 0 on success, -ENODEV if no cpus are online or -EINVAL if
 * the calculated frequency could not be converted to a valid state.
 * The latter should not happen unless the frequencies available to
 * cpufreq have changed since the initialization of the cpu cooling
 * device.
 */
static int cpufreq_power2state(struct thermal_cooling_device *cdev,
			       struct thermal_zone_device *tz, u32 power,
			       unsigned long *state)
{
	unsigned int target_freq;
	u32 last_load, normalised_power;
	struct cpufreq_cooling_device *cpufreq_cdev = cdev->devdata;
	struct cpufreq_policy *policy = cpufreq_cdev->policy;

	last_load = cpufreq_cdev->last_load ?: 1;
	normalised_power = (power * 100) / last_load;
	target_freq = cpu_power_to_freq(cpufreq_cdev, normalised_power);

	*state = get_level(cpufreq_cdev, target_freq);
	trace_thermal_power_cpu_limit(policy->related_cpus, target_freq, *state,
				      power);
	return 0;
}

/* Bind cpufreq callbacks to thermal cooling device ops */

static struct thermal_cooling_device_ops cpufreq_cooling_ops = {
	.get_max_state = cpufreq_get_max_state,
	.get_cur_state = cpufreq_get_cur_state,
	.set_cur_state = cpufreq_set_cur_state,
};

static struct thermal_cooling_device_ops cpufreq_power_cooling_ops = {
	.get_max_state		= cpufreq_get_max_state,
	.get_cur_state		= cpufreq_get_cur_state,
	.set_cur_state		= cpufreq_set_cur_state,
	.get_requested_power	= cpufreq_get_requested_power,
	.state2power		= cpufreq_state2power,
	.power2state		= cpufreq_power2state,
};

/* Notifier for cpufreq policy change */
static struct notifier_block thermal_cpufreq_notifier_block = {
	.notifier_call = cpufreq_thermal_notifier,
};

static unsigned int find_next_max(struct cpufreq_frequency_table *table,
				  unsigned int prev_max)
{
	struct cpufreq_frequency_table *pos;
	unsigned int max = 0;

	cpufreq_for_each_valid_entry(pos, table) {
		if (pos->frequency > max && pos->frequency < prev_max)
			max = pos->frequency;
	}

	return max;
}

/**
 * __cpufreq_cooling_register - helper function to create cpufreq cooling device
 * @np: a valid struct device_node to the cooling device device tree node
 * @policy: cpufreq policy
 * Normally this should be same as cpufreq policy->related_cpus.
 * @capacitance: dynamic power coefficient for these cpus
 *
 * This interface function registers the cpufreq cooling device with the name
 * "thermal-cpufreq-%x". This api can support multiple instances of cpufreq
 * cooling devices. It also gives the opportunity to link the cooling device
 * with a device tree node, in order to bind it via the thermal DT code.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 * on failure, it returns a corresponding ERR_PTR().
 */
static struct thermal_cooling_device *
__cpufreq_cooling_register(struct device_node *np,
			struct cpufreq_policy *policy, u32 capacitance)
{
	struct thermal_cooling_device *cdev;
	struct cpufreq_cooling_device *cpufreq_cdev;
	char dev_name[THERMAL_NAME_LENGTH];
	unsigned int freq, i, num_cpus;
	int ret;
	struct thermal_cooling_device_ops *cooling_ops;
	bool first;

	if (IS_ERR_OR_NULL(policy)) {
		pr_err("%s: cpufreq policy isn't valid: %p\n", __func__, policy);
		return ERR_PTR(-EINVAL);
	}

	i = cpufreq_table_count_valid_entries(policy);
	if (!i) {
		pr_debug("%s: CPUFreq table not found or has no valid entries\n",
			 __func__);
		return ERR_PTR(-ENODEV);
	}

	cpufreq_cdev = kzalloc(sizeof(*cpufreq_cdev), GFP_KERNEL);
	if (!cpufreq_cdev)
		return ERR_PTR(-ENOMEM);

	cpufreq_cdev->policy = policy;
	num_cpus = cpumask_weight(policy->related_cpus);
	cpufreq_cdev->idle_time = kcalloc(num_cpus,
					 sizeof(*cpufreq_cdev->idle_time),
					 GFP_KERNEL);
	if (!cpufreq_cdev->idle_time) {
		cdev = ERR_PTR(-ENOMEM);
		goto free_cdev;
	}

	/* max_level is an index, not a counter */
	cpufreq_cdev->max_level = i - 1;

	cpufreq_cdev->freq_table = kmalloc_array(i,
					sizeof(*cpufreq_cdev->freq_table),
					GFP_KERNEL);
	if (!cpufreq_cdev->freq_table) {
		cdev = ERR_PTR(-ENOMEM);
		goto free_idle_time;
	}

	ret = ida_simple_get(&cpufreq_ida, 0, 0, GFP_KERNEL);
	if (ret < 0) {
		cdev = ERR_PTR(ret);
		goto free_table;
	}
	cpufreq_cdev->id = ret;

	snprintf(dev_name, sizeof(dev_name), "thermal-cpufreq-%d",
		 cpufreq_cdev->id);

	/* Fill freq-table in descending order of frequencies */
	for (i = 0, freq = -1; i <= cpufreq_cdev->max_level; i++) {
		freq = find_next_max(policy->freq_table, freq);
		cpufreq_cdev->freq_table[i].frequency = freq;

		/* Warn for duplicate entries */
		if (!freq)
			pr_warn("%s: table has duplicate entries\n", __func__);
		else
			pr_debug("%s: freq:%u KHz\n", __func__, freq);
	}

	if (capacitance) {
		ret = update_freq_table(cpufreq_cdev, capacitance);
		if (ret) {
			cdev = ERR_PTR(ret);
			goto remove_ida;
		}

		cooling_ops = &cpufreq_power_cooling_ops;
	} else {
		cooling_ops = &cpufreq_cooling_ops;
	}

	cdev = thermal_of_cooling_device_register(np, dev_name, cpufreq_cdev,
						  cooling_ops);
	if (IS_ERR(cdev))
		goto remove_ida;

	cpufreq_cdev->clipped_freq = cpufreq_cdev->freq_table[0].frequency;

	mutex_lock(&cooling_list_lock);
	/* Register the notifier for first cpufreq cooling device */
	first = list_empty(&cpufreq_cdev_list);
	list_add(&cpufreq_cdev->node, &cpufreq_cdev_list);
	mutex_unlock(&cooling_list_lock);

	if (first)
		cpufreq_register_notifier(&thermal_cpufreq_notifier_block,
					  CPUFREQ_POLICY_NOTIFIER);

	return cdev;

remove_ida:
	ida_simple_remove(&cpufreq_ida, cpufreq_cdev->id);
free_table:
	kfree(cpufreq_cdev->freq_table);
free_idle_time:
	kfree(cpufreq_cdev->idle_time);
free_cdev:
	kfree(cpufreq_cdev);
	return cdev;
}

/**
 * cpufreq_cooling_register - function to create cpufreq cooling device.
 * @policy: cpufreq policy
 *
 * This interface function registers the cpufreq cooling device with the name
 * "thermal-cpufreq-%x". This api can support multiple instances of cpufreq
 * cooling devices.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 * on failure, it returns a corresponding ERR_PTR().
 */
struct thermal_cooling_device *
cpufreq_cooling_register(struct cpufreq_policy *policy)
{
	return __cpufreq_cooling_register(NULL, policy, 0);
}
EXPORT_SYMBOL_GPL(cpufreq_cooling_register);

/**
 * of_cpufreq_cooling_register - function to create cpufreq cooling device.
 * @policy: cpufreq policy
 *
 * This interface function registers the cpufreq cooling device with the name
 * "thermal-cpufreq-%x". This api can support multiple instances of cpufreq
 * cooling devices. Using this API, the cpufreq cooling device will be
 * linked to the device tree node provided.
 *
 * Using this function, the cooling device will implement the power
 * extensions by using a simple cpu power model.  The cpus must have
 * registered their OPPs using the OPP library.
 *
 * It also takes into account, if property present in policy CPU node, the
 * static power consumed by the cpu.
 *
 * Return: a valid struct thermal_cooling_device pointer on success,
 * and NULL on failure.
 */
struct thermal_cooling_device *
of_cpufreq_cooling_register(struct cpufreq_policy *policy)
{
	struct device_node *np = of_get_cpu_node(policy->cpu, NULL);
	struct thermal_cooling_device *cdev = NULL;
	u32 capacitance = 0;

	if (!np) {
		pr_err("cpu_cooling: OF node not available for cpu%d\n",
		       policy->cpu);
		return NULL;
	}

	if (of_find_property(np, "#cooling-cells", NULL)) {
		of_property_read_u32(np, "dynamic-power-coefficient",
				     &capacitance);

		cdev = __cpufreq_cooling_register(np, policy, capacitance);
		if (IS_ERR(cdev)) {
			pr_err("cpu_cooling: cpu%d failed to register as cooling device: %ld\n",
			       policy->cpu, PTR_ERR(cdev));
			cdev = NULL;
		}
	}

	of_node_put(np);
	return cdev;
}
EXPORT_SYMBOL_GPL(of_cpufreq_cooling_register);

/**
 * cpufreq_cooling_unregister - function to remove cpufreq cooling device.
 * @cdev: thermal cooling device pointer.
 *
 * This interface function unregisters the "thermal-cpufreq-%x" cooling device.
 */
void cpufreq_cooling_unregister(struct thermal_cooling_device *cdev)
{
	struct cpufreq_cooling_device *cpufreq_cdev;
	bool last;

	if (!cdev)
		return;

	cpufreq_cdev = cdev->devdata;

	mutex_lock(&cooling_list_lock);
	list_del(&cpufreq_cdev->node);
	/* Unregister the notifier for the last cpufreq cooling device */
	last = list_empty(&cpufreq_cdev_list);
	mutex_unlock(&cooling_list_lock);

	if (last)
		cpufreq_unregister_notifier(&thermal_cpufreq_notifier_block,
					    CPUFREQ_POLICY_NOTIFIER);

	thermal_cooling_device_unregister(cdev);
	ida_simple_remove(&cpufreq_ida, cpufreq_cdev->id);
	kfree(cpufreq_cdev->idle_time);
	kfree(cpufreq_cdev->freq_table);
	kfree(cpufreq_cdev);
}
EXPORT_SYMBOL_GPL(cpufreq_cooling_unregister);
#endif /* CONFIG_CPU_FREQ_THERMAL */

#ifdef CONFIG_CPU_IDLE_THERMAL
/**
 * struct cpuidle_cooling_device - data for the idle cooling device
 * @cdev: a pointer to a struct thermal_cooling_device
 * @cpumask: a cpumask containing the CPU managed by the cooling device
 * @timer: a hrtimer giving the tempo for the idle injection cycles
 * @kref: a kernel refcount on this structure
 * @count: an atomic to keep track of the last task exiting the idle cycle
 * @idle_cycle: an integer defining the duration of the idle injection
 * @state: an normalized integer giving the state of the cooling device
 */
struct cpuidle_cooling_device {
	struct thermal_cooling_device *cdev;
	struct cpumask *cpumask;
	struct hrtimer timer;
	struct kref kref;
	atomic_t count;
	unsigned int idle_cycle;
	unsigned long state;
};

struct cpuidle_cooling_thread {
	struct task_struct *tsk;
	int should_run;
};

static DEFINE_PER_CPU(struct cpuidle_cooling_thread, cpuidle_cooling_thread);
static DEFINE_PER_CPU(struct cpuidle_cooling_device *, cpuidle_cooling_device);

/**
 * cpuidle_cooling_wakeup - Wake up all idle injection threads
 * @idle_cdev: the idle cooling device
 *
 * Every idle injection task belonging to the idle cooling device and
 * running on an online cpu will be wake up by this call.
 */
static void cpuidle_cooling_wakeup(struct cpuidle_cooling_device *idle_cdev)
{
	struct cpuidle_cooling_thread *cct;
	int cpu;

	for_each_cpu_and(cpu, idle_cdev->cpumask, cpu_online_mask) {
		cct = per_cpu_ptr(&cpuidle_cooling_thread, cpu);
		cct->should_run = 1;
		wake_up_process(cct->tsk);
	}
}

/**
 * cpuidle_cooling_wakeup_fn - Running cycle timer callback
 * @timer: a hrtimer structure
 *
 * When the mitigation is acting, the CPU is allowed to run an amount
 * of time, then the idle injection happens for the specified delay
 * and the idle task injection schedules itself until the timer event
 * wakes the idle injection tasks again for a new idle injection
 * cycle. The time between the end of the idle injection and the timer
 * expiration is the allocated running time for the CPU.
 *
 * Always returns HRTIMER_NORESTART
 */
static enum hrtimer_restart cpuidle_cooling_wakeup_fn(struct hrtimer *timer)
{
	struct cpuidle_cooling_device *idle_cdev =
		container_of(timer, struct cpuidle_cooling_device, timer);

	cpuidle_cooling_wakeup(idle_cdev);

	return HRTIMER_NORESTART;
}

/**
 * cpuidle_cooling_runtime - Running time computation
 * @idle_cdev: the idle cooling device
 *
 * The running duration is computed from the idle injection duration
 * which is fixed. If we reach 100% of idle injection ratio, that
 * means the running duration is zero. If we have a 50% ratio
 * injection, that means we have equal duration for idle and for
 * running duration.
 *
 * The formula is deduced as the following:
 *
 *  running = idle x ((100 / ratio) - 1)
 *
 * For precision purpose for integer math, we use the following:
 *
 *  running = (idle x 100) / ratio - idle
 *
 * For example, if we have an injected duration of 50%, then we end up
 * with 10ms of idle injection and 10ms of running duration.
 *
 * Returns a s64 nanosecond based
 */
static s64 cpuidle_cooling_runtime(struct cpuidle_cooling_device *idle_cdev)
{
	s64 next_wakeup;
	unsigned long state = idle_cdev->state;

	/*
	 * The function should not be called when there is no
	 * mitigation because:
	 * - that does not make sense
	 * - we end up with a division by zero
	 */
	if (!state)
		return 0;

	next_wakeup = (s64)((idle_cdev->idle_cycle * 100) / state) -
		idle_cdev->idle_cycle;

	return next_wakeup * NSEC_PER_USEC;
}

/**
 * cpuidle_cooling_injection - Idle injection mainloop thread function
 * @cpu: an integer giving the cpu number the thread is pinned on
 *
 * This main function does basically two operations:
 *
 * - Goes idle for a specific amount of time
 *
 * - Sets a timer to wake up all the idle injection threads after a
 *   running period
 *
 * That happens only when the mitigation is enabled, otherwise the
 * task is scheduled out.
 *
 * In order to keep the tasks synchronized together, it is the last
 * task exiting the idle period which is in charge of setting the
 * timer.
 *
 * This function never returns.
 */
static void cpuidle_cooling_injection(unsigned int cpu)
{
	s64 next_wakeup;

	struct cpuidle_cooling_device *idle_cdev =
		per_cpu(cpuidle_cooling_device, cpu);

	struct cpuidle_cooling_thread *cct =
		per_cpu_ptr(&cpuidle_cooling_thread, cpu);

	atomic_inc(&idle_cdev->count);

	cct->should_run = 0;

	play_idle(idle_cdev->idle_cycle / USEC_PER_MSEC);

	/*
	 * The last CPU waking up is in charge of setting the
	 * timer. If the CPU is hotplugged, the timer will
	 * move to another CPU (which may not belong to the
	 * same cluster) but that is not a problem as the
	 * timer will be set again by another CPU belonging to
	 * the cluster, so this mechanism is self adaptive and
	 * does not require any hotplugging dance.
	 */
	if (!atomic_dec_and_test(&idle_cdev->count))
		return;

	next_wakeup = cpuidle_cooling_runtime(idle_cdev);
	if (next_wakeup)
		hrtimer_start(&idle_cdev->timer, ns_to_ktime(next_wakeup),
			      HRTIMER_MODE_REL_PINNED);
}

static void cpuidle_cooling_setup(unsigned int cpu)
{
	struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO/2 };

	set_freezable();

	sched_setscheduler(current, SCHED_FIFO, &param);
}

static int cpuidle_cooling_should_run(unsigned int cpu)
{
	struct cpuidle_cooling_thread *cct =
		per_cpu_ptr(&cpuidle_cooling_thread, cpu);

	return cct->should_run;
}

static struct smp_hotplug_thread cpuidle_cooling_threads = {
	.store                  = &cpuidle_cooling_thread.tsk,
	.thread_fn              = cpuidle_cooling_injection,
	.thread_comm            = "thermal-idle/%u",
	.thread_should_run	= cpuidle_cooling_should_run,
	.setup                  = cpuidle_cooling_setup,
};

/**
 * cpuidle_cooling_get_max_state - Get the maximum state
 * @cdev  : the thermal cooling device
 * @state : a pointer to the state variable to be filled
 *
 * The function always gives 100 as the injection ratio is percentile
 * based for consistency accros different platforms.
 *
 * The function can not fail, it always returns zero.
 */
static int cpuidle_cooling_get_max_state(struct thermal_cooling_device *cdev,
					 unsigned long *state)
{
	/*
	 * Depending on the configuration or the hardware, the running
	 * cycle and the idle cycle could be different. We want unify
	 * that to an 0..100 interval, so the set state interface will
	 * be the same whatever the platform is.
	 *
	 * The state 100% will make the cluster 100% ... idle. A 0%
	 * injection ratio means no idle injection at all and 50%
	 * means for 10ms of idle injection, we have 10ms of running
	 * time.
	 */
	*state = 100;

	return 0;
}

/**
 * cpuidle_cooling_get_cur_state - Get the current cooling state
 * @cdev: the thermal cooling device
 * @state: a pointer to the state
 *
 * The function just copy the state value from the private thermal
 * cooling device structure, the mapping is 1 <-> 1.
 *
 * The function can not fail, it always returns zero.
 */
static int cpuidle_cooling_get_cur_state(struct thermal_cooling_device *cdev,
					 unsigned long *state)
{
	struct cpuidle_cooling_device *idle_cdev = cdev->devdata;

	*state = idle_cdev->state;

	return 0;
}

/**
 * cpuidle_cooling_set_cur_state - Set the current cooling state
 * @cdev: the thermal cooling device
 * @state: the target state
 *
 * The function checks first if we are initiating the mitigation which
 * in turn wakes up all the idle injection tasks belonging to the idle
 * cooling device. In any case, it updates the internal state for the
 * cooling device.
 *
 * The function can not fail, it always returns zero.
 */
static int cpuidle_cooling_set_cur_state(struct thermal_cooling_device *cdev,
					 unsigned long state)
{
	struct cpuidle_cooling_device *idle_cdev = cdev->devdata;
	unsigned long current_state = idle_cdev->state;

	idle_cdev->state = state;

	if (current_state == 0 && state > 0) {
		pr_debug("Starting cooling cpus '%*pbl'\n",
			 cpumask_pr_args(idle_cdev->cpumask));
		cpuidle_cooling_wakeup(idle_cdev);
	} else if (current_state > 0 && !state)  {
		pr_debug("Stopping cooling cpus '%*pbl'\n",
			 cpumask_pr_args(idle_cdev->cpumask));
	}

	return 0;
}

/**
 * cpuidle_cooling_ops - thermal cooling device ops
 */
static struct thermal_cooling_device_ops cpuidle_cooling_ops = {
	.get_max_state = cpuidle_cooling_get_max_state,
	.get_cur_state = cpuidle_cooling_get_cur_state,
	.set_cur_state = cpuidle_cooling_set_cur_state,
};

/**
 * cpuidle_cooling_release - Kref based release helper
 * @kref: a pointer to the kref structure
 *
 * This function is automatically called by the kref_put function when
 * the idle cooling device refcount reaches zero. At this point, we
 * have the guarantee the structure is no longer in use and we can
 * safely release all the ressources.
 */
static void __init cpuidle_cooling_release(struct kref *kref)
{
	struct cpuidle_cooling_device *idle_cdev =
		container_of(kref, struct cpuidle_cooling_device, kref);

	if (idle_cdev->cdev)
		thermal_cooling_device_unregister(idle_cdev->cdev);

	hrtimer_cancel(&idle_cdev->timer);
	kfree(idle_cdev);
}

/**
 * cpuilde_cooling_unregister - Idle cooling device exit function
 *
 * This function unregisters the cpuidle cooling device and frees the
 * ressources previously allocated by the init function. This function
 * is called when the initialization fails.
 */
static void __init cpuidle_cooling_unregister(void)
{
	struct cpuidle_cooling_device *idle_cdev;
	int cpu;

	for_each_possible_cpu(cpu) {
		idle_cdev = per_cpu(cpuidle_cooling_device, cpu);
		if (idle_cdev)
			kref_put(&idle_cdev->kref, cpuidle_cooling_release);
	}
}


/**
 * cpuidle_cooling_alloc - Allocate and initialize an idle cooling device
 * @cpumask: a cpumask containing all the cpus handled by the cooling device
 * 
 * The function is called at init time only. It allocates and
 * initializes the different fields of the cpuidle cooling device
 *
 * It returns a pointer to an cpuidle_cooling_device structure on
 * success, NULL on error.
 */
static struct cpuidle_cooling_device * __init cpuidle_cooling_alloc(
	cpumask_t *cpumask)
{
	struct cpuidle_cooling_device *idle_cdev;
	int cpu;

	idle_cdev = kzalloc(sizeof(*idle_cdev), GFP_KERNEL);
	if (!idle_cdev)
		return NULL;

	/*
	 * The idle duration injection. As we don't have yet a way to
	 * specify from the DT configuration, let's default to a tick
	 * duration.
	 */
	idle_cdev->idle_cycle = TICK_USEC;

	/*
	 * Initialize the timer to wakeup all the idle injection tasks
	 */
	hrtimer_init(&idle_cdev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	/*
	 * The wakeup function callback which is in charge of waking
	 * up all CPUs belonging to the same cluster
	 */
	idle_cdev->timer.function = cpuidle_cooling_wakeup_fn;

	idle_cdev->cpumask = cpumask;

	/*
	 * Assign on a per cpu basis belonging to the cluster, the per
	 * cpu cpuidle_cooling_device pointer and increment its
	 * refcount on it
	 */
	for_each_cpu(cpu, cpumask) {
		kref_get(&idle_cdev->kref);
		per_cpu(cpuidle_cooling_device, cpu) = idle_cdev;
	}

	return idle_cdev;
}

/**
 * cpuidle_cooling_register - Idle cooling device initialization function
 *
 * This function is in charge of creating a cooling device per cluster
 * and register it to thermal framework. For this we rely on the
 * topology as there is nothing yet describing better the idle state
 * power domains.
 *
 * We create a cpuidle cooling device per cluster. For this reason we
 * must, for each cluster, allocate and initialize the cooling device
 * and for each cpu belonging to this cluster, do the initialization
 * on a cpu basis.
 *
 * This approach for creating the cooling device is needed as we don't
 * have the guarantee the CPU numbering is sequential.
 *
 * Unfortunately, there is no API to browse from top to bottom the
 * topology, cluster->cpu, only the usual for_each_possible_cpu loop.
 * In order to solve that, we use a cpumask to flag the cluster_id we
 * already processed. The cpumask will always have enough room for all
 * the cluster because it is based on NR_CPUS and it is not possible
 * to have more clusters than cpus.
 *
 */
void __init cpuidle_cooling_register(void)
{
	struct cpuidle_cooling_device *idle_cdev = NULL;
	struct thermal_cooling_device *cdev;
	struct device_node *np;
	cpumask_var_t cpumask;
	char dev_name[THERMAL_NAME_LENGTH];
	int ret = -ENOMEM, cpu;
	int cluster_id;

	if (!zalloc_cpumask_var(&cpumask, GFP_KERNEL))
		return;

	for_each_possible_cpu(cpu) {

		cluster_id = topology_physical_package_id(cpu);
		if (cpumask_test_cpu(cluster_id, cpumask))
			continue;

		/*
		 * Allocate the cpuidle cooling device with the list
		 * of the cpus belonging to the cluster.
		 */
		idle_cdev = cpuidle_cooling_alloc(topology_core_cpumask(cpu));
		if (!idle_cdev)
			goto out;

		/*
		 * The thermal cooling device name, we use the
		 * cluster_id as the numbering index for the idle
		 * cooling device.
		 */
		snprintf(dev_name, sizeof(dev_name), "thermal-idle-%d",
			 cluster_id);

		np = of_cpu_device_node_get(cpu);
		cdev = thermal_of_cooling_device_register(np, dev_name,
							  idle_cdev,
							  &cpuidle_cooling_ops);
		if (IS_ERR(cdev)) {
			ret = PTR_ERR(cdev);
			goto out;
		}

		idle_cdev->cdev = cdev;
		cpumask_set_cpu(cluster_id, cpumask);
	}

	ret = smpboot_register_percpu_thread(&cpuidle_cooling_threads);
	if (ret)
		goto out;

	pr_info("Created cpuidle cooling device\n");
out:
	free_cpumask_var(cpumask);

	if (ret) {
		cpuidle_cooling_unregister();
		pr_err("Failed to create idle cooling device (%d)\n", ret);
	}
}
#endif /* CONFIG_CPU_IDLE_THERMAL */

