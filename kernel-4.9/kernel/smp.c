/*
 * Generic helpers for smp ipi calls
 *
 * (C) Jens Axboe <jens.axboe@oracle.com> 2008
 */
#include <linux/irq_work.h>
#include <linux/rcupdate.h>
#include <linux/rculist.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/percpu.h>
#include <linux/init.h>
#include <linux/gfp.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/hypervisor.h>

#include "smpboot.h"

#ifdef VENDOR_EDIT
/* Fuchun.Liao@BSP.CHG.Basic 2020/04/21 add for csd_lock hwt debug */
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#define CSD_NOT_START			0
#define CSD_START_MAGIC			3
#define CSD_NOT_END				0
#define CSD_END_MAGIC			9
static void monitor_csd_data_work_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(monitor_csd_data_work, monitor_csd_data_work_func);
static int create_monitor_csd_data_proc(void);
#endif /* VENDOR_EDIT */

enum {
	CSD_FLAG_LOCK		= 0x01,
	CSD_FLAG_SYNCHRONOUS	= 0x02,
};

struct call_function_data {
	struct call_single_data	__percpu *csd;
	cpumask_var_t		cpumask;
};

static DEFINE_PER_CPU_SHARED_ALIGNED(struct call_function_data, cfd_data);

static DEFINE_PER_CPU_SHARED_ALIGNED(struct llist_head, call_single_queue);

static void flush_smp_call_function_queue(bool warn_cpu_offline);

int smpcfd_prepare_cpu(unsigned int cpu)
{
	struct call_function_data *cfd = &per_cpu(cfd_data, cpu);

	if (!zalloc_cpumask_var_node(&cfd->cpumask, GFP_KERNEL,
				     cpu_to_node(cpu)))
		return -ENOMEM;
	cfd->csd = alloc_percpu(struct call_single_data);
	if (!cfd->csd) {
		free_cpumask_var(cfd->cpumask);
		return -ENOMEM;
	}

	return 0;
}

int smpcfd_dead_cpu(unsigned int cpu)
{
	struct call_function_data *cfd = &per_cpu(cfd_data, cpu);

	free_cpumask_var(cfd->cpumask);
	free_percpu(cfd->csd);
	return 0;
}

int smpcfd_dying_cpu(unsigned int cpu)
{
	/*
	 * The IPIs for the smp-call-function callbacks queued by other
	 * CPUs might arrive late, either due to hardware latencies or
	 * because this CPU disabled interrupts (inside stop-machine)
	 * before the IPIs were sent. So flush out any pending callbacks
	 * explicitly (without waiting for the IPIs to arrive), to
	 * ensure that the outgoing CPU doesn't go offline with work
	 * still pending.
	 */
	flush_smp_call_function_queue(false);
	return 0;
}

void __init call_function_init(void)
{
	int i;

	for_each_possible_cpu(i)
		init_llist_head(&per_cpu(call_single_queue, i));

	smpcfd_prepare_cpu(smp_processor_id());
}

/*
 * csd_lock/csd_unlock used to serialize access to per-cpu csd resources
 *
 * For non-synchronous ipi calls the csd can still be in use by the
 * previous function call. For multi-cpu calls its even more interesting
 * as we'll have to ensure no other cpu is observing our csd.
 */
static __always_inline void csd_lock_wait(struct call_single_data *csd)
{
	smp_cond_load_acquire(&csd->flags, !(VAL & CSD_FLAG_LOCK));
}

static __always_inline void csd_lock(struct call_single_data *csd)
{
	csd_lock_wait(csd);
	csd->flags |= CSD_FLAG_LOCK;

	/*
	 * prevent CPU from reordering the above assignment
	 * to ->flags with any subsequent assignments to other
	 * fields of the specified call_single_data structure:
	 */
	smp_wmb();
}

static __always_inline void csd_unlock(struct call_single_data *csd)
{
	WARN_ON(!(csd->flags & CSD_FLAG_LOCK));

	/*
	 * ensure we're all done before releasing data:
	 */
	smp_store_release(&csd->flags, 0);
}

static DEFINE_PER_CPU_SHARED_ALIGNED(struct call_single_data, csd_data);

/*
 * Insert a previously allocated call_single_data element
 * for execution on the given CPU. data must already have
 * ->func, ->info, and ->flags set.
 */
static int generic_exec_single(int cpu, struct call_single_data *csd,
			       smp_call_func_t func, void *info)
{
	if (cpu == smp_processor_id()) {
		unsigned long flags;

		/*
		 * We can unlock early even for the synchronous on-stack case,
		 * since we're doing this from the same CPU..
		 */
		csd_unlock(csd);
		local_irq_save(flags);
		func(info);
		local_irq_restore(flags);
#ifdef VENDOR_EDIT
/* Fuchun.Liao@BSP.CHG.Basic 2020/04/21 add for smp hwt issue debug */
		csd->end_time = CSD_END_MAGIC;
#endif
		return 0;
	}


	if ((unsigned)cpu >= nr_cpu_ids || !cpu_online(cpu)) {
		csd_unlock(csd);
#ifdef VENDOR_EDIT
/* Fuchun.Liao@BSP.CHG.Basic 2020/04/21 add for smp hwt issue debug */
		csd->end_time = CSD_END_MAGIC;
#endif
		return -ENXIO;
	}

	csd->func = func;
	csd->info = info;

	/*
	 * The list addition should be visible before sending the IPI
	 * handler locks the list to pull the entry off it because of
	 * normal cache coherency rules implied by spinlocks.
	 *
	 * If IPIs can go out of order to the cache coherency protocol
	 * in an architecture, sufficient synchronisation should be added
	 * to arch code to make it appear to obey cache coherency WRT
	 * locking and barrier primitives. Generic code isn't really
	 * equipped to do the right thing...
	 */
	if (llist_add(&csd->llist, &per_cpu(call_single_queue, cpu)))
		arch_send_call_function_single_ipi(cpu);
	
#ifdef VENDOR_EDIT
/* Fuchun.Liao@BSP.CHG.Basic 2020/04/21 add for smp hwt issue debug */
	if(!cpu_online(cpu))
		csd->offline_after_add_list=1;
#endif /* VENDOR_EDIT */

	return 0;
}

/**
 * generic_smp_call_function_single_interrupt - Execute SMP IPI callbacks
 *
 * Invoked by arch to handle an IPI for call function single.
 * Must be called with interrupts disabled.
 */
void generic_smp_call_function_single_interrupt(void)
{
	flush_smp_call_function_queue(true);
}

#ifdef CONFIG_MTPROF
static unsigned long long mt_record_smp_call_func_start(void)
{
	return sched_clock();
}

static void mt_record_smp_call_func_end(struct call_single_data *csd,
					unsigned long long start)
{
#define WARN_LONG_CALL_FUNC_TIME	3000000
	unsigned long long duration = sched_clock() - start;

	if (unlikely(duration > WARN_LONG_CALL_FUNC_TIME))
		pr_warn("func:%pF: too long: %llu ns\n", csd->func, duration);
}
#else
static unsigned long long mt_record_smp_call_func_start(void) { return 0; }
static void mt_record_smp_call_func_end(struct call_single_data *csd,
					unsigned long long start)
{

}
#endif

/**
 * flush_smp_call_function_queue - Flush pending smp-call-function callbacks
 *
 * @warn_cpu_offline: If set to 'true', warn if callbacks were queued on an
 *		      offline CPU. Skip this check if set to 'false'.
 *
 * Flush any pending smp-call-function callbacks queued on this CPU. This is
 * invoked by the generic IPI handler, as well as by a CPU about to go offline,
 * to ensure that all pending IPI callbacks are run before it goes completely
 * offline.
 *
 * Loop through the call_single_queue and run all the queued callbacks.
 * Must be called with interrupts disabled.
 */
static void flush_smp_call_function_queue(bool warn_cpu_offline)
{
	struct llist_head *head;
	struct llist_node *entry;
	struct call_single_data *csd, *csd_next;
	static bool warned;

	WARN_ON(!irqs_disabled());

	head = this_cpu_ptr(&call_single_queue);
	entry = llist_del_all(head);
	entry = llist_reverse_order(entry);

	/* There shouldn't be any pending callbacks on an offline CPU. */
	if (unlikely(warn_cpu_offline && !cpu_online(smp_processor_id()) &&
		     !warned && !llist_empty(head))) {
		warned = true;
		WARN(1, "IPI on offline CPU %d\n", smp_processor_id());

		/*
		 * We don't have to use the _safe() variant here
		 * because we are not invoking the IPI handlers yet.
		 */
		llist_for_each_entry(csd, entry, llist)
			pr_warn("IPI callback %pS sent to offline CPU\n",
				csd->func);
	}

	llist_for_each_entry_safe(csd, csd_next, entry, llist) {
		smp_call_func_t func = csd->func;
		void *info = csd->info;
		unsigned long long start;

		/* Do we wait until *after* callback? */
		if (csd->flags & CSD_FLAG_SYNCHRONOUS) {
			start = mt_record_smp_call_func_start();
			func(info);
			mt_record_smp_call_func_end(csd, start);
#ifdef VENDOR_EDIT
/* Fuchun.Liao@BSP.CHG.Basic 2020/04/21 add for smp hwt issue debug */
			csd->end_time = CSD_END_MAGIC;
#endif /* VENDOR_EDIT */
			csd_unlock(csd);
		} else {
#ifdef VENDOR_EDIT
/* Fuchun.Liao@BSP.CHG.Basic 2020/04/21 add for smp hwt issue debug */
			csd->end_time = CSD_END_MAGIC;
#endif /* VENDOR_EDIT */
			csd_unlock(csd);
			start = mt_record_smp_call_func_start();
			func(info);
			mt_record_smp_call_func_end(csd, start);
		}
	}

	/*
	 * Handle irq works queued remotely by irq_work_queue_on().
	 * Smp functions above are typically synchronous so they
	 * better run first since some other CPUs may be busy waiting
	 * for them.
	 */
	irq_work_run();
}

/*
 * smp_call_function_single - Run a function on a specific CPU
 * @func: The function to run. This must be fast and non-blocking.
 * @info: An arbitrary pointer to pass to the function.
 * @wait: If true, wait until function has completed on other CPUs.
 *
 * Returns 0 on success, else a negative status code.
 */
int smp_call_function_single(int cpu, smp_call_func_t func, void *info,
			     int wait)
{
	struct call_single_data *csd;
	struct call_single_data csd_stack = { .flags = CSD_FLAG_LOCK | CSD_FLAG_SYNCHRONOUS };
	int this_cpu;
	int err;

	/*
	 * prevent preemption and reschedule on another processor,
	 * as well as CPU removal
	 */
	this_cpu = get_cpu();

	/*
	 * Can deadlock when called with interrupts disabled.
	 * We allow cpu's that are not yet online though, as no one else can
	 * send smp call function interrupt to this cpu and as such deadlocks
	 * can't happen.
	 */
	WARN_ON_ONCE(cpu_online(this_cpu) && irqs_disabled()
		     && !oops_in_progress);

	csd = &csd_stack;
	if (!wait) {
		csd = this_cpu_ptr(&csd_data);
		csd_lock(csd);
	}
#ifdef VENDOR_EDIT
/* Fuchun.Liao@BSP.CHG.Basic 2020/04/21 add for smp hwt issue debug */
	csd->source_cpu = this_cpu;
	csd->target_cpu = cpu;
	csd->start_time = CSD_START_MAGIC;
	csd->end_time = CSD_NOT_END;
	csd->offline_after_add_list = 0;
#endif
	err = generic_exec_single(cpu, csd, func, info);

	if (wait)
		csd_lock_wait(csd);

	put_cpu();

	return err;
}
EXPORT_SYMBOL(smp_call_function_single);

/**
 * smp_call_function_single_async(): Run an asynchronous function on a
 * 			         specific CPU.
 * @cpu: The CPU to run on.
 * @csd: Pre-allocated and setup data structure
 *
 * Like smp_call_function_single(), but the call is asynchonous and
 * can thus be done from contexts with disabled interrupts.
 *
 * The caller passes his own pre-allocated data structure
 * (ie: embedded in an object) and is responsible for synchronizing it
 * such that the IPIs performed on the @csd are strictly serialized.
 *
 * NOTE: Be careful, there is unfortunately no current debugging facility to
 * validate the correctness of this serialization.
 */
int smp_call_function_single_async(int cpu, struct call_single_data *csd)
{
	int err = 0;

	preempt_disable();

	/* We could deadlock if we have to wait here with interrupts disabled! */
	if (WARN_ON_ONCE(csd->flags & CSD_FLAG_LOCK))
		csd_lock_wait(csd);

	csd->flags = CSD_FLAG_LOCK;
	smp_wmb();

	err = generic_exec_single(cpu, csd, csd->func, csd->info);
	preempt_enable();

	return err;
}
EXPORT_SYMBOL_GPL(smp_call_function_single_async);

/*
 * smp_call_function_any - Run a function on any of the given cpus
 * @mask: The mask of cpus it can run on.
 * @func: The function to run. This must be fast and non-blocking.
 * @info: An arbitrary pointer to pass to the function.
 * @wait: If true, wait until function has completed.
 *
 * Returns 0 on success, else a negative status code (if no cpus were online).
 *
 * Selection preference:
 *	1) current cpu if in @mask
 *	2) any cpu of current node if in @mask
 *	3) any other online cpu in @mask
 */
int smp_call_function_any(const struct cpumask *mask,
			  smp_call_func_t func, void *info, int wait)
{
	unsigned int cpu;
	const struct cpumask *nodemask;
	int ret;

	/* Try for same CPU (cheapest) */
	cpu = get_cpu();
	if (cpumask_test_cpu(cpu, mask))
		goto call;

	/* Try for same node. */
	nodemask = cpumask_of_node(cpu_to_node(cpu));
	for (cpu = cpumask_first_and(nodemask, mask); cpu < nr_cpu_ids;
	     cpu = cpumask_next_and(cpu, nodemask, mask)) {
		if (cpu_online(cpu))
			goto call;
	}

	/* Any online will do: smp_call_function_single handles nr_cpu_ids. */
	cpu = cpumask_any_and(mask, cpu_online_mask);
call:
	ret = smp_call_function_single(cpu, func, info, wait);
	put_cpu();
	return ret;
}
EXPORT_SYMBOL_GPL(smp_call_function_any);

/**
 * smp_call_function_many(): Run a function on a set of other CPUs.
 * @mask: The set of cpus to run on (only runs on online subset).
 * @func: The function to run. This must be fast and non-blocking.
 * @info: An arbitrary pointer to pass to the function.
 * @wait: If true, wait (atomically) until function has completed
 *        on other CPUs.
 *
 * If @wait is true, then returns once @func has returned.
 *
 * You must not call this function with disabled interrupts or from a
 * hardware interrupt handler or from a bottom half handler. Preemption
 * must be disabled when calling this function.
 */
void smp_call_function_many(const struct cpumask *mask,
			    smp_call_func_t func, void *info, bool wait)
{
	struct call_function_data *cfd;
	int cpu, next_cpu, this_cpu = smp_processor_id();

	/*
	 * Can deadlock when called with interrupts disabled.
	 * We allow cpu's that are not yet online though, as no one else can
	 * send smp call function interrupt to this cpu and as such deadlocks
	 * can't happen.
	 */
	WARN_ON_ONCE(cpu_online(this_cpu) && irqs_disabled()
		     && !oops_in_progress && !early_boot_irqs_disabled);

	/* Try to fastpath.  So, what's a CPU they want? Ignoring this one. */
	cpu = cpumask_first_and(mask, cpu_online_mask);
	if (cpu == this_cpu)
		cpu = cpumask_next_and(cpu, mask, cpu_online_mask);

	/* No online cpus?  We're done. */
	if (cpu >= nr_cpu_ids)
		return;

	/* Do we have another CPU which isn't us? */
	next_cpu = cpumask_next_and(cpu, mask, cpu_online_mask);
	if (next_cpu == this_cpu)
		next_cpu = cpumask_next_and(next_cpu, mask, cpu_online_mask);

	/* Fastpath: do that cpu by itself. */
	if (next_cpu >= nr_cpu_ids) {
		smp_call_function_single(cpu, func, info, wait);
		return;
	}

	cfd = this_cpu_ptr(&cfd_data);

	cpumask_and(cfd->cpumask, mask, cpu_online_mask);
	cpumask_clear_cpu(this_cpu, cfd->cpumask);

	/* Some callers race with other cpus changing the passed mask */
	if (unlikely(!cpumask_weight(cfd->cpumask)))
		return;

	for_each_cpu(cpu, cfd->cpumask) {
		struct call_single_data *csd = per_cpu_ptr(cfd->csd, cpu);

		csd_lock(csd);
		if (wait)
			csd->flags |= CSD_FLAG_SYNCHRONOUS;
		csd->func = func;
		csd->info = info;
		llist_add(&csd->llist, &per_cpu(call_single_queue, cpu));
	}

	/* Send a message to all CPUs in the map */
	arch_send_call_function_ipi_mask(cfd->cpumask);

	if (wait) {
		for_each_cpu(cpu, cfd->cpumask) {
			struct call_single_data *csd;

			csd = per_cpu_ptr(cfd->csd, cpu);
			csd_lock_wait(csd);
		}
	}
}
EXPORT_SYMBOL(smp_call_function_many);

/**
 * smp_call_function(): Run a function on all other CPUs.
 * @func: The function to run. This must be fast and non-blocking.
 * @info: An arbitrary pointer to pass to the function.
 * @wait: If true, wait (atomically) until function has completed
 *        on other CPUs.
 *
 * Returns 0.
 *
 * If @wait is true, then returns once @func has returned; otherwise
 * it returns just before the target cpu calls @func.
 *
 * You must not call this function with disabled interrupts or from a
 * hardware interrupt handler or from a bottom half handler.
 */
int smp_call_function(smp_call_func_t func, void *info, int wait)
{
	preempt_disable();
	smp_call_function_many(cpu_online_mask, func, info, wait);
	preempt_enable();

	return 0;
}
EXPORT_SYMBOL(smp_call_function);

/* Setup configured maximum number of CPUs to activate */
unsigned int setup_max_cpus = NR_CPUS;
EXPORT_SYMBOL(setup_max_cpus);


/*
 * Setup routine for controlling SMP activation
 *
 * Command-line option of "nosmp" or "maxcpus=0" will disable SMP
 * activation entirely (the MPS table probe still happens, though).
 *
 * Command-line option of "maxcpus=<NUM>", where <NUM> is an integer
 * greater than 0, limits the maximum number of CPUs activated in
 * SMP mode to <NUM>.
 */

void __weak arch_disable_smp_support(void) { }

static int __init nosmp(char *str)
{
	setup_max_cpus = 0;
	arch_disable_smp_support();

	return 0;
}

early_param("nosmp", nosmp);

/* this is hard limit */
static int __init nrcpus(char *str)
{
	int nr_cpus;

	get_option(&str, &nr_cpus);
	if (nr_cpus > 0 && nr_cpus < nr_cpu_ids)
		nr_cpu_ids = nr_cpus;

	return 0;
}

early_param("nr_cpus", nrcpus);

static int __init maxcpus(char *str)
{
	get_option(&str, &setup_max_cpus);
	if (setup_max_cpus == 0)
		arch_disable_smp_support();

	return 0;
}

early_param("maxcpus", maxcpus);

/* Setup number of possible processor ids */
int nr_cpu_ids __read_mostly = NR_CPUS;
EXPORT_SYMBOL(nr_cpu_ids);

/* An arch may set nr_cpu_ids earlier if needed, so this would be redundant */
void __init setup_nr_cpu_ids(void)
{
	nr_cpu_ids = find_last_bit(cpumask_bits(cpu_possible_mask),NR_CPUS) + 1;
}

void __weak smp_announce(void)
{
	printk(KERN_INFO "Brought up %d CPUs\n", num_online_cpus());
}

/* Called by boot processor to activate the rest. */
void __init smp_init(void)
{
	unsigned int cpu;
#ifdef VENDOR_EDIT
/* Fuchun.Liao@BSP.CHG.Basic 2020/04/20 add for smp hwt debug */
	static bool monitor_csd_data_init = false;
#endif /* VENDOR_EDIT */

	idle_threads_init();
	cpuhp_threads_init();

	/* FIXME: This should be done in userspace --RR */
	for_each_present_cpu(cpu) {
		if (num_online_cpus() >= setup_max_cpus)
			break;
		if (!cpu_online(cpu))
			cpu_up(cpu);
	}

	/* Final decision about SMT support */
	cpu_smt_check_topology();
	/* Any cleanup work */
	smp_announce();
	smp_cpus_done(setup_max_cpus);
#ifdef VENDOR_EDIT
/* Fuchun.Liao@BSP.CHG.Basic 2020/04/20 add for smp hwt debug */
	if (monitor_csd_data_init == false) {
		monitor_csd_data_init = true;
		if (!create_monitor_csd_data_proc()) {
			schedule_delayed_work(&monitor_csd_data_work, 
				round_jiffies_relative(msecs_to_jiffies(60000)));
		}
	}
#endif /* VENDOR_EDIT */
}

/*
 * Call a function on all processors.  May be used during early boot while
 * early_boot_irqs_disabled is set.  Use local_irq_save/restore() instead
 * of local_irq_disable/enable().
 */
int on_each_cpu(void (*func) (void *info), void *info, int wait)
{
	unsigned long flags;
	int ret = 0;

	preempt_disable();
	ret = smp_call_function(func, info, wait);
	local_irq_save(flags);
	func(info);
	local_irq_restore(flags);
	preempt_enable();
	return ret;
}
EXPORT_SYMBOL(on_each_cpu);

/**
 * on_each_cpu_mask(): Run a function on processors specified by
 * cpumask, which may include the local processor.
 * @mask: The set of cpus to run on (only runs on online subset).
 * @func: The function to run. This must be fast and non-blocking.
 * @info: An arbitrary pointer to pass to the function.
 * @wait: If true, wait (atomically) until function has completed
 *        on other CPUs.
 *
 * If @wait is true, then returns once @func has returned.
 *
 * You must not call this function with disabled interrupts or from a
 * hardware interrupt handler or from a bottom half handler.  The
 * exception is that it may be used during early boot while
 * early_boot_irqs_disabled is set.
 */
void on_each_cpu_mask(const struct cpumask *mask, smp_call_func_t func,
			void *info, bool wait)
{
	int cpu = get_cpu();

	smp_call_function_many(mask, func, info, wait);
	if (cpumask_test_cpu(cpu, mask)) {
		unsigned long flags;
		local_irq_save(flags);
		func(info);
		local_irq_restore(flags);
	}
	put_cpu();
}
EXPORT_SYMBOL(on_each_cpu_mask);

/*
 * on_each_cpu_cond(): Call a function on each processor for which
 * the supplied function cond_func returns true, optionally waiting
 * for all the required CPUs to finish. This may include the local
 * processor.
 * @cond_func:	A callback function that is passed a cpu id and
 *		the the info parameter. The function is called
 *		with preemption disabled. The function should
 *		return a blooean value indicating whether to IPI
 *		the specified CPU.
 * @func:	The function to run on all applicable CPUs.
 *		This must be fast and non-blocking.
 * @info:	An arbitrary pointer to pass to both functions.
 * @wait:	If true, wait (atomically) until function has
 *		completed on other CPUs.
 * @gfp_flags:	GFP flags to use when allocating the cpumask
 *		used internally by the function.
 *
 * The function might sleep if the GFP flags indicates a non
 * atomic allocation is allowed.
 *
 * Preemption is disabled to protect against CPUs going offline but not online.
 * CPUs going online during the call will not be seen or sent an IPI.
 *
 * You must not call this function with disabled interrupts or
 * from a hardware interrupt handler or from a bottom half handler.
 */
void on_each_cpu_cond(bool (*cond_func)(int cpu, void *info),
			smp_call_func_t func, void *info, bool wait,
			gfp_t gfp_flags)
{
	cpumask_var_t cpus;
	int cpu, ret;

	might_sleep_if(gfpflags_allow_blocking(gfp_flags));

	if (likely(zalloc_cpumask_var(&cpus, (gfp_flags|__GFP_NOWARN)))) {
		preempt_disable();
		for_each_online_cpu(cpu)
			if (cond_func(cpu, info))
				cpumask_set_cpu(cpu, cpus);
		on_each_cpu_mask(cpus, func, info, wait);
		preempt_enable();
		free_cpumask_var(cpus);
	} else {
		/*
		 * No free cpumask, bother. No matter, we'll
		 * just have to IPI them one by one.
		 */
		preempt_disable();
		for_each_online_cpu(cpu)
			if (cond_func(cpu, info)) {
				ret = smp_call_function_single(cpu, func,
								info, wait);
				WARN_ON_ONCE(ret);
			}
		preempt_enable();
	}
}
EXPORT_SYMBOL(on_each_cpu_cond);

static void do_nothing(void *unused)
{
}

/**
 * kick_all_cpus_sync - Force all cpus out of idle
 *
 * Used to synchronize the update of pm_idle function pointer. It's
 * called after the pointer is updated and returns after the dummy
 * callback function has been executed on all cpus. The execution of
 * the function can only happen on the remote cpus after they have
 * left the idle function which had been called via pm_idle function
 * pointer. So it's guaranteed that nothing uses the previous pointer
 * anymore.
 */
void kick_all_cpus_sync(void)
{
	/* Make sure the change is visible before we kick the cpus */
	smp_mb();
	smp_call_function(do_nothing, NULL, 1);
}
EXPORT_SYMBOL_GPL(kick_all_cpus_sync);

/**
 * wake_up_all_idle_cpus - break all cpus out of idle
 * wake_up_all_idle_cpus try to break all cpus which is in idle state even
 * including idle polling cpus, for non-idle cpus, we will do nothing
 * for them.
 */
void wake_up_all_idle_cpus(void)
{
	int cpu;

	preempt_disable();
	for_each_online_cpu(cpu) {
		if (cpu == smp_processor_id())
			continue;

		wake_up_if_idle(cpu);
	}
	preempt_enable();
}
EXPORT_SYMBOL_GPL(wake_up_all_idle_cpus);

/**
 * smp_call_on_cpu - Call a function on a specific cpu
 *
 * Used to call a function on a specific cpu and wait for it to return.
 * Optionally make sure the call is done on a specified physical cpu via vcpu
 * pinning in order to support virtualized environments.
 */
struct smp_call_on_cpu_struct {
	struct work_struct	work;
	struct completion	done;
	int			(*func)(void *);
	void			*data;
	int			ret;
	int			cpu;
};

static void smp_call_on_cpu_callback(struct work_struct *work)
{
	struct smp_call_on_cpu_struct *sscs;

	sscs = container_of(work, struct smp_call_on_cpu_struct, work);
	if (sscs->cpu >= 0)
		hypervisor_pin_vcpu(sscs->cpu);
	sscs->ret = sscs->func(sscs->data);
	if (sscs->cpu >= 0)
		hypervisor_pin_vcpu(-1);

	complete(&sscs->done);
}

int smp_call_on_cpu(unsigned int cpu, int (*func)(void *), void *par, bool phys)
{
	struct smp_call_on_cpu_struct sscs = {
		.done = COMPLETION_INITIALIZER_ONSTACK(sscs.done),
		.func = func,
		.data = par,
		.cpu  = phys ? cpu : -1,
	};

	INIT_WORK_ONSTACK(&sscs.work, smp_call_on_cpu_callback);

	if (cpu >= nr_cpu_ids || !cpu_online(cpu))
		return -ENXIO;

	queue_work_on(cpu, system_wq, &sscs.work);
	wait_for_completion(&sscs.done);

	return sscs.ret;
}
EXPORT_SYMBOL_GPL(smp_call_on_cpu);

#ifdef VENDOR_EDIT
/* Fuchun.Liao@BSP.CHG.Basic 2020/04/21 add for smp hwt issue debug */
#define LOOP_COUNT_60S		20
static int monitor_csd_enable = 1;
static int  workaround_csd_enable = 1;
#define MIN_HANG_TIME			1	// 1 * 3s
#define DEFAULT_HANG_TIME		3	// 3 * 3s
static int hang_time = DEFAULT_HANG_TIME;
#define MONITOR_CSD_INTERVAL	3000
static void monitor_csd_data_work_func(struct work_struct *work)
{
	struct call_single_data *csd;
	static int offline_count = 0;
	static int empty_count = 0;
	static int hang_count = 0;
	static int send_ipi_count = 0;
	static int loop_count = 0;
	bool queue_is_empty = false;
	int cpu;

	if (monitor_csd_enable == 0) {
		offline_count = 0;
		empty_count = 0;
		hang_count = 0;
		send_ipi_count = 0;
		pr_info("monitor_csd_data disable\n");
		return;
	}
	for_each_possible_cpu(cpu) {
		csd = per_cpu_ptr(&csd_data, cpu);
		if (csd->start_time!= CSD_NOT_START && csd->end_time == CSD_NOT_END && (csd->flags & CSD_FLAG_LOCK)) {
			if(!cpu_online(csd->target_cpu)) {
				pr_info("cpu%d smp call cpu%d, cpu%d is offline, source:%d\n",
					cpu, csd->target_cpu, csd->target_cpu, csd->source_cpu);
				offline_count++;
			}
			if (llist_empty(&per_cpu(call_single_queue, csd->target_cpu))) {
				pr_info("cpu%d smp call cpu%d, cpu%d call_single_quue is empty, source:%d\n",
					cpu, csd->target_cpu, csd->target_cpu, csd->source_cpu);
				queue_is_empty = true;
				empty_count++;
			} else {
				queue_is_empty = false;
			}
			csd->hang_time++;
			hang_count++;
			if (csd->hang_time >= hang_time) {
				csd->hang_time = 0;
				pr_info("cpu%d maybe hang because of cpu%d, source:%d empty:%d\n",
					cpu, csd->target_cpu, csd->source_cpu, queue_is_empty);
				if (workaround_csd_enable && (queue_is_empty == false)) {
					send_ipi_count++;
					arch_send_call_function_single_ipi(csd->target_cpu);
				}
			}
		} else {
			csd->hang_time = 0;
		}
	}

	if (offline_count > 1000) 
		offline_count = 0;
	if (empty_count > 1000) 
		empty_count = 0;
	if (hang_count > 1000)
		hang_count = 0;
	if (send_ipi_count > 1000)
		send_ipi_count = 0;
	
	loop_count++;
	if (loop_count > LOOP_COUNT_60S) {
		loop_count = 0;
		pr_info("monitor_csd_data [%d/%d/%d/%d/%d]\n",
			offline_count, empty_count, hang_count, send_ipi_count, workaround_csd_enable);
	}
	schedule_delayed_work(&monitor_csd_data_work, 
		round_jiffies_relative(msecs_to_jiffies(MONITOR_CSD_INTERVAL)));
}

static ssize_t monitor_csd_data_proc_read(struct file *file, char __user *buf, 
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(&page[len],"%d %d %d\n", 
		monitor_csd_enable, workaround_csd_enable, hang_time);

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

static ssize_t monitor_csd_data_proc_write(struct file *file, const char __user *buf, 
		size_t count,loff_t *off)
{
	int monitor_enable = 0;
	int workaround_enable = 0;
	int timeout = 0;
	int ret = 0;
    	char buffer[32] = {0};
	int monitor_enable_pre = monitor_csd_enable;

	if (count > 32) {
		count = 32;
	}

	if (copy_from_user(buffer, buf, count)) {
		pr_err("%s: read proc input error.\n", __func__);
		return count;
	}
	ret = sscanf(buffer, "%d %d %d", &monitor_enable, &workaround_enable, &timeout);

	if (ret == 1) {
		monitor_csd_enable = monitor_enable;
	} else if (ret ==2) {
		workaround_csd_enable = workaround_enable;
	} else if (ret == 3) {
		if (timeout < MIN_HANG_TIME)
			hang_time = DEFAULT_HANG_TIME;
		else
			hang_time = timeout;
	} else {
		pr_info("%s invalid content: '%s', length = %zd\n", __func__, buf, count);
	}

	if (monitor_enable_pre == 0 && monitor_csd_enable == 1) {
		schedule_delayed_work(&monitor_csd_data_work, 
				round_jiffies_relative(msecs_to_jiffies(2000)));
	}

	pr_info("%s: monitor_enable = %d workaround_enable = %d timeout = %d ret = %d\n", 
		__func__, monitor_csd_enable, workaround_csd_enable, hang_time, ret);

	return count;

}

struct file_operations monitor_csd_data_proc_fops = {
	.read = monitor_csd_data_proc_read,
	.write = monitor_csd_data_proc_write,
};

static int create_monitor_csd_data_proc(void)
{
	struct proc_dir_entry *p = NULL;
	
	p = proc_create("monitor_csd_data", 0664, NULL, &monitor_csd_data_proc_fops);
	if (!p) {
		pr_err("%s fail\n", __func__);
		return -1;
	} else {
		return 0;
	}
}

#endif /* VENDOR_EDIT */
