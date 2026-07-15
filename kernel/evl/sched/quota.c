/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Derived from Xenomai Cobalt, https://xenomai.org/
 * Copyright (C) 2013, 2018 Philippe Gerum  <rpm@xenomai.org>
 */

#include <linux/bitmap.h>
#include <asm/div64.h>
#include <evl/sched.h>
#include <evl/memory.h>
#include <uapi/evl/sched-abi.h>

/*
 * Each time a thread is picked from the runnable thread queue, we
 * check whether the group it belongs to still has runtime budget.  If
 * so, a timer is armed to fire when that group has no more budget,
 * would the incoming thread run unpreempted until then
 * (i.e. quota->limit_timer).
 *
 * NOTE: SCHED_QUOTA piggybacks off the per-CPU runqueue of the
 * SCHED_FIFO class in order for the threads of both classes to
 * compete for the same CPU.
 *
 * Otherwise, if no budget remains in the group for running the
 * candidate thread, we move the latter to a local expiry queue
 * maintained by the group. This process is done on the fly as we pull
 * from the runqueue.
 *
 * Updating the remaining budget is done each time the EVL core
 * schedules out a thread undergoing the quota scheduling policy,
 *
 * Finally, a per-CPU timer (quota->refill_timer) periodically ticks
 * in the background, in accordance to the defined quota interval,
 * replenishing per-group budgets, pushing all expired threads back to
 * the runqueue.
 *
 * NOTE: forcing a call to the rescheduling procedure is enough to
 * apply a budget change.
 *
 * CAUTION: quota_group->nr_active does count both the threads from
 * that group linked to the runqueue, _and_ the threads moved to the
 * local expiry queue. As a matter of fact, the expired threads -
 * those for which we consumed all the per-group budget - are still
 * seen as runnable (i.e. not blocked/suspended) by the EVL core. This
 * only means that the SCHED_QUOTA policy won't pick them until the
 * corresponding budget is replenished.
 */

#define MAX_QUOTA_GROUPS  1024

static ktime_t quota_period = 1000000000UL; /* 1s */

static DECLARE_BITMAP(group_map, MAX_QUOTA_GROUPS);

static LIST_HEAD(group_list);

static inline bool current_on_quota(struct evl_quota_group *tg)
{
	struct evl_rq *rq = tg->rq;
	struct evl_thread *curr = rq->curr;
	struct evl_sched_quota *qs = &rq->quota;

	if (curr->quota != tg)
		return false;

	if (curr->state & (EVL_T_READY|EVL_T_KICKED|EVL_THREAD_BLOCK_MASK))
		return false;

	return evl_timer_is_running(&qs->limit_timer);
}

static inline bool group_is_active(struct evl_quota_group *tg)
{
	if (tg->nr_active)
		return true;

	/*
	 * EVL_T_READY set for @thread would mean that it is linked to the
	 * runqueue, in which case tg->nr_active already accounted for
	 * it.
	 */
	return current_on_quota(tg);
}

static inline void replenish_budget(struct evl_sched_quota *qs,
				struct evl_quota_group *tg)
{
	ktime_t budget, credit;

	/*
	 * If a group consumes less than its allotted quota during a
	 * period, the unconsumed time accumulates as a credit for the
	 * next period(s) if permitted.
	 *
	 * A group is allotted a full quota plus its credit, provided
	 * the sum does not exceed a peak value though, not to
	 * monopolize the CPU. Otherwise, the extra time is spread
	 * over multiple periods.
	 *
	 * The credit is dropped whenever a group has no runnable
	 * threads.
	 */

	if (tg->quota == tg->quota_peak) {
		/*
		 * Fast path: we don't accumulate runtime credit.
		 * This includes groups with no runtime limit
		 * (i.e. quota off: quota >= period && quota == peak).
		 */
		tg->run_budget = tg->quota;
		return;
	}

	if (!group_is_active(tg)) {
		/* Drop accumulated credit. */
		tg->run_credit = 0;
		tg->run_budget = tg->quota;
		return;
	}

	budget = ktime_add(tg->run_budget, tg->quota);
	if (budget > tg->quota_peak) {
		/* Too much budget, spread it over periods. */
		tg->run_credit =
			ktime_add(tg->run_credit,
				ktime_sub(budget, tg->quota_peak));
		tg->run_budget = tg->quota_peak;
	} else if (tg->run_credit > 0) {
		credit = ktime_sub(tg->quota_peak, budget);
		/* Consume the accumulated credit. */
		if (tg->run_credit >= credit) {
			tg->run_credit =
				ktime_sub(tg->run_credit, credit);
		} else {
			credit = tg->run_credit;
			tg->run_credit = 0;
		}
		/* Allot extended budget up to the peak value. */
		tg->run_budget = ktime_add(budget, credit);
	} else {
		/* No credit, budget was below peak value. */
		tg->run_budget = budget;
	}
}

static void quota_refill_handler(struct evl_timer *timer) /* oob stage stalled */
{
	struct evl_thread *thread, *tmp;
	struct evl_quota_group *tg;
	struct evl_sched_quota *qs;
	struct evl_rq *rq;

	qs = container_of(timer, struct evl_sched_quota, refill_timer);
	rq = container_of(qs, struct evl_rq, quota);

	raw_spin_lock(&rq->lock);

	list_for_each_entry(tg, &qs->groups, next) {
		/* Allot a new runtime budget for the group. */
		replenish_budget(qs, tg);

		if (tg->run_budget == 0 || list_empty(&tg->expired))
			continue;
		/*
		 * For each group pinned on this CPU, move all expired
		 * threads back to the runqueue. Since those threads
		 * were moved out of the runqueue as we were
		 * considering them for execution, we push them back
		 * in LIFO order to their respective priority group.
		 * The expiry queue is FIFO to keep ordering right
		 * among expired threads.
		 */
		list_for_each_entry_safe_reverse(thread, tmp,
						&tg->expired, quota_expired) {
			list_del_init(&thread->quota_expired);
			/*
			 * thread is still accounted for in
			 * tg->nr_active although parked.
			 */
			evl_add_schedq(&rq->fifo.runnable, thread);
		}
	}

	evl_set_self_resched(evl_get_timer_rq(timer));

	raw_spin_unlock(&rq->lock);
}

static int quota_sum_all(struct evl_sched_quota *qs)
{
	struct evl_quota_group *tg;
	int sum;

	if (list_empty(&qs->groups))
		return 0;

	sum = 0;
	list_for_each_entry(tg, &qs->groups, next)
		sum += tg->quota_percent;

	return sum;
}

static bool quota_setparam(struct evl_thread *thread,
			const union evl_sched_param *p)
{
	struct evl_quota_group *tg;
	struct evl_sched_quota *qs;
	bool effective;

	thread->state &= ~EVL_T_WEAK;
	effective = evl_set_effective_thread_priority(thread, p->quota.prio);

	qs = &thread->rq->quota;
	list_for_each_entry(tg, &qs->groups, next) {
		if (tg->tgid != p->quota.tgid)
			continue;
		if (thread->quota) {
			/* Dequeued earlier by our caller. */
			list_del(&thread->quota_next);
			thread->quota->nr_threads--;
		}
		thread->quota = tg;
		list_add(&thread->quota_next, &tg->members);
		tg->nr_threads++;
		return effective;
	}

	return false;		/* not reached. */
}

static void quota_getparam(struct evl_thread *thread,
			union evl_sched_param *p)
{
	p->quota.prio = thread->cprio;
	p->quota.tgid = thread->quota->tgid;
}

static void quota_trackprio(struct evl_thread *thread,
			const union evl_sched_param *p)
{
	if (p) {
		/* We should not cross groups during PI boost. */
		EVL_WARN_ON(CORE,
			thread->base_class == &evl_sched_quota &&
			thread->quota->tgid != p->quota.tgid);
		thread->cprio = p->quota.prio;
	} else {
		thread->cprio = thread->bprio;
	}
}

static void quota_ceilprio(struct evl_thread *thread, int prio)
{
	if (prio > EVL_QUOTA_MAX_PRIO)
		prio = EVL_QUOTA_MAX_PRIO;

	thread->cprio = prio;
}

static int quota_chkparam(struct evl_thread *thread,
			const union evl_sched_param *p)
{
	struct evl_quota_group *tg;
	struct evl_sched_quota *qs;
	int tgid;

	if (p->quota.prio < EVL_QUOTA_MIN_PRIO ||
		p->quota.prio > EVL_QUOTA_MAX_PRIO)
		return -EINVAL;

	tgid = p->quota.tgid;
	if (tgid < 0 || tgid >= MAX_QUOTA_GROUPS)
		return -EINVAL;

	/*
	 * The group must be managed on the same CPU the thread
	 * currently runs on.
	 */
	qs = &thread->rq->quota;
	list_for_each_entry(tg, &qs->groups, next) {
		if (tg->tgid == tgid)
			return 0;
	}

	/*
	 * If that group exists nevertheless, we give userland a
	 * specific error code.
	 */
	if (test_bit(tgid, group_map))
		return -EPERM;

	return -EINVAL;
}

static void quota_forget(struct evl_thread *thread)
{
	thread->quota->nr_threads--;
	EVL_WARN_ON_ONCE(CORE, thread->quota->nr_threads < 0);
	list_del(&thread->quota_next);
	thread->quota = NULL;
}

static void quota_kick(struct evl_thread *thread)
{
	struct evl_quota_group *tg = thread->quota;
	struct evl_rq *rq = thread->rq;

	/*
	 * Allow a kicked thread to be elected for running until it
	 * switches to in-band context, even if the group it belongs
	 * to lacks runtime budget.
	 */
	if (tg->run_budget == 0 && !list_empty(&thread->quota_expired)) {
		list_del_init(&thread->quota_expired);
		evl_add_schedq_tail(&rq->fifo.runnable, thread);
	}
}

static inline int thread_is_runnable(struct evl_thread *thread)
{
	return thread->quota->run_budget > 0 || (thread->info & EVL_T_KICKED);
}

static void quota_enqueue(struct evl_thread *thread)
{
	struct evl_quota_group *tg = thread->quota;
	struct evl_rq *rq = thread->rq;

	if (!thread_is_runnable(thread))
		list_add_tail(&thread->quota_expired, &tg->expired);
	else
		evl_add_schedq_tail(&rq->fifo.runnable, thread);

	tg->nr_active++;
}

static void quota_dequeue(struct evl_thread *thread)
{
	struct evl_quota_group *tg = thread->quota;
	struct evl_rq *rq = thread->rq;

	if (!list_empty(&thread->quota_expired))
		list_del_init(&thread->quota_expired);
	else
		evl_del_schedq(&rq->fifo.runnable, thread);

	tg->nr_active--;

	EVL_WARN_ON_ONCE(CORE, tg->nr_active < 0);
}

static void quota_requeue(struct evl_thread *thread)
{
	struct evl_quota_group *tg = thread->quota;
	struct evl_rq *rq = thread->rq;

	if (!thread_is_runnable(thread))
		list_add(&thread->quota_expired, &tg->expired);
	else
		evl_add_schedq(&rq->fifo.runnable, thread);

	tg->nr_active++;
}

static void quota_charge(struct evl_quota_group *tg)
{
	ktime_t now, consumed;

	/*
	 * Charge the time consumed by the outgoing thread to the
	 * group it belongs to.
	 */
	now = evl_ktime_monotonic();
	consumed = ktime_sub(now, tg->run_start);
	if (consumed < tg->run_budget)
		tg->run_budget = ktime_sub(tg->run_budget, consumed);
	else
		tg->run_budget = 0;
}

static void quota_charge_and_stop(struct evl_sched_quota *qs,
			struct evl_quota_group *tg)
{
	quota_charge(tg);
	evl_stop_timer(&qs->limit_timer);
}

static void quota_limit_handler(struct evl_timer *timer) /* oob stage stalled */
{
	struct evl_quota_group *tg;
	struct evl_rq *rq;

	rq = container_of(timer, struct evl_rq, quota.limit_timer);
	tg = rq->curr->quota;

	/*
	 * The limit timer should be enabled only when a quota-limited
	 * thread is running. So ticking for a non-quota thread would
	 * be terminally wrong..
	 */
	if (EVL_WARN_ON_ONCE(CORE, !tg))
		return;

	quota_charge(tg);

	/*
	 * Force a rescheduling on the return path of the current
	 * interrupt, causing quota_pick() to check for overruns.
	 */
	raw_spin_lock(&rq->lock);
	evl_set_self_resched(rq);
	raw_spin_unlock(&rq->lock);
}

static struct evl_thread *quota_pick(struct evl_rq *rq)
{
	struct evl_quota_group *otg = rq->curr->quota, *tg;
	struct evl_sched_quota *qs = &rq->quota;
	struct evl_thread *next;

pick:
	next = evl_get_schedq(&rq->fifo.runnable);
	if (next == NULL)
		return NULL;

	/*
	 * Since we piggyback off of the SCHED_FIFO runqueue, make
	 * sure to pick plain fifo threads unconditionally.
	 */
	tg = next->quota;
	if (tg == NULL)
		return next;

	tg->nr_active--;
	EVL_WARN_ON_ONCE(CORE, tg->nr_active < 0);

	/*
	 * Same quota group to charge: keep going transparently and
	 * leave it to the limit timer to detect any overrun for this
	 * group.
	 */
	if (otg == tg)
		return next;

	/*
	 * NOTE: the timer status tells us whether we already charged
	 * the runtime to the group the outgoing thread belongs to,
	 * either because we need to pick again (see below), or the
	 * limit timer has elapsed.
	 */
	if (otg && evl_timer_is_running(&qs->limit_timer))
		quota_charge_and_stop(qs, otg);

	/*
	 * Don't consider budget if kicked, we have to allow this
	 * thread to run unrestricted until it eventually switches to
	 * in-band context.
	 */
	if (next->info & EVL_T_KICKED)
		return next;

	if (ktime_to_ns(tg->run_budget) == 0) {
		/* Park expired group members as we go. */
		list_add_tail(&next->quota_expired, &tg->expired);
		tg->nr_active++;
		goto pick;
	}

	tg->run_start = evl_ktime_monotonic();
	evl_start_timer(&qs->limit_timer,
			ktime_add(tg->run_start, tg->run_budget),
			EVL_INFINITE);

	return next;
}

static void quota_switch(struct evl_thread *prev, struct evl_thread *next)
{
	struct evl_sched_quota *qs = &prev->rq->quota;

	if (!next->quota)
		quota_charge_and_stop(qs, prev->quota);
}

static void quota_migrate(struct evl_thread *thread, struct evl_rq *rq)
{
	union evl_sched_param param;

	/*
	 * Runtime quota groups are defined per-CPU, so leaving the
	 * current CPU means exiting the group. We do this by moving
	 * the target thread to the FIFO class.
	 */
	param.fifo.prio = thread->cprio;
	evl_set_thread_schedparam_locked(thread, &evl_sched_fifo, &param);
}

static const char *quota_name(struct evl_thread *thread)
{
	return "quota";
}

static ssize_t quota_show(struct evl_thread *thread,
			char *buf, ssize_t count)
{
	return snprintf(buf, count, "%d\n",
			thread->quota->tgid);
}

static int quota_create_group(struct evl_quota_group *tg,
			struct evl_rq *rq,
			int *quota_sum_r)
{
	int tgid, nr_groups = MAX_QUOTA_GROUPS;
	struct evl_sched_quota *qs = &rq->quota;
	ktime_t period = READ_ONCE(quota_period);

	assert_hard_lock(&rq->lock);

	tgid = find_first_zero_bit(group_map, nr_groups);
	if (tgid >= nr_groups)
		return -EAGAIN;

	__set_bit(tgid, group_map);
	tg->tgid = tgid;
	tg->rq = rq;
	tg->run_budget = period;
	tg->run_credit = 0;
	tg->quota_percent = 100;
	tg->quota_peak_percent = 100;
	tg->quota = period;
	tg->quota_peak = period;
	tg->nr_active = 0;
	tg->nr_threads = 0;
	INIT_LIST_HEAD(&tg->members);
	INIT_LIST_HEAD(&tg->expired);

	if (list_empty(&qs->groups))
		evl_start_timer(&qs->refill_timer,
				evl_abs_timeout(&qs->refill_timer, period),
				period);

	list_add(&tg->next, &qs->groups);
	*quota_sum_r = quota_sum_all(qs);

	return 0;
}

static int quota_destroy_group(struct evl_quota_group *tg,
			bool force, unsigned long flags,
			int *quota_sum_r)
{
	struct evl_rq *rq = tg->rq;
	struct evl_sched_quota *qs = &rq->quota;
	union evl_sched_param param;
	struct evl_thread *thread;

	assert_hard_lock(&rq->lock);

	if (!list_empty(&tg->members) && !force)
		return -EBUSY;

	/*
	 * Unregister the group before we drop rq->lock. As a result,
	 * it won't accept threads anymore while we are busy moving
	 * the current members to the fifo class, and concurrent
	 * quota_remove requests would receive -EINVAL.
	 */
	__clear_bit(tg->tgid, group_map);
	list_del(&tg->next);

	if (list_empty(&qs->groups))
		evl_stop_timer(&qs->refill_timer);

	/*
	 * Move group members to the fifo class. Since the correct
	 * locking order is thread->lock => rq->lock but we already
	 * hold rq->lock on entry, we do a trylock dance to prevent an
	 * ABBA issue. No livelock is possible since we unregistered
	 * that group already, so &tg->members can only be depleted
	 * (by this loop exclusively).
	 */

	while (!list_empty(&tg->members)) {
		thread = list_first_entry(&tg->members, struct evl_thread,
					quota_next);
		param.fifo.prio = thread->cprio;
		if (raw_spin_trylock(&thread->lock)) {
			evl_set_thread_schedparam_locked(thread,
						&evl_sched_fifo, &param);
			raw_spin_unlock(&thread->lock);
		}
		raw_spin_unlock_irqrestore(&rq->lock, flags);
		cpu_relax();
		raw_spin_lock_irqsave(&rq->lock, flags);
	}

	*quota_sum_r = quota_sum_all(qs);

	return 0;
}

static void quota_set_limit(struct evl_quota_group *tg,
			int quota_percent, int quota_peak_percent,
			int *quota_sum_r)
{
	ktime_t period = READ_ONCE(quota_period);
	struct evl_rq *rq = tg->rq;
	struct evl_thread *thread, *tmp;
	struct evl_sched_quota *qs = &rq->quota;
	ktime_t old_quota = tg->quota;
	ktime_t consumed;
	u64 n;

	assert_hard_lock(&rq->lock);

	if (current_on_quota(tg))
		quota_charge_and_stop(qs, tg);

	if (quota_percent < 0 || quota_percent > 100) { /* Quota off. */
		quota_percent = 100;
		tg->quota = period;
	} else {
		n = period * quota_percent;
		do_div(n, 100);
		tg->quota = n;
	}

	if (quota_peak_percent < quota_percent)
		quota_peak_percent = quota_percent;

	if (quota_peak_percent < 0 || quota_peak_percent > 100) {
		quota_peak_percent = 100;
		tg->quota_peak = period;
	} else {
		n = period * quota_peak_percent;
		do_div(n, 100);
		tg->quota_peak = n;
	}

	tg->quota_percent = quota_percent;
	tg->quota_peak_percent = quota_peak_percent;

	if (tg->run_budget <= old_quota)
		consumed = ktime_sub(old_quota, tg->run_budget);
	else
		consumed = 0;

	if (tg->quota >= consumed)
		tg->run_budget = ktime_sub(tg->quota, consumed);
	else
		tg->run_budget = 0;

	tg->run_credit = 0;	/* Drop accumulated credit. */

	*quota_sum_r = quota_sum_all(qs);

	if (tg->run_budget > 0) {
		list_for_each_entry_safe_reverse(thread, tmp, &tg->expired,
						quota_expired) {
			list_del_init(&thread->quota_expired);
			evl_add_schedq(&rq->fifo.runnable, thread);
		}
	}

	/*
	 * Apply the new budget immediately, in case a member of this
	 * group is currently running.
	 */
	evl_set_resched(rq);
}

static struct evl_quota_group *
find_quota_group(struct evl_rq *rq, int tgid)
{
	struct evl_quota_group *tg;

	assert_hard_lock(&rq->lock);

	/* Quick check using the global id. map first. */
	if (!test_bit(tgid, group_map))
		return NULL;

	/*
	 * The group does exist, we need to check whether it is
	 * attached to the given runqueue.
	 */
	if (list_empty(&rq->quota.groups))
		return NULL;

	list_for_each_entry(tg, &rq->quota.groups, next) {
		if (tg->tgid == tgid)
			return tg;
	}

	return NULL;
}

static ssize_t quota_control(int cpu, union evl_sched_ctlparam *ctlp,
			union evl_sched_ctlinfo *infp)
{
	struct evl_quota_ctlparam *pq = &ctlp->quota;
	struct evl_quota_ctlinfo *iq = &infp->quota;
	struct evl_sched_group *group;
	struct evl_quota_group *tg;
	unsigned long flags;
	int ret, quota_sum;
	struct evl_rq *rq;

	if (cpu < 0 || !cpu_present(cpu) || !is_threading_cpu(cpu))
		return -EINVAL;

	switch (pq->op) {
	case evl_quota_add:
		group = evl_alloc(sizeof(*group));
		if (group == NULL)
			return -ENOMEM;
		tg = &group->quota;
		rq = evl_cpu_rq(cpu);
		raw_spin_lock_irqsave(&rq->lock, flags);
		ret = quota_create_group(tg, rq, &quota_sum);
		if (ret) {
			raw_spin_unlock_irqrestore(&rq->lock, flags);
			evl_free(group);
			return ret;
		}
		list_add(&group->next, &group_list);
		break;
	case evl_quota_remove:
	case evl_quota_force_remove:
		rq = evl_cpu_rq(cpu);
		raw_spin_lock_irqsave(&rq->lock, flags);
		tg = find_quota_group(rq, pq->u.remove.tgid);
		if (tg == NULL)
			goto bad_tgid;
		group = container_of(tg, struct evl_sched_group, quota);
		ret = quota_destroy_group(tg,
					pq->op == evl_quota_force_remove,
					flags,
					&quota_sum);
		if (ret) {
			raw_spin_unlock_irqrestore(&rq->lock, flags);
			return ret;
		}
		list_del(&group->next);
		raw_spin_unlock_irqrestore(&rq->lock, flags);
		evl_free(group);
		goto done;
	case evl_quota_set:
		rq = evl_cpu_rq(cpu);
		raw_spin_lock_irqsave(&rq->lock, flags);
		tg = find_quota_group(rq, pq->u.set.tgid);
		if (tg == NULL)
			goto bad_tgid;
		group = container_of(tg, struct evl_sched_group, quota);
		quota_set_limit(tg, pq->u.set.quota, pq->u.set.quota_peak,
				&quota_sum);
		break;
	case evl_quota_get:
		rq = evl_cpu_rq(cpu);
		raw_spin_lock_irqsave(&rq->lock, flags);
		tg = find_quota_group(rq, pq->u.get.tgid);
		if (tg == NULL)
			goto bad_tgid;
		quota_sum = quota_sum_all(&rq->quota);
		break;
	default:
		return -EINVAL;
	}

	iq->tgid = tg->tgid;
	iq->quota = tg->quota_percent;
	iq->quota_peak = tg->quota_peak_percent;
	raw_spin_unlock_irqrestore(&rq->lock, flags);
	iq->quota_sum = quota_sum;
done:
	evl_schedule();

	return sizeof(*iq);
bad_tgid:
	raw_spin_unlock_irqrestore(&rq->lock, flags);

	return -EINVAL;
}

static void quota_init(struct evl_rq *rq)
{
	struct evl_sched_quota *qs = &rq->quota;

	INIT_LIST_HEAD(&qs->groups);

	evl_init_timer_on_rq(&qs->refill_timer,
			&evl_mono_clock, quota_refill_handler, rq,
			EVL_TIMER_IGRAVITY);
	evl_set_timer_name(&qs->refill_timer, "[quota-refill]");

	evl_init_timer_on_rq(&qs->limit_timer,
			&evl_mono_clock, quota_limit_handler, rq,
			EVL_TIMER_IGRAVITY);
	evl_set_timer_name(&qs->limit_timer, "[quota-limit]");
}

static void __reset_refill_timer(void *arg)
{
	struct evl_rq *rq = evl_cpu_rq(smp_processor_id());
	struct evl_sched_quota *qs = &rq->quota;
	ktime_t period = *(ktime_t *)arg;

	evl_start_timer(&qs->refill_timer,
			evl_abs_timeout(&qs->refill_timer, 0),
			period);
}

void evl_set_quota_period(ktime_t period)
{
	WRITE_ONCE(quota_period, period);

	/*
	 * Writing to the base period variable resets the refill timer
	 * on each runqueue, even if the interval does not actually
	 * change. This is a courtesy to userland, giving it a simple
	 * way to (roughly) synchronize on the start of the periodic
	 * timeline.
	 */
	on_each_cpu_mask(&evl_oob_cpus,	__reset_refill_timer, &period, true);
}

ktime_t evl_get_quota_period(void)
{
	return READ_ONCE(quota_period);
}

struct evl_sched_class evl_sched_quota = {
	.sched_init		=	quota_init,
	.sched_enqueue		=	quota_enqueue,
	.sched_dequeue		=	quota_dequeue,
	.sched_requeue		=	quota_requeue,
	.sched_pick		=	quota_pick,
	.sched_switch		=	quota_switch,
	.sched_migrate		=	quota_migrate,
	.sched_chkparam		=	quota_chkparam,
	.sched_setparam		=	quota_setparam,
	.sched_getparam		=	quota_getparam,
	.sched_trackprio	=	quota_trackprio,
	.sched_ceilprio		=	quota_ceilprio,
	.sched_forget		=	quota_forget,
	.sched_kick		=	quota_kick,
	.sched_name		=	quota_name,
	.sched_show		=	quota_show,
	.sched_control		=	quota_control,
	.weight			=	EVL_CLASS_WEIGHT(2),
	.policy			=	SCHED_QUOTA,
	.name			=	"quota"
};
EXPORT_SYMBOL_GPL(evl_sched_quota);
