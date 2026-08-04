/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * STIMER compare/capture contract test for the apollo510_evb Renode
 * platform. Drives the real Ambiq HAL (am_hal_stimer_*) against the
 * simulated STIMER and validates the behaviors the simulation model must
 * honor:
 *
 *  - compare: SCMPR write-is-delta / read-is-absolute, firing at the
 *    requested distance (COMPARE B, so the kernel's COMPARE A is
 *    undisturbed);
 *  - capture: arming via am_hal_stimer_capture_start() incl. a GPIO pin
 *    number >= 128 (Apollo510's 8-bit STSEL), polarity in both
 *    directions, the TIMER->GLOBEN.ENABLEALLINPUTS gate, no re-capture on
 *    a repeated pin level, and SCAPTn/status latching;
 *  - counter jumps: a preloaded counter that passes an armed compare
 *    target raises that comparator's status immediately;
 *  - overflow: STMINTSTAT.OVERFLOW sets on wrap, is clearable via
 *    STMINTCLR (a regression for the interrupt-flag aliasing defect that
 *    latched IRQ 40 forever), and IRQ 40 delivery works.
 *
 * The sequencing partner is compcap.robot (renode-test): the guest prints
 * READY markers and spins on a condition or on the SNVR0 mailbox word,
 * which the robot side advances with monitor commands (sysbus.stimer
 * OnGPIO ..., CounterValue ..., WriteDoubleWord on SNVR0). Every phase
 * prints "COMPCAP <phase>: PASS" or a FAIL line with diagnostics; the
 * robot only ever waits for the PASS lines, so any FAIL times the test
 * out visibly.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>

#include <soc.h>

#define CAP0_PIN     200 /* deliberately >= 128: 8-bit STSEL regression */
#define CAP1_PIN     100
#define COMPB_DELTA  8192   /* 0.25 s at 32768 Hz */
#define COMPC_DELTA  327680 /* 10 s at 32768 Hz */
#define OVF_IRQ      40     /* STIMER_OVF_IRQn: overflow + capture A-D */

/* Poll budget: iterations x 50 us of k_busy_wait = 10 s of virtual time. */
#define POLL_BUDGET  200000

static volatile uint32_t ovf_count;

static uint32_t mailbox_get(void)
{
	return STIMER->SNVR0;
}

static bool wait_mailbox(uint32_t value)
{
	for (int i = 0; i < POLL_BUDGET; i++) {
		if (mailbox_get() == value) {
			return true;
		}
		k_busy_wait(50);
	}
	return false;
}

static bool wait_int_status(uint32_t mask)
{
	for (int i = 0; i < POLL_BUDGET; i++) {
		if ((am_hal_stimer_int_status_get(false) & mask) != 0) {
			return true;
		}
		k_busy_wait(50);
	}
	return false;
}

static void ovf_isr(const void *arg)
{
	ARG_UNUSED(arg);

	if (am_hal_stimer_int_status_get(false) & AM_HAL_STIMER_INT_OVERFLOW) {
		am_hal_stimer_int_clear(AM_HAL_STIMER_INT_OVERFLOW);
		ovf_count++;
	}
}

static int test_compare_b(void)
{
	STIMER->STCFG_b.COMPAREBEN = STIMER_STCFG_COMPAREBEN_ENABLE;

	uint32_t snap = am_hal_stimer_counter_get();

	am_hal_stimer_compare_delta_set(1, COMPB_DELTA);

	uint32_t readback = am_hal_stimer_compare_get(1);
	int32_t target_err = (int32_t)(readback - (snap + COMPB_DELTA));

	/* delta_set internally re-snapshots and compensates the 3-cycle write
	 * latency; allow a few cycles of slack in both directions.
	 */
	if (target_err < -8 || target_err > 8) {
		printk("COMPCAP compareB: FAIL readback 0x%08x snap 0x%08x err %d\n",
		       readback, snap, target_err);
		return -1;
	}

	/* Enabling the comparator against its stale (reset) COMPARE target can
	 * legitimately assert the status right away - the condition is
	 * COUNTER >= COMPARE and the HAL documents that "the application could
	 * get a stale interrupt". The real delta is armed now, so clear the
	 * stale status before timing the fire.
	 */
	am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREB);
	if (am_hal_stimer_int_status_get(false) & AM_HAL_STIMER_INT_COMPAREB) {
		printk("COMPCAP compareB: FAIL stale status re-asserted with target ahead\n");
		return -1;
	}

	if (!wait_int_status(AM_HAL_STIMER_INT_COMPAREB)) {
		printk("COMPCAP compareB: FAIL no fire within budget\n");
		return -1;
	}

	uint32_t fired = am_hal_stimer_counter_get();
	uint32_t elapsed = fired - snap;

	if (elapsed < COMPB_DELTA - 2 || elapsed > COMPB_DELTA + 64) {
		printk("COMPCAP compareB: FAIL fired after %u cycles (wanted ~%u)\n",
		       elapsed, COMPB_DELTA);
		return -1;
	}

	am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREB);
	if (am_hal_stimer_int_status_get(false) & AM_HAL_STIMER_INT_COMPAREB) {
		printk("COMPCAP compareB: FAIL status did not clear\n");
		return -1;
	}

	printk("COMPCAP compareB: PASS\n");
	return 0;
}

static int test_capture0(void)
{
	am_hal_stimer_capture_start(0, CAP0_PIN, false); /* low-to-high */

	if (STIMER->SCAPCTRL0_b.STSEL0 != CAP0_PIN) {
		printk("COMPCAP capture0: FAIL STSEL0 %u != %u\n",
		       (unsigned int)STIMER->SCAPCTRL0_b.STSEL0, CAP0_PIN);
		return -1;
	}
	if (TIMER->GLOBEN_b.ENABLEALLINPUTS == 0) {
		printk("COMPCAP capture0: FAIL GLOBEN.ENABLEALLINPUTS not set\n");
		return -1;
	}

	printk("READY CAP0\n"); /* robot: OnGPIO 200 true */

	if (!wait_int_status(AM_HAL_STIMER_INT_CAPTUREA)) {
		printk("COMPCAP capture0: FAIL no capture within budget\n");
		return -1;
	}

	uint32_t captured = am_hal_stimer_capture_get(0);
	uint32_t now = am_hal_stimer_counter_get();

	if (captured == 0 || (int32_t)(now - captured) < 0) {
		printk("COMPCAP capture0: FAIL captured 0x%08x now 0x%08x\n", captured, now);
		return -1;
	}

	am_hal_stimer_int_clear(AM_HAL_STIMER_INT_CAPTUREA);

	printk("READY CAP0-DUP\n"); /* robot: OnGPIO 200 true again, then SNVR0=1 */

	if (!wait_mailbox(1)) {
		printk("COMPCAP capture0: FAIL mailbox 1 timeout\n");
		return -1;
	}
	if (am_hal_stimer_int_status_get(false) & AM_HAL_STIMER_INT_CAPTUREA) {
		printk("COMPCAP capture0: FAIL re-captured on a repeated high level\n");
		return -1;
	}
	if (am_hal_stimer_capture_get(0) != captured) {
		printk("COMPCAP capture0: FAIL SCAPT0 changed without an edge\n");
		return -1;
	}

	printk("COMPCAP capture0: PASS\n");
	return 0;
}

static int test_capture1(void)
{
	am_hal_stimer_capture_start(1, CAP1_PIN, true); /* high-to-low */

	printk("READY CAP1\n"); /* robot: OnGPIO 100 true (wrong edge), then SNVR0=2 */

	if (!wait_mailbox(2)) {
		printk("COMPCAP capture1: FAIL mailbox 2 timeout\n");
		return -1;
	}
	if (am_hal_stimer_int_status_get(false) & AM_HAL_STIMER_INT_CAPTUREB) {
		printk("COMPCAP capture1: FAIL captured on the wrong (rising) edge\n");
		return -1;
	}

	printk("READY CAP1-FALL\n"); /* robot: OnGPIO 100 false */

	if (!wait_int_status(AM_HAL_STIMER_INT_CAPTUREB)) {
		printk("COMPCAP capture1: FAIL no capture on the falling edge\n");
		return -1;
	}
	if (am_hal_stimer_capture_get(1) == 0) {
		printk("COMPCAP capture1: FAIL SCAPT1 empty after capture\n");
		return -1;
	}

	am_hal_stimer_int_clear(AM_HAL_STIMER_INT_CAPTUREB);

	printk("COMPCAP capture1: PASS\n");
	return 0;
}

static int test_capture_stop(void)
{
	/* GLOBEN.ENABLEALLINPUTS must stay set until the LAST capture unit is
	 * stopped (am_hal_stimer_capture_stop() clears it only when all four
	 * CAPTUREn enables read back disabled).
	 */
	am_hal_stimer_capture_stop(0);
	if (TIMER->GLOBEN_b.ENABLEALLINPUTS == 0) {
		printk("COMPCAP capstop: FAIL GLOBEN cleared with capture 1 still armed\n");
		return -1;
	}

	am_hal_stimer_capture_stop(1);
	if (TIMER->GLOBEN_b.ENABLEALLINPUTS != 0) {
		printk("COMPCAP capstop: FAIL GLOBEN still set after all captures stopped\n");
		return -1;
	}

	printk("COMPCAP capstop: PASS\n");
	return 0;
}

static int test_jump_past_target(void)
{
	STIMER->STCFG_b.COMPARECEN = STIMER_STCFG_COMPARECEN_ENABLE;

	am_hal_stimer_compare_delta_set(2, COMPC_DELTA);

	uint32_t target = am_hal_stimer_compare_get(2);

	/* Discard any stale-target status from the enable (see compareB); the
	 * armed target is 10 s ahead, so the status must stay clear until the
	 * jump.
	 */
	am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREC);
	if (am_hal_stimer_int_status_get(false) & AM_HAL_STIMER_INT_COMPAREC) {
		printk("COMPCAP jump: FAIL stale status re-asserted with target ahead\n");
		return -1;
	}

	/* Publish the absolute target for the robot side, which jumps the
	 * counter just past it.
	 */
	STIMER->SNVR1 = target;

	printk("READY JUMP\n"); /* robot: CounterValue = target + 64, then SNVR0=3 */

	if (!wait_mailbox(3)) {
		printk("COMPCAP jump: FAIL mailbox 3 timeout\n");
		return -1;
	}
	if (!(am_hal_stimer_int_status_get(false) & AM_HAL_STIMER_INT_COMPAREC)) {
		printk("COMPCAP jump: FAIL target 0x%08x not raised after jump (counter 0x%08x)\n",
		       target, am_hal_stimer_counter_get());
		return -1;
	}

	am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREC);

	printk("COMPCAP jump: PASS\n");
	return 0;
}

static int test_overflow(void)
{
	IRQ_CONNECT(OVF_IRQ, 0, ovf_isr, NULL, 0);
	irq_enable(OVF_IRQ);
	am_hal_stimer_int_enable(AM_HAL_STIMER_INT_OVERFLOW);

	printk("READY OVF\n"); /* robot: CounterValue 0xFFFFFF00 */

	/* The wrap (and its interrupt) arrives 256 counter cycles after the
	 * preload; the ISR clears the status. If the model's overflow flag
	 * were not clearable, IRQ 40 would storm and this loop would never
	 * be reached again - a visible timeout.
	 */
	for (int i = 0; i < POLL_BUDGET; i++) {
		if (ovf_count != 0) {
			break;
		}
		k_busy_wait(50);
	}
	if (ovf_count == 0) {
		printk("COMPCAP overflow: FAIL no overflow interrupt\n");
		return -1;
	}

	k_busy_wait(1000);
	if (am_hal_stimer_int_status_get(false) & AM_HAL_STIMER_INT_OVERFLOW) {
		printk("COMPCAP overflow: FAIL status stuck after clear\n");
		return -1;
	}
	if (ovf_count != 1) {
		printk("COMPCAP overflow: FAIL %u interrupts for one wrap\n", ovf_count);
		return -1;
	}

	am_hal_stimer_int_disable(AM_HAL_STIMER_INT_OVERFLOW);

	printk("COMPCAP overflow: PASS\n");
	return 0;
}

int main(void)
{
	printk("STIMER compare/capture contract test\n");

	if (test_compare_b() != 0 || test_capture0() != 0 || test_capture1() != 0 ||
	    test_capture_stop() != 0 || test_jump_past_target() != 0 || test_overflow() != 0) {
		printk("COMPCAP: FAILED\n");
		return 0;
	}

	printk("COMPCAP: ALL PASS\n");
	return 0;
}
