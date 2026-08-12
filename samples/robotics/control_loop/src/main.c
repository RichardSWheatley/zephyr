/*
 * Robotics control-loop sample: deterministic 100 Hz closed loop over
 * zbus with acquisition timestamps and a pluggable transport bridge.
 * See README.rst for the control-period budget and how to read the
 * output.
 *
 * main() only orchestrates the demo: a square-wave setpoint, then an
 * emergency stop. The loop itself lives in control.c and never calls
 * into a transport - remote setpoints and telemetry go through the
 * bridge, which is just another zbus observer.
 *
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/sys/printk.h>

#include "app.h"

int main(void)
{
	uint32_t seq = 0;

	printk("robotics control_loop sample on %s (period %u us)\n",
	       CONFIG_BOARD,
	       (uint32_t)k_ticks_to_us_floor64(CONTROL_PERIOD_TICKS));

	control_start();

	/* square-wave setpoint until the demo's cycle budget is spent */
	while (loop_cycles < CONTROL_CYCLES) {
		struct setpoint_msg sp;

		sp.header.stamp_us = now_us();
		sp.header.seq = seq++;
		sp.header.type = WIRE_ID_SETPOINT;
		sp.header.source = 0;
		sp.velocity = ((seq / 2) % 2 == 0) ? 1.0f : -1.0f;
		(void)zbus_chan_pub(&setpoint_chan, &sp, K_MSEC(10));
		k_sleep(K_MSEC(500));
	}

	/* e-stop: the actuator latches to zero via its own listener */
	struct estop_msg stop;

	stop.header.stamp_us = now_us();
	stop.header.seq = 0;
	stop.header.type = WIRE_ID_ESTOP;
	stop.header.source = 0;
	stop.reason = 1;
	(void)zbus_chan_pub(&estop_chan, &stop, K_MSEC(10));
	k_sleep(K_MSEC(50));

	printk("estop: actuator commanded to zero, control loop halted\n");
	return 0;
}
