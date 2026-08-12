/*
 * The three stages of the loop, decoupled by zbus channels:
 *
 *   sensor thread (100 Hz, absolute deadlines) -> sensor_chan
 *   controller (subscriber thread, PI)         -> actuator_chan
 *   actuator (listener)                        -> plant / hardware
 *
 * The sensor thread sleeps to absolute deadlines (K_TIMEOUT_ABS_TICKS),
 * so the period cannot drift with execution time, and measures its own
 * wakeup jitter against each deadline. The actuator listener measures
 * end-to-end latency: sensor acquisition stamp -> actuation, possible
 * because derived messages propagate the causing sample's stamp.
 *
 * An e-stop latches the actuator to zero: safety does not depend on the
 * controller thread being alive.
 *
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/sys/printk.h>

#include "app.h"

ZBUS_CHAN_DEFINE(setpoint_chan, struct setpoint_msg, NULL, NULL,
		 ZBUS_OBSERVERS_EMPTY, ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(sensor_chan, struct sensor_msg, NULL, NULL,
		 ZBUS_OBSERVERS(controller_sub), ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(actuator_chan, struct actuator_msg, NULL, NULL,
		 ZBUS_OBSERVERS(actuator_listener), ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(estop_chan, struct estop_msg, NULL, NULL,
		 ZBUS_OBSERVERS(estop_listener), ZBUS_MSG_INIT(0));

volatile uint32_t loop_cycles;

/* per-100-cycle stats windows, integer microseconds */
static uint32_t wake_min = UINT32_MAX, wake_max, wake_sum;
static volatile uint32_t e2e_min = UINT32_MAX, e2e_max, e2e_sum, e2e_n;
static volatile bool stopped;

/* ---- actuator (runs in the publisher's context) -------------------------- */

static void actuator_cb(const struct zbus_channel *chan)
{
	const struct actuator_msg *m = zbus_chan_const_msg(chan);

	if (stopped) {
		plant_apply(0.0f);
		return;
	}
	plant_apply(m->command);

	/* end-to-end: sensor acquisition -> actuation applied */
	uint32_t e2e = (uint32_t)(now_us() - m->header.stamp_us);

	e2e_min = MIN(e2e_min, e2e);
	e2e_max = MAX(e2e_max, e2e);
	e2e_sum += e2e;
	e2e_n++;
}
ZBUS_LISTENER_DEFINE(actuator_listener, actuator_cb);

static void estop_cb(const struct zbus_channel *chan)
{
	ARG_UNUSED(chan);
	stopped = true;
	plant_apply(0.0f);
}
ZBUS_LISTENER_DEFINE(estop_listener, estop_cb);

/* ---- controller: PI against the latest setpoint -------------------------- */

ZBUS_SUBSCRIBER_DEFINE(controller_sub, 4);

#define KP 0.8f
#define KI 2.5f

static void controller_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	const struct zbus_channel *chan;
	float integ = 0.0f;
	uint32_t seq = 0;

	while (zbus_sub_wait(&controller_sub, &chan, K_FOREVER) == 0) {
		struct sensor_msg s;
		struct setpoint_msg sp;
		struct actuator_msg out;

		if (chan != &sensor_chan || stopped) {
			continue;
		}
		if (zbus_chan_read(&sensor_chan, &s, K_MSEC(2)) != 0 ||
		    zbus_chan_read(&setpoint_chan, &sp, K_MSEC(2)) != 0) {
			continue;
		}

		float err = sp.velocity - s.velocity;
		float dt = 1.0f / CONTROL_RATE_HZ;

		integ += err * dt;
		integ = CLAMP(integ, -0.5f, 0.5f);

		float u = CLAMP(KP * err + KI * integ, -1.0f, 1.0f);

		/* propagate the causing sample's acquisition stamp */
		out.header.stamp_us = s.header.stamp_us;
		out.header.seq = seq++;
		out.header.type = WIRE_ID_ACTUATOR;
		out.header.source = 0;
		out.command = u;
		(void)zbus_chan_pub(&actuator_chan, &out, K_MSEC(2));
	}
}
K_THREAD_DEFINE(controller_tid, 2048, controller_thread, NULL, NULL, NULL, 5,
		0, 0);

/* ---- sensor: 100 Hz on absolute deadlines -------------------------------- */

static void sensor_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	uint32_t seq = 0;
	uint64_t hw_freq = (uint64_t)sys_clock_hw_cycles_per_sec();

	/* align to a tick, then run on absolute deadlines */
	k_sleep(K_TICKS(1));
	int64_t next = k_uptime_ticks();

	while (1) {
		next += CONTROL_PERIOD_TICKS;
		k_sleep(K_TIMEOUT_ABS_TICKS(next));

		/*
		 * Wakeup jitter vs the deadline, measured in the kernel
		 * timer's own clock domain (same clock as the deadline, so
		 * no cross-oscillator drift; resolution = 1 hw cycle).
		 */
		uint64_t deadline_cyc =
			((uint64_t)next * hw_freq) /
			CONFIG_SYS_CLOCK_TICKS_PER_SEC;
		uint64_t now_cyc = IS_ENABLED(
					   CONFIG_TIMER_HAS_64BIT_CYCLE_COUNTER)
					   ? k_cycle_get_64()
					   : (uint64_t)k_cycle_get_32();
		uint32_t wake_us =
			(now_cyc > deadline_cyc)
				? (uint32_t)k_cyc_to_us_floor64(now_cyc -
								deadline_cyc)
				: 0;

		struct sensor_msg s;

		s.header.stamp_us = now_us(); /* acquisition instant */
		s.header.seq = seq++;
		s.header.type = WIRE_ID_SENSOR;
		s.header.source = 0;
		s.velocity = plant_step(1.0f / CONTROL_RATE_HZ);
		(void)zbus_chan_pub(&sensor_chan, &s, K_MSEC(2));

		if (stopped) {
			continue; /* keep sampling; stats are done */
		}

		wake_min = MIN(wake_min, wake_us);
		wake_max = MAX(wake_max, wake_us);
		wake_sum += wake_us;
		loop_cycles++;

		if ((loop_cycles % 100) == 0) {
			printk("loop: n=%u period_us=%u wake_jitter_us min=%u avg=%u max=%u e2e_us min=%u avg=%u max=%u\n",
			       loop_cycles,
			       (uint32_t)k_ticks_to_us_floor64(
				       CONTROL_PERIOD_TICKS),
			       wake_min, wake_sum / 100, wake_max,
			       e2e_min, e2e_n ? e2e_sum / e2e_n : 0, e2e_max);
			if (loop_cycles == CONTROL_CYCLES) {
				printk("RECORD: {\"sample\":\"control_loop\",\"board\":\"%s\",\"n\":%u,\"wake_avg_us\":%u,\"wake_max_us\":%u,\"e2e_avg_us\":%u,\"e2e_max_us\":%u}\n",
				       CONFIG_BOARD, loop_cycles,
				       wake_sum / 100, wake_max,
				       e2e_n ? e2e_sum / e2e_n : 0, e2e_max);
			}
			wake_min = UINT32_MAX;
			wake_max = 0;
			wake_sum = 0;
			e2e_min = UINT32_MAX;
			e2e_max = 0;
			e2e_sum = 0;
			e2e_n = 0;
		}
	}
}
K_THREAD_DEFINE(sensor_tid, 2048, sensor_thread, NULL, NULL, NULL, 4, 0, 100);

void control_start(void)
{
	/* threads are statically defined; nothing to do (hook for ports) */
}
