/*
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SAMPLES_ROBOTICS_CONTROL_LOOP_APP_H
#define SAMPLES_ROBOTICS_CONTROL_LOOP_APP_H

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

/*
 * Plain-data message vocabulary. Every message starts with the same
 * header so generic tooling (and the wire bridge) can treat them
 * uniformly. stamp_us always carries *acquisition* time: derived messages
 * propagate the stamp of the sensor sample that caused them, which is
 * what makes end-to-end latency measurable at the actuation point.
 */
struct msg_header {
	uint64_t stamp_us; /* acquisition timestamp, microseconds */
	uint32_t seq;      /* per-producer sequence number        */
	uint16_t type;     /* wire id, see WIRE_ID_*              */
	uint16_t source;   /* producer id (0 = unspecified)       */
};

#define WIRE_ID_SETPOINT 1U
#define WIRE_ID_SENSOR   2U
#define WIRE_ID_ACTUATOR 3U
#define WIRE_ID_ESTOP    4U

struct setpoint_msg {
	struct msg_header header;
	float velocity; /* desired plant velocity */
};

struct sensor_msg {
	struct msg_header header;
	float velocity; /* measured plant velocity */
};

struct actuator_msg {
	struct msg_header header;
	float command; /* normalized actuator command, -1..1 */
};

struct estop_msg {
	struct msg_header header;
	uint8_t reason;
};

ZBUS_CHAN_DECLARE(setpoint_chan);
ZBUS_CHAN_DECLARE(sensor_chan);
ZBUS_CHAN_DECLARE(actuator_chan);
ZBUS_CHAN_DECLARE(estop_chan);

static inline uint64_t now_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

/* control loop parameters */
#define CONTROL_RATE_HZ     100U
#define CONTROL_PERIOD_TICKS                                                   \
	MAX(1, CONFIG_SYS_CLOCK_TICKS_PER_SEC / CONTROL_RATE_HZ)
#define CONTROL_CYCLES      300U /* run length before the e-stop demo */

/* plant simulation (plant.c) */
void plant_apply(float command);
float plant_step(float dt_s);

/* control.c */
void control_start(void);
extern volatile uint32_t loop_cycles;

/* wire bridge (bridge.c, compiled unless APP_TRANSPORT_NONE) */
#if defined(CONFIG_APP_TRANSPORT_SERIAL) || defined(CONFIG_APP_TRANSPORT_UDP)
void bridge_rx(const uint8_t *buf, size_t len);
struct bridge_backend {
	int (*init)(void);
	int (*write)(const uint8_t *buf, size_t len);
};
extern const struct bridge_backend bridge_backend;
#endif

#endif /* SAMPLES_ROBOTICS_CONTROL_LOOP_APP_H */
