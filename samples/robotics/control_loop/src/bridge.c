/*
 * Transport bridge: the loop's only connection to the outside world, and
 * itself just another zbus observer. Outbound, sensor and actuator
 * publications are framed and handed to the backend; inbound, decoded
 * setpoint and e-stop frames are published locally, so remote and local
 * producers are indistinguishable to the loop.
 *
 * The backend below this file is two functions (init + write, plus
 * calling bridge_rx() with received bytes) - the seam a TSN, Zenoh or
 * any other transport implementation drops into.
 *
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/sys/printk.h>

#include "app.h"
#include "framing.h"

static struct frame_decoder decoder;
static K_MUTEX_DEFINE(tx_lock);

/* the thread republishing an inbound frame must not re-export it */
static k_tid_t rx_pub_tid;

static void bridge_tx(uint16_t id, const void *msg, size_t len)
{
	uint8_t buf[FRAME_MAX_PAYLOAD + FRAME_OVERHEAD];

	k_mutex_lock(&tx_lock, K_FOREVER);

	size_t n = frame_encode(id, msg, len, buf, sizeof(buf));

	if (n > 0) {
		(void)bridge_backend.write(buf, n);
	}
	k_mutex_unlock(&tx_lock);
}

static void export_cb(const struct zbus_channel *chan)
{
	if (k_current_get() == rx_pub_tid) {
		return;
	}
	if (chan == &sensor_chan) {
		bridge_tx(WIRE_ID_SENSOR, zbus_chan_const_msg(chan),
			  sizeof(struct sensor_msg));
	} else if (chan == &actuator_chan) {
		bridge_tx(WIRE_ID_ACTUATOR, zbus_chan_const_msg(chan),
			  sizeof(struct actuator_msg));
	}
}
ZBUS_LISTENER_DEFINE(bridge_listener, export_cb);
ZBUS_CHAN_ADD_OBS(sensor_chan, bridge_listener, 5);
ZBUS_CHAN_ADD_OBS(actuator_chan, bridge_listener, 5);

static void frame_rx_cb(uint16_t id, const void *payload, size_t len,
			void *user)
{
	ARG_UNUSED(user);
	const struct zbus_channel *chan = NULL;

	if (id == WIRE_ID_SETPOINT && len == sizeof(struct setpoint_msg)) {
		chan = &setpoint_chan;
	} else if (id == WIRE_ID_ESTOP && len == sizeof(struct estop_msg)) {
		chan = &estop_chan;
	}
	if (chan == NULL) {
		return;
	}
	rx_pub_tid = k_current_get();
	(void)zbus_chan_pub(chan, payload, K_MSEC(5));
	rx_pub_tid = NULL;
}

void bridge_rx(const uint8_t *buf, size_t len)
{
	frame_decoder_feed(&decoder, buf, len);
}

static int bridge_init(void)
{
	frame_decoder_init(&decoder, frame_rx_cb, NULL);

	int rc = bridge_backend.init();

	if (rc != 0) {
		printk("bridge: backend init failed (%d)\n", rc);
	}
	return rc;
}
SYS_INIT(bridge_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
