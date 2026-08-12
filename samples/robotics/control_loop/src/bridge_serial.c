/*
 * Serial bridge backend: interrupt-driven RX into a ring buffer decoded
 * on the system workqueue; polled TX. The UART comes from the
 * "bridge-uart" devicetree alias.
 *
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include "app.h"

static const struct device *const uart_dev =
	DEVICE_DT_GET(DT_ALIAS(bridge_uart));

RING_BUF_DECLARE(rx_ring, 512);
static struct k_work rx_work;

static int serial_write(const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
	return 0;
}

static void rx_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	uint8_t buf[64];
	uint32_t n;

	while ((n = ring_buf_get(&rx_ring, buf, sizeof(buf))) > 0) {
		bridge_rx(buf, n);
	}
}

static void uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t buf[32];

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev) > 0) {
		int n = uart_fifo_read(dev, buf, sizeof(buf));

		if (n <= 0) {
			break;
		}
		(void)ring_buf_put(&rx_ring, buf, n);
	}
	k_work_submit(&rx_work);
}

static int serial_init(void)
{
	if (!device_is_ready(uart_dev)) {
		return -ENODEV;
	}
	k_work_init(&rx_work, rx_work_fn);
	uart_irq_callback_user_data_set(uart_dev, uart_isr, NULL);
	uart_irq_rx_enable(uart_dev);
	return 0;
}

const struct bridge_backend bridge_backend = {
	.init = serial_init,
	.write = serial_write,
};
