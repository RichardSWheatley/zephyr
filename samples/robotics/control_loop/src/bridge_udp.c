/*
 * UDP bridge backend: whole frames inside datagrams. On native_sim,
 * combine with NSOS offloaded sockets (overlay-udp.conf) for zero host
 * setup.
 *
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/net/socket.h>

#include "app.h"

static int sock = -1;
static struct sockaddr_in peer;

/*
 * Auto-started and gated on a semaphore: static threads are set up after
 * APPLICATION-level SYS_INIT, so starting the thread from init would be
 * silently undone.
 */
static K_SEM_DEFINE(udp_ready, 0, 1);

static void rx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	static uint8_t buf[256];

	k_sem_take(&udp_ready, K_FOREVER);

	while (1) {
		ssize_t n = zsock_recvfrom(sock, buf, sizeof(buf), 0, NULL,
					   NULL);

		if (n <= 0) {
			k_sleep(K_MSEC(10));
			continue;
		}
		bridge_rx(buf, (size_t)n);
	}
}
K_THREAD_DEFINE(bridge_udp_rx, 2048, rx_thread, NULL, NULL, NULL, 7, 0, 0);

static int udp_write(const uint8_t *buf, size_t len)
{
	if (sock < 0) {
		return -ENOTCONN;
	}
	ssize_t n = zsock_sendto(sock, buf, len, 0, (struct sockaddr *)&peer,
				 sizeof(peer));

	return (n == (ssize_t)len) ? 0 : -EIO;
}

static int udp_init(void)
{
	struct sockaddr_in local = {
		.sin_family = AF_INET,
		.sin_port = htons(CONFIG_APP_UDP_LOCAL_PORT),
		.sin_addr = { .s_addr = htonl(INADDR_ANY) },
	};

	sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		return -errno;
	}
	if (zsock_bind(sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
		return -errno;
	}

	peer.sin_family = AF_INET;
	peer.sin_port = htons(CONFIG_APP_UDP_PEER_PORT);
	if (zsock_inet_pton(AF_INET, CONFIG_APP_UDP_PEER_ADDR,
			    &peer.sin_addr) != 1) {
		return -EINVAL;
	}

	k_sem_give(&udp_ready);
	return 0;
}

const struct bridge_backend bridge_backend = {
	.init = udp_init,
	.write = udp_write,
};
