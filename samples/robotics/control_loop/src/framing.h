/*
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SAMPLES_ROBOTICS_CONTROL_LOOP_FRAMING_H
#define SAMPLES_ROBOTICS_CONTROL_LOOP_FRAMING_H

#include <stddef.h>
#include <stdint.h>

#define FRAME_SYNC        0x7EU
#define FRAME_MAX_PAYLOAD 64U
#define FRAME_OVERHEAD    7U /* sync + id(2) + len(2) + crc(2) */

typedef void (*frame_cb_t)(uint16_t id, const void *payload, size_t len,
			   void *user);

enum frame_state {
	FRAME_ST_SYNC,
	FRAME_ST_ID_LO,
	FRAME_ST_ID_HI,
	FRAME_ST_LEN_LO,
	FRAME_ST_LEN_HI,
	FRAME_ST_PAYLOAD,
	FRAME_ST_CRC_LO,
	FRAME_ST_CRC_HI,
};

struct frame_decoder {
	enum frame_state state;
	uint16_t id;
	uint16_t len;
	uint16_t pos;
	uint16_t crc;
	uint16_t crc_rx;
	uint8_t payload[FRAME_MAX_PAYLOAD];
	frame_cb_t cb;
	void *user;
};

size_t frame_encode(uint16_t id, const void *payload, size_t len,
		    uint8_t *out, size_t out_size);
void frame_decoder_init(struct frame_decoder *d, frame_cb_t cb, void *user);
void frame_decoder_feed(struct frame_decoder *d, const uint8_t *buf,
			size_t len);

#endif /* SAMPLES_ROBOTICS_CONTROL_LOOP_FRAMING_H */
