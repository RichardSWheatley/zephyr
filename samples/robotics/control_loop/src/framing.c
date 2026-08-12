/*
 * Byte framing for the transport bridge:
 *
 *   0x7E | id(u16 LE) | len(u16 LE) | payload[len] | crc16(u16 LE)
 *
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over id+len+payload.
 * Deliberately interoperable with vendor implementations of the same
 * framing; the decoder is a plain byte-stream state machine, so any
 * transport that delivers the bytes in order can carry it.
 *
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include "framing.h"

static uint16_t crc16_step(uint16_t crc, uint8_t byte)
{
	crc ^= (uint16_t)byte << 8;
	for (int i = 0; i < 8; i++) {
		crc = (crc & 0x8000U) ? (crc << 1) ^ 0x1021U : (crc << 1);
	}
	return crc;
}

static uint16_t crc16_buf(uint16_t crc, const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		crc = crc16_step(crc, buf[i]);
	}
	return crc;
}

size_t frame_encode(uint16_t id, const void *payload, size_t len,
		    uint8_t *out, size_t out_size)
{
	if (len > FRAME_MAX_PAYLOAD || out_size < len + FRAME_OVERHEAD) {
		return 0;
	}

	uint8_t hdr[4] = { (uint8_t)(id & 0xFF), (uint8_t)(id >> 8),
			   (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
	uint16_t crc = 0xFFFF;

	crc = crc16_buf(crc, hdr, sizeof(hdr));
	crc = crc16_buf(crc, payload, len);

	out[0] = FRAME_SYNC;
	memcpy(&out[1], hdr, sizeof(hdr));
	memcpy(&out[5], payload, len);
	out[5 + len] = (uint8_t)(crc & 0xFF);
	out[6 + len] = (uint8_t)(crc >> 8);
	return len + FRAME_OVERHEAD;
}

void frame_decoder_init(struct frame_decoder *d, frame_cb_t cb, void *user)
{
	memset(d, 0, sizeof(*d));
	d->cb = cb;
	d->user = user;
}

void frame_decoder_feed(struct frame_decoder *d, const uint8_t *buf,
			size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uint8_t b = buf[i];

		switch (d->state) {
		case FRAME_ST_SYNC:
			if (b == FRAME_SYNC) {
				d->state = FRAME_ST_ID_LO;
				d->crc = 0xFFFF;
			}
			break;
		case FRAME_ST_ID_LO:
			d->id = b;
			d->crc = crc16_step(d->crc, b);
			d->state = FRAME_ST_ID_HI;
			break;
		case FRAME_ST_ID_HI:
			d->id |= (uint16_t)b << 8;
			d->crc = crc16_step(d->crc, b);
			d->state = FRAME_ST_LEN_LO;
			break;
		case FRAME_ST_LEN_LO:
			d->len = b;
			d->crc = crc16_step(d->crc, b);
			d->state = FRAME_ST_LEN_HI;
			break;
		case FRAME_ST_LEN_HI:
			d->len |= (uint16_t)b << 8;
			d->crc = crc16_step(d->crc, b);
			if (d->len > FRAME_MAX_PAYLOAD) {
				d->state = FRAME_ST_SYNC; /* resync */
				break;
			}
			d->pos = 0;
			d->state = (d->len > 0) ? FRAME_ST_PAYLOAD
						: FRAME_ST_CRC_LO;
			break;
		case FRAME_ST_PAYLOAD:
			d->payload[d->pos++] = b;
			d->crc = crc16_step(d->crc, b);
			if (d->pos == d->len) {
				d->state = FRAME_ST_CRC_LO;
			}
			break;
		case FRAME_ST_CRC_LO:
			d->crc_rx = b;
			d->state = FRAME_ST_CRC_HI;
			break;
		case FRAME_ST_CRC_HI:
			d->crc_rx |= (uint16_t)b << 8;
			if (d->crc_rx == d->crc && d->cb != NULL) {
				d->cb(d->id, d->payload, d->len, d->user);
			}
			d->state = FRAME_ST_SYNC;
			break;
		default:
			d->state = FRAME_ST_SYNC;
			break;
		}
	}
}
