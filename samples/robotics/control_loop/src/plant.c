/*
 * Simulated plant: a first-order DC-motor-like system,
 *
 *   dv/dt = (K * u - v) / tau
 *
 * so the sample closes a real loop on every board, hardware or not. On a
 * real robot this file is replaced by the motor driver and encoder
 * feedback; nothing else in the sample changes.
 *
 * Copyright (c) 2026 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "app.h"

#define PLANT_GAIN 2.0f  /* steady-state velocity per unit command */
#define PLANT_TAU  0.15f /* time constant, seconds                 */

static float velocity;
static volatile float command;

void plant_apply(float cmd)
{
	command = cmd;
}

float plant_step(float dt_s)
{
	velocity += ((PLANT_GAIN * command) - velocity) * (dt_s / PLANT_TAU);
	return velocity;
}
