/*
 * Copyright (c) 2026 Ambiq Micro Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MVE (Helium) smoke test for simulator validation.
 *
 * Exercises the instruction areas that matter for running MVE workloads on
 * emulated Cortex-M55 platforms (Renode apollo510_evb, QEMU mps3-an547):
 *  1. contiguous vector load/add/store (VLDR/VADD/VSTR, int32x4)
 *  2. vector float multiply-accumulate (VFMA, float32x4 - MVEF/FPv5)
 *  3. a VCTP tail-predicated loop over a non-multiple-of-4 element count
 *     (predication is the area flagged as immature in emulators)
 *  4. an explicit low-overhead-branch loop (DLS/LE - "rudimentary" in
 *     Renode 1.16.1 per the Armv8.1-M announcement)
 *
 * Each stage asserts exact results; the final line is either
 * "MVE smoke: PASS" or "MVE smoke: FAIL (<stage>)".
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <arm_mve.h>

#define TAIL_LEN 13

/*
 * GCC's arm_mve.h intrinsics are generated against the compiler-native
 * 32-bit type ('long' on arm-eabi); Zephyr's zephyr_stdint.h remaps
 * int32_t to 'int', which makes int32_t pointers incompatible with the
 * intrinsic prototypes. Use the intrinsic-native lane type for every
 * buffer accessed through MVE loads/stores.
 */
typedef long mve_s32_t;

/* Defeats constant-folding: forces the vector code to actually load from and
 * store to memory at runtime instead of being computed at compile time.
 * (Named to avoid Zephyr's own compiler_barrier() macro.)
 */
static inline void opt_barrier(void)
{
	__asm volatile("" ::: "memory");
}

static mve_s32_t src_a[16];
static mve_s32_t src_b[16];
static mve_s32_t dst_i[16];

static float src_f[8];
static float dst_f[8];

static mve_s32_t tail_src[TAIL_LEN];
static mve_s32_t tail_dst[TAIL_LEN + 3]; /* +3 to detect predicated-lane overwrite */

static int test_vector_int(void)
{
	for (int i = 0; i < 16; i++) {
		src_a[i] = i + 1;
		src_b[i] = 100 * (i + 1);
		dst_i[i] = 0;
	}

	opt_barrier();

	for (int i = 0; i < 16; i += 4) {
		int32x4_t va = vld1q_s32(&src_a[i]);
		int32x4_t vb = vld1q_s32(&src_b[i]);
		int32x4_t vr = vaddq_s32(va, vb);

		vst1q_s32(&dst_i[i], vr);
	}

	opt_barrier();

	for (int i = 0; i < 16; i++) {
		if (dst_i[i] != 101 * (i + 1)) {
			printk("int lane %d: got %ld want %d\n", i, dst_i[i], 101 * (i + 1));
			return -1;
		}
	}
	return 0;
}

static int test_vector_float(void)
{
	for (int i = 0; i < 8; i++) {
		src_f[i] = (float)(i + 1);
		dst_f[i] = 1.0f;
	}

	opt_barrier();

	for (int i = 0; i < 8; i += 4) {
		float32x4_t acc = vld1q_f32(&dst_f[i]);
		float32x4_t va = vld1q_f32(&src_f[i]);

		/* acc += va * 2.0f */
		acc = vfmaq_n_f32(acc, va, 2.0f);
		vst1q_f32(&dst_f[i], acc);
	}

	opt_barrier();

	for (int i = 0; i < 8; i++) {
		float want = 1.0f + 2.0f * (float)(i + 1);

		if (dst_f[i] != want) {
			printk("float lane %d: got %d.%03d want %d.%03d\n", i,
			       (int)dst_f[i], (int)(dst_f[i] * 1000) % 1000,
			       (int)want, (int)(want * 1000) % 1000);
			return -1;
		}
	}
	return 0;
}

static int test_vctp_tail_loop(void)
{
	const mve_s32_t canary = 0x5A5A5A5A;

	for (int i = 0; i < TAIL_LEN; i++) {
		tail_src[i] = 7 * (i + 1);
	}
	for (int i = 0; i < TAIL_LEN + 3; i++) {
		tail_dst[i] = canary;
	}

	opt_barrier();

	/*
	 * Classic Helium tail-predicated pattern: full 4-lane iterations,
	 * then a VCTP-masked tail so the last (13 % 4 = 1) element is
	 * processed without touching the lanes beyond the buffer.
	 */
	int32_t n = TAIL_LEN;
	const mve_s32_t *in = tail_src;
	mve_s32_t *out = tail_dst;

	while (n > 0) {
		mve_pred16_t p = vctp32q((uint32_t)n);
		/* vldrwq/vstrwq are declared with the header-level int32_t (which
		 * Zephyr remaps to 'int'), unlike the pragma-generated vld1q above -
		 * hence the casts.
		 */
		int32x4_t v = vldrwq_z_s32((const int32_t *)in, p);

		v = vaddq_x_s32(v, vdupq_n_s32(3), p);
		vstrwq_p_s32((int32_t *)out, v, p);

		in += 4;
		out += 4;
		n -= 4;
	}

	opt_barrier();

	for (int i = 0; i < TAIL_LEN; i++) {
		if (tail_dst[i] != 7 * (i + 1) + 3) {
			printk("tail lane %d: got %ld want %d\n", i, tail_dst[i],
			       7 * (i + 1) + 3);
			return -1;
		}
	}
	/* Predicated-off lanes of the final iteration must not have stored. */
	for (int i = TAIL_LEN; i < TAIL_LEN + 3; i++) {
		if (tail_dst[i] != canary) {
			printk("tail overwrite at %d: got 0x%08lx\n", i, tail_dst[i]);
			return -1;
		}
	}
	return 0;
}

static int test_low_overhead_branch(void)
{
	uint32_t iterations = 37;
	uint32_t acc = 0;

	/*
	 * Explicit DLS/LE loop - guarantees the low-overhead-branch
	 * instructions are executed no matter how the compiler chooses to
	 * lower C loops.
	 */
	__asm volatile(
		"dls lr, %[cnt]\n"
		"1:\n\t"
		"adds %[acc], %[acc], #1\n\t"
		"le lr, 1b\n"
		: [acc] "+r"(acc)
		: [cnt] "r"(iterations)
		: "lr", "cc", "memory");

	if (acc != iterations) {
		printk("LOB loop: got %u want %u\n", acc, iterations);
		return -1;
	}
	return 0;
}

int main(void)
{
	int rc;

	printk("MVE smoke on %s\n", CONFIG_BOARD_TARGET);

	rc = test_vector_int();
	if (rc != 0) {
		printk("MVE smoke: FAIL (vector-int)\n");
		return 0;
	}

	rc = test_vector_float();
	if (rc != 0) {
		printk("MVE smoke: FAIL (vector-float)\n");
		return 0;
	}

	rc = test_vctp_tail_loop();
	if (rc != 0) {
		printk("MVE smoke: FAIL (vctp-tail)\n");
		return 0;
	}

	rc = test_low_overhead_branch();
	if (rc != 0) {
		printk("MVE smoke: FAIL (low-overhead-branch)\n");
		return 0;
	}

	printk("MVE smoke: PASS\n");
	return 0;
}
