# Copyright (c) 2026 Ambiq Micro Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(renode_mve_smoke)

target_sources(app PRIVATE src/main.c)

# With SMOKE_TREE_NOMVE=y the image is built with MVE disabled (+nomve
# keeps FPv5) except this test's own source, which is restored to full MVE
# by a source-level -mcpu override (source COMPILE_OPTIONS land last on the
# command line and GCC honors the final -mcpu). This isolates the MVE
# instructions under test to main.c: Renode 1.16.1 faults on the MVE scalar
# 64-bit shifts (LSLL, encoding 0xEA50118F) that the compiler emits in the
# Ambiq HAL boot path (am_hal_uart.c config_baudrate 64-bit baud math) when
# the whole tree is compiled with MVE - see BLOCKERS.md.
if(SMOKE_TREE_NOMVE)
  zephyr_compile_options(-mcpu=cortex-m55+nomve)
  set_source_files_properties(src/main.c
    PROPERTIES COMPILE_OPTIONS "-mcpu=cortex-m55")
endif()
