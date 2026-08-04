# Copyright (c) 2025 Ambiq Micro Inc.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=AP510NFA-CBR" "--iface=swd" "--speed=1000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

set(SUPPORTED_EMU_PLATFORMS renode)
set(RENODE_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/support/apollo510_evb.resc)
set(RENODE_UART sysbus.uart0)

set_ifndef(BOARD_SIM_RUNNER renode)
set_ifndef(BOARD_ROBOT_RUNNER renode-robot)
include(${ZEPHYR_BASE}/boards/common/renode.board.cmake)
include(${ZEPHYR_BASE}/boards/common/renode_robot.board.cmake)
