# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(robotics_control_loop)

target_sources(app PRIVATE
  src/main.c
  src/control.c
  src/plant.c
)

if(CONFIG_APP_TRANSPORT_SERIAL OR CONFIG_APP_TRANSPORT_UDP)
  target_sources(app PRIVATE src/bridge.c src/framing.c)
endif()
target_sources_ifdef(CONFIG_APP_TRANSPORT_SERIAL app PRIVATE src/bridge_serial.c)
target_sources_ifdef(CONFIG_APP_TRANSPORT_UDP app PRIVATE src/bridge_udp.c)
