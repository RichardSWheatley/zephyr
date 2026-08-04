# Copyright (c) 2026 Ambiq Micro Inc.
# SPDX-License-Identifier: Apache-2.0

*** Settings ***
Resource                      ${KEYWORDS}

*** Test Cases ***
STIMER Compare And Capture Contract
    Prepare Machine
    Wait For Line On Uart     COMPCAP compareB: PASS

    Wait For Line On Uart     READY CAP0
    Execute Command           sysbus.stimer OnGPIO 200 true
    Wait For Line On Uart     READY CAP0-DUP
    Execute Command           sysbus.stimer OnGPIO 200 true
    Execute Command           sysbus WriteDoubleWord 0x40008850 1
    Wait For Line On Uart     COMPCAP capture0: PASS

    Wait For Line On Uart     READY CAP1
    Execute Command           sysbus.stimer OnGPIO 100 true
    Execute Command           sysbus WriteDoubleWord 0x40008850 2
    Wait For Line On Uart     READY CAP1-FALL
    Execute Command           sysbus.stimer OnGPIO 100 false
    Wait For Line On Uart     COMPCAP capture1: PASS

    Wait For Line On Uart     COMPCAP capstop: PASS

    Wait For Line On Uart     READY JUMP
    ${target}=                Execute Command    sysbus ReadDoubleWord 0x40008854
    ${jump}=                  Evaluate           hex(int("""${target}""".strip(), 16) + 64)
    Execute Command           sysbus.stimer CounterValue ${jump}
    Execute Command           sysbus WriteDoubleWord 0x40008850 3
    Wait For Line On Uart     COMPCAP jump: PASS

    Wait For Line On Uart     READY OVF
    Execute Command           sysbus.stimer CounterValue 0xFFFFFF00
    Wait For Line On Uart     COMPCAP overflow: PASS

    Wait For Line On Uart     COMPCAP: ALL PASS
