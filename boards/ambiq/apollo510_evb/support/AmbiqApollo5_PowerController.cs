//
// Copyright (c) 2026 Ambiq Micro Inc.
//
// SPDX-License-Identifier: Apache-2.0
//
// Minimal Apollo510 PWRCTRL model for running Zephyr under Renode.
//
// The pattern follows Renode's AmbiqApollo4_PowerController (MIT): power
// STATUS registers mirror their ENABLE counterparts, so HAL code that
// writes xPWREN and then spin-waits on xPWRSTATUS - in both directions,
// power-up (am_hal_pwrctrl_periph_enable) and power-down
// (am_hal_pwrctrl_periph_disable, exercised constantly by Zephyr's
// CONFIG_PM_DEVICE_RUNTIME) - always falls straight through.
//
// Register offsets and bit layouts come from CMSIS apollo510.h
// (PWRCTRL_Type, base 0x40021000):
//  * DEVPWREN 0x04 -> DEVPWRSTATUS 0x08: all 30 PWREN*/PWRST* bits are at
//    identical positions, so status is an identity mirror of enable.
//  * AUDSSPWREN 0x0C -> AUDSSPWRSTATUS 0x10: identity mirror.
//  * MEMPWREN 0x14 -> MEMPWRSTATUS 0x18: bits are NOT aligned -
//    PWRENTCM[2:0]->PWRSTTCM[2:0], PWRENNVM bit3->PWRSTNVM0 bit3,
//    PWRENNVM1 bit4->PWRSTNVM1 bit6, PWRENROM bit5->PWRSTROM bit7;
//    PWRSTCACHE bit4 has no enable counterpart and reads as always-on.
//  * SSRAMPWREN 0x24 -> SSRAMPWRST 0x28: identity mirror (3-bit field).
//  * MCUPERFREQ 0x00: the requested MCUPERFREQ[1:0] is acknowledged
//    immediately - MCUPERFACK (bit 2) reads 1 and MCUPERFSTATUS[4:3]
//    reflects the request, which satisfies the mode-switch wait loop in
//    am_hal_pwrctrl_mcu_mode_select().
//  * VRSTATUS 0x108 reads 0x3F: SIMOBUCKST/MEMLDOST/CORELDOST all in ACT
//    state (each enum: ACT = 3), as checked e.g. by am_hal_sysctrl_sleep().
//
// Every other register in the block behaves as read/write scratch
// (reads return the last written value, initially 0), which both lets
// unmodeled HAL state round-trip and allows a .resc script to pre-seed
// any specific status word with WriteDoubleWord.
//
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AmbiqApollo5_PowerController : IDoubleWordPeripheral, IKnownSize
    {
        public AmbiqApollo5_PowerController(IMachine machine)
        {
            Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            uint value;
            switch(offset)
            {
            case McuPerformanceRequest:
                var request = Stored(McuPerformanceRequest) & 0x3;
                value = request | (1u << 2) | (request << 3);
                break;
            case DevicePowerStatus:
                value = Stored(DevicePowerEnable);
                break;
            case AudioSubsystemPowerStatus:
                value = Stored(AudioSubsystemPowerEnable);
                break;
            case MemoryPowerStatus:
                var memoryEnable = Stored(MemoryPowerEnable);
                value = (memoryEnable & 0x7)                    // PWRSTTCM[2:0]  <- PWRENTCM[2:0]
                    | (memoryEnable & (1u << 3))                // PWRSTNVM0 (3)  <- PWRENNVM (3)
                    | (1u << 4)                                 // PWRSTCACHE: no enable bit, always powered
                    | (((memoryEnable >> 4) & 1u) << 6)         // PWRSTNVM1 (6)  <- PWRENNVM1 (4)
                    | (((memoryEnable >> 5) & 1u) << 7);        // PWRSTROM  (7)  <- PWRENROM  (5)
                break;
            case SharedSramPowerStatus:
                value = Stored(SharedSramPowerEnable) & 0x7;
                break;
            case VoltageRegulatorsStatus:
                // CORELDOST=ACT | MEMLDOST=ACT | SIMOBUCKST=ACT
                value = 0x3F;
                break;
            default:
                value = Stored(offset);
                break;
            }
            this.Log(LogLevel.Noisy, "Read 0x{0:X} from offset 0x{1:X}", value, offset);
            return value;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            this.Log(LogLevel.Noisy, "Written 0x{0:X} to offset 0x{1:X}", value, offset);
            storage[offset] = value;
        }

        public void Reset()
        {
            storage.Clear();
        }

        public long Size => 0x250;

        private uint Stored(long offset)
        {
            return storage.TryGetValue(offset, out var stored) ? stored : 0;
        }

        private readonly Dictionary<long, uint> storage = new Dictionary<long, uint>();

        private const long McuPerformanceRequest = 0x00;
        private const long DevicePowerEnable = 0x04;
        private const long DevicePowerStatus = 0x08;
        private const long AudioSubsystemPowerEnable = 0x0C;
        private const long AudioSubsystemPowerStatus = 0x10;
        private const long MemoryPowerEnable = 0x14;
        private const long MemoryPowerStatus = 0x18;
        private const long SharedSramPowerEnable = 0x24;
        private const long SharedSramPowerStatus = 0x28;
        private const long VoltageRegulatorsStatus = 0x108;
    }
}
