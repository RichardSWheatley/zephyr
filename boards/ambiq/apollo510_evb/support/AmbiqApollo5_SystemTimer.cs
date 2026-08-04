//
// Copyright (c) 2010-2026 Antmicro
// Copyright (c) 2026 Ambiq Micro Inc.
//
// This file is licensed under the MIT License.
// Full license text is available in Renode's 'licenses/MIT.txt'.
//
// Apollo510 STIMER model, originally adapted from Renode's
// AmbiqApollo4_SystemTimer (renode-infrastructure, MIT). The Apollo510
// STIMER register map is identical to the Apollo4 one at every offset this
// model implements (STCFG 0x00, STTMR 0x04, SCAPCTRLn 0x10-0x1C, SCMPRn
// 0x20-0x3C, SCAPTn 0x40-0x4C, SNVR0/1 0x50-0x54, STMINTEN/STAT/CLR/SET
// 0x100-0x10C; CMSIS apollo510.h STIMER_Type). Divergences from the stock
// Apollo4 model, each tracked to the Apollo510 sources:
//
//  * Counter modulus is 2^32: STTMR counts 0..0xFFFFFFFF and overflows
//    "from 0xFFFFFFFF back to 0x00000000" (STMINT OVERFLOW description).
//    The stock model used limit uint.MaxValue = 2^32-1, which made
//    0xFFFFFFFF unobservable and lost one tick per wrap - exactly
//    cancelling an off-by-one in software that extends the wrapped
//    counter by 0xFFFFFFFF instead of 2^32.
//  * A COMPARE target the counter has already met or passed raises the
//    interrupt status instead of stalling until the next 2^32-tick lap:
//    the STMINT COMPAREx bits are documented as "COUNTER is greater than
//    or equal to COMPARE register x". This matters when the counter is
//    jumped over a pending target (CounterValue preload) or a comparator
//    is enabled against a stale target; SCMPR writes themselves always
//    produce a target ahead of the counter (write = unsigned delta).
//  * The OVERFLOW interrupt status is one backing flag manipulated by
//    STMINTEN/STAT/CLR/SET callbacks. (The stock model re-bound one field
//    variable in three register definitions, leaving STMINTCLR unable to
//    clear the status - IRQ 40 latched forever after a wrap - and
//    STMINTSET.OVERFLOW a no-op.)
//  * SCAPCTRL.STSEL is 8 bits wide with reset 0xFF (Apollo510 has 224
//    GPIO pads; Apollo4 had 7 bits/0x7F).
//  * Selecting NOCLK (or an unsupported CTIMER source) stops the counter
//    even after a valid clock ran it (the stock model kept the previous
//    frequency and the counter free-ran).
//  * The capture path is functional: a GPIO edge on the STSEL-selected
//    pin (identity mapping to pad number, polarity per STPOL: 0 = low to
//    high, 1 = high to low) latches SCAPTn <= STTMR and the CAPTUREx
//    status atomically, gated on TIMER->GLOBEN.ENABLEALLINPUTS (forwarded
//    into GlobalInputsEnabled by AmbiqApollo5_TimerStub below, matching
//    am_hal_stimer_capture_start()/_stop()). Edges are tracked per pin so
//    repeated same-level stimuli do not re-capture. SCAPTn accepts writes
//    (am_hal_stimer_reset_config() zeroes them) and STMINTSTAT accepts
//    direct stores (reset_config writes 0 to it; CMSIS declares it __IOM).
//  * XTAL-derived CLKSEL frequencies are the real crystal-divided values
//    (32768/16384/1024 Hz), matching CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC.
//  * CounterValue is settable from the monitor (e.g.
//    "sysbus.stimer CounterValue 0xFFFAC000") so tests can preload the
//    counter near wrap-around; comparator views are resynced and targets
//    the jump passes over fire immediately, as they would had the time
//    actually elapsed.
//
// Compare semantics (verified against the Apollo510 sources): a write to
// SCMPRn is a DELTA relative to the current COUNTER value - the hardware
// adds it to COUNTER when the write takes effect and the comparator fires
// when COUNTER reaches that absolute target; reading SCMPRn returns the
// absolute target. See hal_ambiq mcu/apollo510/hal/am_hal_stimer.c,
// am_hal_stimer_compare_delta_set() (write of the adjusted delta to
// AM_REG_STIMER_COMPARE, with the delta reduced by 3 because "It takes 3
// STIMER clock cycles for writes to COMPARE to be effective. Also the
// interrupt itself is delayed by a cycle") and the SCMPR0 register
// description in CMSIS apollo510.h ("write the number of ticks in the
// future ... The hardware does the addition to the COUNTER value in the
// STIMER clock domain").
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Timers
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AmbiqApollo5_SystemTimer : BasicDoubleWordPeripheral, IKnownSize, IGPIOReceiver
    {
        public AmbiqApollo5_SystemTimer(IMachine machine) : base(machine)
        {
            // Changing 'CLKSEL' (it's 'NOCLK' by default) is necessary to enable 'systemTimer'.
            systemTimer = new LimitTimer(machine.ClockSource, InvalidFrequency, this, "System Timer", CounterSpan,
                Direction.Ascending, enabled: false, workMode: WorkMode.Periodic, eventEnabled: true, autoUpdate: true, divider: 1);
            systemTimer.LimitReached += () => HandleLimitReached();

            for(var i = 0; i < interruptOutputs.Length; i++)
            {
                interruptOutputs[i] = new GPIO();
            }
            for(var i = 0; i < captureRegisters.Length; i++)
            {
                captureRegisters[i] = new CaptureRegister(this, i);
            }
            for(var i = 0; i < compareRegisters.Length; i++)
            {
                compareRegisters[i] = new CompareRegister(this, i, interruptOutputs[i], systemTimer);
            }

            DefineRegisters();
            Reset();
        }

        public void OnGPIO(int number, bool value)
        {
            // Track pin levels so only genuine transitions reach the capture
            // logic - monitor-driven or scripted senders may repeat levels.
            var previous = gpioStates.TryGetValue(number, out var state) && state;
            if(value == previous)
            {
                return;
            }
            gpioStates[number] = value;
            this.Log(LogLevel.Debug, "GPIO#{0} {1}", number, value ? "set" : "unset");

            // am_hal_stimer_capture_start() arms SCAPCTRLn and then sets
            // TIMER->GLOBEN.ENABLEALLINPUTS; captures only occur while that
            // cross-peripheral gate is open.
            if(!GlobalInputsEnabled)
            {
                return;
            }
            foreach(var captureRegister in captureRegisters)
            {
                captureRegister.OnGPIOEdge(number, value);
            }
        }

        public override void Reset()
        {
            Array.ForEach(captureRegisters, register => register.Reset());
            Array.ForEach(compareRegisters, register => register.Reset());
            // All but IRQI should be reset with CompareRegister.Reset; nevertheless, let's unset all IRQs.
            Array.ForEach(interruptOutputs, irq => irq.Unset());
            systemTimer.Reset();
            overflowStatus = false;
            gpioStates.Clear();
            GlobalInputsEnabled = false;

            base.Reset();
        }

        public ulong Frequency => systemTimer.Frequency;

        // TIMER->GLOBEN.ENABLEALLINPUTS (bit 29 of the TIMER block at
        // 0x40008008) - forwarded here by AmbiqApollo5_TimerStub.
        public bool GlobalInputsEnabled { get; set; }

        // Comparator IRQs
        public GPIO IRQA => interruptOutputs[0];

        public GPIO IRQB => interruptOutputs[1];

        public GPIO IRQC => interruptOutputs[2];

        public GPIO IRQD => interruptOutputs[3];

        public GPIO IRQE => interruptOutputs[4];

        public GPIO IRQF => interruptOutputs[5];

        public GPIO IRQG => interruptOutputs[6];

        public GPIO IRQH => interruptOutputs[7];

        // Capture + overflow event IRQ
        public GPIO IRQI => interruptOutputs[8];

        public long Size => 0x110;

        public uint Value
        {
            get
            {
                if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
                {
                    // being here means we are on the CPU thread
                    cpu.SyncTime();
                }
                return (uint)systemTimer.Value;
            }
        }

        // Monitor-settable counter preload used to test counter wrap-around
        // handling in guest software. Comparator views are kept coherent:
        // every enabled comparator whose absolute target lies inside the
        // jumped-over interval fires immediately, exactly as it would had
        // the counter really advanced through that range.
        public uint CounterValue
        {
            get => Value;
            set
            {
                if(clear.Value)
                {
                    this.Log(LogLevel.Warning, "Ignoring COUNTER preload of 0x{0:X}: STCFG.CLEAR holds the counter in reset", value);
                    return;
                }
                var previous = Value;
                systemTimer.Value = value;
                Array.ForEach(compareRegisters, register => register.HandleCounterJump(previous, value));
                this.Log(LogLevel.Info, "COUNTER preloaded with 0x{0:X} (was 0x{1:X})", value, previous);
            }
        }

        private void DefineRegisters()
        {
            Registers.Capture0.DefineMany(this, 4,
                (register, registerIndex) =>
                {
                    // Writable: am_hal_stimer_reset_config() zeroes SCAPT0-3.
                    register.WithValueField(0, 32, name: $"SCAPT{registerIndex}",
                        valueProviderCallback: _ => captureRegisters[registerIndex].ValueCaptured,
                        writeCallback: (_, newValue) => captureRegisters[registerIndex].ValueCaptured = (uint)newValue);
                }, stepInBytes: 4);

            Registers.CaptureControl0.DefineMany(this, 4,
                (register, registerIndex) =>
                {
                    var captureRegister = captureRegisters[registerIndex];
                    // STSEL is 8 bits on Apollo510 (224 GPIO pads; CMSIS
                    // STIMER_SCAPCTRL0_STSEL0_Msk = 0xFF) - the Apollo4
                    // layout was 7 bits + reserved.
                    register.WithValueField(0, 8, out captureRegister.TriggerSourceGPIOPinNumber, name: $"STSEL{registerIndex}")
                        .WithFlag(8, out captureRegister.CaptureOnHighToLowGPIOTransition, name: $"STPOL{registerIndex}")
                        .WithFlag(9, out captureRegister.Enabled, name: $"CAPTURE{registerIndex}")
                        .WithReservedBits(10, 22)
                        ;
                }, stepInBytes: 4, resetValue: 0xFF);

            Registers.Compare0.DefineMany(this, 8,
                (register, registerIndex) =>
                {
                    register.WithValueField(0, 32, name: $"SCMPR{registerIndex}",
                        // SCMPR value written is relative to the current COUNTER (systemTimer's Value).
                        writeCallback: (_, newValue) =>
                        {
                            // On hardware a COMPARE write takes 3 STIMER clock cycles to become
                            // effective and the interrupt is delayed by a further cycle;
                            // am_hal_stimer_compare_delta_set() compensates by subtracting 3 from
                            // the requested delta (am_hal_stimer.c: "It takes 3 STIMER clock
                            // cycles for writes to COMPARE to be effective. Also the interrupt
                            // itself is delayed by a cycle"). Model that latency by pushing the
                            // absolute target 3 cycles past the current counter. (The upstream
                            // Apollo4 model instead did systemTimer.Increment(3), which advances
                            // the base counter but not the per-comparator timers - the two clocks
                            // then drift 3 cycles apart on every write, which accumulates into
                            // visibly-late timeouts in re-arm-heavy workloads such as
                            // tests/kernel/timer/timeout_churn.)
                            compareRegisters[registerIndex].CompareValue = Value + (uint)newValue + 3;
                        },
                        valueProviderCallback: _ => compareRegisters[registerIndex].CompareValue);
                }, stepInBytes: 4);

            Registers.Configuration.Define(this, 0x80000000)
                .WithEnumField(0, 4, out clockSelect, name: "CLKSEL", changeCallback: (_, __) => UpdateFrequency())
                .WithReservedBits(4, 4)
                .WithFlags(8, 8, name: "COMPARExEN", changeCallback: (registerIndex, _, newValue) => compareRegisters[registerIndex].Enabled = newValue)
                .WithReservedBits(16, 14)
                .WithFlag(30, out clear, name: "CLEAR", changeCallback: (_, __) => UpdateSystemTimerState())
                .WithFlag(31, out freeze, name: "FREEZE", changeCallback: (_, __) => UpdateSystemTimerState())
                ;

            Registers.InterruptClear.Define(this)
                .WithFlags(0, 8, FieldMode.Write, name: "COMPAREx",
                    writeCallback: (registerIndex, _, newValue) => { if(newValue) compareRegisters[registerIndex].InterruptStatus = false; })
                .WithFlag(8, FieldMode.Write, name: "OVERFLOW",
                    writeCallback: (_, newValue) => { if(newValue) { overflowStatus = false; UpdateCaptureOverflowIRQ(); } })
                .WithFlags(9, 4, FieldMode.Write, name: "CAPTUREx",
                    writeCallback: (registerIndex, _, newValue) => { if(newValue) captureRegisters[registerIndex].InterruptStatus = false; })
                .WithReservedBits(13, 19)
                ;

            Registers.InterruptEnable.Define(this)
                .WithFlags(0, 8, name: "COMPAREx",
                    changeCallback: (registerIndex, _, newValue) => compareRegisters[registerIndex].InterruptEnable = newValue,
                    valueProviderCallback: (registerIndex, _) => compareRegisters[registerIndex].InterruptEnable)
                .WithFlag(8, out overflowInterruptEnable, name: "OVERFLOW",
                    changeCallback: (_, __) => UpdateCaptureOverflowIRQ())
                .WithFlags(9, 4, name: "CAPTUREx",
                    changeCallback: (registerIdx, _, newValue) => captureRegisters[registerIdx].InterruptEnable = newValue,
                    valueProviderCallback: (registerIdx, _) => captureRegisters[registerIdx].InterruptEnable)
                .WithReservedBits(13, 19)
                ;

            Registers.InterruptSet.Define(this)
                .WithFlags(0, 8, FieldMode.Write, name: "COMPAREx",
                    writeCallback: (registerIndex, _, newValue) => { if(newValue) compareRegisters[registerIndex].InterruptStatus = true; })
                .WithFlag(8, FieldMode.Write, name: "OVERFLOW",
                    writeCallback: (_, newValue) => { if(newValue) { overflowStatus = true; UpdateCaptureOverflowIRQ(); } })
                .WithFlags(9, 4, FieldMode.Write, name: "CAPTUREx",
                    writeCallback: (registerIndex, _, newValue) => { if(newValue) captureRegisters[registerIndex].InterruptStatus = true; })
                .WithReservedBits(13, 19)
                ;

            // Read composes the live statuses; direct writes are honored
            // (CMSIS declares STMINTSTAT __IOM and
            // am_hal_stimer_reset_config() stores 0 to it).
            Registers.InterruptStatus.Define(this)
                .WithFlags(0, 8, name: "COMPAREx",
                    valueProviderCallback: (registerIndex, _) => compareRegisters[registerIndex].InterruptStatus,
                    writeCallback: (registerIndex, _, newValue) => compareRegisters[registerIndex].InterruptStatus = newValue)
                .WithFlag(8, name: "OVERFLOW",
                    valueProviderCallback: _ => overflowStatus,
                    writeCallback: (_, newValue) => { overflowStatus = newValue; UpdateCaptureOverflowIRQ(); })
                .WithFlags(9, 4, name: "CAPTUREx",
                    valueProviderCallback: (registerIdx, _) => captureRegisters[registerIdx].InterruptStatus,
                    writeCallback: (registerIdx, _, newValue) => captureRegisters[registerIdx].InterruptStatus = newValue)
                .WithReservedBits(13, 19)
                ;

            Registers.SystemTimerCount.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "STTMR", valueProviderCallback: _ => Value)
                ;

            // SNVR0/SNVR1 are non-volatile scratch words; 0x58 is HALSTATES on
            // Apollo510 ("reserved for internal HAL usage" clock-mux state +
            // SIGNATURE) but the HAL only ever stores to and reloads from it,
            // so plain read/write storage models all three. Offset 0x5C is not
            // a STIMER register at all: RSTGEN's STAT register (RSTGEN base
            // 0x40000000 + 0x885C) physically lands inside this page at
            // STIMER_BASE + 0x5C. The boot path reads STAT.POASTAT once
            // (am_hal_pwrctrl_low_power_init, am_hal_sysctrl_clkmuxrst_low_
            // power_init) - a zero-reset scratch word ("no power-on-analog
            // reset pending") makes both recovery paths no-ops.
            Registers.SystemTimerNVRAM0.DefineMany(this, 4,
                (register, registerIndex) =>
                {
                    var names = new[] { "SNVR0", "SNVR1", "HALSTATES", "RSTGEN_STAT" };
                    register.WithValueField(0, 32, name: names[registerIndex]);
                }, stepInBytes: 4);
        }

        private void HandleLimitReached()
        {
            this.Log(LogLevel.Debug, "COUNTER overflow occurred");
            overflowStatus = true;
            UpdateCaptureOverflowIRQ();
        }

        private void UpdateCaptureOverflowIRQ()
        {
            var newIrqState = false;
            foreach(var captureRegister in captureRegisters)
            {
                if(captureRegister.InterruptEnable && captureRegister.InterruptStatus)
                {
                    newIrqState = true;
                    break;
                }
            }
            if(!newIrqState && overflowStatus && overflowInterruptEnable.Value)
            {
                newIrqState = true;
            }

            if(IRQI.IsSet != newIrqState)
            {
                this.Log(LogLevel.Debug, "IRQI {0}", newIrqState ? "set" : "unset");
                IRQI.Set(newIrqState);
            }
        }

        private void UpdateFrequency()
        {
            var frequencySet = InvalidFrequency;
            switch(clockSelect.Value)
            {
            case ClockSelectValues.NOCLK:
                this.Log(LogLevel.Debug, "CLKSEL set to NOCLK. Timer will be disabled.");
                break;
            case ClockSelectValues.HFRC_6MHZ:
                frequencySet = 6000000;
                break;
            case ClockSelectValues.HFRC_375KHZ:
                frequencySet = 375000;
                break;
            case ClockSelectValues.XTAL_32KHZ:
                // 32768 Hz from the 32.768 kHz crystal (CMSIS apollo510.h:
                // "XTAL_32KHZ : 32768Hz from the crystal oscillator") - matches
                // Zephyr's CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=32768.
                frequencySet = 32768;
                break;
            case ClockSelectValues.XTAL_16KHZ:
                frequencySet = 16384;
                break;
            case ClockSelectValues.XTAL_1KHZ:
                frequencySet = 1024;
                break;
            case ClockSelectValues.LFRC_1KHZ:
                // "LFRC_NOMINAL : Approximately 900Hz from the LFRC oscillator
                // (uncalibrated)" - modeled at its nominal value. Software
                // assuming the historical "1 kHz" name will run ~10% slow.
                this.Log(LogLevel.Warning, "CLKSEL set to LFRC: modeled at the nominal (uncalibrated) 900 Hz");
                frequencySet = 900;
                break;
            case ClockSelectValues.CTIMER0:
            case ClockSelectValues.CTIMER1:
                this.Log(LogLevel.Warning, "Unsupported CLKSEL value: {0}. Timer will be disabled.", clockSelect.Value);
                break;
            default:
                this.Log(LogLevel.Error, "Invalid CLKSEL value: 0x{0:X}. Timer will be disabled.", (uint)clockSelect.Value);
                break;
            }

            // Always write the (possibly sentinel) frequency through: leaving
            // the previous value in place kept the counter free-running after
            // a switch back to NOCLK.
            this.Log(LogLevel.Debug, "Updating timer's frequency to {0} Hz; CLKSEL={1} ({2})", frequencySet, (uint)clockSelect.Value, clockSelect.Value);
            systemTimer.Frequency = frequencySet;
            Array.ForEach(compareRegisters, register => register.Frequency = frequencySet);
            // CLKSEL influences the timer state depending on whether the frequency is valid or not.
            UpdateSystemTimerState();
        }

        private void UpdateSystemTimerState()
        {
            systemTimer.Enabled = !freeze.Value && !clear.Value && systemTimer.Frequency != InvalidFrequency;
            if(clear.Value)
            {
                systemTimer.ResetValue();
            }
            Array.ForEach(compareRegisters, register => register.UpdateState());
        }

        // True iff 'target' lies inside the modular interval (from, to] -
        // i.e. a counter jumping from 'from' to 'to' passed (or landed on)
        // the target.
        private static bool JumpPassedTarget(uint from, uint to, uint target)
        {
            return (uint)(target - from - 1) < (uint)(to - from);
        }

        private IFlagRegisterField clear;
        private IEnumRegisterField<ClockSelectValues> clockSelect;
        private IFlagRegisterField freeze;
        private IFlagRegisterField overflowInterruptEnable;
        private bool overflowStatus;

        private readonly CaptureRegister[] captureRegisters = new CaptureRegister[CaptureRegistersCount];
        private readonly CompareRegister[] compareRegisters = new CompareRegister[CompareRegistersCount];
        private readonly GPIO[] interruptOutputs = new GPIO[InterruptOutputsCount];
        private readonly LimitTimer systemTimer;
        private readonly Dictionary<int, bool> gpioStates = new Dictionary<int, bool>();

        private const uint CompareRegistersCount = 8;
        private const uint CaptureRegistersCount = 4;
        private const uint InterruptOutputsCount = 9;
        // STTMR counts 0..0xFFFFFFFF and overflows "from 0xFFFFFFFF back to
        // 0x00000000" - a full 2^32-tick lap. (uint.MaxValue here would make
        // 0xFFFFFFFF unobservable and lose one tick per wrap.)
        private const ulong CounterSpan = 0x100000000UL;
        // It's used for CLKSEL options which stop the timer. 0 can't be set as the timer's frequency, hence 1.
        private const ulong InvalidFrequency = 1;

        private class CaptureRegister
        {
            public CaptureRegister(AmbiqApollo5_SystemTimer owner, int index)
            {
                this.owner = owner;
                var nameSuffix = (char)('A' + index);
                name = $"CAPTURE_{nameSuffix}";
            }

            public void OnGPIOEdge(int number, bool high)
            {
                if(Enabled.Value
                        && (int)TriggerSourceGPIOPinNumber.Value == number
                        // The 'high' value is the new pin level, so a low-to-high
                        // transition arrives as 'true'. STPOL: 0 = capture on
                        // low-to-high, 1 = capture on high-to-low.
                        && CaptureOnHighToLowGPIOTransition.Value != high)
                {
                    // "Whenever the event is detected, the value in the COUNTER
                    // is copied into this register and the corresponding
                    // interrupt status bit is set." A subsequent event simply
                    // overwrites - no FIFO, no overrun flag.
                    ValueCaptured = owner.Value;
                    InterruptStatus = true;
                    owner.Log(LogLevel.Debug, "{0}: Register set with value: 0x{1:X}", name, ValueCaptured);
                }
            }

            public void Reset()
            {
                interruptEnable = false;
                interruptStatus = false;
                ValueCaptured = 0;
            }

            public bool InterruptEnable
            {
                get => interruptEnable;
                set
                {
                    owner.Log(LogLevel.Noisy, "{0}: Setting Interrupt Enable to: {1}", name, value);
                    interruptEnable = value;
                    owner.UpdateCaptureOverflowIRQ();
                }
            }

            public bool InterruptStatus
            {
                get => interruptStatus;
                set
                {
                    owner.Log(LogLevel.Noisy, "{0}: Setting Interrupt Status to: {1}", name, value);
                    interruptStatus = value;
                    owner.UpdateCaptureOverflowIRQ();
                }
            }

            public IFlagRegisterField CaptureOnHighToLowGPIOTransition;
            public IFlagRegisterField Enabled;
            public IValueRegisterField TriggerSourceGPIOPinNumber;
            public uint ValueCaptured;

            private bool interruptEnable;
            private bool interruptStatus;

            private readonly string name;
            private readonly AmbiqApollo5_SystemTimer owner;
        }

        private class CompareRegister
        {
            public CompareRegister(AmbiqApollo5_SystemTimer owner, int index, GPIO irq, LimitTimer systemTimer)
            {
                this.irq = irq;
                this.owner = owner;
                this.systemTimer = systemTimer;
                var nameSuffix = (char)('A' + index);
                name = $"COMPARE_{nameSuffix}";

                innerTimer = new ComparingTimer(owner.machine.ClockSource, owner.Frequency, owner, name, direction: Direction.Ascending, limit: CounterSpan,
                        enabled: false, workMode: WorkMode.Periodic, eventEnabled: true, compare: 0, divider: 1);
                innerTimer.CompareReached += () =>
                {
                    owner.Log(LogLevel.Debug, "{0}: Compare value (0x{1:X}) reached", name, innerTimer.Compare);
                    InterruptStatus = true;
                };
            }

            public void Reset()
            {
                enabled = false;
                innerTimer.Reset();
                interruptEnable = false;
                interruptStatus = false;
                irq.Unset();
            }

            public void UpdateState()
            {
                if(enabled && systemTimer.Enabled)
                {
                    var value = owner.Value;
                    innerTimer.Value = value;
                    innerTimer.Enabled = true;
                    // STMINT COMPAREx documents a "COUNTER is greater than or
                    // equal to COMPARE" condition. A comparator (re)enabled
                    // against a target the counter is already at or past would
                    // otherwise wait a full 2^32-tick lap. "Past" cannot be
                    // expressed exactly in modular arithmetic; treat a forward
                    // distance of more than half the counter span as passed.
                    var distance = (uint)(CompareValue - value);
                    if(!interruptStatus && (distance == 0 || distance > HalfCounterSpan))
                    {
                        // Debug, not Warning: enabling a comparator whose SCMPR
                        // still holds a stale (e.g. reset) target is a normal
                        // software flow - the HAL explicitly warns that "the
                        // application could get a stale interrupt" and expects
                        // it to be handled gracefully.
                        owner.Log(LogLevel.Debug, "{0}: enabled with target 0x{1:X} at/behind COUNTER 0x{2:X} - raising the interrupt status immediately (hardware condition is COUNTER >= COMPARE)", name, CompareValue, value);
                        InterruptStatus = true;
                    }
                }
                else
                {
                    innerTimer.Enabled = false;
                }
            }

            // Keeps this comparator coherent across a monitor-driven counter
            // jump (see CounterValue): the view of COUNTER is resynced, and a
            // target inside the jumped-over interval fires immediately - just
            // as it would had the counter really advanced through the range.
            public void HandleCounterJump(uint from, uint to)
            {
                innerTimer.Value = to;
                if(enabled && systemTimer.Enabled && JumpPassedTarget(from, to, CompareValue))
                {
                    owner.Log(LogLevel.Info, "{0}: COUNTER jump 0x{1:X}->0x{2:X} passed target 0x{3:X} - raising the interrupt status", name, from, to, CompareValue);
                    InterruptStatus = true;
                }
            }

            public uint CompareValue
            {
                get => (uint)innerTimer.Compare;
                set
                {
                    owner.Log(LogLevel.Noisy, "{0}: Setting compare value to: 0x{1:X}", name, value);
                    innerTimer.Compare = value;
                }
            }

            public bool Enabled
            {
                get => enabled;
                set
                {
                    owner.Log(LogLevel.Noisy, "{0}: Setting Enabled to: {1}", name, value);
                    enabled = value;
                    UpdateState();
                }
            }

            public ulong Frequency
            {
                get => innerTimer.Frequency;
                set => innerTimer.Frequency = value;
            }

            public bool InterruptEnable
            {
                get => interruptEnable;
                set
                {
                    owner.Log(LogLevel.Noisy, "{0}: Setting interrupt enable to: {1}", name, value);
                    interruptEnable = value;
                    UpdateIRQ();
                }
            }

            public bool InterruptStatus
            {
                get => interruptStatus;
                set
                {
                    owner.Log(LogLevel.Noisy, "{0}: Setting interrupt status to: {1}", name, value);
                    interruptStatus = value;
                    UpdateIRQ();
                }
            }

            private void UpdateIRQ()
            {
                var newIrqState = interruptEnable && interruptStatus;
                if(irq.IsSet != newIrqState)
                {
                    owner.Log(LogLevel.Debug, "{0}: {1} IRQ", name, newIrqState ? "Setting" : "Clearing");
                    irq.Set(newIrqState);
                }
            }

            private const uint HalfCounterSpan = 0x80000000;

            private bool enabled;
            private bool interruptEnable;
            private bool interruptStatus;

            private readonly ComparingTimer innerTimer;
            private readonly GPIO irq;
            private readonly string name;
            private readonly AmbiqApollo5_SystemTimer owner;
            private readonly LimitTimer systemTimer;
        }

        private enum ClockSelectValues
        {
            NOCLK,
            HFRC_6MHZ,
            HFRC_375KHZ,
            XTAL_32KHZ,
            XTAL_16KHZ,
            XTAL_1KHZ,
            LFRC_1KHZ,
            CTIMER0,
            CTIMER1,
        }

        private enum Registers : long
        {
            Configuration = 0x0,
            SystemTimerCount = 0x4,
            CaptureControl0 = 0x10,
            CaptureControl1 = 0x14,
            CaptureControl2 = 0x18,
            CaptureControl3 = 0x1C,
            Compare0 = 0x20,
            Compare1 = 0x24,
            Compare2 = 0x28,
            Compare3 = 0x2C,
            Compare4 = 0x30,
            Compare5 = 0x34,
            Compare6 = 0x38,
            Compare7 = 0x3C,
            Capture0 = 0x40,
            Capture1 = 0x44,
            Capture2 = 0x48,
            Capture3 = 0x4C,
            SystemTimerNVRAM0 = 0x50,
            SystemTimerNVRAM1 = 0x54,
            HalStates = 0x58,
            ResetGeneratorStatusAlias = 0x5C,
            InterruptEnable = 0x100,
            InterruptStatus = 0x104,
            InterruptClear = 0x108,
            InterruptSet = 0x10C,
        }
    }

    // Minimal stand-in for the Apollo510 TIMER block (0x40008000, the page
    // below the STIMER): reads-as-written scratch for the whole page, plus
    // forwarding of GLOBEN.ENABLEALLINPUTS (GLOBEN is at offset 0x10 -
    // CTRL, STATUS, two reserved words, then GLOBEN per CMSIS TIMER_Type -
    // bit 29) into the STIMER model's capture gate.
    // am_hal_stimer_capture_start() sets the bit after arming SCAPCTRLn;
    // am_hal_stimer_capture_stop() clears it once all four capture units
    // are disabled.
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class AmbiqApollo5_TimerStub : IDoubleWordPeripheral, IKnownSize
    {
        public AmbiqApollo5_TimerStub(IMachine machine, AmbiqApollo5_SystemTimer systemTimer)
        {
            this.systemTimer = systemTimer;
            Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            return storage.TryGetValue(offset, out var stored) ? stored : 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            storage[offset] = value;
            if(offset == GlobalEnableOffset)
            {
                systemTimer.GlobalInputsEnabled = ((value >> EnableAllInputsBit) & 1) != 0;
            }
        }

        public void Reset()
        {
            storage.Clear();
            systemTimer.GlobalInputsEnabled = false;
        }

        public long Size => 0x800;

        private readonly Dictionary<long, uint> storage = new Dictionary<long, uint>();
        private readonly AmbiqApollo5_SystemTimer systemTimer;

        private const long GlobalEnableOffset = 0x10;
        private const int EnableAllInputsBit = 29;
    }
}
