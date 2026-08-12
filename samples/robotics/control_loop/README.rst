.. zephyr:code-sample:: robotics-control-loop
   :name: Control loop
   :relevant-api: zbus_apis

   Run a deterministic closed control loop over zbus with acquisition
   timestamps and a pluggable transport bridge.

Overview
********

This sample is a skeleton for the recurring shape of robotics firmware: a
fixed-rate control loop whose stages are decoupled by messages, whose
timestamps mean *acquisition time*, and whose link to the outside world is
pluggable. Everything is plain-old-data structs and zbus channels — no
framework.

.. code-block:: none

   sensor thread (100 Hz, absolute deadlines)
        │  sensor_chan (stamped at acquisition)
        ▼
   controller (subscriber thread, PI)
        │  actuator_chan (carries the causing sample's stamp)
        ▼
   actuator (listener) ──► plant (simulated first-order motor)

   setpoint_chan ◄── local square-wave demo, or the bridge
   estop_chan    ◄── latches the actuator to zero, bypassing the controller

Three properties worth copying:

* **Absolute deadlines.** The sensor thread sleeps with
  :c:macro:`K_TIMEOUT_ABS_TICKS`, so the period never accumulates handler
  execution time, and its wakeup error against each deadline is measured in
  the kernel timer's own clock domain (no cross-oscillator drift).

* **Acquisition timestamps.** ``stamp_us`` is set when the sample is taken
  and *propagated* through derived messages, which is what makes the
  end-to-end (acquisition → actuation) latency measurable at the actuator.

* **Transport as an observer.** The optional bridge exports sensor/actuator
  telemetry and injects setpoint/e-stop commands by publishing locally; the
  loop cannot tell remote producers from local ones. Backends are two
  functions (``init`` + ``write``, plus feeding received bytes back);
  serial-UART and UDP implementations are provided, and the same seam fits
  a TSN or Zenoh-style transport.

The wire format is ``0x7E | id(u16) | len(u16) | payload | crc16-CCITT``
with little-endian POD payloads — deliberately interoperable with vendor
implementations of the same framing.

Building and Running
********************

Local loop only (any board; ``native_sim`` and ``qemu_cortex_m3`` run it
in CI):

.. zephyr-app-commands::
   :zephyr-app: samples/robotics/control_loop
   :board: native_sim
   :goals: build run
   :compact:

With the UDP bridge on ``native_sim`` (NSOS offloaded sockets — no host
setup; frames arrive on UDP port 17448):

.. zephyr-app-commands::
   :zephyr-app: samples/robotics/control_loop
   :board: native_sim
   :gen-args: -DEXTRA_CONF_FILE=overlay-udp.conf
   :goals: build run
   :compact:

With the serial bridge (UART named by the ``bridge-uart`` alias; a PTY on
``native_sim``, UART1 on ``apollo510_evb``):

.. zephyr-app-commands::
   :zephyr-app: samples/robotics/control_loop
   :board: apollo510_evb
   :gen-args: -DEXTRA_CONF_FILE=overlay-serial.conf
   :goals: build
   :compact:

Sample Output
*************

.. code-block:: console

   robotics control_loop sample on native_sim (period 10000 us)
   loop: n=100 period_us=10000 wake_jitter_us min=0 avg=0 max=0 e2e_us min=8 avg=14 max=41
   loop: n=200 period_us=10000 wake_jitter_us min=0 avg=0 max=0 e2e_us min=8 avg=13 max=38
   loop: n=300 period_us=10000 wake_jitter_us min=0 avg=0 max=0 e2e_us min=8 avg=13 max=35
   RECORD: {"sample":"control_loop","board":"native_sim","n":300,"wake_avg_us":0,...}
   estop: actuator commanded to zero, control loop halted

The ``RECORD:`` line is machine-readable and captured by twister
(``harness_config.record`` with ``as_json``), so numbers diff cleanly
across boards and runs.

Control-period budget
*********************

The loop runs at 100 Hz — a 10 ms period. A budget for that period on a
Cortex-M-class target looks like:

===========================  =========
Stage                        Budget
===========================  =========
Sensor acquisition + publish 200 µs
Controller math + publish    100 µs
Actuation                    100 µs
Wakeup jitter allowance      500 µs
Headroom (telemetry, logs)   9.1 ms
===========================  =========

Two caveats when reading measured numbers:

* On ``native_sim`` simulated time does not advance while code executes,
  so jitter and latency read as ~0 — the numbers are CI plumbing, not
  data. Measure on hardware.
* On boards whose kernel tick is a slow always-on timer (for example
  Apollo510's 32.768 kHz stimer), the wakeup-jitter resolution equals one
  timer cycle (~30.5 µs) — which is also the deadline granularity itself.
