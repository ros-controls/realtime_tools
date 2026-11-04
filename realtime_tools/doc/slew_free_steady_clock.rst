SlewFreeSteadyClock
---------------------
``realtime_tools::SlewFreeSteadyClock`` is a drop-in ``std::chrono`` clock for realtime loops. It reads ``CLOCK_MONOTONIC_RAW``, the hardware counter that time daemons never adjust, and provides its own ``sleep_for()`` / ``sleep_until()`` that never wake early on that clock.

.. code-block:: cpp

  #include <realtime_tools/slew_free_steady_clock.hpp>

  using realtime_tools::SlewFreeSteadyClock;

  // Measure durations
  const auto start = SlewFreeSteadyClock::now();
  // ... realtime work ...
  const auto elapsed = SlewFreeSteadyClock::now() - start;  // std::chrono::nanoseconds

  // Fixed-rate loop timed on the raw clock
  auto next = SlewFreeSteadyClock::now();
  while (running) {
    next += std::chrono::milliseconds(1);
    // ... realtime work ...
    SlewFreeSteadyClock::sleep_until(next);
  }

  // Relative sleep
  SlewFreeSteadyClock::sleep_for(std::chrono::microseconds(500));

Why not ``std::chrono::steady_clock``?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
On Linux, ``std::chrono::steady_clock`` reads ``CLOCK_MONOTONIC``. That clock never jumps, but NTP, PTP and chrony keep adjusting its *rate* (slewing it) to track a reference clock. In a realtime loop, those adjustments show up as period and ``dt`` errors that have nothing to do with the hardware:

* **Frequency adjustment** (``adjtimex`` ``ADJ_FREQUENCY``) is limited by the kernel to ±500 ppm. ntpd and steady-state chrony stay in this range.
* **Tick adjustment** (``adjtimex`` ``ADJ_TICK``) allows up to ±10 %. chrony uses it to correct large offsets quickly; its default ``maxslewrate`` is 83 333 ppm (8.3 %).

``CLOCK_MONOTONIC_RAW`` follows the local oscillator only, so a 1 ms period measured on it is 1 ms of hardware time, whatever the daemon is doing at the time.

.. list-table::
   :header-rows: 1

   * -
     - ``std::chrono::steady_clock``
     - ``SlewFreeSteadyClock``
   * - Linux clock source
     - ``CLOCK_MONOTONIC``
     - ``CLOCK_MONOTONIC_RAW``
   * - Can jump
     - No
     - No
   * - Rate adjusted by NTP/PTP/chrony
     - Yes, up to ±10 %
     - No
   * - ``now()`` cost
     - vDSO call, tens of ns
     - vDSO call, tens of ns
   * - Sleeping can return early on its own clock
     - Yes (see below)
     - No
   * - Tracks wall-clock time over the long run
     - Yes
     - No, drifts with the crystal (typically ±20–50 ppm)

Use ``SlewFreeSteadyClock`` to time control loops, measure ``dt`` and benchmark code. Keep ``std::chrono::steady_clock`` or ROS time when timestamps must agree with other machines or with the system time.

Sleeping
^^^^^^^^
Always use the clock's own ``SlewFreeSteadyClock::sleep_for()`` / ``sleep_until()``, not ``std::this_thread::sleep_for()`` / ``sleep_until()``:

* ``std::this_thread::sleep_for()`` calls ``nanosleep()``, which counts time on ``CLOCK_MONOTONIC``, the slewed clock.
* For any steady clock, libstdc++'s ``std::this_thread::sleep_until()`` reads ``Clock::now()`` once, calls ``sleep_for()`` once, and returns without checking the clock again. With this clock it can therefore return early by up to ``period × slew rate``.
* ``clock_nanosleep()`` does not accept ``CLOCK_MONOTONIC_RAW``, so the kernel cannot sleep on the raw clock directly.

``SlewFreeSteadyClock::sleep_until()`` sleeps for the remaining time, re-reads the raw clock, and sleeps again until the target is reached. Each pass shrinks the remaining error by a factor of at least 10 (at the ±10 % worst case), so it never returns early. ``sleep_for(d)`` is ``sleep_until(now() + d)``, with ``d`` rounded up to whole nanoseconds; zero or negative durations return immediately.

How large is the slew error?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The error is proportional to the sleep length: ``error ≤ period × slew rate``.

.. list-table::
   :header-rows: 1

   * - Period
     - Steady-state slew (≤ 500 ppm)
     - chrony fast slew (8.3 %)
   * - 10 ms (100 Hz)
     - ≤ 5 µs
     - ≤ 830 µs
   * - 1 ms (1 kHz)
     - ≤ 0.5 µs
     - ≤ 83 µs
   * - 500 µs (2 kHz)
     - ≤ 0.25 µs
     - ≤ 42 µs
   * - 100 µs (10 kHz)
     - ≤ 0.05 µs
     - ≤ 8 µs

Compare these values with the kernel's wake-up latency, which is typically 5–50 µs on a tuned PREEMPT_RT system with ``SCHED_FIFO``:

* **Steady state:** the slew error is far below the wake-up latency. The first wake already lands after the target, and the re-check costs only one extra ``now()`` call.
* **Fast slew:** the error can exceed the latency. ``std::this_thread`` would then return early without any sign that it happened. ``SlewFreeSteadyClock`` does one more short sleep instead, which costs one extra wake-up latency but keeps the period correct.

Choosing a loop frequency
^^^^^^^^^^^^^^^^^^^^^^^^^
Reading the clock costs a few tens of nanoseconds, so ``now()`` can be used at any rate. The practical limit is the cost of sleeping and waking:

* Sleeping works well while the period is much longer than the wake-up latency, roughly up to 5–10 kHz on a tuned PREEMPT_RT system.
* Above about 10 kHz the latency, and an occasional extra pass, become a large share of the period. Busy-wait on ``now()`` instead, or sleep through most of the period and spin for the last 20–50 µs.
* Run the thread with ``SCHED_FIFO`` (see ``realtime_tools::configure_sched_fifo()``). Non-realtime threads get a default timer slack of 50 µs, which is larger than the slew error at 1 kHz.

The latency figures above are typical values. Measure your target with ``cyclictest`` to get the real numbers.

Notes
^^^^^
* The epoch is unspecified (around boot on Linux) and differs from ``CLOCK_MONOTONIC``. Do not mix its time points with ``std::chrono::steady_clock`` or ROS time.
* On platforms without ``CLOCK_MONOTONIC_RAW`` (e.g. Windows, whose ``QueryPerformanceCounter`` is not slewed), the clock falls back to ``std::chrono::steady_clock``.
