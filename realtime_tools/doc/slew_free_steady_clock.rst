SlewFreeSteadyClock
---------------------
``std::chrono::steady_clock`` maps to ``CLOCK_MONOTONIC`` on Linux. It never jumps, but its rate is continuously corrected (slewed) by NTP/PTP daemons, which shows up as period and ``dt`` jitter in realtime loops. ``realtime_tools::SlewFreeSteadyClock`` reads ``CLOCK_MONOTONIC_RAW`` instead, so measured durations reflect the hardware oscillator only. It is a drop-in ``std::chrono`` clock with a ``noexcept``, allocation-free ``now()``:

.. code-block:: cpp

  #include <realtime_tools/slew_free_steady_clock.hpp>

  const auto start = realtime_tools::SlewFreeSteadyClock::now();
  // ... realtime work ...
  const auto elapsed = realtime_tools::SlewFreeSteadyClock::now() - start;  // std::chrono::nanoseconds

Notes:

* Its epoch differs from ``CLOCK_MONOTONIC``; do not mix its time points with ``std::chrono::steady_clock`` or ROS time.
* ``clock_nanosleep()`` does not support ``CLOCK_MONOTONIC_RAW``; use it to measure time, not to schedule absolute wake-ups.
* On platforms without ``CLOCK_MONOTONIC_RAW`` (e.g. Windows) it falls back to ``std::chrono::steady_clock``.
