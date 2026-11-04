SlewFreeSteadyClock
---------------------
``std::chrono::steady_clock`` maps to ``CLOCK_MONOTONIC`` on Linux. It never jumps, but its rate is continuously corrected (slewed) by NTP/PTP daemons, which shows up as period and ``dt`` jitter in realtime loops. ``realtime_tools::SlewFreeSteadyClock`` reads ``CLOCK_MONOTONIC_RAW`` instead, so measured durations reflect the hardware oscillator only. It is a drop-in ``std::chrono`` clock with a ``noexcept``, allocation-free ``now()``:

.. code-block:: cpp

  #include <realtime_tools/slew_free_steady_clock.hpp>

  const auto start = realtime_tools::SlewFreeSteadyClock::now();
  // ... realtime work ...
  const auto elapsed = realtime_tools::SlewFreeSteadyClock::now() - start;  // std::chrono::nanoseconds

  // Fixed-rate loop timed on the raw clock
  auto next = realtime_tools::SlewFreeSteadyClock::now();
  while (running) {
    next += std::chrono::milliseconds(1);
    // ... realtime work ...
    realtime_tools::SlewFreeSteadyClock::sleep_until(next);
  }

  realtime_tools::SlewFreeSteadyClock::sleep_for(std::chrono::microseconds(500));

Notes:

* Its epoch differs from ``CLOCK_MONOTONIC``; do not mix its time points with ``std::chrono::steady_clock`` or ROS time.
* Use the static ``SlewFreeSteadyClock::sleep_for()`` / ``sleep_until()``, not ``std::this_thread::sleep_for()`` / ``sleep_until()``. ``clock_nanosleep()`` does not accept ``CLOCK_MONOTONIC_RAW``, and for steady clocks libstdc++ sleeps once on ``CLOCK_MONOTONIC`` without re-checking, so it can wake early by the NTP slew. The members re-check the raw clock and sleep again until the target is reached, so they never return early.
* On platforms without ``CLOCK_MONOTONIC_RAW`` (e.g. Windows) it falls back to ``std::chrono::steady_clock``.
