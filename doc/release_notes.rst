:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Jazzy (previous) and Kilted (current) releases.

AsyncFunctionHandler
*******************************
* Add DETACHED scheduling policy for Async function handler (`#383 <https://github.com/ros-controls/realtime_tools/pull/383>`__).
* Added the ``SELF_PACED`` scheduling policy. The worker thread does not sleep. The callback (a blocking hardware ``read()``), sets the cycle time. If the callback returns in less than 10% of the ``exec_rate`` period, the handler sleeps for 100 us and prints a warning (`#473 <https://github.com/ros-controls/realtime_tools/pull/473>`__).

RealtimeBox
*******************************
* ``RealtimeBox`` got renamed to ``RealtimeThreadSafeBox`` and uses real-time mutexes now (`#318 <https://github.com/ros-controls/realtime_tools/pull/318>`__, `#342 <https://github.com/ros-controls/realtime_tools/pull/342>`__).
* ``RealtimePublisher`` is updated with a new ``try_publish`` API and few methods are deprecated (`#323 <https://github.com/ros-controls/realtime_tools/pull/323>`__).

LockFreeQueue
*******************************
* Added ``LockFreePooledSPSCQueue`` for large or move-only types, backed by a preallocated pool (`#606 <https://github.com/ros-controls/realtime_tools/pull/606>`__, `#615 <https://github.com/ros-controls/realtime_tools/pull/615>`__).
