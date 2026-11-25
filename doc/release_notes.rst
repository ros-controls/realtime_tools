:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

AsyncFunctionHandler
*******************************
* Added the ``SELF_PACED`` scheduling policy. The worker thread does not sleep. The callback (a blocking hardware ``read()``), sets the cycle time. If the callback returns in less than 10% of the ``exec_rate`` period, the handler sleeps for 100 us and prints a warning (`#473 <https://github.com/ros-controls/realtime_tools/pull/473>`__).

LockFreeQueue
*******************************
* Added ``LockFreePooledSPSCQueue`` for large or move-only types, backed by a preallocated pool (`#606 <https://github.com/ros-controls/realtime_tools/pull/606>`__, `#615 <https://github.com/ros-controls/realtime_tools/pull/615>`__).
