:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

LockFreeQueue
*******************************
* Added ``LockFreePooledSPSCQueue`` for large or move-only types, backed by a preallocated pool (`#606 <https://github.com/ros-controls/realtime_tools/pull/606>`__).

Realtime helpers
*******************************
* Added ``check_realtime_environment()``, which reports the realtime kernel, RT priority and memlock limits/capabilities, CPU governor and isolated CPUs with hints on how to fix each issue (`#617 <https://github.com/ros-controls/realtime_tools/pull/617>`__).
