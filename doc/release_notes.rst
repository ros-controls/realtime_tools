:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

LockFreeQueue
*******************************
* Added ``LockFreePooledSPSCQueue`` for large or move-only types, backed by a preallocated pool (`#606 <https://github.com/ros-controls/realtime_tools/pull/606>`__).

RealtimeTripleBuffer
*******************************
* Added ``RealtimeTripleBuffer``, a wait-free single-producer single-consumer channel that always holds the latest value, e.g. to pass the latest sensor sample from a driver thread to the control loop (`#616 <https://github.com/ros-controls/realtime_tools/pull/616>`__).
