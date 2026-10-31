:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

LockFreeQueue
*******************************
* The ``LockFreeSPSCQueue`` and ``LockFreeMPMCQueue`` no longer have a virtual destructor, and ``push``/``bounded_push`` now only accept types that are convertible to the stored type (`#606 <https://github.com/ros-controls/realtime_tools/pull/606>`__).
* ``LockFreeSPSCQueue::get_latest`` only copies the latest element instead of every element in the queue (`#606 <https://github.com/ros-controls/realtime_tools/pull/606>`__).
* Added ``LockFreePooledSPSCQueue`` for large or move-only types, backed by a preallocated pool (`#606 <https://github.com/ros-controls/realtime_tools/pull/606>`__).
