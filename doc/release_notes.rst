:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

LockFreeQueue
*******************************
* Added ``LockFreePooledSPSCQueue`` for large or move-only types, backed by a preallocated pool (`#606 <https://github.com/ros-controls/realtime_tools/pull/606>`__).

Memory locking
*******************************
* Added a ``lock_memory(prefault_stack_size, prefault_heap_size)`` overload that additionally keeps freed heap memory in the process (glibc) and prefaults the given amount of heap and of the calling thread's stack. ``lock_memory()`` is unchanged (`#618 <https://github.com/ros-controls/realtime_tools/pull/618>`__).
