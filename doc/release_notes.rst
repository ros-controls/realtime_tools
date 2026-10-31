:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Humble to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Humble (previous) and Jazzy (current) releases.

<<<<<<< HEAD
.. note::

  This list was created in June 2025 (tag 3.6.0), earlier changes may not be included.

AsyncFunctionHandler
*******************************
* Add DETACHED scheduling policy for Async function handler (`#383 <https://github.com/ros-controls/realtime_tools/pull/383>`__).

RealtimeBox
*******************************
* ``RealtimeBox`` got renamed to ``RealtimeThreadSafeBox`` and uses real-time mutexes now (`#318 <https://github.com/ros-controls/realtime_tools/pull/318>`__, `#342 <https://github.com/ros-controls/realtime_tools/pull/342>`__).
* ``RealtimePublisher`` is updated with a new ``try_publish`` API and few methods are deprecated (`#323 <https://github.com/ros-controls/realtime_tools/pull/323>`__).
=======
This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

LockFreeQueue
*******************************
* Added ``LockFreePooledSPSCQueue`` for large or move-only types, backed by a preallocated pool (`#606 <https://github.com/ros-controls/realtime_tools/pull/606>`__).
>>>>>>> 7f89683 (Improve lock free queue and add pooled SPSC queue (#606))
