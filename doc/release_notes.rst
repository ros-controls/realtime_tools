:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Jazzy (previous) and Kilted (current) releases.

AsyncFunctionHandler
*******************************
* Add DETACHED scheduling policy for Async function handler (`#383 <https://github.com/ros-controls/realtime_tools/pull/383>`__).

RealtimeBox
*******************************
* ``RealtimeBox`` got renamed to ``RealtimeThreadSafeBox`` and uses real-time mutexes now (`#318 <https://github.com/ros-controls/realtime_tools/pull/318>`__, `#342 <https://github.com/ros-controls/realtime_tools/pull/342>`__).
* ``RealtimePublisher`` is updated with a new ``try_publish`` API and few methods are deprecated (`#323 <https://github.com/ros-controls/realtime_tools/pull/323>`__).
