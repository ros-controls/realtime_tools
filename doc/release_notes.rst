:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Humble to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Humble (previous) and Jazzy (current) releases.

.. note::

  This list was created in June 2025 (tag 3.6.0), earlier changes may not be included.

RealtimeBox
*******************************
* ``RealtimeBox`` got renamed to ``RealtimeThreadSafeBox`` and uses real-time mutexes now (`#318 <https://github.com/ros-controls/realtime_tools/pull/318>`__, `#342 <https://github.com/ros-controls/realtime_tools/pull/342>`__).
* ``RealtimePublisher`` is updated with a new ``try_publish`` API and few methods are deprecated (`#323 <https://github.com/ros-controls/realtime_tools/pull/323>`__).
