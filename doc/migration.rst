:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Humble to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Humble (previous) and Jazzy (current) releases, where changes to user code might be necessary.

.. note::

  This list was created in June 2025 (tag 3.6.0), earlier changes may not be included.

RealtimeBox
*******************************
* ``RealtimeBox`` is deprecated. Update your code to use ``realtime_thread_safe_box.hpp`` header, and class name ``RealtimeThreadSafeBox`` instead. (`#318 <https://github.com/ros-controls/realtime_tools/pull/318>`__, `#342 <https://github.com/ros-controls/realtime_tools/pull/342>`__).
* ``RealtimePublisher`` is updated with a new ``try_publish`` API. Update your code with a local message variable and call ``try_publish`` with that variable. The old API is deprecated and will be removed in a future release. (`#323 <https://github.com/ros-controls/realtime_tools/pull/323>`__).
