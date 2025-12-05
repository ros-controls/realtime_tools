:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases, where changes to user code might be necessary.

RealtimeBox
*******************************
* ``RealtimePublisher`` is updated with a new ``try_publish`` API.

  * Update your code with a local message variable and call ``try_publish`` with that variable. (`#323 <https://github.com/ros-controls/realtime_tools/pull/323>`__).
  * ``msg_`` variable is inaccessible now (`#421 <https://github.com/ros-controls/realtime_tools/pull/421>`__).
