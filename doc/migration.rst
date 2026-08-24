:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Humble to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Humble (previous) and Jazzy (current) releases, where changes to user code might be necessary.

.. note::

  This list was created in June 2025 (tag 3.6.0), earlier changes may not be included.

RealtimeBox
*******************************
* ``RealtimeBox`` is deprecated. Update your code to use ``realtime_thread_safe_box.hpp`` header, and class name ``RealtimeThreadSafeBox`` instead. (`#318 <https://github.com/ros-controls/realtime_tools/pull/318>`__, `#342 <https://github.com/ros-controls/realtime_tools/pull/342>`__).

RealtimePublisher
*****************
* ``RealtimePublisher`` is updated with a new ``try_publish`` API.

  * Update your code with a local message variable and call ``try_publish`` with that variable. (`#323 <https://github.com/ros-controls/realtime_tools/pull/323>`__).
  * ``msg_`` variable is inaccessible now (`#421 <https://github.com/ros-controls/realtime_tools/pull/421>`__).

* ``RealtimePublisher`` is updated with a template constructor that creates the publisher internally, and the legacy constructor taking a pre-created publisher has been deprecated.

  * Instead of creating a publisher first and passing it to the constructor, you can now pass the node (or node interface/pointer), topic name, QoS, and publisher options directly to ``RealtimePublisher``. (`#573 <https://github.com/ros-controls/realtime_tools/pull/573>`__).
