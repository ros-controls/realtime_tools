:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Kilted Kaiju to Lyrical Luth
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This list summarizes important changes between Kilted Kaiju (previous) and Lyrical Luth (current) releases.

- Added a static ``AsyncFunctionHandlerParams::declare()`` method to declare required ROS 2 node parameters. This moves the parameter definition within ``realtime_tools`` and simplifies node initialization in, for example ros2 controllers.
- For AsyncParams, ``exec_rate`` parameter is now read from ``update_rate`` parameter of the controller ``.yaml`` config. If not set, controller_manager update rate is used for async component update rate.
