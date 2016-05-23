^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realtime_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
* Add feedback sending capability to RealtimeServerGoalHandle.
* Contributors: Bence Magyar, Miguel Prada

1.9.1 (2015-04-30)
------------------
* RealtimeBox: Fix member doc
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

1.9.0 (2014-05-12)
------------------
* Remove rosbuild artifacts.
* Cleaned up CMake and removed unnecessary dependencies
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

1.8.3 (2014-02-05)
------------------
* Fix linking
  The library needs to be linked against roscpp and Boost thread.
  GCC won't complain about missing symbols for a shared library,
  but other linkers (like clang's) will not accept it by default.
* Added Travis support
* Renamed manifest.xml so it doesn't brek rosdep
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Paul Mathieu

1.8.2 (2013-08-29)
------------------
* Append newline.
* Merge pull request `#4 <https://github.com/ros-controls/realtime_tools/issues/4>`_ from pal-robotics/hydro-devel
  Add realtime action server goal handle.
* Add realtime action server goal handle.
  - Factored out from PR2's implementation of the JointTrajectoryActionController.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix

1.8.1 (2013-07-29)
------------------
* Merge remote-tracking branch 'origin/master' into hydro-devel
* Merge pull request `#2 <https://github.com/ros-controls/realtime_tools/issues/2>`_ from davetcoleman/master
  Made member vars mutable in realtime buffer to allow const read
* initialize realtime_data_ and non_realtime_data_ before dereferencing and assigning to them in copy constructor
* Added comments
* Added readFromNonRT() function, overloaded assignment and copy constructor, and made mutex mutable.
* Fix typos.
* Fix build order.
* Contributors: Austin Hendrix, Dave Coleman

1.8.0 (2013-06-25)
------------------
* Version 1.8.0
* Install channelecho.py under catkin.
* adding install targets
* adding missing manifests
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* catkinizing, could still be cleaned up
* initialize correctly
* compile realtime clock into library
* new interface with time and duration
* support both condition and polling version to allow re-use of binaries in realtime; add realtime buffer to get data from non-RT into RT
* Make the realtime publisher realtime safe, without needing an rt_condition; we need the same binaries to work on both non-rt and rt.
* Make the realtime publisher realtime safe, without needing an rt_condition; we need the same binaries to work in both non-rt and rt.
* move realtime tools in ros control, and create empty constructors for handles
* Contributors: Austin Hendrix, Jonathan Bohren, Wim Meeussen, hiDOF
