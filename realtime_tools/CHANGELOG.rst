^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realtime_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.4.0 (2025-06-13)
------------------
* Silence some deprecation warnings (`#355 <https://github.com/ros-controls/realtime_tools/issues/355>`_)
* Add new API for the RealtimePublisher (`#323 <https://github.com/ros-controls/realtime_tools/issues/323>`_)
* Add guidelines for realtimebox/queue (`#347 <https://github.com/ros-controls/realtime_tools/issues/347>`_)
* Add docs for control.ros.org (`#346 <https://github.com/ros-controls/realtime_tools/issues/346>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.3.0 (2025-05-25)
------------------
* Change default mutex of `RealtimeThreadSafeBox` and add more aliases (`#342 <https://github.com/ros-controls/realtime_tools/issues/342>`_)
* Rename RealtimeBox to RealtimeThreadsafeBox and use `prio_inherit_mutex` (`#318 <https://github.com/ros-controls/realtime_tools/issues/318>`_)
* export Boost dependency (`#336 <https://github.com/ros-controls/realtime_tools/issues/336>`_)
* Use `Boost::boost` instead of ${Boost_LIBRARIES} (`#333 <https://github.com/ros-controls/realtime_tools/issues/333>`_)
* Use target_link_libraries instead of ament_target_dependencies (`#331 <https://github.com/ros-controls/realtime_tools/issues/331>`_)
* Cleanup deprecated constructor of RTPublisher (`#330 <https://github.com/ros-controls/realtime_tools/issues/330>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.2.1 (2025-05-04)
------------------
* Fix the deadlock in the destructor of RealtimePublisher (`#320 <https://github.com/ros-controls/realtime_tools/issues/320>`_)
* Contributors: Sai Kishor Kothakota

4.2.0 (2025-04-06)
------------------
* Fix realtime publisher race condition upon initialization (`#309 <https://github.com/ros-controls/realtime_tools/issues/309>`_)
* Move the package to a subfolder (`#295 <https://github.com/ros-controls/realtime_tools/issues/295>`_)
* Use ros2_control_cmake (`#293 <https://github.com/ros-controls/realtime_tools/issues/293>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

4.1.0 (2025-02-19)
------------------
* [RTPublisher] Use NON_POLLING as default for the realtime pubisher  (`#280 <https://github.com/ros-controls/realtime_tools/issues/280>`_)
* Bump version of pre-commit hooks (`#276 <https://github.com/ros-controls/realtime_tools/issues/276>`_)
* Contributors: Sai Kishor Kothakota, github-actions[bot]

4.0.0 (2025-01-30)
------------------
* Remove deprecated code (`#262 <https://github.com/ros-controls/realtime_tools/issues/262>`_)
* Remove RealtimeClock (`#261 <https://github.com/ros-controls/realtime_tools/issues/261>`_)
* Install boost on jazzy as well (`#273 <https://github.com/ros-controls/realtime_tools/issues/273>`_)
* Use ABI workflow from ros2_control_ci (`#264 <https://github.com/ros-controls/realtime_tools/issues/264>`_)
* Improve has_realtime_kernel method (`#260 <https://github.com/ros-controls/realtime_tools/issues/260>`_)
* Branch for jazzy (`#263 <https://github.com/ros-controls/realtime_tools/issues/263>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

3.2.0 (2025-01-29)
------------------
* Add Lock-free Queue (`#253 <https://github.com/ros-controls/realtime_tools/issues/253>`_)
* Use humble branch of control_toolbox repo (`#258 <https://github.com/ros-controls/realtime_tools/issues/258>`_)
* Avoid to include windows.h in realtime_helpers.hpp (`#255 <https://github.com/ros-controls/realtime_tools/issues/255>`_)
* Bump version of pre-commit hooks (`#251 <https://github.com/ros-controls/realtime_tools/issues/251>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota, Silvio Traversaro, github-actions[bot]

3.1.0 (2024-12-29)
------------------
* Fix ref for scheduled build (`#248 <https://github.com/ros-controls/realtime_tools/issues/248>`_)
* Add realtime priority inheritance mutexes (`#197 <https://github.com/ros-controls/realtime_tools/issues/197>`_)
* Deprecate RealtimeClock class (`#244 <https://github.com/ros-controls/realtime_tools/issues/244>`_)
* Remove wrong comments (`#240 <https://github.com/ros-controls/realtime_tools/issues/240>`_)
* Add `get_thread` method to `RealtimePublisher` (`#228 <https://github.com/ros-controls/realtime_tools/issues/228>`_)
* sleep after starting thread to fix flaky tests (`#235 <https://github.com/ros-controls/realtime_tools/issues/235>`_)
* Fix the badges in the readme (`#234 <https://github.com/ros-controls/realtime_tools/issues/234>`_)
* First step towards modernizing the rt publisher (`#210 <https://github.com/ros-controls/realtime_tools/issues/210>`_)
* Add missing change to .hpp for realtime_clock (`#227 <https://github.com/ros-controls/realtime_tools/issues/227>`_)
* Remove duplicate downstream build workflow (`#230 <https://github.com/ros-controls/realtime_tools/issues/230>`_)
* Changes after branching humble (`#217 <https://github.com/ros-controls/realtime_tools/issues/217>`_)
* CI downstream build (`#214 <https://github.com/ros-controls/realtime_tools/issues/214>`_)
* Contributors: Christoph Fröhlich, Lennart Nachtigall, Patrick Roncagliolo, Sai Kishor Kothakota

3.0.0 (2024-12-03)
------------------
* remove unused state\_ field (`#215 <https://github.com/ros-controls/realtime_tools/issues/215>`_)
* Bump version of pre-commit hooks (`#213 <https://github.com/ros-controls/realtime_tools/issues/213>`_)
* Add job for clang build (`#207 <https://github.com/ros-controls/realtime_tools/issues/207>`_)
* Add support to parse multiple cores for setting CPU affinity (`#208 <https://github.com/ros-controls/realtime_tools/issues/208>`_)
* Adapt API style of lock_memory to match the one of the other functions (`#209 <https://github.com/ros-controls/realtime_tools/issues/209>`_)
* Move the header files to `.hpp` extension (`#206 <https://github.com/ros-controls/realtime_tools/issues/206>`_)
* Use windows CI build (`#204 <https://github.com/ros-controls/realtime_tools/issues/204>`_)
* Add downstream build CI job (`#201 <https://github.com/ros-controls/realtime_tools/issues/201>`_)
* Fix RealtimeBox broken API + `realtime_box_best_effort.h` proper deprecation (`#202 <https://github.com/ros-controls/realtime_tools/issues/202>`_)
* Replace existing RealtimeBox implementation with RealtimeBoxBestEffort implementation (`#146 <https://github.com/ros-controls/realtime_tools/issues/146>`_)
* Overloading the set_thread_affinity method for Windows compatibility (`#193 <https://github.com/ros-controls/realtime_tools/issues/193>`_)
* Remove iron workflows and update readme (`#184 <https://github.com/ros-controls/realtime_tools/issues/184>`_)
* Add method to get the current callback time and period (`#192 <https://github.com/ros-controls/realtime_tools/issues/192>`_)
* Use pthread_setaffinity_np for setting affinity rather than sched_setaffinity (`#190 <https://github.com/ros-controls/realtime_tools/issues/190>`_)
* Add the same compile flags as with ros2_controllers and fix errors (`#185 <https://github.com/ros-controls/realtime_tools/issues/185>`_)
* Contributors: Christoph Fröhlich, Gilmar Correia, Lennart Nachtigall, Sai Kishor Kothakota, github-actions[bot]

2.8.1 (2024-11-05)
------------------
* Fix libcap-dev dependency (`#189 <https://github.com/ros-controls/realtime_tools/issues/189>`_)
  Update libcap-dev dependency
* Contributors: Sai Kishor Kothakota

2.8.0 (2024-11-04)
------------------
* Add methods to set the thread affinity (`#186 <https://github.com/ros-controls/realtime_tools/issues/186>`_)
* Fix pre-commit of `#187 <https://github.com/ros-controls/realtime_tools/issues/187>`_ (`#188 <https://github.com/ros-controls/realtime_tools/issues/188>`_)
* Removing thread_priority.hpp warning for Windows Systems (`#187 <https://github.com/ros-controls/realtime_tools/issues/187>`_)
* Fix build error with clang (`#183 <https://github.com/ros-controls/realtime_tools/issues/183>`_)
* Bump version of pre-commit hooks (`#182 <https://github.com/ros-controls/realtime_tools/issues/182>`_)
* [AsyncFunctionHandler] return execution time in nanoseconds (`#181 <https://github.com/ros-controls/realtime_tools/issues/181>`_)
* Rename thread_priority to realtime_helpers header (`#178 <https://github.com/ros-controls/realtime_tools/issues/178>`_)
* Include <functional> in realtime_box_best_effort (`#173 <https://github.com/ros-controls/realtime_tools/issues/173>`_)
* Bump version of pre-commit hooks (`#171 <https://github.com/ros-controls/realtime_tools/issues/171>`_)
* Add fixes to the code to work for the windows systems (`#180 <https://github.com/ros-controls/realtime_tools/issues/180>`_)
* Update thread_priority.cpp (`#170 <https://github.com/ros-controls/realtime_tools/issues/170>`_)
* Add a helper method to lock the pages of memory in the RAM (`#175 <https://github.com/ros-controls/realtime_tools/issues/175>`_)
* Contributors: Christoph Fröhlich, Felix Exner (fexner), Gilmar Correia, Luca Della Vedova, Sai Kishor Kothakota, github-actions[bot]

2.7.0 (2024-10-29)
------------------
* [AsyncFunctionHandler] Add exception handling (`#172 <https://github.com/ros-controls/realtime_tools/issues/172>`_)
* Bump version of pre-commit hooks (`#169 <https://github.com/ros-controls/realtime_tools/issues/169>`_)
* Contributors: Sai Kishor Kothakota, github-actions[bot]

2.6.0 (2024-08-14)
------------------
* Add Async Function Handler  (`#168 <https://github.com/ros-controls/realtime_tools/issues/168>`_)
* Bump version of pre-commit hooks (`#167 <https://github.com/ros-controls/realtime_tools/issues/167>`_)
* [CI] Add jazzy :rocket:  (`#165 <https://github.com/ros-controls/realtime_tools/issues/165>`_)
* [CI] Specify runner/container images (`#163 <https://github.com/ros-controls/realtime_tools/issues/163>`_)
* Add custom rosdoc2 config (`#161 <https://github.com/ros-controls/realtime_tools/issues/161>`_)
* Added a new implementation of the RealtimeBox with added best effort behaviour (`#139 <https://github.com/ros-controls/realtime_tools/issues/139>`_)
* [CI] Code coverage and pre-commit (`#154 <https://github.com/ros-controls/realtime_tools/issues/154>`_)
* [CI] Use reusable workflows and matrix strategy (`#151 <https://github.com/ros-controls/realtime_tools/issues/151>`_)
* Bump ros-tooling/action-ros-ci from 0.3.5 to 0.3.6 (`#148 <https://github.com/ros-controls/realtime_tools/issues/148>`_)
* Fix RHEL workflows (`#144 <https://github.com/ros-controls/realtime_tools/issues/144>`_)
* update unlock method to also include the part of the NON_POLLING (`#142 <https://github.com/ros-controls/realtime_tools/issues/142>`_)
* Bump actions/upload-artifact from 4.1.0 to 4.2.0 (`#143 <https://github.com/ros-controls/realtime_tools/issues/143>`_)
* [CI] Add debian workflows (`#145 <https://github.com/ros-controls/realtime_tools/issues/145>`_)
* Test fix: initialize the global context to avoid runtime_error upon destruction (`#128 <https://github.com/ros-controls/realtime_tools/issues/128>`_)
* Contributors: Christoph Fröhlich, Felix Exner (fexner), Lennart Nachtigall, Sai Kishor Kothakota, dependabot[bot], github-actions[bot]

2.5.0 (2023-01-28)
------------------
* Fix overriding of install (`#105 <https://github.com/ros-controls/realtime_tools/issues/105>`_)
* Add missing test dependency on ament_cmake_gmock (`#94 <https://github.com/ros-controls/realtime_tools/issues/94>`_)
* Contributors: Bence Magyar, Denis Štogl, Scott K Logan, Tyler Weaver

2.4.0 (2022-11-19)
------------------
* bug fix for RealtimePublisher with NON_POLLING (`#85 <https://github.com/ros-controls/realtime_tools/issues/85>`_)
* ci: :construction_worker: update rhel container (`#92 <https://github.com/ros-controls/realtime_tools/issues/92>`_)
* Make `thread_priority` a shared library (`#91 <https://github.com/ros-controls/realtime_tools/issues/91>`_)
* Contributors: Andy Zelenak, Jaron Lundwall, Yoav Fekete, Denis Štogl

2.3.0 (2022-08-26)
------------------
* Fix source-based branch names (`#90 <https://github.com/ros-controls/realtime_tools/issues/90>`_)
* Fix formatting (`#89 <https://github.com/ros-controls/realtime_tools/issues/89>`_)
* fix cpplint errors
* Add a library to set thread priority
* Add CI setup as for ros2_control repository.
* Contributors: Andy Zelenak, Bence Magyar, Denis Štogl

2.2.0 (2021-11-03)
------------------
* Adding new reset() function for Issue-247.
* Contributors: bailaC

2.1.1 (2021-03-21)
------------------
* Fix deprecation warnings when constructing rclcpp::Duration
  Since https://github.com/ros2/rclcpp/pull/1432 (upcoming in Galactic), we should not initialize with a single integer
  as the units are ambiguous.
* fix the mis-type error.
* Fix uninitialized variable
* Contributors: Jacob Perron, Victor Lopez, seanyen

2.1.0 (2020-07-03)
------------------
* fix msbuild warning
* address linter failures
* enable linters
* avoid deprecations
* Realtime server goal thread handle safety + additional warning fixes (`#2 <https://github.com/ros-controls/realtime_tools/issues/2>`_) (`#57 <https://github.com/ros-controls/realtime_tools/issues/57>`_)
  * Made code thread safe, fixed warnings with repeated aborts/success/cancels
  Fixed -reorder warning
  Early return
  * removed atomic
  * removed unneeded header
* use template instead
* use std::atomic instead of volatile
* Contributors: Karsten Knese, Yutaka Kondo, ddengster

2.0.0 (2019-09-09)
------------------
* Add test_depend ament_cmake_gmock
* Update CI for dashing
* Add sloretz as another author
* Typename and typos in RTPublisher
* Shorter type names
* Port RealtimeServerGoalHandle to ROS 2
* Port RealtimePublisher to ROS 2
  Use test_msgs instead of std_msgs
* Box and buffer work in ROS 2 unchanged
* Port RealtimeClock to ROS 2
* Remove actionlib definitions
* Contributors: Shane Loretz

1.14.0 (2019-07-22)
-------------------
* Undo action typedef changes
* Remove boost
* Clean up includes in `realtime_tools` namespace
* Switch to gmock
* Remove TARGET check on tests
* Add unit test for RealtimeServerGoalHandle, RealtimePublisher, RealtimeClock, RealtimeBuffer, RealtimeBox
* Fix race where first message won't get published
* Clean up dependencies and package.xml
* Contributors: Shane Loretz

1.13.1 (2019-02-14)
-------------------
* Fix actionlib regression
* Contributors: Bence Magyar

1.13.0 (2019-02-11)
-------------------
* Update readme
* use this_thread::sleep_for instead of usleep (`#32 <https://github.com/ros-controls/realtime_tools/issues/32>`_)
* specify RUNTIME DESTINATION for libraries (`#33 <https://github.com/ros-controls/realtime_tools/issues/33>`_)
  needed for exporting DLLs on Windows
* Made RealtimeBuffer's copy-constructor const
* Contributors: Bence Magyar, Gennaro Raiola, James Xu, Mathias Lüdtke, Matt Reynolds

1.12.0 (2018-05-19)
-------------------
* Add RealtimePublisherSharedPtr<T>
* boost::shared_ptr -> std::shared_ptr
* Contributors: Bence Magyar

1.11.0 (2017-11-06)
-------------------
* Updated RT goal handle to handle cancel requests (`#22 <https://github.com/ros-controls/realtime_tools/issues/22>`_)
* switch to industrial_ci (`#20 <https://github.com/ros-controls/realtime_tools/issues/20>`_)
* Contributors: Mathias Lüdtke, Nick Lamprianidis

1.10.0 (2017-06-28)
-------------------
* Added constructor in RTB for objects without default constructor
* Add feedback sending capability to RealtimeServerGoalHandle.
* Contributors: Bence Magyar, Aris Synodinos, Miguel Prada, graiola

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
