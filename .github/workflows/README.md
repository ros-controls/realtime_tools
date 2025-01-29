## Build status

ROS 2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`master`](https://github.com/ros-controls/realtime_tools/tree/master) | [![Binary Build](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-binary-build.yml?branch=master) <br /> [![Source Build](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-source-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-source-build.yml?branch=master) | [API](http://docs.ros.org/en/rolling/p/realtime_tools/) | [realtime_tools](https://index.ros.org/p/realtime_tools/#rolling)
**Jazzy** | [`jazzy`](https://github.com/ros-controls/realtime_tools/tree/jazzy) | [![Binary Build](https://github.com/ros-controls/realtime_tools/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=jazzy)](https://github.com/ros-controls/realtime_tools/actions/workflows/jazzy-binary-build.yml?branch=jazzy) <br /> [![Source Build](https://github.com/ros-controls/realtime_tools/actions/workflows/jazzy-source-build.yml/badge.svg?branch=jazzy)](https://github.com/ros-controls/realtime_tools/actions/workflows/jazzy-source-build.yml?branch=jazzy) | [API](http://docs.ros.org/en/jazzy/p/realtime_tools/) | [realtime_tools](https://index.ros.org/p/realtime_tools/#jazzy)
**Humble** | [`humble`](https://github.com/ros-controls/realtime_tools/tree/humble) | [![Binary Build](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-binary-build.yml?branch=master) <br /> [![Source Build](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-source-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-source-build.yml?branch=master) | [API](http://docs.ros.org/en/humble/p/realtime_tools/) | [realtime_tools](https://index.ros.org/p/realtime_tools/#humble)


### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
