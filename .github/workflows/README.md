## Build status

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** <br />**Jazzy**<br />**Iron** <br />**Humble** | [`master`](https://github.com/ros-controls/realtime_tools/tree/master) | [![Binary Build](https://github.com/ros-controls/realtime_tools/actions/workflows/binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/binary-build.yml?branch=master) <br /> [![Source Build](https://github.com/ros-controls/realtime_tools/actions/workflows/source-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/source-build.yml?branch=master) | [API](http://docs.ros.org/en/rolling/p/realtime_tools/) | [realtime_tools](https://index.ros.org/p/realtime_tools/#rolling)



### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
