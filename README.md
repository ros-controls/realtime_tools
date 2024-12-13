realtime_tools
===========
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![codecov](https://codecov.io/gh/ros-controls/realtime_tools/branch/master/graph/badge.svg?token=Osge1FOaAh)](https://app.codecov.io/gh/ros-controls/realtime_tools/tree/master)

Contains a set of tools that can be used from a hard realtime thread, without breaking the realtime behavior.

## Build status
ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`master`](https://github.com/ros-controls/realtime_tools/tree/master) | [![Binary Build](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-binary-build.yml?branch=master) <br /> [![Source Build](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-source-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-source-build.yml?branch=master) | [API](http://docs.ros.org/en/rolling/p/realtime_tools/)  | [realtime_tools](https://index.ros.org/p/realtime_tools/#rolling)
**Jazzy** | [`master`](https://github.com/ros-controls/realtime_tools/tree/master) | see above | [API](http://docs.ros.org/en/jazzy/p/realtime_tools/) | [realtime_tools](https://index.ros.org/p/realtime_tools/#jazzy)
**Humble** | [`humble`](https://github.com/ros-controls/realtime_tools/tree/humble) | [![Binary Build](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-binary-build.yml?branch=master) <br /> [![Source Build](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-source-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-source-build.yml?branch=master) | [API](http://docs.ros.org/en/humble/p/realtime_tools/) | [realtime_tools](https://index.ros.org/p/realtime_tools/#humble)


### Explanation of different build types

**NOTE**: There are different build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

## Publication

If you find this work useful please give credits to the authors by citing:

* S. Chitta, E. Marder-Eppstein, W. Meeussen, V. Pradeep, A. Rodríguez Tsouroukdissian, J. Bohren, D. Coleman, B. Magyar, G. Raiola, M. Lüdtke and E. Fernandez Perdomo
**"ros_control: A generic and simple control framework for ROS"**,
The Journal of Open Source Software, 2017. ([PDF](http://www.theoj.org/joss-papers/joss.00456/10.21105.joss.00456.pdf))

```
@article{ros_control,
author = {Chitta, Sachin and Marder-Eppstein, Eitan and Meeussen, Wim and Pradeep, Vijay and Rodr{\'i}guez Tsouroukdissian, Adolfo  and Bohren, Jonathan and Coleman, David and Magyar, Bence and Raiola, Gennaro and L{\"u}dtke, Mathias and Fern{\'a}ndez Perdomo, Enrique},
title = {ros\_control: A generic and simple control framework for ROS},
journal = {The Journal of Open Source Software},
year = {2017},
doi = {10.21105/joss.00456},
URL = {http://www.theoj.org/joss-papers/joss.00456/10.21105.joss.00456.pdf}
}
```
