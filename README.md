realtime_tools
===========
See [control.ros.org](http://control.ros.org) and [realtime_tools](http://wiki.ros.org/realtime_tools) documentation on ros.org


## Build status
ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`rolling`](https://github.com/ros-controls/realtime_tools/tree/rolling) | [![Rolling Binary Build](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-binary-build-main.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-binary-build-main.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-semi-binary-build-main.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/rolling-semi-binary-build-main.yml?branch=master) |   | [realtime_tools](https://index.ros.org/p/realtime_tools/#rolling)
**Humble** | [`humble`](https://github.com/ros-controls/realtime_tools/tree/humble) | [![Humble Binary Build](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-binary-build-main.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-binary-build-main.yml?branch=master) <br /> [![Humble Semi-Binary Build](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-semi-binary-build-main.yml/badge.svg?branch=master)](https://github.com/ros-controls/realtime_tools/actions/workflows/humble-semi-binary-build-main.yml?branch=master) |  | [realtime_tools](https://index.ros.org/p/realtime_tools/#humble)


### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`


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
