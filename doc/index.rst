:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/index.rst

realtime_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Contains a set of tools that can be used from a hard realtime thread, without breaking the realtime behavior.

Exchange data between different threads
---------------------------------------
This package contains different concepts for exchanging data between different threads. In the following,
a guideline for the usage for ros2_controllers is given.

RealtimeThreadSafeBox
+++++++++++++++++++++
A Box that ensures thread safe access to the boxed contents. Access is best effort. If it can not lock it will return.

LockFreeQueue
+++++++++++++
This class provides a base implementation for lock-free queues on top of `Boost.Lockfree <https://www.boost.org/doc/libs/latest/doc/html/lockfree.html>`__ for lock-free queues with various functionalities

 * such as pushing, popping, and checking the state of the queue. It supports both single-producer
 * single-consumer (SPSC) and multiple-producer multiple-consumer (MPMC) queues.

Guidelines
++++++++++
There exist the following two typical use-cases for ros2_controllers:

* Exchange data between a non-realtime thread and a realtime-thread like state information, current goals etc.:

   * For variables of type ``bool``, you can simply use ``std::atomic<bool>``
   * For all other types, use the ``RealtimeThreadSafeBox``

* Pass command messages from topic subscribers in a non-realtime thread to the realtime-thread: Use the ``LockFreeQueue`` and set a suitable queue size for your application, i.e., the maximum expected topic rate vs controller update rate.

.. realtime_publisher
.. ++++++++++++++++++
.. include:: ../realtime_tools/doc/realtime_publisher.rst
