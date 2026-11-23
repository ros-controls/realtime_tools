:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/index.rst

##############
realtime_tools
##############

Contains a set of tools that can be used from a hard realtime thread, without breaking the realtime behavior.

***************************************
Exchange data between different threads
***************************************
This package contains different concepts for exchanging data between different threads. In the following, a guideline for the usage for ros2_controllers is given.

Provided concepts
=================

RealtimeThreadSafeBox
---------------------
A Box that ensures thread-safe access to the boxed contents. Access is best effort. If it can not lock it will return.

LockFreeQueue
---------------------
This class provides a base implementation for lock-free queues on top of `Boost.Lockfree <https://www.boost.org/doc/libs/latest/doc/html/lockfree.html>`__ for lock-free queues with various functionalities, such as pushing, popping, and checking the state of the queue. It supports both single-producer single-consumer (SPSC) and multiple-producer multiple-consumer (MPMC) queues.

.. RealtimePublisher
.. ---------------------
.. include:: ../realtime_tools/doc/realtime_publisher.rst

Guidelines
=================
There exist the following typical use-cases for ros2_controllers:

* Passing command messages from topic subscribers in a non-realtime thread to the realtime-thread (one-way).

   * If you care only about the latest received message, use the ``realtime_tools::RealtimeThreadSafeBox``.
   * If you care about the intermediate messages, i.e., receiving more than one message before the realtime-thread can process it: Use the ``realtime_tools::LockFreeQueue`` and set a suitable queue size for your application, i.e., consider the maximum expected topic rate vs controller update rate.

* Send data from the realtime thread to the non-realtime thread for publishing data (one-way): Use the ``realtime_tools::RealtimePublisher``.

* Exchange data (two-way) between a non-realtime thread and a realtime-thread like state information, current goals etc.:

   * For primitive types like ``bool``, you can simply use ``std::atomic<bool>``, see `cppreference <https://en.cppreference.com/w/cpp/atomic/atomic>`__.
   * For all other types, when missing some data samples from RT loop is not of major importance, then use the ``realtime_tools::RealtimeThreadSafeBox`` with ``try_set`` method. In the contrary situation, it is recommended to use ``realtime_tools::LockFreeQueue`` to avoid missing any samples.

***************************************
Other classes and helper methods
***************************************

RealtimeLogger
=================
The ``RCLCPP_*`` logging macros are not realtime-safe: they format into heap memory, take locks in the output handlers and publish to ``/rosout``.
``realtime_tools::RealtimeLogger`` wraps an ``rclcpp::Logger``. The realtime thread formats each message into a preallocated slot of a wait-free queue, and a background thread forwards it to the wrapped logger with the original severity and call-site location. Console output, log files, ``/rosout`` and logger levels therefore work as with ``RCLCPP_*``.

The ``RT_LOG_<SEVERITY>[_<VARIANT>]`` macros mirror the ``RCLCPP_*`` ones, taking a ``RealtimeLogger`` as first argument. Severities: ``DEBUG``, ``INFO``, ``WARN``, ``ERROR``, ``FATAL``. Variants: none, ``_ONCE``, ``_SKIPFIRST``, ``_EXPRESSION``, ``_FUNCTION``, ``_THROTTLE`` and ``_SKIPFIRST_THROTTLE``. There are no ``_STREAM`` variants, because ``std::stringstream`` allocates.

.. code-block:: cpp

   #include "realtime_tools/realtime_logger.hpp"

   // in on_configure() (non-realtime)
   rt_logger_ = std::make_unique<realtime_tools::RealtimeLogger>(get_node()->get_logger());

   // in update() (realtime)
   RT_LOG_WARN_THROTTLE(*rt_logger_, *get_node()->get_clock(), 1000, "Joint %s exceeded limit: %f", name, value);

Limitations:

* One producer thread per ``RealtimeLogger`` instance.
* At most ``RealtimeLogger::kQueueCapacity`` messages can be pending. Further messages are dropped instead of blocking, and ``get_dropped_count()`` counts them.
* Messages are truncated to ``RealtimeLogger::kMaxMessageSize - 1`` characters.
* Formatting uses ``vsnprintf``, which can allocate for extreme float widths or precisions in glibc. The clock passed to the ``_THROTTLE`` variants must be realtime-safe to read.
