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

Checking the realtime environment
=================================
``realtime_tools::check_realtime_environment(priority)`` reads the system configuration (it changes nothing) and returns a report with one line per check, prefixed with ``[OK]``, ``[WARN]`` or ``[INFO]``, plus a boolean that is ``false`` if any check warned. Each warning includes how to fix it. It checks for a PREEMPT_RT kernel, whether ``RLIMIT_RTPRIO``/``CAP_SYS_NICE`` allow the requested SCHED_FIFO priority, whether ``RLIMIT_MEMLOCK``/``CAP_IPC_LOCK`` allow ``lock_memory()``, whether the CPUs use the ``performance`` cpufreq governor, and which CPUs are isolated.

.. code-block:: cpp

   #include <realtime_tools/realtime_helpers.hpp>

   const auto [ok, report] = realtime_tools::check_realtime_environment(50);
   if (!ok) {
     RCLCPP_WARN(logger, "System is not fully configured for realtime:\n%s", report.c_str());
   }

Example output on a non-realtime system:

.. code-block:: text

   [WARN] Realtime kernel: no PREEMPT_RT kernel detected, expect high latencies. ...
   [WARN] RT priority: RLIMIT_RTPRIO is 0 but priority 50 was requested, setting SCHED_FIFO will fail. ...
   [WARN] Memory locking: RLIMIT_MEMLOCK is 8192 KiB, lock_memory() will most likely fail. ...
   [WARN] CPU governor: frequency scaling adds latency, use the 'performance' governor ...
   [INFO] Isolated CPUs: none (optionally isolate cores with the 'isolcpus' kernel parameter ...)
