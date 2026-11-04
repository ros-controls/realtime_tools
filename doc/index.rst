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

Locking and prefaulting memory
==============================
``realtime_tools::lock_memory()`` locks all current and future pages of the process into RAM with ``mlockall``, so they are never swapped out.

Locking alone does not avoid page faults on the first use of stack and heap memory, and glibc returns freed memory to the OS by default. ``realtime_tools::lock_memory(prefault_stack_size, prefault_heap_size)`` additionally:

* configures glibc's malloc to keep freed memory in the process (``M_TRIM_THRESHOLD = -1``) and to not use ``mmap`` for large allocations (``M_MMAP_MAX = 0``). This affects the whole process;
* touches ``prefault_heap_size`` bytes of heap, so later allocations up to that size don't page fault;
* touches ``prefault_stack_size`` bytes of the **calling thread's** stack, so call it from the realtime thread.

The prefaulting is done even if locking fails. The variant without arguments keeps its previous behavior.

.. code-block:: cpp

   #include <realtime_tools/realtime_helpers.hpp>

   // In the realtime thread, before entering the control loop
   const auto [ok, message] = realtime_tools::lock_memory(512 * 1024, 64 * 1024 * 1024);
   if (!ok) {
     RCLCPP_WARN(logger, "%s", message.c_str());
   }

.. note::
   Once the memory is locked, everything the process maps later on counts against ``RLIMIT_MEMLOCK`` (unless it has ``CAP_IPC_LOCK``), so with a finite limit the prefaulted heap has to fit into it as well.
