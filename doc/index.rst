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

RealtimeTripleBuffer
---------------------
A wait-free single-producer single-consumer channel that always holds the latest value. Three slots are preallocated and the writer and reader only swap slot indices with one atomic exchange, so neither side ever blocks, fails or allocates. Intermediate values are dropped.

.. code-block:: cpp

   #include <realtime_tools/realtime_triple_buffer.hpp>

   // Preallocate every slot, e.g. with one entry per joint
   realtime_tools::RealtimeTripleBuffer<std::vector<double>> positions(std::vector<double>(n_joints));

   // Driver thread (single writer): fill the slot in place, then publish it
   auto & slot = positions.write_buffer();
   std::copy(raw.begin(), raw.end(), slot.begin());
   positions.publish();

   // Realtime thread (single reader): always get the latest published value
   if (positions.has_new_data()) { /* optional: react to fresh data */ }
   const auto & latest = positions.read();

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

   * If you care only about the latest received message, use the ``realtime_tools::RealtimeThreadSafeBox``, or the ``realtime_tools::RealtimeTripleBuffer`` if the realtime thread must never miss an update because of lock contention.
   * If you care about the intermediate messages, i.e., receiving more than one message before the realtime-thread can process it: Use the ``realtime_tools::LockFreeQueue`` and set a suitable queue size for your application, i.e., consider the maximum expected topic rate vs controller update rate.

* Passing the latest sample from a hardware driver thread to the realtime thread (one-way, e.g. in ``read()``): Use the ``realtime_tools::RealtimeTripleBuffer``. Neither the driver nor the control loop ever blocks.

* Send data from the realtime thread to the non-realtime thread for publishing data (one-way): Use the ``realtime_tools::RealtimePublisher``.

* Exchange data (two-way) between a non-realtime thread and a realtime-thread like state information, current goals etc.:

   * For primitive types like ``bool``, you can simply use ``std::atomic<bool>``, see `cppreference <https://en.cppreference.com/w/cpp/atomic/atomic>`__.
   * For all other types, when missing some data samples from RT loop is not of major importance, then use the ``realtime_tools::RealtimeThreadSafeBox`` with ``try_set`` method. In the contrary situation, it is recommended to use ``realtime_tools::LockFreeQueue`` to avoid missing any samples.

***************************************
Other classes and helper methods
***************************************

Tba.
