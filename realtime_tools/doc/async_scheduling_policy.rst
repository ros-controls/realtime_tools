Async Scheduling Policies
=========================
``realtime_tools`` allows hardware interfaces to be run in a thread separate from the main ``controller_manager`` thread.

The ``AsyncFunctionHandler`` supports three distinct scheduling policies as to how the asynchronous worker thread executes its callback.

.. _scheduling-policies:

SYNCHRONIZED
------------
In ``SYNCHRONIZED`` mode, the async worker thread does not govern its own timing. Instead, it waits on a condition variable and is explicitly triggered by the main thread.

Used when the asynchronous task must run in lockstep with the main control loop, but needs to be offloaded to a separate thread to prevent blocking the real-time path. Used with slow hardware interfaces doing heavy processing.

* **Scheduling Control:** ``controller_manager`` main thread.
* **Sleep Mechanism:** Waits on a ``contition_variable`` from the main thread.

DETACHED
--------
In ``DETACHED`` mode, the async worker thread runs completely independently of the main ``controller_manager`` thread clock. It acts similarly to a standalone ROS 2 node, maintaining its own execution cycle based on a target ``exec_rate``.

Hardware interfaces that need to poll or update at a fixed, independent frequency.

* **Scheduling Control:** Internal software clock.
* **Sleep Mechanism:** Calculates the elapsed time and explicitly calls ``std::this_thread::sleep_until()``, same as ``ros2_control_node`` executable.

SLAVE
-----
In ``SLAVE`` mode, the async worker thread runs independently of the main thread, but *without any software rate-limiting or sleeping*. The handler continuously loops and immediately restarts the callback, expecting the callback function itself to block execution.

Used when ehardware interfaces that must synchronize directly with an external hardware clock. Avoids drift between control loops of manipulator controllers and ``controller_manager`` thread. This requires the hardware interface to wait on an UDP heartbeat/sync signal from hardware that a new control cycle can start.

* **Scheduling Control:** Blocking in hardware interface ``read()`` function.
* **Sleep Mechanism:** None. Relies on a blocking hardware ``read()`` to pace the thread.
