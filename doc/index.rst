Welcome to the documentation for realtime_tools
===============================================

Contains a set of tools that can be used from a hard realtime thread, without breaking the realtime behavior.

For more information of the ros2_control framework see `control.ros.org <https://control.ros.org/>`__.

Realtime Publisher
------------------
The ``realtime_tools::RealtimePublisher`` allows users that write C++ ros2_controllers to publish messages on a ROS topic from a hard realtime loop. The normal ROS publisher is not realtime safe, and should not be used from within the update loop of a realtime controller. The realtime publisher is a wrapper around the ROS publisher; the wrapper creates an extra non-realtime thread that publishes messages on a ROS topic. The example below shows a typical usage of the realtime publisher in the ``on_configure()`` (non-realtime method) and ``update()`` (realtime method) methods of a realtime controller:

.. code-block:: cpp

  #include <realtime_tools/realtime_publisher.hpp>

  class MyController : public controller_interface::ControllerInterface
  {
  ...
  private:
    std::shared_ptr<realtime_tools::RealtimePublisher<my_msgs::msg::MyMsg>> state_publisher_;
    std::shared_ptr<rclcpp::Publisher<my_msgs::msg::MyMsg>> s_publisher_;
  }

  controller_interface::CallbackReturn MyController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    ...
    s_publisher_ = get_node()->create_publisher<my_msgs::msg::MyMsg>(
      "~/status", rclcpp::SystemDefaultsQoS());
    state_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<ControllerStateMsg>>(s_publisher_);
    ...
  }

  controller_interface::return_type MyController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    ...
    // Publish controller state
    state_publisher_->lock();
    state_publisher_->msg_ = some_msg;
    state_publisher_->unlockAndPublish();
  }


API documentation
------------------

.. toctree::
   :maxdepth: 2

   C++ API <generated/index>


Indices and Search
==================

* :ref:`genindex`
* :ref:`search`
