RealtimePublisher
---------------------
The ``realtime_tools::RealtimePublisher`` allows users that write C++ ros2_controllers to publish messages on a ROS topic from a hard realtime loop. The normal ROS publisher is not realtime safe, and should not be used from within the update loop of a realtime controller. The realtime publisher is a wrapper around the ROS publisher; the wrapper creates an extra non-realtime thread that publishes messages on a ROS topic. The example below shows a typical usage of the realtime publisher in the ``on_configure()`` (non-realtime method) and ``update()`` (realtime method) methods of a realtime controller:

.. code-block:: cpp

  #include <realtime_tools/realtime_publisher.hpp>

  class MyController : public controller_interface::ControllerInterface
  {
  ...
  private:
    std::unique_ptr<realtime_tools::RealtimePublisher<my_msgs::msg::MyMsg>> state_publisher_;
    std::shared_ptr<rclcpp::Publisher<my_msgs::msg::MyMsg>> s_publisher_;
    my_msgs::msg::MyMsg some_msg_;
  }

  controller_interface::CallbackReturn MyController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    ...
    s_publisher_ = get_node()->create_publisher<my_msgs::msg::MyMsg>(
      "~/status", rclcpp::SystemDefaultsQoS());
    state_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<my_msgs::msg::MyMsg>>(s_publisher_);
    some_msg_.header.frame_id = params_.frame_id;
    ...
  }

  controller_interface::return_type MyController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    ...
    // Publish controller state
    some_msg_.header.stamp = get_node()->now();
    // Fill in the rest of the message
    ....
    state_publisher_->try_publish(some_msg_);
  }
