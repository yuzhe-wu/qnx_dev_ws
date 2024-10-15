// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "chatter_interfaces/msg/chatter.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    std::cout << "MinimalSubscriber construction" << std::endl;
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    RCLCPP_INFO(this->get_logger(), "Using RMW implementation: %s", rmw_get_implementation_identifier());

    subscription_ = this->create_subscription<chatter_interfaces::msg::Chatter>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    std::cout << "subscription_ has been created created" << std::endl;
  }

private:
  void topic_callback(const chatter_interfaces::msg::Chatter &msg) const
  {
    // std::cout << "Received" << std::endl;
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    static uint64_t prev_count{0};
    uint64_t receive_timestamp_us =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();

    RCLCPP_INFO(this->get_logger(), "Received message count (%u): data_length = %d byte, received timestamp %d:%d, published timestamp %d:%d, latency:%d us ",
                msg.count,
                msg.data_length,
                receive_timestamp_us / 1000000,
                receive_timestamp_us % 1000000,
                msg.timestamp / 1000000,
                msg.timestamp % 1000000,
                receive_timestamp_us - msg.timestamp);

    if ((prev_count != 0) && (msg.count != prev_count + 1))
    {
      RCLCPP_WARN(this->get_logger(), "Missing message from count: %u -> %u", prev_count + 1, msg.count);
    }
    prev_count = msg.count;
  }
  rclcpp::Subscription<chatter_interfaces::msg::Chatter>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  std::cout << "Enter sub function" << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
