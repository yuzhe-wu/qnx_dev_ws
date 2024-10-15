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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "chatter_interfaces/msg/chatter.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(double frequency)
      : Node("minimal_publisher"), callback_period_ms_(1000 / frequency), count_(0)
  {
    // auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    RCLCPP_INFO(this->get_logger(), "Using RMW implementation: %s", rmw_get_implementation_identifier());

    payload_ = std::string(chatter_interfaces::msg::Chatter::MAX_DATA_SIZE - 1, 'a');

    publisher_ = this->create_publisher<chatter_interfaces::msg::Chatter>("topic", 10);

    RCLCPP_INFO(this->get_logger(), "Publish to topic 'topic': rate = %f Hz (callback_period = %dms), payload size = %d", 1000.0 / callback_period_ms_, callback_period_ms_, chatter_interfaces::msg::Chatter::MAX_DATA_SIZE);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(callback_period_ms_), std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = this->publisher_->borrow_loaned_message();
    size_t copy_size = payload_.size();
    std::copy(payload_.begin(), payload_.begin() + copy_size, message.get().data.begin());
    message.get().data_length = copy_size;
    ++count_;
    message.get().count = count_;
    message.get().timestamp =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();
    RCLCPP_INFO(this->get_logger(),
                "Publishing message with count: '%u', data_length = %d byte, timestamp = %d.%d",
                message.get().count,
                message.get().data_length,
                message.get().timestamp / 1000000u,
                message.get().timestamp % 1000000u);
    publisher_->publish(std::move(message));
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<chatter_interfaces::msg::Chatter>::SharedPtr publisher_;
  size_t count_;
  int const callback_period_ms_;
  std::string payload_;
};

int main(int argc, char *argv[])
{
  int arg_processing = 1;
  bool show_usage = false;
  double frequency = 10.0;

  while (arg_processing < argc)
  {
    if (strcmp(argv[arg_processing], "-f") == 0 || strcmp(argv[arg_processing], "--frequency") == 0)
    {
      frequency = std::stod(argv[arg_processing + 1]);
      arg_processing += 2;
    }
    else if (strcmp(argv[arg_processing], "-h") == 0 || strcmp(argv[arg_processing], "--help") == 0)
    {
      std::cout << "Example application." << std::endl;
      show_usage = true;
      break;
    }
    else
    {
      std::cout << "Bad parameter." << std::endl;
      show_usage = true;
      break;
    }
  }

  if (show_usage)
  {
    std::cout << "Usage:\n"
                 "    -d, --domain       <int>   Domain ID this application will\n"
                 "                               subscribe in.  \n"
                 "                               Default: 0\n"
                 "    -s, --sample_count <int>   Number of samples to receive before\n"
                 "                               cleanly shutting down. \n"
                 "                               Default: infinite\n"
                 "    -v, --verbosity    <int>   How much debugging output to show.\n"
                 "                               Range: 0-3 \n"
                 "                               Default: 1"
                 "    -f, --frequency    <double> Publish frequency.\n"
                 "                               Range: >1.0"
                 "                               Default: 10.0"
              << std::endl;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>(frequency));
  rclcpp::shutdown();
  return 0;
}
