// Copyright 2021 Real-Time Innovations, Inc. (RTI)
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

#include "rclcpp_dds_examples/visibility_control.h"

#include "rclcpp_dds_examples/ping/subscriber.hpp"

#include "camera/CameraImage.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using rti::camera::plain::CameraImage;

namespace rclcpp_dds_examples
{

class CameraImageSubscriberPlain
  : public rclcpp_dds_examples::PingPongSubscriber<CameraImage>
{
public:
  RCLCPP_DDS_EXAMPLES_PUBLIC
  explicit CameraImageSubscriberPlain(const rclcpp::NodeOptions & options)
  : PingPongSubscriber("camera_sub_plain", options)
  {
    this->init_test();
  }

protected:
  virtual CameraImage * alloc_sample()
  {
    return &ping_msg_;
  }

  virtual void prepare_pong(CameraImage * const pong, const uint64_t ping_ts)
  {
    pong->timestamp(ping_ts);
  }

  virtual void process_ping(
    dds::sub::LoanedSamples<CameraImage> & ping_samples,
    uint64_t & ping_timestamp)
  {
    ping_timestamp = ping_samples[0].data().timestamp();
  }

  virtual void dump_ping(
    dds::sub::LoanedSamples<CameraImage> & ping_samples,
    std::ostringstream & msg)
  {
    auto & sample = ping_samples[0].data();

    msg << "[" << sample.timestamp() << "] " << sample.format();

    for (int i = 0; i < 4; i++) {
      msg << "0x" <<
        std::hex << std::uppercase <<
        std::setfill('0') << std::setw(2) <<
        static_cast<int>(sample.data()[i]) <<
        std::nouppercase << std::dec <<
        " ";
    }
  }

  CameraImage ping_msg_;
};

}  // namespace rclcpp_dds_examples

RCLCPP_COMPONENTS_REGISTER_NODE(rclcpp_dds_examples::CameraImageSubscriberPlain)
