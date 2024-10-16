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

#ifndef RCLCPP_DDS_EXAMPLES__PING__PUBLISHER_HPP_
#define RCLCPP_DDS_EXAMPLES__PING__PUBLISHER_HPP_

#include "rclcpp_dds_examples/ping/tester.hpp"

namespace rclcpp_dds_examples
{
template<typename T>
class PingPongPublisher : public PingPongTester<T>
{
public:
  PingPongPublisher(
    const char * const name,
    const rclcpp_dds::DDSNodeOptions & options)
  : PingPongTester<T>(name, options, true /* ping */)
  {}

protected:
  // Helper function to fill in the contents of a sample
  virtual void prepare_ping(T & sample, const bool final) = 0;

  // Process received pong sample and return the timestamp
  virtual void process_pong(
    dds::sub::LoanedSamples<T> & pong_samples,
    uint64_t & pong_timestamp) = 0;

  virtual void init_test()
  {
    PingPongTester<T>::init_test();

    // Start timer to periodically check exit conditions

    if (this->test_options_.max_execution_time > 0) {
      exit_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
          if (this->is_test_complete()) {
            // Cancel timer and mark test as complete. The process will exit
            // after detecting the subscriber going offline.
            exit_timer_->cancel();
            this->test_complete();
          }
        });
    }

    RCLCPP_INFO(
      this->get_logger(),
      "ping-pong publisher ready, waiting for subscriber...");
  }

  // Overload `test_start()` to initialize test state and send an initial ping.
  virtual void test_start()
  {
    PingPongTester<T>::test_start();
    ping();
  }

  // Overload `test_stop()` to print the final computed latency
  virtual void test_stop()
  {
    if (this->test_complete_) {
      print_latency(true /* final */);
    }
    PingPongTester<T>::test_stop();
  }

  // Once the test is complete, we send a final ping with timestamp=0 to notify
  // that reader that the test is done. Once the reader is detected as offline,
  // the writer will also exit.
  virtual void test_complete()
  {
    PingPongTester<T>::test_complete();
    ping(true /* final */);
  }

  // Compute a running average of the latency measurements and log it to stdout.
  virtual void print_latency(const bool final_log = false)
  {
    std::ostringstream msg;

    if (this->count_ > 0) {
      const uint64_t avg_us = total_latency_ / (this->count_ * 2);
      const double avg_ms = static_cast<double>(avg_us) / 1000.0;
      msg << "Avg-Latency [" << this->count_ << "/" <<
        this->test_options_.max_samples << "] ";
      double avg_print = avg_ms;
      const char * avg_unit = "ms";

      if (avg_print < 1) {
        avg_print = avg_print * 1000.0;
        avg_unit = "us";
      }

      double avg_int;
      if (std::modf(avg_print, &avg_int) >= 0.001) {
        msg << std::fixed << std::setprecision(3);
      } else {
        msg << std::fixed << std::setprecision(1);
      }

      msg << avg_print << " " << avg_unit;

      if (final_log) {
        msg << " [FINAL]";
      }
    } else {
      msg << "Avg-Latency: no samples yet";
    }

    RCLCPP_INFO(this->get_logger(), msg.str().c_str());
  }

  // Send a ping message unless the test has already completed.
  virtual void ping(const bool final = false)
  {
    // Allocate a sample and prepare it for publishing
    auto sample = this->alloc_sample();

    prepare_ping(*sample, final);

    // Write the sample out
    this->writer_.write(*sample);
  }

  // Overload on_data() callback to process the pong sample and compute the
  // round-trip latency. We store the value halved to compute a running average
  // of one-way latency.
  virtual void on_data(dds::sub::LoanedSamples<T> & pong_samples)
  {
    if (!this->test_active_) {
      RCLCPP_ERROR(this->get_logger(), "pong received while test inactive");
      this->shutdown();
      return;
    }

    // Extract timestamp from pong sample.
    uint64_t pong_ts = 0;
    process_pong(pong_samples, pong_ts);

    uint64_t receive_ts = this->ts_now();
    // this test will not cope well with a drifting clock
    assert(receive_ts >= pong_ts);
    uint64_t latency = (receive_ts - pong_ts) / 2;

    // we ignore the first few samples to let things ramp up to steady state
    const bool ignored =
      static_cast<uint64_t>(this->test_options_.ignored_initial_samples) > this->count_ignored_;
    if (!ignored) {
      this->count_ += 1;
      total_latency_ += latency;

      if (last_print_ == 0 ||
        receive_ts - static_cast<uint64_t>(this->test_options_.print_interval) > last_print_)
      {
        print_latency();
        last_print_ = receive_ts;
      }
    } else {
      this->count_ignored_ += 1;
    }

    // Check if thest is complete, otherwise send another ping
    if (this->is_test_complete()) {
      test_complete();
    } else {
      ping();
    }
  }

  uint64_t total_latency_ = 0;
  uint64_t last_print_ = 0;
  uint64_t ping_ts_ = 0;
  rclcpp::TimerBase::SharedPtr exit_timer_;
};
}  // namespace rclcpp_dds_examples

#endif  // RCLCPP_DDS_EXAMPLES__PING__PUBLISHER_HPP_
