#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>
#include "rclcpp_dds/rclcpp_dds.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "Chatter.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp_dds::DDSNode
{
public:
    explicit MinimalPublisher(const rclcpp::NodeOptions &options)
        : DDSNode("talker", options)
    {
        // Create a function for when messages are to be sent.
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);

        RCLCPP_INFO(this->get_logger(), "Using RMW implementation: %s", rmw_get_implementation_identifier());

        payload_ = std::string(rti_eval::MAX_DATA_SIZE - 1, 'a');

        writer_ = this->create_datawriter<rti_eval::Chatter>("chatter");

        RCLCPP_INFO(this->get_logger(), "Publish to topic 'topic': rate = %f Hz (callback_period = %dms), payload size = %d", 1000.0 / callback_period_ms_, callback_period_ms_, rti_eval::MAX_DATA_SIZE);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(callback_period_ms_), std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        ::rti_eval::Chatter *sample = writer_->get_loan();
        size_t copy_size = payload_.size();
        std::copy(payload_.begin(), payload_.end(), sample->data().begin());

        sample->data_length(sizeof(sample->data()));
        ++count_;
        sample->count(count_);

        sample->timestamp(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

        RCLCPP_INFO(this->get_logger(),
                    "Publishing message with count: '%u', data_length = %d byte, timestamp = %d.%d",
                    sample->count(),
                    sample->data_length(),
                    sample->timestamp() / 1000000u,
                    sample->timestamp() % 1000000u);
        writer_.write(*sample);
    }

    dds::pub::DataWriter<rti_eval::Chatter> writer_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    std::string payload_;
    int const callback_period_ms_ = 1000;
};

RCLCPP_COMPONENTS_REGISTER_NODE(MinimalPublisher)
