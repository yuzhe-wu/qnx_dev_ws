#include "rclcpp_dds/rclcpp_dds.hpp"

#include "Chatter.hpp"

using namespace dds::core;

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class MinimalSubscriber : public rclcpp_dds::DDSNode
{
public:
    explicit MinimalSubscriber()
        : DDSNode("listener")
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        std::cout << "MinimalSubscriber construction" << std::endl;
        RCLCPP_INFO(this->get_logger(), "Using RMW implementation: %s", rmw_get_implementation_identifier());

        sub_ = this->create_datareader<rti_eval::Chatter>("chatter");

        this->set_data_callback<rti_eval::Chatter>(
            sub_, [this](const rti_eval::Chatter &msg)
            {
                static uint64_t prev_count{0};
                uint64_t receive_timestamp_us =
                    std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                        .count();

                RCLCPP_INFO(this->get_logger(), "Received message count (%u): data_length = %d byte, received timestamp %d:%d, published timestamp %d:%d, latency:%d us ",
                            msg.count(),
                            msg.data_length(),
                            receive_timestamp_us / 1000000,
                            receive_timestamp_us % 1000000,
                            msg.timestamp() / 1000000,
                            msg.timestamp() % 1000000,
                            receive_timestamp_us - msg.timestamp());

                if ((prev_count != 0) && (msg.count() != prev_count + 1))
                {
                    RCLCPP_WARN(this->get_logger(), "Missing message from count: %u -> %u", prev_count + 1, msg.count());
                }
                prev_count = msg.count(); });
    }

private:
    dds::sub::DataReader<rti_eval::Chatter> sub_{nullptr};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}