#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

/*
 * Completed Class 'AddTwoIntsClient' inheriting from rclcpp::Node.
 */
class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient()
        : Node("add_two_ints_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    auto send_request(int64_t a, int64_t b)
    {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        RCLCPP_INFO(this->get_logger(), "Service available, sending request...");
        return client_->async_send_request(request).future.share();
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto client_node = std::make_shared<AddTwoIntsClient>();

    int64_t a = 41;
    int64_t b = 1;
    auto result_future = client_node->send_request(a, b);

    if (rclcpp::spin_until_future_complete(client_node, result_future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = result_future.get();
        RCLCPP_INFO(client_node->get_logger(), "Result: %ld + %ld = %ld", a, b, result->sum);
    }
    else
    {
        RCLCPP_ERROR(client_node->get_logger(), "Failed to call service add_two_ints");
    }

    rclcpp::shutdown();
    return 0;
}
