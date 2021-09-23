#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include <unistd.h>

#include <ipc_test_msgs/srv/detail/ipc_test__struct.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <fastrtps/log/Log.h>
#include "ipc_test_msgs/srv/ipc_test.hpp"

using namespace std::chrono_literals;

namespace {
constexpr auto NODE_NAME = "ipc_test_client";
constexpr auto SERVICE_NAME = "ipc_test_service";
}  // namespace

rclcpp::Client<ipc_test_msgs::srv::IPCTest>::SharedFuture send_request(
    const rclcpp::Logger& logger, rclcpp::Client<ipc_test_msgs::srv::IPCTest>& client, char* num,
    const std::chrono::seconds timeout = std::chrono::seconds(0)) {
    const std::chrono::seconds time_step = 1s;
    std::chrono::seconds time_counter = 0s;
    bool is_timeout = false;
    while (!is_timeout && !client.wait_for_service(time_step)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
            return {};
        }
        if (timeout.count() != 0) {
            time_counter += time_step;
            if (time_counter >= timeout) {
                is_timeout = true;
            }
        }
        RCLCPP_INFO(logger, "IPC test service not available, waiting again...");
    }

    if (is_timeout) {
        RCLCPP_ERROR(logger, "IPC test service is not available");
        return {};
    }

    auto request = std::make_shared<ipc_test_msgs::srv::IPCTest::Request>();
    request->request_data = std::string(num) + std::string(1023 * 100, 'r');
    RCLCPP_INFO(logger, "Send request");
    return client.async_send_request(
        request, [logger](rclcpp::Client<ipc_test_msgs::srv::IPCTest>::SharedFuture result) {
            const auto& payload = result.get()->response_data;
            RCLCPP_INFO(logger, "Got response " + payload.substr(0, 10));
            RCLCPP_INFO(logger, "Response size: %lu", payload.size());
        });
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::string node_name = std::string(NODE_NAME) + (argc > 1 ? argv[1] : "");

    const auto client_node = std::make_shared<rclcpp::Node>(node_name);
    const auto logger = client_node->get_logger();
    const auto client = client_node->create_client<ipc_test_msgs::srv::IPCTest>(SERVICE_NAME);
    RCLCPP_INFO(logger, "Client started");

    const auto future = send_request(logger, *client, argv[1], std::chrono::seconds(5));
    sleep(2);
    if (future.valid()) {
        const auto res =
            rclcpp::spin_until_future_complete(client_node, future, std::chrono::minutes(1));
        if (res != rclcpp::FutureReturnCode::SUCCESS) {
            throw std::runtime_error(node_name + " Spin until complete returned error " +
                                     std::to_string(static_cast<int>(res)));
        } else {
            RCLCPP_INFO(logger, "Response handled properly");
        }
    } else {
        throw std::runtime_error(node_name + " Invalid future");
    }

    rclcpp::shutdown();
    return 0;
}
