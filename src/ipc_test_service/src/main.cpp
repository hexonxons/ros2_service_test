#include <string>

//#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "ipc_test_msgs/srv/ipc_test.hpp"

namespace {
constexpr auto NODE_NAME = "ipc_test_service";
}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    const auto service_node = std::make_shared<rclcpp::Node>(NODE_NAME);
    const auto logger = service_node->get_logger();

    size_t requests_cnt = 0;

    const auto service = service_node->create_service<ipc_test_msgs::srv::IPCTest>(
        NODE_NAME, [&](const std::shared_ptr<ipc_test_msgs::srv::IPCTest::Request> request,
                       std::shared_ptr<ipc_test_msgs::srv::IPCTest::Response> response) {
            RCLCPP_INFO(logger, "Got %lu request from client: " + request->request_data.substr(0, 5), ++requests_cnt);

            // Build response splitted by 65384 block size
            auto response_data = request->request_data.substr(0, 5)
                + std::string(65379, 'a')
                + std::string(65384, 'b')
                + std::string(65384, 'c')
                + std::string(65384, 'd')
                + std::string(65384, 'e')
                + std::string(65384, 'f')
                + std::string(65384, 'j')
                + std::string(65384, 'h')
                + std::string(65384, 'i')
                + std::string(65384, 'j')
                + std::string(65384, 'k')
                + std::string(65384, 'l')
                + std::string(65384, 'm')
                + std::string(65384, 'n')
                + std::string(65384, 'o')
                + std::string(65384, 'p')
                + std::string(65384, 'q')
                + std::string(65384, 'r')
                + std::string(65384, 's')
                + std::string(65384, 't')
                + std::string(65384, 'u')
                + std::string(65384, 'v')
                + std::string(65384, 'w')
                + std::string(65384, 'x')
                + std::string(65384, 'y')
                + std::string(65384, 'z');
            // Fragment size = 65384
            response->response_data = response_data + std::string(1023 * 1023 * 10 - response_data.length(), 'a');
            RCLCPP_INFO(logger, "Send %lu response to client: " + request->request_data.substr(0, 5), requests_cnt);
        });

    RCLCPP_INFO(logger, "%s started", NODE_NAME);
    rclcpp::spin(service_node);
    rclcpp::shutdown();
}
