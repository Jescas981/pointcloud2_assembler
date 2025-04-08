#include <pointcloud2_assembler/srv/assembler_query.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class AssemblerClient : public rclcpp::Node {
public:
  AssemblerClient() : Node("assembler_client") {
    // Declare parameters
    frequency_ = declare_parameter("frequency", 1.0); // Frequency in Hz (default is 1 Hz)
    
    // Create client for the service
    client_ = this->create_client<pointcloud2_assembler::srv::AssemblerQuery>("assembler_service");
    
    // Wait for the service to be available
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupt signal received, shutting down.");
        return;
      }
      RCLCPP_INFO(get_logger(), "Waiting for service...");
    }
    
    // Create publisher for assembled point clouds
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("assembled_cloud", 10);
    
    // Set up a timer to call the service at the specified frequency
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / frequency_),
      std::bind(&AssemblerClient::call_service_and_publish, this));
    
    RCLCPP_INFO(get_logger(),
                "Assembler Client Node initialized with frequency: %.2f Hz",
                frequency_);
  }

private:
  void call_service_and_publish() {
    // Create a request
    auto request = std::make_shared<pointcloud2_assembler::srv::AssemblerQuery::Request>();
    
    // Set begin time to 0 (epoch time)
    request->begin = rclcpp::Time(0);
    
    // Set end time to now
    request->end = this->now();
    
    // Call the service and set up a callback to handle the response
    auto response_received_callback = [this](
        rclcpp::Client<pointcloud2_assembler::srv::AssemblerQuery>::SharedFuture future) {
      auto result = future.get();
      if (result) {
        auto merged_cloud = result->cloud;
        cloud_pub_->publish(merged_cloud);
        RCLCPP_INFO(get_logger(), "Published point cloud with %u points", merged_cloud.width);
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to call service.");
      }
    };
    
    // Send the request asynchronously
    client_->async_send_request(request, response_received_callback);
  }

  rclcpp::Client<pointcloud2_assembler::srv::AssemblerQuery>::SharedPtr client_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double frequency_; // Frequency for periodic service calls
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AssemblerClient>());
  rclcpp::shutdown();
  return 0;
}