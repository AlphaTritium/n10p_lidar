#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarAppNode : public rclcpp::Node
{
public:
    LidarAppNode() : Node("lidar_app_node")
    {
        // Declare parameter for scan topic
        this->declare_parameter<std::string>("scan_topic", "/scan");
        std::string scan_topic = this->get_parameter("scan_topic").as_string();

        // Create subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10, std::bind(&LidarAppNode::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "LidarAppNode started, subscribed to %s", scan_topic.c_str());
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Example: print some basic info about the scan
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Scan: ranges size=%zu, angle_min=%.2f, angle_max=%.2f",
                             msg->ranges.size(), msg->angle_min, msg->angle_max);

        // Here you can add your own processing logic, e.g.:
        // - Filter invalid ranges
        // - Convert to point cloud
        // - Detect obstacles
        // - Publish a processed scan on a different topic
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarAppNode>());
    rclcpp::shutdown();
    return 0;
}