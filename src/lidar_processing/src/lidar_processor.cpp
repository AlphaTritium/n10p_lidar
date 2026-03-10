#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>  // For LaserProjection
#include <vector>
#include <algorithm>
#include <limits>   // for quiet_NaN

class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor() : Node("lidar_processor")
    {
        // ---- Declare and retrieve parameters ----
        // These can be changed at runtime via the parameter server or launch file.
        this->declare_parameter<std::string>("input_scan_topic", "/scan");
        this->declare_parameter<std::string>("output_scan_topic", "/scan_filtered");
        this->declare_parameter<std::string>("output_cloud_topic", "");
        this->declare_parameter<float>("range_min", 0.15);
        this->declare_parameter<float>("range_max", 12.0);
        this->declare_parameter<bool>("publish_cloud", false);
        this->declare_parameter<std::string>("target_frame", "laser");

        input_topic_   = this->get_parameter("input_scan_topic").as_string();
        output_topic_  = this->get_parameter("output_scan_topic").as_string();
        cloud_topic_   = this->get_parameter("output_cloud_topic").as_string();
        range_min_     = this->get_parameter("range_min").as_double();
        range_max_     = this->get_parameter("range_max").as_double();
        publish_cloud_ = this->get_parameter("publish_cloud").as_bool();
        target_frame_  = this->get_parameter("target_frame").as_string();

        // ---- Create subscriber for raw scans ----
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_topic_, 10, std::bind(&LidarProcessor::scan_callback, this, std::placeholders::_1));

        // ---- Create publisher for filtered scans ----
        pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);

        // ---- Optional point cloud publisher ----
        if (publish_cloud_ && !cloud_topic_.empty()) {
            pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);
            projector_ = std::make_shared<laser_geometry::LaserProjection>();
        }

        RCLCPP_INFO(this->get_logger(), "LidarProcessor started");
        RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output scan topic: %s", output_topic_.c_str());
        if (publish_cloud_) {
            RCLCPP_INFO(this->get_logger(), "  Output cloud topic: %s", cloud_topic_.c_str());
        }
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr raw_msg)
    {
        // Create a copy of the incoming scan – we'll modify the ranges.
        auto filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*raw_msg);

        // Apply range filtering to each measurement.
        for (size_t i = 0; i < filtered_msg->ranges.size(); ++i) {
            float& r = filtered_msg->ranges[i];

            // If the range is not finite (NaN, inf) or below minimum, set to NaN (invalid).
            if (!std::isfinite(r) || r < range_min_) {
                r = std::numeric_limits<float>::quiet_NaN();
            }
            // If above maximum, clip to maximum.
            else if (r > range_max_) {
                r = range_max_;
            }
        }

        // (Optional) You could also filter intensities here if needed.

        // Publish the filtered scan.
        pub_scan_->publish(*filtered_msg);

        // Optionally project the filtered scan to a point cloud.
        if (publish_cloud_ && pub_cloud_) {
            sensor_msgs::msg::PointCloud2 cloud;
            // laser_geometry::LaserProjection converts LaserScan to PointCloud2.
            projector_->projectLaser(*filtered_msg, cloud);
            // Make sure the cloud has the correct target frame (e.g., "laser").
            cloud.header.frame_id = target_frame_;
            pub_cloud_->publish(cloud);
        }
    }

    // Parameter storage
    std::string input_topic_;
    std::string output_topic_;
    std::string cloud_topic_;
    double range_min_;
    double range_max_;
    bool publish_cloud_;
    std::string target_frame_;

    // ROS 2 handles
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    std::shared_ptr<laser_geometry::LaserProjection> projector_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}