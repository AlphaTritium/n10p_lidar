#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <lslidar_msgs/msg/detected_objects.hpp>
#include <lslidar_msgs/msg/detected_object.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <limits>
#include <cmath>

class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor() : Node("lidar_processor")
    {
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

        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_topic_, 10, std::bind(&LidarProcessor::scan_callback, this, std::placeholders::_1));

        pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);

        if (publish_cloud_ && !cloud_topic_.empty()) {
            pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10);
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
        auto filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*raw_msg);

        for (size_t i = 0; i < filtered_msg->ranges.size(); ++i) {
            float& r = filtered_msg->ranges[i];

            if (!std::isfinite(r) || r < range_min_) {
                r = std::numeric_limits<float>::quiet_NaN();
            }
            else if (r > range_max_) {
                r = range_max_;
            }
        }

        pub_scan_->publish(*filtered_msg);

        if (publish_cloud_ && pub_cloud_) {
            sensor_msgs::msg::PointCloud2 cloud;
            convertLaserScanToPointCloud2(*filtered_msg, cloud);
            cloud.header.frame_id = target_frame_;
            pub_cloud_->publish(cloud);
        }
    }

    void convertLaserScanToPointCloud2(const sensor_msgs::msg::LaserScan& scan, sensor_msgs::msg::PointCloud2& cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            float range = scan.ranges[i];
            if (std::isfinite(range)) {
                pcl::PointXYZI point;
                double angle = scan.angle_min + i * scan.angle_increment;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                point.z = 0.0;
                if (i < scan.intensities.size()) {
                    point.intensity = scan.intensities[i];
                } else {
                    point.intensity = 0.0;
                }
                pcl_cloud->push_back(point);
            }
        }
        
        pcl::toROSMsg(*pcl_cloud, cloud);
        cloud.header = scan.header;
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
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}