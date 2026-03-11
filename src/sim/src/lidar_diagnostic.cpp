#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <lslidar_msgs/msg/detected_objects.hpp>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>

class LidarDiagnosticNode : public rclcpp::Node
{
public:
    LidarDiagnosticNode() : Node("lidar_diagnostic")
    {
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("cloud_topic", "/lslidar_point_cloud");
        this->declare_parameter<std::string>("objects_topic", "/detected_objects");
        this->declare_parameter<float>("min_range", 0.15);
        this->declare_parameter<float>("max_range", 12.0);
        this->declare_parameter<int>("expected_beams", 720);
        
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        cloud_topic_ = this->get_parameter("cloud_topic").as_string();
        objects_topic_ = this->get_parameter("objects_topic").as_string();
        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        expected_beams_ = this->get_parameter("expected_beams").as_int();
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10, std::bind(&LidarDiagnosticNode::scanCallback, this, std::placeholders::_1));
        
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, 10, std::bind(&LidarDiagnosticNode::cloudCallback, this, std::placeholders::_1));
        
        objects_sub_ = this->create_subscription<lslidar_msgs::msg::DetectedObjects>(
            objects_topic_, 10, std::bind(&LidarDiagnosticNode::objectsCallback, this, std::placeholders::_1));
        
        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        centroid_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_objects_centroid", 10);
        bbox_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bounding_boxes", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/lidar_status", 10);
        
        diag_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LidarDiagnosticNode::publishDiagnostics, this));
        
        last_scan_time_ = this->now();
        last_cloud_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "LiDAR Diagnostic Node initialized");
    }

private:
    struct ScanStats {
        int total_points = 0;
        int valid_points = 0;
        double min_detected_range = 100.0;
        double max_detected_range = 0.0;
        double avg_intensity = 0.0;
        double angular_resolution = 0.0;
        int beams_with_data = 0;
        double scan_frequency = 0.0;
    };
    
    struct CloudStats {
        int total_points = 0;
        int points_in_range = 0;
        double min_x = 100.0, max_x = -100.0;
        double min_y = 100.0, max_y = -100.0;
        double min_z = 100.0, max_z = -100.0;
        double point_density = 0.0;
    };
    
    struct ObjectStats {
        int object_count = 0;
        std::vector<double> distances;
        std::vector<double> sizes;
    };
    
    ScanStats scan_stats_;
    CloudStats cloud_stats_;
    ObjectStats object_stats_;
    rclcpp::Time last_scan_time_;
    rclcpp::Time last_cloud_time_;
    
    std::string scan_topic_;
    std::string cloud_topic_;
    std::string objects_topic_;
    double min_range_;
    double max_range_;
    int expected_beams_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_sub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto now = this->now();
        double dt = (now - last_scan_time_).seconds();
        last_scan_time_ = now;
        
        scan_stats_.total_points = msg->ranges.size();
        scan_stats_.valid_points = 0;
        scan_stats_.min_detected_range = 100.0;
        scan_stats_.max_detected_range = 0.0;
        scan_stats_.beams_with_data = 0;
        
        double intensity_sum = 0.0;
        for (float range : msg->ranges) {
            if (std::isfinite(range) && range > 0.0) {
                scan_stats_.valid_points++;
                if (range < scan_stats_.min_detected_range) {
                    scan_stats_.min_detected_range = range;
                }
                if (range > scan_stats_.max_detected_range) {
                    scan_stats_.max_detected_range = range;
                }
                scan_stats_.beams_with_data++;
            }
        }
        
        if (!msg->intensities.empty()) {
            for (float intensity : msg->intensities) {
                intensity_sum += intensity;
            }
            scan_stats_.avg_intensity = intensity_sum / msg->intensities.size();
        }
        
        scan_stats_.angular_resolution = (msg->angle_max - msg->angle_min) / msg->ranges.size();
        scan_stats_.scan_frequency = dt > 0 ? 1.0 / dt : 0.0;
        
        checkBlindZone(msg);
    }
    
    void checkBlindZone(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int close_range_count = 0;
        double blind_zone_threshold = min_range_ + 0.1;
        
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range < blind_zone_threshold && range > 0.0) {
                close_range_count++;
            }
        }
        
        double close_ratio = static_cast<double>(close_range_count) / msg->ranges.size();
        
        if (close_ratio < 0.01 && scan_stats_.valid_points > 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "⚠️ BLIND ZONE DETECTED: Very few points in close range (%.1f%%). "
                "Objects too close may not be detected.", close_ratio * 100);
            
            std_msgs::msg::String status_msg;
            status_msg.data = "WARNING: Object too close - entering blind zone";
            status_pub_->publish(status_msg);
        }
    }
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto now = this->now();
        double dt = (now - last_cloud_time_).seconds();
        last_cloud_time_ = now;
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        
        cloud_stats_.total_points = cloud->points.size();
        cloud_stats_.points_in_range = 0;
        cloud_stats_.min_x = 100.0; cloud_stats_.max_x = -100.0;
        cloud_stats_.min_y = 100.0; cloud_stats_.max_y = -100.0;
        cloud_stats_.min_z = 100.0; cloud_stats_.max_z = -100.0;
        
        for (const auto& pt : cloud->points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                cloud_stats_.points_in_range++;
                
                double range_2d = std::sqrt(pt.x * pt.x + pt.y * pt.y);
                if (range_2d >= min_range_ && range_2d <= max_range_) {
                    cloud_stats_.min_x = std::min(cloud_stats_.min_x, static_cast<double>(pt.x));
                    cloud_stats_.max_x = std::max(cloud_stats_.max_x, static_cast<double>(pt.x));
                    cloud_stats_.min_y = std::min(cloud_stats_.min_y, static_cast<double>(pt.y));
                    cloud_stats_.max_y = std::max(cloud_stats_.max_y, static_cast<double>(pt.y));
                    cloud_stats_.min_z = std::min(cloud_stats_.min_z, static_cast<double>(pt.z));
                    cloud_stats_.max_z = std::max(cloud_stats_.max_z, static_cast<double>(pt.z));
                }
            }
        }
        
        double area = (cloud_stats_.max_x - cloud_stats_.min_x) * (cloud_stats_.max_y - cloud_stats_.min_y);
        cloud_stats_.point_density = area > 0 ? cloud_stats_.points_in_range / area : 0.0;
    }
    
    void objectsCallback(const lslidar_msgs::msg::DetectedObjects::SharedPtr msg)
    {
        object_stats_.object_count = msg->objects.size();
        object_stats_.distances.clear();
        object_stats_.sizes.clear();
        
        visualization_msgs::msg::MarkerArray bbox_array;
        bbox_array.markers.reserve(msg->objects.size());
        
        for (size_t i = 0; i < msg->objects.size(); i++) {
            const auto& obj = msg->objects[i];
            
            double distance = std::sqrt(obj.x * obj.x + obj.y * obj.y);
            object_stats_.distances.push_back(distance);
            
            geometry_msgs::msg::PointStamped centroid_msg;
            centroid_msg.header = msg->header;
            centroid_msg.point.x = obj.x;
            centroid_msg.point.y = obj.y;
            centroid_msg.point.z = obj.z;
            centroid_pub_->publish(centroid_msg);
            
            visualization_msgs::msg::Marker bbox_marker;
            bbox_marker.header = msg->header;
            bbox_marker.ns = "bounding_boxes";
            bbox_marker.id = i;
            bbox_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            bbox_marker.action = visualization_msgs::msg::Marker::ADD;
            
            double size_x = 0.3;
            double size_y = 0.3;
            double size_z = 0.5;
            
            bbox_marker.points.push_back(createPoint(obj.x - size_x/2, obj.y - size_y/2, obj.z));
            bbox_marker.points.push_back(createPoint(obj.x + size_x/2, obj.y - size_y/2, obj.z));
            bbox_marker.points.push_back(createPoint(obj.x + size_x/2, obj.y + size_y/2, obj.z));
            bbox_marker.points.push_back(createPoint(obj.x - size_x/2, obj.y + size_y/2, obj.z));
            bbox_marker.points.push_back(bbox_marker.points[0]);
            
            bbox_marker.scale.x = 0.02;
            bbox_marker.color.r = 0.0;
            bbox_marker.color.g = 1.0;
            bbox_marker.color.b = 0.0;
            bbox_marker.color.a = 1.0;
            bbox_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            
            bbox_array.markers.push_back(bbox_marker);
        }
        
        bbox_pub_->publish(bbox_array);
    }
    
    geometry_msgs::msg::Point createPoint(double x, double y, double z)
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }
    
    void publishDiagnostics()
    {
        diagnostic_msgs::msg::DiagnosticArray diag_array;
        diag_array.header.stamp = this->now();
        
        diagnostic_msgs::msg::DiagnosticStatus scan_status;
        scan_status.name = "LiDAR Scan Performance";
        scan_status.hardware_id = "N10_P_LiDAR";
        
        scan_status.values.push_back(createKeyValue("Total Beams", std::to_string(scan_stats_.total_points)));
        scan_status.values.push_back(createKeyValue("Valid Returns", std::to_string(scan_stats_.valid_points)));
        scan_status.values.push_back(createKeyValue("Data Coverage", 
            fmt::format("{:.1f}%", 100.0 * scan_stats_.beams_with_data / (scan_stats_.total_points > 0 ? scan_stats_.total_points : 1))));
        scan_status.values.push_back(createKeyValue("Angular Resolution", 
            fmt::format("{:.4f}° ({:.2f} mrad)", 
                scan_stats_.angular_resolution * 180.0 / M_PI,
                scan_stats_.angular_resolution * 1000.0)));
        scan_status.values.push_back(createKeyValue("Min Range Detected", 
            fmt::format("{:.3f} m", scan_stats_.min_detected_range)));
        scan_status.values.push_back(createKeyValue("Max Range Detected", 
            fmt::format("{:.3f} m", scan_stats_.max_detected_range)));
        scan_status.values.push_back(createKeyValue("Configured Range", 
            fmt::format("{:.2f} - {:.2f} m", min_range_, max_range_)));
        scan_status.values.push_back(createKeyValue("Scan Frequency", 
            fmt::format("{:.1f} Hz", scan_stats_.scan_frequency)));
        scan_status.values.push_back(createKeyValue("Avg Intensity", 
            fmt::format("{:.2f}", scan_stats_.avg_intensity)));
        
        double theoretical_temp = 25.0 + (scan_stats_.avg_intensity / 255.0) * 40.0;
        scan_status.values.push_back(createKeyValue("Estimated Temperature", 
            fmt::format("{:.1f}°C (simulated)", theoretical_temp)));
        
        scan_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        scan_status.message = "LiDAR scan operating normally";
        
        diag_array.status.push_back(scan_status);
        
        diagnostic_msgs::msg::DiagnosticStatus cloud_status;
        cloud_status.name = "Point Cloud Statistics";
        cloud_status.hardware_id = "N10_P_LiDAR";
        
        cloud_status.values.push_back(createKeyValue("Total Points", std::to_string(cloud_stats_.total_points)));
        cloud_status.values.push_back(createKeyValue("Points in Range", std::to_string(cloud_stats_.points_in_range)));
        cloud_status.values.push_back(createKeyValue("Point Density", 
            fmt::format("{:.2f} pts/m²", cloud_stats_.point_density)));
        cloud_status.values.push_back(createKeyValue("FOV X", 
            fmt::format("{:.2f} m", cloud_stats_.max_x - cloud_stats_.min_x)));
        cloud_status.values.push_back(createKeyValue("FOV Y", 
            fmt::format("{:.2f} m", cloud_stats_.max_y - cloud_stats_.min_y)));
        cloud_status.values.push_back(createKeyValue("FOV Z", 
            fmt::format("{:.2f} m", cloud_stats_.max_z - cloud_stats_.min_z)));
        
        cloud_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        cloud_status.message = "Point cloud processing normal";
        
        diag_array.status.push_back(cloud_status);
        
        diagnostic_msgs::msg::DiagnosticStatus object_status;
        object_status.name = "Object Detection Performance";
        object_status.hardware_id = "N10_P_LiDAR";
        
        object_status.values.push_back(createKeyValue("Objects Detected", std::to_string(object_stats_.object_count)));
        
        if (!object_stats_.distances.empty()) {
            double avg_dist = std::accumulate(object_stats_.distances.begin(), 
                                              object_stats_.distances.end(), 0.0) / object_stats_.distances.size();
            double min_dist = *std::min_element(object_stats_.distances.begin(), object_stats_.distances.end());
            double max_dist = *std::max_element(object_stats_.distances.begin(), object_stats_.distances.end());
            
            object_status.values.push_back(createKeyValue("Avg Distance", fmt::format("{:.2f} m", avg_dist)));
            object_status.values.push_back(createKeyValue("Distance Range", fmt::format("{:.2f} - {:.2f} m", min_dist, max_dist)));
        }
        
        object_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        object_status.message = "Object detection active";
        
        diag_array.status.push_back(object_status);
        
        diag_pub_->publish(diag_array);
        
        std_msgs::msg::String status_msg;
        status_msg.data = fmt::format(
            "LiDAR Status | Points: {} | Objects: {} | Range: {:.2f}-{:.2f}m | Freq: {:.1f}Hz",
            cloud_stats_.points_in_range,
            object_stats_.object_count,
            scan_stats_.min_detected_range,
            scan_stats_.max_detected_range,
            scan_stats_.scan_frequency
        );
        status_pub_->publish(status_msg);
    }
    
    diagnostic_msgs::msg::KeyValue createKeyValue(const std::string& key, const std::string& value)
    {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        return kv;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarDiagnosticNode>());
    rclcpp::shutdown();
    return 0;
}