#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <lslidar_msgs/msg/detected_objects.hpp>
#include <lslidar_msgs/msg/detected_object.hpp>

class LidarDetector : public rclcpp::Node
{
public:
    LidarDetector() : Node("lidar_detector")
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_scan_topic", "/scan_filtered");
        this->declare_parameter<float>("cluster_tolerance", 0.1);
        this->declare_parameter<int>("min_cluster_size", 5);
        this->declare_parameter<int>("max_cluster_size", 100);

        input_topic_ = this->get_parameter("input_scan_topic").as_string();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();

        // Subscriber to filtered scan
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_topic_, 10, std::bind(&LidarDetector::callback, this, std::placeholders::_1));

        // Publisher for detected objects
        pub_ = this->create_publisher<lslidar_msgs::msg::DetectedObjects>("/detected_objects", 10);

        // Laser projection tool
        projector_ = std::make_shared<laser_geometry::LaserProjection>();

        RCLCPP_INFO(this->get_logger(), "LidarDetector started, listening to %s", input_topic_.c_str());
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Project scan to point cloud
        sensor_msgs::msg::PointCloud2 cloud_msg;
        projector_->projectLaser(*scan, cloud_msg);

        // Convert to PCL cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_msg, *cloud);

        // Set up KD-Tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // Prepare output message
        lslidar_msgs::msg::DetectedObjects out;
        out.header = scan->header;

        for (const auto& indices : cluster_indices)
        {
            double x_sum = 0, y_sum = 0, z_sum = 0;
            for (int idx : indices.indices) {
                x_sum += cloud->points[idx].x;
                y_sum += cloud->points[idx].y;
                z_sum += cloud->points[idx].z;
            }
            size_t n = indices.indices.size();
            lslidar_msgs::msg::DetectedObject obj;
            obj.label = "unknown";
            obj.x = x_sum / n;
            obj.y = y_sum / n;
            obj.z = z_sum / n;
            obj.confidence = 1.0; // placeholder, can be improved
            out.objects.push_back(obj);
        }

        pub_->publish(out);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Detected %zu objects", out.objects.size());
    }

    std::string input_topic_;
    double cluster_tolerance_;
    int min_cluster_size_, max_cluster_size_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr pub_;
    std::shared_ptr<laser_geometry::LaserProjection> projector_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarDetector>());
    rclcpp::shutdown();
    return 0;
}