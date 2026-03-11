#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
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
#include <sstream>
#include <iomanip>

class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor() : Node("lidar_processor")
    {
        // ---- Declare and retrieve parameters ----
        this->declare_parameter<std::string>("input_topic", "/lslidar_point_cloud");
        this->declare_parameter<std::string>("output_scan_topic", "/scan_processed");
        this->declare_parameter<std::string>("output_cloud_topic", "/cloud_processed");
        this->declare_parameter<double>("range_min", 0.15);
        this->declare_parameter<double>("range_max", 12.0);
        this->declare_parameter<double>("z_min", -1.0);
        this->declare_parameter<double>("z_max", 2.0);
        this->declare_parameter<double>("voxel_leaf_size", 0.05);
        this->declare_parameter<int>("cluster_min_size", 10);
        this->declare_parameter<int>("cluster_max_size", 500);
        this->declare_parameter<double>("cluster_tolerance", 0.2);
        this->declare_parameter<bool>("publish_filtered_cloud", true);
        this->declare_parameter<bool>("detect_objects", true);
        this->declare_parameter<bool>("classify_surfaces", false);

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_scan_topic_ = this->get_parameter("output_scan_topic").as_string();
        output_cloud_topic_ = this->get_parameter("output_cloud_topic").as_string();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        z_min_ = this->get_parameter("z_min").as_double();
        z_max_ = this->get_parameter("z_max").as_double();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        cluster_min_size_ = this->get_parameter("cluster_min_size").as_int();
        cluster_max_size_ = this->get_parameter("cluster_max_size").as_int();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        publish_filtered_cloud_ = this->get_parameter("publish_filtered_cloud").as_bool();
        detect_objects_ = this->get_parameter("detect_objects").as_bool();
        classify_surfaces_ = this->get_parameter("classify_surfaces").as_bool();

        // Create subscribers and publishers
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&LidarProcessor::cloudCallback, this, std::placeholders::_1));

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_scan_topic_, 10);
        
        if (publish_filtered_cloud_) {
            cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
        }

        if (detect_objects_) {
            objects_pub_ = this->create_publisher<lslidar_msgs::msg::DetectedObjects>("/detected_objects", 10);
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_markers", 10);
            
            // New: Distance publishers
            nearest_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/nearest_object_distance", 10);
            all_distances_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/all_object_distances", 10);
            object_positions_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/object_xy_positions", 10);
        }

        RCLCPP_INFO(this->get_logger(), "LidarProcessor started");
        RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output scan topic: %s", output_scan_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output cloud topic: %s", output_cloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  ✅ Publishing distances on /nearest_object_distance, /all_object_distances, /object_xy_positions");
    }
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // PassThrough filter for Z-axis (height filtering)
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*filtered_cloud);

        // Voxel Grid filter for downsampling
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        sor.filter(*filtered_cloud);

        return filtered_cloud;
    }

    void classifySurfaces(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        // Simple ground segmentation using RANSAC
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "No ground plane detected");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Ground plane detected with %zu points", inliers->indices.size());
        }
    }

    void detectObjects(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std_msgs::msg::Header& header)
    {
        if (cloud->empty() || cloud->size() < static_cast<size_t>(cluster_min_size_)) {
            return;
        }

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(cluster_min_size_);
        ec.setMaxClusterSize(cluster_max_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        lslidar_msgs::msg::DetectedObjects objects_msg;
        objects_msg.header = header;

        visualization_msgs::msg::MarkerArray markers;
        
        std::vector<float> distances;
        std::vector<float> xy_positions;

        int obj_id = 0;
        for (const auto& indices : cluster_indices)
        {
            if (indices.indices.size() < static_cast<size_t>(cluster_min_size_)) {
                continue;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (int idx : indices.indices)
                cluster->push_back(cloud->points[idx]);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);

            lslidar_msgs::msg::DetectedObject obj;
            obj.label = "object_" + std::to_string(obj_id);
            obj.x = centroid[0];
            obj.y = centroid[1];
            obj.z = centroid[2];
            
            double distance = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]);
            obj.confidence = static_cast<float>(indices.indices.size()) / 
                           static_cast<float>(cluster_max_size_) * 
                           std::exp(-distance / 5.0);
            
            distances.push_back(static_cast<float>(distance));
            xy_positions.push_back(centroid[0]);
            xy_positions.push_back(centroid[1]);
            
            double min_x = INFINITY, max_x = -INFINITY, min_y = INFINITY, max_y = -INFINITY;
            for (const auto& pt : cluster->points)
            {
                min_x = std::min(min_x, static_cast<double>(pt.x));
                max_x = std::max(max_x, static_cast<double>(pt.x));
                min_y = std::min(min_y, static_cast<double>(pt.y));
                max_y = std::max(max_y, static_cast<double>(pt.y));
            }
            
            obj.confidence *= 1.0 / (1.0 + (max_x - min_x) * (max_y - min_y));

            objects_msg.objects.push_back(obj);

            // Sphere marker at object position (RED)
            visualization_msgs::msg::Marker sphere_marker;
            sphere_marker.header = header;
            sphere_marker.ns = "detected_objects";
            sphere_marker.id = obj_id * 2;
            sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::msg::Marker::ADD;
            sphere_marker.pose.position.x = centroid[0];
            sphere_marker.pose.position.y = centroid[1];
            sphere_marker.pose.position.z = centroid[2];
            sphere_marker.pose.orientation.w = 1.0;
            sphere_marker.scale.x = 0.3;
            sphere_marker.scale.y = 0.3;
            sphere_marker.scale.z = 0.3;
            sphere_marker.color.r = 1.0;
            sphere_marker.color.g = 0.0;
            sphere_marker.color.b = 0.0;
            sphere_marker.color.a = 0.9;
            sphere_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            markers.markers.push_back(sphere_marker);
            
            // Text marker showing coordinates (WHITE TEXT ABOVE SPHERE)
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = header;
            text_marker.ns = "object_labels";
            text_marker.id = obj_id * 2 + 1;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = centroid[0];
            text_marker.pose.position.y = centroid[1];
            text_marker.pose.position.z = centroid[2] + 0.4;
            text_marker.pose.orientation.w = 1.0;
            
            std::ostringstream label_stream;
            label_stream << std::fixed << std::setprecision(2);
            label_stream << "#" << obj_id << " (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")";
            text_marker.text = label_stream.str();
            
            text_marker.scale.z = 0.15;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            markers.markers.push_back(text_marker);

            obj_id++;
        }

        if (!objects_msg.objects.empty()) {
            objects_pub_->publish(objects_msg);
            marker_pub_->publish(markers);
            
            // Publish nearest object distance
            float min_distance = *std::min_element(distances.begin(), distances.end());
            std_msgs::msg::Float32 nearest_msg;
            nearest_msg.data = min_distance;
            nearest_distance_pub_->publish(nearest_msg);
            
            // Publish all distances
            std_msgs::msg::Float32MultiArray distances_msg;
            distances_msg.data = distances;
            all_distances_pub_->publish(distances_msg);
            
            // Publish XY positions
            std_msgs::msg::Float32MultiArray positions_msg;
            positions_msg.data = xy_positions;
            object_positions_pub_->publish(positions_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "✅ Detected %zu objects | Nearest: %.2fm", 
                                objects_msg.objects.size(), min_distance);
        }
    }

    void convertToLaserScan(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std_msgs::msg::Header& header)
    {
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header = header;
        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = M_PI / 180.0;
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 0.1;
        scan_msg.range_min = range_min_;
        scan_msg.range_max = range_max_;

        int num_ranges = static_cast<int>(360.0 / (scan_msg.angle_increment * 180.0 / M_PI));
        scan_msg.ranges.resize(num_ranges, std::numeric_limits<float>::quiet_NaN());
        scan_msg.intensities.resize(num_ranges, 0.0);

        for (const auto& point : cloud->points)
        {
            double angle = std::atan2(point.y, point.x);
            int index = static_cast<int>((angle + M_PI) / scan_msg.angle_increment);
            
            if (index >= 0 && index < num_ranges) {
                float range = std::sqrt(point.x * point.x + point.y * point.y);
                if (range >= range_min_ && range <= range_max_) {
                    scan_msg.ranges[index] = range;
                    scan_msg.intensities[index] = point.intensity;
                }
            }
        }

        scan_pub_->publish(scan_msg);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        if (cloud->empty() || cloud->points.size() < 10) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Input cloud is empty or too small (%zu points)", 
                                cloud->points.size());
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %zu points", cloud->points.size());

        auto filtered_cloud = filterCloud(cloud);
        
        if (filtered_cloud->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Filtered cloud is empty after filtering");
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Filtered cloud has %zu points", filtered_cloud->points.size());
        
        if (publish_filtered_cloud_)
        {
            sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
            pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
            filtered_cloud_msg.header = cloud_msg->header;
            cloud_pub_->publish(filtered_cloud_msg);
        }

        if (classify_surfaces_ && filtered_cloud->size() > 100)
        {
            classifySurfaces(filtered_cloud);
        }

        if (detect_objects_ && filtered_cloud->size() >= static_cast<size_t>(cluster_min_size_))
        {
            detectObjects(filtered_cloud, cloud_msg->header);
        }

        convertToLaserScan(filtered_cloud, cloud_msg->header);
    }

    // Parameter storage
    std::string input_topic_;
    std::string output_scan_topic_;
    std::string output_cloud_topic_;
    double range_min_;
    double range_max_;
    double z_min_;
    double z_max_;
    double voxel_leaf_size_;
    int cluster_min_size_;
    int cluster_max_size_;
    double cluster_tolerance_;
    bool publish_filtered_cloud_;
    bool detect_objects_;
    bool classify_surfaces_;

    // ROS 2 handles
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr nearest_distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr all_distances_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr object_positions_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}