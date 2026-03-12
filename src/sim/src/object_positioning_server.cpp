#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rc2026_interfaces/action/gripper_control.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <lslidar_msgs/msg/detected_objects.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <vector>
#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

class LidarPointcloudProcessor : public rclcpp::Node
{
public:
    LidarPointcloudProcessor() : Node("lidar_pointcloud_processor"), next_pole_id_(0)
    {
        this->declare_parameter<std::string>("input_topic", "/lslidar_point_cloud");
        this->declare_parameter<std::string>("output_cloud_topic", "/cloud_processed");
        this->declare_parameter<double>("range_min", 0.2);
        this->declare_parameter<double>("range_max", 3.0);
        this->declare_parameter<double>("z_min", -0.3);
        this->declare_parameter<double>("z_max", 0.3);
        this->declare_parameter<double>("voxel_leaf_size", 0.02);
        this->declare_parameter<int>("cluster_min_size", 6);
        this->declare_parameter<int>("cluster_max_size", 150);
        this->declare_parameter<double>("cluster_tolerance", 0.05);
        this->declare_parameter<bool>("detect_objects", true);
        this->declare_parameter<double>("pole_expected_radius", 0.0125);
        this->declare_parameter<double>("pole_radius_tolerance", 0.008);
        this->declare_parameter<int>("max_poles", 6);
        this->declare_parameter<double>("association_distance", 0.2);
        this->declare_parameter<bool>("enable_tracking", true);
        this->declare_parameter<bool>("debug_mode", false);
        this->declare_parameter<int>("max_invisible_frames", 10);
        this->declare_parameter<std::vector<double>>("expected_inter_pole_distances", std::vector<double>{0.205});
        this->declare_parameter<double>("distance_match_tolerance", 0.01);
        this->declare_parameter<bool>("publish_distance_matrix", false);
        this->declare_parameter<bool>("enable_pattern_matching", false);

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_cloud_topic_ = this->get_parameter("output_cloud_topic").as_string();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        z_min_ = this->get_parameter("z_min").as_double();
        z_max_ = this->get_parameter("z_max").as_double();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        cluster_min_size_ = this->get_parameter("cluster_min_size").as_int();
        cluster_max_size_ = this->get_parameter("cluster_max_size").as_int();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        detect_objects_ = this->get_parameter("detect_objects").as_bool();
        pole_expected_radius_ = this->get_parameter("pole_expected_radius").as_double();
        pole_radius_tolerance_ = this->get_parameter("pole_radius_tolerance").as_double();
        max_poles_ = this->get_parameter("max_poles").as_int();
        association_distance_ = this->get_parameter("association_distance").as_double();
        enable_tracking_ = this->get_parameter("enable_tracking").as_bool();
        debug_mode_ = this->get_parameter("debug_mode").as_bool();
        max_invisible_frames_ = this->get_parameter("max_invisible_frames").as_int();
        expected_distances_ = this->get_parameter("expected_inter_pole_distances").as_double_array();
        distance_match_tol_ = this->get_parameter("distance_match_tolerance").as_double();
        publish_distance_matrix_ = this->get_parameter("publish_distance_matrix").as_bool();
        enable_pattern_matching_ = this->get_parameter("enable_pattern_matching").as_bool();

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LidarPointcloudProcessor::cloudCallback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
        markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pole_markers", 10);
        poles_pub_ = this->create_publisher<lslidar_msgs::msg::DetectedObjects>("/detected_poles", 10);
        objects_pub_ = this->create_publisher<lslidar_msgs::msg::DetectedObjects>("/detected_objects", 10);
        distance_matrix_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pole_distance_matrix", 10);

        RCLCPP_INFO(this->get_logger(), "LiDAR Pole Detector initialized");
        RCLCPP_INFO(this->get_logger(), "Expected pole radius: %.3f m (+/-%.3f)", pole_expected_radius_, pole_radius_tolerance_);
        RCLCPP_INFO(this->get_logger(), "Max poles to track: %d", max_poles_);
        RCLCPP_INFO(this->get_logger(), "Tracking: %s", enable_tracking_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(this->get_logger(), "Debug mode: %s", debug_mode_ ? "ON" : "OFF");
        RCLCPP_INFO(this->get_logger(), "Association distance: %.3f m", association_distance_);
        RCLCPP_INFO(this->get_logger(), "Expected inter-pole distances: ");
        for (const auto& d : expected_distances_) {
            RCLCPP_INFO(this->get_logger(), "  %.3f m (+/-%.3f)", d, distance_match_tol_);
        }
    }

private:
    struct TrackedPole {
        int id;
        double world_x, world_y;
        double avg_radius;
        int total_detections;
        rclcpp::Time first_seen;
        rclcpp::Time last_seen;
        int invisible_count;
        bool is_confirmed;

        TrackedPole(int id_, double wx, double wy, double r, rclcpp::Time stamp)
            : id(id_), world_x(wx), world_y(wy), avg_radius(r), total_detections(1),
              first_seen(stamp), last_seen(stamp), invisible_count(0), is_confirmed(false) {}

        void update(double wx, double wy, double r, rclcpp::Time stamp)
        {
            world_x = 0.8 * world_x + 0.2 * wx;
            world_y = 0.8 * world_y + 0.2 * wy;
            avg_radius = 0.9 * avg_radius + 0.1 * r;
            last_seen = stamp;
            total_detections++;
            invisible_count = 0;
            if (total_detections >= 3) is_confirmed = true;
        }
    };

    struct PoleCandidate {
        int id;
        double x, y, z;
        double radius;
        int num_points;
        rclcpp::Time timestamp;

        PoleCandidate(int id_, double x_, double y_, double z_, double r_, int n_, rclcpp::Time t_)
            : id(id_), x(x_), y(y_), z(z_), radius(r_), num_points(n_), timestamp(t_) {}
    };

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr poles_pub_;
    rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr distance_matrix_pub_;

    std::string input_topic_;
    std::string output_cloud_topic_;
    double range_min_, range_max_;
    double z_min_, z_max_;
    double voxel_leaf_size_;
    int cluster_min_size_, cluster_max_size_;
    double cluster_tolerance_;
    bool detect_objects_;
    double pole_expected_radius_;
    double pole_radius_tolerance_;
    int max_poles_;
    double association_distance_;
    bool enable_tracking_;
    bool debug_mode_;
    int max_invisible_frames_;
    std::vector<double> expected_distances_;
    double distance_match_tol_;
    bool publish_distance_matrix_;
    bool enable_pattern_matching_;

    std::vector<TrackedPole> tracked_poles_;
    int next_pole_id_;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Empty cloud received");
            return;
        }

        cloud = preprocessCloud(cloud);

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header = msg->header;
        output_msg.header.frame_id = "laser_link";
        cloud_pub_->publish(output_msg);

        if (detect_objects_) {
            processAndDetect(cloud, msg->header);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pt : cloud->points) {
            double d = std::sqrt(pt.x*pt.x + pt.y*pt.y);
            if (d >= range_min_ && d <= range_max_) {
                filtered->push_back(pt);
            }
        }

        if (voxel_leaf_size_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setInputCloud(filtered);
            voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel.filter(*filtered);
        }

        return filtered;
    }

    void processAndDetect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          const std_msgs::msg::Header& header)
    {
        std::vector<pcl::PointIndices> cluster_indices;
        clusterCloud(cloud, cluster_indices);

        if (debug_mode_) {
            RCLCPP_DEBUG(this->get_logger(), "Found %zu raw clusters", cluster_indices.size());
        }

        std::vector<PoleCandidate> current_detections;

        for (const auto& indices : cluster_indices) {
            if (indices.indices.size() < static_cast<size_t>(cluster_min_size_)) continue;
            if (indices.indices.size() > static_cast<size_t>(cluster_max_size_)) continue;

            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : indices.indices) {
                if (idx >= 0 && idx < static_cast<int>(cloud->points.size())) {
                    cluster->push_back(cloud->points[idx]);
                }
            }

            pcl::PointXYZ centroid;
            computeCentroid(*cluster, centroid);

            double radius = estimateClusterRadius(*cluster, centroid);

            if (debug_mode_) {
                RCLCPP_DEBUG(this->get_logger(), "Cluster: pts=%zu, radius=%.4f, pos=(%.3f, %.3f)",
                            cluster->size(), radius, centroid.x, centroid.y);
            }

            double radius_error = std::abs(radius - pole_expected_radius_);
            if (radius_error <= pole_radius_tolerance_) {
                PoleCandidate pole(current_detections.size(), centroid.x, centroid.y, centroid.z,
                                  radius, cluster->size(), header.stamp);
                current_detections.push_back(pole);

                if (debug_mode_) {
                    RCLCPP_DEBUG(this->get_logger(), "Valid pole candidate! ID=%d", pole.id);
                }
            } else {
                if (debug_mode_) {
                    RCLCPP_DEBUG(this->get_logger(), "Rejected: radius error=%.4f (tol=%.4f)",
                                radius_error, pole_radius_tolerance_);
                }
            }
        }

        if (debug_mode_ && !current_detections.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Total valid pole candidates: %zu", current_detections.size());
        }

        if (enable_tracking_ && !current_detections.empty()) {
            updateTracker(current_detections, header);
        }

        if (debug_mode_) {
            RCLCPP_DEBUG(this->get_logger(), "Currently tracking %zu poles", tracked_poles_.size());
        }

        if (!tracked_poles_.empty()) {
            if (enable_pattern_matching_ && tracked_poles_.size() >= 2) {
                matchPolePattern();
            }

            if (publish_distance_matrix_) {
                publishDistanceMatrix(header);
            }

            publishMarkers(header);
            publishTrackedPoles(header);
        }

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Detected %zu clusters, tracking %zu poles", 
                             current_detections.size(), tracked_poles_.size());
    }

    void clusterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      std::vector<pcl::PointIndices>& cluster_indices)
    {
        if (cloud->empty()) return;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(cluster_min_size_);
        ec.setMaxClusterSize(cluster_max_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
    }

    void computeCentroid(const pcl::PointCloud<pcl::PointXYZ>& cluster, pcl::PointXYZ& centroid)
    {
        centroid.x = centroid.y = centroid.z = 0.0;
        if (cluster.empty()) return;
        for (const auto& pt : cluster.points) {
            centroid.x += pt.x;
            centroid.y += pt.y;
            centroid.z += pt.z;
        }
        centroid.x /= cluster.size();
        centroid.y /= cluster.size();
        centroid.z /= cluster.size();
    }

    double estimateClusterRadius(const pcl::PointCloud<pcl::PointXYZ>& cluster, const pcl::PointXYZ& centroid)
    {
        if (cluster.empty()) return 0.0;
        double max_dist_sq = 0.0;
        for (const auto& pt : cluster.points) {
            double dx = pt.x - centroid.x;
            double dy = pt.y - centroid.y;
            double dz = pt.z - centroid.z;
            double dist_sq = dx*dx + dy*dy + dz*dz;
            if (dist_sq > max_dist_sq) max_dist_sq = dist_sq;
        }
        return std::sqrt(max_dist_sq);
    }

    void updateTracker(const std::vector<PoleCandidate>& candidates, const std_msgs::msg::Header&)
    {
        for (auto& pole : tracked_poles_) {
            pole.invisible_count++;
        }

        for (const auto& cand : candidates) {
            double wx = cand.x;
            double wy = cand.y;

            int best_idx = -1;
            double best_dist = association_distance_ * 2;
            for (size_t i = 0; i < tracked_poles_.size(); ++i) {
                double dx = tracked_poles_[i].world_x - wx;
                double dy = tracked_poles_[i].world_y - wy;
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist < association_distance_ && dist < best_dist) {
                    best_dist = dist;
                    best_idx = i;
                }
            }

            if (best_idx >= 0) {
                tracked_poles_[best_idx].update(wx, wy, cand.radius, cand.timestamp);
            } else {
                if (tracked_poles_.size() < static_cast<size_t>(max_poles_)) {
                    tracked_poles_.emplace_back(next_pole_id_++, wx, wy, cand.radius, cand.timestamp);
                    RCLCPP_INFO(this->get_logger(), "New pole tracked: ID=%d at (%.3f, %.3f)",
                                tracked_poles_.back().id, wx, wy);
                }
            }
        }

        tracked_poles_.erase(std::remove_if(tracked_poles_.begin(), tracked_poles_.end(),
            [this](const TrackedPole& p) { return p.invisible_count > max_invisible_frames_; }),
            tracked_poles_.end());
    }

    void publishTrackedPoles(const std_msgs::msg::Header& header)
    {
        lslidar_msgs::msg::DetectedObjects poles_msg;
        poles_msg.header = header;

        for (const auto& pole : tracked_poles_) {
            lslidar_msgs::msg::DetectedObject obj;
            obj.label = "pole_" + std::to_string(pole.id);
            obj.x = pole.world_x;
            obj.y = pole.world_y;
            obj.z = 0.0;
            obj.confidence = std::min(1.0, pole.total_detections / 10.0);
            obj.velocity_x = 0.0;
            obj.velocity_y = 0.0;
            poles_msg.objects.push_back(obj);

            RCLCPP_DEBUG(this->get_logger(), "Pole %d: (%.3f, %.3f) conf=%.2f detections=%d %s",
                        pole.id, pole.world_x, pole.world_y, obj.confidence,
                        pole.total_detections, pole.is_confirmed ? "[CONFIRMED]" : "");
        }

        if (!poles_msg.objects.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Publishing %zu tracked poles", poles_msg.objects.size());
        }

        poles_pub_->publish(poles_msg);

        lslidar_msgs::msg::DetectedObjects objects_msg;
        objects_msg.header = header;
        objects_msg.objects = poles_msg.objects;
        objects_pub_->publish(objects_msg);
    }

    void publishMarkers(const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::MarkerArray markers;
        int id = 0;
        for (const auto& pole : tracked_poles_) {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.header.frame_id = "laser_link";
            marker.ns = "poles";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = pole.world_x;
            marker.pose.position.y = pole.world_y;
            marker.pose.position.z = 0.0;
            marker.scale.x = pole.avg_radius * 2;
            marker.scale.y = pole.avg_radius * 2;
            marker.scale.z = pole.avg_radius * 2;
            marker.color.r = pole.is_confirmed ? 0.0 : 1.0;
            marker.color.g = pole.is_confirmed ? 1.0 : 0.5;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            markers.markers.push_back(marker);

            visualization_msgs::msg::Marker text;
            text.header = marker.header;
            text.ns = "pole_labels";
            text.id = id++;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = pole.world_x;
            text.pose.position.y = pole.world_y;
            text.pose.position.z = 0.2;
            text.scale.z = 0.1;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;
            text.text = "P" + std::to_string(pole.id);
            text.lifetime = rclcpp::Duration::from_seconds(0.5);
            markers.markers.push_back(text);
        }
        markers_pub_->publish(markers);
    }

    void publishDistanceMatrix(const std_msgs::msg::Header&)
    {
    }

    void matchPolePattern()
    {
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPointcloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}