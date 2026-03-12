#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <lslidar_msgs/msg/detected_objects.hpp>
#include <lslidar_msgs/msg/detected_object.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <limits>
#include <cmath>
#include <mutex>
#include <deque>
#include <memory>
#include <algorithm>
#include <map>
#include <set>

struct PoleCandidate
{
    int id;
    double x, y, z;
    double radius;
    int points_count;
    rclcpp::Time first_seen;
    rclcpp::Time last_seen;
    int detection_count;
    std::vector<double> neighbor_distances;
    
    PoleCandidate(int id_, double x_, double y_, double z_, double r_, int pts, rclcpp::Time stamp)
        : id(id_), x(x_), y(y_), z(z_), radius(r_), points_count(pts), 
          first_seen(stamp), last_seen(stamp), detection_count(1) {}
};

class TrackedPole
{
public:
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
        if (total_detections >= 5) is_confirmed = true;
    }
    
    void markInvisible()
    {
        invisible_count++;
    }
    
    bool isStale(int max_invisible = 20) const
    {
        return invisible_count > max_invisible;
    }
};

class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor()
        : Node("lidar_processor"),
          max_poles_(6),
          association_distance_(0.15),
          max_invisible_frames_(20),
          next_pole_id_(0)
    {
        this->declare_parameter<std::string>("input_topic", "/lslidar_point_cloud");
        this->declare_parameter<std::string>("output_cloud_topic", "/cloud_processed");
        this->declare_parameter<double>("range_min", 0.2);
        this->declare_parameter<double>("range_max", 5.0);
        this->declare_parameter<double>("z_min", -0.3);
        this->declare_parameter<double>("z_max", 0.3);
        this->declare_parameter<double>("voxel_leaf_size", 0.02);
        this->declare_parameter<int>("cluster_min_size", 8);
        this->declare_parameter<int>("cluster_max_size", 150);
        this->declare_parameter<double>("cluster_tolerance", 0.05);
        this->declare_parameter<bool>("publish_filtered_cloud", true);
        this->declare_parameter<bool>("detect_objects", true);
        
        this->declare_parameter<double>("pole_expected_radius", 0.0125);
        this->declare_parameter<double>("pole_radius_tolerance", 0.008);
        this->declare_parameter<double>("pole_min_distance", 0.1);
        this->declare_parameter<double>("pole_max_distance", 3.0);
        this->declare_parameter<std::vector<double>>("expected_inter_pole_distances", std::vector<double>{0.185, 0.100});
        this->declare_parameter<double>("distance_match_tolerance", 0.03);
        this->declare_parameter<bool>("enable_pattern_matching", true);
        this->declare_parameter<bool>("publish_distance_matrix", true);
        
        this->declare_parameter<int>("max_poles", 6);
        this->declare_parameter<double>("association_distance", 0.15);
        this->declare_parameter<int>("max_invisible_frames", 20);
        this->declare_parameter<bool>("enable_tracking", true);

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
        publish_filtered_cloud_ = this->get_parameter("publish_filtered_cloud").as_bool();
        detect_objects_ = this->get_parameter("detect_objects").as_bool();
        
        pole_expected_radius_ = this->get_parameter("pole_expected_radius").as_double();
        pole_radius_tolerance_ = this->get_parameter("pole_radius_tolerance").as_double();
        pole_min_distance_ = this->get_parameter("pole_min_distance").as_double();
        pole_max_distance_ = this->get_parameter("pole_max_distance").as_double();
        expected_distances_ = this->get_parameter("expected_inter_pole_distances").as_double_array();
        distance_match_tol_ = this->get_parameter("distance_match_tolerance").as_double();
        enable_pattern_matching_ = this->get_parameter("enable_pattern_matching").as_bool();
        publish_distance_matrix_ = this->get_parameter("publish_distance_matrix").as_bool();
        
        max_poles_ = this->get_parameter("max_poles").as_int();
        association_distance_ = this->get_parameter("association_distance").as_double();
        max_invisible_frames_ = this->get_parameter("max_invisible_frames").as_int();
        enable_tracking_ = this->get_parameter("enable_tracking").as_bool();

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&LidarProcessor::cloudCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LidarProcessor::odomCallback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
        objects_pub_ = this->create_publisher<lslidar_msgs::msg::DetectedObjects>("/detected_objects", 10);
        poles_pub_ = this->create_publisher<lslidar_msgs::msg::DetectedObjects>("/detected_poles", 10);
        distance_matrix_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pole_distance_matrix", 10);
        pole_distances_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/inter_pole_distances", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pole_markers", 10);
        
        // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "LiDAR Pole Detector initialized");
        RCLCPP_INFO(this->get_logger(), "Expected pole radius: %.3f m (+/-%.3f)", pole_expected_radius_, pole_radius_tolerance_);
        RCLCPP_INFO(this->get_logger(), "Max poles to track: %d", max_poles_);
        RCLCPP_INFO(this->get_logger(), "Tracking: %s", enable_tracking_ ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(this->get_logger(), "Expected inter-pole distances: ");
        for (const auto& d : expected_distances_) {
            RCLCPP_INFO(this->get_logger(), "  %.3f m (+/-%.3f)", d, distance_match_tol_);
        }
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!msg || msg->width == 0 || msg->data.empty()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        convertPointCloud(msg, input_cloud);

        if (input_cloud->empty()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        applyFilters(input_cloud, filtered_cloud);

        if (filtered_cloud->empty()) return;

        if (publish_filtered_cloud_) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::PCLPointCloud2 pcl_pc2;
            pcl::toPCLPointCloud2(*filtered_cloud, pcl_pc2);
            pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
            cloud_msg.header = msg->header;
            cloud_msg.header.frame_id = "laser_link";
            cloud_pub_->publish(cloud_msg);
        }

        if (detect_objects_) {
            processAndDetect(filtered_cloud, msg->header);
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_ = *msg;
        has_odom_ = true;
    }

    void convertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
        output->header.frame_id = msg->header.frame_id;
        output->width = msg->width;
        output->height = msg->height;
        output->is_dense = msg->is_dense;
        output->points.resize(msg->width * msg->height);

        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (size_t i = 0; i < msg->fields.size(); ++i) {
            const auto& field = msg->fields[i];
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
        }

        if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
            output->clear();
            return;
        }

        const uint8_t* data = msg->data.data();
        for (uint32_t i = 0; i < msg->width * msg->height; ++i) {
            const uint8_t* point_data = data + i * msg->point_step;
            output->points[i].x = *reinterpret_cast<const float*>(point_data + x_offset);
            output->points[i].y = *reinterpret_cast<const float*>(point_data + y_offset);
            output->points[i].z = *reinterpret_cast<const float*>(point_data + z_offset);
        }
    }

    void applyFilters(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(input);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*output);

        if (output->empty()) return;

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(output);
        sor.setMeanK(8);
        sor.setStddevMulThresh(1.0);
        sor.filter(*output);

        if (output->empty()) return;

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(output);
        ror.setRadiusSearch(0.03);
        ror.setMinNeighborsInRadius(3);
        ror.filter(*output);

        if (output->empty()) return;

        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(output);
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel.filter(*output);
    }

    void processAndDetect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          const std_msgs::msg::Header& header)
    {
        std::vector<pcl::PointIndices> cluster_indices;
        clusterCloud(cloud, cluster_indices);

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

            double radius_error = std::abs(radius - pole_expected_radius_);
            if (radius_error <= pole_radius_tolerance_) {
                PoleCandidate pole(current_detections.size(), centroid.x, centroid.y, centroid.z,
                                  radius, cluster->size(), header.stamp);
                current_detections.push_back(pole);
            }
        }

        if (enable_tracking_ && !current_detections.empty()) {
            updateTracker(current_detections, header);
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

    void updateTracker(const std::vector<PoleCandidate>& detections, 
                       const std_msgs::msg::Header& header)
    {
        std::vector<bool> matched_detections(detections.size(), false);
        std::vector<bool> matched_tracks(tracked_poles_.size(), false);

        for (size_t i = 0; i < detections.size(); ++i) {
            double best_dist = association_distance_;
            int best_track = -1;
            
            for (size_t j = 0; j < tracked_poles_.size(); ++j) {
                if (matched_tracks[j]) continue;
                
                double dx = detections[i].x - tracked_poles_[j].world_x;
                double dy = detections[i].y - tracked_poles_[j].world_y;
                double dist = std::hypot(dx, dy);
                
                if (dist < best_dist) {
                    best_dist = dist;
                    best_track = j;
                }
            }
            
            if (best_track >= 0) {
                matched_detections[i] = true;
                matched_tracks[best_track] = true;
                tracked_poles_[best_track].update(
                    detections[i].x, detections[i].y, detections[i].radius, header.stamp);
            }
        }

        for (size_t i = 0; i < detections.size(); ++i) {
            if (!matched_detections[i]) {
                if (tracked_poles_.size() < static_cast<size_t>(max_poles_)) {
                    TrackedPole new_pole(next_pole_id_++, detections[i].x, detections[i].y,
                                        detections[i].radius, header.stamp);
                    tracked_poles_.push_back(new_pole);
                }
            }
        }

        for (size_t j = 0; j < tracked_poles_.size(); ++j) {
            if (!matched_tracks[j]) {
                tracked_poles_[j].markInvisible();
            }
        }

        tracked_poles_.erase(
            std::remove_if(tracked_poles_.begin(), tracked_poles_.end(),
                [this](const TrackedPole& t) { return t.isStale(max_invisible_frames_); }),
            tracked_poles_.end());
    }

    void computeCentroid(const pcl::PointCloud<pcl::PointXYZ>& cluster, pcl::PointXYZ& centroid)
    {
        Eigen::Vector4f centroid_4f;
        pcl::compute3DCentroid(cluster, centroid_4f);
        centroid.x = centroid_4f[0];
        centroid.y = centroid_4f[1];
        centroid.z = centroid_4f[2];
    }

    double estimateClusterRadius(const pcl::PointCloud<pcl::PointXYZ>& cluster, const pcl::PointXYZ& centroid)
    {
        double sum_dist = 0.0;
        double max_dist = 0.0;
        int count = 0;
        
        for (const auto& pt : cluster.points) {
            double dist = std::hypot(pt.x - centroid.x, pt.y - centroid.y);
            sum_dist += dist;
            if (dist > max_dist) max_dist = dist;
            count++;
        }
        
        if (count == 0) return 0.0;
        
        double avg_dist = sum_dist / count;
        // Deleted:double rms_dist = std::sqrt(sum_dist * sum_dist / count);
        
        return avg_dist * 1.15;
    }

    void clusterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      std::vector<pcl::PointIndices>& cluster_indices)
    {
        if (!cloud || cloud->empty()) return;

        try {
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud);

            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(cluster_tolerance_);
            ec.setMinClusterSize(cluster_min_size_);
            ec.setMaxClusterSize(cluster_max_size_);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);

            RCLCPP_DEBUG(this->get_logger(), "Found %zu clusters", cluster_indices.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Clustering failed: %s", e.what());
        }
    }

    void matchPolePattern()
    {
        if (tracked_poles_.size() < 2) return;

        std::map<std::pair<int, int>, double> distances;
        for (size_t i = 0; i < tracked_poles_.size(); ++i) {
            for (size_t j = i + 1; j < tracked_poles_.size(); ++j) {
                double dx = tracked_poles_[i].world_x - tracked_poles_[j].world_x;
                double dy = tracked_poles_[i].world_y - tracked_poles_[j].world_y;
                double dist = std::hypot(dx, dy);
                distances[std::make_pair(tracked_poles_[i].id, tracked_poles_[j].id)] = dist;
            }
        }

        int matches = 0;
        int total = distances.size();
        
        for (const auto& pair_dist : distances) {
            double measured = pair_dist.second;
            bool matched = false;
            
            for (const auto& expected : expected_distances_) {
                if (std::abs(measured - expected) <= distance_match_tol_) {
                    matched = true;
                    break;
                }
            }
            
            if (matched) matches++;
        }

        double match_ratio = static_cast<double>(matches) / std::max(1, total);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Pattern match: %.1f%% (%d/%d distances match expected)", 
                            match_ratio * 100.0, matches, total);
    }

    void publishDistanceMatrix(const std_msgs::msg::Header&)
    {
        if (tracked_poles_.size() < 2) return;

        std_msgs::msg::Float32MultiArray matrix_msg;
        matrix_msg.layout.dim.resize(2);
        matrix_msg.layout.dim[0].label = "pole_i";
        matrix_msg.layout.dim[0].size = tracked_poles_.size();
        matrix_msg.layout.dim[0].stride = tracked_poles_.size();
        matrix_msg.layout.dim[1].label = "pole_j";
        matrix_msg.layout.dim[1].size = tracked_poles_.size();
        matrix_msg.layout.dim[1].stride = 1;

        std_msgs::msg::Float32MultiArray distances_msg;
        distances_msg.layout.dim.resize(1);
        distances_msg.layout.dim[0].label = "measurements";
        distances_msg.layout.dim[0].size = tracked_poles_.size() * (tracked_poles_.size() - 1) / 2 * 5;

        for (size_t i = 0; i < tracked_poles_.size(); ++i) {
            for (size_t j = 0; j < tracked_poles_.size(); ++j) {
                if (i == j) {
                    matrix_msg.data.push_back(0.0);
                } else {
                    double dist = std::hypot(tracked_poles_[i].world_x - tracked_poles_[j].world_x,
                                           tracked_poles_[i].world_y - tracked_poles_[j].world_y);
                    matrix_msg.data.push_back(dist);
                }
            }
        }

        for (size_t i = 0; i < tracked_poles_.size(); ++i) {
            for (size_t j = i + 1; j < tracked_poles_.size(); ++j) {
                double dist = std::hypot(tracked_poles_[i].world_x - tracked_poles_[j].world_x,
                                        tracked_poles_[i].world_y - tracked_poles_[j].world_y);
                double dx = tracked_poles_[i].world_x - tracked_poles_[j].world_x;
                double dy = tracked_poles_[i].world_y - tracked_poles_[j].world_y;

                distances_msg.data.push_back(dist);
                distances_msg.data.push_back(std::abs(dx));
                distances_msg.data.push_back(std::abs(dy));
                distances_msg.data.push_back(tracked_poles_[i].id);
                distances_msg.data.push_back(tracked_poles_[j].id);
            }
        }

        distance_matrix_pub_->publish(matrix_msg);
        pole_distances_pub_->publish(distances_msg);
    }

    void publishMarkers(const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::MarkerArray markers;

        for (size_t i = 0; i < tracked_poles_.size(); ++i) {
            const auto& pole = tracked_poles_[i];

            visualization_msgs::msg::Marker sphere;
            sphere.header = header;
            sphere.ns = "poles";
            sphere.id = i * 2;
            sphere.type = visualization_msgs::msg::Marker::SPHERE;
            sphere.action = visualization_msgs::msg::Marker::ADD;
            sphere.pose.position.x = pole.world_x;
            sphere.pose.position.y = pole.world_y;
            sphere.pose.position.z = 0.0;
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = pole.avg_radius * 2.0 * 3.0;
            sphere.scale.y = pole.avg_radius * 2.0 * 3.0;
            sphere.scale.z = pole.avg_radius * 2.0 * 3.0;
            sphere.color.r = pole.is_confirmed ? 0.0 : 1.0;
            sphere.color.g = pole.is_confirmed ? 1.0 : 0.0;
            sphere.color.b = 0.0;
            sphere.color.a = 0.8;
            sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
            markers.markers.push_back(sphere);

            visualization_msgs::msg::Marker text;
            text.header = header;
            text.ns = "pole_labels";
            text.id = i * 2 + 1;
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = pole.world_x;
            text.pose.position.y = pole.world_y;
            text.pose.position.z = 0.3;
            text.pose.orientation.w = 1.0;
            text.scale.z = 0.15;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;
            text.text = "P" + std::to_string(pole.id) + "\n(" + 
                       std::to_string(static_cast<int>(pole.total_detections)) + ")";
            text.lifetime = rclcpp::Duration::from_seconds(0.5);
            markers.markers.push_back(text);
        }

        marker_pub_->publish(markers);
    }
    
    void publishTrackedPoles(const std_msgs::msg::Header& header)
    {
        lslidar_msgs::msg::DetectedObjects poles_msg;
        poles_msg.header = header;
        
        for (const auto& pole : tracked_poles_) {
            if (!pole.is_confirmed) continue;
            
            lslidar_msgs::msg::DetectedObject obj;
            obj.label = "pole_" + std::to_string(pole.id);
            obj.x = pole.world_x;
            obj.y = pole.world_y;
            obj.z = 0.0;
            obj.confidence = std::min(1.0, pole.total_detections / 20.0);
            obj.velocity_x = 0.0;
            obj.velocity_y = 0.0;
            poles_msg.objects.push_back(obj);
        }
        
        poles_pub_->publish(poles_msg);
        
        lslidar_msgs::msg::DetectedObjects objects_msg;
        objects_msg.header = header;
        objects_msg.objects = poles_msg.objects;
        objects_pub_->publish(objects_msg);
    }

    std::string input_topic_;
    std::string output_cloud_topic_;
    double range_min_, range_max_;
    double z_min_, z_max_;
    double voxel_leaf_size_;
    int cluster_min_size_, cluster_max_size_;
    double cluster_tolerance_;
    bool publish_filtered_cloud_;
    bool detect_objects_;

    double pole_expected_radius_;
    double pole_radius_tolerance_;
    double pole_min_distance_;
    double pole_max_distance_;
    std::vector<double> expected_distances_;
    double distance_match_tol_;
    bool enable_pattern_matching_;
    bool publish_distance_matrix_;
    
    int max_poles_;
    double association_distance_;
    int max_invisible_frames_;
    bool enable_tracking_;
    bool debug_mode_;

    std::vector<TrackedPole> tracked_poles_;
    int next_pole_id_;
    
    nav_msgs::msg::Odometry latest_odom_;
    bool has_odom_;
    std::mutex odom_mutex_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
    rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr poles_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr distance_matrix_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pole_distances_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}