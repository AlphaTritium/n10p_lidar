#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <lslidar_msgs/msg/detected_objects.hpp>
#include <lslidar_msgs/msg/detected_object.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>

#include <limits>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <deque>
#include <memory>

// -------------------------------------------------------------------
// Simple 2D constant velocity Kalman filter for tracking
// -------------------------------------------------------------------
class KalmanFilter2D
{
public:
    KalmanFilter2D()
    {
        // State: [x, y, vx, vy]
        x_ = Eigen::VectorXd::Zero(4);
        P_ = Eigen::MatrixXd::Identity(4, 4) * 0.1;
        F_ = Eigen::MatrixXd::Identity(4, 4);
        F_(0, 2) = 1.0;
        F_(1, 3) = 1.0;
        H_ = Eigen::MatrixXd::Zero(2, 4);
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;
        R_ = Eigen::MatrixXd::Identity(2, 2) * 0.05;
        Q_ = Eigen::MatrixXd::Identity(4, 4) * 0.01;
    }

    void predict(double dt)
    {
        F_(0, 2) = dt;
        F_(1, 3) = dt;
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(double mx, double my)
    {
        Eigen::Vector2d z(mx, my);
        Eigen::Vector2d y = z - H_ * x_;
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(4, 4) - K * H_) * P_;
    }

    Eigen::VectorXd getState() const { return x_; }
    double getX() const { return x_(0); }
    double getY() const { return x_(1); }
    double getVx() const { return x_(2); }
    double getVy() const { return x_(3); }

private:
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_, F_, H_, R_, Q_;
};

// -------------------------------------------------------------------
// Track structure
// -------------------------------------------------------------------
struct Track
{
    int id;
    KalmanFilter2D kf;
    rclcpp::Time last_update;
    int age;
    int invisible_count;
    double diameter;
    double circularity;
    double confidence;

    Track(int id_, double x, double y, double diam, double circ, double conf, rclcpp::Time stamp)
        : id(id_), last_update(stamp), age(1), invisible_count(0),
          diameter(diam), circularity(circ), confidence(conf)
    {
        kf.update(x, y);
    }

    void predict(rclcpp::Time current_time)
    {
        double dt = (current_time - last_update).seconds();
        if (dt > 0.0) {
            kf.predict(dt);
        }
    }

    void update(double x, double y, double diam, double circ, double conf, rclcpp::Time stamp)
    {
        kf.update(x, y);
        last_update = stamp;
        age++;
        invisible_count = 0;
        diameter = 0.9 * diameter + 0.1 * diam;
        circularity = 0.9 * circularity + 0.1 * circ;
        confidence = 0.9 * confidence + 0.1 * conf;
    }

    void markInvisible()
    {
        invisible_count++;
    }

    bool isStale(int max_invisible = 5) const
    {
        return invisible_count > max_invisible;
    }
};

// -------------------------------------------------------------------
// Main Node
// -------------------------------------------------------------------
class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor() : Node("lidar_processor"), system_enabled_(true), processing_active_(false),
                       next_track_id_(0)
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "/lslidar_point_cloud");
        this->declare_parameter<std::string>("output_scan_topic", "/scan_processed");
        this->declare_parameter<std::string>("output_cloud_topic", "/cloud_processed");
        this->declare_parameter<double>("range_min", 0.15);
        this->declare_parameter<double>("range_max", 12.0);
        this->declare_parameter<double>("z_min", -1.0);
        this->declare_parameter<double>("z_max", 2.0);
        this->declare_parameter<double>("voxel_leaf_size", 0.005);
        this->declare_parameter<int>("cluster_min_size", 10);
        this->declare_parameter<int>("cluster_max_size", 500);
        this->declare_parameter<double>("cluster_tolerance", 0.2);
        this->declare_parameter<bool>("publish_filtered_cloud", true);
        this->declare_parameter<bool>("detect_objects", true);
        this->declare_parameter<bool>("classify_surfaces", false);
        this->declare_parameter<bool>("system_enabled", true);
        this->declare_parameter<bool>("enable_circle_detection", false);
        this->declare_parameter<double>("expected_object_diameter", 0.025);
        this->declare_parameter<double>("detection_tolerance", 0.005);
        this->declare_parameter<int>("accumulated_scans", 1);
        this->declare_parameter<bool>("enable_tracking", true);
        this->declare_parameter<double>("track_max_association_distance", 0.2);
        this->declare_parameter<int>("track_max_invisible", 5);

        // Get parameters
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
        system_enabled_ = this->get_parameter("system_enabled").as_bool();
        enable_circle_detection_ = this->get_parameter("enable_circle_detection").as_bool();
        expected_diameter_ = this->get_parameter("expected_object_diameter").as_double();
        detection_tolerance_ = this->get_parameter("detection_tolerance").as_double();
        num_accumulated_scans_ = this->get_parameter("accumulated_scans").as_int();
        enable_tracking_ = this->get_parameter("enable_tracking").as_bool();
        track_max_association_dist_ = this->get_parameter("track_max_association_distance").as_double();
        track_max_invisible_ = this->get_parameter("track_max_invisible").as_int();

        // Subscribers
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&LidarProcessor::cloudCallback, this, std::placeholders::_1));

        // Publishers
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_scan_topic_, 10);
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
        objects_pub_ = this->create_publisher<lslidar_msgs::msg::DetectedObjects>("/detected_objects", 10);
        markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_markers", 10);
        tracked_objects_pub_ = this->create_publisher<lslidar_msgs::msg::DetectedObjects>("/tracked_objects", 10);

        RCLCPP_INFO(this->get_logger(), "LiDAR processor initialized");
        RCLCPP_INFO(this->get_logger(), "Circle detection: %s", enable_circle_detection_ ? "ENABLED" : "DISABLED");
        if (enable_circle_detection_) {
            RCLCPP_INFO(this->get_logger(), "Expected diameter: %.3f m (±%.3f)", 
                       expected_diameter_, detection_tolerance_);
            RCLCPP_INFO(this->get_logger(), "Accumulating %d scans for higher resolution", 
                       num_accumulated_scans_);
        }
        RCLCPP_INFO(this->get_logger(), "Tracking: %s", enable_tracking_ ? "ENABLED" : "DISABLED");
    }

private:
    struct ShapeFitResult {
        bool valid;
        double center_x, center_y, radius;
        double rmse;
        int points_used;
        double circularity;
        double aspect_ratio;
    };

    // Enhanced filtering
    void applyEnhancedFilters(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr output)
    {
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(input);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*output);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(output);
        sor.setMeanK(8);
        sor.setStddevMulThresh(1.0);
        sor.filter(*output);

        pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
        ror.setInputCloud(output);
        ror.setRadiusSearch(0.02);
        ror.setMinNeighborsInRadius(2);
        ror.filter(*output);

        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(output);
        voxel.setLeafSize(voxel_leaf_size_ * 0.5, voxel_leaf_size_ * 0.5, voxel_leaf_size_ * 0.5);
        voxel.filter(*output);
    }

    ShapeFitResult fitShapeToCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr points)
    {
        ShapeFitResult result;
        result.valid = false;
        if (points->size() < 5) return result;

        Eigen::MatrixXd A(points->size(), 3);
        Eigen::VectorXd B(points->size());
        for (size_t i = 0; i < points->size(); i++) {
            A(i, 0) = points->points[i].x;
            A(i, 1) = points->points[i].y;
            A(i, 2) = 1.0;
            B(i) = -(points->points[i].x * points->points[i].x +
                     points->points[i].y * points->points[i].y);
        }

        Eigen::VectorXd x = (A.transpose() * A).ldlt().solve(A.transpose() * B);
        if (!x.array().isFinite().all()) return result;

        double center_x = -x(0) / 2.0;
        double center_y = -x(1) / 2.0;
        double radius = std::sqrt(x(0) * x(0) + x(1) * x(1) - 4.0 * x(2)) / 2.0;
        if (!std::isfinite(radius) || radius <= 0 || radius > 0.5) return result;

        double rmse = 0.0;
        std::vector<double> distances;
        for (const auto& pt : points->points) {
            double d = std::hypot(pt.x - center_x, pt.y - center_y);
            distances.push_back(d);
            rmse += (d - radius) * (d - radius);
        }
        rmse = std::sqrt(rmse / points->size());

        double radial_variance = 0.0;
        for (double d : distances) radial_variance += (d - radius) * (d - radius);
        radial_variance /= points->size();
        double circularity = 1.0 / (1.0 + radial_variance / (radius * radius));

        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*points, min_pt, max_pt);
        double extent_x = max_pt.x - min_pt.x;
        double extent_y = max_pt.y - min_pt.y;
        double aspect_ratio = (extent_y > 0) ? std::max(extent_x, extent_y) / std::min(extent_x, extent_y) : 1.0;

        result.valid = true;
        result.center_x = center_x;
        result.center_y = center_y;
        result.radius = radius;
        result.rmse = rmse;
        result.points_used = points->size();
        result.circularity = circularity;
        result.aspect_ratio = aspect_ratio;
        return result;
    }

    // Detection (returns vector of DetectedObject)
    void detectObjects(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                       const std_msgs::msg::Header& header,
                       std::vector<lslidar_msgs::msg::DetectedObject>& out_detections)
    {
        std::vector<pcl::PointIndices> cluster_indices;
        clusterCloud(cloud, cluster_indices);

        for (const auto& indices : cluster_indices) {
            if (indices.indices.size() < static_cast<size_t>(cluster_min_size_)) continue;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (int idx : indices.indices) cluster->push_back(cloud->points[idx]);

            if (enable_circle_detection_) {
                ShapeFitResult shape = fitShapeToCluster(cluster);
                if (shape.valid) {
                    double diameter = shape.radius * 2.0;
                    double diameter_error = std::abs(diameter - expected_diameter_);
                    if (diameter_error <= detection_tolerance_ && shape.circularity >= 0.6) {
                        lslidar_msgs::msg::DetectedObject obj;
                        obj.label = "circular_" + std::to_string(static_cast<int>(diameter*1000)) + "mm";
                        obj.x = shape.center_x;
                        obj.y = shape.center_y;
                        obj.z = 0.0;
                        obj.confidence = shape.circularity * (1.0 / (1.0 + diameter_error * 100.0));
                        out_detections.push_back(obj);
                    }
                }
            } else {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cluster, centroid);
                lslidar_msgs::msg::DetectedObject obj;
                obj.label = "object";
                obj.x = centroid[0];
                obj.y = centroid[1];
                obj.z = centroid[2];
                obj.confidence = 1.0;
                out_detections.push_back(obj);
            }
        }
    }

    // Wrapper for publishing raw detections
    void detectObjects(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std_msgs::msg::Header& header)
    {
        std::vector<lslidar_msgs::msg::DetectedObject> dets;
        detectObjects(cloud, header, dets);
        lslidar_msgs::msg::DetectedObjects msg;
        msg.header = header;
        msg.objects = dets;
        objects_pub_->publish(msg);
    }

    // Tracking update
    void updateTracker(const std::vector<lslidar_msgs::msg::DetectedObject>& detections, rclcpp::Time stamp)
    {
        // Predict all tracks
        for (auto& track : tracks_) {
            track.predict(stamp);
        }

        std::vector<bool> matched_detections(detections.size(), false);
        std::vector<bool> matched_tracks(tracks_.size(), false);

        // Association: nearest neighbor
        for (size_t i = 0; i < detections.size(); ++i) {
            const auto& d = detections[i];
            double best_dist = track_max_association_dist_;
            int best_track = -1;
            for (size_t j = 0; j < tracks_.size(); ++j) {
                if (matched_tracks[j]) continue;
                double dx = d.x - tracks_[j].kf.getX();
                double dy = d.y - tracks_[j].kf.getY();
                double dist = std::hypot(dx, dy);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_track = j;
                }
            }
            if (best_track >= 0) {
                matched_detections[i] = true;
                matched_tracks[best_track] = true;
                tracks_[best_track].update(d.x, d.y, expected_diameter_, 0.8, d.confidence, stamp);
            }
        }

        // New tracks
        for (size_t i = 0; i < detections.size(); ++i) {
            if (!matched_detections[i]) {
                const auto& d = detections[i];
                tracks_.emplace_back(next_track_id_++, d.x, d.y, expected_diameter_, 0.8, d.confidence, stamp);
            }
        }

        // Mark unmatched as invisible
        for (size_t j = 0; j < tracks_.size(); ++j) {
            if (!matched_tracks[j]) {
                tracks_[j].markInvisible();
            }
        }

        // Remove stale tracks
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                      [this](const Track& t) { return t.isStale(track_max_invisible_); }),
                      tracks_.end());

        // Publish tracked objects
        lslidar_msgs::msg::DetectedObjects tracked_msg;
        tracked_msg.header.frame_id = "laser_link";  // or whatever the sensor frame is
        tracked_msg.header.stamp = stamp;
        for (const auto& track : tracks_) {
            lslidar_msgs::msg::DetectedObject obj;
            obj.label = "track_" + std::to_string(track.id);
            obj.x = track.kf.getX();
            obj.y = track.kf.getY();
            obj.z = 0.0;
            obj.confidence = track.confidence;
            tracked_msg.objects.push_back(obj);
        }
        tracked_objects_pub_->publish(tracked_msg);

        // Visualization markers
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;
        for (const auto& track : tracks_) {
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "laser_link";
            text_marker.header.stamp = stamp;
            text_marker.ns = "tracked_objects";
            text_marker.id = marker_id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = track.kf.getX();
            text_marker.pose.position.y = track.kf.getY();
            text_marker.pose.position.z = 0.1;
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.1;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            std::ostringstream ss;
            ss << "ID:" << track.id << "\n"
               << std::fixed << std::setprecision(2)
               << track.kf.getVx() << "," << track.kf.getVy() << " m/s";
            text_marker.text = ss.str();
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            markers.markers.push_back(text_marker);
        }
        markers_pub_->publish(markers);
    }

    // Main callback (no TF)
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!system_enabled_) return;

        // Convert directly (no transform)
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        try {
            pcl::fromROSMsg(*msg, *input_cloud);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert point cloud: %s", e.what());
            return;
        }

        if (input_cloud->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Empty cloud received");
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        applyEnhancedFilters(input_cloud, filtered_cloud);

        if (filtered_cloud->empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No points after filtering");
            return;
        }

        if (publish_filtered_cloud_) {
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*filtered_cloud, cloud_msg);
            cloud_msg.header = msg->header;
            cloud_pub_->publish(cloud_msg);
        }

        convertToLaserScan(filtered_cloud, msg->header);

        if (detect_objects_) {
            if (enable_circle_detection_ && num_accumulated_scans_ > 1) {
                accumulateAndDetect(msg, filtered_cloud);
            } else {
                std::vector<lslidar_msgs::msg::DetectedObject> detections;
                detectObjects(filtered_cloud, msg->header, detections);
                if (enable_tracking_) {
                    updateTracker(detections, msg->header.stamp);
                } else {
                    lslidar_msgs::msg::DetectedObjects objects_msg;
                    objects_msg.header = msg->header;
                    objects_msg.objects = detections;
                    objects_pub_->publish(objects_msg);
                }
            }
        }
    }

    void accumulateAndDetect(const sensor_msgs::msg::PointCloud2::SharedPtr& current_msg,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud)
    {
        scan_buffer_.push_back(filtered_cloud);
        if (scan_buffer_.size() > static_cast<size_t>(num_accumulated_scans_)) {
            scan_buffer_.pop_front();
        }

        if (scan_buffer_.size() < static_cast<size_t>(num_accumulated_scans_)) {
            RCLCPP_DEBUG(this->get_logger(), "Accumulating: %zu/%d scans", 
                        scan_buffer_.size(), num_accumulated_scans_);
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& cloud : scan_buffer_) {
            *accumulated_cloud += *cloud;
        }

        RCLCPP_DEBUG(this->get_logger(), "Accumulated %zu points from %d scans", 
                    accumulated_cloud->size(), num_accumulated_scans_);

        std::vector<lslidar_msgs::msg::DetectedObject> detections;
        detectObjects(accumulated_cloud, current_msg->header, detections);
        if (enable_tracking_) {
            updateTracker(detections, current_msg->header.stamp);
        } else {
            lslidar_msgs::msg::DetectedObjects objects_msg;
            objects_msg.header = current_msg->header;
            objects_msg.objects = detections;
            objects_pub_->publish(objects_msg);
        }
    }

    void clusterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                     std::vector<pcl::PointIndices>& cluster_indices)
    {
        if (cloud->empty()) return;

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_tolerance_ * 0.5);
        ec.setMinClusterSize(std::max(3, cluster_min_size_ / 2));
        ec.setMaxClusterSize(cluster_max_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
    }

    void convertToLaserScan(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            const std_msgs::msg::Header& header)
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

        int num_ranges = 360;
        scan_msg.ranges.assign(num_ranges, std::numeric_limits<float>::quiet_NaN());
        scan_msg.intensities.assign(num_ranges, 0.0f);

        for (const auto& point : cloud->points) {
            double angle = std::atan2(point.y, point.x);
            int index = static_cast<int>((angle + M_PI) / scan_msg.angle_increment);
            if (index >= num_ranges) index = 0;

            float range = std::sqrt(point.x * point.x + point.y * point.y);
            if (range >= range_min_ && range <= range_max_) {
                if (std::isnan(scan_msg.ranges[index]) || range < scan_msg.ranges[index]) {
                    scan_msg.ranges[index] = range;
                    scan_msg.intensities[index] = point.intensity;
                }
            }
        }

        scan_pub_->publish(scan_msg);
    }

    // Member variables
    std::string input_topic_;
    std::string output_scan_topic_;
    std::string output_cloud_topic_;
    double range_min_, range_max_;
    double z_min_, z_max_;
    double voxel_leaf_size_;
    int cluster_min_size_, cluster_max_size_;
    double cluster_tolerance_;
    bool publish_filtered_cloud_;
    bool detect_objects_;
    bool classify_surfaces_;
    bool system_enabled_;
    bool processing_active_;
    std::mutex mutex_;

    bool enable_circle_detection_;
    double expected_diameter_;
    double detection_tolerance_;
    int num_accumulated_scans_;

    bool enable_tracking_;
    double track_max_association_dist_;
    int track_max_invisible_;
    int next_track_id_;
    std::vector<Track> tracks_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<lslidar_msgs::msg::DetectedObjects>::SharedPtr tracked_objects_pub_;

    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> scan_buffer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}