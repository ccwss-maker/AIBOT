#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

struct FilterRange {
    double angle_min;
    double angle_max; 
    double range_min;
    double range_max;
};


class LaserScanFilter {
private:
    ros::NodeHandle nh_;
    ros::Subscriber front_sub_;
    ros::Subscriber back_sub_;
    
    // Publishers for filtered laser scans
    ros::Publisher front_filtered_pub_;
    ros::Publisher back_filtered_pub_;
    
    // Publishers for visualization markers
    ros::Publisher front_marker_pub_;
    ros::Publisher back_marker_pub_;
    
    std::vector<FilterRange> front_filters_;
    std::vector<FilterRange> back_filters_;
    
    std::string front_topic_;
    std::string back_topic_;
    std::string front_filtered_topic_;
    std::string back_filtered_topic_;
    std::string front_marker_topic_;
    std::string back_marker_topic_;

public:
    LaserScanFilter() {
        loadConfig();
        
        front_sub_ = nh_.subscribe(front_topic_, 1, &LaserScanFilter::frontCallback, this);
        back_sub_ = nh_.subscribe(back_topic_, 1, &LaserScanFilter::backCallback, this);
        
        // Initialize publishers
        front_filtered_pub_ = nh_.advertise<sensor_msgs::LaserScan>(front_filtered_topic_, 1);
        back_filtered_pub_ = nh_.advertise<sensor_msgs::LaserScan>(back_filtered_topic_, 1);
        
        // Initialize marker publishers
        front_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(front_marker_topic_, 1);
        back_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(back_marker_topic_, 1);
        
        ROS_INFO("LaserScanFilter initialized");
        ROS_INFO("Front topic: %s", front_topic_.c_str());
        ROS_INFO("Back topic: %s", back_topic_.c_str());
        ROS_INFO("Publishing filtered laser scans on:");
        ROS_INFO("  - %s", front_filtered_topic_.c_str());
        ROS_INFO("  - %s", back_filtered_topic_.c_str());
        ROS_INFO("Publishing filter visualization markers on:");
        ROS_INFO("  - %s", front_marker_topic_.c_str());
        ROS_INFO("  - %s", back_marker_topic_.c_str());
    }
    
    void loadConfig() {
        std::string config_file;
        std::string package_path = ros::package::getPath("laser_scan_filter");
        std::string default_config = package_path + "/config/filter_config.yaml";
        nh_.param<std::string>("config_file", config_file, default_config);
        
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            
            // Load front laser config
            auto front_config = config["front_laser"];
            front_topic_ = front_config["topic"].as<std::string>();
            front_filtered_topic_ = front_config["filtered_topic"].as<std::string>();
            front_marker_topic_ = front_config["marker_topic"].as<std::string>();
            
            for (const auto& range : front_config["filter_ranges"]) {
                FilterRange fr;
                fr.angle_min = range["angle_min"].as<double>();
                fr.angle_max = range["angle_max"].as<double>();
                fr.range_min = range["range_min"].as<double>();
                fr.range_max = range["range_max"].as<double>();
                front_filters_.push_back(fr);
                ROS_INFO("Front filter: angle %.1f-%.1f deg, range %.2f-%.2f m", 
                         fr.angle_min, fr.angle_max, fr.range_min, fr.range_max);
            }
            
            // Load back laser config
            auto back_config = config["back_laser"];
            back_topic_ = back_config["topic"].as<std::string>();
            back_filtered_topic_ = back_config["filtered_topic"].as<std::string>();
            back_marker_topic_ = back_config["marker_topic"].as<std::string>();
            
            for (const auto& range : back_config["filter_ranges"]) {
                FilterRange bk;
                bk.angle_min = range["angle_min"].as<double>();
                bk.angle_max = range["angle_max"].as<double>();
                bk.range_min = range["range_min"].as<double>();
                bk.range_max = range["range_max"].as<double>();
                back_filters_.push_back(bk);
                ROS_INFO("Back filter: angle %.1f-%.1f deg, range %.2f-%.2f m", 
                         bk.angle_min, bk.angle_max, bk.range_min, bk.range_max);
            }
            
            
        } catch (const YAML::Exception& e) {
            ROS_ERROR("Error loading config file: %s", e.what());
            throw;
        }
    }
    
    visualization_msgs::MarkerArray createFilterMarkers(const std::vector<FilterRange>& filters, 
                                                        const std::string& frame_id,
                                                        const std::string& ns,
                                                        double r, double g, double b) {
        visualization_msgs::MarkerArray marker_array;
        
        for (size_t i = 0; i < filters.size(); ++i) {
            const FilterRange& filter = filters[i];
            
            // Create sector marker for the filter range
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = ns;
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1;
            
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            
            marker.color.r = static_cast<float>(r);
            marker.color.g = static_cast<float>(g);
            marker.color.b = static_cast<float>(b);
            marker.color.a = 0.3f; // Semi-transparent
            
            // Convert angles from degrees to radians
            double angle_min_rad = filter.angle_min * M_PI / 180.0;
            double angle_max_rad = filter.angle_max * M_PI / 180.0;
            
            // Normalize angles to -PI to PI range
            while (angle_min_rad < -M_PI) angle_min_rad += 2*M_PI;
            while (angle_max_rad < -M_PI) angle_max_rad += 2*M_PI;
            while (angle_min_rad > M_PI) angle_min_rad -= 2*M_PI;
            while (angle_max_rad > M_PI) angle_max_rad -= 2*M_PI;
            
            // Create sector with multiple triangles for smooth visualization
            int num_segments = 20;
            double angle_step;
            
            if (angle_min_rad <= angle_max_rad) {
                angle_step = (angle_max_rad - angle_min_rad) / num_segments;
            } else {
                // Handle wrap-around case
                angle_step = ((angle_max_rad + 2*M_PI) - angle_min_rad) / num_segments;
            }
            
            // Center point
            geometry_msgs::Point center;
            center.x = 0;
            center.y = 0;
            center.z = 0;
            
            // Create triangular sectors
            for (int j = 0; j < num_segments; ++j) {
                double current_angle = angle_min_rad + j * angle_step;
                double next_angle = angle_min_rad + (j + 1) * angle_step;
                
                // Handle wrap-around
                while (current_angle > M_PI) current_angle -= 2*M_PI;
                while (next_angle > M_PI) next_angle -= 2*M_PI;
                
                // Inner radius points
                geometry_msgs::Point p1_inner, p2_inner;
                p1_inner.x = filter.range_min * cos(current_angle);
                p1_inner.y = filter.range_min * sin(current_angle);
                p1_inner.z = 0;
                
                p2_inner.x = filter.range_min * cos(next_angle);
                p2_inner.y = filter.range_min * sin(next_angle);
                p2_inner.z = 0;
                
                // Outer radius points
                geometry_msgs::Point p1_outer, p2_outer;
                p1_outer.x = filter.range_max * cos(current_angle);
                p1_outer.y = filter.range_max * sin(current_angle);
                p1_outer.z = 0;
                
                p2_outer.x = filter.range_max * cos(next_angle);
                p2_outer.y = filter.range_max * sin(next_angle);
                p2_outer.z = 0;
                
                // Create two triangles to form a sector segment
                // Triangle 1: inner1 -> inner2 -> outer1
                marker.points.push_back(p1_inner);
                marker.points.push_back(p2_inner);
                marker.points.push_back(p1_outer);
                
                // Triangle 2: inner2 -> outer2 -> outer1
                marker.points.push_back(p2_inner);
                marker.points.push_back(p2_outer);
                marker.points.push_back(p1_outer);
            }
            
            marker_array.markers.push_back(marker);
        }
        
        return marker_array;
    }
    
    
    
    sensor_msgs::LaserScan createFilteredLaserScan(const sensor_msgs::LaserScan::ConstPtr& original_msg,
                                                   const std::vector<FilterRange>& filters) {
        sensor_msgs::LaserScan filtered_scan = *original_msg;
        
        double angle_increment = original_msg->angle_increment;
        double angle_min = original_msg->angle_min;
        
        // Remove points that fall within the specified filter ranges
        for (const auto& config : filters) {
            double angle_min_deg = config.angle_min;
            double angle_max_deg = config.angle_max;
            
            // Normalize angles to 0-360 range
            while (angle_min_deg < 0) angle_min_deg += 360;
            while (angle_max_deg < 0) angle_max_deg += 360;
            while (angle_min_deg >= 360) angle_min_deg -= 360;
            while (angle_max_deg >= 360) angle_max_deg -= 360;
            
            // Convert to radians
            double angle_min_rad = angle_min_deg * M_PI / 180.0;
            double angle_max_rad = angle_max_deg * M_PI / 180.0;
            
            // For each point in the scan, check if it should be filtered
            for (int i = 0; i < static_cast<int>(original_msg->ranges.size()); ++i) {
                double point_angle_rad = angle_min + i * angle_increment;
                double point_angle_deg = point_angle_rad * 180.0 / M_PI;
                
                // Normalize point angle to 0-360 range
                while (point_angle_deg < 0) point_angle_deg += 360;
                while (point_angle_deg >= 360) point_angle_deg -= 360;
                
                bool angle_match = false;
                
                // Check if point angle is within filter range
                if (angle_min_deg <= angle_max_deg) {
                    // Normal case: range doesn't cross 0 degrees
                    angle_match = (point_angle_deg >= angle_min_deg && point_angle_deg <= angle_max_deg);
                } else {
                    // Range crosses 0 degrees (e.g., 350-10)
                    angle_match = (point_angle_deg >= angle_min_deg || point_angle_deg <= angle_max_deg);
                }
                
                if (angle_match) {
                    double distance = original_msg->ranges[i];
                    
                    // Filter invalid points (NaN, infinity, negative, or in specified range)
                    if (!std::isfinite(distance) || distance <= 0 ||
                        (distance >= config.range_min && distance <= config.range_max)) {
                        filtered_scan.ranges[i] = std::numeric_limits<float>::infinity();
                        
                        // Debug: Log filtered points
                        // if (std::isfinite(distance) && distance > 0 && distance <= 1.5) {
                        //     ROS_INFO("Filtered close point: idx=%d, angle=%.1fdeg, distance=%.3f", 
                        //             i, point_angle_deg, distance);
                        // }
                    }
                }
            }
        }
        
        return filtered_scan;
    }
    
    
    void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        sensor_msgs::LaserScan filtered_scan = createFilteredLaserScan(msg, front_filters_);
        front_filtered_pub_.publish(filtered_scan);
        
        // Publish filter visualization markers
        visualization_msgs::MarkerArray front_markers = createFilterMarkers(
            front_filters_, msg->header.frame_id, "front_filter", 1.0, 0.0, 0.0); // Red
        front_marker_pub_.publish(front_markers);
    }
    
    void backCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        sensor_msgs::LaserScan filtered_scan = createFilteredLaserScan(msg, back_filters_);
        back_filtered_pub_.publish(filtered_scan);
        
        // Publish filter visualization markers
        visualization_msgs::MarkerArray back_markers = createFilterMarkers(
            back_filters_, msg->header.frame_id, "back_filter", 0.0, 0.0, 1.0); // Blue
        back_marker_pub_.publish(back_markers);
    }
    
    
    void run() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_filter");
    
    try {
        LaserScanFilter filter;
        filter.run();
    } catch (const std::exception& e) {
        ROS_ERROR("LaserScanFilter error: %s", e.what());
        return -1;
    }
    
    return 0;
}