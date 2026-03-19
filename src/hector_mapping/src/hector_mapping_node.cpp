/**
 * hector_mapping_node.cpp
 * =======================
 * ROS 2 Hector SLAM node.
 *
 * Subscribes to /scan (sensor_msgs/LaserScan).
 * Publishes  /map  (nav_msgs/OccupancyGrid)  and the  map → base_link  TF.
 * Parameters are loaded from hector_mapping_params.yaml.
 */

#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "hector_mapping/occupancy_grid_map.hpp"
#include "hector_mapping/scan_matcher.hpp"

using namespace std::chrono_literals;

namespace hector_mapping
{

class HectorMappingNode : public rclcpp::Node
{
public:
  HectorMappingNode()
  : Node("hector_mapping"), first_scan_(true)
  {
    // ── Declare & read parameters ────────────────────────────────
    this->declare_parameter("base_frame",          "base_link");
    this->declare_parameter("odom_frame",          "base_link");
    this->declare_parameter("map_frame",           "map");
    this->declare_parameter("map_update_distance_thresh", 0.1);
    this->declare_parameter("map_update_angle_thresh",    0.04);
    this->declare_parameter("map_resolution",      0.05);
    this->declare_parameter("map_size",            2048);
    this->declare_parameter("map_start_x",         0.5);
    this->declare_parameter("map_start_y",         0.5);
    this->declare_parameter("laser_min_dist",      0.1);
    this->declare_parameter("laser_max_dist",      12.0);
    this->declare_parameter("update_factor_free",  0.4);
    this->declare_parameter("update_factor_occupied", 0.9);
    this->declare_parameter("pub_map_odom_transform", true);
    this->declare_parameter("map_pub_period",      1.0);
    this->declare_parameter("scan_topic",          "/scan");
    this->declare_parameter("map_topic",           "/map");
    this->declare_parameter("map_multi_res_levels", 3);

    base_frame_  = this->get_parameter("base_frame").as_string();
    odom_frame_  = this->get_parameter("odom_frame").as_string();
    map_frame_   = this->get_parameter("map_frame").as_string();
    dist_thresh_ = this->get_parameter("map_update_distance_thresh").as_double();
    angle_thresh_= this->get_parameter("map_update_angle_thresh").as_double();
    laser_min_   = this->get_parameter("laser_min_dist").as_double();
    laser_max_   = this->get_parameter("laser_max_dist").as_double();
    pub_tf_      = this->get_parameter("pub_map_odom_transform").as_bool();

    int    map_sz  = this->get_parameter("map_size").as_int();
    double map_res = this->get_parameter("map_resolution").as_double();
    double sx      = this->get_parameter("map_start_x").as_double();
    double sy      = this->get_parameter("map_start_y").as_double();
    double uf      = this->get_parameter("update_factor_free").as_double();
    double uo      = this->get_parameter("update_factor_occupied").as_double();

    std::string scan_topic = this->get_parameter("scan_topic").as_string();
    std::string map_topic  = this->get_parameter("map_topic").as_string();
    double map_pub_period   = this->get_parameter("map_pub_period").as_double();

    int multi_res = this->get_parameter("map_multi_res_levels").as_int();

    // ── Build multi-resolution maps ──────────────────────────────
    // Level 0 = finest (original resolution).
    // Each subsequent level has half the resolution (double cell size).
    for (int l = 0; l < multi_res; ++l) {
      int sz = map_sz >> l;     // halve grid each level
      double res = map_res * (1 << l);
      maps_.emplace_back(
          std::make_unique<OccupancyGridMap>(sz, res, sx, sy, uf, uo));
    }

    pose_ = Eigen::Vector3d::Zero();
    last_update_pose_ = Eigen::Vector3d(1e10, 1e10, 1e10);   // force first update

    // ── ROS 2 plumbing ──────────────────────────────────────────
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rclcpp::QoS scan_qos(10);
    scan_qos.best_effort();
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, scan_qos,
        std::bind(&HectorMappingNode::scanCallback, this, std::placeholders::_1));

    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos);

    auto period_ms = std::chrono::milliseconds(static_cast<int>(map_pub_period * 1000));
    map_timer_ = this->create_wall_timer(
        period_ms, std::bind(&HectorMappingNode::publishMap, this));

    RCLCPP_INFO(this->get_logger(),
        "Hector Mapping started — map %dx%d @ %.3f m/cell, %d levels",
        map_sz, map_sz, map_res, multi_res);
  }

private:
  // ── Scan callback ──────────────────────────────────────────────
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Convert LaserScan → local-frame endpoints (filter by range)
    std::vector<Eigen::Vector2d> endpoints;
    endpoints.reserve(msg->ranges.size());
    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
      double r = msg->ranges[i];
      if (!std::isfinite(r) || r < laser_min_ || r > laser_max_) continue;
      endpoints.emplace_back(r * std::cos(angle), r * std::sin(angle));
    }
    if (endpoints.size() < 10) return;     // not enough valid points

    // ── Multi-resolution scan matching ───────────────────────
    // Match coarse-to-fine for robustness.
    Eigen::Vector3d new_pose = pose_;
    for (int l = static_cast<int>(maps_.size()) - 1; l >= 0; --l) {
      ScanMatcher::match(new_pose, *maps_[l], endpoints, 20);
    }
    pose_ = new_pose;

    // ── Check if map should be updated ───────────────────────
    Eigen::Vector2d d = pose_.head<2>() - last_update_pose_.head<2>();
    double da = std::abs(pose_.z() - last_update_pose_.z());
    if (first_scan_ || d.norm() > dist_thresh_ || da > angle_thresh_) {
      first_scan_ = false;
      last_update_pose_ = pose_;

      // Compute world-frame endpoints
      std::vector<Eigen::Vector2d> world_pts;
      world_pts.reserve(endpoints.size());
      for (auto & pt : endpoints) {
        world_pts.push_back(ScanMatcher::transformPoint(pose_, pt));
      }
      Eigen::Vector2d sensor_world(pose_.x(), pose_.y());

      // Update every resolution level
      for (auto & m : maps_) {
        m->updateByScan(sensor_world, world_pts);
      }
    }

    // ── Publish TF: map → odom_frame (== base_link) ─────────
    if (pub_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = msg->header.stamp;
      t.header.frame_id = map_frame_;
      t.child_frame_id  = odom_frame_;
      t.transform.translation.x = pose_.x();
      t.transform.translation.y = pose_.y();
      t.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, pose_.z());
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(t);
    }
  }

  // ── Publish the finest-level map as OccupancyGrid ─────────────
  void publishMap()
  {
    if (first_scan_) return;   // don't publish until we have data

    auto & finest = *maps_[0];
    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.stamp = this->now();
    msg->header.frame_id = map_frame_;
    msg->info.resolution = finest.resolution();
    msg->info.width  = finest.size();
    msg->info.height = finest.size();
    msg->info.origin.position.x = finest.originX();
    msg->info.origin.position.y = finest.originY();
    msg->info.origin.position.z = 0.0;
    msg->info.origin.orientation.w = 1.0;

    int total = finest.size() * finest.size();
    msg->data.resize(total);
    auto & logodds = finest.logOdds();
    for (int i = 0; i < total; ++i) {
      if (std::abs(logodds[i]) < 1e-6) {
        msg->data[i] = -1;   // unknown
      } else {
        double p = 1.0 / (1.0 + std::exp(-logodds[i]));
        msg->data[i] = static_cast<int8_t>(p * 100.0);
      }
    }
    map_pub_->publish(std::move(msg));
  }

  // ── Members ───────────────────────────────────────────────────
  std::vector<std::unique_ptr<OccupancyGridMap>> maps_;
  Eigen::Vector3d pose_;
  Eigen::Vector3d last_update_pose_;
  bool first_scan_;

  std::string base_frame_, odom_frame_, map_frame_;
  double dist_thresh_, angle_thresh_;
  double laser_min_, laser_max_;
  bool pub_tf_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_pub_;
  rclcpp::TimerBase::SharedPtr                                  map_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
};

}  // namespace hector_mapping

// ── main ───────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hector_mapping::HectorMappingNode>());
  rclcpp::shutdown();
  return 0;
}
