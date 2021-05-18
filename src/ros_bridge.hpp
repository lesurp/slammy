#ifndef ROS_BRIDGE_HPP_
#define ROS_BRIDGE_HPP_

#include "camera_driver/camera_driver.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "pose.hpp"
#include "pose_graph.hpp"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "utils/ass.hpp"
#include "utils/fmt.hpp"
#include <atomic>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <spdlog/spdlog.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace slammy::ros_bridge {
class RvizBridge {
public:
  using PointDataType = float;

  sensor_msgs::PointField::_datatype_type constexpr type2flag() {
    if constexpr (std::is_same_v<PointDataType, double>) {
      return sensor_msgs::PointField::FLOAT64;
    } else if (std::is_same_v<PointDataType, float>) {
      return sensor_msgs::PointField::FLOAT32;
    }
  }

  RvizBridge(ros::NodeHandle &n)
      : _path_publisher(n.advertise<nav_msgs::Path>("slammy_path", 5)),
        _pose_publisher(
            n.advertise<geometry_msgs::PoseStamped>("slammy_pose", 5)),
        _points_publisher(
            n.advertise<sensor_msgs::PointCloud2>("slammy_points", 1)) {

    _live_path.header.frame_id = "map";
    _live_path.header.seq = 0;

    _last_pose = geometry_msgs::PoseStamped();
    _last_pose.header.frame_id = "map";
    _last_pose.header.seq = 0;

    _points.header.frame_id = "map";
    _points.header.seq = 0;
    _points.height = 1;
    _points.is_dense = false;
    _points.is_bigendian = false;

    auto mod = sensor_msgs::PointCloud2Modifier{_points};
    mod.setPointCloud2FieldsByString(1, "xyz");
  }

  void publish(slammy::pose::PoseCW const &pose) {
    spdlog::info("Publisher got pose: {}", pose);
    auto const &[t, q] = pose;
    _last_pose.pose.orientation.w = q.w();
    _last_pose.pose.orientation.x = q.x();
    _last_pose.pose.orientation.y = q.y();
    _last_pose.pose.orientation.z = q.z();
    _last_pose.pose.position.x = t.x();
    _last_pose.pose.position.y = t.y();
    _last_pose.pose.position.z = t.z();
    _live_path.poses.emplace_back(_last_pose);
    _path_publisher.publish(_live_path);
    _pose_publisher.publish(_last_pose);
  }

  void publish(std::vector<Eigen::Vector3d> const &points) {
    ASS(!points.empty());
    auto mod = sensor_msgs::PointCloud2Modifier{_points};
    mod.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> out_x(_points, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(_points, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(_points, "z");

    for (auto const &p : points) {
      *out_x = p.x();
      *out_y = p.y();
      *out_z = p.z();
      ++out_x;
      ++out_y;
      ++out_z;
    }
    _points_publisher.publish(_points);
  }

  void publish(
      std::unordered_set<std::unique_ptr<pose_graph::Point>> const &points) {
    _points.width = points.size();
    _points.row_step = points.size() * _points.point_step;
    _points.data.clear();
    for (auto const &pt_ptr : points) {
      auto const &p = pt_ptr->t;
      //_points.data.emplace_back(p.x(), p.y(), p.z());
    }
    _points_publisher.publish(_points);
  }

private:
  ros::Publisher _path_publisher;
  ros::Publisher _pose_publisher;
  ros::Publisher _points_publisher;
  nav_msgs::Path _live_path;
  sensor_msgs::PointCloud2 _points;
  geometry_msgs::PoseStamped _last_pose;
};

class CameraSubscriber : public camera_driver::CameraDriver {
public:
  struct EndOfDataException {};
  CameraSubscriber(ros::NodeHandle &n, std::string const &topic,
                   uint32_t queue_size)
      : _sub(n.subscribe(topic, queue_size, CameraSubscriber::on_next_frame)) {}

  cv::Mat next_frame() override;

  void stop() override;

private:
  static cv_bridge::CvImagePtr _last_image;
  static std::mutex _last_image_mutex;
  static std::condition_variable _last_image_cv;
  ros::Subscriber _sub;
  std::atomic_bool _should_stop = false;

  static void on_next_frame(sensor_msgs::Image const &image);
};
} // namespace slammy::ros_bridge

#endif // ROS_BRIDGE_HPP_
