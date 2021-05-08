#ifndef ROS_BRIDGE_HPP_
#define ROS_BRIDGE_HPP_

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "pose.hpp"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <atomic>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/Path.h>

namespace slammy::ros_bridge {
class RvizBridge {
public:
  RvizBridge(ros::NodeHandle &n)
      : _path_publisher(n.advertise<nav_msgs::Path>("slammy_path", 5)),
        _pose_publisher(
            n.advertise<geometry_msgs::PoseStamped>("slammy_pose", 5)) {
            /*

    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = static_turtle_name;
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    static_transformStamped.transform.rotation.x = 0;
    static_transformStamped.transform.rotation.y = 0;
    static_transformStamped.transform.rotation.z = 0;
    static_transformStamped.transform.rotation.w = 1;
    static_broadcaster.sendTransform(static_transformStamped);
    */

    _live_path.header.frame_id = "map";
    _live_path.header.seq = 0;

    _last_pose = geometry_msgs::PoseStamped();
    _last_pose.header.frame_id = "map";
    _last_pose.header.seq = 0;
  }

  void publish(slammy::pose::PoseCW const &pose) {
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

private:
  ros::Publisher _path_publisher;
  ros::Publisher _pose_publisher;
  nav_msgs::Path _live_path;
  geometry_msgs::PoseStamped _last_pose;
};

class CameraSubscriber {
public:
  struct EndOfDataException {};
  CameraSubscriber(ros::NodeHandle &n, std::string const &topic,
                   uint32_t queue_size)
      : _sub(n.subscribe(topic, queue_size, CameraSubscriber::on_next_frame)) {}

  cv_bridge::CvImagePtr next_frame();

  void stop();

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
