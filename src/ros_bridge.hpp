#ifndef ROS_BRIDGE_HPP_
#define ROS_BRIDGE_HPP_

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <atomic>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <nav_msgs/Path.h>

namespace slammy::ros_bridge {
class RvizBridge {
public:
  RvizBridge(ros::NodeHandle &n)
      : _path_publisher(n.advertise<nav_msgs::Path>("slammy_path", 5)),
        _pose_publisher(
            n.advertise<geometry_msgs::PoseStamped>("slammy_pose", 5)) {

    _live_path.header.frame_id = "slammy_path";
    _live_path.header.seq = 0;

    _last_pose = geometry_msgs::PoseStamped();
    _last_pose.header.frame_id = "slammy_pose";
    _last_pose.header.seq = 0;
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
