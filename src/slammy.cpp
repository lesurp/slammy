#include "pose.hpp"
#include "ros_bridge.hpp"
#include "tracker.hpp"
#include "utils.hpp"
#include <opencv4/opencv2/core/types.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <vector>

using namespace slammy;

int main(int argc, char **argv) {
  ros::init(argc, argv, "slammy");
  ros::NodeHandle n;

  auto camera = ros_bridge::CameraSubscriber{n, "/camera", 1000};
  auto rviz = ros_bridge::RvizBridge{n};
  spdlog::info("Subscribed to {}", "/camera");

  auto tracker = tracker::Tracker{458.654, 457.296, 367.215, 248.375};

  auto tracker_thread = std::thread([&] {
    for (;;) {
      try {
        auto next_frame = camera.next_frame();
        spdlog::info("Got next frame");

        auto state = tracker.next(next_frame->image);
        std::visit(
            slammy::utils::variant_bullshit{
                [](tracker::Initializing) {},
                [&](tracker::Tracking const &t) { rviz.publish(t.pose); },
                [&](tracker::Lost const &t) { rviz.publish(t.last_pose); },
                [&](tracker::JustInitialized const &map) {
                  for (auto const &p : map.poses) {
                    rviz.publish(p);
                  }
                },
            },
            state);

        // path.poses.clear();
        /*
        for (auto const &kf : tracker.kfs()) {
          auto const &[t, q] = kf.pose;
          last_pose.pose.orientation.w = q.w();
          last_pose.pose.orientation.x = q.x();
          last_pose.pose.orientation.y = q.y();
          last_pose.pose.orientation.z = q.z();
          last_pose.pose.position.x = t.x();
          last_pose.pose.position.y = t.y();
          last_pose.pose.position.z = t.z();
          path.poses.emplace_back(last_pose);
        }
        */
        // path_displayer.publish(path);
        DBG_LOOP();
      } catch (ros_bridge::CameraSubscriber::EndOfDataException const &) {
        spdlog::info("Camera ran out of data. Exiting tracker thread");
        return;
      }
    }
  });

  ros::spin();
  camera.stop();
  spdlog::info("ROS looped finished, joining on threads");

  tracker_thread.join();
  return 0;
}
