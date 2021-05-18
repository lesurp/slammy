#include "camera_driver/usb_camera_driver.hpp"
#include "pose.hpp"
#include "ros_bridge.hpp"
#include "tracker.hpp"
#include "utils/common.hpp"
#include "utils/debug.hpp"
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

  auto camera = [&] {
    if constexpr (true) {
      spdlog::info("Subscribed to {}", "/camera");
      return ros_bridge::CameraSubscriber{n, "/camera", 1000};
    } else {
      return camera_driver::UsbCameraDriver{0, cv::CAP_V4L2};
    }
  }();

  auto rviz = ros_bridge::RvizBridge{n};
  auto tracker = tracker::Tracker{458.654, 457.296, 367.215, 248.375};

  auto tracker_thread = std::thread([&] {
    for (;;) {
      try {
        auto next_frame = camera.next_frame();
        spdlog::info("Got next frame");
        if (next_frame.empty()) {
          spdlog::error("Frame is empty?!");
          continue;
        }

        auto state = tracker.next(next_frame);
        std::visit(
            slammy::utils::common::variant_bullshit{
                [](tracker::Initializing) {},
                [&](tracker::Tracking const &t) { rviz.publish(t.pose); },
                [&](tracker::Lost const &t) { rviz.publish(t.last_pose); },
                [&](tracker::JustInitialized const &map) {
                  for (auto const &p : map.poses) {
                    rviz.publish(p);
                  }
                  rviz.publish(map.points);
                },
            },
            state);

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
