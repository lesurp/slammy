#include "ros_bridge.hpp"
#include <spdlog/spdlog.h>

namespace slammy::ros_bridge {
cv_bridge::CvImagePtr CameraSubscriber::_last_image = {};
std::mutex CameraSubscriber::_last_image_mutex = {};
std::condition_variable CameraSubscriber::_last_image_cv = {};

cv_bridge::CvImagePtr CameraSubscriber::next_frame() {
  spdlog::info("Waiting for next frame");
  auto l = std::unique_lock{_last_image_mutex};
  _last_image_cv.wait(l, [&] {
    if (_should_stop) {
      throw EndOfDataException{};
    }
    return _last_image;
  });
  spdlog::info("Got it");
  return std::move(_last_image);
}

void CameraSubscriber::stop() {
  _should_stop = true;
  _last_image_cv.notify_one();
}

void CameraSubscriber::on_next_frame(sensor_msgs::Image const &image) {
  spdlog::info("on_next_frame called");
  {
    auto l = std::lock_guard{_last_image_mutex};
    _last_image =
        cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  }
  _last_image_cv.notify_one();
}

} // namespace slammy::ros_bridge
