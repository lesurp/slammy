#ifndef USB_CAMERA_DRIVER_HPP_
#define USB_CAMERA_DRIVER_HPP_

#include "camera_driver.hpp"
#include <opencv2/videoio.hpp>
#include <opencv4/opencv2/core/types.hpp>

namespace camera_driver {
class UsbCameraDriver : public CameraDriver {
public:
  template <typename... Args>
  UsbCameraDriver(Args &&...args)
      : _video_capture(std::forward<Args>(args)...) {}
  cv::Mat next_frame() override {
    _video_capture.retrieve(_buf);
    return _buf.clone();
  }

private:
  cv::VideoCapture _video_capture;
  cv::Mat _buf;
};
} // namespace camera_driver

#endif // USB_CAMERA_DRIVER_HPP_
