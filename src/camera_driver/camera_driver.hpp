#ifndef CAMERA_DRIVER_HPP_
#define CAMERA_DRIVER_HPP_

#include <opencv4/opencv2/core/types.hpp>

namespace camera_driver {
struct CameraDriver {
  virtual cv::Mat next_frame() = 0;
  virtual void stop() {}
};
} // namespace camera_driver

#endif // CAMERA_DRIVER_HPP_
