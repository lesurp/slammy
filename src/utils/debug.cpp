#include "debug.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace slammy::utils::debug {
void dbg_kpts_impl(cv::Mat const &frame, char const *frame_name,
                   std::vector<cv::KeyPoint> const &kpts) {
  auto f = frame.clone();
  for (auto const &kp : kpts) {
    cv::circle(f, kp.pt, 2, cv::Scalar(0, 255, 0));
  }
  cv::imshow(frame_name, f);
}

void wait_loop() { cv::waitKey(1); }
} // namespace slammy::utils::details
