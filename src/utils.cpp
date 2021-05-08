#include "utils.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <signal.h>
#include <spdlog/spdlog.h>

namespace slammy::utils::details {
void dbg_kpts_impl(cv::Mat const &frame, char const *frame_name,
                   std::vector<cv::KeyPoint> const &kpts) {
  auto f = frame.clone();
  for (auto const &kp : kpts) {
    cv::circle(f, kp.pt, 2, cv::Scalar(0, 255, 0));
  }
  cv::imshow(frame_name, f);
}

void assert_impl(bool val, char const *stringified_val, char const *fn,
                 char const *msg) {
  if (val) {
    return;
  }

  spdlog::critical("Assertion `{}` failed at {}", stringified_val, fn);
  if (msg) {
    spdlog::critical("Additional information: {}", msg);
  }

  // use with LD_PRELOAD=/lib/libSegFault.so !
  kill(getpid(), SIGSEGV);
}

void wait_loop() { cv::waitKey(1); }
} // namespace slammy::utils::details
