#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include "common.hpp"
#include <opencv2/core.hpp>

#ifdef SLAMMY_NO_DBG

#define DBG_LOOP() SLAMMY_EMPTY
#define DBG_KPTS(img, kpts) SLAMMY_EMPTY

#else

#define DBG_LOOP() slammy::utils::debug::wait_loop()

#define DBG_KPTS(img, kpts) slammy::utils::debug::dbg_kpts_impl(img, #img, kpts)

namespace slammy::utils::debug {
void dbg_kpts_impl(cv::Mat const &frame, char const *frame_name,
                   std::vector<cv::KeyPoint> const &kpts);

void wait_loop();
} // namespace slammy::utils::debug
#endif

#endif // DEBUG_HPP_
