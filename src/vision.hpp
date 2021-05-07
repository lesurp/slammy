#ifndef VISION_HPP_
#define VISION_HPP_

#include "pose.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

namespace slammy::vision {
slammy::pose::Pose<slammy::pose::Camera, slammy::pose::World>
relative_pose(std::vector<cv::Point2f> const &pts1,
              std::vector<cv::Point2f> const &pts2, cv::Mat const &k_cv);
} // namespace slammy::vision

#endif // VISION_HPP_
