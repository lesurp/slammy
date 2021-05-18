#ifndef VISION_HPP_
#define VISION_HPP_

#include "pose.hpp"
#include "pose_graph.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

namespace slammy::vision {
slammy::pose::PoseCW relative_pose(std::vector<cv::Point2f> const &pts1,
                                   std::vector<cv::Point2f> const &pts2,
                                   cv::Mat const &k_cv);

slammy::pose_graph::Point
triangulate(std::vector<std::pair<slammy::pose::PoseCW, Eigen::Vector2d>> const
                &observations,
            Eigen::Matrix3d const &k_inv);
} // namespace slammy::vision

#endif // VISION_HPP_
