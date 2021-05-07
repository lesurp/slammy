#include "vision.hpp"
#include <opencv2/core/eigen.hpp>

using slammy::pose::Camera;
using slammy::pose::Pose;
using slammy::pose::World;

namespace slammy::vision {
Pose<Camera, World> relative_pose(std::vector<cv::Point2f> const &pts1,
                                  std::vector<cv::Point2f> const &pts2,
                                  cv::Mat const &k_cv) {
  auto e = cv::findEssentialMat(pts1, pts2, k_cv);

  cv::Mat t;
  cv::Mat r;
  cv::recoverPose(e, pts1, pts2, k_cv, r, t);

  Eigen::Vector3d t_eigen;
  Eigen::Matrix3d r_eigen;
  cv::cv2eigen(t, t_eigen);
  cv::cv2eigen(r, r_eigen);
  return Pose<Camera, World>{t_eigen, Eigen::Quaterniond{r_eigen}};
}
} // namespace slammy::vision
