#include "vision.hpp"
#include <eigen3/Eigen/src/Core/util/Constants.h>
#include <fmt/ostream.h>
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

slammy::pose_graph::Point triangulate(
    std::vector<
        std::pair<slammy::pose::Pose<slammy::pose::Camera, slammy::pose::World>,
                  Eigen::Vector2d>> const &observations,
    Eigen::Matrix3d const &k_inv) {

  ASS(observations.size() > 1);
  Eigen::MatrixX3d a(2 * observations.size(), 3);
  Eigen::VectorXd b(2 * observations.size());

#pragma omp parallel for
  for (size_t i = 0; i < observations.size(); ++i) {
    auto const &[pose, px] = observations[i];

    Eigen::Matrix3d r = pose.q.toRotationMatrix();
    double mn_x = k_inv(0, 0) * px.x() + k_inv(0, 2);
    double mn_y = k_inv(1, 1) * px.y() + k_inv(1, 2);

    a.row(2 * i) = r.row(0) - mn_x * r.row(2);
    a.row(2 * i + 1) = r.row(1) - mn_y * r.row(2);
    b(2 * i) = pose.t.z() * mn_x - pose.t.x();
    b(2 * i + 1) = pose.t.z() * mn_y - pose.t.y();
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU |
                                               Eigen::ComputeThinV);
  Eigen::Vector3d x = svd.solve(b);
  return slammy::pose_graph::Point{x};
}
} // namespace slammy::vision
