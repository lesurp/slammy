#include "vision.hpp"
#include <eigen3/Eigen/src/Core/util/Constants.h>
#include <fmt/ostream.h>
#include <opencv2/core/eigen.hpp>

using slammy::pose::PoseCW;

namespace slammy::vision {
PoseCW relative_pose(std::vector<cv::Point2f> const &pts1,
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
  return PoseCW{t_eigen, Eigen::Quaterniond{r_eigen}};
}

slammy::pose_graph::Point triangulate(
    std::vector<
        std::pair<slammy::pose::PoseCW,
                  Eigen::Vector2d>> const &observations,
    Eigen::Matrix3d const &k_inv) {

  ASS(observations.size() > 1);
  Eigen::MatrixX4d a(2 * observations.size(), 4);

#pragma omp parallel for
  for (size_t i = 0; i < observations.size(); ++i) {
    auto const &[pose, px] = observations[i];

    Eigen::Matrix3d r = pose.q.toRotationMatrix();
    Eigen::Vector2d mn = k_inv.col(2).head<2>() + k_inv.block<2, 2>(0, 0) * px;

    a.block<2, 3>(2 * i, 0) = r.block<2, 3>(0, 0) - mn * r.row(2);
    a.block<2, 1>(2 * i, 3) = pose.t.head<2>() - pose.t.z() * mn;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU |
                                               Eigen::ComputeFullV);
  Eigen::Vector3d x = svd.matrixV().col(3).hnormalized();
  //Eigen::VectorXd residual = a * x.homogeneous();
  return slammy::pose_graph::Point{x, {}};
}
} // namespace slammy::vision
