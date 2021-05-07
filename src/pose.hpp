#ifndef POSE_HPP_
#define POSE_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace slammy::pose {
struct World {};
struct Camera {};

template <typename To, typename From> struct Pose {
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  Pose<From, To> inverse() const {
    auto qj = q.conjugate();
    return Pose<From, To>{-qj._transformVector(t), std::move(qj)};
  }

  bool is_approx(Pose const &p) const {
    return (t - p.t).squaredNorm() < 1e-5 && (q.isApprox(p.q));
  }
};

template <typename A, typename B, typename C>
Pose<A, C> operator*(Pose<A, B> const &p_ab, Pose<B, C> const &p_bc) {
  Pose<A, C> p_ac;
  p_ac.q = p_ab.q * p_bc.q;
  p_ac.t = p_ab.t + p_ab.q._transformVector(p_bc.t);
  return p_ac;
}

template <typename A, typename B>
Eigen::Vector3d operator*(Pose<A, B> const &p, Eigen::Vector3d const &pt) {
  return p.q._transformVector(pt) + p.t;
}
} // namespace slammy::pose
#endif // POSE_HPP_
