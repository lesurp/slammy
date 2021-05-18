#ifndef CONVERSIONS_HPP_
#define CONVERSIONS_HPP_

#include "../pose_graph.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <memory>

namespace slammy::utils::conversions {
template <typename From, typename To> struct Converter {
  Converter() = delete;
};

template <>
struct Converter<std::unique_ptr<pose_graph::Point>, Eigen::Vector3d> {
  Eigen::Vector3d operator()(std::unique_ptr<pose_graph::Point> const &pt) {
    return pt->t;
  }
};

template <> struct Converter<pose_graph::Point, Eigen::Vector3d> {
  Eigen::Vector3d operator()(pose_graph::Point const &pt) { return pt.t; }
};

template <typename FromPtr, typename To>
struct Converter<std::unique_ptr<FromPtr>, To> {
  Eigen::Vector3d operator()(std::unique_ptr<FromPtr> const &t) {
    return Converter<FromPtr, To>{}(t);
  }
};

} // namespace slammy::utils::conversions

#endif // CONVERSIONS_HPP_
