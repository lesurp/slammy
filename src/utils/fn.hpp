#ifndef FN_HPP_
#define FN_HPP_

#include "conversions.hpp"
#include <algorithm>

namespace slammy::utils {} // namespace slammy::utils

namespace slammy::utils::fn {
template <typename To, template <typename> class ContainerTo = std::vector,
          typename ContainerFrom,
          typename Converter = slammy::utils::conversions::Converter<
              std::remove_cv_t<std::remove_reference_t<decltype(*std::declval<ContainerFrom>().begin())>>, To>>
auto map(ContainerFrom &&from, Converter &&conv = Converter{}) {
  auto out = ContainerTo<To>{};
  out.reserve(from.size());
  std::transform(std::begin(from), std::end(from), std::back_inserter(out),
                 conv);
  return out;
}

} // namespace slammy::utils::fn

#endif // FN_HPP_
