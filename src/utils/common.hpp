#ifndef COMMON_HPP_
#define COMMON_HPP_

#define SLAMMY_EMPTY                                                           \
  do {                                                                         \
  } while (false)

namespace slammy::utils::common {

template <class... Ts> struct variant_bullshit : Ts... {
  using Ts::operator()...;
};
template <class... Ts> variant_bullshit(Ts...) -> variant_bullshit<Ts...>;

} // namespace slammy::utils::common

#endif // COMMON_HPP_
