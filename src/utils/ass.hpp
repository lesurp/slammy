#ifndef ASS_HPP_
#define ASS_HPP_

#ifdef SLAMMY_NO_DBG

#define ASS(x) SLAMMY_EMPTY
#define ASS_EQ(x, y) SLAMMY_EMPTY

#else
namespace slammy::utils::ass {
void assert_impl(bool val, char const *stringified_val, char const *fn,
                 char const *msg = nullptr);
}

#define ASS(...) ASS_SELECTOR(__VA_ARGS__, ASS_MSG, ASS_NOMSG)(__VA_ARGS__)

#define ASS_SELECTOR(_1, _2, selected, ...) selected

#define ASS_NOMSG(x)                                                           \
  slammy::utils::ass::assert_impl(static_cast<bool>(x), #x, __PRETTY_FUNCTION__)

#define ASS_MSG(x, y)                                                          \
  slammy::utils::ass::assert_impl(static_cast<bool>(x), #x,                    \
                                  __PRETTY_FUNCTION__, y)

#define ASS_EQ(x, y)                                                           \
  slammy::utils::ass::assert_impl(x == y, #x " == " #y, __PRETTY_FUNCTION__)

#endif

#endif // ASS_HPP_
