#include "ass.hpp"
#include <signal.h>
#include <spdlog/spdlog.h>

namespace slammy::utils::ass {
void assert_impl(bool val, char const *stringified_val, char const *fn,
                 char const *msg) {
  if (val) {
    return;
  }

  spdlog::critical("Assertion `{}` failed at {}", stringified_val, fn);
  if (msg) {
    spdlog::critical("Additional information: {}", msg);
  }

  // use with LD_PRELOAD=/lib/libSegFault.so !
  kill(getpid(), SIGSEGV);
}

} // namespace slammy::utils::details
