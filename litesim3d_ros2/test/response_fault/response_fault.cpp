// Test-only Linux interposer. It is built in pytest's temporary directory and
// injected only into test children; it is never linked or installed in the package.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <rcl/error_handling.h>
#include <rcl/service.h>

extern "C" rcl_ret_t rcl_send_response(const rcl_service_t *service,
                                      rmw_request_id_t *header, void *response) {
  using Send = rcl_ret_t (*)(const rcl_service_t *, rmw_request_id_t *, void *);
  static const auto real_send = reinterpret_cast<Send>(dlsym(RTLD_NEXT, "rcl_send_response"));
  if (!real_send)
    std::abort();
  const char *target = std::getenv("LWS_TEST_RESPONSE_SERVICE");
  if (!target || std::strcmp(rcl_service_get_service_name(service), target) != 0)
    return real_send(service, header, response);

  static std::atomic<unsigned> sends{0};
  const auto attempt = ++sends;
  char mode[32] = {};
  if (const char *path = std::getenv("LWS_TEST_RESPONSE_FAULT")) {
    if (auto file = std::fopen(path, "r")) {
      (void)std::fgets(mode, sizeof(mode), file);
      std::fclose(file);
    }
  }
  rcl_ret_t result;
  if (std::strcmp(mode, "error") == 0) {
    RCL_SET_ERROR_MSG("injected permanent response failure");
    result = RCL_RET_ERROR;
  } else if (std::strcmp(mode, "always") == 0 || attempt <= std::strtoul(mode, nullptr, 10)) {
    RCL_SET_ERROR_MSG("injected response subscription discovery timeout");
    result = RCL_RET_TIMEOUT;
  } else {
    result = real_send(service, header, response);
  }
  if (const char *path = std::getenv("LWS_TEST_RESPONSE_TRACE")) {
    if (auto file = std::fopen(path, "a")) {
      std::fprintf(file, "{\"attempt\":%u,\"sequence\":%lld,\"result\":%d,\"guid\":\"",
                   attempt, static_cast<long long>(header->sequence_number), result);
      for (auto byte : header->writer_guid)
        std::fprintf(file, "%02x", static_cast<unsigned char>(byte));
      std::fputs("\"}\n", file);
      std::fclose(file);
    }
  }
  return result;
}
