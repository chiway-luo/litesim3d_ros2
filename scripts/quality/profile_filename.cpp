// Coverage-build plumbing only: this file is compiled WITHOUT profiling or
// coverage mapping. It changes filenames, never counters or runtime behavior.
#include <cstdlib>
#include <cstring>

// compiler-rt keeps one hidden profiling runtime in each ELF image. Binding
// locally is essential: one plugin must not rename another plugin's output.
extern "C" __attribute__((visibility("hidden")))
void __llvm_profile_set_filename(const char*);

namespace {
__attribute__((constructor)) void select_module_profile_filename() {
  const char* pattern = std::getenv("LLVM_PROFILE_FILE");
  if (!pattern || !*pattern) return;
  const char* slash = std::strrchr(pattern, '/');
  const auto directory = slash ? static_cast<std::size_t>(slash - pattern + 1) : 0;
  const char* module = LITESIM_COVERAGE_MODULE;
  const auto module_size = std::strlen(module);
  const auto original_size = std::strlen(pattern);
  auto* filename = static_cast<char*>(std::malloc(original_size + module_size + 1));
  if (!filename) return;  // The collector rejects any unqualified fallback.
  std::memcpy(filename, pattern, directory);
  std::memcpy(filename + directory, module, module_size);
  std::memcpy(filename + directory + module_size, pattern + directory,
              original_size - directory + 1);
  // This documented compiler-rt API copies the pattern. It has higher
  // precedence than runtime/environment initialization regardless of order.
  __llvm_profile_set_filename(filename);
  std::free(filename);
}
}  // namespace
