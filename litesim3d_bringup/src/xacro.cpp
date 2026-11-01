#include "model.hpp"
#include <openssl/evp.h>
#include <iomanip>
#include <iostream>
#include <memory>

namespace litesim3d::bringup {
namespace {
std::string digest(const std::string &content) {
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  require(bool(context), "cannot allocate SHA256 context");
  unsigned char result[EVP_MAX_MD_SIZE]; unsigned length = 0;
  require(EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) == 1 &&
    EVP_DigestUpdate(context.get(), content.data(), content.size()) == 1 &&
    EVP_DigestFinal_ex(context.get(), result, &length) == 1, "SHA256 failed");
  std::ostringstream out; out << std::hex << std::setfill('0');
  for (unsigned i = 0; i < length; ++i) out << std::setw(2) << int(result[i]);
  return out.str();
}
void meshes(Element *element, const fs::path &base, Json &resources) {
  const std::string kind = element->Name();
  if (kind == "mesh" || kind == "texture") {
    const char *filename = element->Attribute("filename"); require(filename, "resource filename is required");
    const std::string name = filename;
    const auto path = resolve(name, base); resources[path.string()] = digest(read(path));
    element->SetAttribute("filename", path.c_str());
  }
  for (auto *child = element->FirstChildElement(); child; child = child->NextSiblingElement()) meshes(child, base, resources);
}
}
Json prepare_xacro(const fs::path &input, const fs::path &destination, const std::vector<std::string> &mappings,
                   const fs::path &resource_base, const CommandRunner &runner) {
  const auto source = canonical_path(input), output = canonical_path(destination);
  require(source != output, "output must differ from the source model");
  const fs::path metadata = output.string() + ".json";
  for (const auto &path : {output, metadata})
    require(!fs::exists(path) || fs::is_regular_file(path), "output artifact must be a regular file: " + path.string());
  const auto base = resource_base.empty() ? source.parent_path() : canonical_path(resource_base);
  std::vector<std::string> command{"xacro", source.string()}; command.insert(command.end(), mappings.begin(), mappings.end());
  const auto expanded = runner(command, 30, "", {});
  command.insert(command.begin() + 1, "--deps"); const auto dependencies = runner(command, 30, "", {});
  tinyxml2::XMLDocument document; parse_xml(document, expanded);
  auto resources = Json::object(); meshes(document.RootElement(), base, resources);
  const auto text = xml_text(*document.RootElement()) + "\n";
  auto includes = Json::object();
  // Upstream --deps is whitespace-separated without quoting. Match existing
  // paths before splitting so an include under a directory with spaces works.
  std::size_t offset = 0;
  while ((offset = dependencies.find_first_not_of(" \t\r\n", offset)) != std::string::npos) {
    std::vector<std::pair<std::string, std::size_t>> matches;
    for (std::size_t end = offset + 1; end <= dependencies.size(); ++end) {
      if (end != dependencies.size() && !std::isspace(static_cast<unsigned char>(dependencies[end]))) continue;
      const auto name = dependencies.substr(offset, end - offset);
      std::error_code error;
      if (fs::is_regular_file(name, error)) matches.emplace_back(name, end);
    }
    require(matches.size() == 1, "cannot unambiguously resolve xacro --deps paths");
    includes[matches[0].first] = digest(read(matches[0].first)); offset = matches[0].second;
  }
  Json provenance = {{"source", source.string()}, {"source_sha256", digest(read(source))}, {"resource_base", base.string()},
    {"mappings", mappings}, {"includes", includes}, {"resources", resources}, {"expanded_sha256", digest(text)}};
  // Complete all reads and hashing before replacing either output artifact.
  fs::create_directories(output.parent_path());
  TemporaryDirectory staged(".litesim-xacro-", output.parent_path());
  write(staged.path() / "model", text); write(staged.path() / "provenance", provenance.dump(2) + "\n");
  // Keep the previous model available until both replacement operations succeed.
  const bool previous = fs::exists(output);
  if (previous) fs::copy_file(output, staged.path() / "previous");
  fs::rename(staged.path() / "model", output);
  try { fs::rename(staged.path() / "provenance", metadata); }
  catch (...) {
    if (previous) fs::rename(staged.path() / "previous", output);
    else fs::remove(output);
    throw;
  }
  return provenance;
}
}  // namespace litesim3d::bringup
