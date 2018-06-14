#include "common/find_resource.h"

#include <cstdlib>
#include <spruce.hh>

#include "drake/common/drake_throw.h"
#include "drake/common/drake_optional.h"


using std::string;

namespace dairlib {

using Result = drake::FindResourceResult;
using drake::optional;
using drake::nullopt;

// Returns true iff the path is relative (not absolute).
bool IsRelativePath(const string& path) {
  return !path.empty() && (path[0] != '/');
}

// Returns candidate_dir iff it exists and contains our sentinel file.
optional<string> CheckCandidateDir(const spruce::path& candidate_dir) {
  // If we found the sentinel, we win.
  spruce::path candidate_file = candidate_dir;
  candidate_file.append(".dairlib-find_resource-sentinel");
  if (candidate_file.isFile()) {
    return candidate_dir.getStr();
  }
  return nullopt;
}


// Returns the directory that contains our sentinel file, searching from the
// current directory and working up through all transitive parent directories
// up to "/".
optional<string> FindSentinelDir() {
  spruce::path candidate_dir = spruce::dir::getcwd();
  std::cout << candidate_dir.getStr() << std::endl;
  int num_attempts = 0;
  while (true) {
    DRAKE_THROW_UNLESS(num_attempts < 1000);  // Insanity fail-fast.
    ++num_attempts;

    // If we fall off the end of the world somehow, stop.
    if (!candidate_dir.isDir()) {
      return nullopt;
    }

    // If we found the sentinel, we win.
    optional<string> result = CheckCandidateDir(candidate_dir);
    if (result) {
      return result;
    }

    // Move up one directory; with spruce, "root" means "parent".
    candidate_dir = candidate_dir.root();
  }
}

// Returns the absolute_path iff the `$dirpath/$relpath` exists, else nullopt.
// As a convenience to callers, if `dirpath` is nullopt, the result is nullopt.
// (To inquire about an empty `dirpath`, pass the empty string, not nullopt.)
optional<string> FileExists(
    const optional<string>& dirpath, const string& relpath) {
  DRAKE_ASSERT(IsRelativePath(relpath));
  if (!dirpath) { return nullopt; }
  const spruce::path dir_query(*dirpath);
  if (!dir_query.isDir()) { return nullopt; }
  const spruce::path file_query(dir_query.getStr() + '/' + relpath);
  if (!file_query.exists()) { return nullopt; }
  return file_query.getStr();
}

Result FindResource(string resource_path) {
  if (!IsRelativePath(resource_path)) {
    return Result::make_error(
        std::move(resource_path),
        "resource_path is not a relative path");
  }
  std::cout << *FindSentinelDir() <<  " ** " << resource_path << std::endl;
  if (auto absolute_path = FileExists(FindSentinelDir(), resource_path)) {
    return Result::make_success(
        std::move(resource_path), std::move(*absolute_path));
  }

  // Nothing found.
  string error_message = "could not find resource: " + resource_path;
  return Result::make_error(std::move(resource_path), error_message);
}

std::string FindResourceOrThrow(std::string resource_path) {
  return FindResource(std::move(resource_path)).get_absolute_path_or_throw();
}



}  // namespace drake
