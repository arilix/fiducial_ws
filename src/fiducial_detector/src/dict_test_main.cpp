#include "fiducial_detector/marker_generator.hpp"
#include "fiducial_detector/dictionary_manager.hpp"
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace fiducial_detector;

static void testDictionary(const std::string& dict_name,
                           const std::string& output_dir)
{
  std::string marker_dir = output_dir + "/" + dict_name;

  auto& dict_map = DictionaryManager::getDictMap();
  if (dict_map.find(dict_name) == dict_map.end()) {
    std::fprintf(stderr, "[dict_test] Unknown dictionary: %s\n", dict_name.c_str());
    return;
  }

  // Startup validation (prints to stdout)
  {
    DictionaryManager mgr;
    mgr.printStartupValidation(dict_name);
  }

  std::printf("[dict_test] Generating markers for %s -> %s\n",
    dict_name.c_str(), marker_dir.c_str());

  int written = MarkerGenerator::generateDictionarySet(dict_name, marker_dir);
  if (written < 0) {
    std::fprintf(stderr, "[dict_test] Generation failed for %s\n", dict_name.c_str());
    return;
  }
  std::printf("[dict_test] Generated %d marker(s)\n", written);

  std::vector<MarkerValidationResult> results;
  MarkerGenerator::validateGeneratedDictionary(dict_name, marker_dir, results);
  MarkerGenerator::printValidationReport(dict_name, results);
}

int main(int argc, char** argv)
{
  std::string output_root = "/tmp/aruco_dict_test";

  if (argc >= 3) output_root = argv[2];

  if (argc < 2 || std::strcmp(argv[1], "ALL") == 0) {
    // Test all registered dictionaries
    auto names = DictionaryManager::getAllNames();
    std::printf("[dict_test] Running test for ALL %zu dictionaries\n", names.size());
    for (const auto& name : names) {
      testDictionary(name, output_root);
    }
  } else {
    testDictionary(std::string(argv[1]), output_root);
  }

  return 0;
}
