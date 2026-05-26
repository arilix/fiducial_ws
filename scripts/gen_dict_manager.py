#!/usr/bin/env python3
"""Generate dictionary_manager.hpp and dictionary_manager.cpp — full 22-dict support."""
import os
INC = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector/include/fiducial_detector"
SRC = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector/src"
HPP = r"""#pragma once
namespace fiducial_detector {
enum class DetectionMode : uint8_t {
  SINGLE    = 0,
  AUTO      = 1,
  MULTI     = 2,
  BENCHMARK = 3,
};
struct DictionaryInfo {
  std::string                    name;
  int                            dict_id;
  cv::Ptr<cv::aruco::Dictionary> dict;
  std::string                    family;
  int                            marker_bits;
  int                            total_markers;
};
struct DictionaryScore {
  std::string dict_name;
  int         valid_markers{0};
  int         rejected_count{0};
  double      reprojection_error{0.0};
  double      pose_stability{0.0};
  double      latency_ms{0.0};
  double      score{0.0};
  static double compute(int valid, int rejected,
                        double reproj_err, double stability) {
    return static_cast<double>(valid) * 10.0
           - static_cast<double>(rejected) * 0.5
           - reproj_err
           - stability;
  }
};
struct DictDetectionResult {
  std::string                              dict_name;
  std::vector<int>                         ids;
  std::vector<std::vector<cv::Point2f>>   corners;
  std::vector<std::vector<cv::Point2f>>   rejected;
  DictionaryScore                          score;
};
class DictionaryManager {
public:
  DictionaryManager();
  static cv::Ptr<cv::aruco::Dictionary> getDictionaryByName(const std::string& name);
  static int getDictIdByName(const std::string& name);
  static const std::map<std::string, int>& getDictMap();
  static std::vector<std::string> getAllNames();
  bool setActive(const std::string& name);
  std::string activeName() const;
  cv::Ptr<cv::aruco::Dictionary> activeDict() const;
  bool hotReload(const std::string& name);
  std::string autoDetect(
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params);
  std::vector<DictDetectionResult> detectAll(
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params);
  void benchmark(
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params,
    int n_frames = 30);
  const DictionaryInfo& info(const std::string& name) const;
  const std::map<std::string, DictionaryScore>& scores() const { return scores_; }
  DictionaryScore lastScore(const std::string& name) const;
  static const std::vector<std::string> AUTO_SUBSET;
private:
  void initAll();
  DictDetectionResult detectWith(
    const std::string& dict_name,
    const cv::Mat& gray,
    const cv::Ptr<cv::aruco::DetectorParameters>& params) const;
  std::map<std::string, DictionaryInfo> dicts_;
  std::map<std::string, DictionaryScore> scores_;
  std::string active_name_{"DICT_4X4_50"};
  mutable std::mutex mutex_;
};
}
"""
CPP = r"""#include "fiducial_detector/dictionary_manager.hpp"
namespace fiducial_detector {
static const std::map<std::string, int> GLOBAL_DICT_MAP = {
  {"DICT_4X4_50",          cv::aruco::DICT_4X4_50},
  {"DICT_4X4_100",         cv::aruco::DICT_4X4_100},
  {"DICT_4X4_250",         cv::aruco::DICT_4X4_250},
  {"DICT_4X4_1000",        cv::aruco::DICT_4X4_1000},
  {"DICT_5X5_50",          cv::aruco::DICT_5X5_50},
  {"DICT_5X5_100",         cv::aruco::DICT_5X5_100},
  {"DICT_5X5_250",         cv::aruco::DICT_5X5_250},
  {"DICT_5X5_1000",        cv::aruco::DICT_5X5_1000},
  {"DICT_6X6_50",          cv::aruco::DICT_6X6_50},
  {"DICT_6X6_100",         cv::aruco::DICT_6X6_100},
  {"DICT_6X6_250",         cv::aruco::DICT_6X6_250},
  {"DICT_6X6_1000",        cv::aruco::DICT_6X6_1000},
  {"DICT_7X7_50",          cv::aruco::DICT_7X7_50},
  {"DICT_7X7_100",         cv::aruco::DICT_7X7_100},
  {"DICT_7X7_250",         cv::aruco::DICT_7X7_250},
  {"DICT_7X7_1000",        cv::aruco::DICT_7X7_1000},
  {"DICT_ARUCO_ORIGINAL",  cv::aruco::DICT_ARUCO_ORIGINAL},
  {"DICT_ARUCO_MIP_36h12", cv::aruco::DICT_ARUCO_MIP_36h12},
  {"DICT_APRILTAG_16h5",   cv::aruco::DICT_APRILTAG_16h5},
  {"DICT_APRILTAG_25h9",   cv::aruco::DICT_APRILTAG_25h9},
  {"DICT_APRILTAG_36h10",  cv::aruco::DICT_APRILTAG_36h10},
  {"DICT_APRILTAG_36h11",  cv::aruco::DICT_APRILTAG_36h11},
};
const std::vector<std::string> DictionaryManager::AUTO_SUBSET = {
  "DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50", "DICT_7X7_50",
  "DICT_ARUCO_ORIGINAL", "DICT_ARUCO_MIP_36h12",
  "DICT_APRILTAG_36h11",
};
const std::map<std::string, int>& DictionaryManager::getDictMap() {
  return GLOBAL_DICT_MAP;
}
std::vector<std::string> DictionaryManager::getAllNames() {
  std::vector<std::string> names;
  names.reserve(GLOBAL_DICT_MAP.size());
  for (const auto& kv : GLOBAL_DICT_MAP) names.push_back(kv.first);
  return names;
}
int DictionaryManager::getDictIdByName(const std::string& name) {
  auto it = GLOBAL_DICT_MAP.find(name);
  if (it == GLOBAL_DICT_MAP.end())
    throw std::invalid_argument("Unknown dictionary: " + name);
  return it->second;
}
cv::Ptr<cv::aruco::Dictionary> DictionaryManager::getDictionaryByName(
  const std::string& name)
{
  return cv::aruco::getPredefinedDictionary(getDictIdByName(name));
}
DictionaryManager::DictionaryManager() {
  initAll();
}
void DictionaryManager::initAll()
{
  struct Meta { std::string family; int bits; int total; };
  static const std::map<std::string, Meta> META = {
    {"DICT_4X4_50",          {"4x4",    4,   50}},
    {"DICT_4X4_100",         {"4x4",    4,  100}},
    {"DICT_4X4_250",         {"4x4",    4,  250}},
    {"DICT_4X4_1000",        {"4x4",    4, 1000}},
    {"DICT_5X5_50",          {"5x5",    5,   50}},
    {"DICT_5X5_100",         {"5x5",    5,  100}},
    {"DICT_5X5_250",         {"5x5",    5,  250}},
    {"DICT_5X5_1000",        {"5x5",    5, 1000}},
    {"DICT_6X6_50",          {"6x6",    6,   50}},
    {"DICT_6X6_100",         {"6x6",    6,  100}},
    {"DICT_6X6_250",         {"6x6",    6,  250}},
    {"DICT_6X6_1000",        {"6x6",    6, 1000}},
    {"DICT_7X7_50",          {"7x7",    7,   50}},
    {"DICT_7X7_100",         {"7x7",    7,  100}},
    {"DICT_7X7_250",         {"7x7",    7,  250}},
    {"DICT_7X7_1000",        {"7x7",    7, 1000}},
    {"DICT_ARUCO_ORIGINAL",  {"aruco",  5, 1024}},
    {"DICT_ARUCO_MIP_36h12", {"mip",    6,  250}},
    {"DICT_APRILTAG_16h5",   {"apriltag",4,  30}},
    {"DICT_APRILTAG_25h9",   {"apriltag",5,  35}},
    {"DICT_APRILTAG_36h10",  {"apriltag",6,  2320}},
    {"DICT_APRILTAG_36h11",  {"apriltag",6,  587}},
  };
  for (const auto& kv : GLOBAL_DICT_MAP) {
    DictionaryInfo info;
    info.name    = kv.first;
    info.dict_id = kv.second;
    info.dict    = cv::aruco::getPredefinedDictionary(kv.second);
    const auto& m = META.at(kv.first);
    info.family       = m.family;
    info.marker_bits  = m.bits;
    info.total_markers = m.total;
    dicts_[kv.first] = std::move(info);
  }
}
bool DictionaryManager::setActive(const std::string& name) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (dicts_.find(name) == dicts_.end()) return false;
  active_name_ = name;
  return true;
}
std::string DictionaryManager::activeName() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return active_name_;
}
cv::Ptr<cv::aruco::Dictionary> DictionaryManager::activeDict() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return dicts_.at(active_name_).dict;
}
bool DictionaryManager::hotReload(const std::string& name) {
  return setActive(name);
}
const DictionaryInfo& DictionaryManager::info(const std::string& name) const {
  return dicts_.at(name);
}
DictionaryScore DictionaryManager::lastScore(const std::string& name) const {
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = scores_.find(name);
  if (it == scores_.end()) return DictionaryScore{};
  return it->second;
}
DictDetectionResult DictionaryManager::detectWith(
  const std::string& dict_name,
  const cv::Mat& gray,
  const cv::Ptr<cv::aruco::DetectorParameters>& params) const
{
  DictDetectionResult res;
  res.dict_name = dict_name;
  const auto it = dicts_.find(dict_name);
  if (it == dicts_.end()) return res;
  auto t0 = std::chrono::steady_clock::now();
  cv::aruco::detectMarkers(gray, it->second.dict, res.corners, res.ids, params, res.rejected);
  auto t1 = std::chrono::steady_clock::now();
  double lat = std::chrono::duration<double, std::milli>(t1 - t0).count();
  int    valid    = (int)res.ids.size();
  int    rejected = (int)res.rejected.size();
  res.score.dict_name      = dict_name;
  res.score.valid_markers  = valid;
  res.score.rejected_count = rejected;
  res.score.latency_ms     = lat;
  res.score.score          = DictionaryScore::compute(valid, rejected, 0.0, 0.0);
  return res;
}
std::string DictionaryManager::autoDetect(
  const cv::Mat& gray,
  const cv::Ptr<cv::aruco::DetectorParameters>& params)
{
  std::string best_name = active_name_;
  double      best_score = -1e9;
  for (const auto& name : AUTO_SUBSET) {
    auto res = detectWith(name, gray, params);
    if (res.score.score > best_score) {
      best_score = res.score.score;
      best_name  = name;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    scores_[name] = res.score;
  }
  if (best_score > 0.0 && best_name != active_name_) {
    setActive(best_name);
  }
  return best_name;
}
std::vector<DictDetectionResult> DictionaryManager::detectAll(
  const cv::Mat& gray,
  const cv::Ptr<cv::aruco::DetectorParameters>& params)
{
  std::vector<std::future<DictDetectionResult>> futures;
  for (const auto& kv : dicts_) {
    futures.push_back(std::async(std::launch::async,
      [this, &kv, &gray, &params]() {
        return detectWith(kv.first, gray, params);
      }));
  }
  std::vector<DictDetectionResult> results;
  results.reserve(futures.size());
  for (auto& f : futures) {
    auto r = f.get();
    {
      std::lock_guard<std::mutex> lk(mutex_);
      scores_[r.dict_name] = r.score;
    }
    results.push_back(std::move(r));
  }
  return results;
}
void DictionaryManager::benchmark(
  const cv::Mat& gray,
  const cv::Ptr<cv::aruco::DetectorParameters>& params,
  int n_frames)
{
  std::printf("\n%s\n", std::string(70, '=').c_str());
  std::printf("  DICTIONARY BENCHMARK  (n=%d)\n", n_frames);
  std::printf("%-25s %6s %8s %8s %10s\n",
    "Dictionary", "Valid", "Rejected", "Lat(ms)", "Score");
  std::printf("%s\n", std::string(70, '-').c_str());
  for (const auto& kv : dicts_) {
    double total_lat = 0.0;
    int    total_valid = 0, total_rej = 0;
    for (int i = 0; i < n_frames; ++i) {
      auto res = detectWith(kv.first, gray, params);
      total_lat   += res.score.latency_ms;
      total_valid += res.score.valid_markers;
      total_rej   += res.score.rejected_count;
    }
    double avg_lat   = total_lat   / n_frames;
    double avg_valid = (double)total_valid / n_frames;
    double avg_rej   = (double)total_rej   / n_frames;
    double score     = DictionaryScore::compute(
      (int)avg_valid, (int)avg_rej, 0.0, 0.0);
    DictionaryScore s;
    s.dict_name      = kv.first;
    s.valid_markers  = (int)avg_valid;
    s.rejected_count = (int)avg_rej;
    s.latency_ms     = avg_lat;
    s.score          = score;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      scores_[kv.first] = s;
    }
    std::printf("%-25s %6.1f %8.1f %8.2f %10.1f\n",
      kv.first.c_str(), avg_valid, avg_rej, avg_lat, score);
  }
  std::printf("%s\n\n", std::string(70, '=').c_str());
}
}
"""
os.makedirs(INC, exist_ok=True)
os.makedirs(SRC, exist_ok=True)
with open(os.path.join(INC, "dictionary_manager.hpp"), "w") as f:
    f.write(HPP.lstrip("\n"))
with open(os.path.join(SRC, "dictionary_manager.cpp"), "w") as f:
    f.write(CPP.lstrip("\n"))
print(f"HPP: {HPP.count(chr(10))} lines")
print(f"CPP: {CPP.count(chr(10))} lines")
print("DICT MANAGER OK")
