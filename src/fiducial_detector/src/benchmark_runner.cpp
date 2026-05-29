#include "fiducial_detector/benchmark_runner.hpp"
#include "fiducial_detector/hybrid_detector.hpp"
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <cstring>
#if defined(__linux__)
  #include <sys/resource.h>
  #include <unistd.h>
  #include <fstream>
#endif
namespace fiducial_detector {
std::string BenchmarkResult::toJson() const {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
    "{\"dict\":\"%s\","
    "\"fps_avg\":%.2f,"
    "\"latency_avg_ms\":%.3f,"
    "\"latency_min_ms\":%.3f,"
    "\"latency_max_ms\":%.3f,"
    "\"latency_std_ms\":%.3f,"
    "\"cpu_percent\":%.1f,"
    "\"memory_mb\":%.1f,"
    "\"markers_found\":%d,"
    "\"false_positives\":%d,"
    "\"false_pos_rate\":%.4f,"
    "\"pose_stability\":%.3f}",
    dict_name.c_str(),
    fps_avg, latency_avg_ms,
    latency_min_ms, latency_max_ms, latency_std_ms,
    cpu_percent, memory_mb,
    markers_found, false_positives, false_pos_rate,
    pose_stability);
  return buf;
}
std::string BenchmarkReport::toJson() const {
  std::ostringstream ss;
  ss << "{\"results\":[";
  for (std::size_t i = 0; i < results.size(); ++i) {
    if (i) ss << ",";
    ss << results[i].toJson();
  }
  ss << "],"
     << "\"best_fps\":\"" << best_fps_dict << "\","
     << "\"best_latency\":\"" << best_latency_dict << "\","
     << "\"best_accuracy\":\"" << best_accuracy_dict << "\","
     << "\"total_time_s\":" << total_time_s
     << "}";
  return ss.str();
}
std::string BenchmarkReport::toTable() const {
  std::ostringstream ss;
  ss << std::string(90, '=') << "\n";
  ss << "  DICTIONARY BENCHMARK REPORT\n";
  ss << std::string(90, '-') << "\n";
  ss << std::left
     << std::setw(28) << "Dictionary"
     << std::setw(8)  << "FPS"
     << std::setw(12) << "Lat(ms)"
     << std::setw(12) << "LatStd"
     << std::setw(8)  << "Markers"
     << std::setw(10) << "CPU%"
     << std::setw(10) << "Mem(MB)"
     << "\n";
  ss << std::string(90, '-') << "\n";
  for (const auto& r : results) {
    ss << std::left
       << std::setw(28) << r.dict_name
       << std::setw(8)  << std::fixed << std::setprecision(1) << r.fps_avg
       << std::setw(12) << std::setprecision(2) << r.latency_avg_ms
       << std::setw(12) << std::setprecision(2) << r.latency_std_ms
       << std::setw(8)  << r.markers_found
       << std::setw(10) << std::setprecision(1) << r.cpu_percent
       << std::setw(10) << std::setprecision(1) << r.memory_mb
       << "\n";
  }
  ss << std::string(90, '=') << "\n";
  ss << "Best FPS:      " << best_fps_dict << "\n";
  ss << "Best Latency:  " << best_latency_dict << "\n";
  ss << "Best Accuracy: " << best_accuracy_dict << "\n";
  ss << "Total time:    " << total_time_s << "s\n";
  return ss.str();
}
double BenchmarkRunner::getCpuUsagePercent() {
#if defined(__linux__)
  std::ifstream stat("/proc/self/stat");
  if (!stat.is_open()) return 0.0;
  std::string line;
  std::getline(stat, line);
  std::istringstream ss(line);
  std::string token;
  for (int i = 0; i < 13; ++i) ss >> token;
  long utime = 0, stime = 0;
  ss >> utime >> stime;
  long total_ticks = utime + stime;
  long clk_tck = sysconf(_SC_CLK_TCK);
  if (clk_tck <= 0) clk_tck = 100;
  std::ifstream uptime_f("/proc/uptime");
  double uptime = 1.0;
  uptime_f >> uptime;
  double cpu_s = (double)total_ticks / (double)clk_tck;
  return (uptime > 0) ? (cpu_s / uptime * 100.0) : 0.0;
#else
  return 0.0;
#endif
}
double BenchmarkRunner::getMemoryUsageMB() {
#if defined(__linux__)
  std::ifstream status("/proc/self/status");
  if (!status.is_open()) return 0.0;
  std::string line;
  while (std::getline(status, line)) {
    if (line.rfind("VmRSS:", 0) == 0) {
      std::istringstream ss(line.substr(6));
      long kb = 0;
      ss >> kb;
      return (double)kb / 1024.0;
    }
  }
  return 0.0;
#else
  return 0.0;
#endif
}
std::vector<double> BenchmarkRunner::measureLatencies(
  const cv::Mat& gray,
  cv::Ptr<cv::aruco::Dictionary> dict,
  const cv::Ptr<cv::aruco::DetectorParameters>& params,
  int n_frames)
{
  std::vector<double> latencies;
  latencies.reserve(n_frames);
  for (int i = 0; i < n_frames; ++i) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    auto t0 = std::chrono::steady_clock::now();
    cv::aruco::detectMarkers(gray, dict, corners, ids, params, rejected);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    latencies.push_back(ms);
  }
  return latencies;
}
double BenchmarkRunner::computeStdDev(
  const std::vector<double>& values, double mean)
{
  if (values.size() < 2) return 0.0;
  double var = 0.0;
  for (double v : values) var += (v - mean) * (v - mean);
  return std::sqrt(var / (double)(values.size() - 1));
}
BenchmarkResult BenchmarkRunner::runSingle(
  const cv::Mat& gray,
  cv::Ptr<cv::aruco::Dictionary> dict,
  const cv::Ptr<cv::aruco::DetectorParameters>& params,
  const std::string& dict_name,
  int n_frames)
{
  BenchmarkResult r;
  r.dict_name = dict_name;
  auto latencies = measureLatencies(gray, dict, params, n_frames);
  if (latencies.empty()) return r;
  double sum = std::accumulate(latencies.begin(), latencies.end(), 0.0);
  r.latency_avg_ms = sum / (double)latencies.size();
  r.latency_min_ms = *std::min_element(latencies.begin(), latencies.end());
  r.latency_max_ms = *std::max_element(latencies.begin(), latencies.end());
  r.latency_std_ms = computeStdDev(latencies, r.latency_avg_ms);
  r.fps_avg = (r.latency_avg_ms > 0.0) ? 1000.0 / r.latency_avg_ms : 0.0;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(gray, dict, corners, ids, params);
  r.markers_found = (int)ids.size();
  r.cpu_percent = getCpuUsagePercent();
  r.memory_mb   = getMemoryUsageMB();
  return r;
}
BenchmarkResult BenchmarkRunner::runSingleHybrid(
  const cv::Mat& gray,
  HybridDetector& hybrid,
  const std::string& dict_name,
  int n_frames)
{
  BenchmarkResult r;
  r.dict_name = dict_name + " [Hybrid]";
  std::vector<double> latencies;
  latencies.reserve(n_frames);
  int total_markers = 0;
  for (int i = 0; i < n_frames; ++i) {
    auto t0 = std::chrono::steady_clock::now();
    auto hr = hybrid.detect(gray, dict_name, DetectorBackend::FUSION);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    latencies.push_back(ms);
    total_markers += static_cast<int>(hr.markers.size());
  }
  if (latencies.empty()) return r;
  double sum = std::accumulate(latencies.begin(), latencies.end(), 0.0);
  r.latency_avg_ms = sum / static_cast<double>(latencies.size());
  r.latency_min_ms = *std::min_element(latencies.begin(), latencies.end());
  r.latency_max_ms = *std::max_element(latencies.begin(), latencies.end());
  r.latency_std_ms = computeStdDev(latencies, r.latency_avg_ms);
  r.fps_avg        = (r.latency_avg_ms > 0.0) ? 1000.0 / r.latency_avg_ms : 0.0;
  r.markers_found  = total_markers / std::max(n_frames, 1);
  r.cpu_percent    = getCpuUsagePercent();
  r.memory_mb      = getMemoryUsageMB();
  return r;
}
BenchmarkReport BenchmarkRunner::runAll(
  const cv::Mat& gray,
  const std::map<std::string, cv::Ptr<cv::aruco::Dictionary>>& dicts,
  const cv::Ptr<cv::aruco::DetectorParameters>& params,
  int n_frames)
{
  BenchmarkReport report;
  auto t_start = std::chrono::steady_clock::now();
  for (const auto& kv : dicts) {
    auto r = runSingle(gray, kv.second, params, kv.first, n_frames);
    report.results.push_back(r);
  }
  auto t_end = std::chrono::steady_clock::now();
  report.total_time_s = std::chrono::duration<double>(t_end - t_start).count();
  if (!report.results.empty()) {
    auto best_fps = std::max_element(report.results.begin(), report.results.end(),
      [](const BenchmarkResult& a, const BenchmarkResult& b) {
        return a.fps_avg < b.fps_avg;
      });
    auto best_lat = std::min_element(report.results.begin(), report.results.end(),
      [](const BenchmarkResult& a, const BenchmarkResult& b) {
        return a.latency_avg_ms < b.latency_avg_ms;
      });
    auto best_acc = std::max_element(report.results.begin(), report.results.end(),
      [](const BenchmarkResult& a, const BenchmarkResult& b) {
        return a.markers_found < b.markers_found;
      });
    report.best_fps_dict      = best_fps->dict_name;
    report.best_latency_dict  = best_lat->dict_name;
    report.best_accuracy_dict = best_acc->dict_name;
  }
  last_report_ = report;
  return report;
}
double BenchmarkRunner::measureFPS(
  const cv::Mat& gray,
  cv::Ptr<cv::aruco::Dictionary> dict,
  const cv::Ptr<cv::aruco::DetectorParameters>& params,
  double duration_s)
{
  int frame_count = 0;
  auto t0 = std::chrono::steady_clock::now();
  auto deadline = t0 + std::chrono::duration<double>(duration_s);
  while (std::chrono::steady_clock::now() < deadline) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dict, corners, ids, params);
    ++frame_count;
  }
  auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - t0).count();
  return (elapsed > 0) ? (double)frame_count / elapsed : 0.0;
}
void BenchmarkRunner::printReport(const BenchmarkReport& report) {
  std::printf("%s\n", report.toTable().c_str());
}
bool BenchmarkRunner::saveReport(
  const BenchmarkReport& report, const std::string& path)
{
  std::ofstream f(path);
  if (!f.is_open()) return false;
  f << report.toJson();
  return true;
}
} 
