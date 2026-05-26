#!/usr/bin/env python3
"""Patch visualization.cpp/.hpp to add dictionary info panel. Also patch CMakeLists."""
import os
BASE = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector"
VIZ_HPP = os.path.join(BASE, "include/fiducial_detector/visualization.hpp")
with open(VIZ_HPP) as f:
    hpp = f.read()
OLD_HUD = '
NEW_HUD = OLD_HUD.replace(
    '
    '
    '  void drawDictionaryPanel(\n'
    '    cv::Mat& frame,\n'
    '    const std::string& active_dict,\n'
    '    const std::string& mode,\n'
    '    const std::map<std::string,double>& scores) const;\n\n'
    '
)
hpp = hpp.replace(OLD_HUD, NEW_HUD, 1)
if '#include <map>' not in hpp:
    hpp = hpp.replace('#include <string>', '#include <string>\n#include <map>', 1)
with open(VIZ_HPP, "w") as f:
    f.write(hpp)
print(f"Patched visualization.hpp ({len(hpp.splitlines())} lines)")
VIZ_CPP = os.path.join(BASE, "src/visualization.cpp")
with open(VIZ_CPP) as f:
    cpp = f.read()
DICT_PANEL_IMPL = r"""
void Visualizer::drawDictionaryPanel(
  cv::Mat& frame,
  const std::string& active_dict,
  const std::string& mode,
  const std::map<std::string, double>& scores) const
{
  int W = frame.cols;
  int panel_w = 280, panel_h = 80 + (int)scores.size() * 14;
  int px = W - panel_w - 8, py = 8;
  panel_h = std::min(panel_h, frame.rows - py - 8);
  alphaRect(frame, cv::Rect(px, py, panel_w, panel_h), CLR_BLACK, 0.65);
  cv::rectangle(frame, cv::Rect(px, py, panel_w, panel_h), CLR_GRAY, 1);
  int y = py + 16;
  auto line = [&](const std::string& s, cv::Scalar c, double fs = 0.42){
    cv::putText(frame, s, {px+6, y}, cv::FONT_HERSHEY_SIMPLEX,
                fs, c, 1, cv::LINE_AA);
    y += 14;
  };
  line("[ DICT MANAGER ]", CLR_WHITE, 0.48);
  line("Mode: " + mode, CLR_LOCKED, 0.44);
  line("Active: " + active_dict, CLR_ALIGNED, 0.44);
  y += 4;
  line("--- Scores ---", CLR_GRAY, 0.40);
  std::vector<std::pair<double,std::string>> sorted;
  for (const auto& kv : scores) sorted.push_back({kv.second, kv.first});
  std::sort(sorted.rbegin(), sorted.rend());
  int shown = 0;
  for (const auto& sv : sorted) {
    if (shown >= 7) break;
    if (y + 14 > py + panel_h - 4) break;
    char buf[80];
    std::snprintf(buf, sizeof(buf), "  %-22s %5.1f", sv.second.c_str(), sv.first);
    cv::Scalar c = (sv.second == active_dict) ? CLR_ALIGNED : CLR_GRAY;
    line(buf, c, 0.38);
    ++shown;
  }
}
"""
cpp = cpp.rstrip()
if cpp.endswith("}
    cpp = cpp[:-len("}
else:
    cpp += DICT_PANEL_IMPL
with open(VIZ_CPP, "w") as f:
    f.write(cpp)
print(f"Patched visualization.cpp ({len(cpp.splitlines())} lines)")
ARUCO_CPP = os.path.join(BASE, "src/aruco.cpp")
with open(ARUCO_CPP) as f:
    cpp2 = f.read()
OLD_VIZ = '  visualizer_->drawHUD(annotated,'
NEW_VIZ = """
  std::map<std::string,double> score_map;
  for (const auto& nm : DictionaryManager::getAllNames()) {
    score_map[nm] = dict_manager_->lastScore(nm).score;
  }
  visualizer_->drawDictionaryPanel(annotated,
    dict_manager_->activeName(), detection_mode_str_, score_map);
  """ + OLD_VIZ
cpp2 = cpp2.replace(OLD_VIZ, NEW_VIZ, 1)
with open(ARUCO_CPP, "w") as f:
    f.write(cpp2)
print(f"Patched aruco.cpp ({len(cpp2.splitlines())} lines)")
CMAKE = os.path.join(BASE, "CMakeLists.txt")
with open(CMAKE) as f:
    cmake = f.read()
cmake = cmake.replace(
    '  src/detector_parameters.cpp\n)',
    '  src/detector_parameters.cpp\n  src/dictionary_manager.cpp\n)'
)
with open(CMAKE, "w") as f:
    f.write(cmake)
print(f"Patched CMakeLists.txt ({len(cmake.splitlines())} lines)")
YAML = os.path.join(BASE, "config/detector.yaml")
with open(YAML) as f:
    yaml = f.read()
EXTRA_YAML = """
    detection_mode: "SINGLE"
    benchmark_on_start: false
"""
yaml = yaml.rstrip() + "\n" + EXTRA_YAML
with open(YAML, "w") as f:
    f.write(yaml)
print("Patched detector.yaml")
BENCH_LAUNCH = os.path.join(BASE, "launch/benchmark.launch.py")
BENCH_CONTENT = '''#!/usr/bin/env python3
"""benchmark.launch.py — Run fiducial_detector in BENCHMARK mode."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    pkg = get_package_share_directory("fiducial_detector")
    params = os.path.join(pkg, "config", "detector.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw"),
        LogInfo(msg="Starting fiducial_detector in BENCHMARK mode..."),
        Node(
            package="fiducial_detector", executable="aruco_node",
            name="aruco_node",
            parameters=[params, {
                "camera_topic":     LaunchConfiguration("camera_topic"),
                "detection_mode":   "BENCHMARK",
                "benchmark_on_start": True,
                "show_window":      True,
            }], output="screen"),
    ])
'''
with open(BENCH_LAUNCH, "w") as f:
    f.write(BENCH_CONTENT)
print("Created benchmark.launch.py")
print("\nALL PATCHES DONE")
