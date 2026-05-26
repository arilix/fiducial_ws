#!/usr/bin/env python3
"""Patch aruco.hpp and aruco.cpp with DictionaryManager + new topics + auto-detect mode."""
import os, re
BASE = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector"
HPP_PATH = os.path.join(BASE, "include/fiducial_detector/aruco.hpp")
with open(HPP_PATH) as f:
    hpp = f.read()
OLD_INC = '#include "fiducial_detector/visualization.hpp"'
NEW_INC = '#include "fiducial_detector/dictionary_manager.hpp"\n' + OLD_INC
hpp = hpp.replace(OLD_INC, NEW_INC, 1)
OLD_PUB = '  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_rejected_;'
NEW_PUB = OLD_PUB + """
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_cur_dict_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_dict_score_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            pub_det_stats_;"""
hpp = hpp.replace(OLD_PUB, NEW_PUB, 1)
OLD_PARAMS = '  int         max_missed_frames_{5};'
NEW_PARAMS = OLD_PARAMS + """
  DetectionMode detection_mode_{DetectionMode::SINGLE};
  std::string   detection_mode_str_{"SINGLE"};
  bool          benchmark_mode_{false};"""
hpp = hpp.replace(OLD_PARAMS, NEW_PARAMS, 1)
OLD_MEMBER = '  std::unique_ptr<PoseEstimator>             pose_estimator_;'
NEW_MEMBER = OLD_MEMBER + """
  std::unique_ptr<DictionaryManager>        dict_manager_;"""
hpp = hpp.replace(OLD_MEMBER, NEW_MEMBER, 1)
with open(HPP_PATH, "w") as f:
    f.write(hpp)
print(f"Patched aruco.hpp ({len(hpp.splitlines())} lines)")
CPP_PATH = os.path.join(BASE, "src/aruco.cpp")
with open(CPP_PATH) as f:
    cpp = f.read()
OLD_DICT = '
NEW_DICT = """
  dict_manager_ = std::make_unique<DictionaryManager>();
  dict_manager_->setActive(dictionary_type_);
  aruco_dict_ = dict_manager_->activeDict();"""
cpp = cpp.replace(OLD_DICT, NEW_DICT, 1)
OLD_DECL = '
NEW_DECL = """
  declare_parameter("detection_mode", "SINGLE");
  declare_parameter("benchmark_on_start", false);
  det_params_mgr_ = std::make_unique<DetectorParametersManager>();"""
cpp = cpp.replace(OLD_DECL, NEW_DECL, 1)
OLD_LOAD = '  max_missed_frames_   = get_parameter("max_missed_frames").as_int();'
NEW_LOAD = OLD_LOAD + """
  detection_mode_str_  = get_parameter("detection_mode").as_string();
  benchmark_mode_      = get_parameter("benchmark_on_start").as_bool();
  if      (detection_mode_str_ == "AUTO")      detection_mode_ = DetectionMode::AUTO;
  else if (detection_mode_str_ == "MULTI")     detection_mode_ = DetectionMode::MULTI;
  else if (detection_mode_str_ == "BENCHMARK") detection_mode_ = DetectionMode::BENCHMARK;
  else                                         detection_mode_ = DetectionMode::SINGLE;"""
cpp = cpp.replace(OLD_LOAD, NEW_LOAD, 1)
OLD_PUBS = '  RCLCPP_INFO(get_logger(), "Publishers created on /fiducial/{...");'
NEW_PUBS = """  pub_cur_dict_  = create_publisher<std_msgs::msg::String>("/fiducial/current_dictionary", 10);
  pub_dict_score_= create_publisher<std_msgs::msg::String>("/fiducial/dictionary_score", 10);
  pub_det_stats_ = create_publisher<std_msgs::msg::String>("/fiducial/detection_stats", 10);
  RCLCPP_INFO(get_logger(), "Publishers created on /fiducial/{...");"""
cpp = cpp.replace(OLD_PUBS, NEW_PUBS, 1)
OLD_DETECT = """
  detectAruco(gray, result);"""
NEW_DETECT = """
  if (detection_mode_ == DetectionMode::AUTO) {
    std::string best = dict_manager_->autoDetect(gray, det_params_mgr_->params());
    aruco_dict_ = dict_manager_->activeDict();
    dictionary_type_ = best;
    RCLCPP_DEBUG(get_logger(), "AUTO: selected %s", best.c_str());
  } else if (detection_mode_ == DetectionMode::MULTI) {
    auto all = dict_manager_->detectAll(gray, det_params_mgr_->params());
    for (const auto& dr : all) {
      for (std::size_t i = 0; i < dr.ids.size(); ++i) {
        DetectedMarker m;
        m.id      = dr.ids[i];
        m.type    = MarkerType::ARUCO;
        m.corners = dr.corners[i];
        m.center  = computeCenter(dr.corners[i]);
        result.markers.push_back(std::move(m));
      }
      result.rejected.insert(result.rejected.end(),
        dr.rejected.begin(), dr.rejected.end());
    }
    if (enable_charuco_) detectCharuco(gray, frame, result);
    return result;
  } else if (detection_mode_ == DetectionMode::BENCHMARK) {
    dict_manager_->benchmark(gray, det_params_mgr_->params(), 10);
  }
  detectAruco(gray, result);"""
cpp = cpp.replace(OLD_DETECT, NEW_DETECT, 1)
OLD_DETECT_ARUCO = '  cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, dp, rejected);'
NEW_DETECT_ARUCO = '
cpp = cpp.replace(OLD_DETECT_ARUCO, NEW_DETECT_ARUCO, 1)
OLD_PUB_REJ = '
NEW_PUB_EXTRA = """
  {
    auto msg = std_msgs::msg::String();
    msg.data = dict_manager_->activeName();
    pub_cur_dict_->publish(msg);
  }
  {
    auto s = dict_manager_->lastScore(dict_manager_->activeName());
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "{\"dict\":\"%s\",\"valid\":%d,\"rejected\":%d,\"latency_ms\":%.2f,\"score\":%.1f}",
      s.dict_name.c_str(), s.valid_markers, s.rejected_count, s.latency_ms, s.score);
    auto msg = std_msgs::msg::String();
    msg.data = buf;
    pub_dict_score_->publish(msg);
  }
  {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
      "{\"mode\":\"%s\",\"fps\":%.1f,\"markers\":%zu,\"frame\":%llu}",
      detection_mode_str_.c_str(), fps_monitor_.getFps(),
      result.markers.size(), (unsigned long long)frame_count_);
    auto msg = std_msgs::msg::String();
    msg.data = buf;
    pub_det_stats_->publish(msg);
  }
  """ + OLD_PUB_REJ
cpp = cpp.replace(OLD_PUB_REJ, NEW_PUB_EXTRA, 1)
with open(CPP_PATH, "w") as f:
    f.write(cpp)
print(f"Patched aruco.cpp ({len(cpp.splitlines())} lines)")
print("PATCH OK")
