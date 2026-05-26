#!/usr/bin/env python3
"""Patch aruco.cpp: wrap AprilTag code blocks with #if HAVE_APRILTAG guards"""
TARGET = "/home/arilix/Documents/vtol/vtol aruco/fiducial_ws/src/fiducial_detector/src/aruco.cpp"
with open(TARGET, "r") as f:
    content = f.read()
content = content.replace(
    "
    "  if (enable_apriltag_) {\n"
    "    tag_family_   = tag36h11_create();\n"
    "    tag_detector_ = apriltag_detector_create();\n"
    "    apriltag_detector_add_family(tag_detector_, tag_family_);\n"
    "    tag_detector_->quad_decimate = 2.0f;\n"
    "    tag_detector_->nthreads      = 2;\n"
    "    tag_detector_->debug         = 0;\n"
    "  }",
    "
    "#if HAVE_APRILTAG\n"
    "  if (enable_apriltag_) {\n"
    "    tag_family_   = tag36h11_create();\n"
    "    tag_detector_ = apriltag_detector_create();\n"
    "    apriltag_detector_add_family(tag_detector_, tag_family_);\n"
    "    tag_detector_->quad_decimate = 2.0f;\n"
    "    tag_detector_->nthreads      = 2;\n"
    "    tag_detector_->debug         = 0;\n"
    "  }\n"
    "#endif
)
content = content.replace(
    "  if (tag_detector_) apriltag_detector_destroy(tag_detector_);\n"
    "  if (tag_family_)   tag36h11_destroy(tag_family_);\n",
    "#if HAVE_APRILTAG\n"
    "  if (tag_detector_) apriltag_detector_destroy(tag_detector_);\n"
    "  if (tag_family_)   tag36h11_destroy(tag_family_);\n"
    "#endif\n"
)
content = content.replace(
    "  if (enable_apriltag_) detectAprilTag(frame, detections);",
    "#if HAVE_APRILTAG\n"
    "  if (enable_apriltag_) detectAprilTag(frame, detections);\n"
    "#endif"
)
AT_IMPL_START = "
AT_IMPL_END   = "  apriltag_detections_destroy(dets);\n}"
idx_start = content.find(AT_IMPL_START)
idx_end   = content.find(AT_IMPL_END, idx_start)
if idx_start != -1 and idx_end != -1:
    idx_end += len(AT_IMPL_END)
    impl = content[idx_start:idx_end]
    content = content[:idx_start] + "#if HAVE_APRILTAG\n" + impl + "\n#endif
with open(TARGET, "w") as f:
    f.write(content)
print("PATCHED OK — line count:", content.count("\n"))
