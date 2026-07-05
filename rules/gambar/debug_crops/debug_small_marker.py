#!/usr/bin/env python3
"""
Debug script: Capture frame, find ALL square-like contours,
crop each one, upscale to 300px, try ArUco detection, and save results.

Usage:
  python3 debug_small_marker.py [camera_id]
  
Press 's' to save debug crops, 'q' to quit.
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import os
from datetime import datetime

# Setup
cam_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_7X7_50)

# Parameter sangat toleran untuk marker kecil
try:
    params = aruco.DetectorParameters_create()
except AttributeError:
    params = aruco.DetectorParameters()
params.adaptiveThreshWinSizeMin = 3
params.adaptiveThreshWinSizeMax = 53
params.adaptiveThreshWinSizeStep = 2
params.adaptiveThreshConstant = 5.0
params.minMarkerPerimeterRate = 0.003
params.maxMarkerPerimeterRate = 4.0
params.minCornerDistanceRate = 0.01
params.minMarkerDistanceRate = 0.003
params.minDistanceToBorder = 0
params.perspectiveRemovePixelPerCell = 10
params.perspectiveRemoveIgnoredMarginPerCell = 0.20
params.maxErroneousBitsInBorderRate = 0.60
params.errorCorrectionRate = 0.90
params.detectInvertedMarker = True
params.markerBorderBits = 1
params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

save_dir = os.path.dirname(os.path.abspath(__file__))

print(f"Camera {cam_id} opened. Press 's' to save debug crops, 'q' to quit.")
print(f"Save directory: {save_dir}")

frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera read failed")
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    display = frame.copy()
    
    # 1. Deteksi normal
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=params)
    
    if ids is not None:
        aruco.drawDetectedMarkers(display, corners, ids)
        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]
            cx, cy = int(c[:, 0].mean()), int(c[:, 1].mean())
            cv2.putText(display, f"ID={marker_id}", (cx-20, cy-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # 2. Tampilkan rejected candidates
    if rejected:
        for r in rejected:
            pts = r[0].astype(int)
            for j in range(4):
                cv2.line(display, tuple(pts[j]), tuple(pts[(j+1)%4]), (0, 0, 255), 1)
    
    # Info text
    n_det = len(ids) if ids is not None else 0
    n_rej = len(rejected) if rejected else 0
    cv2.putText(display, f"Detected: {n_det} | Rejected: {n_rej}", (10, 25),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv2.putText(display, "Press 's' to save debug crops", (10, 50),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    cv2.imshow("Debug - Full Frame", display)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        timestamp = datetime.now().strftime("%H%M%S")
        prefix = f"{save_dir}/frame{frame_count}_{timestamp}"
        
        # Save full frame
        cv2.imwrite(f"{prefix}_full.png", frame)
        cv2.imwrite(f"{prefix}_gray.png", gray)
        print(f"\n{'='*60}")
        print(f"SAVED frame {frame_count} @ {timestamp}")
        print(f"Detected IDs: {ids.flatten().tolist() if ids is not None else []}")
        print(f"Rejected candidates: {n_rej}")
        
        # 3. Find ALL contours (RETR_TREE) and save crops
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        
        # Multiple threshold methods
        thresh_methods = {
            "adaptive": cv2.adaptiveThreshold(enhanced, 255, 
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 7),
            "adaptive_inv": cv2.adaptiveThreshold(enhanced, 255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 7),
            "otsu": cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1],
        }
        
        crop_idx = 0
        all_rois = []
        
        for method_name, binary in thresh_methods.items():
            contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 100:  # skip tiny
                    continue
                
                rect = cv2.boundingRect(cnt)
                x, y, w, h = rect
                aspect = w / h if h > 0 else 0
                
                # Filter: roughly square-ish (ArUco markers are square)
                if aspect < 0.4 or aspect > 2.5:
                    continue
                
                # Skip if too similar to existing
                duplicate = False
                for existing in all_rois:
                    overlap = max(0, min(x+w, existing[0]+existing[2]) - max(x, existing[0])) * \
                              max(0, min(y+h, existing[1]+existing[3]) - max(y, existing[1]))
                    if overlap > 0.5 * min(w*h, existing[2]*existing[3]):
                        duplicate = True
                        break
                if duplicate:
                    continue
                all_rois.append(rect)
                
                # Crop with padding
                pad = max(15, int(0.25 * max(w, h)))
                x1 = max(0, x - pad)
                y1 = max(0, y - pad)
                x2 = min(gray.shape[1], x + w + pad)
                y2 = min(gray.shape[0], y + h + pad)
                
                crop = gray[y1:y2, x1:x2].copy()
                if crop.size == 0:
                    continue
                
                # Upscale to ~300px
                max_side = max(crop.shape)
                scale = max(1.0, 300.0 / max_side)
                upscaled = cv2.resize(crop, None, fx=scale, fy=scale, 
                                     interpolation=cv2.INTER_CUBIC)
                
                # Sharpen
                blurred = cv2.GaussianBlur(upscaled, (0, 0), 1.5)
                sharpened = cv2.addWeighted(upscaled, 1.8, blurred, -0.8, 0)
                
                # CLAHE on crop
                crop_clahe = clahe.apply(sharpened)
                
                # Binary versions
                _, binary_crop = cv2.threshold(crop_clahe, 0, 255, 
                                               cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                inv_binary = cv2.bitwise_not(binary_crop)
                
                # Try detection on each version
                for version_name, img in [("sharp", sharpened), ("clahe", crop_clahe),
                                           ("binary", binary_crop), ("inverted", inv_binary)]:
                    # Add white border
                    bordered = cv2.copyMakeBorder(img, 30, 30, 30, 30,
                                                  cv2.BORDER_CONSTANT, value=255)
                    
                    c2, id2, r2 = aruco.detectMarkers(bordered, aruco_dict, parameters=params)
                    
                    detected_str = ""
                    if id2 is not None and len(id2) > 0:
                        detected_str = f"_DETECTED_ID{id2.flatten()[0]}"
                        # Draw detection on bordered
                        aruco.drawDetectedMarkers(bordered, c2, id2)
                    
                    fname = f"{prefix}_crop{crop_idx}_{method_name}_{version_name}" \
                            f"_{w}x{h}{detected_str}.png"
                    cv2.imwrite(fname, bordered)
                
                crop_idx += 1
        
        # 4. Also save specific areas around detected markers (tab regions)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                marker_rect = cv2.boundingRect(corners[i][0])
                mx, my, mw, mh = marker_rect
                
                # Search in 4 directions around detected marker
                directions = {
                    "top": (mx - mw//4, my - int(mh*0.5), mw + mw//2, int(mh*0.5)),
                    "bottom": (mx - mw//4, my + mh, mw + mw//2, int(mh*0.5)),
                    "left": (mx - int(mw*0.5), my - mh//4, int(mw*0.5), mh + mh//2),
                    "right": (mx + mw, my - mh//4, int(mw*0.5), mh + mh//2),
                    "overlap_top": (mx, my - int(mh*0.3), mw, int(mh*0.6)),
                    "overlap_bottom": (mx, my + int(mh*0.7), mw, int(mh*0.6)),
                }
                
                for dir_name, (rx, ry, rw, rh) in directions.items():
                    rx = max(0, rx)
                    ry = max(0, ry)
                    rx2 = min(gray.shape[1], rx + rw)
                    ry2 = min(gray.shape[0], ry + rh)
                    
                    if rx2 - rx < 10 or ry2 - ry < 10:
                        continue
                    
                    region = gray[ry:ry2, rx:rx2].copy()
                    scale = max(1.0, 300.0 / max(region.shape))
                    up = cv2.resize(region, None, fx=scale, fy=scale,
                                   interpolation=cv2.INTER_CUBIC)
                    
                    # Sharpen + CLAHE
                    blr = cv2.GaussianBlur(up, (0, 0), 1.5)
                    shrp = cv2.addWeighted(up, 1.8, blr, -0.8, 0)
                    enh = clahe.apply(shrp)
                    
                    # Add border and try detect
                    bordered = cv2.copyMakeBorder(enh, 40, 40, 40, 40,
                                                  cv2.BORDER_CONSTANT, value=255)
                    
                    c3, id3, _ = aruco.detectMarkers(bordered, aruco_dict, parameters=params)
                    det_str = ""
                    if id3 is not None and len(id3) > 0:
                        det_str = f"_DETECTED_ID{id3.flatten()[0]}"
                        aruco.drawDetectedMarkers(bordered, c3, id3)
                    
                    fname = f"{prefix}_id{marker_id}_{dir_name}{det_str}.png"
                    cv2.imwrite(fname, bordered)
                    print(f"  Saved tab region: id{marker_id}_{dir_name} "
                          f"({rx2-rx}x{ry2-ry}px){det_str}")
        
        print(f"Total crops saved: {crop_idx}")
        print(f"{'='*60}")
        frame_count += 1

cap.release()
cv2.destroyAllWindows()
