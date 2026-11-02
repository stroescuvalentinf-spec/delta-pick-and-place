import cv2
import numpy as np
import time
import threading
from PIL import Image
import customtkinter as ctk
import tkinter.messagebox as tkmb


class VisionManager:
    # Physical demonstration square size in mm (30.10 mm x 30.10 mm)
    OBJECT_NOMINAL_SIZE_MM = 30.10
    # Percentage threshold above single-square dimension to classify as multiple objects (28%)
    DIMENSION_OVERFLOW_RATIO = 1.28
    # Minimum hull solidity before black background indentations indicate touching objects
    MIN_SINGLE_SOLIDITY = 0.88

    def __init__(self, main_win):
        self.win = main_win
        self.running = False
        self.thread = None
        self.cap = None
        self.last_frame_bgr = None

        self.mode = "CALIBRATE"
        self.last_clicked_pixel = None
        self.manual_test_pixel = None
        self.test_target = None

        self.calib_pts_pixel = []
        self.calib_pts_world = []
        self.transform_matrix = None

        self.camera_offset = [0, 0]
        self.test_recipe = {}

        self.roi = [0, 0, 640, 480]
        self.roi_dragging = False
        self.roi_start_pt = None
        self.roi_current_pt = None

        # Morphological structuring elements:
        # Tight (3, 3) kernels remove isolated noise and pinhole glare WITHOUT bridging
        # narrow black conveyor gaps between close objects.
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.kernel_blackhat = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def build_ui_connects(self):
        self.win.btn_camera_toggle.configure(command=self.toggle_camera)

        self.win.lbl_video_feed.bind("<ButtonPress-1>", self.on_video_press)
        self.win.lbl_video_feed.bind("<B1-Motion>", self.on_video_drag)
        self.win.lbl_video_feed.bind("<ButtonRelease-1>", self.on_video_release)

        self.win.btn_calib_add.configure(command=self.add_calibration_point)
        self.win.btn_calib_compute.configure(command=self.compute_calibration)
        self.win.btn_calib_reset.configure(command=self.reset_calibration)
        self.win.btn_test_mode.configure(command=self.toggle_test_mode)
        self.win.btn_test_move.configure(command=self.move_to_test_point)
        self.win.btn_teach_test.configure(command=self.teach_test_target)

        if hasattr(self.win, 'btn_set_roi'):
            self.win.btn_set_roi.configure(command=self.start_roi_drawing)
        if hasattr(self.win, 'btn_reset_roi'):
            self.win.btn_reset_roi.configure(command=self.reset_roi)

    def build_filtered_mask(self, frame_bgr, lower, upper):
        """
        Pre-processes the BGR image with a light Gaussian filter to suppress sensor
        chroma noise, applies HSV thresholding + ROI masking, and regularizes the
        binary mask using tight Morphological Opening and Closing so narrow black
        conveyor gaps between adjacent objects are preserved.
        """
        blurred = cv2.GaussianBlur(frame_bgr, (3, 3), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        if hasattr(self, 'roi') and self.roi and len(self.roi) == 4 and self.roi != [0, 0, 640, 480]:
            rx1, ry1, rx2, ry2 = self.roi
            rx1, rx2 = min(rx1, rx2), max(rx1, rx2)
            ry1, ry2 = min(ry1, ry2), max(ry1, ry2)
            roi_mask = np.zeros_like(mask)
            roi_mask[max(0, ry1):min(frame_bgr.shape[0], ry2), max(0, rx1):min(frame_bgr.shape[1], rx2)] = 255
            mask = cv2.bitwise_and(mask, roi_mask)

        # Remove isolated background specks and fill tiny internal pinholes without bridging close parts
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close, iterations=1)
        return mask

    def _measure_contour_metrics(self, cnt):
        """
        Computes pixel and world-projected geometry metrics for a contour to determine
        whether it represents a single 30.10 mm square or multiple close/touching objects.
        """
        hull = cv2.convexHull(cnt)
        cnt_area = float(cv2.contourArea(cnt))
        hull_area = float(cv2.contourArea(hull))
        solidity = cnt_area / max(hull_area, 1.0)

        rect = cv2.minAreaRect(cnt)
        (rcx, rcy), (rw, rh), rangle = rect
        long_side_px = float(max(rw, rh))
        short_side_px = float(max(1.0, min(rw, rh)))
        aspect_ratio_px = long_side_px / short_side_px

        aspect_ratio_world = aspect_ratio_px
        if self.transform_matrix is not None and rw > 1.0 and rh > 1.0:
            try:
                box_pts = cv2.boxPoints(rect).reshape(-1, 1, 2).astype(np.float32)
                world_pts = cv2.perspectiveTransform(box_pts, self.transform_matrix).reshape(4, 2)
                d1 = float(np.linalg.norm(world_pts[1] - world_pts[0]))
                d2 = float(np.linalg.norm(world_pts[2] - world_pts[1]))
                w_long = max(d1, d2)
                w_short = max(1e-5, min(d1, d2))
                aspect_ratio_world = w_long / w_short
            except Exception:
                aspect_ratio_world = aspect_ratio_px

        effective_ratio = max(aspect_ratio_px, aspect_ratio_world)
        est_length_mm = self.OBJECT_NOMINAL_SIZE_MM * effective_ratio

        return {
            'hull': hull,
            'cnt_area': cnt_area,
            'hull_area': hull_area,
            'solidity': solidity,
            'rect': rect,
            'long_side_px': long_side_px,
            'short_side_px': short_side_px,
            'effective_ratio': effective_ratio,
            'est_length_mm': est_length_mm
        }

    def _is_merged_contour(self, metrics, ref_single_area=None):
        """
        Checks if a contour exceeds the expected 30.10 mm square dimensions or contains
        black conveyor background indentations between touching parts.
        """
        # 1. Dimension check: length exceeds width (30.10 mm square) by > 28% (~38.5 mm)
        if metrics['effective_ratio'] > self.DIMENSION_OVERFLOW_RATIO:
            return True

        # 2. Solidity check: black background wedges inside the convex hull (corner/angled touch)
        if metrics['solidity'] < self.MIN_SINGLE_SOLIDITY and metrics['effective_ratio'] > 1.12:
            return True

        # 3. Area check against single square reference area
        expected_sq_area = metrics['short_side_px'] * metrics['short_side_px']
        if metrics['cnt_area'] > 1.55 * expected_sq_area:
            return True

        if ref_single_area is not None and ref_single_area > 0:
            if metrics['cnt_area'] > 1.60 * ref_single_area:
                return True

        return False

    def _cut_by_convexity_defects(self, sub_mask, cnt_local):
        """
        Uses the black conveyor background wedges (convexity defects) intruding between
        two touching squares to sever the contact neck.
        """
        if len(cnt_local) < 8:
            return False

        hull_idx = cv2.convexHull(cnt_local, returnPoints=False)
        if hull_idx is None or len(hull_idx) < 4:
            return False

        try:
            defects = cv2.convexityDefects(cnt_local, hull_idx)
        except Exception:
            return False

        if defects is None or len(defects) == 0:
            return False

        # Collect significant black-background indentations (depth >= 2.0 px)
        valid_defects = []
        n_pts = len(cnt_local)
        for i in range(defects.shape[0]):
            s_idx, e_idx, f_idx, depth_fixed = defects[i, 0]
            depth_px = depth_fixed / 256.0
            if depth_px >= 2.0:
                far_pt = cnt_local[f_idx][0]
                valid_defects.append({
                    'f_idx': int(f_idx),
                    'pt': (int(far_pt[0]), int(far_pt[1])),
                    'depth': float(depth_px)
                })

        if not valid_defects:
            return False

        valid_defects.sort(key=lambda d: d['depth'], reverse=True)

        # Case A: Two or more opposing black wedges -> cut directly between their tips
        if len(valid_defects) >= 2:
            best_pair = None
            best_score = -1.0
            for i in range(len(valid_defects)):
                for j in range(i + 1, len(valid_defects)):
                    d1 = valid_defects[i]
                    d2 = valid_defects[j]
                    # Ensure the two defects are not at the exact same pixel
                    dist = float(np.hypot(d1['pt'][0] - d2['pt'][0], d1['pt'][1] - d2['pt'][1]))
                    if dist < 2.0:
                        continue
                    # Check perimeter index separation so they lie on opposite sides of the neck
                    idx_diff = abs(d1['f_idx'] - d2['f_idx'])
                    perim_sep = min(idx_diff, n_pts - idx_diff)
                    if perim_sep < max(3, n_pts // 10):
                        continue
                    score = (d1['depth'] + d2['depth']) / max(dist, 4.0)
                    if score > best_score:
                        best_score = score
                        best_pair = (d1['pt'], d2['pt'])

            if best_pair is not None:
                cv2.line(sub_mask, best_pair[0], best_pair[1], 0, thickness=2)
                return True

        # Case B: Single deep black wedge (e.g., V-notch between two squares touching at one corner)
        deepest = valid_defects[0]
        if deepest['depth'] >= 3.0:
            f_idx = deepest['f_idx']
            f_pt = np.array(deepest['pt'], dtype=np.float32)
            min_arc_sep = max(4, n_pts // 5)

            best_opp_pt = None
            min_dist = float('inf')
            for k in range(n_pts):
                arc_dist = min(abs(k - f_idx), n_pts - abs(k - f_idx))
                if arc_dist < min_arc_sep:
                    continue
                cand_pt = cnt_local[k][0].astype(np.float32)
                d = float(np.linalg.norm(cand_pt - f_pt))
                if 2.0 <= d < min_dist:
                    min_dist = d
                    best_opp_pt = (int(cand_pt[0]), int(cand_pt[1]))

            if best_opp_pt is not None:
                cv2.line(sub_mask, deepest['pt'], best_opp_pt, 0, thickness=2)
                return True

        return False

    def _split_by_watershed(self, sub_bgr, sub_mask):
        """
        Applies Euclidean Distance Transform + Watershed to separate touching squares
        around their distinct distance peaks.
        """
        dist = cv2.distanceTransform(sub_mask, cv2.DIST_L2, 5)
        dist_max = float(dist.max())
        if dist_max <= 2.0:
            return sub_mask

        dist_smooth = cv2.GaussianBlur(dist, (3, 3), 0)

        # Find threshold level that splits the distance map into >= 2 core seed markers
        markers_fg = None
        for frac in (0.62, 0.55, 0.48, 0.42, 0.36):
            _, fg = cv2.threshold(dist_smooth, frac * dist_max, 255, cv2.THRESH_BINARY)
            fg = np.uint8(fg)
            num_labels, _ = cv2.connectedComponents(fg)
            if num_labels >= 3:  # background (0) + at least 2 foreground object seeds
                markers_fg = fg
                break

        if markers_fg is None:
            return sub_mask

        sure_bg = cv2.dilate(sub_mask, self.kernel_open, iterations=2)
        unknown = cv2.subtract(sure_bg, markers_fg)

        _, markers = cv2.connectedComponents(markers_fg)
        markers = markers + 1
        markers[unknown == 255] = 0

        ws_input = sub_bgr.copy()
        markers = cv2.watershed(ws_input, markers)

        # Create cut lines along watershed boundaries (markers == -1) between foreground regions
        boundary_mask = np.zeros_like(sub_mask)
        boundary_mask[markers == -1] = 255
        # Remove outer image border from boundary mask
        boundary_mask[0, :] = 0
        boundary_mask[-1, :] = 0
        boundary_mask[:, 0] = 0
        boundary_mask[:, -1] = 0
        boundary_mask = cv2.dilate(boundary_mask, self.kernel_open, iterations=1)

        result_mask = sub_mask.copy()
        result_mask[boundary_mask > 0] = 0
        return result_mask

    def _split_by_oriented_box(self, sub_mask, cnt_local):
        """
        Fallback geometric splitter: slices an elongated merged contour perpendicular
        to its major axis into N equal 30.10 mm square sub-regions.
        """
        rect = cv2.minAreaRect(cnt_local)
        (cx, cy), (rw, rh), angle = rect
        long_side = max(rw, rh)
        short_side = max(1.0, min(rw, rh))
        n_parts = max(2, int(round(long_side / short_side)))

        box = cv2.boxPoints(rect)
        # Identify major and minor axis unit vectors from box corners
        v01 = box[1] - box[0]
        v12 = box[2] - box[1]
        len01 = float(np.linalg.norm(v01))
        len12 = float(np.linalg.norm(v12))

        if len01 >= len12:
            u_major = v01 / max(len01, 1e-5)
            u_minor = v12 / max(len12, 1e-5)
        else:
            u_major = v12 / max(len12, 1e-5)
            u_minor = v01 / max(len01, 1e-5)

        center = np.array([cx, cy], dtype=np.float32)
        half_span_minor = 0.75 * short_side

        result_mask = sub_mask.copy()
        for k in range(1, n_parts):
            offset_major = ((float(k) / float(n_parts)) - 0.5) * long_side
            cut_center = center + offset_major * u_major
            p1 = cut_center - half_span_minor * u_minor
            p2 = cut_center + half_span_minor * u_minor
            cv2.line(
                result_mask,
                (int(round(p1[0])), int(round(p1[1]))),
                (int(round(p2[0])), int(round(p2[1]))),
                0,
                thickness=2
            )
        return result_mask

    def _separate_merged_contour(self, frame_bgr, cnt, min_area):
        """
        Separates a multi-object contour into individual single-square contours using:
        1. Black conveyor background valley / dark seam removal
        2. Opposing convexity defects cut across black background wedges
        3. Distance Transform + Watershed segmentation
        4. Oriented bounding-box geometric bisection fallback
        """
        h_img, w_img = frame_bgr.shape[:2]
        bx, by, bw, bh = cv2.boundingRect(cnt)
        pad = 6
        x1 = max(0, bx - pad)
        y1 = max(0, by - pad)
        x2 = min(w_img, bx + bw + pad)
        y2 = min(h_img, by + bh + pad)

        sub_bgr = frame_bgr[y1:y2, x1:x2]
        sub_mask = np.zeros((y2 - y1, x2 - x1), dtype=np.uint8)
        cnt_local = cnt - np.array([[x1, y1]])
        cv2.drawContours(sub_mask, [cnt_local], -1, 255, -1)

        # --- Stage 2A: Reinforce Black Conveyor Background Gap ---
        sub_hsv = cv2.cvtColor(sub_bgr, cv2.COLOR_BGR2HSV)
        v_chan = sub_hsv[:, :, 2]
        obj_v_pixels = v_chan[sub_mask > 0]
        if obj_v_pixels.size > 0:
            v_median = float(np.median(obj_v_pixels))
            # Detect dark valleys (black conveyor belt showing between two lighter objects)
            blackhat = cv2.morphologyEx(v_chan, cv2.MORPH_BLACKHAT, self.kernel_blackhat)
            dark_seam = (v_chan < (0.58 * v_median)) | (blackhat > max(18.0, 0.22 * v_median))
            sub_mask[dark_seam] = 0
            sub_mask = cv2.morphologyEx(sub_mask, cv2.MORPH_OPEN, self.kernel_open, iterations=1)

        # --- Stage 2B: Cut Between Black Background Convexity Defects ---
        sub_cnts, _ = cv2.findContours(sub_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for sc in sub_cnts:
            if cv2.contourArea(sc) > min_area * 0.45:
                sc_metrics = self._measure_contour_metrics(sc)
                if self._is_merged_contour(sc_metrics):
                    self._cut_by_convexity_defects(sub_mask, sc)

        # --- Stage 2C: Distance Transform + Watershed if Still Merged ---
        sub_cnts, _ = cv2.findContours(sub_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        still_merged = False
        for sc in sub_cnts:
            if cv2.contourArea(sc) > min_area * 0.45:
                if self._is_merged_contour(self._measure_contour_metrics(sc)):
                    still_merged = True
                    break

        if still_merged:
            sub_mask = self._split_by_watershed(sub_bgr, sub_mask)

        # --- Stage 2D: Oriented Bounding-Box Split Fallback if Still Merged ---
        sub_cnts, _ = cv2.findContours(sub_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for sc in sub_cnts:
            if cv2.contourArea(sc) > min_area * 0.45:
                sc_metrics = self._measure_contour_metrics(sc)
                if sc_metrics['effective_ratio'] > self.DIMENSION_OVERFLOW_RATIO:
                    single_mask = np.zeros_like(sub_mask)
                    cv2.drawContours(single_mask, [sc], -1, 255, -1)
                    split_single = self._split_by_oriented_box(single_mask, sc)
                    sub_mask[single_mask > 0] = split_single[single_mask > 0]

        # Collect final separated contours and shift back to full-frame coordinates
        final_sub_cnts, _ = cv2.findContours(sub_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        separated = []
        min_sub_area = max(120.0, float(min_area) * 0.55)
        for sc in final_sub_cnts:
            hull_sc = cv2.convexHull(sc)
            if cv2.contourArea(hull_sc) > min_sub_area:
                sc_full = sc + np.array([[x1, y1]])
                separated.append(sc_full)

        return separated if separated else [cnt]

    def extract_separated_regions(self, frame_bgr, mask, min_area):
        """
        Unified detection & splitting pipeline used by live HMI detection, Vision Test mode,
        Master Image Teaching, and the Recipe Wizard live preview.
        Returns a list of dicts containing individual object geometry:
        [{'hull': hull, 'box': box, 'cx': cX, 'cy': cY, 'rect': rect}, ...]
        """
        raw_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not raw_contours:
            return []

        # First pass: measure candidate contours and estimate single-square reference area
        candidates = []
        single_areas = []
        for cnt in raw_contours:
            metrics = self._measure_contour_metrics(cnt)
            if metrics['hull_area'] > min_area:
                candidates.append((cnt, metrics))
                if (metrics['effective_ratio'] <= self.DIMENSION_OVERFLOW_RATIO and
                        metrics['solidity'] >= self.MIN_SINGLE_SOLIDITY):
                    single_areas.append(metrics['cnt_area'])

        ref_single_area = float(np.median(single_areas)) if single_areas else None

        # Second pass: keep single-square contours as-is, and split merged multi-object contours
        final_contours = []
        for cnt, metrics in candidates:
            if self._is_merged_contour(metrics, ref_single_area):
                split_cnts = self._separate_merged_contour(frame_bgr, cnt, min_area)
                final_contours.extend(split_cnts)
            else:
                final_contours.append(cnt)

        # Build individual geometric descriptors for each separated object
        regions = []
        for cnt in final_contours:
            hull = cv2.convexHull(cnt)
            rect = cv2.minAreaRect(hull)
            box = np.intp(cv2.boxPoints(rect))
            cX = int(round(rect[0][0]))
            cY = int(round(rect[0][1]))
            regions.append({
                'cnt': cnt,
                'hull': hull,
                'box': box,
                'cx': cX,
                'cy': cY,
                'rect': rect
            })

        return regions

    def sample_hsv_region(self, frame_bgr, u, v, half_win=3):
        """
        Samples a (2*half_win+1)x(2*half_win+1) neighborhood around pixel (u, v)
        on a smoothed image so single-pixel glare or noise does not skew thresholds.
        Because the conveyor background is black, s_max and v_max default to 255
        so shaded or saturated perimeter edges of bright objects (like yellow) are preserved.
        """
        h_img, w_img = frame_bgr.shape[:2]
        u = max(0, min(w_img - 1, int(u)))
        v = max(0, min(h_img - 1, int(v)))

        blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
        y1, y2 = max(0, v - half_win), min(h_img, v + half_win + 1)
        x1, x2 = max(0, u - half_win), min(w_img, u + half_win + 1)

        patch_bgr = blurred[y1:y2, x1:x2]
        patch_hsv = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV)

        h = int(np.median(patch_hsv[:, :, 0]))
        s = int(np.median(patch_hsv[:, :, 1]))
        v_val = int(np.median(patch_hsv[:, :, 2]))

        bounds = {
            'h_min': max(0, h - 20),
            'h_max': min(179, h + 20),
            's_min': max(20, s - 80),
            's_max': 255,
            'v_min': max(40, v_val - 90),
            'v_max': 255
        }
        return h, s, v_val, bounds

    def start_roi_drawing(self):
        self.mode = "DRAW_ROI"
        self.win.lbl_calib_info.configure(
            text="Mode: DRAW ROI - Click and drag a rectangle over the conveyor area on the video feed.")
        if hasattr(self.win, 'btn_set_roi'):
            self.win.btn_set_roi.configure(text="DRAGGING...\n(Release mouse)")

    def reset_roi(self):
        self.roi = [0, 0, 640, 480]
        self.win.save_config()
        self.win.lbl_calib_info.configure(text="Conveyor ROI reset to full video frame.")
        tkmb.showinfo("ROI Reset", "Conveyor ROI reset to full frame (640x480).")

    def on_video_press(self, event):
        if self.mode == "DRAW_ROI":
            self.roi_dragging = True
            self.roi_start_pt = (event.x, event.y)
            self.roi_current_pt = (event.x, event.y)
        else:
            self.on_video_clicked(event)

    def on_video_drag(self, event):
        if self.mode == "DRAW_ROI" and self.roi_dragging:
            self.roi_current_pt = (max(0, min(640, event.x)), max(0, min(480, event.y)))

    def on_video_release(self, event):
        if self.mode == "DRAW_ROI" and self.roi_dragging:
            self.roi_dragging = False
            end_pt = (max(0, min(640, event.x)), max(0, min(480, event.y)))
            x1, y1 = min(self.roi_start_pt[0], end_pt[0]), min(self.roi_start_pt[1], end_pt[1])
            x2, y2 = max(self.roi_start_pt[0], end_pt[0]), max(self.roi_start_pt[1], end_pt[1])

            if (x2 - x1) > 10 and (y2 - y1) > 10:
                self.roi = [x1, y1, x2, y2]
                self.win.save_config()
                self.win.lbl_calib_info.configure(text=f"Conveyor ROI saved: [{x1}, {y1}] to [{x2}, {y2}]")
                tkmb.showinfo("ROI Saved",
                              f"Conveyor ROI defined successfully.\nObjects outside this area will be ignored.")
            else:
                self.win.lbl_calib_info.configure(text="ROI drag too small, cancelled.")

            self.mode = "CALIBRATE"
            if hasattr(self.win, 'btn_set_roi'):
                self.win.btn_set_roi.configure(text="Draw\nROI Area")

    def toggle_camera(self):
        if self.running:
            self.stop()
            self.win.btn_camera_toggle.configure(text="START CAMERA STREAM")
            self.win.lbl_vision_status.configure(text="Status: Stopped")
        else:
            self.running = True
            self.thread = threading.Thread(target=self._process_loop, daemon=True)
            self.thread.start()
            self.win.btn_camera_toggle.configure(text="STOP STREAM")
            self.win.lbl_vision_status.configure(text="Status: Starting...")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()
            self.cap = None

    def _process_loop(self):
        cam_selection = str(self.win.cmb_camera_id.get())
        cam_path = cam_selection
        cam_index = 0

        if "/dev/video" in cam_selection:
            try:
                cam_index = int(cam_selection.replace("/dev/video", ""))
            except ValueError:
                cam_index = 0
        elif cam_selection.isdigit():
            cam_index = int(cam_selection)
            cam_path = f"/dev/video{cam_index}"

        self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        else:
            pipeline = f"v4l2src device={cam_path} ! image/jpeg,width=640,height=480 ! jpegdec ! videoconvert ! appsink"
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cam_index)

        if not self.cap.isOpened():
            self.win.after(0,
                           lambda: self.win.lbl_vision_status.configure(text=f"Error: Could not open {cam_selection}."))
            self.win.after(0, lambda: self.win.btn_camera_toggle.configure(text="START CAMERA STREAM"))
            self.running = False
            return

        self.win.after(0, lambda: self.win.lbl_vision_status.configure(text=f"Status: Running ({cam_selection})"))

        while self.running:
            ret, frame_bgr = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            self.last_frame_bgr = frame_bgr.copy()
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            if self.win.ee_pos is not None:
                rx, ry, rz = self.win.ee_pos
                txt_robot = f"Robot: X={rx:.1f} Y={ry:.1f}"
                cv2.putText(frame_rgb, txt_robot, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
                cv2.putText(frame_rgb, txt_robot, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            if self.roi and len(self.roi) == 4 and self.roi != [0, 0, 640, 480]:
                rx1, ry1, rx2, ry2 = self.roi
                rx1, rx2 = min(rx1, rx2), max(rx1, rx2)
                ry1, ry2 = min(ry1, ry2), max(ry1, ry2)
                cv2.rectangle(frame_rgb, (rx1, ry1), (rx2, ry2), (0, 255, 255), 2)
                cv2.putText(frame_rgb, "CONVEYOR ROI", (rx1 + 5, max(20, ry1 + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 255), 2)

            if self.mode == "DRAW_ROI" and self.roi_dragging and self.roi_start_pt and self.roi_current_pt:
                cv2.rectangle(frame_rgb, self.roi_start_pt, self.roi_current_pt, (255, 165, 0), 2)

            if self.mode == "CALIBRATE":
                for i, pt in enumerate(self.calib_pts_pixel):
                    cv2.circle(frame_rgb, (pt[0], pt[1]), 5, (255, 0, 0), -1)
                    txt = f"P{i + 1}"
                    ts, _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    tx = pt[0] + 10 if pt[0] + 10 + ts[0] < 640 else pt[0] - 10 - ts[0]
                    ty = pt[1] - 10 if pt[1] - 10 - ts[1] > 0 else pt[1] + 10 + ts[1]
                    cv2.putText(frame_rgb, txt, (int(tx), int(ty)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                    cv2.putText(frame_rgb, txt, (int(tx), int(ty)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                if self.last_clicked_pixel:
                    cx, cy = self.last_clicked_pixel
                    cv2.circle(frame_rgb, (cx, cy), 6, (0, 255, 255), -1)
                    txt = f"Pixel: {cx}, {cy}"
                    ts, _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    tx = cx + 10 if cx + 10 + ts[0] < 640 else cx - 10 - ts[0]
                    ty = cy - 10 if cy - 10 - ts[1] > 0 else cy + 10 + ts[1]
                    cv2.putText(frame_rgb, txt, (int(tx), int(ty)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                    cv2.putText(frame_rgb, txt, (int(tx), int(ty)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            elif self.mode == "TEST":
                if self.test_recipe and 'h_max' in self.test_recipe:
                    objs = self.detect_objects(self.test_recipe)
                    if objs:
                        self.test_target = (objs[0]['x'], objs[0]['y'])
                        for idx, obj in enumerate(objs):
                            if 'hull' in obj:
                                cv2.drawContours(frame_rgb, [obj['hull']], -1, (0, 255, 0), 2)
                            if 'box' in obj:
                                cv2.drawContours(frame_rgb, [obj['box']], 0, (0, 255, 255), 1)
                            cv2.circle(frame_rgb, (obj['cx'], obj['cy']), 6, (255, 0, 0), -1)

                            if idx == 0:
                                txt = f"Target X:{obj['x']:.1f} Y:{obj['y']:.1f}"
                            else:
                                txt = f"#{idx + 1} X:{obj['x']:.1f} Y:{obj['y']:.1f}"
                            ts, _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                            tx = obj['cx'] + 15 if obj['cx'] + 15 + ts[0] < 640 else obj['cx'] - 15 - ts[0]
                            ty = obj['cy'] - 15 if obj['cy'] - 15 - ts[1] > 0 else obj['cy'] + 15 + ts[1]
                            cv2.putText(frame_rgb, txt, (int(tx), int(ty)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                            cv2.putText(frame_rgb, txt, (int(tx), int(ty)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if self.manual_test_pixel:
                    mx, my = self.manual_test_pixel
                    cv2.circle(frame_rgb, (mx, my), 8, (255, 0, 255), -1)
                    if self.test_target:
                        txt = f"Move X:{self.test_target[0]:.1f} Y:{self.test_target[1]:.1f}"
                        ts, _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                        tx = mx + 15 if mx + 15 + ts[0] < 640 else mx - 15 - ts[0]
                        ty = my - 15 if my - 15 - ts[1] > 0 else my + 15 + ts[1]
                        cv2.putText(frame_rgb, txt, (int(tx), int(ty)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                        cv2.putText(frame_rgb, txt, (int(tx), int(ty)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            try:
                pil_img = Image.fromarray(frame_rgb)
                pil_img = pil_img.resize((640, 480), Image.Resampling.LANCZOS)
                ctk_img = ctk.CTkImage(light_image=pil_img, dark_image=pil_img, size=(640, 480))
                self.win.after(0, lambda i=ctk_img: self.win.lbl_video_feed.configure(image=i, text=""))
            except Exception:
                pass
            time.sleep(0.03)

        if self.cap:
            self.cap.release()

    def on_video_clicked(self, event):
        u, v = event.x, event.y
        if self.mode == "CALIBRATE":
            self.last_clicked_pixel = (u, v)
            if self.win.ee_pos is not None:
                self.win.txt_calib_x.delete(0, 'end')
                self.win.txt_calib_x.insert(0, f"{self.win.ee_pos[0]:.2f}")
                self.win.txt_calib_y.delete(0, 'end')
                self.win.txt_calib_y.insert(0, f"{self.win.ee_pos[1]:.2f}")
            self.win.lbl_calib_info.configure(text=f"Pixel ({u}, {v}) - Robot Coords auto-filled. Press 'Add Point'")

        elif self.mode == "TEACH_TARGET":
            if self.last_frame_bgr is not None:
                h, s, v_val, bounds = self.sample_hsv_region(self.last_frame_bgr, u, v)
                self.test_recipe = {
                    'h_min': bounds['h_min'], 'h_max': bounds['h_max'],
                    's_min': bounds['s_min'], 's_max': bounds['s_max'],
                    'v_min': bounds['v_min'], 'v_max': bounds['v_max'],
                    'min_area': 500
                }
                self.win.save_config()
                tkmb.showinfo("Target Taught",
                              f"Object Learned (H:{h}, S:{s}, V:{v_val}).\nEntering Test Mode automatically.")
                self.mode = "TEST"
                self.win.btn_test_mode.configure(text="DISABLE\nTEST MODE")
                self.win.lbl_calib_info.configure(text="Mode: TESTING (Tracking live target...)")

        elif self.mode == "TEST":
            self.manual_test_pixel = (u, v)
            if self.transform_matrix is not None:
                pts = np.array([[[u, v]]], dtype=np.float32)
                world = cv2.perspectiveTransform(pts, self.transform_matrix)
                final_x, final_y = world[0][0][0], world[0][0][1]
                self.test_target = (float(final_x), float(final_y))
                self.win.lbl_calib_info.configure(text=f"Manual Override: X={final_x:.2f}, Y={final_y:.2f}")

    def teach_test_target(self):
        self.mode = "TEACH_TARGET"
        self.manual_test_pixel = None
        self.win.btn_test_mode.configure(text="ENABLE\nTEST MODE")
        self.win.lbl_calib_info.configure(text="Mode: TEACH - Click the object on the live video feed!")

    def move_to_test_point(self):
        if self.mode == "TEST":
            if self.test_target is not None:
                if hasattr(self.win, 'hardware_manager') and self.win.hardware_manager.is_manual_mode:
                    tkmb.showwarning("Mode Error", "Switch to AUTO mode in the status bar before moving.")
                    return
                if not self.win.serial_manager.homed:
                    tkmb.showwarning("Homing Required", "Please Home the robot first in the Controls page!")
                    return
                wx, wy = self.test_target
                if self.win.ee_pos is not None:
                    ui_z = self.win.ee_pos[2] - self.win.offsets.get('z', 0.0) - self.win.offsets.get('tool', 0.0)
                else:
                    ui_z = -280.0
                target_pos = np.array([wx, wy, ui_z])
                success = self.win._attempt_set_ee_pos(target_pos, from_ui=True)
                if not success:
                    tkmb.showwarning("Kinematics Error", "Target position is out of reach or violates angle limits!")
            else:
                tkmb.showwarning("Test Mode",
                                 "Click on the video feed or teach an object to select a coordinate first.")
        else:
            tkmb.showwarning("Mode Error", "Must be in TEST mode to execute a test move.")

    def add_calibration_point(self):
        if not hasattr(self, 'last_clicked_pixel') or self.last_clicked_pixel is None:
            tkmb.showwarning("Calibration", "Click on the video feed first to select a pixel.")
            return
        try:
            wx = float(self.win.txt_calib_x.get())
            wy = float(self.win.txt_calib_y.get())
        except ValueError:
            tkmb.showwarning("Calibration", "Invalid World X/Y coordinates.")
            return
        self.calib_pts_pixel.append([self.last_clicked_pixel[0], self.last_clicked_pixel[1]])
        self.calib_pts_world.append([wx, wy])
        self.last_clicked_pixel = None
        count = len(self.calib_pts_pixel)
        self.win.btn_calib_add.configure(text=f"Add Point\n({count}/4)")
        self.win.lbl_calib_info.configure(text=f"Added point {count}/4")
        if count == 4:
            self.compute_calibration()

    def compute_calibration(self):
        if len(self.calib_pts_pixel) < 4:
            tkmb.showwarning("Calibration", "Need at least 4 points to compute transform matrix.")
            return
        src_pts = np.array(self.calib_pts_pixel, dtype=np.float32)
        dst_pts = np.array(self.calib_pts_world, dtype=np.float32)
        try:
            self.transform_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
            self.win.lbl_calib_info.configure(text="Calibration Matrix Computed & Saved Successfully!")
            self.win.save_config()
        except Exception as e:
            tkmb.showwarning("Calibration Error",
                             f"Points are invalid (Collinear line). Reset and try again.\nDetails: {e}")

    def reset_calibration(self):
        self.calib_pts_pixel = []
        self.calib_pts_world = []
        self.transform_matrix = None
        self.win.btn_calib_add.configure(text="Add Point\n(0/4)")
        self.win.lbl_calib_info.configure(text="Calibration Reset. Add 4 points.")

    def toggle_test_mode(self):
        if self.mode == "TEST":
            self.mode = "CALIBRATE"
            self.win.btn_test_mode.configure(text="ENABLE\nTEST MODE")
            self.win.lbl_calib_info.configure(text="Mode: CALIBRATION")
        else:
            self.mode = "TEST"
            self.test_target = None
            self.manual_test_pixel = None
            self.win.btn_test_mode.configure(text="DISABLE\nTEST MODE")
            self.win.lbl_calib_info.configure(text="Mode: TESTING (Tracking live target...)")

    def open_vision_teach_window(self, recipe, on_save_callback):
        teach_win = ctk.CTkToplevel(self.win)
        teach_win.title("Master Image Teaching")
        teach_win.geometry("1000x650")
        teach_win.attributes("-topmost", True)
        img_frame = ctk.CTkFrame(teach_win)
        img_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        lbl_img = ctk.CTkLabel(img_frame, text="No Snapshot. Click 'TAKE SNAPSHOT'.", width=640, height=480)
        lbl_img.pack(pady=20)
        ctrl_frame = ctk.CTkScrollableFrame(teach_win, width=320)
        try:
            ctrl_frame._scrollbar.configure(width=30)
        except Exception:
            pass
        ctrl_frame.pack(side="right", fill="y", padx=10, pady=10)
        self.snapshot_bgr = None

        def update_preview(*args):
            if self.snapshot_bgr is None:
                return
            img = self.snapshot_bgr.copy()
            lower = np.array([int(sliders['h_min'].get()), int(sliders['s_min'].get()), int(sliders['v_min'].get())])
            upper = np.array([int(sliders['h_max'].get()), int(sliders['s_max'].get()), int(sliders['v_max'].get())])
            mask = self.build_filtered_mask(img, lower, upper)

            if hasattr(self, 'roi') and self.roi and len(self.roi) == 4 and self.roi != [0, 0, 640, 480]:
                rx1, ry1, rx2, ry2 = self.roi
                rx1, rx2 = min(rx1, rx2), max(rx1, rx2)
                ry1, ry2 = min(ry1, ry2), max(ry1, ry2)
                cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (0, 255, 255), 2)

            min_area = int(sliders['min_area'].get())
            regions = self.extract_separated_regions(img, mask, min_area)
            for reg in regions:
                cv2.drawContours(img, [reg['hull']], -1, (0, 255, 0), 2)
                cv2.drawContours(img, [reg['box']], 0, (255, 255, 0), 1)
                cv2.circle(img, (reg['cx'], reg['cy']), 4, (0, 0, 255), -1)

            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(img_rgb).resize((640, 480), Image.Resampling.LANCZOS)
            ctk_img = ctk.CTkImage(light_image=pil_img, dark_image=pil_img, size=(640, 480))
            lbl_img.configure(image=ctk_img, text="")

        def take_snapshot():
            if self.last_frame_bgr is not None:
                self.snapshot_bgr = self.last_frame_bgr.copy()
                update_preview()
            else:
                tkmb.showwarning("Error", "No camera feed active. Start the camera stream first.", parent=teach_win)

        def on_img_click(event):
            if self.snapshot_bgr is None:
                return
            u, v = event.x, event.y
            if u >= 640 or v >= 480:
                return
            _, _, _, bounds = self.sample_hsv_region(self.snapshot_bgr, u, v)
            sliders['h_min'].set(bounds['h_min'])
            sliders['h_max'].set(bounds['h_max'])
            sliders['s_min'].set(bounds['s_min'])
            sliders['s_max'].set(bounds['s_max'])
            sliders['v_min'].set(bounds['v_min'])
            sliders['v_max'].set(bounds['v_max'])
            for k, sl in sliders.items():
                sl._command(sl.get())

        lbl_img.bind("<Button-1>", on_img_click)
        ctk.CTkButton(ctrl_frame, text="TAKE SNAPSHOT & CLICK IMAGE", command=take_snapshot, fg_color="#3498DB",
                      height=40,
                      font=("Arial", 14, "bold")).pack(pady=10, fill="x")
        sliders = {}

        def add_slider(name, key, from_, to_, default):
            ctk.CTkLabel(ctrl_frame, text=name, font=("Arial", 12, "bold")).pack(pady=(10, 0))
            val_lbl = ctk.CTkLabel(ctrl_frame, text=str(int(default)))
            val_lbl.pack()

            def on_slide(val, l=val_lbl):
                l.configure(text=str(int(val)))
                update_preview()

            s = ctk.CTkSlider(ctrl_frame, from_=from_, to=to_, command=on_slide, height=40, button_length=40)
            s.set(default)
            s.pack(fill="x")
            sliders[key] = s

        add_slider("Hue Min (0-179)", "h_min", 0, 179, recipe.get("h_min", 0))
        add_slider("Hue Max (0-179)", "h_max", 0, 179, recipe.get("h_max", 10))
        add_slider("Sat Min (0-255)", "s_min", 0, 255, recipe.get("s_min", 100))
        add_slider("Sat Max (0-255)", "s_max", 0, 255, recipe.get("s_max", 255))
        add_slider("Val Min (0-255)", "v_min", 0, 255, recipe.get("v_min", 100))
        add_slider("Val Max (0-255)", "v_max", 0, 255, recipe.get("v_max", 255))
        add_slider("Min Area (px)", "min_area", 0, 10000, recipe.get("min_area", 1000))

        def save_and_close():
            for k, s in sliders.items():
                recipe[k] = int(s.get())
            if on_save_callback:
                on_save_callback()
            teach_win.destroy()

        ctk.CTkButton(ctrl_frame, text="SAVE RECIPE & CLOSE", command=save_and_close, fg_color="#2ECC71", height=50,
                      font=("Arial", 14, "bold")).pack(pady=20, fill="x")

    def detect_objects(self, recipe):
        if self.transform_matrix is None or self.last_frame_bgr is None:
            return []
        if not recipe or 'h_max' not in recipe:
            return []

        lower = np.array([recipe.get('h_min', 0), recipe.get('s_min', 100), recipe.get('v_min', 100)])
        upper = np.array([recipe.get('h_max', 10), recipe.get('s_max', 255), recipe.get('v_max', 255)])
        mask = self.build_filtered_mask(self.last_frame_bgr, lower, upper)

        min_area = recipe.get('min_area', 1000)
        regions = self.extract_separated_regions(self.last_frame_bgr, mask, min_area)

        objs = []
        for reg in regions:
            cX = reg['cx']
            cY = reg['cy']
            pts = np.array([[[cX, cY]]], dtype=np.float32)
            world = cv2.perspectiveTransform(pts, self.transform_matrix)
            final_x = float(world[0][0][0])
            final_y = float(world[0][0][1])
            objs.append({
                'x': final_x,
                'y': final_y,
                'cx': cX,
                'cy': cY,
                'hull': reg['hull'],
                'box': reg['box']
            })
        return objs

    def detect_object(self, recipe):
        objs = self.detect_objects(recipe)
        return objs[0] if objs else None