import numpy as np
import customtkinter as ctk
from kinematics import delta_calcInverse
import animation
import tkinter.messagebox as tkmb
from PIL import Image
import cv2
import time
import copy


class EventHandlers:
    ACCEL_PER_JOG_VEL = 104.75
    # Center-to-center distance (mm) below which a second object is considered too close
    # to the picked object (covers touching 30.1mm blocks up to ~35mm edge-to-edge gap)
    CLOSE_OBJECT_MIN_DIST_MM = 65.0
    # Tracking association radius (mm) to follow an ignored close object across frames
    # Must remain < 30.10 mm (single object size) so tracking never jumps to the next valid object
    IGNORED_TRACK_MATCH_RADIUS_MM = 24.0

    def update_param_fields(self):
        if self.ee_pos is not None:
            try:
                self.x_edit.delete(0, "end")
                self.x_edit.insert(0, f"{self.ee_pos[0]:.2f}")
                self.y_edit.delete(0, "end")
                self.y_edit.insert(0, f"{self.ee_pos[1]:.2f}")
                self.z_edit.delete(0, "end")
                self.z_edit.insert(0, f"{self.ee_pos[2]:.2f}")
                st, _, _, _ = delta_calcInverse(self.ee_pos[0], self.ee_pos[1], self.ee_pos[2], self.e, self.f, self.re,
                                                self.rf, self.min_upper_arm_angle, self.max_upper_arm_angle)
                if st == 0:
                    self.lbl_ee.configure(
                        text=f"Coords: X{self.ee_pos[0]:.1f} Y{self.ee_pos[1]:.1f} Z{self.ee_pos[2]:.1f}")
                else:
                    self.lbl_ee.configure(text="Coords: INVALID/LIMIT")
            except:
                pass

    def _get_default_jog_vel(self, fallback=50.0):
        try:
            val = float(self.jog_vel_spin.get())
            return val if val > 0 else fallback
        except:
            return fallback

    def _calc_accel_from_vel(self, vel_mms=None):
        if vel_mms is None:
            vel_mms = self._get_default_jog_vel(50.0)
        return int(round(float(vel_mms) * self.ACCEL_PER_JOG_VEL))

    def _sync_accel_spin_ui(self, accel_int):
        try:
            if hasattr(self, 'accel_spin') and self.accel_spin is not None:
                curr_txt = self.accel_spin.get().strip()
                new_txt = str(int(accel_int))
                if curr_txt != new_txt:
                    self.accel_spin.delete(0, "end")
                    self.accel_spin.insert(0, new_txt)
        except:
            pass

    def on_jog_vel_changed(self, event=None, apply_hw=False):
        try:
            vel_mms = float(self.jog_vel_spin.get())
            if vel_mms > 0:
                accel_int = self._calc_accel_from_vel(vel_mms)
                self._sync_accel_spin_ui(accel_int)
        except:
            pass

    def _get_default_accel(self, fallback=35000.0):
        try:
            if hasattr(self, 'accel_spin') and self.accel_spin is not None:
                val = float(self.accel_spin.get())
                if val > 0:
                    return val
        except:
            pass
        try:
            vel_mms = float(self.jog_vel_spin.get())
            if vel_mms > 0:
                accel_int = self._calc_accel_from_vel(vel_mms)
                self._sync_accel_spin_ui(accel_int)
                return float(accel_int)
        except:
            pass
        return fallback

    def _apply_hw_accel(self, accel_val, force=False):
        # Hardware acceleration is now uploaded manually only via the UPLOAD PARAMS button in Settings
        pass

    def _get_roi_bounds(self):
        if hasattr(self, 'vision_manager') and hasattr(self.vision_manager, 'roi') and self.vision_manager.roi and len(
                self.vision_manager.roi) == 4:
            rx1, ry1, rx2, ry2 = self.vision_manager.roi
            rx1, rx2 = max(0, min(int(rx1), int(rx2))), min(640, max(int(rx1), int(rx2)))
            ry1, ry2 = max(0, min(int(ry1), int(ry2))), min(480, max(int(ry1), int(ry2)))
            if rx2 > rx1 and ry2 > ry1:
                return rx1, ry1, rx2, ry2
        return 0, 0, 640, 480

    def _world_x_to_pixel_x(self, target_x_mm, ref_y_px=None):
        rx1, ry1, rx2, ry2 = self._get_roi_bounds()
        if ref_y_px is None:
            ref_y_px = 0.5 * (ry1 + ry2)
        if hasattr(self, 'vision_manager') and self.vision_manager.transform_matrix is not None:
            M = self.vision_manager.transform_matrix
            denom = float(M[0, 0] - target_x_mm * M[2, 0])
            if abs(denom) > 1e-9:
                num = float(target_x_mm * (M[2, 1] * ref_y_px + M[2, 2]) - (M[0, 1] * ref_y_px + M[0, 2]))
                u_px = int(round(num / denom))
                return max(rx1, min(rx2, u_px))
        return int(round(0.5 * (rx1 + rx2)))

    def _pixel_x_to_world_x(self, pixel_x, ref_y_px=None):
        rx1, ry1, rx2, ry2 = self._get_roi_bounds()
        if ref_y_px is None:
            ref_y_px = 0.5 * (ry1 + ry2)
        if hasattr(self, 'vision_manager') and self.vision_manager.transform_matrix is not None:
            try:
                pts = np.array([[[float(pixel_x), float(ref_y_px)]]], dtype=np.float32)
                world = cv2.perspectiveTransform(pts, self.vision_manager.transform_matrix)
                return float(world[0][0][0])
            except Exception:
                pass
        return 40.0 if getattr(self, 'conveyor_speed_mm_s', 0.0) <= 0 else 26.0

    def _get_default_detect_line_px(self):
        default_world_x = 40.0 if getattr(self, 'conveyor_speed_mm_s', 0.0) <= 0 else 26.0
        return self._world_x_to_pixel_x(default_world_x)

    def _get_close_object_threshold_mm(self):
        conv_spd = abs(float(getattr(self, 'conveyor_speed_mm_s', 0.0)))
        return max(self.CLOSE_OBJECT_MIN_DIST_MM, conv_spd * 1.8)

    def _get_roi_world_x_span(self):
        rx1, ry1, rx2, ry2 = self._get_roi_bounds()
        ref_y = 0.5 * (ry1 + ry2)
        wx1 = self._pixel_x_to_world_x(rx1, ref_y_px=ref_y)
        wx2 = self._pixel_x_to_world_x(rx2, ref_y_px=ref_y)
        return min(wx1, wx2), max(wx1, wx2)

    def _collect_all_hmi_detections(self):
        entries = []
        if not hasattr(self, 'active_recipe') or not self.active_recipe:
            return entries
        default_line_px = self._get_default_detect_line_px()
        for obj_cfg in self.active_recipe.get('objects', []):
            objs = self.vision_manager.detect_objects(obj_cfg)
            if not objs:
                continue
            detect_line_px = int(obj_cfg.get('detect_line_px', default_line_px))
            for obj in objs:
                entries.append({
                    'obj': obj,
                    'obj_cfg': obj_cfg,
                    'detect_line_px': detect_line_px,
                    'ignored': False
                })
        return entries

    def _update_ignored_close_objects(self, all_entries, now=None):
        if now is None:
            now = time.time()
        if not hasattr(self, 'ignored_close_objects') or self.ignored_close_objects is None:
            self.ignored_close_objects = []
        if not hasattr(self, 'last_picked_belt_ref'):
            self.last_picked_belt_ref = None

        conv_spd = float(getattr(self, 'conveyor_speed_mm_s', 0.0))
        roi_x_min, roi_x_max = self._get_roi_world_x_span()

        # 1. Propagate existing ignored objects along the conveyor and prune those that left the ROI
        active_ignored = []
        for ign in self.ignored_close_objects:
            dt = max(0.0, now - float(ign['t']))
            pred_x = float(ign['x']) + conv_spd * dt
            pred_y = float(ign['y'])

            if (now - float(ign.get('created_t', ign['t']))) > 8.0:
                continue
            if conv_spd < -1e-3 and pred_x < (roi_x_min - 20.0):
                continue
            if conv_spd > 1e-3 and pred_x > (roi_x_max + 20.0):
                continue

            ign['_pred_x'] = pred_x
            ign['_pred_y'] = pred_y
            active_ignored.append(ign)

        # 2. Match detected objects against active ignored tracks and re-anchor coordinates
        for ign in active_ignored:
            best_entry = None
            best_dist = self.IGNORED_TRACK_MATCH_RADIUS_MM
            for entry in all_entries:
                if entry.get('ignored', False):
                    continue
                obj = entry['obj']
                dx = abs(float(obj['x']) - ign['_pred_x'])
                dy = abs(float(obj['y']) - ign['_pred_y'])
                dist = float(np.hypot(dx, dy))
                if dist <= best_dist and dx <= 22.0:
                    best_dist = dist
                    best_entry = entry
            if best_entry is not None:
                best_entry['ignored'] = True
                ign['x'] = float(best_entry['obj']['x'])
                ign['y'] = float(best_entry['obj']['y'])
                ign['t'] = now

        # 3. Check if any un-ignored object lies within the close-proximity zone of the last picked object
        # (handles cases where the 2nd close object finished entering the ROI while the 1st was being picked)
        if self.last_picked_belt_ref is not None:
            dt_pick = max(0.0, now - float(self.last_picked_belt_ref['t']))
            phantom_x = float(self.last_picked_belt_ref['x']) + conv_spd * dt_pick
            phantom_y = float(self.last_picked_belt_ref['y'])
            close_thresh_mm = self._get_close_object_threshold_mm()

            # Expire the picked belt reference once its close neighborhood has completely exited the ROI
            ref_expired = False
            if dt_pick > 8.0:
                ref_expired = True
            elif conv_spd < -1e-3 and (phantom_x + close_thresh_mm) < (roi_x_min - 20.0):
                ref_expired = True
            elif conv_spd > 1e-3 and (phantom_x - close_thresh_mm) > (roi_x_max + 20.0):
                ref_expired = True

            if ref_expired:
                self.last_picked_belt_ref = None
            else:
                for entry in all_entries:
                    if entry.get('ignored', False):
                        continue
                    obj = entry['obj']
                    dist_to_picked = float(np.hypot(float(obj['x']) - phantom_x, float(obj['y']) - phantom_y))
                    if dist_to_picked <= close_thresh_mm:
                        entry['ignored'] = True
                        active_ignored.append({
                            'x': float(obj['x']),
                            'y': float(obj['y']),
                            't': now,
                            'created_t': now
                        })

        self.ignored_close_objects = active_ignored

    def _register_close_objects_for_pick(self, picked_candidate, all_entries=None, now=None):
        if now is None:
            now = time.time()
        if not hasattr(self, 'ignored_close_objects') or self.ignored_close_objects is None:
            self.ignored_close_objects = []

        picked_obj = picked_candidate['obj']
        px = float(picked_obj['x'])
        py = float(picked_obj['y'])

        # Record conveyor reference for the picked first object
        self.last_picked_belt_ref = {
            'x': px,
            'y': py,
            't': now
        }

        if all_entries is None:
            all_entries = self._collect_all_hmi_detections()

        close_thresh_mm = self._get_close_object_threshold_mm()
        for entry in all_entries:
            obj = entry['obj']
            if obj is picked_obj:
                continue
            dist = float(np.hypot(float(obj['x']) - px, float(obj['y']) - py))
            # Skip duplicate detections of the exact same physical object at the same pixel center
            if dist < 8.0 and obj.get('cx') == picked_obj.get('cx') and obj.get('cy') == picked_obj.get('cy'):
                continue
            if dist <= close_thresh_mm:
                # Avoid duplicating an already-tracked ignored entry at the same spot
                already_tracked = False
                for ign in self.ignored_close_objects:
                    dt_ign = max(0.0, now - float(ign['t']))
                    ign_x = float(ign['x']) + float(getattr(self, 'conveyor_speed_mm_s', 0.0)) * dt_ign
                    ign_y = float(ign['y'])
                    if float(np.hypot(float(obj['x']) - ign_x, float(obj['y']) - ign_y)) <= self.IGNORED_TRACK_MATCH_RADIUS_MM:
                        ign['x'] = float(obj['x'])
                        ign['y'] = float(obj['y'])
                        ign['t'] = now
                        already_tracked = True
                        break
                if not already_tracked:
                    self.ignored_close_objects.append({
                        'x': float(obj['x']),
                        'y': float(obj['y']),
                        't': now,
                        'created_t': now
                    })

    def on_apply_target(self):
        if hasattr(self, 'hardware_manager') and self.hardware_manager.is_manual_mode:
            tkmb.showwarning("MODE ERROR", "Switch to AUTO mode for direct moves.")
            return
        if self.ee_pos is None:
            return
        try:
            pos = np.array([float(self.x_edit.get()), float(self.y_edit.get()), float(self.z_edit.get())])
            self._attempt_set_ee_pos(pos, from_ui=True)
        except:
            pass

    def apply_offsets(self):
        try:
            self.offsets['j1'] = float(self.joff1_edit.get())
            self.offsets['j2'] = float(self.joff2_edit.get())
            self.offsets['j3'] = float(self.joff3_edit.get())
            self.offsets['x'] = float(self.xoff_edit.get())
            self.offsets['y'] = float(self.yoff_edit.get())
            self.offsets['z'] = float(self.zoff_edit.get())
            self.offsets['tool'] = float(self.tool_offset_edit.get())
            self.e = float(self.e_edit.get())
            self.f = float(self.f_edit.get())
            self.re = float(self.re_edit.get())
            self.rf = float(self.rf_edit.get())
            self.update_param_fields()
        except ValueError:
            tkmb.showerror("Input Error", "Please enter valid numbers in the parameters page.")

    def save_offsets(self):
        self.apply_offsets()
        self.save_config()
        tkmb.showinfo("Success", "Parameters and Offsets Saved!")

    def reset_offsets(self):
        self.load_config()

    def on_move_to_target(self):
        self.on_apply_target()

    def start_jog_repeat(self, vec):
        if hasattr(self, 'hardware_manager') and not self.hardware_manager.is_manual_mode:
            tkmb.showwarning("MODE ERROR", "Switch to MANUAL mode to Jog.")
            return
        if self.ee_pos is None:
            return
        key = tuple(vec)
        if key in self._jog_repeat_timers:
            return
        self._jog_repeat_timers[key] = True
        self._jog_loop(key, vec)

    def _jog_loop(self, key, vec):
        if key not in self._jog_repeat_timers:
            return
        if hasattr(self, 'hardware_manager') and not self.hardware_manager.is_manual_mode:
            self.stop_jog_repeat(vec)
            return
        vel_mms = self._get_default_jog_vel(50.0)
        dt_cmd = 0.150
        dist = vel_mms * dt_cmd
        ui_pos = self.ee_pos - np.array([self.offsets['x'], self.offsets['y'], self.offsets['z']])
        target_pos = ui_pos + np.array(vec, dtype=float) * dist
        self._attempt_set_ee_pos(target_pos, from_ui=True, forced_vel=vel_mms * 60.0)
        self.after(100, lambda: self._jog_loop(key, vec))

    def stop_jog_repeat(self, vec):
        key = tuple(vec)
        if key in self._jog_repeat_timers:
            del self._jog_repeat_timers[key]
        if hasattr(self.serial_manager, 'serial_write'):
            self.serial_manager.serial_write("STOP")

    def on_recorder_play(self, _):
        if hasattr(self, 'hardware_manager') and self.hardware_manager.is_manual_mode:
            tkmb.showwarning("MODE ERROR", "Switch to AUTO mode to run programs.")
            return
        # Ensure HMI is not running while starting a TEACHING IDE program
        if getattr(self, 'hmi_running', False):
            self.on_hmi_stop()
        self.on_trajectory_requested(None)
        if not self.move_steps:
            return
        self.step_index = 0
        self.on_playback_timer()

    def _attempt_set_ee_pos(self, new_pos_ui, from_ui=False, forced_vel=None):
        if self.ee_pos is None:
            return False
        tool_offset = self.offsets.get('tool', 0.0)
        pos_machine = new_pos_ui.copy()
        pos_machine[2] += tool_offset
        pos_machine += np.array([self.offsets['x'], self.offsets['y'], self.offsets['z']])
        st, _, _, _ = delta_calcInverse(pos_machine[0], pos_machine[1], pos_machine[2], self.e, self.f, self.re,
                                        self.rf, self.min_upper_arm_angle, self.max_upper_arm_angle)
        if st == 0:
            self.ee_pos = pos_machine
            self.update_param_fields()
            if self.serial_manager.ser and self.serial_manager.homed:
                vel = forced_vel if forced_vel else 3000.0
                self._send_current_pos_to_hw(vel=vel)
            return True
        return False

    def _send_current_pos_to_hw(self, vel=None):
        if self.ee_pos is None:
            return
        st, t1, t2, t3 = delta_calcInverse(self.ee_pos[0], self.ee_pos[1], self.ee_pos[2], self.e, self.f, self.re,
                                           self.rf, self.min_upper_arm_angle, self.max_upper_arm_angle)
        if st == 0:
            t1 += self.offsets['j1']
            t2 += self.offsets['j2']
            t3 += self.offsets['j3']
            if vel is None:
                vel = 3000.0
            cmd = f"G1 A{t1:.3f} B{t2:.3f} C{t3:.3f} F{vel:.0f}"
            self.serial_manager.serial_write(cmd)

    def on_home_send(self):
        if hasattr(self, 'hardware_manager') and self.hardware_manager.is_manual_mode:
            tkmb.showwarning("MODE ERROR", "Switch to AUTO mode for Homing move.")
            return
        try:
            hx, hy, hz = float(self.home_x.get()), float(self.home_y.get()), float(self.home_z.get())
            self._attempt_set_ee_pos(np.array([hx, hy, hz]), from_ui=True)
        except:
            pass

    def on_stop(self):
        if hasattr(self, 'playback_timer'):
            self.playback_timer.stop()
        if hasattr(self, 'hmi_playback_timer'):
            self.hmi_playback_timer.stop()
        self.serial_manager.serial_write("STOP")
        self.on_tool_toggle(False, sync_ui=True)
        if hasattr(self, 'recorder_widget'):
            self.recorder_widget.clear_highlight()
            self.recorder_widget.reset_pause_button()

    def on_recorder_register(self, _):
        if self.ee_pos is not None:
            ui_pos = self.ee_pos - np.array([self.offsets['x'], self.offsets['y'], self.offsets['z']])
            vel_mms = self._get_default_jog_vel(50.0)
            accel_val = self._get_default_accel()
            self.recorder_widget.add_move({
                'x': ui_pos[0],
                'y': ui_pos[1],
                'z': ui_pos[2],
                'jog': vel_mms,
                'accel': accel_val
            })

    def on_recorder_pause_toggled(self, p):
        if p:
            self.playback_timer.stop()
        else:
            self.on_playback_timer()

    def on_trajectory_requested(self, _):
        self.move_steps = self.recorder_widget.moves()

    def on_tool_toggle(self, state, sync_ui=False):
        if hasattr(self, 'hardware_manager'):
            self.hardware_manager.set_tool_state(bool(state))
        if sync_ui and hasattr(self, 'recorder_widget'):
            self.recorder_widget.set_tool_checkbox(bool(state))

    def _execute_single_ide_step(self, step):
        line = step.get('line', 0)
        if line > 0 and hasattr(self, 'recorder_widget'):
            self.recorder_widget.highlight_line(line)
        if step['type'] == 'move':
            step_jog = step.get('jog')
            vel_mms = float(step_jog) if (step_jog is not None and float(step_jog) > 0) else self._get_default_jog_vel(50.0)
            target_ui = np.array([step['x'], step['y'], step['z']])
            if self.ee_pos is not None:
                curr_ui = self.ee_pos - np.array([self.offsets['x'], self.offsets['y'], self.offsets['z']])
                dist = np.linalg.norm(target_ui - curr_ui)
                time_to_move = dist / max(vel_mms, 0.1)
                delay = time_to_move * 0.6
            else:
                delay = 0.05
            if delay < 0.01:
                delay = 0.01
            self._attempt_set_ee_pos(target_ui, from_ui=False, forced_vel=vel_mms * 60.0)
            return delay
        elif step['type'] == 'delay':
            return float(step['dt'])
        elif step['type'] == 'tool':
            self.on_tool_toggle(step['state'], sync_ui=True)
            return 0.1
        return 0.05

    def on_playback_timer(self):
        # Strictly dedicated to TEACHING IDE playback (completely isolated from HMI)
        if not self.move_steps or self.step_index >= len(self.move_steps):
            if hasattr(self, 'recorder_widget'):
                self.recorder_widget.clear_highlight()
                if self.recorder_widget.loop_enabled and self.move_steps:
                    self.step_index = 0
                else:
                    self.recorder_widget.reset_pause_button()
                    return
            else:
                return
        step = self.move_steps[self.step_index]
        delay = self._execute_single_ide_step(step)
        self.step_index += 1
        self.playback_timer.start(int(delay * 1000))

    def on_recorder_step(self, _):
        if hasattr(self, 'hardware_manager') and self.hardware_manager.is_manual_mode:
            tkmb.showwarning("MODE ERROR", "Switch to AUTO mode to step through programs.")
            return
        if getattr(self, 'hmi_running', False):
            self.on_hmi_stop()
        if hasattr(self, 'playback_timer'):
            self.playback_timer.stop()
        if not self.move_steps:
            self.on_trajectory_requested(None)
        if not self.move_steps:
            return
        if self.step_index >= len(self.move_steps):
            self.step_index = 0
        step = self.move_steps[self.step_index]
        self._execute_single_ide_step(step)
        self.step_index += 1

    def on_hmi_playback_timer(self):
        # Strictly dedicated to HMI Pick & Place autonomous mode (completely isolated from TEACHING IDE)
        if not getattr(self, 'hmi_running', False):
            return
        if not self.hmi_move_steps or self.hmi_step_index >= len(self.hmi_move_steps):
            if self.hmi_state == "PICKING":
                if hasattr(self, 'current_pick_obj'):
                    self.hmi_stats[self.current_pick_obj] = self.hmi_stats.get(self.current_pick_obj, 0) + 1
                self.hmi_ok_parts += 1
                self.update_hmi_stats()
                # Always return to start (ready_pos) after Pick & Place before scanning for the next object
                self.hmi_state = "RETURNING"
                self.lbl_hmi_state.configure(text="STATE: RETURNING TO READY POS", text_color="yellow")
                rx = float(self.active_recipe['ready_pos']['x'])
                ry = float(self.active_recipe['ready_pos']['y'])
                rz = float(self.active_recipe['ready_pos']['z'])
                self.hmi_move_steps = [
                    {'type': 'move', 'x': rx, 'y': ry, 'z': rz}
                ]
                self.hmi_step_index = 0
                self.on_hmi_playback_timer()
                return
            elif self.hmi_state == "RETURNING":
                self.hmi_state = "SCANNING"
                self.lbl_hmi_state.configure(text="STATE: SCANNING FOR OBJECT", text_color="yellow")
                self.after(50, self.hmi_process_loop)
                return
            return

        step = self.hmi_move_steps[self.hmi_step_index]
        if step['type'] == 'move':
            vel_mms = self._get_default_jog_vel(50.0)
            target_ui = np.array([step['x'], step['y'], step['z']])
            if self.ee_pos is not None:
                curr_ui = self.ee_pos - np.array([self.offsets['x'], self.offsets['y'], self.offsets['z']])
                dist = np.linalg.norm(target_ui - curr_ui)
                time_to_move = dist / max(vel_mms, 0.1)
                delay = time_to_move * 0.6
            else:
                delay = 0.05
            if delay < 0.01:
                delay = 0.01
            self._attempt_set_ee_pos(target_ui, from_ui=False, forced_vel=vel_mms * 60.0)
        elif step['type'] == 'delay':
            delay = step['dt']
        elif step['type'] == 'tool':
            self.on_tool_toggle(step['state'], sync_ui=True)
            delay = 0.1
        else:
            delay = 0.05
        self.hmi_step_index += 1
        self.hmi_playback_timer.start(int(delay * 1000))

    def on_generate_animation(self):
        anim_name = self.animation_combo.get()
        if not anim_name:
            return
        geom = {'e': self.e, 'f': self.f, 're': self.re, 'rf': self.rf, 'min_alpha_deg': self.min_upper_arm_angle,
                'max_alpha_deg': self.max_upper_arm_angle, 'collision_check': False}
        start = [0.0, 0.0, -280.0]
        if self.ee_pos is not None:
            start = (self.ee_pos - np.array([self.offsets['x'], self.offsets['y'], self.offsets['z']])).tolist()
        vel_mms = self._get_default_jog_vel(50.0)
        accel_val = self._get_default_accel()
        params = {'start_pos': start, 'pts': 200, 'turns': 5, 'vel': vel_mms, 'dt': 0.15}
        try:
            path = animation.generate_validated_animation(anim_name, params, geom)
        except Exception as e:
            tkmb.showerror("Generation Error", str(e))
            return
        if not path:
            return
        self.recorder_widget.clear()
        sb = [f"// Path: {anim_name}", "Begin()", "Tool(ON)"]
        for p in path:
            if p.get('type') == 'delay':
                sb.append(f"Delay({p.get('dt', 0.5)})")
            elif 'x' in p:
                sb.append(f"Move({p['x']:.2f}, {p['y']:.2f}, {p['z']:.2f}, {vel_mms:.1f}, {accel_val:.0f})")
        sb.append("Tool(OFF)")
        sb.append("End()")
        self.recorder_widget.insert_text("\n".join(sb))

    def on_calibrate_conveyor(self):
        self.calib_win = ctk.CTkToplevel(self)
        self.calib_win.title("Conveyor Speed Calibration")
        self.calib_win.geometry("650x600")
        self.calib_win.attributes("-topmost", True)
        ctk.CTkLabel(self.calib_win, text="Conveyor Speed Calibration", font=("Arial", 24, "bold")).pack(pady=15)
        manual_frm = ctk.CTkFrame(self.calib_win, fg_color="#2b2b2b", corner_radius=10)
        manual_frm.pack(fill="x", padx=30, pady=10)
        ctk.CTkLabel(manual_frm, text="Current / Manual Conveyor Speed (mm/s):", font=("Arial", 16, "bold")).pack(
            side="top", pady=(10, 5))
        spd_input_frm = ctk.CTkFrame(manual_frm, fg_color="transparent")
        spd_input_frm.pack(pady=5)
        self.txt_calib_speed = ctk.CTkEntry(spd_input_frm, width=150, height=40, font=("Consolas", 18),
                                            justify="center")
        self.txt_calib_speed.insert(0, f"{self.conveyor_speed_mm_s:.3f}")
        self.txt_calib_speed.pack(side="left", padx=10)

        def save_manual_speed():
            try:
                val = float(self.txt_calib_speed.get())
                self.conveyor_speed_mm_s = val
                self.save_config()
                self.lbl_calib_status.configure(text=f"Manual Speed Saved: {val:.3f} mm/s", text_color="#2ECC71",
                                                font=("Arial", 18, "bold"))
                tkmb.showinfo("Speed Saved", f"Conveyor speed manually updated to {val:.3f} mm/s")
            except ValueError:
                tkmb.showerror("Input Error", "Please enter a valid numeric speed value.")

        ctk.CTkButton(spd_input_frm, text="SAVE SPEED", font=("Arial", 14, "bold"), fg_color="#2ECC71", height=40,
                      command=save_manual_speed).pack(side="left", padx=10)
        self.lbl_calib_status = ctk.CTkLabel(self.calib_win, text="Step 1: Teach the object to follow",
                                             font=("Arial", 18))
        self.lbl_calib_status.pack(pady=10)
        self.calib_samples = []
        self.calib_tracking = False
        self.calib_start_x = 0
        self.calib_start_t = 0
        self.calib_recipe = {}
        self.calib_miss_count = 0
        if not self.vision_manager.running:
            self.vision_manager.toggle_camera()
        self.btn_teach_calib = ctk.CTkButton(self.calib_win, text="TEACH OBJECT", font=("Arial", 16, "bold"), height=50,
                                             command=self._start_calib_teach)
        self.btn_teach_calib.pack(pady=15)

    def _start_calib_teach(self):
        self.vision_manager.open_vision_teach_window(self.calib_recipe, self._start_calib_measurements)

    def _start_calib_measurements(self):
        if hasattr(self, 'btn_teach_calib') and self.btn_teach_calib.winfo_exists():
            self.btn_teach_calib.pack_forget()
        self.lbl_calib_status.configure(text="Step 2: Place the taught object on the conveyor...", text_color="#3498DB",
                                        font=("Arial", 18, "bold"))
        self.vision_manager.test_recipe = self.calib_recipe
        self.vision_manager.mode = "TEST"
        self._calib_conveyor_loop()

    def _calib_conveyor_loop(self):
        if not hasattr(self, 'calib_win') or not self.calib_win.winfo_exists():
            return
        if len(self.calib_samples) >= 5:
            avg_speed = float(sum(self.calib_samples) / 5.0)
            self.conveyor_speed_mm_s = float(avg_speed)
            self.save_config()
            if hasattr(self, 'txt_calib_speed') and self.txt_calib_speed.winfo_exists():
                self.txt_calib_speed.delete(0, "end")
                self.txt_calib_speed.insert(0, f"{avg_speed:.3f}")
            self.lbl_calib_status.configure(text=f"Calibration Complete!\nSpeed: {avg_speed:.2f} mm/s",
                                            text_color="#2ECC71", font=("Arial", 22, "bold"))
            ctk.CTkButton(self.calib_win, text="CLOSE", command=self.calib_win.destroy, font=("Arial", 16, "bold"),
                          height=50, fg_color="#E74C3C").pack(pady=20)
            return
        objs = self.vision_manager.detect_objects(self.calib_recipe)
        obj = objs[0] if objs else None
        if obj:
            self.calib_miss_count = 0
            curr_x = float(obj['x'])
            curr_t = time.time()
            if not self.calib_tracking:
                self.calib_tracking = True
                self.calib_start_x = curr_x
                self.calib_start_t = curr_t
                self.lbl_calib_status.configure(text=f"Tracking Sample {len(self.calib_samples) + 1}/5...",
                                                text_color="#F39C12")
            else:
                dx = curr_x - self.calib_start_x
                dt = curr_t - self.calib_start_t
                if abs(dx) > 30.0 and dt > 0.2:
                    speed = float(dx / dt)
                    self.calib_samples.append(speed)
                    self.calib_tracking = False
                    self.lbl_calib_status.configure(text=f"Sample {len(self.calib_samples)} recorded: {speed:.1f} mm/s",
                                                    text_color="#3498DB")
                    self.after(1500, self._calib_conveyor_loop)
                    return
        else:
            if self.calib_tracking:
                self.calib_miss_count += 1
                if self.calib_miss_count > 10:
                    self.calib_tracking = False
                    self.lbl_calib_status.configure(text="Lost object! Place another.", text_color="#E74C3C")
        self.after(50, self._calib_conveyor_loop)

    def on_hmi_start(self):
        if not (self.serial_manager.ser and self.serial_manager.ser.is_open):
            tkmb.showwarning("HMI", "Robot is not connected!")
            return
        recipe_name = self.cmb_hmi_recipe.get()
        if recipe_name not in self.recipes or 'objects' not in self.recipes[recipe_name] or not \
                self.recipes[recipe_name]['objects']:
            tkmb.showwarning("HMI", "Selected recipe is empty! Please Edit Recipe first.")
            return
        if self.vision_manager.transform_matrix is None:
            tkmb.showwarning("HMI", "Vision Matrix not loaded. Please Calibrate Camera first.")
            return
        # Stop any active TEACHING IDE playback and ensure tool starts OFF
        if hasattr(self, 'playback_timer'):
            self.playback_timer.stop()
        if hasattr(self, 'recorder_widget'):
            self.recorder_widget.clear_highlight()
            self.recorder_widget.reset_pause_button()
        self.on_tool_toggle(False, sync_ui=True)

        self.ignored_close_objects = []
        self.last_picked_belt_ref = None

        self.active_recipe = self.recipes[recipe_name]
        ready_pos = self.active_recipe.get('ready_pos', {'x': 0.0, 'y': 0.0, 'z': -250.0})
        rx = float(ready_pos.get('x', 0.0))
        ry = float(ready_pos.get('y', 0.0))
        rz = float(ready_pos.get('z', -250.0))
        self.hmi_move_steps = [
            {'type': 'move', 'x': rx, 'y': ry, 'z': rz}
        ]
        self.hmi_step_index = 0
        self.hmi_running = True
        self.hmi_state = "RETURNING"
        self.hmi_start_time = time.time()
        self.lbl_hmi_state.configure(text="STATE: MOVING TO READY POS", text_color="yellow")
        if hasattr(self, 'hardware_manager'):
            self.hardware_manager.set_green_light(True)
        if not self.vision_manager.running:
            self.vision_manager.toggle_camera()
        self.on_hmi_playback_timer()

    def on_hmi_stop(self):
        self.hmi_running = False
        self.hmi_state = "IDLE"
        self.hmi_move_steps = []
        self.hmi_step_index = 0
        self.ignored_close_objects = []
        self.last_picked_belt_ref = None
        if hasattr(self, 'hmi_playback_timer'):
            self.hmi_playback_timer.stop()
        self.lbl_hmi_state.configure(text="STATE: STOPPED", text_color="red")
        if hasattr(self, 'hardware_manager'):
            self.hardware_manager.set_green_light(False)
        # Turn off vacuum pump / tool immediately when stopping HMI process
        self.on_tool_toggle(False, sync_ui=True)
        self.on_stop()

    def update_hmi_stats(self):
        for widget in self.stats_container.winfo_children():
            widget.destroy()
        for obj_name, count in self.hmi_stats.items():
            frm = ctk.CTkFrame(self.stats_container, fg_color="#1a1a1a", corner_radius=5)
            frm.pack(fill="x", pady=5)
            ctk.CTkLabel(frm, text=f"{obj_name}:", font=("Arial", 18, "bold"), text_color="gray").pack(side="left",
                                                                                                       padx=10, pady=5)
            ctk.CTkLabel(frm, text=str(count), font=("Consolas", 24, "bold"), text_color="#3498DB").pack(side="right",
                                                                                                         padx=10,
                                                                                                         pady=5)

    def _find_best_hmi_candidate(self):
        now = time.time()
        all_entries = self._collect_all_hmi_detections()
        if not all_entries:
            self._update_ignored_close_objects([], now=now)
            return None

        # Update conveyor-synchronized ignore list and filter out any 2nd very close object
        self._update_ignored_close_objects(all_entries, now=now)

        best_candidate = None
        for entry in all_entries:
            if entry.get('ignored', False):
                continue

            obj = entry['obj']
            obj_cfg = entry['obj_cfg']
            detect_line_px = entry['detect_line_px']

            curr_ui = self.ee_pos - np.array([self.offsets['x'], self.offsets['y'], self.offsets['z']])
            z_hover = float(obj_cfg['z_hover'])
            vel_mms = self._get_default_jog_vel(100.0)
            dist_to_hover = np.linalg.norm(np.array([obj['x'], obj['y'], z_hover]) - curr_ui)
            time_to_hover = dist_to_hover / max(vel_mms, 0.1)
            z_pick = float(obj_cfg['z_pick'])
            dist_to_pick = abs(z_pick - z_hover)
            time_to_pick = dist_to_pick / max(vel_mms, 0.1)
            t_hover = time_to_hover + 0.15
            t_pick = t_hover + time_to_pick + 0.10
            pred_x_hover = float(obj['x'] + (self.conveyor_speed_mm_s * t_hover))
            pred_x_pick = float(obj['x'] + (self.conveyor_speed_mm_s * t_pick))
            cam_off_x = float(obj_cfg.get('cam_offset_x', 0.0))
            cam_off_y = float(obj_cfg.get('cam_offset_y', 0.0))
            target_x_hover = pred_x_hover + cam_off_x
            target_y_hover = obj['y'] + cam_off_y
            target_x_pick = pred_x_pick + cam_off_x
            target_y_pick = obj['y'] + cam_off_y
            pos_machine = np.array([target_x_pick, target_y_pick, z_pick])
            pos_machine[2] += self.offsets.get('tool', 0.0)
            pos_machine += np.array([self.offsets['x'], self.offsets['y'], self.offsets['z']])
            st, _, _, _ = delta_calcInverse(pos_machine[0], pos_machine[1], pos_machine[2],
                                            self.e, self.f, self.re, self.rf,
                                            self.min_upper_arm_angle, self.max_upper_arm_angle)
            if st == 0:
                trigger_x_mm = self._pixel_x_to_world_x(detect_line_px, ref_y_px=obj.get('cy'))
                is_ready = False
                if self.conveyor_speed_mm_s <= 0:
                    if obj['x'] <= trigger_x_mm:
                        is_ready = True
                else:
                    if obj['x'] >= trigger_x_mm:
                        is_ready = True
                if is_ready:
                    candidate = {
                        'obj': obj,
                        'obj_cfg': obj_cfg,
                        'target_x_hover': target_x_hover,
                        'target_y_hover': target_y_hover,
                        'target_x_pick': target_x_pick,
                        'target_y_pick': target_y_pick,
                        'z_hover': z_hover,
                        'z_pick': z_pick,
                        'px': float(obj_cfg['place_x']),
                        'py': float(obj_cfg['place_y']),
                        'pz': float(obj_cfg['place_z']),
                        'pump_delay': float(obj_cfg.get('pump_delay', 0.3)),
                        'drop_delay': float(obj_cfg.get('drop_delay', 0.2)),
                        '_all_entries': all_entries,
                        '_detected_t': now
                    }
                    if best_candidate is None:
                        best_candidate = candidate
                    else:
                        if self.conveyor_speed_mm_s <= 0:
                            if obj['x'] < best_candidate['obj']['x']:
                                best_candidate = candidate
                        else:
                            if obj['x'] > best_candidate['obj']['x']:
                                best_candidate = candidate
        return best_candidate

    def _execute_pick(self, candidate):
        # Register the picked object and mark any very close second object on the conveyor to be ignored
        self._register_close_objects_for_pick(
            candidate,
            all_entries=candidate.get('_all_entries'),
            now=candidate.get('_detected_t')
        )

        self.hmi_state = "PICKING"
        obj_cfg = candidate['obj_cfg']
        self.current_pick_obj = obj_cfg['name']
        self.lbl_hmi_state.configure(text=f"STATE: PICKING {obj_cfg['name'].upper()}", text_color="#3498DB")
        # Build trajectory in dedicated HMI step list (isolated from TEACHING IDE)
        self.hmi_move_steps = [
            {'type': 'move', 'x': candidate['target_x_hover'], 'y': candidate['target_y_hover'],
             'z': candidate['z_hover']},
            {'type': 'move', 'x': candidate['target_x_pick'], 'y': candidate['target_y_pick'],
             'z': candidate['z_pick']},
            {'type': 'tool', 'state': 1},
            {'type': 'delay', 'dt': candidate['pump_delay']},
            {'type': 'move', 'x': candidate['target_x_pick'], 'y': candidate['target_y_pick'],
             'z': candidate['z_hover']},
            {'type': 'move', 'x': candidate['px'], 'y': candidate['py'], 'z': candidate['z_hover']},
            {'type': 'move', 'x': candidate['px'], 'y': candidate['py'], 'z': candidate['pz']},
            {'type': 'delay', 'dt': candidate['drop_delay']},
            {'type': 'tool', 'state': 0},
            {'type': 'delay', 'dt': 0.3},
            # Move EE slightly up to hover Z to avoid collisions when returning to ready pos
            {'type': 'move', 'x': candidate['px'], 'y': candidate['py'], 'z': candidate['z_hover']}
        ]
        self.hmi_step_index = 0
        self.on_hmi_playback_timer()

    def hmi_process_loop(self):
        if not self.hmi_running:
            return
        if self.hmi_state == "SCANNING":
            elapsed_min = int((time.time() - self.hmi_start_time) / 60)
            self.lbl_hmi_state.configure(text=f"STATE: SCANNING | Runtime: {elapsed_min}m")
            best_candidate = self._find_best_hmi_candidate()
            if best_candidate:
                self._execute_pick(best_candidate)
                return
            self.after(50, self.hmi_process_loop)

    def on_hmi_edit_recipe(self):
        recipe_name = self.cmb_hmi_recipe.get()
        if recipe_name not in self.recipes:
            self.recipes[recipe_name] = {}
        working_recipe = copy.deepcopy(self.recipes[recipe_name])
        if 'objects' not in working_recipe:
            working_recipe['objects'] = []
        if 'ready_pos' not in working_recipe:
            working_recipe['ready_pos'] = {'x': 0.0, 'y': 0.0, 'z': -250.0}
        wizard_win = ctk.CTkToplevel(self)
        wizard_win.title("Recipe Wizard")
        wizard_win.geometry("900x700")
        wizard_win.attributes("-topmost", True)
        container = ctk.CTkFrame(wizard_win)
        container.pack(fill="both", expand=True)
        current_obj_index = [0]
        num_objects = [1]

        def clear_container():
            for widget in container.winfo_children():
                widget.destroy()

        def step_1_setup():
            clear_container()
            ctk.CTkLabel(container, text="STEP 1: Recipe Settings", font=("Arial", 28, "bold")).pack(pady=40)
            ctk.CTkLabel(container, text="Mode:", font=("Arial", 20)).pack(pady=10)
            ctk.CTkOptionMenu(container, values=["Pick & Place"], state="disabled", width=300, height=50,
                              font=("Arial", 18)).pack(pady=10)
            ctk.CTkLabel(container, text="Number of Objects (1-3):", font=("Arial", 20)).pack(pady=(30, 10))
            cmb_num = ctk.CTkOptionMenu(container, values=["1", "2", "3"], width=300, height=50, font=("Arial", 18))
            cmb_num.set(str(len(working_recipe['objects']) if working_recipe['objects'] else 1))
            cmb_num.pack(pady=10)

            def next_step():
                num_objects[0] = int(cmb_num.get())
                working_recipe['objects'] = working_recipe['objects'][:num_objects[0]]
                step_2_object()

            ctk.CTkButton(container, text="NEXT  ", font=("Arial", 20, "bold"), height=60, width=300,
                          command=next_step).pack(pady=60)

        def step_2_object():
            if current_obj_index[0] >= num_objects[0]:
                step_3_finish()
                return
            clear_container()
            idx = current_obj_index[0]
            existing_obj = working_recipe['objects'][idx] if idx < len(working_recipe['objects']) else {}
            ctk.CTkLabel(container, text=f"STEP 2: Configure Object {idx + 1} of {num_objects[0]}",
                         font=("Arial", 24, "bold"), text_color="#F39C12").pack(pady=10)
            scroll = ctk.CTkScrollableFrame(container)
            try:
                scroll._scrollbar.configure(width=30)
            except Exception:
                pass
            scroll.pack(fill="both", expand=True, padx=20, pady=10)
            ctk.CTkLabel(scroll, text="Object Name:", font=("Arial", 18, "bold")).pack(pady=5)
            txt_name = ctk.CTkEntry(scroll, width=300, height=45, font=("Consolas", 18))
            txt_name.insert(0, existing_obj.get('name', f"Object_{idx + 1}"))
            txt_name.pack(pady=5)
            teach_frm = ctk.CTkFrame(scroll, fg_color="#2b2b2b", corner_radius=10)
            teach_frm.pack(fill="x", pady=20, padx=20)
            ctk.CTkLabel(teach_frm, text="Live Feed (Click object to teach color)", font=("Arial", 18, "bold")).pack(
                pady=5)
            lbl_img = ctk.CTkLabel(teach_frm, text="Starting Feed...", width=400, height=300, fg_color="black")
            lbl_img.pack(pady=10)
            # --- Detection Start Line Control (Inside ROI) ---
            rx1_bound, ry1_bound, rx2_bound, ry2_bound = self._get_roi_bounds()
            default_line_px = self._get_default_detect_line_px()
            if 'detect_line_px' in existing_obj:
                init_line_px = int(existing_obj['detect_line_px'])
            elif idx > 0 and len(working_recipe['objects']) > 0 and 'detect_line_px' in working_recipe['objects'][0]:
                init_line_px = int(working_recipe['objects'][0]['detect_line_px'])
            else:
                init_line_px = default_line_px
            init_line_px = max(rx1_bound, min(rx2_bound, init_line_px))
            detect_line_state = [init_line_px]
            line_ctrl_frm = ctk.CTkFrame(teach_frm, fg_color="#1f1f1f", corner_radius=8)
            line_ctrl_frm.pack(fill="x", padx=15, pady=(5, 10))
            line_hdr_frm = ctk.CTkFrame(line_ctrl_frm, fg_color="transparent")
            line_hdr_frm.pack(fill="x", padx=10, pady=(8, 2))
            ctk.CTkLabel(line_hdr_frm, text="Pick & Place Detection Start Line (ROI):",
                         font=("Arial", 16, "bold"), text_color="#FF4500").pack(side="left")
            init_world_x = self._pixel_x_to_world_x(init_line_px)
            lbl_line_info = ctk.CTkLabel(
                line_hdr_frm,
                text=f"{init_line_px} px  ({init_world_x:.1f} mm)",
                font=("Consolas", 18, "bold"),
                text_color="#2ECC71"
            )
            lbl_line_info.pack(side="right")
            line_adj_frm = ctk.CTkFrame(line_ctrl_frm, fg_color="transparent")
            line_adj_frm.pack(fill="x", padx=10, pady=(2, 8))

            def sync_detect_line(new_px):
                clamped_px = max(rx1_bound, min(rx2_bound, int(round(float(new_px)))))
                detect_line_state[0] = clamped_px
                slider_detect_line.set(clamped_px)
                wx_mm = self._pixel_x_to_world_x(clamped_px)
                lbl_line_info.configure(text=f"{clamped_px} px  ({wx_mm:.1f} mm)")

            btn_line_minus5 = ctk.CTkButton(
                line_adj_frm, text="-5", width=50, height=40, font=("Arial", 16, "bold"),
                command=lambda: sync_detect_line(detect_line_state[0] - 5)
            )
            btn_line_minus5.pack(side="left", padx=3)
            btn_line_minus1 = ctk.CTkButton(
                line_adj_frm, text="-1", width=50, height=40, font=("Arial", 16, "bold"),
                command=lambda: sync_detect_line(detect_line_state[0] - 1)
            )
            btn_line_minus1.pack(side="left", padx=3)
            slider_detect_line = ctk.CTkSlider(
                line_adj_frm,
                from_=rx1_bound,
                to=max(rx1_bound + 1, rx2_bound),
                height=40,
                button_length=40,
                progress_color="#FF4500",
                button_color="#FF4500",
                button_hover_color="#E03E00"
            )
            slider_detect_line.set(init_line_px)
            slider_detect_line.configure(command=lambda v: sync_detect_line(v))
            slider_detect_line.pack(side="left", fill="x", expand=True, padx=10)
            btn_line_plus1 = ctk.CTkButton(
                line_adj_frm, text="+1", width=50, height=40, font=("Arial", 16, "bold"),
                command=lambda: sync_detect_line(detect_line_state[0] + 1)
            )
            btn_line_plus1.pack(side="left", padx=3)
            btn_line_plus5 = ctk.CTkButton(
                line_adj_frm, text="+5", width=50, height=40, font=("Arial", 16, "bold"),
                command=lambda: sync_detect_line(detect_line_state[0] + 5)
            )
            btn_line_plus5.pack(side="left", padx=3)
            sliders = {}

            def add_slider(parent, name, key, from_, to_, default):
                f = ctk.CTkFrame(parent, fg_color="transparent")
                f.pack(fill="x", padx=10, pady=5)
                ctk.CTkLabel(f, text=name, width=80, anchor="w", font=("Arial", 16, "bold")).pack(side="left")
                val_lbl = ctk.CTkLabel(f, text=str(int(default)), width=50, font=("Consolas", 18))
                val_lbl.pack(side="right")
                s = ctk.CTkSlider(f, from_=from_, to=to_, height=40, button_length=40)
                s.set(default)
                s.configure(command=lambda v, l=val_lbl: l.configure(text=str(int(v))))
                s.pack(side="left", fill="x", expand=True, padx=15)
                sliders[key] = s

            slider_frm = ctk.CTkFrame(teach_frm, fg_color="transparent")
            slider_frm.pack(fill="x", pady=10)
            add_slider(slider_frm, "H Min", "h_min", 0, 179, existing_obj.get("h_min", 0))
            add_slider(slider_frm, "H Max", "h_max", 0, 179, existing_obj.get("h_max", 10))
            add_slider(slider_frm, "S Min", "s_min", 0, 255, existing_obj.get("s_min", 100))
            add_slider(slider_frm, "S Max", "s_max", 0, 255, existing_obj.get("s_max", 255))
            add_slider(slider_frm, "V Min", "v_min", 0, 255, existing_obj.get("v_min", 100))
            add_slider(slider_frm, "V Max", "v_max", 0, 255, existing_obj.get("v_max", 255))
            add_slider(slider_frm, "Area", "min_area", 0, 5000, existing_obj.get("min_area", 500))
            feed_running = [True]

            def on_feed_click(event):
                if self.vision_manager.last_frame_bgr is None:
                    return
                u = int(event.x * (640.0 / 400.0))
                v = int(event.y * (480.0 / 300.0))
                _, _, _, bounds = self.vision_manager.sample_hsv_region(self.vision_manager.last_frame_bgr, u, v)
                sliders['h_min'].set(bounds['h_min'])
                sliders['h_max'].set(bounds['h_max'])
                sliders['s_min'].set(bounds['s_min'])
                sliders['s_max'].set(bounds['s_max'])
                sliders['v_min'].set(bounds['v_min'])
                sliders['v_max'].set(bounds['v_max'])
                for k, sl in sliders.items():
                    sl._command(sl.get())

            lbl_img.bind("<Button-1>", on_feed_click)

            def update_preview():
                if not feed_running[0] or not wizard_win.winfo_exists():
                    return
                if self.vision_manager.last_frame_bgr is not None:
                    img = self.vision_manager.last_frame_bgr.copy()
                    lower = np.array(
                        [int(sliders['h_min'].get()), int(sliders['s_min'].get()), int(sliders['v_min'].get())])
                    upper = np.array(
                        [int(sliders['h_max'].get()), int(sliders['s_max'].get()), int(sliders['v_max'].get())])
                    mask = self.vision_manager.build_filtered_mask(img, lower, upper)
                    rx1, ry1, rx2, ry2 = self._get_roi_bounds()
                    if [rx1, ry1, rx2, ry2] != [0, 0, 640, 480]:
                        cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (0, 255, 255), 2)
                    # Draw vertical detection start line from top to bottom of the ROI area
                    line_x = max(rx1, min(rx2, int(detect_line_state[0])))
                    cv2.line(img, (line_x, ry1), (line_x, ry2), (0, 69, 255), 3)
                    lbl_txt = f"START ({line_x}px)"
                    tx = line_x + 6 if line_x + 135 < rx2 else max(rx1 + 4, line_x - 130)
                    ty = max(20, ry1 + 22)
                    cv2.putText(img, lbl_txt, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                    cv2.putText(img, lbl_txt, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 255), 2)
                    min_area_val = int(sliders['min_area'].get())
                    regions = self.vision_manager.extract_separated_regions(img, mask, min_area_val)
                    for reg in regions:
                        cv2.drawContours(img, [reg['hull']], -1, (0, 255, 0), 2)
                        cv2.drawContours(img, [reg['box']], 0, (255, 255, 0), 1)
                        cv2.circle(img, (reg['cx'], reg['cy']), 4, (0, 0, 255), -1)
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    pil_img = Image.fromarray(img_rgb).resize((400, 300), Image.Resampling.LANCZOS)
                    ctk_img = ctk.CTkImage(light_image=pil_img, dark_image=pil_img, size=(400, 300))
                    lbl_img.configure(image=ctk_img, text="")
                self.after(50, update_preview)

            update_preview()
            ctk.CTkLabel(scroll, text="Z Heights (mm):", font=("Arial", 18, "bold")).pack(pady=10)
            h_frm = ctk.CTkFrame(scroll, fg_color="transparent")
            h_frm.pack(fill="x", pady=5)
            ctk.CTkLabel(h_frm, text="Hover Z:", font=("Arial", 16)).pack(side="left", padx=10)
            txt_hz = ctk.CTkEntry(h_frm, width=120, height=45, font=("Consolas", 18))
            txt_hz.insert(0, str(existing_obj.get('z_hover', -250.0)))
            txt_hz.pack(side="left", padx=10)
            ctk.CTkLabel(h_frm, text="Pick Z:", font=("Arial", 16)).pack(side="left", padx=10)
            txt_pz = ctk.CTkEntry(h_frm, width=120, height=45, font=("Consolas", 18))
            txt_pz.insert(0, str(existing_obj.get('z_pick', -300.0)))
            txt_pz.pack(side="left", padx=10)
            ctk.CTkLabel(scroll, text="Camera Offset (mm):", font=("Arial", 18, "bold")).pack(pady=(20, 10))
            cam_frm = ctk.CTkFrame(scroll, fg_color="transparent")
            cam_frm.pack(fill="x", pady=5)
            ctk.CTkLabel(cam_frm, text="Offset X:", font=("Arial", 16)).pack(side="left", padx=10)
            txt_cam_x = ctk.CTkEntry(cam_frm, width=120, height=45, font=("Consolas", 18))
            txt_cam_x.insert(0, str(existing_obj.get('cam_offset_x', 0.0)))
            txt_cam_x.pack(side="left", padx=10)
            ctk.CTkLabel(cam_frm, text="Offset Y:", font=("Arial", 16)).pack(side="left", padx=10)
            txt_cam_y = ctk.CTkEntry(cam_frm, width=120, height=45, font=("Consolas", 18))
            txt_cam_y.insert(0, str(existing_obj.get('cam_offset_y', 0.0)))
            txt_cam_y.pack(side="left", padx=10)
            ctk.CTkLabel(scroll, text="Vacuum Pump Delay (s):", font=("Arial", 18, "bold")).pack(pady=(20, 10))
            pump_frm = ctk.CTkFrame(scroll, fg_color="transparent")
            pump_frm.pack(fill="x", pady=5)
            ctk.CTkLabel(pump_frm, text="Delay Time:", font=("Arial", 16)).pack(side="left", padx=10)

            def adjust_delay(delta):
                try:
                    val = float(txt_pump.get())
                    val = max(0.0, val + delta)
                    txt_pump.delete(0, "end")
                    txt_pump.insert(0, f"{val:.2f}")
                except ValueError:
                    pass

            btn_minus = ctk.CTkButton(pump_frm, text="-", width=45, height=45, font=("Arial", 20, "bold"),
                                      command=lambda: adjust_delay(-0.1))
            btn_minus.pack(side="left", padx=5)
            txt_pump = ctk.CTkEntry(pump_frm, width=100, height=45, font=("Consolas", 18))
            txt_pump.insert(0, str(existing_obj.get('pump_delay', 0.3)))
            txt_pump.pack(side="left", padx=5)
            btn_plus = ctk.CTkButton(pump_frm, text="+", width=45, height=45, font=("Arial", 20, "bold"),
                                     command=lambda: adjust_delay(0.1))
            btn_plus.pack(side="left", padx=5)
            # --- Drop Delay Configuration ---
            ctk.CTkLabel(scroll, text="Vacuum Drop Delay (s):", font=("Arial", 18, "bold")).pack(pady=(20, 10))
            drop_frm = ctk.CTkFrame(scroll, fg_color="transparent")
            drop_frm.pack(fill="x", pady=5)
            ctk.CTkLabel(drop_frm, text="Delay Time:", font=("Arial", 16)).pack(side="left", padx=10)

            def adjust_drop_delay(delta):
                try:
                    val = float(txt_drop.get())
                    val = max(0.0, val + delta)
                    txt_drop.delete(0, "end")
                    txt_drop.insert(0, f"{val:.2f}")
                except ValueError:
                    pass

            btn_drop_minus = ctk.CTkButton(drop_frm, text="-", width=45, height=45, font=("Arial", 20, "bold"),
                                           command=lambda: adjust_drop_delay(-0.1))
            btn_drop_minus.pack(side="left", padx=5)
            txt_drop = ctk.CTkEntry(drop_frm, width=100, height=45, font=("Consolas", 18))
            txt_drop.insert(0, str(existing_obj.get('drop_delay', 0.2)))
            txt_drop.pack(side="left", padx=5)
            btn_drop_plus = ctk.CTkButton(drop_frm, text="+", width=45, height=45, font=("Arial", 20, "bold"),
                                          command=lambda: adjust_drop_delay(0.1))
            btn_drop_plus.pack(side="left", padx=5)
            ctk.CTkLabel(scroll, text="Drop Box Location (mm):", font=("Arial", 18, "bold")).pack(pady=(20, 10))
            c_frm = ctk.CTkFrame(scroll, fg_color="transparent")
            c_frm.pack(fill="x", pady=5)
            entries_coords = {}
            for axis in ['X', 'Y', 'Z']:
                ctk.CTkLabel(c_frm, text=f"{axis}:", font=("Arial", 16)).pack(side="left", padx=5)
                e = ctk.CTkEntry(c_frm, width=100, height=45, font=("Consolas", 18))
                e.insert(0, str(existing_obj.get(f'place_{axis.lower()}', 0.0)))
                e.pack(side="left", padx=5)
                entries_coords[axis.lower()] = e

            def next_obj():
                feed_running[0] = False
                obj_data = {
                    'name': txt_name.get(),
                    'h_min': int(sliders['h_min'].get()), 'h_max': int(sliders['h_max'].get()),
                    's_min': int(sliders['s_min'].get()), 's_max': int(sliders['s_max'].get()),
                    'v_min': int(sliders['v_min'].get()), 'v_max': int(sliders['v_max'].get()),
                    'min_area': int(sliders['min_area'].get()),
                    'detect_line_px': int(detect_line_state[0]),
                    'z_hover': float(txt_hz.get()), 'z_pick': float(txt_pz.get()),
                    'cam_offset_x': float(txt_cam_x.get()), 'cam_offset_y': float(txt_cam_y.get()),
                    'pump_delay': float(txt_pump.get()),
                    'drop_delay': float(txt_drop.get()),
                    'place_x': float(entries_coords['x'].get()),
                    'place_y': float(entries_coords['y'].get()),
                    'place_z': float(entries_coords['z'].get()),
                }
                if idx < len(working_recipe['objects']):
                    working_recipe['objects'][idx] = obj_data
                else:
                    working_recipe['objects'].append(obj_data)
                current_obj_index[0] += 1
                step_2_object()

            btn_txt = "NEXT OBJECT  " if current_obj_index[0] < num_objects[0] - 1 else "FINISH SETTINGS  "
            ctk.CTkButton(scroll, text=btn_txt, font=("Arial", 20, "bold"), height=60, command=next_obj).pack(pady=30)

        def step_3_finish():
            clear_container()
            ctk.CTkLabel(container, text="STEP 3: Global Settings", font=("Arial", 24, "bold")).pack(pady=40)
            ctk.CTkLabel(container, text="Robot Ready Position (mm)", font=("Arial", 20, "bold")).pack(pady=20)
            existing_rp = working_recipe['ready_pos']
            c_frm = ctk.CTkFrame(container, fg_color="transparent")
            c_frm.pack(pady=10)
            entries_coords = {}
            for axis in ['X', 'Y', 'Z']:
                ctk.CTkLabel(c_frm, text=f"{axis}:", font=("Arial", 18)).pack(side="left", padx=10)
                e = ctk.CTkEntry(c_frm, width=120, height=50, font=("Consolas", 20))
                e.insert(0, str(existing_rp.get(axis.lower(), 0.0)))
                e.pack(side="left", padx=10)
                entries_coords[axis.lower()] = e

            def save_all():
                working_recipe['ready_pos'] = {
                    'x': float(entries_coords['x'].get()),
                    'y': float(entries_coords['y'].get()),
                    'z': float(entries_coords['z'].get())
                }
                self.recipes[recipe_name] = working_recipe
                self.save_config()
                wizard_win.destroy()
                tkmb.showinfo("Recipe Saved", f"Saved configuration for {recipe_name}")

            ctk.CTkButton(container, text="SAVE RECIPE", font=("Arial", 22, "bold"), fg_color="#2ECC71", height=70,
                          width=350, command=save_all).pack(pady=60)

        if not self.vision_manager.running:
            self.vision_manager.toggle_camera()
        step_1_setup()

    def compute_slider_limits_from_geometry(self):
        pass

    def on_slider_changed(self, v):
        pass

    def on_debounced_slider_update(self):
        pass

    def on_generate_workspace(self):
        pass

    def on_open_realtime_graph(self):
        pass

    def on_steps_info_clicked(self):
        pass

    def on_plane_clicked_coord(self, x, y):
        pass