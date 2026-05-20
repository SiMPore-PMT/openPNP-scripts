# Jython 2.7 / OpenPnP
# Single-stage, two-circle top-camera calibration diagnostic.

import json
import math
import os
import shutil
import sys
import time
import traceback

from java.io import File
from java.lang import Thread, Throwable
from javax.imageio import ImageIO
from org.openpnp.model import Configuration, Location, LengthUnit
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import VisionUtils, OpenCvUtils

try:
    SCRIPT_DIR_FOR_IMPORT = os.path.dirname(os.path.abspath(__file__))
    if SCRIPT_DIR_FOR_IMPORT not in sys.path:
        sys.path.append(SCRIPT_DIR_FOR_IMPORT)
except:
    pass

import Events.Homing.OLD.pick_cameraAlign_Calibration as base

TAG = "SingleCircleTipCalibration"
CONFIG_FILE = "pick_cameraAlign_SingleCircleTipCalibration_config.json"
RESULT_FILE = "pick_cameraAlign_SingleCircleTipCalibration_last_result.json"
LAST_CONFIRM_FILE = "pick_cameraAlign_SingleCircleTipCalibration_last_confirm.json"
LAST_IMAGE_FILE = "pick_cameraAlign_SingleCircleTipCalibration_last_capture.png"
LAST_OVERLAY_FILE = "pick_cameraAlign_SingleCircleTipCalibration_last_overlay.json"
LAST_BOTTOM_IMAGE_FILE = "pick_cameraAlign_SingleCircleTipCalibration_last_bottom_square_capture.png"
LAST_BOTTOM_OVERLAY_FILE = "pick_cameraAlign_SingleCircleTipCalibration_last_bottom_square_overlay.json"

_progress_callback = None
_cancel_token = None
_last_vision_image_path = None
_last_capture_image_size = None
_last_pipeline_working_image_size = None

PIPELINE_ATTEMPTS = 3
SETTLE_MS = 250

LEGACY_CONFIG_KEYS = {
    "single_circle_algorithm": True,
    "single_circle_pair_angles_deg": True,
    "single_circle_pair_apply_xy_only": True,
    "single_circle_pair_disable_bottom_square": True,
    "single_circle_pair_max_delta_mm": True,
    "single_circle_pair_max_step_mm": True,
    "single_circle_pair_min_pairs": True,
    "single_circle_pair_place_delta_deg": True,
    "single_circle_pair_write_head_offsets": True
}


DEFAULT_CONFIG = {
    "part_id_or_name": "PickCameraAlignDie",
    "nozzle_name": "Pick Head",
    "camera_name": "",

    "die_storage_location_xyz": {"x": 10.0, "y": 10.0, "z": 2.0, "rotation": 0.0},
    "cal_work_location_xyz": {"x": 40.0, "y": 40.0, "z": 2.0, "rotation": 0.0},
    "safe_travel_z": 25.0,

    "single_circle_result_stage_name": "results",
    "single_circle_expected_result_count": 2,
    "single_circle_model_mode": "openpnp_model_camera_offset",
    "single_circle_apply_model_sign": -1.0,
    "single_circle_fit_model_type": "openpnp_pairwise_180_offset",
    "single_circle_max_model_center_mm": 1.5,
    "single_circle_max_model_radius_mm": 1.5,
    "single_circle_max_model_rms_mm": 0.12,
    "single_circle_max_model_peak_mm": 0.30,
    "single_circle_block_active_correction_on_model_fail": True,
    "single_circle_auto_select_model_sign": False,
    "single_circle_head_delta_sign_override": 1.0,
    "single_circle_enable_runtime_runout_compensation": False,
    "single_circle_center_expected_size_px": 182.0,
    "single_circle_center_min_size_px": 150.0,
    "single_circle_center_max_size_px": 215.0,
    "single_circle_orientation_expected_size_px": 75.0,
    "single_circle_orientation_min_size_px": 55.0,
    "single_circle_orientation_max_size_px": 95.0,
    "single_circle_min_size_ratio": 1.75,
    "single_circle_require_size_data": True,

    "angle_start_deg": 0.0,
    "angle_stop_deg": 360.0,
    "angle_subdivisions": 8,
    "allow_misdetections": 0,
    "openpnp_pairwise_angles_deg": [],
    "openpnp_pairwise_max_total_walk_mm": 3.0,
    "openpnp_pairwise_abort_on_walk_exceeded": True,
    "openpnp_pairwise_nominal_reacquire_guard_mm": 0.25,

    "single_circle_rotation_mode": "continuous",
    "single_circle_allow_continuous_rotation": True,
    "single_circle_max_rotation_step_deg": 9999.0,
    "single_circle_use_shortest_rotation_path": False,
    "single_circle_normalize_motion_rotation": False,
    "single_circle_unwind_after_run": False,
    "single_circle_cleanup_rotation_mode": "leave_current",
    "single_circle_initial_pick_rotation_deg": 0.0,
    "single_circle_use_current_rotation_for_pick": False,

    "use_high_level_nozzle_pick": True,
    "use_high_level_nozzle_place": False,
    "single_circle_call_nozzle_pick_after_manual_vacuum": True,
    "inherit_high_level_pick_from_four_circle_config": True,
    "force_vacuum_actuator_name": "",
    "force_blowoff_actuator_name": "",
    "vacuum_settle_ms": 150,
    "blowoff_pulse_ms": 120,
    "enable_blowoff_on_place": False,
    "require_vacuum_actuator_on_place": True,

    "offset_threshold_mm": 2.0,
    "reject_measurements_outside_threshold": False,
    "fit_even_if_residual_high": True,
    "fit_residual_warning_only": True,

    "measurement_method": "camera_only",
    "nozzle_target_method": "tool_aware",

    "return_to_storage_use_computed_offset": True,
    "return_to_storage_apply_offset_to_pick": True,
    "return_to_storage_apply_offset_to_place": False,
    "allow_static_delta_as_same_run_pick_jog": False,

    "return_orientation_mode": "configured",
    "return_allowed_orientations_deg": [90.0, 180.0, 270.0],
    "desired_die_storage_orientation_deg": 0.0,
    "return_held_object_rotation_sign": -1.0,
    "return_orientation_marker_enabled": True,

    "single_circle_preview_include_static_delta": True,
    "single_circle_preview_include_residual_angle_vector": True,
    "single_circle_confirm_descend_to_surface": True,
    "single_circle_confirm_surface_clearance_mm": 0.0,
    "single_circle_confirm_use_orientation_rotation": True,
    "single_circle_pick_use_orientation_rotation": True,
    "single_circle_pick_orientation_rotation_mode": "raw_marker_angle",
    "single_circle_verify_static_xy_after_fit": True,
    "single_circle_physical_ab_verification": True,
    "single_circle_verify_angles_deg": [0.0, 90.0, 180.0, 270.0],
    "single_circle_verify_old_new_preview": True,
    "single_circle_min_visible_preview_shift_mm": 0.02,
    "single_circle_orientation_zero_offset_deg": 0.0,
    "single_circle_orientation_sign": 1.0,
    "single_circle_calibration_acquire_from_nominal_xy": True,
    "single_circle_update_current_die_loc_after_measure": False,
    "calibration_pick_rotation_policy": "preserve_current_nozzle_rotation",
    "return_pick_rotation_policy": "squared_marker_to_desired_storage",
    "return_place_rotation_policy": "squared_marker_to_desired_storage",
    "confirm_rotation_policy": "preserve_current_nozzle_rotation",
    "confirm_use_nozzle_target_frame": True,
    "return_pick_include_residual_angle_vector": False,
    "confirm_include_residual_angle_vector": False,
    "calibration_fit_include_residual_angle_vector": True,
    "return_verify_orientation_after_place": True,
    "return_orientation_tolerance_deg": 10.0,

    "top_camera_focus_plane_source": "object_surface_z",
    "top_camera_measurement_z_mm": None,
    "top_camera_use_expected_loc_z_for_pixel_transform": True,
    "top_camera_log_focus_plane": True,

    "nozzle_pick_z_source": "part_top_surface_z",
    "nozzle_place_z_source": "part_top_surface_z",
    "nozzle_pick_z_manual_mm": None,
    "nozzle_place_z_manual_mm": None,
    "log_nozzle_z_plan": True,

    "run_bottom_camera_nozzle_square_after_fit": True,
    "bottom_square_stage_name": "squareResults",
    "bottom_square_stage_fallbacks": ["results", "orient", "preResults"],
    "bottom_square_angle_sign": 1.0,
    "square_bias_apply_sign": 1.0,

    "bottom_square_set_global_rotation_offset": True,
    "bottom_square_apply_motion_correction": True,
    "bottom_square_verify_after_apply": True,
    "bottom_square_apply_mode": "script_local_rotation_bias",

    "bottom_square_settle_ms": 1000,
    "bottom_square_pipeline_attempts": 3,

    "bottom_square_nominal_z_source": "bottom_camera_location",
    "bottom_square_safe_z": 28.13,

    "bottom_square_x_offsets_mm": [-0.50, -0.25, 0.0, 0.25, 0.50],
    "bottom_square_y_offsets_mm": [-0.50, -0.25, 0.0, 0.25, 0.50],
    "bottom_square_verify_offsets_mm": [-0.30, 0.0, 0.30],

    "bottom_square_max_angle_abs_deg": 90.0,
    "bottom_square_max_angle_range_deg": 2.0,
    "bottom_square_max_post_apply_abs_deg": 1.0,
    "bottom_square_treat_near_45_as_ambiguous": False,
    "bottom_square_reject_ambiguous_45": True,
    "bottom_square_reject_near_square_orientation": True,
    "bottom_square_min_orientation_aspect_ratio": 1.15,
    "bottom_square_reject_near_45_as_ambiguous": True,
    "bottom_square_near_45_tolerance_deg": 3.0,
    "bottom_square_max_aspect_ratio": 1.20,

    "bottom_square_use_median_angle": True,
    "bottom_square_reject_if_position_dependent": False,
    "bottom_square_warning_only": True,

    "return_to_die_after_bottom_square": True,
    "return_to_storage_after_bottom_square": True,
    "return_pick_use_pre_square_correction_after_bottom_square": False,
    "post_square_recompute_static_pick_delta": True,
    "post_square_return_pick_delta_mode": "fresh_center_only",
    "allow_confirm_with_stale_pre_square_correction": False,
    "post_square_physical_pick_place_verify": False,
    "run_post_square_single_angle_recenter": False,
    "post_square_recenter_angle_deg": 0.0,

    "allow_machine_writes": False,
    "allow_apply_without_verification": False,
    "write_machine_xml_backup": True,
    "machine_xml_path": None,
    "max_apply_mm": 1.5,

    "confirm_allow_diagnostic_preview": True,

    "spiral_search": {
        "enabled": True,
        "start_radius_mm": 0.15,
        "radius_step_mm": 0.15,
        "max_radius_mm": 1.20,
        "angle_step_deg": 45.0,
        "settle_ms": 150
    },

    "overlay_style": {
        "square_width": 2.0,
        "circle_width": 2.0,
        "circle_radius": 14.0,
        "cross_size": 18.0,
        "key_scale": 0.8,
        "actual_color": "#24B35F",
        "expected_color": "#FFB020",
        "circle_color": "#2A8CFF",
        "orientation_color": "#F04C4C"
    }
}


class SingleCircleDetectionError(Exception):
    pass


def log(tag, msg):
    line = "[%s][%s] %s" % (TAG, tag, str(msg))
    print line
    try:
        if _progress_callback is not None:
            _progress_callback(tag, str(msg), line)
    except:
        pass


def script_dir():
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except:
        try:
            return os.path.join(scripting.getScriptsDirectory().toString(), "Events", "Homing")
        except:
            return os.getcwd()


def config_path():
    return os.path.join(script_dir(), CONFIG_FILE)


def result_path():
    return os.path.join(script_dir(), RESULT_FILE)


def last_confirm_path():
    return os.path.join(script_dir(), LAST_CONFIRM_FILE)


def last_image_path():
    return os.path.join(script_dir(), LAST_IMAGE_FILE)


def last_overlay_path():
    return os.path.join(script_dir(), LAST_OVERLAY_FILE)


def last_bottom_image_path():
    return os.path.join(script_dir(), LAST_BOTTOM_IMAGE_FILE)


def last_bottom_overlay_path():
    return os.path.join(script_dir(), LAST_BOTTOM_OVERLAY_FILE)


def bool_value(value):
    return base.bool_value(value)


def finite_number(value):
    return base.finite_number(value)


def deep_update(base_cfg, updates):
    out = dict(base_cfg)
    for key in updates.keys():
        if isinstance(updates[key], dict) and isinstance(out.get(key), dict):
            child = dict(out[key])
            child.update(updates[key])
            out[key] = child
        else:
            out[key] = updates[key]
    return out


def require_key(cfg, key):
    if key not in cfg:
        raise Exception("Missing required config key: %s" % key)


def validate_no_unknown_keys(defaults, data, prefix):
    allowed = {}
    for key in defaults.keys():
        allowed[key] = True
    for key in data.keys():
        if key not in allowed:
            raise Exception("Unknown config key: %s%s" % (prefix, key))
        if isinstance(defaults.get(key), dict) and isinstance(data.get(key), dict):
            validate_no_unknown_keys(defaults[key], data[key], prefix + key + ".")


def strip_legacy_config_keys(data):
    clean = dict(data)
    ignored = []
    for key in LEGACY_CONFIG_KEYS.keys():
        if key in clean:
            ignored.append(key)
            del clean[key]
    if ignored:
        log("Config", "Ignored legacy single-circle config keys: %s" % ", ".join(sorted(ignored)))
    return clean


def validate_config(cfg):
    validate_no_unknown_keys(DEFAULT_CONFIG, cfg, "")
    for key in DEFAULT_CONFIG.keys():
        require_key(cfg, key)
    for loc_key in ["die_storage_location_xyz", "cal_work_location_xyz"]:
        for axis in ["x", "y", "z", "rotation"]:
            require_key(cfg[loc_key], axis)
            if not finite_number(cfg[loc_key][axis]):
                raise Exception("%s.%s must be finite." % (loc_key, axis))
    if str(cfg.get("single_circle_result_stage_name", "results")) != "results":
        raise Exception("single_circle_result_stage_name must be results.")
    if int(cfg.get("single_circle_expected_result_count", 2)) != 2:
        raise Exception("single_circle_expected_result_count must be 2.")
    if str(cfg.get("single_circle_model_mode", "openpnp_model_camera_offset")).strip().lower() not in ["openpnp_model_camera_offset"]:
        raise Exception("single_circle_model_mode must be openpnp_model_camera_offset.")
    if str(cfg.get("single_circle_fit_model_type", "openpnp_pairwise_180_offset")).strip().lower() != "openpnp_pairwise_180_offset":
        raise Exception("single_circle_fit_model_type must be openpnp_pairwise_180_offset.")
    if str(cfg.get("measurement_method", "camera_only")) != "camera_only":
        raise Exception("measurement_method must be camera_only.")
    if str(cfg.get("nozzle_target_method", "tool_aware")) != "tool_aware":
        raise Exception("nozzle_target_method must be tool_aware.")
    if str(cfg.get("single_circle_rotation_mode", "continuous")).lower() not in ["continuous", "bounded_shortest_path"]:
        raise Exception("single_circle_rotation_mode must be continuous or bounded_shortest_path.")
    if str(cfg.get("single_circle_cleanup_rotation_mode", "leave_current")).lower() not in ["leave_current", "return_to_zero", "return_to_start"]:
        raise Exception("single_circle_cleanup_rotation_mode must be leave_current, return_to_zero, or return_to_start.")
    if int(cfg["angle_subdivisions"]) < 1:
        raise Exception("angle_subdivisions must be >= 1.")
    if int(cfg["allow_misdetections"]) < 0:
        raise Exception("allow_misdetections must be >= 0.")
    if float(cfg["safe_travel_z"]) <= float(cfg["die_storage_location_xyz"]["z"]):
        raise Exception("safe_travel_z must be above die_storage_location_xyz.z.")
    if float(cfg["safe_travel_z"]) <= float(cfg["cal_work_location_xyz"]["z"]):
        raise Exception("safe_travel_z must be above cal_work_location_xyz.z.")
    for key in ["single_circle_center_expected_size_px",
                "single_circle_center_min_size_px",
                "single_circle_center_max_size_px",
                "single_circle_orientation_expected_size_px",
                "single_circle_orientation_min_size_px",
                "single_circle_orientation_max_size_px",
                "single_circle_min_size_ratio",
                "single_circle_max_rotation_step_deg",
                "single_circle_max_model_center_mm",
                "single_circle_max_model_radius_mm",
                "single_circle_max_model_rms_mm",
                "single_circle_max_model_peak_mm",
                "offset_threshold_mm",
                "max_apply_mm"]:
        if float(cfg[key]) <= 0.0:
            raise Exception("%s must be > 0." % key)
    if not finite_number(cfg.get("single_circle_apply_model_sign", -1.0)):
        raise Exception("single_circle_apply_model_sign must be finite.")
    if abs(float(cfg.get("single_circle_apply_model_sign", -1.0))) < 0.000001:
        raise Exception("single_circle_apply_model_sign must be non-zero.")
    if not finite_number(cfg.get("single_circle_initial_pick_rotation_deg", 0.0)):
        raise Exception("single_circle_initial_pick_rotation_deg must be finite.")
    if float(cfg["single_circle_center_min_size_px"]) > float(cfg["single_circle_center_max_size_px"]):
        raise Exception("single_circle_center_min_size_px must be <= max.")
    if float(cfg["single_circle_orientation_min_size_px"]) > float(cfg["single_circle_orientation_max_size_px"]):
        raise Exception("single_circle_orientation_min_size_px must be <= max.")
    if len(cfg.get("return_allowed_orientations_deg", [])) < 1:
        raise Exception("return_allowed_orientations_deg must not be empty.")
    for value in cfg.get("return_allowed_orientations_deg", []):
        if not finite_number(value):
            raise Exception("return_allowed_orientations_deg values must be finite.")
    return_orientation_mode = str(cfg.get("return_orientation_mode", "configured")).strip().lower()
    if return_orientation_mode not in ["configured", "nearest_cardinal"]:
        raise Exception("return_orientation_mode must be configured or nearest_cardinal.")
    for key in ["enabled", "start_radius_mm", "radius_step_mm", "max_radius_mm", "angle_step_deg", "settle_ms"]:
        require_key(cfg["spiral_search"], key)
    for key in ["start_radius_mm", "radius_step_mm", "max_radius_mm", "angle_step_deg"]:
        if float(cfg["spiral_search"][key]) <= 0.0:
            raise Exception("spiral_search.%s must be > 0." % key)
    if int(float(cfg["spiral_search"]["settle_ms"])) < 0:
        raise Exception("spiral_search.settle_ms must be >= 0.")
    for key in ["single_circle_require_size_data",
                "single_circle_allow_continuous_rotation",
                "single_circle_use_shortest_rotation_path",
                "single_circle_normalize_motion_rotation",
                "single_circle_unwind_after_run",
                "single_circle_use_current_rotation_for_pick",
                "use_high_level_nozzle_pick",
                "use_high_level_nozzle_place",
                "single_circle_call_nozzle_pick_after_manual_vacuum",
                "inherit_high_level_pick_from_four_circle_config",
                "enable_blowoff_on_place",
                "require_vacuum_actuator_on_place",
                "reject_measurements_outside_threshold",
                "fit_even_if_residual_high",
                "fit_residual_warning_only",
                "return_to_storage_use_computed_offset",
                "return_to_storage_apply_offset_to_pick",
                "return_to_storage_apply_offset_to_place",
                "allow_static_delta_as_same_run_pick_jog",
                "return_orientation_marker_enabled",
                "single_circle_preview_include_static_delta",
                "single_circle_preview_include_residual_angle_vector",
                "single_circle_confirm_descend_to_surface",
                "single_circle_confirm_use_orientation_rotation",
                "single_circle_pick_use_orientation_rotation",
                "single_circle_verify_static_xy_after_fit",
                "single_circle_physical_ab_verification",
                "single_circle_verify_old_new_preview",
                "single_circle_calibration_acquire_from_nominal_xy",
                "single_circle_update_current_die_loc_after_measure",
                "single_circle_block_active_correction_on_model_fail",
                "single_circle_auto_select_model_sign",
                "single_circle_enable_runtime_runout_compensation",
                "openpnp_pairwise_abort_on_walk_exceeded",
                "confirm_use_nozzle_target_frame",
                "return_pick_include_residual_angle_vector",
                "confirm_include_residual_angle_vector",
                "calibration_fit_include_residual_angle_vector",
                "top_camera_use_expected_loc_z_for_pixel_transform",
                "top_camera_log_focus_plane",
                "log_nozzle_z_plan",
                "return_verify_orientation_after_place",
                "run_bottom_camera_nozzle_square_after_fit",
                "bottom_square_set_global_rotation_offset",
                "bottom_square_apply_motion_correction",
                "bottom_square_verify_after_apply",
                "bottom_square_treat_near_45_as_ambiguous",
                "bottom_square_reject_ambiguous_45",
                "bottom_square_reject_near_square_orientation",
                "bottom_square_reject_near_45_as_ambiguous",
                "bottom_square_use_median_angle",
                "bottom_square_reject_if_position_dependent",
                "bottom_square_warning_only",
                "return_to_die_after_bottom_square",
                "return_to_storage_after_bottom_square",
                "return_pick_use_pre_square_correction_after_bottom_square",
                "post_square_recompute_static_pick_delta",
                "allow_confirm_with_stale_pre_square_correction",
                "post_square_physical_pick_place_verify",
                "run_post_square_single_angle_recenter",
                "allow_machine_writes",
                "allow_apply_without_verification",
                "write_machine_xml_backup",
                "confirm_allow_diagnostic_preview"]:
        text = str(cfg.get(key)).strip().lower()
        if text not in ["true", "false", "1", "0", "yes", "no", "on", "off"]:
            raise Exception("%s must be boolean-compatible." % key)
    mode = str(cfg.get("single_circle_pick_orientation_rotation_mode", "raw_marker_angle")).strip().lower()
    if mode not in ["raw_marker_angle", "nearest_cardinal", "current_nozzle_rotation", "configured"]:
        raise Exception("single_circle_pick_orientation_rotation_mode must be raw_marker_angle, nearest_cardinal, current_nozzle_rotation, or configured.")
    calibration_policy = str(cfg.get("calibration_pick_rotation_policy", "preserve_current_nozzle_rotation")).strip().lower()
    if calibration_policy not in ["preserve_current_nozzle_rotation", "match_detected_marker", "configured"]:
        raise Exception("calibration_pick_rotation_policy must be preserve_current_nozzle_rotation, match_detected_marker, or configured.")
    return_policy = str(cfg.get("return_pick_rotation_policy", "squared_marker_to_desired_storage")).strip().lower()
    if return_policy not in ["match_detected_marker", "preserve_current_nozzle_rotation", "squared_marker_to_desired_storage"]:
        raise Exception("return_pick_rotation_policy must be match_detected_marker, preserve_current_nozzle_rotation, or squared_marker_to_desired_storage.")
    return_place_policy = str(cfg.get("return_place_rotation_policy", "squared_marker_to_desired_storage")).strip().lower()
    if return_place_policy not in ["squared_marker_to_desired_storage", "preserve_current_nozzle_rotation"]:
        raise Exception("return_place_rotation_policy must be squared_marker_to_desired_storage or preserve_current_nozzle_rotation.")
    confirm_policy = str(cfg.get("confirm_rotation_policy", "preserve_current_nozzle_rotation")).strip().lower()
    if confirm_policy not in ["match_detected_marker", "preserve_current_nozzle_rotation"]:
        raise Exception("confirm_rotation_policy must be match_detected_marker or preserve_current_nozzle_rotation.")
    for key in ["single_circle_confirm_surface_clearance_mm",
                "single_circle_min_visible_preview_shift_mm",
                "single_circle_orientation_zero_offset_deg",
                "single_circle_orientation_sign",
                "single_circle_head_delta_sign_override",
                "return_held_object_rotation_sign",
                "return_orientation_tolerance_deg",
                "bottom_square_angle_sign",
                "square_bias_apply_sign",
                "bottom_square_safe_z",
                "bottom_square_max_angle_abs_deg",
                "bottom_square_max_angle_range_deg",
                "bottom_square_max_post_apply_abs_deg",
                "bottom_square_max_aspect_ratio",
                "bottom_square_min_orientation_aspect_ratio",
                "bottom_square_near_45_tolerance_deg",
                "post_square_recenter_angle_deg"]:
        if not finite_number(cfg.get(key)):
            raise Exception("%s must be finite." % key)
    for key in ["bottom_square_settle_ms", "bottom_square_pipeline_attempts"]:
        if not finite_number(cfg.get(key)):
            raise Exception("%s must be finite." % key)
        if int(float(cfg.get(key))) < 0:
            raise Exception("%s must be >= 0." % key)
    if int(float(cfg.get("bottom_square_pipeline_attempts", 3))) < 1:
        raise Exception("bottom_square_pipeline_attempts must be >= 1.")
    if abs(float(cfg.get("bottom_square_angle_sign", 1.0))) < 0.000001:
        raise Exception("bottom_square_angle_sign must be non-zero.")
    if abs(float(cfg.get("square_bias_apply_sign", 1.0))) < 0.000001:
        raise Exception("square_bias_apply_sign must be non-zero.")
    for key in ["bottom_square_max_angle_abs_deg",
                "bottom_square_max_angle_range_deg",
                "bottom_square_max_post_apply_abs_deg"]:
        if float(cfg.get(key)) < 0.0:
            raise Exception("%s must be >= 0." % key)
    nominal_z_source = str(cfg.get("bottom_square_nominal_z_source", "bottom_camera_location")).strip().lower()
    if nominal_z_source not in ["bottom_camera_location", "safe_z"]:
        raise Exception("bottom_square_nominal_z_source must be bottom_camera_location or safe_z.")
    square_apply_mode = str(cfg.get("bottom_square_apply_mode", "script_local_rotation_bias")).strip().lower()
    if square_apply_mode not in ["script_local_rotation_bias", "preview_only"]:
        raise Exception("bottom_square_apply_mode must be script_local_rotation_bias or preview_only.")
    for key in ["bottom_square_stage_fallbacks",
                "bottom_square_x_offsets_mm",
                "bottom_square_y_offsets_mm",
                "bottom_square_verify_offsets_mm"]:
        values = cfg.get(key, [])
        if not isinstance(values, list) or len(values) < 1:
            raise Exception("%s must be a non-empty list." % key)
        for value in values:
            if key == "bottom_square_stage_fallbacks":
                if str(value).strip() == "":
                    raise Exception("bottom_square_stage_fallbacks values must not be empty.")
            elif not finite_number(value):
                raise Exception("%s values must be finite." % key)
    if str(cfg.get("bottom_square_stage_name", "")).strip() == "":
        raise Exception("bottom_square_stage_name must not be empty.")
    if float(cfg.get("single_circle_min_visible_preview_shift_mm", 0.02)) < 0.0:
        raise Exception("single_circle_min_visible_preview_shift_mm must be >= 0.")
    if abs(float(cfg.get("single_circle_orientation_sign", 1.0))) < 0.000001:
        raise Exception("single_circle_orientation_sign must be non-zero.")
    if abs(float(cfg.get("single_circle_head_delta_sign_override", 1.0))) < 0.000001:
        raise Exception("single_circle_head_delta_sign_override must be non-zero.")
    if abs(float(cfg.get("return_held_object_rotation_sign", -1.0))) < 0.000001:
        raise Exception("return_held_object_rotation_sign must be non-zero.")
    focus_source = str(cfg.get("top_camera_focus_plane_source", "object_surface_z")).strip().lower()
    if focus_source not in ["object_surface_z", "cal_work_location_z", "storage_location_z", "manual"]:
        raise Exception("top_camera_focus_plane_source must be object_surface_z, cal_work_location_z, storage_location_z, or manual.")
    if focus_source == "manual" and not finite_number(cfg.get("top_camera_measurement_z_mm", None)):
        raise Exception("top_camera_measurement_z_mm must be finite when top_camera_focus_plane_source is manual.")
    pick_z_source = str(cfg.get("nozzle_pick_z_source", "part_top_surface_z")).strip().lower()
    if pick_z_source not in ["part_top_surface_z", "configured_location_z", "manual"]:
        raise Exception("nozzle_pick_z_source must be part_top_surface_z, configured_location_z, or manual.")
    if pick_z_source == "manual" and not finite_number(cfg.get("nozzle_pick_z_manual_mm", None)):
        raise Exception("nozzle_pick_z_manual_mm must be finite when nozzle_pick_z_source is manual.")
    place_z_source = str(cfg.get("nozzle_place_z_source", "part_top_surface_z")).strip().lower()
    if place_z_source not in ["part_top_surface_z", "configured_location_z", "manual"]:
        raise Exception("nozzle_place_z_source must be part_top_surface_z, configured_location_z, or manual.")
    if place_z_source == "manual" and not finite_number(cfg.get("nozzle_place_z_manual_mm", None)):
        raise Exception("nozzle_place_z_manual_mm must be finite when nozzle_place_z_source is manual.")
    if float(cfg.get("return_orientation_tolerance_deg", 10.0)) < 0.0:
        raise Exception("return_orientation_tolerance_deg must be >= 0.")
    if float(cfg.get("bottom_square_max_aspect_ratio", 1.20)) < 1.0:
        raise Exception("bottom_square_max_aspect_ratio must be >= 1.0.")
    if float(cfg.get("bottom_square_min_orientation_aspect_ratio", 1.15)) < 1.0:
        raise Exception("bottom_square_min_orientation_aspect_ratio must be >= 1.0.")
    if float(cfg.get("bottom_square_near_45_tolerance_deg", 3.0)) < 0.0:
        raise Exception("bottom_square_near_45_tolerance_deg must be >= 0.")
    post_square_mode = str(cfg.get("post_square_return_pick_delta_mode", "fresh_center_only")).strip().lower()
    if post_square_mode not in ["fresh_center_only", "pre_square_virtual_delta", "post_square_recomputed_delta"]:
        raise Exception("post_square_return_pick_delta_mode must be fresh_center_only, pre_square_virtual_delta, or post_square_recomputed_delta.")
    if len(cfg.get("single_circle_verify_angles_deg", [])) < 1:
        raise Exception("single_circle_verify_angles_deg must not be empty.")
    for value in cfg.get("single_circle_verify_angles_deg", []):
        if not finite_number(value):
            raise Exception("single_circle_verify_angles_deg values must be finite.")
    if str(cfg["spiral_search"].get("enabled", "")).strip().lower() not in ["true", "false", "1", "0", "yes", "no", "on", "off"]:
        raise Exception("spiral_search.enabled must be boolean-compatible.")
    values = cfg.get("openpnp_pairwise_angles_deg", [])
    if values is not None:
        if not isinstance(values, list):
            raise Exception("openpnp_pairwise_angles_deg must be a list.")
        for value in values:
            if not finite_number(value):
                raise Exception("openpnp_pairwise_angles_deg values must be finite.")
    if not finite_number(cfg.get("openpnp_pairwise_max_total_walk_mm", 3.0)):
        raise Exception("openpnp_pairwise_max_total_walk_mm must be finite.")
    if float(cfg.get("openpnp_pairwise_max_total_walk_mm", 3.0)) < 0.0:
        raise Exception("openpnp_pairwise_max_total_walk_mm must be >= 0.")
    if not finite_number(cfg.get("openpnp_pairwise_nominal_reacquire_guard_mm", 0.25)):
        raise Exception("openpnp_pairwise_nominal_reacquire_guard_mm must be finite.")
    if float(cfg.get("openpnp_pairwise_nominal_reacquire_guard_mm", 0.25)) < 0.0:
        raise Exception("openpnp_pairwise_nominal_reacquire_guard_mm must be >= 0.")
    return True


def migrate_continuous_angle_defaults(cfg):
    mode = str(cfg.get("single_circle_rotation_mode", "continuous")).lower()
    if mode != "continuous":
        return cfg
    start = float(cfg.get("angle_start_deg", 0.0))
    stop = float(cfg.get("angle_stop_deg", 360.0))
    if abs(start + 180.0) < 0.000001 and abs(stop - 180.0) < 0.000001:
        log("Config", "Migrating single-circle angle range -180..180 to 0..360 because single_circle_rotation_mode=continuous.")
        cfg["angle_start_deg"] = 0.0
        cfg["angle_stop_deg"] = 360.0
    return cfg


def migrate_single_circle_debug_defaults(cfg, raw_data):
    if "return_pick_include_residual_angle_vector" not in raw_data:
        cfg["return_pick_include_residual_angle_vector"] = False
        if str(cfg.get("return_pick_rotation_policy", "")).strip().lower() == "match_detected_marker":
            cfg["return_pick_rotation_policy"] = "preserve_current_nozzle_rotation"
    if "confirm_include_residual_angle_vector" not in raw_data:
        cfg["confirm_include_residual_angle_vector"] = False
        if str(cfg.get("confirm_rotation_policy", "")).strip().lower() == "match_detected_marker":
            cfg["confirm_rotation_policy"] = "preserve_current_nozzle_rotation"
    return cfg


def four_circle_config_path():
    return os.path.join(script_dir(), "pick_cameraAlign_Calibration_config.json")


def inherit_motion_io_defaults(cfg):
    path = four_circle_config_path()
    if not os.path.exists(path):
        return cfg
    try:
        f = open(path, "r")
        try:
            four = json.loads(f.read())
        finally:
            f.close()
    except:
        return cfg
    inherited_keys = [
        "force_vacuum_actuator_name",
        "force_blowoff_actuator_name",
        "vacuum_settle_ms",
        "blowoff_pulse_ms",
        "enable_blowoff_on_place",
        "spiral_search"
    ]
    if bool_value(cfg.get("inherit_high_level_pick_from_four_circle_config", False)):
        inherited_keys.append("use_high_level_nozzle_pick")
    for key in inherited_keys:
        if key not in four:
            continue
        current = cfg.get(key, DEFAULT_CONFIG.get(key, None))
        if key in ["force_vacuum_actuator_name", "force_blowoff_actuator_name"]:
            if str(current).strip() == "":
                cfg[key] = four.get(key, current)
        elif current == DEFAULT_CONFIG.get(key, None):
            cfg[key] = four.get(key, current)
    if str(cfg.get("force_vacuum_actuator_name", "")).strip() != "":
        log("Config", "Inherited force_vacuum_actuator_name=%s" %
            str(cfg.get("force_vacuum_actuator_name", "")))
    if str(cfg.get("force_blowoff_actuator_name", "")).strip() != "":
        log("Config", "Inherited force_blowoff_actuator_name=%s" %
            str(cfg.get("force_blowoff_actuator_name", "")))
    return cfg


def save_config(cfg):
    validate_config(cfg)
    clean = {}
    for key in DEFAULT_CONFIG.keys():
        clean[key] = cfg.get(key, DEFAULT_CONFIG[key])
    f = open(config_path(), "w")
    try:
        f.write(json.dumps(clean, sort_keys=True, indent=2))
    finally:
        f.close()


def load_config():
    path = config_path()
    if not os.path.exists(path):
        cfg = dict(DEFAULT_CONFIG)
        inherit_motion_io_defaults(cfg)
        migrate_continuous_angle_defaults(cfg)
        validate_config(cfg)
        save_config(cfg)
        log("Config", "Config did not exist; wrote %s" % path)
        return cfg
    f = open(path, "r")
    try:
        data = json.loads(f.read())
    finally:
        f.close()
    data = strip_legacy_config_keys(data)
    validate_no_unknown_keys(DEFAULT_CONFIG, data, "")
    cfg = deep_update(DEFAULT_CONFIG, data)
    inherit_motion_io_defaults(cfg)
    migrate_continuous_angle_defaults(cfg)
    migrate_single_circle_debug_defaults(cfg, data)
    validate_config(cfg)
    return cfg


def save_result(result):
    f = open(result_path(), "w")
    try:
        f.write(json.dumps(result, sort_keys=True, indent=2))
    finally:
        f.close()


def load_last_result():
    if not os.path.exists(result_path()):
        raise Exception("No saved calibration result found: %s" % result_path())
    f = open(result_path(), "r")
    try:
        return json.loads(f.read())
    finally:
        f.close()


def save_last_confirm_result(result):
    f = open(last_confirm_path(), "w")
    try:
        f.write(json.dumps(result, sort_keys=True, indent=2))
    finally:
        f.close()


def is_cancel_requested():
    token = _cancel_token
    if token is None:
        return False
    try:
        return bool(token.isCancelled())
    except:
        pass
    try:
        return bool(token.cancelled)
    except:
        pass
    return False


def check_cancel(tag="Abort"):
    if is_cancel_requested():
        log(tag, "Cancellation requested.")
        raise Exception("Calibration cancelled by user.")


def sleep_ms(ms):
    if int(ms) > 0:
        check_cancel()
        Thread.sleep(int(ms))


def wait_still(movable):
    try:
        movable.waitForCompletion(CompletionType.WaitForStillstand)
    except:
        pass


def location_to_diag(loc):
    return base.location_to_diag(loc)


def loc_from_xyz(data):
    return base.loc_from_xyz(data)


def infer_part_height_mm(part):
    return base.infer_part_height_mm(part)


def part_height_z_mm(cfg):
    return base.part_height_z_mm(cfg)


def pick_surface_z_mm(base_loc, cfg):
    return base.pick_surface_z_mm(base_loc, cfg)


def storage_pick_surface_z_mm(cfg, storage_loc=None):
    return base.storage_pick_surface_z_mm(cfg, storage_loc)


def resolve_nozzle_pick_z(base_loc, cfg):
    base_loc = base_loc.convertToUnits(LengthUnit.Millimeters)
    source = str(cfg.get("nozzle_pick_z_source", "part_top_surface_z")).strip().lower()
    if source == "part_top_surface_z":
        return base.pick_surface_z_mm(base_loc, cfg)
    if source == "configured_location_z":
        return float(base_loc.getZ())
    if source == "manual":
        return float(cfg["nozzle_pick_z_manual_mm"])
    raise Exception("Invalid nozzle_pick_z_source: %s" % source)


def resolve_nozzle_place_z(target_loc, cfg):
    target_loc = target_loc.convertToUnits(LengthUnit.Millimeters)
    source = str(cfg.get("nozzle_place_z_source", "part_top_surface_z")).strip().lower()
    if source == "part_top_surface_z":
        return base.pick_surface_z_mm(target_loc, cfg)
    if source == "configured_location_z":
        return float(target_loc.getZ())
    if source == "manual":
        return float(cfg["nozzle_place_z_manual_mm"])
    raise Exception("Invalid nozzle_place_z_source: %s" % source)


def log_nozzle_pick_z_plan(base_loc, cfg, selected_pick_z):
    if not bool_value(cfg.get("log_nozzle_z_plan", True)):
        return
    base_loc = base_loc.convertToUnits(LengthUnit.Millimeters)
    surface_z = base.pick_surface_z_mm(base_loc, cfg)
    source = str(cfg.get("nozzle_pick_z_source", "part_top_surface_z")).strip().lower()
    log("NozzleZPlan", "purpose=pick")
    log("NozzleZPlan", "base_loc_z=%.5f" % float(base_loc.getZ()))
    log("NozzleZPlan", "part_height_z=%.5f" % float(part_height_z_mm(cfg)))
    log("NozzleZPlan", "pick_surface_z=%.5f" % float(surface_z))
    log("NozzleZPlan", "selected_pick_z=%.5f" % float(selected_pick_z))
    log("NozzleZPlan", "source=%s" % source)


def log_nozzle_place_z_plan(target_loc, cfg, selected_place_z):
    if not bool_value(cfg.get("log_nozzle_z_plan", True)):
        return
    target_loc = target_loc.convertToUnits(LengthUnit.Millimeters)
    surface_z = base.pick_surface_z_mm(target_loc, cfg)
    source = str(cfg.get("nozzle_place_z_source", "part_top_surface_z")).strip().lower()
    log("NozzleZPlan", "purpose=place")
    log("NozzleZPlan", "target_loc_z=%.5f" % float(target_loc.getZ()))
    log("NozzleZPlan", "part_height_z=%.5f" % float(part_height_z_mm(cfg)))
    log("NozzleZPlan", "place_surface_z=%.5f" % float(surface_z))
    log("NozzleZPlan", "selected_place_z=%.5f" % float(selected_place_z))
    log("NozzleZPlan", "source=%s" % source)


def base_z_for_context(tag, cfg):
    t = str(tag).strip().lower()
    if t == "storage":
        return float(loc_from_xyz(cfg["die_storage_location_xyz"]).getZ())
    return float(loc_from_xyz(cfg["cal_work_location_xyz"]).getZ())


def force_base_z(loc, base_z):
    loc = loc.convertToUnits(LengthUnit.Millimeters)
    return mm_loc(float(loc.getX()), float(loc.getY()), float(base_z), float(loc.getRotation()))


def force_cal_base_z(loc, cfg):
    return force_base_z(loc, float(loc_from_xyz(cfg["cal_work_location_xyz"]).getZ()))


def force_storage_base_z(loc, cfg):
    return force_base_z(loc, float(loc_from_xyz(cfg["die_storage_location_xyz"]).getZ()))


def measured_xy_as_base_loc(measured_loc, base_context, cfg):
    if str(base_context).strip().lower() == "storage":
        return force_storage_base_z(measured_loc, cfg)
    return force_cal_base_z(measured_loc, cfg)


def force_context_base_z(loc, cfg, tag):
    if str(tag).strip().lower() == "storage":
        return force_storage_base_z(loc, cfg)
    return force_cal_base_z(loc, cfg)


def guard_expected_base_location(loc, cfg, tag, context):
    incoming = loc.convertToUnits(LengthUnit.Millimeters)
    base_z = base_z_for_context(tag, cfg)
    surface_z = base_z + float(part_height_z_mm(cfg))
    if abs(float(incoming.getZ()) - surface_z) < 0.001:
        log("BaseZGuard", "WARNING expected_loc_z looked like surface_z; forcing back to base_z")
    out = force_base_z(incoming, base_z)
    log("BaseZGuard", "context=%s" % str(context))
    log("BaseZGuard", "expected_loc_z_before=%.5f" % float(incoming.getZ()))
    log("BaseZGuard", "expected_loc_z_after=%.5f" % float(out.getZ()))
    log("BaseZGuard", "part_height_z=%.5f" % float(part_height_z_mm(cfg)))
    log("BaseZGuard", "expected_focus_z=%.5f" % float(surface_z))
    return out


def base_loc_with_measured_orientation(measured_loc, pose, base_context, cfg):
    loc = measured_xy_as_base_loc(measured_loc, base_context, cfg)
    return mm_loc(loc.getX(), loc.getY(), loc.getZ(),
                  float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", loc.getRotation()))))


def guard_pick_place_z(base_z, selected_z, cfg, purpose):
    part_z = float(part_height_z_mm(cfg))
    max_z = float(base_z) + part_z + 0.05
    if float(selected_z) > max_z:
        raise Exception("%s Z double-added part height: base %.5f selected %.5f part_height %.5f" %
                        (str(purpose), float(base_z), float(selected_z), part_z))


def mm_loc(x, y, z, r):
    return base.mm_loc(x, y, z, r)


def normalize_angle(angle):
    return base.normalize_angle(angle)


def mean(values):
    return base.mean(values)


def rms(values):
    return base.rms(values)


def object_name(obj):
    return base.object_name(obj)


def image_size_dict_from_image(image):
    if image is None:
        return None
    try:
        return {"width": int(image.getWidth()), "height": int(image.getHeight())}
    except:
        return None


def image_size_dict_from_mat(mat):
    if mat is None:
        return None
    try:
        if mat.empty():
            return None
    except:
        pass
    try:
        return {"width": int(mat.cols()), "height": int(mat.rows())}
    except:
        return None


def save_last_vision_image(image):
    global _last_vision_image_path
    if image is None:
        return
    path = last_image_path()
    try:
        ImageIO.write(image, "png", File(path))
        _last_vision_image_path = path
        log("VisionImage", "saved_last_image=%s" % path)
    except Exception, e:
        log("VisionImage", "Could not save last capture image: %s" % e)


def save_pipeline_or_capture_image(pipeline, capture_image):
    global _last_capture_image_size, _last_pipeline_working_image_size
    _last_capture_image_size = image_size_dict_from_image(capture_image)
    _last_pipeline_working_image_size = None
    image = capture_image
    try:
        mat = pipeline.getWorkingImage()
        _last_pipeline_working_image_size = image_size_dict_from_mat(mat)
        if mat is not None and not mat.empty():
            image = OpenCvUtils.toBufferedImage(mat)
    except:
        pass
    save_last_vision_image(image)


def number_attr(obj, names):
    for name in names:
        try:
            value = getattr(obj, name)
            if callable(value):
                value = value()
            return float(value)
        except:
            pass
    return None


def size_diameter_from_item(item):
    if isinstance(item, dict):
        for key in ["diameter_px", "diameter", "size_px", "size"]:
            if key in item:
                return float(item[key])
        for key in ["radius_px", "radius"]:
            if key in item:
                return 2.0 * float(item[key])
        return None
    diameter = number_attr(item, ["diameter", "getDiameter", "size", "getSize"])
    if diameter is not None:
        return diameter
    try:
        sz = item.size
        width = number_attr(sz, ["width", "getWidth"])
        height = number_attr(sz, ["height", "getHeight"])
        if width is not None and height is not None:
            return max(float(width), float(height))
    except:
        pass
    radius = number_attr(item, ["radius", "getRadius"])
    if radius is not None:
        return 2.0 * float(radius)
    return None


def point_from_item(item, require_size):
    if isinstance(item, dict):
        if "x" in item and "y" in item:
            x = float(item["x"])
            y = float(item["y"])
        elif "x_px" in item and "y_px" in item:
            x = float(item["x_px"])
            y = float(item["y_px"])
        else:
            return None
        size = size_diameter_from_item(item)
        if size is None and bool(require_size):
            return None
        out = {"x_px": x, "y_px": y}
        if size is not None:
            out["size_px"] = float(size)
            out["diameter_px"] = float(size)
            out["radius_px"] = float(size) / 2.0
        return out
    center = None
    try:
        center = item.center
    except:
        center = None
    if center is None:
        try:
            center = item.pt
        except:
            center = None
    owner = center if center is not None else item
    x = number_attr(owner, ["x", "getX"])
    y = number_attr(owner, ["y", "getY"])
    if x is None or y is None:
        return None
    size = size_diameter_from_item(item)
    if size is None and bool(require_size):
        return None
    out = {"x_px": float(x), "y_px": float(y)}
    if size is not None:
        out["size_px"] = float(size)
        out["diameter_px"] = float(size)
        out["radius_px"] = float(size) / 2.0
    return out


def java_list_to_py(model):
    return base.java_list_to_py(model)


def get_model(result):
    return base.get_model(result)


def extract_stage_items(pipeline, stage_name):
    try:
        model = get_model(pipeline.getResult(stage_name))
    except:
        return []
    return java_list_to_py(model)


def extract_stage_points_from_items(items, cfg):
    out = []
    require_size = bool_value(cfg.get("single_circle_require_size_data", True))
    for item in items:
        point = point_from_item(item, require_size)
        if point is not None:
            out.append(point)
    return out


def circle_for_log(circle):
    if circle is None:
        return "(None,None)"
    return "(%.3f, %.3f)" % (float(circle.get("x_px", 0.0)), float(circle.get("y_px", 0.0)))


def reject_classification(reason, raw_count):
    log("SingleCircleClassify", "raw_count=%d" % int(raw_count))
    log("SingleCircleClassify", "accepted=False reason=%s" % reason)
    raise SingleCircleDetectionError(reason)


def extract_and_classify_two_circles_from_results(pipeline, cfg):
    stage_name = str(cfg.get("single_circle_result_stage_name", "results"))
    log("SingleCircleVision", "result_stage=%s" % stage_name)
    raw_items = extract_stage_items(pipeline, stage_name)
    raw_count = len(raw_items)
    expected = int(cfg.get("single_circle_expected_result_count", 2))
    if raw_count != expected:
        reject_classification("raw_count %d != expected %d" % (raw_count, expected), raw_count)
    raw_points = extract_stage_points_from_items(raw_items, cfg)
    if len(raw_points) != expected:
        reject_classification("usable circle count %d != expected %d; x/y or size data missing" %
                              (len(raw_points), expected), raw_count)
    for point in raw_points:
        if "x_px" not in point or "y_px" not in point:
            reject_classification("circle missing x/y", raw_count)
        if bool_value(cfg.get("single_circle_require_size_data", True)) and "size_px" not in point:
            reject_classification("circle missing size/radius/diameter data", raw_count)
    sorted_points = sorted(raw_points, key=lambda p: float(p.get("size_px", 0.0)))
    orientation = dict(sorted_points[0])
    center = dict(sorted_points[1])
    center_size = float(center.get("size_px", 0.0))
    orientation_size = float(orientation.get("size_px", 0.0))
    if center_size < float(cfg["single_circle_center_min_size_px"]) or center_size > float(cfg["single_circle_center_max_size_px"]):
        reject_classification("center size %.5f outside [%.5f, %.5f]" %
                              (center_size, float(cfg["single_circle_center_min_size_px"]),
                               float(cfg["single_circle_center_max_size_px"])), raw_count)
    if orientation_size < float(cfg["single_circle_orientation_min_size_px"]) or orientation_size > float(cfg["single_circle_orientation_max_size_px"]):
        reject_classification("orientation size %.5f outside [%.5f, %.5f]" %
                              (orientation_size, float(cfg["single_circle_orientation_min_size_px"]),
                               float(cfg["single_circle_orientation_max_size_px"])), raw_count)
    ratio = center_size / orientation_size if orientation_size > 0.000001 else 999999.0
    if ratio < float(cfg["single_circle_min_size_ratio"]):
        reject_classification("size ratio %.5f < single_circle_min_size_ratio %.5f" %
                              (ratio, float(cfg["single_circle_min_size_ratio"])), raw_count)
    center["role"] = "center"
    orientation["role"] = "orientation"
    log("SingleCircleClassify", "raw_count=%d" % raw_count)
    log("SingleCircleClassify", "center_size_px=%.5f" % center_size)
    log("SingleCircleClassify", "orientation_size_px=%.5f" % orientation_size)
    log("SingleCircleClassify", "size_ratio=%.5f" % ratio)
    log("SingleCircleClassify", "center_circle_px=%s" % circle_for_log(center))
    log("SingleCircleClassify", "orientation_circle_px=%s" % circle_for_log(orientation))
    log("SingleCircleClassify", "accepted=True")
    return {"center_circle": center,
            "orientation_circle": orientation,
            "raw_count": raw_count,
            "center_size_px": center_size,
            "orientation_size_px": orientation_size,
            "size_ratio": ratio,
            "accepted": True}


def orientation_from_circles(center, orientation, cfg=None):
    dx = float(orientation["x_px"]) - float(center["x_px"])
    dy = float(orientation["y_px"]) - float(center["y_px"])
    marker_angle = normalize_angle(math.degrees(math.atan2(dy, dx)))
    sign = 1.0
    offset = 0.0
    if cfg is not None:
        sign = float(cfg.get("single_circle_orientation_sign", 1.0))
        offset = float(cfg.get("single_circle_orientation_zero_offset_deg", 0.0))
    die_orientation = normalize_angle(sign * marker_angle + offset)
    return {"raw_marker_angle_deg": marker_angle,
            "die_orientation_deg": die_orientation,
            "marker_dx_px": dx,
            "marker_dy_px": dy}


def nearest_allowed_orientation(angle, allowed):
    best = None
    best_error = None
    for candidate in allowed:
        err = abs(normalize_angle(float(angle) - float(candidate)))
        if best_error is None or err < best_error:
            best = float(candidate)
            best_error = err
    return normalize_angle(best)


def assert_center_invariant(pose, tag):
    center = pose["center_circle_px"]
    orientation = pose["orientation_circle_px"]
    log("CenterInvariant", "center_source=large_single_center_circle_only")
    log("CenterInvariant", "orientation_circle_used_for_center=False")
    log("CenterInvariant", "center_circle_px=%s" % circle_for_log(center))
    log("CenterInvariant", "orientation_circle_px=%s" % circle_for_log(orientation))
    if abs(float(pose["center_x_px"]) - float(center["x_px"])) > 0.000001:
        raise Exception("%s center_x_px changed from large center circle." % tag)
    if abs(float(pose["center_y_px"]) - float(center["y_px"])) > 0.000001:
        raise Exception("%s center_y_px changed from large center circle." % tag)


def get_part_pipeline(part):
    return base.get_part_pipeline(part)


def run_pipeline_and_classify(camera, nozzle, part, cfg, attempts=None):
    pipeline = get_part_pipeline(part)
    try:
        pipeline.setProperty("camera", camera)
    except:
        pass
    try:
        pipeline.setProperty("nozzle", nozzle)
    except:
        pass
    try:
        pipeline.setProperty("part", part)
    except:
        pass
    last_error = None
    if attempts is None:
        attempts = PIPELINE_ATTEMPTS
    for attempt in range(1, int(attempts) + 1):
        check_cancel("SingleCircleVision")
        try:
            capture = None
            try:
                capture = camera.settleAndCapture()
            except:
                sleep_ms(SETTLE_MS)
            pipeline.process()
            save_pipeline_or_capture_image(pipeline, capture)
            return extract_and_classify_two_circles_from_results(pipeline, cfg)
        except SingleCircleDetectionError, e:
            last_error = e
            log("SingleCircleVision", "Pipeline attempt %d rejected: %s" % (attempt, e))
        except Exception, e:
            last_error = e
            log("SingleCircleVision", "Pipeline attempt %d failed: %s" % (attempt, e))
        sleep_ms(SETTLE_MS)
    if last_error is not None:
        raise last_error
    raise SingleCircleDetectionError("No usable result from pipeline.")


def single_circle_search_offsets(cfg):
    yield (0.0, 0.0)
    spiral = cfg.get("spiral_search", {})
    if not bool_value(spiral.get("enabled", True)):
        return
    r = float(spiral.get("start_radius_mm", 0.15))
    max_r = float(spiral.get("max_radius_mm", 1.20))
    step_r = float(spiral.get("radius_step_mm", 0.15))
    step_a = float(spiral.get("angle_step_deg", 45.0))
    while r <= max_r + 0.0000001:
        a = 0.0
        while a < 360.0:
            yield (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
            a += step_a
        r += step_r


def acquire_classified_with_search(camera, nozzle, part, cfg, expected_camera_loc, tag):
    origin = expected_camera_loc.convertToUnits(LengthUnit.Millimeters)
    last_error = None
    for dx, dy in single_circle_search_offsets(cfg):
        check_cancel("SingleCircleVision")
        target = camera_measurement_loc(camera, origin.getX() + dx, origin.getY() + dy, origin.getZ())
        if abs(dx) > 0.000001 or abs(dy) > 0.000001:
            log("SingleCircleSearch", "tag=%s offset_dx=%.5f offset_dy=%.5f" % (tag, dx, dy))
        else:
            log("SingleCircleSearch", "tag=%s offset_dx=0.00000 offset_dy=0.00000" % tag)
        base.move_tool_safe(camera, target, "SingleCircleVision")
        sleep_ms(cfg.get("spiral_search", {}).get("settle_ms", SETTLE_MS))
        try:
            classified = run_pipeline_and_classify(camera, nozzle, part, cfg, 1)
            classified["search_dx_mm"] = float(dx)
            classified["search_dy_mm"] = float(dy)
            log("SingleCircleSearch", "tag=%s acquired=True offset_dx=%.5f offset_dy=%.5f" %
                (tag, dx, dy))
            return classified
        except SingleCircleDetectionError, e:
            last_error = e
            log("SingleCircleSearch", "tag=%s acquired=False offset_dx=%.5f offset_dy=%.5f reason=%s continuing spiral search" %
                (tag, dx, dy, str(e)))
    if last_error is not None:
        raise last_error
    raise SingleCircleDetectionError("Could not detect both circles in configured search area.")


def camera_measurement_loc(camera, x, y, z):
    cam = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
    return mm_loc(float(x), float(y), float(z), cam.getRotation())


def top_camera_measurement_z_for_location(expected_loc, cfg):
    expected_loc = expected_loc.convertToUnits(LengthUnit.Millimeters)
    source = str(cfg.get("top_camera_focus_plane_source", "object_surface_z")).strip().lower()
    if source == "object_surface_z":
        measurement_z = pick_surface_z_mm(expected_loc, cfg)
    elif source == "cal_work_location_z":
        measurement_z = float(cfg["cal_work_location_xyz"]["z"])
    elif source == "storage_location_z":
        measurement_z = float(cfg["die_storage_location_xyz"]["z"])
    elif source == "manual":
        measurement_z = float(cfg.get("top_camera_measurement_z_mm"))
    else:
        raise Exception("Invalid top_camera_focus_plane_source: %s" % source)
    return source, float(measurement_z)


def pixel_locations_for_pose(camera, nozzle, pose, fallback_loc, cfg):
    px = float(pose["center_x_px"])
    py = float(pose["center_y_px"])
    camera_only = VisionUtils.getPixelLocation(camera, px, py).convertToUnits(LengthUnit.Millimeters)
    tool_aware = VisionUtils.getPixelLocation(camera, nozzle, px, py).convertToUnits(LengthUnit.Millimeters)
    dx = tool_aware.getX() - camera_only.getX()
    dy = tool_aware.getY() - camera_only.getY()
    mag = math.sqrt(dx * dx + dy * dy)
    log("PixelTransform", "camera_only_measurement=(%.5f, %.5f)" %
        (camera_only.getX(), camera_only.getY()))
    log("PixelTransform", "nozzle_target_tool_aware=(%.5f, %.5f)" %
        (tool_aware.getX(), tool_aware.getY()))
    log("PixelTransform", "measurement_method=camera_only")
    log("PixelTransform", "nozzle_target_method=tool_aware")
    log("PixelTransform", "tool_minus_camera_only=(%.5f, %.5f, %.5f)" % (dx, dy, mag))
    log("FrameGuard", "fit_frame=camera_only")
    log("FrameGuard", "motion_frame=tool_aware")
    log("FrameGuard", "model_delta_frame=machine_xy")
    log("FrameGuard", "tool_aware_minus_camera_only_logged_only=True")
    fallback = fallback_loc.convertToUnits(LengthUnit.Millimeters)
    return {
        "measurement_location": mm_loc(camera_only.getX(), camera_only.getY(), fallback.getZ(), fallback.getRotation()),
        "nozzle_target_location": mm_loc(tool_aware.getX(), tool_aware.getY(), fallback.getZ(), fallback.getRotation()),
        "camera_only": camera_only,
        "tool_aware": tool_aware,
        "tool_minus_camera_only": {"dx": dx, "dy": dy, "mag": mag},
        "measurement_coordinate_frame": "camera_only",
        "nozzle_target_coordinate_frame": "tool_aware"
    }


def overlay_point(circle):
    out = {"x": float(circle.get("x_px", 0.0)), "y": float(circle.get("y_px", 0.0))}
    if "radius_px" in circle:
        out["radius"] = float(circle["radius_px"])
    elif "size_px" in circle:
        out["radius"] = float(circle["size_px"]) / 2.0
    return out


def diag_point(circle):
    out = {"x_px": float(circle.get("x_px", 0.0)), "y_px": float(circle.get("y_px", 0.0))}
    if "size_px" in circle:
        out["size_px"] = float(circle["size_px"])
        out["diameter_px"] = float(circle["size_px"])
        out["radius_px"] = float(circle["size_px"]) / 2.0
    elif "radius_px" in circle:
        out["radius_px"] = float(circle["radius_px"])
        out["diameter_px"] = float(circle["radius_px"]) * 2.0
        out["size_px"] = float(circle["radius_px"]) * 2.0
    return out


def save_pose_overlay_json(pose, lock, context):
    overlay = {
        "image_path": last_image_path(),
        "center_circle_px": diag_point(pose["center_circle_px"]),
        "orientation_marker_px": diag_point(pose["orientation_circle_px"]),
        "orientation_circle_px": overlay_point(pose["orientation_circle_px"]),
        "actual_center_px": overlay_point(pose["center_circle_px"]),
        "detected_circles_px": [overlay_point(pose["center_circle_px"]), overlay_point(pose["orientation_circle_px"])],
        "measured_center_mm": location_to_diag(lock["measurement_location"]),
        "nozzle_target_mm": location_to_diag(lock["nozzle_target_location"]),
        "orientation_deg": float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0))),
        "orientation_raw_deg": float(pose.get("orientation_raw_deg", 0.0)),
        "orientation_raw_marker_deg": float(pose.get("orientation_raw_marker_deg", pose.get("orientation_raw_deg", 0.0))),
        "die_orientation_deg": float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0))),
        "orientation_snapped_deg": float(pose.get("orientation_snapped_deg", 0.0)),
        "center_source": "large_single_center_circle_only",
        "orientation_circle_used_for_center": False,
        "orientation_marker_used_for_center": False,
        "context": context,
        "timestamp": time.time()
    }
    f = open(last_overlay_path(), "w")
    try:
        f.write(json.dumps(overlay, sort_keys=True, indent=2))
    finally:
        f.close()
    log("VisionImage", "saved_pose_overlay_json=%s" % last_overlay_path())
    return overlay


def acquire_pose_for_location(camera, nozzle, part, expected_loc, cfg, tag):
    expected_loc = guard_expected_base_location(expected_loc, cfg, tag, "AcquirePose")
    focus_source, measurement_z = top_camera_measurement_z_for_location(expected_loc, cfg)
    target = camera_measurement_loc(camera, expected_loc.getX(), expected_loc.getY(), measurement_z)
    if bool_value(cfg.get("top_camera_log_focus_plane", True)):
        log("FocusPlane", "source=%s" % focus_source)
        log("FocusPlane", "expected_loc_z=%.5f" % float(expected_loc.getZ()))
        log("FocusPlane", "measurement_z=%.5f" % float(measurement_z))
        log("FocusPlane", "camera_command_z=%.5f" % float(target.getZ()))
        log("FocusPlane", "pixel_transform_z_basis=%.5f" % float(measurement_z))
    log("SingleCircleVision", "[%s] acquiring X=%.4f Y=%.4f Z=%.4f" %
        (tag, target.getX(), target.getY(), target.getZ()))
    classified = acquire_classified_with_search(camera, nozzle, part, cfg, target, tag)
    center = classified["center_circle"]
    orientation = classified["orientation_circle"]
    orientation_info = orientation_from_circles(center, orientation, cfg)
    raw_marker_angle = float(orientation_info["raw_marker_angle_deg"])
    die_orientation = float(orientation_info["die_orientation_deg"])
    log("Orientation", "marker_vector_px=(%.5f, %.5f)" %
        (float(orientation_info["marker_dx_px"]), float(orientation_info["marker_dy_px"])))
    log("Orientation", "raw_marker_angle_deg=%.5f" % raw_marker_angle)
    log("Orientation", "die_orientation_deg=%.5f" % die_orientation)
    log("Orientation", "detected_die_orientation_deg=%.5f" % die_orientation)
    log("Orientation", "orientation_circle_used_for_center=False")
    log("Orientation", "used_for_model_fit=False")
    log("Orientation", "used_for_xy_center=False")
    log("Orientation", "used_for_storage_orientation=%s" %
        str(bool_value(cfg.get("return_orientation_marker_enabled", True))))
    snapped = nearest_allowed_orientation(die_orientation, cfg.get("return_allowed_orientations_deg", [90.0, 180.0, 270.0]))
    pose = {
        "center_x_px": float(center["x_px"]),
        "center_y_px": float(center["y_px"]),
        "center_circle_px": diag_point(center),
        "orientation_circle_px": diag_point(orientation),
        "orientation_raw_deg": raw_marker_angle,
        "orientation_raw_marker_deg": raw_marker_angle,
        "die_orientation_deg": die_orientation,
        "orientation_snapped_deg": snapped,
        "orientation_used_for_xy": False,
        "circle_count": 2,
        "classification": classified,
        "search_dx_mm": float(classified.get("search_dx_mm", 0.0)),
        "search_dy_mm": float(classified.get("search_dy_mm", 0.0)),
        "center_source": "large_single_center_circle_only",
        "orientation_circle_used_for_center": False
    }
    assert_center_invariant(pose, tag)
    pixel_basis_loc = mm_loc(expected_loc.getX(), expected_loc.getY(), measurement_z, expected_loc.getRotation())
    pixel_locs = pixel_locations_for_pose(camera, nozzle, pose, pixel_basis_loc, cfg)
    measurement_loc = pixel_locs["measurement_location"]
    nozzle_target_loc = pixel_locs["nozzle_target_location"]
    camera_loc = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
    pose["measured_center_x_mm"] = float(measurement_loc.getX())
    pose["measured_center_y_mm"] = float(measurement_loc.getY())
    pose["nozzle_target_x_mm"] = float(nozzle_target_loc.getX())
    pose["nozzle_target_y_mm"] = float(nozzle_target_loc.getY())
    detected_dx = float(measurement_loc.getX() - expected_loc.getX())
    detected_dy = float(measurement_loc.getY() - expected_loc.getY())
    log("PixelTransformDiag", "camera_location=(%.5f,%.5f,%.5f,%.5f)" %
        (float(camera_loc.getX()), float(camera_loc.getY()), float(camera_loc.getZ()), float(camera_loc.getRotation())))
    log("PixelTransformDiag", "expected_camera_target=(%.5f,%.5f,%.5f,%.5f)" %
        (float(target.getX()), float(target.getY()), float(target.getZ()), float(target.getRotation())))
    log("PixelTransformDiag", "detected_px=(%.5f,%.5f)" %
        (float(pose["center_x_px"]), float(pose["center_y_px"])))
    log("PixelTransformDiag", "pixel_location_mm=(%.5f,%.5f)" %
        (float(measurement_loc.getX()), float(measurement_loc.getY())))
    log("PixelTransformDiag", "detected_minus_expected_mm=(%.5f,%.5f)" %
        (detected_dx, detected_dy))
    log("PixelTransformDiag", "search_offset_used=(%.5f,%.5f)" %
        (float(pose.get("search_dx_mm", 0.0)), float(pose.get("search_dy_mm", 0.0))))
    lock = {
        "pose": pose,
        "measurement_location": measurement_loc,
        "nozzle_target_location": nozzle_target_loc,
        "camera_location": camera_loc,
        "expected_camera_target": target,
        "measurement_coordinate_frame": "camera_only",
        "nozzle_target_coordinate_frame": "tool_aware",
        "tool_minus_camera_only": pixel_locs["tool_minus_camera_only"]
    }
    overlay = save_pose_overlay_json(pose, lock, tag)
    lock["overlay"] = overlay
    log("SingleCircleVision", "center_circle_px=%s" % circle_for_log(center))
    log("SingleCircleVision", "orientation_circle_px=%s" % circle_for_log(orientation))
    return lock, measurement_loc


def resolve_assigned_actuator(nozzle, cfg, kind):
    return base.resolve_assigned_actuator(nozzle, cfg, kind)


def set_actuator_state(actuator, on_off):
    return base.set_actuator_state(actuator, on_off)


def set_vacuum_state(nozzle, cfg, on_off, fatal):
    actuator = resolve_assigned_actuator(nozzle, cfg, "vacuum")
    if actuator is None:
        log("Vacuum", "No vacuum actuator resolved for %s" % ("ON" if bool(on_off) else "OFF"))
        if fatal:
            raise Exception("Vacuum actuator is not configured and could not be resolved.")
        return False
    set_actuator_state(actuator, on_off)
    log("Vacuum", "%s -> %s" % (object_name(actuator), "ON" if bool(on_off) else "OFF"))
    return True


def set_blowoff_state(nozzle, cfg, on_off):
    actuator = resolve_assigned_actuator(nozzle, cfg, "blowoff")
    if actuator is None:
        log("Blowoff", "No blowoff actuator resolved for %s" % ("ON" if bool(on_off) else "OFF"))
        return False
    set_actuator_state(actuator, on_off)
    log("Blowoff", "%s -> %s" % (object_name(actuator), "ON" if bool(on_off) else "OFF"))
    return True


def single_circle_command_rotation(current_r, target_r, cfg):
    mode = str(cfg.get("single_circle_rotation_mode", "continuous")).lower()
    if mode == "continuous":
        if bool_value(cfg.get("single_circle_use_shortest_rotation_path", False)):
            return base.nearest_equivalent_rotation(float(target_r), float(current_r))
        if bool_value(cfg.get("single_circle_normalize_motion_rotation", False)):
            return normalize_angle(float(target_r))
        return float(target_r)
    if mode == "bounded_shortest_path":
        return base.nearest_equivalent_rotation(float(target_r), float(current_r))
    raise Exception("Invalid single_circle_rotation_mode: %s" % mode)


def single_circle_check_rotation_step(current_r, command_r, cfg, context):
    delta = float(command_r) - float(current_r)
    max_step = float(cfg.get("single_circle_max_rotation_step_deg", 9999.0))
    if abs(delta) > max_step:
        raise Exception("Refusing single-circle rotation step %.4f deg > single_circle_max_rotation_step_deg %.4f context=%s" %
                        (delta, max_step, context))
    log("Rotate", "context=%s mode=%s current_r=%.4f command_r=%.4f delta_r=%.4f continuous_allowed=%s" %
        (context,
         str(cfg.get("single_circle_rotation_mode", "continuous")),
         float(current_r),
         float(command_r),
         float(delta),
         str(bool_value(cfg.get("single_circle_allow_continuous_rotation", True)))))
    return delta


def single_circle_rotation_location(nozzle, target, cfg, context):
    target = target.convertToUnits(LengthUnit.Millimeters)
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    command_r = single_circle_command_rotation(current.getRotation(), target.getRotation(), cfg)
    single_circle_check_rotation_step(current.getRotation(), command_r, cfg, context)
    return mm_loc(target.getX(), target.getY(), target.getZ(), command_r)


def single_circle_move_nozzle_split(nozzle, final_loc, cfg, tag, context):
    final_loc = final_loc.convertToUnits(LengthUnit.Millimeters)
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    strategy = base.nozzle_motion_strategy_for_context(cfg, context)
    if strategy == "machine_safe_split":
        clearance_z = float(cfg["safe_travel_z"])
    else:
        clearance_z = max(float(current.getZ()), float(final_loc.getZ()) + base.nozzle_local_clearance_for_context(cfg, context))
    log(tag, "context=%s strategy=%s clearance_z=%.4f" % (context, strategy, clearance_z))

    lift = single_circle_rotation_location(
        nozzle, mm_loc(current.getX(), current.getY(), clearance_z, current.getRotation()),
        cfg, "%sLift" % context)
    log("Move", "phase=lift_to_clearance tag=%s context=%s x=%.4f y=%.4f z=%.4f r=%.4f" %
        (tag, context, lift.getX(), lift.getY(), lift.getZ(), lift.getRotation()))
    base.move_to_with_speed(nozzle, lift, cfg)

    after_lift = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    command_r = single_circle_command_rotation(after_lift.getRotation(), final_loc.getRotation(), cfg)
    single_circle_check_rotation_step(after_lift.getRotation(), command_r, cfg, context)
    if bool_value(cfg.get("split_rotation_from_xy_moves", True)):
        log("Move", "phase=rotate_only tag=%s context=%s x=%.4f y=%.4f z=%.4f r=%.4f" %
            (tag, context, after_lift.getX(), after_lift.getY(), clearance_z, command_r))
        base.move_to_with_speed(nozzle, mm_loc(after_lift.getX(), after_lift.getY(), clearance_z, command_r), cfg)
        after_rotate = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        log("Move", "phase=xy_only tag=%s context=%s x=%.4f y=%.4f z=%.4f r=%.4f" %
            (tag, context, final_loc.getX(), final_loc.getY(), clearance_z, after_rotate.getRotation()))
        base.move_to_with_speed(nozzle, mm_loc(final_loc.getX(), final_loc.getY(), clearance_z, after_rotate.getRotation()), cfg)
    else:
        log("Move", "phase=xy_with_rotation tag=%s context=%s x=%.4f y=%.4f z=%.4f r=%.4f" %
            (tag, context, final_loc.getX(), final_loc.getY(), clearance_z, command_r))
        base.move_to_with_speed(nozzle, mm_loc(final_loc.getX(), final_loc.getY(), clearance_z, command_r), cfg)

    after_xy = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    log("Move", "phase=z_only tag=%s context=%s x=%.4f y=%.4f z=%.4f r=%.4f" %
        (tag, context, after_xy.getX(), after_xy.getY(), final_loc.getZ(), after_xy.getRotation()))
    base.move_to_with_speed(nozzle, mm_loc(after_xy.getX(), after_xy.getY(), final_loc.getZ(), after_xy.getRotation()), cfg)


def single_circle_lift_nozzle_after_pick_place(nozzle, cfg, tag, context, final_z):
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    strategy = base.nozzle_motion_strategy_for_context(cfg, context)
    if strategy == "machine_safe_split":
        z = float(cfg["safe_travel_z"])
    else:
        z = max(float(current.getZ()), float(final_z) + base.nozzle_local_clearance_for_context(cfg, context))
    target = single_circle_rotation_location(
        nozzle, mm_loc(current.getX(), current.getY(), z, current.getRotation()),
        cfg, "%sLift" % context)
    base.move_to_with_speed(nozzle, target, cfg)


def continuous_equivalent_near_current(display_angle, current_r):
    base_angle = float(display_angle)
    k = int(round((float(current_r) - base_angle) / 360.0))
    return base_angle + 360.0 * float(k)


def rotate_xy(x, y, angle_deg):
    return base.rotate_xy(x, y, angle_deg)


def rotate_xy_vector(dx, dy, angle_deg):
    a = math.radians(float(angle_deg))
    ca = math.cos(a)
    sa = math.sin(a)
    return (float(dx) * ca - float(dy) * sa,
            float(dx) * sa + float(dy) * ca)


def corrected_nozzle_rotation_for_die_orientation(desired_die_orientation_deg, square_rotation_bias_deg, cfg):
    # square_rotation_bias_deg:
    # Bottom-camera nozzle/tip square bias. It affects commanded nozzle angle,
    # not the large-circle XY center measurement.
    sign = float(cfg.get("square_bias_apply_sign", 1.0))
    bias = sign * float(square_rotation_bias_deg or 0.0)
    command_r = float(desired_die_orientation_deg) - bias
    log("RotationPlan", "desired_die_orientation_deg=%.5f" % float(desired_die_orientation_deg))
    log("RotationPlan", "square_rotation_bias_deg=%.5f" % float(square_rotation_bias_deg or 0.0))
    log("RotationPlan", "square_bias_apply_sign=%.5f" % sign)
    log("RotationPlan", "commanded_nozzle_rotation_deg=%.5f" % command_r)
    log("RotationPlan", "policy=squared_desired_orientation")
    return command_r


def corrected_nozzle_rotation_for_physical_die_orientation(desired_die_orientation_deg,
                                                           detected_die_orientation_deg,
                                                           current_nozzle_rotation_deg,
                                                           square_rotation_bias_deg,
                                                           cfg):
    # Pick occurs at the current/determined grip rotation. The required held-object
    # rotation is desired marker orientation minus detected marker orientation.
    sign = float(cfg.get("square_bias_apply_sign", 1.0))
    orientation_delta = normalize_angle(float(desired_die_orientation_deg) - float(detected_die_orientation_deg))
    signed_square_bias = sign * float(square_rotation_bias_deg or 0.0)
    raw_command = float(current_nozzle_rotation_deg) + orientation_delta - signed_square_bias
    command_r = normalize_angle(raw_command)
    inverted_raw_command = float(current_nozzle_rotation_deg) - orientation_delta - signed_square_bias
    inverted_command = normalize_angle(inverted_raw_command)
    log("ReturnOrientationPlan", "detected_die_orientation_deg=%.5f" % float(detected_die_orientation_deg))
    log("ReturnOrientationPlan", "desired_storage_orientation_deg=%.5f" % float(desired_die_orientation_deg))
    log("ReturnOrientationPlan", "current_nozzle_rotation_deg=%.5f" % float(current_nozzle_rotation_deg))
    log("ReturnOrientationPlan", "die_orientation_delta_deg=%.5f" % float(orientation_delta))
    log("ReturnOrientationPlan", "square_rotation_bias_deg=%.5f" % float(square_rotation_bias_deg or 0.0))
    log("ReturnOrientationPlan", "square_bias_apply_sign=%.5f" % sign)
    log("ReturnOrientationPlan", "place_rotation_deg=%.5f" % float(command_r))
    log("ReturnOrientationPlan", "inverted_convention_test_place_rotation_deg=%.5f" % float(inverted_command))
    log("ReturnOrientationPlan", "policy=squared_marker_to_desired_storage")
    return command_r


def return_storage_desired_orientation(marker_before_pick_deg, cfg):
    mode = str(cfg.get("return_orientation_mode", "")).strip().lower()
    if mode == "nearest_cardinal":
        return nearest_allowed_orientation(float(marker_before_pick_deg),
                                           cfg.get("return_allowed_orientations_deg", [90.0, 180.0, 270.0]))
    return float(cfg.get("desired_die_storage_orientation_deg", 0.0))


def held_object_return_rotation_command(marker_before_pick_deg, nozzle_rotation_at_pick_deg,
                                        current_nozzle_rotation_deg, cfg):
    desired = return_storage_desired_orientation(marker_before_pick_deg, cfg)
    delta_marker_needed = normalize_angle(float(desired) - float(marker_before_pick_deg))
    convention_sign = float(cfg.get("return_held_object_rotation_sign", -1.0))
    if convention_sign < 0.0:
        raw_command = float(nozzle_rotation_at_pick_deg) - delta_marker_needed
    else:
        raw_command = float(nozzle_rotation_at_pick_deg) + delta_marker_needed
    command_r = continuous_equivalent_near_current(raw_command, float(current_nozzle_rotation_deg))
    log("HeldObjectRotation", "marker_before_pick=%.5f" % float(marker_before_pick_deg))
    log("HeldObjectRotation", "nozzle_rotation_at_pick=%.5f" % float(nozzle_rotation_at_pick_deg))
    log("HeldObjectRotation", "desired_final_marker_angle=%.5f" % float(desired))
    log("HeldObjectRotation", "delta_marker_needed=%.5f" % float(delta_marker_needed))
    log("HeldObjectRotation", "return_held_object_rotation_sign=%.5f" % convention_sign)
    log("HeldObjectRotation", "command_nozzle_rotation=%.5f" % float(command_r))
    log("HeldObjectRotation", "rotation_math=marker_delta_with_machine_convention_sign")
    log("StorageReturn", "rotation_math=marker_delta_with_machine_convention_sign")
    return {"command_r": float(command_r),
            "desired": float(desired),
            "delta_marker_needed": float(delta_marker_needed),
            "return_held_object_rotation_sign": convention_sign,
            "marker_before_pick": float(marker_before_pick_deg),
            "nozzle_rotation_at_pick": float(nozzle_rotation_at_pick_deg),
            "rotation_math": "marker_delta_with_machine_convention_sign"}


def static_candidate_delta_from_computed(computed):
    # persistent_head_offset_delta:
    # Candidate delta for OpenPnP nozzle/head offsets. This changes future
    # coordinate transforms; it is not a same-run jog for a fresh visual lock.
    candidates = [
        ("static_head_offset_delta", "static_head_offset_delta_x_mm", "static_head_offset_delta_y_mm"),
        ("selected_head_delta", "selected_head_delta_x_mm", "selected_head_delta_y_mm"),
        ("nozzle_head_offset_delta", "nozzle_head_offset_delta_x_mm", "nozzle_head_offset_delta_y_mm")
    ]
    source = "none"
    dx = 0.0
    dy = 0.0
    for name, x_key, y_key in candidates:
        if x_key in computed and y_key in computed:
            dx = float(computed.get(x_key, 0.0))
            dy = float(computed.get(y_key, 0.0))
            source = name
            break
    if source == "none":
        for name in ["persistent_head_offset_delta",
                     "virtual_static_delta",
                     "pairwise_average_pair_offset",
                     "average_pair_offset"]:
            data = computed.get(name, None)
            if isinstance(data, dict) and "dx" in data and "dy" in data:
                dx = float(data.get("dx", 0.0))
                dy = float(data.get("dy", 0.0))
                source = name
                break
            if name == "virtual_static_delta" and \
               "virtual_static_delta_x_mm" in computed and "virtual_static_delta_y_mm" in computed:
                dx = float(computed.get("virtual_static_delta_x_mm", 0.0))
                dy = float(computed.get("virtual_static_delta_y_mm", 0.0))
                source = name
                break
    return {"dx": dx, "dy": dy, "mag": math.sqrt(dx * dx + dy * dy),
            "static_dx": dx, "static_dy": dy,
            "angle_dx": 0.0, "angle_dy": 0.0,
            "candidate_delta_source": source}


def zero_same_run_visual_pick_jog(reason):
    # same_run_visual_pick_jog:
    # Direct XY jog for cases without a reliable fresh top-camera lock.
    # With a fresh lock, the detected nozzle target is already the physical pick target.
    return {"dx": 0.0, "dy": 0.0, "mag": 0.0,
            "static_dx": 0.0, "static_dy": 0.0,
            "angle_dx": 0.0, "angle_dy": 0.0,
            "reason": str(reason)}


def refresh_candidate_head_offsets_after_static_delta(nozzle, computed):
    old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
    dx = float(computed.get("static_head_offset_delta_x_mm", 0.0))
    dy = float(computed.get("static_head_offset_delta_y_mm", 0.0))
    computed["candidate_head_offsets_after"] = {
        "x": float(old.getX()) + dx,
        "y": float(old.getY()) + dy,
        "z": float(old.getZ()),
        "rotation": float(old.getRotation())}
    computed["selected_head_delta_x_mm"] = dx
    computed["selected_head_delta_y_mm"] = dy
    computed["selected_head_delta_mag_mm"] = math.sqrt(dx * dx + dy * dy)
    computed["nozzle_head_offset_delta_x_mm"] = dx
    computed["nozzle_head_offset_delta_y_mm"] = dy
    computed["nozzle_head_offset_delta_mag_mm"] = math.sqrt(dx * dx + dy * dy)
    computed["static_head_offset_delta_mag_mm"] = math.sqrt(dx * dx + dy * dy)
    computed["persistent_head_offset_delta"] = {"dx": dx, "dy": dy,
                                                "mag": math.sqrt(dx * dx + dy * dy)}
    return computed


def rotate_static_vector_after_square(nozzle, computed, bottom_square, cfg):
    if computed is None:
        return computed
    bias = bottom_square.get("square_rotation_bias_deg",
                             bottom_square.get("selected_correction_deg", 0.0))
    if bias is None:
        bias = 0.0
    pre_dx = float(computed.get("static_head_offset_delta_x_mm",
                                computed.get("selected_head_delta_x_mm", 0.0)))
    pre_dy = float(computed.get("static_head_offset_delta_y_mm",
                                computed.get("selected_head_delta_y_mm", 0.0)))
    signed_bias = float(cfg.get("square_bias_apply_sign", 1.0)) * float(bias)
    post_dx, post_dy = rotate_xy_vector(pre_dx, pre_dy, signed_bias)
    computed["static_head_offset_delta_x_mm_pre_square"] = pre_dx
    computed["static_head_offset_delta_y_mm_pre_square"] = pre_dy
    computed["static_head_offset_delta_x_mm"] = post_dx
    computed["static_head_offset_delta_y_mm"] = post_dy
    computed["static_head_offset_rotated_by_square_bias_deg"] = signed_bias
    computed["square_rotation_bias_deg"] = float(bias)
    computed["square_bias_apply_sign"] = float(cfg.get("square_bias_apply_sign", 1.0))
    computed["square_rotation_bias_applied_to_static_vector"] = True
    log("VectorRotate", "pre_static_delta=(%.5f, %.5f)" % (pre_dx, pre_dy))
    log("VectorRotate", "square_bias_deg=%.5f" % signed_bias)
    log("VectorRotate", "post_static_delta=(%.5f, %.5f)" % (post_dx, post_dy))
    refresh_candidate_head_offsets_after_static_delta(nozzle, computed)
    return computed


def pick_rotation_policy_for_purpose(cfg, purpose):
    purpose = str(purpose).lower()
    if purpose == "calibration":
        return str(cfg.get("calibration_pick_rotation_policy", "preserve_current_nozzle_rotation")).lower()
    if purpose == "return":
        return str(cfg.get("return_pick_rotation_policy", "preserve_current_nozzle_rotation")).lower()
    if purpose == "confirm":
        return str(cfg.get("confirm_rotation_policy", "preserve_current_nozzle_rotation")).lower()
    if purpose == "storage_initial":
        if bool_value(cfg.get("single_circle_use_current_rotation_for_pick", False)):
            return "preserve_current_nozzle_rotation"
        return "configured"
    return "preserve_current_nozzle_rotation"


def determine_pick_rotation(nozzle, base_loc, pose, cfg, purpose):
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    current_r = float(current.getRotation())
    detected = float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0)))
    policy = pick_rotation_policy_for_purpose(cfg, purpose)
    if policy == "squared_marker_to_desired_storage":
        command_r = continuous_equivalent_near_current(current_r, current_r)
    elif policy == "preserve_current_nozzle_rotation":
        command_r = current_r
    elif policy == "match_detected_marker":
        command_r = continuous_equivalent_near_current(detected, current_r)
    elif policy == "configured":
        command_r = float(base_loc.getRotation())
    else:
        raise Exception("Invalid pick rotation policy %s for purpose %s" % (policy, purpose))
    log("PickRotation", "purpose=%s" % str(purpose))
    log("PickRotation", "policy=%s" % policy)
    log("PickRotation", "detected_die_orientation_deg=%.4f" % detected)
    log("PickRotation", "current_nozzle_rotation=%.4f" % current_r)
    log("PickRotation", "commanded_nozzle_rotation=%.4f" % command_r)
    return command_r


def loc_with_measured_orientation(measured_loc, pose):
    measured = measured_loc.convertToUnits(LengthUnit.Millimeters)
    return mm_loc(measured.getX(), measured.getY(), measured.getZ(),
                  float(pose.get("die_orientation_deg", measured.getRotation())))


def compute_return_rotation_command(nozzle, pose, cfg, orientation_reference=None):
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    current_r = float(current.getRotation())
    current_die_orientation = float(pose["die_orientation_deg"])
    allowed = cfg.get("return_allowed_orientations_deg", [90.0, 180.0, 270.0])
    desired_die_orientation = nearest_allowed_orientation(current_die_orientation, allowed)
    delta = normalize_angle(desired_die_orientation - current_die_orientation)
    command_r = current_r + delta
    plan = {
        "current_nozzle_rotation_deg": current_r,
        "current_die_orientation_deg": current_die_orientation,
        "desired_die_orientation_deg": desired_die_orientation,
        "die_rotation_delta_deg": delta,
        "return_nozzle_command_deg": command_r
    }
    if orientation_reference is not None:
        plan["orientation_reference"] = str(orientation_reference)
        log("StorageReturn", "orientation_reference=%s" % str(orientation_reference))
    log("StorageReturn", "current_die_orientation_deg=%.5f" % current_die_orientation)
    log("StorageReturn", "desired_die_orientation_deg=%.5f" % desired_die_orientation)
    log("StorageReturn", "die_rotation_delta_deg=%.5f" % delta)
    log("StorageReturn", "current_nozzle_rotation_deg=%.5f" % current_r)
    log("StorageReturn", "return_nozzle_command_deg=%.5f" % command_r)
    log("StorageReturn", "rotation_math=relative_marker_to_allowed_orientation")
    return plan


def compute_return_rotation_command_from_held_object(nozzle, pose, cfg, state, orientation_reference=None):
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    current_r = float(current.getRotation())
    marker_before_pick = float(state.get("held_die_marker_angle_before_pick",
                                         pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0))))
    nozzle_rotation_at_pick = float(state.get("nozzle_rotation_at_pick", current_r))
    marker_to_nozzle_offset = normalize_angle(marker_before_pick - nozzle_rotation_at_pick)
    allowed = cfg.get("return_allowed_orientations_deg", [90.0, 180.0, 270.0])
    desired_die_orientation = nearest_allowed_orientation(marker_before_pick, allowed)
    raw_command = float(desired_die_orientation) - float(marker_to_nozzle_offset)
    command_r = continuous_equivalent_near_current(raw_command, current_r)
    delta = float(command_r) - current_r
    plan = {
        "current_nozzle_rotation_deg": current_r,
        "current_die_orientation_deg": marker_before_pick,
        "desired_die_orientation_deg": desired_die_orientation,
        "die_rotation_delta_deg": delta,
        "return_nozzle_command_deg": command_r,
        "marker_before_pick_deg": marker_before_pick,
        "nozzle_rotation_at_pick_deg": nozzle_rotation_at_pick,
        "marker_to_nozzle_offset_deg": marker_to_nozzle_offset,
        "rotation_math": "held_marker_to_nozzle_offset"
    }
    if orientation_reference is not None:
        plan["orientation_reference"] = str(orientation_reference)
        log("StorageReturn", "orientation_reference=%s" % str(orientation_reference))
    log("HeldObjectRotation", "marker_before_pick=%.5f" % marker_before_pick)
    log("HeldObjectRotation", "nozzle_rotation_at_pick=%.5f" % nozzle_rotation_at_pick)
    log("HeldObjectRotation", "marker_to_nozzle_offset=%.5f" % marker_to_nozzle_offset)
    log("HeldObjectRotation", "desired_final_marker_angle=%.5f" % desired_die_orientation)
    log("HeldObjectRotation", "command_nozzle_rotation=%.5f" % command_r)
    log("ReturnRotateAfterPick", "marker_orientation_before_pick=%.5f" % marker_before_pick)
    log("ReturnRotateAfterPick", "desired_storage_orientation=%.5f" % desired_die_orientation)
    log("ReturnRotateAfterPick", "command_rotation=%.5f" % command_r)
    log("StorageReturn", "rotation_math=held_marker_to_nozzle_offset")
    return plan


def virtual_correction_for_rotation(computed, rotation_deg, cfg, context):
    if computed is not None and "model_center_x_mm" in computed and "model_center_y_mm" in computed:
        delta = command_delta_for_rotation(computed, rotation_deg, cfg)
        log("Correction", "context=%s model_mode=%s" %
            (str(context), str(computed.get("single_circle_model_mode", computed.get("model", "openpnp_model_camera_offset")))))
        log("Correction", "static_delta=(%.5f,%.5f)" %
            (float(delta.get("static_dx", 0.0)), float(delta.get("static_dy", 0.0))))
        log("Correction", "angle_delta=(%.5f,%.5f)" %
            (float(delta.get("angle_dx", 0.0)), float(delta.get("angle_dy", 0.0))))
        log("Correction", "total=(%.5f,%.5f,%.5f)" %
            (float(delta["dx"]), float(delta["dy"]), float(delta["mag"])))
        return delta
    dx = 0.0
    dy = 0.0
    static_dx = 0.0
    static_dy = 0.0
    angle_dx = 0.0
    angle_dy = 0.0
    context = str(context)
    include_static = bool_value(cfg.get("single_circle_preview_include_static_delta", True))
    include_angle = bool_value(cfg.get("single_circle_preview_include_residual_angle_vector", True))

    if context == "return_pick":
        include_angle = bool_value(cfg.get("return_pick_include_residual_angle_vector", False))
    elif context == "confirm":
        include_angle = bool_value(cfg.get("confirm_include_residual_angle_vector", False))
    elif context in ["verification", "result_summary"]:
        include_angle = bool_value(cfg.get("calibration_fit_include_residual_angle_vector", True))

    if include_static:
        static_dx = float(computed.get("selected_head_delta_x_mm", 0.0))
        static_dy = float(computed.get("selected_head_delta_y_mm", 0.0))
        dx += static_dx
        dy += static_dy

    if include_angle:
        vx = float(computed.get("residual_angle_vector_x_mm", 0.0))
        vy = float(computed.get("residual_angle_vector_y_mm", 0.0))

        rotated = rotate_xy(vx, vy, rotation_deg)
        angle_dx = -float(rotated["x"])
        angle_dy = -float(rotated["y"])
        dx += angle_dx
        dy += angle_dy

    log("Correction", "context=%s include_static=%s include_angle_vector=%s" %
        (context, str(bool(include_static)), str(bool(include_angle))))
    log("Correction", "static_delta=(%.5f,%.5f)" % (static_dx, static_dy))
    log("Correction", "angle_delta=(%.5f,%.5f)" % (angle_dx, angle_dy))
    log("Correction", "total=(%.5f,%.5f,%.5f)" %
        (dx, dy, math.sqrt(dx * dx + dy * dy)))
    return {"dx": dx, "dy": dy, "mag": math.sqrt(dx * dx + dy * dy),
            "static_dx": static_dx, "static_dy": static_dy,
            "angle_dx": angle_dx, "angle_dy": angle_dy}


def confirm_target_z(anchor_loc, cfg):
    return pick_surface_z_mm(anchor_loc, cfg) + float(cfg.get("single_circle_confirm_surface_clearance_mm", 0.0))


def move_nozzle_confirm_preview(nozzle, final_loc, cfg):
    final_loc = final_loc.convertToUnits(LengthUnit.Millimeters)
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    safe_z = float(cfg["safe_travel_z"])
    lift = single_circle_rotation_location(nozzle, mm_loc(current.getX(), current.getY(), safe_z, current.getRotation()), cfg, "ConfirmLift")
    log("Move", "phase=lift_to_safe_z tag=Confirm context=Confirm x=%.4f y=%.4f z=%.4f r=%.4f" %
        (lift.getX(), lift.getY(), lift.getZ(), lift.getRotation()))
    base.move_to_with_speed(nozzle, lift, cfg)

    after_lift = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    rotate = single_circle_rotation_location(nozzle, mm_loc(after_lift.getX(), after_lift.getY(), safe_z, final_loc.getRotation()), cfg, "ConfirmRotate")
    log("Move", "phase=rotate_to_target_r tag=Confirm context=Confirm x=%.4f y=%.4f z=%.4f r=%.4f" %
        (rotate.getX(), rotate.getY(), rotate.getZ(), rotate.getRotation()))
    base.move_to_with_speed(nozzle, rotate, cfg)

    after_rotate = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    xy = mm_loc(final_loc.getX(), final_loc.getY(), safe_z, after_rotate.getRotation())
    log("Move", "phase=xy_at_safe_z tag=Confirm context=Confirm x=%.4f y=%.4f z=%.4f r=%.4f" %
        (xy.getX(), xy.getY(), xy.getZ(), xy.getRotation()))
    base.move_to_with_speed(nozzle, xy, cfg)

    after_xy = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    z = mm_loc(after_xy.getX(), after_xy.getY(), final_loc.getZ(), after_xy.getRotation())
    log("Move", "phase=descend_to_confirm_z tag=Confirm context=Confirm x=%.4f y=%.4f z=%.4f r=%.4f" %
        (z.getX(), z.getY(), z.getZ(), z.getRotation()))
    base.move_to_with_speed(nozzle, z, cfg)


def do_pick(nozzle, part, cfg, expected_pick_z=None):
    if expected_pick_z is not None:
        actual = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        passed = abs(float(actual.getZ()) - float(expected_pick_z)) <= 0.05
        log("PickZGuard", "actual_z=%.5f expected_z=%.5f passed=%s" %
            (float(actual.getZ()), float(expected_pick_z), str(bool(passed))))
        if not passed:
            raise Exception("Pick Z mismatch: actual %.5f expected %.5f" %
                            (float(actual.getZ()), float(expected_pick_z)))

    set_vacuum_state(nozzle, cfg, True, True)
    sleep_ms(cfg.get("vacuum_settle_ms", 150))

    if bool_value(cfg.get("use_high_level_nozzle_pick", True)):
        try:
            before_high = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            log("Pick", "calling_high_level_nozzle_pick=True")
            log("Pick", "before_high_level_pick_location=(x=%.5f,y=%.5f,z=%.5f,r=%.5f)" %
                (float(before_high.getX()), float(before_high.getY()), float(before_high.getZ()), float(before_high.getRotation())))
            nozzle.pick(part, None)
            after_high = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            log("Pick", "after_high_level_pick_location=(x=%.5f,y=%.5f,z=%.5f,r=%.5f)" %
                (float(after_high.getX()), float(after_high.getY()), float(after_high.getZ()), float(after_high.getRotation())))
            log("Pick", "nozzle.pick(part, None) succeeded")
            return
        except Throwable, t:
            log("Pick", "nozzle.pick failed; continuing with vacuum and part state fallback: %s" % t)
        except Exception, e:
            log("Pick", "nozzle.pick failed; continuing with vacuum and part state fallback: %s" % e)

    try:
        nozzle.setPart(part)
        log("Pick", "nozzle.setPart(part) succeeded")
    except Exception, e:
        log("Pick", "WARNING nozzle.setPart(part) failed: %s" % e)


def call_high_level_place(nozzle):
    try:
        nozzle.place()
        log("Place", "nozzle.place() succeeded")
        return True
    except Throwable, t:
        log("Place", "nozzle.place() unavailable/failed: %s" % t)
    except Exception, e:
        log("Place", "nozzle.place() unavailable/failed: %s" % e)
    try:
        nozzle.place(None)
        log("Place", "nozzle.place(None) succeeded")
        return True
    except Throwable, t2:
        log("Place", "nozzle.place(None) unavailable/failed: %s" % t2)
    except Exception, e2:
        log("Place", "nozzle.place(None) unavailable/failed: %s" % e2)
    return False


def do_place(nozzle, cfg, expected_place_z=None):
    if expected_place_z is not None:
        actual = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        passed = abs(float(actual.getZ()) - float(expected_place_z)) <= 0.05
        log("PlaceZGuard", "actual_z=%.5f expected_z=%.5f passed=%s" %
            (float(actual.getZ()), float(expected_place_z), str(bool(passed))))
        if not passed:
            raise Exception("Place Z mismatch: actual %.5f expected %.5f" %
                            (float(actual.getZ()), float(expected_place_z)))

    set_vacuum_state(nozzle, cfg, False, False)

    if bool_value(cfg.get("enable_blowoff_on_place", False)):
        if set_blowoff_state(nozzle, cfg, True):
            sleep_ms(cfg.get("blowoff_pulse_ms", 120))
            set_blowoff_state(nozzle, cfg, False)
    try:
        nozzle.setPart(None)
    except:
        pass


def pick_delta_components(pick_delta):
    dx = 0.0
    dy = 0.0
    if pick_delta is not None:
        dx = float(pick_delta.get("dx", 0.0))
        dy = float(pick_delta.get("dy", 0.0))
    return dx, dy, math.sqrt(dx * dx + dy * dy)


def pick_correction_source(purpose, pick_delta, pick_delta_info):
    if pick_delta is None:
        return "none"
    if pick_delta_info is not None and str(pick_delta_info.get("correction_source", "")).strip() != "":
        return str(pick_delta_info.get("correction_source"))
    if str(purpose).lower() == "return":
        return "active_post_bottom_square_fit"
    if str(purpose).lower() == "confirm":
        return "confirm_preview"
    return "none"


def log_pick_target(raw_target, dx, dy, final_target, lock_reused, correction_source):
    mag = math.sqrt(dx * dx + dy * dy)
    log("PickTarget", "raw_nozzle_target=(%.5f,%.5f)" %
        (float(raw_target.getX()), float(raw_target.getY())))
    log("PickTarget", "applied_delta=(%.5f,%.5f,%.5f)" % (dx, dy, mag))
    log("PickTarget", "final_nozzle_target=(%.5f,%.5f)" %
        (float(final_target.getX()), float(final_target.getY())))
    log("PickTarget", "lock_reused=%s" % str(bool(lock_reused)))
    log("PickTarget", "correction_source=%s" % str(correction_source))
    if mag > 0.000001:
        final_dx = float(final_target.getX() - raw_target.getX())
        final_dy = float(final_target.getY() - raw_target.getY())
        if abs(final_dx) < 0.000001 and abs(final_dy) < 0.000001:
            raise Exception("Pick delta is nonzero but final pick target equals raw target.")


def pick_die_from_existing_lock(camera, nozzle, part, base_loc, cfg, tag, state, lock, detected_measurement_loc, pick_delta, purpose, pick_delta_info=None, lock_reused=True):
    base_loc = force_context_base_z(base_loc, cfg, tag)
    pose = lock["pose"]
    if int(pose.get("circle_count", 0)) != 2:
        raise Exception("[VisionLock][%s] pick aborted: both circles are required." % tag)
    nozzle_target = lock["nozzle_target_location"]
    dx, dy, delta_mag = pick_delta_components(pick_delta)
    lock_source = "existing_lock" if bool(lock_reused) else "fresh_acquire"
    fresh_visual_lock = False
    if pick_delta_info is not None:
        fresh_visual_lock = bool_value(pick_delta_info.get("fresh_visual_lock", False))
    if str(purpose).lower() == "return" and (lock_source in ["fresh_acquire", "existing_lock"] or fresh_visual_lock):
        if delta_mag > 0.000001 and not bool_value(cfg.get("allow_static_delta_as_same_run_pick_jog", False)):
            raise Exception("same_run_visual_pick_jog must be zero for a fresh visual lock unless allow_static_delta_as_same_run_pick_jog is true.")
    pick_rotation = determine_pick_rotation(nozzle, base_loc, pose, cfg, purpose)
    pick_z = resolve_nozzle_pick_z(base_loc, cfg)
    log("BaseZGuard", "context=Pick")
    log("BaseZGuard", "base_loc_z_after=%.5f" % float(base_loc.getZ()))
    log("BaseZGuard", "pick_surface_z=%.5f" % float(base.pick_surface_z_mm(base_loc, cfg)))
    guard_pick_place_z(base_loc.getZ(), pick_z, cfg, "Pick")
    log_nozzle_pick_z_plan(base_loc, cfg, pick_z)
    pick_target = mm_loc(nozzle_target.getX() + dx, nozzle_target.getY() + dy, pick_z, pick_rotation)
    log("VisionLockUse", "source=%s" % lock_source)
    log("VisionLockUse", "reacquire_before_pick=False")
    log("VisionLockUse", "lock_center=(%.5f,%.5f)" %
        (float(nozzle_target.getX()), float(nozzle_target.getY())))
    log("VisionLockUse", "final_pick_target=(%.5f,%.5f)" %
        (float(pick_target.getX()), float(pick_target.getY())))
    log_pick_target(nozzle_target, dx, dy, pick_target, bool(lock_reused),
                    pick_correction_source(purpose, pick_delta, pick_delta_info))
    if str(purpose).lower() == "return":
        pre_dx = 0.0
        pre_dy = 0.0
        active_dx = dx
        active_dy = dy
        mode = "fresh_center_only"
        if pick_delta_info is not None:
            pre_dx = float(pick_delta_info.get("pre_square_dx", 0.0))
            pre_dy = float(pick_delta_info.get("pre_square_dy", 0.0))
            active_dx = float(pick_delta_info.get("active_fit_dx", dx))
            active_dy = float(pick_delta_info.get("active_fit_dy", dy))
            mode = str(pick_delta_info.get("mode", mode))
        log("StorageReturnPick", "raw_fresh_detected_target=(%.5f, %.5f)" %
            (float(nozzle_target.getX()), float(nozzle_target.getY())))
        if pick_delta_info is not None:
            log("StorageReturnPick", "fresh_visual_lock=%s" %
                str(bool_value(pick_delta_info.get("fresh_visual_lock", True))))
            log("StorageReturnPick", "persistent_head_offset_delta_not_used_as_same_run_jog=%s" %
                str(bool_value(pick_delta_info.get("persistent_head_offset_delta_not_used_as_same_run_jog", False))))
            log("StorageReturnPick", "using_rejected_calibration_delta=%s" %
                str(bool_value(pick_delta_info.get("using_rejected_calibration_delta", False))))
        log("StorageReturnPick", "pre_square_virtual_delta=(%.5f, %.5f)" %
            (pre_dx, pre_dy))
        log("StorageReturnPick", "active_post_bottom_square_delta=(%.5f, %.5f)" %
            (active_dx, active_dy))
        log("StorageReturnPick", "post_square_pick_delta_mode=%s" % mode)
        log("StorageReturnPick", "applied_pick_delta=(%.5f, %.5f)" % (dx, dy))
        log("StorageReturnPick", "final_pick_target=(%.5f, %.5f)" %
            (float(pick_target.getX()), float(pick_target.getY())))
    log("Pick", "%s detected_measurement_center=(%.5f, %.5f)" %
        (tag, detected_measurement_loc.getX(), detected_measurement_loc.getY()))
    log("Pick", "%s nozzle_pick_target=(%.5f, %.5f)" %
        (tag, pick_target.getX(), pick_target.getY()))
    log("Pick", "purpose=%s" % str(purpose))
    log("Pick", "detected_die_orientation_deg=%.4f" % float(pose.get("die_orientation_deg", 0.0)))
    log("Pick", "commanded_nozzle_rotation_deg=%.4f" % float(pick_rotation))
    log("Pick", "rotation_policy=%s" % pick_rotation_policy_for_purpose(cfg, purpose))
    log("Pick", "orientation_circle_used_for_center=False")
    if str(purpose).lower() == "return":
        log("ReturnPick", "rotation_policy=%s" % pick_rotation_policy_for_purpose(cfg, purpose))
        log("ReturnPick", "marker_used_for_pick_rotation=%s" %
            str(pick_rotation_policy_for_purpose(cfg, purpose) in ["match_detected_marker", "squared_marker_to_desired_storage"]))
    single_circle_move_nozzle_split(nozzle, single_circle_rotation_location(nozzle, pick_target, cfg, "Pick"), cfg, "Pick", tag)
    do_pick(nozzle, part, cfg, pick_z)
    base.verify_part_on_after_pick(nozzle)
    state["die_on_nozzle"] = True
    state["current_die_loc_trusted"] = False
    actual = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    state["last_pick_rotation_deg"] = normalize_angle(actual.getRotation())
    state["held_die_marker_angle_before_pick"] = float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0)))
    state["nozzle_rotation_at_pick"] = float(actual.getRotation())
    state["held_marker_to_nozzle_offset"] = normalize_angle(
        float(state["held_die_marker_angle_before_pick"]) - float(state["nozzle_rotation_at_pick"]))
    single_circle_lift_nozzle_after_pick_place(nozzle, cfg, "Pick", tag, pick_z)
    return lock, detected_measurement_loc


def pick_die_with_visual_lock(camera, nozzle, part, base_loc, cfg, tag, state, pick_delta, purpose, pick_delta_info=None):
    lock, detected_measurement_loc = acquire_pose_for_location(camera, nozzle, part, base_loc, cfg, tag)
    return pick_die_from_existing_lock(camera, nozzle, part, base_loc, cfg, tag, state,
                                       lock, detected_measurement_loc, pick_delta, purpose, pick_delta_info, False)


def pick_die_from_lock_with_rotation(nozzle, part, base_loc, cfg, tag, state, lock, detected_measurement_loc, pick_rotation, purpose):
    base_loc = force_context_base_z(base_loc, cfg, tag)
    pose = lock["pose"]
    if int(pose.get("circle_count", 0)) != 2:
        raise Exception("[VisionLock][%s] pick aborted: both circles are required." % tag)
    nozzle_target = lock["nozzle_target_location"]
    pick_z = resolve_nozzle_pick_z(base_loc, cfg)
    guard_pick_place_z(base_loc.getZ(), pick_z, cfg, "Pick")
    log_nozzle_pick_z_plan(base_loc, cfg, pick_z)
    pick_target = mm_loc(nozzle_target.getX(), nozzle_target.getY(), pick_z, float(pick_rotation))
    log("Pick", "%s detected_measurement_center=(%.5f, %.5f)" %
        (tag, detected_measurement_loc.getX(), detected_measurement_loc.getY()))
    log("Pick", "%s nozzle_pick_target=(%.5f, %.5f)" %
        (tag, pick_target.getX(), pick_target.getY()))
    log("Pick", "purpose=%s" % str(purpose))
    log("Pick", "commanded_nozzle_rotation_deg=%.4f" % float(pick_rotation))
    log("Pick", "rotation_policy=explicit_openpnp_pairwise_180")
    single_circle_move_nozzle_split(nozzle, single_circle_rotation_location(nozzle, pick_target, cfg, "Pick"), cfg, "Pick", tag)
    do_pick(nozzle, part, cfg, pick_z)
    base.verify_part_on_after_pick(nozzle)
    state["die_on_nozzle"] = True
    state["current_die_loc_trusted"] = False
    actual = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    state["last_pick_rotation_deg"] = normalize_angle(actual.getRotation())
    state["held_die_marker_angle_before_pick"] = float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0)))
    state["nozzle_rotation_at_pick"] = float(actual.getRotation())
    state["held_marker_to_nozzle_offset"] = normalize_angle(
        float(state["held_die_marker_angle_before_pick"]) - float(state["nozzle_rotation_at_pick"]))
    single_circle_lift_nozzle_after_pick_place(nozzle, cfg, "Pick", tag, pick_z)
    return lock, detected_measurement_loc


def place_die_at(nozzle, part, target_loc, cfg, tag, state):
    target_loc = force_context_base_z(target_loc, cfg, tag)
    place_z = resolve_nozzle_place_z(target_loc, cfg)
    log("BaseZGuard", "context=Place")
    log("BaseZGuard", "base_loc_z_after=%.5f" % float(target_loc.getZ()))
    log("BaseZGuard", "place_surface_z=%.5f" % float(base.pick_surface_z_mm(target_loc, cfg)))
    guard_pick_place_z(target_loc.getZ(), place_z, cfg, "Place")
    log_nozzle_place_z_plan(target_loc, cfg, place_z)
    place_target = single_circle_rotation_location(
        nozzle, mm_loc(target_loc.getX(), target_loc.getY(), place_z, target_loc.getRotation()),
        cfg, "Place")
    log("Place", "%s command=(%.5f, %.5f, %.5f)" %
        (tag, place_target.getX(), place_target.getY(), place_target.getRotation()))
    single_circle_move_nozzle_split(nozzle, place_target, cfg, "Place", tag)
    do_place(nozzle, cfg, place_z)
    sleep_ms(cfg.get("blowoff_pulse_ms", 120))
    base.verify_part_off_after_place(nozzle)
    state["die_on_nozzle"] = False
    state["die_at_work"] = (tag == "Cal")
    if tag == "Cal":
        state["current_die_loc"] = target_loc
        state["current_die_loc_trusted"] = False
        log("DieState", "after_place trusted=False reason=placed_without_vision")
    single_circle_lift_nozzle_after_pick_place(nozzle, cfg, "Place", tag, place_z)
    return target_loc


def transfer_die_from_storage_to_cal(camera, nozzle, part, storage_loc, cal_loc, cfg, state):
    if bool_value(cfg.get("single_circle_use_current_rotation_for_pick", False)):
        current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        pick_r = float(current.getRotation())
    else:
        pick_r = float(cfg.get("single_circle_initial_pick_rotation_deg", 0.0))
    log("Pick", "initial_storage_pick_rotation=%.4f use_current=%s" %
        (pick_r, str(bool_value(cfg.get("single_circle_use_current_rotation_for_pick", False)))))
    storage_pick = mm_loc(storage_loc.getX(), storage_loc.getY(), storage_loc.getZ(), pick_r)
    lock, detected_storage = pick_die_with_visual_lock(camera, nozzle, part, storage_pick, cfg, "Storage", state, None, "storage_initial")
    place_die_at(nozzle, part, mm_loc(cal_loc.getX(), cal_loc.getY(), cal_loc.getZ(), 0.0), cfg, "Cal", state)
    cal_lock, actual = acquire_pose_for_location(camera, nozzle, part, cal_loc, cfg, "Cal")
    state["current_die_loc"] = base_loc_with_measured_orientation(actual, cal_lock["pose"], "Cal", cfg)
    state["current_die_loc_trusted"] = True
    state["last_trusted_die_loc"] = state["current_die_loc"]
    return lock, detected_storage, cal_lock


def generated_angles(cfg):
    start = float(cfg["angle_start_deg"])
    stop = float(cfg["angle_stop_deg"])
    subdivisions = int(cfg["angle_subdivisions"])
    span = stop - start
    full_circle = abs(abs(span) - 360.0) < 0.000001
    out = []
    if full_circle:
        step = span / float(subdivisions)
        for i in range(subdivisions):
            out.append(start + step * float(i))
    else:
        if subdivisions == 1:
            out.append(start)
            out.append(stop)
        else:
            step = span / float(subdivisions)
            for i in range(subdivisions + 1):
                out.append(start + step * float(i))
    return out


def xy_distance_loc(a, b):
    a = a.convertToUnits(LengthUnit.Millimeters)
    b = b.convertToUnits(LengthUnit.Millimeters)
    dx = float(a.getX()) - float(b.getX())
    dy = float(a.getY()) - float(b.getY())
    return dx, dy, math.sqrt(dx * dx + dy * dy)


def resolve_cal_acquire_anchor(cal_loc, cfg, state, pairwise_context, context):
    cal_loc = cal_loc.convertToUnits(LengthUnit.Millimeters)
    anchor = cal_loc
    anchor_source = "nominal_cal_location"
    use_trusted_priority = bool(pairwise_context)
    if use_trusted_priority and bool_value(state.get("current_die_loc_trusted", False)) and \
       state.get("current_die_loc", None) is not None:
        anchor = state.get("current_die_loc").convertToUnits(LengthUnit.Millimeters)
        anchor_source = "state_current_die_loc"
    elif use_trusted_priority and state.get("last_trusted_die_loc", None) is not None:
        anchor = state.get("last_trusted_die_loc").convertToUnits(LengthUnit.Millimeters)
        anchor_source = "state_last_trusted_die_loc"
    elif bool_value(cfg.get("single_circle_calibration_acquire_from_nominal_xy", True)):
        anchor = cal_loc
        anchor_source = "nominal_cal_location"
    else:
        anchor = state.get("current_die_loc", cal_loc).convertToUnits(LengthUnit.Millimeters)
        if bool_value(state.get("current_die_loc_trusted", False)):
            anchor_source = "state_current_die_loc"
        else:
            anchor_source = "state_current_die_loc_untrusted"
    dx, dy, mag = xy_distance_loc(anchor, cal_loc)
    log("AcquirePolicy", "sample_acquire_anchor=%s" % anchor_source)
    log("AcquirePolicy", "anchor_xy=(%.5f, %.5f)" % (float(anchor.getX()), float(anchor.getY())))
    log("AcquirePolicy", "nominal_cal_xy=(%.5f, %.5f)" % (float(cal_loc.getX()), float(cal_loc.getY())))
    log("AcquirePolicy", "anchor_minus_nominal=(%.5f, %.5f, %.5f)" % (dx, dy, mag))
    if bool(pairwise_context) and bool_value(state.get("current_die_loc_trusted", False)) and \
       anchor_source == "nominal_cal_location":
        guard = float(cfg.get("openpnp_pairwise_nominal_reacquire_guard_mm", 0.25))
        trusted = state.get("current_die_loc", cal_loc).convertToUnits(LengthUnit.Millimeters)
        tdx, tdy, tmag = xy_distance_loc(trusted, cal_loc)
        if tmag > guard:
            raise Exception("Pairwise verification attempted nominal reacquire while walked die location is trusted.")
    return anchor, anchor_source


def is_pairwise_computed(computed):
    if computed is None:
        return False
    return str(computed.get("single_circle_fit_model_type",
                            computed.get("model", ""))).strip().lower() == "openpnp_pairwise_180_offset"


def measure_single_circle_sample(camera, nozzle, part, cal_loc, cfg, state, sample_index, angle,
                                 anchor_loc=None, anchor_source=None, pairwise_context=False):
    if bool(pairwise_context):
        log("AcquirePolicy", "pairwise_override_acquire_from_nominal=False")
        log("AcquirePolicy", "pairwise_override_update_current_die_loc=True")
    if anchor_loc is None:
        current_die_loc, resolved_source = resolve_cal_acquire_anchor(cal_loc, cfg, state, pairwise_context, "sample")
        if anchor_source is None:
            anchor_source = resolved_source
    else:
        current_die_loc = anchor_loc.convertToUnits(LengthUnit.Millimeters)
        if anchor_source is None:
            anchor_source = "explicit_anchor"
        dx, dy, mag = xy_distance_loc(current_die_loc, cal_loc)
        log("AcquirePolicy", "sample_acquire_anchor=%s" % str(anchor_source))
        log("AcquirePolicy", "anchor_xy=(%.5f, %.5f)" %
            (float(current_die_loc.getX()), float(current_die_loc.getY())))
        log("AcquirePolicy", "nominal_cal_xy=(%.5f, %.5f)" %
            (float(cal_loc.getX()), float(cal_loc.getY())))
        log("AcquirePolicy", "anchor_minus_nominal=(%.5f, %.5f, %.5f)" % (dx, dy, mag))
    update_after_measure = bool_value(cfg.get("single_circle_update_current_die_loc_after_measure", False)) or \
        bool(pairwise_context)
    log("AcquirePolicy", "updating_current_die_loc_after_measure=%s" %
        str(bool(update_after_measure)))
    log("SingleCircleCal", "sample index=%d" % int(sample_index))
    log("SingleCircleCal", "angle=%.5f" % float(angle))
    pre_lock, pre_loc = acquire_pose_for_location(camera, nozzle, part, current_die_loc, cfg, "Cal")
    pre_base = measured_xy_as_base_loc(pre_loc, "Cal", cfg)
    pick_base = mm_loc(pre_base.getX(), pre_base.getY(), pre_base.getZ(), current_die_loc.getRotation())
    cal_policy = pick_rotation_policy_for_purpose(cfg, "calibration")
    log("SingleCircleCal", "marker_angle_used_for_pick=%s" % str(cal_policy == "match_detected_marker"))
    if cal_policy == "match_detected_marker":
        log("SingleCircleCal", "WARNING calibration pickup matches marker then rotates to sample angle; this causes two rotations by design.")
    pick_die_from_existing_lock(camera, nozzle, part, pick_base, cfg, "Cal", state,
                                pre_lock, pre_loc, None, "calibration")
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    rotate_target = single_circle_rotation_location(
        nozzle, mm_loc(current.getX(), current.getY(), current.getZ(), float(angle)),
        cfg, "SingleCircleCalRotateWithObject")
    command_r = float(rotate_target.getRotation())
    log("Rotate", "purpose=SingleCircleCalRotateWithObject")
    log("Rotate", "commanded_nozzle_rotation_deg=%.5f" % command_r)
    log("SingleCircleCal", "sample_target_rotation=%.5f" % float(angle))
    log("SingleCircleCal", "rotate_with_object from_r=%.5f to_r=%.5f" %
        (float(current.getRotation()), float(command_r)))
    base.move_to_with_speed(nozzle, rotate_target, cfg)
    place_r = float(angle)
    if bool(pairwise_context):
        sample_target = mm_loc(current_die_loc.getX(), current_die_loc.getY(), current_die_loc.getZ(), place_r)
    else:
        sample_target = mm_loc(cal_loc.getX(), cal_loc.getY(), cal_loc.getZ(), place_r)
    place_die_at(nozzle, part, sample_target, cfg, "Cal", state)
    lock, measured = acquire_pose_for_location(camera, nozzle, part, sample_target, cfg, "Cal")
    pose = lock["pose"]
    if float(pose["center_x_px"]) != float(pose["center_circle_px"]["x_px"]):
        raise Exception("Sample center_x_px no longer equals large center circle x.")
    if float(pose["center_y_px"]) != float(pose["center_circle_px"]["y_px"]):
        raise Exception("Sample center_y_px no longer equals large center circle y.")
    error_x = float(measured.getX() - cal_loc.getX())
    error_y = float(measured.getY() - cal_loc.getY())
    error_mag = math.sqrt(error_x * error_x + error_y * error_y)
    sample_loc = mm_loc(error_x, error_y, 0.0, angle)
    sample = {
        "sample_index": int(sample_index),
        "command_angle_deg": float(angle),
        "display_angle_deg": normalize_angle(angle),
        "commanded_nozzle_rotation_deg": float(command_r),
        "measured_location": location_to_diag(sample_loc),
        "measured_center_x_mm": float(measured.getX()),
        "measured_center_y_mm": float(measured.getY()),
        "nominal_center_x_mm": float(cal_loc.getX()),
        "nominal_center_y_mm": float(cal_loc.getY()),
        "error_x_mm": error_x,
        "error_y_mm": error_y,
        "error_mag_mm": error_mag,
        "center_circle_px": pose["center_circle_px"],
        "orientation_circle_px": pose["orientation_circle_px"],
        "orientation_used_for_xy": False,
        "orientation_raw_deg": float(pose.get("orientation_raw_deg", 0.0)),
        "orientation_raw_marker_deg": float(pose.get("orientation_raw_marker_deg", pose.get("orientation_raw_deg", 0.0))),
        "die_orientation_deg": float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0))),
        "search_dx_mm": float(pose.get("search_dx_mm", 0.0)),
        "search_dy_mm": float(pose.get("search_dy_mm", 0.0)),
        "sample_valid": True,
        "sample_rejected_reason": None
    }
    log("SingleCircleCal", "measured_center=(%.5f, %.5f)" % (measured.getX(), measured.getY()))
    log("SingleCircleCal", "error=(%.5f, %.5f)" % (error_x, error_y))
    log("SampleFrame", "command_angle_deg=%.5f" % float(angle))
    log("SampleFrame", "marker_angle_diag_deg=%.5f" %
        float(pose.get("orientation_raw_marker_deg", pose.get("orientation_raw_deg", 0.0))))
    log("SampleFrame", "marker_used_for_model_fit=False")
    if update_after_measure:
        state["current_die_loc"] = base_loc_with_measured_orientation(measured, pose, "Cal", cfg)
        state["current_die_loc_trusted"] = True
        state["last_trusted_die_loc"] = state["current_die_loc"]
        log("DieState", "after_sample_detection trusted=True source=top_camera_large_circle")
    else:
        state["current_die_loc"] = force_cal_base_z(cal_loc, cfg)
        state["current_die_loc_trusted"] = False
    return sample


def collect_samples(camera, nozzle, part, cal_loc, cfg, state, pairwise_context=False):
    samples = []
    rejected = []
    misdetects = 0
    angles = generated_angles(cfg)
    for i in range(len(angles)):
        check_cancel("SingleCircleCal")
        try:
            sample = measure_single_circle_sample(camera, nozzle, part, cal_loc, cfg, state, i, angles[i],
                                                  None, None, pairwise_context)
            samples.append(sample)
        except SingleCircleDetectionError, e:
            misdetects += 1
            rejected.append({"sample_index": i, "angle_deg": float(angles[i]),
                             "sample_valid": False, "sample_rejected_reason": str(e)})
            log("SingleCircleCal", "sample index=%d rejected_misdetect=%s count=%d" %
                (i, str(e), misdetects))
            if misdetects > int(cfg["allow_misdetections"]):
                raise
        except Exception, e:
            raise
    if len(samples) < 4:
        raise Exception("Need at least four valid single-circle samples; got %d." % len(samples))
    return samples, rejected


def openpnp_pairwise_angles(cfg):
    configured = cfg.get("openpnp_pairwise_angles_deg", None)
    if isinstance(configured, list) and len(configured) > 0:
        angles = [float(value) for value in configured]
        log("OpenPnpPair", "angle_source=openpnp_pairwise_angles_deg")
    else:
        angles = generated_angles(cfg)
        log("OpenPnpPair", "angle_source=generated_angle_range")
    log("OpenPnpPair", "pair_count=%d" % int(len(angles)))
    return angles


def run_openpnp_pairwise_180_nozzle_offset_calibration(camera, nozzle, part, cal_loc, cfg, state):
    angles = openpnp_pairwise_angles(cfg)
    if len(angles) < 1:
        raise Exception("Need at least one angle for openpnp_pairwise_180_offset calibration.")
    initial_anchor, initial_anchor_source = resolve_cal_acquire_anchor(cal_loc, cfg, state, True, "OpenPnpPair")
    log("OpenPnpPair", "initial_anchor_source=%s" % str(initial_anchor_source))
    lock, current_location = acquire_pose_for_location(camera, nozzle, part, initial_anchor, cfg, "Cal")
    current_location = measured_xy_as_base_loc(current_location, "Cal", cfg)
    initial_location = current_location.convertToUnits(LengthUnit.Millimeters)
    old_hx, old_hy, old_hz, old_hr = nozzle_head_offsets_mm(nozzle)
    pairs = []
    sum_dx = 0.0
    sum_dy = 0.0
    accumulated = 0
    pairwise_walk_exceeded = False
    pairwise_stopped_early = False
    log("OpenPnpPair", "initial_center=(%.5f, %.5f)" %
        (float(current_location.getX()), float(current_location.getY())))
    for i in range(len(angles)):
        check_cancel("OpenPnpPair")
        angle = float(angles[i])
        old_location = current_location.convertToUnits(LengthUnit.Millimeters)
        old_lock = lock
        old_nozzle_target = old_lock["nozzle_target_location"].convertToUnits(LengthUnit.Millimeters)
        old_pick_base = mm_loc(old_location.getX(), old_location.getY(), old_location.getZ(), angle)
        pick_die_from_lock_with_rotation(nozzle, part, old_pick_base, cfg, "Cal", state,
                                         old_lock, old_location, angle, "openpnp_pairwise_180")
        place_rotation = angle + 180.0
        target_dx = float(old_nozzle_target.getX()) - float(old_location.getX())
        target_dy = float(old_nozzle_target.getY()) - float(old_location.getY())
        target_mag = math.sqrt(target_dx * target_dx + target_dy * target_dy)
        log("OpenPnpPair", "place_xy_source=old_detected_center")
        log("OpenPnpPair", "old_location_xy=(%.5f, %.5f)" %
            (float(old_location.getX()), float(old_location.getY())))
        log("OpenPnpPair", "old_nozzle_target_xy=(%.5f, %.5f)" %
            (float(old_nozzle_target.getX()), float(old_nozzle_target.getY())))
        log("OpenPnpPair", "old_target_minus_old_location=(%.5f, %.5f, %.5f)" %
            (target_dx, target_dy, target_mag))
        if target_mag > 0.02:
            log("OpenPnpPair", "WARNING old_nozzle_target differs from old detected center by > 0.02 mm")
        place_base = mm_loc(old_location.getX(), old_location.getY(),
                            old_location.getZ(), place_rotation)
        place_die_at(nozzle, part, place_base, cfg, "Cal", state)
        lock, measured = acquire_pose_for_location(camera, nozzle, part, old_location, cfg, "Cal")
        new_location = measured_xy_as_base_loc(measured, "Cal", cfg)
        raw_dx = float(new_location.getX()) - float(old_location.getX())
        raw_dy = float(new_location.getY()) - float(old_location.getY())
        pair_dx = raw_dx / 2.0
        pair_dy = raw_dy / 2.0
        pair_mag = math.sqrt(pair_dx * pair_dx + pair_dy * pair_dy)
        record = {"sample_index": int(i),
                  "angle_deg": angle,
                  "pick_rotation_deg": angle,
                  "place_rotation_deg": place_rotation,
                  "old_center_x_mm": float(old_location.getX()),
                  "old_center_y_mm": float(old_location.getY()),
                  "new_center_x_mm": float(new_location.getX()),
                  "new_center_y_mm": float(new_location.getY()),
                  "raw_delta_x_mm": raw_dx,
                  "raw_delta_y_mm": raw_dy,
                  "pair_offset_x_mm": pair_dx,
                  "pair_offset_y_mm": pair_dy,
                  "pair_offset_mag_mm": pair_mag,
                  "old_nozzle_target_x_mm": float(old_nozzle_target.getX()),
                  "old_nozzle_target_y_mm": float(old_nozzle_target.getY())}
        pairs.append(record)
        sum_dx += pair_dx
        sum_dy += pair_dy
        accumulated += 2
        log("OpenPnpPair", "angle=%.5f" % angle)
        log("OpenPnpPair", "old_center=(%.5f, %.5f)" %
            (float(old_location.getX()), float(old_location.getY())))
        log("OpenPnpPair", "new_center=(%.5f, %.5f)" %
            (float(new_location.getX()), float(new_location.getY())))
        log("OpenPnpPair", "raw_delta=(new-old)=(%.5f, %.5f)" % (raw_dx, raw_dy))
        log("OpenPnpPair", "pair_offset=(raw_delta/2)=(%.5f, %.5f)" % (pair_dx, pair_dy))
        walk_dx = float(new_location.getX()) - float(initial_location.getX())
        walk_dy = float(new_location.getY()) - float(initial_location.getY())
        walk_mag = math.sqrt(walk_dx * walk_dx + walk_dy * walk_dy)
        max_walk = float(cfg.get("openpnp_pairwise_max_total_walk_mm", 3.0))
        log("OpenPnpPairWalk", "initial_center=(%.5f, %.5f)" %
            (float(initial_location.getX()), float(initial_location.getY())))
        log("OpenPnpPairWalk", "current_center=(%.5f, %.5f)" %
            (float(new_location.getX()), float(new_location.getY())))
        log("OpenPnpPairWalk", "total_walk=(%.5f, %.5f, %.5f)" %
            (walk_dx, walk_dy, walk_mag))
        log("OpenPnpPairWalk", "max_allowed=%.5f" % max_walk)
        current_location = new_location
        state["current_die_loc"] = new_location
        state["current_die_loc_trusted"] = True
        state["last_trusted_die_loc"] = new_location
        state["pairwise_walked_die_location_active"] = True
        log("DieState", "after_pair_detection trusted=True source=top_camera_large_circle")
        if walk_mag > max_walk:
            pairwise_walk_exceeded = True
            if bool_value(cfg.get("openpnp_pairwise_abort_on_walk_exceeded", True)):
                pairwise_stopped_early = True
                log("OpenPnpPairWalk", "walk_exceeded=True stopped_early=True")
                break
            log("OpenPnpPairWalk", "walk_exceeded=True stopped_early=False")
    if len(pairs) < 1:
        raise Exception("Pairwise 180 calibration produced no valid pair samples.")
    avg_dx = sum_dx / float(len(pairs))
    avg_dy = sum_dy / float(len(pairs))
    avg_mag = math.sqrt(avg_dx * avg_dx + avg_dy * avg_dy)
    offsets_diff_x = sum_dx * 2.0
    offsets_diff_y = sum_dy * 2.0
    residuals = []
    for rec in pairs:
        rx = float(rec["pair_offset_x_mm"]) - avg_dx
        ry = float(rec["pair_offset_y_mm"]) - avg_dy
        residual = math.sqrt(rx * rx + ry * ry)
        rec["residual_x_mm"] = rx
        rec["residual_y_mm"] = ry
        rec["residual_mm"] = residual
        residuals.append(residual)
    rms_error = rms(residuals)
    peak_error = max(residuals) if residuals else 0.0
    residuals_ok = True
    if len(pairs) > 1:
        residuals_ok = rms_error <= float(cfg.get("single_circle_max_model_rms_mm", 0.12)) and \
            peak_error <= float(cfg.get("single_circle_max_model_peak_mm", 0.30))
    walk_ok = (not bool(pairwise_walk_exceeded)) or \
        (not bool_value(cfg.get("openpnp_pairwise_abort_on_walk_exceeded", True)))
    safe = avg_mag <= float(cfg["max_apply_mm"]) and \
        avg_mag <= float(cfg["offset_threshold_mm"]) and \
        bool(walk_ok) and bool(residuals_ok)
    block_reason = None
    if avg_mag > float(cfg["max_apply_mm"]):
        block_reason = "pairwise XY candidate exceeds max_apply_mm"
    elif avg_mag > float(cfg["offset_threshold_mm"]):
        block_reason = "pairwise XY candidate exceeds offset_threshold_mm"
    elif not bool(walk_ok):
        block_reason = "pairwise walk exceeded configured limit"
    elif not bool(residuals_ok):
        block_reason = "pairwise residuals exceed configured limits"
    computed = {
        "model": "openpnp_pairwise_180_offset",
        "computed_model": "openpnp_pairwise_180_offset",
        "script_model": "single_circle",
        "script_file": "pick_cameraAlign_SingleCircleTipCalibration.py",
        "single_circle_model_mode": str(cfg.get("single_circle_model_mode", "openpnp_model_camera_offset")),
        "single_circle_fit_model_type": "openpnp_pairwise_180_offset",
        "fit_model_type": "openpnp_pairwise_180_offset",
        "calibration_frame": "camera_only_large_center_circle_mm",
        "preview_frame": "openpnp_nozzle_head_offsets",
        "active_fit_source": "openpnp_pairwise_180_offset",
        "candidate_source": "openpnp_pairwise_180_offset",
        "old_model_description": "current_active_openpnp_offsets",
        "new_model_description": "openpnp_pairwise_180_static_nozzle_head_offset_candidate",
        "diagnostic_model_description": "pairwise 180 degree pick/place average; runout cancels by construction",
        "persistent_write_allowed": True,
        "persistent_write_warning": "Only the pairwise XY nozzle head offset delta may be written.",
        "persistent_storage_note": "No Kasa circle center, radius, phase, or runout vector drives this calibration.",
        "persistent_head_offset_delta": {"dx": avg_dx, "dy": avg_dy, "mag": avg_mag},
        "pairwise_average_pair_offset": {"dx": avg_dx, "dy": avg_dy, "mag": avg_mag},
        "average_pair_offset": {"dx": avg_dx, "dy": avg_dy, "mag": avg_mag},
        "same_run_visual_pick_jog": {"dx": 0.0, "dy": 0.0, "mag": 0.0,
                                     "default_zero_with_fresh_visual_lock": True},
        "square_rotation_bias_deg": None,
        "virtual_correction_available": True,
        "virtual_static_delta_x_mm": avg_dx,
        "virtual_static_delta_y_mm": avg_dy,
        "virtual_residual_angle_vector_x_mm": 0.0,
        "virtual_residual_angle_vector_y_mm": 0.0,
        "virtual_runout_preview_enabled": False,
        "runout_model_enabled": False,
        "samples": pairs,
        "pairwise_samples": pairs,
        "measured_locations_count": len(pairs),
        "selected_head_delta_x_mm": avg_dx,
        "selected_head_delta_y_mm": avg_dy,
        "selected_head_delta_mag_mm": avg_mag,
        "static_head_offset_delta_x_mm": avg_dx,
        "static_head_offset_delta_y_mm": avg_dy,
        "static_head_offset_delta_mag_mm": avg_mag,
        "nozzle_head_offset_delta_x_mm": avg_dx,
        "nozzle_head_offset_delta_y_mm": avg_dy,
        "nozzle_head_offset_delta_mag_mm": avg_mag,
        "bias_x_mm": avg_dx,
        "bias_y_mm": avg_dy,
        "residual_static_bias_x_mm": avg_dx,
        "residual_static_bias_y_mm": avg_dy,
        "residual_angle_vector_x_mm": 0.0,
        "residual_angle_vector_y_mm": 0.0,
        "residual_angle_radius_mm": 0.0,
        "residual_angle_phase_deg": 0.0,
        "rms_error_mm": rms_error,
        "peak_error_mm": peak_error,
        "model_rms_error_mm": rms_error,
        "model_peak_error_mm": peak_error,
        "pairwise_residuals_passed": bool(residuals_ok),
        "pairwise_walk_passed": bool(walk_ok),
        "pairwise_xy_candidate_valid": bool(safe),
        "candidate_delta_available": True,
        "diagnostic_preview_available": True,
        "confirm_preview_available": True,
        "diagnostic_preview_allowed": bool_value(cfg.get("confirm_allow_diagnostic_preview", True)),
        "safe_to_apply": bool(safe),
        "model_quality_passed": bool(safe),
        "xy_fit_passed": bool(safe),
        "active_correction_allowed": bool(safe),
        "active_correction_block_reason": block_reason,
        "apply_available": False,
        "apply_block_reason": None,
        "residual_warning_only": bool_value(cfg.get("fit_residual_warning_only", True)),
        "orientation_circle_used_for_xy": False,
        "orientation_circle_used_for_model_fit": False,
        "runout_write_allowed": False,
        "nozzle_tip_runout_written": False,
        "openpnp_pair_accumulated": int(accumulated),
        "openpnp_offsets_diff_x_mm": offsets_diff_x,
        "openpnp_offsets_diff_y_mm": offsets_diff_y,
        "openpnp_pair_count": len(pairs),
        "pairwise_walk_exceeded": bool(pairwise_walk_exceeded),
        "pairwise_stopped_early": bool(pairwise_stopped_early),
        "pairwise_walk_guard_max_mm": float(cfg.get("openpnp_pairwise_max_total_walk_mm", 3.0)),
        "show_shift_delta_at_0": {"dx": avg_dx, "dy": avg_dy, "mag": avg_mag},
        "show_shift_delta_at_90": {"dx": avg_dx, "dy": avg_dy, "mag": avg_mag},
        "command_delta_at_0": {"dx": avg_dx, "dy": avg_dy, "mag": avg_mag},
        "command_delta_at_90": {"dx": avg_dx, "dy": avg_dy, "mag": avg_mag},
        "current_head_offsets_before": {"x": old_hx, "y": old_hy, "z": old_hz, "rotation": old_hr},
        "candidate_head_offsets_after": {"x": old_hx + avg_dx, "y": old_hy + avg_dy,
                                         "z": old_hz, "rotation": old_hr}
    }
    log("OpenPnpPair", "average_pair_offset=(%.5f, %.5f)" % (avg_dx, avg_dy))
    log("OpenPnpPair", "selected_head_delta=(%.5f, %.5f, %.5f)" % (avg_dx, avg_dy, avg_mag))
    log("OpenPnpPair", "residuals_ok=%s rms=%.5f peak=%.5f" %
        (str(bool(residuals_ok)), float(rms_error), float(peak_error)))
    log("OpenPnpPair", "walk_ok=%s walk_exceeded=%s" %
        (str(bool(walk_ok)), str(bool(pairwise_walk_exceeded))))
    log("OpenPnpPair", "pairwise_xy_candidate_valid=%s" % str(bool(safe)))
    if block_reason is not None:
        log("OpenPnpPair", "candidate_block_reason=%s" % block_reason)
    log("OpenPnpPair", "accumulated=%d" % int(accumulated))
    return computed, pairs, []


def solve_fit(samples):
    # Unused legacy diagnostic; not reachable from the supported pairwise model.
    n = 4
    ata = [[0.0 for j in range(n)] for i in range(n)]
    atb = [0.0 for i in range(n)]
    try:
        for s in samples:
            a = math.radians(float(s["command_angle_deg"]))
            ca = math.cos(a)
            sa = math.sin(a)
            rows = [([1.0, 0.0, ca, -sa], float(s["error_x_mm"])),
                    ([0.0, 1.0, sa, ca], float(s["error_y_mm"]))]
            for row, value in rows:
                for i in range(n):
                    atb[i] += row[i] * value
                    for j in range(n):
                        ata[i][j] += row[i] * row[j]
        return base.solve_linear_system(ata, atb)
    except Exception, e:
        log("SingleCircleFit", "rotating-vector fit failed; using mean-only fit: %s" % e)
        bx = mean([float(s["error_x_mm"]) for s in samples])
        by = mean([float(s["error_y_mm"]) for s in samples])
        return [bx, by, 0.0, 0.0]


def predicted_error(bx, by, vx, vy, angle_deg):
    # Unused legacy diagnostic; not reachable from the supported pairwise model.
    rotated = base.rotate_xy(vx, vy, angle_deg)
    return {"x": bx + rotated["x"], "y": by + rotated["y"]}


def circular_mean_degrees(values):
    if values is None or len(values) < 1:
        return 0.0
    sx = 0.0
    sy = 0.0
    for value in values:
        a = math.radians(float(value))
        sx += math.cos(a)
        sy += math.sin(a)
    if abs(sx) < 0.000000001 and abs(sy) < 0.000000001:
        return 0.0
    return normalize_angle(math.degrees(math.atan2(sy, sx)))


def fit_kasa_circle(points):
    # Unused legacy diagnostic; Kasa fit is not a selectable calibration model.
    if points is None or len(points) < 3:
        raise Exception("Need at least three points for Kasa circle fit.")
    ata = [[0.0 for j in range(3)] for i in range(3)]
    atb = [0.0 for i in range(3)]
    for p in points:
        x = float(p["x"])
        y = float(p["y"])
        row = [x, y, 1.0]
        value = -(x * x + y * y)
        for i in range(3):
            atb[i] += row[i] * value
            for j in range(3):
                ata[i][j] += row[i] * row[j]
    d, e, f = base.solve_linear_system(ata, atb)
    center_x = -float(d) / 2.0
    center_y = -float(e) / 2.0
    r2 = center_x * center_x + center_y * center_y - float(f)
    if r2 < 0.0 and r2 > -0.000000001:
        r2 = 0.0
    if r2 < 0.0:
        raise Exception("Kasa circle fit produced negative radius squared %.8f." % r2)
    return {"center_x": center_x,
            "center_y": center_y,
            "radius": math.sqrt(r2)}


def model_offset_for_rotation(computed, rotation_deg):
    # Unused legacy diagnostic for saved pre-pairwise results only.
    center_x = float(computed["model_center_x_mm"])
    center_y = float(computed["model_center_y_mm"])
    radius = float(computed["model_radius_mm"])
    phase = float(computed["model_phase_shift_deg"])
    a = math.radians(float(rotation_deg) - phase)
    runout_x = radius * math.cos(a)
    runout_y = radius * math.sin(a)
    x = center_x + runout_x
    y = center_y + runout_y
    return {"x": x,
            "y": y,
            "mag": math.sqrt(x * x + y * y),
            "center_x": center_x,
            "center_y": center_y,
            "runout_x": runout_x,
            "runout_y": runout_y,
            "radius": radius,
            "phase_shift_deg": phase}


def command_delta_for_rotation(computed, rotation_deg, cfg):
    # Unused legacy diagnostic for saved pre-pairwise results only.
    # Corrected architecture:
    # - the static center term is the OpenPnP nozzle head-offset candidate
    # - the angle-dependent term is runout diagnostics unless explicitly enabled
    # - callers must not treat center+runout as a fake always-on jog correction
    offset = model_offset_for_rotation(computed, rotation_deg)
    sign = float(computed.get("static_head_offset_sign",
                              computed.get("single_circle_apply_model_sign",
                                           cfg.get("single_circle_head_delta_sign_override", 1.0))))
    include_runout = bool_value(cfg.get("single_circle_enable_runtime_runout_compensation", False))
    static_dx = sign * float(offset["center_x"])
    static_dy = sign * float(offset["center_y"])
    angle_dx = 0.0
    angle_dy = 0.0
    if include_runout:
        angle_dx = sign * float(offset["runout_x"])
        angle_dy = sign * float(offset["runout_y"])
    dx = static_dx + angle_dx
    dy = static_dy + angle_dy
    mag = math.sqrt(dx * dx + dy * dy)
    log("ModelEval", "rotation_deg=%.5f" % float(rotation_deg))
    log("ModelEval", "measured_model_offset=(center+runout)=(%.5f, %.5f, %.5f)" %
        (float(offset["x"]), float(offset["y"]), float(offset["mag"])))
    log("ModelEval", "static_head_offset_sign=%.5f" % sign)
    log("ModelEval", "runtime_runout_compensation_enabled=%s" % str(bool(include_runout)))
    log("ModelEval", "command_delta=(%.5f, %.5f, %.5f)" % (dx, dy, mag))
    return {"dx": dx, "dy": dy, "mag": mag,
            "offset": offset,
            "correction_sign": sign,
            "static_dx": static_dx,
            "static_dy": static_dy,
            "angle_dx": angle_dx,
            "angle_dy": angle_dy,
            "runtime_runout_compensation_enabled": bool(include_runout)}


def command_delta_for_rotation_with_sign(computed, rotation_deg, sign):
    offset = model_offset_for_rotation(computed, rotation_deg)
    dx = float(sign) * float(offset["x"])
    dy = float(sign) * float(offset["y"])
    return {"dx": dx, "dy": dy, "mag": math.sqrt(dx * dx + dy * dy),
            "offset": offset}


def evaluate_model_quality(computed, cfg):
    center_x = float(computed.get("model_center_x_mm", 0.0))
    center_y = float(computed.get("model_center_y_mm", 0.0))
    center_mag = math.sqrt(center_x * center_x + center_y * center_y)
    radius = float(computed.get("model_radius_mm", 0.0))
    rms_error = float(computed.get("model_rms_error_mm", 0.0))
    peak_error = float(computed.get("model_peak_error_mm", 0.0))
    passed = center_mag <= float(cfg.get("single_circle_max_model_center_mm", 1.5)) and \
        radius <= float(cfg.get("single_circle_max_model_radius_mm", 1.5)) and \
        rms_error <= float(cfg.get("single_circle_max_model_rms_mm", 0.12)) and \
        peak_error <= float(cfg.get("single_circle_max_model_peak_mm", 0.30))
    block_on_fail = bool_value(cfg.get("single_circle_block_active_correction_on_model_fail", True))
    active_allowed = bool(passed or not block_on_fail)
    reason = None
    if not active_allowed:
        reason = "model quality failed and single_circle_block_active_correction_on_model_fail is true"
    computed["model_center_mag_mm"] = center_mag
    computed["model_quality_passed"] = bool(passed)
    computed["active_correction_allowed"] = bool(active_allowed)
    computed["active_correction_block_reason"] = reason
    log("ModelQuality", "center_mag=%.5f" % center_mag)
    log("ModelQuality", "radius=%.5f" % radius)
    log("ModelQuality", "rms=%.5f" % rms_error)
    log("ModelQuality", "peak=%.5f" % peak_error)
    log("ModelQuality", "passed=%s" % str(bool(passed)))
    log("ModelQuality", "active_correction_allowed=%s" % str(bool(active_allowed)))
    if reason is not None:
        log("ModelQuality", "active_correction_block_reason=%s" % reason)
    return computed


def nozzle_head_offsets_mm(nozzle):
    old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
    return float(old.getX()), float(old.getY()), float(old.getZ()), float(old.getRotation())


def nozzle_head_offsets_diag(nozzle):
    x, y, z, r = nozzle_head_offsets_mm(nozzle)
    return {"x": x, "y": y, "z": z, "rotation": r}


def set_nozzle_head_offsets_xy(nozzle, x, y):
    old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
    new = mm_loc(float(x), float(y), float(old.getZ()), float(old.getRotation()))
    nozzle.setHeadOffsets(new)
    log("HeadOffset", "set_nozzle_head_offsets=(%.5f, %.5f)" % (float(x), float(y)))
    return new


def apply_temporary_head_offsets(nozzle, x, y, reason):
    old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
    set_nozzle_head_offsets_xy(nozzle, x, y)
    log("HeadOffset", "temporary_apply reason=%s old=(%.5f, %.5f) new=(%.5f, %.5f)" %
        (str(reason), float(old.getX()), float(old.getY()), float(x), float(y)))
    return old


def restore_temporary_head_offsets(nozzle, old, reason):
    if old is None:
        return
    nozzle.setHeadOffsets(old)
    restored = old.convertToUnits(LengthUnit.Millimeters)
    log("HeadOffset", "temporary_restore reason=%s restored=(%.5f, %.5f)" %
        (str(reason), float(restored.getX()), float(restored.getY())))


def derive_machine_xml_path(cfg):
    configured = cfg.get("machine_xml_path", None)
    if configured is not None and str(configured).strip() != "":
        return str(configured)
    try:
        directory = Configuration.get().getConfigurationDirectory()
        if directory is not None:
            return os.path.join(str(directory), "machine.xml")
    except:
        pass
    return None


def backup_machine_xml_if_requested(cfg):
    if not bool_value(cfg.get("write_machine_xml_backup", True)):
        return None
    path = derive_machine_xml_path(cfg)
    if path is None or not os.path.exists(path):
        log("Apply", "machine.xml backup skipped; path unavailable.")
        return None
    backup = path + ".pick_cameraAlign_SingleCircleTipCalibration.%d.bak" % int(time.time())
    shutil.copy2(path, backup)
    log("Apply", "machine.xml backup written: %s" % backup)
    return backup


def add_model_evaluations(computed, cfg):
    offset0 = model_offset_for_rotation(computed, 0.0)
    delta0 = command_delta_for_rotation(computed, 0.0, cfg)
    offset90 = model_offset_for_rotation(computed, 90.0)
    delta90 = command_delta_for_rotation(computed, 90.0, cfg)
    computed["model_offset_at_0"] = {"x": offset0["x"], "y": offset0["y"], "mag": offset0["mag"]}
    computed["command_delta_at_0"] = {"dx": delta0["dx"], "dy": delta0["dy"], "mag": delta0["mag"]}
    computed["model_offset_at_90"] = {"x": offset90["x"], "y": offset90["y"], "mag": offset90["mag"]}
    computed["command_delta_at_90"] = {"dx": delta90["dx"], "dy": delta90["dy"], "mag": delta90["mag"]}
    computed["show_shift_delta_at_0"] = dict(computed["command_delta_at_0"])
    computed["show_shift_delta_at_90"] = dict(computed["command_delta_at_90"])
    return computed


def fit_single_circle_model(samples, cfg):
    # Unused legacy diagnostic; Kasa fit is not reachable from run().
    points = []
    for s in samples:
        ex = float(s.get("measured_center_x_mm", s.get("error_x_mm", 0.0))) - \
            float(s.get("nominal_center_x_mm", 0.0))
        ey = float(s.get("measured_center_y_mm", s.get("error_y_mm", 0.0))) - \
            float(s.get("nominal_center_y_mm", 0.0))
        points.append({"x": ex, "y": ey, "sample": s})
        s["error_x_mm"] = ex
        s["error_y_mm"] = ey
        s["error_mag_mm"] = math.sqrt(ex * ex + ey * ey)
    try:
        circle = fit_kasa_circle(points)
    except Exception, e:
        log("ModelFit", "Kasa circle fit failed; using mean center and mean radius fallback: %s" % e)
        cx = mean([float(p["x"]) for p in points])
        cy = mean([float(p["y"]) for p in points])
        rr = [math.sqrt((float(p["x"]) - cx) * (float(p["x"]) - cx) +
                        (float(p["y"]) - cy) * (float(p["y"]) - cy)) for p in points]
        circle = {"center_x": cx, "center_y": cy, "radius": mean(rr)}
    center_x = float(circle["center_x"])
    center_y = float(circle["center_y"])
    radius = float(circle["radius"])
    phase_degenerate = radius < 0.005
    phases = []
    if phase_degenerate:
        phase_shift = 0.0
    else:
        for p in points:
            centered_x = float(p["x"]) - center_x
            centered_y = float(p["y"]) - center_y
            measured_angle = normalize_angle(math.degrees(math.atan2(centered_y, centered_x)))
            command_angle = float(p["sample"]["command_angle_deg"])
            phases.append(normalize_angle(command_angle - measured_angle))
        phase_shift = circular_mean_degrees(phases)
    temp = {"model_center_x_mm": center_x,
            "model_center_y_mm": center_y,
            "model_radius_mm": radius,
            "model_phase_shift_deg": phase_shift}
    residuals = []
    predictions = []
    configured_sign = -float(cfg.get("single_circle_head_delta_sign_override", 1.0))
    sign = configured_sign
    if bool_value(cfg.get("single_circle_auto_select_model_sign", False)):
        configured_score = 0
        opposite_score = 0
        for p in points:
            s = p["sample"]
            angle = float(s["command_angle_deg"])
            measured_x = float(p["x"])
            measured_y = float(p["y"])
            measured_mag = math.sqrt(measured_x * measured_x + measured_y * measured_y)
            configured_delta = command_delta_for_rotation_with_sign(temp, angle, configured_sign)
            opposite_delta = command_delta_for_rotation_with_sign(temp, angle, -configured_sign)
            configured_x = measured_x + float(configured_delta["dx"])
            configured_y = measured_y + float(configured_delta["dy"])
            opposite_x = measured_x + float(opposite_delta["dx"])
            opposite_y = measured_y + float(opposite_delta["dy"])
            if measured_mag - math.sqrt(configured_x * configured_x + configured_y * configured_y) > 0.0:
                configured_score += 1
            if measured_mag - math.sqrt(opposite_x * opposite_x + opposite_y * opposite_y) > 0.0:
                opposite_score += 1
        if opposite_score > configured_score:
            sign = -configured_sign
            log("ModelVerify", "auto_selected_correction_sign=%.5f configured_sign=%.5f" %
                (sign, configured_sign))
    opposite_sign = -sign
    configured_improves = 0
    opposite_improves = 0
    configured_worsens = 0
    opposite_worsens = 0
    sample_model_verification = []
    for p in points:
        s = p["sample"]
        angle = float(s["command_angle_deg"])
        measured_x = float(p["x"])
        measured_y = float(p["y"])
        offset = model_offset_for_rotation(temp, angle)
        rx = measured_x - float(offset["x"])
        ry = measured_y - float(offset["y"])
        residual_mag = math.sqrt(rx * rx + ry * ry)
        residuals.append(residual_mag)
        configured_delta = command_delta_for_rotation_with_sign(temp, angle, sign)
        opposite_delta = command_delta_for_rotation_with_sign(temp, angle, opposite_sign)
        corrected_x = measured_x + float(configured_delta["dx"])
        corrected_y = measured_y + float(configured_delta["dy"])
        corrected_mag = math.sqrt(corrected_x * corrected_x + corrected_y * corrected_y)
        measured_mag = math.sqrt(measured_x * measured_x + measured_y * measured_y)
        improvement = measured_mag - corrected_mag
        opposite_x = measured_x + float(opposite_delta["dx"])
        opposite_y = measured_y + float(opposite_delta["dy"])
        opposite_improvement = measured_mag - math.sqrt(opposite_x * opposite_x + opposite_y * opposite_y)
        if improvement > 0.0:
            configured_improves += 1
        if improvement < 0.0:
            configured_worsens += 1
        if opposite_improvement > 0.0:
            opposite_improves += 1
        if opposite_improvement < 0.0:
            opposite_worsens += 1
        predictions.append({"command_angle_deg": angle,
                            "measured_error_x_mm": measured_x,
                            "measured_error_y_mm": measured_y,
                            "predicted_error_x_mm": float(offset["x"]),
                            "predicted_error_y_mm": float(offset["y"]),
                            "residual_x_mm": rx,
                            "residual_y_mm": ry,
                            "residual_mm": residual_mag})
        verify_record = {"angle_deg": angle,
                         "measured_error_x_mm": measured_x,
                         "measured_error_y_mm": measured_y,
                         "model_offset_x_mm": float(offset["x"]),
                         "model_offset_y_mm": float(offset["y"]),
                         "command_delta_x_mm": float(configured_delta["dx"]),
                         "command_delta_y_mm": float(configured_delta["dy"]),
                         "predicted_corrected_error_x_mm": corrected_x,
                         "predicted_corrected_error_y_mm": corrected_y,
                         "predicted_corrected_error_mag_mm": corrected_mag,
                         "improvement_mm": improvement}
        sample_model_verification.append(verify_record)
        log("ModelVerify", "angle=%.5f" % angle)
        log("ModelVerify", "measured_error=(%.5f, %.5f, %.5f)" %
            (measured_x, measured_y, measured_mag))
        log("ModelVerify", "model_offset=(%.5f, %.5f, %.5f)" %
            (float(offset["x"]), float(offset["y"]), float(offset["mag"])))
        log("ModelVerify", "command_delta=(%.5f, %.5f, %.5f)" %
            (float(configured_delta["dx"]), float(configured_delta["dy"]), float(configured_delta["mag"])))
        log("ModelVerify", "predicted_corrected_error=(%.5f, %.5f, %.5f)" %
            (corrected_x, corrected_y, corrected_mag))
        log("ModelVerify", "improvement=%.5f" % improvement)
    rms_error = rms(residuals)
    peak_error = max(residuals) if residuals else 0.0
    selected_dx = sign * center_x
    selected_dy = sign * center_y
    selected_mag = math.sqrt(selected_dx * selected_dx + selected_dy * selected_dy)
    vector_a = math.radians(0.0 - phase_shift)
    residual_vx = radius * math.cos(vector_a)
    residual_vy = radius * math.sin(vector_a)
    safe = selected_mag <= float(cfg["max_apply_mm"]) and \
        selected_mag <= float(cfg["offset_threshold_mm"]) and \
        rms_error <= float(cfg.get("single_circle_max_model_rms_mm", 0.12)) and \
        peak_error <= float(cfg.get("single_circle_max_model_peak_mm", 0.30))
    computed = {
        "model": "openpnp_model_camera_offset",
        "script_model": "single_circle",
        "script_file": "pick_cameraAlign_SingleCircleTipCalibration.py",
        "single_circle_model_mode": str(cfg.get("single_circle_model_mode", "openpnp_model_camera_offset")),
        "single_circle_fit_model_type": str(cfg.get("single_circle_fit_model_type", "openpnp_pairwise_180_offset")),
        "single_circle_apply_model_sign": sign,
        "static_head_offset_sign": sign,
        "sign_selection_method": "negative_measured_static_bias_in_camera_only_calibration_frame",
        "calibration_frame": "camera_only_large_center_circle_mm",
        "preview_frame": "openpnp_nozzle_head_offsets",
        "old_model_description": "current_active_openpnp_offsets_and_runout",
        "new_model_description": "openpnp_static_nozzle_head_offset_candidate_plus_separate_runout_diagnostic",
        "diagnostic_model_description": "static center term is writable head offset; dynamic runout term is separate diagnostic",
        "persistent_write_allowed": True,
        "persistent_write_warning": "Only the static nozzle head offset delta may be written. Runout is not written.",
        "persistent_storage_note": "Runout diagnostics remain separate from OpenPnP nozzle head offsets.",
        "persistent_head_offset_delta": {"dx": selected_dx, "dy": selected_dy, "mag": selected_mag},
        "same_run_visual_pick_jog": {"dx": 0.0, "dy": 0.0, "mag": 0.0,
                                     "default_zero_with_fresh_visual_lock": True},
        "dynamic_runout_vector": {"x": float(residual_vx), "y": float(residual_vy),
                                  "radius": radius, "phase_deg": phase_shift,
                                  "persistent": False},
        "square_rotation_bias_deg": None,
        "virtual_correction_available": True,
        "virtual_static_delta_x_mm": selected_dx,
        "virtual_static_delta_y_mm": selected_dy,
        "virtual_residual_angle_vector_x_mm": float(residual_vx),
        "virtual_residual_angle_vector_y_mm": float(residual_vy),
        "virtual_runout_preview_enabled": bool_value(cfg.get("single_circle_enable_runtime_runout_compensation", False)),
        "runout_model_enabled": bool_value(cfg.get("single_circle_enable_runtime_runout_compensation", False)),
        "samples": samples,
        "measured_locations_count": len(samples),
        "model_center_x_mm": center_x,
        "model_center_y_mm": center_y,
        "model_radius_mm": radius,
        "model_phase_shift_deg": phase_shift,
        "phase_degenerate_radius": bool(phase_degenerate),
        "model_rms_error_mm": rms_error,
        "model_peak_error_mm": peak_error,
        "selected_head_delta_x_mm": selected_dx,
        "selected_head_delta_y_mm": selected_dy,
        "selected_head_delta_mag_mm": selected_mag,
        "static_head_offset_delta_x_mm": selected_dx,
        "static_head_offset_delta_y_mm": selected_dy,
        "static_head_offset_delta_mag_mm": selected_mag,
        "nozzle_head_offset_delta_x_mm": selected_dx,
        "nozzle_head_offset_delta_y_mm": selected_dy,
        "nozzle_head_offset_delta_mag_mm": selected_mag,
        "bias_x_mm": center_x,
        "bias_y_mm": center_y,
        "residual_static_bias_x_mm": center_x,
        "residual_static_bias_y_mm": center_y,
        "residual_angle_vector_x_mm": float(residual_vx),
        "residual_angle_vector_y_mm": float(residual_vy),
        "residual_angle_radius_mm": radius,
        "residual_angle_phase_deg": phase_shift,
        "runout_vector_x_mm": float(residual_vx),
        "runout_vector_y_mm": float(residual_vy),
        "runout_radius_mm": radius,
        "runout_phase_deg": phase_shift,
        "rms_error_mm": rms_error,
        "peak_error_mm": peak_error,
        "sample_predictions": predictions,
        "sample_model_verification": sample_model_verification,
        "configured_sign_improves_count": int(configured_improves),
        "configured_sign_worsens_count": int(configured_worsens),
        "opposite_sign_improves_count": int(opposite_improves),
        "opposite_sign_worsens_count": int(opposite_worsens),
        "safe_to_apply": bool(safe),
        "diagnostic_preview_available": True,
        "confirm_preview_available": True,
        "apply_available": False,
        "apply_block_reason": None,
        "residual_warning_only": bool_value(cfg.get("fit_residual_warning_only", True)),
        "orientation_circle_used_for_xy": False,
        "orientation_circle_used_for_model_fit": False,
        "runout_write_allowed": False,
        "nozzle_tip_runout_written": False
    }
    most_samples = int(len(samples) / 2) + 1
    if configured_sign < 0.0 and sign == configured_sign and opposite_improves >= most_samples and configured_worsens >= most_samples:
        log("ModelVerify", "WARNING correction sign may be wrong")
        computed["correction_sign_warning"] = "correction sign may be wrong"
    add_model_evaluations(computed, cfg)
    evaluate_model_quality(computed, cfg)
    log("SingleCircleFit", "measured_locations_count=%d" % len(samples))
    log("ModelFit", "center=(%.5f, %.5f)" % (center_x, center_y))
    log("ModelFit", "radius=%.5f" % radius)
    log("ModelFit", "phase_shift_deg=%.5f" % phase_shift)
    log("ModelFit", "phase_degenerate_radius=%s" % str(bool(phase_degenerate)))
    log("ModelFit", "rms=%.5f" % rms_error)
    log("ModelFit", "peak=%.5f" % peak_error)
    log("SingleCircleFit", "selected_head_delta=(correction_sign*center)=(%.5f, %.5f)" %
        (selected_dx, selected_dy))
    log("SingleCircleFit", "residual_warning_only=%s" % str(bool_value(cfg.get("fit_residual_warning_only", True))))
    return computed


def sample_for_angle(samples, angle_deg):
    best = None
    best_error = None
    for s in samples:
        err = abs(normalize_angle(float(s.get("command_angle_deg", 0.0)) - float(angle_deg)))
        if best_error is None or err < best_error:
            best = s
            best_error = err
    if best is not None and best_error is not None and best_error <= 0.0001:
        return best
    return None


def verify_single_circle_static_xy(camera, nozzle, part, cal_loc, cfg, state, computed):
    records = []
    if (not bool_value(cfg.get("single_circle_verify_static_xy_after_fit", True))) or \
       (not bool_value(cfg.get("single_circle_verify_old_new_preview", True))):
        computed["verification_old_new"] = records
        computed["verification_old_new_available"] = False
        computed["verification_old_new_is_predicted"] = True
        log("Verify", "static_xy_after_fit_enabled=%s old_new_preview_enabled=%s" %
            (str(bool_value(cfg.get("single_circle_verify_static_xy_after_fit", True))),
             str(bool_value(cfg.get("single_circle_verify_old_new_preview", True)))))
        return records
    samples = computed.get("samples", [])
    for angle in cfg.get("single_circle_verify_angles_deg", [0.0, 90.0, 180.0, 270.0]):
        angle = float(angle)
        s = sample_for_angle(samples, angle)
        if s is not None:
            old_x = float(s.get("error_x_mm", 0.0))
            old_y = float(s.get("error_y_mm", 0.0))
            old_source = "measured_sample"
        else:
            if "model_center_x_mm" in computed:
                pred = model_offset_for_rotation(computed, angle)
                old_x = float(pred["x"])
                old_y = float(pred["y"])
                old_source = "openpnp_model_prediction"
            else:
                pred = predicted_error(float(computed.get("bias_x_mm", 0.0)),
                                       float(computed.get("bias_y_mm", 0.0)),
                                       float(computed.get("residual_angle_vector_x_mm", 0.0)),
                                       float(computed.get("residual_angle_vector_y_mm", 0.0)),
                                       angle)
                old_x = float(pred["x"])
                old_y = float(pred["y"])
                old_source = "model_prediction"
        correction = virtual_correction_for_rotation(computed, angle, cfg, "verification")
        new_x = old_x + float(correction["dx"])
        new_y = old_y + float(correction["dy"])
        old_mag = math.sqrt(old_x * old_x + old_y * old_y)
        new_mag = math.sqrt(new_x * new_x + new_y * new_y)
        improvement = old_mag - new_mag
        record = {
            "angle_deg": angle,
            "physical_verification": False,
            "old_error_source": old_source,
            "old_error_x_mm": old_x,
            "old_error_y_mm": old_y,
            "old_error_mag_mm": old_mag,
            "virtual_correction_x_mm": float(correction["dx"]),
            "virtual_correction_y_mm": float(correction["dy"]),
            "virtual_correction_mag_mm": float(correction["mag"]),
            "virtual_static_delta_x_mm": float(correction["static_dx"]),
            "virtual_static_delta_y_mm": float(correction["static_dy"]),
            "virtual_angle_delta_x_mm": float(correction["angle_dx"]),
            "virtual_angle_delta_y_mm": float(correction["angle_dy"]),
            "predicted_new_error_x_mm": new_x,
            "predicted_new_error_y_mm": new_y,
            "predicted_new_error_mag_mm": new_mag,
            "improvement_mag_mm": improvement
        }
        records.append(record)
        log("Verify", "angle=%.5f" % angle)
        log("Verify", "old_error=(%.5f, %.5f, %.5f) source=%s" %
            (old_x, old_y, old_mag, old_source))
        log("Verify", "virtual_correction=(%.5f, %.5f, %.5f)" %
            (float(correction["dx"]), float(correction["dy"]), float(correction["mag"])))
        log("Verify", "predicted_new_error=(%.5f, %.5f, %.5f)" %
            (new_x, new_y, new_mag))
        log("Verify", "improvement_mag=%.5f" % improvement)
    computed["verification_old_new"] = records
    computed["verification_old_new_available"] = True
    computed["verification_old_new_is_predicted"] = True
    return records


def verification_stats_from_samples(samples):
    errors = [float(s.get("error_mag_mm", 0.0)) for s in samples if bool_value(s.get("sample_valid", True))]
    return {"sample_count": len(errors),
            "rms_error_mm": rms(errors),
            "peak_error_mm": max(errors) if errors else None,
            "mean_error_mm": mean(errors) if errors else None}


def is_top_camera_miss_error(error):
    text = str(error)
    return "raw_count 0" in text or "expected 2" in text or "No usable result" in text


def measure_verify_ab_sample(camera, nozzle, part, cal_loc, cfg, state, sample_index, angle, pairwise_context):
    anchor, anchor_source = resolve_cal_acquire_anchor(cal_loc, cfg, state, pairwise_context, "VerifyAB")
    try:
        return measure_single_circle_sample(camera, nozzle, part, cal_loc, cfg, state, sample_index, angle,
                                            anchor, anchor_source, pairwise_context)
    except Exception, e:
        last = state.get("last_trusted_die_loc", None)
        if not bool(pairwise_context) or last is None or not is_top_camera_miss_error(e):
            raise
        last = last.convertToUnits(LengthUnit.Millimeters)
        dx, dy, mag = xy_distance_loc(last, anchor)
        if mag <= 0.000001:
            raise
        log("VerifyAB", "retrying_at_last_trusted_die_loc=True")
        log("VerifyAB", "original_anchor_source=%s" % str(anchor_source))
        log("VerifyAB", "retry_anchor_source=state_last_trusted_die_loc")
        return measure_single_circle_sample(camera, nozzle, part, cal_loc, cfg, state, sample_index, angle,
                                            last, "state_last_trusted_die_loc", pairwise_context)


def run_physical_ab_verification(camera, nozzle, part, cal_loc, cfg, state, computed):
    out = {"enabled": bool_value(cfg.get("single_circle_physical_ab_verification", True)),
           "verification_mode": "physical_ab_head_offset_substitution",
           "verification_is_physical": False,
           "before_samples": [],
           "after_samples": [],
           "verification_before_after_records": [],
           "passed": None,
           "warning": None}
    if not bool_value(out["enabled"]):
        log("VerifyAB", "enabled=False")
        return out
    old_offsets = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
    old_x = float(old_offsets.getX())
    old_y = float(old_offsets.getY())
    new_x = float(computed.get("candidate_head_offsets_after", {}).get("x", old_x + float(computed.get("static_head_offset_delta_x_mm", 0.0))))
    new_y = float(computed.get("candidate_head_offsets_after", {}).get("y", old_y + float(computed.get("static_head_offset_delta_y_mm", 0.0))))
    angles = cfg.get("single_circle_verify_angles_deg", [0.0, 90.0, 180.0, 270.0])
    pairwise_context = is_pairwise_computed(computed)
    try:
        before = []
        after = []
        log("VerifyAB", "phase=current_active_geometry")
        for i in range(len(angles)):
            check_cancel("VerifyAB")
            before.append(measure_verify_ab_sample(camera, nozzle, part, cal_loc, cfg, state,
                                                   i, float(angles[i]), pairwise_context))
        apply_temporary_head_offsets(nozzle, new_x, new_y, "physical_ab_candidate_geometry")
        try:
            log("VerifyAB", "phase=candidate_head_offset_only")
            for i in range(len(angles)):
                check_cancel("VerifyAB")
                after.append(measure_verify_ab_sample(camera, nozzle, part, cal_loc, cfg, state,
                                                      i, float(angles[i]), pairwise_context))
        finally:
            restore_temporary_head_offsets(nozzle, old_offsets, "physical_ab_candidate_geometry")
        records = []
        for i in range(min(len(before), len(after))):
            b = before[i]
            a = after[i]
            bmag = float(b.get("error_mag_mm", 0.0))
            amag = float(a.get("error_mag_mm", 0.0))
            rec = {"angle_deg": float(b.get("command_angle_deg", angles[i])),
                   "before_error_x_mm": float(b.get("error_x_mm", 0.0)),
                   "before_error_y_mm": float(b.get("error_y_mm", 0.0)),
                   "before_error_mag_mm": bmag,
                   "after_error_x_mm": float(a.get("error_x_mm", 0.0)),
                   "after_error_y_mm": float(a.get("error_y_mm", 0.0)),
                   "after_error_mag_mm": amag,
                   "improvement_mm": bmag - amag}
            records.append(rec)
            log("VerifyAB", "angle=%.5f before=%.5f after=%.5f improvement=%.5f" %
                (float(rec["angle_deg"]), bmag, amag, float(rec["improvement_mm"])))
        before_stats = verification_stats_from_samples(before)
        after_stats = verification_stats_from_samples(after)
        passed = after_stats.get("rms_error_mm", 999999.0) <= before_stats.get("rms_error_mm", 0.0) and \
            after_stats.get("peak_error_mm", 999999.0) <= float(cfg.get("single_circle_max_model_peak_mm", 0.30))
        out.update({"verification_is_physical": True,
                    "before_samples": before,
                    "after_samples": after,
                    "verification_before_after_records": records,
                    "before_stats": before_stats,
                    "after_stats": after_stats,
                    "rms_improvement_mm": float(before_stats.get("rms_error_mm", 0.0)) - float(after_stats.get("rms_error_mm", 0.0)),
                    "peak_improvement_mm": float(before_stats.get("peak_error_mm", 0.0) or 0.0) - float(after_stats.get("peak_error_mm", 0.0) or 0.0),
                    "passed": bool(passed)})
        log("VerifyAB", "before_rms=%.5f after_rms=%.5f rms_improvement=%.5f passed=%s" %
            (float(before_stats.get("rms_error_mm", 0.0)), float(after_stats.get("rms_error_mm", 0.0)),
             float(out["rms_improvement_mm"]), str(bool(passed))))
        return out
    except Exception, e:
        try:
            restore_temporary_head_offsets(nozzle, old_offsets, "physical_ab_error")
        except:
            pass
        out["warning"] = str(e)
        out["passed"] = False
        log("VerifyAB", "WARNING physical verification failed: %s" % e)
        return out


def apply_physical_verification_gate(computed, physical_verify, context):
    if not bool_value(physical_verify.get("enabled", True)):
        log("VerifyAB", "context=%s gate_skipped=True enabled=False" % str(context))
        return computed
    before_stats = physical_verify.get("before_stats", {}) or {}
    after_stats = physical_verify.get("after_stats", {}) or {}
    before_rms = before_stats.get("rms_error_mm", None)
    after_rms = after_stats.get("rms_error_mm", None)
    worse = False
    if before_rms is not None and after_rms is not None:
        worse = float(after_rms) > float(before_rms)
    passed = bool_value(physical_verify.get("passed", False)) and not worse
    if not passed:
        reason = "physical AB candidate verification failed"
        if worse:
            reason = "physical AB candidate verification got worse"
        computed["safe_to_apply"] = False
        computed["persistent_apply_allowed"] = False
        computed["active_correction_allowed"] = False
        computed["calibrated_return_allowed"] = False
        # Preview candidate delta must remain available even when persistent
        # apply is blocked by physical verification.
        computed["candidate_delta_available"] = True
        computed["diagnostic_preview_allowed"] = True
        computed["persistent_apply_block_reason"] = reason
        computed["active_correction_block_reason"] = reason
        log("VerifyAB", "context=%s persistent_apply_allowed=False reason=%s" %
            (str(context), reason))
        log("VerifyAB", "diagnostic_preview_allowed=True")
        log("VerifyAB", "candidate_delta_available=True")
        log("VerifyAB", "candidate_delta_preserved_for_preview=True")
    return computed


def should_skip_bottom_square_for_xy_failure(computed, physical_verify):
    warning = str(physical_verify.get("warning", ""))
    physical_failed = bool_value(physical_verify.get("enabled", True)) and \
        not bool_value(physical_verify.get("passed", False))
    vision_failed = physical_failed and \
        ("raw_count" in warning or "expected 2" in warning or "No usable result" in warning)
    xy_candidate_valid = bool_value(computed.get("pairwise_xy_candidate_valid",
                                                computed.get("xy_fit_passed", False)))
    if vision_failed:
        return True, "xy_verification_failed"
    if not xy_candidate_valid:
        return True, "xy_candidate_invalid"
    return False, None


def run_post_square_static_xy_verification(camera, nozzle, part, cal_loc, cfg, state, computed):
    if bool_value(cfg.get("post_square_physical_pick_place_verify", False)):
        cfg2 = dict(cfg)
        cfg2["single_circle_verify_angles_deg"] = [0.0, 180.0]
        log("PostSquareVerify", "physical_pick_place_verify=True")
        result = run_physical_ab_verification(camera, nozzle, part, cal_loc, cfg2, state, computed)
        result["post_square_verification"] = True
        result["large_circle_center_only"] = True
        log("PostSquareVerify", "passed=%s" % str(bool_value(result.get("passed", False))))
        return result
    log("PostSquareVerify", "physical_pick_place_verify=False")
    log("PostSquareVerify", "non_destructive_top_camera_reacquire=True")
    anchor, anchor_source = resolve_cal_acquire_anchor(cal_loc, cfg, state, is_pairwise_computed(computed),
                                                       "PostSquareVerify")
    log("PostSquareVerify", "anchor_source=%s" % str(anchor_source))
    lock, measured = acquire_pose_for_location(camera, nozzle, part, anchor, cfg, "Cal")
    error_x = float(measured.getX() - cal_loc.getX())
    error_y = float(measured.getY() - cal_loc.getY())
    error_mag = math.sqrt(error_x * error_x + error_y * error_y)
    result = {"enabled": True,
              "post_square_verification": True,
              "verification_mode": "post_square_non_destructive_top_camera_reacquire",
              "verification_is_physical": False,
              "large_circle_center_only": True,
              "destructive_pick_place": False,
              "passed": True,
              "residual_error_x_mm": error_x,
              "residual_error_y_mm": error_y,
              "residual_error_mag_mm": error_mag,
              "detected_measurement_center": location_to_diag(measured),
              "detected_die_orientation_deg": float(lock["pose"].get("die_orientation_deg", 0.0)),
              "overlay": lock.get("overlay", None)}
    log("PostSquareVerify", "residual_error=(%.5f, %.5f, %.5f)" %
        (error_x, error_y, error_mag))
    return result


def log_compensation_stack():
    log("CompStack", "Existing OpenPnP nozzle/nozzle-tip compensation may be active.")
    log("CompStack", "This script measures residual behavior on top of active compensation.")
    log("CompStack", "Large circle XY fit stays in camera_only_large_center_circle_mm frame.")
    log("CompStack", "Persistent apply writes only the static nozzle head offset delta.")
    log("CompStack", "Dynamic runout remains separate and diagnostic unless explicit runtime compensation is enabled.")
    log("CompStack", "Small orientation circle is reserved for post-XY tip squaring only.")


def log_model_identity(cfg):
    log("Model", "mode=%s" % str(cfg.get("single_circle_model_mode", "openpnp_model_camera_offset")))
    log("Model", "fit_model_type=%s" % str(cfg.get("single_circle_fit_model_type", "openpnp_pairwise_180_offset")))
    log("Model", "upstream_behavior=CalibrationSolutions_calibrateNozzleOffsets_pairwise_180")
    log("Model", "persistent_write_allowed=True pairwise_xy_head_offset_only=True")


def java_class_name(obj):
    try:
        return obj.getClass().getName()
    except:
        return str(type(obj))


def median(values):
    clean = sorted([float(v) for v in values])
    if len(clean) < 1:
        return 0.0
    mid = len(clean) / 2
    if len(clean) % 2 == 1:
        return float(clean[mid])
    return (float(clean[mid - 1]) + float(clean[mid])) / 2.0


def get_bottom_camera(machine):
    for cam in machine.getCameras():
        try:
            if cam.getLooking() == SpiCamera.Looking.Up:
                return cam
        except:
            pass
    return None


def get_tip_calibration_pipeline(nozzle):
    try:
        tip = nozzle.getNozzleTip()
    except Exception, e:
        log("BottomSquare", "Nozzle tip lookup failed: %s" % e)
        return None
    if tip is None:
        log("BottomSquare", "No nozzle tip loaded on nozzle %s." % object_name(nozzle))
        return None
    try:
        calib = tip.getCalibration()
    except Exception, e:
        log("BottomSquare", "Nozzle tip calibration lookup failed for %s: %s" %
            (object_name(tip), e))
        return None
    if calib is None:
        log("BottomSquare", "No calibration on tip %s." % object_name(tip))
        return None
    try:
        pipeline = calib.getPipeline()
    except Exception, e:
        log("BottomSquare", "Nozzle tip pipeline lookup failed for %s: %s" %
            (object_name(tip), e))
        return None
    if pipeline is None:
        log("BottomSquare", "Nozzle tip calibration pipeline is None.")
        return None
    return pipeline


def first_model_item(model):
    if model is None:
        return None
    try:
        cname = java_class_name(model)
        if "RotatedRect" in cname or "KeyPoint" in cname:
            return model
    except:
        pass
    try:
        if hasattr(model, "size") and model.size() > 0:
            return model.get(0)
    except:
        pass
    try:
        if hasattr(model, "__len__") and len(model) > 0:
            return model[0]
    except:
        pass
    return None


def read_bottom_square_result_model(pipeline, stage_name):
    try:
        res = pipeline.getResult(stage_name)
    except:
        return None
    if res is None:
        return None
    try:
        return res.getModel()
    except:
        pass
    try:
        return res.model
    except:
        return None


def bottom_square_number_attr(obj, names):
    return number_attr(obj, names)


def bottom_square_item_geometry(item):
    if item is None:
        return None
    width = None
    height = None
    center_x = None
    center_y = None
    try:
        sz = item.size
        width = bottom_square_number_attr(sz, ["width", "getWidth"])
        height = bottom_square_number_attr(sz, ["height", "getHeight"])
    except:
        pass
    if width is None or height is None:
        try:
            width = bottom_square_number_attr(item, ["width", "getWidth"])
            height = bottom_square_number_attr(item, ["height", "getHeight"])
        except:
            pass
    try:
        center = item.center
        center_x = bottom_square_number_attr(center, ["x", "getX"])
        center_y = bottom_square_number_attr(center, ["y", "getY"])
    except:
        pass
    if center_x is None or center_y is None:
        try:
            center_x = bottom_square_number_attr(item, ["x", "getX"])
            center_y = bottom_square_number_attr(item, ["y", "getY"])
        except:
            pass
    if width is None or height is None:
        return None
    width = abs(float(width))
    height = abs(float(height))
    if width <= 0.000001 or height <= 0.000001:
        return None
    aspect = width / height if width >= height else height / width
    return {"width_px": width,
            "height_px": height,
            "aspect_ratio": aspect,
            "center_x_px": center_x,
            "center_y_px": center_y}


def validate_bottom_square_geometry(item, cfg, stage_name):
    geom = bottom_square_item_geometry(item)
    if geom is None:
        log("BottomSquare", "Rejected stage '%s': square result has no readable width/height." % stage_name)
        return None
    max_aspect = float(cfg.get("bottom_square_max_aspect_ratio", 1.20))
    log("BottomSquare", "rect_width_px=%.5f rect_height_px=%.5f aspect_ratio=%.5f" %
        (float(geom["width_px"]), float(geom["height_px"]), float(geom["aspect_ratio"])))
    if float(geom["aspect_ratio"]) > max_aspect:
        log("BottomSquare", "Rejected stage '%s': aspect_ratio %.5f > %.5f." %
            (stage_name, float(geom["aspect_ratio"]), max_aspect))
        return None
    return geom


def save_bottom_square_image_and_overlay(pipeline, capture_image, record, target, cfg):
    image = capture_image
    try:
        mat = pipeline.getWorkingImage()
        if mat is not None and not mat.empty():
            image = OpenCvUtils.toBufferedImage(mat)
    except:
        pass
    try:
        if image is not None:
            ImageIO.write(image, "png", File(last_bottom_image_path()))
            log("BottomSquare", "saved_bottom_image=%s" % last_bottom_image_path())
    except Exception, e:
        log("BottomSquare", "Could not save bottom capture image: %s" % e)
    try:
        overlay = {"image_path": last_bottom_image_path(),
                   "target": location_to_diag(target),
                   "record": record,
                   "timestamp": time.time()}
        f = open(last_bottom_overlay_path(), "w")
        try:
            f.write(json.dumps(overlay, sort_keys=True, indent=2))
        finally:
            f.close()
        log("BottomSquare", "saved_bottom_overlay_json=%s" % last_bottom_overlay_path())
    except Exception, e2:
        log("BottomSquare", "Could not save bottom overlay json: %s" % e2)


def raw_angle_from_square_item(item):
    if item is None:
        return None
    cname = java_class_name(item)
    log("BottomSquare", "Inspecting vision result item type %s: %s" % (cname, item))
    if "RotatedRect" in cname:
        try:
            return float(item.angle)
        except Exception, e:
            log("BottomSquare", "RotatedRect had no readable angle: %s" % e)
            return None
    if "KeyPoint" in cname:
        try:
            angle = float(item.angle)
            if angle == -1.0:
                log("BottomSquare", "Ignoring KeyPoint angle -1.0000; unknown orientation.")
                return None
            return angle
        except Exception, e:
            log("BottomSquare", "KeyPoint had no readable angle: %s" % e)
            return None
    try:
        return float(item.angle)
    except:
        pass
    try:
        text = str(item)
        marker = text.rfind("*")
        if marker >= 0:
            tail = text[marker + 1:].replace("}", " ").strip()
            return float(tail.split()[0])
    except:
        pass
    return None


def square_correction_degrees(raw_angle):
    correction = ((float(raw_angle) + 45.0) % 90.0) - 45.0
    if correction == -45.0 and float(raw_angle) > 0.0:
        correction = 45.0
    return correction


def bottom_square_bias_valid_from_result(bottom_square):
    if bottom_square is None:
        return False
    if "square_rotation_bias_valid" in bottom_square:
        return bool_value(bottom_square.get("square_rotation_bias_valid", False))
    return bool_value(bottom_square.get("verification_passed", False)) and \
        bottom_square.get("square_rotation_bias_deg", None) is not None


def effective_square_bias_from_result(computed, cfg, context):
    bottom_square = computed.get("bottom_square", {}) or {}
    valid = bottom_square_bias_valid_from_result(bottom_square)
    raw_value = computed.get("square_rotation_bias_deg", None)
    if raw_value is None:
        raw_value = bottom_square.get("square_rotation_bias_deg",
                                      bottom_square.get("selected_correction_deg", 0.0))
    if raw_value is None:
        raw_value = 0.0
    raw_bias = float(raw_value)
    effective = raw_bias if bool(valid) else 0.0
    log(context, "square_rotation_bias_valid=%s" % str(bool(valid)))
    log(context, "effective_square_bias_deg=%.5f" % float(effective))
    log(context, "rejected_invalid_square_bias=%s" % str(bool(not valid and abs(raw_bias) > 0.000001)))
    if str(context) != "RotationPlan":
        log("RotationPlan", "square_rotation_bias_valid=%s" % str(bool(valid)))
        log("RotationPlan", "effective_square_bias_deg=%.5f" % float(effective))
        log("RotationPlan", "rejected_invalid_square_bias=%s" %
            str(bool(not valid and abs(raw_bias) > 0.000001)))
    return effective, bool(valid), raw_bias


def log_nozzle_rotation_state(nozzle, bottom_square):
    try:
        current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        machine_r = float(current.getRotation())
    except:
        machine_r = 0.0
    valid = bottom_square_bias_valid_from_result(bottom_square)
    visual_tip = bottom_square.get("measurement_angle_deg",
                                   bottom_square.get("selected_correction_deg", None))
    if visual_tip is None:
        visual_tip = 0.0
    software_zero_matches_visual_zero = bool(valid) and \
        abs(float(bottom_square.get("square_rotation_bias_deg", 0.0) or 0.0)) <= \
        float(DEFAULT_CONFIG.get("bottom_square_max_post_apply_abs_deg", 1.0))
    log("NozzleRotationState", "machine_r_deg=%.5f" % float(machine_r))
    log("NozzleRotationState", "visual_tip_angle_deg=%.5f" % float(visual_tip))
    log("NozzleRotationState", "square_bias_valid=%s" % str(bool(valid)))
    log("NozzleRotationState", "software_zero_matches_visual_zero=%s" %
        str(bool(software_zero_matches_visual_zero)))


def log_rotation_state(nozzle, detected_die_orientation_deg, desired_die_orientation_deg,
                       square_bias_valid, square_bias_deg):
    try:
        current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        machine_r = float(current.getRotation())
    except:
        machine_r = 0.0
    software_zero_matches_visual_zero = bool(square_bias_valid) and \
        abs(float(square_bias_deg or 0.0)) <= \
        float(DEFAULT_CONFIG.get("bottom_square_max_post_apply_abs_deg", 1.0))
    log("RotationState", "machine_nozzle_r_deg=%.5f" % float(machine_r))
    log("RotationState", "detected_die_orientation_deg=%.5f" % float(detected_die_orientation_deg))
    log("RotationState", "desired_die_orientation_deg=%.5f" % float(desired_die_orientation_deg))
    log("RotationState", "square_bias_valid=%s" % str(bool(square_bias_valid)))
    log("RotationState", "software_zero_matches_visual_zero=%s" %
        str(bool(software_zero_matches_visual_zero)))
    log("RotationState", "note=die_orientation_and_nozzle_rotation_are_not_the_same_quantity")


def bottom_square_stage_names(cfg):
    names = [str(cfg.get("bottom_square_stage_name", "squareResults"))]
    for name in cfg.get("bottom_square_stage_fallbacks", []):
        name = str(name)
        if name not in names:
            names.append(name)
    return names


def bottom_square_z(bottom_camera, cfg):
    cam_loc = bottom_camera.getLocation().convertToUnits(LengthUnit.Millimeters)
    source = str(cfg.get("bottom_square_nominal_z_source", "bottom_camera_location")).strip().lower()
    if source == "safe_z":
        return float(cfg.get("bottom_square_safe_z", 28.13))
    try:
        return float(cam_loc.getZ())
    except:
        return float(cfg.get("bottom_square_safe_z", 28.13))


def read_bottom_square_angle_from_pipeline(pipeline, cfg):
    checked = []
    for stage_name in bottom_square_stage_names(cfg):
        if stage_name in checked:
            continue
        checked.append(stage_name)
        model = read_bottom_square_result_model(pipeline, stage_name)
        if model is None:
            log("BottomSquare", "No model from stage '%s'." % stage_name)
            continue
        item = first_model_item(model)
        if item is None:
            log("BottomSquare", "Model from stage '%s' is empty or not indexable." % stage_name)
            continue
        geom = validate_bottom_square_geometry(item, cfg, stage_name)
        if geom is None:
            continue
        raw_angle = raw_angle_from_square_item(item)
        if raw_angle is None:
            log("BottomSquare", "No usable angle from stage '%s'." % stage_name)
            continue
        correction = square_correction_degrees(raw_angle)
        near_square = float(geom.get("aspect_ratio", 1.0)) < \
            float(cfg.get("bottom_square_min_orientation_aspect_ratio", 1.15))
        near_45 = abs(abs(float(correction)) - 45.0) <= \
            float(cfg.get("bottom_square_near_45_tolerance_deg", 3.0))
        if near_45 and bool_value(cfg.get("bottom_square_treat_near_45_as_ambiguous", False)):
            log("BottomSquare", "near_45_correction_detected=True correction_deg=%.5f" % float(correction))
        reject_near_45 = bool_value(cfg.get("bottom_square_reject_near_45_as_ambiguous",
                                            cfg.get("bottom_square_reject_ambiguous_45", False)))
        ambiguous_reason = None
        if near_square and bool_value(cfg.get("bottom_square_reject_near_square_orientation", True)):
            ambiguous_reason = "near_square_rotated_rect_angle_is_not_reliable"
        elif near_45 and reject_near_45:
            ambiguous_reason = "near_45_square_correction_is_ambiguous"
        signed = float(cfg.get("bottom_square_angle_sign", 1.0)) * correction
        if ambiguous_reason is not None:
            log("BottomSquare", "orientation_ambiguous=True")
            log("BottomSquare", "reason=%s" % ambiguous_reason)
            log("BottomSquare", "rect_width_px=%.5f" % float(geom["width_px"]))
            log("BottomSquare", "rect_height_px=%.5f" % float(geom["height_px"]))
            log("BottomSquare", "aspect_ratio=%.5f" % float(geom["aspect_ratio"]))
            log("BottomSquare", "raw_angle_deg=%.5f" % float(raw_angle))
            log("BottomSquare", "rejected_square_bias_deg=%.5f" % float(signed))
            log("BottomSquare", "square_rotation_bias_valid=False")
            log("BottomSquare", "Rejected stage '%s': %s." % (stage_name, ambiguous_reason))
            return {"raw_angle_deg": float(raw_angle),
                    "correction_deg": float(correction),
                    "signed_correction_deg": float(signed),
                    "rejected_square_bias_deg": float(signed),
                    "square_rotation_bias_valid": False,
                    "orientation_ambiguous": True,
                    "ambiguous_reason": ambiguous_reason,
                    "stage_name": stage_name,
                    "item_type": java_class_name(item),
                    "rect": geom,
                    "valid": False,
                    "warning": ambiguous_reason}
        item_type = java_class_name(item)
        log("BottomSquare", "stage=%s" % stage_name)
        log("BottomSquare", "raw_angle_deg=%.5f" % raw_angle)
        log("BottomSquare", "square_correction_deg=%.5f" % correction)
        log("BottomSquare", "signed_correction_deg=%.5f" % signed)
        log("BottomSquare", "square_rotation_bias_valid=True")
        return {"raw_angle_deg": float(raw_angle),
                "correction_deg": float(correction),
                "signed_correction_deg": float(signed),
                "square_rotation_bias_valid": True,
                "stage_name": stage_name,
                "item_type": item_type,
                "rect": geom}
    return None


def measure_bottom_square_angle_at(nozzle, bottom_camera, pipeline, x_offset_mm, y_offset_mm, cfg, tag):
    cam_loc = bottom_camera.getLocation().convertToUnits(LengthUnit.Millimeters)
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    target = mm_loc(cam_loc.getX() + float(x_offset_mm),
                    cam_loc.getY() + float(y_offset_mm),
                    bottom_square_z(bottom_camera, cfg),
                    current.getRotation())
    axis = "X" if abs(float(x_offset_mm)) >= abs(float(y_offset_mm)) else "Y"
    if abs(float(y_offset_mm)) < 0.000001:
        axis = "X"
    if abs(float(x_offset_mm)) < 0.000001 and abs(float(y_offset_mm)) >= 0.000001:
        axis = "Y"
    log("BottomSquareScan", "axis=%s offset=%.5f target=(%.5f, %.5f, %.5f, %.5f) tag=%s" %
        (axis, float(x_offset_mm) if axis == "X" else float(y_offset_mm),
         target.getX(), target.getY(), target.getZ(), target.getRotation(), str(tag)))
    base.move_to_with_speed(nozzle, target, cfg)
    sleep_ms(cfg.get("bottom_square_settle_ms", 1000))
    try:
        pipeline.setProperty("camera", bottom_camera)
    except Exception, e:
        log("BottomSquare", "WARNING could not set pipeline property camera: %s" % e)
    try:
        pipeline.setProperty("nozzle", nozzle)
    except:
        pass
    last_error = None
    for attempt in range(1, int(float(cfg.get("bottom_square_pipeline_attempts", 3))) + 1):
        try:
            capture = None
            try:
                capture = bottom_camera.settleAndCapture()
            except:
                pass
            pipeline.process()
            angle = read_bottom_square_angle_from_pipeline(pipeline, cfg)
            if angle is not None:
                record = dict(angle)
                record["x_offset_mm"] = float(x_offset_mm)
                record["y_offset_mm"] = float(y_offset_mm)
                record["axis"] = axis
                record["tag"] = str(tag)
                record["valid"] = bool_value(record.get("valid", True)) and \
                    bool_value(record.get("square_rotation_bias_valid", True))
                save_bottom_square_image_and_overlay(pipeline, capture, record, target, cfg)
                if bool_value(record.get("valid", False)):
                    log("BottomSquareScan", "axis=%s offset=%.5f angle=%.5f correction=%.5f" %
                        (axis, float(x_offset_mm) if axis == "X" else float(y_offset_mm),
                         float(record["raw_angle_deg"]), float(record["signed_correction_deg"])))
                else:
                    log("BottomSquareScan", "axis=%s offset=%.5f rejected_square_bias_deg=%.5f reason=%s" %
                        (axis, float(x_offset_mm) if axis == "X" else float(y_offset_mm),
                         float(record.get("rejected_square_bias_deg",
                                          record.get("signed_correction_deg", 0.0))),
                         str(record.get("ambiguous_reason", record.get("warning", "invalid")))))
                return record
            last_error = "no usable angle"
        except Exception, e:
            last_error = str(e)
            log("BottomSquareScan", "attempt=%d failed: %s" % (attempt, e))
        sleep_ms(cfg.get("bottom_square_settle_ms", 1000))
    return {"x_offset_mm": float(x_offset_mm),
            "y_offset_mm": float(y_offset_mm),
            "axis": axis,
            "tag": str(tag),
            "valid": False,
            "warning": str(last_error)}


def run_bottom_square_scan(nozzle, bottom_camera, pipeline, cfg, verify_mode):
    records = []
    if bool(verify_mode):
        offsets = cfg.get("bottom_square_verify_offsets_mm", [-0.30, 0.0, 0.30])
        tag = "verify"
        x_offsets = offsets
        y_offsets = offsets
    else:
        tag = "pre_apply"
        x_offsets = cfg.get("bottom_square_x_offsets_mm", [-0.50, -0.25, 0.0, 0.25, 0.50])
        y_offsets = cfg.get("bottom_square_y_offsets_mm", [-0.50, -0.25, 0.0, 0.25, 0.50])
    for offset in x_offsets:
        check_cancel("BottomSquare")
        records.append(measure_bottom_square_angle_at(nozzle, bottom_camera, pipeline, offset, 0.0, cfg, tag))
    for offset in y_offsets:
        check_cancel("BottomSquare")
        records.append(measure_bottom_square_angle_at(nozzle, bottom_camera, pipeline, 0.0, offset, cfg, tag))
    return records


def bottom_square_stats(records, cfg):
    valid = [r for r in records if bool(r.get("valid", False)) and "signed_correction_deg" in r]
    ambiguous = [r for r in records if bool_value(r.get("orientation_ambiguous", False))]
    values = [float(r["signed_correction_deg"]) for r in valid]
    stats = {"valid_count": len(valid),
             "record_count": len(records),
             "ambiguous_count": len(ambiguous),
             "correction_mean_deg": None,
             "correction_median_deg": None,
             "correction_min_deg": None,
             "correction_max_deg": None,
             "correction_range_deg": None,
             "x_scan_range_deg": None,
             "y_scan_range_deg": None}
    if len(values) < 1:
        return stats
    stats["correction_mean_deg"] = mean(values)
    stats["correction_median_deg"] = median(values)
    stats["correction_min_deg"] = min(values)
    stats["correction_max_deg"] = max(values)
    stats["correction_range_deg"] = max(values) - min(values)
    for axis in ["X", "Y"]:
        axis_values = [float(r["signed_correction_deg"]) for r in valid if str(r.get("axis", "")) == axis]
        key = "x_scan_range_deg" if axis == "X" else "y_scan_range_deg"
        if len(axis_values) > 0:
            stats[key] = max(axis_values) - min(axis_values)
    if bool_value(cfg.get("bottom_square_use_median_angle", True)):
        stats["selected_correction_deg"] = stats["correction_median_deg"]
    else:
        stats["selected_correction_deg"] = stats["correction_mean_deg"]
    return stats


def bottom_square_handle_warning(message, cfg, warnings, reject):
    warnings.append(message)
    log("BottomSquareFit", "WARNING %s" % message)
    if bool(reject) and not bool_value(cfg.get("bottom_square_warning_only", True)):
        raise Exception(message)


def choose_bottom_square_correction(records, cfg):
    warnings = []
    stats = bottom_square_stats(records, cfg)
    if int(stats.get("valid_count", 0)) < 1:
        if int(stats.get("ambiguous_count", 0)) > 0 and \
           int(stats.get("ambiguous_count", 0)) == int(stats.get("record_count", 0)):
            bottom_square_handle_warning("all bottom camera square measurements rejected as orientation ambiguous",
                                         cfg, warnings, True)
        else:
            bottom_square_handle_warning("no valid bottom camera square measurements", cfg, warnings, True)
        stats["warnings"] = warnings
        stats["measurement_valid"] = False
        return {"selected_correction_deg": 0.0,
                "square_rotation_bias_valid": False,
                "measurement_valid": False,
                "stats": stats,
                "warnings": warnings}
    selected = float(stats.get("selected_correction_deg", 0.0))
    position_dependent = False
    if float(stats.get("correction_range_deg", 0.0)) > float(cfg.get("bottom_square_max_angle_range_deg", 2.0)):
        position_dependent = True
        bottom_square_handle_warning("bottom camera square angle depends on XY position", cfg, warnings,
                                     bool_value(cfg.get("bottom_square_reject_if_position_dependent", False)))
    if abs(selected) > float(cfg.get("bottom_square_max_angle_abs_deg", 20.0)):
        bottom_square_handle_warning("bottom square correction is large but may be valid", cfg, warnings, False)
    stats["position_dependent_warning"] = bool(position_dependent)
    stats["warnings"] = warnings
    log("BottomSquareFit", "selected_correction_deg=%.5f" % selected)
    log("BottomSquareFit", "correction_range_deg=%.5f" % float(stats.get("correction_range_deg", 0.0)))
    log("BottomSquareFit", "x_scan_range_deg=%.5f" % float(stats.get("x_scan_range_deg", 0.0) or 0.0))
    log("BottomSquareFit", "y_scan_range_deg=%.5f" % float(stats.get("y_scan_range_deg", 0.0) or 0.0))
    log("BottomSquareFit", "position_dependent_warning=%s" % str(bool(position_dependent)))
    stats["measurement_valid"] = True
    return {"selected_correction_deg": selected,
            "square_rotation_bias_valid": True,
            "measurement_valid": True,
            "stats": stats,
            "warnings": warnings}


def bottom_square_pre_apply_passed(choice, cfg):
    stats = choice.get("stats", {}) or {}
    selected = float(choice.get("selected_correction_deg", 0.0))
    if not bool_value(choice.get("measurement_valid", False)) or \
       not bool_value(choice.get("square_rotation_bias_valid", False)):
        return False, "bottom square orientation measurement is invalid"
    if int(stats.get("valid_count", 0)) < 1:
        return False, "no valid bottom camera square measurements"
    if float(stats.get("correction_range_deg", 0.0) or 0.0) > float(cfg.get("bottom_square_max_angle_range_deg", 2.0)):
        return False, "bottom camera square angle depends on XY position"
    if abs(selected) > float(cfg.get("bottom_square_max_angle_abs_deg", 90.0)):
        return False, "bottom square correction exceeds bottom_square_max_angle_abs_deg"
    return True, None


def find_bottom_square_rot_axis(machine):
    axes = machine.getAxes()
    chosen = None
    for axis in axes:
        axis_id = axis.getId() or "<?>"
        name = axis.getName() or "<?>"
        atype = str(axis.getType() or "<?>")
        log("BottomSquareApply", "Axis discovered: id='%s', name='%s', type=%s" %
            (axis_id, name, atype))
        if name.lower() == "a":
            return axis
        if chosen is None and "ROTATION" in atype.upper():
            chosen = axis
    return chosen


def apply_bottom_square_correction(machine, nozzle, correction_deg, cfg):
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    current_r = float(current.getRotation())
    target_r = current_r + float(correction_deg)
    applied = False
    moved_rotation = False
    visual_zeroed = False
    global_offsets_called = False
    square_apply_mode = str(cfg.get("bottom_square_apply_mode", "script_local_rotation_bias")).strip().lower()
    log("BottomSquareApply", "current_r=%.5f" % current_r)
    log("BottomSquareApply", "correction_deg=%.5f" % float(correction_deg))
    log("BottomSquareApply", "target_r=%.5f" % target_r)
    if bool_value(cfg.get("bottom_square_apply_motion_correction", True)):
        base.move_to_with_speed(nozzle, mm_loc(current.getX(), current.getY(), current.getZ(), target_r), cfg)
        wait_still(nozzle)
        applied = True
        moved_rotation = abs(float(correction_deg)) > 0.000001
    log("BottomSquareApply", "square_rotation_apply_mode=%s" % square_apply_mode)
    log("BottomSquareApply", "set_global_rotation_offset_requested=%s ignored=True" %
        str(bool_value(cfg.get("bottom_square_set_global_rotation_offset", True))))
    log("BottomSquareApply", "global_offsets_called=False reason=forbidden_for_nozzle_specific_squaring")
    log("BottomSquareApply", "visual_rotation_zeroed=False")
    changed_state = bool(moved_rotation or global_offsets_called)
    log("BottomSquare", "changed_rotation_state=%s" % str(bool(changed_state)))
    log("BottomSquare", "rotation_state_changed_before_top_camera_fit=%s" % str(bool(changed_state)))
    return {"applied_motion_correction": bool(applied),
            "bottom_square_apply_motion_correction_moved_rotation": bool(moved_rotation),
            "set_global_rotation_offset": False,
            "global_offsets_called": bool(global_offsets_called),
            "visual_rotation_zeroed": bool(visual_zeroed),
            "square_rotation_bias_deg": float(correction_deg),
            "square_rotation_bias_valid": True,
            "bottom_square_rotation_valid": True,
            "measurement_valid": True,
            "square_rotation_apply_mode": square_apply_mode,
            "square_rotation_bias_scope": "script_local_nozzle_specific",
            "bottom_square_changed_rotation_state": bool(changed_state),
            "top_camera_correction_valid_after_bottom_square": True,
            "requires_post_square_remeasure": False}


def run_bottom_camera_nozzle_square(machine, nozzle, cfg):
    result = {"enabled": bool_value(cfg.get("run_bottom_camera_nozzle_square_after_fit", True)),
              "ran": False,
              "selected_correction_deg": None,
              "square_rotation_bias_deg": 0.0,
              "square_rotation_bias_valid": False,
              "bottom_square_rotation_valid": False,
              "measurement_valid": False,
              "safe_return_allowed": True,
              "square_rotation_apply_mode": str(cfg.get("bottom_square_apply_mode", "script_local_rotation_bias")),
              "square_rotation_bias_scope": "script_local_nozzle_specific",
              "applied_motion_correction": False,
              "bottom_square_apply_motion_correction_moved_rotation": False,
              "set_global_rotation_offset": False,
              "global_offsets_called": False,
              "visual_rotation_zeroed": False,
              "bottom_square_changed_rotation_state": False,
              "top_camera_correction_valid_after_bottom_square": True,
              "requires_post_square_remeasure": False,
              "pre_apply_records": [],
              "pre_apply_stats": {},
              "post_apply_records": [],
              "post_apply_stats": {},
              "verification_passed": None,
              "warnings": []}
    if not bool_value(result["enabled"]):
        log("BottomSquare", "enabled=False")
        return result
    log("BottomSquare", "enabled=True")
    try:
        bottom_camera = get_bottom_camera(machine)
        if bottom_camera is None:
            raise Exception("No bottom camera found.")
        bottom_loc = bottom_camera.getLocation().convertToUnits(LengthUnit.Millimeters)
        log("BottomSquare", "moving_to_bottom_camera=True")
        log("BottomSquare", "bottom_camera_location=(%.5f, %.5f, %.5f, %.5f)" %
            (float(bottom_loc.getX()), float(bottom_loc.getY()), float(bottom_loc.getZ()), float(bottom_loc.getRotation())))
        pipeline = get_tip_calibration_pipeline(nozzle)
        if pipeline is None:
            raise Exception("No nozzle tip calibration pipeline found.")
        log("BottomSquare", "moving nozzle to bottom camera")
        pre_records = run_bottom_square_scan(nozzle, bottom_camera, pipeline, cfg, False)
        result["pre_apply_records"] = pre_records
        choice = choose_bottom_square_correction(pre_records, cfg)
        result["pre_apply_stats"] = choice["stats"]
        result["warnings"].extend(choice["warnings"])
        selected = float(choice["selected_correction_deg"])
        result["selected_correction_deg"] = selected
        result["square_rotation_bias_valid"] = bool_value(choice.get("square_rotation_bias_valid", False))
        result["measurement_valid"] = bool_value(choice.get("measurement_valid", False))
        result["bottom_square_rotation_valid"] = bool_value(result["square_rotation_bias_valid"])
        result["square_rotation_bias_deg"] = selected if bool_value(result["square_rotation_bias_valid"]) else 0.0
        result["measurement_angle_deg"] = float((choice.get("stats", {}) or {}).get("correction_median_deg", selected) or selected)
        log("BottomSquare", "measurement_angle_deg=%.5f" %
            float(result["measurement_angle_deg"]))
        log("BottomSquare", "selected_square_bias_deg=%.5f" % selected)
        pre_apply_passed, pre_apply_reason = bottom_square_pre_apply_passed(choice, cfg)
        result["pre_apply_validation_passed"] = bool(pre_apply_passed)
        result["pre_apply_validation_reason"] = pre_apply_reason
        log("BottomSquareFit", "pre_apply_validation_passed=%s" % str(bool(pre_apply_passed)))
        if pre_apply_reason is not None:
            log("BottomSquareFit", "pre_apply_validation_reason=%s" % str(pre_apply_reason))
        if pre_apply_passed and bool_value(result.get("square_rotation_bias_valid", False)):
            apply_result = apply_bottom_square_correction(machine, nozzle, selected, cfg)
            result.update(apply_result)
        else:
            warning = "bottom square pre-apply validation failed; rotation/global zero not applied"
            result["warnings"].append(warning)
            log("BottomSquareFit", "WARNING %s" % warning)
            if not bool_value(cfg.get("bottom_square_warning_only", True)):
                raise Exception(warning)
        if bool_value(cfg.get("bottom_square_verify_after_apply", True)) and \
           bool_value(result.get("applied_motion_correction", False) or result.get("global_offsets_called", False)):
            post_records = run_bottom_square_scan(nozzle, bottom_camera, pipeline, cfg, True)
            post_stats = bottom_square_stats(post_records, cfg)
            result["post_apply_records"] = post_records
            result["post_apply_stats"] = post_stats
            post_median = float(post_stats.get("correction_median_deg", 0.0) or 0.0)
            post_range = float(post_stats.get("correction_range_deg", 0.0) or 0.0)
            passed = abs(post_median) <= float(cfg.get("bottom_square_max_post_apply_abs_deg", 1.0)) and \
                post_range <= float(cfg.get("bottom_square_max_angle_range_deg", 2.0)) and \
                int(post_stats.get("valid_count", 0)) > 0
            result["verification_passed"] = bool(passed)
            log("BottomSquare", "verification_passed=%s" % str(bool(passed)))
            log("BottomSquareVerify", "post_apply_median_correction_deg=%.5f" % post_median)
            log("BottomSquareVerify", "post_apply_range_deg=%.5f" % post_range)
            log("BottomSquareVerify", "passed=%s" % str(bool(passed)))
            if not passed:
                warning = "bottom square post-apply verification failed"
                result["warnings"].append(warning)
                log("BottomSquareVerify", "WARNING %s" % warning)
                if not bool_value(cfg.get("bottom_square_warning_only", True)):
                    raise Exception(warning)
        elif bool_value(cfg.get("bottom_square_verify_after_apply", True)):
            result["verification_passed"] = False
            log("BottomSquareVerify", "skipped because bottom square correction was not applied")
            log("BottomSquare", "verification_passed=False")
        if not bool_value(result.get("square_rotation_bias_valid", False)):
            result["verification_passed"] = False
            result["measurement_valid"] = False
            result["bottom_square_rotation_valid"] = False
            result["square_rotation_bias_deg"] = 0.0
            result["selected_correction_deg"] = 0.0
            result["bottom_square_changed_rotation_state"] = False
            result["bottom_square_apply_motion_correction_moved_rotation"] = False
            result["applied_motion_correction"] = False
            result["safe_return_allowed"] = True
            log("BottomSquare", "measurement_valid=False")
            log("BottomSquare", "square_rotation_bias_valid=False")
        result["ran"] = True
        log_nozzle_rotation_state(nozzle, result)
        return result
    except Exception, e:
        result["warnings"].append(str(e))
        log("BottomSquare", "WARNING %s" % e)
        if not bool_value(cfg.get("bottom_square_warning_only", True)):
            raise
        return result


def run_post_calibration_orientation_square(camera, nozzle, part, cal_loc, cfg, state):
    result = {"enabled": bool_value(cfg.get("run_bottom_camera_nozzle_square_after_fit", True)),
              "ran": False,
              "method": "top_camera_small_orientation_circle",
              "orientation_circle_used_for_xy": False,
              "orientation_circle_used_for_model_fit": False,
              "orientation_circle_used_for_square": True,
              "square_rotation_bias_deg": None,
              "square_rotation_bias_valid": False,
              "measurement_valid": False,
              "safe_return_allowed": True,
              "square_rotation_apply_mode": str(cfg.get("bottom_square_apply_mode", "script_local_rotation_bias")),
              "square_rotation_bias_scope": "script_local_nozzle_specific",
              "global_offsets_called": False,
              "set_global_rotation_offset": False,
              "applied_motion_correction": False,
              "bottom_square_apply_motion_correction_moved_rotation": False,
              "bottom_square_changed_rotation_state": False,
              "verification_passed": None,
              "pre_apply_records": [],
              "post_apply_records": [],
              "warnings": []}
    if not bool_value(result["enabled"]):
        log("MarkerSquare", "enabled=False")
        return result
    try:
        log("MarkerSquare", "phase=measure_small_orientation_circle_after_xy_fit")
        anchor, anchor_source = resolve_cal_acquire_anchor(
            cal_loc, cfg, state, bool_value(state.get("pairwise_walked_die_location_active", False)), "MarkerSquare")
        log("MarkerSquare", "anchor_source=%s" % str(anchor_source))
        lock, detected = acquire_pose_for_location(camera, nozzle, part, anchor, cfg, "Cal")
        pose = lock["pose"]
        current_orientation = float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0)))
        desired = nearest_allowed_orientation(current_orientation, [0.0, 90.0, 180.0, -90.0])
        bias = normalize_angle(desired - current_orientation)
        result["pre_apply_records"].append({
            "detected_die_orientation_deg": current_orientation,
            "desired_square_orientation_deg": desired,
            "square_rotation_bias_deg": bias,
            "valid": True})
        result["selected_correction_deg"] = bias
        result["square_rotation_bias_deg"] = bias
        result["square_rotation_bias_valid"] = True
        result["measurement_valid"] = True
        log("MarkerSquare", "detected_die_orientation_deg=%.5f" % current_orientation)
        log("MarkerSquare", "desired_square_orientation_deg=%.5f" % desired)
        log("MarkerSquare", "script_local_square_rotation_bias_deg=%.5f" % bias)
        log("MarkerSquare", "global_offsets_called=False")
        if bool_value(cfg.get("bottom_square_apply_motion_correction", True)) and \
                str(cfg.get("bottom_square_apply_mode", "script_local_rotation_bias")).strip().lower() == "script_local_rotation_bias":
            current = state.get("current_die_loc", cal_loc)
            current = force_cal_base_z(current, cfg)
            detected_base = measured_xy_as_base_loc(detected, "Cal", cfg)
            pick_base = mm_loc(detected_base.getX(), detected_base.getY(), current.getZ(), current.getRotation())
            pick_die_from_existing_lock(camera, nozzle, part, pick_base, cfg, "Cal", state,
                                        lock, detected, None, "return")
            after_pick = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            target_r = float(after_pick.getRotation()) + bias
            rotate_target = single_circle_rotation_location(
                nozzle, mm_loc(after_pick.getX(), after_pick.getY(), after_pick.getZ(), target_r),
                cfg, "MarkerSquareRotate")
            base.move_to_with_speed(nozzle, rotate_target, cfg)
            place_target = mm_loc(anchor.getX(), anchor.getY(), anchor.getZ(), target_r)
            place_die_at(nozzle, part, place_target, cfg, "Cal", state)
            result["applied_motion_correction"] = True
            result["bottom_square_apply_motion_correction_moved_rotation"] = abs(bias) > 0.000001
            result["bottom_square_changed_rotation_state"] = abs(bias) > 0.000001
            lock2, detected2 = acquire_pose_for_location(camera, nozzle, part, anchor, cfg, "Cal")
            pose2 = lock2["pose"]
            final_orientation = float(pose2.get("die_orientation_deg", pose2.get("orientation_raw_deg", 0.0)))
            final_error = normalize_angle(final_orientation - desired)
            passed = abs(final_error) <= float(cfg.get("return_orientation_tolerance_deg", 10.0))
            result["post_apply_records"].append({
                "final_die_orientation_deg": final_orientation,
                "desired_square_orientation_deg": desired,
                "orientation_error_deg": final_error,
                "valid": True})
            result["verification_passed"] = bool(passed)
            log("MarkerSquareVerify", "final_die_orientation_deg=%.5f" % final_orientation)
            log("MarkerSquareVerify", "orientation_error_deg=%.5f" % final_error)
            log("MarkerSquareVerify", "passed=%s" % str(bool(passed)))
        else:
            result["verification_passed"] = None
            log("MarkerSquare", "apply_mode=preview_only")
        result["ran"] = True
        return result
    except Exception, e:
        result["warnings"].append(str(e))
        log("MarkerSquare", "WARNING %s" % e)
        if not bool_value(cfg.get("bottom_square_warning_only", True)):
            raise
        return result


def relocate_die_after_bottom_square(camera, nozzle, part, cal_loc, cfg, state):
    current = state.get("current_die_loc", None)
    if current is None:
        current = cal_loc
    before_z = current.convertToUnits(LengthUnit.Millimeters).getZ()
    current = force_cal_base_z(current, cfg)
    log("PostSquareDieRelocate", "acquiring die at calibration table")
    log("PostSquareDieRelocate", "moving_top_camera_to_cal_location=True")
    log("BaseZGuard", "context=PostSquareDieRelocate")
    log("BaseZGuard", "expected_loc_z_before=%.5f" % float(before_z))
    log("BaseZGuard", "expected_loc_z_after=%.5f" % float(current.getZ()))
    log("BaseZGuard", "part_height_z=%.5f" % float(part_height_z_mm(cfg)))
    log("BaseZGuard", "expected_focus_z=%.5f" % float(base.pick_surface_z_mm(current, cfg)))
    lock, detected = acquire_pose_for_location(camera, nozzle, part, current, cfg, "Cal")
    pose = lock["pose"]
    state["current_die_loc"] = base_loc_with_measured_orientation(detected, pose, "Cal", cfg)
    state["current_die_loc_trusted"] = True
    state["last_trusted_die_loc"] = state["current_die_loc"]
    log("PostSquareDieRelocate", "measured_center=(%.5f, %.5f)" %
        (detected.getX(), detected.getY()))
    log("PostSquareDieRelocate", "detected_die_orientation_deg=%.5f" %
        float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0))))
    log("PostSquareDieRelocate", "using_fresh_pose=True")
    return lock, detected


def run_post_square_single_angle_recenter(camera, nozzle, part, cal_loc, cfg, state, relock):
    result = {"enabled": bool_value(cfg.get("run_post_square_single_angle_recenter", True)),
              "ran": False,
              "correction_source": None,
              "post_square_static_delta_x_mm": 0.0,
              "post_square_static_delta_y_mm": 0.0,
              "post_square_static_delta_mag_mm": 0.0,
              "measured_error_x_mm": None,
              "measured_error_y_mm": None}
    if not bool_value(result["enabled"]):
        log("PostSquareRecenter", "enabled=False")
        return result
    if relock is None or relock.get("lock", None) is None or relock.get("detected", None) is None:
        lock, detected = relocate_die_after_bottom_square(camera, nozzle, part, cal_loc, cfg, state)
    else:
        lock = relock["lock"]
        detected = relock["detected"]
    pose = lock["pose"]
    current = force_cal_base_z(state.get("current_die_loc", cal_loc), cfg)
    detected_base = measured_xy_as_base_loc(detected, "Cal", cfg)
    pick_base = mm_loc(detected_base.getX(), detected_base.getY(), current.getZ(), current.getRotation())
    log("PostSquareRecenter", "picking_fresh_detected_center=True")
    pick_die_from_existing_lock(camera, nozzle, part, pick_base, cfg, "Cal", state,
                                lock, detected, None, "return")
    recenter_angle = float(cfg.get("post_square_recenter_angle_deg", 0.0))
    place_target = mm_loc(cal_loc.getX(), cal_loc.getY(), cal_loc.getZ(), recenter_angle)
    place_die_at(nozzle, part, place_target, cfg, "Cal", state)
    lock2, measured = acquire_pose_for_location(camera, nozzle, part, place_target, cfg, "Cal")
    pose2 = lock2["pose"]
    error_x = float(measured.getX() - cal_loc.getX())
    error_y = float(measured.getY() - cal_loc.getY())
    dx = -error_x
    dy = -error_y
    mag = math.sqrt(dx * dx + dy * dy)
    state["current_die_loc"] = base_loc_with_measured_orientation(measured, pose2, "Cal", cfg)
    state["current_die_loc_trusted"] = True
    state["last_trusted_die_loc"] = state["current_die_loc"]
    log("PostSquareRecenter", "measured_error=(%.5f, %.5f)" % (error_x, error_y))
    log("PostSquareRecenter", "post_square_static_delta=(%.5f, %.5f)" % (dx, dy))
    log("PostSquareRecenter", "correction_source=post_square_recenter")
    result.update({"ran": True,
                   "correction_source": "post_square_recenter",
                   "post_square_static_delta_x_mm": float(dx),
                   "post_square_static_delta_y_mm": float(dy),
                   "post_square_static_delta_mag_mm": float(mag),
                   "measured_error_x_mm": error_x,
                   "measured_error_y_mm": error_y,
                   "recenter_angle_deg": recenter_angle,
                   "detected_die_orientation_deg": float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0))),
                   "post_place_detected_die_orientation_deg": float(pose2.get("die_orientation_deg", pose2.get("orientation_raw_deg", 0.0))),
                   "detected_measurement_center": location_to_diag(measured)})
    return result


def verify_storage_return_orientation(camera, nozzle, part, storage_loc, cfg, rotation_plan):
    desired = float(rotation_plan.get("desired_die_orientation_deg", 0.0))
    out = {"enabled": bool_value(cfg.get("return_verify_orientation_after_place", True)),
           "desired_die_orientation_deg": desired,
           "passed": None,
           "warning": None}
    if not bool_value(cfg.get("return_verify_orientation_after_place", True)):
        log("StorageReturnVerify", "enabled=False")
        return out
    try:
        lock, detected = acquire_pose_for_location(camera, nozzle, part, storage_loc, cfg, "Storage")
        pose = lock["pose"]
        final_orientation = float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0)))
        error = normalize_angle(final_orientation - desired)
        passed = abs(error) <= float(cfg.get("return_orientation_tolerance_deg", 10.0))
        out["final_die_orientation_deg"] = final_orientation
        out["orientation_error_deg"] = error
        out["tolerance_deg"] = float(cfg.get("return_orientation_tolerance_deg", 10.0))
        out["passed"] = bool(passed)
        out["detected_measurement_center"] = location_to_diag(detected)
        log("StorageReturnVerify", "final_die_orientation_deg=%.5f" % final_orientation)
        log("StorageReturnVerify", "desired_die_orientation_deg=%.5f" % desired)
        log("StorageReturnVerify", "orientation_error_deg=%.5f" % error)
        log("StorageReturnVerify", "passed=%s" % str(bool(passed)))
        if not passed:
            out["warning"] = "return orientation outside tolerance"
        return out
    except Exception, e:
        out["passed"] = False
        out["warning"] = str(e)
        log("StorageReturnVerify", "passed=False warning=%s" % str(e))
        return out


def return_die_to_storage(camera, nozzle, part, storage_loc, cfg, state, computed, relock=None):
    current = state.get("current_die_loc", None)
    if current is None:
        current = loc_from_xyz(cfg["cal_work_location_xyz"])
    current = force_cal_base_z(current, cfg)
    if relock is not None:
        lock = relock.get("lock", None)
        detected = relock.get("detected", None)
    else:
        lock = None
        detected = None
    if lock is None or detected is None:
        lock, detected = acquire_pose_for_location(camera, nozzle, part, current, cfg, "Cal")
        orientation_reference = "fresh_return_relocated_die"
    else:
        orientation_reference = "post_bottom_square_relocated_die"
    pose = lock["pose"]
    state["current_die_loc"] = base_loc_with_measured_orientation(detected, pose, "Cal", cfg)
    state["current_die_loc_trusted"] = True
    state["last_trusted_die_loc"] = state["current_die_loc"]
    persistent_delta = static_candidate_delta_from_computed(computed)
    same_run_jog = zero_same_run_visual_pick_jog("fresh_visual_lock_no_static_jog")
    pick_correction_used = False
    bottom_square = computed.get("bottom_square", {}) or {}
    bottom_changed = bool_value(computed.get("bottom_square_changed_rotation_state",
                                             bottom_square.get("bottom_square_changed_rotation_state", False)))
    bottom_enabled = bool_value(bottom_square.get("enabled", cfg.get("run_bottom_camera_nozzle_square_after_fit", True)))
    bottom_passed = (not bottom_enabled) or (bool_value(bottom_square.get("ran", False)) and
                                            bool_value(bottom_square.get("verification_passed", False)))
    if bottom_enabled and bottom_changed and \
       (not bool_value(computed.get("square_rotation_bias_applied_to_static_vector", False))) and \
       (not bool_value(cfg.get("allow_confirm_with_stale_pre_square_correction", False))):
        log("StorageReturnPick", "stale_pre_square_correction_ignored_for_safe_return=True")
    calibrated_return_allowed = bool_value(computed.get("calibrated_return_allowed", bottom_passed))
    mode = "fresh_visual_lock_no_static_jog"
    detected_base = measured_xy_as_base_loc(detected, "Cal", cfg)
    pick_base = mm_loc(detected_base.getX(), detected_base.getY(), current.getZ(), current.getRotation())
    square_bias, square_bias_valid, raw_square_bias = effective_square_bias_from_result(computed, cfg, "ReturnOrientationPlan")
    detected_die_orientation = float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0)))
    desired_storage_orientation = return_storage_desired_orientation(detected_die_orientation, cfg)
    current_before_pick = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    pick_rotation = float(current_before_pick.getRotation())
    log("StorageReturnPick", "candidate_static_head_offset_used_as_same_run_pick_jog=False")
    log("StorageReturnPick", "fresh_visual_lock_current_active_geometry=True")
    log("StorageReturnPick", "using_fresh_visual_lock=True")
    log("StorageReturnPick", "persistent_head_offset_delta_not_used_as_same_run_jog=True")
    log("StorageReturnPick", "using_rejected_calibration_delta=False")
    log("StorageReturnPick", "persistent_head_offset_delta=(%.5f, %.5f, %.5f)" %
        (float(persistent_delta.get("dx", 0.0)), float(persistent_delta.get("dy", 0.0)),
         float(persistent_delta.get("mag", 0.0))))
    pick_info = {"mode": mode,
                 "correction_source": "fresh_visual_lock_no_static_jog",
                 "fresh_visual_lock": True,
                 "persistent_head_offset_delta_not_used_as_same_run_jog": True,
                 "using_rejected_calibration_delta": False,
                 "pre_square_dx": float(computed.get("static_head_offset_delta_x_mm_pre_square", 0.0)),
                 "pre_square_dy": float(computed.get("static_head_offset_delta_y_mm_pre_square", 0.0)),
                 "active_fit_dx": 0.0,
                 "active_fit_dy": 0.0}
    pick_die_from_existing_lock(camera, nozzle, part, pick_base, cfg, "Cal", state,
                                lock, detected, same_run_jog, "return", pick_info)
    current_after_pick = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    pick_rotation = float(current_after_pick.getRotation())
    held_plan = held_object_return_rotation_command(detected_die_orientation, pick_rotation,
                                                    float(current_after_pick.getRotation()), cfg)
    return_command_rotation = float(held_plan["command_r"])
    desired_storage_orientation = float(held_plan["desired"])
    log("ReturnOrientationPlan", "pick_rotation_deg=%.5f" % pick_rotation)
    log("ReturnOrientationPlan", "place_rotation_deg=%.5f" % return_command_rotation)
    log("ReturnOrientationPlan", "square_rotation_bias_valid=%s" % str(bool(square_bias_valid)))
    log("ReturnOrientationPlan", "effective_square_bias_deg=%.5f" % float(square_bias))
    log("ReturnOrientationPlan", "rejected_invalid_square_bias=%s" %
        str(bool((not square_bias_valid) and abs(raw_square_bias) > 0.000001)))
    log("ReturnOrientationPlan", "invalid_bottom_square_bias_suppressed=%s" %
        str(bool((not square_bias_valid) and abs(raw_square_bias) > 0.000001)))
    log("ReturnOrientationPlan", "bottom_square_not_used_for_return=%s" %
        str(bool(not square_bias_valid)))
    log_rotation_state(nozzle, detected_die_orientation, desired_storage_orientation,
                       square_bias_valid, square_bias)
    rotation_plan = {"current_nozzle_rotation_deg": float(nozzle.getLocation().convertToUnits(LengthUnit.Millimeters).getRotation()),
                     "current_die_orientation_deg": detected_die_orientation,
                     "detected_die_orientation_deg": detected_die_orientation,
                     "desired_die_storage_orientation_deg": desired_storage_orientation,
                     "desired_die_orientation_deg": desired_storage_orientation,
                     "square_rotation_bias_deg": square_bias,
                     "square_rotation_bias_valid": bool(square_bias_valid),
                     "raw_square_rotation_bias_deg": float(raw_square_bias),
                     "marker_before_pick_deg": float(held_plan["marker_before_pick"]),
                     "nozzle_rotation_at_pick_deg": float(held_plan["nozzle_rotation_at_pick"]),
                     "delta_marker_needed_deg": float(held_plan["delta_marker_needed"]),
                     "return_held_object_rotation_sign": float(held_plan["return_held_object_rotation_sign"]),
                     "return_nozzle_command_deg": return_command_rotation,
                     "orientation_reference": orientation_reference,
                     "pick_rotation_deg": pick_rotation,
                     "place_rotation_deg": return_command_rotation,
                     "rotation_math": held_plan["rotation_math"]}
    return_display_rotation = desired_storage_orientation
    rotate_after_pick = single_circle_rotation_location(
        nozzle, mm_loc(current_after_pick.getX(), current_after_pick.getY(), current_after_pick.getZ(), return_command_rotation),
        cfg, "ReturnRotateAfterPick")
    base.move_to_with_speed(nozzle, rotate_after_pick, cfg)
    place_dx = 0.0
    place_dy = 0.0
    place_correction_used = False
    if bool_value(cfg.get("return_to_storage_apply_offset_to_place", False)):
        place_correction = virtual_correction_for_rotation(computed, return_command_rotation, cfg, "return_place")
        place_dx = float(place_correction["dx"])
        place_dy = float(place_correction["dy"])
        place_correction_used = True
        log("StorageReturn", "place_virtual_correction_rotation_deg=%.5f" % float(return_command_rotation))
        log("StorageReturn", "place_virtual_correction=(%.5f, %.5f, %.5f)" %
            (float(place_correction["dx"]), float(place_correction["dy"]), float(place_correction["mag"])))
    storage_target = mm_loc(storage_loc.getX() + place_dx, storage_loc.getY() + place_dy,
                            storage_loc.getZ(), return_command_rotation)
    log("ReturnPlan", "persistent_head_offset_delta_not_used_as_same_run_jog=True")
    log("ReturnPlan", "persistent_head_offset_delta=(%.5f, %.5f, %.5f)" %
        (float(persistent_delta.get("dx", 0.0)), float(persistent_delta.get("dy", 0.0)), float(persistent_delta.get("mag", 0.0))))
    log("ReturnPlan", "square_rotation_bias_deg=%.5f" % square_bias)
    log("ReturnPlan", "square_rotation_bias_valid=%s" % str(bool(square_bias_valid)))
    log("ReturnPlan", "desired_storage_orientation_deg=%.4f" % desired_storage_orientation)
    log("ReturnPlan", "commanded_storage_place_rotation_deg=%.5f" % return_command_rotation)
    log("ReturnPlan", "storage_place_target=(%.5f, %.5f, %.5f, %.5f)" %
        (storage_target.getX(), storage_target.getY(), storage_target.getZ(), storage_target.getRotation()))
    log("ReturnPlan", "using_squared_calibrated_nozzle=%s" % str(bool(calibrated_return_allowed)))
    log("StorageReturnPlace", "desired_orientation_deg=%.5f" % desired_storage_orientation)
    log("StorageReturnPlace", "commanded_rotation_deg=%.5f" % return_command_rotation)
    log("StorageReturnPlace", "calibration_failed_but_safe_return=%s" %
        str(not bool_value(computed.get("persistent_apply_allowed", False))))
    place_die_at(nozzle, part, storage_target, cfg, "Storage", state)
    verify = verify_storage_return_orientation(camera, nozzle, part, storage_loc, cfg, rotation_plan)
    orientation_verified = bool_value(verify.get("passed", False))
    final_die_orientation_for_log = float(verify.get("final_die_orientation_deg", desired_storage_orientation))
    log_rotation_state(nozzle, final_die_orientation_for_log, desired_storage_orientation,
                       square_bias_valid, square_bias)
    log("StorageReturn", "orientation_circle_used_for_center=False")
    log("StorageReturn", "current_orientation_deg=%.5f" % float(rotation_plan["current_die_orientation_deg"]))
    log("StorageReturn", "return_display_rotation_deg=%.0f" % return_display_rotation)
    log("StorageReturn", "return_command_rotation_deg=%.0f" % return_command_rotation)
    log("StorageReturn", "return_rotation_deg=%.0f" % return_display_rotation)
    log("StorageReturn", "continuous_rotation=%s" %
        str(str(cfg.get("single_circle_rotation_mode", "continuous")).lower() == "continuous"))
    log("StorageReturn", "configured_storage_pocket_unchanged=True")
    log("StorageReturn", "pick_correction_used=%s" % str(bool(pick_correction_used)))
    log("StorageReturn", "place_correction_used=%s" % str(bool(place_correction_used)))
    log("StorageReturn", "orientation_marker_used=True")
    log("StorageReturn", "orientation_plan_used=True")
    log("StorageReturn", "orientation_verified=%s" % str(bool(orientation_verified)))
    log("StorageReturn", "placed=True")
    return {
        "status": "placed" if orientation_verified else "placed_orientation_not_verified",
        "configured_storage_pocket_unchanged": True,
        "pick_correction_used": bool(pick_correction_used),
        "fresh_visual_lock_no_static_jog": True,
        "persistent_head_offset_delta_not_used_as_same_run_jog": True,
        "calibrated_return_allowed": bool(calibrated_return_allowed),
        "post_square_pick_delta_mode": mode,
        "bottom_square_changed_rotation_state": bool(bottom_changed),
        "place_correction_used": bool(place_correction_used),
        "return_rotation_deg": float(return_display_rotation),
        "return_display_rotation_deg": float(return_display_rotation),
        "return_command_rotation_deg": float(return_command_rotation),
        "current_orientation_deg": float(rotation_plan["current_die_orientation_deg"]),
        "rotation_plan": rotation_plan,
        "orientation_plan_used": True,
        "orientation_verified": bool(orientation_verified),
        "post_return_orientation_check": verify,
        "orientation_circle_used_for_center": False,
        "orientation_marker_used": True,
        "target": location_to_diag(storage_target)
    }


def apply_availability_for_result(result, cfg):
    computed = result.get("computed", {}) or {}
    safe = bool_value(computed.get("safe_to_apply", False))
    if not safe:
        return False, computed.get("apply_block_reason", "result is not safe_to_apply")
    if not bool_value(cfg.get("allow_machine_writes", False)):
        return False, "allow_machine_writes is false"
    verification = computed.get("physical_ab_verification", {}) or result.get("verification", {}) or {}
    if not bool_value(verification.get("passed", False)) and not bool_value(cfg.get("allow_apply_without_verification", False)):
        return False, "physical verification did not pass and allow_apply_without_verification is false"
    if float(computed.get("static_head_offset_delta_mag_mm",
                          computed.get("selected_head_delta_mag_mm", 999999.0))) > float(cfg.get("max_apply_mm", 1.5)):
        return False, "static head offset delta exceeds max_apply_mm"
    return True, None


def get_last_result_apply_preview(progress_callback=None, cancel_token=None):
    cfg = load_config()
    result = load_last_result()
    computed = result.get("computed", {}) or {}
    can_apply, reason = apply_availability_for_result(result, cfg)
    return {"can_apply": bool(can_apply),
            "reason": reason,
            "model": computed.get("model", None),
            "measured_error_x_mm": computed.get("bias_x_mm", None),
            "measured_error_y_mm": computed.get("bias_y_mm", None),
            "delta_x_mm": computed.get("static_head_offset_delta_x_mm", computed.get("selected_head_delta_x_mm", None)),
            "delta_y_mm": computed.get("static_head_offset_delta_y_mm", computed.get("selected_head_delta_y_mm", None)),
            "current_head_offsets_before": computed.get("current_head_offsets_before", None),
            "candidate_head_offsets_after": computed.get("candidate_head_offsets_after", None),
            "verification_is_physical": bool_value((computed.get("physical_ab_verification", {}) or {}).get("verification_is_physical", False)),
            "verification_passed": bool_value((computed.get("physical_ab_verification", {}) or {}).get("passed", False)),
            "safe_to_apply": bool_value(computed.get("safe_to_apply", False)),
            "max_apply_mm": cfg.get("max_apply_mm", None)}


def apply_result(result=None, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    old_progress = _progress_callback
    old_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token
    try:
        cfg = load_config()
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = base._prepare_runtime(cfg)
        if result is None or not isinstance(result, dict):
            result = load_last_result()
        if result.get("error"):
            raise Exception("Refusing to apply failed result: %s" % result.get("error"))
        computed = result.get("computed", {}) or {}
        can_apply, reason = apply_availability_for_result(result, cfg)
        if not can_apply:
            raise Exception(reason)
        log("Apply", "Applying static nozzle head-offset delta only.")
        log("Apply", "Runout diagnostics and square rotation bias are not written to machine configuration.")
        backup = backup_machine_xml_if_requested(cfg)
        old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
        dx = float(computed.get("static_head_offset_delta_x_mm",
                                computed.get("nozzle_head_offset_delta_x_mm", 0.0)))
        dy = float(computed.get("static_head_offset_delta_y_mm",
                                computed.get("nozzle_head_offset_delta_y_mm", 0.0)))
        new = mm_loc(float(old.getX()) + dx, float(old.getY()) + dy,
                     float(old.getZ()), float(old.getRotation()))
        nozzle.setHeadOffsets(new)
        applied = {"target": "nozzle_head_offsets",
                   "old_x_mm": float(old.getX()), "old_y_mm": float(old.getY()),
                   "new_x_mm": float(new.getX()), "new_y_mm": float(new.getY()),
                   "delta_x_mm": dx,
                   "delta_y_mm": dy,
                   "machine_xml_backup": backup,
                   "runout_written": False,
                   "square_rotation_bias_written": False,
                   "timestamp": time.time()}
        result["applied"] = applied
        result["applied_head_offsets"] = {"x": float(new.getX()), "y": float(new.getY()),
                                           "z": float(new.getZ()), "rotation": float(new.getRotation())}
        result["machine_config_modified"] = True
        result["applied_timestamp"] = applied["timestamp"]
        save_result(result)
        log("Apply", "Applied nozzle head offsets old=(%.5f, %.5f) new=(%.5f, %.5f)" %
            (float(old.getX()), float(old.getY()), float(new.getX()), float(new.getY())))
        return result
    finally:
        _progress_callback = old_progress
        _cancel_token = old_cancel


def correction_delta_from_result(result, cfg):
    computed = result.get("computed", {}) or {}
    dx = float(computed.get("static_head_offset_delta_x_mm", computed.get("nozzle_head_offset_delta_x_mm", 0.0)))
    dy = float(computed.get("static_head_offset_delta_y_mm", computed.get("nozzle_head_offset_delta_y_mm", 0.0)))
    return {"dx": dx,
            "dy": dy,
            "model": computed.get("model", "openpnp_pairwise_180_offset"),
            "available": True,
            "diagnostic_preview": False,
            "apply_forbidden": False,
            "reason": "single-circle static nozzle head offset delta"}


def test_vision_lock(location_key, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    old_progress = _progress_callback
    old_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token
    try:
        cfg = load_config()
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = base._prepare_runtime(cfg)
        key = str(location_key).lower()
        if key == "storage":
            base_loc = storage_loc
            tag = "Storage"
        elif key in ["cal", "calibration"]:
            base_loc = cal_loc
            tag = "Cal"
        else:
            raise Exception("location_key must be 'storage' or 'cal'.")
        lock, detected = acquire_pose_for_location(camera, nozzle, part, base_loc, cfg, tag)
        pose = lock["pose"]
        return {
            "type": "vision_lock",
            "script_model": "single_circle",
            "script_file": "pick_cameraAlign_SingleCircleTipCalibration.py",
            "location_key": key,
            "circle_count": int(pose.get("circle_count", 0)),
            "center_x_mm": float(detected.getX()),
            "center_y_mm": float(detected.getY()),
            "rotation_deg": float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0))),
            "orientation_raw_marker_deg": float(pose.get("orientation_raw_marker_deg", pose.get("orientation_raw_deg", 0.0))),
            "die_orientation_deg": float(pose.get("die_orientation_deg", pose.get("orientation_raw_deg", 0.0))),
            "search_dx_mm": float(pose.get("search_dx_mm", 0.0)),
            "search_dy_mm": float(pose.get("search_dy_mm", 0.0)),
            "overlay": lock.get("overlay", None),
            "timestamp": time.time()
        }
    finally:
        _progress_callback = old_progress
        _cancel_token = old_cancel


def move_tool_over_location(location_key, tool_kind, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    old_progress = _progress_callback
    old_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token
    try:
        cfg = load_config()
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = base._prepare_runtime(cfg)
        key = str(location_key).lower()
        if key == "storage":
            base_loc = storage_loc
        elif key in ["cal", "calibration"]:
            base_loc = cal_loc
        else:
            raise Exception("location_key must be 'storage' or 'cal'.")
        tool = str(tool_kind).lower()
        if tool == "camera":
            target = camera_measurement_loc(camera, base_loc.getX(), base_loc.getY(), float(cfg["safe_travel_z"]))
            base.move_tool_safe(camera, target, "Move")
        elif tool == "nozzle":
            target = single_circle_rotation_location(
                nozzle, mm_loc(base_loc.getX(), base_loc.getY(), float(cfg["safe_travel_z"]), base_loc.getRotation()),
                cfg, "Move")
            single_circle_move_nozzle_split(nozzle, target, cfg, "Move", "Move")
        else:
            raise Exception("tool_kind must be 'camera' or 'nozzle'.")
        return {"type": "move_tool", "location_key": key, "tool_kind": tool,
                "x_mm": float(target.getX()), "y_mm": float(target.getY()),
                "z_mm": float(target.getZ()), "rotation_deg": float(target.getRotation())}
    finally:
        _progress_callback = old_progress
        _cancel_token = old_cancel


def confirm_shift(result=None, show_shift=False, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    old_progress = _progress_callback
    old_cancel = _cancel_token
    temp_head_offsets = None
    nozzle = None
    _progress_callback = progress_callback
    _cancel_token = cancel_token
    try:
        cfg = load_config()
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = base._prepare_runtime(cfg)
        if result is None or not isinstance(result, dict):
            if result is not None and not isinstance(result, dict):
                show_shift = bool_value(result)
            result = load_last_result()
        computed = result.get("computed", {}) or {}
        preview_mode = "candidate_head_offset_only" if bool_value(show_shift) else "current_active_geometry"
        log("Confirm", "preview_mode=%s" % preview_mode)
        lock, detected = acquire_pose_for_location(camera, nozzle, part, storage_loc, cfg, "Storage")
        pose = lock["pose"]
        nozzle_target = lock["nozzle_target_location"]
        measurement_center = lock["measurement_location"]
        current_nozzle = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        confirm_policy = pick_rotation_policy_for_purpose(cfg, "confirm")
        square_bias_for_confirm, square_bias_valid_for_confirm, raw_square_bias_for_confirm = \
            effective_square_bias_from_result(computed, cfg, "RotationPlan")
        desired_confirm_orientation = float(cfg.get("desired_die_storage_orientation_deg", 0.0))
        if bool_value(computed.get("candidate_delta_available", False)):
            target_r = corrected_nozzle_rotation_for_die_orientation(
                desired_confirm_orientation, square_bias_for_confirm, cfg)
            rotation_source = "squared_desired_storage_orientation"
            if not square_bias_valid_for_confirm:
                rotation_source = "squared_desired_storage_orientation_invalid_square_bias_suppressed"
            confirm_policy = "squared_desired_orientation"
        elif confirm_policy == "match_detected_marker":
            target_r = continuous_equivalent_near_current(float(pose["die_orientation_deg"]), current_nozzle.getRotation())
            rotation_source = "orientation_circle"
        elif confirm_policy == "preserve_current_nozzle_rotation":
            target_r = float(current_nozzle.getRotation())
            rotation_source = "current_nozzle_rotation"
        else:
            raise Exception("Invalid confirm_rotation_policy: %s" % confirm_policy)
        use_nozzle_target_frame = bool_value(cfg.get("confirm_use_nozzle_target_frame", True))
        anchor_center = nozzle_target if use_nozzle_target_frame else measurement_center
        anchor_frame = "nozzle_target_tool_aware" if use_nozzle_target_frame else "camera_only_measurement"
        anchor_loc = mm_loc(anchor_center.getX(), anchor_center.getY(), storage_loc.getZ(), target_r)
        target_surface_z = confirm_target_z(anchor_loc, cfg)
        descend_to_surface = bool_value(cfg.get("single_circle_confirm_descend_to_surface", True))
        target_z = target_surface_z if descend_to_surface else float(cfg["safe_travel_z"])
        no_shift = mm_loc(anchor_center.getX(), anchor_center.getY(), target_z, target_r)
        bottom_changed = bool_value(result.get("bottom_square_changed_rotation_state",
                                               computed.get("bottom_square_changed_rotation_state", False)))
        correction_valid_after_square = bool_value(result.get("top_camera_correction_valid_after_bottom_square",
                                                              computed.get("top_camera_correction_valid_after_bottom_square", True)))
        correction_source = str(computed.get("active_fit_source", "large_circle_xy_fit_before_tip_squaring"))
        show_shift_blocked_or_diagnostic_only = False
        active_allowed = bool_value(computed.get("active_correction_allowed", True))
        diagnostic_allowed = bool_value(cfg.get("confirm_allow_diagnostic_preview", True))
        candidate_delta_available = bool_value(computed.get("candidate_delta_available", False))
        candidate_source = str(computed.get("candidate_source",
                                            computed.get("single_circle_fit_model_type", correction_source)))
        using_candidate_delta = bool_value(show_shift) and candidate_delta_available
        virtual_delta = static_candidate_delta_from_computed(computed)
        candidate_delta_source = str(virtual_delta.get("candidate_delta_source", "none"))
        log("ConfirmPreview", "candidate_delta_source=%s" % candidate_delta_source)
        log("ConfirmPreview", "candidate_delta_available=%s" % str(bool(candidate_delta_available)))
        log("ConfirmPreview", "virtual_delta=(%.5f, %.5f, %.5f)" %
            (float(virtual_delta.get("dx", 0.0)), float(virtual_delta.get("dy", 0.0)),
             float(virtual_delta.get("mag", 0.0))))
        if candidate_delta_available and float(virtual_delta.get("mag", 0.0)) <= 0.000001:
            raise Exception("candidate_delta_available=True but static_candidate_delta_from_computed returned zero; schema/read bug.")
        if not using_candidate_delta:
            virtual_delta = {"dx": 0.0, "dy": 0.0, "mag": 0.0,
                             "static_dx": 0.0, "static_dy": 0.0,
                             "angle_dx": 0.0, "angle_dy": 0.0,
                             "offset": None,
                             "candidate_delta_source": candidate_delta_source}
        else:
            virtual_delta["offset"] = None
        if "model_center_x_mm" in computed:
            model_offset = virtual_delta.get("offset", None)
            if model_offset is None:
                model_offset = model_offset_for_rotation(computed, target_r)
                virtual_delta["offset"] = model_offset
            log("Confirm", "model_mode=%s" %
                str(computed.get("single_circle_model_mode", computed.get("model", "openpnp_model_camera_offset"))))
            log("Confirm", "model_offset_at_rotation=(%.5f, %.5f, %.5f)" %
                (float(model_offset["x"]), float(model_offset["y"]), float(model_offset["mag"])))
            log("Confirm", "command_delta=(%.5f, %.5f, %.5f)" %
                (float(virtual_delta["dx"]), float(virtual_delta["dy"]), float(virtual_delta["mag"])))
            if not bool_value(computed.get("active_correction_allowed", True)):
                log("Confirm", "active_correction_allowed=False diagnostic_preview_only=True")
        shift = mm_loc(anchor_center.getX() + virtual_delta["dx"], anchor_center.getY() + virtual_delta["dy"],
                       target_z, target_r)
        correction_used = bool(using_candidate_delta) and not bool(show_shift_blocked_or_diagnostic_only)
        selected = shift if correction_used else no_shift
        mag = float(virtual_delta["mag"])
        if using_candidate_delta and mag < float(cfg.get("single_circle_min_visible_preview_shift_mm", 0.02)):
            log("ConfirmPreview", "WARNING candidate available but preview delta is below visible threshold")
            log("Confirm", "WARNING shift magnitude %.5f mm < single_circle_min_visible_preview_shift_mm %.5f" %
                (mag, float(cfg.get("single_circle_min_visible_preview_shift_mm", 0.02))))
        log("ConfirmFrame", "measurement_center=(%.5f,%.5f)" %
            (float(measurement_center.getX()), float(measurement_center.getY())))
        log("ConfirmFrame", "nozzle_target_center=(%.5f,%.5f)" %
            (float(nozzle_target.getX()), float(nozzle_target.getY())))
        log("ConfirmFrame", "anchor_frame=%s" % anchor_frame)
        log("Confirm", "anchor_source=%s" %
            ("fresh_detected_nozzle_target" if use_nozzle_target_frame else "fresh_detected_measurement_center"))
        log("Confirm", "detected_die_orientation_deg=%.4f" % float(pose.get("die_orientation_deg", 0.0)))
        log("Confirm", "target_nozzle_rotation_deg=%.4f" % float(target_r))
        log("Confirm", "rotation_source=%s" % rotation_source)
        log("Confirm", "rotation_policy=%s" % confirm_policy)
        log("Confirm", "marker_orientation_detected_but_not_used_for_rotation=%s" %
            str(confirm_policy == "preserve_current_nozzle_rotation"))
        log("Confirm", "correction_source=%s" % correction_source)
        log("Confirm", "show_shift_blocked_or_diagnostic_only=%s" % str(bool(show_shift_blocked_or_diagnostic_only)))
        log("Confirm", "current_geometry=current_active_openpnp_offsets")
        log("Confirm", "candidate_geometry=temporary_static_nozzle_head_offsets")
        log("Confirm", "virtual_static_delta=(%.5f, %.5f)" %
            (float(virtual_delta["static_dx"]), float(virtual_delta["static_dy"])))
        log("Confirm", "virtual_angle_delta_at_rotation=(%.5f, %.5f)" %
            (float(virtual_delta["angle_dx"]), float(virtual_delta["angle_dy"])))
        log("Confirm", "preview_extra_jog_delta=(%.5f, %.5f, %.5f)" %
            (float(virtual_delta["dx"]), float(virtual_delta["dy"]), mag))
        log("Confirm", "no_shift_command=(%.5f, %.5f, %.5f, %.5f)" %
            (no_shift.getX(), no_shift.getY(), no_shift.getZ(), no_shift.getRotation()))
        log("Confirm", "shift_command=(%.5f, %.5f, %.5f, %.5f)" %
            (shift.getX(), shift.getY(), shift.getZ(), shift.getRotation()))
        log("Confirm", "shift_minus_no_shift=(%.5f, %.5f, %.5f)" %
            (float(virtual_delta["dx"]), float(virtual_delta["dy"]), mag))
        log("ConfirmPreview", "show_shift=%s" % str(bool_value(show_shift)))
        log("ConfirmPreview", "candidate_delta_available=%s" % str(bool(candidate_delta_available)))
        log("ConfirmPreview", "candidate_source=%s" % candidate_source)
        log("ConfirmPreview", "candidate_delta_source=%s" % candidate_delta_source)
        log("ConfirmPreview", "active_correction_allowed=%s" % str(bool(active_allowed)))
        log("ConfirmPreview", "diagnostic_preview_allowed=%s" % str(bool(diagnostic_allowed)))
        log("ConfirmPreview", "using_candidate_delta_even_if_apply_blocked=%s" %
            str(bool(using_candidate_delta and not active_allowed)))
        log("ConfirmPreview", "no_shift_xy=(%.5f, %.5f)" %
            (float(no_shift.getX()), float(no_shift.getY())))
        log("ConfirmPreview", "shift_xy=(%.5f, %.5f)" %
            (float(shift.getX()), float(shift.getY())))
        log("ConfirmPreview", "xy_delta=(%.5f, %.5f, %.5f)" %
            (float(virtual_delta["dx"]), float(virtual_delta["dy"]), mag))
        log("GUI", "[ConfirmCompare] no_shift=(%.5f, %.5f, %.5f, %.5f)" %
            (no_shift.getX(), no_shift.getY(), no_shift.getZ(), no_shift.getRotation()))
        log("GUI", "[ConfirmCompare] shift=(%.5f, %.5f, %.5f, %.5f)" %
            (shift.getX(), shift.getY(), shift.getZ(), shift.getRotation()))
        log("GUI", "[ConfirmCompare] delta=(%.5f, %.5f, %.5f)" %
            (float(virtual_delta["dx"]), float(virtual_delta["dy"]), mag))
        log("ConfirmPreview", "no_shift_r=%.5f" % float(no_shift.getRotation()))
        log("ConfirmPreview", "shift_r=%.5f" % float(shift.getRotation()))
        log("ConfirmPreview", "r_source=%s" % str(rotation_source))
        log("ConfirmPreview", "square_bias_valid=%s" % str(bool(square_bias_valid_for_confirm)))
        log("ConfirmPreview", "invalid_square_bias_suppressed=%s" %
            str(bool((not square_bias_valid_for_confirm) and abs(raw_square_bias_for_confirm) > 0.000001)))
        if not active_allowed:
            log("ConfirmPreview", "diagnostic_only_not_safe_to_apply=True")
            log("ConfirmPreview", "physical_pick_should_not_use_candidate_delta=True")
        log("ConfirmPreview", "no_shift_command=(%.5f, %.5f, %.5f, %.5f)" %
            (no_shift.getX(), no_shift.getY(), no_shift.getZ(), no_shift.getRotation()))
        log("ConfirmPreview", "shift_command=(%.5f, %.5f, %.5f, %.5f)" %
            (shift.getX(), shift.getY(), shift.getZ(), shift.getRotation()))
        log("ConfirmPreview", "shift_minus_no_shift=(%.5f, %.5f, %.5f)" %
            (float(virtual_delta["dx"]), float(virtual_delta["dy"]), mag))
        log("ConfirmPreview", "preview_delta_norm_mm=%.5f" % mag)
        log("Confirm", "correction_used=%s" % str(bool(correction_used)))
        log("Confirm", "descend_to_surface=%s" % str(bool(descend_to_surface)))
        log("Confirm", "target_surface_z=%.5f" % float(target_surface_z))
        log("Confirm", "final_target=(%.5f, %.5f, %.5f, %.5f)" %
            (selected.getX(), selected.getY(), selected.getZ(), selected.getRotation()))
        move_nozzle_confirm_preview(nozzle, selected, cfg)
        log("Confirm", "diagnostic_preview=%s" % str(bool(using_candidate_delta and not active_allowed)))
        log("Confirm", "apply_forbidden=%s" % str(bool(not active_allowed)))
        confirm = {"type": "confirm_shift",
                   "script_model": "single_circle",
                   "script_file": "pick_cameraAlign_SingleCircleTipCalibration.py",
                   "mode": preview_mode,
                   "preview_mode": preview_mode,
                   "show_shift": bool(correction_used),
                   "show_shift_requested": bool_value(show_shift),
                   "result_loaded": True,
                   "result_model": computed.get("model", "single_circle_openpnp_style"),
                   "model_mode": computed.get("single_circle_model_mode", computed.get("model", None)),
                   "active_fit_source": correction_source,
                   "old_model_description": "current_active_openpnp_offsets",
                   "new_model_description": "temporary_static_nozzle_head_offsets",
                   "virtual_correction_available": bool(candidate_delta_available),
                   "correction_available": True,
                   "anchor_source": "fresh_detected_nozzle_target" if use_nozzle_target_frame else "fresh_detected_measurement_center",
                   "anchor_frame": anchor_frame,
                   "detected_measurement_center": location_to_diag(detected),
                   "detected_nozzle_target_center": location_to_diag(nozzle_target),
                   "detected_die_orientation_deg": float(pose.get("die_orientation_deg", 0.0)),
                   "target_nozzle_rotation_deg": float(target_r),
                   "rotation_source": rotation_source,
                   "rotation_policy": confirm_policy,
                   "square_rotation_bias_valid": bool(square_bias_valid_for_confirm),
                   "effective_square_bias_deg": float(square_bias_for_confirm),
                   "invalid_square_bias_suppressed": bool((not square_bias_valid_for_confirm) and abs(raw_square_bias_for_confirm) > 0.000001),
                   "no_shift_command": location_to_diag(no_shift),
                   "shift_command": location_to_diag(shift),
                   "shift_minus_no_shift": {"dx": virtual_delta["dx"], "dy": virtual_delta["dy"], "mag": mag},
                   "model_offset_at_rotation": virtual_delta.get("offset", None),
                   "command_delta": {"dx": virtual_delta["dx"], "dy": virtual_delta["dy"], "mag": mag},
                   "active_correction_allowed": bool_value(computed.get("active_correction_allowed", True)),
                   "candidate_delta_available": bool(candidate_delta_available),
                   "candidate_source": candidate_source,
                   "diagnostic_preview_allowed": bool(diagnostic_allowed),
                   "using_candidate_delta_even_if_apply_blocked": bool(using_candidate_delta and not active_allowed),
                   "correction_used": bool(correction_used),
                   "correction_source": correction_source,
                   "show_shift_blocked_or_diagnostic_only": bool(show_shift_blocked_or_diagnostic_only),
                   "bottom_square_changed_rotation_state": bool(bottom_changed),
                   "top_camera_correction_valid_after_bottom_square": bool(correction_valid_after_square),
                   "correction_model": preview_mode,
                   "correction_reason": "temporary nozzle head offset substitution preview",
                   "target": location_to_diag(selected),
                   "target_x_mm": float(selected.getX()),
                   "target_y_mm": float(selected.getY()),
                   "target_z_mm": float(selected.getZ()),
                   "target_rotation_deg": float(selected.getRotation()),
                   "target_surface_z_mm": float(target_surface_z),
                   "descend_to_surface": bool(descend_to_surface),
                   "virtual_static_delta_x_mm": float(virtual_delta["static_dx"]),
                   "virtual_static_delta_y_mm": float(virtual_delta["static_dy"]),
                   "virtual_angle_delta_x_mm": float(virtual_delta["angle_dx"]),
                   "virtual_angle_delta_y_mm": float(virtual_delta["angle_dy"]),
                   "virtual_total_delta_x_mm": float(virtual_delta["dx"]),
                   "virtual_total_delta_y_mm": float(virtual_delta["dy"]),
                   "virtual_total_delta_mag_mm": mag,
                   "correction_x_mm": float(virtual_delta["dx"]),
                   "correction_y_mm": float(virtual_delta["dy"]),
                   "circle_count": 2,
                   "diagnostic_preview": bool(using_candidate_delta and not active_allowed),
                   "apply_forbidden": bool(not active_allowed),
                   "overlay": lock.get("overlay", None),
                   "timestamp": time.time()}
        save_last_confirm_result(confirm)
        return confirm
    finally:
        if temp_head_offsets is not None:
            try:
                restore_temporary_head_offsets(nozzle, temp_head_offsets, "confirm_candidate_preview")
            except:
                pass
        _progress_callback = old_progress
        _cancel_token = old_cancel


def log_result_summary(result):
    computed = result.get("computed", {}) or {}
    cfg = result.get("config_snapshot", {}) or DEFAULT_CONFIG
    log("Result", "computed_model=%s safe_to_apply=%s delta=(%.5f, %.5f, %.5f)" %
        (computed.get("model", None),
         str(bool_value(computed.get("safe_to_apply", False))),
         float(computed.get("selected_head_delta_x_mm", 0.0)),
         float(computed.get("selected_head_delta_y_mm", 0.0)),
         float(computed.get("selected_head_delta_mag_mm", 0.0))))
    log("Result", "active_fit_source=%s" % str(computed.get("active_fit_source", None)))
    log("Result", "apply_available=%s" % str(bool_value(computed.get("apply_available", False))))
    log("Result", "diagnostic_preview_available=%s" % str(bool_value(computed.get("diagnostic_preview_available", False))))
    if "model_center_x_mm" in computed:
        offset0 = model_offset_for_rotation(computed, 0.0)
        delta0 = command_delta_for_rotation(computed, 0.0, cfg)
        offset90 = model_offset_for_rotation(computed, 90.0)
        delta90 = command_delta_for_rotation(computed, 90.0, cfg)
    else:
        offset0 = None
        offset90 = None
        delta0 = virtual_correction_for_rotation(computed, 0.0, cfg, "result_summary")
        delta90 = virtual_correction_for_rotation(computed, 90.0, cfg, "result_summary")
    log("Result", "selected_head_delta=(%.5f, %.5f, %.5f)" %
        (float(computed.get("selected_head_delta_x_mm", 0.0)),
         float(computed.get("selected_head_delta_y_mm", 0.0)),
         float(computed.get("selected_head_delta_mag_mm", 0.0))))
    log("Result", "residual_angle_vector=(%.5f, %.5f)" %
        (float(computed.get("residual_angle_vector_x_mm", 0.0)),
         float(computed.get("residual_angle_vector_y_mm", 0.0))))
    log("Result", "virtual_runout_preview_enabled=%s" %
        str(bool_value(computed.get("virtual_runout_preview_enabled", False))))
    if offset0 is not None:
        log("Result", "model_offset_at_0=(%.5f, %.5f, %.5f)" %
            (float(offset0["x"]), float(offset0["y"]), float(offset0["mag"])))
        log("Result", "command_delta_at_0=(%.5f, %.5f, %.5f)" %
            (float(delta0["dx"]), float(delta0["dy"]), float(delta0["mag"])))
        log("Result", "model_offset_at_90=(%.5f, %.5f, %.5f)" %
            (float(offset90["x"]), float(offset90["y"]), float(offset90["mag"])))
        log("Result", "command_delta_at_90=(%.5f, %.5f, %.5f)" %
            (float(delta90["dx"]), float(delta90["dy"]), float(delta90["mag"])))
    log("Result", "show_shift_delta_at_0=(%.5f, %.5f, %.5f)" %
        (float(delta0["dx"]), float(delta0["dy"]), float(delta0["mag"])))
    log("Result", "show_shift_delta_at_90=(%.5f, %.5f, %.5f)" %
        (float(delta90["dx"]), float(delta90["dy"]), float(delta90["mag"])))
    log("Result", "verification_old_new_available=%s" %
        str(len(computed.get("verification_old_new", [])) > 0))
    bottom_square = computed.get("bottom_square", {}) or {}
    log("Result", "bottom_square_ran=%s" % str(bool_value(bottom_square.get("ran", False))))
    selected = bottom_square.get("selected_correction_deg", None)
    if selected is None:
        log("Result", "bottom_square_selected_correction_deg=None")
    else:
        log("Result", "bottom_square_selected_correction_deg=%.5f" % float(selected))
    log("Result", "bottom_square_verification_passed=%s" %
        str(bottom_square.get("verification_passed", None)))
    log("Result", "bottom_square_changed_rotation_state=%s" %
        str(bool_value(computed.get("bottom_square_changed_rotation_state",
                                    bottom_square.get("bottom_square_changed_rotation_state", False)))))
    log("Result", "top_camera_correction_valid_after_bottom_square=%s" %
        str(bool_value(computed.get("top_camera_correction_valid_after_bottom_square", True))))
    log("Result", "requires_post_square_remeasure=%s" %
        str(bool_value(computed.get("requires_post_square_remeasure", False))))


def run(apply_offsets=False, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    old_progress = _progress_callback
    old_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token
    cfg = None
    nozzle = None
    result = None
    state = {"die_on_nozzle": False, "die_at_work": False,
             "current_die_loc": None, "current_die_loc_trusted": False,
             "last_trusted_die_loc": None}
    try:
        if bool_value(apply_offsets):
            raise Exception("run(apply_offsets=True) is not supported. Run calibration, then call apply_result() explicitly.")
        cfg = load_config()
        log("Mode", "selected_calibration_object_type=single_circle")
        log("Mode", "executing pick_cameraAlign_SingleCircleTipCalibration.py")
        log_compensation_stack()
        log_model_identity(cfg)
        log("SingleCircle", "result_stage=%s" % str(cfg.get("single_circle_result_stage_name", "results")))
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = base._prepare_runtime(cfg)
        try:
            start_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            state["starting_nozzle_rotation_deg"] = float(start_loc.getRotation())
        except:
            state["starting_nozzle_rotation_deg"] = 0.0
        storage_lock, detected_storage, cal_lock = transfer_die_from_storage_to_cal(
            camera, nozzle, part, storage_loc, cal_loc, cfg, state)
        bottom_square = {"enabled": bool_value(cfg.get("run_bottom_camera_nozzle_square_after_fit", True)),
                         "ran": False,
                         "selected_correction_deg": None,
                         "square_rotation_bias_deg": 0.0,
                         "square_rotation_bias_valid": False,
                         "bottom_square_rotation_valid": False,
                         "measurement_valid": False,
                         "safe_return_allowed": True,
                         "square_rotation_apply_mode": str(cfg.get("bottom_square_apply_mode", "script_local_rotation_bias")),
                         "applied_motion_correction": False,
                         "bottom_square_apply_motion_correction_moved_rotation": False,
                         "set_global_rotation_offset": False,
                         "global_offsets_called": False,
                         "visual_rotation_zeroed": False,
                         "bottom_square_changed_rotation_state": False,
                         "top_camera_correction_valid_after_bottom_square": True,
                         "requires_post_square_remeasure": False,
                         "pre_apply_records": [],
                         "pre_apply_stats": {},
                         "post_apply_records": [],
                         "post_apply_stats": {},
                         "verification_passed": None,
                         "warnings": []}
        post_bottom_square_lock = None
        post_bottom_square_detected = None
        log("Order", "xy_large_circle_calibration_before_tip_squaring=True")
        fit_type = str(cfg.get("single_circle_fit_model_type", "openpnp_pairwise_180_offset")).strip().lower()
        if fit_type != "openpnp_pairwise_180_offset":
            raise Exception("Unsupported fit model. This script only supports openpnp_pairwise_180_offset.")
        computed, samples, rejected_samples = run_openpnp_pairwise_180_nozzle_offset_calibration(
            camera, nozzle, part, cal_loc, cfg, state)
        old_hx, old_hy, old_hz, old_hr = nozzle_head_offsets_mm(nozzle)
        computed["current_head_offsets_before"] = {"x": old_hx, "y": old_hy, "z": old_hz, "rotation": old_hr}
        computed["candidate_head_offsets_after"] = {
            "x": old_hx + float(computed.get("static_head_offset_delta_x_mm", 0.0)),
            "y": old_hy + float(computed.get("static_head_offset_delta_y_mm", 0.0)),
            "z": old_hz,
            "rotation": old_hr}
        computed["active_fit_source"] = computed.get("active_fit_source", "openpnp_pairwise_180_offset")
        computed["pre_square_fit_exists"] = False
        computed["post_square_recenter"] = {"enabled": False,
                                            "ran": False,
                                            "disabled_reason": "destructive post-square recenter disabled by default"}
        computed["bottom_square"] = bottom_square
        changed_rotation_state = bool_value(bottom_square.get("bottom_square_changed_rotation_state", False))
        top_valid_after_square = True
        requires_post_square_remeasure = False
        computed["bottom_square_changed_rotation_state"] = bool(changed_rotation_state)
        computed["top_camera_correction_valid_after_bottom_square"] = bool(top_valid_after_square)
        computed["requires_post_square_remeasure"] = bool(requires_post_square_remeasure)
        if changed_rotation_state:
            log("BottomSquare", "changed_rotation_state=True")
            log("BottomSquare", "top_camera_pairwise_fit_collected_before_bottom_square=True")
        verify_single_circle_static_xy(camera, nozzle, part, cal_loc, cfg, state, computed)
        if is_pairwise_computed(computed):
            physical_verify = {"enabled": False,
                               "verification_mode": "skipped_for_pairwise_destructive_ab_not_applicable",
                               "verification_is_physical": False,
                               "before_samples": [],
                               "after_samples": [],
                               "verification_before_after_records": [],
                               "passed": bool_value(computed.get("pairwise_xy_candidate_valid", False)),
                               "warning": "Kasa-style destructive AB verification is not applicable to openpnp_pairwise_180_offset."}
            log("VerifyAB", "skipped=True reason=pairwise_destructive_ab_not_applicable")
            log("VerifyAB", "pairwise_xy_candidate_valid=%s" %
                str(bool_value(computed.get("pairwise_xy_candidate_valid", False))))
        elif bool_value(computed.get("model_quality_passed", False)):
            physical_verify = run_physical_ab_verification(camera, nozzle, part, cal_loc, cfg, state, computed)
        else:
            physical_verify = {"enabled": bool_value(cfg.get("single_circle_physical_ab_verification", True)),
                               "verification_mode": "skipped_model_quality_failed",
                               "verification_is_physical": False,
                               "before_samples": [],
                               "after_samples": [],
                               "verification_before_after_records": [],
                               "passed": False,
                               "warning": "model quality failed; destructive AB verification skipped"}
            computed["safe_to_apply"] = False
            computed["active_correction_allowed"] = False
            computed["active_correction_block_reason"] = "model quality failed; destructive AB verification skipped"
            log("VerifyAB", "skipped=True reason=model_quality_failed")
        computed["physical_ab_verification"] = physical_verify
        computed["persistent_candidate_verification"] = physical_verify
        apply_physical_verification_gate(computed, physical_verify, "pre_bottom_square")
        computed["verification_mode"] = physical_verify.get("verification_mode", "predicted_only_diagnostic")
        computed["verification_is_physical"] = bool_value(physical_verify.get("verification_is_physical", False))
        computed["verification_before_after_records"] = physical_verify.get("verification_before_after_records", [])
        computed["xy_fit_passed"] = bool_value(computed.get("model_quality_passed", False))
        computed["candidate_delta_available"] = True
        computed["diagnostic_preview_allowed"] = bool_value(cfg.get("confirm_allow_diagnostic_preview", True))
        computed["bottom_square_attempted"] = False
        computed["bottom_square_passed"] = None
        computed["static_vector_rotated_after_square"] = False
        computed["post_square_verification_passed"] = None
        computed["calibrated_return_allowed"] = False
        computed["persistent_apply_allowed"] = False
        computed["safe_return_allowed"] = True
        skip_bottom_square, bottom_square_skip_reason = should_skip_bottom_square_for_xy_failure(computed, physical_verify)
        if skip_bottom_square:
            bottom_square["skipped"] = True
            bottom_square["skip_reason"] = bottom_square_skip_reason
            bottom_square["verification_passed"] = False
            computed["bottom_square"] = bottom_square
            computed["bottom_square_attempted"] = False
            computed["bottom_square_passed"] = False
            computed["bottom_square_rotation_valid"] = False
            computed["square_rotation_bias_valid"] = False
            computed["square_rotation_bias_deg"] = 0.0
            log("BottomSquare", "skipped=True")
            log("BottomSquare", "skip_reason=%s" % str(bottom_square_skip_reason))
            log("BottomSquare", "xy_candidate_valid=%s" %
                str(bool_value(computed.get("pairwise_xy_candidate_valid", False))))
            log("BottomSquare", "persistent_apply_allowed=%s" %
                str(bool_value(computed.get("persistent_apply_allowed", False))))
        elif bool_value(cfg.get("run_bottom_camera_nozzle_square_after_fit", True)):
            if bool_value(state.get("die_on_nozzle", False)):
                raise Exception("Refusing bottom-camera nozzle square while die is on nozzle.")
            log("Order", "bottom_camera_nozzle_square_after_xy_offset_calibration=True")
            bottom_square = run_bottom_camera_nozzle_square(machine_obj, nozzle, cfg)
            computed["bottom_square_attempted"] = True
            computed["bottom_square_passed"] = bool_value(bottom_square.get("verification_passed", False))
            computed["square_rotation_bias_valid"] = bottom_square_bias_valid_from_result(bottom_square)
            computed["safe_return_allowed"] = bool_value(bottom_square.get("safe_return_allowed", True))
            if not bool_value(bottom_square.get("ran", False)):
                warning = "bottom square was enabled but did not run"
                bottom_square["warnings"].append(warning)
                log("BottomSquare", "WARNING %s" % warning)
                if not bool_value(cfg.get("bottom_square_warning_only", True)):
                    raise Exception(warning)
            if bool_value(bottom_square.get("ran", False)) and \
               bottom_square_bias_valid_from_result(bottom_square) and \
               bottom_square.get("selected_correction_deg", None) is not None:
                if str(computed.get("single_circle_fit_model_type", "")).strip().lower() == "openpnp_pairwise_180_offset":
                    log("BottomSquare", "pairwise_xy_candidate_independent_of_bottom_square=True")
                    computed["static_vector_rotated_after_square"] = False
                else:
                    rotate_static_vector_after_square(nozzle, computed, bottom_square, cfg)
                    computed["static_vector_rotated_after_square"] = True
                computed["square_rotation_bias_deg"] = float(bottom_square.get("square_rotation_bias_deg",
                                                                                bottom_square.get("selected_correction_deg", 0.0)) or 0.0)
                computed["square_rotation_bias_valid"] = True
                computed["bottom_square_rotation_valid"] = True
            elif bool_value(bottom_square.get("ran", False)) and not bottom_square_bias_valid_from_result(bottom_square):
                computed["square_rotation_bias_deg"] = 0.0
                computed["square_rotation_bias_valid"] = False
                computed["safe_return_allowed"] = True
                computed["bottom_square_rotation_valid"] = False
                log("SafetyState", "bottom_square_orientation_ambiguous=True")
                log("SafetyState", "bottom_square_blocks_xy_candidate=False")
                log("SafetyState", "pairwise_xy_candidate_valid=%s" %
                    str(bool_value(computed.get("pairwise_xy_candidate_valid", False))))
                log("SafetyState", "safe_return_allowed=True")
            if bool_value(bottom_square.get("ran", False)) and bool_value(bottom_square.get("verification_passed", False)):
                post_square_verify = run_post_square_static_xy_verification(camera, nozzle, part, cal_loc, cfg, state, computed)
                if bool_value(cfg.get("post_square_physical_pick_place_verify", False)):
                    computed["physical_ab_verification"] = post_square_verify
                    apply_physical_verification_gate(computed, post_square_verify, "post_bottom_square")
                computed["post_square_verification"] = post_square_verify
                computed["post_square_verification_passed"] = bool_value(post_square_verify.get("passed", False))
                if bool_value(cfg.get("post_square_physical_pick_place_verify", False)):
                    computed["verification_mode"] = post_square_verify.get("verification_mode", "post_square_physical_ab")
                    computed["verification_is_physical"] = bool_value(post_square_verify.get("verification_is_physical", False))
                    computed["verification_before_after_records"] = post_square_verify.get("verification_before_after_records", [])
                    computed["safe_to_apply"] = bool_value(computed.get("safe_to_apply", False)) and bool_value(post_square_verify.get("passed", False))
            else:
                computed["bottom_square_rotation_valid"] = False
                log("SafetyState", "bottom_square_did_not_pass_but_xy_candidate_preserved=True")
                computed["post_square_verification_passed"] = False
            post_bottom_square_lock, post_bottom_square_detected = relocate_die_after_bottom_square(
                camera, nozzle, part, cal_loc, cfg, state)
            computed["bottom_square"] = bottom_square
            changed_rotation_state = bool_value(bottom_square.get("bottom_square_changed_rotation_state", False))
            computed["bottom_square_changed_rotation_state"] = bool(changed_rotation_state)
            computed["top_camera_correction_valid_after_bottom_square"] = True
            computed["requires_post_square_remeasure"] = bool(changed_rotation_state)
            if changed_rotation_state:
                log("BottomSquare", "changed_rotation_state=True")
                log("BottomSquare", "xy_fit_was_collected_before_square=True")
                log("BottomSquare", "requires_post_square_remeasure=True")
        else:
            computed["bottom_square_attempted"] = False
            computed["bottom_square_passed"] = True
        computed["calibrated_return_allowed"] = bool_value(computed.get("candidate_delta_available", False)) and \
            bool_value(computed.get("active_correction_allowed", False))
        log("SafetyState", "candidate_delta_available=%s" % str(bool_value(computed.get("candidate_delta_available", False))))
        log("SafetyState", "diagnostic_preview_allowed=%s" % str(bool_value(computed.get("diagnostic_preview_allowed", True))))
        log("SafetyState", "active_correction_allowed=%s" % str(bool_value(computed.get("active_correction_allowed", False))))
        log("SafetyState", "calibrated_return_allowed=%s" % str(bool_value(computed.get("calibrated_return_allowed", False))))
        log("SafetyState", "safe_return_allowed=%s" % str(bool_value(computed.get("safe_return_allowed", True))))
        if rejected_samples:
            computed["rejected_samples"] = rejected_samples
        if bool_value(cfg.get("return_to_storage_after_bottom_square", True)):
            relock = None
            if post_bottom_square_lock is not None and post_bottom_square_detected is not None:
                relock = {"lock": post_bottom_square_lock, "detected": post_bottom_square_detected}
            storage_return = return_die_to_storage(camera, nozzle, part, storage_loc, cfg, state, computed, relock)
        else:
            storage_return = {"status": "skipped",
                              "reason": "return_to_storage_after_bottom_square is false",
                              "configured_storage_pocket_unchanged": True}
            log("StorageReturn", "skipped return_to_storage_after_bottom_square=False")
        computed["storage_return"] = storage_return
        result = {
            "timestamp": time.time(),
            "script_model": "single_circle",
            "script_file": "pick_cameraAlign_SingleCircleTipCalibration.py",
            "model": computed.get("model", "openpnp_pairwise_180_offset"),
            "computed_model": computed.get("model", "openpnp_pairwise_180_offset"),
            "single_circle_model_mode": computed.get("single_circle_model_mode", "openpnp_model_camera_offset"),
            "single_circle_fit_model_type": computed.get("single_circle_fit_model_type", "openpnp_pairwise_180_offset"),
            "fit_model_type": computed.get("fit_model_type", computed.get("single_circle_fit_model_type", "openpnp_pairwise_180_offset")),
            "single_circle_apply_model_sign": computed.get("single_circle_apply_model_sign", -1.0),
            "active_fit_source": computed.get("active_fit_source", "openpnp_pairwise_180_offset"),
            "calibration_frame": computed.get("calibration_frame", "camera_only_large_center_circle_mm"),
            "preview_frame": computed.get("preview_frame", "openpnp_nozzle_head_offsets"),
            "old_model_description": "current_active_openpnp_offsets",
            "new_model_description": computed.get("new_model_description", "openpnp_pairwise_180_static_nozzle_head_offset_candidate"),
            "persistent_write_allowed": True,
            "virtual_correction_available": True,
            "virtual_static_delta_x_mm": computed["selected_head_delta_x_mm"],
            "virtual_static_delta_y_mm": computed["selected_head_delta_y_mm"],
            "virtual_residual_angle_vector_x_mm": computed["residual_angle_vector_x_mm"],
            "virtual_residual_angle_vector_y_mm": computed["residual_angle_vector_y_mm"],
            "virtual_runout_preview_enabled": bool_value(computed.get("virtual_runout_preview_enabled", False)),
            "runout_model_enabled": bool_value(computed.get("runout_model_enabled", False)),
            "current_head_offsets_before": computed.get("current_head_offsets_before", None),
            "candidate_head_offsets_after": computed.get("candidate_head_offsets_after", None),
            "applied_head_offsets": None,
            "static_head_offset_delta_x_mm": computed.get("static_head_offset_delta_x_mm", computed["selected_head_delta_x_mm"]),
            "static_head_offset_delta_y_mm": computed.get("static_head_offset_delta_y_mm", computed["selected_head_delta_y_mm"]),
            "static_head_offset_delta_mag_mm": computed.get("static_head_offset_delta_mag_mm", computed["selected_head_delta_mag_mm"]),
            "square_rotation_bias_deg": bottom_square.get("square_rotation_bias_deg", bottom_square.get("selected_correction_deg", None)),
            "square_rotation_bias_valid": bottom_square_bias_valid_from_result(bottom_square),
            "bottom_square_measurement_valid": bool_value(bottom_square.get("measurement_valid", False)),
            "square_rotation_apply_mode": bottom_square.get("square_rotation_apply_mode", str(cfg.get("bottom_square_apply_mode", "script_local_rotation_bias"))),
            "bottom_square_changed_rotation_state": bool_value(computed.get("bottom_square_changed_rotation_state", False)),
            "top_camera_correction_valid_after_bottom_square": bool_value(computed.get("top_camera_correction_valid_after_bottom_square", True)),
            "requires_post_square_remeasure": bool_value(computed.get("requires_post_square_remeasure", False)),
            "apply_offsets_requested": False,
            "machine_config_modified": False,
            "part_id_or_name": cfg["part_id_or_name"],
            "nozzle_name": object_name(nozzle),
            "camera_name": object_name(camera),
            "config_snapshot": dict(cfg),
            "computed": computed,
            "samples": samples,
            "selected_head_delta_x_mm": computed["selected_head_delta_x_mm"],
            "selected_head_delta_y_mm": computed["selected_head_delta_y_mm"],
            "selected_head_delta_mag_mm": computed["selected_head_delta_mag_mm"],
            "bias_x_mm": computed["bias_x_mm"],
            "bias_y_mm": computed["bias_y_mm"],
            "model_rms_error_mm": computed.get("model_rms_error_mm", None),
            "model_peak_error_mm": computed.get("model_peak_error_mm", None),
            "residual_angle_vector_x_mm": computed["residual_angle_vector_x_mm"],
            "residual_angle_vector_y_mm": computed["residual_angle_vector_y_mm"],
            "residual_angle_radius_mm": computed["residual_angle_radius_mm"],
            "model_offset_at_0": computed.get("model_offset_at_0", None),
            "command_delta_at_0": computed.get("command_delta_at_0", None),
            "model_offset_at_90": computed.get("model_offset_at_90", None),
            "command_delta_at_90": computed.get("command_delta_at_90", None),
            "show_shift_delta_at_0": computed.get("show_shift_delta_at_0", None),
            "show_shift_delta_at_90": computed.get("show_shift_delta_at_90", None),
            "rms_error_mm": computed["rms_error_mm"],
            "peak_error_mm": computed["peak_error_mm"],
            "safe_to_apply": computed["safe_to_apply"],
            "calibration_failed": not bool_value(computed.get("safe_to_apply", False)),
            "model_quality_passed": computed.get("model_quality_passed", None),
            "active_correction_allowed": computed.get("active_correction_allowed", None),
            "active_correction_block_reason": computed.get("active_correction_block_reason", None),
            "diagnostic_preview_available": True,
            "apply_available": computed["apply_available"],
            "applied": None,
            "verification": {"model": computed.get("model", "openpnp_pairwise_180_offset"),
                             "mode": computed.get("verification_mode", None),
                             "verification_is_physical": bool_value(computed.get("verification_is_physical", False)),
                             "passed": bool_value(computed.get("safe_to_apply", False)),
                             "rms_error_mm": computed.get("rms_error_mm", None),
                             "peak_error_mm": computed.get("peak_error_mm", None),
                             "samples": samples,
                             "old_new": computed.get("verification_old_new", []),
                             "before_after_records": computed.get("verification_before_after_records", [])},
            "verification_mode": computed.get("verification_mode", None),
            "verification_is_physical": bool_value(computed.get("verification_is_physical", False)),
            "verification_before_after_records": computed.get("verification_before_after_records", []),
            "verification_old_new": computed.get("verification_old_new", []),
            "verification_old_new_available": len(computed.get("verification_old_new", [])) > 0,
            "verification_passed": bool_value(computed.get("safe_to_apply", False)),
            "xy_fit_passed": bool_value(computed.get("xy_fit_passed", False)),
            "bottom_square_attempted": bool_value(computed.get("bottom_square_attempted", False)),
            "bottom_square_passed": bool_value(computed.get("bottom_square_passed", False)),
            "static_vector_rotated_after_square": bool_value(computed.get("static_vector_rotated_after_square", False)),
            "post_square_verification_passed": bool_value(computed.get("post_square_verification_passed", False)),
            "calibrated_return_allowed": bool_value(computed.get("calibrated_return_allowed", False)),
            "persistent_apply_allowed": False,
            "safe_return_allowed": bool_value(computed.get("safe_return_allowed", True)),
            "candidate_delta_available": bool_value(computed.get("candidate_delta_available", False)),
            "diagnostic_preview_allowed": bool_value(computed.get("diagnostic_preview_allowed", True)),
            "storage": {"configured": location_to_diag(storage_loc),
                        "detected_initial_pick": location_to_diag(detected_storage),
                        "used_for_calibration": False},
            "initial_work_pose": location_to_diag(cal_lock["measurement_location"]),
            "post_bottom_square_work_pose": location_to_diag(post_bottom_square_lock["measurement_location"])
                                       if post_bottom_square_lock is not None else None,
            "storage_return": storage_return,
            "overlay": post_bottom_square_lock.get("overlay", None) if post_bottom_square_lock is not None else cal_lock.get("overlay", None)
        }
        result["apply_available"], result["apply_block_reason"] = apply_availability_for_result(result, cfg)
        computed["apply_available"] = result["apply_available"]
        computed["apply_block_reason"] = result["apply_block_reason"]
        computed["persistent_apply_allowed"] = bool_value(result["apply_available"])
        result["persistent_apply_allowed"] = bool_value(result["apply_available"])
        log("SafetyState", "persistent_apply_allowed=%s" % str(bool_value(result["apply_available"])))
        save_result(result)
        log_result_summary(result)
        return result
    except Exception, e:
        log("Error", e)
        if result is None:
            result = {"error": str(e), "timestamp": time.time(),
                      "script_model": "single_circle",
                      "script_file": "pick_cameraAlign_SingleCircleTipCalibration.py",
                      "traceback": traceback.format_exc()}
        else:
            result["error"] = str(e)
            result["traceback"] = traceback.format_exc()
        save_result(result)
        raise
    finally:
        if nozzle is not None and cfg is not None:
            try:
                loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
                cleanup_mode = str(cfg.get("single_circle_cleanup_rotation_mode", "leave_current")).lower()
                if bool_value(cfg.get("single_circle_unwind_after_run", False)) and cleanup_mode == "leave_current":
                    cleanup_mode = "return_to_start"
                cleanup_r = float(loc.getRotation())
                if cleanup_mode == "return_to_zero":
                    cleanup_r = 0.0
                elif cleanup_mode == "return_to_start":
                    cleanup_r = float(state.get("starting_nozzle_rotation_deg", loc.getRotation()))
                lift = single_circle_rotation_location(
                    nozzle, mm_loc(loc.getX(), loc.getY(), float(cfg["safe_travel_z"]), cleanup_r),
                    cfg, "Cleanup")
                base.move_to_with_speed(nozzle, lift, cfg)
                log("Cleanup", "Nozzle lifted to safe Z cleanup_rotation_mode=%s command_r=%.4f." %
                    (cleanup_mode, float(lift.getRotation())))
            except Exception, cleanup_error:
                log("Cleanup", "Final safe lift failed: %s" % cleanup_error)
        _progress_callback = old_progress
        _cancel_token = old_cancel


def gui_status():
    cfg = load_config()
    status = {"effective_mode": "single_circle",
              "last_result_model": None,
              "safe_to_apply": False,
              "apply_available": False,
              "shift_delta_mag_mm": None,
              "warning": None}
    try:
        result = load_last_result()
        computed = result.get("computed", {}) or {}
        status["last_result_model"] = computed.get("model", None)
        status["safe_to_apply"] = bool_value(computed.get("safe_to_apply", False))
        status["apply_available"], reason = apply_availability_for_result(result, cfg)
        status["apply_block_reason"] = reason
        status["shift_delta_mag_mm"] = computed.get("selected_head_delta_mag_mm", None)
    except Exception, e:
        status["last_result_error"] = str(e)
    return status


def open_last_log():
    return result_path()


def open_last_result():
    return result_path()


if __name__ == "__main__":
    print json.dumps(run(False), sort_keys=True, indent=2)
