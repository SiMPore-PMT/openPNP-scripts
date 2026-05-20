# Jython 2.7 / OpenPnP
# Single-mode pick/nozzle-to-top-camera calibration using a 4-circle object.
#
# This script intentionally supports only round_object_nozzle_axis. Older
# calibration modes were removed because silent mode selection was the failure
# mode: new code existed, but an old selected path could still execute.

import json
import math
import os
import shutil
import time
import traceback

from java.io import File
from java.lang import Thread, Throwable
from javax.imageio import ImageIO
from org.openpnp.model import Configuration, Location, LengthUnit
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.spi import Nozzle as SpiNozzle
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import MovableUtils, VisionUtils, OpenCvUtils

TAG = "PickCameraAlignCalibration"
CONFIG_FILE = "pick_cameraAlign_Calibration_config.json"
RESULT_FILE = "pick_cameraAlign_Calibration_last_result.json"
LAST_CONFIRM_FILE = "pick_cameraAlign_Calibration_last_confirm.json"
LAST_IMAGE_FILE = "pick_cameraAlign_Calibration_last_capture.png"
LAST_OVERLAY_FILE = "pick_cameraAlign_Calibration_last_overlay.json"

_progress_callback = None
_cancel_token = None
_legacy_config_warning_logged = False
_last_vision_image_path = None
_last_capture_image_size = None
_last_pipeline_working_image_size = None
_last_pose_image_size = None
_last_pose_image_size_source = None

PIPELINE_ATTEMPTS = 3
SETTLE_MS = 250


class RoundAxisResidualSampleAbort(Exception):
    def __init__(self, message, samples=None):
        Exception.__init__(self, message)
        self.samples = samples or []


RoundAxisRawSampleAbort = RoundAxisResidualSampleAbort

DEFAULT_CONFIG = {
    "part_id_or_name": "PickCameraAlignDie",
    "nozzle_name": "Pick Head",
    "camera_name": "",

    "die_storage_location_xyz": {"x": 10.0, "y": 10.0, "z": 2.0, "rotation": 0.0},
    "cal_work_location_xyz": {"x": 40.0, "y": 40.0, "z": 2.0, "rotation": 0.0},
    "safe_travel_z": 25.0,

    "calibration_nozzle_motion_strategy": "local_clearance_split",
    "storage_nozzle_motion_strategy": "machine_safe_split",
    "calibration_local_clearance_mm": 0.75,
    "storage_local_clearance_mm": 0.75,
    "split_rotation_from_xy_moves": True,
    "max_nozzle_rotation_step_deg": 120.0,
    "round_object_max_sample_rotation_step_deg": 181.0,
    "round_object_allow_large_sample_rotation": True,
    "abort_on_large_nozzle_rotation_step": True,
    "bound_nozzle_rotation_commands": True,
    "use_openpnp_safe_z_for_pick_place": False,

    "use_high_level_nozzle_pick": True,
    "require_visual_lock_before_pick": True,
    "align_nozzle_to_detected_die_rotation": True,
    "force_vacuum_actuator_name": "",
    "force_blowoff_actuator_name": "",
    "vacuum_settle_ms": 150,
    "blowoff_pulse_ms": 120,
    "enable_blowoff_on_place": False,

    "circle_detection_min_count": 4,
    "circle_result_stage_name": "results",
    "orientation_circle_from_results": True,
    "orientation_circle_max_square_size_ratio": 0.8,
    "require_orientation_circle": True,
    "orientation_x_sign": -1.0,
    "orientation_snap_to_cardinal": True,
    "orientation_max_error_deg": 25.0,
    "die_center_source": "four_circle_square_center",
    "die_center_crosscheck_enabled": True,
    "die_center_max_circle_geometry_rms_px": 3.0,

    "validate_pixel_to_machine_transform": True,
    "pixel_to_machine_transform_method": "camera_only",
    "pixel_transform_require_camera_at_requested_z": True,
    "pixel_transform_max_camera_z_error_mm": 0.01,
    "pixel_transform_compare_camera_only_and_tool_aware": True,
    "require_pipeline_working_image_size_for_pose_pixels": True,
    "allow_raw_camera_size_fallback_for_pose_pixels": False,
    "log_image_frame_dimensions": True,

    "circle_selection_require_size_data": True,
    "circle_selection_corner_count": 4,
    "circle_selection_orientation_count": 1,
    "circle_selection_min_raw_count": 5,
    "circle_selection_min_corner_radius_px": 50.0,
    "circle_selection_max_corner_radius_px": 120.0,
    "circle_selection_min_orientation_radius_px": 10.0,
    "circle_selection_max_orientation_radius_px": 70.0,
    "circle_selection_orientation_max_corner_radius_ratio": 0.80,
    "circle_selection_corner_radius_rms_max_px": 3.0,
    "circle_selection_corner_square_side_rms_max_px": 5.0,
    "circle_selection_reject_first_four_fallback": True,

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
    },

    "calibration_result_mode": "round_object_nozzle_axis",
    "round_object_angles_deg": [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0],
    "round_object_cycles": 2,
    "round_object_pick_rotation_mode": "current_die_rotation",
    "round_object_pick_rotation_deg": 0.0,
    "round_object_rotate_after_pick": True,
    "round_object_place_rotation_mode": "sample_angle",
    "round_object_measure_after_place": True,
    "round_object_fit_static_offset": True,
    "round_object_fit_runout": True,
    "round_object_max_fit_rms_mm": 0.08,
    "round_object_max_fit_peak_mm": 0.20,
    "round_object_max_static_offset_mm": 2.0,
    "round_object_max_runout_radius_mm": 2.0,
    "round_object_max_sample_error_mm": 0.35,
    "round_object_max_sample_error_before_abort_mm": 0.60,
    "round_object_retry_bad_sample_once": False,
    "round_object_abort_on_bad_sample": False,
    "round_object_raw_sample_error_warn_mm": 0.75,
    "round_object_raw_sample_error_abort_mm": 999.0,
    "round_object_abort_on_raw_sample_error": False,
    "round_object_min_good_samples": 6,
    "round_object_block_on_fit_residuals": False,
    "round_object_fit_residual_warning_only": True,
    "allow_high_residual_diagnostic_preview": True,
    "allow_high_residual_final_return_preview": True,
    "allow_high_residual_apply": False,
    "round_object_head_delta_sign_override": 1.0,
    "round_object_disable_detected_orientation_alignment": True,
    "round_object_validate_die_orientation": False,
    "round_object_force_camera_only_transform": True,
    "allow_tool_aware_transform_in_round_object_mode": False,
    "avoid_exact_180_deg": False,

    "allow_machine_writes": False,
    "allow_apply_without_verification": False,
    "max_apply_mm": 0.5,
    "write_machine_xml_backup": True,
    "machine_xml_path": None,
    "apply_target": "nozzle_head_offsets",

    "confirm_shift_descend_to_surface": False,
    "confirm_visual_min_shift_mm": 0.05,
    "confirm_allow_unsafe_diagnostic_preview": False,

    "spiral_search": {
        "enabled": True,
        "start_radius_mm": 0.15,
        "radius_step_mm": 0.15,
        "max_radius_mm": 1.20,
        "angle_step_deg": 45.0,
        "settle_ms": 150
    }
}


# ---------------------------------------------------------------------------
# config / utility

def log(tag, msg):
    line = "[%s][%s] %s" % (TAG, tag, str(msg))
    print line
    try:
        if _progress_callback is not None:
            _progress_callback(tag, str(msg), line)
    except:
        pass


def log_compensation_stack_assumptions():
    log("CompStack", "Existing OpenPnP compensation is assumed active during nozzle moves.")
    log("CompStack", "Samples are residual-on-existing-compensation, not raw nozzle runout.")
    log("CompStack", "Same-run preview correction uses residual delta only.")
    log("CompStack", "Persistent apply writes nozzle head offset delta only.")
    log("CompStack", "Runout writes are disabled.")


def compensation_stack_result_fields():
    return {
        "existing_openpnp_compensation_assumed_active": True,
        "measurement_interpretation": "residual_on_existing_compensation",
        "same_run_preview_correction_type": "residual_delta_only",
        "persistent_apply_type": "nozzle_head_offset_delta_only",
        "runout_write_allowed": False,
        "model_composition_warning": "If persistent runout/nozzle-tip compensation is ever written, it must be composed with the existing OpenPnP model. Residual fitted values must not replace existing compensation."
    }


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
    try:
        return bool(token.get("cancelled", False))
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


def deep_update_known(base, updates):
    out = dict(base)
    ignored = []
    for k in updates.keys():
        if k not in base:
            ignored.append(k)
            continue
        if isinstance(base.get(k), dict) and isinstance(updates.get(k), dict):
            child = dict(base[k])
            for ck in updates[k].keys():
                if ck in child:
                    child[ck] = updates[k][ck]
                else:
                    ignored.append("%s.%s" % (k, ck))
            out[k] = child
        else:
            out[k] = updates[k]
    return out, ignored


def deep_update(base, updates):
    out = dict(base)
    for k in updates.keys():
        if isinstance(updates[k], dict) and isinstance(out.get(k), dict):
            child = dict(out[k])
            child.update(updates[k])
            out[k] = child
        else:
            out[k] = updates[k]
    return out


def calibration_result_mode(cfg):
    mode = str(cfg.get("calibration_result_mode", "round_object_nozzle_axis")).lower()
    if mode != "round_object_nozzle_axis":
        raise Exception("Old calibration_result_mode=%s is no longer supported. Set calibration_result_mode=round_object_nozzle_axis." % mode)
    return "round_object_nozzle_axis"


def enforce_round_object_pixel_transform_policy(cfg):
    mode = calibration_result_mode(cfg)
    method = str(cfg.get("pixel_to_machine_transform_method", "camera_only")).lower()
    if mode == "round_object_nozzle_axis" and method != "camera_only":
        force_camera_only = bool_value(cfg.get("round_object_force_camera_only_transform", True))
        allow_tool_aware = bool_value(cfg.get("allow_tool_aware_transform_in_round_object_mode", False))
        if force_camera_only and not allow_tool_aware:
            log("Config", "Forcing pixel_to_machine_transform_method=camera_only for round_object_nozzle_axis; old config had %s." % method)
            cfg["pixel_to_machine_transform_method"] = "camera_only"
        elif not allow_tool_aware:
            raise Exception("tool_aware transform is not allowed in round_object_nozzle_axis because nozzle/tool offset is being calibrated.")
    return cfg


def assert_round_object_pixel_transform_policy(cfg):
    mode = calibration_result_mode(cfg)
    method = str(cfg.get("pixel_to_machine_transform_method", "camera_only")).lower()
    if mode == "round_object_nozzle_axis" and method == "tool_aware":
        if not bool_value(cfg.get("allow_tool_aware_transform_in_round_object_mode", False)):
            raise Exception("tool_aware transform is not allowed in round_object_nozzle_axis because nozzle/tool offset is being calibrated.")


def normalize_round_object_place_rotation_mode(cfg):
    mode = str(cfg.get("round_object_place_rotation_mode", "sample_angle")).lower()
    if mode == "same_as_pick_angle":
        log("Config", "Migrating round_object_place_rotation_mode=same_as_pick_angle to sample_angle.")
        mode = "sample_angle"
    cfg["round_object_place_rotation_mode"] = mode
    return mode


def save_config(cfg):
    clean = {}
    for key in DEFAULT_CONFIG.keys():
        clean[key] = cfg.get(key, DEFAULT_CONFIG[key])
    f = open(config_path(), "w")
    try:
        f.write(json.dumps(clean, sort_keys=True, indent=2))
    finally:
        f.close()


def load_config():
    global _legacy_config_warning_logged
    path = config_path()
    if not os.path.exists(path):
        cfg = dict(DEFAULT_CONFIG)
        normalize_round_object_place_rotation_mode(cfg)
        sanitize_config_angles(cfg)
        validate_config(cfg)
        save_config(cfg)
        log("Config", "Config did not exist; wrote %s" % path)
        return cfg
    f = open(path, "r")
    try:
        data = json.loads(f.read())
    finally:
        f.close()
    old_mode = str(data.get("calibration_result_mode", DEFAULT_CONFIG["calibration_result_mode"])).lower()
    if old_mode != "round_object_nozzle_axis":
        raise Exception("Old calibration_result_mode=%s is no longer supported. Set calibration_result_mode=round_object_nozzle_axis." % old_mode)
    cfg, ignored = deep_update_known(DEFAULT_CONFIG, data)
    if ignored and not _legacy_config_warning_logged:
        log("Config", "Ignored legacy keys: %s" % ", ".join(sorted(ignored)))
        _legacy_config_warning_logged = True
    cfg["calibration_result_mode"] = calibration_result_mode(cfg)
    enforce_round_object_pixel_transform_policy(cfg)
    normalize_round_object_place_rotation_mode(cfg)
    sanitize_config_angles(cfg)
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


def require_key(cfg, key):
    if key not in cfg:
        raise Exception("Missing required config key: %s" % key)


def bool_value(value):
    try:
        text = str(value).strip().lower()
        if text in ["false", "0", "no", "off"]:
            return False
        if text in ["true", "1", "yes", "on"]:
            return True
    except:
        pass
    return bool(value)


def is_bool_argument(value):
    if value is True or value is False:
        return True
    try:
        return str(value).strip().lower() in ["true", "false", "1", "0", "yes", "no", "on", "off"]
    except:
        return False


def finite_number(value):
    try:
        v = float(value)
        return not (math.isnan(v) or math.isinf(v))
    except:
        return False


def mean(values):
    if not values:
        return 0.0
    return sum(values) / float(len(values))


def rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum([v * v for v in values]) / float(len(values)))


def normalize_angle(angle):
    angle = float(angle)
    while angle <= -180.0:
        angle += 360.0
    while angle > 180.0:
        angle -= 360.0
    return angle


def nearest_cardinal_angle(angle_deg):
    angle_deg = normalize_angle(angle_deg)
    best = 0.0
    best_error = None
    for candidate in [0.0, 90.0, 180.0, -90.0]:
        err = abs(normalize_angle(angle_deg - candidate))
        if best_error is None or err < best_error:
            best = candidate
            best_error = err
    return normalize_angle(best)


def rotation_error_deg(actual, expected):
    return normalize_angle(float(actual) - float(expected))


def rotate_xy(x, y, angle_deg):
    a = math.radians(float(angle_deg))
    return {"x": math.cos(a) * float(x) - math.sin(a) * float(y),
            "y": math.sin(a) * float(x) + math.cos(a) * float(y)}


def sanitize_rotation_angle_list(values, name, avoid_exact_180):
    out = []
    seen = {}
    for value in values:
        if avoid_exact_180:
            angle = normalize_angle(float(value))
            if abs(abs(angle) - 180.0) < 0.000001:
                if angle >= 0.0:
                    angle = 179.0
                else:
                    angle = -179.0
                log("Config", "adjusted exact 180 angle to avoid wrap: %s -> %.1f" %
                    (str(value), angle))
            key = "%.6f" % angle
        else:
            angle = float(value)
            key = "%.6f" % (angle % 360.0)
        if key not in seen:
            seen[key] = True
            out.append(angle)
    if len(out) != len(values):
        log("Config", "%s sanitized to %s" % (name, str(out)))
    return out


def sanitize_config_angles(cfg):
    cfg["round_object_angles_deg"] = sanitize_rotation_angle_list(
        cfg.get("round_object_angles_deg", DEFAULT_CONFIG["round_object_angles_deg"]),
        "round_object_angles_deg", bool_value(cfg.get("avoid_exact_180_deg", True)))


def validate_config(cfg):
    for key in DEFAULT_CONFIG.keys():
        require_key(cfg, key)
    for key in ["x", "y", "z"]:
        require_key(cfg["die_storage_location_xyz"], key)
        require_key(cfg["cal_work_location_xyz"], key)
    calibration_result_mode(cfg)
    if int(cfg["circle_detection_min_count"]) != 4:
        raise Exception("circle_detection_min_count must be 4.")
    if str(cfg.get("die_center_source", "")).lower() != "four_circle_square_center":
        raise Exception("die_center_source must be four_circle_square_center.")
    if str(cfg.get("pixel_to_machine_transform_method", "camera_only")).lower() not in ["tool_aware", "camera_only"]:
        raise Exception("pixel_to_machine_transform_method must be tool_aware or camera_only.")
    for key in ["calibration_nozzle_motion_strategy", "storage_nozzle_motion_strategy"]:
        if str(cfg.get(key, "")).lower() not in ["machine_safe_split", "local_clearance_split"]:
            raise Exception("%s must be machine_safe_split or local_clearance_split." % key)
    if str(cfg.get("round_object_place_rotation_mode", "sample_angle")).lower() not in ["sample_angle", "zero", "paired_opposite"]:
        raise Exception("round_object_place_rotation_mode must be sample_angle, zero, or paired_opposite.")
    if str(cfg.get("round_object_pick_rotation_mode", "current_die_rotation")).lower() not in ["fixed", "current_nozzle_rotation", "current_die_rotation", "previous_place_rotation"]:
        raise Exception("round_object_pick_rotation_mode must be fixed, current_nozzle_rotation, current_die_rotation, or previous_place_rotation.")
    if len(cfg.get("round_object_angles_deg", [])) < 4:
        raise Exception("round_object_angles_deg must contain at least four unique angles.")
    if int(cfg.get("round_object_cycles", 1)) < 1:
        raise Exception("round_object_cycles must be >= 1.")
    if str(cfg.get("apply_target", "nozzle_head_offsets")).lower() != "nozzle_head_offsets":
        raise Exception("apply_target must be nozzle_head_offsets.")
    if not finite_number(cfg.get("round_object_head_delta_sign_override", 1.0)):
        raise Exception("round_object_head_delta_sign_override must be finite.")
    for key in ["abort_on_large_nozzle_rotation_step",
                "round_object_disable_detected_orientation_alignment",
                "round_object_rotate_after_pick",
                "round_object_validate_die_orientation",
                "round_object_force_camera_only_transform",
                "allow_tool_aware_transform_in_round_object_mode",
                "round_object_allow_large_sample_rotation",
                "avoid_exact_180_deg",
                "circle_selection_require_size_data",
                "circle_selection_reject_first_four_fallback",
                "round_object_retry_bad_sample_once",
                "round_object_abort_on_bad_sample",
                "round_object_abort_on_raw_sample_error",
                "round_object_block_on_fit_residuals",
                "round_object_fit_residual_warning_only",
                "allow_high_residual_diagnostic_preview",
                "allow_high_residual_final_return_preview",
                "allow_high_residual_apply",
                "confirm_allow_unsafe_diagnostic_preview"]:
        if not is_bool_argument(cfg.get(key, False)):
            raise Exception("%s must be boolean-compatible." % key)
    if int(cfg.get("circle_selection_corner_count", 4)) != 4:
        raise Exception("circle_selection_corner_count must be 4.")
    if int(cfg.get("circle_selection_orientation_count", 1)) != 1:
        raise Exception("circle_selection_orientation_count must be 1.")
    if int(cfg.get("circle_selection_min_raw_count", 5)) < 5:
        raise Exception("circle_selection_min_raw_count must be >= 5.")
    if float(cfg["safe_travel_z"]) <= float(cfg["die_storage_location_xyz"]["z"]):
        raise Exception("safe_travel_z must be above die_storage_location_xyz.z.")
    if float(cfg["safe_travel_z"]) <= float(cfg["cal_work_location_xyz"]["z"]):
        raise Exception("safe_travel_z must be above cal_work_location_xyz.z.")
    for key in ["max_nozzle_rotation_step_deg",
                "round_object_max_sample_rotation_step_deg",
                "round_object_max_fit_rms_mm", "round_object_max_fit_peak_mm",
                "round_object_max_static_offset_mm", "round_object_max_runout_radius_mm",
                "round_object_max_sample_error_mm",
                "round_object_max_sample_error_before_abort_mm",
                "round_object_raw_sample_error_warn_mm",
                "round_object_raw_sample_error_abort_mm",
                "round_object_min_good_samples",
                "max_apply_mm", "confirm_visual_min_shift_mm",
                "circle_selection_min_raw_count",
                "circle_selection_min_corner_radius_px",
                "circle_selection_max_corner_radius_px",
                "circle_selection_min_orientation_radius_px",
                "circle_selection_max_orientation_radius_px",
                "circle_selection_orientation_max_corner_radius_ratio",
                "circle_selection_corner_radius_rms_max_px",
                "circle_selection_corner_square_side_rms_max_px"]:
        if float(cfg[key]) <= 0.0:
            raise Exception("%s must be > 0." % key)
    if float(cfg["circle_selection_min_corner_radius_px"]) > float(cfg["circle_selection_max_corner_radius_px"]):
        raise Exception("circle_selection_min_corner_radius_px must be <= circle_selection_max_corner_radius_px.")
    if float(cfg["circle_selection_min_orientation_radius_px"]) > float(cfg["circle_selection_max_orientation_radius_px"]):
        raise Exception("circle_selection_min_orientation_radius_px must be <= circle_selection_max_orientation_radius_px.")


# ---------------------------------------------------------------------------
# locations / runtime

def mm_loc(x, y, z, r):
    return Location(LengthUnit.Millimeters, float(x), float(y), float(z), float(r))


def loc_from_xyz(data):
    return mm_loc(data.get("x", 0.0), data.get("y", 0.0),
                  data.get("z", 0.0), data.get("rotation", 0.0))


def location_to_diag(loc):
    if loc is None:
        return None
    loc = loc.convertToUnits(LengthUnit.Millimeters)
    return {"x": float(loc.getX()), "y": float(loc.getY()),
            "z": float(loc.getZ()), "rotation": float(loc.getRotation())}


def length_to_mm(value):
    if value is None:
        return None
    try:
        return float(value.convertToUnits(LengthUnit.Millimeters).getValue())
    except:
        try:
            return float(value)
        except:
            return None


def infer_part_height_mm(part):
    try:
        if part.isPartHeightUnknown():
            return 0.0
    except:
        pass
    for name in ["getHeight", "getHeightForSafeZ"]:
        try:
            value = length_to_mm(getattr(part, name)())
            if value is not None and value >= 0.0:
                return value
        except:
            pass
    return 0.0


def part_height_z_mm(cfg):
    return float(cfg.get("_part_height_z_mm", 0.0))


def pick_surface_z_mm(base_loc, cfg):
    return float(base_loc.getZ()) + part_height_z_mm(cfg)


def storage_pick_surface_z_mm(cfg, storage_loc=None):
    if storage_loc is None:
        storage_loc = loc_from_xyz(cfg["die_storage_location_xyz"])
    return pick_surface_z_mm(storage_loc, cfg)


def camera_z_for_location(base_loc, cfg):
    return pick_surface_z_mm(base_loc, cfg)


def wait_still(movable):
    try:
        movable.waitForCompletion(CompletionType.WaitForStillstand)
    except:
        pass


def object_name(obj):
    if obj is None:
        return "None"
    try:
        return str(obj.getName())
    except:
        pass
    try:
        return str(obj.getId())
    except:
        pass
    return str(obj)


def get_machine():
    try:
        return machine
    except:
        return Configuration.get().getMachine()


def get_config():
    try:
        return config
    except:
        return Configuration.get()


def java_list_to_py(model):
    if model is None:
        return []
    if hasattr(model, "size") and hasattr(model, "get"):
        out = []
        for i in range(model.size()):
            out.append(model.get(i))
        return out
    try:
        return list(model)
    except:
        return [model]


def find_nozzle(head, name):
    nozzle = None
    if name:
        try:
            nozzle = head.getNozzleByName(name)
        except:
            nozzle = None
    if nozzle is None:
        nozzle = head.getDefaultNozzle()
    if nozzle is None:
        raise Exception("Nozzle not found: %s" % name)
    return nozzle


def find_top_camera(machine_obj, head, camera_name):
    if camera_name:
        for owner in [head, machine_obj]:
            try:
                for cam in owner.getCameras():
                    if cam.getName() == camera_name:
                        return cam
            except:
                pass
    try:
        cam = head.getDefaultCamera()
        if cam is not None and (not camera_name or cam.getName() == camera_name):
            return cam
    except:
        pass
    try:
        for cam in machine_obj.getCameras():
            try:
                if cam.getLooking() == SpiCamera.Looking.Down:
                    if not camera_name or cam.getName() == camera_name:
                        return cam
            except:
                pass
    except:
        pass
    raise Exception("Top/down-looking camera not found: %s" % camera_name)


def get_part(part_id_or_name):
    cfg_obj = get_config()
    try:
        part = cfg_obj.getPart(part_id_or_name)
        if part is not None:
            return part
    except:
        pass
    try:
        for part in cfg_obj.getParts():
            if part.getId() == part_id_or_name or part.getName() == part_id_or_name:
                return part
    except:
        pass
    raise Exception("Part not found by id/name: %s" % part_id_or_name)


def _prepare_runtime(cfg):
    machine_obj = get_machine()
    head = machine_obj.getDefaultHead()
    if head is None:
        raise Exception("Machine default head is null.")
    nozzle = find_nozzle(head, cfg["nozzle_name"])
    camera = find_top_camera(machine_obj, head, cfg["camera_name"])
    part = get_part(cfg["part_id_or_name"])
    cfg["_part_height_z_mm"] = infer_part_height_mm(part)
    storage_loc = loc_from_xyz(cfg["die_storage_location_xyz"])
    cal_loc = loc_from_xyz(cfg["cal_work_location_xyz"])
    if float(cfg["safe_travel_z"]) <= pick_surface_z_mm(storage_loc, cfg):
        raise Exception("safe_travel_z must be above storage surface Z.")
    if float(cfg["safe_travel_z"]) <= pick_surface_z_mm(cal_loc, cfg):
        raise Exception("safe_travel_z must be above calibration surface Z.")
    log("Runtime", "Nozzle=%s Camera=%s Part=%s part_height=%.4f" %
        (object_name(nozzle), object_name(camera), object_name(part), part_height_z_mm(cfg)))
    return machine_obj, head, nozzle, camera, part, storage_loc, cal_loc


def move_tool_safe(movable, target, tag):
    check_cancel(tag)
    target = target.convertToUnits(LengthUnit.Millimeters)
    log(tag, "Safe move %s -> X=%.4f Y=%.4f Z=%.4f R=%.4f" %
        (object_name(movable), target.getX(), target.getY(), target.getZ(), target.getRotation()))
    MovableUtils.moveToLocationAtSafeZ(movable, target)
    wait_still(movable)


def move_to_with_speed(movable, target, cfg):
    target = target.convertToUnits(LengthUnit.Millimeters)
    try:
        movable.moveTo(target, 1.0)
    except TypeError:
        movable.moveTo(target)
    wait_still(movable)


def nearest_equivalent_rotation(target_deg, current_deg):
    target = normalize_angle(target_deg)
    current = float(current_deg)
    current_norm = normalize_angle(current)
    delta = normalize_angle(target - current_norm)
    return current + delta


def check_nozzle_rotation_step_for_context(command_r, current_r, cfg, context):
    delta = float(command_r) - float(current_r)
    if context == "RoundAxisRotateWithObject":
        max_step = float(cfg.get("round_object_max_sample_rotation_step_deg", 181.0))
        allow_large = bool_value(cfg.get("round_object_allow_large_sample_rotation", True))
    else:
        max_step = float(cfg.get("max_nozzle_rotation_step_deg", 120.0))
        allow_large = False
    if abs(delta) > max_step:
        if context == "RoundAxisRotateWithObject":
            msg = "Refusing large calibration sample rotation step %.4f deg > round_object_max_sample_rotation_step_deg %.4f" % (delta, max_step)
        else:
            msg = "Refusing large nozzle rotation step %.4f deg > max_nozzle_rotation_step_deg %.4f" % (delta, max_step)
        raise Exception(msg)
    if context == "RoundAxisRotateWithObject" and abs(delta) > float(cfg.get("max_nozzle_rotation_step_deg", 120.0)):
        if allow_large:
            log("Rotate", "Allowing large calibration sample rotation %.4f deg; this is expected for round_object_nozzle_axis." % delta)
        else:
            raise Exception("Refusing large calibration sample rotation %.4f deg because round_object_allow_large_sample_rotation is false." % delta)
    return delta


def nearest_nozzle_rotation_location(nozzle, target, tag, cfg):
    target = target.convertToUnits(LengthUnit.Millimeters)
    try:
        current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        current_r = float(current.getRotation())
        requested_r = float(target.getRotation())
        r = nearest_equivalent_rotation(requested_r, current_r)
        delta = check_nozzle_rotation_step_for_context(r, current_r, cfg, "Normal")
        log("Rotate", "current_r=%.4f requested_target_r=%.4f command_r=%.4f delta_r=%.4f short_path=True" %
            (current_r, requested_r, r, delta))
    except:
        raise
    return mm_loc(target.getX(), target.getY(), target.getZ(), r)


def nozzle_motion_strategy_for_context(cfg, context):
    if context == "Storage":
        return str(cfg.get("storage_nozzle_motion_strategy", "machine_safe_split")).lower()
    return str(cfg.get("calibration_nozzle_motion_strategy", "local_clearance_split")).lower()


def nozzle_local_clearance_for_context(cfg, context):
    if context == "Storage":
        return float(cfg.get("storage_local_clearance_mm", 0.75))
    return float(cfg.get("calibration_local_clearance_mm", 0.75))


def move_nozzle_split(nozzle, final_loc, cfg, tag, context):
    final_loc = final_loc.convertToUnits(LengthUnit.Millimeters)
    if bool_value(cfg.get("use_openpnp_safe_z_for_pick_place", False)):
        move_tool_safe(nozzle, nearest_nozzle_rotation_location(nozzle, final_loc, tag, cfg), tag)
        return
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    strategy = nozzle_motion_strategy_for_context(cfg, context)
    if strategy == "machine_safe_split":
        clearance_z = float(cfg["safe_travel_z"])
    else:
        clearance_z = max(float(current.getZ()), float(final_loc.getZ()) + nozzle_local_clearance_for_context(cfg, context))
    log(tag, "context=%s strategy=%s clearance_z=%.4f" % (context, strategy, clearance_z))
    lift = nearest_nozzle_rotation_location(nozzle, mm_loc(current.getX(), current.getY(), clearance_z, current.getRotation()), tag, cfg)
    log("Move", "phase=lift_to_clearance tag=%s context=%s x=%.4f y=%.4f z=%.4f r=%.4f" %
        (tag, context, lift.getX(), lift.getY(), lift.getZ(), lift.getRotation()))
    move_to_with_speed(nozzle, lift, cfg)
    after_lift = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    r = nearest_equivalent_rotation(final_loc.getRotation(), after_lift.getRotation())
    check_nozzle_rotation_step_for_context(r, after_lift.getRotation(), cfg, "Normal")
    log("Rotate", "current_r=%.4f requested_target_r=%.4f command_r=%.4f delta_r=%.4f short_path=True" %
        (float(after_lift.getRotation()), float(final_loc.getRotation()), float(r),
         float(r) - float(after_lift.getRotation())))
    if bool_value(cfg.get("split_rotation_from_xy_moves", True)):
        log("Move", "phase=rotate_only tag=%s context=%s x=%.4f y=%.4f z=%.4f r=%.4f" %
            (tag, context, after_lift.getX(), after_lift.getY(), clearance_z, r))
        move_to_with_speed(nozzle, mm_loc(after_lift.getX(), after_lift.getY(), clearance_z, r), cfg)
        after_rotate = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        log("Move", "phase=xy_only tag=%s context=%s x=%.4f y=%.4f z=%.4f r=%.4f" %
            (tag, context, final_loc.getX(), final_loc.getY(), clearance_z, after_rotate.getRotation()))
        move_to_with_speed(nozzle, mm_loc(final_loc.getX(), final_loc.getY(), clearance_z, after_rotate.getRotation()), cfg)
    else:
        log("Move", "phase=xy_with_rotation tag=%s context=%s x=%.4f y=%.4f z=%.4f r=%.4f" %
            (tag, context, final_loc.getX(), final_loc.getY(), clearance_z, r))
        move_to_with_speed(nozzle, mm_loc(final_loc.getX(), final_loc.getY(), clearance_z, r), cfg)
    after_xy = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    log("Move", "phase=z_only tag=%s context=%s x=%.4f y=%.4f z=%.4f r=%.4f" %
        (tag, context, after_xy.getX(), after_xy.getY(), final_loc.getZ(), after_xy.getRotation()))
    move_to_with_speed(nozzle, mm_loc(after_xy.getX(), after_xy.getY(), final_loc.getZ(), after_xy.getRotation()), cfg)


def lift_nozzle_after_pick_place(nozzle, cfg, tag, context, final_z):
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    strategy = nozzle_motion_strategy_for_context(cfg, context)
    if strategy == "machine_safe_split":
        z = float(cfg["safe_travel_z"])
    else:
        z = max(float(current.getZ()), float(final_z) + nozzle_local_clearance_for_context(cfg, context))
    target = nearest_nozzle_rotation_location(nozzle, mm_loc(current.getX(), current.getY(), z, current.getRotation()), tag, cfg)
    move_to_with_speed(nozzle, target, cfg)


# ---------------------------------------------------------------------------
# vision

def get_part_pipeline(part):
    settings = None
    try:
        settings = part.getFiducialVisionSettings()
    except:
        pass
    if settings is None:
        raise Exception("Part '%s' has no FiducialVisionSettings." % object_name(part))
    pipeline = settings.getPipeline()
    if pipeline is None:
        raise Exception("Part '%s' fiducial pipeline is null." % object_name(part))
    return pipeline


def get_model(result):
    if result is None:
        return None
    try:
        return result.getModel()
    except:
        try:
            return result.model
        except:
            return None


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


def radius_from_model_item(item):
    radius = number_attr(item, ["radius", "getRadius"])
    if radius is not None:
        return radius
    diameter = number_attr(item, ["diameter", "getDiameter", "size", "getSize"])
    if diameter is not None:
        return diameter / 2.0
    return None


def point_from_model_item(item):
    if isinstance(item, dict):
        if "x" in item and "y" in item:
            out = {"x_px": float(item["x"]), "y_px": float(item["y"])}
        elif "x_px" in item and "y_px" in item:
            out = {"x_px": float(item["x_px"]), "y_px": float(item["y_px"])}
        else:
            return None
        for key in ["radius_px", "radius", "diameter_px", "diameter", "size_px", "size"]:
            if key in item:
                if "diameter" in key or "size" in key:
                    out["radius_px"] = float(item[key]) / 2.0
                else:
                    out["radius_px"] = float(item[key])
                break
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
    out = {"x_px": float(x), "y_px": float(y)}
    radius = radius_from_model_item(item)
    if radius is not None:
        out["radius_px"] = float(radius)
    return out


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


def copy_image_size(size):
    if size is None:
        return None
    return {"width": int(size.get("width", 0)), "height": int(size.get("height", 0))}


def format_image_size(size):
    if size is None:
        return "(None,None)"
    return "(%s,%s)" % (str(size.get("width", None)), str(size.get("height", None)))


def image_frame_diagnostics():
    return {
        "capture_image_size": copy_image_size(_last_capture_image_size),
        "pipeline_working_image_size": copy_image_size(_last_pipeline_working_image_size),
        "pose_coordinate_frame_size": copy_image_size(_last_pose_image_size),
        "frame_size_source": _last_pose_image_size_source
    }


def update_image_frame_dimensions(pipeline, capture_image, cfg):
    global _last_capture_image_size, _last_pipeline_working_image_size
    global _last_pose_image_size, _last_pose_image_size_source
    _last_capture_image_size = image_size_dict_from_image(capture_image)
    _last_pipeline_working_image_size = None
    try:
        _last_pipeline_working_image_size = image_size_dict_from_mat(pipeline.getWorkingImage())
    except:
        pass
    if _last_pipeline_working_image_size is not None:
        _last_pose_image_size = copy_image_size(_last_pipeline_working_image_size)
        _last_pose_image_size_source = "pipeline_working_image"
    elif bool_value(cfg.get("allow_raw_camera_size_fallback_for_pose_pixels", False)) and _last_capture_image_size is not None:
        _last_pose_image_size = copy_image_size(_last_capture_image_size)
        _last_pose_image_size_source = "capture_image"
    else:
        _last_pose_image_size = None
        _last_pose_image_size_source = None
    if bool_value(cfg.get("log_image_frame_dimensions", True)):
        log("ImageFrame", "capture=%s pipeline=%s pose=%s" %
            (format_image_size(_last_capture_image_size),
             format_image_size(_last_pipeline_working_image_size),
             format_image_size(_last_pose_image_size)))
    if _last_pose_image_size is None and bool_value(cfg.get("require_pipeline_working_image_size_for_pose_pixels", True)):
        raise Exception("Pose image size unavailable; pipeline working image size is required for pose pixels.")


def save_last_vision_image(image):
    global _last_vision_image_path
    if image is None:
        return
    path = last_image_path()
    try:
        ImageIO.write(image, "png", File(path))
        _last_vision_image_path = path
        line = "[VisionImage] saved_last_image=%s" % path
        log("VisionImage", "saved_last_image=%s" % path)
        try:
            if _progress_callback is not None:
                _progress_callback("VisionImage", path, line)
        except:
            pass
    except Exception, e:
        log("VisionImage", "Could not save last capture image: %s" % e)
        log("VisionImage", traceback.format_exc())


def save_pipeline_or_capture_image(pipeline, capture_image, cfg):
    update_image_frame_dimensions(pipeline, capture_image, cfg)
    image = capture_image
    try:
        mat = pipeline.getWorkingImage()
        if mat is not None and not mat.empty():
            image = OpenCvUtils.toBufferedImage(mat)
    except:
        pass
    save_last_vision_image(image)


def save_pose_overlay_json(pose, selected, tag):
    path = last_overlay_path()
    try:
        corners = []
        overlay_corners = []
        for p in pose.get("corners", []):
            corners.append(point_diag_px(p))
            overlay_corners.append(overlay_point_px(p))
        orientation = None
        overlay_orientation = None
        if "orientation_x_px" in pose and "orientation_y_px" in pose:
            orientation = {"x_px": float(pose["orientation_x_px"]),
                           "y_px": float(pose["orientation_y_px"])}
            overlay_orientation = {"x": float(pose["orientation_x_px"]),
                                   "y": float(pose["orientation_y_px"])}
            if "orientation_radius_px" in pose:
                overlay_orientation["radius"] = float(pose["orientation_radius_px"])
        actual_center = {"x": pose.get("center_x_px", None),
                         "y": pose.get("center_y_px", None)}
        overlay = {
            "image_path": last_image_path(),
            "center_source": "four_large_corner_circles_only",
            "orientation_dot_used_for_center": False,
            "orientation_dot_role": "rotation_only",
            "pose_center_px": {"x": pose.get("center_x_px", None),
                               "y": pose.get("center_y_px", None)},
            "selected_corners_px": corners,
            "orientation_px": orientation,
            "actual_center_px": actual_center,
            "expected_center_px": get_processed_image_midpoint_px(),
            "actual_corners_px": overlay_corners,
            "detected_circles_px": overlay_corners,
            "orientation_circle_px": overlay_orientation,
            "machine_xy": {"x": float(selected.getX()), "y": float(selected.getY())},
            "context": tag,
            "timestamp": time.time()
        }
        f = open(path, "w")
        try:
            f.write(json.dumps(overlay, sort_keys=True, indent=2))
        finally:
            f.close()
        line = "[VisionImage] saved_pose_overlay_json=%s" % path
        log("VisionImage", "saved_pose_overlay_json=%s" % path)
        try:
            if _progress_callback is not None:
                _progress_callback("VisionImage", path, line)
        except:
            pass
    except Exception, e:
        log("VisionImage", "Could not save pose overlay json: %s" % e)
        log("VisionImage", traceback.format_exc())


def get_processed_image_midpoint_px():
    if _last_pose_image_size is None:
        return None
    w = float(_last_pose_image_size.get("width", 0))
    h = float(_last_pose_image_size.get("height", 0))
    if w <= 0.0 or h <= 0.0:
        return None
    return {"x": 0.5 * (w - 1.0), "y": 0.5 * (h - 1.0),
            "width": w, "height": h, "source": _last_pose_image_size_source}


def unique_stage_names(names):
    out = []
    for name in names:
        if name is not None and str(name) != "" and str(name) not in out:
            out.append(str(name))
    return out


def extract_stage_points(pipeline, stage_name):
    try:
        model = get_model(pipeline.getResult(stage_name))
    except:
        return []
    out = []
    for item in java_list_to_py(model):
        point = point_from_model_item(item)
        if point is not None:
            out.append(point)
    return out


def point_size_px(point):
    try:
        return float(point["radius_px"])
    except:
        return None


def point_diag_px(point):
    out = {"x_px": point.get("x_px", None), "y_px": point.get("y_px", None)}
    if "radius_px" in point:
        out["radius_px"] = point.get("radius_px", None)
    return out


def overlay_point_px(point):
    out = {"x": point.get("x_px", None), "y": point.get("y_px", None)}
    if "radius_px" in point:
        out["radius"] = point.get("radius_px", None)
    return out


def points_diag_px(points):
    out = []
    for p in points:
        try:
            out.append(point_diag_px(p))
        except:
            out.append(str(p))
    return out


def format_float_list(values):
    return "[" + ", ".join(["%.3f" % float(v) for v in values]) + "]"


def format_point_list(points):
    values = []
    for p in points:
        values.append("(%.3f, %.3f)" % (float(p["x_px"]), float(p["y_px"])))
    return "[" + ", ".join(values) + "]"


def classify_failure(points, reason, diagnostics):
    diagnostics["accepted"] = False
    diagnostics["reason"] = reason
    log("CircleClassify", "accepted=False reason=%s" % reason)
    log("CircleClassify", "raw_points=%s" % str(points_diag_px(points)))
    return {"ok": False, "corner_circles": [], "orientation_points": [],
            "raw_points": points, "reason": reason, "diagnostics": diagnostics}


def classify_calibration_circles(points, cfg, tag):
    if points is None:
        points = []
    raw_count = len(points)
    diagnostics = {"raw_count": raw_count, "accepted": False}
    min_raw_count = int(cfg.get("circle_selection_min_raw_count", 5))
    log("CircleClassify", "raw_count=%d" % raw_count)
    if raw_count < min_raw_count:
        return classify_failure(points, "raw_count %d < circle_selection_min_raw_count %d" %
                                (raw_count, min_raw_count), diagnostics)

    sized = []
    require_size_data = bool_value(cfg.get("circle_selection_require_size_data", True))
    for p in points:
        if "x_px" not in p or "y_px" not in p:
            return classify_failure(points, "point missing x_px/y_px", diagnostics)
        size = point_size_px(p)
        if size is None:
            if require_size_data:
                return classify_failure(points, "point missing radius_px", diagnostics)
            return classify_failure(points, "circle size data unavailable; refusing unordered fallback", diagnostics)
        sized.append((size, p))
    sized = sorted(sized, key=lambda item: item[0])

    orientation_radius, orientation = sized[0]
    min_orient = float(cfg.get("circle_selection_min_orientation_radius_px", 10.0))
    max_orient = float(cfg.get("circle_selection_max_orientation_radius_px", 70.0))
    if orientation_radius < min_orient or orientation_radius > max_orient:
        return classify_failure(points, "orientation radius %.5f outside [%.5f, %.5f]" %
                                (orientation_radius, min_orient, max_orient), diagnostics)

    min_corner = float(cfg.get("circle_selection_min_corner_radius_px", 50.0))
    max_corner = float(cfg.get("circle_selection_max_corner_radius_px", 120.0))
    corner_candidates = []
    for size, p in sized[1:]:
        if size >= min_corner and size <= max_corner:
            corner_candidates.append((size, p))
    if len(corner_candidates) < 4:
        return classify_failure(points, "only %d valid corner-size circles found" %
                                len(corner_candidates), diagnostics)
    corners_with_size = corner_candidates[-4:]
    corners = [item[1] for item in corners_with_size]
    corner_radii = [float(item[0]) for item in corners_with_size]
    avg_corner_radius = mean(corner_radii)
    ratio = orientation_radius / avg_corner_radius if avg_corner_radius > 0.000001 else 999999.0
    max_ratio = float(cfg.get("circle_selection_orientation_max_corner_radius_ratio", 0.80))
    if ratio > max_ratio:
        return classify_failure(points, "orientation/corner radius ratio %.5f > %.5f" %
                                (ratio, max_ratio), diagnostics)

    radius_rms = rms([r - avg_corner_radius for r in corner_radii])
    max_radius_rms = float(cfg.get("circle_selection_corner_radius_rms_max_px", 3.0))
    if radius_rms > max_radius_rms:
        return classify_failure(points, "corner radius RMS %.5f > %.5f" %
                                (radius_rms, max_radius_rms), diagnostics)

    sorted_corners = sort_square_corners_px(corners)
    side_lengths = []
    for i in range(4):
        a = sorted_corners[i]
        b = sorted_corners[(i + 1) % 4]
        dx = float(b["x_px"]) - float(a["x_px"])
        dy = float(b["y_px"]) - float(a["y_px"])
        side_lengths.append(math.sqrt(dx * dx + dy * dy))
    avg_side = mean(side_lengths)
    side_rms = rms([s - avg_side for s in side_lengths])
    max_side_rms = float(cfg.get("circle_selection_corner_square_side_rms_max_px", 5.0))
    if side_rms > max_side_rms:
        return classify_failure(points, "corner square side RMS %.5f > %.5f" %
                                (side_rms, max_side_rms), diagnostics)

    xs = [float(p["x_px"]) for p in sorted_corners]
    ys = [float(p["y_px"]) for p in sorted_corners]
    mean_center = {"x": mean(xs), "y": mean(ys)}
    bbox_center = {"x": (min(xs) + max(xs)) / 2.0, "y": (min(ys) + max(ys)) / 2.0}
    center_delta = math.sqrt((mean_center["x"] - bbox_center["x"]) ** 2 +
                             (mean_center["y"] - bbox_center["y"]) ** 2)
    max_center_delta = float(cfg.get("die_center_max_circle_geometry_rms_px", 3.0))
    if center_delta > max_center_delta:
        return classify_failure(points, "corner mean/bbox center delta %.5f > %.5f" %
                                (center_delta, max_center_delta), diagnostics)

    try:
        orientation["circle_classification_role"] = "orientation"
        for p in sorted_corners:
            p["circle_classification_role"] = "corner"
    except:
        pass

    diagnostics.update({
        "raw_count": raw_count,
        "corner_count": 4,
        "orientation_count": 1,
        "corner_radii_px": corner_radii,
        "orientation_radius_px": float(orientation_radius),
        "avg_corner_radius_px": float(avg_corner_radius),
        "orientation_corner_ratio": float(ratio),
        "corner_radius_rms_px": float(radius_rms),
        "corner_side_lengths_px": side_lengths,
        "corner_side_rms_px": float(side_rms),
        "mean_center_px": mean_center,
        "bbox_center_px": bbox_center,
        "mean_bbox_center_delta_px": float(center_delta),
        "selected_corners": points_diag_px(sorted_corners),
        "corner_circles": points_diag_px(sorted_corners),
        "orientation_point": point_diag_px(orientation),
        "orientation_points": points_diag_px([orientation]),
        "accepted": True,
        "reason": "ok"
    })
    log("CircleClassify", "orientation_radius=%.5f" % float(orientation_radius))
    log("CircleClassify", "corner_radii=%s" % format_float_list(corner_radii))
    log("CircleClassify", "avg_corner_radius=%.5f" % float(avg_corner_radius))
    log("CircleClassify", "orientation_corner_ratio=%.5f" % float(ratio))
    log("CircleClassify", "corner_side_lengths=%s" % format_float_list(side_lengths))
    log("CircleClassify", "corner_side_rms=%.5f" % float(side_rms))
    log("CircleClassify", "selected_corners_px=%s" % format_point_list(sorted_corners))
    log("CircleClassify", "selected_orientation_px=(%.3f, %.3f)" %
        (float(orientation["x_px"]), float(orientation["y_px"])))
    log("CircleClassify", "accepted=True")
    return {"ok": True, "corner_circles": sorted_corners,
            "orientation_points": [orientation], "raw_points": points,
            "reason": "ok", "diagnostics": diagnostics}


def sort_square_corners_px(points):
    cx = mean([p["x_px"] for p in points])
    cy = mean([p["y_px"] for p in points])
    def angle_key(p):
        return math.atan2(float(p["y_px"]) - cy, float(p["x_px"]) - cx)
    return sorted(points, key=angle_key)


def square_fit_pose(circles, cfg=None):
    if len(circles) != 4:
        raise Exception("square_fit_pose requires exactly 4 classified corner circles; got %d" % len(circles))
    for p in circles:
        if str(p.get("circle_classification_role", "corner")) == "orientation":
            raise Exception("square_fit_pose received an orientation circle as a corner.")
        radius = point_size_px(p)
        if cfg is not None and radius is not None:
            min_corner = float(cfg.get("circle_selection_min_corner_radius_px", 50.0))
            max_corner = float(cfg.get("circle_selection_max_corner_radius_px", 120.0))
            if radius < min_corner or radius > max_corner:
                raise Exception("square_fit_pose corner radius %.5f outside [%.5f, %.5f]" %
                                (radius, min_corner, max_corner))
    pts = sort_square_corners_px(circles)
    cx = mean([p["x_px"] for p in pts])
    cy = mean([p["y_px"] for p in pts])
    sides = []
    edge_angles = []
    for i in range(4):
        a = pts[i]
        b = pts[(i + 1) % 4]
        dx = float(b["x_px"]) - float(a["x_px"])
        dy = float(b["y_px"]) - float(a["y_px"])
        sides.append(math.sqrt(dx * dx + dy * dy))
        edge_angles.append(normalize_angle(math.degrees(math.atan2(dy, dx)) - 90.0 * i))
    return {
        "center_x_px": cx,
        "center_y_px": cy,
        "center_source": "four_large_corner_circles_only",
        "orientation_dot_used_for_center": False,
        "rotation_deg": normalize_angle(mean(edge_angles)),
        "side_px": mean(sides),
        "side_rms_px": rms([s - mean(sides) for s in sides]),
        "corners": pts
    }


def classification_points(classification, key):
    if classification is None:
        return []
    try:
        value = classification.get(key, None)
    except:
        value = None
    if value is None:
        return []
    return value


def assert_orientation_not_used_for_center(pose, classification, tag):
    corners = classification_points(classification, "selected_corners")
    if not corners:
        corners = classification_points(classification, "corner_circles")
    orientation = None
    try:
        orientation = classification.get("orientation_point", None)
    except:
        orientation = None
    if orientation is None:
        orientation_points = classification_points(classification, "orientation_points")
        if orientation_points:
            orientation = orientation_points[0]
    if len(corners) != 4 or orientation is None:
        raise Exception("Center invariant cannot be checked; classified corners or orientation dot are missing.")
    corner_center_x = mean([float(p["x_px"]) for p in corners])
    corner_center_y = mean([float(p["y_px"]) for p in corners])
    log("CenterInvariant", "tag=%s" % tag)
    log("CenterInvariant", "center_source=four_large_corner_circles_only")
    log("CenterInvariant", "orientation_dot_used_for_center=False")
    log("CenterInvariant", "corner_center_px=(%.3f, %.3f)" % (corner_center_x, corner_center_y))
    log("CenterInvariant", "orientation_px=(%.3f, %.3f)" %
        (float(orientation["x_px"]), float(orientation["y_px"])))
    if abs(float(pose["center_x_px"]) - corner_center_x) > 0.001 or abs(float(pose["center_y_px"]) - corner_center_y) > 0.001:
        raise Exception("Pose center is not equal to four-corner center; refusing because orientation dot may have contaminated center.")


def select_orientation_point(orientation_points, pose):
    if not orientation_points:
        return None
    if len(orientation_points) == 1:
        return orientation_points[0]
    cx = float(pose["center_x_px"])
    cy = float(pose["center_y_px"])
    best = None
    best_d2 = -1.0
    for p in orientation_points:
        dx = float(p["x_px"]) - cx
        dy = float(p["y_px"]) - cy
        d2 = dx * dx + dy * dy
        if d2 > best_d2:
            best = p
            best_d2 = d2
    return best


def apply_orientation_circle_to_pose(pose, orientation_points, cfg):
    old_center_x = pose["center_x_px"]
    old_center_y = pose["center_y_px"]
    point = select_orientation_point(orientation_points, pose)
    if point is None:
        pose["orientation_found"] = False
        if bool_value(cfg.get("require_orientation_circle", True)):
            raise Exception("Orientation circle required but not detected.")
        if pose["center_x_px"] != old_center_x or pose["center_y_px"] != old_center_y:
            raise Exception("apply_orientation_circle_to_pose modified center; forbidden.")
        return pose
    dx = float(point["x_px"]) - float(pose["center_x_px"])
    dy = float(point["y_px"]) - float(pose["center_y_px"])
    raw = normalize_angle(math.degrees(math.atan2(float(cfg.get("orientation_x_sign", -1.0)) * dx, dy)))
    snapped = nearest_cardinal_angle(raw)
    pose["orientation_found"] = True
    pose["orientation_x_px"] = float(point["x_px"])
    pose["orientation_y_px"] = float(point["y_px"])
    if "radius_px" in point:
        pose["orientation_radius_px"] = float(point["radius_px"])
    pose["rotation_raw_deg"] = raw
    pose["rotation_snapped_deg"] = snapped
    pose["orientation_snap_error_deg"] = rotation_error_deg(raw, snapped)
    pose["rotation_deg"] = raw if not bool_value(cfg.get("orientation_snap_to_cardinal", True)) else snapped
    if pose["center_x_px"] != old_center_x or pose["center_y_px"] != old_center_y:
        raise Exception("apply_orientation_circle_to_pose modified center; forbidden.")
    return pose


def line_intersection_px(a, b, c, d):
    x1 = float(a["x_px"]); y1 = float(a["y_px"])
    x2 = float(b["x_px"]); y2 = float(b["y_px"])
    x3 = float(c["x_px"]); y3 = float(c["y_px"])
    x4 = float(d["x_px"]); y4 = float(d["y_px"])
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 0.000001:
        return None
    return {
        "x": ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den,
        "y": ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
    }


def diagnose_four_circle_square_geometry(circles, pose, cfg, tag):
    px_points = []
    for p in circles:
        if "x_px" in p and "y_px" in p:
            px_points.append(p)
    if len(px_points) != 4:
        return {"available": False, "reason": "pixel centers unavailable"}
    sorted_pts = sort_square_corners_px(px_points)
    pose_center = {"x": float(pose["center_x_px"]), "y": float(pose["center_y_px"])}
    xs = [float(p["x_px"]) for p in sorted_pts]
    ys = [float(p["y_px"]) for p in sorted_pts]
    mean_center = {"x": mean(xs), "y": mean(ys)}
    bbox_center = {"x": (min(xs) + max(xs)) / 2.0, "y": (min(ys) + max(ys)) / 2.0}
    diag_center = line_intersection_px(sorted_pts[0], sorted_pts[2], sorted_pts[1], sorted_pts[3])
    centers = [mean_center, bbox_center]
    residuals = []
    for c in centers:
        residuals.append(math.sqrt((float(c["x"]) - pose_center["x"]) ** 2 + (float(c["y"]) - pose_center["y"]) ** 2))
    center_rms = rms(residuals)
    log("CircleGeometry", "pose_center_px=(%.3f, %.3f) center_rms=%.5f" %
        (pose_center["x"], pose_center["y"], center_rms))
    if bool_value(cfg.get("die_center_crosscheck_enabled", True)) and center_rms > float(cfg.get("die_center_max_circle_geometry_rms_px", 3.0)):
        raise Exception("Four-circle geometry RMS %.5f px exceeds limit." % center_rms)
    out = {"available": True, "pose_center_px": pose_center,
           "mean_center_px": mean_center, "bbox_center_px": bbox_center,
           "diagonal_intersection_px": diag_center, "center_consistency_rms_px": center_rms}
    pose["circle_geometry_diagnostic"] = out
    return out


def run_fiducial_pipeline_and_extract_pose_points(camera, nozzle, part, cfg):
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
    stages = unique_stage_names([cfg.get("circle_result_stage_name", "results"), "results", "circles", "fiducials", "preResults"])
    best = []
    best_orientation = []
    best_stage = None
    best_classification = None
    hard_failure_reason = None
    for attempt in range(1, PIPELINE_ATTEMPTS + 1):
        check_cancel("Vision")
        try:
            capture = None
            try:
                capture = camera.settleAndCapture()
            except:
                sleep_ms(SETTLE_MS)
            pipeline.process()
            save_pipeline_or_capture_image(pipeline, capture, cfg)
            for stage in stages:
                raw = extract_stage_points(pipeline, stage)
                classified = classify_calibration_circles(raw, cfg, "Vision")
                if not classified.get("ok", False):
                    if len(raw) >= int(cfg.get("circle_selection_min_raw_count", 5)):
                        hard_failure_reason = classified.get("reason", "classification failed")
                    log("Vision", "Stage %s circle classification rejected: %s" %
                        (stage, classified.get("reason", "unknown")))
                    continue
                circles = classified["corner_circles"]
                orient = classified["orientation_points"]
                if len(circles) > len(best):
                    best = circles
                    best_stage = stage
                    best_classification = classified.get("diagnostics", None)
                if len(orient) > len(best_orientation):
                    best_orientation = orient
                if len(circles) == 4 and len(orient) == 1:
                    return {"circles": circles, "orientation_points": orient,
                            "circle_stage_name": stage,
                            "circle_classification": classified.get("diagnostics", None)}
        except Exception, e:
            log("Vision", "Pipeline attempt %d failed: %s" % (attempt, e))
        sleep_ms(SETTLE_MS)
    if hard_failure_reason is not None:
        raise Exception("Circle classification failed; refusing to compute die center from first four raw points. %s" %
                        hard_failure_reason)
    return {"circles": best, "orientation_points": best_orientation,
            "circle_stage_name": best_stage, "circle_classification": best_classification}


def spiral_offsets(spiral):
    yield (0.0, 0.0)
    if not bool_value(spiral.get("enabled", True)):
        return
    r = float(spiral["start_radius_mm"])
    while r <= float(spiral["max_radius_mm"]) + 0.0000001:
        a = 0.0
        while a < 360.0:
            yield (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
            a += float(spiral["angle_step_deg"])
        r += float(spiral["radius_step_mm"])


def camera_measurement_loc(camera, x, y, z):
    cam = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
    return mm_loc(float(x), float(y), float(z), cam.getRotation())


def assert_camera_at_requested_z(camera, requested_z, cfg, tag):
    loc = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
    dz = float(loc.getZ()) - float(requested_z)
    log(tag, "camera_z_check requested_z=%.5f actual_z=%.5f dz=%.5f" %
        (float(requested_z), loc.getZ(), dz))
    if bool_value(cfg.get("pixel_transform_require_camera_at_requested_z", True)):
        if abs(dz) > float(cfg.get("pixel_transform_max_camera_z_error_mm", 0.01)):
            raise Exception("Camera is not at requested measurement Z before pixel conversion.")


def pixel_locations_for_pose(camera, nozzle, pose, fallback_loc, cfg):
    if "center_x_px" not in pose or "center_y_px" not in pose:
        raise Exception("Detected center has no pixel coordinates.")
    px = float(pose["center_x_px"])
    py = float(pose["center_y_px"])
    log("PixelTransform", "center_px=(%.3f, %.3f)" % (px, py))
    camera_only = VisionUtils.getPixelLocation(camera, px, py).convertToUnits(LengthUnit.Millimeters)
    tool_aware = VisionUtils.getPixelLocation(camera, nozzle, px, py).convertToUnits(LengthUnit.Millimeters)
    dx = tool_aware.getX() - camera_only.getX()
    dy = tool_aware.getY() - camera_only.getY()
    mag = math.sqrt(dx * dx + dy * dy)
    log("PixelTransform", "camera_only_measurement=(%.5f, %.5f)" %
        (camera_only.getX(), camera_only.getY()))
    log("PixelTransform", "nozzle_target_tool_aware=(%.5f, %.5f)" %
        (tool_aware.getX(), tool_aware.getY()))
    log("PixelTransform", "tool_minus_camera_only=(%.5f, %.5f, %.5f)" % (dx, dy, mag))
    log("PixelTransform", "measurement_method=camera_only")
    log("PixelTransform", "nozzle_target_method=tool_aware")
    fallback = fallback_loc.convertToUnits(LengthUnit.Millimeters)
    measurement_loc = mm_loc(camera_only.getX(), camera_only.getY(), fallback.getZ(), fallback.getRotation())
    nozzle_target_loc = mm_loc(tool_aware.getX(), tool_aware.getY(), fallback.getZ(), fallback.getRotation())
    return {
        "measurement_location": measurement_loc,
        "nozzle_target_location": nozzle_target_loc,
        "measurement_coordinate_frame": "camera_only",
        "nozzle_target_coordinate_frame": "tool_aware",
        "camera_only": camera_only,
        "tool_aware": tool_aware,
        "camera_only_location": measurement_loc,
        "tool_aware_location": nozzle_target_loc,
        "tool_minus_camera_only": {"dx": dx, "dy": dy, "mag": mag},
        "tool_minus_camera_only_mag_mm": mag
    }


def assert_nozzle_motion_uses_tool_aware(lock, tag):
    frame = lock.get("nozzle_target_coordinate_frame", None)
    if frame != "tool_aware":
        raise Exception("%s nozzle motion target must be tool_aware, not camera_only." % tag)


def assert_measurement_uses_camera_only(lock, tag):
    frame = lock.get("measurement_coordinate_frame", None)
    if frame != "camera_only":
        raise Exception("%s measurement must be camera_only." % tag)


def diagnose_pixel_transform(camera, nozzle, pose, fallback_loc, requested_z, cfg, tag):
    if not bool_value(cfg.get("validate_pixel_to_machine_transform", True)):
        return {}
    assert_camera_at_requested_z(camera, requested_z, cfg, "PixelTransform")
    if "center_x_px" not in pose or "center_y_px" not in pose:
        return {"available": False, "reason": "no pose center px"}
    px = float(pose["center_x_px"])
    py = float(pose["center_y_px"])
    camera_only = VisionUtils.getPixelLocation(camera, px, py).convertToUnits(LengthUnit.Millimeters)
    tool_aware = VisionUtils.getPixelLocation(camera, nozzle, px, py).convertToUnits(LengthUnit.Millimeters)
    dx = tool_aware.getX() - camera_only.getX()
    dy = tool_aware.getY() - camera_only.getY()
    mag = math.sqrt(dx * dx + dy * dy)
    if bool_value(cfg.get("pixel_transform_compare_camera_only_and_tool_aware", True)):
        log("PixelTransform", "camera_only_measurement=(%.5f, %.5f) nozzle_target_tool_aware=(%.5f, %.5f) diff=(%.5f, %.5f, %.5f)" %
            (camera_only.getX(), camera_only.getY(), tool_aware.getX(), tool_aware.getY(), dx, dy, mag))
        log("PixelTransform", "measurement_method=camera_only")
        log("PixelTransform", "nozzle_target_method=tool_aware")
        log("PixelTransform", "tool_minus_camera_only=(%.5f, %.5f, %.5f)" % (dx, dy, mag))
    out = {"available": True, "center_px": {"x": px, "y": py},
           "camera_only": location_to_diag(camera_only),
           "tool_aware": location_to_diag(tool_aware),
           "measurement_method": "camera_only",
           "nozzle_target_method": "tool_aware",
           "tool_minus_camera_only": {"dx": dx, "dy": dy, "mag": mag},
           "tool_minus_camera_only_mag_mm": mag,
           "image_frame_diagnostics": image_frame_diagnostics()}
    pose["pixel_transform_diagnostic"] = out
    return out


def detect_with_spiral(camera, nozzle, part, cfg, expected_camera_loc):
    origin = expected_camera_loc.convertToUnits(LengthUnit.Millimeters)
    best_count = 0
    for dx, dy in spiral_offsets(cfg["spiral_search"]):
        target = camera_measurement_loc(camera, origin.getX() + dx, origin.getY() + dy, origin.getZ())
        if abs(dx) > 0.00001 or abs(dy) > 0.00001:
            log("Spiral", "Search offset dx=%.4f dy=%.4f" % (dx, dy))
        move_tool_safe(camera, target, "VisionLock")
        assert_camera_at_requested_z(camera, target.getZ(), cfg, "VisionLock")
        sleep_ms(cfg["spiral_search"].get("settle_ms", SETTLE_MS))
        detected = run_fiducial_pipeline_and_extract_pose_points(camera, nozzle, part, cfg)
        circles = detected.get("circles", [])
        if len(circles) > best_count:
            best_count = len(circles)
        if len(circles) == 4 and len(detected.get("orientation_points", [])) == 1:
            pose = square_fit_pose(circles, cfg)
            apply_orientation_circle_to_pose(pose, detected.get("orientation_points", []), cfg)
            assert_orientation_not_used_for_center(pose, detected.get("circle_classification", None), "Vision")
            pose["circle_count"] = len(circles)
            pose["circle_stage_name"] = detected.get("circle_stage_name", None)
            pose["circle_classification"] = detected.get("circle_classification", None)
            pose["search_dx_mm"] = dx
            pose["search_dy_mm"] = dy
            diagnose_four_circle_square_geometry(circles, pose, cfg, "Vision")
            return pose
    raise Exception("Could not detect 4 circles. Best count: %d" % best_count)


def acquire_die_pose_at_location(camera, nozzle, part, base_loc, camera_z, cfg, tag):
    target = camera_measurement_loc(camera, base_loc.getX(), base_loc.getY(), float(camera_z))
    log("VisionLock", "[%s] acquiring X=%.4f Y=%.4f Z=%.4f" %
        (tag, target.getX(), target.getY(), target.getZ()))
    pose = detect_with_spiral(camera, nozzle, part, cfg, target)
    diagnose_pixel_transform(camera, nozzle, pose, base_loc, camera_z, cfg, tag)
    pixel_locs = pixel_locations_for_pose(camera, nozzle, pose, base_loc, cfg)
    measurement_loc = pixel_locs["measurement_location"]
    nozzle_target_loc = pixel_locs["nozzle_target_location"]
    pose["centered_machine_x_mm"] = measurement_loc.getX()
    pose["centered_machine_y_mm"] = measurement_loc.getY()
    pose["measured_center_x_mm"] = measurement_loc.getX()
    pose["measured_center_y_mm"] = measurement_loc.getY()
    pose["nozzle_target_x_mm"] = nozzle_target_loc.getX()
    pose["nozzle_target_y_mm"] = nozzle_target_loc.getY()
    pose["measurement_coordinate_frame"] = "camera_only"
    pose["nozzle_target_coordinate_frame"] = "tool_aware"
    pose["requested_camera_z"] = float(camera_z)
    pose["image_frame_diagnostics"] = image_frame_diagnostics()
    save_pose_overlay_json(pose, measurement_loc, tag)
    return {"pose": pose,
            "measurement_location": measurement_loc,
            "nozzle_target_location": nozzle_target_loc,
            "centered_location": measurement_loc,
            "measurement_coordinate_frame": "camera_only",
            "nozzle_target_coordinate_frame": "tool_aware",
            "camera_only": pixel_locs["camera_only"],
            "tool_aware": pixel_locs["tool_aware"],
            "tool_minus_camera_only": pixel_locs["tool_minus_camera_only"]}


def acquire_pose_for_location(camera, nozzle, part, expected_loc, cfg, tag):
    expected_loc = expected_loc.convertToUnits(LengthUnit.Millimeters)
    z = camera_z_for_location(expected_loc, cfg)
    base = camera_measurement_loc(camera, expected_loc.getX(), expected_loc.getY(), z)
    lock = acquire_die_pose_at_location(camera, nozzle, part, base, z, cfg, tag)
    pose = lock["pose"]
    r = normalize_angle(float(expected_loc.getRotation()))
    if bool_value(cfg.get("align_nozzle_to_detected_die_rotation", True)) and bool_value(pose.get("orientation_found", False)):
        r = normalize_angle(float(pose.get("rotation_raw_deg", pose.get("rotation_deg", r))))
    measurement = lock["measurement_location"]
    nozzle_target = lock["nozzle_target_location"]
    measurement_loc = mm_loc(measurement.getX(), measurement.getY(), expected_loc.getZ(), r)
    nozzle_target_loc = mm_loc(nozzle_target.getX(), nozzle_target.getY(), expected_loc.getZ(), r)
    lock["measurement_location"] = measurement_loc
    lock["nozzle_target_location"] = nozzle_target_loc
    lock["centered_location"] = measurement_loc
    assert_measurement_uses_camera_only(lock, "%s vision lock" % tag)
    return lock, measurement_loc


def should_validate_die_orientation_for_motion(cfg):
    mode = str(cfg.get("calibration_result_mode", "round_object_nozzle_axis")).lower()
    if mode == "round_object_nozzle_axis":
        return bool_value(cfg.get("round_object_validate_die_orientation", False))
    return bool_value(cfg.get("require_orientation_circle", True))


def validate_pose_orientation(pose, expected_loc, cfg, tag):
    if str(cfg.get("calibration_result_mode", "")).lower() == "round_object_nozzle_axis":
        if not bool_value(cfg.get("round_object_validate_die_orientation", False)):
            log("Orientation", "[%s] orientation angle validation skipped for round_object_nozzle_axis." % tag)
            return
    if not bool_value(cfg.get("require_orientation_circle", True)):
        return
    if not bool_value(pose.get("orientation_found", False)):
        raise Exception("[Orientation][%s] required orientation circle missing." % tag)
    expected = normalize_angle(expected_loc.getRotation())
    actual = normalize_angle(float(pose.get("rotation_snapped_deg", pose.get("rotation_deg", 0.0))))
    err = rotation_error_deg(actual, expected)
    if abs(err) > float(cfg.get("orientation_max_error_deg", 25.0)):
        raise Exception("[Orientation][%s] expected %.4f actual %.4f error %.4f." % (tag, expected, actual, err))


# ---------------------------------------------------------------------------
# pick / place

def find_actuator_by_name(machine_or_head, actuator_name):
    if actuator_name is None or str(actuator_name).strip() == "":
        return None
    for obj in [machine_or_head, get_machine()]:
        if obj is None:
            continue
        for method_name in ["getActuatorByName", "getActuator"]:
            try:
                actuator = getattr(obj, method_name)(actuator_name)
                if actuator is not None:
                    return actuator
            except:
                pass
        try:
            for actuator in java_list_to_py(obj.getActuators()):
                if object_name(actuator) == str(actuator_name):
                    return actuator
        except:
            pass
    raise Exception("Actuator not found by name: %s" % actuator_name)


def resolve_assigned_actuator(nozzle, cfg, kind):
    cfg_key = "force_vacuum_actuator_name" if kind == "vacuum" else "force_blowoff_actuator_name"
    configured = cfg.get(cfg_key, "")
    if str(configured).strip() != "":
        try:
            return find_actuator_by_name(nozzle.getHead(), configured)
        except:
            return find_actuator_by_name(get_machine(), configured)
    method_names = ["getVacuumActuator", "getVacuumValveActuator"] if kind == "vacuum" else ["getBlowOffActuator", "getBlowoffActuator"]
    for owner in [nozzle, nozzle.getHead()]:
        for name in method_names:
            try:
                actuator = getattr(owner, name)()
                if actuator is not None:
                    return actuator
            except:
                pass
    return None


def set_actuator_state(actuator, on_off):
    if actuator is None:
        return False
    if hasattr(actuator, "actuate"):
        actuator.actuate(bool(on_off))
    elif hasattr(actuator, "setActuated"):
        actuator.setActuated(bool(on_off))
    else:
        raise Exception("Actuator has no supported state method: %s" % object_name(actuator))
    return True


def set_vacuum_state(nozzle, cfg, on_off, fatal):
    actuator = resolve_assigned_actuator(nozzle, cfg, "vacuum")
    if actuator is None:
        if fatal:
            raise Exception("Vacuum actuator is not configured and could not be resolved.")
        return False
    set_actuator_state(actuator, on_off)
    log("Vacuum", "%s -> %s" % (object_name(actuator), "ON" if bool(on_off) else "OFF"))
    return True


def set_blowoff_state(nozzle, cfg, on_off):
    actuator = resolve_assigned_actuator(nozzle, cfg, "blowoff")
    if actuator is None:
        return False
    set_actuator_state(actuator, on_off)
    log("Blowoff", "%s -> %s" % (object_name(actuator), "ON" if bool(on_off) else "OFF"))
    return True


def do_pick(nozzle, part, cfg):
    set_vacuum_state(nozzle, cfg, True, True)
    sleep_ms(cfg.get("vacuum_settle_ms", 150))
    if bool_value(cfg.get("use_high_level_nozzle_pick", True)):
        try:
            nozzle.pick(part, None)
            log("Pick", "nozzle.pick(part, None) succeeded")
            return
        except Throwable, t:
            log("Pick", "nozzle.pick failed; continuing with vacuum and part state fallback: %s" % t)
    try:
        nozzle.setPart(part)
    except:
        pass


def do_place(nozzle, cfg):
    set_vacuum_state(nozzle, cfg, False, False)
    if bool_value(cfg.get("enable_blowoff_on_place", False)):
        if set_blowoff_state(nozzle, cfg, True):
            sleep_ms(cfg.get("blowoff_pulse_ms", 120))
            set_blowoff_state(nozzle, cfg, False)
    try:
        nozzle.setPart(None)
    except:
        pass


def verify_part_on_after_pick(nozzle):
    try:
        if nozzle.isPartOnEnabled(SpiNozzle.PartOnStep.AfterPick) and not nozzle.isPartOn():
            raise Exception("Part-on sensor did not detect a part after pick.")
    except AttributeError:
        pass


def verify_part_off_after_place(nozzle):
    try:
        if nozzle.isPartOffEnabled(SpiNozzle.PartOffStep.AfterPlace) and not nozzle.isPartOff():
            raise Exception("Part-off sensor did not verify release after place.")
    except AttributeError:
        pass


def pose_rotation_for_pick(pose, cfg):
    if bool_value(pose.get("orientation_found", False)):
        return normalize_angle(float(pose.get("rotation_raw_deg", pose.get("rotation_deg", 0.0))))
    return normalize_angle(float(pose.get("rotation_deg", 0.0)))


def pick_die_with_visual_lock(camera, nozzle, part, base_loc, cfg, tag, state):
    lock, detected_measurement_loc = acquire_pose_for_location(camera, nozzle, part, base_loc, cfg, tag)
    assert_nozzle_motion_uses_tool_aware(lock, "%s pick" % tag)
    pose = lock["pose"]
    detected_nozzle_target_loc = lock["nozzle_target_location"]
    if bool_value(cfg.get("require_visual_lock_before_pick", True)) and int(pose.get("circle_count", 0)) < 4:
        raise Exception("[VisionLock][%s] pick aborted: visual lock count < 4." % tag)
    if tag != "Storage":
        validate_pose_orientation(pose, base_loc, cfg, "%s pre-pick" % tag)
    pick_r = base_loc.getRotation()
    align_to_detected = bool_value(cfg.get("align_nozzle_to_detected_die_rotation", True))
    detected_r = pose_rotation_for_pick(pose, cfg)
    if align_to_detected:
        pick_r = detected_r
    log("Pick", "align_nozzle_to_detected_die_rotation=%s" % str(bool(align_to_detected)))
    log("Pick", "detected_die_rotation_raw=%.4f" % float(detected_r))
    log("Pick", "commanded_nozzle_rotation=%.4f" % float(pick_r))
    if cfg.get("_pick_rotation_source", None) is not None:
        log("Pick", "source=%s" % str(cfg.get("_pick_rotation_source", None)))
    pick_target = mm_loc(detected_nozzle_target_loc.getX(), detected_nozzle_target_loc.getY(), base_loc.getZ(), pick_r)
    pick_z = pick_surface_z_mm(base_loc, cfg)
    log("Pick", "%s detected_measurement_center=(%.5f, %.5f) configured=(%.5f, %.5f)" %
        (tag, detected_measurement_loc.getX(), detected_measurement_loc.getY(), base_loc.getX(), base_loc.getY()))
    log("Pick", "%s nozzle_pick_target=(%.5f, %.5f)" %
        (tag, detected_nozzle_target_loc.getX(), detected_nozzle_target_loc.getY()))
    log("Pick", "measurement_frame=%s" % str(lock.get("measurement_coordinate_frame", None)))
    log("Pick", "nozzle_target_frame=%s" % str(lock.get("nozzle_target_coordinate_frame", None)))
    move_nozzle_split(nozzle, nearest_nozzle_rotation_location(nozzle, mm_loc(pick_target.getX(), pick_target.getY(), pick_z, pick_r), "Pick", cfg), cfg, "Pick", tag)
    do_pick(nozzle, part, cfg)
    verify_part_on_after_pick(nozzle)
    state["die_on_nozzle"] = True
    if tag == "Cal":
        state["die_at_work"] = False
        state["current_die_loc_trusted"] = False
    actual = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    state["last_pick_rotation_deg"] = normalize_angle(actual.getRotation())
    lift_nozzle_after_pick_place(nozzle, cfg, "Pick", tag, pick_z)
    return detected_measurement_loc


def place_die_at(nozzle, part, target_loc, cfg, tag, state):
    target_loc = target_loc.convertToUnits(LengthUnit.Millimeters)
    place_z = pick_surface_z_mm(target_loc, cfg)
    place_target = nearest_nozzle_rotation_location(nozzle, mm_loc(target_loc.getX(), target_loc.getY(), place_z, target_loc.getRotation()), "Place", cfg)
    log("Place", "%s command=(%.5f, %.5f, %.5f)" %
        (tag, place_target.getX(), place_target.getY(), place_target.getRotation()))
    move_nozzle_split(nozzle, place_target, cfg, "Place", tag)
    do_place(nozzle, cfg)
    sleep_ms(cfg.get("blowoff_pulse_ms", 120))
    verify_part_off_after_place(nozzle)
    state["die_on_nozzle"] = False
    state["die_at_work"] = (tag == "Cal")
    if tag == "Cal":
        state["current_die_loc"] = target_loc
        state["current_die_loc_trusted"] = False
    lift_nozzle_after_pick_place(nozzle, cfg, "Place", tag, place_z)
    return target_loc


def transfer_die_from_storage_to_cal(camera, nozzle, part, storage_loc, cal_loc, cfg, state):
    pick_cfg = dict(cfg)
    detected_storage = pick_die_with_visual_lock(camera, nozzle, part, storage_loc, pick_cfg, "Storage", state)
    dx = detected_storage.getX() - storage_loc.getX()
    dy = detected_storage.getY() - storage_loc.getY()
    log("Storage", "initial detected_measurement_minus_configured=(%.5f, %.5f); not used for calibration correction" % (dx, dy))
    place_die_at(nozzle, part, cal_loc, cfg, "Cal", state)
    lock, actual = acquire_pose_for_location(camera, nozzle, part, cal_loc, cfg, "Cal")
    assert_measurement_uses_camera_only(lock, "initial work pose")
    if should_validate_die_orientation_for_motion(cfg):
        validate_pose_orientation(lock["pose"], cal_loc, cfg, "initial work pose")
    else:
        log("Orientation", "Skipping initial work pose orientation validation in round_object_nozzle_axis mode; orientation dot is classification-only.")
    state["current_die_loc"] = actual
    state["current_die_loc_trusted"] = True
    state["last_trusted_die_loc"] = actual
    return detected_storage


def return_die_to_storage(camera, nozzle, part, storage_loc, cfg, state):
    current = state.get("current_die_loc", None)
    if current is None or not bool_value(state.get("current_die_loc_trusted", False)):
        raise Exception("Cannot return object to storage: current work pose is not trusted.")
    preview_delta = state.get("round_axis_same_run_preview_delta", None)
    high_residual_preview = bool_value(state.get("round_axis_same_run_preview_delta_high_residual", False))
    use_preview_delta = preview_delta is not None
    if use_preview_delta:
        if high_residual_preview:
            log("StorageReturn", "HIGH RESIDUAL DIAGNOSTIC RETURN PREVIEW ONLY")
            log("StorageReturn", "using computed residual static delta despite high fit residuals")
        log("StorageReturn", "using residual same-run preview delta on top of active OpenPnP compensation")
        log("StorageReturn", "Returning object to configured storage pocket with residual same-run preview correction.")
    else:
        log("StorageReturn", "Returning object to configured storage pocket with no correction.")
    log("StorageReturn", "pick_correction_used=%s" % str(bool(use_preview_delta)))
    pick_cfg = dict(cfg)
    pick_cfg["align_nozzle_to_detected_die_rotation"] = bool_value(cfg.get("align_nozzle_to_detected_die_rotation", True))
    pick_die_with_visual_lock(camera, nozzle, part, current, pick_cfg, "Cal", state)
    dx = 0.0
    dy = 0.0
    if use_preview_delta:
        dx = float(preview_delta.get("dx", 0.0))
        dy = float(preview_delta.get("dy", 0.0))
    storage_target = mm_loc(storage_loc.getX() + dx, storage_loc.getY() + dy,
                            storage_loc.getZ(), storage_loc.getRotation())
    place_die_at(nozzle, part, storage_target, cfg, "Storage", state)
    state["current_die_loc_trusted"] = False
    out = {"status": "placed", "target": location_to_diag(storage_target),
           "correction_used": bool(use_preview_delta),
           "diagnostic_preview_used": bool(high_residual_preview and use_preview_delta),
           "correction_x_mm": dx, "correction_y_mm": dy,
           "reason": "residual same-run preview delta on top of active OpenPnP compensation" if use_preview_delta else "configured storage pocket; no correction"}
    log("StorageReturn", "final storage placement complete; no refind/retry/pick will be attempted")
    return out


# ---------------------------------------------------------------------------
# round-axis calibration

def solve_linear_system(a, b):
    n = len(b)
    m = []
    for i in range(n):
        row = []
        for j in range(n):
            row.append(float(a[i][j]))
        row.append(float(b[i]))
        m.append(row)
    for col in range(n):
        pivot = col
        pivot_abs = abs(m[col][col])
        for row in range(col + 1, n):
            value = abs(m[row][col])
            if value > pivot_abs:
                pivot = row
                pivot_abs = value
        if pivot_abs < 0.000000000001:
            raise Exception("Fit matrix is singular.")
        if pivot != col:
            tmp = m[col]; m[col] = m[pivot]; m[pivot] = tmp
        scale = m[col][col]
        for j in range(col, n + 1):
            m[col][j] = m[col][j] / scale
        for row in range(n):
            if row == col:
                continue
            factor = m[row][col]
            for j in range(col, n + 1):
                m[row][j] = m[row][j] - factor * m[col][j]
    out = []
    for i in range(n):
        out.append(m[i][n])
    return out


def round_axis_place_angle(command_angle, sample_index, cfg):
    mode = str(cfg.get("round_object_place_rotation_mode", "sample_angle")).lower()
    if mode == "same_as_pick_angle":
        log("Config", "Treating legacy round_object_place_rotation_mode=same_as_pick_angle as sample_angle.")
        mode = "sample_angle"
    if mode == "zero":
        return 0.0
    if mode == "paired_opposite" and int(sample_index) % 2 == 1:
        angle = normalize_angle(float(command_angle) + 180.0)
    else:
        angle = normalize_angle(command_angle)
    if bool_value(cfg.get("avoid_exact_180_deg", True)) and abs(abs(angle) - 180.0) < 0.000001:
        if angle >= 0.0:
            return 179.0
        return -179.0
    return angle


def round_axis_predicted_error(fit, angle_deg):
    rv = rotate_xy(float(fit.get("residual_angle_vector_x_mm", fit.get("runout_vector_x_mm", 0.0))),
                   float(fit.get("residual_angle_vector_y_mm", fit.get("runout_vector_y_mm", 0.0))), angle_deg)
    return {"x": float(fit.get("residual_static_bias_x_mm", fit.get("bias_x_mm", 0.0))) + rv["x"],
            "y": float(fit.get("residual_static_bias_y_mm", fit.get("bias_y_mm", 0.0))) + rv["y"]}


def determine_round_object_pick_rotation(nozzle, current_die_loc, state, cfg):
    mode = str(cfg.get("round_object_pick_rotation_mode", "current_die_rotation")).lower()
    if mode == "fixed":
        return normalize_angle(float(cfg.get("round_object_pick_rotation_deg", 0.0)))
    if mode == "current_nozzle_rotation":
        return float(nozzle.getLocation().convertToUnits(LengthUnit.Millimeters).getRotation())
    if mode == "previous_place_rotation":
        return normalize_angle(float(state.get("last_round_axis_place_rotation_deg", current_die_loc.getRotation())))
    if mode == "current_die_rotation":
        return normalize_angle(float(current_die_loc.getRotation()))
    raise Exception("Invalid round_object_pick_rotation_mode: %s" % mode)


def measure_round_axis_sample(camera, nozzle, part, cal_loc, cfg, state, axis_cfg,
                              current_die_loc, cycle_index, sample_index, angle):
    requested_sample_angle = float(angle)
    command_angle = requested_sample_angle
    normalized_sample_angle = normalize_angle(command_angle)
    place_angle = round_axis_place_angle(command_angle, sample_index, cfg)
    nominal = mm_loc(cal_loc.getX(), cal_loc.getY(), cal_loc.getZ(), place_angle)
    log("RoundAxis", "sample cycle=%d index=%d angle=%.3f place_angle=%.3f" %
        (cycle_index + 1, sample_index + 1, command_angle, place_angle))
    log("RoundAxis", "previous die rotation=%.3f" % float(current_die_loc.getRotation()))
    log("RoundAxis", "phase=measure_before_pick")
    pre_lock, pre_loc = acquire_pose_for_location(camera, nozzle, part, current_die_loc, axis_cfg, "Cal")
    assert_measurement_uses_camera_only(pre_lock, "RoundAxis pre-pick")
    pick_rotation_mode = str(cfg.get("round_object_pick_rotation_mode", "current_die_rotation")).lower()
    pick_rotation = determine_round_object_pick_rotation(nozzle, current_die_loc, state, cfg)
    axis_cfg["_pick_rotation_source"] = "round_object_pick_rotation_mode:%s" % pick_rotation_mode
    log("RoundAxis", "phase=pick_object pick_rotation=%.3f pick_rotation_mode=%s" %
        (float(pick_rotation), pick_rotation_mode))
    pick_base = mm_loc(pre_loc.getX(), pre_loc.getY(), pre_loc.getZ(), pick_rotation)
    pick_die_with_visual_lock(camera, nozzle, part, pick_base, axis_cfg, "Cal", state)
    if not bool_value(state.get("die_on_nozzle", False)):
        raise Exception("Round-axis sample cannot rotate: object is not on nozzle after pick.")
    picked_r = float(state.get("last_pick_rotation_deg", pick_rotation))
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    command_r = nearest_equivalent_rotation(command_angle, current.getRotation())
    rotation_delta = check_nozzle_rotation_step_for_context(command_r, current.getRotation(), cfg, "RoundAxisRotateWithObject")
    log("RoundAxis", "phase=rotate_with_object from_r=%.3f to_r=%.3f delta_r=%.3f" %
        (float(current.getRotation()), float(command_r), float(rotation_delta)))
    move_to_with_speed(nozzle, mm_loc(current.getX(), current.getY(), current.getZ(), command_r), cfg)
    log("RoundAxis", "phase=place_object place_rotation=%.3f" % float(place_angle))
    place_die_at(nozzle, part, nominal, axis_cfg, "Cal", state)
    log("RoundAxis", "phase=measure_after_place")
    lock, actual_measurement_loc = acquire_pose_for_location(camera, nozzle, part, nominal, axis_cfg, "Cal")
    assert_measurement_uses_camera_only(lock, "RoundAxis sample error")
    pose = lock["pose"]
    error_x = actual_measurement_loc.getX() - nominal.getX()
    error_y = actual_measurement_loc.getY() - nominal.getY()
    error_mag = math.sqrt(error_x * error_x + error_y * error_y)
    sample = {
        "label": "round_object_nozzle_axis",
        "sample_index": int(sample_index),
        "cycle_index": int(cycle_index),
        "requested_sample_angle_deg": requested_sample_angle,
        "commanded_nozzle_rotation_deg": float(command_r),
        "actual_rotation_delta_deg": float(rotation_delta),
        "normalized_sample_angle_deg": normalized_sample_angle,
        "command_angle_deg": command_angle,
        "pick_rotation_deg": normalize_angle(picked_r),
        "place_rotation_deg": normalize_angle(place_angle),
        "nominal_place_center_x_mm": float(nominal.getX()),
        "nominal_place_center_y_mm": float(nominal.getY()),
        "measured_after_place_center_x_mm": float(actual_measurement_loc.getX()),
        "measured_after_place_center_y_mm": float(actual_measurement_loc.getY()),
        "measurement_coordinate_frame": lock.get("measurement_coordinate_frame", None),
        "nozzle_target_coordinate_frame": lock.get("nozzle_target_coordinate_frame", None),
        "error_x_mm": error_x,
        "error_y_mm": error_y,
        "error_mag_mm": error_mag,
        "sample_valid": True,
        "sample_rejected_reason": None,
        "center_px": pose.get("center_px", {"x": pose.get("center_x_px", None), "y": pose.get("center_y_px", None)}),
        "image_frame_diagnostics": pose.get("image_frame_diagnostics", image_frame_diagnostics()),
        "selected_pixel_to_machine_transform_method": pose.get("measurement_coordinate_frame", None),
        "pixel_transform_diagnostic": pose.get("pixel_transform_diagnostic", None),
        "circle_classification": pose.get("circle_classification", None),
        "circle_geometry_diagnostic": pose.get("circle_geometry_diagnostic", None)
    }
    trusted_loc = mm_loc(actual_measurement_loc.getX(), actual_measurement_loc.getY(), actual_measurement_loc.getZ(), place_angle)
    state["current_die_loc"] = trusted_loc
    state["current_die_loc_trusted"] = True
    state["last_trusted_die_loc"] = trusted_loc
    state["last_round_axis_place_rotation_deg"] = place_angle
    log("RoundAxis", "measured_center=(%.5f, %.5f) error=(%.5f, %.5f)" %
        (actual_measurement_loc.getX(), actual_measurement_loc.getY(), error_x, error_y))
    log("RoundAxis", "sample angle=%.3f measured_center=(%.5f, %.5f) error=(%.5f, %.5f)" %
        (command_angle, actual_measurement_loc.getX(), actual_measurement_loc.getY(), error_x, error_y))
    return sample, trusted_loc


def check_round_axis_sample(sample, cfg):
    missing = []
    for key in ["error_x_mm", "error_y_mm", "error_mag_mm", "command_angle_deg"]:
        if key not in sample:
            missing.append(key)
    if missing:
        sample["sample_valid"] = False
        sample["sample_rejected_reason"] = "missing sample fields: %s" % ", ".join(missing)
        log("RoundAxisSampleCheck", "invalid sample: %s" % sample["sample_rejected_reason"])
        return False
    error_x = float(sample.get("error_x_mm", 0.0))
    error_y = float(sample.get("error_y_mm", 0.0))
    error_mag = float(sample.get("error_mag_mm", math.sqrt(error_x * error_x + error_y * error_y)))
    if not finite_number(error_x) or not finite_number(error_y) or not finite_number(error_mag):
        sample["sample_valid"] = False
        sample["sample_rejected_reason"] = "non-finite sample measurement"
        log("RoundAxisSampleCheck", "invalid sample: %s" % sample["sample_rejected_reason"])
        return False
    log("RoundAxisSampleCheck", "angle=%.3f" % float(sample.get("command_angle_deg", 0.0)))
    log("RoundAxisSampleCheck", "error=(%.5f, %.5f)" % (error_x, error_y))
    log("RoundAxisSampleCheck", "error_mag=%.5f" % error_mag)
    warn_limit = float(cfg.get("round_object_raw_sample_error_warn_mm", 0.75))
    abort_limit = float(cfg.get("round_object_raw_sample_error_abort_mm", 999.0))
    abort_enabled = bool_value(cfg.get("round_object_abort_on_raw_sample_error", False))
    if error_mag > warn_limit:
        sample["raw_sample_error_warning"] = True
        log("RoundAxisSampleCheck", "WARNING residual sample error %.5f mm is large; continuing to fit." % error_mag)
        log("RoundAxisSampleCheck", "WARNING residual sample error is large; continuing because fit residual determines validity.")
        log("Diagnosis", "Large residual-on-existing-compensation sample error detected. This may be real calibration offset/contact behavior. Continuing to fit; final validity will be based on residuals.")
    else:
        sample["raw_sample_error_warning"] = False
    if abort_enabled and error_mag > abort_limit:
        sample["sample_valid"] = False
        sample["sample_rejected_reason"] = "residual sample error %.5f > round_object_raw_sample_error_abort_mm %.5f" % (error_mag, abort_limit)
        raise RoundAxisResidualSampleAbort("Round-axis residual sample error %.5f mm exceeded explicit abort limit %.5f mm." %
                                           (error_mag, abort_limit), [sample])
    sample["sample_valid"] = True
    sample["sample_rejected_reason"] = None
    return True


def collect_round_object_axis_samples(camera, nozzle, part, cal_loc, cfg, state):
    if not bool_value(cfg.get("round_object_measure_after_place", True)):
        raise Exception("round_object_measure_after_place must be true.")
    if not bool_value(cfg.get("round_object_rotate_after_pick", True)):
        raise Exception("round_object_rotate_after_pick must be true for round_object_nozzle_axis calibration.")
    samples = []
    angles = cfg.get("round_object_angles_deg", DEFAULT_CONFIG["round_object_angles_deg"])
    cycles = int(cfg.get("round_object_cycles", 2))
    current_die_loc = state.get("current_die_loc", cal_loc)
    sample_index = 0
    axis_cfg = dict(cfg)
    axis_cfg["require_orientation_circle"] = True
    axis_cfg["round_object_validate_die_orientation"] = False
    axis_cfg["align_nozzle_to_detected_die_rotation"] = False
    for cycle_index in range(cycles):
        for angle in angles:
            check_cancel("RoundAxis")
            sample, current_die_loc = measure_round_axis_sample(
                camera, nozzle, part, cal_loc, cfg, state, axis_cfg,
                current_die_loc, cycle_index, sample_index, angle)
            try:
                check_round_axis_sample(sample, cfg)
            except RoundAxisResidualSampleAbort, e:
                prior = list(samples)
                prior.extend(e.samples)
                e.samples = prior
                raise
            samples.append(sample)
            sample_index += 1
    return samples


def unique_round_axis_angles(samples):
    seen = {}
    for s in samples:
        if not bool_value(s.get("sample_valid", True)):
            continue
        seen["%.3f" % normalize_angle(float(s.get("command_angle_deg", 0.0)))] = True
    return seen.keys()


def valid_round_axis_samples(samples):
    out = []
    for s in samples:
        if bool_value(s.get("sample_valid", True)):
            out.append(s)
    return out


def fit_round_object_axis_model(samples, cfg):
    samples = valid_round_axis_samples(samples)
    if samples is None or len(samples) < 4:
        raise Exception("Need at least four round-object axis samples.")
    if len(samples) < int(cfg.get("round_object_min_good_samples", 6)):
        raise Exception("Need at least %d valid round-object samples; got %d." %
                        (int(cfg.get("round_object_min_good_samples", 6)), len(samples)))
    if len(unique_round_axis_angles(samples)) < 4:
        raise Exception("Need at least four unique round-object angles.")
    fit_static = bool_value(cfg.get("round_object_fit_static_offset", True))
    fit_residual_angle_vector = bool_value(cfg.get("round_object_fit_runout", True))
    if fit_static and fit_residual_angle_vector:
        n = 4
        ata = [[0.0 for j in range(n)] for i in range(n)]
        atb = [0.0 for i in range(n)]
        for s in samples:
            a = math.radians(float(s["command_angle_deg"]))
            ca = math.cos(a); sa = math.sin(a)
            rows = [([1.0, 0.0, ca, -sa], float(s["error_x_mm"])),
                    ([0.0, 1.0, sa, ca], float(s["error_y_mm"]))]
            for row, value in rows:
                for i in range(n):
                    atb[i] += row[i] * value
                    for j in range(n):
                        ata[i][j] += row[i] * row[j]
        bx, by, vx, vy = solve_linear_system(ata, atb)
    elif fit_static:
        bx = mean([float(s["error_x_mm"]) for s in samples])
        by = mean([float(s["error_y_mm"]) for s in samples])
        vx = 0.0; vy = 0.0
    else:
        n = 2
        ata = [[0.0, 0.0], [0.0, 0.0]]
        atb = [0.0, 0.0]
        bx = 0.0; by = 0.0
        for s in samples:
            a = math.radians(float(s["command_angle_deg"]))
            ca = math.cos(a); sa = math.sin(a)
            rows = [([ca, -sa], float(s["error_x_mm"])),
                    ([sa, ca], float(s["error_y_mm"]))]
            for row, value in rows:
                for i in range(n):
                    atb[i] += row[i] * value
                    for j in range(n):
                        ata[i][j] += row[i] * row[j]
        vx, vy = solve_linear_system(ata, atb)
    fit = {"residual_static_bias_x_mm": bx,
           "residual_static_bias_y_mm": by,
           "residual_angle_vector_x_mm": vx,
           "residual_angle_vector_y_mm": vy,
           "bias_x_mm": bx, "bias_y_mm": by,
           "runout_vector_x_mm": vx, "runout_vector_y_mm": vy}
    residuals = []
    predictions = []
    for s in samples:
        pred = round_axis_predicted_error(fit, s["command_angle_deg"])
        rx = float(s["error_x_mm"]) - pred["x"]
        ry = float(s["error_y_mm"]) - pred["y"]
        residual = math.sqrt(rx * rx + ry * ry)
        residuals.append(residual)
        predictions.append({"command_angle_deg": float(s["command_angle_deg"]),
                            "measured_error_x_mm": float(s["error_x_mm"]),
                            "measured_error_y_mm": float(s["error_y_mm"]),
                            "predicted_error_x_mm": pred["x"],
                            "predicted_error_y_mm": pred["y"],
                            "residual_mm": residual})
    radius = math.sqrt(vx * vx + vy * vy)
    phase = normalize_angle(math.degrees(math.atan2(vy, vx))) if radius > 0.000000001 else 0.0
    fit.update({"model": "residual_compensated_round_axis",
                "sample_count": len(samples),
                "unique_angle_count": len(unique_round_axis_angles(samples)),
                "residual_static_bias_mag_mm": math.sqrt(bx * bx + by * by),
                "residual_angle_radius_mm": radius,
                "residual_angle_phase_deg": phase,
                "residual_angle_vector_diagnostic_only": True,
                "bias_mag_mm": math.sqrt(bx * bx + by * by),
                "runout_radius_mm": radius,
                "runout_phase_deg": phase,
                "rms_error_mm": rms(residuals),
                "peak_error_mm": max(residuals) if residuals else 0.0,
                "sample_predictions": predictions})
    return fit


def select_round_axis_head_delta(samples, fit, cfg):
    sign_override = float(cfg.get("round_object_head_delta_sign_override", 1.0))
    bx = float(fit.get("residual_static_bias_x_mm", fit.get("bias_x_mm", 0.0)))
    by = float(fit.get("residual_static_bias_y_mm", fit.get("bias_y_mm", 0.0)))
    dx = -bx * sign_override
    dy = -by * sign_override
    log("RoundAxisFit", "residual_static_bias=(%.5f, %.5f)" % (bx, by))
    log("RoundAxisFit", "residual_angle_vector=(%.5f, %.5f)" %
        (float(fit.get("residual_angle_vector_x_mm", 0.0)),
         float(fit.get("residual_angle_vector_y_mm", 0.0))))
    log("RoundAxisFit", "residual_angle_vector_diagnostic_only=True")
    log("RoundAxisFit", "selected_head_delta_from=residual_static_bias_only")
    log("RoundAxisFit", "proposed_command_delta=(-residual_static_bias)=(%.5f, %.5f)" % (-bx, -by))
    log("RoundAxisFit", "sign_selection_method=explicit_negative_bias_not_rms_tie")
    log("RoundAxisFit", "round_object_head_delta_sign_override=%.5f" % sign_override)
    log("RoundAxisFit", "selected_head_delta=(%.5f, %.5f)" % (dx, dy))
    return {"sign_override": sign_override, "delta_x_mm": dx, "delta_y_mm": dy,
            "method": "explicit_negative_bias_not_rms_tie"}


def residual_sample_error_range(samples):
    values = []
    for s in samples:
        try:
            value = float(s.get("error_mag_mm", None))
            if finite_number(value):
                values.append(value)
        except:
            pass
    if not values:
        return None
    return {"min": min(values), "max": max(values)}


def round_axis_safety_check(samples, fit, selected_delta, cfg):
    reasons = []
    if len(unique_round_axis_angles(samples)) < 4:
        reasons.append("at least four unique angles are required")
    rms_error = float(fit.get("rms_error_mm", 999999.0))
    peak_error = float(fit.get("peak_error_mm", 999999.0))
    rms_limit = float(cfg.get("round_object_max_fit_rms_mm", 0.08))
    peak_limit = float(cfg.get("round_object_max_fit_peak_mm", 0.20))
    residuals_exceed = rms_error > rms_limit or peak_error > peak_limit
    if residuals_exceed:
        log("RoundAxisFit", "WARNING fit residuals exceed limits: rms=%.5f peak=%.5f limits=(%.5f, %.5f)" %
            (rms_error, peak_error, rms_limit, peak_limit))
        if bool_value(cfg.get("round_object_block_on_fit_residuals", False)):
            reasons.append("fit residuals exceed limits")
        else:
            log("RoundAxisFit", "Continuing because round_object_block_on_fit_residuals=false.")
    if float(fit.get("bias_mag_mm", 999999.0)) > float(cfg.get("round_object_max_static_offset_mm", 2.0)):
        reasons.append("static bias exceeds round_object_max_static_offset_mm")
    if float(fit.get("runout_radius_mm", 999999.0)) > float(cfg.get("round_object_max_runout_radius_mm", 2.0)):
        reasons.append("residual angle radius exceeds round_object_max_runout_radius_mm")
    if not finite_number(selected_delta.get("delta_x_mm", None)) or not finite_number(selected_delta.get("delta_y_mm", None)):
        reasons.append("selected head delta is non-finite")
    return len(reasons) == 0, "; ".join(reasons) if reasons else None


def log_fit_quality_warning(fit):
    log("Diagnosis", "orientation_dot_used_for_center=False")
    if (float(fit.get("bias_mag_mm", 0.0)) > 0.5 and
            float(fit.get("runout_radius_mm", 999999.0)) < 0.15 and
            float(fit.get("peak_error_mm", 0.0)) > 0.4):
        log("Diagnosis", "Residual-on-existing-compensation behavior is not explained by static bias + simple angle vector.")
        log("Diagnosis", "Fit residuals exceed limits. Result will be saved as diagnostic; preview remains available unless explicitly blocked.")
        return True
    return False


def log_fit_quality_diagnosis(fit):
    return log_fit_quality_warning(fit)


def nozzle_head_offsets_mm(nozzle):
    old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
    return float(old.getX()), float(old.getY())


def diagnostic_round_axis_result_from_failure(samples, reason, nozzle):
    old_x, old_y = nozzle_head_offsets_mm(nozzle)
    result = {
        "ok": False,
        "model": "round_object_nozzle_axis",
        "official_model": "round_object_nozzle_axis",
        "samples": samples or [],
        "valid_sample_count": len(valid_round_axis_samples(samples or [])),
        "nozzle_head_offset_delta_x_mm": 0.0,
        "nozzle_head_offset_delta_y_mm": 0.0,
        "nozzle_head_offset_delta_mag_mm": 0.0,
        "old_nozzle_head_offset_x_mm": old_x,
        "old_nozzle_head_offset_y_mm": old_y,
        "preview_new_nozzle_head_offset_x_mm": old_x,
        "preview_new_nozzle_head_offset_y_mm": old_y,
        "bias_x_mm": None,
        "bias_y_mm": None,
        "runout_radius_mm": None,
        "rms_error_mm": None,
        "peak_error_mm": None,
        "safe_to_apply": False,
        "correction_available": False,
        "confirm_preview_available": False,
        "apply_available": False,
        "apply_target": "nozzle_head_offsets",
        "apply_block_reason": reason,
        "diagnostic_note": "Calibration aborted before fit because an explicit residual sample abort limit or execution failure was hit."
    }
    result.update(compensation_stack_result_fields())
    return result


def compute_round_object_nozzle_axis_result(camera, nozzle, part, storage_loc, cal_loc, cfg, state):
    log("Mode", "EXECUTING ROUND OBJECT NOZZLE AXIS CALIBRATION")
    log("Mode", "round_object_nozzle_axis is interpreted as residual_compensated_round_axis; external mode name retained for compatibility.")
    samples = collect_round_object_axis_samples(camera, nozzle, part, cal_loc, cfg, state)
    valid_samples = valid_round_axis_samples(samples)
    if len(valid_samples) < int(cfg.get("round_object_min_good_samples", 6)):
        raise Exception("Need at least %d valid round-object samples; got %d." %
                        (int(cfg.get("round_object_min_good_samples", 6)), len(valid_samples)))
    fit = fit_round_object_axis_model(samples, cfg)
    selected_delta = select_round_axis_head_delta(samples, fit, cfg)
    old_x, old_y = nozzle_head_offsets_mm(nozzle)
    delta_x = float(selected_delta["delta_x_mm"])
    delta_y = float(selected_delta["delta_y_mm"])
    delta_mag = math.sqrt(delta_x * delta_x + delta_y * delta_y)
    safe, reason = round_axis_safety_check(samples, fit, selected_delta, cfg)
    fit_residuals_exceed_limits = (
        float(fit.get("rms_error_mm", 999999.0)) > float(cfg.get("round_object_max_fit_rms_mm", 0.08)) or
        float(fit.get("peak_error_mm", 999999.0)) > float(cfg.get("round_object_max_fit_peak_mm", 0.20)))
    diagnosis_failed = log_fit_quality_warning(fit)
    fit_residual_warning = None
    if fit_residuals_exceed_limits:
        fit_residual_warning = "fit residuals exceed limits; using computed delta as diagnostic/bring-up correction only"
        log("RoundAxisFit", "WARNING %s" % fit_residual_warning)
        if bool_value(cfg.get("round_object_block_on_fit_residuals", False)):
            safe = False
            reason = "fit residuals exceed limits"
    if diagnosis_failed and bool_value(cfg.get("round_object_block_on_fit_residuals", False)):
        safe = False
        reason = "fit residuals exceed limits"
    if delta_mag > float(cfg.get("max_apply_mm", 0.5)):
        safe = False
        if reason is None:
            reason = "selected head delta exceeds max_apply_mm"
    log("RoundAxisFit", "residual_static_bias=(%.5f, %.5f)" %
        (fit["residual_static_bias_x_mm"], fit["residual_static_bias_y_mm"]))
    log("RoundAxisFit", "residual_angle_vector=(%.5f, %.5f) residual_angle_radius=%.5f residual_angle_phase=%.5f diagnostic_only=True rms=%.5f peak=%.5f" %
        (fit["residual_angle_vector_x_mm"], fit["residual_angle_vector_y_mm"],
         fit["residual_angle_radius_mm"], fit["residual_angle_phase_deg"],
         fit["rms_error_mm"], fit["peak_error_mm"]))
    residual_range = residual_sample_error_range(samples)
    if residual_range is not None:
        log("RoundAxisFit", "residual_sample_error_range=(%.5f, %.5f)" %
            (float(residual_range["min"]), float(residual_range["max"])))
    else:
        log("RoundAxisFit", "residual_sample_error_range=(None,None)")
    log("RoundAxisFit", "residual_rms=%.5f" % float(fit["rms_error_mm"]))
    log("RoundAxisFit", "residual_peak=%.5f" % float(fit["peak_error_mm"]))
    log("RoundAxisFit", "residual_limits_exceeded=%s" % str(bool(fit_residuals_exceed_limits)))
    log("RoundAxisFit", "residual_limit_action=%s" %
        ("block" if bool_value(cfg.get("round_object_block_on_fit_residuals", False)) else "warning_only"))
    log("RoundAxisFit", "validity_basis=post_fit_residuals_on_existing_compensation")
    preview_allowed = bool(safe) or bool_value(cfg.get("allow_high_residual_diagnostic_preview", True)) or bool_value(cfg.get("confirm_allow_unsafe_diagnostic_preview", False))
    final_return_preview_allowed = bool(safe) or (bool(fit_residuals_exceed_limits) and bool_value(cfg.get("allow_high_residual_final_return_preview", True)))
    if final_return_preview_allowed:
        state["round_axis_same_run_preview_delta"] = {"dx": delta_x, "dy": delta_y}
        state["round_axis_same_run_preview_delta_high_residual"] = bool(fit_residuals_exceed_limits)
    computed = {
        "ok": True,
        "model": "round_object_nozzle_axis",
        "official_model": "round_object_nozzle_axis",
        "internal_model": "residual_compensated_round_axis",
        "samples": samples,
        "valid_sample_count": len(valid_samples),
        "fit": fit,
        "sign_selection_method": "explicit_negative_bias_not_rms_tie",
        "round_object_head_delta_sign_override": float(selected_delta["sign_override"]),
        "nozzle_head_offset_delta_x_mm": delta_x,
        "nozzle_head_offset_delta_y_mm": delta_y,
        "nozzle_head_offset_delta_mag_mm": delta_mag,
        "old_nozzle_head_offset_x_mm": old_x,
        "old_nozzle_head_offset_y_mm": old_y,
        "preview_new_nozzle_head_offset_x_mm": old_x + delta_x,
        "preview_new_nozzle_head_offset_y_mm": old_y + delta_y,
        "bias_x_mm": float(fit["bias_x_mm"]),
        "bias_y_mm": float(fit["bias_y_mm"]),
        "residual_static_bias_x_mm": float(fit["residual_static_bias_x_mm"]),
        "residual_static_bias_y_mm": float(fit["residual_static_bias_y_mm"]),
        "residual_static_bias_mag_mm": float(fit["residual_static_bias_mag_mm"]),
        "residual_angle_vector_x_mm": float(fit["residual_angle_vector_x_mm"]),
        "residual_angle_vector_y_mm": float(fit["residual_angle_vector_y_mm"]),
        "residual_angle_radius_mm": float(fit["residual_angle_radius_mm"]),
        "residual_angle_phase_deg": float(fit["residual_angle_phase_deg"]),
        "residual_angle_vector_diagnostic_only": True,
        "runout_vector_x_mm": float(fit["runout_vector_x_mm"]),
        "runout_vector_y_mm": float(fit["runout_vector_y_mm"]),
        "runout_radius_mm": float(fit["runout_radius_mm"]),
        "runout_phase_deg": float(fit["runout_phase_deg"]),
        "rms_error_mm": float(fit["rms_error_mm"]),
        "peak_error_mm": float(fit["peak_error_mm"]),
        "fit_residuals_exceed_limits": bool(fit_residuals_exceed_limits),
        "fit_residual_warning": fit_residual_warning,
        "safe_to_apply": bool(safe),
        "correction_available": bool(preview_allowed),
        "confirm_preview_available": bool(preview_allowed),
        "diagnostic_preview_available": bool(preview_allowed),
        "correction_is_diagnostic": not bool(safe),
        "apply_available": False,
        "apply_target": "nozzle_head_offsets",
        "apply_block_reason": reason,
        "diagnostic_note": "Residual angle-dependent term is diagnostic only and is not raw runout; apply writes residual static nozzle head offset delta only."
    }
    computed.update(compensation_stack_result_fields())
    log("RoundAxisFit", "selected_head_delta=(%.5f, %.5f)" % (delta_x, delta_y))
    log("RoundAxisFit", "correction_available=%s" % str(bool(preview_allowed)))
    log("RoundAxisFit", "diagnostic_preview_available=%s" % str(bool(preview_allowed)))
    log("RoundAxisFit", "apply_available=False")
    log("Result", "computed_model=round_object_nozzle_axis")
    return computed


# ---------------------------------------------------------------------------
# confirm / apply

def apply_availability_for_result(result, cfg):
    computed = result.get("computed", {}) or {}
    reason = None
    safe = bool_value(computed.get("safe_to_apply", False))
    residual_bad = bool_value(computed.get("fit_residuals_exceed_limits", False))
    high_residual_apply_override = (
        residual_bad and
        bool_value(cfg.get("allow_high_residual_apply", False)) and
        bool_value(cfg.get("allow_apply_without_verification", False)))
    if computed.get("model", "") != "round_object_nozzle_axis":
        reason = "Only round_object_nozzle_axis results can be applied."
    elif residual_bad and not high_residual_apply_override:
        reason = "fit residuals exceed limits; allow_high_residual_apply is false"
    elif not safe and not high_residual_apply_override:
        reason = computed.get("apply_block_reason", "result is not safe_to_apply")
    elif not bool_value(cfg.get("allow_machine_writes", False)):
        reason = "allow_machine_writes is false"
    elif float(computed.get("nozzle_head_offset_delta_mag_mm", 999999.0)) > float(cfg.get("max_apply_mm", 0.5)):
        reason = "nozzle head offset delta exceeds max_apply_mm"
    return reason is None, reason


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
    backup = path + ".pick_cameraAlign_Calibration.%d.bak" % int(time.time())
    shutil.copy2(path, backup)
    log("Apply", "machine.xml backup written: %s" % backup)
    return backup


def enforce_write_guard(cfg):
    if not bool_value(cfg.get("allow_machine_writes", False)):
        log("Safety", "write guard active")
        raise Exception("allow_machine_writes is false; refusing machine calibration writes.")


def get_last_result_apply_preview(progress_callback=None, cancel_token=None):
    cfg = load_config()
    result = load_last_result()
    computed = result.get("computed", {}) or {}
    can_apply, reason = apply_availability_for_result(result, cfg)
    return {
        "can_apply": bool(can_apply),
        "reason": reason,
        "model": computed.get("model", None),
        "measured_error_x_mm": computed.get("bias_x_mm", None),
        "measured_error_y_mm": computed.get("bias_y_mm", None),
        "delta_x_mm": computed.get("nozzle_head_offset_delta_x_mm", None),
        "delta_y_mm": computed.get("nozzle_head_offset_delta_y_mm", None),
        "old_nozzle_head_offset_x_mm": computed.get("old_nozzle_head_offset_x_mm", None),
        "old_nozzle_head_offset_y_mm": computed.get("old_nozzle_head_offset_y_mm", None),
        "preview_new_nozzle_head_offset_x_mm": computed.get("preview_new_nozzle_head_offset_x_mm", None),
        "preview_new_nozzle_head_offset_y_mm": computed.get("preview_new_nozzle_head_offset_y_mm", None),
        "verification_passed": bool_value(result.get("verification_passed", False)),
        "safe_to_apply": bool_value(computed.get("safe_to_apply", False)),
        "max_apply_mm": cfg.get("max_apply_mm", None)
    }


def apply_result(result=None, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    old_progress = _progress_callback
    old_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token
    try:
        cfg = load_config()
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = _prepare_runtime(cfg)
        result = load_last_result()
        if result.get("error"):
            raise Exception("Refusing to apply failed result: %s" % result.get("error"))
        computed = result.get("computed", {}) or {}
        if computed.get("model", "") != "round_object_nozzle_axis":
            raise Exception("Refusing to apply model '%s'; only round_object_nozzle_axis is supported." % computed.get("model", "unknown"))
        can_apply, reason = apply_availability_for_result(result, cfg)
        if not can_apply:
            raise Exception(reason)
        enforce_write_guard(cfg)
        log("Apply", "Applying residual static nozzle-head-offset delta.")
        log("Apply", "Existing OpenPnP compensation remains active.")
        log("Apply", "No runout/nozzle-tip compensation written.")
        backup = backup_machine_xml_if_requested(cfg)
        old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
        new = mm_loc(float(old.getX()) + float(computed["nozzle_head_offset_delta_x_mm"]),
                     float(old.getY()) + float(computed["nozzle_head_offset_delta_y_mm"]),
                     old.getZ(), old.getRotation())
        nozzle.setHeadOffsets(new)
        applied = {"target": "nozzle_head_offsets",
                   "old_x_mm": float(old.getX()), "old_y_mm": float(old.getY()),
                   "new_x_mm": float(new.getX()), "new_y_mm": float(new.getY()),
                   "delta_x_mm": float(new.getX() - old.getX()),
                   "delta_y_mm": float(new.getY() - old.getY()),
                   "machine_xml_backup": backup,
                   "runout_written": False,
                   "timestamp": time.time()}
        result["applied"] = applied
        result["machine_config_modified"] = True
        result["applied_timestamp"] = applied["timestamp"]
        save_result(result)
        log("Apply", "Applied nozzle head offsets old=(%.5f, %.5f) new=(%.5f, %.5f); runout/nozzle-tip compensation not written." %
            (old.getX(), old.getY(), new.getX(), new.getY()))
        return result
    finally:
        _progress_callback = old_progress
        _cancel_token = old_cancel


def correction_delta_from_result(result, cfg):
    computed = result.get("computed", {}) or {}
    if computed.get("model", "") != "round_object_nozzle_axis":
        return {"dx": 0.0, "dy": 0.0, "model": computed.get("model", None),
                "available": False, "reason": "Confirm requires round_object_nozzle_axis result."}
    safe = bool_value(computed.get("safe_to_apply", False))
    residual_bad = bool_value(computed.get("fit_residuals_exceed_limits", False))
    allow_diag = (bool_value(cfg.get("allow_high_residual_diagnostic_preview", True)) or
                  bool_value(cfg.get("confirm_allow_unsafe_diagnostic_preview", False)))
    dx = float(computed.get("nozzle_head_offset_delta_x_mm", 0.0))
    dy = float(computed.get("nozzle_head_offset_delta_y_mm", 0.0))
    if not safe and not allow_diag:
        return {"dx": 0.0, "dy": 0.0, "model": "round_object_nozzle_axis",
                "available": False,
                "diagnostic_preview": False,
                "apply_forbidden": True,
                "reason": computed.get("apply_block_reason", "result is not safe_to_apply")}
    if residual_bad and allow_diag:
        log("Confirm", "HIGH RESIDUAL DIAGNOSTIC PREVIEW ONLY")
        log("Confirm", "Using computed residual static delta despite high fit residuals.")
    elif not safe:
        log("Confirm", "UNSAFE DIAGNOSTIC PREVIEW ONLY")
    log("Confirm", "using residual same-run preview delta on top of active OpenPnP compensation")
    log("Confirm", "not applying/writing persistent runout")
    return {"dx": dx,
            "dy": dy,
            "model": "round_object_nozzle_axis",
            "available": True,
            "diagnostic_preview": bool(residual_bad or not safe),
            "apply_forbidden": bool(residual_bad or not safe),
            "reason": "saved computed residual static nozzle_head_offset_delta"}


def test_vision_lock(location_key, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    old_progress = _progress_callback
    old_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token
    try:
        cfg = load_config()
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = _prepare_runtime(cfg)
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
        assert_measurement_uses_camera_only(lock, "Vision lock test")
        pose = lock["pose"]
        result = {
            "type": "vision_lock",
            "location_key": key,
            "circle_count": int(pose.get("circle_count", 0)),
            "center_x_mm": float(detected.getX()),
            "center_y_mm": float(detected.getY()),
            "rotation_deg": float(detected.getRotation()),
            "search_dx_mm": float(pose.get("search_dx_mm", 0.0)),
            "search_dy_mm": float(pose.get("search_dy_mm", 0.0)),
            "timestamp": time.time()
        }
        return result
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
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = _prepare_runtime(cfg)
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
            move_tool_safe(camera, target, "Move")
        elif tool == "nozzle":
            target = nearest_nozzle_rotation_location(nozzle, mm_loc(base_loc.getX(), base_loc.getY(), float(cfg["safe_travel_z"]), base_loc.getRotation()), "Move", cfg)
            move_tool_safe(nozzle, target, "Move")
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
    _progress_callback = progress_callback
    _cancel_token = cancel_token
    try:
        cfg = load_config()
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = _prepare_runtime(cfg)
        if result is None or not isinstance(result, dict):
            if result is not None and not isinstance(result, dict):
                show_shift = bool_value(result)
            result = load_last_result()
        computed = result.get("computed", {}) or {}
        delta = correction_delta_from_result(result, cfg)
        log("Confirm", "diagnostic preview uses residual delta only on top of active OpenPnP compensation.")
        correction_available = bool_value(delta.get("available", False))
        correction_used = correction_available and bool_value(show_shift)
        if result.get("error") or not correction_available:
            log("Confirm", "result_loaded=True")
            log("Confirm", "result_model=%s" % computed.get("model", None))
            log("Confirm", "correction_available=False")
            log("Confirm", "correction_reason=%s" % delta.get("reason", result.get("error", "result is unsafe")))
            log("Confirm", "showing no-shift only because result is unsafe")
            show_shift = False
        anchor_loc = storage_loc
        tag = "Storage"
        if str(cfg.get("confirm_anchor_location", "Storage")).lower() == "cal":
            anchor_loc = cal_loc
            tag = "Cal"
        lock, detected = acquire_pose_for_location(camera, nozzle, part, anchor_loc, cfg, tag)
        assert_measurement_uses_camera_only(lock, "Confirm")
        assert_nozzle_motion_uses_tool_aware(lock, "Confirm")
        nozzle_target = lock["nozzle_target_location"]
        no_shift_command = mm_loc(nozzle_target.getX(), nozzle_target.getY(), float(cfg["safe_travel_z"]), nozzle_target.getRotation())
        shift_command = mm_loc(nozzle_target.getX() + delta["dx"], nozzle_target.getY() + delta["dy"],
                               float(cfg["safe_travel_z"]), nozzle_target.getRotation())
        mag = math.sqrt(delta["dx"] * delta["dx"] + delta["dy"] * delta["dy"])
        if mag < float(cfg.get("confirm_visual_min_shift_mm", 0.05)):
            log("Confirm", "WARNING shift magnitude %.5f is below confirm_visual_min_shift_mm; views may look identical." % mag)
        log("Confirm", "result_loaded=True")
        log("Confirm", "result_model=%s" % computed.get("model", None))
        log("Confirm", "detected_measurement_center=(%.5f, %.5f)" %
            (detected.getX(), detected.getY()))
        log("Confirm", "nozzle_target_center=(%.5f, %.5f)" %
            (nozzle_target.getX(), nozzle_target.getY()))
        log("Confirm", "no_shift_command=%s" % str(location_to_diag(no_shift_command)))
        log("Confirm", "shift_command=%s" % str(location_to_diag(shift_command)))
        log("Confirm", "shift_minus_no_shift=(%.5f, %.5f, %.5f)" % (delta["dx"], delta["dy"], mag))
        log("Confirm", "correction_available=%s" % str(bool(correction_available)))
        log("Confirm", "correction_used=%s" % str(bool(correction_used)))
        log("Confirm", "apply_forbidden=%s" % str(bool_value(delta.get("apply_forbidden", False))))
        log("Confirm", "correction_model=%s" % delta["model"])
        log("Confirm", "correction_reason=%s" % delta["reason"])
        selected = shift_command if bool(correction_used) else no_shift_command
        move_nozzle_split(nozzle, nearest_nozzle_rotation_location(nozzle, selected, "Confirm", cfg), cfg, "Confirm", tag)
        final = selected
        if bool_value(cfg.get("confirm_shift_descend_to_surface", False)):
            down = mm_loc(selected.getX(), selected.getY(), pick_surface_z_mm(anchor_loc, cfg), selected.getRotation())
            move_nozzle_split(nozzle, nearest_nozzle_rotation_location(nozzle, down, "Confirm", cfg), cfg, "Confirm", tag)
            final = down
        confirm = {"type": "confirm_shift",
                   "mode": "show_shift" if bool_value(show_shift) else "show_no_shift",
                   "show_shift": bool(correction_used),
                   "result_loaded": True,
                   "result_model": computed.get("model", None),
                   "correction_available": bool(correction_available),
                   "detected_measurement_center": location_to_diag(detected),
                   "detected_nozzle_target_center": location_to_diag(nozzle_target),
                   "no_shift_command": location_to_diag(no_shift_command),
                   "shift_command": location_to_diag(shift_command),
                   "shift_minus_no_shift": {"dx": delta["dx"], "dy": delta["dy"], "mag": mag},
                   "correction_used": bool(correction_used),
                   "correction_model": delta["model"],
                   "correction_reason": delta["reason"],
                   "warning": None if bool(correction_available) else "unsafe result; showing no-shift only",
                   "target": location_to_diag(final),
                   "target_x_mm": float(final.getX()),
                   "target_y_mm": float(final.getY()),
                   "target_z_mm": float(final.getZ()),
                   "target_rotation_deg": float(final.getRotation()),
                   "correction_x_mm": float(delta["dx"]),
                   "correction_y_mm": float(delta["dy"]),
                   "circle_count": int(lock.get("pose", {}).get("circle_count", 0)),
                   "diagnostic_preview": bool_value(delta.get("diagnostic_preview", False)),
                   "apply_forbidden": bool_value(delta.get("apply_forbidden", False)),
                   "timestamp": time.time()}
        save_last_confirm_result(confirm)
        return confirm
    finally:
        _progress_callback = old_progress
        _cancel_token = old_cancel


# ---------------------------------------------------------------------------
# run / GUI entry points

def log_result_summary(result):
    computed = result.get("computed", {}) or {}
    log("Result", "computed_model=%s safe_to_apply=%s delta=(%.5f, %.5f, %.5f)" %
        (computed.get("model", None),
         str(bool_value(computed.get("safe_to_apply", False))),
         float(computed.get("nozzle_head_offset_delta_x_mm", 0.0)),
         float(computed.get("nozzle_head_offset_delta_y_mm", 0.0)),
         float(computed.get("nozzle_head_offset_delta_mag_mm", 0.0))))
    log("Result", "apply_available=%s" % str(bool_value(computed.get("apply_available", result.get("apply_available", False)))))
    log("Result", "confirm_preview_available=%s" % str(bool_value(computed.get("confirm_preview_available", False))))


def run(apply_offsets=False, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    old_progress = _progress_callback
    old_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token
    cfg = None
    nozzle = None
    state = {"die_on_nozzle": False, "die_at_work": False,
             "current_die_loc": None, "current_die_loc_trusted": False,
             "last_trusted_die_loc": None}
    result = None
    try:
        if bool_value(apply_offsets):
            raise Exception("run(apply_offsets=True) is not supported. Run calibration, then call apply_result() explicitly.")
        cfg = load_config()
        mode = calibration_result_mode(cfg)
        assert_round_object_pixel_transform_policy(cfg)
        log("Mode", "effective_calibration_result_mode=%s" % mode)
        log_compensation_stack_assumptions()
        log("Mode", "round_object_validate_die_orientation=%s" %
            str(bool(bool_value(cfg.get("round_object_validate_die_orientation", False)))))
        log("PixelTransform", "effective_measurement_method=camera_only")
        log("PixelTransform", "effective_nozzle_target_method=tool_aware")
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = _prepare_runtime(cfg)
        runtime_cfg = dict(cfg)
        if bool_value(runtime_cfg.get("round_object_disable_detected_orientation_alignment", True)):
            runtime_cfg["align_nozzle_to_detected_die_rotation"] = False
            log("Mode", "round_object_disable_detected_orientation_alignment=True")
        detected_storage_pick_loc = transfer_die_from_storage_to_cal(camera, nozzle, part, storage_loc, cal_loc, runtime_cfg, state)
        lock, initial_work_pose = acquire_pose_for_location(camera, nozzle, part, cal_loc, runtime_cfg, "Cal")
        assert_measurement_uses_camera_only(lock, "Initial work pose")
        state["current_die_loc"] = initial_work_pose
        state["current_die_loc_trusted"] = True
        state["last_trusted_die_loc"] = initial_work_pose
        computed = compute_round_object_nozzle_axis_result(camera, nozzle, part, storage_loc, cal_loc, runtime_cfg, state)
        storage_return = return_die_to_storage(camera, nozzle, part, storage_loc, runtime_cfg, state)
        result = {
            "timestamp": time.time(),
            "script_model": "four_circle",
            "script_file": "pick_cameraAlign_Calibration.py",
            "apply_offsets_requested": False,
            "machine_config_modified": False,
            "calibration_result_mode": "round_object_nozzle_axis",
            "part_id_or_name": cfg["part_id_or_name"],
            "nozzle_name": object_name(nozzle),
            "camera_name": object_name(camera),
            "computed": computed,
            "applied": None,
            "verification": {"model": "round_object_nozzle_axis_fit",
                             "passed": bool_value(computed.get("safe_to_apply", False)),
                             "rms_error_mm": computed.get("rms_error_mm", None),
                             "peak_error_mm": computed.get("peak_error_mm", None),
                             "samples": computed.get("samples", [])},
            "verification_passed": bool_value(computed.get("safe_to_apply", False)),
            "storage": {"configured": location_to_diag(storage_loc),
                        "detected_initial_pick": location_to_diag(detected_storage_pick_loc),
                        "detected_minus_configured": {
                            "dx": float(detected_storage_pick_loc.getX() - storage_loc.getX()),
                            "dy": float(detected_storage_pick_loc.getY() - storage_loc.getY())},
                        "used_for_calibration": False},
            "initial_work_pose": location_to_diag(initial_work_pose),
            "storage_return": storage_return
        }
        result["apply_available"], result["apply_block_reason"] = apply_availability_for_result(result, runtime_cfg)
        computed["apply_available"] = result["apply_available"]
        computed["apply_block_reason"] = result["apply_block_reason"]
        save_result(result)
        log_result_summary(result)
        return result
    except RoundAxisResidualSampleAbort, e:
        log("Error", e)
        computed = diagnostic_round_axis_result_from_failure(e.samples, str(e), nozzle)
        result = {
            "timestamp": time.time(),
            "script_model": "four_circle",
            "script_file": "pick_cameraAlign_Calibration.py",
            "apply_offsets_requested": False,
            "machine_config_modified": False,
            "calibration_result_mode": "round_object_nozzle_axis",
            "part_id_or_name": cfg["part_id_or_name"] if cfg is not None else None,
            "nozzle_name": object_name(nozzle),
            "camera_name": None,
            "computed": computed,
            "applied": None,
            "verification": {"model": "round_object_nozzle_axis_fit",
                             "passed": False,
                             "rms_error_mm": None,
                             "peak_error_mm": None,
                             "samples": computed.get("samples", [])},
            "verification_passed": False,
            "apply_available": False,
            "apply_block_reason": str(e),
            "confirm_preview_available": False,
            "error": str(e),
            "traceback": traceback.format_exc()
        }
        log("Result", "computed_model=round_object_nozzle_axis")
        log("Result", "safe_to_apply=False")
        log("Result", "apply_available=False")
        log("Result", "confirm_preview_available=False")
        save_result(result)
        raise
    except Exception, e:
        log("Error", e)
        if result is None:
            result = {"error": str(e), "timestamp": time.time(),
                      "script_model": "four_circle",
                      "script_file": "pick_cameraAlign_Calibration.py",
                      "traceback": traceback.format_exc()}
        else:
            result["script_model"] = "four_circle"
            result["script_file"] = "pick_cameraAlign_Calibration.py"
            result["error"] = str(e)
            result["traceback"] = traceback.format_exc()
        save_result(result)
        raise
    finally:
        if nozzle is not None and cfg is not None:
            try:
                loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
                lift = nearest_nozzle_rotation_location(nozzle, mm_loc(loc.getX(), loc.getY(), float(cfg["safe_travel_z"]), loc.getRotation()), "Cleanup", cfg)
                move_to_with_speed(nozzle, lift, cfg)
                log("Cleanup", "Nozzle lifted to safe Z.")
            except Exception, cleanup_error:
                log("Cleanup", "Final safe lift failed: %s" % cleanup_error)
        _progress_callback = old_progress
        _cancel_token = old_cancel


def gui_status():
    cfg = load_config()
    status = {"effective_mode": calibration_result_mode(cfg),
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
        status["shift_delta_mag_mm"] = computed.get("nozzle_head_offset_delta_mag_mm", None)
        if status["shift_delta_mag_mm"] is not None and float(status["shift_delta_mag_mm"]) < float(cfg.get("confirm_visual_min_shift_mm", 0.05)):
            status["warning"] = "shift magnitude is tiny and may look identical"
    except Exception, e:
        status["last_result_error"] = str(e)
    return status


def open_last_log():
    return result_path()


def open_last_result():
    return result_path()


if __name__ == "__main__":
    print json.dumps(run(False), sort_keys=True, indent=2)
