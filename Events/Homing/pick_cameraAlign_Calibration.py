# Jython 2.7 / OpenPnP
# Runtime pick/nozzle-to-top-camera alignment calibration using a 4-circle die.
#
# Location Z values are support/base Z values. Pick/place Z is base Z plus
# the selected part height, for both storage and calibration work locations.

import json
import math
import os
import time
import traceback
import inspect

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
_dry_run_mode_warning_logged = False
_legacy_mode_warning_logged = False
_deprecated_config_warning_logged = False
_last_vision_image_path = None
_last_vision_image_size = None

DEFAULT_CONFIG = {
    "enabled_on_homing": False,
    "apply_offsets_on_homing": False,
    "allow_machine_writes": False,
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
    "pick_z_offset": 0.0,
    "pick_z_offset_mode": "relative_to_location_z",
    "pick_z_source_mode": "location_z_only",
    "fixed_pick_z_mm": None,
    "max_pick_descent_mm": 2.0,
    "z_clearance_before_pick_mm": 0.0,
    "lock_xy_to_configured_location": True,
    "pick_place_xy_mode": "detected_center",
    "feeder_name_override": None,
    "use_high_level_nozzle_pick": True,
    "allow_vacuum_only_pick_without_part_state": False,
    "dry_run_motion_mode": "vision_only",
    "require_visual_lock_before_pick": True,
    "verify_lock_after_centering": True,
    "storage_camera_z_mode": "relative_to_storage_z",
    "storage_camera_z_offset": 0.0,
    "cal_camera_z_mode": "relative_to_cal_work_z",
    "cal_camera_z_offset": 0.0,
    "test_iterations": 3,
    "verification_iterations": 4,
    "max_retries_per_step": 3,
    "circle_detection_min_count": 4,
    "circle_result_stage_name": "results",
    "orientation_result_stage_name": "OrientCircleResult",
    "orientation_circle_from_results": True,
    "orientation_circle_max_square_size_ratio": 0.8,
    "require_orientation_circle": True,
    "orientation_x_sign": -1.0,
    "orientation_sign_probe_enabled": True,
    "orientation_sign_probe_degrees": [12.0, -12.0],
    "orientation_sign_probe_max_error_deg": 3.0,
    "auto_update_orientation_x_sign": False,
    "orientation_snap_to_cardinal": True,
    "orientation_max_error_deg": 25.0,
    "orientation_validation_mode": "auto",
    "orientation_continuous_max_error_deg": 5.0,
    "overlay_style": {
        "square_width": 8.0,
        "circle_width": 6.0,
        "circle_radius": 14.0,
        "cross_size": 18.0,
        "key_scale": 1.4,
        "actual_color": "#24B35F",
        "expected_color": "#FFB020",
        "circle_color": "#2A8CFF",
        "orientation_color": "#F04C4C"
    },
    "abort_on_large_computed_offset": True,
    "expected_center_tolerance_mm": 2.0,
    "align_nozzle_to_detected_die_rotation": True,
    "calibration_step_mm": 0.5,
    "motion_speed": 1.0,
    "calibration_moves": [
        {"dx_mm": 0.5, "dy_mm": 0.0},
        {"dx_mm": 0.0, "dy_mm": 0.5},
        {"dx_mm": -0.5, "dy_mm": 0.0},
        {"dx_mm": 0.0, "dy_mm": -0.5},
        {"dx_mm": 0.35, "dy_mm": 0.35},
        {"dx_mm": -0.35, "dy_mm": 0.35},
        {"dx_mm": -0.35, "dy_mm": -0.35},
        {"dx_mm": 0.35, "dy_mm": -0.35}
    ],
    "max_apply_mm": 0.5,
    "max_verify_correction_mm": 2.0,
    "pivot_probe_enabled": True,
    "pivot_probe_angles_deg": [0.0, 45.0, 90.0, 135.0, 180.0, -135.0, -90.0, -45.0],
    "pivot_probe_xy_mm": {"x": 0.0, "y": 0.0},
    "use_pivot_correction": True,
    "storage_return_use_correction_only_if_verified": True,
    "final_storage_return_mode": "simple_configured_pocket_return",
    "final_storage_offset_source": "manual",
    "manual_final_storage_offset_mm": {"x": 0.0, "y": 0.0},
    "final_return_pick_max_centering_error_mm": 0.75,
    "final_pickup_apply_pick_vector": True,
    "final_pickup_pick_vector_sign": 1.0,
    "final_pickup_max_correction_mm": 1.5,
    "auto_update_manual_storage_offset_from_confirm": False,
    "confirm_shift_descend_to_surface": False,
    "storage_closed_loop_return_enabled": False,
    "storage_post_place_verify_enabled": False,
    "allow_storage_retry_pick_after_place": False,
    "max_storage_return_command_offset_mm": 0.5,
    "storage_closed_loop_max_passes": 2,
    "storage_closed_loop_tolerance_mm": 0.15,
    "storage_closed_loop_tolerance_deg": 2.0,
    "verification_pass_max_offset_mm": 0.20,
    "verification_pass_max_theta_deg": 1.0,
    "apply_target": "nozzle_head_offsets",
    "force_vacuum_actuator_name": "VAC1",
    "force_blowoff_actuator_name": "Nitrogen",
    "vacuum_settle_ms": 150,
    "blowoff_pulse_ms": 120,
    "enable_blowoff_on_place": True,
    "use_openpnp_safe_z_for_pick_place": False,
    "angles_deg": [0.0, 45.0, 90.0, 135.0, 180.0, -135.0, -90.0, -45.0],
    "settle_ms": 250,
    "spiral_search": {
        "enabled": True,
        "start_radius_mm": 0.25,
        "radius_step_mm": 0.25,
        "max_radius_mm": 3.0,
        "angle_step_deg": 45.0,
        "settle_ms": 250
    }
}


def log(tag, msg):
    line = "[%s][%s] %s" % (TAG, tag, str(msg))
    print line
    try:
        if _progress_callback is not None:
            _progress_callback(tag, str(msg), line)
    except:
        pass


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


def load_config():
    global _legacy_mode_warning_logged
    global _deprecated_config_warning_logged
    path = config_path()
    if not os.path.exists(path):
        save_config(DEFAULT_CONFIG)
        log("Init", "Config did not exist; wrote example config to %s" % path)
        return dict(DEFAULT_CONFIG)

    f = open(path, "r")
    try:
        data = json.loads(f.read())
    finally:
        f.close()
    cfg = deep_update(DEFAULT_CONFIG, data)
    if len(cfg.get("pivot_probe_angles_deg", [])) < 8:
        log("Safety", "Configured pivot_probe_angles_deg has fewer than 8 angles; using mixed 8-angle default for this run.")
        cfg["pivot_probe_angles_deg"] = list(DEFAULT_CONFIG["pivot_probe_angles_deg"])
    if len(cfg.get("angles_deg", [])) < 8:
        log("Safety", "Configured angles_deg has fewer than 8 angles; using mixed 8-angle default for this run.")
        cfg["angles_deg"] = list(DEFAULT_CONFIG["angles_deg"])
    if int(cfg.get("verification_iterations", 0)) < 4:
        log("Safety", "Configured verification_iterations is below 4; using 4 for this run.")
        cfg["verification_iterations"] = 4
    if not _legacy_mode_warning_logged:
        if ("pick_z_offset_mode" in data or "pick_z_source_mode" in data or
                "storage_camera_z_mode" in data or "cal_camera_z_mode" in data):
            log("Safety", "Deprecated Z mode keys are loaded for migration only; runtime uses single-Z model.")
            _legacy_mode_warning_logged = True
    if not _deprecated_config_warning_logged:
        deprecated = []
        for key in ["storage_base_z_mm", "part_height_z_mm", "die_storage_end_location_xyz"]:
            if key in data:
                deprecated.append(key)
        if deprecated:
            log("Safety", "Deprecated config keys ignored at runtime: %s" % ", ".join(deprecated))
            _deprecated_config_warning_logged = True
    mode = str(cfg.get("final_storage_return_mode", "simple_configured_pocket_return")).lower()
    if mode in ["simple_offset", "explicit_storage_offset", "vector_model_diagnostic", "diagnostic_computed_offset"]:
        log("Safety", "Migrating legacy final_storage_return_mode=%s to simple_configured_pocket_return for this run." % mode)
        cfg["final_storage_return_mode"] = "simple_configured_pocket_return"
    derive_storage_z_fields(cfg)
    return cfg


def save_config(cfg):
    path = config_path()
    parent = os.path.dirname(path)
    if not os.path.exists(parent):
        os.makedirs(parent)
    f = open(path, "w")
    try:
        f.write(json.dumps(cfg, sort_keys=True, indent=2))
    finally:
        f.close()


def save_result(result):
    f = open(result_path(), "w")
    try:
        f.write(json.dumps(result, sort_keys=True, indent=2))
    finally:
        f.close()


def load_last_result():
    path = result_path()
    if not os.path.exists(path):
        raise Exception("No saved calibration result found: %s" % path)
    f = open(path, "r")
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


def load_last_confirm_result():
    path = last_confirm_path()
    if not os.path.exists(path):
        raise Exception("No saved confirm result found: %s" % path)
    f = open(path, "r")
    try:
        return json.loads(f.read())
    finally:
        f.close()


def require_key(cfg, key):
    if key not in cfg or cfg[key] is None:
        raise Exception("Missing required config key: %s" % key)


def length_to_mm(value):
    if value is None:
        return None
    try:
        value = value.convertToUnits(LengthUnit.Millimeters)
        return float(value.getValue())
    except:
        try:
            return float(value)
        except:
            return None


def infer_part_height_mm(part):
    if part is None:
        return 0.0
    try:
        if part.isPartHeightUnknown():
            return 0.0
    except:
        pass
    for method_name in ["getHeight", "getHeightForSafeZ"]:
        try:
            height = length_to_mm(getattr(part, method_name)())
            if height is not None and height >= 0.0:
                return height
        except:
            pass
    return 0.0


def derive_storage_z_fields(cfg, part=None):
    storage = cfg.get("die_storage_location_xyz", {})
    cfg["_storage_base_z_mm"] = float(storage.get("z", 0.0))
    cfg["_part_height_z_mm"] = infer_part_height_mm(part)
    return cfg


def validate_config(cfg):
    for key in [
        "part_id_or_name", "nozzle_name", "camera_name",
        "allow_machine_writes",
        "die_storage_location_xyz", "cal_work_location_xyz",
        "safe_travel_z",
        "calibration_nozzle_motion_strategy", "storage_nozzle_motion_strategy",
        "calibration_local_clearance_mm", "storage_local_clearance_mm",
        "split_rotation_from_xy_moves",
        "max_pick_descent_mm", "z_clearance_before_pick_mm",
        "lock_xy_to_configured_location", "dry_run_motion_mode",
        "use_high_level_nozzle_pick", "allow_vacuum_only_pick_without_part_state",
        "require_visual_lock_before_pick", "verify_lock_after_centering",
        "storage_camera_z_offset", "cal_camera_z_offset",
        "test_iterations", "verification_iterations", "max_retries_per_step",
        "circle_detection_min_count", "orientation_result_stage_name",
        "orientation_circle_from_results", "orientation_circle_max_square_size_ratio",
        "require_orientation_circle", "orientation_x_sign", "orientation_snap_to_cardinal",
        "orientation_sign_probe_enabled", "orientation_sign_probe_degrees",
        "orientation_sign_probe_max_error_deg", "auto_update_orientation_x_sign",
        "orientation_max_error_deg", "orientation_validation_mode",
        "orientation_continuous_max_error_deg",
        "abort_on_large_computed_offset", "spiral_search",
        "align_nozzle_to_detected_die_rotation", "calibration_step_mm", "motion_speed",
        "calibration_moves", "pivot_probe_enabled", "pivot_probe_angles_deg",
        "pivot_probe_xy_mm", "use_pivot_correction",
        "storage_return_use_correction_only_if_verified",
        "final_storage_return_mode", "final_storage_offset_source",
        "manual_final_storage_offset_mm", "final_return_pick_max_centering_error_mm",
        "final_pickup_apply_pick_vector", "final_pickup_pick_vector_sign",
        "final_pickup_max_correction_mm",
        "auto_update_manual_storage_offset_from_confirm",
        "confirm_shift_descend_to_surface",
        "storage_closed_loop_max_passes",
        "storage_closed_loop_tolerance_mm", "storage_closed_loop_tolerance_deg",
        "verification_pass_max_offset_mm", "verification_pass_max_theta_deg",
        "max_storage_return_command_offset_mm",
        "use_openpnp_safe_z_for_pick_place"
    ]:
        require_key(cfg, key)

    for key in ["x", "y", "z"]:
        require_key(cfg["die_storage_location_xyz"], key)
        require_key(cfg["cal_work_location_xyz"], key)

    if int(cfg["circle_detection_min_count"]) != 4:
        raise Exception("circle_detection_min_count must be 4 for this 4-circle die routine.")
    if float(cfg.get("orientation_max_error_deg", 25.0)) <= 0.0:
        raise Exception("orientation_max_error_deg must be > 0.")
    if str(cfg.get("orientation_validation_mode", "auto")).lower() not in ["auto", "cardinal", "continuous"]:
        raise Exception("orientation_validation_mode must be auto, cardinal, or continuous.")
    if float(cfg.get("orientation_continuous_max_error_deg", 5.0)) <= 0.0:
        raise Exception("orientation_continuous_max_error_deg must be > 0.")

    if cfg.get("pick_place_xy_mode", "detected_center") not in ["configured", "detected_center"]:
        raise Exception("pick_place_xy_mode must be 'configured' or 'detected_center'.")
    allowed_nozzle_strategies = ["machine_safe_split", "local_clearance_split"]
    if str(cfg.get("calibration_nozzle_motion_strategy", "local_clearance_split")).lower() not in allowed_nozzle_strategies:
        raise Exception("calibration_nozzle_motion_strategy must be machine_safe_split or local_clearance_split.")
    if str(cfg.get("storage_nozzle_motion_strategy", "machine_safe_split")).lower() not in allowed_nozzle_strategies:
        raise Exception("storage_nozzle_motion_strategy must be machine_safe_split or local_clearance_split.")
    if float(cfg.get("calibration_local_clearance_mm", 0.75)) < 0.0:
        raise Exception("calibration_local_clearance_mm must be >= 0.")
    if float(cfg.get("storage_local_clearance_mm", 0.75)) < 0.0:
        raise Exception("storage_local_clearance_mm must be >= 0.")
    if float(cfg["die_storage_location_xyz"]["z"]) < 0.0:
        raise Exception("die_storage_location_xyz.z must be >= 0.")
    if part_height_z_mm(cfg) < 0.0:
        raise Exception("Selected part height must be >= 0.")
    if float(cfg["max_pick_descent_mm"]) <= 0.0:
        raise Exception("max_pick_descent_mm must be > 0.")
    if float(cfg["z_clearance_before_pick_mm"]) < 0.0:
        raise Exception("z_clearance_before_pick_mm must be >= 0.")
    if int(cfg["test_iterations"]) < 1:
        raise Exception("test_iterations must be >= 1.")
    if int(cfg["verification_iterations"]) < 1:
        raise Exception("verification_iterations must be >= 1.")
    if float(cfg.get("calibration_step_mm", 0.0)) < 0.0:
        raise Exception("calibration_step_mm must be >= 0.")
    if float(cfg.get("motion_speed", 1.0)) <= 0.0:
        raise Exception("motion_speed must be > 0.")
    if len(cfg.get("calibration_moves", [])) < 1:
        raise Exception("calibration_moves must contain at least one XY offset.")
    if len(cfg.get("angles_deg", [])) < 1:
        raise Exception("angles_deg must contain at least one rotation angle.")
    if len(cfg.get("orientation_sign_probe_degrees", [])) < 2:
        raise Exception("orientation_sign_probe_degrees must contain at least two angle deltas.")
    if float(cfg.get("orientation_sign_probe_max_error_deg", 3.0)) <= 0.0:
        raise Exception("orientation_sign_probe_max_error_deg must be > 0.")
    if len(cfg.get("pivot_probe_angles_deg", [])) < 8:
        raise Exception("pivot_probe_angles_deg must contain at least eight mixed cardinal/non-cardinal angles.")
    require_key(cfg["pivot_probe_xy_mm"], "x")
    require_key(cfg["pivot_probe_xy_mm"], "y")
    if float(cfg.get("verification_pass_max_offset_mm", 0.20)) <= 0.0:
        raise Exception("verification_pass_max_offset_mm must be > 0.")
    if float(cfg.get("verification_pass_max_theta_deg", 1.0)) <= 0.0:
        raise Exception("verification_pass_max_theta_deg must be > 0.")
    if float(cfg.get("final_return_pick_max_centering_error_mm", 0.75)) <= 0.0:
        raise Exception("final_return_pick_max_centering_error_mm must be > 0.")
    if abs(float(cfg.get("final_pickup_pick_vector_sign", 1.0))) <= 0.0:
        raise Exception("final_pickup_pick_vector_sign must be non-zero.")
    if float(cfg.get("final_pickup_max_correction_mm", 1.5)) <= 0.0:
        raise Exception("final_pickup_max_correction_mm must be > 0.")
    if str(cfg.get("final_storage_return_mode", "simple_configured_pocket_return")).lower() not in ["simple_configured_pocket_return", "calibrated_result", "raw_configured"]:
        raise Exception("final_storage_return_mode must be simple_configured_pocket_return, calibrated_result, or raw_configured.")
    require_key(cfg["manual_final_storage_offset_mm"], "x")
    require_key(cfg["manual_final_storage_offset_mm"], "y")
    if bool(cfg.get("storage_closed_loop_return_enabled", False)):
        log("Safety", "storage_closed_loop_return_enabled is deprecated for final storage return; final storage return is one-shot.")
    if int(cfg.get("storage_closed_loop_max_passes", 2)) < 1:
        raise Exception("storage_closed_loop_max_passes must be >= 1.")
    if float(cfg.get("storage_closed_loop_tolerance_mm", 0.15)) <= 0.0:
        raise Exception("storage_closed_loop_tolerance_mm must be > 0.")
    if float(cfg.get("storage_closed_loop_tolerance_deg", 2.0)) <= 0.0:
        raise Exception("storage_closed_loop_tolerance_deg must be > 0.")
    storage_surface_z = storage_pick_surface_z_mm(cfg)
    cal_surface_z = pick_surface_z_mm(loc_from_xyz(cfg["cal_work_location_xyz"]), cfg)
    if float(cfg["safe_travel_z"]) <= float(storage_surface_z):
        raise Exception("safe_travel_z must be above Die Storage Z + selected part height.")
    if float(cfg["safe_travel_z"]) <= float(cal_surface_z):
        raise Exception("safe_travel_z must be above Calibration Work Z + selected part height.")
    if float(cfg["safe_travel_z"]) <= float(cfg["die_storage_location_xyz"]["z"]):
        raise Exception("safe_travel_z must be above die_storage_location_xyz.z.")
    if float(cfg["safe_travel_z"]) <= float(cfg["cal_work_location_xyz"]["z"]):
        raise Exception("safe_travel_z must be above cal_work_location_xyz.z.")

    spiral = cfg["spiral_search"]
    for key in ["enabled", "start_radius_mm", "radius_step_mm", "max_radius_mm", "angle_step_deg", "settle_ms"]:
        require_key(spiral, key)

    if float(spiral["radius_step_mm"]) <= 0.0:
        raise Exception("spiral_search.radius_step_mm must be > 0.")
    if float(spiral["angle_step_deg"]) <= 0.0:
        raise Exception("spiral_search.angle_step_deg must be > 0.")


def mm_loc(x, y, z, r):
    return Location(LengthUnit.Millimeters, float(x), float(y), float(z), float(r))


def loc_from_xyz(data):
    return mm_loc(
        data.get("x", 0.0),
        data.get("y", 0.0),
        data.get("z", 0.0),
        data.get("rotation", 0.0)
    )


def wait_still(movable):
    try:
        movable.waitForCompletion(CompletionType.WaitForStillstand)
    except:
        pass


def move_tool_safe(movable, target, tag):
    check_cancel(tag)
    target = target.convertToUnits(LengthUnit.Millimeters)
    log(tag, "Safe move %s -> X=%.4f Y=%.4f Z=%.4f R=%.4f" %
        (object_name(movable), target.getX(), target.getY(), target.getZ(), target.getRotation()))
    MovableUtils.moveToLocationAtSafeZ(movable, target)
    wait_still(movable)
    check_cancel(tag)


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


def same_named_object(a, b):
    if a is None or b is None:
        return False
    if a == b:
        return True
    return object_name(a) == object_name(b)


def current_nozzle_tip_name(nozzle):
    try:
        return object_name(nozzle.getNozzleTip())
    except:
        return "None"


def get_compatible_nozzle_tips(nozzle, part):
    candidates = []
    pkg = None
    try:
        pkg = part.getPackage()
    except:
        pkg = None

    sources = []
    if pkg is not None:
        sources.append((pkg, "getCompatibleNozzleTips"))
        sources.append((pkg, "getNozzleTips"))
    sources.append((part, "getCompatibleNozzleTips"))

    for obj, method_name in sources:
        try:
            values = getattr(obj, method_name)()
            for tip in java_list_to_py(values):
                if tip is not None:
                    candidates.append(tip)
        except:
            pass

    try:
        values = nozzle.getCompatibleNozzleTips(part)
        for tip in java_list_to_py(values):
            if tip is not None:
                candidates.append(tip)
    except:
        pass

    unique = []
    for tip in candidates:
        found = False
        for existing in unique:
            if same_named_object(existing, tip):
                found = True
                break
        if not found:
            unique.append(tip)
    return unique


def prepare_nozzle_for_part(nozzle, part):
    current_tip = None
    try:
        current_tip = nozzle.getNozzleTip()
    except:
        current_tip = None

    compatible = get_compatible_nozzle_tips(nozzle, part)
    if not compatible:
        raise Exception("No compatible nozzle tip found for part '%s'." % part.getId())

    for tip in compatible:
        if same_named_object(current_tip, tip):
            log("Tooling", "Selected nozzle tip '%s' is compatible with part '%s'." %
                (object_name(current_tip), part.getId()))
            return current_tip

    selected = compatible[0]
    log("Tooling", "Changing nozzle tip from '%s' to compatible tip '%s' for part '%s'." %
        (object_name(current_tip), object_name(selected), part.getId()))
    try:
        nozzle.loadNozzleTip(selected)
    except Exception, first_error:
        try:
            nozzle.setNozzleTip(selected)
        except Exception, second_error:
            raise Exception("Could not load/select compatible nozzle tip '%s' for part '%s': %s | %s" %
                            (object_name(selected), part.getId(), first_error, second_error))
    wait_still(nozzle)
    log("Tooling", "Selected nozzle tip '%s'." % object_name(selected))
    return selected


def part_id(part):
    try:
        return str(part.getId())
    except:
        return object_name(part)


def do_place(nozzle, part=None, cfg=None):
    if cfg is None:
        cfg = {}

    # The script has already descended to the placement Z. Release pneumatics
    # here directly so high-level place sequencing cannot move before release.
    release_part_at_current_z(nozzle, cfg)
    log("Place", "Manual pneumatic release completed at placement Z.")
    return "manual_pneumatic_release"


def find_feeder_by_name(machine_obj, feeder_name):
    if feeder_name is None or str(feeder_name).strip() == "":
        return None
    try:
        for feeder in java_list_to_py(machine_obj.getFeeders()):
            if object_name(feeder) == str(feeder_name):
                return feeder
    except:
        pass
    raise Exception("Feeder not found by name: %s" % feeder_name)


def do_pick(nozzle, part, feeder=None, cfg=None):
    if cfg is None:
        cfg = {}
    before_loc = None
    try:
        before_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        log("Pick", "Before pick actuation nozzle X=%.4f Y=%.4f Z=%.4f R=%.4f" %
            (before_loc.getX(), before_loc.getY(), before_loc.getZ(), before_loc.getRotation()))
    except:
        pass

    use_high_level = bool(cfg.get("use_high_level_nozzle_pick", True))
    vacuum_only_allowed = bool(cfg.get("allow_vacuum_only_pick_without_part_state", False))
    log("Pick", "use_high_level_nozzle_pick=%s" % str(bool(use_high_level)))
    log("Pick", "vacuum_only_manual_pick_allowed=%s" % str(bool(vacuum_only_allowed)))

    if not use_high_level:
        part_state_ok = False
        feeder_state_ok = False
        part_on_sensor_verified = False
        try:
            nozzle.setPart(part)
            part_state_ok = True
            log("Pick", "Manual nozzle part state set to '%s' before vacuum actuation." % part_id(part))
        except Exception, e:
            log("Pick", "Could not set nozzle part state: %s" % e)
        try:
            nozzle.setPartsFeeder(feeder)
            feeder_state_ok = True
            log("Pick", "Manual nozzle feeder state set to '%s' before vacuum actuation." % object_name(feeder))
        except Exception, e:
            log("Pick", "Could not set nozzle feeder state: %s" % e)
        log("Pick", "manual_part_state_supported=%s" % str(bool(part_state_ok)))
        log("Pick", "manual_feeder_state_supported=%s" % str(bool(feeder_state_ok)))
        set_vacuum_state(nozzle, cfg, True, True)
        sleep_ms(cfg.get("vacuum_settle_ms", 150))
        try:
            if nozzle.isPartOnEnabled(SpiNozzle.PartOnStep.AfterPick):
                if not nozzle.isPartOn():
                    raise Exception("Part-on sensor did not detect a part after manual vacuum pick.")
                part_on_sensor_verified = True
                log("Pick", "Part-on sensor verified manual vacuum pick.")
            else:
                log("Pick", "Part-on sensor check after manual vacuum pick is disabled.")
        except Exception, e:
            log("Pick", "Part-on sensor manual pick check did not verify pickup: %s" % e)
        if not part_state_ok and not vacuum_only_allowed and not part_on_sensor_verified:
            raise Exception("Manual pick mode is unsafe: nozzle does not support setPart/setPartsFeeder, and allow_vacuum_only_pick_without_part_state is false. Enable use_high_level_nozzle_pick or configure a verified manual pick path.")
        try:
            after_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            log("Pick", "After manual pick actuation nozzle X=%.4f Y=%.4f Z=%.4f R=%.4f" %
                (after_loc.getX(), after_loc.getY(), after_loc.getZ(), after_loc.getRotation()))
        except:
            pass
        log("Pick", "physical_pick_path=manual_vacuum_part_state")
        return "manual_vacuum_part_state"

    log("Pick", "manual_part_state_supported=not_applicable")
    log("Pick", "manual_feeder_state_supported=not_applicable")
    set_vacuum_state(nozzle, cfg, True, True)
    sleep_ms(cfg.get("vacuum_settle_ms", 150))
    try:
        nozzle.pick(part, feeder)
        log("Pick", "nozzle.pick(part, feeder) succeeded")
        try:
            after_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            log("Pick", "After high-level pick actuation nozzle X=%.4f Y=%.4f Z=%.4f R=%.4f" %
                (after_loc.getX(), after_loc.getY(), after_loc.getZ(), after_loc.getRotation()))
        except:
            pass
        log("Pick", "physical_pick_path=manual_vacuum_plus_nozzle.pick")
        return "manual_vacuum_plus_nozzle.pick"
    except Exception, first_error:
        log("Pick", "nozzle.pick(part, feeder) failed: %s" % first_error)
        try:
            nozzle.setPart(part)
        except:
            pass
        try:
            nozzle.setPartsFeeder(feeder)
        except:
            pass
        try:
            after_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            log("Pick", "After high-level pick failure fallback nozzle X=%.4f Y=%.4f Z=%.4f R=%.4f" %
                (after_loc.getX(), after_loc.getY(), after_loc.getZ(), after_loc.getRotation()))
        except:
            pass
        log("Pick", "Continuing with manual vacuum already ON.")
        log("Pick", "physical_pick_path=manual_vacuum_after_nozzle.pick_failure")
        return "manual_vacuum_after_nozzle.pick_failure"
    except Throwable, throwable:
        log("Pick", "nozzle.pick(part, feeder) threw non-Exception Throwable: %s" % throwable)
        try:
            nozzle.setPart(part)
        except:
            pass
        try:
            nozzle.setPartsFeeder(feeder)
        except:
            pass
        try:
            after_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            log("Pick", "After high-level pick Throwable fallback nozzle X=%.4f Y=%.4f Z=%.4f R=%.4f" %
                (after_loc.getX(), after_loc.getY(), after_loc.getZ(), after_loc.getRotation()))
        except:
            pass
        log("Pick", "Continuing with manual vacuum already ON after Throwable.")
        log("Pick", "physical_pick_path=manual_vacuum_after_nozzle.pick_throwable")
        return "manual_vacuum_after_nozzle.pick_throwable"


def verify_part_on_after_pick(nozzle):
    try:
        if nozzle.isPartOnEnabled(SpiNozzle.PartOnStep.AfterPick):
            if not nozzle.isPartOn():
                raise Exception("Part-on sensor did not detect a part after pick.")
            log("Pick", "Part-on sensor verified after pick.")
        else:
            log("Pick", "Part-on sensor check after pick is disabled; continuing.")
    except Exception, e:
        raise


def verify_part_off_after_place(nozzle):
    try:
        if nozzle.isPartOffEnabled(SpiNozzle.PartOffStep.AfterPlace):
            if not nozzle.isPartOff():
                raise Exception("Part-off sensor did not verify release after place.")
            log("Place", "Part-off sensor verified after place.")
        else:
            log("Place", "Part-off sensor check after place is disabled; continuing.")
    except Exception, e:
        raise


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


def find_actuator_for_nozzle(nozzle, actuator_name):
    head = None
    try:
        head = nozzle.getHead()
    except:
        head = None
    try:
        return find_actuator_by_name(head, actuator_name)
    except:
        return find_actuator_by_name(get_machine(), actuator_name)


def set_actuator_state_by_name(machine_or_head, actuator_name, on_off):
    if actuator_name is None or str(actuator_name).strip() == "":
        return False
    actuator = find_actuator_by_name(machine_or_head, actuator_name)
    actuator.actuate(bool(on_off))
    log("Actuator", "%s -> %s" % (actuator_name, "ON" if bool(on_off) else "OFF"))
    return True


def set_actuator_state(actuator, on_off):
    if actuator is None:
        return False
    if hasattr(actuator, "actuate"):
        actuator.actuate(bool(on_off))
    elif hasattr(actuator, "setActuated"):
        actuator.setActuated(bool(on_off))
    elif hasattr(actuator, "setOn"):
        actuator.setOn(bool(on_off))
    else:
        raise Exception("Resolved object '%s' has no supported actuator method." % object_name(actuator))
    log("Actuator", "%s -> %s" % (object_name(actuator), "ON" if bool(on_off) else "OFF"))
    return True


def actuator_debug_methods(obj):
    if obj is None:
        return "null"
    out = []
    try:
        for m in obj.getClass().getMethods():
            name = str(m.getName())
            low = name.lower()
            if "actuat" in low or "vac" in low or "blow" in low or "air" in low:
                out.append(name)
    except:
        pass
    out = sorted(list(set(out)))
    return ",".join(out[:25])


def try_set_actuator_state_by_name(machine_or_head, actuator_name, on_off, fatal=False):
    try:
        return set_actuator_state_by_name(machine_or_head, actuator_name, on_off)
    except Exception, e:
        log("Actuator", "Failed to set %s -> %s: %s" %
            (actuator_name, "ON" if bool(on_off) else "OFF", e))
        if fatal:
            raise
    return False


def resolve_assigned_actuator(nozzle, cfg, kind):
    cfg_key = "force_vacuum_actuator_name" if kind == "vacuum" else "force_blowoff_actuator_name"
    configured = cfg.get(cfg_key, None)
    if configured is not None and str(configured).strip() != "":
        return find_actuator_for_nozzle(nozzle, str(configured))

    method_names = []
    fallback_names = []
    if kind == "vacuum":
        method_names = ["getVacuumActuator", "getVacuumValveActuator"]
        fallback_names = ["VAC1", "Vacuum"]
    else:
        method_names = ["getBlowOffActuator", "getBlowoffActuator"]
        fallback_names = ["Nitrogen", "Blowoff", "Blow Off"]

    nozzle_tip = None
    try:
        nozzle_tip = nozzle.getNozzleTip()
    except:
        nozzle_tip = None

    # Prefer actuator attached to the loaded nozzle tip.
    if nozzle_tip is not None:
        for method_name in method_names:
            try:
                actuator = getattr(nozzle_tip, method_name)()
                if actuator is not None:
                    return actuator
            except:
                pass

    for method_name in method_names:
        try:
            actuator = getattr(nozzle, method_name)()
            if actuator is not None:
                return actuator
        except:
            pass
        try:
            actuator = getattr(nozzle.getHead(), method_name)()
            if actuator is not None:
                return actuator
        except:
            pass

    # Some ReferenceNozzle builds expose assigned actuators as JavaBean
    # properties instead of public interface methods.
    for attr_name in ["vacuumActuator", "vacuumValveActuator"] if kind == "vacuum" else ["blowOffActuator", "blowoffActuator"]:
        try:
            actuator = getattr(nozzle, attr_name)
            if actuator is not None:
                return actuator
        except:
            pass

    head = None
    try:
        head = nozzle.getHead()
    except:
        head = None

    for name in fallback_names:
        try:
            actuator = find_actuator_by_name(head, name)
            if actuator is not None:
                return actuator
        except:
            pass
    return None


def set_vacuum_state(nozzle, cfg, on_off, fatal=True):
    actuator = resolve_assigned_actuator(nozzle, cfg, "vacuum")
    if actuator is None:
        if fatal:
            raise Exception("Vacuum actuator is not configured and could not be auto-resolved from nozzle/head.")
        return False
    path = "resolved_actuator:%s" % object_name(actuator)
    try:
        set_actuator_state(actuator, on_off)
    except Exception, e:
        tip = None
        try:
            tip = nozzle.getNozzleTip()
        except:
            tip = None
        log("Actuator", "Vacuum actuation failed. actuator=%s class=%s methods=%s tip=%s tip_methods=%s nozzle_methods=%s" %
            (object_name(actuator), actuator.getClass().getName(), actuator_debug_methods(actuator),
             object_name(tip), actuator_debug_methods(tip), actuator_debug_methods(nozzle)))
        if fatal:
            raise
        return False
    log("Vacuum", "path=%s state=%s" % (path, "ON" if bool(on_off) else "OFF"))
    return True


def set_blowoff_state(nozzle, cfg, on_off, fatal=False):
    actuator = resolve_assigned_actuator(nozzle, cfg, "blowoff")
    if actuator is None:
        if fatal:
            raise Exception("Blowoff actuator is not configured and could not be auto-resolved from nozzle/head.")
        return False
    path = "resolved_actuator:%s" % object_name(actuator)
    try:
        set_actuator_state(actuator, on_off)
    except Exception, e:
        log("Actuator", "Blowoff actuation failed. actuator=%s class=%s methods=%s error=%s" %
            (object_name(actuator), actuator.getClass().getName(), actuator_debug_methods(actuator), e))
        if fatal:
            raise
        return False
    log("Blowoff", "path=%s state=%s" % (path, "ON" if bool(on_off) else "OFF"))
    return True


def release_part_at_current_z(nozzle, cfg):
    try:
        nozzle.setPart(None)
    except:
        pass
    try:
        nozzle.setPartsFeeder(None)
    except:
        pass

    released = set_vacuum_state(nozzle, cfg, False, False)
    if not released:
        head = None
        try:
            head = nozzle.getHead()
        except:
            head = None

        for name in ["VAC1", "Vacuum"]:
            try:
                if try_set_actuator_state_by_name(head, name, False, False):
                    released = True
                    break
            except:
                pass
    if not released:
        raise Exception("Failed to release vacuum: no vacuum OFF actuator path succeeded.")

    blowoff_ms = int(cfg.get("blowoff_pulse_ms", 120))
    if not bool(cfg.get("enable_blowoff_on_place", False)):
        blowoff_ms = 0
    if blowoff_ms > 0:
        if set_blowoff_state(nozzle, cfg, True, False):
            sleep_ms(blowoff_ms)
            set_blowoff_state(nozzle, cfg, False, False)
        else:
            log("Blowoff", "No blowoff actuator resolved; vacuum OFF was still commanded.")
    return True


def sleep_ms(ms):
    if int(ms) > 0:
        check_cancel()
        Thread.sleep(int(ms))


def motion_speed(cfg):
    return float(cfg.get("motion_speed", 1.0))


def move_to_with_speed(movable, target, cfg):
    target = target.convertToUnits(LengthUnit.Millimeters)
    speed = motion_speed(cfg)
    try:
        movable.moveTo(target, speed)
    except TypeError:
        movable.moveTo(target)
    wait_still(movable)


def nozzle_motion_strategy_for_context(cfg, context):
    if context == "Storage":
        return str(cfg.get("storage_nozzle_motion_strategy", "machine_safe_split")).lower()
    if context == "Cal":
        return str(cfg.get("calibration_nozzle_motion_strategy", "local_clearance_split")).lower()
    return str(cfg.get("calibration_nozzle_motion_strategy", "local_clearance_split")).lower()


def nozzle_local_clearance_for_context(cfg, context):
    if context == "Storage":
        return float(cfg.get("storage_local_clearance_mm", 0.75))
    return float(cfg.get("calibration_local_clearance_mm", 0.75))


def nozzle_clearance_z(current, final_loc, cfg, context, strategy):
    if strategy == "machine_safe_split":
        return float(cfg["safe_travel_z"])
    if strategy == "local_clearance_split":
        return max(float(current.getZ()),
                   float(final_loc.getZ()) + nozzle_local_clearance_for_context(cfg, context))
    raise Exception("Unsupported nozzle motion strategy: %s" % strategy)


def log_pickflow_phase(tag, phase, loc):
    loc = loc.convertToUnits(LengthUnit.Millimeters)
    log(tag, "phase=%s -> X=%.4f Y=%.4f Z=%.4f R=%.4f" %
        (phase, loc.getX(), loc.getY(), loc.getZ(), loc.getRotation()))


def _move_nozzle_split_strategy(nozzle, final_loc, cfg, tag, context, strategy):
    final_loc = final_loc.convertToUnits(LengthUnit.Millimeters)
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    clearance_z = nozzle_clearance_z(current, final_loc, cfg, context, strategy)
    log(tag, "context=%s strategy=%s clearance_z=%.4f" %
        (context, strategy, clearance_z))

    lift = mm_loc(current.getX(), current.getY(), clearance_z, current.getRotation())
    lift = nearest_nozzle_rotation_location(nozzle, lift, tag)
    log_pickflow_phase(tag, "lift_to_clearance", lift)
    move_to_with_speed(nozzle, lift, cfg)
    wait_still(nozzle)

    after_lift = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    final_r = nearest_equivalent_angle(final_loc.getRotation(), after_lift.getRotation())
    if bool(cfg.get("split_rotation_from_xy_moves", True)):
        rotate = mm_loc(after_lift.getX(), after_lift.getY(), clearance_z, final_r)
        log_pickflow_phase(tag, "rotate_only", rotate)
        move_to_with_speed(nozzle, rotate, cfg)
        wait_still(nozzle)

        after_rotate = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        xy = mm_loc(final_loc.getX(), final_loc.getY(), clearance_z, after_rotate.getRotation())
        log_pickflow_phase(tag, "xy_only", xy)
        move_to_with_speed(nozzle, xy, cfg)
        wait_still(nozzle)
    else:
        xy = mm_loc(final_loc.getX(), final_loc.getY(), clearance_z, final_r)
        log_pickflow_phase(tag, "xy_r", xy)
        move_to_with_speed(nozzle, xy, cfg)
        wait_still(nozzle)

    after_xy = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    vertical = mm_loc(after_xy.getX(), after_xy.getY(), final_loc.getZ(),
                      after_xy.getRotation())
    log_pickflow_phase(tag, "z_only", vertical)
    move_to_with_speed(nozzle, vertical, cfg)
    wait_still(nozzle)


def move_nozzle_split(nozzle, final_loc, cfg, tag, context):
    strategy = nozzle_motion_strategy_for_context(cfg, context)
    return _move_nozzle_split_strategy(nozzle, final_loc, cfg, tag, context, strategy)


def move_nozzle_xy_r_then_z(nozzle, final_loc, cfg, tag):
    return _move_nozzle_split_strategy(nozzle, final_loc, cfg, tag,
                                       "Storage", "machine_safe_split")


def lift_nozzle_after_pick_place(nozzle, cfg, tag, context, final_z):
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    strategy = nozzle_motion_strategy_for_context(cfg, context)
    if strategy == "machine_safe_split":
        lift_z = float(cfg["safe_travel_z"])
    elif strategy == "local_clearance_split":
        lift_z = max(float(current.getZ()),
                     float(final_z) + nozzle_local_clearance_for_context(cfg, context))
    else:
        raise Exception("Unsupported nozzle motion strategy: %s" % strategy)
    lift = mm_loc(current.getX(), current.getY(), lift_z, current.getRotation())
    lift = nearest_nozzle_rotation_location(nozzle, lift, tag)
    log(tag, "context=%s strategy=%s post_action_lift_z=%.4f" %
        (context, strategy, lift_z))
    log_pickflow_phase(tag, "post_action_lift", lift)
    move_to_with_speed(nozzle, lift, cfg)
    wait_still(nozzle)


def safe_staged_move(movable, target, safe_z, tag):
    # The configured OpenPnP safe-Z model owns lift/XY/drop sequencing.
    move_tool_safe(movable, target, tag)


def move_safe_then_xy_then_z(movable, target, safe_z, tag):
    return safe_staged_move(movable, target, safe_z, tag)


def set_nozzle_angle(nozzle, angle_deg):
    check_cancel("Move")
    loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    target = mm_loc(loc.getX(), loc.getY(), loc.getZ(), float(angle_deg))
    target = nearest_nozzle_rotation_location(nozzle, target, "Move")
    nozzle.moveTo(target)
    wait_still(nozzle)


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


def find_nozzle(head, name):
    noz = None
    if name:
        try:
            noz = head.getNozzleByName(name)
        except:
            noz = None
    if noz is None:
        noz = head.getDefaultNozzle()
    if noz is None:
        raise Exception("Nozzle not found: %s" % name)
    return noz


def find_top_camera(machine_obj, head, camera_name):
    if camera_name:
        try:
            cams = head.getCameras()
            for cam in cams:
                if cam.getName() == camera_name:
                    return cam
        except:
            pass
        try:
            cams = machine_obj.getCameras()
            for cam in cams:
                if cam.getName() == camera_name:
                    return cam
        except:
            pass

    try:
        cam = head.getDefaultCamera()
        if cam is not None:
            if not camera_name or cam.getName() == camera_name:
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
    cfg = get_config()
    try:
        part = cfg.getPart(part_id_or_name)
        if part is not None:
            return part
    except:
        pass
    try:
        for part in cfg.getParts():
            try:
                if part.getId() == part_id_or_name or part.getName() == part_id_or_name:
                    return part
            except:
                pass
    except:
        pass
    raise Exception("Part not found by id/name: %s" % part_id_or_name)


def get_part_pipeline(part):
    settings = None
    try:
        settings = part.getFiducialVisionSettings()
    except:
        settings = None
    if settings is None:
        raise Exception("Part '%s' has no FiducialVisionSettings." % part.getId())
    pipeline = settings.getPipeline()
    if pipeline is None:
        raise Exception("Part '%s' fiducial pipeline is null." % part.getId())
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
        return float(radius)
    diameter = number_attr(item, ["diameter", "getDiameter", "size", "getSize"])
    if diameter is not None:
        return float(diameter) / 2.0
    return None


def radius_from_dict(item):
    for key in ["radius_px", "radius"]:
        if key in item:
            return float(item[key])
    for key in ["diameter_px", "diameter", "size_px", "size"]:
        if key in item:
            return float(item[key]) / 2.0
    return None


def point_from_model_item(item, camera):
    # OpenCV Point, KeyPoint, Circle-like, RotatedRect, Location, or dict.
    try:
        if isinstance(item, dict):
            if "x_mm" in item and "y_mm" in item:
                p = {"x_mm": float(item["x_mm"]), "y_mm": float(item["y_mm"])}
                radius = radius_from_dict(item)
                if radius is not None:
                    p["radius_px"] = radius
                return p
            if "x" in item and "y" in item:
                p = pixel_to_machine_offsets(camera, float(item["x"]), float(item["y"]))
                radius = radius_from_dict(item)
                if radius is not None:
                    p["radius_px"] = radius
                return p
    except:
        pass

    try:
        if hasattr(item, "getX") and hasattr(item, "getY"):
            if isinstance(item, Location):
                return {"x_mm": float(item.getX()), "y_mm": float(item.getY())}
    except:
        pass

    center = None
    try:
        center = item.center
    except:
        center = None
    if center is not None:
        try:
            p = image_point_to_camera_mm(camera, center)
            radius = radius_from_model_item(item)
            if radius is not None:
                p["radius_px"] = float(radius)
            return p
        except:
            pass

    pt = None
    try:
        pt = item.pt
    except:
        pt = None
    if pt is not None:
        try:
            p = image_point_to_camera_mm(camera, pt)
            radius = radius_from_model_item(item)
            if radius is not None:
                p["radius_px"] = float(radius)
            return p
        except:
            return pixel_to_camera_mm(camera, float(pt.x), float(pt.y), True)

    x = number_attr(item, ["x", "getX"])
    y = number_attr(item, ["y", "getY"])
    if x is not None and y is not None:
        try:
            p = image_point_to_camera_mm(camera, item)
        except:
            p = pixel_to_camera_mm(camera, x, y, True)
        radius = radius_from_model_item(item)
        if radius is not None:
            p["radius_px"] = float(radius)
        return p

    return None


def camera_units_per_pixel(camera):
    upp = None
    try:
        upp = camera.getUnitsPerPixelAtZ()
    except:
        pass
    if upp is None:
        try:
            upp = camera.getUnitsPerPixel()
        except:
            upp = None
    if upp is None:
        return None
    try:
        upp = upp.convertToUnits(LengthUnit.Millimeters)
    except:
        pass
    return upp


def camera_image_dimensions(camera):
    w = None
    h = None
    for name in ["getWidth", "getImageWidth"]:
        try:
            w = float(getattr(camera, name)())
            break
        except:
            pass
    for name in ["getHeight", "getImageHeight"]:
        try:
            h = float(getattr(camera, name)())
            break
        except:
            pass
    return w, h


def save_last_vision_image(image):
    global _last_vision_image_path, _last_vision_image_size
    if image is None:
        return
    try:
        path = last_image_path()
        ImageIO.write(image, "png", File(path))
        _last_vision_image_path = path
        _last_vision_image_size = {
            "width": int(image.getWidth()),
            "height": int(image.getHeight())
        }
    except Exception, e:
        log("Vision", "Could not save last captured image: %s" % e)


def save_pipeline_or_capture_image(pipeline, capture_image):
    image = capture_image
    if image is None:
        try:
            mat = pipeline.getWorkingImage()
            if mat is not None and not mat.empty():
                image = OpenCvUtils.toBufferedImage(mat)
        except:
            image = None
    save_last_vision_image(image)


def image_center_px(camera):
    w = None
    h = None
    if _last_vision_image_size is not None:
        w = float(_last_vision_image_size.get("width", 0))
        h = float(_last_vision_image_size.get("height", 0))
    if w is None or h is None or w <= 0.0 or h <= 0.0:
        w, h = camera_image_dimensions(camera)
    if w is None or h is None or w <= 0.0 or h <= 0.0:
        return None
    return {"x": w / 2.0, "y": h / 2.0, "width": w, "height": h}


def expected_center_from_pose(camera, pose):
    if "center_x_px" not in pose or "center_y_px" not in pose:
        return image_center_px(camera)
    upp = camera_units_per_pixel(camera)
    if upp is None:
        return image_center_px(camera)
    try:
        ux = float(upp.getX())
        uy = float(upp.getY())
        if abs(ux) < 0.0000001 or abs(uy) < 0.0000001:
            return image_center_px(camera)
        return {
            "x": float(pose["center_x_px"]) - (float(pose.get("center_x_mm", 0.0)) / ux),
            "y": float(pose["center_y_px"]) - (float(pose.get("center_y_mm", 0.0)) / uy)
        }
    except:
        return image_center_px(camera)


def build_pose_overlay(camera, pose, expected_loc):
    overlay = {
        "image_path": _last_vision_image_path,
        "image_size": _last_vision_image_size,
        "actual_center_px": None,
        "expected_center_px": expected_center_from_pose(camera, pose),
        "actual_corners_px": [],
        "detected_circles_px": [],
        "orientation_circle_px": None,
        "expected_rotation_deg": float(expected_loc.getRotation()),
        "actual_rotation_deg": float(pose.get("rotation_deg", expected_loc.getRotation())),
        "side_mm": float(pose.get("side_mm", 0.0))
    }
    if "center_x_px" in pose and "center_y_px" in pose:
        overlay["actual_center_px"] = {"x": float(pose["center_x_px"]), "y": float(pose["center_y_px"])}
    for p in pose.get("corners", []):
        if "x_px" in p and "y_px" in p:
            item = {"x": float(p["x_px"]), "y": float(p["y_px"])}
            if "radius_px" in p:
                item["radius"] = float(p["radius_px"])
            overlay["actual_corners_px"].append(item)
            overlay["detected_circles_px"].append(item)
    if "orientation_x_px" in pose and "orientation_y_px" in pose:
        overlay["orientation_circle_px"] = {
            "x": float(pose["orientation_x_px"]),
            "y": float(pose["orientation_y_px"])
        }
        if "orientation_radius_px" in pose:
            overlay["orientation_circle_px"]["radius"] = float(pose["orientation_radius_px"])
    return overlay


def save_last_overlay(overlay):
    if overlay is None:
        return
    try:
        f = open(last_overlay_path(), "w")
        try:
            f.write(json.dumps(overlay, sort_keys=True, indent=2))
        finally:
            f.close()
    except Exception, e:
        log("Vision", "Could not save last overlay metadata: %s" % e)


def pixel_to_machine_offsets(camera, px, py):
    offsets = VisionUtils.getPixelCenterOffsets(camera, float(px), float(py))
    offsets = offsets.convertToUnits(LengthUnit.Millimeters)
    return {
        "x_mm": float(offsets.getX()),
        "y_mm": float(offsets.getY()),
        "x_px": float(px),
        "y_px": float(py)
    }


def pixel_to_camera_mm(camera, px, py, assume_image_coords=True):
    px_local = float(px)
    py_local = float(py)
    if assume_image_coords:
        w, h = camera_image_dimensions(camera)
        if w is not None and h is not None and w > 0.0 and h > 0.0:
            px_local = px_local - (w / 2.0)
            py_local = py_local - (h / 2.0)
    upp = camera_units_per_pixel(camera)
    if upp is None:
        return {"x_mm": float(px_local), "y_mm": float(py_local), "x_px": float(px), "y_px": float(py)}
    return {
        "x_mm": float(px_local) * upp.getX(),
        "y_mm": float(py_local) * upp.getY(),
        "x_px": float(px),
        "y_px": float(py)
    }


def image_point_to_camera_mm(camera, point):
    px = number_attr(point, ["x", "getX"])
    py = number_attr(point, ["y", "getY"])
    if px is not None and py is not None:
        try:
            return pixel_to_machine_offsets(camera, float(px), float(py))
        except:
            pass

    try:
        loc = camera.getLocation(point).convertToUnits(LengthUnit.Millimeters)
        cam_loc = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
        return {"x_mm": cam_loc.getX() - loc.getX(), "y_mm": cam_loc.getY() - loc.getY()}
    except:
        if px is None or py is None:
            raise
        return pixel_to_camera_mm(camera, float(px), float(py), True)


def unique_stage_names(names):
    out = []
    for name in names:
        if name is None:
            continue
        name = str(name)
        if name == "":
            continue
        if name not in out:
            out.append(name)
    return out


def extract_stage_points(pipeline, camera, stage_name):
    try:
        result = pipeline.getResult(stage_name)
    except:
        return []
    model = get_model(result)
    points = []
    for item in java_list_to_py(model):
        p = point_from_model_item(item, camera)
        if p is not None:
            points.append(p)
    return points


def point_size_px(point):
    for key in ["radius_px", "diameter_px", "size_px"]:
        try:
            return float(point[key])
        except:
            pass
    return None


def split_circle_stage_points(points, cfg):
    if not bool(cfg.get("orientation_circle_from_results", True)):
        return points, []
    if len(points) < 5:
        return points, []

    sized = []
    for p in points:
        size = point_size_px(p)
        if size is None:
            return points, []
        sized.append((size, p))

    sized = sorted(sized, key=lambda item: item[0])
    orient_size, orient = sized[0]
    square_sized = sized[1:]
    if len(square_sized) < 4:
        return points, []

    square_sizes = [item[0] for item in square_sized[:4]]
    avg_square_size = mean(square_sizes)
    ratio = 1.0
    if avg_square_size > 0.000001:
        ratio = orient_size / avg_square_size
    max_ratio = float(cfg.get("orientation_circle_max_square_size_ratio", 0.8))
    if ratio > max_ratio:
        return points, []

    square_points = [item[1] for item in square_sized[:4]]
    log("Vision", "Split single circle stage by size: orientation_size=%.2f square_avg_size=%.2f ratio=%.3f" %
        (orient_size, avg_square_size, ratio))
    return square_points, [orient]


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

    stage_names = unique_stage_names([
        cfg.get("circle_result_stage_name", "results"),
        "results", "circles", "fiducials", "preResults"
    ])
    orientation_stage = str(cfg.get("orientation_result_stage_name", "OrientCircleResult"))
    require_orientation = bool(cfg.get("require_orientation_circle", True))
    best = []
    best_raw = []
    best_orientation = []
    best_stage = None
    best_orientation_stage = None

    for attempt in range(1, int(cfg["max_retries_per_step"]) + 1):
        check_cancel("Vision")
        try:
            log("Vision", "Pipeline attempt %d" % attempt)
            capture_image = None
            try:
                capture_image = camera.settleAndCapture()
            except:
                sleep_ms(cfg.get("settle_ms", 250))
            pipeline.process()
            save_pipeline_or_capture_image(pipeline, capture_image)

            orientation_points = extract_stage_points(pipeline, camera, orientation_stage)
            if len(orientation_points) > len(best_orientation):
                best_orientation = orientation_points
                best_orientation_stage = orientation_stage

            for stage in stage_names:
                raw_points = extract_stage_points(pipeline, camera, stage)
                circles, split_orientation_points = split_circle_stage_points(raw_points, cfg)
                stage_orientation_points = orientation_points
                stage_orientation_name = orientation_stage
                if len(stage_orientation_points) == 0 and len(split_orientation_points) > 0:
                    stage_orientation_points = split_orientation_points
                    stage_orientation_name = stage
                if len(circles) > len(best):
                    best = circles
                    best_raw = raw_points
                    best_stage = stage
                if len(stage_orientation_points) > len(best_orientation):
                    best_orientation = stage_orientation_points
                    best_orientation_stage = stage_orientation_name
                if len(circles) >= int(cfg["circle_detection_min_count"]) and (
                        len(stage_orientation_points) > 0 or not require_orientation):
                    log("Vision", "Detected %d square circles from stage '%s' (%d raw), %d orientation points from stage '%s'" %
                        (len(circles), stage, len(raw_points), len(stage_orientation_points), stage_orientation_name))
                    return {
                        "circles": circles[:4],
                        "orientation_points": stage_orientation_points,
                        "circle_stage_name": stage,
                        "orientation_stage_name": stage_orientation_name,
                        "raw_circle_stage_count": len(raw_points)
                    }
        except Exception, e:
            log("Vision", "Pipeline attempt %d failed: %s" % (attempt, e))
        sleep_ms(cfg.get("settle_ms", 250))

    log("Vision", "Best circle count was %d from stage '%s'; best orientation count was %d from stage '%s'" %
        (len(best), best_stage, len(best_orientation), best_orientation_stage))
    return {
        "circles": best[:4],
        "orientation_points": best_orientation,
        "circle_stage_name": best_stage,
        "orientation_stage_name": best_orientation_stage,
        "raw_circle_stage_count": len(best_raw)
    }


def spiral_offsets(spiral):
    yield (0.0, 0.0)
    if not bool(spiral.get("enabled", True)):
        return

    r = float(spiral["start_radius_mm"])
    max_r = float(spiral["max_radius_mm"])
    step = float(spiral["radius_step_mm"])
    angle_step = float(spiral["angle_step_deg"])

    while r <= max_r + 0.0000001:
        a = 0.0
        while a < 360.0:
            rad = math.radians(a)
            yield (r * math.cos(rad), r * math.sin(rad))
            a += angle_step
        r += step


def detect_with_spiral(camera, nozzle, part, cfg, expected_camera_loc):
    spiral = cfg["spiral_search"]
    origin = expected_camera_loc.convertToUnits(LengthUnit.Millimeters)
    best_count = 0
    best_orientation_count = 0

    for dx, dy in spiral_offsets(spiral):
        check_cancel("Spiral")
        target = mm_loc(origin.getX() + dx, origin.getY() + dy, origin.getZ(), origin.getRotation())
        if abs(dx) > 0.00001 or abs(dy) > 0.00001:
            log("Spiral", "Search offset dx=%.4f dy=%.4f" % (dx, dy))
        move_safe_then_xy_then_z(camera, target, cfg["safe_travel_z"], "Move")
        sleep_ms(spiral.get("settle_ms", cfg.get("settle_ms", 250)))
        detected = run_fiducial_pipeline_and_extract_pose_points(camera, nozzle, part, cfg)
        circles = detected.get("circles", [])
        orientation_points = detected.get("orientation_points", [])
        if len(circles) > best_count:
            best_count = len(circles)
            log("Vision", "Best circle count now %d" % best_count)
        if len(orientation_points) > best_orientation_count:
            best_orientation_count = len(orientation_points)
            log("Vision", "Best orientation count now %d" % best_orientation_count)
        if len(circles) >= 4 and (len(orientation_points) > 0 or not bool(cfg.get("require_orientation_circle", True))):
            pose = square_fit_offset_solver(circles)
            apply_orientation_circle_to_pose(pose, orientation_points,
                                             detected.get("orientation_stage_name", None), cfg)
            pose["circle_stage_name"] = detected.get("circle_stage_name", None)
            pose["search_dx_mm"] = dx
            pose["search_dy_mm"] = dy
            pose["circle_count"] = len(circles)
            pose["raw_circle_stage_count"] = int(detected.get("raw_circle_stage_count", len(circles)))
            pose["orientation_count"] = len(orientation_points)
            return pose

    if bool(cfg.get("require_orientation_circle", True)):
        raise Exception("Could not detect 4 square circles and orientation circle. Best square count: %d; best orientation count: %d" %
                        (best_count, best_orientation_count))
    raise Exception("Could not detect 4 circles. Best count: %d" % best_count)


def mean(values):
    if not values:
        return 0.0
    return sum(values) / float(len(values))


def median(values):
    if not values:
        return 0.0
    values = sorted(values)
    n = len(values)
    if n % 2 == 1:
        return values[n // 2]
    return (values[n // 2 - 1] + values[n // 2]) / 2.0


def rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum([v * v for v in values]) / float(len(values)))


def robust_center(values):
    if len(values) < 4:
        return mean(values)
    med = median(values)
    dev = [abs(v - med) for v in values]
    mad = median(dev)
    if mad <= 0.000001:
        return med
    kept = []
    limit = 3.0 * 1.4826 * mad
    for v in values:
        if abs(v - med) <= limit:
            kept.append(v)
    if not kept:
        return med
    return mean(kept)


def sort_square_corners(points):
    cx = mean([p["x_mm"] for p in points])
    cy = mean([p["y_mm"] for p in points])

    def angle_key(p):
        return math.atan2(p["y_mm"] - cy, p["x_mm"] - cx)

    return sorted(points, key=angle_key)


def normalize_angle(angle):
    while angle <= -180.0:
        angle += 360.0
    while angle > 180.0:
        angle -= 360.0
    return angle


def nearest_equivalent_angle(target_deg, current_deg):
    delta = normalize_angle(float(target_deg) - float(current_deg))
    return float(current_deg) + delta


def nearest_cardinal_angle(angle_deg):
    angle_deg = normalize_angle(float(angle_deg))
    best = 0.0
    best_error = None
    for candidate in [0.0, 90.0, 180.0, -90.0]:
        error = abs(normalize_angle(angle_deg - candidate))
        if best_error is None or error < best_error:
            best = candidate
            best_error = error
    return normalize_angle(best)


def rotation_error_deg(actual_deg, expected_deg):
    return normalize_angle(float(actual_deg) - float(expected_deg))


def angle_is_cardinal(angle_deg, tol_deg):
    snapped = nearest_cardinal_angle(angle_deg)
    return abs(rotation_error_deg(angle_deg, snapped)) <= float(tol_deg)


def orientation_validation_mode_for_expected(expected_deg, cfg, tag):
    mode = str(cfg.get("orientation_validation_mode", "auto")).lower()
    if mode in ["cardinal", "continuous"]:
        return mode

    cardinal_tol = float(cfg.get("orientation_max_error_deg", 25.0))
    if angle_is_cardinal(expected_deg, cardinal_tol):
        return "cardinal"
    return "continuous"


def rotate_xy(x, y, angle_deg):
    a = math.radians(float(angle_deg))
    ca = math.cos(a)
    sa = math.sin(a)
    return {
        "x": ca * float(x) - sa * float(y),
        "y": sa * float(x) + ca * float(y)
    }


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
            v = abs(m[row][col])
            if v > pivot_abs:
                pivot = row
                pivot_abs = v
        if pivot_abs < 0.000000000001:
            raise Exception("Pivot fit matrix is singular.")
        if pivot != col:
            tmp = m[col]
            m[col] = m[pivot]
            m[pivot] = tmp
        scale = m[col][col]
        for j in range(col, n + 1):
            m[col][j] = m[col][j] / scale
        for row in range(n):
            if row == col:
                continue
            factor = m[row][col]
            if abs(factor) <= 0.0:
                continue
            for j in range(col, n + 1):
                m[row][j] = m[row][j] - factor * m[col][j]

    out = []
    for i in range(n):
        out.append(m[i][n])
    return out


def fit_bias_plus_rotating_pivot(samples):
    has_pick_place_angles = True
    for s in samples:
        if "pick_rotation_deg" not in s or "place_rotation_deg" not in s:
            has_pick_place_angles = False
            break
    if has_pick_place_angles:
        try:
            return fit_bias_plus_pick_place_vectors(samples)
        except Exception, e:
            log("PivotFit", "Pick/place vector fit failed; falling back to legacy place-only model: %s" % e)

    # Unknowns are [bias_x, bias_y, local_v_x, local_v_y].
    ata = [[0.0, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0]]
    atb = [0.0, 0.0, 0.0, 0.0]
    for s in samples:
        a = math.radians(float(s["commanded_angle_deg"]))
        ca = math.cos(a)
        sa = math.sin(a)
        rows = [
            ([1.0, 0.0, ca, -sa], float(s["error_x_mm"])),
            ([0.0, 1.0, sa, ca], float(s["error_y_mm"]))
        ]
        for row, value in rows:
            for i in range(4):
                atb[i] += row[i] * value
                for j in range(4):
                    ata[i][j] += row[i] * row[j]

    bx, by, vx, vy = solve_linear_system(ata, atb)
    residuals = []
    for s in samples:
        rv = rotate_xy(vx, vy, s["commanded_angle_deg"])
        px = bx + rv["x"]
        py = by + rv["y"]
        rx = float(s["error_x_mm"]) - px
        ry = float(s["error_y_mm"]) - py
        residuals.append(math.sqrt(rx * rx + ry * ry))

    theta_errors = [float(s["rotation_error_deg"]) for s in samples]
    vector_mag = math.sqrt(vx * vx + vy * vy)
    bias_mag = math.sqrt(bx * bx + by * by)
    return {
        "label": "computed",
        "model": "bias_plus_rotating_pivot_vector",
        "sample_count": len(samples),
        "bias_x_mm": bx,
        "bias_y_mm": by,
        "bias_mag_mm": bias_mag,
        "pivot_vector_x_mm": vx,
        "pivot_vector_y_mm": vy,
        "pivot_vector_mag_mm": vector_mag,
        "theta_error_deg": robust_center(theta_errors),
        "rotation_error_deg": robust_center(theta_errors),
        "offset_x_mm": bx,
        "offset_y_mm": by,
        "offset_mag_mm": bias_mag,
        "repeatability_rms_mm": rms(residuals),
        "residual_rms_mm": rms(residuals),
        "max_residual_mm": max(residuals) if residuals else 0.0
    }


def fit_bias_plus_pick_place_vectors(samples):
    # Unknowns are [bias_x, bias_y, place_v_x, place_v_y, pick_v_x, pick_v_y].
    n = 6
    ata = []
    atb = []
    for i in range(n):
        row = []
        for j in range(n):
            row.append(0.0)
        ata.append(row)
        atb.append(0.0)

    for s in samples:
        place_a = math.radians(float(s["place_rotation_deg"]))
        pick_a = math.radians(float(s["pick_rotation_deg"]))
        cpl = math.cos(place_a)
        spl = math.sin(place_a)
        cpk = math.cos(pick_a)
        spk = math.sin(pick_a)
        rows = [
            ([1.0, 0.0, cpl, -spl, -cpk, spk], float(s["error_x_mm"])),
            ([0.0, 1.0, spl, cpl, -spk, -cpk], float(s["error_y_mm"]))
        ]
        for row, value in rows:
            for i in range(n):
                atb[i] += row[i] * value
                for j in range(n):
                    ata[i][j] += row[i] * row[j]

    bx, by, place_vx, place_vy, pick_vx, pick_vy = solve_linear_system(ata, atb)
    residuals = []
    for s in samples:
        place_term = rotate_xy(place_vx, place_vy, s["place_rotation_deg"])
        pick_term = rotate_xy(pick_vx, pick_vy, s["pick_rotation_deg"])
        px = bx + place_term["x"] - pick_term["x"]
        py = by + place_term["y"] - pick_term["y"]
        rx = float(s["error_x_mm"]) - px
        ry = float(s["error_y_mm"]) - py
        residuals.append(math.sqrt(rx * rx + ry * ry))

    theta_errors = [float(s["rotation_error_deg"]) for s in samples]
    place_mag = math.sqrt(place_vx * place_vx + place_vy * place_vy)
    pick_mag = math.sqrt(pick_vx * pick_vx + pick_vy * pick_vy)
    bias_mag = math.sqrt(bx * bx + by * by)
    return {
        "label": "computed",
        "model": "bias_plus_pick_place_vectors",
        "sample_count": len(samples),
        "bias_x_mm": bx,
        "bias_y_mm": by,
        "bias_mag_mm": bias_mag,
        "place_vector_x_mm": place_vx,
        "place_vector_y_mm": place_vy,
        "pick_vector_x_mm": pick_vx,
        "pick_vector_y_mm": pick_vy,
        "place_vector_mag_mm": place_mag,
        "pick_vector_mag_mm": pick_mag,
        "theta_error_deg": robust_center(theta_errors),
        "rotation_error_deg": robust_center(theta_errors),
        "offset_x_mm": bx,
        "offset_y_mm": by,
        "offset_mag_mm": bias_mag,
        "repeatability_rms_mm": rms(residuals),
        "residual_rms_mm": rms(residuals),
        "max_residual_mm": max(residuals) if residuals else 0.0
    }


def pose_rotation_for_pick(pose, cfg):
    # Use raw continuous angle so square nozzle aligns to the actual skewed die.
    if bool(pose.get("orientation_found", False)):
        return normalize_angle(float(pose.get("rotation_raw_deg",
                   pose.get("orientation_raw_rotation_deg",
                   pose.get("rotation_deg", 0.0)))))
    return normalize_angle(float(pose.get("rotation_deg", 0.0)))


def pose_rotation_for_validation(pose, cfg):
    # Use snapped only to confirm the die is near the expected cardinal orientation.
    if bool(cfg.get("orientation_snap_to_cardinal", True)):
        return normalize_angle(float(pose.get("rotation_snapped_deg",
                   pose.get("orientation_rotation_deg",
                   pose.get("rotation_deg", 0.0)))))
    return pose_rotation_for_pick(pose, cfg)


def storage_result(storage_loc, detected_storage_pick_loc):
    if storage_loc is None:
        return None
    detected = detected_storage_pick_loc
    if detected is not None:
        detected = detected.convertToUnits(LengthUnit.Millimeters)
    return {
        "configured_x_mm": storage_loc.getX(),
        "configured_y_mm": storage_loc.getY(),
        "detected_initial_pick_x_mm": detected.getX() if detected is not None else None,
        "detected_initial_pick_y_mm": detected.getY() if detected is not None else None,
        "detected_initial_pick_rotation_deg": detected.getRotation() if detected is not None else None,
        "storage_pick_offset_x_mm": detected.getX() - storage_loc.getX() if detected is not None else None,
        "storage_pick_offset_y_mm": detected.getY() - storage_loc.getY() if detected is not None else None,
        "used_for_calibration": False
    }


def mark_current_die_loc_trusted(state, loc, trusted):
    if state is None:
        return
    state["current_die_loc"] = loc
    state["current_die_loc_trusted"] = bool(trusted)
    if bool(trusted) and loc is not None:
        state["last_trusted_die_loc"] = loc


def validate_pose_orientation(pose, expected_loc, cfg, tag):
    expected = normalize_angle(float(expected_loc.getRotation()))
    if not bool(pose.get("orientation_found", False)):
        if bool(cfg.get("require_orientation_circle", True)):
            raise Exception("[Orientation][%s] required orientation circle missing; expected R=%.4f measured_xy=(%.4f, %.4f)" %
                            (tag, expected,
                             float(pose.get("centered_machine_x_mm", pose.get("center_x_mm", 0.0))),
                             float(pose.get("centered_machine_y_mm", pose.get("center_y_mm", 0.0)))))
        return 0.0

    raw = pose_rotation_for_pick(pose, cfg)
    snapped = pose_rotation_for_validation(pose, cfg)
    raw_error = rotation_error_deg(raw, expected)
    snapped_error = rotation_error_deg(snapped, expected)
    snap_error = float(pose.get("orientation_snap_error_deg", 0.0))
    mode = orientation_validation_mode_for_expected(expected, cfg, tag)
    if mode == "continuous":
        max_error = float(cfg.get("orientation_continuous_max_error_deg", 5.0))
        if abs(raw_error) > max_error:
            raise Exception("[Orientation][%s] validation failed: expected R=%.4f raw=%.4f raw_error=%.4f max_error=%.4f snapped=%.4f snap_error=%.4f measured_xy=(%.4f, %.4f)" %
                            (tag, expected, raw, raw_error, max_error, snapped, snap_error,
                             float(pose.get("centered_machine_x_mm", pose.get("center_x_mm", 0.0))),
                             float(pose.get("centered_machine_y_mm", pose.get("center_y_mm", 0.0)))))
        log("Orientation", "[%s] mode=continuous expected=%.4f raw=%.4f raw_error=%.4f snapped=%.4f snap_error_from_cardinal=%.4f" %
            (tag, expected, raw, raw_error, snapped, snap_error))
        return raw_error

    max_error = float(cfg.get("orientation_max_error_deg", 25.0))
    if bool(cfg.get("orientation_snap_to_cardinal", True)) and abs(snap_error) > max_error:
        raise Exception("[Orientation][%s] validation failed: raw angle is not near a cardinal orientation; expected R=%.4f raw=%.4f snapped=%.4f raw_error=%.4f snapped_error=%.4f snap_error=%.4f measured_xy=(%.4f, %.4f)" %
                        (tag, expected, raw, snapped, raw_error, snapped_error, snap_error,
                         float(pose.get("centered_machine_x_mm", pose.get("center_x_mm", 0.0))),
                         float(pose.get("centered_machine_y_mm", pose.get("center_y_mm", 0.0)))))
    if abs(snapped_error) > max_error:
        raise Exception("[Orientation][%s] validation failed: expected R=%.4f raw=%.4f snapped=%.4f raw_error=%.4f snapped_error=%.4f snap_error=%.4f measured_xy=(%.4f, %.4f)" %
                        (tag, expected, raw, snapped, raw_error, snapped_error,
                         snap_error,
                         float(pose.get("centered_machine_x_mm", pose.get("center_x_mm", 0.0))),
                         float(pose.get("centered_machine_y_mm", pose.get("center_y_mm", 0.0)))))
    log("Orientation", "[%s] mode=cardinal expected=%.4f raw=%.4f snapped=%.4f raw_error=%.4f snapped_error=%.4f snap_error_from_cardinal=%.4f" %
        (tag, expected, raw, snapped, raw_error, snapped_error, snap_error))
    return raw_error


def nearest_nozzle_rotation_location(nozzle, target, tag):
    target = target.convertToUnits(LengthUnit.Millimeters)
    try:
        current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        current_r = float(current.getRotation())
    except:
        return target

    target_r = float(target.getRotation())
    commanded_r = nearest_equivalent_angle(target_r, current_r)
    delta = commanded_r - current_r
    if abs(commanded_r - target_r) > 0.000001:
        log(tag, "Nozzle R target=%.4f current=%.4f commanded=%.4f delta=%.4f" %
            (target_r, current_r, commanded_r, delta))
    return mm_loc(target.getX(), target.getY(), target.getZ(), commanded_r)


def same_machine_location(a, b, linear_tol=0.0001, rotation_tol=0.0001):
    a = a.convertToUnits(LengthUnit.Millimeters)
    b = b.convertToUnits(LengthUnit.Millimeters)
    if abs(float(a.getX()) - float(b.getX())) > float(linear_tol):
        return False
    if abs(float(a.getY()) - float(b.getY())) > float(linear_tol):
        return False
    if abs(float(a.getZ()) - float(b.getZ())) > float(linear_tol):
        return False
    return abs(normalize_angle(float(a.getRotation()) - float(b.getRotation()))) <= float(rotation_tol)


def square_fit_offset_solver(circles):
    if len(circles) < 4:
        raise Exception("Need 4 circles, got %d" % len(circles))

    pts = sort_square_corners(circles[:4])
    cx = mean([p["x_mm"] for p in pts])
    cy = mean([p["y_mm"] for p in pts])

    side_lengths = []
    edge_angles = []
    for i in range(4):
        a = pts[i]
        b = pts[(i + 1) % 4]
        dx = b["x_mm"] - a["x_mm"]
        dy = b["y_mm"] - a["y_mm"]
        side_lengths.append(math.sqrt(dx * dx + dy * dy))
        edge_angles.append(normalize_angle(math.degrees(math.atan2(dy, dx)) - 90.0 * i))

    angle_deg = normalize_angle(mean(edge_angles))
    side_mm = mean(side_lengths)
    side_rms_mm = rms([v - side_mm for v in side_lengths])

    pose = {
        "center_x_mm": cx,
        "center_y_mm": cy,
        "rotation_deg": angle_deg,
        "side_mm": side_mm,
        "side_rms_mm": side_rms_mm,
        "corners": pts
    }
    pixel_points = []
    for p in pts:
        if "x_px" in p and "y_px" in p:
            pixel_points.append(p)
    if len(pixel_points) == len(pts):
        pose["center_x_px"] = mean([p["x_px"] for p in pixel_points])
        pose["center_y_px"] = mean([p["y_px"] for p in pixel_points])
    return pose


def select_orientation_point(orientation_points, pose):
    if not orientation_points:
        return None
    if len(orientation_points) == 1:
        return orientation_points[0]

    cx = float(pose.get("center_x_mm", 0.0))
    cy = float(pose.get("center_y_mm", 0.0))
    best = None
    best_d2 = -1.0
    for p in orientation_points:
        try:
            dx = float(p["x_mm"]) - cx
            dy = float(p["y_mm"]) - cy
            d2 = dx * dx + dy * dy
            if d2 > best_d2:
                best = p
                best_d2 = d2
        except:
            pass
    if best is None:
        return orientation_points[0]
    return best


def apply_orientation_circle_to_pose(pose, orientation_points, orientation_stage_name, cfg):
    point = select_orientation_point(orientation_points, pose)
    if point is None:
        pose["square_edge_rotation_deg"] = float(pose.get("rotation_deg", 0.0))
        if bool(cfg.get("require_orientation_circle", True)):
            raise Exception("Orientation circle required but stage '%s' returned no point." %
                            cfg.get("orientation_result_stage_name", "OrientCircleResult"))
        pose["orientation_stage_name"] = orientation_stage_name
        pose["orientation_found"] = False
        pose["rotation_source"] = "square_edge_diagnostic"
        return pose

    cx = float(pose.get("center_x_mm", 0.0))
    cy = float(pose.get("center_y_mm", 0.0))
    ox = float(point["x_mm"])
    oy = float(point["y_mm"])
    dx = ox - cx
    dy = oy - cy
    orientation_x_sign = float(cfg.get("orientation_x_sign", -1.0))
    runtime_sign = float(cfg.get("_orientation_runtime_sign_multiplier", 1.0))
    raw_angle_deg = normalize_angle(math.degrees(math.atan2(runtime_sign * orientation_x_sign * dx, dy)))
    snapped_angle_deg = nearest_cardinal_angle(raw_angle_deg)
    snap_error_deg = rotation_error_deg(raw_angle_deg, snapped_angle_deg)
    pose["orientation_found"] = True
    pose["orientation_x_mm"] = ox
    pose["orientation_y_mm"] = oy
    pose["orientation_dx_mm"] = dx
    pose["orientation_dy_mm"] = dy
    pose["orientation_raw_rotation_deg"] = raw_angle_deg
    pose["orientation_rotation_deg"] = snapped_angle_deg
    pose["rotation_raw_deg"] = raw_angle_deg
    pose["rotation_snapped_deg"] = snapped_angle_deg
    pose["orientation_snap_error_deg"] = snap_error_deg
    pose["orientation_stage_name"] = orientation_stage_name
    if "x_px" in point:
        pose["orientation_x_px"] = float(point["x_px"])
    if "y_px" in point:
        pose["orientation_y_px"] = float(point["y_px"])
    if "radius_px" in point:
        pose["orientation_radius_px"] = float(point["radius_px"])
    pose["square_edge_rotation_deg"] = float(pose.get("rotation_deg", 0.0))
    pose["rotation_deg"] = raw_angle_deg
    pose["rotation_source"] = "orientation_circle_raw"
    log("Vision", "Orientation circle stage='%s' dx=%.5f dy=%.5f orientation_x_sign=%.1f runtime_sign=%.1f raw=%.4f snapped=%.4f snap_error=%.4f rotation=%.4f source=%s" %
        (orientation_stage_name, dx, dy, orientation_x_sign, runtime_sign, raw_angle_deg, snapped_angle_deg,
         snap_error_deg, raw_angle_deg, "orientation_circle_raw"))
    return pose


def summarize_samples(samples, label):
    xs = [s["error_x_mm"] for s in samples]
    ys = [s["error_y_mm"] for s in samples]
    rs = [s["rotation_error_deg"] for s in samples]
    ox = robust_center(xs)
    oy = robust_center(ys)
    omag = math.sqrt(ox * ox + oy * oy)
    residuals = []
    for s in samples:
        residuals.append(math.sqrt((s["error_x_mm"] - ox) ** 2 + (s["error_y_mm"] - oy) ** 2))

    model = "simple_xy"
    try:
        if samples and samples[0].get("placement_model", None) is not None:
            model = samples[0].get("placement_model", "simple_xy")
    except:
        pass
    residual_rms = rms(residuals)
    return {
        "label": label,
        "model": model,
        "sample_count": len(samples),
        "offset_x_mm": ox,
        "offset_y_mm": oy,
        "offset_mag_mm": omag,
        "rotation_error_deg": robust_center(rs),
        "repeatability_rms_mm": residual_rms,
        "residual_rms_mm": residual_rms,
        "max_residual_mm": max(residuals) if residuals else 0.0
    }


def storage_base_z_mm(cfg, storage_loc=None):
    if cfg.get("_storage_base_z_mm", None) is not None:
        return float(cfg["_storage_base_z_mm"])
    if storage_loc is not None:
        return float(storage_loc.getZ())
    return float(cfg.get("die_storage_location_xyz", {}).get("z", 0.0))


def part_height_z_mm(cfg):
    return float(cfg.get("_part_height_z_mm", 0.0))


def pick_surface_z_mm(base_loc, cfg):
    return float(base_loc.getZ()) + part_height_z_mm(cfg)


def storage_pick_surface_z_mm(cfg, storage_loc=None):
    return storage_base_z_mm(cfg, storage_loc) + part_height_z_mm(cfg)


def storage_vision_z_mm(cfg, storage_loc=None):
    return storage_pick_surface_z_mm(cfg, storage_loc)


def pick_z_for(base_loc, cfg):
    if location_kind(base_loc, cfg) == "Storage":
        return storage_pick_surface_z_mm(cfg, base_loc)
    return pick_surface_z_mm(base_loc, cfg)


def xy_for_pick_place(base_loc, pose, cfg, tag):
    centered_x = pose.get("centered_machine_x_mm", base_loc.getX())
    centered_y = pose.get("centered_machine_y_mm", base_loc.getY())
    dx = base_loc.getX() - centered_x
    dy = base_loc.getY() - centered_y
    mode = cfg.get("pick_place_xy_mode", "detected_center")
    if mode == "configured":
        log("XY", "mode=configured tag=%s validation_delta dx=%.4f dy=%.4f" % (tag, dx, dy))
        return mm_loc(base_loc.getX(), base_loc.getY(), base_loc.getZ(), base_loc.getRotation())
    log("XY", "mode=detected_center tag=%s applying centered delta dx=%.4f dy=%.4f" % (tag, dx, dy))
    return mm_loc(centered_x, centered_y, base_loc.getZ(), base_loc.getRotation())


def tool_aware_camera_target_for_location(camera, tool, tool_target_loc, camera_z):
    cam_loc = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
    tool_center_loc = camera.getLocation(tool).convertToUnits(LengthUnit.Millimeters)
    tool_target_loc = tool_target_loc.convertToUnits(LengthUnit.Millimeters)
    dx = tool_target_loc.getX() - tool_center_loc.getX()
    dy = tool_target_loc.getY() - tool_center_loc.getY()
    return mm_loc(cam_loc.getX() + dx, cam_loc.getY() + dy,
                  float(camera_z), cam_loc.getRotation())


def camera_measurement_loc(camera, x, y, z):
    cam_loc = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
    return mm_loc(float(x), float(y), float(z), cam_loc.getRotation())


def tool_aware_pixel_location(camera, tool, pose, fallback_loc):
    if "center_x_px" in pose and "center_y_px" in pose:
        loc = VisionUtils.getPixelLocation(camera, tool, float(pose["center_x_px"]), float(pose["center_y_px"]))
        loc = loc.convertToUnits(LengthUnit.Millimeters)
        fallback_loc = fallback_loc.convertToUnits(LengthUnit.Millimeters)
        return mm_loc(loc.getX(), loc.getY(), fallback_loc.getZ(), fallback_loc.getRotation())

    current_tool_center = camera.getLocation(tool).convertToUnits(LengthUnit.Millimeters)
    return mm_loc(current_tool_center.getX() + float(pose.get("center_x_mm", 0.0)),
                  current_tool_center.getY() + float(pose.get("center_y_mm", 0.0)),
                  fallback_loc.getZ(), fallback_loc.getRotation())


def camera_pixel_location(camera, pose, fallback_loc):
    if "center_x_px" in pose and "center_y_px" in pose:
        loc = VisionUtils.getPixelLocation(camera, float(pose["center_x_px"]), float(pose["center_y_px"]))
        loc = loc.convertToUnits(LengthUnit.Millimeters)
        fallback_loc = fallback_loc.convertToUnits(LengthUnit.Millimeters)
        return mm_loc(loc.getX(), loc.getY(), fallback_loc.getZ(), fallback_loc.getRotation())

    cam_loc = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
    return mm_loc(cam_loc.getX() + float(pose.get("center_x_mm", 0.0)),
                  cam_loc.getY() + float(pose.get("center_y_mm", 0.0)),
                  fallback_loc.getZ(), fallback_loc.getRotation())


def log_coordinate_mode(tag, base_loc, xy_loc, pick_z, cfg, computed_xy_loc=None):
    if computed_xy_loc is None:
        computed_xy_loc = xy_loc
    approach_z = float(pick_z) + float(cfg["z_clearance_before_pick_mm"])
    log(tag, "base xyzr=(%.4f, %.4f, %.4f, %.4f) computed_centered_xy=(%.4f, %.4f) descent_xy=(%.4f, %.4f) pick_z=%.4f approach_z=%.4f base_z=%.4f selected_part_height_z=%.4f pick_surface_z=%.4f" %
        (base_loc.getX(), base_loc.getY(), base_loc.getZ(), base_loc.getRotation(),
         computed_xy_loc.getX(), computed_xy_loc.getY(), xy_loc.getX(), xy_loc.getY(), float(pick_z), approach_z,
         float(base_loc.getZ()), part_height_z_mm(cfg), pick_surface_z_mm(base_loc, cfg)))


def enforce_write_guard(cfg):
    if not bool(cfg.get("allow_machine_writes", False)):
        log("Safety", "write guard active")
        raise Exception("apply_offsets requested but allow_machine_writes is false; refusing machine calibration writes.")


def camera_z_for_storage(storage_loc, cfg):
    return storage_vision_z_mm(cfg, storage_loc)


def camera_z_for_cal(cal_loc, cfg):
    return pick_surface_z_mm(cal_loc, cfg) + float(cfg.get("cal_camera_z_offset", 0.0))


def camera_z_for(cal_loc, cfg):
    return camera_z_for_cal(cal_loc, cfg)


def vision_lock_name(tag):
    if str(tag).lower().startswith("storage"):
        return "Storage"
    if str(tag).lower().startswith("cal"):
        return "Cal"
    return str(tag)


def acquire_die_pose_at_location(camera, nozzle, part, base_loc, camera_z, cfg, tag):
    lock_tag = vision_lock_name(tag)
    target = camera_measurement_loc(camera, base_loc.getX(), base_loc.getY(), float(camera_z))
    try:
        log("VisionLock", "[%s] acquiring at X=%.4f Y=%.4f Z=%.4f R=%.4f" %
            (lock_tag, target.getX(), target.getY(), target.getZ(), target.getRotation()))
        move_safe_then_xy_then_z(camera, target, cfg["safe_travel_z"], "VisionLock")
        pose = detect_with_spiral(camera, nozzle, part, cfg, target)
        count = int(pose.get("circle_count", 4))
        if count < int(cfg["circle_detection_min_count"]):
            log("VisionLock", "[%s] failed: only %d circles" % (lock_tag, count))
            raise Exception("[VisionLock][%s] failed: detected %d circles, need 4" % (lock_tag, count))

        centered_camera_loc = camera_pixel_location(camera, pose, base_loc)
        centered_tool_loc = tool_aware_pixel_location(camera, nozzle, pose, base_loc)
        centered_x = centered_camera_loc.getX()
        centered_y = centered_camera_loc.getY()
        pose["centered_machine_x_mm"] = centered_x
        pose["centered_machine_y_mm"] = centered_y
        pose["centered_tool_x_mm"] = centered_tool_loc.getX()
        pose["centered_tool_y_mm"] = centered_tool_loc.getY()
        current = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
        pose["camera_machine_x_mm"] = current.getX()
        pose["camera_machine_y_mm"] = current.getY()
        if "center_x_px" in pose and "center_y_px" in pose:
            pose["center_px"] = {"x": pose["center_x_px"], "y": pose["center_y_px"]}
        pose["overlay"] = build_pose_overlay(camera, pose, base_loc)
        save_last_overlay(pose["overlay"])
        log("VisionLock", "[%s] acquired: centered_location X=%.4f Y=%.4f Z=%.4f R=%.4f; centered_tool_location X=%.4f Y=%.4f Z=%.4f R=%.4f" %
            (lock_tag,
             centered_camera_loc.getX(), centered_camera_loc.getY(),
             centered_camera_loc.getZ(), centered_camera_loc.getRotation(),
             centered_tool_loc.getX(), centered_tool_loc.getY(),
             centered_tool_loc.getZ(), centered_tool_loc.getRotation()))
        return {
            "pose": pose,
            "centered_location": centered_camera_loc,
            "centered_tool_location": centered_tool_loc
        }
    except Exception, e:
        log("VisionLock", "[%s] failed: %s" % (lock_tag, e))
        raise


def compute_centering_delta_mm(pose):
    dx = float(pose.get("center_x_mm", 0.0))
    dy = float(pose.get("center_y_mm", 0.0))
    log("Vision", "Centering delta: pixel_to_machine dx=%.5f dy=%.5f" % (dx, dy))
    return dx, dy


def acquire_storage_center(camera, nozzle, part, storage_loc, cfg):
    vision_z = storage_vision_z_mm(cfg, storage_loc)
    camera_search_loc = camera_measurement_loc(camera, storage_loc.getX(), storage_loc.getY(), vision_z)
    lock = acquire_die_pose_at_location(camera, nozzle, part, camera_search_loc, vision_z, cfg, "Storage")
    pose = lock["pose"]
    dx, dy = compute_centering_delta_mm(pose)
    centered_nozzle_loc = lock["centered_tool_location"]
    lock["centered_nozzle_location"] = centered_nozzle_loc
    lock["camera_search_location"] = camera_search_loc
    return lock, dx, dy, vision_z


def center_tool_on_detected_die(camera_or_nozzle, base_machine_loc, pose, cfg, tag,
                                camera=None, nozzle=None, part=None, camera_z=None):
    centered_x = pose.get("centered_machine_x_mm", None)
    centered_y = pose.get("centered_machine_y_mm", None)
    if centered_x is None or centered_y is None:
        centered_x = base_machine_loc.getX() + pose.get("search_dx_mm", 0.0) + pose["center_x_mm"]
        centered_y = base_machine_loc.getY() + pose.get("search_dy_mm", 0.0) + pose["center_y_mm"]

    current = camera_or_nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    centered_loc = mm_loc(centered_x, centered_y, current.getZ(), current.getRotation())
    safe_staged_move(camera_or_nozzle, centered_loc, cfg["safe_travel_z"], tag)
    log("PickFlow", "centered_xy=(%.4f, %.4f) using detected die pose" % (centered_x, centered_y))

    if bool(cfg.get("verify_lock_after_centering", True)) and camera is not None and nozzle is not None and part is not None:
        verify_base = mm_loc(centered_x, centered_y, base_machine_loc.getZ(), base_machine_loc.getRotation())
        verify_z = camera_z
        if verify_z is None:
            verify_z = current.getZ()
        verify = acquire_die_pose_at_location(camera, nozzle, part, verify_base, verify_z, cfg, tag)
        vpose = verify["pose"]
        if int(vpose.get("circle_count", 4)) < int(cfg["circle_detection_min_count"]):
            raise Exception("[VisionLock][%s] failed after centering." % vision_lock_name(tag))

    return mm_loc(centered_x, centered_y, base_machine_loc.getZ(), base_machine_loc.getRotation())


def guarded_descent(nozzle, xy_loc, target_z, cfg, action_tag, context="Cal"):
    approach_z = float(target_z) + float(cfg["z_clearance_before_pick_mm"])
    descent_mm = abs(approach_z - float(target_z))
    log("ZGuard", "approach_z=%.4f target_pick_z=%.4f descent_mm=%.4f" %
        (approach_z, float(target_z), descent_mm))
    if descent_mm > float(cfg["max_pick_descent_mm"]):
        raise Exception("[ZGuard] Refusing descent %.4f mm > max_pick_descent_mm %.4f" %
                        (descent_mm, float(cfg["max_pick_descent_mm"])))

    final = mm_loc(xy_loc.getX(), xy_loc.getY(), float(target_z), xy_loc.getRotation())
    final = nearest_nozzle_rotation_location(nozzle, final, action_tag)
    try:
        current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        if same_machine_location(current, final):
            log(action_tag, "guarded descent skipped; nozzle already at final target")
            return
    except:
        pass

    log(action_tag, "async-safe staged nozzle move; context=%s" % context)
    move_nozzle_split(nozzle, final, cfg, action_tag, context)
    try:
        actual = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        log(action_tag, "After guarded descent intended X=%.4f Y=%.4f Z=%.4f R=%.4f actual X=%.4f Y=%.4f Z=%.4f R=%.4f delta X=%.5f Y=%.5f Z=%.5f R=%.5f" %
            (final.getX(), final.getY(), final.getZ(), final.getRotation(),
             actual.getX(), actual.getY(), actual.getZ(), actual.getRotation(),
             actual.getX() - final.getX(), actual.getY() - final.getY(),
             actual.getZ() - final.getZ(), normalize_angle(actual.getRotation() - final.getRotation())))
    except:
        pass


def move_pick_place_target(movable, target, cfg, tag):
    target = target.convertToUnits(LengthUnit.Millimeters)
    log(tag, "Async-safe pick/place move %s -> X=%.4f Y=%.4f Z=%.4f R=%.4f" %
        (object_name(movable), target.getX(), target.getY(), target.getZ(), target.getRotation()))
    move_nozzle_split(movable, target, cfg, tag, "Cal")


def location_kind(base_loc, cfg):
    storage = loc_from_xyz(cfg["die_storage_location_xyz"])
    cal = loc_from_xyz(cfg["cal_work_location_xyz"])
    if abs(base_loc.getX() - storage.getX()) < 0.001 and abs(base_loc.getY() - storage.getY()) < 0.001:
        return "Storage"
    if abs(base_loc.getX() - cal.getX()) < 0.001 and abs(base_loc.getY() - cal.getY()) < 0.001:
        return "Cal"
    return "Location"


def camera_z_for_location(base_loc, cfg):
    kind = location_kind(base_loc, cfg)
    if kind == "Storage":
        return camera_z_for_storage(base_loc, cfg)
    return camera_z_for_cal(base_loc, cfg)


def pick_die_with_visual_lock(camera, nozzle, part, base_loc, cfg, tag, cycle_state=None):
    lock_tag = vision_lock_name(tag)
    if lock_tag == "Storage":
        lock, dx, dy, camera_z = acquire_storage_center(camera, nozzle, part, base_loc, cfg)
        cam_search = lock.get("camera_search_location", None)
        if cam_search is not None:
            log("VisionLock", "[Storage] storage_camera_xy=(%.4f, %.4f) camera_search_xy=(%.4f, %.4f) vision_z=%.4f final_delta_mm=(%.5f, %.5f)" %
                (base_loc.getX(), base_loc.getY(), cam_search.getX(), cam_search.getY(), camera_z, dx, dy))
        else:
            log("VisionLock", "[Storage] vision_z=%.4f final_delta_mm=(%.5f, %.5f)" % (camera_z, dx, dy))
    else:
        camera_z = camera_z_for_location(base_loc, cfg)
        camera_base_loc = tool_aware_camera_target_for_location(camera, nozzle, base_loc, camera_z)
        lock = acquire_die_pose_at_location(camera, nozzle, part, camera_base_loc, camera_z, cfg, lock_tag)
    if lock_tag == "Storage":
        log("Sequence", "Storage vision-lock acquired")
    pose = lock["pose"]
    if bool(cfg.get("require_visual_lock_before_pick", True)) and int(pose.get("circle_count", 4)) < 4:
        raise Exception("[VisionLock][%s] pick aborted: visual lock count < 4" % lock_tag)
    if lock_tag == "Cal":
        try:
            validate_pose_orientation(pose, base_loc, cfg, "%s pre-pick" % tag)
        except Exception:
            if cycle_state is not None:
                cycle_state["current_die_loc_trusted"] = False
            raise

    if lock_tag == "Storage":
        centered_camera = lock.get("centered_location", base_loc)
        computed_centered = lock.get("centered_nozzle_location", base_loc)
        if cfg.get("pick_place_xy_mode", "detected_center") == "configured":
            centered = base_loc
        else:
            centered = computed_centered
    else:
        centered_camera = lock.get("centered_location", base_loc)
        computed_centered = lock.get("centered_tool_location", base_loc)
        if cfg.get("pick_place_xy_mode", "detected_center") == "configured":
            centered = base_loc
        else:
            centered = computed_centered
    if bool(cfg.get("align_nozzle_to_detected_die_rotation", True)) and bool(pose.get("orientation_found", False)):
        detected_r = pose_rotation_for_pick(pose, cfg)
        validation_r = pose_rotation_for_validation(pose, cfg)
        centered = mm_loc(centered.getX(), centered.getY(), centered.getZ(), detected_r)
        if computed_centered is not None:
            computed_centered = mm_loc(computed_centered.getX(), computed_centered.getY(),
                                      computed_centered.getZ(), detected_r)
        log("Pick", "Aligning nozzle rotation to detected die raw rotation %.4f deg; snapped/cardinal=%.4f deg" %
            (detected_r, validation_r))
    elif bool(cfg.get("align_nozzle_to_detected_die_rotation", True)):
        log("Pick", "Orientation circle not found; leaving nozzle rotation unchanged.")
    pick_z = pick_z_for(base_loc, cfg)
    try:
        cam_now = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
        noz_now = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        log("Frame", "camera_xy=(%.4f, %.4f) nozzle_xy=(%.4f, %.4f) camera_target_xy=(%.4f, %.4f) target_nozzle_xy=(%.4f, %.4f) transform=VisionUtils.getPixelLocation(camera,nozzle,px,py)" %
            (cam_now.getX(), cam_now.getY(), noz_now.getX(), noz_now.getY(),
             centered_camera.getX(), centered_camera.getY(), centered.getX(), centered.getY()))
    except:
        pass
    log_coordinate_mode("Pick", base_loc, centered, pick_z, cfg, computed_centered)
    guarded_descent(nozzle, centered, pick_z, cfg, "PickFlow", lock_tag)
    log("Pick", "Picking die part '%s'" % part.getId())
    log_coordinate_mode("Pick", base_loc, centered, pick_z, cfg, computed_centered)
    feeder_name = cfg.get("feeder_name_override", None)
    log("Pick", "Resolving feeder override: %s" % feeder_name)
    try:
        feeder = find_feeder_by_name(get_machine(), feeder_name)
    except Exception, feeder_error:
        log("Pick", "Feeder override lookup failed, continuing with feeder=None: %s" % feeder_error)
        feeder = None
    log("Pick", "Feeder resolved: %s" % object_name(feeder))
    try:
        pick_path = do_pick(nozzle, part, feeder, cfg)
    except Throwable, throwable:
        log("Pick", "Pick actuation path failed before state update: %s" % throwable)
        raise
    verify_part_on_after_pick(nozzle)
    wait_still(nozzle)
    try:
        actual_pick_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        detected_pick_r = pose_rotation_for_pick(pose, cfg)
        if cycle_state is not None:
            cycle_state["last_pick_x_mm"] = float(actual_pick_loc.getX())
            cycle_state["last_pick_y_mm"] = float(actual_pick_loc.getY())
            cycle_state["last_pick_rotation_deg"] = normalize_angle(float(actual_pick_loc.getRotation()))
            cycle_state["last_pick_detected_rotation_deg"] = normalize_angle(float(detected_pick_r))
        log("Pick", "pick_pose actual=(%.4f, %.4f, %.4f) detected_rotation=%.4f" %
            (actual_pick_loc.getX(), actual_pick_loc.getY(),
             normalize_angle(float(actual_pick_loc.getRotation())), detected_pick_r))
    except Exception, pick_pose_error:
        log("Pick", "Could not record pick pose: %s" % pick_pose_error)
    if cycle_state is not None:
        cycle_state["die_on_nozzle"] = True
        if lock_tag == "Cal":
            cycle_state["die_at_work"] = False
            cycle_state["current_die_loc_trusted"] = False
    log("Pick", "Successful pick with nozzle_tip=%s path=%s" %
        (current_nozzle_tip_name(nozzle), pick_path))
    if lock_tag == "Storage":
        log("Sequence", "Picked from storage")
    elif lock_tag == "Cal":
        log("Sequence", "Picked from calibration location")
    lift_nozzle_after_pick_place(nozzle, cfg, "PickFlow", lock_tag, pick_z)
    log("PickFlow", "centered_xy=(%.4f, %.4f) and pick step complete" % (centered.getX(), centered.getY()))
    return centered


def pick_die_for_final_storage_return(camera, nozzle, part, expected_die_loc, cfg, state, correction=None):
    expected_die_loc = expected_die_loc.convertToUnits(LengthUnit.Millimeters)
    lock, camera_center = acquire_work_die_pose(camera, nozzle, part, expected_die_loc, cfg, "FinalReturnPick")
    pose = lock["pose"]
    detected_center = lock.get("centered_tool_location", camera_center)
    detected_center = detected_center.convertToUnits(LengthUnit.Millimeters)
    detected_r = pose_rotation_for_pick(pose, cfg)
    raw_pick_target = mm_loc(detected_center.getX(), detected_center.getY(),
                             expected_die_loc.getZ(), detected_r)

    dx = raw_pick_target.getX() - expected_die_loc.getX()
    dy = raw_pick_target.getY() - expected_die_loc.getY()
    mag = math.sqrt(dx * dx + dy * dy)
    log("FinalReturnPick", "expected_die_loc=(%.4f, %.4f, %.4f)" %
        (expected_die_loc.getX(), expected_die_loc.getY(), expected_die_loc.getRotation()))
    log("FinalReturnPick", "detected_die_center=(%.4f, %.4f, %.4f)" %
        (raw_pick_target.getX(), raw_pick_target.getY(), detected_r))
    log("FinalReturnPick", "detected_center_delta=(%.5f, %.5f, %.5f)" %
        (dx, dy, mag))
    log("FinalReturnPick", "detected_rotation=%.4f" % detected_r)

    max_centering_error = float(cfg.get("final_return_pick_max_centering_error_mm", 0.75))
    if mag > max_centering_error:
        raise Exception("[FinalReturnPick] detected center delta %.5f mm exceeds final_return_pick_max_centering_error_mm %.5f" %
                        (mag, max_centering_error))

    if bool(cfg.get("align_nozzle_to_detected_die_rotation", True)) and bool(pose.get("orientation_found", False)):
        validation_r = pose_rotation_for_validation(pose, cfg)
        log("FinalReturnPick", "Aligning nozzle rotation to detected die raw rotation %.4f deg; snapped/cardinal=%.4f deg" %
            (detected_r, validation_r))
    elif bool(cfg.get("align_nozzle_to_detected_die_rotation", True)):
        log("FinalReturnPick", "Orientation circle not found; leaving nozzle rotation unchanged.")

    if bool(cfg.get("require_visual_lock_before_pick", True)) and int(pose.get("circle_count", 4)) < 4:
        raise Exception("[FinalReturnPick] pick aborted: visual lock count < 4")

    pick_z = pick_z_for(expected_die_loc, cfg)
    corrected_pick_target = pickup_loc_for_detected_die_center(raw_pick_target, correction, detected_r, cfg)
    log_coordinate_mode("FinalReturnPick", expected_die_loc, corrected_pick_target, pick_z, cfg, raw_pick_target)
    log("FinalReturnPick", "final_return_pick_detected_vs_descent detected_center=(%.4f, %.4f) descent_xy=(%.4f, %.4f) applied_delta=(%.5f, %.5f)" %
        (raw_pick_target.getX(), raw_pick_target.getY(),
         corrected_pick_target.getX(), corrected_pick_target.getY(),
         corrected_pick_target.getX() - raw_pick_target.getX(),
         corrected_pick_target.getY() - raw_pick_target.getY()))
    guarded_descent(nozzle, corrected_pick_target, pick_z, cfg, "PickFlow", "Cal")
    log("FinalReturnPick", "Picking die part '%s' from freshly detected center" % part.getId())
    feeder_name = cfg.get("feeder_name_override", None)
    log("Pick", "Resolving feeder override: %s" % feeder_name)
    try:
        feeder = find_feeder_by_name(get_machine(), feeder_name)
    except Exception, feeder_error:
        log("Pick", "Feeder override lookup failed, continuing with feeder=None: %s" % feeder_error)
        feeder = None
    log("Pick", "Feeder resolved: %s" % object_name(feeder))
    try:
        pick_path = do_pick(nozzle, part, feeder, cfg)
    except Throwable, throwable:
        log("Pick", "Pick actuation path failed before state update: %s" % throwable)
        raise
    verify_part_on_after_pick(nozzle)
    wait_still(nozzle)

    try:
        actual_pick_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        if state is not None:
            state["last_pick_x_mm"] = float(actual_pick_loc.getX())
            state["last_pick_y_mm"] = float(actual_pick_loc.getY())
            state["last_pick_rotation_deg"] = normalize_angle(float(actual_pick_loc.getRotation()))
            state["last_pick_detected_rotation_deg"] = normalize_angle(float(detected_r))
        log("FinalReturnPick", "pick_pose actual=(%.4f, %.4f, %.4f) detected_rotation=%.4f" %
            (actual_pick_loc.getX(), actual_pick_loc.getY(),
             normalize_angle(float(actual_pick_loc.getRotation())), detected_r))
    except Exception, pick_pose_error:
        log("FinalReturnPick", "Could not record pick pose: %s" % pick_pose_error)

    if state is not None:
        state["die_on_nozzle"] = True
        state["die_at_work"] = False
        state["current_die_loc_trusted"] = False
    log("FinalReturnPick", "Successful final return pick with nozzle_tip=%s path=%s" %
        (current_nozzle_tip_name(nozzle), pick_path))
    log("Sequence", "Step complete: pick_cal_for_final_storage_return")
    lift_nozzle_after_pick_place(nozzle, cfg, "PickFlow", "Cal", pick_z)
    log("PickFlow", "centered_xy=(%.4f, %.4f) and final return pick step complete" %
        (corrected_pick_target.getX(), corrected_pick_target.getY()))
    return corrected_pick_target


def place_die_with_visual_lock(camera, nozzle, part, target_loc, cfg, tag, placement_target_loc=None, cycle_state=None):
    lock_tag = vision_lock_name(tag)
    # placement_target_loc is a complete nozzle placement target (X/Y/Z/R), not just XY.
    place_xy = placement_target_loc
    computed_xy = place_xy
    if place_xy is None:
        if bool(cfg.get("require_visual_lock_before_pick", True)):
            camera_z = camera_z_for_location(target_loc, cfg)
            camera_target_loc = tool_aware_camera_target_for_location(camera, nozzle, target_loc, camera_z)
            lock = acquire_die_pose_at_location(camera, nozzle, part, camera_target_loc, camera_z, cfg, lock_tag)
            computed_xy = lock["centered_tool_location"]
            if cfg.get("pick_place_xy_mode", "detected_center") == "configured":
                place_xy = target_loc
            else:
                place_xy = computed_xy
        else:
            place_xy = target_loc
            computed_xy = place_xy

    place_z = pick_z_for(target_loc, cfg)
    log_coordinate_mode(tag, target_loc, place_xy, place_z, cfg, computed_xy)
    guarded_descent(nozzle, place_xy, place_z, cfg, "PickFlow", lock_tag)
    log(tag, "Placing die")
    log_coordinate_mode(tag, target_loc, place_xy, place_z, cfg, computed_xy)
    do_place(nozzle, part, cfg)
    sleep_ms(cfg.get("blowoff_pulse_ms", 120))
    verify_part_off_after_place(nozzle)
    wait_still(nozzle)
    if cycle_state is not None:
        cycle_state["die_on_nozzle"] = False
        if lock_tag == "Cal":
            cycle_state["die_at_work"] = True
            cycle_state["current_die_loc_trusted"] = False
        elif lock_tag == "Storage":
            cycle_state["die_at_work"] = False
            cycle_state["current_die_loc_trusted"] = False
    if lock_tag == "Cal":
        log("Sequence", "Placed at calibration location")
    elif lock_tag == "Storage":
        log("Sequence", "Returned die to storage")
    lift_nozzle_after_pick_place(nozzle, cfg, "PickFlow", lock_tag, place_z)
    log("PickFlow", "centered_xy=(%.4f, %.4f) and place step complete" % (place_xy.getX(), place_xy.getY()))
    return place_xy


def die_loc_from_lock(lock, fallback_loc):
    pose = lock["pose"]
    x = float(pose.get("centered_machine_x_mm", fallback_loc.getX()))
    y = float(pose.get("centered_machine_y_mm", fallback_loc.getY()))
    if bool(pose.get("orientation_found", False)):
        r = pose_rotation_for_pick(pose, {})
    else:
        r = normalize_angle(float(fallback_loc.getRotation()))
    return mm_loc(x, y, fallback_loc.getZ(), r)


def acquire_work_die_pose(camera, nozzle, part, expected_loc, cfg, tag):
    expected_loc = expected_loc.convertToUnits(LengthUnit.Millimeters)
    camera_z = camera_z_for_cal(expected_loc, cfg)
    camera_target = camera_measurement_loc(camera, expected_loc.getX(), expected_loc.getY(), camera_z)
    lock = acquire_die_pose_at_location(camera, nozzle, part, camera_target, camera_z, cfg, tag)
    return lock, die_loc_from_lock(lock, expected_loc)


def move_entry_value(entry, names, default_value):
    for name in names:
        try:
            if name in entry:
                return float(entry[name])
        except:
            pass
    return float(default_value)


def calibration_move_entry(cfg, sample_index):
    moves = cfg.get("calibration_moves", [])
    entry = moves[int(sample_index) % len(moves)]
    if isinstance(entry, dict):
        return entry
    try:
        return {"dx_mm": float(entry[0]), "dy_mm": float(entry[1])}
    except:
        raise Exception("Invalid calibration_moves entry at index %d: %s" % (int(sample_index) % len(moves), entry))


def calibration_target_for_sample(cal_loc, cfg, sample_index, angle_deg):
    entry = calibration_move_entry(cfg, sample_index)
    step = float(cfg.get("calibration_step_mm", 0.5))
    dx = move_entry_value(entry, ["dx_mm", "dx", "x_mm", "x"], 0.0)
    dy = move_entry_value(entry, ["dy_mm", "dy", "y_mm", "y"], 0.0)
    if "dx_mm" not in entry and "x_mm" not in entry:
        dx *= step
    if "dy_mm" not in entry and "y_mm" not in entry:
        dy *= step

    target_r = float(angle_deg)
    if "rotation_deg" in entry:
        target_r = float(entry["rotation_deg"])
    elif "r_deg" in entry:
        target_r = float(entry["r_deg"])
    elif "drotation_deg" in entry:
        target_r = cal_loc.getRotation() + float(entry["drotation_deg"])
    elif "rotation_delta_deg" in entry:
        target_r = cal_loc.getRotation() + float(entry["rotation_delta_deg"])

    return mm_loc(cal_loc.getX() + dx, cal_loc.getY() + dy, cal_loc.getZ(), normalize_angle(target_r))


def placement_loc_for_target(target_loc, correction, pick_rotation_deg=None):
    if correction is None:
        correction = {}
    model = correction.get("model", "")
    if bool(correction.get("use_pivot_correction", True)) and model == "bias_plus_pick_place_vectors":
        theta = float(correction.get("theta", correction.get("theta_error_deg",
                      correction.get("rotation_error_deg", 0.0))))
        command_r = normalize_angle(float(target_loc.getRotation()) - theta)
        bias_x = float(correction.get("bias_x_mm", correction.get("offset_x_mm", 0.0)))
        bias_y = float(correction.get("bias_y_mm", correction.get("offset_y_mm", 0.0)))
        place_vx = float(correction.get("place_vector_x_mm", 0.0))
        place_vy = float(correction.get("place_vector_y_mm", 0.0))
        pick_vx = float(correction.get("pick_vector_x_mm", 0.0))
        pick_vy = float(correction.get("pick_vector_y_mm", 0.0))
        if pick_rotation_deg is None:
            pick_rotation_deg = command_r
            log("PlaceCommand", "WARNING pick_rotation_deg missing for pick/place model; using command_r %.4f" %
                command_r)
        place_term = rotate_xy(place_vx, place_vy, command_r)
        pick_term = rotate_xy(pick_vx, pick_vy, pick_rotation_deg)
        command_x = float(target_loc.getX()) - bias_x - place_term["x"] + pick_term["x"]
        command_y = float(target_loc.getY()) - bias_y - place_term["y"] + pick_term["y"]
        log("PlaceCommand", "model=%s target=(%.4f, %.4f, %.4f) pick_r=%.4f command_r=%.4f bias=(%.5f, %.5f) place_term=(%.5f, %.5f) pick_term=(%.5f, %.5f) command=(%.4f, %.4f, %.4f)" %
            (model, target_loc.getX(), target_loc.getY(), target_loc.getRotation(),
             float(pick_rotation_deg), command_r, bias_x, bias_y,
             place_term["x"], place_term["y"], pick_term["x"], pick_term["y"],
             command_x, command_y, command_r))
        return mm_loc(command_x, command_y, target_loc.getZ(), command_r)

    if bool(correction.get("use_pivot_correction", True)) and model == "bias_plus_rotating_pivot_vector":
        theta = float(correction.get("theta", correction.get("theta_error_deg",
                      correction.get("rotation_error_deg", 0.0))))
        command_r = normalize_angle(float(target_loc.getRotation()) - theta)
        bias_x = float(correction.get("bias_x_mm", correction.get("offset_x_mm", 0.0)))
        bias_y = float(correction.get("bias_y_mm", correction.get("offset_y_mm", 0.0)))
        vx = float(correction.get("pivot_vector_x_mm", 0.0))
        vy = float(correction.get("pivot_vector_y_mm", 0.0))
        rotated_v = rotate_xy(vx, vy, command_r)
        command_x = float(target_loc.getX()) - bias_x - rotated_v["x"]
        command_y = float(target_loc.getY()) - bias_y - rotated_v["y"]
        log("PlaceCommand", "target=(%.4f, %.4f, %.4f) model=%s command=(%.4f, %.4f, %.4f) rotated_v=(%.5f, %.5f) bias=(%.5f, %.5f)" %
            (target_loc.getX(), target_loc.getY(), target_loc.getRotation(), model,
             command_x, command_y, command_r, rotated_v["x"], rotated_v["y"], bias_x, bias_y))
        return mm_loc(command_x, command_y, target_loc.getZ(), command_r)

    dx = float(correction.get("x", 0.0))
    dy = float(correction.get("y", 0.0))
    theta = float(correction.get("theta", correction.get("rotation_error_deg", 0.0)))

    command_r = normalize_angle(target_loc.getRotation() - theta)
    command = mm_loc(target_loc.getX() - dx,
                     target_loc.getY() - dy,
                     target_loc.getZ(),
                     command_r)
    log("PlaceCommand", "target=(%.4f, %.4f, %.4f) model=simple_xy command=(%.4f, %.4f, %.4f) correction=(%.5f, %.5f, %.5f)" %
        (target_loc.getX(), target_loc.getY(), target_loc.getRotation(),
         command.getX(), command.getY(), command.getRotation(), dx, dy, theta))
    return command


def pickup_loc_for_detected_die_center(detected_die_center, correction, pick_rotation_deg, cfg):
    detected_die_center = detected_die_center.convertToUnits(LengthUnit.Millimeters)
    command_r = normalize_angle(float(pick_rotation_deg))
    raw_command = mm_loc(detected_die_center.getX(), detected_die_center.getY(),
                         detected_die_center.getZ(), command_r)
    model = None
    if correction is not None:
        try:
            model = correction.get("model", None)
        except:
            model = None

    log("FinalReturnPick", "pickup_correction_model=%s" % (model if model is not None else "none"))
    log("FinalReturnPick", "pick_rotation_deg=%.4f" % command_r)
    log("FinalReturnPick", "raw_pick_command=(%.4f, %.4f, %.4f)" %
        (raw_command.getX(), raw_command.getY(), raw_command.getRotation()))

    if model == "bias_plus_pick_place_vectors":
        pick_vx = float(correction.get("pick_vector_x_mm", 0.0))
        pick_vy = float(correction.get("pick_vector_y_mm", 0.0))
        pick_term = rotate_xy(pick_vx, pick_vy, command_r)
        sign = float(cfg.get("final_pickup_pick_vector_sign", 1.0))
        log("FinalReturnPick", "pickup correction not applied for final storage return; commanding detected center directly")
        log("FinalReturnPick", "pick_vector_raw=(%.5f, %.5f)" % (pick_vx, pick_vy))
        log("FinalReturnPick", "pick_vector_rotated=(%.5f, %.5f)" %
            (pick_term["x"], pick_term["y"]))
        log("FinalReturnPick", "final_pickup_pick_vector_sign=%.1f" % sign)
        log("FinalReturnPick", "applied_pick_correction=(0.00000, 0.00000)")
        log("FinalReturnPick", "corrected_pick_command=(%.4f, %.4f, %.4f)" %
            (raw_command.getX(), raw_command.getY(), raw_command.getRotation()))
        log("FinalReturnPick", "corrected_pick_delta_from_detected_center=(0.00000, 0.00000, 0.00000)")
        return raw_command

    if not bool(cfg.get("final_pickup_apply_pick_vector", True)):
        log("FinalReturnPick", "pickup correction not used; commanding detected center directly")
        log("FinalReturnPick", "pick_vector_raw=(0.00000, 0.00000)")
        log("FinalReturnPick", "pick_vector_rotated=(0.00000, 0.00000)")
        log("FinalReturnPick", "final_pickup_pick_vector_sign=%.1f" %
            float(cfg.get("final_pickup_pick_vector_sign", 1.0)))
        log("FinalReturnPick", "corrected_pick_command=(%.4f, %.4f, %.4f)" %
            (raw_command.getX(), raw_command.getY(), raw_command.getRotation()))
        log("FinalReturnPick", "corrected_pick_delta_from_detected_center=(0.00000, 0.00000, 0.00000)")
        return raw_command
    if correction is None:
        log("FinalReturnPick", "pickup correction not used; commanding detected center directly")
        log("FinalReturnPick", "pick_vector_raw=(0.00000, 0.00000)")
        log("FinalReturnPick", "pick_vector_rotated=(0.00000, 0.00000)")
        log("FinalReturnPick", "final_pickup_pick_vector_sign=%.1f" %
            float(cfg.get("final_pickup_pick_vector_sign", 1.0)))
        log("FinalReturnPick", "corrected_pick_command=(%.4f, %.4f, %.4f)" %
            (raw_command.getX(), raw_command.getY(), raw_command.getRotation()))
        log("FinalReturnPick", "corrected_pick_delta_from_detected_center=(0.00000, 0.00000, 0.00000)")
        return raw_command
    log("FinalReturnPick", "WARNING pickup correction model '%s' is unsupported for pickup centering; commanding detected center directly" %
        (model if model is not None else "none"))
    log("FinalReturnPick", "pickup correction not used; commanding detected center directly")
    log("FinalReturnPick", "pick_vector_raw=(0.00000, 0.00000)")
    log("FinalReturnPick", "pick_vector_rotated=(0.00000, 0.00000)")
    log("FinalReturnPick", "final_pickup_pick_vector_sign=%.1f" %
        float(cfg.get("final_pickup_pick_vector_sign", 1.0)))
    log("FinalReturnPick", "applied_pick_correction=(0.00000, 0.00000)")
    log("FinalReturnPick", "corrected_pick_command=(%.4f, %.4f, %.4f)" %
        (raw_command.getX(), raw_command.getY(), raw_command.getRotation()))
    log("FinalReturnPick", "corrected_pick_delta_from_detected_center=(0.00000, 0.00000, 0.00000)")
    return raw_command


def configured_storage_return_target(storage_loc):
    storage_loc = storage_loc.convertToUnits(LengthUnit.Millimeters)
    return mm_loc(storage_loc.getX(), storage_loc.getY(), storage_loc.getZ(),
                  storage_loc.getRotation())


def storage_return_command_target(storage_target_loc, correction=None, offsets_applied=False, pick_rotation_deg=None):
    storage_target_loc = storage_target_loc.convertToUnits(LengthUnit.Millimeters)
    target = mm_loc(storage_target_loc.getX(), storage_target_loc.getY(),
                    storage_target_loc.getZ(), storage_target_loc.getRotation())

    if correction is None:
        correction = {"x": 0.0, "y": 0.0, "theta": 0.0}

    if bool(offsets_applied):
        # XY offset has already been written to machine config.
        # Do not subtract X/Y again.
        # Theta is not written to machine config by the current apply path,
        # so theta should still be applied if present.
        theta_only = {
            "x": 0.0,
            "y": 0.0,
            "theta": float(correction.get("theta",
                     correction.get("theta_error_deg",
                     correction.get("rotation_error_deg", 0.0))))
        }
        return placement_loc_for_target(target, theta_only, pick_rotation_deg)

    return placement_loc_for_target(target, correction, pick_rotation_deg)


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


def correction_model_name(correction):
    if correction is None:
        return None
    try:
        return correction.get("model", "translation_theta")
    except:
        return "translation_theta"


def command_delta_from_target(command_loc, target_loc):
    command_loc = command_loc.convertToUnits(LengthUnit.Millimeters)
    target_loc = target_loc.convertToUnits(LengthUnit.Millimeters)
    dx = command_loc.getX() - target_loc.getX()
    dy = command_loc.getY() - target_loc.getY()
    return dx, dy, math.sqrt(dx * dx + dy * dy)


def overlay_with_command_target(camera, overlay, command_loc, reference_loc):
    if overlay is None:
        return None
    out = dict(overlay)
    expected = out.get("expected_center_px", None)
    if expected is None:
        return out
    try:
        upp = camera_units_per_pixel(camera)
        if upp is None:
            return out
        ux = float(upp.getX())
        uy = float(upp.getY())
        if abs(ux) < 0.0000001 or abs(uy) < 0.0000001:
            return out
        dx = float(command_loc.getX()) - float(reference_loc.getX())
        dy = float(command_loc.getY()) - float(reference_loc.getY())
        out["expected_center_px"] = {
            "x": float(expected.get("x", 0.0)) + dx / ux,
            "y": float(expected.get("y", 0.0)) + dy / uy
        }
        out["expected_command_delta_mm"] = {"x": dx, "y": dy}
        out["expected_command_x_mm"] = float(command_loc.getX())
        out["expected_command_y_mm"] = float(command_loc.getY())
    except:
        pass
    return out


def acquire_fresh_confirm_storage_pose(camera, nozzle, part, storage_target, cfg, preview_command):
    camera_z = camera_z_for_storage(storage_target, cfg)
    camera_target = camera_measurement_loc(camera, storage_target.getX(), storage_target.getY(), camera_z)
    log("Confirm", "fresh_detection=required camera_target=(%.4f, %.4f, %.4f)" %
        (camera_target.getX(), camera_target.getY(), camera_z))
    lock = acquire_die_pose_at_location(camera, nozzle, part, camera_target, camera_z, cfg, "Confirm")
    pose = lock["pose"]
    detected_loc = die_loc_from_lock(lock, storage_target)
    log("Confirm", "fresh_detection center=(%.4f, %.4f, %.4f) circles=%d" %
        (detected_loc.getX(), detected_loc.getY(), detected_loc.getRotation(),
         int(pose.get("circle_count", 0))))
    return lock, detected_loc, pose.get("overlay", None)


def sample_from_measured_pose(target_loc, placement_loc, actual_loc, pose, correction, label, sample_index, cfg,
                              pick_rotation_deg=None):
    error_x = actual_loc.getX() - target_loc.getX()
    error_y = actual_loc.getY() - target_loc.getY()
    actual_r = pose_rotation_for_pick(pose, cfg)
    target_r = normalize_angle(float(target_loc.getRotation()))
    snapped_r = pose.get("rotation_snapped_deg", pose.get("orientation_rotation_deg", None))
    rotation_error = normalize_angle(actual_r - target_r)
    return {
        "label": label,
        "sample_index": int(sample_index),
        "commanded_angle_deg": target_r,
        "pick_rotation_deg": normalize_angle(float(pick_rotation_deg if pick_rotation_deg is not None else placement_loc.getRotation())),
        "place_rotation_deg": normalize_angle(float(placement_loc.getRotation())),
        "target_x_mm": float(target_loc.getX()),
        "target_y_mm": float(target_loc.getY()),
        "target_rotation_deg": target_r,
        "placement_x_mm": float(placement_loc.getX()),
        "placement_y_mm": float(placement_loc.getY()),
        "placement_model": correction.get("model", "simple_xy"),
        "placement_correction_x_mm": float(correction.get("x", 0.0)),
        "placement_correction_y_mm": float(correction.get("y", 0.0)),
        "placement_bias_x_mm": float(correction.get("bias_x_mm", 0.0)),
        "placement_bias_y_mm": float(correction.get("bias_y_mm", 0.0)),
        "placement_pivot_vector_x_mm": float(correction.get("pivot_vector_x_mm", 0.0)),
        "placement_pivot_vector_y_mm": float(correction.get("pivot_vector_y_mm", 0.0)),
        "actual_x_mm": float(actual_loc.getX()),
        "actual_y_mm": float(actual_loc.getY()),
        "actual_rotation_deg": actual_r,
        "actual_rotation_raw_deg": actual_r,
        "actual_rotation_snapped_deg": snapped_r,
        "error_x_mm": error_x,
        "error_y_mm": error_y,
        "rotation_error_deg": rotation_error,
        "orientation_found": pose.get("orientation_found", False),
        "rotation_raw_deg": pose.get("rotation_raw_deg", None),
        "rotation_snapped_deg": pose.get("rotation_snapped_deg", None),
        "orientation_raw_rotation_deg": pose.get("orientation_raw_rotation_deg", None),
        "orientation_rotation_deg": pose.get("orientation_rotation_deg", None),
        "orientation_snap_error_deg": pose.get("orientation_snap_error_deg", None),
        "orientation_stage_name": pose.get("orientation_stage_name", None),
        "orientation_x_mm": pose.get("orientation_x_mm", None),
        "orientation_y_mm": pose.get("orientation_y_mm", None),
        "orientation_dx_mm": pose.get("orientation_dx_mm", None),
        "orientation_dy_mm": pose.get("orientation_dy_mm", None),
        "square_edge_rotation_deg": pose.get("square_edge_rotation_deg", None),
        "rotation_source": pose.get("rotation_source", None),
        "side_mm": pose["side_mm"],
        "side_rms_mm": pose["side_rms_mm"],
        "search_dx_mm": pose.get("search_dx_mm", 0.0),
        "search_dy_mm": pose.get("search_dy_mm", 0.0),
        "corners": pose["corners"],
        "overlay": pose.get("overlay", None)
    }


def run_pick_place_measure_cycles(camera, nozzle, part, storage_loc, cal_loc, cfg, iterations, correction, label):
    state = {"die_on_nozzle": False, "die_at_work": False,
             "current_die_loc": None, "current_die_loc_trusted": False,
             "last_trusted_die_loc": None, "detected_storage_pick_loc": None,
             "last_pick_x_mm": None, "last_pick_y_mm": None,
             "last_pick_rotation_deg": None, "last_pick_detected_rotation_deg": None}
    detected_storage_pick_loc = transfer_die_from_storage_to_cal(camera, nozzle, part, storage_loc, cal_loc, cfg, state)
    try:
        samples, current_die_loc = run_calibration_zone_measure_cycles(
            camera, nozzle, part, cal_loc, cfg, int(iterations), correction, label, state
        )
        state["current_die_loc"] = current_die_loc
        return_die_to_storage(camera, nozzle, part, storage_loc, cfg, state, detected_storage_pick_loc,
                              correction, False)
        return samples
    except Exception, e:
        cleanup_after_calibration_failure(camera, nozzle, part, storage_loc, cal_loc, cfg, state, str(e))
        raise


def transfer_die_from_storage_to_cal(camera, nozzle, part, storage_loc, cal_loc, cfg, state):
    prepare_nozzle_for_part(nozzle, part)
    log("StoragePick", "configured_search=(%.4f, %.4f, %.4f, %.4f)" %
        (storage_loc.getX(), storage_loc.getY(), storage_loc.getZ(), storage_loc.getRotation()))
    detected_storage_pick_loc = pick_die_with_visual_lock(camera, nozzle, part, storage_loc, cfg, "Storage", state)
    state["detected_storage_pick_loc"] = detected_storage_pick_loc
    log("StoragePick", "detected_die_pick=(%.4f, %.4f, %.4f, %.4f)" %
        (detected_storage_pick_loc.getX(), detected_storage_pick_loc.getY(),
         detected_storage_pick_loc.getZ(), detected_storage_pick_loc.getRotation()))
    log("StoragePick", "detected_storage_pick_loc is pick-only and will not be used for calibration offsets or storage return target.")
    log("StoragePick", "detected_minus_configured=(%.5f, %.5f, %.5f)" %
        (detected_storage_pick_loc.getX() - storage_loc.getX(),
         detected_storage_pick_loc.getY() - storage_loc.getY(),
         normalize_angle(detected_storage_pick_loc.getRotation() - storage_loc.getRotation())))
    log("Sequence", "Storage pickup complete. Storage offset is not used for calibration.")
    log("Sequence", "Step complete: pick_storage")
    cal_place_loc = mm_loc(cal_loc.getX(), cal_loc.getY(), cal_loc.getZ(), cal_loc.getRotation())
    place_die_with_visual_lock(camera, nozzle, part, cal_loc, cfg, "Cal", cal_place_loc, state)
    log("Sequence", "Step complete: place_initial_cal")
    log("Sequence", "Calibration begins after initial cal placement and cal vision measurement.")
    lock, current_die_loc = acquire_work_die_pose(camera, nozzle, part, cal_loc, cfg, "Cal")
    validate_pose_orientation(lock["pose"], cal_loc, cfg, "initial calibration pose")
    mark_current_die_loc_trusted(state, current_die_loc, True)
    state["die_at_work"] = True
    log("Sequence", "Initial calibration pose: X=%.4f Y=%.4f R=%.4f" %
        (current_die_loc.getX(), current_die_loc.getY(), current_die_loc.getRotation()))
    return detected_storage_pick_loc


def run_orientation_sign_probe(camera, nozzle, part, cal_loc, cfg, state):
    cfg["_orientation_runtime_sign_multiplier"] = 1.0
    if not bool(cfg.get("orientation_sign_probe_enabled", True)):
        log("ThetaProbe", "disabled; using orientation_x_sign=%.1f runtime_sign=1.0" %
            float(cfg.get("orientation_x_sign", -1.0)))
        return {"enabled": False, "selected_orientation_sign": 1.0}

    current_die_loc = state.get("current_die_loc", cal_loc)
    lock, base_loc = acquire_work_die_pose(camera, nozzle, part, current_die_loc, cfg, "Cal")
    base_pose = lock["pose"]
    base_raw = pose_rotation_for_pick(base_pose, cfg)
    base_target_r = nearest_cardinal_angle(base_raw) if bool(cfg.get("orientation_snap_to_cardinal", True)) else base_raw
    log("ThetaProbe", "base_raw=%.5f base_target=%.5f" % (base_raw, base_target_r))

    probes = []
    deltas = cfg.get("orientation_sign_probe_degrees", [12.0, -12.0])
    for i in range(2):
        command_delta = float(deltas[i])
        target_r = normalize_angle(base_target_r + command_delta)
        target = mm_loc(cal_loc.getX(), cal_loc.getY(), cal_loc.getZ(), target_r)
        pick_die_with_visual_lock(camera, nozzle, part, current_die_loc, cfg, "Cal", state)
        place_die_with_visual_lock(camera, nozzle, part, target, cfg, "Cal", target, state)
        lock, actual_loc = acquire_work_die_pose(camera, nozzle, part, target, cfg, "Cal")
        measured_raw = pose_rotation_for_pick(lock["pose"], cfg)
        measured_delta = rotation_error_deg(measured_raw, base_raw)
        log("ThetaProbe", "command_delta=%+.3f measured_delta=%+.5f measured_raw=%.5f" %
            (command_delta, measured_delta, measured_raw))
        probes.append({
            "command_delta_deg": command_delta,
            "measured_delta_deg": measured_delta,
            "measured_raw_deg": measured_raw
        })
        current_die_loc = actual_loc
        mark_current_die_loc_trusted(state, current_die_loc, True)

    same_score = mean([abs(rotation_error_deg(p["measured_delta_deg"], p["command_delta_deg"])) for p in probes])
    inverted_score = mean([abs(rotation_error_deg(-p["measured_delta_deg"], p["command_delta_deg"])) for p in probes])
    selected = 1.0
    if inverted_score < same_score:
        selected = -1.0
    log("ThetaProbe", "score_if_current=%.5f score_if_flipped=%.5f max_error=%.5f" %
        (same_score, inverted_score, float(cfg.get("orientation_sign_probe_max_error_deg", 3.0))))
    log("ThetaProbe", "selected_orientation_sign=%+.0f" % selected)
    selected_score = same_score if selected > 0.0 else inverted_score
    if selected_score > float(cfg.get("orientation_sign_probe_max_error_deg", 3.0)):
        log("ThetaProbe", "WARNING selected sign still has %.5f deg mean error above threshold; inspect raw probe data before trusting calibration." %
            selected_score)

    if selected < 0.0:
        log("ThetaProbe", "current orientation_x_sign appears incorrect")
    else:
        log("ThetaProbe", "current orientation_x_sign appears correct")

    if selected < 0.0 and bool(cfg.get("auto_update_orientation_x_sign", False)):
        old_sign = float(cfg.get("orientation_x_sign", -1.0))
        cfg["orientation_x_sign"] = -old_sign
        cfg["_orientation_runtime_sign_multiplier"] = 1.0
        saved = {}
        try:
            f = open(config_path(), "r")
            try:
                saved = json.loads(f.read())
            finally:
                f.close()
        except:
            saved = dict(DEFAULT_CONFIG)
        saved["orientation_x_sign"] = cfg["orientation_x_sign"]
        save_config(saved)
        log("ThetaProbe", "auto_update_orientation_x_sign=True; wrote orientation_x_sign %.1f -> %.1f" %
            (old_sign, cfg["orientation_x_sign"]))
    else:
        cfg["_orientation_runtime_sign_multiplier"] = selected
        if selected < 0.0:
            log("ThetaProbe", "auto_update_orientation_x_sign=False; using runtime sign multiplier -1 for this run only.")

    lock, actual_loc = acquire_work_die_pose(camera, nozzle, part, current_die_loc, cfg, "Cal")
    mark_current_die_loc_trusted(state, actual_loc, True)
    state["die_at_work"] = True
    return {
        "enabled": True,
        "base_raw_deg": base_raw,
        "base_target_deg": base_target_r,
        "probes": probes,
        "score_if_current": same_score,
        "score_if_flipped": inverted_score,
        "selected_orientation_sign": selected,
        "runtime_sign_multiplier": float(cfg.get("_orientation_runtime_sign_multiplier", 1.0))
    }


def rebaseline_after_theta_probe(camera, nozzle, part, cal_loc, cfg, state):
    current_die_loc = state.get("current_die_loc", cal_loc)
    pick_die_with_visual_lock(camera, nozzle, part, current_die_loc, cfg, "Cal", state)
    target = mm_loc(cal_loc.getX(), cal_loc.getY(), cal_loc.getZ(), 0.0)
    place_die_with_visual_lock(camera, nozzle, part, target, cfg, "Cal", target, state)
    lock, actual_loc = acquire_work_die_pose(camera, nozzle, part, target, cfg, "Cal")
    validate_pose_orientation(lock["pose"], target, cfg, "rebaseline after theta probe")
    mark_current_die_loc_trusted(state, actual_loc, True)
    state["die_at_work"] = True
    log("Rebaseline", "After theta probe, re-placed die at cal origin before pivot fit.")
    return actual_loc


def run_pivot_probe(camera, nozzle, part, cal_loc, cfg, state):
    if not bool(cfg.get("pivot_probe_enabled", True)) or not bool(cfg.get("use_pivot_correction", True)):
        log("PivotProbe", "disabled; simple XY correction fallback remains available.")
        return None, []

    log("Model", "simple_xy correction disabled because errors vary with rotation")
    angles = cfg.get("pivot_probe_angles_deg", DEFAULT_CONFIG["pivot_probe_angles_deg"])
    xy = cfg.get("pivot_probe_xy_mm", {"x": 0.0, "y": 0.0})
    target_x = cal_loc.getX() + float(xy.get("x", 0.0))
    target_y = cal_loc.getY() + float(xy.get("y", 0.0))
    samples = []
    current_die_loc = state.get("current_die_loc", cal_loc)

    for sample_index in range(len(angles)):
        check_cancel("PivotProbe")
        angle = normalize_angle(float(angles[sample_index]))
        target = mm_loc(target_x, target_y, cal_loc.getZ(), angle)
        picked_loc = pick_die_with_visual_lock(camera, nozzle, part, current_die_loc, cfg, "Cal", state)
        pick_r = float(state.get("last_pick_rotation_deg", picked_loc.getRotation()))
        log("PivotProbe", "sample=%d pick_rotation_deg=%.5f place_rotation_deg=%.5f" %
            (sample_index + 1, pick_r, target.getRotation()))
        place_die_with_visual_lock(camera, nozzle, part, target, cfg, "Cal", target, state)
        lock, actual_loc = acquire_work_die_pose(camera, nozzle, part, target, cfg, "Cal")
        pose = lock["pose"]
        validate_pose_orientation(pose, target, cfg, "pivot probe %.3f" % angle)
        sample = sample_from_measured_pose(target, target, actual_loc, pose,
                                           {"x": 0.0, "y": 0.0, "theta": 0.0},
                                           "pivot", sample_index, cfg, pick_r)
        samples.append(sample)
        current_die_loc = actual_loc
        mark_current_die_loc_trusted(state, current_die_loc, True)
        log("PivotProbe", "angle=%.3f command=(%.4f, %.4f, %.4f) actual=(%.4f, %.4f, %.4f) error=(%.5f, %.5f, %.5f)" %
            (angle, target.getX(), target.getY(), target.getRotation(),
             actual_loc.getX(), actual_loc.getY(), sample["actual_rotation_deg"],
             sample["error_x_mm"], sample["error_y_mm"], sample["rotation_error_deg"]))

    computed = fit_bias_plus_rotating_pivot(samples)
    log("PivotFit", "bias=(%.5f, %.5f)" %
        (computed["bias_x_mm"], computed["bias_y_mm"]))
    if computed.get("model", "") == "bias_plus_pick_place_vectors":
        log("PivotFit", "place_vector=(%.5f, %.5f) pick_vector=(%.5f, %.5f)" %
            (computed["place_vector_x_mm"], computed["place_vector_y_mm"],
             computed["pick_vector_x_mm"], computed["pick_vector_y_mm"]))
        log("PivotFit", "place_vector_mag=%.5f pick_vector_mag=%.5f" %
            (computed["place_vector_mag_mm"], computed["pick_vector_mag_mm"]))
    else:
        log("PivotFit", "local_pivot_vector=(%.5f, %.5f)" %
            (computed["pivot_vector_x_mm"], computed["pivot_vector_y_mm"]))
        log("PivotFit", "vector_mag=%.5f" % computed["pivot_vector_mag_mm"])
    log("PivotFit", "residual_rms=%.5f" % computed["residual_rms_mm"])
    state["current_die_loc"] = current_die_loc
    return computed, samples


def run_calibration_zone_measure_cycles(camera, nozzle, part, cal_loc, cfg, iterations, correction, label, state):
    samples = []
    angles = cfg.get("angles_deg", DEFAULT_CONFIG["angles_deg"])
    current_die_loc = state.get("current_die_loc", cal_loc)
    stage_tag = "Compute" if label == "test" else "Verify"

    for sample_index in range(int(iterations)):
        check_cancel(stage_tag)
        angle = float(angles[sample_index % len(angles)])
        log(stage_tag, "Sample %d of %d" % (sample_index + 1, int(iterations)))
        check_cancel("Vision")
        target_loc = calibration_target_for_sample(cal_loc, cfg, sample_index, angle)
        picked_loc = pick_die_with_visual_lock(camera, nozzle, part, current_die_loc, cfg, "Cal", state)
        pick_r = float(state.get("last_pick_rotation_deg", picked_loc.getRotation()))
        log("Sequence", "Step complete: pick_cal")
        placement_loc = placement_loc_for_target(target_loc, correction, pick_r)
        log("Sequence", "%s sample %d target=(%.4f, %.4f, %.4f) pick_rotation_deg=%.5f place_command=(%.4f, %.4f, %.4f) place_rotation_deg=%.5f" %
            (label, sample_index + 1, target_loc.getX(), target_loc.getY(), target_loc.getRotation(),
             pick_r, placement_loc.getX(), placement_loc.getY(), placement_loc.getRotation(),
             placement_loc.getRotation()))
        place_die_with_visual_lock(camera, nozzle, part, placement_loc, cfg, "Cal", placement_loc, state)
        log("Sequence", "Step complete: place_cal_perturbed")

        lock, actual_loc = acquire_work_die_pose(camera, nozzle, part, target_loc, cfg, "Cal")
        pose = lock["pose"]
        validate_pose_orientation(pose, target_loc, cfg,
                                  "%s sample %d post-place" % (label, sample_index + 1))
        sample = sample_from_measured_pose(target_loc, placement_loc, actual_loc, pose,
                                           correction, label, sample_index, cfg, pick_r)
        samples.append(sample)
        current_die_loc = actual_loc
        mark_current_die_loc_trusted(state, current_die_loc, True)
        state["die_at_work"] = True
        log(stage_tag, "Sample %d error X=%.5f Y=%.5f R=%.5f" %
            (sample_index + 1, sample["error_x_mm"], sample["error_y_mm"],
             sample["rotation_error_deg"]))

    return samples, current_die_loc


def storage_closed_loop_return(camera, nozzle, part, storage_loc, cfg, state,
                               correction=None, offsets_applied=False,
                               correction_allowed=True,
                               initial_pick_rotation_deg=None):
    # Deprecated diagnostic helper only. Normal final storage return is one-shot
    # and must not call this path.
    log("StorageClosedLoop", "DEPRECATED diagnostic helper; normal final storage return is one-shot.")
    storage_loc = storage_loc.convertToUnits(LengthUnit.Millimeters)
    storage_target = mm_loc(storage_loc.getX(), storage_loc.getY(), storage_loc.getZ(),
                            storage_loc.getRotation())
    max_passes = int(cfg.get("storage_closed_loop_max_passes", 2))
    tol_mm = float(cfg.get("storage_closed_loop_tolerance_mm", 0.15))
    tol_deg = float(cfg.get("storage_closed_loop_tolerance_deg", 2.0))
    allow_retry_pick = bool(cfg.get("allow_storage_retry_pick_after_place", False))
    if not allow_retry_pick:
        max_passes = 1
        log("StorageClosedLoop", "allow_storage_retry_pick_after_place=False; no storage retry pick will be attempted.")
    last_actual_loc = None
    residual_error_x = 0.0
    residual_error_y = 0.0
    residual_error_r = 0.0
    correction_for_storage = correction
    if not bool(correction_allowed):
        correction_for_storage = {"x": 0.0, "y": 0.0, "theta": 0.0}

    for pass_index in range(max_passes):
        check_cancel("StorageClosedLoop")
        pick_r = state.get("last_pick_rotation_deg", None)
        if pick_r is None:
            pick_r = initial_pick_rotation_deg if initial_pick_rotation_deg is not None else storage_target.getRotation()
        pick_r = float(pick_r)
        base_command = storage_return_command_target(storage_target, correction_for_storage,
                                                     offsets_applied, pick_r)
        command = base_command
        if pass_index > 0:
            command = mm_loc(base_command.getX() - residual_error_x,
                             base_command.getY() - residual_error_y,
                             base_command.getZ(),
                             normalize_angle(base_command.getRotation() - residual_error_r))

        log("StorageClosedLoop", "pass=%d base_corrected_command=(%.4f, %.4f, %.4f)" %
            (pass_index + 1, base_command.getX(), base_command.getY(), base_command.getRotation()))
        log("StorageClosedLoop", "pass=%d residual_adjustment=(%.5f, %.5f, %.5f)" %
            (pass_index + 1, residual_error_x, residual_error_y, residual_error_r))
        log("StorageClosedLoop", "pass=%d final_command=(%.4f, %.4f, %.4f)" %
            (pass_index + 1, command.getX(), command.getY(), command.getRotation()))

        place_die_with_visual_lock(camera, nozzle, part, storage_target, cfg, "Storage",
                                   command, state)
        log("Sequence", "Step complete: place_storage_closed_loop_pass_%d" % (pass_index + 1))

        camera_z = camera_z_for_storage(storage_target, cfg)
        camera_target = camera_measurement_loc(camera, storage_target.getX(), storage_target.getY(), camera_z)
        try:
            lock = acquire_die_pose_at_location(camera, nozzle, part, camera_target, camera_z, cfg,
                                                "StorageClosedLoop")
        except Exception, inspect_error:
            log("StorageClosedLoop", "WARNING diagnostic post-place inspection failed: %s" % inspect_error)
            state["die_at_work"] = False
            state["die_on_nozzle"] = False
            state["current_die_loc_trusted"] = False
            return {
                "enabled": True,
                "passed": False,
                "passes": pass_index + 1,
                "target_x_mm": storage_target.getX(),
                "target_y_mm": storage_target.getY(),
                "target_rotation_deg": storage_target.getRotation(),
                "error": str(inspect_error),
                "final_command_x_mm": command.getX(),
                "final_command_y_mm": command.getY(),
                "final_command_rotation_deg": command.getRotation(),
                "pick_rotation_deg": pick_r,
                "correction_used": bool(correction_allowed),
                "offsets_applied": bool(offsets_applied),
                "retry_pick_allowed": bool(allow_retry_pick)
            }
        actual_loc = die_loc_from_lock(lock, storage_target)
        pose = lock["pose"]
        actual_r = pose_rotation_for_pick(pose, cfg)
        target_r = normalize_angle(float(storage_target.getRotation()))
        error_x = actual_loc.getX() - storage_target.getX()
        error_y = actual_loc.getY() - storage_target.getY()
        error_mag = math.sqrt(error_x * error_x + error_y * error_y)
        error_r = rotation_error_deg(actual_r, target_r)
        last_actual_loc = mm_loc(actual_loc.getX(), actual_loc.getY(), storage_target.getZ(), actual_r)
        state["die_at_work"] = False
        state["die_on_nozzle"] = False
        state["current_die_loc_trusted"] = False

        log("StorageClosedLoop", "pass=%d measured=(%.4f, %.4f, %.4f) error=(%.5f, %.5f, %.5f)" %
            (pass_index + 1, actual_loc.getX(), actual_loc.getY(), actual_r,
             error_x, error_y, error_r))
        log("StorageClosedLoop", "pass=%d target=(%.4f, %.4f, %.4f) error_mag=%.5f" %
            (pass_index + 1, storage_target.getX(), storage_target.getY(),
             storage_target.getRotation(), error_mag))

        if error_mag <= tol_mm and abs(error_r) <= tol_deg:
            log("StorageClosedLoop", "pass=%d within tolerance xy=%.5f/%.5f theta=%.5f/%.5f" %
                (pass_index + 1, error_mag, tol_mm, abs(error_r), tol_deg))
            return {
                "enabled": True,
                "passed": True,
                "passes": pass_index + 1,
                "target_x_mm": storage_target.getX(),
                "target_y_mm": storage_target.getY(),
                "target_rotation_deg": storage_target.getRotation(),
                "measured_x_mm": actual_loc.getX(),
                "measured_y_mm": actual_loc.getY(),
                "measured_rotation_deg": actual_r,
                "error_x_mm": error_x,
                "error_y_mm": error_y,
                "error_mag_mm": error_mag,
                "error_rotation_deg": error_r,
                "final_command_x_mm": command.getX(),
                "final_command_y_mm": command.getY(),
                "final_command_rotation_deg": command.getRotation(),
                "base_command_x_mm": base_command.getX(),
                "base_command_y_mm": base_command.getY(),
                "base_command_rotation_deg": base_command.getRotation(),
                "pick_rotation_deg": pick_r,
                "correction_used": bool(correction_allowed),
                "offsets_applied": bool(offsets_applied),
                "retry_pick_allowed": bool(allow_retry_pick)
            }

        if pass_index + 1 < max_passes:
            residual_error_x = error_x
            residual_error_y = error_y
            residual_error_r = error_r
            if bool(allow_retry_pick):
                pick_die_with_visual_lock(camera, nozzle, part, last_actual_loc, cfg,
                                          "StorageClosedLoop", state)
                log("Sequence", "Step complete: pick_storage_closed_loop_retry")

    if not allow_retry_pick:
        log("StorageClosedLoop", "Diagnostic inspection did not meet tolerance; no retry pick attempted.")
        return {
            "enabled": True,
            "passed": False,
            "passes": 1,
            "target_x_mm": storage_target.getX(),
            "target_y_mm": storage_target.getY(),
            "target_rotation_deg": storage_target.getRotation(),
            "measured_x_mm": last_actual_loc.getX() if last_actual_loc is not None else None,
            "measured_y_mm": last_actual_loc.getY() if last_actual_loc is not None else None,
            "measured_rotation_deg": last_actual_loc.getRotation() if last_actual_loc is not None else None,
            "error_x_mm": error_x,
            "error_y_mm": error_y,
            "error_mag_mm": error_mag,
            "error_rotation_deg": error_r,
            "retry_pick_allowed": False
        }

    raise Exception("Storage closed-loop return failed: final error %.5f mm / %.5f deg exceeds tolerance %.5f mm / %.5f deg." %
                    (error_mag, abs(error_r), tol_mm, tol_deg))


def final_storage_xy_offset_from_result(computed, cfg):
    # TODO: Split calibration fitting, storage return, confirm moves, and
    # actuator control into separate modules. Final storage return must remain
    # independent from calibration correction helpers except diagnostic modes.
    source = str(cfg.get("final_storage_offset_source", "manual")).lower()
    if source == "manual":
        manual = cfg.get("manual_final_storage_offset_mm", {})
        return float(manual.get("x", 0.0)), float(manual.get("y", 0.0)), "manual"
    if computed is None:
        return 0.0, 0.0, "no computed result"

    base_x = float(computed.get("bias_x_mm", computed.get("offset_x_mm", 0.0)))
    base_y = float(computed.get("bias_y_mm", computed.get("offset_y_mm", 0.0)))

    if source == "computed_machine_delta":
        return -base_x, -base_y, "computed_machine_delta"
    if source == "computed_error":
        return base_x, base_y, "computed_error"
    raise Exception("Unsupported final_storage_offset_source: %s" % source)


def manual_storage_offset(cfg):
    manual = cfg.get("manual_final_storage_offset_mm", {})
    return float(manual.get("x", 0.0)), float(manual.get("y", 0.0))


def set_manual_storage_offset(x_mm, y_mm, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    previous_progress = _progress_callback
    previous_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token

    try:
        check_cancel("StorageTune")
        cfg = load_config()
        old_x, old_y = manual_storage_offset(cfg)
        new_x = float(x_mm)
        new_y = float(y_mm)
        cfg["manual_final_storage_offset_mm"] = {"x": new_x, "y": new_y}
        save_config(cfg)
        log("StorageTune", "Manual storage offset changed old=(%.5f, %.5f) new=(%.5f, %.5f)" %
            (old_x, old_y, new_x, new_y))
        return {
            "type": "set_manual_storage_offset",
            "old_manual_storage_offset": {"x": old_x, "y": old_y},
            "new_manual_storage_offset": {"x": new_x, "y": new_y},
            "config_path": config_path(),
            "timestamp": time.time()
        }
    finally:
        _progress_callback = previous_progress
        _cancel_token = previous_cancel


def log_storage_tune(observed_loc, storage_loc, cfg):
    if observed_loc is None or storage_loc is None:
        return None
    observed_loc = observed_loc.convertToUnits(LengthUnit.Millimeters)
    storage_loc = storage_loc.convertToUnits(LengthUnit.Millimeters)
    error_x = observed_loc.getX() - storage_loc.getX()
    error_y = observed_loc.getY() - storage_loc.getY()
    current_x, current_y = manual_storage_offset(cfg)
    suggested_x = current_x - error_x
    suggested_y = current_y - error_y
    log("StorageTune", "observed_storage_center=(%.4f, %.4f)" %
        (observed_loc.getX(), observed_loc.getY()))
    log("StorageTune", "configured_storage_center=(%.4f, %.4f)" %
        (storage_loc.getX(), storage_loc.getY()))
    log("StorageTune", "observed_error=(%.5f, %.5f)" % (error_x, error_y))
    log("StorageTune", "suggested_manual_final_storage_offset_adjustment=(%.5f, %.5f)" %
        (-error_x, -error_y))
    log("StorageTune", "current_manual_offset=(%.5f, %.5f)" % (current_x, current_y))
    log("StorageTune", "suggested_new_manual_offset=(%.5f, %.5f)" %
        (suggested_x, suggested_y))
    log("StorageTune", "suggested_manual_final_storage_offset_mm={\"x\": %.5f, \"y\": %.5f}" %
        (suggested_x, suggested_y))
    return {
        "observed_storage_center": {
            "x": float(observed_loc.getX()),
            "y": float(observed_loc.getY()),
            "rotation": float(observed_loc.getRotation())
        },
        "configured_storage_center": {
            "x": float(storage_loc.getX()),
            "y": float(storage_loc.getY()),
            "rotation": float(storage_loc.getRotation())
        },
        "observed_error": {
            "x": float(error_x),
            "y": float(error_y)
        },
        "current_manual_offset": {
            "x": float(current_x),
            "y": float(current_y)
        },
        "suggested_manual_final_storage_offset_mm": {
            "x": float(suggested_x),
            "y": float(suggested_y)
        },
        "suggested_new_manual_offset": {
            "x": float(suggested_x),
            "y": float(suggested_y)
        }
    }


def storage_return_summary(status, mode, storage_target, command, correction_used,
                           reason, final_pick_r, correction):
    cmd_x = None
    cmd_y = None
    cmd_r = None
    cmd_mag = None
    cmd_dx = None
    cmd_dy = None
    if command is not None:
        cmd_x = float(command.getX())
        cmd_y = float(command.getY())
        cmd_r = float(command.getRotation())
        cmd_dx = cmd_x - float(storage_target.getX())
        cmd_dy = cmd_y - float(storage_target.getY())
        cmd_mag = math.sqrt(cmd_dx * cmd_dx + cmd_dy * cmd_dy)
    return {
        "status": status,
        "mode": mode,
        "final_storage_return_mode": mode,
        "target_basis": "configured die_storage_location_xyz",
        "target_x_mm": float(storage_target.getX()),
        "target_y_mm": float(storage_target.getY()),
        "target_z_mm": float(storage_target.getZ()),
        "target_rotation_deg": float(storage_target.getRotation()),
        "configured_storage_target": {
            "x": float(storage_target.getX()),
            "y": float(storage_target.getY()),
            "z": float(storage_target.getZ()),
            "rotation": float(storage_target.getRotation())
        },
        "command_x_mm": cmd_x,
        "command_y_mm": cmd_y,
        "command_z_mm": float(command.getZ()) if command is not None else None,
        "command_rotation_deg": cmd_r,
        "final_storage_command": {
            "x": cmd_x,
            "y": cmd_y,
            "z": float(command.getZ()) if command is not None else None,
            "rotation": cmd_r
        },
        "command_delta_x_mm": cmd_dx,
        "command_delta_y_mm": cmd_dy,
        "command_delta_mag_mm": cmd_mag,
        "final_pick_rotation_deg": float(final_pick_r),
        "correction_model": correction_model_name(correction),
        "correction_used": bool(correction_used),
        "correction_allowed": bool(correction_used),
        "correction_not_allowed_reason": reason,
        "message": reason
    }


def return_die_to_storage(camera, nozzle, part, storage_loc, cfg, state, detected_storage_pick_loc=None,
                          storage_return_correction=None, offsets_applied=False, verification_passed=False):
    storage_loc = storage_loc.convertToUnits(LengthUnit.Millimeters)
    mode = "simple_configured_pocket_return"
    log("StorageReturn", "final return mode=%s" % mode)
    log("StorageReturn", "storage target basis=configured die_storage_location_xyz")
    log("StorageReturn", "post-place storage closed-loop disabled for final return")
    current_die_loc = state.get("current_die_loc", None)
    if current_die_loc is None:
        raise Exception("Cannot return die to storage: no current calibration pose is available.")
    if not bool(state.get("current_die_loc_trusted", False)):
        trusted = state.get("last_trusted_die_loc", None)
        if trusted is not None:
            log("Cleanup", "Refusing blind storage return; last trusted work pose X=%.4f Y=%.4f R=%.4f" %
                (trusted.getX(), trusted.getY(), trusted.getRotation()))
        raise Exception("Cannot return die to storage: current calibration pose is not trusted.")
    state["final_storage_return_started"] = True
    final_pick_loc = pick_die_for_final_storage_return(
        camera, nozzle, part, current_die_loc, cfg, state, storage_return_correction
    )
    storage_target = configured_storage_return_target(storage_loc)
    storage_command = storage_target
    final_pick_r = float(state.get("last_pick_rotation_deg", final_pick_loc.getRotation()))
    correction_used = False
    correction_reason = "no calibration XY correction is applied to storage return"

    log("StorageReturn", "configured_storage_target=(%.4f, %.4f, %.4f, %.4f)" %
        (storage_target.getX(), storage_target.getY(), storage_target.getZ(),
         storage_target.getRotation()))
    log("StorageReturn", "storage_command=(%.4f, %.4f, %.4f, %.4f)" %
        (storage_command.getX(), storage_command.getY(), storage_command.getZ(),
         storage_command.getRotation()))
    log("StorageReturn", "storage_xy_delta=(0.00000, 0.00000, 0.00000)")
    log("StorageReturn", "no calibration XY correction is applied to storage return")
    log("StorageReturn", "final_pick_rotation_deg=%.4f" % final_pick_r)
    log("StorageReturn", "rotating/placing die at configured storage rotation R=%.4f" %
        storage_command.getRotation())

    cmd_dx, cmd_dy, cmd_mag = command_delta_from_target(storage_command, storage_target)
    log("StorageReturn", "command_delta_from_configured_target=(%.5f, %.5f, %.5f)" %
        (cmd_dx, cmd_dy, cmd_mag))
    if cmd_mag > 0.0001:
        raise Exception("Internal bug: final storage command changed configured storage XY.")

    place_die_with_visual_lock(camera, nozzle, part, storage_target, cfg, "Storage",
                               storage_command, state)
    try:
        final_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        log("StorageReturn", "final_commanded_nozzle_R=%.4f final_nozzle X=%.4f Y=%.4f Z=%.4f R=%.4f" %
            (storage_command.getRotation(), final_loc.getX(), final_loc.getY(),
             final_loc.getZ(), final_loc.getRotation()))
    except:
        pass
    state["die_on_nozzle"] = False
    state["die_at_work"] = False
    state["current_die_loc_trusted"] = False
    log("StorageReturn", "final storage placement complete; no refind/retry/pick will be attempted")
    log("Sequence", "Step complete: place_storage_final")
    state["storage_return"] = storage_return_summary(
        "placed", mode, storage_target, storage_command,
        correction_used, correction_reason, final_pick_r,
        None)
    return state["storage_return"]


def cleanup_after_calibration_failure(camera, nozzle, part, storage_loc, cal_loc, cfg, state, reason):
    log("Error", "Calibration cleanup after failure: %s" % reason)
    if state.get("die_on_nozzle", False):
        safe_cleanup(camera, nozzle, part, storage_loc, cfg, reason,
                     state.get("detected_storage_pick_loc", None))
        state["die_on_nozzle"] = True
    elif state.get("die_at_work", False):
        if not bool(state.get("current_die_loc_trusted", False)):
            trusted = state.get("last_trusted_die_loc", None)
            if trusted is not None:
                log("Cleanup", "Die may remain near last trusted work pose X=%.4f Y=%.4f R=%.4f; skipping blind recovery." %
                    (trusted.getX(), trusted.getY(), trusted.getRotation()))
            else:
                log("Cleanup", "Current work pose is not trusted; skipping blind recovery.")
            return
        try:
            current_die_loc = state.get("current_die_loc", cal_loc)
            pick_die_with_visual_lock(camera, nozzle, part, current_die_loc, cfg, "Cal", state)
            safe_cleanup(camera, nozzle, part, storage_loc, cfg, reason,
                         state.get("detected_storage_pick_loc", None))
        except Exception, cleanup_error:
            log("Cleanup", "Could not recover die from work location: %s" % cleanup_error)


def apply_offsets_to_machine(nozzle, camera, correction, cfg):
    enforce_write_guard(cfg)
    if correction.get("model", "") in ["bias_plus_rotating_pivot_vector", "bias_plus_pick_place_vectors"]:
        raise Exception("Refusing to apply pivot/vector model as a static machine offset; use placement command compensation and verification instead.")
    target = cfg.get("apply_target", "nozzle_head_offsets")
    mag = math.sqrt(correction["offset_x_mm"] ** 2 + correction["offset_y_mm"] ** 2)
    if mag > float(cfg.get("max_apply_mm", 0.5)):
        raise Exception("Refusing to apply %.4f mm correction above max_apply_mm %.4f" %
                        (mag, float(cfg.get("max_apply_mm", 0.5))))

    delta = mm_loc(-correction["offset_x_mm"], -correction["offset_y_mm"], 0.0, 0.0)

    if target == "nozzle_head_offsets":
        old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
        new = old.add(delta)
        nozzle.setHeadOffsets(new)
        written = {
            "target": target,
            "old_x_mm": old.getX(),
            "old_y_mm": old.getY(),
            "new_x_mm": new.getX(),
            "new_y_mm": new.getY(),
            "delta_x_mm": delta.getX(),
            "delta_y_mm": delta.getY()
        }
        log("Apply", "Wrote nozzle head offsets: old=(%.5f, %.5f) new=(%.5f, %.5f)" %
            (old.getX(), old.getY(), new.getX(), new.getY()))
        return written

    if target == "camera_head_offsets":
        old = camera.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
        new = old.add(delta)
        camera.setHeadOffsets(new)
        written = {
            "target": target,
            "old_x_mm": old.getX(),
            "old_y_mm": old.getY(),
            "new_x_mm": new.getX(),
            "new_y_mm": new.getY(),
            "delta_x_mm": delta.getX(),
            "delta_y_mm": delta.getY()
        }
        log("Apply", "Wrote camera head offsets: old=(%.5f, %.5f) new=(%.5f, %.5f)" %
            (old.getX(), old.getY(), new.getX(), new.getY()))
        return written

    raise Exception("Unsupported apply_target: %s" % target)


def abort_if_computed_offset_too_large(computed, cfg):
    mag = float(computed.get("offset_mag_mm", 0.0))
    max_verify = float(cfg.get("max_verify_correction_mm", 2.0))
    if mag > max_verify:
        raise Exception("Computed offset %.4f mm exceeds max_verify_correction_mm %.4f; aborting before verification correction." %
                        (mag, max_verify))


def computed_offset_too_large_reason(computed, cfg):
    mag = float(computed.get("offset_mag_mm", 0.0))
    max_verify = float(cfg.get("max_verify_correction_mm", 2.0))
    if mag > max_verify:
        return "Computed offset %.4f mm exceeds max_verify_correction_mm %.4f; skipping verification correction." % (mag, max_verify)
    return None


def skipped_verification_summary(reason):
    return {
        "label": "verification",
        "model": "skipped",
        "sample_count": 0,
        "offset_x_mm": 0.0,
        "offset_y_mm": 0.0,
        "offset_mag_mm": 0.0,
        "repeatability_rms_mm": 0.0,
        "residual_rms_mm": 0.0,
        "rotation_error_deg": 0.0,
        "skipped": True,
        "skip_reason": reason
    }


def verification_passed_summary(verification, cfg):
    max_offset = float(cfg.get("verification_pass_max_offset_mm", 0.20))
    max_theta = float(cfg.get("verification_pass_max_theta_deg", 1.0))
    residual_rms = float(verification.get("residual_rms_mm",
                         verification.get("repeatability_rms_mm", 999999.0)))
    passed = (float(verification.get("offset_mag_mm", 999999.0)) <= max_offset and
              abs(float(verification.get("rotation_error_deg", 999999.0))) <= max_theta and
              residual_rms <= max_offset)
    verification["passed"] = bool(passed)
    verification["pass_max_offset_mm"] = max_offset
    verification["pass_max_theta_deg"] = max_theta
    verification["pass_max_residual_rms_mm"] = max_offset
    log("Verify", "pass/fail=%s model=%s offset_mag=%.5f threshold=%.5f rotation_error=%.5f threshold=%.5f residual_rms=%.5f threshold=%.5f" %
        ("pass" if passed else "fail",
         verification.get("model", "unknown"),
         float(verification.get("offset_mag_mm", 0.0)), max_offset,
         float(verification.get("rotation_error_deg", 0.0)), max_theta,
         residual_rms, max_offset))
    return passed


def log_result_summary(result):
    if result is None:
        return
    computed = result.get("computed", {}) or {}
    verification = result.get("verification", {}) or {}
    storage_return = result.get("storage_return", {}) or {}
    log("Result", "computed_model=%s" % computed.get("model", "unknown"))
    log("Result", "computed_offset=(%.5f, %.5f, %.5f)" %
        (float(computed.get("offset_x_mm", 0.0)),
         float(computed.get("offset_y_mm", 0.0)),
         float(computed.get("offset_mag_mm", 0.0))))
    log("Result", "verification_passed=%s" % str(bool(result.get("verification_passed", False))))
    log("Result", "verification_offset=(%.5f, %.5f, %.5f)" %
        (float(verification.get("offset_x_mm", 0.0)),
         float(verification.get("offset_y_mm", 0.0)),
         float(verification.get("offset_mag_mm", 0.0))))
    log("Result", "verification_residual=%.5f" %
        float(verification.get("residual_rms_mm", verification.get("repeatability_rms_mm", 0.0))))
    log("Result", "final_storage_return status=%s correction_used=%s" %
        (storage_return.get("status", "unknown"),
         str(bool(storage_return.get("correction_used", False)))))
    log("Result", "final_storage_return_mode=%s" %
        storage_return.get("final_storage_return_mode", storage_return.get("mode", "unknown")))
    log("Result", "configured_storage_target=(%.4f, %.4f, %.4f)" %
        (float(storage_return.get("target_x_mm", 0.0)),
         float(storage_return.get("target_y_mm", 0.0)),
         float(storage_return.get("target_rotation_deg", 0.0))))
    log("Result", "final_storage_command=(%.4f, %.4f, %.4f)" %
        (float(storage_return.get("command_x_mm", 0.0) or 0.0),
         float(storage_return.get("command_y_mm", 0.0) or 0.0),
         float(storage_return.get("command_rotation_deg", 0.0) or 0.0)))
    log("Result", "final_storage_command_delta=(%.5f, %.5f, %.5f)" %
        (float(storage_return.get("command_delta_x_mm", 0.0) or 0.0),
         float(storage_return.get("command_delta_y_mm", 0.0) or 0.0),
         float(storage_return.get("command_delta_mag_mm", 0.0) or 0.0)))
    log("Result", "correction_model=%s final_pick_rotation_deg=%.4f" %
        (storage_return.get("correction_model", None),
         float(storage_return.get("final_pick_rotation_deg", 0.0) or 0.0)))
    confirm = result.get("confirm_shift", None)
    storage_tune = None
    if isinstance(confirm, dict):
        storage_tune = confirm.get("storage_tune", None)
    if storage_tune is None:
        storage_tune = result.get("storage_tune", None)
    if isinstance(storage_tune, dict):
        observed = storage_tune.get("observed_storage_center", None)
        suggested = storage_tune.get("suggested_new_manual_offset", None)
        if observed is not None:
            log("Result", "observed_storage_center=(%.4f, %.4f)" %
                (float(observed.get("x", 0.0)), float(observed.get("y", 0.0))))
        if suggested is not None:
            log("Result", "suggested_new_manual_offset=(%.5f, %.5f)" %
                (float(suggested.get("x", 0.0)), float(suggested.get("y", 0.0))))
    log("Result", "command_delta_from_pocket=(%.5f, %.5f, %.5f)" %
        (float(storage_return.get("command_delta_x_mm", 0.0) or 0.0),
         float(storage_return.get("command_delta_y_mm", 0.0) or 0.0),
         float(storage_return.get("command_delta_mag_mm", 0.0) or 0.0)))
    log("Result", "result saved to %s" % result_path())


def safe_cleanup(camera, nozzle, part, storage_loc, cfg, reason, detected_storage_pick_loc=None):
    try:
        log("Cleanup", "Operator handoff after failure: %s" % reason)
        if detected_storage_pick_loc is not None:
            log("Cleanup", "detected_initial_pick_only diagnostic X=%.4f Y=%.4f Z=%.4f R=%.4f" %
                (detected_storage_pick_loc.getX(), detected_storage_pick_loc.getY(),
                 detected_storage_pick_loc.getZ(), detected_storage_pick_loc.getRotation()))
        log("Cleanup", "configured_storage_target X=%.4f Y=%.4f Z=%.4f R=%.4f" %
            (storage_loc.getX(), storage_loc.getY(), storage_loc.getZ(), storage_loc.getRotation()))
        loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        lift = mm_loc(loc.getX(), loc.getY(), float(cfg["safe_travel_z"]), loc.getRotation())
        move_to_with_speed(nozzle, nearest_nozzle_rotation_location(nozzle, lift, "Cleanup"), cfg)
        log("Cleanup", "Raw configured storage placement was not attempted; nozzle lifted to safe Z and vacuum state was left unchanged.")
    except Exception, e:
        log("Cleanup", "Operator handoff lift failed: %s" % e)
        try:
            loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            lift = mm_loc(loc.getX(), loc.getY(), float(cfg["safe_travel_z"]), loc.getRotation())
            move_to_with_speed(nozzle, nearest_nozzle_rotation_location(nozzle, lift, "Cleanup"), cfg)
        except:
            pass


def _prepare_runtime(cfg):
    global _dry_run_mode_warning_logged
    if "dry_run_motion_mode" in cfg and not _dry_run_mode_warning_logged:
        log("Safety", "dry_run_motion_mode is deprecated; dry run now performs full motion and skips only offset writes.")
        _dry_run_mode_warning_logged = True

    machine_obj = get_machine()
    head = machine_obj.getDefaultHead()
    if head is None:
        raise Exception("Machine default head is null.")

    nozzle = find_nozzle(head, cfg["nozzle_name"])
    camera = find_top_camera(machine_obj, head, cfg["camera_name"])
    part = get_part(cfg["part_id_or_name"])
    derive_storage_z_fields(cfg, part)
    validate_config(cfg)
    try:
        if not camera.isUnitsPerPixelAtZCalibrated():
            raise Exception("Camera '%s' units-per-pixel-at-Z calibration is not initialized." %
                            object_name(camera))
    except AttributeError:
        log("Safety", "Camera does not expose isUnitsPerPixelAtZCalibrated(); continuing with legacy calibration API.")
    log("Safety", "Storage base Z from Die Storage Z: %.4f; selected part height: %.4f" %
        (storage_base_z_mm(cfg), part_height_z_mm(cfg)))
    storage_loc = loc_from_xyz(cfg["die_storage_location_xyz"])
    cal_loc = loc_from_xyz(cfg["cal_work_location_xyz"])
    return machine_obj, head, nozzle, camera, part, storage_loc, cal_loc


def apply_result(result=None, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    previous_progress = _progress_callback
    previous_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token

    try:
        check_cancel("Apply")
        cfg = load_config()
        cfg["_apply_offsets_requested"] = True
        log("Init", "Loading and validating %s" % config_path())
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = _prepare_runtime(cfg)
        enforce_write_guard(cfg)
        if result is None:
            result = load_last_result()
        if result.get("error"):
            raise Exception("Refusing to apply failed result: %s" % result.get("error"))
        computed = result.get("computed")
        if computed is None:
            raise Exception("Result does not contain computed offsets.")
        applied = apply_offsets_to_machine(nozzle, camera, computed, cfg)
        result["applied"] = applied
        result["apply_offsets_requested"] = True
        result["applied_timestamp"] = time.time()
        save_result(result)
        return result
    finally:
        _progress_callback = previous_progress
        _cancel_token = previous_cancel


def test_vision_lock(location_key, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    previous_progress = _progress_callback
    previous_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token

    cfg = load_config()
    camera = None
    try:
        check_cancel("VisionLock")
        log("VisionLock", "Testing %s vision lock" % location_key)
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = _prepare_runtime(cfg)
        key = str(location_key).lower()
        if key == "storage":
            base_loc = storage_loc
            camera_z = camera_z_for_storage(storage_loc, cfg)
            tag = "Storage"
        elif key == "cal":
            base_loc = cal_loc
            camera_z = camera_z_for_cal(cal_loc, cfg)
            tag = "Cal"
        else:
            raise Exception("location_key must be 'storage' or 'cal'.")

        if key in ["storage", "cal"]:
            camera_base = camera_measurement_loc(camera, base_loc.getX(), base_loc.getY(), camera_z)
        else:
            camera_base = tool_aware_camera_target_for_location(camera, nozzle, base_loc, camera_z)
        lock = acquire_die_pose_at_location(camera, nozzle, part, camera_base, camera_z, cfg, tag)
        pose = lock["pose"]
        result = {
            "type": "vision_lock",
            "location_key": key,
            "circle_count": int(pose.get("circle_count", 0)),
            "center_x_mm": pose.get("center_x_mm", 0.0),
            "center_y_mm": pose.get("center_y_mm", 0.0),
            "rotation_deg": pose.get("rotation_deg", 0.0),
            "rotation_raw_deg": pose.get("rotation_raw_deg", None),
            "rotation_snapped_deg": pose.get("rotation_snapped_deg", None),
            "orientation_found": pose.get("orientation_found", False),
            "orientation_raw_rotation_deg": pose.get("orientation_raw_rotation_deg", None),
            "orientation_rotation_deg": pose.get("orientation_rotation_deg", None),
            "orientation_snap_error_deg": pose.get("orientation_snap_error_deg", None),
            "orientation_stage_name": pose.get("orientation_stage_name", None),
            "orientation_dx_mm": pose.get("orientation_dx_mm", None),
            "orientation_dy_mm": pose.get("orientation_dy_mm", None),
            "rotation_source": pose.get("rotation_source", None),
            "search_dx_mm": pose.get("search_dx_mm", 0.0),
            "search_dy_mm": pose.get("search_dy_mm", 0.0),
            "camera_z_mm": camera_z,
            "centered_machine_x_mm": pose.get("centered_machine_x_mm", 0.0),
            "centered_machine_y_mm": pose.get("centered_machine_y_mm", 0.0),
            "overlay": pose.get("overlay", None),
            "timestamp": time.time()
        }
        log("VisionLock", "%s test found %d circles center=(%.5f, %.5f)" %
            (tag, result["circle_count"], result["center_x_mm"], result["center_y_mm"]))
        return result
    finally:
        if camera is not None:
            try:
                loc = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
                camera.moveTo(mm_loc(loc.getX(), loc.getY(), float(cfg["safe_travel_z"]), loc.getRotation()))
                wait_still(camera)
                log("Cleanup", "Camera lifted to safe Z after vision-lock test.")
            except Exception, cleanup_error:
                log("Cleanup", "Vision-lock test safe lift failed: %s" % cleanup_error)
        _progress_callback = previous_progress
        _cancel_token = previous_cancel


def configured_location_for_key(location_key, storage_loc, cal_loc):
    key = str(location_key).lower()
    if key in ["storage", "die_storage", "die_storage_location_xyz"]:
        return storage_loc, "Storage"
    if key in ["cal", "calibration", "cal_work", "cal_work_location_xyz"]:
        return cal_loc, "Cal"
    raise Exception("location_key must be 'storage' or 'cal'.")


def move_tool_over_location(location_key, tool_kind, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    previous_progress = _progress_callback
    previous_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token

    try:
        check_cancel("Move")
        cfg = load_config()
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = _prepare_runtime(cfg)
        base_loc, tag = configured_location_for_key(location_key, storage_loc, cal_loc)
        tool_name = str(tool_kind).lower()
        if tool_name == "camera":
            target = camera_measurement_loc(camera, base_loc.getX(), base_loc.getY(), float(cfg["safe_travel_z"]))
            log("Move", "Moving camera over %s at safe Z." % tag)
            move_tool_safe(camera, target, "Move")
        elif tool_name == "nozzle":
            target = mm_loc(base_loc.getX(), base_loc.getY(), float(cfg["safe_travel_z"]), base_loc.getRotation())
            log("Move", "Moving nozzle over %s at safe Z." % tag)
            move_target = nearest_nozzle_rotation_location(nozzle, target, "Move")
            move_tool_safe(nozzle, move_target, "Move")
        else:
            raise Exception("tool_kind must be 'camera' or 'nozzle'.")
        return {
            "type": "move_tool",
            "location_key": str(location_key),
            "tool_kind": tool_name,
            "x_mm": target.getX(),
            "y_mm": target.getY(),
            "z_mm": target.getZ(),
            "rotation_deg": target.getRotation()
        }
    finally:
        _progress_callback = previous_progress
        _cancel_token = previous_cancel


def confirm_shift(result=None, show_shift=True, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    if not is_bool_argument(show_shift) and progress_callback is None:
        progress_callback = show_shift
        show_shift = True
    show_shift = bool_value(show_shift)
    previous_progress = _progress_callback
    previous_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token

    try:
        check_cancel("Confirm")
        cfg = load_config()
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = _prepare_runtime(cfg)
        prepare_nozzle_for_part(nozzle, part)
        result_loaded = False
        already_applied = False
        if result is None:
            try:
                result = load_last_result()
                result_loaded = True
            except Exception, e:
                result = None
                log("Confirm", "No last result loaded for confirm diagnostics: %s" % e)
        if result is not None and result.get("error"):
            raise Exception("Refusing confirm shift for failed result: %s" % result.get("error"))
        if result is not None:
            already_applied = result.get("applied") is not None

        log("Confirm", "Storage confirm is a saved-pocket command preview; loose storage die position is not used as the target basis.")
        log("Confirm", "confirm preview does not modify final storage return; final storage return always uses configured storage XY.")
        mode = "show_shift" if bool(show_shift) else "show_no_shift"
        storage_target = configured_storage_return_target(storage_loc)
        computed = {}
        if result is not None:
            computed = result.get("computed", {}) or {}
        correction = None
        preview_pick_r = float(storage_target.getRotation())
        storage_return = {}
        if result is not None:
            storage_return = result.get("storage_return", {}) or {}
        try:
            preview_pick_r = float(storage_return.get("final_pick_rotation_deg", preview_pick_r))
        except:
            pass
        if computed:
            correction = dict(computed)
            correction["x"] = float(computed.get("offset_x_mm", computed.get("x", 0.0)))
            correction["y"] = float(computed.get("offset_y_mm", computed.get("y", 0.0)))
            correction["theta"] = float(computed.get("theta", computed.get("theta_error_deg",
                                      computed.get("rotation_error_deg", 0.0))))
            if "model" not in correction:
                correction["model"] = "translation_theta"
            correction["use_pivot_correction"] = True
        correction_used = bool(show_shift) and correction is not None
        if correction_used:
            preview_command = placement_loc_for_target(storage_target, correction, preview_pick_r)
        else:
            preview_command = storage_target
        lock, detected_loc, overlay = acquire_fresh_confirm_storage_pose(
            camera, nozzle, part, storage_target, cfg, preview_command
        )
        pose = lock["pose"]
        target_delta_x, target_delta_y, target_delta_mag = command_delta_from_target(preview_command, storage_target)
        if bool(show_shift):
            target_x = detected_loc.getX() + target_delta_x
            target_y = detected_loc.getY() + target_delta_y
        else:
            target_x = detected_loc.getX()
            target_y = detected_loc.getY()
        target = mm_loc(target_x,
                        target_y,
                        float(cfg["safe_travel_z"]),
                        preview_command.getRotation())
        overlay = overlay_with_command_target(camera, overlay, target, detected_loc)
        if overlay is not None:
            save_last_overlay(overlay)
        log("Confirm", "mode=%s" % mode)
        log("Confirm", "target_basis=fresh_detected_center")
        log("Confirm", "configured_storage_target=(%.4f, %.4f, %.4f)" %
            (storage_target.getX(), storage_target.getY(), storage_target.getRotation()))
        log("Confirm", "toggle_anchor_detected_center=(%.4f, %.4f, %.4f)" %
            (detected_loc.getX(), detected_loc.getY(), detected_loc.getRotation()))
        log("Confirm", "correction_used=%s" % str(bool(correction_used)))
        if bool(correction_used):
            log("Confirm", "correction_model=%s" % correction_model_name(correction))
            log("Confirm", "preview_pick_rotation_deg=%.4f" % preview_pick_r)
        log("Confirm", "preview_command=(%.4f, %.4f, %.4f)" %
            (preview_command.getX(), preview_command.getY(), preview_command.getRotation()))
        log("Confirm", "toggle_no_shift_command=(%.4f, %.4f) delta_from_anchor=(0.00000, 0.00000, 0.00000)" %
            (detected_loc.getX(), detected_loc.getY()))
        log("Confirm", "toggle_shift_command=(%.4f, %.4f) delta_from_anchor=(%.5f, %.5f, %.5f)" %
            (detected_loc.getX() + target_delta_x, detected_loc.getY() + target_delta_y,
             target_delta_x, target_delta_y, target_delta_mag))
        if bool(show_shift):
            log("Confirm", "command_delta_from_no_shift=(%.5f, %.5f, %.5f)" %
                (target_delta_x, target_delta_y, target_delta_mag))
        log("Confirm", "already_applied=%s result_loaded=%s verification_passed=%s" %
            (str(already_applied), str(result_loaded),
             str(bool(result.get("verification_passed", False))) if result is not None else "False"))
        move_target = nearest_nozzle_rotation_location(nozzle, target, "Confirm")
        log("Confirm", "toggle_safe_lift=before_xy; moving to safe Z above fresh-detection-based preview target.")
        move_nozzle_xy_r_then_z(nozzle, move_target, cfg, "Confirm")
        down_target = mm_loc(target.getX(), target.getY(), storage_pick_surface_z_mm(cfg, storage_loc), target.getRotation())
        down_target = nearest_nozzle_rotation_location(nozzle, down_target, "Confirm")
        log("Confirm", "toggle_descend_to_surface=True target_z=%.4f; using async-safe staged descent." %
            down_target.getZ())
        move_nozzle_xy_r_then_z(nozzle, down_target, cfg, "Confirm")
        confirm_result = {
            "type": "confirm_shift",
            "mode": mode,
            "target_basis": "fresh_detected_center",
            "target_x_mm": target.getX(),
            "target_y_mm": target.getY(),
            "target_z_mm": down_target.getZ(),
            "target_rotation_deg": target.getRotation(),
            "circle_count": int(pose.get("circle_count", 0)),
            "detected_center_x_mm": float(detected_loc.getX()),
            "detected_center_y_mm": float(detected_loc.getY()),
            "detected_rotation_deg": float(detected_loc.getRotation()),
            "result_already_applied": already_applied,
            "result_loaded": result_loaded,
            "correction_used": correction_used,
            "correction_model": correction_model_name(correction) if correction_used else None,
            "preview_pick_rotation_deg": preview_pick_r if correction_used else None,
            "configured_storage_target": {
                "x": float(storage_target.getX()),
                "y": float(storage_target.getY()),
                "z": float(storage_target.getZ()),
                "rotation": float(storage_target.getRotation())
            },
            "preview_command": {
                "x": float(preview_command.getX()),
                "y": float(preview_command.getY()),
                "z": float(preview_command.getZ()),
                "rotation": float(preview_command.getRotation())
            },
            "target_delta_x_mm": target_delta_x,
            "target_delta_y_mm": target_delta_y,
            "target_delta_mag_mm": target_delta_mag,
            "correction_x_mm": target_delta_x,
            "correction_y_mm": target_delta_y,
            "toggle_anchor_detected_center": {
                "x": float(detected_loc.getX()),
                "y": float(detected_loc.getY()),
                "rotation": float(detected_loc.getRotation())
            },
            "toggle_no_shift_command": {
                "x": float(detected_loc.getX()),
                "y": float(detected_loc.getY())
            },
            "toggle_shift_command": {
                "x": float(detected_loc.getX() + target_delta_x),
                "y": float(detected_loc.getY() + target_delta_y)
            },
            "confirm_shift_descend_to_surface": True,
            "fresh_detection_required": True,
            "fresh_detection_timestamp": time.time(),
            "overlay": overlay,
            "timestamp": time.time()
        }
        save_last_confirm_result(confirm_result)
        return confirm_result
    finally:
        _progress_callback = previous_progress
        _cancel_token = previous_cancel


def suggested_offset_from_result(result):
    if result is None:
        return None
    direct = result.get("suggested_new_manual_offset", None)
    if direct is None:
        direct = result.get("suggested_manual_final_storage_offset_mm", None)
    if direct is None:
        storage_tune = result.get("storage_tune", None)
        if isinstance(storage_tune, dict):
            direct = storage_tune.get("suggested_new_manual_offset", None)
            if direct is None:
                direct = storage_tune.get("suggested_manual_final_storage_offset_mm", None)
    if direct is None:
        confirm = result.get("confirm_shift", None)
        if isinstance(confirm, dict):
            return suggested_offset_from_result(confirm)
    if direct is None:
        return None
    return {
        "x": float(direct.get("x", 0.0)),
        "y": float(direct.get("y", 0.0))
    }


def apply_suggested_storage_offset_from_last_result(result=None, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    previous_progress = _progress_callback
    previous_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token

    try:
        check_cancel("StorageTune")
        cfg = load_config()
        source = "provided result"
        if result is None:
            try:
                result = load_last_confirm_result()
                source = "last confirm result"
            except Exception, confirm_error:
                log("StorageTune", "No last confirm result loaded: %s" % confirm_error)
                result = load_last_result()
                source = "last calibration result"
        suggested = suggested_offset_from_result(result)
        if suggested is None:
            raise Exception("No suggested_new_manual_offset found in %s." % source)
        old_x, old_y = manual_storage_offset(cfg)
        new_x = float(suggested["x"])
        new_y = float(suggested["y"])
        cfg["manual_final_storage_offset_mm"] = {"x": new_x, "y": new_y}
        save_config(cfg)
        log("StorageTune", "Applied suggested manual storage offset from %s: old=(%.5f, %.5f) new=(%.5f, %.5f)" %
            (source, old_x, old_y, new_x, new_y))
        return {
            "type": "apply_suggested_storage_offset",
            "source": source,
            "old_manual_storage_offset": {"x": old_x, "y": old_y},
            "new_manual_storage_offset": {"x": new_x, "y": new_y},
            "config_path": config_path(),
            "timestamp": time.time()
        }
    finally:
        _progress_callback = previous_progress
        _cancel_token = previous_cancel


def run(apply_offsets=False, progress_callback=None, cancel_token=None):
    global _progress_callback, _cancel_token
    previous_progress = _progress_callback
    previous_cancel = _cancel_token
    _progress_callback = progress_callback
    _cancel_token = cancel_token

    cfg = load_config()
    cfg["_apply_offsets_requested"] = bool(apply_offsets)
    result = None
    nozzle = None
    camera = None
    part = None
    storage_loc = None
    cal_loc = None
    state = None
    detected_storage_pick_loc = None

    try:
        log("Init", "Loading and validating %s" % config_path())
        machine_obj, head, nozzle, camera, part, storage_loc, cal_loc = _prepare_runtime(cfg)
        if bool(apply_offsets):
            enforce_write_guard(cfg)

        log("Init", "Nozzle=%s Camera=%s Part=%s" % (nozzle.getName(), camera.getName(), part.getId()))
        prepare_nozzle_for_part(nozzle, part)
        check_cancel("Init")

        state = {"die_on_nozzle": False, "die_at_work": False,
                 "current_die_loc": None, "current_die_loc_trusted": False,
                 "last_trusted_die_loc": None, "detected_storage_pick_loc": None,
                 "last_pick_x_mm": None, "last_pick_y_mm": None,
                 "last_pick_rotation_deg": None, "last_pick_detected_rotation_deg": None}
        detected_storage_pick_loc = transfer_die_from_storage_to_cal(
            camera, nozzle, part, storage_loc, cal_loc, cfg, state
        )

        theta_probe = run_orientation_sign_probe(camera, nozzle, part, cal_loc, cfg, state)
        rebaseline_after_theta_probe(camera, nozzle, part, cal_loc, cfg, state)
        pivot_computed, pivot_samples = run_pivot_probe(camera, nozzle, part, cal_loc, cfg, state)

        test_samples = []
        diagnostic_computed = None
        if pivot_computed is None:
            test_samples, current_die_loc = run_calibration_zone_measure_cycles(
                camera, nozzle, part, cal_loc, cfg, int(cfg["test_iterations"]),
                {"x": 0.0, "y": 0.0, "theta": 0.0}, "test", state
            )
            state["current_die_loc"] = current_die_loc
            diagnostic_computed = summarize_samples(test_samples, "computed")
            computed = diagnostic_computed
            correction = {
                "x": float(computed.get("offset_x_mm", 0.0)),
                "y": float(computed.get("offset_y_mm", 0.0)),
                "theta": float(computed.get("rotation_error_deg", 0.0))
            }
        else:
            computed = pivot_computed
            correction = dict(computed)
            correction["theta"] = float(computed.get("theta_error_deg", 0.0))
            correction["use_pivot_correction"] = True

        log("Compute", "Computed model=%s bias/offset X=%.5f Y=%.5f mag=%.5f repeatability_rms=%.5f" %
            (computed.get("model", "simple_xy"), computed["offset_x_mm"], computed["offset_y_mm"],
             computed["offset_mag_mm"], computed["repeatability_rms_mm"]))
        max_apply = float(cfg.get("max_apply_mm", 0.5))
        if float(computed.get("offset_mag_mm", 0.0)) > max_apply:
            log("Compute", "Computed offset exceeds max_apply_mm, so machine write/apply is blocked.")
        abort_reason = computed_offset_too_large_reason(computed, cfg)
        if abort_reason is not None:
            log("Verify", abort_reason)
            result = {
                "timestamp": time.time(),
                "apply_offsets_requested": bool(apply_offsets),
                "part_id_or_name": cfg["part_id_or_name"],
                "nozzle_name": nozzle.getName(),
                "camera_name": camera.getName(),
                "computed": computed,
                "applied": None,
                "verification": skipped_verification_summary(abort_reason),
                "verification_skipped": True,
                "verification_passed": False,
                "verification_pass_max_offset_mm": float(cfg.get("verification_pass_max_offset_mm", 0.20)),
                "verification_pass_max_theta_deg": float(cfg.get("verification_pass_max_theta_deg", 1.0)),
                "abort_reason": abort_reason,
                "storage": storage_result(storage_loc, detected_storage_pick_loc),
                "theta_probe": theta_probe,
                "pivot_samples": pivot_samples,
                "test_samples": test_samples,
                "diagnostic_computed": diagnostic_computed,
                "verification_samples": [],
                "overlay": pivot_samples[-1].get("overlay", None) if pivot_samples else (test_samples[-1].get("overlay", None) if test_samples else None),
                "storage_return": {"status": "pending"}
            }
            result["storage_return"] = return_die_to_storage(
                camera, nozzle, part, storage_loc, cfg, state, detected_storage_pick_loc,
                None, False, False)
            result["final_pick_rotation_deg"] = result["storage_return"].get("final_pick_rotation_deg", None)
            result["storage_closed_loop_return"] = state.get("storage_closed_loop_return", None)
            save_result(result)
            log_result_summary(result)
            return result

        applied = None
        if computed.get("model", "") in ["bias_plus_rotating_pivot_vector", "bias_plus_pick_place_vectors"] and bool(apply_offsets):
            log("Apply", "apply_offsets=True ignored for vector model; pick/place compensation cannot be written as a static machine offset.")
            verification_correction = correction
        elif bool(apply_offsets) and float(computed.get("offset_mag_mm", 0.0)) <= max_apply:
            applied = apply_offsets_to_machine(nozzle, camera, computed, cfg)
            verification_correction = {
                "x": 0.0,
                "y": 0.0,
                "theta": float(computed.get("rotation_error_deg", 0.0))
            }
        elif bool(apply_offsets):
            log("Apply", "apply_offsets=True but computed offset exceeds max_apply_mm; no machine calibration storage was modified.")
            log("Verify", "Dry-run verification will still run using computed correction because it is below max_verify_correction_mm.")
            verification_correction = correction
        else:
            log("Apply", "apply_offsets=False; no machine calibration storage was modified.")
            if float(computed.get("offset_mag_mm", 0.0)) > max_apply:
                log("Verify", "Dry-run verification will still run using computed correction because it is below max_verify_correction_mm.")
            verification_correction = correction

        verification_samples, current_die_loc = run_calibration_zone_measure_cycles(
            camera, nozzle, part, cal_loc, cfg, int(cfg["verification_iterations"]),
            verification_correction, "verify", state
        )
        state["current_die_loc"] = current_die_loc

        verification = summarize_samples(verification_samples, "verification")
        verification_passed = verification_passed_summary(verification, cfg)
        result = {
            "timestamp": time.time(),
            "apply_offsets_requested": bool(apply_offsets),
            "part_id_or_name": cfg["part_id_or_name"],
            "nozzle_name": nozzle.getName(),
            "camera_name": camera.getName(),
            "computed": computed,
            "applied": applied,
            "verification": verification,
            "verification_passed": verification_passed,
            "verification_pass_max_offset_mm": float(cfg.get("verification_pass_max_offset_mm", 0.20)),
            "verification_pass_max_theta_deg": float(cfg.get("verification_pass_max_theta_deg", 1.0)),
            "storage": storage_result(storage_loc, detected_storage_pick_loc),
            "theta_probe": theta_probe,
            "pivot_samples": pivot_samples,
            "test_samples": test_samples,
            "diagnostic_computed": diagnostic_computed,
            "verification_samples": verification_samples,
            "overlay": verification_samples[-1].get("overlay", None) if verification_samples else None,
            "storage_return": {"status": "pending"}
        }
        log("Verify", "Verification offset X=%.5f Y=%.5f mag=%.5f repeatability_rms=%.5f" %
            (verification["offset_x_mm"], verification["offset_y_mm"],
             verification["offset_mag_mm"], verification["repeatability_rms_mm"]))
        result["storage_return"] = return_die_to_storage(
            camera, nozzle, part, storage_loc, cfg, state, detected_storage_pick_loc,
            computed, applied is not None, verification_passed)
        result["final_pick_rotation_deg"] = result["storage_return"].get("final_pick_rotation_deg", None)
        result["storage_closed_loop_return"] = state.get("storage_closed_loop_return", None)
        save_result(result)
        log_result_summary(result)
        return result

    except Exception, e:
        log("Error", e)
        if state is not None and storage_loc is not None and cal_loc is not None and camera is not None and part is not None:
            if state.get("die_on_nozzle", False) or state.get("die_at_work", False):
                cleanup_after_calibration_failure(camera, nozzle, part, storage_loc, cal_loc, cfg, state, str(e))
        if result is None:
            result = {"error": str(e), "timestamp": time.time(), "traceback": traceback.format_exc()}
        else:
            result["error"] = str(e)
            result["traceback"] = traceback.format_exc()
        save_result(result)
        raise
    finally:
        if nozzle is not None:
            try:
                loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
                lift = mm_loc(loc.getX(), loc.getY(), float(cfg["safe_travel_z"]), loc.getRotation())
                move_to_with_speed(nozzle, nearest_nozzle_rotation_location(nozzle, lift, "Cleanup"), cfg)
                log("Cleanup", "Nozzle lifted to safe Z.")
            except Exception, cleanup_error:
                log("Cleanup", "Final safe lift failed: %s" % cleanup_error)
        _progress_callback = previous_progress
        _cancel_token = previous_cancel


if __name__ == "__main__":
    print json.dumps(run(False), sort_keys=True, indent=2)
