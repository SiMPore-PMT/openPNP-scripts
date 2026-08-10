# needle_calibration.py
#
# Purpose-built dispensing needle calibration for OpenPnP Jython 2.7.
# Workflow:
#   1. Z touch-off on one conductive pad.
#   2. XY measurement by probing the side of a raised metal post.

import json
import math
import os

from java.lang import Thread, Throwable
from org.openpnp.model import Configuration, LengthUnit, Location
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import MovableUtils, VisionUtils
from org.openpnp.vision.pipeline import CvPipeline


try:
    basestring
except NameError:
    basestring = str


try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except:
    SCRIPT_DIR = os.getcwd()
SCRIPTS_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
DEFAULT_CONFIG_PATH = os.path.join(SCRIPTS_ROOT, "needle_calibration_config.json")


POST_ANGLES_8 = [0, 45, 90, 135, 180, 225, 270, 315]
LEGACY_CONFIG_KEYS = [
    "bottom_camera", "xy_bootstrap", "optional_verification", "edge_scan",
    "pattern_probe", "solver", "virtual_transform", "failure_policy"
]
LEGACY_FIDUCIAL_KEYS = ["coarse_xy", "fine_xy"]


DEFAULT_CONFIG = {
    "schema_version": 2,

    "head_name": "",
    "nozzle_name": "Dispense Head - Needle",
    "top_camera_name": "",
    "contact_actuator_name": "NeedleContactProbe_Read",

    "safe_z_mm": 25.0,
    "travel_speed": 1.0,
    "probe_speed": 0.20,
    "fine_probe_speed": 0.08,

    "max_apply_delta_mm": 5.0,
    "xy_offset_sign": 1.0,
    "z_offset_sign": 1.0,

    "z_probe": {
        "start_above_mm": 4.0,
        "max_descent_mm": 8.0,
        "coarse_step_mm": 0.10,
        "fine_step_mm": 0.02,
        "retract_after_contact_mm": 0.40,
        "verify_retract_mm": 0.40,
        "release_retract_step_mm": 0.20,
        "max_release_retract_mm": 2.0,
        "read_settle_ms": 80,
        "debounce_count": 3,
        "debounce_retries": 3,
        "debounce_interval_ms": 20,
        "verify_enabled": True,
        "verify_mode": "fast",
        "verify_start_above_mm": 0.75,
        "verify_coarse_step_mm": 0.05,
        "max_fine_backtrack_mm": 0.15,
        "log_each_step": False,
        "log_summary_only": True
    },

    "fiducials": {
        "z_touch": {
            "location": {"x": 0.0, "y": 0.0, "z": 0.0, "rotation": 0.0},
            "result_stage": "results",
            "pipeline": {"mode": "named", "name": "", "path": ""}
        }
    },

    "post_xy": {
        "location": {"x": 0.0, "y": 0.0, "z": 0.0, "rotation": 0.0},
        "post_diameter_mm": 6.35,
        "result_stage": "results",
        "pipeline": {"mode": "named", "name": "", "path": ""},

        "search_radius_mm": 1.0,
        "scan_clearance_mm": 2.0,
        "scan_overtravel_mm": 1.0,
        "max_radial_probe_travel_mm": 4.0,

        "probe_z_mode": "post_location_plus_offset",
        "side_probe_z_offset_mm": 0.20,
        "allow_configured_probe_z": False,
        "configured_probe_z_mm": None,

        "local_lift_mm": 0.60,
        "release_lift_mm": 0.60,
        "max_release_lift_mm": 3.0,

        "coarse_step_mm": 0.10,
        "fine_step_mm": 0.01,
        "fine_backoff_mm": 0.15,

        "read_settle_ms": 10,
        "debounce_count": 1,
        "debounce_retries": 1,
        "debounce_interval_ms": 0,

        "calibration_angles_deg": [0, 90, 180, 270],
        "verify_angles_deg": POST_ANGLES_8,

        "outside_start_clearance_multiplier": 1.0,
        "outside_retry_clearance_multiplier": 2.0,
        "max_outside_retries": 2,

        "abort_if_outside_start_touching_after_retries": True,
        "touching_start_safe_z_recover": True,

        "calibration_probe_profile": "coarse_fine",
        "verify_probe_profile": "single_touch",
        "single_touch_step_mm": 0.05,
        "single_touch_speed": 0.20,
        "single_touch_use_coarse_step": True,
        "use_verify_fit_for_proposal": False,
        "run_fast_verify_after_cardinal": True,
        "max_verify_center_delta_mm": 0.10,
        "max_verify_rms_mm": 0.08,
        "max_verify_contact_radius_error_mm": 0.15,
        "fail_on_fast_verify_failure": False,

        "repeat_measurement_count": 1,
        "max_center_error_mm": 2.0,
        "max_verify_residual_mm": 0.05,
        "min_effective_contact_diameter_mm": 0.0,
        "max_effective_contact_diameter_mm": 1.5,

        "apply_xy_offsets": False
    },

    "debug": {
        "verbose": False,
        "save_last_capture": True
    }
}


class NeedleCalibrationError(Exception):
    pass


def log_msg(logger, msg):
    if logger:
        logger(str(msg))
    else:
        print(str(msg))


def deep_copy(value):
    return json.loads(json.dumps(value, sort_keys=True))


def deep_merge(defaults, loaded):
    out = deep_copy(defaults)
    if loaded is None:
        return out
    for key in loaded:
        value = loaded[key]
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def clean_to_schema(defaults, loaded):
    out = deep_copy(defaults)
    if not isinstance(loaded, dict):
        return out
    for key in defaults:
        if key not in loaded:
            continue
        if isinstance(defaults.get(key), dict) and isinstance(loaded.get(key), dict):
            out[key] = clean_to_schema(defaults.get(key), loaded.get(key))
        else:
            out[key] = loaded.get(key)
    return out


def legacy_warnings(loaded):
    dropped = []
    if not isinstance(loaded, dict):
        return dropped
    for key in LEGACY_CONFIG_KEYS:
        if key in loaded:
            dropped.append(key)
    fids = loaded.get("fiducials", {})
    if isinstance(fids, dict):
        for key in LEGACY_FIDUCIAL_KEYS:
            if key in fids:
                dropped.append("fiducials.%s" % key)
    return dropped


def safety_warnings(loaded):
    warnings = []
    if not isinstance(loaded, dict):
        return warnings
    post = loaded.get("post_xy", {})
    if isinstance(post, dict):
        try:
            if float(post.get("scan_clearance_mm", 2.0)) < 1.0:
                warnings.append(
                    "post_xy.scan_clearance_mm is below 1.0 mm; 2.0 mm is recommended during bring-up.")
        except:
            pass
    return warnings


def load_config(path=None):
    cfg_path = path or DEFAULT_CONFIG_PATH
    if not os.path.exists(cfg_path):
        cfg = deep_copy(DEFAULT_CONFIG)
        cfg["_config_path"] = cfg_path
        return cfg
    fh = open(cfg_path, "r")
    try:
        loaded = json.loads(fh.read())
    finally:
        fh.close()
    cfg = clean_to_schema(DEFAULT_CONFIG, loaded)
    cfg["_config_path"] = cfg_path
    dropped = legacy_warnings(loaded)
    warnings = safety_warnings(loaded)
    if len(dropped):
        msg = "Loaded legacy config; dropped old XY keys: %s" % ", ".join(dropped)
        cfg["_migration_warnings"] = [msg]
        log_msg(None, msg)
    if len(warnings):
        current = cfg.get("_migration_warnings", [])
        current.extend(warnings)
        cfg["_migration_warnings"] = current
        for msg in warnings:
            log_msg(None, msg)
    return cfg


def public_clean_config(cfg):
    clean = clean_to_schema(DEFAULT_CONFIG, cfg)
    clean["schema_version"] = 2
    return clean


def save_config(cfg, path=None):
    cfg_path = path or cfg.get("_config_path", DEFAULT_CONFIG_PATH)
    clean = public_clean_config(cfg)
    fh = open(cfg_path, "w")
    try:
        fh.write(json.dumps(clean, sort_keys=True, indent=2))
    finally:
        fh.close()
    return cfg_path


def mm_loc(x, y, z, r=0.0):
    return Location(LengthUnit.Millimeters, float(x), float(y), float(z), float(r))


def loc_to_dict(loc):
    loc = loc.convertToUnits(LengthUnit.Millimeters)
    return {
        "x": float(loc.getX()),
        "y": float(loc.getY()),
        "z": float(loc.getZ()),
        "rotation": float(loc.getRotation())
    }


def loc_to_log(loc):
    d = loc_to_dict(loc)
    return "X %.6f Y %.6f Z %.6f R %.6f" % (
        d["x"], d["y"], d["z"], d["rotation"])


def dict_to_loc(data, fallback=None):
    if hasattr(data, "convertToUnits"):
        return data.convertToUnits(LengthUnit.Millimeters)
    if fallback is None:
        fallback = Location(LengthUnit.Millimeters)
    fallback = fallback.convertToUnits(LengthUnit.Millimeters)
    if data is None:
        return fallback
    return mm_loc(
        data.get("x", fallback.getX()),
        data.get("y", fallback.getY()),
        data.get("z", fallback.getZ()),
        data.get("rotation", data.get("r", fallback.getRotation())))


def java_list_to_py(model):
    if model is None:
        return []
    if isinstance(model, list):
        return model
    if hasattr(model, "size") and hasattr(model, "get"):
        out = []
        for i in range(model.size()):
            out.append(model.get(i))
        return out
    try:
        return list(model)
    except:
        return [model]


def object_name(obj):
    if obj is None:
        return ""
    for name in ["getName", "getId"]:
        try:
            value = getattr(obj, name)()
            if value is not None:
                return str(value)
        except:
            pass
    return str(obj)


def get_machine(machine_obj=None):
    if machine_obj is not None:
        return machine_obj
    try:
        return machine
    except:
        return Configuration.get().getMachine()


def all_heads(machine_obj):
    heads = []
    try:
        heads = java_list_to_py(machine_obj.getHeads())
    except:
        pass
    if len(heads) == 0:
        try:
            head = machine_obj.getDefaultHead()
            if head is not None:
                heads.append(head)
        except:
            pass
    return heads


def find_head(machine_obj, head_name):
    wanted = str(head_name or "").strip()
    if wanted == "":
        try:
            head = machine_obj.getDefaultHead()
            if head is not None:
                return head
        except:
            pass
    for head in all_heads(machine_obj):
        if wanted == "" or object_name(head) == wanted:
            return head
    raise NeedleCalibrationError("Head not found: %s" % wanted)


def find_nozzle(cfg, nozzle=None, machine_obj=None):
    if nozzle is not None:
        return nozzle
    machine_obj = get_machine(machine_obj)
    wanted = str(cfg.get("nozzle_name", "") or "").strip()
    heads = all_heads(machine_obj)
    if str(cfg.get("head_name", "") or "").strip():
        heads = [find_head(machine_obj, cfg.get("head_name"))]
    for head in heads:
        if wanted:
            try:
                found = head.getNozzleByName(wanted)
                if found is not None:
                    return found
            except:
                pass
            try:
                for item in java_list_to_py(head.getNozzles()):
                    if object_name(item) == wanted:
                        return item
            except:
                pass
        else:
            try:
                found = head.getDefaultNozzle()
                if found is not None:
                    return found
            except:
                pass
    raise NeedleCalibrationError("Nozzle not found: %s" % (wanted or "<default>"))


def find_nozzle_by_name(machine_obj, name):
    wanted = str(name or "").strip()
    if wanted == "":
        raise NeedleCalibrationError("Nozzle name is required")
    for head in all_heads(machine_obj):
        try:
            found = head.getNozzleByName(wanted)
            if found is not None:
                return found
        except:
            pass
        try:
            for item in java_list_to_py(head.getNozzles()):
                if object_name(item) == wanted:
                    return item
        except:
            pass
    raise NeedleCalibrationError("Nozzle not found: %s" % wanted)


def find_top_camera(cfg, machine_obj=None):
    machine_obj = get_machine(machine_obj)
    wanted = str(cfg.get("top_camera_name", "") or "").strip()
    if wanted:
        for head in all_heads(machine_obj):
            try:
                for cam in java_list_to_py(head.getCameras()):
                    if object_name(cam) == wanted:
                        return cam
            except:
                pass
        try:
            for cam in java_list_to_py(machine_obj.getCameras()):
                if object_name(cam) == wanted:
                    return cam
        except:
            pass
        raise NeedleCalibrationError("Top camera not found: %s" % wanted)
    try:
        cam = machine_obj.getDefaultHead().getDefaultCamera()
        if cam is not None:
            return cam
    except:
        pass
    for head in all_heads(machine_obj):
        try:
            for cam in java_list_to_py(head.getCameras()):
                if cam.getLooking() == SpiCamera.Looking.Down:
                    return cam
        except:
            pass
    raise NeedleCalibrationError("No top camera found")


def find_actuator_by_name(machine_obj, cfg):
    name = str(cfg.get("contact_actuator_name", "") or "").strip()
    if name == "":
        raise NeedleCalibrationError("contact_actuator_name is required")
    owners = [machine_obj]
    try:
        nozzle = find_nozzle(cfg, None, machine_obj)
        owners.insert(0, nozzle.getHead())
    except:
        pass
    for owner in owners:
        if owner is None:
            continue
        for method in ["getActuatorByName", "getActuator"]:
            try:
                actuator = getattr(owner, method)(name)
                if actuator is not None:
                    return actuator
            except:
                pass
        try:
            for actuator in java_list_to_py(owner.getActuators()):
                if object_name(actuator) == name:
                    return actuator
        except:
            pass
    raise NeedleCalibrationError("Contact actuator not found: %s" % name)


def wait_still(movable):
    movable.waitForCompletion(CompletionType.WaitForStillstand)


def move_safe(movable, target):
    target = target.convertToUnits(LengthUnit.Millimeters)
    MovableUtils.moveToLocationAtSafeZ(movable, target)
    wait_still(movable)


def move_direct_speed(movable, target, speed=None):
    target = target.convertToUnits(LengthUnit.Millimeters)
    if speed is None:
        try:
            movable.moveTo(target)
        except TypeError:
            movable.moveTo(target, 1.0)
    else:
        try:
            movable.moveTo(target, float(speed))
        except TypeError:
            movable.moveTo(target)
    wait_still(movable)


def get_movable_location(movable):
    return movable.getLocation().convertToUnits(LengthUnit.Millimeters)


def lift_nozzle_to_safe_z(nozzle, cfg, logger=None):
    current = get_movable_location(nozzle)
    safe_z = float(cfg.get("safe_z_mm", 25.0))
    if float(current.getZ()) >= safe_z:
        return current
    target = mm_loc(current.getX(), current.getY(), safe_z, current.getRotation())
    log_msg(logger, "Retract to safe Z: %s" % loc_to_log(target))
    move_direct_speed(nozzle, target, cfg.get("travel_speed", 1.0))
    return get_movable_location(nozzle)


def parse_contact_state(value):
    if value is None:
        raise NeedleCalibrationError("Contact actuator returned None")
    if isinstance(value, bool):
        return not value
    try:
        return float(value) == 0.0
    except:
        pass
    text = str(value).strip().lower()
    if text in ["0", "false", "f", "no", "n", "off", "low", "closed", "contact"]:
        return True
    if text in ["1", "true", "t", "yes", "y", "on", "high", "open", "released"]:
        return False
    raise NeedleCalibrationError("Cannot parse contact actuator value: %s" % value)


def read_actuator_raw(actuator):
    for method in ["read", "getRead", "getValue", "getState", "isActuated"]:
        try:
            return getattr(actuator, method)()
        except:
            pass
    raise NeedleCalibrationError("Contact actuator has no readable method: %s" %
        object_name(actuator))


def read_contact_once(actuator):
    return parse_contact_state(read_actuator_raw(actuator))


def read_contact_debounced(actuator, read_cfg, expect_contact=None, logger=None,
        label="contact", mixed_result=False):
    count = int(read_cfg.get("debounce_count", 1))
    if count < 1:
        count = 1
    interval_ms = int(read_cfg.get("debounce_interval_ms", 0))
    retries = int(read_cfg.get("debounce_retries", 3))
    if retries < 1:
        retries = 1
    last_readings = []
    for attempt in range(retries):
        readings = []
        for i in range(count):
            readings.append(read_contact_once(actuator))
            if i + 1 < count and interval_ms > 0:
                Thread.sleep(interval_ms)
        first = readings[0]
        stable = True
        for item in readings:
            if item != first:
                stable = False
                break
        if stable:
            if bool(read_cfg.get("log_reads", False)):
                log_msg(logger, "%s stable=%s readings=%s" % (label, first, readings))
            if expect_contact is not None and first != bool(expect_contact):
                raise NeedleCalibrationError("Contact state mismatch expected=%s actual=%s readings=%s" %
                    (bool(expect_contact), first, readings))
            return first
        last_readings = readings
        log_msg(logger, "%s unstable contact readings attempt %d/%d: %s" %
            (label, attempt + 1, retries, readings))
    if expect_contact is not None:
        raise NeedleCalibrationError("Contact state mismatch expected=%s actual=%s" %
            (bool(expect_contact), last_readings))
    return bool(mixed_result)


def dwell_for_read(read_cfg):
    ms = int(read_cfg.get("read_settle_ms", 0))
    if ms > 0:
        Thread.sleep(ms)


def z_read_cfg(cfg):
    return deep_copy(cfg.get("z_probe", {}))


def post_read_cfg(cfg):
    return deep_copy(cfg.get("post_xy", {}))


def get_attr(obj, names):
    for name in names:
        try:
            if isinstance(obj, dict) and name in obj:
                return obj.get(name)
        except:
            pass
        try:
            return getattr(obj, name)
        except:
            pass
        try:
            getter = "get" + name[:1].upper() + name[1:]
            if hasattr(obj, getter):
                return getattr(obj, getter)()
        except:
            pass
    return None


def number_attr(obj, names):
    value = get_attr(obj, names)
    if value is None:
        return None
    try:
        return float(value)
    except:
        return None


def is_location(obj):
    return hasattr(obj, "convertToUnits") and hasattr(obj, "getX") and hasattr(obj, "getY")


def parse_feature_item(item):
    if is_location(item):
        loc = item.convertToUnits(LengthUnit.Millimeters)
        return {
            "kind": "location",
            "x_mm": float(loc.getX()),
            "y_mm": float(loc.getY()),
            "z_mm": float(loc.getZ()),
            "location": loc,
            "raw": str(item)
        }
    center = get_attr(item, ["center", "pt", "point"])
    if center is not None:
        x = number_attr(center, ["x"])
        y = number_attr(center, ["y"])
    else:
        x = number_attr(item, ["x"])
        y = number_attr(item, ["y"])
    diameter = number_attr(item, ["diameter", "d", "size"])
    radius = number_attr(item, ["radius", "r"])
    if diameter is None and radius is not None:
        diameter = radius * 2.0
    if x is None or y is None:
        raise NeedleCalibrationError("Pipeline feature has no readable x/y: %s" % item)
    out = {
        "kind": "pixel",
        "x_px": float(x),
        "y_px": float(y),
        "raw": str(item)
    }
    if diameter is not None:
        out["diameter_px"] = float(diameter)
        out["radius_px"] = float(diameter) / 2.0
    elif radius is not None:
        out["radius_px"] = float(radius)
        out["diameter_px"] = float(radius) * 2.0
    return out


def flatten_model(model):
    out = []
    for item in java_list_to_py(model):
        if item is None:
            continue
        if is_location(item):
            out.append(item)
        elif isinstance(item, list):
            out.extend(item)
        elif hasattr(item, "size") and hasattr(item, "get") and "Point" not in str(type(item)):
            out.extend(java_list_to_py(item))
        else:
            out.append(item)
    return out


def find_named_vision_settings(name):
    cfg = Configuration.get()
    wanted = str(name or "").strip()
    for method in ["getVisionSettings", "getVisionSettingsByName"]:
        try:
            value = getattr(cfg, method)(wanted)
            if value is not None:
                return value
        except:
            pass
    for owner in [cfg, get_machine(None)]:
        for method in ["getVisionSettings", "getVisionSettingsList"]:
            try:
                items = java_list_to_py(getattr(owner, method)())
                for item in items:
                    if object_name(item) == wanted:
                        return item
            except:
                pass
    return None


def load_pipeline_from_source(source):
    if hasattr(source, "process") and hasattr(source, "setProperty"):
        return source
    mode = str(source.get("mode", "named") or "named").lower()
    if mode in ["named", "vision_settings", "named_vision_settings"]:
        name = str(source.get("name", "") or "").strip()
        if name == "":
            raise NeedleCalibrationError("Named vision settings source requires name")
        settings = find_named_vision_settings(name)
        if settings is None:
            raise NeedleCalibrationError("Named vision settings not found: %s" % name)
        pipeline = settings.getPipeline()
        if pipeline is None:
            raise NeedleCalibrationError("Named vision settings has no pipeline: %s" % name)
        return pipeline
    if mode not in ["xml_path", "file", "path"]:
        raise NeedleCalibrationError("Only XML path or named pipeline mode is supported: %s" % mode)
    path = str(source.get("path", "") or "").strip()
    if path == "":
        raise NeedleCalibrationError("Pipeline XML path is empty")
    if not os.path.isabs(path):
        path = os.path.join(SCRIPTS_ROOT, path)
    fh = open(path, "r")
    try:
        xml = fh.read()
    finally:
        fh.close()
    return CvPipeline(str(xml))


def add_vision_settings_names_from_owner(owner, names):
    if owner is None:
        return
    for method in ["getVisionSettings", "getVisionSettingsList"]:
        try:
            value = getattr(owner, method)()
            for item in java_list_to_py(value):
                names.append(object_name(item))
        except:
            pass
    for method in ["getVisionSettings", "getVisionSettingsByName"]:
        try:
            value = getattr(owner, method)()
            if value is not None and not isinstance(value, basestring):
                for item in java_list_to_py(value):
                    names.append(object_name(item))
        except:
            pass


def add_nozzle_tip_pipeline_names(nozzle, names):
    owners = [nozzle]
    for method in ["getNozzleTip", "getCalibration", "getNozzleTipCalibration"]:
        more = []
        for owner in owners:
            try:
                value = getattr(owner, method)()
                if value is not None:
                    more.append(value)
            except:
                pass
        owners.extend(more)
    seen_owners = {}
    for owner in owners:
        key = str(owner)
        if key in seen_owners:
            continue
        seen_owners[key] = True
        add_vision_settings_names_from_owner(owner, names)


def list_available_vision_pipeline_names(machine_obj=None):
    names = []
    try:
        cfg = Configuration.get()
        add_vision_settings_names_from_owner(cfg, names)
    except:
        pass
    try:
        machine_obj = get_machine(machine_obj)
        add_vision_settings_names_from_owner(machine_obj, names)
        for head in all_heads(machine_obj):
            add_vision_settings_names_from_owner(head, names)
            try:
                for cam in java_list_to_py(head.getCameras()):
                    add_vision_settings_names_from_owner(cam, names)
            except:
                pass
            try:
                for nozzle in java_list_to_py(head.getNozzles()):
                    add_vision_settings_names_from_owner(nozzle, names)
                    add_nozzle_tip_pipeline_names(nozzle, names)
            except:
                pass
        try:
            for cam in java_list_to_py(machine_obj.getCameras()):
                add_vision_settings_names_from_owner(cam, names)
        except:
            pass
    except:
        pass
    seen = {}
    out = []
    for name in names:
        text = str(name or "").strip()
        if text and text not in seen:
            seen[text] = True
            out.append(text)
    out.sort()
    return out


def get_result_model(pipeline, stage_name):
    result = pipeline.getResult(stage_name)
    if result is None:
        raise NeedleCalibrationError("No pipeline result from stage '%s'" % stage_name)
    try:
        return result.getModel()
    except:
        return result


def add_machine_coordinates_to_feature(camera, nozzle, feat):
    if feat["kind"] == "pixel":
        px = feat["x_px"]
        py = feat["y_px"]
        loc = VisionUtils.getPixelLocation(camera, nozzle, float(px), float(py))
        loc = loc.convertToUnits(LengthUnit.Millimeters)
        feat["x_mm"] = float(loc.getX())
        feat["y_mm"] = float(loc.getY())
        feat["z_mm"] = float(loc.getZ())
        feat["location"] = loc
        if feat.get("diameter_px", None) is not None:
            try:
                rp = float(feat["diameter_px"]) / 2.0
                left = VisionUtils.getPixelLocation(camera, nozzle, px - rp, py).convertToUnits(LengthUnit.Millimeters)
                right = VisionUtils.getPixelLocation(camera, nozzle, px + rp, py).convertToUnits(LengthUnit.Millimeters)
                bottom = VisionUtils.getPixelLocation(camera, nozzle, px, py - rp).convertToUnits(LengthUnit.Millimeters)
                top = VisionUtils.getPixelLocation(camera, nozzle, px, py + rp).convertToUnits(LengthUnit.Millimeters)
                dx = math.sqrt(math.pow(right.getX() - left.getX(), 2.0) + math.pow(right.getY() - left.getY(), 2.0))
                dy = math.sqrt(math.pow(top.getX() - bottom.getX(), 2.0) + math.pow(top.getY() - bottom.getY(), 2.0))
                feat["diameter_mm"] = (dx + dy) / 2.0
                feat["radius_mm"] = feat["diameter_mm"] / 2.0
            except:
                pass
    return feat


def capture_fiducial_features(cfg, role, nozzle=None, machine_obj=None, logger=None):
    machine_obj = get_machine(machine_obj)
    nozzle = find_nozzle(cfg, nozzle, machine_obj)
    fid = cfg.get("fiducials", {}).get(role)
    if fid is None:
        raise NeedleCalibrationError("Missing fiducial config: %s" % role)
    camera = find_top_camera(cfg, machine_obj)
    target = dict_to_loc(fid.get("location"))
    log_msg(logger, "Capture %s at %s" % (role, loc_to_log(target)))
    move_safe(camera, target)

    pipeline = load_pipeline_from_source(fid.get("pipeline", {}))
    pipeline.setProperty("camera", camera)
    pipeline.setProperty("nozzle", nozzle)
    image = camera.settleAndCapture()
    pipeline.process()

    stage_name = fid.get("result_stage", "results")
    items = flatten_model(get_result_model(pipeline, stage_name))
    if len(items) == 0:
        raise NeedleCalibrationError("Pipeline stage '%s' returned no features" % stage_name)

    features = []
    for item in items:
        feat = parse_feature_item(item)
        feat = add_machine_coordinates_to_feature(camera, nozzle, feat)
        features.append(feat)
    log_msg(logger, "Capture %s found %d feature(s)" % (role, len(features)))
    out = {
        "role": role,
        "camera": object_name(camera),
        "stage": stage_name,
        "camera_location_mm": loc_to_dict(get_movable_location(camera)),
        "features": features
    }
    if bool(cfg.get("debug", {}).get("save_last_capture", True)):
        out["image"] = image
    return out


def single_feature_location(capture, role):
    features = capture.get("features", [])
    if len(features) < 1:
        raise NeedleCalibrationError("%s capture has no features" % role)
    f = features[0]
    return mm_loc(f["x_mm"], f["y_mm"], f.get("z_mm", 0.0), 0.0)


def retract_until_released(nozzle, actuator, read_cfg, x_mm, y_mm, step_mm,
        max_mm, speed, logger=None, label="release"):
    moved = 0.0
    while True:
        dwell_for_read(read_cfg)
        if not read_contact_debounced(actuator, read_cfg, logger=logger, label=label,
                mixed_result=True):
            return moved
        if moved >= max_mm - 1.0e-9:
            raise NeedleCalibrationError("%s: contact did not release after %.6f mm" %
                (label, max_mm))
        inc = min(float(step_mm), float(max_mm) - moved)
        moved += inc
        current = get_movable_location(nozzle)
        target = mm_loc(x_mm, y_mm, current.getZ() + inc, current.getRotation())
        log_msg(logger, "%s: retract to %s" % (label, loc_to_log(target)))
        move_direct_speed(nozzle, target, speed)


def descend_until_contact(nozzle, actuator, read_cfg, x_mm, y_mm, start_z,
        limit_z, step_mm, speed, logger=None, label="probe", return_steps=False):
    z = float(start_z)
    step = abs(float(step_mm))
    steps = 0
    log_each = bool(read_cfg.get("log_each_step", True))
    while z - step >= float(limit_z) - 1.0e-9:
        z = z - step
        steps += 1
        target = mm_loc(x_mm, y_mm, z, get_movable_location(nozzle).getRotation())
        if log_each:
            log_msg(logger, "%s: probe step Z %.6f" % (label, z))
        move_direct_speed(nozzle, target, speed)
        dwell_for_read(read_cfg)
        if read_contact_debounced(actuator, read_cfg, logger=logger, label=label):
            contact = get_movable_location(nozzle)
            log_msg(logger, "%s: confirmed contact at Z %.6f" % (label, contact.getZ()))
            if return_steps:
                return contact, steps
            return contact
    if return_steps:
        return None, steps
    return None


def probe_z_touch(nozzle, actuator, cfg, x_mm, y_mm, surface_z_mm,
        logger=None, label="z_touch", verify=False):
    zp = z_read_cfg(cfg)
    zp["verbose"] = cfg.get("debug", {}).get("verbose", False)
    if verify and str(zp.get("verify_mode", "fast")).lower() == "fast":
        start_above = float(zp.get("verify_start_above_mm", zp.get("start_above_mm", 4.0)))
        coarse_step = abs(float(zp.get("verify_coarse_step_mm", zp.get("coarse_step_mm", 0.10))))
    else:
        start_above = float(zp.get("start_above_mm", 4.0))
        coarse_step = abs(float(zp.get("coarse_step_mm", 0.10)))
    start_z = float(surface_z_mm) + start_above
    limit_z = start_z - float(zp.get("max_descent_mm", 8.0))
    fine_step = abs(float(zp.get("fine_step_mm", 0.02)))
    speed = float(cfg.get("probe_speed", 0.20))
    fine_speed = float(cfg.get("fine_probe_speed", 0.08))
    retract_after = max(float(zp.get("retract_after_contact_mm", 0.40)),
        float(zp.get("verify_retract_mm", 0.40)))
    fine_backtrack = min(retract_after, abs(float(zp.get("max_fine_backtrack_mm", retract_after))))
    if limit_z > float(surface_z_mm) - fine_step:
        raise NeedleCalibrationError(
            "%s: z_probe max_descent cannot reach surface. surface_z=%.6f start_above=%.6f start_z=%.6f max_descent=%.6f limit_z=%.6f" %
            (label, float(surface_z_mm), start_above,
             start_z, float(zp.get("max_descent_mm", 8.0)), limit_z))

    start = mm_loc(x_mm, y_mm, start_z, get_movable_location(nozzle).getRotation())
    log_msg(logger, "%s: move above touch point %s" % (label, loc_to_log(start)))
    log_msg(logger, "%s: Z probe plan surface=%.6f start_above=%.6f start=%.6f max_descent=%.6f limit=%.6f coarse_step=%.6f fine_step=%.6f fine_backtrack=%.6f" %
        (label, float(surface_z_mm), start_above, start_z,
         float(zp.get("max_descent_mm", 8.0)), limit_z, coarse_step, fine_step,
         fine_backtrack))
    move_safe(nozzle, start)
    dwell_for_read(zp)
    if read_contact_debounced(actuator, zp, logger=logger, label=label + "_precheck"):
        raise NeedleCalibrationError("%s: probe already active above pad" % label)

    coarse, coarse_steps = descend_until_contact(nozzle, actuator, zp, x_mm, y_mm,
        start_z, limit_z, coarse_step, speed, logger, label + "_coarse", True)
    if coarse is None:
        raise NeedleCalibrationError("%s: no coarse contact within max descent. start_z=%.6f limit_z=%.6f step=%.6f" %
            (label, start_z, limit_z, coarse_step))

    retract_z = float(coarse.getZ()) + fine_backtrack
    move_direct_speed(nozzle, mm_loc(x_mm, y_mm, retract_z, coarse.getRotation()), speed)
    coarse_release_extra = retract_until_released(nozzle, actuator, zp, x_mm, y_mm,
        float(zp.get("release_retract_step_mm", 0.20)),
        float(zp.get("max_release_retract_mm", 2.0)),
        speed, logger, label + "_coarse_release")
    if coarse_release_extra > 0.0:
        log_msg(logger, "%s: coarse release needed extra %.6f mm beyond fine backtrack" %
            (label, coarse_release_extra))

    fine, fine_steps = descend_until_contact(nozzle, actuator, zp, x_mm, y_mm,
        get_movable_location(nozzle).getZ(), limit_z, fine_step, fine_speed,
        logger, label + "_fine", True)
    if fine is None:
        raise NeedleCalibrationError("%s: no fine contact after coarse contact. restart_z=%.6f limit_z=%.6f step=%.6f" %
            (label, get_movable_location(nozzle).getZ(), limit_z, fine_step))

    final_retract_z = float(fine.getZ()) + retract_after
    move_direct_speed(nozzle, mm_loc(x_mm, y_mm, final_retract_z, fine.getRotation()), speed)
    extra = retract_until_released(nozzle, actuator, zp, x_mm, y_mm,
        float(zp.get("release_retract_step_mm", 0.20)),
        float(zp.get("max_release_retract_mm", 2.0)),
        speed, logger, label + "_fine_release")
    log_msg(logger, "%s: summary coarse_z=%.6f fine_z=%.6f coarse_steps=%d fine_steps=%d z_error=%.6f" %
        (label, float(coarse.getZ()), float(fine.getZ()), coarse_steps, fine_steps,
         float(surface_z_mm) - float(fine.getZ())))
    return {
        "label": label,
        "coarse_contact": loc_to_dict(coarse),
        "contact": loc_to_dict(fine),
        "surface_z_mm": float(surface_z_mm),
        "z_error_mm": float(surface_z_mm) - float(fine.getZ()),
        "coarse_steps": coarse_steps,
        "fine_steps": fine_steps,
        "coarse_release_extra_mm": coarse_release_extra,
        "release_extra_mm": extra
    }


def proposed_offsets_from_delta(cfg, old_offsets, dx=None, dy=None, dz=None):
    old_offsets = old_offsets.convertToUnits(LengthUnit.Millimeters)
    x = float(old_offsets.getX())
    y = float(old_offsets.getY())
    z = float(old_offsets.getZ())
    if dx is not None:
        x += float(cfg.get("xy_offset_sign", 1.0)) * float(dx)
    if dy is not None:
        y += float(cfg.get("xy_offset_sign", 1.0)) * float(dy)
    if dz is not None:
        z += float(cfg.get("z_offset_sign", 1.0)) * float(dz)
    return mm_loc(x, y, z, old_offsets.getRotation())


def apply_xyz_to_openpnp_nozzle(cfg, nozzle, x=None, y=None, z=None, logger=None):
    old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
    nx = float(old.getX()) if x is None else float(x)
    ny = float(old.getY()) if y is None else float(y)
    nz = float(old.getZ()) if z is None else float(z)
    max_delta = float(cfg.get("max_apply_delta_mm", 5.0))
    for name, before, after in [("X", old.getX(), nx), ("Y", old.getY(), ny), ("Z", old.getZ(), nz)]:
        if abs(float(after) - float(before)) > max_delta:
            raise NeedleCalibrationError("%s offset delta %.6f exceeds max_apply_delta_mm %.6f" %
                (name, abs(float(after) - float(before)), max_delta))
    new = mm_loc(nx, ny, nz, old.getRotation())
    log_msg(logger, "Apply needle head offsets old=%s new=%s" %
        (loc_to_dict(old), loc_to_dict(new)))
    nozzle.setHeadOffsets(new)
    return {
        "old_offsets": loc_to_dict(old),
        "new_offsets": loc_to_dict(new),
        "delta": {"x": nx - old.getX(), "y": ny - old.getY(), "z": nz - old.getZ()}
    }


def fail_payload(stage, message, nozzle=None, cfg=None, logger=None, old_offsets=None):
    log_msg(logger, "Needle calibration failed: %s" % message)
    if nozzle is not None and old_offsets is not None:
        try:
            nozzle.setHeadOffsets(dict_to_loc(old_offsets))
        except:
            pass
    if nozzle is not None and cfg is not None:
        try:
            lift_nozzle_to_safe_z(nozzle, cfg, logger)
        except:
            pass
    details = {}
    try:
        details = getattr(message, "details")
    except:
        details = {}
    return {"ok": False, "stage": stage, "message": str(message),
        "old_offsets": old_offsets, "new_offsets": old_offsets, "details": details}


def run_z_only(cfg, nozzle=None, machine_obj=None, logger=None):
    stage = "z_touch"
    old_offsets = None
    try:
        machine_obj = get_machine(machine_obj)
        nozzle = find_nozzle(cfg, nozzle, machine_obj)
        actuator = find_actuator_by_name(machine_obj, cfg)
        old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
        old_offsets = loc_to_dict(old)

        capture = capture_fiducial_features(cfg, "z_touch", nozzle, machine_obj, logger)
        center = single_feature_location(capture, "z_touch")
        fid_loc = dict_to_loc(cfg.get("fiducials", {}).get("z_touch", {}).get("location"))
        surface_z = float(fid_loc.getZ())

        first = probe_z_touch(nozzle, actuator, cfg, center.getX(), center.getY(),
            surface_z, logger, "z_touch_first")
        proposed = proposed_offsets_from_delta(cfg, old, dz=first["z_error_mm"])
        applied = apply_xyz_to_openpnp_nozzle(cfg, nozzle, z=proposed.getZ(), logger=logger)

        verify = None
        if bool(cfg.get("z_probe", {}).get("verify_enabled", True)):
            verify = probe_z_touch(nozzle, actuator, cfg, center.getX(), center.getY(),
                surface_z, logger, "z_touch_verify", True)
            cfg["_last_z_probe_mm"] = verify["contact"]["z"]
            log_msg(logger, "Z verification result contact_z=%.6f error=%.6f" %
                (float(verify["contact"]["z"]), float(verify["z_error_mm"])))
        else:
            cfg["_last_z_probe_mm"] = first["contact"]["z"]
            log_msg(logger, "Z verification disabled by z_probe.verify_enabled")
        cfg["_last_z_touch_capture"] = capture
        if not isinstance(cfg.get("_runtime"), dict):
            cfg["_runtime"] = {}
        cfg["_runtime"]["z_touch"] = {
            "surface_z_mm": surface_z,
            "verified_contact_z_mm": float((verify or first)["contact"]["z"]),
            "offsets_after_z": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)),
            "timestamp_note": "runtime only"
        }

        new_offsets = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        log_msg(logger, "Z offsets before=%s after=%s verified_contact_z=%.6f" %
            (old_offsets, new_offsets, float((verify or first)["contact"]["z"])))
        return {
            "ok": True,
            "stage": stage,
            "message": "ok",
            "old_offsets": old_offsets,
            "new_offsets": new_offsets,
            "details": {
                "capture": capture,
                "first_touch": first,
                "verify_touch": verify,
                "applied": applied
            }
        }
    except Throwable as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)
    except Exception as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)


def is_finite(value):
    try:
        v = float(value)
        return not (math.isnan(v) or math.isinf(v))
    except:
        return False


def resolve_post_side_probe_z_mm(cfg, logger=None):
    post = cfg.get("post_xy", {})
    if bool(post.get("allow_configured_probe_z", False)) and post.get("configured_probe_z_mm", None) is not None:
        z = float(post.get("configured_probe_z_mm"))
        log_msg(logger, "Post XY using configured probe Z override %.6f" % z)
    else:
        loc = dict_to_loc(post.get("location", {}))
        z = float(loc.getZ()) + float(post.get("side_probe_z_offset_mm", 0.20))
    if not is_finite(z) or z < -1000.0 or z > 1000.0:
        raise NeedleCalibrationError("Resolved post side probe Z is not reasonable: %s" % z)
    try:
        offset = float(post.get("side_probe_z_offset_mm", 0.20))
        if offset < 0.05:
            log_msg(logger, "Warning: Side probe Z offset may be too low; needle may touch top/shoulder instead of side diameter.")
        if offset > 1.0:
            log_msg(logger, "Warning: Side probe Z offset may be above useful side contact height.")
    except:
        pass
    return z


def capture_post_xy(cfg, nozzle=None, machine_obj=None, logger=None):
    machine_obj = get_machine(machine_obj)
    nozzle = find_nozzle(cfg, nozzle, machine_obj)
    camera = find_top_camera(cfg, machine_obj)
    post = cfg.get("post_xy", {})
    target = dict_to_loc(post.get("location"))
    log_msg(logger, "Capture post_xy at %s" % loc_to_log(target))
    move_safe(camera, target)

    pipeline = load_pipeline_from_source(post.get("pipeline", {}))
    pipeline.setProperty("camera", camera)
    pipeline.setProperty("nozzle", nozzle)
    image = camera.settleAndCapture()
    pipeline.process()

    stage_name = post.get("result_stage", "results")
    items = flatten_model(get_result_model(pipeline, stage_name))
    if len(items) == 0:
        raise NeedleCalibrationError("Post pipeline stage '%s' returned no features" % stage_name)
    features = []
    for item in items:
        feat = parse_feature_item(item)
        features.append(add_machine_coordinates_to_feature(camera, nozzle, feat))
    feature = features[0]
    out = {
        "role": "post_xy",
        "camera": object_name(camera),
        "stage": stage_name,
        "camera_location_mm": loc_to_dict(get_movable_location(camera)),
        "features": features,
        "feature": feature,
        "vision_center": {"x": float(feature["x_mm"]), "y": float(feature["y_mm"])},
        "detected_post_diameter_mm": feature.get("diameter_mm", None)
    }
    if bool(cfg.get("debug", {}).get("save_last_capture", True)):
        out["image"] = image
    return out


def local_lift_release_post(nozzle, actuator, cfg, read_cfg, logger=None, label="post_release"):
    post = cfg.get("post_xy", {})
    step = abs(float(post.get("release_lift_mm", 0.30)))
    max_lift = abs(float(post.get("max_release_lift_mm", 1.5)))
    speed = float(cfg.get("probe_speed", 0.20))
    moved = 0.0
    while True:
        dwell_for_read(read_cfg)
        if not read_contact_debounced(actuator, read_cfg, logger=logger, label=label,
                mixed_result=True):
            return moved
        if moved >= max_lift - 1.0e-9:
            raise NeedleCalibrationError("%s: contact did not release after local lift" % label)
        inc = min(step, max_lift - moved)
        moved += inc
        cur = get_movable_location(nozzle)
        move_direct_speed(nozzle, mm_loc(cur.getX(), cur.getY(), cur.getZ() + inc,
            cur.getRotation()), speed)


def move_nozzle_xy_lifted(nozzle, x, y, z_probe, lift, speed):
    cur = get_movable_location(nozzle)
    lifted_z = float(z_probe) + abs(float(lift))
    if float(cur.getZ()) < lifted_z - 1.0e-9:
        move_direct_speed(nozzle, mm_loc(cur.getX(), cur.getY(), lifted_z,
            cur.getRotation()), speed)
    move_direct_speed(nozzle, mm_loc(x, y, lifted_z,
        get_movable_location(nozzle).getRotation()), speed)


def descend_to_post_probe_z(nozzle, x, y, z_probe, speed):
    move_direct_speed(nozzle, mm_loc(x, y, z_probe,
        get_movable_location(nozzle).getRotation()), speed)


def radial_move_until_contact(nozzle, actuator, read_cfg, start_x, start_y, ux, uy,
        max_distance, step_mm, z_probe, speed, logger=None, label="post_radial"):
    traveled = 0.0
    step = abs(float(step_mm))
    last = None
    while traveled < float(max_distance) - 1.0e-9:
        traveled = min(float(max_distance), traveled + step)
        x = float(start_x) - float(ux) * traveled
        y = float(start_y) - float(uy) * traveled
        move_direct_speed(nozzle, mm_loc(x, y, z_probe,
            get_movable_location(nozzle).getRotation()), speed)
        dwell_for_read(read_cfg)
        if read_contact_debounced(actuator, read_cfg, logger=logger, label=label):
            last = get_movable_location(nozzle)
            return last, traveled
    return None, traveled


def post_probe_error(message, diagnostics):
    err = NeedleCalibrationError(message)
    err.details = {"probe_diagnostics": diagnostics}
    return err


def probe_post_radial_edge(cfg, nozzle, actuator, center_x, center_y, angle_deg,
        post_radius_mm, z_probe_mm, logger=None, probe_profile="coarse_fine"):
    post = cfg.get("post_xy", {})
    read_cfg = post_read_cfg(cfg)
    speed = float(cfg.get("probe_speed", 0.20))
    fine_speed = float(cfg.get("fine_probe_speed", 0.08))
    lift = abs(float(post.get("local_lift_mm", 0.60)))
    clearance = abs(float(post.get("scan_clearance_mm", 2.0)))
    overtravel = abs(float(post.get("scan_overtravel_mm", 1.0)))
    max_travel = abs(float(post.get("max_radial_probe_travel_mm", 4.0)))
    coarse_step = abs(float(post.get("coarse_step_mm", 0.10)))
    fine_step = abs(float(post.get("fine_step_mm", 0.01)))
    fine_backoff = abs(float(post.get("fine_backoff_mm", 0.15)))
    profile = str(probe_profile or "coarse_fine").strip().lower()
    if profile not in ["coarse_fine", "single_touch"]:
        raise NeedleCalibrationError("Unsupported post probe profile: %s" % probe_profile)
    theta = math.radians(float(angle_deg))
    ux = math.cos(theta)
    uy = math.sin(theta)
    label = "post_edge_%s" % str(angle_deg).replace(".", "_")
    max_retries = int(post.get("max_outside_retries", 2))
    if max_retries < 0:
        max_retries = 0
    first_mult = float(post.get("outside_start_clearance_multiplier", 1.0))
    retry_mult = float(post.get("outside_retry_clearance_multiplier", 2.0))
    diagnostics = {
        "angle_deg": float(angle_deg),
        "probe_profile": profile,
        "post_center": {"x": float(center_x), "y": float(center_y)},
        "configured_post_radius_mm": float(post_radius_mm),
        "z_probe_mm": float(z_probe_mm),
        "outside_attempts": []
    }

    try:
        local_lift_release_post(nozzle, actuator, cfg, read_cfg, logger, label + "_pre_release")
    except Exception as e:
        raise post_probe_error(str(e), diagnostics)
    start_x = None
    start_y = None
    start_distance = None
    outside_retry_used = False
    outside_touching = True
    for attempt in range(max_retries + 1):
        if attempt == 0:
            multiplier = first_mult
        else:
            multiplier = retry_mult + float(attempt - 1)
        start_distance = float(post_radius_mm) + clearance * multiplier
        start_x = float(center_x) + start_distance * ux
        start_y = float(center_y) + start_distance * uy
        log_msg(logger, "%s: outside attempt=%d center=(%.6f, %.6f) radius=%.6f start_distance=%.6f start=(%.6f, %.6f) z_probe=%.6f" %
            (label, attempt + 1, float(center_x), float(center_y), float(post_radius_mm),
             start_distance, start_x, start_y, float(z_probe_mm)))
        move_nozzle_xy_lifted(nozzle, start_x, start_y, z_probe_mm, lift, speed)
        descend_to_post_probe_z(nozzle, start_x, start_y, z_probe_mm, speed)
        dwell_for_read(read_cfg)
        outside_touching = read_contact_debounced(actuator, read_cfg,
            logger=logger, label=label + "_outside_attempt_%d" % (attempt + 1))
        diagnostics["outside_attempts"].append({
            "attempt": attempt + 1,
            "clearance_multiplier": multiplier,
            "start_distance_mm": start_distance,
            "start_x": start_x,
            "start_y": start_y,
            "touching": bool(outside_touching)
        })
        log_msg(logger, "%s: outside attempt=%d touching=%s" %
            (label, attempt + 1, outside_touching))
        if not outside_touching:
            break
        outside_retry_used = True
        move_direct_speed(nozzle, mm_loc(start_x, start_y, float(z_probe_mm) + lift,
            get_movable_location(nozzle).getRotation()), speed)
        dwell_for_read(read_cfg)
        if read_contact_debounced(actuator, read_cfg, logger=logger,
                label=label + "_outside_lift_check", mixed_result=True):
            high_lift = abs(float(post.get("max_release_lift_mm", 3.0)))
            log_msg(logger, "%s: outside start still touching at local lift; raising to %.6f before moving outward" %
                (label, float(z_probe_mm) + high_lift))
            move_direct_speed(nozzle, mm_loc(start_x, start_y, float(z_probe_mm) + high_lift,
                get_movable_location(nozzle).getRotation()), speed)
    if outside_touching:
        if bool(post.get("touching_start_safe_z_recover", True)):
            lift_nozzle_to_safe_z(nozzle, cfg, logger)
        raise post_probe_error(
            "%s: Outside start still touching after %d retries; increase scan_clearance_mm or verify post center/diameter/probe Z." %
            (label, max_retries),
            diagnostics)

    coarse_limit = min(max_travel, start_distance - float(post_radius_mm) + overtravel)
    diagnostics["start_distance_mm"] = start_distance
    diagnostics["start"] = {"x": start_x, "y": start_y}
    diagnostics["coarse_limit_mm"] = coarse_limit
    if profile == "single_touch":
        step = abs(float(post.get("single_touch_step_mm", coarse_step)))
        if bool(post.get("single_touch_use_coarse_step", True)):
            step = coarse_step
        if step <= 0.0:
            step = coarse_step
        single_speed = float(post.get("single_touch_speed", speed))
        contact, contact_travel = radial_move_until_contact(nozzle, actuator, read_cfg,
            start_x, start_y, ux, uy, coarse_limit, step, z_probe_mm, single_speed,
            logger, label + "_single_touch")
        if contact is None:
            diagnostics["contact_travel_mm"] = float(contact_travel)
            raise post_probe_error("%s: no single-touch contact within %.6f mm" %
                (label, coarse_limit), diagnostics)
        move_direct_speed(nozzle, mm_loc(contact.getX(), contact.getY(), float(z_probe_mm) + lift,
            contact.getRotation()), speed)
        diagnostics["contact"] = loc_to_dict(contact)
        diagnostics["contact_travel_mm"] = float(contact_travel)
        try:
            release_extra = local_lift_release_post(nozzle, actuator, cfg, read_cfg, logger,
                label + "_single_touch_release")
        except Exception as e:
            raise post_probe_error(str(e), diagnostics)
        log_msg(logger, "%s: single_touch angle=%.6f attempts=%d travel=%.6f contact=(%.6f, %.6f) step=%.6f release_lift=%.6f" %
            (label, float(angle_deg), len(diagnostics["outside_attempts"]),
             float(contact_travel), float(contact.getX()), float(contact.getY()),
             float(step), float(release_extra)))
        return {
            "angle_deg": float(angle_deg),
            "x": float(contact.getX()),
            "y": float(contact.getY()),
            "z_probe_mm": float(z_probe_mm),
            "probe_profile": profile,
            "single_touch": True,
            "post_center": {"x": float(center_x), "y": float(center_y)},
            "configured_post_radius_mm": float(post_radius_mm),
            "start_distance_mm": float(start_distance),
            "start": {"x": start_x, "y": start_y},
            "unit": {"x": ux, "y": uy},
            "outside_retry_used": outside_retry_used,
            "outside_attempts": diagnostics["outside_attempts"],
            "outside_start_touching": False,
            "contact": loc_to_dict(contact),
            "fine_contact": loc_to_dict(contact),
            "contact_travel_mm": float(contact_travel),
            "step_mm": float(step),
            "max_radial_probe_travel_mm": max_travel,
            "release_extra_mm": release_extra
        }

    coarse, coarse_travel = radial_move_until_contact(nozzle, actuator, read_cfg,
        start_x, start_y, ux, uy, coarse_limit, coarse_step, z_probe_mm, speed,
        logger, label + "_coarse")
    if coarse is None:
        diagnostics["coarse_travel_mm"] = float(coarse_travel)
        raise post_probe_error("%s: no coarse contact within %.6f mm" % (label, coarse_limit),
            diagnostics)

    move_direct_speed(nozzle, mm_loc(coarse.getX(), coarse.getY(), float(z_probe_mm) + lift,
        coarse.getRotation()), speed)
    diagnostics["coarse_contact"] = loc_to_dict(coarse)
    diagnostics["coarse_travel_mm"] = float(coarse_travel)
    try:
        local_lift_release_post(nozzle, actuator, cfg, read_cfg, logger, label + "_coarse_release")
    except Exception as e:
        raise post_probe_error(str(e), diagnostics)
    fine_start_travel = max(0.0, float(coarse_travel) - fine_backoff)
    fine_start_x = float(start_x) - ux * fine_start_travel
    fine_start_y = float(start_y) - uy * fine_start_travel
    move_nozzle_xy_lifted(nozzle, fine_start_x, fine_start_y, z_probe_mm, lift, speed)
    descend_to_post_probe_z(nozzle, fine_start_x, fine_start_y, z_probe_mm, fine_speed)
    fine_limit = min(max_travel - fine_start_travel, fine_backoff + coarse_step + fine_step)
    fine, fine_travel = radial_move_until_contact(nozzle, actuator, read_cfg,
        fine_start_x, fine_start_y, ux, uy, fine_limit, fine_step, z_probe_mm,
        fine_speed, logger, label + "_fine")
    if fine is None:
        diagnostics["coarse_travel_mm"] = float(coarse_travel)
        diagnostics["fine_start"] = {"x": fine_start_x, "y": fine_start_y}
        diagnostics["fine_limit_mm"] = float(fine_limit)
        raise post_probe_error("%s: no fine contact within %.6f mm" % (label, fine_limit),
            diagnostics)
    move_direct_speed(nozzle, mm_loc(fine.getX(), fine.getY(), float(z_probe_mm) + lift,
        fine.getRotation()), speed)
    diagnostics["fine_contact"] = loc_to_dict(fine)
    diagnostics["fine_travel_mm"] = float(fine_start_travel + fine_travel)
    try:
        release_extra = local_lift_release_post(nozzle, actuator, cfg, read_cfg, logger,
            label + "_fine_release")
    except Exception as e:
        raise post_probe_error(str(e), diagnostics)
    log_msg(logger, "%s: summary coarse_travel=%.6f fine_travel=%.6f contact=(%.6f, %.6f) release_lift=%.6f" %
        (label, float(coarse_travel), float(fine_start_travel + fine_travel),
         float(fine.getX()), float(fine.getY()), float(release_extra)))

    return {
        "angle_deg": float(angle_deg),
        "x": float(fine.getX()),
        "y": float(fine.getY()),
        "z_probe_mm": float(z_probe_mm),
        "probe_profile": profile,
        "single_touch": False,
        "post_center": {"x": float(center_x), "y": float(center_y)},
        "configured_post_radius_mm": float(post_radius_mm),
        "start_distance_mm": float(start_distance),
        "start": {"x": start_x, "y": start_y},
        "unit": {"x": ux, "y": uy},
        "outside_retry_used": outside_retry_used,
        "outside_attempts": diagnostics["outside_attempts"],
        "outside_start_touching": False,
        "coarse_contact": loc_to_dict(coarse),
        "fine_contact": loc_to_dict(fine),
        "coarse_travel_mm": float(coarse_travel),
        "fine_travel_mm": float(fine_start_travel + fine_travel),
        "max_radial_probe_travel_mm": max_travel,
        "release_extra_mm": release_extra
    }


def parse_angles(value, fallback):
    if value is None:
        return list(fallback)
    if isinstance(value, basestring):
        text = str(value).strip()
        if text == "":
            return list(fallback)
        try:
            value = json.loads(text)
        except:
            return [float(part.strip()) for part in text.split(",") if part.strip()]
    return [float(item) for item in value]


def post_probe_profile_for_pass(cfg, pass_name):
    post = cfg.get("post_xy", {})
    if str(pass_name or "").strip() == "verify":
        return str(post.get("verify_probe_profile", "single_touch") or "single_touch").strip()
    return str(post.get("calibration_probe_profile", "coarse_fine") or "coarse_fine").strip()


def measure_post_perimeter(cfg, nozzle, actuator, post_center, post_radius_mm,
        z_contact_mm, angles=None, probe_profile="coarse_fine", pass_name="measurement",
        logger=None):
    angles = parse_angles(angles, POST_ANGLES_8)
    contacts = []
    try:
        for angle in angles:
            contact = probe_post_radial_edge(cfg, nozzle, actuator,
                post_center["x"], post_center["y"], angle, post_radius_mm,
                z_contact_mm, logger, probe_profile)
            contact["pass_name"] = pass_name
            contacts.append(contact)
        lift_nozzle_to_safe_z(nozzle, cfg, logger)
        return contacts
    except:
        try:
            lift_nozzle_to_safe_z(nozzle, cfg, logger)
        except:
            pass
        raise


def solve_3x3(matrix, vector):
    a = [[float(matrix[r][c]) for c in range(3)] + [float(vector[r])] for r in range(3)]
    for col in range(3):
        pivot = col
        for row in range(col + 1, 3):
            if abs(a[row][col]) > abs(a[pivot][col]):
                pivot = row
        if abs(a[pivot][col]) < 1.0e-12:
            raise NeedleCalibrationError("Circle fit normal equations are singular")
        if pivot != col:
            tmp = a[col]
            a[col] = a[pivot]
            a[pivot] = tmp
        div = a[col][col]
        for c in range(col, 4):
            a[col][c] = a[col][c] / div
        for row in range(3):
            if row == col:
                continue
            factor = a[row][col]
            for c in range(col, 4):
                a[row][c] = a[row][c] - factor * a[col][c]
    return [a[0][3], a[1][3], a[2][3]]


def circle_fit_contacts(contacts):
    if len(contacts) < 3:
        raise NeedleCalibrationError("Circle fit requires at least three contacts")
    ata = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    atb = [0.0, 0.0, 0.0]
    for p in contacts:
        x = float(p["x"])
        y = float(p["y"])
        row = [x, y, 1.0]
        b = -(x * x + y * y)
        for r in range(3):
            atb[r] += row[r] * b
            for c in range(3):
                ata[r][c] += row[r] * row[c]
    a, b, c = solve_3x3(ata, atb)
    cx = -a / 2.0
    cy = -b / 2.0
    rr = (a * a + b * b) / 4.0 - c
    if rr <= 0.0:
        raise NeedleCalibrationError("Circle fit produced non-positive radius")
    radius = math.sqrt(rr)
    residuals = []
    sum_sq = 0.0
    max_err = 0.0
    for p in contacts:
        d = math.sqrt(math.pow(float(p["x"]) - cx, 2.0) + math.pow(float(p["y"]) - cy, 2.0))
        err = d - radius
        residuals.append(err)
        sum_sq += err * err
        max_err = max(max_err, abs(err))
    rms = math.sqrt(sum_sq / float(len(contacts))) if len(contacts) else 0.0
    return {
        "center": {"x": cx, "y": cy},
        "radius_mm": radius,
        "rms_error_mm": rms,
        "max_error_mm": max_err,
        "residuals_mm": residuals,
        "point_count": len(contacts)
    }


def solve_post_xy_from_contacts(cfg, vision_center, contacts, post_radius_mm, logger=None):
    raw_fit = circle_fit_contacts(contacts)
    threshold = max(0.05, 3.0 * float(raw_fit.get("rms_error_mm", 0.0)))
    filtered = []
    rejected = []
    for i, contact in enumerate(contacts):
        if abs(float(raw_fit["residuals_mm"][i])) <= threshold:
            filtered.append(contact)
        else:
            rejected.append(contact)
    filtered_fit = None
    if len(filtered) >= 5 and len(filtered) < len(contacts):
        filtered_fit = circle_fit_contacts(filtered)
        log_msg(logger, "Post XY outlier rejection kept %d/%d contacts" %
            (len(filtered), len(contacts)))
    else:
        filtered = list(contacts)
        filtered_fit = raw_fit
        if len(contacts) >= 6:
            best = None
            best_contacts = None
            for drop_index in range(len(contacts)):
                candidate = []
                for i, contact in enumerate(contacts):
                    if i != drop_index:
                        candidate.append(contact)
                if len(candidate) < 5:
                    continue
                try:
                    candidate_fit = circle_fit_contacts(candidate)
                except:
                    continue
                if best is None or float(candidate_fit["rms_error_mm"]) < float(best["rms_error_mm"]):
                    best = candidate_fit
                    best_contacts = candidate
            if best is not None and float(best["rms_error_mm"]) < float(raw_fit["rms_error_mm"]) * 0.50:
                filtered = best_contacts
                filtered_fit = best
                rejected = []
                for contact in contacts:
                    found = False
                    for kept in filtered:
                        if kept is contact:
                            found = True
                            break
                    if not found:
                        rejected.append(contact)
                log_msg(logger, "Post XY leave-one-out outlier rejection kept %d/%d contacts" %
                    (len(filtered), len(contacts)))
    measured = filtered_fit["center"]
    dx = float(measured["x"]) - float(vision_center["x"])
    dy = float(measured["y"]) - float(vision_center["y"])
    return {
        "raw_contacts": contacts,
        "raw_circle_fit": raw_fit,
        "filtered_contacts": filtered,
        "filtered_circle_fit": filtered_fit,
        "rejected_contacts": rejected,
        "outlier_threshold_mm": threshold,
        "measured_center_delta": {"x": dx, "y": dy}
    }


def estimate_effective_contact_diameter_from_contacts(cfg, contacts, post_radius_mm):
    fit = circle_fit_contacts(contacts)
    radius = float(fit["radius_mm"]) - float(post_radius_mm)
    diameter = 2.0 * radius
    return {"radius_mm": radius, "diameter_mm": diameter}


def diameter_warnings(cfg, diameter):
    post = cfg.get("post_xy", {})
    warnings = []
    mn = float(post.get("min_effective_contact_diameter_mm", 0.0))
    mx = float(post.get("max_effective_contact_diameter_mm", 1.5))
    if float(diameter) < mn:
        warnings.append("Effective electrical contact diameter %.6f mm is below minimum %.6f mm" %
            (float(diameter), mn))
    if float(diameter) > mx:
        warnings.append("Effective electrical contact diameter %.6f mm is above maximum %.6f mm" %
            (float(diameter), mx))
    return warnings


def make_xy_proposal(cfg, nozzle, dx, dy):
    old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
    sign = float(cfg.get("xy_offset_sign", 1.0))
    delta_x = -sign * float(dx)
    delta_y = -sign * float(dy)
    proposed = mm_loc(float(old.getX()) + delta_x, float(old.getY()) + delta_y,
        float(old.getZ()), float(old.getRotation()))
    return {
        "dx_mm": float(dx),
        "dy_mm": float(dy),
        "offset_delta": {"x": delta_x, "y": delta_y, "z": 0.0},
        "proposed_offsets": loc_to_dict(proposed)
    }


def apply_proposed_xy_offsets(cfg, nozzle, proposal, logger=None):
    offsets = proposal.get("proposed_offsets", proposal)
    return apply_xyz_to_openpnp_nozzle(cfg, nozzle,
        x=float(offsets["x"]), y=float(offsets["y"]), logger=logger)


def run_post_xy_measurement_pass(cfg, nozzle, machine_obj, actuator, angles,
        pass_name, probe_profile, logger=None, capture=None):
    if capture is None:
        capture = capture_post_xy(cfg, nozzle, machine_obj, logger)
    vision_center = capture.get("vision_center")
    post = cfg.get("post_xy", {})
    diameter = float(post.get("post_diameter_mm", 6.35))
    radius = diameter / 2.0
    z_probe = resolve_post_side_probe_z_mm(cfg, logger)
    angles = parse_angles(angles, POST_ANGLES_8)
    log_msg(logger, "Post XY %s pass angles=%s profile=%s" %
        (str(pass_name), angles, str(probe_profile)))
    contacts = measure_post_perimeter(cfg, nozzle, actuator, vision_center,
        radius, z_probe, angles, probe_profile, pass_name, logger)
    solved = solve_post_xy_from_contacts(cfg, vision_center, contacts, radius, logger)
    effective = estimate_effective_contact_diameter_from_contacts(
        cfg, solved["filtered_contacts"], radius)
    warnings = diameter_warnings(cfg, effective["diameter_mm"])
    return capture, solved, effective, warnings, z_probe, radius, pass_name, probe_profile


def run_post_xy_measurement(cfg, nozzle, machine_obj, actuator, angles, logger=None):
    capture, solved, effective, warnings, z_probe, radius, pass_name, profile = \
        run_post_xy_measurement_pass(cfg, nozzle, machine_obj, actuator, angles,
            "measurement", "coarse_fine", logger)
    return capture, solved, effective, warnings, z_probe, radius


def fit_center(solved):
    return solved.get("filtered_circle_fit", solved.get("circle_fit", {})).get("center", {})


def fit_radius(solved):
    return float(solved.get("filtered_circle_fit", solved.get("circle_fit", {})).get("radius_mm", 0.0))


def compare_post_fits(cardinal_solved, verify_solved, post_radius_mm, cfg):
    post = cfg.get("post_xy", {})
    cfit = cardinal_solved.get("filtered_circle_fit", {})
    vfit = verify_solved.get("filtered_circle_fit", {})
    cc = cfit.get("center", {})
    vc = vfit.get("center", {})
    dx = float(vc.get("x", 0.0)) - float(cc.get("x", 0.0))
    dy = float(vc.get("y", 0.0)) - float(cc.get("y", 0.0))
    dr = math.sqrt(dx * dx + dy * dy)
    verify_rms = float(vfit.get("rms_error_mm", 0.0))
    verify_max = float(vfit.get("max_error_mm", 0.0))
    radius_delta = float(vfit.get("radius_mm", 0.0)) - float(cfit.get("radius_mm", 0.0))
    effective_delta = 2.0 * radius_delta
    max_center = float(post.get("max_verify_center_delta_mm", 0.10))
    max_rms = float(post.get("max_verify_rms_mm", 0.08))
    max_radius = float(post.get("max_verify_contact_radius_error_mm", 0.15))
    warnings = []
    if dr > max_center:
        warnings.append("Fast verify center delta %.6f mm exceeds %.6f mm" % (dr, max_center))
    if verify_rms > max_rms:
        warnings.append("Fast verify RMS %.6f mm exceeds %.6f mm" % (verify_rms, max_rms))
    if abs(radius_delta) > max_radius:
        warnings.append("Fast verify radius delta %.6f mm exceeds %.6f mm" %
            (abs(radius_delta), max_radius))
    return {
        "center_delta": {"x": dx, "y": dy, "r": dr},
        "verify_rms_mm": verify_rms,
        "verify_max_error_mm": verify_max,
        "fitted_radius_delta_mm": radius_delta,
        "effective_contact_diameter_delta_mm": effective_delta,
        "ok": len(warnings) == 0,
        "warnings": warnings
    }


def post_pass_result(angles, probe_profile, solved, effective, warnings):
    return {
        "angles_deg": angles,
        "probe_profile": probe_profile,
        "contacts": solved["raw_contacts"],
        "raw_contacts": solved["raw_contacts"],
        "raw_circle_fit": solved["raw_circle_fit"],
        "filtered_contacts": solved["filtered_contacts"],
        "rejected_contacts": solved.get("rejected_contacts", []),
        "filtered_circle_fit": solved["filtered_circle_fit"],
        "circle_fit": solved["filtered_circle_fit"],
        "effective_electrical_contact": effective,
        "diameter_warnings": warnings,
        "measured_center_delta": solved["measured_center_delta"]
    }


def execute_post_xy_measurement(cfg, nozzle, machine_obj, actuator, logger=None):
    post = cfg.get("post_xy", {})
    pass_details = {}
    cal_angles = parse_angles(post.get("calibration_angles_deg"), [0, 90, 180, 270])
    cal_profile = post_probe_profile_for_pass(cfg, "calibration")
    capture, cal_solved, cal_effective, cal_warnings, z_probe, radius, pass_name, profile = \
        run_post_xy_measurement_pass(cfg, nozzle, machine_obj, actuator,
            cal_angles, "calibration", cal_profile, logger)
    pass_details["calibration_pass"] = post_pass_result(
        cal_angles, cal_profile, cal_solved, cal_effective, cal_warnings)
    primary_solved = cal_solved
    primary_effective = cal_effective
    primary_warnings = list(cal_warnings)
    primary_angles = cal_angles
    proposal_source = "calibration_cardinal"
    pass_details["primary_solve"] = "calibration_pass"
    if bool(post.get("run_fast_verify_after_cardinal", True)):
        verify_angles = parse_angles(post.get("verify_angles_deg"), POST_ANGLES_8)
        verify_profile = post_probe_profile_for_pass(cfg, "verify")
        vcap, verify_solved, verify_effective, verify_warnings, vz, vr, vpass, vprofile = \
            run_post_xy_measurement_pass(cfg, nozzle, machine_obj, actuator,
                verify_angles, "fast verify", verify_profile, logger, capture)
        comparison = compare_post_fits(cal_solved, verify_solved, radius, cfg)
        fast_verify_warnings = list(verify_warnings)
        fast_verify_warnings.extend(comparison.get("warnings", []))
        fast_ok = bool(comparison.get("ok", False)) and len(verify_warnings) == 0
        fast_verify = post_pass_result(
            verify_angles, verify_profile, verify_solved, verify_effective,
            fast_verify_warnings)
        fast_verify["enabled"] = True
        fast_verify["comparison_to_calibration"] = comparison
        fast_verify["ok"] = fast_ok
        fast_verify["warnings"] = fast_verify_warnings
        pass_details["fast_verify"] = fast_verify
        if not fast_ok:
            primary_warnings.extend(fast_verify_warnings)
            if bool(post.get("fail_on_fast_verify_failure", False)):
                raise NeedleCalibrationError("Fast verify failed: %s" %
                    "; ".join(fast_verify_warnings))
        if bool(post.get("use_verify_fit_for_proposal", False)):
            primary_solved = verify_solved
            primary_effective = verify_effective
            primary_warnings = fast_verify_warnings
            primary_angles = verify_angles
            proposal_source = "fast_verify"
            pass_details["primary_solve"] = "fast_verify"
    else:
        pass_details["fast_verify"] = {"enabled": False}
    return capture, primary_solved, primary_effective, primary_warnings, z_probe, radius, \
        primary_angles, pass_details, proposal_source


def finish_post_xy_result(cfg, nozzle, old_offsets, capture, solved, effective,
        warnings, z_probe, radius, angles, pass_details=None,
        proposal_source="primary", logger=None):
    post = cfg.get("post_xy", {})
    measured_delta = solved["measured_center_delta"]
    proposal = make_xy_proposal(cfg, nozzle, measured_delta["x"], measured_delta["y"])
    applied = False
    apply_result = None
    if bool(post.get("apply_xy_offsets", False)):
        apply_result = apply_proposed_xy_offsets(cfg, nozzle, proposal, logger)
        applied = True
    new_offsets = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
    post_loc = dict_to_loc(post.get("location", {}))
    verification = {
        "mode": "predicted_only" if not applied else "requires_verify_post_xy_after_apply",
        "predicted_residual_mm": float(solved["filtered_circle_fit"].get("rms_error_mm", 0.0))
    }
    result = {
        "ok": True,
        "stage": "post_xy",
        "message": "ok",
        "old_offsets": old_offsets,
        "new_offsets": new_offsets,
        "applied": applied,
        "post_side_probe_z_mm": float(z_probe),
        "post_location_z_mm": float(post_loc.getZ()),
        "side_probe_z_offset_mm": float(post.get("side_probe_z_offset_mm", 0.20)),
        "configured_post_diameter_mm": float(post.get("post_diameter_mm", 6.35)),
        "configured_post_radius_mm": float(radius),
        "detected_post_diameter_mm": capture.get("detected_post_diameter_mm", None),
        "proposal_source": proposal_source,
        "angles_deg": angles,
        "proposal": proposal,
        "details": {
            "camera_capture": capture,
            "vision_center": capture.get("vision_center"),
            "post_location_z_mm": float(post_loc.getZ()),
            "side_probe_z_offset_mm": float(post.get("side_probe_z_offset_mm", 0.20)),
            "post_side_probe_z_mm": float(z_probe),
            "configured_post_diameter_mm": float(post.get("post_diameter_mm", 6.35)),
            "configured_post_radius_mm": float(radius),
            "detected_post_diameter_mm": capture.get("detected_post_diameter_mm", None),
            "proposal_source": proposal_source,
            "angles_deg": angles,
            "calibration_pass": (pass_details or {}).get("calibration_pass"),
            "fast_verify": (pass_details or {}).get("fast_verify"),
            "primary_solve": (pass_details or {}).get("primary_solve", proposal_source),
            "contacts": solved["raw_contacts"],
            "raw_contacts": solved["raw_contacts"],
            "raw_circle_fit": solved["raw_circle_fit"],
            "filtered_contacts": solved["filtered_contacts"],
            "rejected_contacts": solved.get("rejected_contacts", []),
            "filtered_circle_fit": solved["filtered_circle_fit"],
            "circle_fit": solved["filtered_circle_fit"],
            "effective_electrical_contact": effective,
            "estimated_needle": effective,
            "diameter_warnings": warnings,
            "measured_center_delta": measured_delta,
            "verification": verification,
            "applied": apply_result
        }
    }
    cfg["_last_post_xy_proposal"] = proposal
    cfg["_last_post_xy_result"] = result
    return result


def run_post_xy_only(cfg, nozzle=None, machine_obj=None, logger=None):
    stage = "post_xy"
    old_offsets = None
    try:
        machine_obj = get_machine(machine_obj)
        nozzle = find_nozzle(cfg, nozzle, machine_obj)
        actuator = find_actuator_by_name(machine_obj, cfg)
        old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
        old_offsets = loc_to_dict(old)
        capture, solved, effective, warnings, z_probe, radius, angles, \
            pass_details, proposal_source = execute_post_xy_measurement(
                cfg, nozzle, machine_obj, actuator, logger)
        return finish_post_xy_result(cfg, nozzle, old_offsets, capture, solved, effective,
            warnings, z_probe, radius, angles, pass_details,
            proposal_source, logger)
    except Throwable as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)
    except Exception as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)


def run_post_xy_debug_single_angle(cfg, angle_deg, nozzle=None, machine_obj=None,
        logger=None):
    stage = "post_xy_single_angle"
    old_offsets = None
    try:
        machine_obj = get_machine(machine_obj)
        nozzle = find_nozzle(cfg, nozzle, machine_obj)
        actuator = find_actuator_by_name(machine_obj, cfg)
        old_offsets = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        capture = capture_post_xy(cfg, nozzle, machine_obj, logger)
        post = cfg.get("post_xy", {})
        radius = float(post.get("post_diameter_mm", 6.35)) / 2.0
        z_probe = resolve_post_side_probe_z_mm(cfg, logger)
        contact = probe_post_radial_edge(cfg, nozzle, actuator,
            capture.get("vision_center")["x"], capture.get("vision_center")["y"],
            float(angle_deg), radius, z_probe, logger)
        lift_nozzle_to_safe_z(nozzle, cfg, logger)
        return {
            "ok": True,
            "stage": stage,
            "message": "ok",
            "old_offsets": old_offsets,
            "new_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)),
            "details": {
                "camera_capture": capture,
                "angle_deg": float(angle_deg),
                "post_side_probe_z_mm": z_probe,
                "configured_post_radius_mm": radius,
                "contact": contact,
                "raw_contacts": [contact],
                "diameter_warnings": diameter_warnings(cfg, 0.0)
            }
        }
    except Throwable as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)
    except Exception as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)


def verify_post_xy_after_apply(cfg, nozzle=None, machine_obj=None, logger=None):
    stage = "post_xy_verify"
    old_offsets = None
    try:
        machine_obj = get_machine(machine_obj)
        nozzle = find_nozzle(cfg, nozzle, machine_obj)
        actuator = find_actuator_by_name(machine_obj, cfg)
        old_offsets = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        post = cfg.get("post_xy", {})
        calibration_angles = parse_angles(post.get("calibration_angles_deg"), [0, 90, 180, 270])
        calibration_profile = post_probe_profile_for_pass(cfg, "calibration")
        capture, calibration_solved, calibration_effective, calibration_warnings, z_probe, radius, \
            calibration_pass_name, calibration_profile_used = \
            run_post_xy_measurement_pass(cfg, nozzle, machine_obj, actuator,
                calibration_angles, "calibration", calibration_profile, logger)

        angles = parse_angles(post.get("verify_angles_deg"), POST_ANGLES_8)
        probe_profile = post_probe_profile_for_pass(cfg, "verify")
        verify_capture, solved, effective, warnings, verify_z_probe, verify_radius, pass_name, profile = \
            run_post_xy_measurement_pass(cfg, nozzle, machine_obj, actuator,
                angles, "verify", probe_profile, logger, capture)
        comparison = compare_post_fits(calibration_solved, solved, radius, cfg)
        verify_warnings = list(warnings)
        verify_warnings.extend(comparison.get("warnings", []))
        residual = float(solved["filtered_circle_fit"].get("rms_error_mm", 0.0))
        max_residual = float(post.get("max_verify_residual_mm", 0.05))
        ok = residual <= max_residual and bool(comparison.get("ok", False)) and len(warnings) == 0
        result = {
            "ok": ok,
            "stage": stage,
            "message": "ok" if ok else "verification failed",
            "old_offsets": old_offsets,
            "new_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)),
            "angles_deg": angles,
            "probe_profile": profile,
            "pass_name": pass_name,
            "calibration_angles_deg": calibration_angles,
            "calibration_probe_profile": calibration_profile_used,
            "details": {
                "camera_capture": verify_capture,
                "post_side_probe_z_mm": verify_z_probe,
                "configured_post_radius_mm": verify_radius,
                "angles_deg": angles,
                "probe_profile": profile,
                "pass_name": pass_name,
                "calibration_pass": post_pass_result(
                    calibration_angles, calibration_profile_used, calibration_solved,
                    calibration_effective, calibration_warnings),
                "comparison_to_calibration": comparison,
                "raw_contacts": solved["raw_contacts"],
                "raw_circle_fit": solved["raw_circle_fit"],
                "filtered_contacts": solved["filtered_contacts"],
                "rejected_contacts": solved.get("rejected_contacts", []),
                "filtered_circle_fit": solved["filtered_circle_fit"],
                "effective_electrical_contact": effective,
                "diameter_warnings": verify_warnings,
                "max_verify_residual_mm": max_residual
            }
        }
        if residual > max_residual:
            result["message"] = "verification residual %.6f exceeds %.6f" % (
                residual, max_residual)
        elif len(verify_warnings) > 0:
            result["message"] = "; ".join(verify_warnings)
        cfg["_last_post_xy_verify"] = result
        return result
    except Throwable as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)
    except Exception as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)


def run_full_calibration(cfg, nozzle=None, machine_obj=None, logger=None):
    old_offsets = None
    z = None
    post = None
    try:
        machine_obj = get_machine(machine_obj)
        nozzle = find_nozzle(cfg, nozzle, machine_obj)
        old_offsets = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        z = run_z_only(cfg, nozzle, machine_obj, logger)
        if not z.get("ok", False):
            raise NeedleCalibrationError("Z touch failed: %s" % z.get("message"))
        post = run_post_xy_only(cfg, nozzle, machine_obj, logger)
        if not post.get("ok", False):
            raise NeedleCalibrationError("Post XY failed: %s" % post.get("message"))
        final_offsets = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        return {
            "ok": True,
            "stage": "full_calibration",
            "message": "ok",
            "z_touch": z,
            "post_xy": post,
            "old_offsets": old_offsets,
            "new_offsets": final_offsets,
            "final_offsets": final_offsets
        }
    except Throwable as e:
        log_msg(logger, "Needle calibration failed: %s" % e)
        if nozzle is not None:
            try:
                lift_nozzle_to_safe_z(nozzle, cfg, logger)
            except:
                pass
        final = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)) if nozzle is not None else old_offsets
        return {"ok": False, "stage": "full_calibration", "message": str(e),
            "z_touch": z, "post_xy": post, "old_offsets": old_offsets,
            "new_offsets": final, "final_offsets": final, "details": {}}
    except Exception as e:
        log_msg(logger, "Needle calibration failed: %s" % e)
        if nozzle is not None:
            try:
                lift_nozzle_to_safe_z(nozzle, cfg, logger)
            except:
                pass
        final = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)) if nozzle is not None else old_offsets
        return {"ok": False, "stage": "full_calibration", "message": str(e),
            "z_touch": z, "post_xy": post, "old_offsets": old_offsets,
            "new_offsets": final, "final_offsets": final, "details": {}}


if __name__ == "__main__":
    cfg = load_config()
    log_msg(None, "Loaded needle calibration config: %s" % cfg.get("_config_path"))
