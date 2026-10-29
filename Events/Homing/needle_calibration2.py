# needle_calibration2.py
#
# Simplified dispensing needle calibration for OpenPnP Jython 2.7.
# Produces offset proposals only. Applying offsets is explicit.

import json
import math
import os
import re

from java.lang import Boolean, Class, Object, System, Thread, Throwable
from java.lang.reflect import Array
from org.openpnp.model import AxesLocation, Configuration, Length, LengthUnit, Location
from org.openpnp.spi import Axis
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


class NeedleCalibrationError(Exception):
    pass


class NeedleCalibrationSafetyError(NeedleCalibrationError):
    """A machine state could not be restored or bounded safely."""


DEFAULT_CONFIG = {
    "schema_version": 6,
    "head_name": "",
    "nozzle_name": "Dispense Head - Needle",
    "top_camera_name": "",
    "contact_actuator_name": "NeedleContactProbe_Read",
    "z_actuator_name": "Voron Z Probe",
    "safe_z_mm": 25.0,
    "travel_speed": 1.0,
    "probe_speed": 0.8,
    "fine_probe_speed": 0.05,
    "xy_offset_sign": 1.0,
    "z_offset_sign": 1.0,
    "max_apply_delta_mm": 5.0,
    "z_probe": {
        "acquisition_method": "hardware_interrupt",
        "mode": "actuator",
        "sequence_actuator_name": "Run Voron Probe",
        "channel": "Q0",
        "probe_acceleration": 1000.0,
        "coarse_feed": 3000.0,
        "fine_feed": 500.0,
        "fine_repeats": 1,
        "polling_coarse_repeats": 1,
        "polling_fine_repeats": 1,
        "start_above_mm": 4.0,
        "max_descent_mm": 8.0,
        "coarse_step_mm": 0.10,
        "fine_step_mm": 0.02,
        "retract_after_contact_mm": 0.40,
        "release_retract_step_mm": 0.20,
        "max_release_retract_mm": 2.0,
        "read_settle_ms": 80,
        "debounce_count": 3,
        "debounce_retries": 3,
        "debounce_interval_ms": 20
    },
    "fiducials": {
        "z_touch": {
            "location": {"x": 0.0, "y": 0.0, "z": 0.0, "rotation": 0.0},
            "trigger_offset_mm": 0.0,
            "result_stage": "results",
            "pipeline": {"mode": "named", "name": "", "path": ""}
        }
    },
    "post_xy": {
        "acquisition_method": "polling",
        "actuator_name": "NeedleContactProbe_Read",
        "location": {"x": 0.0, "y": 0.0, "z": 0.0, "rotation": 0.0},
        "result_stage": "results",
        "pipeline": {"mode": "named", "name": "", "path": ""},
        "post_diameter_mm": 6.35,
        "angles_deg": [0, 45, 90, 135, 180, 225, 270, 315],
        "scan_clearance_mm": 2.0,
        "scan_overtravel_mm": 3.0,
        "max_radial_probe_travel_mm": 5.0,
        "step_mm": 0.05,
        "speed": 0.8,
        "hardware_channel": "Q1",
        "hardware_probe_acceleration": 1000.0,
        "hardware_coarse_feed": 3000.0,
        "hardware_fine_feed": 500.0,
        "hardware_coarse_repeats": 1,
        "hardware_fine_repeats": 2,
        "hardware_fine_retry_attempts": 2,
        "hardware_g38_attempts": 2,
        "hardware_start_recovery_attempts": 1,
        "hardware_start_radius_mm": 4.175,
        "hardware_coarse_max_travel_mm": 7.0,
        "hardware_min_coarse_travel_mm": 0.25,
        "hardware_fine_retract_mm": 1.0,
        "hardware_fine_max_travel_mm": 2.0,
        "hardware_repeat_tolerance_mm": 0.15,
        "confirm_backoff_mm": 0.4,
        "confirm_reapproach_mm": 0.8,
        "confirm_step_mm": None,
        "confirm_position_tolerance_mm": 0.15,
        "confirm_reject_early": True,
        "read_settle_ms": 10,
        "debounce_count": 3,
        "debounce_retries": 2,
        "debounce_interval_ms": 5,
        "allow_single_read_contact": False,
        "min_valid_contacts_for_fit": 5,
        "side_probe_z_offset_mm": 0.25,
        "local_lift_mm": 0.60,
        "outside_recover_step_mm": 0.25,
        "outside_recover_max_mm": 3.0,
        "min_angular_coverage_deg": 180.0,
        "max_angular_gap_deg": 180.0,
        "max_geometry_condition": 10.0,
        "max_contact_line_error_mm": 0.20,
        "max_pair_projection_error_mm": 0.20,
        "max_center_disagreement_mm": 0.20,
        "min_fitted_radius_tolerance_mm": 0.05,
        "max_fit_rms_mm": 0.15,
        "max_fit_error_mm": 0.30,
        "max_center_error_mm": 2.0,
        "log_every_step": False,
        "log_step_interval": 10
    },
    "debug": {
        "verbose": False,
        "save_last_capture": True
    }
}


def log_msg(logger, msg):
    if logger:
        logger(str(msg))
    else:
        print(str(msg))


def log_section(logger, title):
    log_msg(logger, "")
    log_msg(logger, "== %s ==" % str(title))


def log_kv(logger, label, data):
    log_msg(logger, "%s: %s" % (str(label), data))


def fmt_float(v):
    try:
        return "%.6f" % float(v)
    except:
        return str(v)


def fmt_xy(x, y):
    return "(%s,%s)" % (fmt_float(x), fmt_float(y))


def fmt_loc(loc):
    try:
        if hasattr(loc, "convertToUnits"):
            d = loc_to_dict(loc)
        else:
            d = loc
        return "(%s,%s,%s,%s)" % (
            fmt_float(d.get("x", 0.0)), fmt_float(d.get("y", 0.0)),
            fmt_float(d.get("z", 0.0)), fmt_float(d.get("rotation", 0.0)))
    except:
        return str(loc)


def deep_copy(value):
    return json.loads(json.dumps(value, sort_keys=True))


def deep_merge(defaults, loaded):
    out = deep_copy(defaults)
    if not isinstance(loaded, dict):
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


def normalize_loaded_config(loaded):
    cfg = deep_merge(DEFAULT_CONFIG, loaded)
    legacy_actuator = str(cfg.get("contact_actuator_name", "") or "").strip()
    if legacy_actuator:
        if not isinstance(loaded, dict) or "z_actuator_name" not in loaded:
            cfg["z_actuator_name"] = legacy_actuator
        try:
            post_cfg = cfg.get("post_xy", {})
            post_loaded_cfg = loaded.get("post_xy", {}) if isinstance(loaded, dict) else {}
            if "actuator_name" not in post_loaded_cfg:
                post_cfg["actuator_name"] = legacy_actuator
        except:
            pass
    post_loaded = loaded.get("post_xy", {}) if isinstance(loaded, dict) else {}
    post = cfg.get("post_xy", {})
    # Schema 5 gives hardware probing literal, independent distances. Older
    # configs derived these values from the polling scan/confirmation fields.
    if "hardware_start_radius_mm" not in post_loaded:
        post["hardware_start_radius_mm"] = (
            abs(float(post.get("post_diameter_mm", 6.35))) / 2.0 +
            abs(float(post.get("scan_clearance_mm", 2.0))))
    if "hardware_coarse_max_travel_mm" not in post_loaded:
        post["hardware_coarse_max_travel_mm"] = min(
            abs(float(post.get("max_radial_probe_travel_mm", 5.0))),
            abs(float(post.get("scan_clearance_mm", 2.0))) +
            abs(float(post.get("scan_overtravel_mm", 3.0))))
    if "hardware_fine_retract_mm" not in post_loaded:
        post["hardware_fine_retract_mm"] = abs(float(
            post.get("confirm_backoff_mm", 0.4)))
    if "hardware_fine_max_travel_mm" not in post_loaded:
        post["hardware_fine_max_travel_mm"] = abs(float(
            post.get("confirm_reapproach_mm", 0.8)))
    if "hardware_repeat_tolerance_mm" not in post_loaded:
        post["hardware_repeat_tolerance_mm"] = abs(float(
            post.get("confirm_position_tolerance_mm", 0.15)))
    fid_loaded = loaded.get("fiducials", {}) if isinstance(loaded, dict) else {}
    z_touch_loaded = fid_loaded.get("z_touch", {}) if isinstance(fid_loaded, dict) else {}
    z_probe_loaded = loaded.get("z_probe", {}) if isinstance(loaded, dict) else {}
    if "acquisition_method" not in z_probe_loaded:
        # Schema 3's actuator/direct_gcode modes were both interrupt modes.
        cfg["z_probe"]["acquisition_method"] = "hardware_interrupt"
    if "channel" not in z_probe_loaded:
        cfg["z_probe"]["channel"] = "Q0"
    if "trigger_offset_mm" not in z_touch_loaded and "trigger_offset_mm" in z_probe_loaded:
        cfg["fiducials"]["z_touch"]["trigger_offset_mm"] = z_probe_loaded.get("trigger_offset_mm")
    # Compatibility with older two-pass post XY configs.
    if "angles_deg" not in post_loaded and "final_angles_deg" in post_loaded:
        post["angles_deg"] = post_loaded.get("final_angles_deg")
    if "angles_deg" not in post_loaded and "verify_angles_deg" in post_loaded:
        post["angles_deg"] = post_loaded.get("verify_angles_deg")
    if "angles_deg" not in post_loaded and "rough_angles_deg" in post_loaded:
        # Last-resort fallback only. Preferred default is the 8-point set.
        try:
            if len(post_loaded.get("rough_angles_deg")) >= 8:
                post["angles_deg"] = post_loaded.get("rough_angles_deg")
        except:
            pass
    if "max_fit_rms_mm" not in post_loaded and "max_final_rms_mm" in post_loaded:
        post["max_fit_rms_mm"] = post_loaded.get("max_final_rms_mm")
    if "max_fit_error_mm" not in post_loaded and "max_verify_rms_mm" in post_loaded:
        post["max_fit_error_mm"] = post_loaded.get("max_verify_rms_mm")
    if "step_mm" not in post_loaded and "final_step_mm" in post_loaded:
        post["step_mm"] = post_loaded.get("final_step_mm")
    if "step_mm" not in post_loaded and "rough_step_mm" in post_loaded:
        post["step_mm"] = post_loaded.get("rough_step_mm")
    if "speed" not in post_loaded or post_loaded.get("speed", None) is None:
        post["speed"] = 0.8
    if not config_bool(post.get("allow_single_read_contact", False), False):
        try:
            post["debounce_count"] = max(int(float(post.get("debounce_count", 3))), 3)
        except:
            post["debounce_count"] = 3
        try:
            post["debounce_retries"] = max(int(float(post.get("debounce_retries", 2))), 2)
        except:
            post["debounce_retries"] = 2
        try:
            post["debounce_interval_ms"] = max(int(float(post.get("debounce_interval_ms", 5))), 5)
        except:
            post["debounce_interval_ms"] = 5
    cfg["schema_version"] = 6
    return cfg


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
    cfg = normalize_loaded_config(loaded)
    cfg["_config_path"] = cfg_path
    return cfg


def public_clean_config(cfg):
    clean = clean_to_schema(DEFAULT_CONFIG, cfg)
    clean["schema_version"] = 6
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


def loc_to_log(loc):
    d = loc_to_dict(loc)
    return "X %.6f Y %.6f Z %.6f R %.6f" % (
        d["x"], d["y"], d["z"], d["rotation"])


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


def actuator_name_for_role(cfg, role):
    legacy = str(cfg.get("contact_actuator_name", "") or "").strip()
    if role == "z":
        name = str(cfg.get("z_actuator_name", "") or "").strip()
        return name or legacy
    if role == "post_xy":
        post = cfg.get("post_xy", {})
        name = str(post.get("actuator_name", "") or "").strip()
        return name or legacy
    raise NeedleCalibrationError("Unknown actuator role: %s" % role)


def find_actuator_by_name(machine_obj, cfg, role=None):
    name = str(cfg.get("contact_actuator_name", "") or "").strip()
    if role is not None:
        name = actuator_name_for_role(cfg, role)
    if name == "":
        if role == "z":
            raise NeedleCalibrationError("z_actuator_name is required")
        if role == "post_xy":
            raise NeedleCalibrationError("post_xy.actuator_name is required")
        raise NeedleCalibrationError("contact_actuator_name is required")
    owners = [machine_obj]
    try:
        owners.insert(0, find_nozzle(cfg, None, machine_obj).getHead())
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
    if role == "z":
        raise NeedleCalibrationError("Z actuator not found: %s" % name)
    if role == "post_xy":
        raise NeedleCalibrationError("Post XY actuator not found: %s" % name)
    raise NeedleCalibrationError("Contact actuator not found: %s" % name)


def find_named_actuator(machine_obj, name, nozzle=None):
    wanted = str(name or "").strip()
    if wanted == "":
        raise NeedleCalibrationError("Actuator name is required")
    owners = []
    if nozzle is not None:
        try:
            owners.append(nozzle.getHead())
        except:
            pass
    owners.append(machine_obj)
    for owner in owners:
        if owner is None:
            continue
        for method in ["getActuatorByName", "getActuator"]:
            try:
                actuator = getattr(owner, method)(wanted)
                if actuator is not None:
                    return actuator
            except:
                pass
        try:
            for actuator in java_list_to_py(owner.getActuators()):
                if object_name(actuator) == wanted:
                    return actuator
        except:
            pass
    raise NeedleCalibrationError("Actuator not found: %s" % wanted)


def wait_still(movable):
    movable.waitForCompletion(CompletionType.WaitForStillstand)


def move_safe(movable, target):
    target = target.convertToUnits(LengthUnit.Millimeters)
    current = get_movable_location(movable)
    # MovableUtils treats a tiny transformed XY discrepancy as a lateral move
    # and then runs the head-wide safe-Z sequence. After the camera has placed
    # the head, the first radial hardware start is often already within a few
    # microns of the nozzle position. On this machine that sequence can repeat
    # the raw safe-Z command without ever reaching the requested Z target.
    # If there is no meaningful lateral or rotational move, a direct Z-only
    # move is safe and avoids that planner loop.
    xy_delta = math.sqrt(math.pow(float(target.getX()) - float(current.getX()), 2.0) +
        math.pow(float(target.getY()) - float(current.getY()), 2.0))
    rotation_delta = abs(float(target.getRotation()) -
        float(current.getRotation()))
    if xy_delta <= 0.01 and rotation_delta <= 0.01:
        move_direct_speed(movable, target)
        return
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
    cur = get_movable_location(nozzle)
    safe_z = float(cfg.get("safe_z_mm", 25.0))
    if float(cur.getZ()) >= safe_z:
        return cur
    target = mm_loc(cur.getX(), cur.getY(), safe_z, cur.getRotation())
    log_msg(logger, "Retract to safe Z: %s" % loc_to_log(target))
    move_direct_speed(nozzle, target, cfg.get("travel_speed", 1.0))
    return get_movable_location(nozzle)


def get_vacuum_actuator(nozzle):
    for method_name in ["getVacuumActuator", "getVacuumValveActuator"]:
        try:
            actuator = getattr(nozzle, method_name)()
            if actuator is not None:
                return actuator
        except:
            pass
    try:
        head = nozzle.getHead()
        for method_name in ["getVacuumActuator", "getVacuumValveActuator"]:
            try:
                actuator = getattr(head, method_name)()
                if actuator is not None:
                    return actuator
            except:
                pass
    except:
        pass
    return None


def abort_nozzle_cleanup(nozzle, cfg, logger=None):
    errors = []
    part_was_on = False

    def record_error(label, err):
        errors.append("%s failed: %s" % (label, err))
        log_msg(logger, "Abort cleanup warning: %s" % errors[-1])

    try:
        vacuum = get_vacuum_actuator(nozzle)
        if vacuum is None:
            log_msg(logger, "Abort cleanup: no vacuum actuator configured")
        else:
            try:
                vacuum.actuate(False)
            except:
                vacuum.setActuated(False)
            try:
                vacuum.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
            except:
                wait_still(nozzle)
            log_msg(logger, "Abort cleanup: vacuum off")
    except Throwable as e:
        record_error("vacuum off", e)
    except Exception as e:
        record_error("vacuum off", e)

    try:
        part = nozzle.getPart()
        if part is not None:
            part_was_on = True
            nozzle.place()
            if nozzle.getPart() is not None:
                raise NeedleCalibrationError("nozzle still reports a part after place()")
            log_msg(logger, "Abort cleanup: nozzle part state cleared")
        else:
            log_msg(logger, "Abort cleanup: nozzle already reports no part")
    except Throwable as e:
        record_error("part-state clear", e)
    except Exception as e:
        record_error("part-state clear", e)

    try:
        lift_nozzle_to_safe_z(nozzle, cfg, logger)
        wait_still(nozzle)
        log_msg(logger, "Abort cleanup: nozzle moved to safe Z")
    except Throwable as e:
        record_error("safe-Z move", e)
    except Exception as e:
        record_error("safe-Z move", e)

    return {
        "ok": len(errors) == 0,
        "part_was_on": part_was_on,
        "errors": errors,
    }


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


def dwell_for_read(read_cfg):
    ms = int(read_cfg.get("read_settle_ms", 0))
    if ms > 0:
        Thread.sleep(ms)


def read_contact_debounced(actuator, read_cfg, expect_contact=None, logger=None,
        label="contact", mixed_result=False):
    count = int(read_cfg.get("debounce_count", 1))
    if count < 1:
        count = 1
    retries = int(read_cfg.get("debounce_retries", 1))
    if retries < 1:
        retries = 1
    interval_ms = int(read_cfg.get("debounce_interval_ms", 0))
    last = []
    for attempt in range(retries):
        readings = []
        for i in range(count):
            readings.append(read_contact_once(actuator))
            if i + 1 < count and interval_ms > 0:
                Thread.sleep(interval_ms)
        stable = True
        first = readings[0]
        for item in readings:
            if item != first:
                stable = False
                break
        if stable:
            if expect_contact is not None and first != bool(expect_contact):
                raise NeedleCalibrationError("%s expected contact=%s actual=%s" %
                    (label, bool(expect_contact), first))
            return first
        last = readings
        log_msg(logger, "%s unstable contact readings attempt %d/%d: %s" %
            (label, attempt + 1, retries, readings))
    if expect_contact is not None:
        raise NeedleCalibrationError("%s expected contact=%s actual readings=%s" %
            (label, bool(expect_contact), last))
    return bool(mixed_result)


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
        return {"kind": "location", "x_mm": float(loc.getX()),
            "y_mm": float(loc.getY()), "z_mm": float(loc.getZ()),
            "location": loc, "raw": str(item)}
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
    out = {"kind": "pixel", "x_px": float(x), "y_px": float(y), "raw": str(item)}
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


def add_vision_settings_names_from_owner(owner, names):
    if owner is None:
        return
    for method in ["getVisionSettings", "getVisionSettingsList"]:
        try:
            for item in java_list_to_py(getattr(owner, method)()):
                names.append(object_name(item))
        except:
            pass


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
                for item in java_list_to_py(getattr(owner, method)()):
                    if object_name(item) == wanted:
                        return item
            except:
                pass
    return None


def list_available_vision_pipeline_names(machine_obj=None):
    names = []
    try:
        add_vision_settings_names_from_owner(Configuration.get(), names)
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


def load_pipeline_from_source(source):
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
        raise NeedleCalibrationError("Unsupported pipeline mode: %s" % mode)
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


def capture_with_pipeline(cfg, role, fid, nozzle=None, machine_obj=None, logger=None):
    machine_obj = get_machine(machine_obj)
    nozzle = find_nozzle(cfg, nozzle, machine_obj)
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
        raise NeedleCalibrationError("%s pipeline stage '%s' returned no features" %
            (role, stage_name))
    features = []
    for item in items:
        features.append(add_machine_coordinates_to_feature(camera, nozzle, parse_feature_item(item)))
    feature = features[0]
    out = {
        "role": role,
        "camera": object_name(camera),
        "stage": stage_name,
        "camera_location_mm": loc_to_dict(get_movable_location(camera)),
        "features": features,
        "feature": feature,
        "center": {"x": float(feature["x_mm"]), "y": float(feature["y_mm"])}
    }
    if role == "post_xy":
        out["vision_center"] = out["center"]
        out["detected_post_diameter_mm"] = feature.get("diameter_mm", None)
    if bool(cfg.get("debug", {}).get("save_last_capture", True)):
        out["image"] = image
    return out


def capture_z_touch_center(cfg, nozzle=None, machine_obj=None, logger=None):
    fid = cfg.get("fiducials", {}).get("z_touch", {})
    return capture_with_pipeline(cfg, "z_touch", fid, nozzle, machine_obj, logger)


def capture_post_center(cfg, nozzle=None, machine_obj=None, logger=None):
    return capture_with_pipeline(cfg, "post_xy", cfg.get("post_xy", {}),
        nozzle, machine_obj, logger)


def descend_until_contact(nozzle, actuator, read_cfg, x_mm, y_mm, start_z,
        limit_z, step_mm, speed, logger=None, label="probe"):
    z = float(start_z)
    step = abs(float(step_mm))
    steps = 0
    while z - step >= float(limit_z) - 1.0e-9:
        z -= step
        steps += 1
        move_direct_speed(nozzle, mm_loc(x_mm, y_mm, z,
            get_movable_location(nozzle).getRotation()), speed)
        dwell_for_read(read_cfg)
        if read_contact_debounced(actuator, read_cfg, logger=logger, label=label):
            return get_movable_location(nozzle), steps
    return None, steps


def retract_until_released(nozzle, actuator, read_cfg, x_mm, y_mm, step_mm,
        max_mm, speed, logger=None, label="release"):
    moved = 0.0
    while True:
        dwell_for_read(read_cfg)
        if not read_contact_debounced(actuator, read_cfg, logger=logger, label=label,
                mixed_result=True):
            return moved
        if moved >= float(max_mm) - 1.0e-9:
            raise NeedleCalibrationError("%s: contact did not release after %.6f mm" %
                (label, float(max_mm)))
        inc = min(abs(float(step_mm)), float(max_mm) - moved)
        moved += inc
        cur = get_movable_location(nozzle)
        move_direct_speed(nozzle, mm_loc(x_mm, y_mm, cur.getZ() + inc,
            cur.getRotation()), speed)


def _positive_feed(value, label):
    try:
        result = float(value)
    except:
        raise NeedleCalibrationError("%s must be a number" % label)
    if result <= 0.0:
        raise NeedleCalibrationError("%s must be > 0" % label)
    return result


def _fine_repeat_count(value):
    try:
        number = float(value)
        result = int(number)
    except:
        raise NeedleCalibrationError("z_probe.fine_repeats must be a whole number")
    if number != float(result) or result < 1:
        raise NeedleCalibrationError("z_probe.fine_repeats must be a whole number >= 1")
    return result


def _repeat_count(value, label, minimum=1):
    try:
        number = float(value)
        result = int(number)
    except:
        raise NeedleCalibrationError(
            "%s must be a whole number >= %d" % (label, int(minimum)))
    if number != float(result) or result < int(minimum):
        raise NeedleCalibrationError(
            "%s must be a whole number >= %d" % (label, int(minimum)))
    return result


def _probe_channel(value, label):
    channel = str(value or "").strip().upper()
    if channel not in ["Q0", "Q1"]:
        raise NeedleCalibrationError("%s must be Q0 or Q1" % label)
    return channel


def _probe_mode(cfg):
    mode = str(cfg.get("z_probe", {}).get("mode", "actuator") or "actuator").strip().lower()
    if mode not in ["actuator", "direct_gcode"]:
        raise NeedleCalibrationError(
            "z_probe.mode must be actuator or direct_gcode")
    return mode


def _driver_for_probe(probe_actuator):
    try:
        driver = probe_actuator.getDriver()
    except:
        driver = None
    if driver is None or not hasattr(driver, "sendCommand") or not hasattr(driver, "getReportedLocation"):
        raise NeedleCalibrationError(
            "Probe/driver actuator must use a GcodeDriver for interrupt probing")
    return driver


def _reported_nozzle_z(nozzle, axes_location):
    return float(_reported_nozzle_location(nozzle, axes_location).getZ())


def _reported_nozzle_location(nozzle, axes_location):
    location = nozzle.toTransformed(axes_location)
    location = nozzle.toHeadMountableLocation(location)
    return location.convertToUnits(LengthUnit.Millimeters)


def _raw_z_to_nozzle_z(nozzle, driver, raw_z):
    machine_obj = get_machine(None)
    raw_axis = None
    try:
        for axis in java_list_to_py(machine_obj.getAxes()):
            try:
                if axis.getType() == Axis.Type.Z and axis.getDriver() == driver:
                    raw_axis = axis
                    break
            except:
                pass
    except:
        pass
    if raw_axis is None:
        raise NeedleCalibrationError("Unable to resolve the raw Z axis for actuator log positions")
    axes_location = AxesLocation(raw_axis, Length(float(raw_z), driver.getUnits()))
    return _reported_nozzle_z(nozzle, axes_location)


def _openpnp_log_path():
    return os.path.join(str(System.getProperty("user.home")), ".openpnp2", "log", "OpenPnP.log")


def _position_report_z_values(text):
    values = []
    pattern = re.compile(r"Position report:.*?\bZ:\s*(-?[0-9]+(?:\.[0-9]+)?)")
    for line in str(text).splitlines():
        match = pattern.search(line)
        if match:
            values.append(float(match.group(1)))
    return values


def _wait_for_sequence_completion_log(log_path, start_offset, timeout_ms=15000):
    deadline = System.currentTimeMillis() + int(timeout_ms)
    emitted = ""
    while System.currentTimeMillis() < deadline:
        fh = open(log_path, "r")
        try:
            fh.seek(start_offset)
            emitted = fh.read()
        finally:
            fh.close()
        restore = re.search(r"confirmed\s+G61\s*S0\s*Z\s*F[0-9.]+", emitted)
        if restore is not None and re.search(r">>\s+G90\b", emitted[restore.end():]):
            return emitted
        Thread.sleep(20)
    raise NeedleCalibrationError(
        "Timed out waiting for the sequence actuator to finish G61/M400")


def _restore_saved_z(driver):
    errors = []
    for command in ["G90", "G61 S0 Z F7500", "M400"]:
        try:
            driver.sendCommand(command)
        except Exception as error:
            errors.append("%s: %s" % (command, error))
        except Throwable as error:
            errors.append("%s: %s" % (command, error))
    try:
        _drain_driver_queue(driver)
        reported = driver.getReportedLocation(-1)
        _drain_driver_queue(driver)
    except Exception as error:
        errors.append("M114: %s" % error)
        reported = None
    except Throwable as error:
        errors.append("M114: %s" % error)
        reported = None
    return reported, errors


def _restore_saved_xy(driver):
    errors = []
    for command in ["G90", "G61 S0 X Y F7500", "M400"]:
        try:
            driver.sendCommand(command)
        except Exception as error:
            errors.append("%s: %s" % (command, error))
        except Throwable as error:
            errors.append("%s: %s" % (command, error))
    try:
        _drain_driver_queue(driver)
        driver.getReportedLocation(-1)
        _drain_driver_queue(driver)
    except Exception as error:
        errors.append("M114: %s" % error)
    except Throwable as error:
        errors.append("M114: %s" % error)
    return errors


def _drain_driver_queue(driver, timeout_ms=60000):
    """Use GcodeAsyncDriver's public unconditional FIFO-drain path."""
    try:
        if hasattr(driver, "isConfirmationFlowControl") and not bool(
                driver.isConfirmationFlowControl()):
            raise NeedleCalibrationError(
                "GcodeAsyncDriver confirmationFlowControl must be enabled for interrupt probing")
    except NeedleCalibrationError:
        raise
    except:
        pass
    if not (hasattr(driver, "isReportedLocationConfirmation") and
            hasattr(driver, "setReportedLocationConfirmation") and
            hasattr(driver, "waitForCompletion")):
        # A synchronous GcodeDriver has completed each sendCommand already.
        return
    reported_confirmation = bool(driver.isReportedLocationConfirmation())
    try:
        # With reported-location confirmation disabled, the async driver's
        # public waitForCompletion() uses its private FIFO null-command
        # sentinel. This cannot be released by an old or unsolicited M114.
        driver.setReportedLocationConfirmation(False)
        driver.waitForCompletion(None,
            CompletionType.WaitForUnconditionalCoordination)
    finally:
        driver.setReportedLocationConfirmation(reported_confirmation)


def _actuate_boolean_true(actuator):
    # Jython 2.7 chooses actuate(double) for both Python True and
    # java.lang.Boolean.TRUE when boolean/double/Object overloads coexist.
    # Invoke the primitive-boolean signature explicitly.
    parameter_types = Array.newInstance(Class, 1)
    parameter_types[0] = Boolean.TYPE
    method = actuator.getClass().getMethod("actuate", parameter_types)
    arguments = Array.newInstance(Object, 1)
    arguments[0] = Boolean.TRUE
    method.invoke(actuator, arguments)


def _run_direct_interrupt_sequence(nozzle, driver, zp, logger, label):
    coarse_feed = _positive_feed(zp.get("coarse_feed", 3000.0),
        "z_probe.coarse_feed")
    fine_feed = _positive_feed(zp.get("fine_feed", 500.0),
        "z_probe.fine_feed")
    fine_repeats = _fine_repeat_count(zp.get("fine_repeats", 1))
    channel = _probe_channel(zp.get("channel", "Q0"), "z_probe.channel")
    acceleration = _positive_feed(zp.get("probe_acceleration", 1000.0),
        "z_probe.probe_acceleration")
    max_descent = _positive_feed(zp.get("max_descent_mm", 5.0),
        "z_probe.max_descent_mm")
    retract = _positive_feed(zp.get("retract_after_contact_mm", 1.0),
        "z_probe.retract_after_contact_mm")
    fine_descent = retract * 2.0
    coarse_z = None
    fine_z_values = []
    restore_errors = []
    try:
        _drain_driver_queue(driver)
        # OpenPnP's motion planner leaves M204 modal. A tiny interpolated move
        # immediately before probing can leave acceleration as low as S11,
        # making identical G38.2 feed rates run inconsistently. Establish the
        # normal Z acceleration used by this machine before every raw probe.
        for command in ["M400", "M204 S%.3f" % acceleration, "G60 S0", "G91"]:
            driver.sendCommand(command)
        driver.sendCommand("G38.2 %s Z%.4f F%.3f" %
            (channel, max_descent, coarse_feed))
        driver.sendCommand("M400")
        _drain_driver_queue(driver)
        coarse_z = _reported_nozzle_z(nozzle, driver.getReportedLocation(-1))
        _drain_driver_queue(driver)
        log_msg(logger, "%s coarse interrupt Z %.6f" % (label, coarse_z))
        for index in range(fine_repeats):
            driver.sendCommand("G0 Z-%.4f F5000" % retract)
            driver.sendCommand("M204 S%.3f" % acceleration)
            driver.sendCommand("G38.2 %s Z%.4f F%.3f" %
                (channel, fine_descent, fine_feed))
            driver.sendCommand("M400")
            _drain_driver_queue(driver)
            fine_z = _reported_nozzle_z(nozzle, driver.getReportedLocation(-1))
            _drain_driver_queue(driver)
            fine_z_values.append(fine_z)
            log_msg(logger, "%s fine interrupt %d/%d Z %.6f" %
                (label, index + 1, fine_repeats, fine_z))
    finally:
        unused_report, restore_errors = _restore_saved_z(driver)
    if restore_errors:
        raise NeedleCalibrationError(
            "%s failed to restore G90/saved Z: %s" % (label, "; ".join(restore_errors)))
    if coarse_z is None or len(fine_z_values) != fine_repeats:
        raise NeedleCalibrationError("%s interrupt probe did not return all requested contacts" % label)
    return coarse_z, fine_z_values


def _run_actuator_interrupt_sequence(nozzle, probe_actuator, machine_obj, zp,
        logger, label):
    sequence_name = str(zp.get("sequence_actuator_name", "Run Voron Probe") or "").strip()
    sequence_actuator = find_named_actuator(machine_obj, sequence_name, nozzle)
    driver = _driver_for_probe(probe_actuator)
    if sequence_actuator.getDriver() != driver:
        raise NeedleCalibrationError(
            "Sequence actuator and Z probe actuator must use the same GcodeDriver")
    log_path = _openpnp_log_path()
    if not os.path.exists(log_path):
        raise NeedleCalibrationError("OpenPnP log not found: %s" % log_path)
    fh = open(log_path, "r")
    try:
        fh.seek(0, 2)
        start_offset = fh.tell()
    finally:
        fh.close()
    log_msg(logger, "%s running actuator '%s'" % (label, sequence_name))
    try:
        _actuate_boolean_true(sequence_actuator)
        # FIFO sentinel: it cannot be written until the actuator's final M400
        # has been acknowledged. It also reinforces absolute mode.
        driver.sendCommand("G90")
        _wait_for_sequence_completion_log(log_path, start_offset)
    except:
        unused_report, restore_errors = _restore_saved_z(driver)
        raise
    # The actuator has confirmed its final M400. Request one new position so
    # OpenPnP is synchronized to the restored saved Z and so the last log
    # report is unambiguously not a contact.
    final_report = driver.getReportedLocation(-1)
    final_z = _reported_nozzle_z(nozzle, final_report)
    Thread.sleep(50)
    fh = open(log_path, "r")
    try:
        fh.seek(start_offset)
        emitted = fh.read()
    finally:
        fh.close()
    raw_values = _position_report_z_values(emitted)
    if len(raw_values) < 3:
        raise NeedleCalibrationError(
            "%s actuator log needs coarse, fine, and restored M114 position reports; found %d. Enable TRACE logging and verify the actuator sequence." %
            (label, len(raw_values)))
    # The explicit blocking M114 above is the last report. Any immediately
    # preceding duplicate is also a restored-position report, not a contact.
    restored_raw = raw_values[-1]
    raw_contacts = raw_values[:-1]
    while len(raw_contacts) > 1 and abs(raw_contacts[-1] - restored_raw) <= 0.0005:
        raw_contacts.pop()
    if len(raw_contacts) < 2:
        raise NeedleCalibrationError(
            "%s actuator sequence did not report both coarse and fine contacts" % label)
    contact_values = [_raw_z_to_nozzle_z(nozzle, driver, value) for value in raw_contacts]
    coarse_z = contact_values[0]
    fine_z_values = contact_values[1:]
    log_msg(logger, "%s actuator coarse interrupt Z %.6f" % (label, coarse_z))
    for index, fine_z in enumerate(fine_z_values):
        log_msg(logger, "%s actuator fine interrupt %d/%d Z %.6f" %
            (label, index + 1, len(fine_z_values), fine_z))
    log_msg(logger, "%s actuator restored Z %.6f" % (label, final_z))
    return coarse_z, fine_z_values


def run_interrupt_probe_sequence(nozzle, probe_actuator, cfg, logger=None,
        label="z_probe"):
    zp = cfg.get("z_probe", {})
    mode = _probe_mode(cfg)
    driver = _driver_for_probe(probe_actuator)
    if mode == "direct_gcode":
        retries = _repeat_count(zp.get("g38_attempts", 2),
            "z_probe.g38_attempts")
        last_error = None
        for attempt in range(retries):
            try:
                coarse_z, fine_z_values = _run_direct_interrupt_sequence(
                    nozzle, driver, zp, logger, label)
                last_error = None
                break
            except Throwable as error:
                last_error = error
            except Exception as error:
                last_error = error
            if attempt + 1 < retries:
                log_msg(logger, "%s G38 attempt %d/%d failed: %s; retrying from restored Z" %
                    (label, attempt + 1, retries, str(last_error)))
        if last_error is not None:
            raise last_error
    else:
        coarse_z, fine_z_values = _run_actuator_interrupt_sequence(
            nozzle, probe_actuator, get_machine(None), zp, logger, label)
    average_z = sum(fine_z_values) / float(len(fine_z_values))
    return {
        "mode": mode,
        "coarse_contact_z_mm": coarse_z,
        "fine_contact_z_values_mm": fine_z_values,
        "fine_repeat_count": len(fine_z_values),
        "averaged_contact_z_mm": average_z,
        "fine_spread_mm": max(fine_z_values) - min(fine_z_values)
    }


def run_polling_z_probe_sequence(nozzle, actuator, cfg, x_mm, y_mm,
        start_z, logger=None, label="z_touch"):
    zp = deep_copy(cfg.get("z_probe", {}))
    coarse_count = _repeat_count(zp.get("polling_coarse_repeats", 1),
        "z_probe.polling_coarse_repeats")
    fine_count = _repeat_count(zp.get("polling_fine_repeats", 1),
        "z_probe.polling_fine_repeats")
    coarse_step = _positive_feed(zp.get("coarse_step_mm", 0.10),
        "z_probe.coarse_step_mm")
    fine_step = _positive_feed(zp.get("fine_step_mm", 0.02),
        "z_probe.fine_step_mm")
    retract = _positive_feed(zp.get("retract_after_contact_mm", 0.40),
        "z_probe.retract_after_contact_mm")
    release_step = _positive_feed(zp.get("release_retract_step_mm", 0.20),
        "z_probe.release_retract_step_mm")
    max_release = _positive_feed(zp.get("max_release_retract_mm", 2.0),
        "z_probe.max_release_retract_mm")
    max_descent = _positive_feed(zp.get("max_descent_mm", 8.0),
        "z_probe.max_descent_mm")
    coarse_speed = _positive_feed(cfg.get("probe_speed", 0.8), "probe_speed")
    fine_speed = _positive_feed(cfg.get("fine_probe_speed", 0.05),
        "fine_probe_speed")
    rotation = get_movable_location(nozzle).getRotation()
    limit_z = float(start_z) - max_descent

    def move_released(z_mm, speed, phase):
        move_direct_speed(nozzle, mm_loc(x_mm, y_mm, z_mm, rotation), speed)
        dwell_for_read(zp)
        if read_contact_debounced(actuator, zp, logger=logger,
                label=phase + " precheck", mixed_result=True):
            retract_until_released(nozzle, actuator, zp, x_mm, y_mm,
                release_step, max_release, speed, logger, phase + " release")
        return float(get_movable_location(nozzle).getZ())

    coarse_values = []
    coarse_records = []
    for index in range(coarse_count):
        phase = "%s coarse %d/%d" % (label, index + 1, coarse_count)
        phase_start = move_released(float(start_z), coarse_speed, phase)
        contact, steps = descend_until_contact(nozzle, actuator, zp, x_mm, y_mm,
            phase_start, limit_z, coarse_step, coarse_speed, logger, phase)
        if contact is None:
            raise NeedleCalibrationError("%s: no contact within max descent" % phase)
        value = float(contact.getZ())
        coarse_values.append(value)
        coarse_records.append({"repeat": index + 1, "trigger_z_mm": value,
            "steps": int(steps), "source": "polling"})
        move_released(value + retract, coarse_speed, phase + " retract")

    coarse_average = sum(coarse_values) / float(len(coarse_values))
    fine_start = coarse_average + retract
    fine_values = []
    fine_records = []
    for index in range(fine_count):
        phase = "%s fine %d/%d" % (label, index + 1, fine_count)
        phase_start = move_released(fine_start, fine_speed, phase)
        contact, steps = descend_until_contact(nozzle, actuator, zp, x_mm, y_mm,
            phase_start, limit_z, fine_step, fine_speed, logger, phase)
        if contact is None:
            raise NeedleCalibrationError("%s: no contact within max descent" % phase)
        value = float(contact.getZ())
        fine_values.append(value)
        fine_records.append({"repeat": index + 1, "trigger_z_mm": value,
            "steps": int(steps), "source": "polling"})
        move_released(value + retract, fine_speed, phase + " retract")

    average_z = sum(fine_values) / float(len(fine_values))
    return {"mode": "polling", "acquisition_method": "polling",
        "coarse_contact_z_mm": coarse_average,
        "coarse_contact_z_values_mm": coarse_values,
        "coarse_repeat_count": len(coarse_values), "coarse_records": coarse_records,
        "fine_contact_z_values_mm": fine_values,
        "fine_repeat_count": len(fine_values), "fine_records": fine_records,
        "averaged_contact_z_mm": average_z,
        "fine_spread_mm": max(fine_values) - min(fine_values)}


def probe_z_touch(nozzle, actuator, cfg, x_mm, y_mm, surface_z_mm, logger=None):
    zp = deep_copy(cfg.get("z_probe", {}))
    z_touch = cfg.get("fiducials", {}).get("z_touch", {})
    start_z = float(surface_z_mm) + float(zp.get("start_above_mm", 4.0))
    trigger_offset = float(z_touch.get("trigger_offset_mm",
        zp.get("trigger_offset_mm", 0.0)))
    start = mm_loc(x_mm, y_mm, start_z, get_movable_location(nozzle).getRotation())
    log_msg(logger, "Z touch move above pad %s" % loc_to_log(start))
    move_safe(nozzle, start)
    acquisition = str(zp.get("acquisition_method", "hardware_interrupt") or
        "hardware_interrupt").strip().lower()
    if acquisition == "polling":
        sequence = run_polling_z_probe_sequence(nozzle, actuator, cfg,
            x_mm, y_mm, start_z, logger, "z_touch")
    elif acquisition == "hardware_interrupt":
        sequence = run_interrupt_probe_sequence(nozzle, actuator, cfg, logger,
            "z_touch")
        sequence["acquisition_method"] = "hardware_interrupt"
    else:
        raise NeedleCalibrationError(
            "z_probe.acquisition_method must be polling or hardware_interrupt")
    raw_contact_z = float(sequence.get("averaged_contact_z_mm"))
    corrected_contact_z = raw_contact_z - trigger_offset
    return {
        "surface_z_mm": float(surface_z_mm),
        "coarse_contact": {"z": float(sequence.get("coarse_contact_z_mm"))},
        "contact": {"z": raw_contact_z},
        "measured_contact_z_mm": raw_contact_z,
        "trigger_offset_mm": trigger_offset,
        "corrected_contact_z_mm": corrected_contact_z,
        "z_error_mm": float(surface_z_mm) - corrected_contact_z,
        "interrupt_sequence": sequence
    }


def resolve_post_side_probe_z_mm(cfg, logger=None):
    post = cfg.get("post_xy", {})
    loc = dict_to_loc(post.get("location", {}))
    z = float(loc.getZ()) + float(post.get("side_probe_z_offset_mm", 0.25))
    log_msg(logger, "Post side probe Z %.6f" % z)
    return z


def optional_float(value, fallback):
    if value is None:
        return float(fallback)
    try:
        text = str(value).strip()
        if text == "" or text.lower() == "none":
            return float(fallback)
    except:
        pass
    return float(value)


def config_bool(value, fallback=False):
    if value is None:
        return bool(fallback)
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ["1", "true", "t", "yes", "y", "on"]:
        return True
    if text in ["0", "false", "f", "no", "n", "off"]:
        return False
    return bool(fallback)


def post_xy_settings(cfg):
    post = cfg.get("post_xy", {})
    # Compatibility with older two-pass post XY configs.
    step = abs(float(post.get("step_mm",
        post.get("final_step_mm", post.get("rough_step_mm", 0.05)))))
    speed = optional_float(post.get("speed", None), cfg.get("probe_speed", 0.8))
    if step <= 0:
        raise NeedleCalibrationError("post_xy step_mm must be > 0")
    return {"step_mm": step, "speed": float(speed)}


def post_scan_settings(cfg):
    post = cfg.get("post_xy", {})
    clearance = abs(float(post.get("scan_clearance_mm", 2.0)))
    overtravel = abs(float(post.get("scan_overtravel_mm", 3.0)))
    cap = abs(float(post.get("max_radial_probe_travel_mm", 5.0)))
    return {"clearance_mm": clearance, "overtravel_mm": overtravel,
        "cap_mm": cap, "travel_limit_mm": min(cap, clearance + overtravel)}


def recover_outside_start_if_touching(nozzle, actuator, read_cfg, start_x, start_y,
        ux, uy, z_probe, speed, logger=None, label="edge",
        start_radius_mm=None, max_start_radius_mm=None,
        minimum_outward_mm=0.0):
    dwell_for_read(read_cfg)
    touching = read_contact_debounced(actuator, read_cfg, logger=logger,
        label=label + " outside", mixed_result=True)
    log_msg(logger, "%s outside pos=%s contact=%s" %
        (label, fmt_xy(start_x, start_y), touching))
    minimum_outward = max(0.0, float(minimum_outward_mm))
    step = abs(float(read_cfg.get("outside_recover_step_mm", 0.25)))
    max_outward = abs(float(read_cfg.get("outside_recover_max_mm", 3.0)))
    if step <= 0.0:
        raise NeedleCalibrationError("%s: outside_recover_step_mm must be > 0" % label)
    if max_outward <= 0.0:
        raise NeedleCalibrationError("%s: outside_recover_max_mm must be > 0" % label)
    if not touching and minimum_outward <= 1.0e-9:
        return get_movable_location(nozzle), 0.0

    if minimum_outward > max_outward + 1.0e-9:
        raise NeedleCalibrationError(
            "%s: required outward recovery %.6f mm exceeds configured maximum %.6f mm" %
            (label, minimum_outward, max_outward))
    log_msg(logger, "%s outside %s; recovering outward" %
        (label, "touching" if touching else "clear but insufficient coarse travel"))
    lift = abs(float(read_cfg.get("local_lift_mm", 0.60)))
    current = get_movable_location(nozzle)
    if float(current.getZ()) < float(z_probe) + lift - 1.0e-9:
        move_direct_speed(nozzle, mm_loc(current.getX(), current.getY(),
            z_probe + lift, current.getRotation()), speed)
    outward = 0.0
    index = 0
    while touching or outward < minimum_outward - 1.0e-9:
        if outward >= max_outward - 1.0e-9:
            break
        next_outward = min(max_outward, max(outward + step, minimum_outward))
        if max_start_radius_mm is not None and start_radius_mm is not None:
            available = float(max_start_radius_mm) - float(start_radius_mm)
            if available <= outward + 1.0e-9:
                cur = get_movable_location(nozzle)
                lift = abs(float(read_cfg.get("local_lift_mm", 0.60)))
                move_direct_speed(nozzle, mm_loc(cur.getX(), cur.getY(),
                    z_probe + lift, cur.getRotation()), speed)
                raise NeedleCalibrationError(
                    "%s: active contact could not be released within the "
                    "reachable outward start envelope (start radius %.6f mm, "
                    "maximum safe radius %.6f mm)" %
                    (label, float(start_radius_mm), float(max_start_radius_mm)))
            next_outward = min(next_outward, available)
        if next_outward <= outward + 1.0e-9:
            break
        outward = next_outward
        index += 1
        x = float(start_x) + float(ux) * outward
        y = float(start_y) + float(uy) * outward
        move_direct_speed(nozzle, mm_loc(x, y, z_probe + lift,
            get_movable_location(nozzle).getRotation()), speed)
        move_direct_speed(nozzle, mm_loc(x, y, z_probe,
            get_movable_location(nozzle).getRotation()), speed)
        dwell_for_read(read_cfg)
        touching = read_contact_debounced(actuator, read_cfg, logger=logger,
            label=label + " recover step %d" % index)
        log_msg(logger, "%s recover step=%d outward=%s pos=%s contact=%s" %
            (label, index, fmt_float(outward), fmt_xy(x, y), touching))
        if not touching and outward >= minimum_outward - 1.0e-9:
            return get_movable_location(nozzle), outward

    cur = get_movable_location(nozzle)
    move_direct_speed(nozzle, mm_loc(cur.getX(), cur.getY(), z_probe + lift,
        cur.getRotation()), speed)
    raise NeedleCalibrationError(
        "%s: outside start remained touching after outward recovery %.6f mm" %
        (label, max_outward))


def move_inward_until_contact(nozzle, actuator, read_cfg, start_x, start_y,
        ux, uy, max_distance, step_mm, z_probe, speed, logger=None, label="edge"):
    traveled = 0.0
    step = abs(float(step_mm))
    index = 0
    log_every_step = config_bool(read_cfg.get("log_every_step", False), False)
    log_step_interval = int(read_cfg.get("log_step_interval", 10))
    if log_step_interval < 1:
        log_step_interval = 1

    while traveled < float(max_distance) - 1.0e-9:
        traveled = min(float(max_distance), traveled + step)
        index += 1

        x = float(start_x) - float(ux) * traveled
        y = float(start_y) - float(uy) * traveled

        move_direct_speed(nozzle, mm_loc(x, y, z_probe,
            get_movable_location(nozzle).getRotation()), speed)

        dwell_for_read(read_cfg)
        touching = read_contact_debounced(actuator, read_cfg, logger=logger,
            label=label + " step %d" % index)

        if log_every_step or touching or index == 1 or index % log_step_interval == 0:
            log_msg(logger, "%s step=%d travel=%s pos=%s contact=%s" %
                (label, index, fmt_float(traveled), fmt_xy(x, y), touching))

        if touching:
            return get_movable_location(nozzle), traveled, index

    return None, traveled, index


def move_outward_by(nozzle, start_loc, ux, uy, distance_mm, z_probe, speed):
    loc = start_loc.convertToUnits(LengthUnit.Millimeters)
    x = float(loc.getX()) + float(ux) * float(distance_mm)
    y = float(loc.getY()) + float(uy) * float(distance_mm)
    move_direct_speed(nozzle, mm_loc(x, y, z_probe, loc.getRotation()), speed)
    return get_movable_location(nozzle)


def confirm_contact_point(cfg, nozzle, actuator, read_cfg, first_contact,
        ux, uy, z_probe, speed, logger=None, label="edge"):
    backoff = abs(float(read_cfg.get("confirm_backoff_mm", 0.4)))
    reapproach = abs(float(read_cfg.get("confirm_reapproach_mm", 0.8)))
    tolerance = abs(float(read_cfg.get("confirm_position_tolerance_mm", 0.15)))
    reject_early = config_bool(read_cfg.get("confirm_reject_early", True), True)
    min_confirm_travel = max(0.0, backoff - tolerance)
    max_confirm_travel = min(reapproach, backoff + tolerance)
    raw_step = read_cfg.get("confirm_step_mm", None)
    try:
        if raw_step is None or str(raw_step).strip() == "" or str(raw_step).strip().lower() == "none":
            confirm_step = float(post_xy_settings(cfg)["step_mm"])
        else:
            confirm_step = abs(float(raw_step))
    except:
        confirm_step = float(post_xy_settings(cfg)["step_mm"])
    if confirm_step <= 0.0:
        raise NeedleCalibrationError("%s: confirm_step_mm must be > 0" % label)
    if max_confirm_travel < min_confirm_travel - 1.0e-9:
        raise NeedleCalibrationError(
            "%s: confirm_reapproach_mm %.6f is shorter than minimum confirmation travel %.6f" %
            (label, reapproach, min_confirm_travel))

    log_msg(logger, "%s first contact=%s backing off=%s" %
        (label, fmt_xy(first_contact.getX(), first_contact.getY()),
         fmt_float(backoff)))
    log_msg(logger, "%s CONFIRM expectedTravel=%s tolerance=%s validWindow=%s..%s" %
        (label, fmt_float(backoff), fmt_float(tolerance),
         fmt_float(min_confirm_travel), fmt_float(max_confirm_travel)))
    backed_off = move_outward_by(nozzle, first_contact, ux, uy, backoff,
        z_probe, speed)
    dwell_for_read(read_cfg)
    touching = read_contact_debounced(actuator, read_cfg, logger=logger,
        label=label + " confirm backoff")
    log_msg(logger, "%s CONFIRM backoff=%s pos=%s contact=%s" %
        (label, fmt_float(backoff), fmt_xy(backed_off.getX(), backed_off.getY()),
         touching))
    if touching:
        raise NeedleCalibrationError(
            "%s: contact still active after %.6f mm confirmation backoff" %
            (label, backoff))

    traveled = 0.0
    steps = 0
    while traveled < max_confirm_travel - 1.0e-9:
        traveled = min(max_confirm_travel, traveled + confirm_step)
        steps += 1
        x = float(backed_off.getX()) - float(ux) * traveled
        y = float(backed_off.getY()) - float(uy) * traveled
        move_direct_speed(nozzle, mm_loc(x, y, z_probe,
            backed_off.getRotation()), speed)
        dwell_for_read(read_cfg)
        touching = read_contact_debounced(actuator, read_cfg, logger=logger,
            label=label + " CONFIRM step %d" % steps)
        log_msg(logger, "%s CONFIRM step=%d travel=%s pos=%s contact=%s" %
            (label, steps, fmt_float(traveled), fmt_xy(x, y), touching))
        if touching:
            if reject_early and traveled < min_confirm_travel - 1.0e-9:
                log_msg(logger, "%s CONFIRM FAILED too early travel=%s min=%s" %
                    (label, fmt_float(traveled), fmt_float(min_confirm_travel)))
                raise NeedleCalibrationError(
                    "%s: confirmation contact too early: confirmTravel %.6f before minimum %.6f; first contact was not repeatable" %
                    (label, traveled, min_confirm_travel))
            confirmed = get_movable_location(nozzle)
            log_msg(logger, "%s CONFIRMED CONTACT pos=%s confirmTravel=%s" %
                (label, fmt_xy(confirmed.getX(), confirmed.getY()),
                 fmt_float(traveled)))
            return confirmed, float(traveled), int(steps), backed_off

    raise NeedleCalibrationError(
        "%s: could not reconfirm contact near first touch; expected %.6f +/- %.6f" %
        (label, backoff, tolerance))


def _run_xy_interrupt_attempt(nozzle, driver, post, dx, dy, feed, logger, label):
    channel = _probe_channel(post.get("hardware_channel", "Q1"),
        "post_xy.hardware_channel")
    acceleration = _positive_feed(
        post.get("hardware_probe_acceleration", 1000.0),
        "post_xy.hardware_probe_acceleration")
    contact = None
    restore_errors = []
    probe_error = None
    try:
        _drain_driver_queue(driver)
        for command in ["M400", "M204 S%.3f" % acceleration, "G60 S0", "G91"]:
            driver.sendCommand(command)
        driver.sendCommand("G38.2 %s X%.4f Y%.4f F%.3f" %
            (channel, float(dx), float(dy), float(feed)))
        driver.sendCommand("M400")
        _drain_driver_queue(driver)
        contact = _reported_nozzle_location(nozzle, driver.getReportedLocation(-1))
        _drain_driver_queue(driver)
        log_msg(logger, "%s interrupt contact %s" % (label, loc_to_log(contact)))
    except Throwable as error:
        probe_error = error
    except Exception as error:
        probe_error = error
    finally:
        restore_errors = _restore_saved_xy(driver)
    if restore_errors:
        detail = ""
        if probe_error is not None:
            detail = "; G38 operation also failed: %s" % str(probe_error)
        raise NeedleCalibrationSafetyError(
            "%s SAFETY-CRITICAL: failed to restore G90/saved XY: %s%s; "
            "interrupt retry is unsafe" %
            (label, "; ".join(restore_errors), detail))
    if probe_error is not None:
        raise probe_error
    if contact is None:
        raise NeedleCalibrationError("%s did not return a contact position" % label)
    return contact


def _run_xy_interrupt_once(nozzle, driver, actuator, post, dx, dy, feed,
        logger, label):
    attempts = _repeat_count(post.get("hardware_g38_attempts", 2),
        "post_xy.hardware_g38_attempts")
    last_error = None
    for attempt in range(attempts):
        try:
            return _run_xy_interrupt_attempt(nozzle, driver, post, dx, dy,
                feed, logger, label)
        except NeedleCalibrationSafetyError:
            raise
        except Throwable as error:
            last_error = error
        except Exception as error:
            last_error = error
        if attempt + 1 < attempts:
            if actuator is None:
                raise NeedleCalibrationSafetyError(
                    "%s G38 attempt %d failed: contact actuator is unavailable; "
                    "refusing an unverified retry: %s" %
                    (label, attempt + 1, str(last_error)))
            dwell_for_read(post)
            try:
                contact_active = read_contact_debounced(actuator, post,
                    logger=logger, label=label + " retry release check",
                    mixed_result=True)
            except Throwable as error:
                raise NeedleCalibrationSafetyError(
                    "%s G38 attempt %d failed and contact release could not be "
                    "verified; refusing retry: %s" %
                    (label, attempt + 1, str(error)))
            except Exception as error:
                raise NeedleCalibrationSafetyError(
                    "%s G38 attempt %d failed and contact release could not be "
                    "verified; refusing retry: %s" %
                    (label, attempt + 1, str(error)))
            if contact_active:
                raise NeedleCalibrationSafetyError(
                    "%s G38 attempt %d failed but contact remains active or "
                    "unstable; refusing retry: %s" %
                    (label, attempt + 1, str(last_error)))
            log_msg(logger, "%s G38 attempt %d/%d failed: %s; retrying from restored XY" %
                (label, attempt + 1, attempts, str(last_error)))
    raise last_error


def _radial_contact(start_x, start_y, ux, uy, contact):
    inward = ((float(start_x) - float(contact.getX())) * float(ux) +
        (float(start_y) - float(contact.getY())) * float(uy))
    projected_x = float(start_x) - float(ux) * inward
    projected_y = float(start_y) - float(uy) * inward
    cross_track = math.sqrt(math.pow(float(contact.getX()) - projected_x, 2.0) +
        math.pow(float(contact.getY()) - projected_y, 2.0))
    return {"travel_mm": inward, "cross_track_error_mm": cross_track,
        "x": projected_x, "y": projected_y,
        "reported": loc_to_dict(contact)}


def _validate_radial_contact(sample, max_travel, tolerance, label,
        expected_travel=None, min_travel=0.01):
    travel = float(sample.get("travel_mm", 0.0))
    cross_track = float(sample.get("cross_track_error_mm", 0.0))
    tolerance = max(0.02, abs(float(tolerance)))
    if travel < float(min_travel):
        raise NeedleCalibrationError(
            "%s reported only %.6f mm inward travel; minimum allowed is %.6f mm; refusing a stale/active-start contact" %
            (label, travel, float(min_travel)))
    if travel > float(max_travel) + tolerance:
        raise NeedleCalibrationError(
            "%s reported travel %.6f beyond commanded %.6f mm" %
            (label, travel, float(max_travel)))
    if cross_track > tolerance:
        raise NeedleCalibrationError(
            "%s reported %.6f mm cross-track error (limit %.6f mm)" %
            (label, cross_track, tolerance))
    if expected_travel is not None and abs(travel - float(expected_travel)) > tolerance:
        raise NeedleCalibrationError(
            "%s contact travel %.6f is not repeatable with expected %.6f +/- %.6f mm" %
            (label, travel, float(expected_travel), tolerance))
    return sample


def _tightest_repeatable_cluster(samples, required_count, tolerance):
    """Return the tightest travel cluster without treating the first hit as truth."""
    required_count = int(required_count)
    if len(samples) < required_count:
        return None
    ordered = sorted(samples, key=lambda item: float(item.get("travel_mm", 0.0)))
    best = None
    best_spread = None
    for start in range(len(ordered) - required_count + 1):
        candidate = ordered[start:start + required_count]
        spread = (float(candidate[-1].get("travel_mm", 0.0)) -
            float(candidate[0].get("travel_mm", 0.0)))
        if spread <= abs(float(tolerance)) and (
                best is None or spread < best_spread):
            best = candidate
            best_spread = spread
    return best


def probe_post_edge_hardware(cfg, nozzle, actuator, center, angle_deg,
        configured_radius, z_probe, logger=None, effective_start_radius=None):
    post = cfg.get("post_xy", {})
    theta = math.radians(float(angle_deg))
    ux = math.cos(theta)
    uy = math.sin(theta)
    lift = abs(float(post.get("local_lift_mm", 0.60)))
    coarse_feed = _positive_feed(post.get("hardware_coarse_feed", 3000.0),
        "post_xy.hardware_coarse_feed")
    fine_feed = _positive_feed(post.get("hardware_fine_feed", 500.0),
        "post_xy.hardware_fine_feed")
    coarse_count = _repeat_count(post.get("hardware_coarse_repeats", 1),
        "post_xy.hardware_coarse_repeats")
    fine_count = _repeat_count(post.get("hardware_fine_repeats", 2),
        "post_xy.hardware_fine_repeats")
    fine_retry_count = _repeat_count(
        post.get("hardware_fine_retry_attempts", 2),
        "post_xy.hardware_fine_retry_attempts", minimum=0)
    start_dist = _positive_feed(post.get("hardware_start_radius_mm", 4.175),
        "post_xy.hardware_start_radius_mm")
    requested_start_dist = start_dist
    if effective_start_radius is not None:
        start_dist = _positive_feed(effective_start_radius,
            "effective hardware start radius")
    travel_limit = _positive_feed(
        post.get("hardware_coarse_max_travel_mm", 7.0),
        "post_xy.hardware_coarse_max_travel_mm")
    min_coarse_travel = _positive_feed(
        post.get("hardware_min_coarse_travel_mm", 0.25),
        "post_xy.hardware_min_coarse_travel_mm")
    fine_backoff = _positive_feed(post.get("hardware_fine_retract_mm", 1.0),
        "post_xy.hardware_fine_retract_mm")
    fine_travel = _positive_feed(post.get("hardware_fine_max_travel_mm", 2.0),
        "post_xy.hardware_fine_max_travel_mm")
    if fine_travel <= fine_backoff:
        raise NeedleCalibrationError(
            "post_xy.hardware_fine_max_travel_mm must exceed hardware_fine_retract_mm")
    driver = _driver_for_probe(actuator)
    # This is deliberately an absolute center-to-nozzle distance. Hardware
    # probing must not silently combine polling clearance/overtravel values.
    start_x = float(center["x"]) + ux * start_dist
    start_y = float(center["y"]) + uy * start_dist
    rotation = get_movable_location(nozzle).getRotation()
    label = "EDGE angle=%s" % str(angle_deg)
    start_ms = System.currentTimeMillis()

    contact_tolerance = abs(float(post.get("hardware_repeat_tolerance_mm", 0.15)))
    log_msg(logger, "%s hardware start=%s startRadius=%s requestedRadius=%s vector=(%s,%s) coarseMax=%s" %
        (label, fmt_xy(start_x, start_y), fmt_float(start_dist),
         fmt_float(requested_start_dist),
         fmt_float(-ux * travel_limit), fmt_float(-uy * travel_limit),
         fmt_float(travel_limit)))

    move_safe(nozzle, mm_loc(start_x, start_y, z_probe + lift, rotation))
    move_direct_speed(nozzle, mm_loc(start_x, start_y, z_probe, rotation),
        cfg.get("travel_speed", 1.0))
    prepared = _prepare_hardware_start(cfg, nozzle, actuator, post, center,
        angle_deg, z_probe, start_x, start_y, ux, uy, start_dist,
        cfg.get("travel_speed", 1.0), logger, label)
    if prepared["outward_recovery_mm"] > 0.0:
        recovered = prepared["location"]
        start_x = float(recovered.getX())
        start_y = float(recovered.getY())
        start_dist = float(prepared["start_radius_mm"])
        # The contact envelope is fixed in machine coordinates; moving the
        # released start outward increases the required inward travel by the
        # same bounded amount. Keep the original coarse limit as the base.
        travel_limit += float(prepared["outward_recovery_mm"])
        log_msg(logger,
            "%s released-start recovery outward=%s startRadius=%s "
            "effectiveCoarseTravel=%s" %
            (label, fmt_float(prepared["outward_recovery_mm"]),
             fmt_float(start_dist), fmt_float(travel_limit)))
    elif prepared.get("limit_reason") is not None:
        log_msg(logger, "%s start envelope limited by soft limit: %s" %
            (label, prepared.get("limit_reason")))
    coarse_samples = []
    coarse_start_recoveries = 0
    coarse_start_recovery_limit = _repeat_count(
        post.get("hardware_start_recovery_attempts", 1),
        "post_xy.hardware_start_recovery_attempts", minimum=0)
    for index in range(coarse_count):
        sample_label = "%s coarse %d/%d" % (label, index + 1, coarse_count)
        while True:
            move_direct_speed(nozzle, mm_loc(start_x, start_y, z_probe, rotation),
                cfg.get("travel_speed", 1.0))
            prepared = _prepare_hardware_start(cfg, nozzle, actuator, post, center,
                angle_deg, z_probe, start_x, start_y, ux, uy, start_dist,
                cfg.get("travel_speed", 1.0), logger, sample_label)
            if prepared["outward_recovery_mm"] > 0.0:
                recovered = prepared["location"]
                start_x = float(recovered.getX())
                start_y = float(recovered.getY())
                start_dist = float(prepared["start_radius_mm"])
                travel_limit += float(prepared["outward_recovery_mm"])
                log_msg(logger, "%s coarse start recovery outward=%s "
                    "effectiveTravel=%s" %
                    (label, fmt_float(prepared["outward_recovery_mm"]),
                     fmt_float(travel_limit)))
            reported = _run_xy_interrupt_once(nozzle, driver, actuator, post,
                -ux * travel_limit, -uy * travel_limit, coarse_feed, logger,
                sample_label)
            sample = _radial_contact(start_x, start_y, ux, uy, reported)
            try:
                validated = _validate_radial_contact(sample, travel_limit,
                    contact_tolerance, sample_label, min_travel=min_coarse_travel)
            except NeedleCalibrationError:
                near_start = (float(sample.get("travel_mm", 0.0)) <
                    float(min_coarse_travel) and
                    float(sample.get("cross_track_error_mm", 0.0)) <=
                    max(0.02, contact_tolerance))
                if not near_start or coarse_start_recoveries >= coarse_start_recovery_limit:
                    raise
                coarse_start_recoveries += 1
                required_outward = (float(min_coarse_travel) -
                    float(sample.get("travel_mm", 0.0)) + 0.01)
                log_msg(logger,
                    "%s reported near-zero coarse travel %.6f mm; "
                    "lifting and attempting bounded outward recovery %.6f mm (%d/%d)" %
                    (sample_label, float(sample.get("travel_mm", 0.0)),
                     required_outward,
                     coarse_start_recoveries, coarse_start_recovery_limit))
                current = get_movable_location(nozzle)
                move_direct_speed(nozzle, mm_loc(current.getX(), current.getY(),
                    z_probe + lift, current.getRotation()),
                    cfg.get("travel_speed", 1.0))
                prepared = _prepare_hardware_start(cfg, nozzle, actuator, post,
                    center, angle_deg, z_probe, start_x, start_y, ux, uy,
                    start_dist, cfg.get("travel_speed", 1.0), logger,
                    sample_label + " near-zero recovery",
                    minimum_outward_mm=required_outward)
                forced_outward = float(prepared["outward_recovery_mm"])
                if forced_outward + 1.0e-9 < required_outward:
                    raise NeedleCalibrationError(
                        "%s: bounded recovery moved %.6f mm, less than the "
                        "%.6f mm required to clear minimum coarse travel" %
                        (sample_label, forced_outward, required_outward))
                recovered = prepared["location"]
                start_x = float(recovered.getX())
                start_y = float(recovered.getY())
                start_dist = float(prepared["start_radius_mm"])
                travel_limit += forced_outward
                log_msg(logger, "%s forced outward retry radius=%s "
                    "effectiveTravel=%s" % (sample_label,
                    fmt_float(start_dist), fmt_float(travel_limit)))
                move_direct_speed(nozzle, mm_loc(start_x, start_y,
                    z_probe + lift, rotation), cfg.get("travel_speed", 1.0))
                move_direct_speed(nozzle, mm_loc(start_x, start_y, z_probe,
                    rotation), cfg.get("travel_speed", 1.0))
                continue
            coarse_samples.append(validated)
            break
        log_msg(logger, "%s validated travel=%s crossTrack=%s" %
            (sample_label, fmt_float(sample["travel_mm"]),
             fmt_float(sample["cross_track_error_mm"])))
    coarse_travel = sum([item["travel_mm"] for item in coarse_samples]) / float(coarse_count)
    coarse_x = start_x - ux * coarse_travel
    coarse_y = start_y - uy * coarse_travel

    coarse_radius = ((coarse_x - float(center["x"])) * ux +
        (coarse_y - float(center["y"])) * uy)
    requested_fine_radius = coarse_radius + fine_backoff
    effective_fine_radius, fine_limit_reason = _bounded_hardware_start_radius(
        nozzle, center, angle_deg, requested_fine_radius, z_probe)
    effective_fine_backoff = effective_fine_radius - coarse_radius
    if effective_fine_backoff <= 0.01:
        raise NeedleCalibrationError(
            "%s has no reachable room to retract outward for the fine probe" % label)
    if fine_limit_reason is not None:
        log_msg(logger,
            "%s fine retract clamped requested=%s effective=%s reason=%s" %
            (label, fmt_float(fine_backoff),
             fmt_float(effective_fine_backoff), fine_limit_reason))
    fine_backoff = effective_fine_backoff
    fine_start_x = float(center["x"]) + ux * effective_fine_radius
    fine_start_y = float(center["y"]) + uy * effective_fine_radius
    log_msg(logger, "%s coarseAverage=%s fineStart=%s nominalFineBackoff=%s" %
        (label, fmt_xy(coarse_x, coarse_y), fmt_xy(fine_start_x, fine_start_y),
         fmt_float(fine_backoff)))
    move_direct_speed(nozzle, mm_loc(fine_start_x, fine_start_y, z_probe,
        rotation), cfg.get("travel_speed", 1.0))
    fine_prepared = _prepare_hardware_start(cfg, nozzle, actuator, post, center,
        angle_deg, z_probe, fine_start_x, fine_start_y, ux, uy,
        effective_fine_radius, cfg.get("travel_speed", 1.0), logger,
        label + " fine")
    if fine_prepared["outward_recovery_mm"] > 0.0:
        recovered = fine_prepared["location"]
        fine_start_x = float(recovered.getX())
        fine_start_y = float(recovered.getY())
        effective_fine_radius = float(fine_prepared["start_radius_mm"])
        fine_backoff += float(fine_prepared["outward_recovery_mm"])
        fine_travel += float(fine_prepared["outward_recovery_mm"])
        log_msg(logger,
            "%s fine released-start recovery outward=%s startRadius=%s "
            "effectiveFineTravel=%s" %
            (label, fmt_float(fine_prepared["outward_recovery_mm"]),
             fmt_float(effective_fine_radius), fmt_float(fine_travel)))
    fine_samples = None
    fine_candidates = []
    fine_rejected_samples = []
    fine_attempt = 0
    fine_attempt_limit = fine_count + fine_retry_count
    while fine_samples is None and fine_attempt < fine_attempt_limit:
        fine_attempt += 1
        if fine_attempt > 1:
            current = get_movable_location(nozzle)
            if float(current.getZ()) < float(z_probe) + lift - 1.0e-9:
                move_direct_speed(nozzle, mm_loc(current.getX(), current.getY(),
                    z_probe + lift, current.getRotation()),
                    cfg.get("travel_speed", 1.0))
        move_direct_speed(nozzle, mm_loc(fine_start_x, fine_start_y, z_probe, rotation),
            cfg.get("travel_speed", 1.0))
        sample_label = "%s fine candidate attempt %d/%d" % (
            label, fine_attempt, fine_attempt_limit)
        fine_prepared = _prepare_hardware_start(cfg, nozzle, actuator, post,
            center, angle_deg, z_probe, fine_start_x, fine_start_y, ux, uy,
            effective_fine_radius, cfg.get("travel_speed", 1.0), logger,
            sample_label)
        if fine_prepared["outward_recovery_mm"] > 0.0:
            recovered = fine_prepared["location"]
            fine_start_x = float(recovered.getX())
            fine_start_y = float(recovered.getY())
            effective_fine_radius = float(fine_prepared["start_radius_mm"])
            fine_backoff += float(fine_prepared["outward_recovery_mm"])
            fine_travel += float(fine_prepared["outward_recovery_mm"])
            log_msg(logger, "%s start recovery outward=%s effectiveFineTravel=%s" %
                (sample_label, fmt_float(fine_prepared["outward_recovery_mm"]),
                 fmt_float(fine_travel)))
        try:
            reported = _run_xy_interrupt_once(nozzle, driver, actuator, post,
                -ux * fine_travel, -uy * fine_travel, fine_feed, logger,
                sample_label)
            sample = _radial_contact(
                fine_start_x, fine_start_y, ux, uy, reported)
            validated = _validate_radial_contact(sample, fine_travel,
                contact_tolerance, sample_label)
            validated["attempt"] = fine_attempt
            fine_candidates.append(validated)
            log_msg(logger, "%s geometrically valid travel=%s crossTrack=%s" %
                (sample_label, fmt_float(sample["travel_mm"]),
                 fmt_float(sample["cross_track_error_mm"])))
            fine_samples = _tightest_repeatable_cluster(
                fine_candidates, fine_count, contact_tolerance)
            if fine_samples is None:
                log_msg(logger,
                    "%s waiting for %d repeatable contacts; candidate travels=%s tolerance=%s" %
                    (label, fine_count,
                     [fmt_float(item.get("travel_mm")) for item in fine_candidates],
                     fmt_float(contact_tolerance)))
        except NeedleCalibrationSafetyError:
            raise
        except Throwable as error:
            if "abort" in str(error).lower():
                raise
            fine_rejected_samples.append({"attempt": fine_attempt,
                "reason": str(error)})
            log_msg(logger, "%s REJECTED: %s; discarded (%d retry attempts remain)" %
                (sample_label, str(error), max(0,
                    fine_retry_count - len(fine_rejected_samples))))
        except Exception as error:
            if error.__class__.__name__ == "CalibrationAborted" or \
                    "abort" in str(error).lower():
                raise
            fine_rejected_samples.append({"attempt": fine_attempt,
                "reason": str(error)})
            log_msg(logger, "%s REJECTED: %s; discarded (%d retry attempts remain)" %
                (sample_label, str(error), max(0,
                    fine_retry_count - len(fine_rejected_samples))))
    if fine_samples is None:
        last_reason = (fine_rejected_samples[-1].get("reason")
            if fine_rejected_samples else
            "fine candidates did not form a repeatable cluster")
        raise NeedleCalibrationError(
            "%s could not form a %d-contact fine cluster after %d attempts "
            "(%d retries allowed, tolerance %s); candidate travels=%s; "
            "last rejection: %s" % (
                label, fine_count, fine_attempt, fine_retry_count,
                fmt_float(contact_tolerance),
                [fmt_float(item.get("travel_mm")) for item in fine_candidates],
                last_reason))
    selected_attempts = set([int(item.get("attempt")) for item in fine_samples])
    for candidate in fine_candidates:
        if int(candidate.get("attempt")) not in selected_attempts:
            fine_rejected_samples.append({
                "attempt": int(candidate.get("attempt")),
                "travel_mm": float(candidate.get("travel_mm")),
                "reason": "outside selected fine-contact cluster"})
    log_msg(logger, "%s accepted fine cluster attempts=%s travels=%s spread=%s" %
        (label, sorted(selected_attempts),
         [fmt_float(item.get("travel_mm")) for item in fine_samples],
         fmt_float(max([item["travel_mm"] for item in fine_samples]) -
             min([item["travel_mm"] for item in fine_samples]))))
    fine_travel_average = sum([item["travel_mm"] for item in fine_samples]) / float(fine_count)
    contact_x = fine_start_x - ux * fine_travel_average
    contact_y = fine_start_y - uy * fine_travel_average
    move_direct_speed(nozzle, mm_loc(contact_x, contact_y, z_probe + lift, rotation),
        cfg.get("travel_speed", 1.0))
    elapsed = System.currentTimeMillis() - start_ms
    contact_dict = {"x": contact_x, "y": contact_y, "z": float(z_probe),
        "rotation": float(rotation)}
    return {"angle_deg": float(angle_deg), "x": contact_x, "y": contact_y,
        "z_probe_mm": float(z_probe), "center_used": deep_copy(center),
        "configured_post_radius_mm": float(configured_radius),
        "start": {"x": start_x, "y": start_y}, "unit": {"x": ux, "y": uy},
        "coarse_samples": coarse_samples, "coarse_travel_average_mm": coarse_travel,
        "fine_samples": fine_samples, "fine_travel_average_mm": fine_travel_average,
        "fine_attempt_count": fine_attempt,
        "fine_rejected_samples": fine_rejected_samples,
        "confirmed_contact": contact_dict, "contact": contact_dict,
        "steps": coarse_count + fine_attempt, "elapsed_ms": int(elapsed),
        "source": "hardware_interrupt"}


def probe_post_edge(cfg, nozzle, actuator, center, angle_deg, configured_radius,
        z_probe, logger=None):
    post = cfg.get("post_xy", {})
    read_cfg = deep_copy(post)
    settings = post_xy_settings(cfg)
    scan = post_scan_settings(cfg)
    theta = math.radians(float(angle_deg))
    ux = math.cos(theta)
    uy = math.sin(theta)

    clearance = scan["clearance_mm"]
    overtravel = scan["overtravel_mm"]
    cap = scan["cap_mm"]
    lift = abs(float(post.get("local_lift_mm", 0.60)))

    step = float(settings["step_mm"])
    speed = float(settings["speed"])

    start_dist = float(configured_radius) + clearance
    start_x = float(center["x"]) + ux * start_dist
    start_y = float(center["y"]) + uy * start_dist

    travel_limit = scan["travel_limit_mm"]

    label = "EDGE angle=%s" % str(angle_deg)
    start_ms = System.currentTimeMillis()

    log_msg(logger, "%s START center=%s start=%s radius=%s clearance=%s overtravel=%s cap=%s travelLimit=%s z=%s step=%s speed=%s" %
        (label,
         fmt_xy(center["x"], center["y"]),
         fmt_xy(start_x, start_y),
         fmt_float(configured_radius),
         fmt_float(clearance),
         fmt_float(overtravel),
         fmt_float(cap),
         fmt_float(travel_limit),
         fmt_float(z_probe),
         fmt_float(step),
         fmt_float(speed)))

    move_safe(nozzle, mm_loc(start_x, start_y, z_probe + lift,
        get_movable_location(nozzle).getRotation()))

    move_direct_speed(nozzle, mm_loc(start_x, start_y, z_probe,
        get_movable_location(nozzle).getRotation()), speed)

    recovered, outward_recovery = recover_outside_start_if_touching(nozzle,
        actuator, read_cfg, start_x, start_y, ux, uy, z_probe, speed, logger, label)
    if outward_recovery > 0.0:
        start_x = float(recovered.getX())
        start_y = float(recovered.getY())
        travel_limit = min(cap + outward_recovery,
            clearance + overtravel + outward_recovery)
        log_msg(logger, "%s recovered start=%s outward=%s effectiveTravelLimit=%s" %
            (label, fmt_xy(start_x, start_y), fmt_float(outward_recovery),
             fmt_float(travel_limit)))

    first_contact, travel, steps = move_inward_until_contact(nozzle, actuator,
        read_cfg, start_x, start_y, ux, uy, travel_limit, step, z_probe,
        speed, logger, label)

    if first_contact is None:
        cur = get_movable_location(nozzle)
        move_direct_speed(nozzle, mm_loc(cur.getX(), cur.getY(), z_probe + lift,
            cur.getRotation()), speed)
        raise NeedleCalibrationError(
            "%s: no contact after moving inward %.6f mm from start %s toward center %s" %
            (label, float(travel_limit), fmt_xy(start_x, start_y),
             fmt_xy(center["x"], center["y"])))

    log_msg(logger, "%s FIRST CONTACT pos=%s travel=%s steps=%d" %
        (label, fmt_xy(first_contact.getX(), first_contact.getY()),
         fmt_float(travel), steps))

    try:
        contact, confirm_travel, confirm_steps, backoff_location = confirm_contact_point(
            cfg, nozzle, actuator, read_cfg, first_contact, ux, uy, z_probe,
            speed, logger, label)
    except Throwable as e:
        raise NeedleCalibrationError("%s failed confirmation: %s" %
            (label, str(e)))
    except Exception as e:
        raise NeedleCalibrationError("%s failed confirmation: %s" %
            (label, str(e)))

    elapsed = System.currentTimeMillis() - start_ms

    log_msg(logger, "%s CONFIRMED CONTACT pos=%s firstTravel=%s confirmTravel=%s" %
        (label, fmt_xy(contact.getX(), contact.getY()),
         fmt_float(travel), fmt_float(confirm_travel)))

    move_direct_speed(nozzle, mm_loc(contact.getX(), contact.getY(),
        z_probe + lift, contact.getRotation()), speed)
    log_msg(logger, "%s DONE using confirmed contact" % label)

    return {
        "angle_deg": float(angle_deg),
        "x": float(contact.getX()),
        "y": float(contact.getY()),
        "z_probe_mm": float(z_probe),
        "center_used": {"x": float(center["x"]), "y": float(center["y"])},
        "configured_post_radius_mm": float(configured_radius),
        "start": {"x": start_x, "y": start_y},
        "unit": {"x": ux, "y": uy},
        "first_contact": loc_to_dict(first_contact),
        "confirmed_contact": loc_to_dict(contact),
        "contact": loc_to_dict(contact),
        "first_travel_mm": float(travel),
        "confirm_backoff_mm": abs(float(read_cfg.get("confirm_backoff_mm", 0.4))),
        "confirm_reapproach_mm": abs(float(read_cfg.get("confirm_reapproach_mm", 0.8))),
        "confirm_position_tolerance_mm": abs(float(read_cfg.get("confirm_position_tolerance_mm", 0.15))),
        "confirm_reject_early": config_bool(read_cfg.get("confirm_reject_early", True), True),
        "confirm_travel_mm": float(confirm_travel),
        "confirm_steps": int(confirm_steps),
        "backoff_location": loc_to_dict(backoff_location),
        "travel_mm": float(confirm_travel),
        "outward_recovery_mm": float(outward_recovery),
        "effective_travel_limit_mm": float(travel_limit),
        "steps": int(steps),
        "elapsed_ms": int(elapsed)
    }


def contact_by_angle(contacts):
    out = {}
    for p in contacts:
        try:
            a = float(p.get("angle_deg"))
        except:
            continue
        a = a % 360.0
        key = int(round(a))
        out[key] = p
    return out


def pair_midpoint_debug(contacts, vision_center=None):
    by_angle = contact_by_angle(contacts)
    pairs = [(0, 180), (45, 225), (90, 270), (135, 315)]
    rows = []
    sx = 0.0
    sy = 0.0
    count = 0
    for a, b in pairs:
        p1 = by_angle.get(a)
        p2 = by_angle.get(b)
        if p1 is None or p2 is None:
            rows.append({"pair": "%d/%d" % (a, b), "valid": False})
            continue
        x1 = float(p1["x"])
        y1 = float(p1["y"])
        x2 = float(p2["x"])
        y2 = float(p2["y"])
        mx = (x1 + x2) / 2.0
        my = (y1 + y2) / 2.0
        span = math.sqrt(math.pow(x2 - x1, 2.0) + math.pow(y2 - y1, 2.0))
        row = {"pair": "%d/%d" % (a, b), "valid": True,
            "midpoint": {"x": mx, "y": my},
            "span_mm": span,
            "point_a": {"angle_deg": float(a), "x": x1, "y": y1},
            "point_b": {"angle_deg": float(b), "x": x2, "y": y2}}
        rows.append(row)
        sx += mx
        sy += my
        count += 1
    center = None
    delta = None
    if count > 0:
        center = {"x": sx / float(count), "y": sy / float(count)}
        if vision_center is not None:
            delta = {"x": center["x"] - float(vision_center["x"]),
                "y": center["y"] - float(vision_center["y"])}
    return {"pairs": rows, "center": center, "delta_from_vision": delta,
        "pair_count": count}


def _normalized_angle_deg(angle):
    value = float(angle) % 360.0
    if value < 0.0:
        value += 360.0
    return value


def _angle_difference_deg(first, second):
    value = (_normalized_angle_deg(first) - _normalized_angle_deg(second)) % 360.0
    return min(value, 360.0 - value)


def _contact_geometry_diagnostics(contacts, vision_center, max_line_error):
    """Validate the commanded radial lines and report their angular coverage."""
    if len(contacts) < 3:
        return {"ok": False, "message": "at least three valid contacts are required"}
    angles = []
    line_errors = []
    radial_distances = []
    for contact in contacts:
        try:
            angle = _normalized_angle_deg(contact["angle_deg"])
            x = float(contact["x"])
            y = float(contact["y"])
        except Exception as error:
            return {"ok": False, "message": "contact is missing numeric angle/x/y: %s" % error}
        theta = math.radians(angle)
        ux = math.cos(theta)
        uy = math.sin(theta)
        dx = x - float(vision_center["x"])
        dy = y - float(vision_center["y"])
        radial = dx * ux + dy * uy
        cross_track = abs(-uy * dx + ux * dy)
        angles.append(angle)
        radial_distances.append(radial)
        line_errors.append(cross_track)
    unique_angles = []
    for angle in sorted(angles):
        duplicate = False
        for previous in unique_angles:
            if _angle_difference_deg(angle, previous) <= 1.0e-6:
                duplicate = True
                break
        if not duplicate:
            unique_angles.append(angle)
    if len(unique_angles) < 3:
        return {"ok": False, "message": "fewer than three independent contact angles",
            "angles_deg": unique_angles, "line_errors_mm": line_errors}
    gaps = []
    for index in range(len(unique_angles)):
        current = unique_angles[index]
        following = unique_angles[(index + 1) % len(unique_angles)]
        gap = following - current
        if index == len(unique_angles) - 1:
            gap += 360.0
        gaps.append(gap)
    max_gap = max(gaps)
    coverage = 360.0 - max_gap
    max_line = max(line_errors)
    return {"ok": True, "angles_deg": unique_angles,
        "angular_coverage_deg": coverage, "max_angular_gap_deg": max_gap,
        "line_errors_mm": line_errors, "max_line_error_mm": max_line,
        "radial_distances_mm": radial_distances}


def _validate_angle_schedule(angles, post_cfg):
    """Reject an unobservable configured scan before any machine motion."""
    normalized = sorted([_normalized_angle_deg(angle) for angle in angles])
    unique = []
    for angle in normalized:
        if not unique or _angle_difference_deg(angle, unique[-1]) > 1.0e-6:
            unique.append(angle)
    if len(unique) < 3:
        raise NeedleCalibrationError(
            "post_xy angle schedule has fewer than three unique directions")
    gaps = []
    for index in range(len(unique)):
        gap = unique[(index + 1) % len(unique)] - unique[index]
        if index == len(unique) - 1:
            gap += 360.0
        gaps.append(gap)
    coverage = 360.0 - max(gaps)
    min_coverage = abs(float(post_cfg.get("min_angular_coverage_deg", 180.0)))
    max_gap = abs(float(post_cfg.get("max_angular_gap_deg", 180.0)))
    if coverage < min_coverage:
        raise NeedleCalibrationError(
            "post_xy angle schedule covers only %.6f degrees; need %.6f" %
            (coverage, min_coverage))
    if max(gaps) > max_gap:
        raise NeedleCalibrationError(
            "post_xy angle schedule has %.6f degree gap; maximum allowed %.6f" %
            (max(gaps), max_gap))
    return {"angles_deg": unique, "angular_coverage_deg": coverage,
        "max_angular_gap_deg": max(gaps)}


def _symmetric_eigenvalues_3x3(matrix):
    """Return eigenvalues of a symmetric 3x3 matrix without external modules."""
    a = [[float(matrix[row][col]) for col in range(3)] for row in range(3)]
    for unused in range(50):
        p = 0
        q = 1
        largest = abs(a[0][1])
        for first, second in [(0, 2), (1, 2)]:
            if abs(a[first][second]) > largest:
                p = first
                q = second
                largest = abs(a[first][second])
        if largest < 1.0e-12:
            break
        angle = 0.5 * math.atan2(2.0 * a[p][q], a[q][q] - a[p][p])
        cosine = math.cos(angle)
        sine = math.sin(angle)
        app = (cosine * cosine * a[p][p] -
            2.0 * sine * cosine * a[p][q] + sine * sine * a[q][q])
        aqq = (sine * sine * a[p][p] +
            2.0 * sine * cosine * a[p][q] + cosine * cosine * a[q][q])
        for index in range(3):
            if index == p or index == q:
                continue
            aip = a[index][p]
            aiq = a[index][q]
            a[index][p] = cosine * aip - sine * aiq
            a[p][index] = a[index][p]
            a[index][q] = sine * aip + cosine * aiq
            a[q][index] = a[index][q]
        a[p][p] = app
        a[q][q] = aqq
        a[p][q] = 0.0
        a[q][p] = 0.0
    return sorted([a[0][0], a[1][1], a[2][2]])


def _design_condition_number(rows):
    """Condition estimate for a small least-squares design matrix."""
    if not rows:
        return float("inf")
    columns = len(rows[0])
    gram = [[0.0 for unused in range(columns)] for unused in range(columns)]
    for row in rows:
        for first in range(columns):
            for second in range(columns):
                gram[first][second] += float(row[first]) * float(row[second])
    scales = []
    for index in range(columns):
        scales.append(math.sqrt(max(0.0, gram[index][index])))
    for index in range(columns):
        if scales[index] <= 1.0e-12:
            return float("inf")
    normalized = [[0.0 for unused in range(columns)] for unused in range(columns)]
    for first in range(columns):
        for second in range(columns):
            normalized[first][second] = gram[first][second] / (scales[first] * scales[second])
    eigenvalues = _symmetric_eigenvalues_3x3(normalized)
    if eigenvalues[0] <= 1.0e-12:
        return float("inf")
    return math.sqrt(max(eigenvalues[-1], 0.0) / eigenvalues[0])


def _opposite_contact_pairs(contacts):
    """Pair contacts on opposite commanded radial lines, independent of spacing."""
    remaining = list(contacts)
    pairs = []
    while remaining:
        first = remaining.pop(0)
        first_angle = _normalized_angle_deg(first.get("angle_deg"))
        match_index = None
        for index, candidate in enumerate(remaining):
            candidate_angle = _normalized_angle_deg(candidate.get("angle_deg"))
            if abs(_angle_difference_deg(candidate_angle, first_angle) - 180.0) <= 1.0e-6:
                match_index = index
                break
        if match_index is not None:
            pairs.append((first, remaining.pop(match_index)))
    return pairs


def _opposite_pair_projection_estimate(contacts, vision_center, max_condition):
    pairs = _opposite_contact_pairs(contacts)
    if len(pairs) < 2:
        return {"ok": False, "pair_count": len(pairs),
            "message": "fewer than two independent opposite contact pairs"}
    matrix = [[0.0, 0.0], [0.0, 0.0]]
    vector = [0.0, 0.0]
    observations = []
    for first, second in pairs:
        angle = _normalized_angle_deg(first.get("angle_deg"))
        theta = math.radians(angle)
        ux = math.cos(theta)
        uy = math.sin(theta)
        midpoint_x = (float(first["x"]) + float(second["x"])) / 2.0
        midpoint_y = (float(first["y"]) + float(second["y"])) / 2.0
        midpoint_x -= float(vision_center["x"])
        midpoint_y -= float(vision_center["y"])
        projection = ux * midpoint_x + uy * midpoint_y
        matrix[0][0] += ux * ux
        matrix[0][1] += ux * uy
        matrix[1][0] += ux * uy
        matrix[1][1] += uy * uy
        vector[0] += ux * projection
        vector[1] += uy * projection
        observations.append({"angle_deg": angle, "unit": {"x": ux, "y": uy},
            "projection_mm": projection,
            "midpoint": {"x": midpoint_x + float(vision_center["x"]),
                "y": midpoint_y + float(vision_center["y"])}})
    trace = matrix[0][0] + matrix[1][1]
    determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
    discriminant = max(0.0, trace * trace - 4.0 * determinant)
    smallest = (trace - math.sqrt(discriminant)) / 2.0
    largest = (trace + math.sqrt(discriminant)) / 2.0
    condition = float("inf") if smallest <= 1.0e-12 else math.sqrt(largest / smallest)
    if condition > float(max_condition):
        return {"ok": False, "pair_count": len(pairs),
            "condition_number": condition,
            "observations": observations,
            "message": "opposite-pair estimator condition %.6f exceeds %.6f" %
                (condition, float(max_condition))}
    determinant = max(determinant, 1.0e-12)
    dx = (vector[0] * matrix[1][1] - vector[1] * matrix[0][1]) / determinant
    dy = (matrix[0][0] * vector[1] - matrix[1][0] * vector[0]) / determinant
    residuals = []
    for observation in observations:
        residuals.append(observation["projection_mm"] -
            (observation["unit"]["x"] * dx + observation["unit"]["y"] * dy))
    return {"ok": True, "center": {"x": float(vision_center["x"]) + dx,
            "y": float(vision_center["y"]) + dy},
        "displacement": {"x": dx, "y": dy}, "pair_count": len(pairs),
        "condition_number": condition, "residuals_mm": residuals,
        "max_residual_mm": max([abs(item) for item in residuals]),
        "observations": observations}


def estimate_xy_contact_center(contacts, vision_center, post_cfg, circle_fit):
    """Recover the full needle displacement and validate its geometry.

    A midpoint of an opposite radial pair is not the circle center. Its
    projection onto that radial line is only u dot d, where d is the full
    needle-center displacement. Solving those scalar projections recovers d
    without assuming a needle radius. A circle fit is retained as an
    independent diagnostic and is the guarded fallback when no two opposite
    pairs survive.
    """
    max_line_error = abs(float(post_cfg.get("max_contact_line_error_mm", 0.20)))
    geometry = _contact_geometry_diagnostics(contacts, vision_center, max_line_error)
    diagnostics = {"geometry": geometry}
    if not geometry.get("ok", False):
        return {"ok": False, "message": geometry.get("message"),
            "diagnostics": diagnostics}
    min_coverage = abs(float(post_cfg.get("min_angular_coverage_deg", 180.0)))
    max_gap = abs(float(post_cfg.get("max_angular_gap_deg", 180.0)))
    if geometry["angular_coverage_deg"] < min_coverage:
        return {"ok": False,
            "message": "angular coverage %.6f deg is below %.6f deg" %
                (geometry["angular_coverage_deg"], min_coverage),
            "diagnostics": diagnostics}
    if geometry["max_angular_gap_deg"] > max_gap:
        return {"ok": False,
            "message": "largest angular gap %.6f deg exceeds %.6f deg" %
                (geometry["max_angular_gap_deg"], max_gap),
            "diagnostics": diagnostics}
    if geometry["max_line_error_mm"] > max_line_error:
        return {"ok": False,
            "message": "contact line error %.6f exceeds %.6f mm" %
                (geometry["max_line_error_mm"], max_line_error),
            "diagnostics": diagnostics}

    max_condition = abs(float(post_cfg.get("max_geometry_condition", 10.0)))
    pair = _opposite_pair_projection_estimate(contacts, vision_center,
        max_condition)
    diagnostics["opposite_pair_estimator"] = pair
    if circle_fit is not None:
        diagnostics["circle_fit"] = circle_fit
    max_pair_residual = abs(float(post_cfg.get("max_pair_projection_error_mm",
        post_cfg.get("max_center_disagreement_mm", 0.20))))
    if pair.get("ok", False) and pair.get("max_residual_mm", 0.0) > max_pair_residual:
        pair["ok"] = False
        pair["message"] = "opposite-pair projection residual %.6f exceeds %.6f mm" % (
            pair["max_residual_mm"], max_pair_residual)
        return {"ok": False, "message": pair["message"],
            "diagnostics": diagnostics}
    if not pair.get("ok", False) and pair.get("pair_count", 0) >= 2:
        # Two or more opposite pairs are an independent observation of the
        # displacement. If that estimator is ill-conditioned, do not hide the
        # inconsistency behind a circle-only fallback.
        return {"ok": False, "message": pair.get("message",
                "opposite-pair estimator rejected the contact geometry"),
            "diagnostics": diagnostics}

    if circle_fit is None:
        return {"ok": False, "message": "circle fit unavailable for XY estimator",
            "diagnostics": diagnostics}
    max_fit_rms = abs(float(post_cfg.get("max_fit_rms_mm", 0.15)))
    max_fit_error = abs(float(post_cfg.get("max_fit_error_mm", 0.30)))
    if float(circle_fit.get("rms_error_mm", float("inf"))) > max_fit_rms:
        return {"ok": False,
            "message": "circle fit RMS %.6f exceeds %.6f mm" %
                (float(circle_fit.get("rms_error_mm")), max_fit_rms),
            "diagnostics": diagnostics}
    if float(circle_fit.get("max_error_mm", float("inf"))) > max_fit_error:
        return {"ok": False,
            "message": "circle fit maximum error %.6f exceeds %.6f mm" %
                (float(circle_fit.get("max_error_mm")), max_fit_error),
            "diagnostics": diagnostics}
    if float(circle_fit.get("condition_number", float("inf"))) > max_condition:
        return {"ok": False,
            "message": "circle fit condition %.6f exceeds %.6f" %
                (float(circle_fit.get("condition_number")), max_condition),
            "diagnostics": diagnostics}

    source = "circle_fit_fallback"
    center = circle_fit.get("center")
    if pair.get("ok", False):
        pair_center = pair["center"]
        disagreement = math.sqrt(
            math.pow(float(pair_center["x"]) - float(center["x"]), 2.0) +
            math.pow(float(pair_center["y"]) - float(center["y"]), 2.0))
        pair["circle_center_disagreement_mm"] = disagreement
        if disagreement > abs(float(post_cfg.get("max_center_disagreement_mm", 0.20))):
            return {"ok": False,
                "message": "XY estimators disagree by %.6f mm" % disagreement,
                "diagnostics": diagnostics}
        center = pair_center
        source = "opposite_pair_projection_ls"
    else:
        if pair.get("pair_count", 0) > 0:
            diagnostics["pair_fallback_reason"] = pair.get("message",
                "opposite-pair estimator unavailable")
    dx = float(center["x"]) - float(vision_center["x"])
    dy = float(center["y"]) - float(vision_center["y"])
    return {"ok": True, "center": {"x": float(center["x"]),
            "y": float(center["y"])}, "displacement": {"x": dx, "y": dy},
        "source": source, "diagnostics": diagnostics}


def _hardware_xy_start_violation(nozzle, center, angle, radius, z_probe):
    """Return a soft-limit violation for a radial start, or None."""
    rotation = get_movable_location(nozzle).getRotation()
    theta = math.radians(float(angle))
    x = float(center["x"]) + math.cos(theta) * float(radius)
    y = float(center["y"]) + math.sin(theta) * float(radius)
    target = mm_loc(x, y, z_probe, rotation)
    try:
        raw = nozzle.toRaw(nozzle.toHeadLocation(target))
    except Exception as e:
        raise NeedleCalibrationError(
            "Cannot transform hardware start for angle %s: %s" %
            (str(angle), str(e)))

    def read_xy_api(description, callback):
        try:
            return callback()
        except Throwable as error:
            raise NeedleCalibrationSafetyError(
                "%s for angle %s: %s" % (description, str(angle), str(error)))
        except Exception as error:
            raise NeedleCalibrationSafetyError(
                "%s for angle %s: %s" % (description, str(angle), str(error)))

    for axis in java_list_to_py(raw.getAxes()):
        axis_type = read_xy_api(
            "Cannot determine axis type while checking hardware start",
            axis.getType)
        if axis_type not in [Axis.Type.X, Axis.Type.Y]:
            continue
        low_enabled = bool(read_xy_api(
            "Cannot read XY soft-limit enable state",
            axis.isSoftLimitLowEnabled))
        high_enabled = bool(read_xy_api(
            "Cannot read XY soft-limit enable state",
            axis.isSoftLimitHighEnabled))
        if not low_enabled and not high_enabled:
            continue
        coordinate = read_xy_api(
            "Cannot read enabled XY soft-limit coordinate",
            lambda: float(raw.getCoordinate(axis)))
        if math.isnan(coordinate) or math.isinf(coordinate):
            raise NeedleCalibrationSafetyError(
                "Enabled XY soft-limit coordinate is not finite for angle %s" %
                str(angle))
        if low_enabled:
            low = read_xy_api(
                "Cannot read enabled XY low soft limit",
                lambda: float(axis.getSoftLimitLow().convertToUnits(
                    AxesLocation.getUnits()).getValue()))
            if math.isnan(low) or math.isinf(low):
                raise NeedleCalibrationSafetyError(
                    "Enabled XY low soft limit is not finite for angle %s" %
                    str(angle))
            if coordinate < low - 1.0e-6:
                return "%s=%s below soft limit %s" % (
                    str(axis.getName()), fmt_float(coordinate), fmt_float(low))
        if high_enabled:
            high = read_xy_api(
                "Cannot read enabled XY high soft limit",
                lambda: float(axis.getSoftLimitHigh().convertToUnits(
                    AxesLocation.getUnits()).getValue()))
            if math.isnan(high) or math.isinf(high):
                raise NeedleCalibrationSafetyError(
                    "Enabled XY high soft limit is not finite for angle %s" %
                    str(angle))
            if coordinate > high + 1.0e-6:
                return "%s=%s above soft limit %s" % (
                    str(axis.getName()), fmt_float(coordinate), fmt_float(high))
    return None


def _bounded_hardware_start_radius(nozzle, center, angle, requested_radius,
        z_probe):
    """Clamp a requested radial start to the reachable machine envelope."""
    requested = float(requested_radius)
    violation = _hardware_xy_start_violation(nozzle, center, angle, requested,
        z_probe)
    if violation is None:
        return requested, None
    center_violation = _hardware_xy_start_violation(nozzle, center, angle, 0.0,
        z_probe)
    if center_violation is not None:
        raise NeedleCalibrationError(
            "Post center is outside the machine envelope at angle %s: %s" %
            (str(angle), center_violation))
    low = 0.0
    high = requested
    for unused in range(48):
        middle = (low + high) / 2.0
        if _hardware_xy_start_violation(nozzle, center, angle, middle,
                z_probe) is None:
            low = middle
        else:
            high = middle
    # Stay just inside the exact boundary to avoid rounding/tolerance failures.
    effective = max(0.0, low - 0.01)
    if effective <= 0.0:
        raise NeedleCalibrationError(
            "No reachable outward hardware start at angle %s: %s" %
            (str(angle), violation))
    return effective, violation


def _prepare_hardware_start(cfg, nozzle, actuator, post, center, angle, z_probe,
        start_x, start_y, ux, uy, start_radius, speed, logger, label,
        minimum_outward_mm=0.0):
    """Require a released sensor before issuing a hardware G38 move."""
    max_outward = abs(float(post.get("outside_recover_max_mm", 3.0)))
    if max_outward <= 0.0:
        raise NeedleCalibrationError(
            "%s: outside_recover_max_mm must be > 0 for hardware start safety" % label)
    max_safe_radius, limit_reason = _bounded_hardware_start_radius(nozzle,
        center, angle, float(start_radius) + max_outward, z_probe)
    recovered, outward = recover_outside_start_if_touching(nozzle, actuator,
        post, start_x, start_y, ux, uy, z_probe, speed, logger, label,
        start_radius_mm=start_radius, max_start_radius_mm=max_safe_radius,
        minimum_outward_mm=minimum_outward_mm)
    if outward <= 0.0:
        return {"location": recovered, "start_radius_mm": float(start_radius),
            "outward_recovery_mm": 0.0, "max_safe_radius_mm": max_safe_radius,
            "limit_reason": limit_reason}
    return {"location": recovered,
        "start_radius_mm": float(start_radius) + float(outward),
        "outward_recovery_mm": float(outward),
        "max_safe_radius_mm": max_safe_radius, "limit_reason": limit_reason}


def probe_8_point_post_xy(cfg, nozzle, actuator, vision_center, configured_radius,
        z_probe, logger=None):
    post = cfg.get("post_xy", {})
    angles = parse_angles(post.get("angles_deg"),
        [0, 45, 90, 135, 180, 225, 270, 315])
    if len(angles) != 8:
        raise NeedleCalibrationError("Post XY requires exactly 8 angles; got %d" %
            len(angles))
    angle_schedule = _validate_angle_schedule(angles, post)
    acquisition = str(post.get("acquisition_method", "polling") or
        "polling").strip().lower()
    effective_start_radii = {}
    if acquisition == "hardware_interrupt":
        requested_radius = _positive_feed(
            post.get("hardware_start_radius_mm", 4.175),
            "post_xy.hardware_start_radius_mm")
        for angle in angles:
            effective, violation = _bounded_hardware_start_radius(nozzle,
                vision_center, angle, requested_radius, z_probe)
            effective_start_radii[float(angle)] = effective
            if violation is not None:
                log_msg(logger,
                    "EDGE angle=%s start radius clamped requested=%s effective=%s reason=%s" %
                    (str(angle), fmt_float(requested_radius),
                     fmt_float(effective), violation))
    all_edges = []
    valid_contacts = []
    invalid_edges = []
    try:
        for angle in angles:
            try:
                if acquisition == "hardware_interrupt":
                    edge = probe_post_edge_hardware(cfg, nozzle, actuator,
                        vision_center, angle, configured_radius, z_probe, logger,
                        effective_start_radii.get(float(angle)))
                elif acquisition == "polling":
                    edge = probe_post_edge(cfg, nozzle, actuator, vision_center,
                        angle, configured_radius, z_probe, logger)
                else:
                    raise NeedleCalibrationError(
                        "post_xy.acquisition_method must be polling or hardware_interrupt")
                edge["valid"] = True
                all_edges.append(edge)
                valid_contacts.append(edge)
            except NeedleCalibrationError as e:
                if acquisition == "hardware_interrupt":
                    # A direct G38 failure can mean a stale/active interrupt or
                    # unsafe controller state. Never continue around the post.
                    try:
                        lift_nozzle_to_safe_z(nozzle, cfg, logger)
                    except:
                        pass
                    raise
                text = str(e)
                stage = "edge"
                lower = text.lower()
                if "confirm" in lower:
                    stage = "confirmation"
                elif "outside" in lower or "recover" in lower:
                    stage = "outside_recovery"
                elif "no contact" in lower:
                    stage = "first_contact"
                invalid = {"angle_deg": float(angle), "valid": False,
                    "error": text, "stage": stage}
                all_edges.append(invalid)
                invalid_edges.append(invalid)
                log_msg(logger, "EDGE angle=%s INVALID reason=%s" %
                    (str(angle), text))
                try:
                    lift_nozzle_to_safe_z(nozzle, cfg, logger)
                except:
                    pass
        try:
            lift_nozzle_to_safe_z(nozzle, cfg, logger)
        except:
            pass
    except:
        try:
            lift_nozzle_to_safe_z(nozzle, cfg, logger)
        except:
            pass
        raise
    min_valid = int(float(post.get("min_valid_contacts_for_fit", 5)))
    if min_valid < 3:
        min_valid = 3
    fit = None
    if len(valid_contacts) >= min_valid:
        fit = circle_fit_contacts(valid_contacts, vision_center)
    return {"angles": angles, "all_edges": all_edges,
        "angle_schedule": angle_schedule,
        "contacts": valid_contacts, "invalid_edges": invalid_edges,
        "valid_contact_count": len(valid_contacts),
        "invalid_contact_count": len(invalid_edges),
        "min_valid_contacts_for_fit": min_valid,
        "fit": fit}



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


def circle_fit_contacts(contacts, reference_center=None):
    if len(contacts) < 3:
        raise NeedleCalibrationError("Circle fit requires at least three contacts")
    if reference_center is None:
        reference_x = sum([float(p["x"]) for p in contacts]) / float(len(contacts))
        reference_y = sum([float(p["y"]) for p in contacts]) / float(len(contacts))
    else:
        reference_x = float(reference_center["x"])
        reference_y = float(reference_center["y"])
    ata = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    atb = [0.0, 0.0, 0.0]
    design_rows = []
    for p in contacts:
        x = float(p["x"]) - reference_x
        y = float(p["y"]) - reference_y
        row = [x, y, 1.0]
        b = -(x * x + y * y)
        design_rows.append(row)
        for r in range(3):
            atb[r] += row[r] * b
            for c in range(3):
                ata[r][c] += row[r] * row[c]
    a, b, c = solve_3x3(ata, atb)
    cx = reference_x - a / 2.0
    cy = reference_y - b / 2.0
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
    return {
        "center": {"x": cx, "y": cy},
        "radius_mm": radius,
        "rms_error_mm": math.sqrt(sum_sq / float(len(contacts))),
        "max_error_mm": max_err,
        "residuals_mm": residuals,
        "point_count": len(contacts),
        "reference_center": {"x": reference_x, "y": reference_y},
        "condition_number": _design_condition_number(design_rows)
    }


def fail_payload(stage, message, nozzle=None, cfg=None, logger=None, old_offsets=None):
    log_msg(logger, "Needle calibration failed: %s" % message)
    if nozzle is not None and cfg is not None:
        try:
            lift_nozzle_to_safe_z(nozzle, cfg, logger)
        except:
            pass
    current = None
    try:
        if nozzle is not None:
            current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
    except:
        current = old_offsets
    return {"ok": False, "stage": stage, "message": str(message),
        "old_offsets": old_offsets, "current_offsets": current,
        "proposal": None, "details": {}}


OFFSET_VERIFY_TOLERANCE_MM = 0.000001


def offsets_match(actual, expected, tolerance=OFFSET_VERIFY_TOLERANCE_MM):
    if actual is None or expected is None:
        return False
    for axis in ["x", "y", "z", "rotation"]:
        try:
            if abs(float(actual.get(axis, 0.0)) -
                    float(expected.get(axis, 0.0))) > float(tolerance):
                return False
        except:
            return False
    return True


def synchronize_coordinate_system(nozzle, machine_obj=None, logger=None):
    """Wait until OpenPnP and the driver agree after a head-offset change."""
    wait_still(nozzle)
    machine_obj = get_machine(machine_obj)
    try:
        planner = machine_obj.getMotionPlanner()
    except:
        planner = None
    if planner is not None:
        try:
            planner.waitForCompletion(None, CompletionType.WaitForStillstand)
        except TypeError:
            planner.waitForCompletion(CompletionType.WaitForStillstand)
    # Reading the transformed location after the planner barrier forces the
    # active nozzle/head transform to be observed before the next move.
    location = get_movable_location(nozzle)
    log_msg(logger, "Coordinate system synchronized at %s" % loc_to_log(location))
    return location


def set_offsets_and_verify(nozzle, offsets, logger=None):
    set_offsets_from_dict(nozzle, offsets)
    actual = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(
        LengthUnit.Millimeters))
    if not offsets_match(actual, offsets):
        raise NeedleCalibrationError(
            "Head offsets did not verify after set: expected=%s actual=%s" %
            (offsets, actual))
    return actual


def restore_offsets_after_safe_retraction(cfg, nozzle, offsets, machine_obj=None,
        logger=None):
    """Restore offsets only after the nozzle is safely clear of the post."""
    try:
        lift_nozzle_to_safe_z(nozzle, cfg, logger)
        synchronize_coordinate_system(nozzle, machine_obj, logger)
    except Throwable as error:
        raise NeedleCalibrationError(
            "SAFETY-CRITICAL: safe nozzle retraction failed; original offsets "
            "were not restored: %s" % error)
    except Exception as error:
        raise NeedleCalibrationError(
            "SAFETY-CRITICAL: safe nozzle retraction failed; original offsets "
            "were not restored: %s" % error)
    try:
        restored = set_offsets_and_verify(nozzle, offsets, logger)
        synchronize_coordinate_system(nozzle, machine_obj, logger)
        verified = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(
            LengthUnit.Millimeters))
        if not offsets_match(verified, offsets):
            raise NeedleCalibrationError(
                "restored offsets did not verify: expected=%s actual=%s" %
                (offsets, verified))
        log_msg(logger, "Original head offsets restored and verified: %s" %
            verified)
        return {"ok": True, "restored_offsets": restored}
    except Throwable as error:
        raise NeedleCalibrationError(
            "SAFETY-CRITICAL: original head-offset restoration failed: %s" %
            error)
    except Exception as error:
        raise NeedleCalibrationError(
            "SAFETY-CRITICAL: original head-offset restoration failed: %s" %
            error)


def make_z_proposal(cfg, old, z_error):
    old = old.convertToUnits(LengthUnit.Millimeters)
    dz = float(cfg.get("z_offset_sign", 1.0)) * float(z_error)
    new = mm_loc(old.getX(), old.getY(), old.getZ() + dz, old.getRotation())
    return {"type": "z", "measured_error": {"z": float(z_error)},
        "offset_delta": {"x": 0.0, "y": 0.0, "z": dz},
        "proposed_offsets": loc_to_dict(new)}


def make_xy_proposal(cfg, old, dx, dy):
    old = old.convertToUnits(LengthUnit.Millimeters)
    sign = float(cfg.get("xy_offset_sign", 1.0))
    delta_x = -sign * float(dx)
    delta_y = -sign * float(dy)
    new = mm_loc(old.getX() + delta_x, old.getY() + delta_y,
        old.getZ(), old.getRotation())
    return {"type": "xy", "measured_error": {"x": float(dx), "y": float(dy)},
        "offset_delta": {"x": delta_x, "y": delta_y, "z": 0.0},
        "proposed_offsets": loc_to_dict(new)}


def run_z_measure(cfg, nozzle=None, machine_obj=None, logger=None):
    stage = "z"
    old_offsets = None
    try:
        machine_obj = get_machine(machine_obj)
        nozzle = find_nozzle(cfg, nozzle, machine_obj)
        zp = cfg.get("z_probe", {})
        if (str(zp.get("acquisition_method", "hardware_interrupt")).lower() ==
                "hardware_interrupt" and _probe_mode(cfg) == "actuator"):
            actuator = find_named_actuator(machine_obj,
                zp.get("sequence_actuator_name", "Run Voron Probe"), nozzle)
        else:
            actuator = find_actuator_by_name(machine_obj, cfg, "z")
        old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
        old_offsets = loc_to_dict(old)
        capture = capture_z_touch_center(cfg, nozzle, machine_obj, logger)
        center = capture.get("center")
        surface_z = float(dict_to_loc(cfg.get("fiducials", {}).get("z_touch", {}).get("location", {})).getZ())
        touch = probe_z_touch(nozzle, actuator, cfg, center["x"], center["y"], surface_z, logger)
        proposal = make_z_proposal(cfg, old, touch["z_error_mm"])
        current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        return {"ok": True, "stage": "z", "message": "ok",
            "old_offsets": old_offsets, "current_offsets": current,
            "proposal": proposal,
            "details": {"capture": capture, "touch": touch, "center": center}}
    except Throwable as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)
    except Exception as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)


def pass_label(pass_name):
    name = str(pass_name or "").lower()
    if name == "rough":
        return "COARSE"
    if name == "final":
        return "FINE"
    return str(pass_name).upper()


def log_fit_summary(logger, pass_name, fit, vision_center=None):
    center = fit.get("center", {})
    log_msg(logger, "%s FIT center=%s radius=%s rms=%s max=%s" %
        (pass_label(pass_name), fmt_xy(center.get("x", 0.0), center.get("y", 0.0)),
         fmt_float(fit.get("radius_mm", 0.0)),
         fmt_float(fit.get("rms_error_mm", 0.0)),
         fmt_float(fit.get("max_error_mm", 0.0))))
    if vision_center is not None:
        dx = float(center.get("x", 0.0)) - float(vision_center.get("x", 0.0))
        dy = float(center.get("y", 0.0)) - float(vision_center.get("y", 0.0))
        log_msg(logger, "%s DELTA from vision=%s" %
            (pass_label(pass_name), fmt_xy(dx, dy)))


def probe_pass(cfg, nozzle, actuator, center, angles, probe_radius, z_probe,
        pass_name, logger=None):
    contacts = []
    try:
        for angle in angles:
            contacts.append(probe_edge_touch_confirm(cfg, nozzle, actuator, center,
                angle, probe_radius, z_probe, pass_name, logger))
        lift_nozzle_to_safe_z(nozzle, cfg, logger)
    except:
        try:
            lift_nozzle_to_safe_z(nozzle, cfg, logger)
        except:
            pass
        raise
    fit = circle_fit_contacts(contacts)
    return {"pass_name": str(pass_name), "angles": angles, "contacts": contacts,
        "fit": fit}


def run_post_xy_measure(cfg, nozzle=None, machine_obj=None, logger=None):
    stage = "post_xy"
    old_offsets = None
    try:
        machine_obj = get_machine(machine_obj)
        nozzle = find_nozzle(cfg, nozzle, machine_obj)
        actuator = find_actuator_by_name(machine_obj, cfg, "post_xy")
        old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
        old_offsets = loc_to_dict(old)
        capture = capture_post_center(cfg, nozzle, machine_obj, logger)
        vision_center = capture.get("vision_center")
        post = cfg.get("post_xy", {})
        configured_diameter = float(post.get("post_diameter_mm", 6.35))
        configured_radius = configured_diameter / 2.0
        z_probe = resolve_post_side_probe_z_mm(cfg, logger)
        angles = parse_angles(post.get("angles_deg"),
            [0, 45, 90, 135, 180, 225, 270, 315])
        acquisition = str(post.get("acquisition_method", "polling") or "polling").lower()
        settings = post_xy_settings(cfg) if acquisition == "polling" else None
        scan = post_scan_settings(cfg) if acquisition == "polling" else None
        log_section(logger, "POST XY 8-POINT MEASURE START")
        log_kv(logger, "nozzle name", object_name(nozzle))
        log_kv(logger, "old offsets", fmt_loc(old_offsets))
        log_kv(logger, "vision center", fmt_xy(vision_center["x"], vision_center["y"]))
        log_kv(logger, "known ball/post diameter/radius",
            "%s/%s" % (fmt_float(configured_diameter), fmt_float(configured_radius)))
        detected = capture.get("detected_post_diameter_mm", None)
        if detected is not None:
            log_kv(logger, "vision detected diameter ignored", fmt_float(detected))
        log_kv(logger, "side probe z", fmt_float(z_probe))
        log_kv(logger, "angles", angles)
        log_kv(logger, "touch method", acquisition)
        if acquisition == "hardware_interrupt":
            log_kv(logger, "hardware probe", "channel=%s accel=%s coarse=%sx%s fine=%sx%s invalidFineRetries=%s" %
                (post.get("hardware_channel", "Q1"),
                 fmt_float(post.get("hardware_probe_acceleration", 1000.0)),
                 int(float(post.get("hardware_coarse_repeats", 1))),
                 fmt_float(post.get("hardware_coarse_feed", 3000.0)),
                 int(float(post.get("hardware_fine_repeats", 2))),
                 fmt_float(post.get("hardware_fine_feed", 500.0)),
                 int(float(post.get("hardware_fine_retry_attempts", 2)))))
            log_kv(logger, "hardware distances",
                "startRadius=%s coarseMax=%s coarseMin=%s fineRetract=%s fineMax=%s repeatTolerance=%s" %
                (fmt_float(post.get("hardware_start_radius_mm", 4.175)),
                 fmt_float(post.get("hardware_coarse_max_travel_mm", 7.0)),
                 fmt_float(post.get("hardware_min_coarse_travel_mm", 0.25)),
                 fmt_float(post.get("hardware_fine_retract_mm", 1.0)),
                 fmt_float(post.get("hardware_fine_max_travel_mm", 2.0)),
                 fmt_float(post.get("hardware_repeat_tolerance_mm", 0.15))))
        else:
            log_kv(logger, "step/speed", "step=%s speed=%s" %
                (fmt_float(settings["step_mm"]), fmt_float(settings["speed"])))
            log_kv(logger, "contact read", "debounce_count=%s retries=%s interval_ms=%s" %
                (int(float(post.get("debounce_count", 3))),
                 int(float(post.get("debounce_retries", 2))),
                 int(float(post.get("debounce_interval_ms", 5)))))
        if acquisition == "polling":
            log_kv(logger, "polling scan", "clearance=%s overtravel=%s cap=%s travelLimit=%s" %
                (fmt_float(scan["clearance_mm"]), fmt_float(scan["overtravel_mm"]),
                 fmt_float(scan["cap_mm"]), fmt_float(scan["travel_limit_mm"])))

        result = probe_8_point_post_xy(cfg, nozzle, actuator, vision_center,
            configured_radius, z_probe, logger)
        fit = result["fit"]
        valid_count = int(result.get("valid_contact_count", 0))
        invalid_count = int(result.get("invalid_contact_count", 0))
        min_valid = int(result.get("min_valid_contacts_for_fit",
            post.get("min_valid_contacts_for_fit", 5)))
        log_msg(logger, "POST XY EDGE SUMMARY valid=%d invalid=%d minRequired=%d" %
            (valid_count, invalid_count, min_valid))
        for invalid in result.get("invalid_edges", []):
            log_msg(logger, "INVALID EDGE angle=%s reason=%s" %
                (invalid.get("angle_deg", ""), invalid.get("error", "")))
        base_details = {"vision_center": vision_center,
            "all_edges": result.get("all_edges", []),
            "contacts": result.get("contacts", []),
            "invalid_edges": result.get("invalid_edges", []),
            "valid_contact_count": valid_count,
            "invalid_contact_count": invalid_count,
            "min_valid_contacts_for_fit": min_valid,
            "post_side_probe_z_mm": z_probe,
            "known_ball_post_radius_mm": configured_radius,
            "known_ball_post_diameter_mm": configured_diameter,
            "configured_post_radius_mm": configured_radius,
            "configured_post_diameter_mm": configured_diameter,
            "camera_capture": capture}
        if fit is None:
            current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
            return {"ok": False, "stage": stage,
                "message": "only %d valid contacts, need at least %d" %
                    (valid_count, min_valid),
                "old_offsets": old_offsets, "current_offsets": current,
                "proposal": None, "details": base_details}

        fitted_center = fit["center"]
        fitted_radius = float(fit["radius_mm"])
        fitted_diameter = 2.0 * fitted_radius
        known_ball_radius = configured_radius
        known_ball_diameter = configured_diameter
        estimated_needle_tip_radius = fitted_radius - known_ball_radius
        estimated_needle_tip_diameter = 2.0 * estimated_needle_tip_radius
        fit_quality_warnings = []
        max_fit_rms = float(post.get("max_fit_rms_mm", 0.15))
        max_fit_error = float(post.get("max_fit_error_mm", 0.30))
        if float(fit.get("rms_error_mm", 0.0)) > max_fit_rms:
            fit_quality_warnings.append(
                "fit RMS %.6f exceeds %.6f" %
                (float(fit.get("rms_error_mm")), max_fit_rms))
        if float(fit.get("max_error_mm", 0.0)) > max_fit_error:
            fit_quality_warnings.append(
                "fit max error %.6f exceeds %.6f" %
                (float(fit.get("max_error_mm")), max_fit_error))

        pair_debug = pair_midpoint_debug(result.get("contacts", []), vision_center)
        estimator = estimate_xy_contact_center(result.get("contacts", []),
            vision_center, post, fit)
        details = dict(base_details)
        details.update({"fit": fit,
            "fitted_contact_circle_radius_mm": fitted_radius,
            "fitted_contact_circle_diameter_mm": fitted_diameter,
            # Backward-compatible aliases for existing GUI code:
            "fitted_contact_radius_mm": fitted_radius,
            "fitted_contact_diameter_mm": fitted_diameter,
            "estimated_needle_tip_radius_mm": estimated_needle_tip_radius,
            "estimated_needle_tip_diameter_mm": estimated_needle_tip_diameter,
            # Backward-compatible aliases for existing GUI code:
            "estimated_effective_probe_radius_mm": estimated_needle_tip_radius,
            "estimated_effective_probe_diameter_mm": estimated_needle_tip_diameter,
            "pair_midpoint_debug": pair_debug,
            "xy_estimator": estimator,
            "fit_quality_warnings": fit_quality_warnings,
            "circle_fit_center_delta": {
                "x": float(fitted_center["x"]) - float(vision_center["x"]),
                "y": float(fitted_center["y"]) - float(vision_center["y"])}})

        radius_tolerance = abs(float(post.get(
            "min_fitted_radius_tolerance_mm", 0.05)))
        if fitted_radius < known_ball_radius - radius_tolerance:
            details["radius_consistency_error_mm"] = (
                known_ball_radius - fitted_radius)
            current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(
                LengthUnit.Millimeters))
            return {"ok": False, "stage": stage,
                "message": "fitted contact radius %.6f mm is below known post "
                    "radius %.6f mm by more than %.6f mm tolerance" %
                    (fitted_radius, known_ball_radius, radius_tolerance),
                "old_offsets": old_offsets, "current_offsets": current,
                "proposal": None, "details": details}

        if not estimator.get("ok", False):
            details["xy_estimator_error"] = estimator.get("message",
                "XY estimator rejected the contact geometry")
            current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(
                LengthUnit.Millimeters))
            return {"ok": False, "stage": stage,
                "message": details["xy_estimator_error"],
                "old_offsets": old_offsets, "current_offsets": current,
                "proposal": None, "details": details}

        xy_center = estimator["center"]
        xy_center_source = estimator.get("source", "xy_estimator")
        dx = float(estimator["displacement"]["x"])
        dy = float(estimator["displacement"]["y"])
        details.update({"xy_center_source": xy_center_source,
            "xy_contact_center": xy_center,
            "center_delta": {"x": dx, "y": dy}})
        proposal = make_xy_proposal(cfg, old, dx, dy)

        for row in pair_debug.get("pairs", []):
            if row.get("valid", False):
                mp = row.get("midpoint", {})
                log_msg(logger, "POST XY PAIR %s midpoint=%s span=%s" %
                    (row.get("pair", ""), fmt_xy(mp.get("x", 0.0), mp.get("y", 0.0)),
                     fmt_float(row.get("span_mm", 0.0))))
            else:
                log_msg(logger, "POST XY PAIR %s midpoint=<missing>" %
                    row.get("pair", ""))

        log_msg(logger, "POST XY KNOWN BALL/POST diameter=%s radius=%s" %
            (fmt_float(known_ball_diameter), fmt_float(known_ball_radius)))
        log_msg(logger, "POST XY FITTED CONTACT CIRCLE center=%s diameter=%s radius=%s rms=%s max=%s" %
            (fmt_xy(fitted_center["x"], fitted_center["y"]),
             fmt_float(fitted_diameter), fmt_float(fitted_radius),
             fmt_float(fit.get("rms_error_mm", 0.0)),
             fmt_float(fit.get("max_error_mm", 0.0))))
        log_msg(logger, "POST XY ESTIMATED NEEDLE TIP diameter=%s radius=%s" %
            (fmt_float(estimated_needle_tip_diameter),
             fmt_float(estimated_needle_tip_radius)))
        geometry_diagnostics = estimator.get("diagnostics", {}).get("geometry", {})
        pair_diagnostics = estimator.get("diagnostics", {}).get(
            "opposite_pair_estimator", {})
        log_msg(logger, "POST XY GEOMETRY coverage=%s maxGap=%s maxLineError=%s "
            "circleCondition=%s pairCondition=%s" %
            (fmt_float(geometry_diagnostics.get("angular_coverage_deg", 0.0)),
             fmt_float(geometry_diagnostics.get("max_angular_gap_deg", 0.0)),
             fmt_float(geometry_diagnostics.get("max_line_error_mm", 0.0)),
             fmt_float(fit.get("condition_number", 0.0)),
             fmt_float(pair_diagnostics.get("condition_number", 0.0))))
        log_msg(logger, "POST XY CIRCLE FIT CENTER DELTA from vision=%s" %
            fmt_xy(fitted_center["x"] - vision_center["x"],
                fitted_center["y"] - vision_center["y"]))
        log_msg(logger, "POST XY CONTACT CENTER source=%s center=%s" %
            (xy_center_source, fmt_xy(xy_center["x"], xy_center["y"])))
        log_msg(logger, "POST XY NEEDLE-TO-CAMERA DELTA from vision=%s" %
            fmt_xy(dx, dy))
        log_msg(logger, "PROPOSED XY offset_delta=%s proposed_offsets=%s" %
            (fmt_xy(proposal["offset_delta"]["x"], proposal["offset_delta"]["y"]),
             fmt_loc(proposal["proposed_offsets"])))

        max_center = float(post.get("max_center_error_mm", 2.0))
        if abs(dx) > max_center or abs(dy) > max_center:
            return {"ok": False, "stage": stage,
                "message": "center delta X/Y %.6f/%.6f exceeds %.6f" %
                    (dx, dy, max_center),
                "old_offsets": old_offsets,
                "current_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)),
                "proposal": None, "details": details}
        current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        return {"ok": True, "stage": stage, "message": "ok",
            "old_offsets": old_offsets, "current_offsets": current,
            "proposal": proposal, "details": details}
    except Throwable as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)
    except Exception as e:
        return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)



def run_full_measure(cfg, nozzle=None, machine_obj=None, logger=None):
    old_offsets = None
    z = None
    post = None
    temporary_offsets = None
    temporary_change_attempted = False
    try:
        machine_obj = get_machine(machine_obj)
        nozzle = find_nozzle(cfg, nozzle, machine_obj)
        old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
        old_offsets = loc_to_dict(old)
        z = run_z_measure(cfg, nozzle, machine_obj, logger)
        if not z.get("ok", False):
            return {"ok": False, "stage": "full",
                "message": "Z measure failed: %s" % z.get("message"),
                "old_offsets": old_offsets,
                "current_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(
                    LengthUnit.Millimeters)),
                "z": z, "post_xy": None, "proposal": None,
                "offsets_restored": True,
                "calibration_state": {"status": "original",
                    "original_offsets": deep_copy(old_offsets),
                    "temporary_offsets": None,
                    "proposed_offsets": None,
                    "temporary_z_active": False}}

        # The Z proposal is calculated against the snapshot taken before any
        # calibration movement. Apply only Z temporarily, preserving X/Y/R.
        z_proposed = z.get("proposal", {}).get("proposed_offsets", {})
        temporary_offsets = deep_copy(old_offsets)
        temporary_offsets["z"] = float(z_proposed["z"])
        check_apply_delta(cfg, old_offsets, temporary_offsets, ["z"])
        log_msg(logger, "Full XYZ original Z=%s temporary Z=%s" %
            (fmt_float(old_offsets["z"]), fmt_float(temporary_offsets["z"])))
        temporary_change_attempted = True
        set_offsets_and_verify(nozzle, temporary_offsets, logger)
        # Explicitly verify that the temporary operation did not touch X/Y/R.
        active_temporary = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(
            LengthUnit.Millimeters))
        if (abs(active_temporary["x"] - old_offsets["x"]) > OFFSET_VERIFY_TOLERANCE_MM or
                abs(active_temporary["y"] - old_offsets["y"]) > OFFSET_VERIFY_TOLERANCE_MM or
                abs(active_temporary["rotation"] - old_offsets["rotation"]) >
                OFFSET_VERIFY_TOLERANCE_MM):
            raise NeedleCalibrationError(
                "Temporary Z application changed X, Y, or rotation")
        synchronize_coordinate_system(nozzle, machine_obj, logger)
        log_msg(logger, "Temporary corrected Z is active for contact-post XY")

        post = run_post_xy_measure(cfg, nozzle, machine_obj, logger)
        if not post.get("ok", False):
            raise NeedleCalibrationError("Post XY measure failed: %s" % post.get("message"))
        zd = z["proposal"]["offset_delta"]
        xd = post["proposal"]["offset_delta"]
        proposed = mm_loc(old_offsets["x"] + float(xd.get("x", 0.0)),
            old_offsets["y"] + float(xd.get("y", 0.0)),
            old_offsets["z"] + float(zd.get("z", 0.0)),
            old_offsets["rotation"])
        proposed_offsets = loc_to_dict(proposed)
        check_apply_delta(cfg, old_offsets, proposed_offsets, ["x", "y", "z"])
        calibration_state = {
            "status": "temporary",
            "original_offsets": deep_copy(old_offsets),
            "temporary_offsets": deep_copy(temporary_offsets),
            "proposed_offsets": deep_copy(proposed_offsets),
            "temporary_z_active": True,
            "accepted": False
        }
        proposal = {"type": "xyz",
            "offset_delta": {"x": float(xd.get("x", 0.0)),
                "y": float(xd.get("y", 0.0)), "z": float(zd.get("z", 0.0))},
            "proposed_offsets": proposed_offsets,
            "original_offsets": deep_copy(old_offsets),
            "temporary_offsets": deep_copy(temporary_offsets),
            "calibration_state": deep_copy(calibration_state)}
        current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        return {"ok": True, "stage": "full", "message": "ok",
            "old_offsets": old_offsets, "current_offsets": current,
            "z": z, "post_xy": post, "proposal": proposal,
            "original_offsets": deep_copy(old_offsets),
            "temporary_offsets": deep_copy(temporary_offsets),
            "offsets_restored": False,
            "calibration_state": calibration_state}
    except Throwable as e:
        error = e
        cleanup = None
        restore_error = None
        if nozzle is not None and temporary_change_attempted:
            try:
                cleanup = restore_offsets_after_safe_retraction(cfg, nozzle,
                    old_offsets, machine_obj, logger)
            except Throwable as cleanup_exception:
                restore_error = str(cleanup_exception)
            except Exception as cleanup_exception:
                restore_error = str(cleanup_exception)
        message = str(error)
        if restore_error is not None:
            message = "%s; %s" % (message, restore_error)
        current = None
        try:
            current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(
                LengthUnit.Millimeters)) if nozzle is not None else old_offsets
        except:
            current = old_offsets
        restored = temporary_change_attempted and restore_error is None
        return {"ok": False, "stage": "full", "message": message,
            "old_offsets": old_offsets, "current_offsets": current,
            "z": z, "post_xy": post, "proposal": None,
            "offsets_restored": restored, "cleanup": cleanup,
            "safety_critical": restore_error is not None,
            "calibration_state": {"status": "reverted" if restored else
                ("restore_failed" if temporary_change_attempted else "original"),
                "original_offsets": deep_copy(old_offsets),
                "temporary_offsets": deep_copy(temporary_offsets),
                "proposed_offsets": None,
                "temporary_z_active": not restored}}
    except Exception as e:
        error = e
        cleanup = None
        restore_error = None
        if nozzle is not None and temporary_change_attempted:
            try:
                cleanup = restore_offsets_after_safe_retraction(cfg, nozzle,
                    old_offsets, machine_obj, logger)
            except Throwable as cleanup_exception:
                restore_error = str(cleanup_exception)
            except Exception as cleanup_exception:
                restore_error = str(cleanup_exception)
        message = str(error)
        if restore_error is not None:
            message = "%s; %s" % (message, restore_error)
        current = None
        try:
            current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(
                LengthUnit.Millimeters)) if nozzle is not None else old_offsets
        except:
            current = old_offsets
        restored = temporary_change_attempted and restore_error is None
        return {"ok": False, "stage": "full", "message": message,
            "old_offsets": old_offsets, "current_offsets": current,
            "z": z, "post_xy": post, "proposal": None,
            "offsets_restored": restored, "cleanup": cleanup,
            "safety_critical": restore_error is not None,
            "calibration_state": {"status": "reverted" if restored else
                ("restore_failed" if temporary_change_attempted else "original"),
                "original_offsets": deep_copy(old_offsets),
                "temporary_offsets": deep_copy(temporary_offsets),
                "proposed_offsets": None,
                "temporary_z_active": not restored}}


def check_apply_delta(cfg, current, proposed, axes):
    max_delta = float(cfg.get("max_apply_delta_mm", 5.0))
    for axis in axes:
        before = float(current[axis])
        after = float(proposed[axis])
        if abs(after - before) > max_delta:
            raise NeedleCalibrationError("%s offset delta %.6f exceeds max_apply_delta_mm %.6f" %
                (axis.upper(), abs(after - before), max_delta))


def set_offsets_from_dict(nozzle, offsets):
    nozzle.setHeadOffsets(mm_loc(offsets.get("x", 0.0), offsets.get("y", 0.0),
        offsets.get("z", 0.0), offsets.get("rotation", 0.0)))


def apply_z(cfg, nozzle, proposal, logger=None):
    proposed = proposal.get("proposed_offsets", proposal)
    current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
    target = deep_copy(current)
    target["z"] = float(proposed["z"])
    check_apply_delta(cfg, current, target, ["z"])
    log_msg(logger, "Apply Z offset old=%s new=%s" % (current, target))
    # Applying offsets is immediately followed by the caller's explicit
    # Configuration.save(). Do not add a motion-planner wait here: if that
    # wait fails after setHeadOffsets(), the operator workflow treats the
    # whole apply as failed and restores the old offsets before saving.
    # The full-measurement path already synchronizes before starting XY.
    new_offsets = set_offsets_and_verify(nozzle, target, logger)
    return {"ok": True, "stage": "apply_z", "old_offsets": current,
        "new_offsets": new_offsets}


def apply_xy(cfg, nozzle, proposal, logger=None):
    proposed = proposal.get("proposed_offsets", proposal)
    current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
    target = deep_copy(current)
    target["x"] = float(proposed["x"])
    target["y"] = float(proposed["y"])
    check_apply_delta(cfg, current, target, ["x", "y"])
    log_msg(logger, "Apply XY offset old=%s new=%s" % (current, target))
    # Keep Apply XY a direct set/verify operation. Persistence is handled by
    # the caller, and a planner wait must not turn a successful set into a
    # rollback before that save occurs.
    new_offsets = set_offsets_and_verify(nozzle, target, logger)
    return {"ok": True, "stage": "apply_xy", "old_offsets": current,
        "new_offsets": new_offsets}


def apply_xyz(cfg, nozzle, proposal, logger=None):
    proposed = proposal.get("proposed_offsets", proposal)
    current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
    target = deep_copy(current)
    target["x"] = float(proposed["x"])
    target["y"] = float(proposed["y"])
    target["z"] = float(proposed["z"])
    if "rotation" in proposed:
        target["rotation"] = float(proposed["rotation"])
    state = proposal.get("calibration_state", {})
    limit_base = state.get("original_offsets") if isinstance(state, dict) else None
    check_apply_delta(cfg, limit_base or current, target, ["x", "y", "z"])
    log_msg(logger, "Apply XYZ offset old=%s new=%s" % (current, target))
    # Keep Apply XY a direct set/verify operation; the caller saves after the
    # function returns successfully.
    new_offsets = set_offsets_and_verify(nozzle, target, logger)
    accepted_state = None
    if isinstance(state, dict) and state:
        accepted_state = {"status": "accepted",
            "original_offsets": deep_copy(state.get("original_offsets")),
            "temporary_offsets": deep_copy(state.get("temporary_offsets")),
            "proposed_offsets": deep_copy(target),
            "temporary_z_active": False, "accepted": True}
    return {"ok": True, "stage": "apply_xyz", "old_offsets": current,
        "new_offsets": new_offsets, "calibration_state": accepted_state}


def revert_offsets(nozzle, offsets, logger=None, cfg=None):
    current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
    if cfg is not None:
        restore_offsets_after_safe_retraction(cfg, nozzle, offsets, None, logger)
    else:
        set_offsets_and_verify(nozzle, offsets, logger)
        synchronize_coordinate_system(nozzle, None, logger)
    new_offsets = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
    log_msg(logger, "Revert offsets old=%s restored=%s" % (current, new_offsets))
    return {"ok": True, "stage": "revert_offsets",
        "old_offsets": current, "new_offsets": new_offsets,
        "calibration_state": {"status": "reverted",
            "original_offsets": deep_copy(offsets),
            "temporary_offsets": None, "proposed_offsets": None,
            "temporary_z_active": False}}


if __name__ == "__main__":
    cfg = load_config()
    log_msg(None, "Loaded needle calibration config: %s" % cfg.get("_config_path"))
# # needle_calibration2.py
# #
# # Simplified dispensing needle calibration for OpenPnP Jython 2.7.
# # Produces offset proposals only. Applying offsets is explicit.

# import json
# import math
# import os

# from java.lang import System, Thread, Throwable
# from org.openpnp.model import Configuration, LengthUnit, Location
# from org.openpnp.spi import Camera as SpiCamera
# from org.openpnp.spi.MotionPlanner import CompletionType
# from org.openpnp.util import MovableUtils, VisionUtils
# from org.openpnp.vision.pipeline import CvPipeline


# try:
#     basestring
# except NameError:
#     basestring = str


# try:
#     SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# except:
#     SCRIPT_DIR = os.getcwd()
# SCRIPTS_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
# DEFAULT_CONFIG_PATH = os.path.join(SCRIPTS_ROOT, "needle_calibration_config.json")


# class NeedleCalibrationError(Exception):
#     pass


# DEFAULT_CONFIG = {
#     "schema_version": 3,
#     "head_name": "",
#     "nozzle_name": "Dispense Head - Needle",
#     "top_camera_name": "",
#     "contact_actuator_name": "NeedleContactProbe_Read",
#     "safe_z_mm": 25.0,
#     "travel_speed": 1.0,
#     "probe_speed": 0.8,
#     "fine_probe_speed": 0.05,
#     "xy_offset_sign": 1.0,
#     "z_offset_sign": 1.0,
#     "max_apply_delta_mm": 5.0,
#     "z_probe": {
#         "start_above_mm": 4.0,
#         "max_descent_mm": 8.0,
#         "coarse_step_mm": 0.10,
#         "fine_step_mm": 0.02,
#         "retract_after_contact_mm": 0.40,
#         "release_retract_step_mm": 0.20,
#         "max_release_retract_mm": 2.0,
#         "read_settle_ms": 80,
#         "debounce_count": 3,
#         "debounce_retries": 3,
#         "debounce_interval_ms": 20
#     },
#     "fiducials": {
#         "z_touch": {
#             "location": {"x": 0.0, "y": 0.0, "z": 0.0, "rotation": 0.0},
#             "result_stage": "results",
#             "pipeline": {"mode": "named", "name": "", "path": ""}
#         }
#     },
#     "post_xy": {
#         "location": {"x": 0.0, "y": 0.0, "z": 0.0, "rotation": 0.0},
#         "result_stage": "results",
#         "pipeline": {"mode": "named", "name": "", "path": ""},
#         "post_diameter_mm": 6.35,
#         "angles_deg": [0, 45, 90, 135, 180, 225, 270, 315],
#         "scan_clearance_mm": 2.0,
#         "scan_overtravel_mm": 3.0,
#         "max_radial_probe_travel_mm": 5.0,
#         "step_mm": 0.05,
#         "speed": 0.8,
#         "confirm_backoff_mm": 0.4,
#         "confirm_reapproach_mm": 0.8,
#         "confirm_step_mm": None,
#         "confirm_position_tolerance_mm": 0.15,
#         "confirm_reject_early": True,
#         "read_settle_ms": 10,
#         "debounce_count": 3,
#         "debounce_retries": 2,
#         "debounce_interval_ms": 5,
#         "allow_single_read_contact": False,
#         "min_valid_contacts_for_fit": 5,
#         "side_probe_z_offset_mm": 0.25,
#         "local_lift_mm": 0.60,
#         "outside_recover_step_mm": 0.25,
#         "outside_recover_max_mm": 3.0,
#         "max_fit_rms_mm": 0.15,
#         "max_fit_error_mm": 0.30,
#         "max_center_error_mm": 2.0,
#         "log_every_step": False,
#         "log_step_interval": 10
#     },
#     "debug": {
#         "verbose": False,
#         "save_last_capture": True
#     }
# }


# def log_msg(logger, msg):
#     if logger:
#         logger(str(msg))
#     else:
#         print(str(msg))


# def log_section(logger, title):
#     log_msg(logger, "")
#     log_msg(logger, "== %s ==" % str(title))


# def log_kv(logger, label, data):
#     log_msg(logger, "%s: %s" % (str(label), data))


# def fmt_float(v):
#     try:
#         return "%.6f" % float(v)
#     except:
#         return str(v)


# def fmt_xy(x, y):
#     return "(%s,%s)" % (fmt_float(x), fmt_float(y))


# def fmt_loc(loc):
#     try:
#         if hasattr(loc, "convertToUnits"):
#             d = loc_to_dict(loc)
#         else:
#             d = loc
#         return "(%s,%s,%s,%s)" % (
#             fmt_float(d.get("x", 0.0)), fmt_float(d.get("y", 0.0)),
#             fmt_float(d.get("z", 0.0)), fmt_float(d.get("rotation", 0.0)))
#     except:
#         return str(loc)


# def deep_copy(value):
#     return json.loads(json.dumps(value, sort_keys=True))


# def deep_merge(defaults, loaded):
#     out = deep_copy(defaults)
#     if not isinstance(loaded, dict):
#         return out
#     for key in loaded:
#         value = loaded[key]
#         if isinstance(value, dict) and isinstance(out.get(key), dict):
#             out[key] = deep_merge(out[key], value)
#         else:
#             out[key] = value
#     return out


# def clean_to_schema(defaults, loaded):
#     out = deep_copy(defaults)
#     if not isinstance(loaded, dict):
#         return out
#     for key in defaults:
#         if key not in loaded:
#             continue
#         if isinstance(defaults.get(key), dict) and isinstance(loaded.get(key), dict):
#             out[key] = clean_to_schema(defaults.get(key), loaded.get(key))
#         else:
#             out[key] = loaded.get(key)
#     return out


# def normalize_loaded_config(loaded):
#     cfg = deep_merge(DEFAULT_CONFIG, loaded)
#     post_loaded = loaded.get("post_xy", {}) if isinstance(loaded, dict) else {}
#     post = cfg.get("post_xy", {})
#     # Compatibility with older two-pass post XY configs.
#     if "angles_deg" not in post_loaded and "final_angles_deg" in post_loaded:
#         post["angles_deg"] = post_loaded.get("final_angles_deg")
#     if "angles_deg" not in post_loaded and "verify_angles_deg" in post_loaded:
#         post["angles_deg"] = post_loaded.get("verify_angles_deg")
#     if "step_mm" not in post_loaded and "final_step_mm" in post_loaded:
#         post["step_mm"] = post_loaded.get("final_step_mm")
#     if "step_mm" not in post_loaded and "rough_step_mm" in post_loaded:
#         post["step_mm"] = post_loaded.get("rough_step_mm")
#     if "speed" not in post_loaded or post_loaded.get("speed", None) is None:
#         post["speed"] = 0.8
#     if not config_bool(post.get("allow_single_read_contact", False), False):
#         try:
#             post["debounce_count"] = max(int(float(post.get("debounce_count", 3))), 3)
#         except:
#             post["debounce_count"] = 3
#         try:
#             post["debounce_retries"] = max(int(float(post.get("debounce_retries", 2))), 2)
#         except:
#             post["debounce_retries"] = 2
#         try:
#             post["debounce_interval_ms"] = max(int(float(post.get("debounce_interval_ms", 5))), 5)
#         except:
#             post["debounce_interval_ms"] = 5
#     cfg["schema_version"] = 3
#     return cfg


# def load_config(path=None):
#     cfg_path = path or DEFAULT_CONFIG_PATH
#     if not os.path.exists(cfg_path):
#         cfg = deep_copy(DEFAULT_CONFIG)
#         cfg["_config_path"] = cfg_path
#         return cfg
#     fh = open(cfg_path, "r")
#     try:
#         loaded = json.loads(fh.read())
#     finally:
#         fh.close()
#     cfg = normalize_loaded_config(loaded)
#     cfg["_config_path"] = cfg_path
#     return cfg


# def public_clean_config(cfg):
#     clean = clean_to_schema(DEFAULT_CONFIG, cfg)
#     clean["schema_version"] = 3
#     return clean


# def save_config(cfg, path=None):
#     cfg_path = path or cfg.get("_config_path", DEFAULT_CONFIG_PATH)
#     clean = public_clean_config(cfg)
#     fh = open(cfg_path, "w")
#     try:
#         fh.write(json.dumps(clean, sort_keys=True, indent=2))
#     finally:
#         fh.close()
#     return cfg_path


# def mm_loc(x, y, z, r=0.0):
#     return Location(LengthUnit.Millimeters, float(x), float(y), float(z), float(r))


# def loc_to_dict(loc):
#     loc = loc.convertToUnits(LengthUnit.Millimeters)
#     return {
#         "x": float(loc.getX()),
#         "y": float(loc.getY()),
#         "z": float(loc.getZ()),
#         "rotation": float(loc.getRotation())
#     }


# def dict_to_loc(data, fallback=None):
#     if hasattr(data, "convertToUnits"):
#         return data.convertToUnits(LengthUnit.Millimeters)
#     if fallback is None:
#         fallback = Location(LengthUnit.Millimeters)
#     fallback = fallback.convertToUnits(LengthUnit.Millimeters)
#     if data is None:
#         return fallback
#     return mm_loc(
#         data.get("x", fallback.getX()),
#         data.get("y", fallback.getY()),
#         data.get("z", fallback.getZ()),
#         data.get("rotation", data.get("r", fallback.getRotation())))


# def loc_to_log(loc):
#     d = loc_to_dict(loc)
#     return "X %.6f Y %.6f Z %.6f R %.6f" % (
#         d["x"], d["y"], d["z"], d["rotation"])


# def java_list_to_py(model):
#     if model is None:
#         return []
#     if isinstance(model, list):
#         return model
#     if hasattr(model, "size") and hasattr(model, "get"):
#         out = []
#         for i in range(model.size()):
#             out.append(model.get(i))
#         return out
#     try:
#         return list(model)
#     except:
#         return [model]


# def object_name(obj):
#     if obj is None:
#         return ""
#     for name in ["getName", "getId"]:
#         try:
#             value = getattr(obj, name)()
#             if value is not None:
#                 return str(value)
#         except:
#             pass
#     return str(obj)


# def get_machine(machine_obj=None):
#     if machine_obj is not None:
#         return machine_obj
#     try:
#         return machine
#     except:
#         return Configuration.get().getMachine()


# def all_heads(machine_obj):
#     heads = []
#     try:
#         heads = java_list_to_py(machine_obj.getHeads())
#     except:
#         pass
#     if len(heads) == 0:
#         try:
#             head = machine_obj.getDefaultHead()
#             if head is not None:
#                 heads.append(head)
#         except:
#             pass
#     return heads


# def find_head(machine_obj, head_name):
#     wanted = str(head_name or "").strip()
#     if wanted == "":
#         try:
#             head = machine_obj.getDefaultHead()
#             if head is not None:
#                 return head
#         except:
#             pass
#     for head in all_heads(machine_obj):
#         if wanted == "" or object_name(head) == wanted:
#             return head
#     raise NeedleCalibrationError("Head not found: %s" % wanted)


# def find_nozzle(cfg, nozzle=None, machine_obj=None):
#     if nozzle is not None:
#         return nozzle
#     machine_obj = get_machine(machine_obj)
#     wanted = str(cfg.get("nozzle_name", "") or "").strip()
#     heads = all_heads(machine_obj)
#     if str(cfg.get("head_name", "") or "").strip():
#         heads = [find_head(machine_obj, cfg.get("head_name"))]
#     for head in heads:
#         if wanted:
#             try:
#                 found = head.getNozzleByName(wanted)
#                 if found is not None:
#                     return found
#             except:
#                 pass
#             try:
#                 for item in java_list_to_py(head.getNozzles()):
#                     if object_name(item) == wanted:
#                         return item
#             except:
#                 pass
#         else:
#             try:
#                 found = head.getDefaultNozzle()
#                 if found is not None:
#                     return found
#             except:
#                 pass
#     raise NeedleCalibrationError("Nozzle not found: %s" % (wanted or "<default>"))


# def find_top_camera(cfg, machine_obj=None):
#     machine_obj = get_machine(machine_obj)
#     wanted = str(cfg.get("top_camera_name", "") or "").strip()
#     if wanted:
#         for head in all_heads(machine_obj):
#             try:
#                 for cam in java_list_to_py(head.getCameras()):
#                     if object_name(cam) == wanted:
#                         return cam
#             except:
#                 pass
#         try:
#             for cam in java_list_to_py(machine_obj.getCameras()):
#                 if object_name(cam) == wanted:
#                     return cam
#         except:
#             pass
#         raise NeedleCalibrationError("Top camera not found: %s" % wanted)
#     try:
#         cam = machine_obj.getDefaultHead().getDefaultCamera()
#         if cam is not None:
#             return cam
#     except:
#         pass
#     for head in all_heads(machine_obj):
#         try:
#             for cam in java_list_to_py(head.getCameras()):
#                 if cam.getLooking() == SpiCamera.Looking.Down:
#                     return cam
#         except:
#             pass
#     raise NeedleCalibrationError("No top camera found")


# def find_actuator_by_name(machine_obj, cfg):
#     name = str(cfg.get("contact_actuator_name", "") or "").strip()
#     if name == "":
#         raise NeedleCalibrationError("contact_actuator_name is required")
#     owners = [machine_obj]
#     try:
#         owners.insert(0, find_nozzle(cfg, None, machine_obj).getHead())
#     except:
#         pass
#     for owner in owners:
#         if owner is None:
#             continue
#         for method in ["getActuatorByName", "getActuator"]:
#             try:
#                 actuator = getattr(owner, method)(name)
#                 if actuator is not None:
#                     return actuator
#             except:
#                 pass
#         try:
#             for actuator in java_list_to_py(owner.getActuators()):
#                 if object_name(actuator) == name:
#                     return actuator
#         except:
#             pass
#     raise NeedleCalibrationError("Contact actuator not found: %s" % name)


# def wait_still(movable):
#     movable.waitForCompletion(CompletionType.WaitForStillstand)


# def move_safe(movable, target):
#     target = target.convertToUnits(LengthUnit.Millimeters)
#     MovableUtils.moveToLocationAtSafeZ(movable, target)
#     wait_still(movable)


# def move_direct_speed(movable, target, speed=None):
#     target = target.convertToUnits(LengthUnit.Millimeters)
#     if speed is None:
#         try:
#             movable.moveTo(target)
#         except TypeError:
#             movable.moveTo(target, 1.0)
#     else:
#         try:
#             movable.moveTo(target, float(speed))
#         except TypeError:
#             movable.moveTo(target)
#     wait_still(movable)


# def get_movable_location(movable):
#     return movable.getLocation().convertToUnits(LengthUnit.Millimeters)


# def lift_nozzle_to_safe_z(nozzle, cfg, logger=None):
#     cur = get_movable_location(nozzle)
#     safe_z = float(cfg.get("safe_z_mm", 25.0))
#     if float(cur.getZ()) >= safe_z:
#         return cur
#     target = mm_loc(cur.getX(), cur.getY(), safe_z, cur.getRotation())
#     log_msg(logger, "Retract to safe Z: %s" % loc_to_log(target))
#     move_direct_speed(nozzle, target, cfg.get("travel_speed", 1.0))
#     return get_movable_location(nozzle)


# def parse_contact_state(value):
#     if value is None:
#         raise NeedleCalibrationError("Contact actuator returned None")
#     if isinstance(value, bool):
#         return not value
#     try:
#         return float(value) == 0.0
#     except:
#         pass
#     text = str(value).strip().lower()
#     if text in ["0", "false", "f", "no", "n", "off", "low", "closed", "contact"]:
#         return True
#     if text in ["1", "true", "t", "yes", "y", "on", "high", "open", "released"]:
#         return False
#     raise NeedleCalibrationError("Cannot parse contact actuator value: %s" % value)


# def read_actuator_raw(actuator):
#     for method in ["read", "getRead", "getValue", "getState", "isActuated"]:
#         try:
#             return getattr(actuator, method)()
#         except:
#             pass
#     raise NeedleCalibrationError("Contact actuator has no readable method: %s" %
#         object_name(actuator))


# def read_contact_once(actuator):
#     return parse_contact_state(read_actuator_raw(actuator))


# def dwell_for_read(read_cfg):
#     ms = int(read_cfg.get("read_settle_ms", 0))
#     if ms > 0:
#         Thread.sleep(ms)


# def read_contact_debounced(actuator, read_cfg, expect_contact=None, logger=None,
#         label="contact", mixed_result=False):
#     count = int(read_cfg.get("debounce_count", 1))
#     if count < 1:
#         count = 1
#     retries = int(read_cfg.get("debounce_retries", 1))
#     if retries < 1:
#         retries = 1
#     interval_ms = int(read_cfg.get("debounce_interval_ms", 0))
#     last = []
#     for attempt in range(retries):
#         readings = []
#         for i in range(count):
#             readings.append(read_contact_once(actuator))
#             if i + 1 < count and interval_ms > 0:
#                 Thread.sleep(interval_ms)
#         stable = True
#         first = readings[0]
#         for item in readings:
#             if item != first:
#                 stable = False
#                 break
#         if stable:
#             if expect_contact is not None and first != bool(expect_contact):
#                 raise NeedleCalibrationError("%s expected contact=%s actual=%s" %
#                     (label, bool(expect_contact), first))
#             return first
#         last = readings
#         log_msg(logger, "%s unstable contact readings attempt %d/%d: %s" %
#             (label, attempt + 1, retries, readings))
#     if expect_contact is not None:
#         raise NeedleCalibrationError("%s expected contact=%s actual readings=%s" %
#             (label, bool(expect_contact), last))
#     return bool(mixed_result)


# def get_attr(obj, names):
#     for name in names:
#         try:
#             if isinstance(obj, dict) and name in obj:
#                 return obj.get(name)
#         except:
#             pass
#         try:
#             return getattr(obj, name)
#         except:
#             pass
#         try:
#             getter = "get" + name[:1].upper() + name[1:]
#             if hasattr(obj, getter):
#                 return getattr(obj, getter)()
#         except:
#             pass
#     return None


# def number_attr(obj, names):
#     value = get_attr(obj, names)
#     if value is None:
#         return None
#     try:
#         return float(value)
#     except:
#         return None


# def is_location(obj):
#     return hasattr(obj, "convertToUnits") and hasattr(obj, "getX") and hasattr(obj, "getY")


# def parse_feature_item(item):
#     if is_location(item):
#         loc = item.convertToUnits(LengthUnit.Millimeters)
#         return {"kind": "location", "x_mm": float(loc.getX()),
#             "y_mm": float(loc.getY()), "z_mm": float(loc.getZ()),
#             "location": loc, "raw": str(item)}
#     center = get_attr(item, ["center", "pt", "point"])
#     if center is not None:
#         x = number_attr(center, ["x"])
#         y = number_attr(center, ["y"])
#     else:
#         x = number_attr(item, ["x"])
#         y = number_attr(item, ["y"])
#     diameter = number_attr(item, ["diameter", "d", "size"])
#     radius = number_attr(item, ["radius", "r"])
#     if diameter is None and radius is not None:
#         diameter = radius * 2.0
#     if x is None or y is None:
#         raise NeedleCalibrationError("Pipeline feature has no readable x/y: %s" % item)
#     out = {"kind": "pixel", "x_px": float(x), "y_px": float(y), "raw": str(item)}
#     if diameter is not None:
#         out["diameter_px"] = float(diameter)
#         out["radius_px"] = float(diameter) / 2.0
#     elif radius is not None:
#         out["radius_px"] = float(radius)
#         out["diameter_px"] = float(radius) * 2.0
#     return out


# def flatten_model(model):
#     out = []
#     for item in java_list_to_py(model):
#         if item is None:
#             continue
#         if is_location(item):
#             out.append(item)
#         elif isinstance(item, list):
#             out.extend(item)
#         elif hasattr(item, "size") and hasattr(item, "get") and "Point" not in str(type(item)):
#             out.extend(java_list_to_py(item))
#         else:
#             out.append(item)
#     return out


# def add_vision_settings_names_from_owner(owner, names):
#     if owner is None:
#         return
#     for method in ["getVisionSettings", "getVisionSettingsList"]:
#         try:
#             for item in java_list_to_py(getattr(owner, method)()):
#                 names.append(object_name(item))
#         except:
#             pass


# def find_named_vision_settings(name):
#     cfg = Configuration.get()
#     wanted = str(name or "").strip()
#     for method in ["getVisionSettings", "getVisionSettingsByName"]:
#         try:
#             value = getattr(cfg, method)(wanted)
#             if value is not None:
#                 return value
#         except:
#             pass
#     for owner in [cfg, get_machine(None)]:
#         for method in ["getVisionSettings", "getVisionSettingsList"]:
#             try:
#                 for item in java_list_to_py(getattr(owner, method)()):
#                     if object_name(item) == wanted:
#                         return item
#             except:
#                 pass
#     return None


# def list_available_vision_pipeline_names(machine_obj=None):
#     names = []
#     try:
#         add_vision_settings_names_from_owner(Configuration.get(), names)
#     except:
#         pass
#     try:
#         machine_obj = get_machine(machine_obj)
#         add_vision_settings_names_from_owner(machine_obj, names)
#         for head in all_heads(machine_obj):
#             add_vision_settings_names_from_owner(head, names)
#             try:
#                 for cam in java_list_to_py(head.getCameras()):
#                     add_vision_settings_names_from_owner(cam, names)
#             except:
#                 pass
#             try:
#                 for nozzle in java_list_to_py(head.getNozzles()):
#                     add_vision_settings_names_from_owner(nozzle, names)
#             except:
#                 pass
#     except:
#         pass
#     seen = {}
#     out = []
#     for name in names:
#         text = str(name or "").strip()
#         if text and text not in seen:
#             seen[text] = True
#             out.append(text)
#     out.sort()
#     return out


# def load_pipeline_from_source(source):
#     mode = str(source.get("mode", "named") or "named").lower()
#     if mode in ["named", "vision_settings", "named_vision_settings"]:
#         name = str(source.get("name", "") or "").strip()
#         if name == "":
#             raise NeedleCalibrationError("Named vision settings source requires name")
#         settings = find_named_vision_settings(name)
#         if settings is None:
#             raise NeedleCalibrationError("Named vision settings not found: %s" % name)
#         pipeline = settings.getPipeline()
#         if pipeline is None:
#             raise NeedleCalibrationError("Named vision settings has no pipeline: %s" % name)
#         return pipeline
#     if mode not in ["xml_path", "file", "path"]:
#         raise NeedleCalibrationError("Unsupported pipeline mode: %s" % mode)
#     path = str(source.get("path", "") or "").strip()
#     if path == "":
#         raise NeedleCalibrationError("Pipeline XML path is empty")
#     if not os.path.isabs(path):
#         path = os.path.join(SCRIPTS_ROOT, path)
#     fh = open(path, "r")
#     try:
#         xml = fh.read()
#     finally:
#         fh.close()
#     return CvPipeline(str(xml))


# def get_result_model(pipeline, stage_name):
#     result = pipeline.getResult(stage_name)
#     if result is None:
#         raise NeedleCalibrationError("No pipeline result from stage '%s'" % stage_name)
#     try:
#         return result.getModel()
#     except:
#         return result


# def add_machine_coordinates_to_feature(camera, nozzle, feat):
#     if feat["kind"] == "pixel":
#         px = feat["x_px"]
#         py = feat["y_px"]
#         loc = VisionUtils.getPixelLocation(camera, nozzle, float(px), float(py))
#         loc = loc.convertToUnits(LengthUnit.Millimeters)
#         feat["x_mm"] = float(loc.getX())
#         feat["y_mm"] = float(loc.getY())
#         feat["z_mm"] = float(loc.getZ())
#         feat["location"] = loc
#         if feat.get("diameter_px", None) is not None:
#             try:
#                 rp = float(feat["diameter_px"]) / 2.0
#                 left = VisionUtils.getPixelLocation(camera, nozzle, px - rp, py).convertToUnits(LengthUnit.Millimeters)
#                 right = VisionUtils.getPixelLocation(camera, nozzle, px + rp, py).convertToUnits(LengthUnit.Millimeters)
#                 bottom = VisionUtils.getPixelLocation(camera, nozzle, px, py - rp).convertToUnits(LengthUnit.Millimeters)
#                 top = VisionUtils.getPixelLocation(camera, nozzle, px, py + rp).convertToUnits(LengthUnit.Millimeters)
#                 dx = math.sqrt(math.pow(right.getX() - left.getX(), 2.0) + math.pow(right.getY() - left.getY(), 2.0))
#                 dy = math.sqrt(math.pow(top.getX() - bottom.getX(), 2.0) + math.pow(top.getY() - bottom.getY(), 2.0))
#                 feat["diameter_mm"] = (dx + dy) / 2.0
#                 feat["radius_mm"] = feat["diameter_mm"] / 2.0
#             except:
#                 pass
#     return feat


# def capture_with_pipeline(cfg, role, fid, nozzle=None, machine_obj=None, logger=None):
#     machine_obj = get_machine(machine_obj)
#     nozzle = find_nozzle(cfg, nozzle, machine_obj)
#     camera = find_top_camera(cfg, machine_obj)
#     target = dict_to_loc(fid.get("location"))
#     log_msg(logger, "Capture %s at %s" % (role, loc_to_log(target)))
#     move_safe(camera, target)
#     pipeline = load_pipeline_from_source(fid.get("pipeline", {}))
#     pipeline.setProperty("camera", camera)
#     pipeline.setProperty("nozzle", nozzle)
#     image = camera.settleAndCapture()
#     pipeline.process()
#     stage_name = fid.get("result_stage", "results")
#     items = flatten_model(get_result_model(pipeline, stage_name))
#     if len(items) == 0:
#         raise NeedleCalibrationError("%s pipeline stage '%s' returned no features" %
#             (role, stage_name))
#     features = []
#     for item in items:
#         features.append(add_machine_coordinates_to_feature(camera, nozzle, parse_feature_item(item)))
#     feature = features[0]
#     out = {
#         "role": role,
#         "camera": object_name(camera),
#         "stage": stage_name,
#         "camera_location_mm": loc_to_dict(get_movable_location(camera)),
#         "features": features,
#         "feature": feature,
#         "center": {"x": float(feature["x_mm"]), "y": float(feature["y_mm"])}
#     }
#     if role == "post_xy":
#         out["vision_center"] = out["center"]
#         out["detected_post_diameter_mm"] = feature.get("diameter_mm", None)
#     if bool(cfg.get("debug", {}).get("save_last_capture", True)):
#         out["image"] = image
#     return out


# def capture_z_touch_center(cfg, nozzle=None, machine_obj=None, logger=None):
#     fid = cfg.get("fiducials", {}).get("z_touch", {})
#     return capture_with_pipeline(cfg, "z_touch", fid, nozzle, machine_obj, logger)


# def capture_post_center(cfg, nozzle=None, machine_obj=None, logger=None):
#     return capture_with_pipeline(cfg, "post_xy", cfg.get("post_xy", {}),
#         nozzle, machine_obj, logger)


# def descend_until_contact(nozzle, actuator, read_cfg, x_mm, y_mm, start_z,
#         limit_z, step_mm, speed, logger=None, label="probe"):
#     z = float(start_z)
#     step = abs(float(step_mm))
#     steps = 0
#     while z - step >= float(limit_z) - 1.0e-9:
#         z -= step
#         steps += 1
#         move_direct_speed(nozzle, mm_loc(x_mm, y_mm, z,
#             get_movable_location(nozzle).getRotation()), speed)
#         dwell_for_read(read_cfg)
#         if read_contact_debounced(actuator, read_cfg, logger=logger, label=label):
#             return get_movable_location(nozzle), steps
#     return None, steps


# def retract_until_released(nozzle, actuator, read_cfg, x_mm, y_mm, step_mm,
#         max_mm, speed, logger=None, label="release"):
#     moved = 0.0
#     while True:
#         dwell_for_read(read_cfg)
#         if not read_contact_debounced(actuator, read_cfg, logger=logger, label=label,
#                 mixed_result=True):
#             return moved
#         if moved >= float(max_mm) - 1.0e-9:
#             raise NeedleCalibrationError("%s: contact did not release after %.6f mm" %
#                 (label, float(max_mm)))
#         inc = min(abs(float(step_mm)), float(max_mm) - moved)
#         moved += inc
#         cur = get_movable_location(nozzle)
#         move_direct_speed(nozzle, mm_loc(x_mm, y_mm, cur.getZ() + inc,
#             cur.getRotation()), speed)


# def probe_z_touch(nozzle, actuator, cfg, x_mm, y_mm, surface_z_mm, logger=None):
#     zp = deep_copy(cfg.get("z_probe", {}))
#     start_z = float(surface_z_mm) + float(zp.get("start_above_mm", 4.0))
#     limit_z = start_z - float(zp.get("max_descent_mm", 8.0))
#     coarse_step = abs(float(zp.get("coarse_step_mm", 0.10)))
#     fine_step = abs(float(zp.get("fine_step_mm", 0.02)))
#     speed = float(cfg.get("probe_speed", 0.20))
#     fine_speed = float(cfg.get("fine_probe_speed", 0.05))
#     retract_after = abs(float(zp.get("retract_after_contact_mm", 0.40)))
#     start = mm_loc(x_mm, y_mm, start_z, get_movable_location(nozzle).getRotation())
#     log_msg(logger, "Z touch move above pad %s" % loc_to_log(start))
#     move_safe(nozzle, start)
#     dwell_for_read(zp)
#     if read_contact_debounced(actuator, zp, logger=logger, label="z_precheck"):
#         raise NeedleCalibrationError("Z probe already active above pad")
#     coarse, coarse_steps = descend_until_contact(nozzle, actuator, zp, x_mm, y_mm,
#         start_z, limit_z, coarse_step, speed, logger, "z_coarse")
#     if coarse is None:
#         raise NeedleCalibrationError("No coarse Z contact within max descent")
#     restart_z = float(coarse.getZ()) + retract_after
#     move_direct_speed(nozzle, mm_loc(x_mm, y_mm, restart_z, coarse.getRotation()), speed)
#     retract_until_released(nozzle, actuator, zp, x_mm, y_mm,
#         float(zp.get("release_retract_step_mm", 0.20)),
#         float(zp.get("max_release_retract_mm", 2.0)), speed, logger, "z_release")
#     fine, fine_steps = descend_until_contact(nozzle, actuator, zp, x_mm, y_mm,
#         get_movable_location(nozzle).getZ(), limit_z, fine_step, fine_speed,
#         logger, "z_fine")
#     if fine is None:
#         raise NeedleCalibrationError("No fine Z contact after coarse contact")
#     move_direct_speed(nozzle, mm_loc(x_mm, y_mm, float(fine.getZ()) + retract_after,
#         fine.getRotation()), speed)
#     extra = retract_until_released(nozzle, actuator, zp, x_mm, y_mm,
#         float(zp.get("release_retract_step_mm", 0.20)),
#         float(zp.get("max_release_retract_mm", 2.0)), speed, logger, "z_release_done")
#     return {
#         "surface_z_mm": float(surface_z_mm),
#         "coarse_contact": loc_to_dict(coarse),
#         "contact": loc_to_dict(fine),
#         "measured_contact_z_mm": float(fine.getZ()),
#         "z_error_mm": float(surface_z_mm) - float(fine.getZ()),
#         "coarse_steps": coarse_steps,
#         "fine_steps": fine_steps,
#         "release_extra_mm": extra
#     }


# def resolve_post_side_probe_z_mm(cfg, logger=None):
#     post = cfg.get("post_xy", {})
#     loc = dict_to_loc(post.get("location", {}))
#     z = float(loc.getZ()) + float(post.get("side_probe_z_offset_mm", 0.25))
#     log_msg(logger, "Post side probe Z %.6f" % z)
#     return z


# def optional_float(value, fallback):
#     if value is None:
#         return float(fallback)
#     try:
#         text = str(value).strip()
#         if text == "" or text.lower() == "none":
#             return float(fallback)
#     except:
#         pass
#     return float(value)


# def config_bool(value, fallback=False):
#     if value is None:
#         return bool(fallback)
#     if isinstance(value, bool):
#         return value
#     text = str(value).strip().lower()
#     if text in ["1", "true", "t", "yes", "y", "on"]:
#         return True
#     if text in ["0", "false", "f", "no", "n", "off"]:
#         return False
#     return bool(fallback)


# def post_xy_settings(cfg):
#     post = cfg.get("post_xy", {})
#     # Compatibility with older two-pass post XY configs.
#     step = abs(float(post.get("step_mm",
#         post.get("final_step_mm", post.get("rough_step_mm", 0.05)))))
#     speed = optional_float(post.get("speed", None), cfg.get("probe_speed", 0.20))
#     if step <= 0:
#         raise NeedleCalibrationError("post_xy step_mm must be > 0")
#     return {"step_mm": step, "speed": float(speed)}


# def post_scan_settings(cfg):
#     post = cfg.get("post_xy", {})
#     clearance = abs(float(post.get("scan_clearance_mm", 2.0)))
#     overtravel = abs(float(post.get("scan_overtravel_mm", 3.0)))
#     cap = abs(float(post.get("max_radial_probe_travel_mm", 5.0)))
#     return {"clearance_mm": clearance, "overtravel_mm": overtravel,
#         "cap_mm": cap, "travel_limit_mm": min(cap, clearance + overtravel)}


# def recover_outside_start_if_touching(nozzle, actuator, read_cfg, start_x, start_y,
#         ux, uy, z_probe, speed, logger=None, label="edge"):
#     dwell_for_read(read_cfg)
#     touching = read_contact_debounced(actuator, read_cfg, logger=logger,
#         label=label + " outside")
#     log_msg(logger, "%s outside pos=%s contact=%s" %
#         (label, fmt_xy(start_x, start_y), touching))
#     if not touching:
#         return get_movable_location(nozzle), 0.0

#     log_msg(logger, "%s outside touching; recovering outward" % label)
#     step = abs(float(read_cfg.get("outside_recover_step_mm", 0.25)))
#     max_outward = abs(float(read_cfg.get("outside_recover_max_mm", 3.0)))
#     if step <= 0.0:
#         raise NeedleCalibrationError("%s: outside_recover_step_mm must be > 0" % label)
#     outward = 0.0
#     index = 0
#     while outward < max_outward - 1.0e-9:
#         outward = min(max_outward, outward + step)
#         index += 1
#         x = float(start_x) + float(ux) * outward
#         y = float(start_y) + float(uy) * outward
#         move_direct_speed(nozzle, mm_loc(x, y, z_probe,
#             get_movable_location(nozzle).getRotation()), speed)
#         dwell_for_read(read_cfg)
#         touching = read_contact_debounced(actuator, read_cfg, logger=logger,
#             label=label + " recover step %d" % index)
#         log_msg(logger, "%s recover step=%d outward=%s pos=%s contact=%s" %
#             (label, index, fmt_float(outward), fmt_xy(x, y), touching))
#         if not touching:
#             return get_movable_location(nozzle), outward

#     cur = get_movable_location(nozzle)
#     lift = abs(float(read_cfg.get("local_lift_mm", 0.60)))
#     move_direct_speed(nozzle, mm_loc(cur.getX(), cur.getY(), z_probe + lift,
#         cur.getRotation()), speed)
#     raise NeedleCalibrationError(
#         "%s: outside start remained touching after outward recovery %.6f mm" %
#         (label, max_outward))


# def move_inward_until_contact(nozzle, actuator, read_cfg, start_x, start_y,
#         ux, uy, max_distance, step_mm, z_probe, speed, logger=None, label="edge"):
#     traveled = 0.0
#     step = abs(float(step_mm))
#     index = 0
#     log_every_step = config_bool(read_cfg.get("log_every_step", False), False)
#     log_step_interval = int(read_cfg.get("log_step_interval", 10))
#     if log_step_interval < 1:
#         log_step_interval = 1

#     while traveled < float(max_distance) - 1.0e-9:
#         traveled = min(float(max_distance), traveled + step)
#         index += 1

#         x = float(start_x) - float(ux) * traveled
#         y = float(start_y) - float(uy) * traveled

#         move_direct_speed(nozzle, mm_loc(x, y, z_probe,
#             get_movable_location(nozzle).getRotation()), speed)

#         dwell_for_read(read_cfg)
#         touching = read_contact_debounced(actuator, read_cfg, logger=logger,
#             label=label + " step %d" % index)

#         if log_every_step or touching or index == 1 or index % log_step_interval == 0:
#             log_msg(logger, "%s step=%d travel=%s pos=%s contact=%s" %
#                 (label, index, fmt_float(traveled), fmt_xy(x, y), touching))

#         if touching:
#             return get_movable_location(nozzle), traveled, index

#     return None, traveled, index


# def move_outward_by(nozzle, start_loc, ux, uy, distance_mm, z_probe, speed):
#     loc = start_loc.convertToUnits(LengthUnit.Millimeters)
#     x = float(loc.getX()) + float(ux) * float(distance_mm)
#     y = float(loc.getY()) + float(uy) * float(distance_mm)
#     move_direct_speed(nozzle, mm_loc(x, y, z_probe, loc.getRotation()), speed)
#     return get_movable_location(nozzle)


# def confirm_contact_point(cfg, nozzle, actuator, read_cfg, first_contact,
#         ux, uy, z_probe, speed, logger=None, label="edge"):
#     backoff = abs(float(read_cfg.get("confirm_backoff_mm", 0.4)))
#     reapproach = abs(float(read_cfg.get("confirm_reapproach_mm", 0.8)))
#     tolerance = abs(float(read_cfg.get("confirm_position_tolerance_mm", 0.15)))
#     reject_early = config_bool(read_cfg.get("confirm_reject_early", True), True)
#     min_confirm_travel = max(0.0, backoff - tolerance)
#     max_confirm_travel = min(reapproach, backoff + tolerance)
#     raw_step = read_cfg.get("confirm_step_mm", None)
#     try:
#         if raw_step is None or str(raw_step).strip() == "" or str(raw_step).strip().lower() == "none":
#             confirm_step = float(post_xy_settings(cfg)["step_mm"])
#         else:
#             confirm_step = abs(float(raw_step))
#     except:
#         confirm_step = float(post_xy_settings(cfg)["step_mm"])
#     if confirm_step <= 0.0:
#         raise NeedleCalibrationError("%s: confirm_step_mm must be > 0" % label)
#     if max_confirm_travel < min_confirm_travel - 1.0e-9:
#         raise NeedleCalibrationError(
#             "%s: confirm_reapproach_mm %.6f is shorter than minimum confirmation travel %.6f" %
#             (label, reapproach, min_confirm_travel))

#     log_msg(logger, "%s first contact=%s backing off=%s" %
#         (label, fmt_xy(first_contact.getX(), first_contact.getY()),
#          fmt_float(backoff)))
#     log_msg(logger, "%s CONFIRM expectedTravel=%s tolerance=%s validWindow=%s..%s" %
#         (label, fmt_float(backoff), fmt_float(tolerance),
#          fmt_float(min_confirm_travel), fmt_float(max_confirm_travel)))
#     backed_off = move_outward_by(nozzle, first_contact, ux, uy, backoff,
#         z_probe, speed)
#     dwell_for_read(read_cfg)
#     touching = read_contact_debounced(actuator, read_cfg, logger=logger,
#         label=label + " confirm backoff")
#     log_msg(logger, "%s CONFIRM backoff=%s pos=%s contact=%s" %
#         (label, fmt_float(backoff), fmt_xy(backed_off.getX(), backed_off.getY()),
#          touching))
#     if touching:
#         raise NeedleCalibrationError(
#             "%s: contact still active after %.6f mm confirmation backoff" %
#             (label, backoff))

#     traveled = 0.0
#     steps = 0
#     while traveled < max_confirm_travel - 1.0e-9:
#         traveled = min(max_confirm_travel, traveled + confirm_step)
#         steps += 1
#         x = float(backed_off.getX()) - float(ux) * traveled
#         y = float(backed_off.getY()) - float(uy) * traveled
#         move_direct_speed(nozzle, mm_loc(x, y, z_probe,
#             backed_off.getRotation()), speed)
#         dwell_for_read(read_cfg)
#         touching = read_contact_debounced(actuator, read_cfg, logger=logger,
#             label=label + " CONFIRM step %d" % steps)
#         log_msg(logger, "%s CONFIRM step=%d travel=%s pos=%s contact=%s" %
#             (label, steps, fmt_float(traveled), fmt_xy(x, y), touching))
#         if touching:
#             if reject_early and traveled < min_confirm_travel - 1.0e-9:
#                 log_msg(logger, "%s CONFIRM FAILED too early travel=%s min=%s" %
#                     (label, fmt_float(traveled), fmt_float(min_confirm_travel)))
#                 raise NeedleCalibrationError(
#                     "%s: confirmation contact too early: confirmTravel %.6f before minimum %.6f; first contact was not repeatable" %
#                     (label, traveled, min_confirm_travel))
#             confirmed = get_movable_location(nozzle)
#             log_msg(logger, "%s CONFIRMED CONTACT pos=%s confirmTravel=%s" %
#                 (label, fmt_xy(confirmed.getX(), confirmed.getY()),
#                  fmt_float(traveled)))
#             return confirmed, float(traveled), int(steps), backed_off

#     raise NeedleCalibrationError(
#         "%s: could not reconfirm contact near first touch; expected %.6f +/- %.6f" %
#         (label, backoff, tolerance))


# def probe_post_edge(cfg, nozzle, actuator, center, angle_deg, configured_radius,
#         z_probe, logger=None):
#     post = cfg.get("post_xy", {})
#     read_cfg = deep_copy(post)
#     settings = post_xy_settings(cfg)
#     scan = post_scan_settings(cfg)
#     theta = math.radians(float(angle_deg))
#     ux = math.cos(theta)
#     uy = math.sin(theta)

#     clearance = scan["clearance_mm"]
#     overtravel = scan["overtravel_mm"]
#     cap = scan["cap_mm"]
#     lift = abs(float(post.get("local_lift_mm", 0.60)))

#     step = float(settings["step_mm"])
#     speed = float(settings["speed"])

#     start_dist = float(configured_radius) + clearance
#     start_x = float(center["x"]) + ux * start_dist
#     start_y = float(center["y"]) + uy * start_dist

#     travel_limit = scan["travel_limit_mm"]

#     label = "EDGE angle=%s" % str(angle_deg)
#     start_ms = System.currentTimeMillis()

#     log_msg(logger, "%s START center=%s start=%s radius=%s clearance=%s overtravel=%s cap=%s travelLimit=%s z=%s step=%s speed=%s" %
#         (label,
#          fmt_xy(center["x"], center["y"]),
#          fmt_xy(start_x, start_y),
#          fmt_float(configured_radius),
#          fmt_float(clearance),
#          fmt_float(overtravel),
#          fmt_float(cap),
#          fmt_float(travel_limit),
#          fmt_float(z_probe),
#          fmt_float(step),
#          fmt_float(speed)))

#     move_safe(nozzle, mm_loc(start_x, start_y, z_probe + lift,
#         get_movable_location(nozzle).getRotation()))

#     move_direct_speed(nozzle, mm_loc(start_x, start_y, z_probe,
#         get_movable_location(nozzle).getRotation()), speed)

#     recovered, outward_recovery = recover_outside_start_if_touching(nozzle,
#         actuator, read_cfg, start_x, start_y, ux, uy, z_probe, speed, logger, label)
#     if outward_recovery > 0.0:
#         start_x = float(recovered.getX())
#         start_y = float(recovered.getY())
#         travel_limit = min(cap + outward_recovery,
#             clearance + overtravel + outward_recovery)
#         log_msg(logger, "%s recovered start=%s outward=%s effectiveTravelLimit=%s" %
#             (label, fmt_xy(start_x, start_y), fmt_float(outward_recovery),
#              fmt_float(travel_limit)))

#     first_contact, travel, steps = move_inward_until_contact(nozzle, actuator,
#         read_cfg, start_x, start_y, ux, uy, travel_limit, step, z_probe,
#         speed, logger, label)

#     if first_contact is None:
#         cur = get_movable_location(nozzle)
#         move_direct_speed(nozzle, mm_loc(cur.getX(), cur.getY(), z_probe + lift,
#             cur.getRotation()), speed)
#         raise NeedleCalibrationError(
#             "%s: no contact after moving inward %.6f mm from start %s toward center %s" %
#             (label, float(travel_limit), fmt_xy(start_x, start_y),
#              fmt_xy(center["x"], center["y"])))

#     log_msg(logger, "%s FIRST CONTACT pos=%s travel=%s steps=%d" %
#         (label, fmt_xy(first_contact.getX(), first_contact.getY()),
#          fmt_float(travel), steps))

#     try:
#         contact, confirm_travel, confirm_steps, backoff_location = confirm_contact_point(
#             cfg, nozzle, actuator, read_cfg, first_contact, ux, uy, z_probe,
#             speed, logger, label)
#     except Throwable as e:
#         raise NeedleCalibrationError("%s failed confirmation: %s" %
#             (label, str(e)))
#     except Exception as e:
#         raise NeedleCalibrationError("%s failed confirmation: %s" %
#             (label, str(e)))

#     elapsed = System.currentTimeMillis() - start_ms

#     log_msg(logger, "%s CONFIRMED CONTACT pos=%s firstTravel=%s confirmTravel=%s" %
#         (label, fmt_xy(contact.getX(), contact.getY()),
#          fmt_float(travel), fmt_float(confirm_travel)))

#     move_direct_speed(nozzle, mm_loc(contact.getX(), contact.getY(),
#         z_probe + lift, contact.getRotation()), speed)
#     log_msg(logger, "%s DONE using confirmed contact" % label)

#     return {
#         "angle_deg": float(angle_deg),
#         "x": float(contact.getX()),
#         "y": float(contact.getY()),
#         "z_probe_mm": float(z_probe),
#         "center_used": {"x": float(center["x"]), "y": float(center["y"])},
#         "configured_post_radius_mm": float(configured_radius),
#         "start": {"x": start_x, "y": start_y},
#         "unit": {"x": ux, "y": uy},
#         "first_contact": loc_to_dict(first_contact),
#         "confirmed_contact": loc_to_dict(contact),
#         "contact": loc_to_dict(contact),
#         "first_travel_mm": float(travel),
#         "confirm_backoff_mm": abs(float(read_cfg.get("confirm_backoff_mm", 0.4))),
#         "confirm_reapproach_mm": abs(float(read_cfg.get("confirm_reapproach_mm", 0.8))),
#         "confirm_position_tolerance_mm": abs(float(read_cfg.get("confirm_position_tolerance_mm", 0.15))),
#         "confirm_reject_early": config_bool(read_cfg.get("confirm_reject_early", True), True),
#         "confirm_travel_mm": float(confirm_travel),
#         "confirm_steps": int(confirm_steps),
#         "backoff_location": loc_to_dict(backoff_location),
#         "travel_mm": float(confirm_travel),
#         "outward_recovery_mm": float(outward_recovery),
#         "effective_travel_limit_mm": float(travel_limit),
#         "steps": int(steps),
#         "elapsed_ms": int(elapsed)
#     }


# def parse_angles(value, fallback):
#     if value is None:
#         return list(fallback)
#     if isinstance(value, basestring):
#         text = str(value).strip()
#         if text == "":
#             return list(fallback)
#         try:
#             value = json.loads(text)
#         except:
#             return [float(part.strip()) for part in text.split(",") if part.strip()]
#     return [float(item) for item in value]


# def solve_3x3(matrix, vector):
#     a = [[float(matrix[r][c]) for c in range(3)] + [float(vector[r])] for r in range(3)]
#     for col in range(3):
#         pivot = col
#         for row in range(col + 1, 3):
#             if abs(a[row][col]) > abs(a[pivot][col]):
#                 pivot = row
#         if abs(a[pivot][col]) < 1.0e-12:
#             raise NeedleCalibrationError("Circle fit normal equations are singular")
#         if pivot != col:
#             tmp = a[col]
#             a[col] = a[pivot]
#             a[pivot] = tmp
#         div = a[col][col]
#         for c in range(col, 4):
#             a[col][c] = a[col][c] / div
#         for row in range(3):
#             if row == col:
#                 continue
#             factor = a[row][col]
#             for c in range(col, 4):
#                 a[row][c] = a[row][c] - factor * a[col][c]
#     return [a[0][3], a[1][3], a[2][3]]


# def circle_fit_contacts(contacts):
#     if len(contacts) < 3:
#         raise NeedleCalibrationError("Circle fit requires at least three contacts")
#     ata = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
#     atb = [0.0, 0.0, 0.0]
#     for p in contacts:
#         x = float(p["x"])
#         y = float(p["y"])
#         row = [x, y, 1.0]
#         b = -(x * x + y * y)
#         for r in range(3):
#             atb[r] += row[r] * b
#             for c in range(3):
#                 ata[r][c] += row[r] * row[c]
#     a, b, c = solve_3x3(ata, atb)
#     cx = -a / 2.0
#     cy = -b / 2.0
#     rr = (a * a + b * b) / 4.0 - c
#     if rr <= 0.0:
#         raise NeedleCalibrationError("Circle fit produced non-positive radius")
#     radius = math.sqrt(rr)
#     residuals = []
#     sum_sq = 0.0
#     max_err = 0.0
#     for p in contacts:
#         d = math.sqrt(math.pow(float(p["x"]) - cx, 2.0) + math.pow(float(p["y"]) - cy, 2.0))
#         err = d - radius
#         residuals.append(err)
#         sum_sq += err * err
#         max_err = max(max_err, abs(err))
#     return {
#         "center": {"x": cx, "y": cy},
#         "radius_mm": radius,
#         "rms_error_mm": math.sqrt(sum_sq / float(len(contacts))),
#         "max_error_mm": max_err,
#         "residuals_mm": residuals,
#         "point_count": len(contacts)
#     }


# def fail_payload(stage, message, nozzle=None, cfg=None, logger=None, old_offsets=None):
#     log_msg(logger, "Needle calibration failed: %s" % message)
#     if nozzle is not None and cfg is not None:
#         try:
#             lift_nozzle_to_safe_z(nozzle, cfg, logger)
#         except:
#             pass
#     current = None
#     try:
#         if nozzle is not None:
#             current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
#     except:
#         current = old_offsets
#     return {"ok": False, "stage": stage, "message": str(message),
#         "old_offsets": old_offsets, "current_offsets": current,
#         "proposal": None, "details": {}}


# def make_z_proposal(cfg, old, z_error):
#     old = old.convertToUnits(LengthUnit.Millimeters)
#     dz = float(cfg.get("z_offset_sign", 1.0)) * float(z_error)
#     new = mm_loc(old.getX(), old.getY(), old.getZ() + dz, old.getRotation())
#     return {"type": "z", "measured_error": {"z": float(z_error)},
#         "offset_delta": {"x": 0.0, "y": 0.0, "z": dz},
#         "proposed_offsets": loc_to_dict(new)}


# def make_xy_proposal(cfg, old, dx, dy):
#     old = old.convertToUnits(LengthUnit.Millimeters)
#     sign = float(cfg.get("xy_offset_sign", 1.0))
#     delta_x = -sign * float(dx)
#     delta_y = -sign * float(dy)
#     new = mm_loc(old.getX() + delta_x, old.getY() + delta_y,
#         old.getZ(), old.getRotation())
#     return {"type": "xy", "measured_error": {"x": float(dx), "y": float(dy)},
#         "offset_delta": {"x": delta_x, "y": delta_y, "z": 0.0},
#         "proposed_offsets": loc_to_dict(new)}


# def run_z_measure(cfg, nozzle=None, machine_obj=None, logger=None):
#     stage = "z"
#     old_offsets = None
#     try:
#         machine_obj = get_machine(machine_obj)
#         nozzle = find_nozzle(cfg, nozzle, machine_obj)
#         actuator = find_actuator_by_name(machine_obj, cfg)
#         old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
#         old_offsets = loc_to_dict(old)
#         capture = capture_z_touch_center(cfg, nozzle, machine_obj, logger)
#         center = capture.get("center")
#         surface_z = float(dict_to_loc(cfg.get("fiducials", {}).get("z_touch", {}).get("location", {})).getZ())
#         touch = probe_z_touch(nozzle, actuator, cfg, center["x"], center["y"], surface_z, logger)
#         proposal = make_z_proposal(cfg, old, touch["z_error_mm"])
#         current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
#         return {"ok": True, "stage": "z", "message": "ok",
#             "old_offsets": old_offsets, "current_offsets": current,
#             "proposal": proposal,
#             "details": {"capture": capture, "touch": touch, "center": center}}
#     except Throwable as e:
#         return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)
#     except Exception as e:
#         return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)


# def probe_8_point_post_xy(cfg, nozzle, actuator, vision_center, configured_radius,
#         z_probe, logger=None):
#     post = cfg.get("post_xy", {})
#     angles = parse_angles(post.get("angles_deg"),
#         [0, 45, 90, 135, 180, 225, 270, 315])
#     if len(angles) != 8:
#         raise NeedleCalibrationError("Post XY requires exactly 8 angles; got %d" %
#             len(angles))
#     all_edges = []
#     valid_contacts = []
#     invalid_edges = []
#     try:
#         for angle in angles:
#             try:
#                 edge = probe_post_edge(cfg, nozzle, actuator, vision_center,
#                     angle, configured_radius, z_probe, logger)
#                 edge["valid"] = True
#                 all_edges.append(edge)
#                 valid_contacts.append(edge)
#             except NeedleCalibrationError as e:
#                 text = str(e)
#                 stage = "edge"
#                 lower = text.lower()
#                 if "confirm" in lower:
#                     stage = "confirmation"
#                 elif "outside" in lower or "recover" in lower:
#                     stage = "outside_recovery"
#                 elif "no contact" in lower:
#                     stage = "first_contact"
#                 invalid = {"angle_deg": float(angle), "valid": False,
#                     "error": text, "stage": stage}
#                 all_edges.append(invalid)
#                 invalid_edges.append(invalid)
#                 log_msg(logger, "EDGE angle=%s INVALID reason=%s" %
#                     (str(angle), text))
#                 try:
#                     lift_nozzle_to_safe_z(nozzle, cfg, logger)
#                 except:
#                     pass
#         try:
#             lift_nozzle_to_safe_z(nozzle, cfg, logger)
#         except:
#             pass
#     except:
#         try:
#             lift_nozzle_to_safe_z(nozzle, cfg, logger)
#         except:
#             pass
#         raise
#     min_valid = int(float(post.get("min_valid_contacts_for_fit", 5)))
#     if min_valid < 3:
#         min_valid = 3
#     fit = None
#     if len(valid_contacts) >= min_valid:
#         fit = circle_fit_contacts(valid_contacts)
#     return {"angles": angles, "all_edges": all_edges,
#         "contacts": valid_contacts, "invalid_edges": invalid_edges,
#         "valid_contact_count": len(valid_contacts),
#         "invalid_contact_count": len(invalid_edges),
#         "min_valid_contacts_for_fit": min_valid,
#         "fit": fit}


# def run_post_xy_measure(cfg, nozzle=None, machine_obj=None, logger=None):
#     stage = "post_xy"
#     old_offsets = None
#     try:
#         machine_obj = get_machine(machine_obj)
#         nozzle = find_nozzle(cfg, nozzle, machine_obj)
#         actuator = find_actuator_by_name(machine_obj, cfg)
#         old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
#         old_offsets = loc_to_dict(old)
#         capture = capture_post_center(cfg, nozzle, machine_obj, logger)
#         vision_center = capture.get("vision_center")
#         post = cfg.get("post_xy", {})
#         configured_diameter = float(post.get("post_diameter_mm", 6.35))
#         configured_radius = configured_diameter / 2.0
#         z_probe = resolve_post_side_probe_z_mm(cfg, logger)
#         angles = parse_angles(post.get("angles_deg"),
#             [0, 45, 90, 135, 180, 225, 270, 315])
#         settings = post_xy_settings(cfg)
#         scan = post_scan_settings(cfg)
#         log_section(logger, "POST XY 8-POINT MEASURE START")
#         log_kv(logger, "nozzle name", object_name(nozzle))
#         log_kv(logger, "old offsets", fmt_loc(old_offsets))
#         log_kv(logger, "vision center", fmt_xy(vision_center["x"], vision_center["y"]))
#         log_kv(logger, "configured post diameter/radius",
#             "%s/%s" % (fmt_float(configured_diameter), fmt_float(configured_radius)))
#         log_kv(logger, "side probe z", fmt_float(z_probe))
#         log_kv(logger, "angles", angles)
#         log_kv(logger, "step/speed", "step=%s speed=%s" %
#             (fmt_float(settings["step_mm"]), fmt_float(settings["speed"])))
#         log_kv(logger, "contact read", "debounce_count=%s retries=%s interval_ms=%s" %
#             (int(float(post.get("debounce_count", 3))),
#              int(float(post.get("debounce_retries", 2))),
#              int(float(post.get("debounce_interval_ms", 5)))))
#         log_kv(logger, "scan", "clearance=%s overtravel=%s cap=%s travelLimit=%s" %
#             (fmt_float(scan["clearance_mm"]), fmt_float(scan["overtravel_mm"]),
#              fmt_float(scan["cap_mm"]), fmt_float(scan["travel_limit_mm"])))

#         result = probe_8_point_post_xy(cfg, nozzle, actuator, vision_center,
#             configured_radius, z_probe, logger)
#         fit = result["fit"]
#         valid_count = int(result.get("valid_contact_count", 0))
#         invalid_count = int(result.get("invalid_contact_count", 0))
#         min_valid = int(result.get("min_valid_contacts_for_fit",
#             post.get("min_valid_contacts_for_fit", 5)))
#         log_msg(logger, "POST XY EDGE SUMMARY valid=%d invalid=%d minRequired=%d" %
#             (valid_count, invalid_count, min_valid))
#         for invalid in result.get("invalid_edges", []):
#             log_msg(logger, "INVALID EDGE angle=%s reason=%s" %
#                 (invalid.get("angle_deg", ""), invalid.get("error", "")))
#         base_details = {"vision_center": vision_center,
#             "all_edges": result.get("all_edges", []),
#             "contacts": result.get("contacts", []),
#             "invalid_edges": result.get("invalid_edges", []),
#             "valid_contact_count": valid_count,
#             "invalid_contact_count": invalid_count,
#             "min_valid_contacts_for_fit": min_valid,
#             "post_side_probe_z_mm": z_probe,
#             "configured_post_radius_mm": configured_radius,
#             "configured_post_diameter_mm": configured_diameter,
#             "camera_capture": capture}
#         if fit is None:
#             current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
#             return {"ok": False, "stage": stage,
#                 "message": "only %d valid contacts, need at least %d" %
#                     (valid_count, min_valid),
#                 "old_offsets": old_offsets, "current_offsets": current,
#                 "proposal": None, "details": base_details}
#         fitted_center = fit["center"]
#         fitted_radius = float(fit["radius_mm"])
#         fitted_diameter = 2.0 * fitted_radius
#         effective_probe_radius = fitted_radius - configured_radius
#         effective_probe_diameter = 2.0 * effective_probe_radius
#         dx = float(fitted_center["x"]) - float(vision_center["x"])
#         dy = float(fitted_center["y"]) - float(vision_center["y"])
#         proposal = make_xy_proposal(cfg, old, dx, dy)
#         details = dict(base_details)
#         details.update({"fit": fit,
#             "fitted_contact_radius_mm": fitted_radius,
#             "fitted_contact_diameter_mm": fitted_diameter,
#             "estimated_effective_probe_radius_mm": effective_probe_radius,
#             "estimated_effective_probe_diameter_mm": effective_probe_diameter,
#             "center_delta": {"x": dx, "y": dy}})

#         log_msg(logger, "POST XY FIT center=%s radius=%s diameter=%s rms=%s max=%s" %
#             (fmt_xy(fitted_center["x"], fitted_center["y"]),
#              fmt_float(fitted_radius), fmt_float(fitted_diameter),
#              fmt_float(fit.get("rms_error_mm", 0.0)),
#              fmt_float(fit.get("max_error_mm", 0.0))))
#         log_msg(logger, "POST XY CENTER DELTA from vision=%s" % fmt_xy(dx, dy))
#         log_msg(logger, "POST XY CONTACT DIAMETER fitted=%s configured_post=%s" %
#             (fmt_float(fitted_diameter), fmt_float(configured_diameter)))
#         log_msg(logger, "POST XY ESTIMATED PROBE/NEEDLE diameter=%s radius=%s" %
#             (fmt_float(effective_probe_diameter), fmt_float(effective_probe_radius)))
#         log_msg(logger, "PROPOSED XY offset_delta=%s proposed_offsets=%s" %
#             (fmt_xy(proposal["offset_delta"]["x"], proposal["offset_delta"]["y"]),
#              fmt_loc(proposal["proposed_offsets"])))

#         fit_rms = float(fit["rms_error_mm"])
#         fit_max = float(fit["max_error_mm"])
#         max_fit_rms = float(post.get("max_fit_rms_mm", 0.15))
#         max_fit_error = float(post.get("max_fit_error_mm", 0.30))
#         max_center = float(post.get("max_center_error_mm", 2.0))
#         if fit_rms > max_fit_rms:
#             return {"ok": False, "stage": stage,
#                 "message": "fit RMS %.6f exceeds %.6f" % (fit_rms, max_fit_rms),
#                 "old_offsets": old_offsets,
#                 "current_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)),
#                 "proposal": None, "details": details}
#         if fit_max > max_fit_error:
#             return {"ok": False, "stage": stage,
#                 "message": "fit max error %.6f exceeds %.6f" %
#                     (fit_max, max_fit_error),
#                 "old_offsets": old_offsets,
#                 "current_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)),
#                 "proposal": None, "details": details}
#         if abs(dx) > max_center or abs(dy) > max_center:
#             return {"ok": False, "stage": stage,
#                 "message": "center delta X/Y %.6f/%.6f exceeds %.6f" %
#                     (dx, dy, max_center),
#                 "old_offsets": old_offsets,
#                 "current_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)),
#                 "proposal": None, "details": details}
#         current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
#         return {"ok": True, "stage": stage, "message": "ok",
#             "old_offsets": old_offsets, "current_offsets": current,
#             "proposal": proposal, "details": details}
#     except Throwable as e:
#         return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)
#     except Exception as e:
#         return fail_payload(stage, e, nozzle, cfg, logger, old_offsets)


# def run_full_measure(cfg, nozzle=None, machine_obj=None, logger=None):
#     old_offsets = None
#     z = None
#     post = None
#     try:
#         machine_obj = get_machine(machine_obj)
#         nozzle = find_nozzle(cfg, nozzle, machine_obj)
#         old = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
#         old_offsets = loc_to_dict(old)
#         z = run_z_measure(cfg, nozzle, machine_obj, logger)
#         if not z.get("ok", False):
#             raise NeedleCalibrationError("Z measure failed: %s" % z.get("message"))
#         post = run_post_xy_measure(cfg, nozzle, machine_obj, logger)
#         if not post.get("ok", False):
#             raise NeedleCalibrationError("Post XY measure failed: %s" % post.get("message"))
#         zd = z["proposal"]["offset_delta"]
#         xd = post["proposal"]["offset_delta"]
#         proposed = mm_loc(old.getX() + float(xd.get("x", 0.0)),
#             old.getY() + float(xd.get("y", 0.0)),
#             old.getZ() + float(zd.get("z", 0.0)),
#             old.getRotation())
#         proposal = {"type": "xyz",
#             "offset_delta": {"x": float(xd.get("x", 0.0)),
#                 "y": float(xd.get("y", 0.0)), "z": float(zd.get("z", 0.0))},
#             "proposed_offsets": loc_to_dict(proposed)}
#         current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
#         return {"ok": True, "stage": "full", "message": "ok",
#             "old_offsets": old_offsets, "current_offsets": current,
#             "z": z, "post_xy": post, "proposal": proposal}
#     except Throwable as e:
#         return {"ok": False, "stage": "full", "message": str(e),
#             "old_offsets": old_offsets,
#             "current_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)) if nozzle is not None else old_offsets,
#             "z": z, "post_xy": post, "proposal": None}
#     except Exception as e:
#         return {"ok": False, "stage": "full", "message": str(e),
#             "old_offsets": old_offsets,
#             "current_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)) if nozzle is not None else old_offsets,
#             "z": z, "post_xy": post, "proposal": None}


# def check_apply_delta(cfg, current, proposed, axes):
#     max_delta = float(cfg.get("max_apply_delta_mm", 5.0))
#     for axis in axes:
#         before = float(current[axis])
#         after = float(proposed[axis])
#         if abs(after - before) > max_delta:
#             raise NeedleCalibrationError("%s offset delta %.6f exceeds max_apply_delta_mm %.6f" %
#                 (axis.upper(), abs(after - before), max_delta))


# def set_offsets_from_dict(nozzle, offsets):
#     nozzle.setHeadOffsets(mm_loc(offsets.get("x", 0.0), offsets.get("y", 0.0),
#         offsets.get("z", 0.0), offsets.get("rotation", 0.0)))


# def apply_z(cfg, nozzle, proposal, logger=None):
#     proposed = proposal.get("proposed_offsets", proposal)
#     current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
#     target = deep_copy(current)
#     target["z"] = float(proposed["z"])
#     check_apply_delta(cfg, current, target, ["z"])
#     log_msg(logger, "Apply Z offset old=%s new=%s" % (current, target))
#     set_offsets_from_dict(nozzle, target)
#     return {"ok": True, "stage": "apply_z", "old_offsets": current,
#         "new_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))}


# def apply_xy(cfg, nozzle, proposal, logger=None):
#     proposed = proposal.get("proposed_offsets", proposal)
#     current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
#     target = deep_copy(current)
#     target["x"] = float(proposed["x"])
#     target["y"] = float(proposed["y"])
#     check_apply_delta(cfg, current, target, ["x", "y"])
#     log_msg(logger, "Apply XY offset old=%s new=%s" % (current, target))
#     set_offsets_from_dict(nozzle, target)
#     return {"ok": True, "stage": "apply_xy", "old_offsets": current,
#         "new_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))}


# def apply_xyz(cfg, nozzle, proposal, logger=None):
#     proposed = proposal.get("proposed_offsets", proposal)
#     current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
#     target = deep_copy(current)
#     target["x"] = float(proposed["x"])
#     target["y"] = float(proposed["y"])
#     target["z"] = float(proposed["z"])
#     check_apply_delta(cfg, current, target, ["x", "y", "z"])
#     log_msg(logger, "Apply XYZ offset old=%s new=%s" % (current, target))
#     set_offsets_from_dict(nozzle, target)
#     return {"ok": True, "stage": "apply_xyz", "old_offsets": current,
#         "new_offsets": loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))}


# def revert_offsets(nozzle, offsets, logger=None):
#     current = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
#     set_offsets_from_dict(nozzle, offsets)
#     new_offsets = loc_to_dict(nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
#     log_msg(logger, "Revert offsets old=%s restored=%s" % (current, new_offsets))
#     return {"ok": True, "stage": "revert_offsets",
#         "old_offsets": current, "new_offsets": new_offsets}


# if __name__ == "__main__":
#     cfg = load_config()
#     log_msg(None, "Loaded needle calibration config: %s" % cfg.get("_config_path"))
