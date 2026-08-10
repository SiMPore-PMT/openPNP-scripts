# adhesive_calibration.py
#
# Lean adhesive dispense calibration routine for OpenPnP Jython 2.7.

import json
import math
import os

from java.lang import Boolean, Class, Object, Thread
from jarray import array
from org.openpnp.model import Configuration, LengthUnit, Location
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import MovableUtils


TAG = "adhesive_calibration"


try:
    basestring
except NameError:
    basestring = str


try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except:
    SCRIPT_DIR = os.getcwd()
SCRIPTS_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
DEFAULT_CONFIG_PATH = os.path.join(SCRIPTS_ROOT, "adhesive_calibration_config.json")


DEFAULT_CONFIG = {
    "schema_version": 1,
    "nozzle_name": "Dispense Head",
    "actuator_name": "GlueDispense",
    "origin_location": {"x": 30.0, "y": 40.0, "z": 5.0, "rotation": 0.0},
    "safe_z_mm": 25.0,
    "safe_move_mode": "split_direct",
    "travel_speed": 1.0,
    "dispense_speed": 0.5,
    "dispense_z_offset_mm": 0.1,
    "pre_trigger_dwell_ms": 0,
    "trigger_pulse_ms": 50,
    "actuator_settle_ms": 50,
    "line": {
        "enabled": True,
        "length_mm": 20.0,
        "pitch_mm": 0.5,
        "lift_mm": 1.0,
        "post_trigger_dwell_ms": 100
    },
    "matrix": {
        "enabled": True,
        "offset_x_mm": 5.0,
        "rows": 5,
        "columns": 5,
        "pitch_x_mm": 1.0,
        "pitch_y_mm": 1.0,
        "lift_mm": 3.0,
        "post_trigger_dwell_ms": 100
    },
    "park_location": {
        "enabled": False,
        "x": 30.0,
        "y": 40.0,
        "z": 25.0,
        "rotation": 0.0
    }
}


class AdhesiveCalibrationError(Exception):
    pass


def log_msg(logger, msg):
    text = "[%s] %s" % (TAG, str(msg))
    if logger:
        logger(text)
    else:
        print text


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


def migrate_config(cfg):
    out = deep_copy(cfg)
    if not isinstance(out, dict):
        return out
    if "post_trigger_dwell_ms" in out:
        old_value = out.get("post_trigger_dwell_ms")
        for section in ["line", "matrix"]:
            if not isinstance(out.get(section), dict):
                out[section] = {}
            if "post_trigger_dwell_ms" not in out[section]:
                out[section]["post_trigger_dwell_ms"] = old_value
    return out


def public_clean_config(cfg):
    clean = clean_to_schema(DEFAULT_CONFIG, migrate_config(cfg))
    clean["schema_version"] = 1
    return clean


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
    cfg = public_clean_config(loaded)
    cfg["_config_path"] = cfg_path
    return cfg


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


def dict_to_loc(data):
    return mm_loc(
        data.get("x", 0.0),
        data.get("y", 0.0),
        data.get("z", 0.0),
        data.get("rotation", data.get("r", 0.0)))


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
    return "X %.4f Y %.4f Z %.4f R %.4f" % (
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


def unique_sorted(values):
    seen = {}
    out = []
    for value in values:
        text = str(value or "").strip()
        if text and text not in seen:
            seen[text] = True
            out.append(text)
    out.sort()
    return out


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


def find_nozzle(cfg, nozzle=None, machine_obj=None):
    if nozzle is not None:
        return nozzle
    machine_obj = get_machine(machine_obj)
    wanted = str(cfg.get("nozzle_name", "") or "").strip()
    for head in all_heads(machine_obj):
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
    raise AdhesiveCalibrationError("Nozzle not found: %s" % (wanted or "<default>"))


def find_actuator(cfg, nozzle=None, machine_obj=None):
    machine_obj = get_machine(machine_obj)
    wanted = str(cfg.get("actuator_name", "") or "").strip()
    if wanted == "":
        raise AdhesiveCalibrationError("actuator_name is required")
    owners = []
    try:
        if nozzle is not None:
            owners.append(nozzle.getHead())
    except:
        pass
    try:
        owners.append(machine_obj.getDefaultHead())
    except:
        pass
    owners.append(machine_obj)

    wanted_lower = wanted.lower()
    if wanted_lower in ("nitrogen", "n2"):
        raise AdhesiveCalibrationError(
            "actuator_name must be GlueDispense, not %s" % wanted)

    alias_names = []
    target_names = [wanted_lower]
    for alias in alias_names:
        target_names.append(str(alias).strip().lower())

    available = []
    seen = {}
    for owner in owners:
        if owner is None:
            continue
        oid = str(owner)
        if oid in seen:
            continue
        seen[oid] = True
        for method in ["getActuatorByName", "getActuator"]:
            for candidate in [wanted] + alias_names:
                try:
                    actuator = getattr(owner, method)(candidate)
                    if actuator is not None:
                        return actuator
                except:
                    pass
        try:
            for actuator in java_list_to_py(owner.getActuators()):
                name = object_name(actuator)
                aid = ""
                try:
                    aid = str(actuator.getId())
                except:
                    pass
                if name:
                    available.append(name)
                if aid:
                    available.append(aid)
                values = [name, aid]
                matched = False
                for value in values:
                    if str(value or "").strip().lower() in target_names:
                        matched = True
                        break
                if matched:
                    return actuator
        except:
            pass
    names = unique_sorted(available)
    suffix = ""
    if len(names):
        suffix = ". Available actuators: %s" % ", ".join(names)
    raise AdhesiveCalibrationError("Actuator not found: %s%s" % (wanted, suffix))


def wait_still(movable):
    movable.waitForCompletion(CompletionType.WaitForStillstand)


def move_safe(movable, target):
    target = target.convertToUnits(LengthUnit.Millimeters)
    MovableUtils.moveToLocationAtSafeZ(movable, target)
    wait_still(movable)


def move_split_safe(movable, target, cfg, logger=None, label="safe move"):
    target = target.convertToUnits(LengthUnit.Millimeters)
    current = movable.getLocation().convertToUnits(LengthUnit.Millimeters)
    safe_z = float(cfg.get("safe_z_mm", 25.0))
    travel_z = max(float(current.getZ()), safe_z, float(target.getZ()))
    travel_speed = float(cfg.get("travel_speed", 1.0))

    if abs(float(current.getZ()) - travel_z) > 0.0001:
        lift = mm_loc(current.getX(), current.getY(), travel_z, current.getRotation())
        log_msg(logger, "%s lift: %s" % (label, loc_to_log(lift)))
        move_direct_speed(movable, lift, travel_speed)

    xy = mm_loc(target.getX(), target.getY(), travel_z, target.getRotation())
    log_msg(logger, "%s XY: %s" % (label, loc_to_log(xy)))
    move_direct_speed(movable, xy, travel_speed)

    if abs(travel_z - float(target.getZ())) > 0.0001:
        final = mm_loc(target.getX(), target.getY(), target.getZ(), target.getRotation())
        log_msg(logger, "%s final Z: %s" % (label, loc_to_log(final)))
        move_direct_speed(movable, final, travel_speed)


def move_configured_safe(movable, target, cfg, logger=None, label="safe move"):
    mode = str(cfg.get("safe_move_mode", "split_direct") or "split_direct").strip().lower()
    if mode in ["openpnp", "openpnp_safe_z", "movableutils"]:
        log_msg(logger, "%s using OpenPnP safe-Z planner: %s" % (label, loc_to_log(target)))
        move_safe(movable, target)
    else:
        move_split_safe(movable, target, cfg, logger, label)


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


def set_actuator_state(actuator, on_off):
    if actuator is None:
        return False
    value = Boolean.TRUE if bool(on_off) else Boolean.FALSE
    errors = []

    # Jython overload resolution can select actuate(double) for plain Python
    # bools/numbers. Force the Java primitive boolean overload used by the UI.
    for method_name in ("actuate", "setActuated"):
        try:
            method = actuator.getClass().getMethod(
                method_name,
                array([Boolean.TYPE], Class)
            )
            method.invoke(actuator, array([value], Object))
            return True
        except Exception, e:
            errors.append("%s(boolean): %s" % (method_name, e))

    raise AdhesiveCalibrationError(
        "Actuator has no invokable boolean state method. Tried %s" %
        "; ".join(errors)
    )


def wait_after_actuator_change(actuator, machine_obj=None, settle_ms=50, logger=None):
    if actuator is not None:
        try:
            actuator.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
            return True
        except:
            pass
    machine_obj = get_machine(machine_obj)
    if machine_obj is not None:
        try:
            machine_obj.getMotionPlanner().waitForCompletion(
                None, CompletionType.WaitForUnconditionalCoordination)
            return True
        except:
            pass
    settle_ms = int(settle_ms)
    if settle_ms > 0:
        Thread.sleep(settle_ms)
    return False


def sleep_ms(ms):
    ms = int(ms)
    if ms > 0:
        Thread.sleep(ms)


def bool_value(value):
    if isinstance(value, basestring):
        return value.strip().lower() in ["1", "true", "yes", "on"]
    return bool(value)


def require_non_negative(name, value):
    value = float(value)
    if value < 0.0:
        raise AdhesiveCalibrationError("%s must be non-negative" % name)
    return value


def require_positive(name, value):
    value = float(value)
    if value <= 0.0:
        raise AdhesiveCalibrationError("%s must be positive" % name)
    return value


def int_value(name, value):
    try:
        out = int(value)
    except:
        raise AdhesiveCalibrationError("%s must be an integer" % name)
    if out < 0:
        raise AdhesiveCalibrationError("%s must be >= 0" % name)
    return out


def line_dot_count(line_cfg):
    if not bool_value(line_cfg.get("enabled", True)):
        return 0
    length = require_non_negative("line.length_mm", line_cfg.get("length_mm", 0.0))
    if length == 0.0:
        return 1
    pitch = require_positive("line.pitch_mm", line_cfg.get("pitch_mm", 0.0))
    return int(math.floor(length / pitch)) + 1


def matrix_dot_count(matrix_cfg):
    if not bool_value(matrix_cfg.get("enabled", True)):
        return 0
    rows = int_value("matrix.rows", matrix_cfg.get("rows", 0))
    cols = int_value("matrix.columns", matrix_cfg.get("columns", 0))
    if rows == 0 or cols == 0:
        return 0
    if cols > 1:
        require_positive("matrix.pitch_x_mm", matrix_cfg.get("pitch_x_mm", 0.0))
    if rows > 1:
        require_positive("matrix.pitch_y_mm", matrix_cfg.get("pitch_y_mm", 0.0))
    return rows * cols


def validate_config(cfg):
    cfg = public_clean_config(cfg)
    origin = cfg.get("origin_location", {})
    dispense_z = float(origin.get("z", 0.0)) + require_non_negative(
        "dispense_z_offset_mm", cfg.get("dispense_z_offset_mm", 0.0))
    safe_z = float(cfg.get("safe_z_mm", 0.0))
    mode = str(cfg.get("safe_move_mode", "split_direct") or "split_direct").strip().lower()
    if mode not in ["split_direct", "direct", "openpnp", "openpnp_safe_z", "movableutils"]:
        raise AdhesiveCalibrationError(
            "safe_move_mode must be split_direct or openpnp")
    require_positive("travel_speed", cfg.get("travel_speed", 1.0))
    require_positive("dispense_speed", cfg.get("dispense_speed", 0.5))
    require_non_negative("pre_trigger_dwell_ms", cfg.get("pre_trigger_dwell_ms", 0))
    require_positive("trigger_pulse_ms", cfg.get("trigger_pulse_ms", 50))
    require_non_negative("actuator_settle_ms", cfg.get("actuator_settle_ms", 0))

    line = cfg.get("line", {})
    matrix = cfg.get("matrix", {})
    line_count = line_dot_count(line)
    matrix_count = matrix_dot_count(matrix)
    require_non_negative("line.lift_mm", line.get("lift_mm", 0.0))
    require_non_negative(
        "line.post_trigger_dwell_ms", line.get("post_trigger_dwell_ms", 0))
    require_non_negative("matrix.offset_x_mm", matrix.get("offset_x_mm", 0.0))
    require_non_negative("matrix.lift_mm", matrix.get("lift_mm", 0.0))
    require_non_negative(
        "matrix.post_trigger_dwell_ms", matrix.get("post_trigger_dwell_ms", 0))
    if line_count + matrix_count == 0:
        raise AdhesiveCalibrationError("At least one line or matrix dot is required")

    max_lift = 0.0
    if line_count > 0:
        max_lift = max(max_lift, float(line.get("lift_mm", 0.0)))
    if matrix_count > 0:
        max_lift = max(max_lift, float(matrix.get("lift_mm", 0.0)))
    if safe_z < dispense_z + max_lift:
        raise AdhesiveCalibrationError(
            "safe_z_mm must be >= dispense_z plus max lift (%.4f)" %
            (dispense_z + max_lift))
    return {
        "cfg": cfg,
        "dispense_z": dispense_z,
        "line_count": line_count,
        "matrix_count": matrix_count
    }


def dispense_one_dot(nozzle, actuator, machine_obj, x, y, dispense_z, lift_mm,
        rotation, cfg, post_trigger_dwell_ms, logger=None):
    lifted_z = dispense_z + float(lift_mm)
    travel_speed = float(cfg.get("travel_speed", 1.0))
    dispense_speed = float(cfg.get("dispense_speed", 0.5))
    settle_ms = int(cfg.get("actuator_settle_ms", 50))

    move_direct_speed(nozzle, mm_loc(x, y, lifted_z, rotation), travel_speed)
    move_direct_speed(nozzle, mm_loc(x, y, dispense_z, rotation), dispense_speed)

    dwell = int(cfg.get("pre_trigger_dwell_ms", 0))
    sleep_ms(dwell)

    # Force a clean low state before every dot. The pressure unit needs a
    # low-to-high edge; repeated "on" commands alone will not retrigger it.
    set_actuator_state(actuator, False)
    wait_after_actuator_change(actuator, machine_obj, settle_ms, logger)
    sleep_ms(settle_ms)

    log_msg(logger, "Trigger ON")
    set_actuator_state(actuator, True)
    wait_after_actuator_change(actuator, machine_obj, settle_ms, logger)

    pulse = int(cfg.get("trigger_pulse_ms", 0))
    sleep_ms(pulse)

    log_msg(logger, "Trigger OFF after %d ms" % pulse)
    set_actuator_state(actuator, False)
    wait_after_actuator_change(actuator, machine_obj, settle_ms, logger)
    sleep_ms(settle_ms)

    dwell = int(post_trigger_dwell_ms)
    sleep_ms(dwell)

    move_direct_speed(nozzle, mm_loc(x, y, lifted_z, rotation), dispense_speed)


def move_to_origin_safe_z(nozzle, cfg, logger=None):
    origin = cfg.get("origin_location", {})
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    rotation = float(origin.get("rotation", current.getRotation()))
    target = mm_loc(origin.get("x", 0.0), origin.get("y", 0.0),
        cfg.get("safe_z_mm", 25.0), rotation)
    log_msg(logger, "Move to origin safe Z: %s" % loc_to_log(target))
    move_configured_safe(nozzle, target, cfg, logger, "origin safe move")
    return target


def run_calibration(cfg, machine_obj=None, logger=None):
    checked = validate_config(cfg)
    cfg = checked["cfg"]
    cfg["_config_path"] = cfg.get("_config_path", DEFAULT_CONFIG_PATH)
    machine_obj = get_machine(machine_obj)
    nozzle = find_nozzle(cfg, None, machine_obj)
    actuator = find_actuator(cfg, nozzle, machine_obj)

    origin = cfg.get("origin_location", {})
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    rotation = float(origin.get("rotation", current.getRotation()))
    dispense_z = checked["dispense_z"]
    line_cfg = cfg.get("line", {})
    matrix_cfg = cfg.get("matrix", {})
    line_count = checked["line_count"]
    matrix_count = checked["matrix_count"]

    log_msg(logger, "Starting adhesive calibration")
    log_msg(logger, "Nozzle: %s" % object_name(nozzle))
    log_msg(logger, "Actuator: %s" % object_name(actuator))
    log_msg(logger, "Dispense Z: %.4f" % dispense_z)

    completed_line = 0
    completed_matrix = 0

    try:
        set_actuator_state(actuator, False)
        wait_after_actuator_change(actuator, machine_obj,
            int(cfg.get("actuator_settle_ms", 50)), logger)

        move_to_origin_safe_z(nozzle, cfg, logger)

        if line_count > 0:
            pitch = float(line_cfg.get("pitch_mm", 0.0))
            lift = float(line_cfg.get("lift_mm", 0.0))
            post_dwell = int(line_cfg.get("post_trigger_dwell_ms", 0))
            for i in range(line_count):
                y = float(origin.get("y", 0.0))
                if i > 0:
                    y += pitch * i
                x = float(origin.get("x", 0.0))
                log_msg(logger, "Line dot %d/%d at X %.4f Y %.4f" %
                    (i + 1, line_count, x, y))
                dispense_one_dot(nozzle, actuator, machine_obj, x, y,
                    dispense_z, lift, rotation, cfg, post_dwell, logger)
                completed_line += 1

        if matrix_count > 0:
            rows = int(matrix_cfg.get("rows", 0))
            cols = int(matrix_cfg.get("columns", 0))
            pitch_x = float(matrix_cfg.get("pitch_x_mm", 0.0))
            pitch_y = float(matrix_cfg.get("pitch_y_mm", 0.0))
            lift = float(matrix_cfg.get("lift_mm", 0.0))
            post_dwell = int(matrix_cfg.get("post_trigger_dwell_ms", 0))
            start_x = float(origin.get("x", 0.0)) + float(matrix_cfg.get("offset_x_mm", 0.0))
            start_y = float(origin.get("y", 0.0))
            dot_index = 0
            for row in range(rows):
                for col in range(cols):
                    dot_index += 1
                    x = start_x + pitch_x * col
                    y = start_y + pitch_y * row
                    log_msg(logger, "Matrix dot %d/%d at X %.4f Y %.4f" %
                        (dot_index, matrix_count, x, y))
                    dispense_one_dot(nozzle, actuator, machine_obj, x, y,
                        dispense_z, lift, rotation, cfg, post_dwell, logger)
                    completed_matrix += 1

        park = cfg.get("park_location", {})
        if bool_value(park.get("enabled", False)):
            park_loc = dict_to_loc(park)
            log_msg(logger, "Move to park location: %s" % loc_to_log(park_loc))
            move_configured_safe(nozzle, park_loc, cfg, logger, "park safe move")
        else:
            target = mm_loc(origin.get("x", 0.0), origin.get("y", 0.0),
                cfg.get("safe_z_mm", 25.0), rotation)
            log_msg(logger, "Move back to origin safe Z: %s" % loc_to_log(target))
            move_configured_safe(nozzle, target, cfg, logger, "return safe move")

        return {
            "ok": True,
            "message": "Adhesive calibration finished",
            "line_dots": completed_line,
            "matrix_dots": completed_matrix,
            "total_dots": completed_line + completed_matrix,
            "nozzle_name": object_name(nozzle),
            "actuator_name": object_name(actuator),
            "config_path": cfg.get("_config_path", DEFAULT_CONFIG_PATH)
        }
    finally:
        try:
            set_actuator_state(actuator, False)
            wait_after_actuator_change(actuator, machine_obj,
                int(cfg.get("actuator_settle_ms", 50)), logger)
            log_msg(logger, "Actuator forced off")
        except Exception, e:
            log_msg(logger, "Unable to force actuator off: %s" % e)
