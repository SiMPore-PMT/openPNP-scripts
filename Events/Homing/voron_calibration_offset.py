# voron_calibration_offset.py
#
# Measures Voron Tap Z trigger offset against a contact actuator reference.
# Returns a single primary value: voron_contact_z_mm - contact_reference_z_mm.

import imp
import json
import math
import os
import tempfile

from java.lang import Throwable


try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except:
    SCRIPT_DIR = os.getcwd()
SCRIPTS_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
DEFAULT_CONFIG_PATH = os.path.join(SCRIPTS_ROOT, "voron_calibration_offset_config.json")
NEEDLE_BACKEND_PATH = os.path.join(SCRIPT_DIR, "needle_calibration2.py")
CONTACT_REFERENCE_START_ABOVE_MM = 1.0

needle = imp.load_source("needle_calibration2_for_voron_offset", NEEDLE_BACKEND_PATH)


class VoronCalibrationOffsetError(Exception):
    pass


DEFAULT_CONFIG = {
    "schema_version": 2,
    "head_name": "",
    "nozzle_name": "Dispense Head",
    "top_camera_name": "Top",
    "contact_actuator_name": "NeedleContactProbe_READ",
    "voron_actuator_name": "Voron Z Probe",
    "safe_z_mm": 25.0,
    "travel_speed": 1.0,
    "probe_speed": 0.8,
    "fine_probe_speed": 0.8,
    "z_touch": {
        "location": {"x": 284.8419, "y": 396.9272, "z": 5.0, "rotation": 0.0},
        "result_stage": "results",
        "pipeline": {"mode": "named", "name": "CAL_1pt_grid", "path": ""}
    },
    "contact_probe": {
        "acquisition_method": "polling",
        "mode": "direct_gcode",
        "sequence_actuator_name": "Run Contact Probe",
        "channel": "Q1",
        "probe_acceleration": 1000.0,
        "coarse_feed": 3000.0,
        "fine_feed": 500.0,
        "direct_fine_repeats": 1,
        "g38_attempts": 2,
        "start_above_mm": 1.0,
        "max_descent_mm": 4.0,
        "coarse_step_mm": 0.10,
        "fine_step_mm": 0.01,
        "retract_after_contact_mm": 0.10,
        "release_retract_step_mm": 0.20,
        "max_release_retract_mm": 2.0,
        "read_settle_ms": 80,
        "debounce_count": 3,
        "debounce_retries": 3,
        "debounce_interval_ms": 20,
        "coarse_repeats": 1,
        "fine_repeats": 1,
        "coarse_speed": 0.8,
        "fine_speed": 0.8
    },
    "z_probe": {
        "acquisition_method": "hardware_interrupt",
        "mode": "actuator",
        "sequence_actuator_name": "Run Voron Probe",
        "channel": "Q0",
        "probe_acceleration": 1000.0,
        "coarse_feed": 3000.0,
        "fine_feed": 500.0,
        "fine_repeats": 1,
        "g38_attempts": 2,
        "polling_coarse_repeats": 1,
        "polling_fine_repeats": 1,
        "polling_coarse_speed": 0.8,
        "polling_fine_speed": 0.8,
        "start_above_mm": 2.0,
        "max_descent_mm": 4.0,
        "coarse_step_mm": 0.10,
        "fine_step_mm": 0.01,
        "retract_after_contact_mm": 0.10,
        "release_retract_step_mm": 0.20,
        "max_release_retract_mm": 2.0,
        "read_settle_ms": 80,
        "debounce_count": 3,
        "debounce_retries": 3,
        "debounce_interval_ms": 20
    },
    "debug": {
        "save_last_capture": True,
        "verbose": False
    }
}


def log_msg(logger, msg):
    needle.log_msg(logger, msg)


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
    loaded_zp = loaded.get("z_probe", {}) if isinstance(loaded, dict) else {}
    loaded_contact = loaded.get("contact_probe", {}) if isinstance(loaded, dict) else {}
    if "acquisition_method" not in loaded_zp:
        # Schema 1 always used the interrupt path for the Voron measurement.
        cfg["z_probe"]["acquisition_method"] = "hardware_interrupt"
    if "channel" not in loaded_zp:
        cfg["z_probe"]["channel"] = "Q0"
    if "probe_acceleration" not in loaded_zp:
        cfg["z_probe"]["probe_acceleration"] = 1000.0
    if "polling_coarse_repeats" not in loaded_zp:
        cfg["z_probe"]["polling_coarse_repeats"] = loaded_zp.get(
            "coarse_count", 1)
    if "polling_fine_repeats" not in loaded_zp:
        cfg["z_probe"]["polling_fine_repeats"] = loaded_zp.get(
            "fine_count", loaded_zp.get("repeat_count", 1))
    if not loaded_contact:
        contact = cfg["contact_probe"]
        for key in ["max_descent_mm", "coarse_step_mm", "fine_step_mm",
                "retract_after_contact_mm", "release_retract_step_mm",
                "max_release_retract_mm", "read_settle_ms", "debounce_count",
                "debounce_retries", "debounce_interval_ms"]:
            if key in loaded_zp:
                contact[key] = loaded_zp.get(key)
        contact["coarse_repeats"] = loaded_zp.get("coarse_count", 1)
        contact["fine_repeats"] = loaded_zp.get("fine_count",
            loaded_zp.get("repeat_count", 1))
        contact["coarse_speed"] = cfg.get("probe_speed", 0.8)
        contact["fine_speed"] = cfg.get("fine_probe_speed", 0.8)
    if "acquisition_method" not in loaded_contact:
        cfg["contact_probe"]["acquisition_method"] = "polling"
    if "channel" not in loaded_contact:
        cfg["contact_probe"]["channel"] = "Q1"
    cfg["schema_version"] = 2
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


def update_voron_offset_configs(voron_offset_mm, scripts_root=None):
    value = float(voron_offset_mm)
    if value != value or math.isinf(value):
        raise VoronCalibrationOffsetError("Voron offset must be a finite number")
    root = scripts_root or SCRIPTS_ROOT
    targets = [
        (os.path.join(root, "needle_calibration_config.json"),
         ["fiducials", "z_touch", "trigger_offset_mm"]),
        (os.path.join(root, "bed_mesh_config.json"),
         ["voron_trigger_offset_mm"])
    ]
    prepared = []
    for path, keys in targets:
        if not os.path.exists(path):
            raise VoronCalibrationOffsetError("Target config not found: %s" % path)
        fh = open(path, "r")
        try:
            original = fh.read()
        finally:
            fh.close()
        try:
            data = json.loads(original)
        except Exception as error:
            raise VoronCalibrationOffsetError(
                "Target config is not valid JSON: %s: %s" % (path, error))
        node = data
        for key in keys[:-1]:
            if key not in node or not isinstance(node.get(key), dict):
                raise VoronCalibrationOffsetError(
                    "Target config is missing %s in %s" % (".".join(keys), path))
            node = node[key]
        leaf = keys[-1]
        old_value = node.get(leaf, None)
        node[leaf] = value
        prepared.append({"path": path, "keys": keys, "old_value": old_value,
            "new_value": value, "data": data, "temp_path": None})

    try:
        for item in prepared:
            fd, temp_path = tempfile.mkstemp(prefix=".voron-offset-",
                suffix=".json", dir=os.path.dirname(item["path"]))
            item["temp_path"] = temp_path
            fh = os.fdopen(fd, "w")
            try:
                fh.write(json.dumps(item["data"], sort_keys=True, indent=2))
                fh.write("\n")
                fh.flush()
                os.fsync(fh.fileno())
            finally:
                fh.close()
        for item in prepared:
            os.rename(item["temp_path"], item["path"])
            item["temp_path"] = None
    finally:
        for item in prepared:
            temp_path = item.get("temp_path")
            if temp_path and os.path.exists(temp_path):
                try:
                    os.unlink(temp_path)
                except:
                    pass
    return [{"path": item["path"], "field": ".".join(item["keys"]),
        "old_value": item["old_value"], "new_value": item["new_value"]}
        for item in prepared]


def build_probe_cfg(cfg):
    out = {
        "schema_version": 1,
        "head_name": cfg.get("head_name", ""),
        "nozzle_name": cfg.get("nozzle_name", ""),
        "top_camera_name": cfg.get("top_camera_name", ""),
        "contact_actuator_name": cfg.get("contact_actuator_name", ""),
        "safe_z_mm": cfg.get("safe_z_mm", 25.0),
        "travel_speed": cfg.get("travel_speed", 1.0),
        "probe_speed": cfg.get("probe_speed", 0.8),
        "fine_probe_speed": cfg.get("fine_probe_speed", 0.05),
        "z_probe": deep_copy(cfg.get("z_probe", {})),
        "debug": deep_copy(cfg.get("debug", {}))
    }
    out["fiducials"] = {"z_touch": deep_copy(cfg.get("z_touch", {}))}
    return out


def build_contact_probe_cfg(cfg):
    out = build_probe_cfg(cfg)
    contact = deep_copy(cfg.get("contact_probe", {}))
    contact["polling_coarse_repeats"] = contact.get("coarse_repeats", 1)
    contact["polling_fine_repeats"] = contact.get("fine_repeats", 1)
    contact["fine_repeats"] = contact.get("direct_fine_repeats", 1)
    out["z_probe"] = contact
    out["probe_speed"] = contact.get("coarse_speed", cfg.get("probe_speed", 0.8))
    out["fine_probe_speed"] = contact.get("fine_speed",
        cfg.get("fine_probe_speed", 0.8))
    return out


def build_voron_polling_cfg(cfg):
    out = build_probe_cfg(cfg)
    zp = out.get("z_probe", {})
    out["probe_speed"] = zp.get("polling_coarse_speed", 0.8)
    out["fine_probe_speed"] = zp.get("polling_fine_speed", 0.8)
    return out


def find_actuator_by_config_name(machine_obj, cfg, key, label):
    name = str(cfg.get(key, "") or "").strip()
    if name == "":
        raise VoronCalibrationOffsetError("%s actuator is required" % label)
    owners = [machine_obj]
    try:
        owners.insert(0, needle.find_nozzle(build_probe_cfg(cfg), None, machine_obj).getHead())
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
            for actuator in needle.java_list_to_py(owner.getActuators()):
                if needle.object_name(actuator) == name:
                    return actuator
        except:
            pass
    raise VoronCalibrationOffsetError("%s actuator not found: %s" % (label, name))


def capture_touch_center(cfg, nozzle=None, machine_obj=None, logger=None):
    probe_cfg = build_probe_cfg(cfg)
    return needle.capture_with_pipeline(
        probe_cfg, "z_touch", probe_cfg.get("fiducials", {}).get("z_touch", {}),
        nozzle, machine_obj, logger)


def mean(values):
    if len(values) == 0:
        raise VoronCalibrationOffsetError("Cannot average empty probe list")
    total = 0.0
    for value in values:
        total += float(value)
    return total / float(len(values))


def max_abs_delta(values, center):
    delta = 0.0
    for value in values:
        delta = max(delta, abs(float(value) - float(center)))
    return delta


def count_from_settings(settings, key, label):
    try:
        value = float(settings.get(key, 1))
        count = int(value)
    except:
        raise VoronCalibrationOffsetError("%s.%s must be a whole number" %
            (label, key))
    if value != float(count):
        raise VoronCalibrationOffsetError("%s.%s must be a whole number" %
            (label, key))
    if count < 1:
        raise VoronCalibrationOffsetError("%s.%s must be >= 1" % (label, key))
    return count


def probe_z_touch_counted(nozzle, actuator, cfg, x_mm, y_mm, surface_z_mm,
        label, logger=None):
    probe_cfg = build_contact_probe_cfg(cfg)
    zp = deep_copy(probe_cfg.get("z_probe", {}))
    coarse_count = count_from_settings(zp, "polling_coarse_repeats",
        "contact_probe")
    fine_count = count_from_settings(zp, "polling_fine_repeats",
        "contact_probe")
    # The conductive reference still uses contact-sensor polling, but it does
    # not need to poll all the way down from safe Z. Travel over the pad at safe
    # Z first, then descend directly to 1 mm above the expected surface before
    # taking the first sensor reading.
    start_above = float(zp.get("start_above_mm", CONTACT_REFERENCE_START_ABOVE_MM))
    start_z = float(surface_z_mm) + start_above
    limit_z = start_z - float(zp.get("max_descent_mm", 8.0))
    coarse_step = abs(float(zp.get("coarse_step_mm", 0.10)))
    fine_step = abs(float(zp.get("fine_step_mm", 0.02)))
    speed = float(zp.get("coarse_speed", cfg.get("probe_speed", 0.20)))
    fine_speed = float(zp.get("fine_speed", cfg.get("fine_probe_speed", 0.05)))
    trigger_offset = float(zp.get("trigger_offset_mm", 0.0))
    retract_after = abs(float(zp.get("retract_after_contact_mm", 0.40)))

    start = needle.mm_loc(x_mm, y_mm, start_z,
        needle.get_movable_location(nozzle).getRotation())
    log_msg(logger,
        "%s safe XY travel, then direct Z approach to expected surface + %.3f mm: %s" %
        (label, start_above, needle.loc_to_log(start)))
    needle.lift_nozzle_to_safe_z(nozzle, probe_cfg, logger)
    safe_location = needle.get_movable_location(nozzle)
    safe_xy = needle.mm_loc(x_mm, y_mm, safe_location.getZ(),
        start.getRotation())
    needle.move_direct_speed(nozzle, safe_xy,
        float(cfg.get("travel_speed", 1.0)))
    needle.move_direct_speed(nozzle, start,
        float(cfg.get("travel_speed", 1.0)))
    needle.dwell_for_read(zp)
    if needle.read_contact_debounced(actuator, zp, logger=logger,
            label=label + "_precheck"):
        raise VoronCalibrationOffsetError("%s probe already active above pad" % label)

    coarse_records = []
    fine_records = []
    for index in range(coarse_count):
        current_z = float(needle.get_movable_location(nozzle).getZ())
        contact, steps = needle.descend_until_contact(nozzle, actuator, zp,
            x_mm, y_mm, current_z, limit_z, coarse_step, speed, logger,
            "%s_coarse_%d" % (label, index + 1))
        if contact is None:
            raise VoronCalibrationOffsetError(
                "%s no coarse Z contact within max descent on repeat %d/%d" %
                (label, index + 1, coarse_count))
        restart_z = float(contact.getZ()) + retract_after
        needle.move_direct_speed(nozzle, needle.mm_loc(x_mm, y_mm, restart_z,
            contact.getRotation()), speed)
        extra = needle.retract_until_released(nozzle, actuator, zp, x_mm, y_mm,
            float(zp.get("release_retract_step_mm", 0.20)),
            float(zp.get("max_release_retract_mm", 2.0)), speed, logger,
            "%s_coarse_release_%d" % (label, index + 1))
        coarse_records.append({
            "repeat_index": index + 1,
            "contact": needle.loc_to_dict(contact),
            "measured_contact_z_mm": float(contact.getZ()),
            "steps": int(steps),
            "release_extra_mm": float(extra)
        })

    fine_values = []
    for index in range(fine_count):
        current_z = float(needle.get_movable_location(nozzle).getZ())
        contact, steps = needle.descend_until_contact(nozzle, actuator, zp,
            x_mm, y_mm, current_z, limit_z, fine_step, fine_speed, logger,
            "%s_fine_%d" % (label, index + 1))
        if contact is None:
            raise VoronCalibrationOffsetError(
                "%s no fine Z contact within max descent on repeat %d/%d" %
                (label, index + 1, fine_count))
        restart_z = float(contact.getZ()) + retract_after
        needle.move_direct_speed(nozzle, needle.mm_loc(x_mm, y_mm, restart_z,
            contact.getRotation()), speed)
        extra = needle.retract_until_released(nozzle, actuator, zp, x_mm, y_mm,
            float(zp.get("release_retract_step_mm", 0.20)),
            float(zp.get("max_release_retract_mm", 2.0)), speed, logger,
            "%s_fine_release_%d" % (label, index + 1))
        corrected_z = float(contact.getZ()) - trigger_offset
        fine_values.append(corrected_z)
        fine_records.append({
            "repeat_index": index + 1,
            "contact": needle.loc_to_dict(contact),
            "measured_contact_z_mm": float(contact.getZ()),
            "corrected_contact_z_mm": corrected_z,
            "steps": int(steps),
            "release_extra_mm": float(extra)
        })

    avg = mean(fine_values)
    return {
        "label": label,
        "actuator_name": needle.object_name(actuator),
        "surface_z_mm": float(surface_z_mm),
        "coarse_count": coarse_count,
        "fine_count": fine_count,
        "coarse_contacts": coarse_records,
        "fine_contacts": fine_records,
        "fine_corrected_contact_z_values_mm": fine_values,
        "corrected_contact_z_mm": avg,
        "measured_contact_z_mm": avg + trigger_offset,
        "trigger_offset_mm": trigger_offset,
        "z_error_mm": float(surface_z_mm) - avg,
        "max_abs_delta_from_average_mm": max_abs_delta(fine_values, avg),
        "contact": fine_records[-1].get("contact")
    }


def probe_contact_reference(nozzle, actuator, cfg, x_mm, y_mm, surface_z_mm,
        label, logger=None):
    profile = cfg.get("contact_probe", {})
    acquisition = str(profile.get("acquisition_method", "polling") or
        "polling").strip().lower()
    start_z = float(surface_z_mm) + float(profile.get("start_above_mm", 1.0))
    start = needle.mm_loc(x_mm, y_mm, start_z,
        needle.get_movable_location(nozzle).getRotation())
    log_msg(logger, "%s move above pad %s" % (label, needle.loc_to_log(start)))
    needle.move_safe(nozzle, start)
    probe_cfg = build_contact_probe_cfg(cfg)
    if acquisition == "polling":
        sequence = needle.run_polling_z_probe_sequence(nozzle, actuator,
            probe_cfg, x_mm, y_mm, start_z, logger, label)
    elif acquisition == "hardware_interrupt":
        sequence = needle.run_interrupt_probe_sequence(nozzle, actuator,
            probe_cfg, logger, label)
        sequence["acquisition_method"] = "hardware_interrupt"
    else:
        raise VoronCalibrationOffsetError(
            "contact_probe.acquisition_method must be polling or hardware_interrupt")
    raw_contact_z = float(sequence.get("averaged_contact_z_mm"))
    trigger_offset = float(profile.get("trigger_offset_mm", 0.0))
    contact_z = raw_contact_z - trigger_offset
    coarse_values = [float(value) for value in
        sequence.get("coarse_contact_z_values_mm", [])]
    if len(coarse_values) == 0:
        coarse_values = [float(sequence.get("coarse_contact_z_mm"))]
    fine_values = [float(value) for value in
        sequence.get("fine_contact_z_values_mm", [])]
    corrected_fine_values = [value - trigger_offset for value in fine_values]
    return {
        "label": label,
        "acquisition_method": acquisition,
        "actuator_name": needle.object_name(actuator),
        "surface_z_mm": float(surface_z_mm),
        "coarse_count": len(coarse_values),
        "fine_count": len(fine_values),
        "coarse_contacts": [{"repeat_index": index + 1,
            "measured_contact_z_mm": value}
            for index, value in enumerate(coarse_values)],
        "fine_contacts": [{"repeat_index": index + 1,
            "measured_contact_z_mm": value,
            "corrected_contact_z_mm": value - trigger_offset}
            for index, value in enumerate(fine_values)],
        "fine_corrected_contact_z_values_mm": corrected_fine_values,
        "corrected_contact_z_mm": contact_z,
        "measured_contact_z_mm": raw_contact_z,
        "trigger_offset_mm": trigger_offset,
        "z_error_mm": float(surface_z_mm) - contact_z,
        "max_abs_delta_from_average_mm": max_abs_delta(
            corrected_fine_values, contact_z),
        "contact": {"z": raw_contact_z},
        "probe_sequence": sequence,
        "interrupt_sequence": sequence if acquisition == "hardware_interrupt" else None
    }


def probe_voron(nozzle, actuator, cfg, x_mm, y_mm, surface_z_mm,
        label, logger=None):
    zp = cfg.get("z_probe", {})
    acquisition = str(zp.get("acquisition_method", "hardware_interrupt") or
        "hardware_interrupt").strip().lower()
    start_z = float(surface_z_mm) + float(zp.get("start_above_mm", 2.0))
    start = needle.mm_loc(x_mm, y_mm, start_z,
        needle.get_movable_location(nozzle).getRotation())
    log_msg(logger, "%s move above pad %s" % (label, needle.loc_to_log(start)))
    needle.move_safe(nozzle, start)
    if acquisition == "polling":
        sequence = needle.run_polling_z_probe_sequence(nozzle, actuator,
            build_voron_polling_cfg(cfg), x_mm, y_mm, start_z, logger, label)
    elif acquisition == "hardware_interrupt":
        sequence = needle.run_interrupt_probe_sequence(
            nozzle, actuator, build_probe_cfg(cfg), logger, label)
        sequence["acquisition_method"] = "hardware_interrupt"
    else:
        raise VoronCalibrationOffsetError(
            "z_probe.acquisition_method must be polling or hardware_interrupt")
    contact_z = float(sequence.get("averaged_contact_z_mm"))
    coarse_values = [float(value) for value in
        sequence.get("coarse_contact_z_values_mm", [])]
    if len(coarse_values) == 0:
        coarse_values = [float(sequence.get("coarse_contact_z_mm"))]
    fine_values = [float(value) for value in
        sequence.get("fine_contact_z_values_mm", [])]
    return {
        "label": label,
        "acquisition_method": acquisition,
        "actuator_name": needle.object_name(actuator),
        "surface_z_mm": float(surface_z_mm),
        "coarse_count": len(coarse_values),
        "fine_count": len(fine_values),
        "coarse_contacts": [{"repeat_index": index + 1,
            "measured_contact_z_mm": value}
            for index, value in enumerate(coarse_values)],
        "fine_contacts": [{
            "repeat_index": index + 1,
            "measured_contact_z_mm": value,
            "corrected_contact_z_mm": value
        } for index, value in enumerate(fine_values)],
        "fine_corrected_contact_z_values_mm": fine_values,
        "corrected_contact_z_mm": contact_z,
        "measured_contact_z_mm": contact_z,
        "trigger_offset_mm": 0.0,
        "z_error_mm": float(surface_z_mm) - contact_z,
        "max_abs_delta_from_average_mm": max_abs_delta(fine_values, contact_z),
        "contact": {"z": contact_z},
        "probe_sequence": sequence,
        "interrupt_sequence": sequence if acquisition == "hardware_interrupt" else None
    }


def fail_payload(message, nozzle=None, cfg=None, logger=None):
    log_msg(logger, "Voron Tap offset calibration failed: %s" % message)
    if nozzle is not None and cfg is not None:
        try:
            needle.lift_nozzle_to_safe_z(nozzle, build_probe_cfg(cfg), logger)
        except:
            pass
    return {"ok": False, "stage": "voron_offset", "message": str(message),
        "voron_offset_mm": None, "contact_reference_z_mm": None,
        "voron_contact_z_mm": None, "details": {}}


def run_measure(cfg, nozzle=None, machine_obj=None, logger=None):
    contact_probe = None
    voron_probe = None
    try:
        machine_obj = needle.get_machine(machine_obj)
        probe_cfg = build_probe_cfg(cfg)
        nozzle = needle.find_nozzle(probe_cfg, nozzle, machine_obj)
        contact_actuator = find_actuator_by_config_name(machine_obj, cfg,
            "contact_actuator_name", "Contact")
        voron_actuator = find_actuator_by_config_name(machine_obj, cfg,
            "voron_actuator_name", "Voron")

        needle.log_section(logger, "VORON TAP OFFSET MEASURE START")
        needle.log_kv(logger, "nozzle", needle.object_name(nozzle))
        needle.log_kv(logger, "contact actuator", needle.object_name(contact_actuator))
        needle.log_kv(logger, "voron actuator", needle.object_name(voron_actuator))
        contact_cfg = cfg.get("contact_probe", {})
        contact_acquisition = str(contact_cfg.get("acquisition_method", "polling")
            or "polling").strip().lower()
        needle.log_kv(logger, "contact acquisition", contact_acquisition)
        if contact_acquisition == "polling":
            needle.log_kv(logger, "contact polling repeats", "coarse=%d fine=%d" %
                (count_from_settings(contact_cfg, "coarse_repeats", "contact_probe"),
                 count_from_settings(contact_cfg, "fine_repeats", "contact_probe")))
        else:
            needle.log_kv(logger, "contact hardware",
                "mode=%s channel=%s attempts=%s" %
                (contact_cfg.get("mode", "direct_gcode"),
                 contact_cfg.get("channel", "Q1"),
                 int(float(contact_cfg.get("g38_attempts", 2)))))
        acquisition = str(cfg.get("z_probe", {}).get("acquisition_method",
            "hardware_interrupt") or "hardware_interrupt").strip().lower()
        needle.log_kv(logger, "voron acquisition", acquisition)
        if acquisition == "hardware_interrupt":
            z_profile = cfg.get("z_probe", {})
            needle.log_kv(logger, "voron hardware",
                "mode=%s channel=%s attempts=%s" %
                (z_profile.get("mode", "actuator"),
                 z_profile.get("channel", "Q0"),
                 int(float(z_profile.get("g38_attempts", 2)))))

        capture = capture_touch_center(cfg, nozzle, machine_obj, logger)
        center = capture.get("center")
        surface_z = float(needle.dict_to_loc(cfg.get("z_touch", {}).get("location", {})).getZ())
        contact_probe = probe_contact_reference(nozzle, contact_actuator, cfg,
            float(center["x"]), float(center["y"]), surface_z,
            "contact_reference", logger)
        voron_probe = probe_voron(nozzle, voron_actuator, cfg,
            float(center["x"]), float(center["y"]), surface_z,
            "voron_tap", logger)

        contact_z = float(contact_probe.get("corrected_contact_z_mm"))
        voron_z = float(voron_probe.get("corrected_contact_z_mm"))
        offset = voron_z - contact_z

        try:
            needle.lift_nozzle_to_safe_z(nozzle, probe_cfg, logger)
        except:
            pass

        log_msg(logger, "VORON OFFSET contact=%.6f voron=%.6f offset=%.6f" %
            (contact_z, voron_z, offset))
        return {"ok": True, "stage": "voron_offset", "message": "ok",
            "voron_offset_mm": offset,
            "contact_reference_z_mm": contact_z,
            "voron_contact_z_mm": voron_z,
            "details": {
                "center": center,
                "capture": capture,
                "contact_probe": contact_probe,
                "voron_probe": voron_probe,
                "offset_formula": "voron_contact_z_mm - contact_reference_z_mm"
            }}
    except Throwable as e:
        return fail_payload(e, nozzle, cfg, logger)
    except Exception as e:
        return fail_payload(e, nozzle, cfg, logger)


def list_available_vision_pipeline_names(machine_obj=None):
    return needle.list_available_vision_pipeline_names(machine_obj)


def __getattr__(name):
    return getattr(needle, name)


if __name__ == "__main__":
    cfg = load_config()
    log_msg(None, "Loaded Voron Tap offset config: %s" % cfg.get("_config_path"))
