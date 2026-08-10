# bed_mesh.py
#
# Standalone Voron Tap bed mesh backend for OpenPnP Jython 2.7.
# This file is safe to import outside OpenPnP for testing its pure helpers;
# OpenPnP/Java dependencies are loaded only when machine motion is requested.

from __future__ import print_function

import datetime
import json
import math
import os

try:
    import imp
except ImportError:
    imp = None

try:
    from java.lang import Throwable as JavaThrowable
except ImportError:
    JavaThrowable = BaseException


try:
    basestring
except NameError:
    basestring = str


try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except:
    SCRIPT_DIR = os.getcwd()

SCRIPTS_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
DEFAULT_CONFIG_PATH = os.path.join(SCRIPTS_ROOT, "bed_mesh_config.json")
DEFAULT_OUTPUT_DIR = os.path.join(SCRIPTS_ROOT, "bed_mesh_results")
NEEDLE_BACKEND_PATH = os.path.join(SCRIPT_DIR, "needle_calibration2.py")


class BedMeshError(Exception):
    pass


def _default_panel(index):
    return {
        "enabled": index == 1,
        "name": "Panel %d" % index,
        "lower_left_x_mm": 0.0,
        "lower_left_y_mm": 0.0,
        "expected_z_mm": 0.0,
        "width_mm": 100.0,
        "height_mm": 100.0,
        "columns": 3,
        "rows": 3,
    }


DEFAULT_CONFIG = {
    "schema_version": 2,
    "head_name": "",
    "nozzle_name": "Dispense Head",
    "probe_actuator_name": "Voron Z Probe",
    "voron_trigger_offset_mm": 0.0,
    "safe_z_mm": 25.0,
    "travel_speed": 1.0,
    "probe": {
        "acquisition_method": "hardware_interrupt",
        "mode": "actuator",
        "sequence_actuator_name": "Run Voron Probe",
        "channel": "Q0",
        "probe_acceleration": 1000.0,
        "g38_attempts": 2,
        "coarse_feed": 3000.0,
        "fine_feed": 500.0,
        "start_above_mm": 2.0,
        "max_descent_mm": 4.0,
        "coarse_step_mm": 0.10,
        "coarse_speed": 0.8,
        "polling_coarse_repeats": 1,
        "fine_step_mm": 0.01,
        "fine_speed": 0.8,
        "fine_repeats": 3,
        "polling_fine_repeats": 3,
        "retract_after_contact_mm": 0.10,
        "release_retract_step_mm": 0.20,
        "max_release_retract_mm": 2.0,
        "read_settle_ms": 40,
        "debounce_enabled": True,
        "debounce_count": 10,
        "debounce_retries": 4,
        "debounce_interval_ms": 20,
    },
    "output_directory": DEFAULT_OUTPUT_DIR,
    "panels": [_default_panel(i) for i in range(1, 5)],
}


_needle_backend = None


def get_needle_backend():
    global _needle_backend
    if _needle_backend is None:
        if imp is not None:
            _needle_backend = imp.load_source(
                "needle_calibration2_for_bed_mesh", NEEDLE_BACKEND_PATH)
        else:
            import importlib.util
            spec = importlib.util.spec_from_file_location(
                "needle_calibration2_for_bed_mesh", NEEDLE_BACKEND_PATH)
            _needle_backend = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(_needle_backend)
    return _needle_backend


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
            out[key] = deep_copy(value)
    return out


def normalize_config(loaded):
    cfg = deep_merge(DEFAULT_CONFIG, loaded)
    loaded_probe = loaded.get("probe", {}) if isinstance(loaded, dict) else {}
    try:
        loaded_schema = int(float(loaded.get("schema_version", 1)))
    except:
        loaded_schema = 1
    if loaded_schema < 2:
        # Schema 1 always executed the interrupt path, even though it retained
        # dormant polling values. Preserve that behavior during migration.
        cfg["probe"]["acquisition_method"] = "hardware_interrupt"
        cfg["probe"]["channel"] = str(
            loaded_probe.get("channel", "Q0") or "Q0").upper()
        cfg["probe"]["probe_acceleration"] = loaded_probe.get(
            "probe_acceleration", 1000.0)
        cfg["probe"]["g38_attempts"] = loaded_probe.get("g38_attempts", 2)
        cfg["probe"]["polling_coarse_repeats"] = loaded_probe.get(
            "polling_coarse_repeats", loaded_probe.get("coarse_repeats", 1))
        cfg["probe"]["polling_fine_repeats"] = loaded_probe.get(
            "polling_fine_repeats", loaded_probe.get("fine_repeats", 3))
    # coarse_repeats was the dormant schema-1 polling field. Its value is now
    # represented unambiguously by polling_coarse_repeats.
    cfg["probe"].pop("coarse_repeats", None)
    loaded_panels = loaded.get("panels", []) if isinstance(loaded, dict) else []
    panels = []
    for index in range(4):
        source = loaded_panels[index] if index < len(loaded_panels) else {}
        panels.append(deep_merge(_default_panel(index + 1), source))
    cfg["panels"] = panels
    cfg["schema_version"] = 2
    return cfg


def load_config(path=None):
    cfg_path = path or DEFAULT_CONFIG_PATH
    if not os.path.exists(cfg_path):
        cfg = deep_copy(DEFAULT_CONFIG)
    else:
        fh = open(cfg_path, "r")
        try:
            cfg = normalize_config(json.loads(fh.read()))
        finally:
            fh.close()
    cfg["_config_path"] = cfg_path
    return cfg


def public_config(cfg):
    clean = normalize_config(cfg)
    if "_config_path" in clean:
        del clean["_config_path"]
    return clean


def save_config(cfg, path=None):
    cfg_path = path or cfg.get("_config_path", DEFAULT_CONFIG_PATH)
    fh = open(cfg_path, "w")
    try:
        fh.write(json.dumps(public_config(cfg), sort_keys=True, indent=2))
        fh.write("\n")
    finally:
        fh.close()
    return cfg_path


def _whole_number(value, label, minimum=1):
    try:
        number = float(value)
        integer = int(number)
    except:
        raise BedMeshError("%s must be a whole number" % label)
    if number != float(integer) or integer < minimum:
        raise BedMeshError("%s must be a whole number >= %d" % (label, minimum))
    return integer


def _positive(value, label):
    try:
        number = float(value)
    except:
        raise BedMeshError("%s must be a number" % label)
    if number != number or abs(number) == float("inf"):
        raise BedMeshError("%s must be finite" % label)
    if number <= 0.0:
        raise BedMeshError("%s must be > 0" % label)
    return number


def validate_config(cfg, require_machine_names=True):
    if require_machine_names:
        if not str(cfg.get("nozzle_name", "") or "").strip():
            raise BedMeshError("Nozzle is required")
        if not str(cfg.get("probe_actuator_name", "") or "").strip():
            raise BedMeshError("Probe actuator is required")

    safe_z = float(cfg.get("safe_z_mm", 0.0))
    _positive(cfg.get("travel_speed", 0.0), "travel_speed")
    probe = cfg.get("probe", {})
    acquisition = str(probe.get("acquisition_method", "hardware_interrupt") or
        "hardware_interrupt").strip().lower()
    if acquisition not in ["polling", "hardware_interrupt"]:
        raise BedMeshError(
            "probe.acquisition_method must be polling or hardware_interrupt")
    mode = str(probe.get("mode", "actuator") or "actuator").strip().lower()
    start_above = _positive(probe.get("start_above_mm"), "probe.start_above_mm")
    if acquisition == "polling":
        _positive(probe.get("max_descent_mm"), "probe.max_descent_mm")
        for key in ["coarse_step_mm", "coarse_speed", "fine_step_mm", "fine_speed",
                "retract_after_contact_mm", "release_retract_step_mm",
                "max_release_retract_mm"]:
            _positive(probe.get(key), "probe.%s" % key)
        for key in ["polling_coarse_repeats", "polling_fine_repeats"]:
            _whole_number(probe.get(key), "probe.%s" % key, 1)
        _whole_number(probe.get("read_settle_ms", 0),
            "probe.read_settle_ms", 0)
        if bool(probe.get("debounce_enabled", True)):
            for key in ["debounce_count", "debounce_retries"]:
                _whole_number(probe.get(key), "probe.%s" % key, 1)
            _whole_number(probe.get("debounce_interval_ms", 0),
                "probe.debounce_interval_ms", 0)
    else:
        if mode not in ["actuator", "direct_gcode"]:
            raise BedMeshError("probe.mode must be actuator or direct_gcode")
        if mode == "actuator" and not str(
                probe.get("sequence_actuator_name", "") or "").strip():
            raise BedMeshError(
                "probe.sequence_actuator_name is required in actuator mode")
        if mode == "direct_gcode":
            channel = str(probe.get("channel", "Q0") or "Q0").strip().upper()
            if channel not in ["Q0", "Q1"]:
                raise BedMeshError("probe.channel must be Q0 or Q1")
            for key in ["coarse_feed", "fine_feed", "probe_acceleration",
                    "max_descent_mm", "retract_after_contact_mm"]:
                _positive(probe.get(key), "probe.%s" % key)
            for key in ["fine_repeats", "g38_attempts"]:
                _whole_number(probe.get(key), "probe.%s" % key, 1)

    enabled = []
    panels = cfg.get("panels", [])
    if len(panels) != 4:
        raise BedMeshError("Exactly four panel configurations are required")
    for index, panel in enumerate(panels):
        if not bool(panel.get("enabled", False)):
            continue
        label = "panels[%d]" % (index + 1)
        enabled.append(panel)
        if not str(panel.get("name", "") or "").strip():
            raise BedMeshError("%s.name is required" % label)
        expected_z = float(panel.get("expected_z_mm"))
        float(panel.get("lower_left_x_mm"))
        float(panel.get("lower_left_y_mm"))
        _positive(panel.get("width_mm"), "%s.width_mm" % label)
        _positive(panel.get("height_mm"), "%s.height_mm" % label)
        _whole_number(panel.get("columns"), "%s.columns" % label, 2)
        _whole_number(panel.get("rows"), "%s.rows" % label, 2)
        if safe_z <= expected_z + start_above:
            raise BedMeshError(
                "safe_z_mm must be above %s expected Z + start-above" % label)
    if not enabled:
        raise BedMeshError("Enable at least one panel")
    return cfg


def generate_grid(panel, panel_index=1):
    columns = _whole_number(panel.get("columns"), "columns", 2)
    rows = _whole_number(panel.get("rows"), "rows", 2)
    x0 = float(panel.get("lower_left_x_mm"))
    y0 = float(panel.get("lower_left_y_mm"))
    width = _positive(panel.get("width_mm"), "width_mm")
    height = _positive(panel.get("height_mm"), "height_mm")
    points = []
    sequence = 0
    for row in range(rows):
        y = y0 + height * float(row) / float(rows - 1)
        column_order = range(columns) if row % 2 == 0 else range(columns - 1, -1, -1)
        for column in column_order:
            x = x0 + width * float(column) / float(columns - 1)
            sequence += 1
            points.append({
                "panel_index": int(panel_index),
                "panel_name": str(panel.get("name", "Panel %d" % panel_index)),
                "sequence": sequence,
                "row": row + 1,
                "column": column + 1,
                "x_mm": x,
                "y_mm": y,
                "expected_z_mm": float(panel.get("expected_z_mm")),
            })
    return points


def mean(values):
    if not values:
        raise BedMeshError("Cannot average an empty value list")
    return sum([float(value) for value in values]) / float(len(values))


def max_abs_delta(values, center):
    if not values:
        return 0.0
    return max([abs(float(value) - float(center)) for value in values])


def calculate_point_result(point, trigger_values, trigger_offset_mm):
    averaged_trigger = mean(trigger_values)
    corrected = averaged_trigger - float(trigger_offset_mm)
    expected = float(point.get("expected_z_mm"))
    result = deep_copy(point)
    result.update({
        "fine_trigger_z_values_mm": [float(value) for value in trigger_values],
        "averaged_trigger_z_mm": averaged_trigger,
        "corrected_surface_z_mm": corrected,
        "z_offset_mm": corrected - expected,
        "fine_repeat_count": len(trigger_values),
        "fine_max_abs_delta_mm": max_abs_delta(trigger_values, averaged_trigger),
        "fine_spread_mm": max(trigger_values) - min(trigger_values),
    })
    return result


def summarize_points(points):
    offsets = [float(point.get("z_offset_mm")) for point in points]
    if not offsets:
        return {"point_count": 0, "min_offset_mm": None, "max_offset_mm": None,
            "mean_offset_mm": None, "peak_to_peak_mm": None}
    minimum = min(offsets)
    maximum = max(offsets)
    return {"point_count": len(points), "min_offset_mm": minimum,
        "max_offset_mm": maximum, "mean_offset_mm": mean(offsets),
        "peak_to_peak_mm": maximum - minimum}


def find_probe_actuator(machine_obj, cfg, nozzle=None):
    needle = get_needle_backend()
    wanted = str(cfg.get("probe_actuator_name", "") or "").strip()
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
                found = getattr(owner, method)(wanted)
                if found is not None:
                    return found
            except:
                pass
        try:
            for actuator in needle.java_list_to_py(owner.getActuators()):
                if needle.object_name(actuator) == wanted:
                    return actuator
        except:
            pass
    raise BedMeshError("Probe actuator not found: %s" % wanted)


def _checkpoint(logger, message):
    if logger:
        logger(str(message))


def _retract_after_contact(nozzle, actuator, probe, x_mm, y_mm, speed,
        logger, label):
    needle = get_needle_backend()
    cur = needle.get_movable_location(nozzle)
    retract = abs(float(probe.get("retract_after_contact_mm")))
    needle.move_direct_speed(nozzle, needle.mm_loc(
        x_mm, y_mm, float(cur.getZ()) + retract, cur.getRotation()), speed)
    return needle.retract_until_released(
        nozzle, actuator, probe, x_mm, y_mm,
        float(probe.get("release_retract_step_mm")),
        float(probe.get("max_release_retract_mm")), speed, logger, label)


def probe_point(nozzle, actuator, cfg, point, logger=None):
    needle = get_needle_backend()
    probe = deep_copy(cfg.get("probe", {}))
    if not bool(probe.get("debounce_enabled", True)):
        # Leave read_settle_ms independent; disabling debounce removes the
        # repeated samples, retry passes, and inter-sample delay.
        probe["debounce_count"] = 1
        probe["debounce_retries"] = 1
        probe["debounce_interval_ms"] = 0
    x_mm = float(point.get("x_mm"))
    y_mm = float(point.get("y_mm"))
    expected_z = float(point.get("expected_z_mm"))
    start_z = expected_z + float(probe.get("start_above_mm"))
    rotation = needle.get_movable_location(nozzle).getRotation()
    label = "panel_%d_r%d_c%d" % (
        int(point.get("panel_index")), int(point.get("row")),
        int(point.get("column")))

    _checkpoint(logger, "%s: retracting to safe Z" % label)
    needle.lift_nozzle_to_safe_z(nozzle, cfg, logger)
    _checkpoint(logger, "%s: moving XY to %.4f, %.4f" % (label, x_mm, y_mm))
    safe = needle.mm_loc(x_mm, y_mm, float(cfg.get("safe_z_mm")), rotation)
    needle.move_direct_speed(nozzle, safe, cfg.get("travel_speed", 1.0))
    needle.move_direct_speed(nozzle, needle.mm_loc(x_mm, y_mm, start_z, rotation),
        cfg.get("travel_speed", 1.0))
    acquisition = str(probe.get("acquisition_method", "hardware_interrupt") or
        "hardware_interrupt").strip().lower()
    probe_cfg = {
        "probe_speed": probe.get("coarse_speed", 0.8),
        "fine_probe_speed": probe.get("fine_speed", 0.8),
        "z_probe": deep_copy(probe)
    }
    if acquisition == "polling":
        sequence = needle.run_polling_z_probe_sequence(
            nozzle, actuator, probe_cfg, x_mm, y_mm, start_z, logger, label)
        coarse_values = [float(value) for value in
            sequence.get("coarse_contact_z_values_mm", [])]
        coarse_records = deep_copy(sequence.get("coarse_records", []))
        fine_records = deep_copy(sequence.get("fine_records", []))
    else:
        sequence = needle.run_interrupt_probe_sequence(
            nozzle, actuator, probe_cfg, logger, label)
        sequence["acquisition_method"] = "hardware_interrupt"
        coarse_values = [float(sequence.get("coarse_contact_z_mm"))]
        coarse_records = [{"repeat": 1, "trigger_z_mm": coarse_values[0],
            "source": "hardware_interrupt"}]
        fine_records = []
    coarse_average = sum(coarse_values) / float(len(coarse_values))
    fine_values = [float(value) for value in
        sequence.get("fine_contact_z_values_mm", [])]
    if acquisition == "hardware_interrupt":
        fine_records = [{"repeat": index + 1, "trigger_z_mm": value,
            "source": "hardware_interrupt"}
            for index, value in enumerate(fine_values)]

    result = calculate_point_result(
        point, fine_values, cfg.get("voron_trigger_offset_mm", 0.0))
    result["coarse_trigger_z_values_mm"] = coarse_values
    result["coarse_averaged_trigger_z_mm"] = coarse_average
    result["coarse_records"] = coarse_records
    result["fine_records"] = fine_records
    result["acquisition_method"] = acquisition
    result["interrupt_sequence"] = (
        sequence if acquisition == "hardware_interrupt" else None)
    result["probe_sequence"] = sequence
    _checkpoint(logger, "%s: offset %.6f mm" %
        (label, float(result.get("z_offset_mm"))))
    needle.lift_nozzle_to_safe_z(nozzle, cfg, logger)
    return result


def run_mesh(cfg, nozzle=None, machine_obj=None, logger=None):
    needle = get_needle_backend()
    validate_config(cfg, True)
    if machine_obj is None:
        machine_obj = needle.get_machine(None)
    if nozzle is None:
        nozzle = needle.find_nozzle(cfg, None, machine_obj)
    actuator = find_probe_actuator(machine_obj, cfg, nozzle)
    result = {
        "ok": False,
        "stage": "bed_mesh",
        "message": "Bed mesh did not complete",
        "timestamp": datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "nozzle_name": needle.object_name(nozzle),
        "probe_actuator_name": needle.object_name(actuator),
        "voron_trigger_offset_mm": float(cfg.get("voron_trigger_offset_mm", 0.0)),
        "panels": [],
        "completed_points": 0,
        "total_points": 0,
        "csv_path": None,
    }
    grids = []
    for panel_index, panel in enumerate(cfg.get("panels", [])):
        if panel.get("enabled", False):
            grid = generate_grid(panel, panel_index + 1)
            grids.append((panel_index + 1, panel, grid))
            result["total_points"] += len(grid)
    _checkpoint(logger, "Bed mesh start: %d panels, %d points" %
        (len(grids), result.get("total_points")))
    probe_cfg = cfg.get("probe", {})
    acquisition = str(probe_cfg.get(
        "acquisition_method", "hardware_interrupt"))
    _checkpoint(logger, "Z touch method: %s" % acquisition)
    if acquisition == "hardware_interrupt":
        mode = str(probe_cfg.get("mode", "actuator"))
        _checkpoint(logger, "Z hardware mode: %s" % mode)
        if mode == "direct_gcode":
            _checkpoint(logger,
                "Z direct probe: channel=%s M204 S%.3f coarse F%.3f "
                "fine F%.3f repeats=%d attempts=%d" % (
                    str(probe_cfg.get("channel", "Q0")),
                    float(probe_cfg.get("probe_acceleration", 1000.0)),
                    float(probe_cfg.get("coarse_feed", 3000.0)),
                    float(probe_cfg.get("fine_feed", 500.0)),
                    int(float(probe_cfg.get("fine_repeats", 1))),
                    int(float(probe_cfg.get("g38_attempts", 2)))))
    else:
        _checkpoint(logger,
            "Z polling: coarse=%d fine=%d coarseStep=%.4f fineStep=%.4f" % (
                int(float(probe_cfg.get("polling_coarse_repeats", 1))),
                int(float(probe_cfg.get("polling_fine_repeats", 1))),
                float(probe_cfg.get("coarse_step_mm", 0.1)),
                float(probe_cfg.get("fine_step_mm", 0.01))))
    try:
        for panel_index, panel, grid in grids:
            panel_result = {
                "panel_index": panel_index,
                "panel_name": str(panel.get("name")),
                "config": deep_copy(panel),
                "points": [],
                "summary": summarize_points([]),
            }
            result["panels"].append(panel_result)
            _checkpoint(logger, "Starting %s (%d points)" %
                (panel_result.get("panel_name"), len(grid)))
            for point in grid:
                measured = probe_point(nozzle, actuator, cfg, point, logger)
                panel_result["points"].append(measured)
                panel_result["summary"] = summarize_points(panel_result["points"])
                result["completed_points"] += 1
        needle.lift_nozzle_to_safe_z(nozzle, cfg, logger)
        result["ok"] = True
        result["message"] = "Bed mesh completed"
        return result
    except Exception as error:
        result["message"] = str(error)
        result["error_type"] = error.__class__.__name__
        try:
            # Do not call the abort-aware logger during mandatory cleanup.
            needle.lift_nozzle_to_safe_z(nozzle, cfg, None)
        except:
            pass
        return result
    except JavaThrowable as error:
        # Jython Java Throwables do not derive from Python BaseException.
        result["message"] = str(error)
        result["error_type"] = error.__class__.__name__
        try:
            needle.lift_nozzle_to_safe_z(nozzle, cfg, None)
        except:
            pass
        return result


CSV_COLUMNS = [
    "run_timestamp", "panel_index", "panel_name", "row", "column",
    "x_mm", "y_mm", "expected_z_mm", "averaged_trigger_z_mm",
    "corrected_surface_z_mm", "z_offset_mm", "fine_repeat_count",
    "fine_spread_mm",
]


def _csv_cell(value):
    if value is None:
        return ""
    if isinstance(value, float):
        text = "%.9f" % value
    else:
        text = str(value)
    if any(character in text for character in [",", "\"", "\n", "\r"]):
        return '"%s"' % text.replace('"', '""')
    return text


def csv_text(result):
    lines = [",".join(CSV_COLUMNS)]
    timestamp = result.get("timestamp", "")
    for panel in result.get("panels", []):
        for point in panel.get("points", []):
            row = {
                "run_timestamp": timestamp,
                "panel_index": panel.get("panel_index"),
                "panel_name": panel.get("panel_name"),
            }
            row.update(point)
            lines.append(",".join([_csv_cell(row.get(column)) for column in CSV_COLUMNS]))
    return "\n".join(lines) + "\n"


def export_csv(result, output_directory=None, filename=None):
    if not result or not result.get("ok", False):
        raise BedMeshError("Only a successful completed mesh can be exported")
    directory = os.path.abspath(str(output_directory or DEFAULT_OUTPUT_DIR))
    if not os.path.isdir(directory):
        os.makedirs(directory)
    if filename is None:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = "bed_mesh_%s.csv" % stamp
    path = os.path.join(directory, filename)
    if os.path.exists(path):
        stem, extension = os.path.splitext(filename)
        suffix = 2
        while os.path.exists(os.path.join(
                directory, "%s_%d%s" % (stem, suffix, extension))):
            suffix += 1
        path = os.path.join(directory, "%s_%d%s" % (stem, suffix, extension))
    fh = open(path, "w")
    try:
        fh.write(csv_text(result))
    finally:
        fh.close()
    result["csv_path"] = path
    return path


if __name__ == "__main__":
    print("Bed mesh backend loaded. Launch Events/Homing/bed_mesh_gui.py to run a mesh.")
