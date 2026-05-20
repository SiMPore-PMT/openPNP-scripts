# lumen_tip_metrology.py
#
# NozzleCalibration.Starting event example:
#   from lumen_tip_metrology import event_preflight_before_nozzle_cal
#   event_preflight_before_nozzle_cal(CONFIG, nozzle=nozzle, camera=camera, machine_obj=machine)

import os
import sys
import math

from java.lang import Thread
from org.openpnp.model import Configuration, Location, LengthUnit, AxesLocation
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.spi.MotionPlanner import CompletionType

_HERE = os.path.dirname(__file__)
if _HERE not in sys.path:
    sys.path.append(_HERE)

from lumen_head_offset_solver import (
    solve_head_offset,
    mm_loc,
    loc_to_dict,
    normalize_rotation,
    pose_from_config,
    get_default_nozzle,
    get_default_camera,
    get_part,
    move_safe,
    move_direct,
    move_direct_speed,
    log_msg,
    find_artifact_top,
    pick_artifact_from_storage,
    place_artifact_to_cal_area,
    config_with_runtime_part_height,
    rotate_nozzle_lifted_unwrapped,
    empty_nozzle_rotation_speed,
)


_EVENT_ACTIVE = False
_SUPPRESS_EVENT = False


def log(msg):
    print "[lumen_tip_metrology] %s" % msg


def bool_config(config, name, default_value):
    return bool(config.get(name, default_value))


def normalize_rotation_360(angle_deg):
    angle = float(angle_deg)
    while angle < 0.0:
        angle += 360.0
    while angle >= 360.0:
        angle -= 360.0
    return angle


def copy_config_with_overrides(config, overrides):
    out = {}
    for key in config:
        out[key] = config[key]
    for key in overrides:
        out[key] = overrides[key]
    return out


def move_nozzle_to_command_zero(config, nozzle, logger=None):
    desired_zero = float(config.get("bottom_square_zero_deg", 0.0))
    speed = config.get("post_pairwise_zero_speed", 0.65)
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    target_r = normalize_rotation(desired_zero)
    target = mm_loc(
        current.getX(),
        current.getY(),
        current.getZ(),
        target_r,
    )
    log_msg(logger, "Post-pairwise nozzle zero restore: old R %.6f target R %.6f speed %.6f" % (
        float(current.getRotation()), float(target_r), float(speed)))
    move_direct_speed(nozzle, target, speed)
    final_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    final_r = float(final_loc.getRotation())
    log_msg(logger, "Post-pairwise nozzle zero restore complete: final R %.6f" % final_r)
    if abs(normalize_rotation(final_r - desired_zero)) > 0.05:
        raise Exception("Failed to restore nozzle to command zero after pairwise: R %.6f target %.6f" % (
            final_r, target_r))
    return final_loc


def get_machine(machine_obj=None):
    if machine_obj is not None:
        return machine_obj
    return Configuration.get().getMachine()


def get_nozzle(machine_obj, nozzle=None):
    if nozzle is not None:
        return nozzle
    return get_default_nozzle(machine_obj)


def get_top_camera(machine_obj):
    return get_default_camera(machine_obj)


def is_up_camera(camera):
    if camera is None:
        return False
    try:
        return camera.getLooking() == SpiCamera.Looking.Up
    except:
        return False


def get_bottom_camera(machine_obj, camera=None):
    if is_up_camera(camera):
        return camera
    for cam in machine_obj.getCameras():
        if is_up_camera(cam):
            return cam
    raise Exception("No bottom camera found")


def get_tip_calibration(nozzle):
    tip = nozzle.getNozzleTip()
    if tip is None:
        raise Exception("No nozzle tip loaded")
    calibration = tip.getCalibration()
    if calibration is None:
        raise Exception("Loaded nozzle tip has no calibration")
    return calibration


def loc_from_offset_dict(data, fallback=None):
    if hasattr(data, "convertToUnits"):
        return data.convertToUnits(LengthUnit.Millimeters)
    if "old_head_offsets_mm" in data:
        data = data["old_head_offsets_mm"]
    if "backup_head_offsets_mm" in data:
        data = data["backup_head_offsets_mm"]
    if fallback is None:
        fallback = Location(LengthUnit.Millimeters)
    fallback = fallback.convertToUnits(LengthUnit.Millimeters)
    return mm_loc(
        data.get("x", fallback.getX()),
        data.get("y", fallback.getY()),
        data.get("z", fallback.getZ()),
        data.get("rotation", data.get("r", fallback.getRotation())),
    )


def java_class_name(obj):
    try:
        return obj.getClass().getName()
    except:
        return str(type(obj))


class InvalidSquareReading(Exception):
    pass


def list_exactly_one(model):
    cname = java_class_name(model)
    if "RotatedRect" in cname:
        return model
    if "KeyPoint" in cname:
        raise InvalidSquareReading("Square stage returned KeyPoint, not raw RotatedRect")
    if hasattr(model, "size") and hasattr(model, "get"):
        if model.size() != 1:
            raise InvalidSquareReading("Square stage must return exactly one RotatedRect")
        return model.get(0)
    try:
        if len(model) != 1:
            raise InvalidSquareReading("Square stage must return exactly one RotatedRect")
        return model[0]
    except TypeError:
        pass
    raise InvalidSquareReading("Square stage result is not a raw RotatedRect")


def square_number_attr(obj, names):
    for name in names:
        try:
            return float(getattr(obj, name))
        except:
            pass
        try:
            getter = "get" + name[:1].upper() + name[1:]
            if hasattr(obj, getter):
                return float(getattr(obj, getter)())
        except:
            pass
    return None


def rotated_rect_width_height(item):
    size = None
    try:
        size = item.size
    except:
        pass
    if size is None:
        try:
            size = item.getSize()
        except:
            pass

    width = None
    height = None
    if size is not None:
        width = square_number_attr(size, ["width", "w"])
        height = square_number_attr(size, ["height", "h"])
    if width is None:
        width = square_number_attr(item, ["width", "w"])
    if height is None:
        height = square_number_attr(item, ["height", "h"])
    return width, height


def validate_square_rotated_rect(item, config):
    if "RotatedRect" not in java_class_name(item):
        raise InvalidSquareReading("Square stage returned %s, not RotatedRect" % java_class_name(item))
    width, height = rotated_rect_width_height(item)
    if width is None or height is None:
        raise InvalidSquareReading("Square stage RotatedRect has no readable width/height")
    width = abs(float(width))
    height = abs(float(height))
    if width <= 0.0 or height <= 0.0:
        raise InvalidSquareReading("Square stage RotatedRect has invalid width/height %.4f %.4f" % (
            width, height))
    ratio = max(width, height) / min(width, height)
    return width, height, ratio


def read_square_tip_angle_raw(config, nozzle=None, camera=None, machine_obj=None, logger=None):
    # This square stage is only for custom square-tip angle metrology. Built-in
    # nozzle-tip calibration should use the normal nozzle-tip calibration result
    # stage expected by OpenPnP. If a shared pipeline logs disabled square-stage
    # errors during built-in calibration, split square-tip metrology into a
    # separate pipeline later.
    machine_obj = get_machine(machine_obj)
    nozzle = get_nozzle(machine_obj, nozzle)
    camera = get_bottom_camera(machine_obj, camera)
    pipeline = get_tip_calibration(nozzle).getPipeline()
    if pipeline is None:
        raise Exception("Nozzle tip calibration pipeline is null")

    pipeline.setProperty("camera", camera)
    pipeline.setProperty("nozzle", nozzle)
    settle_ms = int(config.get("square_settle_ms", 250))
    stage_name = config.get("bottom_square_stage", "squareResults")
    max_invalid = int(config.get("square_invalid_retry_limit", 4))
    invalid_count = 0

    while True:
        if settle_ms > 0:
            Thread.sleep(settle_ms)
        try:
            camera.settleAndCapture()
        except:
            pass
        pipeline.process()

        try:
            result = pipeline.getResult(stage_name)
            if result is None:
                raise InvalidSquareReading("No result from square stage '%s'" % stage_name)
            item = list_exactly_one(result.getModel())
            width, height, ratio = validate_square_rotated_rect(item, config)
            angle = float(item.angle)
            log_msg(logger, "Square-tip raw angle: %.4f width %.4f height %.4f ratio %.4f" % (
                angle, width, height, ratio))
            return angle
        except InvalidSquareReading, e:
            invalid_count += 1
            log_msg(logger, "Invalid square-tip reading %d/%d: %s" % (
                invalid_count, max_invalid, e))
            if invalid_count >= max_invalid:
                raise
            log_msg(logger, "Retrying square-tip image without moving")


def read_square_tip_angle_with_recovery(config, nozzle, camera, machine_obj, target, logger=None):
    target = target.convertToUnits(LengthUnit.Millimeters)
    try:
        return read_square_tip_angle_raw(config, nozzle, camera, machine_obj, logger=logger)
    except InvalidSquareReading, first_error:
        recovery_rounds = int(config.get("square_invalid_recovery_rounds", 2))
        recovery_shift_mm = float(config.get("square_invalid_recovery_shift_mm", 0.10))
        recovery_offsets = [
            (recovery_shift_mm, 0.0),
            (0.0, recovery_shift_mm),
        ]
        last_error = first_error
        for i in range(recovery_rounds):
            dx, dy = recovery_offsets[i % len(recovery_offsets)]
            recovery_target = mm_loc(
                target.getX() + dx,
                target.getY() + dy,
                target.getZ(),
                target.getRotation(),
            )
            log_msg(logger, "Square-tip invalid recovery %d/%d: moving to X %.4f Y %.4f R %.6f after: %s" % (
                i + 1,
                recovery_rounds,
                float(recovery_target.getX()),
                float(recovery_target.getY()),
                float(recovery_target.getRotation()),
                last_error))
            move_direct(nozzle, recovery_target)
            try:
                return read_square_tip_angle_raw(config, nozzle, camera, machine_obj, logger=logger)
            except InvalidSquareReading, e:
                last_error = e
        raise last_error


def bottom_calibration_location(nozzle, camera):
    calibration = get_tip_calibration(nozzle)
    try:
        loc = calibration.getCalibrationLocation(camera, None)
    except:
        loc = camera.getLocation()
    loc = loc.convertToUnits(LengthUnit.Millimeters)
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    return mm_loc(loc.getX(), loc.getY(), loc.getZ(), current.getRotation())


def square_error(raw_angle, desired_zero):
    return ((raw_angle - desired_zero + 45.0) % 90.0) - 45.0


def median(values):
    vals = [float(v) for v in values]
    vals.sort()
    count = len(vals)
    if count == 0:
        return None
    mid = count // 2
    if count % 2:
        return vals[mid]
    return 0.5 * (vals[mid - 1] + vals[mid])


def square_sample_stats(samples):
    errors = []
    center_error = None
    for sample in samples:
        err = float(sample["normalized_error_deg"])
        errors.append(err)
        if str(sample.get("name")) == "center":
            center_error = err

    med = median(errors)
    avg = sum(errors) / float(len(errors)) if len(errors) else 0.0
    max_abs = 0.0
    min_err = None
    max_err = None
    for err in errors:
        if min_err is None or err < min_err:
            min_err = err
        if max_err is None or err > max_err:
            max_err = err
        if abs(err) > max_abs:
            max_abs = abs(err)
    spread = 0.0
    if min_err is not None and max_err is not None:
        spread = max_err - min_err

    if center_error is None:
        center_error = med

    return {
        "avg_error_deg": avg,
        "median_error_deg": med,
        "center_error_deg": center_error,
        "max_abs_error_deg": max_abs,
        "spread_deg": spread,
        "min_error_deg": min_err,
        "max_error_deg": max_err,
    }


def pick_square_error_estimate(stats, mode):
    mode = str(mode).lower()
    if mode == "center":
        return float(stats["center_error_deg"])
    if mode == "avg" or mode == "mean":
        return float(stats["avg_error_deg"])
    if mode == "median":
        return float(stats["median_error_deg"])
    raise Exception("Unsupported square error estimator: %s" % mode)


def normalize_delta_nearest_90(delta_deg):
    # returns equivalent delta in [-45, +45)
    return ((float(delta_deg) + 45.0) % 90.0) - 45.0


def normalize_angle_nearest_equivalent(target_deg, reference_deg):
    # For square-symmetric placement, choose target angle plus k*90
    # that is closest to reference angle.
    best = normalize_rotation(target_deg)
    best_err = abs(normalize_rotation(best - reference_deg))
    for k in [-4, -3, -2, -1, 0, 1, 2, 3, 4]:
        candidate = normalize_rotation(float(target_deg) + 90.0 * k)
        err = abs(normalize_rotation(candidate - reference_deg))
        if err < best_err:
            best = candidate
            best_err = err
    return best


def sample_square_tip_bottom(config, nozzle, camera, machine_obj, base_loc, desired_zero, logger=None):
    step = float(config.get("square_sample_step_mm", 0.25))
    invalid_validation_shift_mm = float(config.get("square_invalid_validation_shift_mm", 0.10))
    samples = [
        ("center", 0.0, 0.0),
        ("+X", step, 0.0),
        ("-X", -step, 0.0),
        ("+Y", 0.0, step),
        ("-Y", 0.0, -step),
    ]
    out = []
    total_error = 0.0
    base_loc = base_loc.convertToUnits(LengthUnit.Millimeters)
    for name, dx, dy in samples:
        target = mm_loc(
            base_loc.getX() + dx,
            base_loc.getY() + dy,
            base_loc.getZ(),
            base_loc.getRotation(),
        )
        move_direct(nozzle, target)
        try:
            raw_angle = read_square_tip_angle_with_recovery(config, nozzle, camera, machine_obj, target, logger=logger)
        except InvalidSquareReading, e:
            log_msg(logger, "Square-tip sample %s invalid after retries at X %.4f Y %.4f: %s" % (
                name, target.getX(), target.getY(), e))
            target = mm_loc(
                target.getX() + invalid_validation_shift_mm,
                target.getY() + invalid_validation_shift_mm,
                target.getZ(),
                target.getRotation(),
            )
            log_msg(logger, "Square-tip sample %s validation shift to X %.4f Y %.4f" % (
                name, target.getX(), target.getY()))
            move_direct(nozzle, target)
            raw_angle = read_square_tip_angle_with_recovery(config, nozzle, camera, machine_obj, target, logger=logger)
        read_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        error = square_error(raw_angle, desired_zero)
        total_error += error
        record = {
            "name": name,
            "offset_mm": {"x": dx, "y": dy},
            "location_mm": loc_to_dict(read_loc),
            "raw_angle_deg": raw_angle,
            "normalized_error_deg": error,
        }
        out.append(record)
        log_msg(logger, "Square-tip sample %s at X %.4f Y %.4f angle %.4f error %.4f" % (
            name, read_loc.getX(), read_loc.getY(), raw_angle, error))
    return out, total_error / float(len(out))


def read_center_square_error(config, nozzle, camera, machine_obj, base_loc, desired_zero, logger=None):
    base_loc = base_loc.convertToUnits(LengthUnit.Millimeters)
    target = mm_loc(
        base_loc.getX(),
        base_loc.getY(),
        base_loc.getZ(),
        base_loc.getRotation(),
    )
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    log_msg(logger, "Square-tip center read move: current R %.6f target R %.6f X %.4f Y %.4f Z %.4f" % (
        float(current.getRotation()),
        float(target.getRotation()),
        float(target.getX()),
        float(target.getY()),
        float(target.getZ())))
    move_direct(nozzle, target)
    raw_angle = read_square_tip_angle_with_recovery(config, nozzle, camera, machine_obj, target, logger=logger)
    read_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    error = square_error(raw_angle, desired_zero)
    record = {
        "name": "center",
        "offset_mm": {"x": 0.0, "y": 0.0},
        "location_mm": loc_to_dict(read_loc),
        "raw_angle_deg": raw_angle,
        "normalized_error_deg": error,
    }
    log_msg(logger, "Square-tip centered read: nozzle R %.6f raw angle %.6f normalized error %.6f" % (
        float(read_loc.getRotation()), float(raw_angle), float(error)))
    return raw_angle, error, record


def find_rotation_axis(machine_obj, config, logger=None):
    axis_name = str(config.get("rotation_axis_name", config.get("rot_axis_name", "a")))
    chosen = None
    axes = machine_obj.getAxes()
    for axis in axes:
        try:
            axis_id = axis.getId()
        except:
            axis_id = None
        try:
            name = axis.getName()
        except:
            name = None
        try:
            atype = str(axis.getType())
        except:
            atype = ""
        log_msg(logger, "Axis discovered for zeroing: id='%s' name='%s' type=%s" % (
            axis_id, name, atype))
        try:
            if name is not None and str(name).lower() == axis_name.lower():
                log_msg(logger, "Selected rotational axis by name '%s'" % name)
                return axis
        except:
            pass
        try:
            if axis_id is not None and str(axis_id).lower() == axis_name.lower():
                log_msg(logger, "Selected rotational axis by id '%s'" % axis_id)
                return axis
        except:
            pass
        if chosen is None and "ROTATION" in atype.upper():
            chosen = axis
    if chosen is not None:
        log_msg(logger, "Selected rotational axis by type fallback id='%s' name='%s'" % (
            chosen.getId(), chosen.getName()))
        return chosen
    raise Exception("No rotational axis found for true nozzle rotation zero")


def true_zero_nozzle_rotation_at_current_position(nozzle, config, logger=None):
    machine_obj = get_machine(None)
    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    current_r = float(current.getRotation())
    log_msg(logger, "True zero requested at current nozzle rotation R %.6f" % current_r)

    axis = find_rotation_axis(machine_obj, config, logger=logger)
    old_offset_value = get_rotation_global_offset_value(machine_obj, axis, logger=logger)
    sign = float(config.get("global_zero_offset_sign", -1.0))
    new_offset_value = old_offset_value + sign * current_r

    log_msg(logger, "Existing rotation global offset %.6f" % old_offset_value)
    log_msg(logger, "Computed new rotation global offset %.6f using sign %.1f" % (
        new_offset_value, sign))
    set_rotation_global_offset_value(machine_obj, axis, new_offset_value, logger=logger)
    wait_nozzle_still(nozzle)

    after = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    final_r = float(after.getRotation())
    log_msg(logger, "Nozzle rotation after true zero call: R %.6f" % final_r)
    if abs(normalize_rotation(final_r)) <= 0.05:
        return {
            "rotation_global_offset_before_deg": old_offset_value,
            "rotation_global_offset_after_deg": new_offset_value,
            "global_zero_offset_sign_used": sign,
        }

    if bool_config(config, "global_zero_auto_try_opposite_sign", True):
        opposite_sign = -sign
        opposite_offset_value = old_offset_value + opposite_sign * current_r
        log_msg(logger, "Computed new rotation global offset %.6f using sign %.1f" % (
            opposite_offset_value, opposite_sign))
        set_rotation_global_offset_value(machine_obj, axis, opposite_offset_value, logger=logger)
        wait_nozzle_still(nozzle)
        after = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        final_r = float(after.getRotation())
        log_msg(logger, "Nozzle rotation after true zero call: R %.6f" % final_r)
        if abs(normalize_rotation(final_r)) <= 0.05:
            log_msg(logger, "Opposite global zero sign succeeded; update config global_zero_offset_sign to %.1f" % opposite_sign)
            return {
                "rotation_global_offset_before_deg": old_offset_value,
                "rotation_global_offset_after_deg": opposite_offset_value,
                "global_zero_offset_sign_used": opposite_sign,
            }

    raise Exception("True zero failed: final R %.6f after applying global offset" % final_r)


def get_rotation_global_offset_value(machine_obj, axis, logger=None):
    planner = machine_obj.getMotionPlanner()
    offsets = None
    try:
        get_offsets = getattr(planner, "getGlobalOffsets")
    except:
        get_offsets = None
    try:
        if get_offsets is not None and callable(get_offsets):
            offsets = get_offsets()
    except Exception, e:
        log_msg(logger, "Could not read planner global offsets: %s" % e)

    value = read_axis_value_from_axes_location(offsets, axis)
    if value is not None:
        return value

    try:
        get_coordinate = getattr(axis, "getCoordinate")
    except:
        get_coordinate = None
    try:
        if get_coordinate is not None and callable(get_coordinate):
            return float(get_coordinate())
    except Exception, e:
        log_msg(logger, "Could not read rotation axis coordinate: %s" % e)

    try:
        get_length_coordinate = getattr(axis, "getLengthCoordinate")
    except:
        get_length_coordinate = None
    try:
        if get_length_coordinate is not None and callable(get_length_coordinate):
            length = get_length_coordinate()
            value = length_to_float(length)
            if value is not None:
                return value
    except Exception, e:
        log_msg(logger, "Could not read rotation axis length coordinate: %s" % e)

    log_msg(logger, "Could not read existing rotation global offset; assuming zero")
    return 0.0


def set_rotation_global_offset_value(machine_obj, axis, value, logger=None):
    planner = machine_obj.getMotionPlanner()
    planner.setGlobalOffsets(AxesLocation(axis, float(value)))
    log_msg(logger, "MotionPlanner global offset set: rotation axis '%s' offset %.6f" % (
        axis_id_for_log(axis), float(value)))
    try:
        planner.waitForCompletion(None, CompletionType.WaitForStillstand)
    except:
        pass
    return True


def wait_nozzle_still(nozzle):
    try:
        nozzle.waitForCompletion(CompletionType.WaitForStillstand)
    except:
        pass


def read_axis_value_from_axes_location(axes_location, axis):
    if axes_location is None:
        return None
    try:
        if hasattr(axes_location, "contains") and not axes_location.contains(axis):
            return None
    except:
        pass
    try:
        return float(axes_location.getCoordinate(axis))
    except:
        pass
    try:
        return length_to_float(axes_location.getLengthCoordinate(axis))
    except:
        pass
    return None


def length_to_float(value):
    if value is None:
        return None
    try:
        return float(value)
    except:
        pass
    try:
        converted = value.convertToUnits(LengthUnit.Millimeters)
        try:
            return float(converted.getValue())
        except:
            pass
        try:
            return float(converted.getValueAsDouble())
        except:
            pass
    except:
        pass
    try:
        return float(value.getValue())
    except:
        pass
    try:
        return float(value.getValueAsDouble())
    except:
        pass
    return None


def axis_id_for_log(axis):
    try:
        return str(axis.getId())
    except:
        pass
    try:
        return str(axis.getName())
    except:
        pass
    return str(axis)


def verify_square_tip_at_command_zero(config, nozzle, camera, machine_obj, base_loc, desired_zero, logger=None):
    samples_zero_verify = []
    if bool_config(config, "zeroed_square_verify_samples", True):
        samples_zero_verify, avg_error_legacy = sample_square_tip_bottom(
            config, nozzle, camera, machine_obj, base_loc, desired_zero, logger=logger)
    else:
        move_direct(nozzle, base_loc)
        raw_angle = read_square_tip_angle_with_recovery(config, nozzle, camera, machine_obj, base_loc, logger=logger)
        read_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        error = square_error(raw_angle, desired_zero)
        samples_zero_verify.append({
            "name": "center",
            "offset_mm": {"x": 0.0, "y": 0.0},
            "location_mm": loc_to_dict(read_loc),
            "raw_angle_deg": raw_angle,
            "normalized_error_deg": error,
        })
    zero_verify_stats = square_sample_stats(samples_zero_verify)
    zero_verify_error = pick_square_error_estimate(
        zero_verify_stats, config.get("square_refine_verify_estimator", "center"))
    log_msg(logger, "Zero verify square stats: center_error %.6f median_error %.6f avg_error %.6f max_abs %.6f spread %.6f verify_error %.6f" % (
        float(zero_verify_stats["center_error_deg"]),
        float(zero_verify_stats["median_error_deg"]),
        float(zero_verify_stats["avg_error_deg"]),
        float(zero_verify_stats["max_abs_error_deg"]),
        float(zero_verify_stats["spread_deg"]),
        float(zero_verify_error)))
    return samples_zero_verify, zero_verify_error, zero_verify_stats


def square_nozzle_tip_bottom(config, nozzle=None, camera=None, machine_obj=None, logger=None):
    machine_obj = get_machine(machine_obj)
    nozzle = get_nozzle(machine_obj, nozzle)
    camera = get_bottom_camera(machine_obj, camera)
    try:
        log_msg(logger, "Bottom camera selected: %s" % camera.getName())
    except:
        log_msg(logger, "Bottom camera selected: %s" % camera)

    base_loc = bottom_calibration_location(nozzle, camera)
    move_safe(nozzle, base_loc)
    desired_zero = float(config.get("bottom_square_zero_deg", 0.0))
    square_angle_sign = float(config.get("square_angle_sign", 1.0))

    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    log_msg(logger, "Square-tip initial nozzle DRO before parking: R %.6f" % float(current.getRotation()))
    park_target = mm_loc(
        current.getX(),
        current.getY(),
        current.getZ(),
        normalize_rotation(desired_zero),
    )
    log_msg(logger, "Square-tip parking move uses normal move_direct")
    move_direct(nozzle, park_target)

    parked = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    old_rotation = float(parked.getRotation())
    log_msg(logger, "Square-tip nozzle parked before measurement: R %.6f" % old_rotation)
    if abs(normalize_rotation(old_rotation - desired_zero)) > 0.05:
        raise Exception("Failed to park nozzle at command zero before square-tip measurement: R %.6f" % old_rotation)

    refine_max_iterations = int(config.get("square_refine_max_iterations", 5))
    if refine_max_iterations < 1:
        refine_max_iterations = 1
    square_refine_error_estimator = config.get("square_refine_error_estimator", "median")
    square_refine_verify_estimator = config.get("square_refine_verify_estimator", "center")
    refine_target = float(config.get(
        "square_refine_target_error_deg",
        config.get(
            "square_refine_target_max_error_deg",
            config.get("zeroed_square_verify_max_error_deg", 0.75))))
    max_spread = float(config.get("square_refine_max_spread_deg", 5.0))
    square_tip_center_only = bool_config(config, "square_tip_center_only", True)
    log_msg(logger, "Square-tip center-only zeroing enabled: %s" % square_tip_center_only)

    current_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    current_rotation = float(current_loc.getRotation())

    samples_before = None
    samples_after = None
    avg_error_before = None
    avg_error_after = None
    max_abs_error_after = None
    final_verify_error = None
    final_stats = None
    square_refine_iterations_used = 0

    for iteration in range(refine_max_iterations):
        sample_base = mm_loc(base_loc.getX(), base_loc.getY(), base_loc.getZ(), current_rotation)
        if square_tip_center_only:
            raw_angle, correction_error, center_sample = read_center_square_error(
                config, nozzle, camera, machine_obj, sample_base, desired_zero, logger=logger)
            samples = [center_sample]
            stats = square_sample_stats(samples)
            verify_error = correction_error
            spread = float(stats["spread_deg"])
            log_msg(logger, "Square-tip center refine iteration %d current R %.6f raw angle %.6f normalized error %.6f target %.6f" % (
                iteration + 1,
                current_rotation,
                float(raw_angle),
                float(correction_error),
                refine_target))
        else:
            samples, avg_error_legacy = sample_square_tip_bottom(
                config, nozzle, camera, machine_obj, sample_base, desired_zero, logger=logger)

            stats = square_sample_stats(samples)
            correction_error = pick_square_error_estimate(stats, square_refine_error_estimator)
            verify_error = pick_square_error_estimate(stats, square_refine_verify_estimator)
            spread = float(stats["spread_deg"])

            log_msg(logger, "Square-tip refine iteration %d current R %.6f center_error %.6f median_error %.6f avg_error %.6f max_abs %.6f spread %.6f correction_error %.6f verify_error %.6f target %.6f max_spread %.6f" % (
                iteration + 1,
                current_rotation,
                float(stats["center_error_deg"]),
                float(stats["median_error_deg"]),
                float(stats["avg_error_deg"]),
                float(stats["max_abs_error_deg"]),
                spread,
                correction_error,
                verify_error,
                refine_target,
                max_spread))

            if spread > max_spread:
                raise Exception("Square-tip scan spread %.4f deg exceeds %.4f deg; bottom-camera angle field too inconsistent" % (
                    spread, max_spread))

        if iteration == 0:
            samples_before = samples
            avg_error_before = stats["avg_error_deg"]

        samples_after = samples
        avg_error_after = stats["avg_error_deg"]
        max_abs_error_after = stats["max_abs_error_deg"]
        final_verify_error = verify_error
        final_stats = stats
        square_refine_iterations_used = iteration + 1

        if abs(verify_error) <= refine_target:
            break

        if iteration == refine_max_iterations - 1:
            break

        new_rotation = normalize_rotation(current_rotation + square_angle_sign * correction_error)
        target = mm_loc(base_loc.getX(), base_loc.getY(), base_loc.getZ(), new_rotation)
        log_msg(logger, "Square-tip refine correction move: current R %.6f error %.6f sign %.6f new R %.6f" % (
            current_rotation, correction_error, square_angle_sign, new_rotation))
        move_direct(nozzle, target)

        current_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        current_rotation = float(current_loc.getRotation())

    physically_aligned_rotation = current_rotation

    result = {
        "ok": True,
        "desired_zero_deg": desired_zero,
        "old_rotation_deg": old_rotation,
        "new_rotation_deg": physically_aligned_rotation,
        "physically_aligned_rotation_deg": physically_aligned_rotation,
        "true_zero_attempted": False,
        "final_nozzle_rotation_deg": physically_aligned_rotation,
        "zero_verified_at_command_zero": False,
        "zero_verify_max_abs_error_deg": None,
        "avg_error_before_deg": avg_error_before,
        "avg_error_after_deg": avg_error_after,
        "samples_before": samples_before,
        "samples_after": samples_after,
        "samples_zero_verify": [],
        "max_abs_error_after_deg": max_abs_error_after,
        "square_refine_iterations_used": square_refine_iterations_used,
        "square_refine_error_estimator": str(square_refine_error_estimator),
        "square_refine_verify_estimator": str(square_refine_verify_estimator),
        "square_refine_target_error_deg": refine_target,
        "square_refine_target_max_error_deg": refine_target,
        "square_refine_max_spread_deg": max_spread,
        "square_tip_center_only": square_tip_center_only,
        "square_refine_final_verify_error_deg": final_verify_error,
        "square_refine_final_stats": final_stats,
    }
    if final_stats is not None and float(final_stats["spread_deg"]) > max_spread:
        result["ok"] = False
        result["message"] = "Square-tip scan spread %.4f deg exceeds %.4f deg; bottom-camera angle field too inconsistent" % (
            float(final_stats["spread_deg"]), max_spread)
        raise Exception(result["message"])
    if abs(float(final_verify_error)) > refine_target:
        result["ok"] = False
        result["message"] = "Square-tip refinement failed before true-zero: verify error %.4f deg exceeds %.4f deg" % (
            final_verify_error, refine_target)
        raise Exception(result["message"])

    # Bottom-camera squaring can measure the C angle where the square tip is
    # physically aligned, but that is not the same as making commanded R=0
    # aligned. Do not feed that measured angle into pairwise as an offset unless
    # the machine coordinate system has actually been zeroed.
    log_msg(logger, "Square tip physically aligned at C command R %.6f" % physically_aligned_rotation)
    log_msg(logger, "enable_true_nozzle_rotation_zero config value: %s" % (
        config.get("enable_true_nozzle_rotation_zero", True)))
    if not bool_config(config, "enable_true_nozzle_rotation_zero", True):
        raise Exception("Square tip is aligned at R %.6f, but true C-axis zeroing is disabled/not implemented. Aborting before head-offset calibration." % physically_aligned_rotation)

    result["true_zero_attempted"] = True
    true_zero_result = true_zero_nozzle_rotation_at_current_position(nozzle, config, logger=logger)
    if true_zero_result is not None:
        for key in [
            "rotation_global_offset_before_deg",
            "rotation_global_offset_after_deg",
            "global_zero_offset_sign_used",
        ]:
            if key in true_zero_result:
                result[key] = true_zero_result[key]

    current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    log_msg(logger, "Nozzle rotation reread after true zeroing before zero command: R %.6f" % (
        float(current.getRotation())))
    zero_target = mm_loc(
        current.getX(),
        current.getY(),
        current.getZ(),
        normalize_rotation(desired_zero),
    )
    if abs(normalize_rotation(float(current.getRotation()) - desired_zero)) > 0.001:
        log_msg(logger, "Square-tip post-zero move uses normal move_direct")
        move_direct(nozzle, zero_target)
    else:
        log_msg(logger, "Nozzle already at command zero after true-zero; skipping zero move")
    final_loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    final_nozzle_rotation_deg = float(final_loc.getRotation())
    log_msg(logger, "Commanded nozzle to zero after true zeroing: R %.6f" % final_nozzle_rotation_deg)

    zero_base = mm_loc(base_loc.getX(), base_loc.getY(), base_loc.getZ(), final_nozzle_rotation_deg)
    if square_tip_center_only:
        zero_verify_raw_angle, zero_verify_error, zero_verify_sample = read_center_square_error(
            config, nozzle, camera, machine_obj, zero_base, desired_zero, logger=logger)
        samples_zero_verify = [zero_verify_sample]
        zero_verify_stats = square_sample_stats(samples_zero_verify)
        log_msg(logger, "Square-tip final centered verify after true zero: nozzle R %.6f raw angle %.6f normalized error %.6f" % (
            final_nozzle_rotation_deg,
            float(zero_verify_raw_angle),
            float(zero_verify_error)))
    else:
        samples_zero_verify, zero_verify_error, zero_verify_stats = verify_square_tip_at_command_zero(
            config, nozzle, camera, machine_obj, zero_base, desired_zero, logger=logger)
    zero_verify_max_allowed = float(config.get("zeroed_square_verify_max_error_deg", 0.75))
    result["samples_zero_verify"] = samples_zero_verify
    result["zero_verify_error_deg"] = zero_verify_error
    result["zero_verify_stats"] = zero_verify_stats
    result["zero_verify_max_abs_error_deg"] = zero_verify_stats["max_abs_error_deg"]
    result["zero_verify_spread_deg"] = zero_verify_stats["spread_deg"]
    result["final_nozzle_rotation_deg"] = final_nozzle_rotation_deg
    if float(zero_verify_stats["spread_deg"]) > max_spread:
        result["ok"] = False
        result["message"] = "Nozzle true-zero verification scan spread %.4f deg exceeds %.4f deg" % (
            float(zero_verify_stats["spread_deg"]), max_spread)
        raise Exception(result["message"])
    if abs(float(zero_verify_error)) > zero_verify_max_allowed:
        result["ok"] = False
        result["message"] = "Nozzle true-zero verification failed at R=0: verify error %.4f deg exceeds %.4f deg" % (
            zero_verify_error, zero_verify_max_allowed)
        raise Exception(result["message"])
    result["zero_verified_at_command_zero"] = True
    return result


def apply_head_offsets_with_backup(nozzle, proposed_head_offsets_mm, logger=None):
    old_offsets = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
    proposed = loc_from_offset_dict(proposed_head_offsets_mm, old_offsets)
    new_offsets = mm_loc(
        proposed.getX(),
        proposed.getY(),
        proposed.getZ(),
        proposed.getRotation(),
    )
    log_msg(logger, "Applying head offsets: old %s new %s" % (
        loc_to_dict(old_offsets), loc_to_dict(new_offsets)))
    nozzle.setHeadOffsets(new_offsets)
    return {
        "ok": True,
        "backup_head_offsets_mm": loc_to_dict(old_offsets),
        "old_head_offsets_mm": loc_to_dict(old_offsets),
        "applied_head_offsets_mm": loc_to_dict(new_offsets),
    }


def restore_head_offsets(nozzle, backup):
    old_offsets = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
    restored = loc_from_offset_dict(backup, old_offsets)
    nozzle.setHeadOffsets(restored)
    # Changing X/Y head offsets invalidates nozzle-tip calibration.
    return {
        "ok": True,
        "old_head_offsets_mm": loc_to_dict(old_offsets),
        "restored_head_offsets_mm": loc_to_dict(restored),
        "requires_nozzle_tip_recalibration": True,
    }


def event_preflight_before_nozzle_cal(config, nozzle=None, camera=None, machine_obj=None):
    global _EVENT_ACTIVE
    if _SUPPRESS_EVENT:
        return {
            "ok": True,
            "message": "event preflight suppressed",
            "square_tip_skipped": True,
            "head_offset_solver_skipped": True,
        }
    if _EVENT_ACTIVE:
        return {
            "ok": True,
            "message": "event preflight re-entry skipped",
            "square_tip_skipped": True,
            "head_offset_solver_skipped": True,
        }

    _EVENT_ACTIVE = True
    try:
        machine_obj = get_machine(machine_obj)
        nozzle = get_nozzle(machine_obj, nozzle)
        top_camera = get_top_camera(machine_obj)

        square_tip_skipped = not bool_config(config, "square_tip_in_event", True)
        head_offset_solver_skipped = not bool_config(config, "run_head_offset_solver_in_event", True)
        result = {
            "ok": True,
            "message": "ok",
            "square_tip_skipped": square_tip_skipped,
            "head_offset_solver_skipped": head_offset_solver_skipped,
            "square": None,
            "head_offset_solver": None,
            "head_offsets": None,
        }

        if not square_tip_skipped:
            log("Squaring nozzle tip before built-in calibration")
            result["square"] = square_nozzle_tip_bottom(config, nozzle, camera, machine_obj)

        if not head_offset_solver_skipped:
            log("Solving nozzle head offset before built-in calibration")
            solver = solve_head_offset(config, nozzle=nozzle, camera=top_camera, machine_obj=machine_obj)
            result["head_offset_solver"] = solver
            if not solver.get("ok", False):
                raise Exception("Head offset solver failed: %s" % solver.get("message"))
            result["head_offsets"] = apply_head_offsets_with_backup(
                nozzle, solver["proposed_head_offsets_mm"])
            move_nozzle_to_command_zero(config, nozzle, logger=log)
            before_builtin = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            log_msg(log, "Nozzle R before built-in nozzle-tip calibration: %.6f" % (
                float(before_builtin.getRotation())))

        return result
    finally:
        _EVENT_ACTIVE = False


def run_builtin_nozzle_tip_calibration(nozzle):
    tip = nozzle.getNozzleTip()
    if tip is None:
        raise Exception("No nozzle tip loaded")
    calibration = tip.getCalibration()
    if calibration is None:
        raise Exception("Loaded nozzle tip has no calibration")
    try:
        get_part_method = getattr(nozzle, "getPart")
    except:
        get_part_method = None
    try:
        get_part_callable = callable(get_part_method)
    except:
        get_part_callable = False
    if get_part_callable:
        part = get_part_method()
        if part is not None:
            raise Exception("Cannot run built-in nozzle-tip calibration because OpenPnP still reports part on nozzle: %s" % part)
    calibration.calibrate(nozzle)
    return {"ok": True, "tip": tip.getName()}


def return_artifact_to_storage(config, nozzle, camera, machine_obj, logger=None):
    log_msg(logger, "Artifact return start")
    part = get_part(config, None)
    config = config_with_runtime_part_height(config, part, logger=logger)
    calibration_pose = pose_from_config(config["calibration_location"])
    storage_pose = pose_from_config(config["storage_location"])

    log_msg(logger, "Artifact return re-image start")
    found = find_artifact_top(camera, nozzle, part, calibration_pose, config, logger=logger)
    log_msg(logger, "Artifact return measured center: %s" % loc_to_dict(found["center_location"]))
    log_msg(logger, "Artifact return measured search offset: %s" % found.get("search_offset_mm"))

    artifact_angle_deg = normalize_rotation_360(float(found["artifact_angle_deg"]))
    artifact_pick_angle_sign = float(config.get("artifact_pick_angle_sign", -1.0))
    artifact_pick_angle_offset_deg = float(config.get("artifact_pick_angle_offset_deg", 0.0))
    pick_rotation_deg = normalize_rotation(
        artifact_pick_angle_sign * artifact_angle_deg
        + artifact_pick_angle_offset_deg)
    place_rotation_deg = float(storage_pose.getRotation())
    log_msg(logger, "Artifact return measured artifact angle: %.4f" % artifact_angle_deg)
    log_msg(logger, "Artifact return pick angle sign: %.4f" % artifact_pick_angle_sign)
    log_msg(logger, "Artifact return pick angle offset deg: %.4f" % (
        artifact_pick_angle_offset_deg))
    log_msg(logger, "Artifact return pick rotation deg: %.4f" % pick_rotation_deg)
    log_msg(logger, "Artifact return nominal storage target: X %.4f Y %.4f R %.4f" % (
        float(storage_pose.getX()), float(storage_pose.getY()), float(storage_pose.getRotation())))
    log_msg(logger, "Artifact return final place_rotation_deg: %.4f" % place_rotation_deg)

    pick_config = copy_config_with_overrides(config, {
        "storage_location": config["calibration_location"],
        "transfer_z_mode": "safe_z",
    })
    place_config = copy_config_with_overrides(config, {
        "calibration_location": config["storage_location"],
        "transfer_z_mode": "safe_z",
    })

    pick = pick_artifact_from_storage(
        nozzle, part, found["center_location"], pick_config, pick_rotation_deg,
        logger=logger)
    place = place_artifact_to_cal_area(
        nozzle, part, storage_pose, place_config, place_rotation_deg,
        logger=logger)
    verified = find_artifact_top(camera, nozzle, part, storage_pose, config, logger=logger)
    verified_center = verified["center_location"].convertToUnits(LengthUnit.Millimeters)
    verified_angle = normalize_rotation(float(verified["artifact_angle_deg"]))
    dx = float(verified_center.getX()) - float(storage_pose.getX())
    dy = float(verified_center.getY()) - float(storage_pose.getY())
    da = normalize_rotation(verified_angle - float(storage_pose.getRotation()))
    log_msg(logger, "Storage return verify actual center: %s angle %.4f" % (
        loc_to_dict(verified_center), verified_angle))
    log_msg(logger, "Storage return verify error: X %.6f Y %.6f R %.6f" % (
        dx, dy, da))
    if "storage_return_verify_max_xy_error_mm" in config:
        max_xy_error = float(config["storage_return_verify_max_xy_error_mm"])
        xy_error = math.sqrt(dx * dx + dy * dy)
        if xy_error > max_xy_error:
            raise Exception("Storage return verify XY error %.6f exceeds %.6f" % (
                xy_error, max_xy_error))
    if "storage_return_verify_max_angle_error_deg" in config:
        max_angle_error = float(config["storage_return_verify_max_angle_error_deg"])
        if abs(da) > max_angle_error:
            raise Exception("Storage return verify angle error %.6f exceeds %.6f" % (
                da, max_angle_error))
    log_msg(logger, "Artifact return end")

    return {
        "ok": True,
        "artifact_angle_before_return_deg": artifact_angle_deg,
        "artifact_pick_angle_sign": artifact_pick_angle_sign,
        "artifact_pick_angle_offset_deg": artifact_pick_angle_offset_deg,
        "pick_rotation_deg": pick_rotation_deg,
        "place_rotation_deg": place_rotation_deg,
        "storage_target_location": loc_to_dict(storage_pose),
        "measurement": {
            "center_location": loc_to_dict(found["center_location"]),
            "artifact_angle_deg": artifact_angle_deg,
            "big_circle": found["big_circle"],
            "small_circle": found["small_circle"],
            "search_offset_mm": found.get("search_offset_mm"),
            "pixel_to_machine": found["pixel_to_machine"],
        },
        "pick": pick,
        "place": place,
        "storage_verify": {
            "center_location": loc_to_dict(verified_center),
            "artifact_angle_deg": verified_angle,
            "big_circle": verified["big_circle"],
            "small_circle": verified["small_circle"],
            "search_offset_mm": verified.get("search_offset_mm"),
            "pixel_to_machine": verified["pixel_to_machine"],
        },
        "storage_verify_error_mm": {"x": dx, "y": dy},
        "storage_verify_error_deg": da,
    }


def run_full_sequence(config, nozzle=None, machine_obj=None, logger=None):
    global _SUPPRESS_EVENT
    result = {
        "ok": False,
        "message": None,
        "square_tip_skipped": False,
        "head_offset_solver_skipped": False,
        "backup_head_offsets_mm": None,
        "square": None,
        "head_offset_solver": None,
        "head_offsets": None,
        "builtin_calibration": None,
        "artifact_return": None,
    }
    try:
        log_msg(logger, "Start full sequence")
        machine_obj = get_machine(machine_obj)
        nozzle = get_nozzle(machine_obj, nozzle)
        try:
            log_msg(logger, "Selected nozzle name: %s" % nozzle.getName())
        except:
            log_msg(logger, "Selected nozzle name: %s" % nozzle)
        part = get_part(config, None)
        try:
            log_msg(logger, "Selected part ID: %s" % part.getId())
        except:
            log_msg(logger, "Selected part ID: %s" % part)
        top_camera = get_top_camera(machine_obj)
        try:
            log_msg(logger, "Top camera selected: %s" % top_camera.getName())
        except:
            log_msg(logger, "Top camera selected: %s" % top_camera)
        result["backup_head_offsets_mm"] = loc_to_dict(
            nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))

        result["square"] = square_nozzle_tip_bottom(config, nozzle, None, machine_obj, logger=logger)

        log_msg(logger, "Nozzle command R=0 verified square; starting head offset solver")
        solver = solve_head_offset(config, nozzle=nozzle, camera=top_camera,
            machine_obj=machine_obj, part=part, logger=logger)
        result["head_offset_solver"] = solver
        if not solver.get("ok", False):
            raise Exception("Head offset solver failed: %s" % solver.get("message"))
        result["head_offsets"] = apply_head_offsets_with_backup(
            nozzle, solver["proposed_head_offsets_mm"], logger=logger)
        move_nozzle_to_command_zero(config, nozzle, logger=logger)

        _SUPPRESS_EVENT = True
        try:
            log_msg(logger, "Checking nozzle is empty before built-in nozzle-tip calibration")
            try:
                get_part_method = getattr(nozzle, "getPart")
            except:
                get_part_method = None
            try:
                get_part_callable = callable(get_part_method)
            except:
                get_part_callable = False
            if get_part_callable:
                part_on_nozzle = get_part_method()
                if part_on_nozzle is not None:
                    raise Exception("Cannot run built-in nozzle-tip calibration because OpenPnP still reports part on nozzle: %s" % part_on_nozzle)
            log_msg(logger, "Nozzle is empty; built-in nozzle-tip calibration can start")
            log_msg(logger, "Pre-built-in nozzle-tip calibration zero move start")
            before_builtin = rotate_nozzle_lifted_unwrapped(
                nozzle, 0.0, logger=logger, speed=empty_nozzle_rotation_speed(config))
            log_msg(logger, "Pre-built-in nozzle-tip calibration zero move complete: final R %.6f" % (
                float(before_builtin.getRotation())))
            before_builtin = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            log_msg(logger, "Nozzle R before built-in nozzle-tip calibration: %.6f" % (
                float(before_builtin.getRotation())))
            log_msg(logger, "Built-in nozzle-tip calibration start")
            result["builtin_calibration"] = run_builtin_nozzle_tip_calibration(nozzle)
            log_msg(logger, "Built-in nozzle-tip calibration end")
            log_msg(logger, "Post-built-in nozzle-tip calibration zero restore start")
            after_builtin = rotate_nozzle_lifted_unwrapped(
                nozzle, 0.0, logger=logger, speed=empty_nozzle_rotation_speed(config))
            log_msg(logger, "Post-built-in nozzle-tip calibration zero restore complete: final R %.6f" % (
                float(after_builtin.getRotation())))
        finally:
            _SUPPRESS_EVENT = False

        result["artifact_return"] = return_artifact_to_storage(
            config, nozzle, top_camera, machine_obj, logger=logger)
        result["ok"] = True
        result["message"] = "ok"
        return result
    except Exception, e:
        log_msg(logger, "Full sequence exception: %s" % e)
        result["message"] = str(e)
        return result
