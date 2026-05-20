# lumen_head_offset_solver.py

import math

from java.lang import Thread, Throwable
from org.openpnp.model import Configuration, Location, LengthUnit
from org.openpnp.spi import Nozzle as SpiNozzle
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import MovableUtils, VisionUtils


DEFAULT_ANGLES_DEG = [-150.0, -90.0, -30.0, 30.0, 90.0, 150.0]


def log_msg(logger, msg):
    if logger:
        logger(str(msg))
    else:
        print str(msg)


def mm_loc(x, y, z, r):
    return Location(LengthUnit.Millimeters, float(x), float(y), float(z), float(r))


def loc_to_dict(loc):
    loc = loc.convertToUnits(LengthUnit.Millimeters)
    return {
        "x": float(loc.getX()),
        "y": float(loc.getY()),
        "z": float(loc.getZ()),
        "rotation": float(loc.getRotation()),
    }


def loc_to_log(loc):
    loc = loc.convertToUnits(LengthUnit.Millimeters)
    return "X %.6f Y %.6f Z %.6f R %.6f" % (
        float(loc.getX()),
        float(loc.getY()),
        float(loc.getZ()),
        float(loc.getRotation()),
    )


def wait_still(movable):
    movable.waitForCompletion(CompletionType.WaitForStillstand)


def move_safe(movable, target):
    target = target.convertToUnits(LengthUnit.Millimeters)
    MovableUtils.moveToLocationAtSafeZ(movable, target)
    wait_still(movable)


def move_direct(movable, target):
    move_direct_speed(movable, target, None)


def move_direct_speed(movable, target, speed=None):
    target = target.convertToUnits(LengthUnit.Millimeters)
    if speed is not None:
        try:
            movable.moveTo(target, float(speed))
        except TypeError:
            movable.moveTo(target)
    else:
        try:
            movable.moveTo(target, 1.0)
        except TypeError:
            movable.moveTo(target)
    wait_still(movable)


def held_die_speed(config):
    return float(config.get("held_die_motion_speed", 0.60))


def empty_nozzle_rotation_speed(config):
    return float(config.get("empty_nozzle_rotation_speed", 0.20))


def normal_motion_speed(config):
    return float(config.get("normal_motion_speed", 1.0))


def get_movable_location(movable):
    return movable.getLocation().convertToUnits(LengthUnit.Millimeters)


def lift_nozzle_relative(nozzle, dz_mm, logger=None, speed=None):
    current = get_movable_location(nozzle)
    target = mm_loc(
        current.getX(),
        current.getY(),
        current.getZ() + float(dz_mm),
        current.getRotation(),
    )
    move_direct_speed(nozzle, target, speed)
    lifted = get_movable_location(nozzle)
    log_msg(logger, "Nozzle relative lift: old Z %.6f new Z %.6f" % (
        float(current.getZ()), float(lifted.getZ())))
    return lifted


def safe_travel_z_for_config(config):
    return float(config.get("safe_travel_z", config.get("safe_z_mm", 25.0)))


def lift_nozzle_to_safe_z(nozzle, config, logger=None, speed=None):
    current = get_movable_location(nozzle)
    target = mm_loc(
        current.getX(),
        current.getY(),
        safe_travel_z_for_config(config),
        current.getRotation(),
    )
    move_direct_speed(nozzle, target, speed)
    lifted = get_movable_location(nozzle)
    log_msg(logger, "Nozzle safe Z lift: old Z %.6f new Z %.6f" % (
        float(current.getZ()), float(lifted.getZ())))
    return lifted


def lower_nozzle_to(nozzle, z_mm, logger=None, speed=None):
    current = get_movable_location(nozzle)
    target = mm_loc(
        current.getX(),
        current.getY(),
        float(z_mm),
        current.getRotation(),
    )
    log_msg(logger, "Place lower old readback/DRO: %s" % loc_to_log(current))
    log_msg(logger, "Place lower commanded target: %s" % loc_to_log(target))
    move_direct_speed(nozzle, target, speed)
    lowered = get_movable_location(nozzle)
    z_error = float(lowered.getZ()) - float(target.getZ())
    log_msg(logger, "Place lower actual readback/DRO: %s" % loc_to_log(lowered))
    log_msg(logger, "Place lower Z readback: commanded %.6f actual %.6f error %.6f" % (
        float(target.getZ()), float(lowered.getZ()), z_error))
    return lowered


def config_with_transfer_z_mode(config, transfer_z_mode):
    out = {}
    for key in config:
        out[key] = config[key]
    out["transfer_z_mode"] = transfer_z_mode
    return out


def length_to_mm(value):
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


def get_openpnp_part_height_mm(part):
    if part is None:
        return None
    for name in ["getHeight", "getPackageHeight", "getPartHeight"]:
        try:
            method = getattr(part, name)
            value = method()
            mm = length_to_mm(value)
            if mm is not None:
                return mm
        except:
            pass
    return None


def config_with_runtime_part_height(config, part, logger=None):
    out = {}
    for key in config:
        out[key] = config[key]

    mm = get_openpnp_part_height_mm(part)
    if mm is None:
        raise Exception("Could not read OpenPnP part height from part '%s'; refusing to use config part_height_mm for motion Z" % part)

    out["part_height_mm"] = mm
    out["runtime_part_height_mm"] = mm
    log_msg(logger, "Runtime OpenPnP part height: %.6f mm" % mm)
    return out


def rotate_nozzle_lifted_unwrapped(nozzle, target_rotation_deg, logger=None, speed=None):
    current = get_movable_location(nozzle).convertToUnits(LengthUnit.Millimeters)
    current_r = float(current.getRotation())
    target_r = unwrap_rotation_near(current_r, target_rotation_deg)

    target = Location(
        LengthUnit.Millimeters,
        float(current.getX()),
        float(current.getY()),
        float(current.getZ()),
        target_r
    )

    log_msg(logger, "Rotate nozzle lifted unwrapped: current %.6f target %.6f delta %.6f speed %s" %
        (current_r, target_r, target_r - current_r, str(speed)))

    move_direct_speed(nozzle, target, speed)
    return get_movable_location(nozzle)


def nozzle_has_part(nozzle):
    try:
        get_part = getattr(nozzle, "getPart")
    except:
        get_part = None
    try:
        if get_part is not None and callable(get_part):
            return get_part() is not None
    except:
        pass
    return False


def move_nozzle_xy_lifted(nozzle, x, y, logger=None, speed=None):
    current = get_movable_location(nozzle)
    target = mm_loc(
        float(x),
        float(y),
        current.getZ(),
        current.getRotation(),
    )
    move_direct_speed(nozzle, target, speed)
    moved = get_movable_location(nozzle)
    log_msg(logger, "Place XY move lifted: old X/Y %.6f %.6f new X/Y %.6f %.6f" % (
        float(current.getX()), float(current.getY()), float(moved.getX()), float(moved.getY())))
    return moved


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


def normalize_rotation(angle_deg):
    angle = float(angle_deg)
    while angle <= -180.0:
        angle += 360.0
    while angle > 180.0:
        angle -= 360.0
    return angle


def normalize_rotation_near(angle_deg, reference_deg):
    angle = float(angle_deg)
    reference = float(reference_deg)
    while angle - reference > 180.0:
        angle -= 360.0
    while angle - reference < -180.0:
        angle += 360.0
    return angle


def solve_rotation_center_correction(dx, dy, theta_deg):
    theta = math.radians(float(theta_deg))
    c = math.cos(theta)
    s = math.sin(theta)

    a = 1.0 - c
    b = s
    cc = -s
    d = 1.0 - c

    det = a * d - b * cc
    if abs(det) < 1.0e-9:
        raise Exception("Rotation delta too close to zero for pairwise solve")

    corr_x = (d * dx - b * dy) / det
    corr_y = (-cc * dx + a * dy) / det
    return corr_x, corr_y


def mean(values):
    if len(values) == 0:
        raise Exception("Cannot compute mean of empty list")
    total = 0.0
    for value in values:
        total += float(value)
    return total / float(len(values))


def median(values):
    if len(values) == 0:
        raise Exception("Cannot compute median of empty list")
    sorted_values = sorted([float(value) for value in values])
    count = len(sorted_values)
    mid = count / 2
    if count % 2 == 1:
        return sorted_values[mid]
    return (sorted_values[mid - 1] + sorted_values[mid]) / 2.0


def compute_correction_estimate(valid_corrections, config, logger):
    if len(valid_corrections) == 0:
        raise Exception("No valid pairwise corrections")

    estimator = str(config.get("pairwise_correction_estimator", "trimmed_mean"))
    trim_worst_count = 0
    used = valid_corrections

    if estimator == "mean":
        pass
    elif estimator == "median":
        pass
    elif estimator == "trimmed_mean":
        if "pairwise_trim_worst_count" in config:
            trim_worst_count = int(config["pairwise_trim_worst_count"])
        elif len(valid_corrections) >= 4:
            trim_worst_count = 1
        if trim_worst_count < 0:
            trim_worst_count = 0
        if trim_worst_count >= len(valid_corrections):
            trim_worst_count = len(valid_corrections) - 1
        ranked = sorted(valid_corrections, key=lambda item: item["corr_mag"], reverse=True)
        used = ranked[trim_worst_count:]
    else:
        raise Exception("Unsupported pairwise_correction_estimator: %s" % estimator)

    if len(used) == 0:
        raise Exception("No pairwise corrections remain after trimming")

    xs = [item["corr_x"] for item in used]
    ys = [item["corr_y"] for item in used]
    if estimator == "median":
        corr_x = median(xs)
        corr_y = median(ys)
    else:
        corr_x = mean(xs)
        corr_y = mean(ys)

    log_msg(logger, "Pairwise correction estimator: %s valid %d used %d trim_worst %d" % (
        estimator, len(valid_corrections), len(used), trim_worst_count))

    return {
        "corr_x": corr_x,
        "corr_y": corr_y,
        "estimator": estimator,
        "trim_worst_count_used": trim_worst_count,
        "used_count": len(used),
    }


def unwrap_rotation_near(current_deg, target_equiv_deg):
    current = float(current_deg)
    target = float(target_equiv_deg)
    while target - current > 180.0:
        target -= 360.0
    while target - current < -180.0:
        target += 360.0
    return target


def unwrapped_plus_180(start_deg, direction=1.0):
    if float(direction) >= 0.0:
        return float(start_deg) + 180.0
    return float(start_deg) - 180.0


def pose_from_config(data):
    return mm_loc(data["x"], data["y"], data["z"], data.get("r", data.get("rotation", 0.0)))


def top_z_for_pose(loc, config):
    loc = loc.convertToUnits(LengthUnit.Millimeters)
    return float(loc.getZ()) + float(config["part_height_mm"])


def xy_with_base_pose(center_loc, base_pose):
    center_loc = center_loc.convertToUnits(LengthUnit.Millimeters)
    base_pose = base_pose.convertToUnits(LengthUnit.Millimeters)
    return mm_loc(center_loc.getX(), center_loc.getY(), base_pose.getZ(), base_pose.getRotation())


def nozzle_target_for_xy(nozzle, center_loc, base_pose, config, rotation_deg):
    center_loc = center_loc.convertToUnits(LengthUnit.Millimeters)
    base_pose = base_pose.convertToUnits(LengthUnit.Millimeters)
    return mm_loc(
        center_loc.getX(),
        center_loc.getY(),
        top_z_for_pose(base_pose, config),
        float(rotation_deg),
    )


def get_default_nozzle(machine_obj):
    head = machine_obj.getDefaultHead()
    if head is None:
        raise Exception("No default head")
    nozzle = head.getDefaultNozzle()
    if nozzle is None:
        raise Exception("No default nozzle")
    return nozzle


def get_default_camera(machine_obj):
    head = machine_obj.getDefaultHead()
    if head is None:
        raise Exception("No default head")
    camera = head.getDefaultCamera()
    if camera is None:
        raise Exception("No default camera")
    return camera


def get_part(config, part):
    if part is not None:
        return part
    part = Configuration.get().getPart(config["part_id"])
    if part is None:
        raise Exception("Part not found: %s" % config["part_id"])
    return part


def get_attr(obj, names):
    for name in names:
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


def parse_circle(item):
    center = get_attr(item, ["center", "pt", "point"])
    if center is not None:
        x = number_attr(center, ["x"])
        y = number_attr(center, ["y"])
    else:
        x = number_attr(item, ["x"])
        y = number_attr(item, ["y"])

    diameter = number_attr(item, ["diameter", "d", "size"])
    if diameter is None:
        radius = number_attr(item, ["radius", "r"])
        if radius is not None:
            diameter = radius * 2.0

    if x is None or y is None or diameter is None:
        raise Exception("Pipeline result shape wrong")

    return {
        "x_px": x,
        "y_px": y,
        "diameter_px": diameter,
        "raw": str(item),
    }


def validate_circle_diameters(big_circle, small_circle, config):
    expected_big = config.get("expected_big_diameter_px")
    expected_small = config.get("expected_small_diameter_px")
    if expected_big is None and expected_small is None:
        return
    tolerance = float(config.get("diameter_tolerance_px", 0.0))
    if expected_big is not None:
        if abs(big_circle["diameter_px"] - float(expected_big)) > tolerance:
            raise Exception("Pipeline result shape wrong")
    if expected_small is not None:
        if abs(small_circle["diameter_px"] - float(expected_small)) > tolerance:
            raise Exception("Pipeline result shape wrong")


def pixel_to_machine_locations(camera, nozzle, x_px, y_px):
    camera_only = VisionUtils.getPixelLocation(camera, float(x_px), float(y_px))
    camera_only = camera_only.convertToUnits(LengthUnit.Millimeters)
    tool_aware = VisionUtils.getPixelLocation(camera, nozzle, float(x_px), float(y_px))
    tool_aware = tool_aware.convertToUnits(LengthUnit.Millimeters)
    return {
        "camera_only": camera_only,
        "tool_aware": tool_aware,
        "debug": {
            "camera_only": loc_to_dict(camera_only),
            "tool_aware": loc_to_dict(tool_aware),
        },
    }


def detection_pick_location(found):
    if found is not None and "nozzle_pick_location" in found:
        return found["nozzle_pick_location"]
    return found["center_location"]


def set_actuator_state(actuator, on_off):
    if actuator is None:
        return False
    if hasattr(actuator, "actuate"):
        actuator.actuate(bool(on_off))
    elif hasattr(actuator, "setActuated"):
        actuator.setActuated(bool(on_off))
    else:
        raise Exception("Actuator has no supported state method")
    return True


def get_vacuum_actuator(nozzle):
    for name in ["getVacuumActuator", "getVacuumValveActuator"]:
        try:
            actuator = getattr(nozzle, name)()
            if actuator is not None:
                return actuator
        except:
            pass
    try:
        head = nozzle.getHead()
        for name in ["getVacuumActuator", "getVacuumValveActuator"]:
            try:
                actuator = getattr(head, name)()
                if actuator is not None:
                    return actuator
            except:
                pass
    except:
        pass
    return None


def get_blowoff_actuator(nozzle):
    for name in ["getBlowOffActuator", "getBlowoffActuator"]:
        try:
            actuator = getattr(nozzle, name)()
            if actuator is not None:
                return actuator
        except:
            pass
    try:
        head = nozzle.getHead()
        for name in ["getBlowOffActuator", "getBlowoffActuator"]:
            try:
                actuator = getattr(head, name)()
                if actuator is not None:
                    return actuator
            except:
                pass
    except:
        pass
    return None


def check_part_on(nozzle):
    try:
        if nozzle.isPartOnEnabled(SpiNozzle.PartOnStep.AfterPick):
            if not nozzle.isPartOn():
                raise Exception("Pick failed")
    except AttributeError:
        pass


def check_part_off(nozzle):
    try:
        if nozzle.isPartOffEnabled(SpiNozzle.PartOffStep.AfterPlace):
            if not nozzle.isPartOff():
                raise Exception("Place failed")
    except AttributeError:
        pass


def clear_nozzle_part_state(nozzle, logger=None):
    # Use only supported public nozzle APIs. Do not call setPart.
    place_succeeded = False
    try:
        place = getattr(nozzle, "place")
    except:
        place = None
    try:
        place_callable = callable(place)
    except:
        place_callable = False
    if place_callable:
        try:
            place()
            log_msg(logger, "Nozzle place() completed; internal part state should be clear")
            place_succeeded = True
        except Throwable, e:
            log_msg(logger, "Nozzle place() failed while clearing internal part state: %s" % e)
        except Exception, e:
            log_msg(logger, "Nozzle place() failed while clearing internal part state: %s" % e)

    try:
        get_part = getattr(nozzle, "getPart")
    except:
        get_part = None
    try:
        get_part_callable = callable(get_part)
    except:
        get_part_callable = False
    if get_part_callable:
        try:
            part = get_part()
            log_msg(logger, "Nozzle getPart() after release: %s" % part)
            if part is None:
                return True
        except Throwable, e:
            log_msg(logger, "Nozzle getPart() failed while checking internal part state: %s" % e)
        except Exception, e:
            log_msg(logger, "Nozzle getPart() failed while checking internal part state: %s" % e)

    if place_succeeded:
        return True

    log_msg(logger, "Warning: could not explicitly clear nozzle part state; built-in nozzle calibration may fail")
    return False


def safe_nozzle_pick(nozzle, part, config, logger=None):
    vacuum = get_vacuum_actuator(nozzle)
    set_actuator_state(vacuum, True)
    log_msg(logger, "Vacuum on")
    Thread.sleep(int(config.get("vacuum_ms", 150)))
    try:
        pick = getattr(nozzle, "pick")
    except:
        pick = None
    try:
        pick_callable = callable(pick)
    except:
        pick_callable = False
    if pick_callable:
        try:
            pick(part, None)
            log_msg(logger, "Nozzle pick(part, None) completed")
        except Throwable, e:
            log_msg(logger, "Nozzle pick(part, None) failed after vacuum-on: %s" % e)
        except Exception, e:
            log_msg(logger, "Nozzle pick(part, None) failed after vacuum-on: %s" % e)
    check_part_on(nozzle)
    return True


def safe_nozzle_release(nozzle, config, logger=None):
    vacuum = get_vacuum_actuator(nozzle)
    set_actuator_state(vacuum, False)
    log_msg(logger, "Vacuum off")
    blowoff = get_blowoff_actuator(nozzle)
    use_blowoff = config.get("use_blowoff", config.get("enable_blowoff", True))
    if not bool(use_blowoff):
        log_msg(logger, "Blowoff skipped by config")
    elif blowoff is not None:
        set_actuator_state(blowoff, True)
        log_msg(logger, "Blowoff on")
        Thread.sleep(int(config.get("blowoff_ms", 120)))
        set_actuator_state(blowoff, False)
        log_msg(logger, "Blowoff off")
    clear_nozzle_part_state(nozzle, logger)
    check_part_off(nozzle)
    try:
        get_part = getattr(nozzle, "getPart")
    except:
        get_part = None
    try:
        get_part_callable = callable(get_part)
    except:
        get_part_callable = False
    if get_part_callable:
        part = get_part()
        if part is not None:
            raise Exception("Nozzle still reports part on nozzle after release")
    return True


def capture_top_pipeline(camera, nozzle, part, config, logger=None):
    settings = part.getFiducialVisionSettings()
    if settings is None:
        raise Exception("Pipeline result shape wrong")
    pipeline = settings.getPipeline()
    if pipeline is None:
        raise Exception("Pipeline result shape wrong")

    pipeline.setProperty("camera", camera)
    pipeline.setProperty("nozzle", nozzle)
    pipeline.setProperty("part", part)

    settle_ms = int(config.get("settle_ms", 0))
    if settle_ms > 0:
        Thread.sleep(settle_ms)
    try:
        actual_camera = get_movable_location(camera)
        log_msg(logger, "Top camera capture location: %s" % loc_to_log(actual_camera))
    except:
        pass
    camera.settleAndCapture()
    pipeline.process()

    result = pipeline.getResult(config["result_stage"])
    if result is None:
        raise Exception("Pipeline result shape wrong")
    model = result.getModel()
    items = java_list_to_py(model)
    if len(items) != 2:
        raise Exception("Pipeline result shape wrong")

    circles = [parse_circle(items[0]), parse_circle(items[1])]
    circles.sort(key=lambda c: c["diameter_px"], reverse=True)
    big_circle = circles[0]
    small_circle = circles[1]
    validate_circle_diameters(big_circle, small_circle, config)

    locations = pixel_to_machine_locations(camera, nozzle, big_circle["x_px"], big_circle["y_px"])
    log_msg(logger, "Top vision camera-only center: %s" % loc_to_log(locations["camera_only"]))
    log_msg(logger, "Top vision nozzle-pick target: %s" % loc_to_log(locations["tool_aware"]))
    dx = small_circle["x_px"] - big_circle["x_px"]
    dy = small_circle["y_px"] - big_circle["y_px"]
    angle_deg = normalize_rotation(math.degrees(math.atan2(dy, dx)))

    image_info = {}
    try:
        working = pipeline.getWorkingImage()
        image_info["working_image_width"] = int(working.getWidth())
        image_info["working_image_height"] = int(working.getHeight())
    except:
        pass

    return {
        "center_location": locations["camera_only"],
        "camera_center_location": locations["camera_only"],
        "nozzle_pick_location": locations["tool_aware"],
        "big_circle": big_circle,
        "small_circle": small_circle,
        "artifact_angle_deg": angle_deg,
        "angle_convention": "atan2(small.y_px - big.y_px, small.x_px - big.x_px), normalized to (-180, 180]",
        "pixel_to_machine": locations["debug"],
        "debug": image_info,
    }


def find_artifact_top(camera, nozzle, part, nominal_loc, config, logger=None):
    nominal_loc = nominal_loc.convertToUnits(LengthUnit.Millimeters)
    radius = float(config["search_radius_mm"])
    step = float(config["search_step_mm"])
    if step <= 0.0:
        step = radius if radius > 0.0 else 1.0

    offsets = [(0.0, 0.0)]
    y = -radius
    while y <= radius + 0.000001:
        x = -radius
        while x <= radius + 0.000001:
            if abs(x) > 0.000001 or abs(y) > 0.000001:
                offsets.append((x, y))
            x += step
        y += step

    last_error = None
    for dx, dy in offsets:
        log_msg(logger, "Artifact search offset start: X %.4f Y %.4f" % (dx, dy))
        target = mm_loc(
            nominal_loc.getX() + dx,
            nominal_loc.getY() + dy,
            top_z_for_pose(nominal_loc, config),
            camera.getLocation().convertToUnits(LengthUnit.Millimeters).getRotation(),
        )
        log_msg(logger, "Top camera focal nominal pose: %s" % loc_to_log(nominal_loc))
        log_msg(logger, "Top camera focal part height: %.6f" % float(config["part_height_mm"]))
        log_msg(logger, "Top camera focal target: %s" % loc_to_log(target))
        try:
            move_safe(camera, target)
            actual_camera = get_movable_location(camera)
            log_msg(logger, "Top camera actual before capture: %s" % loc_to_log(actual_camera))
        except Exception, e:
            raise Exception("Motion failure: %s" % e)
        try:
            found = capture_top_pipeline(camera, nozzle, part, config, logger=logger)
        except Exception, e:
            last_error = e
            log_msg(logger, "Artifact search offset failed: X %.4f Y %.4f: %s" % (dx, dy, e))
            continue
        found["search_offset_mm"] = {"x": dx, "y": dy}
        found["camera_location"] = loc_to_dict(target)
        log_msg(logger, "Artifact search offset succeeded: X %.4f Y %.4f angle %.4f" % (
            dx, dy, float(found["artifact_angle_deg"])))
        return found

    raise Exception("Artifact not found after bounded search: %s" % last_error)


def pick_artifact_from_storage(nozzle, part, center_loc, config, angle_deg=0.0, safe_move=True, logger=None):
    storage_pose = pose_from_config(config["storage_location"])
    post_pick_lift_mm = float(config.get("post_pick_lift_mm", 2.0))
    transfer_z_mode = config.get("transfer_z_mode", "relative")
    target = nozzle_target_for_xy(nozzle, center_loc, storage_pose, config, angle_deg)
    rotation_speed = empty_nozzle_rotation_speed(config)
    rotated = rotate_nozzle_lifted_unwrapped(nozzle, target.getRotation(), logger=logger, speed=rotation_speed)
    target = nozzle_target_for_xy(nozzle, center_loc, storage_pose, config, rotated.getRotation())
    before_descend = get_movable_location(nozzle)
    expected_z = float(storage_pose.getZ()) + float(config["part_height_mm"])
    log_msg(logger, "Pick pre-descend nozzle readback/DRO: %s" % loc_to_log(before_descend))
    log_msg(logger, "Pick commanded target: %s" % loc_to_log(target))
    log_msg(logger, "Pick computed storage base Z: %.6f" % float(storage_pose.getZ()))
    log_msg(logger, "Pick part height: %.6f" % float(config["part_height_mm"]))
    log_msg(logger, "Pick expected target Z: storage base Z %.6f + part height %.6f = %.6f" % (
        float(storage_pose.getZ()), float(config["part_height_mm"]), expected_z))
    log_msg(logger, "Pick target Z: storage base z %.4f + part_height_mm %.4f = z %.4f" % (
        float(storage_pose.getZ()), float(config["part_height_mm"]), float(target.getZ())))
    log_msg(logger, "Pick descend target: %s" % loc_to_dict(target))
    try:
        if safe_move:
            move_safe(nozzle, target)
        else:
            move_direct(nozzle, target)
    except Exception, e:
        raise Exception("Motion failure: %s" % e)

    actual = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
    z_error = float(actual.getZ()) - float(target.getZ())
    log_msg(logger, "Pick descend actual readback/DRO: %s" % loc_to_log(actual))
    log_msg(logger, "Pick descend Z readback: commanded %.6f actual %.6f error %.6f" % (
        float(target.getZ()), float(actual.getZ()), z_error))
    if abs(z_error) > float(config.get("z_readback_tolerance_mm", 0.05)):
        raise Exception("Pick descend Z readback error %.6f exceeds tolerance %.6f" % (
            z_error, float(config.get("z_readback_tolerance_mm", 0.05))))

    try:
        safe_nozzle_pick(nozzle, part, config, logger=logger)
    except Exception, e:
        raise Exception("Pick failed: %s" % e)

    try:
        before_lift = get_movable_location(nozzle)
        speed = held_die_speed(config)
        log_msg(logger, "Held-die speed active for post-pick lift: %.6f" % float(speed))
        if str(transfer_z_mode) == "safe_z":
            lifted = lift_nozzle_to_safe_z(nozzle, config, logger=None, speed=speed)
        else:
            lifted = lift_nozzle_relative(nozzle, post_pick_lift_mm, logger=None, speed=speed)
        log_msg(logger, "Post-pick lift: old Z %.6f new Z %.6f" % (
            float(before_lift.getZ()), float(lifted.getZ())))
    except Exception, e:
        raise Exception("Motion failure: %s" % e)

    return {
        "pick_target": loc_to_dict(target),
        "pick_angle_requested_deg": float(angle_deg),
        "pick_angle_commanded_deg": float(target.getRotation()),
        "post_pick_lift_location": loc_to_dict(lifted),
        "transfer_z_mode": str(transfer_z_mode),
    }


def place_artifact_to_cal_area(nozzle, part, target_center_loc, config, angle_deg=0.0, safe_move=True, logger=None):
    cal_pose = pose_from_config(config["calibration_location"])
    post_place_lift_mm = float(config.get("post_place_lift_mm", 2.0))
    transfer_z_mode = config.get("transfer_z_mode", "relative")
    transfer_speed = None
    if nozzle_has_part(nozzle):
        transfer_speed = held_die_speed(config)
    target = nozzle_target_for_xy(nozzle, target_center_loc, cal_pose, config, angle_deg)
    intended_x = float(target.getX())
    intended_y = float(target.getY())
    intended_z = float(target.getZ())
    intended_r = float(target.getRotation())
    intended_place_target = loc_to_dict(target)
    log_msg(logger, "Place target Z: calibration base z %.4f + part_height_mm %.4f = z %.4f" % (
        float(cal_pose.getZ()), float(config["part_height_mm"]), intended_z))
    try:
        before_rotation = get_movable_location(nozzle)
        log_msg(logger, "Before held rotation readback: %s" % loc_to_log(before_rotation))
        log_msg(logger, "Intended place target: %s" % intended_place_target)
        if transfer_speed is not None:
            log_msg(logger, "Held-die speed active for rotation: %.6f" % float(transfer_speed))
        after_rotation = rotate_nozzle_lifted_unwrapped(nozzle, intended_r, logger=logger, speed=transfer_speed)
        actual_rotation = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        rotation_compensation_dx_mm = float(after_rotation.getX()) - float(before_rotation.getX())
        rotation_compensation_dy_mm = float(after_rotation.getY()) - float(before_rotation.getY())
        rotation_compensation_dr_deg = normalize_rotation(
            float(after_rotation.getRotation()) - float(before_rotation.getRotation()))
        rotation_error = normalize_rotation(
            float(actual_rotation.getRotation()) - float(intended_r))
        log_msg(logger, "After held rotation readback: %s" % loc_to_log(after_rotation))
        log_msg(logger, "Rotation-induced delta: X %.6f Y %.6f R %.6f" % (
            rotation_compensation_dx_mm, rotation_compensation_dy_mm, rotation_compensation_dr_deg))
        log_msg(logger, "Held-die rotation readback: commanded %.6f actual %.6f error %.6f" % (
            intended_r, float(actual_rotation.getRotation()), rotation_error))
        if abs(rotation_error) > float(config.get("held_die_rotation_tolerance_deg", 0.5)):
            raise Exception("Held-die rotation readback error %.6f exceeds tolerance %.6f" % (
                rotation_error, float(config.get("held_die_rotation_tolerance_deg", 0.5))))
        if transfer_speed is not None:
            log_msg(logger, "Held-die speed active for lifted XY: %.6f" % float(transfer_speed))
        current_after_rotation = get_movable_location(nozzle)
        dx_to_target = intended_x - float(current_after_rotation.getX())
        dy_to_target = intended_y - float(current_after_rotation.getY())
        xy_move_mag = math.sqrt(dx_to_target * dx_to_target + dy_to_target * dy_to_target)
        skip_xy_threshold = float(config.get("held_die_skip_xy_move_below_mm", 0.010))
        if transfer_speed is not None and xy_move_mag <= skip_xy_threshold:
            log_msg(logger, "Held-die lifted XY move skipped: delta %.6f below threshold %.6f" % (
                xy_move_mag, skip_xy_threshold))
        else:
            move_nozzle_xy_lifted(nozzle, intended_x, intended_y, logger=logger, speed=transfer_speed)
        before_lower = get_movable_location(nozzle)
        lower_target = mm_loc(
            before_lower.getX(),
            before_lower.getY(),
            intended_z,
            before_lower.getRotation(),
        )
        expected_z = float(cal_pose.getZ()) + float(config["part_height_mm"])
        log_msg(logger, "Place pre-lower nozzle readback/DRO: %s" % loc_to_log(before_lower))
        log_msg(logger, "After lifted XY handling readback: %s" % loc_to_log(before_lower))
        log_msg(logger, "Lower target readback/command: readback %s command %s" % (
            loc_to_log(before_lower), loc_to_log(lower_target)))
        log_msg(logger, "Place commanded lower target: %s" % loc_to_log(lower_target))
        log_msg(logger, "Place computed calibration base Z: %.6f" % float(cal_pose.getZ()))
        log_msg(logger, "Place part height: %.6f" % float(config["part_height_mm"]))
        log_msg(logger, "Place expected target Z: calibration base Z %.6f + part height %.6f = %.6f" % (
            float(cal_pose.getZ()), float(config["part_height_mm"]), expected_z))
        log_msg(logger, "Place lower target: %s" % loc_to_dict(lower_target))
        if transfer_speed is not None:
            log_msg(logger, "Held-die speed active for place lower: %.6f" % float(transfer_speed))
        lowered = lower_nozzle_to(nozzle, intended_z, logger=logger, speed=transfer_speed)
        actual = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        z_error = float(actual.getZ()) - intended_z
        log_msg(logger, "Place lower actual readback/DRO: %s" % loc_to_log(actual))
        log_msg(logger, "Place lower Z readback: commanded %.6f actual %.6f error %.6f" % (
            intended_z, float(actual.getZ()), z_error))
        if abs(z_error) > float(config.get("z_readback_tolerance_mm", 0.05)):
            raise Exception("Place lower Z readback error %.6f exceeds tolerance %.6f" % (
                z_error, float(config.get("z_readback_tolerance_mm", 0.05))))
    except Exception, e:
        raise Exception("Motion failure: %s" % e)

    try:
        safe_nozzle_release(nozzle, config, logger=logger)
    except Exception, e:
        raise Exception("Place failed: %s" % e)

    try:
        before_lift = get_movable_location(nozzle)
        speed = normal_motion_speed(config)
        if str(transfer_z_mode) == "safe_z":
            lifted = lift_nozzle_to_safe_z(nozzle, config, logger=None, speed=speed)
        else:
            lifted = lift_nozzle_relative(nozzle, post_place_lift_mm, logger=None, speed=speed)
        log_msg(logger, "Post-place lift: old Z %.6f new Z %.6f" % (
            float(before_lift.getZ()), float(lifted.getZ())))
    except Exception, e:
        raise Exception("Motion failure: %s" % e)

    return {
        "place_target": loc_to_dict(lower_target),
        "place_angle_requested_deg": float(angle_deg),
        "place_angle_commanded_deg": intended_r,
        "rotation_readback_before": loc_to_dict(before_rotation),
        "rotation_readback_after": loc_to_dict(after_rotation),
        "rotation_compensation_dx_mm": rotation_compensation_dx_mm,
        "rotation_compensation_dy_mm": rotation_compensation_dy_mm,
        "rotation_compensation_dr_deg": rotation_compensation_dr_deg,
        "intended_place_target": intended_place_target,
        "actual_place_xy_before_lower": loc_to_dict(before_lower),
        "post_place_lift_location": loc_to_dict(lifted),
        "transfer_z_mode": str(transfer_z_mode),
    }


def return_artifact_to_known_pose(nozzle, camera, part, current_loc, config):
    storage_pose = pose_from_config(config["storage_location"])
    current_rot = current_loc.convertToUnits(LengthUnit.Millimeters).getRotation()
    pick_artifact_from_storage(nozzle, part, current_loc, {
        "storage_location": config["calibration_location"],
        "part_height_mm": config["part_height_mm"],
        "vacuum_ms": config.get("vacuum_ms", 150),
        "post_pick_lift_mm": config.get("post_pick_lift_mm", 2.0),
        "transfer_z_mode": "safe_z",
        "safe_travel_z": safe_travel_z_for_config(config),
    }, current_rot)
    return place_artifact_to_cal_area(nozzle, part, storage_pose, {
        "calibration_location": config["storage_location"],
        "part_height_mm": config["part_height_mm"],
        "blowoff_ms": config.get("blowoff_ms", 120),
        "post_place_lift_mm": config.get("post_place_lift_mm", 2.0),
        "transfer_z_mode": "safe_z",
        "safe_travel_z": safe_travel_z_for_config(config),
    }, storage_pose.getRotation())


def result_failure(message):
    return {
        "ok": False,
        "message": str(message),
        "old_head_offsets_mm": None,
        "proposed_head_offsets_mm": None,
        "delta_xy_mm": None,
        "angles_deg": [],
        "per_angle_records": [],
        "final_artifact_pose": None,
    }


def solve_head_offset(config, nozzle=None, camera=None, machine_obj=None, part=None, logger=None):
    per_angle_records = []
    angles = [float(a) for a in config.get("angles_deg", DEFAULT_ANGLES_DEG)]

    try:
        log_msg(logger, "Head offset solver start")
        config.setdefault("pairwise_180_direction", 1.0)
        config.setdefault("held_die_motion_speed", 0.60)
        config.setdefault("empty_nozzle_rotation_speed", 0.20)
        if len(angles) == 0:
            raise Exception("No pairwise angles configured")
        if machine_obj is None:
            machine_obj = Configuration.get().getMachine()
        if nozzle is None:
            nozzle = get_default_nozzle(machine_obj)
        try:
            log_msg(logger, "Selected nozzle name: %s" % nozzle.getName())
        except:
            log_msg(logger, "Selected nozzle name: %s" % nozzle)
        if camera is None:
            camera = get_default_camera(machine_obj)
        try:
            log_msg(logger, "Top camera selected: %s" % camera.getName())
        except:
            log_msg(logger, "Top camera selected: %s" % camera)
        part = get_part(config, part)
        config = config_with_runtime_part_height(config, part, logger=logger)
        try:
            log_msg(logger, "Selected part ID: %s" % part.getId())
        except:
            log_msg(logger, "Selected part ID: %s" % part)

        storage_pose = pose_from_config(config["storage_location"])
        calibration_pose = pose_from_config(config["calibration_location"])
        storage_transfer_config = config_with_transfer_z_mode(config, "safe_z")
        pairwise_transfer_config = config_with_transfer_z_mode(config, "relative")

        log_msg(logger, "Storage artifact search start")
        storage_found = find_artifact_top(camera, nozzle, part, storage_pose, config, logger=logger)
        log_msg(logger, "Storage artifact search result: center %s angle %.4f" % (
            loc_to_dict(storage_found["center_location"]), float(storage_found["artifact_angle_deg"])))
        storage_pick_target = detection_pick_location(storage_found)
        log_msg(logger, "Using top vision nozzle-pick target for pick: %s" % loc_to_log(storage_pick_target))
        log_msg(logger, "Using top vision camera-only center for measurement/math: %s" % loc_to_log(
            storage_found["center_location"]))
        pick_artifact_from_storage(nozzle, part, storage_pick_target, storage_transfer_config,
            storage_pose.getRotation(), logger=logger)
        place_artifact_to_cal_area(nozzle, part, calibration_pose, storage_transfer_config,
            calibration_pose.getRotation(), logger=logger)

        nominal_pairwise_center = calibration_pose
        valid_corrections = []
        rejected_pair_records = []
        max_single_raw_delta = 0.0
        max_single_solved_correction = 0.0
        sum_raw_delta_mag_sq = 0.0
        sum_solved_correction_mag_sq = 0.0
        final_artifact_pose = None
        pairwise_pass_count = int(config.get("pairwise_pass_count", 1))
        if pairwise_pass_count < 1:
            pairwise_pass_count = 1
        reject_bad_pairs = bool(config.get("pairwise_reject_bad_pairs", True))
        recenter_between_pairs = bool(config.get("pairwise_recenter_between_pairs", False))
        current_pose = find_artifact_top(camera, nozzle, part, calibration_pose, config, logger=logger)
        current_center = current_pose["center_location"].convertToUnits(LengthUnit.Millimeters)
        current_artifact_angle_deg = float(current_pose["artifact_angle_deg"])
        log_msg(logger, "Pairwise initial measured pose: center %s angle %.6f" % (
            loc_to_dict(current_center), current_artifact_angle_deg))

        for pass_index in range(pairwise_pass_count):
            for angle in angles:
                log_msg(logger, "Pairwise angle start: pass %d angle %.4f" % (pass_index, angle))
                old_center = current_center
                old_pick_target = detection_pick_location(current_pose)
                old_artifact_angle_deg = current_artifact_angle_deg
                log_msg(logger, "Pairwise using carried pose: center %s angle %.6f" % (
                    loc_to_dict(old_center), old_artifact_angle_deg))
                log_msg(logger, "Using top vision nozzle-pick target for pick: %s" % loc_to_log(old_pick_target))
                log_msg(logger, "Using top vision camera-only center for measurement/math: %s" % loc_to_log(old_center))
                pick_angle = float(angle)
                place_angle = unwrapped_plus_180(pick_angle, config.get("pairwise_180_direction", 1.0))
                pick_result = pick_artifact_from_storage(nozzle, part, old_pick_target, {
                    "storage_location": config["calibration_location"],
                    "part_height_mm": config["part_height_mm"],
                    "vacuum_ms": config.get("vacuum_ms", 150),
                    "post_pick_lift_mm": config.get("post_pick_lift_mm", 2.0),
                    "transfer_z_mode": "relative",
                    "safe_travel_z": safe_travel_z_for_config(config),
                    "held_die_motion_speed": config.get("held_die_motion_speed", 0.60),
                    "empty_nozzle_rotation_speed": config.get("empty_nozzle_rotation_speed", 0.20),
                }, pick_angle, logger=logger)
                # Pairwise correction must place back to old_center. Nominal recentering is housekeeping only and must not enter dx/dy correction math.
                log_msg(logger, "Pairwise measurement place target is old_center, not nominal center")
                place_result = place_artifact_to_cal_area(
                    nozzle, part, old_center, pairwise_transfer_config, place_angle, False, logger=logger)

                measured = find_artifact_top(camera, nozzle, part,
                    xy_with_base_pose(old_center, calibration_pose), config, logger=logger)
                new_center = measured["center_location"].convertToUnits(LengthUnit.Millimeters)
                new_artifact_angle_deg = float(measured["artifact_angle_deg"])
                final_artifact_pose = loc_to_dict(new_center)
                dx = new_center.getX() - old_center.getX()
                dy = new_center.getY() - old_center.getY()
                raw_delta_mag = math.sqrt(dx * dx + dy * dy)
                expected_delta_deg = 180.0
                if float(config.get("pairwise_180_direction", 1.0)) < 0.0:
                    expected_delta_deg = -180.0
                actual_delta_deg = normalize_rotation_near(
                    new_artifact_angle_deg - old_artifact_angle_deg,
                    expected_delta_deg
                )
                commanded_delta_deg = (
                    place_result["place_angle_commanded_deg"] - pick_result["pick_angle_commanded_deg"])
                corr_x = None
                corr_y = None
                solved_correction_mag = None
                reject_reason = None

                log_msg(logger, "Pairwise new measured center: %s" % loc_to_dict(new_center))
                log_msg(logger, "Pairwise new measured artifact angle: %.6f" % new_artifact_angle_deg)
                log_msg(logger, "Pairwise rotation delta: commanded %.6f actual %.6f" % (
                    commanded_delta_deg, actual_delta_deg))
                log_msg(logger, "Per-angle raw center delta: pass %d angle %.4f X %.6f Y %.6f mag %.6f" % (
                    pass_index, angle, dx, dy, raw_delta_mag))

                try:
                    abs_actual = abs(actual_delta_deg)
                    if "pairwise_min_actual_rotation_abs_deg" in config:
                        min_actual = float(config["pairwise_min_actual_rotation_abs_deg"])
                        if abs_actual < min_actual:
                            raise Exception("Pairwise actual rotation %.6f below %.6f" % (
                                abs_actual, min_actual))
                    if "pairwise_max_actual_rotation_abs_deg" in config:
                        max_actual = float(config["pairwise_max_actual_rotation_abs_deg"])
                        if abs_actual > max_actual:
                            raise Exception("Pairwise actual rotation %.6f exceeds %.6f" % (
                                abs_actual, max_actual))
                    if "pairwise_max_actual_rotation_error_deg" in config:
                        max_actual_error = float(config["pairwise_max_actual_rotation_error_deg"])
                        actual_error = abs(abs_actual - 180.0)
                        if actual_error > max_actual_error:
                            raise Exception("Pairwise actual rotation error %.6f exceeds %.6f" % (
                                actual_error, max_actual_error))
                    if "pairwise_max_step_center_walk_mm" in config:
                        max_step_walk = float(config["pairwise_max_step_center_walk_mm"])
                        if raw_delta_mag > max_step_walk:
                            raise Exception("Pairwise center walk %.6f exceeds %.6f" % (
                                raw_delta_mag, max_step_walk))
                    if "pairwise_max_single_raw_delta_mm" in config:
                        max_single_raw = float(config["pairwise_max_single_raw_delta_mm"])
                        if raw_delta_mag > max_single_raw:
                            raise Exception("Pairwise raw delta %.6f exceeds %.6f" % (
                                raw_delta_mag, max_single_raw))
                    elif "pairwise_max_single_delta_mm" in config:
                        max_single = float(config["pairwise_max_single_delta_mm"])
                        if raw_delta_mag > max_single:
                            raise Exception("Pairwise raw delta %.6f exceeds legacy %.6f" % (
                                raw_delta_mag, max_single))

                    corr_x, corr_y = solve_rotation_center_correction(dx, dy, actual_delta_deg)
                    solved_correction_mag = math.sqrt(corr_x * corr_x + corr_y * corr_y)
                    log_msg(logger, "Per-angle solved correction: pass %d angle %.4f X %.6f Y %.6f mag %.6f" % (
                        pass_index, angle, corr_x, corr_y, solved_correction_mag))

                    if "pairwise_max_single_solved_correction_mm" in config:
                        max_single_corr = float(config["pairwise_max_single_solved_correction_mm"])
                        if solved_correction_mag > max_single_corr:
                            raise Exception("Pairwise solved correction %.6f exceeds %.6f" % (
                                solved_correction_mag, max_single_corr))
                except Exception, e:
                    reject_reason = str(e)
                    if not reject_bad_pairs:
                        raise Exception("%s at pass %d angle %.4f" % (reject_reason, pass_index, angle))

                pair_record = {
                    "pass_index": pass_index,
                    "angle_deg": angle,
                    "place_angle_deg": place_angle,
                    "pick_angle_requested_deg": pick_angle,
                    "pick_angle_commanded_deg": pick_result["pick_angle_commanded_deg"],
                    "place_angle_requested_deg": place_angle,
                    "place_angle_commanded_deg": place_result["place_angle_commanded_deg"],
                    "old_artifact_angle_deg": old_artifact_angle_deg,
                    "new_artifact_angle_deg": new_artifact_angle_deg,
                    "commanded_rotation_delta_deg": commanded_delta_deg,
                    "actual_artifact_rotation_delta_deg": actual_delta_deg,
                    "rotation_delta_commanded_deg": commanded_delta_deg,
                    "rotation_compensation_dx_mm": place_result["rotation_compensation_dx_mm"],
                    "rotation_compensation_dy_mm": place_result["rotation_compensation_dy_mm"],
                    "rotation_compensation_dr_deg": place_result["rotation_compensation_dr_deg"],
                    "rotation_readback_before": place_result["rotation_readback_before"],
                    "rotation_readback_after": place_result["rotation_readback_after"],
                    "intended_place_target": place_result["intended_place_target"],
                    "actual_place_xy_before_lower": place_result["actual_place_xy_before_lower"],
                    "old_center_mm": loc_to_dict(old_center),
                    "old_nozzle_pick_target_mm": loc_to_dict(old_pick_target),
                    "new_center_mm": loc_to_dict(new_center),
                    "nominal_pairwise_center_mm": loc_to_dict(nominal_pairwise_center),
                    "delta_mm": {"x": dx, "y": dy},
                    "delta_mag_mm": raw_delta_mag,
                    "raw_delta_mm": {"x": dx, "y": dy},
                    "raw_delta_mag_mm": raw_delta_mag,
                    "solved_correction_mm": {"x": corr_x, "y": corr_y},
                    "solved_correction_mag_mm": solved_correction_mag,
                    "accepted": reject_reason is None,
                    "rejected": reject_reason is not None,
                    "reject_reason": reject_reason,
                    "recenter_attempted": False,
                    "measurement": {
                        "big_circle": measured["big_circle"],
                        "small_circle": measured["small_circle"],
                        "artifact_angle_deg": measured["artifact_angle_deg"],
                        "angle_convention": measured["angle_convention"],
                        "pixel_to_machine": measured["pixel_to_machine"],
                        "search_offset_mm": measured.get("search_offset_mm"),
                    },
                }

                if reject_reason is None:
                    if raw_delta_mag > max_single_raw_delta:
                        max_single_raw_delta = raw_delta_mag
                    if solved_correction_mag > max_single_solved_correction:
                        max_single_solved_correction = solved_correction_mag
                    sum_raw_delta_mag_sq += raw_delta_mag * raw_delta_mag
                    sum_solved_correction_mag_sq += solved_correction_mag * solved_correction_mag
                    valid_corrections.append({
                        "pass_index": pass_index,
                        "angle_deg": angle,
                        "corr_x": corr_x,
                        "corr_y": corr_y,
                        "corr_mag": solved_correction_mag,
                        "raw_delta_mag": raw_delta_mag,
                    })
                    log_msg(logger, "Pairwise pair accepted: pass %d angle %.4f" % (pass_index, angle))
                else:
                    rejected_pair_records.append(pair_record)
                    log_msg(logger, "Pairwise pair rejected: pass %d angle %.4f: %s" % (
                        pass_index, angle, reject_reason))

                current_pose = measured
                current_center = new_center
                current_artifact_angle_deg = new_artifact_angle_deg

                if recenter_between_pairs:
                    nominal_mm = nominal_pairwise_center.convertToUnits(LengthUnit.Millimeters)
                    recenter_dx = float(new_center.getX()) - float(nominal_mm.getX())
                    recenter_dy = float(new_center.getY()) - float(nominal_mm.getY())
                    recenter_mag = math.sqrt(recenter_dx * recenter_dx + recenter_dy * recenter_dy)
                    recenter_threshold = float(config.get("pairwise_recenter_threshold_mm", 0.050))
                    pair_record["recenter_attempted"] = recenter_mag > recenter_threshold
                    pair_record["recenter_delta_mm"] = {"x": recenter_dx, "y": recenter_dy}
                    pair_record["recenter_delta_mag_mm"] = recenter_mag
                    log_msg(logger, "Pairwise recenter check: delta %.6f threshold %.6f" % (
                        recenter_mag, recenter_threshold))
                    if recenter_mag > recenter_threshold:
                        log_msg(logger, "Pairwise housekeeping recenter to nominal center")
                        recenter_pick_angle = float(config.get(
                            "pairwise_recenter_pick_angle_deg", new_artifact_angle_deg))
                        recenter_pick_target = detection_pick_location(measured)
                        log_msg(logger, "Using top vision nozzle-pick target for pick: %s" % loc_to_log(recenter_pick_target))
                        log_msg(logger, "Using top vision camera-only center for measurement/math: %s" % loc_to_log(new_center))
                        pick_artifact_from_storage(nozzle, part, recenter_pick_target, {
                            "storage_location": config["calibration_location"],
                            "part_height_mm": config["part_height_mm"],
                            "vacuum_ms": config.get("vacuum_ms", 150),
                            "post_pick_lift_mm": config.get("post_pick_lift_mm", 2.0),
                            "transfer_z_mode": "relative",
                            "safe_travel_z": safe_travel_z_for_config(config),
                            "held_die_motion_speed": config.get("held_die_motion_speed", 0.60),
                            "empty_nozzle_rotation_speed": config.get("empty_nozzle_rotation_speed", 0.20),
                        }, recenter_pick_angle, logger=logger)
                        place_artifact_to_cal_area(nozzle, part, nominal_pairwise_center,
                            pairwise_transfer_config, new_artifact_angle_deg, False, logger=logger)
                        recentered = find_artifact_top(camera, nozzle, part, calibration_pose, config, logger=logger)
                        pair_record["recenter_result_center_mm"] = loc_to_dict(recentered["center_location"])
                        pair_record["recenter_result_artifact_angle_deg"] = float(recentered["artifact_angle_deg"])
                        final_artifact_pose = loc_to_dict(recentered["center_location"])
                        current_pose = recentered
                        current_center = recentered["center_location"].convertToUnits(LengthUnit.Millimeters)
                        current_artifact_angle_deg = float(recentered["artifact_angle_deg"])
                        log_msg(logger, "Pairwise recenter result: center %s angle %.6f" % (
                            loc_to_dict(current_center), current_artifact_angle_deg))
                else:
                    log_msg(logger, "Pairwise recenter skipped: disabled")

                per_angle_records.append(pair_record)
                log_msg(logger, "Pairwise angle end: pass %d angle %.4f" % (pass_index, angle))

        valid_pair_count = len(valid_corrections)
        rejected_pair_count = len(rejected_pair_records)
        min_valid_pairs = int(config.get("pairwise_min_valid_pairs", 4))
        if valid_pair_count < min_valid_pairs:
            raise Exception("Pairwise valid corrections %d below required %d" % (
                valid_pair_count, min_valid_pairs))

        count = float(valid_pair_count)
        pairwise_rms_raw_delta = math.sqrt(sum_raw_delta_mag_sq / count)
        pairwise_rms_solved_correction = math.sqrt(sum_solved_correction_mag_sq / count)
        estimate = compute_correction_estimate(valid_corrections, config, logger)
        sign = float(config.get("head_offset_sign", 1.0))
        delta_x = sign * estimate["corr_x"]
        delta_y = sign * estimate["corr_y"]
        correction_mag = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        log_msg(logger, "Pairwise quality: valid %d rejected %d max raw delta %.6f rms raw delta %.6f max solved correction %.6f rms solved correction %.6f final correction %.6f" % (
            valid_pair_count, rejected_pair_count, max_single_raw_delta, pairwise_rms_raw_delta,
            max_single_solved_correction, pairwise_rms_solved_correction, correction_mag))
        log_msg(logger, "Legacy pairwise_max_single_delta_mm reports raw delta; legacy pairwise_rms_delta_mm reports solved correction RMS")
        if "pairwise_max_rms_solved_correction_mm" in config:
            max_rms_corr = float(config["pairwise_max_rms_solved_correction_mm"])
            if pairwise_rms_solved_correction > max_rms_corr:
                raise Exception("Pairwise solved correction RMS %.6f exceeds %.6f" % (
                    pairwise_rms_solved_correction, max_rms_corr))
        elif "pairwise_max_rms_delta_mm" in config:
            max_rms = float(config["pairwise_max_rms_delta_mm"])
            if pairwise_rms_solved_correction > max_rms:
                raise Exception("Pairwise solved correction RMS %.6f exceeds legacy %.6f" % (
                    pairwise_rms_solved_correction, max_rms))
        if "head_offset_max_apply_delta_mm" in config:
            max_apply = float(config["head_offset_max_apply_delta_mm"])
            if correction_mag > max_apply:
                raise Exception("Head offset apply delta %.6f exceeds %.6f" % (
                    correction_mag, max_apply))
        if "head_offset_min_apply_delta_mm" in config:
            min_apply = float(config["head_offset_min_apply_delta_mm"])
            if correction_mag < min_apply:
                log_msg(logger, "Head offset correction %.6f below apply threshold %.6f; applying zero correction" % (
                    correction_mag, min_apply))
                delta_x = 0.0
                delta_y = 0.0
                correction_mag = 0.0

        old_offsets = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
        proposed = mm_loc(
            old_offsets.getX() + delta_x,
            old_offsets.getY() + delta_y,
            old_offsets.getZ(),
            old_offsets.getRotation(),
        )
        log_msg(logger, "Final proposed head offset: %s" % loc_to_dict(proposed))

        return {
            "ok": True,
            "message": "ok",
            "old_head_offsets_mm": loc_to_dict(old_offsets),
            "proposed_head_offsets_mm": loc_to_dict(proposed),
            "delta_xy_mm": {"x": delta_x, "y": delta_y},
            "pairwise_valid_pair_count": valid_pair_count,
            "pairwise_rejected_pair_count": rejected_pair_count,
            "pairwise_estimator": estimate["estimator"],
            "pairwise_trim_worst_count_used": estimate["trim_worst_count_used"],
            "pairwise_max_single_raw_delta_mm": max_single_raw_delta,
            "pairwise_rms_raw_delta_mm": pairwise_rms_raw_delta,
            "pairwise_max_single_solved_correction_mm": max_single_solved_correction,
            "pairwise_rms_solved_correction_mm": pairwise_rms_solved_correction,
            "pairwise_max_single_delta_mm": max_single_raw_delta,
            "pairwise_rms_delta_mm": pairwise_rms_solved_correction,
            "final_correction_mag_mm": correction_mag,
            "angles_deg": angles,
            "per_angle_records": per_angle_records,
            "rejected_pair_records": rejected_pair_records,
            "final_artifact_pose": final_artifact_pose,
        }

    except Exception, e:
        log_msg(logger, "Head offset solver exception: %s" % e)
        return result_failure(e)
