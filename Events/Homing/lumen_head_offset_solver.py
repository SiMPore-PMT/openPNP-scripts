# lumen_head_offset_solver.py

import math

from jarray import array
from java.lang import Boolean, Class, Object, System, Thread, Throwable
from org.openpnp.model import Configuration, Location, LengthUnit
from org.openpnp.spi import Nozzle as SpiNozzle
from org.openpnp.spi.Locatable import LocationOption
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import MovableUtils, VisionUtils


DEFAULT_ANGLES_DEG = [-150.0, -90.0, -30.0, 30.0, 90.0, 150.0]
_RUNTIME_PICK_COUNT = 0


def log_msg(logger, msg):
    if logger:
        logger(str(msg))
    else:
        print str(msg)


def publish_pipeline_debug_image(config, pipeline, stage_name, logger=None):
    """Publish a just-processed stage while its pipeline image is still valid."""
    callback = config.get("_pipeline_debug_callback")
    if callback is None:
        return
    try:
        callback(pipeline, stage_name)
    except Exception, e:
        log_msg(logger, "Unable to publish pipeline debug stage '%s': %s" % (
            stage_name, e))


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
        movable.moveTo(target)
    wait_still(movable)


def move_no_comp(nozzle, target, speed):
    current_nozzle = nozzle.getLocation().convertToUnits(
        LengthUnit.Millimeters)
    target = target.convertToUnits(LengthUnit.Millimeters)

    head_loc = nozzle.toHeadLocation(
        target,
        current_nozzle,
        LocationOption.SuppressDynamicCompensation
    )

    head = nozzle.getHead()
    move_speed = float(speed)
    head.moveTo(
        nozzle,
        head_loc,
        move_speed
    )
    wait_still(nozzle)


def pairwise_head_move(nozzle, target_head, speed, changed_axes,
        logger=None):
    current_nozzle = nozzle.getLocation().convertToUnits(
        LengthUnit.Millimeters)
    current_head = nozzle.toHeadLocation(current_nozzle)
    current_raw_axes = nozzle.toRaw(current_head)
    target_raw_axes = nozzle.toRaw(target_head)

    head = nozzle.getHead()
    move_speed = float(speed)
    log_msg(logger,
        "Pairwise phase %s: current head %s target head %s "
        "OpenPnP speed scalar %.6f" %
        (changed_axes,
         loc_to_log(current_head),
         loc_to_log(target_head),
         move_speed))
    log_msg(logger,
        "Pairwise phase %s raw axes: current %s target %s" %
        (changed_axes, str(current_raw_axes), str(target_raw_axes)))

    head.moveTo(nozzle, target_head, move_speed)
    wait_still(nozzle)
    log_msg(logger, "Pairwise phase %s complete: stillstand" % changed_axes)


def pairwise_rotate_only_no_comp(nozzle, target_rotation, speed,
        logger=None):
    current_nozzle = nozzle.getLocation().convertToUnits(
        LengthUnit.Millimeters)
    current_head = nozzle.toHeadLocation(current_nozzle)

    logical_rotation_target = mm_loc(
        current_nozzle.getX(),
        current_nozzle.getY(),
        current_nozzle.getZ(),
        float(target_rotation),
    )
    converted_target = nozzle.toHeadLocation(
        logical_rotation_target,
        current_nozzle,
        LocationOption.SuppressDynamicCompensation
    )

    raw_rotation_target = mm_loc(
        current_head.getX(),
        current_head.getY(),
        current_head.getZ(),
        converted_target.getRotation(),
    )

    pairwise_head_move(
        nozzle, raw_rotation_target, speed, "rotation-only", logger=logger)


def pairwise_move_xy_only_no_comp(nozzle, target, speed, logger=None):
    current_nozzle = nozzle.getLocation().convertToUnits(
        LengthUnit.Millimeters)
    current_head = nozzle.toHeadLocation(current_nozzle)
    target = target.convertToUnits(LengthUnit.Millimeters)

    logical_xy_target = mm_loc(
        target.getX(),
        target.getY(),
        current_nozzle.getZ(),
        current_nozzle.getRotation(),
    )
    converted_target = nozzle.toHeadLocation(
        logical_xy_target,
        current_nozzle,
        LocationOption.SuppressDynamicCompensation
    )
    raw_xy_target = mm_loc(
        converted_target.getX(),
        converted_target.getY(),
        current_head.getZ(),
        current_head.getRotation(),
    )

    pairwise_head_move(
        nozzle, raw_xy_target, speed, "XY-only", logger=logger)


def pairwise_move_z_only_no_comp(nozzle, target_z, speed, logger=None):
    current_nozzle = nozzle.getLocation().convertToUnits(
        LengthUnit.Millimeters)
    current_head = nozzle.toHeadLocation(current_nozzle)

    logical_z_target = mm_loc(
        current_nozzle.getX(),
        current_nozzle.getY(),
        float(target_z),
        current_nozzle.getRotation(),
    )
    converted_target = nozzle.toHeadLocation(
        logical_z_target,
        current_nozzle,
        LocationOption.SuppressDynamicCompensation
    )
    raw_z_target = mm_loc(
        current_head.getX(),
        current_head.getY(),
        converted_target.getZ(),
        current_head.getRotation(),
    )

    pairwise_head_move(
        nozzle, raw_z_target, speed, "Z-only", logger=logger)


def pairwise_move_to_safe_z(nozzle, config, speed, logger=None):
    effective_safe_z = length_to_mm(nozzle.getEffectiveSafeZ())
    if effective_safe_z is None:
        effective_safe_z = safe_travel_z_for_config(config)
    log_msg(logger, "Pairwise safe-Z target: %.6f" % effective_safe_z)
    pairwise_move_z_only_no_comp(
        nozzle, effective_safe_z, speed, logger=logger)
    return get_movable_location(nozzle)


def move_pairwise_180_at_safe_z(movable, target, logger=None, speed=None,
        z_speed=None):
    target = target.convertToUnits(LengthUnit.Millimeters)
    if z_speed is None:
        z_speed = speed
    log_msg(logger, "Pairwise placement phase: rotation only")
    log_msg(logger, "Pairwise rotation pure A move start")
    pairwise_rotate_only_no_comp(
        movable, target.getRotation(), speed, logger=logger)
    log_msg(logger, "Pairwise placement phase: XY only")
    pairwise_move_xy_only_no_comp(
        movable, target, speed, logger=logger)
    log_msg(logger, "Pairwise placement phase: Z only down")
    pairwise_move_z_only_no_comp(
        movable, target.getZ(), z_speed, logger=logger)
# def move_pairwise_180_at_safe_z(movable, target, logger=None):
#     """Keep the pairwise 180-degree rotation separate from tiny XY corrections."""
#     target = target.convertToUnits(LengthUnit.Millimeters)

#     # Match MovableUtils safe-Z handling by lifting the complete head first. The
#     # actual readback retains OpenPnP's effective safe Z for the following moves.
#     movable.getHead().moveToSafeZ()
#     wait_still(movable)
#     safe_location = get_movable_location(movable)

#     rotation_target = mm_loc(
#         safe_location.getX(),
#         safe_location.getY(),
#         safe_location.getZ(),
#         target.getRotation(),
#     )
#     log_msg(logger, "Pairwise 180 safe-Z rotation target: %s" % loc_to_log(rotation_target))
#     move_direct(movable, rotation_target)

#     xy_target = mm_loc(
#         target.getX(),
#         target.getY(),
#         safe_location.getZ(),
#         target.getRotation(),
#     )
#     move_direct(movable, xy_target)
#     move_direct(movable, target)


def resolve_motion_speed(config, category, default_value, logger=None):
    # OpenPnP moveTo(location, speed) uses a speed scalar; keep all script aliases
    # mapped in one place so pairwise and held-die moves cannot diverge silently.
    category = str(category)
    key_sets = {
        "normal": ["calibration_motion_speed", "normal_motion_speed"],
        "calibration": ["calibration_motion_speed", "normal_motion_speed"],
        "held_die": ["pairwise_motion_speed", "held_die_motion_speed"],
        "held_die_z": ["held_die_z_speed", "pairwise_motion_speed", "held_die_motion_speed"],
        "pick_z_extra": ["pick_z_extra_speed", "held_die_z_speed", "pairwise_motion_speed", "held_die_motion_speed"],
        "safe_travel": ["safe_travel_speed", "calibration_motion_speed", "normal_motion_speed"],
    }
    keys = key_sets.get(category, [])
    source = "default"
    raw_value = default_value
    for key in keys:
        if key in config:
            source = key
            raw_value = config[key]
            break
    speed = float(raw_value)
    if bool(config.get("speed_clamp_enabled", False)):
        if "speed_min" in config and speed < float(config["speed_min"]):
            log_msg(logger, "Motion speed clamp: category %s %.6f -> min %.6f" % (
                category, speed, float(config["speed_min"])))
            speed = float(config["speed_min"])
        if "speed_max" in config and speed > float(config["speed_max"]):
            log_msg(logger, "Motion speed clamp: category %s %.6f -> max %.6f" % (
                category, speed, float(config["speed_max"])))
            speed = float(config["speed_max"])
    return {
        "speed": speed,
        "category": category,
        "source": source,
        "raw_value": raw_value,
    }


def log_resolved_motion_speed(config, category, default_value, logger=None):
    resolved = resolve_motion_speed(config, category, default_value, logger=logger)
    log_msg(logger, "Motion speed resolved: category %s OpenPnP speed scalar %.6f source %s raw %s" % (
        resolved["category"], resolved["speed"], resolved["source"], str(resolved["raw_value"])))
    return resolved["speed"]


def log_motion_speed_config(config, logger=None):
    for category, default_value in [
        ("normal", 1.0),
        ("calibration", 1.0),
        ("held_die", 0.60),
        ("held_die_z", 0.60),
        ("pick_z_extra", 0.60),
        ("safe_travel", 1.0),
    ]:
        log_resolved_motion_speed(config, category, default_value, logger=logger)


def log_move_speed(logger, config, category, speed, target, nozzle=None):
    if speed is None:
        return
    resolved = resolve_motion_speed(config, category, speed, logger=logger)
    has_part = False
    if nozzle is not None:
        has_part = nozzle_has_part(nozzle)
    log_msg(logger, "Motion move: category %s OpenPnP speed scalar %.6f source %s raw %s target %s nozzle_has_part %s" % (
        category, float(speed), resolved["source"], str(resolved["raw_value"]),
        loc_to_log(target), str(has_part)))


def held_die_speed(config):
    return resolve_motion_speed(config, "held_die", 0.60)["speed"]


def held_die_z_speed(config):
    return resolve_motion_speed(
        config, "held_die_z", held_die_speed(config))["speed"]


def safe_travel_speed(config):
    return resolve_motion_speed(
        config, "safe_travel", normal_motion_speed(config))["speed"]


def normal_motion_speed(config):
    return resolve_motion_speed(config, "normal", 1.0)["speed"]


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


def config_with_transfer_speed(config, transfer_z_mode, speed):
    out = config_with_transfer_z_mode(config, transfer_z_mode)
    out["calibration_motion_speed"] = resolve_motion_speed(config, "normal", 1.0)["speed"]
    out["normal_motion_speed"] = resolve_motion_speed(config, "normal", 1.0)["speed"]
    out["pairwise_motion_speed"] = float(speed)
    out["held_die_motion_speed"] = float(speed)
    return out


def copy_pick_z_extra_config(out, config):
    for key in [
        "Z_EXTRA",
        "z_extra",
        "pick_z_extra_mm",
        "pick_z_extra_max_mm",
        "pick_z_extra_readback_tolerance_mm",
        "pick_z_extra_speed",
        "held_die_z_speed",
        "z_readback_tolerance_mm",
        "actuator_settle_after_change_ms",
        "first_pick_extra_dwell_ms",
        "first_pick_extra_dwell_enabled",
        "force_fallback_dwell",
        "manual_vacuum_before_nozzle_pick",
        "enable_pick_z_extra",
    ]:
        if key in config:
            out[key] = config[key]
    out["enable_pick_z_extra"] = bool(config.get("enable_pick_z_extra", False))
    return out


def disable_pick_z_extra_config(out):
    out["enable_pick_z_extra"] = False
    return out


def get_pick_z_extra_mm(config):
    if not bool(config.get("enable_pick_z_extra", False)):
        return 0.0
    source = None
    raw_value = 0.1
    for key in ["Z_EXTRA", "z_extra", "pick_z_extra_mm"]:
        if key in config:
            source = key
            raw_value = config[key]
            break
    z_extra = float(raw_value)
    if z_extra < 0.0:
        if source is None:
            z_extra = 0.0
        else:
            raise Exception("Pick Z_EXTRA %.6f mm from %s is negative" % (z_extra, source))
    max_extra = float(config.get("pick_z_extra_max_mm", 2.0))
    if max_extra < 0.0:
        raise Exception("pick_z_extra_max_mm %.6f is negative" % max_extra)
    if z_extra > max_extra:
        raise Exception("Pick Z_EXTRA %.6f mm exceeds max %.6f mm" % (z_extra, max_extra))
    return z_extra


def apply_pick_z_extra(nozzle, config, logger=None, speed=None,
        pairwise_phased_motion=False):
    before = get_movable_location(nozzle)
    z_extra = get_pick_z_extra_mm(config)
    if z_extra == 0.0:
        log_msg(logger, "Pick Z_EXTRA disabled: 0.000000 mm")
        before_dict = loc_to_dict(before)
        return {
            "enabled": False,
            "z_extra_mm": z_extra,
            "before_location": before_dict,
            "target_location": before_dict,
            "after_location": before_dict,
            "z_error_mm": 0.0,
        }

    target = mm_loc(
        before.getX(),
        before.getY(),
        float(before.getZ()) - z_extra,
        before.getRotation(),
    )
    log_msg(logger, "Pick Z_EXTRA enabled: %.6f mm" % z_extra)
    log_msg(logger, "Pick Z_EXTRA compression move: old Z %.6f target Z %.6f" % (
        float(before.getZ()), float(target.getZ())))
    log_msg(logger, "Pick Z_EXTRA compression target: %s" % loc_to_log(target))
    if pairwise_phased_motion:
        pairwise_move_z_only_no_comp(
            nozzle, target.getZ(), speed, logger=logger)
    else:
        move_direct_speed(nozzle, target, speed)
    after = get_movable_location(nozzle)
    z_error = float(after.getZ()) - float(target.getZ())
    tolerance = float(config.get(
        "pick_z_extra_readback_tolerance_mm",
        config.get("z_readback_tolerance_mm", 0.05)))
    log_msg(logger, "Pick Z_EXTRA actual readback: %s" % loc_to_log(after))
    log_msg(logger, "Pick Z_EXTRA Z readback: commanded %.6f actual %.6f error %.6f tolerance %.6f" % (
        float(target.getZ()), float(after.getZ()), z_error, tolerance))
    if abs(z_error) > tolerance:
        raise Exception("Pick Z_EXTRA Z readback error %.6f exceeds tolerance %.6f" % (
            z_error, tolerance))
    return {
        "enabled": True,
        "z_extra_mm": z_extra,
        "before_location": loc_to_dict(before),
        "target_location": loc_to_dict(target),
        "after_location": loc_to_dict(after),
        "z_error_mm": z_error,
    }


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


def get_nozzle_tip(nozzle):
    for name in ["getNozzleTip", "getTip"]:
        try:
            method = getattr(nozzle, name)
            tip = method()
            if tip is not None:
                return tip
        except:
            pass
    return None


def dwell_value_to_ms(value):
    if value is None:
        return None
    try:
        return int(round(float(value)))
    except:
        pass
    for name in ["getValue", "getValueAsDouble", "toMillis", "toMilliseconds"]:
        try:
            method = getattr(value, name)
            return int(round(float(method())))
        except:
            pass
    return None


def get_openpnp_dwell_ms(nozzle, names, label, logger=None):
    # Prefer OpenPnP nozzle/nozzle-tip dwell settings. Script dwell config remains
    # only as a compatibility fallback for older configs or missing getters.
    objects = [nozzle, get_nozzle_tip(nozzle)]
    object_names = ["nozzle", "nozzle tip"]
    for i in range(len(objects)):
        obj = objects[i]
        if obj is None:
            continue
        for name in names:
            try:
                method = getattr(obj, name)
                value = method()
                ms = dwell_value_to_ms(value)
                if ms is not None:
                    log_msg(logger, "OpenPnP %s dwell from %s.%s: %d ms" % (
                        label, object_names[i], name, ms))
                    return ms
            except:
                pass
    return None


def get_pick_dwell_ms_from_openpnp(nozzle, logger=None):
    return get_openpnp_dwell_ms(nozzle, [
        "getPickDwellMilliseconds",
        "getPickDwellMs",
        "getPickDwell",
        "getPickDwellTime",
    ], "pick", logger=logger)


def get_place_dwell_ms_from_openpnp(nozzle, logger=None):
    return get_openpnp_dwell_ms(nozzle, [
        "getPlaceDwellMilliseconds",
        "getPlaceDwellMs",
        "getPlaceDwell",
        "getPlaceDwellTime",
    ], "place", logger=logger)


def get_dwell_ms(nozzle, kind, config, fallback_ms, logger=None):
    kind = str(kind)
    if kind == "pick":
        fallback_key = "vacuum_ms"
    else:
        fallback_key = "blowoff_ms"
    fallback = int(config.get(fallback_key, fallback_ms))
    if bool(config.get("force_fallback_dwell", False)):
        log_msg(logger, "Force fallback %s dwell enabled; using %s=%d ms" % (
            kind, fallback_key, fallback))
        return fallback
    if kind == "pick":
        ms = get_pick_dwell_ms_from_openpnp(nozzle, logger=logger)
    else:
        ms = get_place_dwell_ms_from_openpnp(nozzle, logger=logger)
    if ms is not None:
        return ms
    log_msg(logger, "OpenPnP %s dwell unavailable; using legacy fallback %s=%d ms" % (
        kind, fallback_key, fallback))
    return fallback


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

    if speed is None:
        log_msg(logger, "Rotate nozzle lifted unwrapped: current %.6f target %.6f delta %.6f using OpenPnP default motion" %
            (current_r, target_r, target_r - current_r))
    else:
        log_msg(logger, "Rotate nozzle lifted unwrapped: current %.6f target %.6f delta %.6f speed %.6f" %
            (current_r, target_r, target_r - current_r, float(speed)))

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
    value = Boolean.TRUE if bool(on_off) else Boolean.FALSE
    errors = []
    for method_name in ["actuate", "setActuated"]:
        try:
            method = actuator.getClass().getMethod(
                method_name,
                array([Boolean.TYPE], Class)
            )
            method.invoke(actuator, array([value], Object))
            return True
        except Exception, e:
            errors.append("%s(boolean): %s" % (method_name, e))
    raise Exception("Actuator has no supported boolean state method: %s" %
        "; ".join(errors))
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


def current_time_ms():
    try:
        return long(System.currentTimeMillis())
    except:
        return 0


def copy_config(config):
    out = {}
    for key in config:
        out[key] = config[key]
    return out


def machine_from_actuator_or_nozzle(actuator=None, nozzle=None):
    for obj in [actuator, nozzle]:
        if obj is None:
            continue
        try:
            head = obj.getHead()
            if head is not None:
                machine_obj = head.getMachine()
                if machine_obj is not None:
                    return machine_obj
        except:
            pass
    try:
        return Configuration.get().getMachine()
    except:
        return None


def wait_after_actuator_change(actuator, nozzle=None, logger=None, settle_ms=None):
    # Local OpenPnP source inspection:
    # - Actuator extends HeadMountable, so waitForCompletion() is available.
    # - MotionPlanner.WaitForUnconditionalCoordination is documented for custom
    #   actuator G-code that may have moved/queued work even with no pending move.
    # - ReferenceNozzle.pick()/place() actuate vacuum/blowoff and run configured
    #   nozzle/nozzle-tip dwell internally; direct Actuator.actuate() does not
    #   itself mean "dwell complete".
    waited = False
    if actuator is not None:
        try:
            actuator.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
            log_msg(logger, "Actuator completion wait: actuator.waitForCompletion(WaitForUnconditionalCoordination)")
            waited = True
        except Exception, e:
            log_msg(logger, "Actuator completion wait via actuator failed: %s" % e)
    machine_obj = machine_from_actuator_or_nozzle(actuator, nozzle)
    if machine_obj is not None:
        try:
            machine_obj.getMotionPlanner().waitForCompletion(None, CompletionType.WaitForUnconditionalCoordination)
            log_msg(logger, "Actuator completion wait: machine motion planner WaitForUnconditionalCoordination")
            return True
        except Exception, e:
            log_msg(logger, "Actuator completion wait via machine planner failed: %s" % e)
    if waited:
        return True
    if settle_ms is None:
        settle_ms = 50
    settle_ms = int(settle_ms)
    if settle_ms > 0:
        log_msg(logger, "No actuator completion API available; settling explicitly for %d ms" % settle_ms)
        Thread.sleep(settle_ms)
    return False


def reset_runtime_pick_count():
    global _RUNTIME_PICK_COUNT
    _RUNTIME_PICK_COUNT = 0


def runtime_pick_extra_dwell_ms(config, base_dwell_ms):
    global _RUNTIME_PICK_COUNT
    is_first = _RUNTIME_PICK_COUNT == 0
    extra = 0
    if is_first and bool(config.get("first_pick_extra_dwell_enabled", True)):
        extra = int(base_dwell_ms)
    if extra < 0:
        extra = 0
    return is_first, extra


def mark_runtime_pick_success():
    global _RUNTIME_PICK_COUNT
    _RUNTIME_PICK_COUNT += 1


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
    settle_ms = int(config.get("actuator_settle_after_change_ms", 50))
    vacuum = get_vacuum_actuator(nozzle)
    try:
        pick = getattr(nozzle, "pick")
    except:
        pick = None
    try:
        pick_callable = callable(pick)
    except:
        pick_callable = False
    base_dwell_ms = get_dwell_ms(nozzle, "pick", config, 150, logger=logger)
    first_pick, extra_dwell_ms = runtime_pick_extra_dwell_ms(config, base_dwell_ms)
    total_dwell_ms = base_dwell_ms + extra_dwell_ms
    dwell_start_ms = current_time_ms()
    log_msg(logger, "Pick dwell start: base %d extra %d total %d first_pick %s at %d" % (
        base_dwell_ms, extra_dwell_ms, total_dwell_ms, str(first_pick), dwell_start_ms))
    pick_error = None
    if pick_callable:
        try:
            if bool(config.get("manual_vacuum_before_nozzle_pick", False)):
                log_msg(logger, "Vacuum actuator command at %d" % current_time_ms())
                set_actuator_state(vacuum, True)
                log_msg(logger, "Vacuum on")
                wait_after_actuator_change(vacuum, nozzle, logger,
                    config.get("actuator_settle_after_change_ms", settle_ms))
                Thread.sleep(total_dwell_ms)
                now_ms = current_time_ms()
                log_msg(logger, "Pick dwell end: total %d elapsed %d at %d" % (
                    total_dwell_ms, now_ms - dwell_start_ms, now_ms))
            else:
                log_msg(logger, "Vacuum on via nozzle.pick(part, None) at %d" % current_time_ms())
            pick(part, None)
            log_msg(logger, "Nozzle pick(part, None) completed")
        except Throwable, e:
            pick_error = e
            log_msg(logger, "Nozzle pick(part, None) failed after vacuum-on; dwell will still be honored: %s" % e)
        except Exception, e:
            pick_error = e
            log_msg(logger, "Nozzle pick(part, None) failed after vacuum-on; dwell will still be honored: %s" % e)
        wait_after_actuator_change(vacuum, nozzle, logger,
            config.get("actuator_settle_after_change_ms", settle_ms))
        if not bool(config.get("manual_vacuum_before_nozzle_pick", False)):
            if total_dwell_ms > 0:
                log_msg(logger, "Explicit post-pick dwell: %d ms" % total_dwell_ms)
                Thread.sleep(total_dwell_ms)
            now_ms = current_time_ms()
            log_msg(logger, "Pick dwell end: total %d elapsed %d at %d" % (
                total_dwell_ms, now_ms - dwell_start_ms, now_ms))
        if pick_error is not None:
            raise Exception("Nozzle pick(part, None) failed after dwell: %s" % pick_error)
    else:
        log_msg(logger, "Vacuum actuator command at %d" % current_time_ms())
        set_actuator_state(vacuum, True)
        log_msg(logger, "Vacuum on")
        wait_after_actuator_change(vacuum, nozzle, logger,
            config.get("actuator_settle_after_change_ms", settle_ms))
        Thread.sleep(total_dwell_ms)
        now_ms = current_time_ms()
        log_msg(logger, "Pick dwell end: total %d elapsed %d at %d" % (
            total_dwell_ms, now_ms - dwell_start_ms, now_ms))
    check_part_on(nozzle)
    mark_runtime_pick_success()
    log_msg(logger, "Post-pick lift allowed at %d" % current_time_ms())
    return True


def safe_nozzle_release(nozzle, config, logger=None):
    place_error = None
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
            log_msg(logger, "Release via nozzle.place(); OpenPnP owns vacuum/N2 sequence")
            place()
            log_msg(logger, "Nozzle place() completed")
        except Throwable, e:
            place_error = "Nozzle place() failed: %s" % e
            log_msg(logger, place_error)
        except Exception, e:
            place_error = "Nozzle place() failed: %s" % e
            log_msg(logger, place_error)
    else:
        log_msg(logger, "Release without nozzle.place(); vacuum off fallback only")
        vacuum = get_vacuum_actuator(nozzle)
        set_actuator_state(vacuum, False)
        log_msg(logger, "Vacuum off")

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
    if place_error is not None:
        log_msg(logger, "Release actuator sequence completed after place warning")
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

    stage_name = config["result_stage"]
    publish_pipeline_debug_image(config, pipeline, stage_name, logger=logger)
    result = pipeline.getResult(stage_name)
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


def add_unique_offset(offsets, seen, x, y, radius):
    if math.sqrt(x * x + y * y) > radius + 0.000001:
        return
    key = (int(round(x * 1000000.0)), int(round(y * 1000000.0)))
    if key in seen:
        return
    seen[key] = True
    offsets.append((float(x), float(y)))


def normalize_artifact_search_mode(mode):
    mode = str(mode)
    if mode == "radial":
        return "spiral"
    return mode


def normalize_search_radius_step(radius_mm, step_mm):
    radius = float(radius_mm)
    step = float(step_mm)
    if radius < 0.0:
        radius = 0.0
    if step <= 0.0:
        step = radius if radius > 0.0 else 1.0
    return radius, step


def normalize_search_angular_step(angular_step_deg):
    if angular_step_deg is None:
        angular_step_deg = 45.0
    angular_step = float(angular_step_deg)
    if angular_step <= 0.0:
        angular_step = 45.0
    return angular_step


def generate_spiral_search_offsets(radius_mm, step_mm, angular_step_deg=None, pitch_mm=None):
    radius, step = normalize_search_radius_step(radius_mm, step_mm)
    offsets = []
    seen = {}
    add_unique_offset(offsets, seen, 0.0, 0.0, radius)
    if radius <= 0.0:
        return offsets

    # Archimedean spiral: radius grows by roughly one step per revolution, and
    # angular advance is clamped so neighboring points stay close.
    pitch = step
    if pitch_mm is not None:
        pitch = float(pitch_mm)
        if pitch <= 0.0:
            pitch = step
    theta = 0.0
    min_delta = math.radians(10.0)
    max_delta = math.radians(60.0)
    while True:
        r = pitch * theta / (2.0 * math.pi)
        if r > radius + 0.000001:
            break
        if r > 0.000001:
            add_unique_offset(offsets, seen, r * math.cos(theta), r * math.sin(theta), radius)
        delta_theta = step / max(r, step)
        if delta_theta < min_delta:
            delta_theta = min_delta
        if delta_theta > max_delta:
            delta_theta = max_delta
        theta += delta_theta
    return offsets


def generate_ring_search_offsets(radius_mm, step_mm, angular_step_deg=None):
    radius, step = normalize_search_radius_step(radius_mm, step_mm)
    angular_step = normalize_search_angular_step(angular_step_deg)
    offsets = []
    seen = {}
    add_unique_offset(offsets, seen, 0.0, 0.0, radius)
    if radius <= 0.0:
        return offsets

    ring = step
    while ring <= radius + 0.000001:
        if angular_step_deg is None:
            count = int(round((2.0 * math.pi * ring) / step))
            if count < 8:
                count = 8
            angle_step = 360.0 / float(count)
        else:
            angle_step = angular_step
        angle = 0.0
        while angle < 360.0 - 0.000001:
            rad = math.radians(angle)
            add_unique_offset(offsets, seen, ring * math.cos(rad), ring * math.sin(rad), radius)
            angle += angle_step
        ring += step
    return offsets


def generate_cardinal_first_search_offsets(radius_mm, step_mm, angular_step_deg=None):
    radius, step = normalize_search_radius_step(radius_mm, step_mm)
    angular_step = normalize_search_angular_step(angular_step_deg)
    offsets = []
    seen = {}
    add_unique_offset(offsets, seen, 0.0, 0.0, radius)
    if radius <= 0.0:
        return offsets

    ring = step
    while ring <= radius + 0.000001:
        for x, y in [(ring, 0.0), (-ring, 0.0), (0.0, ring), (0.0, -ring)]:
            add_unique_offset(offsets, seen, x, y, radius)
        angle = angular_step
        while angle < 360.0 - 0.000001:
            rad = math.radians(angle)
            add_unique_offset(offsets, seen, ring * math.cos(rad), ring * math.sin(rad), radius)
            angle += angular_step
        ring += step
    return offsets


def generate_raster_search_offsets(radius_mm, step_mm):
    radius, step = normalize_search_radius_step(radius_mm, step_mm)
    offsets = []
    seen = {}
    add_unique_offset(offsets, seen, 0.0, 0.0, radius)
    y = -radius
    while y <= radius + 0.000001:
        x = -radius
        while x <= radius + 0.000001:
            if abs(x) > 0.000001 or abs(y) > 0.000001:
                add_unique_offset(offsets, seen, x, y, radius)
            x += step
        y += step
    return offsets


def generate_artifact_search_offsets(radius_mm, step_mm, mode, angular_step_deg=None, spiral_pitch_mm=None):
    mode = normalize_artifact_search_mode(mode)
    if mode == "spiral":
        return generate_spiral_search_offsets(radius_mm, step_mm, angular_step_deg, spiral_pitch_mm)
    if mode == "rings":
        return generate_ring_search_offsets(radius_mm, step_mm, angular_step_deg)
    if mode == "cardinal_first":
        return generate_cardinal_first_search_offsets(radius_mm, step_mm, angular_step_deg)
    if mode == "raster_legacy":
        return generate_raster_search_offsets(radius_mm, step_mm)
    raise Exception("Unsupported artifact_search_mode: %s" % mode)


def find_artifact_top(camera, nozzle, part, nominal_loc, config, logger=None):
    nominal_loc = nominal_loc.convertToUnits(LengthUnit.Millimeters)
    requested_mode = str(config.get("artifact_search_mode", "spiral"))
    mode = normalize_artifact_search_mode(requested_mode)
    radius = float(config.get("artifact_search_radius_mm", config.get("search_radius_mm", 5.0)))
    step = float(config.get("artifact_search_step_mm", config.get("search_step_mm", 1.0)))
    angular_step = config.get("artifact_search_angular_step_deg")
    spiral_pitch = config.get("artifact_search_spiral_pitch_mm")
    offsets = generate_artifact_search_offsets(radius, step, mode, angular_step, spiral_pitch)
    if config.get("artifact_search_max_points") is not None:
        max_points = int(config["artifact_search_max_points"])
        if max_points > 0 and len(offsets) > max_points:
            offsets = offsets[:max_points]
    use_safe_z_each_point = bool(config.get("artifact_search_use_safe_z_each_point", True))
    log_msg(logger, "Artifact search setup: requested_mode %s mode %s radius %.4f step %.4f angular_step %s spiral_pitch %s points %d safe_z_each_point %s" % (
        requested_mode, mode, radius, step, str(angular_step), str(spiral_pitch), len(offsets), str(use_safe_z_each_point)))
    preview_count = len(offsets)
    if preview_count > 20:
        preview_count = 20
    for preview_index in range(preview_count):
        px, py = offsets[preview_index]
        pr = math.sqrt(px * px + py * py)
        log_msg(logger, "Artifact search offset preview %d/%d: X %.4f Y %.4f R %.4f" % (
            preview_index + 1, preview_count, px, py, pr))

    last_error = None
    direct_search_ready = False
    for index in range(len(offsets)):
        dx, dy = offsets[index]
        log_msg(logger, "Artifact search offset %d/%d start: mode %s X %.4f Y %.4f" % (
            index + 1, len(offsets), mode, dx, dy))
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
            if use_safe_z_each_point or not direct_search_ready:
                move_safe(camera, target)
                direct_search_ready = not use_safe_z_each_point
            else:
                try:
                    move_direct(camera, target)
                except Exception, direct_error:
                    log_msg(logger, "Artifact search direct move failed; retrying this point with safe-Z: %s" % direct_error)
                    direct_search_ready = False
                    use_safe_z_each_point = True
                    move_safe(camera, target)
            actual_camera = get_movable_location(camera)
            log_msg(logger, "Top camera actual before capture: %s" % loc_to_log(actual_camera))
        except Exception, e:
            raise Exception("Motion failure: %s" % e)
        try:
            found = capture_top_pipeline(camera, nozzle, part, config, logger=logger)
        except Exception, e:
            last_error = e
            log_msg(logger, "Artifact search offset %d/%d failed: X %.4f Y %.4f: %s" % (
                index + 1, len(offsets), dx, dy, e))
            continue
        found["search_offset_mm"] = {"x": dx, "y": dy}
        found["camera_location"] = loc_to_dict(target)
        found["search_mode"] = mode
        log_msg(logger, "Artifact search offset succeeded: X %.4f Y %.4f angle %.4f" % (
            dx, dy, float(found["artifact_angle_deg"])))
        return found

    raise Exception("Artifact not found after bounded search: %s" % last_error)


def pick_artifact_from_storage(nozzle, part, center_loc, config, angle_deg=0.0,
        safe_move=True, logger=None, pairwise_phased_motion=False):
    config = copy_pick_z_extra_config(copy_config(config), config)
    log_msg(logger, "Pick Z_EXTRA path enabled: %s" % str(bool(config.get("enable_pick_z_extra", False))))
    storage_pose = pose_from_config(config["storage_location"])
    post_pick_lift_mm = float(config.get("post_pick_lift_mm", 2.0))
    transfer_z_mode = config.get("transfer_z_mode", "relative")
    target = nozzle_target_for_xy(nozzle, center_loc, storage_pose, config, angle_deg)
    expected_z = float(storage_pose.getZ()) + float(config["part_height_mm"])
    log_msg(logger, "Pick commanded target: %s" % loc_to_log(target))
    log_msg(logger, "Pick computed storage base Z: %.6f" % float(storage_pose.getZ()))
    log_msg(logger, "Pick part height: %.6f" % float(config["part_height_mm"]))
    log_msg(logger, "Pick expected target Z: storage base Z %.6f + part height %.6f = %.6f" % (
        float(storage_pose.getZ()), float(config["part_height_mm"]), expected_z))
    log_msg(logger, "Pick nominal target Z: %.6f" % float(target.getZ()))
    log_msg(logger, "Pick target Z: storage base z %.4f + part_height_mm %.4f = z %.4f" % (
        float(storage_pose.getZ()), float(config["part_height_mm"]), float(target.getZ())))
    log_msg(logger, "Pick descend target: %s" % loc_to_dict(target))
    try:
        if pairwise_phased_motion:
            position_speed = normal_motion_speed(config)
            rotation_speed = normal_motion_speed(config)
            z_speed = held_die_z_speed(config)
            log_msg(logger, "Pairwise pickup phase: safe Z")
            pairwise_move_to_safe_z(
                nozzle, config, safe_travel_speed(config), logger=logger)
            log_msg(logger, "Pairwise pickup phase: rotation only")
            pairwise_rotate_only_no_comp(
                nozzle, target.getRotation(), rotation_speed, logger=logger)
            log_msg(logger, "Pairwise pickup phase: XY only")
            pairwise_move_xy_only_no_comp(
                nozzle, target, position_speed, logger=logger)
            log_msg(logger, "Pairwise pickup phase: Z only down")
            pairwise_move_z_only_no_comp(
                nozzle, target.getZ(), z_speed, logger=logger)
        elif safe_move:
            log_msg(logger, "Pick descend safe move uses OpenPnP safe-Z planner speed settings")
            move_safe(nozzle, target)
        else:
            log_msg(logger, "Pick descend direct move uses OpenPnP default motion")
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
        if pairwise_phased_motion:
            z_extra_result = apply_pick_z_extra(
                nozzle, config, logger=logger,
                speed=held_die_z_speed(config),
                pairwise_phased_motion=True)
        else:
            z_extra_result = apply_pick_z_extra(
                nozzle, config, logger=logger)
    except Exception, e:
        raise Exception("Motion failure: %s" % e)

    try:
        safe_nozzle_pick(nozzle, part, config, logger=logger)
    except Exception, e:
        raise Exception("Pick failed: %s" % e)

    try:
        before_lift = get_movable_location(nozzle)
        if pairwise_phased_motion:
            log_msg(logger, "Pairwise pickup phase: Z only up to safe Z")
            lifted = pairwise_move_to_safe_z(
                nozzle, config, held_die_z_speed(config), logger=logger)
        elif str(transfer_z_mode) == "safe_z":
            log_msg(logger, "Post-pick safe-Z lift uses OpenPnP default motion")
            lifted = lift_nozzle_to_safe_z(nozzle, config, logger=None)
        elif post_pick_lift_mm <= 0.0:
            lifted = before_lift
            log_msg(logger, "Post-pick lift skipped; next OpenPnP move owns Z handling")
        else:
            log_msg(logger, "Post-pick relative lift uses OpenPnP default motion")
            lifted = lift_nozzle_relative(nozzle, post_pick_lift_mm, logger=None)
        log_msg(logger, "Post-pick lift: old Z %.6f new Z %.6f" % (
            float(before_lift.getZ()), float(lifted.getZ())))
    except Exception, e:
        raise Exception("Motion failure: %s" % e)

    return {
        "pick_target": loc_to_dict(target),
        "pick_nominal_target": loc_to_dict(target),
        "pick_z_extra": z_extra_result,
        "pick_z_extra_target": z_extra_result["target_location"],
        "pick_actual_contact_target": z_extra_result["target_location"],
        "pick_angle_requested_deg": float(angle_deg),
        "pick_angle_commanded_deg": float(target.getRotation()),
        "post_pick_lift_location": loc_to_dict(lifted),
        "transfer_z_mode": str(transfer_z_mode),
    }


def place_artifact_to_cal_area(nozzle, part, target_center_loc, config, angle_deg=0.0,
        safe_move=True, logger=None, separate_pairwise_180=False):
    has_pick_z_extra_keys = False
    for key in ["Z_EXTRA", "z_extra", "pick_z_extra_mm"]:
        if key in config:
            has_pick_z_extra_keys = True
            break
    config = disable_pick_z_extra_config(copy_config(config))
    log_msg(logger, "Place Z_EXTRA path enabled: %s" % str(bool(config.get("enable_pick_z_extra", False))))
    if has_pick_z_extra_keys:
        log_msg(logger, "Place path ignoring pick-only Z_EXTRA keys")
    cal_pose = pose_from_config(config["calibration_location"])
    post_place_lift_mm = float(config.get("post_place_lift_mm", 2.0))
    transfer_z_mode = config.get("transfer_z_mode", "relative")
    final_storage_assert_location = config.get("final_storage_assert_location")
    final_storage_assert_tolerance = float(config.get(
        "storage_return_target_assert_tolerance_mm",
        config.get("final_storage_target_tolerance_mm",
            config.get("final_storage_assert_tolerance_mm", 0.001))))
    target = nozzle_target_for_xy(nozzle, target_center_loc, cal_pose, config, angle_deg)
    intended_x = float(target.getX())
    intended_y = float(target.getY())
    intended_z = float(target.getZ())
    intended_r = float(target.getRotation())
    intended_place_target = loc_to_dict(target)
    log_msg(logger, "Place target Z: calibration base z %.4f + part_height_mm %.4f = z %.4f" % (
        float(cal_pose.getZ()), float(config["part_height_mm"]), intended_z))
    try:
        before_transfer = get_movable_location(nozzle)
        log_msg(logger, "Before place move readback: %s" % loc_to_log(before_transfer))
        log_msg(logger, "Intended place target: %s" % intended_place_target)
        if safe_move and separate_pairwise_180:
            log_msg(logger, "Pairwise placement phase: confirm safe Z")
            pairwise_move_to_safe_z(
                nozzle, config, held_die_z_speed(config), logger=logger)
            log_msg(logger, "Pairwise placement uses rotation-only, XY-only, and Z-only phases")
            move_pairwise_180_at_safe_z(
                nozzle,
                target,
                logger=logger,
                speed=held_die_speed(config),
                z_speed=held_die_z_speed(config)
            )
        elif safe_move:
            log_msg(logger, "Place move safe-Z uses OpenPnP planner default motion")
            move_safe(nozzle, target)
        else:
            log_msg(logger, "Place move direct uses OpenPnP default motion")
            move_direct(nozzle, target)
        after_transfer = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        rotation_compensation_dx_mm = float(after_transfer.getX()) - float(before_transfer.getX())
        rotation_compensation_dy_mm = float(after_transfer.getY()) - float(before_transfer.getY())
        rotation_compensation_dr_deg = normalize_rotation(
            float(after_transfer.getRotation()) - float(before_transfer.getRotation()))
        rotation_error = normalize_rotation(
            float(after_transfer.getRotation()) - float(intended_r))
        log_msg(logger, "After place move readback: %s" % loc_to_log(after_transfer))
        log_msg(logger, "Place move delta: X %.6f Y %.6f R %.6f" % (
            rotation_compensation_dx_mm, rotation_compensation_dy_mm, rotation_compensation_dr_deg))
        log_msg(logger, "Place rotation readback: commanded %.6f actual %.6f error %.6f" % (
            intended_r, float(after_transfer.getRotation()), rotation_error))
        if abs(rotation_error) > float(config.get("held_die_rotation_tolerance_deg", 0.5)):
            raise Exception("Held-die rotation readback error %.6f exceeds tolerance %.6f" % (
                rotation_error, float(config.get("held_die_rotation_tolerance_deg", 0.5))))
        dx_to_target = intended_x - float(after_transfer.getX())
        dy_to_target = intended_y - float(after_transfer.getY())
        xy_move_mag = math.sqrt(dx_to_target * dx_to_target + dy_to_target * dy_to_target)
        log_msg(logger, "Place move residual XY delta: %.6f" % xy_move_mag)
        lower_target = after_transfer
        expected_z = float(cal_pose.getZ()) + float(config["part_height_mm"])
        log_msg(logger, "Place commanded target: %s" % loc_to_log(target))
        log_msg(logger, "Place computed calibration base Z: %.6f" % float(cal_pose.getZ()))
        log_msg(logger, "Place part height: %.6f" % float(config["part_height_mm"]))
        log_msg(logger, "Place expected target Z: calibration base Z %.6f + part height %.6f = %.6f" % (
            float(cal_pose.getZ()), float(config["part_height_mm"]), expected_z))
        z_error = float(after_transfer.getZ()) - intended_z
        log_msg(logger, "Place actual readback/DRO: %s" % loc_to_log(after_transfer))
        log_msg(logger, "Place Z readback: commanded %.6f actual %.6f error %.6f" % (
            intended_z, float(after_transfer.getZ()), z_error))
        if abs(z_error) > float(config.get("z_readback_tolerance_mm", 0.05)):
            raise Exception("Place Z readback error %.6f exceeds tolerance %.6f" % (
                z_error, float(config.get("z_readback_tolerance_mm", 0.05))))
        if final_storage_assert_location is not None:
            configured_storage = pose_from_config(final_storage_assert_location)
            storage_dx = intended_x - float(configured_storage.getX())
            storage_dy = intended_y - float(configured_storage.getY())
            actual_storage_dx = float(lower_target.getX()) - float(configured_storage.getX())
            actual_storage_dy = float(lower_target.getY()) - float(configured_storage.getY())
            log_msg(logger, "Final storage target assertion: commanded X %.6f Y %.6f actual X %.6f Y %.6f configured X %.6f Y %.6f commanded delta X %.6f Y %.6f actual delta X %.6f Y %.6f tolerance %.6f" % (
                intended_x, intended_y,
                float(lower_target.getX()), float(lower_target.getY()),
                float(configured_storage.getX()), float(configured_storage.getY()),
                storage_dx, storage_dy,
                actual_storage_dx, actual_storage_dy,
                final_storage_assert_tolerance))
            if (abs(storage_dx) > final_storage_assert_tolerance
                    or abs(storage_dy) > final_storage_assert_tolerance
                    or abs(actual_storage_dx) > final_storage_assert_tolerance
                    or abs(actual_storage_dy) > final_storage_assert_tolerance):
                log_msg(logger, "Warning: final storage target assertion exceeded before release; continuing to release: commanded X %.6f Y %.6f actual X %.6f Y %.6f configured X %.6f Y %.6f tolerance %.6f" % (
                    intended_x, intended_y,
                    float(lower_target.getX()), float(lower_target.getY()),
                    float(configured_storage.getX()), float(configured_storage.getY()),
                    final_storage_assert_tolerance))
    except Exception, e:
        raise Exception("Motion failure: %s" % e)

    try:
        safe_nozzle_release(nozzle, config, logger=logger)
    except Exception, e:
        raise Exception("Place failed: %s" % e)

    try:
        before_lift = get_movable_location(nozzle)
        if safe_move and separate_pairwise_180:
            log_msg(logger, "Pairwise placement phase: Z only up to safe Z")
            lifted = pairwise_move_to_safe_z(
                nozzle, config, held_die_z_speed(config), logger=logger)
        elif str(transfer_z_mode) == "safe_z":
            log_msg(logger, "Post-place safe-Z lift uses OpenPnP default motion")
            lifted = lift_nozzle_to_safe_z(nozzle, config, logger=None)
        else:
            log_msg(logger, "Post-place relative lift uses OpenPnP default motion")
            lifted = lift_nozzle_relative(nozzle, post_place_lift_mm, logger=None)
        log_msg(logger, "Post-place lift: old Z %.6f new Z %.6f" % (
            float(before_lift.getZ()), float(lifted.getZ())))
    except Exception, e:
        raise Exception("Motion failure: %s" % e)

    return {
        "place_target": loc_to_dict(lower_target),
        "place_angle_requested_deg": float(angle_deg),
        "place_angle_commanded_deg": intended_r,
        "rotation_readback_before": loc_to_dict(before_transfer),
        "rotation_readback_after": loc_to_dict(after_transfer),
        "rotation_compensation_dx_mm": rotation_compensation_dx_mm,
        "rotation_compensation_dy_mm": rotation_compensation_dy_mm,
        "rotation_compensation_dr_deg": rotation_compensation_dr_deg,
        "intended_place_target": intended_place_target,
        "actual_place_xy_before_lower": loc_to_dict(after_transfer),
        "post_place_lift_location": loc_to_dict(lifted),
        "transfer_z_mode": str(transfer_z_mode),
    }


def return_artifact_to_known_pose(nozzle, camera, part, current_loc, config):
    storage_pose = pose_from_config(config["storage_location"])
    current_rot = current_loc.convertToUnits(LengthUnit.Millimeters).getRotation()
    storage_return_target_assert_tolerance = config.get(
        "storage_return_target_assert_tolerance_mm",
        config.get("final_storage_target_tolerance_mm",
            config.get("final_storage_assert_tolerance_mm", 0.001)))
    return_pick_config = copy_pick_z_extra_config({
        "storage_location": config["calibration_location"],
        "part_height_mm": config["part_height_mm"],
        "vacuum_ms": config.get("vacuum_ms", 150),
        "force_fallback_dwell": config.get("force_fallback_dwell", False),
        "post_pick_lift_mm": config.get("post_pick_lift_mm", 2.0),
        "transfer_z_mode": "safe_z",
        "safe_travel_z": safe_travel_z_for_config(config),
        "calibration_motion_speed": normal_motion_speed(config),
        "normal_motion_speed": normal_motion_speed(config),
        "pairwise_motion_speed": normal_motion_speed(config),
        "held_die_motion_speed": normal_motion_speed(config),
    }, config)
    pick_artifact_from_storage(nozzle, part, current_loc, return_pick_config, current_rot)
    return place_artifact_to_cal_area(nozzle, part, storage_pose, {
        "calibration_location": config["storage_location"],
        "part_height_mm": config["part_height_mm"],
        "blowoff_ms": config.get("blowoff_ms", 120),
        "force_fallback_dwell": config.get("force_fallback_dwell", False),
        "post_place_lift_mm": config.get("post_place_lift_mm", 2.0),
        "transfer_z_mode": "safe_z",
        "safe_travel_z": safe_travel_z_for_config(config),
        "calibration_motion_speed": normal_motion_speed(config),
        "normal_motion_speed": normal_motion_speed(config),
        "pairwise_motion_speed": normal_motion_speed(config),
        "held_die_motion_speed": normal_motion_speed(config),
        "final_storage_assert_location": config["storage_location"],
        "storage_return_target_assert_tolerance_mm": storage_return_target_assert_tolerance,
        "final_storage_assert_tolerance_mm": storage_return_target_assert_tolerance,
    }, storage_pose.getRotation())


def artifact_measurement_summary(found):
    if found is None:
        return {
            "final_artifact_pose": None,
            "final_artifact_pose_mm": None,
            "final_artifact_angle_deg": None,
            "final_nozzle_pick_target_mm": None,
            "final_search_offset_mm": None,
            "final_camera_location_mm": None,
        }
    center = found.get("center_location")
    pick = detection_pick_location(found)
    return {
        "final_artifact_pose": loc_to_dict(center),
        "final_artifact_pose_mm": loc_to_dict(center),
        "final_artifact_angle_deg": float(found["artifact_angle_deg"]),
        "final_nozzle_pick_target_mm": loc_to_dict(pick),
        "final_search_offset_mm": found.get("search_offset_mm"),
        "final_camera_location_mm": found.get("camera_location"),
    }


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
        "final_artifact_pose_mm": None,
        "final_artifact_angle_deg": None,
        "final_nozzle_pick_target_mm": None,
        "final_search_offset_mm": None,
        "final_camera_location_mm": None,
    }


def solve_head_offset(config, nozzle=None, camera=None, machine_obj=None, part=None, logger=None):
    per_angle_records = []
    angles = [float(a) for a in config.get("angles_deg", DEFAULT_ANGLES_DEG)]

    try:
        log_msg(logger, "Head offset solver start")
        reset_runtime_pick_count()
        config.setdefault("pairwise_180_direction", 1.0)
        config.setdefault("calibration_motion_speed", config.get("normal_motion_speed", 1.0))
        config.setdefault("pairwise_motion_speed", config.get("held_die_motion_speed", 0.60))
        config.setdefault("normal_motion_speed", config.get("calibration_motion_speed", 1.0))
        config.setdefault("held_die_motion_speed", config.get("pairwise_motion_speed", 0.60))
        config.setdefault("actuator_settle_after_change_ms", 50)
        config.setdefault("first_pick_extra_dwell_ms", 250)
        config.setdefault("first_pick_extra_dwell_enabled", True)
        config.setdefault("force_fallback_dwell", False)
        log_motion_speed_config(config, logger=logger)
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
        storage_transfer_config = config_with_transfer_speed(
            config, "safe_z", log_resolved_motion_speed(config, "normal", 1.0, logger=logger))
        pairwise_transfer_config = config_with_transfer_speed(
            config, "relative", log_resolved_motion_speed(config, "held_die", 0.60, logger=logger))

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
        final_artifact_measurement = None
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
                old_pick_target = current_pose["center_location"]
                old_artifact_angle_deg = current_artifact_angle_deg
                log_msg(logger, "Pairwise using carried pose: center %s angle %.6f" % (
                    loc_to_dict(old_center), old_artifact_angle_deg))
                log_msg(logger, "Pairwise using top vision camera-only center for pick: %s" % loc_to_log(old_pick_target))
                log_msg(logger, "Using top vision camera-only center for measurement/math: %s" % loc_to_log(old_center))
                pick_angle = float(angle)
                place_angle = unwrapped_plus_180(pick_angle, config.get("pairwise_180_direction", 1.0))
                pair_pick_config = copy_pick_z_extra_config({
                    "storage_location": config["calibration_location"],
                    "part_height_mm": config["part_height_mm"],
                    "vacuum_ms": config.get("vacuum_ms", 150),
                    "post_pick_lift_mm": 0.0,
                    "transfer_z_mode": "relative",
                    "safe_travel_z": safe_travel_z_for_config(config),
                    "safe_travel_speed": safe_travel_speed(config),
                    "calibration_motion_speed": normal_motion_speed(config),
                    "normal_motion_speed": normal_motion_speed(config),
                    "pairwise_motion_speed": held_die_speed(config),
                    "held_die_motion_speed": held_die_speed(config),
                    "held_die_z_speed": held_die_z_speed(config),
                }, config)
                log_msg(logger, "Pairwise: suppress dynamic compensation")
                try:
                    pick_result = pick_artifact_from_storage(
                        nozzle, part, old_pick_target, pair_pick_config, pick_angle,
                        logger=logger, pairwise_phased_motion=True)
                    # Pairwise correction must place back to old_center. Nominal recentering is housekeeping only and must not enter dx/dy correction math.
                    log_msg(logger, "Pairwise measurement place target is old_center, not nominal center")
                    place_result = place_artifact_to_cal_area(
                        nozzle, part, old_center, pairwise_transfer_config, place_angle, True,
                        logger=logger, separate_pairwise_180=True)
                finally:
                    log_msg(logger, "Pairwise: restore normal compensation")

                measured = find_artifact_top(camera, nozzle, part,
                    xy_with_base_pose(old_center, calibration_pose), config, logger=logger)
                new_center = measured["center_location"].convertToUnits(LengthUnit.Millimeters)
                new_artifact_angle_deg = float(measured["artifact_angle_deg"])
                final_artifact_pose = loc_to_dict(new_center)
                final_artifact_measurement = measured
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
                        recenter_pick_target = measured["center_location"]
                        log_msg(logger, "Pairwise recenter using top vision camera-only center for pick: %s" % loc_to_log(recenter_pick_target))
                        log_msg(logger, "Using top vision camera-only center for measurement/math: %s" % loc_to_log(new_center))
                        recenter_pick_config = copy_pick_z_extra_config({
                            "storage_location": config["calibration_location"],
                            "part_height_mm": config["part_height_mm"],
                            "vacuum_ms": config.get("vacuum_ms", 150),
                            "post_pick_lift_mm": 0.0,
                            "transfer_z_mode": "relative",
                            "safe_travel_z": safe_travel_z_for_config(config),
                            "safe_travel_speed": safe_travel_speed(config),
                            "calibration_motion_speed": normal_motion_speed(config),
                            "normal_motion_speed": normal_motion_speed(config),
                            "pairwise_motion_speed": held_die_speed(config),
                            "held_die_motion_speed": held_die_speed(config),
                            "held_die_z_speed": held_die_z_speed(config),
                        }, config)
                        log_msg(logger, "Pairwise recenter: suppress dynamic compensation")
                        try:
                            pick_artifact_from_storage(
                                nozzle, part, recenter_pick_target,
                                recenter_pick_config, recenter_pick_angle,
                                logger=logger, pairwise_phased_motion=True)
                            place_artifact_to_cal_area(
                                nozzle, part, nominal_pairwise_center,
                                pairwise_transfer_config,
                                new_artifact_angle_deg, True,
                                logger=logger, separate_pairwise_180=True)
                        finally:
                            log_msg(logger, "Pairwise recenter: restore normal compensation")
                        recentered = find_artifact_top(camera, nozzle, part, calibration_pose, config, logger=logger)
                        pair_record["recenter_result_center_mm"] = loc_to_dict(recentered["center_location"])
                        pair_record["recenter_result_artifact_angle_deg"] = float(recentered["artifact_angle_deg"])
                        final_artifact_pose = loc_to_dict(recentered["center_location"])
                        final_artifact_measurement = recentered
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

        final_summary = artifact_measurement_summary(final_artifact_measurement)
        log_msg(logger, "Final pairwise artifact pose handoff: center %s angle %s pick %s" % (
            str(final_summary["final_artifact_pose_mm"]),
            str(final_summary["final_artifact_angle_deg"]),
            str(final_summary["final_nozzle_pick_target_mm"])))

        result = {
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
        for key in final_summary:
            result[key] = final_summary[key]
        return result

    except Exception, e:
        log_msg(logger, "Head offset solver exception: %s" % e)
        return result_failure(e)
