# Simple.Footprint.Pad.Dispense.Test.py
#
# Test dispense pads for one hardcoded part.
#
# Before running:
#   Move the dispense needle to the center of the part.
#   Needle Z should be touching / set at the dispense surface height.
#
# The script uses current dispense-head X/Y/Z as:
#   X/Y = part center
#   Z   = surface height
#
# Then it pulls footprint pads from PART_ID and dispenses at each pad.

from jarray import array
from java.lang import Boolean, Class, Double, Math, Object, System, Thread
from org.openpnp.model import Configuration, LengthUnit, Location
from org.openpnp.spi.MotionPlanner import CompletionType
from java.util.concurrent import Callable

# ----------------- HARD CODED TEST CONFIG -----------------

PART_ID = "Si_N2_5.4mm"

DISPENSE_HEAD_NAME     = "Dispense Head"
DISPENSE_ACTUATOR_NAME = "GlueDispense"

# Rotation of the footprint relative to the current needle-centered part.
# 0 means footprint pad X/Y are used directly around current needle X/Y.
PART_ROTATION_DEG = 0.0

# Current needle Z when script starts is surface Z.
# Dispense Z = surface Z + DISPENSE_Z_OFFSET_MM
DISPENSE_Z_OFFSET_MM = 0.1

# Lift between pads.
INTER_PAD_LIFT_MM = 0.1

# Optional final move after all pads.
MOVE_TO_SAFE_Z_AT_END = True
DISPENSE_SAFE_Z_MM = 25.0

TRAVEL_SPEED_FACTOR   = 0.9
DISPENSE_SPEED_FACTOR = 0.9

PRE_DISPENSE_DWELL_MS  = 10
DISPENSE_PULSE_MS      = 15
POST_DISPENSE_DWELL_MS = 10
ACTUATOR_SETTLE_MS     = 20

EXTRA_FIRST_PAD_DOTS = 3
EXTRA_FIRST_PAD_DOT_SPACING_MM = 0.05

PREVENT_CORNER_CUTTING = True
CORNER_INSERT_TOLERANCE_MM = 0.05

FINAL_DRAG_AFTER_LAST_DOT = True
# When enabled, close the final down-at-Z path back to the first dispense
# location before lifting. Distance is reserved for optional future overshoot.
FINAL_DRAG_DISTANCE_MM = 0.15

PAD_SKIP_NAMES = set()

TAG = "Simple.Pad.Dispense.Test"


# ----------------- BASIC HELPERS -----------------

def log(msg):
    print "[%s] %s" % (TAG, str(msg))


def log_err(msg):
    print "[%s ERROR] %s" % (TAG, str(msg))


SCRIPT_T0_NS = System.nanoTime()
LAST_T_NS = SCRIPT_T0_NS


def elapsed_ms_since(t0_ns):
    return (System.nanoTime() - t0_ns) / 1000000.0


def log_t(label):
    global LAST_T_NS
    now_ns = System.nanoTime()
    abs_ms = (now_ns - SCRIPT_T0_NS) / 1000000.0
    delta_ms = (now_ns - LAST_T_NS) / 1000000.0
    LAST_T_NS = now_ns
    log("[TIMER] %+9.3f ms | +%8.3f ms | %s" %
        (abs_ms, delta_ms, label))
    return now_ns


def sleep_ms(ms):
    ms = int(ms)
    if ms > 0:
        Thread.sleep(ms)


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
    for method in ["getName", "getId"]:
        try:
            value = getattr(obj, method)()
            if value is not None:
                return str(value)
        except:
            pass
    return str(obj)


def wait_still(movable, label):
    t0 = log_t("%s: wait_still BEFORE" % label)
    movable.waitForCompletion(CompletionType.WaitForStillstand)
    log_t("%s: wait_still AFTER elapsed=%.3f ms" %
          (label, elapsed_ms_since(t0)))


def wait_coord(movable, label):
    t0 = log_t("%s: wait_coord BEFORE" % label)
    movable.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
    log_t("%s: wait_coord AFTER elapsed=%.3f ms" %
          (label, elapsed_ms_since(t0)))


# ----------------- FIND MACHINE ITEMS -----------------

def find_dispense_head(mac):
    head = mac.getDefaultHead()
    wanted = str(DISPENSE_HEAD_NAME)

    for getter in ["getHeadMountables", "getNozzles"]:
        try:
            for item in java_list_to_py(getattr(head, getter)()):
                if object_name(item) == wanted:
                    log("Using dispense head mountable '%s'." % object_name(item))
                    return item
        except:
            pass

    try:
        item = head.getNozzleByName(wanted)
        if item is not None:
            log("Using dispense head mountable '%s'." % object_name(item))
            return item
    except:
        pass

    raise Exception("Required dispense head not found: %s" % wanted)


def find_glue_dispense_actuator(mac):
    wanted = str(DISPENSE_ACTUATOR_NAME)
    owners = []

    try:
        owners.append(mac.getDefaultHead())
    except:
        pass

    owners.append(mac)

    for owner in owners:
        if owner is None:
            continue

        for method in ["getActuatorByName", "getActuator"]:
            try:
                actuator = getattr(owner, method)(wanted)
                if actuator is not None and object_name(actuator) == wanted:
                    log("Using dispense actuator '%s'." % object_name(actuator))
                    return actuator
            except:
                pass

        try:
            for actuator in java_list_to_py(owner.getActuators()):
                if object_name(actuator) == wanted:
                    log("Using dispense actuator '%s'." % object_name(actuator))
                    return actuator
        except:
            pass

    raise Exception("Required dispense actuator not found: %s" % wanted)


def find_part_by_id(part_id):
    # Try bound global part.
    try:
        if "part" in globals() and part is not None:
            pid = str(part.getId())
            log("Bound global part id=%s" % pid)
            if pid == str(part_id):
                return part
    except:
        pass

    # Try bound placement part.
    try:
        if "placement" in globals() and placement is not None:
            p = placement.getPart()
            if p is not None:
                pid = str(p.getId())
                log("Bound placement part id=%s" % pid)
                if pid == str(part_id):
                    return p
    except:
        pass

    # Try Configuration.get().getParts() if available.
    try:
        cfg = Configuration.get()
        parts = cfg.getParts()
        log("Scanning Configuration.get().getParts()...")
        for p in java_list_to_py(parts):
            try:
                pid = str(p.getId())
                log("Part candidate: %s" % pid)
                if pid == str(part_id):
                    return p
            except:
                pass
    except Exception, e:
        log("Config part scan failed: %s" % e)

    raise Exception(
        "Part ID '%s' was not found. Check the log above for candidate IDs, "
        "or run this from a placement/job context where global 'part' or 'placement' is bound."
        % part_id
    )


# ----------------- ACTUATOR -----------------

def set_boolean_actuator_state_fast(actuator, on_off, label):
    value = Boolean.TRUE if bool(on_off) else Boolean.FALSE

    method = actuator.getClass().getMethod(
        "actuate",
        array([Boolean.TYPE], Class)
    )

    method.invoke(actuator, array([value], Object))

    log("%s boolean actuator=%s queued." % (label, str(bool(on_off))))


def wait_after_actuator_change(actuator, machine_obj, settle_ms, label):
    if actuator is not None:
        try:
            actuator.waitForCompletion(
                CompletionType.WaitForUnconditionalCoordination)
            log_t("%s: actuator wait complete" % label)
            return
        except:
            pass

    if machine_obj is not None:
        try:
            machine_obj.getMotionPlanner().waitForCompletion(
                None,
                CompletionType.WaitForUnconditionalCoordination)
            log_t("%s: motion planner wait complete" % label)
            return
        except:
            pass

    sleep_ms(settle_ms)
    log_t("%s: fallback sleep %d ms complete" %
          (label, int(settle_ms)))


def pulse_glue_controller(actuator, machine_obj, pad_index):
    t0 = log_t(
        "pad %d: pulse_glue_controller START pulse=%d ms settle=%d ms post=%d ms" %
        (pad_index, DISPENSE_PULSE_MS, ACTUATOR_SETTLE_MS,
         POST_DISPENSE_DWELL_MS)
    )

    set_boolean_actuator_state_fast(
        actuator,
        False,
        "pad %d GlueDispense OFF before pulse" % pad_index)
    wait_after_actuator_change(
        actuator,
        machine_obj,
        ACTUATOR_SETTLE_MS,
        "pad %d GlueDispense OFF wait" % pad_index)
    sleep_ms(ACTUATOR_SETTLE_MS)

    set_boolean_actuator_state_fast(
        actuator,
        True,
        "pad %d GlueDispense ON" % pad_index)
    wait_after_actuator_change(
        actuator,
        machine_obj,
        ACTUATOR_SETTLE_MS,
        "pad %d GlueDispense ON wait" % pad_index)
    sleep_ms(DISPENSE_PULSE_MS)

    set_boolean_actuator_state_fast(
        actuator,
        False,
        "pad %d GlueDispense OFF after pulse" % pad_index)
    wait_after_actuator_change(
        actuator,
        machine_obj,
        ACTUATOR_SETTLE_MS,
        "pad %d GlueDispense final OFF wait" % pad_index)
    sleep_ms(ACTUATOR_SETTLE_MS)
    sleep_ms(POST_DISPENSE_DWELL_MS)

    log_t("pad %d: pulse_glue_controller END host elapsed=%.3f ms" %
          (pad_index, elapsed_ms_since(t0)))


def force_glue_off_safe(machine_obj, label):
    try:
        glue = find_glue_dispense_actuator(machine_obj)
        set_boolean_actuator_state_fast(
            glue,
            False,
            "%s GlueDispense forced off" % label)
        wait_after_actuator_change(
            glue,
            machine_obj,
            ACTUATOR_SETTLE_MS,
            "%s GlueDispense forced off wait" % label)
    except Exception, e:
        log_err("%s failed to force GlueDispense off: %s" % (label, e))


# ----------------- NO DYNAMIC COMP MOVE HELPERS -----------------

def get_location_option(option_name):
    try:
        from org.openpnp.spi.Locatable import LocationOption
        return getattr(LocationOption, option_name)
    except:
        pass

    try:
        enum_class = Class.forName("org.openpnp.spi.Locatable$LocationOption")
        values = enum_class.getEnumConstants()
        for value in values:
            if str(value.name()) == option_name:
                return value
    except Exception, e:
        log_err("Could not resolve LocationOption.%s: %s" %
                (option_name, e))

    raise Exception("Could not resolve LocationOption.%s" % option_name)


def full_location_no_nan(movable, target):
    current = movable.getLocation()
    target_units = target.getUnits()
    current_units = current.convertToUnits(target_units)

    x = current_units.getX() if Double.isNaN(target.getX()) else target.getX()
    y = current_units.getY() if Double.isNaN(target.getY()) else target.getY()
    z = current_units.getZ() if Double.isNaN(target.getZ()) else target.getZ()
    r = current_units.getRotation() if Double.isNaN(target.getRotation()) else target.getRotation()

    return Location(target_units, x, y, z, r)


def move_direct_no_dynamic_comp(movable, target, speed, label):
    full_target = full_location_no_nan(movable, target)

    log_t("%s: move_direct_no_dynamic_comp BEFORE target=%s speed=%.3f" %
          (label, str(full_target), float(speed)))

    suppress_dynamic = get_location_option("SuppressDynamicCompensation")

    head_location = movable.toHeadLocation(
        full_target,
        suppress_dynamic
    )

    movable.getHead().moveTo(
        movable,
        head_location,
        movable.getHead().getMaxPartSpeed() * float(speed)
    )

    log_t("%s: move_direct_no_dynamic_comp AFTER head.moveTo" % label)


def move_z_only_no_dynamic_comp(movable, units, z, speed, label):
    target = Location(units, Double.NaN, Double.NaN, z, Double.NaN)

    log_t("%s: move_z_only_no_dynamic_comp BEFORE target=%s speed=%.3f" %
          (label, str(target), float(speed)))

    move_direct_no_dynamic_comp(movable, target, speed, label)

    log_t("%s: move_z_only_no_dynamic_comp AFTER" % label)


def dispense_head_location(units, x, y, z):
    return Location(units, x, y, z, Double.NaN)


# ----------------- PAD HELPERS -----------------

def pad_xy(pad):
    try:
        return pad.getCenterX(), pad.getCenterY()
    except:
        return pad.getX(), pad.getY()


def get_footprint_units(footprint, fallback_units):
    try:
        return footprint.getUnits()
    except:
        return fallback_units


def mm_to_units(value_mm, units):
    loc = Location(LengthUnit.Millimeters, float(value_mm), 0.0, 0.0, 0.0)
    return loc.convertToUnits(units).getX()


def make_pad_path_item(pad_index, pad_name, x, y, dot_index, total_dots):
    return {
        "type": "pad",
        "pad_index": pad_index,
        "pad_name": pad_name,
        "x": x,
        "y": y,
        "dot_index": dot_index,
        "total_dots": total_dots
    }


def make_corner_path_item(from_item, to_item, x, y):
    return {
        "type": "corner",
        "from_pad_index": from_item["pad_index"],
        "to_pad_index": to_item["pad_index"],
        "x": x,
        "y": y
    }


def expand_first_pad_dots(pad_positions, units):
    if len(pad_positions) == 0:
        return []

    extra_dots = int(EXTRA_FIRST_PAD_DOTS)
    if extra_dots < 0:
        extra_dots = 0

    total_dots = 1 + extra_dots
    if total_dots <= 1:
        out = []
        for item in pad_positions:
            out.append(make_pad_path_item(
                item["pad_index"],
                item["pad_name"],
                item["x"],
                item["y"],
                0,
                1))
        return out

    first = pad_positions[0]
    vx = 1.0
    vy = 0.0

    if len(pad_positions) > 1:
        next_item = pad_positions[1]
        dx = next_item["x"] - first["x"]
        dy = next_item["y"] - first["y"]
        dist = Math.sqrt(dx * dx + dy * dy)
        if dist > 0.0:
            vx = dx / dist
            vy = dy / dist

    spacing = mm_to_units(EXTRA_FIRST_PAD_DOT_SPACING_MM, units)
    center_index = (float(total_dots) - 1.0) / 2.0

    out = []
    for dot_index in range(total_dots):
        offset = (float(dot_index) - center_index) * spacing
        out.append(make_pad_path_item(
            first["pad_index"],
            first["pad_name"],
            first["x"] + vx * offset,
            first["y"] + vy * offset,
            dot_index,
            total_dots))

    for i in range(1, len(pad_positions)):
        item = pad_positions[i]
        out.append(make_pad_path_item(
            item["pad_index"],
            item["pad_name"],
            item["x"],
            item["y"],
            0,
            1))

    log("Expanded first non-skipped pad %d '%s' to %d dots with %.4f mm spacing." %
        (first["pad_index"], first["pad_name"], total_dots,
         EXTRA_FIRST_PAD_DOT_SPACING_MM))

    return out


def mostly_horizontal(from_item, to_item):
    dx = abs(to_item["x"] - from_item["x"])
    dy = abs(to_item["y"] - from_item["y"])
    return dx > dy


def mostly_vertical(from_item, to_item):
    dx = abs(to_item["x"] - from_item["x"])
    dy = abs(to_item["y"] - from_item["y"])
    return dy > dx


def choose_corner_xy(items, index):
    current_item = items[index]
    next_item = items[index + 1]

    previous_item = None
    for i in range(index - 1, -1, -1):
        if items[i]["pad_index"] != current_item["pad_index"]:
            previous_item = items[i]
            break

    if previous_item is not None:
        if mostly_horizontal(previous_item, current_item):
            return next_item["x"], current_item["y"]
        if mostly_vertical(previous_item, current_item):
            return current_item["x"], next_item["y"]

    after_next_item = None
    for i in range(index + 2, len(items)):
        if items[i]["pad_index"] != next_item["pad_index"]:
            after_next_item = items[i]
            break

    if after_next_item is not None:
        if mostly_horizontal(next_item, after_next_item):
            return current_item["x"], next_item["y"]
        if mostly_vertical(next_item, after_next_item):
            return next_item["x"], current_item["y"]

    return next_item["x"], current_item["y"]


def insert_corner_waypoints(items, units):
    if not PREVENT_CORNER_CUTTING:
        return items, 0

    tolerance = abs(mm_to_units(CORNER_INSERT_TOLERANCE_MM, units))
    if len(items) < 2:
        return items, 0

    planned = []
    inserted = 0

    for i in range(len(items) - 1):
        current_item = items[i]
        next_item = items[i + 1]
        planned.append(current_item)

        dx = next_item["x"] - current_item["x"]
        dy = next_item["y"] - current_item["y"]

        if current_item["pad_index"] == next_item["pad_index"]:
            continue

        if abs(dx) > tolerance and abs(dy) > tolerance:
            corner_x, corner_y = choose_corner_xy(items, i)
            corner = make_corner_path_item(current_item, next_item, corner_x, corner_y)
            planned.append(corner)
            inserted += 1
            log("Inserted travel-only corner waypoint between pad %d and pad %d at X=%.4f Y=%.4f" %
                (current_item["pad_index"], next_item["pad_index"], corner_x, corner_y))

    planned.append(items[-1])
    return planned, inserted


def is_real_dispense_item(item):
    return item is not None and item.get("type") == "pad"


def find_last_real_dispense_path_index(planned_path):
    for i in range(len(planned_path) - 1, -1, -1):
        if is_real_dispense_item(planned_path[i]):
            return i
    return -1


def find_first_real_dispense_path_index(planned_path):
    for i in range(len(planned_path)):
        if is_real_dispense_item(planned_path[i]):
            return i
    return -1


def find_previous_motion_item(planned_path, current_index):
    for i in range(current_index - 1, -1, -1):
        item = planned_path[i]
        if item is not None and item.get("type") in ["pad", "corner"]:
            return item
    return None


def make_final_closing_drag_points(planned_path, current_index,
                                   first_real_index, units):
    if current_index < 0 or first_real_index < 0:
        return []

    current_item = planned_path[current_index]
    first_item = planned_path[first_real_index]

    if current_item is None or first_item is None:
        return []

    if current_item.get("type") != "pad" or first_item.get("type") != "pad":
        return []

    points = []

    dx = first_item["x"] - current_item["x"]
    dy = first_item["y"] - current_item["y"]

    tolerance = abs(mm_to_units(CORNER_INSERT_TOLERANCE_MM, units))

    if abs(dx) > tolerance and abs(dy) > tolerance:
        previous_item = find_previous_motion_item(planned_path, current_index)

        if previous_item is not None:
            if mostly_horizontal(previous_item, current_item):
                corner_x = first_item["x"]
                corner_y = current_item["y"]
            elif mostly_vertical(previous_item, current_item):
                corner_x = current_item["x"]
                corner_y = first_item["y"]
            else:
                corner_x = first_item["x"]
                corner_y = current_item["y"]
        else:
            corner_x = first_item["x"]
            corner_y = current_item["y"]

        points.append({
            "type": "final_closing_corner",
            "x": corner_x,
            "y": corner_y
        })

    points.append({
        "type": "final_closing_first_location",
        "x": first_item["x"],
        "y": first_item["y"]
    })

    return points


def dispense_dot_at_current_z(glue, machine_obj, pad_index, pad_name,
                              dot_index, total_dots, x, y, z):
    dot_number = int(dot_index) + 1
    log("Pad %d '%s' dot %d/%d at world X=%.4f Y=%.4f Z=%.4f" %
        (pad_index, pad_name, dot_number, total_dots, x, y, z))

    sleep_ms(PRE_DISPENSE_DWELL_MS)

    log_t("pad %d dot %d/%d: BEFORE pulse_glue_controller" %
          (pad_index, dot_number, total_dots))
    pulse_glue_controller(glue, machine_obj, pad_index)
    log_t("pad %d dot %d/%d: AFTER pulse_glue_controller" %
          (pad_index, dot_number, total_dots))


# ----------------- MAIN TEST FUNCTION -----------------

def dispense_part_pads_from_current_needle():
    if "machine" not in globals():
        raise Exception("Missing global: machine")

    mac = machine

    dispense_head = find_dispense_head(mac)
    glue = find_glue_dispense_actuator(mac)

    part = find_part_by_id(PART_ID)
    pkg = part.getPackage()

    if pkg is None:
        raise Exception("Part has no package: %s" % PART_ID)

    footprint = pkg.getFootprint()

    if footprint is None:
        raise Exception("Part package has no footprint: %s" % PART_ID)

    pads = footprint.getPads()

    if pads is None or pads.size() == 0:
        raise Exception("Footprint has no pads: %s" % PART_ID)

    center = dispense_head.getLocation()

    units = center.getUnits()
    center_x = center.getX()
    center_y = center.getY()
    surface_z = center.getZ()

    base_z = surface_z + DISPENSE_Z_OFFSET_MM
    lift_z = base_z + INTER_PAD_LIFT_MM

    footprint_units = get_footprint_units(footprint, units)

    rot_rad = PART_ROTATION_DEG * Math.PI / 180.0
    c = Math.cos(rot_rad)
    s = Math.sin(rot_rad)

    log("Part ID: %s" % PART_ID)
    log("Footprint pads: %d" % pads.size())
    log("Current needle center: X=%.4f Y=%.4f surfaceZ=%.4f units=%s" %
        (center_x, center_y, surface_z, str(units)))
    log("Dispense baseZ=%.4f liftZ=%.4f rotation=%.4f deg" %
        (base_z, lift_z, PART_ROTATION_DEG))
    log("Using dispense head '%s' and actuator '%s'." %
        (object_name(dispense_head), object_name(glue)))
    log("R axis is blocked with R=NaN. Moves use SuppressDynamicCompensation.")

    pad_positions = []

    for i in range(pads.size()):
        pad = pads.get(i)
        pad_name = str(pad.getName())

        if pad_name in PAD_SKIP_NAMES:
            log("Skipping pad %d named %s." % (i, pad_name))
            continue

        px_raw, py_raw = pad_xy(pad)

        # Convert pad offset into current dispense-head units.
        pad_offset_loc = Location(
            footprint_units,
            px_raw,
            py_raw,
            0.0,
            0.0
        )

        pad_offset_units = pad_offset_loc.convertToUnits(units)

        px = pad_offset_units.getX()
        py = pad_offset_units.getY()

        # Rotate footprint pad offset around current needle center.
        wx = center_x + (px * c - py * s)
        wy = center_y + (px * s + py * c)

        log("Pad %d '%s': footprint X=%.4f Y=%.4f -> offset X=%.4f Y=%.4f -> world X=%.4f Y=%.4f" %
            (i, pad_name, px_raw, py_raw, px, py, wx, wy))

        pad_positions.append(make_pad_path_item(
            i,
            pad_name,
            wx,
            wy,
            0,
            1))

    real_dot_items = expand_first_pad_dots(pad_positions, units)
    planned_path, inserted_corner_count = insert_corner_waypoints(real_dot_items, units)
    last_real_dispense_path_index = find_last_real_dispense_path_index(planned_path)
    first_real_dispense_path_index = find_first_real_dispense_path_index(planned_path)

    log("Planned dispense path: original non-skipped pads=%d real dots=%d travel-only corners=%d total items=%d" %
        (len(pad_positions), len(real_dot_items), inserted_corner_count,
         len(planned_path)))

    set_boolean_actuator_state_fast(glue, False, "glue initial off")
    wait_after_actuator_change(
        glue,
        mac,
        ACTUATOR_SETTLE_MS,
        "glue initial off wait")

    # Since the script starts with the needle at the surface,
    # lift vertically first before any XY motion.
    move_z_only_no_dynamic_comp(
        dispense_head,
        units,
        lift_z,
        DISPENSE_SPEED_FACTOR,
        "initial lift from surface")
    wait_still(dispense_head, "initial lift from surface")

    dispensed = 0
    dispense_t0 = log_t("DISPENSE START path_items=%d" % len(planned_path))

    for path_index in range(len(planned_path)):
        item = planned_path[path_index]

        if item["type"] == "corner":
            log("Travel-only corner waypoint %d/%d between pad %d and pad %d at world X=%.4f Y=%.4f Z=%.4f" %
                (path_index + 1, len(planned_path), item["from_pad_index"],
                 item["to_pad_index"], item["x"], item["y"], lift_z))

            move_z_only_no_dynamic_comp(
                dispense_head,
                units,
                lift_z,
                DISPENSE_SPEED_FACTOR,
                "corner waypoint %d ensure lift Z-only" % path_index)
            wait_coord(dispense_head, "corner waypoint %d pre-XY" % path_index)

            move_direct_no_dynamic_comp(
                dispense_head,
                dispense_head_location(units, item["x"], item["y"], lift_z),
                TRAVEL_SPEED_FACTOR,
                "corner waypoint %d lift XY" % path_index)
            wait_still(dispense_head, "corner waypoint %d lift XY" % path_index)
            continue

        pad_index = item["pad_index"]
        pad_name = item["pad_name"]
        dot_number = int(item["dot_index"]) + 1
        total_dots = int(item["total_dots"])

        pad_t0 = log_t("pad %d dot %d/%d: START" %
                       (pad_index, dot_number, total_dots))

        # Move over dot at lift height.
        move_direct_no_dynamic_comp(
            dispense_head,
            dispense_head_location(units, item["x"], item["y"], lift_z),
            TRAVEL_SPEED_FACTOR,
            "dispense pad %d dot %d/%d lift XY" %
            (pad_index, dot_number, total_dots)
        )

        # Move down to dispense Z only.
        move_z_only_no_dynamic_comp(
            dispense_head,
            units,
            base_z,
            DISPENSE_SPEED_FACTOR,
            "dispense pad %d dot %d/%d down Z-only" %
            (pad_index, dot_number, total_dots)
        )

        log_t("pad %d dot %d/%d: BEFORE pre-glue wait_coord" %
              (pad_index, dot_number, total_dots))
        wait_coord(
            dispense_head,
            "dispense pad %d dot %d/%d pre-glue" %
            (pad_index, dot_number, total_dots))
        log_t("pad %d dot %d/%d: AFTER pre-glue wait_coord / AT DISPENSE Z" %
              (pad_index, dot_number, total_dots))

        dispense_dot_at_current_z(
            glue,
            mac,
            pad_index,
            pad_name,
            item["dot_index"],
            total_dots,
            item["x"],
            item["y"],
            base_z)

        is_last_real_dot = path_index == last_real_dispense_path_index

        if is_last_real_dot and FINAL_DRAG_AFTER_LAST_DOT:
            closing_points = make_final_closing_drag_points(
                planned_path,
                path_index,
                first_real_dispense_path_index,
                units
            )

            if len(closing_points) == 0:
                log("Final closing drag skipped for pad %d dot %d/%d because no valid first/last dispense path was available." %
                    (pad_index, dot_number, total_dots))
            else:
                log_t("pad %d dot %d/%d: BEFORE final closing drag at dispense Z points=%d" %
                      (pad_index, dot_number, total_dots,
                       len(closing_points)))

                for close_index in range(len(closing_points)):
                    point = closing_points[close_index]

                    move_direct_no_dynamic_comp(
                        dispense_head,
                        dispense_head_location(
                            units,
                            point["x"],
                            point["y"],
                            base_z
                        ),
                        DISPENSE_SPEED_FACTOR,
                        "dispense pad %d dot %d/%d final closing drag %d/%d %s at dispense Z" %
                        (
                            pad_index,
                            dot_number,
                            total_dots,
                            close_index + 1,
                            len(closing_points),
                            point["type"]
                        )
                    )

                    wait_coord(
                        dispense_head,
                        "dispense pad %d dot %d/%d final closing drag %d/%d %s at dispense Z" %
                        (
                            pad_index,
                            dot_number,
                            total_dots,
                            close_index + 1,
                            len(closing_points),
                            point["type"]
                        )
                    )

                log_t("pad %d dot %d/%d: AFTER final closing drag at dispense Z" %
                      (pad_index, dot_number, total_dots))

        # Retract Z only. Completion is drained before next glue pulse.
        log_t("pad %d dot %d/%d: BEFORE retract queue" %
              (pad_index, dot_number, total_dots))
        move_z_only_no_dynamic_comp(
            dispense_head,
            units,
            lift_z,
            DISPENSE_SPEED_FACTOR,
            "dispense pad %d dot %d/%d retract Z-only" %
            (pad_index, dot_number, total_dots)
        )
        log_t("pad %d dot %d/%d: AFTER retract queue" %
              (pad_index, dot_number, total_dots))

        log_t("pad %d dot %d/%d: END total elapsed=%.3f ms" %
              (pad_index, dot_number, total_dots,
               elapsed_ms_since(pad_t0)))

        dispensed += 1

    log_t("DISPENSE BEFORE final queue drain")
    wait_coord(dispense_head, "dispense final queue drain")
    log_t("DISPENSE AFTER final queue drain")

    set_boolean_actuator_state_fast(glue, False, "glue final off")
    wait_after_actuator_change(
        glue,
        mac,
        ACTUATOR_SETTLE_MS,
        "glue final off wait")

    if MOVE_TO_SAFE_Z_AT_END:
        end = dispense_head_location(
            units,
            center_x,
            center_y,
            DISPENSE_SAFE_Z_MM
        )

        move_direct_no_dynamic_comp(
            dispense_head,
            end,
            TRAVEL_SPEED_FACTOR,
            "dispense head safe end at original center")
        wait_still(dispense_head, "dispense head safe end")

    log_t("DISPENSE END total elapsed=%.3f ms dispensed=%d" %
          (elapsed_ms_since(dispense_t0), dispensed))

    log("Dispense complete: original pads=%d real dispense dots=%d inserted travel-only corner waypoints=%d total planned path items=%d." %
        (len(pad_positions), dispensed, inserted_corner_count,
         len(planned_path)))

    return dispensed


# ----------------- RUN -----------------

class DispenseTask(Callable):
    def call(self):
        try:
            return dispense_part_pads_from_current_needle()
        except Exception, e:
            log_err(e)
            try:
                force_glue_off_safe(machine, "script error")
            except:
                pass
            raise


future = machine.submit(DispenseTask())

try:
    result = future.get()
    log("Machine task finished. Dispensed dots=%s" % str(result))

except Exception, e:
    log_err("Machine task failed: %s" % e)
    raise
