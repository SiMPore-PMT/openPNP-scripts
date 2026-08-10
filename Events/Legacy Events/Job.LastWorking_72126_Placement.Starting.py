# Job.Placement.Starting.py
#
# Combined board correction + adhesive dispense flow:
#   1. Measure board fiducial before dispense.
#   2. Temporarily apply that corrected board pose and dispense footprint pads.
#   3. Measure board fiducial after dispense and update boardLocation.
#   4. Return normally so OpenPnP continues the standard job flow.

from jarray import array
from java.lang import Boolean, Class, Double, Math, Object, System, Thread
from org.openpnp.model import Location, LengthUnit
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import MovableUtils, Utils2D
from org.opencv.core import RotatedRect
from org.openpnp.spi.Locatable import LocationOption


# ----------------- FLOW CONFIG -----------------
SKIP_DISPENSE = False
DRY_SECOND_PASS = True
POST_DISPENSE_IMAGE_CAPTURE_ENABLED = False
POST_DISPENSE_IMAGING_DELAY_MS = 250 #1000


# ----------------- VISION CONFIG -----------------
MAX_NUDGE_MM           = 4.0
APPLY_ROTATION         = True
EXPECTED_ANGLE_DEG     = 0.0
DESIGN_BOARD_ROT_DEG   = 0.0

PRE_CAPTURE_SETTLE_MS  = 250
RESULT_STAGE_KEY       = "results"
MAX_ATTEMPTS           = 3
RETRY_DELAY_MS         = 250
VISION_MEASUREMENT_ATTEMPTS = 2
RECENTER_MAX_PASSES    = 3
RECENTER_TOLERANCE_MM  = 0.02

FLIP_IMAGE_X = False
FLIP_IMAGE_Y = True


# ----------------- DISPENSE CONFIG -----------------
DISPENSE_HEAD_NAME      = "Dispense Head"
DISPENSE_ACTUATOR_NAME  = "GlueDispense"

DISPENSE_Z_OFFSET_MM    = 0.6#we are floating right now... fix later 
INTER_PAD_LIFT_MM       = 0.05
DISPENSE_SAFE_Z_MM      = 25.0

TRAVEL_SPEED_FACTOR     = 1
DISPENSE_SPEED_FACTOR   = 1

PRE_DISPENSE_DWELL_MS   = 0
DISPENSE_PULSE_MS       = 10
POST_DISPENSE_DWELL_MS  = 0
ACTUATOR_SETTLE_MS      = 0

PAD_SKIP_NAMES          = set()

EXTRA_FIRST_PAD_DOTS = 2
EXTRA_FIRST_PAD_DOT_SPACING_MM = -0.30

PREVENT_CORNER_CUTTING = True
CORNER_INSERT_TOLERANCE_MM = 0.05

FINAL_DRAG_AFTER_LAST_DOT = True
FINAL_DRAG_DISTANCE_MM = 0.6

DAP_OFF_X_MM = 60.3
DAP_OFF_Y_MM = 170.6
DAP_OFF_Z_MM = 0.0
DAP_OFF_LIFT_MM = 2.0
DAP_OFF_TOUCHES = 3
DAP_OFF_RANDOM_DIAMETER_MM = 12.0

TAG = "Job.Placement.Starting"


def log(msg):
    print "[%s] %s" % (TAG, str(msg))


def log_err(msg):
    print "[%s ERROR] %s" % (TAG, str(msg))


class VisionMeasurementError(Exception):
    pass


SCRIPT_T0_NS = System.nanoTime()
LAST_T_NS = SCRIPT_T0_NS


def now_ms():
    return (System.nanoTime() - SCRIPT_T0_NS) / 1000000.0


def elapsed_ms_since(t0_ns):
    return (System.nanoTime() - t0_ns) / 1000000.0


def log_t(label):
    global LAST_T_NS
    now_ns = System.nanoTime()
    abs_ms = (now_ns - SCRIPT_T0_NS) / 1000000.0
    delta_ms = (now_ns - LAST_T_NS) / 1000000.0
    LAST_T_NS = now_ns
    log("[TIMER] %+9.3f ms | +%8.3f ms | %s" % (abs_ms, delta_ms, label))
    return now_ns


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
    t0 = log_t("%s: wait_still BEFORE WaitForStillstand" % label)
    movable.waitForCompletion(CompletionType.WaitForStillstand)
    log_t("%s: wait_still AFTER WaitForStillstand elapsed=%.3f ms" %
          (label, elapsed_ms_since(t0)))


def wait_coord(movable, label):
    t0 = log_t("%s: wait_coord BEFORE WaitForUnconditionalCoordination" % label)
    movable.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
    log_t("%s: wait_coord AFTER WaitForUnconditionalCoordination elapsed=%.3f ms" %
          (label, elapsed_ms_since(t0)))


def move_safe_wait(movable, target, label):
    MovableUtils.moveToLocationAtSafeZ(movable, target)
    wait_still(movable, label)


def move_direct(movable, target, speed, label):
    log_t("%s: move_direct BEFORE queue target=%s speed=%.3f" %
          (label, str(target), float(speed)))
    try:
        movable.moveTo(target, float(speed))
    except TypeError:
        movable.moveTo(target)
    log_t("%s: move_direct AFTER moveTo" % label)


def move_z_only(movable, units, z, speed, label):
    target = Location(units, Double.NaN, Double.NaN, z, Double.NaN)
    log_t("%s: move_z_only BEFORE target=%s speed=%.3f" %
          (label, str(target), float(speed)))
    try:
        movable.moveTo(target, float(speed))
    except TypeError:
        movable.moveTo(target)
    log_t("%s: move_z_only AFTER" % label)


def set_boolean_actuator_state_fast(actuator, on_off, label):
    value = Boolean.TRUE if bool(on_off) else Boolean.FALSE
    method = actuator.getClass().getMethod(
        "actuate",
        array([Boolean.TYPE], Class)
    )
    method.invoke(actuator, array([value], Object))
    log("%s boolean actuator=%s queued." % (label, str(bool(on_off))))


def find_rotated_rect_with_retries(pipeline):
    candidate_keys = [RESULT_STAGE_KEY, "results", "preResults"]
    for attempt in range(1, MAX_ATTEMPTS + 1):
        try:
            log("Vision attempt %d of %d" % (attempt, MAX_ATTEMPTS))
            pipeline.process()
            log("pipeline.process() OK on attempt %d" % attempt)

            for key in candidate_keys:
                try:
                    res = pipeline.getResult(key)
                except:
                    res = None
                if res is None:
                    continue

                model = res.getModel()
                if model is None:
                    continue

                if isinstance(model, RotatedRect):
                    log("Got RotatedRect directly from results key '%s'." % key)
                    return model

                if hasattr(model, "size") and hasattr(model, "get"):
                    if model.size() > 0:
                        first = model.get(0)
                        if isinstance(first, RotatedRect) or hasattr(first, "center"):
                            log("Got RotatedRect[0] from list at results key '%s'." % key)
                            return first

            log("No RotatedRect found on attempt %d." % attempt)
        except Exception, e:
            log("Exception during pipeline attempt %d: %s" % (attempt, e))

        if attempt < MAX_ATTEMPTS:
            Thread.sleep(RETRY_DELAY_MS)
    return None


def get_camera_image_size(cam):
    try:
        return cam.getWidth(), cam.getHeight()
    except:
        pass
    image = cam.settleAndCapture()
    return image.getWidth(), image.getHeight()


def get_camera_units_per_pixel(cam):
    if hasattr(cam, "getUnitsPerPixelAtZ"):
        upp_loc = cam.getUnitsPerPixelAtZ()
    else:
        upp_loc = cam.getUnitsPerPixel()
    if upp_loc is None:
        raise Exception("Camera units-per-pixel not configured.")
    return upp_loc


def extract_rotated_rects_from_model(model):
    rects = []
    if model is None:
        return rects
    if isinstance(model, RotatedRect) or hasattr(model, "center"):
        rects.append(model)
        return rects
    if hasattr(model, "size") and hasattr(model, "get"):
        for i in range(model.size()):
            item = model.get(i)
            if isinstance(item, RotatedRect) or hasattr(item, "center"):
                rects.append(item)
        return rects
    try:
        for item in model:
            if isinstance(item, RotatedRect) or hasattr(item, "center"):
                rects.append(item)
    except:
        pass
    return rects


def get_pipeline_rotated_rects(pipeline):
    rects = []
    for key in [RESULT_STAGE_KEY, "results", "preResults"]:
        try:
            res = pipeline.getResult(key)
        except:
            res = None
        if res is None:
            continue
        try:
            model = res.getModel()
        except:
            try:
                model = res.model
            except:
                model = None
        rects.extend(extract_rotated_rects_from_model(model))
    return rects


def choose_closest_rect_to_image_center(rects, width, height):
    best = None
    best_dist2 = None
    cx = width / 2.0
    cy = height / 2.0
    for rr in rects:
        dx = rr.center.x - cx
        dy = rr.center.y - cy
        dist2 = dx * dx + dy * dy
        if best is None or dist2 < best_dist2:
            best = rr
            best_dist2 = dist2
    return best, best_dist2


def rect_center_machine_offset(cam_loc, upp_loc, rr, width, height):
    px = rr.center.x - (width / 2.0)
    py = rr.center.y - (height / 2.0)
    if FLIP_IMAGE_X:
        px = -px
    if FLIP_IMAGE_Y:
        py = -py

    dx_cam = px * upp_loc.getX()
    dy_cam = py * upp_loc.getY()

    rad = cam_loc.getRotation() * Math.PI / 180.0
    c = Math.cos(rad)
    s = Math.sin(rad)
    dx = dx_cam * c - dy_cam * s
    dy = dx_cam * s + dy_cam * c
    return dx, dy, px, py


def location_xy_distance_mm(a, b):
    a_mm = a.convertToUnits(a.getUnits())
    b_units = a_mm.getUnits()
    b_mm = b.convertToUnits(b_units)
    dx = a_mm.getX() - b_mm.getX()
    dy = a_mm.getY() - b_mm.getY()
    return abs(dx), abs(dy)


def detection_inside_initial_view(initial_view_loc, detected_loc, upp_loc, width, height):
    dx, dy = location_xy_distance_mm(initial_view_loc, detected_loc)
    upp_mm = upp_loc.convertToUnits(initial_view_loc.getUnits())
    half_x = abs(upp_mm.getX()) * width / 2.0
    half_y = abs(upp_mm.getY()) * height / 2.0
    return dx <= half_x and dy <= half_y, dx, dy, half_x, half_y


def move_camera_to_detected_center(cam, detected_loc, label):
    cam_loc = cam.getLocation()
    detected_cam_units = detected_loc.convertToUnits(cam_loc.getUnits())
    target = Location(
        cam_loc.getUnits(),
        detected_cam_units.getX(),
        detected_cam_units.getY(),
        cam_loc.getZ(),
        cam_loc.getRotation()
    )
    move_safe_wait(cam, target, label)


def get_board_placements(board):
    for method in ["getPlacements", "getBoardPlacements"]:
        try:
            placements = getattr(board, method)()
            if placements is not None:
                log("Using board.%s()." % method)
                return placements
        except:
            pass
    raise Exception("Board placements list is null.")


def is_fiducial_placement(plc):
    try:
        if hasattr(plc, "isFiducial") and plc.isFiducial():
            return True
    except:
        pass
    try:
        if hasattr(plc, "getType"):
            t = plc.getType()
            if t is not None and "FIDUCIAL" in str(t).upper():
                return True
    except:
        pass
    return False


def is_normal_placement(plc):
    try:
        if hasattr(plc, "getType"):
            t = plc.getType()
            if t is not None and str(t).upper() == "PLACEMENT":
                return True
    except:
        pass
    return not is_fiducial_placement(plc)


def get_vision_settings_for_fiducial(plc):
    for method in ["getFiducialVisionSettings", "getVisionSettings"]:
        try:
            settings = getattr(plc, method)()
            if settings is not None:
                return settings
        except:
            pass
    try:
        p = plc.getPart()
        if p is not None and hasattr(p, "getFiducialVisionSettings"):
            settings = p.getFiducialVisionSettings()
            if settings is not None:
                return settings
    except:
        pass
    return None


def get_board_fiducial_placement_and_settings(board):
    placements = get_board_placements(board)
    count = len(java_list_to_py(placements))
    log("Board placements count: %d" % count)

    for i, plc in enumerate(java_list_to_py(placements)):
        try:
            if hasattr(plc, "isEnabled") and not plc.isEnabled():
                continue
        except:
            pass
        if not is_fiducial_placement(plc):
            continue
        settings = get_vision_settings_for_fiducial(plc)
        if settings is None:
            log("Fiducial placement index %d has no vision settings; skipping." % i)
            continue
        loc = plc.getLocation()
        label = object_name(plc) or "<unnamed>"
        log("Selected fiducial placement '%s' at index %d." % (label, i))
        return plc, loc, settings, label

    raise Exception("No enabled board fiducial placement with vision settings found.")


def get_dispense_placement(board, current_placement):
    try:
        if current_placement is not None and is_normal_placement(current_placement):
            if current_placement.getPart() is not None:
                return current_placement
    except:
        pass

    for plc in java_list_to_py(get_board_placements(board)):
        try:
            if hasattr(plc, "isEnabled") and not plc.isEnabled():
                continue
        except:
            pass
        if is_normal_placement(plc):
            try:
                if plc.getPart() is not None:
                    return plc
            except:
                pass
    raise Exception("No enabled normal placement with a part found for dispense.")


def make_board_location(bl_loc, x, y, z, r):
    return Location(bl_loc.getUnits(), x, y, z, r)


def prepare_pipeline_context(pipeline, cam, fid_placement, bl, prt):
    for name, value in [
        ("camera", cam),
        ("placement", fid_placement),
        ("boardLocation", bl),
    ]:
        try:
            pipeline.setProperty(name, value)
        except Exception, e:
            log("Could not set pipeline %s property: %s" % (name, e))

    try:
        p = None
        if hasattr(fid_placement, "getPart"):
            p = fid_placement.getPart()
        if p is None:
            p = prt
        if p is not None:
            pipeline.setProperty("part", p)
    except Exception, e:
        log("Could not set pipeline part property: %s" % e)


def measure_board_fiducial_correction(mac, bl, board, prt, label):
    fid_placement, fid_loc, settings, fid_label = \
        get_board_fiducial_placement_and_settings(board)

    log("[%s] Using BOARD fiducial '%s' design location: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
        (label, fid_label, fid_loc.getX(), fid_loc.getY(),
         fid_loc.getZ(), fid_loc.getRotation()))

    pipeline = settings.getPipeline()
    if pipeline is None:
        raise Exception("Fiducial vision settings pipeline is null.")

    bl_loc = bl.getLocation()
    p_loc = Utils2D.calculateBoardPlacementLocation(bl, fid_loc)
    head = mac.getDefaultHead()
    cam = head.getDefaultCamera()
    if cam is None:
        raise Exception("Default head camera is null.")

    prepare_pipeline_context(pipeline, cam, fid_placement, bl, prt)

    base_cam_loc = cam.getLocation()
    cam_target = Location(
        p_loc.getUnits(),
        p_loc.getX(),
        p_loc.getY(),
        base_cam_loc.getZ(),
        base_cam_loc.getRotation()
    )

    last_reject = "not attempted"
    for measurement_attempt in range(1, VISION_MEASUREMENT_ATTEMPTS + 1):
        if measurement_attempt > 1:
            Thread.sleep(RETRY_DELAY_MS)
        log("[%s] Measurement attempt %d of %d." %
            (label, measurement_attempt, VISION_MEASUREMENT_ATTEMPTS))

        move_safe_wait(
            cam, cam_target,
            "%s camera nominal attempt %d" % (label, measurement_attempt))
        initial_view_loc = cam_target

        for recenter_pass in range(1, RECENTER_MAX_PASSES + 1):
            wait_still(
                cam,
                "%s recenter pass %d camera" % (label, recenter_pass))
            if PRE_CAPTURE_SETTLE_MS > 0:
                Thread.sleep(PRE_CAPTURE_SETTLE_MS)
                log("[%s] Recenter pass %d settled %d ms before pipeline.process()." %
                    (label, recenter_pass, PRE_CAPTURE_SETTLE_MS))

            try:
                pipeline.process()
                log("[%s] pipeline.process() OK attempt=%d pass=%d" %
                    (label, measurement_attempt, recenter_pass))
            except Exception, e:
                last_reject = "pipeline exception: %s" % e
                log("[%s] Rejected vision pass: %s" % (label, last_reject))
                break

            rects = get_pipeline_rotated_rects(pipeline)
            if len(rects) == 0:
                last_reject = "no RotatedRect results"
                log("[%s] Rejected vision pass: %s" % (label, last_reject))
                break

            width, height = get_camera_image_size(cam)
            rr, dist2 = choose_closest_rect_to_image_center(rects, width, height)
            if rr is None:
                last_reject = "no usable RotatedRect results"
                log("[%s] Rejected vision pass: %s" % (label, last_reject))
                break

            cam_loc = cam.getLocation()
            upp_loc = get_camera_units_per_pixel(cam)
            center_dx, center_dy, px, py = rect_center_machine_offset(
                cam_loc, upp_loc, rr, width, height)

            # Match the original placement-starting correction convention:
            # the measured machine-frame offset is applied as p_loc + dx/dy.
            detected_loc = Location(
                cam_loc.getUnits(),
                cam_loc.getX() + center_dx,
                cam_loc.getY() + center_dy,
                cam_loc.getZ(),
                cam_loc.getRotation()
            )
            p_loc_bl_units = p_loc.convertToUnits(bl_loc.getUnits())
            detected_bl_units = detected_loc.convertToUnits(bl_loc.getUnits())
            board_dx = detected_bl_units.getX() - p_loc_bl_units.getX()
            board_dy = detected_bl_units.getY() - p_loc_bl_units.getY()

            log("[%s] attempt=%d pass=%d rects=%d chosen px=%.3f py=%.3f centered dx=%.6f dy=%.6f board dx=%.6f dy=%.6f angle=%.4f" %
                (label, measurement_attempt, recenter_pass, len(rects),
                 px, py, center_dx, center_dy, board_dx, board_dy, rr.angle))

            inside, view_dx, view_dy, half_x, half_y = \
                detection_inside_initial_view(
                    initial_view_loc, detected_loc, upp_loc, width, height)
            if not inside:
                last_reject = (
                    "outside initial view dx=%.6f/%.6f dy=%.6f/%.6f" %
                    (view_dx, half_x, view_dy, half_y))
                log("[%s] Rejected vision pass: %s" % (label, last_reject))
                break

            if abs(board_dx) > MAX_NUDGE_MM or abs(board_dy) > MAX_NUDGE_MM:
                last_reject = (
                    "beyond MAX_NUDGE_MM board dx=%.6f dy=%.6f limit=%.6f" %
                    (board_dx, board_dy, MAX_NUDGE_MM))
                log("[%s] Rejected vision pass: %s" % (label, last_reject))
                break

            if (abs(center_dx) <= RECENTER_TOLERANCE_MM and
                    abs(center_dy) <= RECENTER_TOLERANCE_MM):
                new_rot = bl_loc.getRotation()
                if APPLY_ROTATION:
                    new_rot = DESIGN_BOARD_ROT_DEG + (EXPECTED_ANGLE_DEG - rr.angle)
                    log("[%s] Rotation newBoard=%.4f from measured=%.4f" %
                        (label, new_rot, rr.angle))

                new_bl = make_board_location(
                    bl_loc,
                    bl_loc.getX() + board_dx,
                    bl_loc.getY() + board_dy,
                    bl_loc.getZ(),
                    new_rot
                )

                log("[%s] Accepted centered fiducial correction -> board X=%.4f Y=%.4f R=%.4f" %
                    (label, new_bl.getX(), new_bl.getY(), new_bl.getRotation()))

                return {
                    "boardLocation": new_bl,
                    "dx": board_dx,
                    "dy": board_dy,
                    "rotation": new_rot,
                    "angle": rr.angle,
                    "recenterPass": recenter_pass,
                    "measurementAttempt": measurement_attempt,
                }

            last_reject = (
                "not centered dx=%.6f dy=%.6f tolerance=%.6f" %
                (center_dx, center_dy, RECENTER_TOLERANCE_MM))
            if recenter_pass < RECENTER_MAX_PASSES:
                log("[%s] Recenter pass %d moving camera to detected center: X=%.4f Y=%.4f" %
                    (label, recenter_pass,
                     detected_loc.getX(), detected_loc.getY()))
                move_camera_to_detected_center(
                    cam, detected_loc,
                    "%s recenter pass %d" % (label, recenter_pass))
            else:
                log("[%s] Rejected vision attempt: %s" % (label, last_reject))

        log("[%s] Measurement attempt %d failed: %s" %
            (label, measurement_attempt, last_reject))

    raise VisionMeasurementError(
        "[%s] Board fiducial not confirmed after %d attempts; last reject: %s" %
        (label, VISION_MEASUREMENT_ATTEMPTS, last_reject))


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


def pad_xy(pad):
    try:
        return pad.getCenterX(), pad.getCenterY()
    except:
        return pad.getX(), pad.getY()


def dispense_head_location(units, x, y, z):
    return Location(units, x, y, z, Double.NaN)


def mm_to_units(value_mm, units):
    loc = Location(LengthUnit.Millimeters, float(value_mm), 0.0, 0.0, 0.0)
    return loc.convertToUnits(units).getX()


def make_pad_path_item(pad_index, pad_name, x, y, dot_index, total_dots,
                       local_x=None, local_y=None):
    return {
        "type": "pad",
        "pad_index": pad_index,
        "pad_name": pad_name,
        "x": x,
        "y": y,
        "local_x": local_x,
        "local_y": local_y,
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
                1,
                item.get("local_x"),
                item.get("local_y")))
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
            total_dots,
            first.get("local_x"),
            first.get("local_y")))

    for i in range(1, len(pad_positions)):
        item = pad_positions[i]
        out.append(make_pad_path_item(
            item["pad_index"],
            item["pad_name"],
            item["x"],
            item["y"],
            0,
            1,
            item.get("local_x"),
            item.get("local_y")))

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


def path_bounds(items):
    xs = []
    ys = []

    for item in items:
        if item is not None and item.get("type") == "pad":
            xs.append(item["x"])
            ys.append(item["y"])

    if len(xs) == 0:
        return None

    return {
        "min_x": min(xs),
        "max_x": max(xs),
        "min_y": min(ys),
        "max_y": max(ys)
    }


def near_value(a, b, tol):
    return abs(a - b) <= tol


def point_edge_flags(item, bounds, tol):
    return {
        "left": near_value(item["x"], bounds["min_x"], tol),
        "right": near_value(item["x"], bounds["max_x"], tol),
        "bottom": near_value(item["y"], bounds["min_y"], tol),
        "top": near_value(item["y"], bounds["max_y"], tol)
    }


def find_previous_real_pad_item(items, current_index):
    current_item = items[current_index]
    current_pad_index = current_item.get("pad_index")

    for i in range(current_index - 1, -1, -1):
        item = items[i]
        if item is None or item.get("type") != "pad":
            continue
        if item.get("pad_index") != current_pad_index:
            return item

    return None


def find_next_real_pad_item(items, current_index):
    current_item = items[current_index]
    current_pad_index = current_item.get("pad_index")

    for i in range(current_index + 1, len(items)):
        item = items[i]
        if item is None or item.get("type") != "pad":
            continue
        if item.get("pad_index") != current_pad_index:
            return item

    return None


def choose_corner_xy(items, index, units):
    current_item = items[index]
    next_item = items[index + 1]

    tolerance = abs(mm_to_units(CORNER_INSERT_TOLERANCE_MM, units))
    if tolerance <= 0.0:
        tolerance = 1.0e-6

    bounds = path_bounds(items)

    if bounds is not None:
        current_edges = point_edge_flags(current_item, bounds, tolerance)
        next_edges = point_edge_flags(next_item, bounds, tolerance)

        # Vertical edge -> horizontal edge:
        # keep moving vertically first, then turn horizontally.
        #
        # Right edge going up to top edge:
        #   current on right, next on top
        #   corner = current.x, next.y
        #   motion = up, then left
        if ((current_edges["left"] or current_edges["right"]) and
                (next_edges["top"] or next_edges["bottom"])):
            return current_item["x"], next_item["y"]

        # Horizontal edge -> vertical edge:
        # keep moving horizontally first, then turn vertically.
        #
        # Top edge going left to left edge:
        #   current on top, next on left
        #   corner = next.x, current.y
        #   motion = left, then down
        if ((current_edges["top"] or current_edges["bottom"]) and
                (next_edges["left"] or next_edges["right"])):
            return next_item["x"], current_item["y"]

    # Fallback: preserve the incoming tangent direction using only real pad
    # items. Do not use inserted corner items for this decision.
    previous_item = find_previous_real_pad_item(items, index)

    if previous_item is not None:
        if mostly_horizontal(previous_item, current_item):
            return next_item["x"], current_item["y"]
        if mostly_vertical(previous_item, current_item):
            return current_item["x"], next_item["y"]

    # If no previous real pad exists, infer from the next segment.
    after_next_item = find_next_real_pad_item(items, index + 1)

    if after_next_item is not None:
        if mostly_horizontal(next_item, after_next_item):
            return current_item["x"], next_item["y"]
        if mostly_vertical(next_item, after_next_item):
            return next_item["x"], current_item["y"]

    return next_item["x"], current_item["y"]


def should_insert_corner_waypoint(current_item, next_item, units):
    if current_item["pad_index"] == next_item["pad_index"]:
        return False

    local_x0 = current_item.get("local_x")
    local_y0 = current_item.get("local_y")
    local_x1 = next_item.get("local_x")
    local_y1 = next_item.get("local_y")

    # Decide from footprint-local coordinates when available. Same physical
    # footprint edge keeps one local axis unchanged even after board rotation.
    if (local_x0 is not None and local_y0 is not None and
            local_x1 is not None and local_y1 is not None):
        local_dx = local_x1 - local_x0
        local_dy = local_y1 - local_y0
        local_tol = abs(CORNER_INSERT_TOLERANCE_MM)

        return abs(local_dx) > local_tol and abs(local_dy) > local_tol

    dx = next_item["x"] - current_item["x"]
    dy = next_item["y"] - current_item["y"]
    tolerance = abs(mm_to_units(CORNER_INSERT_TOLERANCE_MM, units))

    return abs(dx) > tolerance and abs(dy) > tolerance


def insert_corner_waypoints(items, units):
    if not PREVENT_CORNER_CUTTING:
        return items, 0

    if len(items) < 2:
        return items, 0

    planned = []
    inserted = 0

    for i in range(len(items) - 1):
        current_item = items[i]
        next_item = items[i + 1]
        planned.append(current_item)

        if should_insert_corner_waypoint(current_item, next_item, units):
            corner_x, corner_y = choose_corner_xy(items, i, units)
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


def find_previous_real_dispense_item(planned_path, current_index):
    current_item = planned_path[current_index]
    current_pad_index = current_item.get("pad_index")

    for i in range(current_index - 1, -1, -1):
        item = planned_path[i]
        if item is None or item.get("type") != "pad":
            continue
        if item.get("pad_index") != current_pad_index:
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
    if tolerance <= 0.0:
        tolerance = 1.0e-6

    if abs(dx) > tolerance and abs(dy) > tolerance:
        bounds = path_bounds(planned_path)
        corner_x = None
        corner_y = None

        if bounds is not None:
            current_edges = point_edge_flags(current_item, bounds, tolerance)
            first_edges = point_edge_flags(first_item, bounds, tolerance)

            # Vertical edge -> horizontal edge:
            # continue vertical first.
            if ((current_edges["left"] or current_edges["right"]) and
                    (first_edges["top"] or first_edges["bottom"])):
                corner_x = current_item["x"]
                corner_y = first_item["y"]

            # Horizontal edge -> vertical edge:
            # continue horizontal first.
            #
            # Top edge closing to first pad on left edge:
            #   corner = first.x, current.y
            #   motion = left, then down
            elif ((current_edges["top"] or current_edges["bottom"]) and
                    (first_edges["left"] or first_edges["right"])):
                corner_x = first_item["x"]
                corner_y = current_item["y"]

        if corner_x is None or corner_y is None:
            previous_item = find_previous_real_dispense_item(
                planned_path,
                current_index
            )

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


def sleep_ms(ms):
    ms = int(ms)
    if ms > 0:
        Thread.sleep(ms)


def wait_after_actuator_change(actuator, machine_obj, settle_ms, label):
    if actuator is not None:
        try:
            actuator.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
            log_t("%s: actuator wait complete" % label)
            return
        except:
            pass
    if machine_obj is not None:
        try:
            machine_obj.getMotionPlanner().waitForCompletion(
                None, CompletionType.WaitForUnconditionalCoordination)
            log_t("%s: motion planner wait complete" % label)
            return
        except:
            pass
    sleep_ms(settle_ms)
    log_t("%s: fallback sleep %d ms complete" % (label, int(settle_ms)))


def pulse_glue_controller(actuator, machine_obj, pad_index):
    t0 = log_t("pad %d: pulse_glue_controller START boolean pulse=%d ms settle=%d ms post=%d ms" %
               (pad_index, DISPENSE_PULSE_MS, ACTUATOR_SETTLE_MS, POST_DISPENSE_DWELL_MS))

    set_boolean_actuator_state_fast(actuator, False, "pad %d GlueDispense OFF before pulse" % pad_index)
    wait_after_actuator_change(
        actuator, machine_obj, ACTUATOR_SETTLE_MS,
        "pad %d GlueDispense OFF wait" % pad_index)
    sleep_ms(ACTUATOR_SETTLE_MS)

    set_boolean_actuator_state_fast(actuator, True, "pad %d GlueDispense ON" % pad_index)
    wait_after_actuator_change(
        actuator, machine_obj, ACTUATOR_SETTLE_MS,
        "pad %d GlueDispense ON wait" % pad_index)
    sleep_ms(DISPENSE_PULSE_MS)

    set_boolean_actuator_state_fast(actuator, False, "pad %d GlueDispense OFF after pulse" % pad_index)
    wait_after_actuator_change(
        actuator, machine_obj, ACTUATOR_SETTLE_MS,
        "pad %d GlueDispense final OFF wait" % pad_index)
    sleep_ms(ACTUATOR_SETTLE_MS)
    sleep_ms(POST_DISPENSE_DWELL_MS)

    log_t("pad %d: pulse_glue_controller END host elapsed=%.3f ms" %
          (pad_index, elapsed_ms_since(t0)))


def force_glue_off_safe(machine_obj, label):
    if machine_obj is None:
        try:
            machine_obj = machine
        except:
            machine_obj = None
    if machine_obj is None:
        return
    try:
        glue = find_glue_dispense_actuator(machine_obj)
        set_boolean_actuator_state_fast(glue, False, "%s GlueDispense forced off" % label)
        wait_after_actuator_change(
            glue, machine_obj, ACTUATOR_SETTLE_MS,
            "%s GlueDispense forced off wait" % label)
    except Exception, e:
        log_err("%s failed to force GlueDispense off: %s" % (label, e))
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
        log_err("Could not resolve LocationOption.%s: %s" % (option_name, e))

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


def solve_3x3(matrix, values):
    a = [
        [float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(values[0])],
        [float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(values[1])],
        [float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(values[2])]
    ]

    for col in range(3):
        pivot = col
        pivot_abs = abs(a[col][col])
        for row in range(col + 1, 3):
            row_abs = abs(a[row][col])
            if row_abs > pivot_abs:
                pivot = row
                pivot_abs = row_abs

        if pivot_abs <= 1.0e-12:
            raise Exception("Singular 3x3 system.")

        if pivot != col:
            tmp = a[col]
            a[col] = a[pivot]
            a[pivot] = tmp

        pivot_value = a[col][col]
        for j in range(col, 4):
            a[col][j] = a[col][j] / pivot_value

        for row in range(3):
            if row == col:
                continue
            factor = a[row][col]
            for j in range(col, 4):
                a[row][j] = a[row][j] - factor * a[col][j]

    return [a[0][3], a[1][3], a[2][3]]


def fit_local_to_world_affine(pad_positions):
    n = 0
    sx = 0.0
    sy = 0.0
    sxx = 0.0
    sxy = 0.0
    syy = 0.0
    swx = 0.0
    swy = 0.0
    sxwx = 0.0
    sywx = 0.0
    sxwy = 0.0
    sywy = 0.0
    min_local_x = None
    max_local_x = None
    min_local_y = None
    max_local_y = None

    for item in pad_positions:
        local_x = item.get("local_x")
        local_y = item.get("local_y")
        if local_x is None or local_y is None:
            continue

        lx = float(local_x)
        ly = float(local_y)
        wx = float(item["x"])
        wy = float(item["y"])

        n += 1
        sx += lx
        sy += ly
        sxx += lx * lx
        sxy += lx * ly
        syy += ly * ly
        swx += wx
        swy += wy
        sxwx += lx * wx
        sywx += ly * wx
        sxwy += lx * wy
        sywy += ly * wy

        if min_local_x is None or lx < min_local_x:
            min_local_x = lx
        if max_local_x is None or lx > max_local_x:
            max_local_x = lx
        if min_local_y is None or ly < min_local_y:
            min_local_y = ly
        if max_local_y is None or ly > max_local_y:
            max_local_y = ly

    if n < 3:
        raise Exception("Need at least three pads with local coordinates for dry square corners.")

    normal = [
        [float(n), sx, sy],
        [sx, sxx, sxy],
        [sy, sxy, syy]
    ]

    coeff_x = solve_3x3(normal, [swx, sxwx, sywx])
    coeff_y = solve_3x3(normal, [swy, sxwy, sywy])

    return {
        "coeff_x": coeff_x,
        "coeff_y": coeff_y,
        "min_local_x": min_local_x,
        "max_local_x": max_local_x,
        "min_local_y": min_local_y,
        "max_local_y": max_local_y
    }


def affine_local_to_world(transform, local_x, local_y):
    cx = transform["coeff_x"]
    cy = transform["coeff_y"]
    return {
        "x": cx[0] + cx[1] * local_x + cx[2] * local_y,
        "y": cy[0] + cy[1] * local_x + cy[2] * local_y
    }


def make_dry_square_corners(pad_positions):
    transform = fit_local_to_world_affine(pad_positions)

    min_x = transform["min_local_x"]
    max_x = transform["max_local_x"]
    min_y = transform["min_local_y"]
    max_y = transform["max_local_y"]

    corners = [
        ("bottom-left", min_x, min_y),
        ("bottom-right", max_x, min_y),
        ("top-right", max_x, max_y),
        ("top-left", min_x, max_y)
    ]

    out = []
    for name, local_x, local_y in corners:
        world = affine_local_to_world(transform, local_x, local_y)
        out.append({
            "name": name,
            "local_x": local_x,
            "local_y": local_y,
            "x": world["x"],
            "y": world["y"]
        })

    return out


def run_dispense_path_pass(mac, dispense_head, glue, planned_path, units,
                           base_z, lift_z, last_real_dispense_path_index,
                           first_real_dispense_path_index, timer_label,
                           motion_label):
    dispensed = 0
    pass_t0 = log_t("%s START path_items=%d" %
                    (timer_label, len(planned_path)))

    for path_index in range(len(planned_path)):
        item = planned_path[path_index]

        if item["type"] == "corner":
            log("%s travel-only corner waypoint %d/%d between pad %d and pad %d at world X=%.4f Y=%.4f Z=%.4f" %
                (motion_label, path_index + 1, len(planned_path),
                 item["from_pad_index"], item["to_pad_index"],
                 item["x"], item["y"], lift_z))

            move_z_only_no_dynamic_comp(
                dispense_head,
                units,
                lift_z,
                DISPENSE_SPEED_FACTOR,
                "%s corner waypoint %d ensure lift Z-only" %
                (motion_label, path_index))
            wait_coord(
                dispense_head,
                "%s corner waypoint %d pre-XY" %
                (motion_label, path_index))

            move_direct_no_dynamic_comp(
                dispense_head,
                dispense_head_location(units, item["x"], item["y"], lift_z),
                TRAVEL_SPEED_FACTOR,
                "%s corner waypoint %d lift XY" %
                (motion_label, path_index))
            wait_still(
                dispense_head,
                "%s corner waypoint %d lift XY" %
                (motion_label, path_index))
            continue

        pad_index = item["pad_index"]
        pad_name = item["pad_name"]
        dot_number = int(item["dot_index"]) + 1
        total_dots = int(item["total_dots"])

        pad_t0 = log_t("%s pad %d dot %d/%d: START" %
                       (motion_label, pad_index, dot_number, total_dots))

        move_direct_no_dynamic_comp(
            dispense_head,
            dispense_head_location(units, item["x"], item["y"], lift_z),
            TRAVEL_SPEED_FACTOR,
            "%s pad %d dot %d/%d lift XY" %
            (motion_label, pad_index, dot_number, total_dots)
        )

        move_z_only_no_dynamic_comp(
            dispense_head,
            units,
            base_z,
            DISPENSE_SPEED_FACTOR,
            "%s pad %d dot %d/%d down Z-only" %
            (motion_label, pad_index, dot_number, total_dots)
        )

        log_t("%s pad %d dot %d/%d: BEFORE pre-glue wait_coord" %
              (motion_label, pad_index, dot_number, total_dots))
        wait_coord(
            dispense_head,
            "%s pad %d dot %d/%d pre-glue" %
            (motion_label, pad_index, dot_number, total_dots))
        log_t("%s pad %d dot %d/%d: AFTER pre-glue wait_coord / AT DISPENSE Z" %
              (motion_label, pad_index, dot_number, total_dots))

        log("%s head reached pad %d '%s' dot %d/%d at dispense Z." %
            (motion_label, pad_index, pad_name, dot_number, total_dots))

        sleep_ms(PRE_DISPENSE_DWELL_MS)

        log_t("%s pad %d dot %d/%d: BEFORE pulse_glue_controller" %
              (motion_label, pad_index, dot_number, total_dots))
        pulse_glue_controller(glue, mac, pad_index)
        log_t("%s pad %d dot %d/%d: AFTER pulse_glue_controller" %
              (motion_label, pad_index, dot_number, total_dots))
        dispensed += 1

        is_last_real_dot = path_index == last_real_dispense_path_index

        if is_last_real_dot and FINAL_DRAG_AFTER_LAST_DOT:
            closing_points = make_final_closing_drag_points(
                planned_path,
                path_index,
                first_real_dispense_path_index,
                units
            )

            if len(closing_points) == 0:
                log("%s final closing drag skipped for pad %d dot %d/%d because no valid first/last dispense path was available." %
                    (motion_label, pad_index, dot_number, total_dots))
            else:
                log_t("%s pad %d dot %d/%d: BEFORE final closing drag at dispense Z points=%d" %
                      (motion_label, pad_index, dot_number, total_dots,
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
                        "%s pad %d dot %d/%d final closing drag %d/%d %s at dispense Z" %
                        (
                            motion_label,
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
                        "%s pad %d dot %d/%d final closing drag %d/%d %s at dispense Z" %
                        (
                            motion_label,
                            pad_index,
                            dot_number,
                            total_dots,
                            close_index + 1,
                            len(closing_points),
                            point["type"]
                        )
                    )

                log_t("%s pad %d dot %d/%d: AFTER final closing drag at dispense Z" %
                      (motion_label, pad_index, dot_number, total_dots))

        log_t("%s pad %d dot %d/%d: BEFORE retract queue" %
              (motion_label, pad_index, dot_number, total_dots))
        move_z_only_no_dynamic_comp(
            dispense_head,
            units,
            lift_z,
            DISPENSE_SPEED_FACTOR,
            "%s pad %d dot %d/%d retract Z-only" %
            (motion_label, pad_index, dot_number, total_dots)
        )
        log_t("%s pad %d dot %d/%d: AFTER retract queue; completion waits at next pre-glue or final drain" %
              (motion_label, pad_index, dot_number, total_dots))

        log_t("%s pad %d dot %d/%d: END total elapsed=%.3f ms" %
              (motion_label, pad_index, dot_number, total_dots,
               elapsed_ms_since(pad_t0)))

    log_t("%s BEFORE final retract/queue drain" % timer_label)
    wait_coord(dispense_head, "%s final retract/queue drain" % motion_label)
    log_t("%s AFTER final retract/queue drain" % timer_label)

    log_t("%s END total elapsed=%.3f ms dispensed=%d" %
          (timer_label, elapsed_ms_since(pass_t0), dispensed))

    return dispensed


def run_continuous_dry_path_pass(mac, dispense_head, glue, planned_path,
                                 pad_positions, units,
                                 base_z, lift_z,
                                 last_real_dispense_path_index,
                                 first_real_dispense_path_index):
    pass_t0 = log_t("DRY SECOND PASS CONTINUOUS START path_items=%d" %
                    len(planned_path))

    if first_real_dispense_path_index < 0 or last_real_dispense_path_index < 0:
        log("DRY SECOND PASS CONTINUOUS skipped because no real pad path items were available.")
        return 0
    if len(pad_positions) == 0:
        log("DRY SECOND PASS CONTINUOUS skipped because no original pad positions were available.")
        return 0

    first_item = pad_positions[0]
    square_corners = make_dry_square_corners(pad_positions)

    set_boolean_actuator_state_fast(
        glue, False,
        "dry second pass continuous GlueDispense forced off before pass")
    wait_after_actuator_change(
        glue, mac, ACTUATOR_SETTLE_MS,
        "dry second pass continuous GlueDispense forced off before pass wait")

    log("DRY SECOND PASS CONTINUOUS move to first pad %d at lift Z X=%.4f Y=%.4f Z=%.4f" %
        (first_item["pad_index"], first_item["x"], first_item["y"], lift_z))
    move_direct_no_dynamic_comp(
        dispense_head,
        dispense_head_location(units, first_item["x"], first_item["y"], lift_z),
        TRAVEL_SPEED_FACTOR,
        "dry second pass continuous first pad lift XY")
    wait_still(dispense_head, "dry second pass continuous first pad lift XY")

    log("DRY SECOND PASS CONTINUOUS lowering once to dispense Z=%.4f" % base_z)
    move_z_only_no_dynamic_comp(
        dispense_head,
        units,
        base_z,
        DISPENSE_SPEED_FACTOR,
        "dry second pass continuous initial down Z-only")
    wait_coord(dispense_head, "dry second pass continuous initial down Z-only")

    queued_points = 0
    for corner_index in range(len(square_corners)):
        corner = square_corners[corner_index]
        log("DRY SECOND PASS CONTINUOUS queue square corner %d/%d %s local X=%.4f Y=%.4f -> world X=%.4f Y=%.4f Z=%.4f" %
            (corner_index + 1, len(square_corners), corner["name"],
             corner["local_x"], corner["local_y"], corner["x"],
             corner["y"], base_z))
        move_direct_no_dynamic_comp(
            dispense_head,
            dispense_head_location(units, corner["x"], corner["y"], base_z),
            DISPENSE_SPEED_FACTOR,
            "dry second pass continuous square corner %d/%d %s at dispense Z" %
            (corner_index + 1, len(square_corners), corner["name"]))
        queued_points += 1

    log_t("DRY SECOND PASS CONTINUOUS BEFORE queued square drain points=%d" %
          queued_points)
    wait_coord(dispense_head, "dry second pass continuous queued path drain")
    log_t("DRY SECOND PASS CONTINUOUS AFTER queued square drain")

    log("DRY SECOND PASS CONTINUOUS retracting once to lift Z=%.4f" % lift_z)
    move_z_only_no_dynamic_comp(
        dispense_head,
        units,
        lift_z,
        DISPENSE_SPEED_FACTOR,
        "dry second pass continuous final retract Z-only")
    wait_coord(dispense_head, "dry second pass continuous final retract Z-only")

    set_boolean_actuator_state_fast(
        glue, False,
        "dry second pass continuous GlueDispense forced off after pass")
    wait_after_actuator_change(
        glue, mac, ACTUATOR_SETTLE_MS,
        "dry second pass continuous GlueDispense forced off after pass wait")

    log_t("DRY SECOND PASS CONTINUOUS END elapsed=%.3f ms queued_points=%d" %
          (elapsed_ms_since(pass_t0), queued_points))
    return queued_points


def dispense_current_placement(mac, board, bl, current_placement):
    dispense_head = find_dispense_head(mac)
    glue = find_glue_dispense_actuator(mac)
    dispense_placement = get_dispense_placement(board, current_placement)

    try:
        log("Dispense head Java class=%s" % dispense_head.getClass().getName())
    except:
        pass

    log_t("DISPENSE CONFIG pulse=%d post=%d pre=%d settle=%d lift=%.3f travelSpeed=%.3f dispenseSpeed=%.3f" %
          (DISPENSE_PULSE_MS, POST_DISPENSE_DWELL_MS, PRE_DISPENSE_DWELL_MS,
           ACTUATOR_SETTLE_MS,
           INTER_PAD_LIFT_MM, TRAVEL_SPEED_FACTOR, DISPENSE_SPEED_FACTOR))

    part = dispense_placement.getPart()
    if part is None:
        raise Exception("Dispense placement has no part.")

    pkg = part.getPackage()
    if pkg is None or pkg.getFootprint() is None:
        raise Exception("Dispense part has no package footprint.")

    footprint = pkg.getFootprint()
    pads = footprint.getPads()
    if pads is None or pads.size() == 0:
        raise Exception("Dispense footprint has no pads.")

    active_bl_loc = bl.getLocation()
    log("Dispense using ACTIVE corrected boardLocation X=%.4f Y=%.4f Z=%.4f R=%.4f" %
        (active_bl_loc.getX(), active_bl_loc.getY(),
         active_bl_loc.getZ(), active_bl_loc.getRotation()))

    fid_placement, fid_loc, fid_settings, fid_label = \
        get_board_fiducial_placement_and_settings(board)

    fid_world = Utils2D.calculateBoardPlacementLocation(bl, fid_loc)

    base_z = fid_world.getZ() + DISPENSE_Z_OFFSET_MM
    lift_z = base_z + INTER_PAD_LIFT_MM
    units = fid_world.getUnits()

    fid_units = fid_loc.getUnits()
    fid_r = fid_loc.getRotation()
    fid_rad = fid_r * Math.PI / 180.0
    fid_c = Math.cos(fid_rad)
    fid_s = Math.sin(fid_rad)

    try:
        footprint_units = footprint.getUnits()
    except:
        footprint_units = fid_units

    log("Dispensing placement %s part %s footprint pads=%d using %s + %s" %
        (object_name(dispense_placement), part.getId(), pads.size(),
         object_name(dispense_head), object_name(glue)))
    log("Dispense anchor BOARD fiducial '%s' design X=%.4f Y=%.4f Z=%.4f R=%.4f" %
        (fid_label, fid_loc.getX(), fid_loc.getY(),
         fid_loc.getZ(), fid_loc.getRotation()))
    log("Dispense anchor WORLD from ACTIVE boardLocation X=%.4f Y=%.4f Z=%.4f R=%.4f baseZ=%.4f liftZ=%.4f" %
        (fid_world.getX(), fid_world.getY(), fid_world.getZ(),
         fid_world.getRotation(), base_z, lift_z))
    log("Dispense pad transform: footprint pad -> board fiducial/pocket location -> ACTIVE boardLocation world location.")
    log("Dispense needle B/rotation axis blocked: all dispense-head move targets use R=NaN.")
    log("Dispense moves use SuppressDynamicCompensation to bypass nozzle runout / rotation-mode compensation.")

    set_boolean_actuator_state_fast(glue, False, "glue initial off")

    start = dispense_head_location(units, fid_world.getX(), fid_world.getY(), DISPENSE_SAFE_Z_MM)
    move_direct_no_dynamic_comp(
        dispense_head,
        start,
        TRAVEL_SPEED_FACTOR,
        "dispense head start no dynamic comp"
    )
    wait_still(dispense_head, "dispense head start")

    pad_positions = []
    for i in range(pads.size()):
        pad = pads.get(i)
        pad_name = str(pad.getName())

        if pad_name in PAD_SKIP_NAMES:
            log("Skipping pad %d named %s." % (i, pad_name))
            continue

        px_raw, py_raw = pad_xy(pad)

        pad_offset_loc = Location(footprint_units, px_raw, py_raw, 0.0, 0.0)
        pad_offset_board_units = pad_offset_loc.convertToUnits(fid_units)

        px = pad_offset_board_units.getX()
        py = pad_offset_board_units.getY()

        pad_board_x = fid_loc.getX() + (px * fid_c - py * fid_s)
        pad_board_y = fid_loc.getY() + (px * fid_s + py * fid_c)

        pad_board_loc = Location(
            fid_units,
            pad_board_x,
            pad_board_y,
            fid_loc.getZ(),
            0.0
        )

        pad_world = Utils2D.calculateBoardPlacementLocation(bl, pad_board_loc)
        pad_world_units = pad_world.convertToUnits(units)

        wx = pad_world_units.getX()
        wy = pad_world_units.getY()

        log("Pad %d '%s': footprint X=%.4f Y=%.4f -> board-from-fid X=%.4f Y=%.4f -> world X=%.4f Y=%.4f R=NaN" %
            (i, pad_name, px_raw, py_raw, pad_board_x, pad_board_y, wx, wy))

        pad_positions.append(make_pad_path_item(
            i,
            pad_name,
            wx,
            wy,
            0,
            1,
            px_raw,
            py_raw))

    real_dot_items = expand_first_pad_dots(pad_positions, units)
    planned_path, inserted_corner_count = insert_corner_waypoints(real_dot_items, units)
    last_real_dispense_path_index = find_last_real_dispense_path_index(planned_path)
    first_real_dispense_path_index = find_first_real_dispense_path_index(planned_path)

    log("Planned dispense path: original non-skipped pads=%d real dots=%d travel-only corners=%d total items=%d" %
        (len(pad_positions), len(real_dot_items), inserted_corner_count,
         len(planned_path)))

    dispensed = run_dispense_path_pass(
        mac,
        dispense_head,
        glue,
        planned_path,
        units,
        base_z,
        lift_z,
        last_real_dispense_path_index,
        first_real_dispense_path_index,
        "DISPENSE",
        "dispense")

    if DRY_SECOND_PASS:
        log("DRY_SECOND_PASS is True; running continuous dry path at dispense height.")
        run_continuous_dry_path_pass(
            mac,
            dispense_head,
            glue,
            planned_path,
            pad_positions,
            units,
            base_z,
            lift_z,
            last_real_dispense_path_index,
            first_real_dispense_path_index)

    set_boolean_actuator_state_fast(glue, False, "glue final off")

    end = dispense_head_location(units, fid_world.getX(), fid_world.getY(), DISPENSE_SAFE_Z_MM)
    move_direct_no_dynamic_comp(
        dispense_head,
        end,
        TRAVEL_SPEED_FACTOR,
        "dispense head safe end no dynamic comp"
    )
    wait_still(dispense_head, "dispense head safe end")

    log("Dispense complete: %d pads pulsed." % dispensed)
    return dispensed


def dap_off_dispense_head(mac):
    dispense_head = find_dispense_head(mac)
    units = LengthUnit.Millimeters
    safe_z = DISPENSE_SAFE_Z_MM
    touch_z = DAP_OFF_Z_MM
    lift_z = DAP_OFF_Z_MM + DAP_OFF_LIFT_MM
    touches = int(DAP_OFF_TOUCHES)
    random_radius = DAP_OFF_RANDOM_DIAMETER_MM / 2.0
    random_distance = Math.sqrt(Math.random()) * random_radius
    random_angle = Math.random() * 2.0 * Math.PI
    selected_x = DAP_OFF_X_MM + (Math.cos(random_angle) * random_distance)
    selected_y = DAP_OFF_Y_MM + (Math.sin(random_angle) * random_distance)

    if touches < 1:
        log("Dap-off skipped because DAP_OFF_TOUCHES=%d." % touches)
        return

    log("Dap-off start: center X=%.4f Y=%.4f selected X=%.4f Y=%.4f radius=%.4f Z=%.4f lift=%.4f touches=%d safeZ=%.4f" %
        (DAP_OFF_X_MM, DAP_OFF_Y_MM, selected_x, selected_y,
         random_radius, touch_z, DAP_OFF_LIFT_MM, touches, safe_z))

    move_z_only_no_dynamic_comp(
        dispense_head,
        units,
        safe_z,
        TRAVEL_SPEED_FACTOR,
        "dap-off ensure safe Z before XY")
    wait_coord(dispense_head, "dap-off ensure safe Z before XY")

    move_direct_no_dynamic_comp(
        dispense_head,
        dispense_head_location(units, selected_x, selected_y, safe_z),
        TRAVEL_SPEED_FACTOR,
        "dap-off move to point at safe Z")
    wait_still(dispense_head, "dap-off move to point at safe Z")

    for touch_index in range(touches):
        touch_number = touch_index + 1
        move_z_only_no_dynamic_comp(
            dispense_head,
            units,
            touch_z,
            DISPENSE_SPEED_FACTOR,
            "dap-off touch %d/%d down Z-only" %
            (touch_number, touches))
        wait_coord(
            dispense_head,
            "dap-off touch %d/%d at Z" %
            (touch_number, touches))

        if touch_index < touches - 1:
            move_z_only_no_dynamic_comp(
                dispense_head,
                units,
                lift_z,
                DISPENSE_SPEED_FACTOR,
                "dap-off touch %d/%d lift Z-only" %
                (touch_number, touches))
            wait_coord(
                dispense_head,
                "dap-off touch %d/%d lifted" %
                (touch_number, touches))

    move_z_only_no_dynamic_comp(
        dispense_head,
        units,
        safe_z,
        TRAVEL_SPEED_FACTOR,
        "dap-off final safe Z retract")
    wait_still(dispense_head, "dap-off final safe Z retract")

    log("Dap-off complete.")


def update_bound_placement_location(bl, current_placement):
    final_loc = Utils2D.calculateBoardPlacementLocation(
        bl, current_placement.getLocation())

    log("[post-dispense] Final die placementLocation from updated boardLocation: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
        (final_loc.getX(), final_loc.getY(),
         final_loc.getZ(), final_loc.getRotation()))

    if "placementLocation" not in globals():
        log("[post-dispense] No placementLocation binding present; boardLocation only was updated.")
        return final_loc

    old_loc = placementLocation
    try:
        log("[post-dispense] Existing bound placementLocation before update: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
            (old_loc.getX(), old_loc.getY(),
             old_loc.getZ(), old_loc.getRotation()))
    except:
        log("[post-dispense] Existing bound placementLocation before update: %s" %
            str(old_loc))

    applied = False
    try:
        if hasattr(old_loc, "setLocation"):
            old_loc.setLocation(final_loc)
            applied = True
            log("[post-dispense] Applied final die target via placementLocation.setLocation(...).")
    except Exception, e:
        log("[post-dispense] placementLocation.setLocation(...) failed: %s" % e)

    if not applied:
        globals()["placementLocation"] = final_loc
        log("[post-dispense] Rebound placementLocation global to final die target.")

    try:
        updated_loc = placementLocation
        log("[post-dispense] Bound placementLocation after update: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
            (updated_loc.getX(), updated_loc.getY(),
             updated_loc.getZ(), updated_loc.getRotation()))
    except:
        pass

    return final_loc


def disable_current_placement(pl, message):
    try:
        if hasattr(pl, "setEnabled"):
            pl.setEnabled(False)
            log("placement.setEnabled(False) called.")
    except Exception, e:
        log("Failed to disable placement: %s" % e)
    try:
        existing = pl.getComments() or ""
        if existing:
            pl.setComments(existing + " " + message)
        else:
            pl.setComments(message)
    except:
        pass


original_bl_loc = None
pre_bl_loc = None
final_update_done = False
dispense_done = False
dap_off_done = False


try:
    required_globals = ("placement", "boardLocation", "machine")
    missing = [name for name in required_globals if name not in globals()]
    if missing:
        raise Exception("Missing globals: %s" % ", ".join(missing))

    pl = placement
    bl = boardLocation
    mac = machine
    prt = part if "part" in globals() else None

    pid = prt.getId() if (prt is not None and hasattr(prt, "getId")) else "<no-part-id>"
    log("Part id=%s" % pid)

    board = None
    try:
        board = bl.getBoard()
    except:
        pass
    if board is None:
        raise Exception("boardLocation.getBoard() returned null.")

    log("Board object: %s" % board)

    original_bl_loc = bl.getLocation()
    log("Original boardLocation: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
        (original_bl_loc.getX(), original_bl_loc.getY(),
         original_bl_loc.getZ(), original_bl_loc.getRotation()))

    if SKIP_DISPENSE:
        log("SKIP_DISPENSE is True; skipping pre-dispense vision and adhesive dispense.")
    else:
        pre = measure_board_fiducial_correction(mac, bl, board, prt, "pre-dispense")
        pre_bl_loc = pre["boardLocation"]
        bl.setLocation(pre_bl_loc)
        dispense_current_placement(mac, board, bl, pl)
        dispense_done = True

    if dispense_done and not POST_DISPENSE_IMAGE_CAPTURE_ENABLED:
        final_bl_loc = pre_bl_loc
        log("[post-dispense] POST_DISPENSE_IMAGE_CAPTURE_ENABLED is False; using pre-dispense corrected boardLocation.")
    else:
        if POST_DISPENSE_IMAGING_DELAY_MS > 0:
            log("[post-dispense] Waiting %d ms before final imaging." %
                POST_DISPENSE_IMAGING_DELAY_MS)
            Thread.sleep(POST_DISPENSE_IMAGING_DELAY_MS)

        post = measure_board_fiducial_correction(mac, bl, board, prt, "post-dispense")
        final_bl_loc = post["boardLocation"]

    bl.setLocation(final_bl_loc)
    update_bound_placement_location(bl, pl)
    final_update_done = True
    log("[post-dispense] Updated boardLocation: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
        (final_bl_loc.getX(), final_bl_loc.getY(),
         final_bl_loc.getZ(), final_bl_loc.getRotation()))
    if SKIP_DISPENSE:
        log("Skip-dispense post alignment flow complete.")
    else:
        dap_off_dispense_head(mac)
        dap_off_done = True
        log("Combined pre-dispense, dispense, post-dispense flow complete.")

except SystemExit:
    pass

except VisionMeasurementError, e:
    log_err(e)
    try:
        force_glue_off_safe(mac, "vision failure")
    except:
        pass
    if dispense_done and not dap_off_done:
        try:
            log("Vision failure after dispense; running dap-off before exiting.")
            dap_off_dispense_head(mac)
            dap_off_done = True
        except Exception, dap_e:
            log_err("Failed to dap-off after vision failure: %s" % dap_e)
    if original_bl_loc is not None and not final_update_done:
        try:
            bl.setLocation(original_bl_loc)
            log("Restored original boardLocation after vision failure: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
                (original_bl_loc.getX(), original_bl_loc.getY(),
                 original_bl_loc.getZ(), original_bl_loc.getRotation()))
        except Exception, restore_e:
            log_err("Failed to restore original boardLocation: %s" % restore_e)
    try:
        disable_current_placement(
            placement,
            "[auto-skip: board fiducial vision not confirmed %s]" % e)
    except:
        pass
    log("Vision failure soft-skip complete; placement was disabled and script is exiting normally.")

except Exception, e:
    log_err(e)
    try:
        force_glue_off_safe(mac, "placement-starting error")
    except:
        pass
    if dispense_done and not dap_off_done:
        try:
            log("Error after dispense; running dap-off before exiting.")
            dap_off_dispense_head(mac)
            dap_off_done = True
        except Exception, dap_e:
            log_err("Failed to dap-off after placement-starting error: %s" % dap_e)
    if original_bl_loc is not None and not final_update_done:
        try:
            bl.setLocation(original_bl_loc)
            log("Restored original boardLocation after failed pre/dispense/post flow: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
                (original_bl_loc.getX(), original_bl_loc.getY(),
                 original_bl_loc.getZ(), original_bl_loc.getRotation()))
        except Exception, restore_e:
            log_err("Failed to restore original boardLocation: %s" % restore_e)
    try:
        disable_current_placement(placement, "[auto-skip: placement-starting error %s]" % e)
    except:
        pass
    raise
