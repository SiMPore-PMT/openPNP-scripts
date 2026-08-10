#DEPREICATED 06/12. Adding corner roundi
# Job.Placement.Starting.py
#
# Combined board correction + adhesive dispense flow:
#   1. Measure board fiducial before dispense.
#   2. Temporarily apply that corrected board pose and dispense footprint pads.
#   3. Measure board fiducial after dispense and update boardLocation.
#   4. Return normally so OpenPnP continues the standard job flow.

from jarray import array
from java.lang import Boolean, Class, Double, Math, Object, System, Thread
from org.openpnp.model import Location
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import MovableUtils, Utils2D
from org.opencv.core import RotatedRect
from org.openpnp.spi.Locatable import LocationOption


# ----------------- FLOW CONFIG -----------------
SKIP_DISPENSE = False


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

DISPENSE_Z_OFFSET_MM    = 0.1
INTER_PAD_LIFT_MM       = 0.1
DISPENSE_SAFE_Z_MM      = 25.0

TRAVEL_SPEED_FACTOR     = 1
DISPENSE_SPEED_FACTOR   = 1

PRE_DISPENSE_DWELL_MS   = 0
DISPENSE_PULSE_MS       = 15
POST_DISPENSE_DWELL_MS  = 10
ACTUATOR_SETTLE_MS      = 20

PAD_SKIP_NAMES          = set()

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

    dispensed = 0
    dispense_t0 = log_t("DISPENSE START pads=%d" % pads.size())

    for i in range(pads.size()):
        pad_t0 = log_t("pad %d: START" % i)
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

        move_direct_no_dynamic_comp(
            dispense_head,
            dispense_head_location(units, wx, wy, lift_z),
            TRAVEL_SPEED_FACTOR,
            "dispense pad %d lift" % i
        )

        move_z_only_no_dynamic_comp(
            dispense_head,
            units,
            base_z,
            DISPENSE_SPEED_FACTOR,
            "dispense pad %d down Z-only" % i
        )

        log_t("pad %d: BEFORE pre-glue wait_coord" % i)
        wait_coord(dispense_head, "dispense pad %d pre-glue" % i)
        log_t("pad %d: AFTER pre-glue wait_coord / AT DISPENSE Z" % i)

        log("Dispense head reached pad %d at dispense Z." % i)

        sleep_ms(PRE_DISPENSE_DWELL_MS)

        log_t("pad %d: BEFORE pulse_glue_controller" % i)
        pulse_glue_controller(glue, mac, i)
        log_t("pad %d: AFTER pulse_glue_controller" % i)

        log_t("pad %d: BEFORE retract queue" % i)
        move_z_only_no_dynamic_comp(
            dispense_head,
            units,
            lift_z,
            DISPENSE_SPEED_FACTOR,
            "dispense pad %d retract Z-only" % i
        )
        log_t("pad %d: AFTER retract queue; completion waits at next pre-glue or final drain" % i)

        log_t("pad %d: END total elapsed=%.3f ms" % (i, elapsed_ms_since(pad_t0)))
        dispensed += 1

    log_t("DISPENSE BEFORE final retract/queue drain")
    wait_coord(dispense_head, "dispense final retract/queue drain")
    log_t("DISPENSE AFTER final retract/queue drain")

    log_t("DISPENSE END total elapsed=%.3f ms dispensed=%d" %
          (elapsed_ms_since(dispense_t0), dispensed))

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
#OLD
# def dispense_current_placement(mac, board, bl, current_placement):
#     dispense_head = find_dispense_head(mac)
#     glue = find_glue_dispense_actuator(mac)
#     dispense_placement = get_dispense_placement(board, current_placement)
#     log_t("DISPENSE CONFIG pulse=%d post=%d pre=%d settle=%d lift=%.3f travelSpeed=%.3f dispenseSpeed=%.3f" %
#           (DISPENSE_PULSE_MS, POST_DISPENSE_DWELL_MS, PRE_DISPENSE_DWELL_MS,
#            ACTUATOR_SETTLE_MS,
#            INTER_PAD_LIFT_MM, TRAVEL_SPEED_FACTOR, DISPENSE_SPEED_FACTOR))

#     part = dispense_placement.getPart()
#     if part is None:
#         raise Exception("Dispense placement has no part.")
#     pkg = part.getPackage()
#     if pkg is None or pkg.getFootprint() is None:
#         raise Exception("Dispense part has no package footprint.")

#     pads = pkg.getFootprint().getPads()
#     if pads is None or pads.size() == 0:
#         raise Exception("Dispense footprint has no pads.")

#     active_bl_loc = bl.getLocation()
#     log("Dispense using ACTIVE corrected boardLocation X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (active_bl_loc.getX(), active_bl_loc.getY(),
#          active_bl_loc.getZ(), active_bl_loc.getRotation()))

#     world = Utils2D.calculateBoardPlacementLocation(
#         bl, dispense_placement.getLocation())
#     base_z = world.getZ() + DISPENSE_Z_OFFSET_MM
#     lift_z = base_z + INTER_PAD_LIFT_MM
#     units = world.getUnits()
#     pr = world.getRotation() * Math.PI / 180.0
#     c = Math.cos(pr)
#     s = Math.sin(pr)

#     log("Dispensing placement %s part %s footprint pads=%d using %s + %s" %
#         (object_name(dispense_placement), part.getId(), pads.size(),
#          object_name(dispense_head), object_name(glue)))
#     log("Dispense world origin X=%.4f Y=%.4f Z=%.4f R=%.4f baseZ=%.4f liftZ=%.4f" %
#         (world.getX(), world.getY(), world.getZ(), world.getRotation(), base_z, lift_z))
#     log("Dispense needle B/rotation axis blocked: all dispense-head move targets use R=NaN.")

#     set_boolean_actuator_state_fast(glue, False, "glue initial off")

#     start = dispense_head_location(units, world.getX(), world.getY(), DISPENSE_SAFE_Z_MM)
#     move_safe_wait(dispense_head, start, "dispense head start")

#     dispensed = 0
#     dispense_t0 = log_t("DISPENSE START pads=%d" % pads.size())
#     for i in range(pads.size()):
#         pad_t0 = log_t("pad %d: START" % i)
#         pad = pads.get(i)
#         pad_name = str(pad.getName())
#         if pad_name in PAD_SKIP_NAMES:
#             log("Skipping pad %d named %s." % (i, pad_name))
#             continue

#         px, py = pad_xy(pad)
#         wx = world.getX() + (px * c - py * s)
#         wy = world.getY() + (px * s + py * c)

#         log("Pad %d '%s': local X=%.4f Y=%.4f -> world X=%.4f Y=%.4f R=NaN" %
#             (i, pad_name, px, py, wx, wy))

#         move_direct(
#             dispense_head,
#             dispense_head_location(units, wx, wy, lift_z),
#             TRAVEL_SPEED_FACTOR,
#             "dispense pad %d lift" % i)
#         move_z_only(
#             dispense_head,
#             units,
#             base_z,
#             DISPENSE_SPEED_FACTOR,
#             "dispense pad %d down Z-only" % i)
#         log_t("pad %d: BEFORE pre-glue wait_coord" % i)
#         wait_coord(dispense_head, "dispense pad %d pre-glue" % i)
#         log_t("pad %d: AFTER pre-glue wait_coord / AT DISPENSE Z" % i)
#         log("Dispense head reached pad %d at dispense Z." % i)
#         sleep_ms(PRE_DISPENSE_DWELL_MS)
#         log_t("pad %d: BEFORE pulse_glue_controller" % i)
#         pulse_glue_controller(glue, mac, i)
#         log_t("pad %d: AFTER pulse_glue_controller" % i)
#         log_t("pad %d: BEFORE retract queue" % i)
#         move_z_only(
#             dispense_head,
#             units,
#             lift_z,
#             DISPENSE_SPEED_FACTOR,
#             "dispense pad %d retract Z-only" % i)
#         log_t("pad %d: AFTER retract queue; completion waits at next pre-glue or final drain" % i)
#         log_t("pad %d: END total elapsed=%.3f ms" % (i, elapsed_ms_since(pad_t0)))
#         dispensed += 1

#     log_t("DISPENSE BEFORE final retract/queue drain")
#     wait_coord(dispense_head, "dispense final retract/queue drain")
#     log_t("DISPENSE AFTER final retract/queue drain")
#     log_t("DISPENSE END total elapsed=%.3f ms dispensed=%d" %
#           (elapsed_ms_since(dispense_t0), dispensed))
#     set_boolean_actuator_state_fast(glue, False, "glue final off")
#     end = dispense_head_location(units, world.getX(), world.getY(), DISPENSE_SAFE_Z_MM)
#     move_safe_wait(dispense_head, end, "dispense head safe end")
#     log("Dispense complete: %d pads pulsed." % dispensed)
#     return dispensed


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
final_update_done = False


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
        log("[pre-dispense] Temporary boardLocation for dispense: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
            (pre_bl_loc.getX(), pre_bl_loc.getY(),
             pre_bl_loc.getZ(), pre_bl_loc.getRotation()))
        bl.setLocation(pre_bl_loc)
        log("[pre-dispense] Temporary boardLocation is active for dispense and post inspection.")
        active_pre_loc = bl.getLocation()
        log("[pre-dispense] Active boardLocation readback before syringe dispense: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
            (active_pre_loc.getX(), active_pre_loc.getY(),
             active_pre_loc.getZ(), active_pre_loc.getRotation()))

        dispense_current_placement(mac, board, bl, pl)

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
        log("Combined pre-dispense, dispense, post-dispense flow complete.")

except SystemExit:
    pass

except VisionMeasurementError, e:
    log_err(e)
    try:
        force_glue_off_safe(mac, "vision failure")
    except:
        pass
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
# # Job.Placement.Starting.py
# #
# # Combined board correction + adhesive dispense flow:
# #   1. Measure board fiducial before dispense.
# #   2. Temporarily apply that corrected board pose and dispense footprint pads.
# #   3. Measure board fiducial after dispense and update boardLocation.
# #   4. Return normally so OpenPnP continues the standard job flow.

# from jarray import array
# from java.lang import Boolean, Class, Double, Math, Object, System, Thread
# from org.openpnp.model import Location
# from org.openpnp.spi.MotionPlanner import CompletionType
# from org.openpnp.util import MovableUtils, Utils2D
# from org.opencv.core import RotatedRect


# # ----------------- FLOW CONFIG -----------------
# SKIP_DISPENSE = False


# # ----------------- VISION CONFIG -----------------
# MAX_NUDGE_MM           = 4.0
# APPLY_ROTATION         = True
# EXPECTED_ANGLE_DEG     = 0.0
# DESIGN_BOARD_ROT_DEG   = 0.0

# PRE_CAPTURE_SETTLE_MS  = 250
# RESULT_STAGE_KEY       = "results"
# MAX_ATTEMPTS           = 3
# RETRY_DELAY_MS         = 250
# VISION_MEASUREMENT_ATTEMPTS = 2
# RECENTER_MAX_PASSES    = 3
# RECENTER_TOLERANCE_MM  = 0.02

# FLIP_IMAGE_X = False
# FLIP_IMAGE_Y = True


# # ----------------- DISPENSE CONFIG -----------------
# DISPENSE_HEAD_NAME      = "Dispense Head"
# DISPENSE_ACTUATOR_NAME  = "GlueDispense"

# DISPENSE_Z_OFFSET_MM    = 0.1
# INTER_PAD_LIFT_MM       = 0.25
# DISPENSE_SAFE_Z_MM      = 25.0

# TRAVEL_SPEED_FACTOR     = 0.9
# DISPENSE_SPEED_FACTOR   = 0.9

# PRE_DISPENSE_DWELL_MS   = 0
# DISPENSE_PULSE_MS       = 20
# POST_DISPENSE_DWELL_MS  = 10
# ACTUATOR_SETTLE_MS      = 30

# PAD_SKIP_NAMES          = set()

# TAG = "Job.Placement.Starting"


# def log(msg):
#     print "[%s] %s" % (TAG, str(msg))


# def log_err(msg):
#     print "[%s ERROR] %s" % (TAG, str(msg))


# class VisionMeasurementError(Exception):
#     pass


# SCRIPT_T0_NS = System.nanoTime()
# LAST_T_NS = SCRIPT_T0_NS


# def now_ms():
#     return (System.nanoTime() - SCRIPT_T0_NS) / 1000000.0


# def elapsed_ms_since(t0_ns):
#     return (System.nanoTime() - t0_ns) / 1000000.0


# def log_t(label):
#     global LAST_T_NS
#     now_ns = System.nanoTime()
#     abs_ms = (now_ns - SCRIPT_T0_NS) / 1000000.0
#     delta_ms = (now_ns - LAST_T_NS) / 1000000.0
#     LAST_T_NS = now_ns
#     log("[TIMER] %+9.3f ms | +%8.3f ms | %s" % (abs_ms, delta_ms, label))
#     return now_ns


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
#     for method in ["getName", "getId"]:
#         try:
#             value = getattr(obj, method)()
#             if value is not None:
#                 return str(value)
#         except:
#             pass
#     return str(obj)


# def wait_still(movable, label):
#     t0 = log_t("%s: wait_still BEFORE WaitForStillstand" % label)
#     movable.waitForCompletion(CompletionType.WaitForStillstand)
#     log_t("%s: wait_still AFTER WaitForStillstand elapsed=%.3f ms" %
#           (label, elapsed_ms_since(t0)))


# def wait_coord(movable, label):
#     t0 = log_t("%s: wait_coord BEFORE WaitForUnconditionalCoordination" % label)
#     movable.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
#     log_t("%s: wait_coord AFTER WaitForUnconditionalCoordination elapsed=%.3f ms" %
#           (label, elapsed_ms_since(t0)))


# def move_safe_wait(movable, target, label):
#     MovableUtils.moveToLocationAtSafeZ(movable, target)
#     wait_still(movable, label)


# def move_direct(movable, target, speed, label):
#     log_t("%s: move_direct BEFORE queue target=%s speed=%.3f" %
#           (label, str(target), float(speed)))
#     try:
#         movable.moveTo(target, float(speed))
#     except TypeError:
#         movable.moveTo(target)
#     log_t("%s: move_direct AFTER moveTo" % label)


# def move_z_only(movable, units, z, speed, label):
#     target = Location(units, Double.NaN, Double.NaN, z, Double.NaN)
#     log_t("%s: move_z_only BEFORE target=%s speed=%.3f" %
#           (label, str(target), float(speed)))
#     try:
#         movable.moveTo(target, float(speed))
#     except TypeError:
#         movable.moveTo(target)
#     log_t("%s: move_z_only AFTER" % label)


# def set_boolean_actuator_state_fast(actuator, on_off, label):
#     value = Boolean.TRUE if bool(on_off) else Boolean.FALSE
#     method = actuator.getClass().getMethod(
#         "actuate",
#         array([Boolean.TYPE], Class)
#     )
#     method.invoke(actuator, array([value], Object))
#     log("%s boolean actuator=%s queued." % (label, str(bool(on_off))))


# def find_rotated_rect_with_retries(pipeline):
#     candidate_keys = [RESULT_STAGE_KEY, "results", "preResults"]
#     for attempt in range(1, MAX_ATTEMPTS + 1):
#         try:
#             log("Vision attempt %d of %d" % (attempt, MAX_ATTEMPTS))
#             pipeline.process()
#             log("pipeline.process() OK on attempt %d" % attempt)

#             for key in candidate_keys:
#                 try:
#                     res = pipeline.getResult(key)
#                 except:
#                     res = None
#                 if res is None:
#                     continue

#                 model = res.getModel()
#                 if model is None:
#                     continue

#                 if isinstance(model, RotatedRect):
#                     log("Got RotatedRect directly from results key '%s'." % key)
#                     return model

#                 if hasattr(model, "size") and hasattr(model, "get"):
#                     if model.size() > 0:
#                         first = model.get(0)
#                         if isinstance(first, RotatedRect) or hasattr(first, "center"):
#                             log("Got RotatedRect[0] from list at results key '%s'." % key)
#                             return first

#             log("No RotatedRect found on attempt %d." % attempt)
#         except Exception, e:
#             log("Exception during pipeline attempt %d: %s" % (attempt, e))

#         if attempt < MAX_ATTEMPTS:
#             Thread.sleep(RETRY_DELAY_MS)
#     return None


# def get_camera_image_size(cam):
#     try:
#         return cam.getWidth(), cam.getHeight()
#     except:
#         pass
#     image = cam.settleAndCapture()
#     return image.getWidth(), image.getHeight()


# def get_camera_units_per_pixel(cam):
#     if hasattr(cam, "getUnitsPerPixelAtZ"):
#         upp_loc = cam.getUnitsPerPixelAtZ()
#     else:
#         upp_loc = cam.getUnitsPerPixel()
#     if upp_loc is None:
#         raise Exception("Camera units-per-pixel not configured.")
#     return upp_loc


# def extract_rotated_rects_from_model(model):
#     rects = []
#     if model is None:
#         return rects
#     if isinstance(model, RotatedRect) or hasattr(model, "center"):
#         rects.append(model)
#         return rects
#     if hasattr(model, "size") and hasattr(model, "get"):
#         for i in range(model.size()):
#             item = model.get(i)
#             if isinstance(item, RotatedRect) or hasattr(item, "center"):
#                 rects.append(item)
#         return rects
#     try:
#         for item in model:
#             if isinstance(item, RotatedRect) or hasattr(item, "center"):
#                 rects.append(item)
#     except:
#         pass
#     return rects


# def get_pipeline_rotated_rects(pipeline):
#     rects = []
#     for key in [RESULT_STAGE_KEY, "results", "preResults"]:
#         try:
#             res = pipeline.getResult(key)
#         except:
#             res = None
#         if res is None:
#             continue
#         try:
#             model = res.getModel()
#         except:
#             try:
#                 model = res.model
#             except:
#                 model = None
#         rects.extend(extract_rotated_rects_from_model(model))
#     return rects


# def choose_closest_rect_to_image_center(rects, width, height):
#     best = None
#     best_dist2 = None
#     cx = width / 2.0
#     cy = height / 2.0
#     for rr in rects:
#         dx = rr.center.x - cx
#         dy = rr.center.y - cy
#         dist2 = dx * dx + dy * dy
#         if best is None or dist2 < best_dist2:
#             best = rr
#             best_dist2 = dist2
#     return best, best_dist2


# def rect_center_machine_offset(cam_loc, upp_loc, rr, width, height):
#     px = rr.center.x - (width / 2.0)
#     py = rr.center.y - (height / 2.0)
#     if FLIP_IMAGE_X:
#         px = -px
#     if FLIP_IMAGE_Y:
#         py = -py

#     dx_cam = px * upp_loc.getX()
#     dy_cam = py * upp_loc.getY()

#     rad = cam_loc.getRotation() * Math.PI / 180.0
#     c = Math.cos(rad)
#     s = Math.sin(rad)
#     dx = dx_cam * c - dy_cam * s
#     dy = dx_cam * s + dy_cam * c
#     return dx, dy, px, py


# def location_xy_distance_mm(a, b):
#     a_mm = a.convertToUnits(a.getUnits())
#     b_units = a_mm.getUnits()
#     b_mm = b.convertToUnits(b_units)
#     dx = a_mm.getX() - b_mm.getX()
#     dy = a_mm.getY() - b_mm.getY()
#     return abs(dx), abs(dy)


# def detection_inside_initial_view(initial_view_loc, detected_loc, upp_loc, width, height):
#     dx, dy = location_xy_distance_mm(initial_view_loc, detected_loc)
#     upp_mm = upp_loc.convertToUnits(initial_view_loc.getUnits())
#     half_x = abs(upp_mm.getX()) * width / 2.0
#     half_y = abs(upp_mm.getY()) * height / 2.0
#     return dx <= half_x and dy <= half_y, dx, dy, half_x, half_y


# def move_camera_to_detected_center(cam, detected_loc, label):
#     cam_loc = cam.getLocation()
#     detected_cam_units = detected_loc.convertToUnits(cam_loc.getUnits())
#     target = Location(
#         cam_loc.getUnits(),
#         detected_cam_units.getX(),
#         detected_cam_units.getY(),
#         cam_loc.getZ(),
#         cam_loc.getRotation()
#     )
#     move_safe_wait(cam, target, label)


# def get_board_placements(board):
#     for method in ["getPlacements", "getBoardPlacements"]:
#         try:
#             placements = getattr(board, method)()
#             if placements is not None:
#                 log("Using board.%s()." % method)
#                 return placements
#         except:
#             pass
#     raise Exception("Board placements list is null.")


# def is_fiducial_placement(plc):
#     try:
#         if hasattr(plc, "isFiducial") and plc.isFiducial():
#             return True
#     except:
#         pass
#     try:
#         if hasattr(plc, "getType"):
#             t = plc.getType()
#             if t is not None and "FIDUCIAL" in str(t).upper():
#                 return True
#     except:
#         pass
#     return False


# def is_normal_placement(plc):
#     try:
#         if hasattr(plc, "getType"):
#             t = plc.getType()
#             if t is not None and str(t).upper() == "PLACEMENT":
#                 return True
#     except:
#         pass
#     return not is_fiducial_placement(plc)


# def get_vision_settings_for_fiducial(plc):
#     for method in ["getFiducialVisionSettings", "getVisionSettings"]:
#         try:
#             settings = getattr(plc, method)()
#             if settings is not None:
#                 return settings
#         except:
#             pass
#     try:
#         p = plc.getPart()
#         if p is not None and hasattr(p, "getFiducialVisionSettings"):
#             settings = p.getFiducialVisionSettings()
#             if settings is not None:
#                 return settings
#     except:
#         pass
#     return None


# def get_board_fiducial_placement_and_settings(board):
#     placements = get_board_placements(board)
#     count = len(java_list_to_py(placements))
#     log("Board placements count: %d" % count)

#     for i, plc in enumerate(java_list_to_py(placements)):
#         try:
#             if hasattr(plc, "isEnabled") and not plc.isEnabled():
#                 continue
#         except:
#             pass
#         if not is_fiducial_placement(plc):
#             continue
#         settings = get_vision_settings_for_fiducial(plc)
#         if settings is None:
#             log("Fiducial placement index %d has no vision settings; skipping." % i)
#             continue
#         loc = plc.getLocation()
#         label = object_name(plc) or "<unnamed>"
#         log("Selected fiducial placement '%s' at index %d." % (label, i))
#         return plc, loc, settings, label

#     raise Exception("No enabled board fiducial placement with vision settings found.")


# def get_dispense_placement(board, current_placement):
#     try:
#         if current_placement is not None and is_normal_placement(current_placement):
#             if current_placement.getPart() is not None:
#                 return current_placement
#     except:
#         pass

#     for plc in java_list_to_py(get_board_placements(board)):
#         try:
#             if hasattr(plc, "isEnabled") and not plc.isEnabled():
#                 continue
#         except:
#             pass
#         if is_normal_placement(plc):
#             try:
#                 if plc.getPart() is not None:
#                     return plc
#             except:
#                 pass
#     raise Exception("No enabled normal placement with a part found for dispense.")


# def make_board_location(bl_loc, x, y, z, r):
#     return Location(bl_loc.getUnits(), x, y, z, r)


# def prepare_pipeline_context(pipeline, cam, fid_placement, bl, prt):
#     for name, value in [
#         ("camera", cam),
#         ("placement", fid_placement),
#         ("boardLocation", bl),
#     ]:
#         try:
#             pipeline.setProperty(name, value)
#         except Exception, e:
#             log("Could not set pipeline %s property: %s" % (name, e))

#     try:
#         p = None
#         if hasattr(fid_placement, "getPart"):
#             p = fid_placement.getPart()
#         if p is None:
#             p = prt
#         if p is not None:
#             pipeline.setProperty("part", p)
#     except Exception, e:
#         log("Could not set pipeline part property: %s" % e)


# def measure_board_fiducial_correction(mac, bl, board, prt, label):
#     fid_placement, fid_loc, settings, fid_label = \
#         get_board_fiducial_placement_and_settings(board)

#     log("[%s] Using BOARD fiducial '%s' design location: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (label, fid_label, fid_loc.getX(), fid_loc.getY(),
#          fid_loc.getZ(), fid_loc.getRotation()))

#     pipeline = settings.getPipeline()
#     if pipeline is None:
#         raise Exception("Fiducial vision settings pipeline is null.")

#     bl_loc = bl.getLocation()
#     p_loc = Utils2D.calculateBoardPlacementLocation(bl, fid_loc)
#     head = mac.getDefaultHead()
#     cam = head.getDefaultCamera()
#     if cam is None:
#         raise Exception("Default head camera is null.")

#     prepare_pipeline_context(pipeline, cam, fid_placement, bl, prt)

#     base_cam_loc = cam.getLocation()
#     cam_target = Location(
#         p_loc.getUnits(),
#         p_loc.getX(),
#         p_loc.getY(),
#         base_cam_loc.getZ(),
#         base_cam_loc.getRotation()
#     )

#     last_reject = "not attempted"
#     for measurement_attempt in range(1, VISION_MEASUREMENT_ATTEMPTS + 1):
#         if measurement_attempt > 1:
#             Thread.sleep(RETRY_DELAY_MS)
#         log("[%s] Measurement attempt %d of %d." %
#             (label, measurement_attempt, VISION_MEASUREMENT_ATTEMPTS))

#         move_safe_wait(
#             cam, cam_target,
#             "%s camera nominal attempt %d" % (label, measurement_attempt))
#         initial_view_loc = cam_target

#         for recenter_pass in range(1, RECENTER_MAX_PASSES + 1):
#             wait_still(
#                 cam,
#                 "%s recenter pass %d camera" % (label, recenter_pass))
#             if PRE_CAPTURE_SETTLE_MS > 0:
#                 Thread.sleep(PRE_CAPTURE_SETTLE_MS)
#                 log("[%s] Recenter pass %d settled %d ms before pipeline.process()." %
#                     (label, recenter_pass, PRE_CAPTURE_SETTLE_MS))

#             try:
#                 pipeline.process()
#                 log("[%s] pipeline.process() OK attempt=%d pass=%d" %
#                     (label, measurement_attempt, recenter_pass))
#             except Exception, e:
#                 last_reject = "pipeline exception: %s" % e
#                 log("[%s] Rejected vision pass: %s" % (label, last_reject))
#                 break

#             rects = get_pipeline_rotated_rects(pipeline)
#             if len(rects) == 0:
#                 last_reject = "no RotatedRect results"
#                 log("[%s] Rejected vision pass: %s" % (label, last_reject))
#                 break

#             width, height = get_camera_image_size(cam)
#             rr, dist2 = choose_closest_rect_to_image_center(rects, width, height)
#             if rr is None:
#                 last_reject = "no usable RotatedRect results"
#                 log("[%s] Rejected vision pass: %s" % (label, last_reject))
#                 break

#             cam_loc = cam.getLocation()
#             upp_loc = get_camera_units_per_pixel(cam)
#             center_dx, center_dy, px, py = rect_center_machine_offset(
#                 cam_loc, upp_loc, rr, width, height)

#             # Match the original placement-starting correction convention:
#             # the measured machine-frame offset is applied as p_loc + dx/dy.
#             detected_loc = Location(
#                 cam_loc.getUnits(),
#                 cam_loc.getX() + center_dx,
#                 cam_loc.getY() + center_dy,
#                 cam_loc.getZ(),
#                 cam_loc.getRotation()
#             )
#             p_loc_bl_units = p_loc.convertToUnits(bl_loc.getUnits())
#             detected_bl_units = detected_loc.convertToUnits(bl_loc.getUnits())
#             board_dx = detected_bl_units.getX() - p_loc_bl_units.getX()
#             board_dy = detected_bl_units.getY() - p_loc_bl_units.getY()

#             log("[%s] attempt=%d pass=%d rects=%d chosen px=%.3f py=%.3f centered dx=%.6f dy=%.6f board dx=%.6f dy=%.6f angle=%.4f" %
#                 (label, measurement_attempt, recenter_pass, len(rects),
#                  px, py, center_dx, center_dy, board_dx, board_dy, rr.angle))

#             inside, view_dx, view_dy, half_x, half_y = \
#                 detection_inside_initial_view(
#                     initial_view_loc, detected_loc, upp_loc, width, height)
#             if not inside:
#                 last_reject = (
#                     "outside initial view dx=%.6f/%.6f dy=%.6f/%.6f" %
#                     (view_dx, half_x, view_dy, half_y))
#                 log("[%s] Rejected vision pass: %s" % (label, last_reject))
#                 break

#             if abs(board_dx) > MAX_NUDGE_MM or abs(board_dy) > MAX_NUDGE_MM:
#                 last_reject = (
#                     "beyond MAX_NUDGE_MM board dx=%.6f dy=%.6f limit=%.6f" %
#                     (board_dx, board_dy, MAX_NUDGE_MM))
#                 log("[%s] Rejected vision pass: %s" % (label, last_reject))
#                 break

#             if (abs(center_dx) <= RECENTER_TOLERANCE_MM and
#                     abs(center_dy) <= RECENTER_TOLERANCE_MM):
#                 new_rot = bl_loc.getRotation()
#                 if APPLY_ROTATION:
#                     new_rot = DESIGN_BOARD_ROT_DEG + (EXPECTED_ANGLE_DEG - rr.angle)
#                     log("[%s] Rotation newBoard=%.4f from measured=%.4f" %
#                         (label, new_rot, rr.angle))

#                 new_bl = make_board_location(
#                     bl_loc,
#                     bl_loc.getX() + board_dx,
#                     bl_loc.getY() + board_dy,
#                     bl_loc.getZ(),
#                     new_rot
#                 )

#                 log("[%s] Accepted centered fiducial correction -> board X=%.4f Y=%.4f R=%.4f" %
#                     (label, new_bl.getX(), new_bl.getY(), new_bl.getRotation()))

#                 return {
#                     "boardLocation": new_bl,
#                     "dx": board_dx,
#                     "dy": board_dy,
#                     "rotation": new_rot,
#                     "angle": rr.angle,
#                     "recenterPass": recenter_pass,
#                     "measurementAttempt": measurement_attempt,
#                 }

#             last_reject = (
#                 "not centered dx=%.6f dy=%.6f tolerance=%.6f" %
#                 (center_dx, center_dy, RECENTER_TOLERANCE_MM))
#             if recenter_pass < RECENTER_MAX_PASSES:
#                 log("[%s] Recenter pass %d moving camera to detected center: X=%.4f Y=%.4f" %
#                     (label, recenter_pass,
#                      detected_loc.getX(), detected_loc.getY()))
#                 move_camera_to_detected_center(
#                     cam, detected_loc,
#                     "%s recenter pass %d" % (label, recenter_pass))
#             else:
#                 log("[%s] Rejected vision attempt: %s" % (label, last_reject))

#         log("[%s] Measurement attempt %d failed: %s" %
#             (label, measurement_attempt, last_reject))

#     raise VisionMeasurementError(
#         "[%s] Board fiducial not confirmed after %d attempts; last reject: %s" %
#         (label, VISION_MEASUREMENT_ATTEMPTS, last_reject))


# def find_dispense_head(mac):
#     head = mac.getDefaultHead()
#     wanted = str(DISPENSE_HEAD_NAME)

#     for getter in ["getHeadMountables", "getNozzles"]:
#         try:
#             for item in java_list_to_py(getattr(head, getter)()):
#                 if object_name(item) == wanted:
#                     log("Using dispense head mountable '%s'." % object_name(item))
#                     return item
#         except:
#             pass

#     try:
#         item = head.getNozzleByName(wanted)
#         if item is not None:
#             log("Using dispense head mountable '%s'." % object_name(item))
#             return item
#     except:
#         pass

#     raise Exception("Required dispense head not found: %s" % wanted)


# def find_glue_dispense_actuator(mac):
#     wanted = str(DISPENSE_ACTUATOR_NAME)
#     owners = []
#     try:
#         owners.append(mac.getDefaultHead())
#     except:
#         pass
#     owners.append(mac)

#     for owner in owners:
#         if owner is None:
#             continue
#         for method in ["getActuatorByName", "getActuator"]:
#             try:
#                 actuator = getattr(owner, method)(wanted)
#                 if actuator is not None and object_name(actuator) == wanted:
#                     log("Using dispense actuator '%s'." % object_name(actuator))
#                     return actuator
#             except:
#                 pass
#         try:
#             for actuator in java_list_to_py(owner.getActuators()):
#                 if object_name(actuator) == wanted:
#                     log("Using dispense actuator '%s'." % object_name(actuator))
#                     return actuator
#         except:
#             pass

#     raise Exception("Required dispense actuator not found: %s" % wanted)


# def pad_xy(pad):
#     try:
#         return pad.getCenterX(), pad.getCenterY()
#     except:
#         return pad.getX(), pad.getY()


# def dispense_head_location(units, x, y, z):
#     return Location(units, x, y, z, Double.NaN)


# def sleep_ms(ms):
#     ms = int(ms)
#     if ms > 0:
#         Thread.sleep(ms)


# def wait_after_actuator_change(actuator, machine_obj, settle_ms, label):
#     if actuator is not None:
#         try:
#             actuator.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
#             log_t("%s: actuator wait complete" % label)
#             return
#         except:
#             pass
#     if machine_obj is not None:
#         try:
#             machine_obj.getMotionPlanner().waitForCompletion(
#                 None, CompletionType.WaitForUnconditionalCoordination)
#             log_t("%s: motion planner wait complete" % label)
#             return
#         except:
#             pass
#     sleep_ms(settle_ms)
#     log_t("%s: fallback sleep %d ms complete" % (label, int(settle_ms)))


# def pulse_glue_controller(actuator, machine_obj, pad_index):
#     t0 = log_t("pad %d: pulse_glue_controller START boolean pulse=%d ms settle=%d ms post=%d ms" %
#                (pad_index, DISPENSE_PULSE_MS, ACTUATOR_SETTLE_MS, POST_DISPENSE_DWELL_MS))

#     set_boolean_actuator_state_fast(actuator, False, "pad %d GlueDispense OFF before pulse" % pad_index)
#     wait_after_actuator_change(
#         actuator, machine_obj, ACTUATOR_SETTLE_MS,
#         "pad %d GlueDispense OFF wait" % pad_index)
#     sleep_ms(ACTUATOR_SETTLE_MS)

#     set_boolean_actuator_state_fast(actuator, True, "pad %d GlueDispense ON" % pad_index)
#     wait_after_actuator_change(
#         actuator, machine_obj, ACTUATOR_SETTLE_MS,
#         "pad %d GlueDispense ON wait" % pad_index)
#     sleep_ms(DISPENSE_PULSE_MS)

#     set_boolean_actuator_state_fast(actuator, False, "pad %d GlueDispense OFF after pulse" % pad_index)
#     wait_after_actuator_change(
#         actuator, machine_obj, ACTUATOR_SETTLE_MS,
#         "pad %d GlueDispense final OFF wait" % pad_index)
#     sleep_ms(ACTUATOR_SETTLE_MS)
#     sleep_ms(POST_DISPENSE_DWELL_MS)

#     log_t("pad %d: pulse_glue_controller END host elapsed=%.3f ms" %
#           (pad_index, elapsed_ms_since(t0)))


# def force_glue_off_safe(machine_obj, label):
#     if machine_obj is None:
#         try:
#             machine_obj = machine
#         except:
#             machine_obj = None
#     if machine_obj is None:
#         return
#     try:
#         glue = find_glue_dispense_actuator(machine_obj)
#         set_boolean_actuator_state_fast(glue, False, "%s GlueDispense forced off" % label)
#         wait_after_actuator_change(
#             glue, machine_obj, ACTUATOR_SETTLE_MS,
#             "%s GlueDispense forced off wait" % label)
#     except Exception, e:
#         log_err("%s failed to force GlueDispense off: %s" % (label, e))


# def dispense_current_placement(mac, board, bl, current_placement):
#     dispense_head = find_dispense_head(mac)
#     glue = find_glue_dispense_actuator(mac)
#     dispense_placement = get_dispense_placement(board, current_placement)
#     log_t("DISPENSE CONFIG pulse=%d post=%d pre=%d settle=%d lift=%.3f travelSpeed=%.3f dispenseSpeed=%.3f" %
#           (DISPENSE_PULSE_MS, POST_DISPENSE_DWELL_MS, PRE_DISPENSE_DWELL_MS,
#            ACTUATOR_SETTLE_MS,
#            INTER_PAD_LIFT_MM, TRAVEL_SPEED_FACTOR, DISPENSE_SPEED_FACTOR))

#     part = dispense_placement.getPart()
#     if part is None:
#         raise Exception("Dispense placement has no part.")
#     pkg = part.getPackage()
#     if pkg is None or pkg.getFootprint() is None:
#         raise Exception("Dispense part has no package footprint.")

#     pads = pkg.getFootprint().getPads()
#     if pads is None or pads.size() == 0:
#         raise Exception("Dispense footprint has no pads.")

#     active_bl_loc = bl.getLocation()
#     log("Dispense using ACTIVE corrected boardLocation X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (active_bl_loc.getX(), active_bl_loc.getY(),
#          active_bl_loc.getZ(), active_bl_loc.getRotation()))

#     world = Utils2D.calculateBoardPlacementLocation(
#         bl, dispense_placement.getLocation())
#     base_z = world.getZ() + DISPENSE_Z_OFFSET_MM
#     lift_z = base_z + INTER_PAD_LIFT_MM
#     units = world.getUnits()
#     pr = world.getRotation() * Math.PI / 180.0
#     c = Math.cos(pr)
#     s = Math.sin(pr)

#     log("Dispensing placement %s part %s footprint pads=%d using %s + %s" %
#         (object_name(dispense_placement), part.getId(), pads.size(),
#          object_name(dispense_head), object_name(glue)))
#     log("Dispense world origin X=%.4f Y=%.4f Z=%.4f R=%.4f baseZ=%.4f liftZ=%.4f" %
#         (world.getX(), world.getY(), world.getZ(), world.getRotation(), base_z, lift_z))
#     log("Dispense needle B/rotation axis blocked: all dispense-head move targets use R=NaN.")

#     set_boolean_actuator_state_fast(glue, False, "glue initial off")

#     start = dispense_head_location(units, world.getX(), world.getY(), DISPENSE_SAFE_Z_MM)
#     move_safe_wait(dispense_head, start, "dispense head start")

#     dispensed = 0
#     dispense_t0 = log_t("DISPENSE START pads=%d" % pads.size())
#     for i in range(pads.size()):
#         pad_t0 = log_t("pad %d: START" % i)
#         pad = pads.get(i)
#         pad_name = str(pad.getName())
#         if pad_name in PAD_SKIP_NAMES:
#             log("Skipping pad %d named %s." % (i, pad_name))
#             continue

#         px, py = pad_xy(pad)
#         wx = world.getX() + (px * c - py * s)
#         wy = world.getY() + (px * s + py * c)

#         log("Pad %d '%s': local X=%.4f Y=%.4f -> world X=%.4f Y=%.4f R=NaN" %
#             (i, pad_name, px, py, wx, wy))

#         move_direct(
#             dispense_head,
#             dispense_head_location(units, wx, wy, lift_z),
#             TRAVEL_SPEED_FACTOR,
#             "dispense pad %d lift" % i)
#         move_z_only(
#             dispense_head,
#             units,
#             base_z,
#             DISPENSE_SPEED_FACTOR,
#             "dispense pad %d down Z-only" % i)
#         log_t("pad %d: BEFORE pre-glue wait_coord" % i)
#         wait_coord(dispense_head, "dispense pad %d pre-glue" % i)
#         log_t("pad %d: AFTER pre-glue wait_coord / AT DISPENSE Z" % i)
#         log("Dispense head reached pad %d at dispense Z." % i)
#         sleep_ms(PRE_DISPENSE_DWELL_MS)
#         log_t("pad %d: BEFORE pulse_glue_controller" % i)
#         pulse_glue_controller(glue, mac, i)
#         log_t("pad %d: AFTER pulse_glue_controller" % i)
#         log_t("pad %d: BEFORE retract queue" % i)
#         move_z_only(
#             dispense_head,
#             units,
#             lift_z,
#             DISPENSE_SPEED_FACTOR,
#             "dispense pad %d retract Z-only" % i)
#         log_t("pad %d: AFTER retract queue; completion waits at next pre-glue or final drain" % i)
#         log_t("pad %d: END total elapsed=%.3f ms" % (i, elapsed_ms_since(pad_t0)))
#         dispensed += 1

#     log_t("DISPENSE BEFORE final retract/queue drain")
#     wait_coord(dispense_head, "dispense final retract/queue drain")
#     log_t("DISPENSE AFTER final retract/queue drain")
#     log_t("DISPENSE END total elapsed=%.3f ms dispensed=%d" %
#           (elapsed_ms_since(dispense_t0), dispensed))
#     set_boolean_actuator_state_fast(glue, False, "glue final off")
#     end = dispense_head_location(units, world.getX(), world.getY(), DISPENSE_SAFE_Z_MM)
#     move_safe_wait(dispense_head, end, "dispense head safe end")
#     log("Dispense complete: %d pads pulsed." % dispensed)
#     return dispensed


# def update_bound_placement_location(bl, current_placement):
#     final_loc = Utils2D.calculateBoardPlacementLocation(
#         bl, current_placement.getLocation())

#     log("[post-dispense] Final die placementLocation from updated boardLocation: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (final_loc.getX(), final_loc.getY(),
#          final_loc.getZ(), final_loc.getRotation()))

#     if "placementLocation" not in globals():
#         log("[post-dispense] No placementLocation binding present; boardLocation only was updated.")
#         return final_loc

#     old_loc = placementLocation
#     try:
#         log("[post-dispense] Existing bound placementLocation before update: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#             (old_loc.getX(), old_loc.getY(),
#              old_loc.getZ(), old_loc.getRotation()))
#     except:
#         log("[post-dispense] Existing bound placementLocation before update: %s" %
#             str(old_loc))

#     applied = False
#     try:
#         if hasattr(old_loc, "setLocation"):
#             old_loc.setLocation(final_loc)
#             applied = True
#             log("[post-dispense] Applied final die target via placementLocation.setLocation(...).")
#     except Exception, e:
#         log("[post-dispense] placementLocation.setLocation(...) failed: %s" % e)

#     if not applied:
#         globals()["placementLocation"] = final_loc
#         log("[post-dispense] Rebound placementLocation global to final die target.")

#     try:
#         updated_loc = placementLocation
#         log("[post-dispense] Bound placementLocation after update: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#             (updated_loc.getX(), updated_loc.getY(),
#              updated_loc.getZ(), updated_loc.getRotation()))
#     except:
#         pass

#     return final_loc


# def disable_current_placement(pl, message):
#     try:
#         if hasattr(pl, "setEnabled"):
#             pl.setEnabled(False)
#             log("placement.setEnabled(False) called.")
#     except Exception, e:
#         log("Failed to disable placement: %s" % e)
#     try:
#         existing = pl.getComments() or ""
#         if existing:
#             pl.setComments(existing + " " + message)
#         else:
#             pl.setComments(message)
#     except:
#         pass


# original_bl_loc = None
# final_update_done = False


# try:
#     required_globals = ("placement", "boardLocation", "machine")
#     missing = [name for name in required_globals if name not in globals()]
#     if missing:
#         raise Exception("Missing globals: %s" % ", ".join(missing))

#     pl = placement
#     bl = boardLocation
#     mac = machine
#     prt = part if "part" in globals() else None

#     pid = prt.getId() if (prt is not None and hasattr(prt, "getId")) else "<no-part-id>"
#     log("Part id=%s" % pid)

#     board = None
#     try:
#         board = bl.getBoard()
#     except:
#         pass
#     if board is None:
#         raise Exception("boardLocation.getBoard() returned null.")

#     log("Board object: %s" % board)

#     original_bl_loc = bl.getLocation()
#     log("Original boardLocation: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (original_bl_loc.getX(), original_bl_loc.getY(),
#          original_bl_loc.getZ(), original_bl_loc.getRotation()))

#     if SKIP_DISPENSE:
#         log("SKIP_DISPENSE is True; skipping pre-dispense vision and adhesive dispense.")
#     else:
#         pre = measure_board_fiducial_correction(mac, bl, board, prt, "pre-dispense")
#         pre_bl_loc = pre["boardLocation"]
#         log("[pre-dispense] Temporary boardLocation for dispense: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#             (pre_bl_loc.getX(), pre_bl_loc.getY(),
#              pre_bl_loc.getZ(), pre_bl_loc.getRotation()))
#         bl.setLocation(pre_bl_loc)
#         log("[pre-dispense] Temporary boardLocation is active for dispense and post inspection.")
#         active_pre_loc = bl.getLocation()
#         log("[pre-dispense] Active boardLocation readback before syringe dispense: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#             (active_pre_loc.getX(), active_pre_loc.getY(),
#              active_pre_loc.getZ(), active_pre_loc.getRotation()))

#         dispense_current_placement(mac, board, bl, pl)

#     post = measure_board_fiducial_correction(mac, bl, board, prt, "post-dispense")
#     final_bl_loc = post["boardLocation"]
#     bl.setLocation(final_bl_loc)
#     update_bound_placement_location(bl, pl)
#     final_update_done = True
#     log("[post-dispense] Updated boardLocation: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (final_bl_loc.getX(), final_bl_loc.getY(),
#          final_bl_loc.getZ(), final_bl_loc.getRotation()))
#     if SKIP_DISPENSE:
#         log("Skip-dispense post alignment flow complete.")
#     else:
#         log("Combined pre-dispense, dispense, post-dispense flow complete.")

# except SystemExit:
#     pass

# except VisionMeasurementError, e:
#     log_err(e)
#     try:
#         force_glue_off_safe(mac, "vision failure")
#     except:
#         pass
#     if original_bl_loc is not None and not final_update_done:
#         try:
#             bl.setLocation(original_bl_loc)
#             log("Restored original boardLocation after vision failure: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#                 (original_bl_loc.getX(), original_bl_loc.getY(),
#                  original_bl_loc.getZ(), original_bl_loc.getRotation()))
#         except Exception, restore_e:
#             log_err("Failed to restore original boardLocation: %s" % restore_e)
#     try:
#         disable_current_placement(
#             placement,
#             "[auto-skip: board fiducial vision not confirmed %s]" % e)
#     except:
#         pass
#     raise SystemExit

# except Exception, e:
#     log_err(e)
#     try:
#         force_glue_off_safe(mac, "placement-starting error")
#     except:
#         pass
#     if original_bl_loc is not None and not final_update_done:
#         try:
#             bl.setLocation(original_bl_loc)
#             log("Restored original boardLocation after failed pre/dispense/post flow: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#                 (original_bl_loc.getX(), original_bl_loc.getY(),
#                  original_bl_loc.getZ(), original_bl_loc.getRotation()))
#         except Exception, restore_e:
#             log_err("Failed to restore original boardLocation: %s" % restore_e)
#     try:
#         disable_current_placement(placement, "[auto-skip: placement-starting error %s]" % e)
#     except:
#         pass
#     raise

#  ----------- DEPRICATED SCRIPT --------------
# DAK: Removed on 06/04/2026. Was working also used old stepping dispense. New script combines dispense into one (ie Job.Placement.Starting.Py)
# Handles correcting the offset rotation and position of ALL the plastic manifold (filter disk body/ mwi body),
# to be assembled. While doing so it will run through and dispense adhesive(PDMS) if enabled.
#
#
# Note: 
#   - This may be depricated in the future with the new dispense technique and will be wombo comboed with the Job.Placement.Starting.py scirpt
# 
#   - runs once when the job begins (script iterates over boards itself).
#
# Job.Starting.py
#
# Dispensing script using board-level fiducial(s) to correct boardLocation,
# then walking the footprint pads of the dispense part and moving a
# "Dispense Head" around to each pad.
#
# - Event: Job.Starting (or Job.Starting if you bound it there)
# - Uses BOARD fiducial placement + its vision pipeline
# - If fiducial vision fails after N attempts: disables that BoardLocation
#   and SKIPS dispensing for that board.
# - Dispense is done by moving a HeadMountable named "Dispense Head"
#   (rotation of that head is used to "pump" adhesive).

#
# *************************
# *****   CONFIG      *****
# *************************

DISABLE_DISPENSING      = True   # True = script does NOTHING (no moves, no correction)
DRY_RUN                 = True    # True = move to pads, but don't rotate/pump or draw back

TAG = "Job.Placement.Starting"

# --- dispense hardware config ---
DISPENSE_HEAD_NAME      = "Dispense Head"  # HeadMountable.getName() or .getId()
MOVE_SPEED_FACTOR       = 0.74            # matches log style moveTo(..., 0.74)

# Z plan:
#   baseZ    = worldLoc.getZ() + DISPENSE_Z_OFFSET_MM
#   retractZ = baseZ + RETRACT_Z_MM
DISPENSE_Z_OFFSET_MM         = 0   # RELATIVE TO job placement Z (worldLoc Z)
RETRACT_Z_MM                 = 0.1   # retract above baseZ between dots

# Rotation-based pumping:
PRE_PRIME_STEP_DEG          = 100  # initial extra rotation before the first pad
DISPENSE_STEP_DEG           = 2   # degrees of rotation per droplet (tune!)
INTER_BOARD_PRIME_STEP_DEG  = 5   # extra push at the start of each pad on non-final boards

# Draw-back behavior:
# - After each board (except last): small retract
# - After the FINAL board only: big cleanup retract
INTER_BOARD_DRAWBACK_STEP_DEG = 5   # small retract between boards
FINAL_DRAWBACK_STEP_DEG       = 200  # big cleanup retract after last board
DRAW_BACK_SETTLE = 100  # ms to wait after draw-back move

# --- vision / fiducial config ---
MAX_VISION_ATTEMPTS     = 3
PRE_CAPTURE_SETTLE_MS   = 250
PRE_PIPELINE_SETTLE_MS  = 1000
RESULT_STAGE_KEY        = "results"
EXPECTED_ANGLE_DEG      = 0.0
DESIGN_BOARD_ROT_DEG    = 0.0   # what you WANT the board rotation to be

# --- pad selection config ---
PAD_SKIP_NAMES          = set()  # e.g. {"PAD1", "PAD2"}

# Image-to-machine sign controls for fiducial offset
FLIP_IMAGE_X = False
FLIP_IMAGE_Y = True

# Optional park/cleanup location (leave None if you don't want it)
CLEANUP_LOCATION = None
# CLEANUP_LOCATION = (
#     41.013,
#     216.252,
#     15.0
# )

from org.openpnp.model import Location
from org.openpnp.util import MovableUtils, Utils2D
from org.opencv.core import RotatedRect
from java.lang import Math, Thread
from java.lang import Thread as JThread
from javax.swing import JOptionPane

# ------------- LOG HELPERS -------------
def log(msg):
    print "[%s] %s" % (TAG, msg)

def log_err(msg):
    print "[%s ERROR] %s" % (TAG, msg)


# ------------- ABORT HELPER -------------
def shouldAbort():
    """
    Best-effort check if the job has been stopped / canceled.
    NOTE: For Job.Placement.Starting, OpenPnP generally cannot hard-kill
    a running script; Stop will usually wait until we return.
    """
    try:
        if JThread.currentThread().isInterrupted():
            log("Abort: thread is interrupted.")
            return True
    except:
        pass

    try:
        if 'job' in globals():
            j = job
            if hasattr(j, "isCancelled") and j.isCancelled():
                log("Abort: job.isCancelled() is True.")
                return True
            if hasattr(j, "getState"):
                try:
                    st = j.getState()
                    if st is not None and st.toString().lower() in ("canceled", "cancelled", "stopped"):
                        log("Abort: job state is %s." % st)
                        return True
                except:
                    pass
    except:
        pass

    return False


# ------------- HELPER: FIND DISPENSE HEAD -------------
def findDispenseHead(mac):
    head = mac.getDefaultHead()
    try:
        mnts = head.getHeadMountables()
    except:
        mnts = None

    if mnts is not None:
        for m in mnts:
            name = ""
            mid  = ""
            try:
                name = m.getName()
            except:
                pass
            try:
                mid = m.getId()
            except:
                pass
            if name == DISPENSE_HEAD_NAME or mid == DISPENSE_HEAD_NAME:
                log("Using dispense head mountable '%s' (%s)" % (name, mid))
                return m

    noz = head.getDefaultNozzle()
    log("WARNING: Could not find head mountable '%s', using default nozzle %s"
        % (DISPENSE_HEAD_NAME, noz))
    return noz


# ------------- HELPER: CHECK IF BOARD IS ALREADY PLACED -------------
def boardAlreadyPlaced(bl):
    """
    Determine if all placements associated with this BoardLocation are marked placed.
    We try:
      - bl.getJobPlacements() if it exists
      - otherwise bl.getPlacements()
    We then look for isPlaced()/getPlaced() on each placement.
    If anything looks unplaced -> return False.
    If we can't figure it out -> assume NOT placed (i.e. we will dispense).
    """
    placements = None
    src = None

    # Try board-location-level job placements first
    try:
        if hasattr(bl, "getJobPlacements"):
            placements = bl.getJobPlacements()
            src = "bl.getJobPlacements()"
    except Exception, e:
        log("Error calling bl.getJobPlacements(): %s" % e)

    # Fallback: generic placements on the boardLocation
    if placements is None:
        try:
            if hasattr(bl, "getPlacements"):
                placements = bl.getPlacements()
                src = "bl.getPlacements()"
        except Exception, e:
            log("Error calling bl.getPlacements(): %s" % e)

    if placements is None:
        log("boardAlreadyPlaced: no job/board placements list on BoardLocation; treating as NOT placed.")
        return False

    try:
        size = placements.size()
    except:
        try:
            size = len(placements)
        except:
            size = None

    log("boardAlreadyPlaced: using %s, size=%s" % (src, size))

    found_any = False
    all_placed = True

    # Support both Java List (size/get) and Python iterable
    def _iter(pls):
        if hasattr(pls, "size") and hasattr(pls, "get"):
            for i in range(pls.size()):
                yield pls.get(i)
        else:
            for p in pls:
                yield p

    try:
        for jp in _iter(placements):
            found_any = True
            placed = None

            if hasattr(jp, "isPlaced"):
                try:
                    placed = bool(jp.isPlaced())
                except:
                    placed = None

            if placed is None and hasattr(jp, "getPlaced"):
                try:
                    placed = bool(jp.getPlaced())
                except:
                    placed = None

            # If we can't tell, treat as NOT placed
            if placed is None:
                log("boardAlreadyPlaced: placement %s has no isPlaced/getPlaced; treating as NOT placed." % jp)
                all_placed = False
                break

            if not placed:
                all_placed = False
                break

        if not found_any:
            log("boardAlreadyPlaced: no placements found on boardLocation; treating as NOT placed.")
            return False

        return all_placed
    except Exception, e:
        log("boardAlreadyPlaced: error iterating placements: %s; treating as NOT placed." % e)
    return False


# ------------- HELPER: BUILD CLEANUP LOCATION -------------
def makeCleanupLocation(units):
    """
    Convert CLEANUP_LOCATION config into a Location with zero rotation.
    Accepts:
      - None: returns None
      - org.openpnp.model.Location: returned as-is (rotation unchanged)
      - tuple/list of (x, y, z) or (x, y, z, rot); rot defaults to 0
    """
    if CLEANUP_LOCATION is None:
        return None

    if isinstance(CLEANUP_LOCATION, Location):
        return CLEANUP_LOCATION

    try:
        coords = list(CLEANUP_LOCATION)
    except Exception, e:
        log("Invalid CLEANUP_LOCATION (%s); ignoring." % e)
        return None

    if len(coords) < 3:
        log("Invalid CLEANUP_LOCATION (need at least x,y,z); ignoring.")
        return None

    x, y, z = coords[0], coords[1], coords[2]
    rot = coords[3] if len(coords) > 3 else 0.0

    try:
        return Location(units, float(x), float(y), float(z), float(rot))
    except Exception, e:
        log("Failed to build cleanup Location: %s" % e)
        return None


# ------------- HELPER: WAIT FOR USER READY -------------
def waitForReadyPrompt():
    """
    Show a modal prompt asking the user to clean the nozzle and continue.
    Returns True if the user pressed Ready/OK, False if cancelled/closed.
    """
    try:
        res = JOptionPane.showConfirmDialog(
            None,
            "clean nozzle dripage. Ready to proceed",
            "Dispense complete",
            JOptionPane.OK_CANCEL_OPTION,
            JOptionPane.WARNING_MESSAGE
        )
        return res == JOptionPane.OK_OPTION
    except Exception, e:
        log("Ready prompt failed (%s); continuing without confirmation." % e)
        return True


# ------------- HELPER: RUN FIDUCIAL VISION ON ONE BOARD -------------
def correctBoardLocationWithFiducial(mac, bl):
    if shouldAbort():
        return False

    board = bl.getBoard()
    if board is None:
        log_err("BoardLocation has no Board; skipping this boardLocation.")
        return False

    log("Board object: %s" % board)

    fidPlacements = []
    for placement in board.getPlacements():
        if shouldAbort():
            return False
        if placement.getType().toString() != "Fiducial":
            continue
        part = placement.getPart()
        if part is None:
            continue
        vs = part.getFiducialVisionSettings()
        if vs is None or vs.getPipeline() is None:
            continue
        try:
            if not placement.isEnabled():
                continue
        except:
            pass
        fidPlacements.append(placement)

    if not fidPlacements:
        log_err("No ENABLED BOARD fiducials with vision pipelines found; skipping this boardLocation.")
        return False

    fidPlacement = fidPlacements[0]
    fidPart      = fidPlacement.getPart()
    fidSettings  = fidPart.getFiducialVisionSettings()
    pipeline     = fidSettings.getPipeline()

    log("Using BOARD fiducial placement %s with part id %s and vision settings %s"
        % (fidPlacement, fidPart.getId(), fidSettings))

    fidDesignLoc = fidPlacement.getLocation()
    log("Board fiducial design location (board-relative): (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (fidDesignLoc.getX(), fidDesignLoc.getY(),
         fidDesignLoc.getZ(), fidDesignLoc.getRotation()))

    blLoc = bl.getLocation()
    fidNominalLoc = Utils2D.calculateBoardPlacementLocation(bl, fidDesignLoc)
    log("Nominal fiducial machine location: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (fidNominalLoc.getX(), fidNominalLoc.getY(),
         fidNominalLoc.getZ(), fidNominalLoc.getRotation()))

    head = mac.getDefaultHead()
    cam  = head.getDefaultCamera()
    if cam is None:
        raise Exception("Default head camera is null.")

    camLoc = cam.getLocation()
    camTarget = Location(
        fidNominalLoc.getUnits(),
        fidNominalLoc.getX(),
        fidNominalLoc.getY(),
        camLoc.getZ(),
        camLoc.getRotation()
    )

    MovableUtils.moveToLocationAtSafeZ(cam, camTarget)
    log("Camera @ (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (camTarget.getX(), camTarget.getY(), camTarget.getZ(), camTarget.getRotation()))

    rr = None
    attempt = 0

    while attempt < MAX_VISION_ATTEMPTS and rr is None:
        if shouldAbort():
            return False

        attempt += 1
        log("Vision attempt %d of %d" % (attempt, MAX_VISION_ATTEMPTS))

        if PRE_CAPTURE_SETTLE_MS > 0:
            Thread.sleep(PRE_CAPTURE_SETTLE_MS)
            log("Settled %d ms before pipeline.process()." % PRE_CAPTURE_SETTLE_MS)

        if PRE_PIPELINE_SETTLE_MS > 0:
            Thread.sleep(PRE_PIPELINE_SETTLE_MS)

        pipeline.process()
        log("pipeline.process() OK on attempt %d" % attempt)

        candidate_keys = [RESULT_STAGE_KEY, "results", "preResults", "8"]
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
                rr = model
                log("Got RotatedRect directly from results key '%s' on attempt %d." %
                    (key, attempt))
                break

            if hasattr(model, "size") and hasattr(model, "get"):
                if model.size() > 0:
                    first = model.get(0)
                    if isinstance(first, RotatedRect) or hasattr(first, "center"):
                        rr = first
                        log("Got RotatedRect[0] from list at results key '%s' on attempt %d." %
                            (key, attempt))
                        break

    if rr is None:
        log_err("Fiducial vision FAILED after %d attempts for this BoardLocation." %
                MAX_VISION_ATTEMPTS)
        return False

    log("Measured RotatedRect angle: %.4f deg" % rr.angle)

    image = cam.capture()
    w = image.getWidth()
    h = image.getHeight()
    log("Captured image for geometry: %dx%d" % (w, h))

    px = rr.center.x - (w / 2.0)
    py = rr.center.y - (h / 2.0)
    log("px,py=%.3f,%.3f" % (px, py))

    if FLIP_IMAGE_X:
        px = -px
    if FLIP_IMAGE_Y:
        py = -py

    if hasattr(cam, "getUnitsPerPixelAtZ"):
        uppLoc = cam.getUnitsPerPixelAtZ()
    else:
        uppLoc = cam.getUnitsPerPixel()

    if uppLoc is None:
        raise Exception("Camera units-per-pixel not configured (uppLoc is null).")

    upx = uppLoc.getX()
    upy = uppLoc.getY()
    log("Units per pixel: upx=%.9f, upy=%.9f at Z=%.3f" %
        (upx, upy, uppLoc.getZ()))

    dx_cam = px * upx
    dy_cam = py * upy
    log("Camera-frame dx_cam=%.6f, dy_cam=%.6f" % (dx_cam, dy_cam))

    rad = camLoc.getRotation() * Math.PI / 180.0
    c   = Math.cos(rad)
    s   = Math.sin(rad)

    dx = dx_cam * c - dy_cam * s
    dy = dx_cam * s + dy_cam * c
    log("Machine-frame dx=%.6f, dy=%.6f" % (dx, dy))

    measuredAngle = rr.angle
    newBoardRot   = DESIGN_BOARD_ROT_DEG + (EXPECTED_ANGLE_DEG - measuredAngle)
    dtheta        = newBoardRot - blLoc.getRotation()

    correctedFid = Location(
        fidNominalLoc.getUnits(),
        fidNominalLoc.getX() + dx,
        fidNominalLoc.getY() + dy,
        fidNominalLoc.getZ(),
        newBoardRot
    )

    log("Rotation tweak dTheta=%.3f deg (measured=%.3f, designBoard=%.3f, newBoard=%.3f)" %
        (dtheta, measuredAngle, DESIGN_BOARD_ROT_DEG, newBoardRot))

    log("CORRECTED fiducial machine target (debug): (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (correctedFid.getX(), correctedFid.getY(),
         correctedFid.getZ(), correctedFid.getRotation()))

    newBl = Location(
        blLoc.getUnits(),
        blLoc.getX() + dx,
        blLoc.getY() + dy,
        blLoc.getZ(),
        newBoardRot
    )
    bl.setLocation(newBl)
    log("Updated boardLocation: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (newBl.getX(), newBl.getY(), newBl.getZ(), newBl.getRotation()))

    return True


# ------------- HELPER: DISPENSE ON ONE BOARD -------------
def dispenseOnBoard(mac, bl, dispenseHead, pre_prime_done, is_last_board, currentRot):
    """
    Returns (units, currentRot, lastPadX, lastPadY, retractZ, pre_prime_done) on success,
    or None if nothing dispensed / early exit.
    """
    if shouldAbort():
        return None

    board = bl.getBoard()
    if board is None:
        log_err("dispenseOnBoard: BoardLocation has no Board.")
        return None

    dispensePlacement = None
    for p in board.getPlacements():
        if shouldAbort():
            return None
        if p.getType().toString() != "Placement":
            continue
        dispensePlacement = p
        break

    if dispensePlacement is None:
        log("No normal Placement found on this board; nothing to dispense.")
        return None

    part = dispensePlacement.getPart()
    if part is None:
        log_err("Dispense placement has no part; skipping.")
        return None

    pkg = part.getPackage()
    if pkg is None or pkg.getFootprint() is None:
        log_err("Part %s has no package/footprint; skipping." % part.getId())
        return None

    footprint = pkg.getFootprint()
    pads = footprint.getPads()
    if pads is None or pads.size() == 0:
        log("Footprint has no pads; nothing to dispense.")
        return None

    dLoc = dispensePlacement.getLocation()  # board-relative
    worldLoc = Utils2D.calculateBoardPlacementLocation(bl, dLoc)
    log("Dispense using placement %s, part id %s, package id %s, footprint %s with %d pads." %
        (dispensePlacement, part.getId(), pkg.getId(), footprint, pads.size()))
    log("Dispense part world location: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (worldLoc.getX(), worldLoc.getY(), worldLoc.getZ(), worldLoc.getRotation()))

    baseZ    = worldLoc.getZ() + DISPENSE_Z_OFFSET_MM
    retractZ = baseZ + RETRACT_Z_MM
    log("Dispense Z plan: worldZ=%.4f, baseZ=%.4f, retractZ=%.4f (offset=%.4f, retract=%.4f)" %
        (worldLoc.getZ(), baseZ, retractZ, DISPENSE_Z_OFFSET_MM, RETRACT_Z_MM))

    units = worldLoc.getUnits()

    # Track last pad position to use for draw-back
    lastPadX = worldLoc.getX()
    lastPadY = worldLoc.getY()

    for i in range(pads.size()):
        if shouldAbort():
            log("Abort requested; stopping pad loop.")
            return (units, currentRot, lastPadX, lastPadY, retractZ, pre_prime_done)

        pad = pads.get(i)
        padName = pad.getName()
        if padName in PAD_SKIP_NAMES:
            log("Skipping pad %d named %s (in skip list)." % (i, padName))
            continue

        try:
            try:
                padX = pad.getCenterX()
                padY = pad.getCenterY()
            except:
                padX = pad.getX()
                padY = pad.getY()

            padRotLocal = pad.getRotation()

            pr = worldLoc.getRotation() * Math.PI / 180.0
            c  = Math.cos(pr)
            s  = Math.sin(pr)

            dpx = padX * c - padY * s
            dpy = padX * s + padY * c

            padWorldX = worldLoc.getX() + dpx
            padWorldY = worldLoc.getY() + dpy
            padWorldR = worldLoc.getRotation() + padRotLocal

            lastPadX = padWorldX
            lastPadY = padWorldY

            log("Pad %d '%s': local(%.4f, %.4f) -> world(%.4f, %.4f), baseZ=%.4f" %
                (i, padName, padX, padY, padWorldX, padWorldY, baseZ))

            # --- NO SAFE MOVES HERE: straight moves only ---

            # Move to retractZ at pad XY
            approachLoc = Location(
                units,
                padWorldX,
                padWorldY,
                retractZ,
                currentRot
            )
            dispenseHead.moveTo(approachLoc, MOVE_SPEED_FACTOR)

            # Drop to dispense Z
            dispenseLoc = Location(
                units,
                padWorldX,
                padWorldY,
                baseZ,
                currentRot
            )
            dispenseHead.moveTo(dispenseLoc, MOVE_SPEED_FACTOR)

            # Inter-board priming: small extra push on every pad except the final board
            #move negative direction to dispense
            if (not is_last_board and
                not DISABLE_DISPENSING and
                not DRY_RUN and
                INTER_BOARD_PRIME_STEP_DEG > 0):
                currentRot -= INTER_BOARD_PRIME_STEP_DEG
                interPrimeLoc = Location(
                    units,
                    padWorldX,
                    padWorldY,
                    baseZ,
                    currentRot
                )
                log("Letting nozzle tip get to location %d ms..." % DRAW_BACK_SETTLE)
                Thread.sleep(DRAW_BACK_SETTLE)

                dispenseHead.moveTo(interPrimeLoc, MOVE_SPEED_FACTOR)
                log("Applied inter-board prime of %.3f deg at pad %d." %
                    (INTER_BOARD_PRIME_STEP_DEG, i))

            # One-time pre-prime before the very first dispense
            if (not pre_prime_done and
                not DISABLE_DISPENSING and
                not DRY_RUN and
                PRE_PRIME_STEP_DEG > 0):
                currentRot -= PRE_PRIME_STEP_DEG
                primeLoc = Location(
                    units,
                    padWorldX,
                    padWorldY,
                    baseZ,
                    currentRot
                )
                
                log("Letting nozzle tip get to location %d ms..." % DRAW_BACK_SETTLE)
                Thread.sleep(DRAW_BACK_SETTLE)

                dispenseHead.moveTo(primeLoc, MOVE_SPEED_FACTOR)
                pre_prime_done = True
                log("Performed pre-prime of %.3f deg before first pad dispense." % PRE_PRIME_STEP_DEG)

            # Pump
            if not DISABLE_DISPENSING and not DRY_RUN:
                currentRot -= DISPENSE_STEP_DEG
                pumpLoc = Location(
                    units,
                    padWorldX,
                    padWorldY,
                    baseZ,
                    currentRot
                )
                dispenseHead.moveTo(pumpLoc, MOVE_SPEED_FACTOR)
            else:
                log("Dry run / dispensing disabled: skipping pump at pad %d." % i)

            # Retract straight up to retractZ
            backLoc = Location(
                units,
                padWorldX,
                padWorldY,
                retractZ,
                currentRot
            )
            dispenseHead.moveTo(backLoc, MOVE_SPEED_FACTOR)

        except Exception, e:
            log("ERROR dispensing at pad %d: %s" % (i, e))

    

    # No draw-back here; done in main loop so we can distinguish
    # inter-board vs final.
    return (units, currentRot, lastPadX, lastPadY, retractZ, pre_prime_done)


# ------------- MAIN BODY -------------
try:
    if 'job' not in globals() or 'machine' not in globals():
        raise Exception("Missing globals job or machine.")

    # HARD skip: no motion, no correction, nothing.
    if DISABLE_DISPENSING:
        log("DISABLE_DISPENSING is True; script will EXIT without any fiducial correction or dispensing.")
    else:
        mac = machine
        dispenseHead = findDispenseHead(mac)

        # Start from the current physical rotation of the dispense head
        try:
            pumpCurrentRot = dispenseHead.getLocation().getRotation()
        except:
            pumpCurrentRot = 0.0
        log("Starting dispense head rotation: %.3f deg" % pumpCurrentRot)

        # Collect enabled BoardLocations first
        enabled_bls = []
        for bl in job.getBoardLocations():
            if shouldAbort():
                log("Abort requested before processing boardLocations.")
                break
            try:
                if not bl.isEnabled():
                    continue
            except:
                pass
            enabled_bls.append(bl)

        log("Found %d enabled boardLocations in job." % len(enabled_bls))

        # Filter out boards whose placements are already marked as placed
        boards_to_dispense = []
        for bl in enabled_bls:
            if boardAlreadyPlaced(bl):
                loc = bl.getLocation()
                log("BoardLocation at (X=%.4f, Y=%.4f) is already placed; skipping dispensing." %
                    (loc.getX(), loc.getY()))
                continue
            boards_to_dispense.append(bl)

        log("Found %d boardLocations that still need dispensing (based on board-level placed flags)." %
            len(boards_to_dispense))

        lastBoardDrawbackInfo = None  # (units, rot, x, y, retractZ)
        pre_prime_done = False

        for idx, bl in enumerate(boards_to_dispense):
            if shouldAbort():
                log("Abort requested; breaking out of board loop.")
                break

            blLoc = bl.getLocation()
            isLast = (idx == len(boards_to_dispense) - 1)
            log("Processing BoardLocation %d/%d at (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f), isLast=%s" %
                (idx+1, len(boards_to_dispense),
                 blLoc.getX(), blLoc.getY(), blLoc.getZ(), blLoc.getRotation(),
                 isLast))

            ok = correctBoardLocationWithFiducial(mac, bl)
            if not ok:
                try:
                    bl.setEnabled(False)
                except:
                    pass
                    log_err("Disabled BoardLocation due to fiducial failure; skipping dispensing for this board.")
                continue

            # Dispense on this board
            info = dispenseOnBoard(mac, bl, dispenseHead, pre_prime_done, isLast, pumpCurrentRot)
            if info is None:
                continue

            # info = (units, currentRot, lastPadX, lastPadY, retractZ, pre_prime_done)
            units, pumpCurrentRot, lastPadX, lastPadY, retractZ, pre_prime_done = info
            lastBoardDrawbackInfo = (units, pumpCurrentRot, lastPadX, lastPadY, retractZ)

            # Inter-board draw-back (small) – only if NOT last board and not in dry run.
            if (not isLast and
                not DRY_RUN and
                INTER_BOARD_DRAWBACK_STEP_DEG > 0 and
                not shouldAbort()):

                try:
                    newRot = pumpCurrentRot + INTER_BOARD_DRAWBACK_STEP_DEG #move positive direction to draw back
                    retractZ += 1 #pull z up a little bit
                    dbLoc = Location(
                        units,
                        lastPadX,
                        lastPadY,
                        retractZ,
                        newRot
                    )
                    dispenseHead.moveTo(dbLoc, MOVE_SPEED_FACTOR)
                    log("Letting nozzle tip get to location %d ms..." % DRAW_BACK_SETTLE)
                    Thread.sleep(DRAW_BACK_SETTLE)
                    pumpCurrentRot = newRot
                    lastBoardDrawbackInfo = (units, pumpCurrentRot, lastPadX, lastPadY, retractZ)
                    log("Performed INTER-BOARD draw-back of %.3f deg at end of board %d." %
                        (INTER_BOARD_DRAWBACK_STEP_DEG, idx+1))
                except Exception, e:
                    log("ERROR performing inter-board draw-back: %s" % e)

        # Final big cleanup draw-back after *last* board only
        if (lastBoardDrawbackInfo is not None and
            not DRY_RUN and
            FINAL_DRAWBACK_STEP_DEG > 0 and
            not shouldAbort()):

            try:
                units, pumpCurrentRot, lastPadX, lastPadY, retractZ = lastBoardDrawbackInfo
                finalRot = pumpCurrentRot + FINAL_DRAWBACK_STEP_DEG #move positive direction to draw back
                retractZ += 1 #pull z up a little bit
                dbLoc = Location(
                    units,
                    lastPadX,
                    lastPadY,
                    retractZ,
                    finalRot
                )
                dispenseHead.moveTo(dbLoc, MOVE_SPEED_FACTOR)
                log("Letting nozzle tip get to location %d ms..." % DRAW_BACK_SETTLE*2)
                Thread.sleep(DRAW_BACK_SETTLE*2)
                pumpCurrentRot = finalRot
                log("Performed FINAL draw-back of %.3f deg after last board." %
                    FINAL_DRAWBACK_STEP_DEG)
            except Exception, e:
                log("ERROR performing final draw-back: %s" % e)

        # Optional final cleanup / park (this *will* use safeZ, but only once)
        cleanupLoc = None
        if not shouldAbort():
            try:
                cleanupLoc = makeCleanupLocation(dispenseHead.getLocation().getUnits())
            except Exception, e:
                log("Could not build cleanup location: %s" % e)

        if cleanupLoc is not None and not shouldAbort():
            log("Moving to cleanup / park location %s" % cleanupLoc)
            # ensure we leave nozzle unrotated at park
            parkLoc = Location(
                cleanupLoc.getUnits(),
                cleanupLoc.getX(),
                cleanupLoc.getY(),
                cleanupLoc.getZ(),
                0.0
            )
            MovableUtils.moveToLocationAtSafeZ(dispenseHead, parkLoc)
        else:
            log("No CLEANUP_LOCATION configured; skipping park move.")

        # Prompt operator once everything is parked
        if not shouldAbort():
            if not waitForReadyPrompt():
                log("User cancelled at ready prompt; exiting script.")

except Exception, e:
    log_err("%s" % e)


# # Job.Placement.Starting.py
# #
# # Dispensing script using board-level fiducial(s) to correct boardLocation,
# # then walking the footprint pads of the dispense part and moving a
# # "Dispense Head" around to each pad.
# #
# # - Event: Job.Placement.Starting
# # - Uses BOARD fiducial placement + its vision pipeline
# # - If fiducial vision fails after N attempts: disables that BoardLocation
# #   and SKIPS dispensing for that board.
# # - Dispense is done by moving a HeadMountable named "Dispense Head"
# #   (rotation of that head is used to "pump" adhesive).
# #
# # *************************
# # *****   CONFIG      *****
# # *************************

# DISABLE_DISPENSING      = False   # hard kill switch for ALL dispensing
# DRY_RUN                 = True    # True = move to pads, but don't rotate/pump

# TAG = "Job.Placement.Starting"

# # --- dispense hardware config ---
# DISPENSE_HEAD_NAME      = "Dispense Head"  # HeadMountable.getName() or .getId()
# MOVE_SPEED_FACTOR       = 0.74            # matches log style moveTo(..., 0.74)

# # Z = worldLoc.Z (job placement Z) + DISPENSE_Z_OFFSET_MM
# #     retractZ = baseZ + RETRACT_Z_MM
# DISPENSE_Z_OFFSET_MM    = 1.0   # RELATIVE TO job placement Z (worldLoc Z)
# RETRACT_Z_MM            = 0.2   # retract above baseZ between dots

# DISPENSE_STEP_DEG       = 1.0   # degrees of rotation per droplet (tune!)
# DRAWBACK_STEP_DEG       = 20.0  # degrees to twist BACK at end of board

# # --- vision / fiducial config ---
# MAX_VISION_ATTEMPTS     = 3
# PRE_CAPTURE_SETTLE_MS   = 250
# PRE_PIPELINE_SETTLE_MS  = 1000
# RESULT_STAGE_KEY        = "results"
# EXPECTED_ANGLE_DEG      = 0.0
# DESIGN_BOARD_ROT_DEG    = 0.0   # what you WANT the board rotation to be

# # --- pad selection config ---
# PAD_SKIP_NAMES          = set()  # e.g. {"PAD1", "PAD2"}

# # Image-to-machine sign controls for fiducial offset
# FLIP_IMAGE_X = False
# FLIP_IMAGE_Y = True

# # Optional park/cleanup location (leave None if you don't want it)
# CLEANUP_LOCATION = None

# from org.openpnp.model import Location
# from org.openpnp.util import MovableUtils, Utils2D
# from org.opencv.core import RotatedRect
# from java.lang import Math, Thread
# from java.lang import Thread as JThread

# # ------------- LOG HELPERS -------------
# def log(msg):
#     print "[%s] %s" % (TAG, msg)

# def log_err(msg):
#     print "[%s ERROR] %s" % (TAG, msg)


# # ------------- ABORT HELPER -------------
# def shouldAbort():
#     """
#     Best-effort check if the job has been stopped / canceled.
#     NOTE: For Job.Placement.Starting, OpenPnP generally cannot hard-kill
#     a running script; Stop will usually wait until we return.
#     """
#     try:
#         if JThread.currentThread().isInterrupted():
#             log("Abort: thread is interrupted.")
#             return True
#     except:
#         pass

#     try:
#         if 'job' in globals():
#             j = job
#             if hasattr(j, "isCancelled") and j.isCancelled():
#                 log("Abort: job.isCancelled() is True.")
#                 return True
#             if hasattr(j, "getState"):
#                 try:
#                     st = j.getState()
#                     if st is not None and st.toString().lower() in ("canceled", "cancelled", "stopped"):
#                         log("Abort: job state is %s." % st)
#                         return True
#                 except:
#                     pass
#     except:
#         pass

#     return False


# # ------------- HELPER: FIND DISPENSE HEAD -------------
# def findDispenseHead(mac):
#     head = mac.getDefaultHead()
#     try:
#         mnts = head.getHeadMountables()
#     except:
#         mnts = None

#     if mnts is not None:
#         for m in mnts:
#             name = ""
#             mid  = ""
#             try:
#                 name = m.getName()
#             except:
#                 pass
#             try:
#                 mid = m.getId()
#             except:
#                 pass
#             if name == DISPENSE_HEAD_NAME or mid == DISPENSE_HEAD_NAME:
#                 log("Using dispense head mountable '%s' (%s)" % (name, mid))
#                 return m

#     noz = head.getDefaultNozzle()
#     log("WARNING: Could not find head mountable '%s', using default nozzle %s"
#         % (DISPENSE_HEAD_NAME, noz))
#     return noz


# # ------------- NEW HELPER: IS THIS BOARD ALREADY PLACED? -------------
# def boardAlreadyPlaced(jobObj, bl):
#     """
#     Check job-level placement state for this BoardLocation.
#     If *all* JobPlacements for this BoardLocation are placed, we skip
#     fiducials + dispensing for this board.
#     """
#     try:
#         if not hasattr(jobObj, "getPlacements"):
#             log("Job has no getPlacements(); cannot check placed state, assuming NOT placed.")
#             return False

#         jps = jobObj.getPlacements()
#         if jps is None or jps.size() == 0:
#             return False

#         anyForBoard = False
#         for jp in jps:
#             try:
#                 # Match this JobPlacement to our BoardLocation
#                 bl2 = None
#                 if hasattr(jp, "getBoardLocation"):
#                     bl2 = jp.getBoardLocation()
#                 elif hasattr(jp, "getBoard"):
#                     bl2 = jp.getBoard()   # fallback, if some versions use this

#                 if bl2 is None or bl2 != bl:
#                     continue

#                 anyForBoard = True

#                 # If we can read placed-state and it's not placed -> board not done
#                 if hasattr(jp, "isPlaced"):
#                     if not jp.isPlaced():
#                         return False
#                 else:
#                     # No placed flag accessible -> play safe, assume not done
#                     return False
#             except:
#                 # Any weirdness, assume not fully placed
#                 return False

#         if not anyForBoard:
#             # No job placements tied to this boardLocation -> treat as not done
#             return False

#         # Got here: we had at least one placement for this boardLocation
#         # and every one we checked was placed.
#         return True

#     except Exception, e:
#         log("Error while checking boardAlreadyPlaced: %s" % e)
#         return False


# # ------------- HELPER: RUN FIDUCIAL VISION ON ONE BOARD -------------
# def correctBoardLocationWithFiducial(mac, bl):
#     if shouldAbort():
#         return False

#     board = bl.getBoard()
#     if board is None:
#         log_err("BoardLocation has no Board; skipping this boardLocation.")
#         return False

#     log("Board object: %s" % board)

#     fidPlacements = []
#     for placement in board.getPlacements():
#         if shouldAbort():
#             return False
#         if placement.getType().toString() != "Fiducial":
#             continue
#         part = placement.getPart()
#         if part is None:
#             continue
#         vs = part.getFiducialVisionSettings()
#         if vs is None or vs.getPipeline() is None:
#             continue
#         try:
#             if not placement.isEnabled():
#                 continue
#         except:
#             pass
#         fidPlacements.append(placement)

#     if not fidPlacements:
#         log_err("No ENABLED BOARD fiducials with vision pipelines found; skipping this boardLocation.")
#         return False

#     fidPlacement = fidPlacements[0]
#     fidPart      = fidPlacement.getPart()
#     fidSettings  = fidPart.getFiducialVisionSettings()
#     pipeline     = fidSettings.getPipeline()

#     log("Using BOARD fiducial placement %s with part id %s and vision settings %s"
#         % (fidPlacement, fidPart.getId(), fidSettings))

#     fidDesignLoc = fidPlacement.getLocation()
#     log("Board fiducial design location (board-relative): (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (fidDesignLoc.getX(), fidDesignLoc.getY(),
#          fidDesignLoc.getZ(), fidDesignLoc.getRotation()))

#     blLoc = bl.getLocation()
#     fidNominalLoc = Utils2D.calculateBoardPlacementLocation(bl, fidDesignLoc)
#     log("Nominal fiducial machine location: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (fidNominalLoc.getX(), fidNominalLoc.getY(),
#          fidNominalLoc.getZ(), fidNominalLoc.getRotation()))

#     head = mac.getDefaultHead()
#     cam  = head.getDefaultCamera()
#     if cam is None:
#         raise Exception("Default head camera is null.")

#     camLoc = cam.getLocation()
#     camTarget = Location(
#         fidNominalLoc.getUnits(),
#         fidNominalLoc.getX(),
#         fidNominalLoc.getY(),
#         camLoc.getZ(),
#         camLoc.getRotation()
#     )

#     MovableUtils.moveToLocationAtSafeZ(cam, camTarget)
#     log("Camera @ (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (camTarget.getX(), camTarget.getY(), camTarget.getZ(), camTarget.getRotation()))

#     rr = None
#     attempt = 0

#     while attempt < MAX_VISION_ATTEMPTS and rr is None:
#         if shouldAbort():
#             return False

#         attempt += 1
#         log("Vision attempt %d of %d" % (attempt, MAX_VISION_ATTEMPTS))

#         if PRE_CAPTURE_SETTLE_MS > 0:
#             Thread.sleep(PRE_CAPTURE_SETTLE_MS)
#             log("Settled %d ms before pipeline.process()." % PRE_CAPTURE_SETTLE_MS)

#         if PRE_PIPELINE_SETTLE_MS > 0:
#             Thread.sleep(PRE_PIPELINE_SETTLE_MS)

#         pipeline.process()
#         log("pipeline.process() OK on attempt %d" % attempt)

#         candidate_keys = [RESULT_STAGE_KEY, "results", "preResults", "8"]
#         for key in candidate_keys:
#             try:
#                 res = pipeline.getResult(key)
#             except:
#                 res = None
#             if res is None:
#                 continue
#             model = res.getModel()
#             if model is None:
#                 continue

#             if isinstance(model, RotatedRect):
#                 rr = model
#                 log("Got RotatedRect directly from results key '%s' on attempt %d." %
#                     (key, attempt))
#                 break

#             if hasattr(model, "size") and hasattr(model, "get"):
#                 if model.size() > 0:
#                     first = model.get(0)
#                     if isinstance(first, RotatedRect) or hasattr(first, "center"):
#                         rr = first
#                         log("Got RotatedRect[0] from list at results key '%s' on attempt %d." %
#                             (key, attempt))
#                         break

#     if rr is None:
#         log_err("Fiducial vision FAILED after %d attempts for this BoardLocation." %
#                 MAX_VISION_ATTEMPTS)
#         return False

#     log("Measured RotatedRect angle: %.4f deg" % rr.angle)

#     image = cam.capture()
#     w = image.getWidth()
#     h = image.getHeight()
#     log("Captured image for geometry: %dx%d" % (w, h))

#     px = rr.center.x - (w / 2.0)
#     py = rr.center.y - (h / 2.0)
#     log("px,py=%.3f,%.3f" % (px, py))

#     if FLIP_IMAGE_X:
#         px = -px
#     if FLIP_IMAGE_Y:
#         py = -py

#     if hasattr(cam, "getUnitsPerPixelAtZ"):
#         uppLoc = cam.getUnitsPerPixelAtZ()
#     else:
#         uppLoc = cam.getUnitsPerPixel()

#     if uppLoc is None:
#         raise Exception("Camera units-per-pixel not configured (uppLoc is null).")

#     upx = uppLoc.getX()
#     upy = uppLoc.getY()
#     log("Units per pixel: upx=%.9f, upy=%.9f at Z=%.3f" %
#         (upx, upy, uppLoc.getZ()))

#     dx_cam = px * upx
#     dy_cam = py * upy
#     log("Camera-frame dx_cam=%.6f, dy_cam=%.6f" % (dx_cam, dy_cam))

#     rad = camLoc.getRotation() * Math.PI / 180.0
#     c   = Math.cos(rad)
#     s   = Math.sin(rad)

#     dx = dx_cam * c - dy_cam * s
#     dy = dx_cam * s + dy_cam * c
#     log("Machine-frame dx=%.6f, dy=%.6f" % (dx, dy))

#     measuredAngle = rr.angle
#     newBoardRot   = DESIGN_BOARD_ROT_DEG + (EXPECTED_ANGLE_DEG - measuredAngle)
#     dtheta        = newBoardRot - blLoc.getRotation()

#     correctedFid = Location(
#         fidNominalLoc.getUnits(),
#         fidNominalLoc.getX() + dx,
#         fidNominalLoc.getY() + dy,
#         fidNominalLoc.getZ(),
#         newBoardRot
#     )

#     log("Rotation tweak dTheta=%.3f deg (measured=%.3f, designBoard=%.3f, newBoard=%.3f)" %
#         (dtheta, measuredAngle, DESIGN_BOARD_ROT_DEG, newBoardRot))

#     log("CORRECTED fiducial machine target (debug): (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (correctedFid.getX(), correctedFid.getY(),
#          correctedFid.getZ(), correctedFid.getRotation()))

#     newBl = Location(
#         blLoc.getUnits(),
#         blLoc.getX() + dx,
#         blLoc.getY() + dy,
#         blLoc.getZ(),
#         newBoardRot
#     )
#     bl.setLocation(newBl)
#     log("Updated boardLocation: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (newBl.getX(), newBl.getY(), newBl.getZ(), newBl.getRotation()))

#     return True


# # ------------- HELPER: DISPENSE ON ONE BOARD -------------
# def dispenseOnBoard(mac, bl, dispenseHead):
#     if shouldAbort():
#         return

#     board = bl.getBoard()
#     if board is None:
#         log_err("dispenseOnBoard: BoardLocation has no Board.")
#         return

#     dispensePlacement = None
#     for p in board.getPlacements():
#         if shouldAbort():
#             return
#         if p.getType().toString() != "Placement":
#             continue
#         dispensePlacement = p
#         break

#     if dispensePlacement is None:
#         log("No normal Placement found on this board; nothing to dispense.")
#         return

#     part = dispensePlacement.getPart()
#     if part is None:
#         log_err("Dispense placement has no part; skipping.")
#         return

#     pkg = part.getPackage()
#     if pkg is None or pkg.getFootprint() is None:
#         log_err("Part %s has no package/footprint; skipping." % part.getId())
#         return

#     footprint = pkg.getFootprint()
#     pads = footprint.getPads()
#     if pads is None or pads.size() == 0:
#         log("Footprint has no pads; nothing to dispense.")
#         return

#     dLoc = dispensePlacement.getLocation()  # board-relative
#     worldLoc = Utils2D.calculateBoardPlacementLocation(bl, dLoc)
#     log("Dispense using placement %s, part id %s, package id %s, footprint %s with %d pads." %
#         (dispensePlacement, part.getId(), pkg.getId(), footprint, pads.size()))
#     log("Dispense part world location: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (worldLoc.getX(), worldLoc.getY(), worldLoc.getZ(), worldLoc.getRotation()))

#     baseZ    = worldLoc.getZ() + DISPENSE_Z_OFFSET_MM
#     retractZ = baseZ + RETRACT_Z_MM
#     log("Dispense Z plan: worldZ=%.4f, baseZ=%.4f, retractZ=%.4f (offset=%.4f, retract=%.4f)" %
#         (worldLoc.getZ(), baseZ, retractZ, DISPENSE_Z_OFFSET_MM, RETRACT_Z_MM))

#     currentRot = worldLoc.getRotation()

#     for i in range(pads.size()):
#         if shouldAbort():
#             log("Abort requested; stopping pad loop.")
#             return

#         pad = pads.get(i)
#         padName = pad.getName()
#         if padName in PAD_SKIP_NAMES:
#             log("Skipping pad %d named %s (in skip list)." % (i, padName))
#             continue

#         try:
#             try:
#                 padX = pad.getCenterX()
#                 padY = pad.getCenterY()
#             except:
#                 padX = pad.getX()
#                 padY = pad.getY()

#             padRotLocal = pad.getRotation()

#             pr = worldLoc.getRotation() * Math.PI / 180.0
#             c  = Math.cos(pr)
#             s  = Math.sin(pr)

#             dpx = padX * c - padY * s
#             dpy = padX * s + padY * c

#             padWorldX = worldLoc.getX() + dpx
#             padWorldY = worldLoc.getY() + dpy
#             padWorldR = worldLoc.getRotation() + padRotLocal

#             log("Pad %d '%s': local(%.4f, %.4f) -> world(%.4f, %.4f), baseZ=%.4f" %
#                 (i, padName, padX, padY, padWorldX, padWorldY, baseZ))

#             # Approach: XY + retractZ (no safeZ, we already know Z is safe here)
#             approachLoc = Location(
#                 worldLoc.getUnits(),
#                 padWorldX,
#                 padWorldY,
#                 retractZ,
#                 currentRot
#             )
#             dispenseHead.moveTo(approachLoc, MOVE_SPEED_FACTOR)

#             # Drop to dispense Z
#             dispenseLoc = Location(
#                 worldLoc.getUnits(),
#                 padWorldX,
#                 padWorldY,
#                 baseZ,
#                 currentRot
#             )
#             dispenseHead.moveTo(dispenseLoc, MOVE_SPEED_FACTOR)

#             # Pump
#             if not DISABLE_DISPENSING and not DRY_RUN:
#                 currentRot += DISPENSE_STEP_DEG
#                 pumpLoc = Location(
#                     worldLoc.getUnits(),
#                     padWorldX,
#                     padWorldY,
#                     baseZ,
#                     currentRot
#                 )
#                 dispenseHead.moveTo(pumpLoc, MOVE_SPEED_FACTOR)
#             else:
#                 log("Dry run / dispensing disabled: skipping pump at pad %d." % i)

#             # Retract
#             backLoc = Location(
#                 worldLoc.getUnits(),
#                 padWorldX,
#                 padWorldY,
#                 retractZ,
#                 currentRot
#             )
#             dispenseHead.moveTo(backLoc, MOVE_SPEED_FACTOR)

#         except Exception, e:
#             log("ERROR dispensing at pad %d: %s" % (i, e))

#     # Draw-back + safe Z up at last pad position
#     if not DISABLE_DISPENSING and DRAWBACK_STEP_DEG > 0 and not shouldAbort():
#         try:
#             rx = padWorldX
#             ry = padWorldY
#             rz = retractZ
#             currentRot -= DRAWBACK_STEP_DEG
#             dbLoc = Location(
#                 worldLoc.getUnits(),
#                 rx, ry, rz,
#                 currentRot
#             )
#             dispenseHead.moveTo(dbLoc, MOVE_SPEED_FACTOR)
#             log("Performed draw-back of %.3f deg at last pad." % DRAWBACK_STEP_DEG)
#         except Exception, e:
#             log("ERROR performing draw-back: %s" % e)

#     # Finally, go to machine safe Z at the last pad XY
#     try:
#         safeLoc = Location(
#             worldLoc.getUnits(),
#             padWorldX,
#             padWorldY,
#             retractZ,
#             currentRot
#         )
#         MovableUtils.moveToLocationAtSafeZ(dispenseHead, safeLoc)
#         log("Moved to safe Z at last pad location.")
#     except:
#         pass


# # ------------- MAIN BODY -------------
# try:
#     if 'job' not in globals() or 'machine' not in globals():
#         raise Exception("Missing globals job or machine.")

#     mac = machine
#     dispenseHead = findDispenseHead(mac)

#     if DISABLE_DISPENSING:
#         log("DISABLE_DISPENSING is True; script will still run fiducial correction but will NOT pump adhesive.")

#     bls = []
#     for bl in job.getBoardLocations():
#         if shouldAbort():
#             log("Abort requested before processing boardLocations.")
#             break
#         try:
#             if not bl.isEnabled():
#                 continue
#         except:
#             pass
#         bls.append(bl)

#     log("Found %d boardLocations in job." % len(bls))

#     for bl in bls:
#         if shouldAbort():
#             log("Abort requested; breaking out of board loop.")
#             break

#         # NEW: skip boards whose job placements are all already marked placed
#         try:
#             if boardAlreadyPlaced(job, bl):
#                 log("BoardLocation %s already has all job placements marked placed; skipping fiducials and dispensing."
#                     % bl)
#                 continue
#         except Exception, e:
#             log("Error checking placed state for BoardLocation %s: %s" % (bl, e))

#         blLoc = bl.getLocation()
#         log("Processing BoardLocation at (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#             (blLoc.getX(), blLoc.getY(), blLoc.getZ(), blLoc.getRotation()))

#         ok = correctBoardLocationWithFiducial(mac, bl)
#         if not ok:
#             try:
#                 bl.setEnabled(False)
#             except:
#                 pass
#             log_err("Disabled BoardLocation due to fiducial failure; skipping dispensing for this board.")
#             continue

#         dispenseOnBoard(mac, bl, dispenseHead)

#     if CLEANUP_LOCATION is not None and not shouldAbort():
#         log("Moving to cleanup / park location %s" % CLEANUP_LOCATION)
#         MovableUtils.moveToLocationAtSafeZ(dispenseHead, CLEANUP_LOCATION)

# except Exception, e:
#     log_err("%s" % e)



