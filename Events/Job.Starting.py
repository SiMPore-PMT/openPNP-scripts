# Job.Starting.py
#
# Dispense-first flow with pocket-fiducial board-pose correction.

import sys

from jarray import array
from java.lang import Boolean, Class, Double, Exception as JavaException
from java.lang import Math, Object, System, Thread
from java.util.concurrent.atomic import AtomicBoolean
from org.openpnp.model import Configuration, LengthUnit, Location, Placement
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import Cycles, MovableUtils, Utils2D
from org.opencv.core import RotatedRect
from org.openpnp.spi.Locatable import LocationOption

SCRIPTS_ROOT = str(scripting.getScriptsDirectory())
JOB_LIB_DIR = SCRIPTS_ROOT + "/Events/_lib"
if JOB_LIB_DIR not in sys.path:
    sys.path.insert(0, JOB_LIB_DIR)
from job_runtime import GuiJobStateApi, JobRuntimeMonitor, JobStopRequested
from pre_dispense_flush import run_pre_dispense_flush
from vision_consistency import resolve_world_measurement, world_xy_distance_mm
from board_pose import (ANCHOR_RESIDUAL_TOLERANCE_MM,
                        apply_measured_board_pose,
                        make_fiducial_locked_transform,
                        restore_board_state,
                        set_location_preserving_transform,
                        snapshot_board_state,
                        xy_shift_mm)


# FLOW CONFIG
FLUSH_ENABLED = True  # True = flush on the dap-off pad before every job start.
FLUSH_PULSES = 4       # Number of DISPENSE_PULSE_MS pulses in the flush.

SKIP_DISPENSE = False
DRY_SECOND_PASS = False #Set to True to run a second pass w/out dispensing, smoths out pdms (sometimes pushese it over the edge)
POST_DISPENSE_IMAGE_CAPTURE_ENABLED = False
POST_DISPENSE_IMAGING_DELAY_MS = 250 #1000


# VISION CONFIG
MAX_NUDGE_MM           = 4.0
APPLY_ROTATION         = True
EXPECTED_ANGLE_DEG     = 0.0
DESIGN_BOARD_ROT_DEG   = 0.0

PRE_CAPTURE_SETTLE_MS  = 250
RESULT_STAGE_KEY       = "results"
RETRY_DELAY_MS         = 250
VISION_MEASUREMENT_ATTEMPTS = 2
RECENTER_MAX_PASSES    = 3
RECENTER_TOLERANCE_MM  = 0.02
MAX_VISION_WORLD_DISAGREEMENT_MM = 0.10
FORCE_VISION_RECENTER  = False

FLIP_IMAGE_X = False
FLIP_IMAGE_Y = True


# DISPENSE CONFIG
DISPENSE_HEAD_NAME      = "Dispense Head"
DISPENSE_ACTUATOR_NAME  = "GlueDispense"

DISPENSE_Z_OFFSET_MM    = 0.1
INTER_PAD_LIFT_MM       = 0.05
DISPENSE_SAFE_Z_MM      = 25.0

# VORON Z PROBE CONFIG
ENABLE_VORON_Z_PROBE            = True
VORON_OFFSET_MM                 = 0.8
VORON_ACTUATOR_NAME             = "Voron Z Probe"
VORON_CORNER_INSET_MM           = 0.1
VORON_START_ABOVE_MM            = 1.0
VORON_MAX_DESCENT_MM            = 5.0
VORON_PROBE_SPEED_FACTOR        = 1

# Marlin feed rates are mm/min; zero fine taps uses coarse contact only.
VORON_COARSE_FEED_MM_PER_MIN    = 3000.0
VORON_FINE_FEED_MM_PER_MIN      = 500.0
VORON_RETRACT_FEED_MM_PER_MIN   = 5000.0
VORON_RESTORE_FEED_MM_PER_MIN   = 7500.0
VORON_PROBE_ACCEL_MM_PER_SEC2   = 1000.0
VORON_FINE_TAP_COUNT            = 1
VORON_FINE_RETRACT_MM           = 1.0
VORON_FINE_MAX_DESCENT_MM       = 2.0

TRAVEL_SPEED_FACTOR     = 1
DISPENSE_SPEED_FACTOR   = 1

PRE_DISPENSE_DWELL_MS   = 0
DISPENSE_PULSE_MS       = 10
POST_DISPENSE_DWELL_MS  = 0
ACTUATOR_SETTLE_MS      = 0

PAD_SKIP_NAMES          = set()

EXTRA_FIRST_PAD_DOTS = 0
EXTRA_FIRST_PAD_DOT_SPACING_MM = -0.40

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

TAG = "Job.Starting"
JOB_STATE_POLL_MS = 50


def log(msg):
    print "[%s] %s" % (TAG, str(msg))


def log_err(msg):
    print "[%s ERROR] %s" % (TAG, str(msg))


class VisionMeasurementError(Exception):
    pass


class VoronProbeError(Exception):
    pass


RUNTIME_MONITOR = None
DISPENSE_DIRTY = AtomicBoolean(False)


def runtime_checkpoint(label, allow_pause=True):
    if RUNTIME_MONITOR is not None:
        RUNTIME_MONITOR.checkpoint(label, allow_pause)


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
    runtime_checkpoint("%s completed" % label, False)


def wait_coord(movable, label):
    t0 = log_t("%s: wait_coord BEFORE WaitForUnconditionalCoordination" % label)
    movable.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
    log_t("%s: wait_coord AFTER WaitForUnconditionalCoordination elapsed=%.3f ms" %
          (label, elapsed_ms_since(t0)))
    runtime_checkpoint("%s completed" % label, False)


def move_safe_wait(movable, target, label):
    runtime_checkpoint("before %s" % label)
    MovableUtils.moveToLocationAtSafeZ(movable, target)
    wait_still(movable, label)
    runtime_checkpoint("after %s" % label)


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
    seen_keys = set()
    for key in [RESULT_STAGE_KEY, "results", "preResults"]:
        if key in seen_keys:
            continue
        seen_keys.add(key)
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
        rects = extract_rotated_rects_from_model(model)
        if rects:
            return rects
    return []


def choose_closest_rect_to_image_center(rects, width, height,
                                        cam_loc=None, upp_loc=None,
                                        reference_loc=None):
    if reference_loc is not None:
        consistent = []
        for candidate in rects:
            dx, dy, px, py = rect_center_machine_offset(
                cam_loc, upp_loc, candidate, width, height)
            candidate_loc = Location(
                cam_loc.getUnits(), cam_loc.getX() + dx,
                cam_loc.getY() + dy, cam_loc.getZ(), cam_loc.getRotation())
            if (world_xy_distance_mm(candidate_loc, reference_loc) <=
                    MAX_VISION_WORLD_DISAGREEMENT_MM):
                consistent.append(candidate)
        if consistent:
            rects = consistent

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


def is_dispense_placement(plc):
    return plc is not None and plc.getType() == Placement.Type.Dispense


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
        try:
            if str(plc.getId()).lower().endswith("_prealign"):
                continue
        except:
            pass
        settings = get_vision_settings_for_fiducial(plc)
        if settings is None:
            log("Fiducial placement index %d has no vision settings; skipping." % i)
            continue
        loc = plc.getLocation()
        label = object_name(plc) or "<unnamed>"
        log("Selected fiducial placement '%s' at index %d." % (label, i))
        return plc, loc, settings, label

    raise Exception("No enabled board fiducial placement with vision settings found.")


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
        runtime_checkpoint("%s measurement attempt %d" %
                           (label, measurement_attempt))
        if measurement_attempt > 1:
            cooperative_sleep(
                RETRY_DELAY_MS,
                "%s measurement retry delay" % label)
        log("[%s] Measurement attempt %d of %d." %
            (label, measurement_attempt, VISION_MEASUREMENT_ATTEMPTS))

        move_safe_wait(
            cam, cam_target,
            "%s camera nominal attempt %d" % (label, measurement_attempt))
        initial_view_loc = cam_target
        reliable_world_loc = None
        pending_suspect_loc = None

        for recenter_pass in range(1, RECENTER_MAX_PASSES + 1):
            wait_still(
                cam,
                "%s recenter pass %d camera" % (label, recenter_pass))
            if PRE_CAPTURE_SETTLE_MS > 0:
                cooperative_sleep(
                    PRE_CAPTURE_SETTLE_MS,
                    "%s recenter pass %d capture settle" %
                    (label, recenter_pass))

            try:
                runtime_checkpoint(
                    "%s recenter pass %d before vision capture" %
                    (label, recenter_pass))
                pipeline.process()
                runtime_checkpoint(
                    "%s recenter pass %d after vision capture" %
                    (label, recenter_pass))
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
            cam_loc = cam.getLocation()
            upp_loc = get_camera_units_per_pixel(cam)
            rr, dist2 = choose_closest_rect_to_image_center(
                rects, width, height, cam_loc, upp_loc, reliable_world_loc)
            if rr is None:
                last_reject = "no usable RotatedRect results"
                log("[%s] Rejected vision pass: %s" % (label, last_reject))
                break

            center_dx, center_dy, px, py = rect_center_machine_offset(
                cam_loc, upp_loc, rr, width, height)

            detected_loc = Location(
                cam_loc.getUnits(),
                cam_loc.getX() + center_dx,
                cam_loc.getY() + center_dy,
                cam_loc.getZ(),
                cam_loc.getRotation()
            )
            p_loc_mm = p_loc.convertToUnits(LengthUnit.Millimeters)
            detected_mm = detected_loc.convertToUnits(LengthUnit.Millimeters)
            board_dx_mm = detected_mm.getX() - p_loc_mm.getX()
            board_dy_mm = detected_mm.getY() - p_loc_mm.getY()

            inside, view_dx, view_dy, half_x, half_y = \
                detection_inside_initial_view(
                    initial_view_loc, detected_loc, upp_loc, width, height)
            if not inside:
                last_reject = (
                    "outside initial view dx=%.6f/%.6f dy=%.6f/%.6f" %
                    (view_dx, half_x, view_dy, half_y))
                log("[%s] Rejected vision pass: %s" % (label, last_reject))
                break

            if (abs(board_dx_mm) > MAX_NUDGE_MM or
                    abs(board_dy_mm) > MAX_NUDGE_MM):
                last_reject = (
                    "beyond MAX_NUDGE_MM board dx=%.6f dy=%.6f mm limit=%.6f" %
                    (board_dx_mm, board_dy_mm, MAX_NUDGE_MM))
                log("[%s] Rejected vision pass: %s" % (label, last_reject))
                break

            reliable_world_loc, pending_suspect_loc, world_disagreement, world_state = \
                resolve_world_measurement(
                    reliable_world_loc, pending_suspect_loc, detected_loc,
                    MAX_VISION_WORLD_DISAGREEMENT_MM)
            cam_mm = cam_loc.convertToUnits(LengthUnit.Millimeters)
            log("[%s] pass=%d rects=%d cam=(%.4f,%.4f) detected=(%.4f,%.4f) image=(%.6f,%.6f) world=%.6f state=%s angle=%.4f" %
                (label, recenter_pass, len(rects), cam_mm.getX(), cam_mm.getY(),
                 detected_mm.getX(), detected_mm.getY(), center_dx, center_dy,
                 world_disagreement, world_state, rr.angle))
            if world_state == "suspect":
                last_reject = (
                    "world disagreement %.6f mm exceeds %.6f mm" %
                    (world_disagreement, MAX_VISION_WORLD_DISAGREEMENT_MM))
                log("[%s] Rejected vision pass: %s" % (label, last_reject))
                if recenter_pass < RECENTER_MAX_PASSES:
                    log("[%s] Holding camera stationary for confirmation capture." %
                        label)
                    continue
                break

            if (not FORCE_VISION_RECENTER or recenter_pass > 1) and abs(center_dx) <= RECENTER_TOLERANCE_MM and abs(center_dy) <= RECENTER_TOLERANCE_MM:
                old_global_tx = bl.getLocalToGlobalTransform()
                old_transform_angle = Utils2D.affineInfo(
                    old_global_tx).rotationAngleDeg
                new_rot = old_transform_angle
                if APPLY_ROTATION:
                    new_rot = DESIGN_BOARD_ROT_DEG + (EXPECTED_ANGLE_DEG - rr.angle)
                    log("[%s] Target global transform angle=%.4f from measured=%.4f" %
                        (label, new_rot, rr.angle))

                new_tx, old_transform_angle = make_fiducial_locked_transform(
                    bl, fid_loc, detected_loc, new_rot)
                log("[%s] Accepted fiducial-locked affine pose; old global angle=%.4f new global angle=%.4f" %
                    (label, old_transform_angle, new_rot))

                return {
                    "boardTransform": new_tx,
                    "fiducialLocation": fid_loc,
                    "measuredFiducial": detected_loc,
                    "dx": board_dx_mm,
                    "dy": board_dy_mm,
                    "rotation": new_rot,
                    "angle": rr.angle,
                    "recenterPass": recenter_pass,
                    "measurementAttempt": measurement_attempt,
                }

            last_reject = (
                "not centered dx=%.6f dy=%.6f tolerance=%.6f" %
                (center_dx, center_dy, RECENTER_TOLERANCE_MM))
            if recenter_pass < RECENTER_MAX_PASSES:
                if center_dx or center_dy:
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


def find_voron_probe_actuator(mac):
    wanted = str(VORON_ACTUATOR_NAME)
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
                    log("Using Voron probe actuator '%s'." % object_name(actuator))
                    return actuator
            except:
                pass
        try:
            for actuator in java_list_to_py(owner.getActuators()):
                if object_name(actuator) == wanted:
                    log("Using Voron probe actuator '%s'." % object_name(actuator))
                    return actuator
        except:
            pass

    raise VoronProbeError("Required Voron probe actuator not found: %s" % wanted)


def positive_voron_value(value, label):
    try:
        result = float(value)
    except:
        raise VoronProbeError("%s must be a number." % label)
    if result <= 0.0:
        raise VoronProbeError("%s must be greater than zero." % label)
    return result


def voron_fine_tap_count():
    try:
        number = float(VORON_FINE_TAP_COUNT)
        result = int(number)
    except:
        raise VoronProbeError("VORON_FINE_TAP_COUNT must be a whole number from 0 to N.")
    if number != float(result) or result < 0:
        raise VoronProbeError("VORON_FINE_TAP_COUNT must be a whole number from 0 to N.")
    return result


def voron_probe_driver(actuator):
    try:
        driver = actuator.getDriver()
    except:
        driver = None
    if (driver is None or not hasattr(driver, "sendCommand") or
            not hasattr(driver, "getReportedLocation")):
        raise VoronProbeError(
            "Voron Z Probe must use a GcodeDriver with sendCommand and getReportedLocation.")
    return driver


def reported_dispense_head_z_mm(dispense_head, driver):
    axes_location = driver.getReportedLocation(-1)
    location = dispense_head.toTransformed(axes_location)
    location = dispense_head.toHeadMountableLocation(location)
    return float(location.convertToUnits(LengthUnit.Millimeters).getZ())


def send_voron_gcode(driver, command, label):
    log("%s G-code: %s" % (label, command))
    driver.sendCommand(command)


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

        # Vertical-to-horizontal: finish the vertical leg first.
        if ((current_edges["left"] or current_edges["right"]) and
                (next_edges["top"] or next_edges["bottom"])):
            return current_item["x"], next_item["y"]

        # Horizontal-to-vertical: finish the horizontal leg first.
        if ((current_edges["top"] or current_edges["bottom"]) and
                (next_edges["left"] or next_edges["right"])):
            return next_item["x"], current_item["y"]

    # Preserve the real-pad tangent; ignore inserted corners.
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

    # Use footprint-local coordinates so board rotation does not change the edge.
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

            # Continue vertical first.
            if ((current_edges["left"] or current_edges["right"]) and
                    (first_edges["top"] or first_edges["bottom"])):
                corner_x = current_item["x"]
                corner_y = first_item["y"]

            # Continue horizontal first, including the closing edge.
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


def cooperative_sleep(ms, label, allow_pause=True):
    remaining = int(ms)
    while remaining > 0:
        runtime_checkpoint(label, allow_pause)
        interval = min(remaining, JOB_STATE_POLL_MS)
        Thread.sleep(interval)
        remaining -= interval
    runtime_checkpoint(label, allow_pause)


def sleep_ms(ms, label="timed wait", allow_pause=True):
    cooperative_sleep(ms, label, allow_pause)


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
    sleep_ms(settle_ms, "%s fallback actuator settle" % label, False)
    log_t("%s: fallback sleep %d ms complete" % (label, int(settle_ms)))


def pulse_glue_controller(actuator, machine_obj, pad_index):
    t0 = log_t("pad %d: pulse_glue_controller START boolean pulse=%d ms settle=%d ms post=%d ms" %
               (pad_index, DISPENSE_PULSE_MS, ACTUATOR_SETTLE_MS, POST_DISPENSE_DWELL_MS))

    set_boolean_actuator_state_fast(actuator, False, "pad %d GlueDispense OFF before pulse" % pad_index)
    wait_after_actuator_change(
        actuator, machine_obj, ACTUATOR_SETTLE_MS,
        "pad %d GlueDispense OFF wait" % pad_index)
    sleep_ms(ACTUATOR_SETTLE_MS,
             "pad %d glue-off settle" % pad_index, False)

    runtime_checkpoint("pad %d before glue pulse" % pad_index, False)
    DISPENSE_DIRTY.set(True)
    set_boolean_actuator_state_fast(actuator, True, "pad %d GlueDispense ON" % pad_index)
    wait_after_actuator_change(
        actuator, machine_obj, ACTUATOR_SETTLE_MS,
        "pad %d GlueDispense ON wait" % pad_index)
    sleep_ms(DISPENSE_PULSE_MS,
             "pad %d glue pulse" % pad_index, False)

    set_boolean_actuator_state_fast(actuator, False, "pad %d GlueDispense OFF after pulse" % pad_index)
    wait_after_actuator_change(
        actuator, machine_obj, ACTUATOR_SETTLE_MS,
        "pad %d GlueDispense final OFF wait" % pad_index)
    sleep_ms(ACTUATOR_SETTLE_MS,
             "pad %d glue final-off settle" % pad_index, False)
    sleep_ms(POST_DISPENSE_DWELL_MS,
             "pad %d post-dispense dwell" % pad_index, False)

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


def safe_retract_dispense_head(mac, label):
    try:
        dispense_head = find_dispense_head(mac)
        move_z_only_no_dynamic_comp(
            dispense_head,
            LengthUnit.Millimeters,
            DISPENSE_SAFE_Z_MM,
            TRAVEL_SPEED_FACTOR,
            "%s safe retract" % label)
        wait_still(dispense_head, "%s safe retract" % label)
        log("%s moved dispense head to safe Z=%.4f mm." %
            (label, DISPENSE_SAFE_Z_MM))
        return True
    except Exception, e:
        log_err("%s could not retract dispense head to safe Z: %s" %
                (label, e))
        return False


def make_voron_probe_point(bl, fid_loc, footprint_units, pad_positions):
    if len(pad_positions) == 0:
        raise VoronProbeError("Cannot choose Voron probe corner without pads.")

    local_x_values = []
    local_y_values = []
    for item in pad_positions:
        if item.get("local_x") is None or item.get("local_y") is None:
            continue
        local_x_values.append(float(item["local_x"]))
        local_y_values.append(float(item["local_y"]))

    if len(local_x_values) == 0 or len(local_y_values) == 0:
        raise VoronProbeError(
            "Cannot choose Voron probe corner because pad coordinates are unavailable.")

    min_x = min(local_x_values)
    max_x = max(local_x_values)
    min_y = min(local_y_values)
    max_y = max(local_y_values)
    inset = abs(mm_to_units(VORON_CORNER_INSET_MM, footprint_units))
    inset_x = min(inset, (max_x - min_x) / 2.0)
    inset_y = min(inset, (max_y - min_y) / 2.0)
    local_x = min_x + inset_x
    local_y = min_y + inset_y

    fid_units = fid_loc.getUnits()
    local_offset = Location(footprint_units, local_x, local_y, 0.0, 0.0)
    board_offset = local_offset.convertToUnits(fid_units)
    fid_rad = fid_loc.getRotation() * Math.PI / 180.0
    fid_c = Math.cos(fid_rad)
    fid_s = Math.sin(fid_rad)
    offset_x = board_offset.getX()
    offset_y = board_offset.getY()
    board_x = fid_loc.getX() + (offset_x * fid_c - offset_y * fid_s)
    board_y = fid_loc.getY() + (offset_x * fid_s + offset_y * fid_c)
    board_point = Location(
        fid_units,
        board_x,
        board_y,
        fid_loc.getZ(),
        0.0)
    world_point = Utils2D.calculateBoardPlacementLocation(bl, board_point)

    log("Voron probe corner footprint bounds X=[%.4f, %.4f] Y=[%.4f, %.4f], inset=%.4f mm -> local X=%.4f Y=%.4f." %
        (min_x, max_x, min_y, max_y, VORON_CORNER_INSET_MM,
         local_x, local_y))
    return {
        "board_point": board_point,
        "world_point": world_point,
        "local_x": local_x,
        "local_y": local_y
    }


def run_voron_g38_sequence(dispense_head, actuator):
    driver = voron_probe_driver(actuator)
    fine_count = voron_fine_tap_count()
    coarse_feed = positive_voron_value(
        VORON_COARSE_FEED_MM_PER_MIN, "VORON_COARSE_FEED_MM_PER_MIN")
    probe_accel = positive_voron_value(
        VORON_PROBE_ACCEL_MM_PER_SEC2, "VORON_PROBE_ACCEL_MM_PER_SEC2")
    restore_feed = positive_voron_value(
        VORON_RESTORE_FEED_MM_PER_MIN, "VORON_RESTORE_FEED_MM_PER_MIN")
    coarse_descent = positive_voron_value(
        VORON_MAX_DESCENT_MM, "VORON_MAX_DESCENT_MM")
    if fine_count > 0:
        fine_feed = positive_voron_value(
            VORON_FINE_FEED_MM_PER_MIN, "VORON_FINE_FEED_MM_PER_MIN")
        retract_feed = positive_voron_value(
            VORON_RETRACT_FEED_MM_PER_MIN, "VORON_RETRACT_FEED_MM_PER_MIN")
        fine_retract = positive_voron_value(
            VORON_FINE_RETRACT_MM, "VORON_FINE_RETRACT_MM")
        fine_descent = positive_voron_value(
            VORON_FINE_MAX_DESCENT_MM, "VORON_FINE_MAX_DESCENT_MM")
        if fine_descent <= fine_retract:
            raise VoronProbeError(
                "VORON_FINE_MAX_DESCENT_MM must be greater than VORON_FINE_RETRACT_MM so each fine G38.2 can retouch the pad.")

    coarse_z_mm = None
    fine_contacts = []
    sequence_error = None
    restore_errors = []
    position_saved = False

    try:
        # G60 saves the pre-probe position; restore G90 before G61.
        send_voron_gcode(driver, "M400", "Voron setup")
        # Override OpenPnP's modal M204 before G38.2.
        send_voron_gcode(
            driver, "M204 S%.3f" % probe_accel, "Voron setup")
        send_voron_gcode(driver, "G60 S0", "Voron setup")
        position_saved = True
        send_voron_gcode(driver, "G91", "Voron setup")

        send_voron_gcode(
            driver,
            "G38.2 Z%.4f F%.3f" % (coarse_descent, coarse_feed),
            "Voron coarse")
        send_voron_gcode(driver, "M400", "Voron coarse")
        coarse_z_mm = reported_dispense_head_z_mm(dispense_head, driver)
        log("Voron coarse G38.2 contact Z=%.6f mm." % coarse_z_mm)

        for fine_index in range(fine_count):
            label = "Voron fine %d/%d" % (fine_index + 1, fine_count)
            send_voron_gcode(
                driver,
                "G0 Z-%.4f F%.3f" % (fine_retract, retract_feed),
                "%s retract" % label)
            send_voron_gcode(driver, "M400", "%s retract" % label)
            send_voron_gcode(
                driver,
                "G38.2 Z%.4f F%.3f" % (fine_descent, fine_feed),
                label)
            send_voron_gcode(driver, "M400", label)
            fine_z_mm = reported_dispense_head_z_mm(dispense_head, driver)
            fine_contacts.append(fine_z_mm)
            log("%s G38.2 contact Z=%.6f mm." % (label, fine_z_mm))
    except Exception, e:
        sequence_error = e
    finally:
        # Restore absolute mode and the G60 position.
        restore_commands = ["G90"]
        if position_saved:
            restore_commands.append("G61 S0 Z F%.3f" % restore_feed)
        restore_commands.append("M400")
        for command in restore_commands:
            try:
                send_voron_gcode(driver, command, "Voron restore")
            except Exception, e:
                restore_errors.append("%s: %s" % (command, e))
        try:
            driver.getReportedLocation(-1)
        except Exception, e:
            restore_errors.append("M114 position sync: %s" % e)

    if len(restore_errors) > 0:
        detail = "; ".join(restore_errors)
        if sequence_error is not None:
            raise VoronProbeError(
                "Voron G38 probe failed (%s), and position restore also failed: %s" %
                (sequence_error, detail))
        raise VoronProbeError("Voron position restore failed: %s" % detail)
    if sequence_error is not None:
        raise VoronProbeError("Voron G38 probe failed: %s" % sequence_error)
    if coarse_z_mm is None:
        raise VoronProbeError("Voron coarse G38.2 did not return a contact position.")

    averaged_contacts = fine_contacts if len(fine_contacts) > 0 else [coarse_z_mm]
    average_z_mm = sum(averaged_contacts) / float(len(averaged_contacts))
    return coarse_z_mm, fine_contacts, average_z_mm


def probe_voron_surface(mac, dispense_head, probe_point):
    actuator = find_voron_probe_actuator(mac)
    world_mm = probe_point["world_point"].convertToUnits(
        LengthUnit.Millimeters)
    x_mm = float(world_mm.getX())
    y_mm = float(world_mm.getY())
    expected_surface_z_mm = float(world_mm.getZ())
    start_z_mm = expected_surface_z_mm + float(VORON_START_ABOVE_MM)

    log("Voron probing world X=%.6f Y=%.6f expectedZ=%.6f startZ=%.6f max descent=%.6f mm." %
        (x_mm, y_mm, expected_surface_z_mm, start_z_mm,
         VORON_MAX_DESCENT_MM))

    try:
        move_z_only_no_dynamic_comp(
            dispense_head,
            LengthUnit.Millimeters,
            DISPENSE_SAFE_Z_MM,
            TRAVEL_SPEED_FACTOR,
            "Voron ensure safe Z before XY")
        wait_still(dispense_head, "Voron ensure safe Z before XY")

        move_direct_no_dynamic_comp(
            dispense_head,
            dispense_head_location(
                LengthUnit.Millimeters, x_mm, y_mm, DISPENSE_SAFE_Z_MM),
            TRAVEL_SPEED_FACTOR,
            "Voron travel to probe point at safe Z")
        wait_still(dispense_head, "Voron travel to probe point at safe Z")

        move_z_only_no_dynamic_comp(
            dispense_head,
            LengthUnit.Millimeters,
            start_z_mm,
            VORON_PROBE_SPEED_FACTOR,
            "Voron lower to search start")
        wait_still(dispense_head, "Voron lower to search start")
        coarse_z_mm, fine_contacts, average_trigger_z_mm = \
            run_voron_g38_sequence(dispense_head, actuator)
        surface_z_mm = average_trigger_z_mm + float(VORON_OFFSET_MM)
        contacts_used = fine_contacts if len(fine_contacts) > 0 else [coarse_z_mm]
        log("Voron contacts used=%s average trigger Z=%.6f mm + offset=%.6f mm -> surface Z=%.6f mm." %
            (contacts_used, average_trigger_z_mm,
             VORON_OFFSET_MM, surface_z_mm))
        return {
            "probe_x_mm": x_mm,
            "probe_y_mm": y_mm,
            "expected_surface_z_mm": expected_surface_z_mm,
            "coarse_trigger_z_mm": coarse_z_mm,
            "fine_trigger_z_values_mm": fine_contacts,
            "average_trigger_z_mm": average_trigger_z_mm,
            "surface_z_mm": surface_z_mm
        }
    except Exception:
        safe_retract_dispense_head(mac, "Voron probe failure")
        raise


def update_board_z_from_voron(bl, probe_point, probe_result):
    old_location = bl.getLocation()
    old_world_z_mm = probe_point["world_point"].convertToUnits(
        LengthUnit.Millimeters).getZ()
    surface_z_mm = float(probe_result["surface_z_mm"])
    correction_mm = surface_z_mm - float(old_world_z_mm)
    correction_in_board_units = mm_to_units(
        correction_mm, old_location.getUnits())
    new_location = make_board_location(
        old_location,
        old_location.getX(),
        old_location.getY(),
        old_location.getZ() + correction_in_board_units,
        old_location.getRotation())
    set_location_preserving_transform(bl, new_location)

    log("Voron board Z update old board Z=%.6f, predicted probe world Z=%.6f mm, measured surface Z=%.6f mm, correction=%.6f mm, new board Z=%.6f; X/Y/R unchanged." %
        (old_location.getZ(), old_world_z_mm, surface_z_mm,
         correction_mm, new_location.getZ()))
    return new_location


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

        sleep_ms(PRE_DISPENSE_DWELL_MS,
                 "%s pad %d pre-dispense dwell" %
                 (motion_label, pad_index), False)

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
        wait_coord(
            dispense_head,
            "%s pad %d dot %d/%d retract" %
            (motion_label, pad_index, dot_number, total_dots))
        runtime_checkpoint(
            "%s pad %d dot %d/%d safely retracted" %
            (motion_label, pad_index, dot_number, total_dots))
        log_t("%s pad %d dot %d/%d: AFTER completed retract" %
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


def dispense_current_placement(mac, board, bl, current_placement,
                               voron_probe_cache):
    dispense_head = find_dispense_head(mac)
    glue = find_glue_dispense_actuator(mac)
    dispense_placement = current_placement
    if not is_dispense_placement(dispense_placement):
        raise Exception(
            "Expected Placement.Type.Dispense, got %s." %
            str(dispense_placement.getType()))

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
    log("Dispense pad transform: footprint pad -> board fiducial/pocket location -> ACTIVE boardLocation world location.")
    log("Dispense needle B/rotation axis blocked: all dispense-head move targets use R=NaN.")
    log("Dispense moves use SuppressDynamicCompensation to bypass nozzle runout / rotation-mode compensation.")

    set_boolean_actuator_state_fast(glue, False, "glue initial off")

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

    if len(pad_positions) == 0:
        raise Exception("Dispense footprint has no non-skipped pads.")

    newly_probed = False

    if not ENABLE_VORON_Z_PROBE:
        log("ENABLE_VORON_Z_PROBE is False; skipping Voron tap and using saved BoardLocation Z=%.6f." %
            bl.getLocation().getZ())
    else:
        probe_point = make_voron_probe_point(
            bl, fid_loc, footprint_units, pad_positions)
        probe_key = board_location_key(bl)

        if probe_key in voron_probe_cache:
            cached = voron_probe_cache[probe_key]
            current_bl_loc = bl.getLocation()
            current_bl_mm = current_bl_loc.convertToUnits(LengthUnit.Millimeters)
            cached_location_mm = Location(
                LengthUnit.Millimeters,
                current_bl_mm.getX(),
                current_bl_mm.getY(),
                float(cached["board_z_mm"]),
                current_bl_mm.getRotation())
            cached_location = cached_location_mm.convertToUnits(
                current_bl_loc.getUnits())
            set_location_preserving_transform(bl, cached_location)
            log("Voron reused cached board Z=%.6f mm for BoardLocation key=%s; corrected X/Y/R retained." %
                (cached["board_z_mm"], probe_key))
        else:
            probe_result = probe_voron_surface(mac, dispense_head, probe_point)
            new_bl_loc = update_board_z_from_voron(
                bl, probe_point, probe_result)
            new_bl_mm = new_bl_loc.convertToUnits(LengthUnit.Millimeters)
            voron_probe_cache[probe_key] = {
                "board_z_mm": float(new_bl_mm.getZ()),
                "surface_z_mm": float(probe_result["surface_z_mm"]),
                "average_trigger_z_mm": float(
                    probe_result["average_trigger_z_mm"])
            }
            newly_probed = True
            log("Voron cached successful measurement for BoardLocation key=%s." %
                probe_key)

    # Recalculate world coordinates with the measured/cached or saved Z. The
    # Z update must not disturb the already-corrected XY affine pose.
    fid_world_after_z = Utils2D.calculateBoardPlacementLocation(bl, fid_loc)
    z_update_dx_mm, z_update_dy_mm = xy_shift_mm(
        fid_world, fid_world_after_z)
    z_update_xy_error = Math.sqrt(z_update_dx_mm * z_update_dx_mm +
                                  z_update_dy_mm * z_update_dy_mm)
    if z_update_xy_error > ANCHOR_RESIDUAL_TOLERANCE_MM:
        raise Exception(
            "Voron Z update changed board fiducial XY by %.6f mm." %
            z_update_xy_error)
    log("Voron/cached-Z XY preservation residual dx=%.6f dy=%.6f mm." %
        (z_update_dx_mm, z_update_dy_mm))
    fid_world = fid_world_after_z
    units = fid_world.getUnits()
    base_z = fid_world.getZ() + mm_to_units(DISPENSE_Z_OFFSET_MM, units)
    lift_z = base_z + mm_to_units(INTER_PAD_LIFT_MM, units)

    for item in pad_positions:
        pad_offset_loc = Location(
            footprint_units,
            item["local_x"],
            item["local_y"],
            0.0,
            0.0)
        pad_offset_board_units = pad_offset_loc.convertToUnits(fid_units)
        px = pad_offset_board_units.getX()
        py = pad_offset_board_units.getY()
        pad_board_loc = Location(
            fid_units,
            fid_loc.getX() + (px * fid_c - py * fid_s),
            fid_loc.getY() + (px * fid_s + py * fid_c),
            fid_loc.getZ(),
            0.0)
        pad_world = Utils2D.calculateBoardPlacementLocation(
            bl, pad_board_loc).convertToUnits(units)
        item["x"] = pad_world.getX()
        item["y"] = pad_world.getY()

    z_source = "Voron measured/cached Z" if ENABLE_VORON_Z_PROBE else "saved BoardLocation Z"
    log("Dispense anchor WORLD using %s X=%.4f Y=%.4f Z=%.4f R=%.4f baseZ=%.4f liftZ=%.4f" %
        (z_source, fid_world.getX(), fid_world.getY(), fid_world.getZ(),
         fid_world.getRotation(), base_z, lift_z))

    if newly_probed:
        move_z_only_no_dynamic_comp(
            dispense_head,
            units,
            lift_z,
            DISPENSE_SPEED_FACTOR,
            "Voron retract to calculated dispense lift Z")
        wait_still(
            dispense_head,
            "Voron retract to calculated dispense lift Z")

    move_z_only_no_dynamic_comp(
        dispense_head,
        units,
        mm_to_units(DISPENSE_SAFE_Z_MM, units),
        TRAVEL_SPEED_FACTOR,
        "dispense head ensure safe Z before start XY")
    wait_still(dispense_head, "dispense head ensure safe Z before start XY")
    runtime_checkpoint("dispense head at safe Z before start XY")

    start = dispense_head_location(
        units,
        fid_world.getX(),
        fid_world.getY(),
        mm_to_units(DISPENSE_SAFE_Z_MM, units))
    move_direct_no_dynamic_comp(
        dispense_head,
        start,
        TRAVEL_SPEED_FACTOR,
        "dispense head start no dynamic comp"
    )
    wait_still(dispense_head, "dispense head start")
    runtime_checkpoint("dispense head at safe path start")

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

    end = dispense_head_location(
        units,
        fid_world.getX(),
        fid_world.getY(),
        mm_to_units(DISPENSE_SAFE_Z_MM, units))
    move_direct_no_dynamic_comp(
        dispense_head,
        end,
        TRAVEL_SPEED_FACTOR,
        "dispense head safe end no dynamic comp"
    )
    wait_still(dispense_head, "dispense head safe end")
    runtime_checkpoint("dispense head at safe end")

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
        return False

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
    t0 = log_t("dap-off final safe Z retract: wait_still BEFORE WaitForStillstand")
    dispense_head.waitForCompletion(CompletionType.WaitForStillstand)
    DISPENSE_DIRTY.set(False)
    log_t("dap-off final safe Z retract: wait_still AFTER WaitForStillstand elapsed=%.3f ms" %
          elapsed_ms_since(t0))
    runtime_checkpoint("dap-off final safe Z retract completed", False)
    log("Dap-off complete.")
    return True


def wait_machine_still(mac, label):
    mac.getMotionPlanner().waitForCompletion(
        None, CompletionType.WaitForStillstand)
    log("%s complete." % label)


def move_head_to_safe_z_for_stop(head, mac, label):
    head.moveToSafeZ()
    wait_machine_still(mac, label)


def force_glue_off_for_stop(mac):
    glue = find_glue_dispense_actuator(mac)
    set_boolean_actuator_state_fast(
        glue, False, "safe-stop GlueDispense forced off")
    mac.getMotionPlanner().waitForCompletion(
        None, CompletionType.WaitForUnconditionalCoordination)
    log("Safe-stop GlueDispense off command coordinated.")


def safe_stop_cleanup(mac):
    errors = []
    head = mac.getDefaultHead()

    glue_is_off = False
    try:
        force_glue_off_for_stop(mac)
        glue_is_off = True
    except Exception, e:
        errors.append("force glue off: %s" % e)
        log_err("Safe-stop could not force glue off: %s" % e)

    safe_for_dap = False
    try:
        move_head_to_safe_z_for_stop(
            head, mac, "Safe-stop initial head safe Z")
        safe_for_dap = True
    except Exception, e:
        errors.append("initial safe Z: %s" % e)
        log_err("Safe-stop initial safe Z failed: %s" % e)

    if DISPENSE_DIRTY.get():
        if not safe_for_dap:
            try:
                if not safe_retract_dispense_head(
                        mac, "safe-stop retry before dap-off"):
                    raise Exception("dispense head safe retract returned failure")
                safe_for_dap = True
            except Exception, e:
                errors.append("safe Z retry before dap-off: %s" % e)
                log_err("Safe-stop safe Z retry before dap-off failed: %s" % e)

        if safe_for_dap and glue_is_off:
            try:
                log("Safe-stop detected dispensed material; running dap-off.")
                if not dap_off_dispense_head(mac):
                    raise Exception("dap-off is disabled by DAP_OFF_TOUCHES")
            except Exception, e:
                errors.append("dap-off: %s" % e)
                log_err("Safe-stop dap-off failed: %s" % e)
        else:
            reason = ("glue could not be forced off" if not glue_is_off else
                      "safe Z was not reached")
            errors.append("dap-off skipped because %s" % reason)
            log_err("Safe-stop skipped dap-off because %s." % reason)

        try:
            if not safe_retract_dispense_head(mac, "safe-stop post-dap-off"):
                raise Exception("dispense head safe retract returned failure")
        except Exception, e:
            errors.append("post-dap-off dispense retract: %s" % e)
            log_err("Safe-stop post-dap-off retract failed: %s" % e)

    if not glue_is_off:
        try:
            force_glue_off_for_stop(mac)
            glue_is_off = True
            log("Safe-stop glue-off retry succeeded before discard.")
        except Exception, e:
            errors.append("force glue off retry: %s" % e)
            log_err("Safe-stop glue-off retry failed: %s" % e)

    safe_for_discard = False
    if glue_is_off:
        try:
            move_head_to_safe_z_for_stop(
                head, mac, "Safe-stop head safe Z before discard")
            safe_for_discard = True
        except Exception, e:
            errors.append("safe Z before discard: %s" % e)
            log_err("Safe-stop cannot safely travel to discard: %s" % e)
    else:
        errors.append("part discard skipped because glue could not be forced off")
        log_err("Safe-stop skipped part discard because glue could not be forced off.")

    if safe_for_discard:
        for nozzle in head.getNozzles():
            try:
                carried_part = nozzle.getPart()
            except Exception, e:
                errors.append("read %s part state: %s" %
                              (object_name(nozzle), e))
                log_err("Could not read part state for nozzle %s: %s" %
                        (object_name(nozzle), e))
                continue
            if carried_part is None:
                continue
            try:
                log("Discarding part %s from nozzle %s at configured OpenPnP discard location." %
                    (object_name(carried_part), object_name(nozzle)))
                Cycles.discard(nozzle)
                if nozzle.getPart() is not None:
                    raise Exception("nozzle still reports a part after discard")
            except Exception, e:
                errors.append("discard from %s: %s" %
                              (object_name(nozzle), e))
                log_err("Safe-stop discard failed for nozzle %s: %s" %
                        (object_name(nozzle), e))

    try:
        move_head_to_safe_z_for_stop(
            head, mac, "Safe-stop final head safe Z")
    except Exception, e:
        errors.append("final safe Z: %s" % e)
        log_err("Safe-stop final safe Z failed: %s" % e)

    if len(errors) > 0:
        raise Exception("Safe-stop cleanup failed: %s" % "; ".join(errors))

    log("Safe-stop cleanup completed successfully.")


def get_reference_job_processor(event_processor):
    expected = "org.openpnp.machine.reference.ReferencePnpJobProcessor"
    cls = event_processor.getClass()
    class_name = str(cls.getName())

    if class_name == expected:
        return event_processor
    if not class_name.startswith(expected + "$"):
        raise Exception(
            "Unsupported Job.Starting jobProcessor class: %s" % class_name)

    try:
        outer_field = cls.getDeclaredField("this$0")
        outer_field.setAccessible(True)
        outer = outer_field.get(event_processor)
    except Exception, e:
        raise Exception(
            "Could not unwrap %s through verified this$0 field: %s" %
            (class_name, e))

    outer_name = str(outer.getClass().getName())
    if outer_name != expected:
        raise Exception(
            "Unexpected outer job processor class: %s" % outer_name)
    return outer


def get_prepared_job_placements(event_processor):
    processor = get_reference_job_processor(event_processor)
    try:
        field = processor.getClass().getDeclaredField("jobPlacements")
        field.setAccessible(True)
        return java_list_to_py(field.get(processor))
    except Exception, e:
        raise Exception(
            "Could not read verified ReferencePnpJobProcessor.jobPlacements field: %s" % e)


def set_job_placement_status(job_placement, wanted_name):
    status_class = job_placement.getStatus().getClass()
    for status in status_class.getEnumConstants():
        if str(status) == wanted_name:
            job_placement.setStatus(status)
            return
    raise Exception("JobPlacement status not found: %s" % wanted_name)


def same_java_object(left, right):
    if left is right:
        return True
    try:
        return bool(left.equals(right))
    except:
        return False


def find_job_placement(prepared, bl, pl):
    for job_placement in prepared:
        if (same_java_object(job_placement.getBoardLocation(), bl) and
                same_java_object(job_placement.getPlacement(), pl)):
            return job_placement
    return None


def get_enabled_dispense_work(job_obj):
    work = []
    for bl in java_list_to_py(job_obj.getBoardLocations()):
        if not bl.isEnabled():
            log("Skipping disabled board location %s." % object_name(bl))
            continue

        board = bl.getBoard()
        if board is None:
            raise Exception(
                "Enabled board location %s has no board." % object_name(bl))

        for pl in java_list_to_py(get_board_placements(board)):
            if not is_dispense_placement(pl):
                continue
            if not job_obj.retrieveEnabledState(bl, pl):
                log("Skipping disabled dispense board=%s placement=%s." %
                    (object_name(bl), object_name(pl)))
                continue
            work.append((bl, pl))
    return work


def mark_dispense_complete(job_obj, job_placement):
    bl = job_placement.getBoardLocation()
    pl = job_placement.getPlacement()
    job_obj.storePlacedStatus(bl, pl.getId(), True)
    if job_placement.getError() is not None:
        job_placement.setError(None)
    set_job_placement_status(job_placement, "Complete")
    log("Recorded dispense complete board=%s placement=%s." %
        (object_name(bl), object_name(pl)))


def mark_dispense_errored(job_obj, job_placement, error):
    bl = job_placement.getBoardLocation()
    pl = job_placement.getPlacement()
    recorded_error = JavaException(
        "Dispense failed for board=%s placement=%s: %s" %
        (object_name(bl), object_name(pl), str(error)))
    job_obj.storePlacedStatus(bl, pl.getId(), False)
    job_obj.storeErrorHandlingState(
        bl, pl, Placement.ErrorHandling.Defer)
    job_placement.setError(recorded_error)
    set_job_placement_status(job_placement, "Errored")
    log("Recorded deferred dispense error board=%s placement=%s." %
        (object_name(bl), object_name(pl)))


def panel_key_for_board(bl):
    try:
        parent = bl.getParent()
        if parent is not None:
            try:
                return str(parent.getUniqueId())
            except:
                return str(parent)
    except:
        pass
    return "<root>"


def board_location_key(bl):
    try:
        unique_id = bl.getUniqueId()
        if unique_id is not None and str(unique_id).strip() != "":
            return str(unique_id)
    except:
        pass
    try:
        return "%s:%d" % (object_name(bl), System.identityHashCode(bl))
    except:
        return str(bl)


def run_dispense_first_job(job_obj, processor):
    runtime_checkpoint("before preparing dispense-first job")
    enabled_work = get_enabled_dispense_work(job_obj)
    if len(enabled_work) == 0:
        log("No enabled Dispense placements on enabled boards; nothing to do.")
        return

    prepared = get_prepared_job_placements(processor)
    pending = []
    for bl, pl in enabled_work:
        if job_obj.retrievePlacedStatus(bl, pl.getId()):
            log("Skipping already dispensed board=%s placement=%s." %
                (object_name(bl), object_name(pl)))
            continue
        job_placement = find_job_placement(prepared, bl, pl)
        if job_placement is None:
            raise Exception(
                "Enabled pending dispense has no prepared JobPlacement: "
                "board=%s placement=%s." %
                (object_name(bl), object_name(pl)))
        pending.append(job_placement)

    log("Dispense-first pass found enabled=%d pending=%d already-placed=%d." %
        (len(enabled_work), len(pending), len(enabled_work) - len(pending)))
    if len(pending) == 0:
        log("All enabled Dispense placements are already complete.")
        return

    mac = Configuration.get().getMachine()

    successful = 0
    skipped = 0
    last_panel_key = None
    dispensed_since_dap = False
    voron_probe_cache = {}

    force_glue_off_safe(mac, "job-start initial state")
    runtime_checkpoint("after initial glue-off")

    for index, job_placement in enumerate(pending):
        runtime_checkpoint("before dispense placement %d/%d" %
                           (index + 1, len(pending)))
        bl = job_placement.getBoardLocation()
        pl = job_placement.getPlacement()
        board = bl.getBoard()
        prt = pl.getPart()
        panel_key = panel_key_for_board(bl)

        if (last_panel_key is not None and panel_key != last_panel_key and
                dispensed_since_dap):
            log("Panel transition %s -> %s; running dap-off." %
                (last_panel_key, panel_key))
            dap_off_dispense_head(mac)
            dispensed_since_dap = False
            runtime_checkpoint("after panel-transition dap-off")
        last_panel_key = panel_key

        label = ("dispense %d/%d board=%s placement=%s" %
                 (index + 1, len(pending), object_name(bl), object_name(pl)))
        original_bl_state = snapshot_board_state(bl)
        log("[%s] Starting." % label)

        try:
            measured = measure_board_fiducial_correction(
                mac, bl, board, prt, label)
            runtime_checkpoint("%s after fiducial correction" % label)
            residual_x, residual_y = apply_measured_board_pose(
                bl, measured)
            log("%s post-correction fiducial residual dx=%.6f dy=%.6f mm" %
                (label, residual_x, residual_y))
            runtime_checkpoint("%s after applying corrected board pose" % label)

            dispensed = dispense_current_placement(
                mac, board, bl, pl, voron_probe_cache)
            runtime_checkpoint("%s after dispense safe retract" % label)
            if dispensed < 1:
                raise Exception("Dispense completed without pulsing any pads.")

            mark_dispense_complete(job_obj, job_placement)
            successful += 1
            dispensed_since_dap = True
            log("[%s] Complete; affine board pose active." % label)

        except JobStopRequested, stop_e:
            try:
                restore_board_state(bl, original_bl_state)
                log("[%s] Restored original board pose for user-requested stop." %
                    label)
            except Exception, restore_e:
                log_err("[%s] Could not restore board pose during stop: %s" %
                        (label, restore_e))
                stop_e.cleanup_errors.append(
                    "restore board pose for %s: %s" % (label, restore_e))
            raise
        except Exception, e:
            skipped += 1
            log_err("[%s] Failed: %s" % (label, e))
            try:
                force_glue_off_safe(mac, "dispense-first placement failure")
            except:
                pass
            safe_retract_dispense_head(
                mac, "dispense-first placement failure")
            try:
                restore_board_state(bl, original_bl_state)
                log("[%s] Restored original board pose after failure." % label)
            except Exception, restore_e:
                log_err("[%s] Could not restore board pose: %s" %
                        (label, restore_e))
            mark_dispense_errored(job_obj, job_placement, e)

    if dispensed_since_dap:
        log("Final dispensed panel complete; running dap-off.")
        dap_off_dispense_head(mac)
        runtime_checkpoint("after final dap-off")

    force_glue_off_safe(mac, "job-start final state")
    log("Dispense-first pass complete: successful=%d skipped=%d total=%d." %
        (successful, skipped, len(pending)))

required_globals = ("job", "jobProcessor", "gui")
missing_globals = [name for name in required_globals if name not in globals()]
if missing_globals:
    raise Exception("Missing Job.Starting globals: %s" %
                    ", ".join(missing_globals))

SCRIPT_JOB_STATE = GuiJobStateApi(
    gui, jobState if "jobState" in globals() else None)
RUNTIME_MONITOR = JobRuntimeMonitor(
    SCRIPT_JOB_STATE, TAG, log, log_err, JOB_STATE_POLL_MS)
try:
    RUNTIME_MONITOR.start()
    runtime_checkpoint("dispense-first script start")
    run_pre_dispense_flush(Configuration.get().getMachine(), globals())
    run_dispense_first_job(job, jobProcessor)
except JobStopRequested, stop_e:
    log("User-requested stop received: %s" % stop_e)
    RUNTIME_MONITOR.begin_cleanup()
    safe_stop_cleanup(Configuration.get().getMachine())
    if len(stop_e.cleanup_errors) > 0:
        raise Exception("Safe-stop cleanup failed: %s" %
                        "; ".join(stop_e.cleanup_errors))
    SCRIPT_JOB_STATE.cancel()
except Exception, e:
    log_err("Dispense-first job failed: %s" % e)
    try:
        force_glue_off_safe(
            Configuration.get().getMachine(), "job-start fatal error")
    except Exception, glue_e:
        log_err("Could not force glue off after fatal error: %s" % glue_e)
    raise
finally:
    RUNTIME_MONITOR.stop()
