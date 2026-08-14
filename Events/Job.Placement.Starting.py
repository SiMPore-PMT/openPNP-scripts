# Job.Placement.Starting.py
#
# Re-measure the current pocket immediately before pick, then update the
# BoardLocation XY and rotation used by OpenPnP's placement calculation.
# This is the routine for picking one die at a time, assuming pdms has already been
# dispensed during Job.Starting.py.

import sys

from java.lang import Math, System, Thread
from org.openpnp.model import Configuration, Location, Placement
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import Cycles, MovableUtils, Utils2D
from org.opencv.core import RotatedRect

SCRIPTS_ROOT = str(scripting.getScriptsDirectory())
JOB_LIB_DIR = SCRIPTS_ROOT + "/Events/_lib"
if JOB_LIB_DIR not in sys.path:
    sys.path.insert(0, JOB_LIB_DIR)
from job_runtime import GuiJobStateApi, JobRuntimeMonitor, JobStopRequested


# ----------------- VISION CONFIG -----------------
MAX_NUDGE_MM = 4.0
APPLY_ROTATION = True
EXPECTED_ANGLE_DEG = 0.0
DESIGN_BOARD_ROT_DEG = 0.0

PRE_CAPTURE_SETTLE_MS = 250
RESULT_STAGE_KEY = "results"
RETRY_DELAY_MS = 250
VISION_MEASUREMENT_ATTEMPTS = 2
RECENTER_MAX_PASSES = 3
RECENTER_TOLERANCE_MM = 0.02

FLIP_IMAGE_X = False
FLIP_IMAGE_Y = True

TAG = "Job.Placement.Starting"
JOB_STATE_POLL_MS = 50


def log(msg):
    print "[%s] %s" % (TAG, str(msg))


def log_err(msg):
    print "[%s ERROR] %s" % (TAG, str(msg))


class VisionMeasurementError(Exception):
    pass


RUNTIME_MONITOR = None


def runtime_checkpoint(label, allow_pause=True):
    if RUNTIME_MONITOR is not None:
        RUNTIME_MONITOR.checkpoint(label, allow_pause)


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

def move_safe_wait(movable, target, label):
    runtime_checkpoint("before %s" % label)
    MovableUtils.moveToLocationAtSafeZ(movable, target)
    wait_still(movable, label)
    runtime_checkpoint("after %s" % label)


def cooperative_sleep(ms, label, allow_pause=True):
    remaining = int(ms)
    while remaining > 0:
        runtime_checkpoint(label, allow_pause)
        interval = min(remaining, JOB_STATE_POLL_MS)
        Thread.sleep(interval)
        remaining -= interval
    runtime_checkpoint(label, allow_pause)

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

        for recenter_pass in range(1, RECENTER_MAX_PASSES + 1):
            wait_still(
                cam,
                "%s recenter pass %d camera" % (label, recenter_pass))
            if PRE_CAPTURE_SETTLE_MS > 0:
                cooperative_sleep(
                    PRE_CAPTURE_SETTLE_MS,
                    "%s recenter pass %d capture settle" %
                    (label, recenter_pass))
                log("[%s] Recenter pass %d settled %d ms before pipeline.process()." %
                    (label, recenter_pass, PRE_CAPTURE_SETTLE_MS))

            try:
                runtime_checkpoint(
                    "%s recenter pass %d before vision capture" %
                    (label, recenter_pass))
                pipeline.process()
                runtime_checkpoint(
                    "%s recenter pass %d after vision capture" %
                    (label, recenter_pass))
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


def get_reference_job_processor(event_processor):
    expected = "org.openpnp.machine.reference.ReferencePnpJobProcessor"
    cls = event_processor.getClass()
    class_name = str(cls.getName())

    if class_name == expected:
        return event_processor
    if not class_name.startswith(expected + "$"):
        raise Exception(
            "Unsupported Job.Placement.Starting jobProcessor class: %s" %
            class_name)

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


def same_java_object(left, right):
    if left is right:
        return True
    try:
        return bool(left.equals(right))
    except:
        return False


def find_current_job_placement(processor, bl, pl):
    for job_placement in get_prepared_job_placements(processor):
        if (same_java_object(job_placement.getBoardLocation(), bl) and
                same_java_object(job_placement.getPlacement(), pl)):
            return job_placement
    return None


def is_dispense_placement(pl):
    return pl is not None and pl.getType() == Placement.Type.Dispense


def enforce_dispense_prerequisites(job_obj, processor, bl, current_pl):
    board = bl.getBoard()
    if board is None:
        raise Exception("boardLocation.getBoard() returned null.")

    enabled_dispense = []
    for dispense_pl in java_list_to_py(get_board_placements(board)):
        if (is_dispense_placement(dispense_pl) and
                job_obj.retrieveEnabledState(bl, dispense_pl)):
            enabled_dispense.append(dispense_pl)

    if len(enabled_dispense) == 0:
        log("No enabled Dispense prerequisite for this board; prealignment allowed.")
        return

    unmet = []
    for dispense_pl in enabled_dispense:
        if job_obj.retrievePlacedStatus(bl, dispense_pl.getId()):
            continue

        job_placement = find_current_job_placement(
            processor, bl, dispense_pl)
        status = "not placed"
        if job_placement is not None:
            status = str(job_placement.getStatus())
            try:
                error = job_placement.getError()
                if error is not None:
                    status = "%s: %s" % (status, str(error))
            except:
                pass
        unmet.append("%s [%s]" % (dispense_pl.getId(), status))

    if len(unmet) == 0:
        log("All %d enabled Dispense prerequisites are complete; prealignment allowed." %
            len(enabled_dispense))
        return

    job_obj.storeErrorHandlingState(
        bl, current_pl, Placement.ErrorHandling.Defer)
    raise Exception(
        "Pick blocked because enabled dispense prerequisite(s) are incomplete: %s" %
        "; ".join(unmet))


def defer_current_instance(job_obj, bl, pl, reason):
    # The Pick step must receive an exception to abort the pick that is already
    # in progress. Defer makes ReferencePnpJobProcessor record it as Errored and
    # continue the job. Keep placed=False because no die was picked or placed.
    job_obj.storeErrorHandlingState(bl, pl, Placement.ErrorHandling.Defer)
    job_obj.storePlacedStatus(bl, pl.getId(), False)
    log("Deferred current board instance; placement remains unplaced: %s" % reason)


def run_placement_vision_only(job_obj, processor, bl, pl, prt):
    mac = Configuration.get().getMachine()
    board = bl.getBoard()
    if board is None:
        raise Exception("boardLocation.getBoard() returned null.")

    original_bl_loc = bl.getLocation()
    try:
        runtime_checkpoint("before pre-placement vision")
        measured = measure_board_fiducial_correction(
            mac, bl, board, prt, "pre-placement")
        runtime_checkpoint("after pre-placement vision")
        corrected_bl_loc = measured["boardLocation"]
        bl.setLocation(corrected_bl_loc)
        runtime_checkpoint("after applying corrected board pose")
        final_loc = Utils2D.calculateBoardPlacementLocation(bl, pl.getLocation())
        log("Vision-only board update complete: board X=%.4f Y=%.4f R=%.4f; placement X=%.4f Y=%.4f R=%.4f" %
            (corrected_bl_loc.getX(), corrected_bl_loc.getY(),
             corrected_bl_loc.getRotation(), final_loc.getX(),
             final_loc.getY(), final_loc.getRotation()))
    except JobStopRequested, stop_e:
        try:
            bl.setLocation(original_bl_loc)
            log("Restored original board pose for user-requested stop.")
        except Exception, restore_e:
            log_err("Could not restore board pose during stop: %s" % restore_e)
            stop_e.cleanup_errors.append(
                "restore board pose: %s" % restore_e)
        raise
    except Exception, e:
        try:
            bl.setLocation(original_bl_loc)
            log("Restored original board pose after placement-time vision failure.")
        except Exception, restore_e:
            log_err("Could not restore board pose: %s" % restore_e)
        defer_current_instance(job_obj, bl, pl, str(e))
        raise Exception(
            "Pocket not confirmed; placement deferred and remains unplaced: %s" % e)


def wait_machine_still(mac, label):
    mac.getMotionPlanner().waitForCompletion(
        None, CompletionType.WaitForStillstand)
    log("%s complete." % label)


def move_head_to_safe_z_for_stop(head, mac, label):
    head.moveToSafeZ()
    wait_machine_still(mac, label)


def safe_stop_cleanup(mac):
    errors = []
    head = mac.getDefaultHead()

    safe_for_discard = False
    try:
        move_head_to_safe_z_for_stop(
            head, mac, "Safe-stop initial head safe Z")
        safe_for_discard = True
    except Exception, e:
        errors.append("initial safe Z: %s" % e)
        log_err("Safe-stop initial safe Z failed: %s" % e)

    if not safe_for_discard:
        try:
            move_head_to_safe_z_for_stop(
                head, mac, "Safe-stop retry head safe Z before discard")
            safe_for_discard = True
        except Exception, e:
            errors.append("safe Z before discard: %s" % e)
            log_err("Safe-stop cannot safely travel to discard: %s" % e)

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

required_globals = (
    "job", "jobProcessor", "placement", "boardLocation", "gui")
missing_globals = [name for name in required_globals if name not in globals()]
if missing_globals:
    raise Exception("Missing Job.Placement.Starting globals: %s" %
                    ", ".join(missing_globals))

SCRIPT_JOB_STATE = GuiJobStateApi(
    gui, jobState if "jobState" in globals() else None)
RUNTIME_MONITOR = JobRuntimeMonitor(
    SCRIPT_JOB_STATE, TAG, log, log_err, JOB_STATE_POLL_MS)
try:
    RUNTIME_MONITOR.start()
    runtime_checkpoint("placement script start")
    enforce_dispense_prerequisites(
        job, jobProcessor, boardLocation, placement)
    runtime_checkpoint("dispense prerequisites satisfied")
    run_placement_vision_only(
        job, jobProcessor, boardLocation, placement,
        part if "part" in globals() else placement.getPart())
except JobStopRequested, stop_e:
    log("User-requested stop received: %s" % stop_e)
    RUNTIME_MONITOR.begin_cleanup()
    safe_stop_cleanup(Configuration.get().getMachine())
    if len(stop_e.cleanup_errors) > 0:
        raise Exception("Safe-stop cleanup failed: %s" %
                        "; ".join(stop_e.cleanup_errors))
    SCRIPT_JOB_STATE.cancel()
except Exception, e:
    log_err(e)
    raise
finally:
    RUNTIME_MONITOR.stop()
