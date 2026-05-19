# pickHead_squaring.py
#
# Squares up the current nozzle tip using its calibration pipeline
# (rectAsList / oriented rectangle), zeros the rotation DRO using
# MotionPlanner.setGlobalOffsets(), then parks the pick head at a fixed
# location while keeping that squared rotation.

from org.openpnp.model import Configuration, Location, AxesLocation, LengthUnit
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.spi.MotionPlanner import CompletionType
from java.lang import Thread


RESULT_STAGE_NAME = "squareResults"
RESULT_STAGE_FALLBACKS = ["results", "orient", "preResults"]
ROT_AXIS_NAME = "a"
ANGLE_SIGN = 1.0

NOZZLE_NAME = "Pick Head"
PARK_X = 16.968
PARK_Y = 152.62
PARK_Z = 28.13
PARK_UNITS = LengthUnit.Millimeters


def log(msg):
    print "[Machine.AfterHoming.NozzleTipSquare] %s" % msg


def get_machine():
    return Configuration.get().getMachine()


def get_bottom_camera(machine):
    for cam in machine.getCameras():
        try:
            if cam.getLooking() == SpiCamera.Looking.Up:
                return cam
        except:
            pass
    return None


def get_default_nozzle(machine):
    head = machine.getDefaultHead()
    if head is None:
        return None
    return head.getDefaultNozzle()


def move_nozzle_to_bottom_cam(noz, cam):
    loc = cam.getLocation()
    log("Moving nozzle %s to bottom camera location: X=%.3f Y=%.3f Z=%.3f Rot=%.3f" %
        (noz.getName(), loc.getX(), loc.getY(), loc.getZ(), loc.getRotation()))
    noz.moveTo(loc)
    noz.waitForCompletion(CompletionType.WaitForStillstand)
    try:
        log("Settling at bottom camera for 1000ms before running pipeline.")
        Thread.sleep(1000)
    except Exception as e:
        log("WARNING: sleep interrupted: %s" % e)


def get_tip_pipeline(noz):
    tip = noz.getNozzleTip()
    if tip is None:
        log("No nozzle tip loaded on nozzle %s; skipping squaring." % noz.getName())
        return None
    calib = tip.getCalibration()
    if calib is None:
        log("No calibration on tip '%s'." % tip.getName())
        return None
    pl = calib.getPipeline()
    if pl is None:
        log("Calibration pipeline is None.")
        return None
    return pl


def java_class_name(obj):
    try:
        return obj.getClass().getName()
    except Exception:
        return str(type(obj))


def first_model_item(model):
    if model is None:
        return None

    try:
        cname = java_class_name(model)
        if "RotatedRect" in cname or "KeyPoint" in cname:
            return model
    except Exception:
        pass

    try:
        if hasattr(model, "size") and model.size() > 0:
            return model.get(0)
    except Exception:
        pass

    try:
        if hasattr(model, "__len__") and len(model) > 0:
            return model[0]
    except Exception:
        pass

    return None


def read_result_model(pipeline, stage_name):
    try:
        res = pipeline.getResult(stage_name)
    except Exception:
        return None
    if res is None:
        return None

    try:
        return res.getModel()
    except Exception:
        pass
    try:
        return res.model
    except Exception:
        return None


def raw_angle_from_item(item):
    if item is None:
        return None

    cname = java_class_name(item)
    log("Inspecting vision result item type %s: %s" % (cname, item))

    if "RotatedRect" in cname:
        try:
            return float(item.angle)
        except Exception as e:
            log("RotatedRect had no readable angle: %s" % e)
            return None

    if "KeyPoint" in cname:
        try:
            angle = float(item.angle)
            if angle == -1.0:
                log("Ignoring KeyPoint angle -1.0000; OpenCV uses this for unknown orientation.")
                return None
            return angle
        except Exception as e:
            log("KeyPoint had no readable angle: %s" % e)
            return None

    try:
        return float(item.angle)
    except Exception:
        pass

    # Last resort for RotatedRect-like toString(), e.g. "{ {x, y} 138x138 * 29.54 }".
    try:
        text = str(item)
        marker = text.rfind("*")
        if marker >= 0:
            tail = text[marker + 1:].replace("}", " ").strip()
            return float(tail.split()[0])
    except Exception:
        pass

    return None


def square_correction_degrees(raw_angle):
    correction = ((raw_angle + 45.0) % 90.0) - 45.0
    if correction == -45.0 and raw_angle > 0.0:
        correction = 45.0
    return correction


def run_pipeline_and_get_angle(pipeline, noz, cam):
    cv = pipeline
    try:
        cv.setProperty("camera", cam)
    except Exception as e:
        log("WARNING: could not set pipeline property 'camera': %s" % e)
    try:
        cv.setProperty("nozzle", noz)
    except Exception:
        pass

    cv.process()

    checked = []
    for stage_name in [RESULT_STAGE_NAME] + RESULT_STAGE_FALLBACKS:
        if stage_name in checked:
            continue
        checked.append(stage_name)

        model = read_result_model(cv, stage_name)
        if model is None:
            log("No model from stage '%s'." % stage_name)
            continue

        item = first_model_item(model)
        if item is None:
            log("Model from stage '%s' is empty or not indexable." % stage_name)
            continue

        raw_angle = raw_angle_from_item(item)
        if raw_angle is None:
            log("No usable angle from stage '%s'." % stage_name)
            continue

        correction = square_correction_degrees(raw_angle)
        log("Measured raw tip angle from stage '%s': %.4f deg; square correction: %.4f deg" %
            (stage_name, raw_angle, correction))
        return correction

    log("No usable angle from any checked stage: %s" % ", ".join(checked))
    return None


def find_rot_axis(machine):
    axes = machine.getAxes()
    chosen = None
    for axis in axes:
        axis_id = axis.getId() or "<?>"
        name = axis.getName() or "<?>"
        atype = str(axis.getType() or "<?>")
        log("Axis discovered: id='%s', name='%s', type=%s" % (axis_id, name, atype))
        if name.lower() == ROT_AXIS_NAME.lower():
            log("Selected rotational axis id='%s', name='%s' for DRO zeroing (preferred)" %
                (axis_id, name))
            return axis
        if chosen is None and "ROTATION" in atype.upper():
            chosen = axis
    if chosen is None:
        log("No rotational axis found; cannot zero DRO.")
    else:
        log("Selected rotational axis id='%s', name='%s' for DRO zeroing (fallback)" %
            (chosen.getId(), chosen.getName()))
    return chosen


def square_nozzle(machine, noz, angle_deg):
    loc = noz.getLocation()
    units = loc.getUnits()
    current_rot = loc.getRotation()
    target_rot = current_rot + ANGLE_SIGN * angle_deg
    log("Current rot = %.4f deg, angle = %.4f deg, target rot = %.4f deg" %
        (current_rot, angle_deg, target_rot))
    target_loc = Location(units, loc.getX(), loc.getY(), loc.getZ(), target_rot)
    noz.moveTo(target_loc)
    noz.waitForCompletion(CompletionType.WaitForStillstand)

    axis = find_rot_axis(machine)
    if axis is None:
        return
    try:
        planner = machine.getMotionPlanner()
        rot_zero = AxesLocation(axis, 0.0)
        planner.setGlobalOffsets(rot_zero)
        log("Global offset set: axis '%s' DRO = 0.0 deg (visual zero)." % axis.getId())
    except Exception as e:
        log("WARNING: failed to set global offsets for rotation axis: %s" % e)


def park_nozzle(machine, noz):
    try:
        log("[Park] Parking the machine...")
        head = machine.getDefaultHead()
        if head is None:
            log("[Park] ERROR: No default head found.")
            return
        park_noz = head.getNozzleByName(NOZZLE_NAME)
        if park_noz is None:
            log("[Park] ERROR: No nozzle named %s." % NOZZLE_NAME)
            return
        loc_before = park_noz.getLocation()
        park_loc = Location(PARK_UNITS, PARK_X, PARK_Y, PARK_Z, loc_before.getRotation())
        park_noz.moveTo(park_loc)
        park_noz.waitForCompletion(CompletionType.WaitForStillstand)
        parked = park_noz.getLocation()
        log("[Park] Nozzle %s parked at X=%.3f Y=%.3f Z=%.3f A=%.3f" %
            (park_noz.getName(), parked.getX(), parked.getY(), parked.getZ(), parked.getRotation()))
        log("[Park] Machine has been parked successfully.")
    except Exception as e:
        log("[Park] ERROR while parking: %s" % e)


try:
    machine = get_machine()
    noz = get_default_nozzle(machine)
    if noz is None:
        log("No default nozzle; skipping squaring.")
        raise SystemExit

    cam = get_bottom_camera(machine)
    if cam is None:
        log("No bottom camera; skipping squaring.")
        raise SystemExit

    log("AfterHoming: squaring tip for nozzle %s." % noz.getName())
    move_nozzle_to_bottom_cam(noz, cam)

    pipeline = get_tip_pipeline(noz)
    if pipeline is None:
        raise SystemExit

    angle = run_pipeline_and_get_angle(pipeline, noz, cam)
    if angle is None:
        log("No valid angle from pipeline; aborting squaring.")
        raise SystemExit

    square_nozzle(machine, noz, angle)
    log("AfterHoming: tip squared and DRO zeroed.")
    park_nozzle(machine, noz)

except SystemExit:
    pass
except Exception as e:
    log("ERROR: %s" % e)
    raise
