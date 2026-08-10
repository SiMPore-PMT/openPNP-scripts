import json
import os
import traceback

from java.lang import System
from org.openpnp.model import Configuration, Location, LengthUnit
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import VisionUtils, MovableUtils

CONFIG_PATH = "/home/engineering-simpore/.openpnp2/scripts/lumen_calibration_gui_config.json"


def mm_loc(x, y, z, r):
    return Location(LengthUnit.Millimeters, float(x), float(y), float(z), float(r))


def loc_to_str(loc):
    loc = loc.convertToUnits(LengthUnit.Millimeters)
    return "(%.6f, %.6f, %.6f, %.6f mm)" % (
        float(loc.getX()), float(loc.getY()), float(loc.getZ()), float(loc.getRotation())
    )


def java_list_to_py(model):
    if model is None:
        return []
    try:
        return [model.get(i) for i in range(model.size())]
    except:
        return list(model)


def parse_circle(item):
    center = None
    for getter in ["getCenter", "getPt", "getPoint"]:
        try:
            center = getattr(item, getter)()
        except:
            pass
        if center is not None:
            break
    if center is None:
        raise Exception("Could not read circle center")

    try:
        x = float(center.getX())
        y = float(center.getY())
    except:
        raise Exception("Could not read circle center XY")

    diameter = None
    for getter in ["getDiameter", "getD", "getSize"]:
        try:
            diameter = float(getattr(item, getter)())
            break
        except:
            pass
    if diameter is None:
        for getter in ["getRadius", "getR"]:
            try:
                diameter = float(getattr(item, getter)()) * 2.0
                break
            except:
                pass
    if diameter is None:
        raise Exception("Could not read circle diameter")

    return {"x_px": x, "y_px": y, "diameter_px": diameter}


def get_default_camera(machine):
    head = machine.getDefaultHead()
    if head is None:
        raise Exception("No default head")
    camera = head.getDefaultCamera()
    if camera is None:
        raise Exception("No default camera")
    return camera


def get_default_nozzle(machine):
    head = machine.getDefaultHead()
    if head is None:
        raise Exception("No default head")
    nozzle = head.getDefaultNozzle()
    if nozzle is None:
        raise Exception("No default nozzle")
    return nozzle


def capture_top(camera, nozzle, part, config):
    settings = part.getFiducialVisionSettings()
    if settings is None:
        raise Exception("No fiducial vision settings on part")

    pipeline = settings.getPipeline()
    if pipeline is None:
        raise Exception("No pipeline for part fiducial vision settings")

    pipeline.setProperty("camera", camera)
    pipeline.setProperty("nozzle", nozzle)
    pipeline.setProperty("part", part)

    camera.settleAndCapture()
    pipeline.process()

    result = pipeline.getResult(config["result_stage"])
    if result is None:
        raise Exception("Pipeline returned no result")

    items = java_list_to_py(result.getModel())
    if len(items) != 2:
        raise Exception("Pipeline did not return exactly 2 shapes")

    circles = [parse_circle(items[0]), parse_circle(items[1])]
    circles.sort(key=lambda x: x["diameter_px"], reverse=True)
    big_circle = circles[0]
    small_circle = circles[1]

    camera_only = VisionUtils.getPixelLocation(camera, float(big_circle["x_px"]), float(big_circle["y_px"]))
    camera_only = camera_only.convertToUnits(LengthUnit.Millimeters)
    tool_aware = VisionUtils.getPixelLocation(camera, nozzle, float(big_circle["x_px"]), float(big_circle["y_px"]))
    tool_aware = tool_aware.convertToUnits(LengthUnit.Millimeters)

    return {
        "camera_only": camera_only,
        "tool_aware": tool_aware,
        "big_circle": big_circle,
        "small_circle": small_circle,
    }


def main():
    config = json.load(open(CONFIG_PATH, 'r'))
    machine = Configuration.get().getMachine()
    camera = get_default_camera(machine)
    nozzle = get_default_nozzle(machine)
    part = Configuration.get().getPart(config["part_id"])
    if part is None:
        raise Exception("Part not found: %s" % config["part_id"])

    storage = config["storage_location"]
    z_target = float(storage["z"])
    if config.get("use_openpnp_part_height", False):
        z_target += float(config.get("part_height_mm", 0.0))
    if config.get("measurement_z_mm") is not None:
        z_target = float(config["measurement_z_mm"])

    current_camera = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
    target = mm_loc(storage["x"], storage["y"], z_target, current_camera.getRotation())

    System.out.println("STORAGE TARGET: %s" % target)
    MovableUtils.moveToLocationAtSafeZ(camera, target)
    camera.waitForCompletion(CompletionType.WaitForStillstand)
    System.out.println("CAMERA AT: %s" % camera.getLocation().convertToUnits(LengthUnit.Millimeters))

    result = capture_top(camera, nozzle, part, config)

    System.out.println("=== DIAGNOSTIC RESULT ===")
    System.out.println("camera_only: %s" % loc_to_str(result["camera_only"]))
    System.out.println("tool_aware: %s" % loc_to_str(result["tool_aware"]))
    System.out.println("big_circle px: %s" % result["big_circle"])
    System.out.println("small_circle px: %s" % result["small_circle"])
    System.out.println("run again after lifting the nozzle 2 mm and compare camera_only XY.")

try:
    main()
except Exception as e:
    System.out.println("ERROR: %s" % e)
    traceback.print_exc()
