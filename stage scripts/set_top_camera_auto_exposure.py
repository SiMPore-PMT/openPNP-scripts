from org.openpnp.model import Configuration
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.vision.pipeline import CvStage


SCRIPT_NAME = "set_top_camera_auto_exposure"


def log(message):
    print("%s: %s" % (SCRIPT_NAME, message))


def java_items(items):
    if items is None:
        return []
    return [items.get(i) for i in range(items.size())]


def object_name(obj):
    if obj is None:
        return ""
    try:
        return str(obj.getName())
    except:
        return str(obj)


def is_down_camera(camera):
    if camera is None:
        return False
    try:
        return camera.getLooking() == SpiCamera.Looking.Down
    except:
        return False


def parse_auto_exposure_arg():
    args_string = ""
    try:
        args_string = str(args)
    except:
        try:
            args_string = str(stage.getArgs())
        except:
            args_string = ""

    args_string = args_string.strip()
    if not args_string:
        raise Exception("Pass ScriptRun args as 1 to enable auto exposure or 0 to disable it.")

    value = args_string.split()[0].strip().lower()
    if value in ("1", "true", "on", "yes", "enable", "enabled"):
        return True
    if value in ("0", "false", "off", "no", "disable", "disabled"):
        return False

    raise Exception("Invalid auto exposure arg '%s'. Use 1 or 0." % value)


def find_top_camera(machine):
    try:
        head = machine.getDefaultHead()
    except:
        head = None

    # Prefer the configured camera named "Top".
    try:
        if head is not None:
            camera = head.getDefaultCamera()
            if camera is not None and object_name(camera).lower() == "top":
                return camera
    except:
        pass

    try:
        if head is not None:
            for camera in java_items(head.getCameras()):
                if object_name(camera).lower() == "top":
                    return camera
    except:
        pass

    try:
        for camera in java_items(machine.getCameras()):
            if object_name(camera).lower() == "top":
                return camera
    except:
        pass

    # Usually the head default camera is the down-looking/top camera.
    try:
        camera = head.getDefaultCamera()
        if camera is not None and is_down_camera(camera):
            return camera
    except:
        pass

    try:
        if head is not None:
            for camera in java_items(head.getCameras()):
                if is_down_camera(camera):
                    return camera
    except:
        pass

    try:
        for camera in java_items(machine.getCameras()):
            if is_down_camera(camera):
                return camera
    except:
        pass

    raise Exception("No top/down-looking camera found.")


def set_auto_exposure(camera, enabled):
    try:
        exposure = camera.getExposure()
    except:
        raise Exception("Camera '%s' does not expose getExposure()." % object_name(camera))

    try:
        current = exposure.isAuto()
    except:
        current = None

    if current == enabled:
        log("camera '%s' auto exposure already %s; no change" %
            (object_name(camera), str(enabled).lower()))
        return

    exposure.setAuto(enabled)
    log("camera '%s' auto exposure set to %s" % (object_name(camera), str(enabled).lower()))


machine = Configuration.get().getMachine()
auto_exposure_enabled = parse_auto_exposure_arg()
top_camera = find_top_camera(machine)
set_auto_exposure(top_camera, auto_exposure_enabled)

# Keep the pipeline image unchanged while making this stage return a valid result.
result = CvStage.Result(pipeline.getWorkingImage())
result
