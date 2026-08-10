from org.openpnp.model import Configuration
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.vision.FluentCv import ColorSpace
from org.openpnp.util import OpenCvUtils
from org.opencv.core import Core
from java.lang import Thread


SCRIPT_NAME = "capture_top_camera_with_temp_exposure_ms"


DEFAULT_EXPOSURE_APPLY_DELAY_MS = 500
DEFAULT_DISCARD_FRAMES = 8


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


def parse_args():
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
        raise Exception(
            "Pass ScriptRun args as: exposure [delayMs] [discardFrames], for example: 20 500 8"
        )

    parts = args_string.split()

    try:
        exposure_value = int(round(float(parts[0])))
    except:
        raise Exception("Invalid exposure arg '%s'. Use a number, for example: 20" % parts[0])

    delay_ms = DEFAULT_EXPOSURE_APPLY_DELAY_MS
    discard_frames = DEFAULT_DISCARD_FRAMES

    if len(parts) >= 2:
        try:
            delay_ms = int(round(float(parts[1])))
        except:
            raise Exception("Invalid delayMs arg '%s'. Use a number, for example: 500" % parts[1])

    if len(parts) >= 3:
        try:
            discard_frames = int(round(float(parts[2])))
        except:
            raise Exception("Invalid discardFrames arg '%s'. Use a number, for example: 8" % parts[2])

    return exposure_value, delay_ms, discard_frames


def find_top_camera(machine):
    try:
        head = machine.getDefaultHead()
    except:
        head = None

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


def get_pipeline_or_top_camera():
    try:
        camera = pipeline.getProperty("camera")
        if camera is not None:
            return camera
    except:
        pass

    return find_top_camera(Configuration.get().getMachine())


def mat_sum(mat):
    try:
        s = Core.sumElems(mat)
        return s.val[0] + s.val[1] + s.val[2] + s.val[3]
    except:
        return -1


def image_sample_sum(buffered_image):
    if buffered_image is None:
        return -1

    w = buffered_image.getWidth()
    h = buffered_image.getHeight()
    if w <= 0 or h <= 0:
        return -1

    total = 0
    step_x = max(1, w // 16)
    step_y = max(1, h // 16)

    y = 0
    while y < h:
        x = 0
        while x < w:
            rgb = buffered_image.getRGB(x, y)
            total += (rgb >> 16) & 0xff
            total += (rgb >> 8) & 0xff
            total += rgb & 0xff
            x += step_x
        y += step_y

    return total


def flush_frames(camera, count):
    last = None
    for i in range(count):
        last = camera.capture()
        log("discarded frame %s/%s sampleSum=%s" %
            (i + 1, count, image_sample_sum(last)))
    return last


def capture_into_pipeline_working_image(camera, temporary_exposure, delay_ms, discard_frames):
    try:
        exposure = camera.getExposure()
    except:
        raise Exception("Camera '%s' does not expose getExposure()." % object_name(camera))

    original_exposure = exposure.getValue()

    log("camera '%s' original exposure is %s" %
        (object_name(camera), original_exposure))

    try:
        # Put the light on BEFORE waiting/flushing, so the camera is warming/flushing
        # under the same lighting as the final image.
        camera.actuateLightBeforeCapture(None)

        try:
            exposure.setValue(temporary_exposure)
            log("camera '%s' exposure temporarily set to %s; readback=%s" %
                (object_name(camera), temporary_exposure, exposure.getValue()))

            if delay_ms > 0:
                log("waiting %s ms for exposure to propagate" % delay_ms)
                Thread.sleep(delay_ms)

            if discard_frames > 0:
                log("discarding %s frames after exposure change" % discard_frames)
                flush_frames(camera, discard_frames)

            # Do the actual final capture. Use Skip here because we already waited/flushed.
            # This avoids auto-settle returning one of the frames from its internal settling loop.
            buffered_image = camera.settleAndCapture(SpiCamera.SettleOption.Skip)

            if buffered_image is None:
                raise Exception("camera '%s' returned no BufferedImage" %
                    object_name(camera))

            pipeline.setLastCapturedImage(buffered_image)

            log("FINAL BufferedImage type=%s size=%sx%s sampleSum=%s exposureReadback=%s" %
                (
                    buffered_image.getType(),
                    buffered_image.getWidth(),
                    buffered_image.getHeight(),
                    image_sample_sum(buffered_image),
                    exposure.getValue()
                ))

            captured_mat = OpenCvUtils.toMat(buffered_image)

            log("FINAL Mat size=%sx%s channels=%s type=%s matSum=%s" %
                (
                    captured_mat.cols(),
                    captured_mat.rows(),
                    captured_mat.channels(),
                    captured_mat.type(),
                    mat_sum(captured_mat)
                ))

            working = pipeline.getWorkingImage()
            captured_mat.copyTo(working)
            pipeline.setWorkingColorSpace(ColorSpace.Bgr)

            log("pipeline working Mat after copy size=%sx%s channels=%s type=%s matSum=%s" %
                (
                    working.cols(),
                    working.rows(),
                    working.channels(),
                    working.type(),
                    mat_sum(working)
                ))

        finally:
            camera.actuateLightAfterCapture()

    finally:
        exposure.setValue(original_exposure)
        log("camera '%s' exposure restored to %s; readback=%s" %
            (object_name(camera), original_exposure, exposure.getValue()))


temporary_exposure, delay_ms, discard_frames = parse_args()
camera = get_pipeline_or_top_camera()

capture_into_pipeline_working_image(camera, temporary_exposure, delay_ms, discard_frames)

# Return null intentionally; we already copied into pipeline.getWorkingImage().
None