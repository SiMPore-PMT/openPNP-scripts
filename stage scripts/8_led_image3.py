# OpenPnP ScriptRun uses javax.script eval(reader) and only accepts the eval
# return value as the stage result. Jython returns None for normal multi-statement
# Python files, so keep this file as one expression that executes the body and
# then evaluates to __script_result.
eval(compile(r'''
from org.openpnp.model import Configuration
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.vision.pipeline import CvStage
from java.lang import Thread

SCRIPT_NAME = "6_led_rgb_full_swap_image"

ACTUATOR_NAME = "LED Color Driver"

# Full LED bank. I0 and I4 are intentionally unused.
LED_COUNT = 8

DEFAULT_INTENSITY = 255
SETTLE_MS = 75


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


def clamp_0_255(v):
    v = int(v)
    if v < 0:
        return 0
    if v > 255:
        return 255
    return v


def get_args_string():
    try:
        return str(args).strip()
    except:
        try:
            return str(stage.getArgs()).strip()
        except:
            return ""


def parse_args():
    cfg = {
        "actuator": ACTUATOR_NAME,
        "intensity": DEFAULT_INTENSITY,
        "settle": SETTLE_MS
    }

    s = get_args_string()
    if not s:
        return cfg

    tokens = s.split()

    if len(tokens) == 1 and tokens[0].find("=") < 0:
        cfg["intensity"] = clamp_0_255(tokens[0])
        return cfg

    i = 0
    while i < len(tokens):
        token = tokens[i]
        if token.find("=") >= 0:
            key, value = token.split("=", 1)
        else:
            key = token
            value = ""

        key_l = key.strip().lower()
        value = value.strip().strip('"')

        if key_l in ("actuator", "actuator_name"):
            if value:
                parts = [value]
                j = i + 1
                while j < len(tokens) and tokens[j].find("=") < 0:
                    parts.append(tokens[j].strip('"'))
                    j += 1
                cfg["actuator"] = " ".join(parts)
                i = j
                continue

        elif key_l in ("p", "intensity"):
            cfg["intensity"] = clamp_0_255(value)

        elif key_l in ("settle", "settle_ms", "delay", "delay_ms"):
            cfg["settle"] = max(0, int(value))

        i += 1

    return cfg


def is_down_camera(camera):
    if camera is None:
        return False
    try:
        return camera.getLooking() == SpiCamera.Looking.Down
    except:
        return False


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


def make_actuator_value(index, intensity, r, g, b):
    return "I%d P%d R%d U%d B%d" % (
        int(index),
        clamp_0_255(intensity),
        clamp_0_255(r),
        clamp_0_255(g),
        clamp_0_255(b)
    )


def actuate_led(actuator, index, intensity, r, g, b):
    value = make_actuator_value(index, intensity, r, g, b)
    actuator.actuate(value)
    log("M150 %s" % value)
    return value


def all_leds_off(actuator):
    for i in range(LED_COUNT):
        actuate_led(actuator, i, 0, 0, 0, 0)


def color_values(color_name, intensity):
    if color_name == "red":
        return intensity, 0, 0
    if color_name == "green":
        return 0, intensity, 0
    if color_name == "blue":
        return 0, 0, intensity

    return 0, 0, 0


def set_one_state(actuator, state, intensity):
    # Explicitly turn skipped LEDs off first.
    for led_index in range(LED_COUNT):
        if led_index not in state:
            actuate_led(actuator, led_index, 0, 0, 0, 0)

    # Then set each active LED independently.
    for led_index in sorted(state.keys()):
        color_name = state[led_index]
        r, g, b = color_values(color_name, intensity)
        actuate_led(actuator, led_index, intensity, r, g, b)
        log("set I%d to %s" % (led_index, color_name))


def capture_image(camera):
    try:
        return camera.capture()
    except Exception as e:
        raise Exception("Camera '%s' could not capture an image: %s" % (object_name(camera), e))


def to_mat(image):
    try:
        return image.clone()
    except:
        pass

    try:
        from org.openpnp.util import OpenCvUtils
        return OpenCvUtils.toMat(image)
    except Exception as e:
        raise Exception("Could not convert captured image to Mat: %s" % e)


def overlay_images(mats):
    if mats is None or len(mats) == 0:
        raise Exception("No images were captured.")

    composite = mats[0].clone()

    try:
        from org.opencv.core import Core
        for i in range(1, len(mats)):
            Core.max(composite, mats[i], composite)
        return composite
    except:
        pass

    try:
        from org.opencv.core import Core
        for i in range(1, len(mats)):
            Core.addWeighted(composite, 1.0, mats[i], 1.0, 0.0, composite)
        return composite
    except Exception as e:
        raise Exception("Could not overlay images: %s" % e)


machine = Configuration.get().getMachine()
cfg = parse_args()

led = machine.getActuatorByName(cfg["actuator"])
if led is None:
    raise Exception("Actuator '%s' not found. Check the name in Machine Setup -> Actuators." % cfg["actuator"])

camera = find_top_camera(machine)
log("using camera '%s' and actuator '%s'" % (object_name(camera), cfg["actuator"]))

# I0 skipped.
# I1 red
# I2 blue
# I3 green
# I4 skipped.
# I5 blue
# I6 green
# I7 red
#
# Then every active LED rotates through all three colors.
COLOR_STATES = [
    {
        1: "red",
        2: "blue",
        3: "green",
        5: "blue",
        6: "green",
        7: "red"
    },
    {
        1: "blue",
        2: "green",
        3: "red",
        5: "green",
        6: "red",
        7: "blue"
    },
    {
        1: "green",
        2: "red",
        3: "blue",
        5: "red",
        6: "blue",
        7: "green"
    }
]

captured = []

try:
    all_leds_off(led)
    Thread.sleep(cfg["settle"])

    for state_index in range(len(COLOR_STATES)):
        log("setting color state %d of %d" % (state_index + 1, len(COLOR_STATES)))

        set_one_state(led, COLOR_STATES[state_index], cfg["intensity"])
        Thread.sleep(cfg["settle"])

        image = capture_image(camera)
        captured.append(to_mat(image))
        log("captured image %d of %d" % (state_index + 1, len(COLOR_STATES)))

        Thread.sleep(cfg["settle"])

    all_leds_off(led)
    Thread.sleep(cfg["settle"])

finally:
    try:
        all_leds_off(led)
    except Exception as e:
        log("warning: failed to turn all LEDs off: %s" % e)

log("captured %d images total" % len(captured))

composite = overlay_images(captured)
__script_result = CvStage.Result(composite)
''', '6_led_rgb_full_swap_image.py', 'exec')) or __script_result