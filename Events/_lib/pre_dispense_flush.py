from java.lang import Math
from org.openpnp.model import LengthUnit


def _random_pad_point(api):
    radius = api["DAP_OFF_RANDOM_DIAMETER_MM"] / 2.0
    distance = Math.sqrt(Math.random()) * radius
    angle = Math.random() * 2.0 * Math.PI
    return (api["DAP_OFF_X_MM"] + Math.cos(angle) * distance,
            api["DAP_OFF_Y_MM"] + Math.sin(angle) * distance)


def run_pre_dispense_flush(mac, api):
    if not api["FLUSH_ENABLED"]:
        api["log"]("Pre-dispense needle flush is disabled.")
        return False

    try:
        count_value = float(api["FLUSH_PULSES"])
        count = int(count_value)
    except:
        raise Exception("FLUSH_PULSES must be a positive whole number.")
    if count_value != float(count) or count < 1:
        raise Exception("FLUSH_PULSES must be a positive whole number.")

    head = api["find_dispense_head"](mac)
    glue = api["find_glue_dispense_actuator"](mac)
    units = LengthUnit.Millimeters
    x, y = _random_pad_point(api)
    api["log"]("Pre-dispense needle flush: X=%.4f Y=%.4f Z=%.4f pulses=%d." %
               (x, y, api["DAP_OFF_Z_MM"], count))

    api["force_glue_off_safe"](mac, "pre-dispense flush initial state")
    api["move_z_only_no_dynamic_comp"](
        head, units, api["DISPENSE_SAFE_Z_MM"], api["TRAVEL_SPEED_FACTOR"],
        "pre-dispense flush ensure safe Z")
    api["wait_coord"](head, "pre-dispense flush ensure safe Z")
    api["move_direct_no_dynamic_comp"](
        head, api["dispense_head_location"](
            units, x, y, api["DISPENSE_SAFE_Z_MM"]),
        api["TRAVEL_SPEED_FACTOR"], "pre-dispense flush move over pad")
    api["wait_still"](head, "pre-dispense flush move over pad")
    api["move_z_only_no_dynamic_comp"](
        head, units, api["DAP_OFF_Z_MM"], api["DISPENSE_SPEED_FACTOR"],
        "pre-dispense flush lower needle")
    api["wait_coord"](head, "pre-dispense flush needle at pad")

    try:
        for index in range(count):
            api["runtime_checkpoint"](
                "before pre-dispense flush pulse %d/%d" % (index + 1, count),
                False)
            api["pulse_glue_controller"](glue, mac, index + 1)
    finally:
        api["force_glue_off_safe"](mac, "pre-dispense flush final state")
        api["move_z_only_no_dynamic_comp"](
            head, units, api["DISPENSE_SAFE_Z_MM"],
            api["TRAVEL_SPEED_FACTOR"], "pre-dispense flush safe retract")
        api["wait_still"](head, "pre-dispense flush safe retract")

    api["log"]("Pre-dispense flush complete; dabbing at a new random point.")
    if not api["dap_off_dispense_head"](mac):
        raise Exception("Pre-dispense flush requires DAP_OFF_TOUCHES >= 1.")
    api["runtime_checkpoint"]("after pre-dispense flush dab-off")
    return True
