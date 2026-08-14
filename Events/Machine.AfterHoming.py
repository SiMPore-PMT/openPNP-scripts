# # #  ----------- ACTIVE SCRIPT --------------
# # Machine.AfterHoming.py
# #
# # This script runs after homing, and is used to square the machine axes using the pick head.

# import imp
# import os


# def scripts_root():
#     try:
#         return scripting.getScriptsDirectory().toString()
#     except:
#         pass
#     try:
#         return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#     except:
#         return os.getcwd()


# SCRIPT_DIR = scripts_root()
# SQUARING_PATH = os.path.join(SCRIPT_DIR, "Events", "Homing", "pickHead_squaring.py")

# imp.load_source("pickHead_squaring", SQUARING_PATH)
# ----------- ACTIVE SCRIPT --------------
# Machine.AfterHoming.py
#
# Required order:
#   1. Physically square the pick-head nozzle and set A-axis global zero.
#   2. Run nozzle-tip runout calibration using that finalized rotation zero.
#   3. Park at the location defined by pickHead_squaring.py.

import imp
import os


LOG_PREFIX = "[Machine.AfterHoming.SquareThenRunout]"
PARK_POSITION_TOLERANCE = 0.05


def log(message):
    print("%s %s" % (LOG_PREFIX, message))


def scripts_root():
    try:
        return scripting.getScriptsDirectory().toString()
    except:
        pass

    try:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    except:
        return os.getcwd()


def require_squaring_success(module):
    nozzle = getattr(module, "noz", None)
    camera = getattr(module, "cam", None)
    pipeline = getattr(module, "pipeline", None)
    measured_angle = getattr(module, "angle", None)

    if nozzle is None:
        raise Exception(
            "Pick-head squaring did not find the default nozzle"
        )

    if camera is None:
        raise Exception(
            "Pick-head squaring did not find the bottom camera"
        )

    if pipeline is None:
        raise Exception(
            "Pick-head squaring did not find a nozzle-tip pipeline"
        )

    if measured_angle is None:
        raise Exception(
            "Pick-head squaring did not obtain a valid square angle"
        )

    return nozzle


def get_mounted_tip_calibration(nozzle):
    tip = nozzle.getNozzleTip()

    if tip is None:
        raise Exception(
            "No nozzle tip is mounted on '%s'" % nozzle.getName()
        )

    calibration = tip.getCalibration()

    if calibration is None:
        raise Exception(
            "Mounted nozzle tip '%s' has no calibration object"
            % tip.getName()
        )

    try:
        enabled = calibration.isEnabled()
    except:
        enabled = True

    if not enabled:
        raise Exception(
            "Runout calibration is disabled for nozzle tip '%s'"
            % tip.getName()
        )

    return tip, calibration


def park_after_calibration(module, nozzle):
    park_nozzle = getattr(module, "park_nozzle", None)
    machine = getattr(module, "machine", None)

    if park_nozzle is None:
        raise Exception(
            "Pick-head squaring module has no park_nozzle function"
        )

    if machine is None:
        raise Exception(
            "Pick-head squaring module did not expose the machine"
        )

    log(
        "Parking nozzle at squaring-script location "
        "X=%.3f Y=%.3f Z=%.3f"
        % (module.PARK_X, module.PARK_Y, module.PARK_Z)
    )

    park_nozzle(machine, nozzle)

    parked = nozzle.getLocation().convertToUnits(module.PARK_UNITS)
    errors = (
        abs(parked.getX() - module.PARK_X),
        abs(parked.getY() - module.PARK_Y),
        abs(parked.getZ() - module.PARK_Z),
    )

    if max(errors) > PARK_POSITION_TOLERANCE:
        raise Exception(
            "Nozzle did not reach the park location; actual "
            "X=%.3f Y=%.3f Z=%.3f"
            % (parked.getX(), parked.getY(), parked.getZ())
        )

    log(
        "Final park completed at X=%.3f Y=%.3f Z=%.3f A=%.3f"
        % (
            parked.getX(),
            parked.getY(),
            parked.getZ(),
            parked.getRotation(),
        )
    )


SCRIPT_DIR = scripts_root()

SQUARING_PATH = os.path.join(
    SCRIPT_DIR,
    "Events",
    "Homing",
    "pickHead_squaring.py"
)


#
# STEP 1: Physically square the nozzle and establish the global A zero.
#
log("Starting physical nozzle squaring and A-axis zeroing")

squaring = imp.load_source(
    "pickHead_squaring_after_homing",
    SQUARING_PATH
)

nozzle = require_squaring_success(squaring)

log(
    "Physical square/global zero completed for nozzle '%s'"
    % nozzle.getName()
)


#
# STEP 2: Recalibrate runout after the new rotation zero exists.
#
tip, calibration = get_mounted_tip_calibration(nozzle)

log(
    "Starting post-zero runout calibration for nozzle '%s', tip '%s'"
    % (nozzle.getName(), tip.getName())
)

# Machine.AfterHoming fires immediately before OpenPnP sets isHomed=true.
# Therefore use:
#
#   homing=True
#   calibrateCamera=False
#
# This performs normal nozzle-tip runout calibration while allowing it to
# execute during the homing sequence.
calibration.calibrate(nozzle, True, False)

if not calibration.isCalibrated(nozzle):
    raise Exception(
        "Runout calibration returned without a calibrated model "
        "for nozzle '%s'" % nozzle.getName()
    )

log("Post-zero runout calibration completed successfully")


#
# STEP 3: Park only after squaring and runout calibration both succeeded.
#
park_after_calibration(squaring, nozzle)

log("Square, runout calibration, and final park completed successfully")
