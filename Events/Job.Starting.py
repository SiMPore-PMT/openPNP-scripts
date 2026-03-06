# Job.Placement.Starting.py
#
# Dispensing script using board-level fiducial(s) to correct boardLocation,
# then walking the footprint pads of the dispense part and moving a
# "Dispense Head" around to each pad.
#
# - Event: Job.Placement.Starting (or Job.Starting if you bound it there)
# - Uses BOARD fiducial placement + its vision pipeline
# - If fiducial vision fails after N attempts: disables that BoardLocation
#   and SKIPS dispensing for that board.
# - Dispense is done by moving a HeadMountable named "Dispense Head"
#   (rotation of that head is used to "pump" adhesive).
## Job.Placement.Starting.py
#
# Dispensing script using board-level fiducial(s) to correct boardLocation,
# then walking the footprint pads of the dispense part and moving a
# "Dispense Head" around to each pad.
#
# - Event: Job.Placement.Starting (or Job.Starting if you bound it there)
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



