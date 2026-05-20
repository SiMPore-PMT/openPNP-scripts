# ----------- ACTIVE SCRIPT --------------
# Handles correcting the offset rotation and position of the current plastic manifold (filter disk body/ mwi body),
# to be assembled. That is, the nozzle already has picked up a part, and is looking to place it.
# This script replaces the old Nozzle.BeforePlace.py
#
# Note:
#    - runs at the start of each placement/board-location (invoked per placement by OpenPnP).
#
# ########################Below below - working code with retrys the OG---- This here is a  new grab from fiducials##########################
# Job.Placement.Starting.py
#
# Use a board-level fiducial *placement* to correct boardLocation (board origin)
# BEFORE OpenPnP computes placementLocation / plans moves.
#
# - Event: Job.Placement.Starting
# - Uses a BOARD FIDUCIAL PLACEMENT only:
#       * Placement from boardLocation.getBoard().getPlacements()/getBoardPlacements()
#         whose type/flag explicitly indicates "Fiducial".
#       * Location from that placement.getLocation()
#       * Vision settings / pipeline from that placement (or its Part).
# - NO FALLBACK to the main part placement, even if that part has fiducialVisionSettings.
# - Does NOT change placement.getLocation() (design remains unchanged).
# - Does NOT move the nozzle; only nudges boardLocation in machine coords.
# - Tries up to MAX_ATTEMPTS vision attempts. If it fails, the placement is
#   disabled and skipped.

from org.openpnp.model import Location
from org.openpnp.util import MovableUtils, Utils2D
from org.opencv.core import RotatedRect
from java.lang import Math, Thread

# ----------------- CONFIG -----------------
MAX_NUDGE_MM           = 4.0      # max +/- tweak per axis
APPLY_ROTATION         = True     # set True if you want to also tweak board rotation
EXPECTED_ANGLE_DEG     = 0.0      # expected fiducial angle in the image
DESIGN_BOARD_ROT_DEG   = 0.0      # board design rotation (what you WANT it to be)

PRE_CAPTURE_SETTLE_MS  = 250      # settle before first pipeline.process()
RESULT_STAGE_KEY       = "results"

# Retry behavior for vision
MAX_ATTEMPTS           = 3        # number of tries to get a RotatedRect
RETRY_DELAY_MS         = 250      # delay between attempts (after the first)

# Image-to-machine sign controls
FLIP_IMAGE_X = False
FLIP_IMAGE_Y = True

TAG = "Job.Placement.Starting"


# ------------- LOG HELPERS -------------
def log(msg):
    # Keep logs ASCII-safe to avoid codec errors
    print "[%s] %s" % (TAG, str(msg))


# ------------- HELPER: run pipeline and get RotatedRect with retries -------------
def find_rotated_rect_with_retries(pipeline):
    """
    Try up to MAX_ATTEMPTS times to:
      - pipeline.process()
      - extract a RotatedRect from its results
    Returns:
      RotatedRect or None
    """
    candidate_keys = [RESULT_STAGE_KEY, "results", "preResults"]

    for attempt in range(1, MAX_ATTEMPTS + 1):
        try:
            log("Vision attempt %d of %d" % (attempt, MAX_ATTEMPTS))

            pipeline.process()
            log("pipeline.process() OK on attempt %d" % attempt)

            rr = None
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

                # model IS a RotatedRect
                if isinstance(model, RotatedRect):
                    rr = model
                    log("Got RotatedRect directly from results key '%s' on attempt %d." %
                        (key, attempt))
                    break

                # model is a java.util.List<RotatedRect>
                if hasattr(model, "size") and hasattr(model, "get"):
                    if model.size() > 0:
                        first = model.get(0)
                        if isinstance(first, RotatedRect) or hasattr(first, "center"):
                            rr = first
                            log("Got RotatedRect[0] from list at results key '%s' on attempt %d." %
                                (key, attempt))
                            break

            if rr is not None:
                return rr

            log("No RotatedRect found on attempt %d." % attempt)

        except Exception, e:
            log("Exception during pipeline attempt %d: %s" % (attempt, e))

        # If not last attempt, wait a bit and try again
        if attempt < MAX_ATTEMPTS:
            log("Retrying in %d ms..." % RETRY_DELAY_MS)
            Thread.sleep(RETRY_DELAY_MS)

    # All attempts failed
    return None


# ------------- HELPER: get board fiducial placement + settings -------------
def get_board_fiducial_placement_and_settings(board):
    """
    Locate a fiducial *placement* on the board and its fiducial vision settings.

    Rules:
      - Only placements explicitly marked as fiducial are considered:
          * placement.isFiducial() == true, OR
          * "FIDUCIAL" in str(placement.getType()).upper()
      - We do NOT treat "any placement whose part has fiducialVisionSettings" as a fiducial.
      - Once a fiducial placement is chosen, its pipeline may still come from:
          * placement.getFiducialVisionSettings(), or
          * placement.getVisionSettings(), or
          * placement.getPart().getFiducialVisionSettings().
    """
    placements = None

    # Try getPlacements()
    if hasattr(board, "getPlacements"):
        try:
            placements = board.getPlacements()
            log("Using board.getPlacements().")
        except:
            placements = None

    # Fallback: getBoardPlacements()
    if placements is None and hasattr(board, "getBoardPlacements"):
        try:
            placements = board.getBoardPlacements()
            log("Using board.getBoardPlacements().")
        except:
            placements = None

    if placements is None:
        raise Exception("Board placements list is null; configure at least one placement on the board.")

    # Count placements
    count = 0
    if hasattr(placements, "size"):
        count = placements.size()
    else:
        try:
            for _ in placements:
                count += 1
        except TypeError:
            count = 0

    log("Board placements count: %d" % count)
    if count == 0:
        raise Exception("Board has no placements; this script requires at least one fiducial placement.")

    # Helper: determine if placement is explicitly a fiducial placement
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
        # IMPORTANT: do NOT infer from part's fiducialVisionSettings here.
        return False

    # Helper: fetch vision settings for a given (already-identified) fiducial placement
    def get_vision_settings_for_placement(plc):
        # Try placement-level settings first
        try:
            if hasattr(plc, "getFiducialVisionSettings"):
                vs = plc.getFiducialVisionSettings()
                if vs is not None:
                    return vs
        except:
            pass

        try:
            if hasattr(plc, "getVisionSettings"):
                vs = plc.getVisionSettings()
                if vs is not None:
                    return vs
        except:
            pass

        # Then part-level settings as a fallback for *fiducial* placements only
        try:
            if hasattr(plc, "getPart"):
                p = plc.getPart()
                if p is not None and hasattr(p, "getFiducialVisionSettings"):
                    vs = p.getFiducialVisionSettings()
                    if vs is not None:
                        return vs
        except:
            pass

        return None

    # Iterate placements and find the first good fiducial candidate
    chosen_plc = None
    chosen_loc = None
    chosen_vs  = None

    if hasattr(placements, "size"):
        # Java List style
        for i in range(placements.size()):
            plc = placements.get(i)
            # Skip disabled placements if that exists
            try:
                if hasattr(plc, "isEnabled") and not plc.isEnabled():
                    log("Skipping placement index %d because it is disabled." % i)
                    continue
            except:
                pass

            if not is_fiducial_placement(plc):
                continue

            vs = get_vision_settings_for_placement(plc)
            if vs is None:
                log("Fiducial placement index %d has no vision settings; skipping." % i)
                continue

            try:
                loc = plc.getLocation()
            except:
                log("Fiducial placement index %d has no location; skipping." % i)
                continue

            chosen_plc = plc
            chosen_loc = loc
            chosen_vs  = vs
            log("Selected fiducial placement at index %d." % i)
            break
    else:
        # Python-iterable style
        idx = 0
        try:
            for plc in placements:
                try:
                    if hasattr(plc, "isEnabled") and not plc.isEnabled():
                        log("Skipping placement index %d because it is disabled." % idx)
                        idx += 1
                        continue
                except:
                    pass

                if not is_fiducial_placement(plc):
                    idx += 1
                    continue

                vs = get_vision_settings_for_placement(plc)
                if vs is None:
                    log("Fiducial placement index %d has no vision settings; skipping." % idx)
                    idx += 1
                    continue

                try:
                    loc = plc.getLocation()
                except:
                    log("Fiducial placement index %d has no location; skipping." % idx)
                    idx += 1
                    continue

                chosen_plc = plc
                chosen_loc = loc
                chosen_vs  = vs
                log("Selected fiducial placement at index %d." % idx)
                break
        except TypeError:
            pass

    if chosen_plc is None or chosen_vs is None or chosen_loc is None:
        raise Exception("No enabled board fiducial placement with vision settings found on the board.")

    # Label for logs
    label = "<unnamed>"
    try:
        if hasattr(chosen_plc, "getName") and chosen_plc.getName() is not None:
            label = chosen_plc.getName()
        elif hasattr(chosen_plc, "getId") and chosen_plc.getId() is not None:
            label = chosen_plc.getId()
    except:
        pass

    return chosen_plc, chosen_loc, chosen_vs, label


# ------------- MAIN BODY -------------
try:
    # ---- Check we have the basics for this event ----
    required_globals = ("placement", "boardLocation", "machine")
    missing = [name for name in required_globals if name not in globals()]
    if missing:
        log("ERROR: Missing globals: %s" % ", ".join(missing))
        raise Exception("Missing globals: %s" % ", ".join(missing))

    pl  = placement
    bl  = boardLocation
    mac = machine

    # Part only used for logging
    prt = None
    if "part" in globals():
        prt = part
    pid = prt.getId() if (prt is not None and hasattr(prt, "getId")) else "<no-part-id>"
    log("Part id=%s" % pid)

    # ---- Get the BOARD from the boardLocation (not from placement) ----
    board = None
    if hasattr(bl, "getBoard"):
        try:
            board = bl.getBoard()
        except:
            board = None

    if board is None:
        raise Exception("boardLocation.getBoard() returned null; board-level fiducial placement is required.")

    log("Board object: %s" % board)

    # ---- Locate board fiducial placement + its vision settings ----
    fidPlacement, fidLoc, settings, fidLabel = get_board_fiducial_placement_and_settings(board)

    log("Using BOARD fiducial placement '%s' design location: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (fidLabel, fidLoc.getX(), fidLoc.getY(), fidLoc.getZ(), fidLoc.getRotation()))

    if settings is None:
        raise Exception("Board fiducial placement '%s' has no vision settings; configure a fiducial pipeline." %
                        fidLabel)

    log("FiducialVisionSettings source=board fiducial placement -> %s" % settings)

    pipeline = settings.getPipeline()
    if pipeline is None:
        raise Exception("Board fiducial placement '%s' vision settings pipeline is null." % fidLabel)

    log("Pipeline=%s" % pipeline)

    # ---- DESIGN fiducial location (board-relative) ----
    dLoc = fidLoc  # board-relative design location of fiducial

    # ---- BOARD origin (machine coords) ----
    blLoc = bl.getLocation()
    log("boardLocation BEFORE: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (blLoc.getX(), blLoc.getY(), blLoc.getZ(), blLoc.getRotation()))

    # ---- Compute nominal machine location of the fiducial ourselves ----
    pLoc = Utils2D.calculateBoardPlacementLocation(bl, dLoc)
    log("Nominal fiducial machine location (computed): (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (pLoc.getX(), pLoc.getY(), pLoc.getZ(), pLoc.getRotation()))

    # ---- Move TOP camera over nominal fiducial XY ----
    head = mac.getDefaultHead()
    cam  = head.getDefaultCamera()
    if cam is None:
        raise Exception("Default head camera is null.")

    camLoc = cam.getLocation()
    camTarget = Location(
        pLoc.getUnits(),
        pLoc.getX(),        # same XY as nominal fiducial
        pLoc.getY(),
        camLoc.getZ(),      # camera's calibrated Z
        camLoc.getRotation()
    )

    MovableUtils.moveToLocationAtSafeZ(cam, camTarget)
    log("Camera @ (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (camTarget.getX(), camTarget.getY(), camTarget.getZ(), camTarget.getRotation()))

    # ---- Settling before pipeline ----
    if PRE_CAPTURE_SETTLE_MS > 0:
        Thread.sleep(PRE_CAPTURE_SETTLE_MS)
        log("Settled %d ms before pipeline.process()." % PRE_CAPTURE_SETTLE_MS)

    # ---- Try up to MAX_ATTEMPTS to get a RotatedRect ----
    rr = find_rotated_rect_with_retries(pipeline)
    if rr is None:
        log("Board fiducial NOT FOUND after %d attempts. Disabling placement and skipping correction." %
            MAX_ATTEMPTS)

        # Disable this placement so the job processor will skip it.
        try:
            if hasattr(pl, "setEnabled"):
                pl.setEnabled(False)
                log("placement.setEnabled(False) called.")
        except Exception, e2:
            log("Failed to disable placement: %s" % e2)

        # Optionally tag a comment so you see it in the job
        try:
            existing = pl.getComments() or ""
            msg = "[auto-skip: board fiducial not found %d attempts]" % MAX_ATTEMPTS
            if existing:
                pl.setComments(existing + " " + msg)
            else:
                pl.setComments(msg)
        except:
            pass

        # Nothing else to do for this placement
        raise SystemExit

    # We have a RotatedRect
    log("Measured RotatedRect angle: %.4f deg" % rr.angle)

    # ---- Capture 1 image just for geometry (w,h) ----
    image = cam.capture()
    w = image.getWidth()
    h = image.getHeight()
    log("Captured image for geometry: %dx%d" % (w, h))

    # Pixel offset from image center
    px = rr.center.x - (w / 2.0)
    py = rr.center.y - (h / 2.0)
    log("px,py=%.3f,%.3f" % (px, py))

    # Optional flips to adapt image axes to machine axes
    if FLIP_IMAGE_X:
        px = -px
    if FLIP_IMAGE_Y:
        py = -py

    # Camera-frame mm offset
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

    # Rotate into machine frame using camera rotation
    rad = camLoc.getRotation() * Math.PI / 180.0
    c   = Math.cos(rad)
    s   = Math.sin(rad)

    dx = dx_cam * c - dy_cam * s
    dy = dx_cam * s + dy_cam * c
    log("Machine-frame dx=%.6f, dy=%.6f" % (dx, dy))

    # ---- Clamp tweak so we don't go wild ----
    if abs(dx) > MAX_NUDGE_MM or abs(dy) > MAX_NUDGE_MM:
        if dx >  MAX_NUDGE_MM: dx =  MAX_NUDGE_MM
        if dx < -MAX_NUDGE_MM: dx = -MAX_NUDGE_MM
        if dy >  MAX_NUDGE_MM: dy =  MAX_NUDGE_MM
        if dy < -MAX_NUDGE_MM: dy = -MAX_NUDGE_MM
        log("Clamped dx,dy to +/-%.3f mm" % MAX_NUDGE_MM)

    # ---- For debugging: corrected nominal fiducial target (not written anywhere) ----
    corrected = Location(
        pLoc.getUnits(),
        pLoc.getX() + dx,
        pLoc.getY() + dy,
        pLoc.getZ(),
        pLoc.getRotation()
    )

    # --- Rotation handling ---
    # Absolute board rotation:
    #   newBoardRot = DESIGN_BOARD_ROT_DEG + (EXPECTED_ANGLE_DEG - measuredAngle)
    # so we do NOT accumulate on top of previous boardLocation rotation.
    dtheta      = 0.0
    newBoardRot = blLoc.getRotation()

    if APPLY_ROTATION:
        measuredAngle = rr.angle
        newBoardRot   = DESIGN_BOARD_ROT_DEG + (EXPECTED_ANGLE_DEG - measuredAngle)
        dtheta        = newBoardRot - blLoc.getRotation()

        corrected = Location(
            corrected.getUnits(),
            corrected.getX(),
            corrected.getY(),
            corrected.getZ(),
            newBoardRot  # reflect final board rotation in debug location
        )
        log("Rotation tweak dTheta=%.3f deg (measured=%.3f, designBoard=%.3f, newBoard=%.3f)" %
            (dtheta, measuredAngle, DESIGN_BOARD_ROT_DEG, newBoardRot))

    log("CORRECTED nominal fid target (debug): (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
        (corrected.getX(), corrected.getY(), corrected.getZ(), corrected.getRotation()))

    # ---- APPLY: update boardLocation ----
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

except SystemExit:
    # Clean early exit used when we decided to skip placement.
    pass

except Exception, e:
    print "[%s ERROR] %s" % (TAG, e)

# 
# # Job.Placement.Starting.py
# #
#WORKING COMMENTED OUT CAUSE OF DISPENSE. IT USES THE SAME CORRECTION METHOD SO THIS IS REDUNDANT ATM
# # Use the part fiducial pipeline to correct boardLocation (board origin)
# # BEFORE OpenPnP computes placementLocation / plans moves.
# #
# # - Event: Job.Placement.Starting
# # - Does NOT change placement.getLocation() (design remains unchanged).
# # - Does NOT move the nozzle; only nudges boardLocation in machine coords.
# # - NEW: Tries up to MAX_ATTEMPTS to find a RotatedRect. If it fails,
# #        it disables the placement and returns (skip this chip).

# from org.openpnp.model import Location
# from org.openpnp.util import MovableUtils, Utils2D
# from org.opencv.core import RotatedRect
# from java.lang import Math, Thread

# # ----------------- CONFIG -----------------
# MAX_NUDGE_MM           = 4.0      # max +/- tweak per axis
# APPLY_ROTATION         = True     # set True if you want to also tweak board rotation
# EXPECTED_ANGLE_DEG     = 0.0      # expected fiducial/part angle in the image
# DESIGN_BOARD_ROT_DEG   = 0.0      # board design rotation (what you WANT it to be)

# PRE_CAPTURE_SETTLE_MS  = 250     # settle before first pipeline.process()
# RESULT_STAGE_KEY       = "results"

# # Retry behavior for vision
# MAX_ATTEMPTS           = 3        # number of tries to get a RotatedRect
# RETRY_DELAY_MS         = 750      # delay between attempts (after the first)

# # Image-to-machine sign controls
# # Try FLIP_IMAGE_Y = True first (standard OpenCV -> machine mapping).
# # If that feels inverted on your machine, change it to False and test.
# FLIP_IMAGE_X = False
# FLIP_IMAGE_Y = True

# TAG = "Job.Placement.Starting"


# # ------------- LOG HELPERS -------------
# def log(msg):
#     # Keep logs ASCII-safe to avoid codec errors
#     print "[%s] %s" % (TAG, str(msg))


# # ------------- HELPER: run pipeline and get RotatedRect with retries -------------
# def find_rotated_rect_with_retries(pipeline):
#     """
#     Try up to MAX_ATTEMPTS times to:
#       - pipeline.process()
#       - extract a RotatedRect from its results
#     Returns:
#       RotatedRect or None
#     """
#     candidate_keys = [RESULT_STAGE_KEY, "results", "preResults"]

#     for attempt in range(1, MAX_ATTEMPTS + 1):
#         try:
#             log("Vision attempt %d of %d" % (attempt, MAX_ATTEMPTS))

#             pipeline.process()
#             log("pipeline.process() OK on attempt %d" % attempt)

#             rr = None
#             for key in candidate_keys:
#                 try:
#                     res = pipeline.getResult(key)
#                 except:
#                     res = None

#                 if res is None:
#                     continue

#                 model = res.getModel()
#                 if model is None:
#                     continue

#                 # model IS a RotatedRect
#                 if isinstance(model, RotatedRect):
#                     rr = model
#                     log("Got RotatedRect directly from results key '%s' on attempt %d." %
#                         (key, attempt))
#                     break

#                 # model is a java.util.List<RotatedRect>
#                 if hasattr(model, "size") and hasattr(model, "get"):
#                     if model.size() > 0:
#                         first = model.get(0)
#                         if isinstance(first, RotatedRect) or hasattr(first, "center"):
#                             rr = first
#                             log("Got RotatedRect[0] from list at results key '%s' on attempt %d." %
#                                 (key, attempt))
#                             break

#             if rr is not None:
#                 return rr

#             log("No RotatedRect found on attempt %d." % attempt)

#         except Exception, e:
#             log("Exception during pipeline attempt %d: %s" % (attempt, e))

#         # If not last attempt, wait a bit and try again
#         if attempt < MAX_ATTEMPTS:
#             log("Retrying in %d ms..." % RETRY_DELAY_MS)
#             Thread.sleep(RETRY_DELAY_MS)

#     # All attempts failed
#     return None


# # ------------- MAIN BODY -------------
# try:
#     # ---- Check we have the basics for this event ----
#     required_globals = ("placement", "part", "boardLocation", "machine")
#     missing = [name for name in required_globals if name not in globals()]
#     if missing:
#         log("ERROR: Missing globals: %s" % ", ".join(missing))
#         raise Exception("Missing globals: %s" % ", ".join(missing))

#     pl  = placement
#     bl  = boardLocation
#     prt = part
#     mac = machine

#     pid = prt.getId() if hasattr(prt, "getId") else "<no-id>"
#     log("Part id=%s" % pid)

#     # ---- Get part fiducial vision settings + pipeline ----
#     settings = prt.getFiducialVisionSettings()
#     if settings is None:
#         raise Exception("part.getFiducialVisionSettings() returned null.")
#     log("FiducialVisionSettings=%s" % settings)

#     pipeline = settings.getPipeline()
#     if pipeline is None:
#         raise Exception("settings.getPipeline() returned null.")
#     log("Pipeline=%s" % pipeline)

#     # ---- DESIGN placement (board-relative) ----
#     dLoc = pl.getLocation()
#     log("placement.getLocation (design): (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (dLoc.getX(), dLoc.getY(), dLoc.getZ(), dLoc.getRotation()))

#     # ---- BOARD origin (machine coords) ----
#     blLoc = bl.getLocation()
#     log("boardLocation BEFORE: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (blLoc.getX(), blLoc.getY(), blLoc.getZ(), blLoc.getRotation()))

#     # ---- Compute nominal machine placementLocation ourselves ----
#     # Proper call: BoardLocation + board-relative Location
#     pLoc = Utils2D.calculateBoardPlacementLocation(bl, dLoc)
#     log("Nominal placementLocation (computed): (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (pLoc.getX(), pLoc.getY(), pLoc.getZ(), pLoc.getRotation()))

#     # ---- Move TOP camera over nominal placement XY ----
#     head = mac.getDefaultHead()
#     cam  = head.getDefaultCamera()
#     if cam is None:
#         raise Exception("Default head camera is null.")

#     camLoc = cam.getLocation()
#     camTarget = Location(
#         pLoc.getUnits(),
#         pLoc.getX(),        # same XY as nominal placement
#         pLoc.getY(),
#         camLoc.getZ(),      # camera's calibrated Z
#         camLoc.getRotation()
#     )

#     MovableUtils.moveToLocationAtSafeZ(cam, camTarget)
#     log("Camera @ (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (camTarget.getX(), camTarget.getY(), camTarget.getZ(), camTarget.getRotation()))

#     # ---- Settling before pipeline ----
#     if PRE_CAPTURE_SETTLE_MS > 0:
#         Thread.sleep(PRE_CAPTURE_SETTLE_MS)
#         log("Settled %d ms before pipeline.process()." % PRE_CAPTURE_SETTLE_MS)

#     # ---- Try up to MAX_ATTEMPTS to get a RotatedRect ----
#     rr = find_rotated_rect_with_retries(pipeline)
#     if rr is None:
#         log("Fiducial NOT FOUND after %d attempts. Disabling placement and skipping correction." %
#             MAX_ATTEMPTS)

#         # Disable this placement so the job processor will skip it.
#         try:
#             if hasattr(pl, "setEnabled"):
#                 pl.setEnabled(False)
#                 log("placement.setEnabled(False) called.")
#         except Exception, e2:
#             log("Failed to disable placement: %s" % e2)

#         # Optionally tag a comment so you see it in the job
#         try:
#             existing = pl.getComments() or ""
#             msg = "[auto-skip: fiducial not found %d attempts]" % MAX_ATTEMPTS
#             if existing:
#                 pl.setComments(existing + " " + msg)
#             else:
#                 pl.setComments(msg)
#         except:
#             pass

#         # Nothing else to do for this placement
#         raise SystemExit

#     # We have a RotatedRect
#     log("Measured RotatedRect angle: %.4f deg" % rr.angle)

#     # ---- Capture 1 image just for geometry (w,h) ----
#     image = cam.capture()
#     w = image.getWidth()
#     h = image.getHeight()
#     log("Captured image for geometry: %dx%d" % (w, h))

#     # Pixel offset from image center
#     px = rr.center.x - (w / 2.0)
#     py = rr.center.y - (h / 2.0)
#     log("px,py=%.3f,%.3f" % (px, py))

#     # Optional flips to adapt image axes to machine axes
#     if FLIP_IMAGE_X:
#         px = -px
#     if FLIP_IMAGE_Y:
#         py = -py

#     # Camera-frame mm offset
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

#     # Rotate into machine frame using camera rotation
#     rad = camLoc.getRotation() * Math.PI / 180.0
#     c   = Math.cos(rad)
#     s   = Math.sin(rad)

#     dx = dx_cam * c - dy_cam * s
#     dy = dx_cam * s + dy_cam * c
#     log("Machine-frame dx=%.6f, dy=%.6f" % (dx, dy))

#     # ---- Clamp tweak so we don't go wild ----
#     if abs(dx) > MAX_NUDGE_MM or abs(dy) > MAX_NUDGE_MM:
#         if dx >  MAX_NUDGE_MM: dx =  MAX_NUDGE_MM
#         if dx < -MAX_NUDGE_MM: dx = -MAX_NUDGE_MM
#         if dy >  MAX_NUDGE_MM: dy =  MAX_NUDGE_MM
#         if dy < -MAX_NUDGE_MM: dy = -MAX_NUDGE_MM
#         log("Clamped dx,dy to +/-%.3f mm" % MAX_NUDGE_MM)

#     # ---- For debugging: corrected nominal placement target (not written anywhere) ----
#     corrected = Location(
#         pLoc.getUnits(),
#         pLoc.getX() + dx,
#         pLoc.getY() + dy,
#         pLoc.getZ(),
#         pLoc.getRotation()
#     )

#     # --- Rotation handling ---
#     # Absolute board rotation:
#     #   newBoardRot = DESIGN_BOARD_ROT_DEG + (EXPECTED_ANGLE_DEG - measuredAngle)
#     # so we do NOT accumulate on top of previous boardLocation rotation.
#     dtheta      = 0.0
#     newBoardRot = blLoc.getRotation()

#     if APPLY_ROTATION:
#         measuredAngle = rr.angle
#         newBoardRot   = DESIGN_BOARD_ROT_DEG + (EXPECTED_ANGLE_DEG - measuredAngle)
#         dtheta        = newBoardRot - blLoc.getRotation()

#         corrected = Location(
#             corrected.getUnits(),
#             corrected.getX(),
#             corrected.getY(),
#             corrected.getZ(),
#             newBoardRot  # reflect final board rotation in debug location
#         )
#         log("Rotation tweak dTheta=%.3f deg (measured=%.3f, designBoard=%.3f, newBoard=%.3f)" %
#             (dtheta, measuredAngle, DESIGN_BOARD_ROT_DEG, newBoardRot))

#     log("CORRECTED nominal target (debug): (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (corrected.getX(), corrected.getY(), corrected.getZ(), corrected.getRotation()))

#     # ---- APPLY: update boardLocation ----
#     # Translation: add dx,dy relative to current board origin.
#     # Rotation: set ABSOLUTE board rotation to newBoardRot.
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

# except SystemExit:
#     # Clean early exit used when we decided to skip placement.
#     pass

# except Exception, e:
#     print "[%s ERROR] %s" % (TAG, e)