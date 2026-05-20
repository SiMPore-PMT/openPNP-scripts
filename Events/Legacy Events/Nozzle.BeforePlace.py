# ----------- DEPRICATED --------------
# This has been moved to job.Placement.Starting.py
#  # Nozzle.BeforePlace.py - pre-place alignment using PART fiducial (Python/Jython)
# # Runs right before a nozzle places a part.
# # - Uses part.getFiducialVisionSettings().getPipeline()
# # - Assumes nozzle is already at nominal placement XY
# # - Moves TOP camera to that XY (blocking, safe)
# # - Expects WM["Results"] = List, first item RotatedRect
# # - Converts px->mm via camera calibration, rotates into machine axes
# # - Nudges the nozzle for THIS placement only; does NOT touch board transforms.

# from java.lang import System, Thread, RuntimeException
# from org.openpnp.model import Location
# from org.openpnp.util import MovableUtils
# from org.opencv.core import RotatedRect
# import math

# WM_KEY = "Results"
# settleTime = 75           # ms for camera settle
# MAX_NUDGE_MM = 2.0        # clamp correction
# APPLY_ROTATION = True     # True = also twist nozzle C by measured angle
# EXPECTED_ANGLE_DEG = 0.0  # expected fiducial angle in degrees

# # Optional: restrict to certain parts only
# # Set to None to run on all parts, or to a list like ["MyPartId"]
# ALLOWED_PART_IDS = None


# def log(msg):
#     print "[Nozzle.BeforePlace] %s" % msg


# def abort(msg):
#     log("ABORT: " + msg)
#     raise RuntimeException(msg)


# def ident(o):
#     try:
#         if o is None:
#             return "null"
#         return "%s@%x" % (o.getClass().getName(), System.identityHashCode(o))
#     except Exception:
#         return str(o)


# try:
#     g = globals()
#     # 0) Required globals
#     if "nozzle" not in g or "part" not in g or "machine" not in g:
#         abort("Missing globals (need nozzle, part, machine).")

#     # If there is no part (e.g. discard or weird state), just bail
#     try:
#         if part is None:
#             log("No part associated with this place; skipping script.")
#             raise SystemExit
#     except Exception:
#         pass

#     # Optional: filter by part id
#     try:
#         pid = part.getId()
#     except Exception:
#         pid = "<no-id>"

#     if ALLOWED_PART_IDS is not None and pid not in ALLOWED_PART_IDS:
#         log("Skipping part id=%r (not in ALLOWED_PART_IDS)." % pid)
#         raise SystemExit

#     log("Part id=%s class=%s" % (pid, part.getClass().getName()))

#     # 1) PART fiducial pipeline
#     try:
#         settings = part.getFiducialVisionSettings()
#     except Exception:
#         settings = None
#     log("part.getFiducialVisionSettings() -> %s" % ident(settings))
#     if settings is None:
#         abort("FiducialVisionSettings is null.")

#     try:
#         pipeline = settings.getPipeline()
#     except Exception:
#         pipeline = None
#     log("settings.getPipeline() -> %s" % ident(pipeline))
#     if pipeline is None:
#         abort("Part fiducial pipeline is null.")

#     # 2) Nozzle target location
#     try:
#         nLoc = nozzle.getLocation()
#     except Exception:
#         nLoc = None

#     if nLoc is None:
#         abort("Cannot obtain nozzle Location.")

#     log("Nozzle Location before correction: (X=%.3f, Y=%.3f, Z=%.3f, R=%.3f)" %
#         (nLoc.getX(), nLoc.getY(), nLoc.getZ(), nLoc.getRotation()))

#     # 3) Move TOP camera over same XY
#     cam = machine.getDefaultHead().getDefaultCamera()
#     if cam is None:
#         abort("No default head camera.")

#     camLoc = cam.getLocation()
#     camTarget = Location(
#         nLoc.getUnits(),
#         nLoc.getX(),
#         nLoc.getY(),
#         camLoc.getZ(),
#         camLoc.getRotation()
#     )

#     MovableUtils.moveToLocationAtSafeZ(cam, camTarget)
#     log("Camera @ (X=%.3f, Y=%.3f, Z=%.3f, R=%.3f)" %
#         (camTarget.getX(), camTarget.getY(), camTarget.getZ(), camTarget.getRotation()))

#     # 3.5) Settle
#     try:
#         Thread.sleep(settleTime)
#         log("Settled for %d ms before capture." % settleTime)
#     except Exception, e:
#         log("Sleep(%d ms) failed (ignored): %s" % (settleTime, e))

#     # 4) Capture and run pipeline
#     image = cam.capture()  # java.awt.image.BufferedImage
#     log("Captured image object = %s" % ident(image))

#     w = image.getWidth()
#     h = image.getHeight()
#     log("Captured %dx%d" % (w, h))

#     pipeline.process(image)
#     log("pipeline.process OK")

#     # 5) Results -> RotatedRect
#     wm = pipeline.getWorkingModel()
#     if wm is None:
#         abort("WorkingModel is null.")

#     results = wm.get(WM_KEY)
#     if results is None:
#         abort('WM["%s"] is null.' % WM_KEY)
#     if results.isEmpty():
#         abort('WM["%s"] is empty.' % WM_KEY)

#     first = results.get(0)
#     log("Results[0] = %s" % ident(first))

#     if not isinstance(first, RotatedRect):
#         abort("Results[0] not RotatedRect (is %s)." %
#               first.getClass().getName())
#     rr = first

#     # 6) Pixels from center -> mm (camera calibration)
#     px = rr.center.x - w / 2.0
#     py = rr.center.y - h / 2.0
#     log("px,py=%.3f,%.3f" % (px, py))

#     cal = cam.getCalibration()
#     if cal is None:
#         abort("Camera calibration is null.")

#     # Newer calibration has getUnitsPerPixelX/Y, older has getMmPerPixelX/Y
#     try:
#         upx = cal.getUnitsPerPixelX()
#         upy = cal.getUnitsPerPixelY()
#     except Exception:
#         upx = cal.getMmPerPixelX()
#         upy = cal.getMmPerPixelY()

#     if upx is None or upy is None:
#         abort("Calibration lacks units-per-pixel.")

#     dx_cam = px * upx
#     dy_cam = -py * upy  # flip Y

#     rad = camLoc.getRotation() * math.pi / 180.0
#     c = math.cos(rad)
#     s = math.sin(rad)

#     dx = dx_cam * c - dy_cam * s
#     dy = dx_cam * s + dy_cam * c
#     log("machine-frame dx,dy=%.3f,%.3f (mm)" % (dx, dy))

#     # Clamp
#     if abs(dx) > MAX_NUDGE_MM or abs(dy) > MAX_NUDGE_MM:
#         dx = max(-MAX_NUDGE_MM, min(MAX_NUDGE_MM, dx))
#         dy = max(-MAX_NUDGE_MM, min(MAX_NUDGE_MM, dy))
#         log("clamped to +/-%.3f mm" % MAX_NUDGE_MM)

#     # 7) Build corrected nozzle location
#     corrected_rot = nLoc.getRotation()
#     if APPLY_ROTATION:
#         dtheta = rr.angle - EXPECTED_ANGLE_DEG
#         corrected_rot = corrected_rot + dtheta
#         log("rotation tweak dtheta=%.2f deg -> new R=%.2f deg" %
#             (dtheta, corrected_rot))

#     corrected = Location(
#         nLoc.getUnits(),
#         nLoc.getX() + dx,
#         nLoc.getY() + dy,
#         nLoc.getZ(),  # keep current Z; job processor will handle final Z
#         corrected_rot
#     )

#     # 8) Apply: nudge nozzle for this placement only
#     MovableUtils.moveToLocationAtSafeZ(nozzle, corrected)
#     log("Nozzle nudged to (X=%.3f, Y=%.3f, Z=%.3f, R=%.3f)" %
#         (corrected.getX(), corrected.getY(), corrected.getZ(), corrected.getRotation()))

# except SystemExit:
#     # normal early exit (filtering etc.)
#     pass
# except Exception, err:
#     print "[Nozzle.BeforePlace] ERROR: %s" % err
