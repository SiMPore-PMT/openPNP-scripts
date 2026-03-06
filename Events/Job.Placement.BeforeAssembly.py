# # Job.Placement.BeforeAssembly.py
# # This somewhat functioned last time i used it, but was transfered over to Placement.Starting.py as the discovered board position would not update until the next placement, not the current

# # Pre-placement tweak using part fiducial pipeline.
# # - Runs BEFORE moving nozzle to place.
# # - Uses part.getFiducialVisionSettings().getPipeline()
# # - Moves TOP camera over placementLocation XY
# # - Runs the pipeline
# # - Expects stage result that produces RotatedRect or List<RotatedRect>
# # - Converts px->mm using camera calibration, rotates into machine frame
# # - Updates placementLocation ONLY (design placement.getLocation() stays unchanged)

# from org.openpnp.model import Location
# from org.openpnp.util import MovableUtils
# from org.opencv.core import RotatedRect
# from java.lang import Math, Thread

# # ----------------- CONFIG -----------------
# MAX_NUDGE_MM       = 2.0
# APPLY_ROTATION     = False
# EXPECTED_ANGLE_DEG = 0.0

# # This is the WM key / stage name rectAsList usually writes into.
# # Your logs show things like "rectAsList: using source stage '8'",
# # but the *results* are typically stored under "results" (or sometimes "preResults").
# RESULT_STAGE_KEY = "results"

# # ------------- LOG HELPERS -------------
# def log(msg):
#     print "[Placement.BeforeAssembly] %s" % msg

# # ------------- MAIN BODY -------------
# try:
#     # ---- Sanity: make sure OpenPnP globals exist ----
#     if ('placement' not in globals() or
#         'placementLocation' not in globals() or
#         'part' not in globals() or
#         'machine' not in globals()):
#         raise Exception("Missing globals (need placement, placementLocation, part, machine).")

#     pid = part.getId() if hasattr(part, "getId") else "<no-id>"
#     log("Part id=%s" % pid)

#     # ---- Get part fiducial vision settings + pipeline ----
#     settings = part.getFiducialVisionSettings()
#     if settings is None:
#         raise Exception("part.getFiducialVisionSettings() returned null.")
#     log("FiducialVisionSettings=%s" % settings)

#     pipeline = settings.getPipeline()
#     if pipeline is None:
#         raise Exception("settings.getPipeline() returned null.")
#     log("Pipeline=%s" % pipeline)

#     # ---- Current machine placement target ----
#     pLoc = placementLocation
#     log("placementLocation BEFORE: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (pLoc.getX(), pLoc.getY(), pLoc.getZ(), pLoc.getRotation()))

#     # DESIGN LOCATION (from job file) — DO NOT CHANGE THIS
#     if hasattr(placement, "getLocation"):
#         dLoc = placement.getLocation()
#         log("placement.getLocation (design, UNCHANGED): (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#             (dLoc.getX(), dLoc.getY(), dLoc.getZ(), dLoc.getRotation()))

#     # ---- Move TOP camera over current placementLocation XY ----
#     head = machine.getDefaultHead()
#     cam  = head.getDefaultCamera()
#     if cam is None:
#         raise Exception("Default head camera is null.")

#     camLoc = cam.getLocation()
#     camTarget = Location(
#         pLoc.getUnits(),
#         pLoc.getX(),      # same XY as placementLocation
#         pLoc.getY(),
#         camLoc.getZ(),    # keep camera's calibrated Z
#         camLoc.getRotation()
#     )

#     MovableUtils.moveToLocationAtSafeZ(cam, camTarget)
#     log("Camera @ (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (camTarget.getX(), camTarget.getY(), camTarget.getZ(), camTarget.getRotation()))

#     # ---- Small settle before pipeline capture ----
#     Thread.sleep(750)
#     log("Settled 750 ms before pipeline.process().")

#     # ---- Run pipeline (let its stages deal with capture) ----
#     # NOTE: some builds log "No Camera set on pipeline." from 'image' stage,
#     # but still produce valid results in later stages (rectAsList).
#     pipeline.process()
#     log("pipeline.process() OK")

#     # ---- Pull RotatedRect from pipeline results ----
#     rr = None

#     # Try a few possible keys; your logs mentioned 'results', 'preResults', and a stage '8'
#     candidate_keys = [RESULT_STAGE_KEY, "results"]

#     for key in candidate_keys:
#         try:
#             res = pipeline.getResult(key)
#         except:
#             res = None

#         if res is None:
#             continue

#         model = res.getModel()
#         if model is None:
#             continue

#         # Case 1: model IS a RotatedRect
#         if isinstance(model, RotatedRect):
#             rr = model
#             log("Got RotatedRect directly from results key '%s'." % key)
#             break

#         # Case 2: model is a java.util.List / ArrayList of RotatedRect
#         if hasattr(model, "size") and hasattr(model, "get"):
#             if model.size() > 0:
#                 first = model.get(0)
#                 # Loose check: RotatedRect or something with .center
#                 if isinstance(first, RotatedRect) or hasattr(first, "center"):
#                     rr = first
#                     log("Got RotatedRect[0] from list at results key '%s'." % key)
#                     break

#     if rr is None:
#         raise Exception("Result is not a RotatedRect (checked keys: %s)" % candidate_keys)

#     # ---- We still need image width/height to define the center ----
#     # Don't feed this into pipeline (that caused the process(image) error).
#     image = cam.capture()
#     w = image.getWidth()
#     h = image.getHeight()
#     log("Captured image for geometry: %dx%d" % (w, h))

#     px = rr.center.x - (w / 2.0)
#     py = rr.center.y - (h / 2.0)
#     log("px,py=%.3f,%.3f" % (px, py))

#         # ---- Units per pixel from camera (no 3D Z arg) ----
#     # Try 3D-aware getUnitsPerPixelAtZ() first; if not present, fall back.
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
#     dy_cam = -py * upy   # flip Y so +Y is machine-up
#     rad    = camLoc.getRotation() * Math.PI / 180.0
#     c      = Math.cos(rad)
#     s      = Math.sin(rad)

#     dx = dx_cam * c - dy_cam * s
#     dy = dx_cam * s + dy_cam * c

#     log("Camera-frame dx_cam=%.6f, dy_cam=%.6f" % (dx_cam, dy_cam))
#     log("Machine-frame dx=%.6f, dy=%.6f" % (dx, dy))

#     # ---- Clamp the tweak so we don't go wild ----
#     if abs(dx) > MAX_NUDGE_MM or abs(dy) > MAX_NUDGE_MM:
#         if dx >  MAX_NUDGE_MM: dx =  MAX_NUDGE_MM
#         if dx < -MAX_NUDGE_MM: dx = -MAX_NUDGE_MM
#         if dy >  MAX_NUDGE_MM: dy =  MAX_NUDGE_MM
#         if dy < -MAX_NUDGE_MM: dy = -MAX_NUDGE_MM
#         log("Clamped dx,dy to ±%.3f mm" % MAX_NUDGE_MM)

#     corrected = Location(
#         pLoc.getUnits(),
#         pLoc.getX() + dx,
#         pLoc.getY() + dy,
#         pLoc.getZ(),
#         pLoc.getRotation()
#     )

#     if APPLY_ROTATION:
#         dtheta = rr.angle - EXPECTED_ANGLE_DEG
#         corrected = Location(
#             corrected.getUnits(),
#             corrected.getX(),
#             corrected.getY(),
#             corrected.getZ(),
#             corrected.getRotation() + dtheta
#         )
#         log("Rotation tweak dθ=%.2f°" % dtheta)

#     log("CORRECTED target: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#         (corrected.getX(), corrected.getY(), corrected.getZ(), corrected.getRotation()))

#         # ---- APPLY: adjust the boardLocation, NOT the placement design ----
#     # This moves the board instance on the machine, leaving CAD data alone.
#     if 'boardLocation' in globals() and boardLocation is not None:
#         blLoc = boardLocation.getLocation()
#         blCorrected = Location(
#             blLoc.getUnits(),
#             blLoc.getX() + dx,
#             blLoc.getY() + dy,
#             blLoc.getZ(),
#             blLoc.getRotation()  # or add dθ here if you later enable rotation tweaks
#         )
#         boardLocation.setLocation(blCorrected)
#         log("Updated boardLocation: (X=%.4f, Y=%.4f, Z=%.4f, R=%.4f)" %
#             (blCorrected.getX(), blCorrected.getY(), blCorrected.getZ(), blCorrected.getRotation()))
#     else:
#         log("No boardLocation in globals; cannot apply correction.")

# except Exception, e:
#     print "[Placement.BeforeAssembly ERROR] %s" % e
