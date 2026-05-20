//-------- DEPRICATED --------------
/* Job.Placement.BeforeAssembly.js — pre-move alignment using PART fiducial (super-verbose)
//  * Runs BEFORE moving to the placement point.
//  * - Uses part.getFiducialVisionSettings().getPipeline()
//  * - Moves TOP cam to placement XY (blocking, safe)
//  * - Expects WM "Results" = List, first item RotatedRect
//  * - px→mm via camera calibration, rotate into machine axes
//  * - Updates THIS placement target; never touches board transforms
//  */

// var imports = new JavaImporter(
//   java.lang.Math,
//   org.openpnp.model.Location,
//   org.openpnp.util.MovableUtils,
//   org.opencv.core.RotatedRect,
//   java.lang.Thread
// );
// with (imports) {
//   var WM_KEY = "results";
//   var MAX_NUDGE_MM = 2.0;
//   var APPLY_ROTATION = false;
//   var EXPECTED_ANGLE_DEG = 0.0;

//   function log(s) {
//     print("[Placement.BeforeAsm] " + s);
//   }
//   function abort(msg) {
//     log("ABORT: " + msg);
//     throw new java.lang.RuntimeException(msg);
//   }
//   function ident(o) {
//     try {
//       return o == null
//         ? "null"
//         : o.getClass().getName() +
//             "@" +
//             java.lang.Integer.toHexString(java.lang.System.identityHashCode(o));
//     } catch (e) {
//       return "" + o;
//     }
//   }

//   try {
//     if (
//       typeof placement === "undefined" ||
//       typeof placementLocation === "undefined" ||
//       typeof part === "undefined" ||
//       typeof machine === "undefined"
//     ) {
//       abort(
//         "Missing globals (need placement, placementLocation, part, machine)."
//       );
//     }

//     // banner
//     var pid = typeof part.getId === "function" ? part.getId() : "<no-id>";
//     log("Part id=" + pid + " class=" + part.getClass().getName());

//     // 1) PART fiducial pipeline
//     var settings =
//       typeof part.getFiducialVisionSettings === "function"
//         ? part.getFiducialVisionSettings()
//         : null;
//     log("part.getFiducialVisionSettings() → " + ident(settings));
//     if (!settings) abort("FiducialVisionSettings is null.");
//     var pipeline = settings.getPipeline ? settings.getPipeline() : null;
//     log("settings.getPipeline() → " + ident(pipeline));
//     if (!pipeline) abort("Part fiducial pipeline is null.");

//     // 2) placement target (placementLocation is already a Location)
//     var pLoc = null;
//     if (placementLocation && typeof placementLocation.getX === "function") {
//       pLoc = placementLocation;
//       log(
//         "placementLocation is Location: (" +
//           pLoc.getX() +
//           "," +
//           pLoc.getY() +
//           "," +
//           pLoc.getZ() +
//           ")"
//       );
//     } else if (typeof placement.getLocation === "function") {
//       pLoc = placement.getLocation();
//       log(
//         "placement.getLocation() used: (" +
//           pLoc.getX() +
//           "," +
//           pLoc.getY() +
//           "," +
//           pLoc.getZ() +
//           ")"
//       );
//     } else {
//       abort("Cannot obtain placement target Location.");
//     }

//     // 3) camera → over placement XY (blocking, safe)
//     var cam = machine.getDefaultHead().getDefaultCamera();
//     if (!cam) abort("No default head camera.");
//     var camLoc = cam.getLocation();
//     var camTarget = new org.openpnp.model.Location(
//       pLoc.getUnits(),
//       pLoc.getX(),
//       pLoc.getY(),
//       camLoc.getZ(),
//       camLoc.getRotation()
//     );
//     org.openpnp.util.MovableUtils.moveToLocationAtSafeZ(cam, camTarget);
//     log(
//       "Camera @ (" +
//         camTarget.getX() +
//         "," +
//         camTarget.getY() +
//         "," +
//         camTarget.getZ() +
//         ")"
//     );

//     // 3.5) small settle delay
//     try {
//       java.lang.Thread.sleep(50); // 50 ms
//       log("Settled for 50 ms before capture.");
//     } catch (e) {
//       log("Sleep(50ms) failed (ignored): " + e);
//     }

//     // 4) capture + run
//     // You *can* keep this capture just for logging if you like:
//     var image = cam.capture(); // java.awt.image.BufferedImage
//     log("Captured image object = " + ident(image));

//     var w = image.getWidth();
//     var h = image.getHeight();
//     log("Captured " + w + "x" + h);

//     // IMPORTANT: CvPipeline.process() has *no* parameters.
//     pipeline.process();
//     log("pipeline.process OK");

//     // 5) results → RotatedRect
//     var wm = pipeline.getWorkingModel();
//     if (!wm) abort("WorkingModel is null.");
//     var results = wm.get(WM_KEY);
//     if (!results) abort('WM["' + WM_KEY + '"] is null.');
//     if (results.isEmpty()) abort('WM["' + WM_KEY + '"] is empty.');
//     var first = results.get(0);
//     log("Results[0] = " + ident(first));
//     if (!(first instanceof org.opencv.core.RotatedRect))
//       abort(
//         "Results[0] not RotatedRect (is " + first.getClass().getName() + ")."
//       );
//     var rr = first;

//     // 6) px from center → mm (camera cal)
//     var px = rr.center.x - w / 2.0;
//     var py = rr.center.y - h / 2.0;
//     log("px,py=" + px.toFixed(3) + "," + py.toFixed(3));

//     var cal = cam.getCalibration();
//     if (!cal) abort("Camera calibration is null.");
//     var upx =
//       typeof cal.getUnitsPerPixelX === "function"
//         ? cal.getUnitsPerPixelX()
//         : cal.getMmPerPixelX();
//     var upy =
//       typeof cal.getUnitsPerPixelY === "function"
//         ? cal.getUnitsPerPixelY()
//         : cal.getMmPerPixelY();
//     if (!upx || !upy) abort("Calibration lacks units-per-pixel.");

//     var dx_cam = px * upx;
//     var dy_cam = -py * upy; // flip Y
//     var rad = (camLoc.getRotation() * Math.PI) / 180.0;
//     var cos = Math.cos(rad),
//       sin = Math.sin(rad);
//     var dx = dx_cam * cos - dy_cam * sin;
//     var dy = dx_cam * sin + dy_cam * cos;
//     log("machine-frame dx,dy=" + dx.toFixed(3) + "," + dy.toFixed(3) + " (mm)");

//     if (Math.abs(dx) > MAX_NUDGE_MM || Math.abs(dy) > MAX_NUDGE_MM) {
//       dx = Math.max(-MAX_NUDGE_MM, Math.min(MAX_NUDGE_MM, dx));
//       dy = Math.max(-MAX_NUDGE_MM, Math.min(MAX_NUDGE_MM, dy));
//       log("clamped to ±" + MAX_NUDGE_MM + " mm");
//     }

//     var corrected = new org.openpnp.model.Location(
//       pLoc.getUnits(),
//       pLoc.getX() + dx,
//       pLoc.getY() + dy,
//       pLoc.getZ(),
//       pLoc.getRotation()
//     );
//     if (APPLY_ROTATION) {
//       var dθ = rr.angle - EXPECTED_ANGLE_DEG;
//       corrected = new org.openpnp.model.Location(
//         corrected.getUnits(),
//         corrected.getX(),
//         corrected.getY(),
//         corrected.getZ(),
//         corrected.getRotation() + dθ
//       );
//       log("rotation tweak dθ=" + dθ.toFixed(2) + "°");
//     }

//     // 7) apply — prefer placement.setLocation(); fallback to placementLocation.setLocation()
//     var applied = false;
//     if (typeof placement.setLocation === "function") {
//       placement.setLocation(corrected);
//       applied = true;
//       log(
//         "Applied via placement.setLocation(). New XY=(" +
//           corrected.getX() +
//           "," +
//           corrected.getY() +
//           ")"
//       );
//     }
//     if (!applied && typeof placementLocation.setLocation === "function") {
//       placementLocation.setLocation(corrected);
//       applied = true;
//       log(
//         "Applied via placementLocation.setLocation(). New XY=(" +
//           corrected.getX() +
//           "," +
//           corrected.getY() +
//           ")"
//       );
//     }
//     if (!applied) abort("No setter available to apply corrected Location.");
//   } catch (err) {
//     print("[Placement.BeforeAsm] ERROR: " + err);
//   }
// }
