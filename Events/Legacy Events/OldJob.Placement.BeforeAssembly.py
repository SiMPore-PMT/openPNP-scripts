# # Job.Placement.BeforeAssembly.py
# #
# # Combined board correction + adhesive dispense flow:
# #   1. Measure board fiducial before dispense.
# #   2. Temporarily apply that corrected board pose and dispense footprint pads.
# #   3. Measure board fiducial after dispense and update boardLocation.
# #   4. Return normally so OpenPnP continues the standard job flow.

# from jarray import array
# from java.lang import Boolean, Class, Double, Math, Object, System, Thread
# from org.openpnp.model import Location
# from org.openpnp.spi.MotionPlanner import CompletionType
# from org.openpnp.util import MovableUtils, Utils2D
# from org.opencv.core import RotatedRect


# # ----------------- VISION CONFIG -----------------
# MAX_NUDGE_MM           = 4.0
# APPLY_ROTATION         = True
# EXPECTED_ANGLE_DEG     = 0.0
# DESIGN_BOARD_ROT_DEG   = 0.0

# PRE_CAPTURE_SETTLE_MS  = 250
# RESULT_STAGE_KEY       = "results"
# MAX_ATTEMPTS           = 3
# RETRY_DELAY_MS         = 250
# VISION_MEASUREMENT_ATTEMPTS = 2
# RECENTER_MAX_PASSES    = 3
# RECENTER_TOLERANCE_MM  = 0.02

# FLIP_IMAGE_X = False
# FLIP_IMAGE_Y = True


# # ----------------- DISPENSE CONFIG -----------------
# DISPENSE_HEAD_NAME      = "Dispense Head"
# DISPENSE_ACTUATOR_NAME  = "GlueDispense"

# DISPENSE_Z_OFFSET_MM    = 0.1
# INTER_PAD_LIFT_MM       = 0.1
# DISPENSE_SAFE_Z_MM      = 25.0

# TRAVEL_SPEED_FACTOR     = 0.74
# DISPENSE_SPEED_FACTOR   = 0.74

# PRE_DISPENSE_DWELL_MS   = 0
# DISPENSE_PULSE_MS       = 15
# POST_DISPENSE_DWELL_MS  = 25
# ACTUATOR_SETTLE_MS      = 25

# PAD_SKIP_NAMES          = set()

# TAG = "Job.Placement.BeforeAssembly"


# def log(msg):
#     print "[%s] %s" % (TAG, str(msg))


# def log_err(msg):
#     print "[%s ERROR] %s" % (TAG, str(msg))


# class VisionMeasurementError(Exception):
#     pass


# SCRIPT_T0_NS = System.nanoTime()
# LAST_T_NS = SCRIPT_T0_NS


# def now_ms():
#     return (System.nanoTime() - SCRIPT_T0_NS) / 1000000.0


# def elapsed_ms_since(t0_ns):
#     return (System.nanoTime() - t0_ns) / 1000000.0


# def log_t(label):
#     global LAST_T_NS
#     now_ns = System.nanoTime()
#     abs_ms = (now_ns - SCRIPT_T0_NS) / 1000000.0
#     delta_ms = (now_ns - LAST_T_NS) / 1000000.0
#     LAST_T_NS = now_ns
#     log("[TIMER] %+9.3f ms | +%8.3f ms | %s" % (abs_ms, delta_ms, label))
#     return now_ns


# def java_list_to_py(model):
#     if model is None:
#         return []
#     if isinstance(model, list):
#         return model
#     if hasattr(model, "size") and hasattr(model, "get"):
#         out = []
#         for i in range(model.size()):
#             out.append(model.get(i))
#         return out
#     try:
#         return list(model)
#     except:
#         return [model]


# def object_name(obj):
#     if obj is None:
#         return ""
#     for method in ["getName", "getId"]:
#         try:
#             value = getattr(obj, method)()
#             if value is not None:
#                 return str(value)
#         except:
#             pass
#     return str(obj)


# def wait_still(movable, label):
#     t0 = log_t("%s: wait_still BEFORE WaitForStillstand" % label)
#     movable.waitForCompletion(CompletionType.WaitForStillstand)
#     log_t("%s: wait_still AFTER WaitForStillstand elapsed=%.3f ms" %
#           (label, elapsed_ms_since(t0)))


# def wait_coord(movable, label):
#     t0 = log_t("%s: wait_coord BEFORE WaitForUnconditionalCoordination" % label)
#     movable.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
#     log_t("%s: wait_coord AFTER WaitForUnconditionalCoordination elapsed=%.3f ms" %
#           (label, elapsed_ms_since(t0)))


# def move_safe_wait(movable, target, label):
#     MovableUtils.moveToLocationAtSafeZ(movable, target)
#     wait_still(movable, label)


# def move_direct(movable, target, speed, label):
#     log_t("%s: move_direct BEFORE queue target=%s speed=%.3f" %
#           (label, str(target), float(speed)))
#     try:
#         movable.moveTo(target, float(speed))
#     except TypeError:
#         movable.moveTo(target)
#     log_t("%s: move_direct AFTER moveTo" % label)


# def move_z_only(movable, units, z, speed, label):
#     target = Location(units, Double.NaN, Double.NaN, z, Double.NaN)
#     log_t("%s: move_z_only BEFORE target=%s speed=%.3f" %
#           (label, str(target), float(speed)))
#     try:
#         movable.moveTo(target, float(speed))
#     except TypeError:
#         movable.moveTo(target)
#     log_t("%s: move_z_only AFTER" % label)


# def set_boolean_actuator_state_fast(actuator, on_off, label):
#     value = Boolean.TRUE if bool(on_off) else Boolean.FALSE
#     method = actuator.getClass().getMethod(
#         "actuate",
#         array([Boolean.TYPE], Class)
#     )
#     method.invoke(actuator, array([value], Object))
#     log("%s boolean actuator=%s queued." % (label, str(bool(on_off))))


# def find_rotated_rect_with_retries(pipeline):
#     candidate_keys = [RESULT_STAGE_KEY, "results", "preResults"]
#     for attempt in range(1, MAX_ATTEMPTS + 1):
#         try:
#             log("Vision attempt %d of %d" % (attempt, MAX_ATTEMPTS))
#             pipeline.process()
#             log("pipeline.process() OK on attempt %d" % attempt)

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

#                 if isinstance(model, RotatedRect):
#                     log("Got RotatedRect directly from results key '%s'." % key)
#                     return model

#                 if hasattr(model, "size") and hasattr(model, "get"):
#                     if model.size() > 0:
#                         first = model.get(0)
#                         if isinstance(first, RotatedRect) or hasattr(first, "center"):
#                             log("Got RotatedRect[0] from list at results key '%s'." % key)
#                             return first

#             log("No RotatedRect found on attempt %d." % attempt)
#         except Exception, e:
#             log("Exception during pipeline attempt %d: %s" % (attempt, e))

#         if attempt < MAX_ATTEMPTS:
#             Thread.sleep(RETRY_DELAY_MS)
#     return None


# def get_camera_image_size(cam):
#     try:
#         return cam.getWidth(), cam.getHeight()
#     except:
#         pass
#     image = cam.settleAndCapture()
#     return image.getWidth(), image.getHeight()


# def get_camera_units_per_pixel(cam):
#     if hasattr(cam, "getUnitsPerPixelAtZ"):
#         upp_loc = cam.getUnitsPerPixelAtZ()
#     else:
#         upp_loc = cam.getUnitsPerPixel()
#     if upp_loc is None:
#         raise Exception("Camera units-per-pixel not configured.")
#     return upp_loc


# def extract_rotated_rects_from_model(model):
#     rects = []
#     if model is None:
#         return rects
#     if isinstance(model, RotatedRect) or hasattr(model, "center"):
#         rects.append(model)
#         return rects
#     if hasattr(model, "size") and hasattr(model, "get"):
#         for i in range(model.size()):
#             item = model.get(i)
#             if isinstance(item, RotatedRect) or hasattr(item, "center"):
#                 rects.append(item)
#         return rects
#     try:
#         for item in model:
#             if isinstance(item, RotatedRect) or hasattr(item, "center"):
#                 rects.append(item)
#     except:
#         pass
#     return rects


# def get_pipeline_rotated_rects(pipeline):
#     rects = []
#     for key in [RESULT_STAGE_KEY, "results", "preResults"]:
#         try:
#             res = pipeline.getResult(key)
#         except:
#             res = None
#         if res is None:
#             continue
#         try:
#             model = res.getModel()
#         except:
#             try:
#                 model = res.model
#             except:
#                 model = None
#         rects.extend(extract_rotated_rects_from_model(model))
#     return rects


# def choose_closest_rect_to_image_center(rects, width, height):
#     best = None
#     best_dist2 = None
#     cx = width / 2.0
#     cy = height / 2.0
#     for rr in rects:
#         dx = rr.center.x - cx
#         dy = rr.center.y - cy
#         dist2 = dx * dx + dy * dy
#         if best is None or dist2 < best_dist2:
#             best = rr
#             best_dist2 = dist2
#     return best, best_dist2


# def rect_center_machine_offset(cam_loc, upp_loc, rr, width, height):
#     px = rr.center.x - (width / 2.0)
#     py = rr.center.y - (height / 2.0)
#     if FLIP_IMAGE_X:
#         px = -px
#     if FLIP_IMAGE_Y:
#         py = -py

#     dx_cam = px * upp_loc.getX()
#     dy_cam = py * upp_loc.getY()

#     rad = cam_loc.getRotation() * Math.PI / 180.0
#     c = Math.cos(rad)
#     s = Math.sin(rad)
#     dx = dx_cam * c - dy_cam * s
#     dy = dx_cam * s + dy_cam * c
#     return dx, dy, px, py


# def location_xy_distance_mm(a, b):
#     a_mm = a.convertToUnits(a.getUnits())
#     b_units = a_mm.getUnits()
#     b_mm = b.convertToUnits(b_units)
#     dx = a_mm.getX() - b_mm.getX()
#     dy = a_mm.getY() - b_mm.getY()
#     return abs(dx), abs(dy)


# def detection_inside_initial_view(initial_view_loc, detected_loc, upp_loc, width, height):
#     dx, dy = location_xy_distance_mm(initial_view_loc, detected_loc)
#     upp_mm = upp_loc.convertToUnits(initial_view_loc.getUnits())
#     half_x = abs(upp_mm.getX()) * width / 2.0
#     half_y = abs(upp_mm.getY()) * height / 2.0
#     return dx <= half_x and dy <= half_y, dx, dy, half_x, half_y


# def move_camera_to_detected_center(cam, detected_loc, label):
#     cam_loc = cam.getLocation()
#     detected_cam_units = detected_loc.convertToUnits(cam_loc.getUnits())
#     target = Location(
#         cam_loc.getUnits(),
#         detected_cam_units.getX(),
#         detected_cam_units.getY(),
#         cam_loc.getZ(),
#         cam_loc.getRotation()
#     )
#     move_safe_wait(cam, target, label)


# def get_board_placements(board):
#     for method in ["getPlacements", "getBoardPlacements"]:
#         try:
#             placements = getattr(board, method)()
#             if placements is not None:
#                 log("Using board.%s()." % method)
#                 return placements
#         except:
#             pass
#     raise Exception("Board placements list is null.")


# def is_fiducial_placement(plc):
#     try:
#         if hasattr(plc, "isFiducial") and plc.isFiducial():
#             return True
#     except:
#         pass
#     try:
#         if hasattr(plc, "getType"):
#             t = plc.getType()
#             if t is not None and "FIDUCIAL" in str(t).upper():
#                 return True
#     except:
#         pass
#     return False


# def is_normal_placement(plc):
#     try:
#         if hasattr(plc, "getType"):
#             t = plc.getType()
#             if t is not None and str(t).upper() == "PLACEMENT":
#                 return True
#     except:
#         pass
#     return not is_fiducial_placement(plc)


# def get_vision_settings_for_fiducial(plc):
#     for method in ["getFiducialVisionSettings", "getVisionSettings"]:
#         try:
#             settings = getattr(plc, method)()
#             if settings is not None:
#                 return settings
#         except:
#             pass
#     try:
#         p = plc.getPart()
#         if p is not None and hasattr(p, "getFiducialVisionSettings"):
#             settings = p.getFiducialVisionSettings()
#             if settings is not None:
#                 return settings
#     except:
#         pass
#     return None


# def get_board_fiducial_placement_and_settings(board):
#     placements = get_board_placements(board)
#     count = len(java_list_to_py(placements))
#     log("Board placements count: %d" % count)

#     for i, plc in enumerate(java_list_to_py(placements)):
#         try:
#             if hasattr(plc, "isEnabled") and not plc.isEnabled():
#                 continue
#         except:
#             pass
#         if not is_fiducial_placement(plc):
#             continue
#         settings = get_vision_settings_for_fiducial(plc)
#         if settings is None:
#             log("Fiducial placement index %d has no vision settings; skipping." % i)
#             continue
#         loc = plc.getLocation()
#         label = object_name(plc) or "<unnamed>"
#         log("Selected fiducial placement '%s' at index %d." % (label, i))
#         return plc, loc, settings, label

#     raise Exception("No enabled board fiducial placement with vision settings found.")


# def get_dispense_placement(board, current_placement):
#     try:
#         if current_placement is not None and is_normal_placement(current_placement):
#             if current_placement.getPart() is not None:
#                 return current_placement
#     except:
#         pass

#     for plc in java_list_to_py(get_board_placements(board)):
#         try:
#             if hasattr(plc, "isEnabled") and not plc.isEnabled():
#                 continue
#         except:
#             pass
#         if is_normal_placement(plc):
#             try:
#                 if plc.getPart() is not None:
#                     return plc
#             except:
#                 pass
#     raise Exception("No enabled normal placement with a part found for dispense.")


# def make_board_location(bl_loc, x, y, z, r):
#     return Location(bl_loc.getUnits(), x, y, z, r)


# def prepare_pipeline_context(pipeline, cam, fid_placement, bl, prt):
#     for name, value in [
#         ("camera", cam),
#         ("placement", fid_placement),
#         ("boardLocation", bl),
#     ]:
#         try:
#             pipeline.setProperty(name, value)
#         except Exception, e:
#             log("Could not set pipeline %s property: %s" % (name, e))

#     try:
#         p = None
#         if hasattr(fid_placement, "getPart"):
#             p = fid_placement.getPart()
#         if p is None:
#             p = prt
#         if p is not None:
#             pipeline.setProperty("part", p)
#     except Exception, e:
#         log("Could not set pipeline part property: %s" % e)


# def measure_board_fiducial_correction(mac, bl, board, prt, label):
#     fid_placement, fid_loc, settings, fid_label = \
#         get_board_fiducial_placement_and_settings(board)

#     log("[%s] Using BOARD fiducial '%s' design location: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (label, fid_label, fid_loc.getX(), fid_loc.getY(),
#          fid_loc.getZ(), fid_loc.getRotation()))

#     pipeline = settings.getPipeline()
#     if pipeline is None:
#         raise Exception("Fiducial vision settings pipeline is null.")

#     bl_loc = bl.getLocation()
#     p_loc = Utils2D.calculateBoardPlacementLocation(bl, fid_loc)
#     head = mac.getDefaultHead()
#     cam = head.getDefaultCamera()
#     if cam is None:
#         raise Exception("Default head camera is null.")

#     prepare_pipeline_context(pipeline, cam, fid_placement, bl, prt)

#     base_cam_loc = cam.getLocation()
#     cam_target = Location(
#         p_loc.getUnits(),
#         p_loc.getX(),
#         p_loc.getY(),
#         base_cam_loc.getZ(),
#         base_cam_loc.getRotation()
#     )

#     last_reject = "not attempted"
#     for measurement_attempt in range(1, VISION_MEASUREMENT_ATTEMPTS + 1):
#         if measurement_attempt > 1:
#             Thread.sleep(RETRY_DELAY_MS)
#         log("[%s] Measurement attempt %d of %d." %
#             (label, measurement_attempt, VISION_MEASUREMENT_ATTEMPTS))

#         move_safe_wait(
#             cam, cam_target,
#             "%s camera nominal attempt %d" % (label, measurement_attempt))
#         initial_view_loc = cam_target

#         for recenter_pass in range(1, RECENTER_MAX_PASSES + 1):
#             wait_still(
#                 cam,
#                 "%s recenter pass %d camera" % (label, recenter_pass))
#             if PRE_CAPTURE_SETTLE_MS > 0:
#                 Thread.sleep(PRE_CAPTURE_SETTLE_MS)
#                 log("[%s] Recenter pass %d settled %d ms before pipeline.process()." %
#                     (label, recenter_pass, PRE_CAPTURE_SETTLE_MS))

#             try:
#                 pipeline.process()
#                 log("[%s] pipeline.process() OK attempt=%d pass=%d" %
#                     (label, measurement_attempt, recenter_pass))
#             except Exception, e:
#                 last_reject = "pipeline exception: %s" % e
#                 log("[%s] Rejected vision pass: %s" % (label, last_reject))
#                 break

#             rects = get_pipeline_rotated_rects(pipeline)
#             if len(rects) == 0:
#                 last_reject = "no RotatedRect results"
#                 log("[%s] Rejected vision pass: %s" % (label, last_reject))
#                 break

#             width, height = get_camera_image_size(cam)
#             rr, dist2 = choose_closest_rect_to_image_center(rects, width, height)
#             if rr is None:
#                 last_reject = "no usable RotatedRect results"
#                 log("[%s] Rejected vision pass: %s" % (label, last_reject))
#                 break

#             cam_loc = cam.getLocation()
#             upp_loc = get_camera_units_per_pixel(cam)
#             center_dx, center_dy, px, py = rect_center_machine_offset(
#                 cam_loc, upp_loc, rr, width, height)

#             # Match the original placement-starting correction convention:
#             # the measured machine-frame offset is applied as p_loc + dx/dy.
#             detected_loc = Location(
#                 cam_loc.getUnits(),
#                 cam_loc.getX() + center_dx,
#                 cam_loc.getY() + center_dy,
#                 cam_loc.getZ(),
#                 cam_loc.getRotation()
#             )
#             p_loc_bl_units = p_loc.convertToUnits(bl_loc.getUnits())
#             detected_bl_units = detected_loc.convertToUnits(bl_loc.getUnits())
#             board_dx = detected_bl_units.getX() - p_loc_bl_units.getX()
#             board_dy = detected_bl_units.getY() - p_loc_bl_units.getY()

#             log("[%s] attempt=%d pass=%d rects=%d chosen px=%.3f py=%.3f centered dx=%.6f dy=%.6f board dx=%.6f dy=%.6f angle=%.4f" %
#                 (label, measurement_attempt, recenter_pass, len(rects),
#                  px, py, center_dx, center_dy, board_dx, board_dy, rr.angle))

#             inside, view_dx, view_dy, half_x, half_y = \
#                 detection_inside_initial_view(
#                     initial_view_loc, detected_loc, upp_loc, width, height)
#             if not inside:
#                 last_reject = (
#                     "outside initial view dx=%.6f/%.6f dy=%.6f/%.6f" %
#                     (view_dx, half_x, view_dy, half_y))
#                 log("[%s] Rejected vision pass: %s" % (label, last_reject))
#                 break

#             if abs(board_dx) > MAX_NUDGE_MM or abs(board_dy) > MAX_NUDGE_MM:
#                 last_reject = (
#                     "beyond MAX_NUDGE_MM board dx=%.6f dy=%.6f limit=%.6f" %
#                     (board_dx, board_dy, MAX_NUDGE_MM))
#                 log("[%s] Rejected vision pass: %s" % (label, last_reject))
#                 break

#             if (abs(center_dx) <= RECENTER_TOLERANCE_MM and
#                     abs(center_dy) <= RECENTER_TOLERANCE_MM):
#                 new_rot = bl_loc.getRotation()
#                 if APPLY_ROTATION:
#                     new_rot = DESIGN_BOARD_ROT_DEG + (EXPECTED_ANGLE_DEG - rr.angle)
#                     log("[%s] Rotation newBoard=%.4f from measured=%.4f" %
#                         (label, new_rot, rr.angle))

#                 new_bl = make_board_location(
#                     bl_loc,
#                     bl_loc.getX() + board_dx,
#                     bl_loc.getY() + board_dy,
#                     bl_loc.getZ(),
#                     new_rot
#                 )

#                 log("[%s] Accepted centered fiducial correction -> board X=%.4f Y=%.4f R=%.4f" %
#                     (label, new_bl.getX(), new_bl.getY(), new_bl.getRotation()))

#                 return {
#                     "boardLocation": new_bl,
#                     "dx": board_dx,
#                     "dy": board_dy,
#                     "rotation": new_rot,
#                     "angle": rr.angle,
#                     "recenterPass": recenter_pass,
#                     "measurementAttempt": measurement_attempt,
#                 }

#             last_reject = (
#                 "not centered dx=%.6f dy=%.6f tolerance=%.6f" %
#                 (center_dx, center_dy, RECENTER_TOLERANCE_MM))
#             if recenter_pass < RECENTER_MAX_PASSES:
#                 log("[%s] Recenter pass %d moving camera to detected center: X=%.4f Y=%.4f" %
#                     (label, recenter_pass,
#                      detected_loc.getX(), detected_loc.getY()))
#                 move_camera_to_detected_center(
#                     cam, detected_loc,
#                     "%s recenter pass %d" % (label, recenter_pass))
#             else:
#                 log("[%s] Rejected vision attempt: %s" % (label, last_reject))

#         log("[%s] Measurement attempt %d failed: %s" %
#             (label, measurement_attempt, last_reject))

#     raise VisionMeasurementError(
#         "[%s] Board fiducial not confirmed after %d attempts; last reject: %s" %
#         (label, VISION_MEASUREMENT_ATTEMPTS, last_reject))


# def find_dispense_head(mac):
#     head = mac.getDefaultHead()
#     wanted = str(DISPENSE_HEAD_NAME)

#     for getter in ["getHeadMountables", "getNozzles"]:
#         try:
#             for item in java_list_to_py(getattr(head, getter)()):
#                 if object_name(item) == wanted:
#                     log("Using dispense head mountable '%s'." % object_name(item))
#                     return item
#         except:
#             pass

#     try:
#         item = head.getNozzleByName(wanted)
#         if item is not None:
#             log("Using dispense head mountable '%s'." % object_name(item))
#             return item
#     except:
#         pass

#     raise Exception("Required dispense head not found: %s" % wanted)


# def find_glue_dispense_actuator(mac):
#     wanted = str(DISPENSE_ACTUATOR_NAME)
#     owners = []
#     try:
#         owners.append(mac.getDefaultHead())
#     except:
#         pass
#     owners.append(mac)

#     for owner in owners:
#         if owner is None:
#             continue
#         for method in ["getActuatorByName", "getActuator"]:
#             try:
#                 actuator = getattr(owner, method)(wanted)
#                 if actuator is not None and object_name(actuator) == wanted:
#                     log("Using dispense actuator '%s'." % object_name(actuator))
#                     return actuator
#             except:
#                 pass
#         try:
#             for actuator in java_list_to_py(owner.getActuators()):
#                 if object_name(actuator) == wanted:
#                     log("Using dispense actuator '%s'." % object_name(actuator))
#                     return actuator
#         except:
#             pass

#     raise Exception("Required dispense actuator not found: %s" % wanted)


# def pad_xy(pad):
#     try:
#         return pad.getCenterX(), pad.getCenterY()
#     except:
#         return pad.getX(), pad.getY()


# def dispense_head_location(units, x, y, z):
#     return Location(units, x, y, z, Double.NaN)


# def sleep_ms(ms):
#     ms = int(ms)
#     if ms > 0:
#         Thread.sleep(ms)


# def wait_after_actuator_change(actuator, machine_obj, settle_ms, label):
#     if actuator is not None:
#         try:
#             actuator.waitForCompletion(CompletionType.WaitForUnconditionalCoordination)
#             log_t("%s: actuator wait complete" % label)
#             return
#         except:
#             pass
#     if machine_obj is not None:
#         try:
#             machine_obj.getMotionPlanner().waitForCompletion(
#                 None, CompletionType.WaitForUnconditionalCoordination)
#             log_t("%s: motion planner wait complete" % label)
#             return
#         except:
#             pass
#     sleep_ms(settle_ms)
#     log_t("%s: fallback sleep %d ms complete" % (label, int(settle_ms)))


# def pulse_glue_controller(actuator, machine_obj, pad_index):
#     t0 = log_t("pad %d: pulse_glue_controller START boolean pulse=%d ms settle=%d ms post=%d ms" %
#                (pad_index, DISPENSE_PULSE_MS, ACTUATOR_SETTLE_MS, POST_DISPENSE_DWELL_MS))

#     set_boolean_actuator_state_fast(actuator, False, "pad %d GlueDispense OFF before pulse" % pad_index)
#     wait_after_actuator_change(
#         actuator, machine_obj, ACTUATOR_SETTLE_MS,
#         "pad %d GlueDispense OFF wait" % pad_index)
#     sleep_ms(ACTUATOR_SETTLE_MS)

#     set_boolean_actuator_state_fast(actuator, True, "pad %d GlueDispense ON" % pad_index)
#     wait_after_actuator_change(
#         actuator, machine_obj, ACTUATOR_SETTLE_MS,
#         "pad %d GlueDispense ON wait" % pad_index)
#     sleep_ms(DISPENSE_PULSE_MS)

#     set_boolean_actuator_state_fast(actuator, False, "pad %d GlueDispense OFF after pulse" % pad_index)
#     wait_after_actuator_change(
#         actuator, machine_obj, ACTUATOR_SETTLE_MS,
#         "pad %d GlueDispense final OFF wait" % pad_index)
#     sleep_ms(ACTUATOR_SETTLE_MS)
#     sleep_ms(POST_DISPENSE_DWELL_MS)

#     log_t("pad %d: pulse_glue_controller END host elapsed=%.3f ms" %
#           (pad_index, elapsed_ms_since(t0)))


# def force_glue_off_safe(machine_obj, label):
#     if machine_obj is None:
#         try:
#             machine_obj = machine
#         except:
#             machine_obj = None
#     if machine_obj is None:
#         return
#     try:
#         glue = find_glue_dispense_actuator(machine_obj)
#         set_boolean_actuator_state_fast(glue, False, "%s GlueDispense forced off" % label)
#         wait_after_actuator_change(
#             glue, machine_obj, ACTUATOR_SETTLE_MS,
#             "%s GlueDispense forced off wait" % label)
#     except Exception, e:
#         log_err("%s failed to force GlueDispense off: %s" % (label, e))


# def dispense_current_placement(mac, board, bl, current_placement):
#     dispense_head = find_dispense_head(mac)
#     glue = find_glue_dispense_actuator(mac)
#     dispense_placement = get_dispense_placement(board, current_placement)
#     log_t("DISPENSE CONFIG pulse=%d post=%d pre=%d settle=%d lift=%.3f travelSpeed=%.3f dispenseSpeed=%.3f" %
#           (DISPENSE_PULSE_MS, POST_DISPENSE_DWELL_MS, PRE_DISPENSE_DWELL_MS,
#            ACTUATOR_SETTLE_MS,
#            INTER_PAD_LIFT_MM, TRAVEL_SPEED_FACTOR, DISPENSE_SPEED_FACTOR))

#     part = dispense_placement.getPart()
#     if part is None:
#         raise Exception("Dispense placement has no part.")
#     pkg = part.getPackage()
#     if pkg is None or pkg.getFootprint() is None:
#         raise Exception("Dispense part has no package footprint.")

#     pads = pkg.getFootprint().getPads()
#     if pads is None or pads.size() == 0:
#         raise Exception("Dispense footprint has no pads.")

#     active_bl_loc = bl.getLocation()
#     log("Dispense using ACTIVE corrected boardLocation X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (active_bl_loc.getX(), active_bl_loc.getY(),
#          active_bl_loc.getZ(), active_bl_loc.getRotation()))

#     world = Utils2D.calculateBoardPlacementLocation(
#         bl, dispense_placement.getLocation())
#     base_z = world.getZ() + DISPENSE_Z_OFFSET_MM
#     lift_z = base_z + INTER_PAD_LIFT_MM
#     units = world.getUnits()
#     pr = world.getRotation() * Math.PI / 180.0
#     c = Math.cos(pr)
#     s = Math.sin(pr)

#     log("Dispensing placement %s part %s footprint pads=%d using %s + %s" %
#         (object_name(dispense_placement), part.getId(), pads.size(),
#          object_name(dispense_head), object_name(glue)))
#     log("Dispense world origin X=%.4f Y=%.4f Z=%.4f R=%.4f baseZ=%.4f liftZ=%.4f" %
#         (world.getX(), world.getY(), world.getZ(), world.getRotation(), base_z, lift_z))
#     log("Dispense needle B/rotation axis blocked: all dispense-head move targets use R=NaN.")

#     set_boolean_actuator_state_fast(glue, False, "glue initial off")

#     start = dispense_head_location(units, world.getX(), world.getY(), DISPENSE_SAFE_Z_MM)
#     move_safe_wait(dispense_head, start, "dispense head start")

#     dispensed = 0
#     dispense_t0 = log_t("DISPENSE START pads=%d" % pads.size())
#     for i in range(pads.size()):
#         pad_t0 = log_t("pad %d: START" % i)
#         pad = pads.get(i)
#         pad_name = str(pad.getName())
#         if pad_name in PAD_SKIP_NAMES:
#             log("Skipping pad %d named %s." % (i, pad_name))
#             continue

#         px, py = pad_xy(pad)
#         wx = world.getX() + (px * c - py * s)
#         wy = world.getY() + (px * s + py * c)

#         log("Pad %d '%s': local X=%.4f Y=%.4f -> world X=%.4f Y=%.4f R=NaN" %
#             (i, pad_name, px, py, wx, wy))

#         move_direct(
#             dispense_head,
#             dispense_head_location(units, wx, wy, lift_z),
#             TRAVEL_SPEED_FACTOR,
#             "dispense pad %d lift" % i)
#         move_z_only(
#             dispense_head,
#             units,
#             base_z,
#             DISPENSE_SPEED_FACTOR,
#             "dispense pad %d down Z-only" % i)
#         log_t("pad %d: BEFORE pre-glue wait_coord" % i)
#         wait_coord(dispense_head, "dispense pad %d pre-glue" % i)
#         log_t("pad %d: AFTER pre-glue wait_coord / AT DISPENSE Z" % i)
#         log("Dispense head reached pad %d at dispense Z." % i)
#         sleep_ms(PRE_DISPENSE_DWELL_MS)
#         log_t("pad %d: BEFORE pulse_glue_controller" % i)
#         pulse_glue_controller(glue, mac, i)
#         log_t("pad %d: AFTER pulse_glue_controller" % i)
#         log_t("pad %d: BEFORE retract queue" % i)
#         move_z_only(
#             dispense_head,
#             units,
#             lift_z,
#             DISPENSE_SPEED_FACTOR,
#             "dispense pad %d retract Z-only" % i)
#         log_t("pad %d: AFTER retract queue; completion waits at next pre-glue or final drain" % i)
#         log_t("pad %d: END total elapsed=%.3f ms" % (i, elapsed_ms_since(pad_t0)))
#         dispensed += 1

#     log_t("DISPENSE BEFORE final retract/queue drain")
#     wait_coord(dispense_head, "dispense final retract/queue drain")
#     log_t("DISPENSE AFTER final retract/queue drain")
#     log_t("DISPENSE END total elapsed=%.3f ms dispensed=%d" %
#           (elapsed_ms_since(dispense_t0), dispensed))
#     set_boolean_actuator_state_fast(glue, False, "glue final off")
#     end = dispense_head_location(units, world.getX(), world.getY(), DISPENSE_SAFE_Z_MM)
#     move_safe_wait(dispense_head, end, "dispense head safe end")
#     log("Dispense complete: %d pads pulsed." % dispensed)
#     return dispensed


# def update_bound_placement_location(bl, current_placement):
#     final_loc = Utils2D.calculateBoardPlacementLocation(
#         bl, current_placement.getLocation())

#     log("[post-dispense] Final die placementLocation from updated boardLocation: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (final_loc.getX(), final_loc.getY(),
#          final_loc.getZ(), final_loc.getRotation()))

#     if "placementLocation" not in globals():
#         log("[post-dispense] No placementLocation binding present; boardLocation only was updated.")
#         return final_loc

#     old_loc = placementLocation
#     try:
#         log("[post-dispense] Existing bound placementLocation before update: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#             (old_loc.getX(), old_loc.getY(),
#              old_loc.getZ(), old_loc.getRotation()))
#     except:
#         log("[post-dispense] Existing bound placementLocation before update: %s" %
#             str(old_loc))

#     applied = False
#     try:
#         if hasattr(old_loc, "setLocation"):
#             old_loc.setLocation(final_loc)
#             applied = True
#             log("[post-dispense] Applied final die target via placementLocation.setLocation(...).")
#     except Exception, e:
#         log("[post-dispense] placementLocation.setLocation(...) failed: %s" % e)

#     if not applied:
#         globals()["placementLocation"] = final_loc
#         log("[post-dispense] Rebound placementLocation global to final die target.")

#     try:
#         updated_loc = placementLocation
#         log("[post-dispense] Bound placementLocation after update: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#             (updated_loc.getX(), updated_loc.getY(),
#              updated_loc.getZ(), updated_loc.getRotation()))
#     except:
#         pass

#     return final_loc


# def disable_current_placement(pl, message):
#     try:
#         if hasattr(pl, "setEnabled"):
#             pl.setEnabled(False)
#             log("placement.setEnabled(False) called.")
#     except Exception, e:
#         log("Failed to disable placement: %s" % e)
#     try:
#         existing = pl.getComments() or ""
#         if existing:
#             pl.setComments(existing + " " + message)
#         else:
#             pl.setComments(message)
#     except:
#         pass


# original_bl_loc = None
# final_update_done = False


# try:
#     required_globals = ("placement", "boardLocation", "machine")
#     missing = [name for name in required_globals if name not in globals()]
#     if missing:
#         raise Exception("Missing globals: %s" % ", ".join(missing))

#     pl = placement
#     bl = boardLocation
#     mac = machine
#     prt = part if "part" in globals() else None

#     pid = prt.getId() if (prt is not None and hasattr(prt, "getId")) else "<no-part-id>"
#     log("Part id=%s" % pid)

#     board = None
#     try:
#         board = bl.getBoard()
#     except:
#         pass
#     if board is None:
#         raise Exception("boardLocation.getBoard() returned null.")

#     log("Board object: %s" % board)

#     original_bl_loc = bl.getLocation()
#     log("Original boardLocation: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (original_bl_loc.getX(), original_bl_loc.getY(),
#          original_bl_loc.getZ(), original_bl_loc.getRotation()))

#     pre = measure_board_fiducial_correction(mac, bl, board, prt, "pre-dispense")
#     pre_bl_loc = pre["boardLocation"]
#     log("[pre-dispense] Temporary boardLocation for dispense: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (pre_bl_loc.getX(), pre_bl_loc.getY(),
#          pre_bl_loc.getZ(), pre_bl_loc.getRotation()))
#     bl.setLocation(pre_bl_loc)
#     log("[pre-dispense] Temporary boardLocation is active for dispense and post inspection.")
#     active_pre_loc = bl.getLocation()
#     log("[pre-dispense] Active boardLocation readback before syringe dispense: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (active_pre_loc.getX(), active_pre_loc.getY(),
#          active_pre_loc.getZ(), active_pre_loc.getRotation()))

#     dispense_current_placement(mac, board, bl, pl)

#     post = measure_board_fiducial_correction(mac, bl, board, prt, "post-dispense")
#     final_bl_loc = post["boardLocation"]
#     bl.setLocation(final_bl_loc)

#     final_loc = Utils2D.calculateBoardPlacementLocation(bl, pl.getLocation())
#     log("[post-dispense] Final die placementLocation from updated boardLocation: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (final_loc.getX(), final_loc.getY(),
#          final_loc.getZ(), final_loc.getRotation()))

#     final_update_done = True
#     log("[post-dispense] Updated boardLocation: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#         (final_bl_loc.getX(), final_bl_loc.getY(),
#          final_bl_loc.getZ(), final_bl_loc.getRotation()))
#     log("Combined pre-dispense, dispense, post-dispense flow complete.")

# except SystemExit:
#     pass

# except VisionMeasurementError, e:
#     log_err(e)
#     try:
#         force_glue_off_safe(mac, "vision failure")
#     except:
#         pass
#     if original_bl_loc is not None and not final_update_done:
#         try:
#             bl.setLocation(original_bl_loc)
#             log("Restored original boardLocation after vision failure: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#                 (original_bl_loc.getX(), original_bl_loc.getY(),
#                  original_bl_loc.getZ(), original_bl_loc.getRotation()))
#         except Exception, restore_e:
#             log_err("Failed to restore original boardLocation: %s" % restore_e)
#     raise

# except Exception, e:
#     log_err(e)
#     try:
#         force_glue_off_safe(mac, "placement-before-assembly error")
#     except:
#         pass
#     if original_bl_loc is not None and not final_update_done:
#         try:
#             bl.setLocation(original_bl_loc)
#             log("Restored original boardLocation after failed pre/dispense/post flow: X=%.4f Y=%.4f Z=%.4f R=%.4f" %
#                 (original_bl_loc.getX(), original_bl_loc.getY(),
#                  original_bl_loc.getZ(), original_bl_loc.getRotation()))
#         except Exception, restore_e:
#             log_err("Failed to restore original boardLocation: %s" % restore_e)
#     raise
