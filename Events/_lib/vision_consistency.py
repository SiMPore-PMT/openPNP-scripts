# Vision measurement consistency helpers for the Jython 2.7 runtime.

from java.lang import Math
from org.openpnp.model import LengthUnit, Location


# Confirmation is deliberately tighter than MAX_VISION_WORLD_DISAGREEMENT_MM.
VISION_CONFIRMATION_TRIGGER_MM = 0.05
VISION_CONFIRMATION_CAPTURES = 3
VISION_CONFIRMATION_INLIER_TOLERANCE_MM = 0.025
VISION_CONFIRMATION_ANGLE_TOLERANCE_DEG = 15.0


def world_xy_distance_mm(first, second):
    first_mm = first.convertToUnits(LengthUnit.Millimeters)
    second_mm = second.convertToUnits(LengthUnit.Millimeters)
    dx = first_mm.getX() - second_mm.getX()
    dy = first_mm.getY() - second_mm.getY()
    return Math.sqrt(dx * dx + dy * dy)


def resolve_world_measurement(anchor, pending, current, tolerance_mm):
    if anchor is None:
        return current, None, 0.0, "anchor"

    disagreement = world_xy_distance_mm(anchor, current)
    if disagreement <= tolerance_mm:
        return anchor, None, disagreement, "consistent"

    if (pending is not None and
            world_xy_distance_mm(pending, current) <= tolerance_mm):
        return current, None, disagreement, "suspect-confirmed"

    return anchor, current, disagreement, "suspect"


def capture_world_measurement(pipeline, cam, image_size_fn,
                              units_per_pixel_fn, rects_fn, choose_fn,
                              offset_fn, reference_loc):
    rects = rects_fn(pipeline)
    if not rects:
        return None

    width, height = image_size_fn(cam)
    cam_loc = cam.getLocation()
    upp_loc = units_per_pixel_fn(cam)
    rr, dist2 = choose_fn(
        rects, width, height, cam_loc, upp_loc, reference_loc)
    if rr is None:
        return None

    dx, dy, px, py = offset_fn(
        cam_loc, upp_loc, rr, width, height)
    detected_loc = Location(
        cam_loc.getUnits(), cam_loc.getX() + dx,
        cam_loc.getY() + dy, cam_loc.getZ(), cam_loc.getRotation())
    return {
        "rects": rects,
        "width": width,
        "height": height,
        "camLoc": cam_loc,
        "uppLoc": upp_loc,
        "rect": rr,
        "dist2": dist2,
        "dx": dx,
        "dy": dy,
        "px": px,
        "py": py,
        "location": detected_loc,
        "angle": rr.angle,
    }


def _angle_distance_deg(first, second):
    difference = abs(float(first) - float(second)) % 180.0
    return min(difference, 180.0 - difference)


def _measurements_match(first, second, tolerance_mm, angle_tolerance_deg):
    return (world_xy_distance_mm(first["location"], second["location"]) <=
            tolerance_mm and
            _angle_distance_deg(first["angle"], second["angle"]) <=
            angle_tolerance_deg)


def _median(values):
    values = sorted(values)
    middle = len(values) / 2
    if len(values) % 2:
        return values[middle]
    return (values[middle - 1] + values[middle]) / 2.0


def _consensus_cluster(measurements, minimum_count,
                       tolerance_mm, angle_tolerance_deg):
    if len(measurements) < minimum_count:
        return None

    clusters = {}
    for seed_index in range(len(measurements)):
        indices = []
        seed = measurements[seed_index]
        for index, candidate in enumerate(measurements):
            if _measurements_match(seed, candidate,
                                   tolerance_mm, angle_tolerance_deg):
                indices.append(index)

        pairwise = True
        for left in range(len(indices)):
            for right in range(left + 1, len(indices)):
                if not _measurements_match(
                        measurements[indices[left]], measurements[indices[right]],
                        tolerance_mm, angle_tolerance_deg):
                    pairwise = False
                    break
            if not pairwise:
                break
        if pairwise:
            clusters[tuple(indices)] = indices

    if not clusters:
        return None

    ranked = sorted(clusters.values(), key=len, reverse=True)
    best = ranked[0]
    if len(best) < minimum_count:
        return None
    if (len(ranked) > 1 and
            len(best) < len(ranked[1]) + 2):
        return None

    units = measurements[best[0]]["location"].getUnits()
    locations_mm = [
        item["location"].convertToUnits(LengthUnit.Millimeters)
        for item in [measurements[index] for index in best]]
    consensus_loc_mm = Location(
        LengthUnit.Millimeters,
        _median([loc.getX() for loc in locations_mm]),
        _median([loc.getY() for loc in locations_mm]),
        locations_mm[0].getZ(), locations_mm[0].getRotation())
    consensus_loc = consensus_loc_mm.convertToUnits(units)

    angle_measurements = [measurements[index] for index in best]
    angle_medoid = angle_measurements[0]
    angle_score = None
    for candidate in angle_measurements:
        score = 0.0
        for other in angle_measurements:
            score += _angle_distance_deg(candidate["angle"], other["angle"])
        if angle_score is None or score < angle_score:
            angle_medoid = candidate
            angle_score = score

    spread_mm = 0.0
    angle_spread_deg = 0.0
    for left in angle_measurements:
        for right in angle_measurements:
            spread_mm = max(
                spread_mm,
                world_xy_distance_mm(left["location"], right["location"]))
            angle_spread_deg = max(
                angle_spread_deg,
                _angle_distance_deg(left["angle"], right["angle"]))

    return {
        "location": consensus_loc,
        "angle": angle_medoid["angle"],
        "count": len(best),
        "spreadMm": spread_mm,
        "angleSpreadDeg": angle_spread_deg,
    }


def confirm_world_measurements(initial_measurements, pipeline, cam,
                               capture_count, label, settle_ms, wait_fn,
                               sleep_fn, checkpoint_fn, image_size_fn,
                               units_per_pixel_fn, rects_fn, choose_fn,
                               offset_fn, initial_view_loc, inside_fn,
                               expected_loc_mm, max_nudge_mm, cam_loc,
                               upp_loc, width, height):
    measurements = list(initial_measurements)
    for unused in range(capture_count):
        capture_label = "%s confirmation capture %d" % (label, unused + 1)
        wait_fn(cam, capture_label)
        if settle_ms > 0:
            sleep_fn(settle_ms, "%s settle" % capture_label)
        checkpoint_fn("%s before vision capture" % capture_label)
        try:
            pipeline.process()
        except Exception:
            return None
        checkpoint_fn("%s after vision capture" % capture_label)
        measurement = capture_world_measurement(
            pipeline, cam, image_size_fn, units_per_pixel_fn, rects_fn,
            choose_fn, offset_fn, None)
        if measurement is None:
            return None
        loc = measurement["location"]
        loc_mm = loc.convertToUnits(LengthUnit.Millimeters)
        dx = loc_mm.getX() - expected_loc_mm.getX()
        dy = loc_mm.getY() - expected_loc_mm.getY()
        if (not inside_fn(initial_view_loc, loc, measurement["uppLoc"],
                          measurement["width"], measurement["height"])[0] or
                abs(dx) > max_nudge_mm or abs(dy) > max_nudge_mm):
            return None
        measurements.append({
            "location": loc, "angle": measurement["angle"]})
    consensus = _consensus_cluster(
        measurements, 3, VISION_CONFIRMATION_INLIER_TOLERANCE_MM,
        VISION_CONFIRMATION_ANGLE_TOLERANCE_DEG)
    if consensus is None:
        return None

    loc = consensus["location"]
    loc_mm = loc.convertToUnits(LengthUnit.Millimeters)
    consensus_cam = loc.convertToUnits(cam_loc.getUnits())
    center_dx = consensus_cam.getX() - cam_loc.getX()
    center_dy = consensus_cam.getY() - cam_loc.getY()
    board_dx = loc_mm.getX() - expected_loc_mm.getX()
    board_dy = loc_mm.getY() - expected_loc_mm.getY()
    if (not inside_fn(initial_view_loc, loc, upp_loc, width, height)[0] or
            abs(board_dx) > max_nudge_mm or abs(board_dy) > max_nudge_mm):
        return None
    return (loc, loc_mm, center_dx, center_dy, board_dx, board_dy,
            consensus["angle"], consensus["count"], consensus["spreadMm"],
            consensus["angleSpreadDeg"])
