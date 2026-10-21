# Vision measurement consistency helpers for the Jython 2.7 runtime.

from java.lang import Math
from org.openpnp.model import LengthUnit


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
