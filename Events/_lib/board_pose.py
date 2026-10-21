# Shared board-pose helpers for the dispense and placement event scripts.
# Compatible with the Jython 2.7 runtime used by OpenPnP.

from java.awt.geom import AffineTransform
from java.lang import Double, Math
from org.openpnp.model import LengthUnit
from org.openpnp.util import Utils2D


ANCHOR_RESIDUAL_TOLERANCE_MM = 0.01


def _finite(value):
    value = float(value)
    return not Double.isNaN(value) and not Double.isInfinite(value)


def _transform_xy(tx, x, y):
    return (
        tx.getScaleX() * x + tx.getShearX() * y + tx.getTranslateX(),
        tx.getShearY() * x + tx.getScaleY() * y + tx.getTranslateY())


def _status_name(status):
    if status is None:
        return ""
    try:
        return str(status.name())
    except:
        return str(status)


def make_fiducial_locked_transform(board_location, fid_loc,
                                   measured_world_loc, target_angle_deg):
    """Return a global affine pose that maps fid_loc to measured_world_loc.

    The existing global linear transform is retained, including panel, bottom,
    scale, and shear semantics. Its output is rotated to the requested global
    angle, then translated so the measured fiducial remains fixed.
    """
    old_tx = board_location.getLocalToGlobalTransform()
    if old_tx is None:
        raise Exception("Board global transform is null.")

    old_info = Utils2D.affineInfo(old_tx)
    old_angle = float(old_info.rotationAngleDeg)
    target_angle_deg = float(target_angle_deg)
    if not _finite(old_angle) or not _finite(target_angle_deg):
        raise Exception("Board transform angle is not finite.")

    delta_rad = (target_angle_deg - old_angle) * Math.PI / 180.0
    new_tx = AffineTransform(old_tx)
    new_tx.preConcatenate(AffineTransform.getRotateInstance(delta_rad))

    fid_mm = fid_loc.convertToUnits(LengthUnit.Millimeters)
    measured_mm = measured_world_loc.convertToUnits(LengthUnit.Millimeters)
    expected_x, expected_y = _transform_xy(
        new_tx, fid_mm.getX(), fid_mm.getY())
    translation_x = measured_mm.getX() - expected_x
    translation_y = measured_mm.getY() - expected_y
    if (not _finite(translation_x) or not _finite(translation_y)):
        raise Exception("Board transform translation is not finite.")

    new_tx.preConcatenate(AffineTransform.getTranslateInstance(
        translation_x, translation_y))

    determinant = (new_tx.getScaleX() * new_tx.getScaleY() -
                   new_tx.getShearY() * new_tx.getShearX())
    if not _finite(determinant) or abs(determinant) < 1.0e-12:
        raise Exception("Board global transform is singular or invalid.")

    return new_tx, old_angle


def snapshot_board_state(board_location):
    """Snapshot raw Location and any explicitly installed local affine pose."""
    state = {
        "location": board_location.getLocation(),
        "globalTransform": None,
        "transformStatus": None,
    }
    try:
        status = board_location.getPlacementsTransformStatus()
        state["transformStatus"] = status
        if _status_name(status) == "LocallySet":
            state["globalTransform"] = AffineTransform(
                board_location.getLocalToGlobalTransform())
    except:
        # Older OpenPnP builds may not expose transform status to scripting.
        # The raw Location remains a valid fallback for rollback.
        pass
    return state


def restore_board_state(board_location, state):
    board_location.setLocation(state["location"])
    tx = state.get("globalTransform")
    if tx is None:
        board_location.setLocalToParentTransform(None)
        return

    board_location.setLocalToGlobalTransform(AffineTransform(tx))
    status = state.get("transformStatus")
    if status is not None:
        try:
            board_location.setPlacementsTransformStatus(status)
        except:
            pass


def apply_board_transform(board_location, transform):
    """Install a global transform without invalidating it via setLocation()."""
    board_location.setLocalToGlobalTransform(AffineTransform(transform))


def set_location_preserving_transform(board_location, location):
    """Change Z/raw Location while preserving an explicit board affine pose."""
    state = snapshot_board_state(board_location)
    board_location.setLocation(location)
    tx = state.get("globalTransform")
    if tx is not None:
        board_location.setLocalToGlobalTransform(AffineTransform(tx))
        status = state.get("transformStatus")
        if status is not None:
            try:
                board_location.setPlacementsTransformStatus(status)
            except:
                pass


def fiducial_xy_residual_mm(board_location, fid_loc, measured_world_loc):
    actual = Utils2D.calculateBoardPlacementLocation(board_location, fid_loc)
    actual_mm = actual.convertToUnits(LengthUnit.Millimeters)
    measured_mm = measured_world_loc.convertToUnits(LengthUnit.Millimeters)
    return (actual_mm.getX() - measured_mm.getX(),
            actual_mm.getY() - measured_mm.getY(),
            actual)


def apply_measured_board_pose(board_location, measured):
    apply_board_transform(board_location, measured["boardTransform"])
    dx, dy, actual = fiducial_xy_residual_mm(
        board_location, measured["fiducialLocation"],
        measured["measuredFiducial"])
    residual = Math.sqrt(dx * dx + dy * dy)
    if residual > ANCHOR_RESIDUAL_TOLERANCE_MM:
        raise Exception(
            "Post-correction fiducial residual %.6f mm exceeds %.6f mm." %
            (residual, ANCHOR_RESIDUAL_TOLERANCE_MM))
    return dx, dy


def xy_shift_mm(before, after):
    before_mm = before.convertToUnits(LengthUnit.Millimeters)
    after_mm = after.convertToUnits(LengthUnit.Millimeters)
    return (after_mm.getX() - before_mm.getX(),
            after_mm.getY() - before_mm.getY())
