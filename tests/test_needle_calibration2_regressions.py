"""Hardware-free regression coverage for needle_calibration2 geometry.

The OpenPnP backend is a Jython script. These tests provide import-only Java
stubs and replace every machine-motion boundary with fakes or mocks.
"""

import copy
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
BACKEND_PATH = ROOT / "Events" / "Homing" / "needle_calibration2.py"


class FakeLocation(object):
    def __init__(self, units, x, y, z, rotation):
        self.units = units
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.rotation = float(rotation)

    def convertToUnits(self, units):
        return FakeLocation(units, self.x, self.y, self.z, self.rotation)

    def getUnits(self):
        return self.units

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getZ(self):
        return self.z

    def getRotation(self):
        return self.rotation


class FakeNozzle(object):
    def __init__(self, offsets=None):
        self.offsets = FakeLocation("mm", 0, 0, 0, 0)
        self.location = FakeLocation("mm", 0, 0, 0, 0)
        if offsets is not None:
            self.setHeadOffsets(FakeLocation("mm", offsets["x"], offsets["y"],
                offsets["z"], offsets.get("rotation", 0)))

    def getHeadOffsets(self):
        return self.offsets

    def setHeadOffsets(self, location):
        self.offsets = location

    def getLocation(self):
        return self.location


def _install_import_stubs():
    """Install only the Java/OpenPnP names needed while importing the script."""
    names = [
        "java", "java.lang", "java.lang.reflect", "org", "org.openpnp",
        "org.openpnp.model", "org.openpnp.spi",
        "org.openpnp.spi.MotionPlanner", "org.openpnp.util",
        "org.openpnp.vision", "org.openpnp.vision.pipeline",
    ]
    modules = {name: types.ModuleType(name) for name in names}
    for name, module in modules.items():
        sys.modules[name] = module
        if "." in name:
            parent_name, child_name = name.rsplit(".", 1)
            setattr(modules[parent_name], child_name, module)

    class JavaBoolean(object):
        TYPE = bool
        TRUE = True
        FALSE = False

    class JavaSystem(object):
        @staticmethod
        def currentTimeMillis():
            return 0

    class JavaThread(object):
        @staticmethod
        def sleep(unused):
            return None

    class JavaArray(object):
        @staticmethod
        def newInstance(unused_type, count):
            return [None] * int(count)

    class FakeAxis(object):
        class Type(object):
            X = "X"
            Y = "Y"

    class FakeLengthUnit(object):
        Millimeters = "mm"

    class FakeAxesLocation(object):
        @staticmethod
        def getUnits():
            return "mm"

    class FakeCompletionType(object):
        WaitForStillstand = "still"
        WaitForUnconditionalCoordination = "coordination"

    java_lang = modules["java.lang"]
    java_lang.Boolean = JavaBoolean
    java_lang.Class = type
    java_lang.Object = object
    java_lang.System = JavaSystem
    java_lang.Thread = JavaThread
    java_lang.Throwable = Exception
    modules["java.lang.reflect"].Array = JavaArray

    model = modules["org.openpnp.model"]
    model.AxesLocation = FakeAxesLocation
    model.Configuration = type("Configuration", (), {})
    model.Length = type("Length", (), {})
    model.LengthUnit = FakeLengthUnit
    model.Location = FakeLocation

    spi = modules["org.openpnp.spi"]
    spi.Axis = FakeAxis
    spi.Camera = type("Camera", (), {})
    modules["org.openpnp.spi.MotionPlanner"].CompletionType = FakeCompletionType
    util = modules["org.openpnp.util"]
    util.MovableUtils = type("MovableUtils", (), {})
    util.VisionUtils = type("VisionUtils", (), {})
    modules["org.openpnp.vision.pipeline"].CvPipeline = type("CvPipeline", (), {})


_install_import_stubs()
spec = importlib.util.spec_from_file_location("needle_calibration2_tested", BACKEND_PATH)
needle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(needle)


def radial_circle_contacts(vision_center, displacement, contact_radius,
        angles=None):
    """Intersect rays from vision center with an offset circular contact locus."""
    if angles is None:
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
    vx = float(vision_center["x"])
    vy = float(vision_center["y"])
    dx, dy = map(float, displacement)
    contacts = []
    for angle in angles:
        theta = math.radians(float(angle))
        ux, uy = math.cos(theta), math.sin(theta)
        along = dx * ux + dy * uy
        perpendicular_sq = dx * dx + dy * dy - along * along
        radicand = float(contact_radius) ** 2 - perpendicular_sq
        if radicand < 0:
            continue
        travel = along + math.sqrt(radicand)
        contacts.append({"angle_deg": float(angle), "x": vx + travel * ux,
            "y": vy + travel * uy})
    return contacts


class NeedleCalibrationGeometryTests(unittest.TestCase):
    def measure(self, contacts, vision=(314.5, 396.9), post_radius=3.175,
            max_fit_rms=0.15, max_fit_error=0.30):
        nozzle = FakeNozzle({"x": 0, "y": 0, "z": 0, "rotation": 0})
        cfg = copy.deepcopy(needle.DEFAULT_CONFIG)
        cfg["post_xy"]["post_diameter_mm"] = 2.0 * float(post_radius)
        cfg["post_xy"]["max_fit_rms_mm"] = max_fit_rms
        cfg["post_xy"]["max_fit_error_mm"] = max_fit_error
        center = {"x": float(vision[0]), "y": float(vision[1])}
        fit = needle.circle_fit_contacts(contacts, center)
        edge_result = {"angles": [p["angle_deg"] for p in contacts],
            "all_edges": contacts, "contacts": contacts, "invalid_edges": [],
            "valid_contact_count": len(contacts), "invalid_contact_count": 0,
            "min_valid_contacts_for_fit": 5, "fit": fit}
        with mock.patch.object(needle, "get_machine", return_value=object()), \
                mock.patch.object(needle, "find_nozzle", return_value=nozzle), \
                mock.patch.object(needle, "find_actuator_by_name", return_value=object()), \
                mock.patch.object(needle, "capture_post_center",
                    return_value={"vision_center": center}), \
                mock.patch.object(needle, "resolve_post_side_probe_z_mm", return_value=0.0), \
                mock.patch.object(needle, "probe_8_point_post_xy", return_value=edge_result):
            return needle.run_post_xy_measure(cfg, nozzle, object(), lambda unused: None)

    def test_centered_needle_has_zero_xy_correction(self):
        contacts = radial_circle_contacts({"x": 0, "y": 0}, (0, 0), 3.8)
        result = self.measure(contacts, vision=(0, 0))
        self.assertTrue(result["ok"], result["message"])
        delta = result["proposal"]["measured_error"]
        self.assertAlmostEqual(delta["x"], 0.0, places=8)
        self.assertAlmostEqual(delta["y"], 0.0, places=8)
        self.assertEqual(result["details"]["valid_contact_count"], 8)

    def test_large_signed_x_y_displacement_is_recovered_in_one_pass(self):
        # For a circle of radius R centered at V+d, opposite radial ray
        # contacts have midpoint projection d·u. The four configured axes
        # have sum(u u^T)=2I, so solving those projections recovers d exactly.
        for displacement in [(1.25, -0.90), (-1.35, 0.75)]:
            contacts = radial_circle_contacts({"x": 0, "y": 0}, displacement, 4.0)
            result = self.measure(contacts, vision=(0, 0), post_radius=3.0)
            self.assertTrue(result["ok"], result["message"])
            measured = result["proposal"]["measured_error"]
            self.assertAlmostEqual(measured["x"], displacement[0], places=6)
            self.assertAlmostEqual(measured["y"], displacement[1], places=6)
            self.assertAlmostEqual(result["proposal"]["offset_delta"]["x"],
                -displacement[0], places=6)
            self.assertAlmostEqual(result["proposal"]["offset_delta"]["y"],
                -displacement[1], places=6)

    def test_diagonal_displacement_is_not_halved(self):
        displacement = (1.0, 1.0)
        contacts = radial_circle_contacts({"x": 0, "y": 0}, displacement, 4.0)
        result = self.measure(contacts, vision=(0, 0), post_radius=3.0)
        measured = result["proposal"]["measured_error"]
        self.assertAlmostEqual(measured["x"], 1.0, places=6)
        self.assertAlmostEqual(measured["y"], 1.0, places=6)

    def test_contact_radius_diagnostic_tracks_multiple_needle_radii(self):
        vision = {"x": 314.5, "y": 396.9}
        for needle_radius in (0.20, 0.65):
            contacts = radial_circle_contacts(vision, (0.35, -0.25),
                3.175 + needle_radius)
            result = self.measure(contacts, vision=(vision["x"], vision["y"]))
            fit = result["details"]["fit"]
            self.assertAlmostEqual(fit["radius_mm"], 3.175 + needle_radius,
                places=5)
            self.assertAlmostEqual(
                result["details"]["estimated_needle_tip_radius_mm"],
                needle_radius, places=5)

    def test_contact_radius_below_known_post_radius_is_rejected(self):
        contacts = radial_circle_contacts({"x": 0, "y": 0}, (0.2, -0.1), 3.05)
        result = self.measure(contacts, vision=(0, 0), post_radius=3.175)
        self.assertFalse(result["ok"])
        self.assertIn("below known post radius", result["message"])
        self.assertIsNone(result["proposal"])

    def test_noisy_non_circular_points_are_reported_as_fit_diagnostics(self):
        vision = {"x": 0.0, "y": 0.0}
        contacts = radial_circle_contacts(vision, (0.5, -0.2), 4.0)
        for index, contact in enumerate(contacts):
            # Perturb along each ray with a four-lobed shape and deterministic noise.
            theta = math.radians(contact["angle_deg"])
            radial_error = 0.22 * math.cos(4.0 * theta) + 0.025 * ((index % 3) - 1)
            contact["x"] += radial_error * math.cos(theta)
            contact["y"] += radial_error * math.sin(theta)
        result = self.measure(contacts, vision=(0, 0), post_radius=3.0)
        self.assertIsNotNone(result["details"]["fit"])
        fit_diagnostics = result["details"]["xy_estimator"]["diagnostics"]["circle_fit"]
        self.assertGreater(fit_diagnostics["rms_error_mm"], 0.15)
        self.assertFalse(result["ok"], "large circularity error must block an XY proposal")

    def test_large_non_circular_residual_is_not_accepted_as_xy(self):
        vision = {"x": 0.0, "y": 0.0}
        contacts = radial_circle_contacts(vision, (0.4, -0.3), 4.0)
        for contact in contacts:
            theta = math.radians(contact["angle_deg"])
            radial_error = 0.45 * math.cos(4.0 * theta)
            contact["x"] += radial_error * math.cos(theta)
            contact["y"] += radial_error * math.sin(theta)
        result = self.measure(contacts, vision=(0, 0), post_radius=3.0)
        self.assertFalse(result["ok"], "inconsistent contact geometry must be rejected")

    def test_asymmetric_five_contact_set_uses_conditioned_circle_fallback(self):
        vision = {"x": 0.0, "y": 0.0}
        contacts = radial_circle_contacts(vision, (0.8, -0.7), 4.0,
            angles=[0, 45, 90, 135, 180])
        result = self.measure(contacts, vision=(0, 0), post_radius=3.0)
        self.assertTrue(result["ok"], result.get("message"))
        measured = result["proposal"]["measured_error"]
        self.assertAlmostEqual(measured["x"], 0.8, places=6)
        self.assertAlmostEqual(measured["y"], -0.7, places=6)

    def test_duplicate_angle_schedule_fails_before_any_probe_motion(self):
        post = copy.deepcopy(needle.DEFAULT_CONFIG["post_xy"])
        post["angles_deg"] = [0, 0, 0, 0, 180, 180, 180, 180]
        cfg = {"post_xy": post, "travel_speed": 1.0}
        with mock.patch.object(needle, "move_safe") as safe_move, \
                mock.patch.object(needle, "move_direct_speed") as direct_move, \
                self.assertRaisesRegex(needle.NeedleCalibrationError,
                    "fewer than three unique directions"):
            needle.probe_8_point_post_xy(cfg, FakeNozzle(), object(),
                {"x": 0.0, "y": 0.0}, 3.175, 0.0)
        safe_move.assert_not_called()
        direct_move.assert_not_called()

    def test_duplicate_contact_angles_fail_estimator_conditioning(self):
        contacts = radial_circle_contacts({"x": 0.0, "y": 0.0}, (0.3, -0.2),
            4.0, angles=[0, 0, 0, 0, 180, 180, 180, 180])
        fit = {"center": {"x": 0.3, "y": -0.2}, "radius_mm": 4.0,
            "rms_error_mm": 0.0, "max_error_mm": 0.0, "condition_number": 1.0}
        result = needle.estimate_xy_contact_center(contacts,
            {"x": 0.0, "y": 0.0}, needle.DEFAULT_CONFIG["post_xy"], fit)
        self.assertFalse(result["ok"])
        self.assertIn("independent contact angles", result["message"])

    def test_pair_projection_residual_rejects_without_circle_fallback(self):
        contacts = radial_circle_contacts({"x": 0.0, "y": 0.0},
            (0.3, -0.2), 4.0)
        contacts[0]["x"] += 1.2
        fit = needle.circle_fit_contacts(contacts, {"x": 0.0, "y": 0.0})
        cfg = copy.deepcopy(needle.DEFAULT_CONFIG["post_xy"])
        cfg["max_fit_rms_mm"] = 2.0
        cfg["max_fit_error_mm"] = 2.0
        result = needle.estimate_xy_contact_center(contacts,
            {"x": 0.0, "y": 0.0}, cfg, fit)
        self.assertFalse(result["ok"])
        self.assertIn("opposite-pair projection residual", result["message"])
        self.assertFalse(result["diagnostics"]["opposite_pair_estimator"]["ok"])
        self.assertIn("circle_fit", result["diagnostics"])
        self.assertNotIn("pair_fallback_reason", result["diagnostics"])

    def test_clustered_contacts_fail_angular_coverage(self):
        contacts = radial_circle_contacts({"x": 0.0, "y": 0.0}, (0.6, 0.4),
            4.0, angles=[0, 20, 40, 60, 80])
        result = self.measure(contacts, vision=(0, 0), post_radius=3.0)
        self.assertFalse(result["ok"])
        self.assertIn("angular coverage", result["message"])

    def test_near_collinear_opposite_pairs_fail_estimator_conditioning(self):
        contacts = radial_circle_contacts({"x": 0.0, "y": 0.0}, (0.7, -0.4),
            4.0, angles=[0, 5, 180, 185])
        estimate = needle._opposite_pair_projection_estimate(contacts,
            {"x": 0.0, "y": 0.0}, 10.0)
        self.assertFalse(estimate["ok"])
        self.assertGreater(estimate["condition_number"], 10.0)


class NeedleCalibrationHardwareSafetyTests(unittest.TestCase):
    def test_move_safe_uses_direct_z_when_xy_is_already_at_target(self):
        current = FakeLocation("mm", 319.560512, 396.903109, 20.0, 0.05)
        target = FakeLocation("mm", 319.559758, 396.905457, 6.9, 0.05)
        movable = object()
        with mock.patch.object(needle, "get_movable_location",
                return_value=current), \
                mock.patch.object(needle, "move_direct_speed") as direct_move, \
                mock.patch.object(needle, "MovableUtils") as movable_utils:
            needle.move_safe(movable, target)
        direct_move.assert_called_once()
        self.assertIs(direct_move.call_args.args[0], movable)
        moved_target = direct_move.call_args.args[1]
        self.assertAlmostEqual(moved_target.getX(), target.getX())
        self.assertAlmostEqual(moved_target.getY(), target.getY())
        self.assertAlmostEqual(moved_target.getZ(), target.getZ())
        self.assertAlmostEqual(moved_target.getRotation(), target.getRotation())
        movable_utils.moveToLocationAtSafeZ.assert_not_called()

    def test_active_start_travel_is_rejected_as_stale_or_preexisting_contact(self):
        sample = {"travel_mm": 0.002941, "cross_track_error_mm": 0.0}
        with self.assertRaisesRegex(needle.NeedleCalibrationError,
                "stale/active-start"):
            needle._validate_radial_contact(sample, 7.0, 0.15,
                "coarse contact", min_travel=0.25)

    def test_g38_controller_failure_retries_from_restored_xy(self):
        contact = FakeLocation("mm", 1.0, 2.0, 0.0, 0.0)
        actuator = object()
        post = {"hardware_g38_attempts": 2, "hardware_channel": "Q1"}
        logger = lambda unused_message: None
        with mock.patch.object(needle, "_run_xy_interrupt_attempt",
                side_effect=[needle.NeedleCalibrationError("controller busy"), contact]) as attempt, \
                mock.patch.object(needle, "dwell_for_read") as dwell, \
                mock.patch.object(needle, "read_contact_debounced",
                    return_value=False) as read_state:
            result = needle._run_xy_interrupt_once(object(), object(), actuator,
                post, -1.0, 0.0, 500.0, logger, "retry test")
        self.assertIs(result, contact)
        self.assertEqual(attempt.call_count, 2)
        dwell.assert_called_once_with(post)
        read_state.assert_called_once_with(actuator, post, logger=logger,
            label="retry test retry release check", mixed_result=True)

    def test_active_contact_after_failed_g38_aborts_before_retry(self):
        actuator = object()
        post = {"hardware_g38_attempts": 3, "hardware_channel": "Q1"}
        with mock.patch.object(needle, "_run_xy_interrupt_attempt",
                side_effect=needle.NeedleCalibrationError("controller busy")) as attempt, \
                mock.patch.object(needle, "dwell_for_read") as dwell, \
                mock.patch.object(needle, "read_contact_debounced",
                    return_value=True) as read_state:
            with self.assertRaisesRegex(needle.NeedleCalibrationSafetyError,
                    "contact remains active or unstable"):
                needle._run_xy_interrupt_once(object(), object(), actuator,
                    post, -1.0, 0.0, 500.0, None, "active sensor test")
        attempt.assert_called_once()
        dwell.assert_called_once_with(post)
        read_state.assert_called_once_with(actuator, post, logger=None,
            label="active sensor test retry release check", mixed_result=True)

    def test_unstable_contact_readings_are_treated_as_active_for_retry(self):
        class Actuator(object):
            def __init__(self):
                self.values = iter([True, False])

            def read(self):
                return next(self.values)

        actuator = Actuator()
        post = {"hardware_g38_attempts": 2, "hardware_channel": "Q1",
            "read_settle_ms": 0, "debounce_count": 2,
            "debounce_retries": 1, "debounce_interval_ms": 0}
        with mock.patch.object(needle, "_run_xy_interrupt_attempt",
                side_effect=needle.NeedleCalibrationError("controller busy")) as attempt:
            with self.assertRaisesRegex(needle.NeedleCalibrationSafetyError,
                    "contact remains active or unstable"):
                needle._run_xy_interrupt_once(object(), object(), actuator,
                    post, -1.0, 0.0, 500.0, None, "unstable sensor test")
        attempt.assert_called_once()

    def test_missing_contact_actuator_disables_g38_retry(self):
        post = {"hardware_g38_attempts": 2, "hardware_channel": "Q1"}
        with mock.patch.object(needle, "_run_xy_interrupt_attempt",
                side_effect=needle.NeedleCalibrationError("controller busy")) as attempt, \
                mock.patch.object(needle, "read_contact_debounced") as read_state:
            with self.assertRaisesRegex(needle.NeedleCalibrationSafetyError,
                    "actuator is unavailable"):
                needle._run_xy_interrupt_once(object(), object(), None, post,
                    -1.0, 0.0, 500.0, None, "missing actuator test")
        attempt.assert_called_once()
        read_state.assert_not_called()

    def test_g38_and_xy_restore_failure_aborts_without_retry(self):
        class Driver(object):
            def __init__(self):
                self.commands = []

            def sendCommand(self, command):
                self.commands.append(command)
                if command.startswith("G38.2"):
                    raise RuntimeError("synthetic G38 failure")
                if command in ("G90", "G61 S0 X Y F7500"):
                    raise RuntimeError("synthetic restore failure: " + command)

            def getReportedLocation(self, unused):
                return FakeLocation("mm", 0, 0, 0, 0)

        driver = Driver()
        with mock.patch.object(needle, "read_contact_debounced") as read_state:
            with self.assertRaises(needle.NeedleCalibrationSafetyError) as raised:
                needle._run_xy_interrupt_once(object(), driver, object(),
                    {"hardware_g38_attempts": 3, "hardware_channel": "Q1"},
                    -1.0, 0.0, 500.0, None, "restore-failure test")
        read_state.assert_not_called()
        message = str(raised.exception)
        self.assertIn("G90", message)
        self.assertIn("G61 S0 X Y F7500", message)
        self.assertIn("synthetic G38 failure", message)
        self.assertIn("retry is unsafe", message)
        self.assertEqual(1, len([cmd for cmd in driver.commands
            if cmd.startswith("G38.2")]))
        self.assertIn("G90", driver.commands)
        self.assertIn("G61 S0 X Y F7500", driver.commands)

    def test_unreadable_enabled_xy_limit_state_aborts_start_preflight(self):
        class Axis(object):
            def getType(self):
                return needle.Axis.Type.X

            def isSoftLimitLowEnabled(self):
                return True

            def isSoftLimitHighEnabled(self):
                return False

            def getSoftLimitLow(self):
                raise RuntimeError("synthetic unreadable low limit")

            def getName(self):
                return "X"

        class Raw(object):
            def getAxes(self):
                return [Axis()]

            def getCoordinate(self, unused_axis):
                return 0.0

        nozzle = FakeNozzle()
        nozzle.toHeadLocation = lambda target: target
        nozzle.toRaw = lambda unused_target: Raw()
        cfg = {"post_xy": copy.deepcopy(needle.DEFAULT_CONFIG["post_xy"]),
            "travel_speed": 1.0}
        with mock.patch.object(needle, "move_direct_speed") as move, \
                self.assertRaisesRegex(needle.NeedleCalibrationSafetyError,
                    "enabled XY low soft limit"):
            needle._prepare_hardware_start(cfg, nozzle, object(), cfg["post_xy"],
                {"x": 0.0, "y": 0.0}, 0.0, 0.0, 4.175, 0.0,
                1.0, 0.0, 4.175, 1.0, None, "unreadable limit")
        move.assert_not_called()

    def test_unreadable_enabled_xy_coordinate_aborts_start_preflight(self):
        class Axis(object):
            def getType(self):
                return needle.Axis.Type.X

            def isSoftLimitLowEnabled(self):
                return True

            def isSoftLimitHighEnabled(self):
                return False

        class Raw(object):
            def getAxes(self):
                return [Axis()]

            def getCoordinate(self, unused_axis):
                raise RuntimeError("synthetic unreadable coordinate")

        nozzle = FakeNozzle()
        nozzle.toHeadLocation = lambda target: target
        nozzle.toRaw = lambda unused_target: Raw()
        with self.assertRaisesRegex(needle.NeedleCalibrationSafetyError,
                "enabled XY soft-limit coordinate"):
            needle._hardware_xy_start_violation(nozzle,
                {"x": 0.0, "y": 0.0}, 0.0, 4.175, 0.0)

    def test_hardware_coarse_start_checks_sensor_before_g38(self):
        # Every movement and interrupt call is mocked; this checks for a
        # software preflight/recovery decision without touching machine IO.
        nozzle = object()
        post = copy.deepcopy(needle.DEFAULT_CONFIG["post_xy"])
        cfg = {"post_xy": post, "travel_speed": 1.0}
        with mock.patch.object(needle, "get_movable_location",
                return_value=FakeLocation("mm", 0, 0, 0, 0)), \
                mock.patch.object(needle, "_driver_for_probe", return_value=object()), \
                mock.patch.object(needle, "move_safe"), \
                mock.patch.object(needle, "move_direct_speed"), \
                mock.patch.object(needle, "_bounded_hardware_start_radius",
                    return_value=(4.175, None)), \
                mock.patch.object(needle, "read_contact_debounced",
                    return_value=True) as read_state, \
                mock.patch.object(needle, "_run_xy_interrupt_once",
                    side_effect=needle.NeedleCalibrationError("active contact")) as interrupt:
            with self.assertRaises(needle.NeedleCalibrationError):
                needle.probe_post_edge_hardware(cfg, nozzle, object(),
                    {"x": 0, "y": 0}, 0, 3.175, 0.0)
        read_state.assert_called()
        interrupt.assert_not_called()

    def test_outside_start_recovers_outward_until_contact_releases(self):
        state = {"location": FakeLocation("mm", 4.175, 0, 0, 0)}
        nozzle = object()

        def fake_move(unused_nozzle, target, unused_speed):
            state["location"] = target

        with mock.patch.object(needle, "dwell_for_read"), \
                mock.patch.object(needle, "get_movable_location",
                    side_effect=lambda unused_nozzle: state["location"]), \
                mock.patch.object(needle, "move_direct_speed", side_effect=fake_move) as move, \
                mock.patch.object(needle, "read_contact_debounced",
                    side_effect=[True, True, False]):
            recovered, outward = needle.recover_outside_start_if_touching(
                nozzle, object(), {"outside_recover_step_mm": 0.25,
                    "outside_recover_max_mm": 3.0}, 4.175, 0.0, 1.0, 0.0,
                0.0, 1.0)
        self.assertAlmostEqual(outward, 0.5)
        self.assertAlmostEqual(recovered.getX(), 4.675)
        self.assertEqual(move.call_count, 5)  # initial lift, then lift/probe per outward step
        self.assertAlmostEqual(move.call_args_list[-1].args[1].getZ(), 0.0)

    def test_near_zero_coarse_contact_recovers_far_enough_for_minimum_travel(self):
        nozzle = FakeNozzle()
        contact_x = 4.177941
        cfg = {"post_xy": copy.deepcopy(needle.DEFAULT_CONFIG["post_xy"]),
            "travel_speed": 1.0}
        interrupt_starts = []
        movements = []

        def fake_move(unused_nozzle, target, speed=None):
            nozzle.location = target
            movements.append((target.getX(), target.getY(), target.getZ()))

        def fake_interrupt(unused_nozzle, unused_driver, unused_actuator,
                unused_post, unused_dx, unused_dy, unused_feed, unused_logger,
                label):
            interrupt_starts.append((label, nozzle.location.getX()))
            return FakeLocation("mm", contact_x, 0.0, 0.0, 0.0)

        with mock.patch.object(needle, "_driver_for_probe", return_value=object()), \
                mock.patch.object(needle, "move_safe", side_effect=fake_move), \
                mock.patch.object(needle, "move_direct_speed", side_effect=fake_move), \
                mock.patch.object(needle, "_bounded_hardware_start_radius",
                    side_effect=lambda unused_nozzle, unused_center, unused_angle,
                        radius, unused_z: (float(radius), None)), \
                mock.patch.object(needle, "read_contact_debounced", return_value=False), \
                mock.patch.object(needle, "_run_xy_interrupt_once",
                    side_effect=fake_interrupt):
            result = needle.probe_post_edge_hardware(cfg, nozzle, object(),
                {"x": 0.0, "y": 0.0}, 0.0, 3.175, 0.0)

        coarse_calls = [item for item in interrupt_starts if "coarse" in item[0]]
        self.assertEqual(2, len(coarse_calls))
        self.assertAlmostEqual(coarse_calls[0][1], 4.175, places=6)
        outward = coarse_calls[1][1] - coarse_calls[0][1]
        self.assertGreater(outward, 0.25)
        self.assertAlmostEqual(outward, 0.262941, places=6)
        self.assertGreaterEqual(result["coarse_samples"][0]["travel_mm"], 0.25)
        self.assertGreater(len(movements), 0)  # all movement stayed in fake state


class NeedleCalibrationRollbackTests(unittest.TestCase):
    def setUp(self):
        self.original = {"x": 0.4, "y": -0.3, "z": 1.2, "rotation": 0.0}
        self.nozzle = FakeNozzle(self.original)
        self.cfg = copy.deepcopy(needle.DEFAULT_CONFIG)

    def patch_full_measure_dependencies(self, post_result):
        patches = [
            mock.patch.object(needle, "get_machine", return_value=object()),
            mock.patch.object(needle, "find_nozzle", return_value=self.nozzle),
            mock.patch.object(needle, "run_z_measure", return_value={"ok": True,
                "proposal": {"proposed_offsets": {"x": 0.4, "y": -0.3,
                    "z": 1.7, "rotation": 0.0},
                    "offset_delta": {"z": 0.5}}}),
            mock.patch.object(needle, "run_post_xy_measure", return_value=post_result),
            mock.patch.object(needle, "synchronize_coordinate_system"),
            mock.patch.object(needle, "lift_nozzle_to_safe_z"),
        ]
        return patches

    def test_temporary_z_is_active_during_xy_and_restored_after_xy_failure(self):
        post_result = {"ok": False, "message": "synthetic rejected XY"}
        patches = self.patch_full_measure_dependencies(post_result)
        with patches[0], patches[1], patches[2], mock.patch.object(
                needle, "run_post_xy_measure", side_effect=self.assert_xy_sees_temp_z), \
                patches[4], patches[5]:
            result = needle.run_full_measure(self.cfg, self.nozzle, object())
        self.assertFalse(result["ok"])
        self.assertTrue(result["offsets_restored"])
        self.assertEqual(result["calibration_state"]["status"], "reverted")
        self.assertEqual(needle.loc_to_dict(self.nozzle.getHeadOffsets()), self.original)

    def assert_xy_sees_temp_z(self, *unused_args):
        active = needle.loc_to_dict(self.nozzle.getHeadOffsets())
        self.assertEqual(active["x"], self.original["x"])
        self.assertEqual(active["y"], self.original["y"])
        self.assertEqual(active["z"], self.original["z"] + 0.5)
        return {"ok": False, "message": "synthetic rejected XY"}

    def test_successful_full_measure_keeps_z_temporary_until_explicit_apply(self):
        post_result = {"ok": True, "proposal": {"offset_delta": {"x": -0.2,
            "y": 0.15}, "proposed_offsets": {"x": 0.2, "y": -0.15,
            "z": 1.7, "rotation": 0.0}}}
        patches = self.patch_full_measure_dependencies(post_result)
        with patches[0], patches[1], patches[2], patches[3], patches[4], patches[5]:
            result = needle.run_full_measure(self.cfg, self.nozzle, object())
        self.assertTrue(result["ok"], result["message"])
        self.assertFalse(result["offsets_restored"])
        self.assertTrue(result["calibration_state"]["temporary_z_active"])
        self.assertEqual(needle.loc_to_dict(self.nozzle.getHeadOffsets())["z"], 1.7)
        self.assertEqual(result["proposal"]["proposed_offsets"],
            {"x": 0.2, "y": -0.15, "z": 1.7, "rotation": 0.0})


if __name__ == "__main__":
    unittest.main()
