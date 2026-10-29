import importlib.util
import math
import os
import sys
import types
import unittest


def _install_openpnp_stubs():
    """Load the Jython script's pure geometry helpers under CPython."""
    java = types.ModuleType("java")
    java_lang = types.ModuleType("java.lang")
    java_lang.Boolean = object
    java_lang.Class = object
    java_lang.Object = object
    java_lang.System = types.SimpleNamespace(currentTimeMillis=lambda: 0,
        getProperty=lambda name: os.path.expanduser("~"))
    java_lang.Thread = types.SimpleNamespace(sleep=lambda unused: None)
    java_lang.Throwable = Exception
    java_reflect = types.ModuleType("java.lang.reflect")
    java_reflect.Array = object

    class FakeLocation(object):
        def __init__(self, *args):
            values = list(args)
            if len(values) >= 5:
                values = values[1:]
            values += [0.0] * (4 - len(values))
            self.x, self.y, self.z, self.rotation = [float(item) for item in values[:4]]

        def convertToUnits(self, unused):
            return self

        def getX(self):
            return self.x

        def getY(self):
            return self.y

        def getZ(self):
            return self.z

        def getRotation(self):
            return self.rotation

    org = types.ModuleType("org")
    openpnp = types.ModuleType("org.openpnp")
    model = types.ModuleType("org.openpnp.model")
    model.AxesLocation = object
    model.Configuration = object
    model.Length = lambda value, units=None: value
    model.LengthUnit = types.SimpleNamespace(Millimeters="mm")
    model.Location = FakeLocation
    spi = types.ModuleType("org.openpnp.spi")
    spi.Axis = types.SimpleNamespace(Type=types.SimpleNamespace(X="x", Y="y", Z="z"))
    spi.Camera = object
    planner = types.ModuleType("org.openpnp.spi.MotionPlanner")
    planner.CompletionType = types.SimpleNamespace(WaitForStillstand="still")
    util = types.ModuleType("org.openpnp.util")
    util.MovableUtils = object
    util.VisionUtils = object
    pipeline = types.ModuleType("org.openpnp.vision.pipeline")
    pipeline.CvPipeline = object

    modules = {
        "java": java, "java.lang": java_lang, "java.lang.reflect": java_reflect,
        "org": org, "org.openpnp": openpnp, "org.openpnp.model": model,
        "org.openpnp.spi": spi, "org.openpnp.spi.MotionPlanner": planner,
        "org.openpnp.util": util, "org.openpnp.vision": types.ModuleType("org.openpnp.vision"),
        "org.openpnp.vision.pipeline": pipeline,
    }
    sys.modules.update(modules)


def _load_script():
    _install_openpnp_stubs()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(root, "Events", "Homing", "needle_calibration2.py")
    spec = importlib.util.spec_from_file_location("needle_calibration2_tested", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CAL = _load_script()


def radial_contacts(center, radius, angles):
    contacts = []
    cx, cy = center
    for angle in angles:
        theta = math.radians(float(angle))
        ux, uy = math.cos(theta), math.sin(theta)
        perpendicular = cx * uy - cy * ux
        discriminant = radius * radius - perpendicular * perpendicular
        if discriminant <= 0.0:
            raise ValueError("test center is not visible on angle %s" % angle)
        travel = cx * ux + cy * uy + math.sqrt(discriminant)
        contacts.append({"angle_deg": float(angle), "x": travel * ux,
            "y": travel * uy})
    return contacts


def post_config():
    return dict(CAL.DEFAULT_CONFIG["post_xy"])


class GeometryTests(unittest.TestCase):
    def test_first_pass_recovers_signed_large_and_diagonal_offsets(self):
        for center in [(0.0, 0.0), (1.35, -0.85), (-1.10, 0.90), (0.85, 0.80)]:
            for radius in [3.45, 4.25]:
                contacts = radial_contacts(center, radius,
                    [0, 45, 90, 135, 180, 225, 270, 315])
                fit = CAL.circle_fit_contacts(contacts, {"x": 0.0, "y": 0.0})
                result = CAL.estimate_xy_contact_center(contacts,
                    {"x": 0.0, "y": 0.0}, post_config(), fit)
                self.assertTrue(result["ok"], result)
                self.assertEqual("opposite_pair_projection_ls", result["source"])
                self.assertAlmostEqual(center[0], result["displacement"]["x"], places=7)
                self.assertAlmostEqual(center[1], result["displacement"]["y"], places=7)

    def test_incomplete_asymmetric_set_uses_guarded_circle_fallback(self):
        angles = [0, 45, 90, 135, 225]
        contacts = radial_contacts((1.1, -0.7), 3.8, angles)
        fit = CAL.circle_fit_contacts(contacts, {"x": 0.0, "y": 0.0})
        result = CAL.estimate_xy_contact_center(contacts, {"x": 0.0, "y": 0.0},
            post_config(), fit)
        self.assertTrue(result["ok"], result)
        self.assertEqual("circle_fit_fallback", result["source"])
        self.assertAlmostEqual(1.1, result["displacement"]["x"], places=6)
        self.assertAlmostEqual(-0.7, result["displacement"]["y"], places=6)

    def test_uneven_coverage_is_rejected_even_with_a_circle_fit(self):
        contacts = radial_contacts((0.7, 0.4), 3.8, [0, 10, 20, 30, 40])
        fit = CAL.circle_fit_contacts(contacts, {"x": 0.0, "y": 0.0})
        result = CAL.estimate_xy_contact_center(contacts, {"x": 0.0, "y": 0.0},
            post_config(), fit)
        self.assertFalse(result["ok"])
        self.assertIn("angular coverage", result["message"])

    def test_non_circular_or_off_line_contact_is_rejected(self):
        contacts = radial_contacts((0.9, -0.5), 3.7,
            [0, 45, 90, 135, 180, 225, 270, 315])
        contacts[3]["x"] += 0.45
        fit = CAL.circle_fit_contacts(contacts, {"x": 0.0, "y": 0.0})
        result = CAL.estimate_xy_contact_center(contacts, {"x": 0.0, "y": 0.0},
            post_config(), fit)
        self.assertFalse(result["ok"])
        self.assertIn("line error", result["message"])

    def test_stale_active_start_and_tiny_interrupt_are_rejected(self):
        with self.assertRaises(CAL.NeedleCalibrationError):
            CAL._validate_radial_contact({"travel_mm": 0.002,
                "cross_track_error_mm": 0.0}, 7.0, 0.15, "EDGE 0",
                min_travel=0.25)
        with self.assertRaises(CAL.NeedleCalibrationError):
            CAL._validate_radial_contact({"travel_mm": 0.8,
                "cross_track_error_mm": 0.4}, 7.0, 0.15, "EDGE 0",
                min_travel=0.25)

    def test_repeat_cluster_does_not_accept_a_single_bad_fine_contact(self):
        samples = [{"travel_mm": 1.01}, {"travel_mm": 1.03},
            {"travel_mm": 1.50}, {"travel_mm": 1.02}]
        cluster = CAL._tightest_repeatable_cluster(samples, 2, 0.05)
        self.assertEqual(2, len(cluster))
        self.assertLessEqual(max(item["travel_mm"] for item in cluster) -
            min(item["travel_mm"] for item in cluster), 0.05)

    def test_offset_restore_helpers_remain_exact(self):
        self.assertTrue(CAL.offsets_match({"x": 1, "y": 2, "z": 3, "rotation": 4},
            {"x": 1, "y": 2, "z": 3, "rotation": 4}))
        self.assertFalse(CAL.offsets_match({"x": 1.001, "y": 2, "z": 3, "rotation": 4},
            {"x": 1, "y": 2, "z": 3, "rotation": 4}))


if __name__ == "__main__":
    unittest.main()
