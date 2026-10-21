# operator_calibration_gui.py
#
# Simplified operator front end for syringe/needle and pick-head calibration.
# This script is intended to be launched from OpenPnP's Scripts menu.

import imp
import json
import math
import os
import sys

from java.awt import BorderLayout, CardLayout, Color, Dimension, FlowLayout, Font
from java.awt.event import WindowAdapter
from java.lang import Boolean, Class, Object, Runnable, Throwable
from java.lang.reflect import Array
from java.util.concurrent import Callable
from java.util.concurrent.atomic import AtomicBoolean
from javax.swing import (
    BorderFactory, JButton, JFrame, JLabel, JOptionPane, JPanel, JRadioButton,
    JScrollPane,
    JTextArea, SwingConstants, SwingUtilities, SwingWorker, Timer
)
from org.openpnp.model import Configuration, LengthUnit, Location
from org.openpnp.spi.MotionPlanner import CompletionType


def resolve_script_dir():
    candidates = []
    try:
        candidates.append(str(scripting.getScriptsDirectory().toString()))
    except:
        pass
    try:
        candidates.append(os.path.dirname(os.path.abspath(__file__)))
    except:
        pass
    candidates.append(os.getcwd())

    # The GUI lives one directory below the OpenPnP Scripts root. Check each
    # discovered directory and its parent so launching from either location
    # still resolves the shared backends and calibration JSON files.
    checked = {}
    for candidate in candidates:
        for possible_root in [candidate, os.path.dirname(candidate)]:
            try:
                possible_root = os.path.abspath(str(possible_root))
            except:
                continue
            if possible_root in checked:
                continue
            checked[possible_root] = True
            backend = os.path.join(
                possible_root, "Events", "Homing", "needle_calibration2.py")
            if os.path.exists(backend):
                return possible_root
    raise Exception(
        "Could not locate the OpenPnP Scripts root from Operator Scripts")


SCRIPT_DIR = resolve_script_dir()
HOMING_DIR = os.path.join(SCRIPT_DIR, "Events", "Homing")
NEEDLE_CONFIG_PATH = os.path.join(SCRIPT_DIR, "needle_calibration_config.json")
LUMEN_CONFIG_PATH = os.path.join(SCRIPT_DIR, "lumen_calibration_gui_config.json")

DISPENSE_NOZZLE_NAME = "Dispense Head"
PICK_NOZZLE_NAME = "Pick Head"
DISPENSE_SOLENOID_NAME = "DispenseVac_Solenoid"
VAC_DISPENSE_ACTUATOR_NAME = "VacDispenseActuator"

LOAD_X_MM = 232.499
LOAD_Y_MM = -68.821
LOAD_Z_MM = 39.198

if HOMING_DIR not in sys.path:
    sys.path.append(HOMING_DIR)

needle_calibration = imp.load_source(
    "operator_needle_calibration_backend",
    os.path.join(HOMING_DIR, "needle_calibration2.py"))
tip_metrology = imp.load_source(
    "operator_lumen_tip_metrology_backend",
    os.path.join(HOMING_DIR, "lumen_tip_metrology.py"))


class CalibrationAborted(Exception):
    pass


def get_machine():
    try:
        return machine
    except:
        return Configuration.get().getMachine()


def require_machine_execute():
    machine_obj = get_machine()
    if not hasattr(machine_obj, "execute"):
        raise Exception("OpenPnP machine task execution is not available")
    return machine_obj


def java_items(items):
    out = []
    if items is None:
        return out
    try:
        iterator = items.iterator()
        while iterator.hasNext():
            out.append(iterator.next())
        return out
    except:
        pass
    try:
        for item in items:
            out.append(item)
    except:
        pass
    return out


def object_name(obj):
    try:
        return str(obj.getName())
    except:
        return str(obj)


def find_nozzle(machine_obj, wanted):
    for head in java_items(machine_obj.getHeads()):
        for nozzle in java_items(head.getNozzles()):
            if object_name(nozzle) == wanted:
                return nozzle
    raise Exception("Nozzle not found: %s" % wanted)


def read_json(path):
    if not os.path.exists(path):
        raise Exception("Configuration file not found: %s" % path)
    handle = open(path, "r")
    try:
        value = json.load(handle)
    finally:
        handle.close()
    if not isinstance(value, dict):
        raise Exception("Configuration root must be a JSON object: %s" % path)
    return value


def compact_json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def deep_copy(value):
    return json.loads(json.dumps(value, sort_keys=True))


def loc_to_dict(location):
    location = location.convertToUnits(LengthUnit.Millimeters)
    return {
        "x": float(location.getX()),
        "y": float(location.getY()),
        "z": float(location.getZ()),
        "rotation": float(location.getRotation()),
    }


def offsets_match(first, second, tolerance=0.000001):
    if first is None or second is None:
        return False
    for axis in ["x", "y", "z", "rotation"]:
        if abs(float(first.get(axis, 0.0)) -
                float(second.get(axis, 0.0))) > tolerance:
            return False
    return True


def numeric_value(value, default=None):
    try:
        if value is None:
            return default
        return float(value)
    except:
        return default


def clamped(value, low, high):
    return max(low, min(high, value))


def scaled_penalty(value, start, full, max_penalty):
    value = numeric_value(value, None)
    if value is None or value <= start:
        return 0.0
    if full <= start:
        return float(max_penalty)
    ratio = (value - start) / float(full - start)
    return clamped(ratio, 0.0, 1.0) * float(max_penalty)


def score_band(score):
    if int(score) >= 90:
        return "Excellent"
    if int(score) >= 75:
        return "Good"
    if int(score) >= 50:
        return "Fair"
    return "Poor"


def xy_magnitude(value):
    if value is None:
        return None
    x = numeric_value(value.get("x"), None)
    y = numeric_value(value.get("y"), None)
    if x is None or y is None:
        return None
    return math.sqrt(x * x + y * y)


def rotation_error_penalty(records):
    worst = 0.0
    count = 0
    for record in records:
        if record.get("accepted", True) is False:
            continue
        actual = numeric_value(record.get("actual_artifact_rotation_delta_deg"), None)
        if actual is None:
            continue
        worst = max(worst, abs(abs(actual) - 180.0))
        count += 1
    if count == 0 and len(records) > 0:
        return 15.0, "no accepted pairwise spin records"
    penalty = scaled_penalty(worst, 2.0, 30.0, 15.0)
    if penalty > 0.0:
        return penalty, "pairwise spin error %.1f deg" % worst
    return 0.0, None


def full_run_warning_penalty(full_result):
    penalty = 0.0
    reasons = []
    for key in ["square", "post_pairwise_resquare",
                "pre_builtin_square_verify", "post_builtin_square_verify"]:
        section = full_result.get(key) or {}
        if not section:
            continue
        if section.get("ok") is False:
            penalty += 3.0
            reasons.append("%s reported not ok" % key)
        error = section.get("normalized_error_deg")
        if error is None:
            error = section.get("final_error_deg")
        error = numeric_value(error, None)
        section_penalty = scaled_penalty(
            abs(error) if error is not None else None, 0.25, 1.50, 3.0)
        if section_penalty > 0.0:
            penalty += section_penalty
            reasons.append("%s error %.2f deg" % (key, abs(error)))
    artifact_return = full_result.get("artifact_return") or {}
    if artifact_return:
        if artifact_return.get("ok") is False:
            penalty += 3.0
            reasons.append("artifact return reported not ok")
        xy_error = xy_magnitude(artifact_return.get("storage_verify_error_mm"))
        xy_penalty = scaled_penalty(xy_error, 0.05, 0.50, 3.0)
        if xy_penalty > 0.0:
            penalty += xy_penalty
            reasons.append("storage return XY error %.3f mm" % xy_error)
        angle = numeric_value(artifact_return.get("storage_verify_error_deg"), None)
        angle_penalty = scaled_penalty(
            abs(angle) if angle is not None else None, 1.0, 10.0, 2.0)
        if angle_penalty > 0.0:
            penalty += angle_penalty
            reasons.append("storage return angle error %.2f deg" % abs(angle))
    return min(10.0, penalty), reasons


def calibration_quality_score(result):
    if result is None or not result.get("ok", False):
        return {"score": 0, "band": "Poor",
                "reasons": ["calibration did not complete successfully"]}
    solver = result.get("head_offset_solver") or {}
    if not solver.get("ok", False):
        return {"score": 0, "band": "Poor",
                "reasons": ["head offset solver did not complete successfully"]}
    score = 100.0
    reasons = []
    valid = int(numeric_value(solver.get("pairwise_valid_pair_count"), 0.0))
    rejected = int(numeric_value(solver.get("pairwise_rejected_pair_count"), 0.0))
    total = valid + rejected
    if total > 0 and rejected > 0:
        score -= 35.0 * float(rejected) / float(total)
        reasons.append("rejected pairwise pairs %d/%d" % (rejected, total))
    penalty, reason = rotation_error_penalty(solver.get("per_angle_records") or [])
    score -= penalty
    if reason is not None:
        reasons.append(reason)
    for key, start, full, maximum, label in [
            ("pairwise_rms_raw_delta_mm", 0.05, 0.50, 15.0, "RMS raw delta"),
            ("pairwise_rms_solved_correction_mm", 0.02, 0.25, 15.0,
             "RMS solved correction"),
            ("final_correction_mag_mm", 0.05, 0.50, 10.0,
             "final correction")]:
        value = numeric_value(solver.get(key), None)
        penalty = scaled_penalty(value, start, full, maximum)
        score -= penalty
        if penalty > 0.0:
            reasons.append("%s %.3f mm" % (label, value))
    penalty, more_reasons = full_run_warning_penalty(result)
    score -= penalty
    reasons.extend(more_reasons)
    score = int(round(clamped(score, 0.0, 100.0)))
    if not reasons:
        reasons.append("no quality penalties")
    return {"score": score, "band": score_band(score), "reasons": reasons}


class OperatorCalibrationGui(object):
    def __init__(self):
        self.frame = JFrame("Operator Calibration")
        self.frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE)
        self.frame.setMinimumSize(Dimension(760, 570))
        self.frame.addWindowListener(self.CloseListener(self))

        self.busy = False
        self.abort_requested = AtomicBoolean(False)
        self.abort_enabled = AtomicBoolean(False)
        self.abort_btn = None
        self.completed = {"needle": False, "lumen": False}
        self.current_flow = None
        self.enable_vac_dispense_on_next_run = False
        self.last_error = None

        self.needle_cfg = None
        self.needle_result = None
        self.needle_before = None
        self.needle_pending_state = None

        self.lumen_cfg = None
        self.lumen_config_snapshot = None
        self.lumen_result = None
        self.lumen_baseline = None
        self.lumen_expected_offsets = None
        self.lumen_quality = None
        self.lumen_pending = False

        self.card_layout = CardLayout()
        self.cards = JPanel(self.card_layout)
        self.card_index = 0
        self.log_area = JTextArea(7, 80)
        self.log_area.setEditable(False)
        self.log_area.setLineWrap(True)
        self.log_area.setWrapStyleWord(True)

    class CloseListener(WindowAdapter):
        def __init__(self, gui_obj):
            self.gui_obj = gui_obj

        def windowClosing(self, event):
            self.gui_obj.request_close()

    def build(self):
        root = JPanel(BorderLayout(8, 8))
        root.setBorder(BorderFactory.createEmptyBorder(12, 12, 12, 12))
        root.add(self.cards, BorderLayout.CENTER)
        log_scroll = JScrollPane(self.log_area)
        log_scroll.setBorder(BorderFactory.createTitledBorder("Diagnostic Log"))
        log_scroll.setPreferredSize(Dimension(720, 145))
        root.add(log_scroll, BorderLayout.SOUTH)
        self.frame.add(root)
        self.show_main()
        self.frame.pack()
        self.frame.setLocationRelativeTo(None)
        self.frame.setVisible(True)
        self.append_log("Operator calibration GUI ready")

    def append_log(self, message):
        if not SwingUtilities.isEventDispatchThread():
            text = str(message)
            gui_self = self

            class AppendTask(Runnable):
                def run(task_self):
                    gui_self.append_log(text)

            SwingUtilities.invokeLater(AppendTask())
            return
        self.log_area.append(str(message) + "\n")
        self.log_area.setCaretPosition(self.log_area.getDocument().getLength())

    def make_button(self, text, callback, primary=False):
        button = JButton(text)
        button.setFont(button.getFont().deriveFont(Font.BOLD, 17.0))
        button.setPreferredSize(Dimension(205, 54))
        button.addActionListener(lambda event: callback())
        if primary:
            button.setBackground(Color(55, 125, 205))
            button.setForeground(Color.WHITE)
        return button

    def make_abort_button(self):
        button = self.make_button("Abort", self.abort_action, False)
        button.setBackground(Color(190, 45, 45))
        button.setForeground(Color.WHITE)
        self.abort_btn = button
        return button

    def abort_action(self):
        if not self.busy or not self.abort_enabled.get():
            return
        if self.abort_requested.compareAndSet(False, True):
            if self.abort_btn is not None:
                self.abort_btn.setEnabled(False)
                self.abort_btn.setText("Abort Requested...")
            self.append_log(
                "Abort requested; stopping at the next calibration checkpoint")

    def calibration_log(self, message):
        if self.abort_requested.get():
            raise CalibrationAborted("Calibration aborted by operator")
        self.append_log(message)

    def show_screen(self, title, message, buttons, details=None, color=None,
                    above_buttons=None):
        panel = JPanel(BorderLayout(12, 18))
        panel.setBorder(BorderFactory.createEmptyBorder(35, 40, 25, 40))
        title_label = JLabel(title, SwingConstants.CENTER)
        title_label.setFont(title_label.getFont().deriveFont(Font.BOLD, 30.0))
        if color is not None:
            title_label.setForeground(color)
        panel.add(title_label, BorderLayout.NORTH)

        center = JPanel(BorderLayout(8, 12))
        message_label = JLabel(
            "<html><div style='text-align:center;width:600px'>%s</div></html>" %
            str(message), SwingConstants.CENTER)
        message_label.setFont(message_label.getFont().deriveFont(Font.PLAIN, 20.0))
        center.add(message_label, BorderLayout.NORTH)
        if details:
            detail_area = JTextArea(str(details))
            detail_area.setEditable(False)
            detail_area.setOpaque(False)
            detail_area.setFont(Font("Monospaced", Font.PLAIN, 16))
            detail_area.setBorder(BorderFactory.createEmptyBorder(16, 35, 5, 35))
            center.add(detail_area, BorderLayout.CENTER)
        panel.add(center, BorderLayout.CENTER)

        button_panel = JPanel(FlowLayout(FlowLayout.CENTER, 14, 4))
        for button in buttons:
            button_panel.add(button)
        if above_buttons:
            bottom_panel = JPanel(BorderLayout(0, 8))
            control_panel = JPanel(FlowLayout(FlowLayout.CENTER, 14, 0))
            for control in above_buttons:
                control_panel.add(control)
            bottom_panel.add(control_panel, BorderLayout.NORTH)
            bottom_panel.add(button_panel, BorderLayout.SOUTH)
            panel.add(bottom_panel, BorderLayout.SOUTH)
        else:
            panel.add(button_panel, BorderLayout.SOUTH)

        name = "screen_%d" % self.card_index
        self.card_index += 1
        self.cards.add(panel, name)
        self.card_layout.show(self.cards, name)
        self.cards.revalidate()
        self.cards.repaint()

    def show_main(self):
        self.current_flow = None
        needle_text = "Cal Syringe"
        lumen_text = "Cal Pick Head"
        if self.completed["needle"]:
            needle_text = "Cal Syringe  -  Complete"
        if self.completed["lumen"]:
            lumen_text = "Cal Pick Head  -  Complete"
        needle_btn = self.make_button(needle_text, self.start_cal_syringe, True)
        lumen_btn = self.make_button(lumen_text, self.start_cal_pick_head, True)
        needle_btn.setEnabled(not self.completed["needle"])
        lumen_btn.setEnabled(not self.completed["lumen"])
        close_btn = self.make_button("Close", self.request_close, False)
        self.show_screen(
            "Operator Calibration",
            "Select the calibration task you would like to perform.",
            [needle_btn, lumen_btn, close_btn])

    def run_async(self, name, work, done, failed=None):
        if self.busy:
            return
        self.busy = True
        self.append_log("Starting %s" % name)
        gui_self = self

        class Worker(SwingWorker):
            def doInBackground(worker_self):
                return work()

            def done(worker_self):
                gui_self.busy = False
                gui_self.abort_enabled.set(False)
                try:
                    result = worker_self.get()
                    done(result)
                except Exception as error:
                    gui_self.append_log("%s failed: %s" % (name, error))
                    if failed is not None:
                        failed(error)
                    else:
                        gui_self.show_error_screen(name, error, gui_self.current_flow)

        Worker().execute()

    def machine_call(self, function):
        machine_obj = require_machine_execute()

        class Task(Callable):
            def call(task_self):
                return function(machine_obj)

        return machine_obj.execute(Task())

    def start_cal_syringe(self):
        self.current_flow = "needle"
        self.enable_vac_dispense_on_next_run = False
        self.show_screen(
            "Moving to Load Position",
            "Please wait while the Dispense Head moves to the syringe load position.",
            [])
        self.run_async("Move to syringe load position", self.do_move_syringe_to_load,
                       self.load_move_done)

    def move_to_load_in_task(self, machine_obj):
        nozzle = find_nozzle(machine_obj, DISPENSE_NOZZLE_NAME)
        current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        target = Location(LengthUnit.Millimeters, LOAD_X_MM, LOAD_Y_MM,
                          LOAD_Z_MM, current.getRotation())
        needle_calibration.move_safe(nozzle, target)
        final_location = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
        return {"rotation_before": float(current.getRotation()),
                "rotation_after": float(final_location.getRotation())}

    def do_move_to_load(self):
        def task(machine_obj):
            return self.move_to_load_in_task(machine_obj)
        return self.machine_call(task)

    def read_boolean_actuator_state(self, actuator):
        # G-code Boolean actuators do not necessarily support a hardware read.
        # ReferenceActuator records the last successful actuation, which is the
        # authoritative state OpenPnP has available for this actuator.
        for method_name in ["getLastActuationValue", "isActuated", "getActuated",
                            "getState", "getValue"]:
            try:
                value = getattr(actuator, method_name)()
            except:
                continue
            if value is None:
                continue
            text = str(value).strip().lower()
            if text in ["true", "1", "on"]:
                return True
            if text in ["false", "0", "off"]:
                return False
        return None

    def do_move_syringe_to_load(self):
        def task(machine_obj):
            result = self.move_to_load_in_task(machine_obj)
            nozzle = find_nozzle(machine_obj, DISPENSE_NOZZLE_NAME)
            actuator = needle_calibration.find_named_actuator(
                machine_obj, VAC_DISPENSE_ACTUATOR_NAME, nozzle)
            result["syringe_vacuum_on"] = self.read_boolean_actuator_state(actuator)
            return result
        return self.machine_call(task)

    def load_move_done(self, result):
        self.append_log("Load move complete; rotation preserved at %.4f degrees" %
                        float(result.get("rotation_after", 0.0)))
        proceed_btn = self.make_button("Proceed", self.proceed_load_syringe, True)
        vacuum_btn = JRadioButton("Turn on syringe vac")
        vacuum_on = result["syringe_vacuum_on"]
        vacuum_btn.setSelected(vacuum_on is True)
        vacuum_btn.setFont(vacuum_btn.getFont().deriveFont(Font.PLAIN, 18.0))
        vacuum_btn.addActionListener(
            lambda event: self.toggle_syringe_vacuum(
                vacuum_btn.isSelected(), vacuum_btn, proceed_btn))
        if vacuum_on is None:
            self.append_log(
                "Syringe vacuum state is unavailable; toggle starts unchecked")
        else:
            self.append_log("Syringe vacuum is %s at load position" %
                            ("on" if vacuum_on else "off"))
        self.show_screen(
            "Load New Syringe",
            "Load the new syringe and needle.<br><br>"
            "Ensure the metal clamp is attached securely.",
            [proceed_btn, self.make_button("Back", self.show_main, False)],
            above_buttons=[vacuum_btn])

    def toggle_syringe_vacuum(self, enabled, vacuum_btn, proceed_btn):
        vacuum_btn.setEnabled(False)
        proceed_btn.setEnabled(False)

        def done(result):
            vacuum_btn.setEnabled(True)
            proceed_btn.setEnabled(True)

        def failed(error):
            vacuum_btn.setSelected(not enabled)
            vacuum_btn.setEnabled(True)
            proceed_btn.setEnabled(True)
            self.append_log("Syringe vacuum toggle failed: %s" % error)
            JOptionPane.showMessageDialog(
                self.frame,
                "VacDispenseActuator could not be turned %s:\n%s" %
                ("on" if enabled else "off", error),
                "Syringe Vacuum", JOptionPane.ERROR_MESSAGE)

        self.run_async(
            "Turn syringe vacuum %s" % ("on" if enabled else "off"),
            lambda: self.do_set_vac_dispense_actuator(enabled), done, failed)

    def proceed_load_syringe(self):
        self.enable_vac_dispense_on_next_run = True
        self.run_needle_calibration()

    def run_needle_calibration(self):
        if self.needle_pending_state and self.needle_pending_state.get(
                "temporary_z_active", False):
            self.reject_needle_then_retry()
            return
        self.current_flow = "needle"
        try:
            self.needle_cfg = needle_calibration.load_config(NEEDLE_CONFIG_PATH)
            configured = str(self.needle_cfg.get("nozzle_name", "") or "")
            if configured != DISPENSE_NOZZLE_NAME:
                raise Exception("Needle config nozzle_name must be '%s', found '%s'" %
                                (DISPENSE_NOZZLE_NAME, configured))
        except Exception as error:
            self.show_error_screen("Needle calibration setup", error, "needle")
            return
        self.abort_requested.set(False)
        self.abort_enabled.set(True)
        self.show_screen(
            "Running XYZ Calibration",
            "Running the XYZ calibration routine. Please wait.<br><br>"
            "The machine will move automatically.",
            [self.make_abort_button()])
        self.run_async("Needle XYZ calibration", self.do_needle_calibration,
                       self.needle_measure_done)

    def reject_needle_then_retry(self):
        if self.needle_before is None:
            self.needle_pending_state = None
            self.run_needle_calibration()
            return
        self.show_screen(
            "Discarding Previous XYZ Result",
            "Restoring the original Dispense Head offsets before retrying.", [])

        def done(result):
            if result is not None and result.get("ok", False):
                self.needle_pending_state = None
                self.append_log("Previous temporary XYZ calibration rejected")
                self.run_needle_calibration()
            else:
                self.show_error_screen(
                    "Cannot Retry XYZ Calibration",
                    (result or {}).get("message", "Original offsets were not restored"),
                    "needle")

        self.run_async("Reject previous Needle XYZ calibration",
                       self.do_reject_needle, done,
                       lambda error: self.show_error_screen(
                           "Cannot Retry XYZ Calibration", error, "needle"))

    def do_reject_needle(self):
        cfg = self.needle_cfg
        original = deep_copy(self.needle_before)

        def task(machine_obj):
            nozzle = find_nozzle(machine_obj, DISPENSE_NOZZLE_NAME)
            return needle_calibration.revert_offsets(
                nozzle, original, self.append_log, cfg)
        return self.machine_call(task)

    def actuate_dispense_solenoid(self, machine_obj, nozzle):
        try:
            actuator = needle_calibration.find_named_actuator(
                machine_obj, DISPENSE_SOLENOID_NAME, nozzle)
            try:
                actuator.actuate(True)
            except:
                actuator.setActuated(True)
            try:
                actuator.waitForCompletion(
                    CompletionType.WaitForUnconditionalCoordination)
            except:
                try:
                    nozzle.waitForCompletion(CompletionType.WaitForStillstand)
                except:
                    pass
            self.append_log("%s turned on" % DISPENSE_SOLENOID_NAME)
        except Exception as error:
            self.append_log(
                "WARNING: %s was not turned on: %s. Continuing calibration." %
                (DISPENSE_SOLENOID_NAME, error))

    def enable_vac_dispense_actuator(self, machine_obj, nozzle):
        self.set_vac_dispense_actuator(machine_obj, nozzle, True)

    def set_vac_dispense_actuator(self, machine_obj, nozzle, enabled):
        actuator = needle_calibration.find_named_actuator(
            machine_obj, VAC_DISPENSE_ACTUATOR_NAME, nozzle)
        parameter_types = Array.newInstance(Class, 1)
        parameter_types[0] = Boolean.TYPE
        method = actuator.getClass().getMethod("actuate", parameter_types)
        arguments = Array.newInstance(Object, 1)
        arguments[0] = Boolean.TRUE if enabled else Boolean.FALSE
        method.invoke(actuator, arguments)
        try:
            actuator.waitForCompletion(
                CompletionType.WaitForUnconditionalCoordination)
        except:
            nozzle.waitForCompletion(CompletionType.WaitForStillstand)
        self.append_log("%s turned %s" %
                        (VAC_DISPENSE_ACTUATOR_NAME, "on" if enabled else "off"))

    def do_set_vac_dispense_actuator(self, enabled):
        def task(machine_obj):
            nozzle = find_nozzle(machine_obj, DISPENSE_NOZZLE_NAME)
            self.set_vac_dispense_actuator(machine_obj, nozzle, enabled)
            return {"enabled": enabled}
        return self.machine_call(task)

    def do_needle_calibration(self):
        cfg = self.needle_cfg

        def task(machine_obj):
            nozzle = find_nozzle(machine_obj, DISPENSE_NOZZLE_NAME)
            if self.enable_vac_dispense_on_next_run:
                self.enable_vac_dispense_actuator(machine_obj, nozzle)
                self.enable_vac_dispense_on_next_run = False
            self.actuate_dispense_solenoid(machine_obj, nozzle)
            before = loc_to_dict(nozzle.getHeadOffsets())
            result = needle_calibration.run_full_measure(
                cfg, nozzle, machine_obj, self.calibration_log)
            if self.abort_requested.get():
                cleanup = needle_calibration.abort_nozzle_cleanup(
                    nozzle, cfg, self.append_log)
                restore_error = None
                try:
                    needle_calibration.restore_offsets_after_safe_retraction(
                        cfg, nozzle, before, machine_obj, self.append_log)
                    self.append_log(
                        "Abort complete; original Dispense Head offsets restored")
                except Exception as error:
                    restore_error = str(error)
                    self.append_log(
                        "Abort cleanup warning: offset restore failed: %s" % error)
                except Throwable as error:
                    restore_error = str(error)
                    self.append_log(
                        "Abort cleanup warning: offset restore failed: %s" % error)
                return {"aborted": True, "before": before,
                        "abort_cleanup": cleanup,
                        "restore_error": restore_error,
                        "offsets_restored": restore_error is None}
            return {"result": result, "before": before}
        return self.machine_call(task)

    def needle_measure_done(self, payload):
        if payload.get("aborted", False):
            self.abort_done("Needle XYZ", payload)
            return
        result = payload.get("result") or {}
        if not result.get("ok", False):
            state = result.get("calibration_state") or {}
            self.needle_pending_state = (deep_copy(state)
                if state.get("temporary_z_active", False) else None)
            self.show_error_screen(
                "XYZ Calibration Failed", result.get("message", "Unknown error"),
                self.current_flow)
            return
        self.needle_result = result
        self.needle_before = deep_copy(
            result.get("original_offsets") or payload.get("before"))
        state = result.get("calibration_state") or {}
        self.needle_pending_state = (deep_copy(state)
            if state.get("temporary_z_active", False) else None)
        proposal = result.get("proposal") or {}
        proposed = proposal.get("proposed_offsets") or {}
        delta = proposal.get("offset_delta") or {}
        details = (
            "Measured correction\n"
            "  dX: %+.6f mm\n"
            "  dY: %+.6f mm\n"
            "  dZ: %+.6f mm\n\n"
            "Proposed Dispense Head offsets\n"
            "   X: %.6f mm\n"
            "   Y: %.6f mm\n"
            "   Z: %.6f mm\n"
            "   R: %.6f deg" %
            (float(delta.get("x", 0.0)), float(delta.get("y", 0.0)),
             float(delta.get("z", 0.0)), float(proposed.get("x", 0.0)),
             float(proposed.get("y", 0.0)), float(proposed.get("z", 0.0)),
             float(proposed.get("rotation", 0.0))))
        if self.needle_pending_state and self.needle_pending_state.get(
                "temporary_z_active", False):
            details += "\n\nTemporary Z correction is active in memory and is not committed."
        self.show_screen(
            "XYZ Calibration Complete",
            "Review the measured correction before applying it.",
            [self.make_button("Apply", self.apply_needle, True),
             self.make_button("Try Again", self.run_needle_calibration, False)],
            details, Color(35, 125, 70))

    def apply_needle(self):
        self.show_screen("Applying XYZ Calibration",
                         "Checking and saving the accepted XYZ offsets.", [])
        self.run_async("Apply needle XYZ offsets", self.do_apply_needle,
                       lambda result: self.task_applied("needle"),
                       self.needle_apply_failed)

    def do_apply_needle(self):
        cfg = self.needle_cfg
        proposal = self.needle_result.get("proposal")

        def task(machine_obj):
            nozzle = find_nozzle(machine_obj, DISPENSE_NOZZLE_NAME)
            try:
                result = needle_calibration.apply_xyz(
                    cfg, nozzle, proposal, self.append_log)
                Configuration.get().save()
                return result
            except Exception as error:
                # Apply and persistence are one operator action. Do not leave
                # the temporary Z correction active when applying or saving
                # fails; restore the original snapshot after safe retraction.
                try:
                    needle_calibration.restore_offsets_after_safe_retraction(
                        cfg, nozzle, self.needle_before, machine_obj,
                        self.append_log)
                    self.needle_pending_state = None
                    self.append_log(
                        "Needle Apply failed; restored the original offsets")
                except Exception as restore_error:
                    raise Exception("%s; %s" % (error, restore_error))
                raise error
            except Throwable as error:
                try:
                    needle_calibration.restore_offsets_after_safe_retraction(
                        cfg, nozzle, self.needle_before, machine_obj,
                        self.append_log)
                    self.needle_pending_state = None
                except Throwable as restore_error:
                    raise Exception("%s; %s" % (error, restore_error))
                raise error
        return self.machine_call(task)

    def needle_apply_failed(self, error):
        self.show_screen(
            "Apply Blocked",
            "The XYZ offsets were not accepted or could not be saved.<br><br>%s" %
            str(error),
            [self.make_button("Review Results", self.needle_measure_done_again, True),
             self.make_button("Try Again", self.run_needle_calibration, False)],
            color=Color(180, 45, 45))

    def needle_measure_done_again(self):
        result = self.needle_result
        # A failed Apply/save path restores the original offsets. Re-display
        # the proposal without claiming that the temporary Z is still live.
        if result is not None and not self.needle_pending_state:
            result = deep_copy(result)
            state = result.get("calibration_state") or {}
            state["status"] = "reverted"
            state["temporary_z_active"] = False
            result["calibration_state"] = state
        self.needle_measure_done(
            {"result": result, "before": self.needle_before})

    def start_cal_pick_head(self):
        self.current_flow = "lumen"
        self.show_screen(
            "Moving to Load Position",
            "Please wait while the machine moves to the Pick Head load position.",
            [])
        self.run_async("Move to Pick Head load position", self.do_move_to_load,
                       self.pick_head_load_move_done)

    def pick_head_load_move_done(self, result):
        self.append_log("Pick Head load move complete; rotation preserved at %.4f degrees" %
                        float(result.get("rotation_after", 0.0)))
        self.show_screen(
            "Prepare Pick Head",
            "Ensure the Pick Head is ready for calibration.<br><br>"
            "When ready, select Proceed.",
            [self.make_button("Proceed", self.proceed_cal_pick_head, True),
             self.make_button("Back", self.show_main, False)])

    def proceed_cal_pick_head(self):
        self.current_flow = "lumen"
        try:
            self.lumen_cfg = read_json(LUMEN_CONFIG_PATH)
            configured = str(self.lumen_cfg.get("nozzle_name", "") or "")
            if configured != PICK_NOZZLE_NAME:
                raise Exception("Lumen config nozzle_name must be '%s', found '%s'" %
                                (PICK_NOZZLE_NAME, configured))
            self.lumen_config_snapshot = compact_json(self.lumen_cfg)
            self.lumen_baseline = None
        except Exception as error:
            self.show_error_screen("Pick Head calibration setup", error, "lumen")
            return
        self.run_lumen_attempt(True)

    def run_lumen_attempt(self, first_attempt=False):
        self.current_flow = "lumen"
        self.abort_requested.set(False)
        self.abort_enabled.set(True)
        self.show_screen(
            "Calibrating Pick Head",
            "Running the lumen calibration routine. Please wait.<br><br>"
            "The machine will move automatically.",
            [self.make_abort_button()])
        self.run_async(
            "Pick Head lumen calibration",
            lambda: self.do_lumen_calibration(first_attempt),
            self.lumen_measure_done)

    def restore_lumen_baseline_in_task(self, nozzle):
        if self.lumen_baseline is None:
            return
        current = loc_to_dict(nozzle.getHeadOffsets())
        if not offsets_match(current, self.lumen_baseline):
            tip_metrology.restore_head_offsets(nozzle, self.lumen_baseline)
            self.append_log("Restored Pick Head offsets to the first-attempt baseline")

    def do_lumen_calibration(self, first_attempt):
        cfg = self.lumen_cfg

        def task(machine_obj):
            nozzle = find_nozzle(machine_obj, PICK_NOZZLE_NAME)
            baseline = self.lumen_baseline
            if first_attempt or baseline is None:
                baseline = loc_to_dict(nozzle.getHeadOffsets())
            else:
                self.restore_lumen_baseline_in_task(nozzle)
            result = None
            restore_error = None
            try:
                result = tip_metrology.run_full_sequence(
                    cfg, nozzle=nozzle, machine_obj=machine_obj,
                    logger=self.calibration_log)
            except Exception as error:
                result = {"ok": False, "message": str(error)}
            if self.abort_requested.get():
                cleanup = self.cleanup_lumen_after_abort(nozzle)
                try:
                    current = loc_to_dict(nozzle.getHeadOffsets())
                    if not offsets_match(current, baseline):
                        tip_metrology.restore_head_offsets(nozzle, baseline)
                    self.append_log(
                        "Abort complete; original Pick Head offsets restored")
                except Exception as error:
                    restore_error = str(error)
                    self.append_log(
                        "Abort cleanup warning: offset restore failed: %s" % error)
                return {"aborted": True, "baseline": baseline,
                        "abort_cleanup": cleanup,
                        "restore_error": restore_error}
            if not result.get("ok", False):
                try:
                    current = loc_to_dict(nozzle.getHeadOffsets())
                    if not offsets_match(current, baseline):
                        tip_metrology.restore_head_offsets(nozzle, baseline)
                except Exception as error:
                    restore_error = str(error)
            return {"result": result, "baseline": baseline,
                    "restore_error": restore_error}
        return self.machine_call(task)

    def lumen_measure_done(self, payload):
        self.lumen_baseline = payload.get("baseline")
        if payload.get("aborted", False):
            self.lumen_pending = bool(payload.get("restore_error"))
            self.abort_done("Pick Head", payload)
            return
        result = payload.get("result") or {}
        if not result.get("ok", False):
            restore_error = payload.get("restore_error")
            self.lumen_pending = bool(restore_error)
            message = result.get("message", "Unknown error")
            if restore_error:
                message += "\n\nOriginal offset restoration also failed: %s" % restore_error
            self.show_error_screen("Pick Head Calibration Failed", message, "lumen")
            return

        self.lumen_result = result
        self.lumen_quality = calibration_quality_score(result)
        applied = (result.get("head_offsets") or {}).get("applied_head_offsets_mm")
        if applied is None:
            applied = (result.get("head_offset_solver") or {}).get(
                "proposed_head_offsets_mm")
        self.lumen_expected_offsets = applied
        self.lumen_pending = True
        solver = result.get("head_offset_solver") or {}
        delta = solver.get("delta_xy_mm") or {}
        reasons = self.lumen_quality.get("reasons") or []
        details = (
            "Calibration score: %d%% - %s\n\n"
            "Measured Pick Head correction\n"
            "  dX: %+.6f mm\n"
            "  dY: %+.6f mm\n\n"
            "Proposed Pick Head offsets\n"
            "   X: %.6f mm\n"
            "   Y: %.6f mm\n"
            "   Z: %.6f mm\n\n"
            "Quality notes: %s" %
            (int(self.lumen_quality.get("score", 0)),
             self.lumen_quality.get("band", ""),
             float(delta.get("x", 0.0)), float(delta.get("y", 0.0)),
             float(applied.get("x", 0.0)), float(applied.get("y", 0.0)),
             float(applied.get("z", 0.0)), "; ".join(reasons[:5])))
        self.show_screen(
            "Pick Head Calibration Complete",
            "Review the calibration quality before applying it.",
            [self.make_button("Apply", self.apply_lumen, True),
             self.make_button("Try Again", self.retry_lumen, False)],
            details, Color(35, 125, 70))

    def retry_lumen(self):
        self.run_lumen_attempt(False)

    def cleanup_lumen_after_abort(self, nozzle):
        errors = []

        def record_error(label, error):
            errors.append("%s failed: %s" % (label, error))
            self.append_log("Abort cleanup warning: %s" % errors[-1])

        try:
            vacuum = needle_calibration.get_vacuum_actuator(nozzle)
            if vacuum is None:
                raise Exception(
                    "No vacuum actuator is configured for the Pick Head")
            try:
                vacuum.actuate(False)
            except:
                vacuum.setActuated(False)
            try:
                vacuum.waitForCompletion(
                    CompletionType.WaitForUnconditionalCoordination)
            except:
                nozzle.waitForCompletion(CompletionType.WaitForStillstand)
            self.append_log("Abort cleanup: Pick Head vacuum off")
        except Throwable as error:
            record_error("vacuum off", error)
        except Exception as error:
            record_error("vacuum off", error)

        try:
            part = nozzle.getPart()
            if part is not None:
                nozzle.place()
                if nozzle.getPart() is not None:
                    raise Exception(
                        "Pick Head still reports a part after place()")
                self.append_log("Abort cleanup: Pick Head part state cleared")
            else:
                self.append_log(
                    "Abort cleanup: Pick Head already reports no part")
        except Throwable as error:
            record_error("part-state clear", error)
        except Exception as error:
            record_error("part-state clear", error)

        try:
            nozzle.moveToSafeZ()
            nozzle.waitForCompletion(CompletionType.WaitForStillstand)
            self.append_log("Abort cleanup: Pick Head moved to safe Z")
        except Throwable as error:
            record_error("safe-Z move", error)
        except Exception as error:
            record_error("safe-Z move", error)

        return {"ok": len(errors) == 0, "errors": errors}

    def abort_done(self, calibration_name, payload):
        cleanup = payload.get("abort_cleanup") or {}
        warnings = list(cleanup.get("errors") or [])
        restore_error = payload.get("restore_error")
        if restore_error:
            warnings.append("offset restore failed: %s" % restore_error)
        self.append_log("%s calibration aborted by operator" % calibration_name)
        if calibration_name == "Needle XYZ" and (
                restore_error or not payload.get("offsets_restored", False)):
            self.needle_before = deep_copy(payload.get("before"))
            self.needle_pending_state = {"status": "restore_failed",
                "original_offsets": deep_copy(self.needle_before),
                "temporary_z_active": True}
            self.show_error_screen(
                "SAFETY-CRITICAL Needle Abort Cleanup Failure",
                "The original Dispense Head offsets could not be verified. "
                "Do not continue until restoration succeeds.\n\n%s" %
                ("\n".join(warnings) or "Offset restoration failed"), "needle")
            return
        if warnings:
            JOptionPane.showMessageDialog(
                self.frame,
                "The calibration was aborted, but cleanup reported:\n\n%s" %
                "\n".join(warnings),
                "Abort Cleanup Warning", JOptionPane.WARNING_MESSAGE)
        self.show_main()

    def apply_lumen(self):
        score = int(self.lumen_quality.get("score", 0))
        if score < 95:
            answer = JOptionPane.showConfirmDialog(
                self.frame,
                "The Pick Head calibration score is %d%%, below the recommended "
                "95%%. These offsets may not be accurate.\n\n"
                "Do you still want to apply and save them?" % score,
                "Confirm Low-Score Calibration",
                JOptionPane.YES_NO_OPTION,
                JOptionPane.WARNING_MESSAGE)
            if answer != JOptionPane.YES_OPTION:
                self.append_log("Low-score Pick Head apply cancelled by operator")
                return
        self.show_screen("Applying Pick Head Calibration",
                         "Checking and saving the accepted Pick Head calibration.", [])
        self.run_async("Apply Pick Head calibration", self.do_apply_lumen,
                       lambda result: self.task_applied("lumen"),
                       self.lumen_apply_failed)

    def do_apply_lumen(self):
        current_cfg = read_json(LUMEN_CONFIG_PATH)
        if compact_json(current_cfg) != self.lumen_config_snapshot:
            raise Exception("Lumen configuration changed since calibration; apply blocked")

        def task(machine_obj):
            nozzle = find_nozzle(machine_obj, PICK_NOZZLE_NAME)
            current = loc_to_dict(nozzle.getHeadOffsets())
            if not offsets_match(current, self.lumen_expected_offsets):
                raise Exception(
                    "Pick Head offsets changed since calibration; apply blocked")
            Configuration.get().save()
            return {"ok": True}
        return self.machine_call(task)

    def lumen_apply_failed(self, error):
        self.show_screen(
            "Apply Blocked",
            "The Pick Head calibration could not be saved.<br><br>%s" % str(error),
            [self.make_button("Review Results", self.lumen_measure_done_again, True),
             self.make_button("Try Again", self.retry_lumen, False)],
            color=Color(180, 45, 45))

    def lumen_measure_done_again(self):
        self.lumen_measure_done(
            {"result": self.lumen_result, "baseline": self.lumen_baseline,
             "restore_error": None})

    def show_error_screen(self, title, error, flow):
        self.current_flow = flow
        self.last_error = str(error)
        if flow == "needle":
            retry = self.run_needle_calibration
        elif flow == "lumen":
            retry = (self.retry_lumen if self.lumen_baseline is not None
                     else self.proceed_cal_pick_head)
        else:
            retry = self.show_main
        self.show_screen(
            title,
            "The operation did not complete.<br><br>%s" % str(error),
            [self.make_button("Try Again", retry, True),
             self.make_button("Back to Main", self.back_to_main, False)],
            color=Color(180, 45, 45))

    def back_to_main(self):
        if self.needle_pending_state and self.needle_pending_state.get(
                "temporary_z_active", False):
            self.restore_needle_then(self.back_to_main)
        elif self.lumen_pending:
            self.restore_lumen_then(self.show_main)
        else:
            self.show_main()

    def restore_needle_then(self, callback):
        if not (self.needle_pending_state and
                self.needle_pending_state.get("temporary_z_active", False)):
            callback()
            return
        if self.needle_before is None:
            self.show_error_screen("Needle Restore Failed",
                "The original Dispense Head offsets are unavailable.", "needle")
            return
        self.show_screen("Restoring Dispense Head",
                         "Restoring the unaccepted XYZ offsets. Please wait.", [])

        def done(result):
            if result is not None and result.get("ok", False):
                self.needle_pending_state = None
                callback()
            else:
                self.show_error_screen(
                    "Needle Restore Failed",
                    (result or {}).get("message", "Original offsets were not restored"),
                    "needle")

        self.run_async("Restore Needle XYZ baseline", self.do_reject_needle,
                       done, lambda error: self.show_error_screen(
                           "Needle Restore Failed", error, "needle"))

    def task_applied(self, task_name):
        self.completed[task_name] = True
        if task_name == "lumen":
            self.lumen_pending = False
        elif task_name == "needle":
            self.needle_pending_state = None
        self.append_log("%s calibration applied and saved" %
                        ("Pick Head" if task_name == "lumen" else "Needle XYZ"))
        if self.completed["needle"] and self.completed["lumen"]:
            self.show_thank_you()
        else:
            self.show_main()

    def show_thank_you(self):
        self.show_screen(
            "Calibration Complete",
            "Thank you. Both calibration routines were completed successfully.<br><br>"
            "This window will close automatically.", [], color=Color(35, 125, 70))
        gui_self = self
        timer = Timer(3000, lambda event: gui_self.frame.dispose())
        timer.setRepeats(False)
        timer.start()

    def restore_lumen_then(self, callback):
        if not self.lumen_pending or self.lumen_baseline is None:
            callback()
            return
        self.show_screen("Restoring Pick Head",
                         "Restoring the unaccepted Pick Head offsets. Please wait.", [])

        def restore():
            def task(machine_obj):
                nozzle = find_nozzle(machine_obj, PICK_NOZZLE_NAME)
                self.restore_lumen_baseline_in_task(nozzle)
                return {"ok": True}
            return self.machine_call(task)

        def done(result):
            self.lumen_pending = False
            callback()

        def failed(error):
            JOptionPane.showMessageDialog(
                self.frame,
                "The original Pick Head offsets could not be restored:\n%s\n\n"
                "The operator window will remain open." % error,
                "Restore Failed", JOptionPane.ERROR_MESSAGE)
            self.show_error_screen("Pick Head Restore Failed", error, "lumen")

        self.run_async("Restore Pick Head baseline", restore, done, failed)

    def request_close(self):
        if self.busy:
            JOptionPane.showMessageDialog(
                self.frame,
                "A machine operation is still running. Wait for it to finish before closing.",
                "Operation In Progress", JOptionPane.INFORMATION_MESSAGE)
            return
        if self.needle_pending_state and self.needle_pending_state.get(
                "temporary_z_active", False):
            self.restore_needle_then(self.request_close)
        elif self.lumen_pending:
            self.restore_lumen_then(lambda: self.frame.dispose())
        else:
            self.frame.dispose()


def start():
    gui_obj = OperatorCalibrationGui()
    gui_obj.build()
    return gui_obj


class StartRunnable(Runnable):
    def run(self):
        start()


try:
    SwingUtilities.invokeLater(StartRunnable())
except:
    start()
