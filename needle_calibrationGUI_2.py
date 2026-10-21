# needle_calibrationGUI_2.py
#
# Simple explicit-apply GUI for needle_calibration2.py.

import imp
import json
import os
import sys

from java.awt import BorderLayout, Color, Dimension, FlowLayout, Font, GridBagConstraints, GridBagLayout, Insets
from java.awt.event import WindowAdapter
from java.lang import Runnable, Throwable
from java.util.concurrent import Callable
from java.util.concurrent.atomic import AtomicBoolean
from javax.swing import (
    BorderFactory, Box, BoxLayout, JButton, JComboBox, JFrame, JLabel,
    JOptionPane, JPanel, JScrollPane, JSplitPane, JTextArea, JTextField,
    SwingUtilities, SwingWorker
)
from javax.swing.event import DocumentListener
from org.openpnp.model import Configuration, LengthUnit, Location


try:
    unicode
except NameError:
    unicode = str
try:
    long
except NameError:
    long = int


def existing_dir(path):
    try:
        if path and os.path.isdir(str(path)):
            return os.path.abspath(str(path))
    except:
        pass
    return None


def candidate_script_dirs():
    out = []
    try:
        out.append(scripting.getScriptsDirectory().toString())
    except:
        pass
    try:
        out.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    except:
        pass
    try:
        out.append(os.path.join(os.path.expanduser("~"), ".openpnp2", "scripts"))
    except:
        pass
    out.append(os.getcwd())
    seen = {}
    unique = []
    for item in out:
        path = existing_dir(item)
        if path is not None and path not in seen:
            seen[path] = True
            unique.append(path)
    return unique


def resolve_scripts_root():
    for path in candidate_script_dirs():
        backend = os.path.join(path, "Events", "Homing", "needle_calibration2.py")
        if os.path.exists(backend):
            return path
    candidates = candidate_script_dirs()
    if len(candidates):
        return candidates[0]
    return os.getcwd()


SCRIPT_DIR = resolve_scripts_root()
HOMING_DIR = os.path.join(SCRIPT_DIR, "Events", "Homing")
if HOMING_DIR not in sys.path:
    sys.path.append(HOMING_DIR)
BACKEND_PATH = os.path.join(HOMING_DIR, "needle_calibration2.py")
needle_calibration = imp.load_source("needle_calibration2_backend", BACKEND_PATH)
CONFIG_PATH = os.path.join(SCRIPT_DIR, "needle_calibration_config.json")

IDLE = "IDLE"
RUNNING = "RUNNING"
READY = "READY"
ERROR = "ERROR"
SAVED = "SAVED"


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


def deep_copy(value):
    return json.loads(json.dumps(value, sort_keys=True))


def compact_json(data):
    return json.dumps(data, sort_keys=True, separators=(",", ":"))


def java_list_to_py(model):
    return needle_calibration.java_list_to_py(model)


def object_name(obj):
    return needle_calibration.object_name(obj)


def unique_sorted(values):
    seen = {}
    out = []
    for value in values:
        text = str(value or "").strip()
        if text and text not in seen:
            seen[text] = True
            out.append(text)
    out.sort()
    return out


def available_nozzle_names():
    names = []
    for head in needle_calibration.all_heads(get_machine()):
        try:
            for nozzle in java_list_to_py(head.getNozzles()):
                names.append(object_name(nozzle))
        except:
            pass
    return unique_sorted(names)


def available_camera_names():
    machine_obj = get_machine()
    names = []
    for head in needle_calibration.all_heads(machine_obj):
        try:
            for cam in java_list_to_py(head.getCameras()):
                names.append(object_name(cam))
        except:
            pass
    try:
        for cam in java_list_to_py(machine_obj.getCameras()):
            names.append(object_name(cam))
    except:
        pass
    return unique_sorted(names)


def available_actuator_names():
    machine_obj = get_machine()
    owners = [machine_obj]
    for head in needle_calibration.all_heads(machine_obj):
        owners.append(head)
    names = []
    for owner in owners:
        try:
            for actuator in java_list_to_py(owner.getActuators()):
                names.append(object_name(actuator))
        except:
            pass
    return unique_sorted(names)


def set_combo_items(combo, values, selected):
    current = str(selected or "")
    combo.removeAllItems()
    if current:
        combo.addItem(current)
    for value in values:
        text = str(value or "")
        if text and text != current:
            combo.addItem(text)
    combo.setSelectedItem(current)


def combo_text(combo):
    item = combo.getSelectedItem()
    return "" if item is None else str(item).strip()


def mm_location_from_dict(data):
    return Location(LengthUnit.Millimeters,
        float(data.get("x", 0.0)), float(data.get("y", 0.0)),
        float(data.get("z", 0.0)), float(data.get("rotation", 0.0)))


def json_safe(value):
    if value is None:
        return None
    if isinstance(value, (str, unicode, int, long, float, bool)):
        return value
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    if isinstance(value, tuple):
        return [json_safe(v) for v in value]
    if isinstance(value, dict):
        out = {}
        for key in value:
            if str(key) == "image":
                out[str(key)] = "<image>"
            else:
                out[str(key)] = json_safe(value[key])
        return out
    try:
        if hasattr(value, "convertToUnits"):
            return needle_calibration.loc_to_dict(value)
    except:
        pass
    return str(value)


class FieldChangeListener(DocumentListener):
    def __init__(self, callback):
        self.callback = callback

    def insertUpdate(self, event):
        self.callback()

    def removeUpdate(self, event):
        self.callback()

    def changedUpdate(self, event):
        self.callback()


class NeedleCalibrationGui2(object):
    def __init__(self):
        self.frame = JFrame("Needle Calibration 2")
        self.cfg = needle_calibration.load_config(CONFIG_PATH)
        self.saved_snapshot = compact_json(needle_calibration.public_clean_config(self.cfg))
        self.loading_ui = False
        self.worker_running = False
        self.abort_requested = AtomicBoolean(False)
        self.abort_enabled = AtomicBoolean(False)
        self.pre_run_state = IDLE
        self.state = IDLE
        self.selector_combos = {}
        self.z_fields = {}
        self.z_probe_fields = {}
        self.z_combos = {}
        self.z_rows = {}
        self.post_fields = {}
        self.post_combos = {}
        self.post_rows = {}
        self.motion_fields = {}
        self.action_buttons = []
        self.last_result = None
        self.last_z_proposal = None
        self.last_xy_proposal = None
        self.last_xyz_proposal = None
        self.original_offsets_snapshot = None
        self.pending_calibration_state = None
        self.revert_offsets_snapshot = None

        self.status_area = JTextArea()
        self.offsets_area = JTextArea()
        self.measurement_results_area = JTextArea()
        self.measurement_results_panel = None
        self.right_top_panel = None
        self.result_area = JTextArea()
        self.log_area = JTextArea()
        self.state_label = JLabel("")
        self.dirty_label = JLabel("")

    def build(self):
        self.frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE)
        self.frame.addWindowListener(self.CloseListener(self))
        self.frame.setMinimumSize(Dimension(1120, 760))
        root = JPanel(BorderLayout(8, 8))
        root.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8))
        root.add(self.build_top(), BorderLayout.NORTH)
        left = JScrollPane(self.build_form())
        left.setPreferredSize(Dimension(540, 640))
        right = self.build_right()
        split = JSplitPane(JSplitPane.HORIZONTAL_SPLIT, left, right)
        split.setContinuousLayout(True)
        split.setResizeWeight(0.46)
        split.setDividerLocation(540)
        root.add(split, BorderLayout.CENTER)
        root.add(self.build_log_panel(), BorderLayout.SOUTH)
        self.frame.add(root, BorderLayout.CENTER)
        self.load_into_ui(self.cfg)
        self.refresh_selectors()
        self.refresh_current_values()
        self.set_state(IDLE)
        self.frame.pack()
        self.frame.setLocationRelativeTo(None)
        self.frame.setVisible(True)

    class CloseListener(WindowAdapter):
        def __init__(self, gui_obj):
            self.gui_obj = gui_obj

        def windowClosing(self, event):
            self.gui_obj.request_close()

    def build_top(self):
        panel = JPanel(BorderLayout(6, 4))
        self.configure_text_area(self.status_area, 2)
        panel.add(self.status_area, BorderLayout.NORTH)
        panel.add(self.build_buttons(), BorderLayout.SOUTH)
        return panel

    def build_buttons(self):
        panel = JPanel(FlowLayout(FlowLayout.LEFT, 5, 2))
        specs = [
            ("refresh_lists_btn", "Refresh Lists", lambda e: self.refresh_selectors()),
            ("test_z_btn", "Test Z Pipeline", lambda e: self.test_pipeline_action("z_touch")),
            ("test_post_btn", "Test Post Pipeline", lambda e: self.test_pipeline_action("post_xy")),
            ("measure_z_btn", "Measure Z", lambda e: self.measure_action("z")),
            ("measure_post_btn", "Measure Post XY", lambda e: self.measure_action("post_xy")),
            ("measure_full_btn", "Measure Full XYZ", lambda e: self.measure_action("full")),
            ("abort_btn", "Abort", lambda e: self.abort_action()),
            ("apply_z_btn", "Apply Z", lambda e: self.apply_action("z")),
            ("apply_xy_btn", "Apply XY", lambda e: self.apply_action("xy")),
            ("apply_xyz_btn", "Apply XYZ", lambda e: self.apply_action("xyz")),
            ("revert_btn", "Revert Offsets", lambda e: self.revert_action()),
            ("save_json_btn", "Save JSON Config", lambda e: self.save_config_action()),
            ("save_machine_btn", "Save OpenPnP Config", lambda e: self.save_machine_action())
        ]
        for name, label, callback in specs:
            button = JButton(label)
            button.addActionListener(callback)
            setattr(self, name, button)
            if name != "abort_btn":
                self.action_buttons.append(button)
            panel.add(button)
        self.abort_btn.setBackground(Color(190, 45, 45))
        self.abort_btn.setForeground(Color.WHITE)
        self.abort_btn.setFont(self.abort_btn.getFont().deriveFont(Font.BOLD))
        self.abort_btn.setVisible(False)
        return panel

    def build_form(self):
        panel = JPanel()
        panel.setLayout(BoxLayout(panel, BoxLayout.Y_AXIS))
        panel.add(self.build_selectors())
        panel.add(Box.createVerticalStrut(6))
        panel.add(self.build_z_section())
        panel.add(Box.createVerticalStrut(6))
        panel.add(self.build_post_section())
        panel.add(Box.createVerticalStrut(6))
        panel.add(self.build_motion_section())
        panel.add(Box.createVerticalGlue())
        return panel

    def build_selectors(self):
        panel = self.section("OpenPnP Objects")
        specs = [("nozzle_name", "Nozzle"), ("top_camera_name", "Top Camera")]
        for row, spec in enumerate(specs):
            key, label = spec
            combo = JComboBox()
            combo.setEditable(True)
            combo.setPreferredSize(Dimension(300, combo.getPreferredSize().height))
            combo.addActionListener(lambda event: self.on_field_changed())
            self.selector_combos[key] = combo
            self.add_row(panel, row, label, combo)
        return panel

    def build_z_section(self):
        panel = self.section("Z Touch")
        specs = [("x", "X mm"), ("y", "Y mm"), ("z", "Surface Z mm"),
            ("rotation", "Rotation"), ("trigger_offset_mm", "Z Trigger Offset mm"),
            ("result_stage", "Result Stage")]
        for row, spec in enumerate(specs):
            key, label = spec
            field = JTextField(12)
            self.z_fields[key] = field
            self.watch_field(field)
            self.add_row(panel, row, label, field)
        row = len(specs)
        acquisition = JComboBox()
        acquisition.addItem("polling")
        acquisition.addItem("hardware_interrupt")
        acquisition.addActionListener(lambda event: self.on_mode_changed())
        self.z_combos["acquisition_method"] = acquisition
        self.add_row(panel, row, "Touch Method", acquisition)
        row += 1
        actuator = JComboBox()
        actuator.setEditable(True)
        actuator.setPreferredSize(Dimension(300, actuator.getPreferredSize().height))
        actuator.addActionListener(lambda event: self.on_field_changed())
        self.z_combos["actuator_name"] = actuator
        self.z_rows["actuator_name"] = self.add_row(panel, row,
            "Z Read / Driver Actuator", actuator)
        row += 1
        probe_mode = JComboBox()
        probe_mode.addItem("actuator")
        probe_mode.addItem("direct_gcode")
        probe_mode.addActionListener(lambda event: self.on_mode_changed())
        self.z_combos["probe_mode"] = probe_mode
        self.z_rows["probe_mode"] = self.add_row(panel, row,
            "Hardware Probe Mode", probe_mode)
        row += 1
        sequence_actuator = JComboBox()
        sequence_actuator.setEditable(True)
        sequence_actuator.setPreferredSize(Dimension(300,
            sequence_actuator.getPreferredSize().height))
        sequence_actuator.addActionListener(lambda event: self.on_field_changed())
        self.z_combos["sequence_actuator_name"] = sequence_actuator
        self.z_rows["sequence_actuator_name"] = self.add_row(panel, row,
            "Sequence Actuator", sequence_actuator)
        row += 1
        channel = JComboBox()
        channel.addItem("Q0 - Voron")
        channel.addItem("Q1 - Contact")
        channel.addActionListener(lambda event: self.on_field_changed())
        self.z_combos["channel"] = channel
        self.z_rows["channel"] = self.add_row(panel, row, "Mux Channel", channel)
        row += 1

        probe_specs = [
            ("start_above_mm", "Z Start Above mm"),
            ("max_descent_mm", "Z Max Descent mm"),
            ("coarse_feed", "Direct Z Coarse F"),
            ("fine_feed", "Direct Z Fine F"),
            ("fine_repeats", "Direct Z Fine Repeats"),
            ("probe_acceleration", "Direct M204 S"),
            ("coarse_step_mm", "Z Coarse Step mm"),
            ("fine_step_mm", "Z Fine Step mm"),
            ("retract_after_contact_mm", "Z Retract After Contact mm"),
            ("release_retract_step_mm", "Release Retract Step mm"),
            ("max_release_retract_mm", "Max Release Retract mm"),
            ("read_settle_ms", "Read Settle ms"),
            ("debounce_count", "Debounce Count"),
            ("debounce_retries", "Debounce Retries"),
            ("debounce_interval_ms", "Debounce Interval ms"),
            ("polling_coarse_repeats", "Polling Coarse Tries"),
            ("polling_fine_repeats", "Polling Fine Tries")]
        for key, label in probe_specs:
            field = JTextField(12)
            self.z_probe_fields[key] = field
            self.watch_field(field)
            self.z_rows[key] = self.add_row(panel, row, label, field)
            row += 1
        mode = JComboBox()
        mode.addItem("named")
        mode.addItem("xml_path")
        mode.addActionListener(lambda event: self.on_field_changed())
        self.z_combos["pipeline_mode"] = mode
        self.add_row(panel, row, "Pipeline Mode", mode)
        row += 1
        source = JComboBox()
        source.setEditable(True)
        source.setPreferredSize(Dimension(300, source.getPreferredSize().height))
        source.addActionListener(lambda event: self.on_field_changed())
        self.z_combos["pipeline_source"] = source
        self.add_row(panel, row, "Pipeline Name / Path", source)
        row += 1

        buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 0))
        capture = JButton("Capture Camera DRO")
        go = JButton("Go Camera")
        capture.addActionListener(lambda event: self.capture_camera_location("z_touch"))
        go.addActionListener(lambda event: self.go_camera_location("z_touch"))
        self.action_buttons.append(capture)
        self.action_buttons.append(go)
        buttons.add(capture)
        buttons.add(go)
        panel.add(buttons, self.full_width_gbc(row))
        return panel

    def build_post_section(self):
        panel = self.section("Post XY")
        specs = [
            ("x", "X mm"), ("y", "Y mm"), ("z", "Camera Z mm"),
            ("rotation", "Rotation"), ("result_stage", "Result Stage"),
            ("post_diameter_mm", "Post Diameter mm"),
            ("angles_deg", "Angles deg"),
            ("side_probe_z_offset_mm", "Side Probe Z Offset mm"),
            ("scan_clearance_mm", "Scan Clearance mm"),
            ("scan_overtravel_mm", "Scan Overtravel mm",
                "extra distance past expected edge to keep searching"),
            ("max_radial_probe_travel_mm", "Max Radial Probe Travel mm",
                "hard cap on inward travel"),
            ("step_mm", "Step mm"),
            ("speed", "Probe Speed"),
            ("confirm_backoff_mm", "Confirm Backoff mm"),
            ("confirm_reapproach_mm", "Confirm Reapproach Max mm"),
            ("confirm_step_mm", "Confirm Step mm"),
            ("confirm_position_tolerance_mm", "Confirm Position Tolerance mm"),
            ("confirm_reject_early", "Reject Early Confirmation"),
            ("read_settle_ms", "Read Settle ms"),
            ("debounce_count", "Contact Debounce Count"),
            ("debounce_retries", "Contact Debounce Retries"),
            ("debounce_interval_ms", "Contact Debounce Interval ms"),
            ("allow_single_read_contact", "Allow Single Read Contact"),
            ("min_valid_contacts_for_fit", "Min Valid Contacts For Fit"),
            ("local_lift_mm", "Local Lift mm"),
            ("outside_recover_step_mm", "Outside Recover Step"),
            ("outside_recover_max_mm", "Outside Recover Max"),
            ("max_fit_rms_mm", "Circle Fit RMS Warning mm",
                "diagnostic warning limit; only blocks calibration if circle-center fallback is used"),
            ("max_fit_error_mm", "Circle Fit Max Error Warning mm",
                "diagnostic warning limit; only blocks calibration if circle-center fallback is used"),
            ("max_center_error_mm", "Max Center Error mm"),
            ("log_every_step", "Log Every Step"),
            ("log_step_interval", "Log Step Interval"),
            ("hardware_probe_acceleration", "Direct M204 S"),
            ("hardware_coarse_feed", "XY Coarse F"),
            ("hardware_fine_feed", "XY Fine F"),
            ("hardware_coarse_repeats", "XY Coarse Repeats"),
            ("hardware_fine_repeats", "XY Fine Repeats"),
            ("hardware_fine_retry_attempts", "XY Fine Invalid Retries",
                "extra discarded fine attempts allowed while collecting the requested valid repeats"),
            ("hardware_g38_attempts", "XY G38 Attempts"),
            ("hardware_start_radius_mm", "Start Radius From Center mm",
                "absolute center-to-nozzle distance where each coarse G38 begins"),
            ("hardware_coarse_max_travel_mm", "Coarse Max Inward Travel mm",
                "maximum inward distance commanded by each coarse G38"),
            ("hardware_min_coarse_travel_mm", "Coarse Minimum Valid Travel mm",
                "abort before fine probing if G38 reports less inward travel"),
            ("hardware_fine_retract_mm", "Fine Retract From Contact mm",
                "outward distance from the averaged coarse contact before each fine G38"),
            ("hardware_fine_max_travel_mm", "Fine Max Inward Travel mm",
                "maximum inward distance commanded by each fine G38"),
            ("hardware_repeat_tolerance_mm", "Fine Repeat Tolerance mm",
                "maximum travel spread among the fine contacts selected for averaging")
        ]
        for row, spec in enumerate(specs):
            key = spec[0]
            label = spec[1]
            field = JTextField(12)
            if len(spec) > 2:
                field.setToolTipText(spec[2])
            self.post_fields[key] = field
            self.watch_field(field)
            self.post_rows[key] = self.add_row(panel, row, label, field)
        row = len(specs)
        acquisition = JComboBox()
        acquisition.addItem("polling")
        acquisition.addItem("hardware_interrupt")
        acquisition.addActionListener(lambda event: self.on_mode_changed())
        self.post_combos["acquisition_method"] = acquisition
        self.add_row(panel, row, "Touch Method", acquisition)
        row += 1
        actuator = JComboBox()
        actuator.setEditable(True)
        actuator.setPreferredSize(Dimension(300, actuator.getPreferredSize().height))
        actuator.addActionListener(lambda event: self.on_field_changed())
        self.post_combos["actuator_name"] = actuator
        self.add_row(panel, row, "Post Read / Driver Actuator", actuator)
        row += 1
        channel = JComboBox()
        channel.addItem("Q0 - Voron")
        channel.addItem("Q1 - Contact")
        channel.addActionListener(lambda event: self.on_field_changed())
        self.post_combos["hardware_channel"] = channel
        self.post_rows["hardware_channel"] = self.add_row(panel, row,
            "Mux Channel", channel)
        row += 1
        mode = JComboBox()
        mode.addItem("named")
        mode.addItem("xml_path")
        mode.addActionListener(lambda event: self.on_field_changed())
        self.post_combos["pipeline_mode"] = mode
        self.add_row(panel, row, "Pipeline Mode", mode)
        row += 1
        source = JComboBox()
        source.setEditable(True)
        source.setPreferredSize(Dimension(300, source.getPreferredSize().height))
        source.addActionListener(lambda event: self.on_field_changed())
        self.post_combos["pipeline_source"] = source
        self.add_row(panel, row, "Pipeline Name / Path", source)
        row += 1

        buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 0))
        capture = JButton("Capture Camera DRO")
        go = JButton("Go Camera")
        capture.addActionListener(lambda event: self.capture_camera_location("post_xy"))
        go.addActionListener(lambda event: self.go_camera_location("post_xy"))
        self.action_buttons.append(capture)
        self.action_buttons.append(go)
        buttons.add(capture)
        buttons.add(go)
        panel.add(buttons, self.full_width_gbc(row))
        return panel

    def build_motion_section(self):
        panel = self.section("Motion / Signs")
        specs = [
            ("safe_z_mm", "Safe Z mm"),
            ("travel_speed", "Travel Speed"),
            ("probe_speed", "Probe Speed"),
            ("fine_probe_speed", "Fine Probe Speed"),
            ("xy_offset_sign", "XY Offset Sign"),
            ("z_offset_sign", "Z Offset Sign"),
            ("max_apply_delta_mm", "Max Apply Delta mm")
        ]
        for row, spec in enumerate(specs):
            key, label = spec
            field = JTextField(10)
            self.motion_fields[key] = field
            self.watch_field(field)
            self.add_row(panel, row, label, field)
        return panel

    def build_right(self):
        panel = JPanel(BorderLayout(6, 6))
        top = JPanel()
        top.setLayout(BoxLayout(top, BoxLayout.Y_AXIS))
        self.right_top_panel = top
        current = JPanel(GridBagLayout())
        current.setBorder(BorderFactory.createTitledBorder("Current"))
        current.setAlignmentX(JPanel.LEFT_ALIGNMENT)
        self.configure_text_area(self.offsets_area, 3)
        self.add_row(current, 0, "Nozzle Offsets", self.offsets_area)
        self.add_row(current, 1, "State", self.state_label)
        self.add_row(current, 2, "JSON Dirty", self.dirty_label)
        current.setMaximumSize(Dimension(32767, current.getPreferredSize().height))
        top.add(current)
        top.add(Box.createVerticalStrut(6))
        results = JPanel(GridBagLayout())
        results.setBorder(BorderFactory.createTitledBorder("Results"))
        results.setAlignmentX(JPanel.LEFT_ALIGNMENT)
        self.configure_text_area(self.measurement_results_area, 5)
        self.add_row(results, 0, "Offset Proposal", self.measurement_results_area)
        results.setMaximumSize(Dimension(32767, results.getPreferredSize().height))
        results.setVisible(False)
        self.measurement_results_panel = results
        top.add(results)
        panel.add(top, BorderLayout.NORTH)
        self.result_area.setEditable(False)
        self.result_area.setLineWrap(False)
        scroll = JScrollPane(self.result_area)
        scroll.setBorder(BorderFactory.createTitledBorder("Summary / Full JSON Result"))
        panel.add(scroll, BorderLayout.CENTER)
        return panel

    def build_log_panel(self):
        panel = JPanel(BorderLayout())
        panel.setBorder(BorderFactory.createTitledBorder("Log"))
        self.log_area.setEditable(False)
        self.log_area.setRows(7)
        panel.add(JScrollPane(self.log_area), BorderLayout.CENTER)
        return panel

    def section(self, title):
        panel = JPanel(GridBagLayout())
        panel.setBorder(BorderFactory.createTitledBorder(title))
        panel.setAlignmentX(JPanel.LEFT_ALIGNMENT)
        panel.setMaximumSize(Dimension(32767, 32767))
        return panel

    def add_row(self, panel, row, label, component):
        label_obj = JLabel(label)
        label_obj.setPreferredSize(Dimension(210, label_obj.getPreferredSize().height))
        gbc = GridBagConstraints()
        gbc.gridx = 0
        gbc.gridy = row
        gbc.insets = Insets(2, 2, 2, 8)
        gbc.anchor = GridBagConstraints.WEST
        panel.add(label_obj, gbc)
        gbc = GridBagConstraints()
        gbc.gridx = 1
        gbc.gridy = row
        gbc.insets = Insets(2, 2, 2, 2)
        gbc.anchor = GridBagConstraints.WEST
        gbc.fill = GridBagConstraints.HORIZONTAL
        gbc.weightx = 1.0
        panel.add(component, gbc)
        return (label_obj, component)

    def full_width_gbc(self, row):
        gbc = GridBagConstraints()
        gbc.gridx = 0
        gbc.gridy = row
        gbc.gridwidth = 2
        gbc.insets = Insets(2, 2, 4, 2)
        gbc.fill = GridBagConstraints.HORIZONTAL
        gbc.weightx = 1.0
        return gbc

    def configure_text_area(self, area, rows):
        area.setEditable(False)
        area.setLineWrap(True)
        area.setWrapStyleWord(True)
        area.setRows(rows)
        area.setOpaque(False)

    def watch_field(self, field):
        field.getDocument().addDocumentListener(FieldChangeListener(self.on_field_changed))

    def append_log(self, msg):
        if not SwingUtilities.isEventDispatchThread():
            text = str(msg)
            gui_self = self
            class AppendTask(Runnable):
                def run(task_self):
                    gui_self.append_log(text)
            SwingUtilities.invokeLater(AppendTask())
            return
        self.log_area.append(str(msg) + "\n")
        self.log_area.setCaretPosition(self.log_area.getDocument().getLength())

    def show_error(self, title, err):
        self.append_log("%s: %s" % (title, err))
        JOptionPane.showMessageDialog(self.frame, str(err), title, JOptionPane.ERROR_MESSAGE)

    def has_pending_temporary_z(self):
        state = self.pending_calibration_state or {}
        return bool(state.get("temporary_z_active", False))

    def set_state(self, state):
        self.state = state
        self.state_label.setText(state)
        self.update_dirty()
        self.update_buttons()

    def update_buttons(self):
        for button in self.action_buttons:
            button.setEnabled(not self.worker_running)
        self.abort_btn.setEnabled(
            self.worker_running and self.abort_enabled.get() and
            not self.abort_requested.get())
        self.apply_z_btn.setEnabled((not self.worker_running) and
            (self.last_z_proposal is not None or self.last_xyz_proposal is not None))
        self.apply_xy_btn.setEnabled((not self.worker_running) and
            (self.last_xy_proposal is not None or self.last_xyz_proposal is not None))
        self.apply_xyz_btn.setEnabled((not self.worker_running) and
            (self.last_xyz_proposal is not None or
             (self.last_z_proposal is not None and self.last_xy_proposal is not None)))
        self.revert_btn.setEnabled((not self.worker_running) and self.revert_offsets_snapshot is not None)
        if hasattr(self, "save_machine_btn"):
            self.save_machine_btn.setEnabled((not self.worker_running) and
                not self.has_pending_temporary_z())

    def abort_action(self):
        if not self.worker_running or not self.abort_enabled.get():
            return
        if self.abort_requested.compareAndSet(False, True):
            self.abort_btn.setEnabled(False)
            self.abort_btn.setText("Abort Requested...")
            self.append_log("Abort requested; stopping at the next calibration checkpoint")

    def calibration_log(self, msg):
        if self.abort_requested.get():
            raise CalibrationAborted("Calibration aborted by operator")
        self.append_log(msg)

    def restore_offsets_after_abort(self, nozzle, before):
        current = needle_calibration.loc_to_dict(
            nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        changed = False
        for axis in ["x", "y", "z", "rotation"]:
            if abs(float(current.get(axis, 0.0)) - float(before.get(axis, 0.0))) > 0.0000001:
                changed = True
                break
        if changed:
            needle_calibration.set_offsets_and_verify(nozzle, before, self.append_log)
            needle_calibration.synchronize_coordinate_system(nozzle, None, self.append_log)
        verified = needle_calibration.loc_to_dict(
            nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        if not needle_calibration.offsets_match(verified, before):
            raise Exception("Abort offset restoration did not verify: %s" % verified)
        self.append_log("Abort complete; original head offsets restored")
        return changed

    def on_field_changed(self):
        if self.loading_ui:
            return
        self.update_dirty()

    def on_mode_changed(self):
        self.update_mode_visibility()
        self.on_field_changed()

    def set_row_visible(self, row, visible):
        if row is None:
            return
        for component in row:
            component.setVisible(bool(visible))

    def update_mode_visibility(self):
        z_acquisition = str(self.z_combos.get("acquisition_method").getSelectedItem()
            if self.z_combos.get("acquisition_method") is not None else "hardware_interrupt")
        z_mode = str(self.z_combos.get("probe_mode").getSelectedItem()
            if self.z_combos.get("probe_mode") is not None else "actuator")
        z_polling = z_acquisition == "polling"
        z_direct = z_acquisition == "hardware_interrupt" and z_mode == "direct_gcode"
        z_actuator = z_acquisition == "hardware_interrupt" and z_mode == "actuator"
        self.set_row_visible(self.z_rows.get("probe_mode"), not z_polling)
        self.set_row_visible(self.z_rows.get("actuator_name"), z_polling or z_direct)
        self.set_row_visible(self.z_rows.get("sequence_actuator_name"), z_actuator)
        self.set_row_visible(self.z_rows.get("channel"), z_direct)
        for key in ["coarse_feed", "fine_feed", "fine_repeats", "probe_acceleration"]:
            self.set_row_visible(self.z_rows.get(key), z_direct)
        for key in ["coarse_step_mm", "fine_step_mm", "release_retract_step_mm",
                "max_release_retract_mm", "read_settle_ms", "debounce_count",
                "debounce_retries", "debounce_interval_ms",
                "polling_coarse_repeats", "polling_fine_repeats"]:
            self.set_row_visible(self.z_rows.get(key), z_polling)
        for key in ["max_descent_mm", "retract_after_contact_mm"]:
            self.set_row_visible(self.z_rows.get(key), z_polling or z_direct)

        post_acquisition = str(self.post_combos.get("acquisition_method").getSelectedItem()
            if self.post_combos.get("acquisition_method") is not None else "polling")
        post_hardware = post_acquisition == "hardware_interrupt"
        polling_keys = ["scan_clearance_mm", "scan_overtravel_mm",
            "max_radial_probe_travel_mm", "confirm_backoff_mm",
            "confirm_reapproach_mm", "confirm_position_tolerance_mm",
            "step_mm", "speed", "confirm_step_mm",
            "confirm_reject_early", "read_settle_ms",
            "debounce_count", "debounce_retries", "debounce_interval_ms",
            "allow_single_read_contact", "outside_recover_step_mm",
            "outside_recover_max_mm", "log_every_step", "log_step_interval"]
        hardware_keys = ["hardware_channel", "hardware_probe_acceleration",
            "hardware_coarse_feed", "hardware_fine_feed",
            "hardware_coarse_repeats", "hardware_fine_repeats",
            "hardware_fine_retry_attempts",
            "hardware_g38_attempts",
            "hardware_start_radius_mm", "hardware_coarse_max_travel_mm",
            "hardware_min_coarse_travel_mm",
            "hardware_fine_retract_mm", "hardware_fine_max_travel_mm",
            "hardware_repeat_tolerance_mm"]
        for key in polling_keys:
            self.set_row_visible(self.post_rows.get(key), not post_hardware)
        for key in hardware_keys:
            self.set_row_visible(self.post_rows.get(key), post_hardware)
        try:
            self.frame.getContentPane().revalidate()
            self.frame.getContentPane().repaint()
        except:
            pass

    def selected_channel(self, combo):
        text = str(combo.getSelectedItem() or "").strip().upper()
        if text.startswith("Q0"):
            return "Q0"
        if text.startswith("Q1"):
            return "Q1"
        return text

    def select_channel(self, combo, value):
        channel = str(value or "Q0").strip().upper()
        combo.setSelectedItem("Q1 - Contact" if channel == "Q1" else "Q0 - Voron")

    def update_dirty(self):
        try:
            dirty = compact_json(needle_calibration.public_clean_config(self.collect_config(False))) != self.saved_snapshot
        except:
            dirty = True
        self.dirty_label.setText("DIRTY" if dirty else "clean")

    def refresh_selectors(self):
        try:
            set_combo_items(self.selector_combos["nozzle_name"], available_nozzle_names(),
                combo_text(self.selector_combos["nozzle_name"]))
            set_combo_items(self.selector_combos["top_camera_name"], available_camera_names(),
                combo_text(self.selector_combos["top_camera_name"]))
            actuator_names = available_actuator_names()
            set_combo_items(self.z_combos["actuator_name"], actuator_names,
                combo_text(self.z_combos["actuator_name"]))
            set_combo_items(self.z_combos["sequence_actuator_name"], actuator_names,
                combo_text(self.z_combos["sequence_actuator_name"]))
            set_combo_items(self.post_combos["actuator_name"], actuator_names,
                combo_text(self.post_combos["actuator_name"]))
            names = needle_calibration.list_available_vision_pipeline_names(get_machine())
            set_combo_items(self.z_combos["pipeline_source"], names,
                combo_text(self.z_combos["pipeline_source"]))
            set_combo_items(self.post_combos["pipeline_source"], names,
                combo_text(self.post_combos["pipeline_source"]))
            self.append_log("Refreshed object and pipeline lists")
        except Exception as e:
            self.show_error("Refresh Lists failed", e)

    def load_into_ui(self, cfg):
        self.loading_ui = True
        try:
            for key in self.selector_combos:
                self.selector_combos[key].setSelectedItem(str(cfg.get(key, "")))
            zfid = cfg.get("fiducials", {}).get("z_touch", {})
            zloc = zfid.get("location", {})
            for axis in ["x", "y", "z", "rotation"]:
                self.z_fields[axis].setText(str(zloc.get(axis, 0.0)))
            self.z_fields["trigger_offset_mm"].setText(str(zfid.get("trigger_offset_mm", 0.0)))
            self.z_fields["result_stage"].setText(str(zfid.get("result_stage", "results")))
            self.z_combos["actuator_name"].setSelectedItem(str(
                cfg.get("z_actuator_name", cfg.get("contact_actuator_name", ""))))
            zprobe = cfg.get("z_probe", {})
            self.z_combos["acquisition_method"].setSelectedItem(str(
                zprobe.get("acquisition_method", "hardware_interrupt")))
            self.z_combos["probe_mode"].setSelectedItem(str(
                zprobe.get("mode", "actuator")))
            self.z_combos["sequence_actuator_name"].setSelectedItem(str(
                zprobe.get("sequence_actuator_name", "Run Voron Probe")))
            self.select_channel(self.z_combos["channel"], zprobe.get("channel", "Q0"))
            for key in self.z_probe_fields:
                value = zprobe.get(key, "")
                self.z_probe_fields[key].setText("" if value is None else str(value))
            self.load_pipeline(self.z_combos, zfid.get("pipeline", {}))

            post = cfg.get("post_xy", {})
            ploc = post.get("location", {})
            for axis in ["x", "y", "z", "rotation"]:
                self.post_fields[axis].setText(str(ploc.get(axis, 0.0)))
            for key in self.post_fields:
                if key in ["x", "y", "z", "rotation"]:
                    continue
                value = post.get(key, "")
                self.post_fields[key].setText("" if value is None else str(value))
            self.post_combos["actuator_name"].setSelectedItem(str(
                post.get("actuator_name", cfg.get("contact_actuator_name", ""))))
            self.post_combos["acquisition_method"].setSelectedItem(str(
                post.get("acquisition_method", "polling")))
            self.select_channel(self.post_combos["hardware_channel"],
                post.get("hardware_channel", "Q1"))
            self.load_pipeline(self.post_combos, post.get("pipeline", {}))

            for key in self.motion_fields:
                value = self.get_nested_value(cfg, key)
                self.motion_fields[key].setText("" if value is None else str(value))
        finally:
            self.loading_ui = False
        self.update_mode_visibility()
        self.update_dirty()

    def load_pipeline(self, combos, pipe):
        mode = str(pipe.get("mode", "named") or "named")
        if mode not in ["named", "xml_path"]:
            mode = "named"
        combos["pipeline_mode"].setSelectedItem(mode)
        combos["pipeline_source"].setSelectedItem(str(pipe.get("name", "") if mode == "named" else pipe.get("path", "")))

    def get_nested_value(self, cfg, dotted):
        data = cfg
        for part in dotted.split("."):
            if not isinstance(data, dict):
                return None
            data = data.get(part)
        return data

    def set_nested_value(self, cfg, dotted, value):
        parts = dotted.split(".")
        data = cfg
        for part in parts[:-1]:
            if part not in data or not isinstance(data[part], dict):
                data[part] = {}
            data = data[part]
        data[parts[-1]] = value

    def collect_config(self, validate):
        cfg = needle_calibration.deep_merge(needle_calibration.DEFAULT_CONFIG, self.cfg)
        cfg["nozzle_name"] = combo_text(self.selector_combos["nozzle_name"])
        cfg["top_camera_name"] = combo_text(self.selector_combos["top_camera_name"])
        cfg["z_actuator_name"] = combo_text(self.z_combos["actuator_name"])
        cfg["z_probe"]["acquisition_method"] = str(
            self.z_combos["acquisition_method"].getSelectedItem()).strip()
        cfg["z_probe"]["mode"] = str(
            self.z_combos["probe_mode"].getSelectedItem()).strip()
        cfg["z_probe"]["sequence_actuator_name"] = combo_text(
            self.z_combos["sequence_actuator_name"])
        cfg["z_probe"]["channel"] = self.selected_channel(self.z_combos["channel"])
        z_acquisition = cfg["z_probe"]["acquisition_method"]
        z_mode = cfg["z_probe"]["mode"]
        z_keys = ["start_above_mm"]
        if z_acquisition == "polling":
            z_keys += ["max_descent_mm", "coarse_step_mm", "fine_step_mm",
                "retract_after_contact_mm", "release_retract_step_mm",
                "max_release_retract_mm", "read_settle_ms", "debounce_count",
                "debounce_retries", "debounce_interval_ms",
                "polling_coarse_repeats", "polling_fine_repeats"]
        elif z_mode == "direct_gcode":
            z_keys += ["max_descent_mm", "retract_after_contact_mm", "coarse_feed",
                "fine_feed", "fine_repeats", "probe_acceleration"]
        for key in z_keys:
            cfg["z_probe"][key] = self.parse_float(
                self.z_probe_fields[key].getText(), "z_probe.%s" % key)
        if str(cfg.get("contact_actuator_name", "") or "").strip() == "":
            cfg["contact_actuator_name"] = cfg["z_actuator_name"]
        cfg["fiducials"]["z_touch"] = {
            "location": self.collect_location(self.z_fields, "z_touch"),
            "trigger_offset_mm": self.parse_float(
                self.z_fields["trigger_offset_mm"].getText(),
                "z_touch.trigger_offset_mm"),
            "result_stage": self.z_fields["result_stage"].getText().strip(),
            "pipeline": self.collect_pipeline(self.z_combos)
        }
        post = cfg["post_xy"]
        post["acquisition_method"] = str(
            self.post_combos["acquisition_method"].getSelectedItem()).strip()
        post["actuator_name"] = combo_text(self.post_combos["actuator_name"])
        post["hardware_channel"] = self.selected_channel(
            self.post_combos["hardware_channel"])
        post["location"] = self.collect_location(self.post_fields, "post_xy")
        post_numeric = ["post_diameter_mm", "side_probe_z_offset_mm",
            "min_valid_contacts_for_fit",
            "local_lift_mm", "max_fit_rms_mm", "max_fit_error_mm",
            "max_center_error_mm"]
        if post["acquisition_method"] == "hardware_interrupt":
            post_numeric += ["hardware_probe_acceleration", "hardware_coarse_feed",
                "hardware_fine_feed", "hardware_coarse_repeats",
                "hardware_fine_repeats", "hardware_fine_retry_attempts",
                "hardware_g38_attempts",
                "hardware_start_radius_mm",
                "hardware_coarse_max_travel_mm", "hardware_min_coarse_travel_mm",
                "hardware_fine_retract_mm",
                "hardware_fine_max_travel_mm", "hardware_repeat_tolerance_mm"]
        else:
            post_numeric += ["scan_clearance_mm", "scan_overtravel_mm",
                "max_radial_probe_travel_mm", "confirm_backoff_mm",
                "confirm_reapproach_mm", "confirm_position_tolerance_mm",
                "step_mm", "read_settle_ms", "debounce_count", "debounce_retries",
                "debounce_interval_ms", "outside_recover_step_mm",
                "outside_recover_max_mm", "log_step_interval"]
        for key in post_numeric:
            post[key] = self.parse_float(self.post_fields[key].getText(), "post_xy.%s" % key)
        if post["acquisition_method"] == "polling":
            for key in ["log_every_step", "confirm_reject_early", "allow_single_read_contact"]:
                post[key] = self.parse_bool(self.post_fields[key].getText(),
                    "post_xy.%s" % key)
            for key in ["speed", "confirm_step_mm"]:
                post[key] = self.parse_optional_float(self.post_fields[key].getText(),
                    "post_xy.%s" % key)
        post["angles_deg"] = needle_calibration.parse_angles(
            self.post_fields["angles_deg"].getText(),
            [0, 45, 90, 135, 180, 225, 270, 315])
        post["result_stage"] = self.post_fields["result_stage"].getText().strip()
        post["pipeline"] = self.collect_pipeline(self.post_combos)
        for key in self.motion_fields:
            self.set_nested_value(cfg, key, self.parse_float(self.motion_fields[key].getText(), key))
        if validate:
            self.validate_config(cfg)
        return cfg

    def collect_location(self, fields, label):
        return {"x": self.parse_float(fields["x"].getText(), "%s.x" % label),
            "y": self.parse_float(fields["y"].getText(), "%s.y" % label),
            "z": self.parse_float(fields["z"].getText(), "%s.z" % label),
            "rotation": self.parse_float(fields["rotation"].getText(), "%s.rotation" % label)}

    def collect_pipeline(self, combos):
        mode = str(combos["pipeline_mode"].getSelectedItem()).strip()
        source = combo_text(combos["pipeline_source"])
        if mode == "xml_path":
            return {"mode": "xml_path", "path": source, "name": ""}
        return {"mode": "named", "name": source, "path": ""}

    def parse_float(self, text, label):
        try:
            return float(str(text).strip())
        except:
            raise Exception("%s must be a number" % label)

    def parse_optional_float(self, text, label):
        value = str(text).strip()
        if value == "" or value.lower() == "none":
            return None
        try:
            return float(value)
        except:
            raise Exception("%s must be blank or a number" % label)

    def parse_bool(self, text, label):
        value = str(text).strip().lower()
        if value in ["1", "true", "t", "yes", "y", "on"]:
            return True
        if value in ["0", "false", "f", "no", "n", "off"]:
            return False
        raise Exception("%s must be true or false" % label)

    def validate_config(self, cfg):
        if str(cfg.get("nozzle_name", "")).strip() == "":
            raise Exception("Nozzle is required")
        if str(cfg.get("top_camera_name", "")).strip() == "":
            raise Exception("Top camera is required")
        zprobe = cfg.get("z_probe", {})
        acquisition = str(zprobe.get("acquisition_method", "hardware_interrupt"))
        if acquisition not in ["polling", "hardware_interrupt"]:
            raise Exception("Z touch method must be polling or hardware_interrupt")
        if acquisition == "polling":
            if str(cfg.get("z_actuator_name", "")).strip() == "":
                raise Exception("Z actuator is required in polling mode")
            for key in ["polling_coarse_repeats", "polling_fine_repeats"]:
                repeats = float(zprobe.get(key, 1))
                if repeats != float(int(repeats)) or repeats < 1:
                    raise Exception("z_probe.%s must be a whole number >= 1" % key)
            for key in ["coarse_step_mm", "fine_step_mm", "max_descent_mm",
                    "retract_after_contact_mm"]:
                if float(zprobe.get(key, 0.0)) <= 0.0:
                    raise Exception("z_probe.%s must be > 0" % key)
        elif str(zprobe.get("mode", "actuator")) == "actuator":
            if not str(zprobe.get("sequence_actuator_name", "")).strip():
                raise Exception("Sequence actuator is required in actuator mode")
        else:
            if str(cfg.get("z_actuator_name", "")).strip() == "":
                raise Exception("Z driver actuator is required in direct_gcode mode")
            repeats = float(zprobe.get("fine_repeats", 1))
            if repeats != float(int(repeats)) or repeats < 1:
                raise Exception("z_probe.fine_repeats must be a whole number >= 1")
            for key in ["coarse_feed", "fine_feed", "probe_acceleration",
                    "max_descent_mm", "retract_after_contact_mm"]:
                if float(zprobe.get(key, 0.0)) <= 0.0:
                    raise Exception("z_probe.%s must be > 0" % key)
            if str(zprobe.get("channel", "")) not in ["Q0", "Q1"]:
                raise Exception("z_probe.channel must be Q0 or Q1")
        if str(cfg.get("post_xy", {}).get("actuator_name", "")).strip() == "":
            raise Exception("Post XY read/driver actuator is required")
        post = cfg.get("post_xy", {})
        post_acquisition = str(post.get("acquisition_method", "polling"))
        if post_acquisition not in ["polling", "hardware_interrupt"]:
            raise Exception("Post XY touch method must be polling or hardware_interrupt")
        if post_acquisition == "hardware_interrupt":
            if str(post.get("hardware_channel", "")) not in ["Q0", "Q1"]:
                raise Exception("post_xy.hardware_channel must be Q0 or Q1")
            for key in ["hardware_probe_acceleration", "hardware_coarse_feed",
                    "hardware_fine_feed"]:
                if float(post.get(key, 0.0)) <= 0.0:
                    raise Exception("post_xy.%s must be > 0" % key)
            for key in ["hardware_coarse_repeats", "hardware_fine_repeats",
                    "hardware_g38_attempts"]:
                repeats = float(post.get(key, 1))
                if repeats != float(int(repeats)) or repeats < 1:
                    raise Exception("post_xy.%s must be a whole number >= 1" % key)
            retries = float(post.get("hardware_fine_retry_attempts", 2))
            if retries != float(int(retries)) or retries < 0:
                raise Exception(
                    "post_xy.hardware_fine_retry_attempts must be a whole number >= 0")
            for key in ["hardware_start_radius_mm", "hardware_coarse_max_travel_mm",
                    "hardware_min_coarse_travel_mm",
                    "hardware_fine_retract_mm", "hardware_fine_max_travel_mm",
                    "hardware_repeat_tolerance_mm"]:
                if float(post.get(key, 0.0)) <= 0.0:
                    raise Exception("post_xy.%s must be > 0" % key)
            if float(post.get("hardware_fine_max_travel_mm", 0.0)) <= float(
                    post.get("hardware_fine_retract_mm", 0.0)):
                raise Exception("Post XY fine max travel must exceed fine retract")
        for label, pipe in [("Z touch", cfg["fiducials"]["z_touch"]["pipeline"]),
                ("Post XY", cfg["post_xy"]["pipeline"])]:
            mode = str(pipe.get("mode", "named") or "named")
            if mode == "named" and str(pipe.get("name", "")).strip() == "":
                raise Exception("%s pipeline name is required" % label)
            if mode == "xml_path" and str(pipe.get("path", "")).strip() == "":
                raise Exception("%s XML pipeline path is required" % label)

    def refresh_current_values(self):
        try:
            cfg = self.collect_config(False)
            nozzle = needle_calibration.find_nozzle(cfg, None, get_machine())
            offsets = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
            temporary_note = ""
            if self.has_pending_temporary_z():
                temporary_note = "\nTEMPORARY Z ACTIVE - not committed"
            self.offsets_area.setText("Nozzle: %s\nX %.4f  Y %.4f  Z %.4f  R %.4f%s" %
                (object_name(nozzle), offsets.getX(), offsets.getY(),
                 offsets.getZ(), offsets.getRotation(), temporary_note))
            self.status_area.setText("Ready%s" %
                (" - temporary Z active (not saved)" if self.has_pending_temporary_z()
                 else ""))
        except Exception as e:
            self.offsets_area.setText("Unavailable: %s" % e)

    def hide_measurement_results(self):
        self.measurement_results_area.setText("")
        if self.measurement_results_panel is not None:
            self.measurement_results_panel.setVisible(False)
        if self.right_top_panel is not None:
            self.right_top_panel.revalidate()
            self.right_top_panel.repaint()

    def show_measurement_results(self, result, saved_offsets=None):
        proposal = result.get("proposal") if result is not None else None
        if not proposal or not proposal.get("proposed_offsets"):
            self.hide_measurement_results()
            return
        proposed = proposal.get("proposed_offsets", {})
        saved = saved_offsets or result.get("old_offsets") or {}
        axes = ["x", "y", "z", "rotation"]
        values = {}
        deltas = {}
        for axis in axes:
            values[axis] = float(proposed.get(axis, saved.get(axis, 0.0)))
            deltas[axis] = values[axis] - float(saved.get(axis, 0.0))
        self.measurement_results_area.setText(
            "Proposed:  X %.6f   Y %.6f\n"
            "           Z %.6f   R %.6f\n"
            "Delta:     X %+.6f   Y %+.6f\n"
            "           Z %+.6f   R %+.6f" %
            (values["x"], values["y"], values["z"], values["rotation"],
             deltas["x"], deltas["y"], deltas["z"], deltas["rotation"]))
        if self.measurement_results_panel is not None:
            self.measurement_results_panel.setVisible(True)
        if self.right_top_panel is not None:
            self.right_top_panel.revalidate()
            self.right_top_panel.repaint()

    def fields_for_role(self, role):
        if role == "z_touch":
            return self.z_fields
        return self.post_fields

    def set_location_fields(self, role, loc):
        loc = loc.convertToUnits(LengthUnit.Millimeters)
        fields = self.fields_for_role(role)
        fields["x"].setText("%.4f" % loc.getX())
        fields["y"].setText("%.4f" % loc.getY())
        fields["z"].setText("%.4f" % loc.getZ())
        fields["rotation"].setText("%.4f" % loc.getRotation())

    def capture_camera_location(self, role):
        try:
            cfg = self.collect_config(False)
            camera = needle_calibration.find_top_camera(cfg, get_machine())
            loc = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
            self.set_location_fields(role, loc)
            self.append_log("Captured %s from camera DRO" % role)
            self.update_dirty()
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Capture Camera DRO failed", e)

    def stored_location_for_role(self, role, cfg):
        if role == "z_touch":
            return cfg["fiducials"]["z_touch"]["location"]
        return cfg["post_xy"]["location"]

    def go_camera_location(self, role):
        try:
            cfg = self.collect_config(False)
            loc = self.stored_location_for_role(role, cfg)
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Go Camera setup failed", e)
            return
        self.run_worker("Go Camera %s" % role,
            lambda: self.do_go_camera_location(cfg, loc), self.go_camera_done)

    def do_go_camera_location(self, cfg, loc):
        machine_obj = require_machine_execute()
        gui_self = self
        class Task(Callable):
            def call(task_self):
                try:
                    camera = needle_calibration.find_top_camera(cfg, machine_obj)
                    needle_calibration.move_safe(camera, mm_location_from_dict(loc))
                    return {"ok": True, "target": "camera", "location": loc}
                finally:
                    pass
        return machine_obj.execute(Task())

    def go_camera_done(self, result):
        self.last_result = result
        self.result_area.setText(self.format_result(result))
        self.refresh_current_values()
        self.set_state(READY)
        self.append_log("Camera move finished")

    def test_pipeline_action(self, role):
        try:
            cfg = self.collect_config(True)
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Pipeline setup failed", e)
            return
        self.run_worker("Test %s Pipeline" % role,
            lambda: self.do_test_pipeline(cfg, role), self.test_pipeline_done)

    def do_test_pipeline(self, cfg, role):
        machine_obj = require_machine_execute()
        gui_self = self
        class Task(Callable):
            def call(task_self):
                try:
                    if role == "post_xy":
                        return needle_calibration.capture_post_center(cfg, None, machine_obj, gui_self.append_log)
                    return needle_calibration.capture_z_touch_center(cfg, None, machine_obj, gui_self.append_log)
                finally:
                    pass
        return machine_obj.execute(Task())

    def test_pipeline_done(self, result):
        self.last_result = result
        self.result_area.setText(self.format_result(result))
        self.refresh_current_values()
        self.set_state(READY)

    def measure_action(self, action):
        if self.has_pending_temporary_z():
            self.set_state(ERROR)
            self.show_error("Pending Calibration",
                "A full XYZ result has a temporary Z offset active. "
                "Apply XYZ or use Revert Offsets before starting another calibration.")
            return
        try:
            cfg = self.collect_config(True)
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Measure setup failed", e)
            return
        self.last_z_proposal = None
        self.last_xy_proposal = None
        self.last_xyz_proposal = None
        self.hide_measurement_results()
        self.run_worker("Measure %s" % action,
            lambda: self.do_measure(action, cfg), self.measure_done, abortable=True)

    def do_measure(self, action, cfg):
        machine_obj = require_machine_execute()
        gui_self = self
        class Task(Callable):
            def call(task_self):
                nozzle = None
                before = None
                result = None
                aborted = False
                cleanup = None
                offsets_changed = False
                try:
                    nozzle = needle_calibration.find_nozzle(cfg, None, machine_obj)
                    before = needle_calibration.loc_to_dict(
                        nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
                    gui_self.calibration_log("Abort checkpoint active")
                    if action == "z":
                        result = needle_calibration.run_z_measure(
                            cfg, nozzle, machine_obj, gui_self.calibration_log)
                    elif action == "post_xy":
                        result = needle_calibration.run_post_xy_measure(
                            cfg, nozzle, machine_obj, gui_self.calibration_log)
                    else:
                        result = needle_calibration.run_full_measure(
                            cfg, nozzle, machine_obj, gui_self.calibration_log)
                except CalibrationAborted:
                    aborted = True
                finally:
                    gui_self.abort_enabled.set(False)
                    if nozzle is not None and (aborted or gui_self.abort_requested.get()):
                        aborted = True
                        cleanup = needle_calibration.abort_nozzle_cleanup(
                            nozzle, cfg, gui_self.append_log)
                        offsets_changed = gui_self.restore_offsets_after_abort(nozzle, before)
                if aborted:
                    result = {
                        "ok": False,
                        "aborted": True,
                        "stage": action,
                        "message": "Calibration aborted by operator; cleanup complete",
                        "abort_cleanup": cleanup,
                        "offsets_restored": True,
                        "offsets_changed_before_restore": offsets_changed,
                    }
                return {"result": result, "before": before}
        return machine_obj.execute(Task())

    def measure_done(self, payload):
        result = payload.get("result")
        self.last_result = result
        if result is not None and result.get("aborted", False):
            cleanup = result.get("abort_cleanup") or {}
            self.result_area.setText(self.format_result(result))
            self.refresh_current_values()
            if cleanup.get("ok", False):
                self.set_state(self.pre_run_state)
            else:
                self.set_state(ERROR)
                self.show_error("Abort cleanup incomplete",
                    "; ".join(cleanup.get("errors") or ["Unknown abort cleanup error"]))
            self.append_log(result.get("message"))
            return
        before = payload.get("before")
        if before is not None:
            self.original_offsets_snapshot = deep_copy(before)
            self.revert_offsets_snapshot = deep_copy(before)
        if result is not None and result.get("ok", False):
            if result.get("stage") == "z":
                self.last_z_proposal = result.get("proposal")
            elif result.get("stage") == "post_xy":
                self.last_xy_proposal = result.get("proposal")
            elif result.get("stage") == "full":
                self.last_xyz_proposal = result.get("proposal")
                self.last_z_proposal = result.get("z", {}).get("proposal")
                self.last_xy_proposal = result.get("post_xy", {}).get("proposal")
                state = result.get("calibration_state") or {}
                if state.get("temporary_z_active", False):
                    self.original_offsets_snapshot = deep_copy(
                        state.get("original_offsets") or before)
                    self.revert_offsets_snapshot = deep_copy(
                        self.original_offsets_snapshot)
                    self.pending_calibration_state = deep_copy(state)
                else:
                    self.pending_calibration_state = None
            self.show_measurement_results(result, payload.get("before"))
        elif result is not None and result.get("stage") == "full":
            state = result.get("calibration_state") or {}
            self.pending_calibration_state = (deep_copy(state)
                if state.get("temporary_z_active", False) else None)
        self.result_area.setText(self.format_result(result))
        self.refresh_current_values()
        self.set_state(READY if result is not None and result.get("ok", False) else ERROR)

    def select_proposal(self, action):
        if action == "z":
            if self.last_z_proposal is not None:
                return self.last_z_proposal
            if self.last_xyz_proposal is not None:
                return self.last_xyz_proposal
        if action == "xy":
            if self.last_xy_proposal is not None:
                return self.last_xy_proposal
            if self.last_xyz_proposal is not None:
                return self.last_xyz_proposal
        if action == "xyz":
            if self.last_xyz_proposal is not None:
                return self.last_xyz_proposal
            if self.last_z_proposal is not None and self.last_xy_proposal is not None:
                zoff = self.last_z_proposal["proposed_offsets"]
                xyoff = self.last_xy_proposal["proposed_offsets"]
                return {"type": "xyz", "proposed_offsets": {
                    "x": xyoff["x"], "y": xyoff["y"], "z": zoff["z"],
                    "rotation": xyoff.get("rotation", zoff.get("rotation", 0.0))}}
        raise Exception("No %s proposal is available" % action)

    def apply_action(self, action):
        try:
            cfg = self.collect_config(False)
            proposal = deep_copy(self.select_proposal(action))
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Apply setup failed", e)
            return
        self.run_worker("Apply %s" % action,
            lambda: self.do_apply(action, cfg, proposal), self.apply_done)

    def do_apply(self, action, cfg, proposal):
        machine_obj = require_machine_execute()
        gui_self = self
        class Task(Callable):
            def call(task_self):
                try:
                    nozzle = needle_calibration.find_nozzle(cfg, None, machine_obj)
                    before = needle_calibration.loc_to_dict(
                        nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
                    if action == "z":
                        result = needle_calibration.apply_z(cfg, nozzle, proposal, gui_self.append_log)
                    elif action == "xy":
                        result = needle_calibration.apply_xy(cfg, nozzle, proposal, gui_self.append_log)
                    else:
                        result = needle_calibration.apply_xyz(cfg, nozzle, proposal, gui_self.append_log)
                    return {"result": result, "before": before, "action": action}
                finally:
                    pass
        return machine_obj.execute(Task())

    def apply_done(self, payload):
        # A full XYZ measurement already captured the original offsets before
        # its temporary Z application. Never replace that rollback snapshot
        # with the temporary offsets observed immediately before Apply.
        if self.original_offsets_snapshot is None:
            self.original_offsets_snapshot = deep_copy(payload.get("before"))
            self.revert_offsets_snapshot = deep_copy(self.original_offsets_snapshot)
        if payload.get("action") == "xyz":
            self.pending_calibration_state = None
        self.last_result = payload.get("result")
        self.result_area.setText(self.format_result(self.last_result))
        self.refresh_current_values()
        self.set_state(READY)

    def revert_action(self):
        if self.revert_offsets_snapshot is None:
            return
        try:
            cfg = self.collect_config(False)
            backup = deep_copy(self.revert_offsets_snapshot)
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Revert setup failed", e)
            return
        self.run_worker("Revert Offsets", lambda: self.do_revert(cfg, backup), self.revert_done)

    def do_revert(self, cfg, backup):
        machine_obj = require_machine_execute()
        gui_self = self
        class Task(Callable):
            def call(task_self):
                try:
                    nozzle = needle_calibration.find_nozzle(cfg, None, machine_obj)
                    return needle_calibration.revert_offsets(nozzle, backup,
                        gui_self.append_log, cfg)
                finally:
                    pass
        return machine_obj.execute(Task())

    def revert_done(self, result):
        self.last_result = result
        self.result_area.setText(self.format_result(result))
        if result is not None and result.get("ok", False):
            self.pending_calibration_state = None
            self.original_offsets_snapshot = None
            self.revert_offsets_snapshot = None
            self.last_z_proposal = None
            self.last_xy_proposal = None
            self.last_xyz_proposal = None
            self.append_log("Revert complete; pending temporary calibration cleared")
        self.refresh_current_values()
        self.set_state(READY if result is not None and result.get("ok", False) else ERROR)

    def request_close(self):
        if self.worker_running:
            JOptionPane.showMessageDialog(self.frame,
                "A machine operation is still running. Wait for it to finish before closing.",
                "Operation In Progress", JOptionPane.INFORMATION_MESSAGE)
            return
        if not self.has_pending_temporary_z():
            self.frame.dispose()
            return
        choice = JOptionPane.showConfirmDialog(self.frame,
            "A temporary Z correction is active and has not been committed. "
            "Revert the original offsets before closing?",
            "Pending Temporary Calibration", JOptionPane.YES_NO_OPTION,
            JOptionPane.WARNING_MESSAGE)
        if choice != JOptionPane.YES_OPTION:
            return
        try:
            cfg = self.collect_config(False)
            backup = deep_copy(self.original_offsets_snapshot or
                self.revert_offsets_snapshot)
            if backup is None:
                raise Exception("Original calibration offsets are unavailable")
        except Exception as e:
            self.show_error("Close setup failed", e)
            return
        self.run_worker("Revert Before Close",
            lambda: self.do_revert(cfg, backup), self.close_revert_done)

    def close_revert_done(self, result):
        if result is not None and result.get("ok", False):
            self.revert_done(result)
            self.frame.dispose()
        else:
            self.set_state(ERROR)
            self.show_error("Close blocked; temporary offsets remain active",
                (result or {}).get("message", "Offset restoration failed"))

    def save_config_action(self):
        try:
            cfg = self.collect_config(False)
            needle_calibration.save_config(cfg, CONFIG_PATH)
            self.cfg = cfg
            self.saved_snapshot = compact_json(needle_calibration.public_clean_config(cfg))
            self.refresh_current_values()
            self.set_state(SAVED)
            self.append_log("Saved JSON config: %s" % CONFIG_PATH)
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Save JSON failed", e)

    def save_machine_action(self):
        if self.has_pending_temporary_z():
            self.set_state(ERROR)
            self.show_error("Save OpenPnP Config blocked",
                "Apply XYZ or Revert Offsets before saving; the temporary Z "
                "correction must not be written to machine.xml.")
            return
        try:
            Configuration.get().save()
            self.refresh_current_values()
            self.set_state(SAVED)
            self.append_log("Saved OpenPnP machine configuration")
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Save OpenPnP Config failed", e)

    def format_result(self, result):
        if result is None:
            return "No result"
        lines = []
        lines.append("ok: %s" % result.get("ok"))
        lines.append("stage: %s" % result.get("stage", ""))
        lines.append("message: %s" % result.get("message", ""))
        state = result.get("calibration_state") or {}
        if state.get("temporary_z_active", False):
            lines.append("TEMPORARY Z ACTIVE: not committed; Apply XYZ or Revert Offsets")
        proposal = result.get("proposal")
        if proposal:
            delta = proposal.get("offset_delta", {})
            proposed = proposal.get("proposed_offsets", {})
            lines.append("offset delta: X %.6f  Y %.6f  Z %.6f  R %.6f" %
                (float(delta.get("x", 0.0)), float(delta.get("y", 0.0)),
                 float(delta.get("z", 0.0)), float(delta.get("rotation", 0.0))))
            lines.append("proposed offsets: X %.6f  Y %.6f  Z %.6f  R %.6f" %
                (float(proposed.get("x", 0.0)), float(proposed.get("y", 0.0)),
                 float(proposed.get("z", 0.0)), float(proposed.get("rotation", 0.0))))
        details = result.get("details", {})
        if details.get("center_delta", None) is not None:
            cd = details.get("center_delta")
            lines.append("center delta: X %.6f  Y %.6f" %
                (float(cd.get("x", 0.0)), float(cd.get("y", 0.0))))
        if details.get("fit", None) is not None:
            fit = details.get("fit", {})
            center = fit.get("center", {})
            lines.append("fit center/radius/diameter/RMS/max: %.6f / %.6f / %.6f / %.6f / %.6f / %.6f" %
                (float(center.get("x", 0.0)), float(center.get("y", 0.0)),
                 float(fit.get("radius_mm", 0.0)),
                 float(details.get("fitted_contact_diameter_mm", 0.0)),
                 float(fit.get("rms_error_mm", 0.0)),
                 float(fit.get("max_error_mm", 0.0))))
        if details.get("estimated_effective_probe_diameter_mm", None) is not None:
            lines.append("estimated probe diameter/radius: %.6f / %.6f" %
                (float(details.get("estimated_effective_probe_diameter_mm", 0.0)),
                 float(details.get("estimated_effective_probe_radius_mm", 0.0))))
        for warning in details.get("fit_quality_warnings", []):
            lines.append("warning: %s (diagnostic only; XY uses opposite-pair midpoints)" %
                warning)
        lines.append("")
        lines.append(json.dumps(json_safe(result), sort_keys=True, indent=2))
        return "\n".join(lines)

    def run_worker(self, name, work, done, abortable=False):
        if self.worker_running:
            return
        self.pre_run_state = self.state
        self.abort_requested.set(False)
        self.abort_enabled.set(bool(abortable))
        self.abort_btn.setText("Abort")
        self.abort_btn.setVisible(bool(abortable))
        self.worker_running = True
        self.set_state(RUNNING)
        self.append_log("Starting %s" % name)
        gui_self = self
        class Worker(SwingWorker):
            def doInBackground(worker_self):
                return work()
            def done(worker_self):
                gui_self.worker_running = False
                gui_self.abort_enabled.set(False)
                gui_self.abort_btn.setVisible(False)
                try:
                    done(worker_self.get())
                except Exception as e:
                    gui_self.set_state(ERROR)
                    gui_self.show_error("%s failed" % name, e)
                gui_self.update_buttons()
        Worker().execute()


def start():
    gui_obj = NeedleCalibrationGui2()
    gui_obj.build()
    return gui_obj


class StartRunnable(Runnable):
    def run(self):
        start()


try:
    SwingUtilities.invokeLater(StartRunnable())
except:
    start()
