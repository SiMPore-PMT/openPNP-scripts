# needle_calibration_gui.py
#
# Lean dispensing needle Z + post XY calibration GUI for OpenPnP Jython 2.7.

import imp
import json
import os
import sys

from java.awt import (
    BasicStroke, BorderLayout, Color, Dimension, FlowLayout, GridBagConstraints,
    GridBagLayout, Image, Insets, RenderingHints
)
from java.awt.image import BufferedImage
from java.lang import Runnable
from java.util.concurrent import Callable
from javax.swing import (
    BorderFactory, Box, BoxLayout, JButton, JCheckBox, JComboBox, JFrame, JLabel,
    JOptionPane, JPanel, JScrollPane, JSplitPane, JTextArea, JTextField,
    ImageIcon, SwingUtilities, SwingWorker
)
from javax.swing.event import DocumentListener
from org.openpnp.model import Configuration, LengthUnit, Location


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
        out.append(os.path.dirname(os.path.abspath(__file__)))
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
        backend = os.path.join(path, "Events", "Homing", "needle_calibration.py")
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
BACKEND_PATH = os.path.join(HOMING_DIR, "needle_calibration.py")
needle_calibration = imp.load_source("needle_calibration_backend", BACKEND_PATH)
CONFIG_PATH = os.path.join(SCRIPT_DIR, "needle_calibration_config.json")


IDLE = "IDLE"
RUNNING = "RUNNING"
READY = "READY"
ERROR = "ERROR"
SAVED = "SAVED"
FIDUCIAL_ROLES = ["z_touch"]


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


def restore_machine_speed(machine_obj, logger=None):
    try:
        machine_obj.setSpeed(1.0)
        if logger:
            logger("Machine speed restored to 100%")
        return True
    except Exception, e:
        if logger:
            logger("Unable to restore machine speed to 100%: %s" % e)
    return False


def deep_copy(value):
    return json.loads(json.dumps(value, sort_keys=True))


def deep_merge(defaults, loaded):
    return needle_calibration.deep_merge(defaults, loaded)


def compact_json(data):
    return json.dumps(data, sort_keys=True, separators=(",", ":"))


def load_config_file():
    return needle_calibration.load_config(CONFIG_PATH)


def save_config_file(cfg):
    return needle_calibration.save_config(cfg, CONFIG_PATH)


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


def all_heads(machine_obj):
    return needle_calibration.all_heads(machine_obj)


def available_nozzle_names():
    machine_obj = get_machine()
    names = []
    for head in all_heads(machine_obj):
        try:
            for nozzle in java_list_to_py(head.getNozzles()):
                names.append(object_name(nozzle))
        except:
            pass
    return unique_sorted(names)


def available_camera_names():
    machine_obj = get_machine()
    names = []
    for head in all_heads(machine_obj):
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
    names = []
    owners = [machine_obj]
    for head in all_heads(machine_obj):
        owners.append(head)
    for owner in owners:
        try:
            for actuator in java_list_to_py(owner.getActuators()):
                names.append(object_name(actuator))
        except:
            pass
    return unique_sorted(names)


def available_vision_settings_names():
    try:
        return needle_calibration.list_available_vision_pipeline_names(get_machine())
    except:
        return []


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
        float(data.get("x", 0.0)),
        float(data.get("y", 0.0)),
        float(data.get("z", 0.0)),
        float(data.get("rotation", data.get("r", 0.0))))


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
                out[key] = "<image>"
            else:
                out[str(key)] = json_safe(value[key])
        return out
    try:
        if hasattr(value, "convertToUnits"):
            return needle_calibration.loc_to_dict(value)
    except:
        pass
    return str(value)


def resolve_pipeline_path(text):
    text = str(text or "").strip()
    if os.path.isabs(text):
        return text
    first = os.path.join(SCRIPT_DIR, text)
    if os.path.exists(first):
        return first
    return os.path.join(HOMING_DIR, text)


class FieldChangeListener(DocumentListener):
    def __init__(self, callback):
        self.callback = callback

    def insertUpdate(self, event):
        self.callback()

    def removeUpdate(self, event):
        self.callback()

    def changedUpdate(self, event):
        self.callback()


class NeedleCalibrationGui(object):
    def __init__(self):
        self.frame = JFrame("Needle Z + Post XY Calibration")
        self.cfg = load_config_file()
        self.saved_snapshot = compact_json(self.cleaned_config(self.cfg))
        self.loading_ui = False
        self.worker_running = False
        self.state = IDLE

        self.selector_combos = {}
        self.z_fields = {}
        self.z_combos = {}
        self.post_fields = {}
        self.post_combos = {}
        self.post_checks = {}
        self.advanced_fields = {}
        self.advanced_checks = {}
        self.advanced_combos = {}
        self.action_buttons = []
        self.motion_buttons = []

        self.pre_run_offsets = None
        self.last_result = None
        self.last_post_proposal = None
        self.last_capture_image = None
        self.last_overlay_feature = None

        self.status_area = JTextArea()
        self.offsets_area = JTextArea()
        self.result_area = JTextArea()
        self.log_area = JTextArea()
        self.state_label = JLabel("")
        self.dirty_label = JLabel("")
        self.computed_probe_z_label = JLabel("")
        self.image_label = JLabel("No image")
        self.image_label.setHorizontalAlignment(JLabel.CENTER)
        self.image_label.setVerticalAlignment(JLabel.CENTER)

    def build(self):
        self.frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE)
        self.frame.setMinimumSize(Dimension(1180, 760))

        root = JPanel(BorderLayout(8, 8))
        root.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8))
        root.add(self.build_top(), BorderLayout.NORTH)

        left = JScrollPane(self.build_form())
        left.setPreferredSize(Dimension(540, 640))
        left.setMinimumSize(Dimension(360, 320))

        right = self.build_right()
        right.setPreferredSize(Dimension(520, 640))
        right.setMinimumSize(Dimension(260, 320))

        split = JSplitPane(JSplitPane.HORIZONTAL_SPLIT, left, right)
        split.setContinuousLayout(True)
        split.setOneTouchExpandable(True)
        split.setResizeWeight(0.45)
        split.setDividerLocation(550)
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

    def build_top(self):
        panel = JPanel(BorderLayout(6, 4))
        self.configure_text_area(self.status_area, 2)
        panel.add(self.status_area, BorderLayout.NORTH)
        panel.add(self.build_buttons(), BorderLayout.SOUTH)
        return panel

    def build_buttons(self):
        panel = JPanel(FlowLayout(FlowLayout.LEFT, 5, 2))
        self.load_btn = JButton("Load Config")
        self.refresh_lists_btn = JButton("Refresh Object Lists")
        self.z_btn = JButton("Z Only")
        self.post_btn = JButton("Post XY Measure")
        self.full_btn = JButton("Full Measure")
        self.apply_btn = JButton("Apply Proposed Offsets")
        self.verify_btn = JButton("Verify Post XY")
        self.revert_btn = JButton("Revert Offsets")
        self.save_json_btn = JButton("Save JSON")
        self.save_machine_btn = JButton("Save OpenPnP Config")

        self.load_btn.addActionListener(lambda event: self.load_action())
        self.refresh_lists_btn.addActionListener(lambda event: self.refresh_selectors())
        self.z_btn.addActionListener(lambda event: self.calibration_action("Z Only", "z"))
        self.post_btn.addActionListener(lambda event: self.calibration_action("Post XY Measure", "post"))
        self.full_btn.addActionListener(lambda event: self.calibration_action("Full Measure", "full"))
        self.apply_btn.addActionListener(lambda event: self.apply_proposal_action())
        self.verify_btn.addActionListener(lambda event: self.calibration_action("Verify Post XY", "verify"))
        self.revert_btn.addActionListener(lambda event: self.revert_action())
        self.save_json_btn.addActionListener(lambda event: self.save_config_action())
        self.save_machine_btn.addActionListener(lambda event: self.save_machine_action())

        self.action_buttons = [
            self.load_btn, self.refresh_lists_btn, self.z_btn, self.post_btn,
            self.full_btn, self.apply_btn, self.verify_btn, self.revert_btn,
            self.save_json_btn, self.save_machine_btn
        ]
        for button in self.action_buttons:
            panel.add(button)
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
        panel.add(self.build_advanced_section())
        panel.add(Box.createVerticalGlue())
        return panel

    def build_selectors(self):
        panel = self.section("OpenPnP Objects")
        specs = [
            ("nozzle_name", "Needle Nozzle"),
            ("top_camera_name", "Top Camera"),
            ("contact_actuator_name", "Contact Actuator")
        ]
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
        panel = self.section("Z Touch Pad")
        for row, spec in enumerate([("x", "X mm"), ("y", "Y mm"), ("z", "Z mm"), ("rotation", "R deg")]):
            key, label = spec
            field = JTextField(10)
            self.z_fields[key] = field
            self.watch_field(field)
            self.add_row(panel, row, label, field)
        stage = JTextField(14)
        self.z_fields["result_stage"] = stage
        self.watch_field(stage)
        self.add_row(panel, 4, "Result Stage", stage)

        mode = JComboBox()
        mode.addItem("named")
        mode.addItem("xml_path")
        mode.addActionListener(lambda event: self.on_field_changed())
        self.z_combos["pipeline_mode"] = mode
        self.add_row(panel, 5, "Pipeline Source", mode)

        source = JComboBox()
        source.setEditable(True)
        source.setPreferredSize(Dimension(300, source.getPreferredSize().height))
        source.addActionListener(lambda event: self.on_field_changed())
        self.z_combos["pipeline_source"] = source
        self.add_row(panel, 6, "Pipeline Name / Path", source)

        pipeline_buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 0))
        self.test_z_btn = JButton("Test Z Pipeline")
        self.test_z_btn.addActionListener(lambda event: self.test_pipeline_action("z_touch"))
        self.motion_buttons.append(self.test_z_btn)
        pipeline_buttons.add(self.test_z_btn)
        panel.add(pipeline_buttons, self.full_width_gbc(7))

        buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 0))
        for label, source_name in [
                ("Capture Camera DRO", "camera"), ("Capture Nozzle DRO", "nozzle"),
                ("Go Camera", "camera_go"), ("Go Nozzle", "nozzle_go")]:
            button = JButton(label)
            if source_name.endswith("_go"):
                target = source_name.split("_")[0]
                button.addActionListener(lambda event, t=target: self.go_location("z_touch", t))
            else:
                button.addActionListener(lambda event, s=source_name: self.capture_location("z_touch", s))
            self.motion_buttons.append(button)
            buttons.add(button)
        panel.add(buttons, self.full_width_gbc(8))
        return panel

    def build_post_section(self):
        panel = self.section("Post XY Probe")
        fields = [
            ("x", "X mm"), ("y", "Y mm"), ("z", "Camera Focal Z mm"),
            ("rotation", "R deg"), ("result_stage", "Result Stage"),
            ("post_diameter_mm", "Post Diameter mm"),
            ("side_probe_z_offset_mm", "Side Probe Z Offset mm"),
            ("max_radial_probe_travel_mm", "Max Radial Probe Travel mm"),
            ("min_effective_contact_diameter_mm", "Min Effective Contact Diameter mm"),
            ("max_effective_contact_diameter_mm", "Max Effective Contact Diameter mm"),
            ("scan_clearance_mm", "Scan Clearance mm"),
            ("scan_overtravel_mm", "Scan Overtravel mm"),
            ("local_lift_mm", "Local Lift mm"),
            ("coarse_step_mm", "Coarse Step mm"),
            ("fine_step_mm", "Fine Step mm"),
            ("fine_backoff_mm", "Fine Backoff mm"),
            ("calibration_angles_deg", "Calibration Angles deg"),
            ("verify_angles_deg", "Verify Angles deg"),
            ("max_verify_residual_mm", "Max Verify Residual mm"),
            ("single_touch_step_mm", "Single Touch Step mm"),
            ("single_touch_speed", "Single Touch Speed"),
            ("max_verify_center_delta_mm", "Max Verify Center Delta mm"),
            ("max_verify_rms_mm", "Max Verify RMS mm"),
            ("max_verify_contact_radius_error_mm", "Max Verify Radius Delta mm"),
            ("configured_probe_z_mm", "Configured Probe Z mm"),
            ("single_probe_angle_deg", "Single Probe Angle deg")
        ]
        for row, spec in enumerate(fields):
            key, label = spec
            field = JTextField(12)
            self.post_fields[key] = field
            self.watch_field(field)
            self.add_row(panel, row, label, field)
        self.add_row(panel, len(fields), "Computed Probe Z mm", self.computed_probe_z_label)

        mode = JComboBox()
        mode.addItem("named")
        mode.addItem("xml_path")
        mode.addActionListener(lambda event: self.on_field_changed())
        self.post_combos["pipeline_mode"] = mode
        self.add_row(panel, len(fields) + 1, "Pipeline Source", mode)

        source = JComboBox()
        source.setEditable(True)
        source.setPreferredSize(Dimension(300, source.getPreferredSize().height))
        source.addActionListener(lambda event: self.on_field_changed())
        self.post_combos["pipeline_source"] = source
        self.add_row(panel, len(fields) + 2, "Pipeline Name / Path", source)

        calibration_profile = JComboBox()
        for item in ["coarse_fine", "single_touch"]:
            calibration_profile.addItem(item)
        calibration_profile.addActionListener(lambda event: self.on_field_changed())
        self.post_combos["calibration_probe_profile"] = calibration_profile
        self.add_row(panel, len(fields) + 3, "Calibration Probe Profile", calibration_profile)

        verify_profile = JComboBox()
        for item in ["single_touch", "coarse_fine"]:
            verify_profile.addItem(item)
        verify_profile.addActionListener(lambda event: self.on_field_changed())
        self.post_combos["verify_probe_profile"] = verify_profile
        self.add_row(panel, len(fields) + 4, "Verify Probe Profile", verify_profile)

        use_configured = JCheckBox("Use Configured Probe Z")
        use_configured.addActionListener(lambda event: self.on_field_changed())
        self.post_checks["allow_configured_probe_z"] = use_configured
        panel.add(use_configured, self.full_width_gbc(len(fields) + 5))

        apply_xy = JCheckBox("Apply XY during Full/Post Measure")
        apply_xy.addActionListener(lambda event: self.on_field_changed())
        self.post_checks["apply_xy_offsets"] = apply_xy
        panel.add(apply_xy, self.full_width_gbc(len(fields) + 6))

        run_fast_verify = JCheckBox("Run Fast Verify After Cardinal")
        run_fast_verify.addActionListener(lambda event: self.on_field_changed())
        self.post_checks["run_fast_verify_after_cardinal"] = run_fast_verify
        panel.add(run_fast_verify, self.full_width_gbc(len(fields) + 7))

        use_verify = JCheckBox("Use Verify Fit For Proposal")
        use_verify.addActionListener(lambda event: self.on_field_changed())
        self.post_checks["use_verify_fit_for_proposal"] = use_verify
        panel.add(use_verify, self.full_width_gbc(len(fields) + 8))

        use_coarse_step = JCheckBox("Single Touch Uses Coarse Step")
        use_coarse_step.addActionListener(lambda event: self.on_field_changed())
        self.post_checks["single_touch_use_coarse_step"] = use_coarse_step
        panel.add(use_coarse_step, self.full_width_gbc(len(fields) + 9))

        probe_buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 0))
        self.test_post_btn = JButton("Test Post Pipeline")
        self.single_angle_btn = JButton("Probe Single Angle")
        self.test_post_btn.addActionListener(lambda event: self.test_pipeline_action("post_xy"))
        self.single_angle_btn.addActionListener(lambda event: self.calibration_action("Probe Single Angle", "post_single"))
        for button in [self.test_post_btn, self.single_angle_btn]:
            self.motion_buttons.append(button)
            probe_buttons.add(button)
        panel.add(probe_buttons, self.full_width_gbc(len(fields) + 10))

        buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 0))
        for label, source_name in [
                ("Capture Camera DRO", "camera"), ("Capture Nozzle DRO", "nozzle"),
                ("Go Camera", "camera_go"), ("Go Nozzle", "nozzle_go")]:
            button = JButton(label)
            if source_name.endswith("_go"):
                target = source_name.split("_")[0]
                button.addActionListener(lambda event, t=target: self.go_location("post_xy", t))
            else:
                button.addActionListener(lambda event, s=source_name: self.capture_location("post_xy", s))
            self.motion_buttons.append(button)
            buttons.add(button)
        panel.add(buttons, self.full_width_gbc(len(fields) + 11))
        return panel

    def build_advanced_section(self):
        panel = self.section("Motion / Z Probe")
        specs = [
            ("safe_z_mm", "Safe Z mm"),
            ("travel_speed", "Travel Speed"),
            ("probe_speed", "Probe Speed"),
            ("fine_probe_speed", "Fine Probe Speed"),
            ("max_apply_delta_mm", "Max Apply Delta mm"),
            ("xy_offset_sign", "XY Offset Sign"),
            ("z_offset_sign", "Z Offset Sign"),
            ("z_probe.start_above_mm", "Z Start Above mm"),
            ("z_probe.max_descent_mm", "Z Max Descent mm"),
            ("z_probe.coarse_step_mm", "Z Coarse Step mm"),
            ("z_probe.fine_step_mm", "Z Fine Step mm"),
            ("z_probe.retract_after_contact_mm", "Z Retract After Contact mm"),
            ("z_probe.verify_retract_mm", "Z Verify Retract mm"),
            ("z_probe.verify_start_above_mm", "Z Verify Start Above mm"),
            ("z_probe.verify_coarse_step_mm", "Z Verify Coarse Step mm"),
            ("z_probe.max_fine_backtrack_mm", "Z Max Fine Backtrack mm"),
            ("z_probe.release_retract_step_mm", "Z Release Step mm"),
            ("z_probe.max_release_retract_mm", "Z Max Release mm")
        ]
        for row, spec in enumerate(specs):
            key, label = spec
            field = JTextField(10)
            self.advanced_fields[key] = field
            self.watch_field(field)
            self.add_row(panel, row, label, field)
        verify_mode = JComboBox()
        for item in ["fast", "full"]:
            verify_mode.addItem(item)
        verify_mode.addActionListener(lambda event: self.on_field_changed())
        self.advanced_combos["z_probe.verify_mode"] = verify_mode
        self.add_row(panel, len(specs), "Z Verify Mode", verify_mode)
        verify_enabled = JCheckBox("Z Verify Enabled")
        verify_enabled.addActionListener(lambda event: self.on_field_changed())
        self.advanced_checks["z_probe.verify_enabled"] = verify_enabled
        panel.add(verify_enabled, self.full_width_gbc(len(specs) + 1))
        log_each_step = JCheckBox("Z Log Each Step")
        log_each_step.addActionListener(lambda event: self.on_field_changed())
        self.advanced_checks["z_probe.log_each_step"] = log_each_step
        panel.add(log_each_step, self.full_width_gbc(len(specs) + 2))
        return panel

    def build_right(self):
        panel = JPanel(BorderLayout(6, 6))
        current = JPanel(GridBagLayout())
        current.setBorder(BorderFactory.createTitledBorder("Live Values"))
        self.configure_text_area(self.offsets_area, 3)
        self.add_row(current, 0, "Nozzle Offsets", self.offsets_area)
        self.add_row(current, 1, "State", self.state_label)
        self.add_row(current, 2, "JSON Dirty", self.dirty_label)
        panel.add(current, BorderLayout.NORTH)

        image_panel = JPanel(BorderLayout())
        image_panel.setBorder(BorderFactory.createTitledBorder("Image Preview"))
        image_buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 0))
        self.refresh_image_btn = JButton("Refresh Image")
        self.refresh_image_btn.addActionListener(lambda event: self.refresh_image_action())
        self.motion_buttons.append(self.refresh_image_btn)
        image_buttons.add(self.refresh_image_btn)
        image_panel.add(image_buttons, BorderLayout.NORTH)
        self.image_label.setPreferredSize(Dimension(520, 320))
        self.image_label.setMinimumSize(Dimension(160, 120))
        image_panel.add(self.image_label, BorderLayout.CENTER)
        panel.add(image_panel, BorderLayout.CENTER)

        self.result_area.setEditable(False)
        self.result_area.setRows(14)
        self.result_area.setLineWrap(False)
        result_scroll = JScrollPane(self.result_area)
        result_scroll.setBorder(BorderFactory.createTitledBorder("Results / Proposed Offsets"))
        result_scroll.setPreferredSize(Dimension(520, 260))
        result_scroll.setMinimumSize(Dimension(160, 120))
        panel.add(result_scroll, BorderLayout.SOUTH)
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
        label_obj.setPreferredSize(Dimension(200, label_obj.getPreferredSize().height))
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
        area.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 0))

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

    def set_state(self, state):
        self.state = state
        self.state_label.setText(state)
        self.update_dirty()
        self.update_buttons()

    def update_buttons(self):
        for button in self.action_buttons + self.motion_buttons:
            button.setEnabled(not self.worker_running)
        self.revert_btn.setEnabled((not self.worker_running) and self.pre_run_offsets is not None)
        self.apply_btn.setEnabled((not self.worker_running) and self.last_post_proposal is not None)

    def on_field_changed(self):
        if self.loading_ui:
            return
        self.update_computed_probe_z()
        self.update_dirty()

    def update_dirty(self):
        try:
            dirty = compact_json(self.cleaned_config(self.collect_config(False))) != self.saved_snapshot
        except:
            dirty = True
        self.dirty_label.setText("DIRTY" if dirty else "clean")

    def update_computed_probe_z(self):
        try:
            z = self.parse_float(self.post_fields["z"].getText(), "post_xy.location.z")
            offset = self.parse_float(self.post_fields["side_probe_z_offset_mm"].getText(),
                "post_xy.side_probe_z_offset_mm")
            if self.post_checks["allow_configured_probe_z"].isSelected():
                text = self.post_fields["configured_probe_z_mm"].getText().strip()
                if text:
                    self.computed_probe_z_label.setText("%.4f (configured)" %
                        self.parse_float(text, "post_xy.configured_probe_z_mm"))
                    return
            text = "%.4f" % (z + offset)
            if offset < 0.05:
                text += "  warning: low offset"
            elif offset > 1.0:
                text += "  warning: high offset"
            self.computed_probe_z_label.setText(text)
        except:
            self.computed_probe_z_label.setText("unavailable")

    def refresh_selectors(self):
        try:
            set_combo_items(self.selector_combos["nozzle_name"], available_nozzle_names(),
                combo_text(self.selector_combos["nozzle_name"]))
            set_combo_items(self.selector_combos["top_camera_name"], available_camera_names(),
                combo_text(self.selector_combos["top_camera_name"]))
            set_combo_items(self.selector_combos["contact_actuator_name"], available_actuator_names(),
                combo_text(self.selector_combos["contact_actuator_name"]))
            names = available_vision_settings_names()
            set_combo_items(self.z_combos["pipeline_source"], names,
                combo_text(self.z_combos["pipeline_source"]))
            set_combo_items(self.post_combos["pipeline_source"], names,
                combo_text(self.post_combos["pipeline_source"]))
            self.append_log("Refreshed OpenPnP object and pipeline lists")
        except Exception, e:
            self.show_error("Refresh lists failed", e)

    def cleaned_config(self, cfg):
        return needle_calibration.public_clean_config(cfg)

    def load_into_ui(self, cfg):
        self.loading_ui = True
        try:
            for key in self.selector_combos:
                self.selector_combos[key].setSelectedItem(str(cfg.get(key, "")))

            zfid = cfg.get("fiducials", {}).get("z_touch", {})
            loc = zfid.get("location", {})
            for axis in ["x", "y", "z", "rotation"]:
                self.z_fields[axis].setText(str(loc.get(axis, 0.0)))
            self.z_fields["result_stage"].setText(str(zfid.get("result_stage", "results")))
            zpipe = zfid.get("pipeline", {})
            zmode = str(zpipe.get("mode", "named") or "named")
            if zmode not in ["named", "xml_path"]:
                zmode = "named"
            self.z_combos["pipeline_mode"].setSelectedItem(zmode)
            self.z_combos["pipeline_source"].setSelectedItem(str(
                zpipe.get("name", "") if zmode == "named" else zpipe.get("path", "")))

            post = cfg.get("post_xy", {})
            post_loc = post.get("location", {})
            for axis in ["x", "y", "z", "rotation"]:
                self.post_fields[axis].setText(str(post_loc.get(axis, 0.0)))
            for key in self.post_fields:
                if key in ["x", "y", "z", "rotation"]:
                    continue
                value = post.get(key, "")
                if key == "single_probe_angle_deg":
                    value = post.get(key, 0.0)
                if key in ["calibration_angles_deg", "verify_angles_deg"]:
                    value = json.dumps(value)
                self.post_fields[key].setText("" if value is None else str(value))
            ppipe = post.get("pipeline", {})
            pmode = str(ppipe.get("mode", "named") or "named")
            if pmode not in ["named", "xml_path"]:
                pmode = "named"
            self.post_combos["pipeline_mode"].setSelectedItem(pmode)
            self.post_combos["pipeline_source"].setSelectedItem(str(
                ppipe.get("name", "") if pmode == "named" else ppipe.get("path", "")))
            self.post_combos["calibration_probe_profile"].setSelectedItem(str(
                post.get("calibration_probe_profile", "coarse_fine") or "coarse_fine"))
            self.post_combos["verify_probe_profile"].setSelectedItem(str(
                post.get("verify_probe_profile", "single_touch") or "single_touch"))
            self.post_checks["allow_configured_probe_z"].setSelected(
                bool(post.get("allow_configured_probe_z", False)))
            self.post_checks["apply_xy_offsets"].setSelected(
                bool(post.get("apply_xy_offsets", False)))
            self.post_checks["run_fast_verify_after_cardinal"].setSelected(
                bool(post.get("run_fast_verify_after_cardinal", True)))
            self.post_checks["use_verify_fit_for_proposal"].setSelected(
                bool(post.get("use_verify_fit_for_proposal", False)))
            self.post_checks["single_touch_use_coarse_step"].setSelected(
                bool(post.get("single_touch_use_coarse_step", True)))

            for key in self.advanced_fields:
                value = self.get_nested_value(cfg, key)
                self.advanced_fields[key].setText("" if value is None else str(value))
            for key in self.advanced_combos:
                value = self.get_nested_value(cfg, key)
                self.advanced_combos[key].setSelectedItem(str(value or "fast"))
            for key in self.advanced_checks:
                self.advanced_checks[key].setSelected(bool(self.get_nested_value(cfg, key)))
        finally:
            self.loading_ui = False
        self.update_computed_probe_z()
        self.update_dirty()

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
        cfg = deep_merge(needle_calibration.DEFAULT_CONFIG, self.cfg)
        cfg["nozzle_name"] = combo_text(self.selector_combos["nozzle_name"])
        cfg["top_camera_name"] = combo_text(self.selector_combos["top_camera_name"])
        cfg["contact_actuator_name"] = combo_text(self.selector_combos["contact_actuator_name"])
        cfg["fiducials"]["z_touch"] = {
            "location": self.collect_location(self.z_fields, "z_touch"),
            "result_stage": self.z_fields["result_stage"].getText().strip(),
            "pipeline": self.collect_pipeline(self.z_combos)
        }
        post = cfg["post_xy"]
        post["location"] = self.collect_location(self.post_fields, "post_xy")
        for key in [
                "post_diameter_mm", "side_probe_z_offset_mm", "max_radial_probe_travel_mm",
                "min_effective_contact_diameter_mm", "max_effective_contact_diameter_mm",
                "scan_clearance_mm", "scan_overtravel_mm", "local_lift_mm",
                "coarse_step_mm", "fine_step_mm", "fine_backoff_mm",
                "max_verify_residual_mm", "single_touch_step_mm", "single_touch_speed",
                "max_verify_center_delta_mm", "max_verify_rms_mm",
                "max_verify_contact_radius_error_mm", "single_probe_angle_deg"]:
            post[key] = self.parse_float(self.post_fields[key].getText(), "post_xy.%s" % key)
        post["configured_probe_z_mm"] = self.parse_optional_float(
            self.post_fields["configured_probe_z_mm"].getText(), "post_xy.configured_probe_z_mm")
        post["allow_configured_probe_z"] = bool(self.post_checks["allow_configured_probe_z"].isSelected())
        post["apply_xy_offsets"] = bool(self.post_checks["apply_xy_offsets"].isSelected())
        post["run_fast_verify_after_cardinal"] = bool(
            self.post_checks["run_fast_verify_after_cardinal"].isSelected())
        post["use_verify_fit_for_proposal"] = bool(
            self.post_checks["use_verify_fit_for_proposal"].isSelected())
        post["single_touch_use_coarse_step"] = bool(
            self.post_checks["single_touch_use_coarse_step"].isSelected())
        post["result_stage"] = self.post_fields["result_stage"].getText().strip()
        post["pipeline"] = self.collect_pipeline(self.post_combos)
        post["calibration_angles_deg"] = self.parse_angles_field(
            self.post_fields["calibration_angles_deg"].getText(), "post_xy.calibration_angles_deg")
        post["verify_angles_deg"] = self.parse_angles_field(
            self.post_fields["verify_angles_deg"].getText(), "post_xy.verify_angles_deg")
        post["calibration_probe_profile"] = str(
            self.post_combos["calibration_probe_profile"].getSelectedItem()).strip()
        post["verify_probe_profile"] = str(
            self.post_combos["verify_probe_profile"].getSelectedItem()).strip()
        post["probe_z_mode"] = "post_location_plus_offset"
        for key in self.advanced_fields:
            self.set_nested_value(cfg, key, self.parse_float(self.advanced_fields[key].getText(), key))
        for key in self.advanced_combos:
            self.set_nested_value(cfg, key, str(self.advanced_combos[key].getSelectedItem()).strip())
        for key in self.advanced_checks:
            self.set_nested_value(cfg, key, bool(self.advanced_checks[key].isSelected()))
        if validate:
            self.validate_config(cfg)
        return cfg

    def collect_pipeline(self, combos):
        mode = str(combos["pipeline_mode"].getSelectedItem()).strip()
        source = combo_text(combos["pipeline_source"])
        if mode == "xml_path":
            return {"mode": "xml_path", "path": source, "name": ""}
        return {"mode": "named", "name": source, "path": ""}

    def collect_location(self, fields, label):
        return {
            "x": self.parse_float(fields["x"].getText(), "%s.location.x" % label),
            "y": self.parse_float(fields["y"].getText(), "%s.location.y" % label),
            "z": self.parse_float(fields["z"].getText(), "%s.location.z" % label),
            "rotation": self.parse_float(fields["rotation"].getText(), "%s.location.rotation" % label)
        }

    def parse_float(self, text, label):
        try:
            return float(str(text).strip())
        except:
            raise Exception("%s must be a number" % label)

    def parse_optional_float(self, text, label):
        if str(text or "").strip() == "":
            return None
        return self.parse_float(text, label)

    def parse_angles_field(self, text, label):
        try:
            value = json.loads(str(text or "").strip())
            if not isinstance(value, list):
                raise Exception("not a list")
            return [float(item) for item in value]
        except:
            try:
                return [float(part.strip()) for part in str(text or "").split(",") if part.strip()]
            except Exception, e:
                raise Exception("%s must be JSON list or comma separated numbers: %s" % (label, e))

    def validate_pipeline(self, pipe, label):
        mode = str(pipe.get("mode", "named") or "named")
        if mode == "named":
            name = str(pipe.get("name", "") or "").strip()
            if name == "":
                raise Exception("%s OpenPnP pipeline name is required" % label)
            if needle_calibration.find_named_vision_settings(name) is None:
                raise Exception("%s OpenPnP pipeline not found: %s" % (label, name))
        else:
            path = str(pipe.get("path", "") or "").strip()
            if path == "":
                raise Exception("%s XML pipeline path is required" % label)
            resolved = resolve_pipeline_path(path)
            if not os.path.exists(resolved):
                raise Exception("%s XML pipeline path does not exist: %s" % (label, resolved))
            pipe["path"] = resolved

    def validate_config(self, cfg):
        if str(cfg.get("nozzle_name", "")).strip() == "":
            raise Exception("Needle nozzle is required")
        if str(cfg.get("top_camera_name", "")).strip() == "":
            raise Exception("Top camera is required")
        if str(cfg.get("contact_actuator_name", "")).strip() == "":
            raise Exception("Contact actuator is required")
        self.validate_pipeline(cfg["fiducials"]["z_touch"]["pipeline"], "Z touch")
        self.validate_pipeline(cfg["post_xy"]["pipeline"], "Post XY")

    def load_action(self):
        try:
            self.cfg = load_config_file()
            self.saved_snapshot = compact_json(self.cleaned_config(self.cfg))
            self.load_into_ui(self.cfg)
            self.refresh_current_values()
            self.set_state(IDLE)
            self.append_log("Loaded %s" % CONFIG_PATH)
            for warning in self.cfg.get("_migration_warnings", []):
                self.append_log(warning)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Load failed", e)

    def refresh_current_values(self):
        try:
            cfg = self.collect_config(False)
            nozzle = needle_calibration.find_nozzle(cfg, None, get_machine())
            offsets = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
            self.offsets_area.setText(
                "Nozzle: %s\nX %.4f  Y %.4f  Z %.4f  R %.4f" %
                (object_name(nozzle), offsets.getX(), offsets.getY(),
                 offsets.getZ(), offsets.getRotation()))
            self.status_area.setText("Ready")
            self.update_dirty()
        except Exception, e:
            self.offsets_area.setText("Unavailable: %s" % e)

    def set_location_fields(self, role, loc):
        loc = loc.convertToUnits(LengthUnit.Millimeters)
        fields = self.z_fields if role == "z_touch" else self.post_fields
        fields["x"].setText("%.4f" % loc.getX())
        fields["y"].setText("%.4f" % loc.getY())
        fields["z"].setText("%.4f" % loc.getZ())
        fields["rotation"].setText("%.4f" % loc.getRotation())

    def capture_location(self, role, source):
        try:
            cfg = self.collect_config(False)
            machine_obj = get_machine()
            if source == "camera":
                movable = needle_calibration.find_top_camera(cfg, machine_obj)
            else:
                movable = needle_calibration.find_nozzle(cfg, None, machine_obj)
            loc = movable.getLocation().convertToUnits(LengthUnit.Millimeters)
            self.set_location_fields(role, loc)
            self.append_log("Captured %s from %s DRO" % (role, source))
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Capture failed", e)

    def go_location(self, role, target_type):
        try:
            cfg = self.collect_config(False)
            loc = cfg["fiducials"]["z_touch"]["location"] if role == "z_touch" else cfg["post_xy"]["location"]
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Move setup failed", e)
            return
        self.run_worker("Go %s %s" % (role, target_type),
            lambda: self.do_go_location(cfg, loc, target_type),
            self.go_done)

    def do_go_location(self, cfg, loc, target_type):
        machine_obj = require_machine_execute()

        class Task(Callable):
            def call(task_self):
                if target_type == "camera":
                    movable = needle_calibration.find_top_camera(cfg, machine_obj)
                else:
                    movable = needle_calibration.find_nozzle(cfg, None, machine_obj)
                needle_calibration.move_safe(movable, mm_location_from_dict(loc))
                return {"ok": True, "target": target_type}

        return machine_obj.execute(Task())

    def go_done(self, result):
        self.refresh_current_values()
        self.append_log("Move finished: %s" % result.get("target", ""))

    def refresh_image_action(self):
        try:
            cfg = self.collect_config(False)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Refresh image setup failed", e)
            return
        self.run_worker("Refresh Image", lambda: self.do_refresh_image(cfg), self.image_done)

    def do_refresh_image(self, cfg):
        machine_obj = require_machine_execute()
        gui_self = self

        class Task(Callable):
            def call(task_self):
                camera = needle_calibration.find_top_camera(cfg, machine_obj)
                loc = mm_location_from_dict(cfg["post_xy"]["location"])
                needle_calibration.move_safe(camera, loc)
                image = camera.settleAndCapture()
                return {"ok": True, "image": image, "feature": None}

        return machine_obj.execute(Task())

    def image_done(self, payload):
        self.update_image_preview(payload.get("image"), payload.get("feature"))
        self.append_log("Image refreshed")
        self.refresh_current_values()

    def test_pipeline_action(self, role):
        try:
            cfg = self.collect_config(True)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Pipeline setup failed", e)
            return
        self.run_worker("Test Pipeline %s" % role,
            lambda: self.do_test_pipeline(cfg, role),
            self.test_pipeline_done)

    def do_test_pipeline(self, cfg, role):
        machine_obj = require_machine_execute()
        gui_self = self

        class Task(Callable):
            def call(task_self):
                if role == "post_xy":
                    return needle_calibration.capture_post_xy(
                        cfg, None, machine_obj, gui_self.append_log)
                return needle_calibration.capture_fiducial_features(
                    cfg, "z_touch", None, machine_obj, gui_self.append_log)

        return machine_obj.execute(Task())

    def test_pipeline_done(self, capture):
        self.last_result = capture
        feature = capture.get("feature", None)
        if feature is None and len(capture.get("features", [])):
            feature = capture.get("features", [])[0]
        self.update_image_preview(capture.get("image", None), feature)
        self.result_area.setText(self.format_capture(capture))
        self.append_log("Pipeline test finished for %s" % capture.get("role", ""))
        self.refresh_current_values()

    def format_capture(self, capture):
        lines = []
        lines.append("role: %s" % capture.get("role", ""))
        lines.append("camera: %s" % capture.get("camera", ""))
        lines.append("features: %d" % len(capture.get("features", [])))
        for i, feat in enumerate(capture.get("features", [])):
            lines.append("%d: X %.4f  Y %.4f  px=%s py=%s diameter_mm=%s" %
                (i + 1, feat.get("x_mm", 0.0), feat.get("y_mm", 0.0),
                 str(feat.get("x_px", "")), str(feat.get("y_px", "")),
                 str(feat.get("diameter_mm", ""))))
        return "\n".join(lines)

    def calibration_action(self, title, action):
        try:
            cfg = self.collect_config(True)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("%s setup failed" % title, e)
            return
        self.run_worker(title, lambda: self.do_calibration_run(action, cfg), self.calibration_done)

    def do_calibration_run(self, action, cfg):
        machine_obj = require_machine_execute()
        gui_self = self

        class Task(Callable):
            def call(task_self):
                try:
                    nozzle = needle_calibration.find_nozzle(cfg, None, machine_obj)
                    old_offsets = needle_calibration.loc_to_dict(
                        nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
                    if action == "z":
                        result = needle_calibration.run_z_only(cfg, nozzle, machine_obj, gui_self.append_log)
                    elif action == "post":
                        result = needle_calibration.run_post_xy_only(cfg, nozzle, machine_obj, gui_self.append_log)
                    elif action == "post_single":
                        angle = float(cfg.get("post_xy", {}).get("single_probe_angle_deg", 0.0))
                        result = needle_calibration.run_post_xy_debug_single_angle(
                            cfg, angle, nozzle, machine_obj, gui_self.append_log)
                    elif action == "verify":
                        result = needle_calibration.verify_post_xy_after_apply(cfg, nozzle, machine_obj, gui_self.append_log)
                    else:
                        result = needle_calibration.run_full_calibration(cfg, nozzle, machine_obj, gui_self.append_log)
                    return {"result": result, "old_offsets": old_offsets, "cfg": cfg}
                finally:
                    restore_machine_speed(machine_obj, gui_self.append_log)

        return machine_obj.execute(Task())

    def calibration_done(self, payload):
        result = payload.get("result")
        self.last_result = result
        self.pre_run_offsets = payload.get("old_offsets")
        cfg_after = payload.get("cfg")
        if cfg_after is not None:
            self.cfg = cfg_after
            self.load_into_ui(self.cfg)
        self.capture_result_image(result)
        self.result_area.setText(self.format_result_summary(result))
        if result is not None and result.get("stage") == "post_xy" and result.get("ok", False):
            self.last_post_proposal = result.get("proposal")
        elif result is not None and result.get("stage") == "full_calibration" and result.get("post_xy", {}).get("ok", False):
            self.last_post_proposal = result.get("post_xy", {}).get("proposal")
        self.refresh_current_values()
        if result is not None and result.get("ok", False):
            self.set_state(READY)
            self.append_log("Calibration finished")
        else:
            self.set_state(ERROR)
            self.append_log("Calibration failed: %s" % self.safe_result_message(result))

    def capture_result_image(self, result):
        try:
            cap = None
            if result is None:
                return
            if result.get("stage") in ["post_xy", "post_xy_single_angle"]:
                cap = result.get("details", {}).get("camera_capture")
            elif result.get("stage") == "full_calibration":
                cap = result.get("post_xy", {}).get("details", {}).get("camera_capture")
            elif result.get("stage") == "post_xy_verify":
                cap = result.get("details", {}).get("camera_capture")
            if cap is not None:
                feature = cap.get("feature", None)
                self.update_image_preview(cap.get("image", None), feature)
        except Exception, e:
            self.append_log("Image preview update skipped: %s" % e)

    def safe_result_message(self, result):
        if result is None:
            return "No result"
        return str(result.get("message", ""))

    def format_result_summary(self, result):
        if result is None:
            return "No result"
        lines = []
        lines.append("ok: %s" % result.get("ok"))
        lines.append("message: %s" % result.get("message", ""))
        z = result.get("z_touch") or (result if result.get("stage") == "z_touch" else None)
        post = result.get("post_xy") or (result if result.get("stage") in ["post_xy", "post_xy_verify", "post_xy_single_angle"] else None)
        if z is not None:
            touch = z.get("details", {}).get("verify_touch", z.get("details", {}).get("first_touch", {}))
            contact = touch.get("contact", {})
            lines.append("last Z touch: %.6f" % float(contact.get("z", 0.0)))
            new_z = z.get("new_offsets", {})
            if new_z:
                lines.append("applied Z offset: %.6f" % float(new_z.get("z", 0.0)))
        if post is not None:
            details = post.get("details", {})
            proposal = post.get("proposal", {})
            measured = details.get("measured_center_delta", {})
            offset_delta = proposal.get("offset_delta", {})
            proposed = proposal.get("proposed_offsets", {})
            eff = details.get("effective_electrical_contact", details.get("estimated_needle", {}))
            fit = details.get("filtered_circle_fit", details.get("circle_fit", {}))
            raw = details.get("raw_circle_fit", {})
            cal_pass = details.get("calibration_pass") or {}
            fast_verify = details.get("fast_verify") or {}
            lines.append("post side probe Z: %.6f" % float(details.get("post_side_probe_z_mm", 0.0)))
            lines.append("proposal source: %s" %
                str(post.get("proposal_source", details.get("proposal_source", ""))))
            lines.append("measured center delta dx/dy: %.6f / %.6f" %
                (float(measured.get("x", proposal.get("dx_mm", 0.0))),
                 float(measured.get("y", proposal.get("dy_mm", 0.0)))))
            lines.append("proposed offset change dX/dY: %.6f / %.6f" %
                (float(offset_delta.get("x", 0.0)), float(offset_delta.get("y", 0.0))))
            if proposed:
                lines.append("proposed offsets: X %.6f Y %.6f Z %.6f" %
                    (float(proposed.get("x", 0.0)), float(proposed.get("y", 0.0)),
                     float(proposed.get("z", 0.0))))
            lines.append("effective electrical contact diameter: %.6f" %
                float(eff.get("diameter_mm", 0.0)))
            lines.append("circle fit filtered RMS/max: %.6f / %.6f" %
                (float(fit.get("rms_error_mm", 0.0)), float(fit.get("max_error_mm", 0.0))))
            if cal_pass:
                cfit = cal_pass.get("circle_fit", {})
                cc = cfit.get("center", {})
                lines.append("calibration pass center/radius/RMS: %.6f / %.6f / %.6f / %.6f" %
                    (float(cc.get("x", 0.0)), float(cc.get("y", 0.0)),
                     float(cfit.get("radius_mm", 0.0)),
                     float(cfit.get("rms_error_mm", 0.0))))
            if fast_verify.get("enabled", False):
                vfit = fast_verify.get("circle_fit", {})
                vc = vfit.get("center", {})
                comp = fast_verify.get("comparison_to_calibration", {})
                cdelta = comp.get("center_delta", {})
                lines.append("fast verify center/radius/RMS: %.6f / %.6f / %.6f / %.6f" %
                    (float(vc.get("x", 0.0)), float(vc.get("y", 0.0)),
                     float(vfit.get("radius_mm", 0.0)),
                     float(vfit.get("rms_error_mm", 0.0))))
                lines.append("verify vs calibration center delta: %.6f / %.6f / %.6f" %
                    (float(cdelta.get("x", 0.0)), float(cdelta.get("y", 0.0)),
                     float(cdelta.get("r", 0.0))))
                lines.append("fast verify ok: %s" % fast_verify.get("ok"))
            if raw:
                lines.append("circle fit raw RMS/max: %.6f / %.6f" %
                    (float(raw.get("rms_error_mm", 0.0)), float(raw.get("max_error_mm", 0.0))))
            warnings = details.get("diameter_warnings", [])
            warnings = list(warnings)
            if fast_verify.get("warnings", None) is not None:
                warnings.extend(fast_verify.get("warnings", []))
            for warning in warnings:
                lines.append("warning: %s" % warning)
            lines.append("applied: %s" % post.get("applied", details.get("applied") is not None))
        final_offsets = result.get("final_offsets") or result.get("new_offsets")
        if final_offsets is not None:
            lines.append("current/final offsets: X %.6f Y %.6f Z %.6f" %
                (float(final_offsets.get("x", 0.0)), float(final_offsets.get("y", 0.0)),
                 float(final_offsets.get("z", 0.0))))
        lines.append("")
        lines.append(json.dumps(json_safe(result), sort_keys=True, indent=2))
        return "\n".join(lines)

    def apply_proposal_action(self):
        if self.last_post_proposal is None:
            return
        try:
            cfg = self.collect_config(False)
            proposal = deep_copy(self.last_post_proposal)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Apply setup failed", e)
            return
        self.run_worker("Apply Proposed Offsets",
            lambda: self.do_apply_proposal(cfg, proposal),
            self.apply_done)

    def do_apply_proposal(self, cfg, proposal):
        machine_obj = require_machine_execute()
        gui_self = self

        class Task(Callable):
            def call(task_self):
                nozzle = needle_calibration.find_nozzle(cfg, None, machine_obj)
                before = needle_calibration.loc_to_dict(
                    nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
                applied = needle_calibration.apply_proposed_xy_offsets(
                    cfg, nozzle, proposal, gui_self.append_log)
                after = needle_calibration.loc_to_dict(
                    nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
                return {"ok": True, "stage": "apply_post_xy", "before": before,
                    "after": after, "applied": applied}

        return machine_obj.execute(Task())

    def apply_done(self, result):
        self.last_result = result
        self.result_area.setText(json.dumps(json_safe(result), sort_keys=True, indent=2))
        self.refresh_current_values()
        self.set_state(READY)
        self.append_log("Applied proposed XY offsets in memory")

    def save_config_action(self):
        try:
            cfg = self.collect_config(False)
            save_config_file(cfg)
            self.cfg = cfg
            self.saved_snapshot = compact_json(self.cleaned_config(cfg))
            self.refresh_current_values()
            self.set_state(SAVED)
            self.append_log("Saved JSON config only; OpenPnP configuration was not saved")
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Save JSON failed", e)

    def save_machine_action(self):
        try:
            Configuration.get().save()
            self.refresh_current_values()
            self.set_state(SAVED)
            self.append_log("Saved current OpenPnP machine configuration, including in-memory offsets")
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Save OpenPnP Config failed", e)

    def revert_action(self):
        if self.pre_run_offsets is None:
            return
        try:
            cfg = self.collect_config(False)
            backup = deep_copy(self.pre_run_offsets)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Revert setup failed", e)
            return
        self.run_worker("Revert Offsets", lambda: self.do_revert(cfg, backup), self.revert_done)

    def do_revert(self, cfg, backup):
        machine_obj = require_machine_execute()

        class Task(Callable):
            def call(task_self):
                nozzle = needle_calibration.find_nozzle(cfg, None, machine_obj)
                before = needle_calibration.loc_to_dict(
                    nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
                nozzle.setHeadOffsets(mm_location_from_dict(backup))
                after = needle_calibration.loc_to_dict(
                    nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
                return {"ok": True, "before": before, "after": after}

        return machine_obj.execute(Task())

    def revert_done(self, result):
        self.last_result = result
        self.result_area.setText(json.dumps(json_safe(result), sort_keys=True, indent=2))
        self.refresh_current_values()
        self.set_state(IDLE)
        self.append_log("Restored pre-run XYZ head offsets in memory")

    def update_image_preview(self, image, feature):
        try:
            if image is None:
                if self.last_capture_image is None:
                    self.image_label.setText("No image")
                    self.image_label.setIcon(None)
                return
            self.last_capture_image = image
            self.last_overlay_feature = feature
            w = self.image_label.getWidth()
            h = self.image_label.getHeight()
            if w <= 1:
                w = 520
            if h <= 1:
                h = 320
            try:
                preview = self.draw_scaled_preview(image, feature, w, h, True)
            except Exception, overlay_error:
                self.append_log("Image overlay warning; showing raw image: %s" % overlay_error)
                preview = self.draw_scaled_preview(image, None, w, h, False)
            self.image_label.setIcon(ImageIcon(preview))
            self.image_label.setText("")
        except Exception, e:
            self.append_log("Image preview failed: %s" % e)

    def draw_scaled_preview(self, image, feature, max_w, max_h, draw_overlay):
        image_w = int(image.getWidth())
        image_h = int(image.getHeight())
        if image_w <= 0 or image_h <= 0:
            raise Exception("Captured image has invalid dimensions")
        scale = min(float(max_w) / float(image_w), float(max_h) / float(image_h))
        if scale <= 0.0:
            scale = 1.0
        scaled_w = max(1, int(round(float(image_w) * scale)))
        scaled_h = max(1, int(round(float(image_h) * scale)))
        x0 = int((int(max_w) - scaled_w) / 2)
        y0 = int((int(max_h) - scaled_h) / 2)
        out = BufferedImage(int(max_w), int(max_h), BufferedImage.TYPE_INT_RGB)
        g = out.createGraphics()
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR)
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
        g.setColor(Color.DARK_GRAY)
        g.fillRect(0, 0, int(max_w), int(max_h))
        g.drawImage(image, x0, y0, scaled_w, scaled_h, None)
        if draw_overlay and feature is not None and feature.get("x_px", None) is not None:
            x = int(round(x0 + float(feature.get("x_px")) * scale))
            y = int(round(y0 + float(feature.get("y_px")) * scale))
            r = None
            if feature.get("radius_px", None) is not None:
                r = int(round(float(feature.get("radius_px")) * scale))
            elif feature.get("diameter_px", None) is not None:
                r = int(round(float(feature.get("diameter_px")) * scale / 2.0))
            g.setColor(Color.GREEN)
            g.setStroke(BasicStroke(2.0))
            if r is not None and r > 0:
                g.drawOval(x - r, y - r, r * 2, r * 2)
            g.drawLine(x - 10, y, x + 10, y)
            g.drawLine(x, y - 10, x, y + 10)
            g.setColor(Color.YELLOW)
            g.drawString("X %.3f Y %.3f D %s" %
                (float(feature.get("x_mm", 0.0)), float(feature.get("y_mm", 0.0)),
                 str(feature.get("diameter_mm", ""))), x0 + 10, y0 + 20)
        g.dispose()
        return out

    def draw_overlay(self, image, feature):
        w = image.getWidth()
        h = image.getHeight()
        out = BufferedImage(w, h, BufferedImage.TYPE_INT_RGB)
        g = out.createGraphics()
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
        g.drawImage(image, 0, 0, None)
        if feature is not None and feature.get("x_px", None) is not None:
            x = int(round(float(feature.get("x_px"))))
            y = int(round(float(feature.get("y_px"))))
            r = None
            if feature.get("radius_px", None) is not None:
                r = int(round(float(feature.get("radius_px"))))
            elif feature.get("diameter_px", None) is not None:
                r = int(round(float(feature.get("diameter_px")) / 2.0))
            g.setColor(Color.GREEN)
            g.setStroke(BasicStroke(2.0))
            if r is not None and r > 0:
                g.drawOval(x - r, y - r, r * 2, r * 2)
            g.drawLine(x - 10, y, x + 10, y)
            g.drawLine(x, y - 10, x, y + 10)
            g.setColor(Color.YELLOW)
            g.drawString("X %.3f Y %.3f D %s" %
                (float(feature.get("x_mm", 0.0)), float(feature.get("y_mm", 0.0)),
                 str(feature.get("diameter_mm", ""))), 10, 20)
        g.dispose()
        return out

    def run_worker(self, name, work, done):
        if self.worker_running:
            return
        self.worker_running = True
        self.set_state(RUNNING)
        self.append_log("Starting %s" % name)
        gui_self = self

        class Worker(SwingWorker):
            def doInBackground(worker_self):
                return work()

            def done(worker_self):
                gui_self.worker_running = False
                try:
                    done(worker_self.get())
                except Exception, e:
                    gui_self.set_state(ERROR)
                    gui_self.show_error("%s failed" % name, e)
                gui_self.update_buttons()

        Worker().execute()


def start():
    gui_obj = NeedleCalibrationGui()
    gui_obj.build()
    return gui_obj


class StartRunnable(Runnable):
    def run(self):
        start()


try:
    SwingUtilities.invokeLater(StartRunnable())
except:
    start()
