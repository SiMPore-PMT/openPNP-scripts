# voron_calibration_offset_gui.py
#
# Simple GUI for Events/Homing/voron_calibration_offset.py.

import imp
import json
import os
import sys

from java.awt import BorderLayout, Color, Dimension, FlowLayout, Font, GridBagConstraints, GridBagLayout, Insets
from java.lang import Runnable
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
        backend = os.path.join(path, "Events", "Homing", "voron_calibration_offset.py")
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
BACKEND_PATH = os.path.join(HOMING_DIR, "voron_calibration_offset.py")
voron_backend = imp.load_source("voron_calibration_offset_backend", BACKEND_PATH)
CONFIG_PATH = os.path.join(SCRIPT_DIR, "voron_calibration_offset_config.json")

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
    return voron_backend.needle.java_list_to_py(model)


def object_name(obj):
    return voron_backend.needle.object_name(obj)


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
    for head in voron_backend.needle.all_heads(get_machine()):
        try:
            for nozzle in java_list_to_py(head.getNozzles()):
                names.append(object_name(nozzle))
        except:
            pass
    return unique_sorted(names)


def available_camera_names():
    machine_obj = get_machine()
    names = []
    for head in voron_backend.needle.all_heads(machine_obj):
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
    for head in voron_backend.needle.all_heads(machine_obj):
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
            return voron_backend.needle.loc_to_dict(value)
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


class VoronCalibrationOffsetGui(object):
    def __init__(self):
        self.frame = JFrame("Voron Tap Z Offset Calibration")
        self.cfg = voron_backend.load_config(CONFIG_PATH)
        self.saved_snapshot = compact_json(voron_backend.public_clean_config(self.cfg))
        self.loading_ui = False
        self.worker_running = False
        self.abort_requested = AtomicBoolean(False)
        self.abort_enabled = AtomicBoolean(False)
        self.pre_run_state = IDLE
        self.state = IDLE
        self.selector_combos = {}
        self.z_fields = {}
        self.pipeline_combos = {}
        self.contact_combos = {}
        self.contact_rows = {}
        self.probe_combos = {}
        self.probe_rows = {}
        self.motion_fields = {}
        self.action_buttons = []
        self.last_result = None

        self.status_area = JTextArea()
        self.current_area = JTextArea()
        self.result_area = JTextArea()
        self.log_area = JTextArea()
        self.final_value_label = JLabel("Voron offset: --")
        self.update_configs_btn = None
        self.state_label = JLabel("")
        self.dirty_label = JLabel("")

    def build(self):
        self.frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE)
        self.frame.setMinimumSize(Dimension(980, 700))
        root = JPanel(BorderLayout(8, 8))
        root.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8))
        root.add(self.build_top(), BorderLayout.NORTH)
        left = JScrollPane(self.build_form())
        left.setPreferredSize(Dimension(480, 580))
        right = self.build_right()
        split = JSplitPane(JSplitPane.HORIZONTAL_SPLIT, left, right)
        split.setContinuousLayout(True)
        split.setResizeWeight(0.45)
        split.setDividerLocation(480)
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
        specs = [
            ("refresh_lists_btn", "Refresh Lists", lambda e: self.refresh_selectors()),
            ("test_pipeline_btn", "Test Pipeline", lambda e: self.test_pipeline_action()),
            ("measure_btn", "Measure Voron Offset", lambda e: self.measure_action()),
            ("abort_btn", "Abort", lambda e: self.abort_action()),
            ("save_json_btn", "Save JSON Config", lambda e: self.save_config_action())
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
        panel.add(self.build_motion_section())
        panel.add(Box.createVerticalGlue())
        return panel

    def build_selectors(self):
        panel = self.section("OpenPnP Objects")
        specs = [
            ("nozzle_name", "Nozzle"),
            ("top_camera_name", "Top Camera"),
            ("contact_actuator_name", "Contact Read / Driver Actuator"),
            ("voron_actuator_name", "Voron Read / Driver Actuator")
        ]
        for row, spec in enumerate(specs):
            key, label = spec
            combo = JComboBox()
            combo.setEditable(True)
            combo.setPreferredSize(Dimension(280, combo.getPreferredSize().height))
            combo.addActionListener(lambda event: self.on_field_changed())
            self.selector_combos[key] = combo
            self.add_row(panel, row, label, combo)
        return panel

    def build_z_section(self):
        panel = self.section("Z Touch Pad")
        specs = [("x", "X mm"), ("y", "Y mm"), ("z", "Surface Z mm"),
            ("rotation", "Rotation"), ("result_stage", "Result Stage")]
        for row, spec in enumerate(specs):
            key, label = spec
            field = JTextField(12)
            self.z_fields[key] = field
            self.watch_field(field)
            self.add_row(panel, row, label, field)

        mode = JComboBox()
        mode.addItem("named")
        mode.addItem("xml_path")
        mode.addActionListener(lambda event: self.on_field_changed())
        self.pipeline_combos["pipeline_mode"] = mode
        self.add_row(panel, len(specs), "Pipeline Mode", mode)

        source = JComboBox()
        source.setEditable(True)
        source.setPreferredSize(Dimension(280, source.getPreferredSize().height))
        source.addActionListener(lambda event: self.on_field_changed())
        self.pipeline_combos["pipeline_source"] = source
        self.add_row(panel, len(specs) + 1, "Pipeline Name / Path", source)

        buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 0))
        capture = JButton("Capture Camera DRO")
        go = JButton("Go Camera")
        capture.addActionListener(lambda event: self.capture_camera_location())
        go.addActionListener(lambda event: self.go_camera_location())
        self.action_buttons.append(capture)
        self.action_buttons.append(go)
        buttons.add(capture)
        buttons.add(go)
        panel.add(buttons, self.full_width_gbc(len(specs) + 2))
        return panel

    def build_motion_section(self):
        panel = self.section("Motion / Probe")
        contact_acquisition = JComboBox()
        contact_acquisition.addItem("polling")
        contact_acquisition.addItem("hardware_interrupt")
        contact_acquisition.addActionListener(lambda event: self.on_mode_changed())
        self.contact_combos["acquisition_method"] = contact_acquisition
        self.contact_rows["acquisition_method"] = self.add_row(panel, 0,
            "Contact Touch Method", contact_acquisition)
        contact_mode = JComboBox()
        contact_mode.addItem("actuator")
        contact_mode.addItem("direct_gcode")
        contact_mode.addActionListener(lambda event: self.on_mode_changed())
        self.contact_combos["mode"] = contact_mode
        self.contact_rows["mode"] = self.add_row(panel, 1,
            "Contact Hardware Mode", contact_mode)
        contact_sequence = JComboBox()
        contact_sequence.setEditable(True)
        contact_sequence.setPreferredSize(Dimension(280,
            contact_sequence.getPreferredSize().height))
        contact_sequence.addActionListener(lambda event: self.on_field_changed())
        self.contact_combos["sequence_actuator_name"] = contact_sequence
        self.contact_rows["sequence_actuator_name"] = self.add_row(panel, 2,
            "Contact Sequence Actuator", contact_sequence)
        contact_channel = JComboBox()
        contact_channel.addItem("Q0 - Voron")
        contact_channel.addItem("Q1 - Contact")
        contact_channel.addActionListener(lambda event: self.on_field_changed())
        self.contact_combos["channel"] = contact_channel
        self.contact_rows["channel"] = self.add_row(panel, 3,
            "Contact Mux Channel", contact_channel)
        acquisition = JComboBox()
        acquisition.addItem("polling")
        acquisition.addItem("hardware_interrupt")
        acquisition.addActionListener(lambda event: self.on_mode_changed())
        self.probe_combos["acquisition_method"] = acquisition
        self.probe_rows["acquisition_method"] = self.add_row(
            panel, 4, "Voron Touch Method", acquisition)
        probe_mode = JComboBox()
        probe_mode.addItem("actuator")
        probe_mode.addItem("direct_gcode")
        probe_mode.addActionListener(lambda event: self.on_mode_changed())
        self.probe_combos["mode"] = probe_mode
        self.probe_rows["mode"] = self.add_row(panel, 5,
            "Voron Hardware Mode", probe_mode)
        sequence_actuator = JComboBox()
        sequence_actuator.setEditable(True)
        sequence_actuator.setPreferredSize(Dimension(280,
            sequence_actuator.getPreferredSize().height))
        sequence_actuator.addActionListener(lambda event: self.on_field_changed())
        self.probe_combos["sequence_actuator_name"] = sequence_actuator
        self.probe_rows["sequence_actuator_name"] = self.add_row(panel, 6,
            "Voron Sequence Actuator", sequence_actuator)
        channel = JComboBox()
        channel.addItem("Q0 - Voron")
        channel.addItem("Q1 - Contact")
        channel.addActionListener(lambda event: self.on_field_changed())
        self.probe_combos["channel"] = channel
        self.probe_rows["channel"] = self.add_row(panel, 7,
            "Voron Mux Channel", channel)
        specs = [
            ("safe_z_mm", "Safe Z mm"),
            ("travel_speed", "Travel Speed"),
            ("contact_probe.start_above_mm", "Contact Start Above mm"),
            ("contact_probe.max_descent_mm", "Contact Max Descent mm"),
            ("contact_probe.coarse_speed", "Contact Coarse Speed"),
            ("contact_probe.fine_speed", "Contact Fine Speed"),
            ("contact_probe.coarse_step_mm", "Contact Coarse Step mm"),
            ("contact_probe.fine_step_mm", "Contact Fine Step mm"),
            ("contact_probe.retract_after_contact_mm", "Contact Retract mm"),
            ("contact_probe.release_retract_step_mm", "Contact Release Step mm"),
            ("contact_probe.max_release_retract_mm", "Contact Max Release mm"),
            ("contact_probe.read_settle_ms", "Contact Read Settle ms"),
            ("contact_probe.debounce_count", "Contact Debounce Count"),
            ("contact_probe.debounce_retries", "Contact Debounce Retries"),
            ("contact_probe.debounce_interval_ms", "Contact Debounce Interval ms"),
            ("contact_probe.coarse_repeats", "Contact Coarse Tries"),
            ("contact_probe.fine_repeats", "Contact Fine Tries"),
            ("contact_probe.coarse_feed", "Direct Contact Coarse F"),
            ("contact_probe.fine_feed", "Direct Contact Fine F"),
            ("contact_probe.direct_fine_repeats", "Direct Contact Fine Repeats"),
            ("contact_probe.g38_attempts", "Contact G38 Attempts"),
            ("contact_probe.probe_acceleration", "Contact Direct M204 S"),
            ("z_probe.polling_coarse_speed", "Voron Polling Coarse Speed"),
            ("z_probe.polling_fine_speed", "Voron Polling Fine Speed"),
            ("z_probe.coarse_feed", "Direct Voron Coarse F"),
            ("z_probe.fine_feed", "Direct Voron Fine F"),
            ("z_probe.fine_repeats", "Direct Voron Fine Repeats"),
            ("z_probe.g38_attempts", "Voron G38 Attempts"),
            ("z_probe.probe_acceleration", "Direct M204 S"),
            ("z_probe.start_above_mm", "Z Start Above mm"),
            ("z_probe.max_descent_mm", "Z Max Descent mm"),
            ("z_probe.coarse_step_mm", "Z Coarse Step mm"),
            ("z_probe.fine_step_mm", "Z Fine Step mm"),
            ("z_probe.retract_after_contact_mm", "Z Retract After Contact mm"),
            ("z_probe.release_retract_step_mm", "Z Release Retract Step mm"),
            ("z_probe.max_release_retract_mm", "Z Max Release Retract mm"),
            ("z_probe.read_settle_ms", "Read Settle ms"),
            ("z_probe.debounce_count", "Debounce Count"),
            ("z_probe.debounce_retries", "Debounce Retries"),
            ("z_probe.debounce_interval_ms", "Debounce Interval ms"),
            ("z_probe.polling_coarse_repeats", "Voron Polling Coarse Tries"),
            ("z_probe.polling_fine_repeats", "Voron Polling Fine Tries")
        ]
        for row, spec in enumerate(specs):
            key, label = spec
            field = JTextField(10)
            self.motion_fields[key] = field
            self.watch_field(field)
            self.probe_rows[key] = self.add_row(panel, row + 8, label, field)
        return panel

    def build_right(self):
        panel = JPanel(BorderLayout(6, 6))
        self.final_value_label.setFont(Font("Dialog", Font.BOLD, 34))
        self.final_value_label.setPreferredSize(Dimension(420, 62))
        final_panel = JPanel(BorderLayout(4, 2))
        final_panel.setBorder(BorderFactory.createTitledBorder("Final Calculated Value"))
        final_panel.add(self.final_value_label, BorderLayout.CENTER)
        update_row = JPanel(FlowLayout(FlowLayout.RIGHT, 4, 0))
        self.update_configs_btn = JButton("Update")
        self.update_configs_btn.setToolTipText(
            "Write this calculated offset to Needle Calibration 2 and Bed Mesh configs")
        self.update_configs_btn.addActionListener(
            lambda event: self.update_target_configs_action())
        self.update_configs_btn.setEnabled(False)
        self.action_buttons.append(self.update_configs_btn)
        update_row.add(self.update_configs_btn)
        final_panel.add(update_row, BorderLayout.SOUTH)
        current = JPanel(GridBagLayout())
        current.setBorder(BorderFactory.createTitledBorder("Current"))
        self.configure_text_area(self.current_area, 3)
        self.add_row(current, 0, "Nozzle Offsets", self.current_area)
        self.add_row(current, 1, "State", self.state_label)
        self.add_row(current, 2, "JSON Dirty", self.dirty_label)
        top = JPanel(BorderLayout(6, 6))
        top.add(current, BorderLayout.CENTER)
        top.add(final_panel, BorderLayout.SOUTH)
        panel.add(top, BorderLayout.NORTH)
        self.result_area.setEditable(False)
        self.result_area.setLineWrap(False)
        scroll = JScrollPane(self.result_area)
        scroll.setBorder(BorderFactory.createTitledBorder("Voron Offset Result / Full JSON"))
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
        label_obj.setPreferredSize(Dimension(190, label_obj.getPreferredSize().height))
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
        if self.update_configs_btn is not None:
            result = self.last_result
            self.update_configs_btn.setEnabled(
                (not self.worker_running) and result is not None and
                bool(result.get("ok", False)) and
                result.get("voron_offset_mm", None) is not None)

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
        current = voron_backend.needle.loc_to_dict(
            nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
        changed = False
        for axis in ["x", "y", "z", "rotation"]:
            if abs(float(current.get(axis, 0.0)) - float(before.get(axis, 0.0))) > 0.0000001:
                changed = True
                break
        if changed:
            voron_backend.needle.set_offsets_from_dict(nozzle, before)
        self.append_log("Abort complete; original head offsets restored")
        return changed

    def set_row_visible(self, row, visible):
        if row is None:
            return
        for component in row:
            component.setVisible(bool(visible))

    def polling_probe_keys(self):
        return ["z_probe.polling_coarse_speed", "z_probe.polling_fine_speed",
            "z_probe.max_descent_mm", "z_probe.coarse_step_mm",
            "z_probe.fine_step_mm", "z_probe.retract_after_contact_mm",
            "z_probe.release_retract_step_mm", "z_probe.max_release_retract_mm",
            "z_probe.read_settle_ms", "z_probe.debounce_count",
            "z_probe.debounce_retries", "z_probe.debounce_interval_ms",
            "z_probe.polling_coarse_repeats", "z_probe.polling_fine_repeats"]

    def contact_polling_keys(self):
        return ["contact_probe.max_descent_mm", "contact_probe.coarse_speed",
            "contact_probe.fine_speed", "contact_probe.coarse_step_mm",
            "contact_probe.fine_step_mm", "contact_probe.retract_after_contact_mm",
            "contact_probe.release_retract_step_mm",
            "contact_probe.max_release_retract_mm", "contact_probe.read_settle_ms",
            "contact_probe.debounce_count", "contact_probe.debounce_retries",
            "contact_probe.debounce_interval_ms", "contact_probe.coarse_repeats",
            "contact_probe.fine_repeats"]

    def contact_direct_keys(self):
        return ["contact_probe.coarse_feed", "contact_probe.fine_feed",
            "contact_probe.direct_fine_repeats",
            "contact_probe.g38_attempts",
            "contact_probe.probe_acceleration", "contact_probe.max_descent_mm",
            "contact_probe.retract_after_contact_mm"]

    def direct_probe_keys(self):
        return ["z_probe.coarse_feed", "z_probe.fine_feed",
            "z_probe.fine_repeats", "z_probe.g38_attempts",
            "z_probe.probe_acceleration",
            "z_probe.max_descent_mm", "z_probe.retract_after_contact_mm"]

    def update_mode_visibility(self):
        contact_acquisition = str(
            self.contact_combos["acquisition_method"].getSelectedItem() or "polling")
        contact_mode = str(self.contact_combos["mode"].getSelectedItem()
            or "direct_gcode")
        contact_hardware = contact_acquisition == "hardware_interrupt"
        contact_direct = contact_hardware and contact_mode == "direct_gcode"
        self.set_row_visible(self.contact_rows.get("mode"), contact_hardware)
        self.set_row_visible(self.contact_rows.get("sequence_actuator_name"),
            contact_hardware and contact_mode == "actuator")
        # Keep each sensor's independent mux selection visible in every mode so
        # it can be configured before switching to direct hardware probing.
        self.set_row_visible(self.contact_rows.get("channel"), True)
        for key in self.contact_polling_keys():
            self.set_row_visible(self.probe_rows.get(key), not contact_hardware)
        for key in self.contact_direct_keys():
            self.set_row_visible(self.probe_rows.get(key), contact_direct)
        for key in ["contact_probe.max_descent_mm",
                "contact_probe.retract_after_contact_mm"]:
            self.set_row_visible(self.probe_rows.get(key),
                (not contact_hardware) or contact_direct)
        self.set_row_visible(self.probe_rows.get("contact_probe.start_above_mm"), True)
        acquisition = str(self.probe_combos["acquisition_method"].getSelectedItem()
            or "hardware_interrupt")
        mode = str(self.probe_combos["mode"].getSelectedItem() or "actuator")
        hardware = acquisition == "hardware_interrupt"
        direct = hardware and mode == "direct_gcode"
        actuator = hardware and mode == "actuator"
        self.set_row_visible(self.probe_rows.get("mode"), hardware)
        self.set_row_visible(self.probe_rows.get("sequence_actuator_name"), actuator)
        self.set_row_visible(self.probe_rows.get("channel"), True)
        for key in self.polling_probe_keys():
            self.set_row_visible(self.probe_rows.get(key), not hardware)
        for key in self.direct_probe_keys():
            self.set_row_visible(self.probe_rows.get(key), direct)
        for key in ["z_probe.max_descent_mm", "z_probe.retract_after_contact_mm"]:
            self.set_row_visible(self.probe_rows.get(key), (not hardware) or direct)
        # Start height is the only Voron probe distance used by all three paths.
        self.set_row_visible(self.probe_rows.get("z_probe.start_above_mm"), True)
        try:
            self.frame.getContentPane().revalidate()
            self.frame.getContentPane().repaint()
        except:
            pass

    def on_mode_changed(self):
        self.update_mode_visibility()
        self.on_field_changed()

    def on_field_changed(self):
        if self.loading_ui:
            return
        self.update_dirty()

    def update_dirty(self):
        try:
            dirty = compact_json(voron_backend.public_clean_config(self.collect_config(False))) != self.saved_snapshot
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
            set_combo_items(self.selector_combos["contact_actuator_name"], actuator_names,
                combo_text(self.selector_combos["contact_actuator_name"]))
            set_combo_items(self.selector_combos["voron_actuator_name"], actuator_names,
                combo_text(self.selector_combos["voron_actuator_name"]))
            set_combo_items(self.probe_combos["sequence_actuator_name"], actuator_names,
                combo_text(self.probe_combos["sequence_actuator_name"]))
            set_combo_items(self.contact_combos["sequence_actuator_name"], actuator_names,
                combo_text(self.contact_combos["sequence_actuator_name"]))
            names = voron_backend.list_available_vision_pipeline_names(get_machine())
            set_combo_items(self.pipeline_combos["pipeline_source"], names,
                combo_text(self.pipeline_combos["pipeline_source"]))
            self.append_log("Refreshed object and pipeline lists")
        except Exception as e:
            self.show_error("Refresh Lists failed", e)

    def load_into_ui(self, cfg):
        self.loading_ui = True
        try:
            for key in self.selector_combos:
                self.selector_combos[key].setSelectedItem(str(cfg.get(key, "")))
            z = cfg.get("z_touch", {})
            loc = z.get("location", {})
            for axis in ["x", "y", "z", "rotation"]:
                self.z_fields[axis].setText(str(loc.get(axis, 0.0)))
            self.z_fields["result_stage"].setText(str(z.get("result_stage", "results")))
            self.load_pipeline(z.get("pipeline", {}))
            probe = cfg.get("z_probe", {})
            contact = cfg.get("contact_probe", {})
            self.contact_combos["acquisition_method"].setSelectedItem(str(
                contact.get("acquisition_method", "polling")))
            self.contact_combos["mode"].setSelectedItem(str(
                contact.get("mode", "direct_gcode")))
            self.contact_combos["sequence_actuator_name"].setSelectedItem(str(
                contact.get("sequence_actuator_name", "Run Contact Probe")))
            contact_channel = str(contact.get("channel", "Q1") or "Q1").upper()
            self.contact_combos["channel"].setSelectedItem(
                "Q1 - Contact" if contact_channel == "Q1" else "Q0 - Voron")
            self.probe_combos["acquisition_method"].setSelectedItem(str(
                probe.get("acquisition_method", "hardware_interrupt")))
            self.probe_combos["mode"].setSelectedItem(str(
                probe.get("mode", "actuator")))
            self.probe_combos["sequence_actuator_name"].setSelectedItem(str(
                probe.get("sequence_actuator_name", "Run Voron Probe")))
            channel = str(probe.get("channel", "Q0") or "Q0").upper()
            self.probe_combos["channel"].setSelectedItem(
                "Q1 - Contact" if channel == "Q1" else "Q0 - Voron")
            for key in self.motion_fields:
                value = self.get_nested_value(cfg, key)
                self.motion_fields[key].setText("" if value is None else str(value))
        finally:
            self.loading_ui = False
        self.update_mode_visibility()
        self.update_dirty()

    def load_pipeline(self, pipe):
        mode = str(pipe.get("mode", "named") or "named")
        if mode not in ["named", "xml_path"]:
            mode = "named"
        self.pipeline_combos["pipeline_mode"].setSelectedItem(mode)
        self.pipeline_combos["pipeline_source"].setSelectedItem(str(
            pipe.get("name", "") if mode == "named" else pipe.get("path", "")))

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
        cfg = voron_backend.deep_merge(voron_backend.DEFAULT_CONFIG, self.cfg)
        for key in self.selector_combos:
            cfg[key] = combo_text(self.selector_combos[key])
        cfg["z_touch"] = {
            "location": self.collect_location(),
            "result_stage": self.z_fields["result_stage"].getText().strip(),
            "pipeline": self.collect_pipeline()
        }
        contact_acquisition = str(
            self.contact_combos["acquisition_method"].getSelectedItem()).strip()
        contact_mode = str(self.contact_combos["mode"].getSelectedItem()).strip()
        cfg["contact_probe"]["acquisition_method"] = contact_acquisition
        cfg["contact_probe"]["mode"] = contact_mode
        cfg["contact_probe"]["sequence_actuator_name"] = combo_text(
            self.contact_combos["sequence_actuator_name"])
        contact_channel_text = str(
            self.contact_combos["channel"].getSelectedItem() or "")
        cfg["contact_probe"]["channel"] = (
            "Q1" if contact_channel_text.upper().startswith("Q1") else "Q0")
        acquisition = str(
            self.probe_combos["acquisition_method"].getSelectedItem()).strip()
        cfg["z_probe"]["acquisition_method"] = acquisition
        mode = str(self.probe_combos["mode"].getSelectedItem()).strip()
        cfg["z_probe"]["mode"] = mode
        cfg["z_probe"]["sequence_actuator_name"] = combo_text(
            self.probe_combos["sequence_actuator_name"])
        channel_text = str(self.probe_combos["channel"].getSelectedItem() or "")
        cfg["z_probe"]["channel"] = "Q1" if channel_text.upper().startswith("Q1") else "Q0"
        active_keys = ["safe_z_mm", "travel_speed",
            "contact_probe.start_above_mm"]
        if contact_acquisition == "polling":
            active_keys += self.contact_polling_keys()
        elif contact_mode == "direct_gcode":
            active_keys += self.contact_direct_keys()
        active_keys.append("z_probe.start_above_mm")
        if acquisition == "polling":
            active_keys += self.polling_probe_keys()
        elif mode == "direct_gcode":
            active_keys += self.direct_probe_keys()
        seen = {}
        for key in active_keys:
            if key in seen:
                continue
            seen[key] = True
            self.set_nested_value(cfg, key, self.parse_float(self.motion_fields[key].getText(), key))
        if validate:
            self.validate_config(cfg)
        return cfg

    def collect_location(self):
        return {"x": self.parse_float(self.z_fields["x"].getText(), "z_touch.x"),
            "y": self.parse_float(self.z_fields["y"].getText(), "z_touch.y"),
            "z": self.parse_float(self.z_fields["z"].getText(), "z_touch.z"),
            "rotation": self.parse_float(self.z_fields["rotation"].getText(), "z_touch.rotation")}

    def collect_pipeline(self):
        mode = str(self.pipeline_combos["pipeline_mode"].getSelectedItem()).strip()
        source = combo_text(self.pipeline_combos["pipeline_source"])
        if mode == "xml_path":
            return {"mode": "xml_path", "path": source, "name": ""}
        return {"mode": "named", "name": source, "path": ""}

    def parse_float(self, text, label):
        try:
            return float(str(text).strip())
        except:
            raise Exception("%s must be a number" % label)

    def validate_config(self, cfg):
        for key, label in [("nozzle_name", "Nozzle"), ("top_camera_name", "Top camera"),
                ("contact_actuator_name", "Contact actuator"),
                ("voron_actuator_name", "Voron actuator")]:
            if str(cfg.get(key, "")).strip() == "":
                raise Exception("%s is required" % label)
        pipe = cfg.get("z_touch", {}).get("pipeline", {})
        mode = str(pipe.get("mode", "named") or "named")
        if mode == "named" and str(pipe.get("name", "")).strip() == "":
            raise Exception("Z touch pipeline name is required")
        if mode == "xml_path" and str(pipe.get("path", "")).strip() == "":
            raise Exception("Z touch XML pipeline path is required")
        probe = cfg.get("z_probe", {})
        acquisition = str(probe.get("acquisition_method", "hardware_interrupt"))
        if acquisition not in ["polling", "hardware_interrupt"]:
            raise Exception("Voron touch method must be polling or hardware_interrupt")
        if float(probe.get("start_above_mm", 0.0)) <= 0.0:
            raise Exception("z_probe.start_above_mm must be > 0")
        contact = cfg.get("contact_probe", {})
        contact_acquisition = str(contact.get("acquisition_method", "polling"))
        if contact_acquisition not in ["polling", "hardware_interrupt"]:
            raise Exception("Contact touch method must be polling or hardware_interrupt")
        if float(contact.get("start_above_mm", 0.0)) <= 0.0:
            raise Exception("contact_probe.start_above_mm must be > 0")
        if contact_acquisition == "polling":
            for key in ["coarse_repeats", "fine_repeats"]:
                self.validate_repeat(contact.get(key, 1),
                    "contact_probe.%s" % key)
            for key in ["max_descent_mm", "coarse_speed", "fine_speed",
                    "coarse_step_mm", "fine_step_mm", "retract_after_contact_mm",
                    "release_retract_step_mm", "max_release_retract_mm"]:
                if float(contact.get(key, 0.0)) <= 0.0:
                    raise Exception("contact_probe.%s must be > 0" % key)
        else:
            contact_mode = str(contact.get("mode", "direct_gcode"))
            if contact_mode not in ["actuator", "direct_gcode"]:
                raise Exception("Contact hardware mode must be actuator or direct_gcode")
            if contact_mode == "actuator":
                if not str(contact.get("sequence_actuator_name", "")).strip():
                    raise Exception("Contact sequence actuator is required in actuator mode")
            else:
                self.validate_repeat(contact.get("direct_fine_repeats", 1),
                    "contact_probe.direct_fine_repeats")
                self.validate_repeat(contact.get("g38_attempts", 2),
                    "contact_probe.g38_attempts")
                for key in ["coarse_feed", "fine_feed", "probe_acceleration",
                        "max_descent_mm", "retract_after_contact_mm"]:
                    if float(contact.get(key, 0.0)) <= 0.0:
                        raise Exception("contact_probe.%s must be > 0" % key)
                if str(contact.get("channel", "")) not in ["Q0", "Q1"]:
                    raise Exception("contact_probe.channel must be Q0 or Q1")
        if acquisition == "polling":
            for key in ["polling_coarse_repeats", "polling_fine_repeats"]:
                self.validate_repeat(probe.get(key, 1), "z_probe.%s" % key)
            for key in ["polling_coarse_speed", "polling_fine_speed",
                    "max_descent_mm", "coarse_step_mm", "fine_step_mm",
                    "retract_after_contact_mm", "release_retract_step_mm",
                    "max_release_retract_mm"]:
                if float(probe.get(key, 0.0)) <= 0.0:
                    raise Exception("z_probe.%s must be > 0" % key)
        else:
            hardware_mode = str(probe.get("mode", "actuator"))
            if hardware_mode not in ["actuator", "direct_gcode"]:
                raise Exception("Hardware probe mode must be actuator or direct_gcode")
            if hardware_mode == "actuator":
                if not str(probe.get("sequence_actuator_name", "")).strip():
                    raise Exception("Sequence actuator is required in actuator mode")
            else:
                self.validate_repeat(probe.get("fine_repeats", 1),
                    "z_probe.fine_repeats")
                self.validate_repeat(probe.get("g38_attempts", 2),
                    "z_probe.g38_attempts")
                for key in ["coarse_feed", "fine_feed", "probe_acceleration",
                        "max_descent_mm", "retract_after_contact_mm"]:
                    if float(probe.get(key, 0.0)) <= 0.0:
                        raise Exception("z_probe.%s must be > 0" % key)
                if str(probe.get("channel", "")) not in ["Q0", "Q1"]:
                    raise Exception("z_probe.channel must be Q0 or Q1")

    def validate_repeat(self, value, label):
        try:
            number = float(value)
            count = int(number)
        except:
            raise Exception("%s must be a whole number >= 1" % label)
        if number != float(count) or count < 1:
            raise Exception("%s must be a whole number >= 1" % label)

    def refresh_current_values(self):
        try:
            cfg = self.collect_config(False)
            nozzle = voron_backend.needle.find_nozzle(voron_backend.build_probe_cfg(cfg), None, get_machine())
            offsets = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
            self.current_area.setText("Nozzle: %s\nX %.4f  Y %.4f  Z %.4f  R %.4f" %
                (object_name(nozzle), offsets.getX(), offsets.getY(),
                 offsets.getZ(), offsets.getRotation()))
            self.status_area.setText("Ready")
        except Exception as e:
            self.current_area.setText("Unavailable: %s" % e)

    def set_location_fields(self, loc):
        loc = loc.convertToUnits(LengthUnit.Millimeters)
        self.z_fields["x"].setText("%.4f" % loc.getX())
        self.z_fields["y"].setText("%.4f" % loc.getY())
        self.z_fields["z"].setText("%.4f" % loc.getZ())
        self.z_fields["rotation"].setText("%.4f" % loc.getRotation())

    def capture_camera_location(self):
        try:
            cfg = self.collect_config(False)
            camera = voron_backend.needle.find_top_camera(voron_backend.build_probe_cfg(cfg), get_machine())
            loc = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
            self.set_location_fields(loc)
            self.append_log("Captured z_touch from camera DRO")
            self.update_dirty()
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Capture Camera DRO failed", e)

    def go_camera_location(self):
        try:
            cfg = self.collect_config(False)
            loc = cfg["z_touch"]["location"]
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Go Camera setup failed", e)
            return
        self.run_worker("Go Camera z_touch",
            lambda: self.do_go_camera_location(cfg, loc), self.go_camera_done)

    def do_go_camera_location(self, cfg, loc):
        machine_obj = require_machine_execute()
        gui_self = self
        class Task(Callable):
            def call(task_self):
                try:
                    camera = voron_backend.needle.find_top_camera(voron_backend.build_probe_cfg(cfg), machine_obj)
                    voron_backend.needle.move_safe(camera, mm_location_from_dict(loc))
                    return {"ok": True, "target": "camera", "location": loc}
                finally:
                    pass
        return machine_obj.execute(Task())

    def go_camera_done(self, result):
        self.last_result = result
        self.result_area.setText(self.format_result(result))
        self.update_final_value_label(result)
        self.refresh_current_values()
        self.set_state(READY)
        self.append_log("Camera move finished")

    def test_pipeline_action(self):
        try:
            cfg = self.collect_config(True)
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Pipeline setup failed", e)
            return
        self.run_worker("Test Pipeline",
            lambda: self.do_test_pipeline(cfg), self.test_pipeline_done)

    def do_test_pipeline(self, cfg):
        machine_obj = require_machine_execute()
        gui_self = self
        class Task(Callable):
            def call(task_self):
                try:
                    return voron_backend.capture_touch_center(cfg, None, machine_obj,
                        gui_self.append_log)
                finally:
                    pass
        return machine_obj.execute(Task())

    def test_pipeline_done(self, result):
        self.last_result = result
        self.result_area.setText(self.format_result(result))
        self.update_final_value_label(result)
        self.refresh_current_values()
        self.set_state(READY)

    def measure_action(self):
        try:
            cfg = self.collect_config(True)
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Measure setup failed", e)
            return
        self.run_worker("Measure Voron Offset",
            lambda: self.do_measure(cfg), self.measure_done, abortable=True)

    def do_measure(self, cfg):
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
                    probe_cfg = voron_backend.build_probe_cfg(cfg)
                    nozzle = voron_backend.needle.find_nozzle(probe_cfg, None, machine_obj)
                    before = voron_backend.needle.loc_to_dict(
                        nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters))
                    gui_self.calibration_log("Abort checkpoint active")
                    result = voron_backend.run_measure(
                        cfg, nozzle, machine_obj, gui_self.calibration_log)
                except CalibrationAborted:
                    aborted = True
                finally:
                    gui_self.abort_enabled.set(False)
                    if nozzle is not None and (aborted or gui_self.abort_requested.get()):
                        aborted = True
                        cleanup = voron_backend.needle.abort_nozzle_cleanup(
                            nozzle, probe_cfg, gui_self.append_log)
                        offsets_changed = gui_self.restore_offsets_after_abort(nozzle, before)
                if aborted:
                    return {
                        "ok": False,
                        "aborted": True,
                        "stage": "voron_offset",
                        "message": "Calibration aborted by operator; cleanup complete",
                        "abort_cleanup": cleanup,
                        "offsets_restored": True,
                        "offsets_changed_before_restore": offsets_changed,
                    }
                return result
        return machine_obj.execute(Task())

    def measure_done(self, result):
        self.last_result = result
        self.result_area.setText(self.format_result(result))
        self.update_final_value_label(result)
        self.refresh_current_values()
        if result is not None and result.get("aborted", False):
            cleanup = result.get("abort_cleanup") or {}
            if cleanup.get("ok", False):
                self.set_state(self.pre_run_state)
            else:
                self.set_state(ERROR)
                self.show_error("Abort cleanup incomplete",
                    "; ".join(cleanup.get("errors") or ["Unknown abort cleanup error"]))
            self.append_log(result.get("message"))
        else:
            self.set_state(READY if result is not None and result.get("ok", False) else ERROR)

    def update_final_value_label(self, result):
        if result is not None and result.get("voron_offset_mm", None) is not None:
            self.final_value_label.setText("Voron offset: %.6f mm" %
                float(result.get("voron_offset_mm")))
        else:
            self.final_value_label.setText("Voron offset: --")
        self.update_buttons()

    def update_target_configs_action(self):
        result = self.last_result
        if result is None or not result.get("ok", False) or result.get(
                "voron_offset_mm", None) is None:
            self.show_error("Update Configs failed",
                "Run a successful Voron offset measurement first")
            return
        try:
            value = float(result.get("voron_offset_mm"))
            updates = voron_backend.update_voron_offset_configs(value, SCRIPT_DIR)
            self.append_log("Applied calculated Voron Z offset: %.6f mm" % value)
            for update in updates:
                old_value = update.get("old_value")
                old_text = "<missing>" if old_value is None else str(old_value)
                self.append_log("Updated %s: %s -> %.6f in %s" %
                    (update.get("field"), old_text,
                     float(update.get("new_value")), update.get("path")))
        except Exception as error:
            self.show_error("Update Configs failed", error)

    def save_config_action(self):
        try:
            cfg = self.collect_config(False)
            voron_backend.save_config(cfg, CONFIG_PATH)
            self.cfg = cfg
            self.saved_snapshot = compact_json(voron_backend.public_clean_config(cfg))
            self.refresh_current_values()
            self.set_state(SAVED)
            self.append_log("Saved JSON config: %s" % CONFIG_PATH)
        except Exception as e:
            self.set_state(ERROR)
            self.show_error("Save JSON failed", e)

    def format_result(self, result):
        if result is None:
            return "No result"
        lines = []
        lines.append("ok: %s" % result.get("ok"))
        lines.append("stage: %s" % result.get("stage", ""))
        lines.append("message: %s" % result.get("message", ""))
        if result.get("voron_offset_mm", None) is not None:
            lines.append("")
            lines.append("Voron offset mm: %.6f" % float(result.get("voron_offset_mm")))
            lines.append("Contact reference Z mm: %.6f" %
                float(result.get("contact_reference_z_mm")))
            lines.append("Voron contact Z mm: %.6f" %
                float(result.get("voron_contact_z_mm")))
        details = result.get("details", {})
        center = details.get("center", None)
        if center is not None:
            lines.append("Center XY: %.6f / %.6f" %
                (float(center.get("x", 0.0)), float(center.get("y", 0.0))))
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
    gui_obj = VoronCalibrationOffsetGui()
    gui_obj.build()
    return gui_obj


class StartRunnable(Runnable):
    def run(self):
        start()


try:
    SwingUtilities.invokeLater(StartRunnable())
except:
    start()
