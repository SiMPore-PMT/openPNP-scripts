# bed_mesh_gui.py
#
# Swing GUI for the standalone OpenPnP Voron Tap bed mesh backend.

from __future__ import print_function

import imp
import json
import os

from java.awt import (
    BorderLayout, Color, Dimension, FlowLayout, Font, GridBagConstraints,
    GridBagLayout, Insets
)
from java.lang import Runnable, Throwable
from java.util.concurrent import Callable
from java.util.concurrent.atomic import AtomicBoolean
from javax.swing import (
    BorderFactory, Box, BoxLayout, JButton, JCheckBox, JComboBox, JFileChooser,
    JFrame, JLabel, JOptionPane, JPanel, JProgressBar, JScrollPane, JSplitPane,
    JTabbedPane, JTable, JTextArea, JTextField, SwingConstants, SwingUtilities,
    SwingWorker
)
from javax.swing.event import DocumentListener
from javax.swing.table import DefaultTableModel
from org.openpnp.model import Configuration, LengthUnit


def existing_dir(path):
    try:
        if path and os.path.isdir(str(path)):
            return os.path.abspath(str(path))
    except:
        pass
    return None


def candidate_script_dirs():
    candidates = []
    try:
        candidates.append(scripting.getScriptsDirectory().toString())
    except:
        pass
    try:
        candidates.append(os.path.dirname(os.path.abspath(__file__)))
    except:
        pass
    try:
        candidates.append(os.path.join(os.path.expanduser("~"), ".openpnp2", "scripts"))
    except:
        pass
    candidates.append(os.getcwd())
    seen = {}
    result = []
    for candidate in candidates:
        path = existing_dir(candidate)
        if path is not None and path not in seen:
            seen[path] = True
            result.append(path)
    return result


def resolve_scripts_root():
    attempted = []
    for path in candidate_script_dirs():
        # When this GUI lives beside the backend under Events/Homing, recover
        # the scripts root directly from that location.
        local_backend = os.path.join(path, "bed_mesh.py")
        attempted.append(local_backend)
        if (os.path.isfile(local_backend) and
                os.path.basename(path) == "Homing" and
                os.path.basename(os.path.dirname(path)) == "Events"):
            return os.path.dirname(os.path.dirname(path))
        backend = os.path.join(path, "Events", "Homing", "bed_mesh.py")
        attempted.append(backend)
        if os.path.isfile(backend):
            return path
    raise IOError("Bed mesh backend not found. Checked: %s" % "; ".join(attempted))


SCRIPT_DIR = resolve_scripts_root()
CONFIG_PATH = os.path.join(SCRIPT_DIR, "bed_mesh_config.json")
HOMING_DIR = os.path.join(SCRIPT_DIR, "Events", "Homing")
BACKEND_PATH = os.path.join(HOMING_DIR, "bed_mesh.py")
SURFACE_PATH = os.path.join(HOMING_DIR, "bed_mesh_surface.py")
bed_mesh = imp.load_source("bed_mesh_backend", BACKEND_PATH)
bed_mesh_surface = imp.load_source("bed_mesh_surface", SURFACE_PATH)

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


def java_items(model):
    out = []
    if model is None:
        return out
    try:
        for item in model:
            out.append(item)
        return out
    except:
        pass
    try:
        for index in range(model.size()):
            out.append(model.get(index))
    except:
        pass
    return out


def object_name(obj):
    for method in ["getName", "getId"]:
        try:
            value = getattr(obj, method)()
            if value is not None and str(value).strip():
                return str(value)
        except:
            pass
    return str(obj)


def all_heads(machine_obj):
    try:
        return java_items(machine_obj.getHeads())
    except:
        pass
    try:
        head = machine_obj.getDefaultHead()
        return [] if head is None else [head]
    except:
        return []


def unique_sorted(values):
    seen = {}
    for value in values:
        text = str(value or "").strip()
        if text:
            seen[text] = True
    result = seen.keys()
    result.sort()
    return result


def available_nozzle_names():
    names = []
    for head in all_heads(get_machine()):
        try:
            for nozzle in java_items(head.getNozzles()):
                names.append(object_name(nozzle))
        except:
            pass
    return unique_sorted(names)


def available_actuator_names():
    machine_obj = get_machine()
    names = []
    owners = all_heads(machine_obj) + [machine_obj]
    for owner in owners:
        try:
            for actuator in java_items(owner.getActuators()):
                names.append(object_name(actuator))
        except:
            pass
    return unique_sorted(names)


def get_down_camera(machine_obj=None):
    machine_obj = machine_obj or get_machine()
    try:
        head = machine_obj.getDefaultHead()
        camera = head.getDefaultCamera()
        if camera is not None:
            return camera
    except:
        pass
    for head in all_heads(machine_obj):
        try:
            cameras = java_items(head.getCameras())
            if cameras:
                return cameras[0]
        except:
            pass
    try:
        cameras = java_items(machine_obj.getCameras())
        if cameras:
            return cameras[0]
    except:
        pass
    raise Exception("No top/down camera is configured")


def combo_text(combo):
    item = combo.getSelectedItem()
    return "" if item is None else str(item).strip()


def set_combo_items(combo, values, selected):
    current = str(selected or "")
    combo.removeAllItems()
    if current:
        combo.addItem(current)
    for value in values:
        if value != current:
            combo.addItem(value)
    combo.setSelectedItem(current)


class FieldChangeListener(DocumentListener):
    def __init__(self, callback):
        self.callback = callback

    def insertUpdate(self, event):
        self.callback()

    def removeUpdate(self, event):
        self.callback()

    def changedUpdate(self, event):
        self.callback()


class ReadOnlyTableModel(DefaultTableModel):
    def isCellEditable(self, row, column):
        return False


class BedMeshGui(object):
    def __init__(self):
        self.cfg = bed_mesh.load_config(CONFIG_PATH)
        self.frame = JFrame("Voron Tap Bed Mesh")
        self.worker_running = False
        self.abort_requested = AtomicBoolean(False)
        self.loading_ui = False
        self.state = IDLE
        self.last_result = None
        self.saved_snapshot = self.snapshot(self.cfg)
        self.action_buttons = []
        self.selector_combos = {}
        self.probe_combos = {}
        self.machine_rows = {}
        self.motion_rows = {}
        self.motion_fields = {}
        self.debounce_enabled_checkbox = None
        self.panel_fields = []
        self.completed_progress = 0

        self.state_label = JLabel(IDLE)
        self.dirty_label = JLabel("clean")
        self.path_label = JLabel("CSV: --")
        self.status_label = JLabel("Ready")
        self.progress = JProgressBar()
        self.log_area = JTextArea()
        self.json_area = JTextArea()
        self.result_tabs = JTabbedPane()
        self.surface_views = []
        self.surface_scale_model = None

    def snapshot(self, cfg):
        return json.dumps(bed_mesh.public_config(cfg), sort_keys=True,
            separators=(",", ":"))

    def build(self):
        self.frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE)
        self.frame.setMinimumSize(Dimension(1180, 800))
        root = JPanel(BorderLayout(8, 8))
        root.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8))
        root.add(self.build_header(), BorderLayout.NORTH)

        form_scroll = JScrollPane(self.build_form())
        form_scroll.setPreferredSize(Dimension(500, 650))
        split = JSplitPane(JSplitPane.HORIZONTAL_SPLIT, form_scroll, self.build_results())
        split.setResizeWeight(0.40)
        split.setDividerLocation(500)
        root.add(split, BorderLayout.CENTER)
        root.add(self.build_log(), BorderLayout.SOUTH)
        self.frame.add(root)
        self.load_into_ui(self.cfg)
        self.refresh_lists()
        self.set_state(IDLE)
        self.frame.pack()
        self.frame.setLocationRelativeTo(None)
        self.frame.setVisible(True)

    def build_header(self):
        panel = JPanel(BorderLayout(6, 4))
        buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 2))
        specs = [
            ("refresh_button", "Refresh Lists", lambda event: self.refresh_lists()),
            ("run_button", "Run Bed Mesh", lambda event: self.run_action()),
            ("abort_button", "Abort", lambda event: self.abort_action()),
            ("save_button", "Save Config", lambda event: self.save_action()),
        ]
        for name, title, callback in specs:
            button = JButton(title)
            button.addActionListener(callback)
            setattr(self, name, button)
            if name != "abort_button":
                self.action_buttons.append(button)
            buttons.add(button)
        self.abort_button.setBackground(Color(190, 45, 45))
        self.abort_button.setForeground(Color.WHITE)
        self.abort_button.setVisible(False)
        panel.add(buttons, BorderLayout.NORTH)
        status = JPanel(FlowLayout(FlowLayout.LEFT, 12, 2))
        status.add(JLabel("State:"))
        status.add(self.state_label)
        status.add(JLabel("Config:"))
        status.add(self.dirty_label)
        status.add(self.status_label)
        self.progress.setStringPainted(True)
        self.progress.setPreferredSize(Dimension(280, 22))
        status.add(self.progress)
        panel.add(status, BorderLayout.CENTER)
        self.path_label.setBorder(BorderFactory.createEmptyBorder(2, 5, 2, 2))
        panel.add(self.path_label, BorderLayout.SOUTH)
        return panel

    def build_form(self):
        form = JPanel()
        form.setLayout(BoxLayout(form, BoxLayout.Y_AXIS))
        form.add(self.build_machine_section())
        form.add(Box.createVerticalStrut(6))
        form.add(self.build_motion_section())
        for panel_index in range(4):
            form.add(Box.createVerticalStrut(6))
            form.add(self.build_panel_section(panel_index))
        form.add(Box.createVerticalGlue())
        return form

    def section(self, title):
        panel = JPanel(GridBagLayout())
        panel.setBorder(BorderFactory.createTitledBorder(title))
        panel.setAlignmentX(JPanel.LEFT_ALIGNMENT)
        panel.setMaximumSize(Dimension(32767, 32767))
        return panel

    def add_row(self, panel, row, title, component):
        gbc = GridBagConstraints()
        gbc.gridx = 0
        gbc.gridy = row
        gbc.insets = Insets(2, 3, 2, 8)
        gbc.anchor = GridBagConstraints.WEST
        label = JLabel(title)
        label.setPreferredSize(Dimension(210, label.getPreferredSize().height))
        panel.add(label, gbc)
        gbc = GridBagConstraints()
        gbc.gridx = 1
        gbc.gridy = row
        gbc.insets = Insets(2, 2, 2, 3)
        gbc.fill = GridBagConstraints.HORIZONTAL
        gbc.weightx = 1.0
        panel.add(component, gbc)
        return (label, component)

    def watch(self, field):
        field.getDocument().addDocumentListener(FieldChangeListener(self.field_changed))

    def build_machine_section(self):
        panel = self.section("Machine / Output")
        for row, spec in enumerate([
                ("nozzle_name", "Probe-carrying nozzle"),
                ("probe_actuator_name", "Z probe actuator")]):
            key, title = spec
            combo = JComboBox()
            combo.setEditable(True)
            combo.addActionListener(lambda event: self.field_changed())
            self.selector_combos[key] = combo
            self.machine_rows[key] = self.add_row(panel, row, title, combo)
        acquisition = JComboBox()
        acquisition.addItem("polling")
        acquisition.addItem("hardware_interrupt")
        acquisition.addActionListener(lambda event: self.probe_mode_changed())
        self.probe_combos["acquisition_method"] = acquisition
        self.machine_rows["acquisition_method"] = self.add_row(
            panel, 2, "Z touch method", acquisition)
        probe_mode = JComboBox()
        probe_mode.addItem("actuator")
        probe_mode.addItem("direct_gcode")
        probe_mode.addActionListener(lambda event: self.probe_mode_changed())
        self.probe_combos["mode"] = probe_mode
        self.machine_rows["mode"] = self.add_row(panel, 3,
            "Hardware probe mode", probe_mode)
        sequence_actuator = JComboBox()
        sequence_actuator.setEditable(True)
        sequence_actuator.addActionListener(lambda event: self.field_changed())
        self.probe_combos["sequence_actuator_name"] = sequence_actuator
        self.machine_rows["sequence_actuator_name"] = self.add_row(
            panel, 4, "Sequence actuator", sequence_actuator)
        channel = JComboBox()
        channel.addItem("Q0 - Voron")
        channel.addItem("Q1 - Contact")
        channel.addActionListener(lambda event: self.field_changed())
        self.probe_combos["channel"] = channel
        self.machine_rows["channel"] = self.add_row(
            panel, 5, "Direct mux channel", channel)
        output = JTextField(26)
        self.motion_fields["output_directory"] = output
        self.watch(output)
        output_row = JPanel(BorderLayout(4, 0))
        output_row.add(output, BorderLayout.CENTER)
        browse = JButton("Browse")
        browse.addActionListener(lambda event: self.browse_output())
        output_row.add(browse, BorderLayout.EAST)
        self.action_buttons.append(browse)
        self.machine_rows["output_directory"] = self.add_row(
            panel, 6, "Output directory", output_row)
        return panel

    def build_motion_section(self):
        panel = self.section("Motion / Probe")
        specs = [
            ("voron_trigger_offset_mm", "Voron trigger offset mm"),
            ("safe_z_mm", "Absolute safe Z mm"),
            ("travel_speed", "Travel speed"),
            ("probe.start_above_mm", "Start above expected Z mm"),
            ("probe.max_descent_mm", "Maximum probe descent mm"),
            ("probe.retract_after_contact_mm", "Retract after contact mm"),
            ("probe.coarse_speed", "Polling coarse speed"),
            ("probe.fine_speed", "Polling fine speed"),
            ("probe.coarse_step_mm", "Polling coarse step mm"),
            ("probe.fine_step_mm", "Polling fine step mm"),
            ("probe.polling_coarse_repeats", "Polling coarse tries / average"),
            ("probe.polling_fine_repeats", "Polling fine tries / average"),
            ("probe.release_retract_step_mm", "Polling release step mm"),
            ("probe.max_release_retract_mm", "Polling max release mm"),
            ("probe.read_settle_ms", "Polling read settle ms"),
            ("probe.debounce_count", "Polling debounce count"),
            ("probe.debounce_retries", "Polling debounce retries"),
            ("probe.debounce_interval_ms", "Polling debounce interval ms"),
            ("probe.coarse_feed", "Direct coarse F"),
            ("probe.fine_feed", "Direct fine F"),
            ("probe.fine_repeats", "Direct fine repeats / average"),
            ("probe.g38_attempts", "Direct G38 attempts"),
            ("probe.probe_acceleration", "Direct M204 S"),
        ]
        for row, spec in enumerate(specs):
            key, title = spec
            field = JTextField(12)
            self.motion_fields[key] = field
            self.watch(field)
            self.motion_rows[key] = self.add_row(panel, row, title, field)
        self.debounce_enabled_checkbox = JCheckBox("Require stable repeated reads")
        self.debounce_enabled_checkbox.addActionListener(
            lambda event: self.debounce_toggled())
        self.motion_rows["probe.debounce_enabled"] = self.add_row(
            panel, len(specs), "Polling debounce enabled",
            self.debounce_enabled_checkbox)
        return panel

    def build_panel_section(self, panel_index):
        panel = self.section("Panel %d" % (panel_index + 1))
        fields = {}
        enabled = JCheckBox("Include this panel in the run")
        enabled.addActionListener(lambda event: self.field_changed())
        fields["enabled"] = enabled
        self.add_row(panel, 0, "Enabled", enabled)
        specs = [
            ("name", "Panel name"),
            ("lower_left_x_mm", "Lower-left X mm"),
            ("lower_left_y_mm", "Lower-left Y mm"),
            ("expected_z_mm", "Expected surface Z mm"),
            ("width_mm", "Width +X mm"),
            ("height_mm", "Height +Y mm"),
            ("columns", "X columns"),
            ("rows", "Y rows"),
        ]
        for row, spec in enumerate(specs, 1):
            key, title = spec
            field = JTextField(12)
            self.watch(field)
            fields[key] = field
            self.add_row(panel, row, title, field)
        capture_buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 0))
        capture_camera = JButton("Capture Camera DRO")
        capture_dispense = JButton("Capture Dispense DRO")
        capture_camera.addActionListener(
            lambda event, index=panel_index: self.capture_panel_corner(index, "camera"))
        capture_dispense.addActionListener(
            lambda event, index=panel_index: self.capture_panel_corner(index, "dispense"))
        capture_buttons.add(capture_camera)
        capture_buttons.add(capture_dispense)
        self.action_buttons.extend([capture_camera, capture_dispense])
        self.add_row(panel, len(specs) + 1, "Set lower-left corner", capture_buttons)
        self.panel_fields.append(fields)
        return panel

    def build_results(self):
        panel = JPanel(BorderLayout(5, 5))
        self.result_tabs.setBorder(BorderFactory.createTitledBorder("Mesh Results"))
        self.json_area.setEditable(False)
        self.json_area.setRows(8)
        json_scroll = JScrollPane(self.json_area)
        json_scroll.setBorder(BorderFactory.createTitledBorder("Full Result JSON"))
        split = JSplitPane(JSplitPane.VERTICAL_SPLIT, self.result_tabs, json_scroll)
        split.setResizeWeight(0.72)
        split.setDividerLocation(500)
        panel.add(split, BorderLayout.CENTER)
        return panel

    def build_log(self):
        panel = JPanel(BorderLayout())
        panel.setBorder(BorderFactory.createTitledBorder("Run Log"))
        self.log_area.setEditable(False)
        self.log_area.setRows(7)
        panel.add(JScrollPane(self.log_area), BorderLayout.CENTER)
        return panel

    def get_nested(self, data, dotted):
        current = data
        for part in dotted.split("."):
            if not isinstance(current, dict):
                return None
            current = current.get(part)
        return current

    def set_nested(self, data, dotted, value):
        parts = dotted.split(".")
        current = data
        for part in parts[:-1]:
            if part not in current or not isinstance(current.get(part), dict):
                current[part] = {}
            current = current[part]
        current[parts[-1]] = value

    def parse_number(self, text, label):
        try:
            return float(str(text).strip())
        except:
            raise bed_mesh.BedMeshError("%s must be a number" % label)

    def load_into_ui(self, cfg):
        self.loading_ui = True
        try:
            for key in self.selector_combos:
                self.selector_combos[key].setSelectedItem(str(cfg.get(key, "")))
            for key in self.motion_fields:
                value = self.get_nested(cfg, key)
                self.motion_fields[key].setText("" if value is None else str(value))
            probe = cfg.get("probe", {})
            self.probe_combos["acquisition_method"].setSelectedItem(str(
                probe.get("acquisition_method", "hardware_interrupt")))
            self.probe_combos["mode"].setSelectedItem(str(
                probe.get("mode", "actuator")))
            self.probe_combos["sequence_actuator_name"].setSelectedItem(str(
                probe.get("sequence_actuator_name", "Run Voron Probe")))
            channel = str(probe.get("channel", "Q0") or "Q0").upper()
            self.probe_combos["channel"].setSelectedItem(
                "Q1 - Contact" if channel == "Q1" else "Q0 - Voron")
            self.debounce_enabled_checkbox.setSelected(bool(
                probe.get("debounce_enabled", True)))
            panels = cfg.get("panels", [])
            for index, fields in enumerate(self.panel_fields):
                source = panels[index]
                fields["enabled"].setSelected(bool(source.get("enabled", False)))
                for key in fields:
                    if key != "enabled":
                        fields[key].setText(str(source.get(key, "")))
        finally:
            self.loading_ui = False
        self.update_probe_visibility()
        self.update_dirty()

    def collect_config(self, validate=True):
        cfg = bed_mesh.normalize_config(self.cfg)
        cfg["nozzle_name"] = combo_text(self.selector_combos["nozzle_name"])
        cfg["probe_actuator_name"] = combo_text(
            self.selector_combos["probe_actuator_name"])
        cfg["probe"]["acquisition_method"] = str(
            self.probe_combos["acquisition_method"].getSelectedItem()).strip()
        cfg["probe"]["mode"] = str(
            self.probe_combos["mode"].getSelectedItem()).strip()
        cfg["probe"]["sequence_actuator_name"] = combo_text(
            self.probe_combos["sequence_actuator_name"])
        channel_text = str(self.probe_combos["channel"].getSelectedItem() or "")
        cfg["probe"]["channel"] = (
            "Q1" if channel_text.upper().startswith("Q1") else "Q0")
        cfg["probe"]["debounce_enabled"] = (
            self.debounce_enabled_checkbox.isSelected())
        for key in self.motion_fields:
            if key == "output_directory":
                value = str(self.motion_fields[key].getText()).strip()
            else:
                try:
                    value = self.parse_number(self.motion_fields[key].getText(), key)
                except:
                    row = self.motion_rows.get(key)
                    active = row is None or (
                        row[1].isVisible() and row[1].isEnabled())
                    if active:
                        raise
                    # Hidden inactive settings do not block saving or running.
                    value = self.get_nested(cfg, key)
            self.set_nested(cfg, key, value)
        cfg["panels"] = []
        for index, fields in enumerate(self.panel_fields):
            panel = {"enabled": fields["enabled"].isSelected()}
            for key in fields:
                if key == "enabled":
                    continue
                if key == "name":
                    panel[key] = str(fields[key].getText()).strip()
                else:
                    panel[key] = self.parse_number(
                        fields[key].getText(), "Panel %d %s" % (index + 1, key))
            cfg["panels"].append(panel)
        cfg["_config_path"] = CONFIG_PATH
        if validate:
            bed_mesh.validate_config(cfg, True)
            if not str(cfg.get("output_directory", "")).strip():
                raise bed_mesh.BedMeshError("Output directory is required")
        return cfg

    def field_changed(self):
        if not self.loading_ui:
            self.update_dirty()

    def set_row_visible(self, row, visible):
        if row is None:
            return
        for component in row:
            component.setVisible(bool(visible))

    def probe_mode_changed(self):
        self.update_probe_visibility()
        self.field_changed()

    def update_probe_visibility(self):
        acquisition = str(self.probe_combos["acquisition_method"].getSelectedItem()
            or "hardware_interrupt")
        mode = str(self.probe_combos["mode"].getSelectedItem() or "actuator")
        hardware = acquisition == "hardware_interrupt"
        direct = hardware and mode == "direct_gcode"
        self.set_row_visible(self.machine_rows.get("mode"), hardware)
        self.set_row_visible(self.machine_rows.get("sequence_actuator_name"),
            hardware and mode == "actuator")
        self.set_row_visible(self.machine_rows.get("channel"), direct)
        polling_keys = [
            "probe.coarse_speed", "probe.fine_speed", "probe.coarse_step_mm",
            "probe.fine_step_mm", "probe.polling_coarse_repeats",
            "probe.polling_fine_repeats", "probe.release_retract_step_mm",
            "probe.max_release_retract_mm", "probe.read_settle_ms",
            "probe.debounce_count", "probe.debounce_retries",
            "probe.debounce_interval_ms", "probe.debounce_enabled"]
        direct_keys = ["probe.coarse_feed", "probe.fine_feed",
            "probe.fine_repeats", "probe.g38_attempts",
            "probe.probe_acceleration"]
        for key in polling_keys:
            self.set_row_visible(self.motion_rows.get(key), not hardware)
        for key in direct_keys:
            self.set_row_visible(self.motion_rows.get(key), direct)
        for key in ["probe.max_descent_mm", "probe.retract_after_contact_mm"]:
            self.set_row_visible(self.motion_rows.get(key), (not hardware) or direct)
        self.update_debounce_field_state()
        try:
            self.frame.getContentPane().revalidate()
            self.frame.getContentPane().repaint()
        except:
            pass

    def debounce_toggled(self):
        self.update_debounce_field_state()
        self.field_changed()

    def update_debounce_field_state(self):
        polling = str(self.probe_combos["acquisition_method"].getSelectedItem()
            or "hardware_interrupt") == "polling"
        enabled = polling and self.debounce_enabled_checkbox.isSelected()
        for key in ["probe.debounce_count", "probe.debounce_retries",
                "probe.debounce_interval_ms"]:
            self.motion_fields[key].setEnabled(enabled)

    def update_dirty(self):
        try:
            dirty = self.snapshot(self.collect_config(False)) != self.saved_snapshot
        except:
            dirty = True
        self.dirty_label.setText("DIRTY" if dirty else "clean")

    def refresh_lists(self):
        try:
            set_combo_items(self.selector_combos["nozzle_name"],
                available_nozzle_names(), combo_text(self.selector_combos["nozzle_name"]))
            set_combo_items(self.selector_combos["probe_actuator_name"],
                available_actuator_names(),
                combo_text(self.selector_combos["probe_actuator_name"]))
            set_combo_items(self.probe_combos["sequence_actuator_name"],
                available_actuator_names(),
                combo_text(self.probe_combos["sequence_actuator_name"]))
            self.append_log("Refreshed configured nozzles and actuators")
        except Exception as error:
            self.show_error("Refresh Lists failed", error)

    def browse_output(self):
        chooser = JFileChooser(self.motion_fields["output_directory"].getText())
        chooser.setDialogTitle("Choose bed mesh output directory")
        chooser.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY)
        if chooser.showOpenDialog(self.frame) == JFileChooser.APPROVE_OPTION:
            self.motion_fields["output_directory"].setText(
                chooser.getSelectedFile().getAbsolutePath())

    def capture_panel_corner(self, panel_index, source):
        try:
            machine_obj = get_machine()
            if source == "camera":
                movable = get_down_camera(machine_obj)
                source_label = "camera"
            else:
                nozzle_name = combo_text(self.selector_combos["nozzle_name"])
                if not nozzle_name:
                    raise bed_mesh.BedMeshError("Select the dispense nozzle first")
                movable = bed_mesh.get_needle_backend().find_nozzle(
                    {"head_name": "", "nozzle_name": nozzle_name},
                    None, machine_obj)
                source_label = "dispense nozzle"
            loc = movable.getLocation().convertToUnits(LengthUnit.Millimeters)
            fields = self.panel_fields[panel_index]
            fields["lower_left_x_mm"].setText("%.4f" % loc.getX())
            fields["lower_left_y_mm"].setText("%.4f" % loc.getY())
            fields["expected_z_mm"].setText("%.4f" % loc.getZ())
            self.append_log(
                "Captured Panel %d lower-left XYZ from %s DRO: "
                "X %.4f Y %.4f Z %.4f" % (
                    panel_index + 1, source_label,
                    loc.getX(), loc.getY(), loc.getZ()))
            self.update_dirty()
        except Exception as error:
            self.set_state(ERROR)
            self.show_error("Capture Panel %d DRO failed" % (panel_index + 1), error)
        except Throwable as error:
            self.set_state(ERROR)
            self.show_error("Capture Panel %d DRO failed" % (panel_index + 1), error)

    def set_state(self, state):
        self.state = state
        self.state_label.setText(state)
        for button in self.action_buttons:
            button.setEnabled(not self.worker_running)
        self.abort_button.setEnabled(
            self.worker_running and not self.abort_requested.get())

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

    def calibration_log(self, message):
        if self.abort_requested.get():
            raise CalibrationAborted("Bed mesh aborted by operator")
        text = str(message)
        if ": offset " in text:
            self.completed_progress += 1
            completed = self.completed_progress
            gui_self = self
            class ProgressTask(Runnable):
                def run(task_self):
                    gui_self.progress.setValue(completed)
                    gui_self.progress.setString("%d / %d points" %
                        (completed, gui_self.progress.getMaximum()))
            SwingUtilities.invokeLater(ProgressTask())
        self.append_log(text)

    def show_error(self, title, error):
        self.append_log("%s: %s" % (title, error))
        JOptionPane.showMessageDialog(self.frame, str(error), title,
            JOptionPane.ERROR_MESSAGE)

    def save_action(self):
        try:
            cfg = self.collect_config(True)
            bed_mesh.save_config(cfg, CONFIG_PATH)
            self.cfg = cfg
            self.saved_snapshot = self.snapshot(cfg)
            self.update_dirty()
            self.set_state(SAVED)
            self.append_log("Saved config: %s" % CONFIG_PATH)
        except Exception as error:
            self.set_state(ERROR)
            self.show_error("Save Config failed", error)

    def abort_action(self):
        if self.worker_running and self.abort_requested.compareAndSet(False, True):
            self.abort_button.setEnabled(False)
            self.abort_button.setText("Abort Requested...")
            self.status_label.setText("Stopping at next probe checkpoint")
            self.append_log("Abort requested; safe-Z cleanup will follow")

    def run_action(self):
        try:
            cfg = self.collect_config(True)
            total = 0
            for index, panel in enumerate(cfg.get("panels", [])):
                if panel.get("enabled", False):
                    total += len(bed_mesh.generate_grid(panel, index + 1))
        except Exception as error:
            self.set_state(ERROR)
            self.show_error("Bed Mesh setup failed", error)
            return
        self.abort_requested.set(False)
        self.abort_button.setText("Abort")
        self.abort_button.setVisible(True)
        self.completed_progress = 0
        self.progress.setMinimum(0)
        self.progress.setMaximum(total)
        self.progress.setValue(0)
        self.progress.setString("0 / %d points" % total)
        self.path_label.setText("CSV: --")
        self.status_label.setText("Probing bed mesh")
        self.worker_running = True
        self.set_state(RUNNING)
        self.append_log("Starting bed mesh with %d points" % total)
        self.run_worker(cfg)

    def run_worker(self, cfg):
        gui_self = self
        class Worker(SwingWorker):
            def doInBackground(worker_self):
                machine_obj = get_machine()
                if not hasattr(machine_obj, "execute"):
                    raise bed_mesh.BedMeshError(
                        "OpenPnP machine task execution is not available")
                class Task(Callable):
                    def call(task_self):
                        result = None
                        try:
                            result = bed_mesh.run_mesh(
                                cfg, None, machine_obj, gui_self.calibration_log)
                            if gui_self.abort_requested.get():
                                result["ok"] = False
                                result["aborted"] = True
                                result["message"] = "Bed mesh aborted by operator"
                            if result.get("ok", False):
                                try:
                                    bed_mesh.export_csv(
                                        result, cfg.get("output_directory"))
                                except Exception as export_error:
                                    result["ok"] = False
                                    result["message"] = "CSV export failed: %s" % export_error
                            return result
                        finally:
                            # Do not alter OpenPnP's global machine speed
                            # override during or after calibration.
                            pass
                return machine_obj.execute(Task())

            def done(worker_self):
                gui_self.worker_running = False
                gui_self.abort_button.setVisible(False)
                try:
                    gui_self.run_done(worker_self.get())
                except Exception as error:
                    gui_self.set_state(ERROR)
                    gui_self.status_label.setText("Run failed")
                    gui_self.show_error("Bed Mesh failed", error)
                except Throwable as error:
                    gui_self.set_state(ERROR)
                    gui_self.status_label.setText("Run failed")
                    gui_self.show_error("Bed Mesh failed", error)
                gui_self.set_state(gui_self.state)
        Worker().execute()

    def run_done(self, result):
        self.last_result = result
        self.display_result(result)
        if result is not None and result.get("ok", False):
            self.progress.setValue(self.progress.getMaximum())
            self.progress.setString("Complete: %d points" %
                int(result.get("completed_points", 0)))
            self.path_label.setText("CSV: %s" % result.get("csv_path"))
            self.status_label.setText("Mesh complete")
            self.set_state(READY)
            self.append_log("Saved mesh CSV: %s" % result.get("csv_path"))
        elif result is not None and result.get("aborted", False):
            self.progress.setString("Aborted: %d / %d points" %
                (int(result.get("completed_points", 0)),
                 int(result.get("total_points", 0))))
            self.status_label.setText("Aborted; partial values shown, no CSV written")
            self.set_state(IDLE)
        else:
            message = "Unknown failure" if result is None else result.get("message")
            self.status_label.setText("Failed: %s" % message)
            self.set_state(ERROR)
            self.append_log("Bed mesh failed: %s" % message)

    def display_result(self, result):
        self.result_tabs.removeAll()
        self.surface_views = []
        self.surface_scale_model = bed_mesh_surface.new_scale_model()
        if result is None:
            self.json_area.setText("No result")
            return
        self.json_area.setText(json.dumps(result, sort_keys=True, indent=2))
        self.json_area.setCaretPosition(0)
        panel_results = list(result.get("panels", []))
        shared_metrics = bed_mesh_surface.compute_display_metrics(panel_results)
        if panel_results:
            self.add_overview_result(panel_results, result.get("ok", False),
                result.get("message", ""), shared_metrics)
        for panel_result in panel_results:
            self.add_panel_result(panel_result, result.get("ok", False),
                result.get("message", ""), shared_metrics)
        if self.result_tabs.getTabCount() == 0:
            empty = JLabel("No completed panel points")
            empty.setBorder(BorderFactory.createEmptyBorder(20, 20, 20, 20))
            self.result_tabs.addTab("No Results", empty)

    def add_overview_result(self, panel_results, run_ok, run_message,
            shared_metrics):
        root = JPanel(BorderLayout(5, 5))
        point_count = sum([len(panel.get("points", []))
            for panel in panel_results])
        header = JPanel(BorderLayout())
        summary = JLabel(
            "%d panels    %d measured points    Shared color limits: +/- %.6f mm" %
            (len(panel_results), point_count,
             float(shared_metrics.get("max_abs_offset", 0.0))))
        summary.setFont(summary.getFont().deriveFont(Font.BOLD))
        summary.setBorder(BorderFactory.createEmptyBorder(4, 4, 4, 4))
        if not run_ok:
            warning = JLabel("INCOMPLETE MESH - %s" % str(run_message))
            warning.setOpaque(True)
            warning.setBackground(Color(150, 35, 35))
            warning.setForeground(Color.WHITE)
            warning.setFont(warning.getFont().deriveFont(Font.BOLD, 14.0))
            warning.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8))
            header.add(warning, BorderLayout.NORTH)
        header.add(summary, BorderLayout.SOUTH)
        root.add(header, BorderLayout.NORTH)
        root.add(self.build_surface_component(panel_results, shared_metrics),
            BorderLayout.CENTER)
        self.result_tabs.addTab("Overview", root)

    def build_surface_component(self, panel_results, shared_metrics):
        try:
            surface = bed_mesh_surface.BedMeshSurfaceView(
                panel_results, shared_metrics, self.surface_scale_model)
            self.surface_views.append(surface)
            return surface.get_component()
        except Exception as error:
            message = "3D visualization unavailable: %s" % str(error)
        except Throwable as error:
            message = "3D visualization unavailable: %s" % str(error)
        self.append_log(message)
        fallback = JLabel(
            "<html><center><b>%s</b><br>Mesh measurements and table data are "
            "still valid.</center></html>" % message)
        fallback.setOpaque(True)
        fallback.setBackground(Color(90, 48, 30))
        fallback.setForeground(Color.WHITE)
        fallback.setHorizontalAlignment(SwingConstants.CENTER)
        fallback.setBorder(BorderFactory.createEmptyBorder(20, 20, 20, 20))
        return fallback

    def add_panel_result(self, panel_result, run_ok, run_message,
            shared_metrics):
        root = JPanel(BorderLayout(5, 5))
        summary = panel_result.get("summary", {})
        config = panel_result.get("config", {})
        expected_points = int(config.get("rows", 0)) * int(config.get("columns", 0))
        completed_points = int(summary.get("point_count", 0))
        panel_complete = bool(run_ok) and completed_points == expected_points
        if summary.get("point_count", 0):
            summary_text = (
                "Points: %d    Min: %+.6f mm    Max: %+.6f mm    "
                "Mean: %+.6f mm    Peak-to-peak: %.6f mm" % (
                    int(summary.get("point_count")),
                    float(summary.get("min_offset_mm")),
                    float(summary.get("max_offset_mm")),
                    float(summary.get("mean_offset_mm")),
                    float(summary.get("peak_to_peak_mm"))))
        else:
            summary_text = "No completed points"
        summary_label = JLabel(summary_text)
        summary_label.setFont(summary_label.getFont().deriveFont(Font.BOLD))
        summary_label.setBorder(BorderFactory.createEmptyBorder(4, 4, 4, 4))
        header_panel = JPanel(BorderLayout())
        if not panel_complete:
            failure = JLabel(
                "INCOMPLETE MESH - %d of %d points. No CSV exported. %s" %
                (completed_points, expected_points, str(run_message)))
            failure.setOpaque(True)
            failure.setBackground(Color(150, 35, 35))
            failure.setForeground(Color.WHITE)
            failure.setFont(failure.getFont().deriveFont(Font.BOLD, 14.0))
            failure.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8))
            header_panel.add(failure, BorderLayout.NORTH)
        header_panel.add(summary_label, BorderLayout.SOUTH)
        root.add(header_panel, BorderLayout.NORTH)

        surface_component = self.build_surface_component(
            [panel_result], shared_metrics)
        model = ReadOnlyTableModel()
        columns = ["Row", "Column", "X mm", "Y mm", "Corrected Z mm",
            "Offset mm", "Fine spread mm"]
        model.setColumnIdentifiers(columns)
        points = list(panel_result.get("points", []))
        points.sort(key=lambda point: (int(point.get("row")), int(point.get("column"))))
        for point in points:
            model.addRow([
                int(point.get("row")), int(point.get("column")),
                "%.6f" % float(point.get("x_mm")),
                "%.6f" % float(point.get("y_mm")),
                "%.6f" % float(point.get("corrected_surface_z_mm")),
                "%+.6f" % float(point.get("z_offset_mm")),
                "%.6f" % float(point.get("fine_spread_mm")),
            ])
        table = JTable(model)
        table.setAutoCreateRowSorter(True)
        split = JSplitPane(JSplitPane.VERTICAL_SPLIT,
            surface_component, JScrollPane(table))
        split.setResizeWeight(0.65)
        split.setDividerLocation(420)
        root.add(split, BorderLayout.CENTER)
        tab_name = str(panel_result.get("panel_name"))
        if not panel_complete:
            tab_name = "INCOMPLETE - " + tab_name
        self.result_tabs.addTab(tab_name, root)


def start():
    gui = BedMeshGui()
    gui.build()
    return gui


class StartRunnable(Runnable):
    def run(self):
        start()


try:
    SwingUtilities.invokeLater(StartRunnable())
except:
    start()
